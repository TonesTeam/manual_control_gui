//! The CAN adapter, and the two boards that hang off it.
//!
//! Both the Peltier slot-temperature controller and the optical liquid-sensor
//! board sit on one WeAct USB2CAN adapter. A serial port opens exclusively, so
//! they cannot each open it: the adapter is opened and SLCAN-initialised once
//! here, and the handle is shared. Frames for one board are parked on the
//! shared queue rather than dropped, so neither steals the other's answers.
//!
//! One worker drives both, in one thread. Temperature commands block until the
//! board acknowledges them, so they must not sit behind anything slow — but
//! they must also not interleave with a sensor request on the same adapter,
//! which a second thread would let them do.

use std::sync::mpsc::{Receiver, RecvTimeoutError, Sender, channel};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

use slot_temp_sensor_can::can::slcan::{SlcanInterface, open_shared};
use slot_temp_sensor_can::can::{CanInterface, SharedPort};
use slot_temp_sensor_can::{SlotTempController, SlotTempError};

use crate::bus::{BusCmd, BusState, Level, LogLine, StopFired, Wake, next_log_seq};
use crate::detect::{Detector, Tuning};
use crate::rig::Rig;
use crate::sensors::{self, SensorState};
use crate::temperature::{TempOp, TempState};

/// How often the boards are asked for their state.
///
/// Slower than the RS485 poll on purpose: a slot's temperature moves over
/// seconds, and every transaction here blocks until the board answers.
const POLL: Duration = Duration::from_millis(1000);
/// How often the raw ADC is read between status polls.
///
/// A front crossing a detector at 8 rpm is past it in a fraction of a second,
/// so sampling with the temperature status at 1 Hz turns the whole transition
/// into one point. Detecting froth means seeing the shape of it.
///
/// This is a floor, not a promise: each read is a request and a reply on an
/// adapter shared with the temperature exchanges, and asking faster than the
/// board can answer only produces timeouts and reconnects. The recording that
/// the detection work is tuned against came in at about 20 Hz.
const ADC_POLL: Duration = Duration::from_millis(45);
/// A reply to a raw read either comes quickly or not at all; waiting the full
/// sensor timeout for one only delays the next sample.
const ADC_TIMEOUT: Duration = Duration::from_millis(120);
/// At most one logged change per detector per this long. An unthrottled
/// flickering detector produces twenty lines a second, copied to every client,
/// which fills their queues until the server drops them for falling behind.
const CHANGE_LOG_EVERY: Duration = Duration::from_secs(1);
/// Wait before retrying an adapter that would not open.
const RETRY: Duration = Duration::from_secs(5);
const HISTORY_SECS: f64 = 600.0;
/// How long to wait for a sensor board that may not be fitted.
const SENSOR_TIMEOUT: Duration = Duration::from_millis(300);

/// What the worker was asked to drive.
pub struct Config {
    pub port: String,
    /// Where to send a stop when a watched detector trips. This thread sends
    /// it directly rather than reporting a verdict for something else to act
    /// on: the pump is moving while anything else decides.
    pub commands: Option<Sender<BusCmd>>,
    /// Read the optical sensor board as well as the temperature board.
    pub sensors: bool,
    /// CAN id the sensor board answers on.
    pub sensor_device_id: u16,
}

/// Starts the worker. It runs until `rx` is dropped.
pub fn spawn(state: Arc<Mutex<BusState>>, wake: Wake, config: Config) -> Sender<TempOp> {
    let (tx, rx) = channel();
    thread::Builder::new()
        .name("can".into())
        .spawn(move || {
            let detectors = (0..crate::sensors::CHANNELS).map(|_| Detector::new(Tuning::default())).collect();
            Worker {
                state,
                wake,
                config,
                detectors,
                last_change_log: [None; crate::sensors::CHANNELS],
                swallowed: [0; crate::sensors::CHANNELS],
            }
            .run(rx)
        })
        .expect("spawn CAN worker");
    tx
}

struct Worker {
    state: Arc<Mutex<BusState>>,
    wake: Wake,
    config: Config,
    /// One filter and window per board channel, fed at the ADC rate.
    detectors: Vec<Detector>,
    /// When each channel's change was last written to the log, and how many
    /// have been swallowed since.
    last_change_log: [Option<Instant>; crate::sensors::CHANNELS],
    swallowed: [u32; crate::sensors::CHANNELS],
}

impl Worker {
    fn log(&self, level: Level, text: impl Into<String>) {
        let mut s = self.state.lock().unwrap();
        let t = s.now();
        s.push_log(LogLine { t, level, text: text.into(), seq: next_log_seq() });
    }

    fn update(&self, f: impl FnOnce(&mut TempState)) {
        let mut s = self.state.lock().unwrap();
        f(&mut s.temp);
        drop(s);
        self.wake.call();
    }

    fn update_sensors(&self, f: impl FnOnce(&mut SensorState)) {
        let mut s = self.state.lock().unwrap();
        f(&mut s.sensors);
        drop(s);
        self.wake.call();
    }

    fn describe_port(&self) -> String {
        if self.config.port.trim().is_empty() {
            "the CAN adapter".into()
        } else {
            self.config.port.trim().into()
        }
    }

    fn run(mut self, rx: Receiver<TempOp>) {
        self.update(|t| *t = TempState { enabled: true, ..Default::default() });
        self.update_sensors(|s| *s = SensorState { enabled: self.config.sensors, ..Default::default() });
        loop {
            match self.session(&rx) {
                Ok(()) => return, // the channel closed: this backend is done
                Err(e) => {
                    self.update(|t| t.go_offline(Some(e.clone())));
                    self.update_sensors(|s| s.go_offline(Some(e.clone())));
                    self.log(Level::Warn, format!("CAN: {e}"));
                }
            }
            let deadline = Instant::now() + RETRY;
            loop {
                let wait = deadline.saturating_duration_since(Instant::now());
                match rx.recv_timeout(wait) {
                    Ok(op) => self.log(
                        Level::Warn,
                        format!("temperature: {} dropped, no board connected", op.label()),
                    ),
                    Err(RecvTimeoutError::Timeout) => break,
                    Err(RecvTimeoutError::Disconnected) => return,
                }
            }
        }
    }

    /// One connection to the adapter.
    fn session(&mut self, rx: &Receiver<TempOp>) -> Result<(), String> {
        let port = if self.config.port.trim().is_empty() {
            // The crate globs for a WeAct adapter when given nothing.
            default_port()?
        } else {
            self.config.port.trim().to_string()
        };
        // Open and SLCAN-initialise once (S8 + O), then share.
        let bus: SharedPort =
            open_shared(&port).map_err(|e| format!("cannot open {}: {e}", self.describe_port()))?;

        let mut ctl = SlotTempController::connect_shared(bus.clone())
            .map_err(|e| format!("no temperature board on {}: {e}", self.describe_port()))?;
        self.log(Level::Info, format!("temperature board on {}", self.describe_port()));
        self.update(|t| {
            t.connected = true;
            t.error = None;
        });

        // The sensor board answers on its own id, so the shared queue hands
        // each library only its own frames — provided that id is really the
        // sensor board's. An id inside the Peltier's range would make every
        // sensor poll a malformed command to the heater, so it is refused
        // here rather than merely failing to read.
        let sensor_id = self.config.sensor_device_id;
        let mut sensors = if !self.config.sensors {
            None
        } else if sensors::collides_with_temperature_board(sensor_id) {
            self.sensor_down(format!(
                "CAN id 0x{sensor_id:03X} belongs to the temperature board; sensors not read"
            ));
            None
        } else {
            Some(SlcanInterface::from_shared(bus.clone(), (sensor_id, sensor_id)))
        };

        let mut next_poll = Instant::now();
        let mut next_status = Instant::now();
        loop {
            let wait = next_poll.saturating_duration_since(Instant::now());
            match rx.recv_timeout(wait) {
                Ok(op) => {
                    self.execute(&mut ctl, op)?;
                    self.poll_temperature(&mut ctl)?;
                    next_status = Instant::now() + POLL;
                    next_poll = Instant::now();
                }
                Err(RecvTimeoutError::Timeout) => {
                    if Instant::now() >= next_status {
                        self.poll_temperature(&mut ctl)?;
                        if let Some(sensors) = sensors.as_mut() {
                            self.poll_sensors(sensors);
                        }
                        next_status = Instant::now() + POLL;
                    }
                    if let Some(sensors) = sensors.as_mut() {
                        // Thresholds are configuration and do not move, so they
                        // are read once rather than every cycle.
                        let first = self.state.lock().unwrap().sensors.max_threshold.is_none();
                        self.poll_adc(sensors, first);
                    }
                    next_poll = Instant::now() + if sensors.is_some() { ADC_POLL } else { POLL };
                }
                Err(RecvTimeoutError::Disconnected) => {
                    let _ = ctl.disconnect();
                    return Ok(());
                }
            }
        }
    }

    /// One round trip for the reading, the setpoint and the fault words.
    ///
    /// `GET_ALL` rather than `REQUEST_STATUS` because the setpoint has to come
    /// from the config frame: the status frame's `error_c` is an int8 ×10 that
    /// saturates at ±12.7 °C, so deriving the target from it would read wrong
    /// during exactly the long ramp where the target matters most.
    fn poll_temperature(&self, ctl: &mut SlotTempController) -> Result<(), String> {
        let all = ctl.get_all().map_err(|e| format!("status: {e}"))?;
        let status = all.status;
        let now = self.state.lock().unwrap().now();
        let temp = status.temperature.temperature_c;
        self.update(|t| {
            t.connected = true;
            t.error = None;
            t.temperature_c = Some(temp);
            t.duty_pct = Some(status.electrical.duty_pct);
            t.current_a = Some(status.electrical.current_a);
            t.pid_running = status.pid_state.pid_running;
            t.reached = status.pid_state.temp_reached;
            t.sensor_fault = status.pid_state.sensor_fault;
            t.autotuning = status.pid_state.autotuning;
            t.setpoint_c = Some(all.config.setpoint.setpoint_c);
            t.tolerance_c = Some(all.config.setpoint.tolerance_c);
            t.active_word = all.errors.active;
            t.latched_word = all.errors.latched;
            // `active_text` says "none" for a clean board; store nothing
            // instead, so "is there a fault" is a single obvious check.
            t.active_faults = if all.errors.active == 0 { String::new() } else { all.errors.active_text() };
            t.latched_faults = if all.errors.latched == 0 { String::new() } else { all.errors.latched_text() };
            t.last_seen = Some(now);
            let target = t.setpoint_c;
            t.push_sample(now, temp, target, HISTORY_SECS);
        });
        Ok(())
    }

    /// Asks the board for one thing and waits for the matching reply.
    ///
    /// Returns the reply's frame, skipping our own echo and anything else on
    /// the board's id.
    fn ask(&mut self, can: &mut SlcanInterface, frame: [u8; 8], op: u8) -> Option<[u8; 8]> {
        let id = self.config.sensor_device_id;
        if can.send_frame(id, &frame).is_err() {
            return None;
        }
        let deadline = Instant::now() + ADC_TIMEOUT;
        while Instant::now() < deadline {
            let Ok(reply) = can.recv_frame(ADC_TIMEOUT) else { return None };
            if reply.id != id {
                continue;
            }
            if reply.data[0] & 0x01 != 0 && reply.data[0] >> 1 == op {
                return Some(reply.data);
            }
            // A change notification arriving mid-exchange is still news, but
            // a detector with a bubble sitting on it changes tens of times a
            // second, and a line each would be the loudest thing on the bus.
            if let Some(change) = sensors::parse_change(&reply.data) {
                self.log_change(change.channel, change.liquid);
            }
        }
        None
    }

    /// Reads the raw ADC behind the states, and the thresholds they switch on.
    ///
    /// The states alone cannot tell a solid column from froth: both read wet.
    /// The raw value can — a column sits well clear of the switching band and
    /// froth wanders through it.
    fn poll_adc(&mut self, can: &mut SlcanInterface, want_thresholds: bool) {
        if let Some(frame) = self.ask(can, sensors::request_adc(), sensors::OP_RAW_ADC)
            && let Some(adc) = sensors::parse_adc(&frame)
        {
            self.update_sensors(|s| s.adc = Some(adc));
            self.watch(&adc);
        }
        if !want_thresholds {
            return;
        }
        for (op, set) in [
            (sensors::OP_MAX_THRESHOLD, true),
            (sensors::OP_MIN_THRESHOLD, false),
        ] {
            if let Some(frame) = self.ask(can, sensors::request(op), op)
                && let Some(v) = sensors::parse_threshold(&frame, op)
            {
                self.update_sensors(|s| if set { s.max_threshold = Some(v) } else { s.min_threshold = Some(v) });
            }
        }
    }

    /// Asks the sensor board for all six channels.
    ///
    /// A silent sensor board does not end the session: the temperature board is
    /// the one holding a slot, and losing the detectors must not take the
    /// heater's readout down with it.
    fn poll_sensors(&self, can: &mut SlcanInterface) {
        let id = self.config.sensor_device_id;
        if let Err(e) = can.send_frame(id, &sensors::request_states()) {
            self.sensor_down(format!("request failed: {e}"));
            return;
        }
        // Read past our own echo and anything else on the board's id.
        let deadline = Instant::now() + SENSOR_TIMEOUT;
        while Instant::now() < deadline {
            let Ok(frame) = can.recv_frame(SENSOR_TIMEOUT) else { break };
            if frame.id != id {
                continue;
            }
            if let Some(change) = sensors::parse_change(&frame.data) {
                // Unsolicited; note it and keep waiting for the answer.
                self.log(
                    Level::Info,
                    format!(
                        "sensor channel {} -> {}",
                        change.channel,
                        if change.liquid { "liquid" } else { "dry" }
                    ),
                );
                continue;
            }
            if let Some(states) = sensors::parse_states(&frame.data) {
                let now = self.state.lock().unwrap().now();
                let first = !self.state.lock().unwrap().sensors.connected;
                self.update_sensors(|s| {
                    s.connected = true;
                    s.error = None;
                    s.liquid = Some(states);
                    s.last_seen = Some(now);
                });
                if first {
                    self.log(Level::Info, format!("optical sensor board on CAN id 0x{id:03X}"));
                }
                return;
            }
        }
        self.sensor_down("no reply".to_string());
    }

    /// Feeds the detectors and fires an armed stop the instant one trips.
    ///
    /// This runs on the sensor thread, between CAN frames, so the stop leaves
    /// for the pump as soon as the reading that justifies it arrives. Handing
    /// the verdict back to the caller to act on would add a network round trip
    /// and a poll interval, and the piston travels through both.
    fn watch(&mut self, adc: &[u8; crate::sensors::CHANNELS]) {
        // The bus clock, so the filters see the interval that actually
        // elapsed. These samples share an adapter and do not arrive on a
        // cadence; a filter given a nominal rate would be a different filter
        // whenever the real one drifted.
        let now = self.state.lock().unwrap().now() as f32;
        for (channel, detector) in self.detectors.iter_mut().enumerate() {
            detector.push(adc[channel], now);
        }
        let Some(armed) = self.state.lock().unwrap().armed_stop else { return };
        let Some(detector) = self.detectors.get(armed.channel as usize) else { return };
        if !armed.on.met(detector) {
            return;
        }
        let Some(commands) = &self.config.commands else { return };
        // Send first, record after: the frame is what stops the pump.
        let _ = commands.send(BusCmd::Device(crate::devices::DeviceId::Pp01, crate::bus::Op::Stop));
        let (pump, at) = {
            let s = self.state.lock().unwrap();
            (s.dev(crate::devices::DeviceId::Pp01).value, s.now())
        };
        let reading = detector.smoothed().unwrap_or_default();
        self.update_sensors(|_| {});
        {
            let mut s = self.state.lock().unwrap();
            s.armed_stop = None;
            s.stop_fired = Some(StopFired { channel: armed.channel, pump: None, pump_at_trip: pump, reading, at });
        }
        self.log(
            Level::Info,
            format!(
                "channel {} tripped at reading {reading:.0}: PP01 stopped at {}",
                armed.channel,
                pump.map(|p| p.to_string()).unwrap_or_else(|| "?".into())
            ),
        );
        self.wake.call();
    }

    /// Logs a detector change, at most once a second per channel.
    ///
    /// Unthrottled, a flickering detector produces twenty lines a second, and
    /// every one of them is copied to every connected client. That filled the
    /// per-client queues and the server started dropping clients for falling
    /// behind — a sensor problem turning into a connectivity problem. The
    /// count of what was swallowed goes out with the next line, so the flicker
    /// is still visible; it just is not shouted.
    fn log_change(&mut self, channel: u8, liquid: bool) {
        let i = channel as usize;
        if i >= crate::sensors::CHANNELS {
            return;
        }
        let recent = self.last_change_log[i].is_some_and(|t| t.elapsed() < CHANGE_LOG_EVERY);
        if recent {
            self.swallowed[i] = self.swallowed[i].saturating_add(1);
            return;
        }
        let also = match self.swallowed[i] {
            0 => String::new(),
            n => format!(" ({n} more changes in the last second)"),
        };
        self.swallowed[i] = 0;
        self.last_change_log[i] = Some(Instant::now());
        self.log(
            Level::Info,
            format!("sensor channel {channel} -> {}{also}", if liquid { "liquid" } else { "dry" }),
        );
    }

    fn sensor_down(&self, why: String) {
        let was_up = self.state.lock().unwrap().sensors.connected;
        self.update_sensors(|s| s.go_offline(Some(why.clone())));
        if was_up {
            self.log(Level::Warn, format!("optical sensor board: {why}"));
        }
    }

    fn execute(&self, ctl: &mut SlotTempController, op: TempOp) -> Result<(), String> {
        // `clear_errors` answers with the fault words it just cleared; the rest
        // answer with nothing. Only success matters here.
        let result = match op {
            TempOp::SetTemperature(c) => ctl.set_temperature(c),
            TempOp::SetTolerance(c) => ctl.set_tolerance(c),
            TempOp::StartPid => ctl.start_pid(),
            TempOp::StopPid => ctl.stop_pid(),
            TempOp::ClearErrors => ctl.clear_errors().map(|_| ()),
        };
        match result {
            Ok(()) => {
                self.log(Level::Info, format!("temperature: {} — accepted", op.label()));
                Ok(())
            }
            // A refused command is the board disagreeing, not the link failing:
            // say so and stay connected.
            Err(e @ SlotTempError::Nack { .. }) => {
                self.log(Level::Error, format!("temperature: {} rejected — {e}", op.label()));
                Ok(())
            }
            Err(e) => {
                self.log(Level::Error, format!("temperature: {} failed — {e}", op.label()));
                Err(format!("command: {e}"))
            }
        }
    }
}

/// Finds the adapter by what it is rather than where it landed.
///
/// `/dev/serial/by-id` names a device by its USB serial number, so it points at
/// the same adapter after a re-enumeration; `/dev/ttyACM0` points at whatever
/// enumerated first that boot. Falling back to `ttyACM0` is a last resort and
/// is usually wrong when the by-id lookup has failed.
fn default_port() -> Result<String, String> {
    for pattern in [
        "/dev/serial/by-id/usb-WeAct_Studio*USB2CAN*",
        "/dev/serial/by-id/*USB2CAN*",
    ] {
        if let Ok(path) = glob_first(pattern) {
            return Ok(path);
        }
    }
    Ok("/dev/ttyACM0".to_string())
}

fn glob_first(pattern: &str) -> Result<String, ()> {
    let (dir, prefix) = pattern.rsplit_once('/').ok_or(())?;
    let needle: Vec<&str> = prefix.split('*').filter(|p| !p.is_empty()).collect();
    let entries = std::fs::read_dir(dir).map_err(|_| ())?;
    for entry in entries.flatten() {
        let name = entry.file_name().to_string_lossy().to_string();
        if needle.iter().all(|n| name.contains(n)) {
            return Ok(entry.path().to_string_lossy().to_string());
        }
    }
    Err(())
}

/// The sensor board's CAN id for a rig, or `None` when it has no sensors.
pub fn sensor_config(rig: &Rig, device_id: u16) -> Config {
    Config {
        port: String::new(),
        commands: None,
        sensors: !rig.sensors.is_empty(),
        sensor_device_id: device_id,
    }
}

//! Single worker thread that owns the RS485 port (or the simulator), runs
//! queued device commands and polls every device for live state.
//!
//! [`Bus`] is the only thing the front end sees: it takes [`BusCmd`]s and
//! publishes a [`BusState`] snapshot. Whether that work happens in a local
//! worker thread or on `tstand_server` across the network is decided here,
//! from [`Settings::backend`], and nothing above this module needs to know.

use std::collections::VecDeque;
use std::io::{Read, Write};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::mpsc::{Receiver, RecvTimeoutError, Sender, channel};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

use serde::{Deserialize, Serialize};

use crate::config::{Backend, Settings};
use crate::controller_api as api;
use crate::devices::{DeviceId, Kind};
use crate::rig::Role;
use crate::protocol::{self, Reply};
use crate::detect::StopOn;
use crate::sensors::SensorState;
use crate::temperature::{TempOp, TempState};

/// Nudges whoever is waiting on new bus state: the egui repaint in the GUI,
/// the broadcast loop in the server. Keeps this module free of eframe so the
/// server binary builds without a display.
#[derive(Clone)]
pub struct Wake(Option<Arc<dyn Fn() + Send + Sync>>);

impl Wake {
    pub fn none() -> Self {
        Self(None)
    }

    pub fn new(f: impl Fn() + Send + Sync + 'static) -> Self {
        Self(Some(Arc::new(f)))
    }

    pub fn call(&self) {
        if let Some(f) = &self.0 {
            f();
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Op {
    Home,
    ForcedReset,
    Stop,
    ValveTo(u16),
    PumpTo(u16),
    Aspirate(u16),
    Dispense(u16),
    SetSpeed(u16),
    /// true: IO3 low (0x61) = "input", PP01 <-> SV01 (wash side)
    /// false: IO3 high (0x60) = "output", PP01 <-> holding coil <-> SV02
    SolenoidInput(bool),
    SyncPosition,
    QueryVersion,
}

impl Op {
    pub fn label(&self) -> String {
        match self {
            Op::Home => "home".into(),
            Op::ForcedReset => "forced reset".into(),
            Op::Stop => "stop".into(),
            Op::ValveTo(p) => format!("switch to port {p}"),
            Op::PumpTo(p) => format!("move to {p}"),
            Op::Aspirate(s) => format!("aspirate {s} steps"),
            Op::Dispense(s) => format!("dispense {s} steps"),
            Op::SetSpeed(s) => format!("speed {s}"),
            Op::SolenoidInput(true) => "solenoid -> input (SV01)".into(),
            Op::SolenoidInput(false) => "solenoid -> output (coil/SV02)".into(),
            Op::SyncPosition => "sync position".into(),
            Op::QueryVersion => "query version".into(),
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum BusCmd {
    /// Boxed: the settings dwarf every other variant, and this enum travels
    /// through channels where a command is usually a two-word valve move.
    Apply(Box<Settings>),
    Device(DeviceId, Op),
    StopAll,
    HomeAll,
    SetPolling(bool),
    SetTrace(bool),
    /// The Peltier board on CAN. Routed to its own worker, not the RS485 one:
    /// different adapter, different thread, and a blocking temperature command
    /// must never hold up a STOP.
    Temp(TempOp),
    /// Watch a detector and stop PP01 the moment it meets the condition, or
    /// `None` to disarm.
    ///
    /// This is how a pump is stopped somewhere other than a position it was
    /// told to go to. The sensor thread is already reading the board twenty
    /// times a second; it decides and sends the stop itself rather than
    /// handing a verdict to something else to act on, because every hop is
    /// pump travel.
    ArmStop(Option<ArmedStop>),
}

/// A standing instruction to stop PP01 on what a detector sees.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct ArmedStop {
    /// Sensor board channel to watch.
    pub channel: u8,
    pub on: StopOn,
}

/// What happened when an armed stop fired.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct StopFired {
    pub channel: u8,
    /// Where PP01 actually came to rest.
    ///
    /// Filled in once the piston has stopped, not when the stop was sent. The
    /// position in shared state is polled a few times a second, so at the
    /// moment of the trip it is already stale — reporting it would understate
    /// the travel by however far the pump moved between the last poll and the
    /// stop taking effect, which is exactly the number this exists to give.
    pub pump: Option<u16>,
    /// Position as last polled when the trip fired, before it stopped.
    pub pump_at_trip: Option<u16>,
    /// The filtered reading that tripped it.
    pub reading: f32,
    /// Seconds since the bus started.
    pub at: f64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Level {
    Info,
    Warn,
    Error,
    Trace,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct LogLine {
    pub t: f64,
    pub level: Level,
    pub text: String,
    /// Monotonic line number, so a remote client can ask only for what it has
    /// not seen instead of re-reading the whole ring buffer every tick.
    pub seq: u64,
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct DeviceLive {
    pub online: bool,
    pub status: Option<u8>,
    /// Pump: piston position in steps. Valve: current port (0 = reset).
    pub value: Option<u16>,
    pub target: Option<u16>,
    /// Polls seen with the motor idle while a target is pending. The hardware
    /// does not always stop on the value we asked for (a valve reset settles
    /// on its origin port, not on 0), so the target is dropped after two idle
    /// polls instead of waiting for an exact match that never comes.
    #[serde(skip)]
    idle_polls: u8,
    pub speed: Option<u16>,
    pub solenoid_input: Option<bool>,
    pub version: Option<u16>,
    /// +1 position rising (aspirating / valve moving), -1 falling, 0 idle.
    pub motion: i8,
    pub last_seen: Option<f64>,
    pub errors: u32,
    pub last_error: Option<String>,
    /// (t, value) samples for trend plots. Never sent over the wire — two
    /// minutes of samples for five devices dwarfs the rest of a snapshot, and
    /// a remote client rebuilds the same curve from the values it receives.
    #[serde(skip)]
    pub history: VecDeque<(f64, f32)>,
}

/// Coarse device state shown on badges.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DevState {
    Offline,
    Ready,
    Moving,
    Fault,
}

impl DevState {
    pub fn label(self) -> &'static str {
        match self {
            DevState::Offline => "OFFLINE",
            DevState::Ready => "READY",
            DevState::Moving => "MOVING",
            DevState::Fault => "FAULT",
        }
    }
}

impl DeviceLive {
    pub fn state(&self) -> DevState {
        match (self.online, self.status) {
            (false, _) => DevState::Offline,
            (true, Some(s)) if protocol::status_is_fault(s) => DevState::Fault,
            (true, Some(0x04 | 0xFE)) => DevState::Moving,
            (true, _) if self.target.is_some() => DevState::Moving,
            _ => DevState::Ready,
        }
    }

    /// Position change over roughly the last second, in steps/s (positive = aspirating).
    pub fn steps_per_sec(&self) -> f32 {
        if self.motion == 0 {
            return 0.0;
        }
        let Some(&(t1, v1)) = self.history.back() else { return 0.0 };
        let Some(&(t0, v0)) = self.history.iter().rev().find(|(t, _)| t1 - t >= 0.8) else { return 0.0 };
        ((v1 - v0) as f64 / (t1 - t0)) as f32
    }
}

#[derive(Clone, Debug)]
pub struct BusState {
    pub backend: Backend,
    pub connected: bool,
    pub connection_error: Option<String>,
    pub polling: bool,
    pub trace: bool,
    pub devices: [DeviceLive; 5],
    pub log: VecDeque<LogLine>,
    pub frames_tx: u64,
    pub frames_bad: u64,
    pub cycle_ms: f64,
    /// The Peltier slot-temperature board on the CAN adapter.
    pub temp: TempState,
    /// The optical liquid-sensor board, on the same adapter.
    pub sensors: SensorState,
    /// A stop waiting on a detector, if one is armed.
    pub armed_stop: Option<ArmedStop>,
    /// The last armed stop to fire.
    pub stop_fired: Option<StopFired>,
    /// `host:port` of the server driving the rig, when this is a remote view.
    /// `None` when the devices are on this machine's own bus.
    pub remote: Option<String>,
    /// Zero of every timestamp in `log` and `history`. A remote client slides
    /// this back by the server's uptime so both agree on what `t` means.
    pub started: Instant,
}

impl Default for BusState {
    fn default() -> Self {
        Self {
            backend: Backend::Simulator,
            connected: false,
            connection_error: None,
            polling: true,
            trace: false,
            devices: Default::default(),
            log: VecDeque::new(),
            frames_tx: 0,
            frames_bad: 0,
            cycle_ms: 0.0,
            temp: TempState::default(),
            sensors: SensorState::default(),
            armed_stop: None,
            stop_fired: None,
            remote: None,
            started: Instant::now(),
        }
    }
}

impl BusState {
    #[cfg(test)]
    pub fn for_tests() -> Self {
        Self { connected: true, polling: false, ..Default::default() }
    }

    pub fn dev(&self, id: DeviceId) -> &DeviceLive {
        &self.devices[id as usize]
    }

    pub fn now(&self) -> f64 {
        self.started.elapsed().as_secs_f64()
    }

    /// Appends a log line and trims the ring buffer. Used by the local worker
    /// and by the remote client replaying the server's log.
    pub fn push_log(&mut self, line: LogLine) {
        self.log.push_back(line);
        while self.log.len() > LOG_LINES {
            self.log.pop_front();
        }
    }

    /// Records a trend sample for `id` and drops anything older than the plot
    /// window.
    pub fn push_sample(&mut self, id: DeviceId, t: f64, value: f32) {
        let h = &mut self.devices[id as usize].history;
        h.push_back((t, value));
        while h.front().is_some_and(|(t0, _)| t - t0 > HISTORY_SECS) {
            h.pop_front();
        }
    }
}

const HISTORY_SECS: f64 = 120.0;
const LOG_LINES: usize = 800;

/// Source of `LogLine::seq`. One counter for the process is enough: the server
/// runs a single bus, and a client only ever compares sequence numbers from
/// the one server it is connected to.
static LOG_SEQ: AtomicU64 = AtomicU64::new(1);

pub fn next_log_seq() -> u64 {
    LOG_SEQ.fetch_add(1, Ordering::Relaxed)
}

/// Which kind of backend a settings snapshot asks for. Switching between the
/// two means tearing down one worker and starting the other; switching *within*
/// one (simulator to serial, or a different remote host) is handled by the
/// backend itself.
#[derive(Clone, Debug, PartialEq, Eq)]
enum Mode {
    Local,
    Remote { host: String, token: String },
}

impl Mode {
    fn of(s: &Settings) -> Self {
        match s.backend {
            Backend::Remote => {
                Mode::Remote { host: s.server_host.trim().to_string(), token: s.server_token.clone() }
            }
            _ => Mode::Local,
        }
    }
}

/// Handle on the running bus: commands in, state snapshots out.
///
/// The same handle serves a local worker and a remote server, and survives a
/// switch between them — [`Bus::send`] notices when [`BusCmd::Apply`] changes
/// the mode and restarts the backend underneath, so callers keep their `&Bus`.
pub struct Bus {
    pub state: Arc<Mutex<BusState>>,
    wake: Wake,
    inner: Mutex<Inner>,
    /// Replies to controller_v2 requests, whichever route carried them. Behind
    /// a mutex so `Bus` stays `Sync`: the server shares one across threads.
    api_rx: Mutex<Receiver<api::Reply>>,
    api_reply_tx: Sender<api::Reply>,
    /// Direct HTTP worker, used whenever the bus is local.
    api_direct: Sender<(String, api::Request)>,
}

struct Inner {
    mode: Mode,
    tx: Sender<BusCmd>,
    /// The CAN worker, when this machine has the adapter.
    temp_tx: Option<Sender<TempOp>>,
    api_tx: Option<Sender<api::Request>>,
    /// What the running backend was last told to use. The server hands this to
    /// each GUI that connects so Settings shows the rig's truth.
    settings: Settings,
}

impl Bus {
    pub fn spawn(wake: Wake, settings: Settings) -> Self {
        let state = Arc::new(Mutex::new(BusState {
            backend: settings.effective_backend(),
            ..Default::default()
        }));
        let (api_reply_tx, api_rx) = channel();
        let api_direct = api::spawn_direct(wake.clone(), api_reply_tx.clone());
        let inner = Inner {
            mode: Mode::of(&settings),
            tx: channel().0,
            temp_tx: None,
            api_tx: None,
            settings: settings.clone(),
        };
        let bus = Self {
            state,
            wake,
            inner: Mutex::new(inner),
            api_rx: Mutex::new(api_rx),
            api_reply_tx,
            api_direct,
        };
        bus.start(&settings);
        bus
    }

    /// Starts the backend `settings` asks for, replacing any running one. The
    /// old worker's receiver is dropped, which is how it learns to exit.
    fn start(&self, settings: &Settings) {
        let mode = Mode::of(settings);
        let (tx, rx) = channel();
        let mut api_tx = None;
        let mut temp_tx = None;
        match &mode {
            Mode::Local => {
                let state = self.state.clone();
                let wake = self.wake.clone();
                let worker_settings = settings.clone();
                thread::Builder::new()
                    .name("rs485-bus".into())
                    .spawn(move || Worker::new(state, wake, worker_settings).run(rx))
                    .expect("spawn bus worker");
                temp_tx = self.start_temperature(settings, tx.clone());
            }
            Mode::Remote { host, token } => {
                api_tx = Some(crate::remote::client::spawn(
                    self.state.clone(),
                    self.wake.clone(),
                    host.clone(),
                    token.clone(),
                    settings.clone(),
                    rx,
                    self.api_reply_tx.clone(),
                ));
            }
        }
        let mut inner = self.inner.lock().unwrap();
        inner.mode = mode;
        inner.tx = tx;
        inner.temp_tx = temp_tx;
        inner.api_tx = api_tx;
        inner.settings = settings.clone();
    }

    /// Starts the CAN worker if this build has it and the rig is configured
    /// for a temperature board. Dropping the previous sender stops the old one.
    #[cfg(feature = "can")]
    fn start_temperature(&self, settings: &Settings, commands: Sender<BusCmd>) -> Option<Sender<TempOp>> {
        if !settings.temp_enabled {
            let mut s = self.state.lock().unwrap();
            s.temp = TempState::default();
            s.sensors = SensorState::default();
            return None;
        }
        Some(crate::can::spawn(
            self.state.clone(),
            self.wake.clone(),
            crate::can::Config {
                port: settings.temp_port.clone(),
                commands: Some(commands),
                sensors: !settings.rig.sensors.is_empty(),
                sensor_device_id: settings.sensor_can_id,
            },
        ))
    }

    /// Without the `can` feature there is no CAN stack linked in, so the board
    /// is simply reported as absent rather than pretended into existence.
    #[cfg(not(feature = "can"))]
    fn start_temperature(&self, _settings: &Settings, _commands: Sender<BusCmd>) -> Option<Sender<TempOp>> {
        let mut s = self.state.lock().unwrap();
        s.temp = TempState::default();
        s.sensors = SensorState::default();
        None
    }

    pub fn send(&self, cmd: BusCmd) {
        if let BusCmd::Apply(settings) = &cmd
            && Mode::of(settings) != self.inner.lock().unwrap().mode
        {
            // A different kind of backend: reset the mirror so stale device
            // readings from the old one are not shown as live.
            {
                let mut s = self.state.lock().unwrap();
                s.devices = Default::default();
                s.connected = false;
                s.connection_error = None;
                s.remote = None;
                s.backend = settings.effective_backend();
            }
            self.start(settings);
            // A fresh local worker already applied these settings; a fresh
            // remote client sends them on connect. Nothing more to forward.
            self.wake.call();
            return;
        }
        let mut inner = self.inner.lock().unwrap();
        if let BusCmd::Apply(settings) = &cmd {
            inner.settings = (**settings).clone();
        }
        // A temperature command belongs to the CAN worker where the adapter
        // is. On a remote GUI there is no worker, so it goes down the same
        // channel as everything else and the server sorts it out there.
        if let BusCmd::Temp(op) = cmd
            && let Some(tx) = &inner.temp_tx
        {
            let _ = tx.send(op);
            return;
        }
        let _ = inner.tx.send(cmd);
    }

    pub fn device(&self, id: DeviceId, op: Op) {
        self.send(BusCmd::Device(id, op));
    }

    pub fn snapshot(&self) -> BusState {
        self.state.lock().unwrap().clone()
    }

    /// Sends a controller_v2 request: straight over HTTP when the bus is
    /// local, tunnelled through the server when it is remote. `host` is only
    /// used for the direct route — the server knows its own controller
    /// address, which is loopback from where it runs.
    pub fn api(&self, host: &str, request: api::Request) {
        let inner = self.inner.lock().unwrap();
        match &inner.api_tx {
            Some(tx) => {
                let _ = tx.send(request);
            }
            None => {
                let _ = self.api_direct.send((host.to_string(), request));
            }
        }
    }

    /// Next controller_v2 reply, or `None` when there is nothing waiting.
    pub fn next_api_reply(&self) -> Option<api::Reply> {
        self.api_rx.lock().unwrap().try_recv().ok()
    }

    /// The settings the running backend is using.
    pub fn settings(&self) -> Settings {
        self.inner.lock().unwrap().settings.clone()
    }

    /// True when the hardware is on another machine.
    pub fn is_remote(&self) -> bool {
        matches!(self.inner.lock().unwrap().mode, Mode::Remote { .. })
    }

    /// Ports to offer in Settings: the adapter is wherever the bus is, so a
    /// remote bus answers with the server's ports, not this machine's.
    pub fn serial_ports(&self) -> Vec<String> {
        if self.is_remote() {
            crate::remote::client::remote_ports()
        } else {
            crate::remote::server::list_ports()
        }
    }
}

trait Transport: Send {
    fn transact(&mut self, addr: u8, func: u8, param: u16) -> Result<Reply, String>;
    fn describe(&self) -> String;
}

struct SerialTransport {
    port: Box<dyn serialport::SerialPort>,
    name: String,
}

impl SerialTransport {
    fn open(settings: &Settings) -> Result<Self, String> {
        if settings.port.trim().is_empty() {
            return Err("no serial port selected".into());
        }
        let port = serialport::new(settings.port.trim(), settings.baud_rate)
            .data_bits(serialport::DataBits::Eight)
            .parity(serialport::Parity::None)
            .stop_bits(serialport::StopBits::One)
            .timeout(Duration::from_millis(settings.reply_timeout_ms.max(20)))
            .open()
            .map_err(|e| format!("cannot open {}: {e}", settings.port))?;
        Ok(Self {
            port,
            name: format!("{} @ {} bps", settings.port.trim(), settings.baud_rate),
        })
    }
}

impl Transport for SerialTransport {
    fn transact(&mut self, addr: u8, func: u8, param: u16) -> Result<Reply, String> {
        let _ = self.port.clear(serialport::ClearBuffer::Input);
        self.port
            .write_all(&protocol::encode(addr, func, param))
            .map_err(|e| format!("write: {e}"))?;
        // Resynchronise on the frame header in case of line noise.
        let mut byte = [0u8; 1];
        loop {
            self.port.read_exact(&mut byte).map_err(|_| "no reply (timeout)".to_string())?;
            if byte[0] == protocol::STX {
                break;
            }
        }
        let mut frame = [0u8; 8];
        frame[0] = protocol::STX;
        self.port
            .read_exact(&mut frame[1..])
            .map_err(|_| "short reply (timeout)".to_string())?;
        let reply = protocol::decode(&frame)?;
        if reply.addr != addr {
            return Err(format!("reply from address {} (expected {addr})", reply.addr));
        }
        Ok(reply)
    }

    fn describe(&self) -> String {
        self.name.clone()
    }
}

/// Simulated devices that answer the same protocol, timed from the datasheets.
struct SimTransport {
    started: Instant,
    devices: Vec<SimDevice>,
}

struct SimDevice {
    addr: u8,
    id: DeviceId,
    pos: f64,
    from: f64,
    target: f64,
    move_start: f64,
    move_secs: f64,
    speed: u16,
}

impl SimTransport {
    fn new(settings: &Settings) -> Self {
        let devices = DeviceId::ALL
            .iter()
            .map(|&id| SimDevice {
                addr: settings.addr(id),
                id,
                pos: 0.0,
                from: 0.0,
                target: 0.0,
                move_start: 0.0,
                move_secs: 0.0,
                speed: 200,
            })
            .collect();
        Self { started: Instant::now(), devices }
    }
}

impl SimDevice {
    fn update(&mut self, now: f64) {
        if self.move_secs <= 0.0 {
            return;
        }
        let k = ((now - self.move_start) / self.move_secs).clamp(0.0, 1.0);
        self.pos = self.from + (self.target - self.from) * k;
        if k >= 1.0 {
            self.pos = self.target;
            self.move_secs = 0.0;
        }
    }

    fn busy(&self) -> bool {
        self.move_secs > 0.0
    }

    fn start_move(&mut self, now: f64, target: f64) {
        self.from = self.pos;
        self.target = target;
        self.move_start = now;
        self.move_secs = match self.id.kind() {
            // RP-01: 3820 steps in 2.2 s at 500 rpm, scales with speed.
            Kind::Pump { .. } => {
                let steps_per_sec = 3820.0 / 2.2 * (self.speed.max(1) as f64 / 500.0);
                ((target - self.pos).abs() / steps_per_sec).max(0.05)
            }
            // SV-07: ≤2 s (8 port) / ≤3.3 s (16 port) per revolution, shortest path.
            Kind::Valve { ports } => {
                let rev = if ports == 16 { 3.3 } else { 2.0 };
                let n = ports as f64;
                let d = (target - self.pos).abs() % n;
                (rev * d.min(n - d) / n).max(0.3)
            }
        };
    }
}

impl Transport for SimTransport {
    fn transact(&mut self, addr: u8, func: u8, param: u16) -> Result<Reply, String> {
        let now = self.started.elapsed().as_secs_f64();
        thread::sleep(Duration::from_millis(8)); // 9600 bps round trip, roughly
        let dev = self
            .devices
            .iter_mut()
            .find(|d| d.addr == addr)
            .ok_or_else(|| "no reply (timeout)".to_string())?;
        dev.update(now);
        let busy = dev.busy();
        let ok = |status: u8, value: u16| Ok(Reply { addr, status, value });
        use protocol::*;
        match (dev.id.kind(), func) {
            (_, Q_MOTOR_STATUS) => ok(if busy { 0x04 } else { 0x00 }, 0),
            (_, Q_VERSION) => ok(0x00, 0x011E),
            (_, Q_ADDRESS) => ok(0x00, addr as u16),
            (Kind::Pump { .. }, Q_PISTON_POS) => ok(0x00, dev.pos.round() as u16),
            (Kind::Valve { .. }, Q_CHANNEL) => {
                let v = if busy { dev.from } else { dev.pos };
                ok(0x00, v.round() as u16)
            }
            (_, STOP) => {
                dev.target = dev.pos;
                dev.move_secs = 0.0;
                ok(0xFE, 0)
            }
            _ if busy => ok(0x04, 0),
            (Kind::Pump { .. }, RESET | FORCED_RESET) => {
                dev.start_move(now, 0.0);
                ok(0xFE, 0)
            }
            (Kind::Pump { .. }, PUMP_ASPIRATE) => {
                if dev.pos + param as f64 > 12000.0 {
                    return ok(0x02, 0);
                }
                dev.start_move(now, dev.pos + param as f64);
                ok(0xFE, 0)
            }
            (Kind::Pump { .. }, PUMP_DISPENSE) => {
                dev.start_move(now, (dev.pos - param as f64).max(0.0));
                ok(0xFE, 0)
            }
            (Kind::Pump { .. }, PUMP_ABS) => {
                dev.start_move(now, param as f64);
                ok(0xFE, 0)
            }
            (Kind::Pump { .. }, SET_SPEED) => {
                dev.speed = param.clamp(1, 500);
                ok(0x00, 0)
            }
            (Kind::Pump { has_solenoid: true }, SOLENOID_HIGH | SOLENOID_LOW) => ok(0x00, 0),
            (Kind::Pump { .. }, SYNC_PISTON_POS) => ok(0x00, 0),
            (Kind::Valve { ports }, VALVE_SWITCH) => {
                if param == 0 || param > ports {
                    return ok(0x02, 0);
                }
                dev.start_move(now, param as f64);
                ok(0xFE, 0)
            }
            (Kind::Valve { .. }, RESET | VALVE_RESET) => {
                dev.start_move(now, 0.0);
                ok(0xFE, 0)
            }
            _ => ok(0x07, 0),
        }
    }

    fn describe(&self) -> String {
        "simulator".into()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocol::*;

    fn sim() -> (SimTransport, Settings) {
        let s = Settings::default();
        (SimTransport::new(&s), s)
    }

    fn wait_idle(t: &mut SimTransport, addr: u8) {
        for _ in 0..500 {
            if t.transact(addr, Q_MOTOR_STATUS, 0).unwrap().status == 0x00 {
                return;
            }
        }
        panic!("device {addr} never went idle");
    }

    #[test]
    fn valve_switches_and_reports_channel() {
        let (mut t, s) = sim();
        let a = s.addr(DeviceId::Sv02);
        assert_eq!(t.transact(a, VALVE_SWITCH, 15).unwrap().status, 0xFE);
        assert_eq!(t.transact(a, Q_MOTOR_STATUS, 0).unwrap().status, 0x04);
        wait_idle(&mut t, a);
        assert_eq!(t.transact(a, Q_CHANNEL, 0).unwrap().value, 15);
        assert_eq!(t.transact(a, VALVE_SWITCH, 17).unwrap().status, 0x02, "port beyond 16 rejected");
    }

    #[test]
    fn pump_relative_moves() {
        let (mut t, s) = sim();
        let a = s.addr(DeviceId::Pp01);
        t.transact(a, SET_SPEED, 500).unwrap();
        t.transact(a, PUMP_ASPIRATE, 300).unwrap();
        assert_eq!(t.transact(a, PUMP_ASPIRATE, 10).unwrap().status, 0x04, "busy while moving");
        wait_idle(&mut t, a);
        assert_eq!(t.transact(a, Q_PISTON_POS, 0).unwrap().value, 300);
        t.transact(a, PUMP_DISPENSE, 100).unwrap();
        wait_idle(&mut t, a);
        assert_eq!(t.transact(a, Q_PISTON_POS, 0).unwrap().value, 200);
    }

    #[test]
    fn unknown_address_times_out() {
        let (mut t, _) = sim();
        assert!(t.transact(0x55, Q_MOTOR_STATUS, 0).is_err());
    }

    /// A valve like the ones on the rig: a reset parks the rotor on its origin
    /// port and the channel query then answers 1, never the 0 we asked for.
    struct ValveParkingOnPortOne {
        busy: u8,
        port: u16,
    }

    impl Transport for ValveParkingOnPortOne {
        fn transact(&mut self, addr: u8, func: u8, _param: u16) -> Result<Reply, String> {
            let ok = |status: u8, value: u16| Ok(Reply { addr, status, value });
            match func {
                RESET => {
                    self.busy = 2;
                    ok(0x00, 0)
                }
                Q_CHANNEL => ok(0x00, self.port),
                Q_MOTOR_STATUS if self.busy > 0 => {
                    self.busy -= 1;
                    if self.busy == 0 {
                        self.port = 1;
                    }
                    ok(0x04, 0)
                }
                _ => ok(0x00, 0),
            }
        }

        fn describe(&self) -> String {
            "test valve".into()
        }
    }

    /// Frames a test pump was sent: (function, parameter).
    type SentFrames = std::sync::Arc<Mutex<Vec<(u8, u16)>>>;

    /// A pump that answers whatever it is asked, so a test can see which
    /// frames reached it.
    struct Recorder {
        sent: SentFrames,
        /// What the piston reports, since an absolute move reads it to work
        /// out which way it is about to go.
        pos: u16,
    }

    impl Transport for Recorder {
        fn transact(&mut self, addr: u8, func: u8, _param: u16) -> Result<Reply, String> {
            self.sent.lock().unwrap().push((func, _param));
            let value = if func == protocol::Q_PISTON_POS { self.pos } else { 0 };
            Ok(Reply { addr, status: 0x00, value })
        }

        fn describe(&self) -> String {
            "recorder".into()
        }
    }

    /// Sets up a worker with SV02 parked on `port` and the solenoid facing it.
    fn worker_on_sv02(port: u16) -> (Worker, SentFrames) {
        let state = Arc::new(Mutex::new(BusState::for_tests()));
        let mut w = Worker::new(state, Wake::none(), Settings::default());
        // After `new`, which connects and so clears the device table.
        {
            let mut s = w.state.lock().unwrap();
            s.devices[DeviceId::Sv02 as usize].value = Some(port);
            s.devices[DeviceId::Pp01 as usize].value = Some(1000);
            s.devices[DeviceId::Pp01 as usize].solenoid_input = Some(false);
        }
        let sent: SentFrames = std::sync::Arc::new(Mutex::new(Vec::new()));
        w.transport = Some(Box::new(Recorder { sent: sent.clone(), pos: 1000 }));
        sent.lock().unwrap().clear();
        (w, sent)
    }

    fn pump_frames(sent: &SentFrames) -> Vec<u8> {
        sent.lock()
            .unwrap()
            .iter()
            .map(|(f, _)| *f)
            .filter(|f| matches!(*f, protocol::PUMP_ASPIRATE | protocol::PUMP_DISPENSE | protocol::RESET | protocol::FORCED_RESET))
            .collect()
    }

    /// The air port is plumbed to draw air in. Pushing liquid out through it
    /// wets the filter and the line behind it, and no valve downstream stops
    /// that — so the bus does.
    #[test]
    fn liquid_is_never_pushed_out_through_the_air_port() {
        let (mut w, sent) = worker_on_sv02(16); // SV02 port 16 is air on this rig
        w.execute(DeviceId::Pp01, Op::Dispense(100));
        w.execute(DeviceId::Pp01, Op::Home);
        w.execute(DeviceId::Pp01, Op::ForcedReset);
        assert!(pump_frames(&sent).is_empty(), "nothing may be pushed out through air");

        // Drawing air in through it is the whole point, and is allowed.
        w.execute(DeviceId::Pp01, Op::Aspirate(100));
        assert_eq!(pump_frames(&sent), vec![protocol::PUMP_ASPIRATE]);
    }

    /// Waste is plumbed to take liquid away. Drawing through it pulls whatever
    /// has collected there back up into the rig.
    #[test]
    fn nothing_is_ever_drawn_back_out_of_waste() {
        let (mut w, sent) = worker_on_sv02(9); // SV02 port 9 is waste
        w.execute(DeviceId::Pp01, Op::Aspirate(100));
        assert!(pump_frames(&sent).is_empty(), "waste must never be drawn from");

        // Pushing liquid out to waste is what it is for.
        w.execute(DeviceId::Pp01, Op::Dispense(100));
        w.execute(DeviceId::Pp01, Op::Home);
        assert_eq!(pump_frames(&sent), vec![protocol::PUMP_DISPENSE, protocol::RESET]);
    }

    /// With the solenoid on the wash side SV02 is not in the path at all, so
    /// neither rule applies.
    #[test]
    fn the_guard_only_applies_to_the_side_sv02_is_on() {
        let (mut w, sent) = worker_on_sv02(9);
        w.state.lock().unwrap().devices[DeviceId::Pp01 as usize].solenoid_input = Some(true);
        w.execute(DeviceId::Pp01, Op::Aspirate(100));
        assert_eq!(pump_frames(&sent), vec![protocol::PUMP_ASPIRATE], "drawing wash is unaffected");
    }

    /// A move to an absolute position only reveals its direction once the
    /// piston has been read, so the guard has to run after that.
    #[test]
    fn an_absolute_move_is_guarded_by_the_direction_it_turns_out_to_need() {
        // On air: moving further out draws in, which is allowed.
        let (mut w, sent) = worker_on_sv02(16);
        w.execute(DeviceId::Pp01, Op::PumpTo(2000));
        assert_eq!(pump_frames(&sent), vec![protocol::PUMP_ASPIRATE]);

        // On air: moving back in would push out, which is not.
        let (mut w, sent) = worker_on_sv02(16);
        w.execute(DeviceId::Pp01, Op::PumpTo(10));
        assert!(pump_frames(&sent).is_empty(), "that would push liquid into the air line");
    }

    #[test]
    fn home_clears_the_target_even_when_the_valve_parks_elsewhere() {
        let state = Arc::new(Mutex::new(BusState::for_tests()));
        let mut w = Worker::new(state.clone(), Wake::none(), Settings::default());
        w.transport = Some(Box::new(ValveParkingOnPortOne { busy: 0, port: 3 }));

        w.execute(DeviceId::Sv01, Op::Home);
        assert_eq!(state.lock().unwrap().dev(DeviceId::Sv01).state(), DevState::Moving);

        for _ in 0..4 {
            w.poll_all();
        }
        let d = state.lock().unwrap().dev(DeviceId::Sv01).clone();
        assert_eq!(d.value, Some(1), "valve reports its origin port");
        assert_eq!(d.target, None, "target dropped once the motor is idle");
        assert_eq!(d.state(), DevState::Ready);
    }
}

struct Worker {
    state: Arc<Mutex<BusState>>,
    wake: Wake,
    settings: Settings,
    transport: Option<Box<dyn Transport>>,
    next_retry: [f64; 5],
}

impl Worker {
    fn new(state: Arc<Mutex<BusState>>, wake: Wake, settings: Settings) -> Self {
        let mut w = Self { state, wake, settings, transport: None, next_retry: [0.0; 5] };
        w.connect();
        w
    }

    fn now(&self) -> f64 {
        self.state.lock().unwrap().now()
    }

    fn log(&self, level: Level, text: impl Into<String>) {
        let mut s = self.state.lock().unwrap();
        if level == Level::Trace && !s.trace {
            return;
        }
        let t = s.now();
        s.push_log(LogLine { t, level, text: text.into(), seq: next_log_seq() });
    }

    fn connect(&mut self) {
        let result: Result<Box<dyn Transport>, String> = match self.settings.backend {
            Backend::Simulator => Ok(Box::new(SimTransport::new(&self.settings))),
            Backend::Serial => SerialTransport::open(&self.settings).map(|t| Box::new(t) as _),
            // Only reachable if a remote server is handed remote settings.
            Backend::Remote => Err("this bus drives hardware directly; it cannot chain to another server".into()),
        };
        let mut s = self.state.lock().unwrap();
        s.backend = self.settings.backend;
        s.devices = Default::default();
        drop(s);
        self.next_retry = [0.0; 5];
        match result {
            Ok(t) => {
                self.log(Level::Info, format!("connected: {}", t.describe()));
                self.transport = Some(t);
                let mut s = self.state.lock().unwrap();
                s.connected = true;
                s.connection_error = None;
            }
            Err(e) => {
                self.log(Level::Error, e.clone());
                self.transport = None;
                let mut s = self.state.lock().unwrap();
                s.connected = false;
                s.connection_error = Some(e);
            }
        }
    }

    fn run(mut self, rx: Receiver<BusCmd>) {
        let mut next_poll = Instant::now();
        loop {
            let wait = next_poll.saturating_duration_since(Instant::now());
            match rx.recv_timeout(wait) {
                Ok(cmd) => {
                    self.handle(cmd);
                    // Drain anything else queued before polling again.
                    while let Ok(cmd) = rx.try_recv() {
                        self.handle(cmd);
                    }
                }
                Err(RecvTimeoutError::Timeout) => {
                    let polling = self.state.lock().unwrap().polling;
                    if polling && self.transport.is_some() {
                        let t0 = Instant::now();
                        self.poll_all();
                        self.state.lock().unwrap().cycle_ms = t0.elapsed().as_secs_f64() * 1e3;
                        self.wake.call();
                    }
                    next_poll = Instant::now() + Duration::from_millis(self.settings.poll_interval_ms.max(50));
                }
                Err(RecvTimeoutError::Disconnected) => return,
            }
        }
    }

    fn handle(&mut self, cmd: BusCmd) {
        match cmd {
            BusCmd::Apply(settings) => {
                self.transport = None; // close the port before reopening
                self.settings = *settings;
                self.connect();
            }
            BusCmd::SetPolling(on) => self.state.lock().unwrap().polling = on,
            BusCmd::SetTrace(on) => self.state.lock().unwrap().trace = on,
            BusCmd::Device(id, op) => self.execute(id, op),
            BusCmd::StopAll => {
                for id in DeviceId::ALL {
                    self.execute(id, Op::Stop);
                }
            }
            BusCmd::HomeAll => {
                for id in DeviceId::ALL {
                    self.execute(id, Op::Home);
                }
            }
            // Arming is state the sensor thread reads, not a frame for any
            // device, so it is recorded here and acted on there.
            BusCmd::ArmStop(armed) => {
                let mut s = self.state.lock().unwrap();
                s.armed_stop = armed;
                if armed.is_some() {
                    s.stop_fired = None;
                }
                drop(s);
                match armed {
                    Some(a) => self.log(
                        Level::Info,
                        format!("armed: stop PP01 when channel {} sees {}", a.channel, a.on.label()),
                    ),
                    None => self.log(Level::Info, "disarmed the sensor stop"),
                }
            }
            // Only reached when no CAN worker took it: either this build has
            // no CAN support or the rig is configured without a board.
            BusCmd::Temp(op) => self.log(
                Level::Warn,
                format!("temperature: {} ignored, no board configured on this machine", op.label()),
            ),
        }
        self.wake.call();
    }

    fn transact(&mut self, id: DeviceId, func: u8, param: u16) -> Result<Reply, String> {
        let addr = self.settings.addr(id);
        let Some(t) = self.transport.as_mut() else {
            return Err("not connected".into());
        };
        let result = t.transact(addr, func, param);
        let now = self.now();
        let mut s = self.state.lock().unwrap();
        s.frames_tx += 1;
        let trace = s.trace;
        let d = &mut s.devices[id as usize];
        match &result {
            Ok(_) => {
                d.online = true;
                d.last_seen = Some(now);
                d.last_error = None;
            }
            Err(e) => {
                d.errors += 1;
                d.last_error = Some(e.clone());
                if d.online {
                    d.online = false;
                }
            }
        }
        if result.is_err() {
            s.frames_bad += 1;
        }
        drop(s);
        if trace {
            let tx = protocol::encode(addr, func, param);
            match &result {
                Ok(r) => self.log(
                    Level::Trace,
                    format!("{} TX {:02X?} RX status {:02X} value {}", id.tag(), tx, r.status, r.value),
                ),
                Err(e) => self.log(Level::Trace, format!("{} TX {:02X?} RX error: {e}", id.tag(), tx)),
            }
        }
        result
    }

    /// Refuses a pump move that would send liquid the wrong way through SV02.
    ///
    /// The air port exists to draw air *in*; pushing liquid out through it
    /// wets the filter and the line behind it. The waste port exists to push
    /// liquid *out*; drawing through it pulls waste back up into the rig. Both
    /// are one-way by plumbing rather than by valve, so nothing downstream
    /// enforces it — which is why this sits here, on the machine holding the
    /// bus, rather than in whichever interface happened to send the command.
    ///
    /// Only PP01's output side passes through SV02: with the solenoid on the
    /// SV01 side the pump is drawing from wash and SV02 is not in the path.
    fn flow_allowed(&self, id: DeviceId, pushing: bool) -> Result<(), String> {
        if id != DeviceId::Pp01 {
            return Ok(());
        }
        let s = self.state.lock().unwrap();
        if s.dev(DeviceId::Pp01).solenoid_input == Some(true) {
            return Ok(());
        }
        let Some(port) = s.dev(DeviceId::Sv02).value.filter(|p| *p > 0) else {
            return Ok(());
        };
        drop(s);
        match self.settings.rig.role(DeviceId::Sv02, port) {
            Some(Role::Air) if pushing => Err(format!(
                "SV02 is on air (port {port}): air is drawn in through it, never pushed out into"
            )),
            Some(Role::Waste) if !pushing => Err(format!(
                "SV02 is on waste (port {port}): liquid leaves through it, it is never drawn back from"
            )),
            _ => Ok(()),
        }
    }

    fn execute(&mut self, id: DeviceId, op: Op) {
        use protocol::*;
        let kind = id.kind();
        let max = self.settings.max_steps(id);
        let frame: Result<(u8, u16), String> = match (kind, op) {
            // Stop is always allowed: it is how everything below is undone.
            (_, Op::Stop) => Ok((STOP, 0)),
            // The flow guard goes ahead of the arms it protects, or `Home`
            // would match as an ordinary reset before anyone asked which way
            // it was about to move liquid.
            (Kind::Pump { .. }, Op::Aspirate(_)) if self.flow_allowed(id, false).is_err() => {
                Err(self.flow_allowed(id, false).unwrap_err())
            }
            (Kind::Pump { .. }, Op::Dispense(_) | Op::Home | Op::ForcedReset)
                if self.flow_allowed(id, true).is_err() =>
            {
                Err(self.flow_allowed(id, true).unwrap_err())
            }
            (Kind::Pump { .. }, Op::Home) => Ok((RESET, 0)),
            (Kind::Pump { .. }, Op::ForcedReset) => Ok((FORCED_RESET, 0)),
            (Kind::Valve { .. }, Op::Home | Op::ForcedReset) => Ok((RESET, 0)),
            // The rig guard runs here, on the machine holding the bus, so it
            // applies to every screen and to anything else sending commands —
            // not only to the interface that greys the button out.
            (Kind::Valve { .. }, Op::ValveTo(p)) if !self.settings.rig.port_allowed(id, p) => {
                Err(format!(
                    "port {p} ({}) is not plumbed on this rig",
                    self.settings.rig.port_label(id, p)
                ))
            }
            (Kind::Valve { ports }, Op::ValveTo(p)) if (1..=ports).contains(&p) => Ok((VALVE_SWITCH, p)),
            (Kind::Valve { ports }, Op::ValveTo(p)) => Err(format!("port {p} outside 1..={ports}")),
            (Kind::Pump { .. }, Op::Aspirate(n)) => {
                let cur = self.state.lock().unwrap().dev(id).value.unwrap_or(0);
                if cur as u32 + n as u32 > max as u32 {
                    Err(format!("aspirate {n} from {cur} exceeds max {max} steps"))
                } else {
                    Ok((PUMP_ASPIRATE, n))
                }
            }
            (Kind::Pump { .. }, Op::Dispense(n)) => Ok((PUMP_DISPENSE, n)),
            (Kind::Pump { .. }, Op::PumpTo(target)) => {
                if target > max {
                    Err(format!("target {target} exceeds max {max} steps"))
                } else {
                    // Relative moves, as controller_v2 does: read position first.
                    match self.transact(id, Q_PISTON_POS, 0) {
                        // Which way this goes is only known now, so the flow
                        // guard cannot run with the others above.
                        Ok(r) if target > r.value => match self.flow_allowed(id, false) {
                            Err(e) => Err(e),
                            Ok(()) => Ok((PUMP_ASPIRATE, target - r.value)),
                        },
                        Ok(r) if target < r.value => match self.flow_allowed(id, true) {
                            Err(e) => Err(e),
                            Ok(()) => Ok((PUMP_DISPENSE, r.value - target)),
                        },
                        Ok(_) => {
                            self.log(Level::Info, format!("{} already at {target}", id.tag()));
                            return;
                        }
                        Err(e) => Err(e),
                    }
                }
            }
            (Kind::Pump { .. }, Op::SetSpeed(s)) if (1..=500).contains(&s) => Ok((SET_SPEED, s)),
            (Kind::Pump { .. }, Op::SetSpeed(s)) => Err(format!("speed {s} outside 1..=500")),
            (Kind::Pump { has_solenoid: true }, Op::SolenoidInput(input)) => {
                Ok((if input { SOLENOID_LOW } else { SOLENOID_HIGH }, 1))
            }
            (Kind::Pump { .. }, Op::SyncPosition) => Ok((SYNC_PISTON_POS, 0)),
            (_, Op::QueryVersion) => Ok((Q_VERSION, 0)),
            _ => Err(format!("{} not supported on {}", op.label(), id.tag())),
        };

        let (func, param) = match frame {
            Ok(f) => f,
            Err(e) => {
                self.log(Level::Warn, format!("{}: {e}", id.tag()));
                return;
            }
        };
        match self.transact(id, func, param) {
            Ok(r) if r.status == 0x00 || r.status == 0xFE => {
                self.log(Level::Info, format!("{}: {} — accepted", id.tag(), op.label()));
                let cur = self.state.lock().unwrap().dev(id).value;
                let mut s = self.state.lock().unwrap();
                let d = &mut s.devices[id as usize];
                match op {
                    Op::SolenoidInput(v) => d.solenoid_input = Some(v),
                    Op::SetSpeed(v) => d.speed = Some(v),
                    Op::QueryVersion => d.version = Some(r.value),
                    Op::ValveTo(p) | Op::PumpTo(p) => d.target = Some(p),
                    Op::Home | Op::ForcedReset => d.target = Some(0),
                    Op::Aspirate(n) => d.target = cur.map(|c| c.saturating_add(n)),
                    Op::Dispense(n) => d.target = cur.map(|c| c.saturating_sub(n)),
                    Op::Stop => d.target = None,
                    Op::SyncPosition => {}
                }
                d.idle_polls = 0;
            }
            Ok(r) => self.log(
                Level::Error,
                format!("{}: {} rejected — {} (0x{:02X})", id.tag(), op.label(), protocol::status_text(r.status), r.status),
            ),
            Err(e) => self.log(Level::Error, format!("{}: {} failed — {e}", id.tag(), op.label())),
        }
    }

    fn poll_all(&mut self) {
        for id in DeviceId::ALL {
            let now = self.now();
            // Offline devices are retried every 2 s so timeouts don't stall the cycle.
            if !self.state.lock().unwrap().dev(id).online && now < self.next_retry[id as usize] {
                continue;
            }
            let value_query = if id.is_pump() { protocol::Q_PISTON_POS } else { protocol::Q_CHANNEL };
            let value = self.transact(id, value_query, 0);
            let status = match value {
                Ok(_) => self.transact(id, protocol::Q_MOTOR_STATUS, 0),
                Err(ref e) => Err(e.clone()),
            };
            let now = self.now();
            let was_online;
            let mut settled = None;
            {
                let mut s = self.state.lock().unwrap();
                let d = &mut s.devices[id as usize];
                was_online = d.last_seen.is_some() && d.online;
                if let (Ok(v), Ok(st)) = (&value, &status) {
                    let prev = d.value;
                    d.value = Some(v.value);
                    d.status = Some(st.status);
                    let busy = matches!(st.status, 0x04 | 0xFE);
                    // Direction only while the motor reports busy; a move that finished
                    // between the two queries must not keep showing as "dispensing".
                    d.motion = match (prev, busy) {
                        (_, false) => 0,
                        (Some(p), true) if v.value > p => 1,
                        (Some(p), true) if v.value < p => -1,
                        (_, true) => d.motion,
                    };
                    // Clear the pending target once the move is over. An exact
                    // match ends it at once; otherwise two idle polls do, so a
                    // device that stops somewhere else than asked (a valve
                    // reset leaves the rotor on its origin port, not on 0)
                    // does not stay MOVING for good.
                    if busy || d.target.is_none() {
                        d.idle_polls = 0;
                    } else if d.target == Some(v.value) {
                        d.target = None;
                        d.idle_polls = 0;
                    } else {
                        d.idle_polls += 1;
                        if d.idle_polls >= 2 {
                            settled = d.target.take().map(|want| (want, v.value));
                            d.idle_polls = 0;
                        }
                    }
                    d.history.push_back((now, v.value as f32));
                    while d.history.front().is_some_and(|(t, _)| now - t > HISTORY_SECS) {
                        d.history.pop_front();
                    }
                } else {
                    d.motion = 0;
                }
            }
            if let Some((want, got)) = settled {
                let unit = if id.is_pump() { "step" } else { "port" };
                self.log(Level::Info, format!("{}: stopped at {unit} {got} (asked for {want})", id.tag()));
            }
            // A sensor stop reports where the piston came to rest, which is
            // only knowable once it has: fill it in on the first poll that
            // finds PP01 idle after the trip.
            if id == DeviceId::Pp01 {
                let mut s = self.state.lock().unwrap();
                let idle = !matches!(s.dev(id).status, Some(0x04 | 0xFE)) && s.dev(id).target.is_none();
                let value = s.dev(id).value;
                if idle
                    && let Some(fired) = s.stop_fired.as_mut()
                    && fired.pump.is_none()
                {
                    fired.pump = value;
                }
            }
            match (&value, &status) {
                (Ok(_), Ok(st)) if protocol::status_is_fault(st.status) => {
                    let msg = format!("{} status: {} (0x{:02X})", id.tag(), protocol::status_text(st.status), st.status);
                    let changed = self.state.lock().unwrap().dev(id).last_error.as_deref() != Some(msg.as_str());
                    if changed {
                        self.log(Level::Warn, msg.clone());
                    }
                    self.state.lock().unwrap().devices[id as usize].last_error = Some(msg);
                }
                (Err(e), _) | (_, Err(e)) => {
                    self.next_retry[id as usize] = now + 2.0;
                    if was_online {
                        self.log(Level::Error, format!("{} went offline: {e}", id.tag()));
                    }
                }
                _ => {}
            }
        }
    }
}

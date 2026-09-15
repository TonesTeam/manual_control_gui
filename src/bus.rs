//! Single worker thread that owns the RS485 port (or the simulator), runs
//! queued device commands and polls every device for live state.

use std::collections::VecDeque;
use std::io::{Read, Write};
use std::sync::mpsc::{Receiver, RecvTimeoutError, Sender, channel};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

use crate::config::{Backend, Settings};
use crate::devices::{DeviceId, Kind};
use crate::protocol::{self, Reply};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
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

pub enum BusCmd {
    Apply(Settings),
    Device(DeviceId, Op),
    StopAll,
    HomeAll,
    SetPolling(bool),
    SetTrace(bool),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Level {
    Info,
    Warn,
    Error,
    Trace,
}

#[derive(Clone, Debug)]
pub struct LogLine {
    pub t: f64,
    pub level: Level,
    pub text: String,
}

#[derive(Clone, Debug, Default)]
pub struct DeviceLive {
    pub online: bool,
    pub status: Option<u8>,
    /// Pump: piston position in steps. Valve: current port (0 = reset).
    pub value: Option<u16>,
    pub target: Option<u16>,
    pub speed: Option<u16>,
    pub solenoid_input: Option<bool>,
    pub version: Option<u16>,
    /// +1 position rising (aspirating / valve moving), -1 falling, 0 idle.
    pub motion: i8,
    pub last_seen: Option<f64>,
    pub errors: u32,
    pub last_error: Option<String>,
    /// (t, value) samples for trend plots.
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
    pub started: Instant,
}

impl BusState {
    #[cfg(test)]
    pub fn for_tests() -> Self {
        Self {
            backend: Backend::Simulator,
            connected: true,
            connection_error: None,
            polling: false,
            trace: false,
            devices: Default::default(),
            log: VecDeque::new(),
            frames_tx: 0,
            frames_bad: 0,
            cycle_ms: 0.0,
            started: Instant::now(),
        }
    }

    pub fn dev(&self, id: DeviceId) -> &DeviceLive {
        &self.devices[id as usize]
    }

    pub fn now(&self) -> f64 {
        self.started.elapsed().as_secs_f64()
    }
}

const HISTORY_SECS: f64 = 120.0;
const LOG_LINES: usize = 800;

pub struct Bus {
    pub state: Arc<Mutex<BusState>>,
    tx: Sender<BusCmd>,
}

impl Bus {
    pub fn spawn(ctx: eframe::egui::Context, settings: Settings) -> Self {
        let state = Arc::new(Mutex::new(BusState {
            backend: settings.backend,
            connected: false,
            connection_error: None,
            polling: true,
            trace: false,
            devices: Default::default(),
            log: VecDeque::new(),
            frames_tx: 0,
            frames_bad: 0,
            cycle_ms: 0.0,
            started: Instant::now(),
        }));
        let (tx, rx) = channel();
        let worker_state = state.clone();
        thread::Builder::new()
            .name("rs485-bus".into())
            .spawn(move || Worker::new(worker_state, ctx, settings).run(rx))
            .expect("spawn bus worker");
        Self { state, tx }
    }

    pub fn send(&self, cmd: BusCmd) {
        let _ = self.tx.send(cmd);
    }

    pub fn device(&self, id: DeviceId, op: Op) {
        self.send(BusCmd::Device(id, op));
    }

    pub fn snapshot(&self) -> BusState {
        self.state.lock().unwrap().clone()
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
}

struct Worker {
    state: Arc<Mutex<BusState>>,
    ctx: eframe::egui::Context,
    settings: Settings,
    transport: Option<Box<dyn Transport>>,
    next_retry: [f64; 5],
}

impl Worker {
    fn new(state: Arc<Mutex<BusState>>, ctx: eframe::egui::Context, settings: Settings) -> Self {
        let mut w = Self { state, ctx, settings, transport: None, next_retry: [0.0; 5] };
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
        s.log.push_back(LogLine { t, level, text: text.into() });
        while s.log.len() > LOG_LINES {
            s.log.pop_front();
        }
    }

    fn connect(&mut self) {
        let result: Result<Box<dyn Transport>, String> = match self.settings.backend {
            Backend::Simulator => Ok(Box::new(SimTransport::new(&self.settings))),
            Backend::Serial => SerialTransport::open(&self.settings).map(|t| Box::new(t) as _),
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
                        self.ctx.request_repaint();
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
                self.settings = settings;
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
        }
        self.ctx.request_repaint();
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

    fn execute(&mut self, id: DeviceId, op: Op) {
        use protocol::*;
        let kind = id.kind();
        let max = self.settings.max_steps(id);
        let frame: Result<(u8, u16), String> = match (kind, op) {
            (_, Op::Stop) => Ok((STOP, 0)),
            (Kind::Pump { .. }, Op::Home) => Ok((RESET, 0)),
            (Kind::Pump { .. }, Op::ForcedReset) => Ok((FORCED_RESET, 0)),
            (Kind::Valve { .. }, Op::Home | Op::ForcedReset) => Ok((RESET, 0)),
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
                        Ok(r) if target > r.value => Ok((PUMP_ASPIRATE, target - r.value)),
                        Ok(r) if target < r.value => Ok((PUMP_DISPENSE, r.value - target)),
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
                    if !busy && d.target == Some(v.value) {
                        d.target = None;
                    }
                    d.history.push_back((now, v.value as f32));
                    while d.history.front().is_some_and(|(t, _)| now - t > HISTORY_SECS) {
                        d.history.pop_front();
                    }
                } else {
                    d.motion = 0;
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

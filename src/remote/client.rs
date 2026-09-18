//! The GUI's end of the link to `tstand_server`.
//!
//! Mirrors the server's [`BusState`] into the same `Arc<Mutex<BusState>>` the
//! local worker would have filled, so every tab, the schematic and the level
//! tracker keep reading one shape of data and never learn where it came from.
//!
//! Reconnects on its own. While the link is down the mirror is marked
//! disconnected — devices go OFFLINE rather than freezing on their last
//! reading, so nobody reads a stale piston position as live.

use std::io::BufReader;
use std::net::TcpStream;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::mpsc::{Receiver, RecvTimeoutError, Sender, channel};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

use crate::bus::{BusCmd, BusState, Level, LogLine, Wake, next_log_seq};
use crate::config::{Backend, Settings};
use crate::controller_api as api;
use crate::devices::DeviceId;
use crate::wire::{self, ClientMsg, PROTOCOL, ServerMsg, StateMsg};

const RETRY: Duration = Duration::from_secs(2);
/// Nothing from the server for this long means the link is wedged: a TCP
/// connection to a machine that has been unplugged can sit open for minutes.
const SILENCE_TIMEOUT: Duration = Duration::from_secs(10);
/// How long to let the link go quiet before asking the server if it is there.
const KEEPALIVE: Duration = Duration::from_secs(3);
/// Per-address connect budget. Several addresses may be tried in turn, so this
/// stays short enough that a dead one does not hold up a live one.
const CONNECT_TIMEOUT: Duration = Duration::from_secs(3);
/// Matches the temperature worker's own window, so a local and a remote view
/// plot the same span.
const TEMP_HISTORY_SECS: f64 = 600.0;

/// Connects to `host` and keeps `state` mirroring it until `cmds` is dropped.
///
/// Returns the channel for controller_v2 requests to tunnel; replies come back
/// on `api_reply_tx` exactly as the direct HTTP worker would have sent them.
#[allow(clippy::too_many_arguments)]
pub fn spawn(
    state: Arc<Mutex<BusState>>,
    wake: Wake,
    host: String,
    token: String,
    settings: Settings,
    cmds: Receiver<BusCmd>,
    api_reply_tx: Sender<api::Reply>,
) -> Sender<api::Request> {
    let host = wire::normalize_host(&host);
    let (api_tx, api_rx) = channel::<api::Request>();
    thread::Builder::new()
        .name("bus-client".into())
        .spawn(move || {
            Client { state, wake, host, token, settings, api_reply_tx }.run(cmds, api_rx);
        })
        .expect("spawn bus client");
    api_tx
}

struct Client {
    state: Arc<Mutex<BusState>>,
    wake: Wake,
    host: String,
    token: String,
    settings: Settings,
    api_reply_tx: Sender<api::Reply>,
}

impl Client {
    fn log(&self, level: Level, text: impl Into<String>) {
        let mut s = self.state.lock().unwrap();
        let t = s.now();
        s.push_log(LogLine { t, level, text: text.into(), seq: next_log_seq() });
    }

    fn set_down(&self, error: Option<String>) {
        let mut s = self.state.lock().unwrap();
        s.connected = false;
        s.connection_error = error;
        s.remote = Some(self.host.clone());
        // Do not leave the last reading looking live.
        for d in &mut s.devices {
            d.online = false;
            d.motion = 0;
            d.status = None;
        }
        // The board is on the rig, not here: with no link there is no reading.
        s.temp.go_offline(Some("no link to the rig".into()));
        s.sensors.go_offline(Some("no link to the rig".into()));
    }

    /// Turns a write failure into something an operator can act on.
    ///
    /// When the server rejects a handshake it answers and hangs up, so the
    /// next write fails with "broken pipe" — true, and useless. Its own
    /// reason is still in flight on the reader thread; prefer that.
    fn explain(&self, error: String, msgs: &Receiver<ServerMsg>) -> String {
        let deadline = Instant::now() + Duration::from_millis(250);
        while Instant::now() < deadline {
            while let Ok(msg) = msgs.try_recv() {
                if let ServerMsg::Denied(why) = msg {
                    return format!("{} refused the connection: {why}", self.host);
                }
            }
            thread::sleep(Duration::from_millis(10));
        }
        error
    }

    fn run(mut self, cmds: Receiver<BusCmd>, api_rx: Receiver<api::Request>) {
        self.state.lock().unwrap().remote = Some(self.host.clone());
        self.log(Level::Info, format!("remote control: connecting to {}", self.host));
        loop {
            match self.session(&cmds, &api_rx) {
                // The GUI dropped the command channel: this backend is done.
                Ok(Stop::Shutdown) => return,
                Ok(Stop::Disconnected(why)) => {
                    self.set_down(Some(why.clone()));
                    self.log(Level::Warn, format!("{} disconnected: {why}", self.host));
                }
                Err(e) => {
                    let first = self.state.lock().unwrap().connection_error.as_deref() != Some(e.as_str());
                    self.set_down(Some(e.clone()));
                    if first {
                        self.log(Level::Error, e);
                    }
                }
            }
            self.wake.call();
            // Keep draining commands while down so the GUI's channel does not
            // grow without bound; they cannot be delivered, so say so once.
            let deadline = Instant::now() + RETRY;
            let mut dropped = 0usize;
            loop {
                let wait = deadline.saturating_duration_since(Instant::now());
                match cmds.recv_timeout(wait) {
                    Ok(_) => dropped += 1,
                    Err(RecvTimeoutError::Timeout) => break,
                    Err(RecvTimeoutError::Disconnected) => return,
                }
            }
            while api_rx.try_recv().is_ok() {}
            if dropped > 0 {
                self.log(Level::Warn, format!("{dropped} command(s) dropped: no link to {}", self.host));
            }
        }
    }

    /// One connection, from TCP connect to whatever ends it.
    fn session(&mut self, cmds: &Receiver<BusCmd>, api_rx: &Receiver<api::Request>) -> Result<Stop, String> {
        let stream = connect(&self.host)?;
        stream.set_nodelay(true).ok();
        let mut writer = stream.try_clone().map_err(|e| e.to_string())?;
        let mut reader = BufReader::new(stream);

        let client = format!(
            "{}@{}",
            std::env::var("USER").or_else(|_| std::env::var("USERNAME")).unwrap_or_else(|_| "gui".into()),
            hostname(),
        );
        wire::write(
            &mut writer,
            &ClientMsg::Hello { protocol: PROTOCOL, token: self.token.clone(), client },
        )?;

        // The reader runs on its own thread so a silent server cannot block
        // the command path, and hands messages over a channel.
        let (msg_tx, msg_rx) = channel::<ServerMsg>();
        let reader_handle = thread::Builder::new()
            .name("bus-client-rx".into())
            .spawn(move || {
                while let Ok(Some(msg)) = wire::read::<ServerMsg>(&mut reader) {
                    if msg_tx.send(msg).is_err() {
                        return;
                    }
                }
            })
            .map_err(|e| e.to_string())?;
        // Nothing here needs to join the reader: it ends when the socket does.
        drop(reader_handle);

        // Every write after the handshake goes through this, so the reason a
        // server gave for hanging up wins over the symptom.
        macro_rules! send {
            ($msg:expr) => {
                wire::write(&mut writer, &$msg).map_err(|e| self.explain(e, &msg_rx))?
            };
        }

        let mut last_heard = Instant::now();
        let mut last_ping = Instant::now();
        let mut welcomed = false;
        let mut next_api_id = ApiIds::default();
        let mut pending: Vec<(u64, api::Request)> = Vec::new();

        loop {
            // Anything from the server first: it is what keeps the mirror fresh.
            let mut got = false;
            while let Ok(msg) = msg_rx.try_recv() {
                got = true;
                last_heard = Instant::now();
                match msg {
                    ServerMsg::Welcome { protocol, server, settings, clients } => {
                        if protocol != PROTOCOL {
                            return Err(format!(
                                "{} speaks protocol {protocol}, this build speaks {PROTOCOL} — update both ends",
                                self.host
                            ));
                        }
                        welcomed = true;
                        {
                            let mut s = self.state.lock().unwrap();
                            s.connected = true;
                            s.connection_error = None;
                            s.backend = settings.backend;
                        }
                        let others = clients.saturating_sub(1);
                        let also = if others > 0 {
                            format!(" ({others} other client(s) connected)")
                        } else {
                            String::new()
                        };
                        self.log(Level::Info, format!("remote control: {server} at {}{also}", self.host));
                        // Adopt the rig's port, addresses and calibration
                        // rather than pushing ours: the hardware is over
                        // there, and another operator may already have set it
                        // up. Nothing is applied until someone presses
                        // "Apply & save".
                        self.settings.adopt_server_fields(&settings);
                        publish_server_settings(&settings);
                        send!(ClientMsg::ListPorts);
                    }
                    ServerMsg::Denied(why) => return Err(format!("{} refused the connection: {why}", self.host)),
                    ServerMsg::State(msg) => self.apply_state(*msg),
                    ServerMsg::Log(lines) => {
                        let mut s = self.state.lock().unwrap();
                        for line in lines {
                            s.push_log(line);
                        }
                    }
                    ServerMsg::Ports(ports) => {
                        *REMOTE_PORTS.lock().unwrap() = ports;
                    }
                    ServerMsg::ApiReply { id, result } => {
                        if let Some(i) = pending.iter().position(|(pid, _)| *pid == id) {
                            let (_, request) = pending.remove(i);
                            let _ = self.api_reply_tx.send(api::Reply {
                                summary: request.describe(),
                                request,
                                result,
                            });
                        }
                    }
                    ServerMsg::Pong => {}
                }
            }
            if got {
                self.wake.call();
            }

            // Then anything the GUI wants to send.
            while let Ok(req) = api_rx.try_recv() {
                let id = next_api_id.next();
                pending.push((id, req.clone()));
                // Bound the book-keeping if the server never answers.
                if pending.len() > 64 {
                    pending.remove(0);
                }
                send!(ClientMsg::Api { id, request: req });
            }

            match cmds.recv_timeout(Duration::from_millis(50)) {
                Ok(cmd) => {
                    let cmd = match cmd {
                        // Settings the server cares about, with the backend
                        // rewritten to what it should actually run.
                        BusCmd::Apply(s) => {
                            self.settings = (*s).clone();
                            BusCmd::Apply(Box::new(Settings { backend: s.server_backend, ..*s }))
                        }
                        other => other,
                    };
                    send!(ClientMsg::Cmd(cmd));
                    // Settings may have changed the port list on the server.
                    send!(ClientMsg::ListPorts);
                }
                Err(RecvTimeoutError::Timeout) => {}
                Err(RecvTimeoutError::Disconnected) => return Ok(Stop::Shutdown),
            }

            if last_heard.elapsed() > SILENCE_TIMEOUT {
                return Ok(Stop::Disconnected(if welcomed {
                    format!("no state for {}s", SILENCE_TIMEOUT.as_secs())
                } else {
                    "no reply to the handshake".to_string()
                }));
            }
            // A quiet server is normal when polling is off, so prod it rather
            // than assume the link died. Never test the incoming queue here:
            // `try_recv` would take a message and drop it on the floor.
            if last_heard.elapsed() > KEEPALIVE && last_ping.elapsed() > KEEPALIVE {
                last_ping = Instant::now();
                send!(ClientMsg::Ping);
            }
        }
    }

    /// Folds a server snapshot into the local mirror, keeping the parts the
    /// wire does not carry: the log ring, and the trend history this rebuilds
    /// sample by sample.
    fn apply_state(&self, msg: StateMsg) {
        let mut s = self.state.lock().unwrap();
        // Line up our clock with the server's, so log timestamps and history
        // share one origin. Recomputed each time: it is cheap and self-correcting.
        if let Some(started) = Instant::now().checked_sub(Duration::from_secs_f64(msg.uptime.max(0.0))) {
            s.started = started;
        }
        s.backend = msg.backend;
        s.connected = msg.connected;
        s.connection_error = msg.connection_error;
        s.polling = msg.polling;
        s.trace = msg.trace;
        s.frames_tx = msg.frames_tx;
        s.frames_bad = msg.frames_bad;
        s.cycle_ms = msg.cycle_ms;
        // The temperature trend is rebuilt here for the same reason the pumps'
        // is: the samples would dwarf the snapshot on the wire.
        // A setpoint step is worth a sample of its own: the target trace would
        // otherwise jump between two far-apart readings and lose the step edge.
        let temp_changed = s.temp.temperature_c != msg.temp.temperature_c || s.temp.setpoint_c != msg.temp.setpoint_c;
        let temp_history = std::mem::take(&mut s.temp.history);
        s.temp = msg.temp;
        s.temp.history = temp_history;
        s.sensors = msg.sensors;
        s.armed_stop = msg.armed_stop;
        s.stop_fired = msg.stop_fired;
        if let Some(c) = s.temp.temperature_c
            && (temp_changed || s.temp.history.is_empty())
        {
            let target = s.temp.setpoint_c;
            s.temp.push_sample(msg.uptime, c, target, TEMP_HISTORY_SECS);
        }
        for (i, incoming) in msg.devices.into_iter().enumerate() {
            let id = DeviceId::ALL[i];
            let changed = s.devices[i].value != incoming.value;
            let history = std::mem::take(&mut s.devices[i].history);
            s.devices[i] = incoming;
            s.devices[i].history = history;
            if let Some(v) = s.devices[i].value
                && (changed || s.devices[i].history.is_empty())
            {
                s.push_sample(id, msg.uptime, v as f32);
            }
        }
    }
}

enum Stop {
    /// The GUI switched backends or is closing.
    Shutdown,
    /// The link failed; reconnect.
    Disconnected(String),
}

#[derive(Default)]
struct ApiIds(AtomicU64);

impl ApiIds {
    fn next(&mut self) -> u64 {
        self.0.fetch_add(1, Ordering::Relaxed)
    }
}

/// Serial ports the server last reported, for the Settings combo box. A global
/// because the Settings tab asks for it far from any `Client`, and there is
/// only ever one remote connection.
static REMOTE_PORTS: Mutex<Vec<String>> = Mutex::new(Vec::new());

pub fn remote_ports() -> Vec<String> {
    REMOTE_PORTS.lock().unwrap().clone()
}

/// The settings a freshly connected server reported, tagged with a generation
/// so the Settings tab folds each one in exactly once instead of fighting the
/// user for the text fields on every frame.
static SERVER_SETTINGS: Mutex<Option<(u64, Settings)>> = Mutex::new(None);
static SERVER_SETTINGS_GEN: AtomicU64 = AtomicU64::new(0);

fn publish_server_settings(settings: &Settings) {
    let generation = SERVER_SETTINGS_GEN.fetch_add(1, Ordering::Relaxed) + 1;
    *SERVER_SETTINGS.lock().unwrap() = Some((generation, settings.clone()));
}

/// Returns the server's settings when they are newer than `seen`, along with
/// the generation to remember.
pub fn server_settings_since(seen: u64) -> Option<(u64, Settings)> {
    SERVER_SETTINGS.lock().unwrap().clone().filter(|&(generation, _)| generation > seen)
}

fn hostname() -> String {
    std::fs::read_to_string("/etc/hostname")
        .ok()
        .map(|h| h.trim().to_string())
        .filter(|h| !h.is_empty())
        .or_else(|| std::env::var("COMPUTERNAME").ok())
        .or_else(|| std::env::var("HOSTNAME").ok())
        .unwrap_or_else(|| "pc".into())
}

/// Opens a connection to `host`, trying every address it resolves to.
///
/// This matters for the `.local` name the rig is usually reached by: mDNS
/// answers with an IPv6 address first and an IPv4 one second, while the server
/// binds `0.0.0.0`. Stopping at the first address would refuse every time.
fn connect(host: &str) -> Result<TcpStream, String> {
    use std::net::ToSocketAddrs;
    let addrs: Vec<_> = host
        .to_socket_addrs()
        .map_err(|e| format!("cannot resolve {host}: {e}"))?
        .collect();
    if addrs.is_empty() {
        return Err(format!("{host} resolves to no addresses"));
    }
    let mut errors = Vec::new();
    for addr in &addrs {
        match TcpStream::connect_timeout(addr, CONNECT_TIMEOUT) {
            Ok(stream) => return Ok(stream),
            Err(e) => errors.push(format!("{addr}: {e}")),
        }
    }
    Err(format!("cannot reach {host} ({})", errors.join("; ")))
}

/// The backend a fresh remote mirror should claim before the server has said
/// anything. Serial is the safe guess: it keeps the GUI from offering the
/// simulator-only demo against real hardware.
pub const ASSUMED_BACKEND: Backend = Backend::Serial;

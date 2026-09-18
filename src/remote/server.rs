//! The rig's end: owns the RS485 adapter and serves every GUI that connects.
//!
//! One thread accepts, two per connection (read and write), one broadcasts
//! state to all of them, and underneath sits exactly the same [`Bus`] the GUI
//! used to run in-process. Several GUIs may watch at once; any of them can
//! send commands, and they all see the result, because the bus — not the
//! client — holds the truth about the hardware.

use std::io::BufReader;
use std::net::{TcpListener, TcpStream};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::mpsc::{SyncSender, TrySendError, sync_channel};
use std::sync::{Arc, Condvar, Mutex};
use std::thread;
use std::time::Duration;

use crate::bus::{Bus, BusCmd, Level, LogLine, Wake, next_log_seq};
use crate::config::{Backend, SERVER_SETTINGS_FILE, Settings};
use crate::controller_api as api;
use crate::wire::{self, ClientMsg, PROTOCOL, ServerMsg, StateMsg};

/// How often state goes out when nothing has changed. A change wakes the
/// broadcaster immediately, so this is only a floor for "still alive".
const TICK: Duration = Duration::from_millis(200);
/// Queue depth per client. A GUI that cannot keep up with this is not going to
/// catch up, so it gets dropped rather than growing the server's memory.
const QUEUE: usize = 64;

pub struct Config {
    pub bind: String,
    pub token: String,
    pub controller_url: String,
    pub settings: Settings,
}

struct Peer {
    id: u64,
    name: String,
    tx: SyncSender<ServerMsg>,
    /// Highest log line this peer has been sent; the next push starts after it.
    last_log_seq: u64,
}

/// Set whenever there is something new to publish; the broadcaster waits on it.
type Dirty = Arc<(Mutex<bool>, Condvar)>;

fn poke_flag(dirty: &Dirty) {
    *dirty.0.lock().unwrap() = true;
    dirty.1.notify_all();
}

struct Shared {
    bus: Bus,
    token: String,
    controller_url: Mutex<String>,
    peers: Mutex<Vec<Peer>>,
    dirty: Dirty,
    next_id: AtomicU64,
    running: AtomicBool,
}

impl Shared {
    fn poke(&self) {
        poke_flag(&self.dirty);
    }

    fn log(&self, level: Level, text: impl Into<String>) {
        let mut s = self.bus.state.lock().unwrap();
        let t = s.now();
        s.push_log(LogLine { t, level, text: text.into(), seq: next_log_seq() });
    }
}

fn level_tag(level: Level) -> &'static str {
    match level {
        Level::Info => "info",
        Level::Warn => "warn",
        Level::Error => "error",
        Level::Trace => "trace",
    }
}

/// Starts the bus, binds the socket and serves until the process is killed.
pub fn run(config: Config) -> Result<(), String> {
    let listener = TcpListener::bind(&config.bind)
        .map_err(|e| format!("cannot bind {}: {e}", config.bind))?;
    serve_on(listener, config)
}

/// Serves on an already-bound listener. Split out so a caller — a test, or a
/// socket-activated service — can learn the port before anything connects.
pub fn serve_on(listener: TcpListener, config: Config) -> Result<(), String> {
    let Config { bind, token, controller_url, mut settings } = config;
    // A server drives hardware; it never chains to another server.
    if settings.backend == Backend::Remote {
        settings.backend = Backend::Simulator;
    }
    settings.controller_url = controller_url.clone();

    let local = listener.local_addr().map(|a| a.to_string()).unwrap_or(bind);

    // The bus wakes the broadcaster through the same flag the peer threads
    // set, so a command's effect reaches every screen without waiting for TICK.
    let dirty: Dirty = Arc::new((Mutex::new(false), Condvar::new()));
    let bus = {
        let dirty = dirty.clone();
        Bus::spawn(Wake::new(move || poke_flag(&dirty)), settings.clone())
    };
    let shared = Arc::new(Shared {
        bus,
        token,
        controller_url: Mutex::new(controller_url),
        peers: Mutex::new(Vec::new()),
        dirty,
        next_id: AtomicU64::new(1),
        running: AtomicBool::new(true),
    });

    shared.log(
        Level::Info,
        format!(
            "tstand_server listening on {local} · backend {:?}{} · controller {}",
            settings.backend,
            if settings.backend == Backend::Serial { format!(" {}", settings.port) } else { String::new() },
            settings.controller_url,
        ),
    );
    if shared.token.is_empty() {
        shared.log(Level::Warn, "no --token set: anyone who can reach this port can move the pumps");
    }

    {
        let shared = shared.clone();
        thread::Builder::new()
            .name("broadcast".into())
            .spawn(move || broadcast(shared))
            .map_err(|e| e.to_string())?;
    }

    for stream in listener.incoming() {
        match stream {
            Ok(stream) => {
                let shared = shared.clone();
                let _ = thread::Builder::new()
                    .name("peer".into())
                    .spawn(move || {
                        if let Err(e) = serve(&shared, stream) {
                            shared.log(Level::Warn, format!("client error: {e}"));
                        }
                    });
            }
            Err(e) => shared.log(Level::Warn, format!("accept failed: {e}")),
        }
    }
    shared.running.store(false, Ordering::Relaxed);
    Ok(())
}

/// Pushes state and new log lines to every connected client.
fn broadcast(shared: Arc<Shared>) {
    // Everything the rig logs — valve moves, faults, temperature commands —
    // is written once to stderr as well as pushed to the clients. Without
    // this, a server running under systemd would journal its own connections
    // and nothing about the hardware, which is the half you need at 2 a.m.
    let mut mirrored = shared.bus.state.lock().unwrap().log.back().map(|l| l.seq).unwrap_or(0);
    loop {
        {
            // Wait for a change, but never longer than TICK: clients use the
            // gap between snapshots to notice a dead server.
            let (lock, cv) = &*shared.dirty;
            let mut dirty = lock.lock().unwrap();
            while !*dirty {
                let (next, timeout) = cv.wait_timeout(dirty, TICK).unwrap();
                dirty = next;
                if timeout.timed_out() {
                    break;
                }
            }
            *dirty = false;
        }
        if !shared.running.load(Ordering::Relaxed) {
            return;
        }

        let state = shared.bus.snapshot();
        let fresh_for_journal = state.log.iter().filter(|l| l.seq > mirrored);
        let mut highest = mirrored;
        for line in fresh_for_journal {
            // Trace is every RS485 frame; far too much for a journal.
            if line.level != Level::Trace {
                eprintln!("[{}] {}", level_tag(line.level), line.text);
            }
            highest = line.seq;
        }
        mirrored = highest;
        let msg = Box::new(StateMsg {
            backend: state.backend,
            connected: state.connected,
            connection_error: state.connection_error.clone(),
            polling: state.polling,
            trace: state.trace,
            devices: state.devices.clone(),
            frames_tx: state.frames_tx,
            frames_bad: state.frames_bad,
            cycle_ms: state.cycle_ms,
            temp: state.temp.clone(),
            sensors: state.sensors.clone(),
            armed_stop: state.armed_stop,
            stop_fired: state.stop_fired,
            uptime: state.now(),
        });

        let mut peers = shared.peers.lock().unwrap();
        peers.retain_mut(|peer| {
            let fresh: Vec<LogLine> =
                state.log.iter().filter(|l| l.seq > peer.last_log_seq).cloned().collect();
            if let Some(last) = fresh.last() {
                peer.last_log_seq = last.seq;
            }
            let mut ok = send(peer, ServerMsg::State(msg.clone()));
            if ok && !fresh.is_empty() {
                ok = send(peer, ServerMsg::Log(fresh));
            }
            ok
        });
    }
}

/// True while the peer is keeping up. A full queue means it is not.
fn send(peer: &Peer, msg: ServerMsg) -> bool {
    match peer.tx.try_send(msg) {
        Ok(()) => true,
        Err(TrySendError::Full(_)) => {
            eprintln!("[warn] dropping {} ({}): too far behind", peer.name, peer.id);
            false
        }
        Err(TrySendError::Disconnected(_)) => false,
    }
}

/// One client, from handshake to hang-up.
fn serve(shared: &Arc<Shared>, stream: TcpStream) -> Result<(), String> {
    stream.set_nodelay(true).ok();
    let addr = stream.peer_addr().map(|a| a.to_string()).unwrap_or_else(|_| "?".into());
    let mut writer = stream.try_clone().map_err(|e| e.to_string())?;
    let mut reader = BufReader::new(stream);

    // The handshake must not hold a slot open forever.
    reader.get_ref().set_read_timeout(Some(Duration::from_secs(10))).ok();
    let hello = wire::read::<ClientMsg>(&mut reader)?;
    let (protocol, got_token, name) = match hello {
        Some(ClientMsg::Hello { protocol, token, client }) => (protocol, token, client),
        Some(_) => {
            let _ = wire::write(&mut writer, &ServerMsg::Denied("expected Hello first".into()));
            return Ok(());
        }
        None => return Ok(()),
    };
    reader.get_ref().set_read_timeout(None).ok();

    if protocol != PROTOCOL {
        let why = format!("protocol {protocol} not supported, this server speaks {PROTOCOL}");
        shared.log(Level::Warn, format!("{addr} refused: {why}"));
        let _ = wire::write(&mut writer, &ServerMsg::Denied(why));
        return Ok(());
    }
    // Constant-ish comparison is pointless over a LAN token this short, but do
    // not leak whether a token was expected at all.
    if got_token != shared.token {
        shared.log(Level::Warn, format!("{addr} refused: bad token"));
        let _ = wire::write(&mut writer, &ServerMsg::Denied("bad token".into()));
        return Ok(());
    }

    let id = shared.next_id.fetch_add(1, Ordering::Relaxed);
    let name = format!("{name} [{addr}]");
    let (tx, rx) = sync_channel::<ServerMsg>(QUEUE);

    // Writer thread: everything to this client goes through the queue, so no
    // other thread can block on a slow socket.
    let writer_name = name.clone();
    let writer_handle = thread::Builder::new()
        .name("peer-tx".into())
        .spawn(move || {
            for msg in rx {
                if wire::write(&mut writer, &msg).is_err() {
                    break;
                }
            }
            let _ = writer_name;
        })
        .map_err(|e| e.to_string())?;

    let settings = shared.bus.settings();
    let clients = {
        let mut peers = shared.peers.lock().unwrap();
        // Start at the current end of the log: a new client gets fresh lines,
        // not a replay of everything since the server booted.
        let last_log_seq = shared.bus.state.lock().unwrap().log.back().map(|l| l.seq).unwrap_or(0);
        peers.push(Peer { id, name: name.clone(), tx: tx.clone(), last_log_seq });
        peers.len()
    };
    shared.log(Level::Info, format!("{name} connected ({clients} client(s))"));

    let welcome = ServerMsg::Welcome {
        protocol: PROTOCOL,
        server: hostname(),
        settings: Box::new(settings),
        clients,
    };
    let _ = tx.try_send(welcome);
    let _ = tx.try_send(ServerMsg::Ports(list_ports()));
    shared.poke();

    // Read until the client goes away.
    let result = read_loop(shared, &mut reader, &tx);

    shared.peers.lock().unwrap().retain(|p| p.id != id);
    drop(tx);
    let _ = writer_handle.join();
    let left = shared.peers.lock().unwrap().len();
    shared.log(Level::Info, format!("{name} disconnected ({left} client(s) left)"));
    result
}

fn read_loop(
    shared: &Arc<Shared>,
    reader: &mut BufReader<TcpStream>,
    tx: &SyncSender<ServerMsg>,
) -> Result<(), String> {
    while let Some(msg) = wire::read::<ClientMsg>(reader)? {
        match msg {
            ClientMsg::Hello { .. } => {}
            ClientMsg::Ping => {
                let _ = tx.try_send(ServerMsg::Pong);
            }
            ClientMsg::ListPorts => {
                let _ = tx.try_send(ServerMsg::Ports(list_ports()));
            }
            ClientMsg::Cmd(cmd) => {
                if let BusCmd::Apply(settings) = &cmd {
                    *shared.controller_url.lock().unwrap() = settings.controller_url.clone();
                    if let Err(e) = settings.save_to(SERVER_SETTINGS_FILE) {
                        shared.log(Level::Warn, format!("could not save settings: {e}"));
                    }
                }
                shared.bus.send(cmd);
            }
            ClientMsg::Api { id, request } => {
                // On its own thread: an unreachable controller blocks for
                // seconds, and the rig must stay controllable meanwhile.
                let host = shared.controller_url.lock().unwrap().clone();
                let tx = tx.clone();
                let _ = thread::Builder::new().name("api-proxy".into()).spawn(move || {
                    let result = api::execute(&host, &request);
                    let _ = tx.try_send(ServerMsg::ApiReply { id, result });
                });
            }
        }
        shared.poke();
    }
    Ok(())
}

pub fn list_ports() -> Vec<String> {
    serialport::available_ports()
        .map(|ports| ports.into_iter().map(|p| p.port_name).collect())
        .unwrap_or_default()
}

fn hostname() -> String {
    std::fs::read_to_string("/etc/hostname")
        .ok()
        .map(|h| h.trim().to_string())
        .filter(|h| !h.is_empty())
        .unwrap_or_else(|| "tstand".into())
}

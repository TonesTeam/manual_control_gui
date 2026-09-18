//! The split rig, end to end: a `tstand_server` on a loopback port and a `Bus`
//! in `Remote` mode talking to it, which is exactly what the GUI runs.
//!
//! The server drives the simulator, so these exercise the link rather than the
//! hardware: a command typed on the PC reaches the devices, and their state
//! comes back to the PC's mirror.

use std::net::TcpListener;
use std::sync::mpsc::channel;
use std::thread;
use std::time::{Duration, Instant};

use tstand_controler::bus::{Bus, BusCmd, BusState, DevState, Op, Wake};
use tstand_controler::config::{Backend, Settings};
use tstand_controler::devices::DeviceId;
use tstand_controler::remote::server::{self, Config};

/// Starts a server on a free loopback port and returns `host:port`.
fn start_server(token: &str) -> String {
    let listener = TcpListener::bind("127.0.0.1:0").expect("bind loopback");
    let addr = listener.local_addr().expect("local addr").to_string();
    let config = Config {
        bind: addr.clone(),
        token: token.to_string(),
        controller_url: "127.0.0.1:1".to_string(), // nothing listens; unused here
        settings: Settings { backend: Backend::Simulator, ..Settings::default() },
    };
    thread::spawn(move || {
        let _ = server::serve_on(listener, config);
    });
    addr
}

fn remote_bus(host: &str, token: &str) -> Bus {
    Bus::spawn(
        Wake::none(),
        Settings {
            backend: Backend::Remote,
            server_host: host.to_string(),
            server_token: token.to_string(),
            server_backend: Backend::Simulator,
            ..Settings::default()
        },
    )
}

/// Polls `bus` until `done` is happy, and fails the test with what it saw
/// instead if that has not happened within `secs`.
#[track_caller]
fn wait_for(bus: &Bus, secs: u64, expected: &str, done: impl Fn(&BusState) -> bool) -> BusState {
    let deadline = Instant::now() + Duration::from_secs(secs);
    loop {
        let state = bus.snapshot();
        if done(&state) {
            return state;
        }
        if Instant::now() >= deadline {
            panic!(
                "timed out after {secs}s waiting for {expected}\n  \
                 connected={} remote={:?} error={:?}\n  values={:?}\n  log={:?}",
                state.connected,
                state.remote,
                state.connection_error,
                state.devices.iter().map(|d| d.value).collect::<Vec<_>>(),
                state.log.iter().map(|l| &l.text).collect::<Vec<_>>(),
            );
        }
        thread::sleep(Duration::from_millis(25));
    }
}

#[test]
fn a_command_typed_on_the_pc_moves_a_device_on_the_server() {
    let host = start_server("s3cret");
    let bus = remote_bus(&host, "s3cret");

    let state = wait_for(&bus, 10, "the client to connect and see the server's devices online", |s| {
        s.connected && s.dev(DeviceId::Sv02).online
    });
    assert_eq!(state.remote.as_deref(), Some(host.as_str()), "the mirror should name its server");
    assert_eq!(state.backend, Backend::Simulator, "the GUI should show what the server drives");

    // Port 10 is the fitted slot's fill line — one the rig actually has.
    bus.device(DeviceId::Sv02, Op::ValveTo(10));
    let state = wait_for(&bus, 10, "SV02 to reach port 10 and report it back", |s| {
        s.dev(DeviceId::Sv02).value == Some(10)
    });
    assert_eq!(state.dev(DeviceId::Sv02).state(), DevState::Ready);

    // The server's own log reaches the PC, so faults are visible where the
    // operator is rather than only in the Pi's journal.
    assert!(
        state.log.iter().any(|l| l.text.contains("SV02") && l.text.contains("port 10")),
        "expected the switch in the mirrored log, got: {:?}",
        state.log.iter().map(|l| &l.text).collect::<Vec<_>>()
    );
}

#[test]
fn a_pump_move_streams_back_as_it_runs() {
    let host = start_server("");
    let bus = remote_bus(&host, "");
    wait_for(&bus, 10, "a connection", |s| s.connected && s.dev(DeviceId::Pp01).online);

    bus.device(DeviceId::Pp01, Op::SetSpeed(400));
    bus.device(DeviceId::Pp01, Op::Aspirate(1200));

    // Caught mid-stroke: the mirror shows motion, not just the end state.
    wait_for(&bus, 10, "PP01 to report MOVING while it travels", |s| {
        s.dev(DeviceId::Pp01).state() == DevState::Moving
    });

    let state = wait_for(&bus, 30, "PP01 to arrive at 1200 steps", |s| {
        s.dev(DeviceId::Pp01).value == Some(1200) && s.dev(DeviceId::Pp01).state() == DevState::Ready
    });
    assert!(
        state.dev(DeviceId::Pp01).history.len() > 1,
        "the client should rebuild a trend from the values it received"
    );
}

#[test]
fn the_wrong_token_is_refused_and_the_mirror_says_so() {
    let host = start_server("right");
    let bus = remote_bus(&host, "wrong");

    let state = wait_for(&bus, 10, "a bad token to surface as an error rather than a silent hang", |s| {
        s.connection_error.is_some()
    });
    assert!(!state.connected);
    let why = state.connection_error.unwrap_or_default();
    assert!(why.contains("refused"), "expected a refusal, got {why:?}");
    assert!(
        state.devices.iter().all(|d| !d.online),
        "nothing may look online when there is no link"
    );
}

#[test]
fn losing_the_server_marks_every_device_offline() {
    let host = start_server("");
    let bus = remote_bus(&host, "");
    wait_for(&bus, 10, "every device online", |s| s.connected && s.devices.iter().all(|d| d.online));

    // Switching the GUI to its own simulator tears the link down, which is the
    // same path a dropped connection takes: the mirror must not keep showing
    // the rig's last reading as live.
    bus.send(BusCmd::Apply(Box::new(Settings { backend: Backend::Simulator, ..Settings::default() })));
    let state = wait_for(&bus, 10, "the mirror to stop being remote", |s| s.remote.is_none());
    assert!(
        state.devices.iter().all(|d| d.value.is_none()),
        "readings from the old backend must not survive the switch"
    );
}

#[test]
fn several_guis_see_one_rig() {
    let host = start_server("");
    let a = remote_bus(&host, "");
    let b = remote_bus(&host, "");
    wait_for(&a, 10, "the first client to connect", |s| s.connected && s.dev(DeviceId::Sv03).online);
    wait_for(&b, 10, "the second client to connect", |s| s.connected && s.dev(DeviceId::Sv03).online);

    // One operator switches a valve; the other's screen follows, because the
    // server holds the state and neither client guesses at it.
    a.device(DeviceId::Sv03, Op::ValveTo(1));
    wait_for(&b, 10, "the second GUI to see what the first one did", |s| {
        s.dev(DeviceId::Sv03).value == Some(1)
    });
}

#[test]
fn the_api_channel_reports_an_unreachable_controller() {
    let host = start_server("");
    let bus = remote_bus(&host, "");
    wait_for(&bus, 10, "a connection", |s| s.connected);

    // The server proxies to a port nothing listens on, so this exercises the
    // round trip: request out over the bus link, HTTP failure back on the same
    // reply channel the direct route uses.
    bus.api("unused", tstand_controler::controller_api::Request::GetStatus);
    let deadline = Instant::now() + Duration::from_secs(10);
    let reply = loop {
        if let Some(reply) = bus.next_api_reply() {
            break reply;
        }
        assert!(Instant::now() < deadline, "no reply came back from the proxy");
        thread::sleep(Duration::from_millis(25));
    };
    assert!(reply.result.is_err(), "expected the unreachable controller to be reported");
    assert!(reply.summary.contains("/status"), "the reply should name its request: {}", reply.summary);
}

#[test]
fn a_client_survives_the_server_starting_late() {
    // Reserve a port, but do not serve on it yet: the GUI is often open before
    // the rig's server is.
    let listener = TcpListener::bind("127.0.0.1:0").expect("bind");
    let addr = listener.local_addr().unwrap().to_string();
    drop(listener);

    let bus = remote_bus(&addr, "");
    wait_for(&bus, 10, "a report that nothing is listening yet", |s| s.connection_error.is_some());

    let (ready_tx, ready_rx) = channel();
    let addr2 = addr.clone();
    thread::spawn(move || {
        let listener = loop {
            match TcpListener::bind(&addr2) {
                Ok(l) => break l,
                Err(_) => thread::sleep(Duration::from_millis(50)),
            }
        };
        let _ = ready_tx.send(());
        let _ = server::serve_on(
            listener,
            Config {
                bind: addr2,
                token: String::new(),
                controller_url: "127.0.0.1:1".into(),
                settings: Settings { backend: Backend::Simulator, ..Settings::default() },
            },
        );
    });
    ready_rx.recv_timeout(Duration::from_secs(10)).expect("server should come up");

    wait_for(&bus, 20, "the client to reconnect on its own once the server appears", |s| s.connected);
}

#[test]
fn a_host_that_resolves_to_several_addresses_still_connects() {
    // The rig is reached by a `.local` name, and mDNS answers with an IPv6
    // address before the IPv4 one while the server binds 0.0.0.0. A client
    // that tried only the first address would never reach it, so serve on
    // IPv4 loopback alone and connect by a name that offers both.
    let listener = TcpListener::bind("127.0.0.1:0").expect("bind IPv4 loopback");
    let port = listener.local_addr().unwrap().port();

    use std::net::ToSocketAddrs;
    let resolved: Vec<_> = format!("localhost:{port}").to_socket_addrs().map(|a| a.collect()).unwrap_or_default();
    if resolved.len() < 2 || resolved[0].is_ipv4() {
        // Nothing to prove on a host where `localhost` is IPv4-only.
        return;
    }

    let config = Config {
        bind: format!("127.0.0.1:{port}"),
        token: String::new(),
        controller_url: "127.0.0.1:1".into(),
        settings: Settings { backend: Backend::Simulator, ..Settings::default() },
    };
    thread::spawn(move || {
        let _ = server::serve_on(listener, config);
    });

    let bus = remote_bus(&format!("localhost:{port}"), "");
    wait_for(&bus, 15, "a connection made past the first, dead address", |s| s.connected);
}

#[test]
fn the_server_refuses_a_port_the_rig_does_not_have() {
    let host = start_server("");
    let bus = remote_bus(&host, "");
    wait_for(&bus, 10, "a connection", |s| s.connected && s.dev(DeviceId::Sv02).online);

    // Park somewhere real, so a refusal is visible as "did not move".
    bus.device(DeviceId::Sv02, Op::ValveTo(10));
    wait_for(&bus, 10, "SV02 on its slot port", |s| s.dev(DeviceId::Sv02).value == Some(10));

    // Port 15 is another slot's fill line: drawn on the diagram, capped here.
    // The guard lives on the server, so this is refused even though the
    // command came straight from the bus rather than through a greyed button.
    bus.device(DeviceId::Sv02, Op::ValveTo(15));
    let state = wait_for(&bus, 10, "the refusal to be logged", |s| {
        s.log.iter().any(|l| l.text.contains("not plumbed"))
    });
    assert_eq!(
        state.dev(DeviceId::Sv02).value,
        Some(10),
        "the valve must not have moved to a capped port"
    );
}

#[test]
fn a_temperature_setpoint_reaches_the_server() {
    use tstand_controler::bus::BusCmd;
    use tstand_controler::temperature::TempOp;

    let host = start_server("");
    let bus = remote_bus(&host, "");
    wait_for(&bus, 10, "a connection", |s| s.connected);

    // No CAN board in the test server, so the command is reported as having
    // nowhere to go. What is being checked is the route: a temperature command
    // typed on the PC arrives at the machine that would own the adapter.
    bus.send(BusCmd::Temp(TempOp::SetTemperature(37.0)));
    wait_for(&bus, 10, "the server to account for the temperature command", |s| {
        s.log.iter().any(|l| l.text.contains("temperature") && l.text.contains("37"))
    });
}

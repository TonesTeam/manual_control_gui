//! Messages between the GUI and `tstand_server`.
//!
//! One JSON value per line over a plain TCP socket, in both directions. That
//! is enough for five devices polled a few times a second, it debugs with
//! `nc`, and it keeps the rig's Raspberry Pi free of an async runtime.
//!
//! The client speaks first with [`ClientMsg::Hello`]; the server answers
//! [`ServerMsg::Welcome`] or [`ServerMsg::Denied`] and closes. After that both
//! sides send whenever they have something to say.

use std::io::{BufRead, Write};

use serde::{Deserialize, Serialize};
use serde::de::DeserializeOwned;

use crate::bus::{ArmedStop, BusCmd, DeviceLive, LogLine, StopFired};
use crate::config::{Backend, Settings};
use crate::sensors::SensorState;
use crate::temperature::TempState;
use crate::controller_api as api;

/// Bumped when a change would make an old client misread a new server. The
/// server refuses anything that does not match rather than half-working.
pub const PROTOCOL: u32 = 1;

pub const DEFAULT_PORT: u16 = 7373;

#[derive(Serialize, Deserialize, Clone, Debug)]
pub enum ClientMsg {
    Hello {
        protocol: u32,
        token: String,
        /// Who is connecting, for the server's log.
        client: String,
    },
    /// Drive the rig. `Apply` carries the settings the server should adopt.
    Cmd(BusCmd),
    /// Forward an HTTP request to controller_v2, which listens on loopback
    /// where the server runs and is otherwise unreachable from here.
    Api { id: u64, request: api::Request },
    /// Serial ports on the server's machine, for the Settings tab.
    ListPorts,
    Ping,
}

#[derive(Serialize, Deserialize, Clone, Debug)]
pub enum ServerMsg {
    Welcome {
        protocol: u32,
        /// Host name of the machine driving the rig.
        server: String,
        /// What the server is actually running, so Settings shows the truth
        /// rather than whatever this client last saved. Boxed to keep the
        /// enum small; `Box<T>` serialises exactly as `T`, so the wire format
        /// is unchanged.
        settings: Box<Settings>,
        clients: usize,
    },
    Denied(String),
    /// Boxed: this dwarfs every other variant, and each client's queue holds
    /// up to 64 messages.
    State(Box<StateMsg>),
    /// Log lines this client has not seen, oldest first.
    Log(Vec<LogLine>),
    Ports(Vec<String>),
    ApiReply { id: u64, result: Result<(u16, String), String> },
    Pong,
}

/// A `BusState` without the parts that do not travel: the log goes separately
/// and incrementally, `history` is rebuilt client-side, and `started` becomes
/// an uptime the client can subtract from its own clock.
#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct StateMsg {
    pub backend: Backend,
    pub connected: bool,
    pub connection_error: Option<String>,
    pub polling: bool,
    pub trace: bool,
    pub devices: [DeviceLive; 5],
    pub frames_tx: u64,
    pub frames_bad: u64,
    pub cycle_ms: f64,
    pub temp: TempState,
    pub sensors: SensorState,
    pub armed_stop: Option<ArmedStop>,
    pub stop_fired: Option<StopFired>,
    pub uptime: f64,
}

/// Reads one JSON value per line. Returns `Ok(None)` at a clean end of stream.
pub fn read<T: DeserializeOwned>(reader: &mut impl BufRead) -> Result<Option<T>, String> {
    let mut line = String::new();
    loop {
        line.clear();
        match reader.read_line(&mut line) {
            Ok(0) => return Ok(None),
            Ok(_) => {}
            Err(e) => return Err(e.to_string()),
        }
        if line.trim().is_empty() {
            continue; // keepalive newline
        }
        return serde_json::from_str(line.trim())
            .map(Some)
            .map_err(|e| format!("bad message: {e}"));
    }
}

pub fn write<T: Serialize>(writer: &mut impl Write, value: &T) -> Result<(), String> {
    let mut line = serde_json::to_string(value).map_err(|e| e.to_string())?;
    line.push('\n');
    writer.write_all(line.as_bytes()).map_err(|e| e.to_string())?;
    writer.flush().map_err(|e| e.to_string())
}

/// Accepts `tonespi.local`, `tonespi.local:7373` or a bare IP, and always
/// returns something with a port.
pub fn normalize_host(host: &str) -> String {
    let host = host.trim().trim_start_matches("tcp://");
    // Bare IPv6 (`::1`) needs brackets before a port can be appended; if the
    // user already wrote them, or there is a single colon, take it as given.
    let has_port = match host.rsplit_once(':') {
        Some((head, tail)) => tail.chars().all(|c| c.is_ascii_digit()) && !head.is_empty() && !tail.is_empty(),
        None => false,
    };
    if has_port { host.to_string() } else { format!("{host}:{DEFAULT_PORT}") }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn host_gets_a_default_port() {
        assert_eq!(normalize_host("tonespi.local"), "tonespi.local:7373");
        assert_eq!(normalize_host(" tonespi.local:9000 "), "tonespi.local:9000");
        assert_eq!(normalize_host("192.168.1.5"), "192.168.1.5:7373");
        assert_eq!(normalize_host("192.168.1.5:7373"), "192.168.1.5:7373");
        assert_eq!(normalize_host("[::1]:7373"), "[::1]:7373");
    }

    #[test]
    fn messages_round_trip_one_per_line() {
        let mut buf: Vec<u8> = Vec::new();
        write(&mut buf, &ClientMsg::Ping).unwrap();
        write(&mut buf, &ClientMsg::ListPorts).unwrap();
        assert_eq!(buf.iter().filter(|&&b| b == b'\n').count(), 2);

        let mut r = std::io::BufReader::new(&buf[..]);
        assert!(matches!(read::<ClientMsg>(&mut r).unwrap(), Some(ClientMsg::Ping)));
        assert!(matches!(read::<ClientMsg>(&mut r).unwrap(), Some(ClientMsg::ListPorts)));
        assert!(read::<ClientMsg>(&mut r).unwrap().is_none());
    }

    #[test]
    fn a_command_survives_the_wire() {
        use crate::bus::Op;
        use crate::devices::DeviceId;
        let mut buf: Vec<u8> = Vec::new();
        write(&mut buf, &ClientMsg::Cmd(BusCmd::Device(DeviceId::Sv02, Op::ValveTo(11)))).unwrap();
        let mut r = std::io::BufReader::new(&buf[..]);
        match read::<ClientMsg>(&mut r).unwrap() {
            Some(ClientMsg::Cmd(BusCmd::Device(id, op))) => {
                assert_eq!(id, DeviceId::Sv02);
                assert_eq!(op, Op::ValveTo(11));
            }
            other => panic!("expected a device command, got {other:?}"),
        }
    }
}

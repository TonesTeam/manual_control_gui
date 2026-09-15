//! Client for controller_v2's HTTP API — the same packets `signal_sender`
//! builds on the command line, sent from a background thread.

use serde::Serialize;
use std::io::{Read, Write};
use std::net::TcpStream;
use std::sync::mpsc::{Receiver, Sender, channel};
use std::thread;
use std::time::Duration;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CommandKind {
    pub id: u16,
    pub name: &'static str,
    pub uses_reagent: bool,
    pub uses_timing: bool,
    pub uses_temperature: bool,
    pub production: bool,
}

const fn kind(
    id: u16,
    name: &'static str,
    uses_reagent: bool,
    uses_timing: bool,
    uses_temperature: bool,
    production: bool,
) -> CommandKind {
    CommandKind { id, name, uses_reagent, uses_timing, uses_temperature, production }
}

/// Wire command IDs from `task_server.rs` / docs/operational-notes.md.
pub const COMMANDS: &[CommandKind] = &[
    kind(1, "Run protocol (external reagent infill)", true, true, true, true),
    kind(12, "Robot protocol step", true, true, true, true),
    kind(17, "Aggregated infill (experimental)", true, true, true, false),
    kind(2, "Delay", false, true, false, false),
    kind(3, "Drain slot", false, false, false, false),
    kind(4, "Drain tube", false, false, false, false),
    kind(8, "Initialize components", false, false, false, false),
    kind(10, "Empty reagent tube", true, false, false, false),
    kind(11, "Prepare tubes", false, false, false, false),
];

pub const RESUME: u16 = 7;
pub const PAUSE: u16 = 9;
pub const ABORT: u16 = 13;
pub const PAUSE_ALL: u16 = 14;
pub const RESUME_ALL: u16 = 15;
pub const ABORT_ALL: u16 = 16;

#[derive(Serialize, Clone, Debug)]
pub struct SingleCommand {
    pub command_type: u16,
    pub temperature: f64,
    pub time: u64,
    pub wash_reps: u16,
    pub reagent_pos: u16,
    pub wash_time: u64,
    pub is_toxic: bool,
}

impl SingleCommand {
    pub fn control(command_type: u16) -> Self {
        Self {
            command_type,
            temperature: 25.0,
            time: 15,
            wash_reps: 2,
            reagent_pos: 3,
            wash_time: 10,
            is_toxic: false,
        }
    }
}

#[derive(Serialize, Clone, Debug)]
pub struct Packet {
    pub slot_id: u16,
    pub commands: Vec<SingleCommand>,
    pub task_id: u32,
    pub requested_start_ts_ms: u64,
}

#[derive(Clone, Debug)]
pub enum Request {
    PostData(Packet),
    EstimateTime(Packet),
    Initialize { port_name: String, sensor_port_name: String, baud_rate: u32 },
    GetStatus,
    GetSlotStatus,
    GetProtocolData,
}

impl Request {
    fn describe(&self) -> String {
        match self {
            Request::PostData(p) => format!("POST /data task {} slot {}", p.task_id, p.slot_id),
            Request::EstimateTime(p) => format!("POST /estimated-protocol-time slot {}", p.slot_id),
            Request::Initialize { .. } => "POST /initialize".to_string(),
            Request::GetStatus => "GET /status".to_string(),
            Request::GetSlotStatus => "GET /slot-status".to_string(),
            Request::GetProtocolData => "GET /get-protocol-data".to_string(),
        }
    }

    /// Background polls are not echoed into the log.
    pub fn is_poll(&self) -> bool {
        matches!(self, Request::GetStatus | Request::GetSlotStatus | Request::GetProtocolData)
    }
}

#[derive(Clone, Debug)]
pub struct Reply {
    pub request: Request,
    pub summary: String,
    pub result: Result<(u16, String), String>,
}

pub fn spawn(ctx: eframe::egui::Context) -> (Sender<(String, Request)>, Receiver<Reply>) {
    let (tx, rx) = channel::<(String, Request)>();
    let (reply_tx, reply_rx) = channel::<Reply>();
    thread::spawn(move || {
        for (host, request) in rx {
            let result = execute(&host, &request);
            let _ = reply_tx.send(Reply { summary: request.describe(), request, result });
            ctx.request_repaint();
        }
    });
    (tx, reply_rx)
}

fn execute(host: &str, request: &Request) -> Result<(u16, String), String> {
    match request {
        Request::PostData(p) => http(host, "POST", "/data", Some(json(p)?)),
        Request::EstimateTime(p) => http(host, "POST", "/estimated-protocol-time", Some(json(p)?)),
        Request::Initialize { port_name, sensor_port_name, baud_rate } => {
            let body = serde_json::json!({
                "port_name": port_name,
                "sensor_port_name": sensor_port_name,
                "baud_rate": baud_rate,
            });
            http(host, "POST", "/initialize", Some(body.to_string()))
        }
        Request::GetStatus => http(host, "GET", "/status", None),
        Request::GetSlotStatus => http(host, "GET", "/slot-status", None),
        Request::GetProtocolData => http(host, "GET", "/get-protocol-data", None),
    }
}

fn json<T: Serialize>(value: &T) -> Result<String, String> {
    serde_json::to_string(value).map_err(|e| e.to_string())
}

/// Minimal HTTP/1.1 client; the controller only listens on loopback.
fn http(host: &str, method: &str, path: &str, body: Option<String>) -> Result<(u16, String), String> {
    let addr = host
        .trim()
        .trim_start_matches("http://")
        .trim_end_matches('/')
        .to_string();
    let socket = addr
        .parse()
        .map_err(|e| format!("bad controller address '{addr}': {e}"))?;
    let mut stream = TcpStream::connect_timeout(&socket, Duration::from_millis(800))
        .map_err(|e| format!("controller unreachable at {addr}: {e}"))?;
    stream.set_read_timeout(Some(Duration::from_secs(10))).ok();

    let body = body.unwrap_or_default();
    let mut req = format!(
        "{method} {path} HTTP/1.1\r\nHost: {addr}\r\nConnection: close\r\nAccept: application/json\r\n"
    );
    if method == "POST" {
        req.push_str(&format!(
            "Content-Type: application/json\r\nContent-Length: {}\r\n",
            body.len()
        ));
    }
    req.push_str("\r\n");
    req.push_str(&body);
    stream.write_all(req.as_bytes()).map_err(|e| e.to_string())?;

    let mut raw = Vec::new();
    stream.read_to_end(&mut raw).map_err(|e| e.to_string())?;
    let text = String::from_utf8_lossy(&raw);
    let (head, payload) = text.split_once("\r\n\r\n").unwrap_or((&text, ""));
    let code = head
        .split_whitespace()
        .nth(1)
        .and_then(|c| c.parse::<u16>().ok())
        .ok_or_else(|| "malformed HTTP response".to_string())?;
    let chunked = head.to_ascii_lowercase().contains("transfer-encoding: chunked");
    let payload = if chunked { dechunk(payload) } else { payload.to_string() };
    Ok((code, payload))
}

fn dechunk(mut s: &str) -> String {
    let mut out = String::new();
    while let Some((size, rest)) = s.split_once("\r\n") {
        let Ok(n) = usize::from_str_radix(size.trim(), 16) else { break };
        if n == 0 || rest.len() < n {
            break;
        }
        out.push_str(&rest[..n]);
        s = rest[n..].trim_start_matches("\r\n");
    }
    out
}

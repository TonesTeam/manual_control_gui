//! Headless control server for the Tones liquid processing rig.
//!
//! Runs on the machine the RS485 adapter is plugged into — the rig's Raspberry
//! Pi — and is the only program that talks to the pumps and valves. GUIs
//! connect over TCP and get the same live state they used to compute locally.
//!
//! Builds without a display: `cargo build --release --no-default-features`.

use std::process::ExitCode;

use tstand_controler::config::{Backend, SERVER_SETTINGS_FILE, Settings};
use tstand_controler::remote::server::{self, Config};
use tstand_controler::wire::DEFAULT_PORT;

const USAGE: &str = "\
tstand_server — control server for the Tones liquid processing rig

USAGE:
    tstand_server [OPTIONS]

OPTIONS:
    --bind <ADDR>        Address to listen on [default: 0.0.0.0:7373]
    --token <SECRET>     Shared secret clients must send. Also read from
                         $TSTAND_TOKEN. Empty means no authentication.
    --port <DEV>         Serial port for the RS485 adapter, e.g. /dev/ttyUSB0
    --baud <RATE>        Serial baud rate [default: from saved settings]
    --simulator          Run the built-in simulator instead of the adapter
    --controller <ADDR>  controller_v2's HTTP address, reached from here
                         [default: 127.0.0.1:3000]
    --list-ports         Print the serial ports on this machine and exit
    -h, --help           Show this help

Settings are loaded from and saved to tstand_server.json; anything given on
the command line wins for this run. Connected GUIs can change them, and what
they apply is saved back to that file.
";

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("tstand_server: {e}");
            ExitCode::FAILURE
        }
    }
}

fn run() -> Result<(), String> {
    let mut settings = Settings::load_from(SERVER_SETTINGS_FILE);
    // A saved file from a GUI may say `Remote`; a server always drives hardware.
    if settings.backend == Backend::Remote {
        settings.backend = Backend::Serial;
    }
    let mut bind = format!("0.0.0.0:{DEFAULT_PORT}");
    let mut token = std::env::var("TSTAND_TOKEN").unwrap_or_default();
    let mut controller = settings.controller_url.clone();
    if controller.trim().is_empty() {
        controller = "127.0.0.1:3000".to_string();
    }

    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        let mut value = |name: &str| -> Result<String, String> {
            args.next().ok_or_else(|| format!("{name} needs a value"))
        };
        match arg.as_str() {
            "-h" | "--help" => {
                print!("{USAGE}");
                return Ok(());
            }
            "--list-ports" => {
                let ports = server::list_ports();
                if ports.is_empty() {
                    println!("no serial ports found");
                } else {
                    for p in ports {
                        println!("{p}");
                    }
                }
                return Ok(());
            }
            "--bind" => bind = value("--bind")?,
            "--token" => token = value("--token")?,
            "--controller" => controller = value("--controller")?,
            "--port" => {
                settings.port = value("--port")?;
                settings.backend = Backend::Serial;
            }
            "--baud" => {
                let raw = value("--baud")?;
                settings.baud_rate = raw.parse().map_err(|_| format!("bad baud rate '{raw}'"))?;
            }
            "--simulator" => settings.backend = Backend::Simulator,
            other => return Err(format!("unknown option '{other}'\n\n{USAGE}")),
        }
    }

    if settings.backend == Backend::Serial && settings.port.trim().is_empty() {
        return Err(format!(
            "no serial port set. Pass --port /dev/ttyUSB0 (see --list-ports), or --simulator to run without hardware.\n\n{USAGE}"
        ));
    }

    server::run(Config { bind, token, controller_url: controller, settings })
}

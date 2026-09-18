//! Run the prime-and-calibrate sequence from a terminal.
//!
//! The same [`tstand_controler::routine`] state machine the GUI button drives,
//! against the same remote bus — this only prints what it is doing. Useful for
//! commissioning over SSH, and for running the sequence somewhere its progress
//! can be read back later.
//!
//! It moves real liquid, so it asks before it starts unless told not to.

use std::io::Write;
use std::process::ExitCode;
use std::time::{Duration, Instant};

use tstand_controler::bus::{Bus, BusCmd, BusState, Op, Wake};
use tstand_controler::config::{Backend, Settings};
use tstand_controler::devices::DeviceId;
use tstand_controler::fluidics::{self, Fluidics};
use tstand_controler::routine::{self, Routine};

const USAGE: &str = "\
tstand_prime — prime the wash line and measure what it holds

USAGE:
    tstand_prime [OPTIONS]

OPTIONS:
    --server <ADDR>   tstand_server to drive [default: tonespi.local:7373]
    --token <SECRET>  Shared secret, or $TSTAND_TOKEN
    --yes             Do not ask before moving liquid
    --dry-run         Print the plan and the rig's state, then stop
    -h, --help        Show this help

It draws wash into PP01, primes the line to the S2 sensor against waste, then
opens the slot and creeps in until the slot's own sensor wets, reporting what
each leg took. Every move is a bounded chunk: killing this leaves the pump
stopped, not running.
";

fn main() -> ExitCode {
    match run() {
        Ok(true) => ExitCode::SUCCESS,
        Ok(false) => ExitCode::FAILURE,
        Err(e) => {
            eprintln!("tstand_prime: {e}");
            ExitCode::FAILURE
        }
    }
}

fn run() -> Result<bool, String> {
    let mut server = "tonespi.local:7373".to_string();
    let mut token = std::env::var("TSTAND_TOKEN").unwrap_or_default();
    let mut assume_yes = false;
    let mut dry_run = false;
    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        let mut value = |name: &str| args.next().ok_or_else(|| format!("{name} needs a value"));
        match arg.as_str() {
            "-h" | "--help" => {
                print!("{USAGE}");
                return Ok(true);
            }
            "--server" => server = value("--server")?,
            "--token" => token = value("--token")?,
            "--yes" => assume_yes = true,
            "--dry-run" => dry_run = true,
            other => return Err(format!("unknown option '{other}'\n\n{USAGE}")),
        }
    }

    let settings = Settings {
        backend: Backend::Remote,
        server_host: server.clone(),
        server_token: token,
        server_backend: Backend::Serial,
        ..Settings::default()
    };
    let bus = Bus::spawn(Wake::none(), settings);

    eprintln!("connecting to {server}…");
    let state = wait_until(&bus, Duration::from_secs(20), |s| s.connected && s.dev(DeviceId::Pp01).online)
        .ok_or_else(|| format!("no answer from {server}"))?;

    // The rig's own plumbing decides the ports, so take the server's copy
    // rather than this machine's defaults.
    let rig = tstand_controler::remote::client::server_settings_since(0)
        .map(|(_, s)| s.rig)
        .ok_or("the server did not report its rig configuration")?;
    let plan = routine::Plan::for_rig(&rig)?;

    println!("rig      SV01 port {} wash · SV02 port {} waste · SV02 port {} slot", plan.wash_port, plan.waste_port, plan.fill_port);
    println!("sensors  S2 = channel {}, slot = channel {}", plan.edge_sensor, plan.slot_sensor);
    println!("speeds   {} rpm approach, {} rpm creep", plan.coarse_speed, plan.fine_speed);
    println!("caps     {} steps to S2, {} steps into the slot", plan.cap_steps, plan.slot_cap_steps);
    println!(
        "now      PP01 {} steps · S2 {} · slot {}",
        state.dev(DeviceId::Pp01).value.map(|v| v.to_string()).unwrap_or_else(|| "?".into()),
        wetness(state.sensors.channel(plan.edge_sensor)),
        wetness(state.sensors.channel(plan.slot_sensor)),
    );
    if !state.sensors.connected {
        return Err("the optical sensor board is not answering; this run stops on its sensors".into());
    }
    if dry_run {
        println!("\n--dry-run: nothing sent.");
        return Ok(true);
    }
    if !assume_yes && !confirm()? {
        println!("nothing sent.");
        return Ok(true);
    }

    drive(&bus, plan)
}

fn wetness(v: Option<bool>) -> &'static str {
    match v {
        Some(true) => "wet",
        Some(false) => "dry",
        None => "unknown",
    }
}

fn confirm() -> Result<bool, String> {
    print!("\nThis moves liquid on the rig. Watch it. Continue? [y/N] ");
    std::io::stdout().flush().ok();
    let mut line = String::new();
    std::io::stdin().read_line(&mut line).map_err(|e| e.to_string())?;
    Ok(matches!(line.trim(), "y" | "Y" | "yes"))
}

/// Runs the state machine to completion, reporting each stage as it changes.
fn drive(bus: &Bus, plan: routine::Plan) -> Result<bool, String> {
    let mut run = Routine::new(plan);
    let mut sent_at: Option<Instant> = None;
    let mut stage = None;
    let started = Instant::now();
    let deadline = started + Duration::from_secs(600);

    loop {
        if Instant::now() > deadline {
            bus.send(BusCmd::StopAll);
            return Err("gave up after 10 minutes; pumps stopped".into());
        }
        let state = bus.snapshot();
        if !state.connected {
            // No link means no way to stop it either; the pump is between
            // bounded chunks, so it is already stopped.
            return Err("lost the link to the rig mid-run".into());
        }
        if stage != Some(run.stage()) {
            stage = Some(run.stage());
            println!("[{:>5.1}s] {}", started.elapsed().as_secs_f32(), run.stage().label());
        }

        let pump = state.dev(DeviceId::Pp01);
        let busy = |d: &tstand_controler::bus::DeviceLive| {
            d.target.is_some() || matches!(d.status, Some(0x04 | 0xFE))
        };
        let fresh = sent_at.is_some_and(|t| t.elapsed() < Duration::from_millis(700));
        let settled = !fresh
            && !busy(pump)
            && !busy(state.dev(DeviceId::Sv01))
            && !busy(state.dev(DeviceId::Sv02))
            && !busy(state.dev(DeviceId::Sv03));

        let obs = routine::Observation {
            pump: pump.value,
            settled,
            now: started.elapsed().as_secs_f64(),
            edge_wet: state.sensors.channel(plan.edge_sensor),
            slot_wet: state.sensors.channel(plan.slot_sensor),
            first_wet: state.sensors.channel(plan.first_sensor),
        };
        match run.step(obs) {
            routine::Action::Wait => std::thread::sleep(Duration::from_millis(50)),
            routine::Action::Send(id, op) => {
                println!("        → {} {}", id.tag(), op.label());
                bus.device(id, op);
                sent_at = Some(Instant::now());
            }
            routine::Action::Finished(report) => {
                let ul = 2.083;
                let to_first = report.to_first_steps.map(|s| s as f32 * ul);
                let to_edge = report.to_edge_ul(ul);
                let to_slot = report.edge_to_slot_ul(ul);

                // Fold the measurement into the liquid model and keep it: the
                // whole point of the run is that the next one starts knowing
                // this.
                let mut model = Fluidics::load();
                model.record_prime(to_first, to_edge, to_slot);
                let bore = model.bore_id_mm;
                let save = model.save();

                println!();
                let leg = |name: &str, volume: f32, note: &str| {
                    println!("{name:<12} {volume:>6.0} µL  {:>6.0} mm  ({:.1} cm){note}", fluidics::length_mm(volume, bore), fluidics::length_mm(volume, bore) / 10.0);
                };
                match to_first {
                    Some(v) => {
                        leg("PP01 → S1", v, "");
                        leg("S1 → S2", to_edge - v, "");
                    }
                    None => leg("PP01 → S2", to_edge, "  (S1 not seen to change; not split)"),
                }
                leg("S2 → slot", to_slot, if report.slot_timed_out { "  ← slot sensor never wetted" } else { "" });
                println!("\nat {bore:.2} mm bore · {:.4} µL per mm", fluidics::bore_area_mm2(bore));
                if report.verified_steps > 0 {
                    println!(
                        "S2 held wet across a further {} steps ({:.0} µL) of pushing, so this is the column and not its leading edge",
                        report.verified_steps,
                        report.verified_steps as f32 * ul,
                    );
                }
                if report.cleared_through_froth {
                    println!("the line was cleared on froth, not on a dry reading — a detector would not hold still");
                }
                match report.bubbles_seen {
                    0 => println!("no bubbles: every stop agreed with itself first time"),
                    n => println!(
                        "{n} sensor contradiction(s) while the piston was still — bubbles or an unsettled meniscus; \
                         treat the figures above as approximate"
                    ),
                }
                match save {
                    Ok(()) => println!("saved to the liquid model."),
                    Err(e) => println!("could not save the liquid model: {e}"),
                }
                return Ok(!report.slot_timed_out);
            }
            routine::Action::Abort(why) => {
                bus.send(BusCmd::StopAll);
                println!("\naborted: {why}");
                println!("pumps stopped.");
                return Ok(false);
            }
        }
    }
}

fn wait_until(bus: &Bus, within: Duration, done: impl Fn(&BusState) -> bool) -> Option<BusState> {
    let deadline = Instant::now() + within;
    loop {
        let state = bus.snapshot();
        if done(&state) {
            return Some(state);
        }
        if Instant::now() >= deadline {
            return None;
        }
        std::thread::sleep(Duration::from_millis(50));
    }
}

/// Unused, but keeps `Op` in scope for the label call above on older toolchains.
#[allow(dead_code)]
fn _op_label(op: Op) -> String {
    op.label()
}

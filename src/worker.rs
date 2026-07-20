use std::sync::mpsc::{Receiver, Sender, channel};
use std::thread;

use crate::pump::PumpController;
use crate::selector_valve::SelectorController;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum DeviceId {
    MainPump,
    SecondaryPump,
    Selector1,
    Selector2,
    Selector3,
}

impl DeviceId {
    pub const ALL: [DeviceId; 5] = [
        DeviceId::MainPump,
        DeviceId::SecondaryPump,
        DeviceId::Selector1,
        DeviceId::Selector2,
        DeviceId::Selector3,
    ];

    pub fn label(&self) -> &'static str {
        match self {
            DeviceId::MainPump => "Main Pump",
            DeviceId::SecondaryPump => "Secondary Pump",
            DeviceId::Selector1 => "Selector 1 (8-position)",
            DeviceId::Selector2 => "Selector 2 (8-position)",
            DeviceId::Selector3 => "Selector 3 (16-position)",
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub enum DeviceConfig {
    Pump { has_solenoids: bool },
    Selector { max_pos: u16 },
}

#[derive(Clone, Debug)]
pub enum Action {
    GetPos,
    Home,
    ForceStop,
    GetMotorStatus,
    SetPos(u16),
    IncreasePosBy(u16),
    DecreasePosBy(u16),
    SetSpeed(u16),
    SolenoidsInput,
    SolenoidsOutput,
    SetValvePos(u16),
}

impl Action {
    pub fn label(&self) -> String {
        match self {
            Action::GetPos => "Get position".to_string(),
            Action::Home => "Home".to_string(),
            Action::ForceStop => "Force stop".to_string(),
            Action::GetMotorStatus => "Get motor status".to_string(),
            Action::SetPos(p) => format!("Set position to {p}"),
            Action::IncreasePosBy(p) => format!("Increase position by {p}"),
            Action::DecreasePosBy(p) => format!("Decrease position by {p}"),
            Action::SetSpeed(s) => format!("Set speed to {s}"),
            Action::SolenoidsInput => "Switch solenoids to input".to_string(),
            Action::SolenoidsOutput => "Switch solenoids to output".to_string(),
            Action::SetValvePos(p) => format!("Set valve position to {p}"),
        }
    }
}

#[derive(Clone, Debug)]
pub struct Command {
    pub device: DeviceId,
    pub port: String,
    pub baud: u32,
    pub slave_address: u8,
    pub config: DeviceConfig,
    pub action: Action,
}

#[derive(Clone, Debug)]
pub struct Response {
    pub device: DeviceId,
    pub label: String,
    pub result: Result<Option<u16>, String>,
}

fn run_pump_action(pump: &PumpController, action: &Action) -> Result<Option<u16>, String> {
    match action {
        Action::GetPos => pump.get_pos().map(Some),
        Action::Home => pump.set_home_pos().map(|_| None),
        Action::ForceStop => pump.force_stop().map(Some),
        Action::GetMotorStatus => pump.get_motor_status().map(Some),
        Action::SetPos(p) => pump.set_pos_autostop(*p).map(|_| None),
        Action::IncreasePosBy(p) => pump.increase_pos_by(*p).map(|_| None),
        Action::DecreasePosBy(p) => pump.decrease_pos_by(*p).map(|_| None),
        Action::SetSpeed(s) => pump.set_speed(*s).map(|_| None),
        Action::SolenoidsInput => pump.set_solenoids_input().map(|_| None),
        Action::SolenoidsOutput => pump.set_solenoids_output().map(|_| None),
        Action::SetValvePos(_) => Err("Not applicable to a pump".to_string()),
    }
}

fn run_selector_action(selector: &SelectorController, action: &Action) -> Result<Option<u16>, String> {
    match action {
        Action::Home => selector.set_home_pos().map(|_| None),
        Action::SetValvePos(p) => selector.set_valve_pos(*p).map(|_| None),
        _ => Err("Not applicable to a selector valve".to_string()),
    }
}

fn execute(cmd: &Command) -> Result<Option<u16>, String> {
    match cmd.config {
        DeviceConfig::Pump { has_solenoids } => {
            let pump = PumpController::new(cmd.port.clone(), cmd.baud, cmd.slave_address)?
                .with_solenoids(has_solenoids);
            run_pump_action(&pump, &cmd.action)
        }
        DeviceConfig::Selector { max_pos } => {
            let selector =
                SelectorController::new(cmd.port.clone(), cmd.baud, cmd.slave_address, max_pos)?;
            run_selector_action(&selector, &cmd.action)
        }
    }
}

/// Spawns a single background worker thread that executes serial commands
/// sequentially (the bus is shared by all devices, so commands must not overlap).
pub fn spawn_worker(ctx: eframe::egui::Context) -> (Sender<Command>, Receiver<Response>) {
    let (cmd_tx, cmd_rx) = channel::<Command>();
    let (resp_tx, resp_rx) = channel::<Response>();

    thread::spawn(move || {
        for cmd in cmd_rx {
            let label = cmd.action.label();
            let result = execute(&cmd);
            let _ = resp_tx.send(Response {
                device: cmd.device,
                label,
                result,
            });
            ctx.request_repaint();
        }
    });

    (cmd_tx, resp_rx)
}

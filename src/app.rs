use std::collections::HashMap;
use std::sync::mpsc::{Receiver, Sender};

use eframe::egui;

use crate::config::{SELECTOR_16_MAX, SELECTOR_8_MAX, Settings};
use crate::worker::{Action, Command, DeviceConfig, DeviceId, Response, spawn_worker};

#[derive(Default)]
struct DeviceState {
    position: Option<u16>,
    motor_status: Option<u16>,
    last_message: String,
    is_error: bool,
    busy: bool,
}

struct DeviceInputs {
    pos_input: String,
    step_input: String,
    speed_input: String,
}

impl Default for DeviceInputs {
    fn default() -> Self {
        Self {
            pos_input: String::from("0"),
            step_input: String::from("10"),
            speed_input: String::from("200"),
        }
    }
}

#[derive(PartialEq)]
enum Tab {
    Control,
    Settings,
}

pub struct App {
    settings: Settings,
    available_ports: Vec<String>,
    tab: Tab,
    cmd_tx: Sender<Command>,
    resp_rx: Receiver<Response>,
    states: HashMap<DeviceId, DeviceState>,
    inputs: HashMap<DeviceId, DeviceInputs>,
    settings_message: Option<String>,
}

impl App {
    pub fn new(cc: &eframe::CreationContext<'_>) -> Self {
        let (cmd_tx, resp_rx) = spawn_worker(cc.egui_ctx.clone());
        let mut states = HashMap::new();
        let mut inputs = HashMap::new();
        for id in DeviceId::ALL {
            states.insert(id, DeviceState::default());
            inputs.insert(id, DeviceInputs::default());
        }
        Self {
            settings: Settings::load(),
            available_ports: list_ports(),
            tab: Tab::Control,
            cmd_tx,
            resp_rx,
            states,
            inputs,
            settings_message: None,
        }
    }

    fn device_config(&self, id: DeviceId) -> DeviceConfig {
        match id {
            DeviceId::MainPump => DeviceConfig::Pump { has_solenoids: true },
            DeviceId::SecondaryPump => DeviceConfig::Pump { has_solenoids: false },
            DeviceId::Selector1 => DeviceConfig::Selector { max_pos: SELECTOR_8_MAX },
            DeviceId::Selector2 => DeviceConfig::Selector { max_pos: SELECTOR_8_MAX },
            DeviceId::Selector3 => DeviceConfig::Selector { max_pos: SELECTOR_16_MAX },
        }
    }

    fn slave_address(&self, id: DeviceId) -> u8 {
        match id {
            DeviceId::MainPump => self.settings.main_pump_addr,
            DeviceId::SecondaryPump => self.settings.secondary_pump_addr,
            DeviceId::Selector1 => self.settings.selector1_addr,
            DeviceId::Selector2 => self.settings.selector2_addr,
            DeviceId::Selector3 => self.settings.selector3_addr,
        }
    }

    fn send(&mut self, id: DeviceId, action: Action) {
        if self.settings.port.is_empty() {
            if let Some(state) = self.states.get_mut(&id) {
                state.is_error = true;
                state.last_message = "No COM port selected. Configure it in Settings.".to_string();
            }
            return;
        }
        let cmd = Command {
            device: id,
            port: self.settings.port.clone(),
            baud: self.settings.baud_rate,
            slave_address: self.slave_address(id),
            config: self.device_config(id),
            action,
        };
        if let Some(state) = self.states.get_mut(&id) {
            state.busy = true;
        }
        let _ = self.cmd_tx.send(cmd);
    }

    fn drain_responses(&mut self) {
        while let Ok(resp) = self.resp_rx.try_recv() {
            if let Some(state) = self.states.get_mut(&resp.device) {
                state.busy = false;
                match resp.result {
                    Ok(value) => {
                        state.is_error = false;
                        state.last_message = format!("{}: OK", resp.label);
                        if matches!(
                            resp.label.as_str(),
                            "Get position" | "Force stop"
                        ) {
                            state.position = value;
                        } else if resp.label == "Get motor status" {
                            state.motor_status = value;
                        } else if resp.label.starts_with("Set position")
                            || resp.label.starts_with("Set valve position")
                        {
                            // We don't get a readback; remember what we asked for.
                        }
                    }
                    Err(err) => {
                        state.is_error = true;
                        state.last_message = format!("{}: {}", resp.label, err);
                    }
                }
            }
        }
    }

    fn pump_panel(&mut self, ui: &mut egui::Ui, id: DeviceId, has_solenoids: bool) {
        let state_snapshot = self
            .states
            .get(&id)
            .map(|s| (s.position, s.motor_status, s.last_message.clone(), s.is_error, s.busy))
            .unwrap_or_default();
        let (position, motor_status, last_message, is_error, busy) = state_snapshot;

        ui.group(|ui| {
            ui.heading(id.label());
            ui.horizontal(|ui| {
                ui.label("Position:");
                ui.label(position.map(|p| p.to_string()).unwrap_or_else(|| "?".to_string()));
                if ui.button("Get Position").clicked() {
                    self.send(id, Action::GetPos);
                }
                if ui.button("Get Motor Status").clicked() {
                    self.send(id, Action::GetMotorStatus);
                }
                ui.label(format!(
                    "Motor status: {}",
                    motor_status.map(|s| s.to_string()).unwrap_or_else(|| "?".to_string())
                ));
            });

            ui.horizontal(|ui| {
                let inputs = self.inputs.get_mut(&id).unwrap();
                ui.label("Go to position (0-3810):");
                let resp =
                    ui.add(egui::TextEdit::singleline(&mut inputs.pos_input).desired_width(60.0));
                let enter_pressed =
                    resp.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter));
                let target: Option<u16> = inputs.pos_input.trim().parse().ok();
                let clicked = ui
                    .add_enabled(target.is_some(), egui::Button::new("Set Position"))
                    .clicked();
                if let Some(t) = target {
                    if clicked || enter_pressed {
                        self.send(id, Action::SetPos(t));
                    }
                }
            });

            ui.horizontal(|ui| {
                let inputs = self.inputs.get_mut(&id).unwrap();
                ui.label("Step:");
                ui.add(egui::TextEdit::singleline(&mut inputs.step_input).desired_width(50.0));
                let step: Option<u16> = inputs.step_input.trim().parse().ok();
                if ui
                    .add_enabled(step.is_some(), egui::Button::new("Increase"))
                    .clicked()
                {
                    self.send(id, Action::IncreasePosBy(step.unwrap()));
                }
                if ui
                    .add_enabled(step.is_some(), egui::Button::new("Decrease"))
                    .clicked()
                {
                    self.send(id, Action::DecreasePosBy(step.unwrap()));
                }
            });

            ui.horizontal(|ui| {
                let inputs = self.inputs.get_mut(&id).unwrap();
                ui.label("Speed (0-500):");
                let resp = ui
                    .add(egui::TextEdit::singleline(&mut inputs.speed_input).desired_width(50.0));
                let enter_pressed =
                    resp.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter));
                let speed: Option<u16> = inputs.speed_input.trim().parse().ok();
                let clicked = ui
                    .add_enabled(speed.is_some(), egui::Button::new("Set Speed"))
                    .clicked();
                if let Some(s) = speed {
                    if clicked || enter_pressed {
                        self.send(id, Action::SetSpeed(s));
                    }
                }
            });

            ui.horizontal(|ui| {
                if ui.button("Home").clicked() {
                    self.send(id, Action::Home);
                }
                if ui
                    .add(egui::Button::new("Force Stop").fill(egui::Color32::from_rgb(140, 40, 40)))
                    .clicked()
                {
                    self.send(id, Action::ForceStop);
                }
            });

            if has_solenoids {
                ui.horizontal(|ui| {
                    ui.label("Flow direction (solenoid):");
                    if ui.button("Input").clicked() {
                        self.send(id, Action::SolenoidsInput);
                    }
                    if ui.button("Output").clicked() {
                        self.send(id, Action::SolenoidsOutput);
                    }
                });
            }

            if busy {
                ui.horizontal(|ui| {
                    ui.spinner();
                    ui.label("Working...");
                });
            }
            if !last_message.is_empty() {
                let color = if is_error {
                    egui::Color32::from_rgb(220, 80, 80)
                } else {
                    egui::Color32::from_rgb(120, 180, 120)
                };
                ui.colored_label(color, &last_message);
            }
        });
    }

    fn selector_panel(&mut self, ui: &mut egui::Ui, id: DeviceId, max_pos: u16) {
        let state_snapshot = self
            .states
            .get(&id)
            .map(|s| (s.last_message.clone(), s.is_error, s.busy))
            .unwrap_or_default();
        let (last_message, is_error, busy) = state_snapshot;

        ui.group(|ui| {
            ui.heading(id.label());
            ui.horizontal(|ui| {
                let inputs = self.inputs.get_mut(&id).unwrap();
                ui.label(format!("Position (0-{max_pos}):"));
                let resp =
                    ui.add(egui::TextEdit::singleline(&mut inputs.pos_input).desired_width(50.0));
                let enter_pressed =
                    resp.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter));
                let target: Option<u16> = inputs
                    .pos_input
                    .trim()
                    .parse()
                    .ok()
                    .filter(|p| *p <= max_pos);
                let clicked = ui
                    .add_enabled(target.is_some(), egui::Button::new("Set Position"))
                    .clicked();
                if let Some(t) = target {
                    if clicked || enter_pressed {
                        self.send(id, Action::SetValvePos(t));
                    }
                }
                if ui.button("Home").clicked() {
                    self.send(id, Action::Home);
                }
            });

            if busy {
                ui.horizontal(|ui| {
                    ui.spinner();
                    ui.label("Working...");
                });
            }
            if !last_message.is_empty() {
                let color = if is_error {
                    egui::Color32::from_rgb(220, 80, 80)
                } else {
                    egui::Color32::from_rgb(120, 180, 120)
                };
                ui.colored_label(color, &last_message);
            }
        });
    }

    fn control_tab(&mut self, ui: &mut egui::Ui) {
        if self.settings.port.is_empty() {
            ui.colored_label(
                egui::Color32::from_rgb(220, 150, 60),
                "No COM port selected. Go to the Settings tab to configure the connection.",
            );
        } else {
            ui.label(format!(
                "Connected via {} @ {} baud",
                self.settings.port, self.settings.baud_rate
            ));
        }
        if ui.button("Home All").clicked() {
            for id in DeviceId::ALL {
                self.send(id, Action::Home);
            }
        }
        ui.separator();
        egui::ScrollArea::vertical().show(ui, |ui| {
            self.pump_panel(ui, DeviceId::MainPump, true);
            self.pump_panel(ui, DeviceId::SecondaryPump, false);
            self.selector_panel(ui, DeviceId::Selector1, SELECTOR_8_MAX);
            self.selector_panel(ui, DeviceId::Selector2, SELECTOR_8_MAX);
            self.selector_panel(ui, DeviceId::Selector3, SELECTOR_16_MAX);
        });
    }

    fn settings_tab(&mut self, ui: &mut egui::Ui) {
        ui.heading("Connection");
        ui.horizontal(|ui| {
            ui.label("COM port:");
            egui::ComboBox::from_id_salt("port_combo")
                .selected_text(if self.settings.port.is_empty() {
                    "Select a port"
                } else {
                    self.settings.port.as_str()
                })
                .show_ui(ui, |ui| {
                    for port in &self.available_ports {
                        ui.selectable_value(&mut self.settings.port, port.clone(), port);
                    }
                });
            if ui.button("Refresh").clicked() {
                self.available_ports = list_ports();
            }
        });
        ui.horizontal(|ui| {
            ui.label("Or type manually:");
            ui.text_edit_singleline(&mut self.settings.port);
        });
        ui.horizontal(|ui| {
            ui.label("Baud rate:");
            ui.add(egui::DragValue::new(&mut self.settings.baud_rate).range(300..=921_600));
        });

        ui.add_space(10.0);
        ui.heading("Slave addresses");
        ui.label("Each device on the bus needs its own address (0-255).");
        egui::Grid::new("addr_grid").num_columns(2).spacing([20.0, 8.0]).show(ui, |ui| {
            ui.label("Main pump:");
            ui.add(egui::DragValue::new(&mut self.settings.main_pump_addr).range(0..=255));
            ui.end_row();

            ui.label("Secondary pump:");
            ui.add(egui::DragValue::new(&mut self.settings.secondary_pump_addr).range(0..=255));
            ui.end_row();

            ui.label("Selector 1 (8-position):");
            ui.add(egui::DragValue::new(&mut self.settings.selector1_addr).range(0..=255));
            ui.end_row();

            ui.label("Selector 2 (8-position):");
            ui.add(egui::DragValue::new(&mut self.settings.selector2_addr).range(0..=255));
            ui.end_row();

            ui.label("Selector 3 (16-position):");
            ui.add(egui::DragValue::new(&mut self.settings.selector3_addr).range(0..=255));
            ui.end_row();
        });

        ui.add_space(10.0);
        if ui.button("Save settings").clicked() {
            self.settings_message = Some(match self.settings.save() {
                Ok(()) => "Settings saved.".to_string(),
                Err(e) => format!("Failed to save settings: {e}"),
            });
        }
        if let Some(msg) = &self.settings_message {
            ui.label(msg);
        }
    }
}

impl eframe::App for App {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        self.drain_responses();

        egui::Panel::top("tabs").show(ui, |ui| {
            ui.horizontal(|ui| {
                ui.selectable_value(&mut self.tab, Tab::Control, "Control");
                ui.selectable_value(&mut self.tab, Tab::Settings, "Settings");
            });
        });

        egui::CentralPanel::default().show(ui, |ui| match self.tab {
            Tab::Control => self.control_tab(ui),
            Tab::Settings => self.settings_tab(ui),
        });
    }
}

fn list_ports() -> Vec<String> {
    serialport::available_ports()
        .map(|ports| ports.into_iter().map(|p| p.port_name).collect())
        .unwrap_or_default()
}

use std::collections::VecDeque;
use std::time::{Duration, Instant};

use eframe::egui::{self, Color32, RichText};

use crate::bus::{Bus, BusCmd, BusState, DevState, Level, Op, Wake};
use crate::rig::{PortUse, Rig, Role, Sensor, SensorAt};
use crate::routine::{self, Routine};
use crate::temperature::{TempOp, TempSample, TempStatus, trend_range};
use crate::config::{Backend, Settings};
use crate::controller_api::{self as api, Packet, Request, SingleCommand};
use crate::devices::{DeviceId, Kind, sv02_port_for_slot, sv03_port_for_slot};
use crate::protocol;
use crate::schematic::{self, Target};
use crate::tracking::{BOTTLE_CAPACITY_ML, FluidView, Tracker};

#[derive(PartialEq, Clone, Copy)]
enum Tab {
    Monitor,
    Protocols,
    Datasheets,
    Log,
    Settings,
}

struct PumpInputs {
    target: u16,
    steps: u16,
    speed: u16,
    by_volume: bool,
    volume_ul: f32,
}

impl Default for PumpInputs {
    fn default() -> Self {
        Self { target: 0, steps: 100, speed: 200, by_volume: false, volume_ul: 100.0 }
    }
}

struct ProtocolForm {
    slot: u16,
    kind: usize,
    reagent_pos: u16,
    wash_reps: u16,
    time: u64,
    wash_time: u64,
    temperature: f64,
    is_toxic: bool,
    commands: Vec<SingleCommand>,
    next_task_id: u32,
    control_task_id: u32,
    start_delay_s: u64,
    auto_refresh: bool,
    last_refresh: Option<Instant>,
    init_port: String,
    init_sensor_port: String,
    init_baud: u32,
}

impl Default for ProtocolForm {
    fn default() -> Self {
        Self {
            slot: 1,
            kind: 0,
            reagent_pos: 3,
            wash_reps: 2,
            time: 15,
            wash_time: 10,
            temperature: 25.0,
            is_toxic: false,
            commands: Vec::new(),
            next_task_id: 1,
            control_task_id: 1,
            start_delay_s: 0,
            auto_refresh: false,
            last_refresh: None,
            init_port: "/dev/ttyUSB0".into(),
            init_sensor_port: "/dev/ttyACM0".into(),
            init_baud: 9600,
        }
    }
}

#[derive(Default)]
struct ControllerView {
    reachable: Option<bool>,
    status: Option<serde_json::Value>,
    slots: Option<serde_json::Value>,
    protocols: Option<serde_json::Value>,
    log: Vec<(Instant, bool, String)>,
}

pub struct App {
    settings: Settings,
    draft: Settings,
    available_ports: Vec<String>,
    tab: Tab,
    bus: Bus,
    /// Generation of the server settings already folded into `settings`, so a
    /// reconnect does not fight the user for the Settings fields every frame.
    adopted_server_settings: u64,
    /// Backend the level tracker was built for; it persists levels for real
    /// hardware and starts fresh for the simulator.
    tracker_backend: Backend,
    /// How much of the temperature trend to show, in seconds. A fixed span
    /// rather than "whatever has been collected", so the trace scrolls through
    /// a stable axis instead of the axis rescaling under it.
    temp_window_secs: f64,
    /// Setpoint being typed, before it is sent to the board.
    temp_setpoint: f32,
    temp_tolerance: f32,
    /// Setpoint the board last reported, so the box follows it until edited.
    temp_setpoint_seen: Option<f32>,
    /// True once the band has been typed in, so the board stops overwriting it.
    temp_tolerance_edited: bool,
    selected: Option<DeviceId>,
    click_to_actuate: bool,
    valve_confirm: Option<(DeviceId, u16)>,
    /// Component the open right-click menu belongs to.
    menu_target: Option<Target>,
    tracker: Tracker,
    last_slot_poll: Option<Instant>,
    /// Zoom factor last applied from settings or keyboard.
    applied_zoom: f32,
    pump_inputs: [PumpInputs; 2],
    form: ProtocolForm,
    ctrl: ControllerView,
    log_filter: String,
    show_trace: bool,
    settings_message: Option<String>,
    started: Instant,
    demo: VecDeque<(DeviceId, Op)>,
    demo_wait: Option<(DeviceId, Instant)>,
    /// Where the liquid is, tracked from piston travel and sensor edges.
    liquid: crate::fluidics::Tracker,
    /// The prime-and-calibrate run, while one is going.
    run: Option<Routine>,
    /// Settling delay after the run's last command, as for the demo.
    run_sent: Option<Instant>,
    /// What the last run ended with, kept on screen after it finishes.
    run_result: Option<Result<routine::Report, String>>,
    layout: schematic::Layout,
    saved_layout: schematic::Layout,
    edit_layout: bool,
    layout_message: Option<String>,
}

impl App {
    pub fn new(cc: &eframe::CreationContext<'_>) -> Self {
        let settings = Settings::load();
        let ctx = cc.egui_ctx.clone();
        let bus = Bus::spawn(Wake::new(move || ctx.request_repaint()), settings.clone());
        let demo = if std::env::args().any(|a| a == "--demo")
            && settings.effective_backend() == Backend::Simulator
        {
            demo_steps()
        } else {
            VecDeque::new()
        };
        let layout = schematic::Layout::load();
        let tracker = Tracker::for_backend(settings.effective_backend());
        Self {
            saved_layout: layout.clone(),
            layout,
            edit_layout: std::env::args().any(|a| a == "--edit-layout"),
            layout_message: None,
            demo,
            demo_wait: None,
            liquid: crate::fluidics::Tracker::default(),
            run: None,
            run_sent: None,
            run_result: None,
            draft: settings.clone(),
            tracker_backend: settings.effective_backend(),
            settings,
            available_ports: list_ports(),
            adopted_server_settings: 0,
            temp_window_secs: TEMP_TREND_SECS,
            temp_setpoint: 25.0,
            temp_tolerance: 0.5,
            temp_setpoint_seen: None,
            temp_tolerance_edited: false,
            tab: Tab::Monitor,
            bus,
            selected: Some(DeviceId::Pp01),
            click_to_actuate: false,
            valve_confirm: None,
            menu_target: None,
            tracker,
            last_slot_poll: None,
            applied_zoom: 1.0,
            pump_inputs: Default::default(),
            form: ProtocolForm::default(),
            ctrl: ControllerView::default(),
            log_filter: String::new(),
            show_trace: false,
            settings_message: None,
            started: Instant::now(),
        }
    }

    fn api(&self, request: Request) {
        self.bus.api(&self.settings.controller_url, request);
    }

    fn drain_api(&mut self) {
        while let Some(reply) = self.bus.next_api_reply() {
            let quiet = reply.request.is_poll();
            match &reply.result {
                Ok((code, body)) => {
                    self.ctrl.reachable = Some(true);
                    let parsed = serde_json::from_str::<serde_json::Value>(body).ok();
                    match reply.request {
                        Request::GetStatus => self.ctrl.status = parsed,
                        Request::GetSlotStatus => self.ctrl.slots = parsed,
                        Request::GetProtocolData => self.ctrl.protocols = parsed,
                        _ => {}
                    }
                    if !quiet {
                        let ok = (200..300).contains(code);
                        self.ctrl.log.push((Instant::now(), ok, format!("{} → {code} {}", reply.summary, body.trim())));
                    }
                }
                Err(e) => {
                    self.ctrl.reachable = Some(false);
                    if !quiet {
                        self.ctrl.log.push((Instant::now(), false, format!("{} → {e}", reply.summary)));
                    }
                }
            }
            if self.ctrl.log.len() > 200 {
                self.ctrl.log.drain(..50);
            }
        }
    }

    /// Advances the demo once the previous device has settled.
    fn step_demo(&mut self, state: &BusState) {
        if let Some((id, sent)) = self.demo_wait {
            let d = state.dev(id);
            let settled = d.target.is_none() && !matches!(d.status, Some(0x04 | 0xFE));
            if sent.elapsed() < Duration::from_millis(700) || !settled {
                return;
            }
            self.demo_wait = None;
        }
        if let Some((id, op)) = self.demo.pop_front() {
            self.selected = Some(id);
            self.bus.device(id, op);
            self.demo_wait = Some((id, Instant::now()));
        }
    }

    /// Advances the prime-and-calibrate run.
    ///
    /// The decision lives in [`crate::routine`]; this only reports what the
    /// rig is doing and posts what comes back. Every command it sends is a
    /// bounded move, so losing this window mid-run leaves the pump stopped at
    /// the end of its current chunk rather than still travelling.
    fn step_routine(&mut self, state: &BusState) {
        let Some(run) = self.run.as_mut() else { return };
        // Same settling delay the demo uses: a device reports its old status
        // for a poll or two after being given a command.
        let pump = state.dev(DeviceId::Pp01);
        let busy = |d: &crate::bus::DeviceLive| d.target.is_some() || matches!(d.status, Some(0x04 | 0xFE));
        let fresh = self.run_sent.is_some_and(|t| t.elapsed() < Duration::from_millis(700));
        let settled = !fresh
            && !busy(pump)
            && !busy(state.dev(DeviceId::Sv01))
            && !busy(state.dev(DeviceId::Sv02));
        let plan = run.plan();
        let obs = routine::Observation {
            pump: pump.value,
            settled,
            // The bus clock, so a reading's age is measured against the rig
            // rather than against however often this window repaints.
            now: state.now(),
            edge_wet: state.sensors.channel(plan.edge_sensor),
            slot_wet: state.sensors.channel(plan.slot_sensor),
            first_wet: state.sensors.channel(plan.first_sensor),
        };
        match run.step(obs) {
            routine::Action::Wait => {}
            routine::Action::Send(id, op) => {
                // Sent one at a time and waited on, including the homing: the
                // order the valves and the pump are reset in is the run's
                // business, not a broadcast.
                self.bus.device(id, op);
                self.run_sent = Some(Instant::now());
            }
            routine::Action::Finished(report) => {
                self.run = None;
                self.run_sent = None;
                // The run exists to measure these; keep them, or the next run
                // starts as ignorant as this one did.
                let ul = self.settings.pp01_ul_per_step;
                self.liquid.model.record_prime(
                    report.to_first_steps.map(|s| s as f32 * ul),
                    report.to_edge_ul(ul),
                    report.edge_to_slot_ul(ul),
                );
                self.liquid.flush();
                self.run_result = Some(Ok(report));
            }
            routine::Action::Abort(why) => {
                self.run = None;
                self.run_sent = None;
                // Whatever went wrong, stop moving liquid.
                self.bus.send(BusCmd::StopAll);
                self.run_result = Some(Err(why));
            }
        }
    }

    /// Folds this frame's pump position and sensor readings into the liquid
    /// model, and saves it now and then.
    fn track_liquid(&mut self, state: &BusState) {
        let rig = &self.settings.rig;
        let channels: Vec<(crate::fluidics::Landmark, u8)> = [
            (SensorAt::Inline1, crate::fluidics::Landmark::S1),
            (SensorAt::Inline2, crate::fluidics::Landmark::S2),
            (SensorAt::Slot, crate::fluidics::Landmark::Slot),
        ]
        .iter()
        .filter_map(|(at, landmark)| rig.sensor_at(*at).map(|s| (*landmark, s.channel)))
        .collect();
        let sensors: [Option<bool>; 6] = std::array::from_fn(|i| state.sensors.channel(i as u8));
        let waste = rig.ports.iter().find(|p| p.role == Role::Waste).map(|p| p.port).unwrap_or(9);
        let fill = rig.ports.iter().find(|p| p.role == Role::SlotFill).map(|p| p.port).unwrap_or(10);
        let air = rig.ports.iter().find(|p| p.role == Role::Air).map(|p| p.port);
        self.liquid.update(
            state.dev(DeviceId::Pp01).value,
            self.settings.pp01_ul_per_step,
            state.dev(DeviceId::Pp01).solenoid_input,
            state.dev(DeviceId::Sv01).value,
            state.dev(DeviceId::Sv02).value,
            waste,
            fill,
            air,
            &sensors,
            &channels,
        );
        self.liquid.maybe_save();
    }

    /// Slot descriptions from controller_v2 (`Idle`, `Missing`, a step name), only while it answers.
    fn slot_descriptions(&self) -> [Option<String>; 6] {
        let slots = self.ctrl.slots.as_ref().and_then(|v| v.as_array()).filter(|_| self.ctrl.reachable == Some(true));
        std::array::from_fn(|i| {
            slots
                .and_then(|s| s.get(i))
                .and_then(|s| s.get("description"))
                .and_then(|d| d.as_str())
                .map(str::to_string)
        })
    }

    fn refresh_controller(&mut self) {
        self.api(Request::GetStatus);
        self.api(Request::GetSlotStatus);
        self.api(Request::GetProtocolData);
        self.form.last_refresh = Some(Instant::now());
    }

    // ───────────────────────────── top bar ─────────────────────────────

    fn top_bar(&mut self, ui: &mut egui::Ui, state: &BusState) {
        ui.horizontal(|ui| {
            for (tab, name) in [
                (Tab::Monitor, "Monitor"),
                (Tab::Protocols, "Protocols"),
                (Tab::Datasheets, "Datasheets"),
                (Tab::Log, "Log"),
                (Tab::Settings, "Settings"),
            ] {
                ui.selectable_value(&mut self.tab, tab, name);
            }
            ui.separator();

            let (dot, text) = match (&state.remote, state.backend, state.connected) {
                (Some(host), Backend::Simulator, true) => {
                    (Color32::from_rgb(160, 120, 230), format!("{host} · simulator"))
                }
                (Some(host), _, true) => (Color32::from_rgb(60, 180, 90), format!("{host} · RS485 {}", self.settings.port)),
                (Some(host), _, false) => (Color32::from_rgb(220, 70, 70), format!("{host} · no link")),
                (None, Backend::Simulator, true) => (Color32::from_rgb(160, 120, 230), "Simulator".to_string()),
                (None, _, true) => (Color32::from_rgb(60, 180, 90), format!("RS485 {}", self.settings.port)),
                (None, _, false) => (Color32::from_rgb(220, 70, 70), "Disconnected".to_string()),
            };
            status_dot(ui, dot);
            ui.label(text).on_hover_text(match &state.remote {
                Some(host) => format!("The pumps and valves are on {host}; this window is a view of them."),
                None => "The pumps and valves are on this machine's bus.".to_string(),
            });
            let online = state.devices.iter().filter(|d| d.online).count();
            ui.label(format!("{online}/5 online"));
            ui.label(RichText::new(format!("cycle {:.0} ms", state.cycle_ms)).weak());

            let mut polling = state.polling;
            if ui.checkbox(&mut polling, "Live polling").changed() {
                self.bus.send(BusCmd::SetPolling(polling));
            }

            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                let stop = egui::Button::new(RichText::new("■ STOP ALL").strong().color(Color32::WHITE))
                    .fill(Color32::from_rgb(180, 35, 35));
                if ui.add(stop).on_hover_text("Send 0x49 forced stop to every pump and valve").clicked() {
                    self.bus.send(BusCmd::StopAll);
                }
                if ui.button("Home all").clicked() {
                    self.bus.send(BusCmd::HomeAll);
                }
                if !self.demo.is_empty() || self.demo_wait.is_some() {
                    if ui.button(format!("Cancel demo ({} left)", self.demo.len())).clicked() {
                        self.demo.clear();
                        self.demo_wait = None;
                    }
                } else if ui
                    .add_enabled(state.backend == Backend::Simulator, egui::Button::new("Demo infill"))
                    .on_hover_text("Simulator only: fill slot 1 from C4, then drain it to waste")
                    .on_disabled_hover_text("Available with the Simulator backend")
                    .clicked()
                {
                    self.demo = demo_steps();
                }
            });
        });
        if let Some(e) = &state.connection_error {
            ui.colored_label(Color32::from_rgb(220, 90, 90), format!("Connection: {e} — check Settings."));
        }
        if state.backend == Backend::Serial && self.ctrl.reachable == Some(true) {
            ui.colored_label(
                Color32::from_rgb(220, 150, 40),
                "controller_v2 is running. If it uses the same RS485 adapter, stop it before polling here: two masters on one bus corrupt each other's frames.",
            );
        }
    }

    // ───────────────────────────── monitor ─────────────────────────────

    /// The wired optical detectors: what they say, and the raw value behind it.
    ///
    /// The raw reading is shown because the state bit cannot distinguish a
    /// detector solidly immersed from one that has just crossed its threshold.
    /// A settled reading holds the same number sample after sample; a front or
    /// a bubble makes it move. No attempt is made here to turn the number into
    /// a state — the two in-line detectors on this rig cross in opposite
    /// directions, so there is no single rule to apply.
    fn sensor_panel(&mut self, ui: &mut egui::Ui, state: &BusState) {
        ui.horizontal(|ui| {
            ui.heading(RichText::new("Liquid sensors").size(17.0));
            if !state.sensors.connected {
                badge(ui, "OFFLINE", Color32::from_rgb(220, 70, 70), 13.0);
            }
        });
        if let Some(e) = &state.sensors.error {
            ui.colored_label(Color32::from_rgb(220, 90, 90), e);
            return;
        }
        if self.settings.rig.sensors.is_empty() {
            ui.label(RichText::new("No detectors configured (Settings → Rig).").small().weak());
            return;
        }
        egui::Grid::new("sensor_table").num_columns(4).striped(true).spacing([12.0, 5.0]).show(ui, |ui| {
            for sensor in &self.settings.rig.sensors {
                ui.label(RichText::new(&sensor.name).size(15.0).strong());
                let reading = crate::sensors::Reading::of(&state.sensors, sensor.channel);
                let (text, colour) = match reading {
                    crate::sensors::Reading::Liquid => ("LIQUID", Color32::from_rgb(64, 150, 240)),
                    crate::sensors::Reading::Dry => ("dry", Color32::from_rgb(140, 150, 165)),
                    crate::sensors::Reading::Unknown => ("—", Color32::from_rgb(110, 115, 125)),
                };
                badge(ui, text, colour, 13.0);
                match state.sensors.adc_at(sensor.channel) {
                    Some(raw) => ui.label(RichText::new(format!("raw {raw:>3}")).monospace().size(14.0)),
                    None => ui.label(RichText::new("raw —").monospace().size(14.0).weak()),
                };
                ui.label(
                    RichText::new(match sensor.at {
                        SensorAt::Inline1 => "pump → coil",
                        SensorAt::Inline2 => "coil → SV02",
                        SensorAt::Slot => "slot feed",
                    })
                    .small()
                    .weak(),
                );
                ui.end_row();
            }
        });
        if let (Some(max), Some(min)) = (state.sensors.max_threshold, state.sensors.min_threshold) {
            ui.label(
                RichText::new(format!(
                    "board thresholds {max} / {min} · a settled reading holds still, a front or a bubble moves it",
                ))
                .small()
                .weak(),
            );
        }
    }

    /// Prime the wash line and measure what it holds.
    ///
    /// Deliberately a button rather than something that runs on its own: it
    /// moves real liquid, so it starts when someone is watching the rig.
    fn prime_panel(&mut self, ui: &mut egui::Ui, state: &BusState) {
        let running = self.run.is_some();
        ui.horizontal(|ui| {
            ui.heading(RichText::new("Prime & calibrate").size(17.0));
            if let Some(run) = &self.run {
                badge(ui, run.stage().label(), Color32::from_rgb(64, 150, 240), 13.0);
            }
        });

        let plan = routine::Plan::for_rig(&self.settings.rig);
        let ready = state.connected && state.sensors.connected && !running;
        match &plan {
            Ok(p) => {
                if !running {
                    ui.label(
                        RichText::new(format!(
                            "Draws wash from SV01 port {}, pushes it to S2, then on into the slot through SV02 port {}, \
                             and reports what each leg took. Approaches at {} rpm and creeps the last of it at {}, \
                             the speeds controller_v2 moves liquid at. Stops on its own after {} µL a leg.",
                            p.wash_port,
                            p.fill_port,
                            p.coarse_speed,
                            p.fine_speed,
                            (p.cap_steps as f32 * self.settings.pp01_ul_per_step).round(),
                        ))
                        .small()
                        .weak(),
                    );
                }
            }
            Err(why) => {
                ui.colored_label(Color32::from_rgb(220, 160, 40), why);
            }
        }

        if let Some(run) = &self.run {
            ui.add(
                egui::ProgressBar::new(run.leg_fraction())
                    .desired_width(ui.available_width().min(300.0))
                    .text(RichText::new(run.stage().label()).size(12.0)),
            );
        }

        ui.horizontal(|ui| {
            let start = egui::Button::new(RichText::new("Prime & calibrate").strong());
            let response = ui.add_enabled(ready && plan.is_ok(), start);
            let response = if !state.sensors.connected {
                response.on_disabled_hover_text("The optical sensor board is not answering; this run stops on its sensors.")
            } else {
                response.on_hover_text("Moves real liquid. Watch the rig while it runs.")
            };
            if response.clicked()
                && let Ok(p) = plan
            {
                self.run_result = None;
                self.run_sent = None;
                self.run = Some(Routine::new(p));
            }
            if ui
                .add_enabled(running, egui::Button::new(RichText::new("Abort").color(Color32::from_rgb(225, 80, 80))))
                .clicked()
            {
                self.run = None;
                self.run_sent = None;
                self.bus.send(BusCmd::StopAll);
                self.run_result = Some(Err("aborted".into()));
            }
        });

        match &self.run_result {
            Some(Ok(report)) => {
                let ul = self.settings.pp01_ul_per_step;
                let bore = self.liquid.model.bore_id_mm;
                let mm = |v: f32| crate::fluidics::length_mm(v, bore);
                if let Some(first) = report.to_first_steps.map(|s| s as f32 * ul) {
                    ui.label(RichText::new(format!("PP01 → S1: {first:.0} µL · {:.0} mm", mm(first))).size(14.0));
                    let rest = report.to_edge_ul(ul) - first;
                    ui.label(RichText::new(format!("S1 → S2: {rest:.0} µL · {:.0} mm", mm(rest))).size(14.0));
                }
                ui.label(
                    RichText::new(format!(
                        "PP01 → S2: {:.0} µL · {:.0} mm ({} steps{})",
                        report.to_edge_ul(ul),
                        mm(report.to_edge_ul(ul)),
                        report.to_edge_steps,
                        if report.edge_precise { "" } else { ", coarse only" },
                    ))
                    .size(14.0),
                );
                let slot = format!(
                    "S2 → slot: {:.0} µL · {:.0} mm ({} steps)",
                    report.edge_to_slot_ul(ul),
                    mm(report.edge_to_slot_ul(ul)),
                    report.edge_to_slot_steps
                );
                if report.slot_timed_out {
                    ui.colored_label(
                        Color32::from_rgb(220, 160, 40),
                        format!("{slot} — the slot sensor never wetted, so this is how far it got"),
                    );
                } else {
                    ui.label(RichText::new(slot).size(14.0));
                }
                // A sensor changing its mind with the piston stopped is
                // something physically moving past it.
                match report.bubbles_seen {
                    0 => ui.label(RichText::new("no bubbles seen").small().weak()),
                    n => ui.colored_label(
                        Color32::from_rgb(220, 160, 40),
                        format!("{n} sensor contradiction(s) with the piston still — bubbles; figures are approximate"),
                    ),
                };
            }
            Some(Err(why)) => {
                ui.colored_label(Color32::from_rgb(225, 80, 80), why);
            }
            None => {}
        }
    }

    /// The Peltier slot-temperature board: what it reads, and the two things
    /// worth changing from here — the setpoint and whether the PID owns the
    /// output. Gains and autotune stay with the board's own tooling.
    fn temperature_panel(&mut self, ui: &mut egui::Ui, state: &BusState) {
        let t = &state.temp;
        let status = t.status();
        let slot = self.settings.rig.only_slot();
        let title = match slot {
            Some(n) => format!("Temperature — slot {n}"),
            None => "Temperature".to_string(),
        };
        ui.horizontal(|ui| {
            ui.heading(RichText::new(title).size(17.0));
            badge(ui, status.label(), temp_status_color(status), 13.0);
        });

        if !t.enabled {
            ui.label(
                RichText::new("No temperature board configured. Settings → Temperature.")
                    .small()
                    .weak(),
            );
            return;
        }
        if let Some(e) = &t.error {
            ui.colored_label(Color32::from_rgb(220, 90, 90), e);
        }

        // The reading, big enough to read across a bench.
        ui.horizontal(|ui| {
            let reading = t.temperature_c.map(|c| format!("{c:.2} °C")).unwrap_or_else(|| "—".into());
            ui.label(RichText::new(reading).size(34.0).strong());
            ui.vertical(|ui| {
                let target = t.setpoint_c.map(|c| format!("target {c:.2} °C")).unwrap_or_else(|| "no target".into());
                ui.label(RichText::new(target).size(15.0));
                if let (Some(err), Some(tol)) = (t.error_c(), t.tolerance_c) {
                    ui.label(
                        RichText::new(format!("{err:+.2} °C to go · band ±{tol:.2}"))
                            .size(13.0)
                            .weak(),
                    );
                }
            });
        });

        if t.sensor_fault {
            ui.colored_label(
                Color32::from_rgb(225, 80, 80),
                "RTD fault: the reading is not trustworthy and the board will not regulate.",
            );
        }
        if !t.active_faults.is_empty() {
            ui.colored_label(Color32::from_rgb(225, 80, 80), format!("Active: {}", t.active_faults));
        }
        if !t.latched_faults.is_empty() {
            ui.colored_label(
                Color32::from_rgb(220, 160, 40),
                format!("Latched since last clear: {}", t.latched_faults),
            );
        }

        temp_trend(ui, &t.history, state.now(), self.temp_window_secs);
        ui.horizontal(|ui| {
            ui.label(RichText::new("window").small().weak());
            for minutes in [5.0f64, 10.0] {
                let secs = minutes * 60.0;
                let on = (self.temp_window_secs - secs).abs() < 1.0;
                if ui.selectable_label(on, RichText::new(format!("{minutes:.0} min")).small()).clicked() {
                    self.temp_window_secs = secs;
                }
            }
        });

        egui::Grid::new("temp_readout").num_columns(2).spacing([14.0, 4.0]).show(ui, |ui| {
            // Signed duty: the H-bridge reverses to cool, so the sign is the
            // difference between heating and cooling, not a rounding artefact.
            param(
                ui,
                "Bridge duty",
                t.duty_pct
                    .map(|d| format!("{d:+.1} % ({})", if d > 0.0 { "heating" } else if d < 0.0 { "cooling" } else { "idle" }))
                    .unwrap_or_else(|| "—".into()),
            );
            param(ui, "Current", t.current_a.map(|a| format!("{a:.2} A")).unwrap_or_else(|| "—".into()));
            if t.autotuning {
                param(ui, "Autotune", "running".to_string());
            }
        });

        // Follow the board's setpoint until the operator starts typing, so the
        // box shows what is actually set rather than a stale local guess.
        if t.setpoint_c != self.temp_setpoint_seen {
            if let Some(c) = t.setpoint_c {
                self.temp_setpoint = c;
            }
            self.temp_setpoint_seen = t.setpoint_c;
        }
        if let Some(tol) = t.tolerance_c
            && (tol - self.temp_tolerance).abs() > 0.001
            && !self.temp_tolerance_edited
        {
            self.temp_tolerance = tol;
        }

        let live = t.connected;
        ui.add_enabled_ui(live, |ui| {
            ui.horizontal(|ui| {
                ui.label("Set");
                ui.add(
                    egui::DragValue::new(&mut self.temp_setpoint)
                        .range(TEMP_MIN_C..=TEMP_MAX_C)
                        .speed(0.1)
                        .suffix(" °C"),
                );
                if ui.button("Apply").on_hover_text("Send the setpoint to the board").clicked() {
                    self.bus.send(BusCmd::Temp(TempOp::SetTemperature(self.temp_setpoint)));
                }
                ui.label("±");
                let tol = ui.add(
                    egui::DragValue::new(&mut self.temp_tolerance).range(0.05..=10.0).speed(0.05).suffix(" °C"),
                );
                if tol.changed() {
                    self.temp_tolerance_edited = true;
                }
                if ui.add_enabled(self.temp_tolerance_edited, egui::Button::new("Set band")).clicked() {
                    self.bus.send(BusCmd::Temp(TempOp::SetTolerance(self.temp_tolerance)));
                    self.temp_tolerance_edited = false;
                }
            });
            ui.horizontal(|ui| {
                if t.pid_running {
                    let stop = egui::Button::new(RichText::new("Stop holding").strong())
                        .fill(Color32::from_rgb(150, 60, 40));
                    if ui.add(stop).on_hover_text("Stop regulating and brake the bridge").clicked() {
                        self.bus.send(BusCmd::Temp(TempOp::StopPid));
                    }
                } else if ui
                    .add_enabled(!t.sensor_fault, egui::Button::new(RichText::new("Hold temperature").strong()))
                    .on_hover_text("Hand the output to the board's PID")
                    .on_disabled_hover_text("The board refuses to regulate while the RTD is in fault")
                    .clicked()
                {
                    self.bus.send(BusCmd::Temp(TempOp::StartPid));
                }
                if ui
                    .add_enabled(t.has_latched(), egui::Button::new("Clear faults"))
                    .on_hover_text("Drop the latched fault history")
                    .clicked()
                {
                    self.bus.send(BusCmd::Temp(TempOp::ClearErrors));
                }
            });
        });
    }

    fn monitor_side(&mut self, ui: &mut egui::Ui, state: &BusState, fluid: &FluidView) {
        egui::ScrollArea::vertical().auto_shrink([false, false]).show(ui, |ui| {
            ui.heading("Devices");
            egui::Grid::new("dev_table").num_columns(3).striped(true).spacing([14.0, 10.0]).show(ui, |ui| {
                for id in DeviceId::ALL {
                    let d = state.dev(id);
                    let st = d.state();
                    let sel = self.selected == Some(id);
                    if ui.selectable_label(sel, RichText::new(id.tag()).strong().size(18.0)).clicked() {
                        self.selected = Some(id);
                    }
                    state_badge(ui, st, 14.0);
                    let value = match (id.kind(), d.value) {
                        (_, None) => "—".to_string(),
                        (Kind::Pump { .. }, Some(v)) => format!("{v} st · {:.0} µL", v as f32 * self.settings.ul_per_step(id)),
                        (Kind::Valve { .. }, Some(0)) => "reset".to_string(),
                        (Kind::Valve { .. }, Some(p)) => {
                            let label = self.settings.rig.port_label(id, p);
                            format!("port {p} · {}", label.split(" (").next().unwrap_or(""))
                        }
                    };
                    ui.vertical(|ui| {
                        ui.label(RichText::new(value).size(17.0));
                        let detail = if d.online { d.status.map(protocol::status_text).unwrap_or("…") } else { "no reply" };
                        ui.label(RichText::new(detail).size(13.0).color(state_color(st)));
                    });
                    ui.end_row();
                }
            });

            ui.add_space(6.0);
            egui::CollapsingHeader::new(RichText::new("Slots & reagents").size(17.0)).default_open(true).show(ui, |ui| {
                let descriptions = self.slot_descriptions();
                egui::Grid::new("slot_table").num_columns(3).striped(true).spacing([12.0, 6.0]).show(ui, |ui| {
                    for (i, (st, vol)) in fluid.slots.iter().enumerate() {
                        let slot = i as u16 + 1;
                        if !self.settings.rig.slot_fitted(slot) {
                            continue;
                        }
                        let name = match self.settings.rig.slot_sensor_name() {
                            Some(sensor) => format!("Slot {slot} ({sensor})"),
                            None => format!("Slot {slot}"),
                        };
                        ui.label(RichText::new(name).size(15.0).strong());
                        badge(ui, st.label(), st.color(), 13.0);
                        let note = descriptions[i].as_deref().filter(|d| !d.eq_ignore_ascii_case("idle")).map(|d| format!(" · {d}")).unwrap_or_default();
                        ui.label(RichText::new(format!("~{vol:.0} µL{note}")).size(14.0));
                        ui.end_row();
                    }
                });
                ui.add_space(4.0);
                egui::Grid::new("bottle_table").num_columns(3).striped(true).spacing([12.0, 6.0]).show(ui, |ui| {
                    for (i, (st, ml)) in fluid.bottles.iter().enumerate() {
                        if !self.settings.rig.bottle_in_use(i as u16 + 1) {
                            continue;
                        }
                        let cap = BOTTLE_CAPACITY_ML[i];
                        ui.label(RichText::new(format!("C{}", i + 1)).size(15.0).strong());
                        badge(ui, st.label(), st.color(), 13.0);
                        ui.add(
                            egui::ProgressBar::new((ml / cap).clamp(0.0, 1.0))
                                .desired_width(180.0)
                                .text(RichText::new(format!("{ml:.0} / {cap:.0} mL")).size(13.0)),
                        );
                        ui.end_row();
                    }
                });
                ui.label(RichText::new("Levels are estimated from piston travel on connected paths; correct them from the right-click menu.").small().weak());
            });

            ui.separator();
            ui.add_enabled(
                !self.edit_layout,
                egui::Checkbox::new(&mut self.click_to_actuate, "Click a port on the schematic to switch the valve"),
            )
            .on_hover_text("Off: clicks only select devices. On: clicking a port asks to switch that valve.");
            ui.horizontal(|ui| {
                ui.toggle_value(&mut self.edit_layout, "Move objects")
                    .on_hover_text("Drag components on the schematic to rearrange them. Valve ports are not clickable while this is on.");
                let dirty = self.layout != self.saved_layout;
                if ui.add_enabled(dirty, egui::Button::new("Save layout")).clicked() {
                    self.save_layout();
                }
                if ui.add_enabled(dirty, egui::Button::new("Revert")).clicked() {
                    self.layout = self.saved_layout.clone();
                }
                if ui.button("Default layout").clicked() {
                    self.layout = schematic::Layout::default();
                }
                if dirty {
                    ui.colored_label(Color32::from_rgb(220, 160, 40), "unsaved");
                }
            });
            if let Some(msg) = &self.layout_message {
                ui.label(RichText::new(msg).small());
            }
            ui.separator();
            self.sensor_panel(ui, state);
            ui.separator();
            self.prime_panel(ui, state);
            ui.separator();
            self.temperature_panel(ui, state);
            ui.separator();

            if let Some(id) = self.selected {
                ui.heading(format!("{} — {}", id.tag(), id.role()));
                ui.label(RichText::new(id.spec().part_number).monospace().weak());
                let d = state.dev(id).clone();
                if let Some(e) = &d.last_error {
                    ui.colored_label(Color32::from_rgb(220, 90, 90), e);
                }
                ui.label(format!(
                    "Address {} · errors {} · last reply {}",
                    self.settings.addr(id),
                    d.errors,
                    d.last_seen.map(|t| format!("{:.1} s ago", state.now() - t)).unwrap_or("never".into())
                ));
                match id.kind() {
                    Kind::Valve { ports } => self.valve_controls(ui, id, ports, &d),
                    Kind::Pump { has_solenoid } => self.pump_controls(ui, id, has_solenoid, &d, state),
                }
            }
        });
    }

    fn valve_controls(&mut self, ui: &mut egui::Ui, id: DeviceId, ports: u16, d: &crate::bus::DeviceLive) {
        ui.add_space(4.0);
        let st = d.state();
        egui::Frame::group(ui.style()).inner_margin(egui::Margin::symmetric(10, 8)).show(ui, |ui| {
            ui.set_width(ui.available_width());
            ui.horizontal(|ui| {
                state_badge(ui, st, 18.0);
                if let Some(t) = d.target.filter(|t| Some(*t) != d.value) {
                    ui.label(RichText::new(format!("switching to {t}")).size(18.0).color(state_color(DevState::Moving)));
                }
            });
            let (headline, detail) = match d.value {
                Some(0) => ("Reset".to_string(), "common port closed".to_string()),
                Some(p) => (format!("Port {p}"), self.settings.rig.port_label(id, p)),
                None => ("—".to_string(), "position unknown".to_string()),
            };
            ui.label(RichText::new(headline).size(34.0).strong());
            ui.label(RichText::new(detail).size(20.0));
            egui::Grid::new(("valve_params", id)).num_columns(2).spacing([18.0, 6.0]).show(ui, |ui| {
                param(ui, "Motor status", d.status.map(|s| format!("{} (0x{s:02X})", protocol::status_text(s))).unwrap_or_else(|| "—".into()));
                param(ui, "Ports", format!("{ports} + common"));
            });
        });
        let cols: u16 = if ports == 16 { 2 } else { 1 };
        let rig = self.settings.rig.clone();
        egui::Grid::new(("ports", id)).num_columns(cols as usize * 2).spacing([6.0, 4.0]).show(ui, |ui| {
            for p in 1..=ports {
                let current = d.value == Some(p);
                let allowed = rig.port_allowed(id, p);
                let plumbed = rig.port_plumbed(id, p);
                let btn = egui::Button::new(RichText::new(format!("{p:>2}")).monospace()).selected(current);
                let response = ui.add_enabled(allowed, btn);
                let response = match rig.blocked_reason(id, p) {
                    Some(why) => response.on_disabled_hover_text(why),
                    None => response,
                };
                if response.clicked() {
                    self.bus.device(id, Op::ValveTo(p));
                }
                // A capped line stays listed but reads as absent, so the port
                // numbering still matches the valve in front of you.
                let label = RichText::new(rig.port_label(id, p)).small();
                ui.label(if plumbed { label } else { label.weak().strikethrough() });
                if p % cols == 0 {
                    ui.end_row();
                }
            }
        });
        ui.horizontal(|ui| {
            if ui.button("Reset / home").clicked() {
                self.bus.device(id, Op::Home);
            }
            if ui.button("Stop").clicked() {
                self.bus.device(id, Op::Stop);
            }
            if ui.button("Firmware version").clicked() {
                self.bus.device(id, Op::QueryVersion);
            }
        });
        if let Some(v) = d.version {
            ui.label(format!("Firmware V{}.{}", v & 0xFF, v >> 8));
        }
    }

    fn pump_controls(&mut self, ui: &mut egui::Ui, id: DeviceId, has_solenoid: bool, d: &crate::bus::DeviceLive, state: &BusState) {
        let max = self.settings.max_steps(id);
        let ul = self.settings.ul_per_step(id);
        let idx = if id == DeviceId::Pp01 { 0 } else { 1 };
        let pos = d.value.unwrap_or(0);

        let st = d.state();
        let frac = (pos as f32 / max.max(1) as f32).clamp(0.0, 1.0);
        let rate = d.steps_per_sec() * ul;
        let motion = match (d.online, d.motion) {
            (false, _) => "no reply",
            (true, m) if m > 0 => "aspirating",
            (true, m) if m < 0 => "dispensing",
            _ => "idle",
        };
        egui::Frame::group(ui.style()).inner_margin(egui::Margin::symmetric(10, 8)).show(ui, |ui| {
            ui.set_width(ui.available_width());
            ui.horizontal(|ui| {
                state_badge(ui, st, 18.0);
                ui.label(RichText::new(motion).size(18.0));
            });
            ui.horizontal(|ui| {
                ui.label(RichText::new(if d.online { pos.to_string() } else { "—".into() }).size(34.0).strong());
                ui.label(RichText::new(format!("/ {max} steps")).size(16.0).weak());
            });
            ui.label(RichText::new(format!("{:.1} µL", pos as f32 * ul)).size(26.0).strong().color(Color32::from_rgb(64, 150, 240)));
            ui.add(
                egui::ProgressBar::new(frac)
                    .desired_height(18.0)
                    .text(RichText::new(format!("{:.0}% of {:.0} µL", frac * 100.0, max as f32 * ul)).size(14.0)),
            );
            egui::Grid::new(("pump_params", id)).num_columns(2).spacing([18.0, 6.0]).show(ui, |ui| {
                param(ui, "Flow rate", if d.motion == 0 { "0 µL/s".into() } else { format!("{rate:+.0} µL/s") });
                param(ui, "Target", d.target.map(|t| format!("{t} st · {:.1} µL", t as f32 * ul)).unwrap_or_else(|| "—".into()));
                param(ui, "Speed", d.speed.map(|s| format!("{s} rpm")).unwrap_or_else(|| "not set this session".into()));
                if has_solenoid {
                    let sol = match d.solenoid_input {
                        Some(true) => "input · SV01 side",
                        Some(false) => "output · coil / SV02",
                        None => "unknown",
                    };
                    param(ui, "Solenoid", sol.into());
                }
                param(ui, "Motor status", d.status.map(|s| format!("{} (0x{s:02X})", protocol::status_text(s))).unwrap_or_else(|| "—".into()));
                param(ui, "Resolution", format!("{ul:.3} µL/step"));
            });
        });
        trend(ui, &d.history, max as f32, state.now());

        let inputs = &mut self.pump_inputs[idx];
        let mut actions: Vec<Op> = Vec::new();
        ui.horizontal(|ui| {
            ui.label("Target");
            ui.add(egui::DragValue::new(&mut inputs.target).range(0..=max).suffix(" st"));
            ui.label(format!("= {:.1} µL", inputs.target as f32 * ul));
            if ui.button("Go").clicked() {
                actions.push(Op::PumpTo(inputs.target));
            }
        });
        ui.horizontal(|ui| {
            ui.checkbox(&mut inputs.by_volume, "by volume");
            if inputs.by_volume {
                ui.add(egui::DragValue::new(&mut inputs.volume_ul).range(0.0..=max as f32 * ul).suffix(" µL"));
                inputs.steps = (inputs.volume_ul / ul.max(1e-6)).round().clamp(0.0, max as f32) as u16;
                ui.label(format!("= {} st", inputs.steps));
            } else {
                ui.add(egui::DragValue::new(&mut inputs.steps).range(0..=max).suffix(" st"));
                ui.label(format!("= {:.1} µL", inputs.steps as f32 * ul));
            }
        });
        ui.horizontal(|ui| {
            if ui.button("⬆ Aspirate").on_hover_text("0x4D: move away from home").clicked() {
                actions.push(Op::Aspirate(inputs.steps));
            }
            if ui.button("⬇ Dispense").on_hover_text("0x42: move towards home").clicked() {
                actions.push(Op::Dispense(inputs.steps));
            }
        });
        ui.horizontal(|ui| {
            ui.label("Speed");
            ui.add(egui::Slider::new(&mut inputs.speed, 1..=500).suffix(" rpm"));
            if ui.button("Set").clicked() {
                actions.push(Op::SetSpeed(inputs.speed));
            }
        });
        ui.label(
            RichText::new(format!(
                "Full stroke at this speed ≈ {:.1} s (datasheet: 2.2 s at 500 rpm)",
                2.2 * 500.0 / inputs.speed.max(1) as f32 * max as f32 / 3820.0
            ))
            .small()
            .weak(),
        );
        if has_solenoid {
            ui.horizontal(|ui| {
                ui.label("Solenoid");
                let cur = d.solenoid_input;
                if ui.add(egui::Button::new("Input (SV01 side)").selected(cur == Some(true))).clicked() {
                    actions.push(Op::SolenoidInput(true));
                }
                if ui.add(egui::Button::new("Output (coil / SV02)").selected(cur == Some(false))).clicked() {
                    actions.push(Op::SolenoidInput(false));
                }
            });
            ui.label(RichText::new("Solenoid state is not readable back; shown as last command sent.").small().weak());
        }
        ui.horizontal(|ui| {
            if ui.button("Home").clicked() {
                actions.push(Op::Home);
            }
            if ui.button("Forced reset").on_hover_text("0x4F: stall to top, back off an offset — protects the seal").clicked() {
                actions.push(Op::ForcedReset);
            }
            if ui.add(egui::Button::new("Stop").fill(Color32::from_rgb(150, 40, 40))).clicked() {
                actions.push(Op::Stop);
            }
        });
        ui.horizontal(|ui| {
            if ui.button("Sync position").on_hover_text("0x67: after power loss mid-move").clicked() {
                actions.push(Op::SyncPosition);
            }
            if ui.button("Firmware version").clicked() {
                actions.push(Op::QueryVersion);
            }
        });
        if let Some(v) = d.version {
            ui.label(format!("Firmware V{}.{}", v & 0xFF, v >> 8));
        }
        for op in actions {
            self.bus.device(id, op);
        }
    }

    fn monitor_central(&mut self, ui: &mut egui::Ui, state: &BusState, fluid: &FluidView) {
        let time = self.started.elapsed().as_secs_f64();
        let resp = schematic::show(ui, state, &self.settings, fluid, &self.liquid.model, self.selected, time, &mut self.layout, self.edit_layout);
        if resp.layout_changed {
            self.layout_message = None;
        }
        if let Some(id) = resp.clicked_device {
            self.selected = Some(id);
        }
        if let Some((id, port)) = resp.clicked_port {
            self.selected = Some(id);
            if self.click_to_actuate {
                self.valve_confirm = Some((id, port));
            }
        }
        if let Some(target) = resp.menu_target {
            self.menu_target = Some(target);
            if let Target::Device(id) | Target::Port(id, _) = target {
                self.selected = Some(id);
            }
        }
        for r in &resp.menu_responses {
            r.context_menu(|ui| {
                if let Some(target) = self.menu_target {
                    self.schematic_menu(ui, target, state);
                }
            });
        }
    }

    // ───────────────────────────── right-click menu ─────────────────────────────

    /// A menu entry that switches a valve, greyed out when the rig says that
    /// port is capped. The guard on the bus would refuse it anyway; this says
    /// so before the click rather than after.
    fn port_menu_button(&mut self, ui: &mut egui::Ui, id: DeviceId, port: u16, text: String) -> bool {
        let allowed = self.settings.rig.port_allowed(id, port);
        let response = ui.add_enabled(allowed, egui::Button::new(text));
        let response = match self.settings.rig.blocked_reason(id, port) {
            Some(why) => response.on_disabled_hover_text(why),
            None => response,
        };
        if response.clicked() {
            self.bus.device(id, Op::ValveTo(port));
            return true;
        }
        false
    }

    fn schematic_menu(&mut self, ui: &mut egui::Ui, target: Target, state: &BusState) {
        ui.set_min_width(240.0);
        let heading = |ui: &mut egui::Ui, text: String| {
            ui.label(RichText::new(text).strong().size(15.0));
            ui.separator();
        };
        match target {
            Target::Port(id, port) => {
                heading(ui, format!("{} port {port} · {}", id.tag(), self.settings.rig.port_label(id, port)));
                let current = state.dev(id).value == Some(port);
                let allowed = self.settings.rig.port_allowed(id, port);
                let button = egui::Button::new(format!("Switch {} to port {port}", id.tag()));
                let response = ui.add_enabled(!current && allowed, button);
                let response = match self.settings.rig.blocked_reason(id, port) {
                    Some(why) => response.on_disabled_hover_text(why),
                    None => response,
                };
                if response.clicked() {
                    self.bus.device(id, Op::ValveTo(port));
                }
                ui.separator();
                self.valve_menu(ui, id, state);
            }
            Target::Device(id) => {
                heading(ui, format!("{} · {}", id.tag(), id.role()));
                if id.is_pump() {
                    self.pump_menu(ui, id, state);
                } else {
                    self.valve_menu(ui, id, state);
                }
            }
            Target::Slot(n) => {
                heading(ui, format!("Slot {n}"));
                let (fill, drain) = (sv02_port_for_slot(n), sv03_port_for_slot(n));
                self.port_menu_button(ui, DeviceId::Sv02, fill, format!("Select fill path · SV02 → port {fill}"));
                self.port_menu_button(ui, DeviceId::Sv03, drain, format!("Select drain path · SV03 → port {drain}"));
                ui.separator();
                if ui.button(format!("New protocol for slot {n}…")).clicked() {
                    self.form.slot = n;
                    self.tab = Tab::Protocols;
                }
                ui.separator();
                let descriptions = self.slot_descriptions();
                let st = self.tracker.slot_state(n, state, descriptions[n as usize - 1].as_deref());
                let vol = self.tracker.levels.slots_ul[n as usize - 1];
                ui.horizontal(|ui| {
                    badge(ui, st.label(), st.color(), 13.0);
                    ui.label(format!("~{vol:.0} µL (estimated)"));
                });
                if ui.button("Mark empty").clicked() {
                    self.tracker.set_slot(n, 0.0);
                }
                if ui.button(format!("Mark full · {:.0} µL", self.settings.slot_volume_ul)).clicked() {
                    self.tracker.set_slot(n, self.settings.slot_volume_ul);
                }
            }
            Target::Bottle(c) => {
                heading(ui, format!("C{c} · {} external reagent", if c <= 3 { "1 L" } else { "0.5 L" }));
                // The solenoid only moves if the port is reachable, so the
                // order is: check the port, then set the side, then switch.
                if self.settings.rig.port_allowed(DeviceId::Sv02, c)
                    && ui
                        .button(format!("Connect to PP01 via SV02 port {c}"))
                        .on_hover_text("Solenoid → output (coil side), then SV02 → this bottle's port")
                        .clicked()
                {
                    self.bus.device(DeviceId::Pp01, Op::SolenoidInput(false));
                    self.bus.device(DeviceId::Sv02, Op::ValveTo(c));
                }
                if c <= 3
                    && self.settings.rig.port_allowed(DeviceId::Sv01, c)
                    && ui
                        .button(format!("Connect to PP01 via SV01 port {c}"))
                        .on_hover_text("Solenoid → input (SV01 side), then SV01 → this bottle's port")
                        .clicked()
                {
                    self.bus.device(DeviceId::Pp01, Op::SolenoidInput(true));
                    self.bus.device(DeviceId::Sv01, Op::ValveTo(c));
                }
                ui.separator();
                let cap = BOTTLE_CAPACITY_ML[c as usize - 1];
                let st = self.tracker.bottle_state(c, state);
                let ml = self.tracker.levels.bottles_ml[c as usize - 1];
                ui.horizontal(|ui| {
                    badge(ui, st.label(), st.color(), 13.0);
                    ui.label(format!("~{ml:.0} of {cap:.0} mL (estimated)"));
                });
                if ui.button("Mark full (refilled)").clicked() {
                    self.tracker.set_bottle(c, cap);
                }
                ui.menu_button("Set level", |ui| {
                    for pct in [100.0, 75.0, 50.0, 25.0, 10.0, 0.0] {
                        if ui.button(format!("{pct:.0}% · {:.0} mL", cap * pct / 100.0)).clicked() {
                            self.tracker.set_bottle(c, cap * pct / 100.0);
                        }
                    }
                });
            }
            Target::Tag(_, id, port) => {
                heading(ui, format!("{} · {} port {port}", self.settings.rig.port_label(id, port), id.tag()));
                self.port_menu_button(ui, id, port, format!("Switch {} to port {port}", id.tag()));
            }
            Target::Part(key) => heading(ui, part_title(key)),
            Target::Background => {
                heading(ui, "Schematic".to_string());
                if ui.button(RichText::new("Stop all").color(Color32::from_rgb(225, 80, 80))).clicked() {
                    self.bus.send(BusCmd::StopAll);
                }
                if ui.button("Home all").clicked() {
                    self.bus.send(BusCmd::HomeAll);
                }
                ui.checkbox(&mut self.click_to_actuate, "Click ports to switch valves");
            }
        }

        ui.separator();
        if let Some(key) = target.layout_key()
            && ui
                .add_enabled(self.layout.pos(&key) != schematic::Layout::default().pos(&key), egui::Button::new("Reset position"))
                .clicked()
        {
            self.layout.reset(&key);
        }
        ui.checkbox(&mut self.edit_layout, "Move objects");
        if ui.add_enabled(self.layout != self.saved_layout, egui::Button::new("Save layout")).clicked() {
            self.save_layout();
        }
        if ui.button("Default layout").clicked() {
            self.layout = schematic::Layout::default();
        }
    }

    fn valve_menu(&mut self, ui: &mut egui::Ui, id: DeviceId, state: &BusState) {
        let Kind::Valve { ports } = id.kind() else { return };
        let current = state.dev(id).value;
        let rig = self.settings.rig.clone();
        ui.menu_button("Switch to port", |ui| {
            for p in 1..=ports {
                let label = format!("{p:>2} · {}", rig.port_label(id, p));
                let allowed = rig.port_allowed(id, p);
                let button = egui::Button::new(label).selected(current == Some(p));
                let response = ui.add_enabled(allowed, button);
                let response = match rig.blocked_reason(id, p) {
                    Some(why) => response.on_disabled_hover_text(why),
                    None => response,
                };
                if response.clicked() {
                    self.bus.device(id, Op::ValveTo(p));
                }
            }
        });
        if ui.button("Reset / home").clicked() {
            self.bus.device(id, Op::Home);
        }
        if ui.button("Stop").clicked() {
            self.bus.device(id, Op::Stop);
        }
        if ui.button("Firmware version").clicked() {
            self.bus.device(id, Op::QueryVersion);
        }
        self.device_menu_footer(ui, id, state);
    }

    fn pump_menu(&mut self, ui: &mut egui::Ui, id: DeviceId, state: &BusState) {
        let Kind::Pump { has_solenoid } = id.kind() else { return };
        let ul = self.settings.ul_per_step(id).max(1e-6);
        let max = self.settings.max_steps(id);
        let steps = move |v: f32| (v / ul).round().clamp(0.0, max as f32) as u16;
        let d = state.dev(id);
        ui.menu_button("Aspirate", |ui| {
            for v in [10.0, 50.0, 100.0, 500.0, 1000.0] {
                if ui.button(format!("{v:.0} µL · {} st", steps(v))).clicked() {
                    self.bus.device(id, Op::Aspirate(steps(v)));
                }
            }
        });
        ui.menu_button("Dispense", |ui| {
            for v in [10.0, 50.0, 100.0, 500.0, 1000.0] {
                if ui.button(format!("{v:.0} µL · {} st", steps(v))).clicked() {
                    self.bus.device(id, Op::Dispense(steps(v)));
                }
            }
        });
        ui.menu_button("Move to", |ui| {
            for pct in [0u32, 25, 50, 75, 100] {
                let st = (max as u32 * pct / 100) as u16;
                if ui.button(format!("{pct}% · {st} st · {:.0} µL", st as f32 * ul)).clicked() {
                    self.bus.device(id, Op::PumpTo(st));
                }
            }
        });
        ui.menu_button("Speed", |ui| {
            for rpm in [50u16, 100, 200, 300, 500] {
                if ui.add(egui::Button::new(format!("{rpm} rpm")).selected(d.speed == Some(rpm))).clicked() {
                    self.bus.device(id, Op::SetSpeed(rpm));
                }
            }
        });
        if has_solenoid {
            ui.menu_button("Solenoid", |ui| {
                if ui.add(egui::Button::new("Input · SV01 side").selected(d.solenoid_input == Some(true))).clicked() {
                    self.bus.device(id, Op::SolenoidInput(true));
                }
                if ui.add(egui::Button::new("Output · coil / SV02").selected(d.solenoid_input == Some(false))).clicked() {
                    self.bus.device(id, Op::SolenoidInput(false));
                }
            });
        }
        ui.separator();
        if ui.button("Home").clicked() {
            self.bus.device(id, Op::Home);
        }
        if ui.button("Forced reset").clicked() {
            self.bus.device(id, Op::ForcedReset);
        }
        if ui.button("Stop").clicked() {
            self.bus.device(id, Op::Stop);
        }
        if ui.button("Sync position").clicked() {
            self.bus.device(id, Op::SyncPosition);
        }
        if ui.button("Firmware version").clicked() {
            self.bus.device(id, Op::QueryVersion);
        }
        self.device_menu_footer(ui, id, state);
    }

    fn device_menu_footer(&mut self, ui: &mut egui::Ui, id: DeviceId, state: &BusState) {
        ui.separator();
        if ui.button("Show details").clicked() {
            self.selected = Some(id);
            self.tab = Tab::Monitor;
        }
        if ui.button("Datasheet").clicked() {
            self.tab = Tab::Datasheets;
        }
        if ui.button("Copy status").clicked() {
            ui.ctx().copy_text(device_status_text(id, state.dev(id), &self.settings));
        }
    }

    fn save_layout(&mut self) {
        self.layout_message = Some(match self.layout.save() {
            Ok(()) => {
                self.saved_layout = self.layout.clone();
                "Layout saved.".to_string()
            }
            Err(e) => format!("Saving layout failed: {e}"),
        });
    }

    fn confirm_window(&mut self, ctx: &egui::Context) {
        let Some((id, port)) = self.valve_confirm else { return };
        let mut open = true;
        let mut decided = false;
        egui::Window::new("Switch valve?")
            .collapsible(false)
            .resizable(false)
            .anchor(egui::Align2::CENTER_CENTER, [0.0, 0.0])
            .open(&mut open)
            .show(ctx, |ui| {
                ui.label(format!("Switch {} to port {port} — {}?", id.tag(), id.port_label(port)));
                ui.horizontal(|ui| {
                    if ui.button("Switch").clicked() {
                        self.bus.device(id, Op::ValveTo(port));
                        decided = true;
                    }
                    if ui.button("Cancel").clicked() {
                        decided = true;
                    }
                });
            });
        if !open || decided {
            self.valve_confirm = None;
        }
    }

    // ───────────────────────────── protocols ─────────────────────────────

    fn protocols_tab(&mut self, ui: &mut egui::Ui) {
        if self.form.auto_refresh && self.form.last_refresh.is_none_or(|t| t.elapsed() > Duration::from_secs(2)) {
            self.refresh_controller();
        }
        egui::ScrollArea::vertical().auto_shrink([false, false]).show(ui, |ui| {
            ui.horizontal(|ui| {
                ui.heading("controller_v2");
                let (c, t) = match self.ctrl.reachable {
                    Some(true) => (Color32::from_rgb(60, 180, 90), "reachable"),
                    Some(false) => (Color32::from_rgb(220, 70, 70), "unreachable"),
                    None => (Color32::GRAY, "not checked"),
                };
                status_dot(ui, c);
                ui.colored_label(c, format!("{} ({t})", self.settings.controller_url));
                if ui.button("Refresh").clicked() {
                    self.refresh_controller();
                }
                ui.checkbox(&mut self.form.auto_refresh, "auto (2 s)");
            });
            if let Some(s) = &self.ctrl.status {
                ui.label(format!(
                    "busy: {} · initialized: {}",
                    s.get("is_busy").and_then(|v| v.as_bool()).unwrap_or(false),
                    s.get("is_initialized").and_then(|v| v.as_bool()).unwrap_or(false)
                ));
            }

            ui.columns(2, |cols| {
                self.packet_builder(&mut cols[0]);
                self.controller_state(&mut cols[1]);
            });

            ui.separator();
            ui.heading("Responses");
            for (t, ok, line) in self.ctrl.log.iter().rev().take(30) {
                let color = if *ok { Color32::from_rgb(90, 170, 110) } else { Color32::from_rgb(220, 90, 90) };
                ui.colored_label(color, format!("[{:>4.0}s ago] {line}", t.elapsed().as_secs_f32()));
            }
        });
    }

    fn packet_builder(&mut self, ui: &mut egui::Ui) {
        ui.group(|ui| {
            ui.heading("Build packet");
            let f = &mut self.form;
            ui.horizontal(|ui| {
                ui.label("Slot");
                for s in 1..=6u16 {
                    ui.selectable_value(&mut f.slot, s, s.to_string());
                }
                ui.label(RichText::new(format!("slot_id {}", f.slot - 1)).weak());
            });
            ui.horizontal(|ui| {
                ui.label("Task ID");
                ui.add(egui::DragValue::new(&mut f.next_task_id).range(1..=u32::MAX));
                ui.label("Start in");
                ui.add(egui::DragValue::new(&mut f.start_delay_s).suffix(" s"));
                ui.label(RichText::new("0 = as soon as possible").weak());
            });
            let kind = api::COMMANDS[f.kind];
            egui::ComboBox::from_id_salt("cmd_kind")
                .selected_text(format!("{} — {}", kind.id, kind.name))
                .width(320.0)
                .show_ui(ui, |ui| {
                    for (i, k) in api::COMMANDS.iter().enumerate() {
                        let label = if k.production { format!("{} — {}", k.id, k.name) } else { format!("{} — {} (debug)", k.id, k.name) };
                        ui.selectable_value(&mut f.kind, i, label);
                    }
                });
            egui::Grid::new("cmd_fields").num_columns(2).spacing([12.0, 6.0]).show(ui, |ui| {
                if kind.uses_reagent {
                    ui.label("Reagent position");
                    ui.add(egui::DragValue::new(&mut f.reagent_pos).range(0..=64));
                    ui.end_row();
                }
                if kind.uses_timing {
                    ui.label(if kind.id == 2 { "Delay" } else { "Incubation time" });
                    ui.add(egui::DragValue::new(&mut f.time).suffix(" s"));
                    ui.end_row();
                }
                if kind.uses_timing && kind.id != 2 {
                    ui.label("Wash repetitions");
                    ui.add(egui::DragValue::new(&mut f.wash_reps).range(0..=20));
                    ui.end_row();
                    ui.label("Wash time");
                    ui.add(egui::DragValue::new(&mut f.wash_time).suffix(" s"));
                    ui.end_row();
                    ui.label("Toxic (danger waste)");
                    ui.checkbox(&mut f.is_toxic, "");
                    ui.end_row();
                }
                if kind.uses_temperature {
                    ui.label("Temperature");
                    ui.add(egui::DragValue::new(&mut f.temperature).range(4.0..=95.0).speed(0.5).suffix(" °C"));
                    ui.end_row();
                }
            });
            if ui.button("+ Add command").clicked() {
                f.commands.push(SingleCommand {
                    command_type: kind.id,
                    temperature: f.temperature,
                    time: f.time,
                    wash_reps: f.wash_reps,
                    reagent_pos: f.reagent_pos,
                    wash_time: f.wash_time,
                    is_toxic: f.is_toxic,
                });
            }

            let mut remove = None;
            for (i, c) in f.commands.iter().enumerate() {
                ui.horizontal(|ui| {
                    if ui.small_button("x").clicked() {
                        remove = Some(i);
                    }
                    let name = api::COMMANDS.iter().find(|k| k.id == c.command_type).map(|k| k.name).unwrap_or("?");
                    ui.label(format!(
                        "{}. [{}] {name} · reagent {} · {} s · wash {}×{} s · {} °C{}",
                        i + 1,
                        c.command_type,
                        c.reagent_pos,
                        c.time,
                        c.wash_reps,
                        c.wash_time,
                        c.temperature,
                        if c.is_toxic { " · toxic" } else { "" }
                    ));
                });
            }
            if let Some(i) = remove {
                f.commands.remove(i);
            }

            let packet = Packet {
                slot_id: f.slot - 1,
                commands: f.commands.clone(),
                task_id: f.next_task_id,
                requested_start_ts_ms: if f.start_delay_s == 0 { 0 } else { now_ms() + f.start_delay_s * 1000 },
            };
            let has_cmds = !packet.commands.is_empty();
            ui.horizontal(|ui| {
                if ui.add_enabled(has_cmds, egui::Button::new("Estimate time")).clicked() {
                    self.api(Request::EstimateTime(packet.clone()));
                }
                if ui.add_enabled(has_cmds, egui::Button::new(RichText::new("Send packet").strong())).clicked() {
                    self.api(Request::PostData(packet.clone()));
                    self.form.control_task_id = self.form.next_task_id;
                    self.form.next_task_id += 1;
                    self.form.commands.clear();
                }
                if ui.add_enabled(has_cmds, egui::Button::new("Clear")).clicked() {
                    self.form.commands.clear();
                }
            });
            egui::CollapsingHeader::new("JSON preview").show(ui, |ui| {
                let text = serde_json::to_string_pretty(&packet).unwrap_or_default();
                ui.label(RichText::new(text).monospace().small());
            });
        });

        ui.group(|ui| {
            ui.heading("Sequence control");
            ui.horizontal(|ui| {
                ui.label("Task ID");
                ui.add(egui::DragValue::new(&mut self.form.control_task_id).range(1..=u32::MAX));
                for (id, name) in [(api::PAUSE, "Pause"), (api::RESUME, "Resume"), (api::ABORT, "Abort")] {
                    if ui.button(name).clicked() {
                        self.send_control(id, self.form.control_task_id);
                    }
                }
            });
            ui.horizontal(|ui| {
                for (id, name) in [(api::PAUSE_ALL, "Pause all"), (api::RESUME_ALL, "Resume all"), (api::ABORT_ALL, "Abort all")] {
                    if ui.button(name).clicked() {
                        self.send_control(id, 0);
                    }
                }
            });
        });

        ui.group(|ui| {
            ui.heading("Initialize ports");
            let f = &mut self.form;
            egui::Grid::new("init").num_columns(2).show(ui, |ui| {
                ui.label("RS485 port");
                ui.text_edit_singleline(&mut f.init_port);
                ui.end_row();
                ui.label("Sensor (CAN) port");
                ui.text_edit_singleline(&mut f.init_sensor_port);
                ui.end_row();
                ui.label("Baud rate");
                ui.add(egui::DragValue::new(&mut f.init_baud));
                ui.end_row();
            });
            if ui.button("POST /initialize").clicked() {
                let req = Request::Initialize {
                    port_name: f.init_port.clone(),
                    sensor_port_name: f.init_sensor_port.clone(),
                    baud_rate: f.init_baud,
                };
                self.api(req);
            }
        });
    }

    fn send_control(&self, command_type: u16, task_id: u32) {
        self.api(Request::PostData(Packet {
            slot_id: self.form.slot - 1,
            commands: vec![SingleCommand::control(command_type)],
            task_id,
            requested_start_ts_ms: 0,
        }));
    }

    fn controller_state(&mut self, ui: &mut egui::Ui) {
        ui.group(|ui| {
            ui.heading("Slots");
            match self.ctrl.slots.as_ref().and_then(|v| v.as_array()) {
                Some(slots) => {
                    egui::Grid::new("slots").num_columns(5).striped(true).show(ui, |ui| {
                        for h in ["Slot", "State", "SV02 port", "Sensor", "SV03 port"] {
                            ui.label(RichText::new(h).strong());
                        }
                        ui.end_row();
                        for (i, s) in slots.iter().enumerate() {
                            ui.label(format!("{}", i + 1));
                            ui.label(s.get("description").and_then(|v| v.as_str()).unwrap_or("?"));
                            ui.label(json_num(s, "selector_pos"));
                            ui.label(json_num(s, "sensor_id"));
                            ui.label(json_num(s, "drain_selector_id"));
                            ui.end_row();
                        }
                    });
                    ui.label(RichText::new("Positions are what controller_v2 has configured; the new rig expects SV02 15…10 and SV03 6…1 for slots 1…6.").small().weak());
                }
                None => {
                    ui.label(RichText::new("No data — press Refresh.").weak());
                }
            }
        });
        ui.group(|ui| {
            ui.heading("Protocols");
            match self.ctrl.protocols.as_ref().and_then(|v| v.as_array()) {
                Some(list) if !list.is_empty() => {
                    for p in list {
                        let done = p.get("completed_step_count").and_then(|v| v.as_u64()).unwrap_or(0);
                        let total = p.get("step_count").and_then(|v| v.as_u64()).unwrap_or(1).max(1);
                        let flag = |k: &str| p.get(k).and_then(|v| v.as_bool()).unwrap_or(false);
                        let state = if flag("is_aborted") {
                            "aborted"
                        } else if flag("has_ended") {
                            "ended"
                        } else if flag("is_paused") {
                            "paused"
                        } else if flag("has_begun") {
                            "running"
                        } else {
                            "queued"
                        };
                        let remaining = duration_secs(p.get("time_remaining_estimate"));
                        ui.label(format!(
                            "Task {} · {} · {state} · {} remaining",
                            json_num(p, "task_id"),
                            p.get("slot_description").and_then(|v| v.as_str()).unwrap_or(""),
                            fmt_secs(remaining)
                        ));
                        ui.add(egui::ProgressBar::new(done as f32 / total as f32).text(format!("step {done}/{total}")));
                        ui.horizontal(|ui| {
                            let id = p.get("task_id").and_then(|v| v.as_u64()).unwrap_or(0) as u32;
                            for (cmd, name) in [(api::PAUSE, "Pause"), (api::RESUME, "Resume"), (api::ABORT, "Abort")] {
                                if ui.small_button(name).clicked() {
                                    self.send_control(cmd, id);
                                }
                            }
                        });
                    }
                }
                Some(_) => {
                    ui.label(RichText::new("No active or queued protocols.").weak());
                }
                None => {
                    ui.label(RichText::new("No data — press Refresh.").weak());
                }
            }
        });
    }

    // ───────────────────────────── datasheets ─────────────────────────────

    fn datasheets_tab(&mut self, ui: &mut egui::Ui) {
        egui::ScrollArea::vertical().auto_shrink([false, false]).show(ui, |ui| {
            let groups: [(&str, &[DeviceId]); 4] = [
                ("PP01", &[DeviceId::Pp01]),
                ("PP02", &[DeviceId::Pp02]),
                ("SV01 · SV03", &[DeviceId::Sv01, DeviceId::Sv03]),
                ("SV02", &[DeviceId::Sv02]),
            ];
            for (title, ids) in groups {
                let spec = ids[0].spec();
                egui::CollapsingHeader::new(RichText::new(format!("{title} — {}", spec.part_number)).strong())
                    .default_open(true)
                    .show(ui, |ui| {
                        ui.label(spec.model);
                        egui::Grid::new(("spec", title)).num_columns(2).striped(true).spacing([24.0, 4.0]).show(ui, |ui| {
                            for (k, v) in spec.rows {
                                ui.label(RichText::new(*k).weak());
                                ui.label(*v);
                                ui.end_row();
                            }
                        });
                        for n in spec.notes {
                            ui.label(RichText::new(format!("Note: {n}")).color(Color32::from_rgb(210, 150, 40)));
                        }
                        if let Kind::Valve { ports } = ids[0].kind() {
                            egui::CollapsingHeader::new("Port map (from drawio)").id_salt(("ports", title)).show(ui, |ui| {
                                egui::Grid::new(("pm", title)).num_columns(1 + ids.len()).striped(true).show(ui, |ui| {
                                    ui.label(RichText::new("Port").strong());
                                    for id in ids {
                                        ui.label(RichText::new(id.tag()).strong());
                                    }
                                    ui.end_row();
                                    for p in 1..=ports {
                                        ui.label(p.to_string());
                                        for id in ids {
                                            ui.label(id.port_label(p));
                                        }
                                        ui.end_row();
                                    }
                                });
                            });
                        }
                        ui.hyperlink_to(format!("Source: {}", spec.source), spec.source_url);
                    });
                ui.add_space(6.0);
            }

            egui::CollapsingHeader::new(RichText::new("RS485 protocol (Runze self-defined, 8 bytes)").strong())
                .default_open(true)
                .show(ui, |ui| {
                    ui.label(RichText::new("TX: CC ADDR FUNC P_lo P_hi DD SUM_lo SUM_hi   RX: CC ADDR STATUS V_lo V_hi DD SUM_lo SUM_hi").monospace());
                    ui.label("SUM = 16-bit sum of bytes 0–5. 8N1, default 9600 bps. Addresses 0x00–0x7F, multicast 0x80–0xFE, broadcast 0xFF.");
                    egui::Grid::new("cmds").num_columns(3).striped(true).show(ui, |ui| {
                        for (code, name, dev) in [
                            ("0x3E", "Query current channel", "valve"),
                            ("0x3F", "Query firmware version", "all"),
                            ("0x4A", "Query motor status", "all"),
                            ("0x66", "Query piston position (steps)", "pump"),
                            ("0x67", "Synchronize piston position", "pump"),
                            ("0x44", "Switch to port (shortest path)", "valve"),
                            ("0x45", "Reset / home", "all"),
                            ("0x4C", "Valve reset", "valve"),
                            ("0x4F", "Forced reset", "pump"),
                            ("0x4D", "Aspirate: relative steps CCW", "pump"),
                            ("0x42", "Dispense: relative steps CW", "pump"),
                            ("0x4E", "Absolute position", "pump"),
                            ("0x4B", "Set speed 1–500 rpm", "pump"),
                            ("0x49", "Forced stop", "all"),
                            ("0x60 / 0x61", "IO3 high / low (solenoid)", "pump with MC12M / MC10+MOS"),
                        ] {
                            ui.label(RichText::new(code).monospace());
                            ui.label(name);
                            ui.label(RichText::new(dev).weak());
                            ui.end_row();
                        }
                    });
                    ui.add_space(4.0);
                    egui::Grid::new("status").num_columns(2).striped(true).show(ui, |ui| {
                        for code in [0x00u8, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0xFE, 0xFF] {
                            ui.label(RichText::new(format!("0x{code:02X}")).monospace());
                            ui.label(protocol::status_text(code));
                            ui.end_row();
                        }
                    });
                    ui.hyperlink_to("RP-01 manual §2.2.3", "https://www.runzefluid.com/uploads/file/rp-01-piston-pump-v1-1.pdf");
                    ui.hyperlink_to("SY-01B manual §2.3", "https://www.runzefluid.com/uploads/file/sy-01b-user's-manual-v1-0.pdf");
                });
        });
    }

    // ───────────────────────────── log ─────────────────────────────

    fn log_tab(&mut self, ui: &mut egui::Ui, state: &BusState) {
        ui.horizontal(|ui| {
            ui.label("Filter");
            ui.text_edit_singleline(&mut self.log_filter);
            if ui.checkbox(&mut self.show_trace, "Raw frames (TX/RX hex)").changed() {
                self.bus.send(BusCmd::SetTrace(self.show_trace));
            }
            if ui.button("Clear").clicked() {
                self.bus.state.lock().unwrap().log.clear();
            }
            ui.label(RichText::new(format!("frames {} · failed {}", state.frames_tx, state.frames_bad)).weak());
        });
        ui.separator();
        let filter = self.log_filter.to_lowercase();
        egui::ScrollArea::vertical().auto_shrink([false, false]).stick_to_bottom(true).show(ui, |ui| {
            for line in state.log.iter().filter(|l| filter.is_empty() || l.text.to_lowercase().contains(&filter)) {
                let color = match line.level {
                    Level::Info => ui.visuals().text_color(),
                    Level::Warn => Color32::from_rgb(220, 160, 40),
                    Level::Error => Color32::from_rgb(225, 80, 80),
                    Level::Trace => Color32::from_rgb(120, 140, 170),
                };
                ui.label(RichText::new(format!("{:>8.2}  {}", line.t, line.text)).monospace().color(color));
            }
        });
    }

    // ───────────────────────────── settings ─────────────────────────────

    fn settings_tab(&mut self, ui: &mut egui::Ui) {
        // The server sends its port list on connect, which may land after the
        // handshake; pick it up while this tab is open rather than making the
        // user press Refresh for a list that has already arrived.
        if self.bus.is_remote() {
            let ports = self.bus.serial_ports();
            if !ports.is_empty() && ports != self.available_ports {
                self.available_ports = ports;
            }
        }
        egui::ScrollArea::vertical().auto_shrink([false, false]).show(ui, |ui| {
            let s = &mut self.draft;
            let remote = s.backend == Backend::Remote;
            ui.heading("Connection");
            ui.horizontal(|ui| {
                ui.radio_value(&mut s.backend, Backend::Simulator, "Simulator")
                    .on_hover_text("No hardware: simulated devices with datasheet timing, on this machine");
                ui.radio_value(&mut s.backend, Backend::Serial, "RS485 serial")
                    .on_hover_text("Drive an adapter plugged into this machine");
                ui.radio_value(&mut s.backend, Backend::Remote, "Remote server")
                    .on_hover_text("Let tstand_server drive the rig; this window only watches and commands");
            });

            if remote {
                ui.horizontal(|ui| {
                    ui.label("Server");
                    ui.add(
                        egui::TextEdit::singleline(&mut s.server_host)
                            .hint_text("tonespi.local:7373")
                            .desired_width(200.0),
                    );
                    ui.label("Token");
                    ui.add(
                        egui::TextEdit::singleline(&mut s.server_token)
                            .password(true)
                            .hint_text("--token on the server")
                            .desired_width(160.0),
                    );
                });
                ui.label(
                    RichText::new(
                        "The adapter stays on the server. Port, addresses and calibration below are the server's, \
                         read from it when this window connects; applying writes them back to it.",
                    )
                    .weak(),
                );
                ui.horizontal(|ui| {
                    ui.label("Server drives");
                    ui.radio_value(&mut s.server_backend, Backend::Simulator, "Simulator");
                    ui.radio_value(&mut s.server_backend, Backend::Serial, "RS485 serial");
                });
            }

            // The serial fields describe whichever machine holds the adapter.
            let serial_side = if remote { s.server_backend } else { s.backend };
            ui.add_enabled_ui(serial_side == Backend::Serial, |ui| {
                ui.horizontal(|ui| {
                    ui.label("Port");
                    egui::ComboBox::from_id_salt("port_combo")
                        .selected_text(if s.port.is_empty() { "Select a port" } else { s.port.as_str() })
                        .show_ui(ui, |ui| {
                            for port in &self.available_ports {
                                ui.selectable_value(&mut s.port, port.clone(), port);
                            }
                        });
                    if ui.button("Refresh").clicked() {
                        self.available_ports = self.bus.serial_ports();
                    }
                    ui.text_edit_singleline(&mut s.port);
                    if remote {
                        ui.label(RichText::new("on the server").weak());
                    }
                });
                ui.horizontal(|ui| {
                    ui.label("Baud rate");
                    egui::ComboBox::from_id_salt("baud")
                        .selected_text(s.baud_rate.to_string())
                        .show_ui(ui, |ui| {
                            for b in [9600u32, 19200, 38400, 57600, 115200] {
                                ui.selectable_value(&mut s.baud_rate, b, b.to_string());
                            }
                        });
                });
            });
            ui.horizontal(|ui| {
                ui.label("Poll interval");
                ui.add(egui::DragValue::new(&mut s.poll_interval_ms).range(50..=5000).suffix(" ms"));
                ui.label("Reply timeout");
                ui.add(egui::DragValue::new(&mut s.reply_timeout_ms).range(20..=2000).suffix(" ms"));
            });

            ui.add_space(8.0);
            ui.heading("Slave addresses");
            egui::Grid::new("addr_grid").num_columns(3).spacing([16.0, 6.0]).show(ui, |ui| {
                for id in DeviceId::ALL {
                    ui.label(RichText::new(id.tag()).strong());
                    ui.add(egui::DragValue::new(s.addr_mut(id)).range(0..=127));
                    ui.label(RichText::new(format!("{} · controller_v2 default {}", id.role(), id.default_addr())).weak());
                    ui.end_row();
                }
            });

            ui.add_space(8.0);
            ui.heading("Pump calibration");
            egui::Grid::new("cal").num_columns(3).spacing([16.0, 6.0]).show(ui, |ui| {
                ui.label("PP01 max steps");
                ui.add(egui::DragValue::new(&mut s.pp01_max_steps).range(1..=3820));
                ui.label(RichText::new("datasheet limit 3820 (0x0EEC)").weak());
                ui.end_row();
                ui.label("PP01 µL/step");
                ui.add(egui::DragValue::new(&mut s.pp01_ul_per_step).range(0.01..=10.0).speed(0.001));
                ui.label(RichText::new("controller_v2 2.083 · RP-01 6 mL 1.5707").weak());
                ui.end_row();
                ui.label("PP02 max steps");
                ui.add(egui::DragValue::new(&mut s.pp02_max_steps).range(1..=24000));
                ui.label(RichText::new("controller_v2 12000").weak());
                ui.end_row();
                ui.label("PP02 µL/step");
                ui.add(egui::DragValue::new(&mut s.pp02_ul_per_step).range(0.01..=10.0).speed(0.001));
                ui.label(RichText::new("controller_v2 0.416").weak());
                ui.end_row();
                ui.label("Slot volume");
                ui.add(egui::DragValue::new(&mut s.slot_volume_ul).range(10.0..=5000.0).suffix(" µL"));
                ui.label(RichText::new("full mark for slot gauges · controller_v2 300").weak());
                ui.end_row();
            });

            ui.add_space(8.0);
            ui.heading("Display");
            egui::Grid::new("display").num_columns(3).spacing([16.0, 6.0]).show(ui, |ui| {
                ui.label("UI scale");
                ui.add(egui::Slider::new(&mut s.ui_scale, 0.75..=2.0).step_by(0.05));
                ui.label(RichText::new("zooms the whole window").weak());
                ui.end_row();
                ui.label("Schematic label size");
                ui.add(egui::Slider::new(&mut s.label_scale, 0.8..=1.6).step_by(0.05));
                ui.label(RichText::new("pump cards, valve port and state badges").weak());
                ui.end_row();
                let (ppp, zoom) = (ui.ctx().pixels_per_point(), ui.ctx().zoom_factor());
                ui.label("Screen scale");
                ui.label(format!("{:.2}× display · {:.2}× with zoom", ppp / zoom, ppp));
                ui.label(RichText::new("from the OS display settings; Ctrl/Cmd +/- also zooms").weak());
                ui.end_row();
            });

            ui.add_space(8.0);
            ui.heading("Temperature (CAN)");
            ui.checkbox(&mut s.temp_enabled, "Drive the Peltier slot-temperature board");
            ui.add_enabled_ui(s.temp_enabled, |ui| {
                ui.horizontal(|ui| {
                    ui.label("CAN adapter");
                    ui.add(
                        egui::TextEdit::singleline(&mut s.temp_port)
                            .hint_text("/dev/ttyACM0 — blank auto-detects")
                            .desired_width(240.0),
                    );
                    if remote {
                        ui.label(RichText::new("on the server").weak());
                    }
                });
            });
            ui.label(
                RichText::new(
                    "The board runs its own PID; this sets the target and starts or stops it. \
                     The adapter is shared with the optical sensors, so only one program may hold it — \
                     stop controller_v2 before enabling this.",
                )
                .small()
                .weak(),
            );

            ui.add_space(8.0);
            ui.heading("Rig");
            ui.checkbox(&mut s.rig.restrict, "Restrict to plumbed ports")
                .on_hover_text("Refuse to switch a valve to a port this rig does not have. Turn off to reach a capped line.");
            ui.horizontal(|ui| {
                ui.label("Fitted slots");
                let mut slots = String::new();
                for (i, n) in s.rig.slots.iter().enumerate() {
                    if i > 0 {
                        slots.push_str(", ");
                    }
                    slots.push_str(&n.to_string());
                }
                if ui
                    .add(egui::TextEdit::singleline(&mut slots).desired_width(120.0).hint_text("e.g. 6"))
                    .changed()
                {
                    s.rig.slots = slots
                        .split(|c: char| !c.is_ascii_digit())
                        .filter_map(|p| p.parse::<u16>().ok())
                        .filter(|n| (1..=6).contains(n))
                        .collect();
                }

                ui.label(
                    RichText::new("slot numbers follow the diagram: SV02 port 16−n, SV03 port 7−n")
                        .small()
                        .weak(),
                );
            });
            ui.label(RichText::new("Optical sensors — where each board channel sits on the diagram.").small().weak());
            let mut drop_sensor: Option<usize> = None;
            egui::Grid::new("rig_sensors").num_columns(4).spacing([10.0, 4.0]).show(ui, |ui| {
                for (i, sensor) in s.rig.sensors.iter_mut().enumerate() {
                    egui::ComboBox::from_id_salt(("rig_sensor_at", i))
                        .selected_text(match sensor.at {
                            SensorAt::Inline1 => "S1 · pump → coil",
                            SensorAt::Inline2 => "S2 · coil → SV02",
                            SensorAt::Slot => "slot feed",
                        })
                        .width(150.0)
                        .show_ui(ui, |ui| {
                            ui.selectable_value(&mut sensor.at, SensorAt::Inline1, "S1 · pump → coil");
                            ui.selectable_value(&mut sensor.at, SensorAt::Inline2, "S2 · coil → SV02");
                            ui.selectable_value(&mut sensor.at, SensorAt::Slot, "slot feed");
                        });
                    ui.add(egui::TextEdit::singleline(&mut sensor.name).desired_width(60.0).hint_text("A2"));
                    ui.add(egui::DragValue::new(&mut sensor.channel).range(0..=5).prefix("channel "))
                        .on_hover_text("Index into the board's six detectors, 0–5");
                    if ui.button("Remove").clicked() {
                        drop_sensor = Some(i);
                    }
                    ui.end_row();
                }
            });
            if let Some(i) = drop_sensor {
                s.rig.sensors.remove(i);
            }
            ui.horizontal(|ui| {
                if ui.button("Add sensor").clicked() {
                    s.rig.sensors.push(Sensor { at: SensorAt::Slot, name: String::new(), channel: 0 });
                }
                ui.label("Board CAN id");
                ui.add(
                    egui::DragValue::new(&mut s.sensor_can_id)
                        .range(0..=0x7FF)
                        .hexadecimal(3, false, true)
                        .prefix("0x"),
                )
                .on_hover_text("The firmware ships as 0x700; some builds assume 0x7FF.");
            });

            ui.add_space(6.0);
            ui.label(RichText::new("Plumbed ports — anything not listed is treated as capped.").small().weak());
            let mut remove: Option<usize> = None;
            egui::Grid::new("rig_ports").num_columns(5).spacing([10.0, 4.0]).show(ui, |ui| {
                for (i, use_) in s.rig.ports.iter_mut().enumerate() {
                    egui::ComboBox::from_id_salt(("rig_valve", i))
                        .selected_text(use_.valve.tag())
                        .width(64.0)
                        .show_ui(ui, |ui| {
                            for v in [DeviceId::Sv01, DeviceId::Sv02, DeviceId::Sv03] {
                                ui.selectable_value(&mut use_.valve, v, v.tag());
                            }
                        });
                    let Kind::Valve { ports } = use_.valve.kind() else { return };
                    ui.add(egui::DragValue::new(&mut use_.port).range(1..=ports).prefix("port "));
                    ui.add(egui::TextEdit::singleline(&mut use_.label).desired_width(150.0).hint_text("label"));
                    egui::ComboBox::from_id_salt(("rig_role", i))
                        .selected_text(use_.role.label())
                        .width(110.0)
                        .show_ui(ui, |ui| {
                            for r in [
                                Role::Wash,
                                Role::Reagent,
                                Role::SlotFill,
                                Role::SlotDrain,
                                Role::Waste,
                                Role::DangerWaste,
                                Role::Air,
                                Role::Other,
                            ] {
                                ui.selectable_value(&mut use_.role, r, r.label());
                            }
                        });
                    if ui.button("Remove").clicked() {
                        remove = Some(i);
                    }
                    ui.end_row();
                }
            });
            if let Some(i) = remove {
                s.rig.ports.remove(i);
            }
            ui.horizontal(|ui| {
                if ui.button("Add port").clicked() {
                    s.rig.ports.push(PortUse { valve: DeviceId::Sv02, port: 1, label: String::new(), role: Role::Other });
                }
                if ui.button("This rig (defaults)").on_hover_text("One slot, wash, reagent, waste and air").clicked() {
                    s.rig = Rig::default();
                }
                if ui.button("Whole diagram").on_hover_text("Six slots, every port, no guard").clicked() {
                    s.rig = Rig::unrestricted();
                }
            });

            ui.add_space(8.0);
            ui.heading("controller_v2 HTTP");
            ui.horizontal(|ui| {
                ui.label("Address");
                ui.text_edit_singleline(&mut s.controller_url);
            });
            if remote {
                ui.label(
                    RichText::new(
                        "Resolved on the server, which forwards these requests for this window — so 127.0.0.1 \
                         here means the server's own loopback, where controller_v2 listens.",
                    )
                    .weak(),
                );
            }
            ui.checkbox(&mut s.poll_controller, "Read slot states (Idle / Missing / running step) from /slot-status every 2 s");

            ui.add_space(10.0);
            ui.horizontal(|ui| {
                if ui.button(RichText::new("Apply & save").strong()).clicked() {
                    if self.draft.effective_backend() != self.settings.effective_backend() {
                        self.tracker.flush();
                        self.tracker = Tracker::for_backend(self.draft.effective_backend());
                    }
                    self.settings = self.draft.clone();
                    self.bus.send(BusCmd::Apply(Box::new(self.settings.clone())));
                    self.settings_message = Some(match self.settings.save() {
                        Ok(()) => "Applied and saved.".to_string(),
                        Err(e) => format!("Applied, but saving failed: {e}"),
                    });
                }
                if ui.button("Revert").clicked() {
                    self.draft = self.settings.clone();
                }
                if ui.button("Defaults").clicked() {
                    self.draft = Settings::default();
                }
            });
            if let Some(msg) = &self.settings_message {
                ui.label(msg);
            }
        });
    }
}

impl App {
    /// A server tells each GUI what it is actually driving when they connect.
    /// Take those fields over once per connection: the rig's port, addresses
    /// and calibration belong to the machine holding the adapter, and another
    /// operator may have set them since this window last saved anything.
    fn adopt_server_settings(&mut self) {
        let Some((generation, server)) = crate::remote::client::server_settings_since(self.adopted_server_settings)
        else {
            return;
        };
        self.adopted_server_settings = generation;
        self.settings.adopt_server_fields(&server);
        // Leave anything the user is part-way through editing alone.
        if self.draft_matches_saved() {
            self.draft.adopt_server_fields(&server);
        }
        self.available_ports = self.bus.serial_ports();
        if self.tracker_backend != self.settings.effective_backend() {
            self.tracker.flush();
            self.tracker_backend = self.settings.effective_backend();
            self.tracker = Tracker::for_backend(self.tracker_backend);
        }
    }

    /// True while the Settings tab shows exactly what is in effect, so there
    /// are no half-typed edits for the server's values to overwrite.
    fn draft_matches_saved(&self) -> bool {
        serde_json::to_string(&self.draft).ok() == serde_json::to_string(&self.settings).ok()
    }
}

impl eframe::App for App {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        self.drain_api();
        self.adopt_server_settings();
        let state = self.bus.snapshot();
        if state.backend != Backend::Simulator {
            self.demo.clear();
            self.demo_wait = None;
        }
        self.step_demo(&state);
        self.step_routine(&state);
        self.tracker.update(&state, &self.settings);
        self.track_liquid(&state);
        if self.settings.poll_controller && self.last_slot_poll.is_none_or(|t| t.elapsed() > Duration::from_secs(2)) {
            self.api(Request::GetSlotStatus);
            self.last_slot_poll = Some(Instant::now());
        }
        let fluid = self.tracker.view(&state, &self.settings, &self.slot_descriptions());
        let ctx = ui.ctx().clone();
        // Apply the UI scale setting when it changes, and keep it in sync with
        // keyboard zoom (Ctrl/Cmd +/-) instead of fighting it every frame.
        let wanted = self.settings.ui_scale.clamp(0.75, 2.0);
        if (wanted - self.applied_zoom).abs() > 1e-3 {
            ctx.set_zoom_factor(wanted);
            self.applied_zoom = wanted;
        } else if (ctx.zoom_factor() - self.applied_zoom).abs() > 1e-3 {
            self.applied_zoom = ctx.zoom_factor();
            self.settings.ui_scale = self.applied_zoom;
            self.draft.ui_scale = self.applied_zoom;
        }

        egui::Panel::top("top").show(ui, |ui| self.top_bar(ui, &state));

        match self.tab {
            Tab::Monitor => {
                let side_width = (ui.available_width() * 0.28).clamp(300.0, 420.0);
                egui::Panel::right("side").resizable(true).default_size(side_width).min_size(260.0).show(ui, |ui| {
                    self.monitor_side(ui, &state, &fluid);
                });
                egui::CentralPanel::default().show(ui, |ui| self.monitor_central(ui, &state, &fluid));
                self.confirm_window(&ctx);
                ctx.request_repaint_after(Duration::from_millis(33));
            }
            Tab::Protocols => {
                egui::CentralPanel::default().show(ui, |ui| self.protocols_tab(ui));
                if self.form.auto_refresh {
                    ctx.request_repaint_after(Duration::from_millis(500));
                }
            }
            Tab::Datasheets => {
                egui::CentralPanel::default().show(ui, |ui| self.datasheets_tab(ui));
            }
            Tab::Log => {
                egui::CentralPanel::default().show(ui, |ui| self.log_tab(ui, &state));
                ctx.request_repaint_after(Duration::from_millis(250));
            }
            Tab::Settings => {
                egui::CentralPanel::default().show(ui, |ui| self.settings_tab(ui));
            }
        }
    }
}

fn part_title(key: &str) -> String {
    match key {
        "TC01" | "TC02" => format!("{key} · 9-channel pass-through"),
        "COIL" => "Spiral tube (holding coil)".to_string(),
        "AIR" => "Air filter / port · SV02 port 16".to_string(),
        k if k.starts_with('S') => format!("{k} · optical sensor (CAN, read by controller_v2)"),
        k => k.to_string(),
    }
}

/// One-line status for the clipboard.
fn device_status_text(id: DeviceId, d: &crate::bus::DeviceLive, settings: &Settings) -> String {
    let mut text = format!("{} {}", id.tag(), d.state().label());
    match (id.kind(), d.value) {
        (Kind::Pump { .. }, Some(v)) => text += &format!(" · {v} st · {:.1} µL", v as f32 * settings.ul_per_step(id)),
        (Kind::Valve { .. }, Some(0)) => text += " · reset",
        (Kind::Valve { .. }, Some(p)) => text += &format!(" · port {p} ({})", id.port_label(p)),
        (_, None) => {}
    }
    if let Some(st) = d.status {
        text += &format!(" · {} (0x{st:02X})", protocol::status_text(st));
    }
    text
}

fn status_dot(ui: &mut egui::Ui, color: Color32) {
    let (rect, _) = ui.allocate_exact_size(egui::vec2(12.0, 12.0), egui::Sense::hover());
    ui.painter().circle_filled(rect.center(), 5.0, color);
}

/// Simulator walkthrough of one slot infill, following docs/operational-notes.md
/// with the new rig's port numbers.
fn demo_steps() -> VecDeque<(DeviceId, Op)> {
    use crate::devices::{sv02_port_for_slot, sv03_port_for_slot};
    VecDeque::from([
        (DeviceId::Pp01, Op::SetSpeed(250)),
        (DeviceId::Pp01, Op::SolenoidInput(true)),
        (DeviceId::Sv01, Op::ValveTo(1)),
        (DeviceId::Pp01, Op::PumpTo(1900)),
        (DeviceId::Pp01, Op::SolenoidInput(false)),
        (DeviceId::Sv02, Op::ValveTo(4)),
        (DeviceId::Pp01, Op::Aspirate(150)),
        (DeviceId::Sv02, Op::ValveTo(sv02_port_for_slot(1))),
        (DeviceId::Pp01, Op::Dispense(200)),
        (DeviceId::Sv03, Op::ValveTo(sv03_port_for_slot(1))),
        (DeviceId::Pp02, Op::Aspirate(900)),
        (DeviceId::Sv03, Op::ValveTo(7)),
        (DeviceId::Pp02, Op::Home),
        (DeviceId::Sv02, Op::ValveTo(9)),
        (DeviceId::Pp01, Op::Dispense(400)),
        (DeviceId::Sv02, Op::Home),
    ])
}

fn state_color(st: DevState) -> Color32 {
    match st {
        DevState::Offline => Color32::from_rgb(140, 140, 150),
        DevState::Fault => Color32::from_rgb(225, 70, 70),
        DevState::Moving => Color32::from_rgb(230, 170, 40),
        DevState::Ready => Color32::from_rgb(60, 180, 90),
    }
}

fn text_on(c: Color32) -> Color32 {
    if 0.299 * c.r() as f32 + 0.587 * c.g() as f32 + 0.114 * c.b() as f32 > 150.0 { Color32::BLACK } else { Color32::WHITE }
}

fn state_badge(ui: &mut egui::Ui, st: DevState, size: f32) {
    badge(ui, st.label(), state_color(st), size);
}

/// Filled label sized to its text (a Frame would stretch to the grid row).
fn badge(ui: &mut egui::Ui, label: &str, color: Color32, size: f32) {
    let galley = ui.painter().layout_no_wrap(label.to_string(), egui::FontId::proportional(size), text_on(color));
    let (rect, _) = ui.allocate_exact_size(galley.size() + egui::vec2(size * 0.8, size * 0.3), egui::Sense::hover());
    ui.painter().rect_filled(rect, 4.0, color);
    ui.painter().galley(rect.center() - galley.size() * 0.5, galley, text_on(color));
}

/// One row of a two-column parameter grid.
fn param(ui: &mut egui::Ui, name: &str, value: String) {
    ui.label(RichText::new(name).size(14.0).weak());
    ui.label(RichText::new(value).size(16.0).strong());
    ui.end_row();
}

/// Position trend over the last two minutes.
fn trend(ui: &mut egui::Ui, history: &std::collections::VecDeque<(f64, f32)>, max: f32, now: f64) {
    let size = egui::vec2(ui.available_width(), 70.0);
    let (rect, _) = ui.allocate_exact_size(size, egui::Sense::hover());
    let painter = ui.painter_at(rect);
    let visuals = ui.visuals();
    painter.rect_filled(rect, 4.0, visuals.extreme_bg_color);
    let span = 120.0;
    let pts: Vec<egui::Pos2> = history
        .iter()
        .map(|&(t, v)| {
            let x = rect.right() - ((now - t) as f32 / span) * rect.width();
            let y = rect.bottom() - (v / max.max(1.0)).clamp(0.0, 1.0) * (rect.height() - 6.0) - 3.0;
            egui::pos2(x.max(rect.left()), y)
        })
        .collect();
    if pts.len() >= 2 {
        painter.line(pts, egui::Stroke::new(1.8, Color32::from_rgb(64, 150, 240)));
    }
    painter.text(rect.left_top() + egui::vec2(6.0, 4.0), egui::Align2::LEFT_TOP, "position, last 2 min", egui::FontId::proportional(10.0), visuals.weak_text_color());
}

/// Measured slot temperature against the target it was chasing.
///
/// Takes one board's trend rather than the whole state, so a second slot is
/// another call rather than a redesign.
fn temp_trend(ui: &mut egui::Ui, history: &VecDeque<TempSample>, now: f64, window: f64) {
    let (rect, response) = ui.allocate_exact_size(egui::vec2(ui.available_width(), 96.0), egui::Sense::hover());
    let painter = ui.painter_at(rect);
    let visuals = ui.visuals();
    painter.rect_filled(rect, 4.0, visuals.extreme_bg_color);
    let small = egui::FontId::proportional(10.0);
    // A fixed span ending now. Scaling the axis to whatever has been collected
    // makes a settling curve look the same shape after ten seconds as after
    // ten minutes; a stable window lets the two be told apart, at the cost of
    // a mostly-empty box for the first minute of a session.
    let window = window.max(1.0);
    let t0 = now - window;
    let shown: Vec<&TempSample> = history.iter().filter(|s| s.t >= t0).collect();
    let Some((lo, hi)) = trend_range(shown.iter().copied()) else {
        painter.text(rect.center(), egui::Align2::CENTER_CENTER, "no samples yet", small, visuals.weak_text_color());
        return;
    };
    // The corner labels get a band of their own at the top and bottom, and the
    // traces stay out of it. Without that the target line — which sits near the
    // top whenever the rig is heating — runs straight through the legend.
    const LABELS: f32 = 13.0;
    let plot = egui::Rect::from_min_max(
        egui::pos2(rect.left() + 5.0, rect.top() + LABELS),
        egui::pos2(rect.right() - 5.0, rect.bottom() - LABELS),
    );
    // `trend_range` never returns a zero-height band, so this cannot divide by
    // zero even with the board holding dead steady.
    let span = window;
    let at = |t: f64, v: f32| {
        let x = plot.right() - ((now - t) / span) as f32 * plot.width();
        egui::pos2(x.max(plot.left()), plot.bottom() - (v - lo) / (hi - lo) * plot.height())
    };

    // The target first, so the measurement reads on top of it.
    let target = Color32::from_rgb(60, 180, 90);
    let mut runs: Vec<Vec<egui::Pos2>> = vec![Vec::new()];
    for s in &shown {
        match s.target {
            Some(v) => runs.last_mut().expect("never empty").push(at(s.t, v)),
            // No setpoint known is a hole in the trace, not a line across it.
            None if !runs.last().expect("never empty").is_empty() => runs.push(Vec::new()),
            None => {}
        }
    }
    for run in &runs {
        if run.len() >= 2 {
            painter.extend(egui::Shape::dashed_line(run, egui::Stroke::new(1.3, target), 5.0, 4.0));
        } else if let [p] = run[..] {
            painter.circle_filled(p, 1.6, target);
        }
    }

    let measured = Color32::from_rgb(64, 150, 240);
    let pts: Vec<egui::Pos2> = shown.iter().map(|s| at(s.t, s.measured)).collect();
    if pts.len() >= 2 {
        painter.line(pts.clone(), egui::Stroke::new(1.8, measured));
    } else if let [p] = pts[..] {
        painter.circle_filled(p, 2.0, measured);
    }

    // One caption in the corner, like the pump's own trend box above. The two
    // words are drawn in their trace's colour, which is the legend — a
    // separate key would be more furniture for the same information.
    let weak = visuals.weak_text_color();
    let mut x = rect.left() + 6.0;
    for (word, colour) in [("measured", measured), (" vs ", weak), ("target", target)] {
        let at = egui::pos2(x, rect.top() + 3.0);
        x = painter.text(at, egui::Align2::LEFT_TOP, word, small.clone(), colour).right();
    }
    painter.text(egui::pos2(x, rect.top() + 3.0), egui::Align2::LEFT_TOP, format!(", last {}", fmt_span(span)), small.clone(), weak);
    // The scale goes on the right, clear of the caption.
    painter.text(rect.right_top() + egui::vec2(-6.0, 3.0), egui::Align2::RIGHT_TOP, format!("{hi:.1} °C"), small.clone(), weak);
    painter.text(rect.right_bottom() + egui::vec2(-6.0, -3.0), egui::Align2::RIGHT_BOTTOM, format!("{lo:.1} °C"), small, weak);

    // Reading a ramp off the picture is guesswork; name the sample under the
    // cursor instead.
    if let Some(cursor) = response.hover_pos()
        && let Some((i, _)) = pts
            .iter()
            .enumerate()
            .min_by(|(_, a), (_, b)| (a.x - cursor.x).abs().total_cmp(&(b.x - cursor.x).abs()))
    {
        let s = shown[i];
        painter.line_segment([egui::pos2(pts[i].x, plot.top()), egui::pos2(pts[i].x, plot.bottom())], egui::Stroke::new(1.0, weak));
        let target = s.target.map(|v| format!("{v:.2}")).unwrap_or_else(|| "—".into());
        response.on_hover_text(format!("{:.2} °C · target {target} °C\n{} ago", s.measured, fmt_span(now - s.t)));
    }
}

/// A rough age or width in time, for a label that only needs the order.
fn fmt_span(secs: f64) -> String {
    let secs = secs.max(0.0);
    if secs >= 90.0 { format!("{:.0} min", secs / 60.0) } else { format!("{secs:.0} s") }
}

fn json_num(v: &serde_json::Value, key: &str) -> String {
    v.get(key).map(|x| x.to_string()).unwrap_or_else(|| "?".into())
}

/// serde serializes std::time::Duration as {"secs": u64, "nanos": u32}.
fn duration_secs(v: Option<&serde_json::Value>) -> f64 {
    let Some(v) = v else { return 0.0 };
    if let Some(n) = v.as_f64() {
        return n;
    }
    let secs = v.get("secs").and_then(|x| x.as_f64()).unwrap_or(0.0);
    let nanos = v.get("nanos").and_then(|x| x.as_f64()).unwrap_or(0.0);
    secs + nanos / 1e9
}

fn fmt_secs(s: f64) -> String {
    let s = s.max(0.0) as u64;
    format!("{:02}:{:02}:{:02}", s / 3600, (s / 60) % 60, s % 60)
}

fn now_ms() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_millis() as u64)
        .unwrap_or(0)
}

/// The band the rig works in. The Peltier can go further, but a setpoint
/// outside this is far more likely to be a typo than an intent.
const TEMP_MIN_C: f32 = 4.0;
const TEMP_MAX_C: f32 = 95.0;
/// How far back the temperature trend is drawn. Matches the ring the workers
/// keep, so the plot shows everything there is rather than a window inside it.
const TEMP_TREND_SECS: f64 = 600.0;

fn temp_status_color(status: TempStatus) -> Color32 {
    match status {
        TempStatus::Off => Color32::from_rgb(110, 115, 125),
        TempStatus::Offline => Color32::from_rgb(220, 70, 70),
        TempStatus::Fault => Color32::from_rgb(225, 80, 80),
        TempStatus::Idle => Color32::from_rgb(140, 150, 165),
        TempStatus::Driving => Color32::from_rgb(64, 150, 240),
        TempStatus::Holding => Color32::from_rgb(60, 180, 90),
    }
}

fn list_ports() -> Vec<String> {
    crate::remote::server::list_ports()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The painter path is easy to break with a slice index or a division by a
    /// zero-width span; run it headless over the shapes it has to survive.
    #[test]
    fn the_trend_draws_every_shape_of_history() {
        let sample = |t: f64, measured: f32, target: Option<f32>| TempSample { t, measured, target };
        let cases: Vec<VecDeque<TempSample>> = vec![
            VecDeque::new(),
            VecDeque::from(vec![sample(0.0, 21.0, None)]),
            VecDeque::from(vec![sample(0.0, 37.0, Some(37.0)), sample(1.0, 37.0, Some(37.0))]),
            // A target that appears, goes away and comes back: three traces
            // with holes, and a measurement that spans them.
            VecDeque::from(vec![
                sample(0.0, 21.0, None),
                sample(30.0, 25.0, Some(60.0)),
                sample(60.0, 40.0, None),
                sample(90.0, 55.0, Some(60.0)),
            ]),
        ];
        for history in &cases {
            // Both ends of the window selector, and a degenerate one, so a
            // fixed span cannot divide by zero or invert the axis.
            for window in [300.0, 600.0, 0.0] {
                egui::__run_test_ui(|ui| temp_trend(ui, history, 120.0, window));
            }
        }
    }
}

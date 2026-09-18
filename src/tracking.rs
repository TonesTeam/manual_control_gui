//! Estimated liquid levels and derived states for the six slots and the six
//! external reagent bottles. Nothing on the rig measures these directly (the
//! optical sensors only see liquid in a tube), so levels are integrated from
//! piston travel while a path is connected, and can be corrected by hand.

use std::time::{Duration, Instant};

use eframe::egui::Color32;
use serde::{Deserialize, Serialize};

use crate::bus::BusState;
use crate::config::{Backend, Settings, data_file};
use crate::devices::{DeviceId, slot_for_sv02_port, slot_for_sv03_port};
use crate::schematic::{bottle_for_tc02, live_reagent};

/// C1..C3 are 1 L, C4..C6 are 0.5 L (drawio).
pub const BOTTLE_CAPACITY_ML: [f32; 6] = [1000.0, 1000.0, 1000.0, 500.0, 500.0, 500.0];
const LEVELS_FILE: &str = "tstand_levels.json";
const AUTOSAVE: Duration = Duration::from_secs(5);

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SlotState {
    /// controller_v2 reports the slot as not connected.
    Missing,
    /// controller_v2 is running a step on it (temperature, incubation, ...).
    Busy,
    Filling,
    Draining,
    FillPath,
    DrainPath,
    Filled,
    Empty,
}

impl SlotState {
    pub fn label(self) -> &'static str {
        match self {
            SlotState::Missing => "MISSING",
            SlotState::Busy => "BUSY",
            SlotState::Filling => "FILLING",
            SlotState::Draining => "DRAINING",
            SlotState::FillPath => "FILL PATH",
            SlotState::DrainPath => "DRAIN PATH",
            SlotState::Filled => "FILLED",
            SlotState::Empty => "EMPTY",
        }
    }

    pub fn color(self) -> Color32 {
        match self {
            SlotState::Missing => Color32::from_rgb(140, 140, 150),
            SlotState::Busy => Color32::from_rgb(230, 170, 40),
            SlotState::Filling | SlotState::Draining => Color32::from_rgb(64, 150, 240),
            SlotState::FillPath | SlotState::DrainPath => Color32::from_rgb(60, 110, 170),
            SlotState::Filled => Color32::from_rgb(60, 180, 90),
            SlotState::Empty => Color32::from_rgb(110, 115, 125),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BottleState {
    /// Connected to PP01, which is aspirating from it.
    Drawing,
    /// Connected to PP01, which is pushing liquid back into it.
    Returning,
    /// Connected to PP01, pump idle.
    Linked,
    Empty,
    Low,
    Ok,
}

impl BottleState {
    pub fn label(self) -> &'static str {
        match self {
            BottleState::Drawing => "DRAWING",
            BottleState::Returning => "RETURN",
            BottleState::Linked => "LINKED",
            BottleState::Empty => "EMPTY",
            BottleState::Low => "LOW",
            BottleState::Ok => "OK",
        }
    }

    pub fn color(self) -> Color32 {
        match self {
            BottleState::Drawing | BottleState::Returning => Color32::from_rgb(64, 150, 240),
            BottleState::Linked => Color32::from_rgb(60, 110, 170),
            BottleState::Empty => Color32::from_rgb(225, 70, 70),
            BottleState::Low => Color32::from_rgb(230, 170, 40),
            BottleState::Ok => Color32::from_rgb(60, 180, 90),
        }
    }
}

#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
#[serde(default)]
pub struct Levels {
    pub bottles_ml: [f32; 6],
    pub slots_ul: [f32; 6],
}

impl Default for Levels {
    fn default() -> Self {
        Self { bottles_ml: BOTTLE_CAPACITY_ML, slots_ul: [0.0; 6] }
    }
}

/// Per-frame snapshot the schematic and side panel draw from.
#[derive(Clone, Debug)]
pub struct FluidView {
    /// (state, estimated µL) per slot.
    pub slots: [(SlotState, f32); 6],
    /// (state, estimated mL) per bottle.
    pub bottles: [(BottleState, f32); 6],
    pub slot_capacity_ul: f32,
}

/// Where a pump stroke goes, so a valve switch between two readings isn't counted.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Path {
    Bottle(u16),
    Slot(u16),
    None,
}

pub struct Tracker {
    pub levels: Levels,
    /// PP01 position with the bottle and slot paths at that reading.
    last_pp01: Option<(u16, Path, Path)>,
    /// PP02 position with the slot path at that reading.
    last_pp02: Option<(u16, Path)>,
    dirty: bool,
    autosave: bool,
    last_save: Instant,
}

impl Tracker {
    pub fn new(levels: Levels) -> Self {
        Self { levels, last_pp01: None, last_pp02: None, dirty: false, autosave: false, last_save: Instant::now() }
    }

    /// Hardware levels persist between runs; the simulator starts full and never
    /// touches the file, so simulated runs can't corrupt the real estimates.
    ///
    /// Pass `Settings::effective_backend`: what matters is whether real liquid
    /// is moving, not whether the bus driving it is local or on the rig's
    /// server. Levels are estimated from the piston travel this window sees,
    /// so with several GUIs watching one rig each keeps its own file; they
    /// agree while all of them are running and drift while one is not.
    pub fn for_backend(backend: Backend) -> Self {
        match backend {
            Backend::Serial => Self::load(),
            // Never reached: `effective_backend` resolves Remote to one of the
            // two above. Treated as simulated so a stray value cannot write
            // over the real estimates.
            Backend::Simulator | Backend::Remote => Self::new(Levels::default()),
        }
    }

    /// Writes pending changes, if this tracker persists at all.
    pub fn flush(&mut self) {
        if self.autosave && self.dirty {
            let _ = self.save();
        }
    }

    /// Restores saved levels and saves changes every few seconds.
    fn load() -> Self {
        let levels = std::fs::read_to_string(data_file(LEVELS_FILE))
            .ok()
            .and_then(|t| serde_json::from_str(&t).ok())
            .unwrap_or_default();
        Self { autosave: true, ..Self::new(levels) }
    }

    pub fn save(&mut self) -> Result<(), String> {
        let text = serde_json::to_string_pretty(&self.levels).map_err(|e| e.to_string())?;
        std::fs::write(data_file(LEVELS_FILE), text).map_err(|e| e.to_string())?;
        self.dirty = false;
        self.last_save = Instant::now();
        Ok(())
    }

    pub fn set_bottle(&mut self, c: u16, ml: f32) {
        self.levels.bottles_ml[c as usize - 1] = ml.clamp(0.0, BOTTLE_CAPACITY_ML[c as usize - 1]);
        self.changed_by_hand();
    }

    pub fn set_slot(&mut self, n: u16, ul: f32) {
        self.levels.slots_ul[n as usize - 1] = ul.max(0.0);
        self.changed_by_hand();
    }

    fn changed_by_hand(&mut self) {
        self.dirty = true;
        if self.autosave {
            let _ = self.save();
        }
    }

    /// Integrates piston travel since the last reading into bottle and slot levels.
    pub fn update(&mut self, state: &BusState, settings: &Settings) {
        let pp01 = state.dev(DeviceId::Pp01);
        let bottle = live_reagent(state).map_or(Path::None, |(ch, _)| Path::Bottle(bottle_for_tc02(ch)));
        let fill = if pp01.solenoid_input.unwrap_or(false) {
            Path::None
        } else {
            state.dev(DeviceId::Sv02).value.and_then(slot_for_sv02_port).map_or(Path::None, Path::Slot)
        };
        self.last_pp01 = match (pp01.online, pp01.value) {
            (true, Some(pos)) => {
                if let Some((prev, prev_bottle, prev_fill)) = self.last_pp01 {
                    // Positive: liquid moved into the pump.
                    let into_pump_ul = (pos as f32 - prev as f32) * settings.ul_per_step(DeviceId::Pp01);
                    if into_pump_ul != 0.0 {
                        if let Path::Bottle(c) = bottle
                            && bottle == prev_bottle
                        {
                            self.add_bottle(c, -into_pump_ul / 1000.0);
                        }
                        if let Path::Slot(n) = fill
                            && fill == prev_fill
                        {
                            self.add_slot(n, -into_pump_ul);
                        }
                    }
                }
                Some((pos, bottle, fill))
            }
            _ => None,
        };

        let pp02 = state.dev(DeviceId::Pp02);
        let drain = state.dev(DeviceId::Sv03).value.and_then(slot_for_sv03_port).map_or(Path::None, Path::Slot);
        self.last_pp02 = match (pp02.online, pp02.value) {
            (true, Some(pos)) => {
                if let Some((prev, prev_drain)) = self.last_pp02
                    && let Path::Slot(n) = drain
                    && drain == prev_drain
                {
                    self.add_slot(n, -(pos as f32 - prev as f32) * settings.ul_per_step(DeviceId::Pp02));
                }
                Some((pos, drain))
            }
            _ => None,
        };

        if self.autosave && self.dirty && self.last_save.elapsed() > AUTOSAVE {
            let _ = self.save();
        }
    }

    fn add_bottle(&mut self, c: u16, ml: f32) {
        let i = c as usize - 1;
        self.levels.bottles_ml[i] = (self.levels.bottles_ml[i] + ml).clamp(0.0, BOTTLE_CAPACITY_ML[i]);
        self.dirty = true;
    }

    fn add_slot(&mut self, n: u16, ul: f32) {
        let i = n as usize - 1;
        self.levels.slots_ul[i] = (self.levels.slots_ul[i] + ul).max(0.0);
        self.dirty = true;
    }

    /// `controller`: slot descriptions from controller_v2 (`Idle`, `Missing`, a step name), when reachable.
    pub fn slot_state(&self, n: u16, state: &BusState, controller: Option<&str>) -> SlotState {
        if controller.is_some_and(|d| d.eq_ignore_ascii_case("missing")) {
            return SlotState::Missing;
        }
        let pp01 = state.dev(DeviceId::Pp01);
        let fill = !pp01.solenoid_input.unwrap_or(false) && state.dev(DeviceId::Sv02).value.and_then(slot_for_sv02_port) == Some(n);
        let drain = state.dev(DeviceId::Sv03).value.and_then(slot_for_sv03_port) == Some(n);
        if fill && pp01.motion < 0 {
            return SlotState::Filling;
        }
        if drain && state.dev(DeviceId::Pp02).motion > 0 {
            return SlotState::Draining;
        }
        if controller.is_some_and(|d| !d.eq_ignore_ascii_case("idle")) {
            return SlotState::Busy;
        }
        if fill {
            return SlotState::FillPath;
        }
        if drain {
            return SlotState::DrainPath;
        }
        if self.levels.slots_ul[n as usize - 1] > 0.5 { SlotState::Filled } else { SlotState::Empty }
    }

    pub fn bottle_state(&self, c: u16, state: &BusState) -> BottleState {
        let i = c as usize - 1;
        if self.levels.bottles_ml[i] < 0.5 {
            return BottleState::Empty;
        }
        let linked = live_reagent(state).is_some_and(|(ch, _)| bottle_for_tc02(ch) == c);
        match (linked, state.dev(DeviceId::Pp01).motion) {
            (true, m) if m > 0 => BottleState::Drawing,
            (true, m) if m < 0 => BottleState::Returning,
            (true, _) => BottleState::Linked,
            _ if self.levels.bottles_ml[i] < BOTTLE_CAPACITY_ML[i] * 0.1 => BottleState::Low,
            _ => BottleState::Ok,
        }
    }

    pub fn view(&self, state: &BusState, settings: &Settings, controller: &[Option<String>; 6]) -> FluidView {
        FluidView {
            slots: std::array::from_fn(|i| {
                let n = i as u16 + 1;
                (self.slot_state(n, state, controller[i].as_deref()), self.levels.slots_ul[i])
            }),
            bottles: std::array::from_fn(|i| (self.bottle_state(i as u16 + 1, state), self.levels.bottles_ml[i])),
            slot_capacity_ul: settings.slot_volume_ul,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn rig() -> (Tracker, BusState, Settings) {
        let mut state = BusState::for_tests();
        for d in state.devices.iter_mut() {
            d.online = true;
            d.value = Some(0);
        }
        (Tracker::new(Levels::default()), state, Settings::default())
    }

    fn set(state: &mut BusState, id: DeviceId, value: u16) {
        state.devices[id as usize].value = Some(value);
    }

    #[test]
    fn aspirating_through_sv01_draws_from_the_bottle() {
        let (mut tr, mut st, cfg) = rig();
        st.devices[DeviceId::Pp01 as usize].solenoid_input = Some(true);
        set(&mut st, DeviceId::Sv01, 1);
        tr.update(&st, &cfg);
        set(&mut st, DeviceId::Pp01, 1000);
        tr.update(&st, &cfg);
        assert!((tr.levels.bottles_ml[0] - (1000.0 - 1000.0 * 2.083 / 1000.0)).abs() < 1e-3);
        assert_eq!(tr.levels.bottles_ml[1], 1000.0);
    }

    #[test]
    fn filling_through_sv02_and_draining_through_sv03() {
        let (mut tr, mut st, cfg) = rig();
        st.devices[DeviceId::Pp01 as usize].solenoid_input = Some(false);
        set(&mut st, DeviceId::Pp01, 500);
        set(&mut st, DeviceId::Sv02, 15); // slot 1
        set(&mut st, DeviceId::Sv03, 6); // slot 1
        tr.update(&st, &cfg);
        set(&mut st, DeviceId::Pp01, 400); // dispense 100 steps into slot 1
        tr.update(&st, &cfg);
        assert!((tr.levels.slots_ul[0] - 208.3).abs() < 1e-2);
        set(&mut st, DeviceId::Pp02, 300); // drain 300 steps out of slot 1
        tr.update(&st, &cfg);
        assert!((tr.levels.slots_ul[0] - (208.3 - 300.0 * 0.416)).abs() < 1e-2);
    }

    #[test]
    fn valve_switch_between_readings_is_not_counted() {
        let (mut tr, mut st, cfg) = rig();
        st.devices[DeviceId::Pp01 as usize].solenoid_input = Some(true);
        set(&mut st, DeviceId::Sv01, 1);
        tr.update(&st, &cfg);
        set(&mut st, DeviceId::Sv01, 2);
        set(&mut st, DeviceId::Pp01, 1000);
        tr.update(&st, &cfg);
        assert_eq!(tr.levels.bottles_ml, BOTTLE_CAPACITY_ML);
    }

    #[test]
    fn states_follow_paths_motion_levels_and_controller() {
        let (mut tr, mut st, _) = rig();
        st.devices[DeviceId::Pp01 as usize].solenoid_input = Some(false);
        set(&mut st, DeviceId::Sv02, 3); // C3
        assert_eq!(tr.bottle_state(3, &st), BottleState::Linked);
        st.devices[DeviceId::Pp01 as usize].motion = 1;
        assert_eq!(tr.bottle_state(3, &st), BottleState::Drawing);
        tr.set_bottle(3, 0.0);
        assert_eq!(tr.bottle_state(3, &st), BottleState::Empty);
        tr.set_bottle(5, 40.0);
        assert_eq!(tr.bottle_state(5, &st), BottleState::Low);

        set(&mut st, DeviceId::Sv02, 12); // slot 4
        st.devices[DeviceId::Pp01 as usize].motion = -1;
        assert_eq!(tr.slot_state(4, &st, Some("Idle")), SlotState::Filling);
        st.devices[DeviceId::Pp01 as usize].motion = 0;
        assert_eq!(tr.slot_state(4, &st, None), SlotState::FillPath);
        assert_eq!(tr.slot_state(2, &st, Some("Increasing temperature")), SlotState::Busy);
        assert_eq!(tr.slot_state(3, &st, Some("Missing")), SlotState::Missing);
        tr.set_slot(6, 150.0);
        assert_eq!(tr.slot_state(6, &st, Some("Idle")), SlotState::Filled);
        assert_eq!(tr.slot_state(5, &st, Some("Idle")), SlotState::Empty);
    }
}

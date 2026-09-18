//! Which parts of the rig are actually plumbed, and what they are plumbed to.
//!
//! The schematic in [`crate::schematic`] draws the rig as designed in
//! `Tones_Liqud_Processing.drawio`: six slots, six reagent bottles, every valve
//! port in use. A rig under commissioning is not that. Lines get capped, slots
//! go unfitted, and a port that the diagram calls "C2 (1 L)" may be carrying
//! something else entirely.
//!
//! This module carries that difference as data. It decides three things:
//!
//! * **what a port is**, which is what the operator reads on screen;
//! * **whether a port may be switched to**, checked on the machine that holds
//!   the bus, not just greyed out in the interface;
//! * **which slots and sensors exist**, so the schematic does not show six of
//!   everything when one is fitted.
//!
//! [`Rig::restrict`] turns the second one off without losing the first: the
//! labels stay, the guard does not. Commissioning needs to reach a capped port
//! sometimes, and a checkbox is better than editing a config to get there.

use serde::{Deserialize, Serialize};

use crate::devices::DeviceId;

/// What a valve port carries on this rig.
#[derive(Serialize, Deserialize, Clone, Copy, Debug, PartialEq, Eq)]
pub enum Role {
    /// Wash / buffer supply.
    Wash,
    /// Reagent supply line.
    Reagent,
    /// Feeds a slot.
    SlotFill,
    /// Drains a slot.
    SlotDrain,
    /// Ordinary waste.
    Waste,
    /// Waste for toxic reagents.
    DangerWaste,
    /// Air, through the filter.
    Air,
    /// Plumbed, but none of the above.
    Other,
}

impl Role {
    pub fn label(self) -> &'static str {
        match self {
            Role::Wash => "wash",
            Role::Reagent => "reagent",
            Role::SlotFill => "slot fill",
            Role::SlotDrain => "slot drain",
            Role::Waste => "waste",
            Role::DangerWaste => "danger waste",
            Role::Air => "air",
            Role::Other => "other",
        }
    }

    /// Whether liquid drawn through this port comes from a reservoir the level
    /// tracker should debit.
    pub fn is_supply(self) -> bool {
        matches!(self, Role::Wash | Role::Reagent)
    }
}

/// One plumbed valve port.
#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
pub struct PortUse {
    pub valve: DeviceId,
    pub port: u16,
    /// What to show on screen. Empty falls back to the drawio label.
    pub label: String,
    pub role: Role,
}

impl PortUse {
    fn new(valve: DeviceId, port: u16, label: &str, role: Role) -> Self {
        Self { valve, port, label: label.to_string(), role }
    }
}

/// Where an optical liquid sensor sits on the diagram.
#[derive(Serialize, Deserialize, Clone, Copy, Debug, PartialEq, Eq)]
pub enum SensorAt {
    /// Between PP01 and the holding coil — drawn as S1 on the drawio.
    Inline1,
    /// Between the coil and SV02's common port — drawn as S2.
    Inline2,
    /// On the fitted slot's feed.
    Slot,
}

/// One channel of the optical liquid-sensor board.
///
/// The board multiplexes six detectors into one CAN frame; `channel` is the
/// index into it, 0–5, and `name` is what that channel is called on the rig
/// (`A0`–`A5` in the firmware's own parameter table).
#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
pub struct Sensor {
    pub at: SensorAt,
    pub name: String,
    pub channel: u8,
}

impl Sensor {
    fn new(at: SensorAt, name: &str, channel: u8) -> Self {
        Self { at, name: name.to_string(), channel }
    }
}

#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
#[serde(default)]
pub struct Rig {
    /// Refuse to switch a valve to a port that is not listed in `ports`, and
    /// grey those ports out. Turn off to reach an uncommissioned port.
    pub restrict: bool,
    /// Slots with a cassette fitted. Everything else is drawn as absent.
    pub slots: Vec<u16>,
    /// The optical liquid sensors that are wired, and which board channel each
    /// one is. Anything not listed is not drawn.
    pub sensors: Vec<Sensor>,
    /// Every port that is plumbed. A port missing from this list is capped.
    pub ports: Vec<PortUse>,
}

impl Default for Rig {
    /// The rig as it stands: one slot, one reagent line, one wash line.
    ///
    /// SV02 port 10 and SV03 port 1 are the two ends of the same slot under
    /// the drawio numbering (`16 - slot` and `7 - slot`), which is slot 6.
    fn default() -> Self {
        Self {
            restrict: true,
            slots: vec![6],
            // A4 sits 100 mm before the router sensor and A5 just after the
            // main selector — the same two the diagram draws as S1 and S2, and
            // the same channels controller_v2 reserves as PRE_ROUTER and
            // ROUTER. A2 is the fitted slot's detector.
            sensors: vec![
                Sensor::new(SensorAt::Inline1, "A4", 4),
                Sensor::new(SensorAt::Inline2, "A5", 5),
                Sensor::new(SensorAt::Slot, "A2", 2),
            ],
            ports: vec![
                PortUse::new(DeviceId::Sv01, 1, "Wash liquid", Role::Wash),
                PortUse::new(DeviceId::Sv02, 2, "Reagent line", Role::Reagent),
                PortUse::new(DeviceId::Sv02, 9, "Waste", Role::Waste),
                PortUse::new(DeviceId::Sv02, 10, "Slot 6 fill", Role::SlotFill),
                PortUse::new(DeviceId::Sv02, 16, "Air", Role::Air),
                PortUse::new(DeviceId::Sv03, 1, "Slot 6 drain", Role::SlotDrain),
            ],
        }
    }
}

impl Rig {
    /// Everything the diagram shows, with no guard. The rig as designed.
    pub fn unrestricted() -> Self {
        Self { restrict: false, slots: (1..=6).collect(), sensors: Vec::new(), ports: Vec::new() }
    }

    pub fn sensor_at(&self, at: SensorAt) -> Option<&Sensor> {
        self.sensors.iter().find(|s| s.at == at)
    }

    /// What the fitted slot's detector is called, for the slot's label.
    pub fn slot_sensor_name(&self) -> Option<&str> {
        self.sensor_at(SensorAt::Slot).map(|s| s.name.as_str())
    }

    pub fn find(&self, valve: DeviceId, port: u16) -> Option<&PortUse> {
        self.ports.iter().find(|u| u.valve == valve && u.port == port)
    }

    /// Whether this port may be switched to.
    ///
    /// Port 0 is the reset position rather than a plumbed line, so it is always
    /// allowed: it is how a valve is parked.
    pub fn port_allowed(&self, valve: DeviceId, port: u16) -> bool {
        !self.restrict || port == 0 || self.find(valve, port).is_some()
    }

    /// Whether this port is plumbed, regardless of the guard. Drives how it is
    /// drawn, so an unrestricted rig still shows which lines are real.
    pub fn port_plumbed(&self, valve: DeviceId, port: u16) -> bool {
        self.ports.is_empty() || self.find(valve, port).is_some()
    }

    /// What to call this port: the rig's own label where there is one, the
    /// drawio label otherwise.
    pub fn port_label(&self, valve: DeviceId, port: u16) -> String {
        match self.find(valve, port) {
            Some(u) if !u.label.trim().is_empty() => u.label.clone(),
            _ => valve.port_label(port).to_string(),
        }
    }

    pub fn role(&self, valve: DeviceId, port: u16) -> Option<Role> {
        self.find(valve, port).map(|u| u.role)
    }

    pub fn slot_fitted(&self, slot: u16) -> bool {
        self.slots.is_empty() || self.slots.contains(&slot)
    }

    /// The single fitted slot, when there is exactly one. The temperature
    /// board controls one slot, so this is what its readout belongs to.
    pub fn only_slot(&self) -> Option<u16> {
        match self.slots.as_slice() {
            [slot] => Some(*slot),
            _ => None,
        }
    }

    /// Reagent bottles still in use, by their C-number. A bottle whose port is
    /// capped is not there to draw from.
    pub fn bottle_in_use(&self, bottle: u16) -> bool {
        if self.ports.is_empty() {
            return true;
        }
        // Bottles hang off SV01 1..3 and SV02 1..6 in the diagram; a bottle is
        // in use if either of its ports is still plumbed.
        self.port_plumbed(DeviceId::Sv02, bottle) || (bottle <= 3 && self.port_plumbed(DeviceId::Sv01, bottle))
    }

    /// Why a port cannot be switched to, for a tooltip.
    pub fn blocked_reason(&self, valve: DeviceId, port: u16) -> Option<String> {
        (!self.port_allowed(valve, port)).then(|| {
            format!(
                "{} port {port} is not plumbed on this rig. Turn off Settings → Rig → \"Restrict to plumbed ports\" to switch to it anyway.",
                valve.tag()
            )
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::devices::{sv02_port_for_slot, sv03_port_for_slot};

    #[test]
    fn the_default_rig_is_the_one_slot_that_is_fitted() {
        let rig = Rig::default();
        // The two ends of the fitted slot, as the drawio numbers them.
        assert_eq!(rig.only_slot(), Some(6));
        assert_eq!(rig.slot_sensor_name(), Some("A2"));
        assert_eq!(sv02_port_for_slot(6), 10, "SV02 port 10 is the slot's fill");
        assert_eq!(sv03_port_for_slot(6), 1, "SV03 port 1 is the slot's drain");
        assert!(rig.slot_fitted(6));
        for slot in [1, 2, 3, 4, 5] {
            assert!(!rig.slot_fitted(slot), "slot {slot} is not fitted");
        }
    }

    #[test]
    fn only_the_plumbed_ports_are_allowed() {
        let rig = Rig::default();
        for (valve, port) in [
            (DeviceId::Sv01, 1),
            (DeviceId::Sv02, 2),
            (DeviceId::Sv02, 9),
            (DeviceId::Sv02, 10),
            (DeviceId::Sv02, 16),
            (DeviceId::Sv03, 1),
        ] {
            assert!(rig.port_allowed(valve, port), "{} port {port} should be allowed", valve.tag());
        }
        // A capped line the diagram still shows.
        assert!(!rig.port_allowed(DeviceId::Sv01, 2));
        assert!(!rig.port_allowed(DeviceId::Sv02, 1));
        assert!(!rig.port_allowed(DeviceId::Sv02, 15));
        assert!(!rig.port_allowed(DeviceId::Sv03, 6));
    }

    #[test]
    fn a_valve_can_always_be_parked() {
        let rig = Rig::default();
        for valve in [DeviceId::Sv01, DeviceId::Sv02, DeviceId::Sv03] {
            assert!(rig.port_allowed(valve, 0), "reset must never be blocked");
        }
    }

    #[test]
    fn the_rig_relabels_ports_the_diagram_names_differently() {
        let rig = Rig::default();
        // The diagram calls these bottles; on this rig they are supply lines.
        assert_eq!(DeviceId::Sv01.port_label(1), "C1 (1 L)");
        assert_eq!(rig.port_label(DeviceId::Sv01, 1), "Wash liquid");
        assert_eq!(DeviceId::Sv02.port_label(2), "C2 (1 L)");
        assert_eq!(rig.port_label(DeviceId::Sv02, 2), "Reagent line");
        // A port with no rig entry keeps the diagram's label.
        assert_eq!(rig.port_label(DeviceId::Sv02, 4), DeviceId::Sv02.port_label(4));
    }

    #[test]
    fn turning_the_guard_off_allows_everything_but_keeps_the_labels() {
        let rig = Rig { restrict: false, ..Rig::default() };
        assert!(rig.port_allowed(DeviceId::Sv02, 15), "the guard is off");
        assert!(!rig.port_plumbed(DeviceId::Sv02, 15), "but it is still not plumbed");
        assert_eq!(rig.port_label(DeviceId::Sv02, 2), "Reagent line");
    }

    #[test]
    fn an_empty_rig_describes_the_diagram_as_drawn() {
        let rig = Rig::unrestricted();
        for slot in 1..=6 {
            assert!(rig.slot_fitted(slot));
        }
        assert!(rig.port_allowed(DeviceId::Sv02, 15));
        assert!(rig.port_plumbed(DeviceId::Sv02, 15));
        assert_eq!(rig.port_label(DeviceId::Sv02, 15), DeviceId::Sv02.port_label(15));
    }

    #[test]
    fn roles_say_where_liquid_comes_from() {
        let rig = Rig::default();
        assert_eq!(rig.role(DeviceId::Sv01, 1), Some(Role::Wash));
        assert_eq!(rig.role(DeviceId::Sv02, 2), Some(Role::Reagent));
        assert_eq!(rig.role(DeviceId::Sv02, 10), Some(Role::SlotFill));
        assert_eq!(rig.role(DeviceId::Sv03, 1), Some(Role::SlotDrain));
        assert!(Role::Wash.is_supply() && Role::Reagent.is_supply());
        assert!(!Role::Waste.is_supply() && !Role::Air.is_supply());
    }
}

#[cfg(test)]
mod sensor_tests {
    use super::*;

    /// The diagram draws two in-line sensors as S1 and S2; on this rig they
    /// are board channels A4 and A5, which is what the operator reads.
    #[test]
    fn the_inline_sensors_are_named_for_the_board_channels() {
        let rig = Rig::default();
        let s1 = rig.sensor_at(SensorAt::Inline1).expect("S1 is wired");
        assert_eq!((s1.name.as_str(), s1.channel), ("A4", 4));
        let s2 = rig.sensor_at(SensorAt::Inline2).expect("S2 is wired");
        assert_eq!((s2.name.as_str(), s2.channel), ("A5", 5));
        let slot = rig.sensor_at(SensorAt::Slot).expect("the slot has a detector");
        assert_eq!((slot.name.as_str(), slot.channel), ("A2", 2));
    }

    #[test]
    fn every_sensor_is_a_channel_the_board_actually_has() {
        for s in Rig::default().sensors {
            assert!(s.channel < 6, "{} is channel {}, the board has 0..=5", s.name, s.channel);
        }
    }
}

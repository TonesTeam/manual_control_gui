//! Where the liquid actually is.
//!
//! The pump displaces a known volume, the tubing has a known bore, and the
//! optical sensors say wet or dry at fixed points. Put together, that is enough
//! to say which stretches of tube hold liquid, which hold air, and — the part
//! that matters most — which nobody knows.
//!
//! # Volume is the coordinate, not length
//!
//! Every measurement this rig can make is a volume: the pump counts steps. A
//! length only appears at the end, by dividing by the bore. So the model is
//! built in µL throughout and converts to millimetres for display. Storing
//! lengths instead would mean re-deriving them every time the bore assumption
//! changed, and getting the arithmetic wrong in two places.
//!
//! # Landmarks are sensors
//!
//! Links run between optical sensors and nothing else, because a sensor is the
//! only thing that can tell you a front has arrived. A link's volume is
//! measured by pushing a front from one sensor to the next and reading the
//! steps off the pump — which is exactly what
//! [`crate::routine`] does. Anything not measured that way is carried as an
//! estimate and says so.
//!
//! # Unknown is a first-class state
//!
//! A tube nobody has watched since the rig was switched on is not empty; it is
//! unknown. Treating unknown as air is how you push a slug of old reagent into
//! a slot and call it a wash. [`Fill::Unknown`] survives every operation that
//! cannot rule it out, and is what the schematic draws as `?`.

use serde::{Deserialize, Serialize};

use crate::config::data_file;

/// What occupies a stretch of tube.
#[derive(Serialize, Deserialize, Clone, Copy, Debug, PartialEq, Eq)]
pub enum Fill {
    Liquid,
    Air,
    /// Never observed, or disturbed by something this model does not track.
    Unknown,
}

impl Fill {
    pub fn label(self) -> &'static str {
        match self {
            Fill::Liquid => "liquid",
            Fill::Air => "air",
            Fill::Unknown => "?",
        }
    }
}

/// A point the rig can actually detect a front at.
#[derive(Serialize, Deserialize, Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Landmark {
    /// The pump's own barrel.
    Pump,
    /// In-line sensor between the pump and the coil (S1 on the diagram).
    S1,
    /// In-line sensor between the coil and the main selector (S2).
    S2,
    /// The fitted slot's own detector.
    Slot,
    /// The wash supply, through SV01.
    Wash,
    /// Waste, through SV02.
    Waste,
}

impl Landmark {
    pub fn label(self) -> &'static str {
        match self {
            Landmark::Pump => "PP01",
            Landmark::S1 => "S1",
            Landmark::S2 => "S2",
            Landmark::Slot => "slot",
            Landmark::Wash => "wash",
            Landmark::Waste => "waste",
        }
    }
}

/// Cross-sectional area of a round bore, mm² — which is also µL per mm, since
/// a µL is a mm³.
pub fn bore_area_mm2(id_mm: f32) -> f32 {
    std::f32::consts::PI * (id_mm / 2.0).powi(2)
}

/// How long a stretch of tube holds this volume.
pub fn length_mm(volume_ul: f32, id_mm: f32) -> f32 {
    let area = bore_area_mm2(id_mm);
    if area <= 0.0 { 0.0 } else { volume_ul / area }
}

/// How much volume a length of tube holds.
pub fn volume_ul(length_mm: f32, id_mm: f32) -> f32 {
    length_mm * bore_area_mm2(id_mm)
}

/// One stretch of tube between two landmarks.
#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct Link {
    pub from: Landmark,
    pub to: Landmark,
    /// Internal volume. Either measured by a calibration run or estimated.
    pub volume_ul: f32,
    /// True once a run has measured it, rather than it being a starting guess.
    pub measured: bool,
    /// Contents from the `from` end, as cumulative µL boundaries. The last
    /// entry always ends at `volume_ul`.
    pub runs: Vec<(f32, Fill)>,
}

impl Link {
    fn new(from: Landmark, to: Landmark, volume_ul: f32) -> Self {
        Self { from, to, volume_ul, measured: false, runs: vec![(volume_ul, Fill::Unknown)] }
    }

    pub fn length_mm(&self, id_mm: f32) -> f32 {
        length_mm(self.volume_ul, id_mm)
    }

    /// What is at `at_ul` from the `from` end.
    pub fn fill_at(&self, at_ul: f32) -> Fill {
        for (end, fill) in &self.runs {
            if at_ul < *end {
                return *fill;
            }
        }
        self.runs.last().map(|(_, f)| *f).unwrap_or(Fill::Unknown)
    }

    /// Where the liquid front sits from the `from` end, if there is one.
    pub fn front_ul(&self) -> Option<f32> {
        let mut at = 0.0;
        for (end, fill) in &self.runs {
            if *fill == Fill::Liquid {
                at = *end;
            } else if at > 0.0 {
                return Some(at);
            }
        }
        (at > 0.0 && at < self.volume_ul).then_some(at)
    }

    /// Merges neighbouring runs of the same fill and drops empty ones, so the
    /// list cannot grow without bound as fronts move through.
    fn tidy(&mut self) {
        let mut out: Vec<(f32, Fill)> = Vec::with_capacity(self.runs.len());
        for (end, fill) in self.runs.drain(..) {
            match out.last_mut() {
                Some((prev_end, prev_fill)) if *prev_fill == fill => *prev_end = end,
                Some((prev_end, _)) if (end - *prev_end).abs() < 1e-4 => {}
                _ => out.push((end, fill)),
            }
        }
        if out.is_empty() {
            out.push((self.volume_ul, Fill::Unknown));
        } else if let Some(last) = out.last_mut() {
            last.0 = self.volume_ul;
        }
        self.runs = out;
    }
}

/// The rig's tubing, and what is in it.
#[derive(Serialize, Deserialize, Clone, Debug)]
#[serde(default)]
pub struct Fluidics {
    /// Internal diameter of the tubing, millimetres.
    pub bore_id_mm: f32,
    /// What the pump barrel is holding.
    pub pump_fill: Fill,
    pub links: Vec<Link>,
}

impl Default for Fluidics {
    fn default() -> Self {
        Self {
            bore_id_mm: 1.0,
            pump_fill: Fill::Unknown,
            // Starting volumes are estimates, flagged as such until a run
            // measures them. Only the S2→slot figure below comes from the rig.
            links: vec![
                Link::new(Landmark::Pump, Landmark::S1, 150.0),
                // Through the holding coil, which is a spiral built to hold a
                // slug — the largest stretch on the rig by a wide margin.
                Link::new(Landmark::S1, Landmark::S2, 2500.0),
                Link::new(Landmark::S2, Landmark::Slot, 225.0),
                Link::new(Landmark::Wash, Landmark::Pump, 200.0),
                Link::new(Landmark::S2, Landmark::Waste, 250.0),
            ],
        }
    }
}

impl Fluidics {
    pub fn index(&self, from: Landmark, to: Landmark) -> Option<usize> {
        self.links.iter().position(|l| l.from == from && l.to == to)
    }

    pub fn link(&self, from: Landmark, to: Landmark) -> Option<&Link> {
        self.index(from, to).map(|i| &self.links[i])
    }

    /// Records a measured volume between two landmarks, from a calibration run.
    pub fn set_measured(&mut self, from: Landmark, to: Landmark, volume_ul: f32) {
        if let Some(i) = self.index(from, to) {
            let link = &mut self.links[i];
            // Rescale the contents so a re-measurement does not teleport a
            // front: the runs are stored in µL along a link whose length has
            // just changed.
            let scale = if link.volume_ul > 0.0 { volume_ul / link.volume_ul } else { 1.0 };
            for (end, _) in &mut link.runs {
                *end *= scale;
            }
            link.volume_ul = volume_ul;
            link.measured = true;
            link.tidy();
        }
    }

    /// Takes the volumes a prime-and-calibrate run measured.
    ///
    /// The run watches S1 in passing, so the dead volume arrives split: the
    /// barrel to S1, and S1 on to S2. Without the S1 crossing there is only
    /// the lump, and attributing it between two links would be invention — so
    /// in that case neither is marked measured.
    pub fn record_prime(&mut self, to_first_ul: Option<f32>, to_edge_ul: f32, edge_to_slot_ul: f32) {
        if edge_to_slot_ul > 0.0 {
            self.set_measured(Landmark::S2, Landmark::Slot, edge_to_slot_ul);
        }
        match to_first_ul {
            Some(first) if first > 0.0 && to_edge_ul > first => {
                self.set_measured(Landmark::Pump, Landmark::S1, first);
                self.set_measured(Landmark::S1, Landmark::S2, to_edge_ul - first);
            }
            // S1 already wet when the push began means the front was sitting
            // on it, so the whole of this leg is the S1→S2 stretch. The
            // barrel-to-S1 volume was not crossed and stays an estimate.
            Some(_) if to_edge_ul > 0.0 => {
                self.set_measured(Landmark::S1, Landmark::S2, to_edge_ul);
            }
            _ => {}
        }
    }

    /// Everything known about a link, for display.
    pub fn describe(&self, from: Landmark, to: Landmark) -> String {
        match self.link(from, to) {
            None => format!("{} → {}: not modelled", from.label(), to.label()),
            Some(l) => format!(
                "{} → {}: {:.0} µL ≈ {:.0} mm{}",
                from.label(),
                to.label(),
                l.volume_ul,
                l.length_mm(self.bore_id_mm),
                if l.measured { "" } else { " (estimated)" },
            ),
        }
    }

    /// Moves the contents of `path` by `ul`.
    ///
    /// Positive pushes away from the pump, inserting whatever the barrel holds;
    /// negative pulls back, drawing `source` in at the far end. Plug flow: no
    /// mixing, no dispersion. That is a simplification, but the thing it is
    /// used for — "has the front reached the sensor yet" — is exactly what
    /// plug flow gets right.
    pub fn advance(&mut self, path: &[usize], ul: f32, source: Fill) {
        if path.is_empty() || ul.abs() < 1e-4 {
            return;
        }
        // Flatten the path into one axis, move along it, write it back.
        let mut axis: Vec<(f32, Fill)> = Vec::new();
        let mut base = 0.0;
        for &i in path {
            for (end, fill) in &self.links[i].runs {
                axis.push((base + end, *fill));
            }
            base += self.links[i].volume_ul;
        }
        let total = base;
        let moved = if ul > 0.0 {
            let mut out = vec![(ul.min(total), self.pump_fill)];
            for (end, fill) in &axis {
                let shifted = end + ul;
                if shifted > ul {
                    out.push((shifted.min(total), *fill));
                }
                if shifted >= total {
                    break;
                }
            }
            out
        } else {
            let pull = (-ul).min(total);
            let mut out: Vec<(f32, Fill)> = Vec::new();
            for (end, fill) in &axis {
                let shifted = end - pull;
                if shifted > 0.0 {
                    out.push((shifted.min(total), *fill));
                }
            }
            // What came in behind it, from whatever the far end is connected to.
            out.push((total, source));
            out
        };
        // The barrel now holds whatever passed into it.
        if ul < 0.0 {
            self.pump_fill = axis.first().map(|(_, f)| *f).unwrap_or(Fill::Unknown);
        }

        let mut cursor = 0.0;
        let mut at = 0;
        for &i in path {
            let link_end = cursor + self.links[i].volume_ul;
            let mut runs: Vec<(f32, Fill)> = Vec::new();
            while at < moved.len() {
                let (end, fill) = moved[at];
                let clipped = end.min(link_end);
                if clipped > cursor {
                    runs.push((clipped - cursor, fill));
                }
                if end >= link_end {
                    break;
                }
                at += 1;
            }
            if runs.is_empty() {
                runs.push((self.links[i].volume_ul, Fill::Unknown));
            }
            self.links[i].runs = runs;
            self.links[i].tidy();
            cursor = link_end;
        }
    }

    /// A sensor has spoken: the boundary it sits on is known exactly.
    ///
    /// This is what keeps the model from drifting. Counting pump steps alone
    /// accumulates error from backlash, compliance and a bore that is only
    /// nominally 1 mm; a sensor transition is ground truth at one point, so
    /// everything upstream of it is re-stated to agree.
    pub fn sensor_says(&mut self, at: Landmark, wet: bool) {
        let fill = if wet { Fill::Liquid } else { Fill::Air };
        // The end of every link arriving at this landmark, and the start of
        // every link leaving it, is now known.
        for link in &mut self.links {
            if link.to == at {
                let tail = link.volume_ul * 0.98;
                link.runs.retain(|(end, _)| *end < tail);
                link.runs.push((link.volume_ul, fill));
                link.tidy();
            }
            if link.from == at {
                let head = link.volume_ul * 0.02;
                let mut runs = vec![(head, fill)];
                runs.extend(link.runs.iter().copied().filter(|(end, _)| *end > head));
                link.runs = runs;
                link.tidy();
            }
        }
    }

    /// Marks everything unknown again — after a power cycle, or a manual move
    /// the model did not see.
    pub fn forget(&mut self) {
        self.pump_fill = Fill::Unknown;
        for link in &mut self.links {
            link.runs = vec![(link.volume_ul, Fill::Unknown)];
        }
    }

    /// How much of the modelled tubing is in each state, in µL.
    pub fn totals(&self) -> (f32, f32, f32) {
        let (mut liquid, mut air, mut unknown) = (0.0, 0.0, 0.0);
        for link in &self.links {
            let mut prev = 0.0;
            for (end, fill) in &link.runs {
                let span = (end - prev).max(0.0);
                match fill {
                    Fill::Liquid => liquid += span,
                    Fill::Air => air += span,
                    Fill::Unknown => unknown += span,
                }
                prev = *end;
            }
        }
        (liquid, air, unknown)
    }

    pub fn load() -> Self {
        std::fs::read_to_string(data_file(FLUIDICS_FILE))
            .ok()
            .and_then(|t| serde_json::from_str(&t).ok())
            .unwrap_or_default()
    }

    pub fn save(&self) -> Result<(), String> {
        let text = serde_json::to_string_pretty(self).map_err(|e| e.to_string())?;
        std::fs::write(data_file(FLUIDICS_FILE), text).map_err(|e| e.to_string())
    }
}

const FLUIDICS_FILE: &str = "tstand_fluidics.json";

/// Keeps [`Fluidics`] up to date from what the rig reports.
///
/// Two inputs: how far the piston moved since the last look, and what the
/// sensors say. The first is dead reckoning and drifts; the second is ground
/// truth at three points and pulls it straight. Neither is enough alone.
#[derive(Debug)]
pub struct Tracker {
    pub model: Fluidics,
    last_pump: Option<u16>,
    last_path: Option<PathKey>,
    last_sensors: [Option<bool>; 6],
    /// SV02's air port, if the rig has one: the only port known to deliver air.
    air_port: Option<u16>,
    dirty: bool,
    last_save: std::time::Instant,
}

/// What the liquid path was when the piston last moved. A change means the
/// valves moved, and dead reckoning across that is meaningless.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct PathKey {
    solenoid_input: bool,
    sv01_port: u16,
    sv02_port: u16,
}

impl Default for Tracker {
    fn default() -> Self {
        Self::new(Fluidics::load())
    }
}

impl Tracker {
    pub fn new(model: Fluidics) -> Self {
        Self {
            model,
            last_pump: None,
            last_path: None,
            last_sensors: [None; 6],
            air_port: None,
            dirty: false,
            last_save: std::time::Instant::now(),
        }
    }

    /// The links the pump pushes through, outward from the barrel.
    ///
    /// Only the output side is modelled as a path: drawing from SV01 fills the
    /// barrel rather than moving a front through tubing anyone can see.
    fn path(&self, key: PathKey, waste_port: u16, fill_port: u16) -> Vec<usize> {
        if key.solenoid_input {
            return Vec::new();
        }
        let tail = if key.sv02_port == fill_port {
            Some((Landmark::S2, Landmark::Slot))
        } else if key.sv02_port == waste_port {
            Some((Landmark::S2, Landmark::Waste))
        } else {
            None
        };
        let mut path: Vec<usize> = [(Landmark::Pump, Landmark::S1), (Landmark::S1, Landmark::S2)]
            .iter()
            .filter_map(|(a, b)| self.model.index(*a, *b))
            .collect();
        if let Some((a, b)) = tail
            && let Some(i) = self.model.index(a, b)
        {
            path.push(i);
        }
        path
    }

    /// Folds one observation in. `sensors` is indexed by board channel.
    #[allow(clippy::too_many_arguments)]
    pub fn update(
        &mut self,
        pump_steps: Option<u16>,
        ul_per_step: f32,
        solenoid_input: Option<bool>,
        sv01_port: Option<u16>,
        sv02_port: Option<u16>,
        waste_port: u16,
        fill_port: u16,
        air_port: Option<u16>,
        sensors: &[Option<bool>; 6],
        channels: &[(Landmark, u8)],
    ) {
        self.air_port = air_port;
        // Sensors first: ground truth should win over the step count in the
        // same frame, not a frame later.
        for (landmark, channel) in channels {
            let now = sensors.get(*channel as usize).copied().flatten();
            let before = self.last_sensors.get(*channel as usize).copied().flatten();
            if now != before {
                if let Some(wet) = now {
                    self.model.sensor_says(*landmark, wet);
                    self.dirty = true;
                }
                if let Some(slot) = self.last_sensors.get_mut(*channel as usize) {
                    *slot = now;
                }
            }
        }

        let key = match (solenoid_input, sv01_port, sv02_port) {
            (Some(solenoid_input), Some(sv01_port), Some(sv02_port)) => {
                Some(PathKey { solenoid_input, sv01_port, sv02_port })
            }
            _ => None,
        };
        let moved = match (self.last_pump, pump_steps) {
            (Some(was), Some(now)) => Some(now as i32 - was as i32),
            _ => None,
        };
        self.last_pump = pump_steps;

        // A path change between two readings means the valves moved while the
        // piston did; there is no telling where that volume went, so it is not
        // guessed at.
        let same_path = key.is_some() && key == self.last_path;
        self.last_path = key;
        let (Some(key), Some(moved)) = (key, moved) else { return };
        if !same_path || moved == 0 {
            return;
        }

        // Aspirating raises the step count and pulls back; dispensing lowers it
        // and pushes out.
        let pushed_ul = -(moved as f32) * ul_per_step;
        let path = self.path(key, waste_port, fill_port);
        if path.is_empty() {
            // Drawing through SV01: the barrel takes on whatever the wash line
            // holds, and nothing observable moves downstream.
            if moved > 0 {
                self.model.pump_fill = Fill::Liquid;
                self.dirty = true;
            }
            return;
        }
        // What is drawn in behind a withdrawal depends on what the valve is
        // open to. Only the air port is known to deliver air; waste could hand
        // back anything that has collected there, and any other port is a line
        // this model does not follow — both are honestly unknown.
        let source = if Some(key.sv02_port) == self.air_port { Fill::Air } else { Fill::Unknown };
        self.model.advance(&path, pushed_ul, source);
        self.dirty = true;
    }

    /// Writes the model out now and then, so a restart does not start blind.
    pub fn maybe_save(&mut self) {
        if self.dirty && self.last_save.elapsed() > std::time::Duration::from_secs(5) {
            let _ = self.model.save();
            self.dirty = false;
            self.last_save = std::time::Instant::now();
        }
    }

    pub fn flush(&mut self) {
        if self.dirty {
            let _ = self.model.save();
            self.dirty = false;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The figure the rig actually produced: 225 µL from S2 to the slot
    /// sensor, in 1 mm bore.
    #[test]
    fn volume_converts_to_the_length_that_holds_it() {
        // 1 mm bore holds 0.7854 µL per mm, so 225 µL is about 286 mm.
        let mm = length_mm(225.0, 1.0);
        assert!((mm - 286.5).abs() < 1.0, "{mm} mm");
        // And back again.
        assert!((volume_ul(mm, 1.0) - 225.0).abs() < 0.1);
        // A µL per mm is the bore area, by definition of a µL being a mm³.
        // For a 1 mm bore that is π/4 — spelled out rather than written as
        // 0.7854, which clippy rightly reads as a constant in disguise.
        assert!((bore_area_mm2(1.0) - std::f32::consts::FRAC_PI_4).abs() < 1e-6);
    }

    #[test]
    fn a_wider_bore_holds_more_in_less_length() {
        assert!(length_mm(225.0, 2.0) < length_mm(225.0, 1.0));
        // Four times the area for twice the bore.
        assert!((bore_area_mm2(2.0) / bore_area_mm2(1.0) - 4.0).abs() < 1e-3);
    }

    fn primed() -> (Fluidics, Vec<usize>) {
        let mut f = Fluidics::default();
        let path = vec![
            f.index(Landmark::Pump, Landmark::S1).unwrap(),
            f.index(Landmark::S1, Landmark::S2).unwrap(),
            f.index(Landmark::S2, Landmark::Slot).unwrap(),
        ];
        f.pump_fill = Fill::Liquid;
        (f, path)
    }

    /// Nothing is assumed empty. A rig that was just switched on knows
    /// nothing, and saying "air" would be a claim it cannot support.
    #[test]
    fn everything_starts_unknown() {
        let f = Fluidics::default();
        let (liquid, air, unknown) = f.totals();
        assert_eq!(liquid, 0.0);
        assert_eq!(air, 0.0);
        assert!(unknown > 0.0);
        assert_eq!(f.pump_fill, Fill::Unknown);
        assert_eq!(f.link(Landmark::S2, Landmark::Slot).unwrap().fill_at(10.0), Fill::Unknown);
    }

    #[test]
    fn pushing_moves_a_front_along_the_path() {
        let (mut f, path) = primed();
        // Push 100 µL: the first link is 150 µL, so the front lands inside it.
        f.advance(&path, 100.0, Fill::Air);
        let first = f.link(Landmark::Pump, Landmark::S1).unwrap();
        assert_eq!(first.fill_at(50.0), Fill::Liquid, "liquid behind the front");
        assert_eq!(first.fill_at(140.0), Fill::Unknown, "and whatever was there ahead of it");
        assert!((first.front_ul().expect("a front") - 100.0).abs() < 1.0);

        // Push another 100: the front crosses into the second link.
        f.advance(&path, 100.0, Fill::Air);
        assert_eq!(f.link(Landmark::Pump, Landmark::S1).unwrap().fill_at(140.0), Fill::Liquid);
        let second = f.link(Landmark::S1, Landmark::S2).unwrap();
        assert!((second.front_ul().expect("a front in the second link") - 50.0).abs() < 1.0);
    }

    #[test]
    fn pulling_draws_the_far_end_back_in() {
        let (mut f, path) = primed();
        f.advance(&path, 300.0, Fill::Air);
        // Now pull back 100 µL with air behind it.
        f.advance(&path, -100.0, Fill::Air);
        let last = f.link(Landmark::S2, Landmark::Slot).unwrap();
        assert_eq!(last.fill_at(last.volume_ul - 1.0), Fill::Air, "air came in at the far end");
        // And the barrel took back what was nearest it.
        assert_eq!(f.pump_fill, Fill::Liquid);
    }

    /// A sensor is ground truth; the model has to agree with it even when the
    /// step count says otherwise.
    #[test]
    fn a_sensor_overrides_what_the_step_count_believed() {
        let (mut f, path) = primed();
        f.advance(&path, 50.0, Fill::Air); // front nowhere near S2
        assert_ne!(f.link(Landmark::S1, Landmark::S2).unwrap().fill_at(799.0), Fill::Liquid);

        f.sensor_says(Landmark::S2, true);
        // The end of the link arriving at S2, and the start of the one leaving
        // it, both now read wet.
        let arriving = f.link(Landmark::S1, Landmark::S2).unwrap();
        assert_eq!(arriving.fill_at(arriving.volume_ul - 1.0), Fill::Liquid);
        let leaving = f.link(Landmark::S2, Landmark::Slot).unwrap();
        assert_eq!(leaving.fill_at(0.5), Fill::Liquid);

        // Dry says the opposite, and is equally believed.
        f.sensor_says(Landmark::S2, false);
        let leaving = f.link(Landmark::S2, Landmark::Slot).unwrap();
        assert_eq!(leaving.fill_at(0.5), Fill::Air);
    }

    #[test]
    fn a_measured_link_replaces_the_estimate_without_moving_the_front() {
        let (mut f, path) = primed();
        f.advance(&path, 200.0, Fill::Air);
        let before = f.link(Landmark::S1, Landmark::S2).unwrap();
        let fraction = before.front_ul().expect("a front") / before.volume_ul;

        f.set_measured(Landmark::S1, Landmark::S2, 400.0);
        let after = f.link(Landmark::S1, Landmark::S2).unwrap();
        assert!(after.measured);
        assert_eq!(after.volume_ul, 400.0);
        // The front is still the same way along the link, not teleported.
        let now = after.front_ul().expect("still a front") / after.volume_ul;
        assert!((now - fraction).abs() < 0.01, "{fraction} -> {now}");
    }

    /// A run that saw S1 splits the dead volume across the two links it
    /// actually spans.
    #[test]
    fn a_run_records_the_legs_it_measured() {
        let mut f = Fluidics::default();
        f.record_prime(Some(120.0), 500.0, 225.0);
        assert_eq!(f.link(Landmark::Pump, Landmark::S1).unwrap().volume_ul, 120.0);
        assert_eq!(f.link(Landmark::S1, Landmark::S2).unwrap().volume_ul, 380.0);
        assert_eq!(f.link(Landmark::S2, Landmark::Slot).unwrap().volume_ul, 225.0);
        assert!(f.links.iter().filter(|l| l.measured).count() == 3);
    }

    /// A front that starts on S1 measures the coil exactly, and says nothing
    /// about the barrel-to-S1 stretch it never crossed.
    #[test]
    fn a_front_starting_on_the_first_sensor_measures_the_coil_alone() {
        let mut f = Fluidics::default();
        f.record_prime(Some(0.0), 1750.0, 208.0);
        let coil = f.link(Landmark::S1, Landmark::S2).unwrap();
        assert_eq!(coil.volume_ul, 1750.0);
        assert!(coil.measured);
        // 1750 µL of 1 mm bore is about 2.2 m — a holding coil.
        assert!((coil.length_mm(1.0) - 2228.0).abs() < 5.0, "{}", coil.length_mm(1.0));
        assert!(!f.link(Landmark::Pump, Landmark::S1).unwrap().measured, "that stretch was never crossed");
    }

    /// Without the S1 crossing there is one lump and no honest way to split
    /// it, so those two links keep their estimates and keep saying so.
    #[test]
    fn a_run_that_missed_the_first_sensor_does_not_invent_a_split() {
        let mut f = Fluidics::default();
        f.record_prime(None, 500.0, 225.0);
        assert!(f.link(Landmark::S2, Landmark::Slot).unwrap().measured, "the slot leg stands alone");
        assert!(!f.link(Landmark::Pump, Landmark::S1).unwrap().measured);
        assert!(!f.link(Landmark::S1, Landmark::S2).unwrap().measured);
    }

    #[test]
    fn the_measured_flag_shows_in_the_description() {
        let mut f = Fluidics::default();
        assert!(f.describe(Landmark::S2, Landmark::Slot).contains("estimated"));
        f.set_measured(Landmark::S2, Landmark::Slot, 225.0);
        let text = f.describe(Landmark::S2, Landmark::Slot);
        assert!(!text.contains("estimated"), "{text}");
        assert!(text.contains("225"), "{text}");
        assert!(text.contains("286"), "the length follows from the bore: {text}");
    }

    /// Fronts moving back and forth must not make the run list grow forever.
    #[test]
    fn the_run_list_stays_bounded() {
        let (mut f, path) = primed();
        for _ in 0..200 {
            f.advance(&path, 30.0, Fill::Air);
            f.advance(&path, -30.0, Fill::Air);
        }
        for link in &f.links {
            assert!(link.runs.len() <= 8, "{:?} has {} runs", link.from, link.runs.len());
        }
    }

    #[test]
    fn forgetting_puts_everything_back_to_unknown() {
        let (mut f, path) = primed();
        f.advance(&path, 300.0, Fill::Air);
        f.forget();
        let (liquid, air, unknown) = f.totals();
        assert_eq!((liquid, air), (0.0, 0.0));
        assert!(unknown > 0.0);
    }

    #[test]
    fn totals_add_up_to_the_tubing_modelled() {
        let (mut f, path) = primed();
        f.advance(&path, 400.0, Fill::Air);
        let (liquid, air, unknown) = f.totals();
        let modelled: f32 = f.links.iter().map(|l| l.volume_ul).sum();
        assert!((liquid + air + unknown - modelled).abs() < 1.0);
        assert!(liquid > 0.0, "some of it is liquid by now");
    }
}

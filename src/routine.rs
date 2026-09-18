//! Prime the wash line and measure what it holds.
//!
//! Draw wash into PP01, push it up to the in-line sensor at S2, then on into
//! the slot, and record how much it took to reach each. Those two figures are
//! the line's dead volume — the thing you have to know before any dispensed
//! volume downstream means anything.
//!
//! # Why this is a state machine and not a script
//!
//! The existing demo sequence is a list of commands played back one at a time:
//! fire, wait for the device to go idle, fire the next. That cannot express
//! "keep going until the sensor wets", which is the whole point here. This
//! decides the next command from what the sensors and the pump are doing, so
//! it is a loop, not a list.
//!
//! It is also pure: [`Routine::step`] takes an [`Observation`] and returns an
//! [`Action`], and touches no hardware. The alternative — driving the pump
//! from inside the logic — would leave the approach and back-off arithmetic
//! testable only against a live rig with real liquid in it, which is no way
//! to find an off-by-one.
//!
//! # Why it moves in chunks
//!
//! Every advance is a bounded `Dispense`, never a free run. If this program
//! dies, the network drops, or the operator walks away mid-run, the pump
//! finishes the chunk it was given and stops. There is no command that would
//! leave it moving.
//!
//! # Finding the edge
//!
//! A coarse chunk overshoots the sensor by up to its own length, so the
//! reading would be "somewhere in the last 83 µL". Instead the run approaches
//! fast, backs off until the sensor is dry again, and creeps up in small
//! chunks at low speed. The recorded figure is the fine approach's, so the
//! precision is the fine chunk, not the coarse one.

use serde::{Deserialize, Serialize};

use crate::bus::Op;
use crate::devices::DeviceId;
use crate::rig::{Rig, Role};

/// PP01 rejects moves past this; the datasheet limit is 3820 steps.
const MAX_BARREL_STEPS: u16 = 3810;

/// What the run is allowed to do, in units the pump understands.
#[derive(Serialize, Deserialize, Clone, Copy, Debug, PartialEq)]
pub struct Plan {
    /// SV01 port carrying wash.
    pub wash_port: u16,
    /// SV02 port feeding the slot.
    pub fill_port: u16,
    /// SV02 port to waste. The pump is homed into this, never into a closed
    /// valve or a capped line.
    pub waste_port: u16,
    /// SV02 port open to air. Pulling back through this draws clean air in
    /// behind the front, which is how the line is cleared before measuring.
    pub air_port: u16,
    /// Channel of the first in-line sensor, passed on the way to S2. Watching
    /// it costs nothing and splits the dead volume into two measured links
    /// instead of one lump.
    pub first_sensor: u8,
    /// Channel of the in-line sensor at S2 to stop on.
    pub edge_sensor: u8,
    /// Channel of the slot's own sensor.
    pub slot_sensor: u8,
    /// How much wash to take in before pushing. Must cover both legs' caps
    /// with margin, or the run stops on an empty barrel part-way.
    pub draw_steps: u16,
    /// Coarse approach: big chunks, at the speed controller_v2 moves liquid.
    pub coarse_steps: u16,
    pub coarse_speed: u16,
    /// Fine approach: small chunks, slower still. This sets the precision of
    /// the recorded figure.
    pub fine_steps: u16,
    pub fine_speed: u16,
    /// Give up on the run to S2 after this much travel with no sensor change.
    /// Overshoot here goes to waste, so this can be generous.
    pub cap_steps: u16,
    /// The same for the last run into the slot, which is not generous at all:
    /// past the valve there is nowhere for surplus to go but the slot, and the
    /// slot holds 300 µL.
    pub slot_cap_steps: u16,
    /// How far to keep pulling once the line reads clear, so the front sits
    /// behind S1 rather than balanced on it.
    pub clear_margin_steps: u16,
    /// Give up clearing after this much travel.
    pub clear_cap_steps: u16,
    /// How long a reading must hold still before it is believed, in seconds.
    ///
    /// This is a duration and not a count of samples on purpose. Liquid
    /// standing at a detector reads the same for as long as you care to look;
    /// a bubble, or air with a film on the wall, flickers. So the question
    /// "is this liquid" is really "has this reading held for long enough",
    /// and counting loop iterations answers a different question — at a 50 ms
    /// poll it confirms three times inside one 1 Hz sensor update, which is
    /// no confirmation at all.
    ///
    /// Two seconds matches controller_v2's own `MIN_STREAM_TIME_MS`.
    pub confirm_secs: f64,
    /// Sensor changes within one clearing pass after which the line is taken
    /// to hold no liquid column, however it reads at the moment of asking.
    pub flicker_is_clear: u16,
    /// How much further to push after S2 first reads wet, checking it stays
    /// wet the whole way.
    ///
    /// First contact is the leading edge, and a leading edge is froth: a
    /// meniscus arrives, wets the detector, and reads stable for a moment
    /// while the liquid column behind it is still some way back. Recording
    /// there gives a figure that is short, and short by however ragged the
    /// front was that day. Pushing on until the reading survives real travel
    /// measures the column instead.
    pub verify_steps: u16,
}

impl Default for Plan {
    fn default() -> Self {
        Self {
            wash_port: 1,
            fill_port: 10,
            waste_port: 9,
            air_port: 16,
            first_sensor: 4,
            edge_sensor: 5,
            slot_sensor: 2,
            // The holding coil is a spiral that exists to hold a slug, so the
            // dead volume from the barrel to S2 is millilitres, not hundreds
            // of microlitres. Measured against this rig: 1 mL did not reach
            // S2, while homing 2092 steps (4.4 mL) pushed past it — and
            // controller_v2 routinely drives this pump to 3600 steps while
            // watching the same sensor. 3000 steps is 6.2 mL, enough to cover
            // that and still leave the slot leg something to push with.
            draw_steps: 2400,
            coarse_steps: 40,
            // Speeds taken from controller_v2's own wash step, not guessed.
            // Its `first_wash_step` pushes at 8 rpm while watching the slot
            // sensor and `retrieve_washing_liquid` pulls back at 14; across
            // `transport_control.rs` every sensor-guided move of the main pump
            // sits between 5 and 25 rpm, and the 270–300 rpm speeds are only
            // used for repositioning with no liquid front to spoil. Pushing a
            // front down a 1.2 mm bore at 200 rpm is a different experiment
            // from the one these sensors were tuned for.
            coarse_speed: 25,
            fine_steps: 4,
            fine_speed: 8,
            // ~5 mL. Generous because every drop of it goes to waste: this
            // leg runs with SV02 on the waste port precisely so the cap can be
            // set by how long the line might be rather than by what an
            // overshoot would ruin.
            cap_steps: 2400,
            // ~500 µL: the slot's own 300 µL plus the line from the valve.
            // A faulty slot sensor spills into the slot, not into waste, so
            // this is the one cap that cannot be set by how long a line might
            // plausibly be.
            slot_cap_steps: 240,
            flicker_is_clear: 12,
            verify_steps: 120,
            clear_margin_steps: 120,
            clear_cap_steps: 1200,
            confirm_secs: 2.0,
        }
    }
}

impl Plan {
    /// Reads the ports and sensor channels off the rig rather than assuming
    /// them, so a re-plumbed rig does not need this edited too.
    pub fn for_rig(rig: &Rig) -> Result<Self, String> {
        let port = |role: Role, what: &str| {
            rig.ports
                .iter()
                .find(|p| p.role == role)
                .map(|p| (p.valve, p.port))
                .ok_or_else(|| format!("this rig has no {what} port configured (Settings → Rig)"))
        };
        let (wash_valve, wash_port) = port(Role::Wash, "wash")?;
        if wash_valve != DeviceId::Sv01 {
            return Err(format!("wash is on {}, but PP01 draws through SV01", wash_valve.tag()));
        }
        let (waste_valve, waste_port) = port(Role::Waste, "waste")?;
        if waste_valve != DeviceId::Sv02 {
            return Err(format!("waste is on {}, but PP01 empties through SV02", waste_valve.tag()));
        }
        let (air_valve, air_port) = port(Role::Air, "air")?;
        if air_valve != DeviceId::Sv02 {
            return Err(format!("air is on {}, but the line is cleared through SV02", air_valve.tag()));
        }
        let (fill_valve, fill_port) = port(Role::SlotFill, "slot fill")?;
        if fill_valve != DeviceId::Sv02 {
            return Err(format!("the slot is fed from {}, but PP01 pushes through SV02", fill_valve.tag()));
        }
        let sensor = |at, what: &str| {
            rig.sensor_at(at).map(|s| s.channel).ok_or_else(|| format!("this rig has no {what} sensor configured"))
        };
        Ok(Self {
            wash_port,
            fill_port,
            waste_port,
            air_port,
            first_sensor: sensor(crate::rig::SensorAt::Inline1, "S1 in-line")?,
            edge_sensor: sensor(crate::rig::SensorAt::Inline2, "S2 in-line")?,
            slot_sensor: sensor(crate::rig::SensorAt::Slot, "slot")?,
            ..Self::default()
        })
    }
}

/// What the rig looks like right now, as far as the run needs to care.
#[derive(Clone, Copy, Debug)]
pub struct Observation {
    /// PP01's piston position in steps, if known.
    pub pump: Option<u16>,
    /// Every device the run drives has finished moving.
    pub settled: bool,
    /// Seconds since the run began, for judging how long a reading has held.
    pub now: f64,
    /// Liquid at the sensor the current leg is chasing.
    pub edge_wet: Option<bool>,
    pub slot_wet: Option<bool>,
    /// The first in-line sensor, watched in passing.
    pub first_wet: Option<bool>,
}

/// What the driver should do next.
#[derive(Clone, Debug, PartialEq)]
pub enum Action {
    /// Nothing to send; the rig is still catching up.
    Wait,
    Send(DeviceId, Op),
    Finished(Report),
    Abort(String),
}

/// What the run measured.
#[derive(Serialize, Deserialize, Clone, Copy, Debug, PartialEq, Eq, Default)]
pub struct Report {
    /// Steps dispensed from the start of the push until S1 wetted, if it was
    /// seen to change during the run.
    pub to_first_steps: Option<u16>,
    /// Steps dispensed from the start of the push until S2 wetted.
    pub to_edge_steps: u16,
    /// Steps from there until the slot's sensor wetted.
    pub edge_to_slot_steps: u16,
    /// The edge was found by the fine approach rather than the coarse one.
    pub edge_precise: bool,
    /// The slot leg never wetted; `edge_to_slot_steps` is how far it got.
    pub slot_timed_out: bool,
    /// How far past first contact S2 held wet before the edge was recorded.
    /// Zero means the edge came from the coarse pass with no verification.
    pub verified_steps: u16,
    /// The line was taken as clear because its detectors would not hold still,
    /// rather than because they read dry.
    pub cleared_through_froth: bool,
    /// How often a sensor contradicted a reading it had just given.
    ///
    /// Every stop waits for several agreeing samples with the piston still.
    /// A stationary sensor changing its mind is something physically moving
    /// past it — a bubble, or a meniscus that has not settled. One or two is
    /// ordinary; a run full of them means the numbers below describe a frothy
    /// line rather than a clean front.
    pub bubbles_seen: u16,
}

impl Report {
    pub fn to_edge_ul(self, ul_per_step: f32) -> f32 {
        self.to_edge_steps as f32 * ul_per_step
    }

    pub fn edge_to_slot_ul(self, ul_per_step: f32) -> f32 {
        self.edge_to_slot_steps as f32 * ul_per_step
    }
}

/// Where the run has got to. Public so the interface can say so.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Stage {
    Homing,
    Drawing,
    Coarse,
    BackingOff,
    Clearing,
    SelectWaste,
    Fine,
    Verify,
    SelectSlot,
    ToSlot,
    Done,
}

impl Stage {
    pub fn label(self) -> &'static str {
        match self {
            Stage::Homing => "homing",
            Stage::Drawing => "drawing wash",
            Stage::Coarse => "approaching S2",
            Stage::BackingOff => "backing off",
            Stage::Clearing => "clearing S2 with air",
            Stage::SelectWaste => "selecting waste",
            Stage::Fine => "creeping to S2",
            Stage::Verify => "holding S2 while pushing on",
            Stage::SelectSlot => "opening the slot",
            Stage::ToSlot => "pushing to the slot",
            Stage::Done => "done",
        }
    }
}

/// One step of setup, in order. Each is sent once and waited on, so the next
/// never overlaps the last.
///
/// The order is controller_v2's, and it matters. Its `initialize` resets the
/// selector valves, waits for them, points the main selector at drain, and
/// only then homes the pump. Homing the pump first — which is what "home all"
/// sounds like it should mean — empties a barrel that may hold millilitres
/// into whatever the valve happens to be on. On this rig that is worse than
/// untidy: an SV-07 closes its common port when it resets, and the valve may
/// be parked on a capped line besides, so the pump would be pushing into a
/// dead end.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Setup {
    HomeSv01,
    HomeSv02,
    HomeSv03,
    SolenoidToWaste,
    WasteValve,
    HomePump,
    AirValve,
    SolenoidToWash,
    WashValve,
    DrawSpeed,
    Draw,
    SolenoidToSlot,
    MeasureFromWaste,
    Done,
}

pub struct Routine {
    plan: Plan,
    stage: Stage,
    setup: Setup,
    /// Position when the push began, to measure travel from.
    push_start: Option<u16>,
    /// Position when S2 first wetted, coarsely.
    edge_coarse: Option<u16>,
    /// Position at the precise edge.
    edge_fine: Option<u16>,
    /// Travel within the current leg, for the cap.
    leg_steps: u32,
    /// A command is out; wait for the rig to settle before deciding again.
    pending: bool,
    /// When the reading this leg is waiting on first took its current value.
    wet_since: Option<f64>,
    /// Waste has been selected after clearing.
    waste_selected: bool,
    /// Piston position when waste was selected — the zero of the measurement.
    pump_at_waste: Option<u16>,
    /// Position when S1 was first seen to wet during the push.
    first_seen: Option<u16>,
    /// How far the verification push has travelled.
    verify_pushed: u32,
    /// Whether a confirmation was part-way through when a reading flipped.
    was_confirming: bool,
    report: Report,
}

impl Routine {
    pub fn new(plan: Plan) -> Self {
        Self {
            plan,
            stage: Stage::Homing,
            setup: Setup::HomeSv01,
            push_start: None,
            edge_coarse: None,
            edge_fine: None,
            leg_steps: 0,
            pending: false,
            wet_since: None,
            waste_selected: false,
            pump_at_waste: None,
            first_seen: None,
            verify_pushed: 0,
            was_confirming: false,
            report: Report::default(),
        }
    }

    pub fn stage(&self) -> Stage {
        self.stage
    }

    /// Piston position the measuring push started from.
    #[cfg(test)]
    fn push_start(&self) -> Option<u16> {
        self.push_start
    }

    pub fn plan(&self) -> Plan {
        self.plan
    }

    /// How far through the current leg the pump is, 0..=1, for a progress bar.
    pub fn leg_fraction(&self) -> f32 {
        (self.leg_steps as f32 / self.plan.cap_steps.max(1) as f32).clamp(0.0, 1.0)
    }

    /// Decides the next command. Call it whenever the rig's state changes.
    pub fn step(&mut self, obs: Observation) -> Action {
        // One command in flight at a time: the pump's position is only
        // meaningful between moves, and every decision below reads it.
        if !obs.settled {
            self.pending = false;
            return Action::Wait;
        }
        if self.pending {
            // Settled with a command outstanding: it has landed.
            self.pending = false;
        }
        match self.stage {
            Stage::Homing | Stage::Drawing => self.run_setup(obs),
            Stage::Coarse => self.approach(obs, true),
            Stage::BackingOff => self.back_off(obs),
            Stage::Clearing => self.clear_edge(obs),
            Stage::SelectWaste => self.select_waste(),
            Stage::Fine => self.approach(obs, false),
            Stage::Verify => self.verify(obs),
            Stage::SelectSlot => self.select_slot(),
            Stage::ToSlot => self.push_to_slot(obs),
            Stage::Done => Action::Finished(self.report),
        }
    }

    fn send(&mut self, id: DeviceId, op: Op) -> Action {
        self.pending = true;
        Action::Send(id, op)
    }

    /// A reading contradicted itself before it had held long enough.
    ///
    /// With the piston stopped, nothing should be changing at a detector. A
    /// reading that flips is something physically moving past it, which on
    /// this rig means a bubble or a film of air breaking up.
    fn contradiction(&mut self) {
        if self.wet_since.is_some() {
            self.report.bubbles_seen = self.report.bubbles_seen.saturating_add(1);
        }
        self.wet_since = None;
        self.was_confirming = false;
    }

    /// Whether the reading has now held still for long enough to believe.
    fn confirmed(&mut self, now: f64) -> bool {
        let since = *self.wet_since.get_or_insert(now);
        self.was_confirming = true;
        if now - since >= self.plan.confirm_secs {
            self.wet_since = None;
            self.was_confirming = false;
            return true;
        }
        false
    }

    /// The fixed opening sequence, one command per call.
    fn run_setup(&mut self, obs: Observation) -> Action {
        let plan = self.plan;
        let (next, action) = match self.setup {
            Setup::HomeSv01 => (Setup::HomeSv02, self.send(DeviceId::Sv01, Op::Home)),
            Setup::HomeSv02 => (Setup::HomeSv03, self.send(DeviceId::Sv02, Op::Home)),
            Setup::HomeSv03 => (Setup::SolenoidToWaste, self.send(DeviceId::Sv03, Op::Home)),
            // Face the pump at SV02 and open a path to waste before the barrel
            // is emptied, so whatever it is holding has somewhere to go.
            Setup::SolenoidToWaste => {
                (Setup::WasteValve, self.send(DeviceId::Pp01, Op::SolenoidInput(false)))
            }
            Setup::WasteValve => (Setup::HomePump, self.send(DeviceId::Sv02, Op::ValveTo(plan.waste_port))),
            Setup::HomePump => (Setup::AirValve, self.send(DeviceId::Pp01, Op::Home)),
            // Open air, then blow the output line clear before any wash is
            // drawn. Clearing afterwards would pull the line's contents into
            // a barrel already full, and would leave the front wherever it
            // stopped rather than back at the barrel.
            Setup::AirValve => {
                self.stage = Stage::Clearing;
                (Setup::SolenoidToWash, self.send(DeviceId::Sv02, Op::ValveTo(plan.air_port)))
            }
            Setup::SolenoidToWash => {
                self.stage = Stage::Drawing;
                (Setup::WashValve, self.send(DeviceId::Pp01, Op::SolenoidInput(true)))
            }
            Setup::WashValve => (Setup::DrawSpeed, self.send(DeviceId::Sv01, Op::ValveTo(plan.wash_port))),
            Setup::DrawSpeed => (Setup::Draw, self.send(DeviceId::Pp01, Op::SetSpeed(plan.coarse_speed))),
            Setup::Draw => (Setup::SolenoidToSlot, self.send(DeviceId::Pp01, Op::Aspirate(plan.draw_steps))),
            Setup::SolenoidToSlot => {
                // Check the barrel actually took the wash before pushing: a
                // dry supply looks exactly like a successful draw until the
                // line fails to wet, several hundred steps later.
                let drawn = obs.pump.unwrap_or(0);
                if drawn + 1 < plan.draw_steps {
                    return Action::Abort(format!(
                        "PP01 holds {drawn} steps after asking for {}; check the wash supply",
                        plan.draw_steps
                    ));
                }
                (Setup::MeasureFromWaste, self.send(DeviceId::Pp01, Op::SolenoidInput(false)))
            }
            // Measure against waste: S2 is upstream of the valve, so the
            // overshoot on this leg leaves through whichever port is open.
            Setup::MeasureFromWaste => (Setup::Done, self.send(DeviceId::Sv02, Op::ValveTo(plan.waste_port))),
            Setup::Done => {
                self.push_start = obs.pump;
                self.leg_steps = 0;
                self.wet_since = None;
                self.stage = Stage::Coarse;
                return self.approach(obs, true);
            }
        };
        self.setup = next;
        action
    }

    /// Pull back through the air port until the line holds no liquid column.
    ///
    /// "No liquid" and "reads dry" are not the same thing, and the difference
    /// is what this rig keeps showing. A detector with a liquid column
    /// standing at it reads wet steadily. A detector with a bubble or a
    /// meniscus parked on it flickers — and flicker means there is no column
    /// there, which for clearing is the answer wanted. So a stable dry reading
    /// clears the line, and so does a reading that will not hold still; only a
    /// steady wet one means liquid is still present.
    ///
    /// Without this the measurement is worthless whenever the line is already
    /// primed — which it is after the barrel has been homed into waste, since
    /// that pushes its whole contents past S2. The first run of this routine
    /// on a primed rig reported the dead volume as zero steps for exactly that
    /// reason. controller_v2 does the same thing with
    /// `wait_air_pocket_set_pos` on this very sensor.
    fn clear_edge(&mut self, obs: Observation) -> Action {
        let plan = self.plan;
        // S1 is the most upstream sensor, so clearing back past it puts the
        // front between the barrel and S1 — which is where a measurement of
        // the whole dead volume has to start. Stopping at S2, as this first
        // did, leaves the front a chunk short of S2 and measures that gap
        // instead of the line.
        let dry = obs.first_wet == Some(false) && obs.edge_wet == Some(false);
        // A detector that has changed its mind this many times in one clearing
        // pass is not looking at a liquid column, whatever it happens to read
        // at the instant it is asked.
        let churning = self.report.bubbles_seen >= plan.flicker_is_clear;
        if dry || churning {
            if !churning && !self.confirmed(obs.now) {
                return Action::Wait;
            }
            if churning {
                self.report.cleared_through_froth = true;
            }
            // A margin beyond the sensor, so the front sits behind S1 rather
            // than balanced on it.
            if self.leg_steps < plan.clear_margin_steps as u32 {
                self.leg_steps += plan.coarse_steps as u32;
                return self.send(DeviceId::Pp01, Op::Aspirate(plan.coarse_steps));
            }
            self.wet_since = None;
            self.leg_steps = 0;
            self.stage = Stage::Drawing;
            return self.run_setup(obs);
        }
        self.contradiction();
        if self.leg_steps >= plan.clear_cap_steps as u32 {
            // Say which of the two it is. A steady wet reading after pulling
            // millilitres means nothing is flowing — a capped air port, or a
            // blockage. A flickering one means froth that would not settle.
            return Action::Abort(if self.report.bubbles_seen > 0 {
                format!(
                    "pulled back {} steps and S1/S2 never settled ({} changes); froth or a bubble parked on a detector",
                    self.leg_steps, self.report.bubbles_seen
                )
            } else {
                format!(
                    "pulled back {} steps and S1/S2 read wet the whole time without one change; nothing is flowing — check the air port on SV02 is open and not capped",
                    self.leg_steps
                )
            });
        }
        // Pulling adds to the barrel, so it can run out of room rather than
        // out of liquid.
        if obs.pump.is_some_and(|p| p as u32 + plan.coarse_steps as u32 > MAX_BARREL_STEPS as u32) {
            return Action::Abort("PP01 has no room left to pull back; the line will not clear".into());
        }
        self.leg_steps += plan.coarse_steps as u32;
        self.send(DeviceId::Pp01, Op::Aspirate(plan.coarse_steps))
    }

    /// Point the valve at waste, then start measuring.
    fn select_waste(&mut self) -> Action {
        self.stage = Stage::SelectWaste;
        if self.waste_selected {
            self.stage = Stage::Coarse;
            self.push_start = self.pump_at_waste;
            self.leg_steps = 0;
            return self.send(DeviceId::Pp01, Op::SetSpeed(self.plan.coarse_speed));
        }
        self.waste_selected = true;
        self.send(DeviceId::Sv02, Op::ValveTo(self.plan.waste_port))
    }

    /// Advance towards the edge sensor, coarsely or finely.
    fn approach(&mut self, obs: Observation, coarse: bool) -> Action {
        let plan = self.plan;
        // S1 sits between the pump and the coil, so the front crosses it on
        // the way to S2. Noting where turns one lumped dead volume into two
        // measured links for no extra travel.
        if self.first_seen.is_none() && obs.first_wet == Some(true) {
            self.first_seen = obs.pump;
        }
        if obs.edge_wet == Some(true) {
            // Hold still and look again rather than believe one reading.
            if !self.confirmed(obs.now) {
                return Action::Wait;
            }
            let here = obs.pump;
            if coarse {
                self.edge_coarse = here;
                self.stage = Stage::BackingOff;
                self.leg_steps = 0;
                self.wet_since = None;
                return self.send(DeviceId::Pp01, Op::Aspirate(plan.coarse_steps));
            }
            self.edge_fine = here;
            self.stage = Stage::Verify;
            self.verify_pushed = 0;
            return self.verify(obs);
        }
        // A blip that does not repeat leaves the front where it was, and is
        // worth counting: it is something moving past a stationary sensor.
        self.contradiction();
        if self.leg_steps >= plan.cap_steps as u32 {
            // Coarse never found it: there is nothing to creep up on.
            if coarse {
                return Action::Abort(format!(
                    "pushed {} steps without wetting S2; stopping before the cap is exceeded",
                    self.leg_steps
                ));
            }
            // Fine ran out after a successful coarse hit: fall back on it.
            return self.finish_edge(obs);
        }
        // Only the chunk size varies per advance. The speed is commanded once
        // when the phase changes rather than before every chunk: it is a frame
        // on the RS485 bus like any other, and the pump keeps the last one it
        // was given. `speeds_are_set_before_the_chunks_they_apply_to` is what
        // holds that arrangement together.
        let chunk = if coarse { plan.coarse_steps } else { plan.fine_steps };
        match self.guard_empty(obs, chunk) {
            Some(abort) => abort,
            None => {
                self.leg_steps += chunk as u32;
                self.send(DeviceId::Pp01, Op::Dispense(chunk))
            }
        }
    }

    /// After overshooting, pull back until the sensor reads dry again.
    fn back_off(&mut self, obs: Observation) -> Action {
        let plan = self.plan;
        if obs.edge_wet == Some(false) {
            self.stage = Stage::Fine;
            self.leg_steps = 0;
            self.wet_since = None;
            return self.send(DeviceId::Pp01, Op::SetSpeed(plan.fine_speed));
        }
        // Still wet after a full coarse chunk back: the sensor is not going to
        // clear, so take the coarse figure rather than pull the line apart.
        if self.leg_steps >= plan.coarse_steps as u32 * 2 {
            return self.finish_edge(obs);
        }
        self.leg_steps += plan.coarse_steps as u32;
        self.send(DeviceId::Pp01, Op::Aspirate(plan.coarse_steps))
    }

    /// Push on past first contact, requiring S2 to stay wet throughout.
    ///
    /// A reading that breaks under travel was a meniscus, not a column, so the
    /// approach resumes from where it is. Only a reading that survives the
    /// whole push is taken as the front, and the position recorded is the one
    /// at the end of it — the column, not its leading edge.
    fn verify(&mut self, obs: Observation) -> Action {
        let plan = self.plan;
        if obs.edge_wet != Some(true) {
            // A front that breaks while being pushed is a bubble by any
            // reading of it, whether or not a confirmation happened to be in
            // flight — so it is counted here rather than left to
            // `contradiction`, which only knows about confirmations.
            self.report.bubbles_seen = self.report.bubbles_seen.saturating_add(1);
            self.wet_since = None;
            self.was_confirming = false;
            self.stage = Stage::Fine;
            self.edge_fine = None;
            return self.approach(obs, false);
        }
        if self.verify_pushed >= plan.verify_steps as u32 {
            self.edge_fine = obs.pump;
            self.report.verified_steps = self.verify_pushed as u16;
            return self.finish_edge(obs);
        }
        match self.guard_empty(obs, plan.fine_steps) {
            Some(abort) => abort,
            None => {
                self.verify_pushed += plan.fine_steps as u32;
                self.send(DeviceId::Pp01, Op::Dispense(plan.fine_steps))
            }
        }
    }

    /// Record the edge and start the run to the slot.
    fn finish_edge(&mut self, obs: Observation) -> Action {
        let start = self.push_start.unwrap_or(0);
        let edge = self.edge_fine.or(self.edge_coarse).or(obs.pump).unwrap_or(start);
        self.report.to_edge_steps = start.saturating_sub(edge);
        self.report.to_first_steps = self.first_seen.map(|at| start.saturating_sub(at));
        self.report.edge_precise = self.edge_fine.is_some();
        self.stage = Stage::SelectSlot;
        self.leg_steps = 0;
        self.wet_since = None;
        // Only now does the slot get opened: the line is primed to S2 and
        // everything before this went to waste.
        self.send(DeviceId::Sv02, Op::ValveTo(self.plan.fill_port))
    }

    fn select_slot(&mut self) -> Action {
        self.stage = Stage::ToSlot;
        // The slot leg runs at the creep speed throughout; it is short, and
        // what it is filling is small.
        self.send(DeviceId::Pp01, Op::SetSpeed(self.plan.fine_speed))
    }

    /// Push on until the slot's own sensor sees liquid.
    fn push_to_slot(&mut self, obs: Observation) -> Action {
        let plan = self.plan;
        let edge = self.edge_fine.or(self.edge_coarse).unwrap_or(0);
        if obs.slot_wet == Some(true) {
            if !self.confirmed(obs.now) {
                return Action::Wait;
            }
            self.report.edge_to_slot_steps = edge.saturating_sub(obs.pump.unwrap_or(edge));
            self.stage = Stage::Done;
            return Action::Finished(self.report);
        }
        self.contradiction();
        if self.leg_steps >= plan.slot_cap_steps as u32 {
            // Say so rather than pretend: an unwetted slot sensor is a result,
            // and the figure so far is still worth reporting.
            self.report.edge_to_slot_steps = edge.saturating_sub(obs.pump.unwrap_or(edge));
            self.report.slot_timed_out = true;
            self.stage = Stage::Done;
            return Action::Finished(self.report);
        }
        // Creep into the slot rather than tip liquid in: the last chunk before
        // the sensor wets is the overshoot, and it lands in the slot.
        match self.guard_empty(obs, plan.fine_steps) {
            Some(abort) => abort,
            None => {
                self.leg_steps += plan.fine_steps as u32;
                self.send(DeviceId::Pp01, Op::Dispense(plan.fine_steps))
            }
        }
    }

    /// Refuse to dispense more than the barrel holds. The pump would reject it
    /// anyway, but as a fault rather than as a sentence anyone can act on.
    fn guard_empty(&self, obs: Observation, chunk: u16) -> Option<Action> {
        let pos = obs.pump?;
        (pos < chunk).then(|| {
            Action::Abort(format!(
                "PP01 is down to {pos} steps, less than one {chunk}-step chunk; draw more wash and run again"
            ))
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn plan() -> Plan {
        Plan {
            draw_steps: 500,
            coarse_steps: 40,
            fine_steps: 4,
            cap_steps: 900,
            slot_cap_steps: 240,
            clear_margin_steps: 40,
            clear_cap_steps: 1200,
            verify_steps: 8,
            ..Plan::default()
        }
    }

    /// Drives a routine against a fake rig.
    ///
    /// The pump is modelled as a position that every command moves instantly;
    /// `wet_below` is the position at which each sensor starts seeing liquid,
    /// which is what "the liquid front has reached it" means in pump terms.
    /// A rig whose liquid front is a position on the piston scale.
    ///
    /// Dispensing lowers the piston count and pushes the front onward, so a
    /// sensor reads wet once the count has fallen to its threshold. S1 is
    /// nearest the pump and so has the highest threshold: the front reaches it
    /// first. Aspirating raises the count and pulls the front back, which is
    /// what clearing the line with air does — no special case needed.
    struct Fake {
        pos: u16,
        s1_at: u16,
        s2_at: u16,
        slot_at: u16,
        /// Seconds. Confirmation is a duration now, so the fake has to have a
        /// clock or nothing would ever hold still for long enough.
        clock: f64,
        /// SV02's port. The slot detector can only see liquid once the valve
        /// has actually connected the slot, which is what keeps the
        /// verification push — run against waste — out of the slot.
        port: u16,
        sent: Vec<(DeviceId, Op)>,
    }

    impl Fake {
        fn new(edge_wet_below: u16, slot_wet_below: u16) -> Self {
            Self {
                pos: 0,
                s1_at: edge_wet_below + 40,
                s2_at: edge_wet_below,
                slot_at: slot_wet_below,
                clock: 0.0,
                port: 0,
                sent: Vec::new(),
            }
        }

        fn observe(&self) -> Observation {
            Observation {
                pump: Some(self.pos),
                settled: true,
                now: self.clock,
                first_wet: Some(self.pos <= self.s1_at),
                edge_wet: Some(self.pos <= self.s2_at),
                slot_wet: Some(self.port == Plan::default().fill_port && self.pos <= self.slot_at),
            }
        }

        fn apply(&mut self, id: DeviceId, op: Op) {
            self.sent.push((id, op));
            if id == DeviceId::Sv02
                && let Op::ValveTo(p) = op
            {
                self.port = p;
            }
            if id != DeviceId::Pp01 {
                return;
            }
            match op {
                Op::Aspirate(n) => self.pos = self.pos.saturating_add(n),
                Op::Dispense(n) => self.pos = self.pos.saturating_sub(n),
                Op::Home => self.pos = 0,
                _ => {}
            }
        }

        /// Drives the setup — homing, clearing, drawing — until the measuring
        /// push begins, so a test can hand-step from there.
        fn run_to_coarse(&mut self, routine: &mut Routine) {
            for _ in 0..4000 {
                self.clock += 0.25;
                if routine.stage() == Stage::Coarse {
                    return;
                }
                if let Action::Send(id, op) = routine.step(self.observe()) {
                    self.apply(id, op);
                }
            }
            panic!("never reached the measuring push, stuck in {:?}", routine.stage());
        }

        /// Runs to completion, or gives up. Returns the outcome.
        fn run(&mut self, routine: &mut Routine) -> Action {
            for _ in 0..4000 {
                self.clock += 0.25;
                match routine.step(self.observe()) {
                    Action::Send(id, op) => self.apply(id, op),
                    Action::Wait => {}
                    done @ (Action::Finished(_) | Action::Abort(_)) => return done,
                }
            }
            panic!("routine did not settle in 4000 steps, stage {:?}", routine.stage());
        }
    }

    /// The pump must never be homed into a closed or capped valve.
    ///
    /// An SV-07 closes its common port on reset, and this rig leaves SV02
    /// parked on capped lines between runs. Emptying a barrel that may hold
    /// millilitres into either of those is how you push a fitting apart, so
    /// the waste path is opened first and the ordering is pinned here.
    #[test]
    fn the_pump_is_only_homed_once_a_path_to_waste_is_open() {
        let mut fake = Fake::new(400, 340);
        let mut routine = Routine::new(plan());
        let _ = fake.run(&mut routine);

        let at = |want: (DeviceId, Op)| fake.sent.iter().position(|s| *s == want).expect("never sent");
        let valves_reset = [DeviceId::Sv01, DeviceId::Sv02, DeviceId::Sv03].map(|v| at((v, Op::Home)));
        let solenoid_out = at((DeviceId::Pp01, Op::SolenoidInput(false)));
        let waste_open = at((DeviceId::Sv02, Op::ValveTo(9)));
        let pump_home = at((DeviceId::Pp01, Op::Home));

        for (valve, reset) in [DeviceId::Sv01, DeviceId::Sv02, DeviceId::Sv03].iter().zip(valves_reset) {
            assert!(reset < pump_home, "{} resets before the pump is homed", valve.tag());
        }
        assert!(solenoid_out < waste_open, "the pump must face SV02 before waste is selected");
        assert!(waste_open < pump_home, "waste must be open before the barrel is emptied");
        // And the fill port must not be selected until after the draw.
        let fill = at((DeviceId::Sv02, Op::ValveTo(10)));
        assert!(pump_home < fill, "the slot is only selected once the pump is empty and refilled");
    }

    /// The failure the first live run actually produced.
    ///
    /// Homing the barrel into waste pushes its whole contents past S2, so by
    /// the time the measuring push began the sensor was already wet and the
    /// dead volume came out as zero steps. Clearing with air first is what
    /// makes the number mean anything.
    #[test]
    fn a_line_that_is_already_primed_is_cleared_before_measuring() {
        // Every threshold is above the homed position, so both in-line sensors
        // read wet the moment the barrel is emptied — exactly the state the
        // rig was in when this first ran.
        let mut fake = Fake::new(470, 400);
        assert!(fake.observe().edge_wet == Some(true), "the line starts primed");
        let mut routine = Routine::new(plan());
        let Action::Finished(report) = fake.run(&mut routine) else { panic!("expected a report") };

        // Waste is selected twice: once to empty the barrel into before the
        // draw, and again after clearing. The one that matters here is the
        // second, so look for it after air rather than taking the first.
        let air = fake.sent.iter().position(|s| *s == (DeviceId::Sv02, Op::ValveTo(16))).expect("air selected");
        let waste = fake.sent[air..]
            .iter()
            .position(|s| *s == (DeviceId::Sv02, Op::ValveTo(9)))
            .map(|i| air + i)
            .expect("waste selected again after clearing");
        let pulled = fake.sent[air..waste].iter().any(|(id, op)| *id == DeviceId::Pp01 && matches!(op, Op::Aspirate(_)));
        assert!(pulled, "clearing means pulling back through air, not just switching the valve");
        assert!(report.to_edge_steps > 0, "a cleared line gives a real dead volume, not zero");
    }

    /// First contact is a leading edge, and a leading edge can be froth.
    ///
    /// A meniscus arrives, wets the detector and reads stable while the column
    /// behind it is still short of the sensor. Recording there gives a figure
    /// short by however ragged the front was. The run pushes on and only keeps
    /// a reading that survives the travel.
    #[test]
    fn a_front_that_breaks_under_travel_is_not_the_column() {
        let mut fake = Fake::new(470, 400);
        let mut routine = Routine::new(plan());
        fake.run_to_coarse(&mut routine);

        // Walk it up to first contact and into verification.
        for _ in 0..400 {
            fake.clock += 0.25;
            if routine.stage() == Stage::Verify {
                break;
            }
            if let Action::Send(id, op) = routine.step(fake.observe()) {
                fake.apply(id, op);
            }
        }
        assert_eq!(routine.stage(), Stage::Verify, "it should be checking the front holds");
        let bubbles_before = 0;

        // The reading breaks part-way through the verification push: that was
        // a meniscus, so the run goes back to approaching rather than
        // recording it.
        fake.clock += 0.25;
        let broken = Observation {
            pump: Some(fake.pos),
            settled: true,
            now: fake.clock,
            edge_wet: Some(false),
            slot_wet: Some(false),
            first_wet: Some(true),
        };
        let _ = routine.step(broken);
        assert_eq!(routine.stage(), Stage::Fine, "a front that breaks sends it back to the creep");

        // And it finishes properly once the real column arrives.
        let Action::Finished(report) = fake.run(&mut routine) else { panic!("expected a report") };
        assert_eq!(report.verified_steps, plan().verify_steps);
        assert!(report.bubbles_seen > bubbles_before, "the break is counted as a bubble");
    }

    /// Flicker and wetness answer different questions.
    ///
    /// A detector with a liquid column at it reads wet steadily. One with a
    /// bubble parked on it flickers. So for "has the front arrived" a flicker
    /// is a no, and for "is the line clear of liquid" a flicker is a yes —
    /// there is no column there either way.
    #[test]
    fn a_flickering_detector_clears_the_line_but_never_marks_an_arrival() {
        let plan = Plan { flicker_is_clear: 4, ..plan() };

        // Clearing: a detector that will not hold still is taken as clear,
        // because froth is not a liquid column.
        let mut routine = Routine::new(plan);
        let mut clock = 0.0;
        let mut flip = true;
        let mut cleared = false;
        for _ in 0..400 {
            clock += 0.25;
            flip = !flip;
            let obs = Observation {
                pump: Some(1000),
                settled: true,
                now: clock,
                edge_wet: Some(flip),
                slot_wet: Some(false),
                first_wet: Some(flip),
            };
            let _ = routine.step(obs);
            if routine.stage() != Stage::Clearing && routine.stage() != Stage::Homing {
                cleared = true;
                break;
            }
        }
        assert!(cleared, "froth should clear the line rather than run to the cap");

        // Arriving: the same flicker at the edge must never end a measuring leg.
        let mut fake = Fake::new(470, 400);
        let mut routine = Routine::new(plan);
        fake.run_to_coarse(&mut routine);
        let before = routine.stage();
        let mut flip = true;
        for _ in 0..40 {
            fake.clock += 0.25;
            flip = !flip;
            let obs = Observation {
                pump: Some(fake.pos),
                settled: true,
                now: fake.clock,
                edge_wet: Some(flip),
                slot_wet: Some(false),
                first_wet: Some(false),
            };
            if let Action::Send(id, op) = routine.step(obs) {
                // It may keep advancing, but it must not have recorded an edge.
                fake.apply(id, op);
            }
        }
        assert_eq!(routine.stage(), before, "a flickering edge is not an arrival");
    }

    /// A line that reads wet and never once changes while millilitres are
    /// pulled through it is not frothy — nothing is moving at all.
    #[test]
    fn a_line_that_never_changes_is_reported_as_not_flowing() {
        let mut routine = Routine::new(plan());
        let mut clock = 0.0;
        let mut last = Action::Wait;
        for _ in 0..600 {
            clock += 0.25;
            let obs = Observation {
                pump: Some(2000),
                settled: true,
                now: clock,
                edge_wet: Some(true),
                slot_wet: Some(false),
                first_wet: Some(true),
            };
            last = routine.step(obs);
            if matches!(last, Action::Abort(_)) {
                break;
            }
        }
        let Action::Abort(why) = last else { panic!("expected an abort, got {last:?}") };
        assert!(why.contains("nothing is flowing"), "{why}");
        assert!(why.contains("air port"), "it should say what to check: {why}");
    }

    /// S1 sits between the pump and the coil, so the front crosses it on the
    /// way. Recording where splits the dead volume into two measured links.
    #[test]
    fn the_first_sensor_is_noted_in_passing() {
        let mut fake = Fake::new(470, 400);
        let mut routine = Routine::new(plan());
        let Action::Finished(report) = fake.run(&mut routine) else { panic!("expected a report") };
        let first = report.to_first_steps.expect("S1 was crossed on the way to S2");
        assert!(first <= report.to_edge_steps, "S1 comes before S2: {first} vs {}", report.to_edge_steps);
    }

    #[test]
    fn it_draws_wash_before_it_pushes_any() {
        // The edge is 100 steps of travel from full, the slot 60 beyond it.
        let mut fake = Fake::new(400, 340);
        let mut routine = Routine::new(plan());
        let _ = fake.run(&mut routine);

        // The solenoid must face SV01 before the aspirate and SV02 after it:
        // drawing with the valve on the coil side pulls from the wrong line.
        // The run faces SV02 twice — once to empty into waste, once to push —
        // so the one that matters here is the last, not the first.
        let sol_in = fake.sent.iter().position(|s| *s == (DeviceId::Pp01, Op::SolenoidInput(true)));
        let draw = fake.sent.iter().position(|s| *s == (DeviceId::Pp01, Op::Aspirate(500)));
        let sol_out = fake.sent.iter().rposition(|s| *s == (DeviceId::Pp01, Op::SolenoidInput(false)));
        assert!(sol_in < draw, "solenoid must face the wash line before drawing");
        assert!(draw < sol_out, "the draw must finish before the path is switched to push");

        // And the valves must be on the rig's own ports.
        assert!(fake.sent.contains(&(DeviceId::Sv01, Op::ValveTo(1))));
        assert!(fake.sent.contains(&(DeviceId::Sv02, Op::ValveTo(10))));
    }

    #[test]
    fn the_edge_is_found_to_the_fine_chunk_not_the_coarse_one() {
        // Wetting at 470 means the front arrives 30 steps into the push, which
        // a 40-step coarse chunk would overshoot.
        let mut fake = Fake::new(470, 400);
        let mut routine = Routine::new(plan());
        let Action::Finished(report) = fake.run(&mut routine) else { panic!("expected a report") };

        assert!(report.edge_precise, "the fine approach should have found the edge");
        // The true answer is the travel from where the push began down to the
        // position at which S2 wets. Derived rather than written down, so the
        // test keeps meaning the same thing if the setup's travel changes.
        // The recorded edge is the column, not its leading edge: the run
        // pushes `verify_steps` further while checking S2 holds, so the answer
        // is that much beyond where the sensor first read wet.
        let first_contact = routine.push_start().expect("a measured push").abs_diff(fake.s2_at);
        let expected = first_contact + plan().verify_steps;
        let error = report.to_edge_steps.abs_diff(expected);
        assert_eq!(report.verified_steps, plan().verify_steps, "the push was verified");
        assert!(
            error <= plan().fine_steps,
            "edge reported at {} steps, expected {expected} (first contact {first_contact} plus {} verified): out by {error}",
            report.to_edge_steps,
            plan().verify_steps
        );
    }

    #[test]
    fn it_measures_the_run_on_from_the_edge_to_the_slot() {
        let mut fake = Fake::new(460, 400);
        let mut routine = Routine::new(plan());
        let Action::Finished(report) = fake.run(&mut routine) else { panic!("expected a report") };
        assert!(!report.slot_timed_out);
        // The slot wets 60 steps past the edge.
        let error = report.edge_to_slot_steps.abs_diff(60);
        assert!(error <= plan().coarse_steps, "slot leg {} steps, expected ~60", report.edge_to_slot_steps);
    }

    /// Everything pushed before the slot is opened must go to waste.
    ///
    /// S2 is upstream of SV02's common port, so an overshoot on the priming
    /// leg leaves through whatever port the valve is on. If that were the slot,
    /// a sensor that failed to wet would put the whole 1 mL cap into something
    /// that holds 300 µL.
    #[test]
    fn priming_overshoot_goes_to_waste_and_not_into_the_slot() {
        let mut fake = Fake::new(470, 400);
        let mut routine = Routine::new(plan());
        let _ = fake.run(&mut routine);

        let waste = fake.sent.iter().position(|s| *s == (DeviceId::Sv02, Op::ValveTo(9))).expect("waste selected");
        let slot = fake.sent.iter().position(|s| *s == (DeviceId::Sv02, Op::ValveTo(10))).expect("slot selected");
        assert!(waste < slot, "the line primes to waste before the slot is ever opened");

        // Nothing may be dispensed between the slot opening and the run's own
        // creep into it beyond the slot leg's cap.
        let after_slot: u32 = fake.sent[slot..]
            .iter()
            .filter_map(|(id, op)| match (id, op) {
                (DeviceId::Pp01, Op::Dispense(n)) => Some(*n as u32),
                _ => None,
            })
            .sum();
        assert!(
            after_slot <= plan().slot_cap_steps as u32,
            "{after_slot} steps went into the slot, cap is {}",
            plan().slot_cap_steps
        );
    }

    /// A slot sensor that never wets must not keep filling a 300 µL slot.
    #[test]
    fn a_dead_slot_sensor_cannot_overfill_the_slot() {
        let mut fake = Fake::new(470, 0); // the slot sensor never reads wet
        let mut routine = Routine::new(plan());
        let Action::Finished(report) = fake.run(&mut routine) else { panic!("expected a report") };
        assert!(report.slot_timed_out);

        let slot = fake.sent.iter().position(|s| *s == (DeviceId::Sv02, Op::ValveTo(10))).expect("slot selected");
        let into_slot: u32 = fake.sent[slot..]
            .iter()
            .filter_map(|(id, op)| match (id, op) {
                (DeviceId::Pp01, Op::Dispense(n)) => Some(*n as u32),
                _ => None,
            })
            .sum();
        // 240 steps at 2.083 µL is ~500 µL: the slot's 300 plus the line.
        assert!(into_slot <= plan().slot_cap_steps as u32, "{into_slot} steps into the slot");
    }

    /// A sensor that never wets must stop the run, not empty the barrel into
    /// the bench.
    #[test]
    fn a_sensor_that_never_wets_aborts_within_the_cap() {
        let mut fake = Fake::new(0, 0); // nothing ever reads wet
        // A cap below what the barrel holds, so the cap is what stops it
        // rather than the barrel running dry — that is the path under test.
        let plan = Plan { cap_steps: 200, ..plan() };
        let mut routine = Routine::new(plan);
        let Action::Abort(why) = fake.run(&mut routine) else { panic!("expected an abort") };
        assert!(why.contains("without wetting"), "{why}");

        let pushed: u32 = fake
            .sent
            .iter()
            .filter_map(|(id, op)| match (id, op) {
                (DeviceId::Pp01, Op::Dispense(n)) => Some(*n as u32),
                _ => None,
            })
            .sum();
        assert!(pushed <= plan.cap_steps as u32, "pushed {pushed} steps, cap is {}", plan.cap_steps);
    }

    /// The slot sensor failing is a result, not a crash: the S2 figure was
    /// still measured and is worth keeping.
    #[test]
    fn a_dead_slot_sensor_still_reports_the_edge() {
        let mut fake = Fake::new(470, 0);
        let mut routine = Routine::new(plan());
        let Action::Finished(report) = fake.run(&mut routine) else { panic!("expected a report") };
        assert!(report.slot_timed_out, "the slot leg should be marked as timed out");
        assert!(report.to_edge_steps > 0, "but the edge was found");
    }

    #[test]
    fn a_dry_supply_is_caught_before_anything_is_pushed() {
        struct Empty;
        let _ = Empty;
        let mut routine = Routine::new(plan());
        // Walk the setup with a barrel that never fills.
        let mut clock = 0.0;
        let mut last = Action::Wait;
        for _ in 0..200 {
            clock += 0.25;
            let obs = Observation {
                pump: Some(0),
                settled: true,
                now: clock,
                edge_wet: Some(false),
                slot_wet: Some(false),
                first_wet: Some(false),
            };
            last = routine.step(obs);
            if matches!(last, Action::Abort(_)) {
                break;
            }
        }
        let Action::Abort(why) = last else { panic!("expected an abort, got {last:?}") };
        assert!(why.contains("wash supply"), "{why}");
    }

    /// A bubble passing the detector reads wet for one sample. Stopping on it
    /// would record the dead volume short by however far the front still had
    /// to travel, and every dispensed volume after that would inherit the
    /// error.
    #[test]
    fn one_spurious_wet_reading_does_not_end_a_leg() {
        let mut fake = Fake::new(300, 200);
        let mut routine = Routine::new(plan());

        fake.run_to_coarse(&mut routine);
        let before = fake.pos;

        // One wet sample while the front is nowhere near, then dry again.
        let blip = Observation { pump: Some(fake.pos), settled: true, now: fake.clock, edge_wet: Some(true), slot_wet: Some(false), first_wet: Some(false) };
        assert_eq!(routine.step(blip), Action::Wait, "a reading that has only just appeared is not believed");
        assert_eq!(routine.stage(), Stage::Coarse, "and not a reason to stop the leg");

        // Back to dry well inside the confirmation window: the run carries on
        // advancing rather than recording an edge.
        fake.clock += 0.25;
        match routine.step(fake.observe()) {
            Action::Send(DeviceId::Pp01, Op::Dispense(n)) => {
                assert_eq!(n, plan().coarse_steps, "it should resume the coarse approach")
            }
            other => panic!("expected the approach to continue, got {other:?}"),
        }
        assert_eq!(fake.pos, before, "nothing moved on the blip itself");
    }

    /// Agreement across the configured number of samples is what stops a leg.
    #[test]
    fn a_sustained_wet_reading_does_end_a_leg() {
        let mut routine = Routine::new(Plan { confirm_secs: 2.0, ..plan() });
        let mut fake = Fake::new(300, 200);
        fake.run_to_coarse(&mut routine);
        let base = fake.clock;
        let at = |t: f64| Observation {
            pump: Some(fake.pos),
            settled: true,
            now: base + t,
            edge_wet: Some(true),
            slot_wet: Some(false),
            first_wet: Some(false),
        };
        // Wet, but only just: not yet believed, however many times it is read.
        assert_eq!(routine.step(at(0.0)), Action::Wait, "the reading has just appeared");
        assert_eq!(routine.step(at(0.1)), Action::Wait, "reading it again does not make it older");
        assert_eq!(routine.step(at(1.9)), Action::Wait, "still short of the two seconds");
        // Held for the full window: now it counts, and the run backs off.
        assert!(matches!(routine.step(at(2.1)), Action::Send(DeviceId::Pp01, Op::Aspirate(_))));
        assert_eq!(routine.stage(), Stage::BackingOff);
    }

    #[test]
    fn nothing_is_sent_while_the_rig_is_still_moving() {
        let mut routine = Routine::new(plan());
        let moving = Observation { pump: Some(100), settled: false, now: 99.0, edge_wet: Some(false), slot_wet: Some(false), first_wet: Some(false) };
        assert_eq!(routine.step(moving), Action::Wait);
        assert_eq!(routine.step(moving), Action::Wait);
    }

    /// Every advance is a bounded chunk. A command that could leave the pump
    /// running would survive losing the interface driving it.
    #[test]
    fn every_pump_move_is_bounded() {
        let mut fake = Fake::new(470, 400);
        let mut routine = Routine::new(plan());
        let _ = fake.run(&mut routine);
        for (id, op) in &fake.sent {
            if *id != DeviceId::Pp01 {
                continue;
            }
            match op {
                Op::Dispense(n) | Op::Aspirate(n) => {
                    assert!(*n > 0 && *n <= plan().draw_steps, "unbounded move {op:?}")
                }
                Op::SetSpeed(_) | Op::SolenoidInput(_) | Op::Home => {}
                other => panic!("the run should not send {other:?} to a pump"),
            }
        }
    }

    /// controller_v2 moves a liquid front past these sensors at 5–25 rpm.
    /// Nothing here should be winding that up without someone deciding to.
    #[test]
    fn liquid_is_moved_at_the_speeds_the_rig_is_tuned_for() {
        let p = Plan::default();
        assert!((5..=25).contains(&p.coarse_speed), "coarse {} rpm", p.coarse_speed);
        assert!((5..=25).contains(&p.fine_speed), "fine {} rpm", p.fine_speed);
        assert!(p.fine_speed <= p.coarse_speed, "the creep must not be the faster of the two");
    }

    #[test]
    fn the_plan_comes_off_the_rig() {
        let plan = Plan::for_rig(&Rig::default()).expect("the default rig is fully configured");
        assert_eq!(plan.wash_port, 1, "SV01 port 1 is the wash line");
        assert_eq!(plan.fill_port, 10, "SV02 port 10 feeds the fitted slot");
        assert_eq!(plan.edge_sensor, 5, "S2 is channel A5");
        assert_eq!(plan.slot_sensor, 2, "the slot's detector is A2");
    }

    #[test]
    fn a_rig_missing_a_line_says_which() {
        let mut rig = Rig::default();
        rig.ports.retain(|p| p.role != Role::Wash);
        let why = Plan::for_rig(&rig).expect_err("no wash line");
        assert!(why.contains("wash"), "{why}");
    }

    #[test]
    fn speeds_are_set_before_the_chunks_they_apply_to() {
        let mut fake = Fake::new(470, 400);
        let mut routine = Routine::new(plan());
        let _ = fake.run(&mut routine);
        // The fine speed must be commanded before the first fine chunk, or the
        // creep runs at the coarse speed and overshoots the edge.
        let fine_speed = fake.sent.iter().position(|s| *s == (DeviceId::Pp01, Op::SetSpeed(plan().fine_speed)));
        let fine_chunk = fake.sent.iter().position(|s| *s == (DeviceId::Pp01, Op::Dispense(plan().fine_steps)));
        assert!(fine_speed.is_some(), "the fine speed is never set");
        assert!(fine_speed < fine_chunk, "the fine chunk runs before its speed is set");
    }
}

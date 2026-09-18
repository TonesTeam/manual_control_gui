//! The Peltier slot-temperature board, on the CAN adapter.
//!
//! A DRV8701 + STM32C092 board holds one slot at a setpoint. It runs its own
//! PID — this drives and watches it, it does not close the loop. The wire
//! protocol lives in the `slot-temp-sensor-can` crate; everything here is the
//! shape the rest of the program sees.
//!
//! The state and the commands are plain `serde` types, compiled always. Only
//! the worker that opens the adapter needs the `can` feature, so a GUI reads
//! temperature from the server exactly as it reads valve positions, without
//! linking the CAN stack or libudev.

use std::collections::VecDeque;

use serde::{Deserialize, Serialize};

/// What the operator can ask the board to do.
///
/// Deliberately small: the board's gains, autotune and manual duty are set at
/// commissioning with the crate's own examples, not from the rig screen where
/// a mis-click drives a Peltier.
#[derive(Serialize, Deserialize, Clone, Copy, Debug, PartialEq)]
pub enum TempOp {
    /// Target temperature in °C.
    SetTemperature(f32),
    /// Half-width of the band counted as "reached", in °C.
    SetTolerance(f32),
    /// Hand the output to the board's PID.
    StartPid,
    /// Stop regulating and brake the bridge.
    StopPid,
    /// Drop the latched fault history.
    ClearErrors,
}

impl TempOp {
    pub fn label(&self) -> String {
        match self {
            TempOp::SetTemperature(c) => format!("set temperature {c:.2} °C"),
            TempOp::SetTolerance(c) => format!("set tolerance ±{c:.2} °C"),
            TempOp::StartPid => "start PID".into(),
            TempOp::StopPid => "stop PID".into(),
            TempOp::ClearErrors => "clear latched faults".into(),
        }
    }
}

/// One trend sample.
///
/// The target rides along with the reading instead of being read off the live
/// setpoint at draw time: a plot of "measurement now" against "target now"
/// would show the setpoint as a flat line that jumps, hiding exactly the thing
/// worth watching — the step, and the measurement chasing it.
#[derive(Serialize, Deserialize, Clone, Copy, Debug, PartialEq)]
pub struct TempSample {
    /// Seconds on the state clock, as for every other trend.
    pub t: f64,
    pub measured: f32,
    /// None while the board has not told us what it is aiming for.
    pub target: Option<f32>,
}

/// Everything known about the board, mirrored to every screen.
#[derive(Serialize, Deserialize, Clone, Debug, Default)]
pub struct TempState {
    /// The rig is configured to have a temperature board at all.
    pub enabled: bool,
    /// The adapter is open and the board is answering.
    pub connected: bool,
    /// Why it is not, when it is not.
    pub error: Option<String>,
    pub temperature_c: Option<f32>,
    pub setpoint_c: Option<f32>,
    pub tolerance_c: Option<f32>,
    /// Bridge duty, signed: negative cools on a bipolar board.
    pub duty_pct: Option<f32>,
    pub current_a: Option<f32>,
    pub pid_running: bool,
    /// Inside the tolerance band.
    pub reached: bool,
    /// The RTD is open, shorted or out of range — the reading is not to be
    /// trusted and the board refuses to regulate.
    pub sensor_fault: bool,
    pub autotuning: bool,
    /// Faults true right now, named. Anything here is a reason not to run.
    /// Empty when the board is clean — the crate's own text says "none", which
    /// is a word, not an absence, so it is never stored here.
    pub active_faults: String,
    /// Faults since the last clear, named. Event bits only ever appear here.
    pub latched_faults: String,
    /// The raw fault words behind those strings. Kept because a badge and a
    /// button have to ask "is anything wrong" without parsing prose.
    pub active_word: u16,
    pub latched_word: u16,
    pub last_seen: Option<f64>,
    /// Trend samples, as for the pumps. Rebuilt client-side, never sent.
    #[serde(skip)]
    pub history: VecDeque<TempSample>,
}

/// How the board reads on a badge.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TempStatus {
    /// No board configured.
    Off,
    /// Configured but not answering.
    Offline,
    /// The RTD or the bridge is in fault.
    Fault,
    /// Answering, PID not running.
    Idle,
    /// Regulating, still outside the band.
    Driving,
    /// Regulating and inside the band.
    Holding,
}

impl TempStatus {
    pub fn label(self) -> &'static str {
        match self {
            TempStatus::Off => "OFF",
            TempStatus::Offline => "OFFLINE",
            TempStatus::Fault => "FAULT",
            TempStatus::Idle => "IDLE",
            TempStatus::Driving => "DRIVING",
            TempStatus::Holding => "HOLDING",
        }
    }
}

impl TempState {
    pub fn status(&self) -> TempStatus {
        match self {
            _ if !self.enabled => TempStatus::Off,
            _ if !self.connected => TempStatus::Offline,
            _ if self.sensor_fault || self.active_word != 0 => TempStatus::Fault,
            _ if self.pid_running && self.reached => TempStatus::Holding,
            _ if self.pid_running => TempStatus::Driving,
            _ => TempStatus::Idle,
        }
    }

    /// Distance left to the setpoint, when both are known.
    pub fn error_c(&self) -> Option<f32> {
        Some(self.setpoint_c? - self.temperature_c?)
    }

    /// Clears the live readings but keeps the trend, for when the link drops.
    pub fn go_offline(&mut self, why: Option<String>) {
        self.connected = false;
        self.error = why;
        self.temperature_c = None;
        self.duty_pct = None;
        self.current_a = None;
        self.pid_running = false;
        self.reached = false;
        self.autotuning = false;
    }

    /// Records a trend sample and drops everything older than `window`
    /// seconds. Both writers — the CAN worker and the client rebuilding the
    /// trend from snapshots — go through here, so the two agree on the shape
    /// of the ring.
    pub fn push_sample(&mut self, t: f64, measured: f32, target: Option<f32>, window: f64) {
        self.history.push_back(TempSample { t, measured, target });
        while self.history.front().is_some_and(|s| t - s.t > window) {
            self.history.pop_front();
        }
    }

    /// Anything latched that a clear would drop.
    pub fn has_latched(&self) -> bool {
        self.latched_word != 0
    }

    #[cfg(test)]
    fn latched_fault_text(&mut self, text: &str) {
        self.latched_faults = text.to_string();
    }
}

/// Smallest half-height a trend plot is drawn with, in °C. A slot holding dead
/// steady has no span at all, and scaling by it would divide by zero; this also
/// keeps sensor noise from filling the whole box with a flat line's jitter.
const TREND_PAD_C: f32 = 0.25;

/// The vertical range a trend plot should cover for these samples: both traces,
/// padded, never collapsed. None when there is nothing to plot.
pub fn trend_range<'a>(samples: impl IntoIterator<Item = &'a TempSample>) -> Option<(f32, f32)> {
    let (mut lo, mut hi) = (f32::INFINITY, f32::NEG_INFINITY);
    for s in samples {
        for v in [Some(s.measured), s.target].into_iter().flatten() {
            lo = lo.min(v);
            hi = hi.max(v);
        }
    }
    if !lo.is_finite() || !hi.is_finite() {
        return None;
    }
    let pad = ((hi - lo) * 0.1).max(TREND_PAD_C);
    Some((lo - pad, hi + pad))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn connected() -> TempState {
        TempState { enabled: true, connected: true, temperature_c: Some(21.7), ..Default::default() }
    }

    /// The crate spells "no faults" as the word "none", so a state built from
    /// its text alone reads every healthy board as broken. The badge asks the
    /// numeric word instead.
    #[test]
    fn a_clean_board_is_not_a_fault() {
        let mut t = connected();
        t.active_word = 0;
        t.latched_word = 0;
        assert_eq!(t.status(), TempStatus::Idle);
        assert!(!t.has_latched());
        assert!(t.active_faults.is_empty(), "no fault means no text to show");
    }

    #[test]
    fn a_real_fault_still_shows() {
        let mut t = connected();
        t.active_word = 0x0001; // RTD_FAULT
        t.active_faults = "RTD_FAULT".into();
        assert_eq!(t.status(), TempStatus::Fault);

        // An RTD fault alone is enough, even with a clear error word.
        let mut t = connected();
        t.sensor_fault = true;
        assert_eq!(t.status(), TempStatus::Fault);
    }

    #[test]
    fn latched_faults_are_clearable_but_are_not_a_fault_now() {
        let mut t = connected();
        t.latched_word = 0x0010;
        t.latched_fault_text("OVERCURRENT");
        assert!(t.has_latched(), "there is history to clear");
        assert_eq!(t.status(), TempStatus::Idle, "but nothing is wrong right now");
    }

    #[test]
    fn regulating_reads_as_driving_until_the_band_is_entered() {
        let mut t = connected();
        t.pid_running = true;
        t.setpoint_c = Some(30.0);
        assert_eq!(t.status(), TempStatus::Driving);
        let left = t.error_c().expect("both ends are known");
        assert!((left - 8.3).abs() < 1e-3, "{left} °C left to go");
        t.reached = true;
        assert_eq!(t.status(), TempStatus::Holding);
    }

    #[test]
    fn a_board_that_stops_answering_does_not_keep_its_last_reading() {
        let mut t = connected();
        t.pid_running = true;
        t.duty_pct = Some(100.0);
        t.go_offline(Some("cable pulled".into()));
        assert_eq!(t.status(), TempStatus::Offline);
        assert_eq!(t.temperature_c, None, "a stale reading must not look live");
        assert_eq!(t.duty_pct, None);
        assert!(!t.pid_running);
    }

    #[test]
    fn no_board_configured_reads_as_off() {
        assert_eq!(TempState::default().status(), TempStatus::Off);
    }

    #[test]
    fn the_trend_keeps_only_the_window() {
        let mut t = connected();
        for i in 0..100 {
            t.push_sample(i as f64, 20.0 + i as f32 * 0.1, Some(37.0), 10.0);
        }
        let first = t.history.front().expect("the window is not empty");
        let last = t.history.back().expect("the window is not empty");
        assert_eq!(last.t, 99.0, "the newest sample is always kept");
        assert!(last.t - first.t <= 10.0, "span {} s is over the window", last.t - first.t);
        assert_eq!(t.history.len(), 11, "ten seconds of one-per-second samples, plus the boundary");
    }

    /// The target is part of the sample, not read off the live setpoint: a
    /// ramp has to plot against the target that was set at the time.
    #[test]
    fn a_sample_remembers_the_target_it_was_chasing() {
        let mut t = TempState::default();
        t.push_sample(0.0, 21.0, None, 600.0);
        t.push_sample(1.0, 21.2, Some(37.0), 600.0);
        t.push_sample(2.0, 22.5, Some(60.0), 600.0);
        let targets: Vec<_> = t.history.iter().map(|s| s.target).collect();
        assert_eq!(targets, [None, Some(37.0), Some(60.0)]);
    }

    #[test]
    fn an_empty_trend_has_no_range() {
        assert_eq!(trend_range(&VecDeque::new()), None);
    }

    #[test]
    fn a_flat_trend_still_gets_a_band() {
        let flat: Vec<_> = (0..5).map(|i| TempSample { t: i as f64, measured: 37.0, target: Some(37.0) }).collect();
        let (lo, hi) = trend_range(&flat).expect("five samples are something to plot");
        assert!(hi > lo, "a range of {lo}..{hi} would divide by zero");
        assert!(lo < 37.0 && hi > 37.0, "the reading sits inside the band, not on its edge");
    }

    #[test]
    fn the_range_covers_both_traces() {
        let samples = vec![
            TempSample { t: 0.0, measured: 21.0, target: Some(60.0) },
            TempSample { t: 1.0, measured: 25.0, target: None },
        ];
        let (lo, hi) = trend_range(&samples).expect("two samples are something to plot");
        assert!(lo < 21.0, "below the coldest reading");
        assert!(hi > 60.0, "above the target, even where the target is unknown");
    }
}

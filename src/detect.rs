//! Deciding what is at a detector: liquid, air, or neither yet.
//!
//! The board gives a raw 8-bit reading and its own wet/dry bit. The bit alone
//! is not enough to drive a pump from, because it says nothing about how
//! settled it is — a front sweeping past crosses the threshold on its way and
//! the bit flips as readily as it does for a tube that has genuinely filled.
//!
//! # What the readings actually look like
//!
//! Recorded on this rig at ~20 Hz, pulling a front back past the coil detector
//! and pushing it out again (`tests/data/a5_front.txt`):
//!
//! | condition | reading | rolling sd over 0.5 s |
//! | --- | --- | --- |
//! | settled liquid | 252 | **0.00** |
//! | settled air | ~107 | ~0.4 |
//! | front going past | swinging 43–147 | up to **71** |
//!
//! A settled detector does not vary at all. That is the whole basis of what
//! follows: **variance says whether to believe the level, and only then does
//! the level say which it is.** A mean on its own cannot tell 252-going-down
//! from 107-going-up at the moment both read 150.
//!
//! # Why a duration as well
//!
//! A front can pause mid-sweep and look settled for a sample or two, so a
//! verdict also has to hold for a while before it is acted on. That is the
//! same reasoning as controller_v2's `MIN_STREAM_TIME_MS`, measured here in
//! samples of quiet rather than in wall-clock alone.

use std::collections::VecDeque;

/// What a detector is looking at.
#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum Verdict {
    /// A liquid column, held still long enough to believe.
    Liquid,
    /// Air, likewise.
    Dry,
    /// Something is moving past: a front, or froth. No answer yet.
    Unsettled,
    /// Not enough samples to say anything.
    Unknown,
}

impl Verdict {
    pub fn label(self) -> &'static str {
        match self {
            Verdict::Liquid => "liquid",
            Verdict::Dry => "dry",
            Verdict::Unsettled => "unsettled",
            Verdict::Unknown => "?",
        }
    }

    /// Whether this is an answer rather than an absence of one.
    pub fn settled(self) -> bool {
        matches!(self, Verdict::Liquid | Verdict::Dry)
    }
}

/// A one-pole low-pass filter, specified by cutoff frequency.
///
/// The coefficient is derived per sample from the interval that actually
/// elapsed, rather than being a fixed weight. A bare `alpha` only means
/// something alongside a sample rate, and this one's is not constant: the
/// readings share a CAN adapter with the temperature exchanges and arrive when
/// they arrive. A filter tuned as "0.35 at 20 Hz" quietly becomes a different
/// filter the moment the rate drifts, and there is nothing in the number to
/// say so. A cutoff in hertz stays the same filter whatever the rate does.
///
/// For a one-pole RC response, `RC = 1 / (2π·fc)` and `α = dt / (RC + dt)`.
#[derive(Clone, Copy, Debug)]
pub struct LowPass {
    cutoff_hz: f32,
    value: Option<f32>,
}

impl LowPass {
    pub fn new(cutoff_hz: f32) -> Self {
        Self { cutoff_hz: cutoff_hz.max(0.001), value: None }
    }

    /// The weight one sample carries, for a cutoff and the gap before it.
    pub fn alpha_for(cutoff_hz: f32, dt: f32) -> f32 {
        if dt <= 0.0 {
            return 0.0;
        }
        let rc = 1.0 / (std::f32::consts::TAU * cutoff_hz.max(0.001));
        (dt / (rc + dt)).clamp(0.0, 1.0)
    }

    /// Folds in a sample that arrived `dt` seconds after the last one.
    pub fn push(&mut self, sample: f32, dt: f32) -> f32 {
        let next = match self.value {
            // Start on the first sample rather than ramping up from zero,
            // which would read as a front arriving every time it is reset.
            None => sample,
            Some(prev) => prev + Self::alpha_for(self.cutoff_hz, dt) * (sample - prev),
        };
        self.value = Some(next);
        next
    }

    pub fn value(&self) -> Option<f32> {
        self.value
    }

    pub fn reset(&mut self) {
        self.value = None;
    }
}

/// A one-pole high-pass, the low-pass's complement.
///
/// Where the low-pass keeps the level and throws away the movement, this keeps
/// the movement and throws away the level — which is the other half of the
/// question. Liquid or air is a level; a front or a bubble going past is
/// movement. One filter answers each, and neither answers both.
///
/// `y[n] = a·(y[n-1] + x[n] − x[n-1])`, with `a = RC / (RC + dt)`, so it
/// passes anything faster than the cutoff and rejects a standing value
/// entirely: a settled detector reads zero here however high or low it sits.
#[derive(Clone, Copy, Debug)]
pub struct HighPass {
    cutoff_hz: f32,
    last_in: Option<f32>,
    value: f32,
}

impl HighPass {
    pub fn new(cutoff_hz: f32) -> Self {
        Self { cutoff_hz: cutoff_hz.max(0.001), last_in: None, value: 0.0 }
    }

    pub fn push(&mut self, sample: f32, dt: f32) -> f32 {
        let Some(prev_in) = self.last_in else {
            // Nothing to difference against yet; a first sample is not a change.
            self.last_in = Some(sample);
            self.value = 0.0;
            return 0.0;
        };
        let rc = 1.0 / (std::f32::consts::TAU * self.cutoff_hz);
        let a = if dt <= 0.0 { 1.0 } else { rc / (rc + dt) };
        self.value = a * (self.value + sample - prev_in);
        self.last_in = Some(sample);
        self.value
    }

    /// How much movement there is right now, regardless of direction.
    pub fn magnitude(&self) -> f32 {
        self.value.abs()
    }

    pub fn reset(&mut self) {
        self.last_in = None;
        self.value = 0.0;
    }
}

/// Rolling statistics over the last `span` seconds.
///
/// Held by time rather than by a sample count, for the same reason the filter
/// is: "the last ten samples" is half a second at one rate and two seconds at
/// another, and the thing being measured is how long the reading has held
/// still.
#[derive(Clone, Debug)]
pub struct Window {
    span: f32,
    samples: VecDeque<(f32, f32)>,
    /// Latched once the window has covered its span.
    ///
    /// Without it, trimming the oldest sample can drop the covered span just
    /// below the threshold for one call and the detector reports "not enough
    /// data yet" in the middle of a steady reading. Samples do not arrive on a
    /// perfect cadence here, so that happened constantly.
    filled: bool,
}

impl Window {
    pub fn new(span_secs: f32) -> Self {
        Self { span: span_secs.max(0.01), samples: VecDeque::new(), filled: false }
    }

    /// Adds a sample taken at `t` seconds and drops anything older than the
    /// span.
    pub fn push(&mut self, t: f32, sample: f32) {
        self.samples.push_back((t, sample));
        if let (Some((first, _)), Some((last, _))) = (self.samples.front(), self.samples.back())
            && last - first >= self.span
            && self.samples.len() >= 4
        {
            self.filled = true;
        }
        while self.samples.front().is_some_and(|(t0, _)| t - t0 > self.span) {
            self.samples.pop_front();
        }
    }

    /// Whether the window has covered its span. Two samples a span apart would
    /// satisfy a time test while saying nothing about the shape between them,
    /// so a minimum count is required as well.
    pub fn full(&self) -> bool {
        self.filled
    }

    pub fn mean(&self) -> Option<f32> {
        (!self.samples.is_empty())
            .then(|| self.samples.iter().map(|(_, v)| v).sum::<f32>() / self.samples.len() as f32)
    }

    /// Population standard deviation — the agitation signal.
    pub fn sd(&self) -> Option<f32> {
        let mean = self.mean()?;
        let n = self.samples.len() as f32;
        let var = self.samples.iter().map(|(_, v)| (v - mean).powi(2)).sum::<f32>() / n;
        Some(var.sqrt())
    }

    pub fn spread(&self) -> Option<f32> {
        let mut it = self.samples.iter().map(|(_, v)| *v);
        let first = it.next()?;
        let (lo, hi) = it.fold((first, first), |(lo, hi), v| (lo.min(v), hi.max(v)));
        Some(hi - lo)
    }

    pub fn reset(&mut self) {
        self.samples.clear();
        self.filled = false;
    }
}

/// How a detector's readings are turned into a verdict.
#[derive(Clone, Copy, Debug)]
pub struct Tuning {
    /// Above this reading is liquid, below is air. The board's own switching
    /// level, which it reports.
    pub level: f32,
    /// Seconds of history the statistics run over.
    pub window_secs: f32,
    /// Standard deviation at or below which the detector counts as still.
    ///
    /// Settled readings measured 0.00 and a passing front reached 71, so this
    /// can sit low without being twitchy.
    pub quiet_sd: f32,
    /// Readings must also stay within this of each other across the window; a
    /// slow steady drift has a small sd but is not settled.
    pub quiet_spread: f32,
    /// How long the reading must stay quiet before a verdict is given.
    pub dwell_secs: f32,
    /// Cutoff of the low-pass that produces the level.
    pub level_cutoff_hz: f32,
    /// Cutoff of the high-pass that produces the agitation.
    pub motion_cutoff_hz: f32,
    /// High-pass magnitude above which the reading counts as moving.
    ///
    /// Measured against the recording, this gate alone reaches the ideal two
    /// changes of mind while blocking only 103 samples of 1414 — against 324
    /// for the variance gate at the same result. Movement is simply a better
    /// question than variance for "is something going past", because that is
    /// the question a high-pass answers by construction.
    pub motion_max: f32,
}

impl Default for Tuning {
    fn default() -> Self {
        Self {
            level: 115.0,
            window_secs: 0.5,
            // The high-pass is the primary gate; these two are here to catch
            // what a high-pass cannot — a drift slower than its corner, and
            // jitter that averages out. Set well below the 71 a front reached,
            // but not so tight that they add dead time of their own.
            quiet_sd: 8.0,
            // A steady ramp is the one thing neither the high-pass nor the
            // deviation catches: a constant slope produces a constant
            // high-pass output, which sits under the gate if the slope is
            // gentle enough, and a ramp's deviation is small over a short
            // window. The spread across the window is what sees it.
            quiet_spread: 12.0,
            dwell_secs: 0.5,
            // The recorded front took about a second and a half to sweep past,
            // so a 3 Hz corner keeps its shape while removing sample noise.
            level_cutoff_hz: 3.0,
            // Lower, so the high-pass responds to the sweep itself rather than
            // only to sample-to-sample jitter.
            motion_cutoff_hz: 2.0,
            motion_max: 4.0,
        }
    }
}

/// Watches one detector and says what is at it.
#[derive(Clone, Debug)]
pub struct Detector {
    tuning: Tuning,
    filter: LowPass,
    motion: HighPass,
    window: Window,
    last_t: Option<f32>,
    quiet_since: Option<f32>,
    verdict: Verdict,
    /// The reading last held steadily, used as the anchor for departures.
    baseline: Option<f32>,
    /// Verdict changes since the last reset, for reporting how restless a run
    /// was.
    pub changes: u32,
}

impl Detector {
    pub fn new(tuning: Tuning) -> Self {
        Self {
            filter: LowPass::new(tuning.level_cutoff_hz),
            motion: HighPass::new(tuning.motion_cutoff_hz),
            window: Window::new(tuning.window_secs),
            tuning,
            last_t: None,
            quiet_since: None,
            verdict: Verdict::Unknown,
            baseline: None,
            changes: 0,
        }
    }

    pub fn verdict(&self) -> Verdict {
        self.verdict
    }

    pub fn smoothed(&self) -> Option<f32> {
        self.filter.value()
    }

    pub fn sd(&self) -> Option<f32> {
        self.window.sd()
    }

    /// The high-pass output: how much the reading is moving right now.
    pub fn motion(&self) -> f32 {
        self.motion.magnitude()
    }

    /// Folds in one raw reading taken at `t` seconds.
    pub fn push(&mut self, raw: u8, t: f32) -> Verdict {
        let dt = self.last_t.map(|prev| (t - prev).max(0.0)).unwrap_or(0.0);
        self.last_t = Some(t);
        let smoothed = self.filter.push(raw as f32, dt);
        self.motion.push(raw as f32, dt);
        // Statistics run on the raw samples, not the filtered ones: a
        // low-pass flattens exactly the variance being measured, and a front
        // that has been smoothed into a gentle ramp looks settled.
        self.window.push(t, raw as f32);

        let next = if !self.window.full() {
            Verdict::Unknown
        } else {
            let sd = self.window.sd().unwrap_or(f32::MAX);
            let spread = self.window.spread().unwrap_or(f32::MAX);
            // Three ways of being still, and all of them must hold. The
            // high-pass catches a front going past; the spread catches a drift
            // too slow for the high-pass to see; the deviation catches jitter
            // that averages out over the window.
            let moving = self.motion.magnitude() > self.tuning.motion_max;
            if !moving && sd <= self.tuning.quiet_sd && spread <= self.tuning.quiet_spread {
                let since = *self.quiet_since.get_or_insert(t);
                if t - since >= self.tuning.dwell_secs {
                    // Quiet and has been for a while: this is what the line
                    // looks like when nothing is happening to it.
                    self.baseline = self.window.mean();
                    if smoothed > self.tuning.level { Verdict::Liquid } else { Verdict::Dry }
                } else {
                    // Quiet, but not for long enough to be sure yet: hold
                    // whatever was already believed rather than flapping.
                    self.verdict
                }
            } else {
                self.quiet_since = None;
                Verdict::Unsettled
            }
        };
        if next != self.verdict {
            self.changes += 1;
            self.verdict = next;
        }
        next
    }

    pub fn reset(&mut self) {
        self.filter.reset();
        self.motion.reset();
        self.window.reset();
        self.last_t = None;
        self.quiet_since = None;
        self.verdict = Verdict::Unknown;
        self.baseline = None;
        self.changes = 0;
    }
}

impl Detector {
    /// The reading the detector last settled on, if it has settled at all.
    ///
    /// This is the anchor the departure test works from, and it is only
    /// updated while the detector is quiet — so a front sweeping past cannot
    /// drag the baseline along with it.
    pub fn baseline(&self) -> Option<f32> {
        self.baseline
    }

    /// How far the present reading has moved from that baseline.
    pub fn departure(&self) -> Option<f32> {
        Some((self.filter.value()? - self.baseline?).abs())
    }

    /// Whether the reading has left its settled value by more than `counts`.
    ///
    /// This, not the settled verdict, is what a pump should be stopped on. A
    /// settled detector measured a standard deviation of exactly zero, so any
    /// real movement stands out immediately — the departure is visible on the
    /// first sample that changes, while a settled verdict cannot be given
    /// until the front has gone past *and* the reading has been quiet again
    /// for the dwell. On the recorded front that is the difference between
    /// noticing at 5.8 s and confirming at 8.4 s, and the pump travels for
    /// every second of it.
    pub fn departed(&self, counts: f32) -> bool {
        self.departure().is_some_and(|d| d > counts)
    }
}

/// What a watcher is waiting for before it stops a pump.
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum StopOn {
    /// The reading has left its settled baseline by this many counts.
    ///
    /// The fastest honest trigger there is: a settled detector varies by
    /// nothing, so a departure shows on the first sample that moves.
    Departure { counts: f32 },
    /// The detector has settled on this verdict. Slower — it cannot be said
    /// until the front has gone past and the reading has been quiet again —
    /// but it is an answer rather than a change.
    Settled(Verdict),
}

impl StopOn {
    /// Whether this detector now meets the condition.
    pub fn met(self, d: &Detector) -> bool {
        match self {
            StopOn::Departure { counts } => d.departed(counts),
            StopOn::Settled(want) => d.verdict() == want,
        }
    }

    pub fn label(self) -> String {
        match self {
            StopOn::Departure { counts } => format!("the reading moves {counts:.0} counts from settled"),
            StopOn::Settled(v) => format!("the reading settles on {}", v.label()),
        }
    }
}

/// The simplest possible rule, for comparison: the board's own threshold on
/// the raw sample, with no filtering and no memory.
pub fn bare_threshold(raw: u8, level: f32) -> Verdict {
    if raw as f32 > level { Verdict::Liquid } else { Verdict::Dry }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The recorded run: pulling a front back past the coil detector on air,
    /// then pushing it out on waste. Real samples, not a simulation.
    const TRACE: &str = include_str!("../tests/data/a5_front.txt");

    struct Sample {
        t: f32,
        raw: u8,
        phase: String,
    }

    fn trace() -> Vec<Sample> {
        TRACE
            .lines()
            .filter(|l| !l.starts_with('#') && !l.trim().is_empty())
            .map(|l| {
                let mut f = l.split_whitespace();
                let t = f.next().unwrap().parse().unwrap();
                let raw = f.next().unwrap().parse().unwrap();
                let _board_state = f.next().unwrap();
                Sample { t, raw, phase: f.next().unwrap().to_string() }
            })
            .collect()
    }

    #[test]
    fn the_trace_is_the_one_that_was_recorded() {
        let t = trace();
        assert_eq!(t.len(), 1414, "the recorded run");
        assert!(t.last().unwrap().t > 60.0, "about a minute of it");
        // It starts on settled liquid and passes through a front.
        assert_eq!(t[0].raw, 252);
        assert!(t.iter().any(|s| s.raw < 60), "the front goes past");
    }

    /// The numbers the module is built on, checked against the recording so
    /// they cannot quietly stop being true.
    #[test]
    fn settled_readings_do_not_vary_and_moving_ones_do() {
        let t = trace();
        let sd_of = |phase: &str| {
            let v: Vec<(f32, f32)> = t.iter().filter(|s| s.phase == phase).map(|s| (s.t, s.raw as f32)).collect();
            let mut w = Window::new(0.5);
            let mut worst: f32 = 0.0;
            for (t, x) in v {
                w.push(t, x);
                if w.full() {
                    worst = worst.max(w.sd().unwrap());
                }
            }
            worst
        };
        assert_eq!(sd_of("idle"), 0.0, "a settled detector does not vary at all");
        assert!(sd_of("aspirate") > 20.0, "a front going past does, a lot");
    }

    /// The board's own bit flips while a front sweeps past, because a sweep
    /// crosses the threshold on its way. Anything driving a pump off that bit
    /// is reacting to the journey, not the destination.
    #[test]
    fn the_bare_threshold_reacts_to_the_journey() {
        let t = trace();
        let mut flips = 0;
        let mut last = None;
        for s in &t {
            let v = bare_threshold(s.raw, 115.0);
            if last.is_some_and(|l| l != v) {
                flips += 1;
            }
            last = Some(v);
        }
        assert!(flips >= 8, "the raw threshold changes its mind {flips} times over one pass");
    }

    /// Counts changes of mind between liquid and dry, ignoring any "not yet"
    /// in between.
    ///
    /// That is the number a pump cares about. Counting every transition would
    /// punish the settled detector for reporting the thing it exists to
    /// report — that a front is going past and there is no answer yet.
    fn settled_changes(verdicts: impl Iterator<Item = Verdict>) -> u32 {
        let (mut changes, mut last) = (0, None);
        for v in verdicts.filter(|v| v.settled()) {
            if last.is_some_and(|l| l != v) {
                changes += 1;
            }
            last = Some(v);
        }
        changes
    }

    /// The recording contains exactly two real transitions: the front leaving
    /// the detector, and coming back. Anything above two is an algorithm
    /// reacting to the journey rather than the destination.
    #[test]
    fn the_settled_detector_calls_only_the_real_transitions() {
        let t = trace();
        let bare = settled_changes(t.iter().map(|s| bare_threshold(s.raw, 115.0)));
        assert!(bare > 2, "the bare threshold changes its mind {bare} times");

        let mut d = Detector::new(Tuning::default());
        let settled = settled_changes(t.iter().map(|s| d.push(s.raw, s.t)));
        assert_eq!(settled, 2, "the front leaves once and comes back once");
        assert!(settled < bare, "{settled} against the bare threshold's {bare}");
    }

    /// The high-pass is the other half: it ignores the level entirely and
    /// reports only movement, so a settled detector reads zero however high or
    /// low it happens to sit.
    #[test]
    fn the_high_pass_sees_the_front_and_not_the_level() {
        let mut hp = HighPass::new(1.0);
        // A standing value, high or low, produces nothing.
        for _ in 0..40 {
            hp.push(252.0, 0.05);
        }
        assert!(hp.magnitude() < 1.0, "a standing reading is not movement: {}", hp.magnitude());
        for _ in 0..40 {
            hp.push(107.0, 0.05);
        }
        let settled_low = hp.magnitude();

        // The recorded sweep produces plenty.
        let t = trace();
        let mut hp = HighPass::new(1.0);
        let mut worst: f32 = 0.0;
        let mut last_t = None;
        for s in &t {
            let dt = last_t.map(|p| s.t - p).unwrap_or(0.05);
            last_t = Some(s.t);
            hp.push(s.raw as f32, dt);
            if (5.5..8.0).contains(&s.t) {
                worst = worst.max(hp.magnitude());
            }
        }
        assert!(worst > 20.0, "the sweep should show as movement: {worst}");
        assert!(worst > settled_low * 10.0, "and stand well clear of a settled reading");
    }

    /// It must still get the answer right at both ends, not just be quiet.
    #[test]
    fn it_reads_the_settled_ends_correctly() {
        let t = trace();
        let mut d = Detector::new(Tuning::default());

        // The run opens on settled liquid.
        for s in t.iter().take_while(|s| s.phase == "idle") {
            d.push(s.raw, s.t);
        }
        assert_eq!(d.verdict(), Verdict::Liquid, "it opens on a liquid column at 252");

        // And by the end of the pull the detector is in air.
        let mut d = Detector::new(Tuning::default());
        for s in t.iter().filter(|s| s.phase == "aspirate") {
            d.push(s.raw, s.t);
        }
        assert_eq!(d.verdict(), Verdict::Dry, "the pull leaves it in air at ~107");
    }

    /// While the front is actually going past, the answer is "not yet" rather
    /// than a guess — which is the verdict a pump should be stopped on.
    #[test]
    fn a_front_going_past_reads_as_unsettled() {
        let t = trace();
        let mut d = Detector::new(Tuning::default());
        let mut unsettled_during_front = false;
        for s in &t {
            let v = d.push(s.raw, s.t);
            // The first transition is around t = 6 s in the recording.
            if (5.8..7.5).contains(&s.t) && v == Verdict::Unsettled {
                unsettled_during_front = true;
            }
        }
        assert!(unsettled_during_front, "the sweep should read as unsettled while it happens");
    }

    /// Departure from a settled baseline is the earliest honest signal there
    /// is, because a settled baseline does not move at all.
    #[test]
    fn a_departure_is_noticed_before_a_verdict_can_be_given() {
        let t = trace();
        let mut d = Detector::new(Tuning::default());
        let (mut departed_at, mut verdict_changed_at) = (None, None);
        let mut last_settled = None;
        for s in &t {
            let v = d.push(s.raw, s.t);
            if departed_at.is_none() && d.departed(20.0) {
                departed_at = Some(s.t);
            }
            if v.settled() {
                if last_settled.is_some_and(|l| l != v) && verdict_changed_at.is_none() {
                    verdict_changed_at = Some(s.t);
                }
                last_settled = Some(v);
            }
        }
        let departed = departed_at.expect("the front departs from the baseline");
        let confirmed = verdict_changed_at.expect("and is eventually confirmed");
        assert!(departed < confirmed, "departure at {departed:.2}s, verdict at {confirmed:.2}s");
        // The recorded front leaves 252 at about 5.8 s.
        assert!((5.5..6.5).contains(&departed), "noticed at {departed:.2}s");
    }

    /// The baseline must not be dragged along by the thing it is measuring.
    #[test]
    fn a_moving_front_does_not_move_the_baseline() {
        let mut d = Detector::new(Tuning::default());
        for i in 0..60 {
            d.push(252, i as f32 * 0.05);
        }
        let settled = d.baseline().expect("settled on liquid");
        assert!((settled - 252.0).abs() < 1.0);

        // Sweep it through the middle: the baseline stays where it was.
        for (i, v) in [200u8, 150, 120, 90, 150, 60].into_iter().enumerate() {
            d.push(v, 3.0 + i as f32 * 0.05);
        }
        assert_eq!(d.baseline(), Some(settled), "an unsettled reading is not a new baseline");
        assert!(d.departed(20.0), "and the departure is obvious");
    }

    #[test]
    fn a_low_pass_starts_on_its_first_sample() {
        let mut f = LowPass::new(3.0);
        // Ramping up from zero would look like a front arriving on every reset.
        assert_eq!(f.push(252.0, 0.05), 252.0);
        assert!(f.push(107.0, 0.05) < 252.0);
        assert!(f.push(107.0, 0.05) > 107.0, "and it approaches rather than jumping");
    }

    #[test]
    fn a_window_reports_nothing_until_it_has_something() {
        let mut w = Window::new(0.2);
        assert_eq!(w.mean(), None);
        assert_eq!(w.sd(), None);
        assert!(!w.full());
        for i in 0..5 {
            w.push(i as f32 * 0.05, 10.0);
        }
        assert!(w.full(), "0.2 s of samples covers a 0.2 s span");
        assert_eq!(w.sd(), Some(0.0), "identical samples do not vary");
        assert_eq!(w.spread(), Some(0.0));
        // And it slides by time rather than growing without bound.
        for i in 5..40 {
            w.push(i as f32 * 0.05, 10.0);
        }
        assert!(w.mean().is_some());
    }

    /// A slow drift has almost no sample-to-sample variance but is not
    /// settled, so the spread has to be checked as well as the deviation.
    #[test]
    fn a_slow_drift_is_not_mistaken_for_settled() {
        let mut d = Detector::new(Tuning::default());
        let mut v = 60.0f32;
        let mut seen = Verdict::Unknown;
        for i in 0..60 {
            v += 2.0; // two counts a sample: quiet, but going somewhere
            seen = d.push(v as u8, i as f32 * 0.05);
        }
        assert_eq!(seen, Verdict::Unsettled, "a steady climb is not a settled reading");
    }

    #[test]
    fn nothing_is_claimed_before_the_window_fills() {
        let mut d = Detector::new(Tuning::default());
        assert_eq!(d.push(252, 0.0), Verdict::Unknown);
        assert_eq!(d.push(252, 0.05), Verdict::Unknown);
    }
}

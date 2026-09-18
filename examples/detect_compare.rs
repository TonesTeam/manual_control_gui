//! Compares ways of deciding liquid-versus-air against a recorded front.
//!
//! The metric is how often each one changes its mind about *liquid versus
//! dry*, ignoring any "not yet" it reports in between. The recording contains
//! exactly two real transitions — the front leaving the detector, and coming
//! back — so anything above two is the algorithm reacting to the journey.
use tstand_controler::detect::{Detector, HighPass, LowPass, Tuning, Verdict, bare_threshold};

/// Counts settled changes of mind, and how late the first real one is called.
fn score(mut verdicts: impl Iterator<Item = (f32, Verdict)>) -> (u32, Option<f32>) {
    let (mut changes, mut last, mut first_change_at) = (0u32, None, None);
    for (t, v) in verdicts.by_ref() {
        if !v.settled() {
            continue;
        }
        if let Some(prev) = last
            && prev != v
        {
            changes += 1;
            first_change_at.get_or_insert(t);
        }
        last = Some(v);
    }
    (changes, first_change_at)
}

fn main() {
    let text = std::fs::read_to_string("tests/data/a5_front.txt").unwrap();
    let rows: Vec<(f32, u8)> = text
        .lines()
        .filter(|l| !l.starts_with('#') && !l.trim().is_empty())
        .map(|l| {
            let mut f = l.split_whitespace();
            (f.next().unwrap().parse().unwrap(), f.next().unwrap().parse().unwrap())
        })
        .collect();

    println!("{:<36} {:>8}  {:>10}  notes", "algorithm", "changes", "1st at");
    println!("{:<36} {:>8}  {:>10}  two real transitions in the recording", "(ideal)", 2, "~6.1 s");

    let (c, t) = score(rows.iter().map(|(t, raw)| (*t, bare_threshold(*raw, 115.0))));
    println!("{:<36} {:>8}  {:>10.2}  reacts to the sweep", "bare threshold (what the board does)", c, t.unwrap_or(0.0));

    for cutoff in [8.0f32, 3.0, 1.0] {
        let mut lp = LowPass::new(cutoff);
        let mut last_t = None;
        let (c, t) = score(rows.iter().map(|(t, raw)| {
            let dt = last_t.map(|p| t - p).unwrap_or(0.05);
            last_t = Some(*t);
            let e = lp.push(*raw as f32, dt);
            (*t, if e > 115.0 { Verdict::Liquid } else { Verdict::Dry })
        }));
        println!("{:<36} {:>8}  {:>10.2}  smoother, but still guesses mid-sweep", format!("low-pass {cutoff:.0} Hz then threshold"), c, t.unwrap_or(0.0));
    }

    // High-pass as the gate: movement says "not yet", the low-pass says which.
    for cutoff in [2.0f32, 1.0] {
        let (mut lp, mut hp) = (LowPass::new(3.0), HighPass::new(cutoff));
        let mut last_t = None;
        let mut blocked = 0u32;
        let (c, t) = score(rows.iter().map(|(t, raw)| {
            let dt = last_t.map(|p| t - p).unwrap_or(0.05);
            last_t = Some(*t);
            let level = lp.push(*raw as f32, dt);
            let moving = hp.push(*raw as f32, dt).abs() > 4.0;
            if moving {
                blocked += 1;
                return (*t, Verdict::Unsettled);
            }
            (*t, if level > 115.0 { Verdict::Liquid } else { Verdict::Dry })
        }));
        println!(
            "{:<36} {:>8}  {:>10.2}  high-pass gate blocks {:>4} of {} samples",
            format!("low-pass + high-pass gate {cutoff:.0} Hz"), c, t.unwrap_or(0.0), blocked, rows.len()
        );
    }

    for (name, tuning) in [
        ("variance gate, sd<=3 dwell 0.5 s", Tuning::default()),
        ("variance gate, sd<=1 dwell 1.0 s", Tuning { quiet_sd: 1.0, dwell_secs: 1.0, ..Tuning::default() }),
        ("variance gate, sd<=8 dwell 0.25 s", Tuning { quiet_sd: 8.0, dwell_secs: 0.25, ..Tuning::default() }),
    ] {
        let mut d = Detector::new(tuning);
        let mut unsettled = 0u32;
        let (c, t) = score(rows.iter().map(|(t, raw)| {
            let v = d.push(*raw, *t);
            if v == Verdict::Unsettled {
                unsettled += 1;
            }
            (*t, v)
        }));
        println!(
            "{:<36} {:>8}  {:>10.2}  says \"not yet\" for {:>4} of {} samples",
            name, c, t.unwrap_or(0.0), unsettled, rows.len()
        );
    }

    // And the trigger a pump is actually stopped on: departure from a settled
    // baseline, which is the earliest thing that can be trusted.
    let mut d = Detector::new(Tuning::default());
    let mut departed_at = None;
    for (t, raw) in &rows {
        d.push(*raw, *t);
        if departed_at.is_none() && d.departed(20.0) {
            departed_at = Some(*t);
        }
    }
    println!(
        "\ndeparture from a settled baseline first trips at {:.2} s — earlier than any verdict above,",
        departed_at.unwrap_or(0.0)
    );
    println!("because a settled reading varies by nothing, so the first sample that moves is already proof.");
}

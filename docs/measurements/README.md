# Measurements taken on the rig

Raw material for the fluidics model and the detection tuning. Everything here
came off the hardware; nothing is simulated.

## `fluidics-2026-09-18.json`

The liquid model as the last run left it — which is **not** the same as the best
figures. It holds whatever the most recent run recorded, and two of those three
legs were compromised:

| link in the file | value | trust it? |
| --- | --- | --- |
| `Pump → S1` | 83 µL | **no** — S1 already read wet when the push began, so this is "how far short the front was", not the segment |
| `S1 → S2` | 2216 µL | **yes** — the verified-column run, the best figure taken |
| `S2 → Slot` | 17 µL | **no** — the slot branch still held liquid from an earlier run, so its detector wetted almost at once |

The table below is what the measurements actually support. Volumes are µL;
lengths follow by dividing by the bore area (0.7854 µL/mm at 1 mm ID).

| segment | volume | length | confidence |
| --- | --- | --- | --- |
| PP01 → S1 (A4) | 417 µL | 53 cm | one clean run |
| S1 → S2 (A4 → A5), through the coil | 1583–2216 µL | 2.0–2.8 m | three runs, see below |
| S2 → slot (A5 → A2) | 208–225 µL | 26–29 cm | two runs agreeing |

The coil figure grew as the method improved, and the later numbers are the
better ones:

- **1583 µL** — stopping when S2 first read wet. Too early: first contact is the
  leading edge of the front, not the column behind it.
- **1750 µL** — an earlier run with the line cleared only as far as S2.
- **2216 µL** — stopping only once S2 had *held* wet across a further 250 µL of
  pushing. This is the column rather than its leading edge, and is the figure to
  trust of the three.

Nothing yet measures the coil twice by the same method, so treat 2.2 m as good
to maybe ±10 % rather than as settled.

**To get a clean set of all three**, the next run needs two things the last one
lacked: the line cleared back *past S1* before the push starts, so the front
begins at the barrel and PP01 → S1 is a real segment; and the slot drained
through PP02/SV03 first, so its detector is dry when the last leg begins.

## `../../tests/data/a5_front.txt`

1414 samples at ~20 Hz of the A5 detector while a front was pulled back past it
on the air port and pushed out again on waste. Columns: seconds, raw 8-bit ADC,
the board's own wet bit, and which phase the run was in.

This is the fixture the detection tests in `src/detect.rs` run against, and the
input to `cargo run --example detect_compare`. It is the reason the detector
gates on movement rather than level:

| condition | reading | rolling sd over 0.5 s |
| --- | --- | --- |
| settled liquid | 252 | 0.00 |
| settled air | ~107 | ~0.4 |
| front going past | swinging 43–147 | up to 71 |

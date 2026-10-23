# Card: the carriers' mocap velocity filter, `slope` against the rig's `ema` (22 Oct 2026)

Owner Wesley. Origin: the 22 Oct rig day (docs/reports/rig_review_2026-10-22.md, finding 1). The filter itself is card
2026-10-14_tracker_timing part B (critic rev 2 answered on 14 Oct; built, switch off, never flown until 22 Oct).
Decisions: docs/decisions.md, DECIDED 2026-10-22 review answers (1)-(3), (11).

## 1. Question
Does the 4-frame least-squares velocity (`slope`) on the carrier quads stop the two-quad ring swing that the rig's
alpha-0.2 filter (`ema`) lets grow, without making three or four quads worse? (Wesley: "we have to come up with a
strategy to fix this"; the slope flights h2slope and f2slope "seemed like P2 fixed it".)

## 2. Hypothesis
- On the rig the carriers' velocity trails by 45-47 ms with `ema`, 16-17 ms with `slope` (the bags, reviewer 22 Oct).
- With two quads the ring's see-saw along the pair's line (~1.3-1.5 Hz at 0.55 m) is damped only through the quads'
  vertical loop. With `ema` the throttle reacts 41-72 ms behind the vertical speed; with `slope` 22-24 ms. The claim:
  that delay decides whether the see-saw grows.
- **Predictions:**
  - rig pair: half the 1.4 Hz see-saw in the climb or less, no growth in the hover;
  - the twin (unfiltered today) shows the swing only once it gets the rig's `ema`;
  - three and four quads are no worse with `slope` (less tilt jitter at 1.1-1.9 Hz, the spin's 1.5 Hz yaw wobble lower,
    the path lag no longer).
- **Not claimed:** why `ema` fails at 1.4 Hz. The reviewer found the commanded-position trail (101-115 ms) unchanged
  by the filter, so the mechanism stays open.

## 3. The one variable
`SIM_MOCAP_VEL_FILTER` / `MDC_MOCAP_VEL_FILTER` on the carriers: `ema` against `slope`. The ring and QUAD4's tip stay
on `ema` (as the 14 Oct design). Everything else as each cell's baseline config (below).

The ring on `slope` too is a **separate arm, later** (new code in the planner's input path, critic first):
`ring_slope` against `slope`.

## 4. Baseline
| Cell | Baseline runs | Number |
|---|---|---|
| Rig pair, 0.55 m, ema | RIG-1022-h2, -h2t9p, -h2rts (h2m excluded: its ring stalled 0.37 m short) | 1.4 Hz see-saw rms 0-6 s after lift-off: 3.72 / 2.60 / 2.99 deg (reviewer) |
| Rig pair, 0.55 m, slope | RIG-1022-h2slope, -f2slope | 1.46 / 1.35 deg; decaying |
| Rig pair, ema, 21 Oct | RIG-1021-c2q | calm (contradicts "ema always grows") |
| Twin pair, 0.55 m, unfiltered | R2244, R2255 | see-saw p-p 0.1-2.4 deg, decaying |

## 5. Pass / fail numbers (frozen 22 Oct ~12:15, before any run of this card)
**M1, the pair swing:**
- The ring normal's horizontal tip (both components, deg), band-passed 1.1-1.9 Hz (3rd-order Butterworth, zero
  phase).
- Its rms magnitude over (a) 0-6 s after lift-off and (b) the hover, from the top to the first of LAND / drop /
  detection (the reviewer's definition, 22 Oct; set at ~12:20 before any stage-1 number was looked at).
- The see-saw p-p per 2 s window, for growth.
- Computed by `tools/pair_swing.py`. Before the first claim run its numbers must reproduce the reviewer's
  (h2 3.72, h2t9p 2.60, h2rts 2.99, h2slope 1.46, f2slope 1.35 deg at 0-6 s) within 0.1 deg.

**Twin as a testbed** (stage 1, 0.55 m pair hover, `b2_hover_vf_{none,ema,slope}`, 2 each; exploratory rows):
- **The twin can test it** if `ema`'s M1(b) is at least 2x `none`'s in both repeats.
- Otherwise the twin cannot test it: the twin arms below are reported as uninformative and the rig decides.

**Stage 1b, added 22 Oct 14:27 after stage 1 failed** (the card's file time; the first stage-1b run started 14:28). The delayed-twin study R2365-R2380
(exploratory) found that 60 ms of extra command delay (SIM_CMD_DELAY_S from 24.0 / 26.5 s on the nodes' sim clock, which is 1.6-3.8 s
before the ring reaches the top on the run's clock: reviewer 22 Oct) makes the
twin's ema pair swing (R2373) while slope stays calm (R2374), as on the rig. The delayed twin is valid up to LAND only:
its landings fault with the delay on. Stage 1b repeats it: `b2_hover_{vf,L100}_{ema,slope}_cd060`, 0.55 m and 1 m,
2 per filter, interleaved.
- **The delayed twin is a testbed** if every ema run's M1(b) (top to LAND) is at least 1.0 deg.
- **Confirms the filter there:** every slope run's M1(b) is at most 0.5x the lower ema run's at the same cable length.
- **Falsifies it:** any slope run at least 0.8x the lower ema run's.
- Outside the bars: the tracker's Smith predictor and the ring on slope did not fix the swing at 60-80 ms (R2375-R2378,
  R2380); cmd_next_tick from the start flipped a drone in the creep (R2379).
- Result (R2381-R2388, reviewer [weak]): both bars hold on the reviewer's own numbers (ema 6.64-7.04 deg, slope
  0.10-0.14 deg). Limits: 60 ms was picked from the exploratory sweep (the 1 m cell had flown once), so only the 0.55 m
  cell is out of sample; the ema pairs sat at the 0.8 throttle cap 18-19 % of the hover, ~10 cm low (the swing is
  throttle-capped); it is the rig's mode (1.38-1.40 Hz, the quads' heights in antiphase) and direction, not the rig's
  flight (the rig swung in the climb; its slope pairs kept some 1.4 Hz swing).

**Twin, 1 m** (the nine baseline cells x {ema, slope} x 2; the same runs freeze the bands, N2 rule):
- **Pair hover and pair figure-8:**
  - confirms: `slope`'s M1 at most 0.5x `ema`'s in both repeats, no abort or drop;
  - falsifies: `slope`'s M1 at least 0.8x `ema`'s in either repeat.
  - Only if the testbed criterion held.
- **Three and four quads, not worse:**
  - `slope` passes if, in both repeats of each cell:
    - the ring tilt's 1.1-1.9 Hz rms over the path is at most 1.2x `ema`'s;
    - the path lag (corner_metrics, s) is at most `ema`'s + 0.02;
    - no abort, drop or envelope fault.
  - Any cell failing any of these falsifies "not worse" for that fleet size.

**Rig, Wed 28** (1 m, QUAD3/QUAD4 or the two best magnets, hover, >= 10 s at the top). The order is interleaved and
the packs matched: E S E S, then P4 (ema on packs run down to the h2slope voltage, ~22.3 V).
- **Confirms:**
  - both `slope` runs below both `ema` runs on M1(a) and on M1(b);
  - neither `slope` hover grows (the last 2 s window's p-p at most the first's + 1 deg).
- **Falsifies:**
  - either `slope` run at or above the lower `ema` run on M1(a) or M1(b);
  - or P4 as calm as both `slope` runs (M1(b) at most the higher `slope` run's): then the pack, not the filter.
- **Three quads on the rig** (the 3-quad cells flown once per filter, interleaved): informational for "not worse", same
  numbers as the twin's.

**Stop rules** (sheet): pair see-saw over 15 deg p-p, or growth over two windows: LAND.

## 6. Repeats
- Twin: 2 per arm.
- Rig: 2 per arm for the pair (plus P4), 1 per arm for the 3-quad cells.

## 7. Cost
- Twin:
  - stage 1: 6 runs at 0.55 m (laptop, 22 Oct);
  - the 36 baseline runs, which double as this card's 1 m arms (lab PC evenings + laptop, 23-27 Oct);
  - later, the ring arm: ~6.
- Rig: 5 pair flights + 6 three-quad flights on Wed 28, in the sheet's order.

## 8. New code?
- **For the carrier arms: no.** The filter and both switches exist (14 Oct).
  - **Harness:** a run config's `env:` block, set by tools/run_experiment.py on the laptop and the lab PC (today only
    the lab's delay_batch.py reads `lab_env`).
  - **The twin default:** `ema`, as the rig (Wesley 22 Oct).
- **For the ring arm: yes.** A ring option in the mocap node and the emulator, default off. The critic before it flies;
  if it fails its bar, the option is removed.

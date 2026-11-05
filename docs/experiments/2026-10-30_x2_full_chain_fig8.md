# Card X2-full: the supervisor's chain 4 -> 3 -> 2 -> 3 -> 4 on the figure-eight in one run (30 Oct 2026)

Owner Wesley. Row X2 of `docs/sim_runs_todo.md` and `docs/plan_rescope_2026-10-30.md` (X2: "a drop, or a release peak
> 30 deg" falsifies); decisions.md DECIDED 2026-10-30 ~15:10 (2): the full square chain on the figure-eight in one
continuous run, two variants differing at the square-to-T step, announced (soft detach) and unannounced with the drop
limit raised in simulation, every other step unannounced; the morning's restart-from-the-T version stays the boundary
case (~14:05). Written 30 Oct ~15:40 AEDT, before any run of it. **Bars FROZEN 30 Oct ~15:46 on Wesley's word, as
proposed (both choices below taken), before any X2-full run.** No new code (section 8).

**Two choices made here for Wesley to confirm before the bars freeze:**
1. **Both variants fly the same params, the drop limit at 80 deg and the 60 deg ring envelope off** (as X1's sqT
   cell), so they differ in the square-to-T event alone (DETACH vs RELEASE of plate 9). A is still scored against the
   rig's rule: its peak must stay at or under 30 deg (the rescope's falsifier and the 4-quad tilt drop), so a peak the
   rig would have dropped is a miss, not a silent pass; the higher limit only lets the rest of A's chain be scored
   after such a miss. The alternative (A under the chain's 45 deg file, `configs/rig/chain_pair_tip.yaml`) is one
   line in `x2_full_fig8_soft.yaml` and makes the variants differ in two things.
2. **Releases mid-lap, rejoins in the lap-end holds** (S's case 7 pattern, X1's `f4` release point: the centre crossing
   at speed), so every stage flies at least one full lap: square 1.5 laps, T 1, pair 1.5, T 1, square 1 (6 laps).
   A rejoin cannot start while the path runs (planner: "REJOIN REFUSED ... the path is running"; the rescope's stated
   limit), so the rejoins have to sit in a hold.

## 1. Question
Wesley (30 Oct): fly the supervisor's whole chain, the square 0/3/6/9 to the T 0/3/6, to the pair 0/6, back to the T
and back to the square, a lap per stage, on the figure-eight in one continuous run. Does it fly with the square-to-T
step announced; and with it unannounced, what does the drop rule's boundary look like when the limit is raised?

## 2. Hypothesis
- **A (square-to-T announced, soft detach 3 s).** Every step flies: the soft release peaks 23-30 deg (R2312 hover
  22.9, R2313 mid figure-8 25.9, R2821/R2822 hover with the marker-check params 29.6/29.0 deg; R2840/R2842 on the
  1.15 kg ring 24.9/26.6), the T's rider lets go with a peak of ~2-4 deg (R2821/R2822 2.2, R2314 3.5) and is caught by
  its magnet marker in ~0.1 s, the two rejoins fold in under 5 deg (hover: T to square 1.27-1.55 deg, R2578/R2579/
  R2596; pair to T 1.45-3.02 deg, R2589/R2594/R2595), and the square's last lap is within P of the no-event square.
  **The 30 deg bar sits within ~0.5 deg of the marker-check hover peaks: it may fall either way.**
- **B (square-to-T unannounced, the drop at 80 deg).** The square-to-T release rolls the ring to 55-65 deg (hover
  57-61, R2249/R2823; the announced hard detach mid figure-8 62.7 deg, R2311) and is back within 5 s; nothing is
  lost at 80 deg; the rest of the chain then flies as in A.

## 3. The one variable
The square-to-T step: an announced DETACH of plate 9 (`drone 1`, soft: `detach_unload_s` 3.0) in A, an unannounced
RELEASE of it in B. Everything else is one config pair, and the no-event base flies the same configs with no event.
- World `four_rigid_ground_rig_L100.sdf` (0.86 kg ring, 1 m rods), typed 0,270,180,90 = plates 0/9/6/3 (drones 0-3).
- Path: the figure-8 at size 1.0 (x +-1.0, y +-0.5 m; X1's `f4` and the thesis's `fig8big`), 0.2 m/s, a 5 s hover hold
  first, `traj_laps` 6 (lap ~34.5 s), `traj_lap_hold_s` 3.0 at each lap end (S's case 7 machinery, 23 Oct).
- Planner: 5 SQP iterations, `plan_fast_refs` on, the `slope` filter; `rejoin_departed` on, `departed_land` off (the
  freed quads stage on their plates' bearings, ~1.03 m from the ring centre, inside the +-1.57 m staging bound at every
  point of this path), io `tether_tips` (the rod ends: the marker check and the weld); `payload_tilt_envelope` false.
- Params `configs/twin/x2_chain_tip80.yaml` (new, twin only): `configs/rig/chain_pair_tip.yaml`'s values (the T and the
  pair fly under `drop_gap_deg` 200 with `min_survivors` 2; `int_k_xy` 0; `tip_detect_m` 0.04, the marker check that
  sees the T's ~0.1 N rider let go; the stretch check alone missed it, R2258) with `drop_tilt_deg` 45 -> 80.
- Events (both variants; WAIT events re-time the ones after them):
  ARM, TAKEOFF, WAIT_LIFT; WAIT_TRAJ lap 1.5 -> **DETACH 1 (A) / RELEASE 1 (B)** (square -> T);
  WAIT_TRAJ lap 2.5 -> RELEASE 3 (the T's rider, plate 3: T -> pair);
  WAIT_LOG "lap 4 done" -> +0.5 s ATTACH 3 -> WAIT_LOG "REJOIN WELD drone 4" (pair -> T);
  WAIT_LOG "lap 5 done" -> +0.5 s ATTACH 1 -> WAIT_LOG "REJOIN WELD drone 2" (T -> square);
  WAIT_TRAJ lap 6 -> +2 s LAND.
- Configs (`configs/experiments/`): `x2_full_fig8_soft` (A), `x2_full_fig8_unann` (B), `x2_full_fig8_base` (the
  square on the same lapped path, no event: P's base and the per-lap reference). The three differ only in `name`,
  `description`, the events and the base's `criteria`; all pass `tools/run_experiment.py --check` (30 Oct 15:39; the
  inertia warning is every L100 square config's).

## 4. Baseline
- **The no-event square on the same path**, `x2_full_fig8_base` x2 on the same tree (new: no lapped 4-quad figure-8
  has flown). P and the per-lap path errors read it. For scale: the single-lap square at size 1.0 in S's C, 11.32 mm
  and 1.05 deg (the it5 mean, s4_fig8big, ema).
- **The steps flown alone (hover unless said; exploratory rows):** soft square-to-T R2312 22.9 deg, R2313 (mid
  figure-8, size 0.5) 25.9; the chain square -> T -> pair in one flight R2821/R2822 (soft 29.6/29.0 deg, the rider's
  release 2.2 deg, caught in 100-118 ms, the pair 0.5-0.6 deg); unannounced square-to-T R2249 57.1 deg, R2823; X1's
  sqT cell (today, hover, the same 80 deg limit) is the twin's own reading of B's step; rejoins: T -> square E1b
  R2578/R2579 (ON_PLATE 24.9 s after START, fold-in 1.27/1.30 deg), pair -> T E2 R2594/R2595 (1.45/2.25 deg).

## 5. Pass / fail numbers (FROZEN 30 Oct ~15:46, Wesley, as proposed, before any X2-full run)
Bars are taken from the cards whose steps these are, unchanged: the chain card (`2026-10-21_chain_release.md`, arm 2,
the T to the pair), the rejoin card (`2026-10-23_rejoin_departed.md`, E1 T -> square, E2 pair -> T, E4 staging on a
path), S's P (`2026-10-23_scalability.md`), X1's sqT U5 (`2026-10-30_x1_release_ablation.md`, B's step), and the
rescope's X2 falsifier (30 deg). Scorers: `tools/s_metrics.py` (per event and per stage, rejoins, late ticks, drops),
`tools/x1_metrics.py` (B's step: abs peak, T6 settle), `tools/n2_bands.py`'s clock-free path error on lap windows
(section 8: the lap wrapper is missing), `tools/pair_swing.py` (the pair's roll and lean). Lap k = the planner's
trajectory clock from (k-1) to k laps.

**A, the claim: the full chain flies with the square-to-T step announced.**
- **A1, square -> T (DETACH 1, soft, at lap 1.5).** UNLOAD then the planner at n = 3 within 6.0 s of the DETACH
  (3 s unload, 2 s gate wait, 1 s), no unload cancelled; ring tilt peak (the tilt vector's largest departure from its
  mean over the 5 s before the DETACH, to the next event) <= 30 deg; no drop or loss. Reported: abs peak, time over
  15 deg, T6 settle.
- **A2, T -> pair (RELEASE 3, unannounced, at lap 2.5; chain card arm 2):** UNANNOUNCED DETACH and the planner at
  n = 2 within 1.0 s of the RELEASE; tilt peak < 15 deg from the RELEASE to the ATTACH; the pair's lean < 5 deg (mean
  over its last 10 s before the ATTACH); no drop. Reported: the pair's roll peak-to-peak (arm 2's 5 deg was a hover
  bar; on the path it is not one), S's L (<= 8.83 deg), detection by marker or stretch.
- **A3, pair -> T (ATTACH 3 at lap 4's end; rejoin card E2) and A4, T -> square (ATTACH 1 at lap 5's end; E1):**
  (a) REJOIN_ON_PLATE within 30 s of REJOIN_START; (b) JOINT_ATTACHED within 0.5 s of it, then REJOIN_WELD and
  FOLD_IN; (c) ring tilt <= 5 deg from REJOIN_WELD to +15 s; the rod end within 2 cm of its plate point through the
  blend; the rejoiner's planned tension at the blend's end within 10 % of the balanced share; no REJOIN_REFUSED or
  REJOIN_ABORT. A4 also: the square's lean <= 2 deg over the last 10 s before LAND. Reported: S's J (the planner one
  bigger within 20 s of the ATTACH; E1's hover rejoin reached its plate 24.9 s after START, so J's 20 s is expected to
  miss here and is not this card's bar).
- **A5, the square's last lap (lap 6, no event; S's P):** clock-free path error <= max(1.25 x the base's lap 6 mean,
  that mean + 3 mm), and tilt peak <= the base's lap 6 mean + 1 deg.
- **A6, the whole run:** no drop, abort or envelope fault; every quad lands; through the laps flown while staged,
  every staged drone inside +-1.6 m, at least 0.6 m from the ring centre and 0.8 m from every carrier body (E4).
- **Confirms:** A1-A6 in both valid repeats. **Falsifies:** a drop or loss; A1's peak over 30 deg; any of A2-A6
  missed in a valid repeat.

**B, the boundary: the square-to-T step unannounced, the drop at 80 deg (simulation only).**
- **B1, square -> T (RELEASE 1 at lap 1.5; X1's sqT U5, frozen 15:20):** no loss (the 80 deg drop, the planner's
  drop, or a quad's own 60 deg attitude envelope); abs peak <= 80 deg; T6 settle (release_bands' time back within
  max(2 sigma_pre, 0.5 deg) of the post-release steady tilt, held 1 s) <= 5.0 s. Reported: the peak against the rig's
  30 and 45 deg rules (predicted over both: that is the boundary), detection time, the resize time.
- **B2-B6:** A2-A6 unchanged.
- **Confirms** (the chain completes past an unannounced square-to-T once the limit is lifted): B1-B6 in both valid
  repeats. **Falsifies:** a loss at 80 deg, T6 over 5.0 s, or any of B2-B6 missed. A loss in B is reported as "lost
  even at 80 deg"; the restart-from-the-T version (decided 30 Oct ~14:05) stays the way the rest of the chain is shown.

**Validity (a void run is flown once more):** late planner or tracker ticks <= 1 %; no failed planner solve;
read-back `traj_laps` 6, `traj_lap_hold_s` 3, `detach_unload_s` 3, `rejoin_departed` true, `drop_tilt_deg` 80,
`tip_detect_m` 0.04, `drop_gap_deg` 200, `min_survivors` 2, the filter slope; every event fired (no TRAJ_TIMEOUT or
LOG_TIMEOUT); a REJOIN_REFUSED "the path is running" means a lap-end hold was missed: void (as S's case 7), not a
falsification; a gap drop of the T or the pair is a config error: void. Lap 1 has no event in any run, so each run's
lap 1 must sit within P's form of the base's lap 1; a miss means something else differs, and the run is looked into
before it is scored.

## 6. Repeats
2 each: A, B and the base (6 runs).

## 7. Cost
Lab PC, 4 slots x 4 cores (4-quad runs flew RTF 0.21-0.27 there today): each run ~345 s of sim time (lift ~25 s,
6 laps 207 s, lap holds 15 s, two rejoins with their 10 s blends ~65 s, the unload and resize holds ~8 s, landing
~20 s), ~30 min wall; the base ~25 min. `python3 tools/remote/lab_batch.py --slots 4 --cpus 4 --wall-cap-min 60
--warm-from-config configs/experiments/x2_full_fig8_{soft,unann,base}.yaml` with each config listed twice
(interleaved). **~1 h of lab PC** (4 + 2). Laptop: none. The table's 1.15 kg cell is not written: the pair cannot hold
1.15 kg at throttle_max 0.8 (R2834 put the ring on the floor) and held it at 0.9 (R2840/R2842), so that cell needs a
second variable (Wesley's call: A on `four_rigid_ground_rig_L100_p115.sdf` at throttle_max 0.9, x1).

## 8. New code?
No change to `src/`. One new twin params file, `configs/twin/x2_chain_tip80.yaml`. Everything the chain needs exists
and has flown in parts: DETACH with `detach_unload_s` on a path (R2313), unannounced releases with the marker check
(R2821), staging and rejoins in hover and staging on a path (rejoin card E1-E5), lapped paths with lap-end holds,
WAIT_TRAJ and rejoins in the holds (S case 7, R2779). Not a new law or gate, so no critic.
What the runner and tools cannot do yet (none blocks the runs; all are scoring or design limits):
- A rejoin cannot start while the path runs (REJOIN REFUSED): rejoins only in a lap-end hold (the rescope's limit).
- A release inside a lap-end hold would cut the hold to `reconfig_hold_s` (1.5 s): the resize sets the hold, it does
  not extend it. Not used here (releases mid-lap); it matters if a later variant releases at a lap end.
- `detach_unload_s` applies to every announced DETACH of a run: one soft step per run is fine, a mixed soft / hard
  announced chain is not expressible.
- No committed scorer cuts the clock-free path error and tilt by trajectory-clock lap (S's case 7 used a scratchpad
  wrapper): ~30 lines over `x1_metrics._clock_free_err` and the planner log's `traj_t`, in `tools/`.
- `payload_tilt_envelope false` sets the trackers' ring envelope to 180 deg; there is no knob for another value (X1).

## Results (31 Oct, first pass)

Scored 31 Oct ~17:40 against the bars as frozen. No bar moved and no metric was re-windowed. R3293-R3298 flew one tree,
9f20f8b plus the dirty diff 3a165505, on lab slots 1-4 (batch log `~/mdc_overnight/lab_1031_evening2.log`). That tree is
from before the T fix, so it is not the claim table's tree. Per-run numbers: `results/analysis/x2_full_2026-10-31.json`.
Scripts: `results/analysis/x2_full_2026-10-31/`. `score_x2.py` calls `tools/s_metrics.py`, `tools/x1_metrics.py`,
`tools/release_bands.py` and `tools/n2_bands.py` unchanged, and `lap_tools.py` is the lap wrapper of section 8.

| Variant | Verdict as frozen |
|---|---|
| A, announced | FALSIFIED by one item in one repeat: R3296's fold-in tilt 5.19 deg against 5, over for 0.11 s |
| B, unannounced | FALSIFIED by T6 in both repeats: 35.47 / 35.56 s against 5.0. No loss at 80 deg, B2-B6 hold |

Every chain run flew all five stages and landed, with no drop, abort or envelope fault. A1's peak stays under the
rescope's 30 deg falsifier in both repeats, and B's peak is over the rig's 30 and 45 deg rules, as predicted.

### Validity

No run is void.

| Run | Arm | RTF | Late planner (%) | Late tracker (%) | Failed | Offset / fit (s) | Lap 1 (mm) | Lap 1 (deg) | Valid |
|---|---|---|---|---|---|---|---|---|---|
| R3293 | A | 0.234 | 0.030 | 0.012 | 0 | 2.509 / 2.509 | 8.68 | 1.14 | yes |
| R3296 | A | 0.243 | 0.030 | 0.025 | 0 | 3.217 / 3.217 | 9.31 | 1.05 | yes |
| R3294 | B | 0.228 | 0.031 | 0.019 | 0 | 7.033 / 7.033 | 9.43 | 1.09 | yes |
| R3297 | B | 0.259 | 0.031 | 0.031 | 0 | 4.243 / 4.243 | 8.88 | 1.09 | yes |
| R3295 | base | 0.253 | 0.000 | 0.015 | 0 | 8.582 / 8.582 | 9.72 | 1.08 | yes |
| R3298 | base | 0.254 | 0.000 | 0.015 | 0 | 3.131 / 3.131 | 9.67 | 1.06 | yes |
| Bar |  |  | <= 1 | <= 1 | 0 |  | <= 12.70 | <= 2.07 |  |

- The offset column is the manifest's `t_ready_sim`, which moves the node logs onto run.csv's clock. The fit column is
  the offset that best lays the planner's ring pose on run.csv's, in 10 ms steps. They agree on all six runs, so every
  window here is on one clock.
- All six: read-back as the card (planner, trackers, mocaps), every event fired, no REJOIN_REFUSED, no gap drop, no
  unconverged solve, no published tick over the residual bar.
- RTF is under S's 0.25 flag in R3293, R3296 and R3294. This card has no RTF bar, and every release there was detected.
- Each chain run logs one SLOW TICK at the last fold-in (wall clock, under 0.9 s). It is inside the late count above.

### Every bar

| Item | R3293 | R3296 | R3294 | R3297 |
|---|---|---|---|---|
| A1 / B1, square to T | holds | holds | **MISS (T6)** | **MISS (T6)** |
| A2 / B2, T to pair | holds | holds | holds | holds |
| A3 / B3, pair to T | holds | holds | holds | holds |
| A4 / B4, T to square | holds | **MISS (tilt)** | holds | holds |
| A5 / B5, lap 6 | holds | holds | holds | holds |
| A6 / B6, whole run | holds | holds | holds | holds |

| Bar | Frozen | R3293 | R3296 | R3294 | R3297 |
|---|---|---|---|---|---|
| A1 planner at n = 3 after the DETACH (s) | <= 6.0 | 3.10 | 3.10 | - | - |
| A1 UNLOAD seen, unloads cancelled | yes, 0 | yes, 0 | yes, 0 | - | - |
| A1 tilt peak from the 5 s mean (deg) | <= 30 | 28.69 | 28.42 | - | - |
| B1 loss: drop, abort, a quad past 60 deg | none | - | - | none | none |
| B1 abs peak (deg) | <= 80 | - | - | 59.89 | 58.78 |
| B1 T6, the T stage's steady tilt (s) | <= 5.0 | - | - | **35.47** | **35.56** |
| B1 T6, `x1_metrics` as coded (s) | <= 5.0 | - | - | **9.18** | **106.62** |
| 2 planner at n = 2 after the RELEASE (s) | <= 1.0 | 0.30 | 0.30 | 0.30 | 0.30 |
| 2 tilt peak from the 5 s mean (deg) | < 15 | 5.41 | 5.20 | 5.49 | 5.50 |
| 2 abs tilt peak (deg) | < 15 | 3.45 | 3.51 | 3.65 | 3.61 |
| 2 pair's lean, last 10 s (deg) | < 5 | 0.74 | 0.85 | 0.75 | 0.82 |
| 3 (a) on plate after START (s) | <= 30 | 18.48 | 19.59 | 17.49 | 18.49 |
| 3 (b) joint after on plate (s) | <= 0.5 | 0.015 | 0.008 | 0.012 | 0.014 |
| 3 (c) tilt, weld to +15 s (deg) | <= 5 | 2.23 | 2.94 | 2.23 | 2.64 |
| 3 (c) rod end through the blend (cm) | <= 2 | 1.5 | 1.3 | 1.5 | 1.4 |
| 3 (c) tension at blend's end (N), share 0.100 | +- 10 % | 0.099 | 0.099 | 0.099 | 0.100 |
| 4 (a) on plate after START (s) | <= 30 | 22.88 | 20.77 | 20.57 | 22.48 |
| 4 (b) joint after on plate (s) | <= 0.5 | 0.025 | 0.027 | 0.014 | 0.023 |
| 4 (c) tilt, weld to +15 s (deg) | <= 5 | 4.50 | **5.19** | 3.17 | 2.86 |
| 4 (c) rod end through the blend (cm) | <= 2 | 1.5 | 1.6 | 1.4 | 1.5 |
| 4 (c) tension at blend's end (N), share 2.435 | +- 10 % | 2.427 | 2.426 | 2.430 | 2.425 |
| 4 square's lean, last 10 s before LAND (deg) | <= 2 | 0.50 | 0.47 | 0.45 | 0.47 |
| 3, 4 REJOIN_REFUSED or REJOIN_ABORT | none | none | none | none | none |
| 5 lap 6 path error (mm) | <= 12.76 | 8.68 | 10.02 | 8.72 | 8.73 |
| 5 lap 6 tilt peak (deg) | <= 2.20 | 1.15 | 1.19 | 1.08 | 1.25 |
| 6 drop, abort, envelope fault | none | none | none | none | none |
| 6 every quad lands | all | all | all | all | all |
| 6 staged, worst of x and y (m) | <= 1.6 | 1.57 | 1.57 | 1.57 | 1.57 |
| 6 staged to the ring centre, least (m) | >= 0.6 | 0.68 | 0.69 | 0.68 | 0.68 |
| 6 staged to a carrier, least (m) | >= 0.8 | 1.01 | 1.01 | 1.01 | 1.01 |

- Lean is the mean tilt magnitude (rejoin card). A2's tilt peak is scored both ways, because the chain card read the
  magnitude and this card's A1 the departure from the 5 s mean. Both hold.
- Staged distances run from the joint's release to REJOIN_START, and they are horizontal, the stricter reading.
- A quad has landed when /fleet/landed came and it is disarmed and within 3 cm of rest on the last row. Drone 0 is still
  about 5 cm above rest at the fleet's disarm in all six runs, the base too.
- The base's own falsifier holds: no abort or drop, and the lap tables below show no growth lap on lap.

### Each event

Rise is the tilt vector's largest departure from its mean over the 5 s before. Dip is `x1_metrics`' form: the 5 s mean
height before, less the lowest height in the next 10 s. Times count from the event. The rejoins are read at the weld
over the bar's window, weld to +15 s, and their last column counts from the ATTACH.

| Event | Run | Rise (deg) | Abs peak (deg) | At (s) | Dip (mm) | Planner resized at (s) |
|---|---|---|---|---|---|---|
| DETACH 1 | R3293 | 28.69 | 29.25 | 3.91 | 59.4 | 3.10 |
| DETACH 1 | R3296 | 28.42 | 29.01 | 3.92 | 59.5 | 3.10 |
| RELEASE 1 | R3294 | 59.33 | 59.89 | 0.71 | 98.7 | 0.30 |
| RELEASE 1 | R3297 | 58.26 | 58.78 | 0.71 | 93.4 | 0.30 |
| RELEASE 3 | R3293 | 5.41 | 3.45 | 1.55 | 1.9 | 0.30 |
| RELEASE 3 | R3296 | 5.20 | 3.51 | 1.53 | 1.7 | 0.30 |
| RELEASE 3 | R3294 | 5.49 | 3.65 | 1.49 | 4.0 | 0.30 |
| RELEASE 3 | R3297 | 5.50 | 3.61 | 1.58 | 3.0 | 0.30 |
| ATTACH 3, weld | R3293 | 2.19 | 2.23 | 13.30 | 3.3 | 18.58 |
| ATTACH 3, weld | R3296 | 2.91 | 2.94 | 0.36 | 3.4 | 19.69 |
| ATTACH 3, weld | R3294 | 2.19 | 2.23 | 13.11 | 3.2 | 17.59 |
| ATTACH 3, weld | R3297 | 2.58 | 2.64 | 13.59 | 3.3 | 18.59 |
| ATTACH 1, weld | R3293 | 3.14 | 4.50 | 2.83 | 7.6 | 22.98 |
| ATTACH 1, weld | R3296 | 2.88 | 5.19 | 0.16 | 9.7 | 20.88 |
| ATTACH 1, weld | R3294 | 1.63 | 3.17 | 0.21 | 9.1 | 20.68 |
| ATTACH 1, weld | R3297 | 2.27 | 2.86 | 0.18 | 5.8 | 22.58 |

| Reported, square to T | R3293 | R3296 | R3294 | R3297 |
|---|---|---|---|---|
| Joint lets go after the event (s) | 3.16 | 3.18 | 0.02 | 0.03 |
| Planner at n = 3, the detection in B (s) | 3.10 | 3.10 | 0.30 | 0.30 |
| Marker: off its plate, for | - | - | 19 cm, 104 ms | 18 cm, 101 ms |
| First over 30 deg after the event (s) | - | - | 0.35 | 0.35 |
| Time over 15 deg (s) | 0.91 | 0.90 | 2.40 | 2.38 |
| Time over 30 deg (s) | 0.00 | 0.00 | 0.79 | 0.80 |
| First over 45 deg after the event (s) | - | - | 0.46 | 0.48 |
| Time over 45 deg (s) | 0.00 | 0.00 | 0.53 | 0.50 |
| Tilt at the resize (deg) | 1.01 | 0.99 | 25.99 | 25.95 |
| Tilt 5 s after the joint let go (deg) | 1.26 | 1.10 | 0.84 | 0.97 |
| T6 from the event, the T stage's steady tilt (s) | 38.58 | 38.71 | 35.47 | 35.56 |
| Largest tilt, joint +6 s to the next event (deg) | 3.08 | 2.79 | 2.74 | 3.18 |
| t_level, `x1_metrics`, not barred (s) | - | - | 4.23 | 4.21 |
| Settled, 5 s before the next event (deg) | 2.10 | 1.88 | 1.98 | 2.03 |
| Path error over 10 s (mm) | 55.31 | 55.16 | 89.69 | 88.14 |
| Quads' attitude peak, whole run, limit 60 (deg) | 17.12 | 17.10 | 17.01 | 17.02 |
| Envelope warning lines at 40 and 50 deg, no fault | 0 | 0 | 8 | 8 |

| Reported, T to pair and the rejoins | R3293 | R3296 | R3294 | R3297 |
|---|---|---|---|---|
| Detected by | marker | marker | marker | marker |
| Marker: off its plate, for | 15 cm, 103 ms | 14 cm, 102 ms | 14 cm, 101 ms | 14 cm, 101 ms |
| Roll p-p, last 10 s before the ATTACH (deg) | 2.65 | 3.28 | 2.81 | 3.12 |
| Roll p-p, worst 2 s of those 10 (deg) | 1.87 | 2.39 | 1.87 | 2.21 |
| Roll p-p, RELEASE +5 s to the ATTACH (deg) | 5.47 | 6.20 | 5.30 | 6.28 |
| S's L: rise against 8.83 (deg) | 5.41 | 5.20 | 5.49 | 5.50 |
| S's L: settled against 3 (deg) | 0.57 | 0.74 | 0.56 | 0.67 |
| S's J: bigger after ATTACH 3, 20 s not a bar (s) | 18.58 | 19.69 | 17.59 | 18.59 |
| S's J: bigger after ATTACH 1, 20 s not a bar (s) | 22.98 | 20.88 | 20.68 | 22.58 |
| Rejoin 3: least to a carrier on approach, not barred (m) | 0.75 | 0.75 | 0.75 | 0.75 |
| Rejoin 1: least to a carrier on approach, not barred (m) | 0.76 | 0.74 | 0.75 | 0.75 |

### Each lap against the base

Lap k is the planner's trajectory clock from (k-1) to k laps, moved onto run.csv's clock. The lap-end holds and the
rejoin holds stop that clock, so they fall outside every window. A release's own hold falls inside its lap.

| Lap, path error (mm) | Stage | Base R3295 / R3298 | R3293 | R3296 | R3294 | R3297 |
|---|---|---|---|---|---|---|
| 1 | square | 9.72 / 9.67 | 8.68 | 9.31 | 9.43 | 8.88 |
| 2 | square, then T from 1.5 | 9.14 / 9.37 | 35.20 | 35.05 | 51.89 | 51.02 |
| 3 | T, then pair from 2.5 | 9.46 / 9.59 | 30.76 | 30.75 | 30.73 | 30.76 |
| 4 | pair | 9.29 / 9.71 | 7.37 | 7.38 | 7.16 | 7.68 |
| 5 | T | 9.79 / 9.63 | 41.01 | 41.57 | 40.95 | 41.50 |
| 6 | square | 9.82 / 9.69 | 8.68 | 10.02 | 8.72 | 8.73 |

| Lap, tilt peak (deg) | Stage | Base R3295 / R3298 | R3293 | R3296 | R3294 | R3297 |
|---|---|---|---|---|---|---|
| 1 | square | 1.08 / 1.06 | 1.14 | 1.05 | 1.09 | 1.09 |
| 2 | square, then T from 1.5 | 1.11 / 1.09 | 29.25 | 29.01 | 59.89 | 58.78 |
| 3 | T, then pair from 2.5 | 1.04 / 1.11 | 3.45 | 3.51 | 3.65 | 3.61 |
| 4 | pair | 1.12 / 1.17 | 2.36 | 2.18 | 2.46 | 2.27 |
| 5 | T | 1.09 / 1.03 | 3.14 | 3.68 | 2.68 | 3.10 |
| 6 | square | 1.22 / 1.18 | 1.15 | 1.19 | 1.08 | 1.25 |

### The T stages

T1 runs from the first release to RELEASE 3, T2 from the first weld to the last. T1 moving starts 5 s after the joint
lets go and leaves out the lap-end hold, of which the row below takes the last 2 s. Brackets give the range over the
window. The signs are those of `docs/analysis/why_integral_code_2026-10-31.md`, where R3202 in hover settles at
+0.60 deg and -63.9 mm. `int_k_xy` 0 read back on all six.

| Roll, + = plate 3 up (deg) | R3293 | R3296 | R3294 | R3297 |
|---|---|---|---|---|
| T1 moving | +0.74 (-1.6 to +3.0) | +0.70 (-1.5 to +2.7) | +0.76 (-1.7 to +2.7) | +0.73 (-1.9 to +3.1) |
| T1 lap-end hold | +1.00 | +0.99 | +1.04 | +1.03 |
| T2 blend's end | -1.47 | -1.45 | -1.55 | -1.31 |
| T2 lap 5 | +0.89 (-2.3 to +3.1) | +0.93 (-1.9 to +3.6) | +0.93 (-1.6 to +2.5) | +0.94 (-1.6 to +3.0) |
| T2 hold, first 5 s | +1.19 | +1.19 | +1.06 | +1.09 |
| T2 hold, last 5 s | +3.17 | +2.86 | +1.72 | +2.42 |

| Off its point, < 0 = towards plate 3 (mm) | R3293 | R3296 | R3294 | R3297 |
|---|---|---|---|---|
| T1 moving | -62.4 | -62.5 | -63.4 | -62.6 |
| T1 lap-end hold | -58.3 | -58.3 | -58.0 | -58.1 |
| T2 blend's end | -33.3 | -33.5 | -33.6 | -33.7 |
| T2 lap 5 | -60.4 | -61.1 | -60.7 | -61.2 |
| T2 hold, first 5 s | -56.6 | -56.9 | -56.2 | -56.4 |
| T2 hold, last 5 s | -61.0 | -60.2 | -55.8 | -58.4 |
| T2 hold, last 5 s, along x | +16.1 | +14.9 | +8.8 | +12.5 |

| Height error, ring minus reference (mm) | R3293 | R3296 | R3294 | R3297 |
|---|---|---|---|---|
| T1 moving | -1.6 | -1.5 | -1.8 | -2.0 |
| T1 lap-end hold | -2.1 | -2.1 | -2.2 | -2.3 |
| T2 blend's end | -2.9 | -3.2 | -2.9 | -2.4 |
| T2 lap 5 | -1.6 | -1.7 | -1.8 | -1.8 |
| T2 hold, first 5 s | -2.3 | -2.4 | -1.9 | -2.0 |
| T2 hold, last 5 s | -18.9 | -16.7 | -7.2 | -12.9 |

### The path after the fold-ins

Each fold-in takes the measured ring as the path's new datum (RESIZE_INPUTS `hover_xy`). A5's error is against the same
lap's reference, so it cannot see the shift. The rows below lay the ring against lap 1's reference as well.

| Lap 6 | R3295 | R3298 | R3293 | R3296 | R3294 | R3297 |
|---|---|---|---|---|---|---|
| The bar's reading, to lap 6's own reference (mm) | 9.82 | 9.69 | 8.68 | 10.02 | 8.72 | 8.73 |
| Lap 5's reference off lap 1's (mm) | 0.2 | 0.4 | 9.4 | 8.8 | 8.8 | 9.8 |
| Lap 6's reference off lap 1's (mm) | 0.1 | 0.3 | 61.9 | 59.4 | 52.7 | 57.2 |
| Ring to lap 1's reference, RMS (mm) | 11.1 | 11.0 | 42.5 | 40.7 | 36.7 | 39.3 |
| Ring to lap 1's reference, worst (mm) | 22.2 | 22.2 | 78.0 | 77.9 | 70.3 | 74.4 |

### The last hold, on the throttle cap

| T2 hold, up to the last weld | R3293 | R3296 | R3294 | R3297 |
|---|---|---|---|---|
| Last weld, run clock (s) | 271.09 | 270.08 | 264.77 | 267.66 |
| Drone 2 on the 0.8 cap from, before the weld (s) | 12.32 | 10.57 | 6.46 | 9.06 |
| Drone 2, T2 samples at the cap (%) | 17.2 | 16.0 | 8.8 | 13.0 |
| Drone 2 throttle, lap 1 | 0.600 | 0.599 | 0.599 | 0.601 |
| Drone 0 throttle, last 5 s | 0.796 | 0.796 | 0.793 | 0.795 |
| Drone 2 pack, lap 1 (V) | 22.75 | 22.75 | 22.75 | 22.75 |
| Drone 2 pack, last 5 s (V) | 18.99 | 19.00 | 19.10 | 19.04 |
| Ring height error, last 5 s (mm) | -18.9 | -16.7 | -7.2 | -12.9 |
| Ring height error, last 0.5 s (mm) | -24.6 | -21.9 | -11.3 | -17.7 |
| Tilt, last 0.5 s (deg) | 3.75 | 3.80 | 2.20 | 2.92 |
| Weld gate: rod end off its plate (cm) | 1.5 | 1.4 | 1.3 | 1.4 |
| Weld gate: rod end speed (cm/s) | 1.9 | 1.6 | 1.4 | 1.5 |
| Rod length at the weld (m) | 1.004 | 1.008 | 0.998 | 0.995 |
| Fold-in tilt peak (deg) | 4.50 | 5.19 | 3.17 | 2.86 |
| Fold-in peak after the weld (s) | 2.83 | 0.16 | 0.21 | 0.18 |
| Fold-in time over 5 deg (s) | 0.00 | 0.11 | 0.00 | 0.00 |

The base's largest throttle in the whole run is 0.67, and no carrier in any chain run touches the cap before this hold.

### Reading

- **A4's miss.** The last weld comes with the T out of throttle. Drone 2 sits on the cap and drone 0 is just under it,
  so the ring sinks and leans for as long as the rejoiner takes to reach its plate. The later the weld, the deeper the
  sink (last table). R3296's peak is that lean plus the weld's own kick, for one short spike. R3293 reached 4.50 deg
  from the same state, so both A repeats sit near the bar. A's unload hold puts its last welds later on the same pack
  than B's, and they are the two deepest. The link from the cap to the peak is a reading of these four runs: no run has
  flown this hold without the cap.
- **B1's T6.** The T is gone by LAND, so the T's steady tilt is the mean over the last 5 s before RELEASE 3, as
  `s_metrics`' settled tilt. On a running path that tilt is not steady: the roll swings over the range in the T table
  with the path, so a band of 0.5 deg round it is met only when the path comes back to the same phase, about a lap on.
  `x1_metrics` as coded compares with the square on lap 6 and gives two unrelated numbers for like flights. Both
  readings miss, so the verdict does not depend on the reading. Neither is a recovery time. The transient itself is
  short: see the tilt at +5 s, the largest tilt from +6 s and t_level, which are reported, not barred.
- **The T.** On the path the T does what the analysis says of it in hover: near level on average with plate 3 slightly
  up, and hanging about 60 mm off its point towards plate 3. T2 at the blend's end is still on its way there, with
  plate 3 low. The pair sits close to its point: lap 4 is the run's best lap, under the base. Laps 2, 3 and 5, which
  hold the T, are the worst.
- **Lap 6.** A5 and B5 hold as frozen, and the square does fly the figure-8 well. It flies a figure-8 moved by about
  the T's offset, because the last fold-in re-zeroed the path where the T left the ring. S's card saw the same ratchet
  on the orbit. The first fold-in moves it far less, since the pair sits near its point.
- **Throttle.** The two carriers holding nearly the whole ring, on the pack model's sagged voltage, are what reach the
  cap, late in the run. It is not a validity item on this card, and it is the twin's pack model, not a rig reading.

### For Wesley

1. A4 is a miss on a valid run, so nothing is re-flown to clear it. Report it as missed. It is read again on the re-fly
   after the T fix, on the final tree. If the chain should stop reaching the cap, the config's levers are a fuller pack
   (`sim_pack_v0`) or a higher `throttle_max`. Each is a second variable and yours to choose.
2. B1's T6 cannot read a recovery on a running path. This result stays a miss. For later runs only, a new bar could be
   frozen before the re-fly. Recommended: the largest tilt from +6 s to the next event, `x1_metrics`' form, which the
   reported table already gives. The alternative is to keep T6 for the hover cell, where it came from.
3. A5 and B5 cannot see the path's moved datum. Recommended: report the ring against lap 1's reference beside the bar on
   the re-fly. A T on its point would leave little to re-zero, so this may shrink with the T fix.

### Not computed

- The rod end's distance is the planner's own figure (its BLEND OVER line). Run.csv has no rod poses, and the Gazebo
  state logs were not read, so it has no independent check here.
- Detection has no time of its own. The planner resizes in the tick that detects, and its log lines carry wall time
  only, so the resize tick stands for both.
- A1's T6 comes from `tools/release_bands.py` directly, because `x1_metrics` as coded takes plate 3's RELEASE in A.

## Bars for the later runs (FROZEN 31 Oct, Wesley, before the re-fly; the first pass stays as scored)
- B1's T6 on the path is replaced by the largest tilt from +6 s to the next event (`x1_metrics`' form; first pass read 2.74, 3.18 deg, reported, not barred; the value of the bar: at most 5 deg, the same as the fold-in bar). T6 stays for hover cells.
- Beside lap 6's bar, the ring's RMS distance from lap 1's reference is reported (first pass 36.7-42.5 mm against 11.0-11.1 mm for the base). Reported, not barred.

## Reviewer (4 Nov ~22:30, on R3586-R3591): supported, with corrections
Every barred number recomputed with the committed scorer (`results/analysis/x2_full_2026-10-31/score_x2.py`), none
more than 0.1 off the registry. One tree for all six (9f20f8b + the same patch), A and B one line apart, late ticks at
most 0.06 %, no failed solves; R3590's missing /tracker_3 read-back is covered by its params.json (drop_tilt_deg 80).
The current tree, not the final one: the claim re-flies after the freeze and the reviewer sees that table too.

| Correction | What the text must say |
|---|---|
| B1's bar | B passes the bar that replaced T6 on 31 Oct before the re-fly; its value, 5 deg, was set by Wesley on 4 Nov ~22:20 after the runs (readings 3.10, 2.66). The original T6 (at most 5.0 s) is missed in both (35.46, 35.40 s), as in the first pass. |
| The throttle cap | In all four chain runs drone 2 sits on the 0.8 cap for 6.7-10.4 s before the last weld (8.6-14.1 % of the T stage); the base never reaches it (p95 0.66); the ring sits 7-14 mm low in the last 5 s. A4 passes (3.00, 3.30 deg; R3590 4.16, 0.84 under the bar) with the T at its throttle limit: not independent of the headroom. |
| A1's label | 29.09 / 27.82 deg is the rise from the 5 s mean (the bar's quantity); the absolute peaks are 29.67 / 28.39. R3588 is 0.91 deg under 30. |
| Lap 6 | It flies a datum moved 53-58 mm: the ring to lap 1's reference is 36.6-39.7 mm RMS against the base's 10.5, 10.6. |

Reviewer's scorer and output: `results/analysis/review_x2_x8_2026-11-04/rev_x2/`.

# Card X1: the release ablation (30 Oct 2026)

Owner Wesley (decisions.md 30 Oct ~12:30: X1 is a headline added-value result; ~12:35: `resize_delay_s` approved as
twin-only code behind a switch, critic waived; the twin's ring for claims 0.86 kg plus one 1.15 kg cell). Plan row X1
of `docs/plan_rescope_2026-10-30.md`. Written 30 Oct ~12:50 AEDT. **Bars frozen before any run** (section 5): the first
X1 run (a smoke, exploratory) starts after this card is saved.

## 1. Question
Is resizing the planner's problem at the detection of an unannounced release, with the measured cable pull, what keeps
the ring up? ("how do I prove my approach has added value": the RA-L review's point 8, no arm without the resize.)

## 2. Hypothesis
- **No resize** (the planner never learns of the loss and keeps the leaver in its model) tilts and sinks the ring
  clearly more than ours: by more than 1 deg of tilt rise or 5 mm of dip on the 4-quad M1 layout (the 120 deg Y the
  release leaves), less at 8 quads (a 90 deg gap).
- **Late resize** (0.5, 1, 2 s after detection) is ordered between the two: the later, the larger the rise.
- **Planned pull** (`cable_source model`, the resize kept) rises more than ours: the 1 m Tier B gap curve had the
  measured pull cut the rise to ~2/3 (G150 17.9 vs G150p 28.4 deg, R2604-R2628).
- The paper's hand-over (network, `net_law`) is **not an arm**: as built, the network phase never runs the release
  detection (`_plan_inner` returns in the network before `_detect_tick`, which also needs `reconfig_mode ocp`), so it
  cannot meet an unannounced loss without new flight-loop code. Left to X8.

## 3. The one variable
How the planner reacts to a detected release. Everything else is the cell's base config, typed in each file.

| Arm | Change from ours | Config suffix |
|---|---|---|
| ours | none (resize at detection, measured pull) | `_ours` |
| no resize | `resize_delay_s: 999` (longer than the flight: never acted on) | `_nors` |
| late 0.5 / 1 / 2 s | `resize_delay_s: 0.5 / 1.0 / 2.0` | `_d05`, `_d10`, `_d20` |
| planned pull | `cable_source: model` (trackers fly the planner's planned pull; resize at detection) | `_plan` |

The switch (`resize_delay_s`, twin only, default `''` = 0) delays the planner's whole reaction to a detected loss
(the drop rule and the resize) by that many sim seconds; meanwhile the leaver stays in the model, keeps its slot
reference and is not re-detected. The resize's 1.5 s path hold comes with the resize (no resize has none).

Cells (conditions), `configs/experiments/x1_<cell>_<arm>.yaml`:

| Cell | Base | Release | Ring |
|---|---|---|---|
| `h4` 4 quads, hover | `s4_release9_3915_L100_it5` (R2708/R2709's cell: M1 layout, typed 330,270,210,90, RELEASE plate 9 = drone 1 at 25 s, LAND 45 s) | plate 9 | 0.86 kg |
| `f4` 4 quads, mid figure-8 | the same + `load_traj fig_8`, `fig8_size 1.0`, `traj_speed 0.2`, `traj_hold_s 5` (the C table's 4-quad figure-8, R2704/R2705); RELEASE 17.24 s after "hover hold done" = mid-lap at the centre crossing (lap 34.5 s), LAND 17.0 s after it (before any arm's lap ends) | plate 9 | 0.86 kg |
| `h8` 8 quads, hover | `s8_hover_R50_L100_it5` + `tether_tips`; RELEASE drone 1 at 30 s, LAND 50 s | drone 1 | 0.9 kg R 0.5 m (the only 8-quad ring) |
| `h4p115` 4 quads, hover, 1.15 kg (ours and no resize only) | `h4` in `four_rigid_ground_rig_3915_L100_p115.sdf` (new, from `tools/make_carry_worlds_p115.py`) with the 28 Oct ring's values (`load_mass 1.15`, `load_ixx 0.024`, `load_izz 0.049`, `attach_radius 0.204`, `attach_z 0.025`, `payload_rest_z 0.013`) | plate 9 | 1.15 kg |

Pinned in every cell: one tree (week7 + this card's code), 5 SQP iterations, `plan_fast_refs` on, the S chain's
params (`configs/twin/s_chain_pair.yaml`: detection 0.06 m, step-out 0.5 m, `drop_on_loss`, `drop_gap_deg 200`,
`min_survivors 2`, tilt drop 45 deg), `reconfig_mode ocp`, the twin's `slope` velocity filter (the default since
28 Oct; R2708/R2709 and the 8-quad table flew `ema`).

## 4. Baseline
- **Named baseline: the no-resize arm** (plan row X1); every cell's comparator is its own `_nors` pair, flown in the
  same batch on the same tree. Ours is the treatment.
- For scale, not compared: R2708/R2709 (ours in `h4`, `ema` filter): tilt rise 8.25 / 7.40 deg, abs peak 6.75 / 5.79
  deg, resized 0.29 / 0.28 s after the release, no drop. R2766/R2767 (8 quads, first release of the chain, `ema`):
  peaks 1.91 / 1.86 deg, n-1 in 0.38 / 0.37 s.

## 5. Pass / fail numbers (FROZEN 30 Oct ~12:50, before any X1 run)
Metrics (`tools/x1_metrics.py`, from run.csv, events.csv, the planner log and launch log; t_e = RELEASE):
- **Tilt rise** (deg): the tilt vector's largest departure from its mean over the 5 s before t_e, t_e to LAND
  (release_bands' delta-theta). Beside it the absolute peak tilt.
- **Dip** (mm): the ring's mean height over the 5 s before t_e minus its lowest height in t_e .. t_e + 10 s.
- **Drop**: the planner's DROP AND LAND (planner rule or the trackers' 45 deg tilt drop) or the ring under 0.3 m
  before LAND.
- **Settle** t_level (s): the first time after the peak from which the tilt vector stays within max(2 sigma_pre,
  1.0 deg) of its pre-release mean for 1 s; censored at LAND. Beside it the settled tilt (mean, last 5 s before LAND)
  and release_bands' T6 time back in band.
- **Path error** (mm): RMS of the ring's 3-D distance to its reference path over t_e .. t_e + 10 s (hover: to the
  hover point; figure-8: clock-free, across the path and height, n2_bands' method).
- Also: detection and resize times (the planner's first tick at n-1), late ticks, failed solves.

Bars ("ours_max" = the larger of ours' two repeats; arm means over 2 repeats):
- **B1 (the headline; the plan's falsifier).** Per fleet, CONFIRMED if in both no-resize repeats the tilt rise is
  > ours_max + 1.0 deg, or the dip > ours_max + 5 mm, or the ring is dropped. **FALSIFIED if on both fleets (`h4` and
  `h8`) the no-resize means are within 1.0 deg (tilt rise) and 5 mm (dip) of ours.** One fleet confirming and the other
  not: the claim holds for that fleet only, said so. `f4` and `h4p115` are scored by the same rule and reported
  (secondary cells; the falsifier reads the hover cells).
- **B2 (late resize is worse, in order).** In `h4` and `f4`: arm means of tilt rise non-decreasing ours <= 0.5 s <=
  1 s <= 2 s <= no resize, no neighbour inverted by more than 0.5 deg, and the 2 s arm > ours_max + 1.0 deg in both
  repeats. Falsified by a neighbour inversion > 0.5 deg or the 2 s mean within 1.0 deg of ours. `h8` reported.
- **B3 (the measured pull).** In `h4`: CONFIRMED if in both planned-pull repeats the tilt rise is > ours_max + 1.0 deg
  or the dip > ours_max + 5 mm; FALSIFIED if its means are within 1.0 deg and 5 mm of ours. `f4`, `h8` reported.
- **B4 (ours is safe).** Every ours run in every cell (1.15 kg included): no drop, absolute peak < 30 deg (the rig's
  tilt rule), and in the hover cells t_level <= 5.0 s. Any miss is reported as a miss of ours.
- **Validity (a void run is re-flown once).** RELEASE fired and the planner logged UNANNOUNCED DETACH within 0.5 s;
  ours / planned pull at n-1 within 0.5 s of t_e; the late arms at detection + delay within 0.2 s (LATE RESIZE line);
  no resize at n throughout (to LAND); read-back `resize_delay_s` / `cable_source` as typed; planner and tracker late
  ticks <= 1 % (s_metrics' LATE_VOID); no abort or failed solve before t_e. A drop after t_e is an outcome, not void.

**Revision 30 Oct ~13:10, before any claim run (only the three smokes had flown).** B4's settle bar is ill-posed for
the M1 layout: before the release the four carriers hold the ring ~1.5 deg tilted (pre-release tilt vector ~(0, -1.5)
deg in R2708 and the smokes), after it the three level it (settled 0.03-0.2 deg), so "back within 1 deg of the
pre-release tilt" measures the layout change, not settling (R2708 passed it at 2.48 s only because the deviation dipped
under 1.0 deg for one second). For the claim runs B4's settle reads **T6 t_back <= 5.0 s** (release_bands' time back
within max(2 sigma_pre, 0.5 deg) of the post-release steady tilt, held 1 s: the 11 Oct release rule); the frozen
t_level is still computed and its B4 outcome reported beside. B1-B3 are unchanged.

**Added 30 Oct ~13:10 (exploratory neighbour, no bar).** The smoke R2915 (no resize, `h4`) flew like ours for scale
(tilt rise 7.15 deg, dip 33.8 mm against R2708/R2709's 8.25/7.40 deg, 34.1/31.9 mm), so the measured pull may be what
replaces the resize. The 2 x 2 neighbour `x1_<cell>_nplan` (no resize **and** planned pull: `resize_delay_s 999`,
`cable_source model`) flies in `h4` and `f4`, 2 runs each; one variable against `_nors` (the pull source) and against
`_plan` (the resize). Reported, not barred. 13:45: added to `h8` too (2 runs, before any `h8` run), to finish the 2 x 2
at 8 quads.

## 6. Repeats
2 per arm and cell. Before them, one smoke each of `x1_h4_nors`, `x1_f4_nors` and `x1_h4_d20` (exploratory rows, not
in the table): the plan's risk "the no-resize arm is ill-defined (the leaver misbehaves)", and the late path's first
flight. If a smoke shows an arm ill-defined, the arm is fixed (code) and the table flies on the fixed tree; the bars
above do not change.

## 7. Cost
Lab PC (no session cap there; CLAUDE.md, 22 Oct), `tools/remote/lab_batch.py`:
- 4 quads: `h4` 6 arms x 2 = 12, `f4` 12, `h4p115` 2 arms x 2 = 4, + 3 smoke + 4 of the 2 x 2 neighbour (added
  13:10) = 35 runs (4 slots x 4 cpus, ~9 min wall each, ~1.4 h);
- 8 quads: `h8` 6 arms x 2 = 12 runs + 2 of the neighbour (2 slots x 12 cpus, ~25 min wall each, ~3 h).
- Total 49 runs (40 in the table), ~4.5 h of lab time. Laptop: none (the real-world stack may be up).

## 8. New code?
Yes, twin only, default off (Wesley 30 Oct ~12:35: approved, **critic waived** for `resize_delay_s`).
- `src/dissipative_planner/dissipative_planner/dissipative_node.py`: `resize_delay_s` (`resize_delay_value`: >= 0,
  refused on a wall-clock node); `_on_unannounced` defers to `_act_on_loss` through `_resize_due` /
  `_resize_due_tick`; the stretch and marker checks skip a leaver whose reaction is pending.
- `src/bringup/bringup/real_mode.py`: in `PLANNER_OPTIONAL`, refused outside the twin's mode:=dissipative with
  reconfig_mode ocp (the rig, attach, carry and network graphs); `src/bringup/config/common.yaml`: `resize_delay_s: ''`.
- Tests: `src/dissipative_planner/test/test_resize_delay.py` (0 acts at detection; a delay acts once, on the node
  clock, without re-detection; 999 never; the drop rule late too; negative and wall clock refused);
  `test_real_mode_launches.py::test_resize_delay_reaches_the_twin_dissipative_planner_only_and_is_refused_elsewhere`.
- Removal: it is a baseline arm, not a control law; it stays off by default. If Wesley wants no ablation code in the
  final tree after the thesis tables, the switch and its tests come out together (one commit), and the X1 rows keep
  the tree they flew.

## Addendum: the uneven cell `sqT`, square to a T (Wesley 30 Oct ~15:10; written and FROZEN ~15:20, before any sqT run)

**Question.** At 4 quads in the even M1 cell the no-resize arm came within ~1 deg and ~3 mm of ours, because the measured
pull catches the release first (with the planned pull the resize cut the dip ~150 -> ~60 mm). Does the resize matter
on its own when the old plan is badly wrong about the geometry: the square losing plate 9, which leaves a T whose
survivors' largest gap is 180 deg (the CoG on the opposite pair's line)?

**Cell.** `x1_sqT_<arm>.yaml`: `x1_h4_<arm>` in the square world `four_rigid_ground_rig_L100.sdf` (carriers typed
0,270,180,90 = plates 0/9/6/3; the 0.86 kg ring; R2249/R2318/R2319/R2823's world), unannounced RELEASE of plate 9
(drone 1) at 25 s in hover, LAND 20 s later. The same seven arms (ours, no resize, late 0.5 / 1 / 2 s, planned pull,
no resize + planned pull), 2 runs each = 14 runs. Params `configs/twin/x1_sqT.yaml` = `s_chain_pair.yaml` (the T flies
under `drop_gap_deg 200`, `min_survivors 2`) with **`drop_tilt_deg 80`**.

**Tilt limits raised, twin only, and why.** The unannounced square-to-T release peaks at 57-61 deg on 1 m cables in
the twin (R2249, R2823; 76.7-80.2 deg after a 45 deg drop at 5 iterations, R2318/R2319), so at 30 or 45 deg every arm
would be dropped and nothing could be compared. Two limits act on the ring's tilt, both in the trackers:
- `drop_tilt_deg` (the trackers' ring-tilt drop request to the planner; 30 by default, 45 in the chain params) -> **80**;
- the trackers' ring-tilt envelope `max_payload_tilt_deg` (60 deg: a fleet DISARM, harder than a drop) is switched off
  by the twin's existing launch switch `payload_tilt_envelope: false` (it sets 180; there is no launch knob for 80), so
  80 deg (the drop) is the ring's only tilt rule.
- The planner has no ring-tilt drop of its own (it drops on the trackers' /fleet/drop request, on a loss leaving fewer
  than `min_survivors`, or on a gap over `drop_gap_deg`). The quads' own 60 deg attitude envelope stays: a quad past it
  is an envelope fault (a loss, reported).

**Bars (frozen ~15:20, before any sqT run).** Metrics as section 5 (tilt rise, abs peak, dip, drop, T6 settle,
t_level, settled tilt, path error over 10 s, resize time). "Loss" = a drop (the 80 deg rule or the planner) or a fleet
abort (an envelope fault). ours_max = the larger of ours' two repeats; means over two.
- **U1 (the resize on its own; the cell's headline).** CONFIRMED if in both no-resize repeats: tilt rise >
  ours_max + 1.0 deg, or dip > ours_max + 5 mm, or settled tilt > ours_max + 1.0 deg, or a loss where ours has none.
  FALSIFIED if the no-resize means are within 1.0 deg (tilt rise), 5 mm (dip) and 1.0 deg (settled tilt) of ours and
  neither arm loses the ring. Otherwise inconclusive.
- **U2 (late resize in order).** Arm means of tilt rise non-decreasing ours <= 0.5 s <= 1 s <= 2 s <= no resize, no
  neighbour inverted by more than max(0.5 deg, 5 % of ours' mean); and the 2 s arm worse than ours in both repeats by
  U1's rule. Falsified by a larger inversion or the 2 s arm within U1's falsifying band of ours.
- **U3 (the measured pull).** Planned pull vs ours by U1's rule (confirm / falsify as U1).
- **U4 (the resize with the planned pull).** No resize + planned pull vs planned pull by U1's rule, the planned-pull arm
  as the reference (its max and means in place of ours').
- **U5 (ours recovers).** Both ours runs: no loss, abs peak <= 80 deg, T6 settle <= 5.0 s. t_level (back to the
  pre-release attitude) is reported, not barred: the T's own level is not the square's (R2249 settled at 6.7 deg).
- **Validity** as section 5, plus: the planner does not drop the T for its gap (`drop_gap_deg 200` flies it; a gap drop
  is a config error, void).

Reading caveat (stated before the runs): the twin's links are rigid hinged rods, which may exaggerate a T's roll
against the rig's slack-able cables (R2249's note).

**Cost.** 14 runs, 4 slots x 4 cpus, ~40 min of lab time, after batch B and its re-fly.

## Results (scored 30 Oct ~15:45; sections 1-8's cells; the sqT addendum's table below)

One tree for every run (week7 284ffb4 + the working tree with `resize_delay_s`; src unchanged across batches A-D, the
manifests' git_diff.patch). 49 valid runs; one void (R2952: the runner deadlocked after LAND, re-flown as R2964).
Scores: `tools/x1_metrics.py` -> `results/analysis/x1_2026-10-30.json`. Each cell's two repeats are listed in order.

| Cell | Arm | Runs | Tilt rise (deg) | Abs peak (deg) | Dip (mm) | Drop | T6 settle (s) | t_level (s) | Settled tilt (deg) | Path error 10 s (mm) | Resize after release (s) |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 4 quads hover, 0.86 kg | ours | R2918, R2934 | 5.73 / 7.16 | 4.29 / 5.62 | 29.1 / 33.7 | no | 4.0 / 3.9 | 2.6 / 2.5 | 0.05 / 0.05 | 42.2 / 44.3 | 0.38 / 0.28 |
| 4 quads hover, 0.86 kg | late 0.5 s | R2920, R2936 | 6.94 / 6.93 | 5.39 / 5.39 | 32.5 / 33.2 | no | 4.9 / 5.0 | 3.6 / 3.7 | 0.07 / 0.07 | 45.5 / 45.9 | 0.79 / 0.87 |
| 4 quads hover, 0.86 kg | late 1 s | R2921, R2937 | 6.90 / 6.80 | 5.35 / 5.25 | 32.5 / 33.1 | no | 5.2 / 5.2 | 3.7 / 3.7 | 0.04 / 0.03 | 44.5 / 44.4 | 1.38 / 1.37 |
| 4 quads hover, 0.86 kg | late 2 s | R2922, R2938 | 6.91 / 7.17 | 5.34 / 5.68 | 32.6 / 34.0 | no | 3.6 / 4.7 | - / - | 0.05 / 0.05 | 41.3 / 42.4 | 2.37 / 2.30 |
| 4 quads hover, 0.86 kg | no resize | R2919, R2935 | 6.97 / 6.74 | 5.43 / 5.17 | 32.8 / 32.7 | no | 3.7 / 3.6 | - / - | 0.20 / 0.20 | 41.6 / 41.0 | none / none |
| 4 quads hover, 0.86 kg | planned pull | R2923, R2939 | 16.73 / 16.62 | 17.18 / 17.07 | 53.3 / 52.3 | no | 10.3 / 10.4 | 13.2 / 13.2 | 1.91 / 1.92 | 41.9 / 41.7 | 0.34 / 0.34 |
| 4 quads hover, 0.86 kg | no resize + planned pull (neighbour) | R2924, R2940 | 21.15 / 20.97 | 21.58 / 21.42 | 142.3 / 142.2 | no | 6.0 / 6.0 | - / - | 18.94 / 18.97 | 304.6 / 303.5 | none / none |
| 4 quads mid figure-8, 0.86 kg | ours | R2925, R2941 | 7.63 / 7.20 | 6.48 / 6.05 | 34.3 / 31.3 | no | 12.9 / 12.8 | - / - | 3.57 / 3.68 | 48.9 / 47.8 | 0.36 / 0.36 |
| 4 quads mid figure-8, 0.86 kg | late 0.5 s | R2927, R2943 | 6.98 / 7.06 | 5.78 / 5.89 | 31.2 / 31.5 | no | 11.6 / 12.6 | - / - | 3.63 / 3.60 | 49.0 / 48.4 | 0.86 / 0.96 |
| 4 quads mid figure-8, 0.86 kg | late 1 s | R2928, R2944 | 7.01 / 6.96 | 5.85 / 5.79 | 31.7 / 32.0 | no | 12.6 / 11.8 | - / - | 3.63 / 3.67 | 48.4 / 48.1 | 1.36 / 1.36 |
| 4 quads mid figure-8, 0.86 kg | late 2 s | R2929, R2945 | 6.68 / 6.94 | 5.50 / 5.76 | 31.0 / 32.4 | no | 12.7 / 12.8 | - / - | 3.66 / 3.65 | 47.8 / 47.9 | 2.36 / 2.36 |
| 4 quads mid figure-8, 0.86 kg | no resize | R2926, R2942 | 6.75 / 6.80 | 5.52 / 5.59 | 30.7 / 30.6 | no | 12.9 / 12.9 | - / - | 3.63 / 3.66 | 46.2 / 47.4 | none / none |
| 4 quads mid figure-8, 0.86 kg | planned pull | R2930, R2946 | 18.36 / 17.78 | 18.78 / 18.21 | 65.6 / 63.9 | no | 8.0 / 7.9 | - / - | 2.63 / 2.60 | 44.7 / 44.4 | 0.36 / 0.36 |
| 4 quads mid figure-8, 0.86 kg | no resize + planned pull (neighbour) | R2931, R2947 | 21.58 / 21.45 | 21.98 / 21.85 | 154.6 / 154.7 | no | 6.2 / 6.2 | - / - | 17.74 / 17.74 | 222.1 / 222.3 | none / none |
| 8 quads hover, 0.9 kg | ours | R2950, R2957 | 1.63 / 1.55 | 1.63 / 1.54 | 15.2 / 14.5 | no | 4.5 / 4.5 | 2.8 / 2.8 | 1.06 / 1.07 | 28.1 / 28.1 | 0.37 / 0.37 |
| 8 quads hover, 0.9 kg | late 0.5 s | R2959, R2964 | 1.57 / 1.59 | 1.57 / 1.59 | 15.2 / 15.0 | no | 5.3 / 5.5 | 2.9 / 2.9 | 1.06 / 1.07 | 30.3 / 30.6 | 0.85 / 0.85 |
| 8 quads hover, 0.9 kg | late 1 s | R2953, R2960 | 1.42 / 1.53 | 1.40 / 1.54 | 13.2 / 14.8 | no | 5.5 / 5.6 | 1.4 / 1.4 | 1.06 / 1.07 | 27.8 / 28.1 | 1.37 / 1.37 |
| 8 quads hover, 0.9 kg | late 2 s | R2954, R2961 | 1.71 / 1.69 | 1.73 / 1.73 | 16.3 / 15.8 | no | 5.3 / 5.5 | 1.4 / 1.4 | 1.06 / 1.07 | 26.7 / 26.4 | 2.36 / 2.35 |
| 8 quads hover, 0.9 kg | no resize | R2951, R2958 | 1.63 / 1.66 | 1.60 / 1.65 | 14.7 / 14.6 | no | 6.1 / 6.2 | - / - | 1.19 / 1.19 | 25.7 / 25.4 | none / none |
| 8 quads hover, 0.9 kg | planned pull | R2955, R2962 | 7.21 / 6.81 | 7.21 / 6.82 | 38.3 / 35.4 | no | 3.7 / 3.5 | 3.6 / 3.5 | 0.40 / 0.39 | 33.3 / 32.0 | 0.40 / 0.41 |
| 8 quads hover, 0.9 kg | no resize + planned pull (neighbour) | R2956, R2963 | 13.40 / 13.44 | 13.41 / 13.44 | 71.3 / 71.2 | no | 7.5 / 7.4 | - / - | 13.18 / 13.18 | 86.7 / 86.7 | none / none |
| 4 quads hover, 1.15 kg | ours | R2932, R2948 | 9.27 / 9.76 | 7.95 / 8.40 | 43.9 / 46.3 | no | 5.5 / 5.5 | 3.3 / 3.3 | 0.04 / 0.06 | 46.6 / 47.7 | 0.33 / 0.34 |
| 4 quads hover, 1.15 kg | no resize | R2933, R2949 | 9.93 / 9.31 | 8.60 / 7.98 | 47.2 / 44.0 | no | 4.4 / 4.3 | - / - | 0.21 / 0.22 | 47.1 / 45.1 | none / none |

**Verdict against the frozen bars.**

| Bar | Outcome | Numbers (arm means unless stated) |
|---|---|---|
| B1 the resize keeps the ring up (no resize vs ours) | **FALSIFIED** as frozen: on both fleets the no-resize arm is within 1 deg and 5 mm of ours, and neither repeat is worse by more | h4 6.85 vs 6.44 deg, 32.8 vs 31.4 mm; h8 1.64 vs 1.59 deg, 14.7 vs 14.9 mm. Secondary cells the same: f4 6.77 vs 7.41 deg, 30.7 vs 32.8 mm; 1.15 kg 9.62 vs 9.52 deg, 45.6 vs 45.1 mm |
| B2 late resize in order, 2 s worse than ours | **FALSIFIED** in h4 and f4: no inversion over 0.5 deg, but the 2 s arm is not 1 deg worse than ours (the late arms sit with no resize) | h4 ours 6.44, 0.5 s 6.93, 1 s 6.85, 2 s 7.04, none 6.85; f4 7.41, 7.02, 6.99, 6.81, 6.77; h8 (reported) 1.59, 1.58, 1.48, 1.70, 1.64 |
| B3 the measured pull matters (planned pull vs ours) | **CONFIRMED** in h4 (both repeats); the same in f4 and h8 (reported) | h4 16.73 / 16.62 vs 5.73 / 7.16 deg, dip 53.3 / 52.3 vs 29.1 / 33.7 mm, settles 10.3 s to 1.9 deg; f4 18.4 / 17.8 vs 7.6 / 7.2 deg; h8 7.2 / 6.8 vs 1.6 / 1.6 deg |
| B4 ours is safe | Frozen rule: **holds** in every cell (no drop, abs peak <= 8.4 deg, t_level 2.5-3.3 s in the hover cells). Revised T6 settle (13:10): holds in h4 (3.9-4.0 s) and h8 (4.5 s), **misses at 1.15 kg** (5.5 / 5.5 s against 5.0) | the no-resize arm settles faster on T6 at 1.15 kg (4.3-4.4 s) |

**2 x 2 neighbour (exploratory, no bar): no resize and no measured pull together lose the level.** The ring hangs at
19.0 deg (h4), 17.7 deg (f4, last 5 s) and 13.2 deg (h8) to LAND, dips 142 / 155 / 71 mm, path error 304 / 222 / 87 mm.
Either mechanism alone keeps it near level: the measured pull alone (no resize) 0.20 / 1.19 deg settled, the resize alone
(planned pull) 1.9 / 0.4 deg.

**Reading.**
- With the measured pull the peak arrives 0.37-0.40 s after the release, at the planner's detection (0.28-0.41 s): the
  trackers' measured pull (50 Hz, on each carrier) has already caught the transient, so a resize at detection, or 0.5-2 s
  later, cannot change the peak or the dip, and does not.
- What the resize still buys with the measured pull is small and steady: settled tilt 0.05 vs 0.20 deg (h4), 1.06 vs
  1.19 deg (h8), 0.05 vs 0.21 deg (1.15 kg); and the leaver leaves the model (without it the freed quad holds its slot
  reference over the ring until the fleet's LAND).
- With the planned pull the resize is what levels the ring (settled 1.9 vs 19.0 deg in h4, 0.4 vs 13.2 deg in h8).
- So in these cells the release is held by two redundant mechanisms, the measured pull first; the thesis claim "the
  resize keeps the ring up" does not hold as worded. Claims that do hold: the measured pull cuts the release peak 2.4-4.4x (tilt rise, arm means)
  (B3), and either mechanism alone keeps a level ring where neither does not (the neighbour).
- Caveat: the twin's rigid rods and zero added delay; the rig's mocap path was ~95 ms late on 30 Oct, which slows the
  measured pull and may move the balance towards the resize (X9's question).

**Surprises.** R2924 (h4) and R2931 (f4), no resize + planned pull: a FLEET ABORT 9 s after LAND ('drone 2 disarmed unexpectedly': the freed quad, still in the planner's model, landed and disarmed outside the fleet's touchdown), after the scored window; not seen in their repeats or in any no-resize run. R2952's runner hung after a normal landing (futex wait, sim clock running, rows only in memory); R2955's
runner stopped 1.65 s after LANDED on a clock freeze (valid: the window was complete). Both 8-quad teardowns.

## Results: uneven cell sqT (scored 30 Oct ~16:15)

14 valid runs, R2965-R2978, the same tree as the main cells (batch D, 4 slots x 4 cpus). Every run's trackers read back
`drop_tilt_deg` 80 and `max_payload_tilt_deg` 180; the planner kept the T (gap 180 deg under `drop_gap_deg` 200) in
every resized run; no fleet abort. "Dropped" = the trackers' 80 deg rule dropped the ring; a dropped run's later tilt and
dip are the fall, so they are not shown. "Held under 10 deg" is a reading added after the runs (not barred): the first
time the tilt stays under 10 deg for 1 s.

| Arm | Runs | Tilt rise (deg) | Abs peak (deg) | Dip (mm) | Loss | Held under 10 deg (s) | Tilt +5 s / +10 s (deg) | Settled tilt (deg) | T6 settle (s) | Path error 10 s (mm) | Resize after release (s) |
|---|---|---|---|---|---|---|---|---|---|---|---|
| ours | R2965, R2972 | 57.11 / 56.99 | 57.11 / 57.00 | 100.2 / 99.5 | no / no | 2.76 / 2.74 | 6.1, 9.1 / 5.8, 9.6 | 8.30 / 10.52 | 5.4 / 13.4 | 100.5 / 100.2 | 0.31 / 0.31 |
| late 0.5 s | R2967, R2974 | 57.30 / 56.62 | 57.30 / 56.62 | 107.8 / 107.1 | no / no | 1.27 / 2.79 | 9.3, 9.6 / 9.0, 9.3 | 7.82 / 7.84 | 15.9 / 16.0 | 86.3 / 86.2 | 0.81 / 0.81 |
| late 1 s | R2968, R2975 | 56.44 / 56.00 | 56.45 / 55.98 | 107.9 / 108.0 | no / no | 1.25 / 1.24 | 8.6, 9.5 / 8.5, 10.5 | 9.72 / 13.07 | 6.6 / 16.7 | 85.5 / 86.8 | 1.31 / 1.32 |
| late 2 s | R2969, R2976 | 56.18 / 56.61 | 56.18 / 56.62 | 106.3 / 106.6 | no / no | 1.24 / 1.25 | 5.8, 7.6 / 6.0, 7.6 | 6.77 / 6.78 | 10.4 / 10.3 | 88.2 / 89.2 | 2.31 / 2.30 |
| no resize | R2966, R2973 | 58.06 / 56.37 | 58.06 / 56.38 | 109.9 / 108.7 | no / no | 1.26 / 1.24 | 2.2, 2.2 / 2.0, 2.2 | 2.19 / 2.19 | 4.5 / 4.4 | 89.2 / 87.6 | none / none |
| planned pull | R2970, R2977 | dropped / 79.33 | dropped / 78.74 | dropped / 114.5 | dropped, past 80 deg at +0.68 s / no | - / 2.02 | dropped / 2.5, 1.3 | dropped / 1.40 | dropped / 7.3 | dropped / 68.2 | 0.27 / 0.26 |
| no resize + planned pull | R2971, R2978 | dropped / dropped | dropped / dropped | dropped / dropped | dropped, past 80 deg at +0.62 s / dropped, past 80 deg at +0.63 s | - / - | dropped / dropped | dropped / dropped | - / - | dropped / dropped | none / none |

**Verdict against the frozen bars (U1-U5).**

| Bar | Outcome | Numbers |
|---|---|---|
| U1 the resize on its own (no resize vs ours) | **CONFIRMED as frozen, by the dip alone**: 109.9 / 108.7 mm against ours_max 100.2 + 5 | the tilt rise is the same (57.2 vs 57.1 deg means) and no resize ends **more** level (2.2 vs 8.3 / 10.5 deg) and nearer its point (88 vs 100 mm) |
| U2 late resize in order | **holds as frozen**: no inversion over 2.85 deg, and the 2 s arm dips more than ours in both repeats (106.3 / 106.6 > 105.2 mm) | tilt rise flat across the measured-pull arms, 56.0-58.1 deg |
| U3 the measured pull (planned pull vs ours) | **CONFIRMED** | planned pull: one ring dropped (past 80 deg at +0.68 s), the other peaked at 78.7 deg, dip 114.5 mm |
| U4 the resize with the planned pull | **CONFIRMED** | no resize + planned pull: both rings dropped (past 80 deg at +0.62 / +0.63 s), against one of two with the resize |
| U5 ours recovers | **MISS**: no loss and peak 57 deg (<= 80), but T6 settle 5.4 / 13.4 s > 5.0 | after the resize the T drifts to 8.3 / 10.5 deg over the next 20 s (the rider planned near 0 N; R2249 settled at 6.7 deg) |

**Reading.**
- The T's peak (56-58 deg at +0.69-0.71 s) is the same in every measured-pull arm: detection is at +0.31 s and the
  resize at detection, late or never does not change it. Only the measured pull keeps the ring under the 80 deg rule:
  with the planned pull the ring passes 80 deg in 0.62-0.68 s in three of four runs.
- The resize at detection saves ~9 mm of dip, but the T it plans is the weak point: with the plate-3 carrier planned
  near 0 N the ring hangs 7-13 deg off level, while the old square plan (no resize) holds it at 2.2 deg. In this cell
  the old plan is "wrong about the geometry" in a direction that helps.
- So the uneven cell does not show the resize mattering on its own for keeping the ring up; it shows the measured pull
  is what survives the T, and that the resized T's steady tilt is worth a look (the rider's planned share).
- Caveat as stated before the runs: rigid hinged rods may exaggerate the T's roll against the rig's cables.


## The claim re-fly on the earlier quadrotor masses (7 Nov, R4150-R4193; feed F087)
Wesley 6 Nov ~11:35: X1 re-flies on the masses of before 4 Nov (the final-mass X1, R3821-R3906 / F059 / F060, stays
recorded). 44 runs, one tree (section-5 hash 057e999eadce, diff_sha256 cc4c1a017289: bf52a8a plus the old-mass worlds,
tools and an unused X8 twin config; no src/ change), no void, every release detected in 0.27-0.38 s, nothing dropped.
Against this card's bars (set by the lab chat before any run; judged after the run, Wesley 7 Nov F086): h4, h8 B1 and B2
FALSIFIED, B4 holds; f4 B1 / B2 FALSIFIED, B4 holds; h4p115 B1 FALSIFIED, B4 misses by T6 5.4-5.5 s > 5 (the frozen t_level
rule holds); sqTnoxy U1 and U2 CONFIRMED, U5 misses in R4181 (T6 5.3 s). The resize leaves the transient unchanged and the
ring more level afterwards (four quads 0.03-0.06 vs 0.21 deg; the square to a T 0.6 vs 2.2 deg, dip 97-99 vs 107-108 mm).
34 solver switches: no first, next or in-1-s solve failed. Scores: results/analysis/x1_oldlinks_2026-11-07.json (+ _bars,
_switches). R4178's planner read-back file is missing (its run times show the 2 s delay). Written 17:35.

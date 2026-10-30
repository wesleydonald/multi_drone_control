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

## Results
(after the runs)

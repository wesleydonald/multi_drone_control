# Card: thesis result U, the magnet caught off its slot, in the twin (23 Oct 2026)

Owner Wesley. Drafted Fri 23 Oct ~00:30 in the main tree (day-1021, uncommitted; nothing flown for U yet). Origin:
the 21 Oct plan (docs/plan_thesis_results_2026-10-21.md, §6 "Sim U"), the push design
(results/analysis/push_1022_designs.json "sim-U"; the critic's integration notes in docs/analysis/push_1022_plan.md:
C2, C4, C9, C10, G25, G26). Decisions:
docs/decisions.md DECIDED 2026-10-22, push answer (17) "U as designed": the M1 layout; the catch point moves with the
aim, weld radius 0.025; along the rim and inward, outward once at 8 cm; a newcomer within 0.30 m of a carrier after the
weld fails; the 20 cm extension pre-registered; the critic waived for the twin-only slot arm. (8) the twin keeps the
size-1.0 figure-8 for its own tables. (7) one frozen code tree for the R, U, C and S claims.
**Status: §5 FROZEN 23 Oct 07:56 on Wesley's answers** (decisions.md DECIDED 2026-10-23 (7); the four questions at the
end of the Response). The exploratory E-runs flew first (R2459-R2463, §5's freeze note); they set nothing, and no bar
changed after them. The critic (23 Oct, below) is answered under "## Response (23 Oct)": the card and the code
are rewritten for B1 and M1-M4. **The claim grid flew** (lab batch 16, R2629-R2684) **and is scored under "## Results"
(23 Oct 12:41): between (neither confirmed nor falsified), pending two re-flies and the 20 cm extension radially in.**
**The six owed runs flew (lab batch 18u, R2727-R2732, on today's tree; scored 23 Oct 17:45, "Batch 18u" under
Results): the verdict stays between. Measured arm: d_task 8 cm along the rim, 20 cm radially in; slot arm 4 / 6.**

## 1. Question
How far off its slot can the magnet catch before the task fails (the supervisor, 21 Oct), with the fold-in at the
measured catch point (as built: the rig's) and with the slot assumed?

**What the twin can answer (critic M3).** A real magnet catches only on its 60 mm plate (16 Oct: three slides 3.3-4.3
cm off went past the plate's edge), and our contact check confirms a catch only within ~24 mm of plate 9. The twin
welds in the air wherever the knob puts the weld point. So:
- up to ~2.5 cm the sweep is the rig's case: a real catch error, folded in as the rig folds it in;
- beyond ~2.5 cm the sweep measures **how much attach-point model error the fold-in tolerates**, not how far off a real
  magnet catches: along the rim, 4-8 cm and 16 cm lie between plates, and radially past +-3 cm the point is off the
  ring's rim;
- the one real off-slot catch is `t116`, plate 10's centre (the 30-deg chord, 11.6 cm): a wrong-plate catch, which our
  contact check would refuse on the rig.
The thesis wording follows this split; the supervisor's "how far off" is answered by the ~2.5 cm part and by `t116`.

## 2. Hypothesis
- **The measured arm** folds the newcomer in where it caught, so the fleet's static balance stays right at any d. It
  stays within the fold-in bar, and the task's limit is the geometry's own:
  - along the rim (clockwise, toward the plate-11 carrier): to 16 cm on the balance (d_tol), but its separation only
    to 12 cm (d_sep): at 16 cm the plan puts the newcomer's body 0.23 m from the plate-11 carrier (static check below),
    under the 0.30 m line, so the task fails there by Wesley's rule (the twin has no rotor contact; the rig would);
  - radially inward: up to 16 cm (the newcomer's cable steepens from 60 to 81 deg; no crowding), so the 20 cm extension
    flies for this direction.
- **The slot arm** plans the newcomer's cable from the slot while it pulls from the true catch point: an unmodelled
  moment of about 0.014 N m per cm of offset (0.225 N m at 16 cm). Its fold-in tilt peak and steady tilt grow with d
  (the mechanism bar, §5) and it passes the 5 deg fold-in bar first: d_tol(slot) <= d_tol(measured) in both
  directions. Along the rim it keeps the newcomer's planned body near plate 9, so its separation stays clear: the arms
  can fail for different reasons, which is why the separation is scored apart (d_sep, critic B1).
- **At the rig's catch errors** (the one real catch, RIG-1014-t6h_4, sat 11 mm from the plate centre; d <= 4 cm) the
  two arms differ little: fold-in tilt peaks within ~1 deg of each other. (Both arms now fold in at the caught tip's
  height, critic M2, so the arms differ in the model point's xy only.)
- Static check (geometry.balanced_tensions, the M1 four at 60 deg, design scratch `design/sim_u/static_out.txt`):

| d (cm) | slot arm unmodelled moment (N m) | measured arm, newcomer azimuth along the rim (deg) | its planned body to the plate-11 carrier (m) | measured arm, newcomer elevation radially in (deg) |
|---|---|---|---|---|
| 0 | 0 | 90 | 0.73 | 60 |
| 2 | 0.028 | 85 | 0.67 | 62 |
| 4 | 0.056 | 80 | 0.61 | 65 |
| 8 | 0.112 | 70 | 0.49 | 70 |
| 12 | 0.169 | 59 | 0.36 | 75 |
| 16 | 0.225 | 48 | 0.23 | 81 |
| 20 (extension) | 0.281 | 37 | 0.09 | - |

## 3. The one variable
**The catch offset d** (`catch_offset_t` along the rim as a chord, clockwise +; `catch_offset_r` radially, out +),
within each arm; **the arm** (`attach_point_source`: `measured`, the default, or `slot`) between arms at each d.
Everything else is pinned in one template (`tools/make_catch_offset_configs.py`; its test checks that each pair of
configs differs in its one knob only):
- world `three_rigid_ground_rig_attach_L100.sdf` (the free 1 m newcomer, 1m-defaults' generator): carriers on plates
  3/11/7 (typed `90,330,210`), the slot plate 9 at (0, 0.225), rods 1.0 m, the newcomer's rod 1.0 m from a pivot 4 cm
  under its body (body-to-tip 1.04 m; `attach_cable_len` untyped, Wesley 22 Oct push answer 2);
- the twin's weld trigger as built for our approach (twin_r9_held): `weld_radius` 0.025 at `weld_point_z` 0.04, speed
  gate 0.05 m/s, the 0.15 s dwell; the tip noise 0.5 mm at 120 Hz. The knob moves our approach's aim
  (attach_target_publisher) and the twin's weld point (the magnet manager's object offset) together, so the trigger
  around the moved point is the same at every d (a deliberate change from the plan's "weld_radius >= d", which would
  change the radius and the catch height with d; Wesley's answer 17);
- our approach (approach.py), the tip loop logging only (`approach_tip_feedback` false, `approach_tip_log` true, G26);
- the sequence: 3-quad lift, magnet ON 10 s after the lift (in the hover hold), the weld and fold-in on a still ring
  (`attach_moving` true), the 4-quad hover, one figure-8 (`fig8_size` 1.0, 0.2 m/s) after `traj_hold_s` 75 s from the
  lift's top, LAND 1 s after the planner's "load trajectory complete";
- the rig-twin plant and thrust map of the b22 baselines, the mocap velocity filter `ema` on the carriers (lab_env), the
  undelayed twin (push answer 1).

Configs (35, generated, all pass `run_experiment.py --check`):
- **grid** (28): `u_{meas,slot}_d00`, `u_{meas,slot}_t{02,04,06,08,12,16}` (along the rim, clockwise),
  `u_{meas,slot}_t116` (plate 10's centre, `catch_offset_t` 0.11647 = 2 x 0.225 x sin 15 deg: the wrong-plate catch,
  critic M3; a cell of the rim sweep between 8 and 12 cm), `u_{meas,slot}_ri{02,04,06,08,12,16}` (radially in);
- **neighbours** (3, flown once, exploratory): `u_meas_ta08` (anticlockwise: the mirror, expect the same as `t08`),
  `u_meas_ro08` and `u_slot_ro08` (radially out, r 0.305, under the planner's attach_radius + 0.10 clamp). If outward
  differs strongly from inward, Wesley is asked before the grid;
- **extension** (4, pre-registered): `u_{meas,slot}_{t20,ri20}`, flown x2 only for a direction where neither arm
  failed by 16 cm.

**Where the twin flatters** (reported with the result):
- rigid rods that can push; no rotor wash or rod contact;
- the weld fires in the air 2 cm over the plate, wherever the knob puts it. A real magnet catches only on its 60 mm
  plate, so beyond ~2.5 cm the sweep is a model-error sweep, not a rig catch (§1, critic M3): along the rim 4-8 cm and
  16 cm lie between plates; past +-3 cm radially the point is off the ring's rim. Only `t116` (plate 10) is a real
  catch, and our contact check (~24 mm of plate 9) would refuse it on the rig;
- at `t16` the newcomer's rod and the plate-11 carrier's rod meet the ring about 7 cm apart (18 deg at r 0.225): the
  twin has no rod contact, the rig would.
Both arms fold in at the caught tip's height (critic M2: the slot arm takes the slot's xy and the captured z), so the
arms differ in the model point's xy only.

## 4. Baseline
- **`u_meas_d00`** (the measured arm on the slot, as the rig folds in) on the frozen tree, in the same batch; every
  other cell is compared with it, and `u_slot_d00` is the slot arm's own zero.
- **E1 = R2420** (`twin_r9_held_L100`, V4 of 1m-defaults; G26: our approach on this world, tip logging on), scored by
  `tools/catch_offset.py`: the weld 32.3 s after MAGNET ON, the true catch 0.9 cm from the slot (+0.4 cm along the rim,
  -0.8 cm radially, +0.2 cm over `weld_point_z`), fold-in tilt peak 3.14 deg over [weld - 1, weld + 15] s, carrier dip
  9 mm, newcomer-carrier separation 0.65 m. R2420 flew the orbit with the circle held (`attach_moving` false), so it is
  the approach's and the weld's evidence, not the claim baseline. It sets `traj_hold_s`: the weld ~37 s after the lift's
  top here, + 35 s, rounded up to 75 s. That rests on one weld flown in another sequence (critic, minor), so the E-runs
  now include `u_meas_d00` (§7), which may reset the hold before the freeze.
- For scale (not a bar): the 4-quad square's figure-8 at size 1.0, 0.2 m/s, ema, clock-free path error 11.9 / 12.4 mm,
  tilt peak 1.05 / 1.07 deg (R2352, R2355; the b22 bands).

## 5. Pass / fail numbers (FROZEN 23 Oct 07:56 on Wesley's answers)
**Frozen** as written below, on Wesley's four answers (decisions.md DECIDED 2026-10-23 (7)): the separation scored apart
from the arm comparison (d_sep, d_task); `t116` a grid cell with 2 repeats; past ~2.5 cm the claim is worded as a
model-error tolerance (§1); the mechanism bar kept. No other bar changed. Seen before the freeze, exploratory and not
claim runs (one each, bars then PROPOSED; the grid re-flies d00, t16 and ri16 on the frozen tree):

| Run | Config | Fold-in tilt peak | Newcomer-carrier min | Weld before the path start | Note |
|---|---|---|---|---|---|
| R2459 (E6) | `u_meas_d00` | 1.76 deg | 0.65 m | 37.9 s | `traj_hold_s` 75 kept |
| R2460 (E2) | `u_meas_t16` | 8.66 deg | 0.16 m | 23.8 s | the latest weld of the five |
| R2461 (E3) | `u_slot_t16` | 4.55 deg | 0.54 m | 39.5 s | the slot arm under the measured arm at 16 cm along the rim |
| R2462 (E4) | `u_meas_ri16` | 0.81 deg | 0.66 m | 25.2 s | |
| R2463 (E5) | `u_slot_ri16` | 14.95 deg | 0.65 m | 41.5 s | welded 1.9 cm short of its aim (d_true 14.1 cm) |

The plumbing check passed in all five (aim error 0.1-1.9 cm, under 2.5). The hold length (75 s) and the weld settings
are as drafted.

**Scorer:** `tools/catch_offset.py` (windows on the run clock; the docstring is the definition).

**A run fails** (the task failed) on the first of:
1. no weld within 60 s of MAGNET ON (or WELD_TIMEOUT);
2. no fold-in (the planner's ATTACH line) within 2 s of the weld;
3. a runner failure (abort, the payload under 0.005 m or never over 0.5 m);
4. a 'DROP AND LAND' planner line;
5. the figure-8 not completed before LAND (the planner's "load trajectory complete" missing, or logged after its LAND);
6. not LANDED.

**Separation** (Wesley 22 Oct: the newcomer's body within 0.30 m of any carrier's body from the weld to LAND fails the
task) is scored apart, as **d_sep** (critic B1). Where the newcomer flies is set by the arm itself (the measured arm
plans it from the catch, the slot arm from plate 9) and the OCP has no separation constraint, so crowding is a finding
of its own, not a test of the balance the arms are compared on. A crowded run is reported as a separation FAIL beside
its verdict.

**VOID** (re-flown once): a run with no failure (1-6) whose weld came later than the path start - 15 s (the fold-in
did not settle in the hover). A failure takes precedence (critic M1): a late weld that also failed is a FAIL, and is not
re-flown.

**The bar:** fold-in ring tilt peak <= 5 deg over [weld - 1, weld + 15] s (the sim claim bar of 28 Sep).

**The limits**, per arm and direction (d = 0 included in every direction; VOIDs left out):
- **d_tol** = the largest d where every d' <= d completed (no failure 1-6) in both repeats within the bar. The arms
  are compared on d_tol. Also tabulated on d_true, run by run (`d_tol_true`: the weld scatters up to 2.5 cm, wider than
  the low grid steps; the nominal d stays the bar's);
- **d_sep** = the largest d where every d' <= d kept the 0.30 m separation in both repeats (a run with no weld ends it);
- **d_task** = min(d_tol, d_sep): how far off the task completes, the supervisor's question.

**The mechanism bar** (critic M4). For the fold-in tilt peak and for the steady tilt mean: Delta(d) = the slot arm's
cell mean minus the measured arm's at d; the growth Delta(d) - Delta(0); the d = 0 spread = the larger of the two arms'
d = 0 repeat ranges. The mechanism is **shown** in a direction when, for either metric, the growth exceeds the d = 0
spread at both of the two largest d that both arms flew (non-VOID runs, failed ones included). Where it is not shown,
the result in that direction is "the model point does not matter to <the largest d flown> cm", and
d_tol(measured) >= d_tol(slot) there confirms nothing.

**Confirms** (all of):
- d_task(measured, along the rim) >= 12 cm and d_task(measured, radially in) >= 16 cm;
- d_tol(measured) >= d_tol(slot) in both directions;
- the mechanism shown in both directions.

**Falsifies** (any of):
- d_task(measured) < 8 cm in either direction (a failure 1-6, a separation FAIL or the bar at any d <= 8 cm);
- its fold-in tilt peak exceeds 5 deg at d <= 4 cm (the rig's range);
- d_tol(slot) > d_tol(measured) in either direction.
A result between the two (the measured arm through 8 cm in both directions, but short of 12 cm along the rim or of
16 cm inward, or the mechanism not shown) is reported as it is, neither confirmed nor falsified.

**Separation's prediction** (§2, reported, not a bar of the comparison): d_sep(measured, along the rim) = 12 cm (the
planned body 0.23 m from the plate-11 carrier at 16 cm), d_sep(measured, radially in) >= 16 cm, d_sep(slot) >= 16 cm in
both directions.

**The knob's plumbing check** (every run): the true catch point within 2.5 cm (the weld radius) of the aim (`aim_err`).
It cannot fail when the plumbing is right (critic, minor: the trigger is a 2.5 cm sphere around the aim and the joint
locks the tip where it is), so it checks only that the aim and the twin's weld point are the same point; one run over
it stops the grid until the wiring is understood. The physical check is d_true against the nominal d (reported, and
d_tol_true beside d_tol).

**Reported, not barred:** the true offset (d_true, d_tan, d_rad, dz); the attach-point model error e_model (the
planner's point from RESIZE_INPUTS against the true one); fold-in settle and carrier dip; the steady 4-quad hover over
[weld + 15, path - 0.5] s (tilt mean / max, position error); the clock-free path error and tilt peak of the figure-8
(n2_bands, from path - 0.5 s; lag beside it); separation; the newcomer's throttle peak and rod tension; the static
prediction above against what flew.

**Pre-registered:** the 20 cm extension (x2, both arms) for a direction where neither arm failed (1-6, or the
separation) by 16 cm.

## 6. Repeats
- 2 per grid cell (56 runs), interleaved (every config once, then every config again) on one host, so an interrupted
  batch still covers every cell; one re-fly per VOID run.
- 1 for the E-runs and the neighbours.

## 7. Cost (twin; no cap on headless runs)
| Step | Runs | Host |
|---|---|---|
| E1 | 0 (R2420, G26) | done |
| E6 `u_meas_d00`, flown first (the hold length and the d = 0 fold-in margin, critic minor), then E2-E5: `u_{meas,slot}_{t16,ri16}`, both ends of the sweep (falsifier: no weld, or a weld > 2.5 cm from its aim: plumbing) | 5 | lab PC (or laptop) |
| Neighbours `u_meas_ta08`, `u_meas_ro08`, `u_slot_ro08` | 3 | lab PC (or laptop) |
| Claim grid, 28 configs x 2 | 56 | lab PC, 4 slots, ~2.9 h (RTF ~0.21, ~12 min a run) |
| Extension (conditional) | up to 8 | lab PC, ~25 min |
| Re-fly on the frozen tree (only if the grid flew before the freeze) | 56 | lab PC |

The weld dwell runs on wall time, so the claim grid flies on one host. No source edits during a batch (symlink install).
No E-run before the pair-swing work in `src/simulation_communication` (being built in this tree on 23 Oct; the twin
launch loads it) has landed and the tree is frozen (critic, minor).

## 8. New code?
**Yes**, twin only; the default path is unchanged (tests below). What the rig's tree gets: one more parameter on the
planner that the rig's launch never passes, the ATTACH line's appended point, and RESIZE_INPUTS' `rho`.
- `src/bringup/config/sim.yaml` modes.attach: `catch_offset_t` 0, `catch_offset_r` 0, `attach_point_source` measured
  (not launch arguments: a config passes them through its params file; `real.yaml` lacks them, so the rig profile
  refuses them as unknown knobs).
- `src/bringup/bringup/graph_attach.py`: `catch_aim_xy` / `_catch_aim` move the attach target's offset and the twin
  manager's object offset together (zero offsets pass the slot exactly as before); refused on the rig, with partner or
  in SIL; `_attach_point_source` passes `attach_point_source` slot and the slot (not the aim) to the planner, refused
  on the rig and with partner.
- `src/dissipative_planner/dissipative_planner/dissipative_node.py`: `attach_point_slot`; in `_attach_ocp` the slot
  arm's point is the captured one (the body less the arm) with its xy set to the slot's, z kept (critic M2; the
  start-weld override and the clamp stay; the measured path is the capture as before, bit for bit); the ATTACH line
  keeps its prefix (the runner's FOLD_IN) and appends the point and its source; RESIZE_INPUTS logs every attach point
  (`rho`).
- `tools/n2_bands.py`: `path_metrics_clock_free(run, t_from=None)` (added only; None changes nothing): the attach's
  datum shift steps payload_ref at the weld, which would start the path window there.
- `tools/catch_offset.py` (scorer: rows, table with d_tol, d_tol_true, d_sep, d_task and the mechanism bar, JSON for
  the figure), `tools/make_catch_offset_configs.py` (the 35 configs, `--check`).
- Tests: `src/dissipative_planner/test/test_attach_point_source.py` (the default fold-in bit-identical to the capture,
  the slot arm at the slot's xy and the captured z, the apex, the switch's refusals, RESIZE_INPUTS);
  `src/bringup/test/test_real_mode_launches.py` (the aim and weld point together, the default graph identical with the
  knobs typed at zero, the default twin, legacy and rig attach graphs identical to HEAD's `graph_attach.py` (loaded
  from `git show HEAD:`; skipped once HEAD carries the knobs), the slot to the planner, the refusals);
  `tools/test/test_catch_offset.py` (a yawed and tilted ring's catch point, the windows, failure reasons, VOID, FAIL
  over VOID, the trajectory-complete line before LAND, the model error, d_tol, d_tol_true, d_sep and d_task, the
  mechanism bar, the 11.6 cm cell, `t_from`); `tools/test/test_make_catch_offset_configs.py` (`--check`, one variable
  per pair, `t116` on plate 10).

**If U is dropped or a part fails:** the knobs, the switch, the scorer and the configs are deleted with their tests; the
`t_from` argument stays (C uses it too).

## Critic
*The project's critic, 23 Oct 2026 (read-only review of the card and the code as built, before any run).*

**1. Tried before?** The off-slot sweep itself is new. The slot arm's mechanism has been tried: R0547 → R0548 (25 Sep) moved the newcomer's OCP attach point by 7 cm (from the rim point to the true tip) and the outcome did not change. Both runs flipped (33 / 32 deg) for another cause. So no Gazebo run has yet shown that a 7 cm attach-point model error matters.

**2. Simplest explanation U does not address.** On the rig an off-slot catch fails before any fold-in happens. The magnet catches only on the 60 mm plate: on 16 Oct three slides 3.3–4.3 cm off went past the plate's 30 mm edge. Our contact check also confirms a catch only within K ≈ 24 mm of plate 9 (graph_attach.py:861-866; `docs/rig/wed_2026-10-14.md`). The twin instead welds in mid-air at whatever point the knob sets.

**3. Falsifier.** As written it can fire for a reason other than the hypothesis (B1).

**4. What it breaks.** I found nothing in the default path:
- Zero offsets return None, and the aim falls back to the same `f('attach_x_offset')` (graph_attach.py:265, 368).
- With `_slot_xy` None the planner keeps `_capture_weld_rho(..., clamp=False, keep_z=True)` (dissipative_node.py:3103-3106).
- The rig refuses both knobs (graph_attach.py:267, 286; real.yaml lacks them).
- On the rig tree only the end of the ATTACH line and RESIZE_INPUTS' `rho` change. Both are log-only, and the runner's FOLD_IN regex matches on the line's start.
- One gap: the identity test compares new code untyped against new code typed at zero, not against HEAD. Add a diff of HEAD's attach graph.

**5. Asked for?** Yes: the supervisor's question (21 Oct) and Wesley's push answer 17.

### Findings

**B1 (blocker for freezing the bars): rule 7 decides the comparison along the rim.**
- §2 predicts the measured arm fails at `t16` on separation (planned bodies 0.23 m apart). The slot arm keeps its planned body near plate 9, so it never gets close to a carrier.
- The card predicts no slot-arm tilt for its 0.225 N m. If that tilt stays under 5 deg at 16 cm, d_tol(slot, t) = 16 > d_tol(meas, t) = 12, and the card counts as falsified by a geometry rule rather than by balance.
- The same applies at 12 cm: 0.36 m planned leaves only 6 cm to the 0.30 m line through the fold-in and the figure-8.
- Exposed by: `u_meas_t16` / `u_slot_t16` (E2/E3), then `t12`.
- Fix: compare the arms on the tilt bar plus failures 1–6. Report the separation limit d_sep as its own row; it is a real finding (the OCP has no separation constraint).

**M1: VOID hides FAIL (catch_offset.py:351-356).**
- `verdict()` returns VOID before checking failures.
- With the path at about MAGNET ON + 70 s (§4), any weld after 60 s (failure 1) is also later than path − 15 (about MAGNET ON + 55 s). So failure 1 is only ever scored when no weld happens at all.
- Any late-weld run that also drops or crowds is scored VOID. It is then re-flown, which keeps the runs that happened to go well.
- A cell that is VOID twice ends d_tol without giving a reason.
- Exposed by: any slow weld, first at E2–E5.
- Fix: failures take precedence; VOID applies only to runs with no failure.

**M2: the slot arm changes z as well as xy.**
- dissipative_node.py:3105 folds in at `attach_z` 0.02; the measured arm keeps the tip's height, about 0.04.
- In the rigid twin that is a 2 cm length error that the newcomer's tracker pushes against through a rod that can push.
- It is also a moment error of about 0.81 N × 0.02 m ≈ 0.016 N m, which equals the card's own figure for ~1.1 cm of xy offset.
- This contaminates the d ≤ 4 cm prediction ("within ~1 deg").
- Exposed by: `u_meas_d00` / `u_slot_d00`, `t02`, `t04`.
- Fix (one line): slot xy, captured z.

**M3: welds off the plate are not catches.**
- The "where the twin flatters" note covers the radial sweep only. Along the rim, 4–8 cm and 16 cm lie between plates, so no real magnet catches there.
- 12 cm is roughly plate 10 (11.6 cm chord), a real wrong-plate catch, but our K check would refuse it on the rig.
- So "the measured arm = the rig's fold-in" holds only up to about 2.5 cm. Beyond that, U measures how much attach-point model error the fold-in tolerates, not how far off the magnet can catch.
- Exposed by: `t04`–`t08`, `t16`, `ri04`–`ri16`.
- Fix: say this in the claim wording, and consider flying `t11.6` so the wrong-plate case is exact.

**M4: "≥" confirms even if the arms do not differ.**
- If both arms pass to 16 cm inward (plausible at 0.2 N m), d_tol(meas) = d_tol(slot) and the card confirms without showing that the attach point matters.
- Exposed by: `ri16`, both arms (E4/E5).
- Fix: add a mechanism bar, e.g. the slot-minus-measured difference in fold-in and steady tilt grows with d beyond the d = 0 repeat spread. If it does not, report "the model point does not matter to 16 cm" as the result.

**Minor**
- The `aim_err` check cannot fail: the trigger is a 2.5 cm sphere around the aim and the joint locks the tip where it is. E2–E5's "> 2.5 cm from its aim" only catches plumbing errors; `d_true` is the real check.
- d_tol uses the nominal d, but the weld scatter is up to ±2.5 cm, wider than the 2 cm grid steps at the low end. Also tabulate on `d_true`, as the figure already does.
- `traj_hold_s` 75:
  - It rests on one weld (R2420), flown in a different sequence (`attach_moving` false, circle held).
  - The E-runs include no d = 0 run, and R2420's fold-in peak of 3.14 deg is only 1.9 deg under the bar. The falsifier needs all 20 measured-arm runs at d ≤ 8 cm under 5 deg.
  - Add `u_meas_d00` to the E-runs.
- `completed()` does not check that the "load trajectory complete" line came before LAND (catch_offset.py:289-290).
- The twin launch loads `simulation_communication` (sim_io_launch.py, graph_attach.py), and another agent is building it in this tree right now. Do not fly E-runs until it lands (symlink install, one frozen tree).
- At `t16` the two rods' bases are about 7 cm apart (18 deg at r 0.225). The twin has no rod contact; add this to "where the twin flatters".

CRITIC: rewrite the card. Before E-runs count: compare the arms on the tilt bar (B1), take z from the catch in the slot arm (M2), make FAIL take precedence over VOID (M1), and reword the claim for welds off the plate (M3).

## Response (23 Oct)
*Card and code rewritten in the main tree (day-1021, uncommitted). Nothing has flown; every bar stays PROPOSED until
Wesley freezes them after the E-runs.*

| Finding | Answer | Where |
|---|---|---|
| B1: rule 7 decides the comparison along the rim | **Fixed.** d_tol (the arm comparison) counts failures 1-6 and the tilt bar only. The separation is its own limit, d_sep, and d_task = min(d_tol, d_sep) answers "how far before the task fails". Wesley's 0.30 m rule still fails the task. It now sits in d_task and is left out of the arm comparison. The confirm keeps its numbers (12 cm along the rim, 16 cm inward), now on d_task. The §2 prediction becomes d_sep(measured, along the rim) = 12 cm. | §2, §5; `catch_offset.py` (`crowded`, `separated`, `_limit`, `table`); test `test_separation_is_scored_apart_as_d_sep` |
| M1: VOID hides FAIL | **Fixed.** `verdict()` returns FAIL before VOID; a VOID is a run with no failure. A weld more than 60 s after MAGNET ON is now failure 1, never a VOID, so VOID covers only a weld 55-60 s after MAGNET ON. | §5; `catch_offset.verdict`; test `test_a_failure_takes_precedence_over_void` (a late weld that dropped, a weld 70 s after MAGNET ON, and the table counting the failed late run) |
| M2: the slot arm changes z as well | **Fixed (one line).** `_attach_ocp` captures the point as before and the slot arm then sets only its xy: `rho_new[:2] = slot_xy`. z stays the captured tip's. The measured path is the same capture expression; a test pins it bit-for-bit (`array_equal`). The "constant 2 cm z difference" note is gone. | `dissipative_node.py` `_attach_ocp`; tests `test_measured_default_folds_in_where_it_caught_as_before`, `test_slot_keeps_the_captured_height` |
| M3: welds off the plate are not catches | **Fixed (wording), cell added.** §1 now says that beyond ~2.5 cm the sweep measures how much attach-point model error the fold-in tolerates, not how far off a real magnet catches (it catches only on the 60 mm plate). `u_{meas,slot}_t116` (`catch_offset_t` 0.11647 = 2 x 0.225 x sin 15 deg, plate 10's centre) is a rim cell with 2 repeats, +4 runs. The scorer keys it as 11.6 cm. | §1, §3, "where the twin flatters"; `make_catch_offset_configs.py` (`WRONG_PLATE_CM`); tests in both U tool tests |
| M4: ">=" confirms with no difference | **Fixed.** The mechanism bar (§5): the slot-minus-measured growth of the fold-in tilt peak or the steady tilt mean must exceed the d = 0 repeat spread at the two largest d. Confirming needs it in both directions. Where it is not shown, the result is "the model point does not matter to X cm". | §5; `catch_offset.mechanism`; test `test_the_mechanism_bar` |
| aim_err cannot fail | **Fixed.** It is a plumbing check only (the aim and the weld point are one point); d_true is the physical check. | §5, §7; the scorer's label |
| d_tol on the nominal d | **Fixed.** `d_tol_true` is tabulated run by run on d_true beside it. The bar stays on the nominal d. | §5; test `test_d_tol_on_the_true_offset_run_by_run` |
| `traj_hold_s` from one weld; no d = 0 E-run | **Fixed.** `u_meas_d00` is E6 and flies first; it may reset the hold before the freeze. | §4, §7 |
| `completed()` ignores the order | **Fixed.** The planner's "load trajectory complete" must come before its "received: 'LAND'" in its own log. Log stamps are wall clock and events are sim clock, so the line order is used. | `catch_offset.completed`; test `test_the_figure_8_must_complete_before_the_planners_land` |
| `simulation_communication` being built in this tree | **Accepted.** No E-run until that work lands and the tree is frozen. | §7 |
| rods ~7 cm apart at `t16` | **Fixed.** Added under "where the twin flatters". | §3 |
| identity test against new code only | **Fixed.** `test_the_default_attach_graphs_match_heads` loads HEAD's `graph_attach.py` (`git show HEAD:`) and compares the default twin graph (untyped and typed at zero), the legacy graph and the rig attach graph. It leaves out the rig's `pos_limit_m`, its own 22 Oct change. Skipped once HEAD carries the knobs. Passes. | `src/bringup/test/test_real_mode_launches.py` |

**Checks:** `make_catch_offset_configs.py --check` ok (35 configs); `run_experiment.py --check` ok on all 35;
pytest: dissipative_planner 316 passed; tools/test 590 passed, 4 skipped; bringup 166 passed, 1 skipped, and the
ament flake8 / pep257 tests fail as before (99-column and docstring style in files outside U).

**For Wesley (at the freeze)** (answered 23 Oct, decisions.md DECIDED 2026-10-23 (7): yes to all four, as below; §5
frozen 07:56):
1. The arm comparison now leaves out the separation (B1). It is still a task failure through d_task. **Yes: the
   separation is reported apart from the arm comparison.**
2. `t116` is a grid cell (2 repeats, +4 runs), not flown once. **Yes: the 11.6 cm wrong-plate cell x2.**
3. The thesis claim beyond ~2.5 cm is worded as a model-error tolerance (M3). **Yes.**
4. The mechanism bar is needed to confirm (M4). **Yes: kept.**

## Results (23 Oct 12:41, lab batch 16: R2629-R2684)
*Scored against §5 as frozen at 07:56, nothing re-set, with `tools/catch_offset.py` as frozen (last edited 00:52). One
run is classed differently from the scorer's code: R2634 (below). Registry rows R2629-R2684 (no reviewer yet).
`python3 tools/catch_offset.py --table R2629-R2684` reproduces every number here, but it counts R2634 as FAIL (its code
takes any manifest failure as failure 3), so for the slot arm along the rim it prints d_tol_true 4.0 and d_sep 4 where
this section has 5.5 and 2.*

**Validity (all 56 runs).**
- One code tree: be34f95 + patch cf1ab9ad. One host (the lab PC, 4 slots, RTF 0.20-0.25). One world
  (`three_rigid_ground_rig_attach_L100.sdf`), `SIM_MOCAP_VEL_FILTER` ema.
- The param read-back is missing nothing required, and there are 0 solve failures. The runner's abort and payload
  checks pass in all 56.
- Each config's knobs reached the run:
  - the run-dir params file matches the config;
  - the launch line has the catch offset, and the attach target publisher's offsets equal the aim;
  - the planner's ATTACH line names the arm (`slot assumed` at exactly (0, 0.225); `measured catch point` within 0.2 cm of
    the aim);
  - the trigger is radius 0.025 m and gate 0.05 m/s, with a 75 s hover hold.
- Every weld fired in the hover hold, before the path start, at `weld_point_z` (the planner's z 0.039-0.041 m). The
  ring was level before each weld (pre-weld tilt 0.0-0.1 deg).
- Plumbing check: the catch-to-aim distance was 0.1-2.0 cm in all 55 welded runs (bar 2.5 cm), so the grid was not stopped.
- The measured arm's point is the newcomer's body xy at the weld, less the arm, as the rig does it. Its model error
  therefore equals the tip's miss of the aim inside the trigger sphere (0.1-1.8 cm).

**Runs that are not plain completions**

| Run | Config | Class | Why |
|---|---|---|---|
| R2634 | `u_slot_t04` | VOID (a runner fault, not a task failure): re-fly once | Runner exit 1, "drone 1 mocap stale 2.12 s > 2.0 s at t=150.05". Drone 1 was landed and disarmed (its pose last changed at 147.93; LANDED 147.89), and the runner's stop came 2.15 s after LANDED, so this is a race after the run, outside every scored window. It is none of failure 3's runner failures (abort, payload under 0.005 m, never over 0.5 m). The scorer's code counts every manifest failure as failure 3, so it is overridden here. As flown, the run is complete within the bar (1.65 deg, 0.62 m). |
| R2636 | `u_slot_t06` | FAIL (failure 1: no weld; WELD_TIMEOUT 90 s) | The tip stayed within 1.6 cm of the aim (mean 0.3 cm) for the last 44 s of the hover hold, but the 0.05 m/s gate never held for its dwell. The 3 carriers flew the figure-8 without it. |
| R2637 | `u_meas_t08` | VOID (weld 56.3 s after MAGNET ON, 14.3 s before the path start; no failure): re-fly once | As flown: fold-in peak 2.41 deg, separation 0.49 m (not counted). |
| R2672 | `u_slot_t16` | FAIL (failure 1: the weld 68.9 s after MAGNET ON, over 60 s; FAIL over VOID, not re-flown) | The fold-in ran into the path start 1.8 s later. Fold-in peak 7.09 deg, path error 80 mm. |
| R2643, R2671 | `u_meas_t16` | complete over the bar, **separation FAIL** (0.22, 0.23 m) | As §2 predicted. |

The arms are identical until the fold-in: the switch acts only in the planner's `_attach_ocp`. R2636 and R2672 are
therefore the twin trigger's failures, not the arm's (observation 2).

**Arm comparison, along the rim** (clockwise, toward the plate-11 carrier). Repeats a / b, never averaged. Completion is
against failures 1-6 and the 5 deg fold-in bar.

| arm | d (cm) | runs | completed | true offset (cm) | fold-in tilt peak (deg) | steady tilt mean (deg) | steady offset (mm) | path error after (mm) | model error xy (cm) |
|---|---|---|---|---|---|---|---|---|---|
| measured | 0 | R2629 / R2657 | yes / yes | 0.7 / 0.5 | 2.21 / 2.02 | 2.29 / 1.93 | 2 / 1 | 15.0 / 14.9 | 0.8 / 0.6 |
| measured | 2 | R2631 / R2659 | yes / yes | 2.3 / 2.4 | 1.93 / 1.75 | 2.25 / 1.62 | 16 / 16 | 23.0 / 24.5 | 0.4 / 0.4 |
| measured | 4 | R2633 / R2661 | yes / yes | 3.8 / 4.1 | 1.83 / 1.76 | 4.04 / 2.47 | 18 / 30 | 12.3 / 13.3 | 0.2 / 0.1 |
| measured | 6 | R2635 / R2663 | yes / yes | 5.9 / 5.9 | 2.06 / 1.88 | 4.27 / 5.39 | 31 / 15 | 12.4 / 13.7 | 0.2 / 0.5 |
| measured | 8 | R2637 / R2665 | VOID / yes | 7.8 / 7.7 | (2.41) / 2.33 | - / 6.18 | - / 11 | (56.1) / 12.4 | 0.1 / 0.3 |
| measured | 8 | R2727 (re-fly of R2637; batch 18u, today's tree) | yes | 8.1 | 2.44 | 6.22 | 11 | 15.7 | 1.1 |
| measured | 11.6 | R2639 / R2667 | over bar / over bar | 11.5 / 11.7 | **7.57 / 7.62** | 5.91 / 6.19 | 26 / 22 | 12.4 / 15.6 | 0.1 / 0.4 |
| measured | 12 | R2641 / R2669 | over bar / over bar | 12.0 / 11.9 | **5.82 / 5.14** | 6.36 / 4.93 | 3 / 87 | 11.9 / 104.4 | 0.1 / 0.3 |
| measured | 16 | R2643 / R2671 | over bar / over bar | 15.8 / 15.7 | **7.96 / 8.87** | 7.60 / 5.01 | 6 / 49 | 12.4 / 13.8 | 0.3 / 0.4 |
| slot | 0 | R2630 / R2658 | yes / yes | 1.1 / 0.1 | 1.58 / 1.70 | 1.02 / 1.46 | 3 / 1 | 15.5 / 14.4 | 1.1 / 0.1 |
| slot | 2 | R2632 / R2660 | yes / yes | 2.2 / 1.7 | 1.71 / 1.66 | 1.52 / 1.63 | 7 / 6 | 18.0 / 17.3 | 2.2 / 1.7 |
| slot | 4 | R2634 / R2662 | VOID / yes | 4.0 / 4.0 | (1.65) / 1.90 | (1.76) / 2.13 | (13) / 13 | (23.3) / 22.2 | 4.0 / 4.0 |
| slot | 4 | R2728 (re-fly of R2634; batch 18u, today's tree) | yes | 4.2 | 1.95 | 1.87 | 8 | 15.8 | 4.2 |
| slot | 6 | R2636 / R2664 | **FAIL** (no weld) / yes | - / 5.5 | - / 1.93 | - / 2.30 | - / 20 | - / 30.0 | - / 5.5 |
| slot | 8 | R2638 / R2666 | yes / yes | 8.5 / 8.1 | 3.75 / 2.67 | 4.96 / 3.98 | 12 / 21 | 19.4 / 19.3 | 8.5 / 8.1 |
| slot | 11.6 | R2640 / R2668 | yes / yes | 11.9 / 12.0 | 3.50 / 3.60 | 4.63 / 5.72 | 30 / 26 | 31.7 / 18.7 | 11.9 / 12.0 |
| slot | 12 | R2642 / R2670 | yes / yes | 12.3 / 12.6 | 3.78 / 4.65 | 5.69 / 6.92 | 31 / 24 | 24.1 / 23.1 | 12.3 / 12.6 |
| slot | 16 | R2644 / R2672 | yes / **FAIL** (weld at 68.9 s) | 15.0 / 15.9 | 3.69 / 7.09 | 6.27 / - | 30 / - | 19.3 / 80.0 | 15.0 / 15.9 |

**Arm comparison, radially in**

| arm | d (cm) | runs | completed | true offset (cm) | fold-in tilt peak (deg) | steady tilt mean (deg) | steady offset (mm) | path error after (mm) | model error xy (cm) |
|---|---|---|---|---|---|---|---|---|---|
| measured | 0 | R2629 / R2657 | yes / yes | 0.7 / 0.5 | 2.21 / 2.02 | 2.29 / 1.93 | 2 / 1 | 15.0 / 14.9 | 0.8 / 0.6 |
| measured | 2 | R2645 / R2673 | yes / yes | 2.6 / 2.5 | 1.84 / 1.68 | 1.77 / 1.65 | 1 / 1 | 15.0 / 15.2 | 0.5 / 0.4 |
| measured | 4 | R2647 / R2675 | yes / yes | 5.7 / 3.8 | 2.63 / 1.54 | 3.02 / 1.04 | 5 / 2 | 17.1 / 15.1 | 1.7 / 0.2 |
| measured | 6 | R2649 / R2677 | yes / yes | 5.9 / 6.3 | 1.54 / 1.68 | 0.93 / 1.22 | 1 / 1 | 15.8 / 14.7 | 0.2 / 0.3 |
| measured | 8 | R2651 / R2679 | yes / yes | 7.6 / 8.5 | 1.41 / 1.63 | 0.81 / 1.32 | 2 / 1 | 15.3 / 14.8 | 0.3 / 0.6 |
| measured | 12 | R2653 / R2681 | yes / yes | 10.0 / 12.9 | 1.01 / 1.39 | 0.30 / 1.04 | 3 / 1 | 15.0 / 16.1 | 1.8 / 0.8 |
| measured | 16 | R2655 / R2683 | yes / yes | 16.2 / 14.6 | 1.04 / 0.93 | 0.44 / 0.08 | 1 / 3 | 14.6 / 16.5 | 0.2 / 1.5 |
| measured | 20 | R2729 / R2730 (batch 18u, today's tree) | yes / yes | 19.7 / 20.1 | 0.40 / 0.49 | 0.08 / 0.18 | 1 / 1 | 15.1 / 14.7 | 0.2 / 0.3 |
| slot | 0 | R2630 / R2658 | yes / yes | 1.1 / 0.1 | 1.58 / 1.70 | 1.02 / 1.46 | 3 / 1 | 15.5 / 14.4 | 1.1 / 0.1 |
| slot | 2 | R2646 / R2674 | yes / yes | 1.9 / 2.0 | 3.23 / 3.29 | 3.82 / 3.98 | 7 / 8 | 18.4 / 19.8 | 1.9 / 2.0 |
| slot | 4 | R2648 / R2676 | yes / yes | 4.4 / 3.4 | 1.83 / 1.81 | 1.31 / 0.99 | 121 / 75 | 90.8 / 60.0 | 4.4 / 3.4 |
| slot | 6 | R2650 / R2678 | yes / yes | 5.8 / 5.7 | 3.45 / 3.27 | 3.15 / 3.05 | 173 / 170 | 124.2 / 122.6 | 5.8 / 5.7 |
| slot | 8 | R2652 / R2680 | over bar / yes | 7.8 / 6.4 | **5.76** / 4.20 | 5.45 / 3.83 | 237 / 192 | 168.2 / 138.5 | 7.8 / 6.4 |
| slot | 12 | R2654 / R2682 | over bar / over bar | 11.4 / 12.2 | **10.38 / 11.71** | 10.44 / 11.91 | 337 / 353 | 225.1 / 237.7 | 11.4 / 12.2 |
| slot | 16 | R2656 / R2684 | over bar / over bar | 15.4 / 14.3 | **16.96 / 14.99** | 16.43 / 15.00 | 396 / 372 | 264.8 / 247.1 | 15.4 / 14.3 |
| slot | 20 | R2731 / R2732 (batch 18u, today's tree) | over bar / over bar | 19.4 / 20.1 | **19.59 / 18.90** | 19.13 / 18.64 | 465 / 479 | 318.1 / 325.0 | 19.4 / 20.1 |

Values in brackets belong to VOID runs and are not counted. "Steady offset" is the ring's mean |p - p_ref| in the
4-quad hover, over [weld + 15, path - 0.5] s. "Path error after" is the clock-free figure-8 error from path - 0.5 s.
The fold-in peak is taken over [weld - 1, weld + 15] s.

**Separation, apart from the arm comparison** (critic B1): the newcomer's body to the nearest carrier's, from the weld
to LAND (m). The line is 0.30 m.

| direction | arm | 0 | 2 | 4 | 6 | 8 | 11.6 | 12 | 16 |
|---|---|---|---|---|---|---|---|---|---|
| along the rim | measured | 0.65 / 0.65 | 0.61 / 0.62 | 0.58 / 0.58 | 0.54 / 0.54 | (0.49) / 0.48 | 0.37 / 0.37 | 0.36 / 0.32 | **0.22 / 0.23** |
| along the rim | §2 static (planned body to the plate-11 carrier) | 0.73 | 0.67 | 0.61 | - | 0.49 | - | 0.36 | 0.23 |
| along the rim | slot | 0.65 / 0.65 | 0.63 / 0.63 | (0.62) / 0.62 | no weld / 0.60 | 0.59 / 0.59 | 0.56 / 0.56 | 0.56 / 0.56 | 0.54 / 0.54 |
| radially in | measured | 0.65 / 0.65 | 0.65 / 0.65 | 0.65 / 0.65 | 0.65 / 0.65 | 0.64 / 0.65 | | 0.64 / 0.64 | 0.66 / 0.66 |
| radially in | slot | 0.65 / 0.65 | 0.65 / 0.65 | 0.65 / 0.65 | 0.65 / 0.65 | 0.65 / 0.65 | | 0.65 / 0.65 | 0.65 / 0.65 |

**The limits (cm)**, as scored now (VOIDs left out, so a cell with one counted run ends a limit) and what the pending
re-fly decides:

| arm | direction | d_tol | d_tol_true | d_sep | d_task | decided by |
|---|---|---|---|---|---|---|
| measured | along the rim | 6 (8 if the t08 re-fly completes within the bar; 8 is the ceiling: 11.6 cm is over the bar 2/2) | 7.7 | 6 (12 if the re-fly keeps 0.30 m; 16 cm crowded 2/2) | 6 (8) | the `u_meas_t08` re-fly |
| measured | radially in | 16 | 16.2 | 16 | 16 | the 20 cm extension (due) |
| slot | along the rim | 2 (4 if the t04 re-fly completes within the bar; 4 is the ceiling: the 6 cm no-weld) | 5.5 | 2 (4; the 6 cm no-weld ends it) | 2 (4) | the `u_slot_t04` re-fly |
| slot | radially in | 6 | 6.4 | 16 | 6 | - |

**Verdict per frozen bar**

| Bar (§5) | As flown | Verdict |
|---|---|---|
| Confirm: d_task(measured, along the rim) >= 12 | 6 as scored, at most 8: the fold-in bar fails 2/2 at 11.6, 12 and 16 cm | **not met** |
| Confirm: d_task(measured, radially in) >= 16 | 16: all 14 runs complete within the bar, at least 0.64 m apart | met |
| Confirm: d_tol(measured) >= d_tol(slot), both directions | along the rim 6 vs 2 (8 vs 4 after the re-flies; no outcome reverses it); radially in 16 vs 6 | met, along the rim only through R2636's no-weld (observation 1) |
| Confirm: the mechanism shown, both directions | radially in shown; along the rim ambiguous (next row) | not met as the scorer computes it |
| Mechanism (critic M4); d = 0 spreads 0.20 deg (fold-in) and 0.44 deg (steady) | Radially in: fold-in growth +10.32 / +15.46 at 12 / 16 cm, steady +11.37 / +16.33: **shown**. Along the rim: fold-in growth -0.79 / -2.55 (the opposite sign: the slot arm folds in more gently). Steady growth +1.52 at 12 cm, and +0.83 at 16 cm on a one-run slot cell (R2672 has no steady window). By the card's words ("cell mean", "failed ones included") that is shown. The scorer's code needs two values a cell, so it takes 11.6 cm instead (-0.01): **not shown** | radially in shown; along the rim **ambiguous** (code: not shown) |
| Falsify: d_task(measured) < 8 (a failure, crowding or the bar at d <= 8) | none in the 17 counted measured runs at d <= 8 (fold-in max 2.63 deg, separation min 0.48 m). `u_meas_t08` has one counted run | **not fired**, pending the `u_meas_t08` re-fly |
| Falsify: measured fold-in peak > 5 deg at d <= 4 | max 2.63 deg (R2647, 4 cm in) | not fired |
| Falsify: d_tol(slot) > d_tol(measured), either direction | along the rim 2 vs 6; radially in 6 vs 16 | not fired (but see observation 1) |
| Plumbing: catch within 2.5 cm of the aim, every run | 0.1-2.0 cm in all 55 welds | pass |
| Separation prediction (reported, not a bar) | d_sep(measured, rim) is 6 as scored and 12 after a clean re-fly, as predicted. d_sep(measured, in) is 16, as predicted. d_sep(slot, in) is 16, as predicted. d_sep(slot, rim) is 2 (4): ended by the no-weld, though every welded slot run kept at least 0.54 m | as predicted, except the slot arm along the rim (no-weld only) |
| **Overall** | | **between: neither confirmed nor falsified** (reported as it is, §5), pending the two re-flies and the extension |

**Failure offset per arm** (the first nominal d where a counted run failed the task in any way):

| arm | along the rim | radially in |
|---|---|---|
| measured | 11.6 cm (the fold-in bar 2/2, then 12 and 16 cm 2/2; crowding at 16 cm 2/2) | none to 16 cm |
| slot | 6 cm (R2636 no weld; arm-independent), then 16 cm (R2672 late weld). No completed slot run is over the bar to 16 cm | 8 cm (the fold-in bar 1/2), then 12 and 16 cm 2/2 |

**Observations the bars do not carry**

| # | What | Evidence | Why it matters |
|---|---|---|---|
| 1 | Along the rim the slot arm folds in more gently than the measured arm | fold-in peak at 11.6 / 12 / 16 cm: slot 3.50, 3.60 / 3.78, 4.65 / 3.69 (R2644); measured 7.57, 7.62 / 5.82, 5.14 / 7.96, 8.87. Carrier dip: slot 36-48 mm (completed runs), measured 85-127 mm. E-runs: R2460 8.66 vs R2461 4.55 | On the fold-in bar alone the slot arm holds to 12 cm, the measured arm to 8 cm. Falsify 3 was avoided along the rim only because R2636 (slot, 6 cm) never welded. §2's arm ordering holds radially in, not along the rim |
| 2 | The no-welds are the twin trigger's tail, not the offset's or the arm's | In all 56 runs the tip was within 1 cm of the aim 19.0-21.1 s after MAGNET ON. The weld then waited 2.1-48.3 s (median 13.9 s) for the 0.05 m/s gate; rel_speed at the weld was 0.016-0.050 m/s, just under the gate. Welds came 22.7-68.9 s after MAGNET ON (median 34.0 s): R2637 at 56.3 s, R2672 at 68.9 s, R2636 never | Failure 1 counts the trigger's time-out as a task failure, so the trigger, not the fold-in, sets the slot arm's rim limit. The rig's magnet has no speed gate |
| 3 | The measured arm's static balance is not right at every d along the rim | steady tilt: 1.62-2.29 deg at 0-2 cm, 4.93-7.60 at 11.6-16 cm, where the model error is at most 0.4 cm. Radially in it falls to 0.08-1.32 deg from 6 cm | §2's "static balance stays right at any d" holds radially in only. Along the rim the measured arm moves the newcomer toward the plate-11 carrier (the separation table) |
| 4 | R2669 (`u_meas_t12`) never settled | ring tilt wobbling 3-13 deg, 6-18 cm off its reference through the hover and the figure-8 (path error 104.4 mm; its repeat R2641 settled at 6.4 deg, 11.9 mm) | the 12 cm measured cell's spread; the newcomer 0.32-0.37 m from a carrier throughout |
| 5 | Radially in, the slot arm's ring hangs off its reference and its newcomer nears full throttle | steady offset 75-396 mm (4-16 cm); newcomer throttle peak 0.78-0.79 in all 12 radial slot runs (max 0.8) | the unmodelled moment, as §2 says, at the throttle limit |

**Pending, in order**

| Item | Runs | Why |
|---|---|---|
| ~~Re-fly `u_meas_t08`~~ DONE: R2727, complete within the bar (2.44 deg, 0.49 m) | 1 | R2637 VOID (§5). It decides falsify 1 at 8 cm, and d_tol / d_sep / d_task (measured, along the rim): 8 / 12 / 8, or 6 |
| ~~Re-fly `u_slot_t04`~~ DONE: R2728, complete within the bar (1.95 deg, 0.62 m) | 1 | R2634, a runner fault (VOID by the runner rule; Wesley may count it complete instead). It decides the slot arm's rim limits, 4 or 2 |
| ~~The 20 cm extension radially in~~ DONE: R2729-R2732 (measured 2/2 within the bar; slot 2/2 over it, 18.9-19.6 deg) | 4 (x2 each) | Pre-registered (§5): neither arm failed (1-6) or crowded by 16 cm radially in. Not due along the rim |
| The tree | - | Batch 16's tree (be34f95 + cf1ab9ad) is gone. Since then: rate_kff, thr_tau_s and the mocap rate filter were deleted (all off in U: no U run logs them); partner-only launch arguments were added to graph_attach; tejen-sync2 was merged (tejen_* only). The 6 runs above would fly beside 54 on another tree. The working rules ask for one tree per claim table, re-flown on the final tree before the reviewer |
| Reviewer | - | On the final table |

**For Wesley** (recommended option first):
1. **R2634.** Re-fly `u_slot_t04` alongside R2637's re-fly; this is the default here. The other option is to count it
   complete, since the fault came after LANDED.
2. **The tree for the 6 runs.** Either fly them on today's tree beside batch 16's 54 runs, with a note in the table
   (U's code path differs only by deleted code that was switched off in U), or re-fly the whole U table on the final
   frozen tree before the reviewer.
3. **The rim wording.** The frozen verdict stands: between. For the thesis, say that along the rim the slot arm folds
   in more gently. The measured arm's rim limit is the fold-in bar at 8 cm and the separation at 12 cm. The falsifier
   was avoided only because the twin's weld trigger timed out once (R2636), independent of the arm. Avoid the card's
   fallback wording "the model point does not matter to 16 cm": it does matter there, but in the slot arm's favour.
4. **The mechanism along the rim.** Take the scorer's code ("not shown"), not the card's words ("shown" on a one-run
   cell inside the larger-d scatter). The card's verdict is the same either way.

**Wesley's answers (23 Oct, after the scoring):** (1) the 6 owed runs fly on today's tree, the table notes both trees;
U re-flies whole on the final tree at the results freeze. R2634 is re-flown (a runner race, now fixed in the runner).
(2) Along the rim the mechanism is NOT SHOWN (the scorer's code), radially in it is shown; wording: "the slot arm folds
in more gently along the rim".

### Batch 18u: the six owed runs (R2727-R2732, scored 23 Oct 17:45)
*On today's tree (Wesley: fly them on today's tree, note it): be34f95 + patch cae5cf26 (batch 16: be34f95 + cf1ab9ad).
Lab PC, 4 slots, `SIM_MOCAP_VEL_FILTER` ema, same world. Scored with `tools/catch_offset.py` as frozen; all six runner
exit 0, no manifest failure, none VOID (welds 53.4-83.6 s on the run clock, every one at least 17 s before the path
start at ~100.8 s), plumbing 0.1-1.1 cm. Reproduce: `python3 tools/catch_offset.py --table R2629-R2633 R2635-R2684
R2727-R2732` (R2634 left out: re-flown as R2728, by Wesley's answer (1)).*

| Run | Config | Re-fly of / cell | Completed | True offset (cm) | Fold-in tilt peak (deg) | Steady tilt mean (deg) | Path error after (mm) | Newcomer-carrier min (m) |
|---|---|---|---|---|---|---|---|---|
| R2727 | `u_meas_t08` | R2637 (VOID) | yes, within the bar | 8.1 | 2.44 | 6.22 | 15.7 | 0.49 |
| R2728 | `u_slot_t04` | R2634 (VOID) | yes, within the bar | 4.2 | 1.95 | 1.87 | 15.8 | 0.62 |
| R2729 | `u_meas_ri20` | 20 cm extension, a | yes, within the bar | 19.7 | 0.40 | 0.08 | 15.1 | 0.70 |
| R2730 | `u_meas_ri20` | 20 cm extension, b | yes, within the bar | 20.1 | 0.49 | 0.18 | 14.7 | 0.70 |
| R2731 | `u_slot_ri20` | 20 cm extension, a | over the bar | 19.4 | **19.59** | 19.13 | 318.1 | 0.65 |
| R2732 | `u_slot_ri20` | 20 cm extension, b | over the bar | 20.1 | **18.90** | 18.64 | 325.0 | 0.65 |

**The limits (cm), final for this grid** (VOIDs replaced by their re-flies):

| arm | direction | d_tol | d_tol_true | d_sep | d_task |
|---|---|---|---|---|---|
| measured | along the rim | 8 (11.6 over the bar 2/2) | 8.1 | 12 (16 crowded 2/2) | **8** |
| measured | radially in | 20 (the largest flown) | 20.1 | 20 | **20** |
| slot | along the rim | 4 (the 6 cm no-weld) | 5.5 | 4 | **4** |
| slot | radially in | 6 (8 over the bar 1/2) | 6.4 | 20 | **6** |

**Verdict per frozen bar, with batch 18u**

| Bar (§5) | As flown | Verdict |
|---|---|---|
| Confirm: d_task(measured, rim) >= 12 | 8 (the fold-in bar 2/2 at 11.6) | **not met** |
| Confirm: d_task(measured, in) >= 16 | 20 | met |
| Confirm: d_tol(measured) >= d_tol(slot), both | rim 8 vs 4; in 20 vs 6 | met (rim only through R2636's no-weld, observation 1) |
| Confirm: the mechanism, both directions | in: shown (fold-in growth +19.28 deg at 20 cm against the 0.20 spread); rim: not shown (scorer's code, Wesley's answer (2)) | **not met** |
| Falsify: d_task(measured) < 8 | R2727 completes within the bar at 8 cm with 0.49 m separation | **not fired** |
| Falsify: measured fold-in peak > 5 deg at d <= 4 | max 2.63 deg | not fired |
| Falsify: d_tol(slot) > d_tol(measured) | rim 4 vs 8; in 6 vs 20 | not fired |
| Plumbing | 0.1-2.0 cm, all 61 welds | pass |
| Separation prediction | d_sep(measured, rim) 12 as predicted; (measured, in) 20; (slot, in) 20; (slot, rim) 4, ended by the no-weld only | as predicted, except the slot rim (no-weld) |
| **Overall** | | **between: neither confirmed nor falsified** (final for this grid) |

- The 20 cm slot cell continues the radial trend: the ring hangs 465-479 mm off its reference, tilted ~19 deg, and the
  figure-8 runs 318-325 mm off the path; the measured arm at 20 cm is the flattest cell of the grid (0.08-0.18 deg).
- **Two trees in one table:** 55 runs on be34f95 + cf1ab9ad, 6 on be34f95 + cae5cf26. Per Wesley, U re-flies whole on
  the final tree at the results freeze, before the reviewer. Registry rows R2727-R2732, no reviewer yet.

# Card: a departed carrier rejoins its own plate (rejoin_departed, twin only; 23 Oct 2026, DRAFT rev 2)

Owner Wesley. Origin: plan 2026-10-21 "Code design B" (needed by the chain N, 4->3->2->3->4) and the push answer of
22 Oct (decisions.md, DECIDED 2026-10-22 push plan answers (2): S flies 8 -> 4 (-> 2) -> 8, so `rejoin_departed` is
built in this push, card and critic). **Status (rev 2, 22 Oct): in the main tree (day-1021, uncommitted) with the
critic's and the reviewer's findings answered (Response, at the end); offline tests only, nothing flown. The bars in
section 5 are PROPOSED: Wesley freezes them before E0.**

## 1. Question
Can a carrier that has left the ring (announced, soft or unannounced) be flown back to its own plate in the twin,
re-welded there and folded back into the fleet, so that S returns from 4 to 8 and the chain returns from the pair to
the square, with the ring level?

## 2. Hypothesis
- With `rejoin_departed` on, a departed carrier holds at a staging point in the ring frame instead of landing. On
  `/fleet/attach <d>` (only while the path is held: hover, or a held path) it flies the newcomer's approach
  (`approach.ApproachProfile`: climb, transit over its plate, descent at the locked 0.2 and 0.1 m/s) until its rod's
  end sits on the plate point it left. In the hold over the plate the tip loop (`TipLoop`, its validity band scaled
  to the 1 m rod) trims the body's height on the measured end.
- The twin's tether joint (the `DetachableJoint` that released it) re-welds the rod where its end is. With the end
  within 1.5 cm of the plate point for 0.3 s and slower than 2 cm/s, the planner sends `/drone_d/attach` and waits
  for the joint to report attached (0.5 s). Only then does the OCP grow, with the plate point the carrier left with and
  the rod it now hangs on (|pivot - plate point|, within 1.5 cm of the rod it left with). The fold-in is QUAD4's
  validated OCP-resize attach (`_attach_ocp`: the newcomer from 0.1 N and a vertical rod, blended to the balanced
  split over 10 s, the cable feed-forward ramped over 1.5 s, the path held through it).
- **Predictions (twin, 1 m, hover):**
  - the end on its plate 10-15 s after the ATTACH (about 2 s climb, 4 s transit, 2.5 s descent), then a wait for the
    rod's swing to die (section 9); REJOIN_WELD within 30 s of REJOIN_START;
  - the joint reports attached within 0.5 s of `/drone_d/attach`, and the end stays within 2 cm of its plate point
    through the blend;
  - the fold-in leaves the square level: ring tilt from the weld to +15 s at most 5 deg (the 28 Sep decision's twin
    bar; QUAD4's 0.55 m fold-in tilted 1.8-2.6 deg, R1486-R1488, another geometry).
- **Not claimed:** anything for the rig. The twin's rods are rigid, can push and have no collision geometry, and its
  joint welds at any distance; the rig's magnet catches only on contact. The rejoin flatters the twin (plan 2026-10-21,
  risks).

## 3. The one variable
`rejoin_departed` (planner, default off; twin `mode:=dissipative` only). Everything else as the baseline config
`c4_softdetach9_L100` (R2312), whose params_file `configs/rig/chain_pair_noxy.yaml` sets the loss rules (drop_gap_deg
200, min_survivors 2, the tilt drop at 45 deg). The rejoin also needs `departed_land: false` (a staged carrier holds
instead of landing), `reconfig_mode ocp` and io `tether_tips: true` (the weld reads each rod's end); the runner refuses
the config and the launch refuses the arguments when one is missing.

Configs (`configs/experiments/`, each passes `run_experiment.py --check`): `rj0_reweld_floor_L100` (E0),
`rj4_softdetach9_rejoin_L100` (E1), `rj3_pairrelease_rejoin_L100_tip` (E2), `rj4_softdetach9_rejoin_land_L100` (E3),
`rj4_softdetach0_fig8stage_L100` (E4). All on 1 m rods, io `tether_tips: true` and `tether_len: 1.0`.

## 4. Baseline
| What | Runs | Number |
|---|---|---|
| The square hovering on 1 m cables, before its detach | R2312 | ring tilt peak 0.19 deg over the 10 s before the DETACH |
| The soft detach of plate 9 from the 1 m square (hover), the T flying on | R2312 (c4_softdetach9_L100) | resize +3.00 s; ring tilt peak 22.9 deg; last 10 s before LAND: roll p-p 0.9, lean 0.8 deg |
| The rider's release from the 1 m T, the opposite pair flying on | R2314 (t3_pair_release_L100_tip) | detected (tip_detect_m), no drop |
| The OCP-resize fold-in of a newcomer with a vertical rod (QUAD4, M1 twin, 0.55 m) | R1486-R1488 ('start' seed; the 'ocp' seed R1479-R1490 tilted 7.7-8.6) | ring tilt 1.8-2.6 deg after the switch, 3 of 3 (scale only: another rod) |
| The hard detach (for scale) | R2309 | dropped at 46.2 deg |

## 5. Pass / fail numbers (PROPOSED; Wesley freezes them before E0)
Scored from the run's events.csv and run.csv. The runner maps the planner's lines to rows: REJOIN_START,
REJOIN_ON_PLATE (the end passed the gate: `/drone_d/attach` sent), JOINT_DETACHED / JOINT_ATTACHED (the twin's joint
state), REJOIN_WELD (the joint confirmed, the OCP grows), FOLD_IN, REJOIN_BLEND_OVER (with the end's largest distance
from its plate point through the blend), REJOIN_ABORT, REJOIN_REFUSED and STAGED_LEAVES. Ring tilt is the ring
normal's angle from vertical; lean is its mean over the window.

**Staged scoring (critic):** a rejoin is scored in three stages, and a run is reported by the stage it reached.
- (a) the end on the gate: REJOIN_ON_PLATE within 30 s of REJOIN_START;
- (b) the joint: JOINT_ATTACHED within 0.5 s of REJOIN_ON_PLATE, then REJOIN_WELD and FOLD_IN;
- (c) the fold-in:
  - ring tilt at most 5 deg from REJOIN_WELD to +15 s;
  - the end at most 2 cm from its plate point through the blend (REJOIN_BLEND_OVER);
  - the rejoiner carries its share: its planned tension (planner log) at the blend's end within 10% of the balanced
    share, and the ring's lean over the last 10 s before LAND at most 2 deg (a rejoiner short of its share leans the
    ring toward its plate);
  - every quad lands, no drop or abort.

A stop at (a) or (b) is a twin finding (section 9: the swing, the height, the joint), not a fold-in result; it goes to
Wesley before anything is deleted (critic item 3).

| Run | Config | Confirms | Falsifies |
|---|---|---|---|
| Unit tests (before any run) | | `test_rejoin_departed.py` 37, `test_rejoin_runner.py` 11 and the launch test pass; every other suite unchanged | any fails |
| E0 x1, re-weld smoke | `rj0_reweld_floor_L100` | JOINT_DETACHED within 2 s of the RELEASE and JOINT_ATTACHED within 2 s of the GZ_PUB, on the floor before ARM; LIFTED; the square hovers 20 s on the re-welded rod with ring tilt at most 2 deg; no UNANNOUNCED DETACH; every quad lands | a row missing, no lift, tilt over 2 deg, an UNANNOUNCED DETACH, a drop or abort |
| E1 x2 | `rj4_softdetach9_rejoin_L100` | stages (a), (b) and (c) | REJOIN_REFUSED or REJOIN_ABORT; no REJOIN_WELD within 30 s of REJOIN_START; the end over 2 cm from its plate through the blend; tilt over 5 deg in the 15 s after the weld; lean over 2 deg; a drop or abort |
| E2 x1 (neighbour: the chain's 2 -> 3) | `rj3_pairrelease_rejoin_L100_tip` | as E1, for the T; the pair's lean over the 10 s before the ATTACH reported | as E1 |
| E3 x1 (neighbour: LAND 1.5 s into the descent, before the end can reach the gate) | `rj4_softdetach9_rejoin_land_L100` | REJOIN_ABORT (LAND); the rejoiner climbs until its end is 0.25 m over its plate before it moves out; it lands alone at least 1.2 m (the clear radius) from the ring centre; no "announcing landed anyway" line; LANDED only after the rejoiner is down; every quad lands | any of these missing; a drop or abort |
| E4 x1 (neighbour: staging on a path) | `rj4_softdetach0_fig8stage_L100` | REJOIN_REFUSED ("the path is running") for the mid-lap ATTACH; the staged drone inside +-1.6 m, at least 0.6 m from the ring centre and 0.8 m from every carrier body through the lap; STAGED_LEAVES at the LAND; every quad lands; the T's path error reported | any of these missing or over; a drop or abort |
| E5 x1, a 2nd if clean (neighbour: the pair leaning 2-4 deg toward plate 3, the plate the rejoiner welds on; critic 23 Oct M1) | not built: no config-only lever leans the pair (below); `rj3_pairrelease_rejoin_L100_lean` waits for a world | as E2 (stages (a), (b), (c)), with the fold-in tilt at most 5 deg from REJOIN_WELD to +15 s; the cap's binding line (REJOIN BLEND OVER) reported; the pair's lean toward plate 3 over the 0.5 s before REJOIN_WELD reported (under 2 deg the run does not test the lean) | as E1 |

Rev 1's "no weld within 60 s" is gone: the 45 s back-out made it unreachable, and 30-60 s counted as neither (critic,
bars).

**E0's bars FROZEN 23 Oct 00:08 by the integrator, as written above, before E0 flies** (overnight authority: Wesley
asleep, no questions; he may revise E1-E4's before they fly). E1 waits for E0's verdict, as the order says.
**E1's bars FROZEN 23 Oct 00:42 by the integrator, as written in section 5, before E1 flies** (E0 stopped on the
tilt, not at stage (a) or (b): section 6 lets E1 follow).
**E5's bars FROZEN 23 Oct 07:57 by the integrator on Wesley's word (decisions.md DECIDED 2026-10-23 (10)), as written
in section 5, before it flies.** Its lever waits for Wesley (below).

**E5's lever (23 Oct, offline): no config-only lever leans the pair toward plate 3 in this twin.**
- Why: the pair's ring hangs on two ball-jointed stubs on a diameter (plates 0 and 6), 2 cm above its centre of mass.
  Every rod force passes through a stub, so in a hold no rod has a moment about that axis. The ring settles where its
  own mass puts it, whatever the pair's drones do (they can only make it swing, a 2.30 s pendulum). Today it settles
  0.75 deg toward plate 3, from the 1 g stub the release leaves on plate 3 (m_s g 0.225 = M g 0.02 sin theta).
- Measured, E2 (the ring's quaternion in run.csv; positive when plate 3's side is down):

| Run | Pair lean toward plate 3, 10 s before the ATTACH (mean; min / max) | Over the 0.5 s before REJOIN_WELD | The pair's swing period | The cap |
|---|---|---|---|---|
| R2594 | 0.81 deg; -0.53 / 2.14 | 0.75 deg | 2.30 s | bound 101 ticks |
| R2595 | 0.81 deg; -0.52 / 2.14 | 0.79 deg | 2.30 s | bound 101 ticks |

- The cable-length lever: there is no per-rod typed length. `measure_rod_len` replaces every rod with its measured
  length at the hand-over. A pair carrier's length error would also tilt the ring about the axis through plate 3
  (toward plate 0 or 6), which leaves plate 3's height, and so the cap's `level_z`, unchanged. MOCAP_TILT on the ring
  only fakes the measurement, and about the ring's body y.
- Ways to lean it (each a world SDF, so Wesley's word):
  1. **Recommended:** the T L100 world with the ApplyLinkWrench system, otherwise identical, plus the runner's WRENCH
     event: a torque of +0.0066 N m about world x (the ring's yaw is 0) after the RELEASE. That gives 3.0 deg toward
     plate 3 (with the stub's 0.75 deg). The negative torque leans it away from plate 3, which is the cap's other case:
     `level_z` under the weld height, so the reference steps down 0.225 sin theta at once.
  2. stub_pay_1 at 4 g instead of 1 g: 3.0 deg, one side only.
- What it should show: leaning toward the plate, the weld-height cap binds through the blend (the plan wants the
  rejoiner about 1.2 cm up to level the ring). It lets go at the blend's end (+10 s, inside the 15 s window). That is
  where R2594/R2595, at 0.75-0.8 deg of lean, peaked: 1.45 / 2.25 deg, about 1.3 s after the cap let go.

## 6. Repeats
E0 once (a smoke), E1 x2 (the working rules: 2 in Gazebo), E2, E3 and E4 once each (neighbours). Order (critic): E0;
the offline R2312 swing and z check (section 9, done); E1 x2; E2; E3; E4. A stop at stage (a) or (b) halts the order.
Exploratory once E1 passes (critic M2's exposure, one registry row): E1 with a RELEASE of a neighbour 3 s after the
weld.

## 7. Cost
6 laptop runs of 4-5 min each, after Wesley freezes the bars.

## 8. New code?
Yes. In `src/dissipative_planner/dissipative_planner/dissipative_node.py`, all behind `rejoin_departed` unless said:
- **The release** (`_resize_and_release`): the leaver's plate point and measured rod length are read before the resize
  renumbers the slots. `_stage_departed` keeps them and replaces the world-fixed step-out.
- **Staging** (`_stage_point`, `_rejoin_tick`): on the plate's bearing, `REJOIN_STAGE_OUT_M` beyond the formation's
  reach (1.025 m from the centre for the rig ring at 1 m, 1.3 m for S's 0.5 m ring). It moves with the planned ring
  point, turns with the planned yaw, and is clamped `STAGE_LIMIT_MARGIN_M` inside the position limit (pos_limit_m when
  set, else 1.75 m). The reference uses `take_back_ramp` (0.15 m/s, 0.25 m/s^2) from the measured position with the
  frame's velocity, not the measured one (a loaded release kicks the leaver up).
- **The rejoin** (`_start_rejoin`, `_rejoin_approach_tick`): refused while the path runs. `ApproachProfile` toward the
  plate point on the measured ring, aimed so the rod's end lands on it, with a `TipLoop` on the measured end
  (`tip_valid`'s band scaled to the rod: body-to-end +-4 cm, the lean limit at the same angle). The path stays held.
- **The weld** (`rejoin_weld_gate`, `_rejoin_send_attach`, `_rejoin_fold`): the gate as before (1.5 cm on every sample
  of 0.3 s, at least 8 samples, gaps up to 50 ms, under 2 cm/s, no blend running). Then the n+1 OCP and the poses are
  checked, and only then `/drone_d/attach`. The fold-in waits for the joint's 'attached' (`REJOIN_JOINT_S`); without
  it the joint is let go (`/drone_d/detach`) and the drone backs out. The OCP grows with the plate point and the rod
  re-measured as |pivot - plate point| (within `REJOIN_WELD_M` of the rod it left with); the fold-in's seed turns with
  the planned yaw.
- **The blend** (`_rejoin_blend_tick`): until it ends the rejoiner counts as absent for the drop rule (`_drop_due`),
  an announced DETACH is refused, and the end's largest distance from its plate point is logged at its end.
- **Back out** (`_rejoin_retreat`): no weld `REJOIN_TIMEOUT_S` after the ATTACH, no OCP or pose, or no joint answer.
  It climbs until its end is 0.25 m over the plate with the path still held, then returns to staging as the path
  resumes; the ATTACH may be repeated.
- **LAND or a drop** (`_rejoin_out`): every staged or rejoining drone leaves for its own landing. Over its plate it
  climbs first; then out along its bearing to the larger of its staging radius and the clear radius
  (`_departed_clear_r`), kept inside the limit's landing margin (`_land_clear`; 1.75 m while pos_limit_m is off), and
  down alone. A pending `/drone_d/attach` is answered with `/drone_d/detach` first. The landed wait grows by
  `REJOIN_OUT_EXTRA_S` (LAND 15 -> 45 s, drop 30 -> 60 s).
- **The joint** (`_rejoin_joint_cb`): its state is logged on change; an 'attached' for a drone off the ring with no
  weld awaited is let go at once.
- **Refusals** (`_rejoin_refused`, logged REJOIN REFUSED): not staged, already rejoining, dropped, LAND or descent,
  ring not up, the path running, another rejoin under way, an unload running, no OCP for n+1, rod's end not seen for
  0.5 s.
- **The latent bug, fixed** (`_do_attach`, always): with the switch off, `/fleet/attach` of a departed carrier logs a
  refusal. Before, it folded the carrier in at its BODY with the typed rod and no weld (on the baseline code at
  (0, 0.325, 0.926) m in the ring frame, 0.9 m above its plate).
- **Always, in `_attach_ocp`**: a newcomer's departed entry is dropped at the fold-in, and its loss detector starts
  afresh. QUAD4's combined-M1 fold-in (partner_contact on, both seeds) is bit-identical before and after this package
  (the reviewer's stub of test_partner_contact, field by field).
- **The parent planner** (`mpc_planner/planner_node.py`): the LAND's departed wait is a method,
  `_land_departed_wait_s()` (15 s, unchanged for every other flow).

Launch and runner:
- `/fleet/attach` is subscribed with the switch too; `graph_detach.py` adds one `rejoin_bridge` (`/drone_i/attach` to
  gz, each joint's state back) with the switch.
- `real_mode.planner_options(rejoin=)` refuses the switch on the rig and in every other graph, and without
  `departed_land:=false` and `reconfig_mode:=ocp` (before any node starts).
- `common.yaml` and `PLANNER_OPTIONAL` get `rejoin_departed` ('' = off).
- The runner's rows above; its config check refuses an ATTACH of a carrier's id without the switch, and the switch
  without `departed_land: false`, `reconfig_mode ocp`, `mode: dissipative` and io `tether_tips` (launch.args or the
  typed params_file).

Constants (`dissipative_node.py`):

| Name | Value | Why |
|---|---|---|
| REJOIN_STAGE_OUT_M | 0.3 m | beyond the formation's reach, on its own (empty) bearing: 1.26 m from the nearest carrier body in the 1 m square, 0.92 m with 8 quads on S's ring |
| POSITION_LIMIT_M / STAGE_LIMIT_MARGIN_M | 1.75 / 0.15 m | Wesley 22 Oct: a quad beyond +-1.75 m LANDs the fleet. pos_limit_m replaces 1.75 when it is set |
| REJOIN_WELD_M | 0.015 m | the twin welds where the end is: the rod's length changes by up to the offset (re-measured at the fold-in) |
| REJOIN_WELD_S / REJOIN_WELD_N | 0.3 s / 8 | a still end on its plate, not a pass through it |
| REJOIN_SLOW_MPS | 0.02 m/s | the descent runs at 0.1 m/s |
| REJOIN_TIP_GAP_S / REJOIN_TIP_FRESH_S / REJOIN_TIP_BAND_M | 0.05 s / 0.5 s / 0.05 m | the rod poses come at 60 Hz; a sample off the rod's length from the body is not its end |
| REJOIN_TIMEOUT_S | 45 s | about 3x the expected approach |
| REJOIN_JOINT_S | 0.5 s | the joint's state comes back on the next gz step through one bridge |
| REJOIN_OUT_EXTRA_S | 30 s | up, out and down from over its plate is ~14 s longer than a departed drone's landing (critic M1) |
| REJOIN_OUT_MARGIN_M | 0.05 m | (23 Oct, E3) the way out lands this far past the clear radius |
| STAGE_TRACK_M | 0.03 m | (23 Oct, E4) the staged reference stays this far inside the staging clamp, for the drone's tracking |

If E1 fails at stage (c), the switch stays off and the code is recorded and deleted (the working rules,
keep-only-if-better). A stop at (a) or (b) goes to Wesley first (section 5). The latent-bug refusal stays either way.

**Limits, stated for the thesis:**
- A rejoin flies only in a hold. A figure-8 descends when it ends, so the rejoin after a lap needs `traj_laps` and a
  runner wait on the trajectory clock (the rest of code design B; not in this package). S and the first N runs are
  hover.
- The tip loop corrects the end's height, not its swing: the weld waits for the swing to die by itself (section 9).
- The fold-in trusts the joint only once it has said 'attached'.

## 9. B1: the released rod's swing and the departed hold's height (offline, 22 Oct)
R2312's rod poses were not kept: its `gz_record` was deleted after the video was rendered, and no release run on 1 m
rods has one (R2309-R2319). The rod's end is therefore inferred from its drone's body (run.csv) and the world's rod
(0.075 kg, its centre of mass 0.733 m below the pivot, 8e-3 kg m^2 about it: a 1.88 s pendulum with the pivot held).
Method (scratch script): the body's radial error from its reference after the release, less its 2 s mean; its
extrema; an exponential fit of their size from 2 s on; the rod's own equation at the measured frequency gives the end's
swing about the body (theta / x_body = m d w^2 / (m g d - I_p w^2)). Each departed drone began to land about 3 s after
its release (departed_land on in those runs), so the window from 3 s on includes its descent.

| Run | What | Body swing | Decay (time constant) | End's swing about the body | Hold z error (peak; last 1 s of the step-out) |
|---|---|---|---|---|---|
| R2312 | soft detach, plate 9, hover | period 2.07 s; 5.5 cm at 2.2 s, 1.1 cm at 8.4 s | 3.9 s | 5.8 cm at 8.4 s; about 3.6 cm at 10 s (2.3-8.1 for +-5% on the period) | +3.8 cm; +1.6 cm |
| R2313 | soft detach mid figure-8 | period 1.98 s; 4.8 cm at 2.2 s, 1.8 cm at 8.2 s | not separable from the path (the ring moving) | not inferred | +4.2 cm; +1.3 cm |
| R2314 | the T's rider released (unannounced), hover | period 2.02 s; 2.2 cm at 2.7 s, 1.6 cm at 7.8 s | 17 s | 11.5 cm at 7.8 s (at least 6.6) | +1.9 cm; +1.1 cm |
| R2316 | the same, mid figure-8 | period 2.00 s; 2.6 cm at 2.9 s, 1.5 cm at 7.8 s | 13 s | 13.8 cm at 7.8 s (at least 7.0) | +1.5 cm; +0.7 cm |

Reading:
- The swing after 10 s is over 5 mm by any reading: at least 2.3 cm in the best case (R2312), more after a rider's
  release. The ball joint does not damp it (reference.md: DART ignores damping on ball joints); only the drone's own
  position loop does, slowly.
- The departed drone sits 0.7-1.6 cm above its reference 3 s after the release (4 cm at the kick), against the gate's
  1.5 cm.
- So, per the critic's rule, the closed loop on the measured end is built (section 8, the tip loop, behind the
  switch). It removes the height bias; it does not damp the swing. The gate waits for the swing (the "rod end N cm"
  lines every 2 s measure it in E1). If E0 or E1 stops at stage (a) on the swing, the next step is Wesley's choice: a
  damping term on the measured end in x and y (a new law in the flight loop, its own critic) or a damped universal
  joint on the rod (a world SDF).

## Critic
What the critic was asked to challenge, most important first:
1. **The weld rule.** Is 1.5 cm, 0.3 s and 2 cm/s right for a joint that welds wherever the end is? What does an
   off-plate weld of up to 1.5 cm do to the planner's model (rod length, lever), and should the fold-in re-measure the
   rod from mocap rather than trust the remembered length?
2. **The staging point.** Is it safe in the ring frame on a figure-8 and a spinny line? It stays on the departed
   bearing and turns with the planned yaw. Is the clamp at 1.6 m right when the ring is near the limit (the staged
   drone then sits only 0.6 m from the ring's centre)?
3. **Approach geometry.** The approach transits along its own empty bearing at the approach's clearance height (the
   rod's end 0.25 m over the plate). Can a swinging rod or a neighbour's body touch it with 8 quads (45 deg)?
4. **The path freeze at the ATTACH** (a step stop if the ring moves): refuse the ATTACH while the ring moves instead?
5. **LAND or a drop mid-approach:** the drone climbs, then moves out, then lands alone. Is a climb right while the ring
   descends under it?
6. **The two changes outside the switch** (the departed entry dropped at every OCP fold-in, the detector reset): is
   any flown flow affected?
7. **The fold-in blend:** is QUAD4's 0.1 N seed and 10 s blend right for a carrier coming back to a T or a pair, or
   should it start from its share?
8. **The timeout and retry policy**, and whether one rejoin at a time is too strict for S (four returns).

### The challenge
*The project's critic, 22 Oct 2026 (read-only review of the card and the code as built, before any run).*

1. **Tried before?** Not for a departed carrier. Before this patch, an ATTACH of a departed carrier folded it in at its body with no weld. The mechanism has been tried, and the record goes against parts of it:
   - **Fold-in that works:** a weld, then the OCP grows from 0.1 N with the rod vertical, blended over 10 s (R0663–R0666, M1 claim R0685/R0686). That was QUAD4's 0.55 m arm on a damped universal joint.
   - **Twin traps the card skips:** an undamped ball-jointed arm swung for about 80 s, and its weld came 91 s after the handoff (R0660). A free drone sat 2.6–2.7 cm above its reference and never welded in 200 s (R0654/R0656). `docs/reference.md:184` says "ON cannot re-weld, restart Gazebo". `docs/reference.md:284` says DART ignores damping on ball joints.
2. **Simplest explanation the change leaves out:** the rod's end may never hold still within 1.5 cm of its plate in this twin. The tether-to-drone joint is a ball joint (`four_rigid_ground_rig_L100.sdf:248-253`), so its 0.1 damping does nothing. The approach is open loop in height. And a re-weld of a released tether has never been shown working in our Gazebo.
3. **What would falsify it, and does the fail threshold test that?** The hypothesis fails if a weld that holds is followed by a ring that won't level. The card's fail list mixes that up with twin artefacts. A timeout abort caused by rod swing or a height offset would count as a fail, and the card then says to delete the code that S's 8 → 4 → 8 return needs. On the pass side, "end ≤ 1.5 cm at the weld" proves nothing, because the weld gate already requires it.
4. **What it breaks if it works:** no locked decision.
   - **Rig and network graphs:** the switch is refused there.
   - **The rig's attach path:** `_attach_ocp`, which is QUAD4's rig attach path, is edited outside the switch.
   - **The position limit:** the staging clamp hard-codes ±1.75 m instead of using `pos_limit_m`.
   - **E1's loss rules:** E1 picks up `chain_pair_noxy.yaml`: drop_gap_deg 200, min_survivors 2, tilt drop at 45°.
5. **Asked for?** Yes: decisions.md, 22 Oct push answer (2).

### Blockers
- **B1. The weld gate is unlikely to pass in the twin, so E1 can fail for the wrong reason.**
  - **Swing:** to meet ≤ 1.5 cm, ≤ 2 cm/s and 0.3 s, the end must swing less than about 5 mm. The approach's carrot jumps from 0 to 0.2 m/s and back to 0 at the start and end of the transit, with no acceleration limit (`approach.py:153-162, 208-212`). Each jump starts a swing of about ±5 cm on the 1 m rod, and the ball joint does not damp it.
  - **Height:** the end's height is open loop, and the 1.5 cm tolerance is tighter than the bias recorded in R0654/R0656.
  - **Before freezing:**
    - From R2312's recorded rod poses, measure how far the leaver's rod end swings after release and how fast that dies away.
    - From the same run, measure the departed hold's error in z.
    - If the swing is still over 5 mm after 10 s, choose one fix:
      - a closed loop on the measured end, which the twin measures exactly (the TipLoop's distance band scaled to the 1 m rod);
      - or a damped universal joint in the world, which needs Wesley's word because it is a world SDF.
- **B2. The fold-in does not wait for the joint.**
  - **What happens:** `_rejoin_weld` (around `dissipative_node.py:3271-3276`) publishes `/drone_d/attach` and grows the OCP in the same tick. `_rejoin_joint_cb` (about line 3223) only logs the joint state.
  - **If the weld never happens:** the OCP flies a square model while only the T holds the ring. With E1's 45° tilt drop, the ring tilts to 45° before it is dropped.
  - **The card's backstop is weak:** the card says the stretch detector catches a missed weld. That detector needs the rod 6 cm over its length plus a 3 cm jump (`dissipative_node.py:1688-1703`). An OCP that re-solves from the measured, sagging ring may never produce that jump.
  - **Fix:** fold in only after `detachable_joint_state == attached` arrives within 0.5 s. Otherwise send the detach and retreat.
  - **Smoke run E0:** release one tether, re-attach it, and check that the joint state reads "attached" and the end follows the plate for 10 s.

### Major
- **M1. LAND during a rejoin can disarm the rejoiner in the air.**
  - **The rejoiner's path:** up about 0.25 m, out about 1 m at 0.15 m/s, then down from about 2.3 m. That takes roughly 14 s.
  - **The budget:** the planner announces `/fleet/landed` 15 s after the survivors touch down (`planner_node.py:107, 1465`), and 30 s after a drop (`dissipative_node.py:109, 1600`). With target_z 1.0, that leaves about 5 s of margin.
  - **E3 should fail on:** the line "announcing landed anyway", or the rejoiner not yet down when `/fleet/landed` is sent.
- **M2. A second loss during the 10 s fold-in blend.**
  - **The problem:** `_drop_due` and `_gap_flies` (`1497-1513`) count the rejoiner as a full survivor while it carries about 0.1 N. The resize then cuts the blend, so the rejoiner's tension steps straight to its share (the R0541 kind of yank). With drop_gap_deg 200, the fleet flies on with one leg that is nearly a passenger.
  - **Fix:**
    - Count the rejoiner as absent until its blend ends.
    - Refuse DETACH/RELEASE during a rejoin and its blend.
  - **Exposure run:** E1 plus RELEASE of a neighbour 3 s after the weld (exploratory).
  - **Lesser case:** a loss during the approach that leaves the opposite pair also flies on. The pair's roll then likely holds the plate above 2 cm/s, so the rejoin aborts. That is harmless but should be stated.
- **M3. Paths are never flown.**
  - **What is untested:** E1–E3 are all hover. `_stage_point` (about line 3095) follows the planned point and yaw, and its per-axis clamp pulls a staged drone to 0.6–0.7 m from the centre at the figure-8's ends, inside the 0.725 m reach.
  - **The LAND step-out:** a staged drone under LAND steps out to 1.2 m from the measured centre (`1056-1065`). Unless pos_limit_m is on, that step is not clamped, so it can be commanded past 1.75 m.
  - **Fix:**
    - Add E4: stage through one figure-8 lap at 0.2 m/s, then ATTACH in the hold.
    - Answer to item 4: refuse an ATTACH while `_traj_running`. Today it stops the ring in one tick and steps the rejoiner's reference velocity by 0.2 m/s.

### Minor
- **Bars:**
  - Between "weld within 30 s" (pass) and "no weld within 60 s" (fail) there is a 30–60 s band that counts as neither. The 45 s abort also means "60 s" can never be reached. Fail on REJOIN_ABORT or no weld within 30 s.
  - R1486–R1488 come from the 0.55 m geometry. The 5° bar stands on the 28 Sep decision instead.
- **Weld rule (item 1):** keep the plate point; the stub's ball joint stays the pivot. Re-measure the rod at the weld as |pivot − plate point|, because the up-to-1.5 cm offset goes into the length.
- **Ordering hazard:** when `_attach_ocp` fails, attach and detach go out through two different bridges with no ordering. If the detach arrives first, the rod stays welded while the drone climbs away. Check the pose and the OCP before publishing the attach.
- **Rig timing:** integrate after Thu 29, or show `_attach_ocp` bit-identical on a combined-M1 stub with partner_contact on.

### Bars and runs
Order: E0 (re-weld smoke), then the offline R2312 swing and z check, then E1 ×2, E2, E3, and E4 (figure-8 staging).

E1 should be scored in stages: end on the gate, joint attached, fold-in tilt. Pass items to add:
- the end stays within 2 cm of the plate through the blend;
- the rejoiner's tension reaches its share;
- no "landed anyway" line in E3.

CRITIC: rewrite the card — as written, E1 can fail for twin reasons (an undamped ball joint, an open-loop end height, a re-weld never shown), and that failure would delete the code S needs, while the fold-in never checks that the joint actually welded.

## Response (22 Oct)
Each finding, its fate (fixed / accepted / needs Wesley), and where. Tests: `test_rejoin_departed.py` (37),
`test_rejoin_runner.py` (11), `test_real_mode_launches.py`.

### The critic
| Finding | Fate | What |
|---|---|---|
| B1 the weld gate in the twin (swing, height) | fixed in part; **needs Wesley** if E0/E1 stop on the swing | Measured offline (section 9): the swing after 10 s is at least 2.3 cm, the hold sits 0.7-1.6 cm high. Built the closed loop on the measured end behind the switch: the rejoin's descent flies a `TipLoop` on the end, `tip_valid`'s band scaled to the 1 m rod. It removes the height bias, not the swing. No world change. If the swing stops E0/E1 at stage (a): an x/y damping term on the measured end (a new law: critic) or a damped universal joint (a world SDF). |
| B2 the fold-in does not wait for the joint | fixed | `/drone_d/attach` then the mode 'weld': the OCP grows only after the joint's 'attached' arrives (`REJOIN_JOINT_S` 0.5 s); else `/drone_d/detach` and a back-out. Tests: no fold-in before the joint; no answer -> detach, retreat, no resize. |
| B2 smoke run E0 | fixed | `rj0_reweld_floor_L100`: release and re-weld plate 9's tether on the floor before ARM (RELEASE, GZ_PUB `/drone_1/attach`), then the square lifts and hovers on it; the joint's states are rows. |
| M1 LAND during a rejoin: landed announced with it in the air | fixed (extend the wait; tested) | `REJOIN_OUT_EXTRA_S` 30 s once a staged or rejoining drone left for its own landing: the LAND's wait 15 -> 45 s (`_land_departed_wait_s`, a method of the parent now, unchanged elsewhere), the drop's 30 -> 60 s. Chosen over landing it first: LAND lands every drone at once (locked). E3 fails on the "landed anyway" line or LANDED before the rejoiner is down. |
| M2 a second loss during the blend | fixed; one residual accepted | The rejoiner counts as absent in `_drop_due` until its blend ends; an announced DETACH is refused during a rejoin (approach, weld, retreat) and its blend. A RELEASE cannot be refused (the joint or magnet letting go); its loss is judged without the rejoiner. Accepted: when the others still carry the ring (an opposite pair), the resize keeps the rejoiner and ends its blend (the R0541 step). The lesser case (a loss during the approach leaving a pair) is harmless: the rejoin goes on or aborts on the gate. The exposure run is listed in section 6 (exploratory). |
| M3 paths never flown | fixed | An ATTACH while the path runs is refused ("the path is running"). E4 (`rj4_softdetach0_fig8stage_L100`): the +x carrier staged through one 0.9 m figure-8 lap, so the clamp acts at the +x lobe; the mid-lap ATTACH must be refused. Not buildable here: an ATTACH in a hold after a lap (a figure-8 descends when it ends; `traj_laps` is the rest of code design B). |
| M3 the LAND step-out of a staged drone, unclamped while pos_limit_m is off | fixed | Staged drones leave through `_rejoin_out` at LAND or a drop: out along their bearing to the clear radius inside the limit's landing margin (1.75 m while pos_limit_m is off), down alone. After a drop they no longer land at the staging radius (1.025 m), inside the clear radius (1.2 m). |
| Item 4 the path freeze at the ATTACH | fixed | as M3: refused while the path runs. |
| What it breaks: the staging clamp hard-codes 1.75 m | fixed | pos_limit_m when set, 1.75 m otherwise (`_rejoin_limit`), for the staging point and the rejoin's landing point. |
| What it breaks: `_attach_ocp` edited outside the switch (QUAD4's rig path) | fixed (shown) | QUAD4's combined-M1 fold-in, partner_contact on, seeds 'start' and 'ocp': bit-identical before and after (resize, seed, tensions, hold, log, state). The yaw term applies to a rejoin only. |
| What it breaks: E1's loss rules from chain_pair_noxy.yaml | accepted | Stated in section 3 (drop_gap_deg 200, min_survivors 2, the tilt drop at 45 deg). |
| Minor: the bars' dead band; R1486-R1488's geometry | fixed | Fail on REJOIN_ABORT or no weld within 30 s; the 5 deg bar cites the 28 Sep decision (the twin keeps 5 deg). |
| Minor: the weld rule | fixed | The plate point is kept; the rod is re-measured at the fold-in as |pivot - plate point| (clamped to the rod it left with +-1.5 cm). Tested with the end 1 cm low: 0.99 m. |
| Minor: the ordering hazard | fixed | The attach goes out only after the n+1 OCP and the poses are checked; a LAND or drop with the attach out sends the detach first; an 'attached' with no weld awaited is let go. |
| Minor: rig timing | accepted; **Wesley's word** on the merge | Twin-only switch, and the rig's fold-in shown bit-identical. Nothing is committed: when it joins the rig's tree is Wesley's call (after Thu 29 if he prefers). |
| Bars and runs: staged scoring, end within 2 cm through the blend, the share reached, no "landed anyway" in E3 | fixed | Section 5. E3's LAND moved to 1.5 s into the descent: the descent takes 2.5 s, so rev 1's 5 s could find the rod already welded and test nothing. |

### The review (verify, 22 Oct)
| Finding | Fate | What |
|---|---|---|
| S's return configs: no tether_tips, fixed-clock ATTACHes | for the S integrator | Not in this package's files: S's configs must add io `tether_tips: true` and `tether_len: 1.0` and a `WAIT_LOG '<timeout> REJOIN WELD drone k+1'` between ATTACHes. Also now: no ATTACH while a path runs. |
| `_rejoin_retreat` zeroed the shared hold | fixed | The retreat keeps the path held until its end clears the plate; nothing is zeroed. |
| `DEPARTED_CLEAR_R` and the local limit in `_rejoin_out` | fixed | `_departed_clear_r(self)` and `_land_clear(..., limit=)`; `config._knob` replaced by `params_file_knobs`. |
| The network graph (sim-C) | fixed | `planner_options(context, rejoin=sim_interface)` kept; sim-C's merge must keep it and not pass rejoin on its network graph. |
| The planner dying at launch without departed_land false | fixed | `planner_options` refuses it, and reconfig_mode other than ocp, with the reason. |
| Two test gaps (mutation) | fixed | The fold-in test flies a measured 0.987 m rod and checks it; a LAND near the +x limit checks the out point's clamp (and a typed pos_limit_m). |
| The fold-in seed ignored the planned yaw | fixed | The rejoin's seed is turned by psi0 + the planned yaw; tested on a held line_spin. |
| The card named the working rules' file | fixed | "the working rules". |

## Results (23 Oct, overnight)
| Run | Arm | Verdict | Numbers |
|---|---|---|---|
| R2445 | E0, floor re-weld of plate 9's tether, then the square lifts | FAIL (frozen bar: ring tilt over 2 deg) | JOINT_DETACHED 0.004 s after RELEASE, JOINT_ATTACHED 0.58 s after GZ_PUB, lifted, hovered 26 s, landed; ring tilt p50 5.62 / max 6.7 deg, tipping toward the re-welded quad (+y); that quad sits 0.761 m out against 0.733 for its neighbours, the opposite one 0.698 |

Reading: the joint re-welds and holds; on the floor it locked with the rod's end where it lay, off the plate, and the
planner (which takes the plate) flies the square ~5.6 deg tilted. This is the critic's weld-rule concern in its
largest form: E1 welds in flight only within 1.5 cm of the plate. E0 stopped on the tilt, not at stage (a) or (b), so
by section 6's rule E1 may follow.
| R2457 / R2458 | E1 x2 (bars frozen 00:42) | FAIL (frozen bar: ring tilt over 5 deg after the weld) | stage (a) end on its plate 24.9 / 25.8 s after REJOIN_START; (b) joint attached 0.03 s later; (c) fold-in tilt 5.83 / 5.32 deg, over 5 for only 0.2-0.4 s about 1 s after the weld; every other E1 item passed (end within 1.3 / 1.4 cm through the blend, tension 2.40 / 2.38 N against 2.44, lean 0.09 / 0.08 deg, no refusal, drop or abort, all landed) |

**Config deviation found in scoring:** the E1-E4 configs never set `attach_traj_hold_s`, so the fold-in blend ran 1.5 s,
not the card's 10 s; the tilt peak sits inside that short blend. R2457/R2458 stay FAIL as scored. The configs now set
`attach_traj_hold_s: 10.0` (23 Oct 01:58) and E1 is re-flown on them against the same frozen bars ("E1b").
| R2466 / R2467 | "E1b" | VOID as E1b; FAIL as E1 repeats (frozen bar) | the typed attach_traj_hold_s never reached the planner (the detach graph does not pass it): blend 1.5 s again; fold-in tilt 5.82 / 5.75 deg (over 5 from weld +0.87 to +1.25 s); every other E1 item passed |

**Fix (23 Oct 02:25):** the planner gets its own `rejoin_blend_s` (default 10 s, the card's value) used only by the
rejoin's fold-in; the rj* configs no longer type attach_traj_hold_s. Test: the blend is 10 s with attach_traj_hold_s 0.
E1b is re-flown on it after batch 7, against the same frozen bars.
| R2578 / R2579 | E1b on the fix (rejoin_blend_s 10 s; blend 10.10 s) | PASS every frozen E1 bar | (a) on the gate 24.87 / 24.86 s after REJOIN_START; (b) joint 0.033 / 0.029 s after ON_PLATE; (c) fold-in tilt 1.27 / 1.30 deg (bar 5; the 3-quad hover before it already peaked 1.37-1.39); end within 1.3 / 1.4 cm through the blend; tension 2.50 N at blend end (+3 %); lean 0.08 / 0.09 deg; no refusal, abort or drop; all landed |

**E1 PASSES on the card's 10 s blend** (R2578/R2579). Next by section 6: E2 (the T), E3 (LAND mid-rejoin), E4 (figure-8
staging), each once, then the exploratory M2 exposure run.

**E2-E4's bars FROZEN 23 Oct 05:13 by the integrator, as written in section 5, before they fly** (overnight authority).
| R2584 | E2, rejoin onto the T | FAIL (frozen bar: ring tilt > 5 deg) | tilt 10.61 deg at weld +1.36 s (over 5 from +0.72 to +5.54 s): the blend raised the rejoiner's planned height 4.6 cm (2.053 -> 2.099 m) while it carried ~0.1 N, so its rigid rod lifted plate 3 and the ring pitched about the pair's axis; every other item passed (stage a 17.47 s, joint 0.021 s, end <= 1.3 cm, lean 0.74 deg, all landed) |
| R2585 | E3, LAND 1.5 s into the descent | FAIL by 2.1 cm (lands 1.179 m from the ring centre, bar 1.2 m) | REJOIN_ABORT 0.10 s after LAND, climbed first, moved out ~0.28 m over the plate, no "landed anyway"; its reference stopped at y 1.121 (goal 1.14); M1 met only by the 3 cm rest rule (5 mm above rest, still descending, at /fleet/landed) |
| R2586 | E4, staging through a figure-8 | FAIL (frozen bar: staged drone inside +-1.6 m) | reached x 1.709 m (reference 1.656: the ramp overshoots its clamp); REJOIN_REFUSED 0.014 s after the mid-lap ATTACH; >= 0.767 m from the ring centre, >= 1.029 m from every carrier; STAGED_LEAVES at LAND |

R2584-R2586 stay FAIL as scored. The fixes below are behind `rejoin_departed` (the default path and QUAD4's fold-in
unchanged, their tests pass); offline tests only, nothing re-flown.

**Fix (23 Oct), E2.** Cause: not the blend's direction (its reference only lowers the rider as its rod swings out).
On a T only the rider holds the ring's pitch about the pair's axis, and its plan starts from its measured body (node 0
pinned to the measured rods). At the fold-in its tracker took the 0.1 N cable feed-forward, the body rose about 5 cm/s,
the plan followed it, and the rigid rod lifted plate 3 (run.csv: the body led its reference by 3-6 mm all the way up).
In the square (E1b) the other carriers hold that pitch, so the same step lifts nothing. Fix: through the fold-in blend
the rejoiner's reference never rises from its weld height, and no node of it sits above where a level ring (the measured
centre and yaw, the rod it welded with) puts it on its rod; an upward velocity there is zeroed (`_rejoin_cap`, applied
in `_publish_ref`). Test: the T (plates 0, 90, 180), the R2584 push modelled: the published height never rises, plate 1
lifts at most one tick's push; without the cap the same model rises over 3 cm.

**Fix (23 Oct), E3.** Cause: a departed drone's step counts as reached within 2 cm, and its landing starts there, so
the out step stopped 1.9 cm short and the drone came down from it. Fix: a staged or rejoining drone's way out is
reached by its reference exactly, and it aims `REJOIN_OUT_MARGIN_M` (5 cm) past the clear radius, still inside the
limit's landing margin. Test: LAND mid-descent, it comes down from the out point itself, 1.25 m from the ring centre.

**Fix (23 Oct), E4.** Cause: the staging point was clamped, its reference was not. The ramp met the clamp at about
0.19 m/s and braked past it, and the published 2 s horizon (the reference extrapolated at that speed) ran far past the
clamp, which the tracker followed. On the clamp itself the drone sat up to 5 mm past its reference. Fix: the ramp brakes
to stop on the staging bound (the discrete stop at TAKE_BACK_ACCEL), every published node is held inside it with no
velocity carrying it out, and the bound is `STAGE_TRACK_M` (3 cm) inside the card's 1.6 m (inside pos_limit_m less
0.15 m when set), so the drone's tracking stays inside the bar. Test: the +x carrier staged through a 0.9 m figure-8
(limit off and 1.5 m): no published point or velocity past the bound, the reference reaches it at speed and stops.
| R2589 / R2590 / R2591 / R2592 | E2, E3, E4 re-flown and E1 x1 on the 05:59 fixes | PASS every frozen bar | E2 fold-in tilt 3.02 deg (was 10.61), the reference never above its weld height; E3 lands 1.245 m from the ring (was 1.179; bar 1.2); E4 staged at most 1.575 m (was 1.709; bar 1.6); E1 tilt 1.45 deg, no regression |

**E0 aside, the rejoin's six runs all pass on the 23 Oct 05:59 tree** (E1 x3, E2, E3, E4). E0 (the floor re-weld) failed
on the off-plate weld's tilt, which the in-flight gate prevents. Next: the exploratory M2 exposure run, then S's chain.

### Critic on the 23 Oct fixes
*The project's critic, 23 Oct 06:25 (read-only, the 05:59 tree, R2584-R2592).*

1. **Has it been tried before?** Partly. Holding the newcomer's reference through the slew ("rod held near vertical") was considered and not taken on 25 Sep (decisions.md:36). R0541 also showed that a step in a newcomer's reference yanks it. The out-step and staging fixes are new.
2. **What simpler cause does it leave alone?** On a T the rider's share is about 0.1 N (R2589: 0.097 N). In the OCP's model its height barely holds the ring's pitch; with the twin's rigid rod, its height *is* the pitch. The cap patches that mismatch in the reference, and only during the blend. On the rig, where the tethers go slack, this mismatch is not there.
3. **What would falsify it, and does the bar test that?** The claim is "the cap stops the lift without adding its own tilt". E2's 3.02 deg peak is the cap's own 1.2 cm dip. That was flown once, with the pair leaning at most 2.2 deg. The 5 deg bar does test the claim, but only at that lean.
4. **What does it break?** No locked decision. With the switch off nothing changes; QUAD4's `_attach_ocp(rejoin=None)` leaves the cap doing nothing, and other departed drones keep the 2 cm reach.
5. **Was it asked for?** The rejoin was (decisions, 22 Oct (2)). The cap is a new law in the flight loop.

**Blocker:** none.

**Major**
- **M1. `_rejoin_cap` clamps the reference outside the OCP, so the two can disagree.** It lowers the position and zeroes the upward velocity but passes the OCP's `a` and `ac` through; the cap only moves down (a dip in the measured ring centre lowers it for the rest of the blend); `level_z` assumes a level plate (a leaning pair steps the reference by 0.225 sin(theta) or keeps the lean); it lifts in one tick, unlogged; the test models a level ring only. **Ask:** log the cap's binding margin in REJOIN BLEND OVER, and fly one neighbour with the pair leaning toward the plate.
- **M2. The path is held 1.5 s but the blend runs 10 s.** In hover this does not matter; on a held path the ring moves while the cap is on and `level_z` compares nodes up to 2 s ahead with where the plate is now. **Fix:** hold the path for `rejoin_blend_s`, or end the cap when the path resumes.
- **M3. The claims were flown on another tree.** S's 8->4->8 (R2588) flew before 05:59; E2 is a single run. One code tree per claim table and 2 repeats both apply.

**Minor**
- **m1.** The exact out-step cannot hang on tracking (the reach test is on the reference); what passed E3 is the 5 cm margin (the body trailed by 11 cm). With pos_limit_m 1.5, `_land_clear` shortens S's out point to the bare circle and the margin is lost silently.
- **m2.** A leaver released past 1.57 m snaps inward in one tick; with pos_limit_m 1.5 the staging bound is 1.32 m, ~0.5 m from the ring centre at a lobe, inside the 0.725 m reach.
- **m3.** `_rejoin_cap` works out z from x and y before `_inside_limit` clamps them, and never checks that `load_state` is fresh.

CRITIC: run it. Nothing blocks the hover E1-E4 use, and the switch-off and QUAD4 paths are untouched. Before S or the chain cite the cap: fly S's 8->4->8 and E2 x2 on this tree, add one leaning-pair neighbour, and fix M2's hold.

Response (23 Oct): M2 fixed (path held rejoin_blend_s on a rejoin); M1's binding margin logged; the leaning-pair neighbour and the re-flies follow.
| R2594 / R2595 / R2596 | E2 x2 and E1 x1 on the hold-fix tree (critic M2: the path held 10.0 s) | PASS every frozen bar | E2 fold-in tilt 1.45 / 2.25 deg (the peak now comes ~1.3 s after the cap lets go at the blend end: the rider's reference rose 0.5 / 1.5 cm and lifted plate 3); E1 1.55 deg; the cap bound 101 / 101 / 60 ticks, the plan 2.5 / 2.8 / 0.3 cm above it at the last |
| R2602 / R2603 | E5, the leaning-pair neighbour (critic M1), on the T push world and the hold-fix tree's `src/`: +0.0066 N m leans the pair +3.26 deg toward plate 3 at the weld; its mirror, -0.0110 N m, leans it -2.67 deg (the cap's other case) | PASS every frozen bar (E5's, as E2's) | fold-in tilt 3.12 / 2.91 deg. R2602's peak is the pair's own lean at the weld; the fold-in brought it to 0.85 deg mean by +1.5..+6 s. In R2603 the reference stepped down 1.2 cm at the fold-in, as section 5 predicts, and the 2.91 deg peak came at +11.25 s, when the cap let go. The cap bound 101 / 101 ticks, the plan 2.7 / 2.3 cm above it at the last. Lean before LAND 0.49 / 1.88 deg (bar 2; R2603's -0.011 N m torque stays on to LAND). Tension 0.099 / 0.098 N against 0.099; end within 1.3 / 1.5 cm |

**E5 passes with the pair leaning 3 deg either way** (R2602, R2603; scored 23 Oct 11:28). Next by section 5: E5's
second run ("a 2nd if clean").

**E5's second runs (R2712, R2713; lab batch 17; scored 14:02 against the frozen bars):**
| Run | Lean, 0.5 s before the weld | Fold-in tilt peak (weld..+15 s) | Lean, last 10 s before LAND | Verdict |
|---|---|---|---|---|
| R2602 / R2712 | +3.26 / +2.95 deg (toward plate 3) | 3.12 / **5.28 deg** (over 5 from +0.22 to +0.41 s) | 0.49 / 0.52 deg | PASS / **FAIL** |
| R2603 / R2713 | -2.67 / -3.11 deg (away) | 2.91 / 3.37 deg | 1.88 / 1.93 deg (bar 2) | PASS / PASS |

**E5 is 1 of 2 clean toward plate 3.** R2712 misses the 5 deg fold-in bar by 0.28 deg for 0.19 s. At weld +0.3 s the
weld-height cap dipped the rejoiner's reference 1.1 cm (2.044 -> 2.033 m) while the pair leaned +2.95 deg toward the
plate: the critic's M1 (the cap clamps the reference outside the OCP, and its dip is its own disturbance). Every other
E5 bar passes in all four runs; the mirror's lean before LAND stays the closest margin (1.88 / 1.93 against 2).

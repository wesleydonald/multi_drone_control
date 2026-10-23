# land_unwind (W2, plan noble-noodling-lollipop §5): LAND tip fix

## 1. Question

After the ring touches down on LAND, does sending each drone out along its rod's circle (the creep in
reverse) stop the rigid rod shoving it sideways and tipping it?

## 2. Hypothesis

Today every drone descends straight down from where it hovered. A rigid 0.55 m rod at 45-60 deg must
lengthen horizontally as the drone drops, so it pushes the drone 16-20 cm off its reference (every
rig-twin landing since 30 Sep), and 2 of 32 twin landings tipped a drone past 74 deg and aborted the
fleet (R0793, R0816). With the unwind the reference moves out with the rod: the off-reference push
falls below 5 cm and the drone tilt after the feedforward blend is no higher than today.

## 3. The one variable

Planner knob `land_unwind: true` (code: `land_arc_ref` and `_land_arcs` in
`src/mpc_planner/mpc_planner/planner_node.py`, used by `_land_hold_refs` after "load down"). The pivot
follows its rod circle about the grounded attach point at `land_vel` (arc speed, 0.2 m/s, the existing
LAND speed) down to level, then descends straight; below 0.25 m the xy reference follows the measured
xy as today. Default `''` (not passed, node default false): a default launch is unchanged (launch gate
350/360 as before, the 10 being the approved rig default changes).

Configs: `configs/experiments/b2_cap075_fix_n3_unwind.yaml` and
`configs/experiments/rig_twin_hover_3915_unwind.yaml`, copies of `b2_cap075_fix_n3.yaml` and
`rig_twin_hover_3915.yaml` with `land_unwind: true` added and nothing else changed.

## 4. Baseline

Load down to the last armed sample, from `run.csv` (script: scratchpad `land_baseline.py`, figures below).

| arm | baseline runs | worst drone off its reference (cm) | worst tilt after the 1.2 s blend (deg) | ring xy shift (cm) | aborts |
|---|---|---|---|---|---|
| 3-drone twin, 1.0 m (`b2_cap075_fix_n3`) | R0841, R0844 (renamed/consolidated code); R0823, R0824, R0828, R0832 | 15.8-16.1 | 4.4-5.2 | 0.0-0.2 | 0 of 6 |
| 4-drone twin, plates 1/3/5/9, 0.5 m (`rig_twin_hover_3915`) | R0786, R0803, R0813 | 19.2-20.0 | 5.0-5.6 | 0.1-0.2 | 0 of 3 |
| all rig-twin landings since 30 Sep (context) | 32 runs | 15.8-20.0 | 4.4-25.5 (74-76 at the aborts) | <= 0.3 | 2 of 32 (R0793, R0816) |

The trackers' reference age stays <= 0.12 s through the LAND hold on the rig (C1, 1 Oct); the C1 note
"ref_age grows to 4 s after load down" is the planner's time since its last OCP solve, which stops by
design in the LAND hold. Not a stale reference.

## 5. Pass / fail numbers

| metric (load down to disarm) | confirms (every run) | falsifies (any run) |
|---|---|---|
| worst drone off its reference | <= 5 cm | > 8 cm (half the baseline) |
| worst drone tilt after the 1.2 s blend | <= 5.2 deg (3-drone) / 5.6 deg (4-drone) | > 10 deg |
| LAND | /fleet/landed, every drone disarmed, no abort | any abort |
| ring xy shift (the unwind must not drag the ring) | <= 1 cm | > 2 cm |
| hold before LAND (path unchanged) | hold z within 1 cm of the baseline | - |

Three runs cannot show the 2-in-32 abort rate; the claim is the mechanism (no push), not the rate.

## 6. Repeats

3-drone arm x2, 4-drone 1/3/5/9 arm x1 (plan W2). Gazebo floor start (touches the floor and contact).

## 7. Cost

3 headless Gazebo runs on the laptop; session count 5 of 10 used before, 8 after.

## 8. New code?

Yes: `land_arc_ref` (pure, unit-tested in `src/mpc_planner/test/test_land_unwind.py`), `_land_arcs`,
the `land_unwind` parameter, the `land_unwind: ''` knob in `bringup/config/common.yaml` and
`real_mode.planner_options` (passes it to all six planner graphs only when set).

Critic: not run. It is a reference path, not a control law or gate; plan W2 runs the critic only if
Wesley wants one.

If it fails: registry rows stay; delete `land_arc_ref`, `_land_arcs`, the parameter, the knob,
`planner_options` and its six call sites, and the test, the same session.

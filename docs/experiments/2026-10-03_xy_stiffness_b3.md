# xy_stiffness_b3 (W3, plan noble-noodling-lollipop §5): the xy error fix

## 1. Question

Wesley: work on the xy error fix first. Does a stiffer ring-position cost, or nominal rods at the
measured hand-over angle (B3), hold the ring within 3 cm against a steady 1 N sideways push, without
making hover or the orbit worse?

## 2. Hypothesis

The ring's xy is held only by what OCP nodes 1-20 plan: a P-only loop of about 4-6 N/m (plan
2026-10-02 Part B), so a steady 1 N push (the rig's -x drift force was 0.41-0.52 N) costs roughly
15-25 cm. Raising the planner's ring-position xy weight 60 -> 180 should cut that about threefold,
-> 600 below 3 cm. B3 removes the mismatch between the planned 45 deg rods and the 51-65 deg the rig
holds, so the plan stops expecting a climb and a widening that never come; its effect on xy is
expected to be small, its effect on the rod angle large.

## 3. The one variable (per arm; everything else pinned to the baseline config)

| arm | change | config |
|---|---|---|
| A0 baseline | none, plus the push | `b2_cap075_fix_n3_push.yaml` (copy of `b2_cap075_fix_n3.yaml`, push world, WRENCH events) |
| A1 B3 | `nominal_elev_from_handover: true`: the rod elevation measured at the hand-over (fleet mean) replaces 45 deg for s_ref and the nominal tension | `b2_cap075_fix_n3_push_b3.yaml` |
| A2 stiffness x3 | `w_load_pos_xy: 180` (planner ring-position x, y weight; default 60, set at run time, no rebuild) | `b2_cap075_fix_n3_push_w180.yaml` |
| A3 stiffness x10 (only if A2 > 3 cm) | `w_load_pos_xy: 600` | `b2_cap075_fix_n3_push_w600.yaml` |
| O orbit check (the passing arm) | that arm's knob on the three-drone orbit, no push | copy of `b2_cap075_fix_n3_orbit.yaml` |
| E even four-ring check (the passing arm) | that arm's knob, four drones on 0/3/6/9 at 1.0 m (Wednesday's first layout), with the push | copy of `b2_nocap_fix.yaml`, push world |

Push: a persistent 1 N force at the ring's centre along world -x (the rig drift direction), on at
hold + 5 s, off after 15 s (runner event `WRENCH`, Gazebo ApplyLinkWrench in a copy of
`three_rigid_ground_rig.sdf` / `four_rigid_ground_rig.sdf`). Three-drone rig twin, floor start, B2 on,
z_ki 0, target 1.0 m.

## 4. Baseline

No push run exists yet: A0 is flown first (x2) and is the baseline for A1-A3. Hover without a push:
R0841/R0844 (hold z 1.047-1.048 m, tilt mean 1.10-1.11 deg, 0 solve failures). Orbit: R0832 (three
drones, r 0.5 at 0.125 m/s, 1.0 m). Even four-ring hover at 1.0 m: R0822 (B2, natural breakaway).

## 5. Pass / fail numbers

| metric | confirms (both repeats) | falsifies (either repeat) |
|---|---|---|
| ring xy displacement under the push (mean over its last 5 s, from the pre-push hold mean) | <= 3 cm | not below half of A0's |
| hold z, no push window (vs R0841/R0844) | within 1 cm | worse by > 2 cm |
| hold tilt mean | <= baseline + 0.5 deg | > 3 deg |
| orbit (check O): xy error mean, tilt mean (vs R0832) | no worse by > 1 cm / 0.5 deg | worse by > 3 cm or tilt > 3 deg |
| B3 only: hold rod elevation minus the latched angle | within 5 deg | > 5 deg off, or a worse hand-over |
| any arm | 0 solve failures, no abort | an abort, or the push drives an oscillation (tilt > 5 deg) |

## 6. Repeats

2 per arm (Gazebo). Checks O and E: 1 each, for the arm that passes.

## 7. Cost

A0 2 + A1 2 + A2 2 (+ A3 2) + O 1 + E 1 = 8-10 headless Gazebo runs. The laptop is at 10 of 10 for
3 Oct: lab PC (Monday) or a new laptop session.

## 8. New code?

Yes, all behind knobs that default to today's behaviour:
- runner event `WRENCH <fx> <fy> <fz> [link]` (persistent wrench via gz transport, as
  `tools/sim_test/stage1_torque_test.py` does; a zero wrench clears it) and two push-world copies;
- planner knob `w_load_pos_xy` (default 60): `cost_set(k, 'W', ...)` on every stage at start-up;
- planner knob `nominal_elev_from_handover` (default false): at the hand-over latch the fleet-mean
  measured rod elevation into `cable_elev_deg`, then rebuild `_s_nom` and the reference builder.

Critic: not run. B3 is a reference and the stiffness a cost weight, not a control law or a gate
(CLAUDE.md); Wesley chose the values on 3 Oct (A1 latch, A2 180, A3 600). Reviewer: on the final
table, before the rig.

If an arm fails: its registry rows stay; its knob and code are deleted the same session. The WRENCH
event and push worlds stay (a test tool, not flight code).

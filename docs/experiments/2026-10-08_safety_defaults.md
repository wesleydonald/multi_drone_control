# safety_defaults (8 Oct 2026): model integral on resizing fleets; mode dissipative and detection on by default

## 1. Question

Wesley, 7 Oct: "Make everything use the model integral not the reference height integral and make the
default mode dissipative (is there any reason why we shouldnt do this)." Detection on by default
(8 Oct plan). Not live on the rig until twin-verified; branch `safety-defaults` (f2f7d74).

## 2. Hypothesis

- **M (model integral through a detach).** Today a fleet that can resize flies the reference height
  integral, because `int_mode auto` refused the model integral there. f2f7d74 keeps the learned ring
  force through a resize and freezes it in the reconfiguration hold. Expected:
  - the unannounced release flies as with the reference integral (tilt, ring drop and detection
    within the run-to-run spread);
  - the integral does not wind up during the release transient.
- **S (mode dissipative as the default carry).** The dissipative node in planner phase flies the same
  OCP as mpc mode, plus detection. Expected:
  - the figure-8 and circle carries track as in mpc mode (RMSE 3D and lag within run-to-run
    spread);
  - the detector never fires in a carry with no release.

## 3. The one variable

| arm | variable | config | code | baseline |
|---|---|---|---|---|
| M1 hover release | int_mode model (auto) on a resizing fleet | `w4_3915_release.yaml` | safety-defaults | R1140, R1145 (reference) |
| M2 circle release | the same | `w4_3915_release_orbit.yaml` | safety-defaults | R1149, R1150 (reference) |
| S1 circle carry | mode dissipative (detection 0.06) | `w4_orbit_0125_diss.yaml` | safety-defaults | `w4_orbit_0125.yaml` (mpc) x2, week4 |
| S2 figure-8 0.2 m/s | mode dissipative (detection 0.06) | `w4_fig8_const0200_diss.yaml` | safety-defaults | R1127 + 1 repeat (mpc), week4 |

## 4. Baseline

| run | numbers |
|---|---|
| R1140, R1145 | detected 0.31, 0.38 s; tilt peak 7.5, 7.9 deg; ring z -2.6, -2.8 cm; landed |
| R1149, R1150 | detected 0.27, 0.21 s; tilt peak 9.3, 8.7 deg; ring z -3.0, -2.8 cm; landed |
| R1127 | figure-8 0.2 m/s: lap 34.5 s; xy err mean 1.61 cm, max 3.23 cm; RMSE 3D 1.74 cm; delay 0.07 s |
| S1 baseline | flown tonight |

## 5. Pass / fail numbers

| metric | supports | falsifies |
|---|---|---|
| M: tilt peak release..+10 s | within baseline +2 deg | > baseline +4 deg, or any abort |
| M: ring z jump 2 s | within baseline +-1.5 cm | worse by > 3 cm |
| M: integral b after the release | changes < 1 cm during the hold | jumps in the hold |
| S: RMSE 3D, lag | within the two mpc runs' spread +20 % | > 1.5 x the mpc mean |
| S: false detection in a carry | none | any |
| S: LAND | landed, no abort | otherwise |

## 6. Repeats

2 per arm (Gazebo).

## 7. Cost

M1 x2, M2 x2, S1 x2 + 2 baseline, S2 x2 + 1 baseline = 11 Gazebo runs. The laptop flies the safety
arms from the worktree, the lab PC flies the week4 baselines (one slot).

## 8. New code?

Yes, on `safety-defaults`, and not new laws:
- `planner_node.resize_fleet` keeps `_lint.b` and calls `_apply_load_force()`;
- `_update_lumped` freezes in `_reconfig_hold_left`;
- the `resizes` refusal in `real_mode.planner_options` is dropped;
- `profiles.DEFAULT_MODE` is `dissipative`, legacy stays `mpc`;
- `common.yaml detach_detect_m 0.06`, legacy off.

Unit tests: test_resize_yaw_datum (2 new) and test_real_mode_launches (updated, 1 new).

The critic is not required: no new law or estimator, and no gate threshold.

If falsified, the arm's change is reverted on the branch and recorded in the registry.

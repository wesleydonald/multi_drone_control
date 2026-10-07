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

## Results (8 Oct night)

**M: model integral through an unannounced release** (laptop, safety-defaults f2f7d74, against the
reference-integral runs):

| arm | runs | detected after | tilt peak release..+10 s | ring z jump | integral change in the 1.5 s hold | LAND |
|---|---|---|---|---|---|---|
| M1 hover | R1157, R1158 | 0.28, 0.23 s | 4.7, 4.1 deg | -2.7, -2.6 cm | 0.3, 0.3 cm | landed |
| baseline (reference) | R1140, R1145 | 0.31, 0.38 s | 7.5, 7.9 deg | -2.6, -2.8 cm | - | landed |
| M2 circle | R1159, R1160 | 0.28, 0.29 s | 4.9, 5.2 deg | -2.9, -2.6 cm | 0.0, 0.0 cm | landed |
| baseline (reference) | R1149, R1150 | 0.27, 0.21 s | 9.3, 8.7 deg | -3.0, -2.8 cm | - | landed |

- Detection and the ring drop are unchanged.
- The tilt peak after the release is about half that of the reference integral.
- In hover, the learned y force (-4.7 cm of integral, the leaver's pull) unwinds within 5 s of the
  release.
- **M SUPPORTS.**

**S: mode dissipative as the carry default.** The first comparison was confounded by the host: the
laptop flies ~0.05 s less lag than the lab PC with identical code (R1169/R1170 against R1152-R1154).
Same host (laptop), mode mpc against mode dissipative with detection on:

| trajectory | mpc | dissipative | A0 (s) mpc / diss | L0 (s) mpc / diss | tilt max (deg) mpc / diss | false detections |
|---|---|---|---|---|---|---|
| circle 0.125 m/s | R1169 | R1171, R1174 | 0.086 / 0.089, 0.084 | 0.186 / 0.191, 0.184 | 3.7 / 4.5, 3.1 | 0 |
| figure-8 0.2 m/s | R1170 | R1167, R1168 | 0.069 / 0.073, 0.048 | 0.168 / 0.176, 0.152 | 3.8 / 2.9, 5.9 | 0 |

- **S SUPPORTS.** On one host the two modes track the same, and the detector never fired in a carry
  with no release.
- R1161 and R1162 were contaminated by my own CPU load on the laptop (156 and 5 slow solves; R1162
  oscillated to 14.5 deg) and are excluded.
- The sim dissipative graph does not pass `ff_cap_force` (the lift's pull cap). It is 0 there, as on
  the rig, against 0.75 in the sim mpc graph; this affects only the lift.

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

## Results (8 Oct night; corrected after the reviewer)

Reviewer verdict on the first write-up: **weak**. Corrected below.

**M: model integral through an unannounced release.** The M arms flew on the laptop and the
reference-integral baselines on the lab PC (week4), and the host alone moves the twin's lag by
0.04-0.05 s. **So "tilt peak about half" is not yet a claim:** same-host reference-integral baselines
(`w4_3915_release_intref`, `w4_3915_release_orbit_intref`) are queued on the laptop.

| arm | runs (host) | detected after | tilt peak release..+10 s | ring z jump | LAND |
|---|---|---|---|---|---|
| M1 hover, model | R1157, R1158* (laptop) | 0.28, 0.23 s | 4.7, 4.1 deg | -2.7, -2.6 cm | landed |
| hover, reference | R1140, R1145 (lab PC) | 0.31, 0.38 s | 7.5, 7.9 deg | -2.6, -2.8 cm | landed |
| M2 circle, model | R1159, R1160* (laptop) | 0.28, 0.29 s | 4.9, 5.2 deg | -2.9, -2.6 cm | landed |
| circle, reference | R1149, R1150 (lab PC) | 0.27, 0.21 s | 9.3, 8.7 deg | -3.0, -2.8 cm | landed |

\* Second repeats: the runner shifted their events by the first repeat's lift wait (~10 s late; fixed
f694300). R1160 released ~143 deg further round the circle than R1159.

- **The integral in the hold:** it does not move inside the reconfiguration hold (0.00 cm). The
  0.3 cm is the first tick after it.
- **In the circle the integral is still** for another reason: the xy gate is off on a trajectory.
- **The hover unwind is confirmed:** -4.7/-5.1 cm fall to -0.35/-0.20 cm within 5 s.
- **int_mode is confirmed:** model in R1157-R1160, reference in the baselines.

**S: mode dissipative as the carry default.** Same host (laptop), lag on one clock
(`tools/lag_metrics.py`, clipped to the runner window), RMSE 3D from the reviewer:

| trajectory | run | mode | A0 (s) | L0 (s) | RMSE 3D (cm) | tilt max (deg) | false detection |
|---|---|---|---|---|---|---|---|
| circle | R1169 | mpc | 0.086 | 0.186 | 1.12 | 3.7 | - |
| circle | R1161 | dissipative | 0.107 | 0.176 | 1.38 | 3.2 | none |
| circle | R1162* | dissipative | 0.059 | 0.160 | 1.32 | 4.6 (in the run) | none |
| circle | R1171 | dissipative | 0.089 | 0.191 | 1.16 | 4.5 | none |
| circle | R1174 | dissipative | 0.084 | 0.184 | 1.13 | 3.1 | none |
| figure-8 | R1170 | mpc | 0.069 | 0.168 | 1.82 | 3.8 | - |
| figure-8 | R1167 | dissipative | 0.073 | 0.176 | 1.58 | 2.9 | none |
| figure-8 | R1168* | dissipative | 0.048 | 0.152 | 2.14 | 5.9 | none |

- **R1161/R1162 are kept.** I first excluded them as "contaminated by my CPU load". The reviewer
  showed that R1162's 14.5 deg tilt and both runs' 0.4-0.5 s solves came after run.csv ended
  (shutdown and post-landing rows).
- **Dissipative mean A0:** circle 0.085 s (4 runs) against mpc 0.086; figure-8 0.061 against 0.069.
- **The spread is large.** The dissipative repeats differ by up to 0.05 s in A0, more than the mode
  difference.
- **Only one mpc run per trajectory flew on this host**, so the card's "within the mpc runs' spread"
  bar cannot be applied yet; a second laptop mpc run per trajectory is queued.
- **R1168 did not land within the run** (repeat-shift bug).
- **Verdict so far:** no mode effect is visible and there are no false detections, but **S is not yet
  SUPPORTED by the card's bars**. Pending: the mpc repeats.
- **The `ff_cap_force` difference** (0 in the sim dissipative graph, as on the rig; 0.75 in sim mpc)
  affects only the lift.
- **Host effect** (identical code apart from the inert drop knobs): circle A0 0.131-0.144 on the lab
  vs 0.086 on the laptop; figure-8 0.104-0.106 vs 0.069.

## Same-host results (8 Oct, 04:30)

**M, laptop only, release metrics (`tools/detach_tensions.py`):**

| case | integral | runs | detected after | tilt peak release..+10 s | ring z jump |
|---|---|---|---|---|---|
| hover | model | R1157, R1158* | 0.28, 0.23 s | 4.7, 4.1 deg | -2.7, -2.6 cm |
| hover | reference | R1187, R1188 | 0.30, 0.27 s | 9.2, 7.2 deg | -3.3, -2.4 cm |
| circle | model | R1159, R1160* | 0.28, 0.29 s | 4.9, 5.2 deg | -2.9, -2.6 cm |
| circle | reference | R1190, R1199 | 0.26, 0.25 s | 8.0, 9.1 deg | -2.5, -3.2 cm |

\\* Released ~10 s late (the repeat-shift bug); R1160 released further round the circle.

- The lab PC shows the same direction with a smaller gap: acceptance run R1189 (model, defaults)
  6.2 deg against R1140/R1145 (reference) 7.5/7.9.
- **M SUPPORTS** on the card's bars:
  - tilt below the baseline, not above it +2;
  - ring z jump within +-1.5 cm;
  - the integral does not move in the hold (0.00 cm).

**S, laptop only, with two mpc runs per trajectory for the spread** (lag on one clock clipped to the
runner window, RMSE 3D = `payload_rmse_sweep_m`):

| trajectory | mpc A0 / RMSE | dissipative A0 / RMSE | bar (mpc + 20 %) |
|---|---|---|---|
| circle | R1169 0.086 / 1.12, R1200 0.096 / 1.27 | R1161 0.107 / 1.38, R1162 0.059 / 1.32, R1171 0.089 / 1.16, R1174 0.084 / 1.13 | A0 <= 0.115 s, RMSE <= 1.52 cm |
| figure-8 | R1170 0.069 / 1.82, R1201 0.065 / 1.30 | R1167 0.073 / 1.58, R1168 0.048 / 2.14 | A0 <= 0.083 s, RMSE <= 2.18 cm |

- **S SUPPORTS:**
  - every dissipative run is inside the mpc spread + 20 % on lag and RMSE;
  - no detector fired in 6 carries;
  - no abort.
- Caveat: R1168 did not land inside its run (the repeat-shift runner bug, fixed f694300).

**Acceptance:** the branch defaults alone resolve to dissipative, detection 0.06, the 0.5 m
step-out and the model integral, and fly U1 (R1189: detected 0.25 s, 6.2 deg, landed).

**What merging safety-defaults changes on the rig profile** (`real/dissipative`, week4 vs the
branch):
- default mode mpc -> dissipative;
- detach_detect_m '' -> 0.06;
- the drop knobs added (off by default);
- int_mode model for detach flights (launch rule).

# cable_mocap (7 Oct 2026): trackers model the rod pull they measure from mocap

## 1. Question

Wesley, 7 Oct: "it seems like the measured cables work better ... do some runs with that and then
if it works better make it the default."

## 2. Hypothesis

With `cable_source: mocap`, each tracker's model carries the rod pull the drone actually feels (mocap
acceleration minus the model's thrust, gravity and drag) instead of the planner's planned pull. The
drones then stop being drawn inward by a pull the model under-states.

| quantity | default (planned pull) | expected with mocap |
|---|---|---|
| rod elevation in hover / orbit | 53-54 deg (twin), 58-67 (rig) | 45-48 deg (twin R1103: 45) |
| ring height, orbit xy error | as baseline | within 0.5 cm of baseline |
| drone lean 1.2 s after touchdown | about 2 deg | about 2 deg (R1103 before the touchdown blend: 19.3) |

## 3. The one variable

Tracker `cable_source: mocap` with `cable_accel_cap: 12` (params file), the touchdown blend included
(the measured term slews back to the planned one once the ring is within 5 cm of the floor). Everything
else pinned to each arm's baseline config:

| arm | config (with `params_file` cable_mocap) | baseline config |
|---|---|---|
| hover + orbit, 10 s hold | `w4_orbit_cablemocap.yaml` | `w4_orbit_delay_c.yaml` without delay = R1074 config + 10 s hold |
| figure-8 | `w4_fig8_cablemocap.yaml` | `w4_even_fig8_def.yaml` |
| spin | `w4_spin_cablemocap.yaml` | `w4_even_spin_lf.yaml` |
| 0.4 N push, orbit, 8 s hold | `w4_orbit_push_cablemocap.yaml` | `w4_orbit_push_hold.yaml` |
| three drones, hover | `n3_hover_cablemocap.yaml` | `b2_cap075_fix_n3_def.yaml` |

## 4. Baseline

| arm | run | numbers |
|---|---|---|
| orbit | R1074 | xy err 0.8 cm, lag 0.02 s, wobble 0.8 cm, rods 53-54 deg, LAND lean 1.9 deg |
| hover (4 drones) | R1072 | z +0.0 cm, xy 0.02 cm, tilt 1.5 deg, LAND lean 2.3 deg |
| figure-8 | R1075 | xy 0.75 / 2.3 cm, z -1.4 cm, LAND lean 2.3 deg |
| spin | R1069 / R1071 | xy 0.49 / 1.09 cm, yaw err 1.3 / 3.2 deg |
| push orbit, hold | R1091 | offset -0.2 cm, lap 0.58 cm, tilt 4.3 deg |
| three drones | R1077 | z +0.1 cm, tilt 1.5 deg, LAND lean 2.2 deg |

All on the rig twin, laptop, floor start, the 5-6 Oct defaults (the `model` path is unchanged since).

## 5. Pass / fail numbers

| metric | supports (make it the default) | falsifies |
|---|---|---|
| aborts, tips | none in any run | any |
| drone lean 1.2 s after touchdown | <= 3 deg in every run | > 5 deg in any run |
| rod elevation, hover and orbit | <= 50 deg | not below the baseline's 53 deg |
| orbit / figure-8 / spin xy error mean | <= baseline + 0.5 cm | > baseline + 1.0 cm |
| ring height error, ring tilt mean | <= baseline + 1 cm, + 1 deg | worse by more |

## 6. Repeats

Neighbours first, one run each (hover + orbit, figure-8, spin, push, three drones); the arms that pass
get their second run.

## 7. Cost

5 + 5 = 10 Gazebo runs, about 80 min on the laptop, one at a time (no session cap on the laptop since
4 Oct). The lab PC is preferred after the laptop's near-crash on 7 Oct.

## 8. New code?

Yes, already in the tree, off by default:
- `tracker_node.py`: `cable_source: mocap`, `_mocap_accel()` (least-squares slope of the mocap velocity
  over 80 ms, then a 50 ms low-pass) and the touchdown blend.
- `configs/rig/cable_mocap.yaml`.

If falsified, it stays opt-in for one more rig look or is deleted the same session (Wesley's call).

## Rig-log replay bar (stated before the replay, 7 Oct)

The measured pull is usable on the rig if, per drone in hover and on the circle (RIG-1007 r002, r006):
mean bias against the planned pull <= 1.5 m/s^2 (about 30 % of the ~5 m/s^2 pull), noise sd after the
filter <= 1.0 m/s^2, and the 12 m/s^2 cap active in < 1 % of ticks. Computed both ways: thrust term
raw (as coded) and synchronised with the acceleration filter (critic must-fix 1).


## Rig-log replay result (7 Oct, no flights)

The tracker's mocap estimator replayed over RIG-1007 r002 (hover) and r006 (circle), r0001, per drone:

| | along the cable vs planned | noise sd | cap | measured - planned, off the cable |
|---|---|---|---|---|
| r002 | bias -0.08 to +0.19 m/s^2 (planned 5.1) | 0.13-0.27 | 0 % | 0.8-1.3 m/s^2 |
| r006 | bias -0.06 to +0.17 m/s^2 (planned 5.0) | 0.14-0.27 | 0 % | 0.8-1.3 m/s^2 |

- Bar met on every drone (bias <= 1.5, sd <= 1.0, cap < 1 %). The synchronised thrust term lowers the
  sd by 0.03-0.05 m/s^2.
- The planned cable direction matches the real one (drone pivot to plate, mocap) within 1.2-1.4 deg, and
  the pull along the cable within 1-4 %: **the cable model is right on the rig**.
- The residual is a force on each drone **not along the cable**: radially inward 0.7-1.2 m/s^2
  (about 0.55 N), upward 0.1-0.6 m/s^2, tangential about 0, on all four drones in all three runs. It is
  not a body-axis bias: in the body frame it points along whichever axis faces the ring, and drone 3's
  turned with its heading (r002 heading +3 deg: body -y; r0001 heading +91 deg: body -x). Most likely
  aerodynamic (the drones' wakes in the formation). It is this morning's "drones lean 3-5 deg further
  out than the model needs", and it pulls the cables steep. The twin has no such force.
- So the full 3-D measured force (not its projection on the cable, removed again) is what the rig needs.
- Must-fix 1 (thrust synchronised with the acceleration filter, raw throttle in mocap mode; pytest
  test_cable_mocap.py), 2 (gaps hold the applied value) and 3 (this replay) are done; 7's rig shadow run
  is this replay (round 1 flew `model` and logged everything the estimator needs).


## Twin matrix result (7 Oct)

| arm | baseline | measured | path err mean (cm) | lag (s) | hover z / xy sd (cm) | cables (deg) | ring tilt (deg) | LAND lean (deg) |
|---|---|---|---|---|---|---|---|---|
| orbit, 10 s hold | R1105 | R1106 | 0.66 -> 1.43 | 0.04 -> 0.11 | 0.5/0.2 -> 0.1/0.0 | 53.6 -> 45.4 | 1.6 -> 0.5 | 1.8 -> 2.1 |
| orbit + 0.4 N push, 8 s hold | R1091 | R1115 (offset form) | 0.63 -> 1.62 | 0.03 -> 0.11 | 0.5/1.6 -> 0.0/0.3 | 53.5 -> 45.2 | 4.2 -> 1.0 | 1.8 -> 1.8 |
| figure-8 | R1075 | R1107 | 0.65 -> 0.90 | - | - | 54.0 -> 45.3 | 1.3 -> 0.4 | 2.3 -> 1.7 |
| spin | R1071 | R1116 | 0.61 -> 0.59 (yaw err 1.2 -> 1.9 deg) | - | - | 54.0 -> 47.1 | 1.0 -> 0.3 | 1.5 -> 1.6 |
| three drones, hover | R1077 | R1110 | - | - | 0.4/0.2 -> 0.1/0.0 | - | - | 2.2 -> 2.4 |

Voids (laptop pose timeouts): R1108, R1109, R1111, R1112, R1113; R1114 flew the default (the runner
dropped the params file; fixed in tools/launch_args.py).

**Verdict against section 5:** no abort or tip; landing lean 1.6-2.4 deg (bar 3); cables 45-47 deg (bar
50); ring tilt down by two thirds; hover 5x steadier. **Fails the path bar on the orbit** (+0.77 and
+0.99 cm, grey zone): the measured term adds about 0.07 s of lag on a circle. The offset form
(planned_k + measured - planned_0, critic should-fix) did not change it (R1115 0.11 s, as R1106).
Not made the default; it stays opt-in for the rig comparison (round 2), where the rig's own 0.45 s lag
and the 0.55 N inward force dominate.

## Critic

Critic agent, 7 Oct 2026 (read-only), condensed; verdict: **rewrite the card** - the twin arms test a
different estimator from the one the rig would fly, and the bars cannot fail on the rig's real risks.

**Tried before.** INDI throttle R0238-R0272 (removed 23 Sep); `cable_source measured` (IMU) 4 Oct,
LAND tips in 4 of 6 runs; the rod-angle effect was also produced by option F (planner `rod_mass` 0.075,
R0868: 49.7 -> 42.8 deg), still in the code.

**Simpler explanation.** The twin's steep rods come from the rod weight being counted on the drones
in the planner model (R0868); the measured pull hides that model error instead of fixing it. On the
rig the drones track their references within 1 cm, so the lag and the bob come from the references
and the loop delay; this change adds filter lag (R1103: orbit lag 0.02 -> 0.06 s).

**Falsifier.** The rod bar is already answered (R1103); the other bars are equivalence bars a no-effect
change passes. The matrix never exercises the rig risks: the raw-throttle path, mocap noise and
latency, the thrust-map error folded into the pull.

**Breaks if made the default in common.yaml.** Legacy kt_trim silently off; detach / attach / m2
trackers change unflown; int_k_xy/int_k_z were identified with the planned term; the thrust-map error
is absorbed (against the 30 Sep "judge the model with the integral off").

Must-fix (before the matrix):
1. **Twin and rig compute different signals**: `_thr_lpf` exists only with the sim IMU; the rig
   subtracts the raw throttle while the acceleration lags ~90 ms (twin) to ~130 ms+ (rig: the mocap
   node's velocity EMA). A throttle change shows at once as a pull change of 35.2*dthr (loop gain ~1).
   Fix: in mocap mode put R*kT*thr through the same sample times, window and low-pass as the
   acceleration, independent of the IMU; pytest: a throttle step with the true acceleration constant
   leaves the pull unchanged.
2. **Gaps step the term**: `_mocap_accel` None -> the planned term with no slew; samples de-duplicated
   by value (velocity rounded to 1 mm/s); stamped with the tick time. Hold or slew; de-duplicate by
   arrival.
3. **Replay the rig logs offline first** (r0001, r002, r006, r0007): per drone bias against planned,
   noise sd, cap and slew activity, behaviour in the bob; state a rig bar before reading the numbers.
4. **Test the mechanism and its rival**: the mocap pull in shadow on R1074/R1072 (sign and size against
   planned); an option F (rod_mass) arm on the 4-drone orbit.
5. **Neighbour arms where the rig risk lives**: per-drone thrust-map error both signs; mocap delay only
   (0.03 s, no command delay), both modes; the lift transient (ring vz max, bar 0.3 m/s).
6. **Orbit baseline confounded** (R1074 no hold vs 10 s hold): fly the same config without the change.
7. **Scope of "default"**: twin carry only; refuse it in legacy / detach / attach / m2; the rig gets a
   shadow run first (log the measured xyz, fly `model`), then one rig hover in mocap mode.

Should-fix: the measured pull as a correction to the planned horizon (planned_k + (meas - planned_0))
instead of a constant (keeps the lift / LAND ramps, orbit rotation, a future detach step); key the
touchdown blend to the /fleet/cable_ff_active edge with hysteresis; add lag/wobble and the push arm's
learned F to the bars, grey zones = not supported; `cable_ff_scale` applied twice on the measured path.


## Rig round 2 result (7 Oct, afternoon)

Rig, 4 drones, even ring, model integral, 5 s hover hold; method 1 = round 1 feedforward (planned
45 deg, flew ~60), method 2 = measured pull with a 60 deg plan. From +4 s on the path to LAND.

| path | runs (method 1 / 2) | path err mean / max (cm) | lag (s) | ring z sd (cm) | tilt mean / max (deg) | cables (deg) |
|---|---|---|---|---|---|---|
| hover (5.3 s) | r002 / r20004 (45 deg plan) | xy sd 2.4 / 0.6 | - | 1.1 / 0.2 | 4.3 / 1.3 (means) | 58-65 / 44-48 |
| circle r 0.5, 0.125 m/s | r006 / r206e60 | 8.25 / 17.3 -> 2.91 / 5.7 | 0.45 -> 0.20 | 4.8 -> 0.5 | 2.7 / 6.6 -> 1.7 / 4.0 | 60.6 / 60.5 |
| figure-8 a 1.0, 0.25 m/s | r0007 / r2008e60 | 8.2 / 20.1 -> 3.0 / 6.8 | 0.41 -> 0.21 | 4.7 -> 0.6 | 3.6 / 10.7 -> 2.6 / 5.9 | 60.3 / 60.6 |

- With a 45 deg plan the measured pull flies the cables at 45 deg (~3.0 N each), and drone 2's magnet
  let go 3.9 s into the circle (r200006, fault on the descent). Wesley (7 Oct evening): the magnets
  hold about 9-10 N; the release came from a sharp jerk, not the steady load. At 60 deg: ~2.5 N, no slip.
- Feedforward already flew ~60 deg (the 0.55 N inward non-cable force), so method 2's cables match
  method 1's: the gain is the plan and the trackers' model matching the real pull.
- One run per path; cap 0 % throughout. Figures: results/rig/2026-10-07/figures/
  circle_method1_vs_method2.png, figure8_method1_vs_method2.png.

**Decision (Wesley, 7 Oct):** first the rig carry default (real.yaml modes.mpc), then the default
everywhere (common.yaml: rig and twin, every mode; legacy profile kept on model / 45). Twin run
waived by Wesley.

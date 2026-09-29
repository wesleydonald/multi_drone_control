# 2026-09-30 Rig carry hover: three on plates 1/5/9, then four on 1/3/5/9 (G7 R3b + R4, rig go/no-go)

Written 2026-09-28, before any of it is flown. Command sheet: tests.txt, section "WED 30 SEP 2026 (G7 R0b-R4)".
Plan: docs/g7_real_world_ladder.md §3 (R3a-R4). Operator: Wesley; hand kill = the panel DISARM (spacebar), which latches every drone (Wesley 2026-09-28).

## 1. Question
Can our stack lift the 0.86 kg M2A ring off the lab floor and hover it at 0.60 m? First three drones on plates 1/5/9
(R3b), then four on plates 1/3/5/9, the M1 carrier layout (R4). Nobody has lifted the ring on the rig yet: in every
real log since 09-16 it stays at z 0.050-0.053 (ladder §0).

## 2. Hypothesis
The fleet starts from the floor on the creep path (start_taut false, sweep to 45 deg), with the typed kT from R1 and
the R3b calibration flight, kt_trim on and z_ki 0.4. It then lifts the ring and holds it at 0.60 m within the sim
twin's band: settled 0.600 +-0.03, ring tilt settled < 5 deg, no envelope fault. R4 is as level as its sim twin
(R0510: settled tilt 0.7 deg).

## 3. The one variable
Sim -> rig: the same control graph (real_control_launch = the hardware twin of mpc_quad_load_launch) with the same
arguments. Only the plant and the rig's own numbers change: weighed masses, the measured rod (~0.47 vs the sim 0.50),
the measured kT (sim 83.1 linear).

Between R3b and R4 the only change is the fourth drone on plate 3: azimuths 30,150,270 -> 30,90,150,270, DM3 -> DM4.

Rig lines (full text in tests.txt):
- R3b claim flights: `real_control_launch.py num_drones:=3 load_mass:=$RING drone_mass:=$DM3 cable_len:=$ROD
  attach_radius:=0.25 attach_z:=0.0 attach_azimuths_deg:=30,150,270 thrust_ratio:=$KT kt_trim:=true z_ki:=0.4
  start_taut:=false handover_elev_deg:=45.0 handover_settle_s:=1.0 creep_vel:=0.2 target_z:=0.6 lift_ramp_vel:=0.15
  load_traj:=hover`.
- R4: the same with `num_drones:=4 drone_mass:=$DM4 attach_azimuths_deg:=30,90,150,270`.

Not part of the claim:
- R3a (creep and HOLD, `lift_ramp_vel:=0.0`, trim off);
- the R3b calibration flight 1 (`kt_trim:=false`, reads the tethered "[kT dN] measured in hover" value that becomes
  `$KT`).

Both get exploratory registry rows.

## 4. Baseline (sim rehearsals, Gazebo headless, flown before Wednesday; ids filled when flown)
| rung | sim twin config | run id | reference numbers already on file |
|---|---|---|---|
| R3a | `ocp_hover_ground_creep_hold.yaml` (new: defaults, lift_ramp_vel 0, trim off) | R0726: ring floats 2.2 cm at full load after a 5.5 cm hand-over overshoot (does not stay down); drone tilt 18.8 | none (first HOLD on the creep path) |
| R3b | `ocp_hover_ground_creep_defaults.yaml` (correct kT, trim on, z_ki 0.4) | R0721 (gyro default): z 0.571 settled window, tilt peak 0.61 | R0558/R0565/R0566: last 20 s 0.600 (0.599-0.601), tilt 0.03-0.1; R0648-R0650: drone tilt 18.6 in hover |
| R3b | `ocp_hover_ground_creep_kt110_kt.yaml` (new: the same, typed kT +10 %) | R0727: peak 0.644, tilt peak 0.48, no abort | none on the creep path; taut-start sibling `ocp_hover_ground_kt110_kt` in card 2026-09-23_kt_trim |
| R4 | `ocp_hover_ground_creep_n4_3915.yaml` (four on 30/90/150/270) | R0729 (gyro default): last 15 s z 0.599-0.600, ring tilt 0.64 max / 0.59 mean, drone tilt 20.7 | R0510: load 0.600 from 26 s, tilt 0.1-0.9 (settled 0.7) |

The three-drone twins fly the even ring 0/120/240 (three_rigid_ground.sdf). That is plates 1/5/9 rotated by 30 deg,
so the geometry is the same. The rig's numbers are compared against these rows. They are not expected to be equal: a
Gazebo pass is the precondition, and the gap is the thesis number.

## 5. Pass / fail numbers (fixed now; per flight, from the planner tick log + tracker logs)
Measured from `load_planner_<time>/log.csv` (payload z and tilt at 10 Hz) and `planner_drone<i>_<time>/log.csv`
(drone attitude, throttle, kt_hat). "Settled" = the last 10 s before LAND. "Lift time" = TAKEOFF to the payload
first above 0.55 m.

| metric | supports | falsifies (any one fails the flight) |
|---|---|---|
| payload z settled | inside 0.55-0.66 (h0_h1 T2 / G2 bar); whether it is within 0.60 +-0.03 is reported as the z_ki number, not a bar | outside 0.55-0.66 |
| ring tilt | peak < 15 deg, settled mean < 5 deg (R4: <= 5 deg) | peak >= 15 deg, or settled mean >= 5 deg |
| drone tilt (max, airborne) | < 25 deg | >= 25 deg |
| lift time | < 40 s (sim: handover ~14 s after TAKEOFF, then ~5 s of eased ramp; 2x margin) | >= 40 s, or a creep / liftoff-gate timeout |
| faults | none; LAND with all disarmed | any envelope fault, any operator ESTOP/abort |

Abort (not a data point; the flight counts as FAIL): global list in tests.txt, pack under 21.6 V, ring tilt > 15 deg
for > 1 s. A flight lost to a non-control cause (radio link, mocap dropout traced in the log, a battery under the
24.0 V start bar) is VOID with the cause written in its row, and it is re-flown.

## 6. Repeats
The rig: 2 claim flights per arm (R3b, R4), both on the same typed `$KT`. A claim holds only if both pass. One pass
and one fail = WEAK, reported as 1/2, not re-flown to a 2/3 the same day without a named cause. Gazebo twins: one run
each for the rehearsal (they are the baseline, not the claim).

## 7. Cost
- Gazebo: 4 headless runs before Wednesday (the three twins that are not yet flown on the current sim default, plus
  the n4 re-fly), within one session's 6.
- Rig (≈5 h lab):
  - R3a 2 flights;
  - R3b 1 calibration + 2 claim flights;
  - R4 2 claim flights (only if R3b passes and the 4th airframe passed R1 + R2).
- Never Gazebo/SIL with the rig stack up.

## 8. New code?
No flight-loop code. The desk tools changed for the lab: preflight's n-drone disc fit and azimuth check, and
fake_mocap's layout flags. Both run before ARM, and neither is in the flight loop, so no critic (working rules). Reviewer:
once, on the final rig table, before it goes into the thesis or is used as the go for R5/R6.

## 9. What is logged per flight
- `results/logs/controller_quad_load/load_planner_<time>/`: planner tick log (payload z, tilt, phase) and params.json.
- `planner_drone<i>_<time>/`: tracker log (u0-u3 with u2 the throttle, pose, qref, solve_status, kt_hat) and
  params.json.
- `results/preflight/<ts>.md`: GO card, read-back, disc fit, azimuth match.
- `results/rig/2026-09-30/<flight>.log`: T2 stdout (the `[kT dN]` lines, which are printed but not logged).
- `results/rig/2026-09-30/<flight>/`: ros2 bag (mocap, ELRS commands, telemetry, fleet topics).
- `wed.env`: the numbers typed.
- One registry row per flight, id prefix H, card column = this file for the claim flights.

## 10. Decision each outcome leads to
| outcome | decision |
|---|---|
| R3b 2/2 and R4 2/2 pass | rig carry hover GO. Next visit: R5 orbit (after the cage footprint, Q5) and R6 detach on real_dissipative (yaw datum fix in place since 2026-09-28). The rig-vs-sim table goes to the reviewer. |
| R3b passes, R4 fails | three-carrier hover is the rig baseline. R4's failure mode picks the next step: a tilt means the uneven 4-ring's balanced tensions on hardware; one drone off means that airframe's kT or magnet. Hold R5/R6 (they need four). Wesley decides whether to retry R4 next visit or fall back to an even 4-ring. |
| R3b fails on height only (z outside the band, tilt fine) | kT/trim question, not geometry. Read kt_hat and zI in the logs. Re-fly once with the printed kT (time box: two runs). Then stop and report. |
| R3b fails on tilt or a fault | no R4. Read the log first: which drone, what phase (creep, handover, lift, hover). Report with the log. Next step is Wesley's. |
| R3a fails (no handover, rods not at 45, ring above 0.15 m in HOLD, not coupled, or a drone at the 0.6 throttle cap for 2 s) | no lift today. The creep on hardware is the problem: rod length, Motive origin, or creep gate. Take the preflight card and the planner log home. |
| any VOID (radio, mocap, battery) | fix the cause, re-fly. The fix goes in the row. |

## Results (filled at the lab)
| flight | rung | KT typed | kt_trim | pack V start/end | lift time s | payload z settled | ring tilt peak / settled mean | drone tilt max | fault | verdict | T2 log | registry |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| | R3a f1 | | false | | — | — | | | | | r3a_f1.log | H |
| | R3a f2 | | false | | — | — | | | | | | H |
| | R3b f1 (calibration) | | false | | | | | | | | r3b_f1.log | H |
| | R3b f2 (claim 1) | | true | | | | | | | | r3b_f2.log | H |
| | R3b f3 (claim 2) | | true | | | | | | | | | H |
| | R4 f1 (claim 1) | | true | | | | | | | | r4_f1.log | H |
| | R4 f2 (claim 2) | | true | | | | | | | | | H |

# 2026-09-23 bounded vertical integrator per drone

## 1. Question
Wesley: "Try the vertical integrator per drone." Does a bounded integrator on each
drone's own vertical tracking error, in the OCP-path MPC tracker the rig flies, remove
the steady hover error a wrong typed thrust ratio or load mass leaves, without the
load tilt that the earlier per-drone (3-axis) integrators produced?

## 2. Hypothesis
With `z_ki` on, a +10 % / +15 % typed kT hovers within 2 cm of the 0.60 target (off:
0.498 / 0.462), +20 % unmodelled mass within 2 cm (off: 0.511), and the correct case
moves from 0.629 to within 2 cm of 0.60. Settled load tilt stays under 3 deg (off:
0.5–2.6 deg). The integrator converges within 15 s and never rails.

## 3. The one variable
Tracker parameter `z_ki` (1/s^2, default **0** = off) with `z_i_max` (m/s^2, 1.5) and
`z_i_node` (which horizon node's height is the setpoint, default the LAST node). Per
tick, when armed, TAKEOFF requested and the payload not resting:

    e_z = ref_z[z_i_node] - z_measured
    I  += z_ki * e_z * dt,  clipped so |I| <= z_i_max
    a_cable_applied = a_cable_planned + [0, 0, I]

i.e. the integral rides on the existing cable-feedforward channel into the tracker's
MPC as an extra vertical specific force, so the MPC's thrust rises by exactly I. Frozen
(not zeroed) while the payload rests or the fleet is not flying; zeroed on ARM.

Why the LAST horizon node and not node 0: the node-0 drone reference is pinned to the
drone's own measured position (critic, effective-load-force card, point 5), so its error
is zero by construction. The end-of-horizon reference is where the planner wants the
drone to be; at hover it is the target geometry, so the integral drives the drone to the
planned point and the load to its target height through the rods.

Why vertical only: the 3-axis per-drone integrators (`vel_ki` 1, R0077 series) tilted
the load to 60 deg because lateral integrals fought each other through the rods. A
vertical integral on each drone's own error has one equilibrium per drone (its own
planned height) and cannot fight laterally.

What it is not: not the network's common-mode trim (that lives on the dissipative path
and acts on load height); not the load-level Kalman filter (deleted 2026-09-23: it
converged to its own fixed point, not the target).

## 4. Baseline
Second SIL matrix, estimator off (one tree 3b33588d+17), hover-only window, LAND at 46 s:
correct 0.628/0.629 (R0355/R0356), kT +10 % 0.498 (R0357, R0363–R0364 re-fly), kT +15 %
0.462/0.461 (R0359/R0360), mass +20 % 0.511 (R0361, R0365–R0366 re-fly). Re-flown for
this card on the new tree (estimator removed, INDI removed), same configs with the
`z_ki` arg added, off vs on.

## 5. Pass / fail numbers
| arm (SIL, free) | supports (z_ki on) | falsifies |
|---|---|---|
| correct mass, kT | within 2 cm of 0.60; settled tilt < 3 deg; \|I\| < 0.5 m/s^2 | > 2 cm from 0.60 both repeats, or tilt > 3 deg, or I railed |
| kT +10 % | within 2 cm of 0.60, tilt < 3 deg, no oscillation > 2 cm p-p | > 3 cm low, or tilt > 3 deg, or p-p > 3 cm, or railed |
| kT +15 % | within 3 cm of 0.60, tilt < 3 deg | > 4 cm low, tilt > 3 deg, railed |
| mass +20 % | within 2 cm of 0.60, tilt < 3 deg | > 3 cm low, tilt > 3 deg, railed |
| attach demo smoke (SIL `attach_ring_45`), z_ki on | weld completes, post-weld tilt not worse than off by > 3 deg | abort, or I winds up through the weld (> 1 m/s^2 within 5 s of the weld) |

Gazebo (kT +10 %, off vs on, 2 repeats = 4 runs) only if SIL supports; 3 runs remain
today, so tomorrow.

## 6. Repeats
2 per arm.

## 7. Cost
SIL: 10 configs x 2 = 20 runs, free. Gazebo: 4, deferred.

## 8. New code?
Yes, ~40 lines in `controller_mpc.py` (three parameters, the integral, the freeze/reset
rules, a log column `z_int`), the launch arg `z_ki` on the seven OCP launches, a unit
test on the integral's clip and freeze. Removal plan: delete the block and the arg.

## Critic (2026-09-23)

**1. The sign is wrong: positive feedback.** `dynamics.py:118` is `v_dot = R·thrust − g + a_cable` and a taut cable's pull has negative z, so `+[0,0,I]` tells the model something extra is lifting the drone: the MPC lowers thrust, the drone sinks, I rails at +1.5. Needs `−[0,0,I]`, and the unit test must check the direction (drone below reference → throttle rises).
**2. The last horizon node is a lead, not a setpoint, whenever the plan moves.** Node N is 2 s ahead: up to 0.15 m during the SIL lift, 0.44 m on a ground lift, the same below during LAND and the attach approach. A type-1 loop fed a ramp lead winds by `z_ki·lead·t` on every lift and overshoots at hover; none of the gates excludes a ramp. At hover node N is near the target (R0284: zN 0.60 / d0N 0.97 vs 0.979) but is reached from a node 0 pinned to measured values, so part of the measured tilt/elevation may survive to node N and the integral chases its own measurement: unchecked. A measurement-free setpoint exists in the tracker: `payload_ref` plus this drone's nominal rod offset.
**3. Vertical only does not remove a kT error.** Vertical specific thrust at hover ≈ 14.5 m/s²; required I: kT +10 % 1.45 m/s² (97 % of the clip), kT +15 % 2.18 m/s² (rails by construction), mass +20 % 0.94 m/s². The horizontal cable pull (4.69 m/s² at 45°) is also 9–13 % under-delivered; the drones drift inward, rods steepen, and the load hangs low with every drone at its planned height. The honest correction for a kT error is a scale on thrust: §11.1(e), locked out as "kT adaptation by another name". The card must say it compensates kT in all but name, and Wesley should rule.
**4. §10 is misread.** Its mechanism, "a differential integral bias is a differential cable tension, i.e. a moment on the payload", applies to vertical integrals too; hover never charged the integrators (R0077, 75 s, 6.7°), the circle sweep did. A hover-only matrix on an even ring is the one layout where per-drone and common-mode integrators coincide by symmetry, so it cannot show the failure. Missing arms: a circle plus a 30 s tail, and 4/12/8 with uneven shares (statically indeterminate with four drones: vertical integrals can trade load through rigid rods until one rails). 3/12/9 and 4/12/8 are where per-drone action could actually help (levelling uneven shares), and the card does not test that.
**5. The freeze never fires in sim.** `payload_rest_z` is −0.1 on every sim launch, so "payload resting" is never true: the integrator runs from the TAKEOFF edge, on the stands through the throttle soft-start, through the lift and the LAND. The "railed" check must cover the whole run and report I at LAND.
**6. On the rig: frozen when needed, wound up when the freeze misses.** At 13:13 the load never left the floor: the integrator is frozen at 0 for the failure that motivates the card. With the payload origin 4.3 cm off, a resting load can read 7 mm under the 0.10 threshold; then I winds to +1.5 against a grounded load and breakaway releases ~2.7 N of extra lift as a step onto an 8.4 N load.
**7. Weld, detach, LAND.** Order matters against the resting branch that zeroes `ref_cable` (`controller_mpc.py:990`). A detached drone's need drops from ~14.5 to ~10 m/s², so an I sized for the kT error is ~30 % too large. Under the network the MPC tracker would carry a second vertical integrator in series with `diss_ki_load`. The SIL attach smoke is on the §6 negatives list (the bench does not discriminate attach), so "not worse than off" there is uninformative.
**8. Bands anchored to a number the integrator may not control.** If the SIL node-N drone height is itself ~3 cm high (open problem 9), a correctly working integrator leaves the correct case at 0.629 and the falsifier fires on the planner. Split the claim: (a) `|z_refN − z| → 0` per drone; (b) load height vs the planner's zN, with 0.60 beside it. Tilt bands as on − off, not an absolute 3°.
**9. Not runnable as written.** No `z_ki` value; R0363–R0366 cited but not registered; "the seven OCP launches" includes the real launches, so the rig path changes.
**10. Cheaper, in order.** (a) Log-only: add `z_refN − z` and `z_ref_static − z` per drone to the tracker log and re-fly the long hover, kT +10 % and mass +20 % arms (free); this also closes the reviewer's untested "trackers supply the force through tracking error" hypothesis. (b) Port the tested common-mode trim into the tracker (`payload_ref.z − payload_z`, same I in every drone). (c) If per-drone action specifically: integrate against the static target geometry, gate on airborne and no vertical ramp, fix the sign, add a 4/12/8 hover arm and a circle-plus-tail arm.

Verdict: not ready. Sign, moving setpoint, a freeze that never fires in sim, a matrix that cannot fire the §10 falsifier, and a kT +15 % arm that rails by arithmetic.

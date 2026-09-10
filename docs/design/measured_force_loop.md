# Measured-force throttle (INDI-lite) and the settle dwell — branch `measured-force-velocity-loop`

Survey and rationale: `docs/experimentation/control_methods_survey.md` (R1, R4). Run
evidence: `learning.txt` 2026-09-10 blocks for this branch.

## 1. The law (`controller_quad_load/velocity_loop.py`, `indi_gain > 0`)

The classic loop turns the required specific thrust `a_sp` (planner feed-forward plus
velocity feedback) into `throttle = |a_sp| / kT`. Every error in kT, load mass or the
cable model is therefore a steady position error on an integrator-free loop.

With the IMU (body specific force `f_imu` = thrust + cable, no gravity) and the reference's
own cable model `a_cable` (the network builds `a_ff = g + a_des − a_cable`):

```
b        = R(q) ẑ                         body thrust axis, world
xdd_des  = a_sp − g ẑ + a_cable            the acceleration the loop is asking for
xdd_meas = R(q) f_imu − g ẑ               the acceleration the drone is producing
T_next   = LPF(kT·u_applied) + clip((xdd_des − LPF(xdd_meas))·b, ±indi_a_max)
throttle = u_nom + indi_gain·(T_next/kT − u_nom)
```

`u_applied` and `xdd_meas` go through the same first-order filter (`indi_tau`, 0.05 s):
the increment compares like with like, which is the whole INDI synchronisation story.
At steady state the increment is zero only when the measured acceleration equals the
commanded one, whatever kT or the cable model said. The attitude path is unchanged; only
the thrust magnitude is incremental. `indi_gain = 0` reproduces the classic loop bit for
bit (test).

Parameters (tracker, velocity modes only): `vel_indi_gain` 0/1, `vel_indi_tau` 0.05,
`vel_indi_a_max` 3.0. Launch args on `three_attach_launch` / `dissipative_only_launch`.

## 2. What it does and does not fix

| Failure (survey §1) | Mechanism | Effect of R1 |
|---|---|---|
| F1 wrong kT / mass belief | tracker equilibrium offset | removed at the drone (SIL R0238→R0239: drone z error after the weld 0.21 m → 0.004 m with kT told 19 % low) |
| F2 weld transient | delivered vs commanded newcomer share | partially: the newcomer's thrust tracks its command; the share it is handed is still the network's choice |
| trim-off capsize (SIL R0236/R0237) | ring cables slack under a full-share central lifter | **not fixed**: drones sit on their references and the load still goes over — an allocation problem (survey R2), masked in production by the z-trim |
| OCP phase (before the handover) | MPC tracker, no INDI | untouched; the derived sim kT covers it |

## 3. Settle dwell (`attach_traj_hold_mode: settle`, dissipative node)

`tools/hybrid_dwell.py` treats the weld as a hybrid jump and fits the tilt decay after it
on the archived runs: jump 6–14°, λ 0.07–0.16 /s, minimum dwell to a 3° excess 17–33 s,
against the 10 s hold in use (ρ = 0.21–0.48 of the excess left when the circle resumed).
In `settle` mode the post-weld hold ends when the load tilt is under `hold_resume_tilt_deg`
(12) and quiet (< `hold_resume_rate_dps` 5 °/s) for `hold_settle_s` (1.5), capped at
`hold_max_s` (30). The hold becomes a measured dwell; the approach hold (until the weld)
is unchanged. Detach is *not* a transient (R0231: a new equilibrium at the 3/12/9 hang),
so no dwell argument applies there.

## 4. Validation ladder and status

1. Unit: wrong-gain / wrong-cable hover plant, byte-identity at gain 0, bounded increment — pass.
2. SIL trim-off central weld (R0236 vs R0237): tracking fixed, capsize not — see §2.
3. SIL wrong kT (28 vs 34.6), trim off (R0238 vs R0239): drone z error 0.21 → 0.004 m post-weld; the trim-off capsize follows in both.
4. SIL wrong kT, trim ON (R0240 off vs R0241 on): drone z error +0.16 m → −0.016 m within
   2.5 s of the weld, load 0.83 → 0.62 m in the first 4 s; both capsize at 16–20 s on a
   ~1.5 Hz load-tilt oscillation that grows identically with and without R1 — excited by
   the 0.6 m handover descent (the OCP phase parked the load at 1.19 m on the wrong kT)
   and living in the network's measured-attitude attach points, not in the trackers.
   R1 is exact at the drone; the next limit is the network (survey R3).
   Throttle chatter in R0241 was the increment running at 2× its design gain (secant vs
   tangent of the quadratic thrust curve): `vel_indi_slope_ratio` = 2.0 in the sim launches.
5. Gazebo mis-seed +25 % mass (R0242, slope ratio still 1): the weld that was fatal in
   R0217/R0218 becomes the cleanest reconfiguration on record (tilt 27 → 15 → 0–5° by
   +6 s, load 0.60–0.65 m); the newcomer then flipped at +12 s after the increment cut
   its throttle to the 0.05 clip on a pushing rod → `vel_indi_thr_min` 0.2.
6. Gazebo settle dwell (R0243): hold ended at 13.6 s with the tilt at 1.0°; resume-window
   tilt 1.2° vs 2.5° timed; circle unchanged. INDI demo (R0244, ratio 2, floor 0.2): weld
   transient 33 → 3° in one second (classic: peak 47°, 12 s), then a ~1 Hz tilt
   oscillation grows after the circle resumes and the newcomer flips at +18 s.

**What the oscillation is (SIL rod tensions, R0240/R0241/R0237):** slack. With INDI every
drone is a stiff position servo; the load then hangs from four fixed points on rods whose
lengths the network only approximates, the inconsistency is resolved by rods going slack
(min tension 0 N in every window while the tilt grows), and a load on intermittently
slack rods is a pendulum. The compliant MPC used to sag into the load and keep the rods
taut by accident. The network's references are yaw-only (open-loop in attitude), so this
is not an attitude feedback loop. Next step: put the compliance back at the cable — a
per-node admittance on measured tension (survey R2), so a slackening rod pulls its
drone's reference in along the cable until it is taut.

Falsifiers: ringing at the filter bandwidth (actuator sync wrong, lower `indi_tau` or the
gain); a mis-seed that still aborts with the drones on their references (allocation, not
tracking — R2 next).

## 5. Status at the end of 2026-09-10

| Test | Classic loop | INDI gain 1 | INDI gain 0.5 |
|---|---|---|---|
| SIL drone z error after the weld, kT told 19 % low | +0.16–0.21 m | 0.004 m | — |
| Gazebo mis-seed +25 % mass: weld | fatal in 1–2 s (R0217/R0218) | clean, tilt ≤ 12° after +4 s (R0242/R0245); abort ~20 s after the resume | — |
| Gazebo demo: weld peak / time to < 10° | 47° / 12 s (R0234) | 36° / 1 s (R0244/R0246); abort ~20 s after the resume | 38° / ~8 s (R0247) |
| Gazebo demo: four-drone circle tilt | 3.7° mean, 9.4° max | — | 8.5° mean, 16° max |
| Settle dwell (R0243) | hold 10 s timed | hold ended at 13.6 s with tilt 1.0° | — |

The remaining failure is slack (SIL rod tensions), i.e. allocation and cable compliance,
not tracking: the next step is a per-node tension admittance (survey R2) so the network
keeps every rod taut while the trackers stay exact.

## 6. Tension admittance — tried and removed (2026-09-10)

A per-node reference-length correction driven by the tracker's IMU-residual tension
estimate (SIL R0249, Gazebo R0250/R0251; learning.txt R2 entries). Delayed the slack in
SIL, made Gazebo worse: the estimate is biased by the secant thrust model and rotor
vibration, and an admittance integrates the bias. Removed from the code; the prerequisite
is a real tension signal.

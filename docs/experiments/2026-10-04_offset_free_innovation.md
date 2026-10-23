# offset_free_innovation (4 Oct 2026, overnight): offset-free MPC from the planner's own prediction error

Wesley, 3 Oct (overnight loop): remove the ring's height and xy errors from internal and external
model errors with a control-law change backed by literature and experiment.

## 1. Question

Does an integrating load-force disturbance, estimated from the planner's one-step prediction
error, remove the ring's steady error from both internal model errors (twin +5 cm, SIL +6.5 cm)
and external ones (push, wind), where option A (a force balance) sees only the external ones?

## 2. Hypothesis

**History (critic 1).** This is the third attack on the same error.
- 23 Sep, `2026-09-23_effective_load_force`: a Kalman load force driven by the node-0 planned
  tensions. R0347/R0348 settled 2.0 cm low (0.580/0.579, f_d +0.32 N) and drifted; card deleted.
- 24 Sep, `2026-09-24_planner_offset`: the same diagnosis.
  - R0473: the plan asks zN 0.60 every cycle with the ring at 0.663.
  - R0475: FF 0.9 gives -10.2 cm.
  - R0477: the offset is additive.
  - ZBias (z_ki on the height reference) closed it: R0488, 0.600 in 6.2 s at z_ki 0.4.
- 4 Oct (this card): `tools/hold_replay.py R0868` re-confirms on the twin. The planner, closed on its
  own model, settles at 1.0000 m from the logged hold state (ring 1.045 m). SIL R0861 (matched
  plant) still holds +6.5 cm.

**What it is (critic 3).** Node 0 is re-pinned, so v_pred(t_k) = v_meas(t_{k-1}) + dv_plan.
The observer therefore sums to

    d_hat = (m / tau) [ (v_now - v_start) - sum dv_plan ]

At rest that is the integral of the plan's own node-0 acceleration, which is the plan's feedback
on the miss. So it is **integral action through the planner's model and feedforward**, not a new
kind of loop.

The differences from ZBias:
- It acts as a force in the OCP and the reference cone, in x, y and z, rather than as a height
  reference bias.
- It reads zero while the plan's predicted motion happens (a moving reference that is tracked).
- Offset-free MPC theory (Muske & Badgwell 2002; Pannocchia & Rawlings 2003; Maeder, Borrelli &
  Morari 2009) is cited for the steady state only. The theorems assume the observer sees the
  applied input, and node-0 tension never is.

Expected speed: tau_eff ~ 2.7 s at tau 2.0 (critic's estimate), about z_ki 0.4's.

**Why it should converge where 23 Sep did not.** The 23 Sep filter took the plan's tension sum as
the input. This takes the plan's whole predicted load velocity: tensions, directions and rates.
SIL R0869 (below) settled with the innovation at zero, d_hat_z steady at +0.59 N for 20 s, no
drift, -0.8 cm.

The critic predicted d_hat_z 0.3-0.8 N from R0475 (falsified outside that range or railed): +0.59 N.

**The -0.8 cm residual.** For a small miss the plan's first move is in the cable rates, which are
free at node 0, not in the ring velocity. A velocity-only innovation reads zero there.

**Where it should differ from z_ki 0.4 (the discriminating arms, critic 6).**

| arm | z_ki | observer |
|---|---|---|
| xy errors (push, wind, map offset) | height only, cannot act | acts |
| SIL circle sag (R0484 0.508 at target 0.60; R0481 z_ki inert) | inert | should act if the sag is unexecuted plan |
| kT +25 % fault | rails, WARN (R0480) | must also rail or warn, not hide it |
| other sign, FF 0.9 (R0475 -10.2 cm; R0486 z_ki 0.2 0.600) | closes it | must close it |

## 3. The one variable

Planner knob `offset_free`: off | shadow (log only) | on. Default off, passed only when set.
`of_tau` is the observer time constant, default 2.0 s.

**Gate (critic 4):**
- As ZBias: lift complete, |vz| < 0.05 m/s, height miss < z_i_gate, fresh pose, rods taut.
- Plus: the full pull fed forward (ff 1: pretension ease and B2 cap released), not descending or
  landing, and the previous tick solved and published.
- The first tick after a freeze is not integrated.
- Held, not reset, across a resize.
- Frozen in the network phase (the OCP tick does not run).

**Refused:**
- `on` with z_ki > 0.
- `on` with kt_trim on (launch).
- `on` with option A applied and of_tau < 6 x dist_tau, i.e. at least 3 x slower than A's two
  stages (critic 5).

Bound: 3 N per axis, railed flag. Logs `of_est.csv`.

## 4. Baseline (one per arm, critic 7)

| arm | config | baseline (method off, z_ki 0) |
|---|---|---|
| SIL long hover | `configs/sil/carry_hover_n3_long` | R0473 0.663 (target 0.600) |
| SIL FF 0.9 | `carry_hover_n3_long_ff090` | R0475 0.561 |
| SIL kT +25 % | `carry_hover_n3_long_kt125*` | R0480 (z_ki 0.2) 0.455, railed + WARN |
| SIL circle | `carry_circle_n3_long` | R0484 0.508 |
| SIL two drones | `carry_hover_n2_long` | fly once |
| twin hold | `b2_cap075_fix_n3` (no rod model) | R0841 +4.8 cm |
| 1 N push | `b2_cap075_fix_n3_push` | R0856 18.0 cm |
| wind circle | `b4_wind_circle_n3_shadow` | R0864 17.9 cm; A R0867 3.9 cm |
| undisturbed orbit | `b2_cap075_fix_n3_orbit` | R0832 2.8 cm xy lap, +4.8 cm z |
| one drone's map offset | `b2_cap075_fix_n3_map` | R0870 z -3.0 cm, xy 3.8 cm, tilt 6.7 deg |

Comparators on every Gazebo arm: z_ki 0.4, option A, and kt_trim on for the hold and map arms
(online thrust adaptation).

## 5. Pass / fail numbers

| arm | supports | falsifies |
|---|---|---|
| SIL hover | within 1 cm; settle into 1 cm no slower than R0488 (6.2 s) + 50 % | > 2 cm, drift, or p-p > 2 cm |
| SIL FF 0.9 | within 1 cm | > 2 cm |
| SIL kT +25 % | rails or warns (the fault is not hidden) | the ring lifts to target with no warning |
| twin hold | within 1 cm, tilt within 0.3 deg of R0841 | > 2.5 cm or p-p > 2 cm |
| 1 N push | <= 3 cm in the last 5 s of the push; overshoot after release reported | > 6 cm |
| wind circle | within 1 cm of R0867 (A), alone | > 1 cm worse than R0867 |
| orbit | xy lap mean <= R0832 + 0.5 cm, z within 1 cm | xy > R0832 + 1 cm |
| map offset | z and xy within 1 cm; tilt reported (outside n_d = 3) | z or xy worse than R0870 |
| any | 0 solve failures, no abort, landed | an abort |

**Against z_ki 0.4:**
- It beats z_ki on a discriminating arm if its error there is at least 2 cm lower.
- It is no worse on the hold (settle time and p-p).

## 6. Repeats

Exploratory: one run per arm (SIL deterministic, Gazebo laptop). The claim later needs 2 per arm in
Gazebo and the reviewer.

## 7. Cost

- SIL: about 5 runs.
- Gazebo: 5 observer arms, 5 z_ki, 4 A, 2 kt_trim, 2 observer + A.
- Unlimited tonight (Wesley).

## 8. New code?

Yes:
- `mpc_planner/offset_free.py`: pure, 5 tests.
- Planner wiring: gate, refusals, sum with A, log, hold on resize.
- The launch refusal with kt_trim.
- `tools/hold_replay.py`, `tools/method_metrics.py`.
- Configs.

**Rules:**
- Sim only (Wesley's overnight instruction). Rig use needs his decision against decisions.md:72 (the
  rig integral is a safety net) and :93 (observer only, no new integral). On the question list.
- If it beats z_ki 0.4 on no discriminating arm, delete the code. The overnight rule keeps losing
  methods behind their off knob until Wesley decides in the morning.

## Critic

**Critic, 4 Oct 2026 (first card): NOT READY.** Rewrite the card: it repeats the 09-23 observer as
integral action, and none of its arms can tell it from z_ki 0.4.
1. Cite 09-23/09-24 and say why this converges where R0347/R0348 did not; fly the long-hover
   analogue first.
2. Offline first: re-solve logged ticks on R0473, R0861, R0841, R0868, R0832 and R0864 (the
   innovation's sign, its rate, whether the orbit charges it). Predict SIL d_hat_z ~ +0.5 N
   (R0475); falsify if outside 0.3-0.8 N or it rails.
3. Call it integral action through the feedforward. State tau_eff and the loop gain against z_ki 0.4.
4. Gate as ZBias (miss < z_i_gate, |vz| < 0.05). Skip the first tick after any freeze. Hold, don't
   reset, across a resize. Refuse in network/dissipative mode, with kt_trim on and with z_ki > 0.
5. With A on, the observer at least 3x slower than A; report the push overshoot.
6. Arms that tell it from z_ki 0.4, scored on settle time and overshoot: the SIL circle (R0481/R0484),
   the other sign (ff090, R0475/R0486), a 5 cm z reference step, kT +25 % fault masking (R0480),
   n=4 on the even 0/3/6/9 ring, and n=2. The observer alone must be within 1 cm of R0867 on the wind
   circle.
7. One baseline per arm; pin the twin's rod model (R0841 or R0868).
8. Map-offset arm: report ring tilt.
9. Wesley's line in decisions.md: sim only. Delete if it beats z_ki 0.4 on no discriminating arm.

**Addressed.**
- 1: section 2 and R0869.
- 2: partly. R0869 lands at +0.59 N, inside the prediction. The per-run re-solve is replaced by
  `shadow`-free live logs (`of_est.csv` on every arm).
- 3, 4, 5, 7, 8, 9: sections 2-5 and 8.
- 6: the SIL arms and the xy arms. The 5 cm z reference step is replaced by the mass step (a -1.96 N
  z WRENCH) if time allows. n=4 even ring and n=2 are in SIL / Gazebo if time allows.
- Network/dissipative: the observer is frozen while the network flies and keeps its value after (a
  question for Wesley if the network hand-back is ever combined with it).

## Result of the one-step version, and the window (4 Oct, night)

**One-step results:**
- SIL: R0872 -0.7, R0873 (FF 0.9) -0.9, R0878 (n=2) -2.9, R0876 (circle) +4.3, R0875 (kT +25 %)
  -12.5 cm with no warning.
- Gazebo hold (R0880): +7.8 cm (without: +4.8), d_z -0.19 N.

In every case the innovation had converged to zero. The critic's point holds: the one-step
innovation is the plan's node-0 move, and for a miss the plan starts its move in the cable states
(free at node 0), so the ring velocity at node 1 does not change and the fixed point floats.

**Window version (`of_window` W, here 0.5 s).** The innovation is the load position the horizon
solved W s earlier predicted for now, against the measured position, less the estimate change since
that solve:

    r = 2 m dp / T^2 - (d_hat - d_hat_then)

Converged, every horizon predicts no load motion over W. Off the reference the plan always moves the
ring within 0.5 s (R0868 replay: -1.4 cm by node 5), so a converged loop sits on the reference.

It is the same integral action through the model, and the first test passes (6 tests). Queued:
SIL hover, n=2, kT +25 %, FF 0.9, circle, then the Gazebo hold.

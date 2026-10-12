# offset_free_ekf (4 Oct 2026): canonical offset-free NMPC for the ring

## 1. Question

Wesley, 4 Oct: "When I told my supervisor about the integral fix, he said it was a bit of a band aid
fix ... I want the thing I put in my thesis to be more defencable and canonical."

Can the load planner remove the ring's steady errors with the textbook offset-free MPC structure? That
is: a disturbance in its own prediction model, estimated together with the full state by an observer,
and node 0 started from that estimate. Can it do this where z_ki (an integral on the height reference,
outside the model) does it today?

## 2. Hypothesis

**The method (nonlinear offset-free MPC).** Sources: Muske & Badgwell 2002; Pannocchia & Rawlings
2003; Maeder, Borrelli & Morari 2009; Morari & Maeder 2012 (the nonlinear form); Rawlings, Mayne &
Diehl 2017 section 1.5. Check exact references before citing.

- **Augmented model.** The planner's own model x+ = F(x, u, d) (the OCP's RK4 step at dt 0.1 s), with
  a constant force on the ring d (3, N): d+ = d + w_d.
  - d enters the ring's v_dot exactly as option A's d_L (already a `p_geom` parameter).
  - The plant is everything below the planner (trackers + drones + rods). Its input is the planner's
    own command u0 (gamma_i, lambda_i over the first interval), which is known exactly. So the
    estimator sees the applied input (the 4 Oct critic's objection to the innovation observer).
- **Estimator.** An EKF on x_a = [x; d] (13 + 14n + 3 states; 58 at n = 3) at the planner tick.
  - It propagates with the u0 that was sent and corrects with y.
  - y: ring position, velocity, attitude, rate (mocap); each rod's direction s_i (pivot to attach);
    each rod's rate r_i (from the drone and ring velocities, the formula unit-tested in
    test_pin_cable_rates).
  - Not measured, estimated through the model: rd, rdd, tension t, td, d.
  - Jacobians from CasADi. The quaternion is renormalised after each update.
  - The only tuning: Q_d (the disturbance bandwidth), against mocap noise R.
- **Node 0 = the estimate**, for every state. Today the planner pins only the ring and the rod
  directions and re-optimises the rates and tensions every tick, so node 0 is not a state estimate.
  The 23 Sep ratchet came from pinning a stale resampled tension; here the tension is estimated.
  - This needs a second compiled OCP (all states in idxbx_0, its own code directory, as tonight's
    `_pr` variant).
- **Target calculation.** The reference cone (ReferenceBuilder g_eff) already makes the stage
  references the steady state consistent with d: hover exactly, trajectories through the flatness
  references.

**The guarantee (Morari & Maeder 2012).** If the closed loop settles with the estimator converged,
the controlled outputs track with zero offset, whatever the error's cause. The conditions:
- the controlled outputs are the ring position (3), with n_d = 3;
- [x; d] is detectable from y.

The ring attitude is not a controlled output, so its tilt is not guaranteed (the map arms).

**Why it should work where tonight's observer did not.** The innovation observer had no state
estimate. Node 0 was partly re-optimised every tick, so the plan's first move lived in free states
the observer never saw. It settled off target: one step R0880 +7.8 cm; 1 s window R0906 +1.6 cm in
Gazebo, -0.01 cm in SIL (R0893).

With the full estimate, every planned move is checked against measurements of the states it moves.
The fixed point is the theorem's, not ours.

**What it is.** Integral action, inside the model: the observer integrates the unexplained force and
the planner plans with it. That is the canonical answer to the supervisor's "band aid" (an integral on
the reference, outside the model). z_ki stays as the rig's safety net, off in these runs.

**Expected.** Steady errors as A + z_ki tonight, from one estimator that also covers internal error:

| arm | expected |
|---|---|
| hold | +0.1 |
| push | <= 1 cm |
| wind | ~4 cm |
| map z / xy | <= 1 cm |

Release overshoot no worse than A + z_ki's +14.8 cm.

## 3. The one variable

Planner knob `offset_free: ekf` (a new mode of the existing knob; default off; passed only when set),
with `of_qd` (disturbance process noise, sets the estimator's force bandwidth). z_ki 0 and `dist_est`
off in every arm.

Configs: `*_ofekf.yaml`, one per arm, generated from tonight's arm configs, one variable each:
- SIL: `carry_hover_n3_long`, `carry_hover_n2_long`, `carry_hover_n3_long_ff090`,
  `carry_hover_n3_long_kt125` (z_ki 0), `carry_circle_n3_long`.
- Gazebo: `b2_cap075_fix_n3` (hold), `_push`, `_map`, `_mapneg`, `_orbit`,
  `b4_wind_circle_n3_shadow`, `b2_cap075_fix` + push world (four drones, `w4_even_push`).

## 4. Baseline

Tonight's table (`docs/archive/overnight_2026-10-04.md`), three-drone rig twin, Gazebo, cm:

| arm | none | z_ki 0.4 | A + z_ki 0.4 (reviewer PARTLY) | window observer |
|---|---|---|---|---|
| hold z | +4.8 (R0841) | +0.1 (R0871) | +0.1 (R0940, R0953) | +1.6 (R0906) |
| push xy (steady / release overshoot / tilt peak) | 18.0 (R0856) / 0 / 3.2 deg | 17.9 (R0908) | 0.4 (R0928, R0942) / +14.8 / 7.1 deg | 2.6 (R0897, R0944) / 12 |
| wind lap xy | 17.9 (R0864) | 18.0 (R0911) | 3.9 / 4.0 (R0929, R0947) | 7.8 (R0898, R0945) |
| orbit xy / z | 2.8 / +4.8 (R0832) | 2.8 / +1.1 (R0910) | 2.6 / +0.9 (R0941) | 2.6 / +2.5 (R0902) |
| map+ z / xy / tilt | -3.0 / 3.8 / 6.7 (R0870) | 0.0 / 3.8 / 6.7 (R0909) | 0.0 / 1.3 / 9.0 (R0930, R0955) | +1.0 / 0.2 / 8.0 (R0903) |
| map- z / xy / tilt | +12.8 / 3.5 / 6.0 (R0919) | 0.0 / 3.5 / 6.0 (R0921) | not flown | +2.4 / 0.1 / 7.5 (R0907) |
| four-drone push | 15.0 (R0949) | | 0.1 (R0950) | |

SIL z error, matched plant:

| arm | none | z_ki 0.4 | window observer |
|---|---|---|---|
| long hover | +6.3 (R0473) | 0.0 in 6.2 s (R0488) | -0.01 (R0893) |
| n=2 | +4.2 (R0879) | | -0.5 (R0894) |
| FF 0.9 | -4.0 (R0874) | | -0.6 (R0885) |
| circle | +5.6 (R0877) | | |
| kT +25 % | | rails + warns (R0480, z_ki 0.2) | |

## 5. Pass / fail numbers

| arm | supports | falsifies |
|---|---|---|
| SIL hover | within 0.5 cm; into 1 cm within 10 s of the gate | > 2 cm, drift, p-p > 1 cm, or EKF divergence (normalised innovation squared above its 99 % bound for > 2 s) |
| SIL n=2, FF 0.9 | within 1 cm | > 2 cm |
| SIL circle | lap z within 2 cm (from +5.6) | no better than +4 cm |
| SIL kT +25 % | \|d\| at its bound with a warning (the fault is visible) | the ring at target with no warning |
| Gazebo hold | within 1 cm, p-p <= 2 cm | > 2 cm |
| push | steady <= 1 cm; release overshoot <= 14.8 cm and tilt peak <= 7.1 deg (no worse than A + z_ki) | steady > 3 cm |
| wind | lap mean <= 5 cm | > 8 cm |
| orbit | xy <= R0832 + 0.5 cm; z within 1.5 cm | xy > R0832 + 1 cm |
| map+/map- | z and xy within 1 cm; tilt reported (not a controlled output) | z or xy > 2 cm |
| four-drone push | steady <= 1 cm | > 3 cm |
| any | 0 solve failures, no abort including LAND, landed | an abort, or the estimate railing in an undisturbed arm |

**Against the comparators.** The method only earns the thesis claim if it matches A + z_ki 0.4 on
hold, push and wind within 1 cm, from one estimator with z_ki off.

## 6. Repeats

Exploratory first, one run per arm (SIL is deterministic: one run). The thesis claim needs 2 Gazebo
runs per arm and the reviewer, on the final table.

## 7. Cost

- SIL: 5 runs.
- Gazebo: 7 arms x 2 = 14, plus laptop voids (tonight about 1 in 4).
- Laptop cap 10 per session: today (Sunday) the exploratory arms (7 runs) on the laptop; the repeats
  on the lab PC from Monday (no cap).

## 8. New code?

Yes:
- `mpc_planner/offset_free_ekf.py`, pure and unit-tested:
  - converges on the planner's own model under an unknown constant ring force;
  - reads zero with none;
  - quaternion handling;
  - the bound and its warning.
- A full-node-0 variant of the OCP (`observed_state_indices` with all states; its own code directory
  and cache signature; prebuilt).
- Planner wiring: EKF update before the solve, x_hat as x_init, d_hat to `set_disturbance` and the
  reference cone, logs `of_ekf.csv` (estimate, innovation, NIS).
- Launch plumbing; configs.

If it fails the hold or the push steady bar, the knob stays off and the code is deleted after
Wesley's decision.

## Critic

**Critic, 4 Oct 2026: NOT READY.** Rewrite the card. The augmented system is not detectable in the
+5 cm channel (common-mode tension looks the same as d_z), so the guarantee the card rests on does not
hold as designed.

- **Theorem.** It needs innovation zero (n_d = n_y and [x; d] detectable), not n_d = number of
  controlled outputs. Here n_d = 3 against n_y = 30 at n = 3, so a permanent rod-channel innovation
  is allowed.
- **Detectability fails.** Tension enters only the ring's v_dot and w_dot. On an even ring a
  common-mode tension change gives no torque and the same vertical force as d_z. The Q_t:Q_d ratio,
  not the theorem, decides where the correction goes (the 23 Sep ratchet with a random walk).
- **u0 is not the plant input.** The plant receives the 21-node horizon through the tracker MPCs
  (gate x ff, fallback shifts). F(x_hat, u0) is the plan's node 1, so this is still a one-step
  innovation.
- **Higher states.** rd, rdd and td amount to mocap differentiated twice. Pinned against the hard
  bounds they can make node 1 infeasible. The quaternion needs an error-state filter.
- **The arms.** The best case is a tie with A + z_ki, which is only a thesis result if the guarantee
  holds.
- **Literature.** Add Pannocchia & Bemporad 2007 (n_d < n_y) and Pannocchia, Gabiccini & Artoni
  2015. Examiners ask the Hautus question first. The cheapest defence may be a derivation: z_ki as an
  integrating output disturbance on height (the DMC / Muske-Badgwell form), A as an input disturbance.

Must-fix:
1. The Hautus rank at hold states for n = 2, 3, 4.
2. State n_y, n_d and the innovation-zero condition; log the innovation per channel.
3. An offline closed loop with a mismatch that is not a ring force.
4. A gate; refused in attach, dissipative, network and real modes.
5. Feasibility.
6. Re-prebuild after editing the sources.
7. Close the 1-2 cm bar gap.
8. Wesley's word on "replaces z_ki".

**Must-fix 1 done (registry M-hautus).** Linearised planner model at the hover equilibrium (1.0 m,
45 deg), d = constant ring force in v_dot; rank of [A, B_d; C, 0] against n_x + n_d:

| n | measured: ring pose and twist, rod s and r | + tension t_i (from the drone force balance) |
|---|---|---|
| 2 | 43 of 44 | 44 of 44 |
| 3 | 57 of 58 | 58 of 58 |
| 4 | 70 of 72 | 72 of 72 |

The null direction at n = 3 is exactly d_z against the common-mode tension (t_0, t_1, t_2 at +0.37
each, d_z -0.77). With tension measured the system is detectable. But tension measured through the
drones' force balance is option A's estimate, and on the twin's hold that balance has no physical
residual (M-dist-split-rod +0.02 N). So the estimated force reads about 0 and the +5 cm stays, as
option A shows (R0915 +3.8 cm).

**Verdict: card withdrawn, nothing built.** The internal error is not a physical force on the ring.
An input-disturbance (force) model can only see it through the unmeasured tension, the
undetectable channel. The canonical model for it is an output disturbance on the controlled output,
which is what z_ki implements. See the reframing proposed to Wesley, 4 Oct.

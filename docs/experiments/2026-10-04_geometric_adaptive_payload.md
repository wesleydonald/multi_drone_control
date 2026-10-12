# geometric_adaptive_payload (4 Oct 2026): a payload-level controller with an asymptotic tracking guarantee

## 1. Question

Wesley, 4 Oct (option 4, after the literature search): instead of the planner + tracker cascade,
does a payload-level adaptive controller with a proven tracking guarantee remove the ring's steady
errors (internal offset, push, wind, one drone's map error) on our rig twin? And how does it compare
with A + z_ki 0.4?

## 2. Hypothesis

**Method.** Lee's geometric adaptive controller for a rigid-body payload carried by n quadrotors on
rigid links: [Lee15c] Lee 2015 ("geometric adaptive control for aerial transportation of a rigid
body"), the same structure as [Lee18b] Lee 2018 (TCST), and the integral and adaptive terms of
Goodarzi & Lee 2016.

Our system matches its model class: a rigid ring, rigid rods with ball ends, n = 3 or 4.

Structure (to be checked against the PDFs when Undermind's limit resets):
1. Payload level: geometric PD on SE(3) errors (position, attitude), plus adaptive estimates of
   unknown constant force and moment terms. This gives the desired resultant force and moment on the
   ring.
2. Allocation: the desired rod forces from the wrench, through the pseudo-inverse of the
   rod-to-wrench map, with an internal-force (null-space) term. These give the desired rod
   directions.
3. Per drone: a geometric controller of its rod direction on S^2, which gives its desired thrust
   vector.
4. Thrust and attitude to the radio: through our tracker in pass-through mode. The velocity loop
   with gains 0 maps a desired acceleration to throttle and attitude, and Betaflight's rate loop
   flies it.

**Guarantee.** [Lee15c] proves asymptotic tracking of the payload position and attitude in the
presence of uncertainties, by Lyapunov analysis on the configuration manifold. It is shown only in
simulation (numerical example).

**Expected.** The steady errors go to zero by construction, including the ring tilt from one drone's
map error, because the attitude has its own adaptive moment term:

| arm | none | expected |
|---|---|---|
| hold | +4.8 | within 1 cm |
| push | 18.0 | within 1 cm |
| wind | 17.9 | <= 4 cm |
| map tilt | 6.7-9.0 deg | < 2 deg |

The costs, against the MPC:
- No constraint handling (thrust and tension limits, throttle cap 0.8).
- No preview on trajectories.
- Transients set by the gains.
- Lee assumes massless links attached at the quadrotor's centre. Ours have 0.075 kg and pivot 4 cm
  below it: model error the adaptive terms must absorb.

**Role in the thesis.** The canonical robust/adaptive alternative to the MPC cascade [Wan24, Vil24,
Kla20], flown on the same arms. Whichever wins, the comparison answers "why MPC" or "why not".

## 3. The one variable

A new controller mode `mode:=geometric` in `sim_control_launch.py`:
- It engages after the lift completes (the existing planner flies the creep, hand-over and lift) and
  hands back for LAND.
- Its gains and adaptation rates are knobs on that mode.

z_ki and `dist_est` do not apply. Everything else (world, masses, map, events) is pinned to each
arm's config, with `*_geo.yaml` per arm:
- SIL `carry_hover_n3_long`;
- Gazebo `b2_cap075_fix_n3` (hold), `_push`, `_map`, `_mapneg`, `_orbit`,
  `b4_wind_circle_n3_shadow`; four drones `w4_even_push`.

## 4. Baseline

Tonight's table, cm:

| arm | none | z_ki 0.4 | A + z_ki 0.4 (reviewer PARTLY) |
|---|---|---|---|
| hold z | +4.8 (R0841) | +0.1 (R0871) | +0.1 (R0940, R0953) |
| push (steady / onset peak / release overshoot / tilt peak) | 18.0 / - / 0 / 3.2 deg (R0856) | 17.9 (R0908) | 0.4 / 14.5 / +14.8 / 7.1 deg (R0928, R0942) |
| wind lap xy | 17.9 (R0864) | 18.0 (R0911) | 3.9 / 4.0 (R0929, R0947) |
| orbit xy / z | 2.8 / +4.8 (R0832) | 2.8 / +1.1 (R0910) | 2.6 / +0.9 (R0941) |
| map+ z / xy / tilt | -3.0 / 3.8 / 6.7 (R0870) | 0.0 / 3.8 / 6.7 (R0909) | 0.0 / 1.3 / 9.0 (R0930, R0955) |
| map- z / xy / tilt | +12.8 / 3.5 / 6.0 (R0919) | 0.0 / 3.5 / 6.0 (R0921) | |
| four-drone push | 15.0 (R0949) | | 0.1 (R0950) |

## 5. Pass / fail numbers

| arm | supports | falsifies |
|---|---|---|
| SIL hover | within 0.5 cm, no oscillation | > 2 cm, or unstable |
| hold | within 1 cm, tilt mean <= R0841's | > 2 cm |
| push | steady <= 1 cm; onset peak and release overshoot <= A + z_ki's | steady > 3 cm |
| wind | lap mean <= 4 cm | > 8 cm |
| orbit | xy <= R0832 + 0.5 cm, z within 1 cm | xy > R0832 + 2 cm |
| map+/map- | z and xy within 1 cm, tilt < 2 deg (the attitude is controlled) | tilt > 4 deg |
| four-drone push | <= 1 cm | > 3 cm |
| any | no throttle saturation over 0.5 s, no slack rod (tension < 0.2 N), no abort including the hand-back for LAND | any of those |

**Against A + z_ki.** It earns the thesis claim if it matches or beats A + z_ki on every arm and fixes
the map tilt. Otherwise it is the comparator that motivates the MPC.

## 6. Repeats

SIL one run; Gazebo exploratory one per arm, then 2 per arm for the claim; reviewer on the final
table.

## 7. Cost

- Implementation 2-3 weeks: controller, allocation, hand-over in and out, unit tests.
- SIL a few runs; Gazebo 7 arms x 2.
- The lab PC for the repeats.

## 8. New code?

Yes:
- A new pure module `payload_geometric.py` (payload wrench law, allocation, rod-direction law,
  adaptation), unit-tested on the reduced model. Tests: the zero-error equilibrium holds; the adaptive
  estimate converges on a constant force and moment; allocation consistency.
- A node, or a mode of the planner node, that engages after the lift.
- The tracker's pass-through use (velocity loop, zero gains).
- Launch mode and configs.

If it fails, the mode stays out of the defaults and the code is deleted after Wesley's decision.

## Critic

**Critic, 4 Oct 2026: NOT READY.** Rewrite the card. Its "zero by construction" is integral action
that no arm separates from the architecture, and its guarantee is cited from an unread paper whose
assumptions our rods, pivot and Betaflight loop violate.

- **Tried before.** No registry row for the method, but each of its mechanisms has been tried:
  - Allocation on the measured ring attitude: the tilt-aware wrench R0212/R0213 gave a 68 deg weld
    transient and aborted 2 of 2.
  - Per-drone integrators: vel_ki 1 (R0077) tilted the ring to 60 deg.
  - Adaptive action on the ring: the L1 card, which "reduces to an integral-like linear controller".
  - docs/experimentation/control_methods_survey.md:135 ruled out full SE(3) Lee control.
- **Simplest explanation.** The +4.8 cm is the cascade's force balance (overnight; offset_free_ekf
  withdrawn). Lee's adaptive terms integrate the payload error, so "zero by construction" is z_ki in
  a new place. No arm separates the architecture from the integrator.
- **Model class.**
  - Lee assumes massless links at the quad's centre of mass, torque-level attitude control,
    continuous time, no saturation and taut links.
  - Ours: rods 0.075 kg each (26 % of the ring), a 4 cm pivot lever (flipped R0548 and T0010),
    Betaflight rate mode behind a P attitude loop plus ELRS, a 50 Hz tracker and a 10 Hz planner.
  - The guarantee therefore degrades to ultimate boundedness at best, with the bound set by the
    gains against the delay.
  - Rods carry compression, so "slack" is not the failure mode.
- **Feasibility.**
  - Pass-through exists (velocity_loop.py:244, gains 0), but it clips at 30 deg tilt, at a_z >= 1 and
    at the model cap of about 0.615.
  - The S^2 law must run at 50 Hz in the tracker, which needs a new topic. Run in the planner, it
    sees 100 ms + about 125 ms of lag.
  - Estimate seeding at hand-over is unspecified, and the +4.8 cm returns in LAND after the
    hand-back.
  - 2-3 weeks is optimistic: expect 4-5 of the 7.5 weeks left.
- **History.** INDI-lite failed as a stiff position servo on approximate rods. Lee servoes rod
  direction, so it may avoid that. Claim it and test it.
- **Comparison gaps.**
  - n=4 only on the push; no figure-8.
  - The gains are knobs, not frozen.
  - The SIL plant is Lee's own model class, so it cannot test the mismatch.
  - n=2 cannot be allocated.
- **Examiner.**
  - Lee TCST 2018 appears to be simulation only, without the adaptive theorem.
  - Wahba & Hoenig (RA-L 2024) flew Lee's controller on hardware with a QP allocation: "why the
    pseudo-inverse?"
  - "Both arms integrate: what does the comparison show?"

Must-fix:
1. Read the PDFs. Cite the theorem, its assumptions and the regressor. Reword the guarantee to UUB.
2. A PD + allocation arm with adaptation off, on hold and map+.
3. Each loop's location and rate. S^2 at 50 Hz in the tracker, or a delay margin. Log the clips.
4. Estimate seeding at hand-over; planner and z_ki state at hand-back. Fly one hand-back run first.
5. Payload-level adaptation only, no per-link integrators (R0077).
6. Falsifiers:
   - error slope over the last 20 s and estimate convergence;
   - wind fails above 4.9 cm;
   - push release judged against R0856 (under 3 cm in 1.3 s);
   - a tension proxy that can be measured.
7. Freeze one gain set on SIL plus one Gazebo arm. Add an n=4 hold and the figure-8.
8. Wesley's word on a payload integral against decisions.md:72/:93 before any code.

**Must-fix 1, from the PDFs (Wesley supplied Lee 2018 TCST, docs/papers, 4 Oct).** Lee 2018 is a
geometric PID controller, not an adaptive one.
- **Model.** n quadrotors on massless rigid links attached at each quadrotor's centre of mass,
  x_i = x_0 + R_0 rho_i - l_i q_i. n >= 3 (rank P >= 6, eq 14).
- **Disturbances.** Fixed unknown Delta_x0 and Delta_R0 on the payload, Delta_xi and Delta_Ri on each
  quadrotor, bounded by a known B_delta (eq 15).
- **Law.**
  - Payload PID: F_d and M_d (eqs 20-21) with integral estimates of the disturbances
    (eqs 30-32: d/dt Delta_bar_x0 = h (e_x_dot + c e_x), and so on).
  - Minimum-norm allocation mu_d = diag(R_0) P^T (P P^T)^-1 [R_0^T F_d; M_d] (eq 23), with no
    internal-force term.
  - Desired link direction q_id = -mu_id/|mu_id|.
  - S^2 link-direction control with its own integral term (eq 27).
  - Quadrotor attitude by finite-time terminal sliding mode (eqs 33-37).
- **Guarantee.** Propositions 1-2: the zero equilibrium is stable in the sense of Lyapunov, the
  tracking errors converge asymptotically to zero, and the estimation errors are uniformly bounded.
  This holds for constant disturbances only.
- **Evidence.** One numerical example (n = 3, 1.5 kg box, figure-8, time-varying disturbances), no
  hardware.

The critic is therefore right: "zero by construction" is the integral terms. What Lee adds over z_ki
is integral action inside a model-based law, with a Lyapunov proof for the coupled system, on the
payload pose and each link. The per-link integral (eq 32) is exactly what R0077 warns about.

Hardware evidence for Lee-type control comes from elsewhere: Wahba & Hoenig RA-L 2024 (QP
allocation), still to be read. Wang et al. 2024 (T-ITS, docs/papers) has hardware, with a point-mass
slung load.

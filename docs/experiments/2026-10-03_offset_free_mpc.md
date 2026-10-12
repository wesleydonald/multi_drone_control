# offset_free_mpc (option A, 3 Oct 2026): disturbance-augmented planner for the payload position

Supersedes the ring-only design of `2026-10-01_load_ext_force.md` (B4; critic NOT READY, its
must-fixes answered in §8). History to read first: `2026-09-23_effective_load_force.md` (an
estimator driven by the plan's own node-0 acceleration converged to `-m a_model` and added a
5-6 cm oscillation).

## 1. Question

Wesley, 3 Oct: "what options do we have to improve the control law for the payload position";
"lets give A a good go". A principled fix rather than an integral term, for xy and height, that
also covers an object thrown into the ring (TU Delft's demo, 1 Oct).

Can the planner hold the ring on its reference under a constant external force, on the ring or on
the drones, by planning with an estimate of that force taken from measured motion?

## 2. Hypothesis

**Model.** Two constant unknown forces enter the planner's model as runtime parameters:
- `d_L` (3): on the ring, `m_L a_L = -sum_i t_i s_i + m_L g + d_L`;
- `d_D` (3): the same force on every drone (air or a common attitude/thrust-axis bias), so each
  drone's required thrust becomes `f_i = m_i (a_i - g) - t_i s_i - d_D`.

**Estimate (measured, not planned).** Each planner tick, from mocap accelerations (a kinematic
filter per body), each drone's delivered thrust (the tracker's `thr_out` through its own affine map
and pack voltage, along its mocap thrust axis), and the measured rod directions `s_i`:
- per drone: `m_i a_i = f_i + m_i g + t_i s_i + d_D` (3 equations each);
- ring: `m_L a_L = -sum_i t_i s_i + m_L g + d_L` (3 equations);
- unknowns: the n tensions, `d_L`, `d_D` (n + 6), against 3n + 3 equations: overdetermined for
  n >= 2. Weighted least squares, then a first-order low-pass (tau about 1 s), time-aligned thrust
  and acceleration. The tensions come out as a by-product (measured rod tensions).

**Use.**
- OCP: `d_L` in the ring dynamics, `d_D` in the thrust expression (thrust bounds and the published
  thrust feedforward).
- References: the effective gravity that tilts the rod cone and scales the nominal tensions gets
  `+ d_L / m_L` (`ReferenceBuilder`, the same path a trajectory acceleration takes), so `w_s` and
  `w_t` stop resisting the correction.
- Trackers: unchanged code. The published external acceleration `c_i` becomes
  `ff * gate * t_i s_i / m_i + d_D / m_i`, so each tracker's model contains the drone-borne force.

**Why offset-free.** With a constant disturbance and a converged estimate, the plan cancels the
estimated force. Any constant model error (thrust-map offset or gain, rod and magnet mass, marker
vs thrust axis) lands in `d_L`/`d_D` and is cancelled with it: the standard offset-free MPC result
(Muske & Badgwell 2002; Pannocchia & Rawlings 2003, to be checked before citing). It does not touch
kT (locked) or the trackers' law.

**Predictions (rig twin, three drones, 1.0 m, z_ki 0):**

| case | estimator off (measured or expected) | estimator on |
|---|---|---|
| 1 N on the ring along -x | 18.5 cm (R0856) | within 3 cm; `d_L,x` within 20 % of -1 N |
| +0.2 kg on the ring (1.96 N down) | about 25 cm low (expected, ~same stiffness as the push or weaker) | within 3 cm of 1.0 within 8 s |
| 0.15 N on each drone along -x (the rig's -0.45 N, drone-borne) | measured first | within 3 cm; `d_D,x` within 20 % of -0.15 N |
| plant thrust offsets +0.01 (a rig-size map error) | measured first | within 2 cm, no oscillation |
| no disturbance | hold as R0841/R0844 | hold within 1 cm of off; \|d\| < 0.3 N per body |

## 3. The one variable

Planner knob `dist_est` (default off, passed only when set, as `land_unwind`). Everything else is
pinned to `b2_cap075_fix_n3_push.yaml` (three-drone rig twin, 1.0 m, B2, z_ki 0, push world).

New code behind it:
- `src/mpc_planner/mpc_planner/disturbance_estimator.py`: pure, unit-tested (force balance, least
  squares, filter, gating, bound, ramp-down);
- `LoadCableDynamics`: parameters `d_L`, `d_D` (6) in the model and thrust expression; one solver
  rebuild per fleet size, cache signature tagged, canonical zeros;
- `planner_node`: subscriptions to each drone's `ELRSCommand` (the tracker's command, pre-mux, which
  carries `thr_out`) and telemetry (voltage), the trackers' map values from the launch; gating
  (airborne, rods taut, not LAND or descent, poses fresher than 0.1 s; frozen otherwise; ramped to 0
  over 1 s at LAND and when the ring is down; reset at the planner phase start); bound per axis
  (ring 3 N, drones 1 N) with a `railed` WARN and the bound held, not dropped; logs `dL*`, `dD*`,
  `t_hat*` columns and a 1 Hz line;
- `ReferenceBuilder`: `d_L` in the effective gravity;
- runner: `WRENCH` already exists (drone links take the scoped name `x3_drone<i>::...`);
- SIL plant: a constant force event on the ring and on the drones, for the cheap first pass.

## 4. Baseline

| arm | baseline (estimator off) |
|---|---|
| ring push 1 N -x | R0856 (18.5 cm, tilt max 3.8 deg) |
| no disturbance | R0841/R0844 (hold 1.047-1.048 m, tilt mean 1.10-1.11 deg) |
| ring mass +0.2 kg, drone push 0.15 N each, map offset +0.01 | flown once with the estimator off in this card |
| orbit (later) | R0832 |

## 5. Pass / fail numbers

Windows: the last 5 s of each disturbance phase (the ring settles within 2 s, R0856); `d` bands on
the change across the step (after minus before), not the absolute value.

| arm | supports (estimator on, both repeats) | falsifies (either repeat) |
|---|---|---|
| nodist | hold within 1 cm of off; \|d_L\|, \|d_D\| < 0.3 N; tilt mean within 0.3 deg of off | \|d\| > 1 N (invents a force), hold moves > 2 cm, or any oscillation > 2 cm p-p |
| ring push | ring xy within 3 cm; d_L,x step within 20 % of -1 N | > 6 cm (not better than a third of off), xy p-p > 3 cm, tilt max > 5 deg |
| ring mass | within 3 cm of 1.0 within 8 s of the step | > 6 cm low after 8 s, z p-p > 3 cm, or railed |
| drone push | ring xy within 3 cm; d_D,x step within 20 % of -0.15 N; d_L,x within 0.15 N of 0 | the ring moves the wrong way (the old critic's -F/k_P case) or > 6 cm |
| map offset | hold within 2 cm of 1.0 | worse than off, or oscillation > 3 cm p-p |
| any | 0 solve failures, no abort, LAND as without the estimator | an abort, a solve failure streak, a LAND disarm above 0.2 m |

## 6. Repeats

SIL: 1 per arm, on and off, first; if SIL falsifies, no Gazebo. Gazebo: one floor-start nodist run
first (CLAUDE.md: the estimator is live from the lift); then 2 per arm on, 1 per arm off where no
baseline exists.

## 7. Cost

SIL about 10 runs. Gazebo: 1 + 5 arms x 2 on + 3 off = 14 runs: two laptop sessions or the lab PC.
Build: estimator, model parameters and wiring, SIL hook, tests: about 1-2 days. After Wednesday.

## 8. New code?

Yes (§3). Critic required: a new estimator in the flight loop.

The 1 Oct critic's must-fixes, answered:
1. Replay done (1 Oct night): the rig shows a world-fixed -0.4 to -0.5 N in every hold; Wesley,
   3 Oct: the mocap frame is not tilted, and a tilt should be adapted to anyway.
2. Rod and magnet masses: the rig drone mass 0.55 includes them; any remaining constant mass error
   is a constant model error, which the estimate absorbs; the bounds are sized above the replay's
   static part (0.45 N) and the twin's (0.05 N).
3. The question now covers ring-borne and drone-borne forces (`d_L`, `d_D`).
4. A drone-borne push arm with its own falsifier (wrong-way motion).
5. The trackers' map in every arm (read from the launch, the same values the trackers fly); the map
   offset arm; time alignment of thrust and acceleration.
6. Bands on the change across the step; predictions from the measured R0856 stiffness; LAND at 55 s
   leaves 15 s per window.
7. Balanced references: `d_L` enters the effective gravity, so the regulariser targets move with it.
8. Ramp to 0 at LAND and with the ring down; rail behaviour stated; the tracker command topic is
   pre-mux.
9. Wesley's word on feeding a delivered-thrust residual to the planner: 3 Oct, "give A a good go".

If it fails: registry rows stay, this card stays, the estimator, the model parameters, the knob
and the SIL hook are deleted the same session; the runner WRENCH tool and push worlds stay.

## Critic

**Critic, 3 Oct 2026: NOT READY.** Rewrite the card: the bands would falsify a working estimator (the
rod-weight split, the +4.8 cm anchor), and no arm tests a per-drone thrust error, the one case this
structure gets wrong.

**Already tried, in parts.** The ring half is the family of the 09-23 load-force KF (R0331-R0362,
deleted); this card avoids its plan-driven fixed point. The drone half (a thrust residual into the
tracker model) is the family of INDI-lite (R0238-R0257, removed), `diss_k_adm` (R0250/R0251, worse)
and `kt_trim` (off on the rig, decisions.md:73). The only data is the 1 Oct replay of the lumped force,
which cannot see the d_L/d_D split.

**Internal errors are invisible.** Every static offset the twin shows today is internal and reads zero
in a force balance: the hold at +4.8 cm with z_ki 0 (R0841/R0844), 26 cm low with the cap held
(R0838), rods 4 deg steeper than planned (R0856/R0860). The 18.5 cm push is the P-only stiffness. So
this is feedforward of external forces, not offset-free: a perfect estimator leaves the hold at ~1.048.

**Identifiability.** The ring block has zero redundancy (3 equations, 3 unknowns d_L), so
d_L = lumped w - n d_D exactly and nothing can flag an error in it. Tensions and d_D come from the
drone block only (redundancy 3 at n=3, 5 at n=4). The vertical axis is the weak one and worsens as
rods steepen (z vs xy information 1.5 at 45 deg, 3.5 at 60 deg). A vertical error eps on ONE drone
is mis-split: d_D,z takes eps/n and a fictitious lateral ring force 0.5-0.67 eps appears, pushing the
ring about 0.6 eps/k sideways: ~0.9 cm for the 0.181/0.187 offset scatter, ~1.8 cm for a 2 % gain
error, ~5 cm for a 1 V voltage error. Uneven 1/3/5/9 rings add similar cross-talk from rod weight.

**Bands never measured for the split.** The twin drone is 0.475 kg + a 0.075 kg rod whose centre of
mass is 0.147 m from the ring end; the planner books all 0.55 kg on the drone. A correctly working
estimator in the twin nodist arm should read d_D,z ~ +0.54 N per drone and d_L,z ~ -1.6 N, which trips
the card's "|d| > 1 N" falsifier. SIL has massless rods and no pivot, so SIL passes and Gazebo
falsifies for an unrelated reason. Rig n=4: the rod split (~-2 N) plus the 0.2 kg demo (-1.96 N)
exceeds the 3 N ring bound. The 20 % d band (0.2 N = 3.7 cm at 5.4 N/m) is inconsistent with the 3 cm
position band.

**Offset-free or not.** Muske-Badgwell / Pannocchia-Rawlings get offset-free tracking from an observer
on the controlled-output prediction error with n_d = n_y and a target calculation. Here n_d = 6,
n_y = 3, and the innovation is a force balance: the citation does not transfer. This is a disturbance
observer used as feedforward, offset-free only if unbiased and the rest of the loop is (it is not:
+4.8 cm). Defensible structure: disturbance observer for external forces + an output-error integrator
(ZBias in z, an xy analogue if needed) for the internal remainder. Consider the momentum-observer form
(mocap velocity rather than differentiated acceleration).

**Estimator in the loop.** With exact physics d does not depend on the motion; the loop closes only
through model error: rod inertia (~7 % lagged acceleration feedback near the 0.23 Hz ring mode, close
to the 0.16 Hz filter corner) and thrust/acceleration misalignment (the d_D -> c_i path has the
kt_trim v2 topology that chattered at 2 Hz with n=2). Same filter on thrust and m a, state the
actuator lag, give a loop-gain number. No n=2 or n=4 neighbour planned.

**d_D in the trackers.** No double count, but `cable_ff_scale` scales d_D in c_i only, and
`_load_grounded()` (tracker_node.py:1330) zeroes c_i (d_D included) the moment the load is down while
the planner's acc blends over LAND_BLEND_S. d_D changes no planned state: apply it in `_publish_refs`
and keep the OCP rebuild to d_L only. kt_trim on (the legacy sim default) would put two estimators on
the same vertical residual: refuse that combination. d_D,z through c_i is an additive thrust-map
correction in the trackers, i.e. thrust adaptation on the rig where kt_trim is off by decision; it is
also the per-drone observer Wesley kept aside.

**Interactions.** ZBias (tau 2.5 s) and the estimate (tau 1 s) both chase a mass step (overshoot near
z_i_gate 0.25). "Airborne" is undefined (the 09-23 gate bug); the rig's z_taut_gate 0.9 never opened
for ZBias, so the estimator could be a silent no-op on the rig. A ramp at LAND start puts the full
transient into the descent and risks the R0852 touchdown disarm: freeze through the descent, zero at
load-down. `DissipativeController` inherits the knob into detach, attach and M2; in network mode it
would build geometry on a measured load quantity, against the open-loop lock. Resize behaviour
unspecified.

**Asked for?** Option A yes (decisions.md:92). Two parts need Wesley's explicit word: the d_D path
into the trackers, and a delivered-thrust residual in the rig's flight loop (1 Oct must-fix 9).

**Must-fix:**
1. Offline replay of the split first (no runs): d_L, d_D, tensions, residuals and condition number on
   R0841, R0844, R0856, R0813 (1/3/5/9) and rig h2/h5/h6/c1. Falsifier: in R0856, the d_L,x step not
   within 15 % of -1 N, or the d_D,x step above 0.1 N. Anchor every band and bound to the replayed
   static split.
2. Re-anchor the position bands to the nodist hold (~1.048), not 1.0; make d and position bands
   consistent (<= 15 % or 4 cm).
3. A per-drone map arm (one drone +0.01): predict ~0.6 eps/k lateral shift, falsify above 2 cm.
4. One variable at a time: shadow mode (logged only), then d_L applied (OCP + g_eff), then d_D to the
   trackers as a separate card with Wesley's explicit word.
5. Drop the offset-free claim and citation; call it disturbance-observer feedforward; keep ZBias as the
   integral term.
6. LAND: freeze through the descent, zero at load-down, acc and c_i consistent at that step; check
   drone heights at LANDED in every arm.
7. Gates: airborne = payload_rest_z + 0.05, a taut gate that opens on the rig, frozen and re-seeded
   across resizes, refused in network mode and with kt_trim on.
8. Neighbours before repeats: n=2, n=4 on the even 0/3/6/9 ring, z_ki 0.4, the orbit (R0832), the
   floor start.
9. Size the bounds from the replay plus the 0.2 kg demo at n=4.
10. Timing: the same filter on thrust and acceleration (or the momentum form), the stated actuator
    lag, a loop-gain number.
11. State SIL's role as a plumbing and stability gate only (massless rods, no pivot).

## Must-fix 1: offline replay of the split (3 Oct evening, no runs)

`tools/dist_replay.py`: per sample, the n drone and ring force balances solved by least squares
for the tensions, d_D and d_L; thrust from each tracker's own map (`thr_out - thr_off`, kT 35.2,
0.55 kg) along its mocap body z; rods from the pivot to the plate (sim: through the logged ring
attitude, `--ring-attitude`; rig: radially, no ring yaw logged). Windows: the 5 s before each push
edge, or 15 s before LAND.

| run | window | d_L (x, y, z) N | d_D (x, y, z) N per drone | cond |
|---|---|---|---|---|
| twin R0841 / R0844, n=3 | hold | +0.10, +0.04, -1.60 | -0.02, -0.01, +0.56 | 5.9 |
| twin R0813, n=4, 1/3/5/9, 0.5 m | hold | +0.13, +0.06, -2.16 | -0.03, -0.01, +0.55 | 8.2 |
| twin R0856, 1 N push -x | step (push minus pre-push) | **-0.73**, 0.00, +0.01 | **-0.09**, 0.00, 0.00 | 5.9 |
| rig h2 (0.8 m) | hold | -0.24, +0.32, -3.03 | -0.07, -0.13, +1.17 | 9.2 |
| rig h5 (1.0 m, rotated 124 deg) | hold | -0.40, -0.36, -4.28 | -0.01, +0.14, +1.27 | 8.4 |
| rig h6 (1.0 m, x +1.1) | hold | -0.40, -0.18, -3.36 | -0.02, +0.04, +1.15 | 9.0 |

Reading:
- **The critic's falsifier is met.** The R0856 step puts -0.73 N on the ring (27 % short of -1 N,
  outside 15 %) and -0.09 N on each drone (under the 0.1 N line). The total, -0.73 - 3 x 0.09 = -1.0 N,
  is right: the force is found, but a quarter of a ring push is booked on the drones. The exact ring
  attitude gives the same split, so it is not the rod approximation; the twin's rods carry mass
  (0.075 kg, centre 0.147 m from the ring end) and pass a non-axial force the massless-rod balance
  cannot represent (inferred). Applied as d_L only, the ring term would cancel about 73 % of a ring
  push (about 5 cm left of 18.5 at 5.4 N/m).
- **The static split is mass booking, as the critic predicted.** Twin: -1.60 N on the ring, +0.56 N
  per drone (predicted -1.6 / +0.54): the rod weight is booked on the drones. Rig: -3.0 to -4.3 N on
  the ring, +1.15 to +1.27 N per drone, about twice the twin's; consistent with the magnets sitting
  at the ring end of each rod (inferred). Constant, so an estimate absorbs it, but it rails the card's
  3 N ring bound before any disturbance.
- **The rig's -x force lands mostly on the ring term** (-0.24 to -0.40 N on the ring, about 0 per
  drone; totals -0.43 to -0.46 N as the 1 Oct lumped replay). Given the twin's 73/27 split of a known
  ring push, this fits a force on the ring, but a drone-push calibration run is needed before reading
  it as ring-borne.
- Conditioning is fine (5.9-9.2); the rig's d_L,z scatters most (sd 0.2-0.7 N).

Next, before any estimator code: put the rod and magnet masses into the force balance (each rod's
weight split between its two ends, with the non-axial end force a massive pinned rod carries) and
re-run this replay; the falsifier stands. Then a drone-push calibration run (WRENCH on every drone
link) to see where a drone-borne force lands.

### Replay with massive rods (`--rod`, same evening)

Each rod's weight handed to its two pinned ends by its centre of mass (twin 0.075 kg, centre 0.147 m
from the ring end, from the world SDF; rig 0.075 kg, nearly all rod with a small magnet at the ring
end per Wesley, centre taken 0.26 m from the ring end); airframe 0.475 kg on the drone.

| run | d_L (x, y, z) N | d_D (x, y, z) N per drone |
|---|---|---|
| twin R0841 hold | +0.11, +0.04, +0.02 | -0.02, -0.01, +0.02 |
| twin R0813 hold (n=4, 1/3/5/9) | +0.13, +0.06, 0.00 | -0.03, -0.01, +0.01 |
| twin R0856 push step | **-0.73**, 0.00, +0.01 | **-0.09**, 0.00, 0.00 |
| rig h2 / h5 / h6 hold | -0.24 / -0.40 / -0.40, +0.32 / -0.36 / -0.18, -1.87 / -3.12 / -2.20 | -0.07 / -0.01 / -0.02, -0.13 / +0.14 / +0.04, +0.79 / +0.88 / +0.76 |

- **The twin's static split was all rod weight:** with the rods modelled it reads within 0.03 N of
  zero on every axis, both terms, three and four drones.
- **The push split is not rod weight:** still 73/27 ring/drones. Unexplained; candidates to test:
  least-squares residuals before and during the push, the ring-end joint type, and a SIL ring push
  (massless rods, known force) once SIL has a force hook.
- **The rig keeps a vertical split** of about +0.8 N per drone and -1.9 to -3.1 N on the ring after
  the rod model (totals within +-0.5 N, as before). Not explained by the rod as described; candidates:
  the rod's mass distribution on the real airframes, per-drone thrust-map scatter, the ring's true
  mass. A weighing of one rod with magnet and the ring would settle the first and last.
- The rig's -x force stays on the ring term (-0.24 to -0.40 N).

## Exploratory Gazebo tests of the live estimator (3 Oct night, Wesley's request; not the card's claim)

Built: `disturbance_estimator.py` (pure, tested), `d_L` as a runtime model parameter (`p_geom` + 3; one
rebuild per n), `d_L` in the reference cone's effective gravity, `d_D` into the published acceleration
and external-force term (`full`), knob `dist_est` off|shadow|ring|full (default off, passed only when
set), gated to an airborne ring in the planner phase, frozen in LAND and zeroed with the ring down, reset
on a resize; logs `dist_est.csv`. Rod model as the replay (twin 0.075 kg, centre 0.147 m from the ring
end). Wind: Gazebo WindEffects, world `three_rigid_ground_rig_wind.sdf` (steady 0.4 m/s along -x on the
ring and the drone bodies); pushes: runner `WRENCH`.

| run | test | ring xy error | estimate vs truth |
|---|---|---|---|
| R0861 (SIL) | no disturbance, `full` | flight = R0796 | gated mean within 0.013 N of 0 |
| R0862 | push wind 0.5 N ring + 0.15 N/drone, `shadow` | 19.6 cm uncorrected | steps -0.50 / -0.15 N vs -0.50 / -0.15 |
| R0864 | circle in 0.4 m/s wind, `shadow` | lap mean 17.9 cm, max 24.1 | ring -0.27, drones -0.18 N (hover -0.28 / -0.18) |
| R0866 | circle in 0.4 m/s wind, `full` | lap mean **3.7 cm**, max 6.5 | as R0864 |
| R0832 | circle, no wind (reference) | lap mean 3.2 cm, max 5.5 | - |

Voids (laptop pose timeouts in the creep, nothing applied yet): R0863, R0865. Graph:
`results/2026-10-03/b4_wind_circle_n3.png`. One run each: exploratory, not the claim; the card's arms,
bands and the critic's remaining must-fixes (per-drone map arm, neighbours, LAND checks) still stand.

Filter (Wesley: smooth the estimate). Offline on the logged raw solution: one first-order stage of
1.0 s left 0.014 N of ring jitter and 2.2 s to 90 % of a step; two stages of 0.6 s give 0.006 N at
2.3 s (two of 0.5 s: 0.007 N at 1.9 s; one of 2.0 s: 0.007 N at 4.6 s). Now the default
(`dist_tau` 0.6 per stage). R0867, the circle with it: lap mean 3.9 cm (R0866 3.7), ring-estimate
jitter 0.0058 N in flight (R0866 0.0154).

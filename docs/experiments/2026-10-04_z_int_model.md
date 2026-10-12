# z_int_model (4 Oct 2026): the ring position integral as a force in the planner's model, x, y and z

## 1. Question

Wesley, 4 Oct: "I would rather it goes into the planner's model", then, after the critic: "Three axes
and run the plan". Can the planner hold the ring on its unchanged reference in x, y and z if the
bounded integral of the ring position error enters its model as a force on the ring, instead of
(z only) being added to the height target? How does the transient compare with z_ki on the target?

## 2. Hypothesis

**What it is (critic 8).** Integral control routed through the model's disturbance input.
- The state is b (m, per axis), integrated from the ring position error to its node-0 reference, with
  ZBias's gain, bound and gates.
- It is applied as a constant force on the ring in the OCP and the reference cone: d = -k * b (N),
  added to `_apply_load_force` (planner_node.py).
- d is the lumped constant disturbance: everything the model gets wrong about the ring's force
  balance, real or not. For a real force (wind, a push, the object) it is that force. For an internal
  error (feedforward, thrust map, rods) it is fictitious: M-hautus shows a vertical ring force and the
  common-mode rod tension look the same from pose.
- It is not an estimator. An offset-free estimator integrates the innovation (the overnight observer,
  whose fixed point floated). This integrates the tracking error.

**Why zero steady error.** Internal model principle (Francis & Wonham 1976): any stable equilibrium
with the gate open and b off its bound has zero position error, whatever the constant cause. This is
the same claim as z_ki's. The two modes differ in where the integral acts, and so in the transient
and in what the plan predicts (critic):
- reference mode plans a climb toward target + b that never comes;
- model mode plans a stationary ring at the target.

In offset-free MPC terms, the choice of disturbance model shapes the transient, not the steady state
(Pannocchia & Rawlings 2003; Pannocchia & Bemporad 2007).

**Gates.**
- z: as ZBias, including the orbit window.
- x and y: ZBias's gate, plus the reference at rest (hover), the ring under 0.05 m/s horizontally and
  the miss under z_i_gate. Through a trajectory, x and y hold the value learned in hover, so a
  world-fixed offset found in the hover (the rig's -x drift on 1 Oct) is carried into the circle.

**Bounds (critic 9).** z_i_max 0.15 m on b per axis, which is k * 0.15 N: from Stage 0 below.
- What a bound can hide: a fault up to that force, silently until the 70 % warning (per axis, once
  per flight).
- Decisions it sits against:
  - 2026-09-30: the integral is a bounded safety net, and the model is judged with it off.
  - 2026-10-03: A, a principled estimate, is preferred over an integral.
  - Wesley's 4 Oct request overrides the placement, not the safety-net role.

**k (critic 1).** The conversion makes model mode's loop gain equal reference mode's
(crossover about z_ki * k / K, where K is the cascade's stiffness to a ring force). So k = K, measured
in Stage 0 from a known real force with nothing on. The old-box estimates (R0475 8.2 N/m z, R0869
8.1 N/m z, R0856 5.4 N/m xy) are not used: the box was probably the soft part.

## 3. The one variable

Planner knob `int_mode`: `reference` (today, default) or `model`, with `int_k_xy` and `int_k_z` (N/m)
set from Stage 0 and not tuned per arm.

Pinned on every arm:
- z_ki 0.4, z_i_max 0.15, z_i_gate 0.25.
- `x0_relax_frac` 0. The claim holds only with the box at zero, and is void if Wesley keeps 0.025
  (critic 10).
- A off, observer off, kt_trim off.

Refused (critic 2, 3; unit-tested in `test_int_mode_model_reaches_the_planner_only_where_the_fleet_cannot_resize`):
- on the rig;
- with kt_trim on;
- with a fleet that can attach or detach: the detach graphs, or the attach graph with reserved_attach > 0
  or enable_approach;
- in the node: with dist_est ring/full, offset_free on, or z_ki 0.

## 4. Stage 0 and baseline

**Stage 0 (critic 1, 5).** Two Gazebo floor-start runs, box at zero, z_ki 0, nothing else:
- `b2_cap075_fix_n3_push04_x0b0`: 0.4 N along -x on the ring, t 25-40;
- `b2_cap075_fix_n3_pushz_x0b0`: 0.6 N down on the ring, t 25-40.

K = F / steady displacement over the last 5 s of the push. The step responses give each axis's time
constant, for the margin.

Push sizes: F <= 0.5 * K * z_i_max with the old K (0.4 N xy, 0.64 N z). The 0.1 kg M1 object (0.98 N)
is above half the z bound; that is a finding about the bound, not a test arm.

Baselines (box at zero):

| arm | none | reference mode (z_ki 0.4) |
|---|---|---|
| Gazebo hold | +0.9 cm (R0960) | not flown |
| SIL hover | -0.12 cm (R0957) | not flown |

## 5. Arms and pass / fail numbers

Cut (critic 6): the SIL circle (gate shut, identical by construction), kt110, the Gazebo hold repeats.

Order (critic 7): Stage 0, then the first model-mode Gazebo run (map-, floor start, the largest b),
checked for touchdown and the force at load-down, then the rest.

| arm | modes | supports | falsifies |
|---|---|---|---|
| Gazebo map- (drone 2 offset 0.155) | reference, model | model: z, xy within 0.5 cm of target; LAND clean; force zero at load-down | an abort; force not zero on the floor |
| Gazebo pushz (0.6 N down) | none (Stage 0), reference, model | model's peak sag, release overshoot and settling (to 1 cm) each within 30 % of reference | any of them > 1.5x reference, or a growing oscillation |
| Gazebo push04 (0.4 N, -x) | none (Stage 0), model | model: steady xy under 0.5 cm by the end of the push; release overshoot reported | steady > 2 cm, or an xy oscillation |
| Gazebo orbit | model | lap z within 1 cm, xy no worse than R0832 + 0.5 cm | a lap-rate oscillation |
| SIL ff090 | reference, model | both within 0.5 cm; K_eff = abs(d_z model) / b (reference) reported | model > 1 cm off |
| SIL kt125 | model | rails and warns, as R0480 | settles silently off target |

**Separating rows (critic 4).** Plan prediction: the planned ring position at node 5 (log.csv
`plan5_*`) against the ring measured 5 nodes later, mean over the hold. Planned against plant tension
is reported, not a bar.

**Amended 4 Oct, after Stage 0 R0961 and before any model-mode run.** With no integral, the twin hold
already reads plan5 z +1.8 cm.
- The twin holds its rods at 52.0 deg (R0960, R0961; 49.7 with the old box, R0841) against the planned
  45.
- The planner plans the swing back to 45 (node-0 rates 0.44 rad/s, `tools/hold_replay.py R0961`).
  In its model, that swing lifts the ring 2 cm by node 5. The trackers never fly it.
- This is the twin's rod model error (rod mass, 4 cm pivot), not the integral. SIL's massless rods
  hold 45.0 (R0957).

Bars (Gazebo, against R0961's +1.8 cm): model mode within 0.5 cm of it; reference mode about it plus b.
SIL keeps the original bar (model under 0.5 cm).

## 6. Repeats

Gazebo one per arm per mode; pushz repeated (both modes) for the claim. SIL one per arm. Reviewer
on the final table.

## 7. Cost

- Gazebo, laptop (Sunday, no lab PC): 2 Stage-0 + map- x2 + pushz x2 + push04 + orbit = 8. With
  R0960, 9 of this session's 10. The pushz repeats go in the next session.
- SIL: 3 runs.

## 8. New code

Built 4 Oct after Wesley's "run the plan" (the critic's challenge was read; no second critic pass):
- `mpc_planner/lumped_integral.py`: `LumpedForceIntegral`, pure; 7 tests.
- **planner_node.**
  - `_init_lumped` (the refusals), `_update_lumped` (gates, near-bound warnings, a 1 Hz
    `[planner int]` line); the force goes through `_apply_load_force`, zero with the ring down.
  - Resets per flight and on a resize.
  - `_update_zbias` is skipped in model mode, so the target is never moved.
  - log.csv appends `int_b*`, `int_f*`, `plan5_*`.
- **Launch:** `int_mode`, `int_k_xy`, `int_k_z` in common.yaml (passed only when set),
  PLANNER_OPTIONAL, and the refusals in `planner_options(context, resizes)`; 1 launch test.

If it fails, the knob stays at `reference` and the code is deleted after Wesley's decision.

## Critic

**Critic, 4 Oct 2026: NOT READY.** Rewrite the card. Integral action makes the steady error zero in
both modes whatever k_z is, so the steady arms cannot tell the modes apart. Stage 0, as written,
measures the value it was given. The one transient arm is sized past both the force bound and the
gate. The card's own refusal would also block every SIL arm.

1. **Tried?** Yes, the mechanism has been tried:
   - an integrating force in `d_L`: the overnight observer (R0869-R0884; window version R0882-R0884;
     Gazebo R0880 +7.8);
   - the 23 Sep Kalman force (R0331-R0366);
   - the integrator itself: z_ki (R0473-R0488).

   The only new part is the driving signal: tracking error instead of the plan's prediction error.
   That change is what ties the fixed point to the target. Node-0 re-pinning and the M-hautus null
   direction change what d_z means and how big k_eff is. They do not change where the integrator
   stops, as long as the gate is open and the integrator is off its rail. Add "gate open" to the
   internal-model-principle condition.
2. **Simpler explanation.** The internal offset was the tracker's node-0 box (SIL R0956 +6.76 ->
   R0957 -0.12; Gazebo R0960 +0.9). On the matched plant, either placement has at most 1 cm left to
   correct. What is left is external error or faults: the rig's C1 sat on the rail at 1.15 m on
   1 Oct, and the B4 replay shows -0.4 to -0.5 N. Moving the integral covers that no better (same
   bound, same gate), and A is the 3 Oct choice for it.
3. **Falsifier.** "Every steady arm" can only fail through slow convergence, the rail or the gate,
   so it tests k_z, not the mechanism.
   - Stage 0's "d_z / b at convergence" is z_int_k by definition in model mode.
   - The real test is the transient, and its only arm (-2.0 N) is above the force bound
     (k_z * z_i_max = 8.5 * 0.15 = 1.28 N).
   - At the old 8.2 N/m it would sag about 24 cm, right at z_i_gate 0.25. Both modes would rail or
     freeze, so the arm measures the bound and the gate.
4. **Breaks.**
   - Every SIL arm launches `mode: attach` (reserved_attach 0), so "refused outside carry" blocks
     Stage 0 and the whole SIL matrix.
   - Nothing refuses model mode with A on (the same `d_L` channel) or with kt_trim on (the same
     common-mode channel; kt_trim is the legacy-sim default).
   - It moves the 30 Sep safety net inside the model, which is the supervisor's "must not stand in
     for the model", and silently changes its bound from metres to newtons.
   - On the rig, the operator loses the "desired + height integral" line.
   - The network, the gate thresholds and the hardware are untouched, because model mode is refused
     there.
5. **Asked?** Yes: Wesley, 4 Oct, "into the planner's model". He did not ask for a 37-run three-way
   matrix. The z_ki 0 column with the box at zero belongs to the box card. x0_relax_frac 0 is not the
   default (common.yaml 0.025) and has one Gazebo run (R0960).

**"Physically consistent" does not separate the modes.**
- At equilibrium both modes plan the same tension: reference mode through its position error toward
  z_tgt + b, model mode through d_z.
- Under a real ring force (pushz), planned equals the plant tension in both modes.
- Under an internal error (ff090, map, kT), planned differs from the plant tension by the same amount
  in both, because d_z is not a force on the ring (M-hautus; R0957). The "planned vs flown tension"
  row is wrong in both columns.
- What separates them is the plan's prediction. Model mode plans a stationary ring at the target,
  and that is what happens. Reference mode plans a climb toward target + b that never comes. One
  appended log.csv column (planned ring z at node 5) scores this.

**Dynamics.**
- The force path does not skip the planner's position loop: the OCP still closes on the re-pinned
  ring.
- What changes is that b reaches the trackers as immediate feedforward tension (node-0 tension is
  free, and the cone's t_ref is proportional to |g_eff|), instead of through the target response.
- That gives less phase at crossover (z_ki * k_z / k_eff). Size it from a measured step, not from
  "possibly faster, possibly less damped".

**Examiner**, in this order:
- What physical force is d_z, given your own Hautus test and the box result?
- Why is this not just z_ki with a different gain, when integral action fixes the DC behaviour?
- Why cite offset-free MPC for an integral of the tracking error? An estimator integrates the
  innovation, not the tracking error.
- What is the phase margin?
- What does a 1.28 N bound hide?

**Cheaper.**
- (a) Zero code: write the derivation the EKF critic proposed. z_ki is the integrating
  output-disturbance form, which is already "in the model" in the offset-free sense. Then ask Wesley
  one question: does he still want the force form?
- (b) If he does, about 7 runs answer it:
  - a Stage-0 step (SIL);
  - ff090, reference and model (SIL);
  - map-, reference and model (Gazebo, floor start);
  - a sized pushz, reference and model (Gazebo);
  - 2 repeats on pushz.

Must-fix:
1. Make Stage 0 a real measurement.
   - Either take k_eff = |d_z(model)| / |b(reference)| from the ff090 pair, or fly one SIL run with
     z_ki 0 and a fixed model force of +-0.5 N. That run gives the DC gain, linearity across both
     signs, and the time constant for the margin.
   - Set z_int_k = k_eff. If it falls outside the window, re-derive; do not fly at 8.5.
   - Say that 8.1-8.2 N/m are old-box numbers (R0475, R0869). The box was likely the soft part: a x10
     planner weight bought only 38 % (R0856-R0858).
2. Key the refusal on attach capability (reserved_attach > 0, enable_approach, reconfig), not on the
   launch mode, and unit-test it.
3. Refuse model mode with dist_est ring/full and with kt_trim on, as the observer does. Allow
   `offset_free shadow`.
4. Replace the tension row with the plan-prediction metric as the scored row that separates the
   modes. Report planned vs plant tension, but not as a bar.
5. Size the push from Stage 0: F <= 0.5 * k_eff * z_i_max, with the z_ki-0 sag below z_i_gate / 2.
   Match the bounds in newtons. The M1 object is 0.1 kg (decisions 2026-09-27), not 0.2.
6. Cut the no-op arms:
   - the SIL circle: v^2/r = 0.32 > 0.05, so the gate never opens (R0481 zI 0) and the modes are
     identical by construction;
   - kt110: it rails like kt125 (R0479/R0480);
   - the Gazebo hold repeats: b is about 0.9 cm there (R0960).

   Repeat only map- and pushz.
7. Fly one Gazebo floor-start run in model mode first (map-, the largest b: R0919 +12.8 with the old
   box), because model mode holds the force through LAND to load-down. Check touchdown and d_z at
   load-down before the SIL matrix.
8. Reframe it as integral control routed through the model's disturbance input, citing the internal
   model principle (Francis & Wonham), not as an input-disturbance estimate. State that d_z is
   fictitious for internal errors.
9. Cite decisions 2026-09-30 (safety net; the model is judged with it off) and 2026-10-03 (A
   preferred). State the bound in N and what it can hide. Put z_force in the panel and the 1 Hz log
   line.
10. State that the claim holds only with x0_relax_frac 0, and is void if Wesley keeps 0.025.

Checked before pasting: the SIL arms do launch `mode: attach` (`carry_hover_n3_long_ff090.yaml:9`),
and the M1 object is 0.1 kg (decisions.md:50).

## Results (4 Oct, one run per arm and mode; exploratory until the repeats and the reviewer)

Stage 0, box at zero, nothing on: K_xy 5.26 N/m (R0961, 0.4 N), K_z 7.13 N/m (R0962, 0.6 N); tau 1.05 /
0.89 s. Model-mode configs use k = K.

| arm | none | reference (z_ki on the target) | model (x/y/z ring force) |
|---|---|---|---|
| SIL ff090: z err / target shift / plan5 z / rods | | -0.00 cm / +10.0 cm / +4.15 cm / 48.0 deg (R0964) | -0.01 cm / 0 / 0.00 cm / 45.0 deg (R0965) |
| SIL kT +25 % fault | | rails, warns (R0480, old box) | rails at 1.07 N, warns (R0966) |
| Gazebo map-: z / xy / tilt | +12.8 / 3.5 / 6.0 (R0919, old box) | -0.03 / 3.45 / 6.4 (R0967) | -0.03 / 0.01 / 7.9 (R0963) |
| Gazebo 0.6 N down: steady / peak / release overshoot / settle | 8.41 / 8.90 / 0.40 cm / 1.5 s (R0962) | 0.24 / 6.67 / 7.15 cm / 4.8 s (R0968) | 0.25 / 6.51 / 7.00 cm / 4.6 s (R0969) |
| Gazebo 0.4 N -x: steady / peak / release overshoot / settle | 7.61 / 7.73 / 0.11 cm / 1.4 s (R0961) | (z only) | 0.08 / 7.11 / 7.00 cm / 5.3 s (R0971) |
| learned force step vs the true push | | | z -0.58 / -0.60 N; x -0.42 / -0.40 N |
| Gazebo orbit: first-lap xy / z | 2.84 / +4.76 (R0832, old box) | 2.79 / +1.09 (R0910, old box) | 0.72 / +0.19 (R0972) |

Readings:
- **Steady state:** the same in both modes, as the critic said.
- **Transient:** the vertical push is within 3 % between the modes. With k = K the loop is the same.
- **What separates the modes:**
  - the plan's prediction (SIL 0.00 against 4.15 cm);
  - the formation (rods at the planned 45.0 against 48.0 deg);
  - x and y, which only model mode covers.
- **Real forces:** for a real force, the learned force is that force within 5 %.
- **Costs:**
  - A 7 cm overshoot on release of a 0.4-0.6 N step, the same as z_ki.
  - +1.5 deg ring tilt when one drone's map is off (map-: the xy correction leans the ring against an
    uncontrolled moment).
  - The bound is meaningful in newtons: SIL ff090 used 89 % of the z bound, because k (Gazebo K) is below
    SIL's K_eff of 9.48 N/m.
- **Orbit:** the xy gain is not the integral. b_xy stayed 0, because the orbit starts at the lift. It is
  most likely the box at zero; confirm with one box-0 orbit without the integral.
- **Twin:** plan5 z is polluted by the rod-swing artefact (R0961, rods 52 deg), so it separates the modes
  only in SIL. In Gazebo, plan5 xy separates them (map-: 1.4 cm against 0.0).

Session: 10 Gazebo runs on the laptop (R0960-R0963, R0967-R0972, one void R0970), 3 SIL. Not done: the
repeats (pushz both modes, push04, map- model), the box-0 orbit without the integral, the reviewer.

**Repeats (4 Oct, afternoon), every arm of the thesis draft flown twice:**
- R0978 hold.
- R0979 orbit.
- R0980 pushz, none.
- R0981 pushz, reference: its LAND aborted (see below).
- R0982 pushz, model.
- R0983 push04, none.
- R0984 push04, model.
- R0985 map-, reference.
- R0986 map-, model.

The repeats agree within 0.2 cm on displacement and 0.1 deg on tilt. Tables:
`thesis/Source/src/tables/` (tools/thesis_sim_tables.py).

**R0981: LAND abort.** At touchdown the cable feedforward steps off. Drone 2's sticks alternate +-0.6,
the rod shoves it 20 cm out, the ground-hold reference jumps 20 cm, and it tips. This is the W2 rod
shove. Box-0 LAND aborts: 1 of 21. With the old box: 0 of 44 on 3-4 Oct, but 2 of 32 since 30 Sep
(R0793, R0816, land_unwind card), so the rate is not distinguishable. Median touchdown tilt 15.0
against 16.8 deg. `land_unwind` is Wesley's decision for the rig.

Gain and gate, one run each:
- R0976, gain 0.8: 14 % less peak, but rings and settles 55 % slower.
- R0977, speed gate after the lift only: 7-8 % less at no cost.

## Reviewer (4 Oct, on the thesis draft tables): weak, corrections applied

Every recomputed number matched the raw logs.

Grouping and labels:
- R0981 had been dropped for an abort that came after LAND; its push is now included, with a note.
- The figure-8 mean was taken over a window that ran into the hold and LAND; over the eight itself it is
  2.6 cm (per-row sampling 2.7).
- The gain-table label for R0977 now says the speed gate is on only for 5 s after the lift.
- "Time to 1 cm" now says it counts from onset, against settling from release.
- "Learned force" is now "change of the integral force".
- Missing (1) counts added.

Overclaims reworded:
- "barely reduces the peak": a quarter vertically.
- "within 5 %": 3 % vertically, 4-5 % horizontally.
- "well inside the bound": 56 and 51 %.
- the force pause: 0.3-0.6 s.
- "leaning the formation": dropped.
- "the remainder comes from the rods": attributed, not tested.
- "this is why": consistent with.

Caveats added:
- the box-0 LAND tip;
- SIL 89 % of the bound;
- the four-drone hover offsets are the old box;
- the attach figure's older configuration, now in the text, not a comment;
- one run shown per figure.

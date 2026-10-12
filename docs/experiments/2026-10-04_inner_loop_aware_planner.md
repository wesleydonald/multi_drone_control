# inner_loop_aware_planner (4 Oct 2026): make the TU Delft planner predict what our trackers actually do

## 1. Question

Wesley, 4 Oct (option 1, after the literature search): keep the TU Delft load–cable planner [Sun25] and
make it aware of how our off-board MPC trackers actually execute its plan. Does that remove the
ring's steady offset without an integral?

## 2. Hypothesis

**What the literature says.** In a cascaded MPC, an exact physical model does not guarantee zero
offset when the outer predictor leaves out the inner loop's dynamics. Re-initialising from
measurements every step, with latent initial states left free, lets the outer model reconcile its
predictions locally while the realised closed loop settles elsewhere ([Ver13] Vermillion, Menezes &
Kolmanovsky, Automatica 2013; [Dev22] closed-loop-model two-loop MPC). The fix is to predict with the
inner loop's closed-loop behaviour: a hierarchical MPC with an inner-loop reference model [Ver13], an
MPC built on the closed-loop model [Dev22, Lio20], with guarantees for quadcopter cascades in
[And24] (Andrien et al., TAC 2024).

**Why it applies here.**
- Sun et al. 2025 assume each drone flies its planned motion. They make that true with onboard
  INDI: accelerometer plus rotor speeds; "the mismatch in the planned cable tension ... is
  compensated for by the trajectory tracking controller".
- Our trackers are off-board acados MPCs on mocap. They use the planned cable term and weight the
  planner's thrust feedforward at 2e-4, and the Betaflight FCs stream no accelerometer.
- SIL with a matched plant holds +6.3 cm (R0473), while the planner closed on its own model holds
  0.0 cm (`tools/hold_replay.py`).
- Three single causes were falsified offline: the rod booking (R0868), the node-0 rate jump
  (M-pin-replay), and the unflown thrust cut (M-ancillary-gap).
- No force estimator can see the error (M-hautus).

What is left is the cascade as a whole: the tracker's closed-loop law under re-pinned plans.

**Stages:**
- **Stage A, locate it (no flight).** The tracker MPC is unconstrained near hover, so it reduces to a
  linear controller: state feedback plus reference feedforward [Faa03b].
  - Extract its equivalent gains offline by finite differences on the compiled acados tracker,
    around the hold state of R0473 (SIL) and R0841 (twin).
  - Build a reduced closed-loop plant: the rigid ring with three point-mass drones on rods, each
    drone driven by that linear tracker law with the published horizon (positions, velocities, the
    thrust feedforward, c).
  - Close the real planner (hold_replay's solver) around it.
  - Prediction: it reproduces the SIL offset. If it does, the mechanism is the tracker law and it
    can be modelled.
- **Stage B, light form: reference shaping between planner and trackers.** The planner stays exactly
  as TU Delft wrote it. A per-drone compensator inverts the identified tracker closed loop on the
  published horizon, so the tracker's realised motion equals the planned motion (an intermediate
  compensator as in [Zhu25b]).
  - Knob `ref_shaping` (off/on) in the planner's publish path.
  - Expected: SIL hover within 1 cm and twin hold within 1.5 cm, with z_ki 0 and A off.
- **Stage C, full form (its own card later).** The tracker's law goes inside the planner's model:
  drones become dynamic states coupled through the rods, and the planner's inputs become the
  references it publishes. This is [Ver13]'s structure, with its stability conditions (the
  error-system terminal set). Only after A and B.

**Expected outcome.** The offset disappears without an integral. The planner's planned transients
are flown as planned, which should also tighten the push and wind responses.

## 3. The one variable

- Stage A: none flown. An offline tool `tools/tracker_linearize.py` plus a reduced closed-loop
  replay.
- Stage B: planner knob `ref_shaping` (default off; passed only when set). z_ki 0, `dist_est` off,
  everything else pinned to each arm's config:
  - SIL `carry_hover_n3_long`, `carry_hover_n2_long`, `carry_circle_n3_long`;
  - Gazebo `b2_cap075_fix_n3` (hold), `_push`, `_orbit`, `b4_wind_circle_n3_shadow`.

## 4. Baseline

| arm | none | z_ki 0.4 | A + z_ki 0.4 (reviewer PARTLY) |
|---|---|---|---|
| SIL hover | +6.3 cm (R0473) | 0.0 in 6.2 s (R0488) | |
| SIL n=2 | +4.2 (R0879) | | |
| SIL circle | +5.6 (R0877) | | |
| twin hold | +4.8 (R0841) | +0.1 (R0871) | +0.1 (R0940, R0953) |
| push xy (steady / onset peak / release overshoot / tilt peak) | 18.0 / - / 0 / 3.2 deg (R0856) | | 0.4 / 14.5 / +14.8 / 7.1 deg (R0928, R0942) |
| wind lap xy | 17.9 (R0864) | | 3.9 / 4.0 (R0929, R0947) |
| orbit xy / z | 2.8 / +4.8 (R0832) | | 2.6 / +0.9 (R0941) |

## 5. Pass / fail numbers

| stage / arm | supports | falsifies |
|---|---|---|
| A: reduced loop vs SIL R0473 | reproduces the offset within 1.5 cm (+4.8 to +7.8) | < +3 cm or wrong sign: the tracker's linear law is not the mechanism; stop before B |
| B: SIL hover | within 1 cm, z_ki 0 | > 3 cm |
| B: SIL n=2, circle | within 1.5 cm (circle: lap z) | no better than half the baseline |
| B: twin hold | within 1.5 cm | > 3 cm |
| B: push | steady no worse than R0856 (no integral here); onset peak and release overshoot <= R0856's | a new oscillation or a worse transient |
| B: orbit | xy <= R0832 + 0.5 cm, z within 2 cm | xy > R0832 + 1 cm |
| any | 0 solve failures, no abort including LAND | an abort |

External forces still need A: the claim here is the internal offset, and A + this replacing A + z_ki.

## 6. Repeats

Stage A: deterministic, one pass per hold state. Stage B: SIL one run per arm; Gazebo 2 per arm for
the claim, reviewer on the final table.

## 7. Cost

- Stage A: about 2-3 days of offline work, no runs.
- Stage B: about 3 SIL runs and 4 Gazebo arms x 2.
- Stage C: weeks, separate card.

## 8. New code?

Yes:
- `tools/tracker_linearize.py`: the tracker's equivalent linear law by finite differences on the
  compiled solver.
- `tools/cascade_replay.py`: the reduced ring + rods + linear trackers closed around the planner.
- Stage B: a `ref_shaping` module in the planner's publish path (pure, unit-tested on the reduced
  model), its knob and launch plumbing.

If Stage A falsifies, nothing is built for B. If B fails, the knob stays off and the code is deleted
after Wesley's decision.

## Critic

**Critic, 4 Oct 2026: NOT READY.** Rewrite the card. Stage A's test is circular, and two untested
tracker-interface causes that do not flip sign cost one SIL run each.

- **Tried?** The diagnosis is overnight section 1, what was left after R0868, M-pin-replay and
  M-ancillary-gap. R0881 (SIL trackers given the true rod force) still held +5.8 cm. "The cascade as
  a whole" fits every result because it predicts none of them.
- **The simpler explanation it misses: the tracker interface.**
  - Node 0 is a +-2.5 % box of the absolute state (tracker_node.py:1438-1447). That leaves 2.5-3.7 cm
    of drone height and +-2.5 % of throttle free. Never flown at zero width: R0476 only changed how it
    handles negative values.
  - Each 10 Hz plan is reused, unshifted, by five 50 Hz tracker solves.
  - Neither flips sign between runs.
- **Stage A is circular.** A law linearised at R0473's hold carries that equilibrium in its constant
  term, so any reduced plant with SIL's statics settles near +6.3 cm. Linearise at the target instead.
  - The law outputs stick rates for a 17-state model with attitude, so it cannot drive point-mass
    drones.
  - SIL already is the plant: put the law in place of the acados solve there.
- **Stage B.** The rod force is a constraint force on rigid rods. A per-drone inverse must assume the
  planned tension, which is right only once the plan is followed. The error lies in the common-mode
  tension direction that mocap cannot see (M-hautus). Inverting the linear part only removes an
  offset it has a model of.
- **Evidence.** The plan's steady thrust matches the flown thrust within 0.5 % (about 1 cm by R0475's
  0.06 m/s^2 per cm). So the 5-6 cm is a resting position of the re-pinned loop, not a force error.
- **Literature.**
  - Vermillion et al. is Automatica 50(1), 2014: a stability result, which supports Stage C, not
    zero offset.
  - Stage B is inversion feedforward (Devasia, Chen & Paden 1996).
  - Examiners: the internal model principle first (without a model of the constant, zero offset only
    on a matched plant). Then: Sun et al.'s INDI is incremental and acts like an integral, while this
    inverse is open-loop.
- **Breaks.**
  - "Replaces A + z_ki" contradicts the 30 Sep safety-net line.
  - The inverse must be re-identified on the rig.
  - No gate for the creep, pretension, LAND, resize or hand-over.

Must-fix:
1. Before any tools, SIL `carry_hover_n3_long`, z_ki 0, one run per change:
   - the node-0 box at zero width;
   - the plan shifted by the 20 ms ticks since it arrived (sim time);
   - `w_thr_ref` 10 (already built, never flown).

   If one closes the offset, fix the interface. If none moves it by more than 2 cm, Stage B has
   nothing to act on.
2. Stage A inside SIL. Check the law against logged R0473 tracker commands; log the active
   constraints.
3. Linearise at the target. Reproduce R0473 +6.3, R0874 -4.0, R0879 +4.2 and the R0841/R0868
   heights and elevations. A verdict for 3-4.8 cm.
4. Stage B: name the term that carries the offset; the coupled loop's eigenvalues in the
   common-mode direction; planned against actual rod force.
5. A gate; refuse in attach, dissipative, network and real modes; reset on a resize.
6. Arms: FF 0.9, map+-, kT +25 %, the four-drone even ring, a Gazebo floor start first, settling
   time against z_ki 0.4's 6.2 s.
7. Remove "replacing A + z_ki", or Wesley's word.

## Must-fix 1 results (4 Oct, SIL `carry_hover_n3_long`, z_ki 0, one run each)

| change | ring z err (cm) | rod elevation | plant tension | run |
|---|---|---|---|---|
| none (today's code) | +6.76 | 42.5 deg | 4.16 N | R0956 |
| node-0 box at zero width (`x0_relax_frac` 0) | -0.12 | 45.0 deg | 3.97 N | R0957 |
| plan advanced by the time since it arrived (`ref_time_shift`) | +5.93 | 42.7 deg | 4.15 N | R0958 |
| `w_thr_ref` 10 | +6.45 | 42.5 deg | 4.16 N | R0959 |

The box is the cause. Its half-width is 2.5 % of the absolute state, so it leaves each tracker a
dead band of about 3.6 cm in drone height and 2.5 % in throttle. With the box at zero width, node 0
equals the measurement (the textbook MPC initial condition), and the cascade holds the planned
45 deg rods and the planned tension with no integral. Stage A and Stage B have nothing left to act on
for the internal offset. The external errors (push, wind, map) still need A.

Next: a Gazebo floor-start hold (the change touches takeoff and LAND; the symmetric box tipped a
drone in LAND in R0527).

Gazebo floor start, twin hold, box at zero, z_ki 0, A off: **R0960 +0.9 cm** (R0841 +4.8; z_ki 0.4
R0871 +0.1). Ring tilt 1.17 deg (1.12). Drone tilt peaks 18.0 deg in takeoff and lift and 17.9 deg
in LAND (19.5 and 19.3). Lift 12.4 s (12.1). Landed clean, 0 solve failures. One exploratory run.

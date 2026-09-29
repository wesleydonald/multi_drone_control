# Card — attitude reference must follow the measured quaternion's hemisphere (2026-09-16)

## 1. Question
Why did one drone per rig flight (16:40 drone 2, 16:42 drone 0) spin through a full turn
with its yaw stick pinned at −1, and does flipping the attitude reference into the
measured quaternion's hemisphere stop it?

## 2. Hypothesis
The tracker's attitude error is `2·vec(q_ref ⊗ conj(q))`, which changes sign when `q`
changes sign. Mocap publishes `w ≥ 0`, so a drone resting near ±180° yaw crosses the
hemisphere the moment it wobbles past 180°; from then on the yaw correction points the
wrong way and the loop is positive feedback. Reconstructed from the 16:40 log: at the
crossing the error read +0.20 rad where the needed correction was −0.19 rad, and it grew
with the spin. With the reference aligned to the measured hemisphere before every solve,
a drone resting at 180° holds its heading through takeoff. The 16:42 spin (drone 0 at
−109°, no crossing) is NOT explained by this hypothesis and is out of scope here; the
new log columns (body rates, node-0 reference quaternion, solver status) exist so the
next occurrence can be read.

Evidence the premise holds on the rig (16:40 log, drone 2, t = 2.54 → 2.57 s): the
quaternion went (+0.013, −0.118, +0.005, +0.993) → (+0.002, +0.115, +0.001, −0.993),
i.e. x and z changed sign with w held ≥ 0; the smallest w in any rig log today is 0.002.
The lab mocap enforces w ≥ 0 whatever our own node does. The sim emulator enforces the
same (`payload_mocap_emulator.py`).

Ruled out: a reversed yaw actuator. Regressing measured yaw rate on the yaw stick over
the 16:28, 16:40 and 16:42 flights gives the same sign on every airframe, spinning or
not, and it matches the tracker model and the sim emulator (`dynamics.py:158`,
`payload_betaflight_comm.py:155`). A `yaw_sign` parameter briefly added on that theory
was removed the same day.

## 3. The one variable
Tracker parameter `qref_align_hemisphere` (new, default false = historical behaviour),
plumbed on every tracker launch. `configs/experiments/qref_hemisphere_seed19.yaml`
(false) vs `qref_hemisphere_seed19_fix.yaml` (true); the two files differ in that one
launch arg (asserted). World: `simulation_assets/three_random_seed19.sdf`
(`generate_random_world.py --seed 19 --ground-start --heading-jitter 180`, then drone 0's
spawn yaw set by hand to exactly 180.0°, recorded in the manifest): drone 0 resting ON
the wrap, so every wobble crosses the hemisphere and the two arms cannot be
bit-identical; drones 1 and 2 at −86° and −84°, payload yaw 63.8°, even ring.

## 4. Baseline
Hardware, 2026-09-16 (no registry rows yet; logs under
`results/logs/controller_quad_load/*_20260916_1640*`): drone 2 resting at +169°, yaw
stick −1.0 from t = 2.6 s, +180° of yaw in 1.4 s, fleet abort on tilt. Sim baseline is
the unfixed arm of this card, 2 repeats.

## 5. Pass / fail numbers
Metric: `tools/yaw_excursion.py <run>`: drone 0's cumulative (unwrapped) yaw excursion
`max |yaw(t) − yaw(t_TAKEOFF)|` over the 15 s after the TAKEOFF event, from the tracker
`log.csv` quaternions; TAKEOFF time from the run's `events.csv`, both in sim time. A full
spin reads ≈ 360°, a wobble across the wrap reads a few degrees. Also the yaw-stick
saturation fraction and whether the fleet aborted (manifest exit reason). On the rig
flights the tool reads 180.5° (16:40 drone 2) and 330.4° (16:42 drone 0) against 1–2° for
the healthy drones.

| arm | supports | falsifies |
|---|---|---|
| baseline (false) | excursion > 90° or an abort in ≥ 1 of 2 | excursion < 15° in 2 of 2 (sim does not reproduce the rig; the hypothesis is untested) |
| fix (true) | excursion < 15°, no abort, 2 of 2 | excursion > 30° or an abort in either run |

Drones 1 and 2 must show excursion < 15° in every run (control: no regression away from
the wrap).

## 6. Repeats
2 per arm.

## 7. Cost
4 Gazebo runs of the session cap of 6 (0 used so far).

## 8. New code?
Yes.
- `acados.align_reference_to_measured` + `q_meas` on `set_planner_reference` (returns
  the node-0 reference quaternion); tracker parameter `qref_align_hemisphere`; 8 launch
  files plumb it, default false.
- Tracker log gains `wx wy wz qref_w qref_x qref_y qref_z solve_status`.
- Unit tests: `test_planner_reference.py` (error sign flips without alignment, exact with
  it; no-op in the same hemisphere). Quick gate green (212 tests).
- Removal plan if it fails: delete the helper, the parameter and its 8 launch lines; the
  log columns stay (diagnostic).
- Proposed default change if it passes: `qref_align_hemisphere` true on every launch
  (Wesley's call).

## Critic

Challenge (critic agent, 2026-09-16), verbatim:

1. Tried? No registry row and nothing in decisions. But the symptom was fixed once
   already, "a drone did a 360° on takeoff", 2026-08-03, cured by the latched heading,
   and a competing fix for these same two flights already sits in this working tree.
2. Simpler explanation not addressed: `elrs_interface.py` `yaw_sign` with a comment
   attributing the spins to a reversed yaw response on all three new airframes. Same day,
   channel_6/AUX4 was driven on tethered airframes for the first time. Either explains
   the 16:42 spin at −109° with no crossing; the hemisphere hypothesis explicitly cannot.
   Sim cannot discriminate.
3. Falsifier. Drone 0 is at 178.854° and the datum re-latches while resting, so it must
   wobble 1.15° past 180° after TAKEOFF or the arms are bit-identical and 4 of 6 runs
   measure nothing. The metric is worse: raw arctan2 makes a 2° wobble read 358° while a
   wrapped difference caps at 180°. Needs unwrapped cumulative yaw and a clock rule
   between `log.csv` and `events.csv`.
4. Sim vs rig. The emulator does enforce w ≥ 0, so sim can reproduce, but
   `normalize_quaternion_positive_w` in our mocap node is never called, so the premise is
   unverified on the rig. `pose_qw` is already in the 16:40 log: check the sign at spin
   onset first, zero Gazebo runs.
5. Breakage/scope. `velocity_loop.py` already takes the shortest rotation, so the attach
   default is untouched; the alignment's own discontinuity at 180° error is never tested
   by a < 15° fix arm.
Verdict: not ready. Read the 16:40 `pose_qw`, rule out `yaw_sign`, fix the wrap-unsafe
metric, force the crossing deterministically.

Response (2026-09-16):
- (1) The August latch removed the initial 180° error; it does not touch the cost's sign
  sensitivity, which is what this card addresses.
- (2) Refuted by data, see §2: identical yaw command sign on all three airframes across
  three flights, spinning or not; `yaw_sign` deleted. The 16:42 spin stays unexplained
  and is stated as out of scope.
- (3) Done: drone 0 now rests at exactly 180.0°; the metric is unwrapped cumulative yaw
  with both clocks in sim time (`tools/yaw_excursion.py`, validated on the rig logs).
- (4) Done from the log, see §2: x and z flipped sign with w ≥ 0 at the onset.
- (5) Accepted: the fix arm tests the ordinary case only. A large-error case (a drone
  180° from its datum) is a separate card if ever needed; the alignment picks the
  hemisphere by dot product and is continuous everywhere except at exactly 90° of
  quaternion angle, which the latch keeps the tracker away from.

## Outcome (2026-09-16, R0276–R0279)

| arm | run | drone 0 excursion | hemisphere crossings in 15 s | rows with reference anti-aligned | abort |
|---|---|---|---|---|---|
| baseline | R0276 | 1.6° | 6 | 60 % | tilt fault on landing, drone 2 |
| baseline | R0277 | 1.7° | 6 | 66 % | tilt fault on landing, drone 2 |
| fix | R0278 | 1.6° | 7 | 0 % | none (landing disarm 0.3 s before the tilt) |
| fix | R0279 | 1.7° | 6 | 0 % | tilt fault on landing, drone 2 |

Reviewer: **not supported.** The mechanism was exercised in the baseline arm (sign
flips and an anti-aligned reference on most rows) and no spin followed; the fix arm is
indistinguishable on the metric. The reason is elementary and was missed at the design
stage: the tracker's cost is the squared attitude error, which is identical for q and −q,
so the sign of the error vector never reaches the optimum. The rig-log "wrong-sign
push" reading in §2 was a misreading of a quantity the controller does not use.

Removed the same session: `align_reference_to_measured`, `q_meas`, the tracker parameter
and its eight launch lines, the two unit tests, the pre-flight wrap warning. Kept: the
tracker log columns (`wx wy wz qref_* solve_status`), `tools/yaw_excursion.py` (its
abort detection and clock anchoring fixed per the reviewer), the seed-19 world.

Both rig spins (16:40 drone 2 at +169°, 16:42 drone 0 at −109°) remain unexplained. The
yaw command sign is consistent on every airframe, the attitude cost is sign-invariant,
the solver did not fail, and the mocap body rates are body-frame. The next rig flight's
logs carry body rates, the reference quaternion and solver status; read those first.

Side finding, 4 of 4 runs: on this ground-start world drone 2 tips past 70° on touchdown
after LAND and trips the tilt envelope (fleet abort with the load already on the floor).
Not seen on the standard worlds; the seed-19 placement lands drone 2 on the disc's rim.

# Planner replay of the 30 Sep rig flights (W2)

The planner OCP was run offline, tick by tick, on the states logged in model-f1 and lift-f7, with one thing changed per arm. The code is the planner at commit 155ee56, run from a clean copy: `tools/planner_replay.py --src <copy of src/controller_load_mpc>`. No flights or Gazebo runs were made.

## Summary

- **The model-f1 solve failures come from two faults acting together.** The replay reproduces the failures only when both are present:
  - the drones sat 30° clockwise of the attach points the planner modelled;
  - the ring quaternion from mocap changes sign when the ring's yaw crosses 180°.
- **Removing either fault removes every failure:**
  - keeping the quaternion sign continuous (arm g);
  - rotating the modelled attach points to where the drones sat (arm h).
- **The other toggles do not help.** Arms a to f each leave the count unchanged, or move the failures without removing them. That includes W8's full reset.
- **The offset was only in model-f1.** It is the only rig flight where the drones sat off their slots, and the only one with solve failures.
- **Node-0 over-planning is small.** At the flown geometry the planner asks for 2 to 8 % more vertical rod pull than the ring's weight, not 14 to 25 %. So it does not explain a 23 % gap at breakaway.
- **f7: the planner corrects the heave rather than causing it.** Its references are rebuilt from the logged states to within 0.6 cm, and they push back against the heave. The f7 hold was one drop of the ring followed by a slow recovery on the height integral.

## 1. What the replay feeds the solver

| input | source | approximation |
|---|---|---|
| tick times | `load_planner_*/log.csv`, one row per `_plan` call | skipped ticks (planner blocked) are skipped here too |
| load p, vz | the planner row | as logged (4 decimals) |
| load vx, vy | drone 0 `payload_x/y` (50 Hz) | differenced after a 5-sample mean |
| load yaw | not logged | the latched datum plus the mean change of the four drones' azimuths about the ring |
| load roll/pitch | not logged | tilt axis from a least-squares fit of the four rod lengths; tilt size set to the logged `tilt_deg` |
| load ω | not logged | 0 (arm e adds noise) |
| drone positions | tracker `pose_x/y/z` | interpolated at the tick |
| lift schedule | `z_tgt`, `lift_progress`, `z_bias` from the row | lift_z0 = z_tgt − lift_progress; lift_vel = row-to-row change × 10 Hz |
| creep | every creep row primes the solver on the hold reference | as `_prime_solver` |
| geometry | params.json and the launch log | model-f1: n 4, load 0.86, drone 0.55, radius 0.225, attach_z 0, rods [0.613, 0.605, 0.587, 0.597] from hand-over, slot map [2, 0, 3, 1], yaw datum +158.9° |
| end of the replay | LAND with the ring within 5 cm of its lift height | the node stops solving there too |

**How far to trust it:**

| check | result |
|---|---|
| f7: replayed node-0 drone references against the tracker-logged ones (hold) | median error 0.1-0.6 cm per axis; z p95 1.1-2.2 cm |
| model-f1: rod-fit tilt against the logged tilt | corr +0.84, median difference 1.3° |
| f7: rod-fit tilt against the logged tilt | corr +0.83, median difference 1.0° |
| model-f1 warm solve time on this laptop (5 SQP iterations) | median 34 ms, max 37 ms |

**Limits:**
- Yaw is the weak part of the reconstruction: drones can swing sideways on their rods without the ring turning.
- The replay is open loop. The logged states are the ones the rig produced with its own references, including those held after each failure. The count of failures is therefore indicative, not exact.

## 2. model-f1: toggles (solve status and node-0 pull)

The rig logged 25 failures: 18 between the hand-over and LAND, and 7 between LAND and the ring touching down. The first came 3.8 s after the hand-over.

Arm meanings:
- a: `solver.reset()` before every reseed.
- b: cable references at 53.6°, the hand-over elevation.
- c: rod from a pivot 4 cm below the drone centre, rods 0.55.
- d: load pose 0.5 s stale, drones current.
- e: ring ω noise, sd 0.3 rad/s.
- f: radius 0.25, rod 0.53, drone 0.525.
- g: quaternion kept in one hemisphere.
- h: attach azimuths rotated by the rest offset (−29.7°).
- y: ring yaw held at the datum.

| arm | status≠0 hand-over..LAND | LAND..ring down | first (s after hand-over) | within 0.15 s of a rig failure | max solve ms | ticks > 100 ms | node-0 Σt·s_z/W median | airborne median | min..max |
|---|---|---|---|---|---|---|---|---|---|
| rig (logged) | 18 | 7 | 3.8 | | | | | | |
| base | 9 | 5 | 4.2 | 4 | 583 | 27 | 0.984 | 1.020 | 0.95..1.12 |
| a reset + u=0 | 9 | 5 | 4.2 | 4 | 546 | 27 | 0.984 | 1.020 | 0.95..1.12 |
| b refs at 53.6° | 9 | 5 | 4.2 | 4 | 944 | 25 | 0.946 | 0.976 | 0.90..1.08 |
| c pivot 4 cm, rods 0.55 | 9 | 5 | 4.2 | 4 | 544 | 27 | 0.973 | 1.007 | 0.93..1.11 |
| d pose 0.5 s stale | 7 | 3 | 4.7 | 6 | 594 | 19 | 1.011 | 1.036 | 0.95..1.10 |
| e ω noise 0.3 rad/s | 9 | 5 | 4.2 | 4 | 494 | 27 | 0.989 | 1.022 | 0.95..1.12 |
| f old geometry | 9 | 5 | 4.2 | 4 | 1175 | 27 | 0.971 | 1.008 | 0.93..1.11 |
| g quaternion sign continuous | 0 | 0 | | 0 | 36 | 0 | 0.984 | 1.021 | 0.95..1.12 |
| h attach azimuths −29.7° | 0 | 0 | | 0 | 50 | 0 | 1.076 | 1.119 | 1.03..1.21 |
| y yaw held at datum | 0 | 0 | | 0 | 37 | 0 | 1.064 | 1.107 | 0.96..1.20 |
| g+h | 0 | 0 | | 0 | 35 | 0 | 1.077 | 1.119 | 1.04..1.21 |
| a+g | 0 | 0 | | 0 | 38 | 0 | 0.984 | 1.021 | 0.95..1.12 |
| b+c | 9 | 5 | 4.2 | 4 | 891 | 27 | 0.934 | 0.963 | 0.89..1.07 |
| a+b+c+e | 9 | 5 | 4.2 | 4 | 964 | 27 | 0.935 | 0.965 | 0.89..1.07 |
| base, current working tree (W8 in progress) | 9 | 5 | 4.2 | 4 | 529 | 14 | 0.984 | 1.019 | 0.95..1.12 |

Failed and reseeded ticks take 100-550 ms against a 100 ms planner period. That matches the gaps in the rig's planner log.

## 3. The mechanism

A quaternion sign flip is a tick where the fed quaternion has the opposite sign to the previous plan's node 1. It happens because the rig's mocap sends w ≥ 0 (every tracker log shows it), so the ring quaternion changes sign whenever the ring's yaw crosses ±180°.

| arm | sign flips in the planner phase | failures | failures at a flip |
|---|---|---|---|
| base | 14 | 14 | 14 |
| h (attach azimuths where the drones sat) | 16 | 0 | 0 |

**The sequence in the base replay:**
1. Pinned to cable directions skewed 30° sideways, the plan carries a ring yaw rate of about −1.5 rad/s at node 1 on every tick.
2. When the ring quaternion then changes sign, the QP returns MINSTEP. The rig terminal shows the same signature: 48 of 80 QP errors were MINSTEP at QP iteration 1.
3. With the attach points where the drones really are, the same sign flips solve cleanly.

The rig log cannot show whether the real ring's yaw crossed 180°, because the ring quaternion was not logged. Two things point that way:
- the datum was +158.9°;
- the drones' mean azimuth moved +21° during the flight, which is what dragging the fleet toward the modelled slots would do to the ring.

**Drones' mean offset from their modelled attach azimuths, per rig flight:**

| flight | yaw datum (deg) | offset at rest (deg) | offset at hand-over (deg) | solve failures |
|---|---|---|---|---|
| r3e f2 | −135.1 | +0.8 | −0.3 | 0 |
| r3b0 f2 | −134.5 | +5.8 | +4.6 | 0 |
| r3b0 f4 | −135.1 | +0.4 | −5.4 | 0 |
| hold f2 | −108.4 | +1.8 | −1.7 | 0 |
| lift f5 | −108.4 | −6.7 | | 0 |
| lift f6 | −178.1 | +2.5 | | 0 |
| lift f7 | −178.4 | −3.3 | −4.1 | 0 |
| lift f8 | +178.5 | +2.7 | −1.8 | 0 |
| lift f9 | +176.3 | −3.3 | | 1 |
| lift f10 | +166.1 | +1.1 | −1.7 | 0 |
| lift f11 | +152.4 | −1.5 | −2.8 | 0 |
| **model-f1** | **+158.9** | **−29.7** | **−27.7** | **25** |

The four model-f1 drones sat at −28, −30, −31 and −30° on the floor, within 3° of each other. That points to the magnets being on the next plate (plates are 30° apart), or to the ring's mocap body having been re-made between f11 and model-f1. It does not look like drones placed badly by hand.

## 4. Static hold (`tools/test/test_planner_static_hold.py`)

Setup: a level ring at 0.5 m, still, with every rod at the given elevation. The planner references stay at 45°. Geometry: n 4, load 0.86, drone 0.55, radius 0.225, rod 0.55. Fifty ticks per case.

| rod elevation | status≠0 in 50 ticks | node-0 Σt·s_z / (0.86·9.81) |
|---|---|---|
| 45° | 0 | 1.000 |
| 53.6° | 0 | 1.053 |
| 65° | 0 | 1.078 |

## 5. f7 heave

f7 was flown on the old geometry: radius 0.25, typed rod 0.47 (the measured rods were rejected), drone 0.525, z_ki 0.4. The window is the hold after the lift topped out, before LAND.

| drone | ticks | replayed − logged node-0 ref, median abs x / y / z (cm) | z p95 abs (cm) | logged ref_z − pose_z sd (cm) | corr(node-5 ref z − pose_z, ring vz) |
|---|---|---|---|---|---|
| 0 | 127 | 0.2 / 0.2 / 0.4 | 1.1 | 0.9 | +0.70 |
| 1 | 127 | 0.4 / 0.2 / 0.4 | 1.5 | 1.0 | +0.68 |
| 2 | 127 | 0.2 / 0.4 / 0.5 | 1.9 | 1.0 | +0.70 |
| 3 | 127 | 0.3 / 0.4 / 0.6 | 2.2 | 1.1 | +0.65 |

| quantity | value |
|---|---|
| hold length | 12.6 s |
| ring z sd / p-p | 4.5 / 19.1 cm |
| height target (z_tgt + z_bias) sd | 6.2 cm (the integral ran from 0 to its 0.15 bound) |
| planned ring z at the horizon end minus the ring now, against the miss: corr / slope | −1.00 / −0.98 |
| node-0 thrust feedforward a_z: corr with ring vz | −0.28 |
| node-0 Σt·s_z/W (hold median; base and pivot arm c) | 1.16 / 1.16 |
| status≠0 (base, c) | 0 / 0 |

**What the numbers say:**
- The planner, fed the logged states, gives back the logged references. So the heave is in the references only because node 0 is pinned to the heaving ring.
- Over the horizon the plan removes the whole miss (slope −0.98), and its feedforward opposes the ring's velocity. The planner acts as a restoring element; it does not drive the heave by itself.
- The f7 "heave" is mostly one event:
  1. the ring overshot to 0.385 m;
  2. it sank to 0.20 m within about 2.5 s;
  3. it climbed back while z_bias integrated to its bound.
- Whether the closed loop (planner, tracker lag, ring) oscillates can only be answered closed loop (SIL or the twin), not by this replay.
- f7's node-0 pull is 16 % over the ring's weight in both base and c, so it is not caused by the typed rod length.

## Decisions for Wesley

1. **The ring's attach frame in model-f1.** Were the rods moved to other plates, or was the ring's mocap body re-made, between f11 and model-f1? For tomorrow, a pre-flight check could warn when the drones sit more than 10° off their slots at rest.
2. **Quaternion continuity in the planner.** Flip the fed ring quaternion into the hemisphere of the previous one. This is the arm g fix. It is a one-line change in `build_x_init`, but it touches the planner's state input, so it goes to the card A critic alongside W8.
3. **W8's full reset (arm a) does not remove these failures.** W8 remains useful as the time budget: failures blocked the planner for 100-550 ms. It should not be credited with fixing model-f1.

## Reproduce

```
tools/planner_replay.py --src <pkg> --arms base,a,b,c,d,e,f,g,h,y     # model-f1 table
tools/planner_replay.py --flight f7 --arms base,c --heave              # f7 table
tools/planner_replay.py --static-hold 45,53.6,65                       # static hold
```

- The acados solver is built under `$TMPDIR/mdc_planner_replay_<uid>/src_<hash>`, one tree per `--src`, never in the repo.
- Each arm runs in its own process, and the per-tick JSON is written next to it.
- Source the acados environment (`ACADOS_SOURCE_DIR`, `LD_LIBRARY_PATH`, the `acados_template` path) first.

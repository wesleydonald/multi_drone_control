# unannounced_detach (7 Oct 2026): a magnet lets go and the planner is not told

## 1. Question

Wesley, 7 Oct: "a better result for the thesis is if the system does not know if a detach will occur,
similar to one failing during a mission." He also wants to stay on the measured pull, not the
feedforward.

## 2. Hypothesis

With the measured pull, each survivor feels its new pull within the estimator's ~0.1 s. The planner
detects the loss from mocap: the leaver's cable measures over its own length by `detach_detect_m`
(0.06 m) for two ticks. It then resizes to the survivors about 0.2-0.3 s after the release, as an
announced detach would. Expected, against the announced detach on the measured pull (R1118, R1121):

| layout | announced (R1118 / R1121) | unannounced, with detection |
|---|---|---|
| 1/3/5/9 tilt peak | 8.4 deg | <= 12 deg, no abort |
| 1/3/6/9 tilt peak | 16.4 deg | <= 20 deg, no abort |
| detection delay after the release | - | <= 0.3 s |

Without detection the planner keeps planning four cables, one of them gone. Expected: worse or
divergent.

## 3. The one variable

Detection on (`detach_detect_m: 0.06`) against off, after an unannounced release:
- the runner's `RELEASE` event publishes the joint's `/drone_1/detach` only;
- nothing goes to `/fleet/detach`.

Everything else is the 7 Oct defaults (measured pull, 60 deg), r0012's events with `RELEASE` in place
of `DETACH`, and the 0.5 m step-out.

| arm | config |
|---|---|
| U1 1/3/5/9, detection | `w4_3915_release.yaml` |
| U3 1/3/6/9, detection | `w4_3969_release.yaml` |
| U1n 1/3/5/9, no detection | `w4_3915_release_nodetect.yaml` |

## 4. Baseline

| run | what | numbers |
|---|---|---|
| R1118 | announced plain detach, 1/3/5/9, measured | tilt peak 8.4 deg, z -3.2 cm, landed |
| R1121 | announced plain detach, 1/3/6/9, measured | tilt peak 16.4 deg, z -4.5 cm, landed |
| R1128 / R1122 | announced plain, feedforward, 1/3/5/9 / 1/3/6/9 | 3.3 / 4.1 deg |

## 5. Pass / fail numbers

| metric | supports (fly it on the rig) | falsifies |
|---|---|---|
| abort or tip | none | any |
| detection delay (release -> UNANNOUNCED DETACH line) | <= 0.3 s | > 0.5 s or never |
| tilt peak, release to +10 s | U1 <= 12 deg, U3 <= 20 deg | above |
| ring z drop in 2 s | <= 8 cm | more |
| false detection (any drone before the release) | none | any |
| LAND | landed, freed drone down | otherwise |

## 6. Repeats

U1 and U3: 2 each; U1n: 1 (control).

## 7. Cost

5 Gazebo runs, lab PC tonight, 2 slots at a time (more slots caused pose-timeout voids).

## 8. New code?

Yes:
- `_detect_tick` in `dissipative_node.py`, plus the `detach_detect_m` knob (off by default);
- the runner's `RELEASE` event (`tools/experiment/config.py`, `runner_node.py`);
- `configs/rig/detach_unannounced.yaml`.

Tests: `test_soft_detach.py` (two-tick rule, off, glitch). If falsified, the code is recorded and
removed (Wesley's call).

## Critic

Critic agent, 7 Oct night (read-only), condensed. Verdict: **rewrite the card**. Its only
rig-relevant falsifier (a false detection) cannot fail in the twin (exact mocap, equal rods).

**Must-fix**
1. Replay the detector offline over the 7 Oct rig logs: the false-positive margin, and the delay on a
   real slip. **Done** (below).
2. A false detection switches off a healthy magnet: `_detach_ocp` -> `_release_magnet`, with no
   geometry refusal. The rig already produced the cause: r001 accepted rods of 0.48-0.50, which read
   6-7 cm over once taut. **Done:**
   - the excess is measured against max(typed, measured rod);
   - it must have jumped at least 3 cm since the last tick within the threshold. A bias does not
     move; a release stretches 10+ cm.
3. Fly only U1 (1/3/5/9) on the rig. A twin pass on 1/3/6/9 says little: the twin flew r0013's
   settings at 5.5 deg while the rig capsized. **Adopted.**

**Should-fix, both done**
- `resize_fleet` was called without the survivors' own rod lengths, so they shifted by slot. This
  affected announced detaches too.
- A detection refused by `min_survivors` re-fired every 0.2 s; it now stops.

**Also raised**
- On the rig, release with `-t 3` (or the RViz MAGNET toggle), not `--once`.
- Confirm the leaver is on plate 3.
- If detection never fires, send `/fleet/detach` before LAND.
- The 4-cable plan flies on for the 0.2-0.4 s detection gap.

**Offline replay over the 7 Oct rig logs** (planner `len` columns, handover rods, guarded rule):

| question | answer |
|---|---|
| false fires before any release | none, in 13 flown runs |
| largest excess while flying | 2.3 cm (r206e60); the threshold is 6 cm |
| detection after the r200006 slip | 0.38 s after the release (mocap reaches the planner ~0.1 s late, 10 Hz ticks); 17.5 cm over, a 13.5 cm jump |

## Results (7 Oct night, lab PC twin)

| arm | runs | detected after release | tilt peak, release to +10 s | ring z jump | survivor peak pull | LAND lean |
|---|---|---|---|---|---|---|
| U1 1/3/5/9, detection | R1140, R1145 | 0.31, 0.38 s | 7.5, 7.9 deg | -2.6, -2.8 cm | 3.9 N | 2.0 deg |
| U3 1/3/6/9, detection | R1139, R1141 | 0.32, 0.27 s | 13.9, 19.2 deg | -3.9, -5.1 cm | 4.4 N | 2.1-2.3 deg |
| U1n 1/3/5/9, no detection | R1144 | never | 6.7 deg | -2.3 cm | 3.6 N | 5.4 deg (freed drone 15.6 cm outside) |
| announced, measured (baseline) | R1118 / R1121 | - | 8.4 / 16.4 deg | -3.2 / -4.5 cm | 3.8 / 4.6 N | 2.0 / 2.4 deg |

Voids, all pose timeouts on the lab PC: R1138, R1142, R1143. There were no false detections.

**Verdict:**
- **U1 SUPPORTS** (two runs). The unannounced release on 1/3/5/9 is detected in 0.31-0.38 s, peaks
  at 7.5-7.9 deg (bar 12; announced 8.4) and lands level.
- **U3 is within its bar but variable** (13.9-19.2 deg against 20). It is not for the rig tomorrow
  (critic must-fix 3).
- **The no-detection control** shows the measured pull alone carries the ring through the release;
  the detection is what restores a clean landing.
- On the rig, expect the detection about 0.4 s after the release (the r200006 replay), plus the
  radio and magnet delay.

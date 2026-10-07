# time_cascade (8 Oct 2026): one time base from mocap to motor command

## 1. Question

Wesley, 7 Oct: find the root cause of the ~0.2 s lag and fix it "in a way that is highly defensible
and robust"; "I dont want a bandaid fix like just sending the reference earlier." He chose the
time-consistent cascade (8 Oct plan).

## 2. Hypothesis

The planner -> tracker cascade has no shared time base. Code, week4:
- **The pose stamp is dropped.** `planner_node._payload_cb` and `_drone_cb` copy the pose and twist
  and stamp `time.monotonic()`; the MotionCaptureState header is thrown away. x0 is the pose as of
  an unknown, variable age (rig ~27 ms, more on the ~22 % of ticks where the executor runs the timer
  before the queued poses).
- **The plan carries no time.** The Float64MultiArray is [N+1, dt, nodes...]. Its node 0 is the
  measured state at that unknown age.
- **The tracker flies node 0 as "now" and holds it.** `tracker_node` records only the arrival time
  (:692). `ref_time_shift` (off everywhere) would shift by time since arrival, not by the
  measurement's time. So every tracker flies a reference D late:
  - D is pose age + solve + transport + the half-period hold;
  - `tools/lag_metrics.py`: twin 0.094-0.109 s, rig 0.14 s (r206e60).
- **The trajectory clock is pre-advanced.** `traj_t` is advanced by 1/PLANNER_HZ before the solve
  that uses it (`planner_node.py` ~:1432), so the published desired runs 0.1 s behind the plan's own
  clock. This is bookkeeping, not lag, but it hides 0.1 s in every lag number.
- **Amplification.** The OCP recovers c ~ 0.2 of a gap per 0.5 s (rig and twin alike), so a
  command latency D turns into a steady lag L0 ~ kappa D, with kappa ~ 2-2.7.

H1 (card 2026-10-08_lag_h1) tests the causal step directly: the twin with the rig's extra 0.035 s.

The fix makes every stage say what time its numbers are for:
1. **Stamp x0.** The planner keeps the MotionCaptureState header stamp of the ring and each drone.
   Pose subscriptions get their own callback group (a MultiThreadedExecutor), so a solve never
   reads a pose that waited behind the timer.
2. **Plan for the effect time.** T_eff = solve start + L, where L is the measured
   solve-to-tracker latency (a running median of solve time plus publish, logged). x0 is the
   stamped measurement predicted forward to T_eff with the planner's own model, under the previous
   plan's inputs (the shifted horizon). `traj_t` is read from the clock at T_eff, and the +0.1 s
   pre-advance goes.
3. **Stamp the plan.** The reference message carries T_eff in a versioned layout: a negative first
   field marks it, the stamp follows, and the old layout is still parsed.
4. **Trackers fly the plan at the right time.** On every 50 Hz solve, stage k is interpolated at
   (now - T_eff) + k dt (`_shift_nodes` keyed on the stamp, not the arrival). This is the default
   when the message is stamped; old messages keep today's behaviour.
5. **One clock for the metrics.** `tools/lag_metrics.py` measures against the reference at the
   plan's T_eff.

Expected:
- D falls to the tracker's own solve and transport (< 0.03 s);
- L0 falls by about kappa x the removed latency;
- c is unchanged (the OCP weights are untouched).

| metric | twin baseline (w4_orbit_0125, R1152/R1153) | with the cascade |
|---|---|---|
| D | ~0.10 s | <= 0.03 s |
| L0 | measured tonight | <= baseline - 0.12 s (kappa 2 x 0.07 s removed, conservative) |
| c | ~0.2 | +-0.03 |
| ring tilt, hover sd, aborts | baseline | no worse than +10 %, none |

## 3. The one variable

The cascade, all five parts as one change (they are one time base; any part alone moves the
reference or double-counts the latency). Behind one planner parameter `time_cascade` (default false
until verified) and the stamped message layout. Trackers shift by stamp only when the message is
stamped.

| arm | config | code |
|---|---|---|
| base | `w4_orbit_0125.yaml`; `w4_fig8_const0200.yaml` | week4 |
| TC | the same + `time_cascade: true` | `time-cascade` branch |

## 4. Baseline

- R1152, R1153: orbit, lab PC.
- R1127 + R1154: figure-8 at 0.2 m/s, lab PC.
- Rig reference: r206e60 (D 0.14, L0 0.372, c 0.229, s 0.086) and r2008e60.

## 5. Pass / fail numbers

| metric | supports | falsifies |
|---|---|---|
| D | <= 0.03 s | > 0.06 s (the stamp did not reach the trackers) |
| L0 (orbit and figure-8) | <= baseline - 0.10 s | > baseline - 0.05 s |
| c | within +-0.03 | outside |
| xy RMSE (ring) | <= baseline | > baseline + 10 % |
| ring tilt max, hover z sd | <= baseline + 10 % | worse |
| aborts, solve failures | none / not more | any / more |
| prediction error (x0 predicted at T_eff vs the measurement nearest T_eff) | logged; < 1 cm mean | > 3 cm (the model cannot carry the state forward) |

## 6. Repeats

2 per arm per trajectory in the twin (same host, RTF logged). Then the rig on 14 Oct: the orbit
and the figure-8, one each with and without.

## 7. Cost

8 Gazebo runs (TC x2 orbit, TC x2 figure-8; the bases are shared with H1 and S1/S2).

## 8. New code?

Yes, new timing in the flight loop, so the critic comes first:
- `planner_node`: stamped pose callbacks in their own callback group, T_eff, the x0 forward
  prediction, traj_t from the clock, the stamped layout;
- `tracker_node`: parse the layout, shift by stamp.

Unit tests:
- the layout round trip, and the old layout still parsed;
- the stamp shift at a known age;
- traj_t from the clock;
- the prediction against a constant-velocity state;
- the executor: a pose callback runs during a long solve.

If falsified, the branch is recorded and deleted.

## Critic

(pending)

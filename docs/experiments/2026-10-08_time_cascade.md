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

Critic agent, 8 Oct (read-only; pasted here by the main session), condensed. Verdict: **rewrite the
card**. Predicting the state forward to the time the plan is used, and stamping the plan with that
time, is standard and defensible. The card has three problems:
- it bundles five parts as one variable;
- two of them (removing the `traj_t` pre-advance, and moving the lag ruler) shift the deciding number
  by about its own falsifier margin;
- it is not gated on H1.

**The five questions**
1. **Tried before?** Only in pieces, and never for lag:
   - `ref_time_shift` (part 4, keyed on arrival): flew once, R0958 (SIL hover, exploratory);
   - `TRACKER_DELAY_COMP_S`: R1101 VOID, never evaluated;
   - stamp-differenced poses: R0700-R0703 VOID, removed in 364889d;
   - the 0.2 s reference lead: struck by Wesley (d1b999b).
2. **Simplest explanation it does not address.** L0 = s/c to the millisecond:

   | case | s/c check |
   |---|---|
   | rig r206e60 | 0.229 x 0.372 = 0.085, against s 0.086 |
   | twin R1106 | 0.248 x 0.269 = 0.067, against s 0.067 |

   So the ~4x amplifier is 1/c, the plan's weak catch-up, which the card leaves alone. D is one source
   of s; the measured pull is another (R1105 vs R1106: +0.07 s L0 at the same D). Only H1 separates
   them.
3. **What falsifies it, and do the bars test it?** No bar measures the ring's lag on a fixed clock:
   - D passes by construction (node 0 becomes the drone's own predicted pose);
   - L0 is measured against `traj_t`, which the card moves;
   - the prediction bar cannot fail.
4. **What it breaks if it works:**
   - the frozen wire format (locked, CURRENT_STATE §5) and every parser of it;
   - the dissipative node's tick-counter `traj_t`;
   - the staleness gates, if `_load_t` becomes a header stamp;
   - SIL determinism under a MultiThreadedExecutor;
   - on the rig, a reference step can jerk the magnets off (r200006).
5. **Asked for?** Yes, but H1 makes it conditional. The multithreaded executor and the pre-advance
   removal were not asked for, and the wire change touches a lock.

**Must-fix**
1. **Gate on H1.** Build nothing but stamp logging until H1 supports; register R1152-R1154 first.
2. **One ruler for both arms, outside `traj_t`.** The pre-advance is a 0.1 s lead of the OCP's node-0
   reference over the published desired. The cascade swaps it for a lead of L, so it moves the
   reference 0.1 - L later, the mirror of the struck lead. The absolute lag changes by
   Δs/c + (0.1 - L); L0 sees only Δs/c. In the twin that is about 0.12 - 0.08 = 0.04 s, inside the
   0.05 s falsifier. Score both arms against path(t - t_start), with t_start the logged end of the
   hold. Or keep the pre-advance in TC and make its removal a separate, later variable.
3. **Split the bundle into rungs:**
   - a. log the stamps (no behaviour change; the rig's latency distribution on 14 Oct);
   - b. `ref_time_shift: true`: it exists, is tracker-only, removes the half-period hold (~half of
     D), and never double-counts. One exploratory twin orbit first;
   - c. the planner's x0 prediction to tick + L, with the tracker shift keyed on arrival;
   - d. the stamped wire layout, only if rung c's logs show the arrival-vs-T_eff jitter matters.
4. **A prediction falsifier that can fail.**
   - Bar: the predicted x0 must beat the unpredicted stamped x0 (position and velocity) against the
     measurement nearest T_eff.
   - Not during creep, pretension, breakaway, cap release, LAND from load-down, or the tick after a
     resize, reseed or failure.
   - Log the innovation and fall back beyond a bound.
   - Prefer the plan-delta form (measurement + last_X(T_eff) - last_X(stamp)) over a new compiled
     solver (prebuild, cache signature).
5. **Double counts and clocks, each with a unit test.**
   - HorizonFallback already shifts by age: restamp it or stop shifting.
   - Stamp shift and `ref_time_shift` are exclusive.
   - `delay_comp_s` > 0: shift to now + delay or refuse; fly both arms at 0.
   - Measure L on the node clock: `last_solve_ms` is wall time, so it over-predicts 1/RTF in the twin.
   - Keep `_load_t`/`_drone_t` monotonic, with the stamp in a new field.
   - `lift_progress`, `_ff_t` and the LAND ramp are tick counters too.
   - Creep and hold references must be stamped from the first message.
   - Bound the tracker's shift (MAX_BRIDGE_S).
6. **Drop the MultiThreadedExecutor.**
   - The gain is about one mocap period on 22 % of ticks.
   - The cost: torn reads of pos/vel/quat while acados releases the GIL, more lab load, and lost SIL
     determinism.
7. **Wire format: Wesley's word first** (locked: "frozen at 12 fields per node").
   - One shared parser for the tracker, `runner_node.py:188-193` (reads data[2:5]; the scoring
     harness), `tools/sil/bench_node.py:170` and `tools/ref_dump.py`.
   - An old tracker silently drops the new layout (a staleness disarm on the rig): preflight checks
     the build.
8. **Scope.** Refuse `time_cascade` in `dissipative_planner` until it is ported:
   - it rewinds `traj_t` in the hold (:497);
   - it advances it in the network (:1510).

   State the 14 Oct mode, and fly TC in it.
9. **Rig on 14 Oct.**
   - Before: one TC run with H1's 0.035 s, and one TC hover (a no-op but for prediction noise).
   - Bars on lift, breakaway, LAND and jerk: the largest reference step between tracker ticks and
     the peak measured-pull rate.
   - Order: hover, then orbit; base before TC on the same pack.
   - Stop rule on innovation or a reference step.

**Should-fix**
- **Measured-pull offset:** evaluate planned[k] at now - 0.09 s. Check the offset statistics, and fly
  one TC neighbour on cable_source model.
- **Fast solves** give e < 0 (a small lead): clamp T_eff to the arrival.
- **A tracker kill switch** to ignore the stamp.
- **Noise floor:** report the base arm's two-run spread on the absolute ruler. Give no verdict if it
  exceeds 0.05 s. Match RTF and slot count.
- **Prime the solver** with the first OCP tick's convention.
- **Registry:** flag that `/payload/desired_position` moves with `traj_t`.

**Also raised**
- The tracker's terminal velocity reference is zero (acados.py:309-314), another source of s.
- The rig's `traj_t` already keeps wall rate (29.90 s in 29.93 s).
- Rig solves take 49-57 ms against a 60 ms budget; log the tick with the prediction added.
- Mocap stamps are receipt times on both plants (use NatNet's latency field if exposed).
- κ = (s/D)/c.

## Revision after the critic (8 Oct night)

Adopted: the critic's rungs, gated on H1.
- **One ruler:** `tools/lag_metrics.py` also reports **A0**, the ring's lag against path(t - t_start)
  on the sim/wall clock, with t_start the first tick of the moving trajectory. That is independent of
  the `traj_t` convention, so both arms are scored on it.
- **Rung b now, no new code:** `w4_orbit_0125_rts.yaml` = the base + `ref_time_shift: true`,
  2 twin runs on the lab beside H1. Prediction if H1 holds: D drops by ~half the plan period's hold
  (~0.05 s), and A0 drops by about kappa x that. If A0 does not move, the premise is in doubt before
  any new code is written.
- **Rungs a, c, d** wait for H1 and rung b. The wire-format change (rung d) needs Wesley's word (lock).
- The MultiThreadedExecutor is dropped. Removing the pre-advance becomes its own later variable.

## Rung b result (8 Oct night, lab PC twin, one ruler)

`ref_time_shift: true`, the existing tracker switch, against the base orbit (`tools/lag_metrics.py`):

| arm | runs | D | A0 (one clock) | L0 | c | s | ring tilt mean / max |
|---|---|---|---|---|---|---|---|
| base | R1152, R1153 | 0.098, 0.098 | 0.144, 0.131 | 0.244, 0.231 | 0.235, 0.225 | 0.058, 0.052 | 0.43 / 2.7, 0.45 / 6.1 |
| rung b | R1165, R1166 | 0.041, 0.043 | 0.015, 0.007 | 0.114, 0.108 | 0.223, 0.238 | 0.025, 0.026 | 0.22 / 2.0, 0.22 / 1.4 |

- **D falls by 0.056 s.** The trackers no longer hold node 0 between plans.
- **The ring's absolute lag falls by 0.13 s, to within 1.5 cm of path at 0.125 m/s.** ΔA0/ΔD = 2.3,
  inside H1's predicted kappa of 2-2.7.
- **The catch-up c is unchanged.** It is the latency that makes the lag, not the plan.
- **Reading:** the premise holds, and rung b alone removes almost all of the twin's lag. What
  remains of D (0.04 s) is the pose age plus solve and transport, which rung c (x0 predicted to the
  effect time) would remove.
- **Rig:** D 0.145 = 0.027 pose age + 0.071 solve-to-arrival + ~0.05 hold, and rung b removes the
  hold. How much rig lag that removes is not predicted (see the reviewer note below); r302 measures
  it.

## Where this leaves the rungs (8 Oct night)

Reviewer 8 Oct 05:00: H1, rung b and the figure-8 numbers SUPPORTED. Two corrections:
- the rung-b tilt max column (2.7/6.1 -> 2.0/1.4) is a touchdown artefact. During the carry, the mean
  goes 0.51/0.49 -> 0.26/0.26 deg and the max 1.1/1.1 -> 0.7/0.8 deg;
- the twin fit A0 = 2.3 D - 0.09 does not carry to the rig: it over-predicts r206e60 by 0.03 s and
  misses r2008e60 by 0.12 s. **There is no rig prediction for rung b**; the rig A/B (r302) measures it.

H1 supports (R1172/R1173: +0.035 s of latency -> +0.083 s of lag, c unchanged), and rung b supports
(R1165/R1166). Accounting, with A0 ~ kappa x D minus the OCP's 0.1 s reference lead (the `traj_t`
pre-advance):

| case | D | A0 measured | A0 from 2.3 D - 0.09 |
|---|---|---|---|
| twin base | 0.098 | 0.131-0.144 | 0.135 |
| twin + 0.035 (H1) | 0.135 | 0.215-0.226 | 0.22 |
| twin rung b | 0.042 | 0.007-0.015 | 0.007 |
| rig r206e60 (orbit) | 0.14 | 0.199 | 0.23 |
| rig r2008e60 (figure-8 0.25 m/s) | 0.131 | 0.33 | 0.21 |

**Next, in order:**
1. **Rung b on the rig:** a figure-8 A/B with `params_file:=.../configs/rig/ref_time_shift.yaml`, base
   first, same pack. It is an existing switch; the figure-8 twin check (week4 code) is queued on the
   lab tonight.
2. **Rung c (daytime, critic first):** predict x0 by the expected application delay, in the
   plan-delta form x0 + (X_prev(t_a + tau) - X_prev(t_a)), skipped in creep/hold/LAND-down/after a
   resize or reseed. tau comes from the rung-a logs: pose age plus tick-to-arrival, ~0.1 s on the rig,
   ~0.06 s in the twin. Removing the 0.1 s pre-advance is a separate later variable: with tau ~ 0.1 s
   on the rig the pre-advance already matches it.
3. **Rung d** (stamped wire layout) only if the rung-c logs show the arrival jitter matters (lock:
   your word).

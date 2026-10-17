# Card: the planner's clock lead set from its measured delay (corner cutting, the build after G2)

Written Sat 17 Oct 2026 (Wesley's decision ~10:30, docs/decisions.md 17 Oct (3)). Bars frozen before any run.
Parent card: `docs/experiments/2026-10-16_corner_cutting.md`. The ruler is Wesley's 16 Oct one: clock-free cut and
across-path error, with lag in seconds kept separate.

## Why

At 0.3 m/s the ring cuts corners because of a timing mismatch. The combination below removes the cut on every shape
in the twin:
- `ref_time_shift`: the trackers fly the plan advanced by the time since it arrived;
- `plan_sqp_iters` 2;
- `plan_ref_lag_s` 0.075.

| shape (0.3 m/s) | base: cut / lag | 0.075: cut / lag | runs (base; 0.075) |
|---|---|---|---|
| orbit | +13.4 mm / +0.049 s | -1.7 mm / -0.003 s | R1991, R1992, R1996; R1993, R1997 |
| circle | +10.8 / +0.052 | -0.8 / +0.002 | R1989, R2002; R2003, R2005 |
| figure-8 | +24.3 / +0.039 | +3.5 / +0.028 | 6 runs; R2001, R2004, R2018, R2021 |

**The lead is a measured delay, not a tuning constant.**
- On the orbit the lag crosses zero at plan_ref_lag_s ~0.078 (G2, R1992-R1999). Pure delay predicts 0.1 - D, with
  D = 0.025 s the plan age left after the shift.
- That leftover has two known sources:
  - **the planner's own measure-to-publish time.** The planner reads the state at the start of `_solve_and_publish`
    and publishes after the solve. `ref_time_shift` counts from the plan's arrival
    (`tracker_node._ref_arrival_sim`), so the solve time is never removed. With 2 iterations, solve p50 is 24-25 ms.
  - **the pose's age when the planner reads it:** mocap, relay and network. It is ~0 in the laptop twin and larger on
    the rig.
- A fixed 0.075 is right for this laptop only. The rig's plan age was ~1.3x the twin's (r305, 8 Oct), and its
  solve time differs.

## The change

Switch `plan_ref_lag_s: meas` (the parameter takes a number today; `meas` is new; '' and numbers unchanged).

**Lead measured each tick:**
- tick start t0 = sim/ROS time at the start of `_solve_and_publish`;
- publish time t1 = the time the horizon goes out;
- pose age a = t0 - the header stamp of the payload pose used. Stamps not later than t0, and age clipped to
  [0, 0.1] s (a mocap stamped at receipt reads ~0);
- D_meas = EMA over ticks (time constant 2 s) of (t1 - t0) + a;
- lag used = clip(0.1 - D_meas, 0.0, 0.1), slewed at most 0.005 s per tick.

**Refused unless the trackers run `ref_time_shift`** (a launch-level check: `meas` without it is an error at
launch). Without the shift, the arrival-to-flight age is not removed and the formula is wrong.

**Logged:** `lead_meas_s`, `solve_pub_s` and `pose_age_s` in the planner's log.csv, so the rig flight shows what it
measured.

**Phases:** as plan_ref_lag_s today, the lag only enters the path reference (`reference_builder`:
t_k = traj_t + dt k - ref_lag_s, only while traj_t > 0). Lift, creep and hover are untouched.

**Code:**
- `planner_node.py`: the parameter, the EMA, the stamp of the payload pose, the clip and slew;
- `bringup` passes the string (`real_mode.PLANNER_OPTIONAL` kind changes from float to str-or-float).

Not changed:
- the trackers;
- the OCP;
- `planner_ocp.py` / `load_cable_dynamics.py` (no solver rebuild);
- every default.

## Tests (unit, before any twin run)

- off ('' or a number): `refs.update` gets exactly today's traj_t argument (bit-identical);
- `meas` with a constant 24 ms solve and 0 pose age: lag settles to 0.076 within 3 time constants, and slews at most
  0.005 per tick;
- a pose stamped in the future, or 0.5 s old: age clipped;
- `meas` without ref_time_shift: launch refuses with a message naming both switches.

## Twin runs (laptop, one at a time; same-session base)

Each arm is ref_time_shift + plan_sqp_iters 2 + one lead source. One variable against the fixed-0.075 arm: the lead
source.

| cell | arm | n |
|---|---|---|
| orbit 0.3 | base (defaults) / fixed 0.075 / meas | 2 / 2 / 3 |
| circle 0.3 | base / fixed 0.075 / meas | 2 / 2 / 3 |
| figure-8 0.3 | base / fixed 0.075 / meas | 2 / 2 / 3 |
| **rig-latency twin**, orbit 0.3 (SIM_MOCAP_DELAY_S calibrated to D ~0.12, SIM_MOCAP_VEL_FILTER ema) | base / fixed 0.075 / meas | 2 / 2 / 3 |
| orbit 0.125 (the rig's speed) | base / meas | 2 / 2 |

**Neighbours, once each with `meas`:**
- hover (the switch must be neutral);
- a floor-start lift and LAND;
- a detach mid-path;
- spin 0.3;
- line_x 0.3;
- an attach on the orbit.

## Bars (frozen now)

| # | bar | pass | fail |
|---|---|---|---|
| L1 | the lead finds the delay | laptop orbit/circle: \|lag\| <= 0.015 s per run; D_meas within 0.01 s of 0.1 minus the G2 crossing (0.078: D 0.022) | outside |
| L2 | it adapts where a fixed value cannot | rig-latency twin orbit: meas \|lag\| <= 0.015 s per run, and fixed 0.075 \|lag\| larger than meas's in both its runs | meas outside, or fixed 0.075 already centred (then the switch adds nothing) |
| L3 | the cut stays removed | laptop: meas cut at the tight bends within 3 mm of the fixed-0.075 arm's mean (orbit, circle) and at most +8 mm on the figure-8 (base +20..+33) | outside |
| L4 | no cost elsewhere | ring tilt <= base + 1 deg; drone tilt and throttle peaks <= base + 10 %; solve p95 <= base + 5 ms; neighbours: no fault the base did not have, hover lag/RMSE within the base's range | any |

**Falsifier:**
- L1 or L2 fails: the measured delay is not the delay the lead must cancel, so a fixed value is as good and the
  switch is not kept;
- or L3/L4 fails.

**What it cannot say:**
- the twin has no downwash and rigid rods;
- the rig's pose age is invisible until the stamps item (rig track item 1) is in;
- `pose_age_s` reads ~0 on the rig before that, so the lead there counts the solve time only.

## Rig

Reopens the 8 Oct decision "lag: report it, do not fix it" (decisions.md:112) for the rig: Wesley's word first.
- On the rig it goes in through a `configs/rig/` params file, in carry only, after ref_time_shift's rig must-fixes:
  `ref_shift_cap_s`, no shift while the ring is grounded.
- The partner's ring prediction (`ring_bridge` predicts from traj_t with no lag) is checked before any timing default.

## Critic

Critic, 17 Oct ~10:55. Verdict: **rewrite the card.** No code was written; the card waits for Wesley (see the note at
the end).

**The five questions:**
1. **Tried before:** the 8 Oct time cascade (`docs/experiments/2026-10-08_time_cascade.md`).
   - Rung b = ref_time_shift (R1165/R1166).
   - H1 = extra latency (R1172/R1173).
   - That critic already flagged that `last_solve_ms` is wall time and that mocap stamps are receipt times.
2. **Simplest explanation not addressed:** the lead the ring needs is not the delay itself but about kappa x delay.
   - H1 at 0.125 m/s: kappa 2.3-2.4.
   - Rung b at 0.125 m/s centred the ring with the full 0.1 s lead kept.
   - G2 at 0.3 m/s implies kappa ~0.9.
   - The "D 0.025 = solve 25 ms" match crosses clocks: solve_ms is wall time (`planner_solver.py:271,324`), and at
     RTF 0.44 it is ~0.011 s of sim time.
3. **The bars test the instrument, not the hypothesis:**
   - with the single-threaded executor (`planner_node.py:1990`), sim-time t1 - t0 reads ~0 (to be checked with one
     logged tick);
   - the rig-latency twin restamps each pose on release (`payload_mocap_emulator.py:254`), so pose age reads ~0 and
     L2 can only fail;
   - the 0.125 m/s cell has no bar.
4. **What it breaks on the rig:**
   - the stamps are receipt times, and the 8 Oct accounting puts the rig's post-shift plan age at ~0.1 s, which means
     a lag ~0 (as `configs/rig/ref_time_shift.yaml`);
   - `meas` would read 0.04-0.06 and move the lead the wrong way;
   - the ring_bridge prediction is unchecked.
5. **Asked for:** yes, 17 Oct (3). On 7 Oct Wesley said "I dont want a bandaid fix like just sending the reference
   earlier". A lead that does not match the plant's delay is closer to that than rung c or a stamped shift.

**Must-fix:**
1. Measure on the right clock first: compute the G2 runs' solve-to-arrival delay in sim time (R1992-R1999) and show
   whether it plus pose age sums to D 0.025.
2. Name the clock of t1 - t0, with a test. Sim time reads ~0; wall time over-predicts by 1/RTF in the twin and is
   right only on the rig.
3. The rig-latency twin must be able to show pose age (keep the Gazebo stamp behind a switch), or drop L2.
4. Test lead = delay against lead = kappa x delay before building: fixed-lead sweeps (0, 0.04, 0.075) on the delayed
   twin and at 0.125 m/s.
5. Bar the 0.125 cell, with fixed 0.075 and fixed 0 arms there.
6. Vary the delay with existing switches first: the G2 orbit sweep at plan_sqp_iters 5, or on the lab host. If the
   crossing does not move by the change in sim-time solve, the decomposition is falsified with no code.
7. Compare against the simpler, stronger fix and name it as the alternative:
   - the trackers key their shift on the planner's t0 carried in the message, not on arrival;
   - one float appended after the nodes; old trackers still parse it (check `runner_node`, `bench_node`, `ref_dump`);
   - this leaves exactly 0.1 to cancel, with no EMA;
   - the wire format was called locked on 8 Oct, so this needs Wesley's word.

**Should-fix:**
1. The off test must compare yref_at / cable refs / q_ref_at, not traj_t.
2. Use a separate `plan_ref_lag_mode`, not a str-or-float parameter (`lag_metrics.py:80` floats it).
3. Push the lead into the current builder every tick, including after `resize_fleet`.
4. Exclude reseeds, fallbacks and 'none' ticks from the EMA; warm it from the first planner solve.
5. Freeze the lead while traj_t is frozen (the reconfig hold, traj_hold_s).
6. The ref_time_shift check must read resolved params files.
7. Store the pose stamp in a new field and keep `_load_t`.
8. A neighbour where the solve time changes in flight (after a detach).
9. Score ring_bridge's prediction error on the attach-on-orbit neighbour.

**Status (17 Oct):**
- The decomposition this card rests on (solve-to-publish + pose age) is not shown.
- Must-fix 1 (offline, from the G2 logs) is next and needs no code.
- The choice between this card, the stamped shift (must-fix 7: a wire-format change) and rung c goes back to Wesley
  with that result.

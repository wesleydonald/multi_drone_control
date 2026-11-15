# C++ exit: revised plan (v2, 15 Nov 2026), replying to Te-Jen and Codex's "decisions and requested plan revision"

**Plan only.** No code, no configuration change, no runs. Numbers are the lab chat's exploratory readings of the 13 Nov
logs and code (not reviewed, not independently reproduced). Te-Jen's decisions are taken as given:
- **Height:** 3.2 m is the general body-height limit; fly the lowest workable exit height, not 3.2 m.
- **Swing:** 5 deg is the exit's trial swing assumption.
- **No separate no-overflight rule.** Collision checks stay.
- **Landing:** one fixed, reserved landing point.
- **Handoff timeout:** 7 s; on a timeout, report it, with no silent Python substitute.
- **Flights:** one flight, then analysis, then a targeted retry or retirement.
- **Opt-in:** the behaviour is opt-in and the frozen paper configuration is untouched.

## 1. Continuity from the live reference to the prefix (no extra readiness gate)
The live CLEAR_DROP_ZONE reference r(t) is generated from the ring's attachment pose and twist plus the clearance offset
c(t): a fixed offset plus the climb at `drop_clear_speed`. At the latch time t_L the prefix is built as:

    P(t) = R(t) + c(t) + b(t),   t in [t_L, t_L + H]

- R(t) is the ring's committed future (the CommittedTrajectory `ring_bridge` already publishes); c(t) is the same
  clearance profile the live generator uses.
- b(t) is a quintic blend. At t_L its value, first and second derivatives equal r - (R + c) in position, velocity and
  acceleration. At t_L + T_b it reaches 0 with zero derivatives.
- So P is C2 with the executed reference at t_L, exactly, and is purely ring-following after T_b. The draft's
  "translate the ring spline" matched position only.
- The ~0.1 m/s mismatch (planned against measured ring velocity) is no longer a step. It is spread over T_b as
  added acceleration of about 1.9 * dv / T_b, i.e. ~0.19 m/s^2 for 0.1 m/s with T_b = 1 s. The replay reports its peak.
- P is sent as a cubic B-spline on the existing knot spacing. Its first three control points are fixed by the existing
  boundary equations from P's p/v/a at the start of its coverage. The interior is fitted to P, and the fit error is
  reported.
- Executing P from t_L means Python publishes P's samples: one owner. C++ joins at its future splice time on P;
  continuity there is checked by the planner's own C2 splice check.
- Readiness stays the agreed relative-motion test (QUAD4 against the reference). The blend removes the need for any
  extra gate on the reference itself.

## 2. Coverage, timers, rejected attempts and cancellation
- **Horizon.** H = 5 s. While preparing (no authority yet), Python **refreshes** the prefix every T_r = 1 s. A refresh
  starts at t_R from the executing prefix's own p/v/a, with the same blend, so a refresh never steps. The backend binds
  each transaction to its newest prefix sequence.
- **No execution past coverage.** A grant needs splice time >= now + lookahead and coverage end >= splice + 2 s.
  Python never executes P past its coverage end:
  - if a refresh cannot be built (the ring commitment older than its freshness limit, e.g. 0.5 s), Python blends back
    to the live ring-following reference, with the same quintic from P's p/v/a, before coverage ends.
- **The 7 s handoff timer** starts at the first latch t_L0, the start of preparation for this exit. Refreshes and
  rejected or late grant attempts do not reset it. Only a new exit transaction does, and that needs a cancel back to
  live and a fresh readiness.
- **During a rejected or late attempt**, the executing reference is the current (refreshed) prefix. It is
  ring-following, never world-fixed.
- **On the 7 s timeout:**
  - report "exit handoff failed" in the status and the log;
  - cancel the transaction (existing recovery cancel);
  - blend back to the live ring-following reference;
  - no Python return and no world-fixed stop.
  - QUAD4 keeps following the ring at its clearance height until the operator acts (LAND, section 5).
- **Every cancel** back to live, and every manual Python takeover, starts from the executing reference's p/v/a with the
  same quintic. So there is no step in position, velocity or acceleration.

## 3. Exit-only context bound to the transaction
- The exit uses the existing recovery protocol with one new action, `<id> seedtraj exit <sequence> <stamp_ns>`. It is
  sent only from the exit branch (RETURN_TO_START preparation).
- In the backend, `exit_context_active` is true only while a `seedtraj exit` transaction is bound and authorized, from
  its grant through RETURN_TO_START until AT_START, cancel or LAND. Only then:
  - the payload-attached checks are skipped;
  - the exit workspace clip and the 5 deg exit swing geometry apply.
- `static_target` alone authorizes none of this. `require_payload_attached_for_enable` stays true at launch.
- The loaded leg's geometry (3 deg bound) and checks are unchanged. The swing geometry switches only inside the exit
  context, using the existing geometry rebuild.
- No second mode framework: the transaction is the context.

## 4. Resolved geometry: 5 deg and the 3.2 m general limit
**Limits are on the body origin:**
- the 3.2 m general limit;
- the fleet's +-1.8 m position limit (motion capture of the body).

**Full-assembly margins** (QUAD4 body box 0.105 m, tracking 0.05 m, cable and magnet swing 1.034 * sin 5 deg = 0.09 m)
belong to his collision model, not to these limits.

| Item | Value | How it fits |
|---|---|---|
| Exit hold (the ring-following climb end), body z | **2.9 m** | The lowest height at which the candidate route was free in all 8 drops at 5 deg (2.8 m left one drop, fm9hc, inside his model) |
| Launch "clear of the drop" check | 2.9 + 0.20 overshoot = 3.10 m | Under 3.2. The arithmetic allows up to a 3.0 m body hold |
| Exit search box, body z | floor 2.20, **top 3.2** | Inside the exit context only |
| Exit workspace, body xy | **+-1.65 m** | 1.8 minus tracking 0.05 minus margin 0.10 |
| Exit goal | the reserved landing point's xy at the start-hover height (~2.5 m) = AT_START | Section 5 |

- The 5 deg trial is a modelling choice; it does not contain all observed swing (4.7-8.5 deg maxima on 13 Nov).
- His model frees a pose above a carrier body only at a body z of 3.34-3.42 m. Within a 3.2 m top a route over a
  carrier is therefore excluded by his collision checks alone, not by an added rule.
- **Candidate route vs demonstrated route.** The candidate route below was geometrically free in his model on the
  recorded carriers, open loop:
  - hold relative to the ring at 2.9 m;
  - leave relative to the ring through plate 9's gap to 1.0 m from the centre;
  - fly straight to the goal.

  Whether his real-time search finds a free route, this one or another, within these bounds has **not** been shown.
  The replay (section 7) must show it.
- **Conflict to settle.** The frozen paper line's search top `cpp_max_z` is 3.3 m, above the new 3.2 m general limit.
  The loaded approach measured 3.04-3.08 m on 13 Nov. See choice 1 in section 8.

## 5. Landing point and manual override
- **The reserved landing point** is QUAD4's mat on Tuesday's line, **(0.00, +1.55)**, which is also its takeoff xy
  (the start hover's xy).
- From the 13 Nov logs, the carriers came to rest at least **0.95 m** from it, horizontally (fm9hc_11 closest; the rest
  1.14-2.83 m). The ring rested within (-0.31..0.39, -0.64..0.34), at least 1.2 m from it.
- During flight, its descent column kept at least 0.80 m from a carrier body and 0.93 m from a cable.
- On 13 Nov QUAD4 came to rest at its own start xy, (-0.2..0.4, 0.92..1.44).
- **AT_START -> LANDING** (opt-in parameter):
  1. AT_START's capture is held for a dwell T_d (2 s);
  2. the existing LANDING runs: xy held, straight down at 0.25 m/s, DISARM at about 0.27 m, LANDED_DISARMED; about 9 s
     from 2.5 m.
- **Ordering on the sheet:** QUAD4 lands first. Wesley's fleet LAND comes after QUAD4 reads LANDED_DISARMED, except in
  an emergency.
- **How our fleet LAND treats QUAD4** (`_take_back`, dissipative_node.py ~3091):
  - Already on the floor (below its rest height + the lifted margin): "it lands where it is". No reference and no
    motion, so a LANDED_DISARMED QUAD4 is left alone.
  - Still in his LANDING: our tracker takes it mid-air through the radio switch (one owner), brakes from its measured
    state and lands it alone. This is existing behaviour, and the sheet ordering avoids it.
- **His manual LAND during the exit.** Today, `external_landing_callback` cancels the recovery and enters LANDING:
  straight down at the current xy, which beside the ring is not acceptable. In the exit context it changes to, in the
  **same control tick**, before any Python command:
  1. revoke C++ authority (authority false published; transaction cancelled; the selector set to Python), atomically;
  2. Python starts from the executing reference's p/v/a, with the quintic into the agreed Python escape/return-and-land
     path:
     - before the grant: the prefix or live reference;
     - after the grant: the C++ committed trajectory sampled at now;
  3. the agreed path is today's direct return (`RET=direct`) to AT_START, then LANDING.
- **Known risk of that backup, for its separate review:** on fm9h8_13 it passed 0.5 m above QUAD1, inside his model
  for 0.4-6.6 s after the release. No change to it is proposed here.
- Immediate DISARM (SPACE) and all existing manual authority are unchanged.
- There is no low-speed takeover trigger. Ordinary slow motion is not a failed handoff.

## 6. Amended file-level plan (every change opt-in, default off; planner algorithms and MPC tuning unchanged)
| File | Change |
|---|---|
| `tejen_dynamic_planner/include/.../reference_recovery.hpp` | The action `seedtraj exit <seq> <stamp>`; the transaction's prefix sequence fixed per transaction; grant with either seed kind; `exit_context` flag on the transaction |
| `tejen_dynamic_planner/src/transfer_backend_node.cpp` | `moving_prefix_topic` ('' = off).<br>Under a bound exit transaction:<br>- seed from the prefix (`initializeCommittedTrajectory`), never `makeStationaryHoverTrajectory` or `shiftedToStartTime`;<br>- certify the prefix over [now, now + splice];<br>- refuse a prefix with coverage end < splice + 2 s;<br>- relative activation readiness against prefix(now);<br>- search-box origin at prefix(now + splice).<br>`exit_context_active` (section 3) gates the payload checks, the exit box top 3.2 and workspace +-1.65, and the 5 deg exit geometry.<br>Status: `seed_kind`, `splice_time_ns`, `exit_context` |
| `tejen_mission/c1f2_handoff.py` | `CommittedSpline` (p/v/a from the message); `build_exit_prefix(r_pva, ring_spline, clearance, t_L, H, T_b)` with the quintic blend and the boundary-fixed first three control points; `blend_to(target_fn, from_pva, T_b)` for every cancel and takeover; `relative_grant_state`; `ReferenceRecovery.begin_exit_prefix` (reset -> seedtraj exit -> grant; same-time accept) |
| `tejen_mission/online_join_planner.py` | Parameters (all off by default):<br>- `c1f2_exit_moving_handoff`;<br>- `exit_prefix_horizon_s` 5, `exit_prefix_refresh_s` 1, `exit_blend_s` 1;<br>- `exit_handoff_timeout_s` 7;<br>- `exit_hold_z_m` 2.9;<br>- `at_start_land_after_s` 2.<br>`c1f2_moving_exit_handoff()`: no world-fixed latch; readiness; latch, refresh and timer per section 2; the timeout report and cancel to live.<br>Selector branches: no early recovery begin; publish P after the latch; continuity at every source change.<br>`external_landing_callback`: the exit-context branch of section 5.<br>AT_START -> LANDING after the dwell.<br>CSV: source, latch/refresh/grant/splice times, blend peak accel, join and cancel steps |
| `tejen_mission/launch/partner_mission.launch.py` | `exit_handoff:=moving` (typed only; requires `return_after_drop:=cpp`); the exit climb typed so the hold is 2.9 m and the "clear of the drop" check uses the 3.2 m limit; keep `require_payload_attached_for_enable` true; pass `moving_prefix_topic` |
| `CHANGES_FROM_TEJEN.md` | One entry per change |
| Tests | Recovery-contract parse and context; the prefix-seed test (the committed path equals P before the splice; p/v/a match at the splice to 1e-7); C2 continuity of P at t_L and at each refresh; blend-to-live and takeover continuity; the timer not reset by refreshes or rejections; the exit context never active on the loaded leg; LAND in the exit context revokes authority before the first Python sample |

## 7. Proposed replay and twin checks (after implementation is authorized)
**Replay (laptop):** his real backend binary on the 13 Nov bags; the driver runs the real recovery topics, with QUAD4 =
the selected reference + the recorded tracking error. fm9h_2 (hover) first, then the circles and figure-eights, each idle
and under 8-core load.
- **Continuity.** p/v/a steps at the latch, each refresh, the grant, the splice and any cancel, against the recorded
  tick-to-tick steps; the blend's peak acceleration; P's spline fit error.
- **Authority.** live -> P -> C++ in order, one source per tick, one publisher, one acknowledgement; zero stationary
  seeds; the exit context only during the exit.
- **Timers and coverage.**
  - Injected: a rejected grant; a stale ring commitment (a refresh failure, so a blend to live); the 7 s timeout (the
    report, then a cancel to live).
  - Coverage never exceeded.
- **Manual LAND** at three moments (preparing, just after the grant, mid-exit): authority revoked in the same tick;
  continuity into the direct return; then LANDING.
- **Geometry.**
  - Does his real-time search find a free route within z <= 3.2 and |x|, |y| <= 1.65? Success rate and time to the
    first plan.
  - The route stays outside his model, and the body never exceeds 3.2 m.
- **AT_START -> LANDING** fires after the dwell.

**Twin (headless, lab PC):** the wiring end to end on the frozen line plus the switch, hover first, then the circle
and figure-eight.
- The 7 s timeout and LAND (preparing and after the grant).
- The fleet LAND after QUAD4 is LANDED_DISARMED, and during its descent (the take-back).

The twin runs at 0.13-0.16 times real time, so it checks wiring, not timing.

**Rig:** one hover flight, then analysis, then a targeted retry or retirement.

## 8. Remaining human choices
1. **The 3.3 m search top on the frozen paper line** against the new 3.2 m general limit: keep the frozen line as
   flown (exit-only 3.2), or apply 3.2 to the approach too (changes the frozen configuration).
2. **The exit hold height:** 2.9 m from the exploratory geometry, or let the replay pick the lowest that works (>= 2.9).
3. **Timing values:** T_b 1 s, the refresh 1 s, H 5 s, the AT_START dwell 2 s.
4. **The sheet ordering:** QUAD4 lands before the fleet LAND.
5. **The direct-return backup's review** (fm9h8_13), now or separately.

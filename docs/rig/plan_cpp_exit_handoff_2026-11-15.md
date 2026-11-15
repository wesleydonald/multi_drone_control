# C++ exit after the drop: the moving-reference hand-off. Checks and a file-level plan (15 Nov 2026)

For Wesley and Te-Jen/Codex. This follows the design agreed on 15 Nov (decisions DECIDED 2026-11-15). **No code yet.**
All numbers come from the code and the 13 Nov rig logs. They are the lab chat's exploratory readings, not reviewed.
- The collision checks used Te-Jen's built libraries: the WorldSnapshot and Separator from
  build/tejen_dynamic_planner, with the rig's parameter dump.
- Tolerances were QUAD4 0.105/0.06 + 0.05, carriers 0.105/0.06 + 0.06, tether 0.015 and cable 1.034, with
  outer_octagon.
- They ran over the eight 13 Nov drops.

## A. The three checks

### A1. Does the released geometry admit an exit under the real limits? Yes, under three conditions.
1. **The exit cannot start where QUAD4 is today.**
   - At the release, QUAD4 is inside his inflated model in 8 of 8 drops. The contact is carrier 7's or 11's tether
     against QUAD4's cable or magnet; in 6 of 8 it is inside even with a 0 deg swing.
   - At the end of today's 3 cm climb (2.49-2.55 m) it is inside in 5-6 of 8. The rest clear by 0.00-0.01 m.
   - **So the ring-following climb must go higher before the hand-off.** It needs at least 2.8 m body height with the
     3 deg swing bound. Larger bounds need more (A1.3).
2. **The route must leave through the gap at plate 9.**
   - Tested: hold relative to the ring at height zh, move out relative to the ring along plate 9's direction to 1.0 m
     from the ring centre, then fly straight to the start hover. With zh 2.8 m and a 3 deg bound this is free in all
     8 drops and all 8 timing and speed variants.
   - Centre-line clearances on that route:
     - cable to cable >= 0.30 m;
     - body to body >= 0.42 m horizontally;
     - |x| <= 0.73 m, |y| <= 1.31 m.
   - A straight line from the hold to the start hits a tether within 0.1-2.7 s in most drops.
   - Whether his search finds the gap by itself in real time over a moving ring cannot be decided from the logs. The
     offline replay (section C) must show it.
3. **Heights.**
   - The launch arithmetic: the "clear of the drop" leg is 1.35 + climb + 1.034 + 0.20 (the overshoot allowance),
     checked against max_body_z 3.0.
   - That allows a climb of at most 0.416 m above the release, i.e. a body height of **2.80 m**: exactly what the 3 deg
     bound needs.
   - A swing bound that covers the measured swing needs more height: the exit's measured swing reached 4.7-8.5 deg
     (median 2.0-3.8 deg). 5 deg needs 2.9 m and 8.5 deg needs 2.99 m. Neither fits under 3.0 with the 0.20 allowance:
     **Wesley's word on max_body_z or the allowance.**
   - The scripted `RET=cpp` (`return_climb:=transit`, 3.19 m) cannot launch at all today.
   - His search box top (3.3 m) keeps him off the carriers. His model frees a pose over a carrier body only at 3.34-3.42
     m, so his C++ cannot plan over a carrier within the caps. (Our Python return did fly over one in fm9h8_13.)
   - **Nothing keeps a C++ exit inside +-1.8 m.**
     - His box reaches y 1.9-2.2 m (2.55 m with Tuesday's mat at y 1.55).
     - The workspace clip is off on the rig line.
     - Our position limit skips QUAD4 while his stream flies it.
   - So the exit needs his workspace clip applied for the exit only. A global clip left the approach with no plan
     (R4366-R4373).

### A2. Does the unloaded-return handling weaken the loaded approach? Today yes, in one place. Every change can be exit-only.
| What | Today | Fix (exit only) |
|---|---|---|
| `require_payload_attached_for_enable` | `RET=cpp` sets it false for the whole flight (launch ~1563): the approach then enables, takes authority and keeps replanning without a fresh "object attached", and is not disabled if the object reads detached | Keep it true at launch. In the backend, skip the four payload checks only while `static_target_` (the exit) is true |
| The hold-and-hand-over function | Shared with the loaded climb (online_join_planner.py ~8159) | The moving variant is called only from the exit branch (~8891) |
| Stationary activation | One global gate | Relative readiness only while the exit transaction is bound (B, backend) |
| Swing bound | One value for both legs | If raised, switch at the release (`objectAttachedCallback` already rebuilds the geometry) or on `static_target_` |
| Workspace clip | Off | On only while `static_target_` |
| His MPC's mass | object_mass 0 for both legs, thrust-ratio feedback | Unchanged |

### A3. Is the descent at the start clear, and how does AT_START land? Clear in all 8; it needs a small addition to land.
- **The column is clear.** From the start hover down to the mat, his margins hold in all 8 drops, at every phase of
  the carriers' path (0 of 8,832 samples inside his model), with swing bounds of 3 to 8.5 deg.
  - The thinnest are fm9h8_13 (0.02-0.04 m spare) and fm9hc_10 (0.05-0.06 m spare).
  - Tuesday's mat at (0.00, 1.55) keeps >= 0.80 m from a carrier body and >= 0.93 m from a cable.
  - Not in the logs: QUAD4's drift and swing during his descent from 2.5 m (never flown from AT_START).
- **How AT_START lands.** AT_START just holds (online_join_planner.py ~8910). His existing LANDING already does the
  right thing:
  - xy held; straight down at 0.25 m/s;
  - DISARM at about 0.27 m, ~9 s from 2.5 m;
  - it is entered from `/join_planner/land_now`.
- The addition: after a dwell at AT_START, switch to LANDING, behind a parameter (default off). No phase-graph change
  is needed.
- `fc_disarm_land` is off on the rig, so QUAD4's disarm does not land the fleet.
- To check in the twin: what our later fleet LAND's take-back does with a QUAD4 already in LANDING or LANDED_DISARMED.

## B. File-level plan (every change behind a switch, default off; planner algorithms and MPC tuning unchanged)
No new message type. `interfaces/msg/CommittedTrajectory` already carries a timed C2 cubic B-spline in absolute time,
and his backend already converts it (`trajectoryFromMessage`).
- His planner core already starts every candidate from the incumbent's p/v/a at the splice time
  (receding_horizon_planner.cpp ~303-343 -> `initialControlPointsFromState`).
- It already rejects a splice that is not C2 (`replaceSuffix`).
- The stationary assumption lives only in the backend node: activation, grant seeding, hold certification and the
  two speed gates.

| File | Change | Tests |
|---|---|---|
| `src/tejen_dynamic_planner/include/.../reference_recovery.hpp` | A new recovery action `seedtraj <sequence> <stamp_ns>` binding the transaction to a prefix; the grant accepts either seed kind | Extend `test_c1f4_execution_contract.cpp`: parse, malformed input, fixed within a transaction, grant, cancel |
| `src/tejen_dynamic_planner/src/transfer_backend_node.cpp` | Param `moving_prefix_topic` ('' = off) and its subscription (CommittedTrajectory; vehicle and frame checked).<br>Under a `seedtraj` transaction only:<br>- seed with `initializeCommittedTrajectory(prefix)`, never `makeStationaryHoverTrajectory`, at activation and at the grant;<br>- certify the prefix over [now, now + splice];<br>- refuse a prefix shorter than now + splice + 2 s;<br>- search-box origin at prefix(now + splice);<br>- `activationKinematicsReady`: measured p/v/a against the prefix, replacing the two stationary gates.<br>Exit-only (`static_target_`): the payload checks (A2) and the workspace clip (A1).<br>Status JSON gains `seed_kind` and `splice_time_ns` | New `test_moving_prefix_seed.cpp`: a curved, moving prefix in a free world.<br>- The committed path equals the prefix before the splice.<br>- p/v/a at the splice match to 1e-7; splice = now + lookahead.<br>A source-contract test: the prefix branch never calls `makeStationaryHoverTrajectory` or `shiftedToStartTime` |
| `src/tejen_mission/tejen_mission/c1f2_handoff.py` | `CommittedSpline` (from the message; p/v/a reusing the de Boor helpers already in the fake world); `exit_prefix_from_ring`; `relative_grant_state`; `ReferenceRecovery.begin_prefix` (reset -> seedtraj -> grant; accept compares the C++ window with the prefix at the same absolute times) | `test_c1f2_handoff.py`: spline against an analytic cubic; exact translation; relative-ready where the absolute check refuses; accepts C++ equal to the prefix, rejects a one-tick shift or a stationary seed |
| `src/tejen_mission/tejen_mission/online_join_planner.py` | Params: `c1f2_exit_moving_handoff`, prefix and ring-commitment topics, `exit_prefix_horizon_s` 5, `exit_handoff_timeout_s`, the exit climb height (A1), `at_start_land_after_s` (A3).<br>`c1f2_moving_exit_handoff()` beside the hold version: no world-fixed latch, so the ring-following clearance reference keeps flying.<br>Readiness is relative to it; at readiness, freeze and publish the prefix (the ring's committed future translated onto the live reference) and begin the prefix transaction.<br>Selector branches (~6013, ~6046, ~6096): no early recovery begin; after the freeze, publish prefix samples (never the stopping pad), so one owner and no step.<br>AT_START -> LANDING after the dwell.<br>CSV columns: source, freeze step, join error | `test_return_after_drop.py`:<br>- switch off = unchanged; no world-fixed latch; the reference keeps moving;<br>- relative readiness; after the freeze the prefix, never stationary;<br>- live -> prefix -> C++ steps within the hand-off tolerances; timeout -> cancel -> live;<br>- AT_START -> LANDING after the dwell |
| `src/tejen_mission/launch/partner_mission.launch.py` | `exit_handoff:=moving` (typed only; requires `return_after_drop:=cpp`).<br>The exit climb typed so the height check passes (<= 2.80 m under 3.0 unless Wesley changes the cap or allowance).<br>Keep `require_payload_attached_for_enable` true; pass `moving_prefix_topic` and the exit workspace | Launch tests: typed only, the refusals, the parameters reach the backend |
| `src/tejen_mission/CHANGES_FROM_TEJEN.md` | An entry for each change | - |

Thresholds: reuse the existing `c1f2_prepare_*`, `c1f2_handoff_*`, `activation_*` and `ego_tracking_half` values, read
as relative values under the switch. On 13 Nov even the hover showed WAITING_STATIONARY_ACTIVATION 1-3 times at the 0.10
m/s tolerance, so the values may need Wesley's word (C).

## C. Offline validation before the hover-first flight
- **Harness:** the replay harness `results/analysis/cpp_realtime_2026-11-13/full/` (the real backend binary, the 13 Nov
  bags). Changes:
  - a `run_exit.sh` starting at each drop's climb end;
  - the prefix topic in the backend parameters;
  - a `driver_exit.py` that runs reset -> seedtraj -> grant against the real backend over the real recovery topics,
    with QUAD4 = the selected reference + the recorded 13 Nov tracking error, so the relative gates are really tested.
- **Order:** fm9h_2 (hover) first; then fm9hc_10/11 (circle) and fm9h8_13/14 (figure-eight). Each idle and under 8-core
  load.
- **Continuity:**
  - the reference's p/v/a step at the freeze and at the switch, against the recorded tick-to-tick steps;
  - C++ minus the prefix before the joining time about 0 (absolute timing kept);
  - joining time after the switch, within the 0.30 s splice.
- **Authority:**
  - live -> prefix -> C++ in order, one source per tick, no gap over 1.5 control periods;
  - one publisher on QUAD4's reference;
  - one acknowledgement per transaction;
  - zero "seeded stationary hold" lines.
- **Geometry on the same replays:** the exit stays outside his model (A1), never over a carrier, inside +-1.8 m, and
  under the height cap.
- Then a headless twin run (working rules), then the one hover-first rig flight after Tuesday's nine. A failure means
  it is not used.

## D. For Wesley (and Te-Jen)
1. **Exit climb height.**
   - 2.80 m with the 3 deg bound fits today's 3.0 m cap.
   - A bound that covers the measured swing (5-8.5 deg) needs 2.9-2.99 m: raise max_body_z (3.0 -> ~3.2) or drop the
     0.20 m allowance for this leg.
2. **The freeze.** For about 0.3 s, from readiness to the switch, Python flies the ring's planned future plus the offset,
   not motion capture. This is inherent: C++ needs the reference at the joining time before it arrives. The step at the
   freeze is up to ~0.1 m/s; report it, don't gate on it.
3. **The relative gate values** (reuse the existing ones, or set them from the replay).
4. **The exit workspace clip** (+-1.8 m), exit only.
5. **AT_START -> LANDING**, automatic after a dwell (A3), default off.
6. The headless twin run before the rig: yes or no.

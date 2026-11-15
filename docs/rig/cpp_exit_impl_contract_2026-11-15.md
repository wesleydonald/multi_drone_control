# C++ exit: implementation contract between the backend and the mission (15 Nov 2026)

This is the shared interface for implementing v3 (`plan_cpp_exit_handoff_2026-11-15_v3.md`; v2 for details it does not
replace) under Te-Jen and Codex's implementation handoff (`tejen_2026-11-15_cpp_exit_implementation_handoff.md`). Both
halves build against this text. **Everything is opt-in. With the new parameters at their defaults, behaviour is
byte-for-byte today's.** Planner algorithms, MPC tuning and the frozen paper configuration are unchanged. No rig flight
is authorised.

## 1. Messages
**Prefix:** `interfaces/msg/CommittedTrajectory`, published by the mission on `exit_prefix_topic` (default
`/dynamic_planner/transfer_prefix`) and read by the backend on `moving_prefix_topic` ('' = off, the default).
- QoS: reliable, transient_local, depth 1.
- `header.stamp` is the publication time and `header.frame_id` the backend's planning frame (as the ring commitment).
- `vehicle_id` is the ego id the backend already uses.
- `sequence` starts at 1 per transaction and increases by 1 per refresh. `terminal_hold` is false.
- `pieces` is a C2 cubic B-spline in absolute time with the existing `valid_from_s`/`valid_until_s` semantics, so
  `trajectoryFromMessage()` accepts it unchanged.

**Recovery request** (the existing String topic): one new action, the others unchanged.

    <id> seedtraj exit <sequence> <stamp_ns> <deadline_ns>

- `stamp_ns` is the header stamp of that prefix sequence.
- `deadline_ns` is the transaction's fixed 7 s deadline (section 4), the same value on every refresh.
- A `grant <stamp_ns>` within a seedtraj transaction refers to the bound prefix; its stamp must be >= the bound
  prefix's stamp.

**Status JSON** (the existing topic), new keys:
- `seed_kind` ('stationary' | 'prefix'), `exit_context` (bool), `prefix_sequence` (int);
- `prefix_coverage_end_ns`, `deadline_ns`, `splice_time_ns` (diagnostic);
- `exit_reject_reason` ('' when none);
- `candidates_attempted`, `candidates_accepted`, `candidates_late`, `candidates_invalidated` (counters per transaction).

## 2. Backend (src/tejen_dynamic_planner; reference_recovery.hpp, transfer_backend_node.cpp, tests)
1. **Parse `seedtraj exit`.** The transaction context gains `seed_kind`, `exit_context`, the bound prefix (sequence,
   stamp, coverage end), `deadline_ns` and the protected-window end.
2. **Open the exit context on accepting `seedtraj exit` (before any activation or grant check, before certification).**
   - **Require** that the prefix with that sequence has been received and validated: vehicle, frame, C2,
     `valid_until` >= now + lookahead + 2 s.
   - **Require** a fresh `/tejen/object_attached` reading `false`, using the backend's existing freshness rule. Reuse the
     existing release evidence; add no new gate, dwell or manoeuvre.
   - **Then install** the exit context:
     - the unloaded geometry (payload off; swing = `exit_max_swing_angle_rad`);
     - the exit assembly workspace (`exit_workspace_min` / `exit_workspace_max`, erosion by the existing
       `assemblyCentreBounds`);
     - the exit search top (`exit_max_search_z_m`);
     - the payload-attached checks skipped **for this transaction only**.
   - On a failed requirement: no context; `exit_reject_reason` set (`prefix_missing`, `prefix_invalid`,
     `prefix_coverage_short`, `object_attached_or_stale`, `deadline_passed`).
3. **Seed from the prefix, never stationary.**
   - Activation and the grant call `initializeCommittedTrajectory(prefix)` with `shadow_commit_ = prefix`.
   - Never `makeStationaryHoverTrajectory` or `shiftedToStartTime` on this path.
   - Splice at now + lookahead **on the prefix** (absolute time kept).
   - Prefix certification over [now, now + lookahead] replaces the hold certification.
   - Search-box origin = prefix(now + lookahead).
4. **Relative readiness** (`activationKinematicsReady`) replaces both stationary gates for this transaction only: the
   measured p, v and a against prefix(now) within the existing `ego_tracking_half`, `activation_speed_tolerance_mps` and
   `activation_accel_tolerance_mps2`. Every other transaction keeps `require_stationary_activation` exactly.
5. **Refresh** (`seedtraj exit <seq+1> ...`, same id): bind the new sequence.
   - Candidates planned against the old sequence stay valid only if the new prefix evaluates identically
     (|dp|, |dv|, |da| <= 1e-9) at 21 times over [now, t_R + `exit_protected_s`] **and** their splice time lies in that
     window. Otherwise they are invalidated (an epoch bump; `candidates_invalidated`++) before rebinding.
   - The deadline never changes.
6. **Admission.** A candidate is installed only if its splice time lies inside the bound prefix's coverage and before
   `deadline_ns`. Otherwise it is counted late and dropped.
7. **Cleanup.** The exit context ends and the loaded geometry, bounds and checks are restored on any of:
   - Cancel;
   - a Reset with a new id;
   - `enable=false`;
   - backend disable;
   - `deadline_ns` passing without a grant.

   Outside an exit context, every existing check and behaviour applies unchanged.
8. **Parameters** (all default off or neutral): `moving_prefix_topic` (''), `exit_max_swing_angle_rad`,
   `exit_workspace_min` / `exit_workspace_max`, `exit_max_search_z_m`, `exit_protected_s` (1.5).

## 3. Mission (src/tejen_mission: online_join_planner.py, c1f2_handoff.py, launch/partner_mission.launch.py, tests)
**Parameters** (off by default): `c1f2_exit_moving_handoff` (False), `exit_prefix_topic`, `exit_prefix_horizon_s` 5,
`exit_prefix_refresh_s` 1, `exit_blend_s` 1, `exit_protected_s` 1.5, `exit_handoff_timeout_s` 7, `exit_hold_z_m` 2.9,
`at_start_land_after_s` (0 = off; 2 on the trial line).

1. **Timer first.** When the existing clearance stage (CLEAR_DROP_ZONE) completes with `return_after_drop == 'cpp'` and
   the switch on, record t0 = now and the deadline t0 + 7 s **before** any new check or fit. Fits, rejections and
   refreshes never reset it.
2. **No world-fixed latch.** The ring-following clearance reference keeps flying. Its climb target is `exit_hold_z_m`
   (body); the launch types the climb so the height check passes.
3. **The prefix.**
   - Fit a C2 cubic B-spline to P(t) = R(t) + c(t) + b(t) (v2 section 1: the ring's committed future, the clearance
     profile, the quintic blend).
   - The **first three control points are fixed by the existing boundary equations** from the executed reference's
     p/v/a at the prefix start time, which is the next publication time.
   - Check the fitted curve: p/v/a continuity at its start (1e-6) and |v|, |a|, |j| under the planner's limits over its
     span. On failure, lengthen the blend and refit, a bounded number of times. If still failing, no transaction this
     tick; log it.
4. **One curve.**
   - From the prefix start, the mission publishes QUAD4's reference by **evaluating the fitted coefficients** at the
     publication times. It never uses the analytic blend.
   - Refresh every `exit_prefix_refresh_s`: copy the bound prefix's pieces over [t_R, t_R + `exit_protected_s`]
     verbatim, append new fitted pieces C2 at the junction, check as above, then publish and send `seedtraj` with the
     next sequence.
   - Never evaluate past coverage end.
5. **Readiness and grant.** The existing relative-motion readiness against the executing reference: no zero-velocity
   requirement and no new dwell. Then grant. Accept the C++ window only if its samples equal the prefix's at the same
   absolute times up to the joining time; then the existing RETURN_TO_START authority switch.
6. **Timeout or failure.**
   - At the deadline without an accepted grant, or on any `exit_reject_reason`: report it, send Cancel and
     `enable=false`, and blend back to the live ring-following reference with a checked fitted cubic from the executed
     curve's p/v/a.
   - No Python return and no world-fixed stop.
7. **Manual LAND** in the exit (preparing or after the grant), in one callback:
   1. local selection first: clear authority flags, end the exit context locally, selector = Python, record the backend
      epoch/sequence to ignore stale results;
   2. Cancel and `enable=false`, with no acknowledgement wait;
   3. the escape starts from the **already published reference horizon evaluated at the actual switch/publication
      time**, never an older sample-0 relabelled as now;
   4. a checked fitted cubic into the existing direct return to AT_START, then LANDING.

   DISARM is unchanged.
8. **Failure branches.** If a refresh, cancel or manual blend cannot be built, or both the ring's committed future and
   the live ring evidence are unavailable: use the mission's **existing** manual and input-loss contracts. Name the
   code you reuse. No indefinite refitting, no executing an expired prefix, no blocked manual override. If no existing
   contract covers a branch, stop and return ONE concrete question instead of inventing a recovery.
9. **AT_START -> LANDING** after `at_start_land_after_s` (> 0) of held capture, through the existing LANDING
   (`transition_to`).
10. **Launch.**
    - `exit_handoff:=moving` (typed only; refused unless `return_after_drop:=cpp`) sets the mission switch and
      parameters and passes the backend `moving_prefix_topic`, `exit_max_swing_angle_rad` (5 deg),
      `exit_workspace_min` (-1.8, -1.8, 0.0) / `exit_workspace_max` (1.8, 1.8, 3.26) and `exit_max_search_z_m` (3.2).
    - `require_payload_attached_for_enable` stays true.
    - The new trial line's general body limit 3.2 m is typed through the existing `max_body_z` (3.2 allowed for this
      trial line), with the exit climb so the hold is 2.9 m.
    - The frozen paper line and its defaults are untouched.

## 4. Tests (each side its own; preserve every existing loaded-approach test)
- **Backend:**
  - parse and context (incl. malformed input; the deadline fixed);
  - prefix seed: the committed path equals the prefix before the splice, p/v/a at the splice to 1e-7, no
    stationary-seed call (a source contract);
  - relative readiness vs stationary for other transactions;
  - refresh: identical protected window keeps candidates; a differing one invalidates;
  - late and out-of-coverage candidates dropped;
  - the exit context opens before activation, requires a fresh detached reading, and restores on every cleanup path;
  - feature-off equivalence.
- **Mission:**
  - prefix fit: start p/v/a exact, junction C2, bounds;
  - published samples = fitted coefficients at absolute times;
  - the timer is not reset by fits, rejections or refreshes;
  - timeout and rejection blend to live without a step;
  - manual LAND between two publications (local selection first, Cancel and enable=false sent, the escape sample at
    the true switch time);
  - failed blend uses the existing contract;
  - AT_START -> LANDING;
  - feature-off equivalence;
  - the launch refusals and the parameters reaching the backend.

## 5. The final state after the three reviews (15 Nov 2026, 21:41)
Sections 1-4 hold as written, with these additions and settings. Feature-off behaviour, the frozen flight
configurations, manual authority (LAND, DISARM), the 7 s preparation budget, geometry and margins, timing settings and
MPC tuning are unchanged.

**Backend:**
- **Exit-transaction windows** carry their sample time as the header stamp. One window is published at the post-grant
  install.
- **Diagnostic logging** (logging only, marked DIAG):
  - the failing check inside `refreshCooperativeWorld()` / `refreshSharedCooperativeObstacle()` and the caught
    exception;
  - the ring pose's header-stamp age and receive gaps;
  - the replan-timer gaps;
  - one `DISABLE_DIAG` error line at each cooperative disable.

**Mission:**
- **Time tolerance.** The exit's freshness and join tests use the planner's 1 us time tolerance.
- **Join deadline.** If granted but no window joined by the join + 0.1 s: a prompt cancel (section 3.6's failure).
- **Source time.** During the granted C++ return (`exit_source_time()`), windows are advanced from their sample-time
  stamp, not their receipt. This covers the accepted window's time base, the selection's elapsed time and a
  source-age freshness check.
- **Arrival.** At RETURN_TO_START -> AT_START after a granted exit, the existing transfer generator is seeded from the
  executed reference at that tick (p, v; `exit_seed_transfer`), and an 'arrival' blend takes up a. The capture and the
  2 s dwell stay measured.
- **Escape.** The LAND escape's direct return is seeded the same way.
- **Failed blends** fall back as follows:
  - escape and arrival: the seeded transfer (p and v continuous, a may step);
  - cancel: the live ring-following reference as it is (may step).

**Launch:**
- **Height.** On the `exit_handoff:=moving` line, `maximum_search_z_m` (the loaded search top) is at most `max_body_z`:
  `cpp_max_z` above it is refused, and untyped it is `max_body_z`. Also `exit_max_search_z_m` = min(3.2, `max_body_z`).
  - The trial line resolves to 3.2 / 3.2.
  - The exit's binding body-origin top is 3.150 m: the exit workspace's 3.26 m through `assemblyCentreBounds`.
- **Trial-line backend values.** The trial line keeps them as the rig script types them for every CPP=on line:
  `cpp_world_refresh` 0.2, `cpp_splice` 0.30, tracking 0.06, settle 0.20 / 0.8. A twin check line must type them too
  (R4535 without them, R4536 with them).

**Twin harness only:**
- **Supervisor status override.** `supervisor_status_timeout_s` (typed per run; the rig refuses it) sets his twin
  supervisor's `planner_status_timeout_s`.
- **Return-line success.** On `return_after_drop` twin lines his supervisor's success is `LANDED_DISARMED` after
  `AT_START` (`success_after_phase`, default off). A landing before `AT_START`, or a disarm in flight, still fails.
- **Runner** (ours): a SIGNAL event (`STOP|CONT <executable>`), and WAIT_LOG on his planner's "C++ exit" lines as
  `HIS:...`.

**Outside this interface** (ours): ring_bridge keeps one `LoadTrajectory` per trajectory. The commitment refresh no
longer rebuilds the figure-eight's arc-length table at each control point, and the output is identical.

**Open:** what the mission does after the switch if a genuine input loss disables the backend. That is Te-Jen's
decision; today QUAD4 parks until the operator's LAND.

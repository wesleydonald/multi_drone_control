# C++ exit: the review round, evidence v2 (15 Nov 2026, evening), for Te-Jen and Codex

This answers `tejen_2026-11-15_cpp_exit_evidence_review.md`. The previous report is `cpp_exit_evidence_2026-11-15.md`.
Nothing is committed and no rig flight is proposed.
- **Replay** is open loop on the 13 Nov recordings and is not flight proof.
- **The twin** runs at 0.13-0.16 times real time. It checks wiring, not deadlines.

Feature correctness and mission completion are reported separately throughout.

## 1. Code
**Checkpoints** in `~/mdc_checkpoints/`:
- `2026-11-15_cpp_exit_review_base/` is the reviewed implementation before any change. Its backend patch has sha1
  ee03e8588e30. Its mission patch has sha1 015901a21e30: the reviewed 3c9f996bf21d plus one launch argument for the
  twin, since removed.
- `2026-11-15_cpp_exit_corrections/` is the corrected tree:
  - `01_backend_tracked.patch` ee03e8588e30 (the backend is unchanged in this round);
  - `02_mission_tracked.patch` 0a3f85be658f;
  - `03_runner.patch` 6ed02e9a26fa (our test runner only).

The baseline is HEAD 8f0afc79. The changes are logged in `src/tejen_mission/CHANGES_FROM_TEJEN.md`, entry "15 Nov 2026",
in the rows stamped 19:30.

**Changes, all scoped to the exit unless stated:**
1. **Source-time windows.** During the granted C++ return (`exit_source_time()`), the selection advances each window from
   its sample-time stamp, not from its receipt. This applies to the accepted window's time base, the fresh candidate's
   elapsed time and a source-age freshness check (1 us tolerance). The loaded line and every other transaction are
   unchanged.
2. **C++ -> AT_START.** At the switch after a granted exit, the existing transfer generator is seeded from the
   outgoing commanded reference evaluated at that tick (p, v; `exit_seed_transfer`). The existing checked exit blend
   ('arrival') takes up a. Unchanged: the measured capture, the 2 s dwell and LAND. There is no new phase or manoeuvre.
3. **Failed blends.** The manual escape's direct return is seeded the same way, at the first escape publication.
   - **Escape and arrival blends:** if one fails its checks, the fallback is the seeded transfer. Position and velocity
     stay continuous; only a steps, to the transfer's own start.
   - **A cancel blend (into the live ring-following reference):** a failed one still flies that reference as it is.
     No existing branch makes it continuous. This is reported in section 6 and not changed.
4. **Height.** On the `exit_handoff:=moving` line, the loaded search top `maximum_search_z_m` is bounded by the line's
   general body limit `max_body_z` (3.2 m on the trial):
   - a typed `cpp_max_z` above it is refused (the trial line typed 3.3);
   - untyped, the loaded search top is the limit;
   - `exit_max_search_z_m` = min(3.2, the limit).

   There is no runtime geofence and no exit XY bounds on the loaded approach. Frozen lines are unchanged.

## 2. The post-hand-over disable (fm9h8_14_e_own_load8): the likely cause, not yet proven at runtime
**Why `refreshCooperativeWorld()`:** the disable's empty blocker string excludes every peer-input check and the
basket check. That leaves `refreshCooperativeWorld()` → `refreshSharedCooperativeObstacle(i, now, true)` →
`cooperativeAttachedTetherGeometry` throwing "attached peer requires fresh measured ring pose". That means a ring pose
older than `cooperative_ring_pose_timeout_s`, which is 0.5 s (the default; the trial line does not type it).

**Where a stale pose comes from: ring_bridge.**
- **The mechanism.** It rebuilds the shared commitments synchronously, once a second, on its single executor
  (`fake_cooperative_transport_world.py` ~633-649). That blocks its 20 Hz ring-pose and carrier-state publishers.
- **Measured in the replay,** under 8 busy cores:
  - every refresh stalls the ring pose for 0.23-0.33 s (0.18-0.20 s idle);
  - all 124 of 124 gaps over 0.15 s fell within 0.05 s of a refresh;
  - the header-stamp gaps equal the receive gaps, so the stall is inside ring_bridge, not in transport;
  - the 13 Nov bag's mocap never gapped more than 0.025 s.
- **Reproduction.** 0 of 5 loaded re-runs of the retained failure reproduced the disable. It needs a stall over 0.5 s.
  - Files: `results/analysis/cpp_exit_replay_2026-11-15/runs_disable/` (attempts 1-5, idle1, `feeds_summary.txt`).
  - Code: these ran on the reviewed mission, snapshot `runs_disable/mission_snapshot/`.
- **On the rig.** ring_bridge is the rig's `fake_cooperative_transport_world` too, with the same synchronous refresh.
  - RIG-1113-fm9h8_3 disabled in the LOADED transit with every carrier state stale. The bridge states came 290-395 ms
    apart, in bursts. That was fixed then by `cooperative_state_timeout_s` 1.0.
  - The 12-13 Nov rig logs show peer-state ages up to 416 ms. The ring-pose limit is still 0.5 s.
  - The branch is not exit-specific: `replanTimer` runs this refresh for every enabled transaction.
- **The diagnostic logging patch, for runtime proof.** It records the failing check, the exception text, the ring-pose
  age against its limit, the peer ages and the replan-timer gaps.
  - File: `~/mdc_checkpoints/2026-11-15_cpp_exit_disable_diag.patch`, logging only.
  - Checked: `git apply --check` and `g++ -fsyntax-only` both pass.
  - **Not applied:** it edits your backend, so it waits for your approval.
- **The smallest correction the evidence supports (proposal, not built):** run ring_bridge's refresh off the
  publishers' executor thread, or give the 20 Hz pose and state timers their own callback group under a
  MultiThreadedExecutor. No freshness limit is relaxed and no stale geometry is used. A separate observation:
  ring_bridge stamps each republished pose "now" even when its mocap input is stale. So the backend's test measures
  ring_bridge's liveness, not the measurement's age.
- **What happens after the correction.** A real ring-pose loss over 0.5 s after the switch would still disable the
  backend. The mission would then park QUAD4, as it does today (section 7, question 3). Parking is not presented here
  as a resolved response.

## 3. Continuity: tests and the replay of the corrected code
**Tests** (`test_cpp_exit_mission.py`, `test_cpp_exit_launch.py`): 10 new.
- **Source-time windows.** Windows of one curve, received 2 ms or 40 ms after their sample time, give the curve at each
  publication's stamp. The error is 12.8 mm with the fix removed and 0 with it. A stale window continues from its
  sample time.
- **Arrival.** It starts on the C++ reference with QUAD4 8-10 cm off: p, v and a continuous to 1e-6. With the seed
  removed, the step equals the tracking error.
- **Escape.** It starts on the executed reference, both while the exit prepares and after the grant.
- **Failed blends.** Failed escape and arrival blends keep p and v.
- **Height.** The body limit bounds the loaded search.
- **Suites.** The mission suite: 1382 passed, with the same 26 older failures as before (identical set). `gate --quick`
  PASSED.

**Replay of the corrected code:** `summary_corrected.csv` and `runs_corrected/`.
- **Setup.** 8 runs: hover fm9h_2, circle fm9hc_10, figure-eights fm9h8_13 and fm9h8_14, each idle and with 8 busy
  cores, recorded error, goal over the mat.
- **Code.** Backend unchanged; the loaded top 3.2.
- **Outcome.** 8 of 8 reached AT_START under C++ authority, then LANDING.

| join (p / v / a) | corrected | the same 8 before |
|---|---|---|
| switch to C++ | <= 0 / 0.0001 / 0.0011 | - |
| C++ ticks | <= 0.00005 / 0.0001 / 0.0032 | <= 0.0109 / 0.0031 / 0.0074 |
| AT_START switch | 0 / 0 / 0 | 0.046-0.091 / <= 0.092 / 0.32-0.80 |
| arrival-blend ticks | <= 0.0004 / 0.0016 / 0.023 | - |

**Times and geometry:**
- Every run joined on the first granted window: 50-155 ms before the splice, 0 late candidates.
- Handoff 0.70-3.84 s.
- Clearance entry to AT_START 12.1-16.2 s.
- Body z <= 3.128 m.
- C++ commanded |x| <= 0.750 m and |y| <= 1.561 m.
- Cable to the carriers' tethers >= 0.396 m (my reconstruction of his shapes).

## 4. Height: the resolved bounds
- **Loaded search top:** `maximum_search_z_m` = 3.2 on the trial line (was 3.3).
- **Exit search top:** `exit_max_search_z_m` 3.2.
- **Exit assembly workspace:** (-1.8, -1.8, 0)-(1.8, 1.8, 3.26), unchanged. Through `assemblyCentreBounds` it gives a
  body-origin centre of +-1.610 m in x and y with a top of 3.150 m. So in the exit the binding body-origin top is
  3.150 m, and the 3.2 m search top is not binding.
- **What these quantities are:** the search tops are body-origin bounds; the 3.26 m is an assembly bound. No margin is
  subtracted twice inside either.
- **The mat (0, +1.55) and the 1.3 m placement rule:** both lie inside the centre bounds. In the twin, a start 1.9 m out
  lies outside them (section 5, R4527).

## 5. Twin wiring checks (headless, lab PC; `results/analysis/cpp_exit_twin_2026-11-15/`)
- **Configs:** `make_configs.py`, which writes `configs/experiments/t15x_*`, `read_run.py` and the per-run JSON.
- **Base line:** the 13 Nov drop-off twin line with `return_after_drop:=cpp exit_handoff:=moving max_body_z:=3.2`.

| run | tree | check | feature | mission |
|---|---|---|---|---|
| R4527 | reviewed | hover, QUAD4 from 1.9 m | join and switch correct | **FAIL**: goal outside the exit bounds; held at y 1.609 m, 2.95 m up, under C++ authority, 630 s (no timeout once granted) |
| R4528 | reviewed | LAND while preparing | Cancel, escape, LANDING correct | complete |
| R4529 | reviewed | hover from 1.4 m | end to end; AT_START step 16 mm (measured restart) | complete; fleet LAND after LANDED_DISARMED |
| R4530 | corrected | hover from 1.4 m | AT_START seeded (QUAD4 15 mm off), arrival blend from 0/0/0.34 | complete |
| R4531 | corrected | LAND while preparing | escape seeded (37 mm off), blend from 0/0/0.22 | complete |
| R4532 | corrected | LAND after the grant | escape seeded (41 mm off), blend from 0/0/0.06 | complete |
| R4533 | corrected | 7 s timeout, backend frozen (SIGSTOP) | the deadline fired at exactly 7.0 s; Cancel; cancel blend | **INVALID method**: his twin `simulation_test_supervisor` saw `/dynamic_planner/transfer_status` stale (15 s wall), stopped with REQUIRED_INPUT_STALE and disarmed QUAD4 about 4 s into the freeze |
| R4534 | corrected | figure-eight exit; fleet LAND during QUAD4's descent | joined exactly, AT_START seeded, LANDING after 2 s | complete; take-back from LANDING, LANDED |
| R4535 | corrected | circle exit | the backend's exit context opened, 33 candidates attempted, 0 accepted; the real 7 s timeout fired; Cancel; cancel blend from 3 mm / 0.010 m/s back to live ring-following | **no exit**: QUAD4 followed the ring until the run's wall cap |

- **R4527.** A twin-geometry mismatch: on the rig the start is within 1.3 m.
- **R4533.** The supervisor's input rule makes the freeze method invalid in the twin. With the method, the escape's seed
  started on the reference while QUAD4 lay on the floor (2.94 m away), which is correct behaviour.
- **R4535's 0 of 33.** The twin does not log the backend's per-candidate reasons; the replay's circle granted in
  2.4-3.8 s. Next: the backend's replan CSV recorded in the twin.

## 6. Not changed (reported)
- **AT_START -> LANDING** steps 4-73 mm and 0.25 m/s in the replay. The existing `initialise_landing()` starts at the
  measured position and descends at `landing_descent_speed` at once. The review keeps LAND as it is.
- **The escape's own AT_START** (Python direct return -> AT_START) restarts from the measured state, as today's direct
  return does: 15 mm in R4531/R4532, 4 cm in R4526. Seeding it is a one-line extension of change 2, if you want it.
- **A failed cancel blend** flies the live ring-following reference as it is. The step is the difference between the
  executed prefix and the live reference: the ring's tracking error plus the fit error, mm to a few cm. No existing
  bounded branch covers it.
- **The C++ return has no timeout once granted** (R4527). The operator's LAND escape is the existing way out.
- **`drop_clear_speed`:** the 6.2-6.4 s climb is left for a separate decision.

## 7. Questions
1. **The logging patch:** apply it to your backend for the runtime proof?
2. **The ring_bridge correction:** the publishers off the refresh's thread, as proposed?
3. **Policy (the one the contract leaves open).** A ring-pose loss over 0.5 s can still disable the backend after the
   switch. Which response?
   - **(a)** Today's park: fly the last window to its stopped end and hold. This is shown unsafe over a moving ring.
   - **(b)** A moving-prefix re-seed from the executed C++ window. This is new and needs its continuity shown.
   - **(c)** The manual escape triggered automatically (the direct Python return). This is excluded unless you approve
     it.
4. **The twin timeout check: which method?**
   - a twin-only switch in the mission that withholds the grant, keeping 7 s and every input live;
   - or the freeze with his twin supervisor's `planner_status_timeout_s` raised for that run.

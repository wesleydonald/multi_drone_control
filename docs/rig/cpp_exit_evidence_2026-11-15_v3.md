# C++ exit: the v2 closeout, evidence v3 (15 Nov 2026, night), for Te-Jen and Codex

This answers `tejen_2026-11-15_cpp_exit_v2_closeout.md`. Nothing is committed and no rig flight is proposed.
- Physical effects are **implemented, flight unverified**.
- **Replay** is open loop and not flight proof.
- **The twin** runs at 0.13-0.16 times real time and checks wiring, not deadlines.

## 1. Code
- **Base:** `~/mdc_checkpoints/2026-11-15_cpp_exit_v2_base/`, which is the v2 evidence tree: HEAD 8f0afc79 + backend
  ee03e8588e30 + mission 0a3f85be658f.
- **This round's changes,** separately reviewable against that base, in `~/mdc_checkpoints/2026-11-15_cpp_exit_closeout/`:

| patch | sha1 | what |
|---|---|---|
| `04_diag_logging.patch` | 694c8d6ecdc8 | the diagnostic logging, as `2026-11-15_cpp_exit_disable_diag.patch` (identical hunks); logging only |
| `05_ring_bridge.patch` + `05_test_ring_bridge_refresh.py` | 12b269ea129c | ring_bridge (our node): one LoadTrajectory per trajectory |
| `06_supervisor_override.patch` + `06_test_cpp_exit_launch.py` | 8f9a234ef6f3 | the twin-only `supervisor_status_timeout_s` |

Logged in `src/tejen_mission/CHANGES_FROM_TEJEN.md` ("15 Nov 2026", the rows stamped 21:00-21:05).

**What is unchanged:** feature-off behaviour, frozen flight configurations, MPC tuning, planner algorithms, margins
and the manual-authority contracts. There are no new phases or gates.

## 2. Diagnostic logging (applied; not yet triggered)
- **Applied** to `transfer_backend_node.cpp` after `git apply --check` against the corrected tree. The backend was
  rebuilt at 21:02: ctest 22/22; the source-contract tests pass.
- **What it records:**
  - which check failed inside `refreshCooperativeWorld()` / `refreshSharedCooperativeObstacle()`, with the caught
    exception;
  - the ring pose's header-stamp age and receive gaps;
  - the replan timer's entry gaps.
- **When it prints:** one DISABLE_DIAG error line at each cooperative disable. That fires once per disable, since the
  backend is then off, so nothing recurs per tick and nothing needed throttling.
- **No disable occurred in any run since** (section 3: 6 replays after the bridge change; 8 twin runs), so there is no
  DISABLE_DIAG line to report.
- **The diagnosis stays qualified:**
  - the retained post-grant disable has not been reproduced in 10 loaded re-runs (5 before the bridge change, 5
    after);
  - the 0.23-0.33 s bridge gaps measured do not themselves breach the 0.5 s ring-pose limit;
  - the empty blocker string is incomplete evidence.

## 3. The bridge refresh off the publication path
**Cause.** ring_bridge's `compute_payload_state` built a new `LoadTrajectory` for every evaluation. Each new
figure-eight rebuilt its 20001-sample arc-length table (`_fig8_table`). One commitment refresh evaluates 252 control
points: 3 carriers + the basket × 63 points (30 s at 0.5 s). It ran synchronously on the node's single executor, ahead of
the same tick's state publication and alongside the 20 Hz ring-pose timer.
- Profiled with the replay's resolved ring_bridge parameters: 134 ms per refresh idle on the laptop, about 80 % of it
  in `_fig8_table`.
- On the circle and hover lines the table does not exist, so the stall there is small.

**Change** (smallest workable). `_trajectory()` keeps one `LoadTrajectory` per set of constructor arguments. Its caches
are pure functions of those arguments, so the commitments are bit-identical.
- Measured: the refresh takes 15 ms idle.
- No thread, executor, lock or asynchronous work: the input snapshot, the one-sequence commitment publication and its
  lifecycle are exactly as before.
- Unchanged: future trajectories, geometry, refresh and publication rates, time bases and freshness thresholds.
- `fake_cooperative_transport_world.py` is untouched.

**Tests** (`test_ring_bridge_refresh.py`):
- one trajectory object per refresh;
- the commitments are equal, element for element, to those built with a fresh object per point (clock held);
- a new trajectory state builds a new object.

**Replay, before against after.**
- Conditions, the same for both: fm9h8_14, own start, recorded error; 1 idle + 5 with 8 busy cores.
- Before: `runs_disable/`, old bridge, backend 17:53. After: `runs_bridge_after/`, new bridge, logging backend 21:02.
  The mission code is identical in content.
- Files: `results/analysis/cpp_exit_replay_2026-11-15/bridge_compare.csv`.

| | ring pose_array max / p99 (s) | pose gaps > 0.15 / > 0.25 s per run | carrier states, worst of 3: max / p99 (s) |
|---|---|---|---|
| before, loaded x5 | 0.264-0.332 / 0.258-0.266 | 22-23 / 6-14 | 0.248-0.295 / 0.224-0.231 |
| before, idle | 0.203 / 0.190 | 22 / 0 | 0.178 / 0.157 |
| after, loaded x4 | 0.069-0.098 / 0.061-0.076 | 0 / 0 | 0.044-0.063 / 0.040-0.041 |
| after, loaded x1 (attempt1) | 0.206 / 0.188 | 9 / 0 | 0.180 / 0.050 |
| after, idle | 0.075 / 0.060 | 0 / 0 | 0.040 / 0.035 |

- **Where the remaining gaps are.** The largest still sit at refreshes. In the clean after-runs, the gap at a refresh
  is about 0.055 s (median) and at most 0.098 s.
- **Remaining stall, reported and not tuned:** after-run attempt1 had 11 ring-pose stalls of 0.16-0.21 s, all at
  refreshes, from 3.8 s before the replay start to +8.3 s, and none after. The same bag window shows none in the other
  four loaded runs. It coincided with config validation and a tree sync started on the same laptop; the cause is not
  established.
- **Commitment validity** (all 12 runs):
  - every refresh published all 3 carriers and the basket with one sequence (0 incomplete; every sequence step +1);
  - coverage after receipt >= 29.67 s before and >= 29.80 s after, of the 30 s horizon;
  - the backend's shared-fresh count was 3/3 at every sample;
  - the commitments now arrive 0.055-0.06 s after their `valid_from`, against 0.17-0.24 s before.
- **Outcome:** all 6 after-runs reached AT_START under C++ authority.

**Republished poses: recorded, not changed.** ring_bridge stamps each republished ring pose with "now" (20 Hz), even
when its mocap input has not updated. That stamp proves ring_bridge is alive, not that the measurement is fresh. Its
outputs are three different things:
- **measured:** the ring pose, re-stamped;
- **predicted:** the carriers' states and the commitments, built from the load planner's trajectory, the present
  being the measured ring;
- **republished held:** the last measured ring pose, if mocap stops.

The upstream timestamp and validity contract is not changed in this patch.

## 4. The circle twin rejection (R4535)
**Records.** The backend's `backend.csv` and `replans.csv` were captured in the lab tree. They are now copied to
`results/2026-11-15/R45xx_*/logs/tejen_backend/` for every twin check, R4527-R4537.

**Cause: authority/version invalidation, not geometry, search or timing.**
- Of the last 45 replans, 41 ended `WORLD_CHANGED_DURING_FINAL_RECHECK` (1 of them a discarded epoch). Their prefix
  and candidate safety was `SAFE_CHANGED_RELATIONS`, octopus `RUNTIME_REACHED` or `GOAL_REACHED`. The other 4 were 3
  solver failures and 1 target-time non-convergence. Over the whole run, 63 of 72 replans, the loaded approach
  included.
- The cooperative world advanced 9 versions during each replan, at a median replan runtime of 288 ms.
- The twin check lines did not type `cpp_world_refresh`, so `cooperative_world_refresh_period_s` stayed at 0 (rebuilt
  on every message).
- The trial line does type it. The rig script has done so for every CPP=on line since 13 Nov (`WR=${WREF-0.2}`),
  together with splice 0.30, tracking 0.06 and the settle bounds. The successful circle replay flew 0.2.
- In the successful figure-eight twin run (R4534) the world advanced a median of 0 versions per replan.

**Smallest supported next action: one circle check on the trial line's values.**
- R4536: granted, the window joined the prefix exactly, and QUAD4 landed.
- Its replans were mostly ACCEPTED (the moving-rendezvous and after-recheck kinds) and GOAL_SEEN; 2 of 58 were
  WORLD_CHANGED.
- No budget, gate or geometry was changed.
- R4530-R4534 passed without these values.

## 5. The valid timeout check (R4537; R4533 retained as invalid)
- **Method:** his backend was frozen (SIGSTOP) at the clearance's completion, as in R4533. His twin supervisor's
  `planner_status_timeout_s` was typed 120 s for this run only (`supervisor_status_timeout_s`; the default stays 15 s;
  the rig refuses it). The backend was resumed (SIGCONT) 2 s after the failure.
- **Why not withhold the grant in the harness:** the harness cannot withhold the grant without stopping the backend's
  status, and no production switch was added for it.
- **Result:**

| check | result |
|---|---|
| cancels at 7 s | the deadline fired at 7.0 s after the clearance (66.2 -> 73.2) |
| follows the live ring reference through the existing blend | cancel blend 1.00 s from 3 mm / 0.011 m/s / 0.008 m/s^2; QUAD4 followed the figure-eight for 10 s: 1.29 m travelled with the ring's 1.25 m, within 0.031 m of its centre in xy, z 2.905-2.915 m |
| still accepts manual LAND | his LAND at +10 s: the escape seeded on the executed reference (19 mm off), its blend, AT_START, LANDING, LANDED_DISARMED; the fleet LAND took it back; no supervisor stop |

A slow twin does not validate the real-time deadline.

## 6. Tests and builds
- **Backend:** ctest 22/22; source contracts pass.
- **Mission suite:** 1385 passed, with the same 26 older failures (identical set).
- **`tools/gate.sh --quick`:** one run failed in pytest (the failing test was not captured), then two runs PASSED
  (2331 passed).

## 7. Twin checks, the whole set
- R4527-R4535 are as in v2.
- R4536 and R4537 are above.
- R4533 stays VOID: the freeze method tripped the twin supervisor.

## 8. Limitations kept
- **A failed cancel blend** can still switch discontinuously to the live reference.
- **Failed escape and arrival blends** keep position and velocity, with a possible acceleration step. They are not
  fully C2-continuous fallbacks.
- **Legacy steps:** AT_START -> LANDING (4-73 mm, 0.25 m/s) and the manual escape's own arrival at AT_START (15 mm)
  keep their steps. Those paths and the clearance-climb speed are left alone.
- **The C++ return has no timeout once granted** (R4527). The operator's LAND escape is the way out.
- **The bridge correction** makes a stall-induced disable less likely. It does not remove real input loss.

## 9. The one question: genuine input loss after the switch
After the switch to C++, a real input loss can still disable the backend. That means the ring pose older than 0.5 s, or
the carriers' states older than 1.0 s, for example mocap dropping the ring. What should QUAD4 then do? The options:
- **(a) Today's response.** Fly the last committed C++ window to its stopped end and hold there until the operator's
  LAND (the escape).
  - Needs no input.
  - QUAD4 parks about 2.9 m up wherever the window ends while the ring keeps moving. In the replay the carrier's tether
    passed 0.073 m from QUAD4's cable (centre lines).
  - Safety rests on the operator's reaction time.
- **(b) The existing timeout response, applied after the grant.** Cancel and blend back to the live ring-following
  hold over the ring (the R4537 path).
  - Needs the measured ring pose. It cannot help when the ring pose itself is what was lost; it fits a loss of carrier
    states only.
  - It flies QUAD4 back over the ring on an unchecked path.
  - It is existing code; using it after the grant is new.
- **(c) A moving-prefix re-seed from the executed C++ window** (reference recovery).
  - Needs the inputs back before it can plan.
  - It is new, and its continuity is not shown.
- **(d) The automatic escape** (the direct Python return, then LANDING).
  - Needs no ring input.
  - It is immediate and continuous (seeded), but not collision-certified.
  - It is excluded unless you approve it.

Which one, or which one per lost input?

# C++ exit: implementation and evidence (15 Nov 2026), for Te-Jen and Codex's review

This answers the implementation handoff (`tejen_2026-11-15_cpp_exit_implementation_handoff.md`): plan v3 is implemented in
Wesley's fork, opt-in, and validated offline. **No rig flight is authorised or proposed here.**
- **Replay:** open loop on the 13 Nov recorded topics, with perfect or recorded tracking and no swing. It is **not
  flight proof**, and the replay clock does not validate real-time deadlines.
- **Twin:** the headless twin checks are not yet run (section 6).

## 1. Code (uncommitted; commits are Wesley's)
**Patches:** `~/mdc_checkpoints/2026-11-15_cpp_exit/`:
- `01_backend_tracked.patch` (sha1 ee03e8588e30) plus `01_backend_new_files.tar.gz`;
- `02_mission_tracked.patch` (3c9f996bf21d) plus `02_mission_new_files.tar.gz`;
- 8 tracked files, +1792 / -54, plus 6 new test files.
- The pre-implementation checkpoint: `~/mdc_checkpoints/2026-11-15_pre_cpp_exit/`.

**Interface:** `cpp_exit_impl_contract_2026-11-15.md`. **Change log:** `src/tejen_mission/CHANGES_FROM_TEJEN.md`, entry
"15 Nov 2026".

**Backend** (`reference_recovery.hpp`, `transfer_backend_node.cpp`):
- `seedtraj exit <seq> <stamp_ns> <deadline_ns>` on the existing recovery protocol.
- The exit context opens on validated evidence **before** activation and grant checks: the prefix, coverage, the
  deadline, and a fresh `object_attached` false. It installs the unloaded geometry (5 deg), the exit assembly workspace
  and the search top. It skips the payload-attached checks for that transaction only.
- Seeding is `initializeCommittedTrajectory(prefix)` and the splice is on the prefix. Never a stationary seed or a clock
  shift.
- Relative readiness, prefix certification, refresh keep/invalidate, admission (coverage, deadline), and restore on
  every cleanup path.
- Two fixes after the first replay:
  - exit-transaction windows carry their sample time;
  - one window is published at the post-grant install.

**Mission** (`c1f2_handoff.py`, `online_join_planner.py`, `partner_mission.launch.py`):
- ports of his spline and boundary equations;
- the fitted prefix (first three control points from the executed p/v/a) and verbatim refreshes;
- the 7 s budget from the clearance's end, never reset;
- QUAD4's reference evaluated from the fitted coefficients at each publication's stamp;
- window acceptance on the prefix to the join, then the existing RETURN_TO_START switch;
- timeout or rejection: Cancel + enable=false and a blend back to live ring-following;
- LAND in the exit: local selection first, then Cancel + enable=false, then the escape from the published horizon at
  the true switch time into the direct return and LANDING;
- AT_START -> LANDING after 2 s;
- `exit_handoff:=moving` in the launch.
- Two fixes after the first replay:
  - the exit's freshness and join tests use the planner's 1 us time tolerance; float seconds near 1.8e9 resolve
    2.4e-7 s;
  - a prompt cancel if granted with no window joined by the join + 0.1 s.

## 2. Resolved parameters (trial line: CPP=on CPPMOVING=1 SWING=3 SPLINE=2 RET=cpp, `exit_handoff:=moving`, `max_body_z:=3.2`)
- **Backend in the exit context:**
  - `exit_max_swing_angle_rad` 0.0873 (5 deg; the loaded line keeps 0.0524);
  - `exit_workspace` (-1.8, -1.8, 0)-(1.8, 1.8, 3.26), assembly bounds through his `assemblyCentreBounds` (centre
    +-1.610 m, top 3.150 m; the mat (0, +1.55, ~2.9) inside);
  - `exit_max_search_z_m` 3.2 (the loaded line keeps 3.3);
  - `exit_protected_s` 1.5;
  - `moving_prefix_topic` `/dynamic_planner/transfer_prefix`.
- **Mission:**
  - `exit_hold_z_m` 2.9, `exit_blend_s` 1, `exit_prefix_refresh_s` 1, `exit_prefix_horizon_s` 5,
    `exit_protected_s` 1.5;
  - `exit_handoff_timeout_s` 7, `at_start_land_after_s` 2, `exit_ring_commitment_max_age_s` 2 (the ring bridge
    refreshes at 1 Hz);
  - knots 0.25 s.
- `require_payload_attached_for_enable` stays true at launch.
- The resolved graph is in `results/analysis/cpp_exit_replay_2026-11-15/launch_dump_trial_fm9h8.json`.

## 3. Tests and build
- **Backend:** ctest 22/22 (one new: parse, context, relative readiness, refresh window, admission; the real planner core
  seeded with a moving prefix: committed path = prefix before the splice to 1e-12, p/v/a at the splice to 1e-7).
- **Backend source contracts:** 13/13. The exit smoke on the binary PASSes, and so does the existing recovery smoke on
  the new binary.
- **Mission:** 45/45 new (prefix fit and refresh; timer never reset; stamps; grant without a stop; timeout, rejection
  and stale ring blend to live without a step; LAND between publications; AT_START -> LANDING; feature-off tick-for-tick
  against HEAD; launch refusals and parameters).
- **The full mission suite:** 1372 passed, with the same 26 older failures as before the change.
- `tools/gate.sh --quick` PASSED.

## 4. Replay milestone: does his search find and install an exit? Yes
`results/analysis/cpp_exit_replay_2026-11-15/`:
- `run_exit.sh`, `driver_exit.py`;
- `summary.csv`, `runs/<run>/` (replans.csv, backend.csv, status.jsonl, events, q4.csv);
- `summary_prefix.csv` + `runs_prefix/` (the pre-fix set).

**Setup:**
- His real binary plus ring_bridge on the 13 Nov bags.
- The mission's real exit methods at 30 Hz.
- QUAD4 is the published reference plus that flight's recorded 13 Nov error, or perfect tracking.
- 8 flights (hover, circle, figure-eight), idle and with 8 busy cores, goal over the mat or over the flight's own
  start.

**Before the fixes:**
- His backend found, certified and installed an exit in every run that asked.
- 59 of 65 reached AT_START. The 6 failures were one join defect of ours: the only joinable window came late or was
  refused by float rounding.

**After the fixes, 67 runs:**

| condition | runs | AT_START under C++ | handoff (s) | C++ return (s) | clearance entry -> AT_START (s) |
|---|---|---|---|---|---|
| recorded error, mat, idle | 11 | 11 | 2.23 [0.70-4.13] | 6.74 [4.17-8.23] | 15.2 [max 18.4] |
| perfect tracking, mat, idle | 8 | 8 | 0.62 [0.43-1.83] | 5.62 | 12.8 |
| recorded error, mat, 8 busy cores | 9 | 9 | 2.63 [0.73-3.83] | 5.24 | 14.6 |
| perfect tracking, mat, 8 busy cores | 8 | 8 | 0.77 [0.50-1.60] | 5.53 | 12.6 |
| recorded error, own start, idle | 10 | 10 | 1.80 | 4.96 | 12.9 |
| recorded error, own start, 8 busy cores | 10 | 9 (1 parked) | 2.13 | 4.87 | 12.8 |
| recorded error, mat, at the real drop instant | 11 | 11 | 1.44 | 5.20 | 13.2 |

- **Joins.** The first granted window joined the prefix in all 67 runs. It arrived 15-245 ms before the splice (median
  120 ms). The 0.1 s prompt cancel never fired. Rounding refusals: 0.
- **Plans.**
  - Before the grant: 171 attempted, 147 accepted, 147 installed. After it: 290 / 277 / 277.
  - Late 2, invalidated 11, none late after the grant.
  - Replan runtime max 349 ms.
- **Time** (median [range]; times in s):
  - prepared 1.33 [0.23-2.47] after the clearance with recorded error, 0.40 with perfect tracking;
  - handoff 1.58 [max 4.13];
  - C++ return 5.23 [max 8.23];
  - CLEAR_DROP_ZONE entry to AT_START 13.2 [11.3-18.5], against 6.7-9.3 for the 13 Nov Python exits.
  - The climb to the 2.9 m hold takes 5.9-6.8 s at the default `drop_clear_speed` 0.08 m/s.
  - The backend's 5 cm relative-readiness gate held grants in 15 recorded-error runs, always well inside 7 s.
- **Reference joins** (p / v / a steps, m, m/s, m/s^2):

  | join | step |
  |---|---|
  | latch | 0 / 0 / 0 |
  | refreshes | <= 0 / 1e-4 / 0.003 |
  | grant | <= 0 / 1e-4 / 0.0025 |
  | switch to C++ | <= 0.0025 / 0.004 / 0.020 |
  | C++ ticks | <= 0.011 / 0.007 / 0.012 |
  | AT_START with recorded error | up to 0.116 / 0.34 / 1.88 |

  Two of these come from existing code:
  - the C++ tick step: the selection advances a window by its receive age, not its stamp (`online_join_planner.py`
    ~6640);
  - the AT_START step: the Python AT_START transfer restarts from the measured state (~9864).
- **Geometry** (66 runs at AT_START; my reconstruction of his shapes, not his checker):
  - body z <= 3.137 m;
  - C++ commanded |x| <= 0.926 m and |y| <= 1.560 m;
  - body gap >= 0.59 m;
  - cable to the carriers' tethers >= 0.344 m;
  - cable to the basket >= 0.593 m.

## 5. The one failure: a backend self-disable after the switch parks QUAD4 (a failed safety result in replay)
**What happened** (`runs/fm9h8_14_e_own_load8`, 8 busy cores):
1. A clean join and switch at +8.7 s (handoff 2.34 s).
2. At +9.62 s his backend disabled itself with `COOPERATIVE_SHARED_INPUT_INVALID_DISABLED`. Its blocker string was empty,
   so the failing check is probably inside `refreshCooperativeWorld()`. That is his existing input-loss logic, not the
   exit's.
3. The mission did what it does today on a stale C++ reference: it flew the last window to its stopped end, then held.
   `c1f2_reference_recovery_enabled` is off on this line.
4. QUAD4 parked at (0.44, -0.16, 2.94). The figure-eight's carrier then passed under it: cable to tether 0.073 m (centre
   lines), under 0.25 m for 6.7 s and under 0.10 m for 1.6 s.

This is the 13 Nov class of park; the same branch exists after a C++ approach today.

**The one question (contract 3.8: no existing contract covers it).** If the backend disables itself after the exit's
switch to C++, what should the mission do?
- **(a)** Park, as today.
- **(b)** The existing reference recovery: re-seed and re-grant on the moving prefix.
- **(c)** The manual-LAND escape at once: the direct Python return from the published horizon, then LANDING. This is
  manually authorised today and not collision-certified.

The lab chat's suggestion is (c), with the qualification kept: over a moving ring, (a) is shown unsafe in this replay.

## 6. Not yet done
- The headless twin wiring and manual-override checks: hover first, the 7 s timeout, LAND preparing and after the
  grant, and the fleet LAND after QUAD4 is LANDED_DISARMED and during its descent.
- Any physical commissioning, which needs the operator's go-ahead after the review.
- The two existing-code steps above (the receive-age advance, the AT_START restart) are reported, not changed.
- `drop_clear_speed` (6-7 s of the exit) is reported, not tuned.

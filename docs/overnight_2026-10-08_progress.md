# Overnight 7-8 Oct: progress log

Plan: `/home/wesley/.claude/plans/whimsical-kindling-minsky.md` (approved 8 Oct ~01:00). Branch `week4`
is tomorrow's rig tree: only tested pieces go there. The new defaults and the drop-and-land work live on
branch `safety-defaults` (git worktree) until they are twin-verified.

| item | status | tested | committed | live for 8 Oct rig |
|---|---|---|---|---|
| A1 plates clockwise 0-11, RViz 1-4, preflight wording | done | mpc_planner 140, bringup 92, preflight 25, sim_comm 45, gate --quick | f3b3072 | yes |
| A2 rig_flight.sh: run name last (old order works), auto-suffix _2.., dry run, per-run code.txt | done | 4 dry-run tests | 34a4dd5 | yes |
| A3 detector glitch guard: no detection on a stale ring pose (>0.1 s), a drone's stale pose skips that drone, all cables over at once = ring pose jump (no action) | done | dissipative 49 (11 detect/soft), full suites, gate; replay over 13 flown 7 Oct rig logs: fires only on r200006 (0.38 s after the slip), no all-over ticks | 80f0519 | yes |
| A4 QUAD1 arming: each arm attempt judged by frames after its own edge (sparse QUAD1 telemetry), retry on an explicit refusal, up to 4 s; rig TAKEOFF refused unless every FC reports armed within 10 s ('unconfirmed' no longer passes); a disarm before takeoff keeps trackers and manager up, so replug + ARM again works without relaunching T2 | done | arm_switch 10, fleet_manager 35, utility 15, tracker 81, bringup 92; twin re-ARM check queued | 618e315 | yes (bench props-off ARM/DISARM first) |
| A5 twin reliability: runner parameter read-back after the run, not during ARM/TAKEOFF (Fix B); tracker pose/ref watchdogs on the node clock, sim time in the twin, wall time on the rig as before (Fix A); schedule starts only once the planner publishes refs for every drone in mpc/dissipative mode (Fix C); a run grounded before TAKEOFF stops 2 s later instead of running out its duration (Fix D) | done | tracker 83, utility 15, tools 277 (new test_runner_ready 8), gate --quick; rig real mode keeps use_sim_time false (test_real_mode_launches) | fdbb39a | twin only (tracker change is a no-op on the rig: wall clock there) |
| A6 circle detach: a freed drone during a circle (orbit/circle/spin) steps 0.5 m out from the ring, then out from the PATH centre to r_orbit + 1.0 m (1.5 m at r 0.5), level, before it holds or lands (departed_land); other trajectories unchanged | done | dissipative 51 (2 new), gate --quick; twin R1149, R1150 (lab, one slot): both PASS, detected 0.27/0.21 s, tilt 9.3/8.7 deg, freed drone 1.45 m from the circle centre | 1ba38bc | **yes: r306 on the sheet** (start the ring 0.5 m toward -y) |
| A6b re-ARM without relaunch in the twin: ARM, operator DISARM before takeoff, ARM, TAKEOFF, hover, LAND (`w4_rearm_3915.yaml`) | done | R1151 PASS: ARM sequence complete twice, lifted, landed, no relaunch | 1ba38bc (config) | yes (the r300 bench check) |
| A8 registry: R1146-R1148 void cause (read-back + wall-clock watchdog), R1148 row, U1 hover 2 valid of 4 | done | - | 04b2884 | - |
| A9 .gitignore agile_and_collective_supplementary/ (2.2 GB) | done | - | 04b2884 | - |
| A7 rig sheet rewritten (`docs/rig_2026-10-08_commands.md`): clockwise plates and a convention table, names last, preflight per run, r300 props-off arm check, ARM FAILED replug + re-ARM line, 1.3 m clear on +y, mats, 10 s steady hover before a release, build line with drone_communication/utility_objects, stop rules restored (+ second loss / 30 deg = disarm today), r306 circle detach; T2 detach lines now name `drone N (/drone_i)` | done | dissipative 51, gate --quick | 5cb06b8 | yes |
| D Tejen: ff-pulled d9c69415 -> 611dc3d2 (14 commits since our fork a7524bae, 25 Sep-7 Oct); classified in `docs/tejen_sync_2026-10-08.md` | done (classify only) | - | 06ebe9b | no: port after the rig on `tejen-sync`, with your word |
| B2 model integral on resizing fleets: learned ring force kept through a resize and applied to the new OCP at once, frozen in the reconfiguration hold; the `resizes` refusal dropped | code done, twin A/B running (laptop, worktree) | mpc_planner 141 (2 new), bringup 92 (3 updated, 1 new), dissipative 51, worktree gate | f2f7d74 (branch safety-defaults) | no |
| B3 default mode dissipative (legacy keeps mpc), detection 0.06 on by default (legacy off) | code done, twin A/B running | as B2 | f2f7d74 (branch safety-defaults) | no |
| B1 drop-and-land | card v1 + critic: REWRITE (the trackers' 60 deg fault wins the race on r0013/r200006; the trigger must start in the trackers); B0 baseline queued on the lab (R1155, R1156); rig-bag tilt replay running | - | card f84b10b | no |
| analysis tools read the dissipative planner's log (metrics, thrust_fit, planner_replay); a param dump holding a DDS error no longer breaks metrics | done | tools 278 | bddad69 | - |
| C1 `tools/lag_metrics.py` (D, L0, L5, c, s on the planner clock) | done; reproduces the overnight study (R1127, R1106, r206e60 within 0.005 s) | - | f788260 | - |
| C2 lag H1: twin + 0.035 s on every reference publish (test-only `ref_publish_delay_s`, branch time-cascade) | card a73292d; code + unit test in the worktree; runs after the lab baselines | - | (branch time-cascade) | no |
| C3 time-consistent cascade | card dd5bca5, critic running | - | - | no (rig check 14 Oct) |
| C4 strike the 0.2 s time-lead fallback in the week plan | done | - | d1b999b | - |

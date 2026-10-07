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
| A6 circle detach: a freed drone during a circle (orbit/circle/spin) steps 0.5 m out from the ring, then out from the PATH centre to r_orbit + 1.0 m (1.5 m at r 0.5), level, before it holds or lands (departed_land); other trajectories unchanged | code done; twin re-fly running (R1149, R1150 lab, one slot) | dissipative 51 (2 new), gate --quick | 1ba38bc | only if both twin runs pass |
| A6b re-ARM without relaunch in the twin: ARM, operator DISARM before takeoff, ARM, TAKEOFF, hover, LAND (`w4_rearm_3915.yaml`) | running (laptop) | - | 1ba38bc (config) | - |
| A8 registry: R1146-R1148 void cause (read-back + wall-clock watchdog), R1148 row, U1 hover 2 valid of 4 | done | - | (this commit) | - |
| A9 .gitignore agile_and_collective_supplementary/ (2.2 GB) | done | - | (this commit) | - |

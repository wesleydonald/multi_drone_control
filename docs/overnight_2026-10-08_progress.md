# Overnight 7-8 Oct: progress log

Plan: `/home/wesley/.claude/plans/whimsical-kindling-minsky.md` (approved 8 Oct ~01:00). Branch `week4`
is tomorrow's rig tree: only tested pieces go there. The new defaults and the drop-and-land work live on
branch `safety-defaults` (git worktree) until they are twin-verified.

| item | status | tested | committed | live for 8 Oct rig |
|---|---|---|---|---|
| A1 plates clockwise 0-11, RViz 1-4, preflight wording | done | mpc_planner 140, bringup 92, preflight 25, sim_comm 45, gate --quick | (this commit) | yes |
| A2 rig_flight.sh: run name last (old order works), auto-suffix _2.., dry run, per-run code.txt | done | 4 dry-run tests | (this commit) | yes |
| A3 detector glitch guard: no detection on a stale ring pose (>0.1 s), a drone's stale pose skips that drone, all cables over at once = ring pose jump (no action) | done | dissipative 49 (11 detect/soft), full suites, gate; replay over 13 flown 7 Oct rig logs: fires only on r200006 (0.38 s after the slip), no all-over ticks | (this commit) | yes |
| A4 QUAD1 arming: each arm attempt judged by frames after its own edge (sparse QUAD1 telemetry), retry on an explicit refusal, up to 4 s; rig TAKEOFF refused unless every FC reports armed within 10 s ('unconfirmed' no longer passes); a disarm before takeoff keeps trackers and manager up, so replug + ARM again works without relaunching T2 | done | arm_switch 10, fleet_manager 35, utility 15, tracker 81, bringup 92; twin re-ARM check queued | (this commit) | yes (bench props-off ARM/DISARM first) |

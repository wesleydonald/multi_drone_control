# For Tejen: the C1F install gate discards almost every plan (23 Oct 2026)

**Short version.** Since the tejen-sync2 merge, your transfer backend installs a finished plan only if the world
version has not moved since the plan's final check. Our carriers move your world 50 times a second (ring_bridge) and
the ring pose 20 times a second, so nearly every finished plan is thrown away and the grant comes at random. Before
the merge the same handoff took under 0.6 s. Could the install re-check the finished plan against the current world,
instead of discarding it when the version changed?

**What we saw** (our twin, 1 m cables, your floor_join and the Nr rejoin, five runs on the merged tree; numbers from
your own `replans.csv`):

| Run | Replans | Accepted by the planner | Installed | Discarded: `world_version_changed` | Final-check failures | Settle -> first install (s) |
|---|---|---|---|---|---|---|
| floor join 1 | 609 | 551 | 2 | 549 | 3 | 73.8 |
| floor join 2 | 1057 | 995 | 1 | 994 | 6 | 177.2 (run stopped before TRANSIT) |
| Nr 1 | 1116 | 1109 | 1 | 1108 | 5 | 51.8 (run stopped before TRANSIT) |
| Nr 2 | 146 | 139 | 2 | 137 | 0 | 8.2 |
| Nr 3 | 761 | 747 | 2 | 745 | 4 | 11.2 (then 135.2 to the second install) |

3,533 of 3,541 accepted plans were discarded; 8 were installed. Collision final-check failures are rare (18 in 3,689
replans), so the wait is the install rule, not the plans.

**Where it happens** (the merged tree):
- `standalone/include/dynamic_planner/execution_authority.hpp` (the install decision, ~lines 28-40):
  `world.runIfVersionCurrent(*result.certified_world_version, ...)` returns `world_version_changed` if anything changed.
- `src/transfer_backend_node.cpp`: every carrier sample (~line 1422) and ring pose sample (~line 1519) calls
  `refreshSharedCooperativeObstacle` -> `world_->upsertCooperativeTrajectory(...)` -> `++version_`
  (`standalone/src/world_snapshot.cpp`).
- TRANSIT needs two installs (the prepared plan, then a fresh one after the grant), and `c1f2b_cpp_authority.yaml` has
  no timeout, so the hold over the mat can last minutes.

**On the rig** the backend runs the same code with authority (the "SIM authority" log line is a fixed string). The
solve runs on wall time, so the race depends on when a solve finishes relative to the next carrier update; our
estimate is 6-30 s per grant on the rig, possibly much longer on a fast laptop. The twin likely overstates it (its
timers fire on the same sim ticks).

**Options we can see** (yours to choose):
1. At install, re-check the finished plan against the current world (or against the world changed only by
   cooperative-trajectory updates within a tolerance), and install it if it still passes.
2. Treat cooperative-trajectory refreshes as not invalidating a certified plan unless a carrier moved more than a set
   distance since the check.
3. On our side we could feed your backend at a lower rate (switch, default off), but your world model would be staler,
   so we would rather not.

**For Thu 29** we will fly with a stop rule: GIVE UP if there is no TRANSIT_TO_REATTACH 60 s after your `settled` line.
If you change the gate, we can re-fly the twin rehearsal on your branch first (`configs/experiments/thu29_*`; your
`logs/tejen_dynamic_planner/c1f6_rendezvous_*/replans.csv` shows the counts).

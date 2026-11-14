# Admission and timing: why candidates are rejected and when progress stops (13 Nov 2026)

These are open-loop real-time replays of the 12 Nov flights through `dynamic_planner_transfer_backend`, from
`../../cpp_realtime_2026-11-13/full/runs/` (read that folder's README for the replay setup and caveats). Each run
is one replay. The virtual QUAD4 follows his reference perfectly. To rebuild every PNG, run
`python3 make_timing_figures.py` (it needs only pandas, numpy and matplotlib, no ROS). The time axis in every
figure is seconds since his authority grant (the first `auth == 1` row of driver.csv). Status colours are the same in
every figure; rare statuses are grouped as "other". "Window" means the replans from the grant to the capture, or to
the end of the run if QUAD4 was never captured.

Settings come from each run's backend.yaml. "refresh off" means `cooperative_world_refresh_period_s` is not set
(binary v0, no coalescing). "load N" means N busy Python loops on the 12-CPU laptop, which only stand in for the rig
stack. fm9_5, fm9hc_2, fm9_5_r02 and fm9_5_latch_legacy have no binary.txt. The other runs used binary v2
(02:03:52).

## fig1a_status_strip_still_ring.png: status over time and distance to target, still ring, no load
Each run has two rows: a strip with one tick per replan, coloured by status, and the horizontal (dxy) and vertical
(|dz|) distance from QUAD4 to the target. In v0 (fm9_5, fm9hc_2), almost every replan after about 13-19 s ends
WORLD_CHANGED_DURING_FINAL_RECHECK. QUAD4 stops at 0.70-0.78 m (fm9_5) or 0.82-0.95 m (fm9hc_2) and is not
captured. With the 0.2 s refresh (fm9_5_r02), the same flight is mostly ACCEPTED / ACCEPTED_MOVING_RENDEZVOUS and is
captured at 14.6 s. legacy_square accepts plans, but they are partial. The run then alternates ACCEPTED_PARTIAL with
INCUMBENT_REMAINING_UNSAFE_AFTER_LOCAL_PLAN_FAILED, and QUAD4 holds at about 0.65 m. This matches the earlier finding
that the drop point is inside the legacy obstacle set.

## fig1b_status_strip_under_load.png: the same flight with 8 busy loops, splice 0.10 / 0.15 / 0.20 s
All four runs use refresh 0.2 and cap 2.9. At splice 0.10 (fm9_5_z29_load8), the strip turns red
(LATE_CANDIDATE_REJECTED_AFTER_RECHECK) from about 5 s after the grant, when QUAD4 gets near the ring. It stays red
for the rest of the run: 194 of the 205 replans after 6 s were late. QUAD4 stops at dxy 0.56-0.63 m and is not
captured. At splice 0.15, A_load8_1 is mostly green and A_load8_3 is red-speckled, and both are captured (18.3 and
16.3 s). At splice 0.20 (B_load8_1) the strip is green with a few SOLVER_FAILED, and QUAD4 is captured at 18.0 s.

## fig2a_runtime_vs_time_splice.png: replan_runtime_ms against the splice line
Each dot is one replan and red x marks `candidate_late = 1`. The horizontal line is `fixed_splice_lookahead_s`. In
every run the runtime steps up from about 50 ms to 100-150 ms about 5 s after the grant, as QUAD4 approaches the
tethers. A candidate is late exactly when its runtime crosses the splice line. Without load, at splice 0.10
(fm9_5_z29), 32 of 88 were late but enough came in time, and runtime fell back to about 50 ms just before the capture.
With 8 loops at splice 0.10, the runtime sits at 130-160 ms for 40 s, so nothing new is ever installed. At 0.15 s the
runtime sits on the line (22 of 79 late). At 0.20 s there were 0 of 91 late, and the highest runtime was 156 ms.

## fig2b_runtime_cdf_load_and_splice.png: runtime distribution per configuration
The window is the first 16 s after the grant, so every run is compared while still approaching. (a) At splice 0.10,
going from 0 to 11 busy loops moves the runtime distribution to the right. The share of late candidates goes from
38 % at 0 and 4 loops, to 66 % at 6 and 8 loops, to 90 % at 11 loops. Capture does not follow the late share
monotonically: load6 and load11 were captured and load8 was not. With one replay per load, capture at 0.10 depends on
a few in-time candidates arriving at the right moment. (b) At 8 loops, the 0.15 and 0.20 runs have almost the same
runtime curve, but the longer splice admits it. The late shares are 66 % at 0.10 s (1 run, not captured), 11.2 % at
0.15 s (5 runs, 5 captured) and 0.0 % at 0.20 s (2 runs, 2 captured). The 0.10 run's curve sits to the right of the
others because QUAD4 was already parked near the tethers, where replans are slow.

## fig3_world_version_rate_and_recheck.png: world version rate and the final-recheck outcome
(a) `world_version` advances about 214 times/s in v0 (fm9_5, fm9hc_2). With the 0.2 s refresh it advances about
32-34 times/s, but those versions come from only about 5 refreshes/s (dashed: `cooperative_world_refresh_count`). The
refresh still produces several version increments per refresh. (b) `recheck_world_version - planning_world_version`
counts the versions that arrived while one candidate was planned. In v0 that was 6-25 (median 9) for every replan.
With the refresh it was 0 for 90-95 % of replans. Because versions arrive at about 32/s but rarely during a planning
interval, they presumably arrive in groups at each refresh (an inference). (c) In v0, 75-84 % of replans needed the
retry recheck, and the world changed again during it, so the commit was refused with
WORLD_CHANGED_DURING_FINAL_RECHECK. With the refresh, 91-95 % of replans pass on the first recheck
(`recheck_change_kind` INITIAL_FULL) and 3 % end WORLD_CHANGED. In his code (receding_horizon_planner.cpp, about
lines 1005-1128) there is one recheck and one retry. `attemptAtomicCommit` then fails if the world version moved
during the retry.

## fig4_status_share_per_configuration.png: status share per run, stacked %
These are the shares within the window, for all selected runs side by side. The dominant cause differs by run:
- v0: WORLD_CHANGED (75-84 %).
- 8 loops at splice 0.10: LATE (83 %).
- legacy_square: accepted partial (59 %) plus incumbent-unsafe (24 %).
- moving ring at splice 0.10 (nro_p02): LATE 39 %, incumbent-unsafe 27 %, PADDED_CLOSEST_PARTIAL 11 %, accepted
  partial 20 %.
- moving ring at splice 0.20 (nro_mv_z29s20): accepted partial 68 %, TARGET_TIME_FIXED_POINT_NOT_CONVERGED 15 %.

## fig5_moving_ring_splice010_vs_020.png: moving ring, nro_p02 (splice 0.10) vs nro_mv_z29s20 (splice 0.20)
Rows from top: status strip, distance to target, clearance (min_cable_tether_m and min_body_m, centre lines), QUAD4
reference speed, and `commit_end_s - now` with `execution_fallback_active` shaded. In nro_p02 a LATE streak runs from
about 29 to 40 s (42 of 60 replans). No new plan is installed, the plan time left falls linearly to 0 at about 40 s,
and the backend enters UNCERTIFIED_HARD_HOVER (reason INCUMBENT_REVALIDATION_UNSAFE) for about 5 s. QUAD4's reference
speed is under 0.05 m/s from 36.9 s to the end, so it is parked for over 50 s while the ring circles. After 40 s, the
only installs are 12 ACCEPTED_PARTIAL plans, each giving about 7 s of plan time, and none of them moves QUAD4. The
replans in between are incumbent-unsafe (115), LATE (72) and PADDED_CLOSEST_PARTIAL (34), and the backend returns to
hard hover each time the plan runs out. The tether clearance cycles with each lap, down to 0.19 m (74 s). At splice
0.20, the status is mostly accepted partial and QUAD4 keeps moving until it is captured at 40.8 s. The closest
cable-tether distance before capture was 0.35 m. The grey area after capture is the replay continuing without the
attach, so it is not a physical state: the 0.00 m there at about 46 s is not a contact.

## fig6_time_budget_and_install_margin.png: where the replan time goes, and the installation margin
(a) These are mean components of `replan_runtime_ms` within the window. With outer_octagon on the still ring,
`safety_check_ms` is the largest part:
| run | safety check (ms) | octopus search (ms) | obstacle build (ms) |
|---|---|---|---|
| no load | 33-52 | 9-15 | about 20 |
| 8 loops at splice 0.10 (fm9_5_z29_load8) | 90 | 14 | about 20 |
| legacy_square | 0.5 | 37 | 2.3 |
On the moving ring the search dominates: 44-55 ms of search against 23-25 ms of safety check. The p95 runtime was
102-167 ms in every octagon run. (b) The installed splice margin is the splice time minus
`authority_install_time_s` (the evidence.py formula). The medians and minimums were:
| run | median (ms) | min (ms) |
|---|---|---|
| nro_p02 (splice 0.10, moving ring) | 5 | 0 |
| fm9_5_A_load8_3 (splice 0.15) | 75 | 3 |
| fm9_5_B_load8_1 (splice 0.20) | 115 | 70 |
| nro_mv_z29s20 (splice 0.20, moving ring) | 105 | 40 |

## fig7_commit_left_and_last_install.png: when progress stops, still ring
(a) `commit_end_s - now`: how much of the installed plan is left. (b) Time since the last replan with
`authority_installed = 1`. Two runs stop making progress:
- fm9_5_z29_load8: the last install is at 4.8 s. Nothing new is installed for 43 s, the plan runs out at about 15 s,
  and SMOOTH_BRAKE fallback is active from 22.8 s.
- fm9_5 v0: the last install is at 13.1 s. The plan runs out at about 24 s, and 156 of the 167 replans after 13.2 s end WORLD_CHANGED.

In the runs that were captured, new plans keep being installed (median gap 0.2 s, longest gap 0.4-1.0 s before the capture), and about 1 s of plan time stays
ahead. legacy_square also keeps installing plans (ACCEPTED_PARTIAL, about 6.4 s of plan each), but they are partial
plans that do not bring it closer to the target. For legacy_square, (b) therefore does not measure progress; fig1a's
dxy does.

# Te-Jen's C++ planner: what was tried, what we found, the plan (13 Nov 2026, 07:00)

From the lab chat. Every number below comes from a log or a motion-capture recording; the figures are in this package
(README.md is the map). One run per setting unless a count is given. "Twin" = Gazebo at 0.11-0.16 of real time.
"Replay" = your backend fed at REAL time with the recorded 12 Nov flights (open loop: a virtual QUAD4 follows your
reference exactly, no cable swing).

## The plan right now

**Today (Wesley, 06:31): row 1 = the drop into the MOVING figure-eight on the direct route** (your Python transfer both
ways, obstacle avoidance bypassed): `RET=direct bash ~/rig_fm9h8.sh`. Backup: the moving circle on the same route,
`RET=direct bash ~/rig_fm9hc.sh` (3 of 3 on 12 Nov). Wesley does not want the ring to stop for the drop-off.

**Your C++ planner today:**
- Over a STOPPED ring it works with the CPP=on line: outer_octagon, swing 5 deg, spline 4, carrier tracking 0.06, 30
  rendezvous-time steps, cooperative world refresh 0.2 s, fixed_splice_lookahead_s 0.20, search cap 2.9 m. Real-time
  replays 8 of 8; twin 11 of 12 full drop-offs (one stall from 5 o'clock). Command (Other options on the sheet):
  `CPP=on RET=direct PAUSE=takeoff bash ~/rig_fm9h8.sh`.
- Into a MOVING ring with that line (swing 5): it reaches the ring in replays (26-41 s at spline 4) but can stop in its
  path (no escape), and in the twin it never reached the moving figure-eight (R4440, 0.04 m from a cable). The scripts
  refuse CPP=on on the figure-eight/circle without PAUSE=takeoff unless CPPMOVING=1 is typed.
- YOUR TEST (swing 3 deg, spline 2.0 / 2.5, everything else as above), real-time replays so far: the moving circle
  caught in 8.7 s (spline 2) and 9.6 s (spline 2.5), the circle's moving window (fm9hc_4) in 8.1 s; QUAD4 at most
  2.57-2.79 m, at least 0.38 m from every carrier tether, no stop before the capture. Repeats, the still hover and a
  twin batch (moving figure-eight, circle, stopped figure-eight) are still running: see `swing3_spline2/` when added.
  Cautions: swing 3 deg is below the real cable swing (4-10 deg on 12 Nov, p95 8.8 deg straight after a lift; up to
  4.5 deg under your own planner, q4d); spline 3 earlier caught the ring fast once and then stopped in its path in 2 of
  3 repeats; replays have no swing and perfect tracking. Rig command if you and Wesley choose to fly it:
  `CPP=on CPPMOVING=1 SWING=3 SPLINE=2 RET=direct bash ~/rig_fm9h8.sh` (dry run checked: swing 0.05236 rad, spline 2,
  outer_octagon, cap 2.9, splice 0.20, refresh 0.2). Stop rule: QUAD4 still beside the moving ring -> our LAND at once.

**What would make it robust into a moving ring (for you to judge):**
1. The no-escape stop (mechanism 4 below): when the incumbent revalidates unsafe near a moving ring, fall back to
   something that keeps moving with the ring (e.g. hand back to the Python transfer, or a ring-frame hold) instead of
   UNCERTIFIED_HARD_HOVER at the measured position.
2. Predict each carrier's tether from the ring's own trajectory (plate endpoint), not frozen and slid along the
   carrier's body trajectory (your Codex, collision_model/).
3. Shorter sweeps of the moving hulls (spline factor or temporal subdivision) while the replan time stays under the
   splice (timing/fig2a).
4. Hand-over thresholds above QUAD4's real noise: settle speed (0.12 m/s, not 0.08), drift (0.30 m), the search floor
   at least 0.10 m under the hover height, the tracking tube (a 5.3 cm error tripped the 5 cm tube on 29 Oct).
5. Test path that worked tonight: the real-time replay harness first (`cpp_realtime_2026-11-13/full/run_flight.sh`,
   ~2 min per replay, no flight), then the twin, then the rig.

## The mechanisms (figures: README.md)
1. **Box shapes** (`legacy_square`, your default): each slanted carrier tether becomes a 0.44 x 0.25 x 0.93 m box;
   two boxes cover the middle of the ring up to 3.4 m, so the drop point is inside the obstacle set. QUAD4 parks on the
   box edge (~0.67 m short), replans end PREFIX_CORRIDOR_INCOMPATIBLE. Twin: 0 of 20 over a still ring.
2. **Real-time admission:** the collision world changed ~214 times/s and the octagon final recheck near the tethers
   takes 60-90 ms, so candidates ended WORLD_CHANGED_DURING_FINAL_RECHECK (0 of 4 replays captured); with refresh 0.2 s
   that goes away. Near the tethers a replan takes 100-220 ms, so with splice 0.10 most candidates are late, and under
   CPU load progress stops; splice 0.20 fixed it in the replays. The twin hid both because it runs 6-9x slower.
3. **Moving hulls close the pocket:** sweeping the carriers' tethers over each planning interval blocks the drop point
   in 26-44 % of circling samples at 1 s of sweep (60-83 % at 1.5 s) and lifts the lowest free height from 2.60 m to
   2.8-2.9 m.
4. **No escape:** late candidates -> the installed plan revalidates unsafe -> brake -> UNCERTIFIED_HARD_HOVER; a still
   QUAD4 beside a moving ring was swept by a carrier tether (0.00 m, centre lines) in 2 of 3 spline-3 replays.
5. **Rig thresholds (29 Oct):** sq3 never handed over (speed bound 0.08 m/s exceeded 22 % of the time; and the search
   floor 16-22 cm above its hover, 0 replans); sq4_5 under the 2.36 m floor 98 % of the hold; sq2_5 flew 8 s of C++,
   then a tracking-tube brake whose restart was blocked under the floor for 12 s.

## Everything tried
**Twin (Gazebo), 91 runs R4338-R4444** - every run with its settings and outcome: `overview/runs_table.csv`,
`overview/overview_all_cpp_twin_runs.png`; registry rows in `records/registry_rows.csv`.
- 12 Nov afternoon (R4338-R4363): your settings, your synced code; box shapes. 1 of 24 drop-offs completed (a circle).
- 12 Nov 20:00-21:00 (R4366-R4382): swing 3/5/7, workspace +-1.8 (no plan from a 1.9 m start), tracking 0.04, carrier
  body 0.09, spline 6; box shapes: still ring 0 of 7 (and 0 of 5 with the workspace), the circle 2 of 2 in slow time.
- 12 Nov night (R4383-R4392): outer_octagon; hover 4 of 4; moving: target_time_fixed_point_iterations 30 (6 left the
  moving rendezvous unconverged); circle/figure-eight completed in slow time at 3.2-3.6 m.
- 13 Nov 00:00-02:50 (R4393-R4422): the return to the start (RET=cpp/direct), the static target topic, return_climb,
  the 2.8/2.9 m caps (the moving ring stalled under 2.9), spline 3, tracking 0.04 (ball touched a cable), swing 3.
- 13 Nov 02:53-05:31 (R4423-R4444): the drop-off over a stopped ring, our LAND aborts, the exact rig line; R4423-R4434
  had your TAKEOFF before the path started (record R4423c); R4435-R4444 in the rig's order.
**Real-time replays (your backend, 12 Nov recordings), ~80 runs:** `cpp_realtime_2026-11-13/README.md` (tables),
`evidence_compact.md`, `zero_windows.md`; raw per run in `cpp_realtime_2026-11-13/full/runs/<run>/` (replans.csv,
backend.csv, driver.csv, backend.yaml); the harness `full/run_flight.sh`, `full/driver.py`.
**Rig:** 29 Oct sq2_5 / sq3 / sq4_5 (C++ with carriers, ring still); 12 Nov q4d_3 / q4d_4 / q4d_7 (QUAD4 alone: your C++
loaded transit worked). No C++ flight with carriers since 29 Oct.

## Changes made to your fork (all typed launch switches; nothing changes untyped)
`records/CHANGES_FROM_TEJEN.md` (every change, why, the tests) and the diffs in `records/diffs/`: swing_envelope,
target_time_iterations, cpp_max_z (maximum_search_z_m), cooperative_world_refresh_period_s (coalescing, in
transfer_backend_node.cpp), cpp_splice (fixed_splice_lookahead_s), temporal_refinement, return_after_drop /
return_climb, the static target topic, the lift cap, start_gate_min_traj_t (twin supervisor). Your Codex reviewed the
diffs (not built or run by it); its open points are in `records/codex_mailbox/`.

## Other records
- `records/fri_2026-11-13_sheet.md`: today's sheet.
- `records/decisions_12-13Nov.md`: our decision log for 12-13 Nov.
- `records/codex_mailbox/`: the whole exchange with your Codex (to_codex.md, from_codex.md, its review files).
- `records/hover_stall_2026-11-12/`: the first stall analysis and the probe source (probe.cpp).

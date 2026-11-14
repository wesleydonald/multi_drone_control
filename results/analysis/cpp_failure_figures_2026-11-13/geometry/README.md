# Geometry: what his collision model blocks, and why the drop point is (not) reachable (13 Nov 2026)

These figures are for Te-Jen. They show how `tejen_dynamic_planner` sees the three carriers' tethers, and why QUAD4
stalls short of the drop point with `legacy_square` but not with `outer_octagon`. Everything comes from saved outputs:
- the stall README and runs in `~/Downloads/tejen_hover_stall_2026-11-12/` (twin: R4374, R4377, R4378-R4382, R4362);
- the probe outputs on the real 12 Nov states in `results/analysis/cpp_realtime_2026-11-13/` (`out/`, `maps/`);
- run.csv for the octagon hovers R4383 and R4392.

None of his code was built or run for these figures. Fig 6 is my own arithmetic and is marked as an estimate. To
redraw a figure, run `python3 figN_*.py` in this folder. All runs are exploratory, one run per row.

## Fig 1: `fig1_free_space_stall_R4374.png`, his free space at the R4374 stall
This is the slice x = 0 through the ring at R4374's stall (twin hover, swing 5 deg, carrier tracking 0.06, t 54.83 s).
Red cells are QUAD4 body-centre positions that his `Separator` puts inside a carrier hull.
- **legacy_square:** every body height from 2.20 to 3.40 m is blocked from y -0.40 to 0.65 m, so the drop point is
  blocked too. QUAD4 stopped at y 0.677, on the edge.
- **outer_octagon:** the drop point is free at a body height of 2.90 m. At 2.75 m and 2.50 m the slice is blocked out
  to y 0.40-0.43 m.

The carrier tethers, the ring and QUAD4's hanging cable are drawn in (projected) to show that the map rows are body
heights: the object hangs 1.11 m lower, among the tethers. The full probe printout was not saved, so the panels are
redrawn from the rows quoted in the stall README. Legacy rows 2.20-3.40 m come from the README's statement that they
are all the same. Grey rows were not saved.

## Fig 2a: `fig2a_octagon_maps_static_vs_swept.png`, the pocket closes when the ring moves (real 12 Nov state)
This is his outer_octagon map in top view, centred on the ring. It uses fm9hc_4 at the settle instant (t 83.5 s, ring
circling at 0.12 m/s) and three body heights. The top row is the static world; the bottom row has the carriers' hulls
swept over 1.0 s.
- **Static:** the drop point has a free pocket of radius 0.08 m at body z 2.74-2.76 and 0.17 m at 2.90.
- **Swept 1 s:** the two hulls on QUAD4's side merge across the ring. The drop point is inside at 2.74 and 2.76, and
  the pocket at 2.90 is 0.04 m.

At a given body height, the gap between the hulls is the only way in.

## Fig 2b: `fig2b_pocket_metrics_vs_sweep.png`, all four 12 Nov flights vs sweep time 0 / 0.5 / 1.0 / 1.5 s
This uses `out/` (transit and settle samples) and `maps/`, all with outer_octagon and swing 5 deg.

| Measure | Ring still (fm9_5) | Ring circling (fm9hc_2/_4/_5) | legacy_square, every flight |
|---|---|---|---|
| Samples with the drop point blocked, static | 0 % | 0 % | 100 % |
| Samples with the drop point blocked, 1.0 s sweep | 0 % | 26-44 % | 100 % |
| Samples with the drop point blocked, 1.5 s sweep | 3 % | 60-83 % | 100 % |
| Lowest body height free over the drop point (median), static | 2.60 m | 2.60-2.62 m | >= 3.20 m (top of the probe's scan) |
| Same, 1.5 s sweep | 2.66 m | 2.80-2.88 m | >= 3.20 m |
| Pocket radius at body z 2.90, static -> 1.0 s sweep | 0.16 -> 0.16 m | 0.17 -> 0.04-0.12 m | |
| Pocket radius at body z 2.76, static -> 1.0 s sweep | 0.10 -> 0.08 m | 0.08-0.10 -> 0-0.04 m | |

A moving ring costs height and pocket. The longer the sweep interval, the higher QUAD4 must hang and the smaller the
target.

## Fig 3: `fig3_geometry_at_stall_R4374.png`, the real geometry at the stall (run.csv, t 54.83 s)
This shows the 3D scene, a top view and the real centre-line distances.
- **Carriers:** they sit 0.73 m out from the ring centre at z 2.12. Their tethers slant down to the plates (r 0.225,
  z 1.20).
- **QUAD4:** its body is at (0.006, 0.677, 2.899). Its object hangs 1.11 m below, at z 1.79.
- **Room at the object's height:** the tethers of drone_1 and drone_2 are 0.95 m apart (centre lines).
  - At the stall, the nearest tether is 0.61 m from QUAD4's cable or object.
  - With QUAD4 over the drop point at z 2.90, the nearest tether is 0.48 m away, at the object.
- **How legacy_square sees it:** each slanted tether becomes an axis-aligned box of 0.44 x 0.25 x 0.93 m (drone_0's is
  0.00 x 0.51 x 0.93). His model then adds the ego component's box and the tracking pads to that box. The gap between
  the tethers is physically wide; the boxes are what close it.

## Fig 4: `fig4_legacy_stall_vs_octagon_reach.png`, the same hover with legacy vs octagon
This compares R4374 (legacy) with R4383 and R4392 (outer_octagon) over the same hover from 9 o'clock. Swing 5, spline
4 and inflation 0.06 are the same in all three.
- **Octagon:** QUAD4 is within the 0.12 m capture radius after 10.3 s and drops the object at +24 s.
- **Legacy:** QUAD4 parks 0.665 m from the drop point for the remaining 212 s of the run.
- **Legacy variants:** R4378-R4381 (swing 3, inflation 0.04, carrier body 0.09, spline 6) have no run.csv. Their last
  replan starts (`initial_q0`) are 0.63-0.67 m from the drop point, the same edge.
- **Statuses:** R4374's replans ended PREFIX_CORRIDOR_INCOMPATIBLE in 74 % of cases (788 of 1061), ACCEPTED_PARTIAL in
  22 % and ACCEPTED in 2 %.

The octagon runs have no replans.csv saved, so no status shares are shown for them.

## Fig 5: `fig5_status_vs_hull_edge.png`, the failure starts at the hull edge
- **Left (R4374):** each replan's start-point distance to the drop point over time, coloured by status. Replans are
  accepted while QUAD4 closes in. Once it reaches 0.67 m (the legacy hull edge at x = 0, Fig 1), almost every replan
  ends PREFIX_CORRIDOR_INCOMPATIBLE. Its committed prefix sits on the hull, so the first-span corridor check finds no
  separating plane.
- **Right:** status shares for all eight README runs.
  - The four legacy hover variants look like R4374: 75-83 % PREFIX_CORRIDOR_INCOMPATIBLE.
  - The legacy circles (R4377, R4382, R4362) do not stall this way. They reached the drop point only because it moved
    under QUAD4.

Tuning swing, inflation, body box or spline does not move the edge. Each value was flown once.

## Fig 6: `fig6_clearance_budget_vs_swing.png`, an ESTIMATE of how the swing bound eats the room
This uses the R4374 twin geometry with QUAD4 over the drop point. The black line is the real centre-line clearance
from QUAD4's cable and object to the nearest tether. It grows about 0.48 m per metre of body height.

The dashed lines are the pads summed:
- the swing reach at the object, 1.11 m x sin(swing);
- the object's half-diagonal, 0.113 m (twin box 0.08);
- the carrier tracking pad, 0.06 m;
- the ego tracking pad, 0.05 m.

Where a dashed line crosses the black line is the estimated lowest free body height. Each degree of swing costs about
0.02 m of clearance, which is about 0.04 m of body height.
- **Swing 5:** the pads come to 0.32 m, which gives an estimated floor of 2.56 m.
- **Swing 10:** the pads come to 0.42 m, which gives an estimated floor of 2.76 m.

**His probe at swing 5 disagrees:** it blocks the drop point at 2.75 m and frees it at 2.90 m. His octagon hull is
therefore about 0.1-0.15 m wider (0.2-0.35 m of height) than these pads explain. Possible sources are the circumscribed
octagon's corners, the payload box's corners, or how the swing envelope covers the whole cable. He knows that code and
can check it. The README's 10 deg probe (the approach blocked at y 0-0.4) agrees with this direction.

## Fig 7: `fig7_which_pair_blocks.png`, which component pair blocks (real 12 Nov states, 282 samples pooled)
This shows which pair his Separator names when a point is blocked.
- **It is almost always a carrier's attached tether** against QUAD4's cable, magnet or payload. A carrier body appears
  only 27 times in all the saved outputs (`drone_0::body_vs_ego_cable`, fm9hc_5 at 1.0-1.5 s sweeps).
- **legacy_square:** the drop point is blocked in 100 % of samples, every time named as
  `drone_0::attached_tether_vs_ego_cable`.
- **outer_octagon, static:** the drop point is never blocked.
- **outer_octagon, 1 s sweep:** the drop point is blocked in 23 % of samples (against the magnet 13 %, the payload 10 %).

The probe reports the first hull that contains the point, in his obstacle-list order. drone_0 is probably over-counted,
and the other tethers may contain the point too.

## Surprising, for Te-Jen
1. **The room is physically wide.** Over the drop point, the nearest tether is 0.48 m from QUAD4's object, and the two
   side tethers are 0.95 m apart. legacy_square fills all of it because it treats a slanted 1 m tether as a
   0.44 x 0.25 x 0.93 m box and then adds the ego boxes and the pads.
2. **The free height window is narrow.** With outer_octagon and the twin's 0.08 m object box, the drop point is blocked
   at a body height of 2.75 m and free at 2.90 m (R4374 state). On the 12 Nov rig states it is free from 2.60 m. The
   rig and twin objects use different boxes, so the window depends on the object box he types.
3. **Small sweeps matter.** Sweeping a ring that moves at 0.12 m/s over 1-1.5 s raises the lowest free body height by
   0.2-0.3 m and blocks the drop point in up to 83 % of the samples.
4. **His octagon is more conservative than its own pads.** It blocks about 0.1-0.15 m more clearance than swing reach
   plus object plus tracking would (Fig 6, an estimate).

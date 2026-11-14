# Overview: every simulated drop-off with Te-Jen's C++ planner (12-13 Nov)

`overview_all_cpp_twin_runs.png` (made by `runs_table.py` -> `runs_table.csv`, then `overview_plot.py`; run from the
repo root / this folder). One dot per run (91 runs, R4338-R4444; five runs without an events file skipped). Rows group
the settings typed to his planner (collision shapes, still or moving ring, workspace box, spline_time_factor, height cap
maximum_search_z_m, fixed_splice_lookahead_s); x is his transit time from the hand-over to the drop point (SETTLE phase),
or the right-hand column if he never got there. Colour = what happened, marker = hover / circle / figure-eight.

How to read it:
- Box shapes (`legacy_square`, his default), still ring: 0 of 20 reached the drop point (15 stalled; 5 with the workspace box (+-1.8 m)
  never started: no plan from QUAD4's start, 1.9 m out). See ../geometry: with boxes the drop point is
  inside his obstacle set.
- Convex shapes (`outer_octagon`), still ring: every run reached it except one stall from 5 o'clock (R4433, 0.53 m short at
  2.87 m for 330 s; the same case R4425 reached it in 24.6 s, and R4437 with splice 0.20 in 25.6 s). Transit 9-26 s, QUAD4 at most 2.90 m.
- Moving ring: mixed, and these are SLOW-TIME results: the simulation ran at 0.11-0.16 of real time, so his planner had
  6-9x more computing time per simulated second than it gets on the real quads. Real-time behaviour is in ../timing
  (why plans get rejected) and ../stops_and_rig (where it stops in the ring's path).
- Grey = our LAND tests (deliberate aborts), black = R4438, QUAD4 crashed in its loaded climb before the C++ part.

`runs_table.csv` columns: settings from each run's launch line, the ring's speed when his transit started, whether his
C++ part started, whether it reached the drop point, transit time, QUAD4's highest body z in the transit, the closest
QUAD4 {body, cable, object} came to a carrier {body, cable} in the transit (centre lines), QUAD4's distance from the ring
centre at the end of the window.

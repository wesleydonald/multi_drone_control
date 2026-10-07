# Thesis figures and tables: plan (4 Oct 2026, overnight)

5 Oct 2026: the rig twin is the simulation. Every result flown on the legacy worlds or the SIL bench
is a `\redo` box in the thesis; what to fly for each is `docs/redo_on_twin.md`. Rows below that rest on
legacy or SIL runs are superseded by that list.

Rule: one figure per claim, built with the thesis-figure skill (print size, the RViz palette, no run ids;
provenance in `configs/thesis_figures.yaml`). Tables come from generators, never typed. Gazebo numbers
are means of 2 runs with n shown. Status: DONE / DATA (runs exist, figure to build) / FLY (runs needed)
/ RIG (Wednesday) / LATER.

## Methodology

| id | what it shows | form | status |
|---|---|---|---|
| M1 | nodes and links | TikZ | DONE (system_overview.tex) |
| M2 | controller cascade | TikZ | DONE (controller.tex) |
| M3 | geometry: ring, 12 plates and their numbering, one rod with its pivot offset delta, rho_i, q_i, the 45 deg working elevation; a 1/3/5/9 layout and the even triangle left after a detach | TikZ, two panels (top view, side view of one rod) | DONE (fig_geometry.tex, input in hardware.tex; local) |
| M4 | one flight from the floor: ring height, rod elevations, drone heights; phases (creep, pretension, lift, hold, LAND, unwind) shaded | time series, 2 panels | DONE (F_flight, in takeoff_landing.tex; critic applied; local) |
| T1 | rig and twin parameters: masses, rod, pivot, ring, plates, thrust map, horizons, rates | table | DONE (tables/params.tex from the rig profile, in hardware.tex; local) |
| T2 | cost weights | table | DONE |

## Simulation results

| id | claim | form | status |
|---|---|---|---|
| R1 | pinning node 0 removes the hover offset and most of the orbit error | 2 panels (SIL panel dropped 5 Oct) | DONE |
| R2 | the model integral rejects a constant force like the target shift, extends to x/y, learns the force | 2x2 | DONE |
| T3-T5 | initial condition; disturbance; gain and gate | tables | DONE (reviewed) |
| R3 | trajectory tracking, three and four UAVs: the circle and the figure-8, top view with the error along the path, and error vs time | 2 plan views + 1 strip | DONE (F_traj, in sim_results.tex; local) |
| T6 | the trajectory table of the outline: hover, line, circle, figure-8, spin; 3 and 4 UAVs; xy mean/max, z, tilt | table | DONE (tables/traj.tex, in sim_results.tex, replaces the old-box tab:four; line R1068/R1070, spin R1069/R1071 with a yaw note; local) |
| R4 | detach 4 to 3 by OCP resize: planned tensions per UAV, ring tilt with the hover band, height error | 3 stacked | DONE (F_detach, in sim_detach.tex; critic applied; local) |
| T7 | detach: peak tilt, time under 2 deg, height dip; OCP resize vs network hand-off | table | DONE for the resize (tables/detach.tex, in sim_detach.tex: 5 box-0 runs vs the R0939 rehearsal; local); the network A/B stays the September table |
| R5 | landing: straight descent against unwind + ramp + idle | 2x2: radial offset, signed lean | DONE (F_landing, in sim_results.tex; critic applied; local) |
| T8 | landing: straight / unwind / unwind + ramp + idle: runs, tipped (aborted), push, tilt | table | DONE (tables/land.tex; local) |
| R6 | attach during a circle on the 0.86 kg ring | storyboard (existing script) | FLY, after the attach work settles; the current figure is the 0.6 kg September run |
| T9 | robustness to a wrong belief: mass +-20 %, rod length +-5 cm, attach radius +-2 cm; hover height, tilt; with and without the integral | table | DONE (tables/robust.tex, in sim_robustness.tex; R1038-R1067; mass +-20/+10 %, rods +5 measured/not, radius +2; reviewer applied; local) |

## Real-world results (after 7 Oct)

| id | claim | form | status |
|---|---|---|---|
| W1 | rig hover and circle against the twin: same axes as R1/R3 | overlay | RIG |
| W2 | rig circle and figure-8 with the reference shifted by the mean offset (the 1 Oct figures, thesis treatment) | plan views | RIG (re-fly with box 0) |
| T10 | sim against real, one row per experiment | table | RIG |

## Left for later

- R6 (attach on the 0.86 kg ring) once the attach work settles; W1, W2, T10 after the rig.
- tables/four.tex is still generated (table_four) but no longer input anywhere.

## Order for tonight (economical)

1. The land_fix matrix (running), then R5 + T8.
2. R4 + T7 (resize part) from the land_fix detach runs.
3. Fly the four-UAV figure-8 (box 0, x2), then R3.
4. M3 (TikZ, no runs).
5. If time remains: T9 runs.

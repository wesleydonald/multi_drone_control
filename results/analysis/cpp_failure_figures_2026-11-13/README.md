# Why Te-Jen's C++ planner fails: figures (13 Nov 2026, for Te-Jen)

Every figure is made from your planner's own logs (replans.csv, backend.csv) or from motion capture, and has a script
next to it. Each folder has a README with one paragraph per figure. Data: the twin (Gazebo, 0.11-0.16 of real time),
open-loop real-time replays of the real 12 Nov flights through your backend (a virtual QUAD4 follows your reference
exactly, no swing), and the real flights of 29 Oct. Your Codex's source walk-through of the collision model is in
`collision_model/` (function names, equations, what is left out).

Start with `overview/overview_all_cpp_twin_runs.png`: all 91 twin runs, settings vs what happened.

## The failures, by mechanism

**1. Box shapes put the drop point inside the obstacle set (still ring stalls ~0.67 m short).**
`legacy_square` turns each slanted carrier tether into a world-axis box (0.44 x 0.25 x 0.93 m); two carriers' boxes
cover the whole middle of the ring at every height up to 3.4 m. QUAD4 parks on the box edge and every replan ends
PREFIX_CORRIDOR_INCOMPATIBLE. Swing, inflation, body box and spline do not change it (each tried once).
`outer_octagon` keeps the slanted prism and frees the drop point from ~2.6-2.9 m.
-> geometry/fig1, fig3, fig4, fig5, fig7; overview.

**2. At real time your plans are thrown away before they are installed.**
(a) Every carrier/plate/ring message changes the collision world (~214 versions/s); with outer_octagon the final
recheck near the tethers takes 60-90 ms, so almost every candidate ends WORLD_CHANGED_DURING_FINAL_RECHECK.
Coalescing the cooperative world to 0.2 s (`cooperative_world_refresh_period_s`) removes it.
(b) Near the tethers a replan takes 100-220 ms; with `fixed_splice_lookahead_s` 0.10 most candidates are late, and
under CPU load progress stops (last install at 4.8 s, nothing for 43 s). At 0.20: 0 late in the load runs.
-> timing/fig1a, fig1b, fig2a, fig2b, fig3, fig6, fig7.

**3. A moving ring closes the free pocket.** The carriers' tethers are swept over each planning interval (hull of the
committed body curve, the tether shape frozen and slid along it; equal-time pairing lost). At 1 s of sweep the drop
point is blocked in 26-44 % of the circling samples and the lowest free height rises from 2.60 to 2.8-2.9 m.
Shorter intervals (spline factor) or temporal subdivision shrink the sweep.
-> geometry/fig2a, fig2b; collision_model/.

**4. No escape: it stops in the moving ring's path.** When the installed plan revalidates unsafe and no new plan is
installed in time (late candidates), the backend brakes; if the brake is unsafe it publishes an UNCERTIFIED_HARD_HOVER
at the measured position. A still QUAD4 beside a moving ring is then swept by a carrier tether (0.00 m twice). This is
not a margin. A fallback that keeps moving with the ring (e.g. a hand-back to the Python transfer) is a PROPOSED design,
not an existing fix: it would need an admitted p/v/a hand-over and a collision-safe continuation (your Codex, entry 23).
-> stops_and_rig/fig1 (top-view snapshots), fig2, fig3 (the same settings capturing in time), fig4; timing/fig5.

**5. On the real quads (29 Oct) the hand-over thresholds sit inside QUAD4's real noise.** sq3: settle speed bound
0.08 m/s, QUAD4 above it 22 % of the time, and the search floor 16-22 cm above its hover (0 replans); sq4_5: QUAD4
under the 2.36 m floor 98 % of the hold; sq2_5: 8 s of C++ flight, then a 5.3 cm tracking-tube violation (5 cm bound)
and a brake whose restart was blocked under the search floor for 12 s.
-> stops_and_rig/fig5, fig6, fig7.

## Parameters and what each one moves
| parameter | what it changes | seen in |
|---|---|---|
| swing_envelope_shape | boxes vs slanted prisms: the still-ring stall | geometry fig1, fig4 |
| max_swing_angle_rad | ~0.02 m clearance (~0.04 m height) per degree; the real cable swung 4-10 deg on 12 Nov | geometry fig6 |
| cooperative_tracking_half_m / ego | added to every pair (0.06 + 0.05); 0.04 gave a ball-cable touch in the twin | geometry fig6 |
| spline_time_factor | the planning interval, so the sweep of moving hulls; faster capture, but spline 3 stopped in the path 2 of 3 | geometry fig2; stops fig1-3 |
| fixed_splice_lookahead_s | how late a plan may be; 0.10 too short near the tethers | timing fig2a, fig2b, fig6 |
| cooperative_world_refresh_period_s | how often the world may change under a recheck | timing fig3 |
| maximum_search_z_m / minimum_search_z_m | the height window; the floor blocked the 29 Oct flights | stops fig6, fig7 |
| c1f2 prepare bounds (speed, drift) | the hand-over gate; QUAD4's real hover exceeds 0.08 m/s | stops fig5 |

## Your test of 13 Nov: swing 3 deg with spline 2.0 / 2.5
See `swing3_spline2/` (added when the runs finish, ~07:15).

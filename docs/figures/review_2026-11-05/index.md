# Review figures, 5 Nov 2026 (quality pass on every Results figure)

Wesley 5 Nov: figures "extremely high quality and elite". Each script is a copy of the earlier review version with the
changes below; data and numbers unchanged. Not in the thesis until Wesley has looked (the preview PDF uses them).

| Figure | From | What changed |
|---|---|---|
| fig_release_rig | review_2026-10-24 | panel labels; the band (highest tilt in the 5 s before + 0.5 deg) and the 1.8 s back in it drawn; the detection (+0.42 s, events.csv) marked; legend "Remaining / Released quadrotor"; height ticks without trailing zeros |
| fig_catch_offset_explained | review_2026-11-02 | (a) redrawn as a top view of the whole ring (nothing clipped, no label on a plate), legend "Plate centre / Measured catch point", y "Ring tilt peak" |
| fig_solve_time | review_2026-10-30 | real world red, simulation blue (as fig_summary); "Number of quadrotors" |
| fig_s8_chain, fig_s8_events, fig_s8_laps | review_2026-10-30 | "Quadrotors", "Distance from hover point", "blend" for the fold-in band, capitalised legend entries |
| fig_summary | review_2026-11-02 | F031 simulation means; value labels 8 pt (audit FAIL at 6.5 pt fixed); "Number of quadrotors" |
| fig_c_time | review_2026-11-02 | legend "Network of Quan et al." |
| fig_traj | review_2026-11-02 | panel names "n quadrotors" (needs ROS sourced: rosbag2_py) |

Not redrawn: fig_rejoin (2 Nov) keeps "Carriers" and "staged": its cable-end series came from the run's Gazebo state
log, deleted with the replays on 5 Nov (F048). It changes when the run is re-flown on the final tree.

## Round 2 (after a blind critic of the nine PNGs)
One colour rule across the set: red = real world or the baseline, blue = simulation or the proposed method, grey = context.
- fig_release_rig: remaining quadrotors grey; t = 0 named "magnet off" (the RELEASE event), detection 0.42 s; the band labelled
  "highest tilt before + 0.5 deg" and the arrow "recovery, 1.84 s" (fig_reaction.py's measure, called recovery time in the
  text); height error in mm; tighter y limits.
- fig_catch_offset_explained: legend "Folded in at the plate centre / at the measured catch point"; "Peak ring tilt"; the
  next plate's centre (11.6 cm) marked in (b); labels moved off the ring.
- fig_solve_time: "32 % of solves over 60 ms". fig_s8_chain / fig_s8_events: "Time after ..." labels, "Carrying", wider
  shade steps. fig_s8_laps: no scale bar (an oblique view has no single scale). fig_summary: one shared y axis, "RMS
  error", panel titles 9 pt. fig_c_time: "Position along / Distance from the line", ticks 0, 0.5, 1. fig_traj: one red
  (real world) for every panel, no unexplained dot, legend Ring / Reference.
Rejected: the tilt dips to 0 at releases in fig_s8_chain are real (the tilt vector swings through level); 81 against 83 mm
is two runs (the chain shows one); fig_c_time's network starting 18 mm off is as flown.

## New headline figures (5 Nov evening, Wesley: "do that then push")
- fig_x1_resize (7 Nov, both mass splits, Wesley F089): (a) ring tilt after the release, four in hover, 0.86 kg ring,
  measured split (R3873-R3876), resize at detection (blue, solid) against none (red, dashed), the larger second swing at
  +2.1 s marked; (b) settled tilt per run, four in hover leaving a triangle, 0.86 and 1.15 kg rings, both splits
  (x1_2026-11-06.json, x1_oldlinks_2026-11-07.json); (c) the square leaving a T (integral off), both splits. Values match
  results/analysis/x1_both_models_2026-11-07.md. Blind critic 7 Nov: combined legend handles, splits named, panels named,
  one style per arm; rejected a longer caption (Wesley: very short) and dots-only (bars plus dots show n = 2).
- fig_x4_relay: (a) each quadrotor's timeline in relay R3519 from take-off to touchdown (carrying blue, away grey,
  recharge ticks from the plan's 24 pad leaves; Q3's two errands are away without a tick), the fixed team R3566 in red
  above; (b) contour error per lap for R3519, R3564 (blue) and R3566, R3567 (red). Both re-run on the weekend re-fly.

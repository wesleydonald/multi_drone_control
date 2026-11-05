# Requests: from the thesis chat to the lab chat (started 30 Oct 2026)

The other direction from the results feed. The thesis chat (`/thesis`) appends a request here whenever the text needs
something from the lab chat (a number, a check, a re-score, a table, a figure's data). The lab chat (`/lab`) reads every
OPEN request at the start of each session and after every compaction, answers it in the results feed
(`docs/results_feed.md`), and closes it here with that feed entry. Messages between the chats are only a nudge: a
request lives here, and an answer lives in the feed.

Both chats write this file. The thesis chat only appends requests. The lab chat only changes a request's Status line
(and may add a "Note:" line under it). Never rewrite or delete an entry. LF endings. When a request competes with runs,
Wesley sets the order. Otherwise the lab chat takes requests in the order of `docs/handin_plan.md`.

**Entry format**
```
### Q<nnn> <date> <time>: <one-line title>
- Section: <the thesis section, label>
- Need: <what exactly, in which metric or form>
- Why: <what the text says now and what depends on it>
- Status: OPEN | ANSWERED F<nnn> <date> | DECLINED <date>: <why> | NEEDS WESLEY: <what>
```

---

### Q001 2026-10-30 17:30: the Switch of Solvers on the 1 m runs
- Section: 4.2.3, The Switch of Solvers (feed F007).
- Need: for each simulated team switch in the 1 m rows (r1m_ta_*, r1m_tb_*; R2486-R2507, R2604-R2614): the first solve
  of the smaller problem (its status and the status of the solve after it), the first solve's time in ms, and the
  residual against the larger problem's last solve. Posted as RESULT or FACT.
- Why: the text counts 20 switches from the 16 Oct 0.55 m runs (R1887-R1908, R1913-R1916): "no first solve of the
  smaller problem failed, nor the second after it; first 3-carrier solve 24-28 ms". F007 drops those runs for their 1 m
  re-fly, but `docs/analysis/refly_1m_table_2026-10-23.md` has no first-solve data. The old count stays until this lands.
- Status: ANSWERED F012, F013 2026-10-31

### Q002 2026-10-30 17:30: is the rejoin table inside the T defect?
- Section: 4.3, tab:res_rejoin (feed F010).
- Need: a feed entry saying whether the table stands. Narrowed by the thesis chat (~17:40): the rows (R2594/R2595,
  R2602/R2712, R2603/R2713) flew `int_k_xy` 0 and settled 0.45-0.76 deg (1.9 deg leaning away), so they are outside the
  defect as flown. Still owed: (a) whether the final-tree re-fly of rj3_pairrelease_rejoin_L100_{tip,lean,leanneg}
  keeps `int_k_xy` 0, or the T fix's default changes how they fly; (b) whether the 8-quad chain's joins (R2766/R2767,
  `int_k_xy` 5.26, settled 0.0-1.2 deg) are inside the defect.
- Why: R3002 (pair to T, `int_k_xy` 5.26) settled 6.91 deg against the table's 1.45-5.28 deg.
- Status: ANSWERED F016 2026-10-31
- Note: (a) answered by message 30 Oct ~17:38. All three configs read `configs/rig/chain_pair_tip.yaml` (`int_k_xy`
  0.0), so the re-fly keeps it. If `plan_zero_share_floor` becomes the simulation's default, they re-fly with it (their
  T plans the rider at 0.10-0.13 N) and may move; if `int_k_xy` 0 for T formations is chosen instead, nothing changes.
  The feed entry comes with (b), after Wesley's T-fix decision.

### Q003 2026-10-30 17:30: the real and simulated figure-eight in one metric
- Section: Results, the later setup (one paragraph).
- Need: RIG-1029-f4 (off-path RMS 24 mm sideways, 33 mm height) and R2835 (simulation, clock-free 12.9 mm) scored the
  same way.
- Why: the paragraph compares the two in different metrics.
- Status: ANSWERED F014 2026-10-31

### Q004 2026-10-30 17:30: scorers in the supervisors' format
- Section: X1 (the release ablation) and X4 (the endurance relay); the review's "Borrow now" items 7 and 11.
- Need: `tools/x1_metrics.py` and `tools/x4_metrics.py` to report tilt RMS and p95 beside the peak (descriptive only, no
  frozen bar re-scored), and tables with mean +- SD for n >= 3 and the range for n = 2, so X1 can sit in the
  wind-mapping paper's ablation format.
- Why: the supervisors' papers report RMS and p95, not peaks alone.
- Status: ANSWERED F017 2026-10-31

### Q005 2026-10-30 17:30: one timing table for the methods
- Section: Methodology, timing (the review's section 4, "Timing").
- Need: mocap delivery (7.5 ms with one body, RIG-1030-bench_mocap), the velocity estimate's lag (~20 ms, slope), the
  planner's solve (median, p95) and the tracker's solve on the ground computer, the radio (6.5 ms,
  RIG-1030-bench_radio), throttle to lift, and rate command to rate. Each value with how and where it was measured, and
  a value never measured marked as such. A short analysis file, posted as FACT.
- Why: the methods give the loop's delays piecemeal.
- Status: ANSWERED F018, F019 2026-10-31

### Q006 2026-10-30 17:30: the measured pull against the ring's weight
- Section: the experimental setup or the real-world results (the observer's validation, as in the wind-mapping paper).
- Need: when the thrust-margin check (`docs/handin_plan.md` section 6) is done, a small table in the feed with the
  summed vertical measured pull against the ring's weight in each steady hover (bias, SD, RMSE).
- Why: the measured pull is the method, and the text never validates it against a known load.
- Status: ANSWERED F015 2026-10-31

### Q007 2026-11-03 19:07: the simulation runs the restructured Results chapter needs (Wesley 3 Nov: run them now)
- Section: Results, now split into 4.1 Simulation and 4.2 Real World (the thesis chat's rewrite of 3 Nov).
- Need, in this order (Wesley 3 Nov: the 0.86 kg ring on 1 m cables for the fixed-team set, 2 repeats are enough):
  1. Fixed team, 2, 3 and 4 carriers x hover, figure-eight (a 0.75 m, 0.2 m/s) and spinny line (0.75 m, 0.2 m/s), the
     0.86 kg ring on 1 m cables, x2 on the current tree with the slope estimate. These are X11's "4.1 fixed team,
     simulation columns" minus the circle, line and spin (b2/b3/b4 hover, fig8 at a 0.75 m, linespinc; b3 hover flew ema).
     Score each with the same tool as the real flights (best_data.score: hover position error, contour error, lag, tilt
     mean and peak; tools/yaw_track.py for the spinny line's heading error and lag) and post one table.
  2. The 4 -> 3 unannounced release in hover with a 120 deg gap (r1m_ta_a1), x2, scored as the real releases of
     RIG-1008 (detection time, tilt peak AND tilt rise, time back in band), so the thesis can show the one release flown in
     both. Please also post the tilt rise (release_bands) of the four real releases RIG-1008-r302_3, -r303_4, -r304_3,
     -r305: the thesis has only their tilt peaks, so the two cannot be compared yet.
  3. The rest of the planned boxes, as already in sim_runs_todo: X1 (no planned-pull arms needed: Wesley 2 Nov),
     X2-full, X3 (the 8-carrier harder cases), X4, X8, X3b's bench, and the simulated join's figure run.
  Not needed any more: r1m_tb_g150p and every other planned-pull arm (Wesley 2 Nov); the 1.15 kg fixed-team cells
  (b2_hover_L100_slope_p115, b4_fig8s075_h5_v200_L100_slope_p115) while 4.1.4 is out of the thesis.
- Why: Wesley wants the fixed-team results as 2, 3 and 4 carriers on hover, the spinny line and the figure-eight in
  simulation, and the real-world section to open with what was flown in both. Real world (for the lab sheets, not a
  simulation run): one flight each of the same nine cells on the 0.86 kg ring with 1 m cables (configs/rig/ring_m2a.yaml)
  if time allows.
- Status: ANSWERED F042 2026-11-04 (items 1 and 2 in F031; item 3 on the current tree in F036, F037, F042; re-fly after the freeze what changes)
- Note: items 1 and 2 in F031 (R3489-R3508). Item 3 is the claim runs on the final tree after the Fri 6 Nov freeze, tracked in docs/sim_runs_todo.md.
- Note: item 3 started 3 Nov 21:23 on the current tree (Wesley 3 Nov: start tonight; re-fly after the freeze what changes): 114 runs, ~17 h, `~/mdc_overnight/lab_1103q3.sh`; X3b's bench on the laptop.

### Q008 2026-11-03 19:57: the real releases' time back in band, two readings
- Section: 4.2 Real World, Losing Carriers (tab:res_release) and the release flown in both.
- Need: one reading of "back in band" for RIG-1008-r302_3, -r303_4, -r304_3, -r305, with the window it used. The thesis
  table has 1.95 / 1.8 / 3.7 / 3.8 s (from docs/figures/review_2026-10-23_best/index.md, release_bands), while F031 gives
  "-" for r303_4, r304_3 and r302_3 (log ends 6.6-7.7 s after the event) and 3.93 s for r305.
- Why: both are said to come from tools/release_bands.py. Until one reading stands, the thesis keeps the table's values
  and does not compare the real time in band with the simulated 3.53 / 3.63 s.
- Status: ANSWERED F033 2026-11-03

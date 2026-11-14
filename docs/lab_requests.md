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

### Q009 2026-11-09 21:24: X1's second swing in layout (e), read from the end of the first swing
- Section: 4.1.2 Losing Quadrotors in Simulation (tab:res_x1), the harder detachment of layout (e) (sqTnoxy, R3893-R3902).
- Need: check and record in the X1 analysis the second swing of the sqTnoxy runs, read as the largest tilt from the end of
  the first swing (its first minimum after the peak) to +4 s. The thesis now gives the two-run means: Baseline 9.9,
  Proposed 22.4, 0.5 s late 14.4, 1 s late 15.4, 2 s late 10.4 deg (docs/figures/review_2026-11-05/tab_x1.py; per run
  9.45/10.34, 23.44/21.27, 14.60/14.11, 15.68/15.13, 10.31/10.50; the first swing ends at +1.4-1.6 s). This reading equals the
  +1 to +4 s window (F091) in all 20 h4 and f4 runs; that window gave n/a for layout (e) (35-44 deg at +1.0 s).
- Why: Wesley 9 Nov asked why the table had N/As; the thesis states that the resize at detection more than doubles the
  second swing in layout (e), which no lab record covers yet. Say in the feed if the reading or a number is wrong.
- Status: ANSWERED F110 2026-11-13 (thesis numbers stand; the comment's 35-44 deg at +1.0 s should read 41-44 deg)

### Q010 2026-11-09 22:34: the harder detachment's tilt while away, one definition in both; the re-attachment time of sq8_2
- Section: 4.2.1 (tab:res_both, the harder detachment of layout (e)) and 4.2.3 (the first real re-attachment).
- Need: (1) check and record the ring's MEAN tilt from the magnet off to the re-attachment: RIG-1105-sq8_2 15.6 deg
  (median 13.3; magnet OFF 1793832639.09 to the second FLEET RESIZED n=4 1793832660.63 in sq8_2.log, 21.5 s; the
  events file's DETACH is the magnet off, the log puts the ring at 1.4 deg there), and R3708, R3712, R3714, R3715
  9.13 / 11.93 / 11.55 / 12.33 (RELEASED to REWELD; their medians 8.49 / 11.43 / 11.19 / 12.02 reproduce F049), mean of
  the four 11.2. The thesis now gives 15.6 against 11.2 (Wesley 9 Nov: averages, not ranges); F049's "21-28 deg" was
  the later part of the window (the real tilt rose from about 7 to 29 deg while away). (2) Confirm the re-attachment
  came 14.4 s after the partner release (1793832646.25) and 21.5 s after the magnet off; the thesis now says "14.4 s
  after it was handed back to the collaborator's controller".
- Why: the thesis replaced the ranges with averages; these two are read from the logs by the thesis chat
  (docs/figures/review_2026-11-05/tab_averages.py), not from a lab record. Say in the feed if either is wrong.
- Status: ANSWERED F109 2026-11-13 (thesis numbers stand; the real median is 13.2, not 13.3)

### Q011 2026-11-09 22:55: re-fly the simulated attachment (4.3 joins) on Setup 3's ring, NOW, on the lab PC (Wesley)
- Section: 4.1.3 Attaching Agents (tab:res_join, fig:res_join_sim).
- Need: re-fly the 7 join cells of chunk 8b (docs/claim_batch_2026-11-07.md: sq_pickup_cppreattach_bd_noxy (over),
  sq_floorjoin_noxy_low, sq_floorjoin_v1a30s_bd_noxy, fri30_sq6b_g_frz, fri30_sq6b_frz, fri30_sq6b_g, fri30_sq6b), x2
  each (14 runs), on the 0.86 kg M2A ring (Setup 3's ring) instead of the 1.15 kg ring: worlds
  three_rigid_ground_rig_attach_pickup_L100.sdf / sq_rigid_ground_rig_attach_pickup_L100.sdf (not *_p115), the
  planner's load mass, inertia and attach geometry for the M2A ring (configs/rig/ring_m2a.yaml values) with its solvers
  prebuilt, everything else as R4094-R4107 (final tree and masses, int_k_xy as flown per cell). Score them as F080/F081
  (tools/attach_clearance.py: his TAKEOFF to the weld, tries/contacts; the fold-in peak absolute tilt after the weld as in
  results/analysis/x11_joins_2026-11-07.json), and name the run that replaces R4106 (layout (d), plain fri30_sq6b) for
  fig:res_join_sim. If a cell needs more than the ring changed to fly on the lighter ring, say so before flying it.
- Why: every run behind tab:res_join and fig:res_join_sim (R4094-R4107) flew the 1.15 kg ring (worlds *_p115), set up
  on 28 Oct when the rig carried that ring and carried unchanged into the 7 Nov batch; the claims' rule is the 0.86 kg
  ring, and the real attachments (4.2.3) flew Setup 3. Wesley 9 Nov: "lets do that right now ... (on the lab pc)".
- ADDENDUM 2026-11-09 23:09 (Wesley: Figure 15 must show the steady state, with no landing in it): the scenario lands
  10 s after the weld (LAND at t 30 after WAIT_WELD at t 20), which is exactly the fold-in blend (attach_traj_hold_s
  10), so R4094-R4107 land the moment the blend ends and never show steady tensions. Hold longer before LAND: LAND 30 s
  after the weld (the blend's 10 s plus 20 s steady) in every cell of the re-fly, or at least in the run that replaces
  R4106 (fri30_sq6b_m2a). Say if the welded new agent cannot hover that long in the twin. The fold-in tilt for the table
  stays read from the weld over the same window as F080 (the weld to +15 s), so the longer hold does not change it.
- Answer: a feed entry with the per-run table (run ids, takeoff to attachment, contacts, ring tilt after) and the run
  for the figure; the thesis chat redraws Figure 15 and updates Table 14 and the prose.
- Status: CLOSED 2026-11-10 00:06 by feed F097 + F098 (the 14 runs R4214-R4227 on the 0.86 kg ring, plate worlds, LAND +30 s; Figure 15: R4227)

### Q012 2026-11-13: the ring's paths of the X11 U runs (R4034-R4093) for Figure 18, as an analysis file
- Section: 4.1.3 Attaching Agents (fig:catch_paths, docs/figures/review_2026-11-05/fig_catch_paths.py).
- Need: one compact file, `results/analysis/x11_u_paths_2026-11-07.csv.gz`, with columns run, t, payload_x, payload_y,
  payload_ref_x, payload_ref_y: every row of each run's logs/run.csv (deduplicated on t) from its PATH_START to its
  LOGGED event (events.csv sim_time), for all 60 runs R4034-R4093 (x11_u_2026-11-07.json). Nothing new is flown or scored.
- Why: the run folders of 7 Nov are no longer on the laptop (moved off with the 6 Nov ones), and the figure must be
  redrawn (review fix 16: one purple scale instead of blue to red, since red means the plate's centre in Figure 17,
  fewer words, the clipped panel titles). The script will read this file instead of the run folders.
- Answer: a feed entry naming the file; no thesis numbers change (the printed RMS per case should match F078/F079).
- Status: ANSWERED F112 2026-11-13

### Q013 2026-11-13 19:10: the real-world tables for the restructured Results, event by event, Setup 3 (Wesley)
- Section: 4.2 Real World, rebuilt (Wesley 13 Nov): Results and Discussion 16-18 pages with at least 12 on the real
  world, told event by event up to the full mission; no setup explanations in the results (Setups 1 and 2 move to the
  Methodology as lessons). Wesley's framing: the DROP shows the proposed planner's disturbance rejection and recovery
  (the drop-off itself is credited to the collaborator's controller, his Python transfer and his C++ planner); the
  REATTACHMENT shows how resizing works (the attachment itself is credited to his single-quadrotor controller).
- Need, each as a reviewed table with one row per flight (run id in a column, the thesis drops it), every Setup 3 flight
  counted, judged as is (no bars), from the session sheets and paper_rig's rules where they exist:
  1. Planned detachments, static and moving: tab:detach_performance's rows per flight (time to resize, swing, settle,
     height dip, horizontal offset), now including 12-13 Nov (every fm9*/nr* detach), plus the two Setup 1 unannounced
     detections already in tab:detach_context_0p55 (RIG-1008).
  2. Reattachment, before and after the funnel: descents, contacts, proven, held; and the resizing at each proven
     attachment: contact -> proof -> fold-in start -> blend end times, the four tensions through the blend, the ring's
     tilt peak, height dip and horizontal offset from the fold-in to +15 s.
  3. The drop as a disturbance: every drop into the net (hover and circle with his Python transfer, 11-12 Nov; the C++
     planner drop-offs of 13 Nov on the circle, figure-eight and hover): object's distance from the ring centre, the
     ring's speed at the drop, and the ring's tilt peak, height dip, horizontal excursion from its reference and time to
     recover (say the recovery rule used) from the release to +10 s; plus the pickups as counts (tries, lifted, held).
  4. The full mission: RIG-1112-fm9_5's phase times (takeoff, detach, pickup tries, lift, drop, re-contact, proof,
     fold-in, LAND) with the number of quadrotors the planner flew in each phase; fm9hc_2/_4/_5's moving-ring half; and
     any full mission flown on 18 or 19 Nov (added when flown).
- Also: time series for the figures, extracted like the 12 Nov `*_poses.csv` (tip, object, ring, quads, tensions,
  phase): RIG-1111-nrtfc_2 and -nrtfc_5 (the reattachment timeline), and confirm which run.csv/poses files hold
  fm9_5's whole flight.
- Why: the paper tables (results/analysis/paper_rig_2026-11/) stop at 11 Nov; the 12-13 Nov flights are in the feed
  (F103, F104, F111) but in no table, and the thesis may only cite reviewed rows.
- Answer: feed entries naming each table file (results/analysis/...), one per item as each is ready; items 3 and 4
  first if you must order them (the drop and the full mission are new to the thesis).
- ADDENDUM 2026-11-13 20:20 (thesis chat, for the drop figure): one CSV of the ring's response around every drop in
  drop_disturbance.csv (the 13 in the net, the C++ fm9hc that missed, and k1ob_2 / k1zo), aligned at the release: columns
  flight, day, t_from_release_s (-5 to +10, the 50 Hz grid), ring_tilt_deg, ring_dz_mm (height minus the 5 s pre-release
  mean), xy_err_cm (horizontal distance from the reference), ref_speed_mps. Most 13 Nov run folders have no TAKEOFF in
  events.csv, so the thesis chat cannot align run.csv with t_release_s safely itself.
- Status: ANSWERED F113-F122 2026-11-13 (items 3-4: F113-F118, reviewed F116; items 1-2 and the time series: F119-F121, reviewed F122)

### Q014 2026-11-14: put back the run folders of four thesis figures (relabel only: "Ring" -> "Payload", "Real world" -> "Experiments")
- Figures (docs/figures/review_2026-11-05/): fig_s8_chain.py (R3972), fig_s8_laps.py (R3924), fig_x4_relay.py (R3910, R3911,
  R3913, R3914), fig_solve_time.py (R2764-R2779, in results/archive/2026-10-2*.tar.zst; it reads logs/dissipative_planner).
- Need: those run folders back under results/<day>/ (or at least the logs each script reads), so the thesis chat can
  re-render the four figures with the new labels (Wesley 14 Nov, decisions DECIDED 2026-11-14: "payload" for the ring in
  general statements and measures, "experiments" for "real world"). No numbers change.
- Answer: a feed line when they are back; the thesis chat re-renders and says when they can go again.
- Status: CLOSED 2026-11-14 11:44 by F123 (figures re-rendered by the thesis chat; the restored copies removed again)

### Q015 2026-11-14 12:25: the run folders of four figures once more (panel titles beside the letters, Wesley 14 Nov)
- Figures: fig_s8_chain (R3972), fig_s8_events (R3972, R3973), fig_x4_relay (R3910, R3911, R3913, R3914), fig_c_time
  (R4112, R4124, R4125).
- Need: those run folders back under results/<day>/ for one re-render each (Wesley 14 Nov: each panel's title goes in the
  figure beside its letter, capitalised, as in the supervisors' papers). No numbers change. Removed again afterwards.
- Status: CLOSED 2026-11-14 12:26 (figures re-rendered by the thesis chat; the restored copies removed again)

### Q016 2026-11-14 15:10: the run folders of four figures once more (in-plot text black and capitalised, Wesley 14 Nov)
- Figures: fig_s8_chain (R3972), fig_s8_events (R3972, R3973), fig_x4_relay (R3910, R3911, R3913, R3914), fig_solve_time
  (R2764-R2779, results/archive/2026-10-23.tar.zst; it reads logs/dissipative_planner).
- Need: those run folders back under results/<day>/ for one re-render each (Wesley 14 Nov: no grey text or arrows in any
  figure, every label starting with a capital). No numbers change. Removed again afterwards.
- Status: CLOSED 2026-11-14 16:23 (re-rendered, Wesley approved, thesis 8c81c7d; the restored copies removed again)

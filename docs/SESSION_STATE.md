# Session state (the LAB chat reads this first, via `/lab`; keep it short and current)

Updated: Sat 14 Nov 2026, 13:42 AEDT. Branch **week8** at **bf52a8a** (the code
freeze); everything since is UNCOMMITTED: src/ (tejen_mission, dissipative_planner, drone_communication, drone_magnet,
bringup, simulation_communication), tools/ (console, experiment runner, mocap_stream_check, rig_cable_check,
frozen_pose_desk_check), configs/experiments/rig_*_twin, docs, registry, results/rig/2026-11-1x scripts. Commit only on
Wesley's word (never offer).

## NOW (Sat 14 Nov 13:42): next week's plan (Wesley 14 Nov, decisions DECIDED 2026-11-14; CLAUDE.md updated)
- Sessions 08:00-11:00: Mon 16 (Wesley: detach/re-attach x3 hover, figure-eight, circle; one 4 -> 2 -> 4; sheet
  docs/rig/mon_2026-11-16.md, scripts results/rig/2026-11-16/scripts/), Tue 17 + Wed 18 (Te-Jen: drop-off x3 hover,
  circle, figure-eight, then hero; sheet docs/rig/tue_2026-11-17.md), Thu 19 + Fri 20 (gaps, the best hero). Thesis takes
  flights up to Thu 19; hand-in Fri 20 17:00.
- DONE 14 Nov: twin regression of Monday's lines after the merge (R4474-R4476: no regression; the re-weld timeout is the
  old twin artifact); runtime desk check of 4 -> 2 -> 4 PASS on wiring (decisions FINDING 14 Nov; CPU 8.5/12 on the ground:
  sheet closes RViz for row 10); ~/rig_424.sh copied; the figure-eight rows press his TAKEOFF ~5 s after the release (a
  carrier passed 0.37 m from the waiting QUAD4 in the twin, R4402); the note for Te-Jen sent to Wesley
  (docs/rig/for_tejen_2026-11-14.md + zip: his OK for QUAD3, his descent, one C++ line, the paper wording).
- WAITING on Te-Jen: OK for QUAD3; his descent choice; the C++ line. On the day: QUAD3's marker 23 + plate 3's record
  (the second funnel is on plate 3); QUAD2's pack cells.
- Thesis requests Q009-Q015 all closed (F109-F123).

## (earlier) NOW (Fri 13 Nov 17:05): lab session DONE (08:00-11:05, Te-Jen flew); next lab day Wed 18 Nov
- Today (registry RIG-1113-*; decisions 13 Nov; Te-Jen's package emailed to him 13 Nov ~15:00 in 4 zips + supplement):
  his C++ approach + Python return into the MOVING ring: 7 successes in 14 attempts that flew; on the FROZEN line (10:40)
  4 of 4: fm9h8_13 (figure-eight), fm9hc_10, fm9hc_11 (circle), fm9h_2 (hover). Fixes that made it work: carrier state
  timeout 1.0, splice 0.30, search 0.10 s, tracking brakes OFF (the supervisor), hard hover latched per episode, goal 3.0 m.
- The hero fm9c_2: pickup (3rd try) and drop into the circling ring; re-attach missed (our contact window fires 4 cm up;
  his lift proof starts before QUAD4 reaches the plate). OPEN fix (decisions): comotion_normal_max 0.01 (Wesley) and/or a
  bottom hold before his LIFT_PROOF (Te-Jen).
- No detach/re-attach-only flight today (nr8 launched with QUAD4 read on plate 6, stopped). The paper's re-attach count is
  still 11 Nov's (nrtfc_2, nrtfc_5 held).
- QUAD2's pack reached 18.9 V under load in fm9c_2 (3.15 V/cell): balance-charge, check cells.
- RUNNING (agents, from ~16:58; outputs in results/analysis/): cpp_rig_2026-11-13 (C++ flights table, figures, draft rows
  for fm9hc_10/_11, fm9h_2), reattach_contact_2026-11-13 (contact-rule replay), pickup_seat_2026-11-13, q009_x1_second_swing,
  q010_tilt_while_away (thesis requests Q009/Q010: draft feed entries), 424_design_2026-11-13 (ours.md, tejen_stack.md,
  rig_and_twin.md: the supervisor's 4 -> 2 -> 4, both returns by Te-Jen's attach, simultaneous commands; Wesley 17:00).
- NEXT: append the C++ rows and the Q009/Q010 feed entries (close the requests), feed F109 (today's rig results for the
  thesis: "C++ approach, Python return"; the hero), the 4 -> 2 -> 4 build plan to Wesley before 18:00, the 18 Nov sheet
  (docs/rig/wed_2026-11-18.md: re-attach fix, 3-5 re-attach flights, the full mission, then 4 -> 2 -> 4 if built).
- 18:21 MERGED the 4 -> 2 -> 4 build into the main tree (41 files; originals backed up in ~/mdc_424/main_backup_1816,
  list in its FILES.txt), colcon build of the 6 packages, tools/gate.sh --quick PASSED (2319 passed, 19 skipped), today's
  rig scripts dry-run clean, the five twin configs pass --check (the ring inertia warning is the old one). New script
  results/rig/2026-11-18/scripts/rig_424.sh; sheet draft ~/mdc_424/work/sheet_424.md. Critic RUNNING on the card + diff
  (~/mdc_424/diff_all_1815.patch). Then: the lab PC batch T1/T3/T4/T5/T7 (free at 18:16), score with tools/score_424.py.
  Agent cap back to 2 (Wesley 18:15).
- 20:15 STATUS: 4 -> 2 -> 4 twin-complete on the final planner (R4461-R4473; R4473 = both together: both caught first try,
  the four share 2.40-2.46 N, ring <= 0.27 deg). 18 Nov sheet DRAFT docs/rig/wed_2026-11-18.md (rows: nrtf x3-5, the hero,
  then 4 -> 2 -> 4 with its ~30 min re-hang; the order is Wesley's call). Left before 4 -> 2 -> 4 flies: the RUNTIME desk
  check (fake mocap, both of his instances, one publisher per topic; critic finding 11), Te-Jen's OK, QUAD3's marker 23 and
  plate 3 calibration on the day. Thesis: Q009/Q010/Q012/Q013 all ANSWERED (F109-F122, reviewed). Nothing committed.
- 19:10 4 -> 2 -> 4: critic GO WITH CHANGES (card ## Critic); Wesley: fold in at once (no queue), both TAKEOFFs together,
  55 deg, the square's split with lift + level rows (~2.4 N each). All merged (backups ~/mdc_424/main_backup_r2_1851,
  _r3_*), gate --quick PASSED 2323 (before the split fix; planner suite 514 after it). Twin: R4461 (sequential, 43.6 deg
  peak) and R4462 (set detach, 0.24 deg, 10.5 cm dip at the release) registered. RUNNING on the lab PC since ~18:53:
  T4, T5, T7 + the critic's t5_land, t5_giveup, t8, t9, t10, t11 (synced BEFORE the split fix: re-fly T4/T5 after).
  Sheet draft ~/mdc_424/work/sheet_424.md (to docs/rig/wed_2026-11-18.md once the twin is scored).
  Thesis requests: Q009 (F110), Q010 (F109), Q012 (F112) answered; Q013 (real-world tables) items 3-4 agent RUNNING.
- 18:12 BUILD STATUS (~/mdc_424/work): P planner DONE (511 tests, 39 single-partner scenarios bit-identical), G launch
  graph DONE (28 legacy launch lines byte-identical to base; bringup 257, drone_magnet 178, drone_communication 48),
  T his stack x2 DONE rig + twin (tejen_mission 1327 passed, the same 26 old failures), O console/rig_424.sh/
  attach_424_pair.yaml/sheet_424.md DONE (46 console tests). W twin world/configs/score RUNNING. Card:
  docs/experiments/2026-11-13_424.md (critic not run). NEXT (each needs Wesley's go, after 18:00): integrate work ->
  main tree (diff vs base), colcon build, gate --quick, critic, twin T1/T3/T4/T5/T7 on the lab PC, desk check.
- 17:40 BUILD RUNNING (Wesley 17:3x: the FULL 4 -> 2 -> 4 for 18 Nov): five builder agents in a SEPARATE copy
  ~/mdc_424/work (pristine copy ~/mdc_424/base for the diff; today's tree untouched), contract ~/mdc_424/CONTRACT.md
  (partners_file switch, default off = today identical; per-partner topics: drone 3 keeps today's names, partner d gets
  /drone_<d>/... and /tejen_d<d>/...; /fleet/detach_set). P planner (dissipative_node), G launch graph + nodes, T his
  stack as two instances, O console + rig_424.sh + attach_424_pair.yaml + sheet, W twin world + configs + score_424.
  Then: integrate (diff work vs base into the main tree only after tests), colcon build, gate --quick, desk check with
  fake mocap, card + critic (docs/experiments/2026-11-13_424.md), twin T0-T7 on the lab PC, the 18 Nov sheet.
  Needs: Te-Jen's OK to run his stack on QUAD3; on the rig: a funnel on plate 3, QUAD3's marker 23 at its tip + plate 3
  calibration, magnet pull test (the pair gets the two strongest), the pair-hang roll check.
- 17:30 contact window 1 cm is the rig scripts' default (CNMAX, decisions); Te-Jen emailed the pickup/re-attach finding.
- Videos: organise_rig_videos.py + rig_flights_2026-09-30_to_11-13.csv sent to Wesley (scratchpad videos_1113/).

## (earlier) NOW (Fri 13 Nov 04:45): the 08:00 sheet for Wesley (docs/rig/fri_2026-11-13.md, final); Te-Jen flies it
- Row 1 (Wesley to confirm by email): `CPP=on RET=direct PAUSE=takeoff bash ~/rig_fm9h8.sh` (his C++ over the figure-eight
  stopped at his TAKEOFF, Python back). Backup: `RET=direct bash ~/rig_fm9h8.sh` (moving, direct route).
- Twin batch 9, the rig line in the rig's order (R4435-R4440): row 1 2 of 2, circle 9/5 o'clock 2 of 2 (pause sequence:
  tools/pause_sequence.py; ring at rest +3 s, C++ from +11 s); R4438 QUAD4 crashed alone in its loaded climb (twin rod
  pendulum, as R4339); R4440 C++ into the MOVING figure-eight: no capture, 0.04 m. Batches 7-8 (R4423-R4434, R4423c):
  still-ring only. All registered; decisions FINDINGs 03:25, 03:40, 04:39, 04:44; feed F106, F107.
- Built tonight (all typed, tested; gate --quick PASSED 2230 at 03:49): scripts refuse CPP=on without PAUSE=takeoff on the
  figure-eight/circle (CPPMOVING=1 overrides; copies in results/rig/2026-11-13/scripts/); his twin supervisor's
  start_gate_min_traj_t (CHANGES_FROM_TEJEN); tools/dropoff_summary.py, tools/pause_sequence.py; make_t13_configs splice +
  the gate. Real-time replay evidence: results/analysis/cpp_realtime_2026-11-13/ (README, evidence_compact.md,
  zero_windows.md).
- Videos: scratchpad/video_row1/R4435_baseline.mp4 (row 1), scratchpad/video_pause/R4428_baseline.mp4 (backup).
- 04:45 EMAIL SENT to Wesley (row 1 decision, videos R4435/R4428, the sheet) + push. 05:16-05:31 batch 10: our LAND in
  row 1's rig order 4 of 4 (R4441-R4444; Codex entry 19's gap). OPEN 05:16 in decisions: the bridge's frozen future
  during the 3 s pause ramp (two candidate fixes, need Wesley/Te-Jen; row 1 relies on the observed >= 7.8 s).
- Codex mailbox: our entries 1-18 sent, its 1-19 read and acted on. Nothing running on the lab PC. Nothing committed.
- Next: the email to Wesley (row 1 decision); then wait for his answer; at 08:00 read each flight's log when Te-Jen or
  Wesley reports it.

## Thu 12 Nov 11:12: LAB SESSION DONE; next lab day Wed 18 Nov
- Flown: fm9_2, fm9_5 (FULL MISSION complete, F103), fm9c (position limit at takeoff), fm9c_3, fm9c_5 (circle pickups failed),
  fm9hc_2/_4/_5 (held object dropped into the circling ring + re-attach: 3 of 3 complete), nro PAUSE=off (no attach);
  eight launches never armed (RIG-1112-launches). All registered; feed F104.
- Built today, typed only: pos_limit_m 1.8 (real.yaml, Wesley), ARAD (his attach_radius_m; flown fm9c_5), PRESS (his
  pickup_candidate_press_m; NOT flown), PAUSE=off for rig_nro.sh (configs/rig/attach_m1_1m_m2a_orbit_det15_nopause.yaml).
- Te-Jen: the full message (sim demos + rig session + 3 questions) emailed to Wesley 11:23 to forward; attachments
  ~/Downloads/tejen_rig_logs_2026-11-12.zip, ~/Downloads/tejen_fork_changes_2026-11-12.zip (attach radius + press), the
  demo figures/clips in ~/Downloads/tejen_demo_videos. Wesley sends.
- Disk 4.7 GB free: archive old rig days before 18 Nov (each session writes ~2-4 GB).
- 16:37 Nothing running. Batch 3 done and registered: his C++ loaded transit with three carriers, twin (R4338-R4349):
  C++ owned the transit 12/12, reached the drop 0/12 (hover stalled at the carriers; circle/fig8 never captured, up to
  3.6 m; R4339 QUAD4 crash, tilt 75 deg); backend padded final check vs the carriers' tethers (decisions FACT 16:37).
  R4350/R4351 (R4316's case, direct route, his reference recorded): both re-attached, cable >= 0.22 m. Diagnostics
  zip for Te-Jen ~/Downloads/tejen_cpp_twin_2026-11-12.zip (24.7 MB). One-quad rig flights q4d_3/q4d_4 (C++ worked,
  no carriers) registered.
- WAITING FOR WESLEY (Te-Jen ~16:55): tomorrow C++ or nothing. Te-Jen's values for a twin batch: swing bound 3/5/7 deg,
  spline_time_factor 4, workspace +-1.8 m x,y and z 0-2.9 m; build two typed switches + run ONLY after Wesley says so.
- SYNC 3 PROPOSED (decisions FINDING 17:04, emailed to Wesley): we fly Te-Jen's 21 Oct planner (fd495db0); his branch is 143
  commits ahead (09fef0b7, fetched read-only in ~/tejen/drone_cage_control). Wesley to decide: merge, gate, re-fly the 12
  C++ twin cases, then the swing/spline/workspace sweep.
- 17:40 SYNC 3 DONE (decisions DECIDED 17:40; CHANGES_FROM_TEJEN sync 3; backup ~/mdc_sync3_backup_*), gate 2225 passed.
  RUNNING: lab PC batch (scratchpad lab_sync3.log): the 12 C++ cases + t13_hover_s9 + t13_circle_s1_gap on the merged code;
  QUEUED after it (waiter): 8 tuned runs (t13_*_cpp_sw{3,5,7}_twin). Swing analysis of rig flights (scratchpad
  sync3/geom.py -> geom.csv) running. To do: rerun his core's test_suspended_system quiet; registry rows; Te-Jen summary.
- 17:58 TE-JEN LEFT; await Wesley. Agreed overnight (Te-Jen, Wesley's plan): read batch 1 (merged code, 14 runs, lab_sync3.log)
  and batch 2 (tuned: swing 3/5/7 deg, spline 4, inflation 0.06, workspace 1.8/2.9; lab_tuned.log, queued by a waiter);
  if C++ still stalls, SWEEP WITHIN LIMITS one variable at a time, hover 9 o'clock first: swing >= 3 deg, inflation >= 0.04,
  spline <= 6, carrier body box >= 0.09 m half; no planner-code changes; backend diagnostics per run. His MPC: 2c1ebe03 already
  ours (cable_length); 13b927bb is display-only (plan viz 10 Hz when watched): NOT taken, told Te-Jen. Rerun his core
  test_suspended_system quiet. One-quad today on sync 3: RIG-1112-q4d_7 (C++ transit with the tuned geometry, drop 10.5 cm).
  Scripts: CPP=on in rig_fm9h/hc/h8 now types the tuned geometry (SWING/SPLINE/TRACK/WSXY/WSZ), desk OK; rig_q4d.sh has
  SWING/SPLINE/WSXY/WSZ (default 5/4/1.8/2.9). Rig cables unplugged; laptop on Wi-Fi (Tailscale to the lab PC).
  For the morning: the sheet docs/rig/fri_2026-11-13.md (rewrite for C++ + 1 trial each, Te-Jen's proposal), morning report,
  email Wesley. Wesley's rule: no one-quad flights 08:00-11:00 (Te-Jen asked for one during setup: Wesley's call).
- 21:00 C++ TWIN STATUS (all registered): sync-3 code, his settings (R4352-R4363): 1/12 completed (R4362 circle s1, C++ on
  both legs, 158 s). Tuned WITH workspace +-1.8 (R4366-R4373): VOID, no plan from the 1.9 m start (FINDING 20:27; CPP=on
  scripts now type the workspace only with WSXY). Tuned WITHOUT workspace (swing 5, spline 4, infl 0.06; R4374-R4378):
  circle s9 COMPLETED (R4377, C++ transit 86 s; but cable-to-carrier-cable 0.10 m, ball 0.07 m); hover 9/1/5 + swing 3:
  stall 0.50-0.67 m from the centre (never inside his capture region over a still ring). RUNNING: sweep on hover s9
  (infl 0.04, body 0.09, spline 6) + R4377 repeat (lab_sweep1.log). Next: Te-Jen diagnostics package, morning report,
  the 13 Nov sheet for Wesley's plan (C++ hover not shown to work; circle 2 of ~6).
- REMIND WESLEY WHEN HE PLANS TOMORROW (Te-Jen ~16:05, his words, no changes until Wesley decides): Wesley will write a plan
  for Te-Jen to follow while Wesley is away. Aim: ONE trial each, not three: 1 hover, 1 circle, 1 figure-eight. Hover and
  circle are known to work with the direct route ("the heuristic"): tomorrow they swap it for his C++ planner (CPP=on).
  Then the figure-eight, the more uncertain one. One-quad today: q4d_3/q4d_4 his C++ flew the loaded transit (twice);
  the settle wait fixed with the rig's gate (0.18 m / 0.20 m/s): 2.6 s; drop 12.8 cm off (lower release next: DZ=0.08
  RELEASE=0.0, not flown).
- FRI 13 NOV LAB (Wesley + Te-Jen, decisions DECIDED 12:45): hover x3, circle x3, figure-eight x3, QUAD4 holding the
  object from 9/1/5 o'clock, ring stops for the re-attach on 9. Scripts ~/rig_fm9h.sh, ~/rig_fm9hc.sh, ~/rig_fm9h8.sh
  (new; contact_rig_fig8, 12 laps; copies results/rig/2026-11-13/scripts/), switch LINEUP=gap. OPEN: Te-Jen's answer
  on the route in (LINEUP=gap vs start within 1.0 m); the twin results; the critic. Sheet docs/rig/fri_2026-11-13.md
  to write when they are in.
- After Te-Jen's review (decisions DECIDED 12:12): ARAD 0.04 default in the pickup scripts; PRESS now a ramp (0.02 m/s,
  floor = the latched contact target), typed only, NOT flown, props-off hand grip test first (Wesley); rig_nro.sh
  PAUSE=off passes the raw twist; rig_flight.sh records /fake_attachment_point/pose again. Reply + nro extract
  (~/Downloads/tejen_nro_extract_2026-11-12.zip) + package 12 (ramp) emailed to Wesley 12:13 to forward.
- Old results archives: UNSW OneDrive via its web page (decisions 2026-10-23; ~/mdc_onedrive_upload/MANIFEST.txt).
- Placement (asked 07:40, Wesley wants the object further out for less wind): the 1.3 m rule is a margin, not a gate
  (the console only turns orange; the hard guard is the pickup box 1.9 m). QUAD4's pose froze only toward the +x,+y
  corner (fm9c (0.65-0.71, 1.82-1.93), fm9c_4 (1.02, 1.72), fm9c_6 (0.29, 2.34)); pickups at y 1.63-1.80 with x -0.24
  to -0.50 tracked fine. Advised: object straight out along one axis (|x| < 0.4) up to ~1.6-1.7 m, ring ~0.3 m toward
  -y. OFFERED, not done: console warning past 1.7 m (or 1.4 m in y with |x| > 0.5) + the sheet's placement line.
- Morning report docs/reports/morning_2026-11-12.md (emailed 02:19); its four decisions are still open (0.55 m demos, fig8
  at 0.08 m/s, send Te-Jen the fork packages, disk).
- TE-JEN'S DEMOS DONE (card docs/experiments/2026-11-11_tejen_app_demos.md; table results/analysis/tejen_app_2026-11-12/
  table.md; videos ~/Downloads/tejen_demo_videos): 0.55 m cables (overnight choice, decisions 00:24 / 01:38, Wesley to
  confirm): hover 5/5 (R4291, R4294, R4297, R4300, R4303), circle 0.5 m at 0.125 m/s 5/5 (R4288, R4292, R4295, R4298,
  R4301), figure-eight 1.0 m at 0.08 m/s with looser limits + 120 s 5/5 (R4306-R4310). 1 m: hover 5/5 (R4254, R4257,
  R4260, R4275, R4276); moving 0 of 12 (trail 5-6 cm at 0.125 m/s). Registry through R4310.
- Open for Wesley: (1) 0.55 m for Te-Jen's demos (or 1 m with the pause, or report 1 m as not attaching); (2) figure-eight
  at 0.08 m/s; (3) send Te-Jen the 11/11b/11c packages + videos; (4) disk 8.1 GB (archive 4-6 Nov rig days?).
- Uncommitted as before (+ tonight: tejen_mission fixes, tejen_mpc/throttle_bound, drone_magnet test, bringup sim
  switch, tools/app_demo_figure.py, tools/frozen_pose_desk_check.py, tools/rig_cable_check.py, lab_batch --keep-record,
  configs tejen_app_*). Commit only on Wesley's word.

## Thu 12 Nov 08:00-11:00: READY (sheet docs/rig/thu_2026-11-12.md; desk check 13 OK 22:27; gate --quick PASSED 2219 at 22:27)
Order (plan approved 11 Nov 15:56, decisions DECIDED 15:56): (1) nrtf hover re-attach to 3 more clean (max 6; two misses
in a row -> funnel v4); (2) fm9 full mission in hover until one completes (max 6; FORCE DROP if the drop stalls; two
pickups failed in a flight -> fm9h); (3) nro x2-3 then `PAUSE=pickup bash ~/rig_fm9c.sh`; (4) QROD NOT recommended (R4251).
Read each log within a minute of landing: outcome, catch offset, `python3 tools/rig_cable_check.py <name>` (QUAD4 slack).
Script defaults now (all six pickup scripts / seven attach scripts; switches on the sheet):
- his bands 0.05/0.05 (BAND=wide|mid); settle drop + release 0.03 (DROP=arrival, RELEASE=); pickup LOW hold 0.04
  (PHOLD=0|0.03|0.05); the small object WITH its funnel: PC 0.011, PX -0.0090, PY -0.0098, PS 0.105, PT 0.050 (OBJ=bare);
  the pickup box 1.9 m (PBOX=0 off; Wesley 20:19); FORCE DROP on the console (acts within 0.25 m of the drop point);
  QROD off; PAUSE=pickup on fm9c.
- Frozen-pose guard: OFF (Wesley 20:19: "I will disarm in this situation anyway"); code kept, MDC_MOCAP_FROZEN_S default 0.
- Console: READY line orange when the object or QUAD4 is over 1.3 m out (fleet_viz summary carries x, y).
- Motive: only 6,8,11,12,13,14,24; `tools/mocap_stream_check.py --expect 6,8,11,12,13,14,24` now fails on a shared ID.
- The sheet has an "If this happens" table (button + next switch per failure) and the simulation table.
- Not simulated (rig-only): the pickup hold, the object's funnel and aim point, his drop check wait. First full-mission
  flight is their test. The funnel on the object is ~4 cm tall (Wesley 20:37): inside the hold's 8 cm, no change.

## Tonight's record (11 Nov 15:56-20:57; all in decisions.md, registry, the card)
- Twin runs R4245-R4252 (R4248 void, the restart): fm9 hover, FORCE DROP in the lowering, fm9c PAUSE=pickup: whole
  mission each; QUAD4 mocap loss under his controller -> QUAD4 down alone (R4249); under ours after LAND -> fleet
  EMERGENCY STOP (R4250, the standing rule); QROD worse in the twin (R4251); pickup box refuses (R4252).
- Fly-away fm9c_6 EXPLAINED (decisions FINDING 17:27): the big object fell off by itself in the lift, slid to y 2.16,
  his retry flew after it (ref to (0.25, 2.15, 2.54) at 0.8 m/s), QUAD4 overshot to (0.29, 2.34, 2.89) and was lost; his
  planner had no bound -> the pickup box. fm9c_4 was a second blind landing (our take-back steps a loaded QUAD4 out to
  ~1.65 m by design): sheet rule "object on: FORCE DROP before LAND".
- Bounce (docs/analysis/quad4_bounce_2026-11-11.md): QUAD4's cable only just taut (17 % share, reference 2-4.5 cm inside
  the cable, plate 9 low 11-19 mm); why three flights tipped into the 3-4 Hz slack-taut cycle is OPEN. Not fixed.
- Te-Jen: note docs/rig/notes_for_tejen_2026-11-11.md; zips ~/wesley-rig-2026-11-11.zip (77 MB, note inside) and
  ~/wesley-rig-2026-11-11-bags.zip (730 MB): WESLEY SENDS (Drive links).
- Feed F102 sent and read by the thesis chat (it fixed 4.2.2's detach swing; the rest goes to Wesley).

## Open, for Wesley
- Archive the 4-6 Nov rig days (~5 GB back; 11 GB free, a lab day writes ~4.6 GB). Not needed for 12 Nov.
- After 12 Nov: tick handin_plan's S3-S6 items with run ids; feed entry for the paper (re-attach count, full mission).

## Wed 11 Nov lab session DONE (08:00-11:20); registry RIG-1111-*, feed F101; next list sent to Wesley for confirmation
- Flown: nrtfc (void: QUAD4 started on plate 1), nrtfc_2 CAUGHT + held (cone bands, v4), nrtfc_3 caught then lost 4 s later
  (loose magnet lead, fixed), nrtfc_4 magnet off in the pretension (same lead); k1zo drop into the circling ring worked but
  the impact detached the ring (release 0.14); k1zo_3 our magnet-gate race (FIXED); fm9hc_8 drop into the circling ring OK
  (release 0.03), his drop check gave up early (FIXED); fm9c / _3 / _4 / _6: pickups failed whenever the magnet arrived
  4.7-9.7 cm off the object (7/7), lifted whenever within ~2 cm (4/4); the bigger object never passed his transit gate on
  the circle (fm9c_4) and fell off in the lift (fm9c_6).
- Built today (live, unit-tested, NOT simulated): magnet_gate held-OFF race fix; his drop check waits drop_puck_wait_s;
  drop_release (RELEASE=, k1zo 0.03); attach_pause_at pickup_reattach (PAUSE=pickup on fm9c); OBJ=big on fm9c; nr1f plate 1
  measured (W 0.0427); console DETACH / magnet OFF-ON. Plate 9 unchanged; body 6 centred.
- Proposed next (awaiting Wesley's confirmation): (1) the pickup alignment (descent gate/band as the attach analysis);
  (2) sim-check today's code + gate --quick; (3) Thursday's sheet in order nrtf (to 3-5 clean), fm9hc, fm9c with the pickup
  fix; (4) the bigger object only with DROP=arrival or not at all; (5) minor: QUAD4 bounce, off-centre catch radius.

## Tue 10 Nov night: NEW RIG PLAN approved (decisions DECIDED 2026-11-10 23:10); lab Wed 11 at 08:00
- Plan: (1) nrtf hover x8 then nro x3 (Wed); (2) k1zo x3, the drop into the circling ring (Thu); (3) held object into the
  circling ring + re-attach, only if pickups fail; then the FULL MISSION on the circle (stop for the re-attach AUTOMATIC,
  new switch, default off) on the 18th-19th, fallback fm9 hover. Dropped: rel, relo, fm9o, k1z (backup). Funnel v4, then
  the supervisor's (105 mm). OPEN: the descent bands; the thesis freeze (Thu 12 vs 19).
- Wesley asked to explain the pre-measurements BEFORE executing the plan (answered in chat 23:15). Then, in order: rewrite
  the four sheets (wed_2026-11-11, thu_2026-11-12, wed_2026-11-18, thu_2026-11-19 + plan_sessions); k1zo script + desk
  check + twin run; the automatic stop-at-re-attach switch + tests + twin run.
- 23:42 DONE: descent bands (decisions DECIDED 23:42): v4 as flown; `BAND=cone` (0.05/0.5/0.05) for the supervisor's
  funnel in the 7 attach scripts; summary results/analysis/attach_band_2026-11-10/summary.md. Morning report: put the band
  result FIRST (Wesley). Built: attach_pause_at (src), console DETACH/QUAD4 magnet OFF/ON, scripts nr1f/k1zo/fm9c/fm9hc;
  gate PASSED, desk 13 OK. Sheet docs/rig/wed_2026-11-11.md rewritten (one list; old in docs/rig/archive/).
- RUNNING (laptop, the lab PC is in use by someone else): ~/mdc_overnight/plan1110.sh (pid file plan1110.pid; out
  plan1110.out): twin checks rig_fm9c_twin, rig_k1zo_twin, rig_fm9hc_twin one at a time; a background wait task notifies.
  Then: registry rows, the sheet's "Simulation checks" section, desk check re-run (BAND switch), morning report.
- 06:38 Wed: discriminators R4243 (nrtf at 1.2 m: still no contact: twin plate artifact, centred pushed tip sinks)
  and R4244 (fm9h re-fly: worked; R4236/R4240 a flake). Sheet + decisions FACT 06:45 + registry done. Morning
  report sent in chat (bands first). Lab session 08:00-11:00: Wesley flies the list; read every log he reports.
- 06:05 Wed: overnight twin checks DONE (R4228-R4242, registry rows written): fm9c, k1zo, fm9hc(arrival), fm9, k1z
  and the settle options worked; nrtf/nro/nr1f/cone did not re-weld in the twin at ring 1.0 (tip pushed through the
  plate, rigid rod); fm9h/fm9hc_settle never climbed (flake?). RUNNING ~/mdc_overnight/plan1111.sh: rig_nrtf_z12_twin
  (ring 1.2) and rig_fm9h_retry_twin; then the morning report (descent bands FIRST).
- 23:50 DROP switch (decisions DECIDED 23:50): circle default = his settle drop (DOA off, DH 0.48), `DROP=arrival`; hover
  default = arrival, `DROP=settle`. QUEUED after plan1110: ~/mdc_overnight/plan1110b.sh (pid plan1110b.pid, out
  plan1110b.out): fm9c_settle, k1zo_settle, nrtf, nro, nr1f, fm9hc_settle, nrtf_cone, k1z, fm9, fm9h, k1z_settle,
  fm9_settle twins (configs/experiments/rig_*_twin.yaml). Waiters: a background task on each script's pid.
- Funnel CAD done today: v5, v6 (cad/funnel/funnel.py), compare.py, render_on_plate.py; files in ~/Downloads/funnel/.

## Fri 6 Nov lab (16:40-18:00): DONE, Wesley packed up at 18:01
- Flown (results/rig/2026-11-06/, registry RIG-1106-*, feed F064): q4f/q4f_2 CRASH (two stacks on QUAD4's radio,
  --once commands; QUAD4's FC disarmed itself at the flip; VOID; decisions FINDING 17:29 + CORRECTION 17:33); q4f_3/q4f_4
  no arm (FC crash lockout until a battery replug); q4f_5, q4f_6 funnel v4 seated first descent (7.2 -> 0.3 cm); q4p
  pickup first try 0.9 cm; q4p_2 not armed. QUAD4 damage after the crash: not reported yet (ask).
- Scripts (results/rig/2026-11-06/scripts/): rig_q4f.sh, rig_q4p.sh, rig_q4m2.sh (never flown), each refuses while any
  flight stack is up and starts one_quad_panel_bridge.py (RViz ArmPanel -> his topics; refuses ARM while the FC reads
  !ERR, normal for ~10-15 s after launch). Sheet docs/rig/fri_2026-11-06_one_quad.md.
- His code changes live ONLY in the overlay ~/mdc_m2one_ws/src/tejen_mission (one-vehicle M2; LAND beside the magnet,
  landing_side_step_m, default 0; 13 new tests): to port into src/tejen_mission + CHANGES_FROM_TEJEN.md after the lab
  batch, on Wesley's word (keep or record-and-delete).
- Open with Wesley: QUAD4 OK?; is the pickup worse with the carriers flying (fly 2-3 more q4p next lab day, then the
  steel disc / fm9h / pickup-before-lift); the 'week8' thesis-chat question.

## RUNNING (second resume Sat 7 Nov 08:09, after the disk filled overnight)
- Overnight: the laptop disk filled (~23:30); X3 had finished (32/32 by ~23:15); X3b's R3980/R3981 ran but their
  fetch failed and lab_batch crashed (R3982/R3983 NOT RUN, registered). 08:05: Wesley OK'd deleting the replays (already
  gone), the journal (needs his sudo: `sudo journalctl --vacuum-size=200M`, not done) and logs/m2_attachment (done,
  2.8 GB), and "delete what you need to": R3978-R3981 fetched by hand from the lab PC; the October sim results archived
  to results/archive/*.tar.zst (32 GB -> 11 GB; ~/mdc_overnight/archive_october.sh); 24 GB free.
- `~/mdc_overnight/claim1107_resume2.sh` (pid file claim1107_resume2.pid, log claim1107_resume2.log): X3b rest as
  R3984/R3985, then 5_x2, 8c_rejoin, claim1107 6, 7, 8a, 8b, 8d, 9; disk guard before each chunk (< 8 GB: archive
  2026-11-01..04 oldest first, verified; still < 8: stop). Same tree (hash 5e7b1d3bd133), no sync. Watcher task on each
  chunk's EXIT (re-arm after each).
- Then `claim1106_x1old.sh` (pid file; waits on claim1107_resume2.pid): X1 old masses (sync + build), void re-flies.
- `x3b_bench_night2.sh` (pid file): tonight 00:30-06:30.
- DONE 7 Nov: X3 old masses (R3922-R3927, R3954-R3979) F065 + reviewer F066 (R3922c); X3b chains (R3980, R3981, R3984,
  R3985) F067 + reviewer F068 (R3980c). Queued after X1 (~/mdc_overnight/void_refly.txt, Wesley can veto): x3b_s8_hover_rt,
  x3b_s8_fig8_rt, x3b_s9_hover_rt on the earlier masses x2 (configs/experiments_oldlinks/, made 08:50) so C8 and N9 sit
  on one quadrotor model. Open with Wesley: keep the old-mass X3 (N4 miss explained) as the thesis numbers?
- DONE 7 Nov: X2 old (R3986-R3991) F069 + reviewer F070 (R3986c; card section added): A FALSIFIED in one repeat, B
  CONFIRMED. Rejoin table old (R3992-R3999) F071 + reviewer F072 (R3992c): all bars hold; bars were set by the lab chat
  23 Oct before each case, not frozen by Wesley (card note added). X8 (R4000-R4013) F073 + F074 (R4000c); 4.1/4.2 (R4014-R4033) F076 + F077 (R4014c: yaw tool alignment bug,
  fix after the batch); X11 U (R4034-R4093) F078 + F079 (R4034c: 10 landing aborts on a touchdown spike, rim ordering
  reversed since 23 Oct, thesis draft results_discussion.tex:201 stale); X11 joins (R4094-R4107) F080 + F081 (TB14 landing aborts); Quan
  (R4108-R4139) F082 + F083 (draft line 325 stale); gap table (R4140-R4149) F084 + F085 (draft lines 79-96 stale).
  X1 (claim1106_x1old.sh) started 15:30: sync + build OK, flying; then the 8 queued runs.
- Wesley 7 Nov ~16:00: bars on every sim claim arbitrary, results as is, compare with other papers where possible (F086,
  BATCH1107d); Te-Jen's one-quad overlay not needed (deleted; diff in results/rig/2026-11-06/scripts/); no one-quad tests
  in sessions (multi-quad only 08:00-11:00; CLAUDE.md); QUAD4 fine; 'week8' = the thesis chat (nudged with F061-F086);
  laptop left idle tonight for the X3b bench. Wesley 7 Nov ~10:10: X3 ACCEPTED after the
  run (bars arbitrary; F075, R3922d); X3b six runs + X8 hub noxy x2 queued after X1. (X2 etc.: F086 covers it.)
- 7 Nov ~18:40: batch DONE (18:20). Drop-rule fix APPLIED to the working tree (gate --quick PASSED, desk 9 OK; worktree
  redundant, Wesley removes it). October results archived: tools name ~300 October runs; extract on demand (F092);
  restored for the gate R1140/1145/1206-1208/1211-1213/1459/1460/1626. F090 (X3b one model) reviewed -> F093 (N9 NOT confirmed:
  R9a pending, R9b void); F091 (X8 hub noxy + x1_both_models table) reviewed -> F094. ALL CLAIMS FINAL AND REVIEWED, incl. X3b's
  R9a bench (8 Nov 00:30; F095 + F096: R8a / R9a hold; R8 FALSIFIED by R8b; N9 not confirmed). THE SIMULATION SET IS COMPLETE.
- 10 Nov: funnel v5 (Wesley: the base as high as the electromagnet): v4 with the bore 20 mm (81 mm tall, play 10.5 mm,
  80 g); cad/funnel/funnel.py VARIANTS['v5'], drawing cad/funnel/draw.py; files in ~/Downloads/funnel/funnel_v5.zip.
- 9-10 Nov: Q011 (the thesis chat, Wesley) the 4.3 joins re-flown on the 0.86 kg ring on PLATE worlds (new:
  tools/make_m2a_plate_world.py, his geometry rig_twin_L100_plate), LAND +30 s: R4214-R4227, F097 + F098, Q011 CLOSED;
  first attempt R4202-R4213 void. Figure 15: R4227. Tonight: X3b bench 00:30-06:30 (laptop idle).
- DONE 7 Nov: tools/yaw_track.py aligned on t_ready_sim (F077's numbers reproduced); gz_record already opt-in. Left: port the overlay
  ~/mdc_m2one_ws changes on Wesley's word, the drop-rule merge, gate --quick + desk check before Wed 11.

## After each chunk lands
Register (append rows, LF), one-tree hash (claim_batch manifest section 5; the old-mass chunks share 3ff122e74ceb with X4 old),
score (section 10 scorers; X3 needs its bases: tools/x3_metrics.py <X3 ids> <bases>), feed entry, a reviewer agent, tick
sim_runs_todo/handin_plan. X3/X3b/X4/X1 final numbers = the OLD-MASS runs (Wesley 6 Nov ~10:30, 'only claims with returns' +
X1 ~11:35); the final-mass runs R3772-R3820, R3821-R3906 stay as recorded comparisons.

## Done today (6 Nov), in one place (details below and in decisions.md / feed F052-F063)
- X4 FINAL on the old masses R3910-R3914 (F062 + reviewer F063): 26/26, packs 575-589 s, 2.9 B, laps one carrier short 21-34 mm.
- X1 (final masses, R3821-R3906, F059/F060) superseded by its old-mass re-fly (Sat). X3/X3b final-mass R3779-R3820 (F055/F057).
- Slower returns = the 4 Nov model change (A/B R3907/R3908, F058); old-mass worlds by tools/make_oldlink_worlds.py (R3909 smoke).
- QUAD4 drop-rule fix BUILT in the worktree .claude/worktrees/agent-aa650a10b71d57ce6 (critic waived; R3915-R3917): MERGE after
  the X1 old-mass re-fly (Sat), then gate --quick + the desk check, before Wed 11. Then remove the worktree (Wesley).
- The paper (decisions 6 Nov): sessions plan APPROVED (docs/rig/plan_sessions_2026-11-11_to_19.md); sheets wed_11, thu_12,
  wed_18, thu_19 rewritten; scripts rel, relo, nrtf, nro, k1z, fm9, fm9h, fm9o (drop_on_arrival_xy 0.08); desk check 9 OK
  (16:18); twin checks R3918-R3921 PASS (watch-points in the sheets); the paper's tables (results/analysis/paper_rig_2026-11/)
  and single-run figures (results/analysis/paper_figs_2026-11/); attach ideas (docs/analysis/attach_ideas_2026-11-06.md).
- Funnel v4 (TPU, 85 mm base, 85 mm tall, 207 x 157 mm mouth, 45 deg, 2 outer bolts, zip-tie notch, 83 g): cad/funnel/funnel.py,
  ~/Downloads/funnel/funnel_v4.*; v2/v3 do not pass the magnet's wire fitting.
- Reviews of 5 Nov night (docs/reports/reviews_2026-11-05/), memory/CLAUDE.md fixes, rig docs archived (docs/rig/archive/).
- Open with Wesley: is the 'week8' session on this laptop his thesis chat? (the F061/F062 nudge not sent until he says).

## Fri 6 Nov ~14:30-15:00: the paper first (decisions 6 Nov); agents unlimited (Wesley: "ultracode no limit on agents")
- The combined paper (supervisor + Te-Jen; Overleaf clone multi_drone_swung_payload_item_collection_wesley_version/, git-
  ignored): attach and detach performance (detach = the unannounced release; moving = the orbit, stop-and-attach) 5 static
  + 3 moving each, and the full mission (fm9, triangle, funnel; fm9h with the object held). Plan APPROVED:
  docs/rig/plan_sessions_2026-11-11_to_19.md. Thesis framing (F061): structure sim then real; X1 supporting; Setup 3; radii as flown.
- **16:38 the paper's single-run figures** (results/analysis/paper_figs_2026-11/: paper_figs.py attach|detach|application
  <flight>, README; examples sq8_2, nr9m, fm9y_2 in figs/; same definitions as paper_rig.py). Re-run after each session.
- **16:11 X4 FINAL on the earlier masses** (R3910-R3914, registered, feed F062): 26/26 in all three relays (planner logs; the
  event list misses the last weld in two), packs 575-581 s (R7 holds), 2.9 B, R4 not confirmed (31-34 mm laps, the massless
  plan). Reviewer DONE 16:20: SUPPORTED WITH CORRECTIONS (R3910c, feed F063: 26/26 is a logging gap not LAND; laps one
  carrier short 21-34 mm, no cause; returns ~3 s slower than 3 Nov). X4 is final. Rig scripts' twin checks R3918-R3921 PASS
  (watch-points in the sheets; desk check 9 OK 16:18). Old-mass X3 + bases (32 runs, from R3922) from 16:11; watcher on its EXIT.
- **~15:30-16:00 funnel:** v4 (TPU): 85 mm base (the magnet's wire fitting), 85 mm tall, 207 x 157 mm mouth, 45 deg, two outer
  bolts, a zip-tie notch, 83 g; v3 back to its 72 mm base; no spare (v2/v3 do not pass the fitting); files ~/Downloads/funnel/
  funnel_v4.*; sheets say v4 (decisions 6 Nov). Agents now: the rig scripts' twin checks (laptop); the paper's single-run
  figure scripts (results/analysis/paper_figs_2026-11/). X4 old: 4/5 done (R3910/R3911 26/26, packs 580 s), R3912 ~16:20.
- **Done ~15:20:** the QUAD4 drop fix BUILT in the worktree (decisions BUILT; R3915-R3917; merge after Sat's X1 re-fly, then
  gate + desk check); rig scripts relo/fm9/fm9h/fm9o + 4 sheets (desk check 9 OK at 15:22, with drop_on_arrival_xy 0.08:
  Wesley); attach ideas doc (funnel v3 on every flight, bench check in Wed's sheet); lab_sync.sh excludes /.claude/.
- (earlier) Agents running: the QUAD4 drop rule in a worktree (.claude/worktrees/agent-aa650a10b71d57ce6, critic waived, twin runs on
  the laptop); the rig scripts (relo, fm9, fm9h, fm9o) + the four sheets + desk check + twin configs; the attach ideas
  (docs/analysis/attach_ideas_2026-11-06.md); the paper's rig metrics (results/analysis/paper_rig_2026-11/).
- Cleanup (Wesley: "clean up some old stuff"): 21 past-day rig docs git-mv'd into docs/rig/archive/ (README; registry
  paths unchanged by name); the 5 Nov ~/rig_*.sh copies and 118 finished overnight scripts/logs moved to
  ~/mdc_overnight/old/; nothing deleted. The X3b night bench re-armed (the restart had stopped it).

## Fri 6 Nov 11:22: X1 done; the old-mass batch running
- 14:20 disk (1.3 GB free): Wesley said no replays; lab_batch now skips gz_record/ from the next chunk (decisions
  DECIDED 14:20); Wesley deleted the 157 existing replays himself -> 11 GB free. The X4 chunk's last 3 runs still fetch theirs.
- X1 chunk done 11:22: 43 runs scored (R3821, R3822, R3865-R3906; R3903 VOID: the runner hung after LAND, re-fly
  x1_h4p115_ours x1 after the batch), registered, feed F059 (B1/B2 falsified as on 30 Oct; the resize's effect is the
  settled level; sqT settles 1.7 deg). X1 reviewer running.
- `claim1106_old.sh` (pid 27393): synced + built 11:22 (lab PC has the 7 worlds, 9 models, 28 configs); old-mass X4
  R3910-R3914 flying since 11:22 (~5 h), then X3 + bases, X3b chains, X2, rejoin, then chunks 6, 7, 8a, 8b, 8d, 9.
  Watcher on the X4 chunk's EXIT.
- X1 reviewer: SUPPORTED WITH CORRECTIONS (R3821c, feed F060: no faster settle; 0.1 deg more level at 8 quads; the T's dip
  -8 mm; the T cell depends on the masses). **Wesley ~11:35: X1 re-flies on the old masses too.** Staged outside the repo
  (~/mdc_overnight/x1_oldlinks_stage/: 2 worlds, 22 configs, oldlinks1106_4_x1.txt 44 runs, the tool with p115 support);
  `~/mdc_overnight/claim1106_x1old.sh` (pid file = the script) waits for claim1106_old.sh, installs, --check, syncs +
  builds, flies X1 old, then the configs in ~/mdc_overnight/void_refly.txt (edit that file, not the script). R3903's
  re-fly is moot (X1 re-flies).

## Fri 6 Nov ~10:30, Wesley: the claims WITH RETURNS re-fly on the pre-4 Nov masses (decisions DECIDED 2026-11-06)
- Built 10:30-10:38: tools/make_oldlink_worlds.py (same generator, old masses; worlds byte-identical to 9f20f8b's), 7
  *_oldlinks worlds + 9 models, configs/experiments_oldlinks/ (28 configs, same names), configs/batches/oldlinks1106_*
  (55 runs: X4 5, X3 + bases 32, X3b chains 4, X2 6, rejoin 8); CLAUDE.md, claim batch manifest section 11.
- The resume script (pid 3893) was stopped ~10:28 so X2 never started on the final masses; X1's runner (pid 3954) finishes
  chunk 4 (~12:30). Checking now: gate --quick + one laptop smoke run of the old-mass rj3 tip (expect ~17 s return).
- 10:49: gate --quick PASSED (10:42); smoke R3909 returned in 17.5 s (as R3908). **RUNNING: `~/mdc_overnight/claim1106_old.sh`
  (pid file = the script itself, log claim1106_old.log)**: waits for X1's runner (pid 3954), syncs + builds, the five old-mass
  sets, then chunks 6, 7, 8a, 8b, 8d, 9 on the final masses (~19 h). Watcher on X1's end: then score X1 (R3821, R3822,
  R3865-R3906). R3793's re-fly is moot (X3 re-flies).

## Fri 6 Nov ~09:50, Wesley: "focus on the sim runs now"
- X1 rest (42 runs, R3865-R3906) flying since 09:13 (~3.3 h); R3865's harness FAIL is the frozen clock after LANDED (its
  flight and run.csv are complete, as R3565). R3823/R3824 registered VOID (the restart). Watcher bv4ud1sxf on its EXIT.
- **X4 reviewer done (~10:00): SUPPORTED WITH CORRECTIONS** (numbers and verdicts stand; R3772c2, decisions CORRECTION,
  feed F056): the ring is 0.9 kg; the slower return's mechanism I gave was wrong (the old link already had its magnet at the
  end; cause = the link change by elimination); R4/tilt follow the massless plan. X3/X3b reviewer still running.
- **X3/X3b reviewer done: weak as written** -> R3779c, feed F057: three returns never welded (R3781, R3795, R3796), every X3
  return case falsified, C8 and N9 falsified as cards, N6 miss (x3b_metrics does not apply n6_ok), the slower return's cause
  NOT established (the 8/9-quad worlds already had the magnet at the end; tracker/dissipative code changed too).
- **A/B done 10:22 (R3907/R3908, feed F058, decisions FINDING):** the frozen code returns in 37.0 s with the final models and
  16.6 s with the 4 Nov models: the slower return is the model change, not code. Asked Wesley: report as flown (recommended)
  or look for a faster end hold before the freeze.
- (done) Exploratory A/B on the laptop (from ~10:05, ~/mdc_overnight/ab_links.sh, pid file = the script):** rj3_pairrelease_rejoin
  _L100_tip on the frozen tree (A, final links) and the same with the 4 Nov world + drone models (B, files outside the repo in
  ~/mdc_overnight/oldlinks/: rod 0.075 kg, airframe 0.475). Old tree: its returns took 16.6-17.7 s (R2594/R2595). A slow and
  B fast = the links; both slow = the code (then bisect the 4-5 Nov code); A fast = the slowdown needs more quads.
- (done) Reviewers: X4 (R3772-R3776, F054) and X3/X3b (R3779-R3820, F055). Their corrections go
  in correction rows and a feed entry before the thesis chat writes them up.

## RESUMED Fri 6 Nov 09:13 after the restart: `~/mdc_overnight/claim1107_resume.sh` (pid file = the script itself, log
   claim1107_resume.log): X1's remaining 42 (from R3865), then chunks 5-9, then R3793's re-fly; watcher on X1's EXIT.
- Funnel v2 (Wesley 09:1x: "make the edge 45 degrees, steeper, same horizontal distance"): cad/funnel/funnel.py v2 at 45 deg,
  mouth 174 x 144 mm as before, now 75 mm tall (was 56), 34 g; files in ~/Downloads/funnel/ (funnel_v2*.step/.stl,
  funnel_v2_45deg_cut.png; the old comparison renamed _OLD_v2_at_32deg). The catch recount (funnel_catch.py, cache
  ~/mdc_overnight/joins_cache) done: triangle plate 9, into the mouth / disc fully inside: v2 at 45 deg 20/24 / 11/24 (v1
  22/24 / 16/24; v2 at 32 deg was 22/26 / 10/26, kept in ~/mdc_overnight/scratch_1106/funnel_catch_v2_32deg.md).
- Funnel strength (Wesley ~09:20: "a fillet where the base meets the funnel ... how can we make it not break"): both
  variants now have a 15 mm outside round at the knee, a 2 mm lead-in round on the bore's top edge, the cone 1.6 mm ACROSS
  the wall (the first drawing's 0.8 mm sideways offset left 0.4-0.6 mm across: one printed line), a 2.4 mm bore wall, a 6
  mm base ring and four 2 mm ribs under the cone (0, +-96, 180 deg; clear of the bosses and the zip tie). Mass: v2 93 g
  (was 34), v1 121 g (was 40). Files and funnel_v2_strong_preview.png in ~/Downloads/funnel/. Asked: keep 93 g or lighter.
- **Funnel DECIDED (Wesley, this morning): he prints v2 (93 g) and v3** (v1's mouth at 45 deg, 97 mm tall, 1.2 mm wall, 2 radial ribs,
  97 g; plate-9 descents into the mouth / fully inside 22/24 / 15/24). decisions.md; ~/Downloads/funnel/funnel_v2.stl,
  funnel_v3.stl (+ _in_ring_frame.step, funnel_v2_v3_print.png). Thursday's nrtf: re-measure W with the magnet in the funnel.

## (09:13: resumed, see above) PAUSED Fri 6 Nov 08:30 for Wesley's laptop restart ("can you pause work")
- The claim batch was stopped cleanly in chunk 4 (X1): the script (pid 26001) first, then lab_batch with TERM, which
  removed its lab PC containers (none left). Done and fetched: R3821, R3822 (x1_h8_ours). Interrupted, no run.csv:
  R3823, R3824 (x1_h8_nors; register VOID). Chunks 1-3 are done, registered and posted (F054, F055).
- **Resume after the restart:** `cd ~ && setsid nohup bash ~/mdc_overnight/claim1107_resume.sh > ~/mdc_overnight/claim1107_resume.log 2>&1 &`
  (X1's remaining 42 configs in ~/mdc_overnight/x1_remaining_configs.txt, then chunks 5-9, then R3793's re-fly; no sync:
  the lab PC keeps the synced tree; new ids from R3865). Record the script's own pid. Re-arm the watchers and the X3b
  night bench (~/mdc_overnight/x3b_bench_night2.sh). Scratch files backed up to ~/mdc_overnight/scratch_1106/ (the
  scratchpad in /tmp is cleared by a restart).

## (before the pause) RUNNING: the final-tree claim batch, started Thu 5 Nov 18:25 (Wesley ~18:00: "Start running any additional simulation results I need")
- `~/mdc_overnight/claim1107.sh` (pid in ~/mdc_overnight/claim1107.pid = 26001, the script itself; 25999 is its bash -c wrapper; log ~/mdc_overnight/claim1107.log): one sync + build
  of the tree as it was at 18:25 (9f20f8b + 509 dirty; gate --quick PASSED 18:24, 2200; 135 configs --check ok), then
  extra1105_nros (sq_nr_orbit_L100_1104_rigargs_pause: the square on the orbit, rig prep for Thu 12 Nov), then chunks 1-9
  (docs/claim_batch_2026-11-07.md; ~22 h, 2 slots x 12 cpus). **THE TREE IS FROZEN: edit nothing under src/, tools/, configs/
  (configs/rig/ too: the one-tree hash reads it) or simulation_assets/ until the last chunk has started.** Docs, results/,
  cad/ and the scratchpad are fine. Stop: kill the script's pid, then the running lab_batch (it removes its containers).
- After each chunk: registry rows, the one-tree hash check (manifest section 5), the void re-flies, score (section 10), feed, ticks.
- Done since 18:25: R3771 (the square on the orbit) PASS, Thursday's nros cleared; the X4 chunk running (R3772-R3776; R3772/R3773 PASS at exit, 171-172 min each, so the chunk ends ~00:15; lab PC load
  ~85 on 24 cores is our two 8-quad slots, nobody else on it, 21:49). The
  k1ob_2 jerk EXPLAINED (the bigger object's impact pulled two magnets off; RIG-1105-k1ob_2c, feed F051); the planner's late
  carrier poses recur (nr9m_4, sq8_2: 2 of 19 flights; decisions FINDING; QoS look after the freeze); no frozen bars (Wesley, rules
  + memory + sheets); Te-Jen's notes updated (send: Wesley); handin_plan ticked.
- **X3b's laptop timing runs R3777/R3778 (R8b/R9b) VOID** (registered): the laptop cannot run 8-9 quads in Gazebo (s8: sim_t 38.9 s
  after 600 s wall, real-time factor ~0.065; s9 never published a pose), as on 31 Oct (R3451: RTF 0.088, R8b falsified; R3452
  void). The ground computer's evidence stays R3451 + the idle bench; the lab PC cannot stand in (the card is about tuf). **Queued on the laptop**
  (`~/mdc_overnight/x3b_bench_night.sh`, pid file = the script itself, log .log): the R8a/R9a idle bench (`tools/solve_scaling.py`
  n 2-10, ~20 min, out results/analysis/x3b_bench_res0_2026-11-05.json), from 00:30 once the load is < 0.5 (the desktop alone
  holds ~1.5 in the evening). Nothing heavy on the laptop from 00:30 until it logs "bench EXIT"; then register it (R8a/R9a-1105).
- Te-Jen's 5 Nov log zips: deleted by Wesley 6 Nov ~14:22.
  Too big for email (25 MB) or a chat attachment (30 MB): Wesley sends them from the laptop's Gmail (Drive links).
- Desk check of both rig sheets re-run on bf52a8a at 20:01: 18 OK, 0 refused (rig_lslow.sh gone with the slow descent);
  Wednesday's sheet says so.
- **Done 20:35: the table of every real-world descent of 4-5 Nov** (an agent; results/analysis/rig_joins_2026-11-04_05/,
  registry RIG-1105-joins + corrections RIG-1105-nr9o_2c, -fm9y_2c; feed F052, the thesis chat told). Triangle re-attach in
  hover: 31 of 37 at plate height, miss 6.2 cm median, 0 caught; the square 1 of 1. The notes for Te-Jen corrected (sq8_2's
  body 2-4 cm OUT, swing 4-6 cm IN, checked from the raw poses; the numbers of question 1). **If Wesley already sent the notes,
  he forwards the correction.** - Videos: Wesley's Mac videos are sorted by a chat on the Mac (brief + every rig flight's time window + a sorter script sent
  to him 5 Nov ~19:58; the scratchpad's videos/ folder has the builders).
- Wesley ~18:50: funnel v2 smaller and shallower (built: cad/funnel/funnel.py --variant v2; 174 x 144 mm, 56 mm, 27 g; v1 kept; both in
  ~/Downloads/funnel/; the 17-descent count of ~19:00 (v1 10, v2 8) was read at the wrong instant; recounted 20:37 from every descent where the
  disc crosses the mouth (results/analysis/rig_joins_2026-11-04_05/funnel_catch.md): triangle plate 9, into the mouth / disc
  fully inside: v1 22/24 / 16/24, v2 22/26 / 10/26 -> recommend v1 (geometry only)); NO ring-mass change for the funnel;
  match_dwell: DECIDED ~21:48, no 1.0 try (stays 0.5); Thursday's nrt line and script removed, desk check 17 OK (21:50).
- **X4 on the frozen tree (R3772/R3773 registered):** 26/26 swaps, no fault, but R7 misses (packs 695-707 s vs B 600): the rejoin
  takes 35 s vs 18 (the twin's magnet mass at the cable end since 4 Nov 23:15 settles slower). Wesley ~21:55: report as flown
  (decisions.md). Feed entry once R3774-R3776 (m115, fixed_B) are in (~00:15); expect slower rejoins in X2/X3/X11 too.
- **Reviews (Wesley ~22:24: "use a few agents to review ... thesis direction, changes over the whole thesis, recent changes")**,
  read-only, reports in the scratchpad's review/ folder, 2 agents at a time: wave 1 (from ~22:25) A the code that flies on
  Wed 11 / Thu 12, B the thesis direction and its evidence; wave 2 next: C the locked decisions and rules against the actual
  defaults over the whole thesis, D the rig sheets' readiness. Then a ranked summary for Wesley; fixes only on his word.
  Done: A (22:45; report scratchpad review/A_rig_code.md): ONE serious finding, checked by the lab chat: QUAD4's tracker has
  the LAND knobs only, no drop rule (graph_attach.py:877), so on the 55 deg drop lines a fast ring upset can fault QUAD4's
  tracker at 60 deg before the planner drops (34-185 ms on the rig) -> fleet EMERGENCY STOP in the air
  (fleet_manager_node.py:399-404). Fix: tracker_drop_options for QUAD4 (a gate change: Wesley's word, critic or waiver);
  build it after the claim batch's last chunk has started (the chunks' manifests record the laptop's diff), before Wed 11.
  Else no blocking findings (LAND defaults, mux checks, takeoff gate, console, every script argument; 94+6 tests pass).
  B (22:35; review/B_thesis_direction.md), checked: the draft's X4 is stale (1670 s, 2.8 B, 'no quad over its budget'),
  the abstract overstates the real re-attach (one catch), X1's B1/B2 falsified (the headline's own claim), X8 rim 'tilt
  peak' is the late tilt, best-of-N real fixed-team rows; recommendations in the report.
  C (22:51) and D (23:02) done; all four reports copied to docs/reports/reviews_2026-11-05/. Acted on (routine, docs only):
  Wed's leftover rig_nrt.sh deleted, Thu's k1z table, --slot-gate back on both sheets (16 Oct decision), desk check 16 OK
  (22:56); 9 memory notes, 4 CLAUDE.md lines, handin_plan / sim_runs_todo bookkeeping, one CLOSED line for 11 old OPENs,
  feed F053 (the ring of the 1 m flights); body 24 identity and first-LAND checks on Wed's sheet, no body 6 on Wed. The
  decisions for Wesley: decisions.md OPEN 2026-11-06 (the QUAD4 drop rule card docs/experiments/2026-11-06_quad4_tracker_drop_rule.md).
- **Overnight (Wesley in bed ~23:05; "are you able to work overnight"):** X4 chunk done 00:09 (R3772-R3776 registered, scored,
  feed F054, R3772c); the X3 chunk runs from 00:09 (R3779-, 24 runs; watcher on its EXIT); the X3b bench from 00:30 (watcher
  on pid 51018). Morning report: docs/reports/morning_2026-11-06.md. No push notifications at night.
- **05:27: X3 chunk done** (R3779-R3802, 24 runs; 23 PASS at exit; **R3793 VOID**: its flight ended (4 re-welds, LAND at
  t=232.8) but the runner's after-run parameter read-back hung and the 60 min cap stopped it before run.csv: re-fly
  x3_s8_chain4b2b_it5_tip x1 after the last chunk). Not registered or scored yet: the plan scores X3 with X3b's chunk (its
  bases); X3b chunk (18) runs from 05:27 (watcher on its EXIT). **One-tree check from chunk 2:** the hash reads
  e3b0c44298fc (an empty code diff) because bf52a8a (committed ~19:05) is now the laptop's HEAD; checked 05:28: all 217 code
  files of R3772's recorded diff are identical in bf52a8a and the commit has no other code change, so it is the same tree
  as ac9caea99661 (chunk 1).
- **08:15: X3b chunk done (R3803-R3820, 18/18 at exit); X3 + X3b scored and registered (42 rows), feed F055.** J misses in
  every X3 case with returns (the slower rejoin, as X4); C8 confirmed in hover and fig-8, its chain falsified by one P
  item; N9 hover confirmed, its chain falsified by J (fold-ins to 7 deg). X1 chunk (44 runs) from 08:15 (watcher on EXIT).
- The X3b bench did not start overnight: the laptop's load stayed 1.3-1.4 (desktop ~18 % + the batch's polling), never
  under the card's 0.5. Re-armed as ~/mdc_overnight/x3b_bench_night2.sh (pid file = the script): 00:30-06:30 only, each
  night, out results/analysis/x3b_bench_res0_2026-11-06.json.
- Waiting for Wesley: which funnel to print (v1/v2; recommended v1), Fri 6.

## Thu 5 Nov evening (from 16:47, after Wesley's restart)
- **Verify of the 4.3 join cells (tracking check 0.20 m):** R3766 PASS; R3765 joined (the landing's abort = the known twin
  artefact: QUAD4's own 60 deg envelope in the post-touchdown unwind, scored apart; R3765c); R3767 (pickup, low route) stalled
  0.7 m short of the drop point (R3767c) -> Wesley ~17:40: this cell flies OVER the carriers (an exception in CLAUDE.md);
  claim1107_8b now lists ..._noxy_over.
- **Rig LAND defaults (real.yaml):** land_descend_to_stall + land_rearm_touchdown + carriers_down_alone (15 cm), built ~17:35 with
  launch tests and the re-arm's logging should-fixes; gate PASSED (2200); CLAUDE.md and Wednesday's sheet (section 4b) say so.
- **Running 17:44:** lab PC batch (task beqc36fpd, configs/batches/verify1105b.txt): the pickup cell over the carriers, and
  cda_{b,d}_k1s_rearm (the stronger carrier with the re-arm on: no false touchdown expected).
- **Done 17:53:** R3768 (pickup over the carriers) PASS: dropped, re-attached, object in the net 3 mm off centre; R3769/R3770 (the
  stronger carrier with the re-arm on) touch down at +0.00, no flip; with carriers_down_alone each disarmed alone 0.6 s after: H1-H5
  hold. Registered; feed F050. **The weekend batch is ready** (docs/claim_batch_2026-11-07.md); Wesley's code freeze is the trigger.

## Thu 5 Nov afternoon: the main chat distributes the work to agents (Wesley ~12:45); the lab PC for simulation
Updated 16:17. **Order of work (handin_plan):** code freeze Fri 6 Nov -> every simulation claim re-flies on that tree Sat 7 -
Sun 8 (docs/claim_batch_2026-11-07.md: 255 runs, ~22 h, scorers tools/*_metrics.py proven on the old runs) -> simulation freeze
Mon 9 -> rig S3 Wed 11 / S4 Thu 12 (sheets docs/rig/wed_2026-11-11.md, thu_2026-11-12.md; freeze Thu 12 11:00).
- **Decided today (decisions.md 5 Nov):** QUAD4's tracker ring checks follow its mux (on); his TAKEOFF held until our release + no
  DISARM on an in-air refusal (rig); no loss-check timestamp skip (removed; the stamp-gap log stays); land_descend_to_stall = the RIG
  default (the slow descent deleted); the landing combination carriers_down_alone + 15 cm band (built, off; its 10 twin runs
  R3755+ flying; if clean -> rig default); the fold-in cap 'level' kept OFF, 'weld' deleted; the weekend batch: T joins int_k_xy 0,
  the gap table re-flies, no more 1.15 kg cells; Te-Jen's tracking check 0.20 m (margin 0.05; critic waived) + the plain floor-join
  on the low route (the three 4.3 join cells stalled at 0.05: R3738/R3749/R3750); rig sheets: M2A ring, Nr claim on the square,
  K at 1.2 m with drop_height 0.28 (k1z), Nr on the orbit typed; the small object for pickups.
- **Lab PC:** carriers_down_alone R3755-R3764 running (task b0qnqcefp); then verify of the three join cells (rebuild; task
  b1kel1r17, configs/batches/verify1105_x11_joins.txt); if they weld -> the weekend batch is ready.
- **Funnel (Wesley's attach aid):** cad/funnel/funnel.py (CadQuery in ~/venvs/cad_pkgs, PYTHONPATH), v1 40 g, bolts M4 through the
  inner-anticlockwise + both outer holes, nut pockets, the zip-tie hole clear; files in ~/Downloads/funnel/ (funnel_in_ring_frame.step
  lands on the 60 deg plate of his ring Part Studio). Waiting for his look in Onshape.
- **Still Wesley's:** the funnel v1/v2 (Fri 6). match_dwell settled (stays 0.5). Settled since: no frozen bars (rig too),
  no ring-mass change for the funnel, the code freeze committed (bf52a8a).
- Findings: sq8_2's carrier poses ~0.6 s late (cause not found); the triangle's misses radial; the square's T lean on today's tree
  differs from 01:30 only with the integral on (R3740 = R3709 with it off).

## Thu 5 Nov, the lab day (registry RIG-1105-*, decisions FINDING/OPEN 5 Nov, feed F048, notes docs/rig/notes_for_tejen_2026-11-05.md)
- **Worked:** k1d (pickup + drop into the flying ring, first try); **sq8_2: the first real detach and re-attach** (square 11/2/5 + QUAD4 on 8; contact_dz -0.03; proven, folded in 14.4 s after the release; the T leaned 21-28 deg; then a false loss in LAND dropped the ring); the orbit pause on the rig (nr9o_2); the bigger object dropped into the MOVING ring (k1ob_2, in the net 4 cm off centre) then the ring jerked to 54+ deg (drop) and **QUAD4's idle tracker disarmed QUAD4 (fell ~2.5 m): BUG, OPEN (a)**.
- **Cause of the morning's lift bounces: a Motive recording** (delay 45-65 -> 75-135 ms); rule in CLAUDE.md and the sheet.
- Typed today (params files in configs/rig/, scripts in ~/rig_*.sh and results/rig/2026-11-05/scripts/): drop 55 deg (attach_m1_1m_m2a_drop55*.yaml), the loss check 15 cm (`_det15`), his contact_dz -0.03, the orbit with no pause for the drop (`_orbitdrop`), the square on 11/2/5 + 8 with reattach_plate 8.
- Failed / open: re-attach accuracy on the triangle (nr9o_2, nr9m: 3.5-10 cm off); his TAKEOFF before the release (nr1r); pickups after body 6 was re-enabled off-centre (k1o_2); the false loss check in LAND (sq8_2).
- Simulation today: R3710 VOID (config), R3711 PASS (the drop into the moving ring in the twin, in the net 4 mm off centre).
- Disk: 596 MB -> 22 GB free (gz_record replays, OneDrive archives, the 4 Nov zips deleted; Wesley's OK).

## 4 Nov, the lab day, in one paragraph (registry rows have the detail)
1.15 kg ring: sq6b dropped at 33 deg in QUAD4's descent; switched to the **0.86 kg M2A ring** (body 8; W 0.0365 X 0.0039 Y 0.2271 measured 08:44, L 1.0525 typed). sq6m: **the first real join**. Then 13 flights: the **release worked on all six** that reached it (tilt 2.9-5.2 deg; the take-back worked every time); **no re-attach finished** (nr1_2 on plate 1: caught within 4 mm, plate rose 18-19 mm, rejected on the 3 mm taut margin; nr9m_4/_5 on plate 9: contacts 24-59 mm off, declared by the 60 mm radius, lost); **no pickup drop** (k1m_2-4: the object on its 1 m cable swung 0.23-0.26 m from the centre, his arrival check 0.18 m held 0.5 s never passed; k1m_3: his reference sank to 2.01 m, LAND's take-back lifted the object with the ring: 30.4 deg, automatic drop). nr9, nr1_2, nr1_3 flew the 1.15 kg config on the M2A ring by mistake; nr1, nr1_3, nr9m_3 VOID. Every flight's command line is in `results/rig/2026-11-04/launches.csv`.

## The command blocks of 4 Nov (M2A ring), for Thursday
env `MDC_RIG_ACADOS_ROOT=$HOME/mdc_rig_generated_m2a_0p86`; `AM` = attach launch with `params_file:=$C/attach_m1_1m_m2a.yaml` (`_around` for the route); `geometry:=rig_m2a` in partner_args; K 0.06 (**too loose: see the OPEN line**); G, T3, K1 as in `docs/rig/archive/wed_2026-11-04.md` + the loosened G (decisions.md 4 Nov ~10:00). Lines: sq6m (floor attach), nr9m (`partner_attached:=true detach_unload_s:=3.0`), nr1r (+ `reattach_plate:=1`, `plate:=1`, `_around` params), k1m (`mission_mode:=pickup_delivery`).

## Wrong advice given 4 Nov (do not repeat)
- `ros2 topic pub /tejen/magnet/command OFF` or `/drone_3/magnet OFF` does NOT release QUAD4's magnet during his pickup: our magnet_gate holds his OFF in TRANSIT_TO_DROP_POINT and ignores ours before our span (k1m_3, k1m_4 logs). There is no working operator drop yet (OPEN proposal 3).
- Read `launches.csv` and every log before advising; the state file must list every flight.

## Code changed 3-4 Nov (committed in bf52a8a)
- `dissipative_node.py`: `release_route()` + `partner_release_around` / `_r` (default off); test `test_release_route.py`. R3633, R3634.
- His planner (our fork): the takeoff from the air (unswitched; decisions 4 Nov); test `src/tejen_mission/test/test_takeoff_from_air.py` (3). `contact_descent_hold` (3 Nov).
- Typed knobs: the co-motion set (graph_attach COMOTION_KNOBS); his launch `descent_abort_speed`, `descent_hold`, `descent_gross_xy`, `pickup_proof_drift/_speed/_loss` (test `test_pickup_proof_only_when_typed`). CHANGES_FROM_TEJEN.md has 4 Nov with notes for Tejen.
- Gate `--quick` PASSED ~11:20 (2096 tests; on the tree before the operator OFF, whose tests pass on their own).
- **Operator OFF (built 11:25, default off):** `magnet_operator_topic:=/drone_3/magnet_operator` on the attach line; then `ros2 topic pub -t 3 /drone_3/magnet_operator std_msgs/msg/String "{data: 'OFF'}"` drops whatever QUAD4 holds, in any phase, and holds every ON until an operator 'ON'. magnet_gate.py `operator()`; tests in test_magnet_gate.py (2) and test_real_mode_launches.py (1). Not desk-checked on the rig launch yet.

- **Drop on arrival (built 11:30, default off):** `drop_on_arrival:=true [drop_on_arrival_xy:=0.12]` in partner_args; our fork of his planner releases at the transit height once the magnet is within the radius of the drop point for 0.1 s. Tests test_drop_on_arrival.py (4), test_partner_mission_rig.py (1); his suite still 24 old failures, 1159 pass. Simulation **R3675 PASS** (registry): dropped at 2.37 m, in the net 3.5 cm from the centre, ring 5 cm / 3 deg; QUAD4 bounced to 2.61 m after the release (over the 2.6 m ceiling: Wesley's call, on the sheet); not starved (wall as R2899/R2900).
- **Thursday's sheet rewritten 11:33** for the M2A ring (`docs/rig/archive/thu_2026-11-05.md`): calls (K 0.025 and drop on arrival marked Wesley's word), re-attach, pickup k1d, release x2, triangle and square baselines; new `configs/rig/traj_hold_m2a.yaml`. **Desk check PASSED 11:48-11:55** (scratchpad desk1104/sweep.sh, domain 79): all 10 lines up, no refusal, read-backs as typed; solvers for the layouts built into ~/mdc_rig_generated_m2a_0p86. Re-check only if the tree changes.
- Not built, needs Wesley: the square detach/re-attach in simulation needs a new world (carriers 0/3/6, QUAD4 welded on 9, M2A ring): a world SDF is his word. The take-back dropping a held object (sheet rule instead: operator OFF before LAND). Battery compensation in his controller (the twin has a pack model: testable).

- **Te-Jen's review (~14:00) and what followed:** his OCP caps throttle at 0.60; with the object QUAD4 sat at the cap (17 s in k1m_3) and sank under his reference, which held 2.43 m (my "reference sank" was wrong: corrected in registry RIG-1104-k1m_3c, feed F041, decisions CORRECTION). `tools/rig_to_run.py` now adds d3_partner_ref_*, dN_partner_thr, dN_radio_thr, dN_mux from the bag (tests pass; old run.csv files are not rebuilt unless rig_summary re-runs). Built, off unless typed: `mpc_throttle_max:=<0.60-0.95>` (tejen_mpc/throttle_bound.py, run time, no rebuild), `lift_full:=true` (up, then over). Tests: test_throttle_bound.py, test_lift_full.py, test_partner_mission_rig.py; his suites: 24 old failures + his controller's 2 old style failures. Simulation **R3676 PASS** (k1_5_lift_full, all three switches): left the pickup at 2.43 m (R3675 1.31 m), level after, nearest cable 0.25 m (0.21), in the net 2.4 cm from the centre, QUAD4 bounce 2.63 m. Thursday's K1 now types `lift_full:=true mpc_throttle_max:=0.8`; desk re-check of nr9m/nr1r/k1d passed 15:29 (after `ros2 daemon stop`: the daemon breaks param reads, `!rclpy.ok()`). Reply drafted: `docs/rig/archive/notes_for_tejen_2026-11-04_reply.md`. Next for Te-Jen: the forecast churn (ring twist input), drop on arrival on a predicted landing point, the release height over the net (measure the net).
- **The square (Wesley: build and run):** the worlds already existed (27 Oct). **R3677 PASS** on today's tree: square -> T -> square, re-attach first try 25 s after the release; ring 21.6 deg peak 0.7 s after the release, the T ~7 deg, then 1 deg. Thursday's sheet section 6 (sq9), desk-checked 15:45, marked Wesley's call with a suggested stop rule for its first swing.
- **Battery compensation built** (default off, typed `mpc_battery_comp:=0.022`): his radio throttle + 0.022 per volt below 23.5 V (cap +0.10), after his partial-step copy-back so his OCP/UKF keep the model throttle; twin controller now remaps telemetry. Unit tests pass; Simulation A/B running on the laptop from 15:50: nr_p9_pack218 (21.8 V, no term) then nr_p9_pack218_bc (with it), task bs0f8u5pk; keep the laptop quiet at each start (~15:50, ~16:01). I edited his controller and ran tests during R3677's first minutes (rule broken; R3677 checked: no import error, not starved).
- Zips for Te-Jen made: ~/wesley-rig-2026-11-04.zip (22 MB), ~/wesley-rig-2026-11-04-bags.zip (272 MB).

## Wed 4 Nov evening, 22:20-23:30 (decisions.md has every line; feed F045, F046)
- **Rod in the plan: flipped on 22:20, back OFF 22:50; Wesley ~23:00: the plan stays MASSLESS everywhere.** My 22:50 physics was wrong (CORRECTION line): the magnet's weight hangs on the cable while the plan books it on the drone, on the rig too, so the T leans on both (a limitation). **The twin's links now match the hardware** (rod 0.002 + magnet 0.038 at the plate, airframe 0.510; 80 world files regenerated, all generators --check ok; snapshot scratchpad simulation_assets_pre_link_2026-11-04.tgz). Every twin run before 23:15 flew the old link. Rod-in-plan configs with the old 0.075 rod now fail --check (retired).
- Weld filter 0.1 s the sim default (stays). X2's later-runs bar 5 deg. OPEN 761 closed.
- **Built (default off/as before):** `cable_pull_up_max` (tracker bound, 0); `ring_low_land_m/_s` (planner; proposed 0.30 m / 3 s from the replay of every rig flight: fires only on h3tp since 7 Oct, on R3444 at 815.8 s). The clamp's runs 3-6 (rod-in-plan T's) CANCELLED; its question on the massless T is open (the analysis: no claim cell's rider is on it).
- **Reviewer, X2-full and X8: supported with corrections** (cards; F045).
- **Orbit: R3696 PASS** (the rig's partner args): the circle runs on after the release, his TAKEOFF stops it, re-weld first try 12.8 s after the release, tilt peak 2.1 deg. On Thursday's sheet as section 5 (nr9o, Wesley's call: `contact_rig_orbit:=true`), params `configs/rig/attach_m1_1m_m2a_orbit.yaml`; desk-checked 23:09. R3685 VOID (his C++ transit stalled; not the rig's args).
- Desk check of every Thursday line on the final tree passed 22:40-22:46 (the rig launch does not read the twin worlds).

## 4 Nov's Running / To do / Waiting lists: superseded (5 Nov night, after review C)
Every item was done or decided by 5 Nov (decisions.md 4 Nov 22:20-23:30 and 5 Nov); what runs now is in the RUNNING block
at the top. The old text is in git (bf52a8a's docs/SESSION_STATE.md).

## How to work (Wesley)
- In the lab: work fast, give full, checked command blocks; read the log before every answer. Agents: at most 1 overnight, 2 otherwise. Critic waived for all (3 Nov). No bars frozen for simulation. Commits on his word only, never push.
- Lab batches: `tools/remote/lab_batch.py`, `setsid nohup`, scripts in `~/mdc_overnight/`; `--cpus` at most 12; chain on a PID; edit `src/` with an atomic copy-and-rename when a launch or sync may start (symlink install).
- Rig: Motive streams 6,8,11,12,13,14,24 (attach); 8,11,12,13,14,21,24 (release, square). The first launch after a layout change rebuilds the solver for 20-40 s.

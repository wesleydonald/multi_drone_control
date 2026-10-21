# Session state (read after every compaction; keep it short and current)

Updated: Thu 22 Oct 2026, 00:14 AEDT (from `date`). Only what a new session needs to act. History: `git log -p docs/SESSION_STATE.md`,
`docs/decisions.md`, `results/registry.csv`, and `docs/archive/`.

## Where things are (21 Oct)
- **Main tree** `~/multi_drone_control` on **day-1021** (cf556ff; from day-1016 at 410f0f1). day-1016 is NOT merged
  into week5 (4b3560e) yet: Wesley merges. Commits by explicit path, authored by Wesley, no trailers, never push.
- **Current plan:** `docs/plan_thesis_results_2026-10-21.md` (approved 21 Oct ~18:50; status table at the top).
  Decisions: `docs/decisions.md` DECIDED 2026-10-21.
- **Worktrees:**
  - `~/mdc_wt/tejen_sync2` (tejen-sync2 6483d90): Tejen's fd495db0 merged into our fork, avoidance on the transit and
    floor_join, `m1_irl_pickup.launch.py`. Not merged into day-1021; merge before the basket runs (plan s5).
  - `~/mdc_wt/tipdetect` (tip-detect b4356aa): the magnet-marker release check **rev 3** (critic round 2's guards:
    ring-glitch hold-off, the distance-from-centre test, 0.1 s on stamps, lone-marker would-fire, frozen markers,
    learning without the ring speed, slack-cable band, forget on attach/detach), the twin ring tilt fault
    (`MOCAP_TILT -1`), `tools/tip_replay.py`, `tools/mocap_glitch_scan.py`, the tracker's raw-pull columns, the rod-pose
    publishers in the existing worlds (Wesley ~23:30). Unit tests 16/16, planner suite 234. **Critic round 3 running**
    (background agent, its verdict goes into the card). Bars frozen 23:45 (card, rev 3 section).
  - `~/mdc_wt/thesis` (thesis-rig): the thesis.
- **Lab PC:** full all evening (Ryan's 11-12 Gazebo sims, load ~107-129 on 24 cores). Everything ran on the laptop.

## Now (Thu 22 Oct, 00:14): Wesley asleep; the 20-min loop (cron job 73b7592d) runs the overnight queue to 07:30
- **Loop prompt:** merge, comparison, scoring, sheet, queue, renders, morning report by 07:15, then stop the loop
  (CronDelete 73b7592d). Wesley ~00:00: work through as much as fits before 07:30; add the 1 m detach/attach runs,
  trajectories, and his pickup with the basket.
- **Running:**
  - the solve sweep `tools/solve_scaling.py` (pid 118508; its 2 h timeout removed at Wesley's word; n 2-9 timed
    clean, rows in `scratchpad/overnight.log` when it exits). The n=10 build is still compiling; its timing will
    overlap twin runs, so re-time n=10 alone later from the cached build (`--n 10`).
  - `scratchpad/overnight2b.sh` (task bpu47gjt6): the six 4-quad 1 m re-runs, on the main tree before the merge.
    overnight.sh and overnight2.sh were stopped (only their waits); the 1 m set's videos are rendered later.
- **Next, when overnight2b ends:**
  1. Merge tip-detect (c56ad44 + configs) into day-1021, apply the 8 N cap (`cable_accel_cap` 14.5), pytest,
     `tools/gate.sh --quick`, commit.
  2. `scratchpad/overnight3a.sh`: the comparison (8 runs, card rev 3 bars frozen 23:45 plus round 3's added bars).
  3. Score it (`scratchpad/score_tip.py`), set Part C's line on the sheet.
  4. `scratchpad/overnight3b.sh`: the A/B tipon x6, the basket at 0.55 m and 1 m, the 1 m floor join, the
     announced 1 m detach in hover and mid figure-8, the check's neighbours, second 1 m basket runs.
  5. `scratchpad/render22.sh` (+ the 21 Oct 1 m set) and the reel by 07:15; the morning report.
- **Critic round 3** (in the card): GO for the twin comparison. Part C ON only after N1-N4:
  - N1: the held-swap scan (`tools/marker_swap_scan.py`, to run tonight);
  - N2: the replay's 50 ms run bar (built);
  - N3: the rig predicts detection ~0.3 s after the OFF;
  - N4: the sheet's ON procedure (to write after the comparison).
  The rider-only slack band is built (17 tests, 235 in the planner suite).
- **Still to score:** the 1 m baselines R2259-R2270 (`scratchpad/score_1m.py`), and R2271-R2276 (void: num_drones
  3 in the io launch).

## Earlier today (21 Oct)
- **Rig day 08:23-11:05** (`docs/reports/rig_2026-10-21.md`, RIG-1021-*):
  - QUAD2's magnet let go 6 times, and its link reads ~2 cm short (bench check on the sheet);
  - the no-hold offset of 5.5-6.6 cm was mostly the ring's mocap centre, 1.7 cm off. Wesley moved the ring pivot in
    xy (RIG-1021-ringpivot); expected next no-hold hover 0.5-2 cm (sheet A1);
  - ring body tilt 1.3 deg; the floor or Motive's ground plane 0.9 deg (spirit level on the sheet).
- **Te-Jen:** QUAD1 pickup on a 1 m cable with the new 0.15 kg object (RIG-1021-q1pick: picked up first try; the lift
  lagged with his kT feedback; LAND drops the object). Draft message to his Codex written in chat.
- RIG-1015-a1c: on 15 Oct the tether struck QUAD2.

## Earlier (16-18 Oct)
- Morning reports `docs/reports/morning_report_2026-10-1{6,7,8}.md`; decisions 16-17 Oct in decisions.md.
- The 0.55 m and 1 m bands are frozen (card 2026-10-16_bands_1m). Touchdown A is on in the twin; the rig waits on a
  critic look. Corner cutting is closed (lag reported).

## 14 Oct rig day in short (detail: registry RIG-1014-*, `git log -p` of this file)
- 46 launches: 8 passed, 11 failed in flight, 11 arm refusals (QUAD1/QUAD2 FC attitude drift), the rest void.
- t5_2 crashed (three-carrier lift bobbing with starved loops; drop_on_loss now on in attach). lt_13 F1 PASS after the
  load cuts and the lean rclpy switch.
- His pickup failed on contact time (t5_4, t5_5, t6); floor_join flown t6f..t6h5_3. t6h_4: the magnet caught plate 9
  and our old proof's taut check rejected it (fixed by WP3).
- Two operator DISARMs with QUAD4 ~0.3 m up on its own landing (fixed by WP8: it now disarms alone a few cm up).
- Notes for Tejen: docs/tejen/notes_2026-10-14.md; his logs zipped at ~/tejen_attach_logs_2026-10-14.zip.

## Rig sheets and twin evidence
- Tomorrow: `docs/rig/thu_2026-10-22.md`. Earlier sheets: `docs/rig/wed_2026-10-21.md`, `docs/rig/wed_2026-10-14.md`.
  Every terminal: the setup line on the sheet (main tree, ROS_LOCALHOST_ONLY=1, ROS_DOMAIN_ID=23).

## Open for Wesley
- Merge day-1016/day-1021 into week5; merge tejen-sync2 before the basket runs.
- Tonight's report decisions: the chain's step-2 card (after the rig); the CoM fix (done on the rig before Part C).
- Older: A on the rig after its critic; the no-floor F3 bar; the orbit-join drops (morning report 18 Oct).

## Working constraints (beyond CLAUDE.md)
- **Helpers:** one general-purpose helper agent at a time (Wesley).
- **Lab:** at most 4 slots, evenings only (Wesley 21 Oct: use it at the same time as the laptop when free), one tree at a time. Check idle first (`who`, load, containers). Each batch
  runs from a script in a fresh `env -i` shell sourcing ONE install; start it detached; kill exact PIDs only.
- **Scorers:** `tools/m1_score.py claim|full|takeback|tilt|tb14 RUN --rev N [--check]` (week5). It prints one LF row:
  back up the registry, append the row yourself, then diff the old lines. Wrapper: `$CLAUDE_JOB_DIR/tmp/sc.sh`.
- **Permissions:** deletions (branches, worktrees), git push and switching the main tree are refused for the
  assistant. Write a script for Wesley.
- **Untracked local files:** `.claude/*` and `docs/claude-setup-proposal/`. Edit them in place; never `git add` them.
- **Timestamps:** read `date` before writing any clock time.
- **Branch rule:** delete a branch once its work is on week5 or the attempt failed. Archive-tag any unmerged SHA the
  registry cites.

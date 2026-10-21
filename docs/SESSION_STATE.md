# Session state (read after every compaction; keep it short and current)

Updated: Wed 21 Oct 2026, 22:51 AEDT (from `date`). Only what a new session needs to act. History: `git log -p docs/SESSION_STATE.md`,
`docs/decisions.md`, `results/registry.csv`, and `docs/archive/`.

## Where things are (21 Oct)
- **Main tree** `~/multi_drone_control` on **day-1021** (cf556ff; from day-1016 at 410f0f1). day-1016 is NOT merged
  into week5 (4b3560e) yet: Wesley merges. Commits by explicit path, authored by Wesley, no trailers, never push.
- **Current plan:** `docs/plan_thesis_results_2026-10-21.md` (approved 21 Oct ~18:50; status table at the top).
  Decisions: `docs/decisions.md` DECIDED 2026-10-21.
- **Worktrees:**
  - `~/mdc_wt/tejen_sync2` (tejen-sync2 6483d90): Tejen's fd495db0 merged into our fork, avoidance on the transit and
    floor_join, `m1_irl_pickup.launch.py`. Not merged into day-1021; merge before the basket runs (plan s5).
  - `~/mdc_wt/tipdetect` (tip-detect 75b1ec2): the magnet-marker release check rev 2, the tracker's raw-pull log
    columns, twin rod-pose markers. Critic round 2 running. Merge into day-1021 only after the overnight batch.
  - `~/mdc_wt/thesis` (thesis-rig): the thesis.
- **Lab PC:** full all evening (Ryan's 11-12 Gazebo sims, load ~107-129 on 24 cores). Everything ran on the laptop.

## Now (Wed 21 Oct, 22:51): rig Thu 22 Oct 8-11 am; overnight running on the laptop
- **Running:**
  - `overnight.sh` (pid 76740): the solve-time sweep `tools/solve_scaling.py` (n 2-10, iterations 5/2/1; built to n=8
    by 22:42; JSON to `results/analysis/solve_scaling_<host>.json`), then the 1 m set's videos and reel.
  - `overnight2.sh` (pid 121269): waits for it, then the six 4-quad 1 m runs again and their reel.
  - Both stop starting new work at 06:30.
  - Logs: the scratchpad's `overnight*.log`.
- **Done overnight so far:**
  - the evening reel `results/videos/reel_2026-10-21_evening.mp4` (Wesley watched it: "look good for the most part");
  - 1 m baselines R2259-R2270: twelve 2- and 3-quad runs passed, still to score;
  - R2271-R2276 (4 quads) failed at startup: the io launch typed num_drones 3, now fixed in every b2/b4 config. The
    2-quad runs R2244-R2248, R2255, R2259-R2264 also ran one spare emulator; no effect.
- **Tonight's twin, scored** (registry R2241-R2258):
  - line_spin flies (3 quads 1.6-1.7 cm, tilt peak 3.4 deg at the end of the return leg; 2 quads 1.5-1.7 cm);
  - 2-quad hover steady (0.3 deg); the figure-8 rocks 6-8 deg p-p then decays;
  - square-to-T unannounced release: 57 deg peak (R2249), dropped at 45 deg with the rule (R2254);
  - T to pair under `drop_gap_deg` 200: detected (+0.8 s) and resized, no drop, but the pair rocks 7.8-8.4 deg p-p
    (card 2026-10-21_chain_release: arm FAILED its frozen 5 deg bar);
  - the rig's attach_z gap wrecks the T (27 deg, R2253) but not the plain pair hover (R2255) or the 3-quad hover
    (R2257);
  - the tether-mass model: T 4.4 deg, and that release went undetected (R2258).
- **Peak cable pulls tonight** (trackers' measured pull from mocap, applied value; `scratchpad/peak_pull.py`): 3 quads
  3.6-4.1 N, 2 quads 5.2-5.6 N, at most 6.3 N. All under the magnets' 10-12 N.
- **Wesley's word tonight (~23:00):**
  - **Part C (T to pair) stays on tomorrow's sheet, last.** Gated on the ring centre-of-mass fix first (he will do
    it), B1 clean, and the detector branch.
  - A released magnet on the rig "will instantly ... absolutely detach because of the angle the quad would be at", so
    the marker leaves its plate at once.
  - **The cable-pull cap goes to 8 N:** `cable_accel_cap` 12 -> 14.5 m/s^2 in `src/bringup/config/common.yaml:76`. To
    apply AFTER overnight2 (so the 1 m set flies one setting), before the rig.
  - The tension IS measured from mocap (the trackers' `cable_source mocap`).
- **Before the rig (from ~06:30, when the batches have stopped):**
  1. Score R2259-R2270 and the 4-quad re-runs, plus their registry rows.
  2. Apply the 8 N cap, then gate.
  3. If critic round 2 passes: merge tip-detect into day-1021, gate, fly `t3_hover_tip`, `t3_pair_release_tip` x2,
     `t3_pair_release_tipoff` and `c4_release9_tip`, and set the sheet's status line. Otherwise Part C uses
     `chain_pair_noxy.yaml` (stretch check) with Wesley's `/fleet/detach 0` call.
  4. Fill in the solve-sweep table; commit.
- **Tomorrow's sheet** `docs/rig/thu_2026-10-22.md`:
  - bench item 0: move the ring's Motive pivot down 3.1 cm to its centre of mass, `attach_z` 0.02, and
    `payload_rest_z` 0.02 typed on every T2 line;
  - magnet markers 22-24 routed (`mocap_extra_bodies`) and bagged on every flight;
  - Part A: the offset check, then the 3-quad hover, figure-8 and line_spin at 0.2;
  - Part B: the 2-quad hover, figure-8 and line_spin;
  - Part C: the T to the pair, QUAD2 on plate 3 as the leaver.
- **Chain step 2 (card next, after the rig):**
  - a force-vector reference split with the tether mass (the T level; the rider ~0.70 N);
  - a reaction to a release within ~0.1 s: re-plan and move the opposite quad, likely bypassing the trackers' 25 m/s^3
    pull slew;
  - the marker cue;
  - the rig ring at its centre of mass;
  - the twin's rigid rods may exaggerate the roll against slack rig cables.

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

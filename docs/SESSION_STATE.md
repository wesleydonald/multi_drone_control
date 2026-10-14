# Session state (read after every compaction; keep it short and current)

Updated: Thu 15 Oct 2026, 06:59 AEDT (from `date`). Only what a new session needs to act. History: `git log -p docs/SESSION_STATE.md`,
`docs/decisions.md`, `results/registry.csv`, and `docs/archive/`.

## Where things are (12 Oct tidy)
- **One tree:** `~/multi_drone_control` on **week5**: code, docs, the registry and the working rules. Built, prebuilt,
  gate --quick passes (609 tests). rig-prep is retired (merged). week5 is the testing tree for the rig and the twin.
- **Layout (12 Oct):** `docs/` top = state, decisions, the manual, the plans; `docs/rig/`, `docs/tejen/`, `docs/reports/`,
  `docs/analysis/`, `docs/experiments/` (cards), `docs/figures/`, `docs/design/`, `docs/archive/` (everything finished;
  a registry row's `docs/X.md` is `docs/archive/X.md`). `configs/experiments/` = the 106 configs current cards, tonight's
  batch or tests use; `configs/experiments/old/` the rest. `tools/old/` = one-off study scripts. `generated/` = every acados solver and
  the harness PNGs (was the repo root; MDC_ACADOS_ROOT on the lab).
- **Other worktrees:**
  - `mdc_wt/thesis` (thesis-rig): the assembled thesis, 122 pp. Ch2/Ch3 rewritten; the Ch4 skeleton has F8/F10/F11/F14,
    and T7/T8 are filled after a reviewer.
  - `mdc_lag` (time-cascade) and `mdc_safety` (safety-defaults) are Wesley's.
- **Backups:** every branch and tag is on GitHub under `backup-2026-10-12/*` (verified), and in
  `~/backups/multi_drone_control_2026-10-12.bundle`. Flight data (`results/`, 34 GB), `docs/papers` and his twin logs
  are local only.
- **Archive tags:** archive/m1-tiltcut, archive/m1-e0, archive/m1-full, archive/combined-m1, archive/m1-video.

## Now (Thu 15 Oct, 06:59): ready for today's lab (15:00-18:00)
- **Tree:** week5 at 8862e6d, clean; quick gate passes (1453). Plan `docs/plan_after_rig_2026-10-14.md`: rows 0-8 done
  or ready; row 9 (after the lab) to do. Today: sheet `docs/rig/thu_2026-10-15.md`, card
  `docs/experiments/2026-10-15_attach_comotion_rig.md` (rev 5), morning report `docs/reports/morning_report_2026-10-15.md`.
- **Tonight's commits (in order):** c15abc4..a0d3963 (14 Oct work), c1ebdc9, 3297746 + dade70d (WP8 arming/landing),
  eacab58 (WP3 attach check), 4a60ca3 (RViz markers; its renumbering reverted), a73d9a4 (console, quiet T2), f9d0b72
  (RViz panel), 657fee1 (attach_window), 4beb32f (trim_speed_gate), bfbd7a9 (plates clockwise again), f035625 (critic
  rounds 1-2 fixes), 305e50c (typed/shown angles clockwise, same names flipped), 33ceec4 (console mid-flight),
  a5bf17d, 0e47f9b (Tejen's first review), 496c11f, 76180f8 (Tejen's second review), 8862e6d (R1722).
- **Twin:** R1718 PASS, R1719 FAIL (led to rev 2), R1720-R1722 PASS (R1722 on 76180f8: contact, proof on his lift signal
  only, fold-in at plate 9 / 270 deg cw, clean landing).
- **Tejen:** `~/tejen_review_package_2026-10-15.zip` (19.5 MB, 76180f8 + the t6h_4 bag + README answering both of his
  review rounds): ready; Wesley sends it. Tejen OK'd his trim at 0.20 m/s above contact (aim B); keep the lift proof;
  no M2 modes, no TAKEOFF change. His puck height 37.9 / 36.2 mm vs 34.5 assumed (for WP9).
- **Wesley's decisions before 15:00:** QUAD4's aim A (t6h_4 offset, trim off; the sheet's default) or B
  (`trim_speed_gate:=0.20`); send `docs/rig/fc_request.md`; send the zip.
- **At the lab (the sheet's order):** props-off console + RViz panel check (the panel is compile-checked only), the proof
  bench by hand (negatives under `dry_hold:=true`), FC acceptance if the FCs were changed; one dry-hold flight; up to
  four attach flights, each scored with `python3 tools/proof_replay.py --catch <run> ...` before the next.
- **Offered, awaiting Wesley's answer:** a twin config that switches the bottom hold on (it is rig-only, so its
  quiet-miss and plate-frame logic has never flown in the twin), ~30 min incl. one run.
- **After the lab (plan row 9):** WP4 wind matrix only if contacts stay rare (supervisor's list, my order: 90 mm discs,
  weight on the magnet, 1 m tether; 45 deg carriers only with stronger carrier magnets; hover-only configs not written);
  WP7 rename (our planner into mpc_planner, mode detach, network controller off week5 to `legacy`, the gate runs the
  ~170 rig-path planner tests); WP9 pickup / full mission.
- **Supervisor answers already given (chat 14 Oct night):** FC accelerometer logs help the arming issue more than the
  attach (mocap gives the body acceleration); 90 mm discs: inside the catch 35 % vs 16 % of the 14 Oct hold time.
- **Scratchpad:** the earlier session's scratchpad is gone; temp files go to /tmp.

## 14 Oct rig day in short (detail: registry RIG-1014-*, `git log -p` of this file)
- 46 launches: 8 passed, 11 failed in flight, 11 arm refusals (QUAD1/QUAD2 FC attitude drift), the rest void.
- t5_2 crashed (three-carrier lift bobbing with starved loops; drop_on_loss now on in attach). lt_13 F1 PASS after the
  load cuts and the lean rclpy switch.
- His pickup failed on contact time (t5_4, t5_5, t6); floor_join flown t6f..t6h5_3. t6h_4: the magnet caught plate 9
  and our old proof's taut check rejected it (fixed by WP3).
- Two operator DISARMs with QUAD4 ~0.3 m up on its own landing (fixed by WP8: it now disarms alone a few cm up).
- Notes for Tejen: docs/tejen/notes_2026-10-14.md; his logs zipped at ~/tejen_attach_logs_2026-10-14.zip.

## Rig sheets and twin evidence
- 14 Oct sheet: `docs/rig/wed_2026-10-14.md`. Thursday's sheet is written from the plan's status on Thursday (anything
  unfinished keeps 14 Oct names and plate numbering). Every terminal: `w5` (main tree, ROS_LOCALHOST_ONLY=1,
  ROS_DOMAIN_ID=23).
- Twin evidence for the rig: the claim batch on rig-wed (R1638-R1681) and the 12 Oct batch on week5 (R1682-R1713,
  docs/reports/morning_report_2026-10-13.md). Lab PC free; scripts for a lab batch in `~/handoff_2026-10-12/`.

## Open for Wesley
- After Wednesday: the T6 amendments A1-A4 (explained again with Wednesday's numbers; decisions.md 12 Oct).
- After tonight: Tier A's announced half (the measured soft release does not unload the leaver).
- From the offline analyses (decisions.md OPEN 12 Oct): the rig null, the stretch threshold, the rig ramp pair.
- The counter-swing filter: not now (docs/analysis/counter_swing_2026-10-12.md).

## Working constraints (beyond CLAUDE.md)
- **Helpers:** one general-purpose helper agent at a time (Wesley).
- **Lab:** at most 4 slots, evenings only, one tree at a time. Check idle first (`who`, load, containers). Each batch
  runs from a script in a fresh `env -i` shell sourcing ONE install; start it detached; kill exact PIDs only.
- **Scorers:** `tools/m1_score.py claim|full|takeback|tilt|tb14 RUN --rev N [--check]` (week5). It prints one LF row:
  back up the registry, append the row yourself, then diff the old lines. Wrapper: `$CLAUDE_JOB_DIR/tmp/sc.sh`.
- **Permissions:** deletions (branches, worktrees), git push and switching the main tree are refused for the
  assistant. Write a script for Wesley.
- **Untracked local files:** `.claude/*` and `docs/claude-setup-proposal/`. Edit them in place; never `git add` them.
- **Timestamps:** read `date` before writing any clock time.
- **Branch rule:** delete a branch once its work is on week5 or the attempt failed. Archive-tag any unmerged SHA the
  registry cites.

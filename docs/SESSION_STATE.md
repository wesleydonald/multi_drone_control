# Session state (read after every compaction; keep it short and current)

Updated: Fri 16 Oct 2026, 08:20 AEDT (from `date`). Only what a new session needs to act. History: `git log -p docs/SESSION_STATE.md`,
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

## Now (Fri 16 Oct, 08:20): day branch `day-1016`, Wesley at work until the evening
- **Merged:** overnight-1015 into week5 (fast-forward to 4b3560e, Wesley ~07:40). Today's work commits on `day-1016`
  (from week5; Wesley merges tonight). Overnight in short: `docs/reports/morning_report_2026-10-16.md`.
- **Decisions this morning** (docs/decisions.md, 16 Oct, two DECIDED lines): drop_takes_back ON at the rig via
  params_file (CLAUDE.md's locked "Open" line closed) plus a console **his LAND** button (his planner's land_now =
  straight down where QUAD4 is, magnet OFF; his ground-ring exit back to his mat is NOT on in our launch, so over a
  fallen ring it lands on it; Wesley kept it with that label); zero twist: hardware first, then A/B if Tejen agrees;
  `--slot-gate` on at the rig; backup of results/ by Wesley tonight. Inventory: carry and detach freeze on week5 now
  (4b3560e), join after the 21-22 Oct rig day; carry card bars frozen as drafted; 0.55 m baseline, 1 m design row;
  appendix re-flown and regenerated; T11/F14 on co-motion (join tree); N2 1 m bands before the rig day; T12 hover
  under a card, circle exploratory (OPEN line closed); Tier B 120/150/180 + 135/165; Tier A no-code cells now; L5
  twin column R1712/R1713 (against the recommendation: different integral from the rig flight, caption says so);
  push-abort bar = QUAD4's own clearance (new runs only).
- **Lab PC:** usable now, but someone may need it soon: check `who`/load/containers before every batch and yield.
- **Built (uncommitted until the gate):** console his LAND button (+2 tests), harness event HIS_LAND, configs
  w4_drop_his_match_hisland / w4_drop_his_bottom_hisland (exploratory, switch off, his LAND 1 s after the drop).
- **Order of work:** commit; lab batch: the two his-LAND runs + the carry claim (traj3 x2 per cell + traj3_L100 x2,
  card 2026-10-16_carry_speed_envelope, bars frozen); then Tier A cells (card rev first), Tier B (worlds, card), N2 1 m
  bands, N3/appendix re-fly; T11/T12 wait for the join tree.

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

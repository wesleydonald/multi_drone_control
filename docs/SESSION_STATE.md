# Session state (read after every compaction; keep it short and current)

Updated: Fri 16 Oct 2026, 10:58 AEDT (from `date`). Only what a new session needs to act. History: `git log -p docs/SESSION_STATE.md`,
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
- **Commits on day-1016:** 5375333 (decisions, his LAND button, HIS_LAND harness event, his-LAND configs), 83a7088
  (detach freeze cards + configs + 3 Tier B worlds, bars frozen 08:41 while Wesley is at work; N2 1 m configs and
  band draft), e836cf3 (sheet). Registry rows to R1856 (uncommitted).
- **Results:** his LAND after a drop (R1844-R1846): straight down onto the fallen ring; after a drop at the plate QUAD4
  stays armed on top of it: use it only clear of the ring. **Carry claim done** (R1847-R1886): 39/40 pass; spin at
  0.3 m/s fails the tilt bar once (R1875 6.31 deg): outside the envelope; 1 m arm passes; reviewer SUPPORTED WITH
  CORRECTIONS (applied: payload_rmse_m is takeoff+5 s to LAND, the tilt peak is whole-flight; must-not-say list).
- **Detach freeze scored** (R1887-R1919, 2c1985a): T3, T8, T9 (no sign), T15, B1-B3, B5, W4/W5 F1+P1-P4, D1/D4 all
  confirmed; K1 planned unresolved (K1 measured confirmed on the tool's reading, which with detection off is the freed
  drone's lean); B4 holds on the measured pull (G150p reached 30.9 before its drop); A6 (planned, detection off) holds
  the ring at 28 deg and aborts at landing 3/3 (scored as the arm's result). Reviewer running on the four cards.
- **Corner cutting (Wesley ~10:00, card 2026-10-16_corner_cutting.md):** the plan cuts, the trackers follow (dact =
  dref within 0.5 mm); circle: the plan carries a ~1 cm shortfall uncorrected; figure-8 shoulders: the plan adds the
  cut (w_r pulls the rod rate to 0 while the path needs up to 0.27 rad/s at 0.3 m/s). Switches built, off by default:
  plan_w_pos_scale, plan_w_s_scale, plan_w_r_scale, plan_rate_ff (+ existing ref_time_shift). **Running:** N2 batch 16
  (blsovaydh, R1920-R1954, tree 6cd87ed: 25 ok, 10 VOID at RTF 0.24-0.29, re-fly), then batch 17 queued (btz7ga08o: base + 5 arms x fig8/circle/orbit
  at 0.3, 18 runs). Scorer: scratchpad score_cc.py (base R1859/R1863/R1871: ring cut 9.6/29.9/13.0 mm).
- **Next:** score batch 17 -> report to Wesley; winners: neighbours (0.125/0.2, lift, landing, detach, attach), card,
  critic, reviewer, his word. Then the appendix batch (54) and N2 scoring (band rule needs his word).
- **For Wesley tonight:** the 3 Tier B world files (new, via make_carry_worlds.py); the 1 m band rule; T15's layout
  (even ring chosen); the bars set in his absence (he may change them for later runs).

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

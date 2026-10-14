# Session state (read after every compaction; keep it short and current)

Updated: Wed 14 Oct 2026, 22:20 AEDT (from `date`). Only what a new session needs to act. History: `git log -p docs/SESSION_STATE.md`,
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

## Now (Wed 14 Oct, 22:20): executing the plan
- **Plan:** `docs/plan_after_rig_2026-10-14.md` (Wesley approved ~20:30). Its status table at the top is the progress
  record; keep it current. Thursday 15 Oct 15:00-18:00 tests whatever is done by then.
- **Wesley ~22:00, correcting WP6:** the plates stay numbered CLOCKWISE (carriers 3/11/7, QUAD4 9 = Tejen's 3); what
  changes is the angles: every angle typed (launch args, configs) or shown (logs, console, RViz, sheets) becomes
  clockwise degrees = 30 x plate (carriers 90/330/210, QUAD4 270); the maths inside stays counter-clockwise, one
  boundary. Thursday flies it if done and twin-checked, else the 14 Oct conventions. The renumbering (4a60ca3,
  a73d9a4) is reverted in the commit after f9d0b72/657fee1/4beb32f (RViz markers, console, summary kept).
- **Committed tonight:** c15abc4, 19804fb, 4209c62, a0d3963, c1ebdc9, 3297746, dade70d, eacab58, 4a60ca3, a73d9a4,
  f9d0b72, 657fee1, 4beb32f, then the revert. Branch `legacy` at a0d3963.
- **Critic on the attach card (F1-F7, to paste under "## Critic"):** F1 the free hang is lost during a long bottom hold
  (fallback = the typed length that failed t6h_4); F3 a hold exit climbs with the coil ON; F4 after "proven" a ring drop
  cannot turn QUAD4's coil OFF (locked drop rule); F2 proof_replay does not use the run's values; F5 bars; F6 bench
  cases + one coil-OFF hold; F7 notes. Fix F1/F3/F4 before the attach flies; card rev 2.
- **Console probe (live twin):** armed state unknown when started after ARM (trackers announce on change only), rest
  heights learned at hover, the twin's summary ~1 Hz wall, a crash at exit: to fix.
- **Tejen's review** of the 14 Oct logs (Wesley pasted it): agrees t6h_4 was our false negative; new: his trim was off on
  the ring, his MPC stayed FREE_SWING in the proof (M1 never switches ATTACH_PROOF/ATTACHED_HOLD); asks for the 14 Oct
  code, configs, commands, commits. A package for him after the fixes.
- **Twin:** R1718 PASS; R1719 (4beb32f) finished: row to write.
- **Next:** the angle change-over (helper), F1/F3/F4, console fixes, card rev 2 + sheet (clockwise numbering), twin re-fly,
  the package for Tejen, the morning report.

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

# Session state (read after every compaction; keep it short and current)

Updated: Wed 14 Oct 2026, 23:11 AEDT (from `date`). Only what a new session needs to act. History: `git log -p docs/SESSION_STATE.md`,
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

## Now (Wed 14 Oct, 23:11): executing the plan
- **Plan:** `docs/plan_after_rig_2026-10-14.md` (Wesley approved ~20:30). Its status table at the top is the progress
  record; keep it current. Thursday 15 Oct 15:00-18:00 tests whatever is done by then.
- **Committed tonight (latest):** bfbd7a9 (plates clockwise again), f035625 (attach card fixes from critic rounds 1-2:
  free hang kept, the tip follows the plate, coil OFF on the climb back, the drop releases QUAD4, dry_hold, replay
  --catch), 305e50c (typed and shown angles clockwise, same names flipped; 215 configs; R1720 PASS), 33ceec4 (console
  mid-flight fixes, TAKEOFF ack). Records commit next (card rev 3, sheet, R1719/R1720 rows).
- **Tejen's review of the package** (Wesley pasted it ~23:05): five code points to fix before flying: quiet miss needs a
  slow tip and resets on stale input; the proof dwell must not span invalid data (gap > 0.1 s restarts it); the 2.5 s lift
  fallback can start early (wait for his signal, 6 s safety net); the 20 mm below-plate exit should use the face
  clearance along the plate normal; ship the graph wiring that selects comotion. His answers: trim learning at 0.20 m/s
  above contact OK (= aim B); keep the lift proof; no M2 modes, no TAKEOFF change. Puck 37.9/36.2 vs 34.5 mm (pickup).
- **Supervisor ideas** (Wesley's chat): my numbers: tip median 5 cm off; 90 mm discs would put the tip inside the catch
  35 % of the hold time instead of 16 % (inside and slow 15 % vs 8 %); FC accel data mainly useful for the arming issue.
- **Next:** Tejen's five fixes + tests + gate + twin re-fly; card rev 4 response; update the package for Tejen (with the
  graph wiring and replay checks); the morning report + one push notification.
- **Open for Wesley:** QUAD4's aim (A, or B now that Tejen agrees); sending docs/rig/fc_request.md and the package;
  12 failing tests in src/tejen_mission/test are his config/launch contracts (not ours).

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

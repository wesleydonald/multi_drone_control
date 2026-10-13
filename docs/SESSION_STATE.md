# Session state (read after every compaction; keep it short and current)

Updated: Tue 13 Oct 2026, 20:50 AEDT (from `date`). Only what a new session needs to act. History: `git log -p docs/SESSION_STATE.md`,
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

## Wednesday 14 Oct (rig) and Thursday 15:00-18:00 (retries)
- **Sheet:** `docs/rig/wed_2026-10-14.md` (concise: setup, tests 1-12 with commands and what each shows).
  - 1-4 props off: streams, the contact height W, the catch radius K from hand catches, the 10-min load test.
  - 5-7 with Tejen: the no-catch mission, the contact attach twice, the full mission if clean.
  - 8-9 if time: the hover and circle releases against the frozen bands. 10-12 Thursday: the two circles, the magnet hold.
  - Every terminal: `w5` (sources the main tree, exports ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=23).
  - The session multi-drone-control-02 is editing the sheet (13 Oct evening): it renamed the catch radius to K (the
    launch variable was also R) and asks whether QUAD4's taut length L should be read in test 2 (Wesley typed 0.575).
- **Wednesday night:** `python3 tools/rig_night.py --date 2026-10-14 R6:t5 R7:t6 R8:t6b R9:t7 R4:t8 R5:t9`.
- **Changes since the rig last flew:** `docs/rig/changes_since_rig_2026-10-08.md`. The detailed sheet and the red-team
  review: `docs/archive/`.
- **Tejen note** `docs/tejen/tejen_note_2026-10.md`: final; Wesley sends it (the credit wording, the P14 rehearsal, item 6).
- **Twin evidence:** the claim batch on rig-wed (R1638-R1681, card §6.3a) and the 12 Oct batch on week5 (R1682-R1713,
  morning report 13 Oct). week5 adds only the take-back race fix (29b2d7b) and configs.

## 12 Oct night: the lab batch is DONE (R1682-R1713, 32 of 32 scored and committed)
- Morning report: docs/reports/morning_report_2026-10-13.md. Brief for the meeting: docs/reports/supervisor_brief_2026-10-13.md.
- Headlines: Wednesday's rehearsals pass (5 of 6; R1683 is a tumble under his controller, the Tejen note's item 6);
  the claim's failed circle lines pass 4 of 4 on week5; the race fix is clean in all four arms; Tier A pair T3
  resolved with measured (5.4-6.2 deg) < planned (17.0-17.2 deg), reviewer pending; the full M1 circle 1 of 4
  (his moving-plate alignment, marginal); the push clearance bar fails in both directions (it measures the push).
- The lab PC is free. Rows R1682-R1711 say "week5 fc9fb04"; the manifests say 70d525f (docs-only commits between).
- Wednesday's sheet is the concise one, docs/rig/wed_2026-10-14.md (tests 1-12: 1-4 props off, 5-7 with Tejen, 8-9
  the releases if time, 10-12 Thursday). The controller changes since 8 Oct: docs/rig/changes_since_rig_2026-10-08.md.
  The long sheet and the red-team review are in docs/archive/.
- Still open for Wesley (morning report §4): the push clearance bar, the circle reattach, Tier A's announced half,
  sending the Tejen note. The T6 amendments wait for Wednesday's numbers.

## Handoff (if a new session picks up tonight; scripts in `~/handoff_2026-10-12/`)
1. Lab idle? `ssh drones@drones 'who; uptime; docker ps --format "{{.Names}}"'` (no wesley_slot containers, low load).
2. Optional: clear the lab copy's stale folders `~/mdc/src/controller_quad_load` and `~/mdc/src/controller_dissipative`.
3. Start detached, so it survives the session:
   `setsid nohup ~/handoff_2026-10-12/lab_tonight.sh > ~/handoff_2026-10-12/lab_tonight.txt 2>&1 < /dev/null &`
   Watch for `] R1xxx <config>: exit` lines in that log (32 runs, about 4 h).
4. Score each run as it lands, one registry row each (back up, append, diff the old lines):
   - m1c_* and m1x_*: `~/handoff_2026-10-12/sc.sh show|add RID claim 10 "<arm>" "<verdict>" "<notes>"`.
   - m1_full_*: family `full`, rev 10. tc_*: family `tilt`, rev 4 (`t1chk.sh RID` lists the T1 items).
   - w4_da_release_hover, w4_gi_g2_release_orbit, ta_*, w4_3915_release: `python3 tools/release_bands.py metrics RID`
     (the T6 columns, as R1626's row).
   - Card and arm: claim card rev 10 §6.3, tilt card rev 4, freeze P2 (W4/W5 checks), Tier A card rev 2 (T3).
5. The morning report (docs/morning_report_2026-10-13.md) before Wesley's supervisor meeting, in the usual format
   (table, one-line verdict, the decisions).
   - Rig findings first.
   - Also: a thesis lint on thesis-rig (no run ids, branch names or QUAD4 outside LaTeX comments), and a short brief
     for the meeting.

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

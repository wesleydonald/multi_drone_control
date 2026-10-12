# Session state (read after every compaction; keep it short and current)

Updated: Mon 12 Oct 2026, 18:48 AEDT (from `date`). Only what a new session needs to act. History: `git log -p docs/SESSION_STATE.md`,
`docs/decisions.md`, `results/registry.csv`, and `docs/archive/`.

## Where things are (12 Oct tidy)
- **One tree:** `~/multi_drone_control` on **week5**: code, docs, the registry and the working rules. Built, prebuilt,
  gate --quick passes (609 tests). rig-prep is retired (merged). week5 is the testing tree for the rig and the twin.
- **Other worktrees:**
  - `mdc_wt/thesis` (thesis-rig): the assembled thesis, 122 pp. Ch2/Ch3 rewritten; the Ch4 skeleton has F8/F10/F11/F14,
    and T7/T8 are filled after a reviewer.
  - `mdc_lag` (time-cascade) and `mdc_safety` (safety-defaults) are Wesley's.
- **Backups:** every branch and tag is on GitHub under `backup-2026-10-12/*` (verified), and in
  `~/backups/multi_drone_control_2026-10-12.bundle`. Flight data (`results/`, 34 GB), `docs/papers` and his twin logs
  are local only.
- **Archive tags:** archive/m1-tiltcut, archive/m1-e0, archive/m1-full, archive/combined-m1, archive/m1-video.

## Wednesday 14 Oct (rig) and Thursday 15:00-18:00 (retries)
- **Sheet:** `docs/rig_2026-10-14_commands.md`, on week5 with the red-team's fixes in.
  - P1-P14 are props off. R1-R9 are flights: the 3-min hover, the measured and planned 60 deg circles, W4/W5 as R4/R5,
    R11a/R11b as R6-R8, and the full M1 as gated R9.
  - Every terminal: `w5` (sources the main tree, exports ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=23).
- **Wednesday night:** `python3 tools/rig_night.py --date 2026-10-14 R1:r1 ... R9:r9` turns each bag into its thesis
  rows and figure inputs.
- **Tejen note** `docs/tejen_note_2026-10.md`: final; Wesley sends it (the credit wording, the P14 rehearsal).
- **Twin evidence:** the claim batch on rig-wed (R1638-R1681, card §6.3a). The hover lines are supported; the circle
  take-back and the push abort are not. week5 adds only the take-back race fix (29b2d7b) and configs.

## Tonight (12 Oct): lab batch on week5, 32 runs, exploratory
- `$CLAUDE_JOB_DIR/tmp/lab_tonight.sh` from 19:00 if the lab is idle (timer task b1zmjvivl). It runs from whichever
  tree is on week5 (now the main tree). First clean the lab copy's stale folders (src/controller_quad_load,
  src/controller_dissipative).
- **Order:**
  1. Wednesday's rehearsals: R11a, R11b, a LAND in his transit, the full M1 on hover, the W4/W5 same-tree checks.
  2. The push x2 and its other direction x2.
  3. The full M1 circle x4.
  4. The failed circle lines, x2 each.
  5. Tilt T1 x2, N3, C1.
  6. Tier A pair T3 (ta_a1_r1-3, ta_a2_r1-3).
  7. The L5 pair (w4_3915_release x2).
- Score each as it lands (one registry row each), then the morning report for Tuesday's supervisor meeting.

## Open for Wesley
- After Wednesday: the T6 amendments A1-A4 (explained again with Wednesday's numbers; decisions.md 12 Oct).
- After tonight: Tier A's announced half (the measured soft release does not unload the leaver).
- From the offline analyses (decisions.md OPEN 12 Oct): the rig null, the stretch threshold, the rig ramp pair.
- The counter-swing filter: not now (docs/counter_swing_2026-10-12.md).

## Working constraints (beyond CLAUDE.md)
- **Helpers:** one general-purpose helper agent at a time (Wesley).
- **Lab:** at most 4 slots, evenings only, one tree at a time. Check idle first (`who`, load, containers). Each batch
  runs from a script in a fresh `env -i` shell sourcing ONE install; start it detached; kill exact PIDs only.
- **Scorers:** `tools/m1_score.py claim|full|takeback|tilt|tb14 RUN --rev N [--check]` (week5). It prints one LF row:
  back up the registry, append the row yourself, then diff the old lines. Wrapper: `$CLAUDE_JOB_DIR/tmp/sc.sh`.
- **Permissions:** deletions (branches, worktrees), git push and switching the main tree are refused for the
  assistant. Write a script for Wesley.
- **Untracked local files:** `general/`, `.claude/*`. Edit them in place; never `git add` them.
- **Timestamps:** read `date` before writing any clock time.
- **Branch rule:** delete a branch once its work is on week5 or the attempt failed. Archive-tag any unmerged SHA the
  registry cites.

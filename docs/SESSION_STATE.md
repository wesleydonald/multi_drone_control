# Session state (read after every compaction; keep it short and current)

Updated: Mon 12 Oct 2026, 11:50 AEDT (from `date`). Only what a new session needs to act. History: `git log -p docs/SESSION_STATE.md`,
`docs/decisions.md`, `results/registry.csv`, and the "since the 8 Oct rig" page
(https://claude.ai/artifact/QwgP2oPSknLyWZacvDuh5T; source `$CLAUDE_JOB_DIR/tmp/resource/m1_since_rig.html`).

## Where things are
- Main tree `/home/wesley/multi_drone_control`: still on rig-prep (OLD code; don't launch from it). The testing tree is
  `~/mdc_wt/week5`, and the sheet points there. The push and the tree switch are blocked for the assistant: they wait for
  Wesley at the laptop (backup_push.sh, then tidy_finish.sh), or for an allow rule.
- **week4** `a25c40d`: Wednesday's rig tree (worktree `mdc_wt/week4fix`). It stays as it is. Next week's tested work
  goes on **week5**, created from the candidate.
- **Candidate lineage** (each tree has its own worktree under `/home/wesley/mdc_wt/`):

  | branch | head | what |
  |---|---|---|
  | m1-rig | `0210a27` | the rig port, contact_still on, the take-back rises |
  | m1-tiltcut4 | `de91ba2` | tilt cut, every switch off; the claim-tree fallback if TB6 fails |
  | m1-comove | `793183d` | the candidate base: co-moving take-back (TB6 passed, flown from `c53ae0c`), m1-guard, the scorers, m1-harden (critic round 2 changes) |
  | m1-guard | `84a3ad7` | merged into m1-comove (31d9c75): delete once the candidate is on week5 |

- **Side branches:**
  - m1-score `986ac0a`: the scorers, fast-forwarded into m1-comove (delete once the candidate is on week5).
  - m1-rec and m1-video: the Gazebo recorder, to merge into the candidate.
  - thesis-rig `a493c7d` (worktree `mdc_wt/thesis`): the thesis draft, with lane 3's Ch2/Ch3 rewrite (fast-forwarded
    12 Oct 07:02; builds, 108 pp, no undefined refs; hand-back `docs/thesis_work_2026-10-12b.md`).
    - Literature corrected against Quan's full text and Zeng; the spine structure; Appendix F; the bar rationale;
      the leaver rule in Section 3.4.
    - Prose from Fri 16.
  - Thesis lane (11 Oct, done; hand-back `docs/thesis_work_2026-10-11.md` on thesis-rig): thesis-work is
    fast-forwarded into thesis-rig; desk-ext is merged into rig-prep (`tools/admissibility_map.py`,
    `release_transient.py`, `mission_value.py`, outputs in `results/analysis/`).
    - Their worktrees and branches were removed 11 Oct (Wesley).
    - Open for that lane: switch `release_transient.py` to the signed T6 tilt-vector form (freeze doc §1.1); the
      14 Oct spring-scale number goes into the bar paragraph.
- **Archive tags** for superseded attempts that registry rows cite: archive/m1-tiltcut, archive/m1-e0, archive/m1-full,
  archive/combined-m1.
- **Not ours to judge** (Wesley): time-cascade, safety-defaults, add-drone, backup/*, cable-payload-controller,
  fixing-rviz, and week1-ready-for-testing (`git branch -D` is his).
- **Plans:**
  - `docs/plan_m1_remaining_2026-10-11.md`: execution, D1-D21.
  - `docs/thesis_plan_2026-10-11.md`: spine RQ1 carry / RQ2 leave / RQ3 join; due **20 Nov**.
- **Card:** `docs/experiments/2026-10-10_combined_m1_contact.md` rev 10; final SHA rig-wed ba40bd0 (filled 11 Oct). Reviewer
  verdict in §6.3a.
- **Reference:** `docs/reference.md` (architecture, negatives, traps, how to run). CURRENT_STATE.md is retired.

## Running
- Lab PC: idle. The smoke on the candidate 793183d passed, 4 of 4 support (R1628-R1631; the 10th clean full M1 is R1631).
- OVERNIGHT LOOP (11 Oct 23:03, Wesley): the claim batch on rig-wed ba40bd0 (card rev 10 §6.3 + tilt rev 4), 41 runs,
  `$CLAUDE_JOB_DIR/tmp/lab_claimwed.txt`. Order: the 17 m1c arms, the 3 full M1, the 2 R11a, then the tilt claim (T0, T1,
  F6b, F6c, C1, C2, F1, C3, each x2) and N1-N3. Score each as it lands: tools/m1_score.py from rig-wed (claim/full/tilt,
  rev 10 / tilt rev 4), one row each. When the combined table is complete: the reviewer, its verdict into the card.
  Morning report by 08:00.
  - Wave 1 (R1638-R1641) is scored. His contact-mode MATCH_VELOCITY gate (xy < 2.5 cm, < 0.08 m/s on the circling
    plate) stalled in 3 of 4 circle arms: his tracking sat 3-5 cm off. R1639 falsifies TB4's ring tilt in its LAND
    take-back. After the batch: re-fly m1c_nocatch_orbit x2 on 2 slots as a diagnostic (lab load?). Scoring wrapper:
    `$CLAUDE_JOB_DIR/tmp/sc.sh show|add RID family rev arm verdict notes`.
  - The 17 claim arms are scored (R1638-R1654), plus R1655 beside the table. Reviewer: **NOT SUPPORTED overall**.
    - Supported: hover attach, hover take-back, no-catch hover, LAND during the proof.
    - Not supported: the circle take-back and the push abort (R1654 rod 0.094 m).
    - Corr rows for R1638/1639/1641/1654 and card §6.3a are committed in 898f775.
  - DONE 02:44: R1638-R1681 scored. Morning report docs/morning_report_2026-10-12.md.
  - Wesley ~10:10-11:00 (decisions.md):
    - the hover lines are claimed; the failed lines go to a week5 table;
    - the race is fixed; the 17 citations are approved; the magnet hold stays 9-10 N;
    - week5 is the testing tree; the rig plan is front-loaded (ambitious Wed, Thu 15 for retries, the full M1 as
      a gated last flight);
    - the lab PC evenings only.
- **Repo tidy (12 Oct, Wesley):** one tree for everything.
  - rig-prep is merged into **week5** (4983d2d) and retired. .gitignore no longer hides docs/, configs/ and tools/.
  - Never-committed work is saved: rig-prep 885ffbc, m1-rig df38269, now in week5.
  - Removed: 25 unreferenced configs and tools/legacy. 11 finished docs moved to docs/archive/.
  - The stale WRENCH test is updated. Gate: 596 tests pass.
  - Backup: ~/backups/multi_drone_control_2026-10-12.bundle (verified). GitHub (public; Tejen OK): Wesley runs
    `$CLAUDE_JOB_DIR/tmp/backup_push.sh` (every branch to backup-2026-10-12/*, tags; the push is his).
  - Then Wesley runs `$CLAUDE_JOB_DIR/tmp/tidy_finish.sh`. It checks the backup, removes the finished worktrees,
    moves ~/multi_drone_control onto week5, and deletes the merged branches (tag archive/m1-video for a cited SHA).
  - After it: rebuild the main tree (colcon, prebuild_planner, gate --quick) before anything launches from it.
  - week5 holds the race fix (29b2d7b), the abort_negx config and feedforward_60_hold.yaml. R9's rig line is
    launch-checked clean.
- **Wednesday sheet** docs/rig_2026-10-14_commands.md (9c9dcc2): P1-P13 props off, R1-R9 flights, all on week5;
  `w5` exports ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=23.
- **Tejen note** docs/tejen_note_2026-10.md (d208958): final, for Wesley to send (credit wording in).
- **Ch4 skeleton** on branch thesis-ch4 (71bed15, worktree mdc_wt/thesis-ch4): skel_*.tex per section, the
  placeholder tables T7/T8/T10-T13, pending figures under the final labels fig:F8-F16. It also adds the two
  never-committed images. Merge into thesis-rig after the other chat's figures replace the pendingfigs.
- **The other chat:** Wesley may give it lane 5 (the deck for Tue 13, a red-team of Wednesday on branch
  wed-review, the thesis figures on thesis-figs, the citations). The prompt is in this session's chat.
- **P6 offline analyses:** a helper agent on branch p6-offline (worktree mdc_wt/p6-offline); it returns a summary
  doc docs/p6_offline_2026-10-12.md.
- **TONIGHT** (`$CLAUDE_JOB_DIR/tmp/lab_tonight.sh`, 24 runs on week5, from 19:00 if the lab is idle; the timer is
  b1zmjvivl):
  - Wednesday's rehearsals first (R11a, R11b, a LAND in his transit, the full M1 on hover, the W4/W5 same-tree
    checks);
  - then the failed lines, the full M1 circle x4, the push and the push the other way, T1 x2, N3 and C1.
  - All exploratory; score each as it lands.
- Laptop: idle. No helper is running. The full M1 on the circle is recorded and cut (R1633, the 11th clean full M1):
  results/videos/full_M1_twin_R1633_orbit_edit.mp4 (2 min 56 s, captioned; raw: _wide, _circle, _plate9). The recorder
  is on the candidate (adaf2c5).
- Critic round 2 done: GO WITH CHANGES for hover R11a/R11b (rig card ## Critic; answers in decisions.md).
  Critic round 2 changes are on the candidate: m1-harden merged as 793183d (205 planner tests, gate pass). Guard
  constants: wait 3 s, +0.05 above slack, guard-stage speed 0.06 m/s, follow-down slack 0.12. Next: the QUAD4
  record loader. No helper is running.
- The other chat (thesis lane):
  - lanes 1 and 2 are merged: thesis-rig 5dc4eb1, rig-prep de41c93; their branches and worktrees are removed;
  - lane 2 built the supervisor deck https://claude.ai/artifact/TwjqA7ebTDrkgDK6nwutiR, the T6 metrics text and the
    three 8 Oct figures on the tilt rise;
  - lane 3 (Ch2/Ch3) is fast-forwarded into thesis-rig a493c7d. Wesley runs
    `$CLAUDE_JOB_DIR/tmp/cleanup_thesis_lane3.sh` (retire modelling/control.tex on thesis-rig, remove the
    thesis-work3 worktree and branch). Its open questions: the 17 citations to approve, and the magnet hold of
    9-10 N.

## TB6 (co-moving take-back, m1-comove c53ae0c; card rev 10 §6.2): COMPLETE, passes its gate criteria
- **All 11 support:** R1615-R1625. R1624 is read on the commanded path (Wesley ~16:20).
- **Circle take-backs:** rod 0.205-0.253 m, body 0.415-0.493 m. TB5 predicts them within about 1 cm.
- **Between the bars, for the Thu 15 gate:**
  - R1618: attitude step at the decouple 2.15 deg.
  - R1621: transit take-back step 3.83 deg (the fixed hold gave 4.4-4.7).
- **Rig finding (R1625):** with his takeoff after the orbit start, the ring passed over QUAD4 (0.23 m xy, 0.12 m
  vertical). This confirms the rule "his TAKEOFF before the orbit" on the rig.
- **Still needed:** the co-move goes into the candidate. A circle on 21-22 Oct also needs the reviewer on the final
  table.

## Open questions for Wesley
- The between-bar values (R1618, R1621) at the Thu gate.
- The plan's D10-D17 and D21, and the thesis plan's D8-D16, all still open (see the plans).

## Next, in order (Wesley 11 Oct: the attach flights move to Wednesday afternoon, reviewer waived)
1. **Pinned Wednesday install:** branch rig-wed (worktree mdc_wt/rig-wed, ba40bd0: 0628d85 + the launch fixes and
   load_check.py). Twin rehearsal done:
   - R11a (R1634, R1636): supports twice.
   - R11b (R1635, R1637): the attach supports twice, with the proof under 1 s; each LAND was a TB14 tip-over, the twin
     artefact.
2. **Tuesday's desk rehearsal is skipped** (Wesley).
   - The desk load test is done: it passes at normal priority and fails when the laptop is loaded.
   - The launch fixes are in: the fleet manager announces at startup; the worktree root lookup works.
   - Wednesday step 3 follows rig-wed `docs/rig_2026-10-14_load_test.md`.
3. **Wednesday 14:**
   - the 09:00 measurements and Part A (W4/W5);
   - Part B props-off on rig-wed;
   - Part C, R11a then R11b on hover, only under its go conditions (sheet).
   - QUAD4 uses QUAD2's values: kT 22, drop 0.575 (L cross-checked).
4. **Tejen:** Wesley sends `docs/tejen_note_2026-10.md` (his presence and OK Wednesday, his test logs). With his logs,
   read his height error at contact and his kT. That decides R11a's +0.03 or +0.05.
5. **After Wednesday:**
   - the claim batch on the rig-wed SHA, then the reviewer;
   - 21-22 Oct: repeats, and the circle only under its gates;
   - the QUAD4 record loader is optional, since the values are typed;
   - week5 from the reviewed tree.

## Working constraints (beyond CLAUDE.md)
- **Helpers:** one general-purpose helper agent at a time (Wesley).
- **Lab:** at most 4 slots, one tree at a time; check idle first (`who`, load, containers).
  - Each batch runs from a script in a fresh `env -i` shell that sources ONE install (copy
    `$CLAUDE_JOB_DIR/tmp/lab_tb6.sh`). Two installs in one shell break the launch argument check.
  - Start it detached with `setsid nohup`. Kill exact PIDs only: `pkill -f` matches its own shell.
- **Scorers:** `tools/m1_score.py claim|full|takeback|tilt RUN --rev N [--check]` and `tools/m1_score.py tb14
  R1479-R1578` (in m1-comove / m1-score). Source that tree's install. It prints one LF row; append it yourself.
  The old ephemeral scorers in `$CLAUDE_JOB_DIR/tmp/score/` are retired.
- **Registry:** back it up, append, then diff the old lines.
- **Untracked files:** `general/CLAUDE.md`, `.claude/agents/*` and `.claude/workflows/*` are local and untracked. Edit
  them in place and never `git add` them.
- **Deletions** (branches, worktrees) are refused by the permission check. Write a script for Wesley to run.
- **Timestamps:** read `date` before writing any clock time. Times estimated from memory ran up to 1.5 h ahead on
  11 Oct; commit times are exact.
- **Papers:** `docs/papers/` is git-ignored and local. The Quan et al. copy is the full 33-page arXiv v1 (16:28).
- **Branch rule:** delete a branch once its work is on week5 or the attempt failed. Archive-tag any unmerged SHA the
  registry cites.

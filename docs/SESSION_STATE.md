# Session state (read after every compaction; keep it short and current)

Updated: Sun 11 Oct 2026, 16:30 AEDT (from `date`). Only what a new session needs to act. History: `git log -p docs/SESSION_STATE.md`,
`docs/decisions.md`, `results/registry.csv`, and the "since the 8 Oct rig" page
(https://claude.ai/artifact/QwgP2oPSknLyWZacvDuh5T; source `$CLAUDE_JOB_DIR/tmp/resource/m1_since_rig.html`).

## Where things are
- Main tree `/home/wesley/multi_drone_control` on **rig-prep** (docs, registry, cards, plans). Nothing is pushed.
- **week4** `a25c40d`: Wednesday's rig tree (worktree `mdc_wt/week4fix`). It stays as it is. Next week's tested work
  goes on **week5**, created from the candidate.
- **Candidate lineage** (each tree has its own worktree under `/home/wesley/mdc_wt/`):

  | branch | head | what |
  |---|---|---|
  | m1-rig | `0210a27` | the rig port, contact_still on, the take-back rises |
  | m1-tiltcut4 | `de91ba2` | tilt cut, every switch off; the claim-tree fallback if TB6 fails |
  | m1-comove | `986ac0a` | the candidate base: co-moving take-back (TB6 passed, flown from `c53ae0c`), m1-guard merged, the committed scorers (tools/m1_score.py) |
  | m1-guard | `84a3ad7` | merged into m1-comove (31d9c75): delete once the candidate is on week5 |

- **Side branches:**
  - m1-score `986ac0a`: the scorers, fast-forwarded into m1-comove (delete once the candidate is on week5).
  - m1-rec and m1-video: the Gazebo recorder, to merge into the candidate.
  - thesis-rig (worktree `mdc_wt/thesis`): the thesis draft.
- **Archive tags** for superseded attempts that registry rows cite: archive/m1-tiltcut, archive/m1-e0, archive/m1-full,
  archive/combined-m1.
- **Not ours to judge** (Wesley): time-cascade, safety-defaults, add-drone, backup/*, cable-payload-controller,
  fixing-rviz, and week1-ready-for-testing (`git branch -D` is his).
- **Plans:**
  - `docs/plan_m1_remaining_2026-10-11.md`: execution, D1-D21.
  - `docs/thesis_plan_2026-10-11.md`: spine RQ1 carry / RQ2 leave / RQ3 join; due **20 Nov**.
- **Card:** `docs/experiments/2026-10-10_combined_m1_contact.md` rev 10. The §3b "as built" block is at the top of §3b;
  the final SHA is still blank.
- **Reference:** `docs/reference.md` (architecture, negatives, traps, how to run). CURRENT_STATE.md is retired.

## Running
- Lab PC: idle (TB6 finished).
- Laptop: idle. No helper agent is running.
- The other chat (thesis lane, Wesley's prompt) may be working in its own worktrees `mdc_wt/thesis-work` and
  `mdc_wt/desk-ext`. It must stay off the lab, the laptop simulations, the registry and this file. Its hand-back note
  is `docs/thesis_work_2026-10-11.md` on its branch. Copy its "decisions for the record" into decisions.md.

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

## Next, in order
1. W, the Wednesday paperwork (Mon 12, for Wesley's review):
   - the concise measurement items in `docs/rig_2026-10-14_commands.md`;
   - the magnet spring-scale hold at 0/30/45/60 deg, video, W4/W5 held 15-20 s;
   - freeze the release metrics, the ledger band rule and the W4/W5 bands (one lab run each, week4);
   - F3 his takeoff order and the F8 wand walk;
   - the DISARM-after-LANDED line and tilt checks (a)-(f);
   - the health check's rig items: pack time against the ~2.5 min M1, tip occlusion near plate 9, the body-rate fix
     unflown, the 0.25 s pose timeout;
   - the Tejen note: kT gate, coil gate, supervisor stop, QUAD2 values, tilt cut, his transit freezing after a mocap
     glitch, his object magnet left ON in a take-back.
2. Tue 13 desk rehearsal and load test. Wed 14 rig. Thu 15 gate and R10a. Fri 16: smoke on the candidate, name the SHA,
   joint claim batch. Then the reviewers, then week5. Tue 20 rehearsal. 21-22 Oct: hover only unless TB6 passed and
   the reviewer has signed off.

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

# Session state (read after every compaction; keep it short and current)

Updated: Sun 11 Oct 2026, ~17:50. Only what a new session needs to act. History: `git log -p docs/SESSION_STATE.md`,
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
  | m1-comove | `0c5fc69` | the co-moving take-back; TB6 flown from `c53ae0c`; the replay clock fix |
  | m1-guard | `84a3ad7` | rig release-guard failure: follow the ring down, LAND at the slack (max 2 s); 185 tests, gate pass; **merges into m1-comove with Wesley's OK** |

- **Side branches:**
  - m1-score (= `0210a27`, nothing written): the committed scorers.
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
- Lab PC: the last TB6 run, R1623 m1x_r11a_circle (`$CLAUDE_JOB_DIR/tmp/lab_tb6.txt`; background waiter b10f8qby0).
  After it the lab is idle.
- Laptop: idle. No helper agent is running.
- The other chat (thesis lane, Wesley's prompt) may be working in its own worktrees `mdc_wt/thesis-work` and
  `mdc_wt/desk-ext`. It must stay off the lab, the laptop simulations, the registry and this file. Its hand-back note
  is `docs/thesis_work_2026-10-11.md` on its branch. Copy its "decisions for the record" into decisions.md.

## TB6 (co-moving take-back, m1-comove c53ae0c; card rev 10 §6.2): 10 of 11 scored
- **Support:**
  - R1615 nocatch_orbit
  - R1616 lead030 (no contact)
  - R1617 ph11
  - R1619 land_rejoin_orbit
  - R1620 full no-catch orbit
  - R1622 nocatch_hover (c6f2032 path)
  - R1625 late takeoff
- **Between the bars:**
  - R1618: attitude step at the decouple 2.15 deg.
  - R1621: transit take-back step 3.83 deg (the fixed hold gave 4.4-4.7).
- **Reading needed (Wesley):** R1624 LAND in his pickup descent.
  - The commanded path went straight down.
  - The measured body was levered 0.33 m sideways after his object magnet caught the ball.
- **Rig finding (R1625):** with his takeoff after the orbit start, the ring passed over QUAD4: 0.23 m in xy with a
  0.12 m vertical gap. This confirms the rule "his TAKEOFF before the orbit" on the rig.
- **TB5 replay:** it predicts the logged clearances within 1 cm (3 cm body on the LAND case).
- **Thu 15 gate:** TB6 passes if no arm meets its falsifier and every circle take-back confirms both clearance bars.

## Open questions for Wesley
- The R1624 reading.
- The m1-guard merge.
- Whether a take-back before his pickup should turn his object magnet OFF (R1624).
- The between-bar values (R1618, R1621) at the Thu gate.
- The plan's D10-D17 and D21, and the thesis plan's D8-D16, all still open (see the plans).

## Next, in order
1. Score R1623 (falsifier: a contact or attach declared, or a TB4 falsifier). Then the TB6 summary for the gate. One
   run on any arm that fails.
2. S, the scorers into `tools/` on m1-score: REWELD keying, rod-segment clearance, overflight, tilt readouts, the
   tip_metrics torque fix. They must reproduce R1573, R1579 and R1581. Resume helper abbe240cb54198744 or start fresh.
3. W, the Wednesday paperwork (Mon 12, for Wesley's review):
   - the concise measurement items in `docs/rig_2026-10-14_commands.md`;
   - the magnet spring-scale hold at 0/30/45/60 deg, video, W4/W5 held 15-20 s;
   - freeze the release metrics, the ledger band rule and the W4/W5 bands (one lab run each, week4);
   - F3 his takeoff order and the F8 wand walk;
   - the DISARM-after-LANDED line and tilt checks (a)-(f);
   - the health check's rig items: pack time against the ~2.5 min M1, tip occlusion near plate 9, the body-rate fix
     unflown, the 0.25 s pose timeout;
   - the Tejen note: kT gate, coil gate, supervisor stop, QUAD2 values, tilt cut, his transit freezing after a mocap
     glitch, his object magnet left ON in a take-back.
4. Tue 13 desk rehearsal and load test. Wed 14 rig. Thu 15 gate and R10a. Fri 16: smoke on the candidate, name the SHA,
   joint claim batch. Then the reviewers, then week5. Tue 20 rehearsal. 21-22 Oct: hover only unless TB6 passed and
   the reviewer has signed off.

## Working constraints (beyond CLAUDE.md)
- **Helpers:** one general-purpose helper agent at a time (Wesley).
- **Lab:** at most 4 slots, one tree at a time; check idle first (`who`, load, containers).
  - Each batch runs from a script in a fresh `env -i` shell that sources ONE install (copy
    `$CLAUDE_JOB_DIR/tmp/lab_tb6.sh`). Two installs in one shell break the launch argument check.
  - Start it detached with `setsid nohup`. Kill exact PIDs only: `pkill -f` matches its own shell.
- **Scorers** (`$CLAUDE_JOB_DIR/tmp/score/`): `tb6_score.py` + `tb6_rows.py` (source the m1-comove install for TB5);
  `claim_row.py` (env TREE, REV; it prints date 10 Oct, so fix the date); `tilt_row.py`; `contact_score.py` keys on
  WELD (the full M1 logs REWELD).
- **Registry:** back it up, append, then diff the old lines.
- **Untracked files:** `general/CLAUDE.md`, `.claude/agents/*` and `.claude/workflows/*` are local and untracked. Edit
  them in place and never `git add` them.
- **Deletions** (branches, worktrees) are refused by the permission check. Write a script for Wesley to run.
- **Branch rule:** delete a branch once its work is on week5 or the attempt failed. Archive-tag any unmerged SHA the
  registry cites.

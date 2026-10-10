# Session state (read after every compaction; keep current at each milestone)

Updated: Sat 10 Oct 2026, afternoon.

## Where things are
- Main tree `/home/wesley/multi_drone_control` on **rig-prep** (see git log), clean. Nothing running.
- rig-prep = week4 (6dd48d4) + creep fix, defaults, twin attach worlds, body-to-tip, and the three newcomer switches
  (`newcomer_lifecycle`, `newcomer_fault_land`, `departed_ground_idle`, all default off; press limit 6 s).
- week4: 33716be (forward fix) still unpushed by Wesley. Nothing pushed tonight.
- Worktrees: `/home/wesley/mdc_wt/newcomer` (branch newcomer-fault-scope = rig-prep), `mdc_wt/twin-attach`,
  `mdc_wt/combined-m1` (8c21fe8, old base: O1 mux start_ours + aux zeroing), `mdc_wt/thesis` (thesis-rig 63a54fe),
  `mdc_wt/week4fix`, `mdc_wt/rig-prep-b`; old `mdc_drop`, `mdc_lag`, `mdc_safety` (remove only with Wesley's OK).
- CLAUDE.md is tracked on rig-prep since b8509fb (Wesley, 9 Oct: it may go to main); commit edits on the current
  branch. The orphan `working-rules` branch was deleted.
- Last registry row: R1302. Morning report given 9 Oct ~07:30 (twin runs R1235-R1302).

## Decided this session (all in docs/decisions.md)
- Lab days Wed 14 Oct (8 h) and Thu 15 Oct 15:00-18:00, then every Wed/Thu to the 7 Nov results freeze.
- QUAD4 = /drone_3 ("drone 4"), body 14, tip 24; M1 layout carriers on plates 3/11/7, weld on plate 9.
- Newcomer abort: OFF or LAND before the weld -> magnet OFF, radially out to 1.2 m at its height, lands by itself.
- A fault on an unwelded QUAD4 lands the fleet ("land everything"); DISARM disarms everything.
- Body-to-tip is the rig default (twin confirmed the approach).
- (superseded 9 Oct) Monday freeze -> no freeze, merge per item.
- Wesley relaunches T2 between rig flights anyway (the creep sheet rule stays).

## Wesley 9 Oct morning (answered; in docs/decisions.md)
- No day deadlines: the plan is an order of work. LAND must land every drone safely whatever it is doing
  (newcomer_lifecycle -> default after a LAND-in-every-phase twin matrix + reviewer).
- Freed/departed drone: pushed down to its rest, then disarmed ALONE (expected disarm); no armed idle. New code.
- Approach corrects on the measured mocap magnet tip (body 24) vs the target. New code (card + critic).
- C1b (QUAD4 fault over the plate): accepted for R8b as is.
- No freeze: merge each item into week4 when it passes (his OK per merge); rehearsal on the current tree, read-back
  line re-run for anything merged after it.

## In progress (9 Oct late evening)
- rig-prep HEAD 53e1a66. Nothing flying. Last registry row R1451 (laptop) / R1446 (lab); gate smoke R1452/R1453.
- DONE tonight: LAND re-fly R1413-R1446 (one tree 8588745 + keep-alive f330bd2): 16/17 phases 2 of 2; R1438 tipped
  QUAD4 0.5 s after a weld in the unwind (3 of 11 within 2 s of a weld, twin rigid link) -> reviewer SUPPORTED with
  corrections (in the card). Disarm-alone laptop re-fly R1411/R1412/R1447-R1451: P1-P4 hold, no stall (17/17).
- DONE: the three newcomer switches ON by default (bfbd181; legacy off; CLAUDE.md lock line; decisions.md DONE line).
  Gate 1-5 pass; SIL smoke fails its two peak bars identically with/without (pre-existing since R0840; the sil_smoke
  re-baseline needs Wesley's word).
- Known, harmless: R1418 race (alone-disarm request crossing the LANDED disarm keeps that tracker up until teardown);
  tracker exits at the fleet disarm stall the graph ~1 s after LANDED.
- Thesis (branch thesis-rig, /home/wesley/mdc_wt/thesis/docs/thesis_drafts/thesis_v2, HEAD c0fc6e8): restructure
  (ebf1b3b), rig figures + 7-8 Oct tables + lit comparison (fa94166), cut to 50 pp main text (e5260a8: secondary
  tables to appendices C/D), trim contradictions settled (c0fc6e8). Builds clean, 82 pp total; ~2 lines spare on
  the conclusion page. Open CHECKs: new refs (plug-and-play MPC, static equilibrium, aerial docking), the "This
  work" row's qualitative cells, the detach metric definition, the parameter tables, "twelve numbers per node",
  the 1-of-21 vs 2-of-21 landing count; conclusion value statement not written.
- Rig finding to report: on 7 Oct the integral-only hover bobbed +-7 cm at 0.2 Hz; with the measured pull it held
  within 1 cm (one 7 s hover; the 36/68 s circle and figure-eight agree: z sd 4.8/4.7 -> 0.5/0.6 cm).
- Tip feedback rev 5: SUPPORTED (earlier today). Rig: Wednesday sheet docs/rig_2026-10-14_commands.md.

## Sat 10 Oct (Wesley's answers this morning; all in docs/decisions.md)
- Merge rig-prep -> week4 AFTER one check run each on rig-prep's tip (c2279cf): lab batch R1454-R1460 running
  (twin_tf_weld_on, ck_r8a_land_loop, ck_r8a_off_loop, w4_gi_g2_release_orbit, twin_r8a0, w4_da_release_hover,
  w4_defaults_acceptance). Then registry rows, reviewer on the R8a0/R8a/R8b twin table, merge (Wesley pushes).
- Tejen port started, sim only: worktree /home/wesley/mdc_wt/tejen-sync, branch tejen-sync (agent). His repo has
  nothing new since 11a0683f. Wesley says Tejen has "another thing for us to do": not in his repo; ask Wesley.
- SIL smoke: agent finding the cause at R0840; re-baseline only if the change was intended (no threshold edits yet).
- Thesis (agents): proposed RQs applied, paragraph spacing restored, value statement, CHECKs (text agent, commits on
  thesis-rig); Undermind references -> docs/thesis_drafts/refs_2026-10-10.md (I integrate); new figures #2 r305
  storyboard, #4 parity, #12 swimlanes (figure agent; no commits; I integrate). Page limit not enforced (Wesley).
- DONE 10 Oct: check runs R1454-R1460 all within bars; reviewer SUPPORTED with corrections (registry + sheet fixed);
  rig-prep MERGED into week4 (b4e11f1, then fast-forwarded with docs; rig-prep == week4 == 26d4227; Wesley pushes).
  SIL smoke re-baselined (0.60 / 12.0; cause = 0.86 kg ring + 4 Oct LAND defaults; R1461-R1467). Tejen port on
  tejen-sync (4 commits, builds, tests as his tree; M1 hand-over 0.12 m lower). Thesis: RQs, spacing, value statement,
  CHECKs (c857460, e3831d0, a8c629c), references (30c7060; 12 papers to approve); main text ends p57, 88 pp.
- R9 twin: held R1468 PASS (weld, 3.0 deg, dip 3 mm, circle resumes on four); moving R1469 FAILS (no tip hold on a
  moving plate -> tip loop never runs -> tip 3 cm high, outside the 2.5 cm weld radius; QUAD4 0.311 m from a carrier
  in transit). Thursday: R9 held only. Repeats + M1 partner baseline (rig-prep) running on the lab PC.
- R9 repeats: held 2 of 2 supports (R1468/R1471), moving 2 of 2 falsifies (R1469/R1472): Thursday R9 held only.
- M1 partner baseline R1470 (rig-prep, legacy) FAILS at the hand-back: QUAD4 gets no reference (approach_hold off for
  partners + _attach_approach set by the hand-off; since bbce103). Fix on branch partner-handoff-fix (worktree
  /home/wesley/mdc_wt/partner-fix, 0b50b40, unit test); check run R1473 from it (lab). Port check branch
  tejen-sync-check (worktree /home/wesley/mdc_wt/tejen-check) = tejen-sync + the fix, for the port's M1 run.
  Worktree batches need MDC_REPO_ROOT=<worktree> MDC_RESULTS_ROOT=/home/wesley/multi_drone_control/results.
- rig_to_run fixed for w4/w5 (a09244b). Figures F_rig_circle_detach, F_twin_rig_parity, F_disarm_lanes in the thesis
  (596f90a). Rig finding: carriers 9-16 cm above their start at LANDED on 7-8 Oct (stall test; thr ~0.24): ask Wesley.
- Thursday: sheet draft docs/rig_2026-10-15_commands.md; card docs/experiments/2026-10-10_twin_r9.md with configs
  twin_r9_held / twin_r9_moving (not flown yet). Tilt not built (waits for a later lab day).

## Open items / next steps (plan: /home/wesley/.claude/plans/whimsical-kindling-minsky.md)
- Reviewer pass on the final newcomer tables Monday before the freeze (graded WEAK on 9 Oct; re-flies done since).
- Unexplained: R1278 carrier flip in the creep (1 in 53). (Planner late ARM: explained, runner fixed e1c6e77.)
- C workstream (combined M1 stack with Tejen's code), P (Tejen port), X (tilt card + critic), R (Wed/Thu sheets,
  Tuesday desk rehearsal, send Tejen docs/tejen_questions_2026-10.md, QUAD1 checklist docs/rig_quad1_checklist.md).
- Re-freeze the launch gate baseline at week4 (backlog).

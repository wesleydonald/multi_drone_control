# Session state (read after every compaction; keep current at each milestone)

Updated: Fri 9 Oct 2026, late evening.

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
- Thesis (branch thesis-rig, /home/wesley/mdc_wt/thesis/docs/thesis_drafts/thesis_v2): restructure committed
  (ebf1b3b); rig figures in (fa94166): thrust_map, land_phases, solve_time, rig_detach, rig_integral; 7 Oct measured
  pull + 8 Oct detach tables; lit comparison in the discussion. Main text 62 pp vs the 50 limit: a cut agent is
  running (moves to appendices first; not committing). New figure scripts in tools/thesis_rig_figures.py
  (rig_integral, land redone after blind critics; 77c502d).
- Rig finding to report: on 7 Oct the integral-only hover bobbed +-7 cm at 0.2 Hz; with the measured pull it held
  within 1 cm (one 7 s hover; the 36/68 s circle and figure-eight agree: z sd 4.8/4.7 -> 0.5/0.6 cm).
- Tip feedback rev 5: SUPPORTED (earlier today). Rig: Wednesday sheet docs/rig_2026-10-14_commands.md.

## Open items / next steps (plan: /home/wesley/.claude/plans/whimsical-kindling-minsky.md)
- Reviewer pass on the final newcomer tables Monday before the freeze (graded WEAK on 9 Oct; re-flies done since).
- Unexplained: R1278 carrier flip in the creep (1 in 53). (Planner late ARM: explained, runner fixed e1c6e77.)
- C workstream (combined M1 stack with Tejen's code), P (Tejen port), X (tilt card + critic), R (Wed/Thu sheets,
  Tuesday desk rehearsal, send Tejen docs/tejen_questions_2026-10.md, QUAD1 checklist docs/rig_quad1_checklist.md).
- Re-freeze the launch gate baseline at week4 (backlog).

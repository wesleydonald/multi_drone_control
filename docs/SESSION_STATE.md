# Session state (read after every compaction; keep current at each milestone)

Updated: Sat 10 Oct 2026, 21:40 (overnight plan running).

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
- R1473 (fix) PASS: detach 2.71 / reweld 2.52 deg, 98.5 s (R0752 2.80 / 2.74, 106.3). R1474 (port + fix): reweld 5.62 deg
  (G1 bar 5), weld 0.29 s after a hand-off 0.12 m lower (the marker fix): port not mergeable as is; 1 run.
- Evening: partner-handoff-fix MERGED into week4 (6049543; rig-prep == week4); papers approved (thesis 1659d7d);
  Wesley to push week4 -> origin main. Q3 (port's lower hand-off) and Q4 (moving R9) re-asked with context.
- Wesley 10 Oct evening (decisions.md, CLAUDE.md lock line): Tejen's controller does EVERY QUAD4 attach on the rig,
  down to contact (the magnet only catches on the plate); OUR motion-capture check alone confirms it; then we take
  QUAD4 over and fold in. R8a/R8b/R9 dropped from the rig sheets (Wed: W4, W5, R8a0 + a props-off attach check at
  09:00 with weld_radius 0.01; Thu: R10a + carry follow-ups). Next work: the combined M1 stack on these roles (his
  descent to contact, our check, mux + fold-in on object_attached, also on a moving ring), then twin.
- rig_to_run fixed for w4/w5 (a09244b). Figures F_rig_circle_detach, F_twin_rig_parity, F_disarm_lanes in the thesis
  (596f90a). Rig finding: carriers 9-16 cm above their start at LANDED on 7-8 Oct (stall test; thr ~0.24): ask Wesley.
- Thursday: sheet draft docs/rig_2026-10-15_commands.md; card docs/experiments/2026-10-10_twin_r9.md with configs
  twin_r9_held / twin_r9_moving (not flown yet). Tilt not built (waits for a later lab day).

## Sat 10 Oct, late
- Wesley: not pushing week4 to main for now (week4 = rig-prep = 44da020, 128 ahead of origin/main).
- Desk dry run of Wednesday's 09:00 props-off attach check against fake_mocap (new `--tip-plate/--tip-dz`):
  radius 0.01 / contact height / plate offset read back; 2 cm above: no ATTACH; on the plate: ATTACH 0.17 s; stays
  attached. Planner warning flood on the ground throttled (2 s). Sheet: fallback to /magnet/attachment_state for d,
  the rehearsal commands. Mocap noise of still bodies (8 Oct r305): 0.01-0.02 m/s max, not a risk for the 0.05 gate.
  Desk script lesson: a background `ros2 launch` in a script ignores SIGINT; use `set -m` and signal the group.
- Wesley: the 9-16 cm drop at LANDED is fine (decisions.md); week4 fast-forwarded to d9490b3 (his OK). The week4fix
  worktree was re-synced by Wesley (reset --hard, clean at d9490b3).
- Combined M1 on the new roles: survey done (no HANDOFF phase: the hand-back is /join_planner/handoff_ready raised in
  ATTACH_READY; the ring magnet ON comes only from our hand-off reaction; no tip-plate contact in either twin).
  Wesley: rig aims the face 1 cm below the plate; twin aims at the plate and holds; old partner world first
  (exploratory), rig twin for the claim (decisions.md). Card docs/experiments/2026-10-10_combined_m1_contact.md
  (switch partner_contact / his contact_attach, both default off). Wesley's three goals (magnet ON through his
  rejoin; contact + small lift proof by HIS controller; bumpless hand-over) -> card rev 2 after critic round 1
  (GO WITH CHANGES: our tracker would fly the welded QUAD4 to the release step-out point; checked true). Critic
  round 2: REWRITE (proof must use drift from the contact offset + plate rise, thresholds from one measured lift run;
  his plate index is CCW (+y = his 3); his MPC needs rig-twin values (thrust_ratio 22 + feedback, cable 0.55, drop
  0.59); bumpless = all four channels + heading + fold-in seeded from the measured pull; ON span only once airborne
  and the spawn joint free). Wesley: the subtest = 3 carriers lift, HIS controller takes QUAD4 off the mat, picks up,
  flies, drops into the ring, then attaches; no attach -> ours (tested abort). Needs a rig-twin world variant with
  his ball + drop target (world SDF: OK'd). Cage 6x6x4 m (memory). Lab PC: remote any day when free (Wesley).
- Branch m1-subtest (worktree /home/wesley/mdc_wt/m1-subtest = tejen-sync-check + rig-prep): 528b693 world variant
  three_rigid_ground_rig_attach_pickup + QUAD4 pickup model (generated), his launch geometry:=rig_twin, config
  m1_subtest_hover (today's hand-back). E0 running on the lab PC. Card rev 3 written (full mission from the mat, our
  abort when no attach, proof = drift from contact + plate rise, thresholds from E1 then frozen); critic round 3
  done: GO WITH CHANGES -> rev 4 (take-back specified as NEW code: our abort was only tested from our own approach;
  clearance pairs; E1 threshold mapping + stop rule; ball 0x05 + repeated pickup detach; transit 2.7 m; rig one coil:
  no OFF before the ON span; _land_transit). R1475 VOID (runner's old-world ball check; fixed 04242f4); E0 = R1476
  ran: R1476 exploratory (his takeoff + pickup work on the rig twin; his loaded lift ended 14 cm low with no z integral,
  under the C++ planner's 2.4 m floor -> WAITING_ABOVE_SEARCH_FLOOR to the 30 min cap). E0 rerun from a clean worktree
  /home/wesley/mdc_wt/m1-e0 (branch m1-e0 866b2b2: lift 2.3, floor 2.7, cap 80 min) running. Builders: manager +
  generator DONE (uncommitted in m1-subtest; contact speed = LSQ slope, the twin's 0.5 mm tip noise makes the
  per-sample speed ~85 mm/s); his side and our planner/tracker/mux/runner DONE. Integrated on m1-subtest b7bb2cb
  (+ fe0d7c2 lift fix): all package tests pass (known: tejen 12, bringup 2 lint), gate --quick passed, launch gate
  0 changed. My additions: WAIT_WELD timeout, 1 Hz contact level. E0 rerun R1477 (m1-e0): past the lift into
  TRANSIT_TO_DROP_POINT. Next: E1a (h_plate out of reach: measure) + E1b (start values: success path) on the lab PC
  after R1477. Open for Wesley: his retry after a failed proof re-presses from the lifted offset (<= 1 cm), not
  from the approach height (builder's call, to avoid a 0.29 m yank if partly caught).
- Runs (registry R1477-R1485): R1485 E0 laptop: his whole mission works on the rig twin (pickup, transit, drop into
  the net, rejoin); our old descent missed (my attach_cable_len 0.59 + pivot). E1b R1479 hover / R1480 orbit: the
  contact attach works end to end (contact 11-13 s after his descent, proof 0.7-0.9 s later, switch the same tick,
  bumpless), but the ring tilts 8.1 / 8.4 deg 1.5 s after the switch (plate 9 sags 2.5-3.5 cm: the fold-in seeds
  QUAD4 at its OCP share; seed switch contact_fold_seed f9536f3) and R1479 aborted at LAND (QUAD4 tipped in the
  landing unwind: rigid-link artefact). R1481 nocatch: no false attach, give-up, take-back out to 1.18 m, landed.
  R1482 LAND in his transit: bumpless take-back with the ball, but landed 0.33 m from carrier 1 (fix d226e85).
  R1483 E1a orbit: plate rise 5-6 mm at the lift peak, 0-3 mm held; plate noise sd 0.8-1.5 mm (0.1 s avg over 0.3 s).
  R1484 E1a hover; R1486-R1490 seed comparison: 'ocp' 4/4 > 5 deg (7.7-8.6), 'start' 3/3 <= 2.6 deg -> default
  'start' (80b9352). Proof thresholds: the card's mapping cannot be met (first-lift rise 4.5-6 mm vs plate noise sd
  0.8-1.5 mm; 3/7 first proofs false-rejected by timeout, retry proved; 0 false attaches) -> Wesley. LAND tip-over of
  QUAD4 in the landing unwind 2/7 (hover) -> Wesley. Card has the E0/E1 results section.
- Wesley (decisions.md): thresholds 1 cm / 2.5 mm / 3 / 8 mm, but FIRST check QUAD4's mocap-estimated cable force as
  an attach signal; landing tip-over = twin artefact (scored separately); retry = re-press (as built). Now: the force
  check (offline on R1479-R1490 + rig noise from the 7-8 Oct carriers' measured pull). tip_calibration prints body-to-tip (the taut length for the rig proof; Wednesday item 4). Build on a branch from
  tejen-sync-check after the critic. Scoring script kept at $CLAUDE_JOB_DIR/tmp/m1_stages.py (to tools/ with the configs).

## Open items / next steps (plan: /home/wesley/.claude/plans/whimsical-kindling-minsky.md)
- Reviewer pass on the final newcomer tables Monday before the freeze (graded WEAK on 9 Oct; re-flies done since).
- Unexplained: R1278 carrier flip in the creep (1 in 53). (Planner late ARM: explained, runner fixed e1c6e77.)
- C workstream (combined M1 stack with Tejen's code), P (Tejen port), X (tilt card + critic), R (Wed/Thu sheets,
  Tuesday desk rehearsal, send Tejen docs/tejen_questions_2026-10.md, QUAD1 checklist docs/rig_quad1_checklist.md).
- Re-freeze the launch gate baseline at week4 (backlog).

## Overnight 10-11 Oct (plan approved: /home/wesley/.claude/plans/parsed-inventing-mist.md)
- Claim batch R1491-R1501 (m1c_*, m1-subtest e097a94) on the lab, 4 slots; m1c_hover_2 on the laptop; m1c_orbit_2
  parked in $CLAUDE_JOB_DIR/tmp (restore, then fly it on the lab from e097a94). Monitor per finished run.
- Helpers: V twin videos (tools/twin_video.py, main tree, results/videos/), R rig port (worktree m1-rig), F full M1
  (worktree m1-full). Me: the integration rundown + real-life concerns (docs/m1_integration_rundown_2026-10-10.md).
- Wesley: think real life first (memory); no thesis now; lab PC any time it is free.
- 22:05: claim scored so far R1491 / R1502 hover PASS, R1492 / R1494 nocatch PASS, R1495 land_transit PASS, R1493 orbit
  no attempt (his pickup stalled: ball pushed by the rod, no timeout); R1496-R1499 flying, R1500-R1501 next, then
  m1c_orbit + m1c_orbit_2 queued (task b2jecacfc, waits on the batch PID). Monitor bbmd52gjc.
- V done: tools/twin_video.py committed (037589d), seven log-rendered videos in results/videos/ (frames checked).
  Gazebo-rendered attempt waits for the lab to free. R (m1-rig) and F (m1-full) still building.
- 22:50: R1496 lead030 PASS (2 alignment aborts, contact 22.5 s), R1499 offset PASS (retry), R1497 high = pickup stall
  (2nd, like R1493), R1498 low FALSIFIES the attach at a 1 cm low target (his proof lift counts from the target; fix =
  rise from z0, changes the claim tree -> Wesley); safe path fine. R1500/R1501 flying. Re-queue (pid 766533, waits on
  725916): m1c_orbit, m1c_orbit_2, m1c_high -> $CLAUDE_JOB_DIR/tmp/lab_m1c_orbit2.txt.
- F done: m1-full ca958da (tests pass). First exploratory m1_full_hover on the LAPTOP (task bs5jcfamg).

## Overnight loop 10-11 Oct (Wesley back ~10:00; decisions 22:20 in docs/decisions.md)
Order of work (each item finished well before the next; look at every run as it lands):
1. Claim on e097a94: R1491-R1502 logged (R1500 land_proof last). These rows are the record, not the final table.
2. Fixes (helper, worktree /home/wesley/mdc_wt/m1-final, branch m1-final from ca958da = e097a94 + full M1): his
   LIFT_PROOF rises from the latched contact height; bounded pickup retry (2 retries, then give-up -> our take-back).
   Review, tests, gate, commit on m1-final; claim_row.py tree string -> m1-final SHA; card rev 6 (same frozen bars).
3. Re-fly all 13 m1c arms on m1-final on the lab (4 slots), score each as it lands, then the reviewer on that table.
4. Full M1: exploratory m1_full_hover on the laptop (bs5jcfamg) + m1_full_orbit / m1_full_hover on the lab from
   m1-full (b2p5t0v9x, after the claim batch). Fix, one run on the failed arm; then on m1-final; card + critic for
   a full-M1 claim once it flies cleanly 2 of 2.
5. Gazebo recording (helper, worktree m1-rec): method + smoke; then ONE recording of the full M1 on m1-final.
6. Rig port (helper R, m1-rig): review, tests, commit; desk test on the laptop (flock the laptop lock).
7. Morning report ~09:45: rig findings first, table, verdict, decisions.
Laptop lock: flock $CLAUDE_JOB_DIR/tmp/laptop.lock for anything that starts ROS/Gazebo on the laptop. Lab: one tree
at a time (lab_batch syncs the tree), at most 4 slots. Monitor: lab_m1c / lab_m1full / lab_final / laptop logs.
- 22:45: R1501 abort PASS (logged). R1503 (laptop, m1-full, exploratory): welded start -> detach -> release -> his
  pickup works up to the ball weld, then his pickup descent runs 0.11 m lower than in the subtest -> tip on the
  floor, stall, QUAD4 tipped on his stream (no fleet abort). Helper F resumed to find the height datum (m1-full).
  RTF of all these runs is 0.15-0.19 (normal). Lab: R1500 last, then R1504/R1505 m1_full orbit/hover (b2p5t0v9x).
- 22:55: R1503 cause found (F): his MPC re-latches its kT estimator start height while disarmed and learns kT only
  0.20 m above it; armed in the air at the step-out (z 1.85) it never climbed 0.2 m -> kT stuck at 22 -> 11 cm low.
  Fix 84b1779 on m1-full (launch arg airborne_start auto on the /partner/release gate). RIG FINDINGS for the report:
  (a) the same latch on the rig if QUAD4 is released to him in the air; (b) nothing automatic stops a tipped QUAD4
  while his stream flies it (our tracker's disarm is not forwarded by the mux; Betaflight has no tilt disarm; only
  DISARM/ESTOP, which drops the ring) -> card docs/experiments/2026-10-11_quad4_tilt_cut.md (F, worktree
  m1-tiltcut), critic, build behind default-off switches, Wesley decides.

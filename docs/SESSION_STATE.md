# Session state (read after every compaction; keep it short and current)

Updated: Wed 14 Oct 2026, 17:55 AEDT (from `date`). Only what a new session needs to act. History: `git log -p docs/SESSION_STATE.md`,
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

## Wed 14 Oct rig day so far (10:26; registry RIG-1014-*; nothing committed yet: Wesley commits at the end of the day)
- **Uncommitted, to commit at the end of the day:** the sheet (W 0.024, K 0.024, plate 9 point X 0.013 / Y 0.210 in M,
  the 3 min load window), `configs/rig/attach_m1.yaml` (weld_point_z 0.024), `docs/decisions.md` (3 DECIDED lines 14 Oct),
  `results/registry.csv` (RIG-1014-*), and the radio fix in `elrs_interface.py` (a refused arm crashed the node: rclpy
  logger severity per call site; fixed with two call sites, 23 tests pass).
- **Props off:** streams ~91 Hz clean (still_2). W 0.024 (three catches, 0.6 mm spread). L kept 0.575 (moving read, peak
  0.572). Test 3: the caught tip centres 1.5 cm inside plate 9's nominal point; K 0.024 around (0.013, 0.210). Tejen's aim
  stays (0, 0.225), the outer -x corner of the catch area: tell him.
- **Load test:** lt_5 FAIL (two controllers starved 0.32 s at his TAKEOFF, load 16 on 12 cores); lt_6 FAIL on one line
  (our drone-3 tracker's warm start, 0.160 s) with every radio-bound stream passing; Wesley overrode F1 for test 5.
- **Test 5 crashed (t5_2):** the three-carrier lift in mode attach bobbed at ~1 Hz (throttle 0-0.5) with the loops late
  (QUAD3's tracker missed ~130 ms just before; the bag's 0.25 s all-topic gap was the recorder, see t5_2-corr), QUAD2's rod released, the ring went to 81 deg, the fleet disarmed from ~1 m (drop_on_loss is off in attach
  mode); QUAD2 flew off and hit the wall. QUAD4 and his stack were not involved. Sheet stop rule: no combined flights today.
- **Diagnosis so far:** the lift is the same code in every mode (dissipative inherits mpc_planner's LoadPlanner; the
  trackers fly the same MPC). The combined stack slows our loops (42-44 Hz, p99 37-47 ms vs 50 Hz, p99 26 ms alone).
  lift3_3 (same lift, our stack only, drop_on_loss on) flew clean to 1.0 m: throttle swing 2.4-2.6x smaller. Free hovers
  fh_q2 / fh3_2: all four airframes fly normally after the crash.
- **QUAD1/QUAD2 FC attitude drifts** 10-15 deg/min (mocap says 12-17 deg); past 25 deg they refuse to arm. Replug within
  ~30 s of fleet arm. Fix (Wesley's call): small_angle 180 or an accelerometer calibration.
- **Load review (during the break):** `docs/analysis/rig_load_2026-10-14.md` (options table, crash chain, order, decisions);
  `tools/rig_pin.sh` (draft, dry run by default: display and the recorder to the E-cores). Both uncommitted.
- **During the break (Wesley ~10:45, all uncommitted):** drop_on_loss on in attach (graph_attach, tracker tilt rule,
  welded coil OFF on a drop; CLAUDE.md locked rule updated; critic waived); load cuts (fleet_viz 10 Hz, tracker RViz 10 Hz,
  attach target off on the combined rig, telemetry prints 1 Hz, his A* off / CSV every 6 / trail 0). Tests 97 + gate pass.
  Desk: 795 % with the cuts; py-spy: rclpy executor overhead dominates (QoS event handlers off = -16 % in a bench).
  Twin delay sweep prepared: ~/handoff_2026-10-14/ (README; refuses dirty src unless committed or ALLOW_DIRTY_SRC=1).
- **11:35 (Wesley):** the lean switch on for the rig (rig_flight.sh exports MDC_LEAN_RCLPY=1 + PYTHONPATH; the sheet's w5 too);
  the bag drops /tejen/pendulum_swing_state and /fake_payload|attachment_point/pose (MCAP: only -10 % and 5 replay tools
  read .db3 directly, so not used); mocap capping NOT done (velocity lag, twin first). Still uncommitted (Wesley).
- **11:47:** mocap node TF off on the rig (publish_drone_tf); rig_pin.sh fixed (Python nodes now match) + --boost (sudo,
  control path nice -10). Next: props-off load test lt_7 with everything (lean, cuts, pin + boost), score on F1 vs lt_6.
- **14:17 lt_13 F1 PASS** (cuts + lean switch, no pinning): carriers 50 Hz p99 27 ms under the combined stack, every
  stream max <= 0.073 s, no faults (lt_6 failed). lt_7-lt_12 void (operator DISARM, Motive streamed QUAD4 only, QUAD3 TX
  USB drops at 12:22 and 14:04, tracker crash on an ARM during start-up, QUAD1/QUAD2 drift). Laptop rebooted ~13:42.
  14:25 liftc_3 PASS: the t5_2 lift under the combined stack flew clean (throttle sd 0.025-0.039, tilt max 8.1 deg, 50 Hz
  p99 25 ms; t5_2: 0.09-0.13, 16.6 deg, 42-44 Hz). Next: test 5 (no-catch mission) with Tejen, hold at 1.0 m first.
- **14:33 t5_4:** his pickup failed (tip swung 3-4 cm round the puck; 2.5 cm low tube held the descent); our side fine
  (F1 PASS over his flight, LAND take-back). Tejen's tuning applied as rig overrides (low_xy 0.045, high 0.06, low dwell
  1.0 s; retry unchanged). Next: test 5 again; read back /online_join_planner pickup_alignment_low_xy_m after launch.
- **14:49 t5_5:** pickup failed 3x (HIGH stall; then 2 catches with the puck not rising, 2.5-4.5 cm off); our coil gate
  and give-up take-back fine. Tubes now low 0.10 / high 0.12 (Wesley). Next: test 5 again, read back the params first.
- **Tejen's M2 review, our side (analysis doc):** our trackers send the 100 ms-ahead node-1 command every 20 ms (5x the
  modelled 0.5/s throttle slew; 19-34 % of ticks over in t5_2's bob, 0-2 % in clean lifts); the shared mocap node's
  alpha 0.2 velocity filter lags ~45 ms on the rig while the twin emulator does not filter; kT adaptation off. Not changed.
- **15:29 tracker timing card** (docs/experiments/2026-10-14_tracker_timing.md, rev 2 after the critic): A next-tick
  throttle + B carrier velocity filter, switches OFF (rig_flight unsets them). Twin tonight: lab_tonight.sh stage 1
  delay sweep, stage 2 the card's 4 arms at +40 ms (ALLOW_DIRTY_SRC=1); extend score_delay.py to the card's bars
  (bob band RMS, tilt, track, hold, switch-line checks) before scoring. Gate 701 pass.
- **15:50 no-pickup modes:** helper coding floor_join (B) + delivery_rejoin (A) in his planner; launch arg mission_mode
  (partner_args); held-ball twin world generated; twin configs m1c_floorjoin_hover / m1c_delivery_hover ready.
  Next: twin B (then A) on the laptop with the rig down, then B on the rig. Props-off puck check ~16:00 (cage busy).
- **16:00 Tejen's upstream** (~/tejen/drone_cage_control pulled to d00934c1): our fork's tejen_mpc is his pre-12 Oct
  controller. Since: swing weights (12 Oct), position/velocity weights 130->190 (13 Oct; his M1 pickup settle 2.7 s vs
  28.7 s, "less visible swing"), the 30 Hz timing fix + attitude bounds 3/s (d2eaf41e, flown in M2), Z 80/120 (14 Oct,
  his four-drone attach-proof-park success). His fast LOW stroke was tried and the speed/entry put back (40 mm/s,
  80 mm, as ours); overtravel stays 36 mm (ours 28). Wesley 16:05: port his HEAD controller behind a profile switch
  (default off; helper running), twin first; skip the fast LOW and the overtravel (decisions.md).
- **16:08 no-pickup modes coded** (helper; 30 new tests pass; refuses TAKEOFF on the mat with his landing disarm if not
  ready). Twin B1 (m1c_floorjoin_hover) running on the laptop (task b4tz8s5fo, log scratchpad/twin_B1.log). Then A.
- **End of day 14 Oct (17:55): packed up; everything still UNCOMMITTED (Wesley: leave it). Every rig run has a
  registry row (RIG-1014-*, morning gaps filled at the close).** Afternoon, in order:
  - Props-off puck check (RIG-1014-puck): contact 3.79 cm (his 3.45); the puck catches anywhere on its face.
  - t6 (full mission, pickup overrides lift dwell 1.5 / lift tol 0.04 / overtravel 36): the descent ended 6-7 cm off
    the puck; the lift gate refused it. Wesley: "just the attach" -> floor_join.
  - floor_join flown t6f..t6h5_3 (twin skipped first; twin smoke R1717 PASS with oct14 later). Found and fixed on the
    way: the takeoff race (his MPC lifts before his planner: send TAKEOFF with `ros2 topic pub -w N`); the tip swing
    (median 4-5 cm) -> match gate 5 cm / 0.15 (abort 8 / 0.25); his target 2 cm off the measured plate + the tip
    hanging 3 cm off -> contact_dx 0.044 / contact_dy -0.005 (new arg contact_dy); his oct14 controller flown once
    (t6g_2): no swing gain, back to oct10.
  - **t6h_4: the magnet caught plate 9** (drift < 1.1 mm, plate rose 6-26 mm) but OUR proof failed it on the taut check
    (proof_taut_len 0.572 above the in-flight taut 0.570) -> his give-up released it. 0.562 flown once (t6h5_3, no
    contact). OPEN in decisions.md.
  - t6h hop: no battery telemetry from QUAD1-3 -> no voltage correction on fresh packs; run preflight --real every ARM.
  - Two operator DISARMs with QUAD4 still ~0.3 m up on its own landing (t6f2_4, t6h5_3); one at 0.3-0.5 m dropped and
    flipped QUAD1/QUAD2 (t6h): check the airframes.
  - Controller port swapped in (mpc_profile oct10 default / oct14; both solvers prebuilt; gate --quick PASS 17:00).
    The swap script's final self-check had a wrong argument order (fixed by hand; script in ~/handoff_2026-10-14/).
  - Notes for Tejen: docs/tejen/notes_2026-10-14.md (Wesley passes them on).
- **20:30 Wesley approved the plan `docs/plan_after_rig_2026-10-14.md` (status table at its top; keep it current).**
  14 Oct work committed (c15abc4, 19804fb, 4209c62 + records). Executing in its order; Thursday tests what is done.
- **Next (Thursday 15:00-18:00 retries):** fly floor_join again with proof_taut_len 0.562 (`P=0.562` then redefine M),
  the target shift, `-w N` TAKEOFF, preflight --real; then the pickup only if Tejen agrees the lift waits on contact.
  Tonight's lab batch (tracker timing card, ~/handoff_2026-10-14/lab_tonight.sh) not started.
- **Open for Wesley:** how to run the combined stack without starving the controllers (second machine for his stack,
  less on this laptop, or priorities); the changes table misses that dissipative mode now flies int_mode model (8 Oct
  flew reference), tests 8-9 included; rig_summary.py crashes on free-hover bags.

## Wednesday 14 Oct (rig) and Thursday 15:00-18:00 (retries)
- **Sheet:** `docs/rig/wed_2026-10-14.md` (concise: setup, tests 1-12 with commands and what each shows).
  - 1-4 props off: streams, the contact height W, the catch radius K from hand catches, the 10-min load test.
  - 5-7 with Tejen: the no-catch mission, the contact attach twice, the full mission if clean.
  - 8-9 if time: the hover and circle releases against the frozen bands. 10-12 Thursday: the two circles, the magnet hold.
  - Every terminal: `w5` (sources the main tree, exports ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=23).
  - 13 Oct evening (multi-drone-control-02, Wesley's word): the catch radius is K (the launch variable was also R, a
    clash), and test 2 also reads QUAD4's taut length L; 0.575 is replaced only if the read is more than 1 cm off.
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

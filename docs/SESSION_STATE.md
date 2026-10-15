# Session state (read after every compaction; keep it short and current)

Updated: Fri 16 Oct 2026, 05:55 AEDT (from `date`). Only what a new session needs to act. History: `git log -p docs/SESSION_STATE.md`,
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

## Overnight 15-16 Oct (Wesley away until the morning: no questions, best option, note why)
- **Branch `overnight-1015`** (from week5 48ccad6): 1057c7e (the rig day's code, docs, registry rows RIG-1015-*),
  d4f8ce8 (traj3 configs). Plan: `docs/plan_after_rig_2026-10-15.md` (situation, ranked blockers, Mitch's numbers,
  work packages A1-A6 / B1-B3 / C1, the overnight queue). Loop: self-paced (ScheduleWakeup), morning report by 07:30.
- **05:15:** BUG fixed (9c04ee3): efb9b5d's moving-ring guard read the planner's `running` (true in a lifted hover),
  so attach_twist did nothing from 04:11 on, twin and rig. R1824-R1826 (before the guard) valid; R1828, R1830,
  R1831-R1834 flew raw (registry rows say so; the drift 'lag' was spread). attach_twist:=lowpass added (e85bb17).
  **05:55 (overnight done):** re-fly on the fixed guard: lowpass = zero twist at 1.8 Hz (R1835, R1842), worse under
  a 3 cm drift (R1836, R1839: median 1.5-1.6 vs raw 0.9-1.1 in R1829-R1831/R1841, zero 1.2-1.3 in R1837/R1840):
  lowpass code removed (b0398f6); zero twist no regression (R1838). Registry to R1842. No lab runs left; the lab PC is
  free. Full gate PASSES on the final tree (99ad8c7, 05:53). Loop stopped. Morning report final:
  `docs/reports/morning_report_2026-10-16.md` (8 decisions). Next: Wesley's decisions; nothing runs.
- **Built tonight (all off by default, gate --quick 1464 pass):** 35d2b86 console --slot-gate; ab6fd18 his
  match_dwell/match_speed typed args + tools/match_gate_replay.py (reproduces a1's descents); 8d2036c filter in the
  twin bridge + m1c_floorjoin_hover_zlp08; cca8803 three_rigid_ground_rig_L100.sdf + traj3_L100_*; 6856cf8
  tools/make_attach_worlds_L100.py, his geometry rig_twin_L100, m1c_floorjoin_hover_L100; bad76e3 drop_takes_back
  (card docs/experiments/2026-10-16_drop_takes_back.md rev 2 after the critic: fix first -> fixed).
- **Done:** B1 30/30 clean (registry R1723-R1752, docs/analysis/traj3_2026-10-15.md). drop_takes_back card rev 2:
  MATCH/release arms PASS (R1754/R1756/R1758), off arms reproduce a2_2 (R1753/R1755/R1757), bottom neighbour R1759
  FAILS bar 2 as frozen (starting geometry; rev 3 relative bar for later runs: bottom_tb x2 + bottom_off x1 queued).
  1 m attach R1760: contact, proof, fold-in, landed (free-hang 1.0405 m). Slide finding in the plan (tip off the
  plate's edge 3.3-4.3 cm along the ring).
- **04:20:** filter card rev 2 PASS for the filter (-35 % at 1.8 Hz), zero twist passes bar 1 (-27 %) but fails the
  confounded bar 2; rev 3 (0.8 Hz at rig amplitude) inconclusive as frozen, reported: filter worse (+17..+32 %), zero
  twist no different; reviewer SUPPORTED WITH CORRECTIONS (weak at 0.8 Hz). Zero-twist critic: fix first -> done
  (efb9b5d: moving-ring guard, choices, start-up log, rig bag records the twist); its rig falsifier passes
  (`match_gate_replay.py --zero-twist`: a3_2's gate opens at 4.9 s). No regression R1824-R1826. drop_takes_back: 8 on
  runs pass incl. early MATCH / DESCEND (R1815/R1816). Running: n6drift_{base,twist0} (0.25 Hz sideways drift,
  lab_batch8.log). Morning report nearly final; decisions: merge, drop_takes_back on, zero-twist A/B after Tejen.
- **02:26:** full gate (all 6 stages, SIL smoke included) PASSES on the branch (HEAD 3507fce). drop_takes_back reviewer:
  SUPPORTED WITH CORRECTIONS (applied, 0a3ac30). Filter critic: N6's 2/2 vs 0/2 withdrawn (sim weld + slope gate
  artefact); zero-twist switch built (attach_twist:=zero, e9ff80a); card rev 2 bars on the mechanism
  (tools/n6_mechanism.py), unseen runs. Batch 4: latency 15/30 ms = 0 ms, 45 ms tips at takeoff; registry to R1799.
  Batch 5 running (byl7adruz, scratchpad lab_batch5.log): rev 2 arms base/zlp08/twist0 x4, n6s08 x3, the two new drop
  cases (R1800-R1816), ~03:45. Supervisor brief draft docs/reports/supervisor_brief_2026-10-16.md.
- **01:25:** batches 2-3 scored (registry to R1789). drop_takes_back: rev 3 bottom runs PASS (R1779/R1780), off
  arm shows the hazard (R1781); reviewer running. 1 m carry = 0.55 m (R1769-R1778); 1 m attach 2/2. N6: at the rig's
  0.85 cm plate bounce the filter attached 2/2, the baseline 0/2 (R1782-R1785); card
  docs/experiments/2026-10-16_plate_height_filter.md (bars frozen 01:20), critic running. Landing tip-over = twin
  artefact in both arms. Batch 4 running (b7epuuj91, via ~/handoff_2026-10-14/delay_batch.py, scratchpad
  lab_batch4.log): n6_arm_base/zlp08 x2 more, traj3d_{circle,fig8}_v300_d{015,030,045}. Morning report draft
  committed (docs/reports/morning_report_2026-10-16.md). Draft carry card 2026-10-16_carry_speed_envelope.md.
- **Batch 2 (done; b7ctlzpk9, scratchpad lab_batch2.log), R1753-R1778:** w4_drop_his_match_{off,tb} x2, w4_release_his_match_{off,tb} x1,
  w4_drop_his_bottom_tb x1, m1c_floorjoin_hover x2 + _zlp08 x2, m1c_floorjoin_hover_L100 x2, traj3_L100_* x2.
- **Background agent:** sim results inventory -> docs/analysis/sim_results_inventory_2026-10-16.md.
- **Wesley's chat (15 Oct evening):** four quads 98 dB(A): lab stopped them; Mitch aims at four-drone approval by
  Wed 21 Oct (early slots), no flying Fri; next hardware: bigger discs, 1 m cables, maybe a smaller basket.

## Now (Thu 15 Oct, 06:59): ready for today's lab (15:00-18:00)
- **Tree:** week5 at 8862e6d, clean; quick gate passes (1453). Plan `docs/plan_after_rig_2026-10-14.md`: rows 0-8 done
  or ready; row 9 (after the lab) to do. Today: sheet `docs/rig/thu_2026-10-15.md`, card
  `docs/experiments/2026-10-15_attach_comotion_rig.md` (rev 5), morning report `docs/reports/morning_report_2026-10-15.md`.
- **Tonight's commits (in order):** c15abc4..a0d3963 (14 Oct work), c1ebdc9, 3297746 + dade70d (WP8 arming/landing),
  eacab58 (WP3 attach check), 4a60ca3 (RViz markers; its renumbering reverted), a73d9a4 (console, quiet T2), f9d0b72
  (RViz panel), 657fee1 (attach_window), 4beb32f (trim_speed_gate), bfbd7a9 (plates clockwise again), f035625 (critic
  rounds 1-2 fixes), 305e50c (typed/shown angles clockwise, same names flipped), 33ceec4 (console mid-flight),
  a5bf17d, 0e47f9b (Tejen's first review), 496c11f, 76180f8 (Tejen's second review), 8862e6d (R1722).
- **Twin:** R1718 PASS, R1719 FAIL (led to rev 2), R1720-R1722 PASS (R1722 on 76180f8: contact, proof on his lift signal
  only, fold-in at plate 9 / 270 deg cw, clean landing).
- **Tejen:** `~/tejen_review_package_2026-10-15.zip` (19.5 MB, 76180f8 + the t6h_4 bag + README answering both of his
  review rounds): ready; Wesley sends it. Tejen OK'd his trim at 0.20 m/s above contact (aim B); keep the lift proof;
  no M2 modes, no TAKEOFF change. His puck height 37.9 / 36.2 mm vs 34.5 assumed (for WP9).
- **Wesley's decisions before 15:00:** QUAD4's aim A (t6h_4 offset, trim off; the sheet's default) or B
  (`trim_speed_gate:=0.20`); send `docs/rig/fc_request.md`; send the zip.
- **At the lab (the sheet's order):** props-off console + RViz panel check (the panel is compile-checked only), the proof
  bench by hand (negatives under `dry_hold:=true`), FC acceptance if the FCs were changed; one dry-hold flight; up to
  four attach flights, each scored with `python3 tools/proof_replay.py --catch <run> ...` before the next.
- **Offered, awaiting Wesley's answer:** a twin config that switches the bottom hold on (it is rig-only, so its
  quiet-miss and plate-frame logic has never flown in the twin), ~30 min incl. one run.
- **After the lab (plan row 9):** WP4 wind matrix only if contacts stay rare (supervisor's list, my order: 90 mm discs,
  weight on the magnet, 1 m tether; 45 deg carriers only with stronger carrier magnets; hover-only configs not written);
  WP7 rename (our planner into mpc_planner, mode detach, network controller off week5 to `legacy`, the gate runs the
  ~170 rig-path planner tests); WP9 pickup / full mission.
- **Supervisor answers already given (chat 14 Oct night):** FC accelerometer logs help the arming issue more than the
  attach (mocap gives the body acceleration); 90 mm discs: inside the catch 35 % vs 16 % of the 14 Oct hold time.
- **Scratchpad:** the earlier session's scratchpad is gone; temp files go to /tmp.
- **Supervisor, 15 Oct morning (chat):** he makes 1 m tethers for next week, looks for bigger discs and a non-flat
  shape, suggests 50 g taped on QUAD4's magnet (Wesley: the magnet, not the ring; ring mass stays 0.86 kg). Wesley
  records Betaflight blackbox logs himself (FC accelerometer vs mocap; the supervisor could add live accel in ~1 week
  if it helps). Laptop: Chrome 155 installed 15 Oct (for app.betaflight.com), ModemManager active (stop it before plugging
  an FC), wesley in dialout, no log decoder yet. He may do `docs/rig/fc_request.md` himself in the same Configurator
  session. Next week's hardware touches the tether length (taut fallback, his planner, the twin), the disc radius (K)
  and shape (contact band): attach geometry and gate thresholds, Wesley's word.
- **Quads and FCs (Wesley 15 Oct):** all four quads are Wesley's (QUAD4 too); any FC setting change needs the
  supervisor's OK first. FC backups in `docs/rig/fc/` (QUAD4 read 08:2x: 4.3.1, TMOTORF7_AIO, blackbox SPIFLASH,
  NORMAL mode = logs while armed, so the supervisor's FC-vs-mocap comparison needs no change).
- **QUAD4 solo, FC vs mocap (RIG-1015-q4solo, 08:53, exploratory):** free_hover alone, hover 0.83 m. FC log
  `results/rig/2026-10-15/q4solo_fc/q4solo_QUAD4.bbl` (starts ~0.4 s after our ARM; 16 MB = ~5.5 min at 2 kHz). FC accel
  agrees with mocap to ~0.1 m/s^2 rms after a constant offset; the offset is the FC's z axis (+2.5 %: 1.022 g on the mat,
  1.027 g in hover, not the cable); x/y offsets ~1 deg (board or Motive axes); QUAD4's FC attitude within 0.5 deg of mocap.
  Hover gives little sideways signal. Landing stalled at 0.28 m (operator DISARM, ~0.24 m drop): expect it again.
  Decoder + comparison scripts are in the session scratchpad only (`tools/` if Wesley wants them kept).
- **Circle for that comparison (Wesley 11:5x: 0.5 m, 6 s laps, no twin, he tests it on the rig):** free_hover
  `circle_radius` etc. (real.yaml modes.free_hover, 0 = off; one drone only), on via
  `params_file:=configs/rig/free_hover_circle.yaml` (0.52 m/s, 0.55 m/s^2, 3 laps + one period spiralling out and in
  around the takeoff point, ccw from above, 5 s after hover_z; LAND mid-circle descends where it is). Unit test +
  desk check (isolated domain: full circle, LAND mid-circle, /fleet/landed) + gate --quick pass. Uncommitted.
- **Circle flown (RIG-1015-q4circle_2, 12:43; q4circle VOID: no mocap until the mocap PC side was fixed ~12:40):**
  laps 0.50 m/s, radius 0.477 (sd 0.006), z sd 5 mm. FC accel rotated to the world by the mocap attitude vs mocap
  d2p/dt2: x/y corr 0.97/0.95, rms 0.09/0.10 m/s^2 at 5 Hz (0.05-0.07 at 2 Hz); z offset +0.22 again. Landing stalled at
  0.29 m again (DISARM). FC log `results/rig/2026-10-15/q4circle_2_fc/q4circle_2_QUAD4.bbl`. Wesley: don't commit yet.
- **Lab 15 Oct afternoon:** Tejen set up (ring turned, yaw now -22 deg; carriers QUAD1/2/3 on plates 3/7/11 after
  two moves; QUAD1/QUAD2 `!ERR*` cleared by replugging packs). c1_3 passed (all four armed 2.8 s, SPACE). Wesley: aim B
  (decisions.md), bench and dry hold skipped, straight to a1. **a1:** no CONTACT (tip slid 6-8 cm inward off plate 9 at
  the bottom twice, coil ON), QUAD4's tether passed 0.10 m from QUAD2 on the way in (his planner has no avoidance; mat
  moved for a2 to straight out from plate 9). Zip for Tejen `~/a1_2026-10-15_for_tejen.zip`. Ring bounce: plate 0.8 cm
  at 1.8 Hz copied into his reference; QUAD4 followed 0.25 s late -> tip vs plate 1.6 cm. Built `attach_z_lowpass`
  (ring_bridge, partner_args, Hz, off by default; unit tests, gate --quick 1457 pass); replay on a1: 0.8 Hz cuts the
  bounce 0.82 -> 0.13 cm, slow lag error rms 0.5 cm (max 1.8). Critic and twin not run (lab): Wesley's waiver needed.
  Wesley waived critic + twin for 0.8 Hz; console pack rule lowered 24.0 -> 22.6 V (Wesley). **a2** launched, not
  armed; **a2_2** flew WITHOUT the filter (not in partner_args): at +20.9 s after his TAKEOFF (his MATCH, QUAD4 0.57 m
  above the ring, nothing touching) QUAD2 let go of plate 7 (rod 0.57 -> 0.76 m in 0.1 s, LQ 97-99, pack 21.9 V);
  tilt 36.8 deg -> drop rule (all magnets OFF, carriers landed alone). LAND (not GIVE UP) left QUAD4 on his mission
  for 14 s toward the dropped ring (the card's open item). Card stop rule: tilt > 30 deg = no more combined flights.
  Wesley (~16:15): keeps the 22.6 V rule, replaces the packs and flies on (overrides the tilt stop rule), filter on.
  **a3** (16:14, filter on): carriers ~2 plates off (slot errors +51..+56 deg, flown anyway): pretension stretched QUAD2's
  rod 9 cm -> unannounced detach -> drop with the ring still on the floor. **a3_2** (layout within 4 deg, filter 0.8
  confirmed on the bridge): 42 s in his MATCH, never descended: his gate (tip xy < 5 cm AND rel speed < 0.15 m/s for
  1.0 s) held both only 36 % of the time, longest 1.03 s (replay reproduces a1's pass times exactly). Plate target
  speed rms 0.09 m/s (ring rocking + the bridge's raw 50 Hz finite difference). Offered dwell 0.5 s / 0.20 m/s
  (replay: a3_2 12 / 8 chances): Wesley skipped it for now.

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

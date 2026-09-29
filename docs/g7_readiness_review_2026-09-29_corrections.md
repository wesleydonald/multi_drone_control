# Adversarial check: sim-to-real risk register and test ladder

**Scope.** Read-only. I edited nothing and launched nothing. I checked the cited lines against the repo, the 09-16 and 09-23 rig tracker logs (`results/logs/controller_quad_load/planner_drone*`, `load_planner_*/params.json`), `results/registry.csv`, the udev rules and `~/.bash_history`.

## A. Wrong or refuted

1. **Ladder §0.3 and K1: "the loaded operating point may not show before the lift; the drones pull the ring off the floor on position error alone".** Refuted.
   - The resting gate zeroes only the prediction model's cable term (`ref_cable`, controller_mpc.py:1172-1174).
   - The throttle feedforward is `planner_ref_acc` = thrust_vec/m_i, and that includes the tension (planner_solver.py:128-131; planner_node.py:10-11: "magnitude sets the tracker throttle").
   - R0726 (the R3a HOLD twin) held throttle 0.181, which is the loaded hover value. The ring sat at z 0.032, inside rest + 0.05, so the gate was active and the load was still carried.
   - So R3a HOLD is already a loaded-throttle test, and the 0.6 cap would bind there first. The claim "the lift go/no-go has to come from the arithmetic, not R3a" is wrong. R3a is the measured go/no-go.
   - The "6 m/s² per metre, 14× the sim" stiffness has no file or run cited: unsupported.
2. **§0.3: "payload_rest_z (0.10 m on the rig)".** The default is 0.05 (real_control_launch.py:98; real_mode.py `RIG_DEFAULTS`). The threshold, rest + 0.05, is 0.10.
3. **K7: "the sim weld capsized at 2× loop gain (R0015-R0019)".** Those ids are not in the registry and not in any doc: unsupported.
4. **K23: "three-drone rig ticks max 36-50 ms, no gap over 50 ms".** The 09-16 logs have maximum gaps of 63-65 ms (planner_drone1_20260916_161530, _163835; planner_drone2_20260916_163630). That is still well under the 0.25 s watchdog.
5. **K23: "trackers about 0.9 core each (CURRENT_STATE.md:777-778)".** The line is CURRENT_STATE.md:766, and it says "while DISARMED". Lines 777-778 are about clean_slate.
6. **K11: "drone 1 jumped to 123-156° tilt 37 times".** A recount of the three logs gives 39 samples above 90° (2 + 29 + 8), ranging 121-172°. Durations were 0.4, 4.2 and 4.3 s.
7. **K2: "His M1: body 12 and ring 9, while ours uses ring 8".**
   - His M1 does not use a ring: `physical_ring_used_for_m1: false`, and 9 is marked "reserved" (irl_commissioning.yaml:9, :29-30).
   - His M2 ring is 8 (m2_irl_two_drone.yaml:5).
   - Our code default is 8 (motion_capture_publisher_node.py:33), but the sheet's example types 9 (tests.txt:48).
   - The real conflict is the sheet versus the code default, which is Q3.
8. **K5: "a one-shot ON sent to /drone_0 only".** The history also has `/drone_1/magnet ON` (~/.bash_history:1776, with a typo `/drone1` at :1775) as well as `/drone_0` at :1880. There is nothing for drone_2. The timing relative to the launches is unknown.
9. **K5: "the ring moved less than 0.3 mm on 09-23".** No payload log exists: `load_planner_*` holds only params.json and the tracker CSVs have no payload column. Ladder:18 gives 0.050-0.053 without a file. The number is unsupported.
10. **Ladder §0.1: "u 0.446 / 0.446 / 0.459".** The medians are 0.451 / 0.446 / 0.459 for d0 / d1 / d2 (planner_drone*_20260916_15091*). The drones sat 0.146-0.199 m low, not 10-18 cm. Drone 2 was on the 23.3 V pack.
11. **§6 item 2: "Step 5 tests ESTOP only".**
    - Step 5 tests the pose-timeout abort (ARM, TAKEOFF, cover markers) and then ESTOP (tests.txt:87-89).
    - What it does not test is the DISARM kill switch (spacebar or panel).
    - Also, ladder:125 says "ESTOP (… or the panel)", not ESTOP only (K25).
12. **R0a': "the R3b twin (ocp_hover_ground_creep_defaults) in SIL".**
    - That file is a Gazebo experiment config (world three_rigid_ground.sdf, mpc_quad_load_launch.py). SIL runs `configs/sil/*` from a taut airborne start (tools/sil/scenario.py:187-194).
    - A SIL arm needs a `configs/sil/carry_hover_n3*` variant with `thrust_c` 22.4.
    - Even then it cannot show the R3a HOLD or the floor lift, which item 1 says is where the cap first binds.
13. **K3 detection and the R0a check: "exactly one publisher (elrs_mux_i)".** That holds only on the M1/M2 graphs. The R0-R4 `real_control` graph has no mux, so the one publisher is `controller_i`.

## B. Overstated, stale, or mis-cited (the substance holds)

14. **K17.**
    - "Mocap rate below 120 Hz; preflight 58-95, never 120": the preflight rate is a Python `spin_once` message count over 4 s (preflight.py:124-142), so it is a lower bound.
    - The node comment says Motive was measured at about 475 Hz per body (motion_capture_publisher_node.py:41-48).
    - Rig readings: 59 Hz (20260923_130656), 95 Hz (_134509), 50 Hz (20260916_105956).
    - "EMA about 33-37 ms" assumes 120 Hz. At 60-95 Hz effective it is about 42-67 ms.
    - The `now - last < 1/120` decimation (:458-466) drops packets that arrive early because of jitter. That supports the inferred bimodal dt if Motive runs at 120.
15. **K19: "+25 % aborted 2/2 (E10)".** This is stale. It was the 2026-09-10 attach-during-circle demo with a mass-belief error (robustness_table.md:11; CURRENT_STATE §4.7), before kt_trim, z_ki and the linear plant. It is not a carry-hover result.
16. **K8: "z_ki cannot push up against a typed-high gain (params.py:93-94)".** params.py only shows the 0.15 m bound, and its node default for z_ki is 0.0 (the launch sets 0.4). The "no upward authority against a typed-high gain" result is at decisions.md:23 (SIL).
17. **K4: "the auto-land commands the magnet OFF (online_join_planner.py:1615-1621)".** Those lines only declare the 30 s parameter. The OFF comes from the LANDING phase spec (mission_definitions.py:259) and the transition at online_join_planner.py:5730-5739.
18. **K10 line numbers.**
    - The 0.5 s radio disarm with the "0.1 s" log text is at elrs_interface.py:299-303.
    - The 10 Hz telemetry republish is at :290-297. Lines 237-250 parse link statistics.
19. **K10: "a CLI `--once` ESTOP pays start-up latency".** This is understated. The repo has sim evidence of silently lost one-shot commands and samples with about 60 DDS participants (configs/dds/fastdds_udp_only.xml header; R0602, R0612, R0617, R0619, R0621). That bears on the sheet's `pub --once` ARM, TAKEOFF and ESTOP (tests.txt:15-16) and on any combined graph.
20. **K12: "a drone-3 pose timeout while its mux is on partner is only a warning (main.py:216-224)".** The warning fires on the tracker's disarm feedback while the mux is 'partner' (main.py:213-221). The substance holds.
21. **K9: "7 failed, 51 passed".** The 7 is documented (notes_for_tejen.md:29). The 51 could not be verified read-only.
22. **K25 and §6 item 12: "tests.txt:170-345 obsolete".** The obsolete part runs from 170 to 386; the file is 386 lines.
23. **Launch docstrings.**
    - real_dissipative says "FIXED" at :45-46.
    - It says "load_mass defaults to 0.1" at :49-50; the actual default is 0.86 at :72.
    - It says "handover_elev_deg 0.0 on the taut real start" at :52-53; the actual default is 45.
24. **§0.2 and K5: comparing the loaded launches with free hover.** The four launches used different configs (load_planner params):
    - 09-16 16:11 and 09-23 13:36: `start_taut:=true`, load_mass 0.6.
    - 09-23 13:47: `start_taut:=true`, load_mass 0.86.
    - 09-16 16:38: creep, load_mass 1.0.
    - The no-coupling conclusion from the drone-1 geometry still holds: z 0.63 / 0.59 / 0.65 / 0.60 confirmed.
25. **The §0.1 formula and table.** The arithmetic checks: R0726 gives 0.179 predicted; the mass thresholds come out at 0.96 / 1.12 / 0.72 kg. It assumes the u_free airframe is the same as the loaded one (rods and magnet on), and the sheet leaves that open (tests.txt:99).

## C. Established facts the drafts missed

26. **K13, drone 0 on the floor.**
    - In all four launches (09-16 16:13 and 16:14, 09-23 13:08 and 13:44), drone 0's pose stayed identical to the millimetre while u2 was 0.37-0.54 for 1.3-11 s. Attitude varied by at most 0.001.
    - Its `ref_z` never left the floor.
    - So "did not respond" means no motion at all. The logs cannot tell a disarmed FC or stopped motors from a frozen, repeated pose (K12).
27. **K2 and K3 have a concrete collision path.**
    - His M2 IRL namespaces are drone_0 = quad2 and drone_1 = quad4 (m2_irl_two_drone.launch.py:24).
    - His router publishes `/drone_0/motion_capture_state` for body 12 (m2_irl_mocap_router.py:98-99). Ours publishes it for body 11.
    - His relative `ELRSCommand` under /drone_0 reaches our QUAD1 radio.
    - Result: one topic carries two airframes. This is crash-level, not just "maps disagree".
28. **Missing risk: udev overwrite.** Tejen's udev install overwrites /etc/udev/rules.d/99-elrs-quad.rules and would leave only QUAD2 and QUAD4 (notes_for_tejen.md:17). Also, `real_io_launch.py:3` and the udev file header still say "defaults to /dev/QUAD<i>".
29. **Pack state already flown.** Starting voltages on 09-23 were 21.8-22.4 V on drones 0 and 1, below both the 22.8 V preflight bar and the sheet's 24.0 V. Drone 1 ended at 16.9 V (131300).

## D. Ranking

- **K13: M → H.** It happened in 4 of the 11 loaded rig launches (09-16 16:11-16:42, 09-23 13:08-13:47) over two days, the cause is unknown, and the pose was frozen solid (item 26).
- **K20: M → H.** Two of the four creep launches spun (16:40 and 16:42). My recompute gives yaw excursions of 180.5° and 330.6°, with u3 at 1.00. The creep is now the rig default and R3a flies it.
- **K23: L → M for R0c onward.** Lost-sample evidence in sim (item 19) plus `pub --once` commands. L is fine for R0-R4.
- **K12:** consider raising it. The frozen-pose mode may already have happened (item 26) and cannot be ruled out.
- **K1:** keep H. Its first measurement is R3a HOLD (item 1), not R3b0.
- **K17:** M is fine. Ladder P9 says it is not a blocker for R1-R4.

## E. Checked and holds

Everything below matched the cited lines, logs or rows:

- The throttle bound and feedforward clip (acados.py:133-136, :272; velocity_loop.py:93) and the kt_trim band (controller_mpc.py:989).
- The clean_slate kill list (clean_slate.sh:41-65, :92, :101), which has killed drone_communication since 08-04, so K6 holds.
- `real_io_launch.py` defaulting to /dev/QUAD{i} (:155-158); real_mode maps drone i to QUAD i+1 (:139-142); udev has four unique serials.
- real_dissipative defaults: `reconfig_mode` network (:177), `detach_magnet` false (:183), settle 1.0 (:77), no attach_z argument. three_attach has no attach_z argument, so ATTACH_Z is 0.025 (params.py:20).
- Weld detection: radius 0.08 (three_attach_launch.py:279), speed 0.05, dwell 0.15 (:806-807), `weld_vel_filter_s` 0.0 (:238), tip body None (motion_capture_publisher_node.py:30). No payload-staleness fault exists; only planner_node.py:1032 checks freshness.
- Partner side: his mocap loop still has `return None` in his repo (~/tejen HEAD d9c69415) while our fork has the fix; his MPC sets channels 5-10 to 1.0 (tejen_mpc/main.py:2451, :2454); his M1 radio sets no serial_port; his ELRSCommand is not remapped; the magnet latches ON at boot in partner_m2 (dissipative_launch.py:80); the M2 driver's arguments (drive_m2_handover.py:139-146) are missing from the docstring command.
- Watchdogs and gates: pose watchdog on the wall clock (controller_mpc.py:1076-1084); ARMING_FEEDBACK_TIMEOUT_SEC is unused; nothing reads Telemetry.mode; kT is checked only for kT > 0 (real_mode.py, preflight.py:168).
- Sag: 0.4-0.5 V in 14 s. The 23.3 V pack hovered lowest.
- Spin launches were creep launches, and the fourth airframe has only sim logs.
- Doc conflicts: R3a bars (tests.txt vs ladder:223 vs ladder:482), card line 103 and line 4, the kt_trim hole (tests.txt:141 vs decisions.md:19), CURRENT_STATE §9 and §7, and real_attach_launch.py:155, which now refuses to start.
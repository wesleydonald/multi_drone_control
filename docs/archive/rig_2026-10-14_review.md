# Red-team of Wednesday 14 October (sheet `docs/rig/rig_2026-10-14_commands.md` at rig-prep 9c9dcc2; load test and card on week5 76594ba)

Written 12 Oct, lane 5, read-only on week5 (`/home/wesley/mdc_wt/week5`): the sheet's P1-P13 and R1-R9 walked through
the code each line runs and the twin runs that exercised each path. Findings are ranked most severe first, each with
its fix. Nothing on week5 or the sheets is changed here; the other session applies the fixes.

**Verdict.** Nothing on the sheet is unsafe as written: every flight's stop rules and the exits hold up in the code.
Three things would stop the day or waste it: the 09:00 I/O launch cannot see the magnet tip or the puck, so W and L
cannot be measured (finding 1); P7's attach check runs with the weld point at zero and records no catches (finding 2);
and the pack budget for a day of about 19 minutes of carrier flight is not on the sheet (finding 4). R9's chain is
consistent in the code and matches the twin's full mission step for step, but two of its links have never run on the
rig and are rehearsable props-off on the morning (finding 3).

## 1. Findings, most severe first

| # | Severity | Finding | Evidence | Fix |
|---|---|---|---|---|
| 1 | **High: stops P2, P3, P4, P10** | T1's line `rio num_drones:=4 magnet_channel:=6 magnet_initial:=ON` starts the mocap node with no tip body and no extra bodies, so `/magnet_tip_pose` (body 24) and the puck (body 6) are never published. `tip_calibration.py` reads `/magnet_tip_pose` and `/drone_3/motion_capture_state` and prints "no tip frames with a ring pose" without them: no W, no L, no d_ok noise, no marker-to-face. P2's gap survey of body 24 and P10's puck check have nothing to echo either. P7 is unaffected (its `$A` line types the tip body). | `real_io_launch.py:76-80` (both default `''`); `tools/tip_calibration.py:95-97`; `graph_attach.py:52` (`BALL_ROUTE = '6:/pickup/motion_capture_state'`). | T1 for the whole day: `rio num_drones:=4 magnet_channel:=6 magnet_initial:=ON mocap_magnet_tip_body_id:=24 mocap_extra_bodies:=6:/pickup/motion_capture_state`. Harmless for R1-R5. P10 then reads `ros2 topic hz /pickup/motion_capture_state`. |
| 2 | **High: P7 records nothing usable** | `$A weld_radius:=0.01 p7` loads `attach_m1.yaml`, whose `weld_point_z: 0.0` puts the plate point on the ring-origin plane. The manager's contact test is the 3-D distance from the tip marker to that point (`magnet_attachment_manager.py:435, 850, 865`), so a face resting on plate 9 reads d of about W (about 3 cm) against a 1 cm radius: no `ATTACH:` line in (b), and every catch's d and dz are biased by W. The sheet's R (max dxy + 5 mm) would be read off a wrong origin. | `configs/rig/attach_m1.yaml:14-15`; the manager lines above. | Run P7 after P3 and type the weld point: `$A weld_radius:=0.01 weld_point_z:=$W p7`. Also put P3's W into `attach_m1.yaml` (the file says "until then: today's values") so a forgotten `weld_point_z` on a later line fails loudly rather than silently. |
| 3 | **Medium-high: R9's two unflown links** | R9 is the first welded start on the rig. The chain is consistent in the code and is the twin's chain (R1655 hover, R1631/R1633 circle): the coil latches ON at boot (`attach_magnet_initial` auto, `graph_attach.py:526-528`); `/fleet/detach 3` reaches `_detach_ocp`, which releases the reserved drone with `/magnet/command OFF`, the manager latches OFF on `/drone_3/magnet_ours`, and the gate (ours from boot) forwards it to the coil (`dissipative_node.py:1291`, `magnet_attachment_manager.py:566-577`, `magnet_gate.py:68-70`); the step-out target is 0.5 m out and 0.3 m up, then `/partner/release {"running": true}` when QUAD4 is quiet (`dissipative_node.py:382-385, 1610-1622`); the mux keeps our stream until his command is armed above idle and cancels the release on a LAND before that (`elrs_mux.py:283-300, 323-327`); the gate hands the coil to him on the release (`magnet_gate.py:153`). Two links differ from the twin: (a) on the rig his supervisor is refused (`partner_mission.launch.py:452`), so his ARM and TAKEOFF are typed on T4, as the sheet says, where the twin's full mission used his supervisor on the release gate; his in-air TAKEOFF under a typed command has only the twin's airborne-mode evidence. (b) His MPC sits armed-but-not-flying for about 40 s while we lift and carry QUAD4; if any of his watchdogs disarms and shuts it down in that time, the release finds no stream, the stop rule's LAND takes QUAD4 back (safe), and the flight is lost. Also `airborne_start` resolves to false on the rig (the gate is not `/partner/release`), so his kT estimator learns only 0.2 m above his arm height; that is the floor, so it learns in the air anyway, but it is not the configuration the twin flew. | as cited | (i) Add `airborne_start:=true` to R9's `partner_args` so his MPC runs the twin's airborne configuration (`partner_mission.launch.py:144-147, 359, 666`). (ii) Rehearse link (b) in P13 at no extra rig time: with the load-test graph up and props off, type his ARM with QUAD4 caught on plate 9 and the ring on the floor, then lift the ring and QUAD4 by hand to 1 m and hold 60 s: T2 must show no `disarm triggered` line from his MPC and `/drone_3/mux_state` stays `ours`. (iii) Keep the sheet's stop rule as written. |
| 4 | **Medium: the pack budget is missing** | Carrier flight time on the sheet is about 19 min and QUAD4's about 15 min (table in section 2), plus 10 min armed at idle in P13. No pack count, no swap plan and no charging slot is on the sheet; only the 23.6 V floor. R1's purpose (does a pack last R9's 3 min) answers the per-flight question, not the day's. | section 2 | Put the number of charged packs per drone on the sheet, and swap all four packs before R6 and again before R9 whatever the voltage reads; charge between the morning block and R1, and during P13. If fewer than three sets exist, R2 and R3 are the ones to drop (they are repeatable on Thursday), not R4/R5. |
| 5 | Medium: what the bags lack for the thesis | `rig_flight.sh` records a fixed list. For R6-R9 it has our side and `/join_planner/*`, but not `/tejen/drone_command` (his ARM and TAKEOFF times, the only record of when the typed commands went) nor his MPC's own state and reference topics, so his height error at contact and his kT estimate, which the Thursday R10a question turns on, would come only from his logs. | `tools/rig_flight.sh:97-104` | Add `/tejen/drone_command` and every `/tejen/*` topic his MPC publishes (list them with `ros2 topic list \| grep /tejen` during P13) to the combined list; or record `-a` for R6-R9 (no cameras on this laptop, so the bag stays small). Ask Tejen to keep his own logs for the four flights. |
| 6 | Medium: R2 and R3 cannot give the sd the falsifier names | R3's falsifier is "within max(1 cm, the lap-to-lap sd) of 2.91 cm", but both flights fly one lap ("fleet land after one full lap"), so there is no lap-to-lap sd and the 1 cm floor is all that applies. | sheet R2/R3 | Fly two laps on each (`fleet land` after about 50 s instead of 25): 25 s more per flight, and the sd exists. |
| 7 | Medium: the release instant for R4 is counted from an unnamed moment | "Release 6-7 s after the ring reaches 1.0 m": the band (R1626/R1627) was computed at the twin's timing, and the T6 pre-window wants the reference still for 5 s before the release. The sheet does not say which T2 line marks "reaches 1.0 m". | freeze doc 2.2; sheet R4 | Name the line: count from T2's lift-complete line (the one the twin's runner waits on, `hover hold done` for the circle; the lift-complete line for the hover), and type the OFF at +6 s by the T2 clock. |
| 8 | Low: the load-test document expects a stale commit | "T3: `git log -1 --format=%h` prints `0628d85`" is rig-wed's head; week5 is 76594ba or later, so the check fails and will be ignored. | `docs/rig/rig_2026-10-14_load_test.md:22` | Change to "prints 76594ba or later on week5". |
| 9 | Low: P12(a)/(b) have no record command | rig_flight records only around a launch; the props-off bags need a plain record. | sheet P12 | `ros2 bag record -o results/rig/2026-10-14/p12a /drone_3/motion_capture_state /magnet_tip_pose /payload/motion_capture_state /pickup/motion_capture_state`, with T1 up (after finding 1). |
| 10 | Low: R3 changes two things, one of them inert | `feedforward_60_hold.yaml` sets `cable_source: model` and `cable_accel_cap: 6.0`; the measured-pull default caps at 12. At 60 deg the planned pull is about 4.4 m/s2 per drone, under 6, so the cap does not bind, but a reader of the table will ask. | `configs/rig/feedforward_60_hold.yaml` | State on the sheet that the cap is inert at this load, or set it to 12 in the file so the pair is one variable by construction. |
| 11 | Low: P6's second ARM may be refused | After `fleet disarm` the manager's state after an operator DISARM was not checked here; a second `fleet arm` in the same launch may be refused. | not read | If it is, relaunch the line (30 s) and count it as the second cycle. |
| 12 | Info: the W4/W5 band's same-tree check | Decision 182 says each band gets one same-tree check on week5 tonight; nothing after R1681 is in the registry yet. If those do not fly, R4/R5 are scored against bands from week4's twin runs, which the rule allows only as "one tree" per row. | registry | Note on the sheet which tree the band came from, so the ledger row is labelled right. |
| 13 | Info: known and accepted | The radio disarms QUAD4 after 0.5 s of silence from his stream and the mux has no silence takeover (card F1, open); P13's gap bars are the only guard. The F6 line (a stuck take-back ends in DISARM) is accepted in writing. R4/R5's release paths are the 8 October ones (`r302_3.log`: `magnet OFF -> /drone_3/magnet`; r303_4/r305: the shell OFF), and the azimuth order `30,90,150,270` relies on `auto_slot_assign: true` (`common.yaml:10`) as it did on 8 October. | | Nothing new. |

## 2. Pack budget (estimates: twin durations R1644/R1645/R1655/R1658 and the 8 October rig flights; lift about 25 s, landing about 15 s)

| flight | carriers (min) | QUAD4 (min) | note |
|---|---|---|---|
| R1 3-min hover | 3.7 | 3.7 | the per-minute voltage curve answers R9's question |
| R2, R3 one lap each | 1.2 + 1.2 | 1.2 + 1.2 | two laps each (finding 6): 1.6 + 1.6 |
| R4, R5 | 1.3 + 1.3 | 0.9 + 0.9 | QUAD4 lands alone about 30 s after the release |
| P13 load test | 10 at idle | 10 at idle | armed, props off: a small draw, but it counts |
| R6 no-catch | 3.0 | 2.8 | his give-up at about 136 s after his takeoff (R1658) |
| R7, R8 | 2.0 + 2.0 | 1.5 + 1.5 | contact 7-11 s after his descent, 20 s hold, land |
| R9 full mission | 3.5 | 3.5 | twin: release at 36 s, reattach at 132 s after our takeoff (R1655) |
| **total** | **about 19** | **about 15** | plus P13 |

If one pack gives about five minutes under the ring, each carrier needs four packs over the day or two charging
cycles; without a count on the sheet the afternoon will be decided by the charger, not the plan.

## 3. What each flight must record (so it goes straight into the thesis)

- **Every flight:** the bag and CSV logs `rig_flight.sh` writes, the `_code.txt`, the phone video from before ARM to
  after the disarm, and pack voltage from the bag's `/drone_i/telemetry` (the manual readings are a backup). The
  thesis table T8 wants per row: tree, before/after the rate fix (after, today), pull source, rod plan, integral mode,
  event kind, layout, largest gap, freed-drone policy, trajectory. Write the tree and the config name on the sheet's
  record line for each flight; the rest follows from the config file.
- **R1:** the voltage curve per drone per minute (from the bag); the hover rock over three minutes gives the rig
  tilt-vector null (sigma_pre at rest) that the resolution rule's minimum detectable effect needs, and the hover
  throttle against voltage at 1.0 m checks the thrust map's voltage term: both at no extra time.
- **R2, R3:** two laps (finding 6); the circle lag and the along-track error come from the planner log; note the
  carriers' plates (even ring) on the record line.
- **R4, R5:** the OFF time from `/drone_3/magnet` in the bag is t_e; the 5 s before it must be a still reference
  (finding 7); hold 15-20 s on three before LAND so time-back-in-band can be scored (plan D5); the freed UAV's
  disarm line and its time; the release and LAND times on the sheet as a backup.
- **R6:** his tip height over plate 9 during his 30 s hold (the F7 measure of his hold error with the plate moving
  under rotor wash), the take-back timeline (`/drone_3/take_back`, mux, guard lines), and plate 9's pose noise with
  QUAD4 overhead (for the proof replay, plan P9).
- **R7, R8:** contact time, proof time, mux switch time, the throttle step, QUAD4's tilt at the proof, ring tilt and dip
  through the admission, clearances: all from the bag with the combined topics; plus finding 5's additions.
- **R9:** everything above, plus the detach-to-release time, his takeoff-to-pickup time, the drop, and whether the
  puck was seen in the net; this is the first full mission on hardware and the thesis's F1 rig insets come from its
  video, so the camera must see the pickup site and the ring together.

## 4. Cheap extra tests (no rig flight time)

1. **09:00, 60 s still bag of every body** (ring on the floor, carriers on their plates, QUAD4 on its mat, puck in
   place): the per-body pose noise and the ring's tilt-vector noise floor; the thesis's measurement-noise sentence
   and the proof replay's noise band come from it.
2. **P7 with the ring tilted 10 deg** (three catches): whether the contact radius depends on the plate's inclination,
   which the circle attach will meet.
3. **P8 at 90 deg** (pure shear) and the "peel" number: with the tip caught, tilt the ring by hand until the magnet
   lets go, and note the angle. The magnet-margin paragraph of the thesis is written around this.
4. **P13 as the R9 prelude** (finding 3): his ARM with QUAD4 caught, a hand lift to 1 m, 60 s.
5. **P13's `--mocap` gap statistics** double as P2's numbers for every body in the graph; P2 then only needs the
   carried-by-hand routes.
6. **P4:** record the tip's z both hanging and under the 2 N pull, so the tether's slack is a number (the release
   guard rises 5 cm above the slack; today that is derived from `attach_cable_len`).

## 5. Walk of the sheet (OK = runs as written; F = see the finding)

| item | status | note |
|---|---|---|
| Setup, `w5`, build, prebuild, `C`/`R`/`D`/`O`/`A` | OK | `fleet_reach.py --send` uppercases the command; preflight flags exist (`tools/preflight.py:112-120`). |
| P1 Motive, body-rate check | F1 | needs T1 with the tip and puck bodies. |
| P2 survey | F1, 4.5 | |
| P3 W, P4 L | F1 | `tip_calibration.py` prints `weld_point_z` and `body 14 to tip 24` once the tip streams. |
| P5 kill path | OK | repeat from T3 and his terminal (card F2). |
| P6 arm check | 11 | |
| P7 attach check and catches | F2 | T1 stopped first (rig_flight refuses otherwise). |
| P8 magnet hold | OK, 4.3 | |
| P9 measures, P10 puck | F1 for P10 | |
| P11 hand ramps | OK | after P3 (the c0 reference is the rest point). |
| P12 tilt checks | 9 | (c) only with Tejen's OK, as written. |
| R1 hover | OK | |
| R2, R3 circles | 6, 10 | |
| R4, R5 detaches | 7, 12 | the configs are the 8 Oct ones plus the departed landing; the release command is the 8 Oct one. |
| P13 load test | 8, 4.4 | run while the packs charge. |
| R6 no-catch | OK | `contact_dz:=0.05` as the card's A8 says; the twin rehearsal R1658 supports. |
| R7, R8 contact attach | OK | the rig default `contact_dz` is -0.01 (`partner_mission.launch.py` RIG), as the card's R11b wants; the proof lift cap is the rig's 0.03. |
| R9 full mission | 3, 5 | the launch line itself passes every refusal (W, L, R typed; `partner_attached` with `weld_radius` > 0; hover). |
| Stop rules | OK | the give-up before LAND, the DISARM line after LANDED, the gap rule and the 23.6 V floor all have their code or their written acceptance. |

# Plan after the 14 Oct rig day (approved by Wesley 14 Oct, ~20:30)

The status table is kept current as work lands: each row says what is done, with commits and run ids. The plan
itself (below the table) only changes when Wesley changes it; dated notes go in the log at the end.

## Status

| # | Work package | Status | Done / evidence |
|---|---|---|---|
| 0 | Tejen's attach logs | done | `~/tejen_attach_logs_2026-10-14.zip` (8 flights, README) |
| 1 | WP0 commit 14 Oct work + FC request; `legacy` branch; the gate runs the attach tests | done | c15abc4, 19804fb, 4209c62, a0d3963; branch `legacy` at a0d3963; gate 1147 tests (6 drop tests had broken unseen on 14 Oct: fixed); docs/rig/fc_request.md ready to send |
| 2 | WP6 clockwise angles (corrected by Wesley 14 Oct: the plates stay clockwise; typed and shown angles become clockwise, angle = 30 x plate) | done | bfbd7a9 (renumbering reverted), 305e50c (same argument names, meaning flipped: profiles.apply and _his_ring convert; 215 configs; shown lines clockwise; manifests carry angles: cw); twin R1720 PASS (plates 3/11/7, weld 270 deg cw) |
| 3 | WP8 arming, landing and small fixes | done (code); USB hub is hardware | 3297746 (tracker start-up crash, full-pack voltage fallback, radio_health, QUAD4 down a few cm above its rest, takeoff hold + climb from liftoff), dade70d (partner first, only refused drones named, DISARM stops ARM, one burst); re-arm after a refused ARM = LAND, not DISARM |
| 4 | WP3 attach: one plate point, Tejen's confirmation rules, bottom hold + quiet-miss retry, measured-tip trim | done (code + twin); critic rounds 1-2 and Tejen's review fixed | eacab58, f035625, 0e47f9b; twin R1718, R1720, R1721 PASS (R1719 FAIL led to rev 2). QUAD4's aim: Wesley's call (A; B = trim at 0.20 m/s, Tejen OK) |
| 5 | WP5 RViz markers + WP2 quiet T2 and console | done; the RViz panel is compile-checked only (RViz cannot render headless): check it props-off | 4a60ca3 (markers, /fleet/mocap_summary); a73d9a4: tools/console.py (READY, who is airborne, the buttons in flight order, SPACE = DISARM), T2 shows the events only (RIG_FLIGHT_VERBOSE=1 for all), launches.csv, quieter sources; the RViz panel commit after it: his row (his ARM, his TAKEOFF, GIVE UP, his DISARM) while his MPC runs, ARM after his ARM, TAKEOFF after "ARM sequence complete", who is in the air; SPACE unchanged |
| 6 | WP4 scoring tool (attach_window) + proof replay tool | done | tools/attach_window.py (per hold phase of his planner: tip error against the plate point, inside K and inside-and-slow, body wander, swing, plate motion, ring tilt, body vs air forcing; reproduces the 14 Oct numbers, e.g. t6g_2 5.6 cm / 7 % / 4 %); tools/proof_replay.py (eacab58). Hover-only configs wait for the wind matrix (after the lab day, revision 1) |
| 7 | Props-off session: FC acceptance, radio soak, magnet hold force, proof bench, load with the console | sheet ready (Thursday tests 1-3) | docs/rig/thu_2026-10-15.md: console + RViz check, the confirmation bench by hand (partner_nodes:=false), FC acceptance if the FCs were changed |
| 8 | Next lab day: card first, then attach flights scored with the tools | ready: card rev 4, sheet, morning report | docs/experiments/2026-10-15_attach_comotion_rig.md, docs/rig/thu_2026-10-15.md, docs/reports/morning_report_2026-10-15.md |
| 9 | After the lab day: WP4 wind matrix (only if contacts stay rare), WP7 rename + legacy, WP9 pickup | to do | |

Thursday 15 Oct 15:00-18:00 tests whatever is done by then; anything unfinished keeps 14 Oct names and numbering
on that day's sheet.

## Context
14 Oct, combined M1 on the rig: 46 launches. 8 passed, 11 failed in flight, 11 never flew because QUAD1/QUAD2
refused to arm, the rest void (arm order, early disarm, a takeoff hop). The best flight (t6h_4) had the magnet
caught on plate 9 (tip within 3 mm of the plate while the plate moved), and our own proof rejected it three times,
so his planner gave up and released. Wesley asked for the best practical course of action, not bound by past
rules, including: RViz showing the swung magnet and puck; a quiet T2 plus a console; the arm refusals fixed; the
word "dissipative" gone (planner folded into mpc_planner, the old controller kept on a `legacy` branch); plates
numbered counter-clockwise (plate 1 = 30 deg; no physical labels); the wind effects countered (tip mass, shorter
tether, lower carrier angle, longer carrier rods all allowed; QUAD4 changes by Wesley; a bigger plate only as a last
resort); the attach confirmed Tejen's way. FC settings must be changed by someone else.

## What the data says (14 Oct bags)
| Finding | Number |
|---|---|
| Tip error over plate 9 (measured plate point, level frame) | median ~5 cm; inside the 2.4 cm catch 7-18 % of the time, inside and slow 0-11 % |
| Error budget | QUAD4 body wander 2-4 cm rms + pendulum swing 2-3 cm rms + plate motion 1.3 cm |
| Swing forcing | QUAD4 body acceleration 0.16-0.50 m/s^2 rms vs air on the tip 0.14-0.23 (his oct14 profile the worst) |
| Swing far from everyone vs over the ring | 1.1-2.1 cm vs 1.8-3.2 cm rms: the others add, QUAD4 itself dominates |
| Steady offsets | tip hangs ~3 cm off QUAD4's vertical; his nominal plate 2 cm off the measured catch centre |
| Ring tilt with QUAD4 over plate 9 | 5-8 deg (max 12), 1-3 deg without |
| QUAD4 hover throttle | 0.49 (model): +50-100 g on the magnet is affordable |
| Arm refusals | 11 launches; QUAD1's own FC attitude read roll -60 to -84 deg while it sat at ~12 deg |
| T2 | ~3000 lines/min; his backend block 41 %, his MPC block 23 %, our radio prints ~20 %; an event list keeps ~100 lines/flight |
| QUAD4 after a take-back | never reached "disarmed alone" (stops 4-7 cm above rest, the 3 cm band can't be met), so DISARM was pressed with it 0.1-0.3 m up |
| Start-up | every node ready ~10 s after the T2 launch (t6h_4); the sheet's 2-min wait is mostly slack, ~20 relaunches/day |

## Order of work
Flight value first, renames last. Each step: unit tests, `tools/gate.sh --quick`, and a headless twin smoke where
flight code changed.
1. WP0 commit 14 Oct work + send the FC request.
2. WP3 attach.
3. WP8 arming and landing fixes.
4. WP2 quiet T2 + console.
5. WP4 wind: scoring tool, hover-only configs, hardware prep.
6. WP5 RViz markers.
7. WP6 plates counter-clockwise.
8. WP7 planner folded into mpc_planner; old controller to `legacy`.
9. WP9 pickup and the full mission, once the attach works.

## WP0 commit + FC request
- The 14 Oct working tree -> logical commits on week5 by explicit path, authored by Wesley: radio + tracker fixes;
  rig load cuts + lean rclpy + rig_pin; drop_on_loss in attach; tracker timing switches + card; floor_join /
  delivery_rejoin + launch args + held-ball world + configs; controller port (mpc_profile); rig overrides (pickup,
  match gate, contact_dy); docs / decisions / registry / session state / Tejen notes.
- `docs/rig/fc_request.md` for whoever configures the FCs (QUAD1-3; QUAD4 with Tejen): run `status` while it shows
  `!ERR` (confirm the ANGLE flag); calibrate the accelerometer level; `set small_angle = 180` (we and Tejen fly rate
  mode through the Betaflight rate curve, so the FC attitude only gates arming); `set gyro_cal_on_first_arm = ON`;
  `save`. Acceptance: 10 min on the plate, then arm, 5 cycles per drone.

## WP3 attach: Tejen's rules for an airborne ring
Our side (`src/drone_magnet/drone_magnet/contact_proof.py`, `magnet_attachment_manager.py`); planner, fleet and mux
keep acting on /magnet/contact and /magnet/object_attached.
- Face point as his: marker + world (0, 0, -0.0225), expressed in the plate frame from the ring pose -> xy and
  normal (same as our weld point at weld_point_z).
- Contact (his M2 gate): magnet ON, xy <= 25 mm, normal -15..+8 mm, relative speed <= 0.03 m/s, held 0.30 s.
  The 14 Oct contacts (xy ~23 mm, normal +3..6 mm, 0.003-0.020 m/s) pass.
- Proof motion: our small vertical lift (his M1 "object follows the lift"); his 15 deg arc would shove an airborne
  ring into the carriers.
- Proven ("the tip moves with the plate"), after the proof lift, for 0.50 s: tether taut (body-to-tip >= this
  flight's own hanging length - 3 mm, measured while holding in MATCH; replaces the typed proof_taut_len), drift
  from the contact seed in the plate frame <= 15 mm, xy <= 40 mm, |normal| <= 15 mm, relative speed <= 0.05 m/s
  (his 0.03 is at the noise floor of our differentiated speed). A taut tether that cannot lift the tip off the
  plate is the co-motion evidence. No plate-rise or body-rise test: the airborne ring bobs (in t6h_4 the plate went
  down 12-21 mm in two of the three real catches) and QUAD4's height wanders; both are logged as diagnostics only.
  In the twin (rigid tether) taut is always true, so the rule reduces to "the tip stays with the plate".
- Replayed on 14 Oct data: all three t6h_4 catches proven at +2.0 s (drift <= 2.8 mm while the plate moved
  24-73 mm, taut 100 %); the no-catch bottoms (t6h5_3, t6f2_4) not proven (drift > 110 mm).
- Failed only on measured separation: xy or |normal| > 45 mm for 0.12 s. No 3 s timeout; the 3-try budget and the
  give-up timer (raised to ~90 s) remain the outer limits.
- His planner (online_join_planner.py, contact_attach branch): ATTACH_READY holds at the bottom with the magnet ON
  until contact, instead of leaving on the first 8 cm / 0.25 m/s excursion; abort only on his loss rule (45 mm for
  0.12 s); his quiet-miss rule (10 s without 5 mm improvement) lifts clear and retries (his 13 Oct QuietMissTracker,
  which accumulates eligible time). Give-up 30 s -> ~90 s on the rig so three tries fit.
- His measured-tip trim for the ring (`target_alignment_mode: measured_tip_trim`, already in our fork, never flown
  by us) instead of the hand-set contact_dx/dy; keep the wider match gate (5 cm / 0.15 m/s).
- Tests: synthetic tracks for t6h_4 (caught: proven), a rim rest with slack tether (not proven), a tip that lifts
  away (failed on separation), stale data (waits). Twin smoke: m1c_floorjoin_hover.

## WP8 arming, landing and small fixes
- Arm refusals (code side): the FCs report AIR (armed, airmode = rate mode), AIR* (disarmed, ready) and !ERR*
  (arming blocked; also transient after boot or a disarm, so its age matters). The console shows each FC's string
  with its age and FC tilt vs mocap tilt, lights ARM only when every drone reads AIR*, and says "replug QUADn" early.
  ARM FAILED names only the drones that failed; the ARM thread stops on a DISARM; refuse at once if QUAD4's FC is
  disarmed in partner mode.
- Re-arm without a relaunch after an ARM that failed before takeoff: a pre-takeoff reset disarms everyone (QUAD4
  included) and, once every FC reads disarmed and every body is on the ground, clears the mux latch without
  restarting nodes. QUAD4 must not re-arm by itself when the latch clears (his MPC keeps streaming armed): the mux
  forwards his stream again only after the next fleet ARM sequence. In-flight aborts keep the latch and the relaunch.
- QUAD4's own landing after a take-back: "down" = below rest + 0.10 m and stalled, so it disarms alone; the console
  says "waiting for QUAD4 to land" until then.
- Tracker start-up crash (ARM while it waits for mocap): state set before the callback manager exists.
- Takeoff race: in WAIT_FOR_TAKEOFF his reference follows the measured pose, so an early lift drifts freely; hold the
  pose latched while QUAD4 sits still on its mat. The console's TAKEOFF publisher lives as long as the console, so
  every subscriber gets the same message (the race came from a fresh `ros2 topic pub` each time).
- Telemetry: no voltage published until a battery frame arrives (not 0.0); the tracker keeps the last good value;
  READY and ARM on the rig need a fresh pack voltage from every radio.
- QUAD3 TX USB drops: the radio node reopens the port; hardware: new cable or a powered hub.
- Registry: rig_flight.sh appends a stub row per launch so none go missing.

## WP2 quiet T2 + console
- T2: `rig_flight.sh` tees everything to the run log (as now) and shows only the ~20 event patterns;
  `RIG_FLIGHT_VERBOSE=1` for the old view. At source: his backend and MPC status blocks to DEBUG (their topics keep
  the text), radio prints off by default, /magnet/contact and ELRS magnet lines on change only, his "Still
  waiting..." throttled, tracker diag off on the rig, DISARM handled once per burst.
- Console: extend `tools/fleet_monitor.py` (PyQt5). Add FC mode + FC tilt, battery with freshness, our phase and his
  phase, contact/proof state, ring tilt, a READY light (all nodes up, poses fresh, telemetry + voltage from every
  radio, every FC ready) that replaces the fixed 2-min wait and the preflight run; buttons for the sequence (his
  ARM, ARM, TAKEOFF, his TAKEOFF, GIVE UP, LAND). Low rate (UI 5 Hz), one process, on this laptop.
- Both surfaces keep the buttons: the console and the RViz arm panel get the same rules (TAKEOFF only after "ARM
  sequence complete", his TAKEOFF reaching every subscriber, GIVE UP). SPACE = DISARM stays instant and unguarded in
  both. Both show who is still airborne so an end-of-flight DISARM is not pressed early.
- The M1 Mac is not needed (after the load cuts the combined stack held 50 Hz with RViz pinned, lt_13).

## WP4 wind and precision
- Tool `tools/attach_window.py <run>`: tip error, inside-and-slow %, body wander, swing, plate motion, ring tilt, air
  vs body forcing.
- Hover-only rig arms (QUAD4 holds over the plate, `auto_descend: false`), 2 x 60 s each: baseline; carriers at
  45 deg (cable_elev_deg / handover_elev_deg); magnet +50 g; +100 g; shorter tether (his cable_length /
  magnet_drop_below_quad + our weld geometry re-measured); QUAD4 over the ring on the floor (no carriers). Longer
  carrier rods only if 45 deg shows that separation matters.
- Adopt what raises inside-and-slow by >= 1.5x without new risk; stack the winners. Bigger plate only if all fail.
- Geometry (rods 0.55 m): nearest carrier body at the moment of contact 0.45 m at 60 deg, 0.58 m at 45 deg, 0.56 m
  with 0.8 m rods at 60 deg; 45 deg costs ~6 % more thrust and ~1.7x the rod's sideways pull per carrier.

## WP5 RViz
`fleet_viz.py`: tip sphere (coloured by coil/contact), tether line body -> tip, puck cylinder, 5-10 Hz, 0.5 s
lifetime; topics /magnet_tip_pose and /pickup/motion_capture_state (twin: /tejen/payload_world_state).

## WP6 clockwise angles (corrected by Wesley, 14 Oct evening)
The plates stay numbered clockwise (carriers 3/11/7, QUAD4 9). Every angle that is typed (launch arguments,
configs) or shown (log lines, the console, RViz, the sheets) becomes clockwise degrees, angle = 30 x plate
(carriers 90/330/210, QUAD4 270); the maths inside (ROS frames, mocap, the optimiser, Tejen's code) stays
counter-clockwise, converted at one boundary. A twin re-fly checks it; Thursday flies it if done and checked, else
the 14 Oct conventions. (The first version of WP6 renumbered the plates instead: wrong, reverted the same evening.)

## WP7 planner into mpc_planner, old controller to `legacy`
- `legacy` branch from the WP0 commit keeps everything for the thesis comparison. Removed from week5: the network
  controller, verify harness, mini plant, the sim_legacy profile, simulation_assets/old_worlds,
  configs/experiments/old, tools/old, docs/archive (each reference on week5 repointed or dropped first).
- On week5: the rig-path planner node -> src/mpc_planner `reconfig_node.py` (ReconfigPlanner(LoadPlanner)); node
  `mpc_planner` in every mode; log prefix `[planner]`; mode dissipative -> `detach`; /drone_i/ELRSCommand_diss ->
  ELRSCommand_ours.
- Old runs keep reading (alias in tools/run_logs.py; the hard-coded readers accept both names).
- The gate runs the planner's rig-path tests (today only 19 network ones, none of the ~170 rig-path).

## WP9 pickup and the full mission (after the attach works)
- Keep the 14 Oct overrides (1.5 s bottom dwell, 4 cm lift gate, 36 mm overtravel), put the low tube back inside the
  puck (<= 0.045 m), take his newer pickup settings (retries 5, relative speed 0.07); his co-motion proof decides.
- If the pickup stays unreliable, delivery_rejoin (start holding the puck; twin first) gives the drop + attach demo.

## Verification
- Per step: the new unit tests, `tools/gate.sh --quick`, then a headless twin smoke (m1c_floorjoin_hover, plus
  m1c_hover for the full mission) on the laptop with the rig stack down.
- Bench before flights: console READY and FC readiness with props off; arm / disarm / re-arm without relaunch.
- Rig (next lab day, in this order): props-off console check; floor_join attach attempts with the new proof
  (target: a fold-in to four); wind hover arms; the full mission once the attach is reliable.

## Log
- 14 Oct ~20:30: Tejen's logs zipped. Plan approved; WP0 started.
- 14 Oct ~20:37: WP0 commits c15abc4, 19804fb, 4209c62, a0d3963. Revision 1 (cross-check), below.
- 14 Oct ~20:42: c1ebdc9 gate runs the attach tests (six drop tests had broken unseen on 14 Oct: fixed); branch legacy.
- 14 Oct ~20:52: WP8 code done (3297746, dade70d); WP6 + WP5 with a helper; WP3 next.
- 14 Oct ~21:04: WP3 code done (eacab58); twin smoke m1c_floorjoin_hover running.
- 14 Oct ~21:27: R1718 PASS (the attach and the new proof in the twin). WP6 + WP5 committed (4a60ca3); the console
  and the quiet T2 next in one commit; decisions.md has tonight's lines (plan, WP3, WP6, WP8; the aim point OPEN).
- 14 Oct ~21:45: a73d9a4 (console, quiet T2); the RViz panel rules built (compile only; RViz needs a display). WP4 next.
- 14 Oct ~22:00: f9d0b72 (RViz panel); WP4 scorer done. Next: the props-off checklist, the attach card + critic, Thursday's sheet.
- 14 Oct ~22:00-23:10: Wesley corrected WP6 (angles, not numbering): bfbd7a9 + 305e50c. Critic rounds 1-2 on the card:
  f035625. Console fixes from a live twin probe: 33ceec4. Tejen's package (~/tejen_review_package_2026-10-15.zip) and
  his review (five code points; trim at 0.20 OK): fixes next.
- 14 Oct ~23:28: Tejen's five points fixed (0e47f9b), card rev 4, R1721 PASS, morning report (496c11f); his
  package rebuilt. Ready for Thursday.

## Revision 1 (14 Oct ~20:50, after a cross-check of the plan against the code and the 14 Oct bags)
Wesley approved the plan above; a second reading against the code changed these points (the sections above stand
where not overridden here):
- **One plate point for both stacks.** The one real catch (t6h_4) sat at (-9.2, 218.5) mm in the ring frame: 11 mm
  from his nominal plate centre (0, 225) and 23.8 mm from our hand-catch point (13, 210), so our 24 mm contact
  radius registered it with 0.3 mm to spare. His target and our weld point become the same configurable point
  (ring_bridge override). Which point (nominal, the t6h_4 catch, or a props-off map) is attach geometry: Wesley's
  word. contact_dx/dy go back to 0; his measured-tip trim corrects QUAD4's own offset.
- **Proof rules, tightened.** Contact: xy <= 35 mm, normal -15..+12 mm (marker vs weld point; no face offset, W
  already is the marker height at contact), speed <= 0.03 m/s, 0.30 s. L_hang frozen at contact from the free hang
  before it. Proven after "lift done" (a Bool from his planner when the LIFT_PROOF profile completes; a timer only
  as fallback; never the body height, which cannot rise when caught) for 1.0 s: drift <= 6 mm, |normal| <= 10 mm,
  speed <= 0.03, body-to-tip >= L_hang - 3 mm, co-motion (plate z range >= 5 mm in the window, or rise >= 3 mm).
  Failed: separated (after lift done, normal > +10 mm or xy > 45 mm for 0.2 s), lost (45 mm for 0.12 s any time),
  inconclusive (3.0 s after lift done), stale (> 0.5 s), aborted (his phase leaves LIFT_PROOF). The 45 mm-only
  failure in the first version could never fire (an uncaught tip lifts only ~20 mm).
- **Bottom hold (his planner).** ATTACH_READY aborts at once only on gross errors (relative speed > 0.35 m/s, tip
  xy > 0.12 m for 0.3 s, face > 20 mm below the plate for 0.12 s); otherwise his quiet-miss retry after 4 s without
  5 mm improvement while xy > 35 mm; 3 misses apart from proof failures; give-up 45 s. In the no-catch flights the
  tip stayed at the bottom under 1 s.
- **The gate runs none of the attach tests** (drone_magnet, the partner-contact tests, his contact-attach and
  floor_join tests): added first.
- **Radios.** All four FTDI radios share one USB hub; QUAD3's port dropped three times (12:22, 14:04, 16:21) and
  three ports dropped together at 09:09. Hardware fix (separate or powered hub) and a 10-min radio soak before any
  props-on flight; in software, close the old serial handle before reopening.
- **Takeoff race, both ways.** In t6h_4 his planner climbed 18 s before his MPC flew. His planner's climb starts
  when QUAD4 is airborne; the console checks his MPC and his planner by node name (the bag recorder alone can
  satisfy `-w N`).
- **Re-arm without a relaunch = LAND before TAKEOFF**, which already disarms cleanly without the abort latch.
  After a flight the trackers exit by design, so one relaunch per flight stays; the READY light saves the wait.
- **FC request.** Drop `gyro_cal_on_first_arm` (it delays the arm inside our confirm window); capture `status` for
  `AIR*` refusals too (no disarm flag there); with small_angle 180, READY needs the mocap tilt under 25 deg.
- **Battery.** Unknown voltage -> assume a full pack (errs low) plus a READY item, instead of a hard ARM refusal.
- **Console.** Fed by low-rate summaries (/fleet/mocap_summary from fleet_viz at 5 Hz, /drone_i/radio_health at
  1 Hz) to stay under ~5 % of a core; long-lived publishers, one message per press. SPACE = DISARM stays instant
  in both surfaces (Wesley); the console also shows who is still airborne.
- **Order.** Plates renumbered first (before new code prints plate numbers); the wind matrix only if contacts stay
  rare after the attach fixes (every attach flight is scored with the same tool); WP7 after the lab day.

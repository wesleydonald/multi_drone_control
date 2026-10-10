# QUAD4 off-nominal endings while Tejen's stream flies it: the tilt cut, the stuck take-back, the partner's own disarm

**Rev 2** (11 Oct, after critic round 1 REWRITE and the rig-port critic's F6). Branch m1-tiltcut (from m1-full 84b1779).
Six switches, each off by default and each its own arm:

| switch (bringup arg unless noted) | what it adds | from |
|---|---|---|
| `partner_tilt_cut` | QUAD4 tilted > 60 deg while his stream flies it: the mux cuts QUAD4 alone, magnet OFF, the fleet LANDs | rev 1, C1, H3, H4, M1, M5 |
| `partner_stuck_guard` | our drone-3 tracker latched while his stream flies QUAD4: no attach is confirmed, announced; at LANDED a mux still on his stream is cut if QUAD4 is at its rest | H1 |
| `warm_fault_not_live` | a fault of our warm drone-3 tracker while his stream flies QUAD4: not live (armed idle), not latched; latches only with authority | F6 (b) |
| `fc_disarm_land` | QUAD4's FC reports disarmed after it flew on his stream: LAND every drone | F6 (c) |
| `taken_back_self_land` (his launch arg) | the mux still on his stream 1 s after TAKEN_BACK: he flies out to 1.2 m from the ring, lands and disarms, magnet as it is | F6 (a) |
| `welded_on_load` | the welded start gets `/drone_3/on_load`, False only once the release is seen, so an off-ring disarm after the detach LANDs | M4 |

Exploratory finding behind it: R1503 (m1_full_hover).
- QUAD4 tipped at his pickup (a rigid-rod artefact: the descent ran 0.11 m low, fixed in 84b1779).
- Our tracker's envelope latched at 90.9 deg, but its disarm was not forwarded.
- The LAND take-back then never switched ("taken back, but our tracker is not flying yet", every 2 s).
- QUAD4 lay at 128 deg, armed at 5 % throttle, until the end of the run, after LANDED too.

## 1. Question
What ends a QUAD4 upset, or a QUAD4 our side can no longer fly, while Tejen's stream flies it?
- Can our side stop QUAD4's motors on its own and land the ring with the three carriers?
- Can a take-back that cannot switch end with QUAD4 down and disarmed instead of hovering until the operator's DISARM?
- And none of it may fire in a normal mission.

### Real life first: the rig failures this is for
The twin's tip-overs (R1503, R1477) are the rigid-rod class. The rig failures this is for are different:
- a snag with a carrier rod in his transit (QUAD4-to-carrier clearance 0.29-0.35 m in older runs, 0.38-0.45 m in the
  claim runs);
- his MPC diverging on a mocap mis-solve while mocap still solves (he self-disarms on a STALE pose, `tejen_mpc/main.py`
  2288: 0.25 s on the rig, 1.0 s in our sim launch, so a lost rigid body is his to stop);
- a motor or ESC failure.

### What stops a tipped QUAD4 on the rig today
- **Our warm drone-3 tracker sees it but cannot act.** Its envelope (60 deg, 3 ticks at 50 Hz) and its pose watchdog
  run while armed (tracker 1545-1561). A latch disarms only our own stream (`_diss`), which the mux does not forward
  while it reads partner.
- **The fleet manager does not escalate it.** Ruling F5 / R0618, fleet manager 314-322.
- **His MPC and supervisor have no tilt cutoff** (checked). His fault latch covers a stale reference and pendulum
  state.
- **His own disarm never reaches us.** His stale-pose self-disarm, a FC that disarms itself: QUAD4 falls and the fleet
  flies on (rig-port F6).
- **Betaflight has no tilt disarm.**
- **A take-back after any latch of our warm tracker cannot switch** (H1). The mux waits for a tracker that will never
  fly again; his TAKEN_BACK ignores LAND, and his MPC has no LAND of its own. QUAD4 hovers on his stream until the
  operator's DISARM, which latches every mux and drops the ring from about 1 m.

**What the cut means physically.**
- A false cut turns a healthy drone into a fall from up to 2.8 m: in his transit it is over or near the ring, with the
  ball.
- 60 deg is not "past recovery" (about 13 m/s^2 of margin leaves more than 1 g of vertical thrust at 60 deg). It is
  **ten times anything his MPC commands** (below).
- The answer to a real 60 deg upset is the carriers' rule (their envelope disarms them at 60 deg), with the softer
  fleet answer of the 8 Oct rule: LAND.

### Evidence, measured (no new flights)
| what | value |
|---|---|
| QUAD4 tilt under his stream, airborne, twin, 22 contact runs R1479-R1502 | max 5.5-6.2 deg (at his takeoff, t ~24.5 s), p99 1.8-4.7 |
| QUAD4 speed under his stream, twin, 24 runs R1479-R1503 (R1503 cut before its tip-over) | per-run max 0.63-1.19 m/s; pooled p99.9 0.70 m/s |
| his reference speed cap | 1.0 m/s per axis (online_join_planner 818-821) |
| body 14 (QUAD4, as a carrier) on the rig, 7-8 Oct, 18 flights, 82,114 poses armed above idle | max tilt 28.1 deg (r200006, the ring capsize); single-pose attitude steps > 30 deg: **0**; the cut never fires; the 30 deg / 0.5 s tier never fires; speed p99.9 0.50-0.81 m/s, max 0.85 m/s |
| body 14, whole bags (disarmed too, rule always on) | fires in 6 bags, every time after the drone's own disarm: r0013 (the capsize fall), r302_3, r303_4, r304_3, r305 (disarmed at z 0.35, fell and toppled to 100+ deg), r2008e60 (a 154 deg one-pose mis-solve on a disarmed drone, frozen there: held off 0.5 s by the glitch guard, then counted); the slow tier also on two disarmed drones resting tilted (r0012, r302) |
| envelope faults in 70 rig logs, 30 Sep-8 Oct | 3, all ring tilt (r0013 x2, r200006); none a drone tilt |
| Tejen's QUAD4 logs at transit height | none in this repo (W4 asks him) |

- **What the twin cannot show:** its mocap is perfect, so F1/F2 cannot measure a rig false trip. The rig replay above is
  that evidence (`tools/tilt_replay.py`).
- **What a fault does on the floor:** every whole-bag firing is a drone already disarmed. That is why the consumers act
  only while his stream flies QUAD4 (below): a QUAD4 disarmed alone at its rest that topples must not land the fleet
  (4 of 18 rig flights did topple after their disarm).

## 2. Hypothesis
- **`partner_tilt_cut`:** with QUAD4 past 60 deg for 3 poses while his stream flies it:
  - our drone-3 tracker publishes `/drone_3/tilt_fault`;
  - the mux forwards disarmed idle to QUAD4 alone within 0.1 s of the fault (sim), with its magnet OFF in the same
    frame;
  - the fleet manager answers LAND, never the emergency stop;
  - the planner takes QUAD4 out of the flight: no take-back, no weld, no references, overriding an earlier take-back;
  - the three carriers land the ring, and every drone ends disarmed.
  In normal flight nothing fires (QUAD4 under 6.2 deg in 22 of 22 twin runs; body 14 under 28.1 deg with no glitch
  steps on the rig).
- **`partner_stuck_guard`:** a latch of our warm tracker while his stream flies QUAD4 is announced. No attach is
  confirmed after it. At LANDED, a mux still on his stream is cut when QUAD4 is at its rest; otherwise the operator is
  told to press DISARM.
- **`warm_fault_not_live`:** the same latch never happens. A fault of the warm tracker makes it not live for 0.5 s and
  nothing more, so the mission completes and the hand-over still works.
- **`taken_back_self_land`:** a take-back that cannot switch ends with QUAD4 out at 1.2 m from the ring, down and
  disarmed by his own planner, magnet as it is.
- **`fc_disarm_land`:** his own disarm, or the FC's, after QUAD4 flew on his stream LANDs every drone.
- **`welded_on_load`:** after the welded start's release is seen, a disarm of QUAD4 while ours steps it out LANDs
  instead of the emergency stop.

Most likely failures:
- **Message races** (H3): the LAND or his give-up reaching the planner before the fault; the tracker's own disarm
  reaching the fleet manager before or after the mux's state change. Every order is unit-tested.
- **The magnet OFF losing to contact** in his ring descent (H4). In the twin the catch needs the manager's ON, which the
  cut refuses; on the rig the coil latch is turned OFF by the mux in the frame of the cut.
- **A falling QUAD4 hitting the ring** (T2, T3, T9). That is the fall's physics, not the rule, and is reported without a
  bar.

## 3. The variables (one per arm)

**`partner_tilt_cut`**
- *Tracker* (drone 3 only, `tilt_fault_topic` `/drone_3/tilt_fault`, volatile and reliable). Its own subscription
  evaluates the rule on every fresh pose, ahead of the pose-timeout and armed gates and of the envelope latch (C1).
  - Its own counter and its own flight latch: set at TAKEOFF, cleared at the next ARM.
  - Rule (`utility_objects/tilt_cut.py`, pure): tilt > `tilt_cut_deg` 60 for 3 consecutive poses.
  - Glitch guard (M1): a single-pose attitude step > 45 deg is not counted, and nothing counts for 0.5 s after it.
  - Optional slow tier `tilt_slow_deg` (M3; default 0 = off): 30 deg held 0.5 s.
  - Speed is logged only, "would cut (speed)" (M2).
  - The fault is published once; the ring's tilt and our reference age are not part of it.
- *Mux* (`tilt_fault_topic`): on the fault while it reads partner, state `cut`.
  - Ranked with the abort latch in `_forward` and `_publish_state` (H3).
  - Disarmed idle from every input, re-sent at 20 Hz on a steady timer.
  - The magnet channel forced OFF in every frame, and on the rig `OFF` on QUAD4's radio latch `/drone_3/magnet` in the
    same callback, re-sent at 2 Hz (H4).
  - A take-back, a weld or a release after it changes nothing. While ours flies QUAD4 it ignores the fault.
  - Volatile input, so a relaunched mux is not cut by an old fault (M5).
- *Fleet manager* (`tilt_fault_land`): on the fault while QUAD4's mux reads `partner` or `cut`, LAND once (the runner's
  FAULT_LAND), never the emergency stop.
  - A drone-3 disarm while its mux reads `cut` is not escalated (R0618 kept).
  - ARM is refused while a mux reads `cut`, until relaunch.
  - While ours flies QUAD4, or after it was disarmed alone, the fault changes nothing: its own disarm decides, as today.
- *Magnet manager* (`cut_state_topic`): on the mux's `cut`, magnet OFF even after a confirmed attach. This overrides
  the arbiter's refusal (H4: a dead QUAD4 must not hang on plate 9), and every later ON is refused.
- *Planner* (`partner_tilt_cut`): on the fault while QUAD4 is off the load (pending, or detached) and the mux reads
  `partner` or `cut`, it is out of the flight (`_newcomer_out` 'fault').
  - No take-back, no weld, no span, no references.
  - It overrides an earlier take-back and clears the departed state, as `_newcomer_arming_cb` does (H3).
  - A release not yet made is cancelled; one already made stops being announced.
  - One ring magnet OFF: a cut drone falls with or without the coil. The "OFF only after the span" rule was for a
    live take-back with the ball.

**`partner_stuck_guard`** (H1)
- *Planner*: our drone-3 tracker reports disarmed after TAKEOFF while the mux reads `partner` (and QUAD4 was not cut).
  It publishes the latched `/drone_3/handover_blocked` and announces "no take-back or hand-over is possible; at LANDED
  its mux is cut if it is at its rest, otherwise press DISARM".
- *Magnet manager* (`handover_block_topic`): while blocked, a proof that passes is turned into a failed one, and the
  twin joint is released.
- *Fleet manager* (`landed_partner_cut`): at LANDED, for a drone whose mux still reads `partner`:
  - at its rest (z <= the height recorded at ARM + 0.05): `/drone_3/mux_cut`, so the mux cuts it;
  - otherwise: "press DISARM once it is down".
- *Mux* (`cut_topic`): the same cut.

**`warm_fault_not_live`** (F6 b): our drone-3 tracker with `warm_follow_applied`, while not in authority.
- An envelope fault or a pose timeout publishes armed idle on our pre-mux stream. The mux's live-takeover rule then
  never hands QUAD4 to it.
- Not latched, no disarm, held 0.5 s after the last fault, one log line per episode.
- With authority, faults latch as before.

**`fc_disarm_land`** (F6 c)
- *Fleet manager*: a drone whose mux reads `partner`, whose FC reported `armed` during this flight and now reports
  `disarmed` or `failed` → LAND once.
- *Rig*: `/drone_3/fc_arm_state` from elrs_interface (ArmSwitch).
- *Twin*: bf_comm's stand-in (`publish_fc_arm_state`, the forwarded command's armed flag; drone 3 only).

**`taken_back_self_land`** (F6 a; his planner, contact_attach only, logged in CHANGES_FROM_TEJEN.md)
- In TAKEN_BACK, if `/drone_3/mux_state` does not read ours, latched or cut 1 s after entering it, a two-leg reference:
  1. radially out from the ring centre to 1.2 m at its height;
  2. down to `taken_back_land_z` 0.20.
- Then his landing disarm command. No magnet command.
- His own LANDING phase is not used: it sends OFF (F9), and on the welded start it would disarm at the air release
  height.

**`welded_on_load`** (M4)
- The welded start gets the reserved drone's latched `/drone_3/on_load`: True once the welded fleet leaves the floor,
  and at the rejoin.
- False only once the release is seen: the magnet tip 0.05 m off its plate in the ring frame. On the rig the slack
  tether lets the body move while the coil still holds, so the body moving proves nothing.
- With it, `newcomer_fault_land`'s existing rule (an off-ring disarm lands the fleet) covers the welded start after its
  detach.

**Harness only** (no flight code; criteria only in the new `tc_*` configs, NOT m1c_* or m1_full_*, whose claim is
being re-flown frozen):
- `max_armed_tilt_deg`: any drone armed above it for 3 run.csv rows, with a MOCAP_TILT's own window (+0.5 s) excluded
  for that drone, because run.csv shows the fake attitude there.
- `disarmed_after_landed_s` (2 in the tc_* configs).
- Offline replay on R1470-R1503:
  - the tilt criterion flags R1502 (already a fleet abort) and R1503 only;
  - the after-LANDED criterion flags R1503 only, among the 38 10-Oct runs with a LANDED.

**Test tooling** (sim only):
- runner `MOCAP_TILT <id> <deg> [s [ramp_s]]`: the sim mocap emulator rotates that drone's published attitude about
  body x, then holds it s. Our trackers and his MPC see it.
  - In one step it is a mis-solve, which the glitch guard holds off 0.5 s.
  - Over `ramp_s` it is a body tipping, which the rule counts.
  - The fault arms ramp it: 90 deg over 0.3 s, 2.5 deg per pose at 120 Hz, crossing 60 deg at 0.2 s.
- WRENCH on `x3_drone3::base_link`: the subtest's pickup push world, and the new generated
  `three_rigid_ground_rig_attach_welded_pickup_push.sdf`.
- `MOCAP_LOSS` (exists).
- runner events TILT_FAULT and MUX:cut.
- `tools/tilt_replay.py` (the rig replay).
- elrs_interface publishes the CRSF attitude it used to print only (`fc_attitude`, [roll, pitch, yaw] rad, M1).

## 4. Baseline
- **T0:** the same tree, every switch off, the same injection. Judged on all five:
  - no cut;
  - no LAND from the manager;
  - QUAD4 armed;
  - the LAND take-back stuck ("taken back, but our tracker is not flying yet");
  - QUAD4 armed after LANDED.
  Note that the MOCAP_TILT also feeds his MPC, so this is a physical upset, not only a detection test. If T0 cuts or
  lands by itself, the card is void.
- **Nominal behaviour:** m1c_hover R1491 (e097a94); m1_full_hover's re-check on 84b1779 (R1504/R1505).
- **Welded start, a disarm while ours steps out (today):** the fleet emergency stop (T5a).

## 5. Pass / fail numbers (per run; written 11 Oct before any run; frozen before the first)
Read from events.csv, launch.log and run.csv.
- "Cut" = `d3_armed` 0 on the mux output.
- After a MOCAP_TILT, QUAD4's attitude columns are fake for its window. Its true tilt is read after the window (or from
  the tip: tip z + 0.59).

| Arm (config) | Switch | Events | Confirms | Falsifies |
|---|---|---|---|---|
| T0 `tc_t0_off_transit` | none | MOCAP_TILT 3 90 2 0.3 (tipping over 0.3 s, held 2 s), 2 s into TRANSIT_TO_DROP_POINT (QUAD4 1.4 m from the nearest carrier in xy, R1491); LAND +20 s | the five baseline items above | a cut or a manager LAND: void |
| T1 `tc_t1_transit` | tilt_cut | as T0 | TILT_FAULT <= 0.3 s after the injection (60 deg at 0.2 s, then 3 poses); MUX:cut and d3_armed 0 <= 0.1 s after TILT_FAULT; FAULT_LAND <= 0.5 s after it; no FLEET_ABORT; carriers armed until LANDED; LANDED; criteria pass; no TAKE_BACK; one ring magnet OFF (no ON after it) | FLEET_ABORT; QUAD4 armed > 0.5 s after the injection; no LAND within 2 s; a carrier disarmed before LANDED; a take-back |
| T2 `tc_t2_descend` | tilt_cut | as T0, 3 s into DESCEND_TO_ATTACHMENT (span begun, tip cm above plate 9) | as T1, and the magnet OFF on the mux output <= 0.1 s after the cut (rig latch path unit-tested); ring tilt peak and dip reported, no bar | as T1; an abort through drone 3's path |
| T3 `tc_t3_proof` | tilt_cut | as T0, 0.3 s into LIFT_PROOF (caught, not confirmed) | as T2; the twin joint released; no ATTACH ever | an ATTACH; as T2. Passes for a different physical reason than the rig (the twin rod has no collision) |
| T4 `tc_t4_full_pickup` | tilt_cut | as T0, full M1, 3 s into DESCEND_TO_PICKUP | as T1 | as T1 |
| T5a `tc_t5a_full_stepout` | tilt_cut | as T0, full M1, 2 s after DETACH (ours steps QUAD4 out) | today's answer: our envelope disarms QUAD4 through ours; FLEET_ABORT; the mux never cut | a manager LAND (the cut must not act while ours flies it) |
| T5b `tc_t5b_full_stepout_onload` | tilt_cut + welded_on_load (one change from T5a) | as T5a | release seen (log) before the fault; FAULT_LAND; no FLEET_ABORT; no /partner/release after the fault | FLEET_ABORT; the release not seen before the fault: void (inject later) |
| T5c `tc_t5c_full_loss_onload` | welded_on_load only | MOCAP_LOSS 3, 2 s after DETACH | its pose-timeout disarm -> FAULT_LAND; no FLEET_ABORT | FLEET_ABORT |
| T6 `tc_t6_push` | tilt_cut | pickup push world, WRENCH 0 10 0 on QUAD4 for 0.5 s, 2 s into his transit | QUAD4's true tilt > 60 deg; TILT_FAULT present AFTER the envelope's speed latch (R1263/R1287: speed first at 0.25 s); then as T1 | as T1; no TILT_FAULT after the speed latch; QUAD4 never past 60 deg: void |
| T7 `tc_t7_debounce` | tilt_cut | MOCAP_TILT 3 90 0.02 in one step (a mis-solve), in his transit | no TILT_FAULT, no cut; the mission goes on to his attach | a cut |
| T7b `tc_t7b_debounce_040` | tilt_cut | the same, 0.04 s | information only | - |
| T8 `tc_t8_carrier` | tilt_cut | MOCAP_TILT 0 90 1 (a carrier) | FLEET_ABORT within 1 s (the carrier lock unchanged) | a LAND instead |
| T9 `tc_t9_net_drop` | tilt_cut | as T0, 20 s into DESCEND_TO_DROP_HEIGHT (QUAD4 ~2.2 m above the ring centre, the ball on) | as T1 for the cut and the LAND; the ring dip, tilt and the carriers' throttle reported, no bar | an abort through drone 3's path |
| H1 `tc_h1_stuck` | stuck_guard | MOCAP_TILT 3 65 0.1 in one step, in his transit (our warm tracker's envelope latches, 3 ticks; his MPC sees 0.1 s of it); WAIT_WELD 150; LAND | HAND-OVER BLOCKED logged once; no ATTACH; his proofs fail, give-up, TAKEN_BACK; at LANDED either MUX:cut with QUAD4 at rest, or the DISARM line with QUAD4 up | an ATTACH; "HOLDING approach authority" after the block; a cut of a QUAD4 not at rest |
| F6a `tc_f6a_self_land` | his taken_back_self_land | the same latch; LAND 3 s later in his transit | TAKE_BACK, the mux stays partner; 1 s later his fly-out to r >= 1.15 m, then down; his DISARM; QUAD4 disarmed and at its rest before the run ends (QUAD4's own landing with the ball on the rod: tip-over reported) | QUAD4 hovering at the end; a magnet command from him |
| F6b `tc_f6b_not_live` | warm_fault_not_live | the same latch | no drone-3 disarm; "not live" logged once; the mission completes: ATTACH, MUX:ours, LAND, all disarmed (the m1c bars) | a drone-3 disarm; a stuck hand-over |
| F6c `tc_f6c_fc_disarm` | fc_disarm_land | MOCAP_LOSS 3 for 2 s in his transit (his MPC self-disarms at 1.0 s) | his stream disarmed -> bf_comm 'disarmed [sim]' -> a manager LAND <= 0.5 s; no FLEET_ABORT | no LAND; FLEET_ABORT |
| F1 `tc_f1_hover` | tilt_cut | nominal m1c_hover | zero TILT_FAULT, zero cut; the m1c claim bars as R1491 | any TILT_FAULT |
| F2 `tc_f2_full` | tilt_cut | nominal m1_full_hover | zero TILT_FAULT, zero cut; as its re-check | any TILT_FAULT |

- **T9's physics, written first.** QUAD4 (0.55 kg) and the ball (0.1 kg, still welded in the twin) fall.
  - The ball first reaches the net (~1.2 m, about 3.5 m/s); QUAD4's body lands on the net or annulus about 0.4 s later.
  - That is about 2.3-2.9 N s down near the ring centre: a dip of order 0.05-0.15 m and a tilt under 10 deg if
    centred, more if the body lands on the rim.
  - The ring then carries about 0.65 kg of dead weight: the carriers' hover throttle rises ~26 %, about 0.53 to 0.67,
    under the 0.8 cap.
  - Then the LAND.
  - Expected: no carrier envelope fault. A carrier ring-tilt abort is physics and is reported; an abort through drone
    3's path falsifies.
- **What T7 does and does not show.** A one-step injection is held off by the glitch guard before the 3-pose count
  is reached, so T7 shows the two together on a mis-solve. The count alone, on a body tipping just past 60 deg for
  fewer than 3 poses, is unit-tested only.
- **Unit-tested only** (all pass):
  - the rule: debounce, glitch hold-off, slow tier, flight latch past a disarm, ARM clears;
  - the emulator's injection at 120 Hz: the ramp counted 3 poses after 60 deg, the step held off 0.5 s;
  - C1 (a) the same-tick crossing and (b) speed first, then tilt;
  - the mux: cut from partner only, ranked with the abort latch, magnet OFF in the frame and on the latch, take-back,
    weld and release after the cut, volatile inputs (relaunch);
  - the magnet manager: a cut overrides a confirmed attach and refuses ON; the hand-over block fails a proof;
  - the fleet manager: LAND once, ours ignored, disarm in 'cut' not escalated, ARM refused while cut, LANDED cut only at
    rest, the FC disarm LAND;
  - the planner, every order: LAND then fault, give-up then fault, fault then LAND, weld after the cut, on the ring,
    ours flying, a release cancelled, the block, the welded on_load;
  - his self-land;
  - the warm not-live tracker;
  - the runner criteria;
  - each switch's launch graph (off: nothing new).

## 6. Repeats and order
1. The unit tests and the offline replays (done: section 1).
2. T0, T1, T7, T8 first: the cut, the debounce, the carrier lock.
3. Then H1, F6b, F6a, F6c: the stuck take-back family. F6b is the candidate to make the others rarely needed.
4. Then T2, T3, T9 (the ring hit), T4, T5a/b/c (the full M1), T6.
5. F1 and F2 last.
6. Repeats: T1, T4 and F6b x2; the rest x1. Between tweaks, one run on the arm that failed; a fix not shown in two
   runs: stop and report.
7. The reviewer on the table. Wesley decides each switch's default.
   - A default-on `partner_tilt_cut` / `fc_disarm_land` adds an exception to the locked "fleet-wide disarm on any
     envelope fault" line, and gives our stack a new authority: it disarms Tejen's drone during his phase.
   - Tejen is told before Wednesday (W4).

## 7. Cost
- 19 configs, 22 runs.
- The lab PC, after the claim batch, ~15-20 min each (the fault arms end at LANDED).

## 8. New code?
Yes. Every switch above changes fault escalation or who flies QUAD4, so the critic reads this rev before any run. All
behind the six switches, default off; with them off, every graph and every node's behaviour is that of 84b1779 (launch
graph tests). Files:
- `utility_objects/tilt_cut.py`, `tracker_node.py`, `elrs_mux.py`, `magnet_attachment_manager.py`,
  `fleet_manager_node.py`, `dissipative_node.py`;
- his `online_join_planner.py` (TAKEN_BACK only; merges beside m1-final's LIFT_PROOF and pickup-retry edits) and
  `partner_mission.launch.py`;
- `payload_mocap_emulator.py` (MOCAP_TILT, a step or a ramp), `payload_betaflight_comm.py` (the FC stand-in), `elrs_interface.py`
  (`fc_attitude`);
- the runner (`MOCAP_TILT`, the two criteria, TILT_FAULT), the generator (the full-M1 push world), `tools/tilt_replay.py`;
- 19 `tc_*` configs.

If it fails, everything behind the failed switch is deleted, and the sheet keeps the operator's DISARM.

## Wednesday (rig, props-off; draft for Wesley, nothing on the sheet yet)
- **W1:** the switches live in common.yaml's partner section, so the real profile has them, default off. Type them on
  the rig combined-M1 launch line and read them back (`ros2 param get` on tracker_3, elrs_mux_3, fleet_manager,
  magnet_attachment_manager, dissipative_planner).
  - Props-off desk test: tip QUAD4 (body 14) by hand past 60 deg while the mux reads partner, his stream arming QUAD4
    (or fake_mocap, ramped: a one-step tilt is held off 0.5 s by the glitch guard).
    Expect fc_arm_state disarmed within 0.2 s of TILT_FAULT; a take_back or a weld must not undo it; ARM refused after.
  - Confirm drone 3's rig tracker is armed with takeoff_requested while his stream flies QUAD4 (require_fc_armed on
    the rig). If not, the rule never runs: its flight latch needs our TAKEOFF.
- **W2:** a props-off coil check under the cut. Does the coil follow ON/OFF through the radio latch while the mux
  forwards his stream and after the cut? Does the FC's disarm by itself release the ball? (Betaflight keeps aux outputs
  through a disarm unless a mode says otherwise.)
- **W3**, sheet lines (draft only):
  - "after LANDED, if drone 4's mux reads partner (T2: 'taken back, but our tracker is not flying yet'), clear the
    cage, press DISARM before anyone walks in (it latches every mux; QUAD4 falls)";
  - "a cut in his descent, LIFT_PROOF or the drop hits the ring; drop_on_loss is off in attach mode: DISARM if the ring
    tilts badly";
  - "his MPC's pose_timeout_s is 0.25 on the rig, not the sim's 1.0".
- **W4**, a note to Tejen (draft): "On our side, when QUAD4 tilts past 60 deg while your stream flies it, our mux will
  disarm QUAD4 alone and land the ring (switch partner_tilt_cut, off until Wesley decides). Would you add the same rule
  inside your MPC, next to your stale-pose self-disarm? Do you have QUAD4 logs at transit height (2.3-2.9 m) we could
  replay the rule on? And a TAKEN_BACK self-landing is proposed in your planner behind a switch (taken_back_self_land):
  out to 1.2 m from the ring, down, disarm, magnet left as it is."

## Critic

### Round 1 (10 Oct night, on rev 1). Verdict: rewrite the card.

#### The five questions
1. **Has it been tried?** Every piece has, never in this combination:
   - the mux's "disarmed idle from every input at 20 Hz" is the abort latch (card 2026-09-28_mux_abort_latch; R0751, R0752, T0034);
   - LAND on an unwelded-QUAD4 fault is newcomer_fault_land (card 2026-10-09_newcomer_fault_scope). Its C2 arm was the same 10 N push as T6 (R1263/R1287), and there the speed rule tripped first, at 0.25 s;
   - not escalating a drone the partner flies is ruling F5 / R0618, and the card keeps it.
   New: our stack disarming a drone while Tejen's software flies it, and the welded-start on_load the code deliberately deferred (dissipative_node.py:412-414).
2. **Simplest explanation not addressed.** R1503's tip-over is the rigid twin rod driven into the floor by a pickup descent that ran about 0.11 m low. That class was ruled a twin artefact (9 Oct R1315; 10 Oct landing unwind), because the rig tether goes slack. R1503's armed-after-LANDED end state is a separate problem: the LAND take-back is stuck behind drone 3's latched tracker (launch.log:6207 on). That happens after ANY drone-3 envelope latch while the mux reads partner.
3. **Falsifiable by the bars?** It is falsified by a real upset that is not cut, or by a cut in normal flight. The bars test the twin mechanism well, but T6 (the only physical upset) can be missed by the design as written (C1). No arm can show the rig false-trip rate: twin mocap is perfect.
4. **What it breaks.**
   - It adds an exception to the locked fleet-wide disarm, and a new authority: our stack disarms Tejen's drone during his phase.
   - It adds new state in the mux, the last hop to QUAD4's radio.
   - The bundled welded-start on_load changes the answer to every QUAD4 disarm after the detach.
   - Criteria in m1c_* would move the frozen claim's bars.
5. **Asked for?** Not as such: "if quad 4 fails we just land everything" (8 Oct), "LAND lands every drone" (9 Oct), "partner-scoped faults do not abort" (28 Sep). It needs Wesley's word; tell Tejen before Wednesday.

#### CRITICAL
**C1. As written the rule usually cannot fire.**
- The tracker's control body runs only under `if self.armed and self.current_pose is not None`, and returns as soon as `_safety_check()` faults (tracker_node.py:1556-1561).
- `cb.disarm` sets armed and takeoff_requested False (callback_manager_multi.py:230-232).
- In a pure tilt, the envelope and the rule cross on the same tick, so the rule placed after `_safety_check` never runs. In a knock, speed latches first (R1263/R1287, 0.25 s) and the window has closed before the tilt passes 60 deg (T6).
- Change:
  - evaluate on every fresh pose, before the pose-timeout and armed gates, with its own counter;
  - give it its own flight latch: set at TAKEOFF, cleared at ARM;
  - unit tests: (a) same-tick crossing publishes TILT_FAULT; (b) speed latches first, tilt later, TILT_FAULT is still published;
  - T6 bar: TILT_FAULT present after an ENVELOPE (speed) latch.

#### HIGH
**H1. R1503's end state is a stuck take-back, not tilt.**
- From launch.log:6207 the mux logs every 2 s: "taken back, but our tracker is not flying yet: the approach stream stays on". Its live-takeover rule waits for a flying command from a tracker that has latched and will never fly again.
- Any drone-3 latch while partner (tilt, speed, stale reference, ring tilt) therefore leaves QUAD4 armed on his stream after LANDED. It also blocks the hand-over after a confirmed attach ("HOLDING approach authority").
- Change:
  - (a) a rig sheet line now: after LANDED, if drone 4's mux reads partner, press DISARM (it latches every mux) before anyone walks in;
  - (b) a drone-3 latch while partner refuses the attach confirmation and is announced "no take-back possible"; at LANDED the manager latches any mux still reading partner if that drone is at rest height;
  - (c) T0's expected column adds "take-back stuck at LAND" and "QUAD4 armed after LANDED".

**H2. Restate the real-life case.**
- The motivating evidence (R1503, R1477's floor tip-over) is the rigid-rod artefact class.
- The rig failures this is for:
  - a snag with a carrier rod in his transit (clearance 0.29-0.35 m in older runs, 0.38-0.45 m in the claim runs);
  - his MPC diverging on a mocap mis-solve;
  - a motor or ESC failure.
- His MPC already self-disarms on a stale pose (tejen_mpc/main.py:2288; 0.25 s default, 1.0 s in partner_mission.launch.py:292 for the sim), so a tumble that loses the rigid body is stopped by him. The niche is a tip-over while mocap still solves.
- Strike "cannot recover": with about 13 m/s^2 margin, 60 deg leaves more than 1 g of vertical thrust. Say "ten times anything his MPC commands". A false cut turns a healthy drone into a fall from up to 2.8 m.

**H3. Message-order races.**
- The manager's LAND or his give-up can reach the planner before tilt_fault. `_take_back` (dissipative_node.py:1633-1687) then sets `_newcomer_out[j]='LAND'`, `_departed_hold` and `_land_alone`, and publishes the latched take_back.
- If the fault path then returns because j is already out (as `_abort_newcomer` at :1748 does), the landed gate waits for a fallen drone.
- `_publish_state` ranks latched > ours > partner. If "cut" does not outrank ours, a late take-back or weld undoes it.
- Change:
  - the fault path overrides an earlier take-back and clears the departed state (as `_newcomer_arming_cb` :1849-1853);
  - "cut" is handled exactly like `_abort_latched` in `_forward` and `_publish_state`;
  - unit tests: LAND then fault, fault then LAND, give-up then fault, take_back and weld after the cut.

**H4. Magnet OFF timing.**
- On the rig the coil catches on contact. In DESCEND_TO_ATTACHMENT the tip is centimetres above plate 9.
- The OFF travels tracker -> tilt_fault -> planner -> manager -> mux merge, so a falling QUAD4 can catch the plate first (C1b: 57 deg ring tilt, R1262/R1286).
- After confirmation the manager refuses OFF, so a cut leaves a dead QUAD4 on plate 9. Plate 9 is outside the 270/30/150 triangle (near edge y 0.11 m, plate 0.225 m), which shifts the CoM about 0.09 m toward that edge.
- Change:
  - the mux forces the magnet channel OFF in the same frame as the cut;
  - a cut overrides the post-confirmation OFF refusal;
  - T2/T3 bar: OFF on the mux output no later than 0.1 s after the cut.

#### MEDIUM
**M1. Rig mocap false trips.**
- Sign invariance covers only a quaternion flip. A Motive mis-solve or marker swap is a different attitude and can last more than 60 ms.
- QUAD4 flies at 1.8-2.8 m near the cameras, with body 24 below it, within about 0.4 m of the carriers. The rig evidence is from carriers below about 1.5 m.
- The ring drop rule has a glitch-step guard (tracker_node.py:775); this rule has none.
- Change:
  - count only ticks reached without a single-tick attitude step above about 45 deg;
  - replay offline on body 14's rig mocap (7-8 Oct four-drone flights) and Tejen's QUAD4 logs at transit height; report the max tilt and the count of single-tick steps above 30 deg;
  - publish and log the CRSF attitude elrs_interface decodes but only prints (elrs_interface.py:266-270).

**M2. Speed rule.**
- Keep it out of the live cut for Wednesday. His references are capped at 1.0 m/s per axis (online_join_planner.py:818-821), so "2 m/s transit" has no source.
- Measure QUAD4's peak speed under his stream in R1479-R1502 and body 14's p99.9 on the rig; log "would cut (speed)" on Wednesday.

**M3. Slow tip-over (optional).** R1503 passed 30 deg at 121.6 s and 60 deg at 127.2 s. A tier of "above 30 deg held 0.5 s" is still five times his 6.2 deg maximum.

**M4. A second variable is bundled in (welded-start on_load).**
- It changes the response to every QUAD4 disarm after the detach. `_newcomer_arming_cb` is subscribed only with `_newcomer_fault_land` on.
- On the rig, "on_load False at the detach" comes before the physical release.
- Change: its own switch and registry rows; on_load False only once the release is seen; T5b MOCAP_LOSS after the detach.

**M5. A latched tilt_fault topic deadlocks a relaunch.**
- A transient-local True re-cuts a relaunched mux, and ARM (which clears the tracker's flag) is refused while cut.
- Change: volatile reliable (the mux re-sends on its own timer), or ignore a True older than the mux's start. Unit-test the relaunch.

#### Test plan
- T0: MOCAP_TILT also feeds his MPC (a physical upset). Judge T0 on no cut, no LAND, QUAD4 armed, take-back stuck, armed after LANDED; not on "the mission goes on".
- `max_drone_tilt_deg` reads run.csv attitude, which is fake during MOCAP_TILT. Use Gazebo's true attitude, or exclude the injection window.
- Criteria in new configs only, not m1c_* / m1_full_*.
- T2/T3: QUAD4 falls on the ring. Report ring tilt without a bar; the falsifier is an abort through drone 3's path.
- Missing arm: the drop over the net (QUAD4 about 2 m above the ring centre). Write the expected physics first.
- T7: fly 0.02 s as the falsifier; 0.04 s is information only.
- T1: inject where QUAD4 is not above the carriers.
- State that F1/F2 cannot measure rig false trips (M1's replay is that evidence).

#### Wednesday rig subtest
- W1:
  - the switch in real.yaml (partner section) and the rig combined-M1 launch, with the read-back;
  - props-off desk test with fake_mocap: tilt body 14 while partner with his stream arming QUAD4. Expect fc_arm_state disarmed within 0.2 s; a take_back or weld must not undo it; ARM refused;
  - confirm that drone 3's rig tracker is armed with takeoff_requested while partner (require_fc_armed on the rig); if not, the rule never runs.
- W2: props-off coil check under the cut. Does the coil follow ON/OFF through the mux merge? Does the disarm by itself release the ball?
- W3, sheet lines:
  - DISARM after LANDED (H1a);
  - a cut in the descent, LIFT_PROOF or the drop hits the ring; drop_on_loss is off in attach mode, so DISARM if the ring tilts badly;
  - his pose_timeout_s 0.25 on the rig, not 1.0.
- W4: tell Tejen before Wednesday; offer the same rule inside his MPC next to his pose self-disarm (logged in CHANGES_FROM_TEJEN.md).

#### LAND by phase
| phase | as built | verdict |
|---|---|---|
| welded start after the detach | envelope disarm + LAND via the bundled on_load | right, but a separate variable (M4); on_load False after the release |
| his transit with the ball | cut + LAND; the ball falls with QUAD4 | right; rig coil check (W2) |
| drop over the net | untested | LAND right; the ring is hit, possible carrier ring-tilt abort; add the arm |
| ring descent (span begun) | cut, then a relayed OFF | right only if the OFF beats contact (H4) |
| LIFT_PROOF | cut + OFF + LAND | right; the twin rod has no collision, so T3 passes for the wrong physical reason |
| after the proof, before the switch | OFF refused: a dead QUAD4 on plate 9 | wrong as built (H4); never ends if our tracker latched (H1) |

### The rig-port critic's F6 (10 Oct night, on m1-rig dd33dca), folded into rev 2

F6 Medium-high: warm drone-3 tracker runs the pose watchdog and envelope while not flying (tracker 1545-1561): a fault
  latches, the mux holds his stream; his TAKEN_BACK ignores LAND, his MPC has no LAND -> hovers until DISARM. His MPC's
  own disarm never lands the fleet (fleet manager 314-322; /drone_3/fc_arm_state has the signal). Behind switches, card +
  critic (the tilt-cut card): his planner flies out 1.2 m and lands if the mux does not read ours 1 s after a take-back;
  warm tracker fault = not live, latches only with authority; FC disarmed after his takeoff -> fleet LAND. Until built,
  sheet line: "T2 says 'taken back, but our tracker is not flying yet': clear the cage, DISARM, QUAD4 falls" (Wesley
  accepts in writing).

### Rev 2's response, item by item
- **C1:** the rule runs in the tracker on its own pose subscription, on every fresh pose, ahead of the pose-timeout and
  armed gates and of the envelope latch, with its own counter and its own flight latch (TAKEOFF to the next ARM).
  Tested: the same-tick crossing, and speed first then tilt. T6's bar: TILT_FAULT after the speed latch.
- **H1:** `partner_stuck_guard` (the hand-over block, refused by the manager, announced; the LANDED cut at rest, else
  the DISARM line); T0 is scored on the stuck take-back and armed after LANDED; the sheet line is drafted (W3), not
  written. F6's `warm_fault_not_live` and `taken_back_self_land` are the two ways out, each its own arm.
- **H2:** section 1 restated (the snag, the mis-solve, the motor; his stale-pose self-disarm, 0.25 s rig / 1.0 s sim);
  "cannot recover" struck; "ten times anything his MPC commands".
- **H3:** the fault path overrides an earlier take-back and clears the departed state; `cut` ranks with
  `_abort_latched` in `_forward` and `_publish_state`; unit tests for LAND/fault, give-up/fault, fault/LAND, take-back
  and weld after the cut.
- **H4:** the mux forces the magnet channel OFF and the rig latch OFF in the frame of the cut; the manager's OFF on
  `cut` overrides the confirmed attach; T2/T3 bar: OFF on the mux output <= 0.1 s after the cut.
- **M1:** the glitch guard (45 deg single-pose step, 0.5 s hold-off); the rig replay of body 14 (7-8 Oct) in section
  1: no glitch step above 30 deg in flight, max 28.1 deg; no Tejen QUAD4 logs in the repo (W4 asks); elrs_interface
  publishes `fc_attitude`.
- **M2:** speed only logged ("would cut (speed)"); QUAD4's peak speed under his stream 0.63-1.19 m/s (p99.9 0.70) in
  the twin, body 14's p99.9 0.50-0.81 m/s on the rig; his references capped at 1.0 m/s per axis.
- **M3:** built as `tilt_slow_deg` (default 0 = off): never fires on body 14 in flight on the rig; fires on disarmed
  drones resting tilted, which no consumer acts on. Recommendation: off for Wednesday.
- **M4:** `welded_on_load` is its own switch and arm (T5b, T5c); on_load False only once the tip has left the plate.
- **M5:** the fault and cut topics are volatile and reliable; unit-tested that a fresh mux starts uncut.
- **Test plan:** T0 scored as asked; the tilt criterion excludes the MOCAP_TILT window; criteria only in the new tc_*
  configs; T2/T3/T9 report ring tilt without a bar (the falsifier: an abort through drone 3's path); T9 added with its
  physics first; T7 falsifies at 0.02 s, T7b (0.04 s) is information; T1 injects 1.4 m from the carriers; F1/F2
  cannot measure rig false trips (the replay does).
- **Consumers act only while his stream flies QUAD4** (new in rev 2, from the rig replay): the planner and the fleet
  manager ignore a tilt fault while ours flies it or after it was disarmed alone; its own disarm decides there, as today.
- **Wednesday:** W1-W4 drafted above.

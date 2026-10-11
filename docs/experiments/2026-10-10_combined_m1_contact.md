# Combined M1 subtest: Tejen flies QUAD4 from its mat through pickup, drop and a contact attach proved by a small lift; our check confirms; bumpless hand-over

Rev 10 (11 Oct, written before any run on it; complete except the final tree SHA). Wesley's decisions of 11 Oct
(decisions.md 11:55 and 12:50):
- a take-back while the ring flies a trajectory moves with the ring (D1); co-move only, no pause-the-circle fallback;
- the take-back hold ramps from the measured velocity, with no switch (D19);
- partner_contact refuses trajectories other than hover and orbit, and anything but hover with `real:=true` until
  the co-move is reviewed (D20);
- every bar frozen before any run (D4);
- contact_still on by default (0210a27).

What rev 10 adds:
- §3b: the take-back (TB3, TB13, TB16).
- §5: the take-back bars (TB4).
- §6: the desk replay (TB5), the twin arms (TB6), the claim on the final tree and the landing tip-over note (TB14).
- §8: the new code, and a round-4 placeholder under "## Critic".

- **Tree lineage:** week4 -> m1-rig 0210a27 -> m1-tiltcut4 de91ba2 (the tilt cut merged with 0210a27, every tilt
  switch off) -> m1-comove (this rev's code, branched from m1-tiltcut4) -> the candidate -> week5. week4 stays as it
  is (D5 as changed 11 Oct).
- **Final tree SHA for the claim:** `________` (named before run 1; nothing else in rev 10 changes then).
- **Thu 15 gate:** if the co-move has not passed TB6 (§6.2) by then, it stays out of the candidate. The claim tree
  is then m1-tiltcut4 plus the TB16 refusals, the circle take-back arms are flown and reported as failing, and the
  rig flies hover only on 21-22 Oct (D1, no fallback; §6.5).
- **Record:** the c6f2032 rows (R1565-R1579) are the record.
- **Unchanged:** the bars of earlier revs. §6's stop-after-two-fails sentence is deleted (the rule was removed
  11 Oct).

Rev 9 (11 Oct 10:30): on ae57bbb the rise-first take-back fixed the rim snag (R1553), but on the circle the step
out at the rise height (1.78 m) passed 0.24 m over a carrier (R1554, under the 0.30 m bar). Fix c6f2032: the rise goes
to the higher of the rim clearance and the highest carrier + 0.4 m before any sideways move. The claim is re-flown on
m1-rig c6f2032 (queued after the tilt-cut wave A); the ae57bbb rows (R1550-R1564) are the record. Bars unchanged.

Rev 8 (11 Oct 09:15): R1543 (_lead030 on d4248af) showed the take-back from plate height hooks the rigid rod on the
rim when it steps out at its height (ring 22 deg, QUAD4 54 deg, fault LAND). Fix ae57bbb on m1-rig: a take-back with
the tip down at the ring rises to ring z + body-to-tip + 0.15 m first, then steps out. The claim table is re-flown on
m1-rig ae57bbb (one tree); the d4248af rows (R1531-R1550) are the record. Bars unchanged.

Rev 7 (11 Oct 07:55, before any run on it): the claim table is flown on the merged rig tree m1-rig d4248af (configs
ead6afe) = m1-final d8c1b65 + the rig port (all rig changes behind rig switches; the launch graph is identical to
d8c1b65 for all 360 frozen cases up to paths). d8c1b65 adds to 6143989: his TAKEN_BACK after the drop holds his object
magnet OFF (R1520: the take-back caught the ball in the net and dragged the ring) and the LIFT_PROOF goal max(target,
measured) + rise (R1522). Bars and thresholds unchanged; contact_still stays OFF in the claim (Wesley's word); four
m1x_still_* runs with it ON fly beside the table as exploratory evidence. The 7b8985a / 6143989 rows (R1505-R1529) are
the record of the failures that led here.

Rev 6 (10 Oct 22:20, Wesley, after the first claim pass R1491-R1502 on e097a94): two fixes, then the whole claim
table is re-flown on the one final tree m1-final 6143989 (= e097a94 + the full M1 + these fixes); the e097a94 rows stay
as the record. (Update 23:35, before any run on 6143989: the pickup stalls were the ball WELDED and his descent pushing on
until the rigid rod buckled (R1507 on 7b8985a, the same in R1493/R1497/R1500), which the first retry could not see; his
descent now ends on the weld and his retry also covers a ball attached but misaligned; 7b8985a runs R1505-R1514 are
exploratory or void.) (1) His LIFT_PROOF rises `proof_rise_max` above the measured contact height, not above his target
(R1498 `_low`: three caught contacts never proved). (2) His bounded pickup retry under `contact_attach` (no ball after
10 s: release, rise, re-approach the ball where it is; the third failure is a give-up and our take-back): his pickup
stalled to LAND in R1493 / R1497 / R1500, so those arms were never scored. Harness: WAIT_WELD 150 -> 210 s so a retry
fits. The bars in §5 and the frozen thresholds are unchanged. No critic round: neither is a control law, an estimator
or a fleet gate (the proof lift is a reference offset; the retry ends in the existing give-up path).

Rev 5 (10 Oct, before the claim runs; Wesley's decisions after E0/E1): the proof thresholds frozen at lift 1 cm,
d_ok 3 mm, d_fail 8 mm, h_plate 2.5 mm (the card's mapping cannot be met; QUAD4's mocap force cannot confirm within the
lift: decisions.md); the fold-in seed is 'start' (QUAD4 at 0.1 N; 'ocp' tilted the ring 7.7-8.6 deg, 4 of 4); after a
failed proof his controller re-presses from the lifted offset; QUAD4 tipping over in the landing unwind is a rigid-twin
artefact, so LAND is scored separately and does not hold the claim; with the ball on, the take-back steps out by
1.2 m + the rod. Claim configs configs/experiments/m1c_*.yaml (13 runs, one tree). The bars in §5 are unchanged.

Rev 4 (10 Oct): the critic's round 3 (GO WITH CHANGES) applied: the take-back is specified as new code (our abort
was only tested from our own approach), the clearance pairs, the threshold mapping and stop rule for E1, the ball's
weld and bitmask, a higher transit, the rig's one coil. Rev 3: the rig twin only, Tejen's full mission from the mat
(Wesley), our abort when no attach happens (Wesley). Rounds 1-3 are condensed under "## Critic".

## 1. Question
Wesley, 10 Oct: "Tejen's controller should fly the quad until it has attached and confirmed the attach, only then
should our controller take over."

The subtest: "we take off 3 drones connected to the payload, he takes off quad4 which picks up the object, flys and
drops it in (using his obstacle avoidance + controller) then attempts the attach".

His three goals:
1. give Tejen's controller the best chance of attaching;
2. a reliable way of telling when an attachment has happened;
3. a good hand-over to our controller.

On the rig twin, with the ring hovering and with it circling, does the subtest run end to end?
- His controller takes QUAD4 from its mat through the pickup, the transit and the drop.
- It brings the magnet onto plate 9 and proves the catch with a small lift.
- Our motion-capture check alone confirms it.
- Only then does the radio switch to our tracker, without a jolt, and the planner fold QUAD4 in.
- If no attach happens, our tracker takes QUAD4 away from the ring and lands it.

## 2. Hypothesis
- **Contact.** His contact descent, with the magnet ON, no reference lead, his ground-ring gates and his target on the
  full plate pose, reaches plate 9:
  - within 10 s of his descent start on the hovering ring;
  - in at most 3 tries on the moving ring (0.125 m/s).
- **Proof.** It separates a catch from a near miss by the tip's drift from where it touched and the plate's own rise:
  no false attach in the `_nocatch` arm, no rejected catch in the main arms.
- **Hand-over.** It happens from a QUAD4 already carrying a little of the ring, with our tracker warm on its measured
  state, the commands actually applied and its heading. There is no jolt (per-channel steps below), and the ring tilt
  peak from contact to the confirmation +8 s stays within the G1 rejoin bar (5 deg).
- **Most likely failures:**
  - his height: his MPC has no z integral, so a thrust-ratio mismatch is a steady z error; in E0 (R1476) his loaded
    lift ended 14 cm under its target while his estimate converged (22 -> 18.1 and still falling). A few cm of that
    at the plate is a contact that never comes (too high) or a tip pushed through it (too low; no contact physics in
    the twin);
  - the moving ring (his real lag puts the tip a few cm off: retries);
  - the rigid twin rod resisting the lift (the proof waits for plate rise; Critic B).

## 3. The one variable
The attach and hand-over, set by `partner_contact` (our launch) and `contact_attach` (his parameter), both default
false (today's ATTACH_READY hand-back, unchanged). The bundle is Wesley's decided design; the neighbours in §6
separate the lead, the target height, the catch and the contact offset.

**The subtest itself is not the variable.** It is the same for every arm:
- World: three_rigid_ground_rig_attach_pickup.sdf, generated (528b693):
  - carriers on plates 3/11/7;
  - QUAD4 on its mat at (0, 1.4) with a pickup weld;
  - his ball at (1.0, 1.6);
  - an open ring with a net (bit 0x04).
- His mission in pickup_delivery with `geometry:=rig_twin` (ring 0.225, rods 0.55 at 60 deg, body-to-tip 0.59, his
  rig MPC values, thrust ratio 22 with feedback), his plate 3 (= rig plate 9; his numbering runs counter-clockwise),
  a high transit with `minimum_search_z_m` 2.7 (the ball's underside clears the carrier bodies at ~1.52 m by ~0.3 m;
  the cage is 4 m). The geometry profile owns his plate and carrier-azimuth defaults.
- The world, from round 3:
  - his ball as a generated copy with collide bitmask 0x05: it hits the floor, the ring and the net but not QUAD4's
    tip, as in the old world (the tip had no collision there);
  - his launch repeats `/pickup/detach` through the ROS bridge until `/pickup/detachable_joint_state` shows free, and
    his takeoff waits for it (the spawn weld is a 1 m lever; a one-shot gz publish was lost before, R0678).
- Ours: the rig-twin attach profile (rig thrust map, measured pull, model integral), the ring at 1.0 m.
- Config m1_subtest_hover / m1_subtest_orbit.

**Goal 1, the best chance to attach** (his code, every change in CHANGES_FROM_TEJEN.md):
- **The magnet.** QUAD4's ring magnet is ON from his rejoin start (TRANSIT_TO_REATTACH, after the ball drop: on the
  rig the one coil's only OFF) until the attach.
  - Our planner sends it, only once QUAD4 is airborne and `/payload/detachable_joint_state` shows the spawn weld
    free.
  - The manager is the arbiter: it refuses OFF once it has published attached, and never declares attached after
    accepting an OFF.
  - OFF comes only from our LAND before the attach, and from his give-up.
- **The descent.** DESCEND_TO_ATTACHMENT descends to the contact target:
  - twin: the tip sphere's centre at the plate top + 0.012 (ring body +0.032; `bridge_attach_z_m` 0.032,
    `final_attach_offset_z` 0);
  - rig: 1 cm below the calibrated face-on-plate height (Wesley).
- **The hold.** ATTACH_READY under `contact_attach`: no hand-back flag, lead 0, his ground-ring gates (match
  0.025 m / 0.08 m/s, abort 0.05 m / 0.15 m/s).
- **LIFT_PROOF** (new). It is entered from DESCEND or ATTACH_READY on our `/magnet/contact`, checked before his
  abort guards.
  - He rises at 0.02 m/s, capped at `proof_rise_max` (twin 0.01 m, rig 0.03 m), and stops as soon as the proof
    passes.
  - On `/magnet/contact` false, back to the descent with the magnet ON.
  - After 3 failed proofs or 30 s from his first descent start: give-up (publishes `/join_planner/give_up`).
- **RING_ATTACHED** (new, terminal). It is entered from LIFT_PROOF or any rejoin phase on our attached signal.
  - It holds the lifted offset (not the contact target), plate-relative with no lead, until `/drone_3/mux_state`
    reads "ours"; then it holds.
  - No magnet command, no hand-back, no restart after our LAND.
- **His target uses the full ring attitude.** ring_bridge rotates the plate offset by the full ring attitude,
  unit-tested equal to our weld point on this world.

**Goal 2, a reliable attach signal** (ours, magnet_attachment_manager `confirm_mode: contact_lift`; the existing
check stays the default; every timer on the sim clock):
- **Contact:** the tip within 0.01 m of the plate point, relative speed <= 0.05 m/s, for 0.15 s. It publishes
  `/magnet/contact` true and latches the tip-to-plate vector c0 and the plate height z0.
  - In the twin this is also when the simulated magnet catches (the gz joint), standing in for the physical catch;
    the `_nocatch` arm skips the joint.
- **Proven** (his PickupProofTracker pattern, m1_pickup_proof.py:100-123), for 0.3 s:
  - the tip-to-plate vector stays within `d_ok` of c0;
  - the plate point has risen >= `h_plate` above z0;
  - rig only: body-to-tip >= the taut length from Wednesday's record (601ef22) minus 3 mm.
  Then `/magnet/object_attached` true.
- **Failed:** drift >= `d_fail` for 0.12 s, or no decision within 3 s of the lift start. Then `/magnet/contact`
  false.
- **Thresholds.** Start values d_ok 0.003 m, d_fail 0.008 m, h_plate 0.003 m. E1 (§6) sets them by this mapping,
  written before E1, and they are frozen before the first claim run:
  - h_plate <= half of E1's plate-9 rise under the capped lift, and >= 3x plate 9's z jitter without an attach
    (from logs on disk: R1228 / R1229 orbit, R1404 / R1410 hover);
  - d_ok >= 4x the tip-to-plate drift jitter in E1's contact hold; d_fail stays 0.008 m;
  - c0 is the mean over the 0.15 s contact dwell; the drift test uses a 0.1 s moving average, not raw samples.
  - **Stop rule:** if E1's plate rise is under 2x the jitter, stop and report: a 1 cm twin cap cannot prove a catch,
    and the cap is Wesley's to change.
  - The rig thresholds come from rig data (plate-marker noise from the 7-8 Oct carries, Wednesday's taut length),
    not from the twin.
- **A failed proof with the twin joint made** (a falsifier in a main arm) releases the joint before he retries, so he
  never descends again while welded.
- **Force:** QUAD4's vertical external force (mocap acceleration minus the applied throttle through the thrust map)
  is logged at 50 Hz from his descent start to the switch. It is not in the decision (Wesley).

**Goal 3, the hand-over** (ours):
- **The mux** switches QUAD4 to ours on `/magnet/object_attached` (the attach) or on the take-back (his give-up, our
  LAND before the attach). His `/join_planner/handoff_ready` is not wired.
- **The planner:**
  - from his descent start, a warm reference for QUAD4: its measured body plus the ring velocity as feed-forward
    over the horizon;
  - no departed hold for QUAD4 (`_attach_ocp` drops any entry);
  - the fold-in's QUAD4 share seeded from the measured pull instead of `attach_t_start_new` 0.1;
  - the model integral logged through LIFT_PROOF and the switch.
- **The drone-3 tracker while the mux reads "partner":**
  - `_applied_u` (all four channels) and `last_cmd_throttle` are pinned to the mux output `/drone_3/ELRSCommand`;
  - `_applied_cable_vec`, the thrust history and its integrators are frozen.
  At the switch: x0 from the measured state, the solver re-seeded (first_solve), the heading datum re-latched.
- **No attach: the take-back** (Wesley 10 Oct: our tracker takes QUAD4 back). This is NEW code. Our abort was only
  ever tested from our own approach (LAND matrix R1413-R1446); from his stream it does nothing today (partner runs
  never start our approach, so `_abort_newcomer` finds nothing, and away from the ring our drone-3 tracker has no
  reference at all). So:
  - **Warm from his takeoff:** from his TAKEOFF on, our drone-3 tracker gets a reference (QUAD4's measured body), its
    takeoff spool marked done, its u-state pinned to the applied commands and its integrators frozen, as for the
    attach hand-over. Warming from ARM cost pose-timeout disarms on the laptop (dissipative_node.py:886-891); these
    runs are on the lab PC, and a zero-pose-timeout bar watches it.
  - **At the take-back** (his give-up, or our LAND before the attach): the mux goes to ours; a fixed hold latched at
    the measured position (rev 10: ramped from the measured velocity, and moving with the ring on a trajectory,
    §3b); the solver re-seeded; then the departed path:
    - within 1.2 m of the ring centre: radially out to 1.2 m at its height, then land alone; over the middle
      (r < 0.225) the step goes out at plate 9's azimuth (+y, the gap between carriers 1 and 2);
    - further out (the pickup): land where it is.
  - **The ball:** while `/tejen/object_attached` is true, drone 3's model mass includes the ball (0.1 kg, ~18 %).
  - **His side:** a terminal TAKEN_BACK phase with no references and no object-magnet commands (the ball stays on
    and lands with QUAD4).
  - **The magnet:** the ring magnet goes OFF only if the ON span has begun. On the rig one coil holds the ball, so an
    OFF before his drop would release the ball over the ring; before the span QUAD4 lands with the ball on.
  - `newcomer_lifecycle`, `newcomer_fault_land` and `departed_ground_idle` stay on under `partner_contact` (today's
    partner mode forces them off), with two gates:
    - `_newcomer_arming_cb` acts only while the mux reads "ours" (a disarm of our idle tracker during his flight must
      not turn the ring magnet OFF or take QUAD4 out);
    - our own span ON bypasses the ATTACH handler (no trajectory hold, no `_start_approach`).
  - LAND at any time before the attach does the same take-back; the runner's WAIT_REWELD gets a timeout (120 s)
    followed by LAND.

## 3b. The take-back that moves with the ring (rev 10; TB3, TB13, TB16)

**As built on m1-comove c53ae0c, with the round-4 response.** Where the steps below differ, this block wins. No bar
changes.
- **Ramp:** 0.25 m/s² (not 0.3), starting from the measured velocity. The step's carrot brakes at 0.2 m/s² and
  closes at 1.5/s near its goal.
- **No escape height.** On an orbit the zone rule decides from the stopping point, in this order:
  - **(b)** its body below ring z - 0.3 m (his takeoff, pickup, mat): it goes down where it stops, with no sideways
    move and no rise;
  - **(a)** outside r_clear (orbit radius + 1.0 m) from the path centre, or its lowest point (the tip, less the ball
    while he carries it) above the highest carrier + 0.15 m: world-fixed, c6f2032's path with the ramp;
  - **(c)** otherwise it moves with the ring: a ring-frame offset from the planned ring point latched at the
    take-back, z absolute. It rises to c6f2032's height only when the tip is at the ring (0.15 m below the ring centre
    or higher) or it is at least 0.9 m from the ring centre. Under the formation it descends with the ring instead,
    to no lower than ring z - 0.3 m or the tip 0.1 m over the floor, then lands where it stops.
  - **Why b comes before a:** a low QUAD4 far out (at his pickup) lands where it is instead of rising and moving
    sideways. That is what the falsifiers of m1x_land_pickup_orbit require.
- **The decouple:** each step counts as reached by its reference, within 0.02 m (F5). The decouple is one bounded
  velocity change into the path-clear move. Under LAND or a drop there is no path-clear move after the step out.
  - TB5 kept the ring centre at least 0.9 m from QUAD4 at every 10° phase after the decouple.
  - So the "decouple waits" branch was not built.
- **Rig release guard** (`take_back_release_guard`, on with `real:=true` only). It acts when our span has begun and
  the tip is on plate 9.
  - The rise stops 0.07 m above the higher of the hold and the tether's slack (the tip 5 cm off the plate, plus
    2 cm). It waits there until the tip has left the plate by 5 cm in the ring frame.
  - **If it fails** (plate 9 rises 2 cm, or the tip has not left within 2 s): QUAD4 is held in the ring frame, OFF is
    re-sent and the fleet lands.
- **Constants:**

  | name | value |
  |---|---|
  | TAKE_BACK_ACCEL | 0.25 m/s² |
  | TAKE_BACK_BRAKE | 0.2 m/s² |
  | TAKE_BACK_K | 1.5/s |
  | TAKE_BACK_REACHED_M | 0.02 m |
  | ZONE_OVER_CARRIERS_M | 0.15 m |
  | ZONE_BELOW_RING_M | 0.3 m |
  | ZONE_RISE_FAR_M | 0.9 m |
  | TIP_AT_RING_BELOW_M | 0.15 m |
  | TIP_OVER_FLOOR_M | 0.1 m |
  | BALL_DIAM_M | 0.16 m |
  | GUARD_PLATE_RISE_M | 0.02 m |
  | GUARD_WAIT_S | 2.0 s |

  The tip-at-ring band, the brake, the gain, the ball and the guard's rise go beyond the critic's values. They are
  Wesley's to confirm.

**Why.** On the circle the take-back has failed three ways, each with a world-fixed hold:
- the step out at plate height hooked the rod on the rim (R1543, R1549);
- a rise to rim height, then out, passed 0.24 m over carrier 1 (R1554);
- a rise over the carriers while held world-fixed let the orbiting formation sweep carrier 2 into QUAD4's rod. Carrier
  2 reached 66.7 deg and the fleet made an emergency stop (R1565). R1566 survived the same code at another phase.

At each give-up QUAD4 was moving with the ring at 0.13-0.17 m/s, body at 1.60-1.63 m, tip at the ring (ring 1.0 m,
carriers 1.53-1.54 m; run.csv of R1543, R1549, R1554, R1565). In `_land_transit` the fixed hold braked QUAD4 from
0.26-0.34 m/s at 2.9-3.0 m in one step: a tilt step of 4.4-4.7 deg in 0.3 s (R1495, R1524, R1542).

**The ring frame.** An orbit translates the formation without yawing it (`yaw_at` yaws only 'spin'), so the ring
frame is the world axes moved with the ring's planned point:
- `c(t) = hover_xy + traj.offset_at(traj_t)`, with velocity `v_c` from the same call;
- the reference uses the plan, not the measured ring, so mocap noise and the ring's own disturbance response stay out
  of QUAD4's reference;
- the cost is the ring's lag behind its plan: 2.5-5 cm at 0.125 m/s (rig circle lag 0.20 s), small against the bars.

**The take-back, step by step.** It starts on his give-up or on our LAND before the attach.
1. **Latch, ramped (TB13).**
   - The reference starts at QUAD4's measured position and velocity. Its velocity then moves toward the target
     velocity (the frame velocity `v_c` plus the current step's carrot) by at most `TAKE_BACK_ACCEL` = 0.3 m/s² per
     tick. There is no velocity step at the latch, when the ring stops, or at the decouple.
   - 0.3 m/s² is a quasi-static tilt of 1.75 deg, under the 2 deg bar. It stops 0.34 m/s (R1542) in 1.1 s over
     0.19 m.
   - Every later decision uses the stopping point, not the latch point: step out or land where it is, and which way
     (R1482 decided while still moving inward).
   - On the circle QUAD4 already moves with the ring, so the ramp is nearly nothing. In hover and in his transit
     `v_c` is 0, and it is the plain ramp.
2. **Ring-relative hold.** The offset `delta = p_ref - c` is held in xy, so QUAD4 moves with the ring. z is absolute:
   QUAD4 never follows the ring down. The magnet OFF and the release wait (`NEWCOMER_RELEASE_S` 0.5 s, today
   world-fixed) run in this frame. The tracker's horizon carries `c(traj_t + k dt) + delta_k` with the velocity fed
   forward (today zero), gated like the planner's own horizon: a frozen clock draws no motion.
3. **Rise, straight up in the ring frame**, before any sideways move. The height is the higher of QUAD4's own height
   and the targets below.
   - **Hover:** c6f2032's rise, unchanged. It goes to the higher of the rim clearance (ring z + body-to-tip + 0.15 m)
     and the highest carrier + 0.4 m, and only when the tip is down at the ring.
   - **Orbit launch (whatever the clock reads):** the escape height, the highest carrier + body-to-tip + 0.15 m, plus
     the ball's diameter while his ball is on.

   | twin heights (ring 1.0 m, carriers 1.53-1.54 m, body-to-tip 0.59 m) | body | tip |
   |---|---|---|
   | rim clearance: ring z + 0.59 + 0.15 | 1.74 m | 1.15 m |
   | c6f2032: highest carrier + 0.4 | 1.94 m | 1.35 m, **0.19 m below the carrier bodies** |
   | escape (orbit): highest carrier + 0.59 + 0.15 | 2.28 m | 1.69 m, 0.15 m above them |

   - **Must the rod clear the carriers' bodies? Yes, wherever a carrier can pass under QUAD4, which on a circle means
     after the decouple.**
     - While QUAD4 moves with the ring, nothing passes under it. The plate-9 corridor (+y in the ring frame)
       keeps at least 0.43 m from the carrier bodies (30 / 150 deg at r 0.5) and about 0.2 m from their rods.
     - After the decouple the formation keeps orbiting, so at c6f2032's height a carrier passing under would meet the
       rod 0.19 m below its body (R1565's mechanism).
     - In hover nothing moves under QUAD4, so the c6f2032 rise stays.
   - The escape height is below his transit (2.3-2.9 m) and well under the 4 m cage.
4. **Step out in the ring frame.**
   - It goes to 1.2 m from the ring centre, plus body-to-tip while his ball is on (as today), at the rise height,
     at `DETACH_STEP_VEL` 0.15 m/s.
   - Over the middle (r < attach_radius) the direction is plate 9's azimuth, the gap between carriers 2 and 3.
     Otherwise it is QUAD4's own offset direction (today's rule).
5. **Decouple, bounded.** Only once:
   - the release wait is over;
   - QUAD4 is at the rise height;
   - the step out has reached its goal.

   Then the frame velocity ramps to zero at <= `TAKE_BACK_ACCEL`. There is no decouple below the escape height on an
   orbit, and none inside the step-out radius.
6. **Clear the path, then land alone.**
   - At its height, QUAD4 moves to `_path_clear_point` (orbit radius + 1.0 m from the path centre), ramped at both
     ends, then descends and lands alone. That is today's departed path: rest, press, disarmed alone.
   - Already that far out, it descends where it stopped.
   - **Overflight:** between the decouple and the path-clear point the formation can pass under QUAD4 inside its
     swept disc (orbit radius + carrier radius ~0.5 m + props). The tip is then 0.15 m above the carrier bodies and
     the body ~0.75 m above. The twin has no rotor wash, so the effect on the carriers and on a slack rig tether is a
     rig finding. TB5 and TB6 report the overflight time and the minimum vertical gap.

**Events during the take-back:**
- **The ring stops:** a reconfiguration hold, the trajectory hold, LAND or the end of a circle freezes `traj_t`. `v_c`
  then goes to 0 and QUAD4 follows through the same acceleration bound (overshoot <= 2.6 cm at 0.125 m/s). The
  remaining steps run in the now-still frame.
- **LAND mid-take-back:** the clock is frozen (the planner is descending), so the frame stops as above. The ring and
  carriers descend away from QUAD4, whose z is absolute. The take-back finishes its rise, step out and path clear,
  then lands alone. The `_taken_back` latch means a second take-back never starts.
- **LAND before his give-up:** the take-back starts at LAND with the frame already stopping. The arm is
  `m1x_land_rejoin_orbit`.
- **Faults** (rewritten after critic round 4, F6):
  - Our tilt-fault callback acts on QUAD4 only while the mux reads partner or cut.
  - Once ours flies QUAD4, as in every take-back, a fault makes our tracker's envelope cut QUAD4's motors. Then:
    - in the subtest, newcomer_fault_land lands the fleet;
    - with the welded start (newcomer_fault_land off), it is a fleet stop.
  - Mid-rise, QUAD4 would fall from the rise height about 0.43 m in xy from carriers 1-2.
  - The tilt cut is off on 21-22 Oct.
  - The co-move state is in the explicit lists of `_tilt_fault_cb` and `_drop_departed` (c53ae0c). The H3 test
    checks that drone 3 gets no reference after a tilt fault in the race window.
- **Far take-backs:** the zone rule above decides them. There is no far exemption and no escape height.

**Hover is unchanged** (c6f2032): the same rise, the same step out through the gap, the same landing. With `v_c = 0`
the frame is the world. The only change in hover is TB13's ramp (D19), which `m1c_land_transit` shows (hence
`_land_transit_2` in the claim).

**TB16, launch refusals** (in the attach graph; not dependent on the co-move, so the refusal is built either way):
- `partner_contact` with `load_traj` other than hover or orbit is refused in the twin and on the rig.
  `_path_clear_point` clears circles only, and the combined M1 has only flown hover and orbit.
- With `real:=true`, orbit is refused too until the co-move is in a reviewed tree. Lifting it is a one-line change
  with Wesley's word after the reviewer.

**Recorded alternative (not a fallback): B2.** QUAD4 keeps moving with the ring through the gap until it is outside
the swept disc, so it never overflies a carrier.
- It costs up to two thirds of a lap (~17 s) and comes to about 2.2 m toward the +y cage side.
- Switching to it needs Wesley's word, its own replay and TB6 before the Thu gate.

## 4. Baseline
- **E0** (exploratory, running on the lab PC): the same subtest with today's ATTACH_READY hand-back. It finds what
  breaks in the takeoff, pickup, transit and drop on the rig twin, and gives the hand-back tilt on this world.
- **The port tree with today's hand-back, old world:** R1474, REWELD 5.62 deg (the same tree as m1-subtest; one
  run), next to R1473's 2.52 deg (rig-prep).
- **Rig twin, our own approach** (not his): R8b hover R1404 / R1410 / R1454; held circle R9 R1468 / R1471 (3.0 /
  2.9 deg); moving circle R1469 / R1472 (no weld).
- **Earlier tries on the old world:**
  - R0623: his hold over the orbit cycled for 220 s, and a late weld at LAND tipped drone 0 to 87 deg;
  - R0640 / R0642 / R0645: a still-moving weld, 25-35 deg;
  - R1473: today's hand-back, 2.52 deg.
- **Scoring:** tools/m1_stages.py (reproduces R0752's DETACH 2.80 / REWELD 2.74), extended for §5.
- **Rev 10 take-back baseline:** c6f2032.
  - The circle fails: R1565 (and R1543 / R1549 / R1554 on earlier trees).
  - Hover passes: R1553, R1562, R1567, R1570.
  - The transit tilt step is 4.4-4.7 deg: R1495, R1524, R1542.
- **Rev 10 scoring:** `tools/m1_score.py` with `tools/m1_metrics.py`. Committed 986ac0a (m1-score), in the candidate
  base m1-comove.
  - Families: claim, full, take-back (TB4 plus the TB5 prediction), tilt, and the TB14 count.
  - Keyed on WELD or REWELD.
  - Rod clearance is segment to segment, with the ball while carried.
  - `-corr` rows are read first.
  - Before scoring any rev 10 run, it reproduced R1573, R1579, R1581, R1615, R1620, R1623 and R1624 field by field.
  - tools/m1_stages.py is not in the rig tree.

## 5. Pass / fail numbers (per run; written 10 Oct before the code; d_ok / d_fail / h_plate frozen after E1)

**Authority:**
- **Confirms:**
  - the mux switches to ours 0 to 0.2 s after `/magnet/object_attached`, and only then (or on a take-back);
  - zero "HOLDING approach authority" lines;
  - zero hand-back-started approaches;
  - zero magnet OFF or DETACH after the ATTACH line;
  - zero double references to drone 3;
  - the ring's trajectory never held by our own magnet ON (Critic H).
- **Falsifies:** any one of these.

**Contact** (his descent start to `/magnet/contact`):
- **Confirms:** within 10 s.
- **Reported as slow:** 10 to 30 s.
- **Falsifies:** no contact by his give-up, then the no-attach path must still pass.
- Counted: tries and his alignment aborts.

**Proof:**
- **Confirms:** proven within 3 s of the last contact.
- **Falsifies:** in a main arm, a contact with the joint made whose proof fails, or a proof that needs more than
  `proof_rise_max`.
- `_nocatch`:
  - confirms if there is never an attach, at least one failed proof, the give-up, our abort and a clean landing of
    every drone;
  - falsifies if an attach is declared.

**Hand-over** (QUAD4's applied command at the switch, last 0.1 s before against the first 0.3 s after):
- **Confirms:** throttle step <= 10 % of its hover throttle, roll and pitch steps <= 2 deg, yaw-rate step
  <= 10 deg/s, heading change over the 1 s after the switch <= 3 deg.
- **Falsifies:** > 25 %, > 5 deg, > 30 deg/s or > 8 deg.

**At the confirmation:**
- **Confirms:** QUAD4's tilt <= 3 deg.
- **Falsifies:** > 5 deg.

**Clearance** (the pairs of tools/attach_metrics.py; rod-to-rod is 0.225 m at contact by geometry, so the rod
window ends at contact):
- QUAD4's body to any carrier body, takeoff to the switch: **confirms** >= 0.35 m (the rig stop rule); **falsifies**
  < 0.30 m.
- QUAD4's body, and its rod until contact, to any carrier rod: **confirms** >= 0.15 m; **falsifies** < 0.10 m.
- The ball (while carried) to any carrier body or rod: **confirms** >= 0.15 m; **falsifies** < 0.10 m.
- Zero pose-timeout disarms of our drone-3 tracker while warm.

**Ring tilt peak:**
- Contact to confirmation +8 s:
  - **confirms:** <= 5 deg (G1);
  - **falsifies:** > 5 deg.
- In runs without a confirmation, over his rejoin:
  - **confirms:** <= 5 deg;
  - **falsifies:** > 5 deg.

**Ring dip** (the 2 s before contact against the minimum up to confirmation +8 s):
- **Confirms:** <= 0.01 m.
- **Falsifies:** > 0.02 m.

**The mission:**
- pickup, the ball in the net at LAND, contact, proof, attach, the fold-in;
- LAND -> every drone landed and disarmed;
- his planner in RING_ATTACHED from the attach to the end.

**Take-back (rev 10, TB4; frozen 11 Oct before any run on it).** These apply to every run with a take-back, from the
take-back to QUAD4 down ("departed ... is down"). The window starts at the take-back, so the earlier windows do not
cover it, and no run flown before rev 10 is re-scored with these.
- **Rod-segment clearance:** QUAD4's rod segment (body to tip, and to the ball's far side while carried) to every
  carrier body and every carrier rod segment.
  - **Confirms:** >= 0.15 m.
  - **Falsifies:** < 0.10 m.
- **Body clearance:** QUAD4's body to any carrier body, with the values above (>= 0.35 m confirms, < 0.30 m
  falsifies).
- **Attitude step:** QUAD4's tilt over the first 0.3 s after the event, against its mean over the 0.1 s before,
  maximum absolute change. Measured at the take-back and again at the decouple.
  - **Confirms:** <= 2 deg.
  - **Falsifies:** > 5 deg.
- **Ring tilt:**
  - **Confirms:** <= 5 deg.
  - **Falsifies:** > 5 deg.
- Values between the bars are reported as such, as R1552 was.
- **Reported, no bar:** the overflight time (any part of a carrier's rotor disc, a 0.2 m radius about its body
  centre, under QUAD4's body or rod in xy) and the minimum vertical gap from QUAD4's tip to a carrier body during
  it. The twin has no rotor wash, so this is a rig finding.

## 6. Repeats and order (the critic's order)
1. Tests (below), including the proof fed synthetic caught, uncaught, off-centre and sunk-tip traces.
2. E1: one exploratory lift run on hover, the new code with loose thresholds and full logging (body rise, plate rise,
   drift, ring tilt, his throttle, the model integral). It sets d_ok / d_fail / h_plate, which are then frozen.
3. Claim runs on one tree, in order (revs 4-9; flown R1491-R1579, the record; rev 10 replaces this with §6.3):
   1. hover;
   2. `_nocatch` hover;
   3. orbit;
   4. `_nocatch` orbit;
   5. the second hover and the second orbit;
   6. `_land_transit` (LAND while QUAD4 carries the ball in the high transit: the take-back), before any further
      orbit arm;
   7. `_lead030` orbit;
   8. `_high` and `_low` (`final_attach_offset_z` +/-0.01);
   9. `_offset` (a deliberate 6 mm contact offset in xy);
   10. `_land_proof` (LAND during LIFT_PROOF);
   11. `_abort`: a runner WRENCH push on the ring during his descent, on a push variant of the pickup world (to build
       with the generator's push mechanism).
4. The reviewer on the table, then the rig plan.

Rev 10 order:
1. The critic on rev 10 (round 4, under "## Critic") and the response.
2. The build on m1-comove with its tests (§8).
3. TB5, the desk replay (§6.1).
4. TB6, the twin arms (§6.2).
5. The Thu 15 gate.
6. The claim on the final SHA (§6.3).
7. The reviewer, then the rig card.

Between tweaks, one run on the arm that failed.

### 6.1 TB5: desk replay, before any lab time (no Gazebo)
The method:
- Feed the logged planned ring point (`payload_ref_x/y`, `traj_t`), the measured ring and the carriers' tracks into
  the m1-comove take-back reference generator, from each logged give-up.
- QUAD4 is placed on its reference, with its rod hanging straight down.
- The replay first runs the c6f2032 generator on R1565 and must reproduce the logged failure (carrier 2 within about
  0.28 m at about 119.4 s). Otherwise the replay is wrong.

The cases:
- R1565, and R1565 with the give-up shifted by a quarter, a half and three quarters of the 25.1 s lap;
- R1554, R1566, R1549 and R1543.

The report gives, for each case: the minimum rod-segment clearance and body clearance (§5 take-back bars), the
overflight time and the minimum vertical gap, and the decouple point against the swept disc.

**Acceptance:** rod-segment >= 0.15 m and body >= 0.35 m in every case. Otherwise fix the design before any lab run.
Every overflight goes to Wesley with its duration and gap, as a rig finding (B2 is his call). The logged carriers
do not react to QUAD4, so this checks geometry only; TB6 flies it.

### 6.2 TB6: twin arms on m1-comove (exploratory, one each, registry rows with these falsifiers)
"TB4" means the §5 take-back bars. Every arm also falsifies on a drone not landed. A TB14 landing tip-over (§6.4)
is the one exception, and it applies only with QUAD4 attached.

| # | arm (config) | what it tests | falsifier |
|---|---|---|---|
| 1 | m1c_nocatch_orbit | the arm that failed (R1565) | an attach declared; a TB4 falsifier |
| 2 | m1c_lead030 | a give-up from his approach on the circle (R1543, R1566) | a TB4 falsifier in its take-back (an attach with the lead is reported, not a falsifier) |
| 3 | m1c_nocatch_orbit_ph11 (own params file, traj_hold_s 11) | the give-up about 90 deg later in the lap | as 1 |
| 4 | m1c_nocatch_orbit_ph175 (own params file, traj_hold_s 17.5) | about 180 deg later | as 1 |
| 5 | m1x_land_rejoin_orbit (new: LAND in his DESCEND_TO_ATTACHMENT on the circle) | the take-back at LAND, the frame stopping with the clock | no take-back at LAND; a TB4 falsifier |
| 6 | m1_full_nocatch_orbit (new: welded start, sim_catch false, circle) | the co-move after a detach and his whole mission, newcomer_fault_land off | an attach declared; a fleet stop; a TB4 falsifier |
| 7 | m1c_land_transit | TB13, the ramp from transit speed (hover) | attitude step > 5 deg; a §5 or TB4 clearance falsifier; the ball not landed with QUAD4 |
| 8 | m1c_nocatch_hover | hover regression | a §5 or TB4 falsifier; a path other than c6f2032's (the TAKE-BACK line and the track: rise to max(rim clearance, highest carrier + 0.4), out to 1.2 m, down) |
| 9 | m1x_r11a_circle (new: `contact_dz +0.03` on the circle, a planned no-catch) | the rig's R11a-circle, rehearsed before R11c | a contact or attach declared; a TB4 falsifier |

**TB6 passes, for the Thu gate,** when:
- no arm meets its falsifier;
- every circle take-back (arms 1-6 and 9) confirms both clearance bars.

Between-bar values and the overflight list go to Wesley at the gate. The working rule applies: between tweaks, one
run on the arm that failed.

### 6.3 The claim on the final tree (rev 10; one tree, the SHA at the top)
- **contact_still** is on, from the default (0210a27); the m1c configs do not pin it.
- **Scoring:** every run against §5 as frozen, plus the take-back bars for every run with a take-back, with the
  committed scorers (§4).
- **Changed arms first, each with a second repeat:**
  - m1c_nocatch_orbit, m1c_nocatch_orbit_2;
  - m1c_lead030, m1c_lead030_2;
  - m1c_land_transit, m1c_land_transit_2.
- **Then the other ten:** hover, hover_2, orbit, orbit_2, nocatch_hover, high, low, offset, land_proof, abort.
  That makes 16 runs in the table.
- **`_lead030` scoring.** His 0.3 s lead has not made contact on the circle since e097a94 (R1525, R1543, R1552,
  R1566; R1496 on e097a94 did).
  - Its attach result is reported, not claimed: no contact supports lead 0, the design.
  - Its take-back is claimed on the take-back bars and the §5 mission and clearance bars.
  - If it does catch and prove, it is scored as a main arm.
- **Beside the table** (exploratory, D8): full M1 x3 (m1_full_hover, m1_full_orbit, m1_full_nocatch_orbit), and the
  R11a rehearsals (m1x_r11a_hover, m1x_r11a_circle). Their falsifiers are as R1503 and as §6.2.
- **TB15:** the proof-time distribution and the count of first-contact timeouts are reported (no new bar; the 3 s bar
  stands).
- **Retired:** the m1x_still_* configs (contact_still is now the default); their rows R1533, R1540, R1548 and R1549
  stay as the record.
- **If the co-move missed the Thu gate:** the claim tree is m1-tiltcut4 plus TB16. nocatch_orbit and lead030 still
  fly on the same bars and are reported as failing; the circle take-back is not claimed.

### 6.4 TB14: the landing tip-over (twin artefact; Wesley 9 and 10 Oct)
**The mechanism** (traces, 11 Oct 10:25):
- the welded QUAD4 sits 0.5 m from the ring centre, outside the rim;
- about 5 s into LAND its rod tip reaches the floor beside plate 9 while its body keeps descending 0.2-0.3 m;
- the rigid twin rod cannot shorten, so it levers QUAD4 sideways (r 0.5 -> 0.74) and past 60 deg: FLEET_ABORT.
- The rig tether goes slack (land_unwind false on the rig).

**Count** (registry rows): 12 of the 48 subtest runs that landed with QUAD4 attached, R1479-R1578.
- The runs: R1479, R1486, R1502, R1518, R1529, R1531, R1534, R1547, R1548, R1556, R1559, R1578.
- 0 of the 9 full M1 runs (R1517-R1579).
- Also R1582 on the tilt card.

**Frozen definition, for every rev 10 run.** A TB14 tip-over needs all of:
- QUAD4 welded at LAND;
- the abort at least 5 s after LAND;
- QUAD4's tip on the floor;
- its tilt passing 60 deg within 1 s while its body is above its rest.

**How it is measured** (Wesley 11 Oct, before any rev 10 claim run; `tools/m1_score.py tb14`).
- **Why:** the round-4 F10 wording (tip z <= 0.03 m, within 1 s of the tip first reaching the floor) cannot be met.
  The welded tip rests on plate 9 at 0.052-0.060 m with the ring on the floor, and the 60 deg crossing comes
  1.5-2.6 s after touchdown. Read that way it finds 0 of the 12 known tip-overs.
- **Tip on the floor:** ring z < 0.07 m and tip z < 0.10 m (the tilt card's §9.2 thresholds).
- **Within 1 s:** QUAD4's tilt rises from under 10 deg to past 60 deg in at most 1 s. It took 0.17-0.34 s in all 12.
- **Body above its rest:** more than 2 cm above its first logged z.
- **Check:** this finds exactly the 12 of 48 above (and R1582), and none of the full M1 runs.

Such a run is scored LAND "tip-over (TB14)" and does not hold the claim. Any other abort or disarm, or one before
LAND, counts against the mission bar. On the rig the attached-QUAD4 landing is checked at R11b.

### 6.5 The rig on 21-22 Oct
- **Hover only (R11a, R11b)** unless the co-move has passed TB6 by the Thu 15 gate (it then goes into the
  candidate) and the reviewer has passed the final table with it before the flight. The `real:=true` refusal of
  orbit stands until then, so the circle cannot be launched by accident.
- **If both hold:** R11a-circle flies before R11c, and R11c flies only after R11b has passed.
- **If not:** R11c waits for 28-29 Oct (D1, no fallback).

## 7. Cost
- E0 + E1: 2 runs.
- Claim: 13 runs.
- Lab PC whenever free (Wesley 10 Oct), ~15-20 min each.
- Rev 10:
  - TB5: 2 h of desk work.
  - TB6: 9 runs, ~1.5 h wall on 4 lab slots.
  - The claim: 16 runs plus 5 beside the table, inside the joint batch (plan step 28).

## 8. New code?
Yes, behind `partner_contact` / `contact_attach` / `confirm_mode`, default off:
- **His planner:** the magnet span, the contact descent and hold, LIFT_PROOF, RING_ATTACHED, the give-up, the
  full-pose target.
- **Our manager:** contact, proof and arbitration on the sim clock, the simulated catch split from the confirmation.
- **Our planner:** the warm reference from his takeoff, no departed hold, the fold-in seed, the magnet span, the
  take-back (fixed hold, departed path with the plate-9 direction rule, the ball's mass), the two lifecycle gates.
- **Our tracker:** the applied u-state while not in authority (throttle through `_realised_model_throttle` from
  (ch2+1)/2, the model units), the spool marked done, frozen states, the re-seed and heading at the switch.
- **The mux:** the take-back input.
- **The runner:** the WAIT_REWELD timeout.
- **His launch and the world generator:** the repeated pickup detach and its gate, the ball copy (0x05), the
  geometry-owned plate and azimuth defaults, the pickup push world.

Tests first:
- the proof state machine (caught, near miss, off-centre catch, sunk tip, slow catch, loss);
- the manager's arbitration;
- his phase transitions (from DESCEND, ATTACH_READY and LIFT_PROOF; the give-up);
- the full-pose target equal to our weld point on this world;
- the tracker's u-state source and the re-seed;
- the mux take-back;
- no double references.

If it fails, everything behind the switches is deleted and the attach roles go back to Wesley with the logs.

**Rev 10 (branch m1-comove from m1-tiltcut4 de91ba2; no new switch: the co-move replaces the take-back that failed
on the circle, and hover keeps c6f2032's path, D19):**
- **dissipative_node.py:**
  - `_take_back` latches the offset and velocity against the planned ring, picks the heights and the ring-frame step;
  - `_publish_departed_refs` gives the moving hold, the release wait and the step, a horizon with the velocity fed
    forward, the ramp and the decouple;
  - the constants as built are listed at the top of §3b. `TAKE_BACK_ESCAPE_TIP_M` went with the escape height
    (F2).
- **Bringup attach graph:** the TB16 refusals.
- **docs/design/m1_contact_contract.md:** updated.

Tests first (src/dissipative_planner/test/test_partner_contact.py and the bringup launch tests):
- the hold tracks the planned ring and feeds its velocity forward;
- the latch starts at the measured velocity;
- the velocity change per tick is bounded at the latch, at a ring stop and at the decouple;
- a test per zone (F1);
- the decouple waits on the reference, not the measured body (F5);
- the ring frame is continuous across the latch (F9);
- the rig release guard (F7);
- LAND or a frozen clock stops the frame motion through the bound;
- the transit take-back ramps and decides from its stopping point;
- the hover path is c6f2032's (the existing tests at L615-645 pass unchanged);
- TB16: hover and orbit accepted in the twin, the other kinds refused, and orbit refused with `real:=true`;
- H3 races: a tilt fault during the co-move cancels it.

Then `gate.sh --quick`, the planner tests and the launch gate: switches off, only the TB16 refusals change the
accepted set. Then `prebuild_planner.py`.

If the co-move does not pass TB6 by the Thu gate, m1-comove stays out of the candidate. The registry keeps the
record; keeping the code needs Wesley's word.

## E0 / E1 results (exploratory, 10 Oct; rig twin, pickup world; registry R1475-R1490)

Tejen's mission now runs end to end on the rig twin: takeoff, ball pickup, high transit (lift 2.3 m, floor 2.7 m),
drop into the net (drop approach 1.35 m), rejoin (R1485 and every E1 run).

| run | arm | contact after his descent (s) | contacts | proof after contact (s) | switch | ring tilt peak contact..proof+8 s (deg) | dip (mm) | LAND |
|---|---|---|---|---|---|---|---|---|
| R1479 | E1b hover, seed ocp | 11.4 | 1 | 0.72 | same tick | 8.1 | 4 | abort (QUAD4 tipped in the landing unwind) |
| R1480 | E1b orbit, seed ocp | 13.0 | 1 | 0.90 | same tick | 8.4 | 4 | landed |
| R1489 | E1b hover, seed ocp | 13.7 | 2 | 0.86 | same tick | 7.7 | 2 | landed |
| R1490 | E1b orbit, seed ocp | 20.6 | 1 | 1.76 | same tick | 8.6 | 4 | landed |
| R1486 | E1b hover, seed start | 6.1 | 1 | 0.82 | same tick | 2.3 | 0.2 | abort (the same tip-over) |
| R1487 | E1b hover, seed start | 11.3 | 2 | 0.87 | same tick | 1.8 | 2 | landed |
| R1488 | E1b orbit, seed start | 11.6 | 2 | 2.89 | same tick | 2.6 | 1 | landed |

- The hand-over is bumpless in every run: QUAD4's throttle steps <= 0.3 % of hover at the switch, QUAD4 tilt at the
  proof <= 0.3 deg.
- Seed: 'ocp' (QUAD4 starts at its OCP share) let plate 9 sag 2.5-3.5 cm and tilted the ring 7.7-8.6 deg after the
  switch, 4 of 4; 'start' (0.1 N) 1.8-2.6 deg, 3 of 3. 'start' is the default on the branch (80b9352).
- `_nocatch` R1481: 3 contacts, 3 failed proofs, his give-up, our take-back out to 1.18 m, landed; no attach declared.
  `_land_transit` R1482: take-back at LAND with the ball on, bumpless, but it landed 0.33 m from carrier 1 (moving
  toward the ring when taken back; with the ball on it now steps out by the rod too, d226e85).
- The proof, measured (E1a R1483 orbit / R1484 hover, the lift held at its 1 cm cap): plate 9 rises 4.5-6.0 mm at the
  first lift's peak, which the carriers pull back to 0-3 mm in the hold; re-presses rise ~1 mm; tip drift when caught
  <= 1.7 mm (sd 0.1-0.3 mm). Plate-9 z noise near contact: the 0.1 s average's change over 0.3 s has sd 0.8-1.5 mm
  (p99 1.7-3.3 mm).
- **The threshold mapping cannot be met:** h_plate <= half the first-lift rise (2.2-3.0 mm) and >= 3x the noise
  (2.3-4.4 mm). With the start values (h_plate 3 mm) 3 of 7 first proofs failed by timeout on a real catch (drift
  0.5-2.5 mm, plate rise -3.9 to +0.9 mm), and the second contact proved each time; no false attach anywhere.
  The stop rule (rise < 2x noise) is not tripped. The thresholds and the twin cap go to Wesley before the claim.
- LAND: QUAD4 tipped over in the landing unwind in 2 of 7 (R1479, R1486; both hover): after touchdown its rigid rod
  lays over (55 -> 16 deg) and it tips past 60 deg in under 1 s; the rigid-link class of the 9 Oct LAND re-fly.
- Contact speed: 6-21 s after his descent start (the bar's 10 s met in 1 of 7): his height has no integral, so his
  hold sits a few mm above the plate point; the `_low` neighbour tests aiming lower.

## Critic

### Round 1 (rev 1; GO WITH CHANGES, rewrite), condensed
The ten findings and rev 2's response are kept in rev 2 of this card (git history). The ones still live:
- the release hold (closed: there is no release in this subtest, and `_attach_ocp` drops any departed entry);
- the lead and gates (lead 0, ground-ring gates);
- the OFF race (the manager arbitrates);
- RING_ATTACHED tracks until the mux reads ours;
- the no-weld exit (give-up, our abort, the WAIT_REWELD timeout);
- the bars (no band between pass and fail; counts);
- what a pass means (the rig twin is now the claim).

### Round 2 (rev 2 refolded onto the rig twin; REWRITE), condensed, with rev 3's response
A. **The proof used an absolute radius**, so in the twin a catch 5-8 mm off was never provable, and a sunk tip
   could pass in `_nocatch`. *Rev 3:* drift from the contact vector plus plate rise (his pattern), and the
   off-centre and sunk-tip tests.

B. **The rigid twin rod** means the body can only rise if plate 9 rises: 8 mm is about 2 deg of tilt, and his
   xy-only integral may not reach it. *Rev 3:* the lift is capped and stops at the proof; the thresholds come from
   E1 and are then frozen; the model integral is logged.

C. **His join mode has no takeoff and no LAND route.** *Rev 3:* Wesley chose his full pickup_delivery mission from
   the mat (it has WAIT_FOR_TAKEOFF and TAKEOFF); no attach means our tracker takes QUAD4 back and runs the tested
   abort.

D. **Plate numbering and geometry.** *Rev 3:*
   - his plate 3 = rig plate 9;
   - `geometry:=rig_twin` in his launch (ring 0.225, 0.55 at 60 deg, diameters 0.45 / 0.51);
   - the twin contact target from the plate top (+0.02) and the tip sphere (0.012);
   - the equality test on this world.

E. **His MPC on the rig-twin plant.** *Rev 3:*
   - thrust ratio 22 with feedback, cable_length 0.55, drop 0.59, vehicle mass 0.55;
   - the pendulum indices are unchanged: the same link order as the old model, verified;
   - the ON span waits for the airborne drone and the free spawn joint.

F. **His ground-ring exit was written for a static ring.** *Rev 3:* not used; the no-attach path is our tested abort.

G. **The hand-over touched the wrong state.** *Rev 3:* all four channels of `_applied_u` and the throttle from the
   mux output, the frozen states named, the re-seed, the heading datum, the fold-in seeded from the measured pull,
   and RING_ATTACHED holding the lifted offset.

H. **Magnet races.** *Rev 3:* the manager arbitrates; there is no ON before airborne and joint-free; and a
   "trajectory never held" bar (our planner hears its own ON). That bar joins the authority bars.

I. **Bars and order.** *Rev 3:*
   - the circular bars are dropped;
   - per-channel steps;
   - clearance >= 0.35 m;
   - a tilt window for runs without a confirmation;
   - the offset and LAND-during-proof neighbours;
   - one `_abort` mechanism;
   - the critic's order;
   - these rig-twin runs are the claim, with the reviewer before the rig.

### Round 3 (rev 3; GO WITH CHANGES), condensed, with rev 4's response
1. **The take-back cannot switch:** our approach never starts in partner runs, so the abort does nothing, and away
   from the ring our drone-3 tracker has no reference, so the mux (live takeover only) never switches. A late warm-up
   would hand QUAD4 a half-throttle spool step at 2.4 m. *Rev 4:* specified in §3 as new code (warm from his takeoff,
   spool done, fixed hold, departed path with the plate-9 rule, the ball's mass, his TAKEN_BACK), plus `_land_transit`.
2. **The clearance bar fails on geometry** (rod to rod 0.225 m at contact). *Rev 4:* attach_metrics pairs, the ball,
   the rod window ends at contact.
3. **The lifecycle couples the attach to a tracker that is not flying.** *Rev 4:* `_newcomer_arming_cb` gated on the
   mux; our span ON bypasses the ATTACH handler.
4. **E1-then-freeze is allowed** (detector parameters, frozen before the runs they score) but the mapping must be
   written first; the rigid rod may give ~3 mm of plate rise. *Rev 4:* the mapping, the noise floors from logs on
   disk, the stop rule, c0 averaged, the joint released on a failed proof; rig values from rig data.
5. **World and profile:** the ball's spawn weld (a 1 m lever, a one-shot release), the tip-ball collision, the ball
   only ~0.1 m over the carriers in the transit, the profile's legacy plate/azimuth defaults, the pendulum indices to
   confirm in E0. *Rev 4:* all applied; the indices are read from E0's bag.
6. **The rig's one coil:** the abort's OFF releases the ball before his drop. *Rev 4:* OFF only after the ON span has
   begun.
7. **R1474 and `_abort`.** *Rev 4:* R1474 cited; the pickup push world to build.

### Round 4 (rev 10: the take-back that moves with the ring, TB13, TB16): the challenge (critic, 11 Oct)
Verdict: GO WITH CHANGES. Build m1-comove once F1, F2, F5, F6's tests and F9's latch rule are written into §3b; fly
TB6 once F4's TB5 (10 deg sweep on a kinematic formation) has passed and F5's time bound is frozen; settle F3 and F8
before any rig circle. The response (applied / rejected, with Wesley's answers) follows below.

- **F1 (high) The far exemption keeps R1565's mechanism.** §3b calls a take-back "far" by its distance from the ring
  centre (>= 1.2 m, 1.79 m with the ball) and keeps it world-fixed, but the circle is centred at hover + (0, 0.5): QUAD4's
  mat is 0.90 m and his ball 1.49 m from the path centre against a swept edge of 1.2 (1.4 with QUAD4's rotor) m, so
  "transit, pickup and mat are far" is false on the circle; a near latch at the pickup would co-move and climb with the
  tip or ball still on the floor; a latch under the formation's footprint would rise into the ring; the orbit rule drops
  c6f2032's "only when the tip is down at the ring". Change: classify orbit take-backs by zone: (a) outside r_clear from
  the path centre (orbit radius + 1.0 m) or QUAD4's lowest point above the highest carrier + 0.15 m: world-fixed, today's
  path with TB13; (b) rotors below the ring and its net (body < ring z - 0.3 m: his takeoff and pickup): land where it is,
  no sideways move, no rise; (c) else: co-move; rise only when the tip is at the ring (his rejoin phases) or QUAD4 is
  >= 0.9 m from the ring centre; under the footprint, descend instead. A unit test per zone; TB5 adds three latches (his
  DESCEND_TO_PICKUP at the ring's nearest phase, his LIFT_OBJECT climb with the ball, his TRANSIT_TO_REATTACH descent);
  TB6 adds m1x_land_pickup_orbit (falsifiers: a sideways move with the tip within 5 cm of the floor, any rise, a TB4
  falsifier).
- **F2 (high) The escape height guards against a sweep the geometry rules out, and costs on the rig.** At the decouple
  QUAD4 is 1.2 m from the ring centre; the formation's edge plus QUAD4's rotor needs 0.9 m to touch it; the path-clear
  move runs straight out from the path centre and always increases the gap; the ring can close at <= 0.125 m/s only
  during ~1 s of brake (back-of-envelope: closest ring centre after the decouple ~1.09-1.2 m at every phase). In the ring
  frame the circle is the hover geometry, which passed 4 of 4. The extra 0.34 m buys a higher fall beside carriers 1-2
  (F6), a dangling magnet 0.15 m over a carrier body centre (not its props), unchecked mocap at 2.3 m toward +y, ~2.3 s
  more co-moving. Change: c6f2032's rise on the orbit too (delete the escape height); freeze a TB5 bar: after the
  decouple the ring centre stays >= 0.9 m from QUAD4 in xy at every phase of a 10 deg sweep; if a phase fails, the
  decouple waits and the ring-frame step continues along the corridor to r_clear - 0.1 m (a longer step, not a higher
  rise); B2 then unnecessary. If the escape height is kept: tip margin >= 0.25 m, overflight = rotor disc over rotor
  disc (centres < 0.4 m), a bar on the gap to the carrier's prop plane, only while the trajectory clock can advance.
- **F3 (high, rig) His takeoff against the circle.** In the twin his TAKEOFF fires at the start of our 5 s hover hold;
  on the rig it is typed by hand after "our planner flies the ring": typed after the hold, the ring centre can pass
  0.40 m from his mat (rim 0.175 m in xy at 1.0 m) while he climbs; his avoidance models drones as spheres, not the
  ring. Change: on the rig circle his TAKEOFF before the orbit starts (a rig traj_hold_s long enough, or the hold ends on
  his phase passing APPROACH_ABOVE_PICKUP); one twin neighbour with his takeoff 12 s into the orbit; measure the rig's
  mat and puck against the swept disc.
- **F4 (medium-high) TB5 cannot run as written; TB6's phase arms do not set the phase.** R1565's carriers are valid
  ~1.8 s after the give-up; the 1/4-3/4 lap shifts run past the data; traj_hold_s shifts the orbit start but the give-up
  time varies more. Change: TB5 on a kinematic formation (planned ring point + each carrier's ring-frame offset and lag,
  fitted on a clean lap, residual <= 2 cm); keep R1565's c6f2032 reproduction; sweep the give-up phase in 10 deg steps;
  each TB6 row records its give-up and decouple phase and compares its minimum clearance with TB5's prediction (a gap
  > 5 cm voids the replay for the gate); keep the phase arms, labelled "uncontrolled phase".
- **F5 (medium) "High and stuck" is not a bar; the decouple conditions are ambiguous.** "At the rise height" and "the
  step out reached its goal" must be REFERENCE conditions (p_hold within 0.02 m, as today); measured ones let a z offset
  hold off the decouple and the descent indefinitely. Change: write "reference"; a unit test with the measured z 5 cm
  under the reference; a TB4 falsifier frozen now: QUAD4 not "down" within 60 s of the take-back; under LAND skip the
  path-clear move once the step-out is done.
- **F6 (medium) The fault text is wrong for the co-move.** _tilt_fault_cb acts only while the mux reads partner or cut;
  once ours flies QUAD4 our tracker's envelope cuts the motors, then newcomer_fault_land lands the fleet (subtest) or an
  emergency stop (welded start); QUAD4 falls from the rise height 0.43 m in xy from carriers 1-2; tilt_cut is off on
  21-22 Oct. Any new co-move state must join the explicit lists in _tilt_fault_cb and _drop_departed. Change: rewrite the
  Faults bullet; an H3 test (no drone-3 reference after a tilt fault in the race window); optional TB6 arm MOCAP_LOSS on
  QUAD4 mid-rise on the circle (falsifier: anything but a fleet LAND with every carrier landed).
- **F7 (medium, rig only) A magnet that does not let go turns the rise into a ring lift.** After a false reject the
  take-back sends OFF, waits 0.5 s and rises 0.34-0.68 m; nothing checks the tip left the plate; the _nocatch arms never
  make the joint. Change (Wesley's call): gate the rise beyond the tether's slack on plate 9 not following (plate rise
  > 2 cm, or the tip not RELEASE_SEEN_M off the plate in the ring frame): hold in the ring frame, re-send OFF, LAND the
  fleet; a unit test.
- **F8 (medium) §6.3/§6.5 against the rules.** §6.5's split condition is consistent: keep it. The rig card §5.6 still
  scores the old path and its §7 keeps the removed two-fails rule: its take-back section needs the co-move path, the TB4
  bars, F5's time bound and a stop rule before R11a-circle. Add m1c_nocatch_hover_2 (TB13 changes every take-back).
  Wednesday: a wand walk at the rise height along the plate-9 corridor and around r_clear toward +y; the rig props' tip
  radius (sets the 0.2 m).
- **F9 (low) Values.** The 0.3 m/s^2 ramp is 1.75 deg quasi-static against a 2 deg bar (0.25 m/s^2 = 1.46 deg; constants
  are Wesley's); blend the decouple brake and the path-clear start into one bounded velocity change; compute the ring
  frame from the PLANNED ring point at the latch tick (a test that c(t) is continuous); report the ring's yaw change.
- **F10 (low) TB14.** "Tilt passing 60 deg within 1 s" needs a reference: "within 1 s after its tip first reaches the
  floor (tip z <= 0.03 m)"; claim_checks produces the count from the frozen definition (the hand list 12 of 48 must
  match).

### Round 4: the response (Wesley 11 Oct 14:30; decisions.md)
All ten findings applied; they change §3b, §5 and §6 as follows (written before any rev 10 run):
- F1 applied: the zone rule replaces the far exemption (a) far/high: today's path + TB13; (b) below the ring: land where
  it is; (c) else co-move, rising only when the tip is at the ring or QUAD4 is >= 0.9 m from the ring centre; under the
  footprint descend. Unit test per zone; TB5 + the three latches; TB6 + m1x_land_pickup_orbit with F1's falsifiers.
- F2 applied: no escape height; c6f2032's rise everywhere; TB5 bar frozen: after the decouple the ring centre stays
  >= 0.9 m from QUAD4 in xy at every phase of a 10 deg sweep, else the decouple waits and the ring-frame step continues
  to r_clear - 0.1 m.
- F3 applied (rig): his TAKEOFF before the orbit starts (sheet + rig traj_hold_s); twin neighbour m1x_late_takeoff_orbit
  (his takeoff 12 s into the orbit); the rig mat and puck measured against the swept disc.
- F4 applied: TB5 on a kinematic formation (fit residual <= 2 cm), R1565 reproduction kept, 10 deg give-up sweep; TB6
  rows record their give-up and decouple phase against TB5's prediction (a gap > 5 cm voids the replay for the gate);
  phase arms labelled "uncontrolled phase".
- F5 applied: reference conditions; a unit test with measured z 5 cm low; TB4 falsifier: QUAD4 not down within 60 s of
  the take-back; under LAND no path-clear move after the step-out.
- F6 applied: Faults bullet rewritten; H3 test (no drone-3 reference after a fault in the race window); new co-move
  state joins _tilt_fault_cb's and _drop_departed's lists; optional TB6 arm MOCAP_LOSS mid-rise on the circle.
- F7 applied (rig): the release guard before the rise, unit-tested.
- F8 applied: rig card §5.6/§7 rewritten with the co-move path, TB4 bars, the 60 s bound and a stop rule before
  R11a-circle; m1c_nocatch_hover_2 added to §6.3; Wednesday: the wand walk at the rise height and the props' tip radius.
- F9 applied: ramp 0.25 m/s^2; one bounded velocity change at the decouple; the ring frame from the planned ring point at
  the latch tick (continuity test); the ring's yaw change reported.
- F10 applied: TB14 referenced to the tip first reaching the floor (tip z <= 0.03 m); claim_checks produces the count.

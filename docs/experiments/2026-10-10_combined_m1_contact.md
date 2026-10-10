# Combined M1 subtest: Tejen flies QUAD4 from its mat through pickup, drop and a contact attach proved by a small lift; our check confirms; bumpless hand-over

Rev 6 (10 Oct 22:20, Wesley, after the first claim pass R1491-R1502 on e097a94): two fixes, then the whole claim
table is re-flown on the one final tree m1-final 7b8985a (= e097a94 + the full M1 + these fixes); the e097a94 rows stay
as the record. (1) His LIFT_PROOF rises `proof_rise_max` above the measured contact height, not above his target
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
    the measured position; the solver re-seeded; then the departed path:
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

## 6. Repeats and order (the critic's order)
1. Tests (below), including the proof fed synthetic caught, uncaught, off-centre and sunk-tip traces.
2. E1: one exploratory lift run on hover, the new code with loose thresholds and full logging (body rise, plate rise,
   drift, ring tilt, his throttle, the model integral). It sets d_ok / d_fail / h_plate, which are then frozen.
3. Claim runs on one tree, in order:
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

Between tweaks, one run on the arm that failed; a fix not shown in two runs: stop and report.

## 7. Cost
- E0 + E1: 2 runs.
- Claim: 13 runs.
- Lab PC whenever free (Wesley 10 Oct), ~15-20 min each.

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

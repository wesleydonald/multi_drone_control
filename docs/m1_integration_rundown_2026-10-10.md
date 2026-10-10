# Combined M1 with Tejen: what was tried, what changed in his controller and method, and real-life concerns

Written 10 Oct 2026 (overnight). Sources: docs/decisions.md, results/registry.csv (run ids in brackets),
src/tejen_mission/CHANGES_FROM_TEJEN.md, card docs/experiments/2026-10-10_combined_m1_contact.md.

## 1. The integration, in order

| when | what was tried | what came out |
|---|---|---|
| 25 Sep | His code forked (a7524bae) into our workspace. His M1 mission (`pickup_delivery`) flies our drone 3 in our old sim world; his nodes in `/tejen`, his MPC feeding our radio mux on `/drone_3/ELRSCommand_tejen`. | Plumbing fixes, all logged: his MPC cable length as a parameter, the pendulum publisher's rate spikes, ARM re-sent, the magnet command re-sent every 1 s (R0600-R0622). |
| 26 Sep | The first full M1 chain in sim: his controller held QUAD4 over the moving ring and waited for our weld. | The hold cycled for 220 s and a late weld at LAND tipped a drone to 87 deg (R0623). |
| 26 Sep | **The ATTACH_READY hand-back:** at his ready point (a few cm above the plate) the mux hands QUAD4 to our tracker, which flies the last descent and the weld. | First full M1 pass (R0624). Welding a still-moving drone tipped the ring 25-35 deg (R0640-R0645), so our weld only fires at rest, 0.05 m/s (R0646). |
| 27 Sep | Rod on universal joints; the rejoin fold-in ramps the newcomer in from 0.1 N; his MPC's thrust gain scheduled on the 0.1 kg ball (G1b). | M1 claims pass (R0685 / R0686: rejoin 1.6-1.7 deg); the combined M1 from a welded start passes (R0696 / R0697). |
| 28-29 Sep | Mux abort latch (an ESTOP or a fault grounds every drone, his included); the abort scope; quad identities; the gyro rate loop default. | Canonical M1 baseline R0752 (rejoin 2.7 deg). A combined RIG stack designed (docs/design/combined_stack_m1.md), not built. |
| 5-8 Oct | The rig twin becomes the simulation; partner worlds stay legacy. QUAD4 = `/drone_3`, M1 layout plates 3/11/7 + QUAD4 on 9; LAND / DISARM / fault authority rules. | Our own attach approach (R8a/R8b/R9) built and flown in the twin. |
| 10 Oct, day | His sync 1 (11a0683f): his marker bug fixed, which lowers his hand-back point by 12 cm. Our hand-back had regressed (R1470); fixed (R1473 PASS). | The port with today's hand-back missed the rejoin bar: 5.6 deg (R1474), because the weld now came 0.29 s after a hand-off from 12 cm lower. |
| 10 Oct, evening | **Your new roles:** Tejen's controller does every attach, down to contact; our motion-capture check alone confirms it; then we take QUAD4 and fold it in. Our own approach leaves the rig plan. | The contact-attach design (card), three critic rounds, the build (3 builders, all behind switches). |
| 10 Oct, night | His whole mission on the rig twin: a pickup world (open ring with a net, his ball), his launch with rig-twin geometry and his rig MPC values. | E0: takeoff, pickup, high transit, drop into the net and his rejoin all work after three fixes of mine: lift too low, drop point under the transit floor, our old descent's rod length (R1476 / R1477 / R1485). |
| 10 Oct, night | E1: contact, lift proof, our confirmation, the hand-over, the fold-in; the no-catch and take-back arms; the measured lift. | The attach works end to end on hover and circle; the hand-over is bumpless (throttle step <= 0.3 %). Seed A/B: QUAD4 starting at its full share tilts the ring 7.7-8.6 deg (4 of 4), starting at 0.1 N gives 1.8-2.6 deg (3 of 3) (R1479-R1490). No false attach (R1481). QUAD4's mocap force can't confirm within a 1 cm lift. |
| 10-11 Oct, night | The claim: 13 runs on one tree (R1491 on), then the reviewer. | Pending (morning report). |

## 2. What changed for Tejen

### His controller (`tejen_mpc`)
- **The control law is unchanged.** These became parameters, with his defaults kept:
  - pose time-out;
  - thrust ratio;
  - rate limits;
  - cable length.
- **Ball gain schedule** (27 Sep): while the ball is attached, his thrust gain is scaled by `m/(m + 0.1 kg)`, instead of waiting for his UKF.
- **On the rig twin** he flies with his rig values: thrust ratio 22 with his thrust-ratio feedback, rod 0.55 m, vehicle 0.55 kg.
- **Observed:** with no height integral, a thrust-ratio mismatch shows up as a steady height error. His loaded lift ended 14 cm low while his estimate converged (R1476).

### His mission and planner (`online_join_planner`, `mission_definitions`), all behind `contact_attach` (off = his flow)

| before (his flow / our 26 Sep hand-back) | now (contact attach) |
|---|---|
| Descends to a ready point 5 cm above the plate (it was ~18 cm before his marker fix), raises `handoff_ready`, and our tracker flies the last descent and our weld. | Descends to **contact** (the tip on the plate point in the twin, 1 cm below the face-on-plate height on the rig) and holds there with no hand-back flag. |
| His hold leads the moving plate by 0.30 s. | Lead 0 (the 0.30 s lead is a neighbour run). |
| Loose gates in partner mode: descends within 18 cm, aborts beyond 25 cm. | His own ground-ring gates for the rejoin: within 2.5 cm and 0.08 m/s to descend, abort beyond 5 cm or 0.15 m/s. His drop keeps its own gates. |
| Magnet ON for the rejoin descent (our 25 Sep edit). | Ring magnet ON from his rejoin start; on the rig the only OFF on that coil is his ball drop. |
| — | **LIFT_PROOF** (new): on our contact signal he rises 2 cm/s, at most 1 cm (twin; rig up to 3 cm), and holds. Our check decides. A failed proof: he re-presses from where the lift ended. 3 failures or 30 s: he gives up. |
| — | **RING_ATTACHED** (new, terminal): on our confirmation he holds plate-relative until the mux reads ours, then holds. No restart after our LAND (it used to restart his approach after LANDED, R1473). |
| — | **TAKEN_BACK** (new, terminal): our tracker took QUAD4 (his give-up, or our LAND before the attach). The ball stays on. |
| His target uses the ring's yaw only. | `ring_bridge` (ours, in his package) gives him the plate point from the ring's full attitude, equal to our check's point. |
| The ball's spawn weld is freed by one gz message. | His supervisor repeats the release until the joint reads free, and holds his takeoff until then. |

The rest of his mission is unchanged: takeoff, pickup with his proof, C++ transit with obstacle avoidance, drop.

### On our side, for the same flow
- **Our check** (`confirm_mode contact_lift`):
  - **Contact:** the tip within 1 cm of the plate point and still against it, for 0.15 s.
  - **Proof:** while he lifts, the tip stays within 3 mm of where it touched and plate 9 rises 2.5 mm, for 0.3 s. On the rig, the body-to-tip length must also reach the taut length.
  - **Then the attach is confirmed.** All timers run on the simulation clock.
- **Our tracker for QUAD4:**
  - warm from his takeoff, following the commands actually sent;
  - its integrators frozen;
  - re-seeded at the switch.
- **The mux** switches only on our confirmation or a take-back.
- **The fold-in** starts QUAD4 at 0.1 N.
- **The take-back:** QUAD4 steps 1.2 m clear of the ring (plus the rod's length while it carries the ball) and lands alone.

## 3. Real life vs the twin: concerns, most important first

1. **The tether.** The rig's tether goes slack; the twin's rod is rigid.
   - The proof on the rig must see the tether go taut (body-to-tip at the taut length, measured Wednesday 09:00) before a plate rise means anything. The twin cannot test that part.
   - The 1 cm overtravel relies on the slack.
   - The twin's landing tip-over of QUAD4 (2 of 7) should not exist on the rig.
2. **The magnet.** It catches only on contact.
   - The plate is ~6 cm across; an off-centre or tilted catch may be weak. Holding force is ~10-12 N, and it releases under jerks (30 Sep).
   - The twin's catch is a perfect joint at the contact.
   - The 09:00 check measures the catch distance; the proof's lift is also a pull test of the catch.
3. **One coil for the ball and the ring.**
   - His drop is the only OFF; the ring span then holds it ON.
   - A take-back before the drop must not switch it off, or the ball drops wherever QUAD4 is. Built; to verify on the rig desk.
   - On the rig his magnet command and ours must go through one gate (the rig port).
4. **His height accuracy.**
   - No height integral, so his hold sits a few mm (twin) or cm (loaded, R1476) off the target. Contact came 6-21 s after his descent start in the twin; the 10 s bar was met once in 7.
   - On the rig, with the real thrust map, pack sag and his kT estimate converging, expect a slower or offset contact.
   - The 1 cm overtravel helps.
5. **Prop wash.** QUAD4 descends onto plate 9 a few cm above the ring, and its downwash pushes on the ring and the plate. Gazebo has no aerodynamic interaction.
   - Expect a ring dip and tilt at plate 9 during his descent.
   - The carriers' measured pull will feel it.
   - It may also blow the tether.
6. **Proof thresholds** come from the twin (1 cm lift, 2.5 mm plate rise). On the rig:
   - plate-9 noise, tip marker noise and a hand lift's plate rise need measuring;
   - the force on QUAD4 is too noisy to help within a small lift (rig noise 0.11-0.35 N against a <0.06 N signal).
7. **Motion capture near the ring.**
   - Body 24 (the tip) is small and next to the plate markers and the ring body. Occlusion or marker swaps at contact would fake or kill a contact.
   - The 09:00 survey and the tip record measure this.
8. **Flight time.**
   - The twin reaches the attach at ~125 s after takeoff, plus the fold-in and LAND, with the carriers holding the ring at high throttle the whole time. 8 Oct flights were ~1 min.
   - Check pack voltage under the ring for 3 min before trying the full sequence (stop rule 23.6 V).
9. **Height and space.**
   - The transit floor (2.7 m) puts QUAD4's body at ~3 m in the 4 m cage, with the ball hanging 0.6-0.7 m under it.
   - Check the motion-capture volume up there.
   - His drop descends inside the ring between carriers ~0.5 m away.
10. **The combined stack on one laptop:**
    - our planner and 4 trackers (QUAD4's warm all flight), his MPC, his C++ planner, the mocap node;
    - CPU starvation caused pose-timeout disarms on 9 Oct (laptop twin);
    - his MPC self-disarms on a stale pose, which over the ring means QUAD4 falls onto it.

    Load-test the stack on the rig laptop at the desk (Tuesday).
11. **The radio and the switch.**
    - QUAD4's one ELRS link is shared through the mux: his stream until our confirmation, ours after.
    - The switch is bumpless in the twin. On the rig the radio latency and Betaflight's own rate loop differ.
    - The props-off test checks the switch.
12. **Moving ring.** The twin attaches on a 0.125 m/s circle (R1488). On the rig his real lag and the prop wash add up. Hover first.

## 4. For Tuesday's meeting (ideas)
- **The video of the subtest:** his whole mission in the rig twin (takeoff, ball pickup, transit, drop into the ring, contact, lift proof, hand-over, all four carrying). Videos in results/videos/.
- **The method on one example:** the seed A/B. A card with bars set before the runs, a critic, exploratory runs, a fix: 8 deg -> 2 deg, 4 of 4 vs 3 of 3.
- **Safety by design:**
  - the no-catch run, where the proof cannot be fooled;
  - the take-back, which lands QUAD4 clear of the ring with the ball still on.
- **Rig evidence:**
  - four detaches on 8 Oct (6.5-17.8 deg);
  - the measured-pull hover (7 cm -> 1 cm);
  - the twin-rig parity figure.
- **A negative result worth saying:** QUAD4's force can't confirm the attach in a small lift (signal < 0.06 N, rig noise 0.1-0.35 N), so the confirmation is geometric (contact + lift).
- **Next week:** Wednesday detaches plus the props-off contact-lift check on the real magnet; Thursday Tejen alone; the combined M1 on 21-22 Oct.

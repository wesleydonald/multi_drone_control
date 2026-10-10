# Morning report, Sun 11 Oct (overnight 10-11 Oct)

**Verdict:** the full M1 now flies end to end in the twin, three times: hover R1517 and R1530, circle R1523. The video is
ready. Getting there took five fixes in Tejen's mission flow, and each one was traced to a cause first. The rig port is
built, desk-tested and through two critic rounds. Its critic rules Wednesday's props-off check GO WITH CHANGES and the
21-22 Oct flight NO-GO until a short list is done. The final claim table is flying now on the one tree that will also fly
on the rig.

## 1. Rig findings first (from twin runs, the desk test and two critics)

| # | Finding | Why it matters on the rig | Status |
|---|---|---|---|
| 1 | **Nothing automatic stops a tipped QUAD4 while Tejen's controller flies it.** Our tracker sees the tilt, but the mux doesn't forward its disarm. Betaflight has no tilt cutoff. His MPC only disarms on a stale pose. | The only stop on Wednesday would be a full DISARM, which drops the ring with the carriers. | A tilt cut is built behind switches that default off. Critic round 2: GO WITH CHANGES. It still has to move onto the final tree. **Sheet line for you:** "after LANDED, if drone 4's mux reads partner, press DISARM". |
| 2 | **The contact latches while the magnet is still arriving.** Desk test at 3, 5 and 8 cm/s: the contact offset is 7.4, 2.9 and 2.1 mm off, and at 3 cm/s the proof fails. The twin showed the same (R1522: drift 3.4-3.8 mm against the 3 mm limit). | Real catches would be rejected, then he gives up after 3 tries. | Fix built: `contact_still`, which takes the offset from the end of the dwell and adds a vertical-speed gate. It brings all three cases within 0.2 mm. It stays OFF until you say: **decision 1**. |
| 3 | **Tejen's MPC stops learning its thrust ratio below the height where it armed.** The gate is 0.2 m above the arm height, checked every tick. | If QUAD4 is ever handed to him in the air (the full M1), he flies 3-11 cm low (R1503: 11 cm, the tip went into the floor). It doesn't matter when he arms on the mat. | Fixed in our launch of his stack (gate -10 m when released in the air). **Tell Tejen.** |
| 4 | **One coil and the puck in the net.** In the twin, a take-back swept QUAD4's tip past the ball lying in the net with his magnet still on. It caught the ball, dragged the ring, and the fleet aborted (R1520). | On the rig the coil is shared. Our take-back's OFF releases it once our span has begun, and the rig gate passes his OFF only in the drop phases. | Fixed: his take-back after the drop holds his magnet OFF. |
| 5 | **His rig values are QUAD2's, not QUAD4's**: marker-to-face, kT 22 and drop 0.575. The puck is 60 g, not 100 g. | His height and contact target would be off. | Measure Wednesday and Thursday. The rig launch now refuses these values unless they are typed. |
| 6 | **The whole combined graph has never run at wall-clock speed on one laptop.** That's four trackers, our planner and his whole stack. | If his command stream goes quiet for 0.5 s, the radio disarms QUAD4 mid-air. | Wednesday props-off load test with frozen bars (critic F1). |
| 7 | **A take-back from plate height must rise before it steps out.** R1543: QUAD4 was taken back with its tip at the ring. It stepped out sideways and the rod hooked the rim. | On the rig the tether is slack and the magnet is OFF, but a magnet sweeping across the plate edge can still catch it. | Fixed (ae57bbb): it rises to ring height + rod + 0.15 m first. |

## 2. Overnight results (twin, rig twin, one row each in results/registry.csv)

| Tree | What changed | Runs | Result |
|---|---|---|---|
| e097a94 (claim pass 1) | the contact attach as designed | R1491-R1502 | Hover x2, no-catch x2, LAND in transit, offset, push abort: pass. **Target 1 cm low fails**: the proof lift counted from the target. 3 arms never reached the attach because his pickup stalled. |
| 7b8985a | lift from the measured contact, first pickup retry | R1505-R1508 | Low and orbit pass. **Pickup still stalls**: the ball welds, then his descent pushes on and the rigid rod buckles. R1509-R1514 void (batch stopped for the fix). |
| 6143989 | descent ends on the weld, retry when attached but misaligned, airborne kT | R1515-R1530 | **Full M1 end to end: R1517 hover, R1523 circle, R1530 recorded.** Hover x2, orbit, low, high, offset, no-catch hover, LAND in transit, LAND during the proof and the push abort pass the claim bars. **Two fail**: no-catch orbit (R1520, ball re-catch) and orbit repeat (R1522, the contact latch plus a lift goal that was too small). 0.3 s lead fails (R1525, plate bounce: keep lead 0). |
| m1-rig d4248af | + take-back magnet OFF, lift goal max(target, measured), the rig port | R1531-R1549 | **12 of 13 claim arms pass**: hover x2, orbit x2, low, high, offset, no-catch hover and orbit, LAND in transit, LAND during the proof, the push abort. The R1520/R1522 failures are gone; proofs 0.63-1.42 s. **Full M1 clean on hover and circle** (R1536, R1541: proof 0.7-1.0 s, ring tilt at most 4.2 deg through the attach). `contact_still` ON: clean in every run so far (R1533, R1540, ...). **The 0.3 s lead fails** (R1543): no contact, then the take-back from plate height hooked the rod on the rim (ring 22 deg, QUAD4 54 deg, fault LAND, all landed). |
| m1-rig ae57bbb (final) | + the take-back rises clear of the ring first | queued | The claim is re-flown on this tree, then the tilt-cut twin arms. |

Landing: QUAD4 tipped in the landing unwind in R1518, R1529, R1531, R1534 and R1547, always about 7 s into LAND with its body 0.2-0.3 m up: the rigid-twin artefact you ruled scored separately. It is now 5 of the last 11 attached runs. That is consistent enough to look at once before Wednesday's four-drone landings.

**Watch:** proof time is 0.7-2.9 s on 6143989, against the frozen 3 s limit. Slow proofs are QUAD4's MPC barely
lifting for about 2 s under the ring's load.

## 3. Built and committed (branches; nothing pushed, nothing merged to week4)

| Branch / commit | What |
|---|---|
| m1-final d8c1b65 | Tejen-side fixes: proof lift, pickup hold and retry, take-back magnet OFF, airborne kT, full M1 (welded start, mux ours from boot, WAIT_PICKUP_FREE). Every change is logged in CHANGES_FROM_TEJEN.md. |
| m1-rig d4248af / ead6afe | The rig port merged with m1-final, so this is the one tree for the claim and the rig. It adds the coil gate (single writer), aux zeroing, DDS reachability checks, his throttle held at idle until we fly, his OFF only in the drop phases, `contact_still` (off), the QUAD4 record refusals, and drafts of the 21-22 Oct rig card and Wednesday's combined props-off items. |
| m1-tiltcut 5c50fa1 | The tilt cut and partner-flight fault handling: 6 switches, all off, 19 test arms. Still to do: rebase onto m1-rig per critic round 2. |
| m1-rec 9e3cd00 / m1-video | `run_experiment --record`, a Gazebo-rendered video of any headless run |
| rig-prep | Registry rows, card rev 7, decisions, session state |

## 4. Video for Tuesday

- `results/videos/full_M1_twin_R1530_edit.mp4`: 2 min 35 s, real Gazebo footage at 1x, captioned. Wide shot for the start, pickup and transit; ring shot for the drop; plate-9 close-up for the contact, proof, attach and landing.
- Raw cameras: `full_M1_twin_R1530_{wide,ring,plate9}.mp4`.
- Other ideas for Tuesday (docs/m1_integration_rundown_2026-10-10.md §4):
  - the card, critic and twin loop on a real example: the pickup stall, traced from a wrong first guess to the real cause;
  - the no-catch take-back;
  - the 8 Oct rig detaches;
  - the force check as a negative result.

## 5. Decisions for you

1. **`contact_still` on the rig.** It moves the contact offset from 2-7 mm to within 0.2 mm, and the twin evidence (m1x_still_*) is flying now. Thresholds are yours: dwell tail 0.05 s and vertical speed 0.02 m/s.
2. **Wednesday sheet additions** (drafts, not in the sheet):
   - the combined props-off items (docs/rig_2026-10-14_combined_props_off_draft.md on m1-rig; they use a second install space, so run them last);
   - the DISARM-after-LANDED line;
   - the tilt-cut measurements that need no code: bag QUAD4 at transit height, a hand tumble, DISARM timing, the RX-off blind spot, the coil under a disarm, the FC attitude against mocap.
3. **The 21-22 Oct flight list** (rig-port critic). In order: the reviewer on the m1-rig table, his QUAD4 values measured, the load bars, a first flight that cannot catch (`contact_dz 0.03`), then contact runs.
4. **The tilt cut:** its twin arms run on the lab after the claim. Which switches become defaults afterwards is your call, and Tejen should hear before Wednesday.
5. **The take-back attitude step** is 4.5 deg in 0.3 s, between the bars of 2 and 5. A ramped hold is proposed.
6. **Tell Tejen:**
   - his kT gate;
   - the coil gate and his OFF rule;
   - his supervisor's stop now gives up instead of landing;
   - his QUAD2 values on QUAD4;
   - the tilt cut.

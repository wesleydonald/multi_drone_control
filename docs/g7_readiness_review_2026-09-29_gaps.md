# What the draft risk register and ladder are missing

I grepped every gap below against tests.txt, the ladder, the card, CURRENT_STATE, decisions, GOALS, THESIS_PLAN, tools/ and src/, and against Tejen's tree where it matters. A search that returned nothing means the repo has nothing on that topic. **[E]** means I read it in code, logs or docs. **[I]** means I inferred it. Nothing was edited or launched.

## A. Human safety and cage procedure

**1. No rule for when people may enter the cage, and no staging for putting props on [E]**
- **Evidence:**
  - Searches for cage-clear, call-outs, safety glasses or goggles, hand-catching, first aid and fire bags return nothing.
  - The only human rule is "anyone inside the cage" as an ESTOP trigger (tests.txt:17-19).
  - In R3a/R3b the drones are placed on the plates with packs in and T1 up, because `magnet_initial:=ON` only latches once the radio node runs (elrs_interface.py:153-156). With props on, that puts people next to live, radio-linked airframes.
  - The big panel button toggles ARM based on feedback (arm_panel.cpp:231-235). Only the spacebar always disarms (:237-241).
- **Why it matters:** placing tethered drones on the ring is the most hands-on step in every carry rung, and it happens with the link up.
- **What to add:**
  - A safe-to-enter state: T2 down, or every drone DISARMED on both the panel and the telemetry mode.
  - Unplug packs before handling tethered drones.
  - Props on last, with packs out.
  - ARM called aloud with the door shut.
  - Never hand-catch a drone. Eye protection.
- **Rung:** written at R0a; first applies at R1/R3a.

**2. "R2 in parallel with R0b" clashes with R0b [E]**
- **Evidence:**
  - tests.txt:91 schedules R2 alongside R0b, and the ladder draft keeps that.
  - R0b step 3 toggles each drone's magnet on the same T1 (tests.txt:78).
  - Step 4 needs the drones sitting on plates 1/5/9 for the disc fit (:82-86).
  - Step 5 ARMs and sends TAKEOFF, which spins the motors (:87-88).
- **Why it matters:** a second person pulling on a magnet gets it switched off mid-pull, the moved drones break preflight geometry, and motors spin up in someone's hands.
- **Fix:** do R2 between steps 3 and 4 with T2 down, or on airframes that are not in the R0b graph.
- **Rung:** R0b/R2.

**3. LiPo handling, pack IDs and a pack budget [E]**
- **Evidence:**
  - Nothing in the repo on LiPo, fire, puffed packs or storage charge.
  - Drone 1's pack went from 21.8 to 16.9 V (about 2.8 V/cell) on 09-23. That pack is probably damaged, and nobody can tell which pack it was: neither tests.txt nor the card's results table has a pack-ID column.
- **Budget [I]:** the Wednesday plan uses about 19 pack-uses for R1 through R3b (4 + 6 + 3 + 6), plus 8 for R4. Every flight needs at least 24.0 V at rest, so every flight needs a fresh pack.
- **What to add:** label the packs; log pack → drone → flight together with start/end V and mAh (`battery_mah_used` is already published); quarantine the 16.9 V pack; charge supervised in a bag; inspect packs after a drop; storage-charge at the end of the day.
- **Rung:** R0a (count packs and chargers), then every flight.

## B. Identification and calibration

**4. The Motive body heading is never aligned with the flight controller's forward axis [E]**
- **Evidence:**
  - The mocap node applies no per-body rotation.
  - preflight checks only resting tilt (preflight.py:194-204).
  - tests.txt:46 re-creates each body "LEVEL" and says nothing about which way the nose points.
- **Why it matters [I]:** a body created with the nose off Motive +x rotates every roll/pitch command by that yaw error. The draft's K11 covers flips and tilt, not heading.
- **Cheap check that already exists:** elrs_interface decodes the flight controller's CRSF ATTITUDE frame and prints it to T1 (elrs_interface.py:252-256). With props off, tilt each airframe nose-down and then right-down by hand, and compare the FC's pitch and roll with mocap.
- **Rung:** R0b, after any body re-creation and before R1.

**5. No check of motor order or prop direction after a repair [E]**
- **Evidence:** nothing in the repo.
- **Assessment [I]:** unlikely to matter today, since these are Tejen's quads and have flown. It belongs in the post-crash checklist (item 22).

**6. No check of Motive's state at the start of each visit [E]**
- **Evidence:**
  - Nothing on calibration quality, ground plane or wanding in the rig docs.
  - preflight only bounds the payload's resting tilt at < 25 deg (preflight.py:273-274) and does not record it as a zero.
- **Why it matters:** the card's bar is a settled ring tilt under 5 deg. A 1-2 deg ground-plane or body-definition offset uses up part of that, and another lab user can recalibrate Motive between visits.
- **What to record:** the Motive calibration result, the ring's resting tilt and z (this becomes both `payload_rest_z` and the tilt zero), and the camera count.
- **Possible issue [I]:** shiny steel plates may produce stray markers.
- **Rung:** R0b, every visit.

**7. No equipment list [E]**
- **Evidence:** a search for spares returns nothing, and the spring scale is still a TODO (tests.txt:92).
- **What is needed:** a mass scale (three of them for the ring's centre of mass), the spring scale, a tape measure, a feeler gauge, spares (props, rods, magnets, markers), phone mounts.
- **Rung:** R0a.

## C. Magnets and tethers

**8. R2 does not test the magnet under realistic conditions [E evidence, I risk]**
- **Evidence:**
  - R2 is a centred, static, axial pull (tests.txt:92-95).
  - Tejen's rig evidence:
    - "first two pickup attempts failed physical lift, third succeeded with better magnet centering" (tejen CODEX_HANDOFF_M2_IRL_20260924.md:512);
    - "attachment lost during loaded lift" in 4 of 5 and then 3 of 4 lifts on 09-18 (experiment-history.md:702, :713), with a 0.060 kg object (his irl_commissioning.yaml:21).
  - These are electromagnets (CURRENT_STATE.md:29), switched ON from T1 launch through preflight. Nothing in the repo covers coil current or heating.
- **What to add:** hold force at the worst plausible placement offset and at `weld_radius`, and hold after the coil has been ON for the usual pre-flight time.
- **Rung:** R2.

**9. Snagging, a dangling rod, and the pivot lever [E/I]**
- **Evidence:**
  - Searches for snag and tangle return nothing.
  - Tejen's config puts the tether anchor 4 cm below the body origin: `tether_anchor_body: [0,0,-0.04]` (m2_irl_two_drone.yaml:26). That is the 0.04-0.05 m lever that flipped the newcomer in sim (decisions.md:33-36). The draft calls the hook offset "unmeasured", but this is the one number on file and it sits in the flip range.
- **Cases the drafts do not cover [I]:**
  - After an R6 detach, the freed drone flies with its rod and magnet hanging.
  - Landing with a rigid 0.47 m rod: the tip touches first and can tip the airframe or swing the rod up.
  - Rods catching a neighbour's landing gear at the floor start.
  - A tether caught in the net.
  - The magnet wire has to cross the pivot. That adds joint stiffness the sim's frictionless joints lack, and a broken wire drops that drone's share.
- **Rungs:** R1 with rods on (first dangling-rod landing), R2 (pivot sweep with the wire, measure the hook offset), R6 and R8b.

## D. The dropped ring and abort choice

**10. What happens when the fleet disarms at height, and what to do next [E/I]**
- **Evidence:**
  - Fleet-wide disarm is a locked decision. THESIS_PLAN §5.1's "descend together, then disarm" is not what shipped.
  - The magnet latch stays in the idle packet (elrs_interface.py:143-146). So while T1 runs, the drones fall still tethered to the ring: the ring from about 0.6 m, the drones from about 0.9 m.
- **Missing:**
  - A drop procedure: wait for DISARMED on the panel and in telemetry; unplug packs; free the rods; inspect props, arms, rods, magnets, markers and packs; check the Motive bodies; one short free hover before the next tethered flight.
  - A fall-height budget. The draft only offers `target_z` 0.35 as an option.
  - A decision on floor mats before `payload_rest_z` is measured, because mats change the rest height and the creep friction.
- **Rung:** decide at R0a; applies from R3b0.

**11. No rule for LAND versus ESTOP [E]**
- **Evidence:** every global abort trigger is ESTOP (tests.txt:17-19). LAND rules appear only in scattered places (:124, :141-142).
- **Why it matters:** slow problems (a pinned throttle, drift, sag, the ring creeping above 0.7 m) have a controlled way out, but ESTOP drops everything.
- **Rung:** R0a (sheet).

## E. Radio and laptop hardware

**12. TX modules and the laptop are not physically secured [E]**
- **Evidence:**
  - Tejen's rig had a run where the drone "fell nearly ballistically despite the MPC still commanding high/max thrust ... they nudged the laptop" (tejen handoff:514).
  - On a serial error our radio node blocks in a reconnect loop inside its timer (elrs_interface.py:333-337, :175-194). That drone falls to Betaflight failsafe while the others keep carrying (draft K10).
  - Searches for hub, cabling and nudge in our docs return nothing.
- **What to add:** strain-relieve the TX modules and hub; put the laptop on a fixed table outside the cage; nobody touches it in flight.
- **Rung:** R0b.

## F. Recording and comparison with the sim twin

**13. Nothing is backed up off the laptop [E]**
- **Evidence:**
  - These are gitignored and local-only since 09-16: results/, results_archive/, general/, docs/, CURRENT_STATE.md, configs/, tools/ (.gitignore:28-32, :48-49).
  - `backup_results.sh` copies only results/ and results_archive/ (:22), and exits 2 when `MDC_BACKUP_DEST` is unset (:49-58).
  - `MDC_BACKUP_DEST` is not set in ~/.bashrc or ~/.profile. The last logged backup was 2026-08-05, to ~/thesis_backups, which is the same disk (THESIS_PLAN:259). There is no crontab.
- **Consequence:** the 09-16/09-23 rig logs, the ladder, the card, the tools and the configs exist only on a laptop that travels to the lab. THESIS_PLAN:264 ("hardware sessions get archived in full") has no working path.
- **Rung:** before Wednesday.

**14. No record of which code was flown [E]**
- **Evidence:**
  - Rig tracker directories hold only log.csv and params.json (planner_drone0_20260923_*): no git SHA, no dirty state.
  - tools/, configs/ and docs/ are untracked, so a SHA alone would not pin them anyway.
- **Why it matters:** E9 ("every HW config paired with an identical sim config", THESIS_PLAN:926) needs it.
- **What to add:** one sheet line per flight: HEAD, `git status --short`, and a tarball or hash of tools/ and configs/.

**15. Rig flights cannot be scored by metrics.py [E]**
- **Evidence:**
  - `metrics.load_run` reads only logs/sil.csv or logs/run.csv plus events.csv (metrics.py:649-681).
  - Rig flights produce per-node log.csv files, a bag and the T2 tee.
  - The bag list (tests.txt:68) lacks topics the runner uses (runner_node.py:99-129): /rosout (for events), /magnet/object_attached, /payload/desired_position.
  - THESIS_PLAN:885-888 requires "tools/metrics.py runs before you leave".
- **What is needed:** a tool that turns logs plus the bag into a run directory (manifest, run.csv, events.csv), and the missing topics in the bag.
- **Rung:** R0a (tool), before the first R3b claim.

**16. No video, no Betaflight blackbox, no Motive take [E]**
- **Evidence:** searches for video, blackbox and .tak find only the sim demo.
- **Why it matters:**
  - Two rig problems are still undiagnosed from logs alone: the 09-16 drone-1 jump and drone 0 sitting on the floor while commanded (4 launches). Tether coupling has never been confirmed by eye.
  - Blackbox motor outputs would show motor saturation that the 0.6 command cap hides. This bears directly on the throttle-headroom risk (K1). Check whether the FCs have blackbox flash in the `diff all`.
  - A .tak take records raw markers independently of the unknown forwarder. That answers flip vs occlusion vs forwarder (K11, K12, K17) after the fact.
- **Also:** add an "observations" column to the card's results table.
- **Rung:** R0b onward.

**17. The sim twin is not re-flown with the rig's numbers [E]**
- **Evidence:**
  - The card's twins fly kT 83.1, rod 0.50, drones 0.64 kg and the even 0/120/240 ring.
  - The draft's R0a' is pre-visit only.
  - The rule "the exact config passed in Gazebo within 3 days" (THESIS_PLAN:885-886) cannot be met for real_control until P5b (ladder:405), which is optional and not done.
- **What to add:** after each visit, one twin re-flown on the weighed masses, measured rod, measured kT and latency, and the exact plates.

**18. Two claim flights or three? [E]**
- **Evidence:** THESIS_PLAN:483 and metrics.py:18 say at least 3 real repeats; card §6 uses 2 per arm. the working rules' "fly its neighbours once" rule has no rig version in the card.
- **Needs:** Wesley's word before the R3b claims.

## G. Time left

**19. The drafts never map rungs to lab visits or say when to descope [E numbers, I mapping]**
- **Evidence:**
  - Real-flight logs exist only for 09-16 (95 dirs) and 09-23 (29 dirs), both Wednesdays. That is one lab day a week, against "about 2 days a week" (CURRENT_STATE:678) and THESIS_PLAN's ~25 lab days.
  - The freeze date conflicts: THESIS_PLAN:953 has the results freeze in W13 (Oct 27-31); CURRENT_STATE:668 says Fri 7 Nov.
  - That leaves 6 Wednesdays: Sep 30, Oct 7, 14, 21, 28 and Nov 4.
- **The squeeze [I]:**
  - The ladder has about 28 rungs, and its own Wednesday target stops at R3a.
  - The path to CURRENT_STATE's descope floor, "hardware carry and detach" (:681-685), is R3b → R4 → R6a0 → R6: at least two more visits if nothing fails.
  - The combined rungs R0c, R10 and M2b-M2d depend on Tejen-side pieces that do not exist yet, and on his time (THESIS_PLAN:903: "H5 is the hidden schedule risk").
- **Needs:** a table of visit → target rungs → which go/no-go triggers which descope step, and a reconciliation of G7 (full M1 and M2 on the rig) with that floor. Wesley decides.

## H. Physical pieces M1 needs on the rig

**20. The ball, the net and the pickup stand [E]**
- **Evidence:**
  - G1 is "picks the steel ball up, drops it into the net on the moving ring" (GOALS.md:12-14).
  - The ladder and tests.txt never mention the ball, a net or a basket.
  - A net on the real ring changes `load_mass`, the inertia (so the planner needs a prebuild) and marker occlusion.
  - The real object has mass (0.060 kg in his IRL config; about 0.1 kg for G1b), and G1b stalls in sim (R0681, GOALS.md:667-671). R11 with a real object has no passing twin.
- **Rung:** before R10b/R11. Wesley also decides whether R11 uses a virtual drop, as Tejen's R10a does.

## I. A mitigation the drafts miss, and two contradictions

**21. Even four-drone ring as the first lift [E/I]**
- **Contradiction:** the register's K1 says that if the throttle prediction is high, fly R4 (plates 1/3/5/9) before R3b. The ladder's own §0.1 says the plate-9 drone in that layout carries 39 % of the ring and needs about 1.12 kg.
- **The better option:** an even four-drone ring gives the most headroom (about 0.61-0.63 predicted throttle at 0.64 kg, against 0.71-0.74). It also avoids the uneven-ring rock seen in SIL (decisions.md:30).
- **Missing twin:** `configs/experiments/ocp_hover_ground_creep_n4.yaml` exists, but its only run, R0468, was falsified on the old plant. It needs one Gazebo re-fly before it can be a conditional rung between R3a and R3b.

**22. Tejen's loaded kT goes the other way from the draft's inference [E evidence, I meaning]**
- **Evidence:** his UKF adapted kT to about 18-20 while carrying a 0.060 kg object with 22 typed (experiment-history.md:702-704, :713).
- **Contradiction:** K7 infers that the loaded kT sits 10-20 % *above* the free one.
- **If his number reflects the airframe:** headroom is worse than the §0.1 table says, so the table should carry both cases.
- **Caveat:** his quad was body 7 in those runs, which may be a different airframe (notes_for_tejen item 10).
- **Rung:** the go/no-go arithmetic at R0a/R0b'.

## J. Recovery procedures

**23. Only the software reset exists [E]**
- **Evidence:** tests.txt and the drafts cover clean_slate and relaunch, and nothing else.
- **What is missing, beyond item 10:**
  - a crash into the net;
  - one drone losing its link mid-carry;
  - a Motive or forwarder restart (bodies can be lost or renumbered; relaunch T1 then T2 and re-run preflight);
  - a laptop freeze;
  - a flight controller left in CLI mode by `fc_probe.py cli`. Its header says to power-cycle before flying (fc_probe.py:15-16), but the draft's R0b' step 2 runs it on every quad without that step.
- **Rung:** R0a (sheet), R0b'.

## Already covered in the drafts, not repeated here

Kill-path timing and Betaflight failsafe/rxfail; pack voltage bar and counting; weighing, rods and ring origin; body IDs, serials and binding phrases; ground effect and downwash (K20, K21); static hold, peel and capture gap; false or missed weld; the P9 bag; DDS domain; CPU soak; ring-pose staleness; operator roles.
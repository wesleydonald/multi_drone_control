# A pose that stops changing counts as no pose (card, 11 Nov 2026; Wesley's word in the approved plan of 11 Nov 15:56)

## What happened (RIG-1111-fm9c_6, registry -fm9c_6c, decisions FINDING 2026-11-11 15:56)
QUAD4 left the tracked volume at (0.29, 2.34, 2.89) m under Te-Jen's controller, 0.67 s before the operator's LAND.
From then its pose on `/drone_3/motion_capture_state` was bit-for-bit the same for 5.62 s, still arriving at 94 Hz: the
mocap PC's forwarder keeps sending a lost body's last pose (its packet is `id | x,y,z | qx,qy,qz,qw`, no validity
flag). Both pose watchdogs read arrival times (ours `pose_timeout_s` 0.25 s, tracker_node.py:1830; his 0.25 s,
tejen_mpc/main.py:2364), so neither fired. Our tracker took QUAD4 over at LAND on the frozen pose and held a constant
correction toward its reference (y 1.70 m): QUAD4 crossed the cage at 2.7-2.9 m (its magnet marker was last seen at
y -2.58 m) and hit the far wall; operator DISARM 4.2 s after LAND.

## How often poses freeze (all 15 flown flights of 11 Nov, every body, runs of identical poses of 0.25 s or more)
- Carriers (QUAD1-3): none from TAKEOFF to the disarm in any flight. Before TAKEOFF only while being handled (nrtfc, 1.3-3.3 s).
- QUAD4 in flight: fm9c three times, 0.34 / 0.68 / 0.50 s, hovering over the pickup at (0.65-0.71, 1.82-1.93, 1.3) m
  (outside the 1.75 m limit); fm9c_4 18.3 s during LAND at (1.02, 1.72, 1.14); fm9c_6 the 5.62 s above.
- The ring: only after it had been dropped (k1zo) or after landing. The magnet marker and the object: often, on the
  floor or under the quad. In fm9c_6 the object's pose was frozen through the whole pickup (26.6 s) and came and went
  in the lift; his proof then read a separation and switched the magnet off with the object in the air.
- A tracked body at rest never repeats: 0.0 % exact repeats for every quad and the ring over whole recordings
  (nrtfc_2, nrtfc_3 and others), jitter 0.01-0.05 mm.

## The change (one variable; rewritten 11 Nov 16:44 after the critic: the quads only)
`drone_communication/motion_capture_publisher_node.py`: for the four quads' bodies (11-14) only, when the packet's pose
text has been identical for longer than `MDC_MOCAP_FROZEN_S`, the node stops publishing that quad until its pose
changes, and says so once each way (`MOCAP FROZEN: QUADn ...`, `mocap live again: QUADn ...`). 0 = off = today's
behaviour, and the code default until Wesley's word; 0.5 s is the value proposed. The check sits before the publish
throttle and before the parser. On resume the parser is re-seeded: the first pose back only restarts the velocity
estimate (position, time, both filters) and is not published, so no speed is read from the jump across the gap.

Not in scope, on purpose (critic F1, F2): the ring (8), the magnet marker (24) and the object (6) pass as today. They do
freeze at rest or under a quad, his planner, his attachment manager and his TAKEOFF need them fresh, and no reader of
the ring's pose has a timeout in flight, so withholding it would add nothing and would hold up `carriers_down_alone`.
The magnet marker's repeats are already skipped where they matter (`tip_skip_repeats`, 30 Oct).

What then happens is what already happens for a mocap loss; nothing else changes:
- QUAD4 while Te-Jen's controller flies it (mux on his stream): his controller disarms QUAD4 on its 0.25 s pose
  timeout; our warm QUAD4 tracker disarms too and stays disarmed (`warm_fault_not_live` is off, and must stay off with
  this change); the fleet manager does not escalate; the carriers fly on and the operator LANDs.
- QUAD4 while OUR tracker flies it off the ring (a LAND take-back): its pose-timeout disarm is a **fleet-wide EMERGENCY
  STOP** (every tomorrow's line is a welded start without `welded_on_load`, so the fleet manager has no "off the ring"
  flag for it). This is the standing rule for any mocap loss of a quad we fly; the guard makes a frozen pose count as
  one. The carriers may still be descending: fm9c_6's LAND came 0.67 s after the freeze, so there the stop would have
  come at LAND +0.08 s with the carriers at height, where the operator's own DISARM came at +4.2 s after the wall.
  `welded_on_load:=true` would turn it into a LAND (twin-tested 11 Oct, R1612/R1613; never flown on the rig, never with
  a re-attach): Wesley's word, not proposed for 12 Nov.
- A carrier, or QUAD4 on the ring: the fleet-wide stop, as for any mocap loss today. Never seen frozen in flight.
Longest blind time: 0.75 s for a solid freeze. A body that gets a new solve at least every 0.5 s is never withheld, so
its readers can fly on poses up to 0.5 s old (critic F7); the real fix is a validity flag from the mocap PC's
forwarder.

Margin (critic F5): QUAD4's longest harmless freeze so far is 0.68 s (fm9c, hovering over the pickup at y 1.83 m),
0.07-0.08 s short of a disarm 1.3 m over the object. That is accepted together with the placement rule below.

## Checks (done 11 Nov 16:38-16:44)
1. Unit tests, `src/drone_communication/test/test_frozen_pose.py` (6): the filter (identical for less than the window:
   published; longer: withheld; a changed pose resumes at once; each body on its own clock; 0 = off) and the parser's
   re-seed (after a 2 s gap and a 1 m jump the first published speed is under 0.1 m/s, with the ema and with the
   slope filter; without the re-seed the same jump reads over 3 m/s).
2. Replay of the recorded quad poses through the filter at 0.5 s, TAKEOFF to the first disarm, 43 flown flights of
   4, 5, 6 and 11 Nov (172 quad recordings; 28 of the flights were not used to choose the window):

   | flight | quad | withheld | where | what the guard would have done |
   |---|---|---|---|---|
   | fm9c (11 Nov) | QUAD4 | 0.17 s and under 0.01 s | hover over the pickup, (0.66, 1.83, 1.33) and (0.71, 1.93, 1.33) | nothing: under the 0.25 s pose rule |
   | fm9c_4 (11 Nov) | QUAD4 | from LAND +19.9 s | (1.02, 1.72, 1.14), our take-back landing it with the object on | disarmed at about LAND +20.2 s with a fleet stop; the carriers had touched down at LAND +13.2 s; the operator's DISARM came at +21.7 s |
   | fm9c_6 (11 Nov) | QUAD4 | from LAND -0.17 s | (0.29, 2.34, 2.89), under his controller | disarmed at about LAND +0.08 s; see the EMERGENCY STOP case above |
   | the other 40 flights | - | never | - | - |

   No carrier withheld in any flight (129 recordings); QUAD4 never withheld inside |y| 1.7 m. The recordings are the
   published poses (about 94 Hz), not the raw packets.
3. The desk check on the real node, `tools/frozen_pose_desk_check.py` (in place of the twin run: the twin does not run
   this node): packets at 240 Hz to the node on a private ROS domain; QUAD4 and the ring frozen for 2 s, QUAD4 back
   1 m away, then QUAD1 frozen for 1.2 s. With the guard: QUAD4's poses stop at freeze +0.50 s and resume at its end,
   QUAD1's the same, peak speed after the resume 0.005-0.006 m/s; QUAD2, QUAD3 and the ring never interrupted; one
   line each way per quad. With the guard off: nothing interrupted, no lines. OK.
4. What a quad's readers do with the gap is not re-tested here: the trackers' pose timeout and the fleet manager's
   answer are the standing rules (the twin's MOCAP_LOSS runs R1612, R1613 of 11 Oct; a run on tonight's tree is in the
   overnight list).
5. gate --quick; the rig desk check of every script.

## Falsifier
A carrier, or QUAD4 inside |y| 1.5 m, withheld in flight while motion capture still tracks it (a log line
`MOCAP FROZEN` with the quad in plain view and its markers lit in Motive); or a quad disarmed by a freeze that would
have ended harmlessly (QUAD4 over the pickup: the 0.07 s margin above).

## Placement rule that goes with it (the sheet)
The object and QUAD4's mat within 1.3 m of the cage centre in x and y: every in-flight freeze of QUAD4 on 11 Nov was at
|y| of 1.7 m or more. One more place it gets there by itself: a LAND while QUAD4 holds the object steps it out to
2.23 m from the ring (1.2 m + the cable, so the object lands clear of the carriers), cut back to 0.10 m inside the
position limit: (0.99, 1.63) in fm9c_4, where its pose froze 4 s later. With the object on and the drop stalling,
FORCE DROP first, LAND second.

## Critic
(Condensed from the critic agent's report. 11 Nov evening, on the card as first written: every body, the check not yet wired; read-only review of the code at
bf52a8a plus the day's uncommitted changes. The card above is the rewrite it asked for.)

**The five questions**
1. **Tried before?** Yes, but only for the magnet marker and only inside the code that reads it. `tip_skip_repeats`
   drops a repeated body-24 pose in our magnet manager (magnet_attachment_manager.py:457-469; decisions 30 Oct; typed
   in every attach script), and Tejen's descent debounce ignores a frozen tip
   (docs/tejen/reply_2026-10-29/FROZEN_MARKER_1920.md). The 17 Oct touchdown critic already wrote that "if Motive
   repeats a lost body's last pose, the frozen pose passes" (2026-10-17_touchdown_height_gate.md:262-265). No registry
   row has ever tested a filter at the mocap node or on the drone bodies.
2. **Simpler explanation the change does not address.** QUAD4 left the tracked volume under his controller before any
   pose froze. The pickup spot was at y 1.77 m, and after the big object fell off QUAD4 climbed 1.3 m and moved 0.7 m
   outward in 2 s (FINDING 15:56; RIG-1111-fm9c_6c). Our take-back then latched onto the frozen pose with a 0.64 m
   constant error. The filter only changes how this ends. The placement rule and "small object only" address the
   cause; the climb itself is still unexplained.
3. **Falsifier.** The real risk is a withheld pose that was correct and needed (the object or the magnet marker at
   rest), or a withheld QUAD4 pose that leads to a worse outcome than flying on (a fleet-wide EMERGENCY STOP). The
   card's falsifier only looks for false positives on the carriers, the ring and QUAD4. It uses the same recordings
   that set the 0.5 s threshold, leaves out bodies 6 and 24, and never runs the nodes that read the poses.
4. **What it breaks.** His pickup and mission when the object or marker is withheld (F1). A whole-fleet EMERGENCY STOP
   when a take-back meets a QUAD4 freeze (F3). `carriers_down_alone` after a landing where the ring's pose froze (F2).
   The twin cannot test any of it (F6). It does not touch the yaw-only network.
5. **Asked for?** Yes: Wesley's word in the approved plan (decisions FINDING 2026-11-11 15:56). The 0.5 s threshold and
   the every-body scope are the card's own choices.

**Findings**
- **F1 (BLOCKER as written): withholding the object (6) and the magnet marker (24) stalls his pickup and mission.**
  Every body goes through this one loop (his own IRL mocap publisher is not launched). The object and the marker
  freeze often; in fm9c_6 the object was frozen for the whole pickup (26.6 s) and that pickup still attached. Withheld,
  his planner's `inputs_ready()` fails (online_join_planner.py:4862-4893, `input_timeout_s` 0.5) and no reference is
  published; 0.5 s later his MPC latches an external-reference fault until DISARM (tejen_mpc/main.py:1108-1136); his
  attachment manager needs both fresh within 0.25 s and latches "ASSUMED ATTACHMENT LOST" after 0.20 s more
  (magnet_attachment_manager_irl.py:304-352); his TAKEOFF can be refused (pendulum state from tip callbacks only).
  Accept if: the filter applies to `drone_body_ids` only.
- **F2 (HIGH): withholding the ring adds no safety but has a cost.** No reader of the ring pose has a timeout in
  flight; `carriers_down_alone` needs a ring pose fresher than 0.2 s (planner_node.py:1357-1362) and the ring freezes
  after landing. Accept if: the ring is out of scope.
- **F3 (HIGH): a take-back during a QUAD4 freeze becomes a whole-fleet EMERGENCY STOP.** Every line tomorrow is a
  welded start; without `welded_on_load`, `/drone_3/on_load` is never published, so any disarm of QUAD4 while its mux
  reads 'ours' is an EMERGENCY STOP (fleet_manager_node.py:393-407). The LAND take-back fires at LAND +0.1 s, latches
  without a freshness check, and the mux switches at once. fm9c_6 missed this by about 20-30 ms (LAND at freeze
  +0.67 s; take-back about +0.77 s against the timeouts at +0.75 s). fm9c_4's QUAD4 was frozen under our take-back.
  Accept if either: Wesley types `welded_on_load:=true` (built, twin-tested, never flown on the rig; his word); or the
  card and the sheet state the EMERGENCY STOP case and Wesley accepts it in writing.
- **F4 (HIGH, implementation): where the check sits, and the resume.** Before `parse_packet`, keyed by body; on resume
  re-seed the parser (position, time, the ema buffers, the slope filter's history) and publish the first sample with
  zero twist or skip it: parsed straight through, fm9c_6's 4.9 m resume would read about 118 m/s through the ema and
  stay over `max_speed` for about 15 frames. Accept if: a unit test resumes after a 1 m jump and the first published
  speed is under 0.1 m/s, for both filters.
- **F5 (MEDIUM): thin threshold margin.** QUAD4's longest harmless hover freeze was 0.68 s against a trip at 0.75 s.
  Acceptable alongside the 1.3 m placement rule, said in the card. Keep `warm_fault_not_live` off with this change.
- **F6 (MEDIUM): the checks.** The twin does not run this node; the replay used the recordings that chose 0.5 s.
  Accept if: a desk check on the real node replaces the twin run (freeze QUAD4, then a carrier; the pose resuming with
  no velocity spike).
- **F7 (LOW):** a body solved at least every 0.5 s is never withheld; the real fix is a validity flag from the
  forwarder. **F8 (LOW):** the card and the sheet must name the same bodies.

**Verdict: fly it with changes.** (1) The quads' bodies only. (2) The check before `parse_packet`, with the re-seed and
its unit test. (3) Either `welded_on_load:=true` on Wesley's word, or the EMERGENCY STOP case written into the card and
the sheet. (4) A desk check on the real node in place of the twin run.

## After the critic (11 Nov 16:44)
| finding | done |
|---|---|
| F1, F2 | the quads only; the ring, the marker and the object pass as before (the desk check freezes the ring for 2 s: never interrupted) |
| F3 | Wesley 11 Nov 20:19: the guard stays OFF ("I will disarm in this situation anyway it does not need to be automatic"). Built, tested, not flown |
| F4 | the check before the throttle and the parser; `ParseData.reseed()`; unit tests for both filters; desk check: peak 0.006 m/s after a 1 m jump |
| F5 | the margin and the placement rule in the card; `warm_fault_not_live` untouched (off) |
| F6 | `tools/frozen_pose_desk_check.py`; the replay extended to 4-6 Nov (28 flights not used for the window) |
| F7, F8 | in the card; the sheet says "a quad" |

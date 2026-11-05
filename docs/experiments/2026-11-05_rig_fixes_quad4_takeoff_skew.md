# Three rig fixes from the lab day of 5 Nov (card for the critic; exploratory, real world next)

Owner: Wesley. Written 5 Nov 2026 after the lab day (registry RIG-1105-*, decisions OPEN 5 Nov (a)-(c)). Wesley's word on
5 Nov: "1) Yes. 2) Ok. 3) Sure." All uncommitted on week8 (9f20f8b +). Not a claim: no thesis table reads these.

## 1. Question
Do the three changes remove the three failures of 5 Nov without opening a new way to lose a quad or the ring?

## 2. The changes (one per failure)

**F1, QUAD4 off the ring keeps the ring-tilt envelope (RIG-1105-k1ob_2).** After the bigger object's drop into the
moving ring, the ring jerked 7 -> 54 deg in 0.2 s, the carriers' 55 deg drop fired, and QUAD4's own tracker (idle,
warm: its stream was Te-Jen's) kept the trackers' 60 deg `max_payload_tilt_deg` envelope and DISARMED QUAD4 at 80 deg
while his MPC flew it 2.5 m up: QUAD4 fell. Fix as first built (superseded, see Response 1: it now reads the mux) (`src/tracker/tracker/tracker_node.py`,
`src/bringup/bringup/graph_attach.py`): the newcomer's tracker (only it, and only with `newcomer_fault_land`, on by default) subscribes to
`/drone_<d>/on_load` (Bool, transient-local, published by our planner's `_set_on_load` at the weld, the detach and the
re-weld; `_set_on_load`, never False before the fleet has left the floor creep; a welded start's False only once
its tip has moved RELEASE_SEEN_M off the plate after the detach, `_release_seen`; an unwelded newcomer's False at
`_announce_on_load`). While it reads false, `_payload_checks_down()` is true (the ring-tilt envelope, the payload checks) and the
tracker's `_drop_rule` is not run. Never heard (another mode, the topic absent): the checks apply as before. The
carriers' trackers do not subscribe. The broader switch `warm_fault_not_live` (card 2026-10-11_quad4_tilt_cut, F6b not
clean in the twin) is not used.

**F2, his TAKEOFF before our release (RIG-1105-nr1r).** Pressed during our 20 s route round the outside, his planner
latched its climb where QUAD4 was then; at the hand-over QUAD4 flew 1.3 m across the ring. Fix (our fork of his
planner, `online_join_planner.py`; his launch; `graph_attach._partner_nodes`; the console): with `takeoff_gate_topic`
set (our combined rig launch sets `/partner/release` when `partner_attached`; default '' = his behaviour), a TAKEOFF in
WAIT_FOR_TAKEOFF before the first message on that topic is held and carried out at it, latched where QUAD4 is then
(his take-off-from-air path). The same button also reaches his MPC (`/tejen/drone_command`), which then flies; our mux
drops its stream until the release and switches at his first armed, above-idle command after it. While held his
planner's WAIT_FOR_TAKEOFF reference is QUAD4's measured pose (not the boot mat 1-2 m below), so his MPC computes a
hover where QUAD4 is and its kT estimator sees a consistent hover. The console's 2 s "his planner did not enter
TAKEOFF" alarm waits for the release while QUAD4 is ours (mux 'ours').

**F3, the false loss at LAND (RIG-1105-sq8_2).** 1 s into LAND the 6 cm cable-length check read QUAD1's cable 1.098 m
and dropped the ring; the mocap had every cable at 1.01-1.03 m. Planner log against the bag: the planner's carrier
poses (drones 0-2) were ~0.6 s behind the bag in the lift and from the end of the hover through LAND, while its ring
and QUAD4 poses were ~0.15 s behind (the same lag as every other flight that day: 60-150 ms on all five bodies). In
hover a late pose changes nothing; when the ring started down at 0.18-0.26 m/s, the late carrier heights read as a
9-13 cm stretch. Cause of the extra 0.5 s: not found (one mocap node publishes all five bodies; the bag has them all at
~93 Hz, 1 ms after their stamps; subscriptions are depth 5; seen in this one flight). Fix
(`dissipative_node._detect_tick`, `planner_node`): the planner keeps each pose's mocap header stamp; with
`detect_skew_s` > 0 (default 0 = off) the check skips a drone whose pose and the ring's were stamped further apart than
that (proposal for the rig: 0.05 s), with a throttled log line naming the drone and the gap. Always on (log only): a
warning at most every 5 s when a slot's pose is stamped more than 0.25 s behind the ring's.

## 3. Risks to look at
- F1: a QUAD4 that reads off the ring while still hanging on it (on_load false too early, or true never re-sent) loses
  its tilt disarm while carrying: when exactly does `_set_on_load(d, False)` fire (detach start? the release?), and is
  every re-weld path followed by `_set_on_load(d, True)` (take-back, the OCP fold-in, the drop path)?
- F2: a held TAKEOFF that never goes (release never published: LAND before the release, a cut on a tilt fault, a
  take-back): his MPC flies on a hover reference whose output the mux drops; is anything left flying or armed that
  should not be? A release published while QUAD4 is not where his planner expects?
- F3: a real release while the poses are skewed is not detected for as long as the skew lasts; the tip check
  (`tip_detect_m`, off on these lines) and the tilt drop still run. Is 0.05 s right given 60-150 ms normal ages (the
  skew between the ring and a carrier, not their age, is what is tested)?

## 4. Tests (pass) and checks
- `src/tracker/test/test_off_ring_checks.py` (4), `src/bringup/test/test_real_mode_launches.py`
  (`test_the_newcomers_tracker_reads_on_load_and_the_carriers_do_not`, `test_a_welded_quad4s_takeoff_waits_for_our_release`),
  `src/tejen_mission/test/test_takeoff_gate.py` (4), `test_partner_mission_rig.py::test_takeoff_gate_only_when_typed`,
  `tools/test/test_console.py` (1 new), `src/dissipative_planner/test/test_detect_skew.py` (5). Gate `--quick`.
- Twin (lab PC): the square detach and re-attach on today's tree (as R3708) and the pickup with the orbit (as R3711),
  once each, for F1's on/off transitions and no change elsewhere. F2 has no twin path (the twin's own start gate already
  holds his TAKEOFF). F3 is off unless typed.
- Rig: F1 and F2 fly as built (default on for the combined rig launch); F3 only if Wesley types `detect_skew_s`.

## Critic
(critic agent, 5 Nov ~11:45; verbatim summary)

Q1 Tried before? In part: F1's mechanism exists as `welded_on_load` (card 2026-10-11_quad4_tilt_cut M4, twin only),
beside the broader `warm_fault_not_live` (F6b not clean); F2 copies the twin's start gate; F3 is new. Q2 Simplest
unaddressed explanation: F3's extra 0.5 s has no known cause and may not show in the header stamps. Q3 Falsifier: QUAD4
disarmed by its own ring-tilt envelope after a detach, or a false loss with agreeing stamps; no fail thresholds written,
no test reaches either. Q4 Breaks: nothing locked; F2 moves a refusal DISARM into the hand-over. Q5 Asked for: yes.

1. (high) F1 does nothing on welded starts: the planner publishes on_load only with `_newcomer_fault_land`, which needs
   `not start_attached or welded_on_load` (dissipative_node.py:814-828); `welded_on_load` is false (common.yaml:293) and
   no rig file sets it. The tracker subscribes, never hears, keeps the 60 deg envelope: nr1r, sq8, nr9o keep k1ob_2's
   failure after the detach. Settle: type welded_on_load (your word: it turns an off-ring disarm into a LAND) or narrow
   the claim; a launch test that a publisher exists wherever the tracker subscribes.
2. (low) F1 window: on the unwelded path on_load goes False at the DETACH command (dissipative_node.py:1793), with
   QUAD4 possibly still on its plate; the carriers' checks still run. Settle: say so.
3. (medium) F2 moves his refusal into the hand-over: at the gate the held TAKEOFF runs `floor_start_refusal`
   (online_join_planner.py:4613, 4671-4674); a refusal publishes DISARM to his MPC (7704-7711), which has flown since the
   press, so the mux may already have switched (elrs_mux.py:323-331): a DISARM to QUAD4 in the air. LAND / tilt cut /
   take-back before the release are safe (no release sent). Settle: no refusal when taking off from the air, and a test.
4. (medium) "Consistent hover" claimed, not shown: his xy integral (main.py:1542, limit 0.30) and kT UKF (1967-1976)
   run on dropped commands for up to 16-20 s. Settle: his MPC log of nr1r at the hand-over.
5. (medium) F3 may not see sq8_2's lag: the arrival-timed stale check did not fire, depth-5 queues at ~93 Hz cannot hold
   0.6 s and the bag was on time, so the stamps probably agreed. Settle: log each slot's stamp gap at 10 Hz; leave
   detect_skew_s untyped until that log exists.
6. (medium) A flickering skew hides a real release: a skewed tick resets the count (dissipative_node.py:2065) and
   detection needs two in a row. Settle: skip without resetting; a test with an alternating skew.

CRITIC: rewrite the card | F1 as built does nothing on the welded-start lines it claims to cover, and F2's in-air
refusal DISARM must be closed before the rig.

## Response (5 Nov ~11:55, uncommitted; tests pass)
1. Confirmed. F1 now reads the newcomer's mux instead of the planner's on_load: `mux_state_topic` (/drone_3/mux_state,
   latched, published by elrs_mux in every Gazebo and rig attach graph), off the ring while it reads 'partner' (his or
   the approach stream flies QUAD4); 'ours', 'cut', 'latched' or nothing heard: checks as before. Covers welded and
   unwelded starts; welded_on_load (the fleet manager's LAND) untouched. The tracker already had a `_mux_state_cb` (the
   bumpless hand-over, partner_contact only); the new handler is `_off_ring_cb` (a test checks it is not shadowed). The
   launch test covers the rig floor start, the rig welded start and the twin, and that the mux node exists.
2. Gone with 1: the mux reads 'partner' only from his first armed, above-idle command after the release; until then
   (DETACH, unload, step-out, route) QUAD4 is ours and its checks apply.
3. Confirmed and closed: a refusal while QUAD4 is in the air (more than TAKEOFF_FROM_AIR_M above the lowest pose seen
   in WAIT_FOR_TAKEOFF) never disarms; his planner holds where QUAD4 was at the refusal (latched, not following), and
   the TAKEOFF is tried again at each release message (1 s) or press. On the mat his refusal (DISARM) is unchanged. It
   also closes the same hazard for a normal-order press after the release, which existed before F2. Tests:
   test_takeoff_gate.py (in the air: no DISARM, hold latched, retried until the ring pose is back; on the mat: DISARM).
4. Read: RIG-1105-nr1r (TAKEOFF 16 s before the release, his reference fixed at the press position, so worse than F2):
   his kT UKF stayed 22.0-22.5 (nominal 22) while his commands were dropped, 22.19 at the hand-over (log lines
   1793830168-184); it fell to 17 only later, in his own flight. His xy integral is not printed; under F2 his reference
   follows QUAD4, so its position error, and the integral's input, stay near zero until the release.
5. Disagree in part: every subscriber gets the same message, and the planner's carrier values were 0.6 s old in
   content, so their stamps were 0.6 s old too (the mocap node stamps at publish); a planner that held old content with
   new stamps would need a second writer, and the bag shows one smooth series per body. A delay of that size behind
   depth-5 queues fits head-of-line blocking in a RELIABLE reader (a lost sample held until its repair; not shown). Agreed
   on the log: log.csv now has skew0..skewN (ring stamp minus each slot's, every 10 Hz row; test). Whether to type
   detect_skew_s before a flight has logged them is Wesley's call (recommend: type it, as it only skips mismatched
   ticks, and read the columns after the first flight).
6. Agreed and done: a skewed tick is skipped with its count kept; test with an aligned / skewed / aligned release.

## Twin check (5 Nov ~12:00-12:25; registry R3712-R3715, exploratory, judged after the runs)
- R3712 (the square, QUAD4 welded on 9; as R3708) and R3714 (the same on 12 cpus): tracker_3 stands its ring checks down at the
  mux's release and back at the re-weld; re-attach first try, 19.3-19.4 s after the magnet release (R3708 19.4).
- R3713 (the pickup on the orbit; as R3711): checks down from boot (his stream), back at the take-back; pickup, drop and the
  object in the net as R3711.
- The square's T stage leaned more than R3708 (median 11.2-12.0 against 8.5 deg; fold-in max 14.8-15.8 against 6.8); R3715 with
  the tracker change off leans the same, so it is not this change; cause not found (decisions FINDING 5 Nov ~12:30).

## Wesley 5 Nov ~12:55: F3 dropped
"Skip the loss check thing, I dont want it": detect_skew_s and its skip removed (test deleted); the stamp-gap log columns and the late-pose warning stay (diagnostic only).

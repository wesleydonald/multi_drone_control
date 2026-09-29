# Card: a tracker that disarms between ARM and TAKEOFF refuses TAKEOFF for the fleet

Date: 2026-09-29. Type: change to a gate that can hold the fleet down -> critic required (working rules). Status: BUILT 2026-09-29 on Wesley's word (Q11 option a + drone 3 single command path); critic pass 2 READY with fixes applied.

## Problem (found in R0749, registry R0749c)
All four trackers logged `Pose timeout (1.9 s) - disarming` between ARM (sim 3.0) and TAKEOFF (sim 8.0) (the wall-clock
pose watchdog under CPU load, as in R0493/R0511). The fleet manager (src/controller_quad_load/controller_quad_load/main.py)
still accepted TAKEOFF: `_takeoff_fleet` checks only `fleet_armed`, set once by the ARM thread, and
`_arming_feedback_callback` escalates a tracker's disarm only `if self.flying`. Every tracker refused (`Cannot takeoff:
not armed`); the planner swept its arc with no drone flying. Nothing flew, so no harm in R0749.
On the rig, a partial case is the hazard: one tracker drops (pose timeout, reference stale, envelope) after ARM and before
TAKEOFF; TAKEOFF is accepted; the other trackers lift and drag the ring one-sided (R0560: three drones hauled the ring to
44 deg with one tracker dead). The operator sees "ARM sequence complete" and has no cue.

## Change (one variable: what a pre-TAKEOFF tracker disarm does)
In `_arming_feedback_callback`: `if self.fleet_armed and not self.flying and was_armed and not msg.data` -> the same path
as a failed ARM (Wesley Q6b, 2026-09-28): log ERROR `Drone i disarmed before TAKEOFF: disarming the fleet; TAKEOFF
refused. RELAUNCH to retry`, `_disarm_fleet(emergency=False, reason=...)` (service disarm of every tracker, no
/fleet/abort, no direct ELRS, `shutdown_requested` true, `fleet_armed` false). TAKEOFF then warns "not armed" and does
nothing. Applies whatever the drone's mux state: before the hand-over a dead tracker of ours means the hand-over would
cut that drone's motors, so the hand-over must not happen either; no /fleet/abort, so the partner's drones behind the
muxes are not latched (Q6b's reasoning). The in-flight path (`if self.flying ...`) is unchanged.
Second hop (same change, belt and braces): `_takeoff_fleet` also refuses when any managed drone's last feedback is
`False` (a feedback lost at the transition). Log names the drones.

## Why not escalate to /fleet/abort
Nothing of ours is flying before TAKEOFF; in M2 the partner's four drones hold the ring behind the muxes and an abort
would latch them down (the reason for Q6b). A fleet-wide service disarm grounds our idling trackers (props at idle) and
leaves his stream alone.

## Risks the critic should weigh
- A tracker whose feedback flickers False before TAKEOFF for a benign reason (e.g. a re-ARM, a tracker restart) now
  holds the fleet down until a relaunch. Which paths publish False on the feedback topic besides a disarm?
- Sequences that legitimately disarm a managed tracker between ARM and TAKEOFF (M1 reserved drone, M2 late controllers,
  the attach launch with n + reserved managed). Must not fire there.
- Race with the ARM thread: feedback True from arming, then fleet_armed set; a stale False queued behind it.
- Second hop: drone_armed starts False for every drone and is only set by feedback; a drone whose feedback never arrives
  (topic remap, QoS) would block TAKEOFF where today it flies. The M2 graph remaps the feedback to /ours/...

## Verification plan
1. pytest (fake-node pattern of test_mux_abort_manager.py): a tracker disarm between ARM and TAKEOFF -> service disarm,
   no abort, no direct ELRS, TAKEOFF refused; the same in flight -> unchanged emergency path (and F5 scoping); disarm
   before ARM or after LAND -> nothing; second hop: one drone's feedback False at TAKEOFF -> refused.
2. Gazebo, no new behaviour expected in clean runs: canonical M1 (partner_attached_orbit) and one creep hover
   (ocp_hover_ground_creep) end as their baselines (R0752, R0726-style). No forced-fault Gazebo arm (the watchdog trip
   is load-dependent; the unit test covers the logic).
Falsifier: TAKEOFF accepted after a managed tracker disarmed pre-TAKEOFF; any clean baseline refused TAKEOFF or aborted.

## Critic (pass 1, 2026-09-29, condensed): NOT READY
1. BLOCKING: in three_attach (sim, SIL, real:=true) drone 3's `/drone_3/command` is remapped to `/fleet/command`
   (three_attach_launch.py:726-728): its tracker arms on the ARM broadcast and takes off on the TAKEOFF broadcast
   (callback_manager_multi.py:140-162) whatever `_takeoff_fleet` decides; the approach MPC too (:671-672). A manager
   refusal holds only drones whose command topic the manager owns. Fix: the second hop also disarms the fleet. Adjacent
   existing hole: the `wait_for_service` failure branch (main.py:243-250) returns without disarming while drone 3 is
   already armed by the broadcast.
2. No benign False found: feedback is event-only (callback_manager_multi.py:107,130,145,156,209,216); pre-TAKEOFF
   disarm sources are the pose watchdog (controller_mpc.py:1078-1084) and repeated solver failures (:1253-1260).
   Tejen's 2 Hz heartbeat lands on /drone_i/arming_state_feedback in M2 (m2c_vehicle_controller.launch.py:61) and is
   kept off our manager only by the partner_m2 /ours remap (dissipative_launch.py:231-235): add a contract test.
3. ARM race both ways (True may land after fleet_armed=True; False may land before it): keep the check at TAKEOFF;
   unit cases (a) True then False before fleet_armed, (b) late True, (c) the fleet's own disarm Falses do not re-enter,
   (d) four simultaneous Falses fire once.
4. No existing graph would newly fail TAKEOFF on the second hop (manager n = tracker n, or both remapped); add a test
   with all feedbacks True and TAKEOFF accepted.
5. M2 reasoning wrong: a dead tracker does not cut motors at the hand-over (require_live_takeover keeps his stream,
   elrs_mux.py:73-76). The real M2 hazard is mixed control (three muxes on ours, one on his). The driver sends
   /fleet/handover unconditionally (drive_m2_handover.py:357-360): make it stop on the new log line.
6. Verification: add a forced disarm over the real topic (no Gazebo, isolated domain, desk_m2_rearm_latch.py pattern):
   `ros2 service call /ours/drone_1/arming_service interfaces/srv/SetArming "{arm: false}"` on the partner_m2 split graph,
   and on the three_attach SIL graph drone 3 must be gone and must not take off. Falsifier under load: a baseline
   refused after a logged tracker disarm is the gate working (void, re-fly), not a failure.
7. Trade-off: the service disarm shuts the trackers down; today a pose timeout leaves the tracker alive for a re-ARM.
   On the rig a floor mocap dropout > 0.25 s between ARM and TAKEOFF would cost a relaunch. Extends Q6b from "ARM
   failed" to "disarmed after ARM": Wesley's word.

## v2 (after critic pass 1): what would be built on Wesley's word
Both hops call `_disarm_fleet(emergency=False, reason)` (service disarm, no /fleet/abort, shutdown_requested): hop 1 on a
managed tracker's False between fleet_armed and flying; hop 2 at TAKEOFF when any managed drone's last feedback is
not True. The wait_for_service failure branch disarms the same way. drive_m2_handover stops on the log line. Tests:
critic 3(a)-(d), 4, the partner_m2 remap contract; the forced service-call disarm on the partner_m2 and three_attach
SIL graphs; then canonical M1 + one creep hover as clean baselines (void, not fail, if a logged tracker disarm precedes
a refusal).

## Built (2026-09-29, Wesley: Q11 (a) + "give drone 3 the same single command path as the others")
- main.py: hop 1 (a managed tracker's False between fleet_armed and flying -> `_disarm_fleet(emergency=False)`, no abort),
  hop 2 (TAKEOFF refused + fleet disarmed when any managed drone's last feedback is not True), the wait_for_service
  failure branch disarms the fleet too.
- three_attach_launch.py: drone 3's tracker has no /fleet/command remap (the manager arms it by service and sends it
  TAKEOFF on /drone_3/command); the manager manages n + 1 only with the approach chain.
- real_attach_launch.py (superseded) refuses to start.
- M2 driver: stops when our fleet is not armed before the hand-over or the lift, or when the manager logged a grounding.
- Runner/metrics: `fleet_grounded` (the manager's grounding lines) makes a run VOID instead of "never lifted".
- Tests: test_pre_takeoff_gate.py (11 cases incl. the critic's race cases), launch tests for drone 3's single path, the
  manager count and the retired launch. SIL R0753: drone 3 armed via service, TAKEOFF from the manager, weld + fold-in.

## Critic pass 2 (condensed): READY
1. real_attach_launch.py still remapped drone 3 to /fleet/command (rig-reachable) -> it now refuses to start.
2. Sim-only residue: the collaborator's approach MPC (three_attach, enable_approach_mpc default true, sim attach demo only)
   still takes ARM/TAKEOFF from /fleet/command, so in that demo drone 3 can lift on it when the manager refuses. The rig
   refuses the approach MPC; partner M1 and SIL run without it. Remapping it to /drone_3/command changes the demo:
   Wesley's call.
3. Nothing drone 3 needed came only by the broadcast (its handler takes ARM/DISARM/TAKEOFF; LAND/ESTOP were unknown to it).
4. main.py logic correct; manager count now n + 1 exactly with the approach chain (was n + reserved_attach).
5-7. Driver ARM loop stops on a grounding; tests (a)-(c) added; decisions line added; grounded runs VOID in the runner.
Owed: Gazebo baselines (canonical M1 vs R0752, one creep hover); the forced service-call disarm on the partner_m2 split graph
(desk, next session).

## Residue closed (Wesley 2026-09-29: "Move it")
The sim approach MPC (three_attach, attach demo) now takes commands on /drone_3/command, not /fleet/command; with
`takeoff_implies_arm` (new, default false, true only there) the manager's accepted TAKEOFF arms and starts it (the manager's
ARM is a service call to our tracker, never to this node). A refused TAKEOFF never reaches it; emergencies reach drone 3
through the mux latch. Tests: test_takeoff_implies_arm.py, test_sim_approach_mpc_takes_commands_from_the_manager_only.
Owed: one Gazebo run of the sim attach demo (approach MPC path) next session.

## Result (2026-09-29)
| check | run | baseline | value | verdict |
|---|---|---|---|---|
| forced drone-1 disarm after ARM, split M2 graph, real topics | D0001 | - | gate once, TAKEOFF refused, 0 takeoffs, no abort, muxes 'partner' | PASS |
| canonical M1, drone 3 via the manager | R0754 | R0752 | peaks 2.98 / 2.08 / 2.61 (2.80 / 2.53 / 2.74) | PASS on the change; G1 time bar missed (146 s: 6 rejoin descent retries in his mission) |
| creep hover n3 (config z_ki 0) | R0755 | R0557 | hover 0.661 (0.661) | PASS |
| attach demo (tracker-flown rejoin in the circle) | R0756 | R0723 | weld 38.9 s, peak 3.61 (37.2 s, 3.39) | PASS |

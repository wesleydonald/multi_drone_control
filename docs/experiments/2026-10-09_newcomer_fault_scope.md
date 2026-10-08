# A fault on an unwelded newcomer grounds it alone (M1 rule c, first step)

1. **Question.** Wesley, 8 Oct night: a fault on QUAD4 before its weld should not bring the ring down; for now QUAD4
   "falls alone" and the carriers fly on (a tracker fault-land follows as its own work). Does the fleet keep flying
   when the newcomer's own tracker faults?
2. **Hypothesis.** Today any in-flight disarm of a drone whose mux is 'ours' makes the fleet manager stop the whole
   fleet (fleet_manager_node.py `_arming_feedback_callback`); the newcomer's mux is 'ours' from boot, so a fault on
   it drops the ring from the air. With `newcomer_fault_alone` on, the planner latches `/drone_3/on_load` False
   until the weld (True while physically on the ring, False again at its detach) and the fleet manager does not
   escalate an in-flight disarm of a drone whose on_load is False; the planner takes that newcomer out of the
   flight (magnet OFF, no landed-gate wait, no later weld). Carrier faults, faults after the weld, DISARM, ESTOP
   and the pre-TAKEOFF gate are unchanged. Amends the locked fleet-wide disarm for an unwelded newcomer only.
3. **The one variable.** `newcomer_fault_alone` (bringup common.yaml, default false; planner param), branch
   newcomer-fault-scope: dissipative_node.py on_load publishers (latched, reliable), `_newcomer_arming_cb`,
   `_weld_refused`; fleet_manager_node.py the on_load subscription (no message = escalate, as today). Off in
   partner runs (the mux 'partner' exemption covers those). Test tooling, sim only: the mocap emulator's
   `/drone_<i>/mocap_dropout` and the runner's `MOCAP_LOSS` (a lost marker set: the tracker's real pose-timeout
   path on a live node, not a service shutdown, critic M6 of the land/abort card); the push variant of the
   free-start attach world (`three_rigid_ground_rig_attach_push.sdf`, ApplyLinkWrench added, Wesley's word 8 Oct
   night) for a tilt/speed envelope fault.
   Pinned: the land/abort card's twin_r8a_dry base with `newcomer_lifecycle` on (both switches on in C1-C5).
4. **Baseline.** C0 (switch off, same tree): pose loss mid-transit gives FLEET ABORT and the ring falls.
   If it does not, the code reading is wrong and this card is void.
5. **Pass / fail.** Twin, `tools/attach_metrics.py` + events.csv + launch.log.
   | Run | Events | Supports if | Falsified if |
   |---|---|---|---|
   | C0 | switch off; MAGNET ON, WAIT_LOG transit, MOCAP_LOSS 3 +2 s | FLEET ABORT within 1 s of the loss | no abort: void |
   | C1 | switch on; as C0 | no FLEET ABORT; QUAD4 disarmed within 0.5 s of the loss; 'disarmed in flight before joining the ring' logged; carriers hold the ring (z within +/-5 cm of 1.0, tilt <= 10 deg) until LAND; LANDED; no WELD | any fleet abort or carrier disarm before LAND |
   | C1b | as C1, loss 8 s into the descend phase (QUAD4 falls ~0.6 m by the ring) | no fleet abort; carriers land the ring on LAND; ring tilt peak reported | a carrier faults |
   | C2 | push world; WRENCH 10 N +y on QUAD4 for 0.5 s, 2 s into the transit | QUAD4's envelope fault (tilt or speed) logged; as C1 | as C1 |
   | C4 | weld_radius 0.08, WAIT_WELD, MOCAP_LOSS 3 +5 s | FLEET ABORT as today (on the ring: a carrier) | the carriers fly on |
   | C5 | DISARM 2 s into the transit | all four disarmed, muxes latched | any drone armed |
6. **Repeats.** C1 x2; C0, C1b, C2, C4, C5 x1.
7. **Cost.** 7 Gazebo runs (laptop).
8. **New code?** Yes: a change to fault escalation (a gate that can hold the fleet down, the other way round: it
   stops holding it down for one drone). Behind `newcomer_fault_alone`, default off; default only after these rows
   pass and the reviewer reads the table; the locked line in the working rules is amended at that merge. Removal:
   drop the branch (fault on the newcomer = fleet-wide stop again).
   Unit tests: fleet manager (no escalation with on_load False; escalation with True or no message; a carrier
   fault while the newcomer is free; operator DISARM; pre-TAKEOFF unchanged; QoS latched); planner (on_load at the
   physical weld, at a refusal and its release, at the fold-in and detach; the arming edge; ARM clears it).

## Critic

The land/abort card's critic (8 Oct night) set this card's must-fixes: M1 (Wesley's word on c: given, "falls alone
now, land later"), M6 (a faithful injector: MOCAP_LOSS on a live tracker and the push world; fired in the transit
and in the hold), M8 (on_load: no message = escalate; latched and reliable on both ends; True at the start-weld
fold-in and any weld, False at its detach, True again at a re-weld; unit tests for a weld and a disarm in either
order). A second critic pass on this card is due before its runs count.

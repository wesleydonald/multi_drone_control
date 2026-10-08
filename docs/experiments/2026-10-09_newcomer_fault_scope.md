# A fault on an unwelded newcomer lands the fleet (M1 rule c, first step)

Revision 2 (critic REWRITE, 8 Oct night; Wesley the same night: "if quad 4 fails we just land everything because we
need to do the next run anyway", which replaces revision 1's "the carriers fly on").

1. **Question.** When QUAD4's own tracker faults before its weld, does the fleet land the ring instead of the
   fleet-wide disarm that drops it from the air?
2. **Hypothesis.** Today any in-flight disarm of a drone whose mux is 'ours' makes the fleet manager stop the whole
   fleet (fleet_manager_node.py `_arming_feedback_callback`); QUAD4's mux is 'ours' from boot, so a fault on it
   drops the ring. With `newcomer_fault_land` on, the planner latches `/drone_3/on_load` False until it folds QUAD4
   in (False again at its detach); the fleet manager answers an in-flight disarm of a drone whose on_load is False
   (and whose planner is still publishing it) with LAND on `/fleet/command`, not the emergency stop. QUAD4's own
   motors are already off (its tracker cut them), so it falls; the carriers land the ring; the planner drops QUAD4
   from the landed gate and never folds it in. Carrier faults, a fault after the fold-in, DISARM, ESTOP and the
   pre-TAKEOFF gate are unchanged. Amends the locked fleet-wide disarm for an unwelded newcomer only.
3. **The one variable.** `newcomer_fault_land` (bringup common.yaml, default false; planner param; it acts only
   with `newcomer_lifecycle` on), branch newcomer-fault-scope: dissipative_node.py on_load publishers (latched,
   reliable; True only at the fold-in), `_newcomer_arming_cb` (out 'fault', magnet OFF, gate and trajectory hold
   released), `_weld_refused` (also refuses a drone whose last feedback was disarmed: the weld-order race);
   fleet_manager_node.py `_on_load_callback` (an excused drone reported on the ring later is a carrier fault:
   emergency stop), `_on_load_live` (the flag counts only while its planner publishes it). Off in every partner run.
   Test tooling, sim only: `MOCAP_LOSS` (the emulator stops QUAD4's state: the tracker's real pose-timeout path on a
   live node; the runner's own mocap watchdog skips that drone) and the push variant of the free-start world
   (`three_rigid_ground_rig_attach_push.sdf`, Wesley's word). Pinned: the land/abort card's dry base
   (weld_radius 0) with `newcomer_lifecycle` on in every row; C0 and C1 differ only in `newcomer_fault_land`.
4. **Baseline.** C0 (switch off, same tree): FLEET_ABORT within 1 s of QUAD4's disarm. If not, the card is void.
   The land/abort card's rows (batch 5) fly this base first; a WAIT_LOG timeout makes a row VOID.
5. **Pass / fail.** Twin, events.csv + launch.log + run.csv (`tools/attach_metrics.py`; QUAD4's mocap is NaN after
   a MOCAP_LOSS, so its height comes from the tip columns, tip z + 0.59 m).
   | Run | Events | Supports if | Falsified if |
   |---|---|---|---|
   | C0 | switch off; MOCAP_LOSS 3 (rest of the run) 2 s into the transit | FLEET_ABORT within 1 s of QUAD4's disarm | no abort: void |
   | C1 | switch on; as C0 | QUAD4 disarmed within 0.5 s of the loss; the manager's 'LAND, every drone lands' within 1 s of it; no FLEET_ABORT; carriers armed until LANDED; ring z within +/-5 cm and tilt <= 10 deg until LAND, then LANDED | any FLEET_ABORT or carrier disarm before LANDED |
   | C1b | as C1, 8 s into the descend phase (QUAD4 falls onto the ring at plate 9) | as C1 for the manager; the carriers land the ring; ring tilt peak and z dip reported | a carrier faults: physics, not the rule (the twin rod has no collision, so the rig would be worse) - report, and the R8b sheet keeps "QUAD4 fault over the plate: DISARM" |
   | C2 | push world; 10 N +y on QUAD4 for 0.5 s, 2 s into the transit | QUAD4's envelope fault logged (tilt or speed); as C1 | as C1; no envelope fault (check QUAD4's y velocity) = VOID |
   | C3 | MOCAP_LOSS 0 (a carrier) 2 s into the transit, switch on | FLEET_ABORT within 1 s (the lock unchanged) | the fleet lands instead |
   | C5 | DISARM 2 s into the transit, switch on | all four disarmed, muxes latched | any drone armed |
   Between the columns (no abort, but the ring outside +/-5 cm or tilt > 10 deg): not supported, one repeat.
   Unit-test only: no later fold-in of a faulted newcomer (no weld is possible at weld_radius 0); the weld-order race
   (both orders, planner and manager); a stale flag without its planner. Deferred: a fault after a weld in this
   world (needs the T-R8b weld runs first).
6. **Repeats.** C1 x2; C0, C1b, C2, C3, C5 x1. C3 flies before the C1 repeat (critic M5).
7. **Cost.** 7 Gazebo runs (laptop, ~11 min each).
8. **New code?** Yes: a change to fault escalation. Behind `newcomer_fault_land`, default off; default only after
   these rows pass and the reviewer reads the table; the locked line in the working rules is amended at that merge.
   Welded-start worlds (R10b/R11) keep it off until a row covers a drone welded from t=0. Removal: drop the
   branch (a fault on the newcomer = the fleet-wide stop again).

## Critic

Revision 1 (8 Oct night): REWRITE. What revision 2 does with each item:
- M1 the runner's own mocap watchdog ended a MOCAP_LOSS run 2 s after the loss: the runner skips the drone it cut
  (tested).
- M2 weld/disarm order race: the planner refuses to fold in a drone whose last feedback was disarmed; the manager
  turns an excused drone that is later reported on the ring into an emergency stop (both tested).
- M3 the rig magnet can grab before a weld is declared: with "land everything" (Wesley) the excuse can cover the
  whole approach; on_load is True only at the fold-in, and a dead drone hanging on the ring lands with the ring.
- M4 C1b falls onto the ring: numeric report, and what a carrier fault there means (row C1b).
- M5 no carrier-fault row with the switch on: C3 added.
- M6 the base was unflown: the land/abort rows fly it first; LOG_TIMEOUT = VOID.
- S1 no WELD cannot fail at weld_radius 0: unit-test only. S2 the grey band: not supported. S3 metrics: tip
  height after the loss, the manager's LAND line; to add to attach_metrics with the batch. S4 C4 deferred.
  S5 stale flag: counts only while its planner publishes. S6 tied to newcomer_lifecycle. S7 a fault releases the
  trajectory hold. S8 C2 void without an envelope fault. S9 welded starts keep it off. S10 C5 kept as a cheap
  regression. S11 "for the rest of the run". S12 Wesley's decisions merged into this branch.

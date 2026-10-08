# Creep lift-off gated on TAKEOFF (r303 fix)

1. **Question.** Does gating the creep's lift-off detection on TAKEOFF (and re-latching the arc anchors from the
   resting pose until TAKEOFF) remove the r303 failure without changing a normal takeoff?
2. **Hypothesis.** r303 (8 Oct): QUAD1 was lifted ~5 cm for a battery replug before TAKEOFF; the creep counted
   the rise as a lift-off, swept the arc with the fleet disarmed, timed out at -3.9 deg and the planner held
   'NOT lifting'. With the gate (6bfb881) a rise before TAKEOFF starts nothing, and the anchors at TAKEOFF are the
   resting pose, so a normal floor start is unchanged.
3. **The one variable.** src/mpc_planner/mpc_planner/creep_controller.py: lifted_off only when takeoff_seen, the
   anchors re-latched every tick before TAKEOFF (arc and soft creep).
4. **Baseline.** The 8 Oct twin floor starts (R1204-R1222): LIFTED 9.8-10.0 s after WAIT_LIFT, no arc-creep
   timeout. Unit tests reproduce r303 (pytest test_creep_controller.py: a drone lifted by hand before TAKEOFF;
   the anchors re-latch until TAKEOFF).
5. **Pass / fail.** Supports if every floor start of the 8 Oct night batch (R1224 onwards) lifts with LIFTED within
   +/- 0.3 s of 9.9 s after WAIT_LIFT, logs 'anchors latched at TAKEOFF' and 'arc sweep started' after TAKEOFF,
   and never logs an arc-creep or lift-off timeout. Falsified by any floor start LIFTED > 1 s later than that, or
   a timeout line.
6. **Repeats.** Every floor start in the batch (11 runs).
7. **Cost.** No extra runs (rides on the night batch).
8. **New code?** Yes, small, in the takeoff gate (a gate that can hold the fleet down): the critic's challenge
   goes under "## Critic" below; removal plan: revert 6bfb881 (one commit).

## Critic
(pending)

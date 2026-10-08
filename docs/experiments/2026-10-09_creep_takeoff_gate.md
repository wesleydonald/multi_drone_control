# Creep lift-off gated on TAKEOFF (r303 fix)

1. **Question.** Does gating the creep's lift-off detection on TAKEOFF (and re-latching the anchors from the
   resting pose until TAKEOFF) remove the r303 failure without changing a normal takeoff?
2. **Hypothesis.** r303 (8 Oct): QUAD1 was lifted ~5-10 cm for a battery replug before TAKEOFF; the creep counted
   the rise as a lift-off, swept the arc with the fleet disarmed, timed out at -3.9 deg and the planner held
   'NOT lifting'. With the gate (rig-prep 6bfb881) a rise before TAKEOFF starts nothing and the anchors at TAKEOFF
   are the resting pose, so a normal floor start is unchanged.
3. **The one variable.** src/mpc_planner/mpc_planner/creep_controller.py: lifted_off only when takeoff_seen, the
   anchors re-latched every tick before TAKEOFF (arc and soft creep). Same commit: the ARM-failure advice text.
   Separate small addition (S1 below): the planner re-runs its slot-offset check once at TAKEOFF (warn only).
4. **Baseline.**
   - The failure, reproduced: unit tests (test_creep_controller.py: a drone lifted by hand before TAKEOFF; anchors
     re-latch until TAKEOFF; resize after a hand lift; soft creep re-latch) and the twin negative control below.
   - Normal takeoff: the 8 Oct week4 laptop floor starts on b811d90 + the model-integral diff, R1217, R1218, R1221,
     R1222 (LIFTED 10.01, 9.85, 9.86, 9.89 s after WAIT_LIFT, from each events.csv), no arc-creep or lift-off timeout line.
5. **Pass / fail.**
   - **Direct test (critic M1):** configs/experiments/w4_creep_handled.yaml (push world, even ring: ARM,
     DISARM, drone 1 pushed up 0.4 s at 5.7 N ~12 cm, ARM, TAKEOFF; drone 1 must be upright, tilt < 30 deg, at
     TAKEOFF or the run is void). Negative control once on 6bfb881^ (the
     creep file reverted): must show the lift-off before TAKEOFF and the ring not lifting (WAIT_LIFT timeout or
     'NOT lifting'); if it lifts normally the twin cannot see r303 and the test is void. Then once on the fix:
     must lift within the normal band below and log 'anchors latched at TAKEOFF' and 'arc sweep started' after
     TAKEOFF. Rig: folded into r400 (props off): ARM, lift QUAD1 10 cm for 5 s, set it down; T2 must show no 'arc
     sweep started' and the phase stays 'waiting for TAKEOFF'.
   - **Normal takeoff (regression):** every floor start of the 8 Oct night batch on rig-prep 8b10be2 (which contains
     6bfb881): supports if LIFTED - WAIT_LIFT is within the baseline band 9.85-10.01 s widened by 0.3 s each way
     (9.55-10.31 s) and no timeout line; falsified if any run is outside 9.0-11.0 s or logs a timeout; between the
     two: grey, one repeat of that config.
6. **Repeats.** Direct test: negative control 1 + fix 2. Regression: the 11 batch runs.
7. **Cost.** 3 Gazebo runs (laptop) after the batch; the rig check costs no flight.
8. **New code?** Yes, small, in a takeoff gate. Removal plan: revert 6bfb881 (note: that also restores the old
   ARM-failure text "no relaunch needed", which is wrong) and the slot re-check commit. Operator rule until the
   direct test passes: keep "relaunch T2 after touching a drone" on the rig sheet (dropping it is Wesley's call).

## Critic

Verdict REWRITE (the card, not the code), 8 Oct evening. Nothing in 6bfb881 must change before the rig; the card's
bars could only show a normal takeoff is unchanged, not that the r303 failure is gone.
- M1: add a direct test of the r303 half: a twin run lifting a drone before TAKEOFF (w4_rearm events on the push
  world with a WRENCH on one drone), negative control on 6bfb881^, then the fix; or a props-off rig check in r300.
  Done above (both).
- M2: the bars left gaps (0.3-1 s late, or early, was neither). Fixed above.
- M3: the baseline mixed branches and hosts and included voids. Fixed above (laptop week4 runs, the batch commit named).
- S1: a replug can put a drone back off its plate (r303_2: drone 1 47 deg off) and the slot check ran only at
  launch: re-run it once at TAKEOFF, warn only. Done (planner_node.py).
- S2: unit tests for resize() after a hand lift and for the soft creep re-latch. Done.
- S3: fly one attach-graph floor start where the start weld joins the creep (the M1 batch never goes through
  resize()). Planned with the legacy floor-start M1 config after the batch.
- S4: say the removal plan also reverts the ARM-failure text. Done.
- Also noted: before TAKEOFF the creep reference follows the live pose (+0.1 m lead) instead of a fixed point;
  harmless at armed-idle; takeoff_seen comes from /fleet/step (one 50 Hz tick after TAKEOFF) and never resets,
  so a DISARM after a flight still forces a T2 relaunch.

## Direct test, first attempt (VOID)

The first push (6.5 N for 0.5 s) threw drone 1 0.46 m up; it tumbled (peak tilt 177-179 deg) and lay on its side
(tilt 90 deg) from 8.5 s to the end in all three runs, so no controller could lift it.

| Run | Code | Lift-off logged | Sweep | Drones 2-4 at sweep end | Drone 1 |
|---|---|---|---|---|---|
| R1235 | 6bfb881^ | before TAKEOFF (on the push) | ran and timed out before TAKEOFF | not swept | on its side |
| R1236 | fix | after TAKEOFF ('anchors latched at TAKEOFF') | after TAKEOFF | 68-70 deg | on its side |
| R1237 | fix | after TAKEOFF | after TAKEOFF | 68-69 deg | on its side |

The gate behaves as designed in the part the code controls, but the pass bar (a normal lift) cannot be met with a
drone on its side. Push resized to 5.7 N for 0.4 s (drone 0.475 kg + half the 0.075 kg rod: predicted rise ~0.12 m,
twice the 0.05 m lift-off margin); negative control and fix re-flown on it.

## Direct test, softer push (result)

| Run | Code | Drone 1 rise / tilt at TAKEOFF | Sweep | Lift after WAIT_LIFT | Verdict |
|---|---|---|---|---|---|
| R1242 | 6bfb881^ | 0.16 m / 0 deg | timed out before TAKEOFF ('NOT lifting') | none (WAIT_LIFT timeout, peak 0.02 m) | reproduces r303: test valid |
| R1241 | fix | 0.163 m / 0 deg | anchors latched at TAKEOFF, started after it | 9.91 s | supports |
| R1243 | fix | 0.163 m / 0 deg | as R1241 | 9.83 s | supports |

SUPPORTS, 2 of 2, inside the band 9.55-10.31 s, no timeout line. Wesley (8 Oct night): he relaunches T2 between rig
flights anyway, so the sheet rule stays; the props-off r400 check is still on the Wednesday setup.

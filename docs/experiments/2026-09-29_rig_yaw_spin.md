# Card: rig yaw spin at the creep floor start (CURRENT_STATE open problem 8)

Date: 2026-09-29. Type: change in the flight loop (tracker output) -> critic (the adversarial check below). Asked for by
Wesley ("make sure to fix this"). Status: BUILT on branch real-world-testing; to be confirmed on the rig.

## What happened (rig, 2026-09-16)
Three tethered launches had one drone spin at spool-up while the others held heading to 2 deg (tools/yaw_excursion.py):
15:54 drone 1 +83 deg (log ends; mocap orientation flips mid-spool, not a clean case), 16:40 drone 2 +180.5 deg,
16:42 drone 0 +330 deg the long way. In all three the tracker COMMANDED the turn: u3 negative = yaw rate positive (the same
sign relation holds in SIL), u3 pinned near -1.

## Diagnosis (investigation 2026-09-29, five agents; established vs inferred)
Established:
- Heading error was zero at onset (the creep reference carries no yaw, the heading latch held the rest heading; error
  -0.02 / -0.07 deg at TAKEOFF). u3 ramped from the first TAKEOFF tick while yaw stayed at the datum for 5-9 ticks.
- Every rig drone rests tilted 9.2-19.6 deg on its rod; sim spawns rest < 2.6 deg (1222 logs).
- The tracker publishes x(1) (0.1 s ahead) every 20 ms and re-pins it as the next x0 input state
  (controller_mpc.py publish/_applied_u, acados.py node spacing and u_dot bounds): the stick can move ~5x faster than the
  model allows, so a grounded solve winds the yaw stick to full while the drone sits on the floor.
- The spinners turned 44-58 deg before leaving the floor; most of each spin happened AIRBORNE with u3 held near -1 against
  errors of +50 to +300 deg: the 09-16 code assumed 670 deg/s at full yaw stick (rates 70/670) while the rig turns ~100 deg/s,
  so the OCP committed to finishing the turn. The rate model was changed to 100/100 after 09-16 (d8aea49 pre-rewrite).
Inferred / not separated: what starts the grounded yaw (wound roll/pitch sticks coning the attitude vs a degenerate
low-throttle spool solve); why these drones and not others. Neither mechanism was reproduced on the current (100/100) code.

## Change (one variable: the yaw channel on a tilted floor start)
controller_mpc.py: if the drone rested tilted more than YAW_HOLD_TILT_DEG (5) when TAKEOFF froze its heading, send yaw 0
(and pin 0 as the x0 yaw input, so it cannot wind up) until the airborne gate (spawn z + AIRBORNE_MARGIN) or at most
YAW_HOLD_MAX_S (1.5 s) after TAKEOFF; once per takeoff. New log column yaw_hold (1 while holding; u3 stays the solver's wish).
Not in this change: publishing u_applied + dt u_dot instead of x(1) (changes every flight and the sim baselines: own card if
the rig check below fires); zeroing the x0 sticks while grounded; yaw weights.

## Critic (adversarial check of the diagnosis and the fix, 2026-09-29): WEAK -> corrections applied
- The hold is safe as a guard: sim never engages (max sim rest tilt 2.52 deg), the x0 yaw box is exactly [0, 0] while held,
  the first published yaw after release is at most 0.1 (no step), failed-solve fallbacks re-send a held message.
- Overstated: the spins were mostly airborne (the 670 deg/s model lock-in, co-primary and already changed); the hold removes
  the floor onset but does not prove the spin is closed. Roll/pitch sticks wound on the floor (seen on the spinners at release)
  may still start a yaw right after release (mechanism 1): watch that window on the rig.
- Applied: a release timeout (a drone tracking a low creep reference may stay under the airborne gate: worst t_air 1.46 s).
- Tests below are plumbing tests (the mechanism is not reproduced on current code); the rig is the real test.

## Verification
- pytest src/controller_quad_load/test/test_yaw_hold.py (7): engages only past 5 deg (13.1/11.8 yes, 2.5/0 no); yaw 0 and
  x0 yaw 0 while held, roll/pitch/throttle untouched; release on the airborne gate, not re-armed on landing; release on the
  timeout; level rest unchanged. Package suite 169 passed.
- SIL R0763 carry_hover_n3_long: yaw_hold 0 on every tick of all three trackers, ring 0.662 m (R0473 0.663): sim unchanged.
- RIG (R1 then R3a, exploratory): read yaw_hold, u3, u0/u1, qref, wx, solve_status in each tracker log.
  Falsifier: any drone turns > 15 deg in the first 2 s after TAKEOFF, or u3 heads to +-1 within 0.3 s of the hold release
  while roll/pitch sticks are wound (mechanism 1 survives -> the x(1) publish card). ESTOP rule unchanged (yaw > 45 deg).

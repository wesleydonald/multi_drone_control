## Where things stand

- **Sim:** the full M1 and the M2 hand-over pass in Gazebo. The known weak spot is occasional "carrier kicks" (glitches in the body rate the tracker gets from mocap). They cost one M2 run in three. The fix waits on the P9 mocap measurement.
- **Rig:** three airframes free-hover (09-16, throttle about 0.45). Tethered flights have been flown, but the ring has never been lifted, and the throttle needed to carry it has never been measured cleanly.
- **Safety paths:** DISARM has been used on the rig many times. Since 28 Sep it also grounds drones that are still on Tejen's stream. A tracker that disarms between ARM and TAKEOFF now grounds the whole fleet and refuses TAKEOFF. The drone-0 "disarmed at takeoff" cases you saw would now stop the flight instead of lifting the ring one-sided.
- **Motive ids:** quads 11-14 (drones 0-3), ring 8, pickup object 6. These match the code defaults.
- **Combined stack (Tejen + us) on the rig:** parked until the carry, detach and rejoin work on our own stack.

## The one number that decides tomorrow: throttle headroom

The tracker caps throttle at **0.6**. Carrying the ring on 45 deg rods needs about 1.2-1.5x free-hover thrust. At your expected **1.2 kg** airframes and the measured free hover of about 0.45:

| Layout | Predicted carry throttle | Margin to 0.6 |
|---|---|---|
| Three drones, plates 1/5/9 | 0.568 | 0.03 |
| Four drones, even ring 0/3/6/9 (R3e) | 0.537 | 0.06 |
| Four drones, plates 1/3/5/9 (M1 layout), plate-9 drone | 0.589 | 0.01 |

These are **predictions** from `tools/headroom.py`. It matched every SIL check at the rig's thrust gain within 0.01:
- three drones pin at the cap at 0.64 kg, and hold at 0.595 at 1.0 kg (R0758, R0759);
- at 1.0 kg the even four-ring carries with every drone at 0.557 and 1.2 deg of tilt (R0762);
- at 1.0 kg, on 1/3/5/9 the plate-9 drone sits at the cap and the ring tilts 7 deg (R0761).

A free hover of 0.43 or 0.47 moves the three-drone number to 0.543 or 0.593. So weigh the airframes, measure u_free in R1 with the rods on, and let the R3a hold measure the real value.

## Tomorrow, in order (commands in docs/rig_2026-09-30_commands.md)

| Step | What | Go on only if |
|---|---|---|
| Setup (45 min) | Weigh drones and ring, tape rods, check Motive (ring origin at centre), prebuild solvers | Motive bodies correct, all four `/dev/QUAD*` present |
| R0b desk, props off (75 min) | mocap rate and rest bag, telemetry, magnets, preflight GO, abort checks (pre-TAKEOFF gate, SPACE, CLI ESTOP, ring markers covered) | every kill path stops every drone; preflight GO |
| R2 magnets (20 min) | pull test, hand-lift coupling, capture gap | every magnet holds 2x its share |
| R1 free hover, rods on (40 min) | four airframes; u_free and kT per drone | tilt < 3 deg, xy < 0.12 m, no yaw turn > 15 deg |
| Headroom (5 min) | `tools/headroom.py` with the weights and u_free | no drone over the cap on the chosen layout |
| R3a hold (30 min) | creep to 45 deg and hold, no lift; first measured carry throttle | coupled, no spin, no drone at 0.6 for 2 s, hold throttle under your stop value |
| R3b0 first lift (stretch) | lift to 0.60, 20 s, LAND; tethered kT printed | R3a passed |

## What is most likely to go wrong

| Risk | What you would see | What to do |
|---|---|---|
| Throttle cap binds | a drone at thr 0.6, ring low or tilted | LAND; fewer grams, even four-ring, or raise the cap (your call) |
| Yaw spin on the creep start (seen 09-16) | a drone turning at spool-up | fixed in code (yaw hold); LAND > 15 deg, ESTOP > 45 deg |
| A kill path does not reach a drone | a drone still armed after DISARM/ESTOP | R0b abort checks before props; spacebar with RViz focused |
| Tethers not coupled | hold throttle no higher than free hover; drone > rod + 3 cm from its plate | R2 hand-lift check; LAND |
| Drone 0 disarms at takeoff | "disarmed before TAKEOFF" in T2 | the fleet now grounds itself; check its mocap body (pose timeout 0.25 s) |
| Mocap dropout or body flip | pose timeout, sudden tilt reading | R0b: cover markers, hand-tilt each body |
| Pack sag | throttle creeping up at constant height | packs >= 24.0 V, matched; flights <= 60 s |

## Yaw spin: diagnosed and fixed in code

On 09-16 three tethered launches had one drone turn 83-330 deg at spool-up. The tracker commanded it, with zero heading error:
- every rig drone rests tilted 9-20 deg on its rod, and the solve on the floor winds the yaw stick to full;
- the drones turned 44-58 deg before leaving the floor;
- the old 670 deg/s yaw-rate model, since corrected to 100 deg/s, then locked the turn in once airborne.

**Fix (2633f68):** on a tilted floor start the tracker now sends yaw 0 until the drone is airborne, or for at most 1.5 s. Sim drones rest level, so sim is unchanged (SIL R0763). The rig confirms it: watch the new `yaw_hold` column and the first 0.3 s after release.

## RViz: suggested changes (not made yet; your pick)

1. **Bug:** the panel's "fleet manager not running" check passes whenever anything listens on `/fleet/command`. `fleet_viz` in T1 does, so with T2 down the panel still says the command was sent. Detect the manager by name instead.
2. **Safety:** an ESTOP button, and send DISARM/ESTOP several times. Today each is a single message.
3. **Safety:** lock the magnet toggles while armed. One mis-click drops a tether mid-carry.
4. **Safety:** DETACH steps to the next drone after each press, so a double-click detaches two drones. Remove the step or add a lockout.
5. Enable TAKEOFF only when every drone is armed (today: any drone).
6. Per-drone label in the 3D view: body id, throttle (red when at 0.6 for 2 s), pack V. Make it vanish on a mocap dropout.
7. Show the manager's refusals ("TAKEOFF REFUSED", "disarmed before TAKEOFF") and the planner phase (creep, hold, lift) on the panel.
8. Panel battery colours: green only from 24.0 V (today 23.4).

Items 6 and 7 are Python display changes. Items 1-5 and 8 are C++ panel changes that need a rebuild: better after Wednesday than the night before. Before flying, test that SPACE still disarms when a text box in RViz has focus.

## Decisions for you

- **Stop value** for the hold throttle (0.55 would block three drones at 1.2 kg; 0.58 would not).
- **Raise the 0.6 cap?** It needs the Betaflight throttle limits read first and a solver rebuild.
- **Fallback layout:** the even four-ring (R3e) if three drones come out over your stop value.
- **RViz:** which of the changes above, and whether before or after Wednesday.

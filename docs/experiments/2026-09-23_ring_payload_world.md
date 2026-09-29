# 2026-09-23 ring payload world (smoke)

## Question
Do the worlds still rest, lift and hover after replacing the 0.6 kg solid-disc payload
with Te-Jen's M2A ring (24 segments, 12 magnet plates, `m2a_ring_fixture.sdf`) at the
official 0.86 kg, with the ring inertia (Ixx 0.0273, Izz 0.0545 vs the disc's 0.0155 /
0.0308)?

## One variable
The payload model (geometry, mass, inertia) in the world SDF, with the planner told the
same mass. Everything else is the launch default. Not a controller claim; a world change
verified before it is used for one.

## Baseline
R0284 / R0285 (`ocp_hover_ground_m10`, 1.0 kg disc, hover 0.537 m) and R0276 (0.6 kg,
0.566 m). The ring at 0.86 kg should land between them if the planner offset scales with
mass (open problem 9).

## Falsifier
Either repeat: payload peak z < 0.35 m, or a fleet abort before LAND, or the payload
sliding/tipping at rest before TAKEOFF (|xy| drift > 5 cm or tilt > 5 deg at t < 6 s).

## Runs
`ocp_hover_ground_ring086`, 2 repeats, headless Gazebo.

## Cost
2 of 6 Gazebo runs.

## Outcome (2026-09-23)
| run | sim interface | hover z | settled tilt | wall / sim |
|---|---|---|---|---|
| R0330 | as shipped | 0.543 | 0.15 deg | 139 s / 40 s |
| R0329 | clock 100 Hz, mocap 120 Hz, viz 30 Hz | 0.544 | 0.26 deg | 80 s / 36.5 s |
| R0328 | throttled, VOID | — | — | clock QoS mismatch, fixed |

Falsifier not met: peak > 0.35 m, no abort before LAND, no rest drift. The ring at 0.86 kg
hovers between the 0.6 kg disc (0.566, R0276) and the 1.0 kg disc (0.537, R0284), as
open problem 9 predicts. Not two repeats of one config: the two runs differ in the
sim-interface rates and agree to 1 mm; a same-config repeat is still owed if this number
is ever cited as a baseline. `min_payload_z: 0.02` in the hover criteria is stale for the
ring (rests at 0.010; the disc rested at 0.025).

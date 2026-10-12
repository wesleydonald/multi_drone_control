# Rig Thu 15 Oct, 15:00-18:00 (DRAFT 10 Oct; checked at Tuesday's desk rehearsal with the Wednesday sheet)

Same rig, layout and setup block as Wednesday (docs/rig_2026-10-14_commands.md: plates, bodies, `C`, `A`).
QUAD4 (`/drone_3`, body 14, tip body 24) welds onto plate 9. Thursday flies only what passed its gate:

| run | gate before Thursday | status 10 Oct |
|---|---|---|
| R9 (our approach on the circle) | - | dropped 10 Oct: Tejen's controller does every attach on the rig; twin results R1468-R1472 stay as thesis evidence |
| R10a | Tejen's own sheet, his stack alone | his |
| tilt (180 deg gap) | card + critic + twin 2 of 2 + neighbours (plan X) | not built: waits for a later lab day |
| R10b / R11 | combined M1 stack (critic, R0c desk run) | slips to 21-22 Oct |

## Setup (15:00, packs from the charger)

As Wednesday's setup block. Kill path check (RViz DISARM, T3 `fleet disarm`) once before the first flight.

## Flights, in order

Any Wednesday carry follow-up that did not fly (circle 0.125 / 0.0625 m/s, figure-eight).

- **R10a:** our stack DOWN (Ctrl-C T1/T2, `ros2 node list` empty), the other three radios unplugged; Tejen runs his
  sheet on QUAD4 with his serial port. We watch only.

## Stop rules

- 8 Oct rules for any detach; ring over 15 deg -> `fleet land`.
- Any hand on a drone or any DISARM in attach: relaunch T2. Packs below 23.6 V. A pose gap over 0.1 s on 14/24/8
  with QUAD4 overhead ends M1 for the day.

## To check at the desk rehearsal (Tuesday)

The R10a hand-over: our stack down (`ros2 node list` empty) before Tejen's launch.

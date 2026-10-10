# Rig Thu 15 Oct, 15:00-18:00 (DRAFT 10 Oct; checked at Tuesday's desk rehearsal with the Wednesday sheet)

Same rig, layout and setup block as Wednesday (docs/rig_2026-10-14_commands.md: plates, bodies, `C`, `A`).
QUAD4 (`/drone_3`, body 14, tip body 24) welds onto plate 9. Thursday flies only what passed its gate:

| run | gate before Thursday | status 10 Oct |
|---|---|---|
| R9 held | 2 clean R8b on Wednesday; twin_r9_held 2 of 2 (card 2026-10-10_twin_r9) | twin 2 of 2 (R1468, R1471); reviewer pending |
| R9 moving | R9 held clean on the rig; twin_r9_moving 2 of 2 | NOT flown: twin falsified 2 of 2 (no weld on a moving plate; 0.31 m to a carrier) |
| R10a | Tejen's own sheet, his stack alone | his |
| tilt (180 deg gap) | card + critic + twin 2 of 2 + neighbours (plan X) | not built: waits for a later lab day |
| R10b / R11 | combined M1 stack (critic, R0c desk run) | slips to 21-22 Oct |

## Setup (15:00, packs from the charger)

```bash
cd ~/multi_drone_control; mdc; source install/setup.bash
C=$HOME/multi_drone_control/configs/rig
# R9 files from Wednesday's calibrated attach_m1.yaml (keeps attach_tip_dz and weld_point_z as signed Wednesday):
{ cat $C/attach_m1.yaml; echo 'traj_hold_s: 5.0'; } > $C/attach_m1_orbit.yaml
{ cat $C/attach_m1_orbit.yaml; echo 'attach_moving: false'; } > $C/attach_m1_orbit_held.yaml
O="load_traj:=orbit traj_radius:=0.5 traj_speed:=0.125"
A="tools/rig_flight.sh real_control_launch.py mode:=attach target_z:=1.0 lift_ramp_vel:=0.15 enable_approach_mpc:=false mocap_magnet_tip_body_id:=24"
```

Weld radius: the value chosen at Wednesday's decision point (default 0.025). Kill path check (RViz DISARM, T3
`fleet disarm`) once before the first flight. Each run: T2 line -> T3 preflight (R8 line of Wednesday's sheet) ->
`fleet arm` -> `ARM sequence complete` -> `fleet takeoff`.

## Flights, in order

```bash
$A $O params_file:=$C/attach_m1_orbit_held.yaml weld_radius:=<Wed> r9h     # circle held for the attach, x2
```

- **r9h:** after the circle has run one lap on three (~25 s): `ros2 topic pub -t 3 /magnet/command std_msgs/msg/String "{data: 'ON'}"`.
  T2: `ATTACH commanded: trajectory held 10 s for the approach`; QUAD4 climbs, transits, descends over plate 9 (the
  ring is still), tip hold, `ATTACH drone 3 (OCP resize)`, the reconfiguration hold, then the circle resumes on four.
  One lap on four, then `fleet land`.
- **r9m:** not flown (twin: the tip loop never runs on a moving plate, so no weld; needs new code first).
- **R10a:** our stack DOWN (Ctrl-C T1/T2, `ros2 node list` empty), the other three radios unplugged; Tejen runs his
  sheet on QUAD4 with his serial port. We watch only.

## Stop rules

- Wednesday's R8b rules (caught but no weld within 1 s -> `/magnet/command OFF`; a weld with nothing caught ->
  `/fleet/detach 3`; ring over 15 deg -> `fleet land`).
- R9: QUAD4 body within 0.35 m of a carrier, or its rod near a carrier rod -> `/magnet/command OFF`; the circle not
  resuming 5 s after the reconfiguration hold -> `fleet land`.
- Any hand on a drone or any DISARM in attach: relaunch T2. Packs below 23.6 V. A pose gap over 0.1 s on 14/24/8
  with QUAD4 overhead ends M1 for the day.

## To check at the desk rehearsal (Tuesday)

The two generated files (`ros2 param get /dissipative_planner attach_moving` -> False for r9h, True for r9m;
`traj_hold_s` 5.0); `load_traj` orbit reaches the planner in mode:=attach on the rig; the circle hold line in T2
after a fake ON against fake_mocap.

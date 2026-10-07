# Rig 8 Oct

Plates clockwise 0-11 from +x (old 1/3/5/9 = 11/9/7/3, old 1/3/6/9 = 11/9/6/3). Detach plates: drone 1
(QUAD1, `/drone_0`) 3, drone 2 11, drone 3 7 (r304: 6), drone 4 (`/drone_3`, leaves) 9.

## Setup

```bash
cd ~/multi_drone_control; mdc
colcon build --symlink-install --packages-select bringup mpc_planner tracker fleet_manager dissipative_planner simulation_communication drone_communication utility_objects && source install/setup.bash
python3 tools/prebuild_planner.py 2 3 4 --load-mass 0.86 --drone-mass 0.55 --cable-len 0.55
rio num_drones:=4 magnet_channel:=6 magnet_initial:=ON          # T1, all session
# T2, once:
R="tools/rig_flight.sh real_control_launch.py num_drones:=4 target_z:=1.0 lift_ramp_vel:=0.15"
C=$HOME/multi_drone_control/configs/rig
D="mode:=dissipative attach_azimuths_deg:=30,90,150,270"
O="load_traj:=orbit traj_radius:=0.5 traj_speed:=0.125"
```

Each run: T2 line -> T3 preflight -> `fleet arm` -> `ARM sequence complete: all 4 armed` -> `fleet takeoff`.
```bash
python3 tools/preflight.py --drones 4 --real --planner dissipative_planner --detach --max-ground-z 0.5 --battery-min 23.6
```
(r301, r306: `--planner mpc_planner`, no `--detach`.) `ARM FAILED: drone N`: replug its battery, `fleet arm`
again. QUAD2 `pending`: power-cycle it. Packs 23.6 V or more; mats under every drone and the ring.

## Runs, in order

```bash
$R mode:=mpc $O params_file:=$C/traj_hold_rts.yaml r301                       # circle + ref_time_shift
$R $D params_file:=$C/detach_drop.yaml r302                                     # hover detach, announced
$R $D params_file:=$C/drop_and_land.yaml r303                                   # hover detach, unannounced
$R mode:=dissipative attach_azimuths_deg:=30,90,180,270 params_file:=$C/detach_drop.yaml r304     # if r302 better
$R mode:=dissipative attach_azimuths_deg:=30,90,180,270 params_file:=$C/drop_and_land.yaml r304   # if r303 better
$R $D $O params_file:=$C/detach_unannounced_orbit_drop.yaml r305               # circle detach, unannounced
$R mode:=mpc load_traj:=fig_8 traj_radius:=1.0 traj_speed:=0.2 params_file:=$C/traj_hold.yaml r306   # figure-8
```

- **r300 (first):** props off, r301's line named `r300`. `fleet arm` -> `ARM sequence complete`; `fleet disarm`;
  `fleet arm` again without relaunching -> same line; `fleet disarm`, Ctrl-C.
- **r301:** even ring 0/3/6/9 at r206e60's start spot, 1.7 m clear on +y. Before ARM,
  `ros2 param get /tracker_0 ref_time_shift` prints True. `fleet land` 30 s into the circle. Base: r206e60, lag 0.20 s.
- **r302-r304:** T2 shows `d4 ... (plate 9)`; 1.3 m clear on +y. r304: drone 3 on plate 6; better = the lower
  ring tilt peak (I read the logs).
- **r305:** ring 0.5 m toward -y of the cage centre; 1.6 m radius clear. Drone 4 lands by itself, the others
  circle on.
- **r306:** even ring, 2.2 m clear along x. Lands itself after the 35 s lap, then `fleet land`. If r301 helped:
  again with `traj_hold_rts.yaml`, named r307.

Release, after 10 s steady (hover or circling):
```bash
ros2 topic pub -t 3 /fleet/detach std_msgs/msg/Int32 "{data: 3}"          # announced (r302); fallback if no UNANNOUNCED line in 2 s
ros2 topic pub -t 3 /drone_3/magnet std_msgs/msg/String "{data: 'OFF'}"    # unannounced (r303, r305)
```
Expect `DETACH` / `UNANNOUNCED DETACH: drone 4 (/drone_3)` within ~0.5 s and drone 4 stepping 0.5 m out.
`fleet land` ~8 s later (r305: ~20 s later, once drone 4 is down).

**Drop and land (r302-r305):** a second loss or ring tilt over 30 deg switches every magnet off, drops the ring
and lands all four. Hands off until they are down and disarmed.

## Stop rules

- A drop, ring tilt over 30 deg or a second drone lost: no more detaches today.
- A magnet slips (drone off its plate without a detach): land, stop.
- Packs below 23.6 V.

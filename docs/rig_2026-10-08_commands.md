# Rig 8 Oct: constant-speed figure-8, unannounced detach

Measured cable tension throughout. Background: `docs/experiments/2026-10-07_unannounced_detach.md`.

## Setup

```bash
cd ~/multi_drone_control; mdc
colcon build --symlink-install --packages-select bringup mpc_planner tracker fleet_manager dissipative_planner simulation_communication && source install/setup.bash
python3 tools/prebuild_planner.py 2 3 4 --load-mass 0.86 --drone-mass 0.55 --cable-len 0.55
D=results/rig/$(date +%F); mkdir -p $D; git rev-parse HEAD > $D/code.txt; git status --short >> $D/code.txt
rio num_drones:=4 magnet_channel:=6 magnet_initial:=ON          # T1, all session
python3 tools/preflight.py --drones 4 --real --planner mpc_planner --max-ground-z 0.5 --battery-min 23.6
```
- Packs at 23.6 V or more.
- Drone 1 armed but not spinning: `fleet disarm; fleet arm`.
- QUAD2 `pending`: power-cycle it.

Each run: launch in T2, then `fleet arm; fleet takeoff` in T3.

## Runs

**r301: figure-8 at 0.2 m/s.**
- Even ring, plates 0/3/6/9; keep 2.2 m clear along x.
- Lap 35 s. It lands itself after the lap; then `fleet land` to disarm.
```bash
tools/rig_flight.sh r301 real_control_launch.py mode:=mpc num_drones:=4 target_z:=1.0 lift_ramp_vel:=0.15 load_traj:=fig_8 traj_radius:=1.0 traj_speed:=0.2 params_file:=$HOME/multi_drone_control/configs/rig/traj_hold.yaml
```

**r303, r304: unannounced detach on 1/3/5/9, twice.**
- Plates: drone 2 on 1, drone 4 on 3 (it leaves), drone 3 on 5, drone 1 on 9.
- Before takeoff, T2 must show `d4 ... (plate 3)`.
```bash
tools/rig_flight.sh r303 real_control_launch.py mode:=dissipative num_drones:=4 target_z:=1.0 lift_ramp_vel:=0.15 attach_azimuths_deg:=30,90,150,270 params_file:=$HOME/multi_drone_control/configs/rig/detach_unannounced.yaml
```
- Hover steady, then switch drone 4's magnet off without telling the planner:
  `ros2 topic pub -t 3 /drone_3/magnet std_msgs/msg/String "{data: 'OFF'}"`
- Expect in T2 within ~0.5 s: `UNANNOUNCED DETACH: drone 3`. Drone 4 then steps 0.5 m out.
- `fleet land` ~8 s later.
- No `UNANNOUNCED` line within 2 s: send `ros2 topic pub -t 3 /fleet/detach std_msgs/msg/Int32 "{data: 3}"`,
  then land.

**r305: unannounced detach on 1/3/6/9** (only if r303 and r304 went well).
- Drone 3 on plate 6 instead of 5; `attach_azimuths_deg:=30,90,180,270`; otherwise as r303.
- Twin peak 13-19 deg; r0013 capsized on this layout.

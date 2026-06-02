# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ROS 2 (Humble) framework for controlling multiple quadcopter UAVs using Betaflight flight controllers, either in Gazebo Harmonic simulation or real-world via a motion capture (MoCap) system + ExpressLRS (ELRS) radio link.

The active work is the **multi-drone MPC controller** (`controller_mpc_multi`) where a central fleet manager node coordinates 4 drones flying formation circles, each controlled by its own MPC node backed by an [acados](https://github.com/acados/acados) OCP solver.

## Build & Run

```bash
# Build workspace
colcon build --symlink-install
source install/setup.bash

# Build specific packages only
colcon build --packages-select controller_mpc_multi interfaces simulation_communication utility_objects --symlink-install
```

### Multi-Drone Simulation (current focus)
```bash
./run_multi.sh    # Launches everything: Gazebo, ROS bridge, 4 MPC controllers, fleet manager
./stop_multi.sh   # Kill all processes
```

After launch, send fleet commands via the "Fleet Commands" terminal (UP arrow cycles through preloaded history):
```bash
ros2 topic pub --once /fleet/command std_msgs/msg/String "{data: ARM}"
ros2 topic pub --once /fleet/command std_msgs/msg/String "{data: TAKEOFF}"
ros2 topic pub --once /fleet/command std_msgs/msg/String "{data: DISARM}"
ros2 topic pub --once /fleet/command std_msgs/msg/String "{data: ESTOP}"
```

### Single-Drone Simulation
```bash
# Single drone, PID controller
gz sim simulation_assets/world_large.sdf -v -r
ros2 launch simulation_communication betaflight_simulation_launch.py
ros2 run controller_pid main

# Single drone, MPC controller (uses default Betaflight rates)
ros2 launch simulation_communication betaflight_simulation_launch.py
ros2 run controller_mpc main

# With drone_id param (multi-drone bridge)
ros2 launch simulation_communication betaflight_simulation_launch.py num_drones:=4
ros2 run controller_pid main --ros-args -p drone_id:=0
```

### Useful debug commands
```bash
# Send motor speeds directly to Gazebo
gz topic -t /x3_drone0/gazebo/command/motor_speed -m gz.msgs.Actuators -p 'velocity: 1000 velocity: 1000 velocity: 1000 velocity: 1000'

# Echo IMU
gz topic -e -t /world/quadcopter/model/x3_drone0/link/base_link/sensor/x3_drone0_imu_sensor/imu
```

## Architecture

### ROS 2 Topic Namespacing
All drone topics are namespaced under `/drone_<id>/`:
- `/drone_N/ELRSCommand` — control output (roll_rate, pitch_rate, throttle, yaw_rate)
- `/drone_N/motion_capture_state` — pose + velocity state (13-element array: xyz, qwxyz, vxyz, ωxyz)
- `/drone_N/arming_service` — `SetArming` service
- `/drone_N/arming_state_feedback` — Bool published by each controller
- `/drone_N/command` — String topic (ARM/TAKEOFF/DISARM)
- `/fleet/command` — String commands to central controller
- `/fleet/step` — Int32 master step counter broadcast by central controller

### Multi-Drone Control Flow
1. **`run_multi.sh`** starts Gazebo (`world_multi.sdf`), the ROS-Gazebo bridge, pre-compiles the acados solver (drone 0 only, others wait on a file lock at `c_generated_code/.compile.lock`), then launches 4 `controller_mpc_multi controller` nodes and the `controller_mpc_multi main` (fleet manager).
2. **Fleet manager** (`main.py` → `CentralController`) owns the master step clock at 30 Hz and arms/disarms individual drones via their `/drone_N/arming_service`. It publishes `/fleet/step` continuously.
3. **Each MPC controller** (`controller_mpc.py` → `Controller`) runs its own acados OCP solver at 30 Hz. It subscribes to `/fleet/step` to synchronise trajectory position across all drones. Each drone applies a fixed `(x, y)` formation offset (`DRONE_OFFSETS`) when building its circle trajectory so the formation flies congruent circles.
4. **`CallbackManagerMulti`** (`utility_objects/callback_manager_multi.py`) handles all per-drone pub/sub/services and owns the arming service handler.

### Simulation Bridge (`simulation_communication`)
- `betaflight_communication` node: translates `/drone_N/ELRSCommand` to Gazebo motor commands via `ros_gz_bridge`
- `motion_capture_emulator` node: reads Gazebo pose and publishes `/drone_N/motion_capture_state`
- The launch file accepts `num_drones:=N` (default 1)

### State Vector
The 13-element state used throughout: `[x, y, z, qw, qx, qy, qz, vx, vy, vz, ωx, ωy, ωz]`

Trajectory arrays are shape `(17, T)`: 13 states + 4 control inputs `[roll_rate, pitch_rate, throttle, yaw_rate]`.

### acados MPC Solver
- OCP defined in `src/controller_mpc_multi/controller_mpc_multi/acados.py` and `dynamics.py`
- Solver parameters (`est_params`): `[38.0, 0.0, 0.12, 70.0, 670.0, 0.5]` — mass, drag, arm length, Betaflight rate params
- Compiled shared library lives in `c_generated_code/` (gitignored); JSON config in `quad_dynamics_sim.json` / `quad_dynamics_ocp.json`
- Horizon: N=20 steps, skip_steps=3
- Acados must be installed at `~/acados/`

### Utility Classes (`utility_objects`)
- `callback_manager_multi.py` — per-drone scoped ROS 2 pub/sub/services (multi-drone variant)
- `data_logger.py` — CSV log files to `logs/<controller_name>/<trajectory_name>_<timestamp>/log.csv`
- `visualization.py` — RViz trajectory/path/MPC-plan publishers

### Trajectories (`trajectories.py`)
All trajectory functions return `(traj_array, name)`. The `circle_trajectory` function accepts `center_offset_x/y` for formation offsets. Trajectories always begin with a takeoff phase and end with land. Available: `takeoff`, `land`, `hover`, `circle`, `z_sin`, `xyz_sine`.

## Key Files

| Path | Purpose |
|------|---------|
| `run_multi.sh` / `stop_multi.sh` | Launch/stop full 4-drone simulation |
| `src/controller_mpc_multi/controller_mpc_multi/main.py` | Fleet manager (central controller) |
| `src/controller_mpc_multi/controller_mpc_multi/controller_mpc.py` | Per-drone MPC node |
| `src/controller_mpc_multi/controller_mpc_multi/trajectories.py` | Trajectory generators |
| `src/controller_mpc_multi/controller_mpc_multi/acados.py` | OCP setup and warm-start helpers |
| `src/utility_objects/utility_objects/callback_manager_multi.py` | Per-drone ROS interface |
| `src/simulation_communication/launch/betaflight_simulation_launch.py` | Sim bridge launch (num_drones param) |
| `simulation_assets/world_multi.sdf` | 4-drone Gazebo world |
| `c_generated_code/` | Compiled acados solver (auto-generated) |
| `quad_dynamics_sim.json` / `quad_dynamics_ocp.json` | acados OCP config |

## Shared Constants to Keep in Sync
- `FREQUENCY_HZ = 30.0` and `DT` appear in both `main.py` and `controller_mpc.py` — they must match.
- `N_DRONES = 4` in `main.py` must match the `num_drones:=4` argument in `run_multi.sh`.
- `DRONE_OFFSETS` in `controller_mpc.py` defines the formation layout.

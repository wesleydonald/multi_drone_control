# Robust Collective Quadcopter Payload Transport
**Thesis project** (forked from Mitch's repo). This repository provides a complete framework for controlling multiple UAVs tethered to a payload within the UNSW Motion Capture system. It includes a simulator to support at-home development and features an efficient transfer pipeline for transitioning controllers from simulation to real-world deployment.

## What this project is about
A fleet of quadcopters carries a payload together on cables. The goal is to let drones **attach to and detach from the payload mid-flight** — so one can drop out (say it fails, or runs low on battery) and the rest redistribute the load, and a fresh drone can fly in, hook on, and join the team, all without putting the payload down.

Two published methods do the heavy lifting:
- **The OCP / centralised MPC**, from *"Agile and Cooperative Aerial Manipulation of a Cable-Suspended Load"* (Sun et al.) — handles trajectory following and takeoff.
- **The dissipative network**, from *"Self-Organizing Aerial Swarm Robotics: A Table-Mechanics-Inspired Approach"* (Quan et al.) — a decentralised spring-damper "virtual node" scheme where a drone leaving the fleet is just how it normally behaves, rather than a special case.

The thesis contribution is the **adding** side: a free-flying drone with a swung electromagnet rendezvouses with a payload that is already being carried, physically welds onto it, control authority hands over mid-air, and the fleet then reconfigures its load sharing and carries on with the trajectory. Detach works, attach is the part being finished.

## Betaflight UAVs
Most UAVs in this project use a flight controller to act as an interface between the pilot (via remote control) and the motors. This reduces the impact of unmodelled disturbances and simplifies control.

The firmware used is **Betaflight**, popular in the FPV community for its simplicity and robustness. The flight controller receives desired **angular velocity rates and throttle commands**, then—using an onboard gyroscope and a PID controller operating at around 8 kHz—produces open-loop motor commands to achieve the target angular velocity.

Since quadcopter control originated from the fixed-wing community, commands are sent in the following format: [roll_rate, pitch_rate, throttle, yaw_rate]

## ExpressLRS (ELRS)
To transmit commands from the computer to the UAV, a radio link is required. This system uses **ExpressLRS (ELRS)**—a protocol popular in the FPV community for its low latency and high reliability.

A physical transmitter is connected to the computer via USB, and a custom **ROS 2 node** handles command transmission to the drone. As ELRS prioritises high frequency over bandwidth, only small packets (containing up to 16 floats) are sent at a time.

## Motion Capture System
The motion capture system uses **infrared cameras** to detect **retro-reflective markers** attached to the UAV. The captured data is streamed to a lab PC running proprietary software that matches known rigid-body marker configurations with the observed markers.  

This process provides highly accurate (theoretically sub-millimetre) **rigid-body pose estimation**, though it does not output velocity information. The pose data is then streamed to the controller laptop, where a **ROS 2 node** processes it, filters noise, estimates velocities, and produces a complete pose state for feedback control.

---

# Operating
This section outlines the basic interface that allows custom controllers to interact with the system. Controller-specific details are covered in the next section.

## Basic Requirements
- ROS 2 Humble  
- Gazebo Harmonic  

## Dependences/install commands
```bash
sudo apt update
sudo pip3 install transforms3d
sudo apt install ros-humble-tf-transformations
sudo apt remove 'ros-humble-ros-gz-*'
sudo apt install ros-humble-ros-gzharmonic-*

pip install pandas
```

## Building
To build the workspace, run:

```bash
colcon build --symlink-install
source install/setup.bash
```

If you only wish to build specific packages, use:

```bash
colcon build --packages-select controller_quad_load controller_load_mpc controller_dissipative interfaces utility_objects simulation_communication --symlink-install
```

## Visualisation
The **visualisation node** provides the RViz interface and exposes the arming/disarm and takeoff services used to interface with the controller.

```bash
ros2 launch drone_visualisation view_frame.launch.py
```

## Real-World Interface (Motion Capture System)
To connect to a real-world drone, attach the **motion capture PC’s Ethernet connection** and the **ELRS transmitter** to your computer. Once connected and configured, run the following nodes:

```bash
ros2 run drone_communication motion_capture_publisher_node
ros2 run drone_communication elrs_interface
```

## Simulation Interface
The simulation interface consists of two parts: the **simulator** and the **bridge**.  
If you are connecting to a real-world drone, **do not launch these**.

To start the simulation, navigate to the simulation assets directory:

```bash
cd simulation_assets
gz sim world_large.sdf -v -r
```

Two Betaflight configurations are provided:
- **Default Betaflight rates** — non-linear relationship (70, 670, 0.5)
- **Linear rates** — maximum rate of 100°/s with a linear relationship

Launch one of the following depending on your setup:

```bash
ros2 launch simulation_communication betaflight_simulation_launch.py
ros2 launch simulation_communication betaflight_linear_simulation_launch.py
```

---

# Controllers
The packages that matter for this project:

| Package | What it does |
|---|---|
| `controller_load_mpc` | Centralised planner (10 Hz). Solves the coupled load + cable + drone OCP and publishes a reference trajectory per drone. Also owns takeoff. |
| `controller_quad_load` | Per-drone cable-aware MPC tracker (50 Hz) + the fleet manager. All the launch files live here. |
| `controller_dissipative` | The dissipative spring-damper network — detach, attach and reconfiguration. Includes an offline verification harness. |
| `controller_mpc_payload` | The approach MPC that flies the magnet drone in (from a collaborator's stack). |
| `drone_magnet` | Tejen's controller for the join planner, magnet manager, ELRS mux. |

Both reference generators (the OCP planner and the dissipative network) publish the **same message format**, so the trackers don't care which one is driving.

Arm and takeoff from the RViz panel, or by topic — see `AA_Learnings.txt` for the raw commands.

**Two things that will waste your time if you get them wrong:**
1. **Start Gazebo from inside `simulation_assets/`.** The worlds include their drone models by *relative* path (`models/x3_drone0.sdf`), so running `gz sim simulation_assets/foo.sdf` from the repo root fails with `FrameAttachedToGraph unable to find ... x3_drone0::base_link` and the world comes up empty.
2. **Start the RViz launch before the controller launch.** It owns the Gazebo pose bridges and the mocap emulators, so without it the trackers and planner both sit forever on "waiting for mocap".

## Cooperative carry (the OCP baseline)
```bash
cd simulation_assets && gz sim three_rigid_ground.sdf -v4 -r
ros2 launch controller_quad_load rviz_quad_load_launch.py num_drones:=3
ros2 launch controller_quad_load mpc_quad_load_launch.py num_drones:=3 load_traj:=circle
```

## Detach (a drone leaves mid-flight)
```bash
cd simulation_assets && gz sim four_rigid_ground.sdf -v4 -r
ros2 launch controller_quad_load rviz_quad_load_launch.py num_drones:=4 detach:=true
ros2 launch controller_quad_load dissipative_launch.py num_drones:=4
#   ARM -> TAKEOFF -> hit DETACH
```

## Attach (a drone joins mid-flight)
```bash
cd simulation_assets && gz sim three_attach.sdf -v4 -r
ros2 launch controller_quad_load rviz_quad_load_launch.py num_drones:=3 attach:=true
ros2 launch controller_quad_load three_attach_launch.py
#   ARM -> TAKEOFF -> hit ATTACH
```

## Flying the whole thing on the dissipative controller
`dissipative_launch.py` only switches to the network when something actually detaches or attaches. To fly the entire flight on it, use `dissipative_only_launch.py` instead.

## Before every launch
```bash
./tools/clean_slate.sh
```
Leftover nodes from a previous run publish to the same topics and quietly corrupt the results. This kills them, clears the Fast-DDS shared memory, and refuses to pass if anything is still alive.

---

# Utility Classes

Shared between controllers:

- **`utility_objects/callback_manager.py`** — Manages shared ROS 2 publishers, subscribers, and client/service interfaces.  
- **`utility_objects/data_logger.py`** — Provides a simple and consistent interface for creating custom CSV log files.  
- **`utility_objects/visualization.py`** — Contains methods and ROS 2 patterns for visualising drone states and trajectories in RViz.
- **`utility_objects/run_context.py`** — Works out where things get written (results root, acados directories) without anyone hardcoding a path.

Several controllers call `DataLogger`, so if you change how it behaves, change it deliberately — everything that logs is downstream of it.

# Results
Runs land in **`results/`** at the repo root.

- `results/` — every raw run. Not tracked, not backed up off-machine.
- `results_archive/` — the quality runs behind thesis figures. **Tracked and pushed**.

Promote anything you care about as soon as it exists:
```bash
./tools/archive_run.sh results/2026-10-14/R0142_real_attach_circle "Fig 7.3 mid-flight attach"
git add results_archive && git commit -m "Archive R0142" && git push
```

Plot a run with `python3 plot_run.py` (`--list` to see what's available).

---
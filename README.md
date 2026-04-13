

# Introduction
This repository provides a complete framework for controlling various UAVs within the UNSW Motion Capture system. It includes a simulator to support at-home development and features an efficient transfer pipeline for transitioning controllers from simulation to real-world deployment.

## Contribution
This is an active repository and serves as the primary codebase for Mitchell’s PhD project. As such, it may occasionally be unclean, contain experimental files, or have rogue commit messages. However, it also acts as a decent platform for other thesis and student projects.

You are welcome to:
- Create your own controller or interface nodes  
- Add new simulations or utilities  

Please **avoid editing, removing, or altering other people’s work without their approval**.  
You may work directly on the main branch, create your own branch, or fork the repository.

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
colcon build --packages-select controller_pid interfaces drone_communication drone_visualisation simulation_communication utility_objects --symlink-install
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
Several demo controllers are provided as inspiration and reference examples:
- `controller_pid`
- `controller_mpc`
- `controller_rl`

To run a controller, first launch the appropriate simulation, then run the controller node.  
Once the controller is active, open RViz and press **Arm** — the propellers should start spinning. Then press **Takeoff** to begin the trajectory.  

Trajectories are defined in separate files, and several basic trajectories are included.

## controller_pid
```bash
ros2 launch simulation_communication betaflight_linear_simulation_launch.py
ros2 run controller_pid main
```

## controller_mpc
```bash
ros2 launch simulation_communication betaflight_simulation_launch.py
ros2 run controller_mpc main
```

## controller_rl
```bash
ros2 launch simulation_communication betaflight_linear_simulation_launch.py
ros2 run controller_rl main
```

Other controllers within this repository are actively being developed by other students — please **look, but don’t edit**.  
If you wish to create your own, simply copy an existing package, rename it, and continue development in your new package.

---



# Utility Classes

Three utility classes are provided and shared between controllers.  
Please **do not modify or edit existing functions**, but you may add new ones if they are likely to be shared and useful across multiple controllers.

- **`utility_objects/callback_manager.py`** — Manages shared ROS 2 publishers, subscribers, and client/service interfaces.  
- **`utility_objects/data_logger.py`** — Provides a simple and consistent interface for creating custom CSV log files.  
- **`utility_objects/visualization.py`** — Contains methods and ROS 2 patterns for visualising drone states and trajectories in RViz.






---



## Mitchell's simulation  reference 


```bash
ros2 launch drone_visualisation view_frame.launch.py
gz sim -v -r world_drone_env.sdf 
ros2 launch simulation_communication betaflight_simulation_launch.py 
ros2 run controller_ukf main 

ros2 run ros2_orb_slam3 mono_node_cpp 
```

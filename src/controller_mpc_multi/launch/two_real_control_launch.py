"""
two_real_control_launch.py  —  TERMINAL 2 (control)
---------------------------------------------------
Real-world two-drone control layer only: the per-drone MPC controllers + the
central fleet manager. Start this AFTER two_real_io_launch.py (terminal 1) is up
and the MoCap poses are streaming — the controllers block waiting for the first
/drone_<id>/motion_capture_state pose.

  * controller (x2)  — per-drone acados MPC (drone_id 0, 1), reads
    /drone_i/motion_capture_state, publishes /drone_i/ELRSCommand.
  * main             — central fleet manager (num_drones:=2): owns /fleet/step
    and ARM/TAKEOFF/DISARM via /fleet/command.

Run:
    ros2 launch controller_mpc_multi two_real_control_launch.py
then drive the fleet:
    ros2 topic pub --once /fleet/command std_msgs/msg/String "{data: ARM}"
    ros2 topic pub --once /fleet/command std_msgs/msg/String "{data: TAKEOFF}"
    ros2 topic pub --once /fleet/command std_msgs/msg/String "{data: DISARM}"

The controller/fleet nodes pin use_sim_time=False internally (wall clock) —
correct for real hardware (there is no /clock here).
"""
from launch import LaunchDescription
from launch_ros.actions import Node

N = 2

def generate_launch_description():
    nodes = []

    # ── Per-drone MPC controllers ────────────────────────────────────────────
    for i in range(N):
        nodes.append(Node(
            package='controller_mpc_multi',
            executable='controller',
            name=f'controller_{i}',
            parameters=[{'drone_id': i}],
            output='screen',
        ))

    # ── Central fleet manager ────────────────────────────────────────────────
    nodes.append(Node(
        package='controller_mpc_multi',
        executable='main',
        name='central_controller',
        parameters=[{'num_drones': N}],
        output='screen',
    ))

    return LaunchDescription(nodes)

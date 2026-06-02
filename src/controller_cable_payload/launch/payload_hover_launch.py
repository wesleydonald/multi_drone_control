"""
payload_hover_launch.py
-----------------------
Full cable-payload hover stack for world_quad_payload.sdf.

Architecture:
  Gazebo world: lift_system model with 4 drones + payload (no rigid tethers)
  per drone:
    payload_betaflight_comm   — Betaflight inner-loop, nested-model pose topic
    payload_mocap_emulator    — Gazebo pose → MotionCaptureState
    drone_controller          — PD position control + cable compensation
  once:
    planner                   — hover setpoints + ARM/TAKEOFF fleet commands
    cable_tension_node        — spring-damper cable forces via gz transport

Confirmed topic paths (from gz topic --list with world running):
  Drone pose:   /model/lift_system/model/x3_drone{i}/pose
  Motor cmd:    /x3_drone{i}/gazebo/command/motor_speed   (no parent prefix)
  Payload pose: /model/lift_system/model/payload/pose
  Force apply:  /world/quad_payload/wrench/persistent    (gz transport)
"""

from launch import LaunchDescription
from launch_ros.actions import Node

PARENT_MODEL = 'lift_system'
DRONE_NAMES = ['x3_drone0', 'x3_drone1', 'x3_drone2', 'x3_drone3']
N = len(DRONE_NAMES)


def generate_launch_description():
    nodes = []

    # ── Clock bridge ──────────────────────────────────────────────────────────
    nodes.append(Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    ))

    # ── Payload pose bridge ───────────────────────────────────────────────────
    nodes.append(Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='payload_pose_bridge',
        arguments=[
            f'/model/{PARENT_MODEL}/model/payload/pose'
            f'@geometry_msgs/msg/PoseArray[ignition.msgs.Pose_V'
        ],
    ))

    for i, drone_name in enumerate(DRONE_NAMES):
        # ── Drone pose bridge ─────────────────────────────────────────────────
        nodes.append(Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name=f'drone_pose_bridge_{i}',
            arguments=[
                f'/model/{PARENT_MODEL}/model/{drone_name}/pose'
                f'@geometry_msgs/msg/PoseArray[ignition.msgs.Pose_V'
            ],
        ))

        # ── Motor command bridge (confirmed: no parent prefix) ────────────────
        nodes.append(Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name=f'motor_bridge_{i}',
            arguments=[
                f'/{drone_name}/gazebo/command/motor_speed'
                f'@actuator_msgs/msg/Actuators]ignition.msgs.Actuators'
            ],
        ))

        # ── Betaflight inner-loop ─────────────────────────────────────────────
        nodes.append(Node(
            package='controller_cable_payload',
            executable='payload_betaflight_comm',
            name=f'bf_comm_{i}',
            parameters=[{
                'drone_id': i,
                'drone_name': drone_name,
                'parent_model': PARENT_MODEL,
            }],
        ))

        # ── Mocap emulator (drone 0 also publishes payload pose) ──────────────
        nodes.append(Node(
            package='controller_cable_payload',
            executable='payload_mocap_emulator',
            name=f'mocap_{i}',
            parameters=[{
                'drone_id': i,
                'drone_name': drone_name,
                'parent_model': PARENT_MODEL,
                'publish_payload': (i == 0),
            }],
        ))

        # ── Per-drone position controller ─────────────────────────────────────
        nodes.append(Node(
            package='controller_cable_payload',
            executable='drone_controller',
            name=f'drone_ctrl_{i}',
            parameters=[{'drone_id': i}],
        ))

    # ── Central hover planner ─────────────────────────────────────────────────
    nodes.append(Node(
        package='controller_cable_payload',
        executable='planner',
        name='planner',
    ))

    # ── Cable tension simulation (gz transport spring-damper) ─────────────────
    nodes.append(Node(
        package='controller_cable_payload',
        executable='cable_tension_node',
        name='cable_tension',
    ))

    return LaunchDescription(nodes)

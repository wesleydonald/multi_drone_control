"""
payload_hover_launch.py
-----------------------
Launches the full cable-payload hover stack for world_quad_payload.sdf.

Nodes started (per drone 0-3):
  payload_betaflight_comm  — Betaflight inner-loop for nested-model topics
  payload_mocap_emulator   — Pose → MotionCaptureState converter

Nodes started (once):
  planner                  — Central hover planner + fleet commander
  per-drone drone_controller

ROS-Gazebo bridges:
  /model/lift_system/model/x3_drone{i}/pose  →  PoseArray
  /lift_system/x3_drone{i}/gazebo/command/motor_speed  →  Actuators
  /model/lift_system/model/payload/pose  →  PoseArray
  /clock

NOTE: The motor command topic path for nested Gazebo models may need
      adjustment after verifying with `gz topic --list` once Gazebo is
      running.  The expected path is:
        /lift_system/x3_drone{i}/gazebo/command/motor_speed
      If that is wrong, update the 'control_bridge_{i}' argument below.
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

        # ── Motor command bridge ──────────────────────────────────────────────
        nodes.append(Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name=f'motor_bridge_{i}',
            arguments=[
                f'/{PARENT_MODEL}/{drone_name}/gazebo/command/motor_speed'
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

        # ── Per-drone controller ──────────────────────────────────────────────
        nodes.append(Node(
            package='controller_cable_payload',
            executable='drone_controller',
            name=f'drone_ctrl_{i}',
            parameters=[{'drone_id': i}],
        ))

    # ── Central planner ───────────────────────────────────────────────────────
    nodes.append(Node(
        package='controller_cable_payload',
        executable='planner',
        name='planner',
    ))

    return LaunchDescription(nodes)

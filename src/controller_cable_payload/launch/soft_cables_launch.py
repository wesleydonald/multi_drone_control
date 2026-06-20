"""
soft_cables_launch.py
---------------------
Launch stack for world_soft_cables.sdf — 4 drones lifting a payload via
NON-RIGID segmented cables (flexible chains modelled directly in the SDF).

Unlike rigid_cables_launch.py there is NO cable_tension_node: the cables are
real jointed link-chains in the world, so tension is computed by the physics
engine every step.  Unlike the rigid world, the cables can go slack and the
drones sit roughly above their attach points, so thrust becomes lift.

Per drone:
  payload_betaflight_comm   — Betaflight inner-loop (nested-model pose topic)
  payload_mocap_emulator    — Gazebo pose → MotionCaptureState
  mpc_drone_controller      — acados MPC (reused from controller_mpc_multi)
Once:
  planner                   — relays /fleet/command (ARM/TAKEOFF/DISARM)

Topic paths (parent model 'lift_system'):
  Drone pose:   /model/lift_system/model/x3_drone{i}/pose
  Motor cmd:    /x3_drone{i}/gazebo/command/motor_speed
  Payload pose: /model/lift_system/model/payload/pose
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

        # ── Per-drone MPC controller ──────────────────────────────────────────
        nodes.append(Node(
            package='controller_cable_payload',
            executable='mpc_drone_controller',
            name=f'drone_ctrl_{i}',
            parameters=[{'drone_id': i}],
        ))

    # ── Central planner (ARM / TAKEOFF / DISARM relay) ────────────────────────
    nodes.append(Node(
        package='controller_cable_payload',
        executable='planner',
        name='planner',
        parameters=[{'tether_length': 0.0}],   # MPC ignores setpoints; no projection
    ))

    return LaunchDescription(nodes)

"""
two_no_cables_launch.py
-----------------------
ROS 2 launch file for two_no_cables.sdf — 2-drone system with a FREE,
unattached payload (no cables, no tethers).

Brings up the full sensing + actuation pipeline for two drones so the
state/command path can be verified before controller work:
  bridges:
    clock_bridge          — /clock
    payload_pose_bridge    — /model/lift_system/model/payload/pose  (Pose_V)
    drone_pose_bridge_{i}  — /model/lift_system/model/x3_drone{i}/pose
    motor_bridge_{i}       — /x3_drone{i}/gazebo/command/motor_speed
  per drone:
    payload_betaflight_comm — Betaflight inner-loop (ELRSCommand -> motors)
    payload_mocap_emulator  — Gazebo pose -> MotionCaptureState
    drone_controller        — PD position control

World geometry (two_no_cables.sdf):
    x3_drone0 : (0.0, 0.5, 0.1)
    x3_drone1 : (1.0, 0.5, 0.1)
    payload   : (0.5, 0.5, 0.025)   free body, not attached

NOTE — no `planner` node here:
  The `planner` executable is currently hardcoded to N_DRONES = 4 (it loops
  range(N_DRONES) and addresses /drone_0../drone_3), so it cannot drive a
  2-drone world unmodified. Once it is parameterised for drone count, add it
  back here. Until then, drive setpoints / ARM / TAKEOFF manually for testing.

NOTE — no `cable_tension_node`: this world has no cables.
"""

from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter

PARENT_MODEL = 'lift_system'
DRONE_NAMES  = ['x3_drone0', 'x3_drone1']
N            = len(DRONE_NAMES)


def generate_launch_description():
    # Drive every node off Gazebo's /clock so velocity dt (finite-differenced
    # in the mocap emulator) stays correct even when RTF < 1 (e.g. cable worlds
    # at ~50%). Applies use_sim_time=true to all nodes below.
    nodes = [SetParameter(name='use_sim_time', value=True)]

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
                'drone_id':     i,
                'drone_name':   drone_name,
                'parent_model': PARENT_MODEL,
            }],
        ))

        # ── Mocap emulator (drone 0 also publishes payload pose) ──────────────
        nodes.append(Node(
            package='controller_cable_payload',
            executable='payload_mocap_emulator',
            name=f'mocap_{i}',
            parameters=[{
                'drone_id':        i,
                'drone_name':      drone_name,
                'parent_model':    PARENT_MODEL,
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

    return LaunchDescription(nodes)

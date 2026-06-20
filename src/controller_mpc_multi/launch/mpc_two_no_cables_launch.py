"""
mpc_two_no_cables_launch.py
---------------------------
MPC control stack for two_no_cables.sdf — 2 drones, free (unattached) payload.

Reuses controller_mpc_multi (the per-drone MPC + central fleet manager) at
N=2, sitting on top of the nested-lift_system bridge nodes that now live in
simulation_communication:
  bridges (per drone + payload):
    clock_bridge / payload_pose_bridge / drone_pose_bridge_{i} / motor_bridge_{i}
    payload_betaflight_comm  — ELRSCommand -> motor_speed
    payload_mocap_emulator   — Gazebo pose -> /drone_{i}/motion_capture_state
  control:
    controller (x2)          — per-drone acados MPC (drone_id 0,1)
    main                     — central fleet manager (num_drones:=2)

Run Gazebo first (separate terminal):
    gz sim two_no_cables.sdf -v 4 -r
then:
    ros2 launch controller_mpc_multi mpc_two_no_cables_launch.py
then drive the fleet:
    ros2 topic pub --once /fleet/command std_msgs/msg/String "{data: ARM}"
    ros2 topic pub --once /fleet/command std_msgs/msg/String "{data: TAKEOFF}"

Notes:
  * SetParameter(use_sim_time=True) below applies to the BRIDGES so the mocap
    emulator's finite-differenced velocity uses sim time (correct at any RTF).
    The MPC controller / fleet nodes pin use_sim_time=False internally via
    parameter_overrides (existing design — their 30 Hz loop / step clock runs
    on wall time); revisit for low-RTF cable worlds.
  * acados compilation is serialised by a file lock inside controller_mpc.py,
    so the two controllers starting together is safe (one compiles, the other
    waits).
"""

from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter

PARENT_MODEL = 'lift_system'
DRONE_NAMES  = ['x3_drone0', 'x3_drone1']
N            = len(DRONE_NAMES)


def generate_launch_description():
    # use_sim_time for the bridge nodes (mocap velocity dt). MPC nodes override
    # this internally, so it only affects the bridges — which is what we want.
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
            package='simulation_communication',
            executable='payload_betaflight_comm',
            name=f'bf_comm_{i}',
            parameters=[{
                'drone_id':     i,
                'drone_name':   drone_name,
                'parent_model': PARENT_MODEL,
            }],
        ))

        # ── Mocap emulator (drone 0 also publishes the payload pose) ──────────
        nodes.append(Node(
            package='simulation_communication',
            executable='payload_mocap_emulator',
            name=f'mocap_{i}',
            parameters=[{
                'drone_id':        i,
                'drone_name':      drone_name,
                'parent_model':    PARENT_MODEL,
                'publish_payload': (i == 0),
            }],
        ))

        # ── Per-drone MPC controller ──────────────────────────────────────────
        nodes.append(Node(
            package='controller_mpc_multi',
            executable='controller',
            name=f'controller_{i}',
            parameters=[{'drone_id': i}],
            output='screen',
        ))

    # ── Central fleet manager ─────────────────────────────────────────────────
    nodes.append(Node(
        package='controller_mpc_multi',
        executable='main',
        name='central_controller',
        parameters=[{'num_drones': N}],
        output='screen',
    ))

    return LaunchDescription(nodes)

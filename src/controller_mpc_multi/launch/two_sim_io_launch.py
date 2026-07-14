"""
two_sim_io_launch.py  —  TERMINAL 1 (sim I/O + visualisation)
-------------------------------------------------------------
Simulation twin of two_real_io_launch.py. Splits mpc_two_no_cables_launch.py into
an I/O + viz layer (this file, terminal 1) and a control layer
(two_sim_control_launch.py, terminal 2).

This terminal owns everything between Gazebo and the controllers, plus RViz:
  * Gazebo <-> ROS bridges: clock, per-drone pose, per-drone motor, payload pose.
  * payload_betaflight_comm (x2)  — /drone_i/ELRSCommand -> motor_speed.
  * payload_mocap_emulator   (x2) — Gazebo pose -> /drone_i/motion_capture_state,
    and (new) TF map -> drone_i/base_link so RViz can draw the drone MESH.
  * robot_state_publisher    (x2) — serves the tbs_sourceone_v5 URDF per drone on
    /robot_description_i (red = drone 0, blue = drone 1) for the RViz RobotModel.
  * rviz2 (toggle rviz:=false)     — two_sim.rviz: drone models + TF + trajectory
    paths + the ArmPanel (ARM / TAKEOFF / DISARM via /fleet/command).

Everything here runs on sim time (use_sim_time:=true) so the mocap velocity
finite-difference and the TF timestamps agree with RViz's /clock.

Run Gazebo first (separate terminal):
    gz sim two_no_cables.sdf -v 4 -r
then, terminal 1:
    ros2 launch controller_mpc_multi two_sim_io_launch.py
then, terminal 2 (controllers + fleet manager):
    ros2 launch controller_mpc_multi two_sim_control_launch.py
Arm / take off from the RViz ArmPanel, or:
    ros2 topic pub --once /fleet/command std_msgs/msg/String "{data: ARM}"
    ros2 topic pub --once /fleet/command std_msgs/msg/String "{data: TAKEOFF}"
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from ament_index_python.packages import get_package_share_directory

PARENT_MODEL = 'lift_system'
DRONE_NAMES  = ['x3_drone0', 'x3_drone1']
N            = len(DRONE_NAMES)

# Per-drone RViz mesh colour (r, g, b, a).
DRONE_COLORS = [
    (1.0, 0.0, 0.0, 0.9),   # drone 0 — red
    (0.0, 0.4, 1.0, 0.9),   # drone 1 — blue
]


def _drone_description(base_urdf: str, i: int) -> str:
    """Per-drone URDF: unique root link name + colour so each RobotModel is distinct."""
    r, g, b, a = DRONE_COLORS[i % len(DRONE_COLORS)]
    desc = base_urdf.replace('name="base_link"', f'name="drone_{i}/base_link"')
    desc = desc.replace(
        '<color rgba="0.15 0.15 0.15 1.0"/>',
        f'<color rgba="{r} {g} {b} {a}"/>')
    return desc


def generate_launch_description():
    viz_share = get_package_share_directory('drone_visualisation')
    with open(os.path.join(viz_share, 'urdf', 'frame.urdf'), 'r') as f:
        base_urdf = f.read()

    # Sim time for the whole I/O + viz layer (bridges, mocap dt, TF stamps, RViz).
    nodes = [
        SetParameter(name='use_sim_time', value=True),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='open RViz2 (drone models + ArmPanel)'),
    ]

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
        #   broadcast_tf:=true -> TF map -> drone_i/base_link for the RViz model.
        nodes.append(Node(
            package='simulation_communication',
            executable='payload_mocap_emulator',
            name=f'mocap_{i}',
            parameters=[{
                'drone_id':        i,
                'drone_name':      drone_name,
                'parent_model':    PARENT_MODEL,
                'publish_payload': (i == 0),
                'broadcast_tf':    True,
                'tf_parent_frame': 'map',
                'tf_child_frame':  f'drone_{i}/base_link',
            }],
        ))

        # ── RViz drone model: serve the URDF on /robot_description_i ───────────
        nodes.append(Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name=f'robot_state_publisher_{i}',
            output='screen',
            parameters=[{'robot_description': _drone_description(base_urdf, i)}],
            remappings=[('robot_description', f'/robot_description_{i}')],
        ))

    # ── RViz2 (toggle rviz:=false) ────────────────────────────────────────────
    rviz_args = []
    cfg = os.path.join(viz_share, 'rviz', 'two_sim.rviz')
    if os.path.exists(cfg):
        rviz_args = ['-d', cfg]
    nodes.append(Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=rviz_args, output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
    ))

    return LaunchDescription(nodes)

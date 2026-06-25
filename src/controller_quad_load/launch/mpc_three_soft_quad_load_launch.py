"""
mpc_three_soft_quad_load_launch.py
----------------------------------
PLANNER-DRIVEN, CABLE-AWARE stack for three_soft.sdf: the centralized
cable-suspended load planner (controller_load_mpc) generates each drone's
reference trajectory AND its per-node cable tension acceleration (t*s/m), and the
per-drone cable-aware MPC (controller_quad_load) tracks it with the cable force in
its prediction model (reference_source:=planner).

vs controller_mpc_multi/mpc_three_soft_planner_launch.py (cable-blind tracker),
this swaps the controllers + fleet manager to the controller_quad_load package.

Run Gazebo first:
    gz sim simulation_assets/three_soft.sdf -v 4 -r
then:
    ros2 launch controller_quad_load mpc_three_soft_quad_load_launch.py
The planner streams references continuously; ARM + TAKEOFF via /fleet/command
hands the drones over to tracking it:
    ros2 topic pub --once /fleet/command std_msgs/msg/String "{data: ARM}"
    ros2 topic pub --once /fleet/command std_msgs/msg/String "{data: TAKEOFF}"

Notes:
  * planner codegen is isolated (c_generated_code_load_planner/), and the
    cable-aware tracker codegen is isolated (c_generated_code_quad_load/), so
    nothing clashes with controller_mpc_multi's quad_dynamics solver.
  * until the planner streams a reference, controllers hold armed-idle.
"""

from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter

PARENT_MODEL = 'lift_system'
DRONE_NAMES  = ['x3_drone0', 'x3_drone1', 'x3_drone2']
N            = len(DRONE_NAMES)


def generate_launch_description():
    nodes = [SetParameter(name='use_sim_time', value=True)]

    # ── Clock + payload pose bridges ───────────────────────────────────────
    nodes.append(Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'], output='screen'))
    nodes.append(Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='payload_pose_bridge',
        arguments=[f'/model/{PARENT_MODEL}/model/payload/pose'
                   f'@geometry_msgs/msg/PoseArray[ignition.msgs.Pose_V']))

    for i, drone_name in enumerate(DRONE_NAMES):
        nodes.append(Node(
            package='ros_gz_bridge', executable='parameter_bridge',
            name=f'drone_pose_bridge_{i}',
            arguments=[f'/model/{PARENT_MODEL}/model/{drone_name}/pose'
                       f'@geometry_msgs/msg/PoseArray[ignition.msgs.Pose_V']))
        nodes.append(Node(
            package='ros_gz_bridge', executable='parameter_bridge',
            name=f'motor_bridge_{i}',
            arguments=[f'/{drone_name}/gazebo/command/motor_speed'
                       f'@actuator_msgs/msg/Actuators]ignition.msgs.Actuators']))
        nodes.append(Node(
            package='simulation_communication', executable='payload_betaflight_comm',
            name=f'bf_comm_{i}',
            parameters=[{'drone_id': i, 'drone_name': drone_name,
                         'parent_model': PARENT_MODEL}]))
        nodes.append(Node(
            package='simulation_communication', executable='payload_mocap_emulator',
            name=f'mocap_{i}',
            parameters=[{'drone_id': i, 'drone_name': drone_name,
                         'parent_model': PARENT_MODEL, 'publish_payload': (i == 0)}]))
        # per-drone CABLE-AWARE MPC tracker — track the planner reference
        nodes.append(Node(
            package='controller_quad_load', executable='controller',
            name=f'controller_{i}',
            parameters=[{'drone_id': i, 'reference_source': 'planner'}],
            output='screen'))

    # ── Central fleet manager ──────────────────────────────────────────────
    nodes.append(Node(
        package='controller_quad_load', executable='main', name='central_controller',
        parameters=[{'num_drones': N}], output='screen'))

    # ── Centralized cable-suspended load planner ───────────────────────────
    nodes.append(Node(
        package='controller_load_mpc', executable='planner', name='load_planner',
        output='screen'))

    return LaunchDescription(nodes)

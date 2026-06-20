"""
rigid_cables_launch.py
----------------------
ROS 2 launch file for world_multi_cables_2.sdf — 4-drone payload system
with RIGID SDF ball-joint tethers (no cable_tension_node needed).

World geometry (payload at centre of 1 m square drone formation):
  Drone positions on the ground:
    x3_drone0: (0.0,  0.0, 0.1)
    x3_drone1: (1.0,  0.0, 0.1)
    x3_drone2: (0.0,  1.0, 0.1)
    x3_drone3: (1.0,  1.0, 0.1)
  Payload: (0.5, 0.5, 0.025)  [centre of the square]
  Tether length: 0.707 m (horizontal at rest)

Hover geometry (rigid tether at 35° diagonal, ±0.25 m XY formation):
  Horizontal drone-payload offset: sqrt(0.25²+0.25²) = 0.354 m
  Vertical component: sqrt(0.707² - 0.354²) = 0.612 m
  → Drones hover at  payload_z + 0.612 m

Step 1 — verify gz (motors, no ROS needed):
  gz topic -t /x3_drone0/gazebo/command/motor_speed \\
           --msgtype gz.msgs.Actuators -p 'velocity:[700,700,700,700]'

Step 2 — bridge verification (launch this file, then check):
  ros2 topic echo /drone_0/motion_capture_state
  ros2 topic echo /payload/motion_capture_state
  ros2 topic pub --once /x3_drone0/gazebo/command/motor_speed \\
    actuator_msgs/msg/Actuators '{velocity: [700.0,700.0,700.0,700.0]}'

Step 3 — run the controller:
  ros2 topic pub --once /fleet/command std_msgs/msg/String "{data: ARM}"
  ros2 topic pub --once /fleet/command std_msgs/msg/String "{data: TAKEOFF}"
"""

from launch import LaunchDescription
from launch_ros.actions import Node

PARENT_MODEL = 'lift_system'
DRONE_NAMES  = ['x3_drone0', 'x3_drone1', 'x3_drone2', 'x3_drone3']
N            = len(DRONE_NAMES)

# Physical tether length in world_multi_cables_2.sdf (SDF joint cylinder length).
# The planner projects every setpoint onto the sphere of this radius so the
# controller never fights the rigid-joint constraint.
TETHER_LENGTH = 0.707

# Desired hover direction for each drone from the payload centre.
# These are the unconstrained offsets — they are normalised + scaled to
# TETHER_LENGTH by the sphere projection, so only the DIRECTION matters here.
# Drone order matches world_multi_cables_2.sdf: drone0 at (-x,-y), etc.
OFFSETS_X = [-0.25,  0.25, -0.25,  0.25]
OFFSETS_Y = [-0.25, -0.25,  0.25,  0.25]

# How high above the payload we want the drones (sets the "up" component of the
# target direction before projection).  Larger = steeper cable angle.
CABLE_LENGTH_VERT = 0.612

# Desired payload hover height (m above ground).
PAYLOAD_HOVER_Z = 0.5


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
                'drone_id':     i,
                'drone_name':   drone_name,
                'parent_model': PARENT_MODEL,
            }],
        ))

        # ── Mocap emulator ────────────────────────────────────────────────────
        nodes.append(Node(
            package='controller_cable_payload',
            executable='payload_mocap_emulator',
            name=f'mocap_{i}',
            parameters=[{
                'drone_id':       i,
                'drone_name':     drone_name,
                'parent_model':   PARENT_MODEL,
                'publish_payload': (i == 0),
            }],
        ))

        # ── Per-drone MPC controller (acados, reused from controller_mpc_multi) ─
        nodes.append(Node(
            package='controller_cable_payload',
            executable='mpc_drone_controller',
            name=f'drone_ctrl_{i}',
            parameters=[{'drone_id': i}],
        ))

    # ── Central hover planner (world-specific geometry) ───────────────────────
    nodes.append(Node(
        package='controller_cable_payload',
        executable='planner',
        name='planner',
        parameters=[{
            'cable_length':    CABLE_LENGTH_VERT,
            'payload_hover_z': PAYLOAD_HOVER_Z,
            'drone_offsets_x': OFFSETS_X,
            'drone_offsets_y': OFFSETS_Y,
            'tether_length':   TETHER_LENGTH,
        }],
    ))

    # NOTE: cable_tension_node is NOT launched here — tethers are rigid SDF
    # ball joints in world_multi_cables_2.sdf, so no gz-transport wrenches needed.

    return LaunchDescription(nodes)

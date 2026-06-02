from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_drones(context):
    N = int(LaunchConfiguration('num_drones').perform(context))
    nodes = []

    # nodes.append(
    #     Node(
    #         package='ros_gz_bridge',
    #         executable='parameter_bridge',
    #         name='payload_pose_bridge',
    #         arguments=[
    #             '/model/lift_system/model/payload/pose@geometry_msgs/msg/PoseArray[ignition.msgs.Pose_V'
    #         ]
    #     )
    # )
    for i in range(N):
        nodes.append(
            Node(
                package='simulation_communication',
                executable='betaflight_communication',
                name=f'betaflight_{i}',
                parameters=[{'drone_id': i}]
            )
        )
        nodes.append(
            Node(
                package='simulation_communication',
                executable='motion_capture_emulator',
                name=f'mocap_{i}',
                parameters=[
                    {'drone_id': i}
                ]
            )
        )
        nodes.append(
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name=f'control_bridge_{i}',
                arguments=[f'/x3_drone{i}/gazebo/command/motor_speed@actuator_msgs/msg/Actuators]ignition.msgs.Actuators']
            )
        )
        nodes.append(
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name=f'pose_bridge_{i}',
                arguments=[
                    f'/model/x3_drone{i}/pose@geometry_msgs/msg/PoseArray[ignition.msgs.Pose_V'
                ]
            )
            # Node(
            #     package='ros_gz_bridge',
            #     executable='parameter_bridge',
            #     name=f'pose_bridge_{i}',
            #     arguments=[
            #         f'/model/quad_lift_system/pose@geometry_msgs/msg/PoseArray[ignition.msgs.Pose_V'
            #     ]
            # ) for when the whole thing is one model cables.sdf
            # Node(
            #     package='ros_gz_bridge',
            #     executable='parameter_bridge',
            #     name=f'pose_bridge_{i}',
            #     arguments=[
            #         f'/model/lift_system/model/x3_drone{i}/pose'
            #         f'@geometry_msgs/msg/PoseArray'
            #         f'[ignition.msgs.Pose_V'
            #     ]
            # )
        )
        
    return nodes


def generate_launch_description():

    num_drones = LaunchConfiguration('num_drones')

    declare_num_drones = DeclareLaunchArgument(
        'num_drones',
        default_value='1'
    )

    return LaunchDescription([

        declare_num_drones,

        OpaqueFunction(function=launch_drones),

        # ROS2 pose bridge
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='pose_bridge',
            arguments=['/world/quadcopter/dynamic_pose/info@geometry_msgs/msg/PoseArray[ignition.msgs.Pose_V']
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='pose_bridgey3',
            arguments=['/model/pendulum/pose@geometry_msgs/msg/PoseArray[ignition.msgs.Pose_V']
        ),

        Node(
            package='simulation_communication',
            executable='pendulum_state_listener',
            name='pendulum_state_listener'
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='clock_bridge',
            arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
            output='screen'
        ),

    ])
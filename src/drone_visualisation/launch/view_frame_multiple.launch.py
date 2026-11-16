from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('drone_visualisation')
    urdf_path = os.path.join(pkg_share, 'urdf', 'frame.urdf')
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'default.rviz')

    # Read base URDF
    with open(urdf_path, 'r') as f:
        base_robot_description = f.read()

    # Define drones with positions matching Gazebo
    robots = [
        {
            'name': 'x3_drone0',
            'tf_prefix': 'drone0',
            'robot_description_topic': 'robot_description0',
            'color': {'r': 1.0, 'g': 0.0, 'b': 0.0, 'a': 0.7},  # Red
            'position': [0.0, 0.0, 0.1]
        },
        {
            'name': 'x3_drone1',
            'tf_prefix': 'drone1',
            'robot_description_topic': 'robot_description1',
            'color': {'r': 0.0, 'g': 0.0, 'b': 1.0, 'a': 0.7},  # Blue
            'position': [1.0, 0.0, 0.1]
        },
        {
            'name': 'x3_drone2',
            'tf_prefix': 'drone2',
            'robot_description_topic': 'robot_description2',
            'color': {'r': 1.0, 'g': 0.0, 'b': 0.0, 'a': 0.7},  # Red
            'position': [0.0, 1.0, 0.1]
        },
        {
            'name': 'x3_drone3',
            'tf_prefix': 'drone3',
            'robot_description_topic': 'robot_description3',
            'color': {'r': 0.0, 'g': 0.0, 'b': 1.0, 'a': 0.7},  # Blue
            'position': [1.0, 1.0, 0.1]
        }
    ]

    nodes = []
    for robot in robots:
        # Replace base_link name with unique tf_prefix
        robot_description = base_robot_description.replace(
            'name="base_link"',
            f'name="{robot["tf_prefix"]}/base_link"'
        )

        # Replace material color
        color = robot['color']
        robot_description = robot_description.replace(
            '<color rgba="0.15 0.15 0.15 1.0"/>',
            f'<color rgba="{color["r"]} {color["g"]} {color["b"]} {color["a"]}"/>'
        )

        # Robot state publisher
        nodes.append(
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name=f'robot_state_publisher_{robot["name"]}',
                namespace=robot['name'],
                output='screen',
                parameters=[{
                    'robot_description': robot_description,
                    'tf_prefix': robot['tf_prefix']
                }],
                remappings=[
                    ('robot_description', robot['robot_description_topic'])
                ]
            )
        )

        # Static transform to world
        x, y, z = robot['position']
        nodes.append(
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name=f'static_tf_{robot["name"]}',
                arguments=[str(x), str(y), str(z), '0', '0', '0', 'world', f'{robot["tf_prefix"]}/base_link']
            )
        )

    # RViz node
    nodes.append(
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path]
        )
    )

    return LaunchDescription(nodes)
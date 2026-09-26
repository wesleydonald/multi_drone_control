from datetime import datetime
from pathlib import Path
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _workspace_root() -> Path:
    return Path(get_package_prefix('tejen_dynamic_planner')).resolve().parents[1]


def generate_launch_description():
    repo = _workspace_root()
    run_dir = repo / 'logs' / 'tejen_dynamic_planner' / f"irl_virtual_static_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    run_dir.mkdir(parents=True, exist_ok=True)
    config = Path(get_package_share_directory('tejen_dynamic_planner')) / 'config' / 'c1f_irl_virtual_static.yaml'
    args = [
        DeclareLaunchArgument('planner_rate_hz', default_value='5.0'),
        DeclareLaunchArgument('minimum_search_z_m', default_value='0.20'),
        DeclareLaunchArgument('obstacle_x', default_value='0.50'),
        DeclareLaunchArgument('obstacle_y', default_value='0.00'),
        DeclareLaunchArgument('obstacle_z', default_value='0.55'),
        DeclareLaunchArgument('obstacle_half_x', default_value='0.12'),
        DeclareLaunchArgument('obstacle_half_y', default_value='0.20'),
        DeclareLaunchArgument('obstacle_half_z', default_value='0.10'),
        DeclareLaunchArgument('obstacle_yaw_rad', default_value='0.0'),
        LogInfo(msg='IRL VIRTUAL static-obstacle commissioning: the physical flight volume must remain empty.'),
        LogInfo(msg=f'IRL C++ log directory: {run_dir}'),
    ]
    overrides = {
        'planner_rate_hz': ParameterValue(LaunchConfiguration('planner_rate_hz'), value_type=float),
        'minimum_search_z_m': ParameterValue(LaunchConfiguration('minimum_search_z_m'), value_type=float),
        'obstacle_center_x_m': ParameterValue(LaunchConfiguration('obstacle_x'), value_type=float),
        'obstacle_center_y_m': ParameterValue(LaunchConfiguration('obstacle_y'), value_type=float),
        'obstacle_center_z_m': ParameterValue(LaunchConfiguration('obstacle_z'), value_type=float),
        'obstacle_half_x_m': ParameterValue(LaunchConfiguration('obstacle_half_x'), value_type=float),
        'obstacle_half_y_m': ParameterValue(LaunchConfiguration('obstacle_half_y'), value_type=float),
        'obstacle_half_z_m': ParameterValue(LaunchConfiguration('obstacle_half_z'), value_type=float),
        'obstacle_yaw_rad': ParameterValue(LaunchConfiguration('obstacle_yaw_rad'), value_type=float),
        'csv_path': str(run_dir / 'backend.csv'),
        'replan_csv_path': str(run_dir / 'replans.csv'),
    }
    args.append(Node(
        package='tejen_dynamic_planner', executable='dynamic_planner_transfer_backend',
        name='dynamic_planner_transfer_backend', output='screen',
        parameters=[str(config), overrides],
    ))
    return LaunchDescription(args)

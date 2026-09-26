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
    run_dir = repo / 'logs' / 'tejen_dynamic_planner' / f"irl_clear_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    run_dir.mkdir(parents=True, exist_ok=True)
    config = Path(get_package_share_directory('tejen_dynamic_planner')) / 'config' / 'c1f_irl_clear.yaml'
    return LaunchDescription([
        DeclareLaunchArgument('planner_rate_hz', default_value='5.0'),
        DeclareLaunchArgument('minimum_search_z_m', default_value='0.20'),
        LogInfo(msg='IRL clear-space C++ transfer: loaded geometry ON, static/cooperative obstacle scenes OFF.'),
        LogInfo(msg='This launch is for clear-space commissioning before any physical obstacle.'),
        LogInfo(msg=f'IRL C++ log directory: {run_dir}'),
        Node(
            package='tejen_dynamic_planner', executable='dynamic_planner_transfer_backend',
            name='dynamic_planner_transfer_backend', output='screen',
            parameters=[str(config), {
                'planner_rate_hz': ParameterValue(LaunchConfiguration('planner_rate_hz'), value_type=float),
                'minimum_search_z_m': ParameterValue(LaunchConfiguration('minimum_search_z_m'), value_type=float),
                'csv_path': str(run_dir / 'backend.csv'),
                'replan_csv_path': str(run_dir / 'replans.csv'),
            }],
        ),
    ])

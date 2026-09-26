"""Launch the current C1F.8d moving-target backend for M1 IRL commissioning."""
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


def generate_launch_description() -> LaunchDescription:
    root = _workspace_root()
    dynamic_share = Path(get_package_share_directory('tejen_dynamic_planner'))
    drone_share = Path(get_package_share_directory('tejen_mission'))
    config = dynamic_share / 'config' / 'c1f_irl_m1_virtual_ring.yaml'
    lab_config = drone_share / 'config' / 'irl_commissioning.yaml'
    run_dir = (
        root
        / 'logs'
        / 'tejen_dynamic_planner'
        / f"irl_m1_virtual_ring_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    )
    run_dir.mkdir(parents=True, exist_ok=True)

    return LaunchDescription([
        DeclareLaunchArgument('planner_rate_hz', default_value='5.0'),
        DeclareLaunchArgument('minimum_search_z_m', default_value='0.20'),
        LogInfo(msg='M1 IRL backend: current C1F.8d moving-target contract, virtual ring, no cooperative companions.'),
        LogInfo(msg=f'IRL C++ log directory: {run_dir}'),
        Node(
            package='tejen_dynamic_planner',
            executable='dynamic_planner_transfer_backend',
            name='dynamic_planner_transfer_backend',
            output='screen',
            # Order matters: lab config overrides only measured carried-object geometry.
            parameters=[
                str(config),
                str(lab_config),
                {
                    'planner_rate_hz': ParameterValue(
                        LaunchConfiguration('planner_rate_hz'), value_type=float
                    ),
                    'minimum_search_z_m': ParameterValue(
                        LaunchConfiguration('minimum_search_z_m'), value_type=float
                    ),
                    'csv_path': str(run_dir / 'backend.csv'),
                    'replan_csv_path': str(run_dir / 'replans.csv'),
                },
            ],
        ),
    ])

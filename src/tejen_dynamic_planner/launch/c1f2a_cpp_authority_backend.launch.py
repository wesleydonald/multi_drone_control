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


def _new_run_directory(repo_root: Path) -> Path:
    log_root = repo_root / 'logs' / 'tejen_dynamic_planner'
    log_root.mkdir(parents=True, exist_ok=True)
    stem = f"c1f2a_authority_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    run_dir = log_root / stem
    suffix = 1
    while run_dir.exists():
        run_dir = log_root / f'{stem}_{suffix}'
        suffix += 1
    run_dir.mkdir(parents=True)
    return run_dir


def generate_launch_description():
    repo_root = _workspace_root()
    run_dir = _new_run_directory(repo_root)
    config = (
        Path(get_package_share_directory('tejen_dynamic_planner'))
        / 'config'
        / 'c1f0_transfer_backend.yaml'
    )
    planner_rate = LaunchConfiguration('planner_rate_hz')
    minimum_search_z_m = LaunchConfiguration('minimum_search_z_m')

    return LaunchDescription([
        DeclareLaunchArgument('planner_rate_hz', default_value='5.0'),
        DeclareLaunchArgument('minimum_search_z_m', default_value='0.20'),
        LogInfo(msg='C1F.2a: C++ authority on TRANSIT_TO_DROP_POINT, Python remains sole MPC-topic publisher.'),
        LogInfo(msg='C1F.2a: moving-target velocity lead + terminal-hold reference recovery enabled.'),
        LogInfo(msg=f'C1F.2a C++ log directory: {run_dir}'),
        Node(
            package='tejen_dynamic_planner',
            executable='dynamic_planner_transfer_backend',
            name='dynamic_planner_transfer_backend',
            output='screen',
            parameters=[
                str(config),
                {
                    'planner_rate_hz': ParameterValue(planner_rate, value_type=float),
                    'minimum_search_z_m': ParameterValue(minimum_search_z_m, value_type=float),
                    'static_scene_enabled': False,
                    'require_scene_witness': False,
                    'require_stationary_activation': False,
                    'seed_stationary_on_activation': False,
                    'allow_live_target_updates': True,
                    'live_target_replan_threshold_m': 0.01,
                    'target_velocity_topic': '/dynamic_planner/transfer_target_velocity',
                    'require_target_velocity': True,
                    'target_lead_enabled': True,
                    'target_lead_nominal_speed_mps': 0.35,
                    'target_lead_min_s': 0.0,
                    'target_lead_max_s': 2.0,
                    'csv_path': str(run_dir / 'backend.csv'),
                    'replan_csv_path': str(run_dir / 'replans.csv'),
                },
            ],
        ),
    ])

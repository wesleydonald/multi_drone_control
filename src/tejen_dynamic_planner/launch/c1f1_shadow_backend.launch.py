from datetime import datetime
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _workspace_root() -> Path:
    # Colcon isolated install prefix: <workspace>/install/tejen_dynamic_planner
    return Path(get_package_prefix('tejen_dynamic_planner')).resolve().parents[1]


def _new_run_directory(repo_root: Path) -> Path:
    log_root = repo_root / 'logs' / 'tejen_dynamic_planner'
    log_root.mkdir(parents=True, exist_ok=True)
    stem = f"c1f1_shadow_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
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
        DeclareLaunchArgument(
            'planner_rate_hz',
            default_value='5.0',
            description='Persistent C1F.1 shadow replan trigger rate.',
        ),
        DeclareLaunchArgument(
            'minimum_search_z_m',
            default_value='0.20',
            description='Lower search bound used by the C++ shadow planner.',
        ),
        LogInfo(msg='C1F.1 SHADOW ONLY: online_join_planner keeps /join_planner/reference authority.'),
        LogInfo(msg='C1F.1 tests mission-target plumbing; canonical C1E obstacle witness is disabled here.'),
        LogInfo(msg=f'C1F.1 C++ shadow log directory: {run_dir}'),
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
                    # C1F.0 already commissioned the physical static obstacle.  C1F.1
                    # isolates the mission<->backend target/enable/reference contract.
                    'static_scene_enabled': False,
                    'require_scene_witness': False,
                    'csv_path': str(run_dir / 'backend.csv'),
                    'replan_csv_path': str(run_dir / 'replans.csv'),
                },
            ],
        ),
    ])

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
    # Follow the repository's long-standing logging convention:
    #   logs/<component>/<trajectory_or_run>_<timestamp>/...
    # Controllers already use this pattern through tejen_utility_objects.DataLogger.
    log_root = repo_root / 'logs' / 'tejen_dynamic_planner'
    log_root.mkdir(parents=True, exist_ok=True)

    stem = f"c1e_static_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
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

    config = Path(get_package_share_directory('tejen_dynamic_planner')) / 'config' / 'c1e_static_sim.yaml'
    planner_rate = LaunchConfiguration('planner_rate_hz')

    csv_path = run_dir / 'active.csv'
    trace_csv_path = run_dir / 'reference_trace.csv'
    replan_csv_path = run_dir / 'replans.csv'

    return LaunchDescription([
        DeclareLaunchArgument(
            'planner_rate_hz',
            default_value='5.0',
            description='Persistent C.1e planner trigger rate.',
        ),
        LogInfo(msg=f'C1E planner data log directory: {run_dir}'),
        Node(
            package='tejen_dynamic_planner',
            executable='dynamic_planner_active_commissioning',
            name='dynamic_planner_active_commissioning',
            output='screen',
            parameters=[
                str(config),
                {
                    'planner_rate_hz': ParameterValue(planner_rate, value_type=float),
                    'csv_path': str(csv_path),
                    'trace_csv_path': str(trace_csv_path),
                    'replan_csv_path': str(replan_csv_path),
                },
            ],
        ),
    ])

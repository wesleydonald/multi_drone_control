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
    stem = f"c1f4_execsafe_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
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
        / 'c1f4_cooperative_execution_safe.yaml'
    )
    planner_rate = LaunchConfiguration('planner_rate_hz')
    minimum_search_z_m = LaunchConfiguration('minimum_search_z_m')
    spline_time_factor = LaunchConfiguration('spline_time_factor')
    factor_alloc = LaunchConfiguration('factor_alloc')
    factor_alloc_close = LaunchConfiguration('factor_alloc_close')
    close_to_goal_m = LaunchConfiguration('close_to_goal_m')
    smoother_jerk_weight = LaunchConfiguration('smoother_jerk_weight')
    smoother_goal_weight = LaunchConfiguration('smoother_goal_weight')
    target_lead_max_distance_m = LaunchConfiguration('target_lead_max_distance_m')

    return LaunchDescription([
        DeclareLaunchArgument('planner_rate_hz', default_value='5.0'),
        DeclareLaunchArgument('minimum_search_z_m', default_value='0.20'),
        DeclareLaunchArgument('spline_time_factor', default_value='2.5'),
        DeclareLaunchArgument('factor_alloc', default_value='1.0'),
        DeclareLaunchArgument('factor_alloc_close', default_value='2.5'),
        DeclareLaunchArgument('close_to_goal_m', default_value='0.20'),
        DeclareLaunchArgument('smoother_jerk_weight', default_value='1.0'),
        DeclareLaunchArgument('smoother_goal_weight', default_value='10.0'),
        DeclareLaunchArgument('target_lead_max_distance_m', default_value='0.15'),
        LogInfo(msg='C1F.4: cooperative authority with execution tracking certificate + automatic checked stop/recovery.'),
        LogInfo(msg='C1F.4: no clearance-cost heuristic added; C1F.3b Octopus path ranking remains unchanged.'),
        LogInfo(msg='C1F.4 defaults: spline_time_factor=2.5, close factor=2.5, smoother weights jerk=1 / goal=10 (launch-overridable).'),
        LogInfo(msg='C1F.4 default: target lead retains the existing time model but is capped to 0.15 m of positional shift (launch-overridable).'),
        LogInfo(msg='C1F.4: physical moving-obstacle IRL remains out of scope until this is validated in simulation.'),
        LogInfo(msg=f'C1F.4 C++ log directory: {run_dir}'),
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
                    'spline_time_factor': ParameterValue(spline_time_factor, value_type=float),
                    'factor_alloc': ParameterValue(factor_alloc, value_type=float),
                    'factor_alloc_close': ParameterValue(factor_alloc_close, value_type=float),
                    'close_to_goal_m': ParameterValue(close_to_goal_m, value_type=float),
                    'smoother_jerk_weight': ParameterValue(smoother_jerk_weight, value_type=float),
                    'smoother_goal_weight': ParameterValue(smoother_goal_weight, value_type=float),
                    'target_lead_max_distance_m': ParameterValue(target_lead_max_distance_m, value_type=float),
                    'csv_path': str(run_dir / 'backend.csv'),
                    'replan_csv_path': str(run_dir / 'replans.csv'),
                },
            ],
        ),
    ])

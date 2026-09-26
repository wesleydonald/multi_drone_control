from datetime import datetime
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _workspace_root() -> Path:
    return Path(get_package_prefix('tejen_dynamic_planner')).resolve().parents[1]


def _new_run_directory(repo_root: Path) -> Path:
    log_root = repo_root / 'logs' / 'tejen_dynamic_planner'
    log_root.mkdir(parents=True, exist_ok=True)
    stem = f"c1f6_rendezvous_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    run_dir = log_root / stem
    suffix = 1
    while run_dir.exists():
        run_dir = log_root / f'{stem}_{suffix}'
        suffix += 1
    run_dir.mkdir(parents=True)
    return run_dir


def _make_backend_node(context, *, config: Path, run_dir: Path):
    count = int(LaunchConfiguration('cooperative_preview_count').perform(context))
    if count <= 0:
        raise RuntimeError('cooperative_preview_count must be positive')

    state_topics = [f'/fake_obstacles/drone_{i}/state' for i in range(count)]
    trajectory_topics = [
        f'/fake_obstacles/drone_{i}/committed_trajectory' for i in range(count)
    ]
    return [
        Node(
            package='tejen_dynamic_planner',
            executable='dynamic_planner_transfer_backend',
            name='dynamic_planner_transfer_backend',
            output='screen',
            parameters=[
                str(config),
                *( [LaunchConfiguration('extra_params_file').perform(context)]
                   if LaunchConfiguration('extra_params_file').perform(context) else [] ),
                {
                    'planner_rate_hz': ParameterValue(
                        LaunchConfiguration('planner_rate_hz'), value_type=float
                    ),
                    'minimum_search_z_m': ParameterValue(
                        LaunchConfiguration('minimum_search_z_m'), value_type=float
                    ),
                    'cooperative_drone_state_topics': state_topics,
                    'cooperative_committed_trajectory_topics': trajectory_topics,
                    # un-namespaced defaults; overridable for a multi-vehicle stack
                    'use_sim_time': ParameterValue(
                        LaunchConfiguration('use_sim_time'), value_type=bool
                    ),
                    'state_topic': LaunchConfiguration('state_topic'),
                    'object_attached_topic': LaunchConfiguration('object_attached_topic'),
                    'csv_path': str(run_dir / 'backend.csv'),
                    'replan_csv_path': str(run_dir / 'replans.csv'),
                },
            ],
        )
    ]


def generate_launch_description():
    repo_root = _workspace_root()
    run_dir = _new_run_directory(repo_root)
    config = (
        Path(get_package_share_directory('tejen_dynamic_planner'))
        / 'config'
        / 'c1f6_moving_rendezvous.yaml'
    )
    # Commissioned planner tuning remains in c1f6_moving_rendezvous.yaml. The
    # system-test-only cooperative preview count changes only fake companion topic
    # cardinality so three_attached can expose all three other drones to C++.

    return LaunchDescription([
        DeclareLaunchArgument('planner_rate_hz', default_value='5.0'),
        DeclareLaunchArgument('minimum_search_z_m', default_value='0.20'),
        DeclareLaunchArgument('cooperative_preview_count', default_value='2'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        # optional overrides applied after the commissioned YAML ('' = none)
        DeclareLaunchArgument('extra_params_file', default_value=''),
        DeclareLaunchArgument('state_topic', default_value='/motion_capture_state'),
        DeclareLaunchArgument('object_attached_topic', default_value='/magnet/object_attached'),
        LogInfo(msg='C1F.8c: planner tuning is loaded from c1f6_moving_rendezvous.yaml; launch no longer duplicates stale tuning values.'),
        LogInfo(msg='C1F.8c: the moving basket advertised commitment is the single future used for both collision geometry and rendezvous prediction.'),
        LogInfo(msg='C1F.8c: prepared motion is readiness evidence only; authority is acknowledged after a fresh post-request plan from the stationary hold.'),
        LogInfo(msg='C1F.8c: local pursuit uses position-only continuation; hard target velocity is reserved for a true local rendezvous.'),
        LogInfo(msg='C1F.8c: 100 ms fixed splice, independent 50 ms Octopus wall budget, no arbitrary terminal hold.'),
        LogInfo(msg=f'C1F.6 C++ log directory: {run_dir}'),
        OpaqueFunction(
            function=_make_backend_node,
            kwargs={'config': config, 'run_dir': run_dir},
        ),
    ])

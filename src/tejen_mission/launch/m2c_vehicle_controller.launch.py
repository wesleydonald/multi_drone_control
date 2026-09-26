"""Launch one M2C controller instance in an identity-isolated namespace.

This file deliberately launches exactly one heavyweight ACADOS controller.  It is
usable directly from a manual terminal, while run_m2c_ground.sh sequences the same
launch command for automated commissioning.
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


M2C_DRONE_IDS = (0, 1, 2, 3)


def _controller_action(context):
    raw_id = LaunchConfiguration("drone_id").perform(context)
    try:
        drone_id = int(raw_id)
    except ValueError as exc:
        raise RuntimeError(f"drone_id must be one of {M2C_DRONE_IDS}, got {raw_id!r}") from exc
    if drone_id not in M2C_DRONE_IDS:
        raise RuntimeError(f"drone_id must be one of {M2C_DRONE_IDS}, got {drone_id}")

    work_dir = Path(LaunchConfiguration("controller_work_dir").perform(context)).expanduser()
    work_dir.mkdir(parents=True, exist_ok=True)
    cache_dir = Path(LaunchConfiguration("acados_cache_dir").perform(context)).expanduser()
    cache_dir.mkdir(parents=True, exist_ok=True)
    raw_external_permission = LaunchConfiguration("require_external_arm_permission").perform(context).strip().lower()
    if raw_external_permission not in {"true", "false"}:
        raise RuntimeError("require_external_arm_permission must be true or false")
    require_external_arm_permission = raw_external_permission == "true"
    raw_arming_feedback_period = LaunchConfiguration("arming_state_feedback_period_s").perform(context)
    try:
        arming_state_feedback_period_s = float(raw_arming_feedback_period)
    except ValueError as exc:
        raise RuntimeError(
            "arming_state_feedback_period_s must be a non-negative float"
        ) from exc
    if arming_state_feedback_period_s < 0.0:
        raise RuntimeError("arming_state_feedback_period_s must be non-negative")

    ns = f"drone_{drone_id}"
    vehicle = ns
    reference_topic = f"/{ns}/join_planner/reference"

    return [
        Node(
            package="tejen_mpc",
            executable="main",
            namespace=ns,
            name="tejen_mpc",
            output="screen",
            cwd=str(work_dir),
            remappings=[
                ("drone_arming_service", "arming_service"),
                ("drone_command", "command"),
                ("drone_arming_state_feedback", "arming_state_feedback"),
                # multi_drone_control handover: route his ELRS through a per-drone mux
                ("ELRSCommand", LaunchConfiguration("elrs_output_topic").perform(context)),
            ],
            parameters=[{
                "use_sim_time": True,
                "use_external_reference": True,
                "disarmed_external_reference_fast_path": True,
                "external_reference_topic": reference_topic,
                "require_external_arm_permission": require_external_arm_permission,
                "external_arm_permission_topic": f"/{ns}/join_planner/arm_permission",
                "arming_state_feedback_period_s": arming_state_feedback_period_s,
                "payload_mpc_mode_topic": f"/{ns}/join_planner/mpc_mode",
                "payload_mpc_mode_status_topic": "tejen_mpc/mpc_mode_status",
                "c1d_status_topic": "tejen_mpc/c1d_status",
                # our linear sim plant, shared x3 motorConstant 0.62e-6 (multi_drone_control)
                "thrust_ratio": 82.7,
                "enable_thrust_ratio_ukf": False,
                "enable_thrust_ratio_feedback": False,
                "acados_cache_dir": str(cache_dir),
                "logging_instance": vehicle,
            }],
        )
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("drone_id"),
        DeclareLaunchArgument("elrs_output_topic", default_value="ELRSCommand"),
        DeclareLaunchArgument("controller_work_dir"),
        DeclareLaunchArgument(
            "acados_cache_dir",
            default_value=str(Path.home() / ".cache" / "drone_cage_control" / "payload_mpc"),
        ),
        DeclareLaunchArgument("require_external_arm_permission", default_value="false"),
        DeclareLaunchArgument("arming_state_feedback_period_s", default_value="0.0"),
        OpaqueFunction(function=_controller_action),
    ])

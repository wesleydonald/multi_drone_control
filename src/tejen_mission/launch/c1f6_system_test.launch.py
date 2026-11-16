"""Launch the frozen C1F.6 simulation topology under a bounded supervisor."""

from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue


def _workspace_root() -> Path:
    return Path(get_package_prefix("tejen_mission")).resolve().parents[1]


def generate_launch_description() -> LaunchDescription:
    root = _workspace_root()
    drone_share = Path(get_package_share_directory("tejen_mission"))
    simulation_share = Path(get_package_share_directory("simulation_communication"))
    dynamic_share = Path(get_package_share_directory("tejen_dynamic_planner"))
    visualisation_share = Path(get_package_share_directory("drone_visualisation"))

    gui = LaunchConfiguration("gui")
    control_mode = LaunchConfiguration("control_mode")
    result_path = LaunchConfiguration("result_path")
    startup_timeout_s = LaunchConfiguration("startup_timeout_s")
    manual_start_timeout_s = LaunchConfiguration("manual_start_timeout_s")
    mission_timeout_s = LaunchConfiguration("mission_timeout_s")
    success_dwell_s = LaunchConfiguration("success_dwell_s")
    freshness_timeout_s = LaunchConfiguration("freshness_timeout_s")
    planner_status_timeout_s = LaunchConfiguration("planner_status_timeout_s")
    supervisor_success_phase = LaunchConfiguration("supervisor_success_phase")
    supervisor_failure_phase = LaunchConfiguration("supervisor_failure_phase")
    supervisor_require_handoff_ready = LaunchConfiguration("supervisor_require_handoff_ready")
    supervisor_landing_request_phase = LaunchConfiguration("supervisor_landing_request_phase")
    case_config = LaunchConfiguration("case_config")
    cooperative_preview_count = LaunchConfiguration("cooperative_preview_count")
    xy_bias_mode = LaunchConfiguration("xy_bias_mode")
    disturbance_force_x_n = LaunchConfiguration("disturbance_force_x_n")
    disturbance_force_y_n = LaunchConfiguration("disturbance_force_y_n")
    case_parameters = ParameterFile(case_config, allow_substs=True)

    world = root / "simulation_assets" / "tejen" / "world_drone_env_detach.sdf"
    gazebo_gui = ExecuteProcess(
        cmd=["gz", "sim", "-v", "2", "-r", str(world)],
        name="c1f6_gazebo_gui",
        output="screen",
        condition=IfCondition(gui),
        sigterm_timeout="3",
        sigkill_timeout="2",
    )
    gazebo_headless = ExecuteProcess(
        cmd=["gz", "sim", "-s", "-v", "2", "-r", str(world)],
        name="c1f6_gazebo_headless",
        output="screen",
        condition=UnlessCondition(gui),
        sigterm_timeout="3",
        sigkill_timeout="2",
    )

    # Simulation-only pickup-bias disturbance window. It applies a small
    # persistent world-frame force to the X3 base link only from the stationary
    # pickup settle through descent, then clears it at LIFT_OBJECT. This targets
    # the real M1 pre-pickup steady-state XY error without perturbing loaded transit.
    xy_disturbance_gate = Node(
        package="tejen_mission",
        executable="simulation_xy_disturbance_gate",
        name="c1f6_xy_disturbance_gate",
        output="screen",
        parameters=[{
            "force_x_n": ParameterValue(disturbance_force_x_n, value_type=float),
            "force_y_n": ParameterValue(disturbance_force_y_n, value_type=float),
            "activation_phase": "SETTLE_ABOVE_PICKUP",
            "deactivation_phase": "LIFT_OBJECT",
            "phase_topic": "/join_planner/phase",
            "world_name": "quadcopter",
            "target_link": "x3::X3/base_link",
        }],
    )

    simulation_interfaces = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(simulation_share / "launch" / "tejen_betaflight_linear_simulation_launch.py")
        )
    )
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(visualisation_share / "launch" / "view_frame.launch.py")
        ),
        condition=IfCondition(gui),
    )

    pendulum = Node(
        package="tejen_mission",
        executable="pendulum_state_publisher",
        name="pendulum_state_publisher",
        output="screen",
        parameters=[{
            "x3_pose_topic": "/model/x3/pose",
            "magnet_index": 0,
            "attachment_index": 5,
            "drone_index": 7,
            "x3_link_poses_are_relative": True,
            "publish_payload_world_state": True,
        }],
    )
    cooperative_world = Node(
        package="tejen_mission",
        executable="fake_cooperative_transport_world",
        name="fake_cooperative_transport_world",
        output="screen",
        parameters=[{
            "trajectory_type": "circle",
            "obstacle_scenario": "custom",
            "center_x": 2.0,
            "center_y": 2.0,
            # Fake-world motion parameters define the advertised payload/drone
            # commitments. C++ rendezvous consumes that payload commitment directly.
            "center_z": 1.5,
            "radius": 0.5,
            "omega": 0.25,
            "payload_committed_trajectory_topic": "/fake_payload/committed_trajectory",
            # M1 ring/net physical contract. Ring centre remains the delivery
            # target; plate 0 is the later reattachment target.
            "ring_plate_count": 12,
            "ring_plate_pitch_diameter_m": 0.50,
            "ring_plate_diameter_m": 0.06,
            "ring_attachment_plate_index": 0,
            "ring_collision_outer_diameter_m": 0.56,
            "ring_collision_top_offset_m": 0.0,
            "ring_collision_bottom_offset_m": -0.27,
            "ring_structure_height_m": 0.07,
            "enable_fake_obstacles": True,
            "fake_drone_count": 2,
            "fake_drone_offsets_x": [0.30, 0.30],
            "fake_drone_offsets_y": [0.60, -0.60],
            "fake_drone_offsets_z": [0.50, 0.50],
            "fake_drone_radius": 0.12,
            "fake_drone_safety_radius": 0.35,
            "publish_shared_trajectories": True,
            "shared_trajectory_duration_s": 180.0,
            "shared_trajectory_control_interval_s": 0.50,
        }, case_parameters],
    )
    magnet = Node(
        package="tejen_mission",
        executable="magnet_attachment_manager",
        name="magnet_attachment_manager",
        output="screen",
        parameters=[{
            "enable_elrs_magnet_output": False,
            "attachment_mode": "fixed_joint",
            "command_backend": "gz_cli",
            "magnet_tip_pose_topic": "/magnet_tip_pose",
            "magnet_tip_pose_msg_type": "pose_stamped",
            "object_pose_topic": "/model/payload_model/pose",
            "object_pose_index": 1,
            "use_fallback_object_pose": False,
            "attach_radius": 0.18,
            "attach_speed_threshold": 1.0,
            "attach_dwell_time_s": 0.0,
        }],
    )
    join_planner = Node(
        package="tejen_mission",
        executable="online_join_planner",
        name="online_join_planner",
        output="screen",
        parameters=[
            str(drone_share / "config" / "payloads" / "spanner_8mm.yaml"),
            str(drone_share / "config" / "visual_astar_visual_only.yaml"),
            str(drone_share / "config" / "c1f2b_cpp_authority.yaml"),
            {
                "vehicle_id": "drone_0",
                "assigned_attachment_id": "attachment_0",
                "mission_mode": "pickup_delivery",
                "pickup_object_pose_topic": "/model/payload_model/pose",
                "pickup_object_index": 1,
                "object_attached_topic": "/magnet/object_attached",
                "magnet_command_topic": "/magnet/command",
                "use_measured_magnet_tip": True,
                "magnet_tip_pose_topic": "/magnet_tip_pose",
                "use_geometry_aware_pickup": True,
                "auto_descend": True,
                "pickup_xy_tolerance": 0.10,
                "pickup_tip_speed_tolerance": 0.60,
                "pickup_approach_clearance": 0.35,
                "pickup_attach_clearance": 0.03,
                "pickup_lift_height": 0.40,
                "pickup_lift_speed": 0.12,
                "pickup_z_tolerance": 0.20,
                "max_reference_speed": 2.0,
                "lift_complete_object_clearance": 0.08,
                "match_xy_threshold": 0.18,
                "match_xy_velocity_threshold": 0.20,
                "reattach_transit_xy_threshold": 0.40,
                "attach_ready_target_lead_time": 0.30,
                "use_static_pickup_object": False,
                "drop_point_source": "payload",
                "payload_pose_topic": "/fake_payload/pose",
                "payload_twist_topic": "/fake_payload/twist",
                "reference_nominal_speed": 0.25,
                "static_obstacle_count": 0,
            },
            case_parameters,
        ],
    )
    dynamic_planner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(dynamic_share / "launch" / "c1f6_moving_rendezvous_backend.launch.py")
        ),
        launch_arguments={
            "planner_rate_hz": "5.0",
            "minimum_search_z_m": "0.20",
            "cooperative_preview_count": cooperative_preview_count,
        }.items(),
    )
    controller = Node(
        package="tejen_mpc",
        executable="main",
        name="tejen_mpc",
        output="screen",
        parameters=[{
            "use_external_reference": True,
            "enable_thrust_ratio_ukf": True,
            # our linear sim plant with the shared x3 motor constant (0.643 kg): 4*0.62e-6*4631^2/0.643
            "thrust_ratio": 82.7,
            "xy_bias_mode": ParameterValue(xy_bias_mode, value_type=str),
        }],
    )
    supervisor = Node(
        package="tejen_mission",
        executable="simulation_test_supervisor",
        name="simulation_test_supervisor",
        output="screen",
        parameters=[{
            "control_mode": ParameterValue(control_mode, value_type=str),
            "result_path": ParameterValue(result_path, value_type=str),
            "startup_timeout_s": ParameterValue(startup_timeout_s, value_type=float),
            "manual_start_timeout_s": ParameterValue(manual_start_timeout_s, value_type=float),
            "mission_timeout_s": ParameterValue(mission_timeout_s, value_type=float),
            "success_dwell_s": ParameterValue(success_dwell_s, value_type=float),
            "freshness_timeout_s": ParameterValue(freshness_timeout_s, value_type=float),
            "planner_status_timeout_s": ParameterValue(
                planner_status_timeout_s, value_type=float
            ),
            "success_phase": ParameterValue(supervisor_success_phase, value_type=str),
            "failure_phase": ParameterValue(supervisor_failure_phase, value_type=str),
            "require_handoff_ready": ParameterValue(
                supervisor_require_handoff_ready, value_type=bool
            ),
            "landing_request_phase": ParameterValue(
                supervisor_landing_request_phase, value_type=str
            ),
        }],
    )

    shutdown_when_supervisor_finishes = RegisterEventHandler(
        OnProcessExit(
            target_action=supervisor,
            on_exit=[EmitEvent(event=Shutdown(reason="simulation supervisor finished"))],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="false"),
        DeclareLaunchArgument("control_mode", default_value="manual"),
        DeclareLaunchArgument(
            "case_config",
            default_value=str(root / "tools" / "sim_test" / "cases" / "v1" / "baseline.yaml"),
        ),
        DeclareLaunchArgument("cooperative_preview_count", default_value="2"),
        DeclareLaunchArgument("xy_bias_mode", default_value="legacy_integral"),
        DeclareLaunchArgument("disturbance_force_x_n", default_value="0.0"),
        DeclareLaunchArgument("disturbance_force_y_n", default_value="0.0"),
        DeclareLaunchArgument("result_path"),
        DeclareLaunchArgument("startup_timeout_s", default_value="60.0"),
        DeclareLaunchArgument("manual_start_timeout_s", default_value="300.0"),
        DeclareLaunchArgument("mission_timeout_s", default_value="120.0"),
        DeclareLaunchArgument("success_dwell_s", default_value="2.0"),
        DeclareLaunchArgument("freshness_timeout_s", default_value="1.0"),
        DeclareLaunchArgument("planner_status_timeout_s", default_value="2.5"),
        DeclareLaunchArgument("supervisor_success_phase", default_value="ATTACH_READY"),
        DeclareLaunchArgument("supervisor_failure_phase", default_value="LANDED_DISARMED"),
        DeclareLaunchArgument("supervisor_require_handoff_ready", default_value="true"),
        DeclareLaunchArgument("supervisor_landing_request_phase", default_value=""),
        LogInfo(msg=["C1F.6 supervised system test case: ", case_config]),
        LogInfo(msg="Automatic arming is performed only by the simulation supervisor."),
        gazebo_gui,
        gazebo_headless,
        TimerAction(period=1.5, actions=[xy_disturbance_gate]),
        TimerAction(period=1.0, actions=[simulation_interfaces, rviz]),
        TimerAction(period=2.0, actions=[pendulum, cooperative_world, magnet]),
        TimerAction(
            period=3.0,
            actions=[join_planner, dynamic_planner, controller, supervisor],
        ),
        shutdown_when_supervisor_finishes,
    ])

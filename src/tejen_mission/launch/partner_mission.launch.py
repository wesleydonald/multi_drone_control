"""Tejen's M1 pickup/drop/rejoin mission flown by drone 3 of our three_attach fleet.

Start AFTER our side is up and the ring is flying:
  ros2 launch controller_quad_load three_attach_launch.py partner:=true ...
  ros2 launch tejen_mission partner_mission.launch.py

His nodes live in /tejen (MPC, supervisor, pendulum, magnet manager) so their relative
topics do not meet ours. Boundary:
  * his MPC   -> /drone_3/ELRSCommand_tejen (our mux forwards it until the ring weld)
  * his state <- /drone_3/motion_capture_state (our mocap emulator)
  * his object magnet -> gz /pickup/attach|detach (second DetachableJoint on the partner model)
  * magnets: his planner and object manager use /tejen/magnet/command; /magnet/command is
    our ring magnet. At his ATTACH_READY (/join_planner/handoff_ready) our mux hands drone
    3 to our tracker, which flies the last descent (his ready point is ~0.18 m above the
    plate), our dissipative node commands the ring magnet ON, and the weld folds it in
  * the ring and carriers <- ring_bridge (our ring's mocap + the load planner's trajectory)
"""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

NS = 'tejen'
DRONE = 3
DRONE_MODEL = 'x3_drone3'
OBJECT_POSE = '/model/payload_model/pose'


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory('tejen_mission'))
    dyn_share = Path(get_package_share_directory('tejen_dynamic_planner'))
    state = f'/drone_{DRONE}/motion_capture_state'
    L = LaunchConfiguration

    object_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='pickup_object_pose_bridge',
        arguments=[f'{OBJECT_POSE}@geometry_msgs/msg/PoseArray[gz.msgs.Pose_V'])

    ring = Node(
        package='tejen_mission', executable='ring_bridge', name='fake_cooperative_transport_world',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'bridge_carrier_azimuths_deg': ParameterValue(L('carrier_azimuths_deg'), value_type=None),
            'bridge_attach_radius_m': 0.25,
            'bridge_cable_len_m': 0.5,
            'bridge_cable_elev_deg': 45.0,
            'payload_committed_trajectory_topic': '/fake_payload/committed_trajectory',
            'ring_plate_count': 12,
            'ring_plate_pitch_diameter_m': 0.50,
            'ring_plate_diameter_m': 0.06,
            # plate 9 = 270 deg = our weld point (attach_y_offset -0.25)
            'ring_attachment_plate_index': ParameterValue(L('plate'), value_type=int),
            'ring_collision_outer_diameter_m': 0.56,
            'ring_collision_top_offset_m': 0.0,
            'ring_collision_bottom_offset_m': -0.27,
            'ring_structure_height_m': 0.07,
            'enable_fake_obstacles': True,
            'fake_drone_count': 3,
            'fake_drone_radius': 0.12,
            'fake_drone_safety_radius': 0.35,
            'publish_shared_trajectories': True,
            # rolling commitments: our ring's path is only known from its live
            # trajectory state (his fake ring published one analytic 180 s plan)
            'shared_trajectory_duration_s': 30.0,
            'shared_trajectory_control_interval_s': 0.50,
            'shared_trajectory_refresh_period_s': 1.0,
        }])

    pendulum = Node(
        package='tejen_mission', executable='pendulum_state_publisher',
        name='pendulum_state_publisher', namespace=NS, output='screen',
        parameters=[{
            'use_sim_time': True,
            'x3_pose_topic': f'/model/{DRONE_MODEL}/pose',
            'magnet_index': 0, 'attachment_index': 5, 'drone_index': 7,
            'x3_link_poses_are_relative': True,
            'object_pose_topic': OBJECT_POSE,
            'publish_payload_world_state': True,
            'publish_rate_hz': 100.0,    # his MPC runs at 30 Hz; 500 Hz cost sim speed
        }])

    magnet = Node(
        package='tejen_mission', executable='magnet_attachment_manager',
        name='magnet_attachment_manager', namespace=NS, output='screen',
        parameters=[{
            'use_sim_time': True,
            'enable_elrs_magnet_output': False,
            'attachment_mode': 'fixed_joint',
            # one-shot gz CLI publishes drop: the object stayed welded after DROP (R0678) and
            # missed the net; the ROS topics go through the bridges below as well
            'command_backend': 'both',
            'magnet_command_topic': f'/{NS}/magnet/command',   # his object magnet; /magnet/command is our ring magnet (ours from the handoff)
            'magnet_tip_pose_topic': f'/{NS}/magnet_tip_pose',
            'magnet_tip_pose_msg_type': 'pose_stamped',
            'object_pose_topic': OBJECT_POSE,
            'object_pose_index': 1,
            'use_fallback_object_pose': False,
            # near-contact, like our ring weld: 0.18 welded the object mid-descent 18 cm off
            # the tip, and the spanner then hung on that lever (R0626); covers his 3 cm
            # attach clearance plus ~5 cm xy error
            # contact weld (Wesley: a magnet touching a steel ball): tip within 0.10 m of the
            # 0.08 m ball's centre = touching its top. 0.18 welded mid-descent 18 cm off
            # the tip, which is why the pickup looked wrong (R0625-R0633)
            'attach_radius': 0.10,
            'attach_speed_threshold': 1.0,
            'attach_dwell_time_s': 0.0,
            # the object only: never /payload/* (that is the ring weld)
            'object_attached_topic': f'/{NS}/object_attached',
            'attachment_state_topic': f'/{NS}/attachment_state',
            'gz_attach_topic': '/pickup/attach', 'gz_detach_topic': '/pickup/detach',
            'ros_attach_topic': '/pickup/attach', 'ros_detach_topic': '/pickup/detach',
        }])

    planner = Node(
        package='tejen_mission', executable='online_join_planner',
        name='online_join_planner', output='screen',
        parameters=[
            str(share / 'config' / 'payloads' / 'spanner_8mm.yaml'),
            str(share / 'config' / 'visual_astar_visual_only.yaml'),
            str(share / 'config' / 'c1f2b_cpp_authority.yaml'),
            {
                'use_sim_time': True,
                'vehicle_id': 'drone_0',
                'assigned_attachment_id': 'attachment_0',
                'mission_mode': 'pickup_delivery',
                'drone_state_topic': state,
                'drone_command_topic': f'/{NS}/drone_command',
                'pickup_object_pose_topic': OBJECT_POSE,
                'pickup_object_index': 1,
                'object_attached_topic': f'/{NS}/object_attached',
                'magnet_command_topic': f'/{NS}/magnet/command',   # his object magnet; /magnet/command is our ring magnet (ours from the handoff)
                'use_measured_magnet_tip': True,
                'magnet_tip_pose_topic': f'/{NS}/magnet_tip_pose',
                'use_geometry_aware_pickup': True,
                'auto_descend': True,
                'pickup_xy_tolerance': 0.10,
                'pickup_tip_speed_tolerance': 0.60,
                'pickup_approach_clearance': 0.35,
                # tip target = ball centre + 0.09: 1 cm above the top of the 0.08 m ball
                # (0.03 put the target inside the ball; the rod was pushed aside)
                'pickup_attach_clearance': 0.09,
                # a HIGH transit (Wesley 2026-09-26: near-collisions): lift the quad to
                # ~1.9 m at the pickup, fly the C++ transit above a 1.8 m floor so the
                # 0.49 m rod + ball clear the carriers (tops ~1.1 m), and only descend
                # inside the ring's clear centre (R0629/R0636 swept the rod into carrier 0)
                'pickup_lift_height': L('pickup_lift_height'),
                'drop_approach_clearance': L('drop_approach_clearance'),
                'pickup_lift_speed': 0.12,
                'pickup_z_tolerance': 0.20,
                'max_reference_speed': 2.0,
                'lift_complete_object_clearance': 0.08,
                'match_xy_threshold': 0.18,
                'match_xy_velocity_threshold': 0.20,
                'reattach_transit_xy_threshold': 0.40,
                'attach_ready_target_lead_time': 0.30,
                'use_static_pickup_object': False,
                'drop_point_source': 'payload',
                'payload_pose_topic': '/fake_payload/pose',
                'payload_twist_topic': '/fake_payload/twist',
                'reference_nominal_speed': 0.25,
                'static_obstacle_count': 0,
                # no auto-land from ATTACH_READY: after our weld its magnet OFF would
                # release drone 3 from the ring
                'attach_ready_land_after_s': 0.0,
                'log_every_n_updates': 6,     # 30 Hz -> 5 Hz CSV (sim speed)
                # our x3_drone3 rod: body centre to tip 0.49 (his airframe 0.50 below a
                # 0.05 pivot); his 0.60 reference floor held our tip 8 cm above the
                # pickup target (R0627)
                'magnet_drop_below_quad': 0.49,
                'min_reference_z': 0.52,
                # the backend publishes at 5 Hz (sim); a 0.20 s freshness limit is one
                # period, and one late tick parked the drone on its terminal hold for the
                # rest of the run (R0622, R0628)
                'c1f1_shadow_reference_timeout_s': 0.6,
                # his hold before the C++ grant wanders 8-15 cm in our loaded sim; the
                # 0.15 m bound cancelled every grant in R0630 (R0609 alike). The C++ plan
                # starts from the re-latched measured state either way.
                'c1f2_prepare_max_lift_drift_m': 0.30,
                'c1f2_prepare_settle_speed_mps': 0.12,
            },
        ])

    backend = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(dyn_share / 'launch' / 'c1f6_moving_rendezvous_backend.launch.py')),
        launch_arguments={'planner_rate_hz': '5.0', 'minimum_search_z_m': L('minimum_search_z_m'),
                          'cooperative_preview_count': '3',
                          # drone 3's state and HIS object flag (/magnet/object_attached is
                          # our ring weld): R0609's backend waited on both forever
                          # the shared trajectories (ring_bridge) are stamped in sim time
                          'use_sim_time': 'true',
                          'extra_params_file': str(share / 'config' / 'partner_backend_overrides.yaml'),
                          'state_topic': state,
                          'object_attached_topic': f'/{NS}/object_attached'}.items())

    controller = Node(
        package='tejen_mpc', executable='main', name='tejen_mpc', namespace=NS, output='screen',
        parameters=[{
            'use_sim_time': True,
            'use_external_reference': True,
            'enable_thrust_ratio_ukf': True,
            # linear sim plant, x3 0.643 kg, motorConstant 0.62e-6: 4*0.62e-6*4631^2/0.643
            'thrust_ratio': 82.7,
            'xy_bias_mode': 'legacy_integral',
            # a cached solver: a fresh acados build at launch starved drone 3's mocap (R0600)
            'acados_cache_dir': os.path.expanduser('~/.cache/multi_drone_control/tejen_payload_mpc'),
            'cable_length': 0.49,
            'pose_timeout_s': 1.0,       # sim at ~0.25 real time (R0620 self-disarm)
            # periodic ARM state heartbeat: a single startup message can be lost, and his
            # supervisor then never arms (R0638, R0641 'arming_feedback_received': False)
            'arming_state_feedback_period_s': 0.5,        # body centre to tip on x3_drone3_magnet_rod_partner
            # carried-object gain schedule (G1b; 0 = off; default 0.1 kg = payload_model.sdf): kT*m_d/(m_d+m_obj) while the object is on
            'object_mass_kg': ParameterValue(L('object_mass_kg'), value_type=float),
            'object_vehicle_mass_kg': 0.643,
            'object_attached_topic': f'/{NS}/object_attached',
        }],
        remappings=[('motion_capture_state', state),
                    ('ELRSCommand', f'/drone_{DRONE}/ELRSCommand_tejen')])

    supervisor = Node(
        package='tejen_mission', executable='simulation_test_supervisor',
        name='simulation_test_supervisor', namespace=NS, output='screen',
        parameters=[{
            'use_sim_time': True,
            'control_mode': 'automatic',
            # his nodes start with the fleet on the ground (a start mid-flight starved our
            # trackers of CPU, R0604); his mission waits for the ring to fly its path
            'start_gate_topic': L('start_gate_topic'),
            'result_path': ParameterValue(L('result_path'), value_type=str),
            'startup_timeout_s': 300.0,   # counted from the start gate (R0638: 60 s ran out)
            'arm_timeout_s': 20.0,
            'manual_start_timeout_s': 300.0,
            # wall-clock in his supervisor: the full stack runs at ~0.25 real time
            'mission_timeout_s': 3000.0,
            'success_dwell_s': 2.0,
            # wall-clock limits; the stack runs at ~0.25 real time and the backend's 1 Hz
            # (sim) status arrives every ~4 s wall (R0613 READINESS_LOST at 3.0 s)
            'freshness_timeout_s': 5.0,
            'planner_status_timeout_s': 15.0,
            # our ring weld is the success, not his phase: a supervisor stop disarms his
            # MPC, which would drop drone 3 if it came before the weld
            'success_phase': 'NONE_PARTNER_WELD_OWNS_SUCCESS',
            'failure_phase': 'LANDED_DISARMED',
            'require_handoff_ready': True,
            'landing_request_phase': '',
        }],
        remappings=[('/motion_capture_state', state),
                    ('/pendulum_swing_state', f'/{NS}/pendulum_swing_state'),
                    ('/payload_world_state', f'/{NS}/payload_world_state')])
    # no shutdown-on-supervisor-exit: his MPC must keep flying until our weld takes over

    pickup_bridges = [Node(
        package='ros_gz_bridge', executable='parameter_bridge', name=f'pickup_{w}_bridge',
        arguments=[f'/pickup/{w}@std_msgs/msg/Empty]gz.msgs.Empty']) for w in ('attach', 'detach')]

    # gz DetachableJoint may weld the object to the tip at spawn: free it before the mission
    release = ExecuteProcess(
        cmd=['gz', 'topic', '-t', '/pickup/detach', '-m', 'gz.msgs.Empty', '-p', ''],
        output='screen')

    # diagnosis record: his view of the drone, his reference and his commands
    bag = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-o', L('bag_path'), '--use-sim-time',
             state, f'/{NS}/pendulum_swing_state', f'/{NS}/magnet_tip_pose',
             '/join_planner/reference', '/join_planner/phase', f'/drone_{DRONE}/ELRSCommand_tejen',
             '/fake_payload/pose', '/magnet/command', f'/{NS}/magnet/command', '/join_planner/handoff_ready', f'/{NS}/object_attached', OBJECT_POSE],
        output='screen', condition=IfCondition(L('record')))

    return LaunchDescription([
        DeclareLaunchArgument('plate', default_value='9'),
        DeclareLaunchArgument('object_mass_kg', default_value='0.1'),   # the 0.1 kg object (payload_model.sdf)
        # the all-attached start: '[150.0, 270.0, 30.0]', plate 3, gate /partner/release
        DeclareLaunchArgument('carrier_azimuths_deg', default_value='[330.0, 90.0, 210.0]'),
        DeclareLaunchArgument('pickup_lift_height', default_value='1.40'),   # lift ends ~1.9 m, above the 1.8 m search floor (R0639 ended at 1.75)
        DeclareLaunchArgument('drop_approach_clearance', default_value='0.60'),
        DeclareLaunchArgument('minimum_search_z_m', default_value='1.8'),
        DeclareLaunchArgument('start_gate_topic', default_value='/payload/trajectory_state'),
        DeclareLaunchArgument('record', default_value='false'),
        DeclareLaunchArgument('bag_path', default_value=os.path.join(
            os.environ.get('MDC_RUN_DIR', '/tmp'), 'partner_bag')),
        DeclareLaunchArgument('result_path', default_value='/tmp/partner_mission_result.json'),
        bag, release, object_bridge, *pickup_bridges, ring, pendulum, magnet, planner, backend, controller, supervisor,
    ])

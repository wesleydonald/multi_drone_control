"""Tejen's M1 pickup/drop/rejoin mission flown by drone 3 of our three_attach fleet.

Start AFTER our side is up and the ring is flying:
  ros2 launch bringup sim_control_launch.py mode:=attach partner:=true ...
  ros2 launch tejen_mission partner_mission.launch.py

His nodes live in /tejen (MPC, supervisor, pendulum, magnet manager) so their relative
topics do not meet ours. Boundary:
  * his MPC   -> /drone_3/ELRSCommand_tejen (our mux forwards it until the ring weld)
  * his state <- /drone_3/motion_capture_state (our mocap emulator)
  * his object magnet -> gz /pickup/attach|detach (second DetachableJoint on the partner model)
  * magnets: his planner and object manager use /tejen/magnet/command; /magnet/command is
    our ring magnet. At his ATTACH_READY (/join_planner/handoff_ready) our mux hands drone
    3 to our tracker, which flies the last descent (his ready point was ~0.18 m above the
    plate before the 10 Oct sync, 0.12 m of it from his label-marker aliasing bug), our
    dissipative node commands the ring magnet ON, and the weld folds it in
  * the ring and carriers <- ring_bridge (our ring's mocap + the load planner's trajectory)

contact_attach:=true (rig_twin only; the combined M1 contact attach, docs/design/m1_contact_contract.md):
his planner descends to the contact target and proves the catch by a small lift (LIFT_PROOF);
our /magnet/object_attached ends it in RING_ATTACHED, our /drone_3/take_back or his give-up in
TAKEN_BACK. No handoff_ready. confirm_pickup_release:=true: his supervisor arms only once the
ball weld made at spawn reads free (repeated /pickup/detach), instead of one gz publish.
"""
import os
from pathlib import Path

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

NS = 'tejen'
DRONE = 3
DRONE_MODEL = 'x3_drone3'
OBJECT_POSE = '/model/payload_model/pose'

# geometry:=legacy is the old partner worlds (our x3_drone3 partner rod, linear plant); geometry:=rig_twin is
# the rig-twin attach world (three_rigid_ground_rig_attach_pickup.sdf, the combined M1 subtest, Wesley 10 Oct):
# plates at r 0.225, 0.55 m rods at 60 deg, QUAD4 0.55 kg with a 0.55 m rod from a pivot 4 cm below the body
# (body to tip 0.59), flown through the rig thrust map, so his MPC takes his rig values (thrust ratio 22 with
# feedback, irl_commissioning.yaml) instead of the linear plant's 82.7.
# His plates count counter-clockwise from +x every 30 deg. legacy: plate 9 = 270 deg = our old weld
# point (attach_y_offset -0.25), carriers at 330/90/210. rig_twin: plate 3 = 90 deg = +y = rig plate 9
# (attach_y_offset 0.225), carriers at 270/30/150 (rig plates 3/11/7). contact_attach_z_m: his contact
# target above the ring origin, the tip sphere's centre on the plate (plate top 0.02 + 0.012 radius).
GEOMETRY = {
    'legacy': {
        'ring_radius': 0.25, 'ring_cable_len': 0.5, 'ring_cable_elev_deg': 45.0,
        'ring_pitch_diameter': 0.50, 'ring_collision_outer_diameter': 0.56,
        'magnet_drop_below_quad': 0.49, 'mpc_cable_length': 0.49,
        'thrust_ratio': 82.7, 'thrust_ratio_feedback': False, 'vehicle_mass': 0.643,
        'plate': '9', 'carrier_azimuths_deg': '[330.0, 90.0, 210.0]', 'contact_attach_z_m': None,
    },
    'rig_twin': {
        'ring_radius': 0.225, 'ring_cable_len': 0.55, 'ring_cable_elev_deg': 60.0,
        'ring_pitch_diameter': 0.45, 'ring_collision_outer_diameter': 0.51,
        'magnet_drop_below_quad': 0.59, 'mpc_cable_length': 0.55,
        'thrust_ratio': 22.0, 'thrust_ratio_feedback': True, 'vehicle_mass': 0.55,
        'plate': '3', 'carrier_azimuths_deg': '[270.0, 30.0, 150.0]', 'contact_attach_z_m': 0.032,
    },
}
# the topics the contact attach adds to the diagnosis bag
CONTACT_TOPICS = ['/magnet/contact', '/magnet/object_attached', '/magnet/attach_proof',
                  '/join_planner/give_up', f'/drone_{DRONE}/take_back', f'/drone_{DRONE}/mux_state']
PICKUP_STATE = '/pickup/detachable_joint_state'
RELEASE_GATE = '/partner/release'    # our planner's release of a start-welded QUAD4, in the air after its step-out


def _flag(context, name):
    return LaunchConfiguration(name).perform(context).strip().lower() in ('1', 'true', 'yes', 'on')


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(_arguments() + [OpaqueFunction(function=_nodes)])


def _nodes(context):
    geometry = LaunchConfiguration('geometry').perform(context)
    if geometry not in GEOMETRY:
        raise RuntimeError(f'partner_mission: geometry:={geometry!r}, expected one of {sorted(GEOMETRY)}')
    G = GEOMETRY[geometry]
    contact = _flag(context, 'contact_attach')
    if contact and G['contact_attach_z_m'] is None:
        raise RuntimeError(f'partner_mission: contact_attach:=true has no contact target for geometry:={geometry}')
    confirm_release = _flag(context, 'confirm_pickup_release')
    # his MPC arms where QUAD4 is (it re-latches its kT estimator's start height every disarmed tick) and learns kT
    # only 0.20 m above it: released in the air (our welded start, gate /partner/release) his mission never climbs
    # that far before the pickup, kT stays at its nominal 22 and he flies ~11 cm under his reference (R1503)
    airborne = LaunchConfiguration('airborne_start').perform(context).strip().lower()
    airborne = (LaunchConfiguration('start_gate_topic').perform(context).strip() == RELEASE_GATE
                if airborne in ('', 'auto') else airborne in ('1', 'true', 'yes', 'on'))
    learn_in_air = airborne and G['thrust_ratio_feedback']
    plate = int(LaunchConfiguration('plate').perform(context) or G['plate'])
    azimuths = [float(a) for a in yaml.safe_load(
        LaunchConfiguration('carrier_azimuths_deg').perform(context) or G['carrier_azimuths_deg'])]
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
            'bridge_carrier_azimuths_deg': azimuths,
            'bridge_attach_radius_m': G['ring_radius'],
            'bridge_cable_len_m': G['ring_cable_len'],
            'bridge_cable_elev_deg': G['ring_cable_elev_deg'],
            'payload_committed_trajectory_topic': '/fake_payload/committed_trajectory',
            'ring_plate_count': 12,
            'ring_plate_pitch_diameter_m': G['ring_pitch_diameter'],
            'ring_plate_diameter_m': 0.06,
            'ring_attachment_plate_index': plate,     # GEOMETRY: his numbering vs ours
            'ring_collision_outer_diameter_m': G['ring_collision_outer_diameter'],
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
            # the contact target, rotated by the full ring attitude like our weld point
            **({'bridge_attach_z_m': G['contact_attach_z_m'], 'full_pose_attachment': True} if contact else {}),
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
                'magnet_drop_below_quad': G['magnet_drop_below_quad'],
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
                # the descent ends on the contact target (ring_bridge's attachment point), held
                # with no lead; the planner's contact-mode gates are his ground-ring values
                # (the card's neighbours move one of them: contact_lead / contact_dz / contact_dx)
                **({'contact_attach': True,
                    'final_attach_offset_z': ParameterValue(L('contact_dz'), value_type=float),
                    'approach_offset_x': ParameterValue(L('contact_dx'), value_type=float),
                    'attach_ready_target_lead_time': ParameterValue(L('contact_lead'), value_type=float)}
                   if contact else {}),
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
            'thrust_ratio': G['thrust_ratio'],
            'enable_thrust_ratio_feedback': G['thrust_ratio_feedback'],
            'xy_bias_mode': 'legacy_integral',
            # a cached solver: a fresh acados build at launch starved drone 3's mocap (R0600)
            'acados_cache_dir': os.path.expanduser('~/.cache/multi_drone_control/tejen_payload_mpc'),
            'cable_length': G['mpc_cable_length'],
            'pose_timeout_s': 1.0,       # sim at ~0.25 real time (R0620 self-disarm)
            # periodic ARM state heartbeat: a single startup message can be lost, and his
            # supervisor then never arms (R0638, R0641 'arming_feedback_received': False)
            'arming_state_feedback_period_s': 0.5,        # body centre to tip on x3_drone3_magnet_rod_partner
            # carried-object gain schedule (G1b; 0 = off; default 0.1 kg = payload_model.sdf): kT*m_d/(m_d+m_obj) while the object is on
            'object_mass_kg': ParameterValue(L('object_mass_kg'), value_type=float),
            'object_vehicle_mass_kg': G['vehicle_mass'],
            'object_attached_topic': f'/{NS}/object_attached',
            **({'thrust_ratio_ukf_min_height_above_start': 0.0} if learn_in_air else {}),
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
            **({'pickup_release_state_topic': PICKUP_STATE} if confirm_release else {}),
        }],
        remappings=[('/motion_capture_state', state),
                    ('/pendulum_swing_state', f'/{NS}/pendulum_swing_state'),
                    ('/payload_world_state', f'/{NS}/payload_world_state')])
    # no shutdown-on-supervisor-exit: his MPC must keep flying until our weld takes over

    pickup_bridges = [Node(
        package='ros_gz_bridge', executable='parameter_bridge', name=f'pickup_{w}_bridge',
        arguments=[f'/pickup/{w}@std_msgs/msg/Empty]gz.msgs.Empty']) for w in ('attach', 'detach')]

    # gz DetachableJoint may weld the object to the tip at spawn: free it before the mission.
    # confirm_pickup_release: the supervisor repeats /pickup/detach until the joint reads free.
    if confirm_release:
        release = Node(
            package='ros_gz_bridge', executable='parameter_bridge', name='pickup_state_bridge',
            arguments=[f'{PICKUP_STATE}@std_msgs/msg/String[gz.msgs.StringMsg'])
    else:
        release = ExecuteProcess(
            cmd=['gz', 'topic', '-t', '/pickup/detach', '-m', 'gz.msgs.Empty', '-p', ''],
            output='screen')

    # diagnosis record: his view of the drone, his reference and his commands
    bag = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-o', L('bag_path'), '--use-sim-time',
             state, f'/{NS}/pendulum_swing_state', f'/{NS}/magnet_tip_pose',
             '/join_planner/reference', '/join_planner/phase', f'/drone_{DRONE}/ELRSCommand_tejen',
             '/fake_payload/pose', '/magnet/command', f'/{NS}/magnet/command', '/join_planner/handoff_ready', f'/{NS}/object_attached', OBJECT_POSE,
             *(CONTACT_TOPICS if contact else []), *([PICKUP_STATE] if confirm_release else [])],
        output='screen', condition=IfCondition(L('record')))

    return [bag, release, object_bridge, *pickup_bridges, ring, pendulum, magnet, planner, backend, controller,
            supervisor]


def _arguments():
    return [
        DeclareLaunchArgument('geometry', default_value='legacy', choices=sorted(GEOMETRY)),
        DeclareLaunchArgument('plate', default_value=''),   # '' = the geometry's
        DeclareLaunchArgument('object_mass_kg', default_value='0.1'),   # the 0.1 kg object (payload_model.sdf)
        # '' = the geometry's; the all-attached start: '[150.0, 270.0, 30.0]', plate 3, gate /partner/release
        DeclareLaunchArgument('carrier_azimuths_deg', default_value=''),
        DeclareLaunchArgument('contact_attach', default_value='false'),
        DeclareLaunchArgument('contact_lead', default_value='0.0'),   # contact_attach: his hold's reference lead (s)
        DeclareLaunchArgument('contact_dz', default_value='0.0'),     # contact_attach: tip target above the contact point (m)
        DeclareLaunchArgument('contact_dx', default_value='0.0'),     # contact_attach: tip target offset along x (m)
        DeclareLaunchArgument('confirm_pickup_release', default_value='false'),
        DeclareLaunchArgument('pickup_lift_height', default_value='1.40'),   # lift ends ~1.9 m, above the 1.8 m search floor (R0639 ended at 1.75)
        DeclareLaunchArgument('drop_approach_clearance', default_value='0.60'),
        DeclareLaunchArgument('minimum_search_z_m', default_value='1.8'),
        DeclareLaunchArgument('start_gate_topic', default_value='/payload/trajectory_state'),
        # his MPC learns kT from where it arms (no climb first); 'auto' = true when the gate is /partner/release.
        # Applied only where his kT feedback flies (rig_twin)
        DeclareLaunchArgument('airborne_start', default_value='auto', choices=['auto', 'true', 'false']),
        DeclareLaunchArgument('record', default_value='false'),
        DeclareLaunchArgument('bag_path', default_value=os.path.join(
            os.environ.get('MDC_RUN_DIR', '/tmp'), 'partner_bag')),
        DeclareLaunchArgument('result_path', default_value='/tmp/partner_mission_result.json'),
    ]

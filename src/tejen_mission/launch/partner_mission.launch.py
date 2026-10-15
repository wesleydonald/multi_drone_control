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
TAKEN_BACK. No handoff_ready. His pickup descent ends on the ball weld (pickup_hold_on_attach) and
his retry is on (pickup_retry_enabled): a pickup not on the ball after his wait rises and
re-approaches the ball, and his third failure is a give-up.
confirm_pickup_release:=true: his supervisor arms only once the
ball weld made at spawn reads free (repeated /pickup/detach), instead of one gz publish.

real:=true (the rig, combined M1; default false = everything above, unchanged): no Gazebo node, the wall
clock, his IRL values (irl_commissioning.yaml blocks copied in with every topic written out; never the file
as a whole: its root topics include our /magnet/object_attached), geometry 'rig', contact_attach required and
contact_target_z typed (the 09:00 calibration's weld_point_z: the marker height with the face on plate 9).
His mocap inputs come from our mocap node through partner_rig_state; his ball manager is his IRL one (no
radio output: QUAD4's one coil goes through our magnet_gate). ARM and TAKEOFF stay manual
(/tejen/drone_command), as in his IRL launches; supervisor:=true adds his supervisor, whose stops request a
LAND (land_now), never a disarm. Usually included by real_control_launch.py mode:=attach partner_nodes:=true.
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
from mpc_planner.geometry import cw_to_ccw_deg, tejen_plate

NS = 'tejen'
DRONE = 3
DRONE_MODEL = 'x3_drone3'
OBJECT_POSE = '/model/payload_model/pose'

# geometry:=legacy is the old partner worlds (our x3_drone3 partner rod, linear plant); geometry:=rig_twin is
# the rig-twin attach world (three_rigid_ground_rig_attach_pickup.sdf, the combined M1 subtest, Wesley 10 Oct):
# plates at r 0.225, 0.55 m rods at 60 deg, QUAD4 0.55 kg with a 0.55 m rod from a pivot 4 cm below the body
# (body to tip 0.59), flown through the rig thrust map, so his MPC takes his rig values (thrust ratio 22 with
# feedback, irl_commissioning.yaml) instead of the linear plant's 82.7.
# plate and carrier_azimuths_deg are typed as everywhere since 15 Oct (Wesley 14 Oct): our plate number, clockwise
# from +x, and clockwise degrees (30 x plate). His nodes count counter-clockwise; _his_ring converts. legacy:
# plate 3 (his 9) = -y = our old weld point (attach_y_offset -0.25), carriers at 30/270/150 cw. rig_twin: plate 9 (his
# 3) = +y (attach_y_offset 0.225), carriers at 90/330/210 cw (plates 3/11/7). contact_attach_z_m: his contact
# target above the ring origin, the tip sphere's centre on the plate (plate top 0.02 + 0.012 radius).
GEOMETRY = {
    'legacy': {
        'ring_radius': 0.25, 'ring_cable_len': 0.5, 'ring_cable_elev_deg': 45.0,
        'ring_pitch_diameter': 0.50, 'ring_collision_outer_diameter': 0.56,
        'magnet_drop_below_quad': 0.49, 'mpc_cable_length': 0.49,
        'thrust_ratio': 82.7, 'thrust_ratio_feedback': False, 'vehicle_mass': 0.643,
        'plate': '3', 'carrier_azimuths_deg': '[30.0, 270.0, 150.0]', 'contact_attach_z_m': None,
    },
    'rig_twin': {
        'ring_radius': 0.225, 'ring_cable_len': 0.55, 'ring_cable_elev_deg': 60.0,
        'ring_pitch_diameter': 0.45, 'ring_collision_outer_diameter': 0.51,
        'magnet_drop_below_quad': 0.59, 'mpc_cable_length': 0.55,
        'thrust_ratio': 22.0, 'thrust_ratio_feedback': True, 'vehicle_mass': 0.55,
        'plate': '9', 'carrier_azimuths_deg': '[90.0, 330.0, 210.0]', 'contact_attach_z_m': 0.032,
    },
    # rig_twin with 1 m cables (the carriers' rods and QUAD4's arm; plan 2026-10-15 A1): world
    # three_rigid_ground_rig_attach_pickup_L100.sdf (tools/make_attach_worlds_L100.py)
    'rig_twin_L100': {
        'ring_radius': 0.225, 'ring_cable_len': 1.0, 'ring_cable_elev_deg': 60.0,
        'ring_pitch_diameter': 0.45, 'ring_collision_outer_diameter': 0.51,
        'magnet_drop_below_quad': 1.04, 'mpc_cable_length': 1.0,
        'thrust_ratio': 22.0, 'thrust_ratio_feedback': True, 'vehicle_mass': 0.55,
        'plate': '9', 'carrier_azimuths_deg': '[90.0, 330.0, 210.0]', 'contact_attach_z_m': 0.032,
    },
    # real:=true only. The rig layout as rig_twin, his IRL values for QUAD4: body origin to magnet marker 0.575
    # (irl_commissioning.yaml, also his M2 QUAD4 geometry), his MPC's own cable length and kT (his IRL launches
    # set neither: the code defaults 0.531 and 22, with his UKF and kT feedback). The contact target is typed
    # (contact_target_z), not a geometry constant.
    'rig': {
        'ring_radius': 0.225, 'ring_cable_len': 0.55, 'ring_cable_elev_deg': 60.0,
        'ring_pitch_diameter': 0.45, 'ring_collision_outer_diameter': 0.51,
        'magnet_drop_below_quad': 0.575, 'mpc_cable_length': 0.531,
        'thrust_ratio': 22.0, 'thrust_ratio_feedback': True, 'vehicle_mass': 0.55,
        'plate': '9', 'carrier_azimuths_deg': '[90.0, 330.0, 210.0]', 'contact_attach_z_m': None,
    },
}
# real:=true: the rig's values where the sim launch has its own defaults (a typed value wins)
RIG = {
    'contact_dz': -0.01,          # Wesley 10 Oct: his descent aims the magnet face 1 cm below the plate
    'proof_rise_max': 0.03,       # card rev 5: the rig's lift-proof cap (the twin's is 0.01)
    # the twin subtest's high transit (m1c_*: the ball under QUAD4 clears the carriers' bodies at ~1.5 m)
    'pickup_lift_height': 2.3, 'minimum_search_z_m': 2.7, 'drop_approach_clearance': 1.35,
    'anchor_offset_z': -0.030,    # his IRL mocap: QUAD4's body origin to its tether anchor
}
# his planner's pickup values from his last successful IRL pickup (m1_ground_ring.yaml, 7 Oct 17:53); the
# rest of that file is his ground-ring trial, which contact_attach refuses
GROUND_RING_PICKUP_KEYS = ('min_reference_z', 'pickup_overtravel', 'pickup_max_overtravel',
                           'pickup_support_surface_overtravel_allowance_m')
TYPED_KEY = 'partner_mission_typed_args'
# the topics the contact attach adds to the diagnosis bag
CONTACT_TOPICS = ['/magnet/contact', '/magnet/object_attached', '/magnet/attach_proof',
                  '/join_planner/give_up', f'/drone_{DRONE}/take_back', f'/drone_{DRONE}/mux_state']
PICKUP_STATE = '/pickup/detachable_joint_state'
RELEASE_GATE = '/partner/release'    # our planner's release of a start-welded QUAD4, in the air after its step-out


def _flag(context, name):
    return LaunchConfiguration(name).perform(context).strip().lower() in ('1', 'true', 'yes', 'on')


def _his_ring(plate, carriers):
    """(his plate index, his carrier azimuths) from the typed plate (our number, clockwise from +x: 9 is +y) and
    carrier_azimuths_deg (clockwise degrees, '[90.0, 330.0, 210.0]'): his nodes count counter-clockwise."""
    return tejen_plate(int(plate)), cw_to_ccw_deg([float(a) for a in yaml.safe_load(carriers)])


def generate_launch_description() -> LaunchDescription:
    # _record_typed first: the rig mode tells a typed value from a sim default by it
    return LaunchDescription([OpaqueFunction(function=_record_typed)] + _arguments()
                             + [OpaqueFunction(function=_nodes)])


def _record_typed(context):
    """Note the arguments set before the declarations ran (the command line or an including launch)."""
    context.launch_configurations[TYPED_KEY] = ','.join(
        sorted(k for k in context.launch_configurations if k != TYPED_KEY))
    return []


def _nodes(context):
    if _flag(context, 'real'):
        return _rig_nodes(context)
    geometry = LaunchConfiguration('geometry').perform(context)
    if geometry not in GEOMETRY or geometry == 'rig':
        raise RuntimeError(f'partner_mission: geometry:={geometry!r}, expected one of '
                           f'{sorted(g for g in GEOMETRY if g != "rig")} (rig is real:=true)')
    G = GEOMETRY[geometry]
    contact = _flag(context, 'contact_attach')
    if contact and G['contact_attach_z_m'] is None:
        raise RuntimeError(f'partner_mission: contact_attach:=true has no contact target for geometry:={geometry}')
    confirm_release = _flag(context, 'confirm_pickup_release')
    # delivery_rejoin (14 Oct) starts holding the object: the ball's spawn weld stays, nothing frees it at start
    held = LaunchConfiguration('mission_mode').perform(context).strip() == 'delivery_rejoin'
    confirm_release = confirm_release and not held
    # his MPC arms where QUAD4 is (it re-latches its kT estimator's start height every disarmed tick) and learns kT
    # only 0.20 m above it: released in the air (our welded start, gate /partner/release) his mission never climbs
    # that far before the pickup, kT stays at its nominal 22 and he flies ~11 cm under his reference (R1503)
    airborne = LaunchConfiguration('airborne_start').perform(context).strip().lower()
    airborne = (LaunchConfiguration('start_gate_topic').perform(context).strip() == RELEASE_GATE
                if airborne in ('', 'auto') else airborne in ('1', 'true', 'yes', 'on'))
    learn_in_air = airborne and G['thrust_ratio_feedback']
    plate, azimuths = _his_ring(LaunchConfiguration('plate').perform(context) or G['plate'],
                                LaunchConfiguration('carrier_azimuths_deg').perform(context) or G['carrier_azimuths_deg'])
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
            # the plate height low-passed (Hz), typed only, as on the rig (twin check of RIG-1015-a3's switch)
            **({'attach_z_lowpass_hz': float(LaunchConfiguration('attach_z_lowpass').perform(context))}
               if LaunchConfiguration('attach_z_lowpass').perform(context).strip() else {}),
            **({'attach_twist': LaunchConfiguration('attach_twist').perform(context).strip()}
               if LaunchConfiguration('attach_twist').perform(context).strip() else {}),
            **({'attach_twist_hz': float(LaunchConfiguration('attach_twist_hz').perform(context))}
               if LaunchConfiguration('attach_twist_hz').perform(context).strip() else {}),
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
                'mission_mode': L('mission_mode').perform(context),
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
                # (the card's neighbours move one of them: contact_lead / contact_dz / contact_dx).
                # His pickup: the descent ends on the ball weld (his object flag is the weld here, not a
                # proximity flag as on the rig) and holds the tip there; his bounded retry: not on the ball
                # after his contact wait -> magnet OFF, rise, re-approach the ball where it now is; his third
                # failure gives up and we take QUAD4 back
                **({'contact_attach': True,
                    # the ring's own pose: the bottom hold judges the tip in the plate frame (Tejen 15 Oct)
                    'ring_pose_topic': '/payload/motion_capture_state',
                    # card 2026-10-11_quad4_tilt_cut (rig-port critic F6): land here if the fleet cannot take it back
                    **({'taken_back_self_land': True} if _flag(context, 'taken_back_self_land') else {}),
                    'final_attach_offset_z': ParameterValue(L('contact_dz'), value_type=float),
                    'approach_offset_x': ParameterValue(L('contact_dx'), value_type=float),
                    'approach_offset_y': ParameterValue(L('contact_dy'), value_type=float),
                    'attach_ready_target_lead_time': ParameterValue(L('contact_lead'), value_type=float),
                    'pickup_hold_on_attach': True,
                    'pickup_retry_enabled': True,
                    'pickup_retry_clearance_m': 0.10,
                    # typed only (his twin values otherwise): the rig's match gate in the twin (plan 2026-10-15 N6)
                    **{k: float(L(arg).perform(context)) for arg, k in (
                        ('match_xy', 'contact_match_xy_threshold'), ('match_speed', 'contact_match_xy_velocity_threshold'),
                        ('match_dwell', 'dwell_time_s')) if L(arg).perform(context).strip()}}
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
            # his gate is re-checked every tick, not only at the start: 0.0 paused the learning below the arm
            # height, i.e. through his whole pickup (R1505); -10 m never gates
            **({'thrust_ratio_ukf_min_height_above_start': -10.0} if learn_in_air else {}),
            'mpc_profile': ParameterValue(L('mpc_profile'), value_type=str),
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

    return [bag, *([] if held else [release]), object_bridge, *pickup_bridges, ring, pendulum, magnet, planner, backend, controller,
            supervisor]


def _irl_block(share, node, name='irl_commissioning.yaml'):
    """One node's ros__parameters from his config file, as a dict (so only that node's values are used)."""
    data = yaml.safe_load((share / 'config' / name).read_text())
    return dict(data[node]['ros__parameters'])


def _rig_nodes(context):
    """real:=true: his M1 on the rig around our real carried ring (see the module docstring)."""
    typed = set(context.launch_configurations.get(TYPED_KEY, '').split(','))

    def val(name, default):
        """A typed launch argument, else the rig default (the sim launch's own default is not the rig's)."""
        return LaunchConfiguration(name).perform(context).strip() if name in typed else default

    geometry = val('geometry', 'rig')
    if geometry != 'rig':
        raise RuntimeError(f'partner_mission real:=true flies geometry:=rig, got {geometry!r}')
    G = GEOMETRY['rig']
    if not _flag(context, 'contact_attach'):
        raise RuntimeError('partner_mission real:=true is the combined M1 contact attach: contact_attach:=true '
                           '(the ATTACH_READY hand-back is not for the rig, decisions 10 Oct)')
    # QUAD4's own numbers (critic F8): his IRL defaults are QUAD2's (body 12 / marker 22), so the rig refuses them
    # untyped; they come from Wednesday's tip record and Thursday's R10a, with weld_point_z and proof_taut_len
    for name, what in (('magnet_drop_below_quad', "QUAD4's body to magnet marker (m, the 09:00 tip record)"),
                       ('mpc_thrust_ratio', "his kT on QUAD4 (Thursday's R10a)")):
        if name not in typed or not LaunchConfiguration(name).perform(context).strip():
            raise RuntimeError(f'partner_mission real:=true: type {name}:=<{what}>; his IRL default is QUAD2\'s')
    if _flag(context, 'supervisor'):
        raise RuntimeError('partner_mission real:=true: supervisor:=true is refused on the combined rig until its '
                           'give-up stop has been through the critic (critic F9); ARM and TAKEOFF stay manual')
    target_z = LaunchConfiguration('contact_target_z').perform(context).strip()
    xy_txt = LaunchConfiguration('contact_target_xy').perform(context).strip()
    target_xy = [float(v) for v in xy_txt.replace(',', ' ').split()] if xy_txt else []
    if target_xy and len(target_xy) != 2:
        raise RuntimeError(f'contact_target_xy {xy_txt!r}: two numbers, x,y in the ring frame (m)')
    if not target_z:
        raise RuntimeError('partner_mission real:=true: type contact_target_z:=<weld_point_z> (the 09:00 '
                           'calibration: the magnet marker above the ring-origin plane with the face on plate 9)')
    plate, azimuths = _his_ring(val('plate', G['plate']), val('carrier_azimuths_deg', G['carrier_azimuths_deg']))
    share = Path(get_package_share_directory('tejen_mission'))
    dyn_share = Path(get_package_share_directory('tejen_dynamic_planner'))
    state = f'/drone_{DRONE}/motion_capture_state'
    tip, ball_pose, ball_attached = f'/{NS}/magnet_tip_pose', f'/{NS}/pickup_object/pose', f'/{NS}/object_attached'
    f = lambda name, default: float(val(name, default))  # noqa: E731

    ring = Node(
        package='tejen_mission', executable='ring_bridge', name='fake_cooperative_transport_world',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'bridge_carrier_azimuths_deg': azimuths,
            'bridge_attach_radius_m': G['ring_radius'],
            'bridge_cable_len_m': G['ring_cable_len'],
            'bridge_cable_elev_deg': G['ring_cable_elev_deg'],
            'payload_committed_trajectory_topic': '/fake_payload/committed_trajectory',
            'ring_plate_count': 12,
            'ring_plate_pitch_diameter_m': G['ring_pitch_diameter'],
            'ring_plate_diameter_m': 0.06,
            'ring_attachment_plate_index': plate,
            'ring_collision_outer_diameter_m': G['ring_collision_outer_diameter'],
            'ring_collision_top_offset_m': 0.0,
            'ring_collision_bottom_offset_m': -0.27,
            'ring_structure_height_m': 0.07,
            'enable_fake_obstacles': True,
            'fake_drone_count': 3,
            'fake_drone_radius': 0.12,
            'fake_drone_safety_radius': 0.35,
            'publish_shared_trajectories': True,
            'shared_trajectory_duration_s': 30.0,
            'shared_trajectory_control_interval_s': 0.50,
            'shared_trajectory_refresh_period_s': 1.0,
            # his contact target = our weld point: the calibrated marker height, the full ring attitude
            'bridge_attach_z_m': float(target_z),
            # one plate point for both stacks (plan 2026-10-14 WP3): our weld point's x, y in the ring frame
            **({'bridge_attach_xy_m': target_xy} if target_xy else {}),
            'full_pose_attachment': True,
            # 14 Oct load cut (Wesley): no 20 s trail MarkerArray at 50 Hz on the rig laptop
            'trail_seconds': 0.0,
            # the plate height his reference follows, low-passed (Hz; '' / 0 = raw, the default): a1 15 Oct
            **({'attach_z_lowpass_hz': f('attach_z_lowpass', 0.0)} if 'attach_z_lowpass' in typed else {}),
            # a zero attachment twist ('' = raw finite difference; card 2026-10-16_plate_height_filter, critic F4)
            **({'attach_twist': val('attach_twist', '')} if 'attach_twist' in typed else {}),
            **({'attach_twist_hz': f('attach_twist_hz', 0.5)} if 'attach_twist_hz' in typed else {}),
        }])

    # his inputs from our mocap node (the only Motive owner): his IRL publisher's outputs, under /tejen
    rig_state = Node(
        package='tejen_mission', executable='partner_rig_state', name='partner_rig_state', namespace=NS,
        output='screen',
        parameters=[{'use_sim_time': False, 'ns': f'/{NS}', 'body_topic': state, 'tip_topic': '/magnet_tip_pose',
                     'ball_topic': '/pickup/motion_capture_state',
                     'anchor_offset_z': f('anchor_offset_z', RIG['anchor_offset_z']),
                     'max_pair_age_s': 0.20}])

    # his ball: attached inferred from the magnet command, the marker and the ball body (his IRL manager); it
    # drives no radio (QUAD4's coil is our magnet_gate's, from his /tejen/magnet/command)
    ball = Node(
        package='tejen_mission', executable='magnet_attachment_manager_irl',
        name='magnet_attachment_manager_irl', namespace=NS, output='screen',
        parameters=[{**_irl_block(share, 'magnet_attachment_manager_irl'),
                     'use_sim_time': False,
                     'magnet_command_topic': f'/{NS}/magnet/command',
                     'object_attached_topic': ball_attached,
                     'attachment_state_topic': f'/{NS}/attachment_state',
                     'magnet_tip_pose_topic': tip,
                     'pickup_object_pose_topic': ball_pose,
                     'pickup_object_index': 0,
                     'enable_elrs_magnet_output': False,
                     'elrs_magnet_command_topic': f'/{NS}/magnet/ELRSCommand'}])

    ground_ring = _irl_block(share, 'online_join_planner', 'm1_ground_ring.yaml')
    planner = Node(
        package='tejen_mission', executable='online_join_planner', name='online_join_planner', output='screen',
        # his IRL order (m1_irl_*_ring.launch.py): later wins; ours last, every topic written out
        parameters=[
            str(share / 'config' / 'visual_astar_visual_only.yaml'),
            str(share / 'config' / 'c1f2b_cpp_authority.yaml'),
            _irl_block(share, 'online_join_planner'),
            str(share / 'config' / 'm1_measured_tip_pickup.yaml'),
            {k: ground_ring[k] for k in GROUND_RING_PICKUP_KEYS},
            str(share / 'config' / 'm1_measured_tip_tracking.yaml'),
            {
                'use_sim_time': False,
                'vehicle_id': 'drone_0',
                'assigned_attachment_id': 'attachment_0',
                'mission_mode': val('mission_mode', 'pickup_delivery'),
                'pickup_alignment_mode': 'measured_tip_tiered',
                'drone_state_topic': state,
                'drone_command_topic': f'/{NS}/drone_command',
                'pickup_object_pose_topic': ball_pose,
                'pickup_object_index': 0,
                'object_attached_topic': ball_attached,
                'magnet_command_topic': f'/{NS}/magnet/command',
                'use_measured_magnet_tip': True,
                'magnet_tip_pose_topic': tip,
                'payload_pose_topic': '/fake_payload/pose',
                'payload_twist_topic': '/fake_payload/twist',
                'attachment_pose_topic': '/fake_attachment_point/pose',
                'attachment_twist_topic': '/fake_attachment_point/twist',
                'external_landing_topic': '/join_planner/land_now',
                'drop_point_source': 'payload',
                'static_obstacle_count': 0,
                'pickup_lift_height': f('pickup_lift_height', RIG['pickup_lift_height']),
                'drop_approach_clearance': f('drop_approach_clearance', RIG['drop_approach_clearance']),
                'magnet_drop_below_quad': f('magnet_drop_below_quad', G['magnet_drop_below_quad']),
                # no auto-land from ATTACH_READY (its magnet OFF would drop QUAD4 off the ring)
                'attach_ready_land_after_s': 0.0,
                'contact_attach': True,
                'ring_pose_topic': '/payload/motion_capture_state',     # the hold's plate frame (Tejen 15 Oct)
                'final_attach_offset_z': f('contact_dz', RIG['contact_dz']),
                'approach_offset_x': f('contact_dx', 0.0),
                'approach_offset_y': f('contact_dy', 0.0),
                'attach_ready_target_lead_time': f('contact_lead', 0.0),
                'proof_rise_max': f('proof_rise_max', RIG['proof_rise_max']),
                # the drop ends on the measured object leaving the marker; stuck = give-up (critic F9)
                'drop_check_puck': True,
                # 0 = trust an unseen object released (no puck pose in the net); set from Wednesday's mocap check
                'drop_puck_wait_s': f('drop_puck_wait_s', 2.0),
                'contact_topic': '/magnet/contact',
                'ring_attached_topic': '/magnet/object_attached',
                'give_up_topic': '/join_planner/give_up',
                'take_back_topic': f'/drone_{DRONE}/take_back',
                'mux_state_topic': f'/drone_{DRONE}/mux_state',
                # 14 Oct load cuts (Wesley): the visual-only A* (display; the mission reference is
                # unchanged) off, and the CSV every 6th update (30 -> 5 Hz) as in the sim
                'visual_astar_enabled': False,
                'log_every_n_updates': 6,
                # 14 Oct, Tejen's tuning after RIG-1014-t5_4 (Wesley): the tip swung a median 3-4 cm round the puck
                # (QUAD4's wash off the floor), inside the 2.5 cm low tube 14-35 % of the time, so the descent stalled;
                # a wider low tube and a longer dwell at the bottom (the final capture gate stays 2.5 cm)
                'pickup_alignment_low_xy_m': 0.10,    # 14 Oct t5_5 (Wesley): 0.045 still stalled HIGH once
                'pickup_alignment_high_xy_m': 0.12,
                'pickup_alignment_low_dwell_s': 1.0,
                # 14 Oct props-off check (RIG-1014-puck, Wesley): the puck catches anywhere on its face, but t5_4/t5_5
                # lifted within 0.3 s of the proximity "attached" (12 cm), so the magnet barely touched or never did:
                # sit 1.5 s at the bottom, lift only with the tip over the puck, and his 13 Oct overtravel
                'pickup_lift_xy_dwell_s': 1.5,
                'pickup_lift_xy_tolerance': 0.04,
                'pickup_overtravel': 0.036,
                'pickup_max_overtravel': 0.036,
                'pickup_support_surface_overtravel_allowance_m': 0.024,
                # 14 Oct RIG-1014-t6f (Wesley): the slack tether's tip swings a median 3.8 cm / 0.105 m/s over plate 9,
                # so the 2.5 cm / 0.08 m/s match gate never held 1 s; the descent abort is widened past the new gate.
                # The contact itself still has to land inside our weld radius.
                'contact_match_xy_threshold': 0.05,
                'contact_match_xy_velocity_threshold': f('match_speed', 0.15) if 'match_speed' in typed else 0.15,
                # his gates hold for dwell_time_s unbroken (his 1.0 s): a3_2 (15 Oct) sat 42 s in MATCH, both conditions
                # true 36 % of the time, never a clean second; replayed, 0.5 s passes at 1.1 s. Typed only.
                **({'dwell_time_s': f('match_dwell', 1.0)} if 'match_dwell' in typed else {}),
                'contact_abort_xy_threshold': 0.08,
                'contact_abort_xy_velocity_threshold': 0.25,
                # plan 2026-10-14 WP3: hold at the bottom for the contact (gross errors and his quiet-miss rule leave),
                # three tries in the give-up time, and his measured-tip trim instead of hand-set contact_dx/dy
                'contact_bottom_hold': True,
                'contact_give_up_s': f('contact_give_up_s', 45.0),
                'target_alignment_mode': val('target_alignment_mode', 'measured_tip_trim'),
                # his trim learns only while the tip moves slower than this (his 0.08 m/s): over plate 9 on 14 Oct the
                # swinging tip read 0.16-0.20 m/s, so in replay it learned <= 1 cm in 7 s of MATCH; 0.20 learned the
                # 4-5 cm offset in 2-4 s. Typed only (the sheet's choice), his value otherwise.
                **({'target_alignment_max_relative_speed_mps': f('trim_speed_gate', 0.08)}
                   if 'trim_speed_gate' in typed else {}),
            },
        ])

    backend = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(dyn_share / 'launch' / 'c1f6_moving_rendezvous_backend.launch.py')),
        launch_arguments={'planner_rate_hz': '5.0',
                          'minimum_search_z_m': str(f('minimum_search_z_m', RIG['minimum_search_z_m'])),
                          'cooperative_preview_count': '3',
                          'use_sim_time': 'false',
                          'extra_params_file': str(share / 'config' / 'partner_backend_overrides_rig.yaml'),
                          'state_topic': state,
                          'object_attached_topic': ball_attached}.items())

    controller = Node(
        package='tejen_mpc', executable='main', name='tejen_mpc', namespace=NS, output='screen',
        parameters=[{
            **_irl_block(share, 'tejen_mpc'),
            'use_sim_time': False,
            'use_external_reference': True,
            'thrust_ratio': f('mpc_thrust_ratio', G['thrust_ratio']),
            'enable_thrust_ratio_feedback': G['thrust_ratio_feedback'],
            'cable_length': f('mpc_cable_length', G['mpc_cable_length']),
            'acados_cache_dir': os.path.expanduser('~/.cache/multi_drone_control/tejen_payload_mpc'),
            'pose_timeout_s': 0.25,
            'arming_state_feedback_period_s': 0.5,
            # his thrust-ratio feedback owns the model gain on the rig, as in his IRL flights
            'object_mass_kg': 0.0,
            'object_vehicle_mass_kg': G['vehicle_mass'],
            'object_attached_topic': ball_attached,
            'mpc_profile': LaunchConfiguration('mpc_profile').perform(context),
        }],
        remappings=[('motion_capture_state', state),
                    ('ELRSCommand', f'/drone_{DRONE}/ELRSCommand_tejen'),
                    ('telemetry', f'/drone_{DRONE}/telemetry')])

    out = [ring, rig_state, ball, planner, backend, controller]
    if _flag(context, 'supervisor'):
        out.append(Node(
            package='tejen_mission', executable='simulation_test_supervisor',
            name='simulation_test_supervisor', namespace=NS, output='screen',
            parameters=[{
                'use_sim_time': False,
                'control_mode': 'manual',          # ARM and TAKEOFF stay with the operator
                'stop_action': 'give_up',          # H5 / F9: a stale input gives up (our take-back), never disarms
                'result_path': ParameterValue(LaunchConfiguration('result_path'), value_type=str),
                'startup_timeout_s': 600.0,
                'manual_start_timeout_s': 600.0,
                'mission_timeout_s': 900.0,
                'success_dwell_s': 2.0,
                'freshness_timeout_s': 0.5,
                'planner_status_timeout_s': 3.0,
                'success_phase': 'NONE_PARTNER_WELD_OWNS_SUCCESS',
                'failure_phase': 'LANDED_DISARMED',
                'require_handoff_ready': True,
                'landing_request_phase': '',
            }],
            remappings=[('/motion_capture_state', state),
                        ('/pendulum_swing_state', f'/{NS}/pendulum_swing_state'),
                        ('/payload_world_state', f'/{NS}/payload_world_state')]))
    out.append(ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-o', LaunchConfiguration('bag_path'), state, f'/{NS}/pendulum_swing_state', tip,
             '/join_planner/reference', '/join_planner/phase', f'/drone_{DRONE}/ELRSCommand_tejen',
             '/fake_payload/pose', '/fake_attachment_point/pose', '/magnet/command', f'/{NS}/magnet/command',
             ball_attached, ball_pose, '/pickup/motion_capture_state', f'/drone_{DRONE}/magnet',
             f'/drone_{DRONE}/magnet_ours', f'/drone_{DRONE}/magnet_gate', '/join_planner/land_now',
             *CONTACT_TOPICS],
        output='screen', condition=IfCondition(LaunchConfiguration('record'))))
    return out


def _arguments():
    return [
        DeclareLaunchArgument('geometry', default_value='legacy', choices=sorted(GEOMETRY)),
        DeclareLaunchArgument('plate', default_value=''),   # our plate number (9 = +y; his 3); '' = the geometry's
        DeclareLaunchArgument('object_mass_kg', default_value='0.1'),   # the 0.1 kg object (payload_model.sdf)
        # clockwise degrees; '' = the geometry's; the all-attached start: '[210.0, 90.0, 330.0]', plate 9, gate
        # /partner/release
        DeclareLaunchArgument('carrier_azimuths_deg', default_value=''),
        DeclareLaunchArgument('contact_attach', default_value='false'),
        # 14 Oct (Wesley): floor_join takes off from the mat and goes straight to the ring rejoin (no pickup, no drop);
        # delivery_rejoin takes off already holding the object (magnet ON from boot), drops it, then rejoins
        DeclareLaunchArgument('mission_mode', default_value='pickup_delivery',
                              choices=['pickup_delivery', 'floor_join', 'delivery_rejoin']),
        DeclareLaunchArgument('contact_lead', default_value='0.0'),   # contact_attach: his hold's reference lead (s)
        DeclareLaunchArgument('contact_dz', default_value='0.0'),     # contact_attach: tip target above the contact point (m)
        DeclareLaunchArgument('contact_dx', default_value='0.0'),     # contact_attach: tip target offset along x (m)
        DeclareLaunchArgument('contact_dy', default_value='0.0'),     # contact_attach: tip target offset along y (m)
        DeclareLaunchArgument('drop_puck_wait_s', default_value='2.0'),   # rig drop check: s to wait for a puck pose; 0 = trust the release
        DeclareLaunchArgument('confirm_pickup_release', default_value='false'),
        # contact_attach: no fleet switch 1 s after TAKEN_BACK -> out 1.2 m from the ring, land, disarm (card
        # 2026-10-11_quad4_tilt_cut); false = TAKEN_BACK holds where it is, as before
        DeclareLaunchArgument('taken_back_self_land', default_value='false'),
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
        # real:=true only (the rig; the sim graph above does not read these)
        DeclareLaunchArgument('real', default_value='false'),
        DeclareLaunchArgument('contact_target_z', default_value=''),     # m, the calibrated weld_point_z
        DeclareLaunchArgument('contact_target_xy', default_value=''),    # 'x,y' m in the ring frame: our weld point ('' = his nominal plate)
        DeclareLaunchArgument('contact_give_up_s', default_value=''),    # rig: s after the first descent ('' = 45)
        DeclareLaunchArgument('target_alignment_mode', default_value=''),  # rig: '' = measured_tip_trim
        DeclareLaunchArgument('trim_speed_gate', default_value=''),      # rig: m/s, his trim's learning speed ('' = his 0.08)
        DeclareLaunchArgument('attach_z_lowpass', default_value=''),     # rig: Hz, his target's plate height low-passed ('' = raw)
        DeclareLaunchArgument('match_dwell', default_value=''),          # rig: s, his gates' unbroken hold ('' = his 1.0)
        DeclareLaunchArgument('match_xy', default_value=''),             # twin: m, his contact match xy gate ('' = his)
        DeclareLaunchArgument('attach_twist', default_value='', choices=['', 'zero', 'lowpass']),   # the bridge's twist: zero in hover, or low-passed ('' = raw)
        DeclareLaunchArgument('attach_twist_hz', default_value=''),      # attach_twist lowpass cutoff ('' = 0.5 Hz)
        DeclareLaunchArgument('match_speed', default_value=''),          # rig: m/s, his contact match speed gate ('' = 0.15)
        DeclareLaunchArgument('magnet_drop_below_quad', default_value=''),   # QUAD4's, typed (refused untyped)
        DeclareLaunchArgument('proof_rise_max', default_value=''),       # '' = the rig's 0.03
        DeclareLaunchArgument('mpc_thrust_ratio', default_value=''),     # QUAD4's, typed (refused untyped)
        DeclareLaunchArgument('mpc_cable_length', default_value=''),     # '' = his MPC's 0.531
        DeclareLaunchArgument('anchor_offset_z', default_value=''),      # '' = his IRL -0.030
        DeclareLaunchArgument('supervisor', default_value='false'),      # his supervisor (manual, stops land)
        # his MPC version, sim and rig (Wesley 14 Oct, twin first): oct10 = his 11a0683f controller, flown to
        # 14 Oct; oct14 = his d00934c1 weights, attitude-rate bounds +-3 and partial-step command execution
        DeclareLaunchArgument('mpc_profile', default_value='oct10', choices=['oct10', 'oct14']),
    ]

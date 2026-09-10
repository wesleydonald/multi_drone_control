"""
real_attach_launch.py  —  TERMINAL 2 (control, MID-FLIGHT ATTACH variant)
------------------------------------------------------------------------
Hardware twin of three_attach_launch.py: num_drones tethered drones carry the ring
payload on the OCP / dissipative stack, and ONE extra drone (id num_drones, the
"newcomer") flies in on the collaborator's approach MPC, welds its magnet to the
rim at attach_x/y_offset, and is folded into the dissipative network -- the
attach-during-circle demo the sim flies (R0234). Same node graph as the sim launch
minus everything that talks to Gazebo (bridges, betaflight inner loops, mocap
emulators, the gz DetachableJoint), and on the wall clock.

Start AFTER real_io_launch.py with the newcomer INCLUDED in its fleet:

    ros2 launch controller_quad_load real_io_launch.py num_drones:=4 attach:=true \\
        drone0_serial:=/dev/QUAD0 ... drone3_serial:=/dev/QUAD3
    ros2 launch controller_quad_load real_attach_launch.py num_drones:=3 \\
        load_mass:=<weighed ring> load_traj:=circle traj_speed:=0.2

then ARM / TAKEOFF, wait for the taut circle, and ATTACH (RViz button or
    ros2 topic pub -t 3 /magnet/command std_msgs/msg/String "{data: ON}").
DETACH 3 later releases the newcomer again (magnet OFF through the manager).

WHAT MUST BE TRUE ON THE RIG (the sim gets these for free):
  * MoCap rigid bodies: the newcomer in RIGID_BODY_TO_DRONE (id num_drones) AND a
    body on its magnet tip, MAGNET_TIP_RIGID_BODY_ID -> /magnet_tip_pose
    (drone_communication/motion_capture_publisher_node.py). The weld trigger and
    the tip-based weld capture both read the TIP, not the drone.
  * The magnet is switched by an ELRS aux channel (elrs_magnet_channel) on the
    newcomer's radio; elrs_mux merges that channel into whatever command it is
    forwarding. Verify the channel index and ON/OFF values against the receiver.
  * The magnet manager cannot feel a real weld: it declares /magnet/object_attached
    when the tip is inside weld_radius of the rim point AND slow, exactly as in sim.
    weld_radius must be inside the magnet's real capture range, or the network will
    take a drone that is not actually holding the load.
  * attach_azimuths_deg is where the three tethers really are (deg, load frame,
    3 o'clock = 0) and attach_x/y_offset where the 4th magnet sits. Both measured,
    not assumed; tools/preflight.py --real reads them back.
  * thrust_ratio stays the measured 24 (NOT the sim's derived value). The approach
    MPC gets the same 24 (a solo drone is a lighter operating point on the same
    airframe; the fixed number is the supervisor-approved choice).

Run tools/preflight.py --real before ARM: it checks the mocap rates, the rod
length from the rim-point mocap, and reads every parameter above back from the
live nodes.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue


def _args():
    return [
        DeclareLaunchArgument('num_drones', default_value='3'),      # TETHERED fleet size
        DeclareLaunchArgument('reserved_attach', default_value='1'),
        # ── rig geometry / payload: MEASURED, never the sim defaults ─────────
        DeclareLaunchArgument('cable_len', default_value='0.5'),
        DeclareLaunchArgument('attach_azimuths_deg', default_value='0,90,180'),
        DeclareLaunchArgument('load_mass', default_value='0.6'),
        DeclareLaunchArgument('target_z', default_value='0.6'),
        DeclareLaunchArgument('start_taut', default_value='true'),
        DeclareLaunchArgument('payload_rest_z', default_value='0.05'),
        # ── takeoff pace (real_control_launch defaults) ──────────────────────
        DeclareLaunchArgument('handover_elev_deg', default_value='0.0'),
        DeclareLaunchArgument('handover_settle_s', default_value='0.5'),
        DeclareLaunchArgument('lift_ramp_vel', default_value='0.20'),
        DeclareLaunchArgument('land_vel', default_value='0.20'),
        DeclareLaunchArgument('takeoff_spool_s', default_value='0.5'),
        # ── tracker ──────────────────────────────────────────────────────────
        DeclareLaunchArgument('cable_ff_scale', default_value='1.0'),
        DeclareLaunchArgument('attitude_ff', default_value='true'),
        DeclareLaunchArgument('cable_source', default_value='model'),
        DeclareLaunchArgument('thrust_ratio', default_value='24.0'),      # measured airframe kT
        DeclareLaunchArgument('takeoff_thrust_ratio', default_value='0.0'),  # = thrust_ratio (ground takeoff)
        DeclareLaunchArgument('kt_batt_sag_frac', default_value='0.0'),
        DeclareLaunchArgument('kt_batt_v_full', default_value='16.8'),
        DeclareLaunchArgument('kt_batt_v_empty', default_value='14.0'),
        DeclareLaunchArgument('kt_print_period_s', default_value='1.0'),
        DeclareLaunchArgument('terminal_vel_ref', default_value='false'),
        # velocity-command mode after the handover: the configuration every sim attach
        # result was flown in. The gains are the sim's; unmeasured on hardware.
        DeclareLaunchArgument('control_mode', default_value='velocity_after_handover'),
        DeclareLaunchArgument('vel_kp_pos', default_value='2.0'),
        DeclareLaunchArgument('vel_kv', default_value='4.0'),
        DeclareLaunchArgument('vel_ki', default_value='0.0'),
        DeclareLaunchArgument('vel_k_att', default_value='8.0'),
        # ── trajectory ───────────────────────────────────────────────────────
        DeclareLaunchArgument('auto_slot_assign', default_value='true'),
        DeclareLaunchArgument('load_traj', default_value='hover'),
        DeclareLaunchArgument('traj_speed', default_value='0.2'),
        DeclareLaunchArgument('traj_distance', default_value='1.0'),
        DeclareLaunchArgument('traj_radius', default_value='0.5'),
        # ── dissipative network (dissipative_launch defaults) ────────────────
        DeclareLaunchArgument('net_horizon_preview', default_value='true'),
        DeclareLaunchArgument('net_traj_lean', default_value='false'),
        DeclareLaunchArgument('diss_k_pay', default_value='40.0'),
        DeclareLaunchArgument('diss_k_anchor', default_value='40.0'),
        DeclareLaunchArgument('diss_c', default_value='6.0'),
        DeclareLaunchArgument('diss_k_ring', default_value='20.0'),
        DeclareLaunchArgument('diss_k_slot', default_value='18.0'),
        DeclareLaunchArgument('diss_node_mass', default_value='0.5'),
        DeclareLaunchArgument('diss_substeps', default_value='10'),
        DeclareLaunchArgument('diss_elev_deg', default_value='45.0'),
        DeclareLaunchArgument('diss_handout_tension_blend', default_value='true'),
        DeclareLaunchArgument('diss_wrench_true_attitude', default_value='false'),
        DeclareLaunchArgument('diss_ki_load', default_value='1.0'),
        DeclareLaunchArgument('diss_a_i_load_max', default_value='2.0'),
        DeclareLaunchArgument('diss_balanced_tensions', default_value='true'),
        DeclareLaunchArgument('net_land_z', default_value='0.06'),
        # ── attach ───────────────────────────────────────────────────────────
        DeclareLaunchArgument('attach_traj_hold_s', default_value='10.0'),
        DeclareLaunchArgument('handover_blend_s', default_value='3.0'),
        DeclareLaunchArgument('attach_x_offset', default_value='0.0'),
        DeclareLaunchArgument('attach_y_offset', default_value='-0.25'),   # 6 o'clock rim point
        DeclareLaunchArgument('weld_radius', default_value='0.08'),
        DeclareLaunchArgument('attach_central', default_value='false'),
        DeclareLaunchArgument('attach_handout', default_value='true'),
        DeclareLaunchArgument('attach_t_handout', default_value='12.0'),
        DeclareLaunchArgument('attach_ff_ramp_s', default_value='1.5'),
        DeclareLaunchArgument('attach_elev_deg', default_value='65.0'),
        DeclareLaunchArgument('enable_approach', default_value='true'),
        DeclareLaunchArgument('enable_approach_mpc', default_value='true'),
        DeclareLaunchArgument('approach_kt_ukf', default_value='false'),
        DeclareLaunchArgument('approach_kt_feedback', default_value='false'),
        DeclareLaunchArgument('enable_obstacle_avoidance', default_value='false'),
        DeclareLaunchArgument('obstacle_safety_radius', default_value='0.15'),
        # ── magnet radio ─────────────────────────────────────────────────────
        DeclareLaunchArgument('magnet_tip_pose_topic', default_value='/magnet_tip_pose'),
        DeclareLaunchArgument('elrs_magnet_channel', default_value='10'),
        DeclareLaunchArgument('elrs_magnet_on_value', default_value='1.0'),
        DeclareLaunchArgument('elrs_magnet_off_value', default_value='-1.0'),
    ]


def launch_setup(context, *args, **kwargs):
    n = int(LaunchConfiguration('num_drones').perform(context))
    if n < 1:
        raise RuntimeError(f'num_drones must be >= 1, got {n}')
    f = lambda name: ParameterValue(LaunchConfiguration(name), value_type=float)
    b = lambda name: ParameterValue(LaunchConfiguration(name), value_type=bool)
    i_ = lambda name: ParameterValue(LaunchConfiguration(name), value_type=int)

    # one dict per tracker node (tethered and newcomer alike)
    def tracker_params():
        return {
        'terminal_vel_ref': b('terminal_vel_ref'),
        'control_mode': LaunchConfiguration('control_mode'),
        'vel_kp_pos': f('vel_kp_pos'),
        'vel_kv': f('vel_kv'),
        'vel_ki': f('vel_ki'),
        'vel_k_att': f('vel_k_att'),
        'cable_ff_scale': f('cable_ff_scale'),
        'attitude_ff': b('attitude_ff'),
        'cable_source': LaunchConfiguration('cable_source'),
        'payload_rest_z': f('payload_rest_z'),
        'thrust_ratio': f('thrust_ratio'),
        'takeoff_thrust_ratio': f('takeoff_thrust_ratio'),
        'kt_batt_sag_frac': f('kt_batt_sag_frac'),
        'kt_batt_v_full': f('kt_batt_v_full'),
        'kt_batt_v_empty': f('kt_batt_v_empty'),
        'kt_print_period_s': f('kt_print_period_s'),
        }

    nodes = [SetParameter(name='use_sim_time', value=False)]
    for i in range(n):
        nodes.append(Node(
            package='controller_quad_load', executable='controller', name=f'controller_{i}',
            parameters=[{'drone_id': i, 'takeoff_spool_s': f('takeoff_spool_s'),
                         **tracker_params()}],
            output='screen'))

    nodes.append(Node(
        package='controller_quad_load', executable='main', name='central_controller',
        parameters=[{'num_drones': n}], output='screen'))

    nodes.append(Node(
        package='controller_dissipative', executable='dissipative', name='dissipative_controller',
        parameters=[{'num_drones': n,
                     'reserved_attach': i_('reserved_attach'),
                     'attach_cable_len': f('cable_len'),
                     'attach_central': b('attach_central'),
                     'attach_handout': b('attach_handout'),
                     'diss_t_handout': f('attach_t_handout'),
                     'attach_ff_ramp_s': f('attach_ff_ramp_s'),
                     'attach_elev_deg': f('attach_elev_deg'),
                     'diss_balanced_tensions': b('diss_balanced_tensions'),
                     'cable_len': f('cable_len'),
                     'attach_azimuths_deg': LaunchConfiguration('attach_azimuths_deg'),
                     'start_taut': b('start_taut'),
                     'load_mass': f('load_mass'),
                     'target_z': f('target_z'),
                     'lift_ramp_vel': f('lift_ramp_vel'),
                     'land_vel': f('land_vel'),
                     'handover_elev_deg': f('handover_elev_deg'),
                     'handover_settle_s': f('handover_settle_s'),
                     'auto_slot_assign': b('auto_slot_assign'),
                     'load_traj': LaunchConfiguration('load_traj'),
                     'traj_speed': f('traj_speed'),
                     'traj_distance': f('traj_distance'),
                     'traj_radius': f('traj_radius'),
                     'net_horizon_preview': b('net_horizon_preview'),
                     'net_traj_lean': b('net_traj_lean'),
                     'diss_k_pay': f('diss_k_pay'),
                     'diss_k_anchor': f('diss_k_anchor'),
                     'diss_c': f('diss_c'),
                     'diss_k_ring': f('diss_k_ring'),
                     'diss_k_slot': f('diss_k_slot'),
                     'diss_node_mass': f('diss_node_mass'),
                     'diss_substeps': i_('diss_substeps'),
                     'diss_elev_deg': f('diss_elev_deg'),
                     'diss_handout_tension_blend': b('diss_handout_tension_blend'),
                     'diss_wrench_true_attitude': b('diss_wrench_true_attitude'),
                     'attach_traj_hold_s': f('attach_traj_hold_s'),
                     'handover_blend_s': f('handover_blend_s'),
                     'diss_ki_load': f('diss_ki_load'),
                     'diss_a_i_load_max': f('diss_a_i_load_max'),
                     'net_land_z': f('net_land_z')}],
        output='screen'))

    enable_approach = (LaunchConfiguration('enable_approach').perform(context).lower()
                       in ('1', 'true', 'yes'))
    if not enable_approach:
        return nodes
    d = n                                   # the newcomer: first id after the tethered fleet
    elrs = f'/drone_{d}/ELRSCommand'

    # ELRSCommand MUX onto the newcomer's real radio: forwards the approach MPC until the
    # weld, then our tracker; merges the magnet aux channel into whichever it forwards.
    nodes.append(Node(
        package='drone_magnet', executable='elrs_mux', name=f'elrs_mux_{d}',
        parameters=[{'drone_id': d, 'latch': True,
                     'magnet_command_topic': '/magnet/ELRSCommand',
                     'magnet_channel': i_('elrs_magnet_channel')}],
        output='screen'))

    enable_approach_mpc = (LaunchConfiguration('enable_approach_mpc').perform(context).lower()
                           in ('1', 'true', 'yes'))
    if enable_approach_mpc:
        nodes.append(Node(
            package='controller_mpc_payload', executable='main', name=f'approach_mpc_{d}',
            parameters=[{'drone_id': d,
                         'use_external_reference': True,
                         'external_reference_topic': '/join_planner/reference',
                         'mpc_thrust_ratio': f('thrust_ratio'),
                         'enable_thrust_ratio_ukf': b('approach_kt_ukf'),
                         'enable_thrust_ratio_feedback': b('approach_kt_feedback'),
                         'thrust_ratio_estimator_backend': 'full_model_kt_ukf'}],
            remappings=[(elrs, f'{elrs}_tejen'),
                        ('drone_command', '/fleet/command')], output='screen'))

    # our tracker for the newcomer -> pre-mux _diss; arms and takes off with the fleet.
    nodes.append(Node(
        package='controller_quad_load', executable='controller', name=f'controller_{d}',
        parameters=[{'drone_id': d, 'takeoff_spool_s': 0.0, 'enable_diag_log': True,
                     **tracker_params()}],
        remappings=[(elrs, f'{elrs}_diss'),
                    (f'/drone_{d}/command', '/fleet/command')], output='screen'))

    nodes.append(Node(
        package='drone_magnet', executable='attach_target_publisher', name='attach_target_publisher',
        parameters=[{'payload_state_topic': '/payload/motion_capture_state',
                     'pose_topic': '/attach_target/pose',
                     'twist_topic': '/attach_target/twist',
                     'x_offset': f('attach_x_offset'),
                     'y_offset': f('attach_y_offset'),
                     'z_offset': 0.05}],
        output='screen'))

    nodes.append(Node(
        package='drone_magnet', executable='online_join_planner', name='online_join_planner',
        parameters=[{'mission_mode': 'join',
                     'drone_state_topic': f'/drone_{d}/motion_capture_state',
                     'attachment_pose_topic': '/attach_target/pose',
                     'attachment_twist_topic': '/attach_target/twist',
                     'reference_topic': '/join_planner/reference',
                     'object_attached_topic': '/magnet/object_attached',
                     'magnet_command_topic': '/magnet/command',
                     'enable_obstacle_avoidance': b('enable_obstacle_avoidance'),
                     'enable_obstacle_diagnostics': b('enable_obstacle_avoidance'),
                     'obstacle_safety_radius': f('obstacle_safety_radius'),
                     'obstacle_state_topics': [f'/drone_{i}/motion_capture_state'
                                               for i in range(n)],
                     'auto_descend': True}],
        output='screen'))

    # magnet manager on MOCAP inputs: tip PoseStamped from the tip rigid body, payload from
    # its MotionCaptureState. It declares the weld (tip inside weld_radius of the rim
    # point and slow) and drives the magnet aux channel; there is no joint to command.
    nodes.append(Node(
        package='drone_magnet', executable='magnet_attachment_manager', name='magnet_attachment_manager',
        parameters=[{'magnet_tip_pose_topic': LaunchConfiguration('magnet_tip_pose_topic'),
                     'magnet_tip_pose_msg_type': 'pose_stamped',
                     'object_pose_topic': '/payload/motion_capture_state',
                     'object_pose_msg_type': 'mocap_state',
                     'object_x_offset': f('attach_x_offset'),
                     'object_y_offset': f('attach_y_offset'),
                     'use_fallback_object_pose': False,
                     'attach_radius': f('weld_radius'),
                     'object_attached_topic': '/magnet/object_attached',
                     'command_backend': 'ros_topic',
                     'ros_attach_topic': '/payload/attach',
                     'ros_detach_topic': '/payload/detach',
                     'enable_elrs_magnet_output': True,
                     'elrs_magnet_command_topic': '/magnet/ELRSCommand',
                     'elrs_magnet_channel': i_('elrs_magnet_channel'),
                     'elrs_magnet_on_value': f('elrs_magnet_on_value'),
                     'elrs_magnet_off_value': f('elrs_magnet_off_value')}],
        output='screen'))
    return nodes


def generate_launch_description():
    return LaunchDescription(_args() + [OpaqueFunction(function=launch_setup)])

"""
three_attach_launch.py
----------------------
ATTACH stack: 3 tethered drones lift the payload with the decentralized dissipative
controller (exactly as dissipative_launch.py), PLUS a 4th free drone (x3_drone3) carrying a
0.5 m magnet arm that flies to the payload, welds on, and JOINS the fleet so the load
trajectory continues with 4 drones. Reverse of the detach flow.

World: simulation_assets/three_attach.sdf (three_rigid_ground + a standalone x3_drone3 with
a magnet arm; the magnet's DetachableJoint welds to the shared lift_system payload).

RUN ORDER (RViz owns the clock/pose bridges + mocap + fleet_viz for the 3 tethered drones):
    gz sim simulation_assets/three_attach.sdf -v 4 -r
    ros2 launch controller_quad_load rviz_quad_load_launch.py num_drones:=3 attach:=true
    ros2 launch controller_quad_load three_attach_launch.py

This launch adds: the 3 tethered trackers + fleet manager + the dissipative node
(num_drones=3, reserved_attach=1), AND the entire drone-3 chain (its own bridges + mocap,
the collaborator's approach MPC, the magnet/join nodes, our drone-3 tracker, and the
ELRSCommand handoff mux). x3_drone3 is standalone (not nested in lift_system), so its sim
interface lives HERE, not in the rviz launch.

Fleet control:
    ros2 topic pub -t 3 /fleet/command std_msgs/msg/String "{data: ARM}"
    ros2 topic pub -t 3 /fleet/command std_msgs/msg/String "{data: TAKEOFF}"
Then click ATTACH in RViz (or:  ros2 topic pub -t 3 /magnet/command std_msgs/msg/String "{data: ON}"
and let the approach run). When the magnet welds, /magnet/object_attached -> True flips the
mux to our tracker AND folds drone 3 into the dissipative network (4th member).

=== SIM-VERIFY (things that depend on the live Gazebo scene; tune here, no rebuild) ===
  * DetachableJoint child_model scoping: three_attach.sdf welds magnet_tip_link ->
    lift_system::payload. If the weld does not fire, inspect `gz topic -l | grep attach` and
    the joint's child_model name; adjust the model plugin.
  * PoseArray indices on /model/x3_drone3/pose: set mocap_3 `pose_index` (drone body) and
    magnet_tip_publisher `magnet_index`/`drone_index` after `gz topic -e -t /model/x3_drone3/pose`.
  * online_join_planner targeting (payload/attachment pose topics) -- defaults + fallback are
    a starting point for a hover approach; refine once the drone is flying.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue

PARENT_MODEL = 'lift_system'
ATTACH_DRONE_ID = 3
ATTACH_DRONE_NAME = 'x3_drone3'


def _args():
    return [
        DeclareLaunchArgument('num_drones', default_value='3'),      # TETHERED fleet size
        DeclareLaunchArgument('reserved_attach', default_value='1'), # extra network capacity
        DeclareLaunchArgument('cable_len', default_value='0.5'),
        DeclareLaunchArgument('start_taut', default_value='false'),
        DeclareLaunchArgument('handover_elev_deg', default_value='45.0'),
        DeclareLaunchArgument('handover_settle_s', default_value='0.75'),
        DeclareLaunchArgument('load_mass', default_value='0.4'),
        DeclareLaunchArgument('target_z', default_value='0.6'),
        DeclareLaunchArgument('lift_ramp_vel', default_value='0.22'),
        DeclareLaunchArgument('land_vel', default_value='0.20'),
        DeclareLaunchArgument('cable_ff_scale', default_value='1.0'),
        DeclareLaunchArgument('attitude_ff', default_value='true'),
        DeclareLaunchArgument('cable_source', default_value='model'),
        DeclareLaunchArgument('payload_rest_z', default_value='-0.1'),
        DeclareLaunchArgument('takeoff_spool_s', default_value='0.5'),
        DeclareLaunchArgument('thrust_ratio', default_value='30.0'),
        # ── kT (thrust ratio) -- kept in step with mpc_quad_load_launch.py ──
        # 30.0, not the hardware 24.0: this is a SIM launch and with
        # motorConstant=0.62e-06 the plant's true free-flight kT is ~29.5 and its
        # loaded-hover kT is ~33. At 24 the takeoff over-thrust is (33/24)^2 = 1.9x,
        # which is the takeoff bounce; at 30 it is a mild 1.2x.
        # ADAPTIVE kT is available here but OFF by default, so this stack keeps the
        # exact process set it was verified with. Turn it on with
        #   adaptive_thrust_ratio:=true
        # which also starts one kt_estimator process per drone (each runs a 39-point
        # UKF, so watch CPU on this stack -- it already runs more nodes than the
        # plain lift). See mpc_quad_load_launch.py for what every knob does.
        DeclareLaunchArgument('adaptive_thrust_ratio', default_value='false'),
        DeclareLaunchArgument('adaptive_thrust_feedback', default_value='true'),
        DeclareLaunchArgument('thrust_ratio_estimator', default_value='ukf'),
        DeclareLaunchArgument('kt_seed', default_value='33.0'),
        DeclareLaunchArgument('kt_max_deviation', default_value='0.15'),
        DeclareLaunchArgument('kt_freeze_after_s', default_value='10.0'),
        DeclareLaunchArgument('ukf_q_kt', default_value='0.001'),
        DeclareLaunchArgument('ukf_rate_hz', default_value='10.0'),
        DeclareLaunchArgument('kt_print_period_s', default_value='1.0'),
        DeclareLaunchArgument('thrust_quad_c', default_value='88.6'),
        DeclareLaunchArgument('auto_slot_assign', default_value='true'),
        DeclareLaunchArgument('load_traj', default_value='hover'),
        DeclareLaunchArgument('traj_speed', default_value='0.6'),
        DeclareLaunchArgument('traj_distance', default_value='1.0'),
        DeclareLaunchArgument('traj_radius', default_value='0.5'),
        DeclareLaunchArgument('diss_k_pay', default_value='40.0'),
        DeclareLaunchArgument('diss_k_anchor', default_value='40.0'),
        DeclareLaunchArgument('diss_c', default_value='6.0'),
        DeclareLaunchArgument('diss_k_ring', default_value='20.0'),
        DeclareLaunchArgument('diss_k_slot', default_value='18.0'),
        DeclareLaunchArgument('diss_node_mass', default_value='0.5'),
        DeclareLaunchArgument('diss_substeps', default_value='10'),
        DeclareLaunchArgument('diss_elev_deg', default_value='45.0'),
        DeclareLaunchArgument('net_land_z', default_value='0.06'),
        # UNEQUAL (moment-balanced) force sharing. Under EQUAL sharing a balanced 4-ring is
        # geometrically impossible on a fixed 120deg tripod (the fleet diverges / the load
        # tilts), which is why attach_central below defaults true. Set this true AND
        # attach_central:=false to let the newcomer be a load-bearing RING member: the network
        # then solves per-drone tensions from a 6-DOF wrench balance so an asymmetric attach set
        # holds the load LEVEL and the fleet visibly reconfigures (offline-gated, Test G). A
        # CENTRE weld still wants the central lifter -- its moment arm is ~0.
        DeclareLaunchArgument('diss_balanced_tensions', default_value='false'),
        # OFF-CENTRE weld point (world-frame metres) for the magnet tip -- see
        # attach_target_publisher. Non-zero => the newcomer welds at a ring point so
        # balanced-tension mode can reconfigure the fleet level. Must stay within the magnet
        # manager's attach_radius (0.15). Ignored/harmless with the central-lifter default.
        DeclareLaunchArgument('attach_x_offset', default_value='0.0'),
        DeclareLaunchArgument('attach_y_offset', default_value='0.0'),
        DeclareLaunchArgument('attach_radius', default_value='0.15'),
        # how the welded newcomer joins the network. true = CENTRAL lifter (tilt-free, stable,
        # the WORKING config for a centre weld). false = RING member (fleet reconfigures) --
        # only stable together with diss_balanced_tensions:=true and an OFF-CENTRE weld.
        DeclareLaunchArgument('attach_central', default_value='true'),
        # SOFT HAND-OUT (the robust ring-attach fix). A welded RING newcomer joins as a central
        # lifter and is handed out to its off-centre ring slot over attach_t_handout seconds --
        # every intermediate is near-equilibrium so the feedforward-trusting, no-integrator
        # tracker never under-thrusts and the load never runs away (the tilt 12->90deg failure).
        # On by default; ignored for a central-lifter weld (attach_central:=true). For the ring
        # reconfigure run: attach_central:=false diss_balanced_tensions:=true + an off-centre weld.
        DeclareLaunchArgument('attach_handout', default_value='true'),
        DeclareLaunchArgument('attach_t_handout', default_value='12.0'),
        # FEEDFORWARD RAMP (s): fade the welded newcomer's cable-tension feedforward 0->1 over
        # this window from the weld, so the no-integrator tracker is not slammed at attach
        # (the thrust-transient fix). 0 restores the old instant gate=1.
        DeclareLaunchArgument('attach_ff_ramp_s', default_value='1.5'),
        # SETTLE elevation (deg) for the welded newcomer. The magnet welds HIGH & near-vertical
        # over its ring point (0.5 m arm + payload height), so a full 45deg rim target makes it
        # transit far out-and-down -- the transit that drove the runaway. A steeper target (e.g.
        # 65-70) keeps it near its weld pose (small transit, mostly-vertical pull) so it settles
        # as a stable 4th ring member. Default = diss_elev_deg (unchanged 45deg rim).
        DeclareLaunchArgument('attach_elev_deg', default_value='65.0'),
        # approach drone's mocap PoseArray index (drone body). With publish_model_pose=true on
        # the standalone x3_drone3, the model WORLD pose (frame_id=world, ~1.2,0,0.12 and rising)
        # is published LAST -- verified via `gz topic -e -t /model/x3_drone3/pose`: entries are
        # [magnet_tip, rotor_3..0, magnet_arm, base_link, MODEL]. So the body is index -1; the
        # link poses ahead of it are model-relative (frozen) and must NOT be used as the body.
        DeclareLaunchArgument('attach_pose_index', default_value='-1'),
        # bring up the whole approach-drone chain; set false to fly only the 3 tethered.
        DeclareLaunchArgument('enable_approach', default_value='true'),
        # the heavy approach MPC (acados) alone; set false to run the chain without it.
        DeclareLaunchArgument('enable_approach_mpc', default_value='true'),
        # Online thrust-ratio (kT) estimation for the approach MPC. The scalar full-model UKF
        # seeds from thrust_ratio (24.0) and re-estimates kT in flight, so a battery-sag /
        # payload-mass mismatch does not leave the approach flying on a stale hover gain.
        # ..._feedback false = shadow mode (estimate + log only, MPC keeps the fixed 24.0).
        DeclareLaunchArgument('approach_kt_ukf', default_value='true'),
        DeclareLaunchArgument('approach_kt_feedback', default_value='true'),
        # Approach-time obstacle avoidance of the REAL fleet drones. true = the approach
        # routes around drones 0..n-1 (needs a SMALL safety radius, below, or it seals the
        # corridor to the payload and never welds). false = ignore the fleet and weld
        # directly (they are teammates, not obstacles) -- use this for a guaranteed attach.
        DeclareLaunchArgument('enable_obstacle_avoidance', default_value='true'),
        # Per-drone safety-sphere radius (m). The tethered drones sit ~0.38 m from the
        # payload, so keep this well under that (default 0.15) to leave a descent corridor
        # through the 120deg gap; raise it only if the approach clips a drone.
        DeclareLaunchArgument('obstacle_safety_radius', default_value='0.15'),
    ]


def launch_setup(context, *args, **kwargs):
    n = int(LaunchConfiguration('num_drones').perform(context))
    if n < 1:
        raise RuntimeError(f'num_drones must be >= 1, got {n}')
    drone_names = [f'x3_drone{i}' for i in range(n)]

    f = lambda name: ParameterValue(LaunchConfiguration(name), value_type=float)
    b = lambda name: ParameterValue(LaunchConfiguration(name), value_type=bool)
    i_ = lambda name: ParameterValue(LaunchConfiguration(name), value_type=int)

    nodes = [SetParameter(name='use_sim_time', value=True)]

    # ── 3 TETHERED drones: bridges + betaflight comm + our tracker (as dissipative_launch) ──
    for i, drone_name in enumerate(drone_names):
        nodes.append(Node(
            package='ros_gz_bridge', executable='parameter_bridge', name=f'motor_bridge_{i}',
            arguments=[f'/{drone_name}/gazebo/command/motor_speed'
                       f'@actuator_msgs/msg/Actuators]ignition.msgs.Actuators']))
        nodes.append(Node(
            package='ros_gz_bridge', executable='parameter_bridge', name=f'imu_bridge_{i}',
            arguments=[f'/{drone_name}/imu@sensor_msgs/msg/Imu[gz.msgs.IMU'],
            remappings=[(f'/{drone_name}/imu', f'/drone_{i}/imu')]))
        nodes.append(Node(
            package='ros_gz_bridge', executable='parameter_bridge', name=f'detach_bridge_{i}',
            arguments=[f'/drone_{i}/detach@std_msgs/msg/Empty]gz.msgs.Empty']))
        nodes.append(Node(
            package='simulation_communication', executable='payload_betaflight_comm',
            name=f'bf_comm_{i}',
            parameters=[{'drone_id': i, 'drone_name': drone_name,
                         'parent_model': PARENT_MODEL}]))
        nodes.append(Node(
            package='controller_quad_load', executable='controller', name=f'controller_{i}',
            parameters=[{'drone_id': i,
                         'cable_ff_scale': f('cable_ff_scale'),
                         'attitude_ff': b('attitude_ff'),
                         'cable_source': LaunchConfiguration('cable_source'),
                         'payload_rest_z': f('payload_rest_z'),
                         'takeoff_spool_s': f('takeoff_spool_s'),
                         'thrust_ratio': f('thrust_ratio'),
                         'thrust_quad_c': f('thrust_quad_c'),
                         'adaptive_thrust_ratio': b('adaptive_thrust_ratio'),
                         'adaptive_thrust_feedback': b('adaptive_thrust_feedback'),
                         'thrust_ratio_estimator':
                             LaunchConfiguration('thrust_ratio_estimator'),
                         'kt_seed': f('kt_seed'),
                         'kt_max_deviation': f('kt_max_deviation'),
                         'kt_freeze_after_s': f('kt_freeze_after_s'),
                         'kt_print_period_s': f('kt_print_period_s')}],
            output='screen'))
        # OUT-OF-LOOP kT estimator, one process per drone (see thrust_ratio_node.py).
        # Only started when adaptive_thrust_ratio is on, which is NOT the default for
        # this stack.
        nodes.append(Node(
            package='controller_quad_load', executable='kt_estimator',
            name=f'kt_estimator_{i}',
            parameters=[{'drone_id': i,
                         'thrust_ratio': f('thrust_ratio'),
                         'kt_seed': f('kt_seed'),
                         'kt_max_deviation': f('kt_max_deviation'),
                         'ukf_q_kt': f('ukf_q_kt'),
                         'ukf_rate_hz': f('ukf_rate_hz'),
                         'kt_print_period_s': f('kt_print_period_s')}],
            condition=IfCondition(PythonExpression(
                ["'", LaunchConfiguration('thrust_ratio_estimator'), "' == 'ukf' and '",
                 LaunchConfiguration('adaptive_thrust_ratio'), "'.lower() == 'true'"])),
            output='screen'))

    # ── Central fleet manager (tethered fleet only) ─────────────────────────
    nodes.append(Node(
        package='controller_quad_load', executable='main', name='central_controller',
        parameters=[{'num_drones': n}], output='screen'))

    # ── Dissipative reference generator with 1 reserved ATTACH node ─────────
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
                     'diss_k_pay': f('diss_k_pay'),
                     'diss_k_anchor': f('diss_k_anchor'),
                     'diss_c': f('diss_c'),
                     'diss_k_ring': f('diss_k_ring'),
                     'diss_k_slot': f('diss_k_slot'),
                     'diss_node_mass': f('diss_node_mass'),
                     'diss_substeps': i_('diss_substeps'),
                     'diss_elev_deg': f('diss_elev_deg'),
                     'net_land_z': f('net_land_z')}],
        output='screen'))

    # ════════════════ APPROACH DRONE (x3_drone3) CHAIN ════════════════
    # Gate the whole approach chain with enable_approach:=false to bring up ONLY the 3
    # tethered drones (isolates the tethered lift from the approach-drone nodes when
    # debugging). Default true.
    enable_approach = (LaunchConfiguration('enable_approach').perform(context).lower()
                       in ('1', 'true', 'yes'))
    if not enable_approach:
        return nodes

    d = ATTACH_DRONE_ID
    dn = ATTACH_DRONE_NAME
    elrs = f'/drone_{d}/ELRSCommand'

    # sim interface for the STANDALONE drone (not covered by the rviz/num_drones launch).
    nodes.append(Node(
        package='ros_gz_bridge', executable='parameter_bridge', name=f'motor_bridge_{d}',
        arguments=[f'/{dn}/gazebo/command/motor_speed'
                   f'@actuator_msgs/msg/Actuators]ignition.msgs.Actuators']))
    nodes.append(Node(
        package='ros_gz_bridge', executable='parameter_bridge', name=f'imu_bridge_{d}',
        arguments=[f'/{dn}/imu@sensor_msgs/msg/Imu[gz.msgs.IMU'],
        remappings=[(f'/{dn}/imu', f'/drone_{d}/imu')]))
    nodes.append(Node(
        package='ros_gz_bridge', executable='parameter_bridge', name=f'pose_bridge_{d}',
        arguments=[f'/model/{dn}/pose@geometry_msgs/msg/PoseArray[gz.msgs.Pose_V']))

    # mocap for the standalone drone -> /drone_3/motion_capture_state (state for both the
    # approach MPC and our tracker). pose_index picks the body link from the PoseArray.
    nodes.append(Node(
        package='simulation_communication', executable='payload_mocap_emulator',
        name=f'mocap_{d}',
        parameters=[{'drone_id': d, 'drone_name': dn,
                     'pose_topic': f'/model/{dn}/pose',
                     'pose_index': i_('attach_pose_index'),
                     'publish_payload': False}]))

    # betaflight inner loop: /drone_3/ELRSCommand -> /x3_drone3/.../motor_speed. Its pose sub
    # is remapped from the nested default to the standalone pose topic.
    nodes.append(Node(
        package='simulation_communication', executable='payload_betaflight_comm',
        name=f'bf_comm_{d}',
        parameters=[{'drone_id': d, 'drone_name': dn, 'parent_model': PARENT_MODEL}],
        remappings=[(f'/model/{PARENT_MODEL}/model/{dn}/pose', f'/model/{dn}/pose')]))

    # ELRSCommand MUX: forwards APPROACH (_tejen) until the weld, then OURS (_diss).
    nodes.append(Node(
        package='drone_magnet', executable='elrs_mux', name=f'elrs_mux_{d}',
        parameters=[{'drone_id': d, 'latch': True}], output='screen'))

    # (a) collaborator's approach MPC -> pre-mux _tejen. Follows /join_planner/reference.
    # Gated separately (enable_approach_mpc:=false) since it is the one heavy node (acados
    # codegen at startup) -- lets you run the rest of the approach chain without it.
    enable_approach_mpc = (LaunchConfiguration('enable_approach_mpc').perform(context).lower()
                           in ('1', 'true', 'yes'))
    if enable_approach_mpc:
        nodes.append(Node(
            package='controller_mpc_payload', executable='main', name=f'approach_mpc_{d}',
            parameters=[{'drone_id': d,
                         'use_external_reference': True,
                         'external_reference_topic': '/join_planner/reference',
                         # kT: same 24.0 the tethered trackers use, and the UKF's initial estimate.
                         'mpc_thrust_ratio': f('thrust_ratio'),
                         'enable_thrust_ratio_ukf': b('approach_kt_ukf'),
                         'enable_thrust_ratio_feedback': b('approach_kt_feedback'),
                         'thrust_ratio_estimator_backend': 'full_model_kt_ukf'}],
            # ELRSCommand out -> pre-mux _tejen. Command IN <- /fleet/command so the fleet
            # ARM/TAKEOFF (RViz ARM button) arms this drone too -- CallbackManager listens on
            # 'drone_command', which we point at the fleet command stream.
            remappings=[(elrs, f'{elrs}_tejen'),
                        ('drone_command', '/fleet/command')], output='screen'))

    # (b) our dissipative tracker for drone 3 -> pre-mux _diss. It ARMS + TAKEOFF with the fleet
    # (its /drone_3/command is remapped to /fleet/command), so it is already flying-ready when the
    # weld hands it authority. Pre-weld it has no reference (dissipative streams drone 3's ref only
    # after attach) so it just publishes an armed-idle command on _diss, which the mux does NOT
    # forward -- the real drone stays on the approach MPC until the weld. takeoff_spool_s=0 so the
    # mid-air takeover applies full MPC hover thrust immediately instead of ramping up from a floor
    # (a spool ramp would drop the drone at the handoff).
    # NOTE: deliberately NO adaptive-kT wiring on the newcomer's tracker, even when
    # the tethered drones have it on. Drone 3's thrust regime CHANGES at the weld --
    # free-flight kT is ~29.5, loaded-hover kT is ~33 -- so a learn-then-lock during
    # its approach would lock the pre-weld value and then fly the post-weld phase on
    # it. It stays on the scheduled/fixed kT, which tracks the operating point through
    # the transition by construction.
    nodes.append(Node(
        package='controller_quad_load', executable='controller', name=f'controller_{d}',
        parameters=[{'drone_id': d,
                     'cable_ff_scale': f('cable_ff_scale'),
                     'attitude_ff': b('attitude_ff'),
                     'cable_source': LaunchConfiguration('cable_source'),
                     'payload_rest_z': f('payload_rest_z'),
                     'takeoff_spool_s': 0.0,
                     'thrust_ratio': f('thrust_ratio'),
                     'thrust_quad_c': f('thrust_quad_c'),
                     # one-line ~2 Hz health log for the newcomer, to trace why it sinks/falls
                     # after the weld (z vs ref, xy error, throttle saturation, cable FF).
                     'enable_diag_log': True}],
        remappings=[(elrs, f'{elrs}_diss'),
                    (f'/drone_{d}/command', '/fleet/command')], output='screen'))

    # attach target: republish the shared payload's mocap as the join planner's magnet-tip
    # target (PoseStamped + TwistStamped). Without this the planner never publishes a
    # reference and the approach MPC never arms.
    nodes.append(Node(
        package='drone_magnet', executable='attach_target_publisher', name='attach_target_publisher',
        parameters=[{'payload_state_topic': '/payload/motion_capture_state',
                     'pose_topic': '/attach_target/pose',
                     'twist_topic': '/attach_target/twist',
                     # OFF-CENTRE weld point (world frame): with diss_balanced_tensions the 4th
                     # welds at a ring point (moment arm != 0) so the fleet reconfigures level.
                     # 0,0 = centre weld (central lifter). Keep within the manager attach_radius.
                     'x_offset': f('attach_x_offset'),
                     'y_offset': f('attach_y_offset'),
                     'z_offset': 0.05}],
        output='screen'))

    # approach planner: emits /join_planner/reference toward the payload attach target with
    # obstacle avoidance; watches /magnet/object_attached to stop once welded.
    nodes.append(Node(
        package='drone_magnet', executable='online_join_planner', name='online_join_planner',
        parameters=[{'mission_mode': 'join',
                     'drone_state_topic': f'/drone_{d}/motion_capture_state',
                     'attachment_pose_topic': '/attach_target/pose',
                     'attachment_twist_topic': '/attach_target/twist',
                     'reference_topic': '/join_planner/reference',
                     'object_attached_topic': '/magnet/object_attached',
                     'magnet_command_topic': '/magnet/command',
                     # Obstacle avoidance: treat the REAL tethered fleet drones as the moving
                     # obstacles the approach must route around (was a fake-world stand-in that
                     # nothing publishes here, so avoidance was inert). Same MotionCaptureState
                     # type/topics the planner already tracks -- pose.position = centre,
                     # twist.linear = velocity. The approach drone (id d) avoids drones 0..n-1.
                     # NOTE the geometric tension: the payload sits INSIDE the ring of tethered
                     # drones (each ~cable_len*cos(elev) ~= 0.38 m out), so a large safety sphere
                     # seals the corridor to the payload and the approach oscillates HOLD_FOR_
                     # OBSTACLE forever. obstacle_safety_radius must be small enough to leave a gap
                     # (default 0.15 here, vs the node's 0.35), or set enable_obstacle_avoidance:=
                     # false to weld without fleet avoidance (they are teammates, not obstacles).
                     'enable_obstacle_avoidance': b('enable_obstacle_avoidance'),
                     'enable_obstacle_diagnostics': b('enable_obstacle_avoidance'),
                     'obstacle_safety_radius': f('obstacle_safety_radius'),
                     'obstacle_state_topics': [f'/drone_{i}/motion_capture_state'
                                               for i in range(n)],
                     # Once ATTACH has started the approach and the drone matches position/velocity
                     # over the payload, actually LOWER the magnet to the payload (MATCH_VELOCITY ->
                     # DESCEND_TO_ATTACHMENT is gated on this). Without it the drone hovers above and
                     # never welds. Safe here because the whole approach is ATTACH-gated already.
                     'auto_descend': True}],
        output='screen'))

    # magnet tip pose (world) from the drone's poses -> /magnet_tip_pose. Verified array order:
    # the magnet_tip_link (model-relative) is index 0 and the body WORLD pose is index -1, so we
    # compose the relative tip against the body world pose. (Matches the node's own defaults.)
    nodes.append(Node(
        package='drone_magnet', executable='magnet_tip_publisher', name='magnet_tip_publisher',
        parameters=[{'x3_pose_topic': f'/model/{dn}/pose',
                     'magnet_tip_topic': '/magnet_tip_pose',
                     'magnet_index': 0,
                     'drone_index': -1,
                     'x3_link_poses_are_relative': True}]))

    # ROS->gz bridges for the magnet DetachableJoint, mirroring the per-drone detach_bridge_{i}
    # above (the PROVEN detach path). The magnet manager publishes std_msgs/Empty on these ROS
    # topics and the bridge forwards to the gz DetachableJoint. This replaces the fragile
    # `gz topic` subprocess backend, which was silently failing to reach the joint (the weld
    # persisted -> pendulum pulled sideways, armed drone dragged the payload).
    nodes.append(Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='payload_attach_bridge',
        arguments=['/payload/attach@std_msgs/msg/Empty]gz.msgs.Empty']))
    nodes.append(Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='payload_detach_bridge',
        arguments=['/payload/detach@std_msgs/msg/Empty]gz.msgs.Empty']))

    # magnet manager: auto-welds (ROS /payload/attach -> bridge -> gz) when the tip is near &
    # slow, and announces /magnet/object_attached (Bool) -- the single signal that flips the mux
    # AND folds drone 3 into the dissipative network. Uses the ros_topic backend so attach/detach
    # go through the bridge above (same mechanism as the working tethered-drone detach).
    nodes.append(Node(
        package='drone_magnet', executable='magnet_attachment_manager', name='magnet_attachment_manager',
        parameters=[{'magnet_tip_pose_topic': '/magnet_tip_pose',
                     'object_pose_topic': f'/model/{PARENT_MODEL}/model/payload/pose',
                     # payload PoseArray: index 1 is the body's WORLD pose (same entry the mocap
                     # uses), index 0 is the frozen/relative model-frame pose. Reading 0 put the
                     # payload at ~ground so distance was ~0.6 m and it never welded. Also drop the
                     # ground fallback (z=0.08): with the load LIFTED, a fallback would weld at the
                     # wrong place -- better to wait for the real pose.
                     'object_pose_index': 1,
                     # MUST match attach_target_publisher's offset: the weld triggers on tip
                     # distance to (payload centre + this offset), so the tip welds at the
                     # off-centre ring point instead of the centre (enables reconfiguration).
                     'object_x_offset': f('attach_x_offset'),
                     'object_y_offset': f('attach_y_offset'),
                     'use_fallback_object_pose': False,
                     # tip-to-reference distance at weld ~0.10 m vertical (the reference is
                     # offset off-centre by object_x/y_offset, so the horizontal term drops
                     # out). 0.15 gives margin without welding mid-descent. Launch arg.
                     'attach_radius': f('attach_radius'),
                     'object_attached_topic': '/magnet/object_attached',
                     'command_backend': 'ros_topic',
                     'ros_attach_topic': '/payload/attach',
                     'ros_detach_topic': '/payload/detach'}],
        output='screen'))

    return nodes


def generate_launch_description():
    return LaunchDescription(_args() + [OpaqueFunction(function=launch_setup)])

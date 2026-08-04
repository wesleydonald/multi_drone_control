"""
dissipative_only_launch.py
--------------------------
DISSIPATIVE-ONLY flight: the decentralized spring-damper network flies the WHOLE FLEET for
the whole flight, with no drone ever detaching or attaching. This is the "just run the
dissipative controller" counterpart of mpc_quad_load_launch.py (which flies the same fleet
from the centralized cable-aware OCP planner instead).

HOW THIS DIFFERS FROM dissipative_launch.py
    dissipative_launch.py runs the same node, but the network is only ever ENTERED by a
    /fleet/detach (or an attach weld). A run with neither event never leaves the OCP, so
    the dissipative controller effectively does nothing -- it is a detach/attach stack.
    This launch sets auto_network_handover:=true, so the node hands the fleet over as soon
    as the OCP lift tops out and settles. From that moment the network alone generates
    every drone's reference: it holds the load, runs the lateral trajectory, and lands.

TAKEOFF IS STILL THE OCP, AND THAT IS DELIBERATE
    The pure decentralized network cannot break the load off the ground from the shallow
    (~23 deg) creep handover -- the per-drone tracker stalls into a hover rather than
    executing the analytic lift (see dissipative_node.py). The network is verified stable
    only from an AIRBORNE taut config (verify_dissipative.py test A), which is exactly what
    the handover hands it. So: OCP does creep + lift, the network does everything after.
    auto_handover_settle_s controls how long to hold at the top before switching.

Run Gazebo, then RViz (it owns the clock/pose bridges, mocap emulators and fleet_viz),
then this -- matching num_drones to the world:
    gz sim simulation_assets/three_rigid_ground.sdf -v 4 -r
    ros2 launch controller_quad_load rviz_quad_load_launch.py       num_drones:=3   # FIRST
    ros2 launch controller_quad_load dissipative_only_launch.py     num_drones:=3

Fleet control (same as the other stacks):
    ros2 topic pub -t 3 /fleet/command std_msgs/msg/String "{data: ARM}"
    ros2 topic pub -t 3 /fleet/command std_msgs/msg/String "{data: TAKEOFF}"
    ros2 topic pub -t 3 /fleet/command std_msgs/msg/String "{data: LAND}"
LAND after handover is handled by the network: it descends the held load target to
net_land_z and announces /fleet/landed so the fleet manager disarms.

Watch for this line to confirm the network actually took over:
    [dissipative] auto handover: OCP lift complete and settled - ...
followed by the usual [dissipative] network diagnostics.

No detach bridges are launched here -- nothing detaches. Use dissipative_launch.py for the
detach flow and three_attach_launch.py for the mid-flight attach flow.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue

PARENT_MODEL = 'lift_system'


def _args():
    return [
        # Fleet size. MUST match the world SDF.
        DeclareLaunchArgument('num_drones', default_value='3'),
        DeclareLaunchArgument('cable_len', default_value='0.5'),
        # true = SKIP THE CREEP PHASE: the planner hands straight to the coupled OCP
        # on its first tick instead of arc-sweeping the rods up to handover_elev_deg.
        # The lift ramp and handover_settle_s still run; only the creep is removed.
        # See mpc_quad_load_launch.py's start_taut comment for what this risks on a
        # ground start (the OCP assumes taut cables; the creep exists for that reason).
        # DEFAULT TRUE, matching mpc_quad_load_launch.py: the takeoff here is the
        # SAME LoadPlanner OCP (the network only engages after the lift), so the
        # skip-the-creep result carries over directly.
        DeclareLaunchArgument('start_taut', default_value='true'),
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
        # 30.0, not the hardware 24.0: this is a SIM launch and with
        # motorConstant=0.62e-06 the sim's true loaded-hover kT is ~33. It is also the
        # kT estimator's SEED and the centre of its +/-kt_max_deviation band, so 24
        # would put the truth outside the band and pin the estimate at 27.60.
        DeclareLaunchArgument('thrust_ratio', default_value='30.0'),
        # Quadratic-plant coefficient c in a(u)=c*u^2, DERIVED FROM THE SDF -- re-derive it
        # whenever motorConstant changes:
        #     c = 4 * motorConstant * maxRotVelocity^2 / mass
        #       = 4 * 0.62e-06 * 4631^2 / 0.6  =  88.6
        # (203.0 was the value for the old motorConstant 1.42e-06; leaving it stale makes the
        # airborne schedule clip at KT_SCHED_CEIL and badly over-estimate kT -> takeoff bounce.)
        DeclareLaunchArgument('thrust_quad_c', default_value='88.6'),
        # ADAPTIVE kT: each drone estimates its own thrust ratio in flight and prints a
        # ~1 Hz [kT dN] line. adaptive_thrust_feedback:=false = shadow (print only).
        DeclareLaunchArgument('adaptive_thrust_ratio', default_value='true'),
        DeclareLaunchArgument('adaptive_thrust_feedback', default_value='true'),
        # 'ukf' = the controller_ukf method (19-state UKF over pose + 6 dynamics
        # params, mocap-driven, run out of loop by the kt_estimator node).
        # 'none' = no estimation. See mpc_quad_load_launch.py for the details.
        DeclareLaunchArgument('thrust_ratio_estimator', default_value='ukf'),
        DeclareLaunchArgument('ukf_q_kt', default_value='0.001'),
        DeclareLaunchArgument('kt_max_deviation', default_value='0.15'),
        # ESTIMATOR SEED / band centre -- the drone's ACTUAL hover kT, decoupled from
        # thrust_ratio (which is the takeoff constant and is kept LOW on purpose).
        # 33.0 = 88.6*0.376, the operating-point secant gain the schedule measures in
        # flight. Seeding at the takeoff-safe 30 left the estimate ~10% low, the MPC
        # over-throttled and the load overshot its target by ~0.18 m. 0 = use
        # thrust_ratio (the old coupled behaviour).
        DeclareLaunchArgument('kt_seed', default_value='33.0'),
        # LEARN-THEN-LOCK: estimate kT for this many seconds after becoming airborne
        # (the lift -- near-hover, well modelled, load going straight up), then FREEZE
        # it for the rest of the flight. kT is a property of the airframe and battery,
        # not of the trajectory, so there is nothing to track once it is known; and a
        # frozen estimate cannot be dragged around by cable-tension error or payload
        # swing during the trajectory. Freezing also stops kt_input, so the estimator
        # nodes idle and hand their cores back exactly when the trajectory needs them.
        # 0 = never freeze (estimate for the whole flight).
        DeclareLaunchArgument('kt_freeze_after_s', default_value='10.0'),
        DeclareLaunchArgument('ukf_rate_hz', default_value='10.0'),
        DeclareLaunchArgument('kt_print_period_s', default_value='1.0'),
        DeclareLaunchArgument('auto_slot_assign', default_value='true'),
        # LOAD reference the NETWORK flies after handover: 'hover', 'line_x', 'circle',
        # 'fig_8', 'spin'. The network keeps its trajectory clock running post-handover, so
        # these behave as they do under the OCP stack.
        DeclareLaunchArgument('load_traj', default_value='hover'),
        DeclareLaunchArgument('traj_speed', default_value='0.6'),
        DeclareLaunchArgument('traj_distance', default_value='1.0'),
        DeclareLaunchArgument('traj_radius', default_value='0.5'),
        # ── the point of this launch ──────────────────────────────────────────
        # Hand the fleet to the dissipative network automatically once the OCP lift tops
        # out, instead of waiting for a detach that never comes. Set false to get exactly
        # dissipative_launch.py behaviour (OCP all the way, network idle).
        DeclareLaunchArgument('auto_network_handover', default_value='true'),
        # Seconds to hold at the top of the lift before switching. The network seeds from
        # MEASURED positions, so give the climb transient time to die out first.
        DeclareLaunchArgument('auto_handover_settle_s', default_value='1.5'),
        # dissipative network tuning (see DissipativeParams; defaults are the tuned
        # rigid-short values). Exposed so a different geometry can be retuned live.
        # Reference-shaping for the NETWORK phase (see dissipative_node).
        #   net_horizon_preview: carry the load trajectory's future displacement along
        #     the published horizon instead of repeating node 0. ON by default -- logged
        #     A/B: payload lag 3.11 s -> 1.11 s, mean error 0.172 -> 0.081 m.
        #   net_traj_lean: tilt the reference cone onto g_eff = g - a_traj so the
        #     formation leads the load into the maneuver (the OCP's flatness relation).
        #     OFF by default: unvalidated, mini_plant cannot reproduce the radius
        #     deficit it targets. Fly it as an A/B against the logs.
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
        # floor the network descends the held load to on a network-phase LAND.
        DeclareLaunchArgument('net_land_z', default_value='0.06'),
    ]


def launch_setup(context, *args, **kwargs):
    # Resolve num_drones to a real int -- a LaunchConfiguration is an unresolved
    # substitution at generate_launch_description() time, so the per-drone loop has to
    # happen inside an OpaqueFunction.
    n = int(LaunchConfiguration('num_drones').perform(context))
    if n < 1:
        raise RuntimeError(f'num_drones must be >= 1, got {n}')
    drone_names = [f'x3_drone{i}' for i in range(n)]

    f = lambda name: ParameterValue(LaunchConfiguration(name), value_type=float)
    b = lambda name: ParameterValue(LaunchConfiguration(name), value_type=bool)
    i_ = lambda name: ParameterValue(LaunchConfiguration(name), value_type=int)

    nodes = [SetParameter(name='use_sim_time', value=True)]

    # NOTE: the clock bridge, the drone/payload POSE bridges and the mocap emulators are
    # NOT here -- they belong to rviz_quad_load_launch.py, which must be running first.
    # Launching this alone gives the trackers no pose and they sit waiting for mocap.
    for i, drone_name in enumerate(drone_names):
        nodes.append(Node(
            package='ros_gz_bridge', executable='parameter_bridge',
            name=f'motor_bridge_{i}',
            arguments=[f'/{drone_name}/gazebo/command/motor_speed'
                       f'@actuator_msgs/msg/Actuators]ignition.msgs.Actuators']))
        nodes.append(Node(
            package='ros_gz_bridge', executable='parameter_bridge',
            name=f'imu_bridge_{i}',
            arguments=[f'/{drone_name}/imu@sensor_msgs/msg/Imu[gz.msgs.IMU'],
            remappings=[(f'/{drone_name}/imu', f'/drone_{i}/imu')]))
        # No detach bridge: nothing detaches in this stack, so the world does not need to
        # be the --detachable variant.
        nodes.append(Node(
            package='simulation_communication', executable='payload_betaflight_comm',
            name=f'bf_comm_{i}',
            parameters=[{'drone_id': i, 'drone_name': drone_name,
                         'parent_model': PARENT_MODEL}]))
        # per-drone CABLE-AWARE MPC tracker. UNCHANGED from the other stacks: it consumes
        # the same reference wire format whether the OCP or the network produced it.
        nodes.append(Node(
            package='controller_quad_load', executable='controller',
            name=f'controller_{i}',
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
                         'kt_max_deviation': f('kt_max_deviation'),
                         'kt_seed': f('kt_seed'),
                         'kt_freeze_after_s': f('kt_freeze_after_s'),
                         'kt_print_period_s': f('kt_print_period_s')}],
            output='screen'))
        # OUT-OF-LOOP kT estimator, one process per drone. The parameter UKF is 39
        # acados propagations; run inside the tracker's 50 Hz timer it measured
        # 25-150 ms and stalled the control loop. Here it cannot: if it falls behind
        # it just produces estimates less often. Skipped unless the ukf backend is
        # selected -- the other backends need no helper process.
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

    # ── Central fleet manager ──────────────────────────────────────────────
    nodes.append(Node(
        package='controller_quad_load', executable='main', name='central_controller',
        parameters=[{'num_drones': n}], output='screen'))

    # ── Dissipative reference generator (OCP takeoff -> network for the rest) ──
    # reserved_attach is left at its 0 default: no mid-flight newcomer in this stack.
    nodes.append(Node(
        package='controller_dissipative', executable='dissipative',
        name='dissipative_controller',
        parameters=[{'num_drones': n,
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
                     'auto_network_handover': b('auto_network_handover'),
                     'auto_handover_settle_s': f('auto_handover_settle_s'),
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
                     'net_land_z': f('net_land_z')}],
        output='screen'))

    return nodes


def generate_launch_description():
    return LaunchDescription(_args() + [OpaqueFunction(function=launch_setup)])

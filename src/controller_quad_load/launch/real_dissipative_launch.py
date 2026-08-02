"""
real_dissipative_launch.py  —  TERMINAL 2 (control, DISSIPATIVE variant)
------------------------------------------------------------------------
Real-world control layer using the DECENTRALIZED DISSIPATIVE reference generator
instead of the centralized OCP planner. This is the hardware twin of
dissipative_launch.py, exactly as real_control_launch.py is the hardware twin of
the control half of mpc_quad_load_launch.py.

Same wiring as real_control_launch.py -- per-drone cable-aware MPC trackers plus
the central fleet manager -- but load_planner is replaced by
controller_dissipative's `dissipative` node. Every Gazebo bridge from
dissipative_launch.py (motor / imu / detach / betaflight_comm) is dropped: real
radios and real MoCap replace them, and there is no /clock on hardware
(use_sim_time:=false).

TAKEOFF IS UNCHANGED. The dissipative node is an OCP-takeoff subclass of
LoadPlanner: it flies the identical centralized OCP until the first
/fleet/detach, then hands over to the spring-damper network. With no detach ever
published it behaves like real_control_launch.py throughout -- which is exactly
what you want for a first hardware bring-up of this stack.

NO DETACH: simply never publish /fleet/detach. reserved_attach defaults to 0, so
no attach topics are subscribed either, and real_io_launch.py already hides the
RViz DETACH/ATTACH buttons. Nothing further is needed to disable them.

Start this AFTER real_io_launch.py (terminal 1) is up and MoCap is streaming --
the trackers and the dissipative node block waiting for the first
/drone_<id>/motion_capture_state and /payload/motion_capture_state poses.

Run (terminal 2, after terminal 1). Pass the SAME num_drones to both terminals:
    ros2 launch controller_quad_load real_io_launch.py num_drones:=3 \
        drone0_serial:=/dev/QUAD1 drone1_serial:=/dev/QUAD2 drone2_serial:=/dev/QUAD3
    ros2 launch controller_quad_load real_dissipative_launch.py num_drones:=3
then drive the fleet:
    ros2 topic pub -t 3 /fleet/command std_msgs/msg/String "{data: ARM}"
    ros2 topic pub -t 3 /fleet/command std_msgs/msg/String "{data: TAKEOFF}"
    ros2 topic pub -t 3 /fleet/command std_msgs/msg/String "{data: LAND}"

HARDWARE NOTES (same as real_control_launch.py, plus the network tuning):
  * thrust_ratio 24 / thrust_quad_c 0.0 -- real prop/motor kT, NOT the sim's
    45 / 203. The quadratic-kT schedule is a sim-plant fit.
  * cable_len / load_mass MUST match your physical rig.
  * The diss_* defaults are the tuned RIGID-SHORT values for cable_len=0.5,
    load_mass=0.4, drone_mass=0.6 (see dissipative_network.py). load_mass
    defaults to 0.1 here, so the stiffnesses are NOT tuned for this payload --
    they only matter after a detach, but retune before relying on them.
  * diss_elev_deg is the network's nominal cable elevation and is independent of
    handover_elev_deg (which is 0.0 on the taut real start).
  * Prove a taut hover (load_traj:=hover) before any circle/fig_8/spin.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue


def _args():
    return [
        # Must match the num_drones given to real_io_launch.py.
        DeclareLaunchArgument('num_drones', default_value='2'),
        # MUST match the physical rig, not the sim SDF.
        DeclareLaunchArgument('cable_len', default_value='0.5'),
        DeclareLaunchArgument('load_mass', default_value='0.1'),
        # Real rig starts taut and level: no creep phase, no handover arc.
        DeclareLaunchArgument('start_taut', default_value='true'),
        DeclareLaunchArgument('handover_elev_deg', default_value='0.0'),
        DeclareLaunchArgument('handover_settle_s', default_value='0.5'),
        DeclareLaunchArgument('target_z', default_value='0.6'),
        DeclareLaunchArgument('lift_ramp_vel', default_value='0.20'),
        DeclareLaunchArgument('land_vel', default_value='0.20'),
        DeclareLaunchArgument('cable_ff_scale', default_value='1.0'),
        DeclareLaunchArgument('attitude_ff', default_value='true'),
        DeclareLaunchArgument('cable_source', default_value='model'),
        DeclareLaunchArgument('payload_rest_z', default_value='0.05'),
        DeclareLaunchArgument('takeoff_spool_s', default_value='0.5'),
        # Real prop/motor kT. NOT the sim's 45 / 203.
        DeclareLaunchArgument('thrust_ratio', default_value='24.0'),
        DeclareLaunchArgument('thrust_quad_c', default_value='0.0'),
        DeclareLaunchArgument('auto_slot_assign', default_value='true'),
        DeclareLaunchArgument('load_traj', default_value='hover'),
        DeclareLaunchArgument('traj_speed', default_value='0.4'),
        DeclareLaunchArgument('traj_distance', default_value='1.0'),
        DeclareLaunchArgument('traj_radius', default_value='0.5'),
        # Dissipative network tuning (DissipativeParams). These only take effect
        # after a detach; with no detach the OCP takeoff path owns the refs.
        DeclareLaunchArgument('diss_k_pay', default_value='40.0'),
        DeclareLaunchArgument('diss_k_anchor', default_value='40.0'),
        DeclareLaunchArgument('diss_c', default_value='6.0'),
        DeclareLaunchArgument('diss_k_ring', default_value='20.0'),
        DeclareLaunchArgument('diss_k_slot', default_value='18.0'),
        DeclareLaunchArgument('diss_node_mass', default_value='0.5'),
        DeclareLaunchArgument('diss_substeps', default_value='10'),
        DeclareLaunchArgument('diss_elev_deg', default_value='45.0'),
        # Floor the network descends the held load to on a network-phase LAND.
        DeclareLaunchArgument('net_land_z', default_value='0.06'),
    ]


def launch_setup(context, *args, **kwargs):
    n = int(LaunchConfiguration('num_drones').perform(context))
    if n < 1:
        raise RuntimeError(f'num_drones must be >= 1, got {n}')

    f = lambda name: ParameterValue(LaunchConfiguration(name), value_type=float)
    b = lambda name: ParameterValue(LaunchConfiguration(name), value_type=bool)
    i_ = lambda name: ParameterValue(LaunchConfiguration(name), value_type=int)

    # Wall clock on hardware -- there is no /clock. The controllers pin this
    # internally; set it here so the dissipative node and fleet manager agree.
    nodes = [SetParameter(name='use_sim_time', value=False)]

    # ── Per-drone cable-aware MPC trackers (identical to the OCP stack) ───────
    for i in range(n):
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
                         'thrust_quad_c': f('thrust_quad_c')}],
            output='screen'))

    # ── Central fleet manager ─────────────────────────────────────────────────
    nodes.append(Node(
        package='controller_quad_load', executable='main', name='central_controller',
        parameters=[{'num_drones': n}], output='screen'))

    # ── Decentralized dissipative reference generator ─────────────────────────
    # Replaces controller_load_mpc/planner. OCP takeoff until /fleet/detach, then
    # the spring-damper network. reserved_attach is left at its default 0, so no
    # attach topics are created.
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

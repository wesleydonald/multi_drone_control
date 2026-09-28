"""
real:=true on the launches M1 and M2 fly (three_attach_launch, dissipative_launch).

Wesley 2026-09-28: the rig flies the sim launches in a real mode, not re-synced real_*
files. The launches are evaluated offline (LaunchContext, nothing is started, no serial
port or UDP socket is opened) and the resulting node graph is checked: no Gazebo-facing
node, the wall clock on every node, the rig I/O per drone, the magnet on channel 6, a
typed kT. real:=false must evaluate exactly as before, which the sim-graph tests pin.
"""
import importlib.util
import os

import pytest
pytest.importorskip('launch_ros')

from launch import LaunchContext, LaunchDescription  # noqa: E402
from launch.actions import (DeclareLaunchArgument, GroupAction,  # noqa: E402
                            IncludeLaunchDescription, LogInfo, OpaqueFunction,
                            PopEnvironment, PushEnvironment, SetLaunchConfiguration,
                            TimerAction)
from launch.actions.pop_launch_configurations import PopLaunchConfigurations  # noqa: E402
from launch.actions.push_launch_configurations import PushLaunchConfigurations  # noqa: E402
from launch.utilities import perform_substitutions  # noqa: E402
from launch_ros.actions import Node, SetParameter  # noqa: E402
from launch_ros.utilities import evaluate_parameters  # noqa: E402

LAUNCH_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'launch')

# executables that only exist to talk to Gazebo (or stand in for the rig's mocap/radio)
SIM_ONLY = {'parameter_bridge', 'payload_betaflight_comm', 'betaflight_communication',
            'tejen_betaflight_communication', 'payload_mocap_emulator',
            'tejen_motion_capture_emulator', 'magnet_tip_publisher', 'clock_throttle'}

# the M1 partner demo (configs/experiments/partner_attached_orbit.yaml, launch args)
PARTNER_ATTACHED_ORBIT = {
    'num_drones': 3, 'reserved_attach': 1, 'start_taut': True, 'load_traj': 'orbit',
    'traj_speed': 0.125, 'traj_radius': 0.5, 'cable_len': 0.5, 'load_mass': 0.86,
    'target_z': 0.6, 'attach_azimuths_deg': '150,270,30', 'attach_central': False,
    'diss_balanced_tensions': True, 'attach_handout': True, 'attach_t_handout': 12.0,
    'attach_x_offset': 0.0, 'attach_y_offset': 0.25, 'attach_elev_deg': 65.0,
    'attach_traj_hold_s': 10.0, 'attach_traj_hold_mode': 'timed',
    'control_mode': 'velocity_after_handover', 'vel_ki': 0.0, 'diss_ki_load': 1.0,
    'enable_obstacle_avoidance': False, 'kt_trim': True, 'z_ki': 0.4, 'reconfig_mode': 'ocp',
    'safety_ref_timeout_s': 4.0, 'pose_timeout_s': 3.0, 'attach_cable_len': 0.49,
    'partner': True, 'partner_attached': True}

# the M2 hand-over driver's args (tools/sim_test/drive_m2_handover.py, configs m2_bench.yaml)
M2_DRIVER = {
    'num_drones': 4, 'sim_interface': False, 'partner_m2': True, 'partner_m2_part': 'all',
    'cable_len': 0.515, 'attach_azimuths_deg': '90.0,0.0,180.0,270.0', 'attach_z': -0.005,
    'start_taut': False, 'handover_elev_deg': 45.0, 'cable_elev_deg': 45.0,
    'handover_settle_s': 1.0, 'creep_vel': 0.2, 'load_mass': 0.86, 'load_traj': 'hover',
    'target_z': 0.6, 'reconfig_mode': 'ocp', 'pose_timeout_s': 3.0,
    'safety_ref_timeout_s': 3.0, 'takeoff_spool_s': 0.0, 'airborne_start': True,
    'auto_slot_assign': False}


def _s(ctx, x):
    if x is None or isinstance(x, str):
        return x
    if hasattr(x, 'perform'):
        return x.perform(ctx)
    return perform_substitutions(ctx, list(x))


def _record(ctx, e):
    params = {}
    for d in evaluate_parameters(ctx, e._Node__parameters or []):
        params.update(d)
    glob = dict(ctx.launch_configurations.get('global_params', []))
    return {
        'package': _s(ctx, e._Node__package),
        'executable': _s(ctx, e._Node__node_executable),
        'name': _s(ctx, e._Node__node_name),
        'namespace': _s(ctx, e._Node__node_namespace),
        'params': params,
        'use_sim_time': bool(params.get('use_sim_time', glob.get('use_sim_time', False))),
        'remappings': sorted((_s(ctx, k), _s(ctx, v)) for k, v in (e._Node__remappings or [])),
        'arguments': [_s(ctx, a) for a in (e._Node__arguments or [])],
    }


def _collect(entities, ctx, out):
    for e in entities:
        if getattr(e, 'condition', None) is not None and not e.condition.evaluate(ctx):
            continue
        if isinstance(e, (DeclareLaunchArgument, SetParameter, SetLaunchConfiguration,
                          PushLaunchConfigurations, PopLaunchConfigurations,
                          PushEnvironment, PopEnvironment)):
            e.execute(ctx)
        elif isinstance(e, Node):
            out.append(_record(ctx, e))
        elif isinstance(e, LogInfo):
            continue
        elif isinstance(e, LaunchDescription):
            _collect(e.entities, ctx, out)
        elif isinstance(e, TimerAction):
            _collect(e.actions, ctx, out)
        elif isinstance(e, (GroupAction, IncludeLaunchDescription, OpaqueFunction)):
            _collect(e.execute(ctx) or [], ctx, out)
        else:
            raise AssertionError(f'unhandled launch entity {type(e).__name__}')


def evaluate(launch_file, **args):
    """Return the node graph a launch file would start with these args (list of dicts)."""
    path = launch_file if os.path.isabs(launch_file) else os.path.join(LAUNCH_DIR, launch_file)
    spec = importlib.util.spec_from_file_location('launch_under_test', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    ctx = LaunchContext()
    ctx.launch_configurations.update({k: str(v).lower() if isinstance(v, bool) else str(v)
                                      for k, v in args.items()})
    out = []
    _collect(mod.generate_launch_description().entities, ctx, out)
    return out


def _by_name(nodes):
    return {(n['namespace'] or '') + '/' + n['name']: n for n in nodes}


@pytest.fixture(autouse=True)
def _installed():
    try:
        from ament_index_python.packages import get_package_share_directory
        get_package_share_directory('controller_quad_load')
        get_package_share_directory('drone_visualisation')
    except Exception:
        pytest.skip('source install/setup.bash')


def _assert_rig_graph(nodes, n_radio, kt):
    assert nodes, 'empty graph'
    sim = sorted(n['name'] for n in nodes if n['executable'] in SIM_ONLY)
    assert not sim, f'Gazebo-facing nodes in real mode: {sim}'
    assert not [n['name'] for n in nodes if n['use_sim_time']], 'use_sim_time true in real mode'
    g = _by_name(nodes)
    assert sum(n['executable'] == 'motion_capture_publisher_node' for n in nodes) == 1
    for i in range(n_radio):
        radio = g[f'/drone_{i}/elrs_interface']
        assert radio['params']['serial_port'] == f'/dev/QUAD{i + 1}'
        assert radio['params']['magnet_channel'] == 6
    assert f'/drone_{n_radio}/elrs_interface' not in g
    assert 'central_controller' in {n['name'] for n in nodes}
    for n in nodes:
        if n['executable'] == 'controller':
            p = n['params']
            assert isinstance(p['thrust_ratio'], float) and p['thrust_ratio'] == kt
            assert p['takeoff_thrust_ratio'] == 0.0            # ground takeoff: = thrust_ratio
            assert p['pose_timeout_s'] == 0.25 and p['safety_ref_timeout_s'] == 1.0
            assert p['payload_rest_z'] == 0.05
    return g


# ── M1 / attach rungs: three_attach_launch.py ─────────────────────────────────────────

def test_m1_real_graph_partner_demo():
    # the sim config's loose Gazebo watchdogs are refused on the rig, so they are dropped
    args = {k: v for k, v in PARTNER_ATTACHED_ORBIT.items()
            if k not in ('pose_timeout_s', 'safety_ref_timeout_s')}
    nodes = evaluate('three_attach_launch.py', real=True, thrust_ratio=24.0, **args)
    g = _assert_rig_graph(nodes, n_radio=4, kt=24.0)
    assert g['/central_controller']['params']['num_drones'] == 4
    assert g['/dissipative_controller']['params']['z_taut_gate'] == 0.9
    # M1's welded start: every magnet ON from boot, drone 3's included (auto)
    assert _latches(g, 4) == ['ON', 'ON', 'ON', 'ON']
    mux = g['/elrs_mux_3']['params']
    assert mux['magnet_channel'] == 6 and mux['magnet_command_topic'] == ''
    assert mux['handoff_topic'] == '/join_planner/handoff_ready'
    mgr = g['/magnet_attachment_manager']['params']
    assert mgr['magnet_latch_topic'] == '/drone_3/magnet'
    assert mgr['enable_elrs_magnet_output'] is False
    assert g['/dissipative_controller']['params']['detach_magnet'] is True
    assert mgr['magnet_tip_pose_topic'] == '/magnet_tip_pose'
    assert mgr['magnet_tip_pose_msg_type'] == 'pose_stamped'
    assert mgr['object_pose_topic'] == '/payload/motion_capture_state'
    assert mgr['object_pose_msg_type'] == 'mocap_state'
    assert mgr['attach_speed_threshold'] == 0.05 and mgr['attach_dwell_time_s'] == 0.15
    assert mgr['velocity_clock'] == 'sim'
    assert mgr['command_backend'] == 'none' and mgr['attached_at_start'] is True
    assert '/approach_mpc_3' not in g and '/online_join_planner' not in g   # partner flies it
    assert g['/controller_3']['params']['thrust_ratio'] == 24.0


def test_m1_real_graph_our_approach_rung():
    """R8a/R8b shape: our tracker flies the newcomer, no partner, no approach MPC."""
    nodes = evaluate('three_attach_launch.py', real=True, thrust_ratio=22.5,
                     enable_approach_mpc=False, weld_radius=0.05, start_taut=False)
    g = _assert_rig_graph(nodes, n_radio=4, kt=22.5)
    assert '/online_join_planner' in g and '/attach_target_publisher' in g
    assert g['/elrs_mux_3']['params']['approach_stream'] is False
    mgr = g['/magnet_attachment_manager']['params']
    assert mgr['attach_radius'] == 0.05 and mgr['attach_speed_threshold'] == 0.05
    assert g['/dissipative_controller']['params']['attach_approach'] is True


def test_m1_real_graph_without_the_approach_chain():
    nodes = evaluate('three_attach_launch.py', real=True, thrust_ratio=24,
                     enable_approach=False)
    g = _assert_rig_graph(nodes, n_radio=3, kt=24.0)
    assert '/elrs_mux_3' not in g and '/magnet_attachment_manager' not in g


def test_m1_real_graph_explicit_serials_and_no_rig_io():
    nodes = evaluate('three_attach_launch.py', real=True, thrust_ratio=24,
                     enable_approach_mpc=False, drone2_serial='/dev/ttyUSB7')
    assert _by_name(nodes)['/drone_2/elrs_interface']['params']['serial_port'] == '/dev/ttyUSB7'
    nodes = evaluate('three_attach_launch.py', real=True, thrust_ratio=24,
                     enable_approach_mpc=False, real_io=False)
    assert not [n for n in nodes if n['executable'] in (
        'elrs_interface', 'motion_capture_publisher_node')]


@pytest.mark.parametrize('launch', ['three_attach_launch.py', 'dissipative_launch.py'])
@pytest.mark.parametrize('kt', ['auto', 'twentyfour', ''])
def test_real_refuses_a_derived_or_missing_kt(launch, kt):
    with pytest.raises(RuntimeError, match='thrust_ratio'):
        evaluate(launch, real=True, thrust_ratio=kt)


@pytest.mark.parametrize('launch', ['three_attach_launch.py', 'dissipative_launch.py'])
def test_real_refuses_looser_watchdogs(launch):
    with pytest.raises(RuntimeError, match='pose_timeout_s'):
        evaluate(launch, real=True, thrust_ratio=24, pose_timeout_s=0.3)
    with pytest.raises(RuntimeError, match='safety_ref_timeout_s'):
        evaluate(launch, real=True, thrust_ratio=24, safety_ref_timeout_s=3.0)
    # tighter than the rig is the operator's call
    nodes = evaluate(launch, real=True, thrust_ratio=24, pose_timeout_s=0.2,
                     enable_approach_mpc=False)
    assert {n['params']['pose_timeout_s'] for n in nodes if n['executable'] == 'controller'} \
        == {0.2}


def test_real_and_sil_are_exclusive():
    with pytest.raises(RuntimeError, match='sil'):
        evaluate('three_attach_launch.py', real=True, sil=True, thrust_ratio=24)


def test_real_refuses_the_approach_mpc():
    # unflown on hardware, unstable on the linear plant, armed by /magnet/command ON
    with pytest.raises(RuntimeError, match='enable_approach_mpc'):
        evaluate('three_attach_launch.py', real=True, thrust_ratio=24)
    g = _by_name(evaluate('three_attach_launch.py', real=True, thrust_ratio=24, partner=True))
    assert '/approach_mpc_3' not in g


def test_real_newcomer_must_be_drone_3():
    # num_drones 4 would put carrier 3 and the newcomer's mux on one radio
    with pytest.raises(RuntimeError, match='num_drones'):
        evaluate('three_attach_launch.py', real=True, thrust_ratio=24,
                 enable_approach_mpc=False, num_drones=4)
    g = _by_name(evaluate('three_attach_launch.py', real=True, thrust_ratio=24,
                          num_drones=4, enable_approach=False, reserved_attach=0))
    assert g['/drone_3/elrs_interface']['params']['magnet_initial'] == 'ON'
    assert '/elrs_mux_3' not in g


def _rviz_panel(nodes):
    cfg = [n for n in nodes if n['executable'] == 'rviz2'][0]['arguments'][1]
    with open(cfg) as fh:
        return fh.read()


def test_rig_panel_rows():
    # M1 needs ATTACH; M2 has no detach step, so no DETACH row
    m1 = evaluate('three_attach_launch.py', real=True, thrust_ratio=24, enable_approach_mpc=False)
    assert 'ShowAttach: true' in _rviz_panel(m1)
    m2 = evaluate('dissipative_launch.py', real=True, thrust_ratio=24, partner_m2=True)
    panel = _rviz_panel(m2)
    assert 'ShowDetach: false' in panel and 'ShowAttach: false' in panel


def test_magnet_channel_is_one_launch_arg():
    g = _by_name(evaluate('three_attach_launch.py', real=True, thrust_ratio=24,
                          enable_approach_mpc=False, magnet_channel=7))
    for i in range(4):
        assert g[f'/drone_{i}/elrs_interface']['params']['magnet_channel'] == 7
    assert g['/elrs_mux_3']['params']['magnet_channel'] == 7
    assert 'elrs_magnet_channel' not in g['/magnet_attachment_manager']['params']


# ── P10: one magnet path per drone (the radio's String latch) ─────────────────────────

def _latches(g, n_radio):
    return [g[f'/drone_{i}/elrs_interface']['params']['magnet_initial'] for i in range(n_radio)]


def _muxes(g):
    return [v['params'] for k, v in g.items() if v['executable'] == 'elrs_mux']


R8A = {'num_drones': 3, 'reserved_attach': 1, 'attach_azimuths_deg': '30,150,270',
       'attach_x_offset': 0.0, 'attach_y_offset': 0.25, 'reconfig_mode': 'ocp',
       'enable_approach_mpc': False, 'weld_radius': 0.0, 'start_taut': False}


def test_p10_m1_partner_demo_magnet_paths():
    args = {k: v for k, v in PARTNER_ATTACHED_ORBIT.items()
            if k not in ('pose_timeout_s', 'safety_ref_timeout_s')}
    g = _by_name(evaluate('three_attach_launch.py', real=True, thrust_ratio=24.0, **args))
    assert all(m['magnet_command_topic'] == '' for m in _muxes(g)) and _muxes(g)
    mgr = g['/magnet_attachment_manager']['params']
    assert mgr['magnet_latch_topic'] == '/drone_3/magnet' and mgr['attached_at_start'] is True
    assert g['/dissipative_controller']['params']['detach_magnet'] is True
    assert g['/dissipative_controller']['params']['start_attached'] is True
    assert _latches(g, 4) == ['ON'] * 4


def test_p10_r8a_dry_approach_keeps_drone_3_off():
    g = _by_name(evaluate('three_attach_launch.py', real=True, thrust_ratio=24.0, **R8A))
    assert _latches(g, 4) == ['ON', 'ON', 'ON', 'OFF']
    # weld blocked: the manager runs (tip distance in its state) but drives no radio
    mgr = g['/magnet_attachment_manager']['params']
    assert mgr['magnet_latch_topic'] == '' and mgr['enable_elrs_magnet_output'] is False
    assert mgr['attach_radius'] == 0.0 and mgr['attached_at_start'] is False
    assert all(m['magnet_command_topic'] == '' for m in _muxes(g))
    assert g['/dissipative_controller']['params']['detach_magnet'] is True
    assert g['/dissipative_controller']['params']['start_attached'] is False


def test_p10_r8a_refuses_a_magnet_that_could_catch_without_a_weld():
    with pytest.raises(RuntimeError, match='no weld'):
        evaluate('three_attach_launch.py', real=True, thrust_ratio=24.0,
                 **dict(R8A, attach_magnet_initial='ON'))
    with pytest.raises(RuntimeError, match='no weld'):
        evaluate('three_attach_launch.py', real=True, thrust_ratio=24.0,
                 **dict(R8A, drone3_magnet_initial='ON'))
    with pytest.raises(RuntimeError, match='partner_attached'):
        evaluate('three_attach_launch.py', real=True, thrust_ratio=24.0, weld_radius=0.0,
                 partner=True, partner_attached=True)


def test_p10_r8b_newcomer_latch_is_the_managers():
    g = _by_name(evaluate('three_attach_launch.py', real=True, thrust_ratio=24.0,
                          **dict(R8A, weld_radius=0.03)))
    assert _latches(g, 4) == ['ON', 'ON', 'ON', 'OFF']
    assert g['/magnet_attachment_manager']['params']['magnet_latch_topic'] == '/drone_3/magnet'


def test_p10_r8a_literal_no_approach_chain():
    """enable_approach:=false: drone 3 has no radio, no mux and no manager at all."""
    g = _by_name(evaluate('three_attach_launch.py', real=True, thrust_ratio=24.0,
                          **dict(R8A, enable_approach=False)))
    assert _latches(g, 3) == ['ON'] * 3 and '/drone_3/elrs_interface' not in g
    assert not _muxes(g) and '/magnet_attachment_manager' not in g
    assert g['/dissipative_controller']['params']['detach_magnet'] is True


def test_p10_per_drone_latch_overrides_and_refusals():
    g = _by_name(evaluate('three_attach_launch.py', real=True, thrust_ratio=24.0,
                          **dict(R8A, weld_radius=0.03, drone1_magnet_initial='off',
                                 attach_magnet_initial='ON')))
    assert _latches(g, 4) == ['ON', 'OFF', 'ON', 'ON']
    for bad in ('', 'maybe'):
        with pytest.raises(RuntimeError, match='ON or OFF'):
            evaluate('three_attach_launch.py', real=True, thrust_ratio=24.0,
                     **dict(R8A, magnet_initial=bad))
        with pytest.raises(RuntimeError, match='ON or OFF'):
            evaluate('dissipative_launch.py', real=True, thrust_ratio=24.0, partner_m2=True,
                     drone2_magnet_initial=bad)


def test_p10_m2_magnet_paths():
    args = dict(M2_DRIVER, pose_timeout_s=0.25, safety_ref_timeout_s=1.0)
    g = _by_name(evaluate('dissipative_launch.py', real=True, thrust_ratio=24.0, **args))
    muxes = _muxes(g)
    assert len(muxes) == 4
    assert all(m['magnet_command_topic'] == '' and m['magnet_channel'] == 6 for m in muxes)
    assert _latches(g, 4) == ['ON'] * 4
    assert g['/dissipative_controller']['params']['detach_magnet'] is True
    assert '/magnet_attachment_manager' not in g
    for part in ('muxes', 'controllers'):
        g = _by_name(evaluate('dissipative_launch.py', real=True, thrust_ratio=24.0,
                              **dict(args, partner_m2_part=part)))
        if part == 'muxes':
            assert all(m['magnet_command_topic'] == '' for m in _muxes(g)) and _muxes(g)
            assert '/dissipative_controller' not in g
        else:
            assert not _muxes(g)
            assert g['/dissipative_controller']['params']['detach_magnet'] is True


def test_p10_sim_graphs_carry_no_rig_magnet_params():
    for launch, args in (('three_attach_launch.py', PARTNER_ATTACHED_ORBIT),
                         ('three_attach_launch.py', dict(R8A, pose_timeout_s=1.0)),
                         ('dissipative_launch.py', M2_DRIVER)):
        g = _by_name(evaluate(launch, **args))
        assert 'detach_magnet' not in g['/dissipative_controller']['params']
        assert all('magnet_command_topic' not in m for m in _muxes(g))
        if '/magnet_attachment_manager' in g:
            assert 'magnet_latch_topic' not in g['/magnet_attachment_manager']['params']


# ── M2: dissipative_launch.py partner_m2 ──────────────────────────────────────────────

def test_m2_real_graph():
    args = dict(M2_DRIVER, pose_timeout_s=0.25, safety_ref_timeout_s=1.0)
    nodes = evaluate('dissipative_launch.py', real=True, thrust_ratio=24.0, **args)
    g = _assert_rig_graph(nodes, n_radio=4, kt=24.0)
    for i in range(4):
        mux = g[f'/elrs_mux_{i}']['params']
        assert mux['magnet_channel'] == 6 and mux['handoff_topic'] == '/fleet/handover'
        assert ('/drone_%d/ELRSCommand' % i, '/drone_%d/ELRSCommand_diss' % i) in \
            g[f'/controller_{i}']['remappings']
        assert g[f'/drone_{i}/elrs_interface']['params']['magnet_initial'] == 'ON'
    assert g['/central_controller']['params']['num_drones'] == 4
    assert '/payload_mocap' not in g


@pytest.mark.parametrize('mode', ['sim', 'sil', 'real'])
def test_m1_drone3_takes_commands_from_the_manager_only(mode):
    """One command path (Wesley 2026-09-29): drone 3's tracker is armed and sent TAKEOFF by
    the fleet manager like the carriers, never straight from the /fleet/command broadcast
    (which let it take off when the manager refused)."""
    args = {'num_drones': 3, 'sil': True} if mode == 'sil' else dict(PARTNER_ATTACHED_ORBIT)
    if mode == 'real':
        args.update(real=True, thrust_ratio=24.0, pose_timeout_s=0.25, safety_ref_timeout_s=1.0)
    g = _by_name(evaluate('three_attach_launch.py', **args))
    assert not [r for r in g['/controller_3']['remappings'] if r[1] == '/fleet/command']
    assert g['/central_controller']['params']['num_drones'] == 4


def test_superseded_rig_attach_launch_refuses():
    with pytest.raises(RuntimeError, match='superseded'):
        evaluate('real_attach_launch.py', num_drones=3)


def test_m1_manager_counts_only_launched_trackers():
    g = _by_name(evaluate('three_attach_launch.py', num_drones=3, enable_approach=False))
    assert '/controller_3' not in g
    assert g['/central_controller']['params']['num_drones'] == 3


@pytest.mark.parametrize('real', [False, True])
def test_m2_manager_feedback_stays_under_ours(real):
    """His MPC publishes a 2 Hz armed heartbeat on /drone_i/arming_state_feedback in M2
    (m2c_vehicle_controller.launch.py): our manager and trackers must stay on /ours, or his
    False before his ARM reaches our fault scoping and ARM gate."""
    args = dict(M2_DRIVER, real=True, thrust_ratio=24.0, pose_timeout_s=0.25,
                safety_ref_timeout_s=1.0) if real else dict(M2_DRIVER)
    g = _by_name(evaluate('dissipative_launch.py', **args))
    for i in range(4):
        fb = (f'/drone_{i}/arming_state_feedback', f'/ours/drone_{i}/arming_state_feedback')
        assert fb in g['/central_controller']['remappings']
        assert fb in g[f'/controller_{i}']['remappings']


def test_m2_real_graph_plain_carry():
    """dissipative_launch without partner_m2 (R3/R4 on the M2 planner): no muxes."""
    nodes = evaluate('dissipative_launch.py', real=True, thrust_ratio=24.0, num_drones=3)
    g = _assert_rig_graph(nodes, n_radio=3, kt=24.0)
    assert not [k for k in g if 'elrs_mux' in k]
    assert not [n for n in nodes if n['remappings'] and n['executable'] == 'controller']


# ── Q7: the rig defaults to the creep floor start and the OCP resize ──────────────────

Q7_KEYS = ('start_taut', 'handover_elev_deg', 'handover_settle_s', 'creep_vel', 'reconfig_mode')
Q7_RIG = (False, 45.0, 2.0, 0.2, 'ocp')
Q7_SIM = {'three_attach_launch.py': (True, 45.0, 0.75, 0.10, 'network'),
          'dissipative_launch.py': (False, 45.0, 1.0, 0.2, 'network')}
# the minimal rig line of each launch (R8a shape for M1, the partner_m2 M2 line)
Q7_RIG_LINE = {'three_attach_launch.py': dict(enable_approach_mpc=False, weld_radius=0.0),
               'dissipative_launch.py': dict(partner_m2=True, num_drones=4)}


def _q7(nodes):
    p = _by_name(nodes)['/dissipative_controller']['params']
    return tuple(p[k] for k in Q7_KEYS)


@pytest.mark.parametrize('launch', sorted(Q7_SIM))
def test_q7_real_defaults_to_the_creep_floor_start_and_ocp(launch):
    nodes = evaluate(launch, real=True, thrust_ratio=24.0, **Q7_RIG_LINE[launch])
    assert _q7(nodes) == Q7_RIG


@pytest.mark.parametrize('launch', sorted(Q7_SIM))
def test_q7_real_keeps_typed_values(launch):
    typed = dict(start_taut=True, handover_elev_deg=65.0, handover_settle_s=0.5,
                 creep_vel=0.15, reconfig_mode='network')
    nodes = evaluate(launch, real=True, thrust_ratio=24.0, **Q7_RIG_LINE[launch], **typed)
    assert _q7(nodes) == tuple(typed[k] for k in Q7_KEYS)
    # a typed value equal to the sim default is typed too: kept, not swapped for the rig's
    sim = dict(zip(Q7_KEYS, Q7_SIM[launch]))
    nodes = evaluate(launch, real=True, thrust_ratio=24.0, **Q7_RIG_LINE[launch], **sim)
    assert _q7(nodes) == Q7_SIM[launch]


def test_q7_real_keeps_one_typed_value_and_defaults_the_rest():
    nodes = evaluate('three_attach_launch.py', real=True, thrust_ratio=24.0,
                     **Q7_RIG_LINE['three_attach_launch.py'], creep_vel=0.3)
    assert _q7(nodes) == (False, 45.0, 2.0, 0.3, 'ocp')


def test_q7_typed_watchdog_at_the_sim_default_is_refused():
    # typed 1.0 (three_attach's sim default) is the operator's value, so it is checked
    with pytest.raises(RuntimeError, match='pose_timeout_s'):
        evaluate('three_attach_launch.py', real=True, thrust_ratio=24.0,
                 **Q7_RIG_LINE['three_attach_launch.py'], pose_timeout_s=1.0)


@pytest.mark.parametrize('launch', sorted(Q7_SIM))
def test_q7_sim_defaults_unchanged(launch):
    assert _q7(evaluate(launch)) == Q7_SIM[launch]
    assert _q7(evaluate(launch, real=False)) == Q7_SIM[launch]


# ── real:=false: the sim graphs M1 and M2 fly ─────────────────────────────────────────

def test_sim_m1_graph_is_unchanged_in_shape():
    nodes = evaluate('three_attach_launch.py', **PARTNER_ATTACHED_ORBIT)
    assert all(n['use_sim_time'] for n in nodes)
    assert not [n for n in nodes if n['executable'] in ('elrs_interface',
                                                        'motion_capture_publisher_node')]
    g = _by_name(nodes)
    assert '/magnet_tip_publisher' in g and '/payload_attach_bridge' in g
    mgr = g['/magnet_attachment_manager']['params']
    assert mgr['command_backend'] == 'ros_topic' and 'elrs_magnet_channel' not in mgr
    assert 'magnet_command_topic' not in g['/elrs_mux_3']['params']
    c = g['/controller_0']['params']
    assert (c['pose_timeout_s'], c['safety_ref_timeout_s'],
            c['payload_rest_z']) == (3.0, 4.0, -0.1)


def test_sim_m2_graph_is_unchanged_in_shape():
    nodes = evaluate('dissipative_launch.py', **M2_DRIVER)
    assert all(n['use_sim_time'] for n in nodes)
    g = _by_name(nodes)
    assert '/payload_mocap' in g
    assert not [n for n in nodes if n['executable'] in ('elrs_interface',
                                                        'motion_capture_publisher_node')]
    assert 'magnet_channel' not in g['/elrs_mux_0']['params']

"""
The rig modes M1 and M2 fly (real_control_launch.py mode:=attach | m2) and their sim graphs.

Wesley 2026-09-28: the rig flies the sim graphs in a rig mode, not re-synced copies. The
launches are evaluated offline (LaunchContext, nothing is started, no serial port or UDP
socket is opened) and the resulting node graph is checked: no Gazebo-facing node, the wall
clock on every node, the rig I/O per drone, the magnet on channel 6, a typed kT. The sim
modes must evaluate exactly as before, which the sim-graph tests pin.
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
            'tejen_motion_capture_emulator', 'magnet_tip_publisher', 'clock_throttle',
            'sim_magnet'}

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
    """Return the node graph a launch file would start with these args (list of dicts).
    For the control launches, knobs that are not launch arguments go in a params_file."""
    args = {k: str(v).lower() if isinstance(v, bool) else str(v) for k, v in args.items()}
    side = {'sim_control_launch.py': 'sim', 'real_control_launch.py': 'real'}.get(launch_file)
    if side:
        import tempfile
        import yaml
        from bringup.profiles import split
        args, params = split(side, args)
        if params:
            with tempfile.NamedTemporaryFile('w', suffix='.yaml', delete=False) as fh:
                yaml.safe_dump(params, fh)
            args['params_file'] = fh.name
    path = launch_file if os.path.isabs(launch_file) else os.path.join(LAUNCH_DIR, launch_file)
    spec = importlib.util.spec_from_file_location('launch_under_test', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    ctx = LaunchContext()
    ctx.launch_configurations.update(args)
    out = []
    _collect(mod.generate_launch_description().entities, ctx, out)
    return out


def sim(mode, **args):
    return evaluate('sim_control_launch.py', mode=mode, **args)


def rig(mode, **args):
    return evaluate('real_control_launch.py', mode=mode, **args)


def _by_name(nodes):
    return {(n['namespace'] or '') + '/' + n['name']: n for n in nodes}


@pytest.fixture(autouse=True)
def _installed():
    try:
        from ament_index_python.packages import get_package_share_directory
        get_package_share_directory('bringup')
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
    assert 'fleet_manager' in {n['name'] for n in nodes}
    for n in nodes:
        if n['executable'] == 'tracker':
            p = n['params']
            assert isinstance(p['thrust_ratio'], float) and p['thrust_ratio'] == kt
            assert p['takeoff_thrust_ratio'] == 0.0            # ground takeoff: = thrust_ratio
            assert p['pose_timeout_s'] == 0.25 and p['safety_ref_timeout_s'] == 1.0
            assert p['payload_rest_z'] == 0.05
    return g


# ── M1 / attach rungs: real_control_launch.py mode:=attach ─────────────────────────────────────────

def test_m1_real_graph_partner_demo():
    # the sim config's loose Gazebo watchdogs are refused on the rig, so they are dropped
    args = {k: v for k, v in PARTNER_ATTACHED_ORBIT.items()
            if k not in ('pose_timeout_s', 'safety_ref_timeout_s')}
    nodes = rig('attach', thrust_ratio=24.0, **args)
    g = _assert_rig_graph(nodes, n_radio=4, kt=24.0)
    assert g['/fleet_manager']['params']['num_drones'] == 4
    assert g['/dissipative_planner']['params']['z_taut_gate'] == 0.9
    # M1's welded start: every magnet ON from boot, drone 3's included (auto)
    assert _latches(g, 4) == ['ON', 'ON', 'ON', 'ON']
    mux = g['/elrs_mux_3']['params']
    assert mux['magnet_channel'] == 6 and mux['magnet_command_topic'] == ''
    assert mux['handoff_topic'] == '/join_planner/handoff_ready'
    mgr = g['/magnet_attachment_manager']['params']
    assert mgr['magnet_latch_topic'] == '/drone_3/magnet'
    assert mgr['enable_elrs_magnet_output'] is False
    assert g['/dissipative_planner']['params']['detach_magnet'] is True
    assert mgr['magnet_tip_pose_topic'] == '/magnet_tip_pose'
    assert mgr['magnet_tip_pose_msg_type'] == 'pose_stamped'
    assert mgr['object_pose_topic'] == '/payload/motion_capture_state'
    assert mgr['object_pose_msg_type'] == 'mocap_state'
    assert mgr['attach_speed_threshold'] == 0.05 and mgr['attach_dwell_time_s'] == 0.15
    assert mgr['velocity_clock'] == 'sim'
    assert mgr['command_backend'] == 'none' and mgr['attached_at_start'] is True
    assert '/approach_mpc_3' not in g and '/online_join_planner' not in g   # partner flies it
    assert g['/tracker_3']['params']['thrust_ratio'] == 24.0


def test_m1_real_graph_our_approach_rung():
    """R8a/R8b shape: our tracker flies the newcomer, no partner, no approach MPC."""
    nodes = rig('attach', thrust_ratio=22.5,
                     enable_approach_mpc=False, weld_radius=0.05, start_taut=False)
    g = _assert_rig_graph(nodes, n_radio=4, kt=22.5)
    assert '/online_join_planner' in g and '/attach_target_publisher' in g
    assert g['/elrs_mux_3']['params']['approach_stream'] is False
    mgr = g['/magnet_attachment_manager']['params']
    assert mgr['attach_radius'] == 0.05 and mgr['attach_speed_threshold'] == 0.05
    assert g['/dissipative_planner']['params']['attach_approach'] is True


def test_m1_real_graph_without_the_approach_chain():
    nodes = rig('attach', thrust_ratio=24,
                     enable_approach=False)
    g = _assert_rig_graph(nodes, n_radio=3, kt=24.0)
    assert '/elrs_mux_3' not in g and '/magnet_attachment_manager' not in g


def test_m1_real_graph_explicit_serials_and_no_rig_io():
    nodes = rig('attach', thrust_ratio=24,
                     enable_approach_mpc=False, drone2_serial='/dev/ttyUSB7')
    assert _by_name(nodes)['/drone_2/elrs_interface']['params']['serial_port'] == '/dev/ttyUSB7'
    nodes = rig('attach', thrust_ratio=24,
                     enable_approach_mpc=False, real_io=False)
    assert not [n for n in nodes if n['executable'] in (
        'elrs_interface', 'motion_capture_publisher_node')]


@pytest.mark.parametrize('mode', ['attach', 'm2'])
@pytest.mark.parametrize('kt', ['auto', 'twentyfour', ''])
def test_real_refuses_a_derived_or_missing_kt(mode, kt):
    with pytest.raises(RuntimeError, match='thrust_ratio'):
        rig(mode, thrust_ratio=kt)


@pytest.mark.parametrize('mode', ['attach', 'm2'])
def test_real_refuses_looser_watchdogs(mode):
    with pytest.raises(RuntimeError, match='pose_timeout_s'):
        rig(mode, thrust_ratio=24, pose_timeout_s=0.3)
    with pytest.raises(RuntimeError, match='safety_ref_timeout_s'):
        rig(mode, thrust_ratio=24, safety_ref_timeout_s=3.0)
    # tighter than the rig is the operator's call
    nodes = rig(mode, thrust_ratio=24, pose_timeout_s=0.2, enable_approach_mpc=False)
    assert {n['params']['pose_timeout_s'] for n in nodes if n['executable'] == 'tracker'} \
        == {0.2}


def test_real_and_sil_are_exclusive():
    with pytest.raises(RuntimeError, match='sil'):
        rig('attach', sil=True, thrust_ratio=24)


def test_real_refuses_the_approach_mpc():
    # unflown on hardware, unstable on the linear plant, armed by /magnet/command ON
    with pytest.raises(RuntimeError, match='enable_approach_mpc'):
        rig('attach', thrust_ratio=24)
    g = _by_name(rig('attach', thrust_ratio=24, partner=True))
    assert '/approach_mpc_3' not in g


def test_real_newcomer_must_be_drone_3():
    # num_drones 4 would put carrier 3 and the newcomer's mux on one radio
    with pytest.raises(RuntimeError, match='num_drones'):
        rig('attach', thrust_ratio=24,
                 enable_approach_mpc=False, num_drones=4)
    g = _by_name(rig('attach', thrust_ratio=24,
                          num_drones=4, enable_approach=False, reserved_attach=0))
    assert g['/drone_3/elrs_interface']['params']['magnet_initial'] == 'ON'
    assert '/elrs_mux_3' not in g


def _rviz_panel(nodes):
    cfg = [n for n in nodes if n['executable'] == 'rviz2'][0]['arguments'][1]
    with open(cfg) as fh:
        return fh.read()


def test_rig_panel_rows():
    # M1 needs ATTACH; M2 has no detach step, so no DETACH row
    m1 = rig('attach', thrust_ratio=24, enable_approach_mpc=False)
    assert 'ShowAttach: true' in _rviz_panel(m1)
    m2 = rig('m2', thrust_ratio=24, partner_m2=True)
    panel = _rviz_panel(m2)
    assert 'ShowDetach: false' in panel and 'ShowAttach: false' in panel


def test_magnet_channel_is_one_launch_arg():
    g = _by_name(rig('attach', thrust_ratio=24,
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
    g = _by_name(rig('attach', thrust_ratio=24.0, **args))
    assert all(m['magnet_command_topic'] == '' for m in _muxes(g)) and _muxes(g)
    mgr = g['/magnet_attachment_manager']['params']
    assert mgr['magnet_latch_topic'] == '/drone_3/magnet' and mgr['attached_at_start'] is True
    assert g['/dissipative_planner']['params']['detach_magnet'] is True
    assert g['/dissipative_planner']['params']['start_attached'] is True
    assert _latches(g, 4) == ['ON'] * 4


def test_p10_r8a_dry_approach_keeps_drone_3_off():
    g = _by_name(rig('attach', thrust_ratio=24.0, **R8A))
    assert _latches(g, 4) == ['ON', 'ON', 'ON', 'OFF']
    # weld blocked: the manager runs (tip distance in its state) but drives no radio
    mgr = g['/magnet_attachment_manager']['params']
    assert mgr['magnet_latch_topic'] == '' and mgr['enable_elrs_magnet_output'] is False
    assert mgr['attach_radius'] == 0.0 and mgr['attached_at_start'] is False
    assert all(m['magnet_command_topic'] == '' for m in _muxes(g))
    assert g['/dissipative_planner']['params']['detach_magnet'] is True
    assert g['/dissipative_planner']['params']['start_attached'] is False


def test_p10_r8a_refuses_a_magnet_that_could_catch_without_a_weld():
    with pytest.raises(RuntimeError, match='no weld'):
        rig('attach', thrust_ratio=24.0,
                 **dict(R8A, attach_magnet_initial='ON'))
    with pytest.raises(RuntimeError, match='no weld'):
        rig('attach', thrust_ratio=24.0,
                 **dict(R8A, drone3_magnet_initial='ON'))
    with pytest.raises(RuntimeError, match='partner_attached'):
        rig('attach', thrust_ratio=24.0, weld_radius=0.0,
                 partner=True, partner_attached=True)


def test_p10_r8b_newcomer_latch_is_the_managers():
    g = _by_name(rig('attach', thrust_ratio=24.0,
                          **dict(R8A, weld_radius=0.03)))
    assert _latches(g, 4) == ['ON', 'ON', 'ON', 'OFF']
    assert g['/magnet_attachment_manager']['params']['magnet_latch_topic'] == '/drone_3/magnet'


def test_p10_r8a_literal_no_approach_chain():
    """enable_approach:=false: drone 3 has no radio, no mux and no manager at all."""
    g = _by_name(rig('attach', thrust_ratio=24.0,
                          **dict(R8A, enable_approach=False)))
    assert _latches(g, 3) == ['ON'] * 3 and '/drone_3/elrs_interface' not in g
    assert not _muxes(g) and '/magnet_attachment_manager' not in g
    assert g['/dissipative_planner']['params']['detach_magnet'] is True


def test_p10_per_drone_latch_overrides_and_refusals():
    g = _by_name(rig('attach', thrust_ratio=24.0,
                          **dict(R8A, weld_radius=0.03, drone1_magnet_initial='off',
                                 attach_magnet_initial='ON')))
    assert _latches(g, 4) == ['ON', 'OFF', 'ON', 'ON']
    for bad in ('', 'maybe'):
        with pytest.raises(RuntimeError, match='ON or OFF'):
            rig('attach', thrust_ratio=24.0,
                     **dict(R8A, magnet_initial=bad))
        with pytest.raises(RuntimeError, match='ON or OFF'):
            rig('m2', thrust_ratio=24.0, partner_m2=True,
                     drone2_magnet_initial=bad)


def test_p10_m2_magnet_paths():
    args = dict(M2_DRIVER, pose_timeout_s=0.25, safety_ref_timeout_s=1.0)
    g = _by_name(rig('m2', thrust_ratio=24.0, **args))
    muxes = _muxes(g)
    assert len(muxes) == 4
    assert all(m['magnet_command_topic'] == '' and m['magnet_channel'] == 6 for m in muxes)
    assert _latches(g, 4) == ['ON'] * 4
    assert g['/dissipative_planner']['params']['detach_magnet'] is True
    assert '/magnet_attachment_manager' not in g
    for part in ('muxes', 'controllers'):
        g = _by_name(rig('m2', thrust_ratio=24.0,
                              **dict(args, partner_m2_part=part)))
        if part == 'muxes':
            assert all(m['magnet_command_topic'] == '' for m in _muxes(g)) and _muxes(g)
            assert '/dissipative_planner' not in g
        else:
            assert not _muxes(g)
            assert g['/dissipative_planner']['params']['detach_magnet'] is True


def test_p10_sim_graphs_carry_no_rig_magnet_params():
    for mode, args in (('attach', PARTNER_ATTACHED_ORBIT),
                       ('attach', dict(R8A, pose_timeout_s=1.0)),
                       ('dissipative', M2_DRIVER)):
        g = _by_name(sim(mode, **args))
        assert 'detach_magnet' not in g['/dissipative_planner']['params']
        assert all('magnet_command_topic' not in m for m in _muxes(g))
        if '/magnet_attachment_manager' in g:
            assert 'magnet_latch_topic' not in g['/magnet_attachment_manager']['params']


# ── M2: real_control_launch.py mode:=m2 (partner_m2) ──────────────────────────────────────────────

def test_m2_real_graph():
    args = dict(M2_DRIVER, pose_timeout_s=0.25, safety_ref_timeout_s=1.0)
    nodes = rig('m2', thrust_ratio=24.0, **args)
    g = _assert_rig_graph(nodes, n_radio=4, kt=24.0)
    for i in range(4):
        mux = g[f'/elrs_mux_{i}']['params']
        assert mux['magnet_channel'] == 6 and mux['handoff_topic'] == '/fleet/handover'
        assert ('/drone_%d/ELRSCommand' % i, '/drone_%d/ELRSCommand_diss' % i) in \
            g[f'/tracker_{i}']['remappings']
        assert g[f'/drone_{i}/elrs_interface']['params']['magnet_initial'] == 'ON'
    assert g['/fleet_manager']['params']['num_drones'] == 4
    assert '/payload_mocap' not in g


@pytest.mark.parametrize('mode', ['sim', 'sil', 'real'])
def test_m1_drone3_takes_commands_from_the_manager_only(mode):
    """One command path (Wesley 2026-09-29): drone 3's tracker is armed and sent TAKEOFF by
    the fleet manager like the carriers, never straight from the /fleet/command broadcast
    (which let it take off when the manager refused)."""
    args = {'num_drones': 3, 'sil': True} if mode == 'sil' else dict(PARTNER_ATTACHED_ORBIT)
    if mode == 'real':
        args.update(thrust_ratio=24.0, pose_timeout_s=0.25, safety_ref_timeout_s=1.0)
    g = _by_name(rig('attach', **args) if mode == 'real' else sim('attach', **args))
    assert not [r for r in g['/tracker_3']['remappings'] if r[1] == '/fleet/command']
    assert g['/fleet_manager']['params']['num_drones'] == 4


def test_sim_approach_mpc_takes_commands_from_the_manager_only():
    """The collaborator's approach MPC (sim attach demo) hears the manager's accepted TAKEOFF
    on /drone_3/command, which arms it; never the /fleet/command broadcast."""
    g = _by_name(sim('attach', num_drones=3))
    approach = [n for k, n in g.items() if n['executable'] != 'tracker'
                and ('drone_command', '/drone_3/command') in n['remappings']]
    assert len(approach) == 1 and approach[0]['params']['takeoff_implies_arm'] is True
    assert not [k for k, n in g.items() if ('drone_command', '/fleet/command') in n['remappings']
                or (k == '/tracker_3' and any(r[1] == '/fleet/command' for r in n['remappings']))]


def test_m1_manager_counts_only_launched_trackers():
    g = _by_name(sim('attach', num_drones=3, enable_approach=False))
    assert '/tracker_3' not in g
    assert g['/fleet_manager']['params']['num_drones'] == 3


@pytest.mark.parametrize('real', [False, True])
def test_m2_manager_feedback_stays_under_ours(real):
    """His MPC publishes a 2 Hz armed heartbeat on /drone_i/arming_state_feedback in M2
    (m2c_vehicle_controller.launch.py): our manager and trackers must stay on /ours, or his
    False before his ARM reaches our fault scoping and ARM gate."""
    g = _by_name(rig('m2', **dict(M2_DRIVER, thrust_ratio=24.0, pose_timeout_s=0.25,
                                  safety_ref_timeout_s=1.0)) if real
                 else sim('dissipative', **M2_DRIVER))
    for i in range(4):
        fb = (f'/drone_{i}/arming_state_feedback', f'/ours/drone_{i}/arming_state_feedback')
        assert fb in g['/fleet_manager']['remappings']
        assert fb in g[f'/tracker_{i}']['remappings']


def test_m2_real_graph_plain_carry():
    """mode:=m2 without partner_m2 (R3/R4 on the M2 planner): no muxes."""
    nodes = rig('m2', thrust_ratio=24.0, num_drones=3)
    g = _assert_rig_graph(nodes, n_radio=3, kt=24.0)
    assert not [k for k in g if 'elrs_mux' in k]
    assert not [n for n in nodes if n['remappings'] and n['executable'] == 'tracker']


# ── Q7: the rig defaults to the creep floor start and the OCP resize ──────────────────

Q7_KEYS = ('start_taut', 'handover_elev_deg', 'handover_settle_s', 'creep_vel', 'reconfig_mode')
Q7_RIG = (False, 45.0, 1.0, 0.2, 'ocp')   # settle 1.0 = the rig carry's (2.0 until 2026-10-03)
# rig mode -> the sim mode whose graph it flies, and that sim mode's defaults
Q7_SIM_MODE = {'attach': 'attach', 'm2': 'dissipative'}
Q7_SIM = {'attach': (True, 45.0, 0.75, 0.10, 'network'),
          'm2': (False, 45.0, 1.0, 0.2, 'network')}
# the minimal rig line of each mode (R8a shape for M1, the partner_m2 M2 line)
Q7_RIG_LINE = {'attach': dict(enable_approach_mpc=False, weld_radius=0.0),
               'm2': dict(partner_m2=True, num_drones=4)}


def _q7(nodes):
    p = _by_name(nodes)['/dissipative_planner']['params']
    return tuple(p[k] for k in Q7_KEYS)


@pytest.mark.parametrize('mode', sorted(Q7_SIM))
def test_q7_real_defaults_to_the_creep_floor_start_and_ocp(mode):
    nodes = rig(mode, thrust_ratio=24.0, **Q7_RIG_LINE[mode])
    assert _q7(nodes) == Q7_RIG


@pytest.mark.parametrize('mode', sorted(Q7_SIM))
def test_q7_real_keeps_typed_values(mode):
    typed = dict(start_taut=True, handover_elev_deg=65.0, handover_settle_s=0.5,
                 creep_vel=0.15, reconfig_mode='network')
    nodes = rig(mode, thrust_ratio=24.0, **Q7_RIG_LINE[mode], **typed)
    assert _q7(nodes) == tuple(typed[k] for k in Q7_KEYS)
    # a typed value equal to the sim default is typed too: kept, not swapped for the rig's
    as_sim = dict(zip(Q7_KEYS, Q7_SIM[mode]))
    nodes = rig(mode, thrust_ratio=24.0, **Q7_RIG_LINE[mode], **as_sim)
    assert _q7(nodes) == Q7_SIM[mode]


def test_q7_real_keeps_one_typed_value_and_defaults_the_rest():
    nodes = rig('attach', thrust_ratio=24.0,
                     **Q7_RIG_LINE['attach'], creep_vel=0.3)
    assert _q7(nodes) == (False, 45.0, 1.0, 0.3, 'ocp')


def test_q7_typed_watchdog_at_the_sim_default_is_refused():
    # typed 1.0 (three_attach's sim default) is the operator's value, so it is checked
    with pytest.raises(RuntimeError, match='pose_timeout_s'):
        rig('attach', thrust_ratio=24.0,
                 **Q7_RIG_LINE['attach'], pose_timeout_s=1.0)


@pytest.mark.parametrize('mode', sorted(Q7_SIM))
def test_q7_sim_defaults_unchanged(mode):
    assert _q7(sim(Q7_SIM_MODE[mode])) == Q7_SIM[mode]


# ── the sim graphs M1 and M2 fly ─────────────────────────────────────────

def test_sim_m1_graph_is_unchanged_in_shape():
    nodes = sim('attach', **PARTNER_ATTACHED_ORBIT)
    assert all(n['use_sim_time'] for n in nodes)
    assert not [n for n in nodes if n['executable'] in ('elrs_interface',
                                                        'motion_capture_publisher_node')]
    g = _by_name(nodes)
    assert '/magnet_tip_publisher' in g and '/payload_attach_bridge' in g
    mgr = g['/magnet_attachment_manager']['params']
    assert mgr['command_backend'] == 'ros_topic' and 'elrs_magnet_channel' not in mgr
    assert 'magnet_command_topic' not in g['/elrs_mux_3']['params']
    c = g['/tracker_0']['params']
    assert (c['pose_timeout_s'], c['safety_ref_timeout_s'],
            c['payload_rest_z']) == (3.0, 4.0, -0.1)


def test_sim_m2_graph_is_unchanged_in_shape():
    nodes = sim('dissipative', **M2_DRIVER)
    assert all(n['use_sim_time'] for n in nodes)
    g = _by_name(nodes)
    assert '/payload_mocap' in g
    assert not [n for n in nodes if n['executable'] in ('elrs_interface',
                                                        'motion_capture_publisher_node')]
    assert 'magnet_channel' not in g['/elrs_mux_0']['params']


# ── sim RViz = rig RViz (Wesley 2026-09-29: rehearse in sim what the rig shows) ───────

@pytest.mark.parametrize('sim_args, rig_args', [
    ({'num_drones': 3}, {'num_drones': 3}),
    ({'num_drones': 4, 'detach': True}, {'num_drones': 4, 'detach': True}),
    # the rig count includes the newcomer, the sim's does not
    ({'num_drones': 3, 'attach': True}, {'num_drones': 4, 'attach': True}),
])
def test_sim_rviz_config_equals_the_rig(sim_args, rig_args):
    sim = _rviz_panel(evaluate('sim_io_launch.py', **sim_args))
    rig = _rviz_panel(evaluate('real_io_launch.py', **rig_args))
    assert 'ShowMagnets: true' in sim
    assert sim == rig


def test_sim_magnet_twin_only_with_the_gui():
    g = _by_name(evaluate('sim_io_launch.py', num_drones=3, attach=True))
    assert g['/sim_magnet']['params'] == {'num_drones': 4, 'num_tethers': 3}
    bridge = g['/sim_magnet_bridge']
    assert sorted(a.split('@')[0] for a in bridge['arguments']) == sorted(
        [f'/drone_{i}/detach' for i in range(3)]
        + [f'/drone_{i}/detachable_joint_state' for i in range(3)])
    assert not any(a.startswith(f'/drone_{i}/attach') for a in bridge['arguments'] for i in range(4))
    assert set(bridge['remappings']) == {(f'/drone_{i}/detach', f'/drone_{i}/magnet_release')
                                         for i in range(3)}
    headless = {n['name'] for n in evaluate('sim_io_launch.py', num_drones=3, rviz=False)}
    assert headless == {'clock_bridge', 'clock_throttle', 'payload_pose_bridge', 'fleet_viz',
                        'sim_telemetry', *(f'drone_pose_bridge_{i}' for i in range(3)),
                        *(f'mocap_{i}' for i in range(3)), *(f'drone_model_{i}' for i in range(3))}


def test_rig_detach_launch_flies_the_rig_control_values():
    """The rig detach mode takes the rig carry's defaults (Wesley 2026-10-03): every
    parameter the two give a node is the same; the detach is an OCP resize with the magnet
    released."""
    c = _by_name(rig('mpc', num_drones=4))
    d = _by_name(rig('dissipative', num_drones=4))
    for a, b in (('/tracker_0', '/tracker_0'), ('/fleet_manager', '/fleet_manager'),
                 ('/mpc_planner', '/dissipative_planner')):
        pa, pb = c[a]['params'], d[b]['params']
        assert set(pa) <= set(pb), sorted(set(pa) - set(pb))
        assert {k: pb[k] for k in pa} == pa
    p = d['/dissipative_planner']['params']
    assert p['reconfig_mode'] == 'ocp' and p['detach_magnet'] is True


def test_int_mode_model_reaches_the_planner_only_where_the_fleet_cannot_resize():
    """Card 2026-10-04_z_int_model: passed only when set; refused on the rig, with kt_trim on and
    with a fleet that can attach or detach mid-flight."""
    knobs = dict(int_mode='model', int_k_xy=5.4, int_k_z=8.5, kt_trim=False)
    p = _by_name(sim('mpc', num_drones=3, **knobs))['/mpc_planner']['params']
    assert (p['int_mode'], p['int_k_xy'], p['int_k_z']) == ('model', 5.4, 8.5)
    assert 'int_mode' not in _by_name(sim('mpc', num_drones=3))['/mpc_planner']['params']
    sil_attach = _by_name(sim('attach', num_drones=3, reserved_attach=0, enable_approach=False,
                              **knobs))['/dissipative_planner']['params']
    assert sil_attach['int_mode'] == 'model'
    with pytest.raises(RuntimeError, match='attach or detach'):
        sim('attach', num_drones=3, **knobs)
    with pytest.raises(RuntimeError, match='attach or detach'):
        sim('dissipative', num_drones=4, **knobs)
    with pytest.raises(RuntimeError, match='kt_trim'):
        sim('mpc', num_drones=3, **{**knobs, 'kt_trim': True})
    with pytest.raises(RuntimeError, match='rig'):
        rig('mpc', num_drones=3, thrust_ratio=35.2, **knobs)

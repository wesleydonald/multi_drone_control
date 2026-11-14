"""4 -> 2 -> 4 (the contract of 13 Nov): the attach graph with partners_file:=<yaml>, two partners after a pair.

Off (partners_file empty, the default) every node, topic, parameter and launch output is the pristine tree's
(legacy_launch_dump.py over today's lines). On: one chain per partner (tracker, mux, magnet manager, coil gate, his
launch), drone 3 under today's names and drone 2 under the contract's (/drone_2/..., his /tejen_d2/...), the planner's
partner lists, one radio and one tip body per partner, every partner's record from the file and refused null on the rig.
The launches are evaluated offline; his launch is only recorded (its include arguments), not evaluated.
"""
import json
import os
import subprocess
import sys

import pytest
pytest.importorskip('launch_ros')

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import legacy_launch_dump as dump  # noqa: E402

LAUNCH_DIR = os.path.join(os.path.dirname(HERE), 'launch')
TREE = os.path.dirname(os.path.dirname(os.path.dirname(HERE)))
PARTNERS_424 = os.path.join(TREE, 'configs', 'rig', 'partners_424.yaml')
# the pristine copy the legacy output is compared with (the 4 -> 2 -> 4 build's base tree)
BASE = os.environ.get('MDC_LEGACY_BASE', '/home/wesley/mdc_424/base')

# QUAD4's record as rig_nr8.sh / rig_nrtf.sh of 13 Nov type it (W, X, Y, Q = L - 0.04, K, P = L - 0.003; L 1.034)
QUAD4 = {'drone': 3, 'plate': 9, 'tip_body': 24, 'weld_point_z': 0.0491, 'attach_x_offset': 0.0,
         'attach_y_offset': 0.2293, 'attach_cable_len': 0.994, 'weld_radius': 0.025, 'proof_taut_len': 1.031}
# a measured-looking QUAD3 record on plate 3 (-y) for the tests
QUAD3 = {'drone': 2, 'plate': 3, 'tip_body': 23, 'weld_point_z': 0.048, 'attach_x_offset': 0.002,
         'attach_y_offset': -0.229, 'attach_cable_len': 0.99, 'weld_radius': 0.025, 'proof_taut_len': 1.027}
# his arguments as rig_424.sh types them for both instances (before each one's thrust ratio and plate)
HIS = ('mpc_cable_length:=1.0 trim_speed_gate:=0.20 carry_clearance:=0.25 carrier_azimuths_deg:=[0.0,180.0] '
       'commit_backdate:=0.04 tracking_check:=off attach_cpp_transit:=false attach_transit_clearance:=0.30 '
       'minimum_search_z_m:=2.20 approach_offset_z:=0.30 attach_twist:=zero match_xy:=0.05 match_dwell:=0.5 '
       'climb_speed:=0.3 match_speed:=0.20 descent_abort_speed:=0.35 descent_hold:=true geometry:=rig_m2a '
       'mission_mode:=floor_join descent_debounce:=1.0 descent_abort_xy:=0.05 contact_give_up_s:=120 contact_dz:=-0.03')
# the 4 -> 2 -> 4 line as rig_424.sh types it (builder O), with the partners file of the test
LINE = {'mode': 'attach', 'num_drones': 2, 'attach_azimuths_deg': '0,180', 'target_z': 1.0, 'throttle_max': 0.95,
        'enable_approach_mpc': False, 'partner': True, 'partner_contact': True, 'partner_nodes': True,
        'magnet_operator_topic': '/drone_3/magnet_operator', 'partner_attached': True, 'detach_unload_s': 3.0,
        'load_traj': 'hover', 'cable_len': 1.0, 'tip_skip_repeats': True,
        'partner_args': f'mpc_thrust_ratio:=22 plate:=9 {HIS}', 'partner_args_d2': f'mpc_thrust_ratio:=21 plate:=3 {HIS}'}


@pytest.fixture(autouse=True)
def _installed():
    try:
        from ament_index_python.packages import get_package_share_directory
        get_package_share_directory('bringup')
        get_package_share_directory('drone_visualisation')
        get_package_share_directory('tejen_mission')
    except Exception:
        pytest.skip('source install/setup.bash')


@pytest.fixture
def dds(monkeypatch):
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.setenv('ROS_DOMAIN_ID', '77')
    monkeypatch.delenv('MDC_PLANNER_CPUS', raising=False)


def _file(tmp_path, *records, name='partners.yaml', **top):
    import yaml
    path = tmp_path / name
    path.write_text(yaml.safe_dump({'partners': [dict(r) for r in records], **top}))
    return str(path)


def _run(launch_file, **args):
    return dump.evaluate(LAUNCH_DIR, launch_file, args, strict=True)


def rig(tmp_path, *records, **args):
    path = _file(tmp_path, *(records or (QUAD4, QUAD3)))
    return _run('real_control_launch.py', **{**LINE, 'partners_file': path, **args})


def twin(tmp_path, *records, **args):
    path = _file(tmp_path, *(records or (QUAD3, QUAD4)))
    base = {'mode': 'attach', 'legacy': False, 'num_drones': 2, 'attach_azimuths_deg': '0,180', 'partner': True,
            'partner_contact': True, 'partner_attached': True, 'partners_file': path}
    return _run('sim_control_launch.py', **{**base, **args})


def _nodes(out):
    return {e['node']: e for e in out if 'node' in e}


def _includes(out):
    return [(dict(e['arguments'])) for e in out if e.get('include') == 'partner_mission.launch.py']


def _publishes(n):
    """The topics a node of these graphs publishes (from its parameters, remappings and bridge arguments)."""
    p, ex, remap = n['params'], n['executable'], dict(map(tuple, n['remappings']))
    if ex == 'elrs_mux':
        return [f'/drone_{p["drone_id"]}/ELRSCommand', f'/drone_{p["drone_id"]}/mux_state']
    if ex == 'magnet_attachment_manager':
        out = [p.get('object_attached_topic', '/magnet/object_attached'),
               p.get('attachment_state_topic', '/magnet/attachment_state'),
               p.get('elrs_magnet_command_topic', '/magnet/ELRSCommand'),
               p.get('ros_attach_topic', '/payload/attach'), p.get('ros_detach_topic', '/payload/detach')]
        out += [p['magnet_latch_topic']] if p.get('magnet_latch_topic') else []
        if p.get('confirm_mode') == 'contact_lift':
            out += [p.get('contact_topic', '/magnet/contact'), p.get('proof_state_topic', '/magnet/attach_proof')]
        return out
    if ex == 'magnet_gate':
        return [p['out_topic'], p.get('state_topic', f'/drone_{p["drone_id"]}/magnet_gate')]
    if ex == 'tracker':
        topic = f'/drone_{p["drone_id"]}/ELRSCommand'
        return [remap.get(topic, topic)]
    if ex == 'magnet_tip_publisher':
        return [p['magnet_tip_topic']]
    if ex == 'payload_mocap_emulator':
        return [f'/drone_{p["drone_id"]}/motion_capture_state']
    if ex == 'parameter_bridge':
        # a gz-to-ROS bridge publishes on ROS ('['); a ROS-to-gz one (']') forwards what a node publishes
        return [remap.get(a.split('@')[0], a.split('@')[0]) for a in n['arguments'] if '@' in a and '[' in a]
    if ex == 'motion_capture_publisher_node':
        out = ['/magnet_tip_pose'] if p.get('magnet_tip_body_id') else []
        return out + [t.split(':', 1)[1] for t in str(p.get('tip_bodies', '')).split(',') if t]
    return []


def _writers(g):
    out = {}
    for name, n in g.items():
        for t in _publishes(n):
            out.setdefault(t, []).append(name)
    return out


# ── off: today's output, unchanged ──────────────────────────────────────────────────────────────────────

def _tree_dump(tree):
    env = dict(os.environ)
    pkgs = [os.path.join(tree, 'src', p) for p in sorted(os.listdir(os.path.join(tree, 'src')))
            if os.path.isdir(os.path.join(tree, 'src', p, p))]
    env['PYTHONPATH'] = os.pathsep.join(pkgs + [env.get('PYTHONPATH', '')])
    env['PYTHONDONTWRITEBYTECODE'] = '1'
    env.pop('MDC_PLANNER_CPUS', None)
    res = subprocess.run([sys.executable, os.path.join(HERE, 'legacy_launch_dump.py'), tree,
                          os.path.join(BASE, 'configs')], capture_output=True, text=True, env=env, timeout=600)
    assert res.returncode == 0, res.stderr[-2000:]
    return json.loads(res.stdout)


@pytest.mark.skipif(not os.path.isdir(os.path.join(BASE, 'src', 'bringup')), reason='no pristine tree to compare')
def test_without_a_partners_file_every_launch_output_is_the_pristine_trees():
    """Today's lines (the 13 Nov rig scripts, the twin, legacy and SIL attach, the rig I/O, the refusals): every node,
    parameter, remapping, argument, log line, timer, include argument and RViz config as the tree before 4 -> 2 -> 4."""
    before, now = _tree_dump(BASE), _tree_dump(TREE)
    assert set(before) == set(now) and len(now) >= 25
    for case in before:
        assert now[case] == before[case], case
    assert sum('node' in e for e in now['rig nrtf']) > 15 and any('error' in e for e in now['rig nrtf no weld point'])


# ── the names ───────────────────────────────────────────────────────────────────────────────────────────

def test_the_contracts_topic_rule():
    from bringup.graph_attach import his_args_name, his_topic, ours_topic, partner_node_name
    for t in ('/magnet/object_attached', '/partner/release', '/drone_3/take_back', '/magnet_tip_pose', ''):
        assert ours_topic(t, 3) == t
    assert ours_topic('/magnet/object_attached', 2) == '/drone_2/magnet/object_attached'
    assert ours_topic('/partner/release', 2) == '/drone_2/partner/release'
    assert ours_topic('/drone_3/magnet_operator', 2) == '/drone_2/magnet_operator'
    assert ours_topic('/magnet_tip_pose', 2) == '/drone_2/magnet_tip_pose'
    assert ours_topic('', 2) == ''
    for t in ('/join_planner/phase', '/tejen/magnet/command'):
        assert his_topic(t, 3) == t
    assert his_topic('/join_planner/phase', 2) == '/tejen_d2/join_planner/phase'
    assert his_topic('/tejen/magnet/command', 2) == '/tejen_d2/magnet/command'
    assert his_topic('/tejen/object_attached', 2) == '/tejen_d2/object_attached'
    assert partner_node_name('magnet_attachment_manager', 3) == 'magnet_attachment_manager'
    assert partner_node_name('magnet_attachment_manager', 2) == 'magnet_attachment_manager_2'
    assert (his_args_name(3), his_args_name(2)) == ('partner_args', 'partner_args_d2')


# ── the file ────────────────────────────────────────────────────────────────────────────────────────────

def test_the_trees_partners_file_holds_quad4s_record_and_quad3_to_measure():
    from bringup.graph_attach import load_partners
    recs, opts = load_partners(PARTNERS_424)
    assert opts == {} and [r['drone'] for r in recs] == [2, 3]
    assert recs[1] == QUAD4
    assert recs[0]['plate'] == 3 and recs[0]['tip_body'] == 23
    assert all(recs[0][k] is None for k in ('weld_point_z', 'attach_x_offset', 'attach_y_offset', 'attach_cable_len',
                                            'weld_radius', 'proof_taut_len'))


def test_the_rig_refuses_a_null_calibration(dds, tmp_path):
    with pytest.raises(RuntimeError, match=r'drone 2 \(QUAD3, plate 3\) has weld_point_z, attach_x_offset.*null: '
                                           r'measure it on the rig first'):
        _run('real_control_launch.py', **{**LINE, 'partners_file': PARTNERS_424})
    with pytest.raises(RuntimeError, match='has proof_taut_len null'):
        rig(tmp_path, QUAD4, dict(QUAD3, proof_taut_len=None))
    with pytest.raises(RuntimeError, match='has tip_body null'):
        rig(tmp_path, dict(QUAD4, tip_body=None), QUAD3)


@pytest.mark.parametrize('records,match', [
    ([{k: v for k, v in QUAD4.items() if k != 'weld_radius'}], 'missing weld_radius'),
    ([dict(QUAD4, weld_radiuss=0.02)], 'unknown field'),
    ([dict(QUAD4, weld_point_z=True)], 'not a number or null'),
    ([dict(QUAD4, plate=12)], 'not a plate number'),
    ([QUAD4, dict(QUAD3, plate=9)], 'share a plate'),
    ([QUAD4, dict(QUAD3, tip_body=24)], 'share a tip_body'),
    ([QUAD4, dict(QUAD3, drone=3)], 'share a drone'),
])
def test_the_file_refuses_a_bad_record(tmp_path, records, match):
    from bringup.graph_attach import load_partners
    with pytest.raises(RuntimeError, match=match):
        load_partners(_file(tmp_path, *records))
    with pytest.raises(RuntimeError, match='partners:'):
        load_partners(_file(tmp_path))


# ── on, the rig ─────────────────────────────────────────────────────────────────────────────────────────

def test_two_partners_on_the_rig_each_have_their_own_chain_on_the_contracts_topics(dds, tmp_path):
    out = rig(tmp_path)
    g = _nodes(out)
    for base in ('/tracker_', '/elrs_mux_', '/magnet_gate_'):
        assert {f'{base}2', f'{base}3'} <= set(g) and f'{base}4' not in g
    assert {'/magnet_attachment_manager', '/magnet_attachment_manager_2'} <= set(g)
    assert sum(n['executable'] == 'magnet_attachment_manager' for n in g.values()) == 2
    assert sum(n['executable'] == 'tracker' for n in g.values()) == 4
    assert len(g) == sum('node' in e for e in out), 'two nodes share a name'
    # drone 3 keeps every name it has today
    m3, m2 = g['/elrs_mux_3']['params'], g['/elrs_mux_2']['params']
    assert 'attached_topic' not in m3 and m3['release_topic'] == '/partner/release'
    assert m3['take_back_topic'] == '/drone_3/take_back' and m3['start_ours'] is True
    assert (m2['drone_id'], m2['attached_topic'], m2['release_topic'], m2['take_back_topic']) == (
        2, '/drone_2/magnet/object_attached', '/drone_2/partner/release', '/drone_2/take_back')
    assert m2['start_ours'] is True and m2['zero_partner_aux'] is True and m2['partner_hold_until_planner'] is True
    g3, g2 = g['/magnet_gate_3']['params'], g['/magnet_gate_2']['params']
    assert (g3['partner_topic'], g3['ours_topic'], g3['out_topic'], g3['phase_topic'], g3['puck_topic'],
            g3['release_topic'], g3['operator_topic']) == (
        '/tejen/magnet/command', '/drone_3/magnet_ours', '/drone_3/magnet_coil', '/join_planner/phase',
        '/tejen/object_attached', '/partner/release', '/drone_3/magnet_operator')
    assert (g2['drone_id'], g2['partner_topic'], g2['ours_topic'], g2['out_topic'], g2['phase_topic'],
            g2['puck_topic'], g2['release_topic'], g2['operator_topic'], g2['start_ours']) == (
        2, '/tejen_d2/magnet/command', '/drone_2/magnet_ours', '/drone_2/magnet_coil', '/tejen_d2/join_planner/phase',
        '/tejen_d2/object_attached', '/drone_2/partner/release', '/drone_2/magnet_operator', True)
    a3, a2 = g['/magnet_attachment_manager']['params'], g['/magnet_attachment_manager_2']['params']
    assert (a3['magnet_tip_pose_topic'], a3['object_attached_topic'], a3['magnet_latch_topic']) == (
        '/magnet_tip_pose', '/magnet/object_attached', '/drone_3/magnet_ours')
    assert 'contact_topic' not in a3 and 'body_pose_topic' not in a3          # today's defaults
    assert (a3['object_x_offset'], a3['object_y_offset'], a3['object_z_offset'], a3['attach_radius'],
            a3['proof_taut_len']) == (0.0, 0.2293, 0.0491, 0.025, 1.031)
    assert a3['attached_at_start'] is True and a2['attached_at_start'] is True
    assert (a2['magnet_tip_pose_topic'], a2['object_attached_topic'], a2['magnet_command_topic'],
            a2['magnet_latch_topic'], a2['body_pose_topic'], a2['contact_topic'], a2['proof_state_topic'],
            a2['attachment_state_topic'], a2['lift_done_topic'], a2['planner_phase_topic']) == (
        '/drone_2/magnet_tip_pose', '/drone_2/magnet/object_attached', '/drone_2/magnet/command', '/drone_2/magnet_ours',
        '/drone_2/motion_capture_state', '/drone_2/magnet/contact', '/drone_2/magnet/attach_proof',
        '/drone_2/magnet/attachment_state', '/tejen_d2/join_planner/proof_lift_complete', '/tejen_d2/join_planner/phase')
    assert (a2['object_x_offset'], a2['object_y_offset'], a2['object_z_offset'], a2['attach_radius'],
            a2['proof_taut_len'], a2['comotion_contact_xy']) == (0.002, -0.229, 0.048, 0.025, 1.027, 0.025)
    assert a2['comotion_taut_fallback'] == pytest.approx(1.030) and a3['comotion_taut_fallback'] == pytest.approx(1.034)
    t2 = g['/tracker_2']
    assert sorted(map(tuple, t2['remappings'])) == [('/drone_2/ELRSCommand', '/drone_2/ELRSCommand_diss'),
                                                    ('/drone_2/ELRSCommand_applied', '/drone_2/ELRSCommand')]
    assert t2['params']['mux_state_topic'] == '/drone_2/mux_state' and t2['params']['warm_follow_applied'] is True


def test_two_partners_on_the_rig_radios_mocap_fleet_and_planner(dds, tmp_path):
    g = _nodes(rig(tmp_path))
    for i in range(4):
        assert g[f'/drone_{i}/elrs_interface']['params']['serial_port'] == f'/dev/QUAD{i + 1}'
        assert g[f'/drone_{i}/elrs_interface']['params']['magnet_initial'] == 'ON'     # both partners start welded
    assert '/drone_4/elrs_interface' not in g
    assert [tuple(r) for r in g['/drone_2/elrs_interface']['remappings']] == [('magnet', '/drone_2/magnet_coil')]
    assert [tuple(r) for r in g['/drone_3/elrs_interface']['remappings']] == [('magnet', '/drone_3/magnet_coil')]
    assert g['/drone_0/elrs_interface']['remappings'] == [] and g['/drone_1/elrs_interface']['remappings'] == []
    assert 'HiddenMagnets: "2,3"' in g['/rviz2']['rviz_config']
    mocap = g['/motion_capture_publisher']['params']
    # QUAD4's tip as today; QUAD3's its own PoseStamped; the slope filter on the pair only, both partners on ema
    assert (mocap['magnet_tip_body_id'], mocap['tip_bodies'], mocap['vel_filter_drones']) == (
        24, '23:/drone_2/magnet_tip_pose', [0, 1])
    assert g['/fleet_manager']['params']['num_drones'] == 4
    p = g['/dissipative_planner']['params']
    assert (p['num_drones'], p['reserved_attach'], p['partner_drones'], p['partner_plates']) == (2, 2, [2, 3], [3, 9])
    assert p['partner_rods'] == [0.99, 0.994]
    assert (p['partner_weld_x'], p['partner_weld_y'], p['partner_weld_z']) == (
        [0.002, 0.0], [-0.229, 0.2293], [0.048, 0.0491])
    assert p['partner_set_detach'] is True and 'partner_serial_foldin' not in p
    # today's single values are drone 3's record
    assert (p['attach_cable_len'], p['attach_x_offset'], p['attach_y_offset'], p['weld_point_z']) == (
        0.994, 0.0, 0.2293, 0.0491)
    assert p['attach_azimuths_deg'] == '0,180' and p['start_attached'] is True and p['partner_release'] is True
    assert '/attach_target_publisher' not in g


def test_two_partners_on_the_rig_no_topic_has_two_writers(dds, tmp_path):
    g = _nodes(rig(tmp_path))
    writers = _writers(g)
    shared = {t: w for t, w in writers.items() if len(w) > 1}
    assert not shared, shared
    for t in ('/drone_2/magnet_coil', '/drone_3/magnet_coil'):
        assert writers[t] == [f'/magnet_gate_{t[7]}']
    assert writers['/drone_2/magnet/object_attached'] == ['/magnet_attachment_manager_2']
    assert writers['/magnet/object_attached'] == ['/magnet_attachment_manager']
    assert writers['/drone_2/magnet_tip_pose'] == ['/motion_capture_publisher']


def test_his_launch_once_per_partner_as_its_own_instance(dds, tmp_path):
    out = rig(tmp_path)
    inc = _includes(out)
    assert len(inc) == 2
    a, b = inc
    # drone 3: today's instance, no instance argument; partner_args as typed (its plate equal to the file's)
    assert 'instance' not in a and 'drone_id' not in a
    assert (a['plate'], a['contact_target_z'], a['contact_target_xy'], a['magnet_drop_below_quad'],
            a['takeoff_gate_topic'], a['mpc_thrust_ratio'], a['carrier_azimuths_deg']) == (
        '9', '0.0491', '0.0,0.2293', '1.0340', '/partner/release', '22', '[0.0,180.0]')
    assert (b['instance'], b['drone_id'], b['plate'], b['contact_target_z'], b['contact_target_xy'],
            b['magnet_drop_below_quad'], b['takeoff_gate_topic'], b['mpc_thrust_ratio']) == (
        'd2', '2', '3', '0.048', '0.002,-0.229', '1.0300', '/drone_2/partner/release', '21')
    for args in (a, b):
        assert (args['real'], args['contact_attach'], args['geometry'], args['contact_dz']) == (
            'true', 'true', 'rig_m2a', '-0.03')
    timers = [e['timer'] for e in out if 'timer' in e]
    assert timers == [5.0, 7.0]
    # untyped carriers: ours, clockwise as typed
    c = _includes(rig(tmp_path, partner_args='mpc_thrust_ratio:=22', partner_args_d2='mpc_thrust_ratio:=21'))
    assert c[0]['carrier_azimuths_deg'] == c[1]['carrier_azimuths_deg'] == '[0.0,180.0]'
    assert c[0]['plate'] == '9' and c[1]['plate'] == '3'
    assert not _includes(rig(tmp_path, partner_nodes=False))


@pytest.mark.parametrize('args,match', [
    ({'partner_args_d2': 'mpc_thrust_ratio:=21 plate:=9'}, 'partner_args_d2 types plate:=9 but the partners file has '
                                                           'drone 2 on plate 3'),
    ({'partner_args': 'mpc_thrust_ratio:=22 plate:=3'}, 'partner_args types plate:=3'),
    ({'partner_args_d3': 'mpc_thrust_ratio:=22'}, 'partner_args_d3'),
    ({'partner_args_d4': 'mpc_thrust_ratio:=22'}, 'partner_args_d4'),
    ({'partner_args_d2': 'instance:=d9'}, 'instance is set by this launch'),
    ({'partner_args_d2': 'contact_target_z:=0.1'}, 'contact_target_z'),
    ({'partner_args_d2': 'magnet_drop_below_quad:=0.9'}, 'magnet_drop_below_quad'),
    ({'weld_point_z': 0.05}, 'drop the typed weld_point_z'),
    ({'attach_cable_len': 0.99}, 'drop the typed attach_cable_len'),
    ({'mocap_magnet_tip_body_id': 24}, 'drop the typed mocap_magnet_tip_body_id'),
    ({'reattach_plate': 1}, 'reattach_plate'),
    ({'reserved_attach': 1}, 'reserved_attach'),
    ({'partner_tilt_cut': True}, 'partner_tilt_cut is wired to one partner'),
    ({'welded_on_load': True}, 'welded_on_load is wired to one partner'),
    ({'attach_pause': True}, 'attach_pause is wired to one partner'),
    ({'approach_tip_feedback': True}, 'approach_tip_feedback is wired to one partner'),
    ({'partner_rod_add_m': 0.01}, 'partner_rod_add_m is wired to one partner'),
    ({'take_back_avoid_pickup_m': 0.3}, 'take_back_avoid_pickup_m is wired to one partner'),
    ({'reconfig_mode': 'network'}, 'reconfig_mode ocp'),
    ({'partner_contact': False}, 'partner_contact'),
    ({'enable_approach': False}, 'enable_approach'),
    ({'num_drones': 3, 'attach_azimuths_deg': '0,120,240'}, r'numbers its 2 reserved drones \[3, 4\]'),
    ({'attach_azimuths_deg': '0,270'}, 'plate 9 holds a carrier'),
])
def test_the_two_partner_rig_line_refuses(dds, tmp_path, args, match):
    with pytest.raises(RuntimeError, match=match):
        rig(tmp_path, **args)


@pytest.mark.parametrize('change,match', [
    ({'attach_y_offset': 0.229}, 'sits on plate 9, its record says plate 3'),
    ({'attach_cable_len': 0.95}, 'disagrees with proof_taut_len'),
    ({'attach_cable_len': 0.8, 'proof_taut_len': 0.837}, 'a record from other cables'),
    ({'weld_point_z': 0.0}, 'ring-origin plane'),
    ({'weld_radius': 0.0}, 'contact radius'),
    ({'tip_body': 13}, "tip_body 13 is a drone's or the ring's Motive body"),
])
def test_each_record_has_quad4s_rig_checks(dds, tmp_path, change, match):
    with pytest.raises(RuntimeError, match=match):
        rig(tmp_path, QUAD4, dict(QUAD3, **change))


QUAD4_TYPED = {'weld_point_z': 0.0491, 'attach_x_offset': 0.0, 'attach_y_offset': 0.2293, 'attach_cable_len': 0.994,
               'weld_radius': 0.025, 'proof_taut_len': 1.031, 'mocap_magnet_tip_body_id': 24}


def test_set_detach_and_serial_fold_ins_from_the_file(dds, tmp_path):
    p = _nodes(rig(tmp_path, partner_rod_add_m=0.0))['/dissipative_planner']['params']
    assert p['partner_set_detach'] is True and 'partner_serial_foldin' not in p
    # fold-ins happen at once (Wesley 13 Nov): the old queue option is an unknown key now
    with pytest.raises(RuntimeError, match='unknown key'):
        _run('real_control_launch.py', **{**LINE, 'partners_file': _file(tmp_path, QUAD4, QUAD3, serial_foldin=True)})
    with pytest.raises(RuntimeError, match='set_detach'):
        _run('real_control_launch.py', **{**LINE, 'partners_file': _file(tmp_path, QUAD4, QUAD3, set_detach='yes')})


def test_a_typed_his_args_without_the_file_is_refused(dds):
    with pytest.raises(RuntimeError, match='partner_args_d2: a second instance'):
        _run('real_control_launch.py', **{**{k: v for k, v in LINE.items() if k != 'partner_args'},
                                          'num_drones': 3, 'attach_azimuths_deg': '90,330,210', **QUAD4_TYPED})


def test_a_one_partner_file_flies_todays_line(dds, tmp_path):
    """QUAD4 alone in the file after three carriers: the graph of today's typed line, but for the planner's partner
    lists and the log lines (his launch gets its plate and carriers from the launch, equal to the typed ones)."""
    his = 'mpc_thrust_ratio:=22 plate:=9 ' + HIS.replace('[0.0,180.0]', '[90.0,330.0,210.0]')
    today = {**{k: v for k, v in LINE.items() if k != 'partner_args_d2'}, 'num_drones': 3,
             'attach_azimuths_deg': '90,330,210', 'partner_args': his}
    old = _run('real_control_launch.py', **today, **QUAD4_TYPED)
    new = rig(tmp_path, QUAD4, **{**today, 'partner_args_d2': ''})
    go, gn = _nodes(old), _nodes(new)
    assert set(go) == set(gn)
    keys = {'partner_drones', 'partner_plates', 'partner_rods', 'partner_weld_x', 'partner_weld_y', 'partner_weld_z',
            'partner_set_detach'}
    for name in go:
        a, b = go[name], gn[name]
        if name == '/dissipative_planner':
            assert set(b['params']) - set(a['params']) == keys
            b = dict(b, params={k: v for k, v in b['params'].items() if k not in keys})
        assert a == b, name
    assert gn['/dissipative_planner']['params']['partner_set_detach'] is False   # one record: today's single DETACH
    assert _includes(old)[0] == _includes(new)[0]


# ── on, the twin ────────────────────────────────────────────────────────────────────────────────────────

def test_two_partners_in_the_twin(tmp_path):
    null = {k: None for k in QUAD4 if k not in ('drone', 'plate')}
    g = _nodes(twin(tmp_path, dict(QUAD3, **null), dict(QUAD4, **null)))
    for d in (2, 3):
        for base in ('motor_bridge_', 'imu_bridge_', 'pose_bridge_', 'mocap_', 'bf_comm_', 'elrs_mux_', 'tracker_'):
            assert f'/{base}{d}' in g
        assert g[f'/mocap_{d}']['params']['pose_topic'] == f'/model/x3_drone{d}/pose'
    for name in ('magnet_tip_publisher', 'payload_attach_bridge', 'payload_detach_bridge', 'payload_joint_state_bridge',
                 'magnet_attachment_manager'):
        assert f'/{name}' in g and f'/{name}_2' in g
    assert g['/payload_attach_bridge_2']['arguments'] == ['/drone_2/payload/attach@std_msgs/msg/Empty]gz.msgs.Empty']
    assert g['/payload_joint_state_bridge_2']['arguments'] == [
        '/drone_2/payload/detachable_joint_state@std_msgs/msg/String[gz.msgs.StringMsg']
    assert g['/payload_attach_bridge']['arguments'] == ['/payload/attach@std_msgs/msg/Empty]gz.msgs.Empty']
    tip2 = g['/magnet_tip_publisher_2']['params']
    assert (tip2['x3_pose_topic'], tip2['magnet_tip_topic']) == ('/model/x3_drone2/pose', '/drone_2/magnet_tip_pose')
    a2 = g['/magnet_attachment_manager_2']['params']
    # nulls in the twin: the launch's plate-9 point turned onto plate 3, the launch's radius and weld height
    assert (a2['object_x_offset'], a2['object_y_offset']) == (0.0, -0.225)
    assert (a2['ros_attach_topic'], a2['ros_detach_topic'], a2['object_attached_topic'], a2['sim_catch']) == (
        '/drone_2/payload/attach', '/drone_2/payload/detach', '/drone_2/magnet/object_attached', True)
    a3 = g['/magnet_attachment_manager']['params']
    assert (a3['object_x_offset'], a3['object_y_offset'], a3['ros_attach_topic']) == (0.0, 0.225, '/payload/attach')
    assert g['/fleet_manager']['params']['num_drones'] == 4
    p = g['/dissipative_planner']['params']
    assert (p['partner_drones'], p['partner_plates'], p['partner_weld_y']) == ([2, 3], [3, 9], [-0.225, 0.225])
    assert '/attach_target_publisher' not in g and not [n for n in g if 'tejen' in n]
    assert not {t: w for t, w in _writers(g).items() if len(w) > 1}
    bridged = [a.split('@')[0] for n in g.values() if n['executable'] == 'parameter_bridge' for a in n['arguments']]
    assert len(bridged) == len(set(bridged))


def test_the_twin_refuses_sil_and_the_legacy_worlds(tmp_path):
    with pytest.raises(RuntimeError, match='not sil'):
        twin(tmp_path, sil=True)
    with pytest.raises(RuntimeError, match='legacy worlds'):
        twin(tmp_path, legacy=True)


def test_sim_io_draws_every_partner(tmp_path):
    def count(**args):
        out = _run('sim_io_launch.py', num_drones=2, attach=True, rviz=False, **args)
        return sum(e.get('executable') == 'robot_state_publisher' for e in out), \
            next(e for e in out if e.get('node') == '/sim_telemetry')['params']['num_drones']
    assert count() == (3, 3)
    assert count(attach_partners=2) == (4, 4)


# ── on, with his launch evaluated: both of his instances beside our graph ──────────────────────────────

def _his_overlay(tmp_path, monkeypatch):
    """An ament prefix in tmp_path for his two packages: tejen_mission's share is this tree's source (its
    partner_mission.launch.py), and tejen_dynamic_planner's prefix sits under tmp_path, so the run directories his
    backend creates (<prefix>/../../logs/tejen_dynamic_planner) land there, never in a flown tree."""
    from ament_index_python.packages import get_package_share_directory
    dyn = get_package_share_directory('tejen_dynamic_planner')
    prefixes = []
    for pkg, share in (('tejen_mission', os.path.join(TREE, 'src', 'tejen_mission')), ('tejen_dynamic_planner', dyn)):
        prefix = tmp_path / 'ws' / 'install' / pkg
        (prefix / 'share' / 'ament_index' / 'resource_index' / 'packages').mkdir(parents=True)
        (prefix / 'share' / 'ament_index' / 'resource_index' / 'packages' / pkg).write_text('')
        os.symlink(share, prefix / 'share' / pkg)
        prefixes.append(str(prefix))
    monkeypatch.setenv('AMENT_PREFIX_PATH', os.pathsep.join(prefixes + [os.environ.get('AMENT_PREFIX_PATH', '')]))
    return tmp_path / 'ws'


def test_both_of_his_instances_beside_our_graph(dds, tmp_path, monkeypatch):
    """With his launch evaluated (this tree's partner_mission.launch.py): every node name once, instance A where it is
    today, instance B under /tejen_d2 on drone 2's radio stream and our drone-2 topics; his run directories only under
    tmp_path."""
    his = open(os.path.join(TREE, 'src', 'tejen_mission', 'launch', 'partner_mission.launch.py')).read()
    if "'instance'" not in his:
        pytest.skip('this tree\'s partner_mission.launch.py has no instance argument yet')
    ws = _his_overlay(tmp_path, monkeypatch)
    from ament_index_python.packages import get_package_prefix
    assert get_package_prefix('tejen_dynamic_planner').startswith(str(tmp_path))
    flown = os.path.expanduser('~/multi_drone_control/logs/tejen_dynamic_planner')
    before = set(os.listdir(flown)) if os.path.isdir(flown) else set()
    out = dump.evaluate(LAUNCH_DIR, 'real_control_launch.py', {**LINE, 'partners_file': _file(tmp_path, QUAD4, QUAD3)},
                        strict=True, follow=True)
    assert (set(os.listdir(flown)) if os.path.isdir(flown) else set()) == before
    names = ['/' + e['node'].lstrip('/') for e in out if 'node' in e]          # his namespaces come without a '/'
    assert len(names) == len(set(names)), sorted(n for n in names if names.count(n) > 1)
    g = {'/' + e['node'].lstrip('/'): e for e in out if 'node' in e}
    assert '/online_join_planner' in g and '/tejen/tejen_mpc' in g                      # instance A, as today
    assert '/tejen_d2/online_join_planner' in g and '/tejen_d2/tejen_mpc' in g          # instance B
    mpc = dict(map(tuple, g['/tejen_d2/tejen_mpc']['remappings']))
    assert mpc.get('ELRSCommand') == '/drone_2/ELRSCommand_tejen'
    assert dict(map(tuple, g['/tejen/tejen_mpc']['remappings'])).get('ELRSCommand') == '/drone_3/ELRSCommand_tejen'
    pb = g['/tejen_d2/online_join_planner']['params']
    for key, topic in (('take_back_topic', '/drone_2/take_back'), ('mux_state_topic', '/drone_2/mux_state'),
                       ('contact_topic', '/drone_2/magnet/contact'),
                       ('ring_attached_topic', '/drone_2/magnet/object_attached'),
                       ('magnet_command_topic', '/tejen_d2/magnet/command'),
                       ('takeoff_gate_topic', '/drone_2/partner/release'),
                       ('give_up_topic', '/tejen_d2/join_planner/give_up'),
                       ('phase_topic', '/tejen_d2/join_planner/phase')):
        assert pb[key] == topic, key
    rs = g['/tejen_d2/partner_rig_state']['params']
    assert (rs['body_topic'], rs['tip_topic']) == ('/drone_2/motion_capture_state', '/drone_2/magnet_tip_pose')
    # nothing of instance B on drone 3's or instance A's names
    b_topics = [v for n, e in g.items() if n.startswith('/tejen_d2/') for v in e['params'].values()
                if isinstance(v, str) and v.startswith('/')]
    assert not [t for t in b_topics if t.startswith('/drone_3/') or t.startswith('/tejen/')
                or t.startswith('/join_planner/') or t == '/magnet_tip_pose']
    assert (ws / 'logs' / 'tejen_dynamic_planner').is_dir()

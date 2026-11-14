"""partner_mission.launch.py (ours), two instances of his stack (4 -> 2 -> 4, mdc_424 CONTRACT.md).

Instance A (no instance typed) is today's QUAD4 instance: its launch dump must equal the pristine tree's. Instance B
(instance:=d2 drone_id:=2) runs his unchanged nodes in /tejen_d2 on its own names: every topic under /tejen_d2/ or
/drone_2/ (or the shared ring and object), none of A's, his YAML values present as parameters (a namespaced node
silently drops a file keyed by its bare name), every topic his code would otherwise default to an absolute name typed,
and his MPC following its own planner's reference."""
import ast
import importlib.util
import os
import re
import sys
import tempfile
from pathlib import Path

import pytest
import yaml
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters

PKG = Path(__file__).resolve().parents[1]
SRC = PKG.parent
LAUNCH = PKG / 'launch' / 'partner_mission.launch.py'
# the pristine copy of the tree the build started from (mdc_424); the identity test skips without it
BASE = Path(os.environ.get('MDC_424_BASE', '/home/wesley/mdc_424/base')) / 'src' / 'tejen_mission' / 'launch' \
    / 'partner_mission.launch.py'
SHARED = {'/payload/motion_capture_state', '/payload/trajectory_state', '/pickup/motion_capture_state'}
OWN = ('/tejen_d2/', '/drone_2/')

# today's rig lines (results/rig/2026-11-13/scripts/rig_nrtf.sh: the HIS args, our launch's adds), with the 4 -> 2 -> 4
# pair on plates 0/6 as the carriers that stay
NRTF = dict(real='true', contact_attach='true', contact_target_z='0.0491', contact_target_xy='0.0,0.2293',
            magnet_drop_below_quad='1.031', ring_z='1.0', ring_cable_len='1.0', mpc_thrust_ratio='22',
            mpc_cable_length='1.0', trim_speed_gate='0.20', carry_clearance='0.25', plate='9',
            carrier_azimuths_deg='[0.0,180.0]', commit_backdate='0.04', tracking_check='off',
            attach_cpp_transit='false', attach_transit_clearance='0.30', minimum_search_z_m='2.20',
            approach_offset_z='0.30', attach_twist='zero', match_xy='0.05', match_dwell='0.5', climb_speed='0.3',
            match_speed='0.20', descent_abort_speed='0.35', descent_hold='true', geometry='rig_m2a',
            mission_mode='floor_join', descent_debounce='1.0', descent_abort_xy='0.05', contact_give_up_s='120',
            contact_dz='-0.03', takeoff_gate_topic='/partner/release', record='true')
B = dict(NRTF, instance='d2', drone_id='2', plate='3', contact_target_xy='0.0,-0.2293',
         takeoff_gate_topic='/drone_2/partner/release')
# the twin's line for the same (configs/experiments/rig_nrtf_z12_twin.yaml's his args), QUAD4 and the carrier on plate 3
TWIN_A = dict(geometry='rig_twin_L100', plate='9', carrier_azimuths_deg='[0.0,180.0]', contact_attach='true',
              carry_clearance='0.25', confirm_pickup_release='true', start_gate_topic='/partner/release',
              mission_mode='floor_join', commit_backdate='0.04', tracking_check='off', attach_cpp_transit='false',
              attach_transit_clearance='0.30', minimum_search_z_m='2.20', approach_offset_z='0.30', attach_twist='zero',
              match_xy='0.08', match_dwell='0.5', climb_speed='0.3', match_speed='0.20', descent_abort_speed='0.35',
              descent_hold='true', descent_debounce='1.0', descent_abort_xy='0.10', contact_give_up_s='120',
              contact_dz='-0.03', ring_z='1.2', record='true')
TWIN_B = dict({k: v for k, v in TWIN_A.items() if k != 'confirm_pickup_release'}, instance='d2', drone_id='2',
              plate='3', start_gate_topic='/drone_2/partner/release')

# instance A's arguments as flown or typed today: the dump must not move (rig and twin paths)
RIG_MIN = dict(real='true', contact_attach='true', contact_target_z='0.031', magnet_drop_below_quad='0.575',
               mpc_thrust_ratio='22.0', ring_cable_len='0.55')
LEGACY_CASES = {
    'rig_min': RIG_MIN,
    'rig_nrtf_triangle': dict(NRTF, carrier_azimuths_deg='[90.0,330.0,210.0]'),
    'rig_nrtf_untyped_carriers': {k: v for k, v in NRTF.items() if k != 'carrier_azimuths_deg'},
    # the 13 Nov pickup line (rig_fm9h8.sh with its defaults): pickup, drop into the net, rejoin
    'rig_fm9h8': dict(RIG_MIN, geometry='rig_m2a', ring_cable_len='1.0', ring_z='1.0', contact_target_z='0.0491',
                      contact_target_xy='0.0,0.2293', magnet_drop_below_quad='1.031', mpc_cable_length='1.0',
                      trim_speed_gate='0.20', carry_clearance='0.25', plate='9',
                      carrier_azimuths_deg='[90.0,330.0,210.0]', commit_backdate='0.04', tracking_check='off',
                      attach_cpp_transit='false', attach_transit_clearance='0.30', minimum_search_z_m='2.20',
                      approach_offset_z='0.30', attach_twist='zero', match_xy='0.05', match_dwell='0.5',
                      climb_speed='0.3', match_speed='0.20', descent_abort_speed='0.35', descent_hold='true',
                      cpp_authority='false', pickup_xy='0.06', pickup_speed='0.20', pickup_contact_height='0.011',
                      pickup_object_thickness='0.050', pickup_object_size='0.105', pickup_point_x='-0.0045',
                      pickup_point_y='-0.0107', pickup_box='1.9', pickup_proof_drift='0.05', pickup_proof_speed='0.08',
                      pickup_proof_loss='0.08', drop_on_arrival_xy='0.08', lift_full='true', mpc_throttle_max='0.8',
                      drop_height='0.48', drop_release='0.03', drop_release_xy='0.06', drop_release_wait='3',
                      max_body_z='2.8', line_up='gap', mpc_battery_comp='0.022', mission_mode='delivery_rejoin',
                      descent_debounce='1.0', descent_abort_xy='0.05', contact_give_up_s='120', contact_dz='-0.03',
                      return_after_drop='direct', record='true'),
    'rig_column': dict(RIG_MIN, carrier_model='column', transit='low'),
    'sim_legacy': {},
    'sim_rig_twin_contact': dict(geometry='rig_twin_L100', plate='9', carrier_azimuths_deg='[90.0,330.0,210.0]',
                                 contact_attach='true', mission_mode='floor_join', start_gate_topic='/partner/release',
                                 record='true', confirm_pickup_release='true', transit='low'),
    'sim_plate_cpp_reattach': dict(geometry='rig_twin_L100_plate', pickup_cpp_reattach='true',
                                   commit_backdate='0.04', swing_envelope='outer_octagon', cpp_splice='0.3',
                                   temporal_refinement='true', workspace_xy='1.8', workspace_z_max='3.5'),
}


def _module(path=LAUNCH):
    spec = importlib.util.spec_from_file_location(f'partner_mission_{abs(hash(str(path)))}', path)
    mod = importlib.util.module_from_spec(spec)
    keep, sys.dont_write_bytecode = sys.dont_write_bytecode, True     # no __pycache__ in the pristine tree
    try:
        spec.loader.exec_module(mod)
    finally:
        sys.dont_write_bytecode = keep
    return mod


def _run(mod, typed):
    ctx = LaunchContext()
    ctx.launch_configurations.update(typed)
    mod._record_typed(ctx)
    for a in mod._arguments():
        if isinstance(a, DeclareLaunchArgument) and a.name not in ctx.launch_configurations:
            ctx.launch_configurations[a.name] = a.default_value[0].text if a.default_value else ''
    return ctx, mod._nodes(ctx)


def _s(ctx, x):
    if x is None or isinstance(x, str):
        return x
    return ''.join(t.perform(ctx) for t in x)


def _location(ctx, include):
    # .location is the substitutions' repr until the include has run
    return _s(ctx, include.launch_description_source._LaunchDescriptionSource__location)


def _plain(v):
    return [_plain(x) for x in v] if isinstance(v, (list, tuple)) else v


def _file(path):
    """A parameter file as its content when it is one of the launch's temporary files (a new name every run)."""
    path = str(path)
    if Path(path).name.startswith('partner_backend_') and path.startswith(tempfile.gettempdir()):
        return ('tempfile', yaml.safe_load(Path(path).read_text()))
    return ('file', path)


def _entity(ctx, e):
    if isinstance(e, Node):
        params = [('dict', {k: _plain(v) for k, v in p.items()}) if isinstance(p, dict) else _file(p)
                  for p in evaluate_parameters(ctx, e._Node__parameters or [])]
        return ('node', _s(ctx, e._Node__package), _s(ctx, e._Node__node_executable), e._Node__node_name,
                e._Node__node_namespace, params, [(_s(ctx, a), _s(ctx, b)) for a, b in e._Node__remappings or []])
    if isinstance(e, IncludeLaunchDescription):
        args = {_s(ctx, k): _s(ctx, v) for k, v in e.launch_arguments}
        if 'extra_params_file' in args:
            args['extra_params_file'] = [_file(p) for p in args['extra_params_file'].split(',') if p]
        return ('include', _location(ctx, e), args)
    if isinstance(e, ExecuteProcess):
        return ('process', [_s(ctx, c) for c in e.cmd], e.condition.evaluate(ctx) if e.condition else None)
    return (type(e).__name__,)


def _dump(path, typed):
    ctx, ents = _run(_module(path), typed)
    return [_entity(ctx, e) for e in ents]


def _graph(typed):
    """{node name: {'params': one merged dict, 'files': [file entries], 'ns', 'remaps'}} plus the raw entities."""
    ctx, ents = _run(_module(), typed)
    out = {}
    for e in ents:
        if not isinstance(e, Node):
            continue
        merged, files = {}, []
        for p in evaluate_parameters(ctx, e._Node__parameters or []):
            if isinstance(p, dict):
                merged.update({k: _plain(v) for k, v in p.items()})
            else:
                files.append(str(p))
        out[e._Node__node_name] = {'params': merged, 'files': files, 'ns': e._Node__node_namespace,
                                   'remaps': dict((_s(ctx, a), _s(ctx, b)) for a, b in e._Node__remappings or [])}
    return out, ctx, ents


def _block(path, node):
    return dict(yaml.safe_load(Path(path).read_text())[node]['ros__parameters'])


def _effective_a(node, typed):
    """Instance A's parameters as his nodes end up with them: files (keyed by the root node's name) and dicts in order;
    his backend as his c1f6 launch builds it from the include's arguments."""
    mod = _module()
    g, ctx, ents = _graph(typed)
    if node == mod.BACKEND_NODE:
        inc = next(e for e in ents if isinstance(e, IncludeLaunchDescription))
        args = {_s(ctx, k): _s(ctx, v) for k, v in inc.launch_arguments}
        dyn = Path(_location(ctx, inc)).parents[1]
        out = _block(dyn / 'config' / 'c1f6_moving_rendezvous.yaml', node)
        for f in args['extra_params_file'].split(','):
            out.update(_block(f, node))
        n = int(args['cooperative_preview_count'])
        out.update({'planner_rate_hz': float(args['planner_rate_hz']),
                    'minimum_search_z_m': float(args['minimum_search_z_m']),
                    'cooperative_drone_state_topics': [f'/fake_obstacles/drone_{i}/state' for i in range(n)],
                    'cooperative_committed_trajectory_topics': [
                        f'/fake_obstacles/drone_{i}/committed_trajectory' for i in range(n)],
                    'use_sim_time': args['use_sim_time'] == 'true', 'state_topic': args['state_topic'],
                    'object_attached_topic': args['object_attached_topic']})
        return out
    e = next(e for e in ents if isinstance(e, Node) and e._Node__node_name == node)
    out = {}
    for p in evaluate_parameters(ctx, e._Node__parameters or []):
        out.update({k: _plain(v) for k, v in p.items()} if isinstance(p, dict) else _block(p, node))
    return out


def _topics(params):
    """Every absolute topic name in a parameter dict (topic keys, string or list)."""
    out = set()
    for k, v in params.items():
        if k.endswith(('_topic', '_topics', '_topic_prefix')):
            for t in (v if isinstance(v, list) else [v]):
                if isinstance(t, str) and t.startswith('/'):
                    out.add(t)
    return out


def _bag(ents, ctx):
    cmd = next(e for e in ents if isinstance(e, ExecuteProcess) and not isinstance(e, Node)
               and 'bag' in str([_s(ctx, c) for c in e.cmd]))
    words = [_s(ctx, c) for c in cmd.cmd]
    return words[words.index('-o') + 1], {w for w in words[words.index('-o') + 2:] if not w.startswith('--')}


# ---- instance A: today's dump, untouched ----------------------------------------------------------------------------

@pytest.mark.skipif(not BASE.exists(), reason=f'no pristine tree at {BASE} (MDC_424_BASE)')
@pytest.mark.parametrize('case', sorted(LEGACY_CASES))
def test_instance_a_is_todays_launch(case):
    assert _dump(LAUNCH, LEGACY_CASES[case]) == _dump(BASE, LEGACY_CASES[case])


def test_instance_a_keeps_todays_names():
    g, ctx, ents = _graph(dict(NRTF, carrier_azimuths_deg='[90.0,330.0,210.0]'))
    assert {n: v['ns'] for n, v in g.items()} == {
        'fake_cooperative_transport_world': None, 'partner_rig_state': 'tejen', 'magnet_attachment_manager_irl': 'tejen',
        'online_join_planner': None, 'tejen_mpc': 'tejen'}
    p, mpc = g['online_join_planner']['params'], g['tejen_mpc']['params']
    assert 'reference_topic' not in p and p['vehicle_id'] == 'drone_0' and p['takeoff_gate_topic'] == '/partner/release'
    assert not {'external_reference_topic', 'payload_mpc_mode_topic', 'logging_instance'} & set(mpc)
    assert g['online_join_planner']['files'] and not g['online_join_planner']['remaps']
    inc = [e for e in ents if isinstance(e, IncludeLaunchDescription)]
    args = {_s(ctx, k): _s(ctx, v) for k, v in inc[0].launch_arguments}
    assert (len(inc), args['cooperative_preview_count'], args['state_topic'], args['object_attached_topic']) == (
        1, '3', '/drone_3/motion_capture_state', '/tejen/object_attached')
    assert g['partner_rig_state']['params']['tip_topic'] == '/magnet_tip_pose'


# ---- instance B: its own names ---------------------------------------------------------------------------------------

def test_instance_b_runs_every_node_in_its_namespace_from_dicts():
    g, ctx, ents = _graph(B)
    assert {n: v['ns'] for n, v in g.items()} == {n: 'tejen_d2' for n in (
        'fake_cooperative_transport_world', 'partner_rig_state', 'magnet_attachment_manager_irl', 'online_join_planner',
        'dynamic_planner_transfer_backend', 'tejen_mpc')}
    assert not [n for n, v in g.items() if v['files']]                # no file keyed by a bare node name
    assert not [e for e in ents if isinstance(e, IncludeLaunchDescription)]      # his backend built here
    # its run directory is made before its backend starts
    kinds = [type(e).__name__ if not isinstance(e, Node) else e._Node__node_name for e in ents]
    assert kinds.index('OpaqueFunction') < kinds.index('dynamic_planner_transfer_backend')


def test_instance_b_every_topic_is_its_own_or_shared():
    g, ctx, ents = _graph(B)
    bad = []
    for name, n in g.items():
        for t in _topics(n['params']) | set(n['remaps'].values()):
            if t.startswith('/') and t not in SHARED and not t.startswith(OWN):
                bad.append((name, t))
    _, bag = _bag(ents, ctx)
    bad += [('bag', t) for t in bag if t not in SHARED and not t.startswith(OWN)]
    assert not bad
    assert g['partner_rig_state']['params']['ns'] == '/tejen_d2'


def test_instance_b_shares_no_topic_with_instance_a():
    mod = _module()
    a_typed = dict(NRTF)
    ga, ctx_a, ents_a = _graph(a_typed)
    defaults = {'online_join_planner': mod.PLANNER_TOPICS, 'tejen_mpc': mod.MPC_TOPICS,
                'fake_cooperative_transport_world': mod.RING_TOPICS, 'magnet_attachment_manager_irl': mod.BALL_TOPICS}
    a = set()
    for name in ga:
        a |= _topics({**defaults.get(name, {}), **_effective_a(name, a_typed)})
        a |= set(ga[name]['remaps'].values())
    a |= _topics({**mod.BACKEND_TOPICS, **_effective_a(mod.BACKEND_NODE, a_typed)})
    a |= {'/tejen/drone_command', '/tejen/pendulum_swing_state', '/tejen/payload_world_state', '/drone_3/telemetry',
          *mod.PLANNER_FIXED_TOPICS, *mod.BALL_FIXED_TOPICS}
    a |= _bag(ents_a, ctx_a)[1]
    gb, ctx_b, ents_b = _graph(B)
    b = set().union(*(_topics(n['params']) | set(n['remaps'].values()) for n in gb.values())) | _bag(ents_b, ctx_b)[1]
    assert '/join_planner/reference' in a and '/tejen_d2/join_planner/reference' in b
    assert (a & b) <= SHARED, sorted((a & b) - SHARED)


def test_instance_b_has_every_value_of_his_files():
    """B's parameters are A's, value for value, except the topics, the vehicle id and the logs (and its plate's target)."""
    mod = _module()
    share = PKG / 'config'
    g, _, _ = _graph(B)
    a_typed = {k: v for k, v in B.items() if k not in ('instance', 'drone_id')}
    a_typed.update(plate='9', contact_target_xy='0.0,0.2293', takeoff_gate_topic='/partner/release')
    own = {'online_join_planner': {'vehicle_id'}, 'tejen_mpc': {'logging_instance'},
           mod.BACKEND_NODE: {'vehicle_id', 'csv_path', 'replan_csv_path'},
           'fake_cooperative_transport_world': {'ring_attachment_plate_index', 'bridge_attach_xy_m'},
           'partner_rig_state': {'label', 'ns'}, 'magnet_attachment_manager_irl': set()}
    for node, skip in own.items():
        a = {k: v for k, v in _effective_a(node, a_typed).items() if not mod._is_topic_key(k) and k not in skip}
        b = {k: v for k, v in g[node]['params'].items() if not mod._is_topic_key(k) and k not in skip}
        assert b == a, node
    p = g['online_join_planner']['params']
    for f in ('visual_astar_visual_only.yaml', 'c1f2b_cpp_authority.yaml', 'm1_measured_tip_pickup.yaml',
              'm1_measured_tip_tracking.yaml'):
        assert set(_block(share / f, 'online_join_planner')) <= set(p), f
    assert set(_block(share / 'irl_commissioning.yaml', 'online_join_planner')) <= set(p)
    assert set(_block(share / 'irl_commissioning.yaml', 'tejen_mpc')) <= set(g['tejen_mpc']['params'])
    backend = g[mod.BACKEND_NODE]['params']
    assert set(_block(SRC / 'tejen_dynamic_planner' / 'config' / 'c1f6_moving_rendezvous.yaml', mod.BACKEND_NODE)) \
        <= set(backend)
    assert set(_block(share / 'partner_backend_overrides_rig.yaml', mod.BACKEND_NODE)) <= set(backend)
    # the c1f2b values that were the trap: his C++ authority on, its topics this instance's
    assert p['c1f2_cpp_authority_enabled'] is True
    assert p['c1f1_shadow_reference_topic'] == '/tejen_d2/dynamic_planner/transfer_reference'


def test_instance_b_types_every_topic_his_code_defaults():
    mod = _module()
    g, _, _ = _graph(B)
    for node, table in (('online_join_planner', mod.PLANNER_TOPICS), (mod.BACKEND_NODE, mod.BACKEND_TOPICS),
                        ('tejen_mpc', mod.MPC_TOPICS), ('fake_cooperative_transport_world', mod.RING_TOPICS),
                        ('magnet_attachment_manager_irl', mod.BALL_TOPICS)):
        p = g[node]['params']
        missing = [k for k in table if k not in p]
        assert not missing, (node, missing)
    assert set(g['online_join_planner']['remaps']) == set(mod.PLANNER_FIXED_TOPICS)
    assert set(g['magnet_attachment_manager_irl']['remaps']) == set(mod.BALL_FIXED_TOPICS)


def test_instance_b_contract_names():
    g, ctx, ents = _graph(B)
    p = g['online_join_planner']['params']
    assert {k: p[k] for k in ('drone_state_topic', 'contact_topic', 'ring_attached_topic', 'take_back_topic',
                              'mux_state_topic', 'takeoff_gate_topic', 'give_up_topic', 'proof_lift_complete_topic',
                              'phase_topic', 'reference_topic', 'external_landing_topic', 'drone_command_topic',
                              'magnet_command_topic', 'object_attached_topic', 'magnet_tip_pose_topic',
                              'handoff_ready_topic', 'ring_pose_topic', 'vehicle_id')} == {
        'drone_state_topic': '/drone_2/motion_capture_state', 'contact_topic': '/drone_2/magnet/contact',
        'ring_attached_topic': '/drone_2/magnet/object_attached', 'take_back_topic': '/drone_2/take_back',
        'mux_state_topic': '/drone_2/mux_state', 'takeoff_gate_topic': '/drone_2/partner/release',
        'give_up_topic': '/tejen_d2/join_planner/give_up',
        'proof_lift_complete_topic': '/tejen_d2/join_planner/proof_lift_complete',
        'phase_topic': '/tejen_d2/join_planner/phase', 'reference_topic': '/tejen_d2/join_planner/reference',
        'external_landing_topic': '/tejen_d2/join_planner/land_now', 'drone_command_topic': '/tejen_d2/drone_command',
        'magnet_command_topic': '/tejen_d2/magnet/command', 'object_attached_topic': '/tejen_d2/object_attached',
        'magnet_tip_pose_topic': '/tejen_d2/magnet_tip_pose', 'handoff_ready_topic': '/tejen_d2/join_planner/handoff_ready',
        'ring_pose_topic': '/payload/motion_capture_state', 'vehicle_id': 'drone_2'}
    assert g['online_join_planner']['remaps'] == {
        '/join_planner/force_drop': '/tejen_d2/join_planner/force_drop',
        '/dynamic_planner/static_target': '/tejen_d2/dynamic_planner/static_target',
        '/experiments/navigation_ready': '/tejen_d2/experiments/navigation_ready',
        '/experiments/start_navigation': '/tejen_d2/experiments/start_navigation'}
    rs = g['partner_rig_state']['params']
    assert (rs['body_topic'], rs['tip_topic'], rs['ball_topic'], rs['ns']) == (
        '/drone_2/motion_capture_state', '/drone_2/magnet_tip_pose', '/pickup/motion_capture_state', '/tejen_d2')
    ring = g['fake_cooperative_transport_world']['params']
    assert (ring['payload_pose_topic'], ring['attachment_pose_topic'], ring['attached_plate_topic_prefix']) == (
        '/tejen_d2/fake_payload/pose', '/tejen_d2/fake_attachment_point/pose', '/tejen_d2/fake_obstacles/drone')
    be = g['dynamic_planner_transfer_backend']['params']
    assert (be['state_topic'], be['object_attached_topic'], be['target_topic'], be['vehicle_id']) == (
        '/drone_2/motion_capture_state', '/tejen_d2/object_attached', '/tejen_d2/dynamic_planner/transfer_target',
        'drone_2')
    assert g['magnet_attachment_manager_irl']['remaps'] == {'/join_planner/phase': '/tejen_d2/join_planner/phase'}


def test_console_names_both_instances():
    """What our console (tools/console.py) finds per instance: his MPC and planner by full name, both on the
    instance's drone_command, and the planner's give_up / land_now / phase and the MPC's arming feedback."""
    his_cb = (SRC / 'tejen_utility_objects' / 'tejen_utility_objects' / 'callback_manager.py').read_text()
    assert "'drone_command'" in his_cb and "'drone_arming_state_feedback'" in his_cb     # relative: the namespace's
    for typed, ns, root, jp in ((dict(NRTF), '/tejen', '', '/join_planner'),
                                (B, '/tejen_d2', '/tejen_d2', '/tejen_d2/join_planner')):
        g, _, _ = _graph(typed)
        full = {n: ('/' + v['ns'] if v['ns'] else '') + '/' + n for n, v in g.items()}
        assert (full['tejen_mpc'], full['online_join_planner']) == (f'{ns}/tejen_mpc', f'{root}/online_join_planner')
        p = g['online_join_planner']['params']
        assert p['drone_command_topic'] == f'{ns}/drone_command'
        assert (p['give_up_topic'], p['external_landing_topic'], p.get('phase_topic', '/join_planner/phase')) == (
            f'{jp}/give_up', f'{jp}/land_now', f'{jp}/phase')


def test_instance_b_mpc_follows_its_own_planner_and_radio():
    g, _, _ = _graph(B)
    p, mpc = g['online_join_planner']['params'], g['tejen_mpc']
    assert mpc['params']['external_reference_topic'] == p['reference_topic'] == '/tejen_d2/join_planner/reference'
    assert mpc['params']['payload_mpc_mode_topic'] == p['m2b_mpc_mode_topic'] == '/tejen_d2/join_planner/mpc_mode'
    assert mpc['params']['external_arm_permission_topic'] == '/tejen_d2/join_planner/arm_permission'
    assert mpc['params']['object_attached_topic'] == '/tejen_d2/object_attached'
    assert mpc['remaps'] == {'motion_capture_state': '/drone_2/motion_capture_state',
                             'ELRSCommand': '/drone_2/ELRSCommand_tejen', 'telemetry': '/drone_2/telemetry'}
    assert mpc['params']['logging_instance'] == 'drone_2'


def test_instance_b_plate_carriers_and_logs():
    g, ctx, ents = _graph(B)
    ring = g['fake_cooperative_transport_world']['params']
    # our plate 3 (-y) is his 9; the pair on our 0/6 (cw 0/180) is his 0/180
    assert (ring['ring_attachment_plate_index'], ring['bridge_carrier_azimuths_deg'], ring['fake_drone_count'],
            ring['bridge_attach_xy_m']) == (9, [0.0, 180.0], 2, [0.0, -0.2293])
    be = g['dynamic_planner_transfer_backend']['params']
    assert be['cooperative_drone_state_topics'] == ['/tejen_d2/fake_obstacles/drone_0/state',
                                                     '/tejen_d2/fake_obstacles/drone_1/state']
    assert len(be['cooperative_committed_trajectory_topics']) == len(be['cooperative_attached_plate_topics']) == 2
    assert Path(be['csv_path']).parent.name.endswith('_d2') and Path(be['replan_csv_path']).parent == Path(
        be['csv_path']).parent
    assert _bag(ents, ctx)[0].endswith('partner_bag_d2')
    _, ctx, ents = _graph(dict(B, bag_path='/tmp/x'))
    assert _bag(ents, ctx)[0] == '/tmp/x'      # typed: as typed


def test_two_carriers_reach_instance_a_too():
    """The carriers that stay set his backend's peers for both instances (his backend waits for every listed peer)."""
    g, ctx, ents = _graph(NRTF)
    inc = next(e for e in ents if isinstance(e, IncludeLaunchDescription))
    args = {_s(ctx, k): _s(ctx, v) for k, v in inc.launch_arguments}
    assert args['cooperative_preview_count'] == '2'
    over = _block(args['extra_params_file'].split(',')[-1], 'dynamic_planner_transfer_backend')
    assert over['cooperative_attached_plate_topics'] == [f'/fake_obstacles/drone_{i}/attached_plate' for i in (0, 1)]
    assert g['fake_cooperative_transport_world']['params']['fake_drone_count'] == 2


def test_tip_input_typed_only():
    rs = _graph(NRTF)[0]['partner_rig_state']['params']
    assert rs['tip_topic'] == '/magnet_tip_pose' and 'tip_msg_type' not in rs
    rs = _graph(dict(B, tip_topic='/drone_2/magnet_tip_state', tip_msg_type='motion_capture_state'))[0][
        'partner_rig_state']['params']
    assert (rs['tip_topic'], rs['tip_msg_type']) == ('/drone_2/magnet_tip_state', 'motion_capture_state')


@pytest.mark.parametrize('typed, match', [
    (dict(NRTF, instance='d2'), 'drone_id'),
    (dict(NRTF, drone_id='2'), 'needs instance'),
    (dict(B, instance='q3'), 'instance:=d<d>'),
    (dict(B, instance='d3', drone_id='3'), 'other than 3'),
    ({k: v for k, v in B.items() if k != 'plate'}, 'type plate'),
    ({k: v for k, v in B.items() if k != 'carrier_azimuths_deg'}, 'type carrier_azimuths_deg'),
    (dict(B, carrier_azimuths_deg='[0.0,90.0,180.0]'), 'its own plate'),
    (dict(B, takeoff_gate_topic='/partner/release'), 'is not drone 2'),
    (dict(TWIN_B, mission_mode='pickup_delivery'), 'mission_mode:=floor_join'),
    (dict(TWIN_B, confirm_pickup_release='true'), 'no confirm_pickup_release'),
    (dict(TWIN_B, start_gate_topic='/partner/release'), 'start_gate_topic:=/partner/release is not drone 2'),
    ({k: v for k, v in TWIN_B.items() if k != 'plate'}, 'type plate'),
    ({k: v for k, v in B.items() if k != 'magnet_drop_below_quad'}, "QUAD3's body to magnet marker"),
])
def test_instance_refusals(typed, match):
    with pytest.raises(RuntimeError, match=re.escape(match)):
        _run(_module(), typed)


def test_instance_t_is_the_contract_rule():
    mod = _module()
    b, a = mod.Instance('d2', 2), mod.Instance()
    pairs = {'/magnet/object_attached': '/drone_2/magnet/object_attached', '/magnet/command': '/drone_2/magnet/command',
             '/magnet_tip_pose': '/drone_2/magnet_tip_pose', '/partner/release': '/drone_2/partner/release',
             '/drone_3/take_back': '/drone_2/take_back', '/drone_3/ELRSCommand_tejen': '/drone_2/ELRSCommand_tejen',
             '/join_planner/phase': '/tejen_d2/join_planner/phase', '/tejen/object_attached': '/tejen_d2/object_attached',
             '/tejen/magnet/command': '/tejen_d2/magnet/command', '/dynamic_planner/transfer_status':
             '/tejen_d2/dynamic_planner/transfer_status', '/fake_payload/pose': '/tejen_d2/fake_payload/pose',
             '/payload/motion_capture_state': '/payload/motion_capture_state', 'drone_command': 'drone_command',
             '/drone_2/partner/release': '/drone_2/partner/release', '/tejen_d2/x': '/tejen_d2/x'}
    assert {k: b.t(k) for k in pairs} == pairs
    assert {k: a.t(k) for k in pairs} == {k: k for k in pairs}
    assert (a.ns, b.ns, a.vehicle_id, b.vehicle_id) == ('tejen', 'tejen_d2', 'drone_0', 'drone_2')


# ---- instance B in the twin: a carrier released to his stack ----------------------------------------------------------

def test_twin_instance_b_node_set():
    g, ctx, ents = _graph(TWIN_B)
    assert {n: v['ns'] for n, v in g.items()} == {n: 'tejen_d2' for n in (
        'fake_cooperative_transport_world', 'partner_rig_state', 'magnet_attachment_manager_irl', 'online_join_planner',
        'dynamic_planner_transfer_backend', 'tejen_mpc', 'simulation_test_supervisor')}
    assert not [n for n, v in g.items() if v['files']]
    procs = [e for e in ents if isinstance(e, ExecuteProcess) and not isinstance(e, Node)]
    assert len(procs) == 1 and 'bag' in str([_s(ctx, c) for c in procs[0].cmd])      # no gz call, only the bag
    assert not [e for e in ents if isinstance(e, IncludeLaunchDescription)]
    rs = g['partner_rig_state']['params']
    assert (rs['body_topic'], rs['tip_topic'], rs['tip_msg_type'], rs['anchor_offset_z'], rs['use_sim_time'],
            rs['label']) == ('/drone_2/motion_capture_state', '/drone_2/magnet_tip_state', 'motion_capture_state',
                             -0.04, True, 'QUAD3')
    assert g['magnet_attachment_manager_irl']['params']['use_sim_time'] is True
    assert g['magnet_attachment_manager_irl']['params']['enable_elrs_magnet_output'] is False


def test_twin_instance_b_supervisor_and_mpc():
    g, ctx, ents = _graph(TWIN_B)
    sup = g['simulation_test_supervisor']
    assert sup['params']['start_gate_topic'] == '/drone_2/partner/release'
    assert sup['params']['result_path'].endswith('partner_mission_result_d2.json')
    assert 'pickup_release_state_topic' not in sup['params']
    assert sup['remaps'] == {
        '/motion_capture_state': '/drone_2/motion_capture_state',
        '/pendulum_swing_state': '/tejen_d2/pendulum_swing_state',
        '/payload_world_state': '/payload/motion_capture_state',     # no object: the ring is his readiness payload
        '/join_planner/phase': '/tejen_d2/join_planner/phase',
        '/join_planner/handoff_ready': '/tejen_d2/join_planner/handoff_ready',
        '/dynamic_planner/transfer_status': '/tejen_d2/dynamic_planner/transfer_status',
        '/join_planner/land_now': '/tejen_d2/join_planner/land_now',
        '/join_planner/give_up': '/tejen_d2/join_planner/give_up'}
    mpc = g['tejen_mpc']
    assert mpc['params']['external_reference_topic'] == '/tejen_d2/join_planner/reference'
    assert mpc['params']['thrust_ratio_ukf_min_height_above_start'] == -10.0     # its release gate: kT learned in the air
    assert mpc['remaps']['ELRSCommand'] == '/drone_2/ELRSCommand_tejen'
    p = g['online_join_planner']['params']
    assert (p['drone_state_topic'], p['magnet_tip_pose_topic'], p['vehicle_id'], p['use_sim_time']) == (
        '/drone_2/motion_capture_state', '/tejen_d2/magnet_tip_pose', 'drone_2', True)
    ring = g['fake_cooperative_transport_world']['params']
    assert (ring['ring_attachment_plate_index'], ring['bridge_carrier_azimuths_deg']) == (9, [0.0, 180.0])
    assert _bag(ents, ctx)[0].endswith('partner_bag_d2')


def test_twin_instance_b_topics_are_its_own_and_none_of_a():
    mod = _module()
    gb, ctx_b, ents_b = _graph(TWIN_B)
    b = set()
    for n in gb.values():
        b |= _topics(n['params']) | set(n['remaps'].values())
    b |= _bag(ents_b, ctx_b)[1]
    assert not [t for t in b if t not in SHARED and not t.startswith(OWN)], sorted(b)
    ga, ctx_a, ents_a = _graph(TWIN_A)
    defaults = {'online_join_planner': mod.PLANNER_TOPICS, 'tejen_mpc': mod.MPC_TOPICS,
                'fake_cooperative_transport_world': mod.RING_TOPICS}
    a = set()
    for name in ga:
        a |= _topics({**defaults.get(name, {}), **_effective_a(name, TWIN_A)}) | set(ga[name]['remaps'].values())
    a |= _topics({**mod.BACKEND_TOPICS, **_effective_a(mod.BACKEND_NODE, TWIN_A)})
    a |= {'/tejen/drone_command', *mod.PLANNER_FIXED_TOPICS, *mod.SUPERVISOR_FIXED_TOPICS} | _bag(ents_a, ctx_a)[1]
    assert '/tejen/magnet_tip_pose' in a and '/join_planner/reference' in a
    assert (a & b) <= SHARED, sorted((a & b) - SHARED)


def test_twin_instance_b_has_twin_a_values():
    mod = _module()
    g, _, _ = _graph(TWIN_B)
    a_typed = {k: v for k, v in TWIN_B.items() if k not in ('instance', 'drone_id')}
    a_typed.update(plate='9', start_gate_topic='/partner/release')
    own = {'online_join_planner': {'vehicle_id'}, 'tejen_mpc': {'logging_instance'},
           mod.BACKEND_NODE: {'vehicle_id', 'csv_path', 'replan_csv_path'},
           'fake_cooperative_transport_world': {'ring_attachment_plate_index'},
           'simulation_test_supervisor': {'result_path'}}
    for node, skip in own.items():
        a = {k: v for k, v in _effective_a(node, a_typed).items() if not mod._is_topic_key(k) and k not in skip}
        b = {k: v for k, v in g[node]['params'].items() if not mod._is_topic_key(k) and k not in skip}
        assert b == a, node


def test_heights_are_labelled_with_the_partner(capsys):
    _run(_module(), TWIN_B)
    assert 'QUAD3 planned body heights' in capsys.readouterr().out
    _run(_module(), B)
    assert 'QUAD3 planned body heights' in capsys.readouterr().out
    _run(_module(), NRTF)
    assert 'QUAD4 planned body heights' in capsys.readouterr().out


# ---- the tables of his absolute topic defaults match his code ---------------------------------------------------------

def _py_topic_defaults(path, calls=('declare_parameter',)):
    out = {}
    for n in ast.walk(ast.parse(Path(path).read_text())):
        if (isinstance(n, ast.Call) and getattr(n.func, 'attr', '') in calls and len(n.args) >= 2
                and isinstance(n.args[0], ast.Constant) and isinstance(n.args[1], ast.Constant)):
            name, value = n.args[0].value, n.args[1].value
            if name.endswith(('_topic', '_topic_prefix')) and isinstance(value, str) and value.startswith('/'):
                out[name] = value
    return out


def _py_fixed_topics(path):
    """Absolute topic names written into create_publisher / create_subscription calls."""
    out = set()
    for n in ast.walk(ast.parse(Path(path).read_text())):
        if isinstance(n, ast.Call) and getattr(n.func, 'attr', '') in ('create_publisher', 'create_subscription'):
            for a in n.args:
                if isinstance(a, ast.Constant) and isinstance(a.value, str) and a.value.startswith('/'):
                    out.add(a.value)
    return out


def test_tables_match_his_code():
    mod = _module()
    his = PKG / 'tejen_mission'
    # the planner reads obstacle_diagnostics_topic into safety_status_topic; every other name is its own
    assert _py_topic_defaults(his / 'online_join_planner.py') == mod.PLANNER_TOPICS
    assert _py_fixed_topics(his / 'online_join_planner.py') == set(mod.PLANNER_FIXED_TOPICS)
    assert _py_topic_defaults(SRC / 'tejen_mpc' / 'tejen_mpc' / 'main.py') == mod.MPC_TOPICS
    ring = {**_py_topic_defaults(his / 'fake_cooperative_transport_world.py'),
            **_py_topic_defaults(his / 'ring_bridge.py', calls=('_param',))}
    assert ring == mod.RING_TOPICS
    assert _py_topic_defaults(his / 'magnet_attachment_manager_irl.py') == mod.BALL_TOPICS
    # his twin supervisor: the three inputs our launch has always remapped, and the rest
    assert _py_fixed_topics(his / 'simulation_test_supervisor.py') == set(mod.SUPERVISOR_FIXED_TOPICS) | {
        '/motion_capture_state', '/pendulum_swing_state', '/payload_world_state'}
    assert _py_fixed_topics(his / 'magnet_attachment_manager_irl.py') == set(mod.BALL_FIXED_TOPICS)
    cpp = (SRC / 'tejen_dynamic_planner' / 'src' / 'transfer_backend_node.cpp').read_text()
    found = dict(re.findall(r'declare_parameter<std::string>\(\s*"(\w+_topic)"\s*,\s*"(/[^"]*)"\s*\)', cpp))
    # recovery_continuation_topic defaults to shadow_reference_topic + "_continuation" in his code
    assert {**found, 'recovery_continuation_topic': found['shadow_reference_topic'] + '_continuation'} \
        == mod.BACKEND_TOPICS
    assert not re.search(r'create_(subscription|publisher)<[^>]+>\(\s*"/', cpp)


# ---- our nodes inside his package -----------------------------------------------------------------------------------

@pytest.fixture
def ros():
    import rclpy
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    started = []

    def init(*params):
        args = ['--ros-args'] + [x for p in params for x in ('-p', p)]
        rclpy.init(args=args, domain_id=96)
        started.append(True)
    yield init
    if started:
        rclpy.shutdown()


@pytest.mark.parametrize('prefix', [None, '/tejen_d2/fake_obstacles/drone'])
def test_ring_bridge_attached_plate_prefix(ros, prefix):
    from tejen_mission.ring_bridge import RingBridge
    ros('publish_attached_plates:=true', 'bridge_carrier_azimuths_deg:=[0.0, 180.0]',
        *([f'attached_plate_topic_prefix:={prefix}'] if prefix else []))
    node = RingBridge()
    try:
        names = {p.topic_name for p in node.publishers}
        base = prefix or '/fake_obstacles/drone'
        assert {f'{base}_0/attached_plate', f'{base}_1/attached_plate'} <= names
        assert f'{base}_2/attached_plate' not in names
    finally:
        node.destroy_node()


@pytest.mark.parametrize('tip_type', [None, 'pose_stamped', 'motion_capture_state'])
def test_partner_rig_state_takes_the_tip_as_either_message(ros, tip_type):
    from geometry_msgs.msg import PoseStamped
    from interfaces.msg import MotionCaptureState
    from tejen_mission.partner_rig_state import PartnerRigState
    ros('ns:=/tejen_d2', 'body_topic:=/drone_2/motion_capture_state', 'tip_topic:=/drone_2/magnet_tip_state',
        *([f'tip_msg_type:={tip_type}'] if tip_type else []))
    node = PartnerRigState()
    try:
        subs = {s.topic_name: s.msg_type for s in node.subscriptions}
        want = MotionCaptureState if tip_type == 'motion_capture_state' else PoseStamped
        assert subs['/drone_2/magnet_tip_state'] is want
        assert {p.topic_name for p in node.publishers} >= {'/tejen_d2/magnet_tip_pose', '/tejen_d2/pendulum_swing_state'}
        sent = []
        node.tip_pub = type('P', (), {'publish': lambda self, m: sent.append(m)})()
        node.pen_pub = type('P', (), {'publish': lambda self, m: sent.append(m)})()
        node._body = (node._now(), (0.0, 0.0, 2.0), (1.0, 0.0, 0.0, 0.0))
        tip = want()
        tip.pose.position.z = 1.0
        node._tip_cb(tip)
        assert isinstance(sent[0], PoseStamped) and sent[0].pose.position.z == 1.0
        assert sent[1].pose.position.z == pytest.approx(0.97)       # the 0.03 anchor drop, as his IRL publisher
    finally:
        node.destroy_node()


def test_partner_rig_state_refuses_an_unknown_tip_type(ros):
    from tejen_mission.partner_rig_state import PartnerRigState
    ros('tip_msg_type:=posearray')
    with pytest.raises(ValueError, match='tip_msg_type'):
        PartnerRigState()

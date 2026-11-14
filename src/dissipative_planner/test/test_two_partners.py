"""4 -> 2 -> 4 (contract of 13 Nov 2026, ours.md items #1-#15): our planner with two partners, each flown by his own
instance until our check confirms its attach. The pair drone_0/drone_1 carries on plates 0/6; drone_2 (QUAD3, plate 3)
and drone_3 (QUAD4, plate 9) are the partners. With partner_drones [] or one partner every path reads today's node
fields (the other test files fly that path unchanged); with two each partner has its own record (_pc): its weld topic,
take-back, phase, span, coil, release, rod, weld point and tip. Also the set detach (/fleet/detach_set: unload both,
one resize to the pair, both released, the survivors' loss check still on) and the second fold-in at once (inside the
first's blend it starts from the blended references)."""
import ast
import inspect
import time
from types import MethodType, SimpleNamespace

import numpy as np
import pytest

pytest.importorskip('acados_template')
import dissipative_planner.dissipative_node as dn  # noqa: E402
from dissipative_planner.dissipative_node import (  # noqa: E402
    LAND_TAKE_BACK_SETTLE_S, DissipativeController as D, _NodePartner, _partners, _pt, partner_topic)
from mpc_planner.geometry import balanced_tensions, nominal_cable_dirs  # noqa: E402
from mpc_planner.reference_builder import ReferenceBuilder, unload_tensions  # noqa: E402
from std_msgs.msg import Bool, String  # noqa: E402

HZ = 10.0
R = 0.225
W3 = (0.0, 0.2293, 0.0491)        # QUAD4's weld point on plate 9 (rig_nr8.sh X/Y/W)
W2 = (0.0, -0.2293, 0.0491)       # QUAD3's on plate 3, mirrored
METHODS = ('_take_back', '_land_take_back_tick', '_contact_span_tick', '_contact_warm_ref', '_publish_pending_attach_refs',
           '_publish_departed_refs', '_magnet_cmd_cb', '_magnet_attached_cb', '_newcomers_out_on_land',
           '_newcomer_off_tick', '_his_phase_cb', '_give_up_cb', '_joint_state_cb', '_ball_cb', '_mux_state_cb',
           '_contact_cb', '_pos', '_reserved_index', '_reserved_plate_dir', '_path_clear_point', '_weld_refused',
           '_set_on_load', '_departed_down', '_drop_departed', '_abort_newcomer', '_take_back_plan', '_take_back_tick',
           '_take_back_next', '_ring_moves', '_ring_plan', '_guard_setup', '_guard_read', '_guard_check',
           '_guard_follow', '_guard_ring_fresh', '_weld_rho_of', '_guard_why', '_take_back_avoid_pickup',
           '_tip_now_cb', '_do_attach', '_start_weld_joins_creep', '_detect_tick', '_loss_in_set_unload',
           '_on_unannounced', '_act_on_loss', '_drop_due', '_capture_weld_rho',
           '_drop_and_land', '_release_all_magnets', '_drop_tick', '_cancel_unload_quietly', '_partner_release_tick',
           '_contact_release_tick', '_fleet_detach_set_cb', '_fleet_detach_cb', '_detach_set_plan', '_gap_flies',
           '_resize_and_release', '_start_unload_set', '_unload_tick', '_cancel_unload', '_unload_cancel_on',
           '_rejoin_limit', '_release_magnet', '_newcomer_arming_cb')


class _Log:
    def __init__(self):
        self.lines = []

    def info(self, m, *a, **k):
        self.lines.append(m)

    warn = warning = error = info


class _Pub:
    def __init__(self):
        self.msgs = []

    def publish(self, m):
        self.msgs.append(m.data if hasattr(m, 'data') else m)


def _params(values):
    """A node that only declares parameters: the typed values, else the default."""
    return lambda name, default=None, desc=None: SimpleNamespace(value=values.get(name, default))


def _records(**over):
    v = {'partner_plates': [3, 9], 'partner_rods': [0.994, 0.994], 'partner_weld_x': [W2[0], W3[0]],
         'partner_weld_y': [W2[1], W3[1]], 'partner_weld_z': [W2[2], W3[2]]}
    v.update(over)
    f = SimpleNamespace(n=2, reserved_attach=2, pivot_offset=np.array([0.0, 0.0, -0.04]), attach_radius=R,
                        attach_z=0.0, declare_parameter=_params(v))
    return D._partner_records(f, [2, 3])


def _rho4():
    """Slots 0-3 = drones 0-3: the pair on plates 0 (+x) and 6 (-x), QUAD3 on plate 3 (-y), QUAD4 on plate 9 (+y)."""
    return [np.array([R, 0.0, 0.0]), np.array([-R, 0.0, 0.0]), np.array([0.0, -R, 0.0]), np.array([0.0, R, 0.0])]


def _node(quads=None, welded=False, ring_z=1.0, unload_s=0.0, recs=None, pair_z=0.0):
    """The two-partner node: the pair flying, both partners pending on their mats (or welded on, the square)."""
    if quads is None:
        quads = ((0.0, -0.5, 1.9), (0.0, 0.5, 1.9)) if welded else ((0.0, -1.4, 0.1), (0.0, 1.4, 0.1))
    log = _Log()
    pc = _records(**(recs or {}))
    for P in pc.values():
        P.cmd_pub, P.take_back_pub, P.release_pub, P.joint_free = _Pub(), _Pub(), _Pub(), True
    carriers = [np.array([0.6, 0.0, 1.5]), np.array([-0.6, 0.0, 1.5])]
    rho = _rho4() if welded else [r + [0.0, 0.0, pair_z] for r in _rho4()[:2]]
    ids = [0, 1, 2, 3] if welded else [0, 1]
    dyn = SimpleNamespace(rho=rho, m=0.86, g=9.81)
    s = nominal_cable_dirs(rho, 60.0)
    f = SimpleNamespace(
        _pc=pc, _partner_ids=[2, 3], _partner_contact=True, _newcomer_lifecycle=True, _newcomer_fault_land=False,
        _newcomer_armed={}, _n_carry0=2, reserved_attach=2, n=len(ids), n_net=4, attach_pending=[not welded] * 2,
        attach_pos=[np.array(q, float) for q in quads], _attach_pos_t=[None, None], _approach={}, _newcomer_out={},
        _land_alone=set(), _off_resend_t=None, _approach_hold_armed=False, _attach_moving=True,
        _reconfig_hold_left=0.0, _attach_traj_hold_s=10.0, _departed_hold={}, _departed_step={}, _departed_next={},
        _departed_td={}, _departed_clear={}, _departed_landing=set(), _departed_wait={}, _departed_land=False,
        _departed_land_vel=0.0, land_vel=0.2, _departed_down_z=0.15, _land_to_ground=False, _dropped=None,
        _landed=False, _land_take_back_t=None, takeoff_seen=True, phase='planner', _reconfig_mode='ocp',
        load_state=np.r_[0.0, 0.0, ring_z, 1.0, 0.0, 0.0, 0.0, np.zeros(6)],
        drone_pos=carriers + ([np.array(q, float) for q in quads] if welded else []),
        drone_vel=[np.zeros(3)] * (4 if welded else 2), dyn=dyn, N=20, dt=0.1, drone_mass=0.55,
        attach_radius=R, traj=SimpleNamespace(kind='hover', radius=0.0), hover_xy=None, _departed_gi=False,
        _rest_z={2: 0.1, 3: 0.1}, _departed_sent=set(), _departed_pubs={},
        _attach_target=np.array([0.0, R, ring_z]),         # the one attach target topic: QUAD4's plate
        _attach_warm_radius=1.0, _ball_mass=0.1, traj_t=0.0, _comove={}, _attach_vel=[None, None],
        _traj_running=False, _release_guard=False, slot2drone=list(ids), detached=[False] * 4, rho=rho,
        cable_len=1.0, cable_len_i=[1.0] * len(ids), _min_survivors=2, _drop_gap_deg=200.0, lift_z0=0.0,
        lift_progress=1.0, target_z=1.0, _unload=None, _unload_s=unload_s, _unload_t=0.5, _unload_tilt=10.0,
        _unload_wait=2.0, _post_blend_s=1.0, refs=ReferenceBuilder(dyn, len(ids), s, 0.1, None),
        solver=SimpleNamespace(last_X=None), _reconfig_hold_s=1.5, _partner_release=True, _release_out=0.5,
        _release_up=0.3, _release_around=False, _detach_step_out=0.0, _release_state={},
        detach_pub=[_Pub(), _Pub()], _magnet_pubs=[_Pub(), _Pub()], _detach_magnet=True, _dropped_pub=_Pub(),
        landed_pub=_Pub(), _drop_on_loss=True, _drop_takes_back=True, _drop_t=None, _drop_centre=None,
        _start_attached=True, start_taut=False,
        _relay_task=False, _rejoin_departed=False)
    f.get_logger = lambda: log
    f.log = log
    f.now = [100.0]
    f.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=int(f.now[0] * 1e9)))
    for name in METHODS:
        setattr(f, name, MethodType(getattr(D, name), f))
    f.published = []
    f._publish_ref = lambda d, nodes: f.published.append((d, [tuple(np.array(x, float) for x in n) for n in nodes]))
    f.attached = []
    f._attach_ocp = lambda d, start_creep=False: f.attached.append(d) or True
    f.resized = []

    def _resize(new_n, ids_, rho=None, cable_lengths=None):
        f.resized.append((new_n, list(ids_)))
        f.n, f.slot2drone, f.rho = new_n, list(ids_), list(rho)
        f.dyn = SimpleNamespace(rho=f.rho, m=0.86, g=9.81)
        f.refs = ReferenceBuilder(f.dyn, new_n, nominal_cable_dirs(f.rho, 60.0), 0.1, None)
        return True
    f.resize_fleet = _resize
    f._prime_solver = lambda: None
    return f


def _tick(f):
    f._land_take_back_tick()
    if f._departed_hold:
        f._publish_departed_refs()
    f._newcomer_off_tick()
    f._publish_pending_attach_refs()
    f._contact_span_tick()
    f.now[0] += 1.0 / HZ


def _flying(f, j, phase='TRANSIT_TO_REATTACH', at=None):
    """Partner j airborne on his stream in phase (its own instance's phase topic)."""
    f._his_phase_cb(String(data='TAKEOFF'), j)
    if at is not None:
        f.attach_pos[j] = np.array(at, float)
    f._his_phase_cb(String(data=phase), j)
    f._mux_state_cb(String(data='partner'), j)


# ── the legacy path: one partner reads and writes today's node fields ────────────────────────────────────────────

def test_partner_topics_drone_3_keeps_todays_names_and_drone_2_gets_the_contract_prefix():
    for name in ('/magnet/object_attached', '/magnet/command', '/magnet/contact', '/magnet_tip_pose',
                 '/partner/release', '/drone_3/take_back', '/payload/detachable_joint_state'):
        assert partner_topic(3, name) == name
    assert partner_topic(3, '/join_planner/phase', his=True) == '/join_planner/phase'
    assert partner_topic(3, '/tejen/object_attached', his=True) == '/tejen/object_attached'
    # the console's (builder O) names for partner 2
    assert partner_topic(2, '/magnet/object_attached') == '/drone_2/magnet/object_attached'
    assert partner_topic(2, '/partner/release') == '/drone_2/partner/release'
    assert partner_topic(2, '/magnet/command') == '/drone_2/magnet/command'
    assert partner_topic(2, '/magnet/contact') == '/drone_2/magnet/contact'
    assert partner_topic(2, '/magnet_tip_pose') == '/drone_2/magnet_tip_pose'
    assert partner_topic(2, '/drone_3/take_back') == '/drone_2/take_back'
    assert partner_topic(2, '/join_planner/phase', his=True) == '/tejen_d2/join_planner/phase'
    assert partner_topic(2, '/join_planner/give_up', his=True) == '/tejen_d2/join_planner/give_up'
    assert partner_topic(2, '/tejen/object_attached', his=True) == '/tejen_d2/object_attached'


def test_with_one_partner_the_partner_state_is_the_nodes_own_fields():
    node = SimpleNamespace(_taken_back=False, _span_t=None, _magnet_cmd_pub='pub', _attach_body_to_tip=1.034)
    P = _pt(node, 0)
    assert isinstance(P, _NodePartner) and _pt(node, 1).taken_back is False      # j does not matter: one partner
    assert P.cmd_pub == 'pub' and P.btt == 1.034 and P.span_t is None
    assert P.mux_state is None and P.span_word == 'ON' and P.his_descended is False   # today's getattr defaults
    P.taken_back, P.span_t = True, 5.0
    assert node._taken_back is True and node._span_t == 5.0                       # written where today's code reads
    assert [j for j, _ in _partners(node)] == [0]
    with pytest.raises(KeyError):
        P.not_a_field


# the single-partner node fields: with more than one partner no path may read them, except the features refused then
_LEGACY = {'_taken_back', '_his_phase', '_his_took_off', '_mux_state', '_ball_on', '_span_t', '_span_word',
           '_his_descended', '_tip_now', '_contact_released', '_joint_free', '_contact_said', '_magnet_cmd_pub',
           '_take_back_pub', '_release_pub', '_attach_body_to_tip', '_attach_cable_len', '_weld_rho', '_slot_xy'}
_REFUSED = {'__init__', '_weld_rho_of',                 # sets them up / reads them only without the records
            '_attach_pause_wanted', '_attach_pause_tick',  # attach_pause
            '_release_seen_tick',                          # welded_on_load
            '_tilt_fault_cb',                              # partner_tilt_cut
            '_tilt_tracker_cb',                            # partner_stuck_guard
            '_partner_magnet_keepalive'}                   # partner_handoff_topic (not partner_contact)


def test_no_path_reads_the_single_partner_fields_but_the_features_refused_with_two():
    tree = ast.parse(inspect.getsource(dn))
    bad = []
    for fn in ast.walk(tree):
        if not isinstance(fn, ast.FunctionDef) or fn.name in _REFUSED:
            continue
        for node in ast.walk(fn):
            if isinstance(node, ast.Attribute) and node.attr in _LEGACY and isinstance(node.value, ast.Name) \
                    and node.value.id in ('self', 'node'):
                bad.append((fn.name, node.attr))
            if isinstance(node, ast.Call) and getattr(node.func, 'id', None) == 'getattr' and len(node.args) >= 2 \
                    and isinstance(node.args[1], ast.Constant) and node.args[1].value in _LEGACY:
                bad.append((fn.name, node.args[1].value))
    assert bad == []


def _refusal_node(**over):
    f = SimpleNamespace(_partner_contact=True, _reconfig_mode='ocp', _welded_on_load=False,
                        _tilt_cut=False, _stuck_guard=False, _partner_rod_add=0.0, _attach_pause=False,
                        _attach_approach=False, _tip_log=False, _take_back_avoid_pickup_m=0.0, _partner_ids=[2, 3],
                        get_parameter=lambda name: SimpleNamespace(value=over.pop('handoff', '')))
    for k, v in over.items():
        setattr(f, k, v)
    return f


def test_two_partners_refuse_every_single_partner_feature():
    D._partner_refusals(_refusal_node())                                          # the 4 -> 2 -> 4 set: accepted
    for k, v, word in (('_welded_on_load', True, 'welded_on_load'), ('_tilt_cut', True, 'partner_tilt_cut'),
                       ('_stuck_guard', True, 'partner_stuck_guard'), ('_partner_rod_add', 0.01, 'partner_rod_add_m'),
                       ('_attach_pause', True, 'attach_pause'), ('_attach_approach', True, 'attach_approach'),
                       ('_tip_log', True, 'approach_tip'), ('handoff', '/join_planner/handoff_ready',
                                                            'partner_handoff_topic'),
                       ('_take_back_avoid_pickup_m', 0.5, 'take_back_avoid_pickup_m'),
                       ('_partner_contact', False, 'partner_contact'), ('_reconfig_mode', 'network', 'reconfig_mode')):
        with pytest.raises(ValueError, match=word):
            D._partner_refusals(_refusal_node(**{k: v}))


def test_unload_tensions_one_slot_as_before_and_a_set_holds_the_ring_with_both_pulls():
    rho = _rho4()
    s = nominal_cable_dirs(rho, 60.0)

    def old(rho, s_dirs, load_mass, k, t_leave, g=9.81, t_min=0.1):     # the function before the set (no rod)
        n = len(rho)
        keep = [i for i in range(n) if i != k]
        s_ = [np.asarray(v, float).reshape(3) for v in s_dirs]
        r = [np.asarray(v, float).reshape(3) for v in rho]
        b = np.array([0.0, 0.0, -float(load_mass) * g, 0.0, 0.0, 0.0])
        f_k = float(t_leave) * s_[k]
        b = b - np.concatenate([f_k, np.cross(r[k], f_k)])
        A = np.stack([np.concatenate([s_[i], np.cross(r[i], s_[i])]) for i in keep], axis=1)
        sz = -np.mean([s_[i][2] for i in keep])
        t_eq = np.full(len(keep), -b[2] / max(len(keep) * sz, 1e-9))
        t = t_eq + np.linalg.pinv(A) @ (b - A @ t_eq)
        out = [0.0] * n
        for i, v in zip(keep, t):
            out[i] = float(max(v, t_min))
        out[k] = float(t_leave)
        return out
    for k in range(4):
        assert unload_tensions(rho, s, 0.86, k, 0.5) == old(rho, s, 0.86, k, 0.5)     # bit for bit
        assert unload_tensions(rho, s, 0.86, [k], 0.5) == old(rho, s, 0.86, k, 0.5)
    t = unload_tensions(rho, s, 0.86, [2, 3], 0.5)
    assert t[2] == t[3] == 0.5
    F = sum(ti * si for ti, si in zip(t, s))
    M = sum(np.cross(ri, ti * si) for ri, ti, si in zip(rho, t, s))
    assert np.allclose(F, [0.0, 0.0, -0.86 * 9.81], atol=1e-9) and np.allclose(M, 0.0, atol=1e-9)   # A t = b
    assert t[0] == pytest.approx(t[1])                                              # the pair shares it evenly


# ── the records ─────────────────────────────────────────────────────────────────────────────────────────────────────

def test_each_partner_gets_its_own_plate_rod_and_weld_point():
    pc = _records(partner_rods=[0.98, 0.994])
    assert sorted(pc) == [0, 1] and pc[0].d == 2 and pc[1].d == 3
    assert pc[0].plate == 3 and pc[1].plate == 9
    assert pc[0].rod == 0.98 and pc[0].btt == pytest.approx(0.98 + 0.04) and pc[1].btt == pytest.approx(1.034)
    assert np.allclose(pc[0].weld_rho, W2) and np.allclose(pc[1].weld_rho, W3)
    assert np.allclose(pc[0].plate_rho, [0.0, -R, 0.0], atol=1e-12)                # plate 3: -y (clockwise 90)
    assert np.allclose(pc[1].plate_rho, [0.0, R, 0.0], atol=1e-12)
    assert pc[0].taken_back is False and pc[0].his_phase == '' and pc[0].span_t is None
    nan = _records(partner_weld_x=[], partner_weld_y=[], partner_weld_z=[])
    assert nan[0].weld_rho is None and nan[1].weld_rho is None                      # the plate point stands in


def test_records_refuse_a_wrong_plate_a_short_list_or_ids_that_are_not_the_reserved_drones():
    with pytest.raises(ValueError, match='on plate 9, not its plate 3'):
        _records(partner_weld_y=[W3[1], W3[1]])                                    # QUAD3's point typed on plate 9
    with pytest.raises(ValueError, match='one entry'):
        _records(partner_rods=[0.994])
    with pytest.raises(ValueError, match='one entry'):
        _records(partner_weld_z=[0.05])
    f = SimpleNamespace(n=2, reserved_attach=2, pivot_offset=np.zeros(3), attach_radius=R, attach_z=0.0,
                        declare_parameter=_params({}))
    with pytest.raises(ValueError, match='one per reserved drone'):
        D._partner_records(f, [3, 4])


# ── #1: each weld folds in only its own partner ─────────────────────────────────────────────────────────────────

def test_each_partners_weld_folds_in_only_that_partner():
    f = _node()
    f._magnet_attached_cb(Bool(data=True), 0)                     # /drone_2/magnet/object_attached
    assert f.attached == [2]
    g = _node()
    g._magnet_attached_cb(Bool(data=True), 1)                     # /magnet/object_attached (QUAD4)
    assert g.attached == [3]
    h = _node()
    del h._pc                                                     # today's one shared topic: every pending one
    h._magnet_attached_cb(Bool(data=True))
    assert h.attached == [2, 3]


def test_a_refused_weld_turns_off_only_that_partners_coil():
    f = _node()
    _flying(f, 0, at=(0.0, -0.6, 1.6))
    _flying(f, 1, at=(0.0, 0.6, 1.6))
    _tick(f)                                                      # both spans began
    f._newcomer_out[0] = 'his give-up'
    f._magnet_attached_cb(Bool(data=True), 0)
    assert f.attached == [] and f._pc[0].cmd_pub.msgs[-1] == 'OFF' and f._pc[1].cmd_pub.msgs == ['ON']


# ── #2-#5, #10, #11: phase, warm reference, span, take-back per partner ─────────────────────────────────────────

def test_each_partner_is_warm_from_its_own_takeoff():
    f = _node()
    f._his_phase_cb(String(data='TAKEOFF'), 1)
    _tick(f)
    assert [d for d, _ in f.published] == [3]                      # QUAD3's instance has not taken off
    f.published.clear()
    f._his_phase_cb(String(data='TAKEOFF'), 0)
    _tick(f)
    assert sorted(d for d, _ in f.published) == [2, 3]


def test_each_partner_has_its_own_span_on_its_own_coil():
    f = _node()
    _flying(f, 0, phase='TRANSIT_TO_DROP_POINT', at=(0.0, -0.6, 1.6))
    _flying(f, 1, at=(0.0, 0.6, 1.6))
    _tick(f)
    assert f._pc[1].cmd_pub.msgs == ['ON'] and f._pc[0].cmd_pub.msgs == []        # only QUAD4 rejoins
    f._his_phase_cb(String(data='TRANSIT_TO_REATTACH'), 0)
    _tick(f)
    assert f._pc[0].cmd_pub.msgs == ['ON'] and f._pc[1].cmd_pub.msgs == ['ON']
    f._his_phase_cb(String(data='DESCEND_TO_ATTACHMENT'), 1)
    _tick(f)
    f._his_phase_cb(String(data='MATCH_VELOCITY'), 1)             # QUAD4 climbs back: its coil OFF, QUAD3's stays
    _tick(f)
    assert f._pc[1].cmd_pub.msgs[-1] == 'OFF' and f._pc[0].cmd_pub.msgs == ['ON']


def test_land_takes_back_both_partners_each_on_its_own_bearing_with_its_own_off():
    f = _node()
    _flying(f, 0, at=(0.0, -0.5, 2.2))
    _flying(f, 1, at=(0.0, 0.5, 2.2))
    _tick(f)                                                      # both spans began
    f._newcomers_out_on_land('LAND')
    f.now[0] += LAND_TAKE_BACK_SETTLE_S + 1e-3
    f._land_take_back_tick()
    for j, sign in ((0, -1.0), (1, 1.0)):
        P, d = f._pc[j], 2 + j
        assert P.taken_back and P.take_back_pub.msgs == [True] and P.cmd_pub.msgs[-1] == 'OFF'
        assert f._newcomer_out[j] == 'LAND' and d in f._land_alone and d in f._departed_hold
        step = f._departed_step[d]                                  # out along its own side, at its height
        assert step[0] == pytest.approx(0.0) and np.sign(step[1]) == sign and step[2] == pytest.approx(2.2)
    assert f._pc[0].cmd_pub.msgs.count('OFF') == 1 and f._pc[1].cmd_pub.msgs.count('OFF') == 1


def test_over_the_middle_each_steps_out_toward_its_own_plate_not_the_one_attach_target():
    f = _node()
    _flying(f, 0, at=(0.05, 0.0, 2.2))
    _flying(f, 1, at=(0.05, 0.0, 2.2))
    f._take_back(0, 'LAND')
    f._take_back(1, 'LAND')
    assert f._departed_step[2][1] < -1.0 and f._departed_step[3][1] > 1.0          # plate 3 (-y), plate 9 (+y)


def test_his_give_up_takes_back_only_the_partner_whose_instance_gave_up():
    f = _node()
    _flying(f, 0, at=(0.0, -0.6, 1.6))
    _flying(f, 1, at=(0.0, 0.6, 1.6))
    _tick(f)
    f._give_up_cb(Bool(data=True), 0)                             # /tejen_d2/join_planner/give_up
    assert f._pc[0].taken_back and not f._pc[1].taken_back
    assert f._pc[0].take_back_pub.msgs == [True] and f._pc[1].take_back_pub.msgs == []
    assert 2 in f._departed_hold and 3 not in f._departed_hold
    f.published.clear()
    _tick(f)
    assert sorted(d for d, _ in f.published) == [2, 3]            # 2 its take-back, 3 still warm on his stream
    assert f._pc[0].cmd_pub.msgs[-1] == 'OFF' and 'OFF' not in f._pc[1].cmd_pub.msgs
    f._give_up_cb(Bool(data=True), 1)
    assert f._pc[1].taken_back and f._pc[1].take_back_pub.msgs == [True]


def test_the_take_back_rise_clears_a_welded_partner_but_not_the_other_one_still_off_the_ring():
    f = _node()
    f.slot2drone = [0, 1, 3]                                      # QUAD4 folded in: a carrier now
    f.attach_pending[1] = False
    f.drone_pos = f.drone_pos + [None, np.array([0.0, 0.55, 1.9])]
    _flying(f, 0, at=(0.3, -0.3, 1.0 + 1.034 + 0.01))             # QUAD3's tip just above the ring
    f._take_back(0, 'his give-up')
    assert f._departed_step[2][2] == pytest.approx(1.9 + 0.4)     # over the highest body on the ring: QUAD4's
    for at in ((0.0, 0.5, 2.35), (0.0, 1.6, 2.4)):                 # QUAD4 over its plate or out in his transit
        g = _node()
        _flying(g, 1, at=at)
        _flying(g, 0, at=(0.3, -0.3, 1.0 + 1.034 + 0.01))
        g._take_back(0, 'his give-up')
        # over the ring and the pair only: counting it lifted the rise to ~2.8 m, over the 2.6 m body cap, for no
        # clearance (it is on the other side)
        assert g._departed_step[2][2] == pytest.approx(max(1.0 + 1.034 + 0.15, 1.5 + 0.4))


def test_the_release_guard_reads_each_partners_own_tip_and_plate():
    f = _node()
    f._release_guard = True
    f._load_t = None
    f._pc[0].tip_now = (f.now[0], np.array([0.0, -0.2293, 1.05]))
    f._pc[1].tip_now = (f.now[0], np.array([0.0, 0.2293, 1.05]))
    tip0, z0 = f._guard_read(0)
    tip1, z1 = f._guard_read(1)
    assert np.allclose(tip0[:2], [0.0, -0.2293]) and np.allclose(tip1[:2], [0.0, 0.2293])
    assert z0 == pytest.approx(1.0491) and z1 == pytest.approx(1.0491)
    f._tip_now_cb(SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(x=1.0, y=2.0, z=3.0))), 0)
    assert np.allclose(f._pc[0].tip_now[1], [1.0, 2.0, 3.0]) and np.allclose(f._pc[1].tip_now[1][:2], [0.0, 0.2293])


def test_a_disarm_while_his_stream_flies_one_partner_is_read_on_that_partners_mux():
    f = _node()
    f._newcomer_fault_land = True
    _flying(f, 0, at=(0.0, -0.6, 1.6))
    _flying(f, 1, at=(0.0, 0.6, 1.6))
    f._mux_state_cb(String(data='ours'), 1)                       # QUAD4 on our tracker, QUAD3 on his stream
    for j in (0, 1):
        f._newcomer_arming_cb(Bool(data=True), j)
        f._newcomer_arming_cb(Bool(data=False), j)
    assert 0 not in f._newcomer_out and f._newcomer_out.get(1) == 'fault'


# ── drop ────────────────────────────────────────────────────────────────────────────────────────────────────────

def test_a_drop_takes_back_both_and_its_off_reaches_both_coils():
    f = _node()
    _flying(f, 0, at=(0.0, -0.6, 1.6))
    _flying(f, 1, at=(0.0, 0.6, 1.6))
    f._drop_and_land('test')
    assert f._pc[0].taken_back and f._pc[1].taken_back
    assert f._pc[0].take_back_pub.msgs == [True] and f._pc[1].take_back_pub.msgs == [True]
    assert f._pc[0].cmd_pub.msgs == ['OFF'] and f._pc[1].cmd_pub.msgs == ['OFF']
    assert all(p.msgs == ['OFF'] for p in f._magnet_pubs)
    f._departed_down = lambda: False
    f._publish_departed_refs = lambda: None
    f._publish_pending_attach_refs = lambda: None
    f.now[0] += 1.0
    f._drop_tick()
    assert f._pc[0].cmd_pub.msgs == ['OFF', 'OFF'] and f._pc[1].cmd_pub.msgs == ['OFF', 'OFF']
    f.now[0] += 30.0
    f._drop_tick()
    assert f.landed_pub.msgs == []                                # the take-back's extra wait (any partner)
    f.now[0] += 30.0
    f._drop_tick()
    assert f.landed_pub.msgs == [True]


def test_a_drop_with_both_welded_turns_off_both_coils():
    f = _node(welded=True)
    f._drop_and_land('test')
    assert f._pc[0].cmd_pub.msgs == ['OFF'] and f._pc[1].cmd_pub.msgs == ['OFF']
    assert not f._pc[0].taken_back and not f._pc[1].taken_back   # not pending: carriers dropped with the ring


# ── #7: the release, per partner ────────────────────────────────────────────────────────────────────────────────

def test_each_partner_is_released_on_its_own_topic_and_his_stream_warms_each_on_its_own():
    f = _node(welded=True)
    f._fleet_detach_set_cb(String(data='2,3'))
    for d in (2, 3):
        f.drone_pos[d] = f._release_state[d]['target'].copy()
    for _ in range(int(HZ * 8)):
        f._partner_release_tick()
        f.now[0] += 1.0 / HZ
    assert f._pc[0].release_pub.msgs[0] == '{"running": true}' and f._pc[1].release_pub.msgs[0] == '{"running": true}'
    assert f.attach_pending == [True, True] and f._pc[0].contact_released and f._pc[1].contact_released
    f._his_phase_cb(String(data='TAKEOFF'), 1)
    f._mux_state_cb(String(data='partner'), 1)
    f._contact_release_tick()
    assert 3 not in f._departed_hold and 2 in f._departed_hold    # only QUAD4's stream flies it yet
    f._give_up_cb(Bool(data=True), 0)
    n0 = len(f._pc[0].release_pub.msgs)
    for _ in range(int(HZ * 2)):
        f._partner_release_tick()
        f.now[0] += 1.0 / HZ
    assert len(f._pc[0].release_pub.msgs) == n0 and len(f._pc[1].release_pub.msgs) > 1


# ── #12: the set detach ─────────────────────────────────────────────────────────────────────────────────────────

def test_the_set_detach_releases_both_with_one_resize_to_the_pair():
    f = _node(welded=True)
    f._fleet_detach_set_cb(String(data='2,3'))
    assert f.resized == [(2, [0, 1])]                             # ONE resize, to the pair
    assert f.detached == [False, False, True, True]
    assert f._pc[0].cmd_pub.msgs == ['OFF'] and f._pc[1].cmd_pub.msgs == ['OFF']   # each its own coil
    assert f.detach_pub[0].msgs == [] and f.detach_pub[1].msgs == []
    assert set(f._release_state) == {2, 3} and f._reconfig_hold_left == 1.5       # both stepped out, one hold
    assert f._release_state[2]['target'][1] < -0.6 and f._release_state[3]['target'][1] > 0.6
    assert sum('DETACH drone 3 (/drone_2), drone 4 (/drone_3) (OCP resize)' in m for m in f.log.lines) == 1


def test_the_set_unload_unloads_both_together_gates_both_then_one_release():
    f = _node(welded=True, unload_s=3.0)
    X = np.zeros((13 + 14 * 4, 21))
    X[13 + 14 * 2 + 12, :] = 0.6
    X[13 + 14 * 3 + 12, :] = 1.5                                  # QUAD4 still planned loaded
    f.solver.last_X = X
    f._fleet_detach_set_cb(String(data='2,3'))
    u = f._unload
    assert u['ids'] == [2, 3] and u['ks'] == [2, 3] and f.refs.blend_active() and f.resized == []
    assert u['target'][2] == u['target'][3] == 0.5
    f._fleet_detach_cb(SimpleNamespace(data=3))                   # a single DETACH of a set member: nothing
    assert f._unload is u and not any('refused' in m for m in f.log.lines)
    for _ in range(35):
        f._unload_tick()
        f.refs.update((0, 0), 0.0, 0.0, 1.0, 0.0, 0.0)
    assert f.resized == [] and f._unload is u                     # one leaver over the gate: no release
    X[13 + 14 * 3 + 12, :] = 0.6
    f._unload_tick()
    assert f.resized == [(2, [0, 1])] and f._unload is None and f.detached == [False, False, True, True]
    assert f.refs.blend_active()                                  # the pair's post-release blend
    assert any('UNLOADED SET [2, 3]' in m for m in f.log.lines)


def test_a_set_unload_whose_gates_fail_cancels_with_both_magnets_on():
    f = _node(welded=True, unload_s=3.0)
    X = np.zeros((13 + 14 * 4, 21))
    X[13 + 14 * 2 + 12, :] = X[13 + 14 * 3 + 12, :] = 1.5
    f.solver.last_X = X
    f._fleet_detach_set_cb(String(data='2,3'))
    for _ in range(60):
        f._unload_tick()
        f.refs.update((0, 0), 0.0, 0.0, 1.0, 0.0, 0.0)
    assert f._unload is None and f.resized == [] and f._pc[0].cmd_pub.msgs == [] and f._pc[1].cmd_pub.msgs == []
    assert any('UNLOAD CANCELLED for drone [2, 3]' in m and 'magnets stay ON' in m for m in f.log.lines)


@pytest.mark.parametrize('why, setup', [
    ('min_survivors', lambda f: setattr(f, '_min_survivors', 3)),
    ('gap', lambda f: setattr(f, '_drop_gap_deg', 0.0)),               # a pair flies only within drop_gap_deg
    ('gap', lambda f: setattr(f, 'lift_progress', 0.5)),               # ... and after the lift
    ('LAND', lambda f: setattr(f, '_land_to_ground', True)),
    ('dropped', lambda f: setattr(f, '_dropped', 'x')),
    ('unloading', lambda f: setattr(f, '_unload', {'d': 0})),
    ('not on the load', lambda f: setattr(f, 'slot2drone', [0, 1, 2])),
])
def test_the_set_detach_is_refused_when_the_survivors_would_not_fly_or_the_fleet_is_busy(why, setup):
    f = _node(welded=True)
    setup(f)
    f._fleet_detach_set_cb(String(data='2,3'))
    assert f.resized == [] and f.detached == [False] * 4 and f._pc[0].cmd_pub.msgs == []


def test_a_bad_set_message_is_refused_and_one_leaver_is_the_plain_detach():
    f = _node(welded=True)
    for bad in ('', 'x', '2,2', '0,1', '1', '2,0'):                 # a set names partners only
        f._fleet_detach_set_cb(String(data=bad))
    assert f.resized == [] and f.detached == [False] * 4 and sum('refused' in m for m in f.log.lines) == 6
    assert f._detach_set_plan([0, 1]) is None
    f._detach_ocp = lambda d: f.attached.append(('plain', d))
    f._fleet_detach_set_cb(String(data='3'))
    assert f.attached == [('plain', 3)]


# ── #13: the second fold-in at once ─────────────────────────────────────────────────────────────────────────────

def _foldin_node(recs=None, pair_z=0.0):
    """The pair flying, both partners hovering with their tips on their plates; the real _attach_ocp on a real
    ReferenceBuilder (resize_fleet's nominal directions)."""
    f = _node(recs=recs, pair_z=pair_z)
    f.__dict__.update(psi0=0.0, attach_z=pair_z, _ocp_attached={}, _weld_time={}, _attach_t_start_new=0.1,
                      _attach_datum_shift=False, _attach_blend_balanced=True, _hold_mode='timed',
                      solver=SimpleNamespace(last_X=np.zeros((1, 1))))
    f._s_nom = nominal_cable_dirs(f.rho, 60.0)
    f.refs = ReferenceBuilder(f.dyn, 2, f._s_nom, 0.1, None)
    f._attach_ocp = MethodType(D._attach_ocp, f)
    f._log_resize_inputs = lambda *a: None

    def _resize(new_n, ids_, rho=None, cable_lengths=None):
        f.resized.append((new_n, list(ids_)))
        f.n, f.slot2drone, f.rho, f.cable_len_i = new_n, list(ids_), list(rho), list(cable_lengths)
        f.dyn = SimpleNamespace(rho=f.rho, m=0.86, g=9.81)
        f._s_nom = nominal_cable_dirs(f.rho, 60.0)
        f.refs = ReferenceBuilder(f.dyn, new_n, f._s_nom, 0.1, None)
        return True
    f.resize_fleet = _resize
    for j, P in f._pc.items():                                    # each tip on its record's weld point
        _flying(f, j, at=(P.weld_xyz[0], P.weld_xyz[1], 1.0 + P.weld_xyz[2] + P.btt))
    return f


def _angle(a, b):
    return float(np.degrees(np.arccos(np.clip(np.dot(a, b) / np.linalg.norm(a) / np.linalg.norm(b), -1.0, 1.0))))


def test_two_quick_proofs_fold_both_in_at_once_and_the_second_starts_from_the_blended_references():
    f = _foldin_node()
    f._magnet_attached_cb(Bool(data=True), 0)
    assert f.resized == [(3, [0, 1, 2])] and f.attach_pending == [False, True] and f.refs.blend_active()
    for _ in range(30):                                           # 3 s into QUAD3's 10 s blend
        f.refs.update((0, 0), 0.0, 1.0, 1.0, 0.0, 0.0)
    s_b, t_b = f.refs._refs_at(0)
    nominal_2 = nominal_cable_dirs(_rho4(), 60.0)[2]              # where today's fold-in would restart QUAD3's cable
    assert _angle(s_b[2], nominal_2) > 5.0                         # QUAD3's cable still swinging out
    f._magnet_attached_cb(Bool(data=True), 1)                     # QUAD4's proof: folded in at once, not held back
    assert f.resized[-1] == (4, [0, 1, 2, 3]) and f.attach_pending == [False, False]
    assert any('inside the blend before it' in m for m in f.log.lines)
    s_a, t_a = f.refs._refs_at(0)
    for i in range(3):                                            # every incumbent from where it was: no step
        assert abs(t_a[i] - t_b[i]) < 0.1 and _angle(s_a[i], s_b[i]) < 0.5
    for _ in range(102):                                          # then on to the square's nominal split
        f.refs.update((0, 0), 0.0, 1.0, 1.0, 0.0, 0.0)
    assert not f.refs.blend_active() and np.allclose(f.refs._refs_at(0)[1], f.refs._t_nom, atol=1e-9)


def test_land_or_a_give_up_right_after_the_second_proof_takes_nothing_welded_back():
    for event in ('give-up', 'LAND', 'drop'):
        f = _foldin_node()
        f._magnet_attached_cb(Bool(data=True), 0)
        f._magnet_attached_cb(Bool(data=True), 1)
        if event == 'give-up':
            f._give_up_cb(Bool(data=True), 1)
            f._give_up_cb(Bool(data=True), 0)
        elif event == 'LAND':
            f._newcomers_out_on_land('LAND')
            f.now[0] += LAND_TAKE_BACK_SETTLE_S + 1e-3
            f._land_take_back_tick()
        else:
            f._drop_and_land('test')
        for P in f._pc.values():
            assert not P.taken_back and P.take_back_pub.msgs == []
            assert P.cmd_pub.msgs == ([] if event != 'drop' else ['OFF'])   # only a drop lets the ring go
        assert f._newcomer_out == {} and f._land_alone == set()      # (a drop steps every drone clear, as today)


def test_start_welds_in_the_creep_fold_in_together():
    f = _node()
    f.phase, f.takeoff_seen = 'creep', False
    f.creep = SimpleNamespace(sweep_started=lambda: False)
    f._magnet_attached_cb(Bool(data=True), 0)
    f._magnet_attached_cb(Bool(data=True), 1)
    assert f.attached == [2, 3]


# ── the survivors' loss check through a set unload (critic finding 6) ───────────────────────────────────────────

def _unloading(drop_on_loss=True):
    f = _node(welded=True, unload_s=3.0)
    X = np.zeros((13 + 14 * 4, 21))
    X[13 + 14 * 2 + 12, :] = X[13 + 14 * 3 + 12, :] = 1.5
    f.solver.last_X = X
    f.__dict__.update(_detect_m=0.06, _detect_count={}, _detect_ok={}, _detect_refused=set(), _drop_on_loss=drop_on_loss)
    f.dists = [1.0] * 4
    f._rim_dist = lambda i: f.dists[i]
    f._fleet_detach_set_cb(String(data='2,3'))
    assert f._unload is not None and f._unload['ids'] == [2, 3]
    return f


def _check(f):
    f._load_t = time.monotonic()
    f._drone_t = {k: time.monotonic() for k in range(4)}
    f._detect_tick()


def test_a_pair_carrier_lost_during_the_set_unload_drops_the_ring():
    f = _unloading()
    _check(f)
    f.dists[0] = 1.12                                             # drone_0's magnet let go: its cable 12 cm over
    _check(f)
    assert f._dropped is None                                     # one tick is not enough
    _check(f)
    assert f._dropped is not None and 'during the set unload of [2, 3]' in f._dropped and f._unload is None
    assert f._pc[0].cmd_pub.msgs == ['OFF'] and f._pc[1].cmd_pub.msgs == ['OFF'] and f.resized == []


def test_the_leavers_going_light_never_read_as_a_loss_and_off_drop_only_cancels():
    f = _unloading()
    for _ in range(4):
        _check(f)
        f.dists[2] = f.dists[3] = f.dists[2] + 0.1                # the leavers' cables (slack, swinging): not checked
    assert f._dropped is None and f._unload is not None
    g = _unloading(drop_on_loss=False)
    _check(g)
    g.dists[1] = 1.12
    _check(g)
    _check(g)
    assert g._dropped is None and g._unload is None and any('UNLOAD CANCELLED' in m for m in g.log.lines)
    h = _node(welded=True)
    h.__dict__.update(_detect_m=0.06, _detect_count={}, _detect_ok={}, _detect_refused=set(),
                      _unload={'d': 3, 'k': 3}, dists=[1.0, 1.12, 1.0, 1.0])
    h._rim_dist = lambda i: h.dists[i]
    h._on_unannounced = lambda d: h.attached.append(d)
    for _ in range(3):
        _check(h)
    assert h.attached == [] and h._dropped is None                # one leaver's soft detach: no check, as today


# ── the fold-in's nominal split with two partners: the lift and level-moment rows (Wesley 13 Nov) ──────────────────

RIG = ({'partner_weld_x': [W2[0], W3[0]], 'partner_weld_y': [W2[1], W3[1]], 'partner_weld_z': [W2[2], W3[2]],
        'partner_rods': [0.994, 0.994]}, 0.0175)          # QUAD4's record, QUAD3's mirrored (to measure); the M2A ring
TWIN = ({'partner_weld_x': [0.0, 0.0], 'partner_weld_y': [-0.225, 0.225], 'partner_weld_z': [0.032, 0.032],
         'partner_rods': [1.0, 1.0]}, 0.02)               # configs/twin/partners_424_twin.yaml; sim.yaml's attach_z


@pytest.mark.parametrize('recs, pair_z', [RIG, TWIN], ids=['rig', 'twin'])
def test_after_both_fold_ins_the_four_share_the_ring_and_the_t_carries_its_rider(recs, pair_z):
    f = _foldin_node(recs, pair_z)
    f._magnet_attached_cb(Bool(data=True), 0)
    t_T = list(f.refs._t_nom)                                     # the T 0/6/3: QUAD3 off the pair's line
    assert t_T[0] == pytest.approx(t_T[1]) and t_T[2] == pytest.approx(0.1)   # its rider: no level split loads it
    f._magnet_attached_cb(Bool(data=True), 1)
    t_sq = list(f.refs._t_nom)
    assert min(t_sq) > 1.5 and max(t_sq) < 3.5                    # every one carries: none at the 0.1 N floor
    s, rho = f.refs._s_nom, f.rho
    lift = sum(t * float(v[2]) for t, v in zip(t_sq, s))
    M = sum(np.cross(r, t * np.asarray(v)) for r, t, v in zip(rho, t_sq, s))
    assert lift == pytest.approx(-0.86 * 9.81) and np.allclose(M[:2], 0.0, atol=1e-9)   # held up and level
    all_rows = balanced_tensions(rho, s, 0.86)                    # the six rows: both partners at the floor
    assert all_rows[2] == pytest.approx(0.1) and all_rows[3] == pytest.approx(0.1)


def test_with_one_partner_the_fold_in_keeps_the_six_row_split():
    """Today's one partner (the real _attach_ocp on test_partner_contact's fold-in node): the six-row balance, as before
    (the comparison script checks the whole path bit for bit against the code before the 4 -> 2 -> 4)."""
    from test_partner_contact import _fold_in_node
    g = _fold_in_node()
    g._magnet_attached_cb(Bool(data=True))
    assert g.attach_pending == [False] and g.refs._t_nom == balanced_tensions(g.dyn.rho, g._s_nom, 0.86)

"""Combined M1 contact attach, our planner (card 2026-10-10_combined_m1_contact rev 4, critic round 3 finding 1):
his mission flies QUAD4 until our check confirms the attach. Our drone-3 tracker is warm on its measured body from his
takeoff, the ring magnet span starts once he rejoins with QUAD4 airborne and the ring weld free, our own ON never
starts an approach or holds the ring, and with no attach (his give-up, our LAND) we take QUAD4 back: a fixed hold,
radially out (over the middle along plate 9's azimuth) and down alone. All behind partner_contact (off by default)."""
from types import MethodType, SimpleNamespace

import numpy as np
import pytest

pytest.importorskip('acados_template')
from dissipative_planner.dissipative_node import (  # noqa: E402
    CONTACT_SPAN_EVERY_S, DEPARTED_CLEAR_R, NEWCOMER_RELEASE_S, DissipativeController as D)
from std_msgs.msg import Bool, String  # noqa: E402

HZ = 10.0
METHODS = ('_take_back', '_contact_span_tick', '_contact_warm_ref', '_publish_pending_attach_refs',
           '_publish_departed_refs', '_magnet_cmd_cb', '_magnet_attached_cb', '_newcomers_out_on_land',
           '_newcomer_arming_cb', '_newcomer_off_tick', '_his_phase_cb', '_give_up_cb', '_joint_state_cb',
           '_ball_cb', '_mux_state_cb', '_pos', '_reserved_index', '_reserved_plate_dir', '_path_clear_point',
           '_weld_refused', '_set_on_load', '_departed_down', '_drop_departed', '_start_approach',
           '_abort_newcomer', '_attach_ocp', '_capture_weld_rho', '_start_weld_joins_creep', '_do_attach')


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


def _node(quad=(0.0, 1.4, 0.1), ring_vel=(0.0, 0.0, 0.0), contact=True):
    log = _Log()
    carriers = [np.array([0.5 * np.cos(np.radians(a)), 0.5 * np.sin(np.radians(a)), 1.5]) for a in (270, 30, 150)]
    f = SimpleNamespace(
        _partner_contact=contact, _newcomer_lifecycle=True, _newcomer_fault_land=True, _newcomer_armed={},
        _n_carry0=3, reserved_attach=1, n=3, attach_pending=[True], attach_pos=[np.array(quad, float)],
        _attach_pos_t=[None], _approach={}, _newcomer_out={}, _land_alone=set(), _off_resend_t=None,
        _approach_hold_armed=False, _attach_moving=True, _reconfig_hold_left=0.0, _attach_traj_hold_s=10.0,
        _departed_hold={}, _departed_step={}, _departed_next={}, _departed_td={}, _departed_clear={},
        _departed_landing=set(), _departed_wait={}, _departed_land=False, _departed_land_vel=0.0, land_vel=0.2,
        _departed_down_z=0.15, _land_to_ground=False, _dropped=None, _landed=False,
        _approach_hold_on=False, _attach_approach=False, takeoff_seen=True, phase='planner',
        load_state=np.r_[0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, ring_vel, np.zeros(3)], drone_pos=list(carriers),
        dyn=SimpleNamespace(g=9.81), N=20, dt=0.1, drone_mass=0.55, attach_radius=0.225,
        _magnet_cmd_pub=_Pub(), _take_back_pub=_Pub(), traj=SimpleNamespace(kind='hover', radius=0.0),
        hover_xy=None, _departed_gi=False, _rest_z={3: 0.1}, _departed_sent=set(), _departed_pubs={},
        _attach_target=np.array([0.0, 0.225, 1.05]), _attach_warm_radius=1.0,
        net=SimpleNamespace(rho=[None, None, None, np.array([0.0, 0.225, 0.02])]),
        _his_phase='', _his_took_off=False, _mux_state='partner', _joint_free=None, _ball_on=False,
        _ball_mass=0.1, _span_t=None, _taken_back=False)
    f.get_logger = lambda: log
    f.log = log
    f.now = [100.0]
    f.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=int(f.now[0] * 1e9)))
    for name in METHODS:
        setattr(f, name, MethodType(getattr(D, name), f))
    f.published = []
    f._publish_ref = lambda d, nodes: f.published.append((d, [tuple(np.array(x, float) for x in n) for n in nodes]))
    return f


def _tick(f):
    """The planner tick's order for these paths (_plan_inner)."""
    if f._departed_hold:
        f._publish_departed_refs()
    f._newcomer_off_tick()
    f._publish_pending_attach_refs()
    f._contact_span_tick()
    f.now[0] += 1.0 / HZ


def _airborne_rejoin(f, z=1.6):
    f._his_phase_cb(String(data='TAKEOFF'))
    f.attach_pos[0] = np.array([0.0, 0.6, z])
    f._his_phase_cb(String(data='TRANSIT_TO_REATTACH'))
    f._joint_state_cb(String(data='detached'))


# ── the warm reference ───────────────────────────────────────────────────────────────────────────────────────

def test_warm_reference_starts_at_his_takeoff_on_its_measured_body_anywhere():
    f = _node()
    f._his_phase_cb(String(data='WAIT_FOR_TAKEOFF'))
    _tick(f)
    assert f.published == []                                   # nothing before his takeoff
    f._his_phase_cb(String(data='TAKEOFF'))
    _tick(f)
    (d, nodes), = f.published                                  # on its mat, 1.4 m from the ring: still warm
    assert d == 3 and np.allclose(nodes[0][0], [0.0, 1.4, 0.1]) and np.allclose(nodes[0][1], 0.0)
    assert np.allclose(nodes[0][2], [0.0, 0.0, 9.81]) and np.allclose(nodes[0][3], 0.0)
    assert len(nodes) == f.N + 1 and np.allclose(nodes[-1][0], nodes[0][0])


def test_near_the_ring_the_ring_velocity_is_fed_forward_along_the_horizon():
    f = _node(quad=(0.0, 0.5, 1.6), ring_vel=(0.12, 0.0, 0.0))
    f._his_phase_cb(String(data='DESCEND_TO_ATTACHMENT'))
    _tick(f)
    (d, nodes), = f.published
    assert np.allclose(nodes[0][0], [0.0, 0.5, 1.6]) and np.allclose(nodes[0][1], [0.12, 0.0, 0.0])
    assert np.allclose(nodes[10][0], [0.12, 0.5, 1.6])         # 1 s along the horizon at the ring's velocity
    g = _node(quad=(0.0, 1.4, 1.6), ring_vel=(0.12, 0.0, 0.0))  # outside the warm radius: no feed-forward
    g._his_phase_cb(String(data='TRANSIT_TO_REATTACH'))
    _tick(g)
    assert np.allclose(g.published[0][1][0][1], 0.0)


def test_off_by_default_the_old_proximity_hold_is_unchanged():
    f = _node(quad=(0.0, 1.4, 1.6), contact=False)
    f._his_took_off = True
    f._publish_pending_attach_refs()
    assert f.published == []                                   # outside attach_warm_radius: no reference


def test_one_reference_per_tick_for_drone_3_warm_taken_back_and_folded_in():
    f = _node()
    _airborne_rejoin(f)
    for _ in range(3):
        _tick(f)
    assert [d for d, _ in f.published] == [3, 3, 3]
    f.published.clear()
    f._give_up_cb(Bool(data=True))
    for _ in range(3):
        _tick(f)
    assert [d for d, _ in f.published] == [3, 3, 3]            # its departed hold only, no warm reference
    g = _node()
    _airborne_rejoin(g)
    g.attach_pending[0] = False                                # folded in: the OCP flies it
    _tick(g)
    assert g.published == []


# ── the ring magnet span ──────────────────────────────────────────────────────────────────────────────────────

def test_the_span_waits_for_his_rejoin_an_airborne_quad4_and_the_free_joint():
    f = _node()
    f._his_phase_cb(String(data='TAKEOFF'))
    f.attach_pos[0] = np.array([0.0, 0.6, 1.6])
    f._joint_state_cb(String(data='detached'))
    f._his_phase_cb(String(data='TRANSIT_TO_DROP_POINT'))
    _tick(f)
    assert f._magnet_cmd_pub.msgs == []                        # not before his rejoin (the ball is on the coil)
    f._his_phase_cb(String(data='TRANSIT_TO_REATTACH'))
    f.attach_pos[0] = np.array([0.0, 0.6, 0.25])
    _tick(f)
    assert f._magnet_cmd_pub.msgs == []                        # not below 0.3 m
    f.attach_pos[0] = np.array([0.0, 0.6, 1.6])
    f._joint_free = None
    _tick(f)
    f._joint_state_cb(String(data='attached'))
    _tick(f)
    assert f._magnet_cmd_pub.msgs == []                        # not before the ring weld reads free
    f._joint_state_cb(String(data='detached'))
    _tick(f)
    assert f._magnet_cmd_pub.msgs == ['ON'] and f._span_t is not None
    for _ in range(int(CONTACT_SPAN_EVERY_S * HZ) - 1):
        _tick(f)
    assert f._magnet_cmd_pub.msgs == ['ON']
    _tick(f)
    assert f._magnet_cmd_pub.msgs == ['ON', 'ON']              # the 1 s keepalive


def test_the_span_stops_at_the_attach_and_at_the_take_back():
    f = _node()
    _airborne_rejoin(f)
    _tick(f)
    f.attach_pending[0] = False
    for _ in range(30):
        _tick(f)
    assert f._magnet_cmd_pub.msgs == ['ON']
    g = _node()
    _airborne_rejoin(g)
    _tick(g)
    g._give_up_cb(Bool(data=True))
    for _ in range(30):
        _tick(g)
    assert 'ON' not in g._magnet_cmd_pub.msgs[1:]


def test_our_own_on_starts_no_approach_and_holds_no_trajectory():
    f = _node()
    f._approach_hold_on = True                                 # even with the hold switched on
    f._attach_approach = True
    f._his_phase_cb(String(data='TAKEOFF'))
    f.attach_pos[0] = np.array([0.0, 0.6, 1.6])
    f._magnet_cmd_cb(String(data='ON'))
    assert f._approach == {} and f._reconfig_hold_left == 0.0 and not f._approach_hold_armed


# ── the take-back ────────────────────────────────────────────────────────────────────────────────────────────

def test_give_up_takes_it_back_holds_then_moves_radially_out_at_its_height():
    f = _node()
    _airborne_rejoin(f)
    f.attach_pos[0] = np.array([0.5, 0.5, 1.6])
    _tick(f)                                                   # the span began
    f._give_up_cb(Bool(data=True))
    assert f._take_back_pub.msgs == [True] and f._newcomer_out == {0: 'his give-up'}
    assert f._magnet_cmd_pub.msgs[-1] == 'OFF'                 # the span began: its OFF
    assert np.allclose(f._departed_hold[3], [0.5, 0.5, 1.6]) and f._departed_wait[3] == NEWCOMER_RELEASE_S
    step = f._departed_step[3]
    assert np.isclose(np.hypot(step[0], step[1]), DEPARTED_CLEAR_R) and np.isclose(step[0], step[1])
    assert step[2] == pytest.approx(1.6) and 3 in f._land_alone
    f.published.clear()
    for _ in range(int(round(NEWCOMER_RELEASE_S * HZ))):
        _tick(f)
    assert all(np.allclose(n[0][0], [0.5, 0.5, 1.6]) for _, n in f.published)   # the fixed hold
    for _ in range(3):
        _tick(f)
    p = f.published[-1][1][0][0]
    assert p[0] > 0.5 and p[1] > 0.5 and p[2] == pytest.approx(1.6)


def test_over_the_middle_it_steps_out_along_its_plate_azimuth():
    f = _node()
    _airborne_rejoin(f)
    f.attach_pos[0] = np.array([0.05, -0.05, 1.6])             # r 0.07 < 0.225, on the far side of plate 9
    f._take_back(0, 'LAND')
    assert np.allclose(f._departed_step[3], [0.0, DEPARTED_CLEAR_R, 1.6])
    g = _node()
    _airborne_rejoin(g)
    g._attach_target = None                                    # no live target: the reserved slot's plate
    g.attach_pos[0] = np.array([0.05, -0.05, 1.6])
    g._take_back(0, 'LAND')
    assert np.allclose(g._departed_step[3], [0.0, DEPARTED_CLEAR_R, 1.6])


def test_far_from_the_ring_it_lands_where_it_is():
    f = _node()
    _airborne_rejoin(f)
    f.attach_pos[0] = np.array([1.0, 1.6, 2.4])
    f._take_back(0, 'LAND')
    assert 3 not in f._departed_step and 3 not in f._departed_next
    for _ in range(int(round(NEWCOMER_RELEASE_S * HZ)) + 10):
        _tick(f)
    p = f.published[-1][1][0][0]
    assert np.allclose(p[:2], [1.0, 1.6]) and p[2] < 2.4 - 0.2  # straight down, alone


def test_land_before_the_attach_is_a_take_back_not_the_approach_abort():
    f = _node()
    _airborne_rejoin(f)
    f._newcomers_out_on_land('LAND')
    assert f._take_back_pub.msgs == [True] and f._newcomer_out == {0: 'LAND'} and 3 in f._departed_hold
    f._newcomers_out_on_land('LAND')
    f._give_up_cb(Bool(data=True))
    assert f._take_back_pub.msgs == [True] and f._newcomer_out == {0: 'LAND'}   # once
    g = _node()
    _airborne_rejoin(g)
    g.attach_pending[0] = False
    g._newcomers_out_on_land('LAND')
    assert g._take_back_pub.msgs == [] and g._departed_hold == {}               # folded in: a carrier now


def test_before_his_takeoff_it_stays_on_its_mat_with_no_reference():
    f = _node()
    f._newcomers_out_on_land('LAND')
    assert f._take_back_pub.msgs == [True] and f._departed_hold == {}
    _tick(f)
    assert f.published == [] and f._magnet_cmd_pub.msgs == []


def test_on_the_floor_after_his_takeoff_it_lands_where_it_is():
    f = _node()
    f._his_phase_cb(String(data='TAKEOFF'))
    f._take_back(0, 'LAND')
    assert np.allclose(f._departed_hold[3], [0.0, 1.4, 0.1]) and 3 not in f._departed_wait
    assert 3 not in f._departed_step and 3 in f._land_alone


def test_the_magnet_goes_off_only_once_the_span_began():
    """On the rig one coil holds his ball until the drop (critic round 3, finding 6)."""
    f = _node()
    f._his_phase_cb(String(data='TRANSIT_TO_DROP_POINT'))
    f.attach_pos[0] = np.array([0.0, 0.6, 2.7])               # carrying the ball over the ring
    f._take_back(0, 'LAND')
    f._land_to_ground = True
    for _ in range(20):
        _tick(f)
    f._magnet_cmd_cb(String(data='ON'))                       # a stray ON in LAND: refused, but no OFF
    f._magnet_attached_cb(Bool(data=True))                    # refused weld: no OFF either
    assert f._magnet_cmd_pub.msgs == [] and f._take_back_pub.msgs == [True]
    g = _node()
    _airborne_rejoin(g)
    _tick(g)
    g._take_back(0, 'LAND')
    for _ in range(int(0.5 * HZ) + 1):
        _tick(g)
    assert g._magnet_cmd_pub.msgs[:2] == ['ON', 'OFF'] and g._magnet_cmd_pub.msgs.count('OFF') >= 2


def test_ball_weight_is_in_drone_3s_references_while_it_carries_it():
    b = 0.1 * 9.81 / 0.55
    f = _node()
    f._his_phase_cb(String(data='TAKEOFF'))
    f._ball_cb(Bool(data=True))
    _tick(f)
    nodes = f.published[-1][1]
    assert np.allclose(nodes[0][2], [0.0, 0.0, 9.81 + b]) and np.allclose(nodes[0][3], [0.0, 0.0, -b])
    f.attach_pos[0] = np.array([0.0, 0.6, 2.7])
    f._take_back(0, 'LAND')
    _tick(f)
    assert np.allclose(f.published[-1][1][0][2], [0.0, 0.0, 9.81 + b])           # the take-back's hold too
    f._ball_cb(Bool(data=False))
    _tick(f)
    assert np.allclose(f.published[-1][1][0][2], [0.0, 0.0, 9.81]) and np.allclose(f.published[-1][1][0][3], 0.0)


# ── the lifecycle gate ───────────────────────────────────────────────────────────────────────────────────────

def test_a_disarm_of_our_tracker_while_his_stream_flies_takes_nothing_out():
    f = _node()
    _airborne_rejoin(f)
    _tick(f)
    f._newcomer_arming_cb(Bool(data=True), 0)
    f._newcomer_arming_cb(Bool(data=False), 0)
    assert f._newcomer_out == {} and f._magnet_cmd_pub.msgs == ['ON']
    f._mux_state_cb(String(data='ours'))
    f._newcomer_arming_cb(Bool(data=True), 0)
    f._newcomer_arming_cb(Bool(data=False), 0)
    assert f._newcomer_out == {0: 'fault'} and f._magnet_cmd_pub.msgs[-1] == 'OFF'


# ── the fold-in ──────────────────────────────────────────────────────────────────────────────────────────────

class _Solver:
    N, dt, last_X = 20, 0.1, np.zeros((3, 3))


def _fold_in_node():
    from mpc_planner.geometry import attach_points
    f = _node(quad=(0.0, 0.225, 1.02 + 0.59))
    _airborne_rejoin(f, z=1.61)
    f.attach_pos[0] = np.array([0.0, 0.225, 1.61])
    rho3 = attach_points(3, 0.225, 0.02, [270.0, 30.0, 150.0])
    f.load_state = np.r_[0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, np.zeros(6)]
    blends = []
    f.__dict__.update(
        n_net=4, slot2drone=[0, 1, 2], rho=list(rho3), cable_len_i=[0.55] * 3, cable_len=0.55, _attach_cable_len=0.59,
        _attach_body_to_tip=0.59, attach_z=0.02, psi0=0.0, _ocp_attached={}, _weld_time={}, detached=[False] * 4,
        drone_vel=[np.zeros(3)] * 3, _start_attached=False, start_taut=False, creep=None, _reconfig_mode='ocp',
        _attach_t_start_new=0.1, _attach_datum_shift=False, _attach_blend_balanced=True, _hold_mode='timed',
        _reconfig_hold_s=1.5, solver=_Solver(), blends=blends,
        refs=SimpleNamespace(_t_nom=[0.86 * 9.81 / 3 / np.sin(np.radians(60.0))] * 3))
    f._prime_solver = lambda: None
    f._log_resize_inputs = lambda *a: None

    def resize(n, ids, rho=None, cable_lengths=None):
        f.n, f.slot2drone, f.rho, f.cable_len_i = n, list(ids), list(rho), list(cable_lengths)
        f.dyn = SimpleNamespace(rho=list(rho), m=0.86, g=9.81)
        f._s_nom = [np.array([-r[0], -r[1], -0.9]) / np.linalg.norm([r[0], r[1], 0.9]) for r in rho]
        f.refs = SimpleNamespace(_s_nom=None, _t_nom=None, balanced_blend=None,
                                 start_blend=lambda s, t, T: blends.append(list(t)))
        return True
    f.resize_fleet = resize
    return f


def test_the_fold_in_seeds_quad4_at_its_ocp_share_and_drops_any_departed_entry():
    f = _fold_in_node()
    f._departed_hold[3] = np.array([0.0, 0.225, 1.61])        # a stray departed entry must not outlive the weld
    f._land_alone.add(3)
    f._magnet_attached_cb(Bool(data=True))
    assert f.attach_pending == [False] and f._ocp_attached == {3: True}
    t_from = f.blends[-1]
    assert t_from[-1] == pytest.approx(f.refs._t_nom[-1]) and t_from[-1] > 1.0   # not attach_t_start_new 0.1
    assert 3 not in f._departed_hold and 3 not in f._land_alone
    g = _fold_in_node()
    g._partner_contact = False
    g._newcomer_lifecycle = g._newcomer_fault_land = False
    g._magnet_attached_cb(Bool(data=True))
    assert g.blends[-1][-1] == pytest.approx(0.1)                                 # off: today's seed
    h = _fold_in_node()
    h._contact_fold_seed = 'start'                                                 # E1b: 'ocp' tilted the ring 8 deg
    h._magnet_attached_cb(Bool(data=True))
    assert h.blends[-1][-1] == pytest.approx(0.1)

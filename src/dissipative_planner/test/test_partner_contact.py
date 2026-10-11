"""Combined M1 contact attach, our planner (card 2026-10-10_combined_m1_contact rev 4, critic round 3 finding 1):
his mission flies QUAD4 until our check confirms the attach. Our drone-3 tracker is warm on its measured body from his
takeoff, the ring magnet span starts once he rejoins with QUAD4 airborne and the ring weld free, our own ON never
starts an approach or holds the ring, and with no attach (his give-up, our LAND) we take QUAD4 back: a fixed hold,
radially out (over the middle along plate 9's azimuth) and down alone. All behind partner_contact (off by default)."""
import re
import time
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
           '_abort_newcomer', '_attach_ocp', '_capture_weld_rho', '_start_weld_joins_creep', '_do_attach',
           '_take_back_plan', '_take_back_tick', '_take_back_next', '_ring_moves', '_ring_plan', '_guard_setup',
           '_guard_read', '_guard_check', '_guard_follow', '_guard_ring_fresh', '_weld_rho_of', '_guard_why')


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
        dyn=SimpleNamespace(g=9.81), N=20, dt=0.1, drone_mass=0.55, attach_radius=0.225, _attach_body_to_tip=0.59,
        _magnet_cmd_pub=_Pub(), _take_back_pub=_Pub(), traj=SimpleNamespace(kind='hover', radius=0.0),
        hover_xy=None, _departed_gi=False, _rest_z={3: 0.1}, _departed_sent=set(), _departed_pubs={},
        _attach_target=np.array([0.0, 0.225, 1.05]), _attach_warm_radius=1.0,
        net=SimpleNamespace(rho=[None, None, None, np.array([0.0, 0.225, 0.02])]),
        _his_phase='', _his_took_off=False, _mux_state='partner', _joint_free=None, _ball_on=False,
        _ball_mass=0.1, _span_t=None, _taken_back=False, traj_t=0.0, _comove={}, _attach_vel=[None],
        _traj_running=False, _release_guard=False, _tip_now=None)
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
    rise = 1.5 + 0.4                                           # tip down at the ring: up over the carriers first (R1543, R1554)
    assert np.allclose(f._departed_step[3], [0.5, 0.5, rise])
    out = f._departed_next[3]
    assert np.isclose(np.hypot(out[0], out[1]), DEPARTED_CLEAR_R) and np.isclose(out[0], out[1])
    assert out[2] == pytest.approx(rise) and 3 in f._land_alone
    f.published.clear()
    for _ in range(int(round(NEWCOMER_RELEASE_S * HZ))):
        _tick(f)
    assert all(np.allclose(n[0][0], [0.5, 0.5, 1.6]) for _, n in f.published)   # the fixed hold
    for _ in range(3):
        _tick(f)
    p = f.published[-1][1][0][0]
    assert np.allclose(p[:2], [0.5, 0.5]) and p[2] > 1.6                          # rising, not yet out
    g = _node()
    _airborne_rejoin(g, z=1.9)                                 # its tip already clear of the ring
    g.attach_pos[0] = np.array([0.5, 0.5, 1.9])
    g._give_up_cb(Bool(data=True))
    step = g._departed_step[3]
    assert np.isclose(np.hypot(step[0], step[1]), DEPARTED_CLEAR_R) and step[2] == pytest.approx(1.9)


def test_over_the_middle_it_steps_out_along_its_plate_azimuth():
    f = _node()
    _airborne_rejoin(f)
    f.attach_pos[0] = np.array([0.05, -0.05, 1.9])             # r 0.07 < 0.225, on the far side of plate 9
    f._take_back(0, 'LAND')
    assert np.allclose(f._departed_step[3], [0.0, DEPARTED_CLEAR_R, 1.9])
    g = _node()
    _airborne_rejoin(g)
    g._attach_target = None                                    # no live target: the reserved slot's plate
    g.attach_pos[0] = np.array([0.05, -0.05, 1.9])
    g._take_back(0, 'LAND')
    assert np.allclose(g._departed_step[3], [0.0, DEPARTED_CLEAR_R, 1.9])


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


def test_his_give_up_at_the_pickup_takes_it_back_with_no_ring_magnet_command():
    """His bounded pickup retry gives up before the rejoin (R1493, R1497 stalled there): no ball, no span."""
    f = _node()
    f._his_phase_cb(String(data='TAKEOFF'))
    f._his_phase_cb(String(data='MAGNET_ATTACH_WAIT'))
    f._joint_state_cb(String(data='detached'))
    f.attach_pos[0] = np.array([1.0, 1.6, 0.86])              # risen clear of his ball
    _tick(f)
    f._give_up_cb(Bool(data=True))
    f._his_phase_cb(String(data='TAKEN_BACK'))
    assert f._take_back_pub.msgs == [True] and f._newcomer_out == {0: 'his give-up'} and 3 in f._land_alone
    assert np.allclose(f._departed_hold[3], [1.0, 1.6, 0.86]) and 3 not in f._departed_step
    for _ in range(int(round(NEWCOMER_RELEASE_S * HZ)) + 10):
        _tick(f)
    p = f.published[-1][1][0][0]
    assert np.allclose(p[:2], [1.0, 1.6]) and p[2] < 0.86 - 0.2  # down where it is, alone; the carriers fly on
    assert f._magnet_cmd_pub.msgs == [] and f._span_t is None and not f._land_to_ground


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
    f._contact_fold_seed = 'ocp'
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
    h = _fold_in_node()                                                            # the default since R1479-R1490
    h._magnet_attached_cb(Bool(data=True))
    assert h.blends[-1][-1] == pytest.approx(0.1)


def test_with_the_ball_on_the_take_back_clears_by_the_rod_too():
    """R1482: taken back at r 1.3 m with the ball on, QUAD4 landed beside the ball at r 1.03, 0.33 m from carrier 1."""
    f = _node()
    f._his_phase_cb(String(data='TRANSIT_TO_DROP_POINT'))
    f._ball_on = True
    f.attach_pos[0] = np.array([0.7, 1.09, 2.93])
    f._take_back(0, 'LAND')
    step = f._departed_step[3]
    assert np.linalg.norm(step[:2]) == pytest.approx(1.2 + f._attach_body_to_tip) and step[2] == pytest.approx(2.93)
    g = _node()
    g._his_phase_cb(String(data='TRANSIT_TO_DROP_POINT'))
    g.attach_pos[0] = np.array([0.7, 1.09, 2.93])            # no ball: r 1.3 is already clear
    g._take_back(0, 'LAND')
    assert 3 not in g._departed_step


def test_rig_span_waits_for_his_ball_released():
    """Rig (critic F9, contact_span_wait_ball): no span ON while /tejen/object_attached reads true; off (the twin), as
    before."""
    for wait, expect in ((True, []), (False, ['ON'])):
        f = _node()
        f._span_wait_ball = wait
        f._his_phase_cb(String(data='TAKEOFF'))
        f._ball_cb(Bool(data=True))
        f.attach_pos[0] = np.array([0.0, 0.6, 1.6])
        f._joint_state_cb(String(data='detached'))
        f._his_phase_cb(String(data='TRANSIT_TO_REATTACH'))
        _tick(f)
        assert f._magnet_cmd_pub.msgs == expect
        f._ball_cb(Bool(data=False))
        _tick(f)
        assert f._magnet_cmd_pub.msgs[:1] == ['ON']


# ── the full M1: QUAD4 welded on plate 9 from spawn, detached, stepped out and released to his mission ───────────

STEP_OUT = np.array([0.0, 1.0, 1.78])     # 0.5 m out and 0.3 m up from its detach point (partner_release_out/up_m)


def _welded_node(contact=True):
    """After the detach: QUAD4 folded in at the creep, detached, its step-out hold at the detach point."""
    f = _node(quad=(0.0, 0.5, 1.48), contact=contact)
    f.attach_pending[0] = False
    hold = np.array([0.0, 0.5, 1.48])
    f._departed_hold[3] = hold.copy()
    f.drone_pos.append(hold.copy())
    f.drone_vel = [np.zeros(3)] * 4
    f._release_state = {3: {'target': STEP_OUT.copy(), 'released': False, 'quiet': 0.0, 'sent': None}}
    f._release_pub = _Pub()
    f._contact_released = False
    f._partner_handed = False
    f._mux_state = 'ours'                  # start_ours: ours from boot until his stream flies it
    f._his_phase_cb(String(data='WAIT_FOR_TAKEOFF'))
    for name in ('_partner_release_tick', '_contact_release_tick'):
        setattr(f, name, MethodType(getattr(D, name), f))
    return f


def _wtick(f, follow=True):
    """_plan_inner's order; QUAD4 tracks drone 3's reference perfectly while ours flies it."""
    n0 = len(f.published)
    f._contact_release_tick()
    if f._departed_hold:
        f._publish_departed_refs()
    f._newcomer_off_tick()
    f._partner_release_tick()
    f._publish_pending_attach_refs()
    f._contact_span_tick()
    f.now[0] += 1.0 / HZ
    mine = [d for d, _ in f.published[n0:] if d == 3]
    assert len(mine) <= 1                                       # never two references for drone 3 in a tick
    if follow and mine:
        p = f.published[-1][1][0][0]
        f.drone_pos[3] = p.copy()
        f.attach_pos[0] = p.copy()
    return mine


def _release(f):
    for _ in range(200):
        _wtick(f)
        if f._release_state[3]['released']:
            return
    raise AssertionError('never released')


def test_the_released_quad4_keeps_its_step_out_hold_until_his_stream_flies_it_then_goes_warm():
    """Critic round 1, finding 1: the step-out hold must give way to the warm reference on QUAD4's measured body
    once his stream flies it, or the switch at the attach would hand our tracker a reference 1-3 m away."""
    f = _welded_node()
    _release(f)
    assert f.attach_pending == [True] and f._contact_released
    assert any('PARTNER RELEASE' in ln for ln in f.log.lines)
    for _ in range(11):
        assert _wtick(f) == [3]                                 # still the fixed hold, ours flies it
    assert np.allclose(f.published[-1][1][0][0], STEP_OUT) and 3 in f._departed_hold
    assert f._release_pub.msgs.count('{"running": true}') >= 1
    f._his_phase_cb(String(data='TAKEOFF'))                     # his mission starts; his MPC not flying yet
    _wtick(f)
    assert 3 in f._departed_hold and np.allclose(f.published[-1][1][0][0], STEP_OUT)
    f._mux_state_cb(String(data='partner'))                     # his first flying command
    f.attach_pos[0] = np.array([0.1, 1.1, 1.9])                 # he moves it
    assert _wtick(f, follow=False) == [3]
    assert 3 not in f._departed_hold and np.allclose(f.published[-1][1][0][0], [0.1, 1.1, 1.9])
    f.attach_pos[0] = np.array([0.6, 1.4, 2.5])
    assert _wtick(f, follow=False) == [3]
    assert np.allclose(f.published[-1][1][0][0], [0.6, 1.4, 2.5])   # the warm reference follows him
    assert any('flies on his stream' in ln for ln in f.log.lines)


def test_land_before_the_release_cancels_it_and_it_lands_as_a_departed_drone():
    f = _welded_node()
    for _ in range(10):
        _wtick(f)                                               # stepping out
    f._newcomers_out_on_land('LAND')                            # not pending: no take-back
    f._land_to_ground = True
    zs = []
    for _ in range(150):
        _wtick(f)
        zs.append(f.published[-1][1][0][0][2])
    assert f._release_pub.msgs == [] and f._take_back_pub.msgs == [] and not f._contact_released
    assert f.attach_pending == [False] and not f._release_state[3]['released']
    p = f.published[-1][1][0][0]
    assert np.hypot(p[0], p[1]) == pytest.approx(DEPARTED_CLEAR_R, abs=0.05)   # out of the ring's way, then down
    assert zs[-1] < zs[0] - 0.5
    assert any('PARTNER RELEASE of drone 4 (/drone_3) cancelled' in ln for ln in f.log.lines)


def test_land_after_the_release_before_his_stream_flies_it_is_a_take_back_where_it_hovers():
    f = _welded_node()
    _release(f)
    f._newcomers_out_on_land('LAND')
    assert f._take_back_pub.msgs == [True] and f._newcomer_out == {0: 'LAND'}
    assert np.allclose(f._departed_hold[3], STEP_OUT, atol=0.03) and 3 in f._land_alone   # held where it is
    step = f._departed_step[3]
    assert step[1] == pytest.approx(DEPARTED_CLEAR_R) and step[2] == pytest.approx(1.78, abs=0.03)
    f._land_to_ground = True
    n_rel = len(f._release_pub.msgs)
    f._his_phase_cb(String(data='TAKEOFF'))
    f._mux_state_cb(String(data='partner'))                     # even if his stream got it: ours keeps the hold
    for _ in range(30):
        _wtick(f)
    assert 3 in f._departed_hold and len(f._release_pub.msgs) == n_rel
    assert f._magnet_cmd_pub.msgs == []                         # no span began: no OFF (rig: the coil is his)


def test_his_give_up_after_the_warm_hand_over_takes_it_back_and_stops_the_release():
    f = _welded_node()
    _release(f)
    f._his_phase_cb(String(data='TAKEOFF'))
    f._mux_state_cb(String(data='partner'))
    _wtick(f)
    f.attach_pos[0] = np.array([0.0, 0.6, 1.6])
    f._his_phase_cb(String(data='TRANSIT_TO_REATTACH'))
    f._joint_state_cb(String(data='detached'))                  # our detach freed the ring weld
    _wtick(f, follow=False)
    assert f._magnet_cmd_pub.msgs == ['ON']                     # the span from his rejoin, as the subtest
    n_rel = len(f._release_pub.msgs)
    f._give_up_cb(Bool(data=True))
    assert f._take_back_pub.msgs == [True] and np.allclose(f._departed_hold[3], [0.0, 0.6, 1.6])
    assert f._magnet_cmd_pub.msgs[-1] == 'OFF'
    for _ in range(20):
        _wtick(f, follow=False)
    assert len(f._release_pub.msgs) == n_rel and 3 in f._departed_hold


def test_off_the_partner_release_is_todays():
    """partner_contact off (the legacy partner_attached flow): the step-out hold stays until his hand-off, and
    LAND does not cancel the release (its tick keeps pulling the hold to the release point, as before)."""
    f = _welded_node(contact=False)
    _release(f)
    f._his_phase_cb(String(data='TAKEOFF'))
    f._mux_state_cb(String(data='partner'))
    for _ in range(5):
        _wtick(f, follow=False)
    assert 3 in f._departed_hold and np.allclose(f.published[-1][1][0][0], STEP_OUT)
    g = _welded_node(contact=False)
    _wtick(g)
    g._land_to_ground = True
    for _ in range(30):
        _wtick(g)
    assert 'cancelled' not in g._release_state[3] and not g._contact_released
    assert not any('cancelled' in ln for ln in g.log.lines)


def test_the_rejoin_fold_in_of_the_welded_start_is_the_subtests():
    """Welded at the start (start_attached) and folded in at the creep; the rejoin after takeoff folds in from the
    measured tip with the 'start' seed, not the plate-at-start rule."""
    f = _fold_in_node()
    f._start_attached = True
    f.takeoff_seen = True
    f._departed_hold[3] = np.array([0.0, 1.0, 1.78])            # a stale entry must not outlive the weld
    f._magnet_attached_cb(Bool(data=True))
    assert f.attach_pending == [False] and f._ocp_attached == {3: True} and 3 not in f._departed_hold
    assert f.blends[-1][-1] == pytest.approx(0.1)
    assert not any('welded at start' in ln for ln in f.log.lines)


# ── the tilt cut (card 2026-10-11_quad4_tilt_cut, critic round 1 H1/H3/M4) ─────────────────────────────────────

TC_METHODS = ('_tilt_fault_cb', '_off_the_load', '_tilt_tracker_cb', '_release_seen_tick', '_tip_now_cb')


def _tc(f, mux='partner'):
    """Add the tilt cut's state to a test node."""
    f.__dict__.update(_tilt_faulted=set(), _handover_blocked=False, _tilt_armed={}, _handover_block_pub=_Pub(),
                      detached=[False] * 4, _release_seen={}, _tip_now=None, _mux_state=mux)
    for name in TC_METHODS:
        setattr(f, name, MethodType(getattr(D, name), f))
    return f


def _in_his_transit():
    f = _tc(_node())
    f._his_phase_cb(String(data='TAKEOFF'))
    f._his_phase_cb(String(data='TRANSIT_TO_DROP_POINT'))
    f.attach_pos[0] = np.array([0.99, 1.54, 2.86])            # R1491, 2 s into his transit
    _tick(f)
    return f


def test_a_tilt_fault_off_the_ring_takes_quad4_out_with_no_take_back_and_no_references():
    f = _in_his_transit()
    f._tilt_fault_cb(Bool(data=True))
    assert f._newcomer_out == {0: 'fault'} and f._tilt_faulted == {0}
    assert f._magnet_cmd_pub.msgs[-1] == 'OFF'                 # the cut drone falls with or without the coil
    f.published.clear()
    for _ in range(5):
        _tick(f)
    assert f.published == []                                   # no warm reference, no departed hold
    f._newcomers_out_on_land('LAND')
    f._give_up_cb(Bool(data=True))
    assert f._take_back_pub.msgs == [] and f._departed_hold == {}
    assert f._departed_down()                                  # the landed gate does not wait for it


@pytest.mark.parametrize('first', ['LAND', 'give-up'])
def test_a_fault_after_the_take_back_overrides_it_and_clears_the_departed_state(first):
    """H3: the manager's LAND or his give-up can reach us before the fault."""
    f = _in_his_transit()
    if first == 'LAND':
        f._newcomers_out_on_land('LAND')
    else:
        f._give_up_cb(Bool(data=True))
    assert f._take_back_pub.msgs == [True] and 3 in f._departed_hold and 3 in f._land_alone
    f._tilt_fault_cb(Bool(data=True))
    assert f._newcomer_out == {0: 'fault'} and f._departed_hold == {} and 3 not in f._land_alone
    assert 3 not in f._departed_step and 3 not in f._departed_wait and f._departed_down()


def test_a_weld_after_the_cut_is_refused():
    f = _in_his_transit()
    f._tilt_fault_cb(Bool(data=True))
    f._magnet_attached_cb(Bool(data=True))
    assert f.attach_pending == [True] and any('refused' in ln for ln in f.log.lines)


def test_on_the_ring_the_fault_is_a_carrier_fault_and_changes_nothing_here():
    f = _in_his_transit()
    f.attach_pending[0] = False                                # folded in
    f._tilt_fault_cb(Bool(data=True))
    assert f._newcomer_out == {} and f._tilt_faulted == set()
    assert any('on the ring' in ln for ln in f.log.lines)


def test_while_ours_flies_it_the_fault_is_left_to_its_disarm():
    """Ours steps the welded start out (or QUAD4 is down, disarmed alone): its own tracker's disarm decides
    (_newcomer_arming_cb with welded_on_load, else today's stop)."""
    f = _tc(_welded_node(), mux='ours')
    f.detached[3] = True
    for _ in range(5):
        _wtick(f)                                              # stepping out
    f._tilt_fault_cb(Bool(data=True))
    assert f._newcomer_out == {} and 3 in f._departed_hold and f._tilt_faulted == set()


def test_a_cut_after_the_release_cancels_the_rest_of_the_release():
    f = _tc(_welded_node(), mux='ours')
    f.detached[3] = True
    _release(f)
    f._mux_state_cb(String(data='partner'))
    f._tilt_fault_cb(Bool(data=True))
    n = len(f._release_pub.msgs)
    for _ in range(30):
        _wtick(f)
    assert f._newcomer_out == {0: 'fault'} and len(f._release_pub.msgs) == n and 3 not in f._departed_hold


def test_our_tracker_latching_while_he_flies_it_blocks_the_hand_over_once():
    """H1: the mux would wait for ever for a tracker that will not fly again."""
    f = _in_his_transit()
    f.takeoff_seen = True
    f._tilt_tracker_cb(Bool(data=True))
    f._tilt_tracker_cb(Bool(data=False))
    assert f._handover_blocked and f._handover_block_pub.msgs == [True]
    f._tilt_tracker_cb(Bool(data=True))
    f._tilt_tracker_cb(Bool(data=False))
    assert f._handover_block_pub.msgs == [True]
    assert any('no take-back or hand-over is possible' in ln for ln in f.log.lines)
    g = _in_his_transit()                                      # ours flying it: its disarm is a fault of ours
    g._mux_state = 'ours'
    g._tilt_tracker_cb(Bool(data=True))
    g._tilt_tracker_cb(Bool(data=False))
    assert not g._handover_blocked
    h = _in_his_transit()                                      # cut: the fault path owns it
    h._tilt_fault_cb(Bool(data=True))
    h._tilt_tracker_cb(Bool(data=True))
    h._tilt_tracker_cb(Bool(data=False))
    assert not h._handover_blocked


def test_welded_on_load_goes_false_only_once_the_tip_has_left_the_plate():
    """M4: on the rig the tether is slack, so on_load False waits for the tip itself to leave plate 9."""
    f = _tc(_welded_node(), mux='ours')
    pub = _Pub()
    f._on_load_pubs, f._on_load_announced = {3: pub}, True
    f._release_seen[3] = None                                  # what the detach does under welded_on_load
    plate = np.array([0.0, 0.225, 1.04])
    for dz in (0.0, 0.01, 0.03, 0.045):                        # the body steps out, the tip still on the plate
        f._tip_now_cb(SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(
            x=plate[0], y=plate[1] + dz, z=plate[2]))))
        f._release_seen_tick()
    assert pub.msgs == [] and 3 in f._release_seen
    f._tip_now_cb(SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(x=0.0, y=0.29, z=1.06))))
    f._release_seen_tick()
    assert pub.msgs == [False] and 3 not in f._release_seen


def test_a_take_back_with_the_tip_at_the_ring_rises_before_it_steps_out():
    """R1543: taken back at ATTACH_READY, the step out at its height dragged the rod's tip through the rim."""
    f = _node(quad=(0.0, 0.3, 1.0 + 0.59 + 0.01))               # body over plate 9, tip 1 cm above the ring
    f._his_phase_cb(String(data='TAKEOFF'))
    f._take_back(0, 'his give-up')
    rise = max(1.0 + 0.59 + 0.15, 1.5 + 0.4)                    # clear of the rim and over the carriers (R1554)
    assert np.allclose(f._departed_step[3], [0.0, 0.3, rise])            # straight up first
    assert np.allclose(f._departed_next[3][2], rise)                     # then out at that height
    assert float(np.linalg.norm(f._departed_next[3][:2])) >= 1.2 - 1e-6
    assert any('clear of the ring' in m for m in f.log.lines)
    for _ in range(int(20 * HZ)):
        _tick(f)
    p = f._departed_hold[3]
    assert float(np.linalg.norm(p[:2])) >= 1.2 - 0.03                   # it got out


def test_a_take_back_high_above_the_ring_steps_out_at_its_height():
    f = _node(quad=(0.0, 0.6, 2.7))                               # in his transit, over the ring
    f._his_phase_cb(String(data='TRANSIT_TO_DROP_POINT'))
    f._take_back(0, 'LAND')
    assert np.allclose(f._departed_step[3][2], 2.7) and 3 not in f._departed_next


def test_the_take_back_rise_clears_the_highest_carrier():
    """R1554: risen only clear of the rim (1.77 m), the step out passed 0.24 m over a carrier at 1.54 m."""
    f = _node(quad=(0.0, 0.3, 1.6))
    f.drone_pos[1] = np.array([0.43, 0.25, 1.8])                 # one carrier higher than the rest
    f._his_phase_cb(String(data='TAKEOFF'))
    f._take_back(0, 'his give-up')
    assert f._departed_step[3][2] == pytest.approx(1.8 + 0.4)


def test_a_fault_during_the_rising_take_back_clears_both_legs():
    """The merge of the tilt cut with the rise (m1-tiltcut4): the fault overrides a take-back that is still rising."""
    f = _tc(_node(quad=(0.0, 0.3, 1.0 + 0.59 + 0.01)))
    f._his_phase_cb(String(data='TAKEOFF'))
    f._take_back(0, 'his give-up')
    assert 3 in f._departed_step and 3 in f._departed_next
    f._tilt_fault_cb(Bool(data=True))
    assert f._newcomer_out == {0: 'fault'} and 3 not in f._departed_step and 3 not in f._departed_next
    assert f._departed_hold == {} and f._departed_down()


# ── the co-moving take-back (card rev 10 §3b with the critic's round 4 response; TB13, F1, F5, F6, F7, F9) ──────────

from mpc_planner.load_trajectory import LoadTrajectory  # noqa: E402
from dissipative_planner.dissipative_node import (  # noqa: E402
    GUARD_LAND_WAIT_S, GUARD_RISE_ABOVE_SLACK_M, GUARD_SLACK_M, GUARD_WAIT_S, TAKE_BACK_ACCEL, TAKE_BACK_REACHED_M,
    ZONE_RISE_FAR_M)

A_TICK = TAKE_BACK_ACCEL / HZ + 1e-9          # the most the take-back's velocity may change in one tick
AZ = (270.0, 30.0, 150.0)


def _place_ring(f):
    """The ring and the carriers on the plan (the kinematic formation)."""
    c, _ = f._ring_plan(f.traj_t)
    f.load_state[0:2] = c
    for i, a in enumerate(AZ):
        f.drone_pos[i] = np.array([c[0] + 0.5 * np.cos(np.radians(a)), c[1] + 0.5 * np.sin(np.radians(a)), 1.53])


def _orbit_node(rel=(0.0, 0.3), z=1.0 + 0.59 + 0.02, vrel=(0.0, 0.0, 0.0), t=10.0, ball=False):
    """On the circle (orbit, r 0.5 at 0.125 m/s about hover + (0, 0.5)), QUAD4 at the ring-relative xy `rel`, moving
    with the ring plus `vrel`; the clock ran on the last tick."""
    f = _tc(_node())
    f.traj = LoadTrajectory('orbit', 0.125, 0.0, 0.5)
    f.hover_xy = (0.0, 0.0)
    f.traj_t, f._t_tick, f._traj_running = t, t - 0.1, True
    f._clock_runs = True
    f.stages = []
    _place_ring(f)
    f._his_phase_cb(String(data='TAKEOFF'))
    f._ball_on = ball
    c, vc = f._ring_plan(t)
    f.attach_pos[0] = np.array([c[0] + rel[0], c[1] + rel[1], z])
    f._attach_vel[0] = np.r_[vc, 0.0] + np.asarray(vrel, float)
    return f


def _otick(f, low=0.0):
    """_plan_inner's order on the circle: the clock flag, the references, then the planner's advance (none in LAND
    or while the test holds the clock). QUAD4 flies its reference, `low` under it."""
    f._traj_running = f.traj_t > f._t_tick + 1e-9
    f._t_tick = f.traj_t
    n0 = len(f.published)
    _tick(f)
    if f._clock_runs and not f._land_to_ground:
        f.traj_t += 1.0 / HZ
    _place_ring(f)
    mine = [n for d, n in f.published[n0:] if d == 3]
    assert len(mine) <= 1
    if mine and 3 in f._departed_hold:
        f.attach_pos[0] = mine[0][0][0] - np.array([0.0, 0.0, low])
    if 3 in f._comove:
        f.stages.append(f._comove[3]['stage'])
    return mine[0] if mine else None


def _fly(f, ticks, on_tick=None):
    """[(traj_t, node 0 position, node 0 velocity)] of drone 3's reference per tick."""
    out = []
    for k in range(ticks):
        if on_tick:
            on_tick(f, k)
        t = f.traj_t
        n = _otick(f)
        if n is not None:
            out.append((t, n[0][0].copy(), n[0][1].copy()))
    return out


def _rel(f, t, p):
    return p[:2] - f._ring_plan(t)[0]


def _assert_bounded(track):
    for (_, _, v0), (_, _, v1) in zip(track, track[1:]):
        assert float(np.linalg.norm(v1 - v0)) <= A_TICK


def test_zone_c_moves_with_the_ring_rises_steps_out_decouples_clears_the_path_and_lands():
    f = _orbit_node()
    f._give_up_cb(Bool(data=True))
    st = f._comove[3]
    assert st['zone'] == 'c' and st['coupled']
    rise = 1.53 + 0.4                                          # c6f2032's rise: the highest carrier + 0.4
    assert np.allclose(f._departed_step[3][2], rise) and np.allclose(f._departed_next[3][2], rise)
    assert any('zone c, moving with the ring' in ln for ln in f.log.lines)
    track = _fly(f, 400)
    _assert_bounded(track)
    t0, p0, _ = track[0]
    hold = [_rel(f, t, p) for t, p, _ in track[:5]]
    assert all(np.allclose(h, hold[0], atol=0.005) for h in hold)       # the release wait: still in the ring frame
    i_dc = next(i for i, ln in enumerate(f.log.lines) if 'decoupled from the ring frame' in ln)
    assert any('out past the ring\'s path to' in ln for ln in f.log.lines[i_dc:])
    # up first, then out in the ring frame: at the step-out the ring-relative radius is the step-out radius
    rels = [(float(np.linalg.norm(_rel(f, t, p))), p[2]) for t, p, _ in track]
    first_out = next(i for i, (r, z) in enumerate(rels) if r > 0.35)
    assert rels[first_out][1] > rise - 0.03                    # it rose before it moved sideways
    i_clear = f.stages.index('clear')                          # the decouple: out at 1.2 m in the ring frame
    assert rels[i_clear][0] == pytest.approx(1.2, abs=0.03) and rels[i_clear][1] == pytest.approx(rise, abs=0.02)
    centre = np.array([0.0, 0.5])
    assert float(np.linalg.norm(track[-1][1][:2] - centre)) >= 1.5 - 0.03   # past the path, then down alone
    assert 3 not in f._comove and 3 in f._departed_landing


def test_zone_a_far_from_the_path_is_world_fixed_todays_path_with_the_ramp():
    f = _orbit_node(rel=(1.3, 1.2), z=1.6)                     # 1.8 m from the path centre (r_clear 1.5)
    f._give_up_cb(Bool(data=True))
    assert f._comove[3]['zone'] == 'a' and not f._comove[3]['coupled']
    p_latch, v_latch = f.attach_pos[0].copy(), f._attach_vel[0].copy()
    track = _fly(f, 60)
    _assert_bounded(track)
    stop = p_latch + v_latch * np.linalg.norm(v_latch) / (2 * 0.2)
    assert np.allclose(track[-1][1][:2], stop[:2], atol=0.03)  # braked to its stopping point and stays there
    assert track[-1][1][2] < 1.6 - 0.2                          # down where it stopped


def test_zone_a_high_over_the_carriers_steps_out_world_fixed_then_clears_the_path():
    f = _orbit_node(rel=(-0.6, 0.0), z=2.6)                    # his transit: lowest point 2.01 > 1.53 + 0.15
    f._give_up_cb(Bool(data=True))
    st = f._comove[3]
    assert st['zone'] == 'a' and not st['coupled'] and f._departed_step[3][2] == pytest.approx(2.6, abs=0.01)
    _fly(f, 300)
    assert any('out past the ring\'s path to' in ln for ln in f.log.lines)


def test_zone_b_below_the_ring_lands_where_it_stops_with_no_sideways_move_and_no_rise():
    f = _orbit_node(rel=(0.6, 0.6), z=0.6)                     # his takeoff or pickup, under the ring's level
    f._attach_vel[0] = np.zeros(3)
    f._give_up_cb(Bool(data=True))
    assert f._comove[3]['zone'] == 'b' and 3 not in f._departed_step
    track = _fly(f, 40)
    xy0 = track[0][1][:2]
    assert all(np.allclose(p[:2], xy0, atol=1e-6) for _, p, _ in track)
    assert all(p[2] <= 0.6 + 1e-9 for _, p, _ in track) and track[-1][1][2] < 0.4


def test_zone_c_under_the_formation_descends_with_the_ring_then_lands():
    f = _orbit_node(rel=(0.55, 0.1), z=1.1)                    # tip 0.51 m, under the ring's level, 0.56 m out
    f._give_up_cb(Bool(data=True))
    st = f._comove[3]
    assert st['zone'] == 'c' and not st['clear'] and f._departed_step[3][2] == pytest.approx(0.7)
    track = _fly(f, 120)
    _assert_bounded(track)
    assert all(p[2] <= 1.1 + 1e-6 for _, p, _ in track)        # never up
    at = next(i for i, (_, p, _) in enumerate(track) if p[2] < 0.7 + TAKE_BACK_REACHED_M)
    rel = [_rel(f, t, p) for t, p, _ in track[:at]]
    assert all(np.allclose(r, rel[0], atol=0.02) for r in rel)   # down with the ring
    assert not any('out past the ring\'s path to' in ln for ln in f.log.lines)


def test_zone_c_far_from_the_ring_centre_rises_before_it_steps_out():
    f = _orbit_node(rel=(0.0, 1.0), z=1.3)                     # 1.0 m from the ring centre, tip under the ring level
    f._give_up_cb(Bool(data=True))
    assert f._comove[3]['zone'] == 'c' and f._departed_step[3][2] == pytest.approx(1.93)
    assert 1.0 >= ZONE_RISE_FAR_M


def test_the_ring_frame_is_continuous_across_the_latch():
    """F9: the frame from the planned ring point at the latch tick; no reference step at the latch."""
    f = _orbit_node()
    _airborne_rejoin(f, z=1.61)
    c, vc = f._ring_plan(f.traj_t)
    f.attach_pos[0] = np.array([c[0], c[1] + 0.3, 1.61])
    f._attach_vel[0] = np.r_[vc, 0.0]
    p_meas, v_meas = f.attach_pos[0].copy(), f._attach_vel[0].copy()
    f._give_up_cb(Bool(data=True))
    t, p, v = _fly(f, 1)[0]
    assert float(np.linalg.norm(v - v_meas)) <= A_TICK
    assert float(np.linalg.norm(p - p_meas)) <= float(np.linalg.norm(v_meas)) / HZ + 0.005
    track = _fly(f, 30)
    _assert_bounded(track)
    rel = [_rel(f, t, p) for t, p, _ in track]
    assert all(float(np.linalg.norm(b - a)) < 0.005 for a, b in zip(rel, rel[1:]))


def test_the_decouple_waits_on_the_reference_not_the_measured_body():
    """F5: with the measured body 5 cm under its reference all along, the decouple comes on the same tick."""
    ticks = []
    for low in (0.0, 0.05):
        f = _orbit_node()
        f._give_up_cb(Bool(data=True))
        n = 0
        for k in range(300):
            _otick(f, low=low)
            n += 1
            if any('decoupled' in ln for ln in f.log.lines):
                break
        ticks.append(n)
        st_goals = f._comove.get(3)
        assert st_goals is not None and not st_goals['coupled']
    assert ticks[0] == ticks[1] and ticks[0] < 300


def test_land_mid_take_back_stops_the_frame_through_the_bound_and_skips_the_path_clear():
    f = _orbit_node()
    f._give_up_cb(Bool(data=True))

    def land(f, k):
        if k == 25:                                            # rising, the frame moving
            f._land_to_ground = True
    track = _fly(f, 300, land)
    _assert_bounded(track)
    assert not any('out past the ring\'s path to' in ln for ln in f.log.lines)
    r = float(np.linalg.norm(track[-1][1][:2] - f.load_state[0:2]))
    assert r == pytest.approx(1.2, abs=0.06)                   # out to the step-out radius in the still frame, down
    assert track[-1][1][2] < 1.5


def test_a_trajectory_stop_mid_take_back_stops_the_frame_and_the_path_clear_still_follows():
    f = _orbit_node(t=27.0)                                    # the ring nearest -y: plate 9's way out is inward
    f._give_up_cb(Bool(data=True))

    def hold(f, k):
        if k == 20:
            f._clock_runs = False                              # a reconfiguration hold freezes traj_t
    track = _fly(f, 400, hold)
    _assert_bounded(track)
    assert any('out past the ring\'s path to' in ln for ln in f.log.lines)


def test_land_before_his_give_up_starts_the_take_back_with_the_frame_already_stopping():
    f = _orbit_node()
    f._newcomers_out_on_land('LAND')
    f._land_to_ground = True
    st = f._comove[3]
    assert st['zone'] == 'c'
    p_meas = f.attach_pos[0].copy()
    track = _fly(f, 200)
    _assert_bounded(track)
    v = np.r_[f._ring_plan(10.0)[1], 0.0]
    stop = p_meas + v * np.linalg.norm(v) / (2 * 0.2)          # braked from the ring's speed in the still frame
    assert np.allclose(track[6][1][:2], stop[:2], atol=0.02)
    assert not any('out past the ring\'s path to' in ln for ln in f.log.lines)


@pytest.mark.parametrize('first', ['LAND', 'give-up'])
def test_h3_a_tilt_fault_in_the_race_window_cancels_the_co_move_and_drone_3_gets_no_reference(first):
    f = _orbit_node()
    if first == 'LAND':
        f._newcomers_out_on_land('LAND')
    else:
        f._give_up_cb(Bool(data=True))
    _otick(f)                                                  # one co-moving reference went out
    assert 3 in f._comove
    f._tilt_fault_cb(Bool(data=True))                          # the mux still reads partner: the race window
    assert 3 not in f._comove and f._departed_hold == {} and 3 not in f._departed_step
    assert all(_otick(f) is None for _ in range(20))
    g = _orbit_node()
    g._tilt_fault_cb(Bool(data=True))                          # the fault first: no take-back at all
    g._give_up_cb(Bool(data=True))
    g._newcomers_out_on_land('LAND')
    assert g._comove == {} and all(_otick(g) is None for _ in range(10))


def test_the_new_state_is_dropped_with_the_departed_state():
    f = _orbit_node()
    f._give_up_cb(Bool(data=True))
    f._drop_departed(3)
    assert f._comove == {}


def test_the_transit_take_back_ramps_and_decides_from_its_stopping_point():
    """TB13 in hover (R1542: 0.34 m/s in his transit): no velocity step, and the step out is decided where it stops."""
    f = _node(quad=(0.0, 0.95, 2.9))
    f._his_phase_cb(String(data='TRANSIT_TO_DROP_POINT'))
    f._attach_vel[0] = np.array([0.0, 0.34, 0.0])               # outward: it stops at r 1.24, past 1.2
    f._take_back(0, 'LAND')
    assert 3 not in f._departed_step
    g = _node(quad=(0.0, 0.95, 2.9))
    g._his_phase_cb(String(data='TRANSIT_TO_DROP_POINT'))
    g._attach_vel[0] = np.array([0.0, -0.34, 0.0])              # inward: it stops at r 0.66, steps out from there
    g._take_back(0, 'LAND')
    assert g._departed_step[3][1] == pytest.approx(1.2)
    vs = []
    for _ in range(40):
        n0 = len(g.published)
        _tick(g)
        vs += [n[0][1] for d, n in g.published[n0:] if d == 3]
    assert float(np.linalg.norm(vs[0] - g._attach_vel[0])) <= A_TICK
    assert all(float(np.linalg.norm(b - a)) <= A_TICK for a, b in zip(vs, vs[1:]))


# ── the rig release guard (critic round 4, F7) ───────────────────────────────────────────────────────────────────

# s from the guard's failure to its follow at the slack: the LAND's first seconds are the ring's step to land_vel
# against the take-back's acceleration bound, from the guard's height
SETTLE_S = 2.7


def _guard_node(z=1.0, weld=None, body_over_tip=0.59):
    """The ring at z (plate 9 at z + 0.02, the ring's rest at 0), QUAD4 body_over_tip over plate 9 with its tip on
    it. weld: the launch's weld point (ring frame); None, the network slot (0, 0.225, 0.02)."""
    f = _node()
    f._weld_rho = None if weld is None else np.asarray(weld, float)
    tip = np.array([0.0, 0.0, z]) + f._weld_rho_of(0)
    f.__dict__.update(_release_guard=True, _fleet_cmd_pub=_Pub(), lift_z0=0.0, _touched_down=False,
                      _load_t=time.monotonic())
    f._load_down = MethodType(D._load_down, f)
    f.load_state[2] = z
    _airborne_rejoin(f, z=tip[2] + body_over_tip)
    f.attach_pos[0] = tip + np.array([0.0, 0.0, body_over_tip])
    _tick(f)                                                   # the span began
    f._tip_now = (f.now[0], tip)
    return f


def _set_tip(f, tip):
    f._tip_now = (f.now[0], np.asarray(tip, float))


def _gtick(f, ring_vz=0.0, fresh=True, tip=None):
    """One tick: the ring moves at ring_vz down to its rest, its pose fresh (or not), the coil holds the tip on plate 9
    (or the tip is at `tip`) and QUAD4 flies its reference, never below its rest. Returns (stage, drone 3's node 0 or
    None, plate 9)."""
    z = max(f.lift_z0, float(f.load_state[2]) + ring_vz / HZ)
    f.load_state[9] = (z - float(f.load_state[2])) * HZ
    f.load_state[2] = z
    plate = np.array([0.0, 0.0, z]) + f._weld_rho_of(0)
    _set_tip(f, plate if tip is None else tip)
    if fresh:
        f._load_t = time.monotonic()
    n0 = len(f.published)
    _tick(f)
    mine = [n for d, n in f.published[n0:] if d == 3]
    if mine:
        f.attach_pos[0] = np.maximum(mine[0][0][0], [-np.inf, -np.inf, f._rest_z[3]])
    return f._comove[3]['stage'] if 3 in f._comove else None, mine[0][0] if mine else None, plate


def _guard_lands(f, ticks):
    """His give-up with the coil holding: the guard fails on its wait, then the ring descends at land_vel from the
    LAND on (a step, the worst case) to its rest. [(stage, node 0, plate 9)] per tick."""
    f._give_up_cb(Bool(data=True))
    return [_gtick(f, -f.land_vel if f._fleet_cmd_pub.msgs else 0.0) for _ in range(ticks)]


def test_rig_guard_lets_the_rise_on_once_the_tip_has_left_the_plate():
    f = _guard_node()
    f._give_up_cb(Bool(data=True))
    g = f._comove[3]['guard']
    assert g is not None and g['z'] == pytest.approx(1.02 + 0.59 + 0.05 + GUARD_RISE_ABOVE_SLACK_M)
    for k in range(60):
        p = f._departed_hold[3]
        f.attach_pos[0] = p.copy()                             # QUAD4 flies its reference, the tip on its tether
        _set_tip(f, p - np.array([0.0, 0.0, 0.59]) if p[2] - 0.59 > 1.02 else [0.0, 0.225, 1.02])
        f._load_t = time.monotonic()
        _tick(f)
    assert f._comove[3]['stage'] == 'move' and f._fleet_cmd_pub.msgs == []
    assert f._departed_hold[3][2] > g['z'] + 0.05                    # rising on past the slack
    assert any('release seen' in ln for ln in f.log.lines)


@pytest.mark.parametrize('why', ['held', 'plate'])
def test_rig_guard_comes_down_to_the_tethers_slack_resends_off_and_lands_the_fleet(why):
    f = _guard_node()
    f._give_up_cb(Bool(data=True))
    zg = f._comove[3]['guard']['z']
    n_off = f._magnet_cmd_pub.msgs.count('OFF')
    zs, stages, lands = [], [], []
    for k in range(int((NEWCOMER_RELEASE_S + GUARD_WAIT_S) * HZ) + 50):
        if why == 'plate' and k == 9:
            f.load_state[2] = 1.0 + 0.03                       # plate 9 follows it up
        stage, n, plate = _gtick(f)                            # (the ring does not descend here)
        zs.append(n[0][2])
        stages.append(stage)
        lands.append(len(f._fleet_cmd_pub.msgs))
    assert stage == 'guard_follow' and f._fleet_cmd_pub.msgs == ['LAND']
    assert f._magnet_cmd_pub.msgs.count('OFF') > n_off
    assert max(zs) <= zg + 0.01                                # no rise past the slack
    assert zs[-1] == pytest.approx(plate[2] + 0.59 - GUARD_SLACK_M, abs=TAKE_BACK_REACHED_M)   # down to its slack
    # LAND goes once QUAD4 is down at the slack, not at the failure, and within GUARD_LAND_WAIT_S of it: from the
    # guard's full height (the timeout) the 0.22 m down to the 0.12 slack takes the whole wait, and LAND goes on it
    # with QUAD4 already 0.10 m under the taut tether
    i_fail, i_land = stages.index('guard_follow'), lands.index(1)
    assert 0 < i_land - i_fail <= GUARD_LAND_WAIT_S * HZ
    assert zs[i_land] <= plate[2] + 0.59 - GUARD_SLACK_M + TAKE_BACK_REACHED_M + 0.02
    assert any('RELEASE GUARD' in ln and "down to its tether's slack" in ln for ln in f.log.lines)
    at = 'at its tether slack' if why == 'plate' else f'not at its slack after {GUARD_LAND_WAIT_S:.0f} s'
    assert any('RELEASE GUARD' in ln and f'{at}: LAND the fleet' in ln for ln in f.log.lines)


def test_rig_guard_lands_the_fleet_after_its_wait_when_the_slack_is_not_seen():
    f = _guard_node()
    f._give_up_cb(Bool(data=True))
    stages, lands, stage = [], [], None
    for k in range(int((NEWCOMER_RELEASE_S + GUARD_WAIT_S + GUARD_LAND_WAIT_S) * HZ) + 20):
        if stage == 'guard_follow':
            f._load_t = time.monotonic() - 1.0                 # no ring pose since the failure
        stage, _, _ = _gtick(f, fresh=stage != 'guard_follow')
        stages.append(stage)
        lands.append(len(f._fleet_cmd_pub.msgs))
    i_fail, i_land = stages.index('guard_follow'), lands.index(1)
    assert i_land - i_fail == pytest.approx(GUARD_LAND_WAIT_S * HZ, abs=1)   # no fresh ring pose: the wait decides
    assert any('not at its slack after 2 s: LAND the fleet' in ln for ln in f.log.lines)


def test_rig_guard_follows_the_landing_ring_down_with_its_tether_slack():
    f = _guard_node(z=2.0)
    track = _guard_lands(f, 135)
    follow = [(n, p) for s, n, p in track if s == 'guard_follow']
    t_fail = next(i for i, (s, _, _) in enumerate(track) if s == 'guard_follow')
    assert len(follow) > 60 and f.load_state[2] < 0.5             # 1.5 m of the ring's descent followed
    settled = follow[int(SETTLE_S * HZ):]
    assert len(settled) > 30                                   # the ring still descending at land_vel
    for n, plate in settled:                                   # slack, never taut, never far below it
        assert plate[2] + 0.59 - GUARD_SLACK_M - 0.03 <= n[0][2] <= plate[2] + 0.59
        assert np.allclose(n[0][:2], [0.0, 0.225], atol=1e-6)  # xy held in the (still) ring frame
    vs = [n[1] for s, n, _ in track[t_fail - 1:] if s in ('guard_follow', 'land')]
    assert all(float(np.linalg.norm(b - a)) <= A_TICK for a, b in zip(vs, vs[1:]))


def test_rig_guard_once_the_ring_is_down_steps_out_and_down_inside_the_tether_and_lands_alone():
    f = _guard_node(z=2.0)
    f.__dict__.update(_departed_gi=True, _departed_pubs={3: _Pub()}, _at_rest_t={}, _press_t={})
    track = _guard_lands(f, 200)
    stages = [s for s, _, _ in track]
    i_out = stages.index('land')
    assert stages[i_out - 1] == 'guard_follow' and None in stages[i_out:]
    assert any('ring down, out by the tether\'s slack and down beside it' in ln for ln in f.log.lines)
    out = [(n, p) for s, n, p in track[i_out:] if s == 'land']
    pivot = np.array([0.0, 0.0, -0.04])
    for n, plate in out:                                       # the tip stays on the plate: the tether never taut
        assert float(np.linalg.norm(n[0] + pivot - plate)) < 0.55
    start, end = out[0][0][0], out[-1][0][0]
    assert end[1] - start[1] == pytest.approx(0.35, abs=0.03) and start[2] - end[2] == pytest.approx(0.35, abs=0.06)
    vs = [n[1] for n, _ in out]
    assert all(float(np.linalg.norm(b - a)) <= A_TICK for a, b in zip(vs, vs[1:]))
    # handed to the departed landing: straight down beside the ring to its rest, then its tracker may idle there
    assert 3 not in f._comove and 3 in f._land_alone and 3 in f._departed_landing
    after = [n for s, n, _ in track[stages.index(None):]]
    assert all(np.allclose(n[0][:2], after[0][0][:2]) for n in after)
    assert f._departed_pubs[3].msgs == [True] and any('is down (at its rest)' in ln for ln in f.log.lines)


def test_rig_guard_steps_out_on_the_survivors_touchdown_if_the_ring_never_reads_down():
    f = _guard_node()
    f._load_down = lambda: False                               # the ring at rest above its lift height + 0.05
    assert _guard_lands(f, 115)[-1][0] == 'guard_follow' and f.load_state[2] == 0.0
    f._touched_down = True                                     # the planner's survivors down
    assert _gtick(f)[0] == 'land'


def test_rig_guard_a_stale_ring_pose_holds_the_last_goal():
    f = _guard_node(z=2.0)
    track = _guard_lands(f, 60)
    assert track[-1][0] == 'guard_follow'
    st = f._comove[3]
    goal = st['goal'].copy()
    f._load_t = time.monotonic() - 1.0                         # no ring pose for 1 s
    f.load_state[2] -= 0.3                                     # (a jump in it is not chased)
    for _ in range(10):
        stage, n, _ = _gtick(f, -f.land_vel, fresh=False)
        assert stage == 'guard_follow' and np.allclose(st['goal'], goal) and st['guard']['vz'] == 0.0
    assert abs(float(n[1][2])) < abs(float(track[-1][1][1][2]))    # braking to the held goal, not chasing
    _gtick(f, -f.land_vel)
    assert st['goal'][2] == pytest.approx(f.load_state[2] + 0.02 + 0.59 - GUARD_SLACK_M)   # fresh again: following


def test_rig_guard_a_tilt_fault_or_a_drop_during_the_follow_clears_it():
    f = _tc(_guard_node(z=2.0))
    assert _guard_lands(f, 40)[-1][0] == 'guard_follow'
    f._tilt_fault_cb(Bool(data=True))
    assert 3 not in f._comove and f._departed_hold == {} and 3 not in f._departed_step
    assert all(_gtick(f, -f.land_vel)[1] is None for _ in range(5))
    g = _guard_node(z=2.0)
    assert _guard_lands(g, 40)[-1][0] == 'guard_follow'
    g._drop_departed(3)
    assert g._comove == {} and all(_gtick(g, -g.land_vel)[1] is None for _ in range(5))


def test_without_the_guard_nothing_waits_for_the_tip():
    f = _guard_node()
    f._release_guard = False
    f._give_up_cb(Bool(data=True))
    assert f._comove[3]['guard'] is None


# ── rig-port critic round 2 (rig card 2026-10-21_combined_m1_rig, ## Critic, round 2 part A): A1, A4, A5 ───────────

import dissipative_planner.dissipative_node as dn  # noqa: E402
from dissipative_planner.dissipative_network import DissipativeNetwork, DissipativeParams  # noqa: E402
from dissipative_planner.dissipative_node import gap_centre_rho  # noqa: E402
from mpc_planner.geometry import attach_points  # noqa: E402

WPZ = 0.031          # weld_point_z: the tip marker over the ring-origin plane with the face on plate 9
TAUT = 0.58          # QUAD4's real body-to-tip, taut (the planner's is 0.59)
LAG_S = 0.3          # QUAD4 follows its reference with this first-order lag


def _fly_released(slack, plane=False):
    """A1: his give-up over plate 9 with the rig's geometry: the tip resting on the plate at WPZ, QUAD4 `slack` under
    its taut tether (his aim low), the coil released, so the tip stays on the plate until the tether is taut and then
    follows the body; QUAD4 flies its reference with a LAG_S lag. plane: the guard reads plate 9 on the ring-origin
    plane (the placeholder's attach_z, before A4). Returns (the stage after the guard, s from its start)."""
    f = _guard_node(weld=(0.0, 0.225, WPZ), body_over_tip=TAUT - slack)
    rest = f._tip_now[1].copy()
    if plane:
        f._weld_rho = np.array([0.0, 0.225, 0.0])
    f._give_up_cb(Bool(data=True))
    body = f.attach_pos[0].copy()
    a = 1.0 - np.exp(-1.0 / (HZ * LAG_S))
    for _ in range(int((NEWCOMER_RELEASE_S + dn.GUARD_WAIT_S) * HZ) + 30):
        stage, n, _ = _gtick(f, tip=np.r_[rest[:2], max(rest[2], body[2] - TAUT)])
        body = body + a * (n[0] - body)
        f.attach_pos[0] = body.copy()
        if stage in ('move', 'guard_follow'):
            # the guard's verdict came on the tick before the clock's step
            return f, stage, round((f.now[0] - 1.0 / HZ - f._comove[3]['guard']['t0']) * HZ) / HZ
    raise AssertionError('the guard never ended')


@pytest.mark.parametrize('slack', [0.0, 0.01, 0.02])
def test_rig_guard_passes_a_released_coil_with_rig_geometry_within_its_wait(slack):
    """Real taut 0.58 against the planner's 0.59, his aim on it, 1 cm low or 1 cm more of sag, a 0.3 s lag, the guard
    stage at GUARD_STAGE_VEL: the pass comes with at least 1 s of GUARD_WAIT_S to spare and no LAND."""
    f, stage, t = _fly_released(slack)
    assert stage == 'move' and t <= GUARD_WAIT_S - 1.0
    assert f._fleet_cmd_pub.msgs == [] and any('release seen' in ln for ln in f.log.lines)


@pytest.mark.parametrize('slack', [0.01, 0.02])
def test_rig_guard_with_the_old_constants_had_no_margin(slack, monkeypatch):
    """Before A1 and A4 (+0.02 over the slack, 2 s, plate 9 on the ring-origin plane, the stage at 0.15 m/s), the
    same release: with 1 cm of slack it passes at best one planner tick before the timeout (on A5's debounce), with
    2 cm it times out (a false RELEASE GUARD: the fleet LANDs)."""
    monkeypatch.setattr(dn, 'GUARD_RISE_ABOVE_SLACK_M', 0.02)
    monkeypatch.setattr(dn, 'GUARD_WAIT_S', 2.0)
    monkeypatch.setattr(dn, 'GUARD_STAGE_VEL', dn.DETACH_STEP_VEL)
    f, stage, t = _fly_released(slack, plane=True)
    assert stage == 'guard_follow' or t >= 2.0 - 1.0 / HZ - 1e-6
    if slack == 0.02:
        assert stage == 'guard_follow' and any('has not left plate 9 in 2 s' in ln for ln in f.log.lines)


@pytest.mark.parametrize('az', [(270.0, 30.0, 150.0), (30.0, 150.0, 270.0), (150.0, 270.0, 30.0),
                                (271.0, 30.0, 150.0)])
def test_rig_guard_reads_plate_9_at_the_weld_point_whatever_the_carrier_azimuths(az):
    """A4: the planner's net with the rig's carriers; the reserved slot's placeholder is the centre of the largest gap.
    At 270/30/150 (in any order: it sorts them) the three gaps tie and argmax's tie-break lands it on plate 9; one
    carrier 1 deg off moves it to ~210 deg, and the guard read it would not arm with the tip on plate 9. The guard
    reads the launch's weld point (attach offset (0, 0.225), weld_point_z) instead."""
    rho = attach_points(3, 0.225, 0.0, ','.join(f'{a:g}' for a in az))
    net = DissipativeNetwork(4, rho + [gap_centre_rho(rho, 0.225, 0.0)], 0.55, 0.55, 0.86, 9.81, DissipativeParams())
    place_az = np.degrees(np.arctan2(net.rho[3][1], net.rho[3][0])) % 360.0
    assert place_az == pytest.approx(90.0 if az[0] != 271.0 else 210.5)
    f = _guard_node(weld=(0.0, 0.225, WPZ))
    f.net = net
    np.testing.assert_allclose(f._weld_rho_of(0), [0.0, 0.225, WPZ])
    roll = np.radians(5.0)                                   # the ring rolled about x: plate 9 (+y) rises
    f.load_state[3:7] = [np.cos(roll / 2.0), np.sin(roll / 2.0), 0.0, 0.0]
    _, plate_z = f._guard_read(0)
    assert plate_z == pytest.approx(f.load_state[2] + 0.225 * np.sin(roll) + WPZ * np.cos(roll))
    f.load_state[3:7] = [1.0, 0.0, 0.0, 0.0]
    f._give_up_cb(Bool(data=True))
    assert f._comove[3]['guard'] is not None                 # armed: the tip is on plate 9
    if az[0] == 271.0:                                       # the placeholder would have left it off
        g = _guard_node(weld=(0.0, 0.225, WPZ))
        g.net, g._weld_rho = net, None
        g._give_up_cb(Bool(data=True))
        assert g._comove[3]['guard'] is None


def _in_guard(**kw):
    """His give-up with the tip held on plate 9, ticked into the guard and past its plate baseline."""
    f = _guard_node(**kw)
    f._give_up_cb(Bool(data=True))
    while _gtick(f)[0] != 'guard':
        pass
    while f._comove[3]['guard']['plate_z0'] is None:
        _gtick(f)
    return f


def _release_tick(f, up=0.06, fresh=True):
    """One tick with the tip `up` off plate 9 on a taut tether, QUAD4's body over it. Returns the stage."""
    tip = np.array([0.0, 0.0, float(f.load_state[2])]) + f._weld_rho_of(0) + np.array([0.0, 0.0, up])
    f.attach_pos[0] = tip + np.array([0.0, 0.0, 0.59])
    if not fresh:
        f._load_t = time.monotonic() - 1.0                   # no ring pose for 1 s
    return _gtick(f, tip=tip, fresh=fresh)[0]


def test_rig_guard_a_real_release_passes_after_consecutive_fresh_samples():
    f = _in_guard()
    stages = [_release_tick(f) for _ in range(4)]
    assert stages[0] == 'guard' and stages.index('move') in (1, 2)   # not on its first sample: GUARD_PASS_S of them
    assert f._fleet_cmd_pub.msgs == []


def test_rig_guard_a_single_sample_spike_does_not_pass():
    f = _in_guard()
    for _ in range(3):
        assert _release_tick(f) == 'guard'                   # one sample off the plate...
        assert _gtick(f)[0] == 'guard'                       # ...then back on it: the run starts again
    assert _release_tick(f) == 'guard'                       # one sample, read on the next ticks (no new frame)
    for _ in range(4):
        f._load_t = time.monotonic()
        _tick(f)
        assert f._comove[3]['stage'] == 'guard'


def test_rig_guard_a_stale_ring_pose_neither_passes_nor_fails():
    f = _in_guard()
    for _ in range(5):                                       # a release, the ring pose stale: no pass
        assert _release_tick(f, fresh=False) == 'guard'
    assert _release_tick(f) == 'guard' and 'move' in [_release_tick(f) for _ in range(2)]   # fresh: it passes
    g = _in_guard()
    g.load_state[2] += 0.05                                  # plate 9 risen 5 cm, its pose stale: no fail...
    stage = 'guard'
    while stage == 'guard':
        g._load_t = time.monotonic() - 1.0
        stage = _gtick(g, fresh=False)[0]
    assert stage == 'guard_follow'                           # ...and the wait still runs
    assert any(f'has not left plate 9 in {GUARD_WAIT_S:.0f} s' in ln for ln in g.log.lines)
    assert not any('plate 9 follows it' in ln for ln in g.log.lines)


def test_rig_guard_a_tip_puck_marker_swap_does_not_pass():
    f = _in_guard()
    for k in range(10):
        body = f.attach_pos[0].copy()
        swap = body - np.array([0.0, 0.0, 0.4]) if k % 2 else np.array([1.5, -1.0, 0.05])   # 0.4 m under it; the net
        assert _gtick(f, tip=swap)[0] == 'guard'


def test_rig_guard_plate_baseline_is_averaged_over_its_first_samples():
    f = _guard_node()
    f._give_up_cb(Bool(data=True))
    while _gtick(f)[0] != 'guard':
        pass
    z = float(f.load_state[2])
    f.load_state[2] = z + 0.03                               # its first sample: one ring pose 3 cm high
    _gtick(f)
    f.load_state[2] = z
    while f._comove[3]['guard']['plate_z0'] is None:
        _gtick(f)
    assert f._comove[3]['guard']['plate_z0'] < z + 0.02 + 0.02
    f.load_state[2] = z + 0.035                              # the coil holds: plate 9 follows QUAD4 up
    assert _gtick(f)[0] == 'guard_follow' and any('plate 9 follows it' in ln for ln in f.log.lines)


# ── critic round 2 follow-up (rig card 2026-10-21_combined_m1_rig): 2 the timeout's causes, 3 the follow-down's
# transient, 4 a held coil's lift, 5 tip0 from the guard's start ────────────────────────────────────────────────────

def test_rig_guard_a_timeout_names_its_causes_a_band_miss():
    """A tip off the plate but 0.4 m from the body for the whole wait: the timeout's line counts the band misses."""
    f = _guard_node()
    f._give_up_cb(Bool(data=True))
    while _gtick(f)[0] != 'guard':
        pass
    _gtick(f)                                                  # the guard's first tip pose: on the plate (tip0)
    stage = 'guard'
    while stage == 'guard':
        stage = _gtick(f, tip=f.attach_pos[0] - np.array([0.0, 0.0, 0.4]))[0]
    line = next(ln for ln in f.log.lines if 'has not left plate 9' in ln)
    m = re.search(r'ring pose stale (\d+), tip pose stale (\d+), body pose stale (\d+), tip on the plate '
                  r'\(< 0\.05 m off\) (\d+), tip-to-body off 0\.590 \+- 0\.03 m (\d+) \(last 0\.400 m\)', line)
    assert m, line
    ring, tip, body, plate, band = map(int, m.groups())
    assert (ring, tip, body, plate) == (0, 0, 0, 0) and band >= GUARD_WAIT_S * HZ - 2


def test_rig_guard_measures_tip0_from_its_start_not_the_latch():
    """The tip slid 6 cm along the plate during the hold (the coil holding it there): measured from the latch it would
    read as a release; measured from the guard's start it does not, and the guard times out on the plate."""
    f = _guard_node()
    f._give_up_cb(Bool(data=True))
    slid = np.array([0.06, 0.225, 1.02])
    stage, tip0 = 'hold', None
    while stage in ('hold', 'guard'):
        f.attach_pos[0] = slid + np.array([0.0, 0.0, 0.59])     # QUAD4 over it, its tether taut (the ring stiff)
        stage = _gtick(f, tip=slid)[0]
        g = f._comove[3]['guard']
        if stage == 'guard' and g['tip0'] is not None and tip0 is None:
            tip0 = g['tip0'].copy()
    np.testing.assert_allclose(tip0, [0.06, 0.225, 0.02], atol=1e-9)   # ring frame
    assert stage == 'guard_follow' and not any('release seen' in ln for ln in f.log.lines)
    assert any(re.search(r'tip on the plate \(< 0\.05 m off\) [1-9]', ln) for ln in f.log.lines)


# QUAD4 tracking a moving reference: half its velocity fed forward, the rest closed at TRACK_TAU_S, so it trails by
# (1 - TRACK_FF) * v * TRACK_TAU_S = 3 cm at land_vel 0.2 m/s (the twin's QUAD4 trails by 2-6 mm median, 1.5-3 cm at
# the 95th percentile: TB6's rise and co-move phases, Wesley 11 Oct)
TRACK_FF = 0.5
TRACK_TAU_S = 0.3


def _held_coil(lifts=False, slack=0.01, ticks=250):
    """His give-up with the coil still holding the tip on plate 9 (rig geometry, QUAD4 `slack` under its taut tether,
    tracking its reference as TRACK_FF and TRACK_TAU_S say). lifts: once the tether is taut plate 9 follows QUAD4's
    body (the ring's stiffness ignored) until the LAND, so the guard's plate-rise check fails it; else the ring is
    stiff and the guard times out. From the LAND the carriers set the ring, stepping down at land_vel 0.2 m/s to its
    rest. Per tick until the follow ends: (stage, reference z, body z, plate 9 z, the ring's own plate 9 z, LAND
    sent)."""
    f = _guard_node(z=2.0, weld=(0.0, 0.225, WPZ), body_over_tip=TAUT - slack)
    f._give_up_cb(Bool(data=True))
    body = f.attach_pos[0].copy()
    ring = float(f.load_state[2])
    rows = []
    for _ in range(ticks):
        landed = bool(f._fleet_cmd_pub.msgs)
        own = ring + WPZ
        plate = max(own, body[2] - TAUT) if lifts and not landed else own
        stage, n, _ = _gtick(f, ring_vz=(plate - WPZ - float(f.load_state[2])) * HZ, tip=np.r_[0.0, 0.225, plate])
        if stage not in ('hold', 'guard', 'guard_follow'):
            break
        rows.append((stage, float(n[0][2]), float(body[2]), plate, own, landed))
        body = body + (TRACK_FF * n[1] + (n[0] - body) / TRACK_TAU_S) / HZ
        f.attach_pos[0] = body.copy()
        if landed:
            ring = max(f.lift_z0, ring - f.land_vel / HZ)
    return f, rows


def test_rig_guard_follow_stays_under_the_taut_tether_through_the_land_step():
    """A held coil on a stiff ring: the guard times out with QUAD4 at the guard's height, over the taut tether (the
    guard stage put it there, not the follow). The follow's reference only comes down from there, and is back at or
    below plate 9 + the taut length before the LAND; from then on, through the LAND's step to land_vel and the whole
    descent, the reference and the tracking body stay at or below it."""
    f, rows = _held_coil()
    follow = [r for r in rows if r[0] == 'guard_follow']
    assert follow[0][1] > follow[0][4] + TAUT and follow[-1][4] < 0.2   # from the guard's height to the ring's rest
    i = next(i for i, r in enumerate(follow) if max(r[1], r[2]) <= r[4] + TAUT)
    assert all(b[1] <= a[1] + 1e-9 for a, b in zip(follow[:i], follow[1:i + 1]))   # only coming down until then
    assert not follow[i][5] and any(r[5] for r in follow[i:])  # back under before the LAND
    for _, ref, body, _, own, _ in follow[i:]:
        assert ref <= own + TAUT and body <= own + TAUT


@pytest.mark.parametrize('slack', [0.0, 0.01, 0.02])
def test_rig_guard_a_held_coil_lifts_plate_9_at_most_4_cm_and_is_down_before_the_land(slack):
    """Plate 9 following QUAD4 once the tether is taut: the 2 cm check reacts on its first sample over it, then the
    reference brakes from the guard stage's GUARD_STAGE_VEL and the body trails it, so the plate peaks under 4 cm
    (~6 cm at the stage's old 0.15 m/s); QUAD4 has let it down before the fleet's LAND goes. Through the follow the
    reference asks at most its braking distance over the taut tether."""
    f, rows = _held_coil(lifts=True, slack=slack)
    lift = [r[3] - r[4] for r in rows]
    i_fail = [r[0] for r in rows].index('guard_follow')
    i_land = [r[5] for r in rows].index(True)
    assert any('plate 9 follows it' in ln for ln in f.log.lines)
    rise = [r[3] - f._comove[3]['guard']['plate_z0'] for r in rows]   # over its baseline (with no slack the lift
    assert rise[i_fail - 1] <= dn.GUARD_PLATE_RISE_M < rise[i_fail] <= dn.GUARD_PLATE_RISE_M + 0.01   # starts in it)
    assert max(lift) <= 0.04 and lift[i_land] == 0.0
    brake = dn.GUARD_STAGE_VEL ** 2 / (2.0 * dn.TAKE_BACK_BRAKE)
    assert all(r[1] <= r[3] + TAUT + brake for r in rows[i_fail:])

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
        dyn=SimpleNamespace(g=9.81), N=20, dt=0.1, drone_mass=0.55, attach_radius=0.225, _attach_body_to_tip=0.59,
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
    rise = 1.0 + 0.59 + 0.15                                   # its tip is down at the ring: up first (R1543)
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
    rise = 1.0 + 0.59 + 0.15
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

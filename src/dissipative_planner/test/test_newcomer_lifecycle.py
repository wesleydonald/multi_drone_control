"""Newcomer LAND and abort (card 2026-10-09_newcomer_land_abort; Wesley's M1 rules a and b, 8 Oct): an unwelded
newcomer taken out of the flight (LAND, /magnet/command OFF) holds while its magnet lets go, moves radially out from
the ring at its height, lands by itself and the landed gate waits for it; it cannot weld again this flight. Off by
default (newcomer_lifecycle), and never in partner runs."""
from types import MethodType, SimpleNamespace

import numpy as np
import pytest

pytest.importorskip('acados_template')
from dissipative_planner.dissipative_node import (  # noqa: E402
    DEPARTED_CLEAR_R, NEWCOMER_OFF_EVERY_S, NEWCOMER_RELEASE_S, NEWCOMER_POSE_STALE_S,
    DissipativeController as D)
from std_msgs.msg import Bool, String  # noqa: E402

HZ = 10.0


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


def _node(p_ref=(0.0, 0.25, 1.64), pending=True, approaching=True, on=True):
    log = _Log()
    prof = SimpleNamespace(p_ref=np.array(p_ref, float))
    carriers = [np.array([0.5 * np.cos(np.radians(a)), 0.5 * np.sin(np.radians(a)), 1.5]) for a in (270, 30, 150)]
    f = SimpleNamespace(
        _newcomer_lifecycle=on, _newcomer_fault_land=False, _newcomer_armed={}, _n_carry0=3, reserved_attach=1,
        n=3, attach_pending=[pending],
        attach_pos=[np.array(p_ref, float)], _attach_pos_t=[None],
        _approach={0: prof} if approaching else {}, _newcomer_out={}, _land_alone=set(), _off_resend_t=None,
        _approach_hold_armed=True, _attach_moving=True, _reconfig_hold_left=0.0,
        _departed_hold={}, _departed_step={}, _departed_next={}, _departed_td={}, _departed_clear={},
        _departed_landing=set(), _departed_wait={}, _departed_land=False, _departed_land_vel=0.0, land_vel=0.2,
        _departed_down_z=0.15, _land_to_ground=False, _dropped=None, _landed=False,
        _approach_hold_on=True, takeoff_seen=True, phase='planner',
        load_state=np.r_[0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, np.zeros(6)], drone_pos=list(carriers),
        dyn=SimpleNamespace(g=9.81), N=20, _magnet_cmd_pub=_Pub(),
        traj=SimpleNamespace(kind='hover', radius=0.0), hover_xy=None,
        _departed_gi=False, _rest_z={}, _departed_sent=set(), _departed_pubs={})
    f.get_logger = lambda: log
    f.log = log
    f.now = [100.0]
    f.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=int(f.now[0] * 1e9)))
    for name in ('_abort_newcomer', '_newcomer_off_tick', '_magnet_cmd_cb', '_magnet_attached_cb',
                 '_newcomers_out_on_land', '_publish_departed_refs', '_departed_down', '_pos', '_reserved_index',
                 '_weld_refused', '_set_on_load', '_path_clear_point'):
        setattr(f, name, MethodType(getattr(D, name), f))
    f.attached = []
    f._do_attach = lambda d: f.attached.append(d)
    f.published = []
    f._publish_ref = lambda d, nodes: f.published.append((d, nodes[0][0].copy()))
    return f


def _ticks(f, n):
    for _ in range(n):
        f._publish_departed_refs()


def test_abort_in_the_hold_departs_from_the_approach_reference_radially_at_its_height():
    f = _node()
    f._abort_newcomer(0, 'abort')
    assert f._newcomer_out == {0: 'abort'} and 0 not in f._approach
    assert f._magnet_cmd_pub.msgs == ['OFF']                       # OFF before anything moves
    assert np.allclose(f._departed_hold[3], [0.0, 0.25, 1.64])     # bumpless: the approach's own reference
    step = f._departed_step[3]                                     # out along its own radial, same height
    assert abs(np.linalg.norm(step[:2]) - DEPARTED_CLEAR_R) < 1e-9 and step[0] == 0.0 and step[1] > 0
    assert step[2] == pytest.approx(1.64) and 3 not in f._departed_next
    assert 3 in f._land_alone and not f._approach_hold_armed
    assert f._departed_wait[3] == NEWCOMER_RELEASE_S


def test_it_holds_while_the_magnet_lets_go_then_moves_level():
    f = _node()
    f._abort_newcomer(0, 'abort')
    n_wait = int(round(NEWCOMER_RELEASE_S * HZ))
    _ticks(f, n_wait)
    assert all(np.allclose(q, [0.0, 0.25, 1.64]) for _, q in f.published)   # held in place
    _ticks(f, 3)
    assert f.published[-1][1][1] > 0.25 and f.published[-1][1][2] == pytest.approx(1.64)


def test_switch_off_or_welded_or_twice_does_nothing_more():
    f = _node(on=False)
    f._abort_newcomer(0, 'abort')
    f._magnet_cmd_cb(String(data='OFF'))
    f._newcomers_out_on_land('LAND')
    assert f._newcomer_out == {} and f._departed_hold == {} and f._magnet_cmd_pub.msgs == []
    g = _node(pending=False)
    g._abort_newcomer(0, 'LAND')
    assert g._newcomer_out == {} and g._magnet_cmd_pub.msgs == []
    h = _node()
    h._abort_newcomer(0, 'abort')
    h._abort_newcomer(0, 'LAND')
    assert h._newcomer_out == {0: 'abort'} and h._magnet_cmd_pub.msgs == ['OFF']


def test_a_newcomer_still_on_the_floor_gets_no_reference():
    f = _node(p_ref=(0.0, 1.4, 0.1), approaching=False)
    f._newcomers_out_on_land('LAND')
    assert f._newcomer_out == {0: 'LAND'} and f._departed_hold == {} and 3 not in f._land_alone


def test_it_moves_clear_then_lands_alone_and_the_gate_waits():
    f = _node()
    f._abort_newcomer(0, 'abort')
    _ticks(f, int(round(NEWCOMER_RELEASE_S * HZ)) + 70)               # 0.95 m out at 0.15 m/s
    zs = [q[2] for _, q in f.published if np.linalg.norm(q[:2]) < DEPARTED_CLEAR_R - 0.02]
    assert max(zs) == min(zs) == pytest.approx(1.64)               # level while it moves out
    assert 3 in f._departed_landing
    f.attach_pos[0] = np.array([0.0, 1.2, 1.0])                    # in the air: not down
    assert not f._departed_down()
    for _ in range(20):                                            # it follows its reference down
        f.attach_pos[0] = f.published[-1][1].copy()
        f._publish_departed_refs()
    assert f.published[-1][1][2] < 1.64 - 0.5                      # descending, no LAND needed
    f.attach_pos[0] = np.array([0.0, 1.2, 0.12])                   # on the floor
    assert f._departed_down()


def test_touchdown_detector_survives_ticks_and_freezes_the_reference():
    f = _node()
    f._abort_newcomer(0, 'abort')
    f._departed_step.pop(3)
    f._departed_wait.pop(3)
    f.attach_pos[0] = np.array([0.0, 1.2, 1.64])
    _ticks(f, 5)
    assert 3 in f._departed_td                                    # kept although departed_land is off
    for _ in range(60):
        f.attach_pos[0] = np.array([0.0, 1.2, 0.11])
        f._publish_departed_refs()
    assert f._departed_td[3].stalled
    frozen = f.published[-1][1][2]
    f._publish_departed_refs()
    assert f.published[-1][1][2] == frozen


def test_land_keeps_an_out_newcomer_on_its_own_path_not_the_fleet_land_branch():
    f = _node()
    f._newcomers_out_on_land('LAND')
    assert f._newcomer_out == {0: 'LAND'}
    f._land_to_ground = True                                       # the parent's LAND follows
    _ticks(f, int(round(NEWCOMER_RELEASE_S * HZ)) + 20)
    q = f.published[-1][1]
    assert q[2] == pytest.approx(1.64) and q[0] == 0.0             # its own 0.15 m/s radial path, level
    assert 0.25 < q[1] < 0.25 + 0.15 * 2.0 + 0.02                  # not the fleet's 0.4 m/s clear-out
    assert not f._departed_clear.get(3, False)


@pytest.mark.parametrize('kw', [{'takeoff_seen': False}, {'_dropped': 'test'}, {'_land_to_ground': True},
                                {'phase': 'network'}])
def test_land_that_is_not_a_fleet_landing_takes_nobody_out(kw):
    f = _node()
    for k, v in kw.items():
        setattr(f, k, v)
    f._newcomers_out_on_land('LAND')
    assert f._newcomer_out == {}


def test_arm_before_takeoff_starts_a_fresh_flight():
    f = _node()
    f._newcomer_out[0] = 'abort'
    f.takeoff_seen = False
    f._newcomers_out_on_land('ARM')
    assert f._newcomer_out == {}


def test_off_aborts_the_approach_and_its_own_off_echo_does_nothing():
    f = _node()
    f._magnet_cmd_cb(String(data='OFF'))
    assert f._newcomer_out == {0: 'abort'} and 3 in f._departed_hold
    f._magnet_cmd_cb(String(data='OFF'))
    assert f._magnet_cmd_pub.msgs == ['OFF']


def test_on_after_an_abort_or_during_land_is_refused_with_off():
    f = _node()
    f._abort_newcomer(0, 'abort')
    f._magnet_cmd_cb(String(data='ON'))
    assert f._magnet_cmd_pub.msgs == ['OFF', 'OFF'] and any('ATTACH refused' in m for m in f.log.lines)
    g = _node()
    g._land_to_ground = True
    g._magnet_cmd_cb(String(data='ON'))
    assert g._magnet_cmd_pub.msgs == ['OFF'] and any('LAND in progress' in m for m in g.log.lines)


def test_off_is_re_sent_while_the_newcomer_is_out():
    f = _node()
    f._abort_newcomer(0, 'abort')
    for _ in range(int(round(3 * NEWCOMER_OFF_EVERY_S * HZ))):
        f._newcomer_off_tick()
    assert f._magnet_cmd_pub.msgs == ['OFF'] * 4                   # the abort's OFF + one every 0.5 s
    g = _node()
    for _ in range(10):
        g._newcomer_off_tick()
    assert g._magnet_cmd_pub.msgs == []                            # nobody out: nothing sent


def test_no_weld_for_an_out_newcomer_or_during_land():
    f = _node()
    f._abort_newcomer(0, 'abort')
    f._magnet_attached_cb(Bool(data=True))
    assert f.attached == [] and any('weld of drone 4 (/drone_3) refused' in m for m in f.log.lines)
    g = _node()
    g._land_to_ground = True
    g._magnet_attached_cb(Bool(data=True))
    assert g.attached == []
    h = _node()
    h._magnet_attached_cb(Bool(data=True))
    assert h.attached == [3]
    k = _node(on=False)                                            # switch off: today's behaviour
    k._land_to_ground = True
    k._magnet_attached_cb(Bool(data=True))
    assert k.attached == [3]


def test_pos_reads_the_newcomer_from_its_own_mocap_and_drops_a_stale_one():
    f = _node(p_ref=(0.1, 0.2, 0.3))
    assert np.allclose(f._pos(3), [0.1, 0.2, 0.3]) and np.allclose(f._pos(0), f.drone_pos[0])
    assert f._pos(7) is None
    f._attach_pos_t[0] = 100.0
    assert f._pos(3) is not None
    f.now[0] = 100.0 + NEWCOMER_POSE_STALE_S + 0.1
    assert f._pos(3) is None


def test_on_during_land_never_sends_off_to_a_welded_newcomer():
    """OFF is how a welded newcomer is released: a refused ON must not send it while one carries (critic, pass 2)."""
    f = _node(pending=False, approaching=False)
    f._land_to_ground = True
    f._magnet_cmd_cb(String(data='ON'))
    assert f._magnet_cmd_pub.msgs == [] and any('ATTACH refused' in m for m in f.log.lines)


def test_a_refused_weld_restarts_the_release_hold():
    f = _node()
    f._abort_newcomer(0, 'abort')
    _ticks(f, int(round(NEWCOMER_RELEASE_S * HZ)) + 2)             # moving out already
    assert f._departed_wait[3] <= 0.0
    f._magnet_attached_cb(Bool(data=True))                         # the magnet caught the plate anyway
    assert f._departed_wait[3] == NEWCOMER_RELEASE_S and f._magnet_cmd_pub.msgs[-1] == 'OFF'
    q0 = f.published[-1][1].copy()
    _ticks(f, int(round(NEWCOMER_RELEASE_S * HZ)))
    assert np.allclose(f.published[-1][1], q0)                     # held while it lets go


def test_an_abort_before_it_leaves_the_floor_lands_it_where_it_is():
    f = _node(p_ref=(0.0, 1.4, 0.18))                              # its climb reference has started
    f.attach_pos[0] = np.array([0.0, 1.4, 0.10])                   # but it is still on the floor
    f._newcomers_out_on_land('LAND')
    assert np.allclose(f._departed_hold[3], [0.0, 1.4, 0.10]) and 3 in f._land_alone
    assert 3 not in f._departed_step and 3 not in f._departed_wait
    _ticks(f, 3)
    assert f.published[-1][1][2] < 0.10                            # its reference goes down, not up


def test_an_abort_during_a_circle_moves_on_out_past_the_rings_path():
    f = _node()
    f.traj, f.hover_xy = SimpleNamespace(kind='orbit', radius=0.5), (0.0, 0.0)   # path centre (0, 0.5)
    f._abort_newcomer(0, 'abort')
    step, nxt = f._departed_step[3], f._departed_next[3]
    assert abs(np.linalg.norm(step[:2]) - DEPARTED_CLEAR_R) < 1e-9                # first clear of the ring
    assert np.linalg.norm(nxt[:2] - [0.0, 0.5]) == pytest.approx(0.5 + 1.0)       # then clear of its path
    assert nxt[2] == step[2] == pytest.approx(1.64)


def test_the_floor_is_the_height_it_rested_at_not_an_absolute_one():
    f = _node(p_ref=(0.0, 1.4, 0.30))
    f._approach_z0 = {0: 0.20}                                     # the rig rests higher than the twin
    f.attach_pos[0] = np.array([0.0, 1.4, 0.22])                   # 2 cm up: still on the floor
    f._abort_newcomer(0, 'LAND')
    assert np.allclose(f._departed_hold[3], [0.0, 1.4, 0.22]) and 3 not in f._departed_wait
    g = _node(p_ref=(0.0, 1.4, 0.30))
    g._approach_z0 = {0: 0.10}
    g.attach_pos[0] = np.array([0.0, 1.4, 0.22])                   # 12 cm up: airborne
    g._abort_newcomer(0, 'LAND')
    assert 3 in g._departed_wait

"""A departed drone landing by itself reaches its own resting height and idles there (card
2026-10-09_departed_ground_idle): its reference goes down to DEPARTED_PRESS_M below its rest (never further below
the drone itself), a stall above its rest is not "down", the landed gate wants it at its rest, and its tracker is
told it is departed (False at ARM and TAKEOFF, True when its own descent starts)."""
from types import MethodType, SimpleNamespace

import numpy as np
import pytest

pytest.importorskip('acados_template')
from dissipative_planner.dissipative_node import (  # noqa: E402
    DEPARTED_PRESS_LIMIT_S, DEPARTED_PRESS_M, DEPARTED_REST_TOL_M, DissipativeController as D)

from test_newcomer_lifecycle import _node as _lifecycle_node, _ticks  # noqa: E402


class _Pub:
    def __init__(self):
        self.msgs = []

    def publish(self, m):
        self.msgs.append(m.data)


def _node(rest=0.10, z=1.2):
    f = _lifecycle_node(p_ref=(0.0, 1.2, z))
    f._departed_gi, f._rest_z, f._departed_sent = True, {3: rest}, set()
    f._at_rest_t, f._press_t, f._high_stall_said = {}, {}, set()
    f._departed_pubs = {d: _Pub() for d in range(4)}
    f.n_net = 4
    f._departed_flight_start = MethodType(D._departed_flight_start, f)
    # landing by itself straight away: no step, no release hold
    f._departed_hold[3] = np.array([0.0, 1.2, z])
    f._land_alone.add(3)
    f.attach_pos[0] = np.array([0.0, 1.2, z])
    return f


def test_the_tracker_is_told_once_when_its_descent_starts():
    f = _node()
    _ticks(f, 3)
    assert f._departed_pubs[3].msgs == [True]


def test_the_reference_goes_below_its_rest_but_not_far_below_the_drone():
    f = _node(z=0.30)
    for _ in range(40):
        _ticks(f, 1)
    assert f.published[-1][1][2] == pytest.approx(0.30 - DEPARTED_PRESS_M)   # the drone stays at 0.30: lead-limited
    g = _node(z=0.06)                                              # 4 cm below its rest (outside the margin)
    _ticks(g, 10)
    assert g.published[-1][1][2] == pytest.approx(0.10 - DEPARTED_PRESS_M)   # at most PRESS below its rest


def test_a_stall_above_its_rest_is_not_down():
    f = _node(z=0.60)
    for z in np.linspace(0.60, 0.24, 20):                          # it descends, then stays light on its rod
        f.attach_pos[0] = np.array([0.0, 1.2, z])                  # 14 cm above its rest (R1252)
        _ticks(f, 1)
    _ticks(f, 40)
    assert not f._departed_down()                                  # pressed, then held there: never 'down'
    assert any('above its rest: still pressing down' in m for m in f.log.lines)


def test_at_its_rest_it_is_down():
    f = _node(z=0.10 + DEPARTED_REST_TOL_M - 0.01)
    assert f._departed_down()
    f.attach_pos[0] = np.array([0.0, 1.2, 0.10 + DEPARTED_REST_TOL_M + 0.02])
    assert not f._departed_down()


def test_arm_and_takeoff_reset_the_flags_and_takeoff_records_rest():
    f = _node()
    f._departed_sent = {3}
    f._rest_z = {}
    f.attach_pos[0] = np.array([0.0, 1.4, 0.061])                  # the rig's resting height, not 0.10
    f._departed_flight_start('TAKEOFF')
    assert all(p.msgs == [False] for p in f._departed_pubs.values()) and f._departed_sent == set()
    assert f._rest_z[3] == pytest.approx(0.061) and f._rest_z[0] == pytest.approx(f.drone_pos[0][2])


def test_switch_off_keeps_todays_landing():
    f = _node(z=0.24)
    f._departed_gi = False
    _ticks(f, 80)
    assert f._departed_pubs[3].msgs == [] and f.published[-1][1][2] >= 0.0


def _at(f, z, ticks):
    for _ in range(ticks):
        f.attach_pos[0] = np.array([0.0, 1.2, z])
        _ticks(f, 1)


def test_a_drone_that_never_left_the_floor_is_not_pressed_for_ever():
    """Critic pass 2 N1: the stall test needs a real descent; at its rest for 0.5 s it is down and the press ends."""
    f = _node(z=0.10)
    _at(f, 0.10, 8)
    assert f._departed_td[3].stalled and f.published[-1][1][2] == pytest.approx(0.10)
    assert f._departed_down()


def test_the_press_after_a_high_stall_is_limited_then_held_where_it_is():
    """Critic pass 2 N2: stopped above its rest -> 2 s more press, then held at its own height (not down)."""
    f = _node(z=0.60)
    for z in np.linspace(0.60, 0.24, 20):
        _at(f, z, 1)
    _at(f, 0.24, 40)                                                # the detector re-arms after its stall
    assert 3 in f._press_t
    _at(f, 0.24, int(round(DEPARTED_PRESS_LIMIT_S * 10)) + 5)       # the press limit
    assert f._departed_td[3].stalled and f.published[-1][1][2] == pytest.approx(0.24)
    assert not f._departed_down() and any('held there (not down)' in m for m in f.log.lines)


def test_arm_clears_the_rest_heights_and_the_warnings():
    f = _node()
    f._high_stall_said = {3}
    f._departed_flight_start('ARM')
    assert f._rest_z == {} and f._high_stall_said == set()


def test_the_press_limit_does_not_fire_once_it_is_back_at_its_rest():
    """R1268: the 2 s press timer ran out the tick the drone reached its rest and logged 'held there (not down)'."""
    f = _node(z=0.60)
    for z in np.linspace(0.60, 0.40, 10):
        _at(f, z, 1)
    _at(f, 0.40, 30)                                                # a high stall: the press timer starts
    assert 3 in f._press_t
    t_press = f._press_t[3]
    for z in np.linspace(0.40, 0.10, 4):                           # the rod falls, it comes down
        _at(f, z, 1)
    assert f._press_t[3] < 2.0                                     # it reached its rest inside the 2 s
    _at(f, 0.10, 25)                                               # at its rest past the 2 s mark
    assert f._press_t[3] <= t_press + 0.4 + 1e-9                   # the timer stopped at its rest
    assert f._departed_down() and not any('held there' in m for m in f.log.lines)
    assert any('is down (at its rest)' in m for m in f.log.lines)


def test_takeoff_records_the_rest_from_a_stale_pose_too():
    """R1270: the newcomer's pose was > 0.5 s old at TAKEOFF (a busy planner) and no rest height was recorded."""
    f = _node()
    f._rest_z = {}
    f.attach_pos[0] = np.array([0.0, 1.4, 0.10])
    f._attach_pos_t[0] = 90.0                                      # 10 s old at f.now = 100
    assert f._pos(3) is None
    f._departed_flight_start('TAKEOFF')
    assert f._rest_z[3] == pytest.approx(0.10)


def test_a_stall_a_few_cm_above_its_rest_is_down():
    """14 Oct: QUAD4 stopped 4-7 cm above its rest after each take-back and was never disarmed alone."""
    f = _node(z=0.60)
    for z in np.linspace(0.60, 0.16, 20):                          # 6 cm above its 0.10 rest
        _at(f, z, 1)
    _at(f, 0.16, 20)
    assert f._departed_td[3].stalled and f.published[-1][1][2] == pytest.approx(0.16)
    assert f._departed_down() and any('is down (+0.06 m above its rest)' in m for m in f.log.lines)

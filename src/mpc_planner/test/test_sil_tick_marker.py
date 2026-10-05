"""The planner's /planner/tick marker (sim time only): published after every timer tick,
also one that returned early or raised, because the SIL bench holds its clock on it."""
import types

import pytest

from mpc_planner.planner_node import LoadPlanner, PLANNER_HZ


class _Pub:
    def __init__(self):
        self.sent = []

    def publish(self, msg):
        self.sent.append(list(msg.data))


class _Clock:
    def now(self):
        return types.SimpleNamespace(nanoseconds=12_340_000_000)


def _node(plan):
    n = types.SimpleNamespace(_tick_pub=_Pub(), get_clock=_Clock, _plan=plan)
    return n


def test_marker_after_a_tick():
    order = []
    n = _node(lambda: order.append('plan'))
    n._tick_pub.publish = lambda msg: order.append(list(msg.data))
    LoadPlanner._plan_timer(n)
    assert order == ['plan', [pytest.approx(12.34), pytest.approx(1.0 / PLANNER_HZ)]]


def test_marker_even_when_the_tick_raises():
    def boom():
        raise RuntimeError('solver')
    n = _node(boom)
    with pytest.raises(RuntimeError):
        LoadPlanner._plan_timer(n)
    assert len(n._tick_pub.sent) == 1


def test_no_marker_on_wall_time():
    n = _node(lambda: None)
    n._tick_pub = None
    LoadPlanner._plan_timer(n)          # rig: no publisher, nothing to do

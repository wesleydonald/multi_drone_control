"""Card 2026-10-08_lag_h1: ref_hold_s applies each plan that long after it arrives (node clock);
the newest due plan wins; 0 applies at once."""
from collections import deque
from types import SimpleNamespace

import pytest

pytest.importorskip('acados_template')
from tracker.tracker_node import Controller as C  # noqa: E402


def _tracker(hold):
    now = [10.0]
    applied = []
    f = SimpleNamespace(ref_hold_s=hold, _held_refs=deque(),
                        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(
                            nanoseconds=int(now[0] * 1e9))))
    f._apply_planner_ref = lambda data: applied.append(data[2])
    f.now, f.applied = now, applied
    return f


def test_held_until_due_and_the_newest_wins():
    f = _tracker(0.035)
    C._planner_ref_callback(f, SimpleNamespace(data=[1.0, 0.1, 1.0]))
    f.now[0] = 10.02
    C._planner_ref_callback(f, SimpleNamespace(data=[1.0, 0.1, 2.0]))
    C._release_held_refs(f)
    assert f.applied == []
    f.now[0] = 10.036
    C._release_held_refs(f)
    assert f.applied == [1.0]
    f.now[0] = 10.06
    C._release_held_refs(f)
    assert f.applied == [1.0, 2.0] and not f._held_refs


def test_zero_applies_at_once():
    f = _tracker(0.0)
    C._planner_ref_callback(f, SimpleNamespace(data=[1.0, 0.1, 3.0]))
    assert f.applied == [3.0]

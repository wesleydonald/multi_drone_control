"""The position limit at the planner's one reference publisher (Wesley 22 Oct; pos_limit_m, off until the critic): every
drone reference, the detach mode's too, leaves POS_LIMIT_REF_MARGIN_M inside +-pos_limit_m in x and y, a velocity
carrying a held node further out zeroed; off, the wire is as it was."""
from types import SimpleNamespace

import numpy as np

from mpc_planner.planner_node import LoadPlanner


class _Pub:
    def __init__(self):
        self.msgs = []

    def publish(self, m):
        self.msgs.append(np.array(m.data))


class _Log:
    def __init__(self):
        self.lines = []

    def warn(self, m, *a, **k):
        self.lines.append(m)


def _planner(limit):
    log = _Log()
    f = SimpleNamespace(N=2, dt=0.1, ref_pub=[_Pub()], _pos_limit_m=limit)
    f.get_logger = lambda: log
    f._inside_limit = lambda i, nodes: LoadPlanner._inside_limit(f, i, nodes)
    return f, log


def _nodes():
    """A horizon running out along +x past 1.75 m at 0.3 m/s, its thrust and pull feed-forward."""
    a, ac = np.array([0.0, 0.0, 9.81]), np.array([0.1, 0.0, 2.0])
    return [(np.array([x, -0.2, 1.0]), np.array([0.3, -0.1, 0.0]), a, ac) for x in (1.6, 1.68, 1.8)]


def _rows(f):
    return np.array(f.ref_pub[0].msgs[-1][2:]).reshape(3, 12)


def test_off_the_wire_is_unchanged():
    f, log = _planner(0.0)
    LoadPlanner._publish_ref(f, 0, _nodes())
    r = _rows(f)
    assert np.allclose(r[:, 0], [1.6, 1.68, 1.8]) and np.allclose(r[:, 3], 0.3) and not log.lines


def test_on_every_node_stays_inside_and_stops_at_the_limit():
    f, log = _planner(1.75)
    LoadPlanner._publish_ref(f, 0, _nodes())
    r = _rows(f)
    assert np.allclose(r[:, 0], [1.6, 1.68, 1.7])                # 1.75 - 0.05
    assert np.allclose(r[:, 3], [0.3, 0.3, 0.0])                 # the held node's onward speed zeroed
    assert np.allclose(r[:, 4], -0.1) and np.allclose(r[:, 6:12], np.r_[0.0, 0.0, 9.81, 0.1, 0.0, 2.0])
    assert len(log.lines) == 1 and '1 reference nodes' in log.lines[0]

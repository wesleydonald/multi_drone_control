"""The position limit on the tracker's side (Wesley 22 Oct; pos_limit_m, off until the critic): a reference from any
sender (the planner, the detach mode, free_hover) is flown POS_LIMIT_REF_MARGIN_M inside +-pos_limit_m in x and y, the
onward velocity of a held node zeroed; off, it is flown as received."""
from types import SimpleNamespace

import numpy as np
import pytest

pytest.importorskip('acados_template')
from std_msgs.msg import Float64MultiArray  # noqa: E402
from tracker.tracker_node import Controller  # noqa: E402

N = 2


def _tracker(limit=None):
    log = []
    f = SimpleNamespace(N=N, drone_id=0, get_logger=lambda: SimpleNamespace(warn=lambda m, **k: log.append(m)),
                        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=0)),
                        _safety_clock=SimpleNamespace(now=lambda: 0.0))
    if limit is not None:
        f.pos_limit_m = limit
    return f, log


def _msg():
    """Three nodes heading out along -y past -1.75 m at 0.4 m/s."""
    rows = [[0.2, y, 1.0, 0.0, -0.4, 0.0, 0.0, 0.0, 9.81, 0.0, 0.0, 2.0] for y in (-1.6, -1.72, -1.9)]
    return Float64MultiArray(data=[float(N + 1), 0.1] + [v for r in rows for v in r])


def test_off_or_unset_the_reference_is_flown_as_received():
    for limit in (None, 0.0):
        f, log = _tracker(limit)
        Controller._planner_ref_callback(f, _msg())
        assert np.allclose(f.planner_ref_pos[:, 1], [-1.6, -1.72, -1.9]) and np.allclose(f.planner_ref_vel[:, 1], -0.4)
        assert not log


def test_on_the_reference_is_held_inside_the_limit():
    f, log = _tracker(1.75)
    Controller._planner_ref_callback(f, _msg())
    assert np.allclose(f.planner_ref_pos[:, 1], [-1.6, -1.7, -1.7])
    assert np.allclose(f.planner_ref_vel[:, 1], [-0.4, 0.0, 0.0]) and np.allclose(f.planner_ref_pos[:, 0], 0.2)
    assert np.allclose(f.planner_ref_acc[:, 2], 9.81) and np.allclose(f.planner_ref_cable[:, 2], 2.0)
    assert len(log) == 1 and '2 nodes' in log[0]

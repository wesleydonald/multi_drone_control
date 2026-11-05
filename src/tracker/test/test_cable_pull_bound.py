"""cable_pull_up_max (docs/analysis/tracker_clamp_2026-10-31.md, option B): the measured pull's upward z is clamped at
the bound, 0 by default (a rod only pulls down); x and y always pass."""
from types import MethodType, SimpleNamespace

import numpy as np

from tracker.tracker_node import Controller as N  # noqa: N814


def _stub(bound=None):
    st = SimpleNamespace(cable_lp_hz=0.0, _lp_cable=[], _notch=None, _notch_z=None, _meas_stats=[0, 0, 0, 0.0],
                         cable_accel_cap=14.5, _cable_lp_alpha=1.0, _applied_cable_vec=np.array([0.1, -0.2, 0.3]))
    if bound is not None:
        st.cable_pull_up_max = bound
    st._condition_measured_cable = MethodType(N._condition_measured_cable, st)
    return st


def test_default_clamps_an_upward_pull_to_zero_and_passes_xy():
    out = _stub()._condition_measured_cable(np.array([0.1, -0.2, 0.3]))
    np.testing.assert_allclose(out, [0.1, -0.2, 0.0])
    out = _stub(0.0)._condition_measured_cable(np.array([0.1, -0.2, 0.3]))
    np.testing.assert_allclose(out, [0.1, -0.2, 0.0])


def test_a_bound_lets_the_upward_pull_through_up_to_it():
    np.testing.assert_allclose(_stub(0.5)._condition_measured_cable(np.array([0.1, -0.2, 0.3])), [0.1, -0.2, 0.3])
    st = _stub(0.5)
    st._applied_cable_vec = np.array([0.0, 0.0, 0.5])
    np.testing.assert_allclose(st._condition_measured_cable(np.array([0.0, 0.0, 2.0])), [0.0, 0.0, 0.5])


def test_a_pull_down_is_untouched_by_the_bound():
    for b in (None, 0.5):
        st = _stub(b)
        st._applied_cable_vec = np.array([0.0, 0.0, -5.0])
        np.testing.assert_allclose(st._condition_measured_cable(np.array([0.0, 0.0, -5.0])), [0.0, 0.0, -5.0])

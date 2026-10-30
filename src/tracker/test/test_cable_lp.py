"""cable_lp_hz (RIG-1029-h4): a first-order low-pass on the measured pull, one or two stages (cable_lp_order). Off, the
pull goes through as before; one stage at 2 Hz cuts the rig ring's 5.3 Hz bending to about a third, two to about an
eighth, while a steady pull and the 0.3 Hz sway come through."""
import math
from types import MethodType, SimpleNamespace

import numpy as np

from tracker.tracker_node import DT, Controller as N  # noqa: N814


def _stub(lp_hz, order=1):
    st = SimpleNamespace(cable_lp_hz=lp_hz, _lp_cable=[None] * order, _notch=None, _notch_z=None,
                         _meas_stats=[0, 0, 0, 0.0], cable_accel_cap=14.5,
                         _cable_lp_alpha=1.0 - math.exp(-2.0 * math.pi * lp_hz * DT) if lp_hz > 0.0 else 1.0,
                         _applied_cable_vec=np.array([0.0, 0.0, -5.0]))
    st._condition_measured_cable = MethodType(N._condition_measured_cable, st)
    return st


def _gain(lp_hz, f_hz, amp=0.5, seconds=6.0, order=1):
    """Applied amplitude over measured amplitude for a pull of -5 m/s^2 with a vertical sine of f_hz on top."""
    st = _stub(lp_hz, order)
    out = []
    for k in range(int(seconds / DT)):
        meas = np.array([0.0, 0.0, -5.0 + amp * math.sin(2.0 * math.pi * f_hz * k * DT)])
        out.append(st._condition_measured_cable(meas)[2])
    tail = np.array(out[len(out) // 2:])
    return (tail.max() - tail.min()) / (2.0 * amp), tail.mean()


def test_off_passes_the_pull_through():
    g, mean = _gain(0.0, 5.3)
    assert g > 0.95 and abs(mean + 5.0) < 0.05


def test_on_cuts_the_rings_bending_and_keeps_the_steady_pull_and_the_sway():
    g, mean = _gain(2.0, 5.3)
    assert 0.25 < g < 0.45                     # first order at 2 Hz: 0.35 at 5.3 Hz
    assert abs(mean + 5.0) < 0.05              # the steady pull is not biased
    assert _gain(2.0, 0.3)[0] > 0.95           # the ring's slow sway still comes through


def test_two_stages_cut_the_bending_to_an_eighth_and_keep_the_sway():
    g, mean = _gain(2.0, 5.3, order=2)
    assert 0.08 < g < 0.18                     # two first-order stages at 2 Hz: 0.125 at 5.3 Hz
    assert abs(mean + 5.0) < 0.05
    assert _gain(2.0, 0.3, order=2)[0] > 0.9


def test_the_filter_starts_from_its_first_sample():
    st = _stub(2.0, order=2)
    st._condition_measured_cable(np.array([0.0, 0.0, -5.0]))
    for stage in st._lp_cable:
        np.testing.assert_allclose(stage, [0.0, 0.0, -5.0])


# ---- cable_notch_hz: the band-stop at the ring's bending (RIG-1029-sq2_2/-sq2_4) ------------------------------------

def _notch_stub(f0=5.3, q=1.0):
    from tracker.tracker_node import notch_coeffs
    st = _stub(0.0)
    st._notch, st._notch_z = notch_coeffs(f0, q, DT), None
    return st


def _notch_gain_and_lag(f_hz, amp=0.5, seconds=10.0):
    st = _notch_stub()
    meas, out = [], []
    for k in range(int(seconds / DT)):
        m = -5.0 + amp * math.sin(2.0 * math.pi * f_hz * k * DT)
        meas.append(m)
        out.append(st._condition_measured_cable(np.array([0.0, 0.0, m]))[2])
    a, b = np.array(meas[len(meas) // 2:]) + 5.0, np.array(out[len(out) // 2:]) + 5.0
    lags = [np.dot(a[:len(a) - L], b[L:]) for L in range(25)]
    return (b.max() - b.min()) / (2.0 * amp), int(np.argmax(lags)) * DT, np.mean(out[len(out) // 2:])


def test_the_notch_removes_the_bending_and_barely_delays_the_swing():
    g53, _, mean = _notch_gain_and_lag(5.3)
    assert g53 < 0.1 and abs(mean + 5.0) < 0.05          # the bending gone, the steady pull kept
    g11, lag11, _ = _notch_gain_and_lag(1.1)
    assert g11 > 0.95 and lag11 <= 0.04                  # the 1.1 Hz swing through, ~30 ms late (two 2 Hz stages: ~145)


def test_the_notch_starts_at_rest_on_its_first_sample():
    st = _notch_stub()
    for _ in range(5):
        y = st._condition_measured_cable(np.array([0.0, 0.0, -5.0]))
    np.testing.assert_allclose(y, [0.0, 0.0, -5.0], atol=1e-9)

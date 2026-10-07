"""cable_source mocap: a throttle step with an unchanged cable pull must not read as a pull change
(card 2026-10-07_cable_mocap, critic must-fix 1)."""
from collections import deque
from types import MethodType, SimpleNamespace

import numpy as np


def _stub():
    from tracker.tracker_node import Controller as N  # noqa: N814
    clock = SimpleNamespace(t=0.0)
    st = SimpleNamespace(_vel_hist=deque(maxlen=12), _acc_lpf=None, _thr_w_lpf=None, _acc_t=None,
                         current_pose=np.zeros(13),
                         get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(
                             nanoseconds=int(clock.t * 1e9))))
    st._mocap_accel = MethodType(N._mocap_accel, st)
    return st, clock


def _pull_through_step(sync=True):
    st, clock = _stub()
    g, c = 9.81, -5.0                      # constant downward cable pull on the drone (m/s^2)
    v, worst = 0.0, 0.0
    dt = 0.02
    for k in range(100):
        t = k * dt
        thrust = g - c + (1.0 if t >= 1.0 else 0.0)      # a +1 m/s^2 throttle step at 1 s
        a_true = thrust - g + c
        v += a_true * dt
        clock.t = t
        st.current_pose[9] = v
        a, f = st._mocap_accel(np.array([0.0, 0.0, thrust]))
        if a is None:
            continue
        thr_term = f[2] if sync else thrust
        est = a[2] - thr_term + g
        if t > 0.5:
            worst = max(worst, abs(est - c))
    return worst


def test_throttle_step_does_not_read_as_a_pull_change():
    assert _pull_through_step(sync=True) < 0.25


def test_the_raw_thrust_term_would():
    assert _pull_through_step(sync=False) > 0.5

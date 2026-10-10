"""partner_rig_state (ours, combined M1 on the rig): his pendulum input from our mocap must be exactly what his
own IRL publisher computes from the same QUAD4 body and magnet marker (motion_capture_publisher_irl), since on the
combined rig our mocap node is the only Motive owner and his publisher does not run."""
import math
import time
from types import SimpleNamespace

import numpy as np
import pytest

from tejen_mission.partner_rig_state import PendulumRates, pendulum_angles

irl = pytest.importorskip('tejen_mission.motion_capture_publisher_irl')


def _his(body, q_wxyz, tip, t_body, t_tip, prev=None, offset=(0.0, 0.0, -0.030)):
    """motion_capture_publisher_irl.MotionCapturePublisher.compute_pendulum_state on a stand-in node."""
    cls = irl.MotionCapturePublisher
    fake = SimpleNamespace(
        latest_quad_data=irl.ObjectData('14', tuple(body), tuple(q_wxyz), (0, 0, 0), (0, 0, 0), t_body),
        latest_magnet_data=irl.ObjectData('24', tuple(tip), (1, 0, 0, 0), (0, 0, 0), (0, 0, 0), t_tip),
        max_pair_age_s=1e9, attach_offset_body=np.array(offset),
        last_pendulum_angles=None if prev is None else prev[0], last_pendulum_time=None if prev is None else prev[1],
        pendulum_rate_buffers={'phi_dot': irl.deque(maxlen=8), 'theta_dot': irl.deque(maxlen=8)},
        get_logger=lambda: SimpleNamespace(warn=lambda *_: None))
    fake.quad_attachment_position = lambda q: cls.quad_attachment_position(fake, q)
    fake.apply_pendulum_rate_filter = lambda a, b: cls.apply_pendulum_rate_filter(fake, a, b)
    return cls.compute_pendulum_state(fake), fake


def _q(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = (f(0.5 * a) for a in (roll, pitch, yaw) for f in (math.cos, math.sin))
    return (cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy)


def test_angles_match_his_irl_publisher():
    rng = np.random.default_rng(3)
    for _ in range(200):
        body = rng.uniform([-1, -1, 0.5], [1, 1, 2.5])
        q = _q(*rng.uniform(-0.4, 0.4, 2), rng.uniform(-math.pi, math.pi))
        tip = body + rng.uniform([-0.2, -0.2, -0.62], [0.2, 0.2, -0.45])
        ours = pendulum_angles(body, q, tip, (0.0, 0.0, -0.030))
        now = time.time()
        (phi, theta, _pd, _td, length), _ = _his(body, q, tip, now, now)
        assert ours == pytest.approx((phi, theta, length), abs=1e-9)


def test_hanging_straight_down_is_zero_with_the_anchor_length():
    phi, theta, length = pendulum_angles((0, 0, 1.6), (1, 0, 0, 0), (0, 0, 1.0), (0, 0, -0.030))
    assert (phi, theta) == (0.0, 0.0) and length == pytest.approx(0.57)
    assert pendulum_angles((0, 0, 1.0), (1, 0, 0, 0), (0, 0, 0.97), (0, 0, -0.030)) is None


def test_rates_are_his_eight_sample_mean_over_the_marker_times():
    rates, t, phi, theta = PendulumRates(), 0.0, 0.0, 0.0
    assert rates.step(t, phi, theta) == (0.0, 0.0)
    got = []
    for k in range(1, 12):
        t, phi, theta = 0.01 * k, 0.02 * k * k, -0.01 * k
        got.append(rates.step(t, phi, theta))
    raw_phi = [(0.02 * k * k - 0.02 * (k - 1) ** 2) / 0.01 for k in range(1, 12)]
    assert got[-1][0] == pytest.approx(np.mean(raw_phi[-8:]))
    assert got[-1][1] == pytest.approx(-1.0)
    assert rates.step(t, phi, theta) == (0.0, 0.0)             # a repeated time: no rate, as his

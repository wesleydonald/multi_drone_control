"""The switchable mocap velocity filter (card docs/experiments/2026-10-14_tracker_timing.md)."""
import numpy as np
import pytest

from drone_communication.velocity_filter import VelocityFilter


def _run(f, ts, ps):
    return [f.update(t, p) for t, p in zip(ts, ps)]


def test_ema_reproduces_the_rig_filter():
    """'ema' is the mocap node's historical path: the difference, then a*v + (1-a)*v_prev."""
    rng = np.random.default_rng(1)
    ts = np.cumsum(rng.uniform(0.009, 0.013, 60))
    ps = np.cumsum(rng.normal(0, 0.002, (60, 3)), axis=0)
    out = _run(VelocityFilter('ema', alpha=0.2), ts, ps)
    v = None
    for i in range(1, 60):
        diff = (ps[i] - ps[i - 1]) / (ts[i] - ts[i - 1])
        v = diff if v is None else 0.2 * diff + 0.8 * v
        assert np.allclose(out[i], v)
    assert out[0] is None


def test_slope_is_exact_on_steady_motion_and_none_is_the_difference():
    ts = np.arange(10) * 0.011
    ps = np.outer(ts, [0.3, -0.2, 0.15]) + [1.0, 2.0, 0.5]
    for mode in ('slope', 'none'):
        out = _run(VelocityFilter(mode, frames=4), ts, ps)
        assert all(np.allclose(o, [0.3, -0.2, 0.15]) for o in out[1:])


def test_slope_lags_less_than_ema_after_a_velocity_step():
    """At 90 Hz: 'ema' (0.2) needs ~4x the frames 'slope' (4 frames) does to reach 90 % of a velocity step."""
    ts = np.arange(80) / 90.0
    z = np.where(ts < 0.3, 0.0, (ts - 0.3) * 0.5)
    ps = np.stack([np.zeros_like(z), np.zeros_like(z), z], axis=1)
    def frames_to_90(mode):
        out = _run(VelocityFilter(mode, frames=4), ts, ps)
        k0 = int(np.argmax(ts >= 0.3))
        return next(i - k0 for i in range(k0, 80) if out[i] is not None and out[i][2] >= 0.45)
    assert frames_to_90('slope') <= 4 < frames_to_90('ema')


def test_rejects_bad_settings_and_out_of_order_samples():
    with pytest.raises(ValueError):
        VelocityFilter('kalman')
    with pytest.raises(ValueError):
        VelocityFilter('slope', frames=1)
    f = VelocityFilter('slope')
    f.update(1.0, (0, 0, 0))
    assert f.update(1.0, (1, 1, 1)) is None

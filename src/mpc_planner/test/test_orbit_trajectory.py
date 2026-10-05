import numpy as np
from mpc_planner.load_trajectory import LoadTrajectory, ORBIT_RAMP_S


def test_orbit_never_completes_and_reaches_constant_speed():
    tr = LoadTrajectory('orbit', 0.2, 1.0, 0.5)
    assert not tr.complete(1e6)
    for t in (ORBIT_RAMP_S + 1.0, 60.0, 300.0):
        _, _, vx, vy = tr.offset_at(t)
        assert np.isclose(np.hypot(vx, vy), 0.2, atol=1e-9)
        dx, dy, _, _ = tr.offset_at(t)
        assert np.isclose(np.hypot(dx, dy - 0.5), 0.5, atol=1e-9)    # on the circle


def test_orbit_is_smooth_and_consistent():
    tr = LoadTrajectory('orbit', 0.2, 1.0, 0.5)
    h = 1e-4
    for t in np.linspace(0.01, 12.0, 200):
        dx0, dy0, vx, vy = tr.offset_at(t)
        dx1, dy1, _, _ = tr.offset_at(t + h)
        assert np.isclose((dx1 - dx0) / h, vx, atol=2e-3) and np.isclose((dy1 - dy0) / h, vy, atol=2e-3)
        ax, ay = tr.accel_at(t)
        _, _, vx1, vy1 = tr.offset_at(t + h)
        assert np.isclose((vx1 - vx) / h, ax, atol=5e-3) and np.isclose((vy1 - vy) / h, ay, atol=5e-3)
    assert tr.offset_at(0.0) == (0.0, 0.0, 0.0, 0.0)

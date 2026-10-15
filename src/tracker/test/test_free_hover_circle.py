"""free_hover's circle: starts and ends at rest on the centre, laps at the radius, and its velocity and
acceleration are the derivatives of its position (the centripetal pull points inward)."""
import numpy as np

from tracker.free_hover import circle_offset, circle_state

R, T, LAPS = 0.5, 6.0, 3
END = (LAPS + 2) * T


def test_rests_on_the_centre_before_and_after():
    for t in (-1.0, 0.0, END, END + 1.0):
        d, v, a = circle_state(t, R, T, LAPS)
        assert np.allclose(d, 0.0) and np.allclose(v, 0.0, atol=1e-3) and np.allclose(a, 0.0, atol=0.2)


def test_laps_at_the_radius_with_the_inward_pull():
    w = 2.0 * np.pi / T
    for t in np.linspace(T + 0.1, (LAPS + 1) * T - 0.1, 50):
        d, v, a = circle_state(t, R, T, LAPS)
        assert abs(np.linalg.norm(d) - R) < 1e-9
        assert abs(np.linalg.norm(v) - R * w) < 1e-4
        assert np.allclose(a, -w * w * d, atol=1e-3)
        assert np.cross(d, v) > 0.0                      # counter-clockwise from above


def test_never_outside_the_radius_and_continuous():
    ts = np.arange(-0.5, END + 0.5, 0.01)
    ds = np.array([circle_offset(t, R, T, LAPS) for t in ts])
    assert np.max(np.linalg.norm(ds, axis=1)) <= R + 1e-9
    assert np.max(np.linalg.norm(np.diff(ds, axis=0), axis=1)) < R * 2.0 * np.pi / T * 0.01 * 1.01
    vs = np.array([circle_state(t, R, T, LAPS)[1] for t in ts])
    assert np.max(np.linalg.norm(np.diff(vs, axis=0), axis=1)) < 0.02      # no step in the velocity
    acc = np.array([circle_state(t, R, T, LAPS)[2] for t in ts])
    assert np.max(np.linalg.norm(acc, axis=1)) < 1.2 * R * (2.0 * np.pi / T) ** 2

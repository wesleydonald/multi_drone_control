"""The position limit's pure helpers (Wesley 22 Oct): a point and a reference horizon kept inside +-bound in x and y,
the velocity carrying a held node further out zeroed; a bound of 0 changes nothing.

    python3 -m pytest src/utility_objects/test/test_position_limit.py -v
"""
import numpy as np
import pytest

from utility_objects.position_limit import breach, clear_point, limit_horizon, limit_ray, limit_xy


def test_a_point_is_clamped_in_x_and_y_only():
    assert np.allclose(limit_xy([2.0, -1.8, 0.3], 1.65), [1.65, -1.65, 0.3])
    assert np.allclose(limit_xy([0.2, 0.1, 0.3], 1.65), [0.2, 0.1, 0.3])
    p = np.array([5.0, 5.0, 5.0])
    assert np.allclose(limit_xy(p, 0.0), p) and limit_xy(p, 0.0) is not p     # off: a copy, unchanged


def test_a_horizon_stops_at_the_bound_and_keeps_its_inward_speed():
    pos = [[1.6, 0.0, 1.0], [1.8, -1.9, 1.0], [1.9, 0.5, 1.0]]
    vel = [[0.3, 0.0, 0.0], [0.3, -0.2, 0.1], [-0.3, 0.2, 0.0]]
    p, v, k = limit_horizon(pos, vel, 1.7)
    assert k == 2 and np.allclose(p[:, :2], [[1.6, 0.0], [1.7, -1.7], [1.7, 0.5]])
    assert np.allclose(v, [[0.3, 0.0, 0.0], [0.0, 0.0, 0.1], [-0.3, 0.2, 0.0]])   # onward zeroed, inward kept
    p0, v0, k0 = limit_horizon(pos, vel, 0.0)
    assert k0 == 0 and np.allclose(p0, pos) and np.allclose(v0, vel)


def test_a_breach_is_past_the_bound_in_x_or_y():
    assert breach((1.76, 0.0), 1.75) and breach((0.0, -1.76), 1.75)
    assert not breach((1.75, 1.75), 1.75) and not breach((3.0, 3.0), 0.0)


def test_a_step_out_is_shortened_along_its_own_ray():
    """Critic m1 (22 Oct): from a ring at (0.9, 0), a step to the 1.2 m clearance through (1.53, 0.36) keeps its 30 deg
    bearing (x and y clamped apart, it turned to 63 deg); a point inside, or a bound of 0, is left as it is."""
    c, p = np.array([0.9, 0.0]), np.array([0.9 + 1.2 * np.cos(np.radians(29.7)), 1.2 * np.sin(np.radians(29.7)), 1.5])
    q = limit_ray(c, p, 1.65)
    assert q[0] == pytest.approx(1.65) and q[2] == 1.5
    assert np.degrees(np.arctan2(q[1], q[0] - 0.9)) == pytest.approx(29.7)
    assert np.allclose(limit_ray(c, [1.2, 0.3, 1.0], 1.65), [1.2, 0.3, 1.0]) and np.allclose(limit_ray(c, p, 0.0), p)
    assert np.allclose(limit_ray([2.0, 0.0], [2.5, 1.0, 1.0], 1.65), [1.65, 1.0, 1.0])   # from outside: x, y apart
    q = limit_ray([0.0, 0.0], [2.0, -2.5, 0.0], 1.65)                                       # the nearer side first
    assert q[1] == pytest.approx(-1.65) and q[0] == pytest.approx(1.32)


def test_the_clearance_point_is_the_inside_point_of_the_circle_nearest_the_bearing():
    c = (0.9, 0.0)
    assert np.allclose(clear_point(c, (0.0, 1.0), 1.2, 1.65), [0.9, 1.2])                  # its own bearing, inside
    p = clear_point(c, (np.cos(np.radians(30.0)), np.sin(np.radians(30.0))), 1.2, 1.65)
    assert p[0] == pytest.approx(1.65) and np.degrees(np.arctan2(p[1], p[0] - 0.9)) == pytest.approx(51.3, abs=0.1)
    p = clear_point(c, (np.cos(np.radians(-30.0)), np.sin(np.radians(-30.0))), 1.2, 1.65)
    assert p[0] == pytest.approx(1.65) and p[1] < 0.0                                       # the near end, not the far
    assert clear_point((0.0, 0.0), (1.0, 0.0), 1.2, 0.8) is None                  # the square inside the circle
    assert clear_point((3.0, 0.0), (1.0, 0.0), 1.0, 1.65) is None                           # all of it outside

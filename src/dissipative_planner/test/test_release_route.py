"""release_route (Wesley 4 Nov): a partner drone released from plate 9 to re-attach on another plate goes out to the
circle past the carriers and round it by the shorter way, at its step-out height, ending on the new plate's line."""
import math

import numpy as np

from dissipative_planner.dissipative_node import release_route  # noqa: E402


def _az(p, c):
    return math.degrees(math.atan2(p[1] - c[1], p[0] - c[0]))


def test_round_the_outside_to_plate_1():
    c = np.array([0.1, -0.05])
    start = np.array([0.1, -0.05 + 0.75, 2.2])        # stepped out on plate 9's line (+y, 90 deg ccw)
    goal = (math.cos(math.radians(-30.0)), math.sin(math.radians(-30.0)))   # plate 1, 30 deg clockwise
    wp = release_route(c, start, goal, 1.2)
    assert len(wp) >= 3
    assert all(abs(np.linalg.norm(w[:2] - c) - 1.2) < 1e-9 and w[2] == 2.2 for w in wp)
    assert abs(_az(wp[0], c) - 90.0) < 1e-6 and abs(_az(wp[-1], c) + 30.0) < 1e-6
    steps = [abs((_az(b, c) - _az(a, c) + 180.0) % 360.0 - 180.0) for a, b in zip(wp, wp[1:])]
    assert max(steps) <= 15.0 + 1e-9 and abs(sum(steps) - 120.0) < 1e-6      # the shorter way, 120 deg


def test_same_plate_needs_no_route_and_a_far_start_keeps_its_radius():
    c = np.zeros(2)
    assert release_route(c, np.array([0.0, 0.9, 2.0]), (0.0, 1.0), 1.2) == []
    wp = release_route(c, np.array([0.0, 1.5, 2.0]), (1.0, 0.0), 1.2)
    assert all(abs(np.linalg.norm(w[:2]) - 1.5) < 1e-9 for w in wp)


def test_a_partner_release_target_past_the_limit_comes_inside_it_with_the_limit_on_or_off():
    """R3683: on the orbit the step-out target (0.5 m out, 0.3 m up) was 1.82 m out, past the freed drone's 1.70 m
    clip, so it never read still and PARTNER RELEASE never came. _resize_and_release now clips it with the rejoin
    limit (pos_limit_m, or 1.75 m while it is off); a target inside is unchanged."""
    from types import SimpleNamespace
    from dissipative_planner.dissipative_node import _land_clear, POSITION_LIMIT_M
    c, hold = np.array([0.49, 0.45]), np.array([0.48, 1.32, 1.95])
    for pos_limit in (0.0, 1.75):
        node = SimpleNamespace(_pos_limit_m=pos_limit)
        out = _land_clear(node, c, hold + [0.0, 0.5, 0.3], 1.0, limit=POSITION_LIMIT_M)
        assert out[1] <= POSITION_LIMIT_M - 0.1 + 1e-9 and np.isclose(out[2], 2.25)
        inside = hold + [0.0, 0.2, 0.3]
        assert np.allclose(_land_clear(node, c, inside, 1.0, limit=POSITION_LIMIT_M), inside)

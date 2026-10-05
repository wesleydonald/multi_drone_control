"""land_arc_ref: the LAND unwind keeps the rod pivot on its circle about the grounded attach
point, so a rigid rod has nothing to push against (the straight descent shoved every twin
drone 18-26 cm outward after load down and tipped two, R0793/R0816)."""
import numpy as np

from mpc_planner.planner_node import land_arc_ref

ATTACH = np.array([0.225, 0.0, 0.04])
RADIAL = np.array([1.0, 0.0, 0.0])
ROD = 0.55
PIVOT_UP = np.array([0.0, 0.0, 0.04])     # rig pivot 4 cm below the drone centre
V = 0.2


def _ref(theta0_deg, s):
    return land_arc_ref(ATTACH, RADIAL, ROD, np.radians(theta0_deg), PIVOT_UP, s, V)


def test_starts_where_the_drone_is():
    pos, _ = _ref(50.0, 0.0)
    th = np.radians(50.0)
    expect = ATTACH + ROD * np.array([np.cos(th), 0.0, np.sin(th)]) + PIVOT_UP
    assert np.allclose(pos, expect)


def test_pivot_stays_on_the_rod_circle_and_moves_out_and_down():
    prev = None
    for s in np.linspace(0.0, np.radians(50.0) * ROD, 30):
        pos, vel = _ref(50.0, s)
        assert abs(np.linalg.norm(pos - PIVOT_UP - ATTACH) - ROD) < 1e-9
        assert vel[0] >= -1e-12 and vel[2] <= 1e-12
        assert abs(np.linalg.norm(vel) - V) < 1e-9
        if prev is not None:
            assert pos[0] >= prev[0] - 1e-12 and pos[2] <= prev[2] + 1e-12
        prev = pos


def test_ends_at_the_rest_radius_then_descends_straight():
    arc = np.radians(50.0) * ROD
    pos, vel = _ref(50.0, arc + 0.01)
    assert np.isclose(pos[0], ATTACH[0] + ROD)
    assert np.isclose(pos[2], ATTACH[2] + PIVOT_UP[2] - 0.01)
    assert np.allclose(vel, [0.0, 0.0, -V])


def test_velocity_matches_the_position_change():
    s, ds = 0.1, 1e-6
    p0, v0 = _ref(55.0, s)
    p1, _ = _ref(55.0, s + ds)
    assert np.allclose((p1 - p0) / ds * V, v0, atol=1e-5)


def test_never_below_the_floor():
    pos, vel = _ref(10.0, 5.0)
    assert pos[2] == 0.0 and vel[2] == 0.0


def test_touched_down_holds_still():
    _, vel = land_arc_ref(ATTACH, RADIAL, ROD, np.radians(40.0), PIVOT_UP, 0.1, 0.0)
    assert np.allclose(vel, 0.0)

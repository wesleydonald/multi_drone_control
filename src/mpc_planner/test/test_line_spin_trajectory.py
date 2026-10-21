"""line_spin (Wesley 21 Oct, plan_thesis_results_2026-10-21 A): out along +x by distance and back on two smootherstep
legs, the load yawing at the spin's rate (one radian per radius travelled) and holding the turned yaw afterwards."""
import types

import numpy as np
import pytest

from mpc_planner.geometry import attach_points, nominal_cable_dirs
from mpc_planner.load_trajectory import KINDS, LoadTrajectory
from mpc_planner.reference_builder import ReferenceBuilder, rot_z


def trip(v=0.2, d=1.0, r=0.5):
    tr = LoadTrajectory('line_spin', v, d, r)
    return tr, tr._line_spin(0.0)[4]


def test_out_and_back_at_rest_at_both_ends_and_the_turnaround():
    tr, T = trip()
    assert np.isclose(T, 2 * 1.875 * 1.0 / 0.2)                       # 18.75 s at 0.2 m/s over 1 m
    for t in (1e-6, T / 2, T - 1e-6):
        _, _, vx, vy = tr.offset_at(t)
        assert abs(vx) < 1e-6 and vy == 0.0 and abs(tr.accel_at(t)[0]) < 1e-3
    assert np.isclose(tr.offset_at(T / 2)[0], 1.0)                    # the far end
    xs = [tr.offset_at(t) for t in np.linspace(0.0, T, 2001)]
    assert max(abs(o[2]) for o in xs) == pytest.approx(0.2, rel=1e-4)  # peak speed is the speed
    assert all(o[1] == 0.0 and o[3] == 0.0 for o in xs)               # a straight line along x
    assert tr.complete(T) and not tr.complete(T - 0.1)
    assert tr.offset_at(T + 5.0) == (0.0, 0.0, 0.0, 0.0)              # back at the start, at rest


def test_position_velocity_acceleration_and_yaw_are_consistent():
    tr, T = trip()
    h = 1e-4
    for t in np.linspace(0.01, T - 0.01, 400):
        x0, _, vx, _ = tr.offset_at(t)
        x1, _, vx1, _ = tr.offset_at(t + h)
        assert np.isclose((x1 - x0) / h, vx, atol=2e-3)
        assert np.isclose((vx1 - vx) / h, tr.accel_at(t)[0], rtol=1e-2, atol=5e-3)
        y0, w = tr.yaw_at(t)
        assert np.isclose((tr.yaw_at(t + h)[0] - y0) / h, w, atol=5e-3)


def test_the_load_turns_at_the_spins_rate_and_holds_the_turned_yaw():
    tr, T = trip()
    ys = [tr.yaw_at(t) for t in np.linspace(1e-3, T, 3001)]
    assert all(b[0] >= a[0] - 1e-12 for a, b in zip(ys, ys[1:]))      # one way only (counter-clockwise)
    assert max(w for _, w in ys) == pytest.approx(0.2 / 0.5, rel=1e-4)  # 0.4 rad/s = 23 deg/s at 0.2 m/s
    assert np.isclose(tr.yaw_at(T)[0], 2 * 1.0 / 0.5)                 # 4 rad = 229 deg
    assert tr.yaw_at(T + 30.0) == (pytest.approx(4.0), 0.0)            # held, not snapped back as spin's full turn
    assert tr.yaw_at(0.0) == (0.0, 0.0)
    spin = LoadTrajectory('spin', 0.2, 1.0, 0.5)                       # same peak rate as the spin at this speed
    T_spin = spin._circle_theta(0.0)[3]
    assert max(spin.yaw_at(t)[1] for t in np.linspace(0, T_spin, 2001)) == pytest.approx(0.4, rel=1e-3)


def test_the_hold_reference_keeps_the_turned_yaw():
    rho = attach_points(3, 0.225, 0.02)
    dyn = types.SimpleNamespace(rho=rho, m=0.86, g=9.81, nu=12, mr=0.0, lam=0.0)
    s_nom = nominal_cable_dirs(rho, 60.0)
    tr, T = trip()
    b = ReferenceBuilder(dyn, 3, s_nom, 0.1, tr)
    b.psi0 = 0.3
    b.update((0.0, 0.0), 0.0, 1.0, 1.0, 0.0, T + 2.0)
    s_hold = b.hold_yref(np.zeros(13))[12:21].reshape(3, 3)
    turned = rot_z(0.3 + 4.0)
    for i in range(3):
        assert np.allclose(s_hold[i], turned @ np.asarray(s_nom[i], float))
    b.update((0.0, 0.0), 0.0, 1.0, 1.0, 0.0, 0.0)                     # before the trip: the datum alone
    s0 = b.hold_yref(np.zeros(13))[12:21].reshape(3, 3)
    assert np.allclose(s0[0], rot_z(0.3) @ np.asarray(s_nom[0], float))


def test_line_spin_is_a_known_kind():
    assert 'line_spin' in KINDS and 'spin' in KINDS

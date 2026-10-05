"""balanced_tensions: equal on an even ring, zero net moment and positive on the uneven
30/90/150/270 ring, and the survivors 30/150/270 are even again."""
import numpy as np
from mpc_planner.geometry import (attach_points, nominal_cable_dirs,
                                  balanced_tensions)

M, G = 0.86, 9.81


def _check(az):
    rho = attach_points(len(az), 0.25, 0.025, az)
    s = nominal_cable_dirs(rho, 45.0)
    t = balanced_tensions(rho, s, M, G)
    F = sum(ti * si for ti, si in zip(t, s))
    Mo = sum(ti * np.cross(ri, si) for ti, ri, si in zip(t, rho, s))
    return t, F, Mo


def test_even_ring_is_the_equal_split():
    t, F, Mo = _check([0, 90, 180, 270])
    eq = M * G / (4 * np.sin(np.radians(45)))
    assert np.allclose(t, eq, rtol=1e-6)
    assert abs(F[2] + M * G) < 1e-6 and np.linalg.norm(Mo) < 1e-6


def test_uneven_ring_balances_force_and_moment():
    t, F, Mo = _check([30, 90, 150, 270])
    assert np.allclose(F, [0, 0, -M * G], atol=1e-6)
    assert np.linalg.norm(Mo) < 1e-6
    assert min(t) > 0.5 and t[1] < t[3]          # the 90 one shares with its 30/150 neighbours


def test_survivor_triple_is_even():
    t, F, Mo = _check([30, 150, 270])
    assert np.allclose(t, t[0], rtol=1e-6) and np.linalg.norm(Mo) < 1e-6

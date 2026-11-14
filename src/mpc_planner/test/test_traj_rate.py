"""The path clock's rate (Wesley 4 Nov, the attach pause on the orbit): the horizon's path times, velocities and
accelerations scale with it, so at rate 0 the reference is the ring at rest at the clock's point (R3683: a frozen
clock with the full path speed ahead let the ring coast 162 mm), and at rate 1 it is today's reference bit for bit."""
import types

import numpy as np

from mpc_planner.geometry import attach_points, nominal_cable_dirs
from mpc_planner.load_trajectory import LoadTrajectory
from mpc_planner.reference_builder import ReferenceBuilder

N_SLOT = 3


def builder(rate=None, kind='orbit', traj_t=6.0):
    rho = attach_points(N_SLOT, 0.225, 0.02)
    dyn = types.SimpleNamespace(rho=rho, m=0.86, g=9.81, nu=4 * N_SLOT, mr=0.0, lam=0.0)
    b = ReferenceBuilder(dyn, N_SLOT, nominal_cable_dirs(rho, 60.0), 0.1, LoadTrajectory(kind, 0.125, 0.5, 0.5))
    if rate is None:
        b.update((0.0, 0.0), 0.0, 1.0, 1.0, 0.0, traj_t)
    else:
        b.update((0.0, 0.0), 0.0, 1.0, 1.0, 0.0, traj_t, traj_rate=rate)
    return b


def test_rate_one_is_todays_reference_exactly():
    a, b = builder(None), builder(1.0)
    for k in (0, 5, 19):
        assert np.array_equal(a.yref_at(k), b.yref_at(k))


def test_rate_zero_is_the_ring_at_rest_at_the_clock_point():
    b, moving = builder(0.0), builder(1.0)
    y0 = b.yref_at(0)
    for k in (1, 10, 19):
        y = b.yref_at(k)
        assert np.allclose(y[0:2], y0[0:2]) and np.allclose(y[3:5], 0.0)
    assert np.allclose(y0[0:2], moving.yref_at(0)[0:2])                    # where the clock stands
    assert not np.allclose(moving.yref_at(19)[0:2], moving.yref_at(0)[0:2])


def test_half_rate_halves_the_speed_and_compresses_the_horizon():
    full, half = builder(1.0), builder(0.5)
    assert np.allclose(half.yref_at(0)[3:5], 0.5 * full.yref_at(0)[3:5])
    assert np.allclose(half.yref_at(10)[0:2], builder(1.0, traj_t=6.5).yref_at(0)[0:2])   # 1 s of horizon = 0.5 s of path

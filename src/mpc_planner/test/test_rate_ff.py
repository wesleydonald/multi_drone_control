"""The cable angular-rate reference (plan_rate_ff, card 2026-10-16_corner_cutting): zero when off or in hover; when
on, the rate the flatness directions turn at, across each rod (s x ds/dt), larger where the path's jerk is."""
import types

import numpy as np

from mpc_planner.geometry import attach_points, nominal_cable_dirs
from mpc_planner.load_trajectory import LoadTrajectory
from mpc_planner.reference_builder import ReferenceBuilder

N_SLOT = 3


def builder(kind, rate_ff, traj_t):
    rho = attach_points(N_SLOT, 0.225, 0.02)
    dyn = types.SimpleNamespace(rho=rho, m=0.86, g=9.81, nu=4 * N_SLOT, mr=0.0, lam=0.0)
    b = ReferenceBuilder(dyn, N_SLOT, nominal_cable_dirs(rho, 60.0), 0.1, LoadTrajectory(kind, 0.3, 1.0, 0.5))
    b.rate_ff = rate_ff
    b.update((0.0, 0.0), 0.0, 1.0, 1.0, 0.0, traj_t)
    return b


def r_part(y):
    o = 12 + 3 * N_SLOT + N_SLOT
    return y[o:o + 3 * N_SLOT].reshape(N_SLOT, 3)


def test_off_and_hover_give_zero_rates_and_on_leaves_everything_else_alone():
    for kind, on, t in (('fig_8', False, 5.0), ('hover', True, 5.0), ('fig_8', True, 0.0)):
        assert np.allclose(r_part(builder(kind, on, t).yref_at(3)), 0.0)
    a, b = builder('fig_8', False, 5.0).yref_at(3), builder('fig_8', True, 5.0).yref_at(3)
    o = 12 + 3 * N_SLOT + N_SLOT
    assert np.allclose(a[:o], b[:o]) and np.allclose(a[o + 3 * N_SLOT:], b[o + 3 * N_SLOT:])


def test_on_gives_the_turning_rate_across_each_rod_and_tracks_the_jerk():
    b = builder('fig_8', True, 0.0)
    s_idx = slice(12, 12 + 3 * N_SLOT)
    peak = {}
    for t in np.arange(4.5, 12.0, 0.1):
        b.update((0.0, 0.0), 0.0, 1.0, 1.0, 0.0, float(t))
        y = b.yref_at(0)
        s, r = y[s_idx].reshape(N_SLOT, 3), r_part(y)
        assert np.allclose((s * r).sum(axis=1), 0.0, atol=1e-9)        # across the rod
        peak[round(float(t), 1)] = float(np.linalg.norm(r, axis=1).max())
    # the planner's flatness tilt turns at about |jerk| / g: at 0.3 m/s up to ~0.27 rad/s on the figure-8's lobes
    assert 0.1 < max(peak.values()) < 0.6
    c = builder('circle', True, 6.0)
    assert float(np.linalg.norm(r_part(c.yref_at(0)), axis=1).max()) < 0.05      # a circle barely needs any


def test_ref_lag_evaluates_the_path_that_much_earlier():
    a, b = builder('fig_8', False, 6.0), builder('fig_8', False, 6.1)
    b.ref_lag_s = 0.1
    assert np.allclose(a.yref_at(4), b.yref_at(4))
    assert not np.allclose(builder('fig_8', False, 6.1).yref_at(4), b.yref_at(4))

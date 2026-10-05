import numpy as np
import pytest
from mpc_planner.reference_builder import blend_refs


def test_blend_endpoints_and_normalisation():
    s_from = [np.array([0.0, 0.0, -1.0])]
    s_nom = [np.array([-0.7071, 0.0, -0.7071])]
    s0, t0 = blend_refs(s_from, s_nom, [0.1], [2.8], 0.0)
    s1, t1 = blend_refs(s_from, s_nom, [0.1], [2.8], 1.0)
    assert np.allclose(s0[0], s_from[0]) and t0 == [0.1]
    assert np.allclose(s1[0], s_nom[0], atol=1e-4) and np.isclose(t1[0], 2.8)
    sh, th = blend_refs(s_from, s_nom, [0.1], [2.8], 0.5)
    assert np.isclose(np.linalg.norm(sh[0]), 1.0) and np.isclose(th[0], 1.45)


def test_blend_clamps_outside_unit_interval():
    s_from = [np.array([0.0, 0.0, -1.0])]; s_nom = [np.array([0.0, -1.0, 0.0])]
    assert np.allclose(blend_refs(s_from, s_nom, [0.0], [1.0], -3.0)[0][0], s_from[0])
    assert np.allclose(blend_refs(s_from, s_nom, [0.0], [1.0], 7.0)[0][0], s_nom[0])


def test_apex_direction_loads_an_off_radius_newcomer():
    from mpc_planner.geometry import nominal_cable_dirs, balanced_tensions, apex_direction
    az = [330, 90, 210]
    rho = [np.array([0.25*np.cos(np.radians(a)), 0.25*np.sin(np.radians(a)), 0.025]) for a in az]
    s = nominal_cable_dirs(rho, 45.0)
    rn = np.array([0.0, -0.28, 0.025])
    t_bad = balanced_tensions(rho + [rn], s + nominal_cable_dirs([rn], 45.0), 0.86)
    t_ok = balanced_tensions(rho + [rn], s + [apex_direction(rho, s, rn)], 0.86)
    assert t_bad[-1] <= 0.1 + 1e-9          # own-45-deg: no share (R0575/R0577)
    assert t_ok[-1] > 1.5                   # apex-pointing: a real share
    # an on-radius newcomer at 45 deg is unchanged
    r0 = np.array([0.0, -0.25, 0.025])
    assert np.allclose(apex_direction(rho, s, r0), nominal_cable_dirs([r0], 45.0)[0], atol=1e-6)


def test_rebalanced_blend_holds_the_load_level_midway():
    from mpc_planner.reference_builder import blend_refs, rebalance_incumbents
    from mpc_planner.geometry import (attach_points, nominal_cable_dirs,
                                      balanced_tensions, apex_direction)
    m = 0.86
    rho = attach_points(4, 0.25, 0.025, [150.0, -90.0, 30.0, 90.0])
    s_nom = nominal_cable_dirs(rho, 45.0)
    s_nom[-1] = apex_direction(rho[:-1], s_nom[:-1], rho[-1])
    t_nom = balanced_tensions(rho, s_nom, m)
    t3 = balanced_tensions(rho[:-1], s_nom[:-1], m)
    s_from = list(s_nom[:-1]) + [np.array([0.0, 0.0, -1.0])]
    s_b, t_b = blend_refs(s_from, s_nom, t3 + [0.1], t_nom, 0.5)
    t_r = rebalance_incumbents(rho, s_b, t_b, m)
    f = sum(t * s for t, s in zip(t_r, s_b))
    mom = sum(np.cross(r, t * s) for r, t, s in zip(rho, t_r, s_b))
    assert t_r[-1] == pytest.approx(t_b[-1])
    assert f[2] == pytest.approx(-m * 9.81, rel=1e-6)
    assert np.linalg.norm(mom[0:2]) < 1e-6

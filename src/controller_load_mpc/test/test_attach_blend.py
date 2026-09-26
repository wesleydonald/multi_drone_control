import numpy as np
from controller_load_mpc.reference_builder import blend_refs


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
    from controller_load_mpc.geometry import nominal_cable_dirs, balanced_tensions, apex_direction
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

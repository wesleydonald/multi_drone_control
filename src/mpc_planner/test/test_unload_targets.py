"""Soft detach (7 Oct): the unload target hands the leaver's share to the survivors, balanced,
and lands exactly on the split the n-1 reference builder starts from after the resize."""
from types import SimpleNamespace

import numpy as np

from mpc_planner.geometry import attach_points, balanced_tensions, nominal_cable_dirs
from mpc_planner.reference_builder import ReferenceBuilder, blend_refs, unload_tensions

M = 0.86


def _ring(az, elev=60.0):
    rho = attach_points(len(az), 0.225, 0.0, ','.join(str(a) for a in az))
    return rho, nominal_cable_dirs(rho, elev)


def _residual(rho, s, t):
    f = sum(ti * np.asarray(si) for ti, si in zip(t, s)) + np.array([0.0, 0.0, M * 9.81])
    mom = sum(np.cross(ri, ti * np.asarray(si)) for ri, ti, si in zip(rho, t, s))
    return float(np.linalg.norm(f)), float(np.linalg.norm(mom[0:2]))


def test_leaver_at_t_leave_survivors_near_the_n_minus_1_split():
    rho, s = _ring([30, 90, 180, 270])           # 1/3/6/9, drone at 90 leaves (r0013)
    t = unload_tensions(rho, s, M, 1, 0.5)
    keep = [0, 2, 3]
    assert t[1] == 0.5
    t3 = balanced_tensions([rho[i] for i in keep], [s[i] for i in keep], M)
    assert np.allclose([t[i] for i in keep], t3, atol=0.6)     # differs by the leaver's share
    assert np.allclose(unload_tensions(rho, s, M, 1, 0.0)[0:1] + unload_tensions(rho, s, M, 1, 0.0)[2:],
                       t3, atol=1e-9)                          # at t_leave 0 it IS that split


def test_r0013_step_is_this_target():
    rho, s = _ring([30, 90, 180, 270], elev=45.0)
    t0 = balanced_tensions(rho, s, M)
    t1 = unload_tensions(rho, s, M, 1, 0.0)
    pct = [100.0 * (t1[i] / t0[i] - 1.0) for i in (0, 2, 3)]
    assert np.allclose(pct, [64.0, 64.0, -35.0], atol=2.0)


def test_target_and_blend_hold_the_ring_level():
    for az, k in (([30, 90, 150, 270], 1), ([30, 90, 180, 270], 1)):
        rho, s = _ring(az)
        t0 = balanced_tensions(rho, s, M)
        t1 = unload_tensions(rho, s, M, k, 0.5)
        for a in (0.0, 0.25, 0.5, 0.75, 1.0):
            _, tb = blend_refs(s, s, t0, t1, a)
            f, mom = _residual(rho, s, tb)
            # the leaver's 0.5 N is counted, so the ring hangs level all the way
            assert f < 0.05 and mom < 0.005, (az, a, f, mom)


def test_retarget_is_continuous_and_reaches_the_target():
    rho, s = _ring([30, 90, 150, 270])
    rb = ReferenceBuilder(SimpleNamespace(rho=rho, m=M, g=9.81), 4, s, 0.1, None)
    t0 = list(rb._t_nom)
    t1 = unload_tensions(rho, s, M, 1, 0.5)
    rb.retarget(t1, 3.0)
    assert np.allclose(rb._refs_at(0)[1], t0)                  # no step at the switch
    for _ in range(30):
        rb.update((0, 0), 0.0, 0.0, 1.0, 0.0, 0.0)
    assert not rb.blend_active() and np.allclose(rb._refs_at(0)[1], t1)

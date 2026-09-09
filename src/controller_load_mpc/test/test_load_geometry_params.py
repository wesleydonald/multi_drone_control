"""
Attachment geometry as a RUNTIME parameter — THESIS_PLAN §12.2.

rho_i (attach points) and l_i (cable lengths) used to be baked into the CasADi model,
so a different attachment layout meant a ~50 s acados rebuild. That is impossible
mid-flight, which blocked two things the project needs: handing the fleet back to the
OCP after a reconfiguration, and describing a newcomer welded WHEREVER ITS MAGNET
LANDED rather than at a nominal ring point.

The property these tests protect is that the parameter now FULLY determines the
geometry — no residual baked dependence — because a model that silently kept using its
construction-time rho would plan against a fleet that no longer exists, and would look
perfectly healthy doing it.
"""
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from controller_load_mpc.load_cable_dynamics import LoadCableDynamics  # noqa: E402


def ring(n, radius=0.08, z=0.025):
    return [[radius * np.cos(2 * np.pi * k / n),
             radius * np.sin(2 * np.pi * k / n), z] for k in range(n)]


def make(n=3, rho=None, cable=0.5):
    return LoadCableDynamics(n_drones=n, load_mass=0.4, load_inertia=[1e-3] * 3,
                             cable_lengths=[cable] * n,
                             attach_points=rho if rho is not None else ring(n),
                             drone_mass=0.6)


def state(n, seed=0):
    """A non-degenerate state: tilted load, moving, taut cables, real tensions."""
    rng = np.random.default_rng(seed)
    d = make(n)
    x = rng.normal(scale=0.3, size=d.nx)
    x[6:10] = [0.94, 0.1, 0.2, 0.25]
    x[6:10] /= np.linalg.norm(x[6:10])
    for i in range(n):
        b = 13 + 14 * i
        s = rng.normal(size=3)
        x[b:b + 3] = s / np.linalg.norm(s)
        x[b + 12] = 2.0 + i
    return x


def test_the_parameter_fully_determines_the_geometry():
    """THE test. A model built on ring A, evaluated with ring B's parameters, must
    equal a model built on ring B. Any residual compile-time dependence shows up here
    as a mismatch -- and would mean the solver plans against a stale layout while
    reporting healthy solves."""
    n = 3
    a, b = make(n, rho=ring(n, 0.08)), make(n, rho=ring(n, 0.19, z=-0.04))
    fa, fb = a.load_cable_dynamics(), b.load_cable_dynamics()
    x, u = state(n), np.random.default_rng(1).normal(scale=0.2, size=a.nu)
    got = np.asarray(fa(x, u, b.geom_values())).ravel()
    want = np.asarray(fb(x, u, b.geom_values())).ravel()
    assert np.allclose(got, want, atol=1e-12)


def test_cable_length_is_also_live():
    n = 3
    a, b = make(n, cable=0.5), make(n, cable=0.9)
    fa, fb = a.load_cable_dynamics(), b.load_cable_dynamics()
    x, u = state(n), np.zeros(a.nu)
    assert np.allclose(np.asarray(fa(x, u, b.geom_values())).ravel(),
                       np.asarray(fb(x, u, b.geom_values())).ravel(), atol=1e-12)


def test_geometry_actually_changes_the_dynamics():
    """The complement: if the two produced the same answer for any parameters, the
    test above would pass trivially on a model that ignored them."""
    n = 3
    d = make(n)
    f = d.load_cable_dynamics()
    x, u = state(n), np.zeros(d.nu)
    near = np.asarray(f(x, u, d.geom_values())).ravel()
    far = np.asarray(f(x, u, make(n, rho=ring(n, 0.30), cable=1.2).geom_values())).ravel()
    assert not np.allclose(near, far, atol=1e-6)


def test_an_off_centre_weld_is_expressible():
    """The attach case: a newcomer welds where its magnet landed, not on the ring.
    That must be a parameter value, not a rebuild."""
    n = 3
    d = make(n)
    weld = [r[:] for r in ring(n)]
    weld[2] = [-0.08, 0.0, 0.05]                 # gap-centre weld, off the ring
    g = d.geom_values(rho=weld)
    assert np.allclose(g[6:9], [-0.08, 0.0, 0.05])
    f = d.load_cable_dynamics()
    x, u = state(n), np.zeros(d.nu)
    assert np.all(np.isfinite(np.asarray(f(x, u, g)).ravel()))


def test_geom_values_defaults_to_the_nominal_geometry():
    """A solver never told otherwise must reproduce the old baked-constant model."""
    n = 4
    d = make(n, rho=ring(n, 0.11), cable=0.62)
    g = d.geom_values()
    assert g.shape == (4 * n,)
    assert np.allclose(g[:3 * n].reshape(n, 3), np.array(ring(n, 0.11)))
    assert np.allclose(g[3 * n:], 0.62)


def test_geom_values_rejects_a_wrong_sized_geometry():
    """A silent length mismatch would shift every drone's rho by one slot."""
    d = make(3)
    with pytest.raises(AssertionError):
        d.geom_values(rho=ring(2))


@pytest.mark.parametrize('n', [2, 3, 4])
def test_dimensions_depend_only_on_fleet_size(n):
    """Why one compiled solver per n is enough: nx/nu are set by n alone, so any
    geometry reuses the same .so."""
    a = make(n, rho=ring(n, 0.08), cable=0.5)
    b = make(n, rho=ring(n, 0.25, z=0.1), cable=1.1)
    assert (a.nx, a.nu, a.p_geom.shape[0]) == (b.nx, b.nu, b.p_geom.shape[0])

"""resize_fleet keeps the placement yaw datum: the rebuilt ReferenceBuilder starts at
yaw 0, so without the carry-over a ring placed rotated is pulled toward world yaw 0 at
every OCP detach/attach (found 2026-09-28 while writing the control-math page)."""
import types

import numpy as np

from mpc_planner.planner_node import LoadPlanner
from mpc_planner.geometry import attach_points


class _Log:
    def warn(self, *_):
        pass

    info = error = warn


class _Solver:
    N, dt = 20, 0.1

    def set_geometry(self, rho, cable_lengths):
        pass


def _planner(n_old, n_new, psi0, latched):
    rho_new = attach_points(n_new, 0.25, 0.025)
    dyn = types.SimpleNamespace(rho=rho_new, m=0.86)
    p = types.SimpleNamespace(
        _solvers={n_new: (dyn, _Solver(), rho_new)}, n=n_old, cable_len=0.5,
        cable_len_i=[0.5] * n_old, cable_elev_deg=45.0, dt=0.1, traj=None,
        psi0=psi0, _yaw_datum_latched=latched, get_logger=lambda: _Log())
    return p, rho_new


def test_resize_keeps_the_latched_yaw_datum():
    psi0 = np.radians(37.0)
    p, rho = _planner(4, 3, psi0, latched=True)
    assert LoadPlanner.resize_fleet(p, 3, [0, 1, 2], rho=rho)
    assert abs(p.refs.psi0 - psi0) < 1e-12


def test_resize_before_the_latch_leaves_the_builder_at_zero():
    p, rho = _planner(4, 3, 0.0, latched=False)
    assert LoadPlanner.resize_fleet(p, 3, [0, 1, 2], rho=rho)
    assert p.refs.psi0 == 0.0


def test_resize_keeps_the_learned_ring_force_and_hands_it_to_the_new_ocp():
    """int_mode model on a resizing fleet (Wesley 8 Oct): b survives the resize and the new
    solver and reference builder get its force at once."""
    from mpc_planner.lumped_integral import LumpedForceIntegral
    p, rho = _planner(4, 3, 0.0, latched=False)
    got = []
    p._solvers[3][1].set_disturbance = lambda d: got.append(np.array(d, float))
    p._lint = LumpedForceIntegral(0.4, 0.15, 10.0, 5.26, 7.13)
    p._lint.b = np.array([0.01, -0.02, 0.05])
    p._lint_applied = p._lint.force()
    p._dist, p._of, p._dist_applied, p._of_applied = None, None, [np.zeros(3)], np.zeros(3)
    p._land_to_ground = False
    p._apply_load_force = lambda: LoadPlanner._apply_load_force(p)
    want = p._lint.force()
    assert LoadPlanner.resize_fleet(p, 3, [0, 1, 2], rho=rho)
    assert np.allclose(p._lint.b, [0.01, -0.02, 0.05])
    assert got and np.allclose(got[-1], want) and np.allclose(p.refs.d_L, want)


def test_the_ring_integral_is_frozen_through_a_reconfiguration_hold():
    from mpc_planner.lumped_integral import LumpedForceIntegral
    ref = np.r_[0.0, 0.0, 1.0, np.zeros(10)]
    p = types.SimpleNamespace(
        load_state=np.r_[0.0, 0.0, 0.97, 1.0, np.zeros(9)], refs=types.SimpleNamespace(yref_at=lambda k: ref),
        get_clock=lambda: types.SimpleNamespace(now=lambda: types.SimpleNamespace(nanoseconds=0)),
        lift_z0=0.1, lift_progress=0.9, target_z=1.0, _lint_lift_t=None, _lint_settle_s=0.0,
        _zbias_gated=lambda vz_gate=True: True, traj_t=0.0, traj=types.SimpleNamespace(kind='hover'),
        _z_i_gate=0.25, _lint=LumpedForceIntegral(0.4, 0.15, 10.0, 5.26, 7.13),
        _lint_warned=np.zeros(3, dtype=bool), _lint_said=1e18, get_logger=lambda: _Log(),
        _reconfig_hold_left=1.5)
    LoadPlanner._update_lumped(p)
    assert np.allclose(p._lint.b, 0.0)
    p._reconfig_hold_left = 0.0
    LoadPlanner._update_lumped(p)
    assert p._lint.b[2] > 0.0

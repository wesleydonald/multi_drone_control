"""Massive rods in the planner model (option F, 2026-10-03): at a level hover with the rod-aware
nominal tensions the load is in equilibrium, the fleet's thrust equals the whole weight, and each
tracker's model (thrust through the map mass plus the published external acceleration) predicts
exactly the planned drone motion."""
import casadi as cs
import numpy as np

from mpc_planner.geometry import attach_points, balanced_tensions, nominal_cable_dirs
from mpc_planner.load_cable_dynamics import CABLE_DIM, LOAD_DIM, LoadCableDynamics

N, M_L, M_MAP, M_R, L, COM = 3, 0.86, 0.55, 0.075, 0.55, 0.147
J = [0.0234, 0.0234, 0.0469]


def hover(rod_mass):
    rho = attach_points(N, 0.225, 0.02)
    dyn = LoadCableDynamics(N, M_L, J, [L] * N, rho, M_MAP, rod_mass=rod_mass, rod_lam=COM / L)
    s = nominal_cable_dirs(rho, 50.0)
    t = balanced_tensions(rho, s, M_L, rod_mass=rod_mass, rod_lam=COM / L)
    x = np.zeros(dyn.nx)
    x[6] = 1.0                                       # level load, at rest
    for i in range(N):
        b = LOAD_DIM + CABLE_DIM * i
        x[b:b + 3] = s[i]
        x[b + 12] = t[i]
    return dyn, x


def test_load_is_in_equilibrium_with_rod_aware_tensions():
    dyn, x = hover(M_R)
    xdot = np.asarray(dyn.load_cable_dynamics()(x, np.zeros(dyn.nu), dyn.geom_values())).ravel()
    assert np.allclose(xdot[3:6], 0.0, atol=1e-9) and np.allclose(xdot[10:13], 0.0, atol=1e-9)


def test_fleet_thrust_carries_the_whole_weight():
    dyn, x = hover(M_R)
    thrust = cs.Function('f', [dyn.x, dyn.p_geom], [cs.vertcat(*[dyn.thrust_vec(i) for i in range(N)])])
    f = np.asarray(thrust(x, dyn.geom_values())).reshape(N, 3)
    assert abs(f[:, 2].sum() - (N * M_MAP + M_L) * 9.81) < 1e-9


def test_tracker_model_predicts_the_planned_motion():
    dyn, x = hover(M_R)
    for i in range(N):
        fn = cs.Function('k', [dyn.x, dyn.p_geom],
                         [dyn.quad_accel(i), dyn.thrust_vec(i) / dyn.mi[i],
                          dyn.cable_accel(i) + dyn.rod_accel(i)])
        a, acc_pub, c = (np.asarray(v).ravel() for v in fn(x, dyn.geom_values()))
        # the tracker: v_dot = thrust accel through the map mass + g + external accel
        assert np.allclose(acc_pub + np.array([0.0, 0.0, -9.81]) + c, a, atol=1e-9)


def test_rod_shifts_tension_onto_the_rods_and_weight_off_the_drones():
    _, x0 = hover(0.0)
    _, x1 = hover(M_R)
    t0, t1 = x0[LOAD_DIM + 12], x1[LOAD_DIM + 12]
    assert t1 > t0 + 0.3                             # the rods also hold their load-end share

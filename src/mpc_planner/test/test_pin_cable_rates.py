"""pin_cable_rates (card 2026-10-04_pin_cable_rates): node 0's rod rates computed from the measured
drone and ring velocities reproduce each drone's measured velocity through the model's own
kinematics (its component across the rod; a rigid rod cannot change length), and node 0 pins them."""
import casadi as cs
import numpy as np

from mpc_planner.geometry import attach_points, nominal_cable_dirs
from mpc_planner.load_cable_dynamics import CABLE_DIM, LOAD_DIM, LoadCableDynamics, observed_state_indices
from mpc_planner.planner_solver import PlannerSolver

N, L = 3, 0.55


def solver_stub():
    rho = attach_points(N, 0.225, 0.02)
    dyn = LoadCableDynamics(N, 0.86, [0.0234, 0.0234, 0.0469], [L] * N, rho, 0.55)
    ps = PlannerSolver.__new__(PlannerSolver)
    ps.dyn, ps.pin_rates, ps.last_X, ps._q_prev = dyn, True, None, None
    return ps, dyn, rho


def test_measured_rates_reproduce_the_drone_velocities():
    ps, dyn, rho = solver_stub()
    q = np.array([np.cos(0.05), 0.0, np.sin(0.05), 0.0])            # a tilted ring
    ls = np.concatenate([[0.1, -0.2, 1.0], q, [0.05, -0.02, 0.03], [0.1, -0.2, 0.3]])
    s = nominal_cable_dirs(rho, 48.0)
    from mpc_planner.geometry import quat_to_rot_np
    R = quat_to_rot_np(q)
    pivots = [ls[0:3] + R @ rho[i] - L * s[i] for i in range(N)]
    rng = np.random.default_rng(1)
    vel = [rng.normal(0.0, 0.2, 3) for _ in range(N)]
    x = ps.build_x_init(ls, pivots, vel)
    for i in range(N):
        fn = cs.Function('v', [dyn.x, dyn.p_geom], [dyn.quad_velocity(i)])
        v_model = np.asarray(fn(x, dyn.geom_values())).ravel()
        b = LOAD_DIM + CABLE_DIM * i
        si = x[b:b + 3]
        across = lambda v: v - si * float(si @ v)
        v_att = ls[7:10] + R @ np.cross(ls[10:13], rho[i])
        # the model's drone velocity equals the measured one across the rod; along the rod it
        # follows the attach point (rigid rod)
        assert np.allclose(across(v_model), across(vel[i]), atol=1e-9)
        assert abs(float(si @ (v_model - v_att))) < 1e-9


def test_node_zero_pins_the_rates_only_when_asked():
    plain, pinned = observed_state_indices(N), observed_state_indices(N, rates=True)
    assert len(pinned) == len(plain) + 3 * N
    for i in range(N):
        b = LOAD_DIM + CABLE_DIM * i
        assert {b + 3, b + 4, b + 5} <= set(pinned) and not {b + 3, b + 4, b + 5} & set(plain)

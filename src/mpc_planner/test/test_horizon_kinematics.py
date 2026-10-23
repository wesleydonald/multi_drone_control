"""Each drone's references from one mapped call over the horizon (planner option plan_fast_refs, card
2026-10-23_scalability) are what the node-by-node extraction gives, bit for bit: the mapped CasADi functions with and
without the rod model at 2, 3 and 8 quads, the vectorised pivot correction at the rig's 4 cm (tilts to 30 deg, any
heading, |a| ~ 0, thrust along the heading), and the references the node publishes with the switch on and off."""
import types

import numpy as np
import pytest

from mpc_planner.geometry import attach_points, centre_from_pivot, centre_from_pivot_many
from mpc_planner.load_cable_dynamics import LoadCableDynamics, LOAD_DIM, CABLE_DIM
from mpc_planner.params import load_inertia
from mpc_planner.planner_node import LoadPlanner
from mpc_planner.planner_solver import PlannerSolver, horizon_map, kinematics_funcs, rod_accel_at

PIVOT = np.array([0.0, 0.0, -0.04])
N = 20


def _solver(n, rod=False, pivot=PIVOT, mapped=True):
    """A PlannerSolver with the real extraction functions and no acados solver."""
    rho = attach_points(n, 0.5 if n > 4 else 0.225, 0.02)
    dyn = LoadCableDynamics(n, 0.9, load_inertia(0.9), [1.0] * n, rho, 0.55,
                            rod_mass=0.075 if rod else 0.0, rod_lam=0.147 if rod else 0.0)
    s = PlannerSolver.__new__(PlannerSolver)
    s.dyn, s.N, s._geom, s.pivot_offset = dyn, N, dyn.geom_values(), np.asarray(pivot, float)
    s.pos_fun, s.vel_fun, s.acc_fun, s.cable_fun, s.rod_fun = kinematics_funcs(dyn)
    if mapped:
        s.kin_map = [horizon_map([s.pos_fun[i], s.vel_fun[i], s.acc_fun[i], s.cable_fun[i]]
                                 + ([s.rod_fun[i]] if rod else []), dyn.nx, len(s._geom), N + 1)
                     for i in range(n)]
    return s


def _horizon(dyn, rng):
    """A horizon in the planner's state layout: a ring 1 m up, unit quaternions and cable directions, tensions."""
    X = rng.normal(0.0, 0.3, (dyn.nx, N + 1))
    X[2] += 1.0
    q = rng.normal(0.0, 1.0, (4, N + 1)) + np.array([[4.0], [0.0], [0.0], [0.0]])
    X[6:10] = q / np.linalg.norm(q, axis=0)
    for i in range(dyn.n):
        b = LOAD_DIM + CABLE_DIM * i
        s = rng.normal(0.0, 0.4, (3, N + 1)) + np.array([[0.0], [0.0], [-1.0]])
        X[b:b + 3] = s / np.linalg.norm(s, axis=0)
        X[b + 12] = rng.uniform(1.0, 6.0, N + 1)
    return X


def _per_node(s, X, i, yaw):
    return [(*s.drone_kinematics(X[:, k], i, yaw), rod_accel_at(s, X[:, k], i)) for k in range(X.shape[1])]


def _same(a, b):
    return len(a) == len(b) and all(len(u) == len(v) and all(np.array_equal(p, q) for p, q in zip(u, v))
                                    for u, v in zip(a, b))


@pytest.mark.parametrize('n', [2, 3, 8])
@pytest.mark.parametrize('rod', [False, True])
@pytest.mark.parametrize('pivot', [np.zeros(3), PIVOT])
def test_the_mapped_extraction_is_the_node_by_node_one_bit_for_bit(n, rod, pivot):
    s = _solver(n, rod, pivot)
    rng = np.random.default_rng(n + 10 * rod)
    for _ in range(3):
        X = _horizon(s.dyn, rng)
        for i in range(n):
            yaw = float(rng.uniform(-np.pi, np.pi))
            assert _same(s.horizon_kinematics(X, i, yaw), _per_node(s, X, i, yaw)), (n, rod, i)


def test_the_vectorised_pivot_correction_is_the_scalar_one_bit_for_bit():
    rng = np.random.default_rng(7)
    for _ in range(300):
        M = 21
        tilt, az = np.radians(rng.uniform(0.0, 30.0, M)), rng.uniform(-np.pi, np.pi, M)
        g = rng.uniform(3.0, 16.0, M)
        A = np.vstack([g * np.sin(tilt) * np.cos(az), g * np.sin(tilt) * np.sin(az), g * np.cos(tilt)])
        P = rng.normal(0.0, 1.0, (3, M))
        yaw = float(rng.uniform(-np.pi, np.pi))
        ref = np.column_stack([centre_from_pivot(P[:, k].copy(), A[:, k].copy(), yaw, PIVOT) for k in range(M)])
        assert np.array_equal(centre_from_pivot_many(P, A, yaw, PIVOT), ref)


@pytest.mark.parametrize('yaw', [0.0, 0.3, -2.9])
def test_a_vanishing_thrust_and_thrust_along_the_heading_take_the_scalar_fallback(yaw):
    A = np.array([[0.0, 1e-12, 9.0 * np.cos(yaw), 9.0 * np.cos(yaw + 1.0), 0.2, 1e-10],
                  [0.0, 0.0, 9.0 * np.sin(yaw), 9.0 * np.sin(yaw + 1.0), -0.1, 0.0],
                  [0.0, 0.0, 0.0, 0.0, 9.8, -1e-10]])
    P = np.arange(18.0).reshape(3, 6) / 7.0
    ref = np.column_stack([centre_from_pivot(P[:, k].copy(), A[:, k].copy(), yaw, PIVOT) for k in range(6)])
    assert np.array_equal(centre_from_pivot_many(P, A, yaw, PIVOT), ref)
    assert np.array_equal(ref[:, 0], P[:, 0] + [0.0, 0.0, 0.04])          # |a| = 0: level, as before


@pytest.mark.parametrize('yaw', [0.0, 0.7])
def test_a_nan_thrust_takes_the_scalar_level_branch(yaw):
    A = np.array([[np.nan, 0.2], [0.0, -0.1], [9.8, 9.8]])
    P = np.arange(6.0).reshape(3, 2) / 7.0
    ref = np.column_stack([centre_from_pivot(P[:, k].copy(), A[:, k].copy(), yaw, PIVOT) for k in range(2)])
    out = centre_from_pivot_many(P, A, yaw, PIVOT)
    assert np.all(np.isfinite(out[:, 0])) and np.array_equal(out, ref)


def test_a_zero_offset_returns_the_pivots():
    P = np.arange(6.0).reshape(3, 2)
    assert centre_from_pivot_many(P, np.ones((3, 2)), 0.4, np.zeros(3)) is P


def test_a_solver_without_the_map_extracts_node_by_node():
    s = _solver(3, rod=True, mapped=False)
    X = _horizon(s.dyn, np.random.default_rng(3))
    assert _same(s.horizon_kinematics(X, 1, 0.5), _per_node(s, X, 1, 0.5))


class _Pub:
    def __init__(self):
        self.data = []

    def publish(self, msg):
        self.data.append(list(msg.data))


def _node(fast, n=4, rod=True, equal_pull=False):
    s = _solver(n, rod)
    p = LoadPlanner.__new__(LoadPlanner)
    p.n, p.N, p.dt, p.solver, p.dyn, p._fast_refs = n, N, 0.1, s, s.dyn, fast
    p.slot2drone = [2, 0, 3, 1][:n] if n == 4 else list(range(n))
    p.ref_pub = [_Pub() for _ in range(n)]
    p.pretension_s, p._ff_t, p._ff_cap = (3.0 if equal_pull else 0.0), 0.6, 1.0
    p._lift_refused, p.lift_ramp_vel, p._land_to_ground = False, 0.15, False
    p._load_down = lambda: False
    p._set_ff_active = lambda on: None
    p._airborne_start = lambda: False
    p.phase, p._settle_left = 'planner', 0.0
    p._cable_taut_gate = lambda i: (0.7 + 0.1 * i, 1.0)
    p._slot_yaw = lambda i: 0.4 * i - 0.9
    p._dist_applied = (np.zeros(3), np.array([0.3, -0.2, 0.5]))
    p._last_air_ref = {}
    p._publish_static_cable = lambda X: None
    p._diag_ctr = 1                                    # not the 1 Hz debug line
    return p


@pytest.mark.parametrize('rod', [False, True])
@pytest.mark.parametrize('equal_pull', [False, True])
def test_the_node_publishes_the_same_references_with_the_switch_on(rod, equal_pull):
    X = _horizon(_solver(4, rod).dyn, np.random.default_rng(11))
    off, on = _node(False, rod=rod, equal_pull=equal_pull), _node(True, rod=rod, equal_pull=equal_pull)
    off._publish_refs(X)
    on._publish_refs(X)
    assert [q.data for q in on.ref_pub] == [q.data for q in off.ref_pub]
    assert all(len(q.data) == 1 and len(q.data[0]) == 2 + 12 * (N + 1) for q in on.ref_pub)
    for d, (a, c) in off._last_air_ref.items():
        assert np.array_equal(on._last_air_ref[d][0], a) and np.array_equal(on._last_air_ref[d][1], c)


def test_the_switch_is_off_unless_typed():
    p = LoadPlanner.__new__(LoadPlanner)
    s = types.SimpleNamespace()
    p._solve_budget_s, p.pivot_offset, p._sqp_iters = 0.0, np.zeros(3), 0
    p._w_pos_scale = p._w_s_scale = p._w_r_scale = 1.0
    p._configure_solver(s)
    assert s.defer_res is False and s.res_every == 1
    assert PlannerSolver.defer_res is False

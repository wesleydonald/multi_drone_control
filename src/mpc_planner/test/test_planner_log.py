"""The planner log.csv (W1, plan 2026-10): the original columns keep their positions, the
solve / per-rod / ring-attitude / reference-age columns are appended, and a row is as
wide as the header even after a resize changes the fleet."""
import types

import numpy as np

from mpc_planner.planner_node import LoadPlanner, tick_log_header, ZBias
from mpc_planner.planner_solver import HorizonFallback
from mpc_planner.load_cable_dynamics import LOAD_DIM, CABLE_DIM

OLD = ['sim_time', 'phase', 'n', 'load_x', 'load_y', 'load_z', 'load_vz', 'tilt_deg',
       'z_tgt', 'z_bias', 'lift_progress', 'traj_t', 'land', 'd0_z', 'd1_z', 'd2_z']


def test_original_columns_keep_their_positions():
    assert tick_log_header(3, 3)[:len(OLD)] == OLD


def test_new_columns_are_appended():
    h = tick_log_header(3, 3)
    for c in ('solve_status', 'solve_ms', 'qp_iter', 't0', 't2', 'tsz1', 'gate2', 'len0',
              'elev1', 'ff', 'ff_active', 'qw', 'qx', 'qy', 'qz', 'wx', 'wy', 'wz',
              'ref_age', 'd2_x', 'd0_qz', 'published', 'res_stat', 'res_eq'):
        assert c in h and h.index(c) >= len(OLD), c
    assert len(set(h)) == len(h)


class _Clock:
    def __init__(self, t):
        self.t = t

    def now(self):
        return types.SimpleNamespace(nanoseconds=int(self.t * 1e9))


def _stub(n=3):
    p = LoadPlanner.__new__(LoadPlanner)
    rho = [np.array([0.25 * np.cos(a), 0.25 * np.sin(a), 0.0])
           for a in np.linspace(0, 2 * np.pi, n, endpoint=False)]
    p.n, p.rho, p.slot2drone = n, rho, list(range(n))
    p.load_state = np.array([0, 0, 0.5, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0], float)
    p.drone_pos = [np.array([0.6 * r[0] / 0.25, 0.6 * r[1] / 0.25, 0.85]) for r in rho]
    p.drone_quat = {k: np.array([1.0, 0, 0, 0]) for k in range(n)}
    p.pivot_offset = np.zeros(3)
    p.cable_len_i = [0.5] * n
    p.target_z, p.lift_z0, p.lift_progress, p.traj_t = 0.6, 0.1, 0.3, 0.0
    p.phase, p._land_to_ground, p._ff_active, p._ff_last = 'planner', False, True, 0.8
    p._zbias = ZBias(0.0, 0.15, 10.0)
    p._fallback = HorizonFallback(3, 0.1)
    p._published = 'shifted'
    p._fallback.success(None, 1.0)
    p.solver = types.SimpleNamespace(last_status=4, last_solve_ms=12.5, last_qp_iter=7,
                                     last_iters=2, last_res=np.array([0.3, 0.02, 0.0, 0.0]))
    X = np.zeros((LOAD_DIM + CABLE_DIM * n, 21))
    for i in range(n):
        X[LOAD_DIM + CABLE_DIM * i + 12, 0] = 4.0
        X[LOAD_DIM + CABLE_DIM * i + 2, 0] = -0.7
    p._pub_X = X
    p.get_clock = lambda: _Clock(1.4)
    p._log_nd, p._log_ns = n, n
    return p


def test_row_matches_header_and_carries_the_solve():
    p = _stub()
    h = tick_log_header(3, 3)
    row = p._tick_log_row()
    assert len(row) == len(h)
    r = dict(zip(h, row))
    assert r['solve_status'] == 4 and r['qp_iter'] == 7
    assert float(r['t1']) == 4.0 and abs(float(r['tsz1']) + 2.8) < 1e-9
    assert abs(float(r['ref_age']) - 0.4) < 1e-9
    assert r['ff_active'] == 1 and float(r['qw']) == 1.0
    assert r['published'] == 'shifted' and float(r['res_eq']) == 0.02


def test_row_width_is_latched_across_a_resize():
    p = _stub()
    h = tick_log_header(3, 3)
    p.n, p.slot2drone, p.rho = 2, [0, 2], [p.rho[0], p.rho[2]]
    assert len(p._tick_log_row()) == len(h)

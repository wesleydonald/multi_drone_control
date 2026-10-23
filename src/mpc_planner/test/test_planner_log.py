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
    X[6, :] = 1.0                                    # level ring, yaw 0
    p._pub_X = X
    p.refs = types.SimpleNamespace(yaw_ref_at=lambda k: 0.0)
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


OCP_COLS = ('solve_status', 'solve_ms', 'qp_iter', 'sqp_calls', 't0', 'tsz2', 'ff', 'ref_age', 'fail_streak',
            'published', 'res_stat', 'res_eq', 'int_bx', 'int_fz', 'plan5_x', 'plan5_z', 'res_ms',
            'yaw_ref0', 'plan_yaw1', 'plan_yaw5', 'az1_0', 'az1_2')


def test_the_tick_columns_follow_res_ms():
    h = tick_log_header(3, 3)
    i = h.index('tick_ms')
    assert h[i:i + 4] == ['tick_ms', 'hover_x', 'hover_y', 'net_trim_z'] and h[i - 1] == 'res_ms'


def test_an_ocp_row_carries_the_tick_time_and_the_datum():
    import time
    p = _stub()
    p.hover_xy, p._tick_t0 = (0.12, -0.05), time.perf_counter()
    r = dict(zip(tick_log_header(3, 3), p._tick_log_row()))
    assert 0.0 <= float(r['tick_ms']) < 1000.0
    assert (r['hover_x'], r['hover_y'], r['net_trim_z']) == ('0.1200', '-0.0500', '')
    assert r['solve_ms'] == '12.5' and r['ff'] == '0.800'


def test_a_network_row_leaves_the_ocp_columns_blank():
    """Card 2026-10-23_paper_method_compare: the planner logs on through the network phase; its rows carry the
    network's height target, LAND, trim and tick time, and no stale solve."""
    import time
    p = _stub()
    p.phase, p._net_hold_z, p._net_landing = 'network', 0.95, True
    p.net = types.SimpleNamespace(_a_trim=np.array([0.0, 0.0, 0.31]))
    p.hover_xy, p._tick_t0 = (0.01, 0.02), time.perf_counter()
    p.solver.last_res_ms = 3.2       # the last OCP solve's, which a network row must not repeat
    h = tick_log_header(3, 3)
    row = p._tick_log_row()
    assert len(row) == len(h)
    r = dict(zip(h, row))
    assert all(r[c] == '' for c in OCP_COLS), {c: r[c] for c in OCP_COLS if r[c] != ''}
    assert r['phase'] == 'network' and float(r['z_tgt']) == 0.95 and r['land'] == 1
    assert r['net_trim_z'] == '+0.3100' and r['tick_ms'] != ''
    assert float(r['len0']) > 0 and r['gate0'] != '' and float(r['qw']) == 1.0      # measured, not planned


def test_the_yaw_lag_columns_follow_the_tick_columns():
    h = tick_log_header(3, 3)
    i = h.index('net_trim_z') + 1
    assert h[i:] == ['yaw_ref0', 'plan_yaw1', 'plan_yaw5', 'az1_0', 'az1_1', 'az1_2', 'az0_0', 'az0_1', 'az0_2']


def _yaw_quat(a):
    return [np.cos(a / 2), 0.0, 0.0, np.sin(a / 2)]


def test_a_spin_row_carries_the_reference_the_plan_and_the_cable_azimuths():
    """Spin yaw lag (rig 0.61 s vs twin 0.25 s): the reference at node 0, the plan's ring yaw at nodes 1 and 5 on
    the reference's branch (a turn past 180 deg does not wrap), the plan's cable azimuths at node 1, the measured."""
    p = _stub()
    p.refs = types.SimpleNamespace(yaw_ref_at=lambda k: 3.30 + 0.05 * k)
    X = p._pub_X
    X[6:10, 1] = _yaw_quat(3.20)
    X[6:10, 5] = _yaw_quat(3.45)
    for i in range(3):
        b = LOAD_DIM + CABLE_DIM * i
        a = 0.4 + i
        X[b:b + 3, 1] = [0.7 * np.cos(a), 0.7 * np.sin(a), -0.7]
    r = dict(zip(tick_log_header(3, 3), p._tick_log_row()))
    assert float(r['yaw_ref0']) == 3.3
    assert abs(float(r['plan_yaw1']) - 3.20) < 1e-4 and abs(float(r['plan_yaw5']) - 3.45) < 1e-4
    assert [round(float(r[f'az1_{i}']), 4) for i in range(3)] == [0.4, 1.4, round(2.4, 4)]
    for i in range(3):                               # the measured cable points drone -> ring: inward
        out = np.arctan2(p.rho[i][1], p.rho[i][0])
        assert abs((float(r[f'az0_{i}']) - out) % (2 * np.pi) - np.pi) < 1e-3


def test_a_tick_without_a_plan_leaves_the_plan_yaw_blank_and_measures_the_cables():
    p = _stub()
    p._published = ''                                # a creep or hold tick: the last horizon was not this tick's
    r = dict(zip(tick_log_header(3, 3), p._tick_log_row()))
    assert all(r[c] == '' for c in ('yaw_ref0', 'plan_yaw1', 'plan_yaw5', 'az1_0', 'az1_1', 'az1_2'))
    assert all(r[f'az0_{i}'] != '' for i in range(3))

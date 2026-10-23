"""A warm solve's logged residual after the references went out (planner option plan_fast_refs, card
2026-10-23_scalability) and how often a warm tick logs one (plan_res_every), on a fake acados solver that counts
get_residuals: finish_tick computes the deferred one with the same values; the gate's ticks (a reseed, a budget cut,
the ticks after an unconverged one) and a failed solve still compute it inside solve_horizon; statuses, the
unconverged flag and the iterate to continue from are those of the solver without the switch; the node finishes the
tick before it writes the log row, which carries what the residual took."""
import types

import numpy as np
import pytest

from mpc_planner.planner_node import LoadPlanner, tick_log_header
from mpc_planner.planner_solver import PlannerSolver, RES_EQ_MAX, RES_EVERY, STATUS_UNCONVERGED


class _FakeAcados:
    def __init__(self, statuses, res_eq):
        self.statuses = list(statuses)
        self.res_eq = list(res_eq)
        self.solves = 0
        self.res_calls = []                  # solves done when each residual was computed

    def set(self, k, field, v):
        pass

    def reset(self):
        pass

    def solve(self):
        self.solves += 1
        return self.statuses.pop(0) if self.statuses else 0

    def get(self, k, field):
        return np.full(4, float(k) + self.solves)

    def get_stats(self, field):
        return np.array([1])

    def dump_last_qp_to_json(self, path, overwrite=False):
        pass

    def get_residuals(self, recompute=False):
        self.res_calls.append(self.solves)
        eq = self.res_eq.pop(0) if len(self.res_eq) > 1 else self.res_eq[0]
        return np.array([0.1, eq, 0.0, 0.0])


def _solver(fake, defer=False, every=RES_EVERY, budget=0.0):
    s = PlannerSolver.__new__(PlannerSolver)
    s.solver, s.N = fake, 5
    s.dyn = types.SimpleNamespace(nu=2, n=2, rho=[np.zeros(3)] * 2, l=[0.5, 0.5])
    s._geom = np.zeros(3)
    s._obs_idx = np.array([0, 1])
    s.last_X, s.recover = None, False
    s.solve_budget_s, s.pivot_offset, s.fail_dump_dir = budget, np.zeros(3), None
    s.steady_iters = 2
    s.defer_res, s.res_every = defer, every
    s._reset_stats()
    return s


def _tick(s, reseed):
    X, st = s.solve_horizon(lambda k: np.zeros(6), lambda k: np.array([1.0, 0, 0, 0]), np.arange(4.0), reseed)
    seen = (None if s.last_res is None else s.last_res.copy(), len(s.solver.res_calls))
    s.finish_tick()
    return X, st, seen


def _flight(defer, statuses, res_eq, reseeds, every=RES_EVERY):
    s = _solver(_FakeAcados(statuses, res_eq), defer, every)
    out = []
    for reseed in reseeds:
        X, st, seen = _tick(s, reseed)
        out.append(dict(X=X, st=st, last=s.last_status, unconv=s._unconverged, pending=s.pending_X,
                        res=None if s.last_res is None else s.last_res.copy(), inside=seen))
    return s, out


# tick 0 reseeds (15 solves), tick 8 fails on its second solve, tick 15 reseeds into three unconverged residuals
SCRIPT = dict(statuses=[0] * 30 + [4] + [0] * 80,
              res_eq=[1e-6] * 15 + [10 * RES_EQ_MAX] * 3 + [1e-6] * 80,
              reseeds=[True] + [False] * 14 + [True] + [False] * 20)


def test_the_deferred_flight_matches_the_flight_without_the_switch():
    _, off = _flight(False, **SCRIPT)
    _, on = _flight(True, **SCRIPT)
    assert len(on) == len(off)
    for a, b in zip(off, on):
        assert a['st'] == b['st'] and a['last'] == b['last'] and a['unconv'] == b['unconv']
        assert (a['X'] is None) == (b['X'] is None) and (a['X'] is None or np.array_equal(a['X'], b['X']))
        assert (a['pending'] is None) == (b['pending'] is None)
        assert np.array_equal(a['res'], b['res'])               # the same values, logged after the publish
    assert STATUS_UNCONVERGED in [b['st'] for b in on] and 4 in [b['st'] for b in on]


def test_a_warm_tick_defers_and_the_gates_ticks_do_not():
    s = _solver(_FakeAcados([0] * 20, [1e-6]), defer=True)
    X, st, (res, calls) = _tick(s, True)                       # reseed: the gate reads it inside
    assert st == 0 and res is not None and calls == 1
    X, st, (res, calls) = _tick(s, False)                      # warm: nothing inside, finish_tick computes it
    assert res is None and calls == 1 and s.last_res is not None and len(s.solver.res_calls) == 2
    assert s.solver.res_calls[-1] == s.solver.solves           # from the same iterate: no solve in between


def test_a_failed_solve_and_a_budget_cut_compute_it_inside():
    s = _solver(_FakeAcados([0] * 15 + [0, 4], [1e-6]), defer=True)
    _tick(s, True)                                             # the reseed's 15 iterations
    X, st, (res, calls) = _tick(s, False)
    assert st == 4 and X is None and res is not None and calls == 2
    fake = _FakeAcados([0] * 20, [1e-6])
    s = _solver(fake, defer=True, budget=1e-9)                 # every warm solve is cut after 1 iteration
    s.steady_iters = 3
    _tick(s, True)
    X, st, (res, calls) = _tick(s, False)
    assert res is not None and calls == 2


def test_after_an_unconverged_tick_the_next_is_checked_inside():
    s = _solver(_FakeAcados([0] * 20, [10 * RES_EQ_MAX] * 2 + [1e-6]), defer=True)
    X, st, _ = _tick(s, True)
    assert st == STATUS_UNCONVERGED and s.pending_X is not None
    X, st, (res, calls) = _tick(s, False)
    assert st == STATUS_UNCONVERGED and res is not None                # still unconverged: held back
    X, st, (res, calls) = _tick(s, False)
    assert st == 0 and X is not None and res is not None              # converged: published, checked inside
    X, st, (res, calls) = _tick(s, False)
    assert st == 0 and res is None                                     # a plain warm tick again


@pytest.mark.parametrize('defer', [False, True])
def test_res_every_0_logs_none_and_10_every_tenth(defer):
    s, out = _flight(defer, [0] * 60, [1e-6], [True] + [False] * 30, every=0)
    assert out[0]['res'] is not None and all(o['res'] is None for o in out[1:])
    s, out = _flight(defer, [0] * 60, [1e-6], [True] + [False] * 30, every=10)
    logged = [k for k, o in enumerate(out) if o['res'] is not None]
    assert logged == [0, 10, 20, 30]


def test_finish_tick_does_nothing_without_a_deferred_residual():
    s = _solver(_FakeAcados([0] * 10, [1e-6]), defer=True)
    s.finish_tick()
    assert s.solver.res_calls == [] and s.last_res is None
    _tick(s, False)
    n = len(s.solver.res_calls)
    s.finish_tick()                                            # a second call (a network tick) computes nothing
    assert len(s.solver.res_calls) == n


def test_the_residual_time_is_kept():
    s = _solver(_FakeAcados([0] * 10, [1e-6]), defer=True)
    _tick(s, False)
    assert np.isfinite(s.last_res_ms) and s.last_res_ms >= 0.0


def test_the_node_finishes_the_tick_before_it_logs():
    order = []
    p = LoadPlanner.__new__(LoadPlanner)
    p.load_state, p.drone_pos = np.zeros(13), [np.zeros(3)]
    p._plan_tick = lambda: order.append('solve')
    p.solver = types.SimpleNamespace(finish_tick=lambda: order.append('finish'))
    p._log_tick = lambda: order.append('log')
    p._plan()
    assert order == ['solve', 'finish', 'log']
    p.solver = types.SimpleNamespace()                         # a stub without it
    order.clear()
    p._plan()
    assert order == ['solve', 'log']


def test_the_log_row_carries_what_the_residual_took():
    h = tick_log_header(3, 3)        # appended after plan5_*; later packages append after it
    assert h[h.index('plan5_z') + 1] == 'res_ms' and h.count('res_ms') == 1

"""Solve-failure handling (W8, plan 2026-10; card 2026-10-01 critic must-fixes 1-3), on a
fake acados solver.

A reseed zeroes the whole iterate (solver.reset) before x is set on every node and u = 0.
A failed SQP step ends the solve. The wall budget (off by default, the rig sets it) cuts
warm and cold solves alike, a cold one never below 5 iterations, and a reseeded or cut
solve above the dynamics-residual bound is not published but continued from. A failed
solve leaves a replay dump. On failure the node publishes the last good horizon advanced
by the time since it was solved, for at most max_shift node periods, then nothing, so the
trackers' reference watchdog still trips."""
import time
import types

import numpy as np
import pytest

from mpc_planner.planner_solver import (PlannerSolver, HorizonFallback,
                                        shift_horizon, STATUS_UNCONVERGED,
                                        RES_EQ_MAX, SOLVE_BUDGET_S)


class _FakeAcados:
    def __init__(self, statuses, sleep=0.0, res_eq=None):
        self.statuses = list(statuses)
        self.sleep = sleep
        self.res_eq = list(res_eq) if res_eq is not None else None
        self.calls = []
        self.solves = 0
        self.dumped = []

    def set(self, k, field, v):
        self.calls.append((k, field, np.array(v, float)))

    def reset(self):
        self.calls.append((None, 'reset', None))

    def solve(self):
        time.sleep(self.sleep)
        self.solves += 1
        return self.statuses.pop(0) if self.statuses else 0

    def get(self, k, field):
        return np.full(4, float(k))

    def get_stats(self, field):
        return np.array([2, 1])

    def dump_last_qp_to_json(self, path, overwrite=False):
        self.dumped.append(path)

    def get_residuals(self, recompute=False):
        if self.res_eq is None:
            raise AttributeError('no residuals')
        eq = self.res_eq.pop(0) if len(self.res_eq) > 1 else self.res_eq[0]
        return np.array([0.1, eq, 0.0, 0.0])


def _solver(fake, N=5):
    s = PlannerSolver.__new__(PlannerSolver)
    s.solver, s.N = fake, N
    s.dyn = types.SimpleNamespace(nu=2, n=2, rho=[np.zeros(3)] * 2, l=[0.5, 0.5])
    s._geom = np.zeros(3)
    s._obs_idx = np.array([0, 1])
    s.last_X = None
    s.recover = False
    s.solve_budget_s = 0.06
    s.pivot_offset = np.zeros(3)
    s.fail_dump_dir = None
    s._reset_stats()
    return s


def _solve(s, reseed, **kw):
    return s.solve_horizon(lambda k: np.zeros(6), lambda k: np.array([1.0, 0, 0, 0]),
                           np.arange(4.0), reseed, **kw)


def test_reseed_resets_the_whole_iterate_then_seeds_every_node():
    fake = _FakeAcados([0] * 20)
    s = _solver(fake)
    X, st = _solve(s, reseed=True)
    assert st == 0 and X.shape == (4, 6)
    fields = [(k, f) for k, f, _ in fake.calls]
    i_reset = fields.index((None, 'reset'))
    xs = [k for k, f in fields[i_reset:] if f == 'x']
    us = [(k, v) for k, f, v in fake.calls[i_reset:] if f == 'u']
    assert xs == list(range(6))
    assert [k for k, _ in us] == list(range(5)) and all(not v.any() for _, v in us)


def test_warm_solve_does_not_reset():
    fake = _FakeAcados([0] * 20)
    _solve(_solver(fake), reseed=False)
    assert all(f != 'reset' for _, f, _ in fake.calls) and fake.solves == 5


def test_the_budget_never_cuts_a_cold_solve_below_5_iterations():
    fake = _FakeAcados([0] * 20, sleep=0.025)
    s = _solver(fake)
    _solve(s, reseed=True)
    assert fake.solves == 5 and s.last_iters == 5                  # 60 ms passed at 3
    assert s.last_qp_iter == 3 * fake.solves


def test_warm_solves_are_budgeted_too():
    fake = _FakeAcados([0] * 20, sleep=0.025, res_eq=[1e-5])
    s = _solver(fake)
    X, st = _solve(s, reseed=False)
    assert 2 <= fake.solves <= 4 and st == 0 and X is not None     # 3 nominally: 75 ms > 60


def test_the_budget_is_off_by_default():
    assert SOLVE_BUDGET_S == 0.0
    fake = _FakeAcados([0] * 20, sleep=0.005)
    s = _solver(fake)
    s.solve_budget_s = SOLVE_BUDGET_S
    _solve(s, reseed=True)
    assert fake.solves == 15                                       # load-independent in sim


def test_a_failed_step_ends_the_solve():
    fake = _FakeAcados([0, 4, 0, 0, 0])
    s = _solver(fake)
    X, st = _solve(s, reseed=False)
    assert X is None and st == 4 and fake.solves == 2 and s.last_iters == 2


def test_an_unconverged_reseed_is_held_back_then_continued(tmp_path, monkeypatch):
    monkeypatch.setenv('MDC_RUN_DIR', str(tmp_path))
    fake = _FakeAcados([0] * 40, res_eq=[10 * RES_EQ_MAX])
    s = _solver(fake)
    X, st = _solve(s, reseed=True)
    assert X is None and st == STATUS_UNCONVERGED and s.pending_X is not None
    assert not list(tmp_path.glob('planner_fail_*'))               # not a solver failure
    # the next (warm) solve is still checked, and is published once it converges
    X, st = _solve(s, reseed=False)
    assert st == STATUS_UNCONVERGED
    fake.res_eq = [RES_EQ_MAX / 10]
    X, st = _solve(s, reseed=False)
    assert st == 0 and X is not None and s.pending_X is None
    fake.res_eq = [10 * RES_EQ_MAX]                                # a plain warm solve is not checked
    X, st = _solve(s, reseed=False)
    assert st == 0


def test_status_0_without_residuals_is_trusted():
    fake = _FakeAcados([0] * 20)
    X, st = _solve(_solver(fake), reseed=True)
    assert st == 0 and X is not None


def test_a_failure_leaves_a_replay_dump(tmp_path, monkeypatch):
    monkeypatch.setenv('MDC_RUN_DIR', str(tmp_path))
    fake = _FakeAcados([4] * 5)
    s = _solver(fake)
    X, st = _solve(s, reseed=False, context={'load_state': np.ones(13), 'slot2drone': [1, 0]})
    assert X is None and st == 4 and s.last_status == 4
    dumps = sorted(tmp_path.glob('planner_fail_*.npz'))
    assert len(dumps) == 1
    d = np.load(dumps[0])
    assert (d['x_init'] == np.arange(4.0)).all() and d['yref'].shape == (6, 6)
    assert (d['ctx_slot2drone'] == [1, 0]).all() and int(d['status']) == 4
    assert fake.dumped and fake.dumped[0].endswith('_qp.json')


def test_shift_horizon_advances_and_repeats_the_last_node():
    X = np.arange(12.0).reshape(2, 6)
    Y = shift_horizon(X, 2)
    assert Y.shape == X.shape
    assert (Y[:, :4] == X[:, 2:]).all() and (Y[:, 4:] == X[:, -1:]).all()


def test_failure_then_success():
    fb = HorizonFallback(3, 0.1)
    X = np.arange(12.0).reshape(2, 6)
    fb.success(X, 10.0)
    Y = fb.failure(10.1)
    assert (Y == shift_horizon(X, 1)).all()
    X2 = X + 100
    fb.success(X2, 10.2)
    assert fb.streak == 0 and fb.age(10.25) == pytest.approx(0.05)
    assert (fb.failure(10.3) == shift_horizon(X2, 1)).all()


def test_shifted_publishes_are_bounded_in_time():
    fb = HorizonFallback(3, 0.1)
    X = np.arange(12.0).reshape(2, 6)
    fb.success(X, 0.0)
    out = [fb.failure(0.1 * k + 0.004) for k in range(1, 6)]      # a few ms of timer jitter
    assert [o is not None for o in out] == [True, True, True, False, False]
    assert (out[2] == shift_horizon(X, 3)).all()


def test_the_shift_follows_elapsed_time_not_the_failure_count():
    fb = HorizonFallback(3, 0.1)
    X = np.arange(12.0).reshape(2, 6)
    fb.success(X, 0.0)
    assert (fb.failure(0.21) == shift_horizon(X, 2)).all()        # the first failing tick overran
    assert fb.failure(0.52) is None                               # one slow failure: past the bound


@pytest.mark.parametrize('bad', [-1, 5, 20, 1000])
def test_max_shift_is_bounded_by_the_watchdog(bad):
    with pytest.raises(ValueError):
        HorizonFallback(bad, 0.1)
    HorizonFallback(4, 0.1)
    fb = HorizonFallback(0, 0.1)
    fb.success(np.zeros((2, 6)), 0.0)
    assert fb.failure(0.1) is None                                # 0 = publish nothing


def test_the_stamp_of_the_last_good_solve_is_kept():
    fb = HorizonFallback(3, 0.1)
    fb.success(np.zeros((2, 6)), 5.0)
    for k in range(1, 4):
        fb.failure(5.0 + 0.1 * k)
        assert fb.t_good == 5.0 and fb.age(5.0 + 0.1 * k) == pytest.approx(0.1 * k)


def test_no_good_horizon_publishes_nothing():
    fb = HorizonFallback(3, 0.1)
    assert fb.failure(1.0) is None and np.isnan(fb.age(1.0))


def _failing_node(max_shift, clock):
    from mpc_planner.planner_node import LoadPlanner
    from mpc_planner.planner_solver import PLAN_N, PLAN_TF
    p = LoadPlanner.__new__(LoadPlanner)
    p.n, p.load_state = 2, np.zeros(13)
    p._pivot_at = lambda i: np.zeros(3)
    p._solve_context = lambda: {}
    p.get_clock = lambda: types.SimpleNamespace(now=lambda: types.SimpleNamespace(
        nanoseconds=int(round(clock[0] * 1e9))))
    p.get_logger = lambda: types.SimpleNamespace(warn=lambda *a, **k: None)
    p.refs = types.SimpleNamespace(yref_at=None, q_ref_at=None)
    p._fallback = HorizonFallback(max_shift, PLAN_TF / PLAN_N)
    p.published = []
    p._publish_refs = lambda X: p.published.append((clock[0], X))
    outcomes = iter([np.zeros((4, PLAN_N + 1))])                   # one good solve, then failures
    sv = types.SimpleNamespace(last_X=None, recover=False, pending_X=None, last_solve_ms=90.0)
    sv.build_x_init = lambda ls, pos: np.zeros(4)
    sv.solve_horizon = lambda *a, **k: (next(outcomes, None), 0 if sv.last_X is None else 4)
    p.solver = sv
    return p


def test_a_failing_planner_goes_quiet_and_the_tracker_watchdog_trips():
    """Node level: after the last good solve at t = 0 every solve fails; the planner bridges
    for max_shift node periods, then publishes nothing, and the tracker's arrival watchdog
    (1.0 s, 3-sample debounce at 50 Hz) latches by 1.4 s."""
    safety = pytest.importorskip('utility_objects.safety')
    clock = [0.0]
    p = _failing_node(3, clock)
    for k in range(20):                                            # 10 Hz for 2 s
        clock[0] = 0.1 * k
        p._solve_and_publish()
    times = [t for t, _ in p.published]
    assert times == pytest.approx([0.0, 0.1, 0.2, 0.3]) and p._published == 'none'
    env = safety.EnvelopeChecker(safety.EnvelopeLimits(ref_timeout_s=1.0))
    tripped = None
    for j in range(100):                                           # the tracker at 50 Hz
        t = 0.02 * j
        last = max(tp for tp in times if tp <= t + 1e-9)
        if env.check(ref_age_s=t - last, airborne=True).is_fault and tripped is None:
            tripped = t
    assert tripped is not None and tripped <= 1.4


def test_the_bridge_is_bounded_even_when_every_failing_tick_overruns():
    clock = [0.0]
    p = _failing_node(3, clock)
    p._solve_and_publish()
    for t in (0.35, 0.7, 1.05):                                    # 250 ms over each tick
        clock[0] = t
        p._solve_and_publish()
    assert [t for t, _ in p.published] == [0.0]

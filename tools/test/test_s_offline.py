"""Card 2026-10-23_scalability's offline tools after its critic: the B calibration (planner_replay's logged-vs-replayed
solve times, matched per tick on the warm planner ticks, and the same-iteration subset), solve_scaling's N_max on the
rig-scaled whole tick, and plan_quality's cost ratio paired with the residual tick by tick (Q1 with Q2)."""
import os
import sys

import numpy as np
import pandas as pd
import pytest

TOOLS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, TOOLS)
import plan_quality as PQ  # noqa: E402
import planner_replay as PR  # noqa: E402
import solve_scaling as SS  # noqa: E402


def _rows_recs():
    t = np.round(np.arange(0.0, 1.0, 0.1), 3)
    rows = pd.DataFrame({'sim_time': t, 'solve_ms': [40.0] * 8 + [80.0, 60.0],
                         'sqp_calls': [5] * 8 + [3, 5]})
    recs = [dict(t=float(v), planner=True, reseed=False, status=0, ms=20.0, iters=5) for v in t]
    recs[0]['reseed'] = True                              # a cold reseed: not a warm tick
    recs[1]['planner'] = False                            # before the hand-over
    return rows, recs


def test_logged_against_replayed_on_the_warm_planner_ticks():
    rows, recs = _rows_recs()
    tm = PR.timing_vs_logged(rows, recs)
    a, s = tm['all'], tm['same_iters']
    assert a['ticks'] == 8 and a['replay_p50'] == 20.0 and a['ratio_p50'] == pytest.approx(2.0)
    assert a['logged_p95'] == pytest.approx(np.percentile([40.0] * 6 + [80.0, 60.0], 95))
    assert s['ticks'] == 7 and s['logged_p95'] == pytest.approx(np.percentile([40.0] * 6 + [60.0], 95))   # the cut out


def test_an_arm_at_other_iterations_or_an_old_log_gives_none():
    rows, recs = _rows_recs()
    for r in recs:
        r['iters'] = 1
    assert PR.timing_vs_logged(rows, recs) is None
    assert PR.timing_vs_logged(rows.drop(columns=['solve_ms']), _rows_recs()[1]) is None


def test_n_max_on_the_rig_scaled_whole_tick():
    def row(n, lat, tick, fails=0):
        return {'n': n, 'iters': 1, 'fails': fails, 'latency_ms': {'p95': lat}, 'tick_ms': {'p95': tick}}
    rows = [row(3, 7.0, 9.0), row(4, 14.0, 16.0), row(5, 28.0, 30.0), row(6, 45.0, 48.0), row(7, 61.0, 90.0, fails=1)]
    m = SS.n_max_of(rows, 1, 60.0, 100.0, 3.5)
    assert m == {'latency': 6, 'tick': 6, 'rig_tick': 4}               # 3.5 x 30 > 100; n 7 failed a solve
    assert SS.n_max_of(rows, 1, 60.0, 100.0, 1.0)['rig_tick'] == 6
    assert SS.n_max_of(rows, 2, 60.0, 100.0, 3.5) == {'latency': None, 'tick': None, 'rig_tick': None}


def test_a_tick_that_breaks_the_dynamics_misses_the_cost_bar_whatever_its_ratio():
    def rec(ratio, res):
        return {'q_conv_status': 0, 'q_cost_ratio': ratio, 'q_res_eq': res}
    recs = [rec(1.01, 1e-6)] * 18 + [rec(0.90, 5e-2), rec(1.02, None)]      # below J_conv on a broken iterate
    q = PQ.summarize(recs)
    assert q['cost_ratio']['p95'] < 1.05                                     # unpaired: looks fine
    cp = q['cost_ratio_paired']
    assert cp['res_over'] == 2 and cp['p50'] == pytest.approx(1.01) and np.isinf(cp['p95'])

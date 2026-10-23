"""tools/s_metrics.py on a synthetic 8 -> 7 -> 8 run (card 2026-10-23_scalability): the tilt peak and settled tilt of
each event, the planner's resize latency, a stage's position error, tilt peak, tensions and carriers' throttle, the
card's N1 / N4 windows (the lift left out) and the lift window itself, the published ticks over the residual bar, the
rejoin lines, a run with more than 1 % late planner ticks is void, and the node logs are read on run.csv's clock and
cut to its rows."""
import json
import os
import sys

import numpy as np
import pandas as pd
import pytest

TOOLS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, TOOLS)
import s_metrics as S  # noqa: E402


def _quat_tilt_x(deg):
    a = np.radians(deg) / 2.0
    return np.stack([np.cos(a), np.sin(a), 0.0 * a, 0.0 * a], 1)


def _run(tmp_path, late=0, chain=True, t_ready=0.0, tail=False):
    """A ring that tilts 3 deg about x for 1 s after the release at 30 s and 2 deg after the attach at 50 s; it lifts
    from 5 to 12 s, 8 cm behind its reference and tilted 5 deg (all of it before N1's window). chain False: s8_hover.
    The node logs stamp /clock, t_ready (the manifest's t_ready_sim) ahead of run.csv; tail: they also log a stalled
    startup before run.csv's first row and a stalled shutdown after its last, with slow solves."""
    logs = tmp_path / 'R9999_sim_gz_s8' / 'logs'
    (logs / 'dissipative_planner' / '20261030_200000').mkdir(parents=True)
    (logs / 'tracker' / 'drone0_20261030_200000').mkdir(parents=True)
    t = np.arange(0.0, 80.0, 0.02)
    lift = (t >= 5.0) & (t < 12.0)
    tilt = 0.2 + 5.0 * lift + chain * (3.0 * ((t >= 30.0) & (t < 31.0)) + 2.0 * ((t >= 50.0) & (t < 51.0)))
    q = _quat_tilt_x(tilt)
    z_ref = np.clip(0.02 + 0.14 * (t - 5.0), 0.02, 1.0)
    freed = (t >= 30.0) & (t < 50.0) & chain
    thr = {f'd{i}_thr': np.full(t.size, 0.6) for i in range(8)}
    thr['d1_thr'] = np.where(freed, 0.0, 0.6)                    # the released quad, off the ring
    thr['d2_thr'] = np.where(freed, 0.8, 0.6)                    # a carrier at the cap while it is gone
    pd.DataFrame({'t': t, 'payload_x': 0.01, 'payload_y': 0.0, 'payload_z': z_ref - 0.08 * lift,
                  'payload_qw': q[:, 0], 'payload_qx': q[:, 1], 'payload_qy': q[:, 2], 'payload_qz': q[:, 3],
                  'payload_ref_x': 0.0, 'payload_ref_y': 0.0, 'payload_ref_z': z_ref, **thr}).to_csv(
        logs / 'run.csv', index=False)
    ev = ([3.0, 30.0, 50.0, 70.0], ['ARM', 'RELEASE', 'ATTACH', 'LAND'], [None, 1, 1, None]) if chain else \
        ([3.0, 70.0], ['ARM', 'LAND'], [None, None])
    pd.DataFrame(dict(zip(('sim_time', 'event', 'arg'), ev))).to_csv(logs / 'events.csv', index=False)
    pt = np.arange(0.0, 80.0, 0.1)
    if late:
        pt = np.concatenate([pt[:100], pt[100:] + 0.2])           # one late tick
    n = np.where((pt >= 30.3) & (pt < 50.5) & chain, 7, 8)
    res = np.full(pt.size, 1e-6)
    pub = np.full(pt.size, 'solved', dtype=object)
    res[200], res[201], res[202], pub[201] = 0.05, 0.5, np.nan, 'shifted'   # one published over, one not, one unlogged
    s2d = np.where(n == 8, '0 1 2 3 4 5 6 7', '0 2 3 4 5 6 7')
    plan = {'sim_time': pt, 'phase': 'planner', 'n': n, 'solve_status': 0, 'solve_ms': 40.0, 'res_eq': res,
            'published': pub, 'slot2drone': s2d}
    for i in range(8):
        plan[f't{i}'] = 1.5
    plan = pd.DataFrame(plan)
    tt = np.arange(0.0, 80.0, 0.02)
    if tail:
        head, shut = np.array([-5.0, -4.9, -2.0, -1.9]), 80.0 + 0.5 * np.arange(20)
        plan = pd.concat([plan.iloc[[0] * head.size].assign(sim_time=head, solve_ms=300.0), plan,
                          plan.iloc[[-1] * shut.size].assign(sim_time=shut, solve_ms=300.0)])
        tt = np.concatenate([tt, 80.0 + 0.1 * np.arange(50)])
    plan.assign(sim_time=plan['sim_time'] + t_ready).to_csv(
        logs / 'dissipative_planner' / '20261030_200000' / 'log.csv', index=False)
    pd.DataFrame({'sim_time': tt + t_ready}).to_csv(
        logs / 'tracker' / 'drone0_20261030_200000' / 'log.csv', index=False)
    (logs.parent / 'manifest.json').write_text(json.dumps({'t_ready_sim': t_ready}))
    (logs / 'launch.log').write_text(
        '[INFO] [1.0] [dissipative_node]: [dissipative] REJOIN START drone 2 (/drone_1): from [1.3 0. 1.] back\n'
        '[WARN] [2.0] [dissipative_node]: [dissipative] REJOIN ABORT drone 2 (/drone_1) (no weld in 45 s): '
        'up to 1.40 m\n'
        '[INFO] [3.0] [dissipative_node]: [dissipative] REJOIN START drone 2 (/drone_1): from [1.3 0. 1.] back\n'
        '[WARN] [4.0] [dissipative_node]: [dissipative] REJOIN WELD drone 2 (/drone_1): its tether joint attached\n')
    return str(logs.parent)


def test_events_stages_and_planner_stats(tmp_path):
    r = S.score(_run(tmp_path))
    rel, att = r['events']
    assert rel['event'] == 'RELEASE' and rel['drone'] == 1 and att['event'] == 'ATTACH'
    assert rel['peak_deg'] == pytest.approx(3.0, abs=0.01) and att['peak_deg'] == pytest.approx(2.0, abs=0.01)
    assert rel['settled_deg'] == pytest.approx(0.2, abs=0.01)
    assert rel['resize_s'] == pytest.approx(0.3, abs=1e-6) and rel['n_after'] == 7
    assert att['resize_s'] == pytest.approx(0.5, abs=1e-6) and att['n_after'] == 8
    assert [s['n'] for s in r['stages']] == [8, 7, 8]
    assert r['stages'][1]['pos_rmse_m'] == pytest.approx(0.01) and r['stages'][1]['tensions'] == [1.5] * 7
    assert r['stages'][1]['tilt_peak_deg'] == pytest.approx(0.2, abs=0.01)          # 5 s after the release
    n1, n4 = r['n1'], r['n4']
    assert n1['from'] == pytest.approx(14.0, abs=0.03) and n1['to'] == 30.0 and n1['n'] == 8
    assert n1['pos_rmse_m'] == pytest.approx(0.01) and n1['tilt_peak_deg'] == pytest.approx(0.2, abs=0.01)
    assert n4['from'] == pytest.approx(52.5, abs=1e-6) and n4['to'] == 70.0 and n4['n'] == 8   # resize 50.5 + 2 s
    assert n4['pos_rmse_m'] == pytest.approx(0.01) and n4['tilt_peak_deg'] == pytest.approx(0.2, abs=0.01)
    p = r['planner']
    assert p['failed'] == 0 and p['solve_ms_p95'] == 40.0 and p['late'] == 0.0 and not r['void']


def test_n4_opens_after_the_last_rejoin_blend(tmp_path):
    """The card's N4 revision (23 Oct): from 2 s after the last REJOIN_BLEND_OVER; the resize's window kept beside."""
    run_dir = _run(tmp_path)
    p = os.path.join(run_dir, 'logs', 'events.csv')
    ev = pd.read_csv(p)
    pd.concat([ev, pd.DataFrame({'sim_time': [60.0], 'event': ['REJOIN_BLEND_OVER'], 'arg': [1]})]).sort_values(
        'sim_time').to_csv(p, index=False)
    r = S.score(run_dir)
    assert r['n4']['from'] == pytest.approx(62.0, abs=1e-6) and r['n4']['to'] == 70.0 and r['n4']['n'] == 8
    assert r['n4_resize']['from'] == pytest.approx(52.5, abs=1e-6)
    assert [s['n'] for s in r['stages']] == [8, 7, 8]                # the blend is not a stage


def test_a_hover_with_no_events_is_one_stage_from_the_top(tmp_path):
    r = S.score(_run(tmp_path, chain=False))
    assert r['events'] == [] and len(r['stages']) == 1 and r['n4'] is None
    n1 = r['n1']
    assert n1['from'] == pytest.approx(14.0, abs=0.03) and n1['to'] == 70.0 and n1['n'] == 8
    assert n1['pos_rmse_m'] == pytest.approx(0.01) and n1['tilt_peak_deg'] == pytest.approx(0.2, abs=0.01)


def test_the_node_logs_on_the_run_clock_cut_to_run_csv(tmp_path):
    """R2708's planner log stamps /clock 12 s ahead of run.csv (its resize read 12.3 s), and R2702/03/10/11 read void on
    the stalled ticks before run.csv's first row and after its last."""
    run_dir = _run(tmp_path, t_ready=11.986, tail=True)
    logs = os.path.join(run_dir, 'logs')
    plan = pd.read_csv(os.path.join(logs, 'dissipative_planner', '20261030_200000', 'log.csv'))
    trk = pd.read_csv(os.path.join(logs, 'tracker', 'drone0_20261030_200000', 'log.csv'))
    assert S.late_fraction(plan['sim_time'], 0.1) > S.LATE_VOID and S.late_fraction(trk['sim_time']) > S.LATE_VOID
    r = S.score(run_dir)
    rel, att = r['events']
    assert rel['resize_s'] == pytest.approx(0.3, abs=1e-6) and rel['n_after'] == 7
    assert att['resize_s'] == pytest.approx(0.5, abs=1e-6) and att['n_after'] == 8
    assert [s['n'] for s in r['stages']] == [8, 7, 8] and r['n4']['from'] == pytest.approx(52.5, abs=1e-6)
    p = r['planner']
    assert p['ticks'] == 800 and p['published'] == 799 and p['solve_ms_max'] == 40.0
    assert p['late'] == 0.0 and r['tracker_late_max'] == 0.0 and not r['void']


def test_a_starved_planner_voids_the_run(tmp_path):
    r = S.score(_run(tmp_path, late=1))
    assert r['planner']['late'] > 0.0 and r['void'] is (r['planner']['late'] > S.LATE_VOID)
    assert S.late_fraction([0.0, 0.1, 0.2, 0.45, 0.55]) == pytest.approx(0.25)


def test_the_lift_window_the_residual_bar_throttle_and_rejoins(tmp_path):
    r = S.score(_run(tmp_path))
    li = r['lift']                                    # the reference rises from 5 s, tops at 12 s
    assert li['from'] == pytest.approx(5.08, abs=0.03) and li['to'] == pytest.approx(14.0, abs=0.03)
    assert li['tilt_peak_deg'] == pytest.approx(5.2, abs=0.01)
    assert li['pos_err_max_m'] == pytest.approx(np.hypot(0.01, 0.08), abs=1e-6)
    p = r['planner']
    assert p['published_res_over'] == 1 and p['published_res_missing'] == 1
    assert p['published'] == 799 and p['published_res_eq_max'] == pytest.approx(0.05)
    s8, s7 = r['stages'][0], r['stages'][1]
    assert s8['thr_p95'] == pytest.approx(0.6) and s8['thr_cap_frac'] == 0.0
    assert s7['thr_p95'] == pytest.approx(0.8) and s7['thr_cap_frac'] == pytest.approx(1.0 / 7.0)   # drone 1 not in
    assert [(j['kind'], j['drone'], j['why']) for j in r['rejoins']] == [
        ('START', 1, None), ('ABORT', 1, 'no weld in 45 s'), ('START', 1, None), ('WELD', 1, None)]


def test_a_run_without_the_new_columns_still_scores(tmp_path):
    run_dir = _run(tmp_path, chain=False)
    logs = os.path.join(run_dir, 'logs')
    pdir = os.path.join(logs, 'dissipative_planner', '20261030_200000', 'log.csv')
    pd.read_csv(pdir).drop(columns=['published', 'slot2drone']).to_csv(pdir, index=False)
    os.remove(os.path.join(logs, 'launch.log'))
    r = S.score(run_dir)
    assert r['planner']['published'] == 0 and r['planner']['published_res_over'] == 0 and r['rejoins'] == []
    assert r['stages'][0]['thr_p95'] == pytest.approx(0.6)            # carriers: slots 0..n-1

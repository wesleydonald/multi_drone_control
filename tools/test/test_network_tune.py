"""tools/network_tune.py, the offline harness of result C (card 2026-10-23_paper_method_compare): the knobs reach the
network, the objective and guards are the card's, a short flight of the port runs and is scored by C's functions."""
import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import network_tune as NT  # noqa: E402


def test_the_knobs_and_the_structure_reach_the_network():
    k, p = NT.params_of({'diss_c': 9.0, 'diss_k_pay': 80.0}, 'paper')
    assert (p.c, p.k_pay, p.k_anchor, p.k_slot, p.elev_deg, p.ki_load) == (9.0, 80.0, 80.0, 18.0, 60.0, 1.0)
    assert p.damp_relative and k['net_traj_lean'] and k['vel_kp_pos'] == 2.0
    _, p = NT.params_of(None, 'port')
    assert not p.damp_relative and p.c == NT.DEFAULTS['diss_c']


def _ok(**kw):
    s = dict(tilt_peak_deg=3.0, quad_tilt_peak_deg=12.0, thr_peak=0.62, yaw_peak_deg=1.0, yaw_growth_deg_s=-0.01)
    s.update(kw)
    return s


def test_the_objective_is_the_headline_on_line_x_04():
    res = {'hover': _ok(err_rms_mm=8.0), 'linex_v400': _ok(headline_mm=21.5), 'linex_v200': _ok(headline_mm=9.0)}
    assert NT.objective(res) == 21.5
    assert NT.objective({'linex_v400': {'diverged': 3.0}}) == math.inf


@pytest.mark.parametrize('bad, why', [(dict(tilt_peak_deg=15.5), 'ring tilt'),
                                      (dict(quad_tilt_peak_deg=36.0), 'quad tilt'),
                                      (dict(thr_peak=0.79), 'throttle'), (dict(yaw_peak_deg=11.0), 'ring yaw 11'),
                                      (dict(yaw_growth_deg_s=0.02), 'ring yaw growing')])
def test_each_guard_fails_a_candidate(bad, why):
    assert NT.guards({'linex_v400': _ok()}) == []
    out = NT.guards({'linex_v400': _ok(**bad)})
    assert len(out) == 1 and why in out[0]


def test_a_divergence_and_a_worse_hover_fail_the_guards():
    assert 'diverged' in NT.guards({'margin_linex_v400': {'diverged': 7.4}})[0]
    res = {'hover': _ok(err_rms_mm=12.5)}
    assert NT.guards(res, hover_ref_mm=10.0) and not NT.guards(res, hover_ref_mm=11.0)


def test_the_yaw_guard_reads_growth_not_only_excursion():
    """M4: a mode growing too slowly to reach 10 deg in the window still fails; a decaying or steady swing passes."""
    t = np.arange(0.0, 60.0, 0.02)
    swing = np.sin(2 * math.pi * t / 7.07)
    assert NT.yaw_growth(t, 2.0 * swing) == pytest.approx(0.0, abs=2e-3)
    assert NT.yaw_growth(t, 2.0 * np.exp(-t / 20.0) * swing) < 0.0
    growing = 0.5 * np.exp(t / 25.0) * swing                       # 0.5 deg to 5.5 deg: under the 10 deg cap
    assert np.max(np.abs(growing)) < NT.GUARD_YAW and NT.yaw_growth(t, growing) > NT.GUARD_YAW_GROWTH


def test_the_line_cases_fly_the_claims_cycles():
    assert NT.CYCLES == {0.2: 4, 0.4: 8, 0.6: 8}


def test_stage_2_starts_from_the_fewest_guard_failures_when_no_grid_point_passes(monkeypatch):
    """The review's gap: no stage-1 point passes (today's yaw mode); stage 2 still runs, from the grid point with the
    fewest guard failures, and keeps a candidate that ranks better (here: one that passes)."""
    def fake(args):
        knobs, structure, vel_filter, cases = args
        k = {**NT.DEFAULTS, **knobs}
        ok = _ok(err_rms_mm=8.0, headline_mm=30.0 - k['vel_kv'])
        bad = 0 if (k['diss_c'] == 12.0 and k['vel_kv'] == 5.0) else 1 if k['diss_c'] == 12.0 else 2
        res = {'hover': ok, 'linex_v400': dict(ok), 'margin_linex_v400': dict(ok)}
        if bad >= 1:
            res['margin_linex_v400'] = {'diverged': 17.0}
        if bad >= 2:
            res['linex_v400'] = {'diverged': 12.0}
        return knobs, structure, res

    class Pool:
        def __init__(self, *a, **k):
            pass

        def __enter__(self):
            return self

        def __exit__(self, *a):
            return False

        def imap(self, f, jobs):
            return map(f, jobs)
    import multiprocessing
    monkeypatch.setattr(multiprocessing, 'Pool', Pool)
    monkeypatch.setattr(NT, 'evaluate', fake)
    lines = []
    best, j, cands = NT.search('port', 'ema', 1, passes=1, log=lines.append)
    assert 'no grid point passes' in lines[0] and "'diss_c': 12.0" in lines[0]
    assert best['diss_c'] == 12.0 and best['vel_kv'] == 5.0 and j == pytest.approx(25.0)


def test_a_short_hover_of_the_port_flies_and_is_scored():
    """2 s from the hand-over at today's gains: the ring stays at its height, the 0.5 deg yaw kick is seen."""
    r = NT.fly('hover', 0.0, hover_s=2.0)
    assert 'diverged' not in r
    s = NT.score(r)
    assert s['err_rms_mm'] < 20.0 and s['tilt_peak_deg'] < 5.0 and 0.0 < s['thr_peak'] <= NT.THR_MAX
    assert NT.YAW_KICK_DEG * 0.5 < abs(r['yaw'][0]) < NT.YAW_KICK_DEG * 1.5
    assert abs(r['act'][-1, 2] - NT.Z_TARGET) < 0.02


def test_the_line_is_scored_by_cs_functions():
    """A ring 0.2 s late on the schedule the harness flies: no clock-free error or reach, the lag is the delay."""
    d, v = NT.LINE_M, 0.4
    T = math.pi * d / v
    t = np.arange(0.0, NT.HOLD_S + 2 * T + NT.TAIL_S, NT.CTRL_DT)
    w = 2.0 * v / d
    x = lambda tt: 0.5 * d * (1.0 - np.cos(w * np.clip(tt, 0.0, 2 * T)))  # noqa: E731
    des = np.c_[x(t - NT.HOLD_S), np.zeros_like(t), np.ones_like(t)]
    act = np.c_[x(t - NT.HOLD_S - 0.2), np.zeros_like(t), np.ones_like(t)]
    r = dict(t=t, p_des=des, act=act, tilt=np.ones_like(t), quad_tilt=np.ones_like(t), thr=np.full_like(t, 0.6),
             yaw=np.zeros_like(t), case='line_x', speed=v, T_path=2 * T, hover_s=NT.HOVER_S)
    s = NT.score(r)
    assert s['cf_rms_mm'] < 0.5 and abs(s['reach_mean_mm']) < 0.5
    assert s['lag_s'] == pytest.approx(0.2, abs=0.02) and s['headline_mm'] < 1.0


def test_emit_writes_a_candidates_twin_configs_from_the_claims(tmp_path):
    import shutil
    import yaml
    from experiment.config import ExperimentConfig
    for sub, f in (('twin', 'c_compare.yaml'), ('twin', 'c_compare_paper.yaml'),
                   ('experiments', 'c_net_hover_L100.yaml'), ('experiments', 'c_netp_linex_v400_L100.yaml')):
        (tmp_path / 'configs' / sub).mkdir(parents=True, exist_ok=True)
        shutil.copy(os.path.join(NT.REPO, 'configs', sub, f), tmp_path / 'configs' / sub / f)
    out = NT.emit('p1', 'paper', {'diss_c': 12.0, 'vel_kp_pos': 1.0}, root=str(tmp_path))
    p = yaml.safe_load(open(out[0]))
    assert (p['diss_c'], p['vel_kp_pos'], p['diss_k_anchor'], p['net_damp_relative'], p['traj_hold_s']) == \
        (12.0, 1.0, 40.0, True, 5.0)
    for f, case in zip(out[1:], ('hover', 'linex_v400')):
        c = ExperimentConfig.from_yaml(f)
        assert c.name == f'c_net_tune_p1_{case}_L100' and c.launch_args['mode'] == 'network'
        assert c.launch_args['params_file'].endswith('configs/twin/c_net_p1.yaml')
        assert 'Exploratory, once.' in open(f).read() and 'Claim config' not in open(f).read()


def test_the_bench_returns_finite_times():
    b = NT.bench(n=5)
    assert set(b) == {'net_step_ms', 'net_step_preview21_ms', 'velocity_loop_ms'}
    assert all(np.isfinite(v).all() and v[0] > 0 for v in b.values())
    assert b['net_step_preview21_ms'][0] > b['net_step_ms'][0]

"""
Tethered-hold metrics (tools/metrics.py, plan 2026-10 W6), the criteria that use them
(tools/experiment/config.py, run_experiment.tethered_checks) and the WAIT_LIFT timeout.

Synthetic signals with known answers, then the model-f1 rig fixture
(tools/test/fixtures/rig_0930/model_f1: its planner log as flown, the four tracker logs cut
to the columns thrust_fit.py reads, and the 25 'solve status' lines of model_f1.log).
"""
import json
import math
import os
import sys

import numpy as np
import pytest

TOOLS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, TOOLS)

import metrics as M  # noqa: E402
import thrust_fit as tf  # noqa: E402
from experiment.config import Criteria, Event, wait_lift_arg  # noqa: E402

F1 = os.path.join(os.path.dirname(__file__), 'fixtures', 'rig_0930', 'model_f1')


# ── heave ────────────────────────────────────────────────────────────────────

def test_heave_of_a_sine_reads_its_amplitude_and_period():
    t = np.arange(0.0, 20.0, 0.1)
    z = 0.5 + 0.015 * np.sin(2 * np.pi * t / 4.0)
    h = M.heave_metrics(t, z)
    assert h['pp_m'] == pytest.approx(0.03, abs=5e-4)
    assert h['sd_m'] == pytest.approx(0.015 / math.sqrt(2), rel=0.02)
    assert h['period_s'] == pytest.approx(4.0, rel=0.03)
    assert h['n_periods'] == pytest.approx(5.0, rel=0.05)
    assert h['short'] is False


def test_heave_window_under_three_periods_is_flagged():
    """model-f1's '14 cm at 0.17 Hz' came off 3.8 s of data: a 6 s period cannot be claimed."""
    t = np.arange(0.0, 12.0, 0.1)
    z = 0.5 + 0.05 * np.sin(2 * np.pi * t / 6.0)
    assert M.heave_metrics(t, z)['short'] is True


def test_heave_period_ignores_a_linear_drift():
    t = np.arange(0.0, 30.0, 0.1)
    z = 0.4 + 0.004 * t + 0.01 * np.sin(2 * np.pi * t / 3.0)
    assert M.heave_metrics(t, z)['period_s'] == pytest.approx(3.0, rel=0.05)


# ── hold window, z error, vz, ref age ────────────────────────────────────────

def _planner(t_end=40.0, target=0.5, land_at=35.0, z_off=0.0):
    t = np.arange(0.0, t_end, 0.1)
    z_tgt = np.minimum(target, 0.05 + 0.1 * np.clip(t - 5.0, 0, None))
    z_tgt[t < 5.0] = np.nan
    phase = np.where(t < 5.0, 'creep', 'planner').astype(object)
    return {'sim_time': t, 'phase': phase, 'z_tgt': z_tgt,
            'load_z': np.where(t < 5.0, 0.05, np.nan_to_num(z_tgt, nan=0.05) + z_off),
            'load_vz': np.where((t >= 5.0) & (t < 9.5), 0.1, 0.0),
            'tilt_deg': np.full(t.size, 1.0), 'land': (t >= land_at).astype(float),
            'z_bias': np.zeros(t.size)}


def test_hold_window_is_the_last_n_seconds_before_land():
    p = _planner()
    t0, t1, hold_s = M.hold_window(p['sim_time'], p['z_tgt'], 0.5, p['land'], 20.0)
    assert t1 == pytest.approx(35.0, abs=0.05)
    assert t0 == pytest.approx(15.0, abs=0.05)
    assert hold_s == pytest.approx(35.0 - 9.5, abs=0.15)


def test_hold_window_is_none_when_the_target_is_never_reached():
    p = _planner()
    assert M.hold_window(p['sim_time'], p['z_tgt'], 0.6, p['land']) is None


def test_hold_z_error_is_signed_mean_minus_target():
    p = _planner(z_off=-0.03)
    out = M.tethered_metrics(p, 0.5, 20.0)
    assert out['hold_z_err_m'] == pytest.approx(-0.03, abs=1e-9)
    assert out['hold']['full'] is True
    assert out['max_payload_vz_mps'] == pytest.approx(0.1)


def test_ref_age_fraction_is_time_weighted_and_airborne_only():
    t = np.arange(0.0, 10.0, 0.1)
    age = np.where((t >= 6.0) & (t < 8.0), 0.5, 0.05)
    air = t >= 2.0
    assert M.ref_age_frac(t, age, air) == pytest.approx(2.0 / 8.0, abs=0.02)
    age[:20] = 5.0                                        # stale while on the floor: not counted
    assert M.ref_age_frac(t, age, air) == pytest.approx(2.0 / 8.0, abs=0.02)


def test_ref_age_fraction_skips_samples_without_an_age():
    t = np.arange(0.0, 4.0, 0.1)
    age = np.full(t.size, 0.1)
    age[:10] = np.nan
    assert M.ref_age_frac(t, age) == pytest.approx(0.0)


# ── solve failures ───────────────────────────────────────────────────────────

def test_solve_failures_count_ocp_ticks_only():
    """A creep tick's solve_status is the priming solve's (card A critic): not counted."""
    status = np.array([4, 4, 0, 4, 0, 0, 20], float)
    published = np.array(['', '', 'solved', 'shifted', 'solved', 'none', 'shifted'], dtype=object)
    out = M.planner_solve_failures(status, published)
    assert out['ticks'] == 2 and out['not_solved'] == 3 and out['n'] == 2


def test_solve_failures_fall_back_to_phase_without_a_published_column():
    status = np.array([4, 0, 4, 4], float)
    phase = np.array(['creep', 'planner', 'planner', 'planner'], dtype=object)
    assert M.planner_solve_failures(status, None, phase)['n'] == 2


def test_solve_failures_from_log_lines_ignore_status_zero():
    text = ('[planner] solve status 4 (12 ms) — holding\n[planner] solve status 0 x\n'
            '[planner] solve status 20 (60 ms) — publishing\n')
    out = M.planner_solve_failures(log_text=text)
    assert out['lines'] == 2 and out['n'] == 2


# ── carried fraction ─────────────────────────────────────────────────────────

def _write_tethered_run(tmp_path, ring_kg=0.86, law=None, thr_out=True, frac=1.0):
    """A level four-drone hover whose thrust carries `frac` of the ring's weight under `law`."""
    law = law or tf.LAW_0930
    root = tmp_path / 'logs' / 'controller_quad_load'
    t = np.arange(100.0, 130.0, 0.02)
    pl = root / 'load_planner_20261001_120000'
    pl.mkdir(parents=True)
    tp = np.arange(100.0, 130.0, 0.1)
    z = np.where(tp < 105.0, 0.05, 0.5)
    with open(pl / 'log.csv', 'w') as fh:
        fh.write('sim_time,phase,load_z,load_vz\n')
        for ti, zi in zip(tp, z):
            fh.write(f'{ti:.3f},{"creep" if ti < 105 else "planner"},{zi:.4f},0.0\n')
    v = 23.5
    for i in range(4):
        d = root / f'planner_drone{i}_20261001_12000{i}'
        d.mkdir()
        m_sup = tf.DRONE_KG + frac * ring_kg / 4.0
        u = law['a'][i] + law['b'] * m_sup + law['c'] * (v - tf.V_REF)
        cols = 'sim_time,u2,battery_v,pose_qw,pose_qx,pose_qy,pose_qz' + (',thr_out' if thr_out else '')
        with open(d / 'log.csv', 'w') as fh:
            fh.write(cols + '\n')
            for ti in t:
                fh.write(f'{ti:.3f},{u:.6f},{v},1,0,0,0' + (f',{u:.6f}' if thr_out else '') + '\n')
        (d / 'params.json').write_text(json.dumps({'thrust_offset': 0.0}))
    return str(tmp_path)


def test_carried_fraction_of_an_exact_hover_is_one(tmp_path):
    run = _write_tethered_run(tmp_path)
    (p, trackers), = tf.launches(run)
    cf = M.carried_fraction(p, trackers, 0.86, tf.LAW_0930, tf.DRONE_KG, window=(110.0, 130.0))
    assert cf['airborne_median'] == pytest.approx(1.0, abs=1e-6)
    assert cf['hold_median'] == pytest.approx(1.0, abs=1e-6)


def test_carried_fraction_uses_the_true_ring_mass(tmp_path):
    """Mismatch arm: the drones carry a 0.70 kg ring exactly, whatever the planner believes."""
    run = _write_tethered_run(tmp_path, ring_kg=0.70)
    (p, trackers), = tf.launches(run)
    assert M.carried_fraction(p, trackers, 0.70)['airborne_median'] == pytest.approx(1.0, abs=1e-6)
    assert M.carried_fraction(p, trackers, 0.86)['airborne_median'] == pytest.approx(0.70 / 0.86, abs=1e-6)


def test_linear_plant_law_carries_the_legacy_sim_hover(tmp_path):
    law = M.plant_law(str(tmp_path), 4, 'linear')
    assert law['a'] == [0.0] * 4 and law['c'] == 0.0
    run = _write_tethered_run(tmp_path, law=law, frac=0.9)
    (p, trackers), = tf.launches(run)
    assert M.carried_fraction(p, trackers, 0.86, law)['airborne_median'] == pytest.approx(0.9, abs=1e-4)


def test_rig_plant_law_takes_the_bridges_offsets(tmp_path):
    prm = tmp_path / 'params'
    prm.mkdir()
    for i, a in enumerate((0.19, 0.2, 0.21, 0.22)):
        (prm / f'bf_comm_{i}.yaml').write_text(
            f'/bf_comm_{i}:\n  ros__parameters:\n    thrust_map: rig\n    thrust_offset: {a}\n')
    law = M.plant_law(str(tmp_path), 4)
    assert law['map'] == 'rig' and law['a'] == [0.19, 0.2, 0.21, 0.22]
    assert law['b'] == tf.LAW_0930['b']


def test_sim_run_without_bridge_readback_is_linear(tmp_path):
    assert M.plant_law(str(tmp_path), 4)['map'] == 'linear'


# ── rig fixture: model-f1 ────────────────────────────────────────────────────

def _f1():
    with open(os.path.join(F1, 'model_f1_solve_status.log')) as fh:
        return M.tethered_run_metrics(F1, log_text=fh.read())


def test_model_f1_carried_fraction_matches_the_force_balance_table():
    """docs/rig_2026-09-30_force_balance.md: model_f1 172526, airborne 12.3 s, median 0.780."""
    cf = _f1()['carried_fraction']
    assert cf['airborne_median'] == pytest.approx(0.780, abs=0.002)
    assert cf['airborne_s'] == pytest.approx(12.3, abs=0.2)
    assert cf['law'] == 'rig'


def test_model_f1_never_held_its_target():
    """Critic C2: z_tgt reached 0.49 for 1.1 s and never 0.5, so there is no hold to score."""
    out = _f1()
    assert out['target_z'] == pytest.approx(0.5)
    assert out['hold'] is None
    assert 'heave' not in out


def test_model_f1_solve_failures_come_from_the_log_lines():
    """The pre-W1 planner log has no solve_status column: 25 'solve status 4' lines."""
    sf = _f1()['solve_failures']
    assert sf['ticks'] is None and sf['lines'] == 25 and sf['n'] == 25


def test_model_f1_airborne_tilt_and_vz():
    out = _f1()
    assert out['tilt_max_airborne_deg'] == pytest.approx(15.98, abs=0.01)
    assert 0.25 < out['max_payload_vz_mps'] < 0.35
    assert 'ref_age_frac' not in out                  # not logged before W1


# ── criteria and the WAIT_LIFT timeout ───────────────────────────────────────

def test_tethered_criteria_keys_parse():
    c = Criteria(max_planner_solve_fail=1, max_hold_z_err_m=0.05, hold_window_s=25,
                 max_heave_pp_m=0.03, max_payload_vz_mps=0.15, min_carried_fraction=0.9,
                 max_ref_age_frac=0.05, max_hold_tilt_mean_deg=2.0, max_z_bias_abs=0.1,
                 report_only=True)
    assert c.hold_window_s == 25.0 and c.report_only is True


def _checks(teth, **kw):
    import run_experiment as R
    return R.tethered_checks(Criteria(**kw), teth)


def test_tethered_checks_pass_and_fail_on_the_numbers():
    teth = M.tethered_metrics(_planner(z_off=-0.03), 0.5, 20.0)
    teth['solve_failures'] = {'n': 2}
    got = {n: ok for n, ok, _ in _checks(teth, max_hold_z_err_m=0.05, max_planner_solve_fail=1,
                                          max_heave_pp_m=0.03, max_payload_vz_mps=0.15)}
    assert got == {'hold |mean z - target|': True, 'planner solve failures': False,
                   'hold heave p-p': True, 'ring |vz| hand-over to hold end': True}


def test_tethered_checks_fail_a_hold_that_never_happened():
    got = _checks(_f1(), max_hold_z_err_m=0.05)
    assert got and not any(ok for _, ok, _ in got)


def test_tethered_checks_fail_when_there_is_no_planner_log():
    got = _checks(None, max_planner_solve_fail=1)
    assert len(got) == 1 and got[0][1] is False


def test_report_only_never_fails_the_run():
    import run_experiment as R

    class Cfg:
        criteria = Criteria(report_only=True, max_planner_solve_fail=0, forbid_abort=True)
        n_total = 4
    rows = [{'t': 0.0, 'payload_z': 0.1}, {'t': 1.0, 'payload_z': 0.2}]
    ok, checks = R.evaluate(Cfg, rows, None, [(0.5, 'abort')], {'solve_failures': {'n': 3}})
    assert ok is True
    assert all('(report only)' in n for n, _, _ in checks)
    assert not all(g for _, g, _ in checks)


def test_wait_lift_arg_takes_an_optional_timeout():
    assert wait_lift_arg(None) == (0.5, None)
    assert wait_lift_arg(0.45) == (0.45, None)
    assert wait_lift_arg('0.3 60') == (0.3, 60.0)
    with pytest.raises(ValueError):
        Event(8.0, 'WAIT_LIFT', '0.3 60 9')
    with pytest.raises(ValueError):
        Event(8.0, 'WAIT_LIFT', '0.3 -1')

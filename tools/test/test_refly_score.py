"""tools/refly_score.py and release_bands' 1 m band options (thesis R, card docs/experiments/2026-10-23_refly_1m.md):
the resolution and agreement rules, S on synthetic runs, the 0.55 m cards' verdicts reproduced from their own runs, and
a band with given factors and sigma flights."""
import math
import os
import sys
from types import SimpleNamespace

import numpy as np
import pandas as pd
import pytest

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HERE)
import refly_score as F  # noqa: E402
import release_bands as rb  # noqa: E402
import thesis_runsets as RS  # noqa: E402

HAVE_RESULTS = os.path.isdir(os.path.join(RS.REPO, 'results', '2026-10-16', 'R1887_sim_gz_ta_a1_r1'))


def test_resolution_is_tier_a_freeze_1_2():
    a1, a2 = [6.05, 7.68, 5.46], [16.68, 16.64, 17.08]           # T3 on the freeze tree (card rev 3)
    r = F.resolved(a1, a2, 1, 1.0)
    assert r['resolved'] and r['gap'] == pytest.approx(10.40, abs=0.01) and r['need'] == pytest.approx(4.44)
    assert not F.resolved(a2, a1, 1, 1.0)['resolved']            # the other sign
    assert not F.resolved([1.0, 2.0], [1.5, 3.0], 1, 0.1)['c2']   # overlapping ranges
    assert not F.resolved([1.0, 1.1], [1.6, 1.7], 1, 1.0)['c3']   # under the floor


def test_agreement_factor_and_floor():
    assert F.agrees([8.0], [15.0], 1.0) and not F.agrees([8.0], [17.0], 1.0)
    assert F.agrees([0.1], [0.6], 1.0)                             # a factor of 6 inside the floor
    assert not F.agrees([4.0], [-4.0], 1.0) and F.agrees([2.0], [], 1.0) is None


def _run(root, rid, cfg, events, log='', armed_end=0, n=3):
    d = root / '2026-10-28' / f'{rid}_sim_gz_{cfg}' / 'logs'
    d.mkdir(parents=True)
    pd.DataFrame([dict(sim_time=t, event=e, arg='') for t, e in events]).to_csv(d / 'events.csv', index=False)
    (d / 'launch.log').write_text(log)
    t = np.arange(0.0, 60.0, 0.5)
    pd.DataFrame({'t': t, **{f'd{i}_armed': np.where(t < 59, 1, armed_end) for i in range(n)}}).to_csv(
        d / 'run.csv', index=False)
    return rid


def test_safety_on_synthetic_runs(tmp_path, monkeypatch):
    monkeypatch.setattr(RS, 'RESULTS', str(tmp_path))
    monkeypatch.setattr(F, 'm_land', lambda rid: {'landed_dz_cm': 2.0})
    ok = [(3, 'ARM'), (6, 'TAKEOFF'), (20, 'RELEASE'), (40, 'LAND'), (50, 'LANDED')]
    assert F.safety(_run(tmp_path, 'R9001', 'r1m_ta_a1_r1', ok, '[dissipative] UNANNOUNCED DETACH: x'))['ok']
    s = F.safety(_run(tmp_path, 'R9002', 'r1m_ta_a1_r1', ok, 'DROP AND LAND: tilt'))
    assert not s['ok'] and 'drop not expected' in s['fails']
    assert F.safety(_run(tmp_path, 'R9003', 'r1m_ta_a6_r1', ok, 'DROP AND LAND: tilt'))['ok']    # T8 support, ">="
    s = F.safety(_run(tmp_path, 'R9004', 'r1m_dl_d1_second_loss', ok))
    assert not s['ok'] and 'expected drop missing' in s['fails']
    s = F.safety(_run(tmp_path, 'R9005', 'r1m_app_x0_hold_eps0', [(3, 'ARM'), (40, 'LAND'), (50, 'LANDED')],
                      '[tracker] UNANNOUNCED DETACH: y'))
    assert 'unannounced detach with no release' in s['fails']
    landing_abort = [(3, 'ARM'), (20, 'RELEASE'), (40, 'LAND'), (47, 'FLEET_ABORT')]
    log = '[tracker-1] [Drone 2] ENVELOPE FAULT: drone tilt 80.9 deg > 60.0'
    s = F.safety(_run(tmp_path, 'R9006', 'r1m_ta_a6_r2', landing_abort, log, armed_end=0))
    assert s['ok'] and s['notes']                                   # as A6 did at 0.55 m (3 of 3)
    assert not F.safety(_run(tmp_path, 'R9007', 'r1m_ta_a1_r2', landing_abort, log))['ok']
    monkeypatch.setattr(F, 'm_land', lambda rid: {'landed_dz_cm': 44.0})
    s = F.safety(_run(tmp_path, 'R9008', 'r1m_app_mapneg_ref', [(3, 'ARM'), (40, 'LAND'), (50, 'LANDED')]))
    assert not s['ok'] and s['fails'][0].startswith('false touchdown')


@pytest.mark.skipif(not HAVE_RESULTS, reason='no 0.55 m run directories')
def test_the_055_lines_reproduce_the_cards():
    want = {'tier_a': {'T3': True, 'T8': True, 'T9': True}, 't15': {'T15': True},
            'x0': {'C-X1': True, 'C-X2': True}, 'robust': {'C-R1': True, 'C-R2': True, 'C-R3': True, 'C-R4': True},
            'land': {'C-L1': False, 'C-L2': True},             # C-L1: the 17 Oct reviewer's four false touchdowns
            'disturbance': {'C-D5': False}}                     # falsified at 0.55 m (not resolved)
    for key, claims in want.items():
        res = F.score(key)
        got = {}
        for name, ok, _ in res['C_old']:
            got.setdefault(name.split()[0], []).append(ok)
        for claim, ok in claims.items():
            assert got[claim] and all(v == ok for v in got[claim]), (key, claim, got.get(claim))
    carry = F.score('T7t')['C_old']
    assert [n.split()[-1] for n, ok, _ in carry if not ok] == ['R1875']    # the spin at 0.3 m/s, bar 3
    assert all(ok for _, ok, _ in F.score('drop')['C_old']) and all(ok for _, ok, _ in F.score('ledger_w5')['C_old'])


def _rb_run(root, sub, tilt_deg, events):
    d = root / sub / 'logs'
    d.mkdir(parents=True)
    t = np.arange(0.0, 50.0, 0.02)
    a = np.radians(tilt_deg(t))
    pd.DataFrame({'t': t, 'payload_qw': np.cos(a / 2), 'payload_qx': np.sin(a / 2), 'payload_qy': 0.0,
                  'payload_qz': 0.0, 'payload_ref_x': 0.0, 'payload_ref_y': 0.0, 'payload_ref_z': 1.0}).to_csv(
        d / 'run.csv', index=False)
    pd.DataFrame([dict(event=e, sim_time=s) for e, s in events]).to_csv(d / 'events.csv', index=False)


def test_a_1m_band_takes_given_factors_and_sigma_flights(tmp_path, monkeypatch):
    monkeypatch.setattr(rb, 'RESULTS', str(tmp_path))
    wob = lambda t: 0.3 * np.sin(2 * np.pi * 1.3 * t)  # noqa: E731
    for rid, step in (('R9101', 5.0), ('R9102', 7.0)):
        _rb_run(tmp_path, f'2026-10-28/{rid}_sim_gz_r1m_w4_da_release_hover',
                lambda t, s=step: wob(t) + np.where(t >= 30.0, s, 0.0), [('RELEASE', 30.0), ('LAND', 48.0)])
    hover = lambda t: 1.0 + 0.8 * np.sin(2 * np.pi * 0.7 * t)  # noqa: E731
    _rb_run(tmp_path, 'rig/2026-10-28/h1_run', hover, [('TAKEOFF', 2.0), ('LAND', 40.0)])
    tw = [rb.Run(r).metrics(rb.BAND_HORIZON_S)['dtheta'] for r in ('R9101', 'R9102')]
    t = np.arange(0.0, 50.0, 0.02)
    w = hover(t)[(t >= 40.0 - rb.SIGMA_HOVER_S) & (t < 40.0)]
    s = math.sqrt(np.mean((w - w.mean()) ** 2))
    a = SimpleNamespace(factors='1,1', sigma_runs='rig:2026-10-28/h1', band_horizon=rb.BAND_HORIZON_S,
                        classes='announce', pair_horizon='rig', ratio='mean', sigma='pre')
    b = rb.band(a, 'W4', ['R9101', 'R9102'])
    assert b['sigma'] == pytest.approx(s, rel=1e-3) and b['f_source'] == 'given'
    assert (b['lo'], b['hi']) == (pytest.approx(max(0.0, min(tw) - 2 * s)), pytest.approx(max(tw) + 2 * s))
    assert rb.sigma_from(['rig:2026-10-28/h1'])[1] == len(w)


@pytest.mark.skipif(not os.path.isdir(os.path.join(rb.RESULTS, 'rig', '2026-10-08', 'r303_4_run')),
                    reason='no 8 Oct rig flights')
def test_the_default_band_is_still_the_frozen_w4_band():
    a = SimpleNamespace(factors=None, sigma_runs=None, band_horizon=rb.BAND_HORIZON_S, classes='announce',
                        pair_horizon='rig', ratio='mean', sigma='pre')
    b = rb.band(a, 'W4', ['R1459', 'R1460', 'R1626'])
    assert (round(b['lo'], 2), round(b['hi'], 2)) == (1.75, 14.51)

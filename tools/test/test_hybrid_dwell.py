"""hybrid_dwell recovers a known jump and decay rate from a synthetic run directory."""
import csv
import math
import sys
import pathlib

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import hybrid_dwell  # noqa: E402


def _synthetic(tmp_path, lam=0.2, jump=30.0, ss=4.0, pre=34.0):
    d = tmp_path / 'R9999_sim_gz_test'
    (d / 'logs').mkdir(parents=True)
    t = np.arange(0.0, 60.0, 0.02)
    tilt = np.full_like(t, pre)
    te, tpk = 20.0, 23.0
    post = t >= te
    tilt[post] = ss + (pre - ss) * np.exp(-2.0 * (t[post] - te))          # old state decays
    peak = t >= tpk
    tilt[peak] = ss + jump * np.exp(-lam * (t[peak] - tpk))
    rise = (t >= te) & (t < tpk)
    tilt[rise] = pre + (ss + jump - pre) * (t[rise] - te) / (tpk - te)
    with open(d / 'logs' / 'run.csv', 'w', newline='') as fh:
        w = csv.writer(fh); w.writerow(['t', 'payload_tilt_deg'])
        w.writerows(zip(t, tilt))
    with open(d / 'logs' / 'events.csv', 'w', newline='') as fh:
        w = csv.writer(fh); w.writerow(['sim_time', 'event', 'arg'])
        w.writerows([[3.0, 'ARM', ''], [te, 'WELD', 'magnet/object_attached'], [55.0, 'LAND', '']])
    return d


def test_decay_rate_and_dwell_are_recovered(tmp_path):
    d = _synthetic(tmp_path, lam=0.2, jump=30.0, ss=4.0)
    (r,) = hybrid_dwell.analyse(d, eps=3.0, hold_s=10.0)
    assert abs(r['lam'] - 0.2) < 0.02
    assert abs(r['tilt_peak'] - 34.0) < 0.5
    assert abs(r['t_dwell'] - math.log(30.0 / 3.0) / 0.2) < 1.0
    assert abs(r["rho"] - math.exp(-2.0)) < 0.03


def test_non_events_are_ignored(tmp_path):
    d = _synthetic(tmp_path)
    rows = hybrid_dwell.analyse(d, eps=3.0)
    assert [x['event'] for x in rows] == ['WELD magnet/object_attached']

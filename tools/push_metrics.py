#!/usr/bin/env python3
"""tools/push_metrics.py -- the push-arm numbers for card 2026-10-04_z_int_model: the ring's response to
a WRENCH step along the push axis, against the 5 s before the push (run.csv, events.csv), and the
plan-prediction error (planner log.csv: plan5_* against the ring 5 nodes later).

    python3 tools/push_metrics.py R0961 R0962 ...

Per run: the force and axis; steady displacement (last 5 s of the push) and K = F / displacement;
the time to 63 % of it (tau); peak displacement; release overshoot (peak past the pre-push level
after the push ends); settling (time after release to stay within 1 cm); and over the 10 s before
LAND, the mean plan-prediction error |plan5 - measured| per axis (cm).
"""
import csv
import glob
import os
import sys

import numpy as np
import pandas as pd

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def run_dir(rid):
    return glob.glob(os.path.join(REPO, 'results', '*', f'{rid}_*'))[0]


def push(rid):
    d = run_dir(rid)
    ev = list(csv.DictReader(open(os.path.join(d, 'logs', 'events.csv'))))
    run = pd.read_csv(os.path.join(d, 'logs', 'run.csv')).drop_duplicates('t')
    wr = [(float(e['sim_time']), e['arg']) for e in ev if e['event'] == 'WRENCH']
    land = [float(e['sim_time']) for e in ev if e['event'] == 'LAND']
    out = dict(run=rid, config=os.path.basename(d), aborted=any('ABORT' in e['event'] for e in ev),
               landed=any(e['event'] == 'LANDED' for e in ev))
    if len(wr) < 2 or not land:
        out['void'] = True
        return out
    (on, arg), (off, _) = wr[0], wr[1]
    f = np.array([float(v) for v in arg.split()[:3]])
    ax = int(np.argmax(np.abs(f)))
    col = 'payload_' + 'xyz'[ax]
    sgn = np.sign(f[ax])
    t = run.t.to_numpy()
    p = run[col].to_numpy()
    base = p[(t >= on - 5) & (t < on)].mean()
    y = sgn * (p - base) * 100.0                    # cm along the push
    win = (t >= off - 5) & (t < off)
    steady = y[win].mean()
    during = (t >= on) & (t < off)
    peak = y[during].max()
    tau = np.nan
    if steady > 0.5:
        hit = np.flatnonzero(during & (y >= 0.63 * steady))
        tau = t[hit[0]] - on if hit.size else np.nan
    after = (t >= off) & (t < land[0])
    over = -y[after].min()                           # past the pre-push level, against the push
    outside = np.flatnonzero(after & (np.abs(y) > 1.0))
    settle = (t[outside[-1]] - off) if outside.size else 0.0
    out.update(axis='xyz'[ax], force_n=float(f[ax]), steady_cm=float(steady), peak_cm=float(peak),
               k_n_per_m=float(abs(f[ax]) / (steady / 100.0)) if steady > 0.5 else np.nan,
               tau_s=float(tau), release_overshoot_cm=float(over), settle_s=float(settle))
    out.update(plan_prediction(d, land[0]))
    return out


def plan_prediction(d, land_run_t):
    logs = glob.glob(os.path.join(d, 'logs', '*planner', '*', 'log.csv'))
    if not logs:
        return {}
    L = pd.read_csv(logs[0])
    if 'plan5_z' not in L or L.plan5_z.isna().all():
        return {}
    L = L[L.phase == 'planner'].dropna(subset=['plan5_z'])
    ts = L.sim_time.to_numpy()
    w = (L.land == 0).to_numpy() & (ts >= ts[L.land.to_numpy() == 0].max() - 10.0)
    res = {}
    for a in 'xyz':
        meas = np.interp(ts + 0.5, ts, L[f'load_{a}'].to_numpy())   # node 5 at node_dt 0.1 s
        e = (L[f'plan5_{a}'].to_numpy() - meas)[w & (ts + 0.5 <= ts.max())]
        res[f'plan_err_{a}_cm'] = float(100 * np.mean(e))
    return res


def main():
    rows = [push(r) for r in sys.argv[1:]]
    keys = ['run', 'axis', 'force_n', 'steady_cm', 'k_n_per_m', 'tau_s', 'peak_cm',
            'release_overshoot_cm', 'settle_s', 'plan_err_x_cm', 'plan_err_y_cm', 'plan_err_z_cm',
            'aborted', 'landed']
    print('| ' + ' | '.join(keys) + ' |')
    print('|' + '---|' * len(keys))
    for r in rows:
        print('| ' + ' | '.join(f'{r[k]:.2f}' if isinstance(r.get(k), float) else str(r.get(k, ''))
                                for k in keys) + ' |')


if __name__ == '__main__':
    main()

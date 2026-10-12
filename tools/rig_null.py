#!/usr/bin/env python3
"""tools/rig_null.py -- the rig's dtheta null and the minimum detectable effect of a lever pair at n = 3 (thesis plan P6).

    python3 tools/rig_null.py [--windows 7 15] [--out DIR] [--rig-runs DIR] [--convert]

The null: dtheta (T6, docs/freeze_2026-10-14.md 1.1, computed by tools/release_bands.py's Run) at pseudo-events in rig
flight where nothing happens: every 0.5 s at which the ring reference had been still for 5 s (the freeze's Q12 test:
height within 5 mm, speed within 0.02 m/s) and stays so, with no release, slip, LAND or fleet stop, for the window W
after it (W = 7 s, about the 8 Oct hovers' hold, and 15 s, the W4/W5 horizon). Hover and path (circle, figure-8, spin)
apart. Flights: every 7 and 8 Oct bag that flew the ring (tools/rig_to_run.py run directories made with the bag:
<results>/rig/<date>/<run>_run, else --rig-runs, where --convert builds them; ROS sourced).
The minimum detectable effect (MDE): the smallest true difference delta between two arms of n runs that the frozen
resolution rule (freeze 1.2: the frozen sign, the ranges apart, |mean B - mean A| >= max(1 deg, 2 x the larger
range)) resolves in at least 80 % of 20000 simulated pairs, each run's dtheta = its arm's true value + noise drawn
(a) from a normal with the null's sd, (b) from the null's own values about their mean (bootstrap), (c) from a normal
with sigma_rig (freeze 2.4, the pooled pre-window spread, 1.568 deg) for comparison.
"""
import argparse
import csv
import glob
import json
import os
import sys

import numpy as np
import pandas as pd

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, 'tools'))
RESULTS_DEFAULT = '/home/wesley/multi_drone_control/results'
FLIGHTS = [('2026-10-07', r) for r in ('r000014', 'r0001', 'r0005', 'r0007', 'r0012', 'r0013', 'r002', 'r006', 'r011',
                                        'r200006', 'r20004', 'r2008e60', 'r206e60')] + \
          [('2026-10-08', r) for r in ('r302_3', 'r303_4', 'r304_3', 'r305')]
STOPS = ('LAND', 'DETACH', 'RELEASE', 'SLIP', 'DETECTED', 'DROP', 'FLEET_ABORT', 'ESTOP', 'DISARM')
STEP_S, PRE_S, AIR_DZ = 0.5, 5.0, 0.05
SIGMA_RIG = 1.568                      # freeze 2.4
TRIALS, POWER = 20000, 0.8


def run_dir(results, rig_runs, date, run, convert):
    for root in (os.path.join(results, 'rig'), rig_runs):
        d = os.path.join(root, date, run + '_run')
        if os.path.isfile(os.path.join(d, 'logs', 'run.csv')):
            return d
    if not convert:
        return None
    import rig_to_run
    base = os.path.join(results, 'rig', date, run)
    out = os.path.join(rig_runs, date, run + '_run')
    rig_to_run.convert(sorted(glob.glob(base + '_logs/logs/tracker/drone*_*')), out, bag=base, t2_log=base + '.log')
    return out


def pseudo_events(rb, d, windows):
    """[(W, t_p, class, dtheta, sigma_pre)] of one flight."""
    ev = pd.read_csv(os.path.join(d, 'logs', 'events.csv'))
    run = rb.Run(d, te=0.0, kind='null', tland=0.0)
    r = pd.read_csv(os.path.join(d, 'logs', 'run.csv')).dropna(subset=['payload_qw']).reset_index(drop=True)
    z = r.payload_z.to_numpy(float)
    air_t = r.t.to_numpy(float)[z > np.nanmedian(z[:50]) + AIR_DZ]
    if not len(air_t):
        return []
    to = ev[ev.event == 'TAKEOFF'].sim_time
    t_start = max(float(air_t[0]), float(to.iloc[0]) if len(to) else -np.inf)
    stop = ev[ev.event.isin(STOPS) & (ev.sim_time > t_start)].sim_time
    t_stop = min(float(stop.min()) if len(stop) else np.inf, float(air_t[-1]))
    out = []
    for W in windows:
        for tp in np.arange(t_start + PRE_S, t_stop - W, STEP_S):
            run.te = float(tp + W)                    # still from tp - 5 s through tp + W
            if run.ref_still_s() < W + PRE_S:
                continue
            run.te, run.tland = float(tp), float(tp + W)
            m = run.metrics(None)
            w = run.win(tp - PRE_S, tp + W)
            v = np.linalg.norm(np.diff(run.ref[w][:, :2], axis=0), axis=1).sum() / (W + PRE_S)
            out.append((W, float(tp), 'hover' if v < 0.01 else 'path', m['dtheta'], m['sigma_pre']))
    return out


def resolved(a, b):
    """Freeze 1.2 with the frozen sign b > a."""
    ma, mb = a.mean(1), b.mean(1)
    ra, rb_ = np.ptp(a, 1), np.ptp(b, 1)
    return (mb > ma) & ((a.max(1) < b.min(1)) | (b.max(1) < a.min(1))) & \
        (np.abs(mb - ma) >= np.maximum(1.0, 2 * np.maximum(ra, rb_)))


def mde(noise, n, rng, deltas=np.arange(0.0, 20.01, 0.1)):
    """(smallest delta resolved in >= 80 % of trials, P(resolved) at each delta)."""
    p = []
    for dl in deltas:
        a, b = noise(rng, (TRIALS, n)), noise(rng, (TRIALS, n)) + dl
        p.append(float(resolved(a, b).mean()))
    p = np.array(p)
    hit = np.nonzero(p >= POWER)[0]
    return (float(deltas[hit[0]]) if len(hit) else np.inf), dict(zip(np.round(deltas, 1), p))


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--results', default=RESULTS_DEFAULT)
    ap.add_argument('--rig-runs', default=os.path.join(REPO, 'results', 'rig_runs'))
    ap.add_argument('--convert', action='store_true')
    ap.add_argument('--windows', nargs='+', type=float, default=[7.0, 15.0])
    ap.add_argument('--out', default=os.path.join(REPO, 'results', 'analysis', 'p6_2026-10-12', '5a_null'))
    a = ap.parse_args()
    os.environ['MDC_RESULTS_ROOT'] = a.results
    import release_bands as rb
    os.makedirs(a.out, exist_ok=True)
    rows = []
    for date, run in FLIGHTS:
        d = run_dir(a.results, a.rig_runs, date, run, a.convert)
        if d is None:
            print(f'{date}/{run}: no run directory (--convert)', file=sys.stderr)
            continue
        for W, tp, cls, dth, sig in pseudo_events(rb, d, a.windows):
            rows.append(dict(date=date, run=run, window_s=W, t_pseudo=round(tp, 2), traj=cls, dtheta_deg=round(dth, 3),
                             sigma_pre_deg=round(sig, 3)))
    with open(os.path.join(a.out, 'null_pseudo_events.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    df = pd.DataFrame(rows)
    rng = np.random.default_rng(12)
    summ, curves = [], []
    for (W, cls), g in df.groupby(['window_s', 'traj']):
        x = g.dtheta_deg.to_numpy(float)
        sd = float(np.std(x, ddof=1))
        per_flight = g.groupby('run').dtheta_deg.agg(['count', 'mean', 'std'])
        base = x - x.mean()
        row = dict(window_s=W, traj=cls, pseudo_events=len(x), flights=int(per_flight.shape[0]),
                   flights_list=' '.join(per_flight.index), null_mean=round(float(x.mean()), 2),
                   null_sd=round(sd, 2), null_p95=round(float(np.percentile(x, 95)), 2),
                   null_max=round(float(x.max()), 2), sd_of_flight_means=round(float(per_flight['mean'].std()), 2),
                   sigma_pre_mean=round(float(g.sigma_pre_deg.mean()), 2))
        for n in (2, 3):
            for lab, fn in (('normal', lambda r, s, sd=sd: r.normal(0.0, sd, s)),
                            ('bootstrap', lambda r, s, b=base: r.choice(b, s)),
                            ('sigma_rig', lambda r, s: r.normal(0.0, SIGMA_RIG, s))):
                m_, curve = mde(fn, n, rng)
                row[f'mde_n{n}_{lab}'] = round(m_, 1)
                curves.append(dict(window_s=W, traj=cls, n=n, noise=lab,
                                   **{f'p_at_{k}': round(v, 3) for k, v in curve.items() if k in (0.8, 1.0, 2.0, 3.0,
                                                                                                    5.0, 8.0, 10.0)}))
        summ.append(row)
    for name, data in (('null_summary.csv', summ), ('mde_power.csv', curves)):
        with open(os.path.join(a.out, name), 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=list(data[0]))
            w.writeheader()
            w.writerows(data)
    for data in (summ, curves):
        keys = list(data[0])
        print('| ' + ' | '.join(keys) + ' |')
        print('|' + '---|' * len(keys))
        for r in data:
            print('| ' + ' | '.join(str(r[k]) for k in keys) + ' |')
        print()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Q009: X1's second swing, read from the end of the first swing (lab_requests.md Q009, 9 Nov).

    python3 second_swing.py [--root <folder holding R38xx_sim_gz_x1_* run folders>]

Independent of docs/figures/review_2026-11-05/tab_x1.py. Per run, t = 0 at the RELEASE event (events.csv, the runner
clock that run.csv's t uses; the same t_e as tools/x1_metrics.py):
  tilt          the ring's tilt from vertical, recomputed from the logged quaternion: acos(1 - 2 (qx^2 + qy^2))
  first peak    the largest tilt in [0, PEAK_WIN_S]
  swing end     the first local minimum after that peak: the first sample whose tilt is the lowest within +-NBR_S
  second swing  the largest tilt in [swing end, +4.0 s]       (the thesis reading)
  F091 window   the largest tilt in [+1.0, +4.0 s]           (feed F091, results/analysis/x1_both_models_2026-11-07.md)
  det+4         the largest tilt in [swing end, detect + 4 s] (detect_s from results/analysis/x1_2026-11-06.json)
Sensitivity: the swing end with NBR_S 0.05 / 0.2 s and the peak window 1.0 / 2.0 s; with a 3-sample moving average.
Shape: the signed tilt about the first peak's tilt direction, atan2(n_h . u, n_z) with u the horizontal direction of the
ring's normal at the first peak: its value at the swing end, its lowest from the first peak to +4 s (negative = leaning
the other way, so the ring passed through level) and the largest tilt off that direction (the motion stays in one plane).
Writes per_run.csv next to this file. Since 13 Nov the run folders live on the lab PC
(drones:~/mdc_archive/results/2026-11-06/): fetch each run's logs/run.csv and logs/events.csv and pass --root.
"""
import argparse
import glob
import hashlib
import json
import os

import numpy as np
import pandas as pd

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
END_S = 4.0
PEAK_WIN_S = 1.5
NBR_S = 0.1
CELLS = ('h4', 'f4', 'sqTnoxy')
ARMS = (('nors', 'Baseline'), ('ours', 'Proposed'), ('d05', '0.5 s late'), ('d10', '1 s late'), ('d20', '2 s late'))
# What the thesis states (Q009): per run in run-id order within the arm, and the two-run means.
THESIS = {'nors': ((9.45, 10.34), 9.9), 'ours': ((23.44, 21.27), 22.4), 'd05': ((14.60, 14.11), 14.4),
          'd10': ((15.68, 15.13), 15.4), 'd20': ((10.31, 10.50), 10.4)}


def load(run_dir):
    run = pd.read_csv(os.path.join(run_dir, 'logs', 'run.csv'),
                      usecols=['t', 'payload_qw', 'payload_qx', 'payload_qy', 'payload_qz', 'payload_tilt_deg'])
    run = run.drop_duplicates('t').sort_values('t')
    ev = pd.read_csv(os.path.join(run_dir, 'logs', 'events.csv'))
    te = float(ev.loc[ev.event.astype(str) == 'RELEASE', 'sim_time'].iloc[0])
    t = run.t.to_numpy(float) - te
    qw, qx, qy, qz = (run[c].to_numpy(float) for c in ('payload_qw', 'payload_qx', 'payload_qy', 'payload_qz'))
    tilt = np.degrees(np.arccos(np.clip(1.0 - 2.0 * (qx * qx + qy * qy), -1.0, 1.0)))
    normal = np.column_stack((2.0 * (qx * qz + qw * qy), 2.0 * (qy * qz - qw * qx), 1.0 - 2.0 * (qx * qx + qy * qy)))
    col_err = float(np.nanmax(np.abs(tilt - run.payload_tilt_deg.to_numpy(float))))
    with open(os.path.join(run_dir, 'logs', 'run.csv'), 'rb') as fh:
        sha = hashlib.sha256(fh.read()).hexdigest()
    return t, tilt, normal, te, col_err, sha


def swing_end(t, tilt, peak_win=PEAK_WIN_S, nbr=NBR_S):
    first = (t >= 0.0) & (t <= peak_win)
    i_pk = np.flatnonzero(first)[np.argmax(tilt[first])]
    for i in range(i_pk + 1, len(t)):
        if t[i] > END_S:
            return t[i_pk], tilt[i_pk], None
        near = (t >= t[i] - nbr) & (t <= t[i] + nbr)
        if tilt[i] <= tilt[near].min():
            return t[i_pk], tilt[i_pk], i
    return t[i_pk], tilt[i_pk], None


def window_max(t, tilt, a, b):
    w = (t >= a - 1e-9) & (t <= b + 1e-9)
    j = np.flatnonzero(w)[np.argmax(tilt[w])]
    return float(tilt[j]), float(t[j])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--root', default=os.path.join(REPO, 'results', '2026-11-06'))
    args = ap.parse_args()
    scores = {r['run']: r for r in json.load(open(os.path.join(REPO, 'results', 'analysis', 'x1_2026-11-06.json')))}
    rows = []
    for cell in CELLS:
        for arm, label in ARMS:
            for d in sorted(glob.glob(os.path.join(args.root, f'R*_sim_gz_x1_{cell}_{arm}'))):
                rid = os.path.basename(d).split('_')[0]
                t, tilt, normal, te, col_err, sha = load(d)
                t_pk, th_pk, k = swing_end(t, tilt)
                t_end = float(t[k])
                sec, t_sec = window_max(t, tilt, t_end, END_S)
                f091, t_f091 = window_max(t, tilt, 1.0, END_S)
                i_pk = int(np.argmin(np.abs(t - t_pk)))
                u = normal[i_pk, :2] / np.linalg.norm(normal[i_pk, :2])
                along = np.degrees(np.arctan2(normal[:, :2] @ u, normal[:, 2]))
                off = np.degrees(np.arctan2(np.abs(normal[:, :2] @ np.array([-u[1], u[0]])), normal[:, 2]))
                span = (t >= t_pk) & (t <= END_S)
                det = scores[rid]['detect_s']
                det4, _ = window_max(t, tilt, t_end, det + END_S)
                sens = {}
                for name, kw in (('end_nbr005', dict(nbr=0.05)), ('end_nbr02', dict(nbr=0.2)),
                                 ('end_pk10', dict(peak_win=1.0)), ('end_pk20', dict(peak_win=2.0))):
                    _, _, kk = swing_end(t, tilt, **kw)
                    sens[name] = float(t[kk])
                sm = np.convolve(tilt, np.ones(3) / 3.0, mode='same')
                _, _, ks = swing_end(t, sm)
                sens['end_smooth3'] = float(t[ks])
                sens['second_swing_spread'] = float(max(window_max(t, tilt, v, END_S)[0] for v in sens.values())
                                                    - min(window_max(t, tilt, v, END_S)[0] for v in sens.values()))
                rows.append(dict(run=rid, cell=cell, arm=arm, label=label, t_release=te, detect_s=det,
                                 t_first_peak=float(t_pk), first_peak_deg=float(th_pk), t_swing_end=t_end,
                                 tilt_at_swing_end=float(tilt[k]), second_swing_deg=sec, t_second_peak=t_sec,
                                 f091_1to4_deg=f091, t_f091_peak=t_f091, equal_to_f091=bool(sec == f091),
                                 rise_from_end_deg=sec - float(tilt[k]), signed_at_end_deg=float(along[k]),
                                 signed_min_deg=float(along[span].min()), off_plane_max_deg=float(off[span].max()),
                                 tilt_at_1s=float(np.interp(1.0, t, tilt)), second_swing_to_det4_deg=det4,
                                 dt_median=float(np.median(np.diff(t))), tilt_col_max_err=col_err,
                                 run_csv_sha256=sha, **sens))
    df = pd.DataFrame(rows)
    df.to_csv(os.path.join(HERE, 'per_run.csv'), index=False, float_format='%.4f', lineterminator='\n')

    pd.set_option('display.width', 250)
    pd.set_option('display.max_columns', 40)
    print(df[['run', 'cell', 'arm', 't_first_peak', 'first_peak_deg', 't_swing_end', 'tilt_at_swing_end',
              'second_swing_deg', 't_second_peak', 'f091_1to4_deg', 'equal_to_f091', 'tilt_at_1s',
              'second_swing_to_det4_deg']].round(3).to_string(index=False))
    print('\nshape (deg): signed tilt along the first swing at its end, lowest from the peak to +4 s, off-plane max')
    print(df[['run', 'cell', 'arm', 'tilt_at_swing_end', 'signed_at_end_deg', 'signed_min_deg', 'second_swing_deg',
              'rise_from_end_deg', 'off_plane_max_deg']].round(2).to_string(index=False))
    print('\nsensitivity of the swing end (s) and the reading spread (deg):')
    print(df[['run', 'cell', 'arm', 't_swing_end', 'end_nbr005', 'end_nbr02', 'end_pk10', 'end_pk20', 'end_smooth3',
              'second_swing_spread']].round(3).to_string(index=False))
    print('\ntilt column vs quaternion, largest difference (deg):', df.tilt_col_max_err.max())

    print('\nh4 / f4: second swing equals the F091 window in', int(df[df.cell != 'sqTnoxy'].equal_to_f091.sum()), 'of',
          int((df.cell != 'sqTnoxy').sum()), 'runs')
    print('\nsqTnoxy against the thesis:')
    sq = df[df.cell == 'sqTnoxy']
    for arm, label in ARMS:
        a = sq[sq.arm == arm].sort_values('run')
        runs, mine = list(a.run), list(a.second_swing_deg)
        exp_runs, exp_mean = THESIS[arm]
        mean = float(np.mean(mine))
        print(f'{label:10s} {runs} mine {[round(v, 2) for v in mine]} thesis {list(exp_runs)} '
              f'diff {[round(m - e, 3) for m, e in zip(mine, exp_runs)]}  mean {mean:.2f} -> {mean:.1f} thesis '
              f'{exp_mean} {"OK" if round(mean, 1) == exp_mean else "MISMATCH"}  det+4 mean '
              f'{a.second_swing_to_det4_deg.mean():.2f}  ends {[round(v, 2) for v in a.t_swing_end]}')
    print('\nh4 / f4 two-run means (F091 window = this reading):')
    for cell in ('h4', 'f4'):
        c = df[df.cell == cell]
        print(cell, ' '.join(f'{label}: {c[c.arm == arm].second_swing_deg.mean():.2f}' for arm, label in ARMS))


if __name__ == '__main__':
    main()

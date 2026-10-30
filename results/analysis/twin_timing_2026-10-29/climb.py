#!/usr/bin/env python3
"""QUAD4's climb in a twin run: from his TAKEOFF to APPROACH_ABOVE_TARGET, or 12 s if he never gets there.
Max tilt (run.csv d3_tilt_deg), the height's turning points, his throttle command (his log u2) swing p-p and std
after a 1 s moving-mean detrend, its dominant frequency (Welch), and whether he reached the approach.

    climb.py <run dir> [<run dir> ...]
"""
import glob
import json
import os
import sys

import numpy as np
import pandas as pd
from scipy.signal import welch

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from align import sim_time_of  # noqa: E402


def turning_points(t, z, min_step=0.05):
    out = [(t[0], z[0])]
    direction = 0
    for i in range(1, len(z)):
        d = z[i] - out[-1][1]
        if direction >= 0 and z[i] > out[-1][1] and direction == 1:
            out[-1] = (t[i], z[i])
        elif direction <= 0 and z[i] < out[-1][1] and direction == -1:
            out[-1] = (t[i], z[i])
        elif abs(d) >= min_step:
            direction = 1 if d > 0 else -1
            out.append((t[i], z[i]))
    return out


def main(runs):
    res = {}
    for run in runs:
        ev = pd.read_csv(os.path.join(run, 'logs', 'events.csv'))
        e = ev.event.astype(str)
        t0 = float(ev[e == 'PHASE:TAKEOFF'].sim_time.iloc[0])
        ap = ev[e == 'PHASE:APPROACH_ABOVE_TARGET'].sim_time
        reached = len(ap) > 0
        t1 = float(ap.iloc[0]) if reached else t0 + 12.0
        x = pd.read_csv(os.path.join(run, 'logs', 'run.csv'))
        w = x[(x.t >= t0) & (x.t <= t1)]
        tilt = float(w.d3_tilt_deg.max())
        t_tilt = float(w.t.values[np.argmax(w.d3_tilt_deg.values)])
        tp = turning_points(w.t.values, w.d3_z.values)
        m = pd.read_csv(glob.glob(os.path.join(run, 'his_mpc', '*', 'log.csv'))[0])
        to_sim, _, _ = sim_time_of(x, m)
        mt = to_sim(m.timestamp.values)
        sel = (mt >= t0) & (mt <= min(t1, t0 + 8.0))
        u2 = m.u2.values[sel]
        st = mt[sel]
        dt = np.median(np.diff(st))
        k = max(3, int(round(1.0 / dt)))
        trend = np.convolve(u2, np.ones(k) / k, mode='same')
        r = u2 - trend
        f, P = welch(r, fs=1.0 / dt, nperseg=min(128, len(r)))
        fdom = float(f[1:][np.argmax(P[1:])])
        drops = [a[1] - b[1] for a, b in zip(tp[:-1], tp[1:]) if b[1] < a[1]]
        peaks = [a[0] for a, b, c in zip(tp[:-2], tp[1:-1], tp[2:]) if b[1] > a[1] and b[1] > c[1]]
        early = sel & (mt >= t0 + 0.5) & (mt <= t0 + 4.0)
        res_extra = {'largest_height_drop_m': round(max(drops), 2) if drops else 0.0,
                     'bounce_period_s': round(float(peaks[1] - peaks[0]), 2) if len(peaks) >= 2 else None,
                     'throttle_pp_first_3_5s': round(float(np.ptp(m.u2.values[early])), 3)}
        res[os.path.basename(run)[:5]] = {**res_extra,
            'his_takeoff': t0, 'reached_approach': reached, 'approach_at': t1 if reached else None,
            'max_tilt_deg': round(tilt, 1), 'max_tilt_at': round(t_tilt, 2),
            'z_turning_points': [(round(a, 2), round(b, 2)) for a, b in tp[:8]],
            'throttle_pp': round(float(u2.max() - u2.min()), 3), 'throttle_detrended_std': round(float(r.std()), 4),
            'throttle_f_dom_hz': round(fdom, 2), 'throttle_window_s': round(float(st[-1] - st[0]), 1)}
        print(os.path.basename(run)[:5], json.dumps(res[os.path.basename(run)[:5]]))
    return res


if __name__ == '__main__':
    main(sys.argv[1:])

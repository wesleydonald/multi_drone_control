#!/usr/bin/env python3
"""tools/l1_replay.py -- offline shadow replay of the option-C L1 law on logged runs (card
2026-10-03_l1_ring, critic must-fix 1). No runs, nothing applied (u_ad = 0).

Per axis, on the tracking error e = p - r (reference velocity and acceleration fed through, so a
well-tracked trajectory reads sigma = 0):

    m e'' = -k e - c e' + k u_ad + sigma

Predictor x_hat = [e_hat, e_hat'] with gain A_s = -a I; piecewise-constant adaptation at the tick
T; the matched force sigma = m * (velocity component of -Phi^-1 e^(A_s T) x_tilde); the L1 filter
C(s) = w / (s + w) on sigma. Printed: sigma's spread over the lap or hold, and its step.

    python3 tools/l1_replay.py R0832 --window lap      # the critic's 0.2 N p-p bar
    python3 tools/l1_replay.py R0856 --window push     # within 15 % of -1 N
"""
import argparse
import csv
import glob
import os

import numpy as np
import pandas as pd

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
XY = dict(k=5.41, c=4.71, m=1.20)        # push fit R0856 (card identification table)


def smooth_diff(t, y, w=0.5):
    dt = float(np.median(np.diff(t)))
    k = max(1, int(round(w / dt)))
    ker = np.ones(k) / k
    ys = np.convolve(y, ker, mode='same')
    return ys, np.gradient(ys, t)


def l1_sigma(t, e, ed, k, c, m, a=10.0, T=0.1, w=2.0):
    """sigma_hat (raw) and C(s) sigma_hat, sampled every T from the logged error."""
    A = np.array([[0.0, 1.0], [-k / m, -c / m]])
    ts = np.arange(t[0], t[-1], T)
    E, ED = np.interp(ts, t, e), np.interp(ts, t, ed)
    gain = a * np.exp(-a * T) / (1.0 - np.exp(-a * T))
    xh = np.array([E[0], ED[0]])
    raw, filt, f = [], [], 0.0
    for j in range(len(ts)):
        x = np.array([E[j], ED[j]])
        xt = xh - x
        sig_vec = -gain * xt                      # -Phi^-1 e^(A_s T) x_tilde, A_s = -a I
        sig = m * sig_vec[1]
        f += (T * w / (1.0 + T * w)) * (sig - f)
        raw.append(sig)
        filt.append(f)
        # predictor over one tick on the MEASURED state (x_hat_dot = A x + sigma_hat + A_s x_tilde,
        # as L1 piecewise-constant laws state it), u_ad = 0
        for _ in range(10):
            xh = xh + (T / 10) * (A @ x + sig_vec - a * (xh - x))
    return ts, np.array(raw), np.array(filt)


def load(rid):
    d = glob.glob(os.path.join(REPO, 'results', '*', f'{rid}_*'))[0]
    ev = list(csv.DictReader(open(os.path.join(d, 'logs', 'events.csv'))))
    return d, ev, pd.read_csv(os.path.join(d, 'logs', 'run.csv'))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('run')
    ap.add_argument('--window', choices=['lap', 'push', 'hold'], required=True)
    ap.add_argument('--axis', default='x')
    ap.add_argument('--a', type=float, default=10.0)
    ap.add_argument('--w', type=float, default=2.0)
    a = ap.parse_args()
    d, ev, run = load(a.run)
    run = run.drop_duplicates('t').sort_values('t')
    run = run.dropna(subset=[f'payload_{a.axis}', f'payload_ref_{a.axis}'])
    t = run.t.values
    p, pd_ = smooth_diff(t, run[f'payload_{a.axis}'].values)
    r, rd = smooth_diff(t, run[f'payload_ref_{a.axis}'].values)
    ts, raw, filt = l1_sigma(t, p - r, pd_ - rd, **XY, a=a.a, w=a.w)
    if a.window == 'push':
        wr = [float(e['sim_time']) for e in ev if e['event'] == 'WRENCH']
        on, off = min(wr), max(wr)
        pre = filt[(ts >= on - 5) & (ts < on)]
        pu = filt[(ts >= off - 5) & (ts < off)]
        print(f'{a.run} {a.axis}: sigma pre {pre.mean():+.3f} N, push {pu.mean():+.3f} N, '
              f'step {pu.mean() - pre.mean():+.3f} N (push p-p {np.ptp(pu):.3f} N)')
    else:
        land = [float(e['sim_time']) for e in ev if e['event'] == 'LAND'][0]
        lifted = [float(e['sim_time']) for e in ev if e['event'] == 'LIFTED'][0]
        sel = (ts >= lifted + 12) & (ts < land - 1)
        s = filt[sel]
        print(f'{a.run} {a.axis} {a.window}: sigma mean {s.mean():+.3f} N, p-p {np.ptp(s):.3f} N, '
              f'sd {s.std():.3f} N over {sel.sum()} ticks (raw p-p {np.ptp(raw[sel]):.3f} N)')


if __name__ == '__main__':
    main()

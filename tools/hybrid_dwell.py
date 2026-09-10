#!/usr/bin/env python3
"""
hybrid_dwell.py -- attach / detach as a hybrid system: measure the jump and the decay.

docs/experimentation/control_methods_survey.md R4. The stack already treats a weld or a
detach as a JUMP (bumpless datum shift + tension blend = the reset map) followed by a
HOLD (dwell time) before the trajectory resumes. This reads what the runs actually did
and reports, per event:

    tilt_pre      steady load tilt before the event (mean over the pre-window)
    tilt_peak     peak tilt after the event, and when
    jump          tilt_peak - tilt_pre                  (the Lyapunov-jump size)
    tilt_ss       steady tilt the system settles to
    lambda        decay rate of |tilt - tilt_ss| after the peak, from a log-linear fit
    tau           1/lambda
    T_dwell(eps)  minimum dwell for the excess to fall under eps deg: ln(jump/eps)/lambda
    rho(T)        per-event contraction of the excess over the hold actually used:
                  exp(-lambda*T_hold); < 1 means each event shrinks the excess it inherited

A conditional ISS-style reading (arXiv 2605.05339): if every event's jump is bounded and
the inter-event dwell exceeds T_dwell, the tilt excess contracts by rho per cycle and the
sequence of events is stable. The numbers here are the measured inputs to that argument.

    tools/hybrid_dwell.py results_archive/R0209_* results/2026-09-10/R0234_* ...
    tools/hybrid_dwell.py --eps 3 --md docs/experimentation/hybrid_dwell.md <runs>
"""
import argparse
import csv
import math
import pathlib
import sys

import numpy as np

EVENTS = ('WELD', 'DETACH')


def load(run_dir):
    run_dir = pathlib.Path(run_dir)
    rows = list(csv.DictReader(open(run_dir / 'logs' / 'run.csv')))
    t = np.array([float(r['t']) for r in rows])
    tilt = np.array([float(r['payload_tilt_deg']) for r in rows])
    ev = [(float(r['sim_time']), r['event'], r.get('arg', ''))
          for r in csv.DictReader(open(run_dir / 'logs' / 'events.csv'))]
    return t, tilt, ev


def fit_decay(t, x, t_peak, t_end, tilt_ss):
    """Log-linear fit of |x - x_ss| between the peak and t_end. Returns (lambda, r2)."""
    m = (t >= t_peak) & (t <= t_end)
    y = np.abs(x[m] - tilt_ss)
    keep = y > 0.5                                   # below 0.5 deg is noise
    if keep.sum() < 10:
        return float('nan'), float('nan')
    tt, ly = t[m][keep] - t_peak, np.log(y[keep])
    A = np.vstack([tt, np.ones_like(tt)]).T
    (slope, _), res, _, _ = np.linalg.lstsq(A, ly, rcond=None)
    ss_tot = float(np.sum((ly - ly.mean()) ** 2))
    r2 = 1.0 - float(res[0]) / ss_tot if len(res) and ss_tot > 0 else float('nan')
    return -float(slope), r2


def analyse(run_dir, eps, pre_s=6.0, settle_s=25.0, hold_s=10.0):
    t, tilt, ev = load(run_dir)
    out = []
    times = [e[0] for e in ev]
    for k, (te, name, arg) in enumerate(ev):
        if name not in EVENTS:
            continue
        t_next = min([x for x in times if x > te] + [t[-1]])
        pre = (t >= te - pre_s) & (t < te)
        post = (t >= te) & (t <= min(te + settle_s, t_next))
        if pre.sum() < 5 or post.sum() < 20:
            continue
        tilt_pre = float(tilt[pre].mean())
        i_pk = int(np.argmax(tilt[post]))
        t_pk, tilt_pk = float(t[post][i_pk]), float(tilt[post][i_pk])
        tail = (t >= min(te + settle_s, t_next) - 5.0) & (t <= min(te + settle_s, t_next))
        tilt_ss = float(tilt[tail].mean()) if tail.sum() else float('nan')
        lam, r2 = fit_decay(t, tilt, t_pk, min(te + settle_s, t_next), tilt_ss)
        jump = tilt_pk - tilt_pre
        excess = max(tilt_pk - tilt_ss, 1e-6)
        t_dwell = math.log(excess / eps) / lam if lam > 0 and excess > eps else 0.0
        rho = math.exp(-lam * hold_s) if lam > 0 else float('nan')
        out.append(dict(run=pathlib.Path(run_dir).name, event=f'{name} {arg}'.strip(),
                        t_event=te, tilt_pre=tilt_pre, tilt_peak=tilt_pk,
                        t_peak_after=t_pk - te, jump=jump, tilt_ss=tilt_ss,
                        lam=lam, tau=(1.0 / lam if lam > 0 else float('nan')), r2=r2,
                        t_dwell=t_dwell, rho=rho))
    return out


def fmt(rows, eps, hold_s):
    hdr = (f"{'run':34s} {'event':9s} {'pre':>5s} {'peak':>5s} {'@+s':>5s} {'jump':>5s} "
           f"{'ss':>5s} {'lambda':>7s} {'tau':>5s} {'r2':>5s} {'Tdwell':>7s} {'rho':>5s}")
    lines = [hdr, '-' * len(hdr)]
    for r in rows:
        lines.append(f"{r['run'][:34]:34s} {r['event']:9s} {r['tilt_pre']:5.1f} "
                     f"{r['tilt_peak']:5.1f} {r['t_peak_after']:5.1f} {r['jump']:5.1f} "
                     f"{r['tilt_ss']:5.1f} {r['lam']:7.3f} {r['tau']:5.1f} {r['r2']:5.2f} "
                     f"{r['t_dwell']:7.1f} {r['rho']:5.2f}")
    lines.append(f"(deg, s; eps = {eps} deg excess; rho over the {hold_s:.0f} s hold actually used)")
    return '\n'.join(lines)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('runs', nargs='+')
    ap.add_argument('--eps', type=float, default=3.0, help='allowed excess tilt (deg)')
    ap.add_argument('--hold', type=float, default=10.0, help='hold used (attach_traj_hold_s)')
    ap.add_argument('--md', help='also write a markdown table here')
    a = ap.parse_args()
    rows = []
    for r in a.runs:
        try:
            rows += analyse(r, a.eps, hold_s=a.hold)
        except FileNotFoundError as e:
            print(f'skip {r}: {e}', file=sys.stderr)
    print(fmt(rows, a.eps, a.hold))
    if a.md:
        md = ['| run | event | tilt pre | peak (at +s) | jump | settled | λ (1/s) | τ (s) | r² | T_dwell(ε) | ρ(hold) |',
              '|---|---|---|---|---|---|---|---|---|---|---|']
        for r in rows:
            md.append(f"| {r['run']} | {r['event']} | {r['tilt_pre']:.1f}° | {r['tilt_peak']:.1f}° (+{r['t_peak_after']:.1f}) | "
                      f"{r['jump']:.1f}° | {r['tilt_ss']:.1f}° | {r['lam']:.3f} | {r['tau']:.1f} | {r['r2']:.2f} | "
                      f"{r['t_dwell']:.1f} s | {r['rho']:.2f} |")
        pathlib.Path(a.md).write_text('\n'.join(md) + '\n')
        print('wrote', a.md)


if __name__ == '__main__':
    main()

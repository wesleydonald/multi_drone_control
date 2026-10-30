#!/usr/bin/env python3
"""S claim: eight carriers to four and back (card docs/experiments/2026-10-23_scalability.md, N; cell chain4back).

Message: eight carriers lose four of their number one at a time, unannounced, take the four back one at a time, and
the ring stays level and close to its hover point throughout.
Hero: the ring's tilt and its distance from the hover point (run.csv, as recorded), from the top of the lift to LAND;
above them, the team size the planner flies (its log, moved onto the run clock by the manifest's t_ready_sim, as
tools/s_metrics.py does). Events from events.csv: RELEASE (thin lines), REJOIN_WELD to REJOIN_BLEND_OVER (the fold-in,
light bands). Left out: the lift and the landing, the rejoiners' approach paths, the bars (in the thesis table), the
repeat (alike to within 0.3 deg and 3 mm at every event; drawn in fig_s8_events).
Data: R2766 (s8_chain4back_R50_L100_it5, lab batch 18, week7 63b504f, 5 SQP iterations, SIM_MOCAP_VEL_FILTER ema);
its repeat R2767.
"""
import csv
import glob
import json
import os
import sys

import numpy as np
import pandas as pd

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from review30_style import S, TEXT  # noqa: E402

RUN = 'R2766'


def load(rid):
    d = glob.glob(os.path.join(S.REPO, 'results', '*', f'{rid}_*'))[0]
    run = pd.read_csv(os.path.join(d, 'logs', 'run.csv')).drop_duplicates('t').dropna(subset=['payload_qw'])
    ev = [(float(e['sim_time']), e['event'], e['arg']) for e in csv.DictReader(open(os.path.join(d, 'logs', 'events.csv')))]
    t_ready = float(json.load(open(os.path.join(d, 'manifest.json')))['t_ready_sim'])
    log = pd.read_csv(glob.glob(os.path.join(d, 'logs', 'dissipative_planner', '*', 'log.csv'))[0],
                      usecols=['sim_time', 'n'])
    t = run.t.to_numpy(float)
    z_ref = run.payload_ref_z.to_numpy(float)
    t_top = float(t[np.argmax(z_ref >= z_ref.max() - 1e-4)])
    t_land = next(te for te, name, _ in ev if name == 'LAND')
    err = np.linalg.norm(run[['payload_x', 'payload_y', 'payload_z']].to_numpy(float)
                         - run[['payload_ref_x', 'payload_ref_y', 'payload_ref_z']].to_numpy(float), axis=1)
    return dict(t=t, tilt=run.payload_tilt_deg.to_numpy(float), err=err, ev=ev, t_top=t_top, t_land=t_land,
                tn=log.sim_time.to_numpy(float) - t_ready, n=log.n.to_numpy(int))


def main():
    r = load(RUN)
    t0, t1 = r['t_top'], r['t_land']
    w = (r['t'] >= t0) & (r['t'] <= t1)
    t = r['t'][w] - t0
    wn = (r['tn'] >= t0) & (r['tn'] <= t1)

    S.setup()
    fig = S.figure('full', 3.2)
    gs = fig.add_gridspec(3, 1, height_ratios=[0.7, 1.35, 1.35])
    ax_n = fig.add_subplot(gs[0])
    ax_th = fig.add_subplot(gs[1], sharex=ax_n)
    ax_e = fig.add_subplot(gs[2], sharex=ax_n)
    axes = (ax_n, ax_th, ax_e)

    rel = [te - t0 for te, name, _ in r['ev'] if name == 'RELEASE' and t0 <= te <= t1]
    weld = [te - t0 for te, name, _ in r['ev'] if name == 'REJOIN_WELD' and t0 <= te <= t1]
    over = [te - t0 for te, name, _ in r['ev'] if name == 'REJOIN_BLEND_OVER' and t0 <= te <= t1]
    for a in axes:
        for x in rel:
            a.axvline(x, color=S.LIGHT, lw=0.5, zorder=0)
        for x0, x1 in zip(weld, over):
            a.axvspan(x0, x1, color=S.FILL, lw=0, zorder=0)

    ax_n.step(r['tn'][wn] - t0, r['n'][wn], where='post', color=S.BLUE, lw=S.LW_HERO)
    ax_n.set_ylim(3.4, 8.6)
    ax_n.set_yticks([4, 6, 8])
    ax_n.set_ylabel('Carriers')
    for xs, text in ((rel, 'releases'), (weld + over, 'rejoins')):
        ax_n.annotate(text, xy=(0.5 * (min(xs) + max(xs)), 1.0), xycoords=('data', 'axes fraction'), xytext=(0, 3),
                      textcoords='offset points', ha='center', va='bottom', fontsize=8, color=TEXT)

    ax_th.plot(t, r['tilt'][w], color=S.BLUE, lw=S.LW_CTX, rasterized=True)
    ax_th.set_ylim(0, 4)
    ax_th.set_yticks([0, 2, 4])
    ax_th.set_ylabel(r'Ring tilt ($^\circ$)')

    ax_e.plot(t, 1e3 * r['err'][w], color=S.BLUE, lw=S.LW_CTX, rasterized=True)
    ax_e.set_ylim(0, 100)
    ax_e.set_yticks([0, 50, 100])
    ax_e.set_ylabel('Ring offset (mm)')
    ax_e.set_xlabel(r'$t$ after the lift (s)')
    ax_e.set_xlim(0, t1 - t0)

    for ax, y, fmt in ((ax_th, r['tilt'][w], r'peak {:.1f}$^\circ$'), (ax_e, 1e3 * r['err'][w], 'peak {:.0f} mm')):
        k = int(np.nanargmax(y))
        ax.annotate(fmt.format(y[k]), xy=(t[k], y[k]), xytext=(4, 0), textcoords='offset points', ha='left',
                    va='center', fontsize=8, color=TEXT)
        print(f'{ax.get_ylabel()!r}: peak {y[k]:.2f} at {t[k]:.1f} s')
    print(f'n {sorted(set(r["n"][wn].tolist()))}; window {t1 - t0:.1f} s; releases {np.round(rel, 1)}; '
          f'welds {np.round(weld, 1)}; blends over {np.round(over, 1)}')

    for a in (ax_n, ax_th):
        a.tick_params(labelbottom=False)
    fig.align_ylabels(axes)
    for a, letter in zip(axes, 'abc'):
        S.ps.panel_label(a, letter)
    S.save(fig, 'fig_s8_chain')


if __name__ == '__main__':
    main()

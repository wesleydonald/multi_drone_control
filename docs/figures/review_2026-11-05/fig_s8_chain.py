#!/usr/bin/env python3
"""5 Nov copy of review_2026-10-30/fig_s8_chain.py: the thesis' words in the figure (quadrotors, rejoin, distance from the hover point).

S claim: eight carriers to four and back (card docs/experiments/2026-10-23_scalability.md, N; cell chain4back).

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

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'review_2026-10-30'))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))     # this folder's fig_s8_chain first
from review30_style import S, TEXT  # noqa: E402
S.HERE = os.path.dirname(os.path.abspath(__file__))     # save beside this 5 Nov copy

VAR = os.environ.get('FIG24_VARIANT', 'A')  # Wesley 14 Nov: options A (no events, his pick), B (markers), C (B + blending)
RUN = 'R3972'     # 7 Nov: the final chain on the earlier masses (F065/F066); was R3805, R2766


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
    gs = fig.add_gridspec(3, 1, height_ratios=[0.85, 1.28, 1.28] if not VAR else [1.0, 1.2, 1.2])
    ax_n = fig.add_subplot(gs[0])
    ax_th = fig.add_subplot(gs[1], sharex=ax_n)
    ax_e = fig.add_subplot(gs[2], sharex=ax_n)
    axes = (ax_n, ax_th, ax_e)

    rel = [te - t0 for te, name, _ in r['ev'] if name == 'RELEASE' and t0 <= te <= t1]
    weld = [te - t0 for te, name, _ in r['ev'] if name == 'REJOIN_WELD' and t0 <= te <= t1]
    over = [te - t0 for te, name, _ in r['ev'] if name == 'REJOIN_BLEND_OVER' and t0 <= te <= t1]
    N_LIM = (3.4, 10.2) if not VAR else (3.4, 8.6)    # today's: headroom over 8 for the two group names
    top = {ax_n: (8.6 - N_LIM[0]) / (N_LIM[1] - N_LIM[0]), ax_th: 1.0, ax_e: 1.0}
    for a in axes if not VAR else ():
        for x in rel:
            a.axvline(x, ymax=top[a], color=S.LIGHT, lw=0.5, zorder=0)
        for x0, x1 in zip(weld, over):
            a.axvspan(x0, x1, ymax=top[a], color=S.FILL, lw=0, zorder=0)

    ax_n.step(r['tn'][wn] - t0, r['n'][wn], where='post', color=S.BLUE, lw=S.LW_HERO)
    ax_n.set_ylim(*N_LIM)
    ax_n.set_yticks([4, 6, 8])
    ax_n.set_ylabel('No. of\nquadrotors')
    # Wesley 14 Nov: the panel titles sit above the axes, so the two group names go just inside the top edge
    for xs, text in ((rel, 'Detachments'), (weld + over, 'Reattachments')) if not VAR else ():
        ax_n.annotate(text, xy=(0.5 * (min(xs) + max(xs)), 1.0), xycoords=('data', 'axes fraction'), xytext=(0, -2),
                      textcoords='offset points', ha='center', va='top', fontsize=8, color=TEXT)

    ax_th.plot(t, r['tilt'][w], color=S.BLUE, lw=S.LW_CTX, rasterized=True)
    ax_th.set_ylim(0, 4)
    ax_th.set_yticks([0, 2, 4])
    ax_th.set_ylabel(r'Payload tilt ($^\circ$)')

    ax_e.plot(t, 1e3 * r['err'][w], color=S.BLUE, lw=S.LW_CTX, rasterized=True)
    ax_e.set_ylim(0, 100)
    ax_e.set_yticks([0, 50, 100])
    ax_e.set_ylabel('Distance from\nhover point (mm)')
    ax_e.set_xlabel('Time after takeoff (s)')
    ax_e.set_xlim(0, t1 - t0)

    for ax, y in ((ax_th, r['tilt'][w]), (ax_e, 1e3 * r['err'][w])):     # Wesley 14 Nov: the peak values are not drawn
        k = int(np.nanargmax(y))
        print(f'{ax.get_ylabel()!r}: peak {y[k]:.2f} at {t[k]:.1f} s')
    tn, nn = r['tn'][wn] - t0, r['n'][wn]
    n_at = lambda x: int(nn[np.searchsorted(tn, x, side='right') - 1])
    keys = []
    if VAR == 'C':           # the blending as a thick pale segment on the step, instead of grey bands
        for x0, x1 in zip(weld, over):
            ax_n.plot([x0, x1], [n_at(x1)] * 2, color=S.BLUE_LIGHT, lw=5.5, solid_capstyle='butt', zorder=1)
        keys.append((S.plt.Line2D([], [], color=S.BLUE_LIGHT, lw=5.5), 'Blending'))
    if VAR in ('B', 'C'):    # one marker per event on the step, instead of event lines and words
        ax_n.plot(rel, [n_at(x + 1.0) + 0.5 for x in rel], 'v', ms=4.5, color=S.INK, zorder=4, clip_on=False)
        ax_n.plot(weld, [n_at(x + 0.05) - 0.5 for x in weld], '^', ms=4.5, mfc='white', mec=S.INK, mew=0.9, zorder=4, clip_on=False)
        keys = [(S.plt.Line2D([], [], ls='', marker='v', ms=4.5, color=S.INK), 'Detachment'),
                (S.plt.Line2D([], [], ls='', marker='^', ms=4.5, mfc='white', mec=S.INK, mew=0.9), 'Reattachment')] + keys
    if keys:
        S.ps.legend_below(fig, [h for h, _ in keys], [n for _, n in keys], ncol=len(keys))
    print(f'n {sorted(set(r["n"][wn].tolist()))}; window {t1 - t0:.1f} s; releases {np.round(rel, 1)}; '
          f'welds {np.round(weld, 1)}; blends over {np.round(over, 1)}')

    for a in (ax_n, ax_th):
        a.tick_params(labelbottom=False)
    fig.align_ylabels(axes)
    for a, letter, name in zip(axes, 'abc', ('Quadrotors carrying', 'Payload tilt', 'Distance from the hover point')):
        S.panel(a, letter, name)
    S.save(fig, 'fig_s8_chain' + (f'_{VAR}' if VAR not in ('', 'A') else ''))


if __name__ == '__main__':
    main()

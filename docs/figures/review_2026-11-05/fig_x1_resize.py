#!/usr/bin/env python3
"""X1, what the resize is worth, on the measured mass distribution (9 Nov: Wesley, keep one distribution, the measured one;
7 Nov: both side by side, F089; was the 3 Nov runs R3520-R3563).

The one sentence: resizing at detection leaves the first swing after an unannounced release unchanged and adds a larger
second swing, but leaves the hovering ring more level once settled; in layout (e) the ring settles at the same tilt
with or without the resize.
Hero: (a) the ring's tilt after the release, four quadrotors in hover, 0.86 kg ring, both runs of each arm, the
detection marked. (b) the settled tilt per run in hover, resize at detection against none.
(c) the same for layout (e) (sideways integral off), whose scale is ten times larger.
Left out: the late arms (in the table), the figure-eight (the path sets its tilt: in the table), eight quadrotors,
the earlier mass distribution (Wesley 9 Nov: one distribution in the thesis), the 1.15 kg ring, the in-plot notes
(Wesley 9 Nov: layout (d) in hover goes after (a)).
Data: the measured split R3821-R3906 (results/analysis/x1_2026-11-06.json); settled tilt as tools/x1_metrics.py scores it (settled_tilt, the tilt of
the mean tilt vector over the 5 s before LAND), checked against results/analysis/x1_both_models_2026-11-07.md (F094).
Events from events.csv.
"""
import glob
import json
import os
import sys

import numpy as np
import pandas as pd

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import review_style as S  # noqa: E402

S.HERE = HERE
SPLITS = (('measured', os.path.join(S.REPO, 'results', 'analysis', 'x1_2026-11-06.json'), 'Measured split'),
          ('earlier', os.path.join(S.REPO, 'results', 'analysis', 'x1_oldlinks_2026-11-07.json'), 'Earlier split'))
ARMS = (('ours', S.BLUE, 'Proposed, resized at detection'), ('nors', S.RED, 'Baseline, not resized'))
DASH = {'ours': '-', 'nors': (0, (3.5, 1.5))}     # a second cue for greyscale print
WINDOW = (-2.0, 10.0)


def tint(colour, a):
    rgb = np.array(S.plt.matplotlib.colors.to_rgb(colour))
    return tuple(1.0 - a * (1.0 - rgb))


def run_dir(rid):
    (d,) = glob.glob(os.path.join(S.REPO, 'results', '2026-11-0*', f'{rid}_*'))
    return d


def tilt_after_release(rid):
    d = run_dir(rid)
    run = pd.read_csv(os.path.join(d, 'logs', 'run.csv'), usecols=['t', 'payload_tilt_deg']).drop_duplicates('t')
    ev = pd.read_csv(os.path.join(d, 'logs', 'events.csv'))
    ev = ev[pd.to_numeric(ev.sim_time, errors='coerce').notna()]
    t_rel = float(ev.loc[ev.event == 'RELEASE', 'sim_time'].astype(float).iloc[0])
    t_land = float(ev.loc[ev.event == 'LAND', 'sim_time'].astype(float).iloc[0])
    det = ev.loc[ev.event == 'DETACH_RESIZE', 'sim_time'].astype(float)
    t_det = float(det.iloc[0]) - t_rel if len(det) else None
    return run.t.to_numpy(float) - t_rel, run.payload_tilt_deg.to_numpy(float), t_land - t_rel, t_det


def settled(rows, cell, arm):
    return [r['settled_tilt'] for r in rows if r['cell'] == cell and r['arm'] == arm]


def bars(ax, rows_by_split, groups, ylim, fmt, nudge=0.0):
    """groups: [(cell, split)]; two bars each (resized, not), one dot per run, the mean printed above."""
    w = 0.36
    for i, (cell, split) in enumerate(groups):
        for j, (arm, col, _) in enumerate(ARMS):
            v = settled(rows_by_split[split], cell, arm)
            x = i + (j - 0.5) * w
            ax.bar(x, np.mean(v), w * 0.88, color=tint(col, 0.3) if arm == 'ours' else 'white', edgecolor=col, lw=0.8)
            ax.plot([x] * len(v), v, 'o', ms=3.0, color=col, mec='white', mew=0.4, zorder=3)
            ax.annotate(fmt.format(np.mean(v)), xy=(x, max(v)), xytext=((j - 0.5) * nudge, 2.5),
                        textcoords='offset points', ha='center', va='bottom', fontsize=7.5, color=col)
            print(cell, split, arm, [round(x_, 3) for x_ in v])
    ax.set_xticks(range(len(groups)))
    ax.set_xticklabels([dict((k, n) for k, _, n in SPLITS)[s] for _, s in groups])
    ax.set_xlim(-0.6, len(groups) - 0.4)
    ax.set_ylim(*ylim)
    ax.tick_params(axis='x', length=0)


def group_label(ax, x, text):
    ax.annotate(text, xy=(x, 0), xycoords=('data', 'axes fraction'), xytext=(0, -17), textcoords='offset points',
                ha='center', va='top', fontsize=8, color=S.INK)


def main():
    rows = {k: json.load(open(f)) for k, f, _ in SPLITS}
    S.setup()
    fig = S.figure('full', 3.7)
    gs = fig.add_gridspec(2, 2, height_ratios=[1.15, 1.0], width_ratios=[1.0, 1.0])
    a = fig.add_subplot(gs[0, :])
    b, c = fig.add_subplot(gs[1, 0]), fig.add_subplot(gs[1, 1])

    keys, t_det = [], None
    for arm, col, _ in ARMS:
        for k, r in enumerate(x for x in rows['measured'] if x['cell'] == 'h4' and x['arm'] == arm):
            t, th, land, td = tilt_after_release(r['run'])
            if arm == 'ours' and k == 0:
                t_det = r['resize_s']      # the planner log's exact clock (events.csv's line comes later)
            m = (t >= WINDOW[0]) & (t <= min(WINDOW[1], land))
            line, = a.plot(t[m], th[m], color=col, lw=S.LW_CTX, ls=DASH[arm])
            if k == 0:
                keys.append(line)
            sec = (t >= 1.0) & (t <= 4.0)
            print(r['run'], arm, f"rise {r['tilt_rise']:.2f}, second swing {th[sec].max():.2f} at "
                  f"{t[sec][np.argmax(th[sec])]:.2f} s, settled {r['settled_tilt']:.3f}, LAND +{land:.1f} s")
    a.axvline(0, color=S.LIGHT, lw=0.6, zorder=0)
    a.annotate('detachment', xy=(0, 1.0), xycoords=('data', 'axes fraction'), xytext=(-3, -3), textcoords='offset points',
               ha='right', va='top', fontsize=8, color=S.GREY)
    a.axvline(t_det, color=S.BLUE, lw=0.6, ls=(0, (3, 2)), zorder=0)
    a.annotate(f'resizing, {t_det:.2f}\\,s', xy=(t_det, 1.0), xycoords=('data', 'axes fraction'), xytext=(3, -3),
               textcoords='offset points', ha='left', va='top', fontsize=8, color=S.BLUE)
    a.set_xlim(*WINDOW)
    a.set_ylim(0, 7)
    a.set_yticks([0, 2, 4, 6])
    a.set_xlabel('Time after detach (s)')
    a.set_ylabel(r'Ring tilt ($^\circ$)')

    bars(b, rows, [('h4', 'measured')], (0, 0.3), '{:.2f}', nudge=5.0)
    b.set_xticks([])
    b.set_yticks([0, 0.1, 0.2, 0.3])
    b.set_yticklabels(['0', '0.1', '0.2', '0.3'])
    b.set_ylabel(r'Settled tilt ($^\circ$)')

    bars(c, rows, [('sqTnoxy', 'measured')], (0, 3.0), '{:.2f}', nudge=5.0)
    c.set_xticks([])
    c.set_yticks([0, 1, 2, 3])

    S.panel(a, 'a', 'layout (d) in hover')
    S.panel(b, 'b', 'layout (d) in hover')
    S.panel(c, 'c', 'layout (e) in hover')
    fig.align_ylabels((a, b))
    import matplotlib.patches as mp
    from matplotlib.legend_handler import HandlerTuple
    handles = [(keys[0], mp.Patch(facecolor=tint(S.BLUE, 0.3), edgecolor=S.BLUE, lw=0.8)),
               (keys[1], mp.Patch(facecolor='white', edgecolor=S.RED, lw=0.8))]
    S.ps.legend_below(fig, handles, [label for *_, label in ARMS], ncol=2,
                      handler_map={tuple: HandlerTuple(ndivide=None, pad=0.6)}, handlelength=3.2)
    S.save(fig, 'fig_x1_resize')


if __name__ == '__main__':
    main()

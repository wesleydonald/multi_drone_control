#!/usr/bin/env python3
"""Tracking accuracy split into its parts: the ring's sideways distance to the path, its height error, and apart from
both its lag on the schedule (Wesley 22 Oct: a lag is fine if the path is followed closely).

Two designs (Wesley picks; the first is recommended):
  fig_components         every chosen path flight: RMS sideways, RMS height (mm) and the mean lag (s), one row each,
                         in its fleet size's colour, rig filled, twin open. Message: on the rig the sideways part is the
                         larger on every path but the spinny line (13-25 mm against 6-26 mm of height), and the ring runs
                         0.05-0.23 s behind its schedule. The spin's parts (3: 40.5 / 18.5 mm, 0.27 s; 4: 70.3 / 49.0 mm)
                         are off this scale: in the index.
  fig_components_eight   one flight (three carriers, the figure-8; its fleet colour) along the eight: (a) signed sideways distance to the
                         path (+ inside the curve), (b) height error, (c) lag, against the ring's position along the
                         eight, the tight bends shaded. Message: where the error builds along one lap.
Numbers: best_data.path_parts (n2_bands' window; metrics.project_on_path / contour_lag). The lag is drawn only where
contour_lag counts it (schedule faster than half its peak, outside clock holds).
"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import best_style as S  # noqa: E402
import best_data as B  # noqa: E402
import metrics as M  # noqa: E402

ROWS = [('figure-8', 2), ('figure-8', 3), ('figure-8', 4), ('circle', 3), ('circle', 4), ('line', 3),
        ('spinny line', 2), ('spinny line', 3), ('spinny line', 4)]


def along_eight(key):
    f = B.Flight(key)
    s = B.path_parts(f)
    d = f.run()
    t = d.t.to_numpy()
    des = d[['payload_ref_x', 'payload_ref_y']].to_numpy()
    c = M.contour_lag(t, d[['payload_x', 'payload_y']].to_numpy(), des, mask=s['w'], full=True)
    w = s['w']
    pos = s['s_at'] - s['s_at'][np.argmax(w)]
    path = s['path']
    tight = np.abs(path[4]) >= np.quantile(np.abs(path[4]), 0.75)
    return dict(pos=pos, w=w, across=s['across'], dz=s['dz'], lag=np.where(c['use'], c['lag'], np.nan),
                s_path=path[1], tight=tight, L=float(path[1][-1]), s=s)


def spans(x, m):
    """(start, end) of the runs of True in m along x."""
    m = np.r_[False, m, False]
    i = np.flatnonzero(np.diff(m.astype(int)))
    return [(x[a], x[min(b, len(x)) - 1]) for a, b in zip(i[::2], i[1::2])]


def design_profile():
    S.setup()
    fig = S.figure('full', 3.2)
    axs = fig.subplots(3, 1, sharex=True)
    data = {3: along_eight(B.RIG_BEST[('figure-8', 3)])}
    ref = data[3]
    for ax in axs:
        for a, b in spans(ref['s_path'], ref['tight']):
            ax.axvspan(a, b, color=S.FILL, lw=0, zorder=0)
        ax.axhline(0, color=S.LIGHT, lw=0.5, zorder=0)
    for n, r in data.items():
        r['w'] = r['w'] & (r['pos'] <= r['L'] - 0.1)    # the last 10 cm: the ring is projected back across the crossing
        o = np.argsort(r['pos'][r['w']])
        x = r['pos'][r['w']][o]
        kw = dict(color=S.FLEET_COLOUR[n], lw=S.LW_CTX)
        axs[0].plot(x, 1000 * r['across'][r['w']][o], **kw)
        axs[1].plot(x, 1000 * r['dz'][r['w']][o], **kw)
        lag = r['lag'][r['w']][o]
        axs[2].plot(x, np.where(np.isfinite(lag), lag, np.nan), **kw)
        print(n, f"sideways RMS {1000 * r['s']['across_rms']:.1f} mm, height RMS {1000 * r['s']['dz_rms']:.1f} mm, "
              f"lag {r['s']['lag_s']:+.2f} s, eight {r['L']:.2f} m")
    axs[0].annotate('bends', xy=(np.mean(spans(ref['s_path'], ref['tight'])[0]), 1), xycoords=('data', 'axes fraction'),
                    xytext=(0, 1), textcoords='offset points', ha='center', va='bottom', fontsize=8, color=S.GREY)
    axs[0].set_ylabel('Sideways (mm)')
    axs[1].set_ylabel('Height (mm)')
    axs[2].set_ylabel('Lag (s)')
    axs[0].set_ylim(-45, 45)
    axs[1].set_ylim(-45, 45)
    axs[0].set_yticks([-40, 0, 40])
    axs[1].set_yticks([-40, 0, 40])
    axs[2].set_ylim(-0.1, 0.5)
    axs[2].set_yticks([0, 0.2, 0.4])
    axs[2].set_xlim(0, ref['L'])
    axs[2].set_xlabel('Ring position along the figure-8 (m)')
    fig.align_ylabels(axs)
    for ax, letter in zip(axs, 'abc'):
        S.ps.panel_label(ax, letter)
    S.g_ticks(*axs)
    S.save(fig, 'fig_components_eight')


def design_table():
    rows = []
    for m, n in ROWS:
        key = B.RIG_BEST.get((m, n)) or B.TWIN_FILL.get((m, n))
        f = B.Flight(key)
        s = B.path_parts(f)
        rows.append((f'{m}, {n}', f.source, 1000 * s['across_rms'], 1000 * s['dz_rms'], s['lag_s'], n))
        print(rows[-1], key)
    S.setup()
    fig = S.figure('full', 2.5)
    axs = fig.subplots(1, 3, sharey=True, gridspec_kw=dict(width_ratios=[1, 1, 0.8]))
    group = np.cumsum([0] + [int(ROWS[i][0] != ROWS[i - 1][0]) for i in range(1, len(ROWS))])
    y = -(np.arange(len(rows)) + 0.6 * group)
    for i, (label, src, side, z, lag, n) in enumerate(rows):
        filled = src == 'rig'
        c = S.FLEET_COLOUR[n]
        for ax, v in zip(axs, (side, z, lag)):
            if math.isfinite(v):
                ax.plot(v, y[i], 'o', ms=S.MS, color=c, mfc=c if filled else 'white', mew=0.9)
    axs[0].set_yticks(y, [r[0] for r in rows])
    for t, r in zip(axs[0].get_yticklabels(), rows):
        t.set_color(S.FLEET_COLOUR[r[5]])
    axs[0].set_xlabel('Sideways RMS (mm)')
    axs[1].set_xlabel('Height RMS (mm)')
    axs[2].set_xlabel('Lag (s)')
    top = 10 * math.ceil(max(max(r[2], r[3]) for r in rows) / 10 + 0.05)
    for ax in axs[:2]:
        ax.set_xlim(0, top)
        ax.set_xticks(range(0, top, 10))           # no label at the panel's right edge, next to the next panel's 0
    axs[2].set_xlim(0, 0.3)
    axs[2].set_xticks([0, 0.1, 0.2])
    for ax in axs:
        ax.tick_params(axis='y', length=0)
        for yy in y:
            ax.axhline(yy, color=S.FILL, lw=0.5, zorder=0)
    axs[2].annotate('open: twin', xy=(1, 1), xycoords='axes fraction', xytext=(0, 4), textcoords='offset points',
                    ha='right', va='bottom', fontsize=8, color=S.GREY)
    S.g_ticks(*axs, which='x')
    S.save(fig, 'fig_components')


if __name__ == '__main__':
    design_table()
    design_profile()

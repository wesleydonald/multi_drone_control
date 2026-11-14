#!/usr/bin/env python3
"""The figure-eight after an attachment with an error in the attachment point (4.1.3; Wesley 11 Nov: show the actual
figure-eight paths, the desired path against the actual one, every offset coloured by its size,
side by side; 13 Nov: one purple light to dark, since red means the plate's centre in Figure 17, fewer words).

The one sentence: with the measured catch point the ring follows the figure-eight at every offset, while with the plate's
centre an inward offset of 6 cm or more takes it far off the path.
Hero: 2 x 2 plan views. Rows: the attachment point at the measured catch point, at the plate's centre. Columns: the
offset along the rim (0-16 cm), radially inward (0-20 cm). Each panel: the reference (black, dashed) and the ring's path
over the one figure-eight flown after the attachment (PATH_START to LOGGED), one line per offset coloured by its size on
one purple scale, with a colour bar. Column names on the top row only, row names in the y labels.
Left out: the second repeat of each case (the first run of each pair is drawn; the printed RMS covers both), the hover
and the attachment before the path, the landing.
Data: R4034-R4093 (results/analysis/x11_u_2026-11-07.json, F078/F079: arm, direction, d_cm per run); the paths from
results/analysis/x11_u_paths_2026-11-07.csv.gz (Q012), else run.csv (payload_x/y, payload_ref_x/y) between events.csv's
PATH_START and LOGGED. Distance to the path: each sample's distance to the
reference polyline (clock-free).
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
ROWS = (('measured', 'measured catch point'), ('slot', "plate's centre"))
COLS = (('t', 'along the rim'), ('ri', 'inward'))
VMAX = 20.0


def run_dir(rid):
    (d,) = glob.glob(os.path.join(S.REPO, 'results', '2026-11-0*', f'{rid}_*'))
    return d


PATHS = os.path.join(S.REPO, 'results', 'analysis', 'x11_u_paths_2026-11-07.csv.gz')   # Q012: the runs left the laptop
_paths = None


def path(rid):
    global _paths
    if os.path.exists(PATHS):
        _paths = _paths if _paths is not None else pd.read_csv(PATHS)
        c = _paths[_paths.run == rid]
        return c.payload_x.to_numpy(), c.payload_y.to_numpy(), c.payload_ref_x.to_numpy(), c.payload_ref_y.to_numpy()
    d = run_dir(rid)
    c = pd.read_csv(os.path.join(d, 'logs', 'run.csv'),
                    usecols=['t', 'payload_x', 'payload_y', 'payload_ref_x', 'payload_ref_y']).drop_duplicates('t')
    ev = pd.read_csv(os.path.join(d, 'logs', 'events.csv'))
    ev = ev[pd.to_numeric(ev.sim_time, errors='coerce').notna()]
    g = lambda k: float(ev.loc[ev.event == k, 'sim_time'].astype(float).iloc[0])  # noqa: E731
    m = (c.t >= g('PATH_START')) & (c.t <= g('LOGGED'))
    return c.payload_x[m].to_numpy(), c.payload_y[m].to_numpy(), c.payload_ref_x[m].to_numpy(), c.payload_ref_y[m].to_numpy()


def dist_to_polyline(px, py, rx, ry):
    a = np.c_[rx[:-1], ry[:-1]]
    b = np.c_[rx[1:], ry[1:]]
    ab = b - a
    L2 = np.maximum((ab ** 2).sum(1), 1e-12)
    out = np.empty(len(px))
    for i, p in enumerate(np.c_[px, py]):
        s = np.clip(((p - a) * ab).sum(1) / L2, 0, 1)
        out[i] = np.min(np.linalg.norm(a + s[:, None] * ab - p, axis=1))
    return out


def main():
    rows = json.load(open(os.path.join(S.REPO, 'results', 'analysis', 'x11_u_2026-11-07.json')))['runs']
    flown = sorted({float(r['d_cm']) for r in rows})                 # the offsets actually flown
    # one purple, light to dark (13 Nov review: red stays 'the plate's centre' as in Figure 17; reads in greyscale)
    cmap_c = S.plt.matplotlib.colors.LinearSegmentedColormap.from_list('offset', ['#cbc3e3', '#6a51a3', '#2d004b'])
    cmap = S.plt.matplotlib.colors.ListedColormap([cmap_c(k / (len(flown) - 1)) for k in range(len(flown))])
    edges = [flown[0] - 1] + [0.5 * (a + b) for a, b in zip(flown[:-1], flown[1:])] + [flown[-1] + 1]
    norm = S.plt.matplotlib.colors.BoundaryNorm(edges, cmap.N)
    S.setup()
    fig = S.figure('full', 3.9)
    fig.get_layout_engine().set(h_pad=0.09)              # room above the top titles (annotations are not laid out)
    axs = fig.subplots(2, 2, sharex=True, sharey=True)                 # one scale: the panels are compared by eye
    ytop = [0.5]
    for i, (arm, arm_name) in enumerate(ROWS):
        for j, (direction, dir_name) in enumerate(COLS):
            ax = axs[i, j]
            cases = sorted({r['d_cm'] for r in rows if r['arm'] == arm and r['direction'] in (direction, 'd0')})
            ref = None
            for d_cm in cases:
                rr = sorted((r for r in rows if r['arm'] == arm and r['d_cm'] == d_cm
                             and r['direction'] in ((direction,) if d_cm else ('d0',))), key=lambda r: r['run'])
                rms = []
                for k, r in enumerate(rr):
                    px, py, rx, ry = path(r['run'])
                    dd = dist_to_polyline(px, py, rx, ry)
                    rms.append(1e3 * float(np.sqrt(np.mean(dd ** 2))))
                    if k == 0:
                        ax.plot(px, py, color=cmap(norm(d_cm)), lw=0.8, zorder=2 + d_cm / 100)
                        ytop[0] = max(ytop[0], float(np.max(py)))
                        ref = ref if ref is not None else (rx, ry)
                print(f'{arm:8s} {direction:2s} {d_cm:5.1f} cm  runs {[r["run"] for r in rr]}  path RMS '
                      f'{", ".join(f"{x:.1f}" for x in rms)} mm')
            ax.plot(*ref, color=S.INK, lw=0.7, ls=(0, (3, 2)), zorder=5)
            ax.set_aspect('equal')
            S.panel(ax, 'abcd'[2 * i + j], f"{arm_name[0].upper() + arm_name[1:]}, {dir_name}")   # Wesley 14 Nov: titles by the letters
    for ax in axs[1, :]:
        ax.set_xlabel('$x$ (m)')
    for ax in axs[:, 0]:
        ax.set_ylabel('$y$ (m)')
    axs[0, 0].set_xlim(-1.35, 1.35)                  # equal aspect at this height: the panels fill the width
    axs[0, 0].set_ylim(-0.6, ytop[0] + 0.08)
    axs[0, 0].set_xticks([-1, 0, 1])
    axs[0, 0].set_yticks([-0.5, 0, 0.5])
    for ax in axs[:, 0]:
        ax.set_yticklabels(['$-0.5$', '$0$', '$0.5$'])   # no trailing zero on 0
    print(f'highest path y {ytop[0]:.2f} m')
    sm = S.plt.matplotlib.cm.ScalarMappable(norm=norm, cmap=cmap)
    cb = fig.colorbar(sm, ax=axs, shrink=0.85, pad=0.02, aspect=28, spacing='uniform')
    cb.set_label('Attachment point error (cm)')
    cb.set_ticks([0.5 * (a + b) for a, b in zip(edges[:-1], edges[1:])])   # one label in the middle of each band
    cb.set_ticklabels([f'{v:g}' for v in flown])
    cb.ax.tick_params(length=0)
    ref_line, = axs[0, 0].plot([], [], color=S.INK, lw=0.8, ls=(0, (3, 2)))
    S.ps.legend_below(fig, [ref_line], ['Reference'], ncol=1)
    S.save(fig, 'fig_catch_paths')


if __name__ == '__main__':
    main()

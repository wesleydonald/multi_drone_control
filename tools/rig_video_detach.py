#!/usr/bin/env python3
"""tools/rig_video_detach.py -- a rig detach video beside live plots from its logs.

    python3 tools/rig_video_detach.py --video ~/Downloads/IMG_2019.MOV \\
        --logs results/rig/2026-10-07/r0012_logs/logs --v0 <epoch of the first frame> \\
        --duration 45.28 --out results/rig/2026-10-07/figures/r0012_detach_video.mp4

Panels: the ring's tilt, its desired and actual height, and its 3-D error from the desired
position (hover target xy, the planner's height target; lift-off to LAND), with lift-off, the
detach and LAND marked. --v0 syncs the video to the log clock (match a visible event: lift-off
or touchdown). Hover runs only: the desired xy is the captured hover point.
"""
import argparse
import glob
import os
import subprocess
import sys

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import plot_style as ps  # noqa: E402

W, H = 746, 1080
INK, GREY, LIGHT, BG = '#1E2633', '#6A7179', '#C9CED6', '#FBFBF8'


def load(logdir, v0):
    L = pd.read_csv(glob.glob(os.path.join(logdir, '*planner', '*', 'log.csv'))[0])
    L['t'] = L.sim_time - v0
    rest = L.load_z.iloc[:20].median()
    ev = {'lift-off': float(L.t[L.load_z > rest + 0.03].iloc[0])}
    k = np.flatnonzero(np.diff(L.n.values) != 0)
    if len(k):
        ev['detach'] = float(L.t.values[k[0] + 1])
    if (L.land != 0).any():
        ev['LAND'] = float(L.t[L.land != 0].iloc[0])
    # the planner captures the hover point from the first load pose (planner_node hover_xy)
    hx, hy = L.load_x.iloc[0], L.load_y.iloc[0]
    L['err_cm'] = 100.0 * np.sqrt((L.load_x - hx) ** 2 + (L.load_y - hy) ** 2 + (L.load_z - L.z_tgt) ** 2)
    L.loc[(L.t < ev['lift-off']) | (L.t > ev.get('LAND', np.inf)), 'err_cm'] = np.nan
    return L, ev


def at(df, col, t):
    return float(np.interp(t, df.t.values, df[col].values))


def style(ax):
    ax.set_facecolor(BG)
    for s in ('top', 'right'):
        ax.spines[s].set_visible(False)
    ax.tick_params(labelsize=13, colors=INK)


def build(L, ev, t_end):
    fig = plt.figure(figsize=(W / 100, H / 100), dpi=100, facecolor=BG)
    axt = fig.add_axes([0.15, 0.70, 0.78, 0.24])
    axz = fig.add_axes([0.15, 0.39, 0.78, 0.24])
    axe = fig.add_axes([0.15, 0.08, 0.78, 0.24])
    for a in (axt, axz, axe):
        style(a)
        a.set_xlim(0, t_end)
    gold = ps.PAYLOAD_LINE
    # ring tilt
    axt.plot(L.t, L.tilt_deg, color=LIGHT, lw=0.8)
    tl_line, = axt.plot([], [], color=gold, lw=2.0)
    axt.set_ylim(0, 15)
    axt.set_yticks([0, 5, 10, 15])
    axt.set_ylabel('Ring tilt (deg)', fontsize=14, color=INK)
    tl_txt = axt.text(0.99, 0.84, '', transform=axt.transAxes, ha='right', fontsize=14, color=INK)
    # desired vs actual height
    lift = L.t >= ev['lift-off'] - 1.0
    axz.plot(L.t[lift], L.z_tgt[lift], color=INK, lw=1.4, ls='--')
    axz.plot(L.t, L.load_z, color=LIGHT, lw=0.8)
    zl, = axz.plot([], [], color=gold, lw=2.0)
    axz.set_ylim(0, 1.3)
    axz.set_yticks([0, 0.5, 1.0])
    axz.set_ylabel('Ring height (m)', fontsize=14, color=INK)
    axz.plot([0.02, 0.09], [0.905, 0.905], transform=axz.transAxes, color=INK, lw=1.4, ls='--')
    axz.plot([0.02, 0.09], [0.765, 0.765], transform=axz.transAxes, color=gold, lw=2.0)
    axz.text(0.11, 0.88, 'desired', transform=axz.transAxes, fontsize=13, color=INK)
    axz.text(0.11, 0.74, 'actual', transform=axz.transAxes, fontsize=13, color=gold)
    z_txt = axz.text(0.99, 0.84, '', transform=axz.transAxes, ha='right', fontsize=14, color=INK)
    # 3-D tracking error
    axe.plot(L.t, L.err_cm, color=LIGHT, lw=0.8)
    el, = axe.plot([], [], color=gold, lw=2.0)
    top = 5 * np.ceil(np.nanmax(L.err_cm) / 5)
    axe.set_ylim(0, top)
    axe.set_ylabel('3-D error (cm)', fontsize=14, color=INK)
    axe.set_xlabel('t (s)', fontsize=14, color=INK)
    e_txt = axe.text(0.99, 0.84, '', transform=axe.transAxes, ha='right', fontsize=14, color=INK)
    curs = []
    for a in (axt, axz, axe):
        for name, te in ev.items():
            a.axvline(te, color=LIGHT, lw=0.8, zorder=0)
            if a is axt:
                a.text(te, 15.4, name, fontsize=13, color=GREY, ha='center', va='bottom')
        curs.append(a.axvline(0, color=INK, lw=0.8))
    fig.align_ylabels([axt, axz, axe])

    def update(tf):
        w = L[L.t <= tf]
        tl_line.set_data(w.t, w.tilt_deg)
        zl.set_data(w.t, w.load_z)
        el.set_data(w.t, w.err_cm)
        tl_txt.set_text(f'{at(L, "tilt_deg", tf):.1f} deg')
        zd = at(L, 'z_tgt', tf) if tf >= ev['lift-off'] - 1.0 else np.nan
        z_txt.set_text(f'{at(L, "load_z", tf):.2f} m' + ('' if np.isnan(zd) else f' (desired {max(zd, 0.0):.2f})'))
        e = w.err_cm.iloc[-1] if len(w) else np.nan
        e_txt.set_text('' if np.isnan(e) else f'{e:.1f} cm')
        for c in curs:
            c.set_xdata([tf, tf])
    return fig, update


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--video', required=True)
    ap.add_argument('--logs', required=True)
    ap.add_argument('--v0', type=float, required=True)
    ap.add_argument('--duration', type=float, required=True)
    ap.add_argument('--fps', type=float, default=29.97)
    ap.add_argument('--out', required=True)
    ap.add_argument('--ffmpeg', default='ffmpeg')
    a = ap.parse_args()
    L, ev = load(a.logs, a.v0)
    fig, update = build(L, ev, a.duration)
    # two passes: the panel to its own file, then the two side by side at a fixed rate (the iPhone
    # clip is variable-rate on a 1/600 s clock; without fps the stack emits far too many frames)
    panel = os.path.splitext(a.out)[0] + '_panel.mp4'
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    p = subprocess.Popen([a.ffmpeg, '-hide_banner', '-loglevel', 'error', '-y', '-f', 'rawvideo',
                          '-pix_fmt', 'rgba', '-s', f'{W}x{H}', '-r', str(a.fps), '-i', '-',
                          '-c:v', 'libx264', '-preset', 'veryfast', '-crf', '18', '-pix_fmt', 'yuv420p', panel],
                         stdin=subprocess.PIPE)
    for j in range(int(a.duration * a.fps)):
        update(j / a.fps)
        fig.canvas.draw()
        p.stdin.write(bytes(fig.canvas.buffer_rgba()))
    p.stdin.close()
    p.wait()
    rc = subprocess.call([a.ffmpeg, '-hide_banner', '-loglevel', 'error', '-y', '-i', os.path.expanduser(a.video),
                          '-i', panel, '-filter_complex',
                          f'[0:v]fps={a.fps},scale=-2:1080,setsar=1[v0];[1:v]fps={a.fps},setsar=1[v1];'
                          '[v0][v1]hstack=inputs=2[v]',
                          '-map', '[v]', '-map', '0:a?', '-r', str(a.fps), '-c:v', 'libx264', '-preset', 'veryfast', '-crf', '20',
                          '-pix_fmt', 'yuv420p', '-c:a', 'aac', '-shortest', a.out])
    print(a.out, 'rc', rc, 'events', {k: round(v, 2) for k, v in ev.items()})


if __name__ == '__main__':
    main()

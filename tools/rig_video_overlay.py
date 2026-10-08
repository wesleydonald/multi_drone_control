#!/usr/bin/env python3
"""tools/rig_video_overlay.py -- a rig flight video beside live plots from its logs, 1920x1080.

    python3 tools/rig_video_overlay.py --video ~/Downloads/IMG_1960.mov \\
        --logs results/rig/2026-10-01/c1_z100_logs/logs/controller_quad_load --stamp 1628 \\
        --v0 1790836108.24 --mode paths --out results/rig/2026-10-01/figures/c1_video_paths.mp4
    python3 tools/rig_video_overlay.py --video ~/Downloads/IMG_2026.mov \\
        --logs results/rig/2026-10-07/r206e60_logs/logs --v0 1791352342.0 --mode paths \\
        --out results/rig/2026-10-07/figures/r206e60_video_paths.mp4

--logs: a run's logs folder (mpc_planner/ + tracker/, since 3 Oct) or, for older runs, the
controller_quad_load folder with --stamp.
--v0: the wall-clock epoch of the video's first frame (sync it on a visible event: ring lift-off,
touchdown). --mode paths: the ring's top-view path and height against its reference. --mode drones:
each UAV's distance to its own reference. --mode detach (--logs = the rig run, e.g.
results/rig/2026-10-08/r305): top view against the offset-corrected reference, ring tilt, and the
3-D error with the offset and the lag removed (zero outside TAKEOFF..LAND), one line at the detach. Needs ffmpeg (--ffmpeg, else on PATH).
"""
import argparse
import glob
import json
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

W, H = 746, 1080          # plot panel; the video is scaled to 1080 high beside it
INK, GREY, LIGHT = '#1E2633', '#6A7179', '#C9CED6'


PATH_NAMES = {'orbit': 'circle', 'fig_8': 'figure-8'}


def load(logs, stamp, v0, t_end):
    drones = []
    if os.path.isdir(os.path.join(logs, 'tracker')):     # the 3 Oct layout
        dirs = [sorted(glob.glob(os.path.join(logs, 'tracker', f'drone{i}_*')))[-1]
                for i in range(len(glob.glob(os.path.join(logs, 'tracker', 'drone*_*'))))
                if glob.glob(os.path.join(logs, 'tracker', f'drone{i}_*'))]
        lp = os.path.dirname(glob.glob(os.path.join(logs, '*planner', '*', 'log.csv'))[0])
        traj = json.load(open(os.path.join(lp, 'params.json'))).get('load_traj', '')
        path = PATH_NAMES.get(traj, traj)
    else:
        dirs = [sorted(glob.glob(os.path.join(logs, f'planner_drone{i}_*_{stamp}*')))[-1] for i in range(3)]
        lp = sorted(glob.glob(os.path.join(logs, f'load_planner_*_{stamp}*')))[-1]
        path = 'circle'
    for d in dirs:
        T = pd.read_csv(os.path.join(d, 'log.csv')).drop_duplicates('sim_time')
        T['t'] = T.sim_time - v0
        drones.append(T[(T.t >= -1) & (T.t <= t_end + 1)].reset_index(drop=True))
    ring = drones[0]
    L = pd.read_csv(os.path.join(lp, 'log.csv'))
    L['t'] = L.sim_time - v0
    rest = L.load_z.iloc[:20].median()
    lift = float(L.t[L.load_z > rest + 0.03].iloc[0])
    events = [('lift-off', lift)]
    if (L.traj_t > 0).any():
        events.append((path, float(L.t[L.traj_t > 0].iloc[0])))
    if (L.land != 0).any():
        events.append(('LAND', float(L.t[L.land != 0].iloc[0])))
    return drones, ring, events


def setup_text(fig, title, sub):
    fig.text(0.06, 0.965, title, fontsize=19, fontweight='bold', color=INK, va='top')
    fig.text(0.06, 0.928, sub, fontsize=13, color=GREY, va='top')


def panel_paths(ring, events, t_end, shift=False):
    fig = plt.figure(figsize=(W / 100, H / 100), dpi=100, facecolor='#FBFBF8')
    axp = fig.add_axes([0.14, 0.43, 0.80, 0.49])
    axz = fig.add_axes([0.14, 0.07, 0.80, 0.27])
    for a in (axp, axz):
        a.set_facecolor('#FBFBF8')
        for s in ('top', 'right'):
            a.spines[s].set_visible(False)
        a.tick_params(labelsize=12, colors=INK)
    r = ring.copy()
    ref_label = 'reference'
    if shift:
        ev = dict(events)
        t_path = events[1][1] if len(events) > 1 and events[1][0] != 'LAND' else 0.0
        w = r[(r.t >= t_path) & (r.t <= ev.get('LAND', t_end))]
        off = (w.payload_x - w.payload_ref_x).mean(), (w.payload_y - w.payload_ref_y).mean()
        r['payload_ref_x'] = r.payload_ref_x + off[0]
        r['payload_ref_y'] = r.payload_ref_y + off[1]
        ref_label = f'reference, shifted by the mean offset ({100 * off[0]:+.1f}, {100 * off[1]:+.1f}) cm'
    axp.plot(r.payload_ref_x, r.payload_ref_y, '--', color=GREY, lw=1.2, dashes=(5, 3), label=ref_label)
    tr, = axp.plot([], [], color=ps.PAYLOAD_LINE, lw=2.2, label='ring (mocap)')
    vec, = axp.plot([], [], color=GREY, lw=0.8)
    ref_pt, = axp.plot([], [], 'o', mfc='none', mec=INK, ms=9, mew=1.6)
    ring_pt, = axp.plot([], [], 'o', color=ps.PAYLOAD_LINE, ms=9)
    xs = np.r_[r.payload_x, r.payload_ref_x]
    ys = np.r_[r.payload_y, r.payload_ref_y]
    pad = 0.12
    axp.set_xlim(xs.min() - pad, xs.max() + pad)
    axp.set_ylim(ys.min() - pad, ys.max() + pad)
    axp.set_aspect('equal', adjustable='box')
    axp.set_xlabel('x (m)', fontsize=13, color=INK)
    axp.set_ylabel('y (m)', fontsize=13, color=INK)
    axp.legend(loc='upper center', bbox_to_anchor=(0.5, 1.12), frameon=False, fontsize=12, ncol=1)
    err_txt = axp.text(0.02, 0.02, '', transform=axp.transAxes, fontsize=13, color=INK)
    axz.plot(r.t, r.payload_ref_z, '--', color=GREY, lw=1.2, dashes=(5, 3))
    ztr, = axz.plot([], [], color=ps.PAYLOAD_LINE, lw=2.2)
    cur = axz.axvline(0, color=INK, lw=0.8)
    axz.set_xlim(0, t_end)
    axz.set_ylim(0, 1.25)
    axz.set_yticks([0, 0.5, 1.0])
    axz.set_xlabel('t (s)', fontsize=13, color=INK)
    axz.set_ylabel('Ring height (m)', fontsize=13, color=INK)
    for name, te in events:
        axz.axvline(te, color=LIGHT, lw=0.8, zorder=0)
        axz.text(te, 1.27, name, fontsize=11, color=GREY, ha='center', va='bottom')
    z_txt = axz.text(0.99, 0.06, '', transform=axz.transAxes, fontsize=12, color=INK, ha='right')

    def update(tf):
        k = int(np.searchsorted(r.t.values, tf, side='right'))
        k = max(k, 1)
        w = r.iloc[:k]
        tr.set_data(w.payload_x, w.payload_y)
        x, y = w.payload_x.iloc[-1], w.payload_y.iloc[-1]
        rx, ry = w.payload_ref_x.iloc[-1], w.payload_ref_y.iloc[-1]
        ring_pt.set_data([x], [y])
        ref_pt.set_data([rx], [ry])
        vec.set_data([x, rx], [y, ry])
        err_txt.set_text(f'horizontal error {100 * np.hypot(x - rx, y - ry):4.1f} cm' + (' (to the shifted reference)' if shift else ''))
        ztr.set_data(w.t, w.payload_z)
        cur.set_xdata([tf, tf])
        z_txt.set_text(f'height {w.payload_z.iloc[-1]:.2f} m (reference {max(w.payload_ref_z.iloc[-1], 0):.2f} m)')
    return fig, update


def load_run(base, v0):
    """A rig run converted by rig_to_run.py (rig_summary.resolve converts it once), on the video's
    clock: run.csv's t + its t0_epoch - v0. Returns (run.csv frame with `tv`, [(event, tv)])."""
    import rig_summary
    import metrics
    run, _, _, _, _ = rig_summary.resolve(base)
    t0 = json.load(open(os.path.join(run, 'manifest.json')))['t0_epoch']
    d = pd.read_csv(os.path.join(run, 'logs', 'run.csv'))
    d['tv'] = d.t + t0 - v0
    return d, [(e, t + t0 - v0) for t, e, _ in metrics.read_event_list(run)]


def panel_detach(d, events, t_end):
    """Top view (the ring against its reference circle, offset-corrected), the ring tilt and its 3-D
    error with the circle's mean offset and the lag removed: the distance to the nearest point of the
    path plus the height error. Tilt and error are zero before TAKEOFF and after LAND."""
    import rig_summary as rs
    t = d.tv.to_numpy(float)
    P = d[['payload_x', 'payload_y', 'payload_z']].to_numpy(float)
    R = rs.continuous_ref(t, d[['payload_ref_x', 'payload_ref_y', 'payload_ref_z']].to_numpy(float))
    ev = {}
    for name, te in events:
        ev.setdefault(name, te)
    t_off, t_land = ev.get('TAKEOFF', t[0]), ev.get('LAND', t[-1])
    t_rel = ev.get('RELEASE', ev.get('DETACH'))
    fly = (t >= t_off) & (t <= t_land)
    moving = rs.error_parts(t, P, R)[4] & fly
    off = np.nanmean(P[moving, :2] - R[moving, :2], axis=0)
    path = R[moving, :2] + off
    near = np.array([np.min(np.hypot(*(path - P[i, :2]).T)) if fly[i] else 0.0 for i in range(len(t))])
    err = np.where(fly, 100 * np.sqrt(near ** 2 + (P[:, 2] - R[:, 2]) ** 2), 0.0)
    tilt = np.where(fly, np.nan_to_num(d.payload_tilt_deg.to_numpy(float)), 0.0)

    fig = plt.figure(figsize=(W / 100, H / 100), dpi=100, facecolor='#FBFBF8')
    axp = fig.add_axes([0.15, 0.50, 0.78, 0.44])
    axt = fig.add_axes([0.15, 0.285, 0.78, 0.15])
    axe = fig.add_axes([0.15, 0.065, 0.78, 0.15])
    for a in (axp, axt, axe):
        a.set_facecolor('#FBFBF8')
        for sp in ('top', 'right'):
            a.spines[sp].set_visible(False)
        a.tick_params(labelsize=12, colors=INK)
    axp.plot(path[:, 0], path[:, 1], '--', color=GREY, lw=1.2, dashes=(5, 3), label='reference')
    tr, = axp.plot([], [], color=ps.PAYLOAD_LINE, lw=2.2, label='ring (mocap)')
    ring_pt, = axp.plot([], [], 'o', color=ps.PAYLOAD_LINE, ms=9)
    rel_pt, = axp.plot([], [], 'o', mfc='white', mec='#E07B00', ms=9, mew=2)
    rel_txt = axp.text(0, 0, '', fontsize=12, color='#E07B00')
    w = fly & (t >= ev.get('HANDOVER', t_off))
    xs, ys = np.r_[P[w, 0], path[:, 0]], np.r_[P[w, 1], path[:, 1]]
    cx, cy = (xs.max() + xs.min()) / 2, (ys.max() + ys.min()) / 2
    half = 0.5 * max(xs.max() - xs.min(), ys.max() - ys.min()) + 0.08
    axp.set_xlim(cx - half, cx + half)
    axp.set_ylim(cy - half, cy + half)
    axp.set_aspect('equal', adjustable='box')
    axp.set_xlabel('x (m)', fontsize=13, color=INK)
    axp.set_ylabel('y (m)', fontsize=13, color=INK)
    axp.legend(loc='upper center', bbox_to_anchor=(0.5, 1.10), frameon=False, fontsize=12, ncol=2)
    lines = []
    for a, y, ylab, top in ((axt, tilt, 'Ring tilt (°)', max(10.0, 1.15 * tilt.max())),
                            (axe, err, '3D error (cm)', max(5.0, 1.15 * err.max()))):
        a.set_xlim(0, t_end)
        a.set_ylim(0, top)
        a.set_ylabel(ylab, fontsize=13, color=INK)
        if t_rel is not None:
            a.axvline(t_rel, color='#E07B00', lw=1.2, zorder=0)
        ln, = a.plot([], [], color=ps.PAYLOAD_LINE if a is axt else INK, lw=1.8)
        cur = a.axvline(0, color=INK, lw=0.8)
        txt = a.text(0.99, 0.82, '', transform=a.transAxes, fontsize=12, color=INK, ha='right')
        lines.append((y, ln, cur, txt, '°' if a is axt else ' cm'))
    if t_rel is not None:
        axt.text(t_rel, axt.get_ylim()[1], ' detach', fontsize=12, color='#E07B00', va='bottom')
    axe.set_xlabel('t (s)', fontsize=13, color=INK)

    def update(tf):
        k = max(int(np.searchsorted(t, tf, side='right')), 1)
        m = fly[:k] & (t[:k] >= ev.get('HANDOVER', t_off))
        tr.set_data(P[:k, 0][m], P[:k, 1][m])
        ring_pt.set_data([P[k - 1, 0]], [P[k - 1, 1]])
        if t_rel is not None and tf >= t_rel:
            i = int(np.searchsorted(t, t_rel))
            rel_pt.set_data([P[i, 0]], [P[i, 1]])
            rel_txt.set_position((P[i, 0] + 0.03, P[i, 1] + 0.02))
            rel_txt.set_text('detach')
        for y, ln, cur, txt, unit in lines:
            ln.set_data(t[:k], y[:k])
            cur.set_xdata([tf, tf])
            txt.set_text(f'{y[k - 1]:4.1f}{unit}')
    return fig, update


def panel_drones(drones, events, t_end):
    fig = plt.figure(figsize=(W / 100, H / 100), dpi=100, facecolor='#FBFBF8')
    axes, upd = [], []
    ymax = 10.0
    for i, T in enumerate(drones):
        a = fig.add_axes([0.14, 0.69 - 0.285 * i, 0.80, 0.22])
        a.set_facecolor('#FBFBF8')
        for s in ('top', 'right'):
            a.spines[s].set_visible(False)
        a.tick_params(labelsize=12, colors=INK)
        e = 100 * np.linalg.norm(T[['pose_x', 'pose_y', 'pose_z']].values - T[['ref_x', 'ref_y', 'ref_z']].values, axis=1)
        T = T.assign(err=e)
        drones[i] = T
        a.set_xlim(0, t_end)
        a.set_ylim(0, ymax)
        a.set_yticks([0, 5, 10])
        a.set_ylabel('Error (cm)', fontsize=12, color=INK)
        a.text(0.0, 1.04, f'UAV {i + 1}', transform=a.transAxes, fontsize=14, fontweight='bold',
               color=ps.drone_colour(i))
        for name, te in events:
            a.axvline(te, color=LIGHT, lw=0.8, zorder=0)
            if i == 0:
                a.text(te, ymax * 1.22, name, fontsize=11, color=GREY, ha='center', va='bottom')
        if i == 2:
            a.set_xlabel('t (s)', fontsize=13, color=INK)
        ln, = a.plot([], [], color=ps.drone_colour(i), lw=1.8)
        cur = a.axvline(0, color=INK, lw=0.8)
        txt = a.text(0.99, 0.80, '', transform=a.transAxes, fontsize=12, color=INK, ha='right')
        axes.append(a)
        upd.append((T, ln, cur, txt))
    fig.text(0.14, 0.012, 'Off scale (above 10 cm) only while creeping up and after touchdown, when the\n'
             'reference is lower than a UAV on its rod can reach.', fontsize=11, color=GREY, va='bottom')

    def update(tf):
        for T, ln, cur, txt in upd:
            k = max(int(np.searchsorted(T.t.values, tf, side='right')), 1)
            ln.set_data(T.t.values[:k], np.minimum(T.err.values[:k], ymax))
            cur.set_xdata([tf, tf])
            txt.set_text(f'{T.err.values[k - 1]:4.1f} cm')
    return fig, update


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--video', required=True)
    ap.add_argument('--logs', required=True)
    ap.add_argument('--stamp', default='', help='old layout only: HHMM prefix of the launch stamp, e.g. 1628')
    ap.add_argument('--v0', type=float, required=True)
    ap.add_argument('--mode', choices=('paths', 'drones', 'detach'), default='paths')
    ap.add_argument('--out', required=True)
    ap.add_argument('--ffmpeg', default='ffmpeg')
    ap.add_argument('--duration', type=float, default=None, help='s (default: the whole video)')
    ap.add_argument('--fps', type=float, default=29.97)
    ap.add_argument('--shift', action='store_true', help='paths: shift the xy reference by the mean offset over the trajectory')
    a = ap.parse_args()
    video = os.path.expanduser(a.video)
    if a.duration is None:
        a.duration = float(subprocess.check_output(
            ['ffprobe', '-v', 'error', '-show_entries', 'format=duration', '-of', 'csv=p=0', video]))
    if a.mode == 'detach':
        d, events = load_run(a.logs, a.v0)
        fig, update = panel_detach(d, events, a.duration)
    else:
        drones, ring, events = load(a.logs, a.stamp, a.v0, a.duration)
        fig, update = (panel_paths(ring, events, a.duration, a.shift) if a.mode == 'paths'
                       else panel_drones(drones, events, a.duration))
    n = int(a.duration * a.fps)
    # two passes at a fixed rate: piping the panel straight into the stack stalls on variable-rate
    # iPhone clips (1/600 s clock; 7 Oct)
    panel = os.path.splitext(a.out)[0] + '_panel.mp4'
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    p = subprocess.Popen([a.ffmpeg, '-hide_banner', '-loglevel', 'error', '-y', '-f', 'rawvideo',
                          '-pix_fmt', 'rgba', '-s', f'{W}x{H}', '-r', str(a.fps), '-i', '-',
                          '-c:v', 'libx264', '-preset', 'veryfast', '-crf', '18', '-pix_fmt', 'yuv420p', panel],
                         stdin=subprocess.PIPE)
    for j in range(n):
        update(j / a.fps)
        fig.canvas.draw()
        p.stdin.write(bytes(fig.canvas.buffer_rgba()))
    p.stdin.close()
    p.wait()
    rc = subprocess.call([a.ffmpeg, '-hide_banner', '-loglevel', 'error', '-y', '-i', video, '-i', panel,
                          '-filter_complex', f'[0:v]fps={a.fps},scale=-2:1080,setsar=1[v0];'
                          f'[1:v]fps={a.fps},setsar=1[v1];[v0][v1]hstack=inputs=2[v]',
                          '-map', '[v]', '-map', '0:a?', '-r', str(a.fps), '-c:v', 'libx264', '-crf', '20',
                          '-preset', 'veryfast', '-pix_fmt', 'yuv420p', '-c:a', 'aac', '-shortest', a.out])
    os.remove(panel)
    print(a.out, 'rc', rc, 'events', [(k, round(v, 2)) for k, v in events])


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Attach during a circle in Gazebo (R0209): plan view, three snapshots, ring tilt.

    python3 storyboard.py [RUN_DIR] [--out STEM]

Brief (thesis-figure skill, step 1)
-----------------------------------
1. Sentence. With three carriers on one side the ring flies the circle tilted by about
   35 deg; after the newcomer welds to the stationary ring at its 6 o'clock rim, the
   four-drone reconfiguration levels it to under 10 deg and the fleet completes the circle.
2. Hero. (c) ring tilt against time: the widest panel and the only heavy trace. It
   carries 35 deg -> under 10 deg and when that happens relative to WELD.
   Support: (a) plan view (where: the frozen target, the newcomer's approach ending at
   the 6 o'clock rim, the resumed circle) and (b) three snapshots of the fleet at
   ATTACH, WELD and RESUME, reconstructed from the logged poses (how: three carriers on
   one side tip the ring, four level it).
3. Left out. Carrier paths in the plan (they orbit the ring and bury the story), the
   newcomer after the weld in the plan, heights, tensions, tracking errors, the lift
   transient before the target moves, everything after LAND, the tilt peak annotation,
   a legend. The newcomer is not drawn in the ATTACH snapshot: it is 1.3 m away, and
   (a) shows where it waits.
4. Window. From the first sample at which the payload target moves (the circle starts)
   to LAND (events.csv).
5. Caption draft. "Attach during a circle in Gazebo (R0209, 0.6 kg ring, 0.5 m rods,
   0.5 m radius at 0.2 m/s; dissipative network). (a) Plan view: payload target
   (dashed; it jumps once, when the target is re-anchored at the weld), ring centre
   (solid gold; open circle: start; arrow: direction of travel) and the newcomer's
   approach from ATTACH to WELD (red; open circle: where it waited), with the ring
   outline at the weld. (b) The fleet at ATTACH, WELD and RESUME, drawn from the logged
   ring pose and drone positions (oblique view; rods drawn from each drone to its rim
   anchor). (c) Ring tilt; the grey band is the 20 s for which the target is held.
   Three drones at 3, 12 and 9 o'clock fly the circle with the ring tilted by about
   35 deg; ATTACH freezes the payload target; the newcomer welds to the stationary ring
   at its 6 o'clock rim; the reconfiguration levels the ring to under 10 deg within
   about 8 s, after which the trajectory resumes and the four-drone fleet completes
   the circle."

Every number drawn comes from the run directory: run.csv (poses, tilt, attach flag),
events.csv (ATTACH = MAGNET, WELD, LAND), launch.log (the reconfiguration hold, hence
RESUME, and the weld offset on the rim), manifest.json (rim azimuths of the carriers)
and params/dissipative_controller.yaml (rim radius).
"""
import argparse
import os
import re
import sys

import numpy as np

REPO = '/home/wesley/multi_drone_control'
sys.dont_write_bytecode = True                  # read the repo, write nothing into it
sys.path.insert(0, os.path.join(REPO, 'tools'))
import plot_style as ps                                   # noqa: E402
import metrics                                            # noqa: E402
import matplotlib.pyplot as plt                           # noqa: E402

RUN = os.path.join(REPO, 'results_archive', 'R0209_sim_gz_attach_circle_n3_fast')
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'storyboard')

# Snapshot camera: azimuth of the viewer about z and elevation above the horizon (deg).
# Chosen so the four drones never overlap and the tilted ring is seen well off edge-on.
VIEW_AZ, VIEW_EL = -50.0, 25.0

GREY_EVENT = '0.55'
GREY_REF = '0.45'
GREY_ROD = '0.55'
FILL = '0.935'


# ── data ────────────────────────────────────────────────────────────────────────────

def load(run_dir):
    run = metrics.load_run(run_dir)
    d, ev = run['data'], run['events']
    log = open(os.path.join(run_dir, 'logs', 'launch.log')).read()
    hold = float(re.search(r'trajectory held ([0-9.]+) s for the reconfiguration', log).group(1))
    m = re.search(r'weld offset captured for slot (\d+): rho=\(([-0-9.]+),([-0-9.]+)\)', log)
    slot, rho = int(m.group(1)), (float(m.group(2)), float(m.group(3)))
    args = run['manifest']['launch_args']
    az = [float(a) for a in str(args['attach_azimuths_deg']).split(',')]
    par = open(os.path.join(run_dir, 'params', 'dissipative_controller.yaml')).read()
    r_ring = float(re.search(r'attach_radius:\s*([0-9.]+)', par).group(1))
    t = d['t']
    step = np.hypot(np.diff(d['payload_ref_x']), np.diff(d['payload_ref_y']))
    t0 = t[1:][step > 0][0]                       # the payload target starts to move
    return dict(d=d, t=t, t0=t0, t_attach=ev['MAGNET'], t_weld=ev['WELD'],
                t_resume=ev['WELD'] + hold, t_land=ev['LAND'], slot=slot, rho=rho,
                az=az, r_ring=r_ring, n=metrics.n_drones(d))


def rot(d, k):
    w, x, y, z = (d['payload_q' + a][k] for a in 'wxyz')
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def pos(d, pre, k):
    return np.array([d[f'{pre}_x'][k], d[f'{pre}_y'][k], d[f'{pre}_z'][k]])


def break_jumps(x, y, max_step):
    """NaN where consecutive samples jump by more than `max_step`, so a step change is
    drawn as a step, not as a line nobody flew."""
    x, y = x.copy(), y.copy()
    j = np.where(np.hypot(np.diff(x), np.diff(y)) > max_step)[0] + 1
    return np.insert(x, j, np.nan), np.insert(y, j, np.nan)


# ── snapshot geometry ──────────────────────────────────────────────────────────────

def project(p):
    """World offset (3,) or (N, 3) -> screen (u, v): orthographic, camera at VIEW_AZ."""
    a, e = np.radians(VIEW_AZ), np.radians(VIEW_EL)
    right = np.array([-np.sin(a), np.cos(a), 0.0])
    away = np.array([-np.cos(a), -np.sin(a), 0.0])
    p = np.atleast_2d(p)
    return np.column_stack([p @ right, p[:, 2] * np.cos(e) + (p @ away) * np.sin(e)])


def snapshot(S, k):
    """Everything drawn in one frame, as screen coordinates relative to the ring centre."""
    d = S['d']
    P, Rk = pos(d, 'payload', k), rot(d, k)
    a = np.linspace(0, 2 * np.pi, 181)
    ring = (Rk @ np.vstack([S['r_ring'] * np.cos(a), S['r_ring'] * np.sin(a), 0 * a])).T
    drones, rods = [], []
    for i in range(S['n']):
        q = pos(d, f'd{i}', k) - P
        if i < len(S['az']):
            anchor = Rk @ (S['r_ring'] * np.array([np.cos(np.radians(S['az'][i])),
                                                   np.sin(np.radians(S['az'][i])), 0.0]))
        elif i == S['slot'] and d[f'd{i}_attached'][k] > 0.5:
            anchor = Rk @ np.array([S['rho'][0], S['rho'][1], 0.0])
        else:
            anchor = None
        drones.append((i, project(q)[0]))
        if anchor is not None:
            rods.append(project(np.vstack([anchor, q])))
    return project(ring), drones, rods


# ── figure ─────────────────────────────────────────────────────────────────────────

def tick_g(v, _):
    """Tick label without trailing zeros: 0, 0.5, 1."""
    return f'${v:g}$'


def at_or_after(t, te):
    """First sample at or after an event, so a frame at WELD already shows the weld."""
    return int(np.searchsorted(t, te))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('run', nargs='?', default=RUN)
    ap.add_argument('--out', default=OUT)
    a = ap.parse_args()
    S = load(a.run)
    d, t = S['d'], S['t']
    newcomer = S['slot']
    c3 = ps.drone_colour(newcomer)
    win = (t >= S['t0']) & (t <= S['t_land'])

    ps.thesis()
    W, H = ps.size('full', height=3.75)
    fig = plt.figure(figsize=(W, H))
    gs = fig.add_gridspec(2, 2, width_ratios=[1.0, 1.95], height_ratios=[1.0, 1.75])
    ax_p = fig.add_subplot(gs[:, 0])
    gsf = gs[0, 1].subgridspec(1, 3, wspace=0.0)
    ax_f = [fig.add_subplot(gsf[0, j]) for j in range(3)]
    ax_t = fig.add_subplot(gs[1, 1])

    # (a) plan view ------------------------------------------------------------------
    rx, ry = break_jumps(d['payload_ref_x'][win], d['payload_ref_y'][win], 0.05)
    ax_p.plot(rx, ry, ls=(0, (3.5, 2)), lw=0.8, color=GREY_REF, zorder=2)
    px, py = d['payload_x'][win], d['payload_y'][win]
    ax_p.plot(px, py, lw=1.0, color=ps.PAYLOAD_LINE, zorder=3,
              rasterized=win.sum() > 2000)
    app = (t >= S['t_attach']) & (t <= S['t_weld'])
    nx, ny = d[f'd{newcomer}_x'][app], d[f'd{newcomer}_y'][app]
    ax_p.plot(nx, ny, lw=0.9, color=c3, zorder=3)
    ax_p.plot(nx[0], ny[0], 'o', ms=3.2, mfc='white', mec=c3, mew=0.8, zorder=4)
    # the ring at the weld (plan projection of the tilted ring): the newcomer's path
    # ends on its 6 o'clock rim
    kw = at_or_after(t, S['t_weld'])
    ang = np.linspace(0, 2 * np.pi, 181)
    rw = (rot(d, kw) @ np.vstack([S['r_ring'] * np.cos(ang), S['r_ring'] * np.sin(ang),
                                   0 * ang])).T + pos(d, 'payload', kw)
    ax_p.plot(rw[:, 0], rw[:, 1], lw=0.7, color=ps.PAYLOAD_LINE, zorder=3)
    ax_p.plot(px[0], py[0], 'o', ms=3.2, mfc='white', mec='0.15', mew=0.8, zorder=5)
    # direction of travel, once, on the resumed circle
    kr = np.where(win & (t > S['t_resume']))[0]
    k1 = kr[len(kr) // 5]
    ax_p.annotate('', xy=(d['payload_x'][k1 + 4], d['payload_y'][k1 + 4]),
                  xytext=(d['payload_x'][k1], d['payload_y'][k1]),
                  arrowprops=dict(arrowstyle='-|>', mutation_scale=8, lw=0,
                                  color=ps.PAYLOAD_LINE, shrinkA=0, shrinkB=0), zorder=4)
    kb = int(np.argmin(rw[:, 1] - 0.5 * rw[:, 0]))        # lower-right of the outline
    ax_p.annotate(r'\textsc{weld}', xy=tuple(rw[kb, :2]),
                  xytext=(6, -16), ha='left', va='top', fontsize=8,
                  textcoords='offset points',
                  arrowprops=dict(arrowstyle='-', lw=0.5, color='0.4', shrinkA=1, shrinkB=1))
    kn = len(nx) // 2
    ax_p.annotate('newcomer', xy=(nx[kn], ny[kn]), xytext=(-6, 0), ha='right', va='center',
                  fontsize=8, color=c3, textcoords='offset points')
    ktop = int(np.nanargmax(py))
    ax_p.annotate('ring', xy=(px[ktop], py[ktop]), xytext=(0, 4), ha='center',
                  va='bottom', fontsize=8, color=ps.PAYLOAD_LINE, textcoords='offset points')
    kend = np.where(np.isfinite(rx))[0][-1]
    ax_p.annotate('target', xy=(rx[kend], ry[kend]), xytext=(2, 5), ha='right',
                  va='bottom', fontsize=8, color=GREY_REF, textcoords='offset points')

    ax_p.set_aspect('equal', adjustable='box')
    xs = np.concatenate([px, nx, rw[:, 0]])
    ys = np.concatenate([py, ny, rw[:, 1]])
    pad = 0.05 * max(np.nanmax(xs) - np.nanmin(xs), np.nanmax(ys) - np.nanmin(ys))
    # extend to the next round tick if the crop would leave the axis with two ticks
    tick = 0.5
    ax_p.set_xlim(min(np.nanmin(xs) - pad, np.floor(np.nanmin(xs) / tick) * tick - 0.02),
                  np.nanmax(xs) + pad)
    ax_p.set_ylim(np.nanmin(ys) - pad, np.nanmax(ys) + pad)
    ax_p.set_xlabel(r'Position $x$ (m)')
    ax_p.set_ylabel(r'Position $y$ (m)')
    ax_p.set_anchor('N')
    for axis in (ax_p.xaxis, ax_p.yaxis):
        axis.set_major_locator(plt.MultipleLocator(tick))
        axis.set_major_formatter(plt.FuncFormatter(tick_g))

    # (b) snapshots --------------------------------------------------------------------
    names = [(r'\textsc{attach}', S['t_attach']), (r'\textsc{weld}', S['t_weld']),
             (r'\textsc{resume}', S['t_resume'])]
    snaps = [snapshot(S, at_or_after(t, te)) for _, te in names]
    # one window for all three frames: the ring and every rod (a rod ends at its drone);
    # the unattached newcomer is drawn only where it falls inside that window
    allp = np.vstack([s[0] for s in snaps] + [r for s in snaps for r in s[2]])
    lo, hi = allp.min(0), allp.max(0)
    m = 0.06 * (hi - lo).max()
    for j, (ax, (nm, te), (ring, drones, rods)) in enumerate(zip(ax_f, names, snaps)):
        for r in rods:
            ax.plot(r[:, 0], r[:, 1], lw=0.6, color=GREY_ROD, zorder=1)
        ax.plot(ring[:, 0], ring[:, 1], lw=1.2, color=ps.PAYLOAD_LINE, zorder=2)
        for i, p in drones:
            if not (lo[0] - m <= p[0] <= hi[0] + m and lo[1] - m <= p[1] <= hi[1] + m):
                continue
            ax.plot(*p, 'o', ms=4.6, mfc=ps.drone_colour(i), mec='white', mew=0.6, zorder=3)
            if i == newcomer and j == 1:
                ax.annotate('newcomer', xy=p, xytext=(0, 4), textcoords='offset points',
                            ha='center', va='bottom', fontsize=8, color=c3)
        ax.set_xlim(lo[0] - m, hi[0] + m)
        ax.set_ylim(lo[1] - m, hi[1] + m)
        ax.set_aspect('equal', adjustable='box')
        ax.set_anchor('N')
        ax.axis('off')
        ax.text(0.5, 1.0, f'{nm}\\enspace ${te:.0f}$\\,s', transform=ax.transAxes,
                ha='center', va='bottom', fontsize=8)

    # (c) tilt -------------------------------------------------------------------------
    tt, th = t[win], d['payload_tilt_deg'][win]
    ax_t.axvspan(S['t_attach'], S['t_resume'], color=FILL, lw=0, zorder=0)
    for name, te in (('attach', S['t_attach']), ('weld', S['t_weld']),
                     ('resume', S['t_resume'])):
        ax_t.axvline(te, color=GREY_EVENT, lw=0.5, zorder=1)
        ax_t.annotate(rf'\textsc{{{name}}}', xy=(te, 1.0), xycoords=('data', 'axes fraction'),
                      xytext=(0, 2), textcoords='offset points', ha='center', va='bottom',
                      fontsize=8)
    ax_t.plot(tt, th, lw=1.2, color=ps.PAYLOAD_LINE, zorder=3, rasterized=tt.size > 2000)
    ax_t.set_xlim(S['t0'], S['t_land'])
    ax_t.set_ylim(0, 45)
    ax_t.yaxis.set_major_locator(plt.MultipleLocator(10))
    ax_t.xaxis.set_major_locator(plt.MultipleLocator(10))
    ax_t.set_xlabel(r'Time $t$ (s)')
    ax_t.set_ylabel(r'Ring tilt $\theta$ (\si{\degree})')
    seg3 = (tt >= S['t_attach']) & (tt <= S['t_weld'])
    seg4 = tt >= S['t_resume']
    mid3 = 0.5 * (S['t_attach'] + S['t_weld'])
    ax_t.text(mid3, np.nanmax(th[seg3]) + 1.5, '3 carriers', ha='center', va='bottom',
              fontsize=8)
    ax_t.text(0.5 * (S['t_resume'] + S['t_land']), np.nanmax(th[seg4]) + 1.5, '4 carriers',
              ha='center', va='bottom', fontsize=8)
    ax_t.text(mid3, 2.5, 'target held', ha='center', va='bottom', fontsize=8, color='0.35')

    ps.panel_label(ax_p, 'a')
    ps.panel_label(ax_f[0], 'b')
    ps.panel_label(ax_t, 'c')
    # Constrained layout sizes everything; then the filmstrip is set over the tilt axes'
    # own width (not its cell, which includes the tilt label) so (b) and (c) line up.
    fig.canvas.draw()
    fig.set_constrained_layout(False)
    bt = ax_t.get_position()
    w = bt.width / len(ax_f)
    for j, ax in enumerate(ax_f):
        bf = ax.get_position(original=True)
        ax.set_position([bt.x0 + j * w, bf.y0, w, bf.height])
    paths = ps.save_print(fig, a.out)
    print('\n'.join(paths))


if __name__ == '__main__':
    main()

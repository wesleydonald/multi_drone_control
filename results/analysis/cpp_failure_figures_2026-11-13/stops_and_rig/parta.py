"""Part A figures: his planner stops in the moving ring's path (open-loop real-time replays of the nro circle).
usage: python3 parta.py   (writes fig1..fig4 PNGs next to this script)"""
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.lines import Line2D
import numpy as np

from common_a import CATS, LINE, OUTER_R, PLATE_R, Run, cat_of, seg_seg

HERE = os.path.dirname(os.path.abspath(__file__))
plt.rcParams.update({'font.size': 10, 'axes.titlesize': 10.5, 'axes.labelsize': 10, 'legend.fontsize': 9})
CAR_C = ['#2166ac', '#4393c3', '#053061']
Q4 = '#d6604d'


def events(R):
    ev = {'auth': R.t_auth, 'cap': R.t_cap}
    pre = R.auth & ~R.cap
    if pre.any():
        i = np.where(pre)[0][np.argmin(R.min_cable_tether_m[pre])]
        ev['closest'] = R.t[i]
        ev['closest_m'] = R.min_cable_tether_m[i]
    return ev


def line_tether(R, t):
    """3-D closest distance, QUAD4's hanging line to each carrier tether, at driver time t."""
    i = R.idx(t)
    b = np.array([R.x[i], R.y[i], R.z[i]])
    lo = b - np.array([0, 0, LINE])
    ring, rv, cars = R.scene(t)
    return [seg_seg(b, lo, top, plate) for _, top, plate in cars], b, lo, ring, rv, cars


def fb_at(R, t):
    k = np.searchsorted(R.bt, t) - 1
    if k < 0 or not R.fb_active[k]:
        return 'no fallback'
    return R.fb_mode[k].replace('UNCERTIFIED_', 'UNCERT. ')


def snapshot(ax, R, t, span=0.95, centre=None, trail=3.0):
    dl, b, lo, ring, rv, cars = line_tether(R, t)
    i = R.idx(t)
    # ring path over the replay
    tt = np.arange(R.t[0], R.t[-1], 0.2)
    path = np.array([R.scene(s)[0][:2] for s in tt])
    ax.plot(path[:, 0], path[:, 1], color='0.8', lw=1, ls='--', zorder=1)
    ax.add_patch(plt.Circle(ring[:2], OUTER_R, fill=False, ls=':', color='0.35', lw=1, zorder=2))
    ax.add_patch(plt.Circle(ring[:2], PLATE_R, fill=True, fc='0.92', ec='k', lw=1.6, zorder=2))
    sp = np.hypot(rv[0], rv[1])
    if sp > 0.01:
        ax.annotate('', xy=ring[:2] + rv[:2] / sp * 0.35, xytext=ring[:2],
                    arrowprops=dict(arrowstyle='-|>', color='k', lw=1.5), zorder=6)
    k = int(np.argmin(dl))
    for j, (c, top, plate) in enumerate(cars):
        col = '#e08214' if (j == k and dl[j] < 0.5) else CAR_C[j]
        ax.plot([c[0], plate[0]], [c[1], plate[1]], color=col, lw=3 if j == k else 2, solid_capstyle='round', zorder=4)
        ax.plot(c[0], c[1], marker='^', ms=9, color=col, mec='k', zorder=5)
        ax.text(c[0] + 0.04, c[1] + 0.04, f'd{j}', fontsize=8.5, color=col, zorder=7, clip_on=True)
    # QUAD4 trail, body and target
    m = (R.t >= t - trail) & (R.t <= t)
    ax.plot(R.x[m], R.y[m], color=Q4, lw=1.4, alpha=0.7, zorder=5)
    ax.plot(R.x[i], R.y[i], 'o', ms=10, color=Q4, mec='k', zorder=8)
    ax.plot(R.tx[i], R.ty[i], 'X', ms=9, color='#1a9850', mec='k', zorder=8)
    cx, cy = (centre if centre is not None else (R.x[i], R.y[i]))
    ax.set_xlim(cx - span, cx + span)
    ax.set_ylim(cy - span, cy + span)
    ax.set_aspect('equal')
    ax.grid(alpha=0.3)
    still = 'STILL' if R.speed[i] < 0.05 else f'{R.speed[i]:.2f} m/s'
    ax.set_title(f't = {t:.1f} s   QUAD4 {still}\nclosest line-tether {min(dl):.2f} m (d{k})   {fb_at(R, t)}',
                 fontsize=9.5)
    ax.tick_params(labelsize=8.5)


def legend_handles():
    return [Line2D([], [], marker='o', ls='', ms=9, color=Q4, mec='k', label='QUAD4 (= his reference); its 1.13 m line hangs straight down'),
            Line2D([], [], color=Q4, lw=1.4, label='QUAD4 last 3 s'),
            Line2D([], [], marker='X', ls='', ms=9, color='#1a9850', mec='k', label='his target (rendezvous point)'),
            Line2D([], [], marker='^', ls='', ms=9, color=CAR_C[0], mec='k', label='carrier body (measured 12 Nov)'),
            Line2D([], [], color=CAR_C[0], lw=2, label='carrier tether, body to plate (top view)'),
            Line2D([], [], color='#e08214', lw=3, label='closest tether (< 0.5 m)'),
            mpatches.Patch(fc='0.92', ec='k', label='ring (plate radius 0.225 m)'),
            Line2D([], [], color='0.35', ls=':', label='planner ring outer 0.56 m'),
            Line2D([], [], color='k', marker=r'$\rightarrow$', ls='', ms=12, label='ring direction of motion'),
            Line2D([], [], color='0.8', ls='--', label='ring path (whole replay)')]


def shade_fallback(ax, R, tmax):
    for k in range(len(R.bt)):
        if R.fb_active[k] and R.bt[k] < tmax:
            ax.axvspan(R.bt[k], R.bt[k] + 1.0, color='#fdd0a2', alpha=0.6, lw=0, zorder=0)


def vlines(ax, R, ev, label=False):
    for key, col, ls, name in (('auth', 'k', ':', 'authority granted'), ('closest', '#e08214', '--', 'closest tether'),
                               ('cap', '#1a9850', '-', 'captured')):
        if ev.get(key) is not None:
            ax.axvline(ev[key], color=col, ls=ls, lw=1.3, label=name if label else None)


def timeseries(axs, R, tmax, title):
    ev = events(R)
    m = R.t <= tmax
    a = axs[0]
    for s, e in R.still_runs():
        if e - s > 0.5 and s < tmax:
            a.axvspan(s, min(e, tmax), color='#d6604d', alpha=0.12, lw=0)
    ringsp = np.array([np.hypot(*R.scene(t)[1][:2]) for t in R.t[m]])
    a.plot(R.t[m], R.speed[m], color=Q4, lw=1.6, label='QUAD4 speed (his reference)')
    a.plot(R.t[m], ringsp, color='k', lw=1.2, label='ring speed (measured)')
    a.set_ylabel('speed (m/s)')
    a.set_title(title)
    a.legend(loc='upper right', ncol=2, fontsize=8.5)
    a = axs[1]
    a.plot(R.t[m], R.dxy[m], color='#1a9850', lw=1.6, label='QUAD4 to target, horizontal')
    a.plot(R.t[m], R.min_cable_tether_m[m], color='#e08214', lw=1.6, label='closest carrier tether to QUAD4 line')
    a.set_ylabel('distance (m)')
    a.set_ylim(bottom=0)
    a.legend(loc='upper right', fontsize=8.5)
    a = axs[2]
    bm = R.bt <= tmax
    a.step(R.bt[bm], R.commit_left[bm], where='post', color='#2166ac', lw=1.6, label='commit end - now (backend, 1 Hz)')
    a.axhline(0, color='k', lw=0.8)
    a.set_ylabel('committed plan\nleft (s)')
    a.legend(loc='upper right', fontsize=8.5)
    rm = R.rt <= tmax
    cats = np.array([cat_of(s, i) for s, i in zip(R.rstat, R.rinst)])
    a = axs[3]
    for k, (lab, col, _) in enumerate(CATS):
        mm = rm & (cats == k)
        a.plot(R.rt[mm], R.rms[mm], '.', ms=4.5, color=col)
    a.axhline(R.splice_ms, color='k', ls='--', lw=1, label=f'splice lookahead {R.splice_ms:.0f} ms')
    a.set_ylabel('replan runtime\n(ms)')
    a.set_ylim(0, max(260, np.nanpercentile(R.rms[rm], 99) * 1.1))
    a.legend(loc='upper left', fontsize=8.5)
    a = axs[4]
    for k, (lab, col, _) in enumerate(CATS):
        mm = rm & (cats == k)
        a.plot(R.rt[mm], np.full(mm.sum(), k), '|', ms=11, mew=1.6, color=col)
    a.set_yticks(range(len(CATS)))
    a.set_yticklabels(['installed', 'accepted, not installed', 'rejected at final recheck', 'no plan (LOCAL_PLAN_FAILED)',
                       'no plan + incumbent unsafe', 'target time not converged'], fontsize=8.5)
    a.set_ylim(len(CATS) - 0.5, -0.5)
    a.set_xlabel('replay time (s)')
    a.set_title('every replan request, by result', fontsize=9.5)
    for k, a in enumerate(axs):
        shade_fallback(a, R, tmax)
        vlines(a, R, ev)
        a.set_xlim(0, tmax)
        a.grid(alpha=0.3)
    return ev


def ts_legend(fig, y):
    h = [mpatches.Patch(color='#fdd0a2', label='backend execution fallback active (UNCERTIFIED_HARD_HOVER), 1 Hz'),
         mpatches.Patch(color='#d6604d', alpha=0.25, label='QUAD4 still (< 0.05 m/s) before capture'),
         Line2D([], [], color='k', ls=':', label='authority granted'),
         Line2D([], [], color='#e08214', ls='--', label='closest tether before capture'),
         Line2D([], [], color='#1a9850', label='captured')]
    fig.legend(handles=h, loc='lower center', bbox_to_anchor=(0.5, y), ncol=3, fontsize=9, frameon=True)


def frame(R, times, margin=0.55):
    pts = []
    for t in times:
        i = R.idx(t)
        pts.append((R.x[i], R.y[i]))
        pts.append(tuple(R.scene(t)[0][:2]))
    pts = np.array(pts)
    lo, hi = pts.min(0), pts.max(0)
    return tuple((lo + hi) / 2), float(max(hi - lo) / 2 + margin)


def fig_snap(R, times, fname, sup, span=None, centre=None):
    if span is None:
        centre, span = frame(R, times)
    fig, axs = plt.subplots(2, 3, figsize=(15, 12.5))
    for a, t in zip(axs.flat, times):
        snapshot(a, R, t, span=span, centre=centre)
    for a in axs[:, 0]:
        a.set_ylabel('y (m)')
    for a in axs[1, :]:
        a.set_xlabel('x (m)')
    fig.suptitle(sup, fontsize=12)
    fig.legend(handles=legend_handles(), loc='lower center', ncol=4, fontsize=8.8, bbox_to_anchor=(0.5, 0.0))
    fig.subplots_adjust(top=0.86, bottom=0.13, left=0.06, right=0.98, hspace=0.32, wspace=0.22)
    fig.savefig(f'{HERE}/{fname}', dpi=130)
    plt.close(fig)


def fig_combined(R, times, tmax, fname, sup, span=None, centre=None):
    if span is None:
        centre, span = frame(R, times)
    fig = plt.figure(figsize=(15, 21))
    gs = fig.add_gridspec(7, 3, height_ratios=[1.35, 1.35, 0.55, 0.55, 0.45, 0.45, 0.75], hspace=0.42, wspace=0.18)
    for k, t in enumerate(times):
        a = fig.add_subplot(gs[k // 3, k % 3])
        snapshot(a, R, t, span=span, centre=centre)
    axs = [fig.add_subplot(gs[2 + r, :]) for r in range(5)]
    for a in axs[1:]:
        a.sharex(axs[0])
    timeseries(axs, R, tmax, 'time series')
    fig.suptitle(sup, fontsize=12, y=0.995)
    fig.legend(handles=legend_handles(), loc='lower center', ncol=4, fontsize=8.5, bbox_to_anchor=(0.5, 0.012))
    ts_legend(fig, 0.045)
    fig.subplots_adjust(top=0.96, bottom=0.10, left=0.17, right=0.98)
    fig.savefig(f'{HERE}/{fname}', dpi=110)
    plt.close(fig)


def main():
    r2 = Run('nro_mv_z29s20sp3_r2')
    l8 = Run('nro_mv_z29s20sp3_load8')
    ok = Run('nro_mv_z38s20sp3')
    p2 = Run('nro_p02')
    for R in (r2, l8, ok, p2):
        ev = events(R)
        print(R.name, {k: (round(v, 2) if v is not None else None) for k, v in ev.items()},
              'still', [(round(a, 1), round(b, 1)) for a, b in R.still_runs() if b - a > 0.5])

    # fig 1: the zero window of r2, top view
    dl, b, lo, ring, rv, cars = line_tether(r2, 32.6)
    k = int(np.argmin(dl))
    hz = (f'heights at +32.6 s: QUAD4 body {b[2]:.2f} m, its hanging line down to {lo[2]:.2f} m;\nd{k} body {cars[k][0][2]:.2f} m, '
          f'its tether down to its plate at {cars[k][2][2]:.2f} m: the heights overlap, so the top-view crossing is a 3-D crossing')
    print(hz)
    fig_snap(r2, [25.5, 27.5, 29.0, 30.5, 31.8, 32.6], 'fig1_r2_zero_window_topview.png',
             'nro_mv_z29s20sp3_r2 (spline 3, cap 2.9 m, splice 0.20): QUAD4 stops at +27.3 s and the moving ring '
             'sweeps a carrier tether through its hanging line (0.00 m at +32.6 s)\n'
             'top view; open-loop real-time replay of the 12 Nov circling flight (ring ~1.0 m high, 0.1-0.15 m/s)\n' + hz,
             )

    # fig 2: time series, r2 and load8 side by side
    fig = plt.figure(figsize=(16, 13.5))
    gs = fig.add_gridspec(5, 2, height_ratios=[1, 1, 0.8, 0.8, 1.2], hspace=0.42, wspace=0.42)
    for col, (R, tmax, title) in enumerate(((r2, 44, 'nro_mv_z29s20sp3_r2'),
                                            (l8, 46, 'nro_mv_z29s20sp3_load8 (8 busy CPU loops)'))):
        axs = [fig.add_subplot(gs[r, col]) for r in range(5)]
        timeseries(axs, R, tmax, title)
    fig.suptitle('Why QUAD4 stops: near the ring every new plan fails, the committed plan runs out (commit end - now < 0),\n'
                 'the incumbent re-checks unsafe and the backend holds an uncertified hard hover while the tether closes to 0.00 m',
                 fontsize=12)
    ts_legend(fig, 0.0)
    fig.subplots_adjust(top=0.91, bottom=0.095, left=0.15, right=0.99)
    fig.savefig(f'{HERE}/fig2_zero_window_timeseries.png', dpi=120)
    plt.close(fig)

    # fig 3: the successful moving capture
    fig_combined(ok, [5.0, 7.0, 9.0, 11.0, 12.5, 13.3], 18, 'fig3_z38_moving_capture.png',
                 'Contrast: nro_mv_z38s20sp3 (spline 3, cap 3.8 m, splice 0.20) captures the moving ring 11.2 s after '
                 'authority without stopping')

    # fig 4: the parked case
    fig_combined(p2, [36.0, 40.0, 45.0, 52.0, 60.0, 76.2], 90, 'fig4_p02_parked.png',
                 'nro_p02 (spline 4, no z cap, splice 0.10): QUAD4 parks beside the ring\'s path from +39 s to the end '
                 'and never captures')


if __name__ == '__main__':
    main()

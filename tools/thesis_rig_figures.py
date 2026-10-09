#!/usr/bin/env python3
"""tools/thesis_rig_figures.py -- thesis figures for the rig and the 9 Oct reconfiguration safety work.
Built with the thesis-figure skill: print size, Computer Modern, no run ids (they are in
configs/thesis_figures.yaml and docs/figures/PROVENANCE.md).

    python3 tools/thesis_rig_figures.py departed      # one figure
    python3 tools/thesis_rig_figures.py               # all with data

F_departed_landing -- "A UAV landing by itself off the ring stalled above its resting height and hovered
until the fleet's disarm dropped it, on the rig and in the twin; with the departed landing it is pushed to
its resting height and disarmed alone 0.2 s after reaching it."
  hero: QUAD4's height above its own resting height, the same UAV in three panels: rig with the old rule,
  twin with the old rule, twin with the departed landing; its reference dashed until it is disarmed; the
  disarm marked (cross) and the ring's landing (dotted). Aligned where it passes 0.8 m on the way down.
  Left out: the carriers, the ring, the tilt. (Critic 9 Oct: the first version compared rig/old against
  twin/new, two changes at once.)

F_rig_detach -- "On the rig a detach by OCP resize, announced or found unannounced from the stretched rod,
held the ring: peak tilt 6.5-17.8 deg and a few cm of height, and the twin predicted the unannounced
transients within its spread (the announced one tilted about twice the twin's)."
  hero: the ring tilt after the release, rig (payload gold) against the two twin repeats (grey), one column
  per detach; second row: the ring's height error. Left out: the UAVs, the tensions (F_detach shows them).
  Window: 3 s before the release to 10 s after.

F_solve_time -- "In flight on the rig the planner's optimisation finished inside its 100 ms budget at every
tick: median 28 ms with three UAVs, 53-57 ms with four (worst 94 ms)."
  hero: the in-flight solve-time distribution per planner and team size on the rig, the budget dashed.
  Left out: the floor creep (not the flying planner), the twin.

F_land_phases -- "A LAND from any phase of a flight brings every UAV down and disarmed in the twin, within about
14 s, with no UAV tilting past 30 deg, except a LAND within about 2 s of a weld, where the twin's rigid rod can
tip the newcomer."
  hero: the worst UAV tilt from LAND to its disarm per phase (two runs), the 30 deg bar dashed; second row: the
  time from LAND to the last disarm. Failures and voids marked. Data: the re-fly on f330bd2.

F_tip_feedback -- "Steering the measured magnet tip removes a 4 cm error in the assumed rod, either
sign, in about 2 s; the approach that flies the body plus a fixed rod length keeps it."
  hero: tip height minus its setpoint from the tip hold's start, switch on (QUAD4's colour, both
  repeats) against switch off (grey), the +-1 cm band shaded; lower panel: the correction c against
  the injected error (dashed). Left out: the carriers, the ring, xy, the swing, freeze and weld arms
  (in the table). Window: 2 s before the hold to 20 s after it (twin, dry approach).
"""
import csv
import glob
import os
import sys

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import plot_style as ps  # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(REPO, 'docs', 'figures')

# QUAD4 (drone 4, /drone_3) landing by itself: rig r303_4 (freed by its magnet, landed with the fleet at LAND,
# old rule); twin R1257 (taken out by OFF in its hold, landed by itself, old rule); twin R1306 (the same, departed
# landing: pressed to its rest, disarmed alone)
DEPARTED = dict(rig=dict(path='results/rig/2026-10-08/r303_4_run', drone=3),
                old=dict(run='R1257', drone=3),
                new=dict(run='R1306', drone=3))


def run_dir(rid):
    return glob.glob(os.path.join(REPO, 'results', '*', f'{rid}_*'))[0]


def load(spec):
    d = os.path.join(REPO, spec['path']) if 'path' in spec else run_dir(spec['run'])
    run = pd.read_csv(os.path.join(d, 'logs', 'run.csv')).drop_duplicates('t').reset_index(drop=True)
    ev = {}
    for e in csv.DictReader(open(os.path.join(d, 'logs', 'events.csv'))):
        ev.setdefault(e['event'], []).append(float(e['sim_time']))
    return run, ev


def departed_trace(spec, align_m=0.8):
    run, ev = load(spec)
    i = spec['drone']
    t = run.t.to_numpy()
    z = run[f'd{i}_z'].to_numpy()
    zr = run[f'd{i}_ref_z'].to_numpy() if f'd{i}_ref_z' in run else np.full_like(z, np.nan)
    armed = run[f'd{i}_armed'].to_numpy() > 0.5
    rest = float(np.nanmedian(z[t <= t[0] + 2.0]))
    h = z - rest
    t_landed = ev.get('LANDED', [None])[0]
    # t = 0 where it last passes align_m on the way down before it is disarmed (or the table ends)
    flew = np.flatnonzero(h > align_m)
    edge = np.flatnonzero((t[1:] > t[flew[0]]) & armed[:-1] & ~armed[1:]) if len(flew) else []
    t_off = float(t[edge[0] + 1]) if len(edge) else None
    if t_off is None and t_landed is not None and t[-1] >= t_landed - 0.1:
        t_off = t_landed            # the rig table ends at LANDED: the fleet disarm is commanded there
    k_end = int(np.searchsorted(t, t_off)) if t_off is not None else len(t) - 1
    down = np.flatnonzero((h[:-1] > align_m) & (h[1:] <= align_m) & (np.arange(len(t) - 1) < k_end))
    t0 = float(t[down[-1] + 1])
    at_rest = np.flatnonzero((t > t0) & (np.abs(h) <= 0.03))
    return dict(t=t - t0, h=h, href=zr - rest, t_off=None if t_off is None else t_off - t0,
                t_landed=None if t_landed is None else t_landed - t0, rest=rest,
                t_rest=float(t[at_rest[0]] - t0) if len(at_rest) else None)


def fig_departed():
    ps.thesis()
    tr = {k: departed_trace(v) for k, v in DEPARTED.items()}
    q4 = ps.drone_colour(3)
    T1 = 13.0
    fig, axs = plt.subplots(1, 3, figsize=ps.size('full', height=2.3), sharey=True, sharex=True)
    stats = {}
    labels = dict(rig='Rig, old rule', old='Twin, old rule', new='Twin, departed landing')
    for ax, key, letter in zip(axs, ('rig', 'old', 'new'), 'abc'):
        d = tr[key]
        ax.axhline(0.0, color='0.6', lw=0.5, zorder=0)
        t, h, hr = d['t'], d['h'], d['href']
        off = d['t_off'] if d['t_off'] is not None else np.inf
        w = (t >= -1.0) & (t <= min(T1, off))
        wr = w & (t <= off)
        ax.plot(t[wr], hr[wr], color='0.5', lw=0.7, drawstyle='steps-post', zorder=1)
        ax.plot(t[w], h[w], color=q4, lw=1.3, zorder=2)
        if d['t_landed'] is not None and d['t_landed'] <= T1:
            ax.axvline(d['t_landed'], color='0.6', lw=0.5, ls=':', zorder=0)
            ax.annotate('ring down', xy=(d['t_landed'], 1.0), xycoords=('data', 'axes fraction'), xytext=(-2, -2),
                        textcoords='offset points', fontsize=8, color='0.4', ha='right', va='top')
        if d['t_off'] is not None and d['t_off'] <= T1:
            k = int(np.searchsorted(t, d['t_off'])) - 1
            stats[key + '_disarm_h'] = float(h[k])
            ax.plot(d['t_off'], h[k], 'x', color=q4, ms=5.5, mew=1.4, zorder=4)
        else:
            k = int(np.searchsorted(t, d['t_off'])) - 1
            stats[key + '_disarm_h'] = float(h[k])
            stats[key + '_disarm_t'] = d['t_off']
            hold = (t > T1 - 3) & (t <= T1)
            ax.annotate(f"held until the fleet\ndisarm at +{d['t_off']:.0f}\\,s:\ndropped {h[k]:.2f}\\,m",
                        xy=(T1, float(np.mean(h[hold]))), xytext=(-2, 6), textcoords='offset points',
                        fontsize=8, color='0.35', ha='right', va='bottom')
        ax.annotate(labels[key], xy=(1.0, 0.0), xycoords='axes fraction', xytext=(-2, 3),
                    textcoords='offset points', fontsize=8, color='0.2', ha='right', va='bottom')
        ax.set_xlabel('$t$ (s)')
        ps.panel_label(ax, letter)
    rig = tr['rig']
    axs[0].annotate(f"fleet disarm:\ndropped {stats['rig_disarm_h']:.2f}\\,m", xy=(rig['t_off'], stats['rig_disarm_h']),
                    xytext=(6, 0), textcoords='offset points', fontsize=8, color='0.35', ha='left', va='center')
    new = tr['new']
    stats['new_alone_after_rest'] = new['t_off'] - new['t_rest']
    # the 0.2 s wait cannot be read at this scale: an inset
    ins = axs[2].inset_axes([0.42, 0.42, 0.55, 0.42])
    wz = (new['t'] >= new['t_rest'] - 0.6) & (new['t'] <= new['t_off'] + 0.4)
    ins.plot(new['t'][wz], new['h'][wz] * 100, color=q4, lw=1.1)
    ins.axhline(0.0, color='0.6', lw=0.5)
    ins.axvline(new['t_rest'], color='0.5', lw=0.5, ls=':')
    ins.plot(new['t_off'], 0.0, 'x', color=q4, ms=5, mew=1.3)
    ins.annotate('', xy=(new['t_rest'], -0.8), xytext=(new['t_off'], -0.8),
                 arrowprops=dict(arrowstyle='<->', color='0.35', lw=0.6, shrinkA=0, shrinkB=0))
    ins.annotate(f"{stats['new_alone_after_rest']:.1f}\\,s at rest", xy=((new['t_rest'] + new['t_off']) / 2, -1.1),
                 ha='center', va='top', fontsize=8, color='0.35')
    ins.set_ylim(-2.6, 3.6)
    ins.set_yticks([0, 3])
    ins.set_yticklabels(['0', '3\\,cm'])
    ins.tick_params(labelsize=7, length=2)
    ins.set_xticks([])
    for sp in ('top', 'right'):
        ins.spines[sp].set_visible(False)
    axs[2].annotate('disarmed alone', xy=(new['t_off'], 0.0), xytext=(6, -3), textcoords='offset points',
                    fontsize=8, color='0.35', ha='left', va='top')
    axs[0].set_ylabel('Height above its rest (m)')
    axs[0].set_ylim(-0.55, 0.9)
    axs[0].set_yticks([-0.4, 0, 0.4, 0.8])
    axs[0].set_xlim(-1.0, T1)
    stem = os.path.join(OUT, 'F_departed_landing', 'departed_landing')
    ps.save_print(fig, stem)
    return stem, stats


TIP_RUNS = dict(em=dict(off='R1394', on=('R1400', 'R1406'), err=-0.04),
                ep=dict(off='R1395', on=('R1401', 'R1407'), err=+0.04))


def tip_trace(rid):
    d = run_dir(rid)
    f = sorted(glob.glob(os.path.join(d, 'logs', '**', 'tip.csv'), recursive=True))[-1]
    r = pd.read_csv(f)
    hold = r.hold.to_numpy() > 0
    t0 = float(r.t[hold].iloc[0])
    return r.t.to_numpy() - t0, (r.tip_z - r.setpoint_z).to_numpy(), r.c_z.to_numpy()


def fig_tip():
    ps.thesis()
    T1 = 10.0
    fig, axs = plt.subplots(2, 1, figsize=ps.size('full', height=2.9), sharex=True,
                            gridspec_kw=dict(height_ratios=[1.6, 1.0]))
    ax, axc = axs
    q4 = ps.drone_colour(3)
    off_c = '0.15'
    stats = {}
    ls = dict(ep='-', em='--')                      # one line style per sign, in both panels
    ax.axhspan(-1.0, 1.0, color='0.9', lw=0, zorder=0)
    ax.axhline(0.0, color='0.65', lw=0.5, zorder=0)
    for key, spec in TIP_RUNS.items():
        t, e, c = tip_trace(spec['off'])
        w = (t >= 0) & (t <= T1)
        ax.plot(t[w], e[w] * 100, color=off_c, lw=1.0, ls=ls[key], zorder=1)
        for rid in spec['on']:
            t, e, c = tip_trace(rid)
            w = (t >= 0) & (t <= T1)
            ax.plot(t[w], e[w] * 100, color=q4, lw=1.1, ls=ls[key], zorder=2)
            axc.plot(t[w], c[w] * 100, color=q4, lw=1.1, ls=ls[key], zorder=2)
            inside = np.flatnonzero((t >= 0) & (np.abs(e) < 0.01))
            if len(inside):
                k = inside[0]
                stats[f'{key}_{rid}_conv'] = float(t[k])
                ax.plot(t[k], e[k] * 100, 'o', color=q4, ms=3.2, mec='white', mew=0.5, zorder=3)
            stats[f'{key}_{rid}_end_mm'] = float(np.mean(np.abs(e[(t > T1 - 3) & (t <= T1)])) * 1000)
        axc.axhline(-spec['err'] * 100, color='0.15', lw=0.6, ls=(0, (4, 2)), zorder=3)
    ax.annotate('loop off, rod assumed 4\\,cm long', xy=(T1, 4.1), xytext=(0, 4), textcoords='offset points',
                fontsize=8, color=off_c, ha='right', va='bottom')
    ax.annotate('loop off, rod assumed 4\\,cm short', xy=(T1, -4.1), xytext=(0, -4), textcoords='offset points',
                fontsize=8, color=off_c, ha='right', va='top')
    ax.annotate('loop on, two runs per sign', xy=(4.2, 1.35), fontsize=8, color=q4, ha='left', va='bottom')
    ax.annotate('$\\pm$1\\,cm', xy=(T1, 1.0), xytext=(0, 1), textcoords='offset points', fontsize=8,
                color='0.45', ha='right', va='bottom')
    axc.annotate('the injected rod error to recover (dashed black)', xy=(T1, -4.0), xytext=(0, -3),
                 textcoords='offset points', fontsize=8, color='0.3', ha='right', va='top')
    ax.set_ylabel('Tip height error (cm)')
    ax.set_ylim(-6.5, 6.5)
    ax.set_yticks([-4, -2, 0, 2, 4])
    axc.set_ylabel('Correction $c$ (cm)')
    axc.set_ylim(-7.5, 6.0)
    axc.set_yticks([-4, 0, 4])
    axc.set_xlabel('$t$ after the still hold over the plate starts (s)')
    axc.set_xlim(0, T1)
    axc.set_xticks([0, 2, 4, 6, 8, 10])
    axc.set_xticks(np.arange(0, T1 + 0.01, 1.0), minor=True)
    ps.panel_label(ax, 'a')
    ps.panel_label(axc, 'b')
    fig.align_ylabels()
    stem = os.path.join(OUT, 'F_tip_feedback', 'tip_feedback')
    ps.save_print(fig, stem)
    return stem, stats


RIG_DETACH = [
    ('Announced, even three left', dict(path='results/rig/2026-10-08/r302_3_run'), ('R1207', 'R1212'), 'DETACH'),
    ('Unannounced, even three left', dict(path='results/rig/2026-10-08/r303_4_run'), ('R1140', 'R1145'), 'RELEASE'),
    ('Unannounced, 150$^\\circ$ gap left', dict(path='results/rig/2026-10-08/r304_3_run'), ('R1206', 'R1211'), 'RELEASE'),
]
DETACH_T0, DETACH_T1 = -2.0, 7.5


def detach_trace(spec, event):
    run, ev = load(spec)
    t0 = ev[event][0]
    t = run.t.to_numpy() - t0
    return t, run.payload_tilt_deg.to_numpy(), (run.payload_z - run.payload_ref_z).to_numpy(), ev


def _rise(t, y):
    pre = (t >= -3.0) & (t < 0.0)
    post = (t >= 0.0) & (t <= 3.0)
    base = float(np.nanmean(y[pre]))
    k = int(np.flatnonzero(post)[np.nanargmax(y[post])])
    return base, float(y[k]), float(t[k])


def fig_rig_detach():
    ps.thesis()
    fig, axs = plt.subplots(2, 3, figsize=ps.size('full', height=3.0), sharex=True, sharey='row',
                            gridspec_kw=dict(height_ratios=[1.4, 1.0]))
    stats = {}
    gold = ps.PAYLOAD_LINE
    for col, (title, rig, twins, event) in enumerate(RIG_DETACH):
        ax, axz = axs[0, col], axs[1, col]
        t, tilt, dz, ev = detach_trace(rig, event)
        t_land = ev['LAND'][0] - ev[event][0] if 'LAND' in ev else 99.0
        t1 = min(DETACH_T1, t_land)
        tw = []
        for rid in twins:
            tt, ti, dzz, _ = detach_trace(dict(run=rid), event)
            tw.append((tt, ti, dzz))
            stats[f'{col}_{rid}'] = dict(zip(('base', 'peak', 't_peak'), _rise(tt, ti)), dz_min=float(np.nanmin(dzz[(tt >= 0) & (tt <= 3)])))
        grid = np.arange(DETACH_T0, t1, 0.02)
        for yi, a in ((1, ax), (2, axz)):
            ys = np.array([np.interp(grid, tr[0], tr[yi] * (100 if yi == 2 else 1)) for tr in tw])
            a.fill_between(grid, ys.min(0), ys.max(0), color='0.7', lw=0, zorder=1)
            a.plot(grid, ys.mean(0), color='0.5', lw=0.7, zorder=1)
        w = (t >= DETACH_T0) & (t <= t1)
        ax.plot(t[w], tilt[w], color=gold, lw=1.1, zorder=2)
        axz.plot(t[w], dz[w] * 100, color=gold, lw=1.1, zorder=2)
        base, peak, tp = _rise(t, tilt)
        post = (t >= 0) & (t <= 3)
        stats[f'{col}_rig'] = dict(base=base, peak=peak, rise=peak - base, dz_min=float(np.nanmin(dz[post])))
        tw_rise = [stats[f'{col}_{r}']['peak'] - stats[f'{col}_{r}']['base'] for r in twins]
        stats[f'{col}_twin_rise'] = (min(tw_rise), max(tw_rise))
        ax.plot(tp, peak, 'o', color=gold, ms=3, zorder=3)
        lo, hi = round(min(tw_rise)), round(max(tw_rise))
        twin_txt = f'+{lo}' if lo == hi else f'+{lo}--{hi}'
        ax.annotate(f'rise: rig +{peak - base:.0f}$^\\circ$,\ntwin {twin_txt}$^\\circ$', xy=(tp, peak),
                    xytext=((0.30, 0.93), (0.45, 0.70), (0.30, 0.93))[col], textcoords='axes fraction', fontsize=8,
                    color='0.25', ha='left', va='top',
                    arrowprops=dict(arrowstyle='-', color='0.5', lw=0.5, shrinkA=1, shrinkB=2))
        for a in (ax, axz):
            a.axvline(0.0, color='0.2', lw=0.6, ls=':', zorder=0)
        if 'DETECTED' in ev and event == 'RELEASE':
            td = ev['DETECTED'][0] - ev[event][0]
            stats[f'{col}_detect_s'] = td
            for a in (ax, axz):
                a.axvline(td, color='0.2', lw=0.6, ls=(0, (1, 2)), zorder=0)
        axz.axhline(0.0, color='0.75', lw=0.5, zorder=0)
        ax.annotate(f"({'abc'[col]}) {title}", xy=(0, 1), xycoords='axes fraction', xytext=(0, 4),
                    textcoords='offset points', fontsize=8, ha='left', va='bottom')
        axz.annotate(f"({'def'[col]})", xy=(0, 1), xycoords='axes fraction', xytext=(0, 4),
                     textcoords='offset points', fontsize=8, ha='left', va='bottom')
    axs[0, 1].annotate('release', xy=(0.0, 19.5), xytext=(-2, 0), textcoords='offset points', fontsize=8,
                       color='0.3', ha='right', va='top')
    td = stats['1_detect_s']
    axs[0, 1].annotate(f'found +{td:.2f}\\,s', xy=(td, 19.5), xytext=(2, 0), textcoords='offset points',
                       fontsize=8, color='0.3', ha='left', va='top')
    axs[0, 0].set_ylabel(r'Ring tilt (\si{\degree})')
    axs[0, 0].set_ylim(0, 20)
    axs[0, 0].set_yticks([0, 5, 10, 15, 20])
    axs[1, 0].set_ylabel('Ring height error (cm)')
    axs[1, 0].set_ylim(-8, 4)
    axs[1, 0].set_yticks([-8, -4, 0, 4])
    axs[1, 0].set_xlim(DETACH_T0, DETACH_T1)
    axs[1, 0].set_xticks([-2, 0, 2, 4, 6])
    axs[0, 2].annotate('rig, one run', xy=(7.3, 12.5), fontsize=8, color=gold, ha='right', va='bottom')
    axs[0, 2].annotate('twin, two runs', xy=(7.3, 10.3), fontsize=8, color='0.45', ha='right', va='bottom')
    fig.supxlabel('$t$ after the release (s)', fontsize=9)
    fig.align_ylabels()
    stem = os.path.join(OUT, 'F_rig_detach', 'rig_detach')
    ps.save_print(fig, stem)
    return stem, stats


def rig_solve_times():
    out = {}
    for f in glob.glob(os.path.join(REPO, 'results', 'rig', '2026-10-0[78]', '*_logs', 'logs', '*planner*', '*', 'log.csv')):
        planner = 'dissipative' if 'dissipative' in f else 'mpc'
        try:
            L = pd.read_csv(f, usecols=['phase', 'n', 'solve_ms'])
        except Exception:
            continue
        L = L[L.phase.isin(['planner', 'network']) & np.isfinite(L.solve_ms) & (L.solve_ms > 0)]
        for n, g in L.groupby('n'):
            out.setdefault((planner, int(n)), []).append(g.solve_ms.to_numpy())
    return {k: np.concatenate(v) for k, v in out.items()}


def fig_solve():
    ps.thesis()
    data = rig_solve_times()
    order = [('dissipative', 3), ('dissipative', 4), ('mpc', 4)]
    names = {('dissipative', 3): 'three UAVs', ('dissipative', 4): 'four UAVs', ('mpc', 4): 'four UAVs,\nmpc mode'}
    fig, ax = plt.subplots(figsize=ps.size('half', height=2.0))
    stats = {}
    vals = [data[k] for k in order]
    bp = ax.boxplot(vals, vert=False, widths=0.5, whis=(1, 99), showfliers=False, patch_artist=True,
                    medianprops=dict(color=ps.PAYLOAD_LINE, lw=1.3), boxprops=dict(facecolor='0.92', color='0.3', lw=0.6),
                    whiskerprops=dict(color='0.3', lw=0.6), capprops=dict(color='0.3', lw=0.6))
    for i, (k, v) in enumerate(zip(order, vals), start=1):
        stats[f'{k[0]}_{k[1]}'] = dict(n_ticks=len(v), p50=float(np.median(v)), p99=float(np.percentile(v, 99)),
                                      max=float(v.max()), over=int((v > 100).sum()))
        ax.plot(v.max(), i, '|', color='0.3', ms=6, mew=0.8)
    ax.axvline(100.0, color='0.3', lw=0.8, ls='--')
    ax.annotate('100\\,ms budget', xy=(100.0, 0.55), xytext=(-2, 0), textcoords='offset points', fontsize=8, color='0.3',
                ha='right', va='center')
    ax.set_yticks([1, 2, 3])
    ax.set_yticklabels([names[k] for k in order], fontsize=8)
    ax.set_xlabel('Solve time in flight (ms)')
    ax.set_xlim(0, 110)
    ax.set_xticks([0, 25, 50, 75, 100])
    ax.invert_yaxis()
    stem = os.path.join(OUT, 'F_solve_time', 'solve_time')
    ps.save_print(fig, stem)
    return stem, stats


LAND_PHASES = [('twin_land_e0_prearm', 'armed on\nthe floor'), ('twin_land_e1_creep', 'creep'), ('twin_land_e2_lift', 'lift'),
               ('twin_land_e3_parked', 'newcomer\nparked'), ('twin_land_e4_climb', 'climb'), ('twin_land_e5_transit', 'transit'),
               ('twin_land_e6_descend', 'descent'), ('twin_all_a1', 'hold over\nthe plate'),
               ('twin_land_e9_welded', 'weld\n+0.5 s'), ('twin_land_e9b_welded2', 'weld\n+2 s'),
               ('twin_land_e9c_welded4', 'weld\n+4 s'), ('w4_land_e11_stepout', 'freed UAV\nstepping out'),
               ('w4_land_e12_landing', 'freed UAV\nlanding'), ('w4_land_e13_drop', 'after\na drop'),
               ('twin_land_e14_twice', 'LAND\ntwice'), ('twin_land_e15_orbit_hold', 'circle,\nhold'),
               ('twin_land_e16_r8a0', 'carry,\nnewcomer idle')]
LAND_FIRST_RUN = 1413          # the re-fly on f330bd2 (9 Oct evening)


def land_runs(cfg):
    out = []
    for d in sorted(glob.glob(os.path.join(REPO, 'results', '*', f'R*_sim_gz_{cfg}'))):
        rid = os.path.basename(d).split('_')[0]
        if int(rid[1:]) >= LAND_FIRST_RUN and os.path.exists(os.path.join(d, 'logs', 'run.csv')):
            out.append(rid)
    return out


def fig_land_phases():
    sys.path.insert(0, os.path.join(REPO, 'tools'))
    import land_metrics as lm
    ps.thesis()
    fig, axs = plt.subplots(2, 1, figsize=ps.size('full', height=3.2), sharex=True,
                            gridspec_kw=dict(height_ratios=[1.3, 1.0]))
    ax, axt = axs
    stats = {}
    for x, (cfg, name) in enumerate(LAND_PHASES):
        for k, rid in enumerate(land_runs(cfg)[:2]):
            m = lm.analyse(rid)
            tilt = max(v['tilt_max'] for v in m['drones'].values())
            ok = not m['abort'] and not m['armed_at_end'] and tilt <= 30.0
            xx = x + (-0.12 if k == 0 else 0.12)
            col = '0.25' if ok else ps.drone_colour(3)
            ax.plot(xx, min(tilt, 95.0), 'o' if ok else 'X', color=col, ms=4 if ok else 6, zorder=3)
            disarms = [v for v in m['drones'].values()]
            if m['landed_t'] is not None and m['land_t'] is not None:
                axt.plot(xx, m['landed_t'] - m['land_t'], 'o', color=col, ms=4, zorder=3)
            stats[rid] = dict(cfg=cfg, tilt=tilt, ok=ok, land_to_landed=m.get('land_to_landed_s'), abort=m['abort'])
    ax.axhline(30.0, color='0.3', lw=0.8, ls='--')
    ax.annotate('30$^\\circ$', xy=(len(LAND_PHASES) - 0.5, 30.0), xytext=(0, 2), textcoords='offset points',
                fontsize=8, color='0.3', ha='right', va='bottom')
    ax.set_ylabel(r'Worst UAV tilt (\si{\degree})')
    ax.set_ylim(0, 100)
    ax.set_yticks([0, 30, 60, 90])
    axt.set_ylabel('LAND to landed (s)')
    axt.set_ylim(0, 20)
    axt.set_yticks([0, 10, 20])
    axt.set_xticks(range(len(LAND_PHASES)))
    axt.set_xticklabels([n for _, n in LAND_PHASES], fontsize=7, rotation=0)
    axt.set_xlim(-0.6, len(LAND_PHASES) - 0.4)
    ps.panel_label(ax, 'a')
    ps.panel_label(axt, 'b')
    fig.align_ylabels()
    stem = os.path.join(OUT, 'F_land_phases', 'land_phases')
    ps.save_print(fig, stem)
    return stem, stats


FIGS = {'land': fig_land_phases, 'solve': fig_solve, 'departed': fig_departed, 'tip': fig_tip, 'rig_detach': fig_rig_detach}

if __name__ == '__main__':
    names = sys.argv[1:] or list(FIGS)
    for n in names:
        stem, stats = FIGS[n]()
        print(stem, {k: round(v, 3) if isinstance(v, float) else v for k, v in stats.items()})

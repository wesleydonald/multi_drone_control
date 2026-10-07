#!/usr/bin/env python3
"""tools/thesis_sim_figures.py -- the simulation-results figures for the controller chapter
(thesis/Source/src/sim_results.tex). Built with the thesis-figure skill: print size, Computer
Modern, no run ids (they are in configs/thesis_figures.yaml and docs/figures/PROVENANCE.md).

    python3 tools/thesis_sim_figures.py            # both figures into docs/figures/

F_tracker_x0 -- "Starting each tracker's MPC from the measured state removes the ring's hover
offset and most of its orbit error."
  hero: the two error traces with the box at zero (payload gold) against the old box (grey);
  left out: drones, tilt, the lift and LAND; windows: the hold and the first lap (Gazebo rig twin).

F_disturbance -- "With the integral in the planner's model a constant force on the ring is
removed in a few seconds, and the force it learns equals the force applied."
  hero: the ring's displacement along the horizontal push, model form (gold); context: no
  integral (light grey); the vertical push and the target-shift form are left to the table; lower
  panel: the change of the integral's force against the applied force (dashed). Window: 5 s before
  the push to 10 s after it ends.
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
import matplotlib.ticker  # noqa: E402,F401

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import plot_style as ps  # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(REPO, 'docs', 'figures')
GREY, LIGHT = '0.35', '0.45'
LAP_S = 27.2

X0_RUNS = dict(hold=('R0841', 'R0960'), orbit=('R0832', 'R0974'))
DIST_RUNS = dict(z=dict(none='R0962', ref='R0968', model='R0969'),
                 x=dict(none='R0961', model='R0971'))


def run_dir(rid):
    return glob.glob(os.path.join(REPO, 'results', '*', f'{rid}_*'))[0]


def events(rid):
    ev = {}
    for e in csv.DictReader(open(os.path.join(run_dir(rid), 'logs', 'events.csv'))):
        ev.setdefault(e['event'], []).append((float(e['sim_time']), e['arg']))
    return ev


def run_csv(rid):
    return pd.read_csv(os.path.join(run_dir(rid), 'logs', 'run.csv')).drop_duplicates('t')


def planner_log(rid):
    d = run_dir(rid)
    f = (glob.glob(os.path.join(d, 'logs', '*planner', '*', 'log.csv'))
         or glob.glob(os.path.join(d, 'logs', '*', 'load_planner_*', 'log.csv')))
    L = pd.read_csv(f[0])
    run = run_csv(rid)
    off = float(L.sim_time[L.load_z > 0.3].iloc[0]) - float(run.t[run.payload_z > 0.3].iloc[0])
    L['t'] = L.sim_time - off                     # on the run.csv clock
    return L


def gz_hold(rid):
    ev = events(rid)
    run = run_csv(rid)
    t0, t1 = ev['LIFTED'][0][0] + 10.0, ev['LAND'][0][0]       # after the post-lift settle
    w = run[(run.t >= t0) & (run.t < t1)]
    return w.t.to_numpy() - t0, 100 * (w.payload_z - w.payload_ref_z).to_numpy()


def gz_lap(rid):
    run = run_csv(rid)
    L = planner_log(rid)
    start = float(L.t[L.traj_t > 0].iloc[0])
    w = run[(run.t >= start) & (run.t < start + LAP_S)]
    # the reference is a 10 Hz hold: compare where it updates, not against a stale step
    w = w[(w.payload_ref_x.diff().abs() > 0) | (w.payload_ref_y.diff().abs() > 0)]
    e = np.hypot(w.payload_x - w.payload_ref_x, w.payload_y - w.payload_ref_y)
    return w.t.to_numpy() - start, 100 * e.to_numpy()


def fig_tracker_x0():
    ps.thesis()
    fig, axs = plt.subplots(1, 2, figsize=ps.size('full', height=2.1))
    panels = [(gz_hold, 'hold', r'Height error $e_z$ (cm)', r'$t$ (s)'),
              (gz_lap, 'orbit', r'Horizontal error (cm)', r'$t$ (s)')]
    stats = {}
    for ax, (fn, key, ylab, xlab), letter in zip(axs, panels, 'ab'):
        old, new = X0_RUNS[key]
        to, eo = fn(old)
        tn, en = fn(new)
        ax.plot(to, eo, color=LIGHT, lw=0.9)
        ax.plot(tn, en, color=ps.PAYLOAD_LINE, lw=1.4 if key != 'orbit' else 1.1)
        if key != 'orbit':
            ax.axhline(0.0, color='0.7', lw=0.5, ls='--', zorder=0)
        mo, mn = float(np.mean(eo)), float(np.mean(en))
        stats[key] = (mo, mn)
        if key == 'hold':               # the colour key, once
            # the hover heaves by 1 cm: label each trace in the clear band between the two
            for t, y, c, lab, dy in ((to, float(np.min(eo)), LIGHT, r'$\epsilon=0.025$', -4),
                                     (tn, float(np.max(en)), ps.PAYLOAD_LINE, r'$\epsilon=0$', 4)):
                ax.annotate(lab, xy=(15.0, y), xytext=(0, dy), textcoords='offset points',
                            ha='center', va='bottom' if dy > 0 else 'top', fontsize=8,
                            color='0.25' if c == LIGHT else c)
        ax.set_xlabel(xlab)
        ax.set_ylabel(ylab)
        if key == 'orbit':
            ax.set_xlim(0, LAP_S)
            ax.set_xticks([0, 10, 20])
            ax.set_ylim(0, 5)
            ax.set_yticks([0, 1, 2, 3, 4, 5])
        else:
            ax.set_xlim(0, 30)
            ax.set_xticks([0, 10, 20, 30])
            ax.set_ylim(-1, 7)
            ax.set_yticks([0, 2, 4, 6])
        ps.panel_label(ax, letter)
    fig.get_layout_engine().set(wspace=0.08)
    fig.align_ylabels()
    stem = os.path.join(OUT, 'F_tracker_x0', 'tracker_x0')
    ps.save_print(fig, stem)
    return stem, stats


def push_window(rid, axis):
    ev = events(rid)
    (on, arg), (off, _) = ev['WRENCH'][0], ev['WRENCH'][1]
    f = float(arg.split()['xyz'.index(axis)])
    run = run_csv(rid)
    col = 'payload_' + axis
    t = run.t.to_numpy()
    p = run[col].to_numpy()
    base = p[(t >= on - 5) & (t < on)].mean()
    w = (t >= on - 5) & (t < off + 10)
    y = np.sign(f) * (p - base) * 100.0           # cm along the push
    return t[w] - on, y[w], on, off, f


def int_force(rid, axis, on):
    L = planner_log(rid)
    c = f'int_f{axis}'
    if c not in L or L[c].isna().all():
        return None
    t = L.t.to_numpy()
    d = L[c].to_numpy()
    pre = np.nanmean(d[(t >= on - 5) & (t < on)])
    return t - on, d - pre


def fig_disturbance(pushes=('x',)):
    ps.thesis()
    n = len(pushes)
    fig, axs = plt.subplots(2, n, figsize=ps.size('full', height=3.4 if n > 1 else 3.0), sharex='col',
                            squeeze=False, gridspec_kw=dict(height_ratios=[1.7, 1.0]))
    stats = {}
    for col, axis in enumerate(pushes):
        runs = DIST_RUNS[axis]
        ax, axf = axs[0, col], axs[1, col]
        t_off = None
        for k in ('none', 'model'):
            t, y, on, off, f = push_window(runs[k], axis)
            t_off = off - on
            st = dict(none=dict(color=LIGHT, lw=0.9, zorder=2), model=dict(color=ps.PAYLOAD_LINE, lw=1.6, zorder=3))[k]
            ax.plot(t, y, **st)
            stats[(axis, k)] = (float(np.max(y[(t >= 0) & (t < t_off)])), abs(f))
            if k == 'none':
                i = np.searchsorted(t, t_off / 2)
                ax.annotate('no integral', xy=(t[i], y[i]), xytext=(0, 5), textcoords='offset points',
                            ha='center', fontsize=8, color='0.25')
            else:
                i = np.searchsorted(t, 4.0)
                ax.annotate('model integral', xy=(t[i], y[i]), xytext=(6, 2), textcoords='offset points',
                            ha='left', va='bottom', fontsize=8, color=ps.PAYLOAD_LINE)
        T = t_off
        for a in (ax, axf):
            for tt in (0.0, T):
                a.axvline(tt, color='0.75', lw=0.4, zorder=0)
        for tt, lab in ((0.0, 'push on'), (T, 'push off')):
            ax.annotate(lab, xy=(tt, 1.0), xycoords=('data', 'axes fraction'), xytext=(2, 1),
                        textcoords='offset points', fontsize=8, color='0.35', va='bottom')
        ax.axhspan(-1.0, 1.0, color='0.5', alpha=0.12, lw=0, zorder=0)
        ax.axhline(0.0, color='0.6', lw=0.5, zorder=1)
        # time to come back within 1 cm and stay there (model form), anchored on the re-entry
        tm, ym, *_ = push_window(runs['model'], axis)
        out = np.flatnonzero((tm >= 0) & (tm < T) & (np.abs(ym) > 1.0))
        k_in = out[-1] + 1
        ts = float(tm[k_in])
        stats[(axis, 'settle')] = ts
        ax.plot([ts], [ym[k_in]], 'o', color=ps.PAYLOAD_LINE, ms=3, zorder=5)
        ax.plot([ts, ts], [ym[k_in], -3.0], color=ps.PAYLOAD_LINE, lw=0.5, ls=':', zorder=1)
        ax.annotate('', xy=(0, -3.0), xytext=(ts, -3.0),
                    arrowprops=dict(arrowstyle='<->', color=ps.PAYLOAD_LINE, lw=0.7, shrinkA=0, shrinkB=0))
        ax.annotate(f'{ts:.1f}\\,s', xy=(ts / 2, -3.0), xytext=(0, -3), textcoords='offset points',
                    ha='center', va='top', fontsize=8, color=ps.PAYLOAD_LINE)
        ax.set_ylabel(r'Displacement (cm)' if col == 0 else '')
        ax.set_ylim(-8, 10)
        ax.set_yticks([-5, 0, 5, 10])
        tf, d = int_force(runs['model'], axis, on)
        d = np.sign(f) * d                               # along the push, as the row above
        F = abs(f)
        m = (tf >= -5) & (tf < T + 10)
        axf.plot(tf[m], d[m], color=ps.PAYLOAD_LINE, lw=1.6, zorder=3)
        axf.plot([-5, 0, 0, T, T, T + 10], [0, 0, F, F, 0, 0], color='0.4', lw=0.8, ls='--', zorder=2)
        axf.annotate('applied', xy=(T * 0.3, F), xytext=(0, 4), textcoords='offset points',
                     ha='center', va='bottom', fontsize=8, color='0.35')
        late = (tf >= T - 3) & (tf < T)
        mean = float(np.mean(d[late]))
        stats[(axis, 'learned')] = mean
        axf.plot([T - 3, T], [mean, mean], color='0.1', lw=1.2, solid_capstyle='butt', zorder=4)
        axf.annotate(f'${mean:.2f}$\\,N', xy=(T - 3.5, mean), xytext=(0, -5), textcoords='offset points',
                     ha='center', va='top', fontsize=8, color='0.1')
        axf.axhline(0.0, color='0.6', lw=0.5, zorder=1)
        axf.set_ylabel(r'Force (N)' if col == 0 else '')
        axf.set_ylim(-0.1, 0.8)
        axf.set_yticks([0, 0.2, 0.4, 0.6])
        axf.yaxis.set_major_formatter(matplotlib.ticker.FuncFormatter(lambda v, _: f'${v:g}$'))
        axf.set_xlabel(r'$t$ (s)')
        axf.set_xlim(-5, T + 10)
        axf.set_xticks([-5, 0, 5, 10, 15, 20, 25])
        ps.panel_label(ax, 'abcd'[col])
        ps.panel_label(axf, 'abcd'[n + col])
    fig.align_ylabels()
    stem = os.path.join(OUT, 'F_disturbance', 'disturbance')
    ps.save_print(fig, stem)
    return stem, stats


LAND_RUNS = dict(before='R0960', after='R1013')
LAND_LABEL = dict(before='straight descent', after='unwind, ramp, idle')


def signed_lean(rid, i, cx, cy, run):
    """UAV i's thrust-axis lean along its outward radial (deg, + outward), from its tracker log
    (50 Hz) on the run.csv clock (aligned on the drone's climb through rest + 0.3 m)."""
    d = run_dir(rid)
    T = pd.read_csv(sorted(glob.glob(os.path.join(d, 'logs', 'tracker', f'drone{i}_*', 'log.csv')))[0])
    z0 = float(run[f'd{i}_z'].iloc[:20].median())
    off = float(run.t[run[f'd{i}_z'] > z0 + 0.3].iloc[0]) - float(T.sim_time[T.pose_z > z0 + 0.3].iloc[0])
    qw, qx, qy, qz = (T[f'pose_q{c}'].to_numpy() for c in 'wxyz')
    b3 = np.stack([2 * (qx * qz + qw * qy), 2 * (qy * qz - qw * qx), 1 - 2 * (qx ** 2 + qy ** 2)], axis=1)
    u = np.stack([T.pose_x.to_numpy() - cx, T.pose_y.to_numpy() - cy], axis=1)
    u /= np.maximum(np.linalg.norm(u, axis=1, keepdims=True), 1e-9)
    lean = np.degrees(np.arctan2(np.sum(b3[:, :2] * u, axis=1), b3[:, 2]))
    return T.sim_time.to_numpy() + off, lean


def land_window(rid):
    """From the payload touching down to LANDED: per UAV, radial offset from its reference about
    the ring centre (cm, + outside; sampled at reference updates, masked below 0.25 m where the
    reference follows the UAV) and signed lean; t from touchdown."""
    ev = events(rid)
    run = run_csv(rid)
    n = len([c for c in run.columns if c.endswith('_tilt_deg') and c.startswith('d')])
    z0 = float(run.payload_z.iloc[:20].median())
    after = run[run.t >= ev['LAND'][0][0]]
    t_down = float(after.t[after.payload_z <= z0 + 0.05].iloc[0])
    t_end = ev['LANDED'][0][0] if 'LANDED' in ev else float(run.t.iloc[-1])
    w = run[(run.t >= t_down) & (run.t <= t_end)]
    cx, cy = float(w.payload_x.iloc[0]), float(w.payload_y.iloc[0])
    t = w.t.to_numpy() - t_down
    off, lean = [], []
    for i in range(n):
        o = np.where(w[f'd{i}_z'].to_numpy() >= 0.25,
                     100 * (np.hypot(w[f'd{i}_x'] - cx, w[f'd{i}_y'] - cy)
                            - np.hypot(w[f'd{i}_ref_x'] - cx, w[f'd{i}_ref_y'] - cy)).to_numpy(), np.nan)
        upd = ((w[f'd{i}_ref_x'].diff().abs() > 1e-6) | (w[f'd{i}_ref_y'].diff().abs() > 1e-6)
               | (w[f'd{i}_ref_z'].diff().abs() > 1e-6)).to_numpy()
        upd[0] = True
        off.append((t[upd], o[upd]))
        tl, ln = signed_lean(rid, i, cx, cy, run)
        m = (tl >= t_down) & (tl <= t_end)
        lean.append((tl[m] - t_down, ln[m]))
    return off, lean, t_end - t_down


def fig_landing():
    ps.thesis()
    fig, axs = plt.subplots(2, 2, figsize=ps.size('full', height=3.1), sharex=True, sharey='row')
    stats = {}
    for col, key in enumerate(('before', 'after')):
        off, lean, t_landed = land_window(LAND_RUNS[key])
        ax, axt = axs[0, col], axs[1, col]
        colour = '0.3' if key == 'before' else ps.PAYLOAD_LINE
        for to, o in off:
            ok = np.isfinite(o)
            ax.plot(to, o, color=colour, lw=1.1)
            if ok.any():
                k = np.flatnonzero(ok)[-1]
                ax.plot(to[k], o[k], 'o', color=colour, ms=2.5)
        for tl, ln in lean:
            axt.plot(tl, ln, color=colour, lw=1.0)
        stats[(key, 'outside')] = max(float(np.nanmax(o)) for _, o in off)
        stats[(key, 'lean_after_1s')] = max(float(np.max(np.abs(ln[tl >= 1.0]))) for tl, ln in lean)
        ax.axhline(0.0, color='0.6', lw=0.5, zorder=0)
        for b in (-2.0, 2.0):
            axt.axhline(b, color='0.6', lw=0.5, ls=':', zorder=0)
        axt.axhline(0.0, color='0.6', lw=0.5, zorder=0)
        if key == 'after':
            for a in (ax, axt):
                a.axvline(1.0, color='0.75', lw=0.4, zorder=0)
            ax.annotate('ramp ends', xy=(1.0, 1.0), xycoords=('data', 'axes fraction'), xytext=(2, 1),
                        textcoords='offset points', fontsize=8, color='0.35', va='bottom')
        axt.axvline(t_landed, color='0.75', lw=0.4, zorder=0)
        axt.annotate('landed', xy=(t_landed, 1.0), xycoords=('data', 'axes fraction'), xytext=(-2, -2),
                     textcoords='offset points', fontsize=8, color='0.35', va='top', ha='right')
        ax.annotate(LAND_LABEL[key], xy=(1.0, 1.0), xycoords='axes fraction', xytext=(-2, -2),
                    textcoords='offset points', fontsize=9, color=colour, ha='right', va='top')
        ends = [(to[np.flatnonzero(np.isfinite(o))[-1]], o[np.flatnonzero(np.isfinite(o))[-1]]) for to, o in off]
        te, oe = max(ends)
        ax.annotate('UAV below 0.25\\,m', xy=(te, oe), xytext=(6, 0), textcoords='offset points',
                    fontsize=8, color='0.4', ha='left', va='center')
        ax.set_xlim(0, 4.5)
        ax.set_ylim(-25, 25)
        ax.set_yticks([-20, -10, 0, 10, 20])
        axt.set_ylim(-10, 20)
        axt.set_yticks([-10, 0, 10, 20])
        axt.set_xlabel(r'$t$ after touchdown (s)')
        ps.panel_label(ax, 'ab'[col])
        ps.panel_label(axt, 'cd'[col])
    axs[0, 0].set_ylabel('Radial offset, + out (cm)')
    axs[1, 0].set_ylabel(r'Lean, + out (\si{\degree})')
    fig.align_ylabels()
    stem = os.path.join(OUT, 'F_landing', 'landing')
    ps.save_print(fig, stem)
    return stem, stats


DETACH_RUN = 'R1012'
DETACH_PLATES = {0: 1, 1: 3, 2: 5, 3: 9}        # w4_3915: drone i on plate 1/3/5/9 (30/90/150/270 deg)


def fig_detach():
    """F_detach -- "Releasing one UAV of four by OCP resize hands its load to the three survivors in
    one planner tick: the uneven tensions of the 1/3/5/9 layout become the equal tensions of the even
    triangle, and the ring stays within a few degrees of level."
    hero: the planned tensions per UAV (departing UAV in its drone colour, survivors dark grey);
    support: ring tilt and height. Window: 5 s before DETACH to 12 s after."""
    ps.thesis()
    ev = events(DETACH_RUN)
    t_det = ev['DETACH'][0][0]
    gone = int(ev['DETACH'][0][1])
    run = run_csv(DETACH_RUN)
    L = planner_log(DETACH_RUN)
    L = L[(L.phase == 'planner') & (L.t >= t_det - 5) & (L.t <= t_det + 12)]
    fig, axs = plt.subplots(3, 1, figsize=ps.size('full', height=3.6), sharex=True,
                            gridspec_kw=dict(height_ratios=[1.6, 1.0, 1.0]))
    ax, axt, axz = axs
    t = L.t.to_numpy() - t_det
    # planned tension per DRONE through the slot map (slots are renumbered at the resize)
    ten = {d: np.full(len(L), np.nan) for d in DETACH_PLATES}
    for k, (_, row) in enumerate(L.iterrows()):
        for sl, d in enumerate(int(x) for x in str(row.slot2drone).split()):
            ten[d][k] = row[f't{sl}']
    for d, y in ten.items():
        if d == gone:
            k = np.flatnonzero(np.isfinite(y))[-1]
            ax.plot(np.r_[t[:k + 1], t[k + 1]], np.r_[y[:k + 1], 0.0], color=ps.drone_colour(d), lw=1.4,
                    ls='--', drawstyle='steps-post')
            ax.plot(t[k + 1], 0.0, 'x', color=ps.drone_colour(d), ms=4, mew=1.2)
        else:
            ax.plot(t, y, color='0.3', lw=1.1, drawstyle='steps-post')
    pre = t < 0
    groups = []                                   # UAVs whose planned tension agrees within 0.1 N
    for d, y in ten.items():
        v = float(np.nanmean(y[pre]))
        for g in groups:
            if abs(g[0] - v) < 0.1 and gone not in g[1] and d != gone:
                g[1].append(d)
                break
        else:
            groups.append([v, [d]])
    for v, ds in groups:
        name = ('plate ' if len(ds) == 1 else 'plates ') + ', '.join(str(DETACH_PLATES[d]) for d in sorted(ds))
        released = gone in ds
        ax.annotate(name + (' (released)' if released else ''), xy=(-5, v), xytext=(2, -3 if released else 3),
                    textcoords='offset points', fontsize=8, color='0.15' if released else '0.25',
                    va='top' if released else 'bottom')
    post = t > 1.0
    v3 = float(np.nanmean([np.nanmean(ten[d][post]) for d in ten if d != gone]))
    ax.annotate(f'three survivors, {v3:.2f}\\,N each', xy=(12, v3), xytext=(-2, 3), textcoords='offset points',
                fontsize=8, color='0.25', ha='right', va='bottom')
    ax.set_ylabel('Planned tension (N)')
    ax.set_ylim(0, 5.5)
    ax.set_yticks([0, 1, 2, 3, 4, 5])
    w = run[(run.t >= t_det - 5) & (run.t <= t_det + 12)]
    tr = w.t.to_numpy() - t_det
    tilt = w.payload_tilt_deg.to_numpy()
    axt.plot(tr, tilt, color=ps.PAYLOAD_LINE, lw=1.3)
    band = float(tilt[tr < 0].max()) + 0.5            # the hover's own tilt plus 0.5 deg
    axt.axhline(band, color='0.45', lw=0.6, ls=':')
    over = np.flatnonzero((tr >= 0) & (tilt > band))
    t2 = float(tr[over[-1] + 1]) if over.size else 0.0
    axt.annotate('', xy=(0, 3.6), xytext=(t2, 3.6),
                 arrowprops=dict(arrowstyle='<->', color='0.3', lw=0.6, shrinkA=0, shrinkB=0))
    axt.annotate(f'{t2:.1f}\\,s', xy=(t2, 3.6), xytext=(4, 0), textcoords='offset points', fontsize=8,
                 color='0.3', va='center')
    axt.set_ylabel(r'Ring tilt (\si{\degree})')
    axt.set_ylim(0, 4)
    axt.set_yticks([0, 1, 2, 3, 4])
    z0 = 1.0                                           # the hover target
    axz.plot(tr, 100 * (w.payload_z.to_numpy() - z0), color=ps.PAYLOAD_LINE, lw=1.3)
    axz.axhline(0.0, color='0.6', lw=0.5)
    axz.set_ylabel(r'Height error (cm)')
    axz.set_ylim(-4, 4)
    axz.set_yticks([-4, -2, 0, 2, 4])
    axz.set_xlabel(r'$t$ after DETACH (s)')
    axz.set_xlim(-5, 12)
    for a in axs:
        a.axvline(0.0, color='0.75', lw=0.4, zorder=0)
    ax.annotate('DETACH', xy=(0, 1.0), xycoords=('data', 'axes fraction'), xytext=(2, 1),
                textcoords='offset points', fontsize=8, color='0.35', va='bottom')
    for a, letter in zip(axs, 'abc'):
        ps.panel_label(a, letter)
    fig.align_ylabels()
    peak = float(w.payload_tilt_deg[(tr >= 0) & (tr < 5)].max())
    stem = os.path.join(OUT, 'F_detach', 'detach')
    ps.save_print(fig, stem)
    return stem, dict(peak_tilt=peak, band=band, settle=t2, survivors=v3, z_change_max=float(100 * np.max(np.abs(w.payload_z[tr >= 0] - z0))))


TRAJ_RUNS = dict(circle=dict(old='R0933', new='R1033'), eight=dict(old='R0935', new='R1035'))


def traj_window(rid, kind):
    """Ring path, reference and horizontal error while the reference moves (faster than 1 cm/s), sampled
    where the 10 Hz reference updates; the circle's first lap from motion onset, or the whole figure-8."""
    run = run_csv(rid)
    ch = (run.payload_ref_x.diff().abs() > 1e-6) | (run.payload_ref_y.diff().abs() > 1e-6)
    u = run[ch]
    sp = np.hypot(u.payload_ref_x.diff(), u.payload_ref_y.diff()) / u.t.diff()
    moving = u.t[sp > 0.01]
    t0 = float(moving.iloc[0])
    t1 = t0 + LAP_S if kind == 'circle' else float(moving.iloc[-1])
    w = u[(u.t >= t0) & (u.t <= t1)]
    e = 100 * np.hypot(w.payload_x - w.payload_ref_x, w.payload_y - w.payload_ref_y).to_numpy()
    zerr = 100 * float((w.payload_z - w.payload_ref_z).mean())
    return (w.t.to_numpy() - t0, w.payload_x.to_numpy(), w.payload_y.to_numpy(),
            w.payload_ref_x.to_numpy(), w.payload_ref_y.to_numpy(), e, zerr)


def fig_trajectories():
    """F_traj -- "With node 0 pinned, four UAVs carry the ring around a circle and a figure-8 about a
    centimetre from the reference; the relaxed box left about three."
    hero: the error strips (gold, epsilon 0) against the relaxed box (grey, dashed); plan views show
    the shapes, with the reference drawn on top."""
    ps.thesis()
    fig = plt.figure(figsize=ps.size('full', height=3.7))
    gs = fig.add_gridspec(2, 2, width_ratios=[1.0, 1.9], height_ratios=[1.5, 1.0])
    stats = {}
    for col, kind in enumerate(('circle', 'eight')):
        runs = TRAJ_RUNS[kind]
        axp = fig.add_subplot(gs[0, col])
        axe = fig.add_subplot(gs[1, col])
        t, x, y, rx, ry, e, zerr = traj_window(runs['new'], kind)
        axp.plot(x, y, color=ps.PAYLOAD_LINE, lw=1.0, zorder=2)
        axp.plot(rx, ry, '--', color='0.15', lw=0.5, dashes=(4, 3), zorder=3)
        axp.plot(x[0], y[0], 'o', color='0.15', ms=3, zorder=4)
        k = len(x) // 8
        axp.annotate('', xy=(x[k + 3], y[k + 3]), xytext=(x[k], y[k]),
                     arrowprops=dict(arrowstyle='-|>', color='0.15', lw=0.8, mutation_scale=8), zorder=5)
        axp.set_aspect('equal')
        axp.set_anchor('W')
        ticks = [-0.5, 0.0, 0.5, 1.0] if kind == 'circle' else [-1.0, -0.5, 0.0, 0.5, 1.0]
        axp.set_xticks([v for v in ticks if v <= 0.5] if kind == 'circle' else ticks)
        axp.set_yticks([0.0, 0.5, 1.0] if kind == 'circle' else [-0.5, 0.0, 0.5])
        axp.set_xlabel(r'$x$ (m)')
        axp.set_ylabel(r'$y$ (m)')
        to, *_, eo, zo = traj_window(runs['old'], kind)
        axe.plot(to, eo, color='0.45', lw=0.8, ls=(0, (3, 1.5)), label=r'$\epsilon = 0.025$')
        axe.plot(t, e, color=ps.PAYLOAD_LINE, lw=1.2, label=r'$\epsilon = 0$')
        stats[(kind, 'new')] = (float(e.mean()), float(e.max()), zerr, float(t[-1]))
        stats[(kind, 'old')] = (float(eo.mean()), float(eo.max()), zo, float(to[-1]))
        axe.set_ylim(0, 6)
        axe.set_yticks([0, 2, 4, 6])
        axe.set_xlim(0, max(t[-1], to[-1]))
        axe.set_xlabel(r'$t$ from motion onset (s)')
        if col == 0:
            axe.set_ylabel('Horizontal error (cm)')
        else:
            axe.legend(loc='upper right', ncol=2, handlelength=2.2, borderaxespad=0.2)
        ps.panel_label(axp, 'ab'[col])
        ps.panel_label(axe, 'cd'[col])
    fig.align_ylabels()
    stem = os.path.join(OUT, 'F_traj', 'trajectories')
    ps.save_print(fig, stem)
    return stem, stats


FLIGHT_RUN = 'R1013'


def fig_flight():
    """F_flight -- "From the floor, the UAVs first swing their rods up to 45 deg (creep), then
    pretension, then lift the ring to 1.0 m; on LAND the ring comes down first and the UAVs follow."
    hero: ring height and rod elevations through the phases; phases as light bands with direct labels."""
    ps.thesis()
    r = FLIGHT_RUN
    ev = events(r)
    run = run_csv(r)
    L = planner_log(r)
    t_to, t_land, t_landed = ev['TAKEOFF'][0][0], ev['LAND'][0][0], ev['LANDED'][0][0]
    t_hand = float(L.t[L.phase == 'planner'].iloc[0])
    P = L[L.phase == 'planner']
    t_lift = float(P.t[P.lift_progress > 1e-3].iloc[0])
    t_up = float(run.t[(run.t > t_lift) & (run.payload_z >= 0.98)].iloc[0])
    z0 = float(run.payload_z.iloc[:20].median())
    t_down = float(run.t[(run.t > t_land) & (run.payload_z <= z0 + 0.05)].iloc[0])
    w = run[(run.t >= t_to - 1) & (run.t <= t_landed + 1)]
    t = w.t.to_numpy() - t_to
    fig, (ax, axe) = plt.subplots(2, 1, figsize=ps.size('full', height=3.0), sharex=True,
                                  gridspec_kw=dict(height_ratios=[1.3, 1.0]))
    n = len([c for c in run.columns if c.endswith('_tilt_deg') and c.startswith('d')])
    Lw = L[(L.t >= t_to - 1) & (L.t <= t_landed + 1)]
    for i in range(n):
        ax.plot(t, w[f'd{i}_z'].to_numpy(), color='0.55', lw=0.8)
        axe.plot(Lw.t.to_numpy() - t_to, Lw[f'elev{i}'].to_numpy(), color='0.3', lw=1.0)   # planner log, 10 Hz
    ax.plot(t, w.payload_z.to_numpy(), color=ps.PAYLOAD_LINE, lw=1.4)
    P2 = P[(P.t >= t_to - 1) & (P.t <= t_land)]
    ax.plot(P2.t.to_numpy() - t_to, P2.z_tgt.to_numpy(), '--', color=ps.PAYLOAD_LINE, lw=0.7, zorder=1)
    ax.annotate('ring', xy=(t[-1] * 0.45, 1.0), xytext=(0, -3), textcoords='offset points', fontsize=8,
                color=ps.PAYLOAD_LINE, va='top')
    ax.annotate('UAVs', xy=(t[-1] * 0.45, 1.5), xytext=(0, 3), textcoords='offset points', fontsize=8,
                color='0.4', va='bottom')
    axe.axhline(45.0, color='0.5', lw=0.6, ls='--', zorder=1)
    hold = Lw[(Lw.t > t_up + 2) & (Lw.t < t_land)]
    e_hold = float(np.nanmean([hold[f'elev{i}'].mean() for i in range(n)]))
    axe.axhline(e_hold, color='0.5', lw=0.5, ls=':', zorder=1)
    axe.annotate(f'{e_hold:.0f}$^\\circ$', xy=((t_up + t_land) / 2 - t_to, e_hold), xytext=(0, 3),
                 textcoords='offset points', fontsize=8, color='0.35', ha='center', va='bottom')
    bands = [(t_to, t_hand, 'creep'), (t_hand, t_lift, 'pre-\ntension'), (t_lift, t_up, 'lift'),
             (t_up, t_land, 'hold'), (t_land, t_down, 'LAND'), (t_down, t_landed, 'unwind')]
    for k, (a, b, lab) in enumerate(bands):
        for axx in (ax, axe):
            if lab != 'hold' and k % 2 == 0:
                axx.axvspan(a - t_to, b - t_to, color='0.93', lw=0, zorder=0)
            axx.axvline(a - t_to, color='0.8', lw=0.4, zorder=0)
            axx.axvline(b - t_to, color='0.8', lw=0.4, zorder=0)
        ax.annotate(lab, xy=((a + b) / 2 - t_to, 1.0), xycoords=('data', 'axes fraction'),
                    xytext=(0, -2), textcoords='offset points', fontsize=8, color='0.35',
                    ha='center', va='top', multialignment='center')
    ax.set_ylabel(r'$z$ (m)')
    ax.set_ylim(0, 2.0)
    ax.set_yticks([0, 0.5, 1.0, 1.5])
    axe.set_ylabel(r'Rod elevation (\si{\degree})')
    axe.set_ylim(0, 70)
    axe.set_yticks([0, 15, 30, 45, 60])
    axe.set_xlabel(r'$t$ after TAKEOFF (s)')
    axe.set_xlim(t[0], t[-1])
    ps.panel_label(ax, 'a')
    ps.panel_label(axe, 'b')
    fig.align_ylabels()
    stem = os.path.join(OUT, 'F_flight', 'flight')
    ps.save_print(fig, stem)
    return stem, dict(creep=t_hand - t_to, pretension=t_lift - t_hand, lift=t_up - t_lift,
                      land_to_down=t_down - t_land, down_to_landed=t_landed - t_down)


if __name__ == '__main__':
    for fn in (fig_tracker_x0, fig_disturbance, fig_landing, fig_detach, fig_trajectories, fig_flight):
        stem, stats = fn()
        print(stem, {k: (tuple(round(v, 2) for v in s) if isinstance(s, tuple) else round(s, 3))
                     for k, s in stats.items()})

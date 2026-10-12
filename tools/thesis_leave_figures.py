#!/usr/bin/env python3
"""tools/thesis_leave_figures.py -- the leave chapter's rig figures (thesis plan of 11 Oct, section 4):

    python3 tools/thesis_leave_figures.py carry      # F8  the circle on the rig: measured pull at 60 deg against the planned pull at 45
    python3 tools/thesis_leave_figures.py releases   # F10 every release on the rig (the hardware record; the twin crossover panel comes with Tier A)
    python3 tools/thesis_leave_figures.py magnet     # F11 how the ring was lost: the tension step and the magnets letting go

Every run id comes from configs/thesis_figures.yaml (tools/figure_runs.py). Outputs go to docs/figures/<ID>/ at print
size (tools/plot_style.thesis). Rig flights are read from their rig_to_run.py conversions (logs/run.csv, events.csv)
and, for the magnet figure, from the flight's bag through tools/drop_trigger_replay.py (rosbag2_py, cached under
results/analysis/bag_cache). The release metrics are tools/release_bands.py's (the signed T6 definition).

Briefs (thesis-figure skill, step 1):

F8  Sentence: with the trackers on the measured rod pull and the rods planned at 60 deg, the ring follows the circle
    within 3 cm and lags it by 0.2 s, where the morning's planned pull at 45 deg lagged by 0.45 s and wandered 8 cm.
    Hero: the along-track error over the lap, both runs. Support: the plan view (the path on the circle) and the
    height error. Left out: the lift, the landing, the carriers, the tilt. Window: the orbit, from the reference's
    first motion to LAND.
F10 Sentence: on the rig the ring was carried on through every commanded release except the one whose planned step
    tore a survivor's magnet off, and lost twice more when magnets let go with no command; the twin predicted the
    reactive releases and not the magnet releases. Hero: one row per release event, the tilt rise as a dot with the
    twin pair's range beside it; the capsizes on the broken axis. Left out: dips, detection times, the ramps.
F11 Sentence: every ring lost on the rig began with a magnet letting go, under rods planned at 45 deg: once 0.16 s
    after a commanded release whose planned step was two thirds of a survivor's share, and twice with no command on
    the circle. Hero: each rod's excess length (the magnet off its plate) against the ring's tilt. Support: the
    planned tensions. Left out: everything before the last second and after the latch.
"""
import csv
import os
import sys

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import plot_style as ps                      # noqa: E402
import figure_runs as fr                     # noqa: E402
import release_bands as rb                   # noqa: E402

REPO = fr.REPO
OUT = os.path.join(REPO, 'docs', 'figures')
GREY_TEXT = '0.35'
PLATE_R = 0.225                              # the ring's plate radius (fig:geometry)
PIVOT_DZ = -0.04                             # the rod pivot below the body (model.tex)
ONSET_M, SEP_M = 0.003, 0.03                 # a magnet off its plate: tools/drop_trigger_replay.py's thresholds


def load_run(tag):
    d = fr.resolve(tag)
    run = pd.read_csv(os.path.join(d, 'logs', 'run.csv')).drop_duplicates('t').reset_index(drop=True)
    ev = {}
    for e in csv.DictReader(open(os.path.join(d, 'logs', 'events.csv'))):
        ev.setdefault(e['event'], []).append(float(e['sim_time']))
        ev.setdefault(e['event'] + '_arg', []).append(e.get('arg', ''))
    return run, ev, d


def rotm(q):
    """Rotation matrices from quaternions (w, x, y, z), shape (n, 3, 3)."""
    q = np.asarray(q, float)
    q = q / np.linalg.norm(q, axis=1, keepdims=True)
    w, x, y, z = q.T
    return np.stack([np.stack([1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)], -1),
                     np.stack([2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)], -1),
                     np.stack([2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)], -1)], 1)


def t6(run, te, t_end):
    """The signed release metric on a run table: tilt rise over [te, t_end] about the 5 s pre-window mean."""
    q = run[['payload_qw', 'payload_qx', 'payload_qy', 'payload_qz']].to_numpy(float)
    ok = np.all(np.isfinite(q), 1)
    t, th = run.t.to_numpy(float)[ok], rb.tilt_vector(q[ok])
    pre = (t >= te - rb.PRE_S) & (t < te)
    mean = th[pre].mean(0)
    sig = float(np.sqrt(np.mean(np.sum((th[pre] - mean) ** 2, 1))))
    w = (t >= te) & (t <= t_end)
    dev = np.linalg.norm(th[w] - mean, axis=1)
    k = int(np.argmax(dev))
    return dict(dtheta=float(dev[k]), t_peak=float(t[w][k] - te), abs_peak=float(np.linalg.norm(th[w], axis=1).max()),
                sigma_pre=sig, pre_mean=float(np.linalg.norm(mean)), t=t - te, dev=np.linalg.norm(th - mean, axis=1),
                mag=np.linalg.norm(th, axis=1))


# ---------------------------------------------------------------- F8: the circle on the rig
def circle_fit(p):
    """Least-squares circle through the points p (n, 2): centre, radius."""
    A = np.c_[2 * p, np.ones(len(p))]
    b = (p ** 2).sum(1)
    cx, cy, c = np.linalg.lstsq(A, b, rcond=None)[0]
    return np.array([cx, cy]), float(np.sqrt(c + cx * cx + cy * cy))


def carry_trace(row):
    run, ev, _ = load_run(row['run'])
    t = run.t.to_numpy(float)
    ref = run[['payload_ref_x', 'payload_ref_y']].to_numpy(float)
    pos = run[['payload_x', 'payload_y']].to_numpy(float)
    t_h = ev['HANDOVER'][0]
    hov = np.nanmedian(ref[(t > t_h) & (t < t_h + 2.0)], 0)
    moved = np.linalg.norm(ref - hov, axis=1) > 0.01
    t0 = float(t[moved & (t > t_h)][0])
    t_end = ev['LAND'][0]
    w = (t >= t0) & (t < t_end)
    c, R = circle_fit(ref[w])
    ang = lambda p: np.arctan2(p[:, 1] - c[1], p[:, 0] - c[0])
    # one lap: the reference's unwrapped angle has advanced 2 pi (the orbit ramps up over its first seconds)
    a_ref = np.unwrap(ang(ref[w]))
    lap = np.flatnonzero(np.abs(a_ref - a_ref[0]) >= 2 * np.pi)
    if len(lap):
        t_end = float(t[w][lap[0]])
        w = (t >= t0) & (t < t_end)
    dang = np.angle(np.exp(1j * (ang(pos[w]) - ang(ref[w]))))
    along = R * dang                                            # m, negative: behind the reference
    # the commanded speed once the ramp is done: 1 s chords of the reference (held on the 50 Hz grid between planner ticks)
    tw, rw = t[w], ref[w]
    ahead = np.stack([np.interp(tw + 1.0, tw, rw[:, i]) for i in range(2)], 1)
    v_ref = np.linalg.norm(ahead - rw, axis=1)
    v = float(np.median(v_ref[(tw > t0 + 5.0) & (tw < tw[-1] - 1.5)]))
    ez = (run.payload_z - run.payload_ref_z).to_numpy(float)
    exy = np.linalg.norm(pos[w] - ref[w], axis=1)
    radial = np.linalg.norm(pos[w] - c, axis=1) - R
    stats = dict(lap_s=float(t_end - t0), v=v, R=R, exy_mean=float(exy.mean()), exy_max=float(exy.max()),
                 along_mean=float(along.mean()), lag_s=float(-along.mean() / v), radial_mean=float(np.abs(radial).mean()),
                 z_sd=float(np.nanstd(ez[w])), z_mean=float(np.nanmean(ez[w])),
                 tilt_mean=float(np.nanmean(run.payload_tilt_deg[w])), tilt_max=float(np.nanmax(run.payload_tilt_deg[w])))
    return dict(t=t[w] - t0, pos=pos[w], ref=ref[w], c=c, R=R, along=along, ez=ez[w], stats=stats)


def fig_carry():
    ps.thesis()
    spec = fr.figure('F_rig_carry')
    rows = spec['rows']
    tr = {r['role']: (r, carry_trace(r)) for r in rows}
    hero, ctx = tr['hero'], tr['context']
    for role, (r, d) in tr.items():
        print(role, r['run'], {k: round(v, 3) for k, v in d['stats'].items()})
    gold, grey = ps.PAYLOAD_LINE, '0.45'
    fig = plt.figure(figsize=ps.size('full', height=2.75))
    gs = fig.add_gridspec(2, 2, width_ratios=[1.0, 1.35], height_ratios=[1, 1])
    ax_p = fig.add_subplot(gs[:, 0])
    ax_a = fig.add_subplot(gs[0, 1])
    ax_z = fig.add_subplot(gs[1, 1], sharex=ax_a)
    # (a) plan view: the reference circle and both paths
    d = hero[1]
    th = np.linspace(0, 2 * np.pi, 361)
    ax_p.plot(d['c'][0] + d['R'] * np.cos(th), d['c'][1] + d['R'] * np.sin(th), color='0.6', lw=0.7, ls=(0, (3, 2)),
              zorder=1)
    ax_p.plot(ctx[1]['pos'][:, 0], ctx[1]['pos'][:, 1], color=grey, lw=0.7, zorder=2)
    ax_p.plot(d['pos'][:, 0], d['pos'][:, 1], color=gold, lw=1.3, zorder=3)
    ax_p.plot(d['pos'][0, 0], d['pos'][0, 1], 'o', color='k', ms=2.5, zorder=4)
    ax_p.annotate('start', (d['pos'][0, 0], d['pos'][0, 1]), xytext=(0, 5), textcoords='offset points', fontsize=8,
                  color=GREY_TEXT, ha='center', va='bottom')
    # the direction of travel: a black arrowhead on the reference circle, a third of the way round
    k = int(len(d['ref']) * 0.3)
    ax_p.annotate('', xy=d['ref'][k + 12], xytext=d['ref'][k], arrowprops=dict(arrowstyle='-|>', color='k', lw=0.8,
                                                                              mutation_scale=9, shrinkA=0, shrinkB=0))
    ax_p.set_aspect('equal', adjustable='box')
    ax_p.set_xlabel('$x$ (m)')
    ax_p.set_ylabel('$y$ (m)')
    ax_p.set_xticks([-0.5, 0, 0.5])
    ax_p.set_yticks([0, 0.5, 1.0])
    ax_p.annotate(hero[0]['label'].replace(', ', ',\\\\'), (0.5, 0.56), xycoords='axes fraction', ha='center',
                  va='center', fontsize=8, color=gold, linespacing=1.1)
    ax_p.annotate(ctx[0]['label'].replace(', ', ',\\\\'), (0.5, 0.40), xycoords='axes fraction', ha='center',
                  va='center', fontsize=8, color=grey, linespacing=1.1)
    ax_p.set_ylim(-0.12, 1.08)
    ps.panel_label(ax_p, 'a')
    # (b) along-track error; (c) height error
    for ax, key, unit, lab in ((ax_a, 'along', 100.0, 'Along-track error (cm)'), (ax_z, 'ez', 100.0, 'Height error (cm)')):
        ax.axhline(0, color='0.7', lw=0.5, zorder=0)
        ax.plot(ctx[1]['t'], unit * ctx[1][key], color=grey, lw=0.7, zorder=2)
        ax.plot(d['t'], unit * d[key], color=gold, lw=1.2, zorder=3)
        ax.set_ylabel(lab)
    s_h, s_c = hero[1]['stats'], ctx[1]['stats']
    ax_a.axhline(100 * s_h['along_mean'], color=gold, lw=0.5, ls=(0, (2, 2)), zorder=1)
    ax_a.axhline(100 * s_c['along_mean'], color=grey, lw=0.5, ls=(0, (2, 2)), zorder=1)
    ax_a.annotate(f"{s_h['lag_s']:.2f}\\,s behind (mean $-${-100 * s_h['along_mean']:.1f}\\,cm)", (0.02, 0.97),
                  xycoords='axes fraction', ha='left', va='top', fontsize=8, color=gold)
    ax_a.annotate(f"{s_c['lag_s']:.2f}\\,s behind (mean $-${-100 * s_c['along_mean']:.1f}\\,cm)", (0.02, 0.82),
                  xycoords='axes fraction', ha='left', va='top', fontsize=8, color=grey)
    ax_a.set_ylim(-18, 14)
    ax_a.set_yticks([-15, -10, -5, 0, 5, 10])
    ax_z.set_yticks([-10, -5, 0, 5, 10])
    ax_z.set_xlabel('$t$ after the orbit starts (s)')
    ax_z.set_xlim(0, max(d['t'][-1], ctx[1]['t'][-1]))
    plt.setp(ax_a.get_xticklabels(), visible=False)
    ps.panel_label(ax_a, 'b')
    ps.panel_label(ax_z, 'c')
    fig.align_ylabels([ax_a, ax_z])
    paths = ps.save_print(fig, os.path.join(OUT, 'F_rig_carry', 'rig_carry'))
    print('\n'.join(paths))


# ---------------------------------------------------------------- F10: every release on the rig
def plates_from_run(run, t_a, t_b):
    """Per drone: the plate it stands on (the 30 deg grid point nearest its pivot's azimuth in the ring frame over
    the taut window [t_a, t_b]) and the excess of pivot-to-plate over its median rod length, as the replay does."""
    t = run.t.to_numpy(float)
    ring = run[['payload_x', 'payload_y', 'payload_z']].to_numpy(float)
    R = rotm(run[['payload_qw', 'payload_qx', 'payload_qy', 'payload_qz']].to_numpy(float))
    out = {}
    for i in range(4):
        if f'd{i}_x' not in run:
            continue
        piv = run[[f'd{i}_x', f'd{i}_y', f'd{i}_z']].to_numpy(float) + np.array([0.0, 0.0, PIVOT_DZ])
        rel = np.einsum('nji,nj->ni', R, piv - ring)
        w = (t >= t_a) & (t < t_b) & np.all(np.isfinite(rel), 1)
        if w.sum() < 10:
            continue
        az = np.arctan2(rel[w, 1], rel[w, 0])
        az = float(np.arctan2(np.median(np.sin(az)), np.median(np.cos(az))))
        paz = np.radians(round(np.degrees(az) / 30.0) * 30.0)
        rho = np.array([PLATE_R * np.cos(paz), PLATE_R * np.sin(paz), 0.0])
        dist = np.linalg.norm(piv - (ring + np.einsum('nij,j->ni', R, rho)), axis=1)
        L = float(np.nanmedian(dist[w]))
        out[i] = dict(exc=dist - L, L=L, az_deg=float(np.degrees(paz)))
    return out


def onsets_from_run(run, ev, plates):
    """Uncommanded releases: per drone the onset of the first sustained excess > SEP_M (6 samples) after the
    hand-over, walked back to base + ONSET_M (tools/drop_trigger_replay.py separation())."""
    t = run.t.to_numpy(float)
    t_h = ev['HANDOVER'][0]
    out = []
    for i, p in plates.items():
        e = p['exc']
        idx = np.flatnonzero((t > t_h + 1.0) & (e > SEP_M))
        for k in idx:
            if np.all(e[k:k + 3] > SEP_M):
                b = (t > t[k] - 1.0) & (t < t[k] - 0.2)
                base = float(np.nanmedian(e[b])) if b.any() else 0.0
                j = k
                while j > 0 and e[j - 1] > base + ONSET_M:
                    j -= 1
                out.append((float(t[j]), i))
                break
    return sorted(out)


def release_rows():
    spec = fr.figure('F_releases')
    rows = []
    for r in spec['rows']:
        run, ev, _ = load_run(r['run'])
        t = run.t.to_numpy(float)
        if r['kind'] == 'uncommanded':
            plates = plates_from_run(run, ev['HANDOVER'][0] + 1.0, ev['HANDOVER'][0] + 3.0)
            ons = onsets_from_run(run, ev, plates)
            te = ons[r['event_index']][0]
            who = ons[r['event_index']][1]
        elif 'RELEASE' in ev:
            te, who = ev['RELEASE'][0], None
        else:
            te, who = ev['DETACH'][0], None
        end_ev = r.get('end_event', 'LAND')
        cands = [x for x in ev.get(end_ev, []) if x > te]
        t_end = cands[0] if cands else float(t[-1])
        if end_ev == 'LAND' and 'LAND' in ev and not cands:
            t_end = float(t[-1])
        m = t6(run, te, t_end)
        twin = []
        for rid in r.get('twin', []):
            tw = rb.Run(fr.resolve(rid))
            twin.append(tw.metrics(t_end - te)['dtheta'])
        rows.append(dict(**r, te=te, t_end=t_end, who=who, dtheta=m['dtheta'], abs_peak=m['abs_peak'],
                         sigma_pre=m['sigma_pre'], twin_vals=twin, in_landing=bool('LAND' in ev and te > ev['LAND'][0])))
        if r['kind'] == 'uncommanded' and r.get('event_index', 0) > 0:
            rows[-1]['gap_s'] = te - ons[0][0]
        print(r['run'], r['kind'], f"te {te:.2f} end {t_end:.2f} ({end_ev}) window {t_end - te:.1f} s "
              f"dtheta {m['dtheta']:.1f} abs {m['abs_peak']:.1f} sigma_pre {m['sigma_pre']:.2f} twin {np.round(twin, 1)}"
              + (f' drone {who}' if who is not None else ''))
    return rows


def row_label(r):
    kind = {'announced': 'announced', 'unannounced': 'unannounced', 'uncommanded': 'no command'}[r['kind']]
    if r['kind'] == 'uncommanded':
        kind += ', first magnet' if r.get('event_index', 0) == 0 else f", second magnet {r.get('gap_s', 0):.1f}\\,s later"
    pull = 'measured pull' if r['pull'] == 'measured' else "planner's pull"
    gap = f"{r['gap']}$^\\circ$ gap" if r['gap'] < 180 else 'even ring of four, 180$^\\circ$ gap'
    return f"{r['traj']}, {kind}", f"{pull}, {gap}"


def fig_releases():
    ps.thesis()
    rows = release_rows()
    n = len(rows)
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=ps.size('full', height=3.5), sharey=True,
                                   gridspec_kw=dict(width_ratios=[3.2, 1.0], wspace=0.04))
    # rows grouped by rod plan (the date), a header and a half-row gap before each group
    groups = []
    for r in rows:
        if not groups or groups[-1][0] != r['plan']:
            groups.append((r['plan'], []))
        groups[-1][1].append(r)
    y, yh = [], []
    cur = 0.0
    for plan, members in groups:
        yh.append((cur, plan))
        cur -= 0.75
        for r in members:
            y.append(cur)
            cur -= 1.0
    y = np.array(y)
    shapes = {'announced': 'o', 'unannounced': 's', 'uncommanded': 'D'}
    for ax in (ax1, ax2):
        for k, yy in enumerate(y):
            if k % 2 == 1:
                ax.axhspan(yy - 0.5, yy + 0.5, color='0.94', lw=0, zorder=0)
    for yy, r in zip(y, rows):
        cap = bool(r.get('capsize'))
        ax = ax2 if r['dtheta'] > 20 else ax1
        if r['twin_vals']:
            lo, hi = min(r['twin_vals']), max(r['twin_vals'])
            ax.plot([lo, hi], [yy, yy], color='0.45', lw=2.4, solid_capstyle='butt', alpha=0.6, zorder=1)
            for v in (lo, hi):
                ax.plot([v, v], [yy - 0.22, yy + 0.22], color='0.45', lw=0.8, alpha=0.9, zorder=1)
        elif r['dtheta'] <= 20:
            ax.annotate('no twin pair', (0.3, yy), xytext=(0, 0), textcoords='offset points', fontsize=7, color='0.6',
                        ha='left', va='center')
        ax.plot(r['dtheta'], yy, shapes[r['kind']], color='k', ms=4.5, mfc='k' if r['pull'] == 'measured' else 'white',
                mew=0.9, zorder=3)
        if r['dtheta'] > 20:
            tag = ('capsized' if cap else 'recovered') + (', in the landing' if r.get('in_landing') else '')
            ax.annotate(f"{tag}, {r['dtheta']:.0f}$^\\circ$", (r['dtheta'], yy), xytext=(-6, 0), textcoords='offset points',
                        ha='right', va='center', fontsize=7, color=GREY_TEXT)
    for yy, plan in yh:
        ax1.annotate(('8 October, rods planned at ' if plan == 60 else '7 October, rods planned at ') + f'{plan}$^\\circ$',
                     (0.0, yy), xycoords=('axes fraction', 'data'), xytext=(-6, 0), textcoords='offset points',
                     ha='right', va='center', fontsize=7, color='k', style='italic')
    for v, lab, side in ((30, 'drop rule', 'right'), (60, 'tracker latch', 'left')):
        ax2.axvline(v, color='0.75', lw=0.5, ls=(0, (1, 1.5)), zorder=0)
        ax2.annotate(lab, (v, 1.0), xycoords=('data', 'axes fraction'), xytext=(-2 if side == 'right' else 2, 2),
                     textcoords='offset points', ha=side, va='bottom', fontsize=7, color='0.5')
    ax1.set_yticks(y)
    ax1.set_yticklabels([row_label(r)[0] for r in rows], fontsize=7)
    for yy, r in zip(y, rows):
        ax1.annotate(row_label(r)[1], (0.0, yy), xycoords=('axes fraction', 'data'), xytext=(-6, -8),
                     textcoords='offset points', ha='right', va='center', fontsize=7, color=GREY_TEXT)
    ax1.tick_params(axis='y', pad=6)
    for lab in ax1.get_yticklabels():
        lab.set_verticalalignment('bottom')
    ax1.set_xlim(0, 20)
    ax1.set_xticks([0, 5, 10, 15, 20])
    ax2.set_xlim(23, 95)
    ax2.set_xticks([30, 60, 90])
    ax1.spines['right'].set_visible(False)
    ax2.spines['left'].set_visible(False)
    ax2.tick_params(left=False)
    for ax in (ax1, ax2):
        ax.set_ylim(y[-1] - 0.6, 0.45)
    ax1.set_xlabel('Tilt rise $\\Delta\\theta$ (\\si{\\degree})')
    ax1.xaxis.set_label_coords(0.68, -0.09)
    # the axis break
    d = 0.015
    for ax, x in ((ax1, 1.0), (ax2, 0.0)):
        ax.plot([x - d, x + d], [-d, d], transform=ax.transAxes, color='k', clip_on=False, lw=0.6)
    # the key: one row of marker glyphs and words above the left panel, drawn by hand (no legend machinery)
    key = [('o', 'k', 'announced'), ('s', 'k', 'unannounced'), ('D', 'k', 'no command'), ('o', 'white', 'hollow: planned pull'),
           (None, None, 'twin pair')]
    fig.get_layout_engine().set(rect=(0, 0, 1, 0.94))     # the key row above the panels
    x, y = 0.29, 0.97
    fig.canvas.draw()
    rend = fig.canvas.get_renderer()
    fw = fig.get_size_inches()[0] * fig.dpi
    for mk, fc, word in key:
        if mk is not None:
            fig.add_artist(matplotlib.lines.Line2D([x], [y], marker=mk, color='k', mfc=fc, ms=4.5, mew=0.9, ls='none',
                                                   transform=fig.transFigure))
            x += 0.013
        else:
            fig.add_artist(matplotlib.lines.Line2D([x, x + 0.02], [y, y], color='0.55', lw=3.0, alpha=0.55,
                                                   transform=fig.transFigure))
            x += 0.028
        txt = fig.text(x, y, word, fontsize=7, color=GREY_TEXT, ha='left', va='center')
        x += txt.get_window_extent(rend).width / fw + 0.016
    paths = ps.save_print(fig, os.path.join(OUT, 'F_releases', 'releases'))
    print('\n'.join(paths))


# ---------------------------------------------------------------- F11: how the ring was lost
def planner_log(logs_dir):
    import glob
    f = sorted(glob.glob(os.path.join(logs_dir, 'logs', '*planner', '*', 'log.csv')))[-1]
    return pd.read_csv(f).drop_duplicates('sim_time').reset_index(drop=True)


def align_planner(P, run, t2_log=None, ev=None):
    """The planner log's clock offset to the run table: exact from the T2 console's ARM line (the same host clock
    as the planner log, and events.csv's ARM) when the console log is given, else from the lift."""
    if t2_log and ev and 'ARM' in ev and os.path.isfile(t2_log):
        import re
        m = re.search(r"\[(\d{9,}\.\d+)\] \[(?:fleet_manager|central_controller)\]: Fleet command received: 'ARM'",
                      open(t2_log, errors='replace').read())
        if m:
            return float(ev['ARM'][0] - float(m.group(1)))
    t, z = run.t.to_numpy(float), run.payload_z.to_numpy(float)
    tp, zp = P.sim_time.to_numpy(float), P.load_z.to_numpy(float)
    ok = np.isfinite(zp)
    tp, zp = tp[ok], zp[ok]
    # coarse: the ring passing 0.15 m on both clocks; fine: least squares over +-0.5 s at 10 ms
    rise_run = float(t[np.flatnonzero(z > 0.15)[0]])
    rise_pl = float(tp[np.flatnonzero(zp > 0.15)[0]])
    best = None
    for off in np.arange(rise_run - rise_pl - 0.5, rise_run - rise_pl + 0.5, 0.01):
        zi = np.interp(tp + off, t, z, left=np.nan, right=np.nan)
        m = np.isfinite(zi)
        err = float(np.mean((zi[m] - zp[m]) ** 2)) if m.sum() > 50 else np.inf
        if best is None or err < best[1]:
            best = (off, err)
    return float(best[0])                # run time = sim_time + offset


def magnet_trace(row):
    run, ev, d = load_run(row['run'])
    P = planner_log(fr.resolve_dir(row['logs']))
    t2_log = os.path.join(os.path.dirname(d), os.path.basename(d).replace('_run', '') + '.log')
    off = align_planner(P, run, t2_log, ev)
    plates = plates_from_run(run, ev['HANDOVER'][0] + 1.0, ev['HANDOVER'][0] + 3.0)
    ons = onsets_from_run(run, ev, plates)
    te = ev['DETACH'][0] if 'DETACH' in ev else ons[0][0]
    t_abort = ev.get('FLEET_ABORT', [np.nan])[0]
    t_land = ev.get('LAND', [np.nan])[0]
    # the planned tensions per slot (t0..t3) with the slot-to-drone map
    tp = P.sim_time.to_numpy(float) + off - te
    slots = [[int(v) for v in str(s).split()] for s in P.slot2drone]
    tens = {}
    cols = [P[f't{k}'].to_numpy(float) for k in range(4) if f't{k}' in P]
    for j, sl in enumerate(slots):
        for k, dr in enumerate(sl):
            if k < len(cols) and np.isfinite(cols[k][j]):
                tens.setdefault(dr, []).append((tp[j], cols[k][j]))
    tens = {d: (np.array([a for a, _ in v]), np.array([b for _, b in v])) for d, v in tens.items()}
    return dict(run=run, ev=ev, te=te, t_abort=t_abort - te if np.isfinite(t_abort) else None,
                t_land=t_land - te if np.isfinite(t_land) else None, plates=plates, onsets=[(o - te, i) for o, i in ons],
                tens=tens, label=row['label'])


def fig_magnet():
    ps.thesis()
    spec = fr.figure('F_magnet_anatomy')
    tr = [magnet_trace(r) for r in spec['rows']]
    fig, axs = plt.subplots(3, 2, figsize=ps.size('full', height=3.9), sharex='col',
                            gridspec_kw=dict(height_ratios=[1.0, 1.0, 1.15]))
    windows = [(-1.0, tr[0]['t_abort'] + 0.06), (-1.5, tr[1]['t_abort'] + 0.06)]
    for col, (d, (t0, t1)) in enumerate(zip(tr, windows)):
        run, ev = d['run'], d['ev']
        t = run.t.to_numpy(float) - d['te']
        w = (t >= t0) & (t <= t1)
        leaver = None
        if 'DETACH' in ev:
            import re
            m = re.search(r'(\d+)', str(ev.get('DETACH_arg', [''])[0]))
            leaver = int(m[1]) if m else None          # the planner's 'DETACH drone N' line (events.csv)
        # (a) planned tensions
        ax = axs[0, col]
        for i, (tt, f) in sorted(d['tens'].items()):
            m = (tt >= t0) & (tt <= t1)
            ax.step(tt[m], f[m], where='post', color=ps.drone_colour(i) if i == leaver else '0.3',
                    lw=1.1 if i == leaver else 0.8, ls='--' if i == leaver else '-', zorder=2)
        ax.set_ylim(0, 6.5)
        if leaver is not None and leaver in d['tens']:
            tt, f = d['tens'][leaver]
            m = (tt >= t0) & (tt < 0)
            ax.annotate('the leaver', (tt[m][-1], f[m][-1]), xytext=(3, -2), textcoords='offset points', fontsize=7,
                        color=ps.drone_colour(leaver), ha='left', va='top')
        ax.annotate('the survivors' if leaver is not None else 'the four rods', (t0, 1.0), xycoords=('data', 'axes fraction'),
                    xytext=(3, -3), textcoords='offset points', fontsize=7, color=GREY_TEXT, ha='left', va='top')
        # (b) ring tilt
        ax = axs[1, col]
        tilt = run.payload_tilt_deg.to_numpy(float)
        ax.plot(t[w], tilt[w], color=ps.PAYLOAD_LINE, lw=1.2, zorder=3)
        for lvl, lab in ((30, 'drop rule, since 8 Oct'), (60, 'tracker latch')):
            ax.axhline(lvl, color='0.6', lw=0.6, ls=(0, (3, 2)), zorder=1)
            ax.annotate(f'{lab}, {lvl}$^\\circ$', (t0, lvl), xytext=(2, 2), textcoords='offset points', ha='left',
                        va='bottom', fontsize=7, color=GREY_TEXT)
        ax.set_ylim(0, 75)
        for o, i in d['onsets']:
            if t0 <= o <= t1 and i != leaver:
                tv = float(np.interp(o, t, tilt))
                ax.plot(o, tv, 'o', color='k', ms=3, zorder=4)
                ax.annotate(f'{tv:.1f}$^\\circ$', (o, tv), xytext=(4, 3), textcoords='offset points', fontsize=7,
                            color=GREY_TEXT, ha='left', va='bottom')
        # (c) rod excess per drone
        ax = axs[2, col]
        for i, p in sorted(d['plates'].items()):
            e = 100 * p['exc']
            col_i = ps.drone_colour(i) if i == leaver else '0.3'
            ax.plot(t[w], e[w], color=col_i, lw=1.1 if i == leaver else 0.8, ls='--' if i == leaver else '-', zorder=2)
        ax.axhline(100 * SEP_M, color='0.6', lw=0.6, ls=(0, (3, 2)), zorder=1)
        ax.annotate('off its plate, 3\\,cm', (t0, 100 * SEP_M), xytext=(2, 2), textcoords='offset points', ha='left',
                    va='bottom', fontsize=7, color=GREY_TEXT)
        n_on = 0
        for o, i in d['onsets']:
            if t0 <= o <= t1 and i != leaver:
                y0 = 100 * np.interp(o, t, d['plates'][i]['exc'])
                ax.plot(o, y0, 'o', color='k', ms=3, zorder=4)
                ax.axvline(o, color='0.5', lw=0.5, ls=(0, (1, 1.5)), zorder=1)
                when = f", {o:+.2f}\\,s" if abs(o) > 0.005 else ''
                right = o > 0.5 * (t0 + t1)
                ax.annotate(f"{d['plates'][i]['az_deg'] % 360:.0f}$^\\circ$ rod off{when}", (o, 1.0),
                            xycoords=('data', 'axes fraction'), xytext=(-3 if right else 3, -3 - 10 * n_on),
                            textcoords='offset points', fontsize=7, color=GREY_TEXT, ha='right' if right else 'left',
                            va='top')
                n_on += 1
        ax.set_ylim(-1.5, 25)
        # events on every panel: the release, the latch
        for ax in axs[:, col]:
            ax.axvline(0.0, color='0.5', lw=0.5, zorder=1)
            if d['t_abort'] is not None and t0 <= d['t_abort'] <= t1:
                ax.axvline(d['t_abort'], color='0.5', lw=0.5, zorder=1)
            if d['t_land'] is not None and t0 <= d['t_land'] <= t1:
                ax.axvline(d['t_land'], color='0.5', lw=0.5, ls=':', zorder=1)
            ax.set_xlim(t0, t1)
        top = axs[0, col]
        top.annotate('release' if 'DETACH' in ev else 'first magnet off', (0.0, 1.0), xycoords=('data', 'axes fraction'),
                     xytext=(-2 if 'DETACH' in ev else 2, 2), textcoords='offset points', fontsize=8, color=GREY_TEXT,
                     ha='right' if 'DETACH' in ev else 'left', va='bottom')
        if d['t_abort'] is not None and t0 <= d['t_abort'] <= t1:
            top.annotate('latch', (d['t_abort'], 1.0), xycoords=('data', 'axes fraction'), xytext=(-2, 2),
                         textcoords='offset points', fontsize=8, color=GREY_TEXT, ha='right', va='bottom')
        if d['t_land'] is not None and t0 <= d['t_land'] <= t1:
            axs[1, col].annotate('LAND', (d['t_land'], 1.0), xycoords=('data', 'axes fraction'), xytext=(2, 2),
                                 textcoords='offset points', fontsize=8, color=GREY_TEXT, ha='left', va='bottom')
        axs[2, col].set_xlabel('$t$ after the release (s)' if col == 0 else '$t$ after the first magnet let go (s)')
        print(d['label'], 'onsets', [(round(o, 3), i) for o, i in d['onsets']], 'abort', d['t_abort'], 'land', d['t_land'],
              'leaver', leaver)
        for i, (tt, f) in sorted(d['tens'].items()):
            pre = f[(tt > -1.0) & (tt < -0.05)]
            post = f[(tt >= 0.0) & (tt < 0.6)]
            if len(pre) and len(post):
                print(f'  drone {i}: planned {np.nanmean(pre):.2f} N -> first tick {post[0]:.2f} N '
                      f'({100 * (post[0] / np.nanmean(pre) - 1):+.0f} %), 0.6 s mean {np.nanmean(post):.2f} N')
    axs[0, 0].set_ylabel('Planned tension (N)')
    axs[1, 0].set_ylabel('Ring tilt (\\si{\\degree})')
    axs[2, 0].set_ylabel('Rod stretch (cm)')
    axs[2, 0].set_xticks([-1, -0.5, 0, 0.5])
    axs[2, 1].set_xticks([-1, 0, 1, 2, 3])
    for ax, letter in zip(axs.T.ravel(), 'abcdef'):
        ps.panel_label(ax, letter)
    fig.align_ylabels(axs[:, 0])
    paths = ps.save_print(fig, os.path.join(OUT, 'F_magnet_anatomy', 'magnet_anatomy'))
    print('\n'.join(paths))


FIGS = {'carry': fig_carry, 'releases': fig_releases, 'magnet': fig_magnet}

if __name__ == '__main__':
    for name in sys.argv[1:] or FIGS:
        FIGS[name]()

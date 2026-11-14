"""Admission and timing figures for Te-Jen's planner (open-loop real-time replays of 12 Nov flights).
Reads ../../cpp_realtime_2026-11-13/full/runs/<run>/{replans,backend,driver}.csv and backend.yaml (read only).
usage: python3 make_timing_figures.py     (writes the PNGs next to this script)
Time axis everywhere: seconds since his authority grant (driver.csv first auth == 1)."""
import os
import re

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Patch
import numpy as np
import pandas as pd
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
RUNS = os.path.normpath(os.path.join(HERE, '../../cpp_realtime_2026-11-13/full/runs'))
plt.rcParams.update({'figure.facecolor': 'white', 'axes.facecolor': 'white', 'savefig.facecolor': 'white',
                     'font.size': 9, 'axes.titlesize': 9.5, 'axes.grid': True, 'grid.alpha': 0.3})

# status groups: one colour each, the same in every figure
GROUPS = [
    ('accepted (complete / moving rendezvous / goal seen)', '#2ca02c'),
    ('accepted partial', '#a6d96a'),
    ('WORLD_CHANGED_DURING_FINAL_RECHECK', '#ff7f0e'),
    ('LATE_CANDIDATE_REJECTED(_AFTER_RECHECK)', '#d62728'),
    ('LOCAL_PLAN_FAILED:SOLVER_FAILED', '#6a3d9a'),
    ('LOCAL_PLAN_FAILED:PADDED_CLOSEST_PARTIAL_FINAL_CHECK_FAILED', '#cab2d6'),
    ('INCUMBENT_..._UNSAFE (remaining or prefix)', '#8c510a'),
    ('TARGET_TIME_FIXED_POINT_NOT_CONVERGED', '#1f78b4'),
    ('LATEST_CANDIDATE_UNSAFE(_AFTER_RETRY)', '#f781bf'),
    ('other (INVALID_INITIAL_Q2, PREFIX_CORRIDOR_INCOMPATIBLE, ...)', '#999999'),
]
GCOL = dict(GROUPS)
GNAMES = [g for g, _ in GROUPS]


def group(s):
    s = str(s)
    if s.startswith('ACCEPTED'):
        return GNAMES[1] if 'PARTIAL' in s else GNAMES[0]
    if s == 'WORLD_CHANGED_DURING_FINAL_RECHECK':
        return GNAMES[2]
    if s.startswith('LATE_CANDIDATE_REJECTED'):
        return GNAMES[3]
    if s == 'LOCAL_PLAN_FAILED:SOLVER_FAILED':
        return GNAMES[4]
    if s == 'LOCAL_PLAN_FAILED:PADDED_CLOSEST_PARTIAL_FINAL_CHECK_FAILED':
        return GNAMES[5]
    if s.startswith('INCUMBENT_'):
        return GNAMES[6]
    if s == 'TARGET_TIME_FIXED_POINT_NOT_CONVERGED':
        return GNAMES[7]
    if s.startswith('LATEST_CANDIDATE_UNSAFE'):
        return GNAMES[8]
    return GNAMES[9]


def params(run):
    txt = open(f'{RUNS}/{run}/backend.yaml').read()
    def get(k, default):
        m = re.search(rf'^\s*{k}:\s*([^\s#]+)', txt, re.M)
        return m.group(1) if m else default
    return {'splice': float(get('fixed_splice_lookahead_s', 'nan')),
            'refresh': get('cooperative_world_refresh_period_s', 'off'),
            'cap': get('maximum_search_z_m', 'default'),
            'shape': get('swing_envelope_shape', '?'),
            'spline': get('spline_time_factor', '?')}


_cache = {}


def load(run):
    if run in _cache:
        return _cache[run]
    d = f'{RUNS}/{run}'
    rp = pd.read_csv(f'{d}/replans.csv')
    dr = pd.read_csv(f'{d}/driver.csv')
    be = pd.read_csv(f'{d}/backend.csv')
    auth = dr.t[dr.auth == 1]
    t_auth = float(auth.iloc[0]) if len(auth) else float(dr.t.iloc[0])
    cap = dr.t[dr.captured == 1]
    t_cap = float(cap.iloc[0]) - t_auth if len(cap) else None
    rp['tt'] = rp.request_ros_time - t_auth
    rp['grp'] = rp.status.map(group)
    dr['tt'] = dr.t - t_auth
    be['tt'] = be.ros_time - t_auth
    t_end = float(dr.tt.iloc[-1])
    p = params(run)
    _cache[run] = dict(rp=rp, dr=dr, be=be, t_cap=t_cap, t_end=t_end, p=p, name=run)
    return _cache[run]


def window(R):
    """replans from the authority grant to the capture (or the run's end)"""
    rp = R['rp']
    end = R['t_cap'] if R['t_cap'] is not None else R['t_end']
    return rp[(rp.tt >= 0) & (rp.tt <= end)]


def label(run, extra=''):
    R = load(run)
    p = R['p']
    load_n = re.search(r'load(\d+)', run)
    s = (f"{run}: splice {p['splice']:.2f} s, refresh {p['refresh']}, cap {p['cap']}, {p['shape']}"
         + (f", {load_n.group(1)} busy loops" if load_n else ''))
    return s + extra


def capstr(R):
    return f"captured at {R['t_cap']:.1f} s" if R['t_cap'] is not None else 'not captured'


def status_strip(ax, R, xlim):
    rp = R['rp']
    for g in GNAMES:
        sel = rp[rp.grp == g]
        if len(sel):
            ax.vlines(sel.tt, 0, 1, color=GCOL[g], lw=1.6)
    ax.set_ylim(0, 1)
    ax.set_yticks([])
    ax.set_xlim(*xlim)
    ax.axvspan(xlim[0], 0, color='0.85', zorder=-1)
    ax.grid(False)


def mark_capture(ax, R):
    if R['t_cap'] is not None:
        ax.axvline(R['t_cap'], color='k', lw=1.4, ls='--')


def legend_patches(groups=None):
    return [Patch(color=GCOL[g], label=g) for g in (groups or GNAMES)]


def save(fig, name):
    fig.savefig(os.path.join(HERE, name), dpi=150, bbox_inches='tight')
    plt.close(fig)
    print('wrote', name)


# ---------------------------------------------------------------- figure 1: status strips + distance to target
def fig1(runs, name, title):
    n = len(runs)
    fig, axes = plt.subplots(2 * n, 1, figsize=(11, 2.35 * n + 1.2), sharex=True,
                             gridspec_kw={'height_ratios': [0.55, 1.6] * n})
    xlim = (-2.5, max(load(r)['t_end'] for r in runs))
    for i, run in enumerate(runs):
        R = load(run)
        a, b = axes[2 * i], axes[2 * i + 1]
        status_strip(a, R, xlim)
        mark_capture(a, R)
        a.set_title(label(run) + f'  ({capStr(R)})', loc='left', fontsize=8.5)
        a.set_ylabel('replan\nstatus', fontsize=7.5)
        dr = R['dr']
        b.plot(dr.tt, dr.dxy, color='k', lw=1.1, label='horizontal distance to target dxy')
        b.plot(dr.tt, dr.dz.abs(), color='0.55', lw=0.9, ls='-', label='|dz| to target')
        mark_capture(b, R)
        b.set_ylabel('distance (m)')
        b.set_ylim(0, max(1.6, float(dr.dxy[dr.tt > 0].max()) * 1.05))
        b.set_xlim(*xlim)
        if i == 0:
            b.legend(loc='upper right', fontsize=7.5, framealpha=0.9)
    axes[-1].set_xlabel('time since his authority grant (s); grey band = before the grant; dashed line = capture')
    fig.suptitle(title, x=0.01, ha='left', fontsize=10.5, y=0.995)
    present = sorted({g for r in runs for g in load(r)['rp'].grp}, key=GNAMES.index)
    fig.legend(handles=legend_patches(present), loc='lower center', ncol=2, fontsize=7.5,
               bbox_to_anchor=(0.5, -0.02 - 0.012 * len(present)), frameon=True)
    fig.tight_layout(rect=(0, 0, 1, 0.985))
    save(fig, name)


def capStr(R):
    return capstr(R)


# ---------------------------------------------------------------- figure 2: replan runtime vs splice
def fig2a():
    runs = ['fm9_5_z29', 'fm9_5_z29_load8', 'fm9_5_A_load8_3', 'fm9_5_B_load8_1']
    fig, axes = plt.subplots(len(runs), 1, figsize=(11, 2.3 * len(runs) + 0.6), sharex=True, sharey=True)
    for ax, run in zip(axes, runs):
        R = load(run)
        rp = R['rp']
        rp = rp[rp.tt <= (R['t_cap'] if R['t_cap'] is not None else R['t_end'])]
        late = rp.candidate_late == 1
        ax.scatter(rp.tt[~late], rp.replan_runtime_ms[~late], s=9, color='#4d4d4d', label='candidate in time')
        ax.scatter(rp.tt[late], rp.replan_runtime_ms[late], s=22, marker='x', color=GCOL[GNAMES[3]],
                   label='candidate_late = 1')
        sp = R['p']['splice'] * 1e3
        ax.axhline(sp, color='#1f78b4', lw=1.5, label=f'fixed_splice_lookahead_s = {sp / 1e3:.2f} s')
        mark_capture(ax, R)
        w = window(R)
        ax.set_title(label(run) + f'  ({capStr(R)}; late {int((w.candidate_late == 1).sum())} of {len(w)} '
                     f'replans grant-to-capture/end)', loc='left', fontsize=8.5)
        ax.set_ylabel('replan_runtime_ms')
        ax.legend(loc='upper right', fontsize=7.5, ncol=3, framealpha=0.9)
    axes[0].set_ylim(0, 260)
    axes[-1].set_xlabel('time since his authority grant (s); dashed line = capture (still ring, replays of fm9_5)')
    fig.suptitle('Replan runtime against the splice lookahead: a candidate that takes longer than the splice is '
                 'rejected as late', x=0.01, ha='left', fontsize=10.5)
    fig.tight_layout()
    save(fig, 'fig2a_runtime_vs_time_splice.png')


def early(R, t1=16.0):
    """replans in the first t1 s after the grant (all runs still approaching; captures came at 15.8-21.6 s)"""
    rp = R['rp']
    return rp[(rp.tt >= 0) & (rp.tt <= t1)]


def cdf(ax, vals, **kw):
    v = np.sort(np.asarray(vals))
    ax.step(v, np.arange(1, len(v) + 1) / len(v), where='post', **kw)


def fig2b():
    fig, (a, b) = plt.subplots(1, 2, figsize=(12, 4.8), sharey=True)
    ladder = [('fm9_5_z29', 0), ('fm9_5_z29_load4', 4), ('fm9_5_z29_load6', 6), ('fm9_5_z29_load8', 8),
              ('fm9_5_z29_load11', 11)]
    cols = plt.cm.viridis(np.linspace(0, 0.9, len(ladder)))
    for (run, nl), c in zip(ladder, cols):
        R = load(run)
        w = early(R)
        cdf(a, w.replan_runtime_ms, color=c, lw=1.6,
            label=f'{nl} busy loops ({run}): late {100 * (w.candidate_late == 1).mean():.0f} % (first 16 s), {capStr(R)}')
    a.axvline(100, color='#1f78b4', lw=1.5, ls='-')
    a.text(102, 1.01, 'splice 0.10 s', color='#1f78b4', fontsize=8)
    a.set_title('(a) splice 0.10 s, cap 2.9 m, refresh 0.2 s: CPU load shifts the runtime past the splice',
                loc='left', fontsize=8.8)
    a.set_xlabel('replan_runtime_ms (ms), replans in the first 16 s after the grant')
    a.set_ylabel('fraction of replans with runtime <= x')
    a.legend(loc='lower right', fontsize=7)
    cfgs = [('splice 0.10 s', ['fm9_5_z29_load8'], '#d62728', 100),
            ('splice 0.15 s', [f'fm9_5_A_load8_{i}' for i in range(1, 6)], '#ff7f0e', 150),
            ('splice 0.20 s', ['fm9_5_B_load8_1', 'fm9_5_B_load8_2'], '#2ca02c', 200)]
    for nm, runs, c, sp in cfgs:
        ws = [early(load(r)) for r in runs]
        allw = pd.concat(ws)
        ncap = sum(load(r)['t_cap'] is not None for r in runs)
        cdf(b, allw.replan_runtime_ms, color=c, lw=1.8,
            label=f'{nm} ({len(runs)} run{"s pooled" if len(runs) > 1 else ""}): late '
                  f'{100 * (allw.candidate_late == 1).mean():.1f} % (first 16 s), captured {ncap} of {len(runs)}')
        b.axvline(sp, color=c, lw=1.2, ls='--')
    b.set_title('(b) 8 busy loops, cap 2.9 m, refresh 0.2 s: same runtimes, a longer splice admits them',
                loc='left', fontsize=8.8)
    b.set_xlabel('replan_runtime_ms (ms), first 16 s after the grant; dashed = splice')
    b.legend(loc='lower right', fontsize=7)
    for ax in (a, b):
        ax.set_xlim(0, 300)
    fig.suptitle('Distribution of replan runtime per configuration, still ring (fm9_5 replays), with the splice '
                 'lookahead drawn', x=0.01, ha='left', fontsize=10.5)
    fig.tight_layout()
    save(fig, 'fig2b_runtime_cdf_load_and_splice.png')


# ---------------------------------------------------------------- figure 3: world version rate and final recheck
def fig3():
    runs = [('fm9_5', 'v0 (no coalescing)'), ('fm9hc_2', 'v0 (no coalescing)'),
            ('fm9_5_r02', 'refresh 0.2 s'), ('fm9_5_B_load8_1', 'refresh 0.2 s, 8 busy loops')]
    cols = ['#e6550d', '#fdae6b', '#3182bd', '#9ecae1']
    fig = plt.figure(figsize=(12, 8.2))
    gs = fig.add_gridspec(2, 2, height_ratios=[1, 1.1])
    a = fig.add_subplot(gs[0, :])
    for (run, nm), c in zip(runs, cols):
        R = load(run)
        be = R['be'].dropna(subset=['world_version'])
        be = be[be.tt > -2]
        rate = be.world_version.diff() / be.ros_time.diff()
        a.plot(be.tt, rate, color=c, lw=1.6, marker='o', ms=2.5, label=f'{run} ({nm}): world_version, {capStr(R)}')
        if R['p']['refresh'] != 'off':
            rr = be.cooperative_world_refresh_count.diff() / be.ros_time.diff()
            a.plot(be.tt, rr, color=c, lw=1.0, ls='--', label=f'{run}: cooperative_world_refresh_count')
    a.set_yscale('log')
    a.set_ylim(1, 8000)
    a.set_ylabel('increments per second\n(backend.csv, sampled at 1 Hz)')
    a.set_xlabel('time since his authority grant (s)')
    a.set_title('(a) how fast his collision world changes: about 214 versions/s without coalescing; with the 0.2 s refresh '
                'about 32 versions/s, from 5 refreshes/s (dashed)', loc='left', fontsize=9)
    a.legend(fontsize=6.8, loc='upper center', ncol=2)
    b = fig.add_subplot(gs[1, 0])
    data, labs = [], []
    for (run, nm), c in zip(runs, cols):
        w = window(load(run))
        ok = (w.recheck_world_version > 0) & (w.planning_world_version > 0)
        data.append((w.recheck_world_version - w.planning_world_version)[ok].values)
        labs.append(f'{run}\n{nm}')
    bp = b.boxplot(data, tick_labels=labs, showfliers=True, patch_artist=True, widths=0.55)
    for patch, c in zip(bp['boxes'], cols):
        patch.set_facecolor(c)
    b.set_ylabel('recheck_world_version - planning_world_version\n(versions while one candidate was planned)')
    b.set_title('(b) world versions that arrive while ONE candidate is planned', loc='left', fontsize=9)
    b.tick_params(axis='x', labelsize=7)
    c_ax = fig.add_subplot(gs[1, 1])
    cats = [('first recheck only (recheck_change_kind INITIAL_FULL)', '#bdbdbd'),
            ('retry recheck, then committed or other outcome', '#80b1d3'),
            ('retry recheck, world changed again: WORLD_CHANGED_DURING_FINAL_RECHECK', GCOL[GNAMES[2]])]
    xs = np.arange(len(runs))
    bottom = np.zeros(len(runs))
    for k, (cn, cc) in enumerate(cats):
        vals = []
        for run, _ in runs:
            w = window(load(run))
            retry = w.recheck_change_kind != 'INITIAL_FULL'
            wc = w.status == 'WORLD_CHANGED_DURING_FINAL_RECHECK'
            sel = [~retry, retry & ~wc, wc][k]
            vals.append(100 * sel.mean())
        c_ax.bar(xs, vals, bottom=bottom, color=cc, label=cn, width=0.6)
        for x, v, b0 in zip(xs, vals, bottom):
            if v >= 4:
                c_ax.text(x, b0 + v / 2, f'{v:.0f}%', ha='center', va='center', fontsize=7.5)
        bottom += vals
    c_ax.set_xticks(xs)
    c_ax.set_xticklabels([f'{r}\n{n}' for r, n in runs], fontsize=7)
    c_ax.set_ylabel('% of replans (grant to capture/end)')
    c_ax.set_ylim(0, 100)
    c_ax.set_title('(c) outcome of his final recheck', loc='left', fontsize=9)
    c_ax.legend(fontsize=6.8, loc='upper center', bbox_to_anchor=(0.5, -0.2), frameon=True)
    fig.suptitle('World changes during planning and the final recheck: v0 (every message a new world) vs the '
                 '0.2 s refresh coalescing, still ring', x=0.01, ha='left', fontsize=10.5)
    fig.tight_layout()
    save(fig, 'fig3_world_version_rate_and_recheck.png')


# ---------------------------------------------------------------- figure 4: status share per configuration
def fig4():
    runs = ['fm9_5', 'fm9hc_2', 'fm9_5_r02', 'fm9_5_latch_legacy', 'fm9_5_z29', 'fm9_5_z29_load8',
            'fm9_5_A_load8_1', 'fm9_5_A_load8_3', 'fm9_5_B_load8_1', 'nro_p02', 'nro_mv_z29s20']
    fig, ax = plt.subplots(figsize=(12.5, 6.2))
    xs = np.arange(len(runs))
    bottom = np.zeros(len(runs))
    shares = {g: [] for g in GNAMES}
    for run in runs:
        w = window(load(run))
        vc = w.grp.value_counts(normalize=True)
        for g in GNAMES:
            shares[g].append(100 * vc.get(g, 0.0))
    for g in GNAMES:
        v = np.array(shares[g])
        if v.sum() == 0:
            continue
        ax.bar(xs, v, bottom=bottom, color=GCOL[g], width=0.7, label=g, edgecolor='white', lw=0.4)
        for x, vi, b0 in zip(xs, v, bottom):
            if vi >= 6:
                ax.text(x, b0 + vi / 2, f'{vi:.0f}', ha='center', va='center', fontsize=7)
        bottom += v
    ticks = []
    for run in runs:
        R = load(run)
        p = R['p']
        ln = re.search(r'load(\d+)', run)
        ring = 'MOVING ring' if run.startswith('nro') else 'still ring'
        ticks.append(f"{run}\nsplice {p['splice']:.2f}\nrefresh {p['refresh']}\ncap {p['cap']}\n"
                     f"{'legacy_square' if p['shape'] == 'legacy_square' else 'octagon'}"
                     f"{', load ' + ln.group(1) if ln else ''}\n{ring}\n{'CAPTURED' if R['t_cap'] else 'not captured'}"
                     f"\nn={len(window(R))}")
    ax.set_xticks(xs)
    ax.set_xticklabels(ticks, fontsize=6.6)
    ax.set_ylabel('% of replans (grant to capture or end)')
    ax.set_ylim(0, 100)
    ax.grid(axis='x', visible=False)
    ax.set_title('Share of replan statuses per configuration (one replay each, window: authority grant to capture '
                 'or end; numbers = %)', loc='left', fontsize=10.5)
    ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.36), ncol=2, fontsize=7.5)
    fig.tight_layout()
    save(fig, 'fig4_status_share_per_configuration.png')


# ---------------------------------------------------------------- figure 5: moving ring, splice 0.10 vs 0.20
def fig5():
    runs = ['nro_p02', 'nro_mv_z29s20']
    fig, axes = plt.subplots(5, 2, figsize=(14, 10.5), sharex='col',
                             gridspec_kw={'height_ratios': [0.45, 1.1, 1.1, 0.9, 1.0]})
    for j, run in enumerate(runs):
        R = load(run)
        dr, be = R['dr'], R['be']
        xlim = (-2.5, R['t_end'])
        a = axes[0, j]
        status_strip(a, R, xlim)
        mark_capture(a, R)
        a.set_title(label(run) + f'\nMOVING ring (nro, 90 s circle), {capStr(R)}', loc='left', fontsize=8.5)
        if j == 0:
            a.set_ylabel('replan\nstatus', fontsize=7.5)
        b = axes[1, j]
        b.plot(dr.tt, dr.dxy, color='k', lw=1.1, label='dxy to target')
        b.plot(dr.tt, dr.dz.abs(), color='0.55', lw=0.9, label='|dz| to target')
        b.set_ylim(0, 2.0)
        b.legend(fontsize=7, loc='upper right')
        c = axes[2, j]
        c.plot(dr.tt, dr.min_cable_tether_m, color='#d62728', lw=1.1, label='min_cable_tether_m (QUAD4 tether to a carrier tether)')
        c.plot(dr.tt, dr.min_body_m, color='#1f78b4', lw=1.0, label='min_body_m (body to body)')
        c.set_ylim(0, 1.5)
        c.legend(fontsize=7, loc='upper right')
        d = axes[3, j]
        spd = np.hypot(dr.vx, dr.vy)
        d.plot(dr.tt, spd, color='#555555', lw=0.9, label='QUAD4 reference horizontal speed')
        d.axhline(0.05, color='0.5', ls=':', lw=1)
        d.set_ylim(0, max(0.6, float(spd.quantile(0.995)) * 1.1))
        d.legend(fontsize=7, loc='upper right')
        e = axes[4, j]
        left = (be.commit_end_s - be.ros_time).where(be.has_commit == 1)
        e.plot(be.tt, left, color='#2ca02c', lw=1.0, label='commit_end_s - now (plan time left)')
        fb = be.execution_fallback_active == 1
        e.fill_between(be.tt, -2, 12, where=fb, color='#8c510a', alpha=0.25, step='post',
                       label='execution_fallback_active = 1')
        e.axhline(0, color='k', lw=0.8)
        e.set_ylim(-2, 12)
        e.legend(fontsize=7, loc='upper right')
        e.set_xlabel('time since his authority grant (s)')
        for k, ax in enumerate(axes[1:, j]):
            mark_capture(ax, R)
            ax.set_xlim(*xlim)
            if R['t_cap'] is not None:
                ax.axvspan(R['t_cap'], xlim[1], color='0.88', zorder=-2)
                if k == 2:
                    ax.text(R['t_cap'] + 0.8, 0.42, 'after capture: the open-loop replay goes on\n(not a physical state: '
                            'QUAD4 would be on the ring)', fontsize=7, color='0.3', va='bottom')
        if j == 0:
            b.set_ylabel('distance (m)')
            c.set_ylabel('clearance (m)\ncentre lines')
            d.set_ylabel('speed (m/s)')
            e.set_ylabel('commit left (s)')
    present = sorted({g for r in runs for g in load(r)['rp'].grp}, key=GNAMES.index)
    fig.legend(handles=legend_patches(present), loc='lower center', ncol=3, fontsize=7.5,
               bbox_to_anchor=(0.5, -0.06), frameon=True)
    fig.suptitle('Moving ring: what his planner did with splice 0.10 s (left, parked, never captured) vs 0.20 s '
                 '(right); dotted line = 0.05 m/s "still"', x=0.01, ha='left', fontsize=10.5)
    fig.tight_layout(rect=(0, 0, 1, 0.98))
    save(fig, 'fig5_moving_ring_splice010_vs_020.png')


# ---------------------------------------------------------------- figure 6: where the replan time goes
def fig6():
    runs = ['fm9_5_latch_legacy', 'fm9_5', 'fm9_5_r02', 'fm9_5_z29', 'fm9_5_z29_load8', 'fm9_5_A_load8_3',
            'fm9_5_B_load8_1', 'nro_p02', 'nro_mv_z29s20']
    comps = [('world_snapshot_ms', '#bdbdbd'), ('obstacle_build_ms', '#fdae6b'), ('octopus_search_ms', '#6baed6'),
             ('qp_solve_ms', '#9e9ac8'), ('safety_check_ms', '#e6550d')]
    fig, (a, b) = plt.subplots(2, 1, figsize=(12.5, 9.5), gridspec_kw={'height_ratios': [1.15, 1]})
    xs = np.arange(len(runs))
    bottom = np.zeros(len(runs))
    for k, c in comps:
        v = np.array([window(load(r))[k].mean() for r in runs])
        a.bar(xs, v, bottom=bottom, color=c, width=0.65, label=f'mean {k}')
        for x, vi, b0 in zip(xs, v, bottom):
            if vi >= 8:
                a.text(x, b0 + vi / 2, f'{vi:.0f}', ha='center', va='center', fontsize=7)
        bottom += v
    tot = np.array([window(load(r)).replan_runtime_ms.mean() for r in runs])
    a.bar(xs, tot - bottom, bottom=bottom, color='white', edgecolor='0.4', hatch='//', width=0.65,
          label='rest of mean replan_runtime_ms')
    p95 = np.array([window(load(r)).replan_runtime_ms.quantile(0.95) for r in runs])
    a.scatter(xs, p95, marker='v', color='k', zorder=5, label='p95 replan_runtime_ms')
    for x, r in zip(xs, runs):
        sp = load(r)['p']['splice'] * 1e3
        a.hlines(sp, x - 0.38, x + 0.38, color='#1f78b4', lw=2.2, zorder=6,
                 label='fixed_splice_lookahead_s' if x == 0 else None)
    ticks = []
    for r in runs:
        p = load(r)['p']
        ln = re.search(r'load(\d+)', r)
        ticks.append(f"{r}\nsplice {p['splice']:.2f}, refresh {p['refresh']}\n"
                     f"{'legacy_square' if p['shape'] == 'legacy_square' else 'octagon'}{', load ' + ln.group(1) if ln else ''}")
    a.set_xticks(xs)
    a.set_xticklabels(ticks, fontsize=6.8)
    a.set_ylabel('time per replan (ms)')
    a.grid(axis='x', visible=False)
    a.legend(fontsize=7, ncol=4, loc='upper left')
    a.set_ylim(0, 260)
    a.set_title('(a) where one replan spends its time (means, grant to capture/end): with outer_octagon the safety check '
                'is the largest part, not the search', loc='left', fontsize=9)
    # (b) installation splice margin: splice time - install time, for installed candidates
    data, labs = [], []
    for r in runs:
        w = window(load(r))
        ok = (w.authority_installed == 1) & w.installation_splice_margin_ms.notna() \
            & w.installation_attempt_time_s.notna() & w.authority_install_time_s.notna()
        m = 1e3 * (w.installation_attempt_time_s + 1e-3 * w.installation_splice_margin_ms
                   - w.authority_install_time_s)[ok]
        data.append(m.values)
        labs.append(f'{r}\n(n={ok.sum()})')
    for i, (dv, r) in enumerate(zip(data, runs)):
        jit = (np.random.default_rng(i).random(len(dv)) - 0.5) * 0.5
        b.scatter(np.full(len(dv), i) + jit, dv, s=6, alpha=0.5, color='#3182bd')
        if len(dv):
            b.hlines(np.median(dv), i - 0.3, i + 0.3, color='k', lw=1.5)
            b.text(i, -12, f'min {np.min(dv):.0f}', ha='center', fontsize=6.8)
    b.axhline(0, color='#d62728', lw=1)
    b.set_xticks(xs)
    b.set_xticklabels(labs, fontsize=6.8)
    b.set_ylim(-20, 210)
    b.set_ylabel('installed splice margin (ms)\n= splice time - authority_install_time_s')
    b.grid(axis='x', visible=False)
    b.set_title('(b) how much time was left before the splice when a candidate was installed (each dot = one installed '
                'candidate; bar = median)', loc='left', fontsize=9)
    fig.suptitle('Replan time budget and installation margin per configuration', x=0.01, ha='left', fontsize=10.5)
    fig.tight_layout()
    save(fig, 'fig6_time_budget_and_install_margin.png')


# ---------------------------------------------------------------- figure 7: when progress stops (commit left, last install)
def fig7():
    runs = [('fm9_5', '#e6550d'), ('fm9_5_r02', '#3182bd'), ('fm9_5_latch_legacy', '#8c510a'),
            ('fm9_5_z29_load8', '#d62728'), ('fm9_5_B_load8_1', '#2ca02c')]
    fig, (a, b) = plt.subplots(2, 1, figsize=(11.5, 8.2), sharex=True)
    for run, c in runs:
        R = load(run)
        be = R['be']
        left = (be.commit_end_s - be.ros_time).where(be.has_commit == 1)
        a.plot(be.tt, left, color=c, lw=1.5, label=label(run) + f' ({capStr(R)})')
        fb = be.execution_fallback_active == 1
        if fb.any():
            a.plot(be.tt[fb], left[fb], color=c, lw=0, marker='s', ms=4)
        rp = R['rp']
        inst = rp.request_ros_time[rp.authority_installed == 1].values - (rp.request_ros_time - rp.tt).iloc[0]
        tgrid = np.arange(0, R['t_end'], 0.1)
        idx = np.searchsorted(inst, tgrid, side='right') - 1
        since = np.where(idx >= 0, tgrid - inst[np.clip(idx, 0, None)], np.nan)
        b.plot(tgrid, since, color=c, lw=1.4, label=run)
        for ax in (a, b):
            if R['t_cap'] is not None:
                ax.axvline(R['t_cap'], color=c, ls='--', lw=1.1)
    a.axhline(0, color='k', lw=0.8)
    a.set_ylabel('commit_end_s - now (s)\n(time left on the installed plan)')
    a.set_title('(a) plan time left; squares = execution_fallback_active = 1 (SMOOTH_BRAKE in fm9_5_z29_load8); '
                'dashed = capture', loc='left', fontsize=9)
    a.legend(fontsize=7, loc='lower left')
    b.set_ylabel('time since the last installed\ncandidate (s)')
    b.set_xlabel('time since his authority grant (s); request time of the last replan with authority_installed = 1')
    b.set_title('(b) how long since a new plan was last installed: progress stops where this grows without reset',
                loc='left', fontsize=9)
    b.legend(fontsize=7.5, loc='upper left')
    fig.suptitle('When his planner stops making progress, still ring: the installed plan runs out when no new '
                 'candidate is admitted', x=0.01, ha='left', fontsize=10.5)
    fig.tight_layout()
    save(fig, 'fig7_commit_left_and_last_install.png')


if __name__ == '__main__':
    fig1(['fm9_5', 'fm9hc_2', 'fm9_5_r02', 'fm9_5_latch_legacy'], 'fig1a_status_strip_still_ring.png',
         'Replan status over time and distance to the target, still ring, no CPU load: v0 (no coalescing) vs '
         'the 0.2 s refresh vs legacy_square')
    fig1(['fm9_5_z29_load8', 'fm9_5_A_load8_1', 'fm9_5_A_load8_3', 'fm9_5_B_load8_1'],
         'fig1b_status_strip_under_load.png',
         'Replan status over time and distance to the target, still ring, 8 busy CPU loops: splice 0.10 vs 0.15 '
         'vs 0.20 s')
    fig2a()
    fig2b()
    fig3()
    fig4()
    fig5()
    fig6()
    fig7()

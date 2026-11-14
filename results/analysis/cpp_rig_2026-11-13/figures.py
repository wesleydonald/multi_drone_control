#!/usr/bin/env python3
"""Figures from metrics.csv and series/*.npz: a top view + clearance panel per FROZEN-line flight, and one summary of
the minimum clearance and the C++ transit time over all 14 flights, coloured by outcome."""
import json
import os
import sys

import numpy as np
import pandas as pd

from common import FLIGHTS, FROZEN, HERE, R_PLATE, ROOT, plate_pos

sys.path.insert(0, os.path.join(ROOT, 'tools'))
import plot_style  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402

plot_style.use()
plt.rcParams.update({'axes.spines.top': True, 'axes.spines.right': True, 'legend.frameon': True,
                     'legend.edgecolor': '#444444', 'legend.fancybox': False})
OUT = os.path.join(HERE, 'figures')
os.makedirs(OUT, exist_ok=True)
Q4 = plot_style.drone_colour(3)
OUTCOME_HEX = {'PASS': '#1f77b4', 'PARTIAL': '#ff7f0e', 'FAIL': '#d62728', 'ABORT': '#7f7f7f'}
TRAJ = {'fig_8': 'figure-eight', 'orbit': 'circle', 'hover': 'hover'}


def legend_below(ax, ncol=3, y=-0.16):
    h, lab = ax.get_legend_handles_labels()
    return ax.legend(h, lab, loc='upper center', bbox_to_anchor=(0.5, y), ncol=ncol, fontsize=7.5)


def flight_figure(name, row):
    s = np.load(os.path.join(HERE, 'series', name + '.npz'))
    ev = json.loads(str(s['ev']))
    t = s['t']
    tr, ce, st, dr = ev['transit'], ev['cpp_end'], ev['settle'], ev['drop']
    fig, (ax, bx) = plt.subplots(1, 2, figsize=(10.0, 4.6), gridspec_kw={'width_ratios': [1.0, 1.25]})
    # top view: C++ transit and settle + descent, until the drop
    m_c = (t >= tr) & (t <= ce)
    m_p = (t >= ce) & (t <= dr)
    m_all = (t >= tr) & (t <= dr)
    for i in range(3):
        p = s[f'd{i}']
        ax.plot(p[m_all, 0], p[m_all, 1], color=plot_style.drone_colour(i), lw=0.8, alpha=0.45,
                label=f'carrier d{i} (plate {int(s["plates"][i])})')
    r = s['ring']
    ax.plot(r[m_all, 0], r[m_all, 1], color='k', lw=1.4, label='ring centre')
    ax.plot(s['d3'][m_c, 0], s['d3'][m_c, 1], color=Q4, lw=1.8, label='QUAD4, C++ transit')
    ax.plot(s['d3'][m_p, 0], s['d3'][m_p, 1], color=Q4, lw=1.4, ls='--', label='QUAD4, settle + descent (Python)')
    ax.plot(s['obj'][m_p, 0], s['obj'][m_p, 1], color='#9467bd', lw=0.9, ls=':', label='object, settle + descent')
    for tt, mk, lab in ((tr, 'o', 'C++ start'), (ce, 's', 'captured'), (dr, '^', 'DROP_OBJECT')):
        j = np.argmin(np.abs(t - tt))
        ax.plot(*s['d3'][j, :2], mk, color=Q4, ms=8, mfc='white', mew=1.6, zorder=6)
        ax.plot(*r[j, :2], mk, color='k', ms=6, mfc='white', mew=1.2, zorder=5)
        # the ring outline and the carriers' cables (top view) at that moment
        ang = np.linspace(0, 2 * np.pi, 80)
        ax.plot(r[j, 0] + R_PLATE * np.cos(ang), r[j, 1] + R_PLATE * np.sin(ang), color='k', lw=0.6, alpha=0.5)
        for i in range(3):
            pl = plate_pos(r[j:j + 1], int(s['plates'][i]))[0]
            if tt != tr:  # carrier cables (top view) at the capture and at the drop
                ax.plot([s[f'd{i}'][j, 0], pl[0]], [s[f'd{i}'][j, 1], pl[1]], color=plot_style.drone_colour(i),
                        lw=1.0, alpha=0.8)
            ax.plot(*s[f'd{i}'][j, :2], mk, color=plot_style.drone_colour(i), ms=5, mfc='white', mew=1.0)
        ax.plot([], [], mk, color='k', mfc='white', ms=5, label=lab)
    ax.set_aspect('equal', adjustable='datalim')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_title('Top view, C++ start to the drop (cables at capture and drop)', fontsize=9)
    legend_below(ax, ncol=2, y=-0.14)
    # clearance against time
    w = (t >= tr - 2.0) & (t <= dr + 2.0)
    cab = np.nanmin(s['cab'], axis=1)
    ob = np.nanmin(np.where(np.isnan(s['ob']), np.inf, s['ob']), axis=1)
    ob[~np.isfinite(ob)] = np.nan
    bx.axvspan(tr, ce, color=Q4, alpha=0.08, lw=0, label='C++ transit')
    bx.axvspan(ce, dr, color='#1f77b4', alpha=0.08, lw=0, label='settle + descent (Python)')
    bx.plot(t[w], cab[w], color=Q4, lw=1.4, label='QUAD4 cable to nearest carrier cable')
    bx.plot(t[w], ob[w], color='#9467bd', lw=1.2, label='object to nearest carrier cable (minus 0.0525 m)')
    bx.axvline(dr, color='k', ls=':', lw=1.0, label='DROP_OBJECT')
    bx.set_xlim(tr - 2.0, dr + 2.0)
    bx.set_ylim(bottom=0.0)
    bx.set_xlabel('time after his TAKEOFF (s)')
    bx.set_ylabel('clearance, centre lines (m)')
    bx.set_title('Clearance to the carrier cables', fontsize=9)
    legend_below(bx, ncol=2, y=-0.14)
    what = 'over the hovering ring' if row.traj == 'hover' else f'into the moving {TRAJ.get(row.traj, row.traj)}'
    fig.suptitle(f'{name}: his C++ planner {what}, Python settle and descent ({row.outcome})', fontsize=10)
    fig.tight_layout()
    for ext in ('png', 'pdf'):
        fig.savefig(os.path.join(OUT, f'{name}_topview_clearance.{ext}'))
    plt.close(fig)


def summary(df):
    df = df.set_index('flight').loc[FLIGHTS].reset_index()
    x = np.arange(len(df))
    fig, (ax, bx) = plt.subplots(2, 1, figsize=(9.0, 6.4), sharex=True)
    for k, r in df.iterrows():
        c = OUTCOME_HEX[r.outcome]
        if np.isfinite(r.min_q4cable_cpp_m):
            ax.plot(x[k] - 0.12, r.min_q4cable_cpp_m, 'o', color=c, ms=7)
            ax.plot(x[k] + 0.12, r.min_obj_cpp_m, 'D', color=c, ms=6, mfc='white', mew=1.4)
        else:
            ax.text(x[k], 0.02, 'no C++', ha='center', va='bottom', fontsize=7, rotation=90, color=c)
        if np.isfinite(r.cpp_transit_s):
            bx.bar(x[k], r.cpp_transit_s, color=c, width=0.6, hatch=None if r.captured else '///',
                   edgecolor='k' if not r.captured else c, lw=0.6)
            bx.text(x[k], r.cpp_transit_s + 0.8, f'{r.cpp_transit_s:.1f}', ha='center', fontsize=7)
        else:
            bx.text(x[k], 1.0, 'no C++', ha='center', va='bottom', fontsize=7, rotation=90, color=c)
    ax.axhline(0.0, color='k', lw=0.8)
    ax.set_ylabel('min clearance in the\nC++ transit (m)')
    bx.set_ylabel('C++ transit (s)')
    bx.set_xticks(x)
    bx.set_xticklabels([f'{f}\n{TRAJ.get(tj, tj)}' + ('\nfrozen' if fz else '') for f, tj, fz in
                        zip(df.flight, df.traj, df.frozen_line)], rotation=55, ha='right', fontsize=7.5)
    for k in df.index[df.frozen_line]:
        for a in (ax, bx):
            a.axvspan(k - 0.5, k + 0.5, color='#999999', alpha=0.12, lw=0)
    ax.set_xlim(-0.6, len(df) - 0.4)
    # legend
    h = [plt.Line2D([], [], marker='o', ls='', color='k', label='QUAD4 cable'),
         plt.Line2D([], [], marker='D', ls='', color='k', mfc='white', label='object (minus 0.0525 m)'),
         plt.Rectangle((0, 0), 1, 1, fc='white', ec='k', hatch='///', label='not captured (ended by our LAND)'),
         plt.Rectangle((0, 0), 1, 1, color='#999999', alpha=0.25, label='FROZEN-line settings')]
    h += [plt.Rectangle((0, 0), 1, 1, color=c, label=k) for k, c in OUTCOME_HEX.items()]
    bx.legend(handles=h, loc='upper center', bbox_to_anchor=(0.5, -0.42), ncol=4, fontsize=7.5)
    ax.set_title('13 Nov, his C++ planner into the ring: closest approach to a carrier cable and transit time',
                 fontsize=9.5)
    fig.tight_layout()
    for ext in ('png', 'pdf'):
        fig.savefig(os.path.join(OUT, f'summary_clearance_transit.{ext}'))
    plt.close(fig)


if __name__ == '__main__':
    df = pd.read_csv(os.path.join(HERE, 'metrics.csv'))
    for name in FROZEN:
        flight_figure(name, df.set_index('flight').loc[name])
    summary(df)

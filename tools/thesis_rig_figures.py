#!/usr/bin/env python3
"""tools/thesis_rig_figures.py -- thesis figures for the rig and the 9 Oct reconfiguration safety work.
Built with the thesis-figure skill: print size, Computer Modern, no run ids (they are in
configs/thesis_figures.yaml and docs/figures/PROVENANCE.md).

    python3 tools/thesis_rig_figures.py departed      # one figure
    python3 tools/thesis_rig_figures.py               # all with data

F_departed_landing -- "On the rig the freed UAV's own landing stalled above its resting height and
it hovered until the fleet's disarm dropped it; with the departed landing it is pushed to its own
resting height and disarmed alone as soon as it is there."
  hero: the freed UAV's height above its own resting height after LAND, rig (8 Oct) against the twin
  with the departed landing; reference height dashed; the drop at the fleet disarm and the disarm
  alone marked. Left out: the carriers, the ring, the tilt. Window: LAND to 2 s after the last disarm.
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

# the freed UAV: rig r303_4 (QUAD4, /drone_3, released by its magnet, landed with the fleet at LAND);
# twin R1304 (the same scenario: the freed UAV lands with the fleet at LAND, departed landing on)
DEPARTED = dict(rig=dict(path='results/rig/2026-10-08/r303_4_run', drone=3),
                twin=dict(run='R1304', drone=1))


def run_dir(rid):
    return glob.glob(os.path.join(REPO, 'results', '*', f'{rid}_*'))[0]


def load(spec):
    d = os.path.join(REPO, spec['path']) if 'path' in spec else run_dir(spec['run'])
    run = pd.read_csv(os.path.join(d, 'logs', 'run.csv')).drop_duplicates('t').reset_index(drop=True)
    ev = {}
    for e in csv.DictReader(open(os.path.join(d, 'logs', 'events.csv'))):
        ev.setdefault(e['event'], []).append(float(e['sim_time']))
    return run, ev


def departed_trace(spec):
    run, ev = load(spec)
    i = spec['drone']
    t = run.t.to_numpy()
    z = run[f'd{i}_z'].to_numpy()
    zr = run[f'd{i}_ref_z'].to_numpy() if f'd{i}_ref_z' in run else np.full_like(z, np.nan)
    armed = run[f'd{i}_armed'].to_numpy() > 0.5
    rest = float(np.nanmedian(z[t <= t[0] + 2.0]))
    t_land = ev['LAND'][0]
    edge = np.flatnonzero((t[1:] > t_land) & armed[:-1] & ~armed[1:])
    t_off = float(t[edge[0] + 1]) if len(edge) else None
    t_landed = ev.get('LANDED', [None])[0]
    if t_off is None and t_landed is not None and t[-1] >= t_landed - 0.1:
        t_off = t_landed            # the rig table ends at LANDED: the fleet disarm is commanded there
    return dict(t=t - t_land, h=z - rest, href=zr - rest, t_off=None if t_off is None else t_off - t_land,
                t_landed=None if t_landed is None else t_landed - t_land, rest=rest)


def fig_departed():
    ps.thesis()
    rig, twin = departed_trace(DEPARTED['rig']), departed_trace(DEPARTED['twin'])
    fig, axs = plt.subplots(1, 2, figsize=ps.size('full', height=2.2), sharey=True)
    stats = {}
    T0 = 3.0                                       # the descent above 0.8 m is context: start here
    spans = dict(rig=(T0, rig['t_off'] + 0.8), twin=(T0, twin['t_landed'] + 1.2))
    for ax, tr, key, colour, letter, label in (
            (axs[0], rig, 'rig', ps.drone_colour(3), 'a', 'Rig, 8 Oct: QUAD4, old landing'),
            (axs[1], twin, 'twin', ps.drone_colour(1), 'b', 'Twin: drone 2, departed landing')):
        t0, t1 = spans[key]
        w = (tr['t'] >= t0) & (tr['t'] <= t1)
        t, h, hr = tr['t'][w], tr['h'][w], tr['href'][w]
        ax.axhline(0.0, color='0.6', lw=0.5, zorder=0)
        ax.plot(t, hr, color='0.55', lw=0.7, ls='--', zorder=1, drawstyle='steps-post')
        ax.plot(t, h, color=colour, lw=1.3, zorder=2)
        k = int(np.argmin(np.abs(tr['t'] - tr['t_off'])))
        h_off = float(tr['h'][max(k - 1, 0)])
        stats[key + '_disarm_h'] = h_off
        ax.plot(tr['t_off'], h_off, 'x', color=colour, ms=5, mew=1.3, zorder=3)
        ax.set_xlim(t0, t1)
        ax.set_xlabel(r'$t$ after LAND (s)')
        ax.annotate(label, xy=(1.0, 1.0), xycoords='axes fraction', xytext=(-2, -2),
                    textcoords='offset points', fontsize=8, color='0.2', ha='right', va='top')
        ps.panel_label(ax, letter)
    r = rig
    after = (r['t'] > 0) & np.isfinite(r['href'])
    z_freeze = float(r['href'][after][-1])
    stats['rig_freeze_h'] = z_freeze
    hover = (r['t'] > r['t_off'] - 2.0) & (r['t'] < r['t_off'])
    stats['rig_hover_h'] = float(np.nanmean(r['h'][hover]))
    a = axs[0]
    a.annotate('frozen at touchdown', xy=(r['t_off'] - 0.05, z_freeze), xytext=(0, -1.5),
               textcoords='offset points', fontsize=8, color='0.35', ha='right', va='top')
    a.annotate(f"dropped {stats['rig_disarm_h']:.2f}\,m by\nthe fleet disarm", xy=(r['t_off'], stats['rig_disarm_h']),
               xytext=(-4, 8), textcoords='offset points', fontsize=8, color='0.35', ha='right', va='bottom')
    b = axs[1]
    tw = twin
    rest_i = np.flatnonzero((tw['t'] > 0) & (np.abs(tw['h']) <= 0.03))[0]
    stats['twin_rest_t'] = float(tw['t'][rest_i])
    stats['twin_alone_after_rest'] = tw['t_off'] - stats['twin_rest_t']
    b.annotate(f"disarmed alone {stats['twin_alone_after_rest']:.1f}\,s\nafter reaching its rest",
               xy=(tw['t_off'], 0.0), xytext=(10, 26), textcoords='offset points', fontsize=8, color='0.35',
               ha='left', va='bottom', arrowprops=dict(arrowstyle='-', color='0.5', lw=0.5))
    b.plot([tw['t_landed']] * 2, [0.0, 0.42], color='0.75', lw=0.5, zorder=0)
    b.annotate('ring down', xy=(tw['t_landed'], 0.42), xytext=(0, 2), textcoords='offset points',
               fontsize=8, color='0.35', ha='center', va='bottom')
    axs[0].set_ylabel('Height above its rest (m)')
    axs[0].set_ylim(-0.12, 0.85)
    axs[0].set_yticks([0, 0.2, 0.4, 0.6, 0.8])
    stem = os.path.join(OUT, 'F_departed_landing', 'departed_landing')
    ps.save_print(fig, stem)
    return stem, stats


FIGS = {'departed': fig_departed}

if __name__ == '__main__':
    names = sys.argv[1:] or list(FIGS)
    for n in names:
        stem, stats = FIGS[n]()
        print(stem, {k: round(v, 3) if isinstance(v, float) else v for k, v in stats.items()})

#!/usr/bin/env python3
"""
tools/compare_runs.py — overlay N runs on shared axes, with a metric table (§9.4)

The ablation and head-to-head workhorse.

    ./tools/compare_runs.py R0034 R0039
    ./tools/compare_runs.py R0034-R0038 R0039-R0043        # two repeat sets
    ./tools/compare_runs.py R0020 R0034 --label bench gazebo --align event
    ./tools/compare_runs.py R0034-R0038 R0039-R0043 --out results/compare/elev_45_v_65

WORKS ACROSS HARNESSES. A SIL bench run and a Gazebo run land on the same axes,
because both write the same wide schema (see metrics.load_run) -- that comparison is
the whole point of having a bench. Panels the Gazebo side cannot fill say so rather
than dropping the Gazebo run silently.

REPEATS ARE GROUPED, NOT AVERAGED INTO A LINE. Runs of the same scenario share a
colour and are drawn as individual thin traces, and the table reports MEDIAN AND FULL
RANGE (§9.2) -- because a mean over five runs hides the one that diverged, and the one
that diverged is usually the finding.

COLOUR MEANS "WHICH RUN" HERE, not "which drone" -- a separate palette from
plot_style.drone_colour for exactly that reason. Per-drone detail gets one panel per
drone instead.
"""
import argparse
import csv
import math
import os
import re
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, HERE)

import matplotlib.pyplot as plt        # noqa: E402
import metrics as M                    # noqa: E402
import plot_style as S                 # noqa: E402
from plot_run import Run               # noqa: E402
from run_dir import resolve            # noqa: E402


def expand(specs):
    """Expand 'R0034-R0038' into the five ids. Repeat sets are always contiguous ids
    (the runner allocates them in one batch), and typing five is how one gets missed."""
    out = []
    for s in specs:
        m = re.fullmatch(r'R(\d{4})-R?(\d{4})', s)
        if m:
            a, b = int(m.group(1)), int(m.group(2))
            out += [f'R{i:04d}' for i in range(a, b + 1)]
        else:
            out.append(s)
    return out


def group_key(r):
    scn = r.manifest.get('scenario_name') or r.manifest.get('scenario') or ''
    return os.path.splitext(os.path.basename(str(scn)))[0] or r.rid


class Group:
    """One scenario's runs: a colour, a label, and the metrics of each member."""

    def __init__(self, key, runs, colour, label=None):
        self.key = key
        self.runs = runs
        self.colour = colour
        self.label = label or key
        self.n = len(runs)

    def display(self):
        return self.label + (f'  (n={self.n})' if self.n > 1 else f'  ({self.runs[0].rid})')


def build_groups(runs, labels, group_by):
    keys = [group_key(r) if group_by else r.rid for r in runs]
    order = []
    for k in keys:
        if k not in order:
            order.append(k)
    if labels and len(labels) != len(order):
        raise SystemExit(f'--label needs {len(order)} labels for {len(order)} groups: '
                         + ', '.join(order))
    return [Group(k, [r for r, kk in zip(runs, keys) if kk == k], S.run_colour(i),
                  labels[i] if labels else None)
            for i, k in enumerate(order)]


def time_axis(r, align):
    """Run time, optionally shifted so the primary event sits at t=0.

    Two attach runs weld at different times; overlaying them on absolute time compares
    the transient of one against the cruise of the other."""
    t = r.t
    if align == 'event':
        t0 = r.metrics.get('primary_event_s')
        if t0 is not None and math.isfinite(t0):
            return t - t0, True
    return t, False


# ── the overlay figures ──────────────────────────────────────────────────────

def fig_paths(groups, out_dir, align):
    fig, (ax, axz) = plt.subplots(1, 2, figsize=(11, 4.8))
    shown_desired = False
    for g in groups:
        for j, r in enumerate(g.runs):
            p = r.xyz('payload_')
            t, _ = time_axis(r, align)
            ax.plot(p[:, 0], p[:, 1], color=g.colour, alpha=0.9 if g.n == 1 else 0.6,
                    lw=1.5 if g.n == 1 else 1.0,
                    label=g.display() if j == 0 else None)
            axz.plot(t, p[:, 2], color=g.colour, alpha=0.9 if g.n == 1 else 0.6,
                     lw=1.5 if g.n == 1 else 1.0,
                     label=g.display() if j == 0 else None)
            if not shown_desired and r.has('payload_ref_x'):
                pr = r.xyz('payload_ref_')
                ax.plot(pr[:, 0], pr[:, 1], color='#666666', label='desired',
                        **S.DESIRED)
                axz.plot(t, pr[:, 2], color='#666666', label='desired', **S.DESIRED)
                shown_desired = True
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_aspect('equal', adjustable='datalim')
    ax.set_title('Payload path', loc='left')
    S.busy_legend(ax)
    axz.set_xlabel(xlabel(align))
    axz.set_ylabel('payload z [m]')
    axz.set_title('Payload height', loc='left')
    S.busy_legend(axz)
    fig.tight_layout()
    return S.save(fig, out_dir, 'c1_paths')


def fig_error(groups, out_dir, align):
    fig, axes = plt.subplots(2, 1, figsize=(10, 6.5), sharex=True)
    for g in groups:
        for j, r in enumerate(g.runs):
            t, _ = time_axis(r, align)
            style = dict(color=g.colour, alpha=0.9 if g.n == 1 else 0.55,
                         lw=1.5 if g.n == 1 else 0.9)
            if r.has('payload_ref_x'):
                err = np.linalg.norm(r.xyz('payload_') - r.xyz('payload_ref_'), axis=1)
                axes[0].plot(t, err, label=g.display() if j == 0 else None, **style)
            worst = np.nanmax(M.stack(r.d, 'd{i}_track_err', r.n), axis=1)
            axes[1].plot(t, worst, label=g.display() if j == 0 else None, **style)
    axes[0].set_ylabel('payload error [m]')
    axes[0].set_title('Payload tracking error', loc='left')
    S.busy_legend(axes[0])
    axes[1].set_ylabel('worst drone error [m]')
    # The WORST drone, not a mean: the fleet is mechanically coupled, so one drone
    # losing its reference is the failure, and averaging it against three healthy ones
    # is how a divergence gets reported as a small degradation.
    axes[1].set_title('Worst per-drone tracking error across the fleet', loc='left')
    axes[1].set_xlabel(xlabel(align))
    for a in axes:
        if align == 'event':
            a.axvline(0.0, color='#4daf4a', ls=':', lw=1.0)
    fig.tight_layout()
    return S.save(fig, out_dir, 'c2_error')


def fig_tilt(groups, out_dir, align):
    fig, (ax, axt) = plt.subplots(2, 1, figsize=(10, 6.5), sharex=True)
    for g in groups:
        for j, r in enumerate(g.runs):
            t, _ = time_axis(r, align)
            style = dict(color=g.colour, alpha=0.9 if g.n == 1 else 0.55,
                         lw=1.5 if g.n == 1 else 0.9)
            ax.plot(t, r.col('payload_tilt_deg'),
                    label=g.display() if j == 0 else None, **style)
            axt.plot(t, np.nanmean(M.stack(r.d, 'd{i}_thr', r.n), axis=1),
                     label=g.display() if j == 0 else None, **style)
    ax.set_ylabel('payload tilt [deg]')
    ax.set_title('Payload tilt — the attach-failure signature', loc='left')
    S.busy_legend(ax)
    axt.axhspan(M.THROTTLE_SAT, 1.0, color='#e41a1c', alpha=0.07)
    axt.set_ylabel('fleet mean throttle [-]')
    axt.set_ylim(0, 1)
    axt.set_title('Fleet mean throttle', loc='left')
    axt.set_xlabel(xlabel(align))
    for a in (ax, axt):
        if align == 'event':
            a.axvline(0.0, color='#4daf4a', ls=':', lw=1.0)
    fig.tight_layout()
    return S.save(fig, out_dir, 'c3_tilt_effort')


def fig_per_drone(groups, out_dir, align):
    """One panel per drone id: the only honest way to show per-drone traces when colour
    is already spent on which run a trace belongs to."""
    n = max(r.n for g in groups for r in g.runs)
    fig, axes = plt.subplots(n, 1, figsize=(10, 2.1 * n), sharex=True, squeeze=False)
    for i in range(n):
        ax = axes[i, 0]
        for g in groups:
            for j, r in enumerate(g.runs):
                if i >= r.n:
                    continue
                t, _ = time_axis(r, align)
                ax.plot(t, r.col(f'd{i}_track_err'), color=g.colour,
                        alpha=0.9 if g.n == 1 else 0.55, lw=1.5 if g.n == 1 else 0.9,
                        label=g.display() if j == 0 else None)
        ax.set_ylabel(f'drone {i}\nerror [m]')
        # The drone id keeps its RViz colour as a swatch on the axis, so the reader can
        # still tie the panel to the aircraft they watched fly.
        ax.tick_params(axis='y', colors=S.drone_colour(i))
        ax.yaxis.label.set_color(S.drone_colour(i))
        if align == 'event':
            ax.axvline(0.0, color='#4daf4a', ls=':', lw=1.0)
        if i == 0:
            S.busy_legend(ax)
    axes[-1, 0].set_xlabel(xlabel(align))
    fig.suptitle('Per-drone tracking error', x=0.01, ha='left', fontsize=10)
    fig.tight_layout()
    return S.save(fig, out_dir, 'c4_per_drone')


def xlabel(align):
    return 'time since event [s]' if align == 'event' else 'sim time [s]'


# ── the metric table ─────────────────────────────────────────────────────────

def metric_rows(groups):
    """[(label, unit, [cell per group])], every number from tools/metrics.py.

    Grouped runs report median and full range (§9.2); a single run reports its value."""
    n_max = max(r.n for g in groups for r in g.runs)

    def per_drone(i, key):
        def f(m):
            pd = m.get('per_drone') or []
            return pd[i].get(key, math.nan) if i < len(pd) else math.nan
        return f

    specs = [('payload RMSE', 'm', lambda m: m.get('payload_rmse_m', math.nan)),
             ('payload radius ratio', '-', lambda m: m.get('payload_radius_ratio',
                                                           math.nan)),
             ('payload phase lag', 's', lambda m: m.get('payload_phase_lag_s',
                                                        math.nan)),
             ('payload tilt, peak', 'deg', lambda m: m.get('payload_tilt_peak_deg',
                                                           math.nan)),
             ('payload tilt, settled', 'deg', lambda m: m.get(
                 'payload_tilt_settled_deg', math.nan)),
             ('payload z, settled', 'm', lambda m: m.get('payload_z_settled_m',
                                                         math.nan)),
             ('tension spread', '-', lambda m: (m.get('tension_share') or {}).get(
                 'spread', math.nan)),
             ('event time', 's', lambda m: m.get('primary_event_s') or math.nan),
             ('duration', 's', lambda m: m.get('duration_s', math.nan))]
    for i in range(n_max):
        specs.append((f'drone {i} peak error', 'm', per_drone(i, 'peak_track_err_m')))
    for i in range(n_max):
        specs.append((f'drone {i} settled error', 'm',
                      per_drone(i, 'settled_track_err_m')))
    for i in range(n_max):
        specs.append((f'drone {i} peak |aCm|', 'm/s²', per_drone(i, 'peak_cable_accel')))

    rows = []
    for name, unit, fn in specs:
        cells, any_finite = [], False
        for g in groups:
            vals = [fn(r.metrics) for r in g.runs]
            s = M.summarise(vals)
            cells.append(s)
            any_finite = any_finite or s['n'] > 0
        if any_finite:
            rows.append((name, unit, cells))
    return rows


def fmt(cell, unit):
    if cell['n'] == 0:
        return '—'
    dp = 1 if unit in ('deg', 'm/s²') else 3
    if cell['n'] == 1 or cell['min'] == cell['max']:
        return f'{cell["median"]:.{dp}f}'
    return (f'{cell["median"]:.{dp}f}  [{cell["min"]:.{dp}f}–{cell["max"]:.{dp}f}]')


def print_table(groups, rows):
    heads = ['metric'] + [g.display() for g in groups]
    body = [[f'{name} [{unit}]'] + [fmt(c, unit) for c in cells]
            for name, unit, cells in rows]
    w = [max(len(h), *(len(r[k]) for r in body)) for k, h in enumerate(heads)]
    line = '  '.join(h.ljust(w[k]) for k, h in enumerate(heads))
    print('\n' + line)
    print('  '.join('-' * x for x in w))
    for r in body:
        print('  '.join(c.ljust(w[k]) for k, c in enumerate(r)))
    print('\n  grouped cells are median [min–max] over the repeats (§9.2).')


def write_table(groups, rows, out_dir):
    path = os.path.join(out_dir, 'metrics.csv')
    with open(path, 'w', newline='') as fh:
        w = csv.writer(fh)
        w.writerow(['metric', 'unit'] +
                   sum([[f'{g.label}_median', f'{g.label}_min', f'{g.label}_max',
                         f'{g.label}_n'] for g in groups], []))
        for name, unit, cells in rows:
            row = [name, unit]
            for c in cells:
                row += [c['median'], c['min'], c['max'], c['n']]
            w.writerow(row)
    return path


def fig_table(groups, rows, out_dir):
    fig, ax = plt.subplots(figsize=(2.6 + 2.4 * len(groups), 0.29 * len(rows) + 0.8))
    ax.axis('off')
    cells = [[f'{name} [{unit}]'] + [fmt(c, unit) for c in cells]
             for name, unit, cells in rows]
    w0 = 0.40 if len(groups) < 4 else 0.30
    table = ax.table(cellText=cells,
                     colLabels=['metric'] + [g.display() for g in groups],
                     colWidths=[w0] + [(1 - w0) / len(groups)] * len(groups),
                     cellLoc='left', loc='upper center')
    table.auto_set_font_size(False)
    table.set_fontsize(7.5)
    table.scale(1, 1.35)
    for k in range(len(groups) + 1):
        table[0, k].set_text_props(weight='bold')
    for j, g in enumerate(groups):
        for i in range(len(cells) + 1):
            table[i, j + 1].set_text_props(color=g.colour)
    ax.set_title('Metrics — median [min–max] over repeats, from tools/metrics.py',
                 loc='left', fontsize=9)
    fig.tight_layout()
    return S.save(fig, out_dir, 'c5_metrics')


# ── driver ───────────────────────────────────────────────────────────────────

def compare(specs, out_dir=None, labels=None, align='auto', group_by=True,
            quiet=False):
    S.use()
    runs = [Run(resolve(s)) for s in expand(specs)]
    if len(runs) < 2:
        raise SystemExit('compare_runs needs at least two runs')
    groups = build_groups(runs, labels, group_by)
    if align == 'auto':
        # Align on the event when every run has one; otherwise absolute time is the
        # only shared axis there is.
        align = ('event' if all(r.metrics.get('primary_event_s') is not None
                                for r in runs) else 'none')
    out_dir = out_dir or os.path.join(REPO, 'results', 'compare',
                                      '_vs_'.join(g.key for g in groups)[:120])
    os.makedirs(out_dir, exist_ok=True)

    written = []
    for fn in (fig_paths, fig_error, fig_tilt, fig_per_drone):
        try:
            written += fn(groups, out_dir, align)
        except Exception as e:                       # noqa: BLE001
            print(f'    !! {fn.__name__} failed: {type(e).__name__}: {e}')
            plt.close('all')
    rows = metric_rows(groups)
    written += fig_table(groups, rows, out_dir)
    written.append(write_table(groups, rows, out_dir))

    if not quiet:
        print(f'\n=== compare: {" vs ".join(g.display() for g in groups)} ===')
        for g in groups:
            srcs = sorted({r.source for r in g.runs})
            print(f'  {g.label}: {", ".join(r.rid for r in g.runs)}  [{"/".join(srcs)}]')
        if len({r.source for r in runs}) > 1:
            print('  !! CROSS-HARNESS comparison: bench and Gazebo runs on one axis. '
                  'The bench has no contact physics, aerodynamics or mocap noise — see '
                  'docs/design/sil_bench.md §7 before quoting agreement as validation.')
        if align == 'event':
            evs = sorted({r.metrics.get('primary_event') for r in runs})
            print(f'  aligned on: t=0 at {"/".join(str(e) for e in evs)}')
            if len(evs) > 1:
                # A run that never welded aligns on MAGNET instead, which is a
                # different instant in the mission -- and usually itself the result.
                print('     !! not the same event in every run — the runs that are '
                      'missing the later event never reached it')
        else:
            print('  aligned on: sim time')
        print_table(groups, rows)
        print(f'\n  -> {out_dir}')
    return out_dir, rows


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('runs', nargs='+', help='run ids, directories, or R0034-R0038')
    ap.add_argument('--out', help='output directory (default results/compare/<key>)')
    ap.add_argument('--label', nargs='+', help='one label per group, in order')
    ap.add_argument('--align', choices=['auto', 'event', 'none'], default='auto',
                    help='put the primary event at t=0 (default: auto)')
    ap.add_argument('--no-group', action='store_true',
                    help='treat every run as its own series instead of grouping '
                         'repeats of a scenario')
    ap.add_argument('--quiet', action='store_true')
    args = ap.parse_args()
    compare(args.runs, out_dir=args.out, labels=args.label, align=args.align,
            group_by=not args.no_group, quiet=args.quiet)
    return 0


if __name__ == '__main__':
    sys.exit(main())

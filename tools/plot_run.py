#!/usr/bin/env python3
"""
tools/plot_run.py — the auto-generated figures for one run (THESIS_PLAN §9.4)

    ./tools/plot_run.py R0034                 # by run id, from results/ or results_archive/
    ./tools/plot_run.py results/2026-08-05/R0034_sim_gz_attach_ring_45_gz
    ./tools/plot_run.py R0034 R0039 --quiet   # several

Called automatically by run_experiment.py and sil_bench.py on completion, so a finished
run is already readable and there is no command to remember. Writes into the run's own
`plots/`:

    01_xy_path        desired vs actual payload path, per-drone reference vs actual
    02_error          per-drone and payload error against time, events + steady window
    03_axes           x / y / z against time, desired vs actual
    04_health         throttle (with the saturation band), tilt, |aCm|, armed state
    05_formation      per-drone tension share and azimuth -- reconfiguration, seen
    06_stage_split    the four-stage radius/phase table (moving trajectories only)
    07_trajectory_3d  the whole flight in 3D, desired vs actual, with formation
                      snapshots -- true equal aspect, so cable elevation angles read
                      correctly off the figure

This file draws logged columns and numbers that came out of tools/metrics.py. It
derives no reported quantity of its own: a plot annotation is a number in the thesis,
and §9.2 says those come from one place.

Panels a harness cannot fill say so on the axes rather than coming out blank -- a
Gazebo run has no cable tension, elevation or |aCm|, and neither harness logs solver
status.
"""
import argparse
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, HERE)

import matplotlib.pyplot as plt        # noqa: E402
import metrics as M                    # noqa: E402
import plot_style as S                 # noqa: E402
from run_dir import resolve            # noqa: E402

AUTHORITY_MS2 = 13.0     # thrust acceleration left after gravity (CLAUDE.md §8)


class Run:
    """One loaded run -- wide table, events, metrics -- from either harness."""

    def __init__(self, path):
        self.path = os.path.abspath(path)
        self.run = M.load_run(self.path)
        if self.run['source'] not in ('sil', 'gazebo'):
            raise SystemExit(f'{path}: no logs/run.csv or logs/sil.csv to plot')
        self.d = self.run['data']
        self.n = M.n_drones(self.d)
        self.t = np.asarray(self.d['t'], float)
        self.events = self.run['events']
        self.manifest = self.run['manifest']
        self.metrics = M.summarise_run(self.path)
        self.source = self.run['source']
        self.rid = self.manifest.get('run_id') or os.path.basename(self.path)[:5]

    # ── conveniences ────────────────────────────────────────────────────
    def col(self, name):
        v = self.d.get(name)
        return np.asarray(v, float) if v is not None else np.full(self.t.shape, np.nan)

    def has(self, name):
        return M.has(self.d, name)

    def xyz(self, prefix):
        return np.column_stack([self.col(f'{prefix}{ax}') for ax in 'xyz'])

    @property
    def label(self):
        scn = (self.manifest.get('scenario_name')
               or os.path.basename(str(self.manifest.get('scenario', ''))) or '?')
        return f'{self.rid} · {self.source} · {scn}'

    def title(self, ax, text):
        ax.set_title(text, loc='left')

    def decorate(self, ax, steady=True, events=True):
        """Events and the steady window, identically on every time axis."""
        if events:
            S.mark_events(ax, self.events, t_max=float(self.t[-1]))
        win = self.metrics.get('steady_window_s') if steady else None
        if win:
            S.shade_steady(ax, win[0], win[1])

    def finish(self, fig, name, out_dir):
        S.stamp(fig, self.label, self.manifest.get('git_sha', '')[:8])
        return S.save(fig, out_dir, name)


# ── the figures ──────────────────────────────────────────────────────────

def fig_xy_path(r, out_dir):
    """1. Where everything went, in plan view."""
    fig, (ax, axd) = plt.subplots(1, 2, figsize=(10, 4.6))

    p = r.xyz('payload_')
    if r.has('payload_ref_x'):
        pr = r.xyz('payload_ref_')
        ax.plot(pr[:, 0], pr[:, 1], color=S.PAYLOAD_LINE, label='payload desired',
                **S.DESIRED)
    ax.plot(p[:, 0], p[:, 1], color=S.PAYLOAD_LINE, label='payload actual', **S.ACTUAL)
    ax.plot(p[0, 0], p[0, 1], 'o', color=S.PAYLOAD_LINE, ms=4, mfc='white')

    # Events marked ON the path, since a plan view has no time axis. Labels are stepped
    # because a hovering payload puts ARM, TAKEOFF and MAGNET at the same point.
    for j, (name, t_ev) in enumerate(sorted(r.events.items(), key=lambda kv: kv[1])):
        k = int(np.argmin(np.abs(r.t - t_ev)))
        if abs(r.t[k] - t_ev) > 1.0:
            continue
        ax.plot(p[k, 0], p[k, 1], marker='x', ms=7, mew=1.6,
                color=S.event_colour(name), linestyle='none')
        ax.annotate(f'{name} {t_ev:.1f}s', (p[k, 0], p[k, 1]),
                    textcoords='offset points', xytext=(6, 4 + 9 * (j % 4)),
                    fontsize=6.5, color=S.event_colour(name))

    for i in range(r.n):
        c = S.drone_colour(i)
        q = r.xyz(f'd{i}_')
        axd.plot(q[:, 0], q[:, 1], color=c, label=f'drone {i}', **S.ACTUAL)
        if r.has(f'd{i}_ref_x'):
            ref = r.xyz(f'd{i}_ref_')
            axd.plot(ref[:, 0], ref[:, 1], color=c, **S.DESIRED)
        axd.plot(q[0, 0], q[0, 1], 'o', color=c, ms=4, mfc='white')
    axd.plot(p[:, 0], p[:, 1], color=S.PAYLOAD_LINE, alpha=0.5, lw=1.0,
             label='payload')

    rmse = r.metrics.get('payload_rmse_m')
    rr = r.metrics.get('payload_radius_ratio')
    sub = [] if rmse is None else [f'RMSE {rmse:.3f} m (steady window)']
    if rr is not None and math.isfinite(rr):
        sub.append(f'radius ratio {rr:.3f}')
    r.title(ax, 'Payload path' + (f'  —  {", ".join(sub)}' if sub else ''))
    r.title(axd, 'Drones: actual (solid) vs reference (dashed)')
    for a in (ax, axd):
        a.set_xlabel('x [m]')
        a.set_ylabel('y [m]')
        a.set_aspect('equal', adjustable='datalim')
        a.legend(loc='best', ncol=2)
    fig.tight_layout()
    return r.finish(fig, '01_xy_path', out_dir)


def fig_error(r, out_dir):
    """2. How far off, against time -- what most questions reduce to."""
    fig, (ax, axp) = plt.subplots(2, 1, figsize=(9.5, 6), sharex=True,
                                  gridspec_kw={'height_ratios': [2, 1]})
    for i in range(r.n):
        e = r.col(f'd{i}_track_err')
        if not np.any(np.isfinite(e)):
            continue
        peak = r.metrics['per_drone'][i].get('peak_track_err_m', float('nan'))
        ax.plot(r.t, e, color=S.drone_colour(i),
                label=f'drone {i}  (peak {peak:.2f} m)', **S.ACTUAL)
    r.decorate(ax)
    r.title(ax, 'Per-drone tracking error ‖p − p_ref‖')
    ax.set_ylabel('error [m]')
    S.busy_legend(ax, ncol=2)

    if r.has('payload_ref_x'):
        err = np.linalg.norm(r.xyz('payload_') - r.xyz('payload_ref_'), axis=1)
        axp.plot(r.t, err, color=S.PAYLOAD_LINE, label='payload', **S.ACTUAL)
        rmse = r.metrics.get('payload_rmse_m')
        if rmse is not None and math.isfinite(rmse):
            axp.axhline(rmse, color=S.PAYLOAD_LINE, lw=0.9, ls='--', alpha=0.7,
                        label=f'RMSE {rmse:.3f} m')
        axp.legend(loc='upper left')
        axp.set_ylabel('error [m]')
    else:
        S.note(axp, 'no payload reference logged in this run')
    r.decorate(axp)
    r.title(axp, 'Payload error')
    axp.set_xlabel('sim time [s]')
    fig.tight_layout()
    return r.finish(fig, '02_error', out_dir)


def fig_axes(r, out_dir):
    """3. Per-axis desired vs actual: a z collapse and an xy phase lag look identical
    in a norm."""
    fig, axes = plt.subplots(3, 2, figsize=(11, 7), sharex=True)
    p, pr = r.xyz('payload_'), r.xyz('payload_ref_')
    for k, ax_name in enumerate('xyz'):
        a = axes[k, 0]
        if r.has('payload_ref_x'):
            a.plot(r.t, pr[:, k], color=S.PAYLOAD_LINE, label='desired', **S.DESIRED)
        a.plot(r.t, p[:, k], color=S.PAYLOAD_LINE, label='actual', **S.ACTUAL)
        r.decorate(a, events=(k == 0))
        a.set_ylabel(f'payload {ax_name} [m]')
        if k == 0:
            a.legend(loc='upper left', ncol=2)
            r.title(a, 'Payload')

        b = axes[k, 1]
        for i in range(r.n):
            c = S.drone_colour(i)
            b.plot(r.t, r.col(f'd{i}_{ax_name}'), color=c, label=f'd{i}', **S.ACTUAL)
            if r.has(f'd{i}_ref_{ax_name}'):
                b.plot(r.t, r.col(f'd{i}_ref_{ax_name}'), color=c, **S.DESIRED)
        r.decorate(b, events=(k == 0))
        b.set_ylabel(f'drone {ax_name} [m]')
        if k == 0:
            b.legend(loc='upper left', ncol=r.n)
            r.title(b, 'Drones — actual (solid) vs reference (dashed)')
    axes[2, 0].set_xlabel('sim time [s]')
    axes[2, 1].set_xlabel('sim time [s]')
    fig.tight_layout()
    return r.finish(fig, '03_axes', out_dir)


def fig_health(r, out_dir):
    """4. Is the aircraft physically able to do what it is being told."""
    has_acm = any(r.has(f'd{i}_acm') for i in range(r.n))
    # A panel that can only say "not observable here" gets a sliver, not a quarter of
    # the figure.
    fig, axes = plt.subplots(4, 1, figsize=(9.5, 8.5 if has_acm else 6.8), sharex=True,
                             gridspec_kw={'height_ratios': [2, 2, 2 if has_acm else 0.5,
                                                            1]})

    ax = axes[0]
    ax.axhspan(M.THROTTLE_SAT, 1.0, color='#e41a1c', alpha=0.07)
    ax.annotate('saturation', xy=(0.005, M.THROTTLE_SAT), xycoords=('axes fraction',
                'data'), fontsize=6.5, color='#e41a1c', va='bottom')
    eff = r.metrics.get('control_effort', {})
    for i in range(r.n):
        lbl = f'drone {i}'
        if eff.get('mean'):
            lbl += f'  (settled mean {eff["mean"][i]:.3f})'
        ax.plot(r.t, r.col(f'd{i}_thr'), color=S.drone_colour(i), label=lbl,
                **S.ACTUAL)
    ax.set_ylabel('throttle [-]')
    ax.set_ylim(0, 1)
    S.busy_legend(ax, ncol=2)
    r.title(ax, 'Throttle')
    r.decorate(ax)

    ax = axes[1]
    ax.plot(r.t, r.col('payload_tilt_deg'), color=S.PAYLOAD_LINE, label='payload',
            **S.ACTUAL)
    for i in range(r.n):
        ax.plot(r.t, r.col(f'd{i}_tilt_deg'), color=S.drone_colour(i), alpha=0.7,
                lw=1.0, label=f'drone {i}')
    peak = r.metrics.get('payload_tilt_peak_deg')
    settled = r.metrics.get('payload_tilt_settled_deg')
    r.title(ax, f'Tilt — payload peak {peak:.1f}°, settled {settled:.1f}°'
            if peak is not None else 'Tilt')
    ax.set_ylabel('tilt [deg]')
    S.busy_legend(ax, ncol=3)
    r.decorate(ax)

    ax = axes[2]
    if has_acm:
        for i in range(r.n):
            ax.plot(r.t, r.col(f'd{i}_acm'), color=S.drone_colour(i),
                    label=f'drone {i}', **S.ACTUAL)
        ax.axhline(AUTHORITY_MS2, color='#e41a1c', lw=0.9, ls='--',
                   label=f'{AUTHORITY_MS2:.0f} m/s² thrust authority')
        ax.set_ylabel('|aCm| [m/s²]')
        S.busy_legend(ax, ncol=2)
        r.decorate(ax)
    else:
        # Not a gap in the run: |aCm| is internal to the tracker, and the Gazebo runner
        # watches from outside over ROS topics.
        S.note(ax, 'measured cable acceleration |aCm| is not observable in a Gazebo\n'
                   'run — the column exists for schema parity and is NaN')
    r.title(ax, 'Measured cable acceleration')

    ax = axes[3]
    for i in range(r.n):
        ax.step(r.t, r.col(f'd{i}_armed') + 0.02 * i, where='post',
                color=S.drone_colour(i), lw=1.4, label=f'drone {i}')
    ax.set_ylabel('armed')
    ax.set_yticks([0, 1])
    ax.set_ylim(-0.15, 1.25)
    ax.set_xlabel('sim time [s]')
    ax.legend(loc='center left', ncol=r.n)
    # §9.4 asks for solver status here; neither harness logs it. Say so rather than
    # leave a reader wondering whether the solver was healthy.
    r.title(ax, 'Armed state (acados solver status is not logged by either harness)')
    r.decorate(ax)

    fig.tight_layout()
    return r.finish(fig, '04_health', out_dir)


def branch_cut(angles, bins=72):
    """Centre of the emptiest 5 deg bin -- where a 0/360 wrap can go unnoticed."""
    a = np.asarray(angles, float)
    a = a[np.isfinite(a)]
    if a.size == 0:
        return 0.0
    counts, edges = np.histogram(a, bins=bins, range=(0.0, 360.0))
    k = int(np.argmin(counts))
    return float(0.5 * (edges[k] + edges[k + 1]))


def fig_formation(r, out_dir):
    """5. Who is carrying, and from where -- reconfiguration, seen."""
    fig, (ax, axa) = plt.subplots(2, 1, figsize=(9.5, 6.5), sharex=True)

    if 'tension_share' in r.metrics:
        tens = M.stack(r.d, 'd{i}_tension', r.n)
        total = np.sum(tens, axis=1)
        with np.errstate(invalid='ignore', divide='ignore'):
            frac = tens / np.where(total > 1e-9, total, np.nan)[:, None]
        share = r.metrics['tension_share']
        for i in range(r.n):
            ax.plot(r.t, 100 * frac[:, i], color=S.drone_colour(i),
                    label=f'drone {i}  (settled {100 * share["fraction"][i]:.0f}%)',
                    **S.ACTUAL)
        ax.axhline(100.0 / r.n, color='#666666', lw=0.8, ls='--',
                   label=f'even share ({100.0 / r.n:.0f}%)')
        ax.set_ylabel('tension share [%]')
        ax.set_ylim(0, 100)
        S.busy_legend(ax, ncol=2)
        r.title(ax, f'Cable tension share — spread {100 * share["spread"]:.0f} '
                    f'points, total {share["mean_total"]:.1f} N')
        r.decorate(ax)
    else:
        S.note(ax, 'cable tension is internal to the tracker and is not observable\n'
                   'in a Gazebo run — this panel is a SIL-bench figure')
        r.title(ax, 'Cable tension share')

    payload_xy = r.xyz('payload_')[:, :2]
    az = [M.azimuth_deg(r.xyz(f'd{i}_')[:, :2], payload_xy) for i in range(r.n)]
    # Put the 0/360 branch cut where no drone is sitting. Drone 0 typically holds
    # azimuth 0, and with the cut at 0 its trace flickers between the top and bottom of
    # the axis for the whole run -- a slot that never moves reads as one thrashing.
    cut = branch_cut(np.concatenate(az))
    for i in range(r.n):
        a = (az[i] - cut) % 360.0 + cut
        # Wrapped, not unwrapped: a circle sweep unwraps to +-1000 deg and loses the
        # formation geometry. The remaining jumps are broken so no vertical streak is
        # mistaken for a drone crossing the formation.
        a = np.where(np.abs(np.diff(a, prepend=a[0])) > 180, np.nan, a)
        axa.plot(r.t, a, color=S.drone_colour(i), label=f'drone {i}', **S.ACTUAL)
    axa.set_ylabel('azimuth from payload [deg]')
    axa.set_xlabel('sim time [s]')
    axa.set_yticks(np.arange(0, 361, 60) + round(cut / 60) * 60)
    axa.set_ylim(cut, cut + 360)
    S.busy_legend(axa, ncol=r.n, loc='lower left')
    r.title(axa, f'Formation azimuth (even {r.n}-gon = {360 // max(r.n, 1)}° apart)')
    r.decorate(axa)

    fig.tight_layout()
    return r.finish(fig, '05_formation', out_dir)


def fig_stage_split(r, out_dir):
    """6. The DISSIPATIVE_TRACKING_ISSUE §2 diagnostic: which stage of desired ->
    reference -> drones -> payload loses the trajectory."""
    fig, (ax, axt) = plt.subplots(1, 2, figsize=(11, 4.4),
                                  gridspec_kw={'width_ratios': [1, 1.25]})
    rows = r.metrics.get('stage_split') or []
    # Stage 1 is measured against the COMMANDED path, so its radius ratio is nan exactly
    # when that path has no radius -- a hover. The later stages still produce numbers
    # there (the drones move regardless), and reporting those would be meaningless.
    moving = bool(rows) and math.isfinite(rows[0].get('radius_ratio', float('nan')))
    if not moving:
        why = ('this run has no payload reference' if not rows else
               'the commanded path has no radius (a hover or a straight line) — the '
               'stage split is defined for moving trajectories only')
        for a in (ax, axt):
            S.note(a, why)
        r.title(ax, 'Stage split — not applicable')
        fig.tight_layout()
        return r.finish(fig, '06_stage_split', out_dir)

    # Drawn over the SWEEP WINDOW only, matching the numbers in the table. On a 75 s
    # run the sweep is ~8 s of it, and the hover either side would dominate the plan
    # view while contributing nothing to either column.
    win = r.metrics.get('sweep_window_s')
    m = M.sweep_window(r.t, r.xyz('payload_ref_')[:, :2])
    stages = [('desired', r.xyz('payload_ref_')[m, :2], S.PAYLOAD_LINE, S.DESIRED),
              ('reference centroid', M.centroid(r.d, r.n, 'ref_')[m, :2],
               '#377eb8', S.REFERENCE),
              ('actual centroid', M.centroid(r.d, r.n)[m, :2], '#4daf4a', S.ACTUAL),
              ('payload', r.xyz('payload_')[m, :2], S.PAYLOAD_LINE, S.ACTUAL)]
    for name, xy, colour, style in stages:
        ax.plot(xy[:, 0], xy[:, 1], color=colour, label=name, **style)
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_aspect('equal', adjustable='datalim')
    ax.legend(loc='best')
    r.title(ax, 'The four stages, in plan view'
            + (f'  (sweep {win[0]:.1f}–{win[1]:.1f} s)' if win else ''))

    axt.axis('off')
    cells = [[s['stage'],
              f'{s["radius_ratio"]:.3f}' if math.isfinite(s['radius_ratio']) else '—',
              f'{s["phase_lag_s"]:+.3f}' if math.isfinite(s['phase_lag_s']) else '—']
             for s in rows]
    table = axt.table(cellText=cells,
                      colLabels=['stage', 'radius ratio', 'phase lag [s]'],
                      colWidths=[0.56, 0.22, 0.22],
                      cellLoc='left', loc='upper center')
    table.auto_set_font_size(False)
    table.set_fontsize(8)
    table.scale(1, 1.6)
    for k in range(3):
        table[0, k].set_text_props(weight='bold')
    axt.set_title('Radius ratio and phase lag, measured stage by stage\n'
                  '(each against the stage before it; the last row is cumulative)',
                  loc='left', fontsize=9)
    fig.tight_layout()
    return r.finish(fig, '06_stage_split', out_dir)


def equal_aspect_3d(ax, pts, pad=0.05):
    """A true cube around the data. Not cosmetic: this project reads CABLE ELEVATION
    ANGLES off its geometry, and axes with independent scales quietly change every
    angle in the picture."""
    p = np.asarray(pts, float)
    p = p[np.all(np.isfinite(p), axis=1)]
    if p.size == 0:
        return
    lo, hi = p.min(axis=0), p.max(axis=0)
    c = 0.5 * (lo + hi)
    r = max(0.5 * float(np.max(hi - lo)), 0.15) * (1.0 + pad)
    ax.set_xlim(c[0] - r, c[0] + r)
    ax.set_ylim(c[1] - r, c[1] + r)
    ax.set_zlim(c[2] - r, c[2] + r)
    ax.set_box_aspect((1, 1, 1))


def snapshot_times(r, n=6):
    """Instants to draw the formation at: every event -- the weld and the abort are the
    reason to look at all -- plus an even spread over the airborne part."""
    t0, t1 = float(r.t[0]), float(r.t[-1])
    z = r.col('payload_z')
    airborne = np.isfinite(z) & (z > np.nanmin(z) + 0.05)
    if np.any(airborne):
        t0, t1 = float(r.t[airborne][0]), float(r.t[airborne][-1])
    times = [t for t in r.events.values() if t0 <= t <= t1]
    times += list(np.linspace(t0, t1, n))
    return sorted(set(round(t, 2) for t in times))


def fig_trajectory_3d(r, out_dir):
    """7. The flight in 3D. Figures 1 and 3 each hide an axis; this shows the shape of
    the manoeuvre, and whether the drones stayed above and around the load."""
    fig = plt.figure(figsize=(9.5, 8))
    ax = fig.add_subplot(111, projection='3d')

    payload = r.xyz('payload_')
    pts = [payload]

    # Formation snapshots first, so the trajectories draw over them.
    for j, t_snap in enumerate(snapshot_times(r)):
        k = int(np.argmin(np.abs(r.t - t_snap)))
        for i in range(r.n):
            q = r.xyz(f'd{i}_')[k]
            if not np.all(np.isfinite(q)) or not np.all(np.isfinite(payload[k])):
                continue
            # A spoke from each drone to the load. NOT drawn as a cable: whether a
            # given drone is tethered at a given instant is not reliably logged
            # (dN_attached means different things in the two harnesses), so this is
            # the formation's shape, not a claim about what is attached to what.
            ax.plot(*zip(q, payload[k]), color='#999999', lw=0.6, alpha=0.45,
                    zorder=1, label='formation snapshot' if j == 0 and i == 0 else None)

    if r.has('payload_ref_x'):
        pr = r.xyz('payload_ref_')
        ax.plot(pr[:, 0], pr[:, 1], pr[:, 2], color=S.PAYLOAD_LINE,
                label='payload desired', zorder=3, **S.DESIRED)
        pts.append(pr)
    ax.plot(payload[:, 0], payload[:, 1], payload[:, 2], color=S.PAYLOAD_LINE,
            label='payload actual', zorder=4, **S.ACTUAL)

    for i in range(r.n):
        c = S.drone_colour(i)
        q = r.xyz(f'd{i}_')
        ax.plot(q[:, 0], q[:, 1], q[:, 2], color=c, label=f'drone {i}', zorder=3,
                **S.ACTUAL)
        pts.append(q)
        if r.has(f'd{i}_ref_x'):
            ref = r.xyz(f'd{i}_ref_')
            ax.plot(ref[:, 0], ref[:, 1], ref[:, 2], color=c, zorder=2, **S.DESIRED)
            pts.append(ref)
        # Without start/end markers a 3D path does not say which way it was travelled.
        good = np.where(np.all(np.isfinite(q), axis=1))[0]
        if good.size:
            ax.plot(*q[good[0]], marker='o', ms=5, mfc='white', color=c, zorder=5)
            ax.plot(*q[good[-1]], marker='X', ms=6, color=c, zorder=5)

    equal_aspect_3d(ax, np.vstack(pts))
    # Label offset in DATA units, stepped per event: 3D text takes no offset in points,
    # and a payload that hovers puts ARM, TAKEOFF and MAGNET at the same point.
    dz = 0.045 * float(np.diff(ax.get_zlim()))
    for j, (name, t_ev) in enumerate(sorted(r.events.items(), key=lambda kv: kv[1])):
        k = int(np.argmin(np.abs(r.t - t_ev)))
        if abs(r.t[k] - t_ev) > 1.0 or not np.all(np.isfinite(payload[k])):
            continue
        ax.plot(*payload[k], marker='x', ms=8, mew=1.8, color=S.event_colour(name),
                linestyle='none', zorder=6)
        x, y, z = payload[k]
        ax.text(x, y, z + dz * (j % 4 - 1.5), f'  {name} {t_ev:.1f}s', fontsize=6.5,
                color=S.event_colour(name), zorder=6)

    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_zlabel('z [m]')
    ax.view_init(elev=24, azim=-58)
    ax.legend(loc='upper left', fontsize=8, framealpha=0.85)
    ax.set_title('Trajectory in 3D — actual (solid) vs desired (dashed);  '
                 '○ start, ✕ end.  Equal aspect: angles are true.', loc='left')
    fig.tight_layout()
    return r.finish(fig, '07_trajectory_3d', out_dir)


FIGURES = (fig_xy_path, fig_error, fig_axes, fig_health, fig_formation,
           fig_stage_split, fig_trajectory_3d)


def plot_run(path, out_dir=None, quiet=False, write_metrics=False):
    """Draw every figure for one run; returns the files written.

    One bad figure never stops the rest: this runs at the end of an experiment, and a
    plotting bug must not destroy a run that has already flown."""
    S.use()
    r = Run(path)
    out_dir = out_dir or os.path.join(r.path, 'plots')
    written = []
    for fn in FIGURES:
        try:
            written += fn(r, out_dir)
        except Exception as e:                       # noqa: BLE001
            print(f'    !! {fn.__name__} failed: {type(e).__name__}: {e}')
            plt.close('all')
    if write_metrics:
        with open(os.path.join(r.path, 'metrics.json'), 'w') as fh:
            json.dump(r.metrics, fh, indent=2, sort_keys=True, default=str)
    if not quiet:
        print(f'    {r.rid}: {len(written) // 2} figures -> {out_dir}')
    return written


def finish_run(run_dir, quiet=False):
    """Metrics + figures for a run that has just finished, returning the metrics dict
    for the manifest. Both harnesses call this on completion (§9.1).

    NEVER RAISES: it runs after the aircraft has flown, and an analysis bug must not
    turn a completed run into a failed one."""
    # SystemExit is caught alongside Exception on purpose: `Run` raises it for a
    # log-less run -- right for the CLI, fatal here, and not an Exception subclass.
    try:
        with open(os.path.join(run_dir, 'metrics.json'), 'w') as fh:
            mtr = M.summarise_run(run_dir)
            json.dump(mtr, fh, indent=2, sort_keys=True, default=str)
    except (Exception, SystemExit) as e:             # noqa: BLE001
        mtr = {'error': f'metrics failed: {type(e).__name__}: {e}'}
        print(f'    !! {mtr["error"]}')
    try:
        plot_run(run_dir, quiet=quiet)
    except (Exception, SystemExit) as e:             # noqa: BLE001
        print(f'    !! auto-plot failed: {type(e).__name__}: {e}')
    return mtr


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('run', nargs='+', help='run id (R0034) or run directory')
    ap.add_argument('--out', help='write here instead of the run\'s own plots/')
    ap.add_argument('--metrics', action='store_true',
                    help='also rewrite metrics.json from tools/metrics.py')
    ap.add_argument('--quiet', action='store_true')
    args = ap.parse_args()
    for spec in args.run:
        plot_run(resolve(spec), out_dir=args.out, quiet=args.quiet,
                 write_metrics=args.metrics)
    return 0


if __name__ == '__main__':
    sys.exit(main())

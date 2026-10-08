#!/usr/bin/env python3
"""tools/rig_summary.py -- one image per run that says whether it worked.

    python3 tools/rig_summary.py r303                              # today's rig run (or the latest day that has it)
    python3 tools/rig_summary.py results/rig/2026-10-07/r206e60    # any rig run
    python3 tools/rig_summary.py R1165                             # a twin run

(a) the ring's top view, reference against actual; (b) its height against the reference; (c) its
tilt; (d) its 3-D error from the reference, split into along the path, off the path and height
(|e|^2 = along^2 + off^2 + height^2; in a hover all of the sideways error is "off the path").
The RMSEs and peaks are over the scored part, shaded on the time axes: from the ring reaching its
target height to LAND (or a drop / abort). The lag in seconds is lag_metrics.py's A0, the ring's lag
on one clock over the constant-speed part of a circle or figure-8 (the ruler of the 8 Oct lag study);
the along-the-path error is against the published reference, which the planner advances 0.1 s, taken
linear between its 10 Hz updates (held, it reads as a sawtooth).
A rig run is converted with rig_to_run.py first (once; the run directory is <name>_run beside its
logs). Writes the day's figures/<name>_summary.png (a twin run: plots/00_summary.png).
"""
import argparse
import glob
import json
import math
import os
import sys
import time

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

TOOLS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TOOLS)
sys.path.insert(0, TOOLS)
import metrics  # noqa: E402
import plot_style as ps  # noqa: E402
import rig_to_run  # noqa: E402
import run_logs  # noqa: E402

END_EVENTS = ('LAND', 'DROP', 'FLEET_ABORT', 'GROUNDED')
SHOWN = ('TAKEOFF', 'HANDOVER', 'RELEASE', 'DETACH', 'DETECTED', 'DROP', 'LAND', 'LANDED',
         'FLEET_ABORT', 'GROUNDED')
EVENT_HEX = {'RELEASE': '#377eb8', 'DETACH': '#ff7f00', 'DETECTED': '#ff7f00', 'DROP': '#e41a1c',
             'FLEET_ABORT': '#e41a1c', 'GROUNDED': '#e41a1c'}
PART_HEX = {'along': '#e41a1c', 'off': '#377eb8', 'height': '#4daf4a'}
PATHS = {'orbit': 'circle', 'circle': 'circle', 'fig_8': 'figure-8', 'hover': 'hover'}
MOVING_MPS = 0.02           # reference speed above which the error has an along-the-path part


def resolve(arg):
    """(run dir with logs/run.csv, node-logs dir, output png, label, git sha) for a rig name/path or a twin run."""
    if os.path.isfile(os.path.join(arg, 'logs', 'run.csv')) and not os.path.isdir(arg + '_logs'):
        run = os.path.abspath(arg)
        man = json.load(open(os.path.join(run, 'manifest.json'))) if os.path.exists(
            os.path.join(run, 'manifest.json')) else {}
        return run, run, os.path.join(run, 'plots', '00_summary.png'), os.path.basename(run), \
            str(man.get('git_sha', ''))[:8]
    if arg.startswith('R') and arg[1:].isdigit():
        hits = sorted(glob.glob(os.path.join(REPO, 'results', '*', f'{arg}_*')))
        if not hits:
            raise SystemExit(f'rig_summary: no twin run {arg}')
        return resolve(hits[0])
    base = arg[:-4] if arg.endswith('.log') else arg.rstrip('/')
    if not os.path.exists(base + '.log'):
        days = sorted(glob.glob(os.path.join(REPO, 'results', 'rig', '*', f'{os.path.basename(base)}.log')))
        today = os.path.join(REPO, 'results', 'rig', time.strftime('%Y-%m-%d'), f'{os.path.basename(base)}.log')
        hit = today if os.path.exists(today) else (days[-1] if days else None)
        if hit is None:
            raise SystemExit(f'rig_summary: no rig run {arg} (no {os.path.basename(base)}.log under results/rig/)')
        base = hit[:-4]
    base = os.path.abspath(base)
    logs, run = base + '_logs', base + '_run'
    trackers = sorted(glob.glob(os.path.join(logs, 'logs', 'tracker', 'drone*')))
    if not trackers:
        raise SystemExit(f'rig_summary: no tracker logs under {logs}/logs/tracker')
    csv = os.path.join(run, 'logs', 'run.csv')
    if not os.path.exists(csv) or os.path.getmtime(csv) < os.path.getmtime(base + '.log'):
        rig_to_run.convert(trackers, run, bag=base if os.path.isdir(base) else None, t2_log=base + '.log')
    sha = ''
    if os.path.exists(base + '_code.txt'):
        sha = open(base + '_code.txt').readline().strip()[:8]
    out = os.path.join(os.path.dirname(base), 'figures', f'{os.path.basename(base)}_summary.png')
    return run, logs, out, os.path.basename(base), sha


def node_params(logs, kind):
    csvs = run_logs.planner_csvs(logs) if kind == 'planner' else run_logs.node_csvs(logs, kind)
    for c in csvs:
        p = os.path.join(os.path.dirname(c), 'params.json')
        if os.path.exists(p):
            return json.load(open(p)), os.path.basename(os.path.dirname(os.path.dirname(c)))
    return {}, ''


def one_clock_lag(logs, kind, run_t, run_z, t_cut=None):
    """lag_metrics.py's A0 (s) for a circle or figure-8, else None. With t_cut (run.csv time of the
    first release / detach / drop) only before it: the planner holds the path at a detach, which A0
    would count as lag (1 s in R1208). The node clock is aligned to run.csv on the ring's lift-off."""
    if kind not in ('orbit', 'circle', 'fig_8'):
        return None
    try:
        import lag_metrics
        pl = lag_metrics.planner_lags(logs)
        if not pl.get('window'):
            return None
        if t_cut is not None:
            L = pd.read_csv(sorted(run_logs.planner_csvs(logs), key=lambda p: -os.path.getsize(p))[0])
            lift = lambda t, z: float(np.asarray(t)[np.asarray(z) > np.median(np.asarray(z)[:20]) + 0.03][0])  # noqa: E731
            end = min(pl['window'][1], t_cut + lift(L.sim_time, L.load_z) - lift(run_t, run_z))
            if end - pl['window'][0] < 15.0:
                return None         # the first seconds of a circle read -0.2..0 s (R1208): not the steady lag
            pl = dict(pl, window=(pl['window'][0], end))
        return lag_metrics.absolute_lag(logs, pl)
    except Exception:          # noqa: BLE001 -- a run without planner logs simply has no lag figure
        return None


def continuous_ref(t, ref, max_gap_s=0.3):
    """The reference between its updates: it arrives at the planner rate (10 Hz) and is held on the
    50 Hz grid, a staircase that reads as a 1 cm sawtooth in the error on a 0.125 m/s circle. Linear
    between consecutive updates; held across a gap longer than max_gap_s (a hover, a pause)."""
    ch = np.concatenate([[0], np.where(np.any(np.abs(np.diff(ref, axis=0)) > 1e-9, axis=1))[0] + 1])
    k = np.searchsorted(ch, np.arange(len(t)), side='right') - 1
    a = ch[k]
    b = np.where(k + 1 < len(ch), ch[np.minimum(k + 1, len(ch) - 1)], a)
    span = t[b] - t[a]
    go = (b > a) & (span <= max_gap_s)
    frac = np.where(go, (t - t[a]) / np.where(go, span, 1.0), 0.0)
    return ref[a] + frac[:, None] * (ref[b] - ref[a])


def error_parts(t, p, ref):
    """3-D error and its along-the-path, off-the-path and height parts (metres, per sample)."""
    e = p - ref
    dt = float(np.nanmedian(np.diff(t))) if len(t) > 1 else 0.02
    k = max(1, int(round(0.5 / dt)))
    # the reference arrives at the planner rate and is held on the 50 Hz grid: smooth before differentiating
    ref_s = pd.DataFrame(ref[:, :2]).rolling(k, center=True, min_periods=1).mean().to_numpy()
    v = np.gradient(ref_s, t, axis=0)
    speed = np.linalg.norm(v, axis=1)
    moving = speed > MOVING_MPS
    tang = np.where(moving[:, None], v / np.where(moving, speed, 1.0)[:, None], 0.0)
    along = np.where(moving, np.sum(e[:, :2] * tang, axis=1), 0.0)
    off = np.sqrt(np.clip(np.sum(e[:, :2] ** 2, axis=1) - along ** 2, 0.0, None))
    return np.linalg.norm(e, axis=1), along, off, e[:, 2], moving


def rms(x, m):
    x = np.asarray(x, float)[m]
    x = x[np.isfinite(x)]
    return float(np.sqrt(np.mean(x ** 2))) if x.size else math.nan


def scored_window(t, ref_z, events, target_z):
    t_off = next((tt for tt, e, _ in events if e == 'TAKEOFF'), float(t[0]))
    t_end = min([tt for tt, e, _ in events if e in END_EVENTS and tt > t_off], default=float(t[-1]))
    z_tgt = target_z if target_z else float(np.nanmax(ref_z[t < t_end]))
    at = np.where((ref_z >= z_tgt - 0.005) & (t > t_off) & (t < t_end))[0]
    t_start = float(t[at[0]]) if at.size else t_off
    return t_start, t_end


def event_marks(events, t0, t1):
    """Shown events in [t0, t1], a repeated command (two LANDs 0.3 s apart) once."""
    out = []
    for tt, e, a in sorted(events):
        if e not in SHOWN or not (t0 <= tt <= t1):
            continue
        if out and out[-1][1] == e and tt - out[-1][0] < 1.0:
            continue
        out.append((tt, e, a))
    return out


def short(e, a):
    if a and e in ('RELEASE', 'DETACH', 'DETECTED') and a.startswith('drone '):
        return f'{e} d{a.split()[1]}'
    return e


def summarise(arg):
    run, logs, out, label, sha = resolve(arg)
    d = pd.read_csv(os.path.join(run, 'logs', 'run.csv'))
    events = metrics.read_event_list(run)
    pp, planner = node_params(logs, 'planner')
    tp, _ = node_params(logs, 'tracker')
    kind = str(pp.get('load_traj', '')) or 'hover'
    t = d.t.to_numpy(float)
    P = d[['payload_x', 'payload_y', 'payload_z']].to_numpy(float)
    R = continuous_ref(t, d[['payload_ref_x', 'payload_ref_y', 'payload_ref_z']].to_numpy(float))
    tilt = d.payload_tilt_deg.to_numpy(float) if 'payload_tilt_deg' in d else np.full(len(t), np.nan)
    t0, t1 = scored_window(t, R[:, 2], events, pp.get('target_z'))
    w = (t >= t0) & (t <= t1)
    e3, along, off, dz, moving = error_parts(t, P, R)
    t_cut = next((tt for tt, e, _ in events if e in ('RELEASE', 'DETACH', 'DETECTED', 'DROP')), None)
    lag = one_clock_lag(logs, kind, t, P[:, 2], t_cut)
    num = {'rmse_3d_cm': 100 * rms(e3, w), 'along_cm': 100 * rms(along, w), 'off_cm': 100 * rms(off, w),
           'height_cm': 100 * rms(dz, w), 'height_mean_cm': 100 * float(np.nanmean(dz[w])),
           'tilt_peak_deg': float(np.nanmax(tilt[w])) if np.isfinite(tilt[w]).any() else math.nan,
           'tilt_rms_deg': rms(tilt, w), 'lag_s': lag, 'window_s': (t0, t1)}
    if np.isfinite(num['tilt_peak_deg']):
        num['tilt_peak_t'] = float(t[w][np.nanargmax(tilt[w])])
    rel = next((tt for tt, e, _ in events if e == 'RELEASE'), None)
    det = next((tt for tt, e, _ in events if e == 'DETECTED'), None)
    if rel is not None and det is not None and det >= rel:
        num['detected_after_s'] = det - rel

    # ── figure ────────────────────────────────────────────────────────────────
    ps.use()
    fig = plt.figure(figsize=(13.5, 7.6), layout='constrained')
    gs = fig.add_gridspec(3, 2, width_ratios=[1.0, 1.45])
    ax_xy = fig.add_subplot(gs[:, 0])
    ax_z = fig.add_subplot(gs[0, 1])
    ax_tilt = fig.add_subplot(gs[1, 1], sharex=ax_z)
    ax_err = fig.add_subplot(gs[2, 1], sharex=ax_z)

    t_off = next((tt for tt, e, _ in events if e == 'TAKEOFF'), float(t[0]))
    t_last = next((tt for tt, e, _ in events if e == 'LANDED' and tt > t0), float(t[-1]))
    xlim = (t_off - 1.0, max(min(float(t[-1]), t_last + 2.0), t_last + 0.5))
    marks = event_marks(events, *xlim)

    # (a) top view
    fly = (t >= t_off) & (t <= xlim[1])
    ax_xy.plot(R[fly, 0], R[fly, 1], color='#555555', label='reference', **ps.DESIRED)
    ax_xy.plot(P[fly & ~w, 0], P[fly & ~w, 1], '.', ms=1.2, color=ps.PAYLOAD_LINE, alpha=0.25)
    ax_xy.plot(np.where(w, P[:, 0], np.nan), np.where(w, P[:, 1], np.nan), color=ps.PAYLOAD_LINE,
               label='ring (scored part)', **ps.ACTUAL)
    for tt, e, a in marks:
        if e in ('RELEASE', 'DETACH', 'DETECTED', 'DROP', 'LAND'):
            i = int(np.argmin(abs(t - tt)))
            c = EVENT_HEX.get(e, '#666666')
            ax_xy.plot(P[i, 0], P[i, 1], 'o', ms=5, mfc='white', mec=c, mew=1.4, zorder=5)
            ax_xy.annotate(short(e, a), (P[i, 0], P[i, 1]), xytext=(5, 4), textcoords='offset points',
                           fontsize=7, color=c)
    xs = np.concatenate([P[w, 0], R[w, 0]])
    ys = np.concatenate([P[w, 1], R[w, 1]])
    cx, cy = (np.nanmax(xs) + np.nanmin(xs)) / 2, (np.nanmax(ys) + np.nanmin(ys)) / 2
    half = max(0.1, 0.55 * max(np.nanmax(xs) - np.nanmin(xs), np.nanmax(ys) - np.nanmin(ys)))
    ax_xy.set_xlim(cx - half, cx + half)
    ax_xy.set_ylim(cy - half, cy + half)
    ax_xy.set_aspect('equal')
    ax_xy.set_anchor('N')
    ax_xy.set_xlabel('x (m)')
    ax_xy.set_ylabel('y (m)')
    ax_xy.set_title('(a) Ring top view')
    ax_xy.legend(loc='upper right')

    # (b) height
    ax_z.plot(t, R[:, 2], color='#555555', label='reference', **ps.DESIRED)
    ax_z.plot(t, P[:, 2], color=ps.PAYLOAD_LINE, label='ring', **ps.ACTUAL)
    ax_z.set_ylabel('z (m)')
    ax_z.set_ylim(min(0.0, float(np.nanmin(P[:, 2]))) - 0.05,
                  max(float(np.nanmax(R[:, 2])), float(np.nanmax(P[:, 2]))) + 0.1)
    ax_z.set_title(f'(b) Ring height   RMSE {num["height_cm"]:.1f} cm, mean {num["height_mean_cm"]:+.1f} cm',
                   loc='left')
    ax_z.legend(loc='center', ncol=2)

    # (c) tilt
    ax_tilt.plot(t, tilt, color=ps.PAYLOAD_LINE, lw=1.3)
    if np.isfinite(num['tilt_peak_deg']):
        ax_tilt.plot(num['tilt_peak_t'], num['tilt_peak_deg'], 'v', color='#333333', ms=5)
        ax_tilt.annotate(f'{num["tilt_peak_deg"]:.1f}°', (num['tilt_peak_t'], num['tilt_peak_deg']),
                         xytext=(5, 0), textcoords='offset points', fontsize=7.5, va='center')
        if pp.get('drop_on_loss') or num['tilt_peak_deg'] > 15.0:
            ax_tilt.axhline(30.0, color='#e41a1c', lw=0.8, ls='--')
            ax_tilt.annotate('30° drop / stop', (xlim[0], 30.0), xytext=(3, 2), textcoords='offset points',
                             fontsize=7, color='#e41a1c')
        ax_tilt.set_title(f'(c) Ring tilt   peak {num["tilt_peak_deg"]:.1f}°, RMS {num["tilt_rms_deg"]:.1f}°',
                          loc='left')
    else:
        ax_tilt.set_title('(c) Ring tilt: no ring orientation (no bag)', loc='left')
    ax_tilt.set_ylabel('tilt (°)')
    ax_tilt.set_ylim(bottom=0)

    # (d) error: zero outside the scored part (the lift-off swing and the landing are not tracking)
    e3, along, off, dz = (np.where(w, x, 0.0) for x in (e3, along, off, dz))
    ax_err.plot(t, 100 * e3, color='#222222', lw=1.4, label=f'3-D  {num["rmse_3d_cm"]:.1f} cm')
    if moving[w].any():
        ax_err.plot(t, 100 * np.abs(along), color=PART_HEX['along'], lw=0.9,
                    label=f'along the path  {num["along_cm"]:.1f} cm')
    ax_err.plot(t, 100 * off, color=PART_HEX['off'], lw=0.9,
                label=f'{"off the path" if moving[w].any() else "sideways"}  {num["off_cm"]:.1f} cm')
    ax_err.plot(t, 100 * np.abs(dz), color=PART_HEX['height'], lw=0.9, label=f'height  {num["height_cm"]:.1f} cm')
    ymax = np.nanpercentile(100 * e3[w], 99.5) if w.any() else 10.0
    ax_err.set_ylim(0, 1.25 * max(ymax, 1.0))
    ax_err.set_ylabel('error (cm)')
    ax_err.set_xlabel('t (s)')
    ax_err.set_title('(d) Ring error from the reference (RMSE over the scored part)', loc='left')
    ps.busy_legend(ax_err, loc='upper left', ncol=4)

    for ax in (ax_z, ax_tilt, ax_err):
        ax.axvspan(t0, t1, color='#000000', alpha=0.045, lw=0, zorder=0)
        for tt, e, a in marks:
            ax.axvline(tt, color=EVENT_HEX.get(e, '#888888'), ls=':', lw=1.0, zorder=1)
    ax_z.set_xlim(*xlim)
    ytop = ax_z.get_ylim()[1]
    for tt, e, a in marks:
        ax_z.annotate(short(e, a), (tt, ytop), xytext=(2, -2), textcoords='offset points', rotation=90,
                      va='top', ha='left', fontsize=6.5, color=EVENT_HEX.get(e, '#666666'))
    ax_z.annotate('scored', ((t0 + t1) / 2, 0), xytext=(0, 2), textcoords='offset points', ha='center',
                  fontsize=7, color='#777777')
    plt.setp(ax_z.get_xticklabels(), visible=False)
    plt.setp(ax_tilt.get_xticklabels(), visible=False)

    path = PATHS.get(kind, kind)
    if kind in ('orbit', 'circle', 'fig_8'):
        path += f' r {float(pp.get("traj_radius", 0)):.2g} m at {float(pp.get("traj_speed", 0)):.3g} m/s'
    mode = {'mpc_planner': 'mpc', 'dissipative_planner': 'dissipative'}.get(planner, planner or '?')
    rts = tp.get('ref_time_shift')
    head = f'{label}   {mode}, {path}' + (f', ref_time_shift {"on" if rts else "off"}' if rts is not None else '')
    line = [f'3-D RMSE {num["rmse_3d_cm"]:.1f} cm']
    if lag is not None:
        line.append(f'lag {lag:.2f} s' + (' (before the release)' if t_cut is not None else ''))
    if 'detected_after_s' in num:
        line.append(f'detected {num["detected_after_s"]:.2f} s after the release')
    if np.isfinite(num['tilt_peak_deg']):
        line.append(f'tilt peak {num["tilt_peak_deg"]:.1f}°')
    if any(e == 'DROP' for _, e, _ in events):
        line.append('DROPPED')
    line.append(f'scored {t0:.1f}-{t1:.1f} s')
    fig.suptitle(head + '\n' + '   |   '.join(line), fontsize=11, x=0.01, ha='left')
    ps.stamp(fig, label, sha)
    os.makedirs(os.path.dirname(out), exist_ok=True)
    fig.savefig(out, dpi=150)
    plt.close(fig)
    return out, num, head, line


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('runs', nargs='+', help='rig run name or path (r303, results/rig/<day>/r303), or a twin run (R1165)')
    a = ap.parse_args(argv)
    for r in a.runs:
        out, num, head, line = summarise(r)
        print(f'{head}\n  ' + '  |  '.join(line) + f'\n  -> {out}')
    return 0


if __name__ == '__main__':
    sys.exit(main())

#!/usr/bin/env python3
"""tools/x1_metrics.py -- score X1 release-ablation runs (card docs/experiments/2026-10-30_x1_release_ablation.md).

    python3 tools/x1_metrics.py R2915 R2916 [--json results/analysis/x1_2026-10-30.json]

Per run, t_e = the RELEASE event (runner clock), the window ends at LAND:
  tilt rise    release_bands' delta-theta: the tilt vector's largest departure from its mean over the 5 s before t_e
  abs peak     the largest tilt magnitude, t_e to LAND
  dip          the ring's mean height over the 5 s before t_e minus its lowest height in t_e .. t_e + 10 s (mm)
  drop         DROP AND LAND in the launch log, or the ring under 0.3 m before LAND
  t_level      the first time after the peak from which the tilt vector stays within max(2 sigma_pre, 1 deg) of its
               pre-release mean for 1 s (censored at LAND: None); beside it the settled tilt (mean magnitude, last 5 s
               before LAND) and release_bands' T6 time back in band
  path error   RMS of the ring's 3-D distance to its reference path over t_e .. t_e + 10 s: in a hover to the reference
               point, on a path clock-free (across the path and height, as tools/n2_bands.py's b22 set)
  validity     detection (the planner's 'detected at' line for a delayed arm, else its resize tick), the resize (the
               planner's first tick at n-1, s_metrics), late ticks, failed solves before t_e, read-back arm values
"""
import argparse
import glob
import json
import os
import re
import sys

import numpy as np
import pandas as pd
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import metrics as M  # noqa: E402
import release_bands as RB  # noqa: E402
import run_logs  # noqa: E402
import s_metrics as S  # noqa: E402

AFTER_S = 10.0          # dip and path-error window after the release
LEVEL_BAND_MIN = 1.0    # deg: t_level's smallest band
HOLD_S = 1.0            # s: t_level holds this long
DROP_Z = 0.3            # m: a ring under this before LAND was dropped


def _clock_free_err(df, t, w):
    """The ring's 3-D distance to the path its reference traces, at the rows w (None on a hover)."""
    des = df[['payload_ref_x', 'payload_ref_y']].to_numpy(float)
    sweep = M.sweep_window(t, des)
    if not sweep.any():
        return None
    tk, xk = M.schedule_knots(t, des, sweep)
    path = M.path_through(xk)
    s_sched = np.interp(t, tk, path[5])
    across, _, _ = M.project_on_path(path, df[['payload_x', 'payload_y']].to_numpy(float), np.where(w, s_sched, np.nan))
    dz = df.payload_z.to_numpy(float) - df.payload_ref_z.to_numpy(float)
    return np.sqrt(across ** 2 + dz ** 2)[w]


def _readback(run_dir):
    out = {}
    pf = os.path.join(run_dir, 'params', 'dissipative_planner.yaml')
    if os.path.isfile(pf):
        txt = open(pf).read()
        m = re.search(r'resize_delay_s:\s*([-\d.eE+]+)', txt)
        out['resize_delay_s'] = float(m.group(1)) if m else None
    srcs = set()
    for tf in glob.glob(os.path.join(run_dir, 'params', 'tracker_*.yaml')):
        m = re.search(r'cable_source:\s*(\S+)', open(tf).read())
        if m:
            srcs.add(m.group(1).strip("'\""))
    out['cable_source'] = ','.join(sorted(srcs)) or None
    return out


def score(run_dir):
    logs = os.path.join(run_dir, 'logs')
    df = pd.read_csv(os.path.join(logs, 'run.csv')).drop_duplicates('t').reset_index(drop=True)
    with open(os.path.join(run_dir, 'manifest.json')) as fh:
        man = json.load(fh)
    name = os.path.basename(run_dir.rstrip('/'))
    m = re.search(r'x1_([a-z0-9]+)_([a-z0-9]+)$', name)
    cell, arm = (m.group(1), m.group(2)) if m else (None, None)
    rb = RB.Run(run_dir)
    te, tland = rb.te, rb.tland
    out = dict(run=name.split('_')[0], cell=cell, arm=arm, t_release=te, t_land=tland)
    met = rb.metrics()
    out.update(tilt_rise=met['dtheta'], t_peak=met['t_peak'], abs_peak=met['abs_peak'], sigma_pre=met['sigma_pre'],
               settled_tilt=met['post'], t_back_t6=met['t_back'])
    mean, sig, _ = rb.pre()
    band = max(2.0 * sig, LEVEL_BAND_MIN)
    inb = np.linalg.norm(rb.th - mean, axis=1) <= band
    out['t_level'] = None
    for i in np.nonzero((rb.t >= te + met['t_peak']) & (rb.t + HOLD_S <= tland))[0]:
        if inb[rb.win(rb.t[i], rb.t[i] + HOLD_S)].all():
            out['t_level'] = float(rb.t[i] - te)
            break
    out['level_band_deg'] = band
    t = df.t.to_numpy(float)
    z = df.payload_z.to_numpy(float)
    pre = (t >= te - RB.PRE_S) & (t < te)
    aft = (t >= te) & (t <= min(te + AFTER_S, tland))
    out['dip_mm'] = 1000.0 * float(np.nanmean(z[pre]) - np.nanmin(z[aft]))
    err = _clock_free_err(df, t, aft)
    if err is None:
        e = (df[['payload_x', 'payload_y', 'payload_z']].to_numpy(float)
             - df[['payload_ref_x', 'payload_ref_y', 'payload_ref_z']].to_numpy(float))[aft]
        err = np.linalg.norm(e, axis=1)
        out['path_kind'] = 'point'
    else:
        out['path_kind'] = 'clock-free'
    err = err[np.isfinite(err)]
    out['path_err_mm'] = 1000.0 * float(np.sqrt(np.mean(err ** 2))) if err.size else None
    s = S.score(run_dir)
    rel = next((e for e in s['events'] if e['event'] == 'RELEASE'), None)
    out['resize_s'] = rel['resize_s'] if rel else None
    out['n_after'] = rel['n_after'] if rel else None
    launch = os.path.join(logs, 'launch.log')
    txt = open(launch, errors='replace').read() if os.path.isfile(launch) else ''
    low = (t < tland) & (t > te) & (z < DROP_Z)
    out['dropped'] = bool(s['dropped'] or low.any())
    out['drop_reason'] = (re.findall(r'DROP AND LAND[^\n]*', txt) or [None])[0]
    det = re.search(r'stays in the model, the loss is acted on at t=([\d.]+) s \(detected at ([\d.]+) s\)', txt)
    t_ready = float(man.get('t_ready_sim', 0.0))
    out['detect_s'] = (float(det.group(2)) - t_ready - te) if det else (out['resize_s'] if arm != 'nors' else None)
    out['unannounced_lines'] = len(re.findall(r'UNANNOUNCED DETACH', txt))
    out['late_resize_lines'] = len(re.findall(r'LATE RESIZE', txt))
    pl = s.get('planner', {})
    out['late_planner'] = pl.get('late')
    out['late_tracker'] = s.get('tracker_late_max')
    out['failed_solves'] = pl.get('failed')
    out['void_late'] = s['void']
    out.update(_readback(run_dir))
    out['rtf'] = man.get('mean_rtf')
    return out


def locate(tag):
    if os.path.isdir(tag):
        return tag
    hits = [h for h in glob.glob(os.path.join(RB.RESULTS, '2*', tag + '_*')) if os.path.isdir(h)]
    if len(hits) != 1:
        sys.exit(f'{tag}: {len(hits)} run directories')
    return hits[0]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('runs', nargs='+')
    ap.add_argument('--json')
    a = ap.parse_args()
    rows = [score(locate(r)) for r in a.runs]
    for r in rows:
        f = lambda v, nd=2: '-' if v is None else (f'{v:.{nd}f}' if isinstance(v, float) else str(v))
        print(f"{r['run']} {r['cell']}/{r['arm']}: rise {f(r['tilt_rise'])} abs {f(r['abs_peak'])} dip {f(r['dip_mm'], 1)} "
              f"drop {r['dropped']} t_level {f(r['t_level'])} settled {f(r['settled_tilt'])} path {f(r['path_err_mm'], 1)} "
              f"({r['path_kind']}) | detect {f(r['detect_s'])} resize {f(r['resize_s'])} n {r['n_after']} late "
              f"{f(r['late_planner'], 4)}/{f(r['late_tracker'], 4)} failed {r['failed_solves']} delay "
              f"{r['resize_delay_s']} src {r['cable_source']} rtf {f(r['rtf'])}")
    if a.json:
        os.makedirs(os.path.dirname(a.json), exist_ok=True)
        with open(a.json, 'w') as fh:
            json.dump(rows, fh, indent=1)


if __name__ == '__main__':
    main()

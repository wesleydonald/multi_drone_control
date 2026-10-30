#!/usr/bin/env python3
"""tools/x4_metrics.py -- score an X4 endurance relay run (card docs/experiments/2026-10-30_x4_endurance_relay.md).

    python3 tools/x4_metrics.py R2950 R2951 [--budget 300] [--json results/analysis/x4_2026-11-07.json]

Reads run.csv and events.csv (the runner's and the planner's lines on the run clock) and the run's config (the
manifest's scenario: its relay: rota, tools/experiment/relay.py):
  mission     TAKEOFF -> /fleet/landed (s), and over the pack budget B
  swaps       leaves tried (DETACH sent), released (the planner's resize), parked at the pad, back on the ring
              (REJOIN WELD); a swap is done when its drone was released and welded back
  faults      TASK REFUSED / ABORT, REJOIN REFUSED / ABORT, RELAY HOLD (the path stopped mid-lap), RELAY LEAVE
              CANCELLED, LOG/TRAJ timeouts, a drop
  events      every release (the planner's resize) and fold-in (REJOIN WELD): the ring's tilt peak from its mean
              over the PRE_S before it (release_bands' tilt vector), up to POST_S after it or the next event
  laps        each lap (PATH_START or 'lap k-1 done' to 'lap k done', the rows where the ring's reference moves):
              the clock-free path error (RMS of the distance to the figure-8 and the height error), the tilt peak,
              the mean height error, and the carriers on the ring; the trend: last lap over the first, and the
              slope over the laps (mm per lap); the height drift: the last lap's mean height error less the first's
  packs       each drone's pack intervals: from TAKEOFF or the end of a pad dwell to its next pad arrival or the
              fleet's touchdown (an errand does not recharge); the longest against B
"""
import argparse
import json
import os
import sys

import numpy as np
import pandas as pd

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import release_bands as RB  # noqa: E402

PRE_S = 5.0               # s before an event that set the tilt it is measured from
POST_S = 10.0             # s after an event its peak is looked for (or to the next event)
REF_MOVING_MPS = 0.002    # the ring's reference moves faster than this while the path runs (not in a hold)
FAULTS = ('TASK_REFUSED', 'TASK_ABORT', 'REJOIN_REFUSED', 'REJOIN_ABORT', 'RELAY_HOLD', 'RELAY_LEAVE_CANCELLED',
          'LOG_TIMEOUT', 'TRAJ_TIMEOUT', 'STAGED_LEAVES')


def load_config(run_dir, path=None):
    """The run's ExperimentConfig (its relay plan) from the manifest's scenario, or path."""
    from experiment.config import ExperimentConfig
    if path is None:
        with open(os.path.join(run_dir, 'manifest.json')) as fh:
            path = json.load(fh)['scenario']
        if not os.path.exists(path):
            path = os.path.join(os.path.dirname(HERE), 'configs', 'experiments', os.path.basename(path))
    return ExperimentConfig.from_yaml(path)


def _launch_says(logs, text):
    path = os.path.join(logs, 'launch.log')
    if not os.path.exists(path):
        return False
    with open(path, errors='replace') as fh:
        return any(text in line for line in fh)


def _drone(arg):
    try:
        return int(float(arg))
    except (TypeError, ValueError):
        return None


def event_tilts(t, th, marks):
    """[(t, kind, drone, peak deg)] of each (t, kind, drone) mark: the tilt vector's largest departure from its mean
    over PRE_S before it, up to POST_S after it or the next mark."""
    out = []
    times = [m[0] for m in marks]
    for k, (te, kind, d) in enumerate(marks):
        end = min(te + POST_S, times[k + 1] if k + 1 < len(times) else te + POST_S)
        pre, win = (t >= te - PRE_S) & (t < te), (t >= te) & (t <= max(end, te + 0.5))
        peak = float(np.linalg.norm(th[win] - th[pre].mean(0), axis=1).max()) if pre.any() and win.any() else None
        out.append({'t': te, 'kind': kind, 'drone': d, 'peak_deg': peak})
    return out


def dist_to_polyline(points, curve):
    """Per point (xy): the distance to the polyline `curve` (xy), its segments, not only its vertices."""
    p = np.asarray(points, float)[:, None, :]
    a, b = np.asarray(curve, float)[:-1][None], np.asarray(curve, float)[1:][None]
    ab = b - a
    s = np.clip(np.sum((p - a) * ab, axis=2) / np.maximum(np.sum(ab * ab, axis=2), 1e-12), 0.0, 1.0)
    return np.min(np.linalg.norm(p - (a + s[..., None] * ab), axis=2), axis=1)


def lap_rows(run, ev, curve):
    """Per lap: clock-free path error (m), tilt peak (deg), mean height error (m), carriers."""
    t = run['t'].to_numpy(float)
    ref = run[['payload_ref_x', 'payload_ref_y']].to_numpy(float)
    v = np.r_[0.0, np.linalg.norm(np.diff(ref, axis=0), axis=1) / np.maximum(np.diff(t), 1e-6)]
    moving = v > REF_MOVING_MPS
    starts = [float(r.sim_time) for r in ev.itertuples() if r.event == 'PATH_START'][:1]
    ends = [float(r.sim_time) for r in ev.itertuples() if r.event == 'LAP_DONE']
    t_land = next((float(r.sim_time) for r in ev.itertuples() if r.event in ('LAND', 'PLANNER_LAND')), float(t[-1]))
    bounds = (starts or [float(t[0])]) + ends + [t_land]
    xy = run[['payload_x', 'payload_y']].to_numpy(float)
    dz = (run['payload_z'] - run['payload_ref_z']).to_numpy(float)
    att = [c for c in run.columns if c.endswith('_attached')]
    rows = []
    for k, (a, b) in enumerate(zip(bounds[:-1], bounds[1:]), 1):
        w = moving & (t > a) & (t <= b) & np.isfinite(xy).all(1) & np.isfinite(dz)
        if w.sum() < 10:
            continue
        across = dist_to_polyline(xy[w], curve)
        e = np.sqrt(across ** 2 + dz[w] ** 2)
        n = int(np.median(run.loc[w, att].sum(axis=1))) if att else None
        rows.append({'lap': k, 't0': float(t[w][0]), 't1': float(t[w][-1]), 'path_rms_m': float(np.sqrt(np.mean(e ** 2))),
                     'tilt_peak_deg': float(np.nanmax(run['payload_tilt_deg'].to_numpy(float)[w])),
                     'dz_mean_m': float(np.mean(dz[w])), 'carriers': n})
    return rows


def packs(ev, plan, t_takeoff, t_down):
    """drone -> [pack intervals s] (TAKEOFF or a pad dwell's end -> the next pad arrival or touchdown), and whether
    the run's TASK AT lines matched the rota."""
    at = {}
    for r in ev.itertuples():
        if r.event == 'TASK_AT':
            at.setdefault(_drone(r.arg), []).append(float(r.sim_time))
    seen = {d: 0 for d in at}
    reset = {d: t_takeoff for d in range(plan.geo.n)}
    out = {d: [] for d in range(plan.geo.n)}
    matched = True
    for lv in plan.leaves:
        k = seen.get(lv.d, 0)
        times = at.get(lv.d, [])
        if k + len(lv.wps) > len(times):
            matched = False
            break
        seen[lv.d] = k + len(lv.wps)
        if lv.kind == 'pad':
            arrive = times[k + len(lv.wps) - 1]
            out[lv.d].append(arrive - reset[lv.d])
            reset[lv.d] = arrive + float(lv.wps[-1][3])
    for d in out:
        out[d].append(t_down - reset[d])
    return out, matched


def score(run_dir, budget=None, config=None):
    cfg = load_config(run_dir, config)
    plan = cfg.relay
    B = float(budget if budget is not None else (plan.budget if plan is not None else 300.0))
    logs = os.path.join(run_dir, 'logs')
    run = pd.read_csv(os.path.join(logs, 'run.csv')).drop_duplicates('t').reset_index(drop=True)
    ev = pd.read_csv(os.path.join(logs, 'events.csv'))
    ev = ev[pd.to_numeric(ev['sim_time'], errors='coerce').notna()].copy()
    ev['sim_time'] = ev['sim_time'].astype(float)
    kinds = ev['event'].astype(str).tolist()
    first = lambda k: next((float(r.sim_time) for r in ev.itertuples() if r.event == k), None)  # noqa: E731
    t_takeoff = first('TAKEOFF') or 0.0
    t_down = first('LANDED') or float(run['t'].max())
    by = lambda k: [(float(r.sim_time), _drone(r.arg)) for r in ev.itertuples() if r.event == k]  # noqa: E731
    tried, released, parked, welded = by('DETACH'), by('DETACH_RESIZE'), by('TASK_PARKED'), by('REJOIN_WELD')
    done = 0
    for tr, d in released:
        if any(tw > tr and dw == d for tw, dw in welded):
            done += 1
    qcols = ['payload_qw', 'payload_qx', 'payload_qy', 'payload_qz']
    ok = run.dropna(subset=qcols)
    th = RB.tilt_vector(ok[qcols].to_numpy(float))
    marks = sorted([(t, 'release', d) for t, d in released] + [(t, 'fold_in', d) for t, d in welded])
    out = {'run': os.path.basename(run_dir.rstrip('/')), 'budget_s': B,
           'mission_s': t_down - t_takeoff, 'mission_over_b': (t_down - t_takeoff) / B,
           'swaps_tried': len(tried), 'released': len(released), 'parked': len(parked), 'welded_back': len(welded),
           'swaps_done': done, 'faults': {k: kinds.count(k) for k in FAULTS if kinds.count(k)},
           'dropped': any('DROP' in k for k in kinds) or _launch_says(logs, 'DROP AND LAND'),
           'events': event_tilts(ok['t'].to_numpy(float), th, marks)}
    peaks = [e['peak_deg'] for e in out['events'] if e['peak_deg'] is not None]
    out['event_tilt_max_deg'] = max(peaks) if peaks else None
    if plan is not None:
        ts = run['t'].to_numpy(float)
        i0 = next((i for i, r in enumerate(ev.itertuples()) if r.event == 'PATH_START'), None)
        t0 = float(ev['sim_time'].iloc[i0]) if i0 is not None else float(ts[0])
        hx, hy = (run.loc[np.searchsorted(ts, t0).clip(0, len(ts) - 1), ['payload_ref_x', 'payload_ref_y']]
                  .to_numpy(float))
        lap = plan.geo.lap - plan.geo.ring + np.array([hx, hy])
        out['laps'] = lap_rows(run, ev, lap)
        if out['laps']:
            e = np.array([r['path_rms_m'] for r in out['laps']])
            out['last_over_first'] = float(e[-1] / e[0]) if e[0] > 0 else None
            out['slope_mm_per_lap'] = float(np.polyfit(np.arange(len(e)), 1e3 * e, 1)[0]) if len(e) > 1 else 0.0
            out['height_drift_m'] = out['laps'][-1]['dz_mean_m'] - out['laps'][0]['dz_mean_m']
        p, matched = packs(ev, plan, t_takeoff, t_down)
        out['packs'] = {str(d): v for d, v in p.items()}
        out['packs_matched_rota'] = matched
        out['max_pack_s'] = max(max(v) for v in p.values())
        out['over_b'] = {str(d): max(v) for d, v in p.items() if max(v) > B}
    return out


def locate(tag):
    """A run directory from R#### (results/<date>/R####_*) or a path."""
    if os.path.isdir(tag):
        return tag
    import glob
    hits = sorted(glob.glob(os.path.join(os.path.dirname(HERE), 'results', '*', f'{tag}_*')))
    if not hits:
        sys.exit(f'{tag}: no run directory')
    return hits[-1]


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('runs', nargs='+')
    ap.add_argument('--budget', type=float, help='B (s); default the config\'s relay budget_s')
    ap.add_argument('--config', help='the config, when the manifest\'s scenario is not this tree\'s')
    ap.add_argument('--json')
    a = ap.parse_args(argv)
    res = [score(locate(r), a.budget, a.config) for r in a.runs]
    for r in res:
        print(f"{r['run']}: mission {r['mission_s']:.0f} s = {r['mission_over_b']:.2f} B; swaps {r['swaps_done']}/"
              f"{r['swaps_tried']} (released {r['released']}, parked {r['parked']}, back {r['welded_back']}); "
              f"event tilt max {r['event_tilt_max_deg'] or 0:.2f} deg; faults {r['faults'] or 'none'}"
              + (' DROPPED' if r['dropped'] else ''))
        if r.get('laps'):
            print(f"  laps {len(r['laps'])}: path {1e3 * r['laps'][0]['path_rms_m']:.1f} -> "
                  f"{1e3 * r['laps'][-1]['path_rms_m']:.1f} mm (x{r['last_over_first']:.2f}, "
                  f"{r['slope_mm_per_lap']:+.2f} mm/lap), height drift {1e3 * r['height_drift_m']:+.1f} mm")
            print(f"  longest pack {r['max_pack_s']:.0f} s against B {r['budget_s']:.0f} s"
                  + (f"; over B: {r['over_b']}" if r['over_b'] else '')
                  + ('' if r['packs_matched_rota'] else ' (TASK AT lines did not match the rota: packs partial)'))
    if a.json:
        with open(a.json, 'w') as fh:
            json.dump(res, fh, indent=1)
    return 0


if __name__ == '__main__':
    sys.exit(main())

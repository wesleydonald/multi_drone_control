#!/usr/bin/env python3
"""tools/x4_metrics.py -- score an X4 endurance relay run (card docs/experiments/2026-10-30_x4_endurance_relay.md).

    python3 tools/x4_metrics.py R2950 R2951 [--budget 300] [--json results/analysis/x4_2026-11-07.json] [--bars]

--bars scores the card's frozen bars (R1-R7 per relay run, F1 per fixed-team run; with --json also <json>_bars.json).

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
              the clock-free path error (RMS of the distance to the figure-8 and the height error), the tilt's peak,
              RMS and p95, the mean height error, and the carriers on the ring; the trend: last lap over the first, and
              the slope over the laps (mm per lap); the height drift: the last lap's mean height error less the first's;
              the tilt's RMS and p95 over all the laps together (the mission's, descriptive)
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
import metrics as M  # noqa: E402
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
    tilt = run['payload_tilt_deg'].to_numpy(float)
    rows = []
    for k, (a, b) in enumerate(zip(bounds[:-1], bounds[1:]), 1):
        w = moving & (t > a) & (t <= b) & np.isfinite(xy).all(1) & np.isfinite(dz)
        if w.sum() < 10:
            continue
        across = dist_to_polyline(xy[w], curve)
        e = np.sqrt(across ** 2 + dz[w] ** 2)
        n = int(np.median(run.loc[w, att].sum(axis=1))) if att else None
        rows.append({'lap': k, 't0': float(t[w][0]), 't1': float(t[w][-1]), 'path_rms_m': float(np.sqrt(np.mean(e ** 2))),
                     'tilt_peak_deg': float(np.nanmax(tilt[w])), 'tilt_rms_deg': float(np.sqrt(np.nanmean(tilt[w] ** 2))),
                     'tilt_p95_deg': float(np.nanpercentile(tilt[w], 95)),
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
    # events.csv can miss a line the planner's log has (R3456's last REJOIN WELD): the fold-in comes right after its weld and proves it
    welded += [(tf, d) for tf, d in by('FOLD_IN') if not any(tf - 5.0 < tw <= tf and dw == d for tw, dw in welded)]
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
    pn = planner_checks(run_dir, run, t_down)
    pt_n = pn.pop('_t_n')
    out.update(pn)
    peaks = [e['peak_deg'] for e in out['events'] if e['peak_deg'] is not None]
    out['event_tilt_max_deg'] = max(peaks) if peaks else None
    out['leaves_planned'] = plan.n_leaves if plan is not None else None
    out['role'] = 'fixed' if not out['leaves_planned'] else 'relay'
    with open(os.path.join(run_dir, 'manifest.json')) as fh:
        out['rtf'] = json.load(fh).get('mean_rtf')
    if plan is not None:
        ts = run['t'].to_numpy(float)
        i0 = next((i for i, r in enumerate(ev.itertuples()) if r.event == 'PATH_START'), None)
        t0 = float(ev['sim_time'].iloc[i0]) if i0 is not None else float(ts[0])
        hx, hy = (run.loc[np.searchsorted(ts, t0).clip(0, len(ts) - 1), ['payload_ref_x', 'payload_ref_y']]
                  .to_numpy(float))
        lap = plan.geo.lap - plan.geo.ring + np.array([hx, hy])
        out['laps'] = lap_rows(run, ev, lap)
        for lp in out['laps']:
            w = (pt_n[0] >= lp['t0']) & (pt_n[0] <= lp['t1'])
            lp['planner_n'] = [int(pt_n[1][w].min()), int(pt_n[1][w].max())] if w.any() else None
        if out['laps']:
            e = np.array([r['path_rms_m'] for r in out['laps']])
            out['last_over_first'] = float(e[-1] / e[0]) if e[0] > 0 else None
            out['slope_mm_per_lap'] = float(np.polyfit(np.arange(len(e)), 1e3 * e, 1)[0]) if len(e) > 1 else 0.0
            out['height_drift_m'] = out['laps'][-1]['dz_mean_m'] - out['laps'][0]['dz_mean_m']
            tilts = np.concatenate([run.loc[(ts >= r['t0']) & (ts <= r['t1']), 'payload_tilt_deg'].to_numpy(float)
                                    for r in out['laps']])
            out['tilt_rms_deg'] = float(np.sqrt(np.nanmean(tilts ** 2)))
            out['tilt_p95_deg'] = float(np.nanpercentile(tilts, 95))
        p, matched = packs(ev, plan, t_takeoff, t_down)
        out['packs'] = {str(d): v for d, v in p.items()}
        out['packs_matched_rota'] = matched
        out['max_pack_s'] = max(max(v) for v in p.values())
        out['over_b'] = {str(d): max(v) for d, v in p.items() if max(v) > B}
    return out


def planner_checks(run_dir, run, t_down):
    """The planner's smallest fleet size while it flies (phase planner, before touchdown; R2: run.csv's d*_attached
    columns read 0 in these runs, so the planner's n stands for the carriers) and its late ticks (s_metrics' rule, the
    void's 1 %)."""
    import run_logs
    import s_metrics as S
    pdirs = run_logs.node_dirs(run_dir, 'dissipative_planner') + run_logs.node_dirs(run_dir, 'mpc_planner')
    if not pdirs:
        return {'planner_n_min': None, 'late_planner': None, '_t_n': (np.array([]), np.array([], int))}
    with open(os.path.join(run_dir, 'manifest.json')) as fh:
        t_ready = float(json.load(fh)['t_ready_sim'])
    p = S.node_log(os.path.join(pdirs[-1], 'log.csv'), t_ready, float(run['t'].min()), float(run['t'].max()))
    fly = (p['phase'].to_numpy() == 'planner') & (p['sim_time'].to_numpy(float) < t_down)
    live = p['phase'].to_numpy() == 'planner'
    return {'planner_n_min': int(p['n'][fly].min()) if fly.any() else None,
            'late_planner': S.late_fraction(p['sim_time'].to_numpy(float), 0.1),
            '_t_n': (p['sim_time'].to_numpy(float)[live], p['n'].to_numpy(int)[live])}


MISSION_MIN_S, CARRIERS_MIN, EVENT_TILT_MAX, LAST_OVER_FIRST_MAX, DRIFT_MAX_M = 1700.0, 7, 5.0, 1.25, 0.010
LATE_VOID, RTF_VOID = 0.01, 0.15
R1_FAULTS = ('TASK_REFUSED', 'TASK_ABORT', 'REJOIN_REFUSED', 'REJOIN_ABORT', 'RELAY_LEAVE_CANCELLED')


def bars(res):
    """The card's frozen bars (section 5): R1-R7 on each relay run, F1 on each fixed-team run; R4's lap bar against the
    fixed team's mean lap (every lap of the fixed runs given); void per section 5 (late planner ticks, RTF)."""
    fixed = [r for r in res if r['role'] == 'fixed']
    laps = [lp['path_rms_m'] for r in fixed for lp in r.get('laps') or []]
    mean_lap = float(np.mean(laps)) if laps else None
    lap_bar = max(1.25 * mean_lap, mean_lap + 0.003) if laps else None
    out = {'fixed_mean_lap_m': mean_lap, 'lap_bar_m': lap_bar, 'runs': []}
    for r in res:
        void = [w for w, bad in (('late planner ticks over 1 %', (r.get('late_planner') or 0) > LATE_VOID),
                                 ('RTF under 0.15', (r.get('rtf') or 1) < RTF_VOID)) if bad]
        row = {'run': r['run'], 'role': r['role'], 'void': void}
        if r['role'] == 'fixed':
            row['F1'] = bool(r['mission_s'] <= r['budget_s'] and not r['dropped'])
        else:
            faults = {k: v for k, v in r['faults'].items() if k in R1_FAULTS}
            carriers = [n for lp in r.get('laps') or [] if lp.get('planner_n') for n in lp['planner_n']]
            worst = max((lp['path_rms_m'] for lp in r.get('laps') or []), default=None)
            row.update({
                'R1': bool(r['swaps_tried'] == r['leaves_planned'] and r['swaps_done'] == r['swaps_tried']
                           and r['mission_s'] >= MISSION_MIN_S and not r['dropped'] and not faults),
                'R1_falsified': bool(r['dropped'] or faults or r['swaps_done'] < r['swaps_tried']),
                'R1_items': {'swaps': f"{r['swaps_done']}/{r['swaps_tried']} of {r['leaves_planned']}",
                             'mission_s': r['mission_s'], 'faults': faults},
                'R2': bool(carriers and min(carriers) >= CARRIERS_MIN
                           and (r.get('planner_n_min') is None or r['planner_n_min'] >= CARRIERS_MIN)),
                'R2_items': {'carriers': [min(carriers), max(carriers)] if carriers else None,
                             'planner_n_min': r.get('planner_n_min')},
                'R3': bool(r['event_tilt_max_deg'] is not None and r['event_tilt_max_deg'] <= EVENT_TILT_MAX),
                'R4': bool(r.get('last_over_first') is not None and r['last_over_first'] <= LAST_OVER_FIRST_MAX),
                'R4_lap_bar': None if lap_bar is None or worst is None else bool(worst <= lap_bar),
                'R4_worst_lap_m': worst,
                'R5': bool(r.get('height_drift_m') is not None and abs(r['height_drift_m']) <= DRIFT_MAX_M),
                'R6': not r['faults'].get('RELAY_HOLD'),
                'R7': bool(r.get('max_pack_s') is not None and r['max_pack_s'] <= r['budget_s'])})
        out['runs'].append(row)
    return out


def bars_text(b, res):
    yes = lambda v: '-' if v is None else ('holds' if v else 'MISS')  # noqa: E731
    by = {r['run']: r for r in res}
    rel = [x for x in b['runs'] if x['role'] == 'relay']
    lines = []
    if rel:
        hdr = '| Bar (section 5) | ' + ' | '.join(x['run'].split('_')[0] for x in rel) + ' |'
        lines += [hdr, '|' + '---|' * (len(rel) + 1)]
        bar = f"{1e3 * b['lap_bar_m']:.1f}" if b['lap_bar_m'] else '- (no fixed run given)'

        def row(name, f):
            lines.append(f'| {name} | ' + ' | '.join(f(x, by[x['run']]) for x in rel) + ' |')
        row('R1 swaps done of planned', lambda x, r: f"{x['R1_items']['swaps']}, faults {x['R1_items']['faults'] or 'none'}")
        row('R1 mission >= 1700 s', lambda x, r: f"{r['mission_s']:.0f} s, {yes(r['mission_s'] >= MISSION_MIN_S)}")
        row('R1 (whole bar)', lambda x, r: yes(x['R1']) + (' (falsified)' if x['R1_falsified'] else ''))
        row("R2 carriers >= 7 (the planner's n over the laps; its least in flight)",
            lambda x, r: f"{x['R2_items']['carriers']}; {x['R2_items']['planner_n_min']}: {yes(x['R2'])}")
        row('R3 event tilt peak <= 5.0 deg', lambda x, r: f"{r['event_tilt_max_deg'] or 0:.2f}: {yes(x['R3'])}")
        row('R4 last lap over first <= 1.25', lambda x, r: f"x{r.get('last_over_first') or 0:.2f}: {yes(x['R4'])}")
        row(f"R4 worst lap <= {bar} mm", lambda x, r: f"{1e3 * (x['R4_worst_lap_m'] or 0):.1f}: {yes(x['R4_lap_bar'])}")
        row('R5 height drift <= 10 mm', lambda x, r: f"{1e3 * (r.get('height_drift_m') or 0):+.1f}: {yes(x['R5'])}")
        row('R6 RELAY HOLD lines', lambda x, r: f"{r['faults'].get('RELAY_HOLD', 0)}: {yes(x['R6'])}")
        row('R7 longest pack <= B', lambda x, r: f"{r.get('max_pack_s') or 0:.0f} s: {yes(x['R7'])}")
        row('mission over B (reported)', lambda x, r: f"{r['mission_over_b']:.2f}")
        row('late planner ticks, RTF (void)', lambda x, r: f"{100 * (r.get('late_planner') or 0):.2f} %, "
            f"{r.get('rtf') or 0:.2f}" + (f" VOID ({'; '.join(x['void'])})" if x['void'] else ''))
    for x in b['runs']:
        if x['role'] == 'fixed':
            r = by[x['run']]
            worst = max((lp['path_rms_m'] for lp in r.get('laps') or []), default=0.0)
            lines.append(f"fixed team {x['run'].split('_')[0]}: mission {r['mission_s']:.0f} s = {r['mission_over_b']:.2f} B, "
                         f"{'DROPPED' if r['dropped'] else 'no drop'}, worst lap {1e3 * worst:.1f} mm: F1 {yes(x['F1'])}"
                         + (f" VOID ({'; '.join(x['void'])})" if x['void'] else ''))
    if b['fixed_mean_lap_m']:
        lines.append(f"fixed team's mean lap {1e3 * b['fixed_mean_lap_m']:.1f} mm -> R4's lap bar {1e3 * b['lap_bar_m']:.1f} mm")
    return '\n'.join(lines)


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
    ap.add_argument('--bars', action='store_true', help="also score the card's frozen bars (give the fixed team's runs "
                    "with the relays: R4's lap bar is their mean lap)")
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
                  f"{r['slope_mm_per_lap']:+.2f} mm/lap), height drift {1e3 * r['height_drift_m']:+.1f} mm, tilt RMS "
                  f"{r['tilt_rms_deg']:.2f} p95 {r['tilt_p95_deg']:.2f} deg")
            print(f"  longest pack {r['max_pack_s']:.0f} s against B {r['budget_s']:.0f} s"
                  + (f"; over B: {r['over_b']}" if r['over_b'] else '')
                  + ('' if r['packs_matched_rota'] else ' (TASK AT lines did not match the rota: packs partial)'))
    if len(res) > 1:
        print('over the runs: event tilt max ' + M.spread_text([r['event_tilt_max_deg'] for r in res])
              + ' deg, mission / B ' + M.spread_text([r['mission_over_b'] for r in res])
              + ', tilt RMS ' + M.spread_text([r.get('tilt_rms_deg') for r in res])
              + ' deg, p95 ' + M.spread_text([r.get('tilt_p95_deg') for r in res]) + ' deg')
    b = bars(res) if a.bars else None
    if b:
        print(bars_text(b, res))
    if a.json:
        with open(a.json, 'w') as fh:
            json.dump(res, fh, indent=1)
        if b:
            with open(os.path.splitext(a.json)[0] + '_bars.json', 'w') as fh:
                json.dump(b, fh, indent=1)
    return 0


if __name__ == '__main__':
    sys.exit(main())

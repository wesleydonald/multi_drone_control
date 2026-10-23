#!/usr/bin/env python3
"""tools/s_metrics.py -- score a scalability S twin run (card docs/experiments/2026-10-23_scalability.md).

    python3 tools/s_metrics.py R2401 R2402 [--json results/analysis/s_chain_2026-10-30.json]

Reads run.csv and events.csv, and the planner's tick log (logs/dissipative_planner/*/log.csv) and the trackers' tick
logs moved onto run.csv's clock (their sim_time minus the manifest's t_ready_sim) and cut to run.csv's rows:
  per event   every RELEASE, DETACH and ATTACH: the ring's tilt peak from its pre-event mean (release_bands'
              delta-theta: the tilt vector's largest departure from its mean over the PRE_S before the event, up to
              the next event), the settled tilt (its mean over the last SETTLE_S before the next event or LAND), and
              how long the planner took to change fleet size after it (the first tick logged at the new n)
  per stage   every fleet size between events: the ring's position error against its reference (RMS) and its tilt
              peak (the tilt vector's magnitude) from STAGE_SKIP_S after the event (the first stage: from TOP_PAD_S
              after the reference's top), and the planned tensions (each slot's mean)
              and the carriers' throttle (the drones in the planner's slot map there): the worst carrier's p95 and
              the fraction of carrier samples at the THROTTLE_MAX cap
  N1 / N4     the card's steady hovers: N1 the first stage (2 s after the top to the first event or LAND; all of
              s8_hover), N4 the fleet back at its final size (2 s after the last rejoin's REJOIN_BLEND_OVER, or with
              no blend 2 s after the planner's resize to it, to LAND; n4_resize keeps the resize's window);
              position RMSE, tilt peak and the planner's n
  lift        the lift the N1 window leaves out: from the ring's reference first LIFT_DZ above where it started to
              2 s after the top; tilt peak (the tilt vector's magnitude), position RMSE and worst error
  planner     solve_ms p50 / p95 / max (wall), failed solves (any status but 0 and the held-back unconverged 20),
              res_eq p95, res_ms p95 where logged; the published ticks (published 'solved') whose logged res_eq is
              over RES_EQ_MAX (the card's 'no published tick with res_eq > 1e-2': at 1 iteration the publish gate
              never reads a warm tick's residual) and those with none logged
  rejoins     the REJOIN START / WELD / ABORT / REFUSED lines of the launch log in order, each with its drone (and
              the abort's reason): which rejoin welded and which timed out
  late ticks  the fraction of planner ticks more than 1.5 x their period after the last in sim time (and the worst
              tracker's): a run above LATE_VOID is void (a starved planner flies late plans)
  dropped     the planner dropped the ring (DROP AND LAND in the launch log)
"""
import argparse
import glob
import json
import os
import re
import sys

import numpy as np
import pandas as pd

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import release_bands as RB  # noqa: E402
import run_logs  # noqa: E402

PRE_S = RB.PRE_S          # s before an event that set the tilt it is measured from
SETTLE_S = 5.0            # s before the next event (or LAND) over which the settled tilt is averaged
STAGE_SKIP_S = 5.0        # s after a stage's event before its position error counts
TOP_PAD_S = 2.0           # N1 / N4: s after the reference's top (or the resize to the final size) before they count
LATE_FACTOR = 1.5         # a tick more than this many periods after the last is late
LATE_VOID = 0.01          # more late planner (or tracker) ticks than this voids the run
LIFT_DZ = 0.01            # m the ring's reference rises before the lift window opens
RES_EQ_MAX = 1e-2         # planner_solver.RES_EQ_MAX: the publish gate's residual bar
THROTTLE_MAX = 0.8        # the s* configs' throttle_max (the carriers' cap)
CHAIN_EVENTS = ('RELEASE', 'DETACH', 'ATTACH')
STATUS_UNCONVERGED = 20   # planner_solver: a held-back solve, not a failure


def late_fraction(t, period=None):
    """Fraction of tick-to-tick steps longer than LATE_FACTOR periods (the median step unless given)."""
    dt = np.diff(np.asarray(t, float))
    if dt.size == 0:
        return float('nan')
    p = float(np.median(dt)) if period is None else float(period)
    return float(np.mean(dt > LATE_FACTOR * p + 1e-6))


def _pct(v, q):
    v = np.asarray([x for x in v if np.isfinite(x)], float)
    return float(np.percentile(v, q)) if v.size else float('nan')


def top_time(t, z_ref, t_land):
    """When the ring's reference reached its top: within 1 mm of its highest before LAND (the b22 bands' hover window,
    tools/n2_bands.py); t[0] if it never rose."""
    m = (t < t_land) & np.isfinite(z_ref)
    if not m.any():
        return float(t[0])
    return float(t[m][np.argmax(z_ref[m] >= np.max(z_ref[m]) - 1e-3)])


def lift_window(t, z_ref, t_top):
    """(start, end) of the lift: the ring's reference first LIFT_DZ above its first value, to TOP_PAD_S after the
    top; the start is t[0] if it never rose before the top."""
    fin = np.isfinite(z_ref)
    if not fin.any():
        return float(t[0]), t_top + TOP_PAD_S
    up = np.nonzero(fin & (z_ref > z_ref[fin][0] + LIFT_DZ) & (t <= t_top))[0]
    return (float(t[up[0]]) if up.size else float(t[0])), t_top + TOP_PAD_S


def slot_drones(v):
    """The drones (0-based) of one planner row's slot2drone ('3 0 2 1'), else None."""
    if isinstance(v, str) and v.strip():
        return [int(x) for x in v.split()]
    if isinstance(v, (int, float, np.integer, np.floating)) and np.isfinite(v):
        return [int(v)]
    return None


def carrier_throttle(run, t, lo, hi, drones):
    """The carriers' throttle over [lo, hi): the worst carrier's p95 and the fraction of their samples at the cap."""
    m = (t >= lo) & (t < hi)
    cols = [f'd{i}_thr' for i in drones if f'd{i}_thr' in run]
    if not m.any() or not cols:
        return None, None
    V = run[cols].to_numpy(float)[m]
    p95 = [_pct(V[:, j], 95) for j in range(V.shape[1])]
    fin = V[np.isfinite(V)]
    return (max(p95) if p95 else None), (float(np.mean(fin >= THROTTLE_MAX - 1e-3)) if fin.size else None)


def rejoin_lines(path):
    """The launch log's REJOIN START / WELD / ABORT / REFUSED lines in order: {kind, drone (0-based), why}."""
    out = []
    if not os.path.isfile(path):
        return out
    pat = re.compile(r'REJOIN (START|WELD|ABORT|REFUSED) drone \d+ \(/drone_(\d+)\)(?: \(([^)]*)\))?')
    with open(path, errors='replace') as fh:
        for line in fh:
            m = pat.search(line)
            if m:
                out.append({'kind': m.group(1), 'drone': int(m.group(2)), 'why': m.group(3)})
    return out


def node_log(path, t_ready, lo, hi):
    """A node's log.csv on run.csv's clock and cut to [lo, hi], run.csv's first and last row: the node stamps /clock,
    which run.csv and events.csv count from t_ready_sim (as tools/render_video.py reads it), and it logs on from
    before the run clock starts to the shutdown after run.csv's last row."""
    df = pd.read_csv(path)
    df['sim_time'] = df['sim_time'] - t_ready
    return df[(df['sim_time'] >= lo) & (df['sim_time'] <= hi)].reset_index(drop=True)


def _steady(t, err, mag, lo, hi, n=None):
    """Position RMSE and tilt peak over [lo, hi) (the card's N1 / N4)."""
    m = (t >= lo) & (t < hi)
    fin = m & np.all(np.isfinite(err), axis=1)
    return {'from': lo, 'to': hi, 'n': n,
            'pos_rmse_m': float(np.sqrt(np.mean(np.sum(err[fin] ** 2, axis=1)))) if fin.sum() > 5 else None,
            'tilt_peak_deg': float(mag[m].max()) if m.any() else None}


def score(run_dir):
    """The S metrics of one run directory (results/<date>/R####_*)."""
    logs = os.path.join(run_dir, 'logs')
    run = pd.read_csv(os.path.join(logs, 'run.csv'))
    ev = pd.read_csv(os.path.join(logs, 'events.csv'))
    with open(os.path.join(run_dir, 'manifest.json')) as fh:
        clock = (float(json.load(fh)['t_ready_sim']), float(run['t'].min()), float(run['t'].max()))
    pdirs = run_logs.node_dirs(run_dir, 'dissipative_planner') + run_logs.node_dirs(run_dir, 'mpc_planner')
    plan = node_log(os.path.join(pdirs[-1], 'log.csv'), *clock) if pdirs else None
    qcols = ['payload_qw', 'payload_qx', 'payload_qy', 'payload_qz']
    ok = run.dropna(subset=qcols)
    t = ok['t'].to_numpy(float)
    th = RB.tilt_vector(ok[qcols].to_numpy(float))
    mag = np.linalg.norm(th, axis=1)
    err = (ok[['payload_x', 'payload_y', 'payload_z']].to_numpy(float)
           - ok[['payload_ref_x', 'payload_ref_y', 'payload_ref_z']].to_numpy(float))
    events = [(float(r.sim_time), str(r.event), r.arg) for r in ev.itertuples()]
    t_land = next((te for te, kind, _ in events if kind in ('LAND', 'PLANNER_LAND')), float(t[-1]))
    t_top = top_time(t, ok['payload_ref_z'].to_numpy(float), t_land)
    chain = [(te, kind, arg) for te, kind, arg in events if kind in CHAIN_EVENTS]
    out = {'run': os.path.basename(run_dir.rstrip('/')), 'events': [], 'stages': []}
    pt = plan['sim_time'].to_numpy(float) if plan is not None else np.array([])
    pn = plan['n'].to_numpy(int) if plan is not None else np.array([], int)
    bounds = [te for te, _, _ in chain] + [t_land]
    for k, (te, kind, arg) in enumerate(chain):
        nxt = bounds[k + 1]
        pre = (t >= te - PRE_S) & (t < te)
        win = (t >= te) & (t < nxt)
        e = {'t': te, 'event': kind, 'drone': None if pd.isna(arg) else int(float(arg)), 'peak_deg': None,
             'abs_peak_deg': None, 'settled_deg': None, 'resize_s': None, 'n_after': None}
        if pre.any() and win.any():
            dev = np.linalg.norm(th[win] - th[pre].mean(0), axis=1)
            e['peak_deg'], e['abs_peak_deg'] = float(dev.max()), float(mag[win].max())
            tail = win & (t >= nxt - SETTLE_S)
            if tail.any():
                e['settled_deg'] = float(mag[tail].mean())
        if pt.size:
            before = pn[pt < te]
            n0 = int(before[-1]) if before.size else None
            after = np.nonzero((pt >= te) & (pt < nxt) & (pn != n0))[0]
            if n0 is not None and after.size:
                e['resize_s'], e['n_after'] = float(pt[after[0]] - te), int(pn[after[0]])
        out['events'].append(e)
    t_run = run['t'].to_numpy(float)
    for a, b in zip([None] + bounds[:-1], bounds):
        lo = t_top + TOP_PAD_S if a is None else a + STAGE_SKIP_S
        st = dict(_steady(t, err, mag, lo, b), event=a, tensions=None, thr_p95=None, thr_cap_frac=None)
        if plan is not None:
            pm = (pt >= lo) & (pt < b) & (plan['phase'].to_numpy() == 'planner')
            if pm.any():
                n = int(np.median(pn[pm]))
                st['n'] = n
                st['tensions'] = [float(pd.to_numeric(plan[f't{i}'][pm], errors='coerce').mean()) for i in range(n)
                                  if f't{i}' in plan]
                maps = [slot_drones(v) for v in plan['slot2drone'][pm]] if 'slot2drone' in plan else []
                maps = [m for m in maps if m and len(m) == n]
                carriers = (pd.Series([tuple(m) for m in maps]).mode()[0] if maps else range(n))
                st['thr_p95'], st['thr_cap_frac'] = carrier_throttle(run, t_run, lo, b, carriers)
        out['stages'].append(st)
    lo, hi = lift_window(t, ok['payload_ref_z'].to_numpy(float), t_top)
    m = (t >= lo) & (t < hi)
    fin = m & np.all(np.isfinite(err), axis=1)
    out['lift'] = dict(_steady(t, err, mag, lo, hi),
                       pos_err_max_m=float(np.linalg.norm(err[fin], axis=1).max()) if fin.any() else None)
    out['n1'] = dict(out['stages'][0])
    out['n4'] = out['n4_resize'] = None
    if chain and plan is not None:
        live = (pt < t_land) & (plan['phase'].to_numpy() == 'planner')
        last = chain[-1][0]
        n_end = int(pn[live][-1]) if live.any() else None
        before = pn[pt < last]
        at = np.nonzero((pt >= last) & live & (pn == n_end))[0]
        t0 = float(pt[at[0]]) if at.size and before.size and int(before[-1]) != n_end else last
        out['n4_resize'] = _steady(t, err, mag, t0 + TOP_PAD_S, t_land, n_end)
        # card revision (Wesley 23 Oct, before the claim runs): N4 opens after the last rejoin's blend, whose fold-in
        # N3 and the rejoin card score; the window from the resize is reported beside it
        blends = [te for te, kind, _ in events if kind == 'REJOIN_BLEND_OVER' and last <= te < t_land]
        out['n4'] = (_steady(t, err, mag, blends[-1] + TOP_PAD_S, t_land, n_end) if blends
                     else out['n4_resize'])
    if plan is not None:
        status = pd.to_numeric(plan['solve_status'], errors='coerce')
        solved = plan[status.notna()]
        ms = pd.to_numeric(solved['solve_ms'], errors='coerce').to_numpy(float)
        st = status[status.notna()].to_numpy(int)
        out['planner'] = {'ticks': int(len(plan)), 'solve_ms_p50': _pct(ms, 50), 'solve_ms_p95': _pct(ms, 95),
                          'solve_ms_max': float(np.nanmax(ms)) if ms.size else float('nan'),
                          'failed': int(np.sum((st != 0) & (st != STATUS_UNCONVERGED))),
                          'unconverged': int(np.sum(st == STATUS_UNCONVERGED)),
                          'res_eq_p95': _pct(pd.to_numeric(plan.get('res_eq'), errors='coerce'), 95)
                          if 'res_eq' in plan else float('nan'),
                          'res_ms_p95': _pct(pd.to_numeric(plan.get('res_ms'), errors='coerce'), 95)
                          if 'res_ms' in plan else float('nan'),
                          'late': late_fraction(pt, 0.1)}
        pub = (plan['published'].astype(str).to_numpy() == 'solved' if 'published' in plan
               else np.zeros(len(plan), bool))
        req = (pd.to_numeric(plan['res_eq'], errors='coerce').to_numpy(float) if 'res_eq' in plan
               else np.full(len(plan), np.nan))
        out['planner'].update(published=int(pub.sum()), published_res_over=int(np.sum(pub & (req > RES_EQ_MAX))),
                              published_res_missing=int(np.sum(pub & ~np.isfinite(req))),
                              published_res_eq_max=float(np.nanmax(req[pub])) if np.isfinite(req[pub]).any()
                              else float('nan'))
    trackers = [late_fraction(node_log(os.path.join(d, 'log.csv'), *clock)['sim_time'])
                for d in run_logs.node_dirs(run_dir, 'tracker') if os.path.isfile(os.path.join(d, 'log.csv'))]
    out['tracker_late_max'] = max(trackers) if trackers else float('nan')
    launch = os.path.join(logs, 'launch.log')
    out['dropped'] = bool(os.path.isfile(launch) and 'DROP AND LAND' in open(launch, errors='replace').read())
    out['rejoins'] = rejoin_lines(launch)
    late = [v for v in (out.get('planner', {}).get('late', float('nan')), out['tracker_late_max']) if np.isfinite(v)]
    out['void'] = bool(late and max(late) > LATE_VOID)
    return out


def locate(tag):
    if os.path.isdir(tag):
        return tag
    hits = glob.glob(os.path.join(RB.RESULTS, '2*', tag + '_*'))
    if len(hits) != 1:
        sys.exit(f'{tag}: {len(hits)} run directories under {RB.RESULTS}')
    return hits[0]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('runs', nargs='+', help='R#### or run directories')
    ap.add_argument('--json', help='write every run\'s metrics here (e.g. results/analysis/s_chain_<date>.json)')
    a = ap.parse_args()
    rows = [score(locate(r)) for r in a.runs]
    f = lambda v, nd=1: '' if v is None or not np.isfinite(v) else f'{v:.{nd}f}'
    for r in rows:
        p = r.get('planner', {})
        print(f"{r['run']}: {'VOID ' if r['void'] else ''}{'DROPPED ' if r['dropped'] else ''}"
              f"solve p50/p95 {f(p.get('solve_ms_p50'))}/{f(p.get('solve_ms_p95'))} ms, failed {p.get('failed')}, "
              f"late {100 * p.get('late', float('nan')):.2f} % (trackers {100 * r['tracker_late_max']:.2f} %), "
              f"published ticks with res_eq > {RES_EQ_MAX:g}: {p.get('published_res_over')} of {p.get('published')} "
              f"({p.get('published_res_missing')} without one)")
        li = r['lift']
        print(f"   LIFT {li['from']:.1f}-{li['to']:.1f} s: tilt peak {f(li['tilt_peak_deg'], 2)} deg, position RMSE "
              f"{f(1e3 * li['pos_rmse_m'] if li['pos_rmse_m'] else None)} mm, worst "
              f"{f(1e3 * li['pos_err_max_m'] if li['pos_err_max_m'] else None)} mm")
        for e in r['events']:
            print(f"   {e['event']:7s} {e['drone'] if e['drone'] is not None else '':>2} @ {e['t']:6.1f}: peak "
                  f"{f(e['peak_deg'], 2)} deg, settled {f(e['settled_deg'], 2)} deg, resize "
                  f"{f(e['resize_s'], 2)} s -> n {e['n_after']}")
        for s in r['stages']:
            cap = None if s['thr_cap_frac'] is None else 100 * s['thr_cap_frac']
            print(f"   stage n={s['n']}: position RMSE {f(1e3 * s['pos_rmse_m'] if s['pos_rmse_m'] else None)} mm, "
                  f"tilt peak {f(s['tilt_peak_deg'], 2)} deg, tensions {[round(v, 2) for v in s['tensions'] or []]}, "
                  f"carriers' throttle p95 {f(s['thr_p95'], 3)} (at the cap {f(cap)} %)")
        for j in r['rejoins']:
            print(f"   REJOIN {j['kind']} /drone_{j['drone']}" + (f" ({j['why']})" if j['why'] else ''))
        for k in ('n1', 'n4', 'n4_resize'):
            s = r.get(k)
            if s is not None:
                rmse = f(1e3 * s['pos_rmse_m'] if s['pos_rmse_m'] else None)
                print(f"   {k.upper()} n={s['n']} {s['from']:.1f}-{s['to']:.1f} s: position RMSE {rmse} mm, "
                      f"tilt peak {f(s['tilt_peak_deg'], 2)} deg")
    if a.json:
        os.makedirs(os.path.dirname(os.path.abspath(a.json)), exist_ok=True)
        with open(a.json, 'w') as fh:
            json.dump(rows, fh, indent=1)
        print(f'wrote {a.json}')
    return 0


if __name__ == '__main__':
    sys.exit(main())

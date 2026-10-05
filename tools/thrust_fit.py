#!/usr/bin/env python3
"""tools/thrust_fit.py -- the rig thrust law from free hovers, and the rod force it implies in tethered flights.

    python3 tools/thrust_fit.py tools/test/fixtures/rig_0930/ladder_points.yaml
    python3 tools/thrust_fit.py --tethered results/rig/2026-09-30/lift_f7_logs [more folders] [--spec SPEC] [--out FILE]

Fit: steady free hovers with known hung masses give u = a_i + b M + c (V - 23.5) by least squares (a per-drone
offset, b throttle per kg, c per volt; M = drone as flown + hung mass, V = pack). The spec is YAML (a list under
`points:`) or CSV with columns path, drone, hung_kg (optional drone_kg); paths are relative to the spec file. Steady
hover is thrust_check.py's rule: the reference plateau from 3 s after reaching it, the drone within 5 cm of its median
height there; u and V are the medians over it.

Tethered: per launch (the load planner log and the four tracker logs started with it), each drone's mass supported
by its thrust is M = (u_real - a_i - c (V - 23.5)) / b, its vertical thrust M g R33 (tilt from the mocap quaternion),
and its vertical pull on the rod M g R33 - m_drone g. Their sum over the ring's weight is the carried fraction.
u_real is the thr_out column when logged, else u2 + offset(V) for a tracker on the affine map (thrust_offset in its
params.json), else u2. Drone vertical acceleration is ignored, so read medians over a hold, not single samples.
The law defaults to RIG-0930-ladder4; --spec refits it."""
import argparse
import json
import math
import os
import sys

import numpy as np
import pandas as pd

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import run_logs  # noqa: E402

G = 9.81
V_REF = 23.5
DRONE_KG = 0.55
RING_KG = 0.86
LAW_0930 = {'a': [0.181, 0.185, 0.187, 0.187], 'b': 0.507, 'c': -0.0219}
BANDS = (0.20, 0.30, 0.40, 0.50, 0.60, 0.80)


def read_log(path):
    try:
        return pd.read_csv(path)
    except (pd.errors.EmptyDataError, FileNotFoundError):
        return pd.DataFrame()


def steady_hover(d):
    """(median throttle, median pack V, samples) over the steady part of the free-hover plateau."""
    if d.empty:
        return math.nan, math.nan, 0
    d = d[d.u2 > 0.1]
    if len(d) < 50:
        return math.nan, math.nan, 0
    plateau = d[d.ref_z > d.ref_z.max() - 0.01]
    plateau = plateau[plateau.sim_time > plateau.sim_time.iloc[0] + 3.0]
    if len(plateau) < 25:
        return math.nan, math.nan, 0
    z_med = plateau.pose_z.median()
    steady = plateau[(plateau.pose_z - z_med).abs() < 0.05]
    if z_med < 0.3 or len(steady) < 25:
        return math.nan, math.nan, len(steady)
    return float(steady.u2.median()), float(steady.battery_v.median()), len(steady)


def read_spec(spec):
    base = os.path.dirname(os.path.abspath(spec))
    if spec.endswith(('.yaml', '.yml')):
        import yaml
        with open(spec) as f:
            rows = yaml.safe_load(f)['points']
    else:
        rows = pd.read_csv(spec).to_dict('records')
    pts = []
    for r in rows:
        p = r['path'] if os.path.isabs(r['path']) else os.path.join(base, r['path'])
        if os.path.isdir(p):
            p = os.path.join(p, 'log.csv')
        pts.append({'path': p, 'drone': int(r['drone']), 'hung_kg': float(r['hung_kg']),
                    'drone_kg': float(r.get('drone_kg', DRONE_KG))})
    return pts


def fit(points):
    """Joint least squares; returns the law plus a row per point (u, V, M, residual)."""
    rows = []
    for p in points:
        u, v, n = steady_hover(read_log(p['path']))
        if n and u == u:
            rows.append(dict(p, u=u, v=v, m=p['drone_kg'] + p['hung_kg'], n=n))
        else:
            print(f'skipped (no steady hover): {p["path"]}', file=sys.stderr)
    ids = sorted({r['drone'] for r in rows})
    A = np.array([[1.0 if r['drone'] == i else 0.0 for i in ids] + [r['m'], r['v'] - V_REF] for r in rows])
    y = np.array([r['u'] for r in rows])
    x, *_ = np.linalg.lstsq(A, y, rcond=None)
    res = y - A @ x
    for r, e in zip(rows, res):
        r['res'] = float(e)
    a = [math.nan] * (max(ids) + 1)
    for k, i in enumerate(ids):
        a[i] = float(x[k])
    return {'a': a, 'b': float(x[-2]), 'c': float(x[-1]), 'rms': float(np.sqrt(np.mean(res ** 2))),
            'max': float(np.abs(res).max()), 'n': len(rows), 'rows': rows}


def supported_kg(u, v, drone, law):
    return (u - law['a'][drone] - law['c'] * (v - V_REF)) / law['b']


def real_throttle(d, params):
    if 'thr_out' in d:
        return d.thr_out.to_numpy(float)
    off = params.get('thrust_offset') or 0.0
    if off > 0:
        slope = params.get('thrust_offset_v_slope', 0.022)
        v_ref = params.get('thrust_v_ref', V_REF)
        return d.u2.to_numpy(float) + off + slope * (v_ref - pack_v(d))
    return d.u2.to_numpy(float)


def pack_v(d):
    # telemetry reads 0 V when the link drops: hold the drone's median instead
    v = d.battery_v.to_numpy(float).copy()
    bad = ~(v > 15.0)
    if bad.all():
        return np.full_like(v, V_REF)
    v[bad] = np.median(v[~bad])
    return v


def _stamp(tag):
    s = tag.rsplit('_', 2)
    return pd.Timestamp(f'{s[-2]}T{s[-1]}').timestamp()


def launches(folder):
    """[(planner log, {drone: tracker log})], trackers matched to the planner started within 30 s."""
    planners = run_logs.node_csvs(folder, 'mpc_planner')
    trackers = run_logs.node_csvs(folder, 'tracker')
    out = []
    for p in planners:
        ts = _stamp(os.path.basename(os.path.dirname(p)))
        group = {}
        for t in trackers:
            tag = os.path.basename(os.path.dirname(t))
            if abs(_stamp(tag) - ts) < 30.0:
                group[run_logs.drone_of(t)] = t
        out.append((p, group))
    return out


def _accel(t, v, win):
    a = np.gradient(np.asarray(v, float), np.asarray(t, float))
    return pd.Series(a).rolling(win, center=True, min_periods=1).mean().to_numpy()


def rod_forces(planner_path, trackers, law, drone_kg=DRONE_KG):
    """Frame on drone 0's tracker clock: ring z, each drone's vertical rod pull (N), carried fraction."""
    pl = read_log(planner_path)
    logs = {i: read_log(p) for i, p in trackers.items()}
    logs = {i: d for i, d in logs.items() if len(d) > 10}
    if pl.empty or not logs:
        return pd.DataFrame(), math.nan
    rest = float(pl[pl.phase == 'creep'].load_z.median()) if 'phase' in pl else float(pl.load_z.iloc[:10].median())
    t0 = max(pl.sim_time.iloc[0], *(d.sim_time.iloc[0] for d in logs.values()))
    t1 = min(pl.sim_time.iloc[-1], *(d.sim_time.iloc[-1] for d in logs.values()))
    ref = logs[min(logs)]
    t = ref.sim_time.to_numpy(float)
    t = t[(t >= t0) & (t <= t1)]
    out = pd.DataFrame({'t': t - t[0] if len(t) else t,
                        'ring_z': np.interp(t, pl.sim_time, pl.load_z)})
    total = np.zeros(len(t))
    total_dyn = np.zeros(len(t))
    for i, d in sorted(logs.items()):
        with open(os.path.join(os.path.dirname(trackers[i]), 'params.json')) as f:
            params = json.load(f)
        q = d[['pose_qw', 'pose_qx', 'pose_qy', 'pose_qz']].to_numpy(float)
        q /= np.linalg.norm(q, axis=1, keepdims=True)
        r33 = 1.0 - 2.0 * (q[:, 1] ** 2 + q[:, 2] ** 2)
        m = supported_kg(real_throttle(d, params), pack_v(d), i, law)
        fz = m * G * r33 - drone_kg * G
        out[f'f{i}'] = np.interp(t, d.sim_time, fz)
        total += out[f'f{i}'].to_numpy()
        if 'x0_vz' in d:
            total_dyn += np.interp(t, d.sim_time, fz - drone_kg * _accel(d.sim_time, d.x0_vz, 10))
    out['frac'] = total / (RING_KG * G)
    if 'load_vz' in pl:
        a_ring = np.interp(t, pl.sim_time, _accel(pl.sim_time, pl.load_vz, 3))
        out['frac_dyn'] = total_dyn / (RING_KG * (G + a_ring))
    return out, rest


def summarize(df, rest):
    air = df[df.ring_z > rest + 0.05]
    s = {'rest': rest, 'n': len(air), 'secs': float(len(air) * np.median(np.diff(df.t))) if len(df) > 1 else 0.0}
    if len(air):
        s['med'] = float(air.frac.median())
        s['q1'], s['q3'] = (float(v) for v in air.frac.quantile([0.25, 0.75]))
        s['z_med'] = float(air.ring_z.median())
        s['dyn'] = float(air.frac_dyn.median()) if 'frac_dyn' in air else math.nan
        s['per_drone'] = [float(air[c].median()) for c in df.columns if c[0] == 'f' and c[1:].isdigit()]
        edges = (rest + 0.05,) + tuple(b for b in BANDS if b > rest + 0.05) + (math.inf,)
        s['bands'] = []
        for lo, hi in zip(edges[:-1], edges[1:]):
            band = air[(air.ring_z >= lo) & (air.ring_z < hi)]
            if len(band):
                dyn = float(band.frac_dyn.median()) if 'frac_dyn' in band else math.nan
                s['bands'].append((lo, hi, len(band), float(band.frac.median()), dyn))
    return s


def law_table(law):
    lines = ['| drone | a_i | b (per kg) | c (per V) |', '|---|---|---|---|']
    for i, a in enumerate(law['a']):
        lines.append(f'| {i} | {a:.4f} | {law["b"]:.4f} | {law["c"]:.4f} |')
    return lines


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('spec', nargs='?', help='hover points (YAML or CSV): fit the law')
    ap.add_argument('--spec', dest='spec_opt', help='with --tethered: refit the law from this spec first')
    ap.add_argument('--tethered', nargs='+', metavar='FOLDER', help='run folders of tethered flights')
    ap.add_argument('--drone-kg', type=float, default=DRONE_KG)
    ap.add_argument('--out', help='also write the tethered tables (markdown) here')
    a = ap.parse_args(argv)

    spec = a.spec or a.spec_opt
    law = dict(LAW_0930)
    if spec:
        law = fit(read_spec(spec))
        print(f'{"drone":>5} {"hung kg":>8} {"M kg":>6} {"V":>5} {"u":>7} {"n":>4} {"resid":>8}  file')
        for r in law['rows']:
            print(f'{r["drone"]:>5} {r["hung_kg"]:>8.3f} {r["m"]:>6.3f} {r["v"]:>5.2f} {r["u"]:>7.4f} {r["n"]:>4} '
                  f'{r["res"]:>+8.4f}  {os.path.relpath(r["path"])}')
        print('u = a_i + b M + c (V - 23.5): a = ' + '/'.join(f'{v:.4f}' for v in law['a'])
              + f', b {law["b"]:.4f}, c {law["c"]:.4f}; rms {law["rms"]:.4f}, max {law["max"]:.4f}, n {law["n"]}')
    if not a.tethered:
        if not spec:
            ap.error('give a spec to fit, or --tethered folders')
        return 0

    md = ['| flight | launch | airborne s | ring z med (m) | carried med | IQR | accel-corrected med | '
          'rod pull per drone med (N) |', '|---|---|---|---|---|---|---|---|']
    bands = ['| flight | launch | ring z band (m) | samples | carried med | accel-corrected med |',
             '|---|---|---|---|---|---|']
    for folder in a.tethered:
        name = os.path.basename(os.path.normpath(folder)).replace('_logs', '')
        for p, trackers in launches(folder):
            df, rest = rod_forces(p, trackers, law, a.drone_kg)
            if df.empty:
                continue
            s = summarize(df, rest)
            if not s['n']:
                continue
            launch = os.path.basename(os.path.dirname(p)).split('_')[-1]
            md.append(f'| {name} | {launch} | {s["secs"]:.1f} | {s["z_med"]:.3f} | {s["med"]:.3f} | '
                      f'{s["q1"]:.3f}-{s["q3"]:.3f} | {s["dyn"]:.3f} | ' + ' / '.join(f'{v:.2f}' for v in s['per_drone']) + ' |')
            for lo, hi, n, med, dyn in s['bands']:
                hi_s = f'{hi:.2f}' if hi != math.inf else 'up'
                bands.append(f'| {name} | {launch} | {lo:.2f}-{hi_s} | {n} | {med:.3f} | {dyn:.3f} |')
    text = '\n'.join(['law: a = ' + '/'.join(f'{v:.4f}' for v in law['a'])
                      + f', b {law["b"]:.4f}, c {law["c"]:.4f}', ''] + md + [''] + bands)
    print(text)
    if a.out:
        with open(a.out, 'w') as f:
            f.write(text + '\n')
    return 0


if __name__ == '__main__':
    sys.exit(main())

#!/usr/bin/env python3
"""tools/dist_replay.py -- offline replay of the option-A disturbance split (card
2026-10-03_offset_free_mpc, critic must-fix 1). No runs.

Per sample, the force balance of every drone and of the ring is solved by least squares for the
n rod tensions t_i, one force d_D shared by every drone and one force d_L on the ring:

    drone i:  t_i s_i + d_D = m a_i - f_i - m g          (3 equations each)
    ring:    -sum t_i s_i + d_L = m_L a_L - m_L g        (3 equations)

f_i is the thrust each tracker believes it delivered: its own gain above its own offset,
m * kT * (thr_out - thr_off), along the drone's mocap body z. s_i runs from the drone's rod pivot
(4 cm below the centre) to its plate, taken radially from the ring centre at the attach radius
(the logs carry no ring yaw; the ring hangs within a few degrees of level). Accelerations come from
the logged positions, smoothed over 0.5 s.

    python3 tools/dist_replay.py R0856                         # windows from the WRENCH events
    python3 tools/dist_replay.py R0841 --before-land 15
    python3 tools/dist_replay.py results/rig/2026-10-01/h5_z100_rot120_logs --before-land 15
"""
import argparse
import csv
import glob
import json
import os
import re
import sys

import numpy as np
import pandas as pd

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))   # tools/ (this file is in tools/old/)
REPO = os.path.dirname(HERE)
sys.path.insert(0, HERE)
import run_logs  # noqa: E402

G = np.array([0.0, 0.0, -9.81])
RING_MASS = 0.86
DRONE_MASS = 0.55
ATTACH_RADIUS = 0.225
PIVOT_BELOW = 0.04
SMOOTH_S = 0.5


def resolve(spec):
    """(log root, run dir or None) of a run id or a folder."""
    if os.path.isdir(spec):
        return spec, (spec if os.path.exists(os.path.join(spec, 'logs', 'events.csv')) else None)
    d = glob.glob(os.path.join(REPO, 'results', '*', f'{spec}_*'))
    if not d:
        raise SystemExit(f'no run {spec}')
    return d[0], d[0]


def tracker_params(csv_path):
    p = os.path.join(os.path.dirname(csv_path), 'params.json')
    try:
        return json.load(open(p))
    except (OSError, ValueError):
        return {}


def load(root):
    """Drone logs resampled onto drone 0's clock: {i: DataFrame}, the clock, the ring xyz."""
    logs = run_logs.trackers(root)
    if not logs:
        raise SystemExit(f'no tracker logs under {root}')
    dfs = {i: pd.read_csv(f) for i, f in sorted(logs.items())}
    t = np.unique(dfs[min(dfs)]['sim_time'].values)
    out = {}
    for i, df in dfs.items():
        df = df.drop_duplicates('sim_time').sort_values('sim_time')
        out[i] = pd.DataFrame({c: np.interp(t, df['sim_time'].values, df[c].values)
                               for c in ('pose_x', 'pose_y', 'pose_z', 'pose_qw', 'pose_qx', 'pose_qy',
                                         'pose_qz', 'thr_out', 'thr_off', 'payload_x', 'payload_y',
                                         'payload_z') if c in df})
    ring = out[min(out)][['payload_x', 'payload_y', 'payload_z']].values
    kts = {i: float(tracker_params(logs[i]).get('thrust_ratio', 35.2)) for i in logs}
    return out, t, ring, kts


def accel(t, p):
    """Second derivative of positions (rows = samples), each column smoothed over SMOOTH_S."""
    dt = float(np.median(np.diff(t)))
    k = max(1, int(round(SMOOTH_S / dt)))
    ker = np.ones(k) / k
    ps = np.column_stack([np.convolve(p[:, j], ker, mode='same') for j in range(p.shape[1])])
    v = np.gradient(ps, t, axis=0)
    vs = np.column_stack([np.convolve(v[:, j], ker, mode='same') for j in range(v.shape[1])])
    return np.gradient(vs, t, axis=0)


def body_z(q):
    w, x, y, z = q
    return np.array([2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)])


def ring_attitude(run_dir, t):
    """Ring rotation matrices on clock t from run.csv's payload quaternion (sim runs), or None."""
    p = os.path.join(run_dir or '', 'logs', 'run.csv')
    if not run_dir or not os.path.exists(p):
        return None
    df = pd.read_csv(p).drop_duplicates('t').sort_values('t')
    q = np.column_stack([np.interp(t, df.t.values, df[c].values)
                         for c in ('payload_qw', 'payload_qx', 'payload_qy', 'payload_qz')])
    q /= np.linalg.norm(q, axis=1, keepdims=True)
    w, x, y, z = q.T
    return np.stack([np.stack([1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)], -1),
                     np.stack([2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)], -1),
                     np.stack([2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)], -1)], 1)


def solve_window(drones, t, ring, kts, attach_z, t0, t1, R_ring=None, rod=None):
    """rod = (airframe mass, rod mass, rod centre of mass from the ring end in m), or None for the
    massless-rod balance with the whole 0.55 kg on the drone. A massive rod pinned at both ends
    hands its weight to the ends: the part across the rod by the lever (lam at the drone end), the
    part along it with the tension (t_i is the tension at the drone end)."""
    """Mean tensions, d_D, d_L, condition number and rod lengths over [t0, t1)."""
    sel = (t >= t0) & (t < t1)
    ids = sorted(drones)
    n = len(ids)
    a_ring = accel(t, ring)
    a_dr = {i: accel(t, drones[i][['pose_x', 'pose_y', 'pose_z']].values) for i in ids}
    m_d, m_r, lam = (DRONE_MASS, 0.0, 0.0) if rod is None else (rod[0], rod[1], rod[2] / 0.55)
    sols, conds, rods = [], [], []
    for k in np.where(sel)[0]:
        A = np.zeros((3 * n + 3, n + 6))
        b = np.zeros(3 * n + 3)
        rc = ring[k] + np.array([0.0, 0.0, attach_z])
        for j, i in enumerate(ids):
            d = drones[i].iloc[k]
            q = (d.pose_qw, d.pose_qx, d.pose_qy, d.pose_qz)
            bz = body_z(q)
            pivot = np.array([d.pose_x, d.pose_y, d.pose_z]) - PIVOT_BELOW * bz
            if R_ring is None:
                u = pivot[:2] - ring[k][:2]
                u = np.array([u[0], u[1], 0.0]) / max(np.hypot(*u), 1e-6)
                plate = rc + ATTACH_RADIUS * u
            else:
                # the ring-frame plate nearest this drone, through the ring's true attitude
                u_b = R_ring[k].T @ (pivot - ring[k])
                az = np.arctan2(u_b[1], u_b[0])
                rho = np.array([ATTACH_RADIUS * np.cos(az), ATTACH_RADIUS * np.sin(az), attach_z])
                plate = ring[k] + R_ring[k] @ rho
            s = plate - pivot
            rods.append(np.linalg.norm(s))
            s = s / np.linalg.norm(s)
            f = DRONE_MASS * kts[i] * (d.thr_out - d.thr_off) * bz
            e = -s
            g_ax = m_r * (G @ e) * e
            A[3 * j:3 * j + 3, j] = s
            A[3 * j:3 * j + 3, n:n + 3] = np.eye(3)
            b[3 * j:3 * j + 3] = (m_d * a_dr[i][k] - f - m_d * G
                                  - lam * (m_r * G - g_ax))
            A[3 * n:, j] = -s
            b[3 * n:] -= lam * g_ax + (1.0 - lam) * m_r * G
        A[3 * n:, n + 3:] = np.eye(3)
        b[3 * n:] += RING_MASS * a_ring[k] - RING_MASS * G
        sols.append(np.linalg.lstsq(A, b, rcond=None)[0])
        conds.append(np.linalg.cond(A))
    sols = np.array(sols)
    return {'n_samples': int(sel.sum()), 'tension': sols[:, :n].mean(0), 'd_D': sols[:, n:n + 3].mean(0),
            'd_L': sols[:, n + 3:].mean(0), 'd_L_sd': sols[:, n + 3:].std(0), 'cond': float(np.median(conds)),
            'rod': float(np.mean(rods)), 'ring': ring[sel].mean(0)}


def land_time(run_dir, root):
    if run_dir:
        for e in csv.DictReader(open(os.path.join(run_dir, 'logs', 'events.csv'))):
            if e['event'] == 'LAND':
                return float(e['sim_time'])
    log = os.path.normpath(root)[:-len('_logs')] + '.log' if root.rstrip('/').endswith('_logs') else None
    if log and os.path.exists(log):
        m = re.search(r"\[(\d+\.\d+)\] \[(?:central_controller|fleet_manager)\]: Fleet command received: 'LAND'",
                      open(log).read())
        if m:
            return float(m.group(1))
    return None


def fmt(v):
    return '(' + ', '.join(f'{x:+.2f}' for x in v) + ')'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('spec')
    ap.add_argument('--before-land', type=float, default=None, help='window length before LAND, s')
    ap.add_argument('--attach-z', type=float, default=None)
    ap.add_argument('--rod', type=float, nargs=3, metavar=('AIRFRAME_KG', 'ROD_KG', 'COM_FROM_RING_M'),
                    help='massive rod: twin 0.475 0.075 0.147')
    ap.add_argument('--ring-attitude', action='store_true',
                    help='place the plates through the logged ring attitude (sim runs)')
    a = ap.parse_args()
    root, run_dir = resolve(a.spec)
    drones, t, ring, kts = load(root)
    attach_z = a.attach_z if a.attach_z is not None else (0.02 if run_dir else 0.0)
    windows = []
    if run_dir:
        wr = [float(e['sim_time']) for e in csv.DictReader(open(os.path.join(run_dir, 'logs', 'events.csv')))
              if e['event'] == 'WRENCH']
        if len(wr) >= 2 and a.before_land is None:
            windows = [('pre-push', wr[0] - 5, wr[0]), ('push', wr[1] - 5, wr[1])]
    if not windows:
        tl = land_time(run_dir, root)
        if tl is None:
            raise SystemExit('no LAND time found; give windows')
        L = a.before_land or 15.0
        windows = [('hold', tl - L, tl - 0.5)]
    res = []
    for name, t0, t1 in windows:
        r = solve_window(drones, t, ring, kts, attach_z, t0, t1,
                         ring_attitude(run_dir, t) if a.ring_attitude else None, a.rod)
        res.append((name, r))
        print(f'{a.spec} {name:8s} [{t0:.1f}, {t1:.1f}) n={len(drones)} samples={r["n_samples"]}  '
              f'd_L={fmt(r["d_L"])} N (sd {fmt(r["d_L_sd"])})  d_D={fmt(r["d_D"])} N  '
              f't={np.round(r["tension"], 2).tolist()} N  cond={r["cond"]:.1f}  rod={r["rod"]:.3f} m  '
              f'ring z={r["ring"][2]:.3f}')
    if len(res) == 2:
        dL = res[1][1]['d_L'] - res[0][1]['d_L']
        dD = res[1][1]['d_D'] - res[0][1]['d_D']
        print(f'{a.spec} step: d_L {fmt(dL)} N, d_D {fmt(dD)} N')


if __name__ == '__main__':
    main()

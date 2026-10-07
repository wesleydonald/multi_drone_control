#!/usr/bin/env python3
"""tools/detach_tensions.py -- the ACTUAL cable pulls through a twin detach (soft-detach card, 7 Oct).

    python3 tools/detach_tensions.py R1118 R1119 ...

Each drone's pull from its cable, per tracker log row: m (a - g) - T R e_z, with a from the mocap pose
(central differences, 0.1 s mean), T the twin's own thrust law (rig_thrust: throttle, pack voltage,
the drone's sim_thrust_offset) and m the 0.55 kg hovering mass. The planner's tensions are its plan,
not the plant, so the card's bars read these instead. Check: in hover the four vertical pulls add up
to the ring's weight (printed as `sum Fz`).

Per run: the leaver's pull before the detach and at release; the survivors' peak pull; ring tilt and
survivor climb during the unload; ring tilt peak and height jump after the release.
"""
import csv
import glob
import json
import os
import sys

import numpy as np
import pandas as pd

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, 'src', 'simulation_communication'))
from simulation_communication.rig_thrust import rig_thrust  # noqa: E402

M, G = 0.55, 9.81


def _R(q):
    w, x, y, z = q / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def pulls(T, offset):
    """(t, F (n,3) N): the cable's pull on the drone, world frame."""
    T = T.drop_duplicates('sim_time').reset_index(drop=True)
    t = T.sim_time.to_numpy()
    p = T[['pose_x', 'pose_y', 'pose_z']].to_numpy()
    k = 5                                                     # 0.1 s at 50 Hz
    sm = lambda a: np.stack([np.convolve(a[:, j], np.ones(k) / k, 'same') for j in range(a.shape[1])], 1)
    v = sm(np.gradient(p, t, axis=0))
    a = sm(np.gradient(v, t, axis=0))
    F = np.zeros_like(p)
    for i, r in T.iterrows():
        thrust = rig_thrust(r.thr_out, r.battery_v if np.isfinite(r.battery_v) else 23.8, offset)
        R = _R(np.array([r.pose_qw, r.pose_qx, r.pose_qy, r.pose_qz]))
        F[i] = M * (a[i] - np.array([0.0, 0.0, -G])) - thrust * R[:, 2]
    return t, F


def run_dir(rid):
    return glob.glob(os.path.join(REPO, 'results', '*', f'{rid}_*'))[0]


def analyse(rid):
    d = run_dir(rid)
    man = json.load(open(os.path.join(d, 'manifest.json')))
    la = man.get('launch_args', {})
    if not isinstance(la, dict):
        la = dict(a.split(':=', 1) for a in la if ':=' in a)
    offs = [float(v) for v in str(la.get('sim_thrust_offset', '0.185')).split(',')]
    ev = list(csv.DictReader(open(os.path.join(d, 'logs', 'events.csv'))))
    det = [e for e in ev if e['event'] in ('DETACH', 'RELEASE')]
    if not det:
        return {'run': rid, 'note': 'no DETACH or RELEASE'}
    unannounced = det[0]['event'] == 'RELEASE'      # the joint let go at the event itself
    L = pd.read_csv(glob.glob(os.path.join(d, 'logs', '*planner', '*', 'log.csv'))[0])
    run = pd.read_csv(os.path.join(d, 'logs', 'run.csv')).drop_duplicates('t')
    # events.csv and run.csv count from the runner's start, the node logs on the sim clock:
    # align them on the ring's lift-off (3 cm over its rest height)
    lift = lambda t, z: float(t[z > np.median(z[:20]) + 0.03].iloc[0])
    off = lift(L.sim_time, L.load_z) - lift(run.t, run.payload_z)
    run = run.assign(t=run.t + off)
    t_cmd, leaver = float(det[0]['sim_time']) + off, int(det[0]['arg'])
    # the release is the resize: the planner's slot list loses the leaver
    gone = L[(L.sim_time >= t_cmd) & ~L.slot2drone.astype(str).str.split().apply(lambda v: str(leaver) in v)]
    t_rel = float(gone.sim_time.iloc[0]) if len(gone) else None
    n0 = len(str(L.slot2drone.iloc[(L.sim_time - (t_cmd - 0.5)).abs().argmin()]).split())
    F, Z = {}, {}
    for i in range(n0):
        T = pd.read_csv(sorted(glob.glob(os.path.join(d, 'logs', 'tracker', f'drone{i}_*', 'log.csv')))[-1])
        t, f = pulls(T, offs[i] if i < len(offs) else offs[-1])
        F[i] = (t, f)
        Z[i] = (T.drop_duplicates('sim_time').sim_time.to_numpy(), T.drop_duplicates('sim_time').pose_z.to_numpy())
    win = lambda t, a, b: (t >= a) & (t < b)
    mag = lambda i, a, b: np.linalg.norm(F[i][1][win(F[i][0], a, b)], axis=1)
    pre = {i: float(np.mean(mag(i, t_cmd - 3, t_cmd))) for i in F}
    sum_fz = float(sum(np.mean(F[i][1][win(F[i][0], t_cmd - 3, t_cmd), 2]) for i in F))
    t_end = (t_rel if t_rel is not None else t_cmd + 8) + 10
    surv = [i for i in F if i != leaver]
    out = {'run': rid, 'name': os.path.basename(d).split('_sim_gz_')[-1], 'leaver': leaver,
           'sum Fz (N, ring 8.4)': round(-sum_fz, 2),
           'pre pull (N)': {i: round(pre[i], 2) for i in F},
           'unload s': round(t_rel - t_cmd, 1) if t_rel is not None else None,
           'survivor peak (N)': {i: round(float(mag(i, t_cmd, t_end).max()), 2) for i in surv}}
    tilt = lambda a, b: float(run.payload_tilt_deg[win(run.t, a, b)].max())
    if unannounced:
        out['detected after (s)'] = round(t_rel - t_cmd, 2) if t_rel is not None else 'never'
        sizes = L.slot2drone.astype(str).str.split().apply(len)
        full = L.sim_time[sizes == n0]
        pre = (L.sim_time > full.iloc[0]) & (L.sim_time < t_cmd) if len(full) else (L.sim_time < 0)
        out['false detection before release'] = bool((sizes[pre] < n0).any())
        out['tilt peak release..+10 s (deg)'] = round(tilt(t_cmd, t_cmd + 10), 1)
        z0 = float(run.payload_z[win(run.t, t_cmd - 1, t_cmd)].mean())
        dz = run.payload_z[win(run.t, t_cmd, t_cmd + 2)] - z0
        out['ring z jump 2 s (cm)'] = round(100 * float(dz[dz.abs().idxmax()]), 1)
        out['survivor peak (N)'] = {i: round(float(mag(i, t_cmd, t_cmd + 10).max()), 2) for i in surv}
        return out
    if t_rel is None:
        out['released'] = False
        out['tilt max detach..+18 s (deg)'] = round(tilt(t_cmd, t_cmd + 18), 1)
    else:
        out['leaver at release (N)'] = round(float(np.mean(mag(leaver, t_rel - 0.3, t_rel))), 2)
        out['leaver at release / pre'] = round(out['leaver at release (N)'] / max(pre[leaver], 1e-6), 2)
        if t_rel - t_cmd > 0.3:
            out['tilt during unload (deg)'] = round(tilt(t_cmd, t_rel), 1)
            climb = max(float(np.max(np.interp(np.arange(t_cmd, t_rel, 0.05), *Z[i]) -
                                     np.mean(Z[i][1][win(Z[i][0], t_cmd - 2, t_cmd)]))) for i in surv)
            out['survivor climb during unload (cm)'] = round(100 * climb, 1)
        out['tilt peak release..+10 s (deg)'] = round(tilt(t_rel, t_rel + 10), 1)
        z0 = float(run.payload_z[win(run.t, t_rel - 1, t_rel)].mean())
        dz = run.payload_z[win(run.t, t_rel, t_rel + 2)] - z0
        out['ring z jump 2 s (cm)'] = round(100 * float(dz[dz.abs().idxmax()]), 1)
    met = os.path.join(d, 'metrics.json')
    if os.path.exists(met):
        mj = json.load(open(met))
        out['criteria'] = mj.get('passed', mj.get('pass', ''))
    return out


if __name__ == '__main__':
    for rid in sys.argv[1:]:
        print(json.dumps(analyse(rid)))

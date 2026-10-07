"""tools/twin_compare.py -- hover bobbing, circle lag and wobble, rods and landing lean for twin or rig runs.

    python3 tools/twin_compare.py R1074 R1104 r006      (R#### = twin run id, rNNN = 2026-10-07 rig run)
"""
import glob
import os
import sys

import numpy as np
import pandas as pd

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
R = os.path.join(REPO, 'results')


def paths(tag):
    d = f'{R}/rig/2026-10-07/{tag}_logs/logs' if tag.startswith('r') else glob.glob(f'{R}/*/{tag}_*/logs')[0]
    return glob.glob(d + '/*planner/*/log.csv')[0], sorted(glob.glob(d + '/tracker/drone0_*/log.csv'))[-1]


def hover(L, win=(2.0, 12.0)):
    top = float(L.sim_time[L.lift_progress >= L.lift_progress.max() - 1e-4].iloc[0])
    end = (float(L.sim_time[L.traj_t > 0].iloc[0]) if (L.traj_t > 0).any()
           else float(L.sim_time[L.land != 0].iloc[0]))
    m = (L.sim_time >= top + win[0]) & (L.sim_time < min(end, top + win[1]))
    if m.sum() < 20:
        return None
    z = L.load_z[m].values
    return dict(span=round(float(L.sim_time[m].iloc[-1] - L.sim_time[m].iloc[0]), 1), z_sd=round(100 * z.std(), 1),
                xy_sd=round(100 * np.hypot(L.load_x[m].std(), L.load_y[m].std()), 1))


def circle(L, T):
    if not (L.traj_t > 0).any():
        return None
    ts = float(L.sim_time[L.traj_t > 0].iloc[0])
    tl = float(L.sim_time[L.land != 0].iloc[0])
    T = T[(T.sim_time >= ts + 4) & (T.sim_time < tl)]
    rx, ry, px, py = T.payload_ref_x.values, T.payload_ref_y.values, T.payload_x.values, T.payload_y.values
    e = np.stack([px - rx, py - ry], 1)
    out = dict(err=round(100 * np.hypot(e[:, 0], e[:, 1]).mean(), 2), err_max=round(100 * np.hypot(e[:, 0], e[:, 1]).max(), 1),
               z_sd=round(100 * T.payload_z.std(), 1))
    c = np.array([rx.mean(), ry.mean()])
    rad = np.stack([rx - c[0], ry - c[1]], 1)
    nr = np.linalg.norm(rad, axis=1)
    if nr.min() > 0.2:                                  # a circle-like path: radial / along-track split
        ur = rad / nr[:, None]
        ang = np.unwrap(np.arctan2(rad[:, 1], rad[:, 0]))
        ut = np.sign(ang[-1] - ang[0]) * np.stack([-ur[:, 1], ur[:, 0]], 1)
        er, et = (e * ur).sum(1), (e * ut).sum(1)
        out.update(lag_s=round(-et.mean() / 0.125, 2), wobble_sd=round(100 * np.hypot(er.std(), et.std()), 1))
    m = (L.sim_time >= ts) & (L.sim_time < tl)
    out['rods'] = round(float(L[[c for c in L.columns if c.startswith('elev')]][m].mean().mean()), 1)
    out['tilt'] = round(float(L.tilt_deg[m].mean()), 1)
    return out


def lean(tag):
    if tag.startswith('r'):
        return None
    sys.path.insert(0, os.path.join(REPO, 'tools'))
    from land_metrics import land
    lv = land(tag)
    return (lv.get('outcome'), round(lv.get('tilt_deg', float('nan')), 1))


if __name__ == '__main__':
    for tag in sys.argv[1:]:
        lp, tp = paths(tag)
        L = pd.read_csv(lp)
        T = pd.read_csv(tp).drop_duplicates('sim_time')
        print({'run': tag, 'hover': hover(L), 'path': circle(L, T), 'land': lean(tag)})

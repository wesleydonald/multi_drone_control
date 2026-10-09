#!/usr/bin/env python3
"""tools/departed_metrics.py -- every drone's touchdown, scored for the departed ground idle card
(docs/experiments/2026-10-09_departed_ground_idle.md).

    python3 tools/departed_metrics.py R1252 R1260 ...

Per drone, from run.csv (runner time):
  rest          its resting height: the median z of the first 2 s (before ARM spin-up)
  at_rest_t     when it is first back within 0.03 m of rest after it flew (None: never)
  idle_t        when its throttle first reads 0 while armed after it flew (the tracker's ground idle)
  disarm_dz     its z at its disarm edge minus rest (bar: <= 0.03 m)
  disarm_t      its first armed -> disarmed edge (the plant's command, not a log line)
  idle_armed_s  disarm_t - idle_t: seconds armed at throttle 0 on the floor (card 2026-10-09_departed_disarm_alone)
  before_landed True when that edge comes before the runner's LANDED (/fleet/landed): disarmed alone, not by the fleet
  press_max_s   the longest stretch within 0.3 m of its rest with its reference >= 0.1 m below it, armed and not
                idle (pressed into the floor, R0997; bar: <= 1 s)
  tilt_idle     its peak tilt from the idle to the disarm (bar: <= 10 deg)
  stand_s       once it has come down through rest + 0.8 m after its highest point, the longest stretch it is still
                (|vz| < 0.05) between rest + 0.3 and rest + 0.8 m: standing on its rod, whatever its reference (the
                old stall freeze held its reference there; bar: < 2 s)
"""
import glob
import json
import os
import sys

import numpy as np
import pandas as pd

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REST_TOL = 0.03
PRESS_M = 0.10


def run_dir(rid):
    if os.path.isdir(rid):
        return rid
    return glob.glob(os.path.join(REPO, 'results', '*', f'{rid}_*'))[0]


def _longest(mask, t):
    best, start = 0.0, None
    for i, m in enumerate(mask):
        if m and start is None:
            start = t[i]
        if (not m or i == len(mask) - 1) and start is not None:
            end = t[i] if m else t[i - 1]
            best = max(best, end - start)
            start = None
    return round(float(best), 2)


def drone(run, i):
    t = run.t.to_numpy()
    z = run[f'd{i}_z'].to_numpy()
    ref = run[f'd{i}_ref_z'].to_numpy() if f'd{i}_ref_z' in run else np.full_like(z, np.nan)
    thr = run[f'd{i}_thr'].to_numpy()
    armed = run[f'd{i}_armed'].to_numpy() > 0.5
    vz = run[f'd{i}_vz'].to_numpy() if f'd{i}_vz' in run else np.gradient(z, t)
    tilt = run[f'd{i}_tilt_deg'].to_numpy()
    early = t <= t[0] + 2.0
    rest = float(np.nanmedian(z[early]))
    flew = np.where(z > rest + 0.2)[0]
    out = {'rest': round(rest, 3)}
    if not len(flew):
        return out
    after = np.arange(len(t)) > flew[0]
    back = np.where(after & (np.abs(z - rest) <= REST_TOL))[0]
    out['at_rest_t'] = round(float(t[back[0]]), 2) if len(back) else None
    idle = np.where(after & armed & (thr <= 1e-3))[0]
    out['idle_t'] = round(float(t[idle[0]]), 2) if len(idle) else None
    edge = np.where(after[1:] & armed[:-1] & ~armed[1:])[0]
    out['disarm_dz'] = round(float(z[edge[0] + 1] - rest), 3) if len(edge) else None
    out['disarm_t'] = round(float(t[edge[0] + 1]), 2) if len(edge) else None
    if len(edge) and len(idle):
        out['idle_armed_s'] = round(float(t[edge[0] + 1] - t[idle[0]]), 2)
    asked_down = np.isfinite(ref) & (z - ref >= PRESS_M)
    press = after & armed & (thr > 1e-3) & asked_down & (z <= rest + 0.3)
    out['press_max_s'] = _longest(press, t)
    if len(idle):
        end = edge[0] + 1 if len(edge) else len(t) - 1
        out['tilt_idle'] = round(float(np.nanmax(tilt[idle[0]:end + 1])), 1)
    top = int(np.nanargmax(np.where(after, z, -np.inf)))
    below = np.where((np.arange(len(t)) > top) & (z < rest + 0.8))[0]
    landing = np.arange(len(t)) >= (below[0] if len(below) else len(t))
    stand = landing & armed & (z >= rest + 0.3) & (z <= rest + 0.8) & (np.abs(vz) < 0.05)
    out['stand_s'] = _longest(stand, t)
    return out


def landed_t(d):
    f = os.path.join(d, 'logs', 'events.csv')
    if not os.path.exists(f):
        return None
    ev = pd.read_csv(f)
    hit = ev[ev.event == 'LANDED']
    return float(hit.sim_time.iloc[0]) if len(hit) else None


def analyse(rid):
    d = run_dir(rid)
    run = pd.read_csv(os.path.join(d, 'logs', 'run.csv')).drop_duplicates('t').reset_index(drop=True)
    n = len([c for c in run.columns if c.startswith('d') and c.endswith('_armed')])
    drones = {i: drone(run, i) for i in range(n)}
    lt = landed_t(d)
    for v in drones.values():
        if v.get('disarm_t') is not None:
            v['before_landed'] = lt is None or v['disarm_t'] < lt
    return {'run': os.path.basename(d).split('_')[0], 'landed_t': lt, 'drones': drones}


def main(argv):
    if not argv:
        print(__doc__)
        return 1
    for rid in argv:
        print(json.dumps(analyse(rid)))
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))

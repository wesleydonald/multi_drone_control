#!/usr/bin/env python3
"""tools/attach_metrics.py -- twin attach runs scored for the newcomer (the M1 attach gates and the newcomer
lifecycle card, docs/experiments/2026-10-09_newcomer_lifecycle.md).

    python3 tools/attach_metrics.py R1238 R1241 ...

Per run, from run.csv and events.csv (runner time):
  weld           WELD time, or none
  ring           payload z range and tilt peak from the latest of TAKEOFF, LIFTED and MAGNET ON to LAND;
                 tilt peak LAND..LANDED
  separation     smallest distance from the newcomer's body to any carrier body while the newcomer is above
                 0.2 m, and when
  approach       the newcomer's approach phases (climb/transit/descend) from the planner log, in runner time
  disarm         the newcomer's height when its arming first drops after TAKEOFF
  at LANDED      the newcomer's z and arming at /fleet/landed; 'down' = it was below 0.15 m before LANDED
  step-out       after LAND: the newcomer's largest horizontal distance from the ring centre, and its z
                 range until that distance first reaches 1.15 m
The newcomer is the column block after the carriers (d<num_drones>); node log stamps are epoch wall time,
anchored on the fleet manager receiving the runner's first ARM and mapped through run.csv's wall column.
"""
import glob
import json
import os
import re
import sys

import numpy as np
import pandas as pd

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
STAMP = re.compile(r'\[(\d{10}\.\d+)\]')
PHASE = re.compile(r'approach: drone (\d+) (climb|transit|descend)\b')
DOWN_Z = 0.15
AIR_Z = 0.2
CLEAR_R = 1.15


def run_dir(rid):
    if os.path.isdir(rid):
        return rid
    return glob.glob(os.path.join(REPO, 'results', '*', f'{rid}_*'))[0]


def event_t(ev, name, after=-1.0):
    t = ev.sim_time[(ev.event == name) & (ev.sim_time > after)]
    return float(t.iloc[0]) if len(t) else None


def phases(log, ev, run):
    """[(runner t, phase)] of the newcomer's approach, from the planner's log. Log stamps are epoch wall
    time and the sim runs slower than real time, so map them through run.csv's wall column."""
    t_arm = event_t(ev, 'ARM')
    arm = next((float(STAMP.search(ln).group(1)) for ln in log
                if "Fleet command received: 'ARM'" in ln and STAMP.search(ln)), None)
    if t_arm is None or arm is None:
        return []
    epoch0 = arm - float(np.interp(t_arm, run.t, run.wall))
    out = []
    for ln in log:
        m, s = PHASE.search(ln), STAMP.search(ln)
        if m and s:
            out.append((round(float(np.interp(float(s.group(1)) - epoch0, run.wall, run.t)), 2), m.group(2)))
    return out


def analyse(rid):
    d = run_dir(rid)
    run = pd.read_csv(os.path.join(d, 'logs', 'run.csv')).drop_duplicates('t').reset_index(drop=True)
    ev = pd.read_csv(os.path.join(d, 'logs', 'events.csv'))
    log = open(os.path.join(d, 'logs', 'launch.log'), errors='replace').read().splitlines()
    n = int(json.load(open(os.path.join(d, 'manifest.json')))['launch_args'].get('num_drones', 3))
    k = f'd{n}_'
    if k + 'z' not in run:
        raise SystemExit(f'{rid}: no newcomer columns ({k}*) in run.csv')
    t = run.t.to_numpy()
    new = run[[k + 'x', k + 'y', k + 'z']].to_numpy()
    armed = run[k + 'armed'].to_numpy()
    ring = run[['payload_x', 'payload_y', 'payload_z']].to_numpy()
    tilt = run.payload_tilt_deg.to_numpy()

    t_to, t_on = event_t(ev, 'TAKEOFF'), event_t(ev, 'MAGNET')
    t_land, t_landed, t_weld = event_t(ev, 'LAND'), event_t(ev, 'LANDED'), event_t(ev, 'WELD')
    end = t[-1]
    r = {'run': os.path.basename(d).split('_')[0], 'weld': t_weld}

    t_lifted = event_t(ev, 'LIFTED')
    a = max(x for x in (t_on, t_to, t_lifted, 0.0) if x is not None)
    b = t_land if t_land is not None else end
    w = (t >= a) & (t <= b)
    r['ring_z'] = (round(float(ring[w, 2].min()), 3), round(float(ring[w, 2].max()), 3)) if w.any() else None
    r['tilt_peak'] = round(float(tilt[w].max()), 1) if w.any() else None
    if t_land is not None:
        wl = (t >= t_land) & (t <= (t_landed if t_landed is not None else end))
        r['tilt_peak_land'] = round(float(tilt[wl].max()), 1) if wl.any() else None

    carriers = [run[[f'd{i}_x', f'd{i}_y', f'd{i}_z']].to_numpy() for i in range(n)]
    sep = np.min([np.linalg.norm(new - c, axis=1) for c in carriers], axis=0)
    air = new[:, 2] > AIR_Z
    if air.any():
        i = int(np.argmin(np.where(air, sep, np.inf)))
        r['separation'] = (round(float(sep[i]), 3), round(float(t[i]), 1))
    else:
        r['separation'] = None
    r['approach'] = phases(log, ev, run)

    after = t > (t_to if t_to is not None else 0.0)
    drop = np.where(after[1:] & (armed[:-1] > 0.5) & (armed[1:] < 0.5))[0]
    r['disarm_z'] = round(float(new[drop[0] + 1, 2]), 3) if len(drop) else None

    if t_landed is not None:
        i = int(np.searchsorted(t, t_landed))
        i = min(i, len(t) - 1)
        below = np.where((t >= (t_land or 0.0)) & (new[:, 2] < DOWN_Z))[0]
        r['at_landed'] = {'z': round(float(new[i, 2]), 3), 'armed': bool(armed[i] > 0.5),
                          'down_before': bool(len(below) and t[below[0]] <= t_landed)}
    if t_land is not None:
        wl = t >= t_land
        horiz = np.linalg.norm(new[:, :2] - ring[:, :2], axis=1)
        r['step_max_r'] = round(float(horiz[wl].max()), 3)
        reach = np.where(wl & (horiz >= CLEAR_R))[0]
        if len(reach):
            ws = wl & (t <= t[reach[0]])
            r['step_z'] = (round(float(new[ws, 2].min()), 3), round(float(new[ws, 2].max()), 3))
        else:
            r['step_z'] = None
    return r


def main(argv):
    if not argv:
        print(__doc__)
        return 1
    for rid in argv:
        print(json.dumps(analyse(rid)))
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))

#!/usr/bin/env python3
"""tools/drop_metrics.py -- drop-and-land twin runs scored against the card's bars
(docs/experiments/2026-10-08_drop_and_land.md, revision v2).

    python3 tools/drop_metrics.py R1175 R1178 ...

Per run, from the trigger (the planner's "DROP AND LAND" line) to the end of the runner window:
  trigger        what asked for the drop (planner detection / gap / LAND, a tracker's tilt
                 request, the runner) and when, in s after the last RELEASE or DROP event
  faults         ENVELOPE FAULT / FLEET ABORT / EMERGENCY lines (must be none)
  disarm_air     drones whose arming dropped above 0.2 m (tracker logs: the motors' u to 0 while
                 high), from the fleet's disarm line otherwise
  tilt, climb    per drone: max tilt after the trigger; peak z minus z at the trigger
  speed          peak horizontal speed after the trigger
  closest pair   smallest drone-drone distance from the trigger to touchdown
  down           s from the trigger until every drone is below 0.15 m; /fleet/landed and the
                 fleet's disarm afterwards
  spot           each drone's landing distance from the ring's position at the trigger
Log stamps are epoch wall time; run.csv carries the runner's own wall and runner time (anchored on the
fleet manager's ARM line), and the node logs are aligned to run.csv on the ring's lift-off (as
detach_tensions.py does).
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


def run_dir(rid):
    return glob.glob(os.path.join(REPO, 'results', '*', f'{rid}_*'))[0]


def _tilt(qw, qx, qy, qz):
    return np.degrees(np.arccos(np.clip(1.0 - 2.0 * (qx * qx + qy * qy), -1.0, 1.0)))


def analyse(rid):
    d = run_dir(rid)
    run = pd.read_csv(os.path.join(d, 'logs', 'run.csv')).drop_duplicates('t')
    ev = pd.read_csv(os.path.join(d, 'logs', 'events.csv'))
    log = open(os.path.join(d, 'logs', 'launch.log'), errors='replace').read().splitlines()
    T = {}
    for f in sorted(glob.glob(os.path.join(d, 'logs', 'tracker', 'drone*_*', 'log.csv'))):
        i = int(re.search(r'drone(\d+)_', f).group(1))
        T[i] = pd.read_csv(f).drop_duplicates('sim_time')
    # node sim time = runner t + off (lift-off alignment on the ring height)
    lift = lambda t, z: float(t[z > np.median(z[:20]) + 0.03].iloc[0])  # noqa: E731
    t0 = T[min(T)]
    off = lift(t0.sim_time, t0.payload_z) - lift(run.t, run.payload_z)
    # run.csv's wall counts from the runner's start, log stamps are epoch: anchor on the fleet
    # manager receiving the runner's first ARM (milliseconds after it is sent)
    t_arm = float(ev.sim_time[ev.event == 'ARM'].iloc[0])
    arm_stamp = next(float(STAMP.search(ln).group(1)) for ln in log
                     if "Fleet command received: 'ARM'" in ln and STAMP.search(ln))
    epoch0 = arm_stamp - float(np.interp(t_arm, run.t, run.wall))
    wall_to_sim = lambda w: float(np.interp(w - epoch0, run.wall, run.t)) + off  # noqa: E731

    def first(pat):
        for ln in log:
            if re.search(pat, ln):
                m = STAMP.search(ln)
                return (wall_to_sim(float(m.group(1))) if m else None), ln
        return None, None

    t_drop, drop_ln = first(r'DROP AND LAND')
    out = {'run': rid, 'name': os.path.basename(d).split('_sim_gz_')[-1]}
    faults = [ln.split(']: ', 1)[-1][:90] for ln in log
              if re.search(r'ENVELOPE FAULT|FLEET ABORT|EMERGENCY STOP|disarmed in flight', ln)]
    out['faults'] = faults[:3]
    out['fault count'] = len(faults)
    req = [ln.split(']: ', 1)[-1][:80] for ln in log if 'DROP REQUEST' in ln]
    out['tracker requests'] = len(req)
    trig_ev = ev[ev.event.isin(['RELEASE', 'DROP'])]
    t_ev = float(trig_ev.sim_time.iloc[-1]) + off if len(trig_ev) else None
    if t_drop is None:
        out['dropped'] = False
        return out
    out['dropped'] = True
    out['trigger'] = drop_ln.split('DROP AND LAND: ', 1)[-1].split(' - every magnet')[0][:90]
    out['after last event (s)'] = round(t_drop - t_ev, 2) if t_ev is not None else None
    t_end = float(run.t.max()) + off
    ring = run[(run.t + off) >= t_drop]
    ring0 = ring[['payload_x', 'payload_y']].iloc[0].to_numpy() if len(ring) else None
    tilt, climb, speed, spot, t_down = {}, {}, {}, {}, {}
    P = {}
    for i, x in T.items():
        w = x[(x.sim_time >= t_drop) & (x.sim_time <= t_end)]
        if len(w) < 2:
            continue
        P[i] = w
        tilt[i] = round(float(_tilt(w.pose_qw, w.pose_qx, w.pose_qy, w.pose_qz).max()), 1)
        climb[i] = round(float(w.pose_z.max() - w.pose_z.iloc[0]), 2)
        v = np.hypot(np.gradient(w.pose_x, w.sim_time), np.gradient(w.pose_y, w.sim_time))
        speed[i] = round(float(pd.Series(v).rolling(5, center=True).mean().max()), 2)
        dn = w.sim_time[w.pose_z < 0.15]
        t_down[i] = round(float(dn.iloc[0] - t_drop), 1) if len(dn) else None
        if ring0 is not None:
            spot[i] = round(float(np.hypot(*(w[['pose_x', 'pose_y']].iloc[-1].to_numpy() - ring0))), 2)
    pair = np.inf
    ids = sorted(P)
    tt = np.arange(t_drop, min(t_end, t_drop + 30.0), 0.05)
    pos = {i: np.c_[[np.interp(tt, P[i].sim_time, P[i][k]) for k in ('pose_x', 'pose_y', 'pose_z')]].T
           for i in ids}
    for a in range(len(ids)):
        for b in range(a + 1, len(ids)):
            pair = min(pair, float(np.min(np.linalg.norm(pos[ids[a]] - pos[ids[b]], axis=1))))
    out.update({'drone tilt max (deg)': tilt, 'climb (m)': climb, 'peak speed (m/s)': speed,
                'closest pair (m)': round(pair, 2), 'down after (s)': t_down,
                'landing spot from ring (m)': spot})
    t_landed, _ = first(r'announcing /fleet/landed')
    out['landed announced after (s)'] = round(t_landed - t_drop, 1) if t_landed else None
    out['LANDED event'] = bool((ev.event == 'LANDED').any())
    return out


if __name__ == '__main__':
    for rid in sys.argv[1:]:
        print(json.dumps(analyse(rid)))

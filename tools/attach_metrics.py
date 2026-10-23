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
  step-out       from its abort (the planner's 'out of this flight' line, else LAND): the newcomer's largest
                 horizontal distance from the ring centre, its z range until it first reaches 1.15 m, and
                 its magnet tip's closest approach to the ring's plate circle on the way (tip columns)
  clearance      newcomer body to every carrier body and to every carrier rod (carrier -> its plate),
                 MAGNET ON -> LANDED (metrics.partner_clearance's geometry; bars 0.30 / 0.15 m)
  floor          the newcomer's peak tilt while armed on the floor after its touchdown, and whether it
                 ever had a reference (N1: an idle floor newcomer must get none, R0532)
  gate_timeout   the planner's 'not down after' line (the landed gate gave up on a departed drone)
  on_refused     the planner's 'ATTACH refused' lines (an ON after an abort, or during LAND)
  fault          a newcomer fault (card newcomer_fault_scope): its disarm time and height (from the tip, tip z +
                 0.59 m, when its mocap was cut), the runner events FAULT_LAND (the manager landed the fleet),
                 PLANNER_LAND and FLEET_ABORT, and whether every carrier stayed armed until LANDED
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
from run_dir import manifest_azimuths_ccw  # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
STAMP = re.compile(r'\[(\d{10}\.\d+)\]')
PHASE = re.compile(r'approach: drone (\d+) (climb|transit|descend)\b')
OUT = re.compile(r'out of this flight \((\w+)\)')
RIM_R = 0.225
BODY_TIP = 0.59                   # the twin newcomer's body to magnet tip on the 0.55 m arm (else its world's: body_tip)
DOWN_Z = 0.15
AIR_Z = 0.2
CLEAR_R = 1.15


def body_tip(d):
    """The newcomer's body to magnet tip in the run's world (its arm less its pivot's height: 1.04 at 1 m), else
    BODY_TIP (no manifest.json, or no world in it)."""
    from check_geometry import newcomer_arm
    try:
        with open(os.path.join(d, 'manifest.json')) as f:
            w = str(json.load(f).get('world') or '')
    except (OSError, ValueError, AttributeError):
        w = ''
    path = w if os.path.isabs(w) else os.path.join(REPO, 'simulation_assets', w)
    arm = newcomer_arm(path) if w and os.path.exists(path) else None
    return arm[0] - arm[1] if arm else BODY_TIP


def run_dir(rid):
    if os.path.isdir(rid):
        return rid
    return glob.glob(os.path.join(REPO, 'results', '*', f'{rid}_*'))[0]


def event_t(ev, name, after=-1.0):
    t = ev.sim_time[(ev.event == name) & (ev.sim_time > after)]
    return float(t.iloc[0]) if len(t) else None


def log_times(log, ev, run, pattern):
    """[(runner t, first group)] of planner log lines matching pattern. Log stamps are epoch wall time and
    the sim runs slower than real time, so map them through run.csv's wall column."""
    t_arm = event_t(ev, 'ARM')
    arm = next((float(STAMP.search(ln).group(1)) for ln in log
                if "Fleet command received: 'ARM'" in ln and STAMP.search(ln)), None)
    if t_arm is None or arm is None:
        return []
    epoch0 = arm - float(np.interp(t_arm, run.t, run.wall))
    out = []
    for ln in log:
        m, s = pattern.search(ln), STAMP.search(ln)
        if m and s:
            out.append((round(float(np.interp(float(s.group(1)) - epoch0, run.wall, run.t)), 2), m.group(1)))
    return out


def _rot(q):
    w, x, y, z = q.T
    return np.stack([np.stack([1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)], -1),
                     np.stack([2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)], -1),
                     np.stack([2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)], -1)], 1)


def _seg_dist(p, a, b):
    ab = b - a
    u = np.clip(np.einsum('ij,ij->i', p - a, ab) / np.maximum(np.einsum('ij,ij->i', ab, ab), 1e-12), 0.0, 1.0)
    return np.linalg.norm(p - (a + u[:, None] * ab), axis=1)


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
    if t_land is None:
        t_land = event_t(ev, 'FAULT_LAND')          # no scheduled LAND: the manager's, on a newcomer fault
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
    if len(drop):
        i = drop[0] + 1
        z = new[i, 2]
        if not np.isfinite(z) and 'tip_z' in run:
            z = float(run.tip_z.iloc[i]) + body_tip(d)   # its mocap was cut: the hanging tip, at the disarm
        r['fault'] = {'disarm_t': round(float(t[i]), 2), 'disarm_z': round(float(z), 3) if np.isfinite(z) else None,
                      'fault_land': event_t(ev, 'FAULT_LAND'), 'planner_land': event_t(ev, 'PLANNER_LAND'),
                      'fleet_abort': event_t(ev, 'FLEET_ABORT')}
        if t_landed is not None:
            w = (t > t[i]) & (t < t_landed - 0.1)
            r['fault']['carriers_armed'] = bool(all((run[f'd{c}_armed'].to_numpy()[w] > 0.5).all()
                                                    for c in range(n)))

    if t_landed is not None:
        i = int(np.searchsorted(t, t_landed))
        i = min(i, len(t) - 1)
        below = np.where((t >= (t_land or 0.0)) & (new[:, 2] < DOWN_Z))[0]
        r['at_landed'] = {'z': round(float(new[i, 2]), 3), 'armed': bool(armed[i] > 0.5),
                          'down_before': bool(len(below) and t[below[0]] <= t_landed)}
    outs = log_times(log, ev, run, OUT)
    r['out'] = outs[0] if outs else None
    t_out = outs[0][0] if outs else t_land
    q = run[['payload_qw', 'payload_qx', 'payload_qy', 'payload_qz']].to_numpy()
    R = _rot(q)
    full = json.load(open(os.path.join(d, 'manifest.json')))
    man = full['launch_args']
    az = (manifest_azimuths_ccw(man.get('attach_azimuths_deg'), full) or [270.0, 30.0, 150.0])[:n]
    attach_z = float(man.get('attach_z') or 0.02)
    if t_out is not None:
        wl = t >= t_out
        horiz = np.linalg.norm(new[:, :2] - ring[:, :2], axis=1)
        r['step_max_r'] = round(float(horiz[wl].max()), 3)
        reach = np.where(wl & (horiz >= CLEAR_R))[0]
        if len(reach):
            ws = wl & (t <= t[reach[0]])
            if k + 'ref_z' in run:                                # ...or until its own descent begins
                rz = run[k + 'ref_z'].to_numpy()
                hold = rz[np.where(wl & np.isfinite(rz))[0][0]] if (wl & np.isfinite(rz)).any() else np.nan
                down = np.where(wl & (rz < hold - 0.01))[0]
                if len(down):
                    ws = ws & (t < t[down[0]])
            r['step_z'] = (round(float(new[ws, 2].min()), 3), round(float(new[ws, 2].max()), 3))
            if k + 'ref_z' in run:                                # height against its own reference
                dz = new[:, 2] - run[k + 'ref_z'].to_numpy()
                ok = ws & np.isfinite(dz)
                r['step_dz_ref'] = round(float(np.abs(dz[ok]).max()), 3) if ok.any() else None
            if 'tip_x' in run:
                tip = run[['tip_x', 'tip_y', 'tip_z']].to_numpy()
                rel = np.einsum('nji,nj->ni', R, tip - ring)          # ring frame
                rim = np.hypot(np.hypot(rel[:, 0], rel[:, 1]) - RIM_R, rel[:, 2] - attach_z)
                ok = ws & np.isfinite(rim)
                r['tip_rim_min'] = round(float(rim[ok].min()), 3) if ok.any() else None
                # parked (at the abort) vs on the way out (after the 0.5 s release hold): the move must not
                # bring the tip closer to the rim than the approach parked it
                at = np.where(ok)[0]
                moving = ok & (t >= t_out + 0.5)
                r['tip_rim_parked'] = round(float(rim[at[0]]), 3) if len(at) else None
                r['tip_rim_moving_min'] = round(float(rim[moving].min()), 3) if moving.any() else None
        else:
            r['step_z'] = None
    a = t_on if t_on is not None else t_to
    b = t_landed if t_landed is not None else end
    if a is not None:
        m = (t >= a) & (t <= b) & np.all(np.isfinite(new), 1)
        body = rod = np.inf
        for i in range(n):
            C = carriers[i]
            rho = np.array([RIM_R * np.cos(np.radians(az[i])), RIM_R * np.sin(np.radians(az[i])), attach_z])
            plate = ring + np.einsum('nij,j->ni', R, rho)
            ok = m & np.all(np.isfinite(C), 1)
            if ok.any():
                body = min(body, float(np.linalg.norm(new[ok] - C[ok], axis=1).min()))
                rod = min(rod, float(_seg_dist(new[ok], C[ok], plate[ok]).min()))
        r['clearance'] = {'body': round(body, 3), 'rod': round(rod, 3)}
    td = np.where((new[:, 2] < DOWN_Z) & (t > (t_out if t_out is not None else end)))[0]
    if len(td):
        fl = (t >= t[td[0]]) & (armed > 0.5) & (new[:, 2] < DOWN_Z)
        r['floor_tilt_max'] = round(float(run[k + 'tilt_deg'].to_numpy()[fl].max()), 1) if fl.any() else None
    r['ref_ever'] = bool(np.isfinite(run[k + 'ref_x'].to_numpy()).any()) if k + 'ref_x' in run else None
    r['gate_timeout'] = any('not down after' in ln for ln in log)
    r['on_refused'] = sum('ATTACH refused' in ln for ln in log)
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

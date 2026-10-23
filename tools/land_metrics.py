#!/usr/bin/env python3
"""tools/land_metrics.py -- one LAND, scored for the LAND matrix (Wesley 9 Oct: LAND lands every drone safely,
whatever it is doing).

    python3 tools/land_metrics.py R1311 R1312 ...

From run.csv and events.csv (runner time), from the first LAND on:
  land_t / landed_t   the first LAND and the runner's LANDED (None: never)
  abort               the launch log shows a /fleet/abort or an EMERGENCY STOP
  ring_tilt_max       the ring's peak tilt from LAND to the end of run.csv (deg)
  per drone           disarm_dz (z at its disarm edge minus its rest; None: still armed at the end),
                      tilt_max (its peak tilt from LAND to its disarm edge, deg), alone (disarmed before LANDED)
  armed_at_end        drones still armed at run.csv's last row
"""
import glob
import json
import os
import sys

import numpy as np
import pandas as pd

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def run_dir(rid):
    if os.path.isdir(rid):
        return rid
    return glob.glob(os.path.join(REPO, 'results', '*', f'{rid}_*'))[0]


def _event(ev, name):
    hit = ev[ev.event == name]
    return float(hit.sim_time.iloc[0]) if len(hit) else None


def _aborted(d):
    for f in glob.glob(os.path.join(d, '**', '*.log'), recursive=True):
        try:
            with open(f, errors='ignore') as fh:
                for line in fh:
                    if 'EMERGENCY STOP' in line or '/fleet/abort at' in line:
                        return True
        except OSError:
            pass
    return False


def analyse(rid):
    d = run_dir(rid)
    run = pd.read_csv(os.path.join(d, 'logs', 'run.csv')).drop_duplicates('t').reset_index(drop=True)
    ev = pd.read_csv(os.path.join(d, 'logs', 'events.csv'))
    t = run.t.to_numpy()
    land_t, landed_t = _event(ev, 'LAND'), _event(ev, 'LANDED')
    out = {'run': os.path.basename(d).split('_')[0], 'land_t': land_t, 'landed_t': landed_t,
           'abort': _aborted(d)}
    if land_t is None:
        return out
    after = t >= land_t
    out['land_to_landed_s'] = round(landed_t - land_t, 2) if landed_t is not None else None
    if 'payload_tilt_deg' in run:
        out['ring_tilt_max'] = round(float(np.nanmax(run.payload_tilt_deg.to_numpy()[after])), 1)
    n = len([c for c in run.columns if c.startswith('d') and c.endswith('_armed')])
    drones, armed_end = {}, []
    for i in range(n):
        z = run[f'd{i}_z'].to_numpy()
        armed = run[f'd{i}_armed'].to_numpy() > 0.5
        tilt = run[f'd{i}_tilt_deg'].to_numpy()
        rest = float(np.nanmedian(z[t <= t[0] + 2.0]))
        edge = np.where(after[1:] & armed[:-1] & ~armed[1:])[0]
        k = edge[0] + 1 if len(edge) else None
        end = k if k is not None else len(t) - 1
        drones[i] = {'disarm_dz': round(float(z[k] - rest), 3) if k is not None else None,
                     'tilt_max': round(float(np.nanmax(tilt[after][:max(1, end - int(np.argmax(after)) + 1)])), 1),
                     'alone': bool(k is not None and landed_t is not None and t[k] < landed_t)}
        if armed[-1]:
            armed_end.append(i)
    out['drones'] = drones
    out['armed_at_end'] = armed_end
    return out


def main(argv):
    if not argv:
        print(__doc__)
        return 1
    for rid in argv:
        print(json.dumps(analyse(rid)))
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))

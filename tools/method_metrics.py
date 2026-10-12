#!/usr/bin/env python3
"""tools/method_metrics.py -- one metric row per run for the overnight method comparison (card
2026-10-04_offset_free_innovation): the ring's error on each arm, from run.csv and events.csv.

    python3 tools/method_metrics.py R0841 R0856 R0864 R0832 ...

Arm from the config name: *_push -> push (last 5 s of the push against the 5 s before it);
*orbit* / *circle* -> first lap (27.2 s from the circle start, as tools/plot_dist_circle.py);
otherwise a hold (the 15 s before LAND). Each also gives the hold z error, tilt mean and
solve failures.
"""
import csv
import glob
import os
import sys

import numpy as np
import pandas as pd

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LAP_S = 27.2


def run_dir(rid):
    return glob.glob(os.path.join(REPO, 'results', '*', f'{rid}_*'))[0]


def metrics(rid):
    d = run_dir(rid)
    name = os.path.basename(d)
    ev = list(csv.DictReader(open(os.path.join(d, 'logs', 'events.csv'))))
    evd = {}
    for e in ev:
        evd.setdefault(e['event'], float(e['sim_time']))
    run = pd.read_csv(os.path.join(d, 'logs', 'run.csv')).drop_duplicates('t')
    out = dict(run=rid, config=name, aborted=any('ABORT' in e['event'] for e in ev),
               landed='LANDED' in evd)
    if 'LAND' not in evd:
        out['arm'] = 'void'
        return out
    land = evd['LAND']
    xy = lambda w: np.hypot(w.payload_x - w.payload_ref_x, w.payload_y - w.payload_ref_y)
    if '_push' in name:
        wr = [float(e['sim_time']) for e in ev if e['event'] == 'WRENCH']
        on, off = wr[0], wr[1]
        pre = run[(run.t >= on - 5) & (run.t < on)]
        pu = run[(run.t >= off - 5) & (run.t < off)]
        out.update(arm='push', err_cm=100 * float(np.hypot(pu.payload_x.mean() - pre.payload_ref_x.mean(),
                                                            pu.payload_y.mean() - pre.payload_ref_y.mean())),
                   z_err_cm=100 * float((pre.payload_z - pre.payload_ref_z).mean()),
                   tilt=float(pre.payload_tilt_deg.mean()))
    elif 'orbit' in name or 'circle' in name:
        logs = (glob.glob(os.path.join(d, 'logs', '*planner', '*', 'log.csv'))
                or glob.glob(os.path.join(d, 'logs', '*', 'load_planner_*', 'log.csv')))   # pre-rename
        L = pd.read_csv(logs[0])
        off = float(L.sim_time[L.load_z > 0.3].iloc[0]) - float(run.t[run.payload_z > 0.3].iloc[0])
        start = float(L.sim_time[L.traj_t > 0].iloc[0]) - off
        lap = run[(run.t >= start) & (run.t < min(land, start + LAP_S))]
        e = xy(lap)
        out.update(arm='circle', err_cm=100 * float(e.mean()), err_max_cm=100 * float(e.max()),
                   z_err_cm=100 * float((lap.payload_z - lap.payload_ref_z).mean()),
                   tilt=float(lap.payload_tilt_deg.mean()))
    else:
        w = run[(run.t >= land - 15) & (run.t < land)]
        out.update(arm='hold', err_cm=100 * float(np.hypot(*(w[['payload_x', 'payload_y']].mean().values
                                                              - w[['payload_ref_x', 'payload_ref_y']].mean().values))),
                   z_err_cm=100 * float((w.payload_z - w.payload_ref_z).mean()),
                   z_pp_cm=100 * float(np.ptp(w.payload_z)), tilt=float(w.payload_tilt_deg.mean()))
    logs = (glob.glob(os.path.join(d, 'logs', '*planner', '*', 'log.csv'))
            or glob.glob(os.path.join(d, 'logs', '*', 'load_planner_*', 'log.csv')))
    if logs:
        L = pd.read_csv(logs[0])
        st = pd.to_numeric(L.solve_status, errors='coerce')
        out['solve_fail'] = int(((L.phase == 'planner') & st.notna() & (st != 0)).sum())
    return out


def main():
    for r in sys.argv[1:]:
        m = metrics(r)
        print(' '.join(f'{k}={v:.2f}' if isinstance(v, float) else f'{k}={v}' for k, v in m.items()))


if __name__ == '__main__':
    main()

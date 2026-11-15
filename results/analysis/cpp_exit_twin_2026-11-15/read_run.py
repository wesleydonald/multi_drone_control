"""Read one C++ exit twin wiring check (R4527+): his phases, the exit's log lines, QUAD4's reference steps from the
clearance on, its height and its closest approach to the carriers. Wiring only: the twin runs at 0.13-0.16x real time.
    python3 read_run.py results/2026-11-15/R45xx_sim_gz_t15x_*   [--json out.json]"""
import argparse
import csv
import json
import math
import re
from pathlib import Path

import numpy as np


def events(run):
    rows = []
    with open(run / 'logs' / 'events.csv') as f:
        for r in csv.DictReader(f):
            try:
                t = float(r['sim_time'])
            except ValueError:
                continue
            rows.append((t, r['event'], r.get('arg', '')))
    return rows


def his_lines(run):
    pat = re.compile(r'C\+\+ exit|Planner phase|seedtraj|Exit prefix|COOPERATIVE|stationary|recovery', re.I)
    out = []
    for f in sorted((run / 'logs').glob('extra_launch_*.log')):
        for ln in f.read_text(errors='replace').splitlines():
            if pat.search(ln):
                out.append(re.sub(r'^\[([a-z_]+)-\d+\] \[([A-Z]+)\] \[[\d.]+\] \[[\w.]+\]: ', r'\1 \2: ', ln)[:400])
    return out


def table(run):
    with open(run / 'logs' / 'run.csv') as f:
        rd = csv.reader(f)
        head = next(rd)
        data = [r for r in rd]
    col = {k: i for i, k in enumerate(head)}

    def arr(k):
        return np.array([float(r[col[k]]) if r[col[k]] not in ('', 'nan') else np.nan for r in data])
    return col, arr


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('run')
    ap.add_argument('--json')
    a = ap.parse_args()
    run = Path(a.run)
    ev = events(run)
    phases = [(t, e[6:]) for t, e, _ in ev if e.startswith('PHASE:')]
    other = [(t, e, arg[:60]) for t, e, arg in ev if not e.startswith('PHASE:') and e not in ('LAUNCH', 'MUX:partner')]
    col, arr = table(run)
    t = arr('t')
    clear = next((pt for pt, p in phases if p == 'CLEAR_DROP_ZONE'), None)
    out = {'run': run.name, 'phases': phases, 'events': other}
    if clear is not None and 'his_ref_x' in col:
        p = np.c_[arr('his_ref_x'), arr('his_ref_y'), arr('his_ref_z')]
        v = np.c_[arr('his_ref_vx'), arr('his_ref_vy'), arr('his_ref_vz')]
        body = np.c_[arr('d3_x'), arr('d3_y'), arr('d3_z')]
        m = t >= clear - 1.0
        dt = np.diff(t)
        # the reference's step per row beyond what its own velocity explains (a jump at a switch shows here)
        pred = p[:-1] + v[:-1] * dt[:, None]
        step = np.linalg.norm(p[1:] - pred, axis=1)
        dv = np.linalg.norm(np.diff(v, axis=0), axis=1)
        mm = m[1:] & np.isfinite(step) & (dt > 0) & (dt < 0.2)
        def phase_at(x):
            ph = [p_ for pt, p_ in phases if pt <= x]
            return ph[-1] if ph else ''
        worst = np.argsort(np.where(mm, step, -1))[::-1][:5]
        out['ref_steps_from_clearance'] = [
            {'t': round(float(t[i + 1]), 2), 'phase': phase_at(t[i + 1]), 'dp_m': round(float(step[i]), 4),
             'dv_mps': round(float(dv[i]), 3)} for i in worst if mm[i]]
        out['ref_dv_max_from_clearance'] = round(float(np.nanmax(np.where(mm, dv, np.nan))), 3)
        out['body_z_max_m'] = round(float(np.nanmax(body[m][:, 2])), 3)
        out['body_xy_absmax_m'] = [round(float(np.nanmax(np.abs(body[m][:, k]))), 3) for k in (0, 1)]
        gaps = []
        for d in range(3):
            if f'd{d}_x' in col:
                c = np.c_[arr(f'd{d}_x'), arr(f'd{d}_y'), arr(f'd{d}_z')]
                g = np.linalg.norm(body - c, axis=1)
                gaps.append(float(np.nanmin(g[m])))
        out['body_gap_min_m'] = round(min(gaps), 3) if gaps else None
        landed = next((pt for pt, p_ in phases if p_ == 'LANDED_DISARMED'), None)
        at_start = next((pt for pt, p_ in phases if p_ == 'AT_START'), None)
        out['clear_to_at_start_s'] = None if at_start is None else round(at_start - clear, 2)
        out['clear_to_landed_s'] = None if landed is None else round(landed - clear, 2)
    out['his_lines'] = his_lines(run)
    print(out['run'])
    print('phases: ' + ', '.join(f'{p_} {pt:.1f}' for pt, p_ in phases))
    print('events: ' + ', '.join(f'{e} {pt:.1f}' for pt, e, _ in other))
    print(json.dumps({k: v for k, v in out.items() if k not in ('his_lines', 'phases', 'events', 'run')}))
    print('--- his exit / phase lines')
    for ln in out['his_lines']:
        if re.search(r'C\+\+ exit|Planner phase: (CLEAR|SETTLE|DROP|RETURN|AT_START|LANDING|LANDED)|Exit prefix|COOPERATIVE', ln):
            print(ln)
    if a.json:
        Path(a.json).write_text(json.dumps(out, indent=1))


if __name__ == '__main__':
    main()

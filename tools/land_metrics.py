#!/usr/bin/env python3
"""tools/land_metrics.py -- LAND after the payload touches down (cards 2026-10-03_land_unwind,
2026-10-04_land_matrix), from run.csv and events.csv, load-down to the last armed sample.

    python3 tools/land_metrics.py R0960 R0987 ...

Per run: the worst drone pushed OUTSIDE its reference radially (the rod shove) and LAGGING INSIDE it
(cm, about the ring centre at load-down); the worst drone tilt after the 1.2 s feedforward blend; the
ring's xy shift; and the outcome (landed / ABORT with its reason). Window: load-down to LANDED (or the
abort).
"""
import csv
import glob
import os
import re
import sys

import numpy as np
import pandas as pd

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def land(rid):
    d = glob.glob(os.path.join(REPO, 'results', '*', f'{rid}_*'))[0]
    ev = list(csv.DictReader(open(os.path.join(d, 'logs', 'events.csv'))))
    t_land = [float(e['sim_time']) for e in ev if e['event'] == 'LAND']
    out = dict(run=rid, config=re.sub(r'^R\d{4}_sim_gz_', '', os.path.basename(d)))
    abort = [f"{e['event']} {e['arg']}" for e in ev if 'ABORT' in e['event']]
    out['outcome'] = ('ABORT ' + abort[0][:50]) if abort else (
        'landed' if any(e['event'] == 'LANDED' for e in ev) else 'no LANDED')
    if not t_land:
        out['outcome'] = 'void (no LAND)' if not abort else out['outcome']
        return out
    df = pd.read_csv(os.path.join(d, 'logs', 'run.csv')).drop_duplicates('t')
    n = len([c for c in df.columns if re.match(r'd\d+_tilt_deg$', c)])
    z0 = float(df.payload_z.iloc[:20].median())
    after = df[df.t >= t_land[0]]
    down = after[after.payload_z <= z0 + 0.05]
    if down.empty:
        out['outcome'] += ' (load never down)'
        return out
    t_down = float(down.t.iloc[0])
    t_end = [float(e['sim_time']) for e in ev if e['event'] in ('LANDED', 'FLEET_ABORT')]
    w = df[(df.t >= t_down) & (df.t <= (t_end[0] if t_end else df.t.max()))]
    armed = [f'd{i}_armed' for i in range(n)]
    if all(c in w for c in armed):
        w = w[w[armed].sum(axis=1) > 0]       # until LANDED (a disarmed drone may flop over after it)
    cx, cy = float(w.payload_x.iloc[0]), float(w.payload_y.iloc[0])
    outside, inside, tilt = [], [], []
    for i in range(n):
        r = np.hypot(w[f'd{i}_x'] - cx, w[f'd{i}_y'] - cy)
        rr = np.hypot(w[f'd{i}_ref_x'] - cx, w[f'd{i}_ref_y'] - cy)
        dr = (r - rr).to_numpy()
        outside.append(float(np.nanmax(dr)))
        inside.append(float(-np.nanmin(dr)))
        late = w[w.t >= t_down + 1.2]
        tilt.append(float(late[f'd{i}_tilt_deg'].max()) if len(late) else float('nan'))
    shift = float(np.hypot(w.payload_x - cx, w.payload_y - cy).max())
    out.update(n=n, outside_cm=100 * max(outside), inside_cm=100 * max(inside),
               tilt_deg=max(tilt), ring_shift_cm=100 * shift, window_s=float(w.t.iloc[-1] - t_down))
    return out


def main():
    keys = ['run', 'config', 'n', 'outside_cm', 'inside_cm', 'tilt_deg', 'ring_shift_cm', 'outcome']
    print('| ' + ' | '.join(keys) + ' |')
    print('|' + '---|' * len(keys))
    for rid in sys.argv[1:]:
        r = land(rid)
        print('| ' + ' | '.join(f'{r[k]:.1f}' if isinstance(r.get(k), float) else str(r.get(k, ''))
                                for k in keys) + ' |')


if __name__ == '__main__':
    main()

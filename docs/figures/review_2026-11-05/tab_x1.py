#!/usr/bin/env python3
"""Table rows of tab:res_x1 (the release study, 4.1.2): the mean of the two runs per cell, measured mass distribution.

Wesley 9 Nov: Baseline first, then Proposed and the delays; averages, not ranges; the caption says how many runs.
Tilt rise, settled tilt and height loss are the lab's scores (results/analysis/x1_2026-11-06.json: tilt_rise,
settled_tilt, dip_mm). The second swing is read here from run.csv: the largest tilt from the end of the first swing (its
first minimum after the peak) to +4 s. In every hover and figure-eight run this equals the lab's reading (largest tilt
from +1 to +4 s, F091); in layout (e) the first swing lasts to +1.4-1.6 s, so the lab's window caught its tail (n/a).
"""
import glob
import json
import os

import numpy as np
import pandas as pd

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
CELLS = (('h4', 'Four, hover'), ('f4', 'Four, figure-eight'), ('sqTnoxy', 'Harder detachment, layout (e)'))
ARMS = (('nors', 'Baseline'), ('ours', 'Proposed'), ('d05', r'0.5\,s late'), ('d10', r'1\,s late'),
        ('d20', r'2\,s late'))


def second_swing(rid):
    (d,) = glob.glob(os.path.join(REPO, 'results', '2026-11-0*', f'{rid}_*'))
    run = pd.read_csv(os.path.join(d, 'logs', 'run.csv'), usecols=['t', 'payload_tilt_deg']).drop_duplicates('t')
    ev = pd.read_csv(os.path.join(d, 'logs', 'events.csv'))
    ev = ev[pd.to_numeric(ev.sim_time, errors='coerce').notna()]
    t = run.t.to_numpy(float) - float(ev.loc[ev.event == 'RELEASE', 'sim_time'].astype(float).iloc[0])
    th = run.payload_tilt_deg.to_numpy(float)
    first = (t >= 0) & (t <= 1.5)
    t_peak = t[first][np.argmax(th[first])]
    m = (t > t_peak) & (t <= 4.0)
    ta, tha = t[m], th[m]
    k = next(i for i in range(5, len(tha) - 5) if tha[i] == tha[i - 5:i + 6].min())
    w = (t >= ta[k]) & (t <= 4.0)
    old = th[(t >= 1.0) & (t <= 4.0)].max()
    return th[w].max(), old, ta[k]


def main():
    rows = json.load(open(os.path.join(REPO, 'results', 'analysis', 'x1_2026-11-06.json')))
    lines = []
    for cell, name in CELLS:
        for j, (arm, label) in enumerate(ARMS):
            rr = [r for r in rows if r['cell'] == cell and r['arm'] == arm]
            assert len(rr) == 2, (cell, arm, len(rr))
            sw = [second_swing(r['run']) for r in rr]
            if cell != 'sqTnoxy':
                assert all(abs(a - b) < 1e-9 for a, b, _ in sw), (cell, arm, sw)
            vals = (np.mean([r['abs_peak'] for r in rr]), np.mean([s[0] for s in sw]),
                    np.mean([r['settled_tilt'] for r in rr]), np.mean([r['dip_mm'] for r in rr]))
            print(cell, arm, [r['run'] for r in rr], [round(s[0], 2) for s in sw],
                  'first swing ends', [round(s[2], 2) for s in sw], [round(v, 3) for v in vals])
            head = rf'\multirow{{5}}{{=}}{{{name}}}' if j == 0 else ''
            rule = r'\hline' if j == len(ARMS) - 1 else r'\cline{2-6}'
            lines.append(f'{head} & {label} & {vals[0]:.1f} & {vals[1]:.1f} & {vals[2]:.2f} & {vals[3]:.0f} \\\\ {rule}')
    print('\n'.join(lines))


if __name__ == '__main__':
    main()

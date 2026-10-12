#!/usr/bin/env python3
"""tools/method_table.py -- the overnight method comparison (card 2026-10-04_offset_free_innovation):
one row per method, one column per arm, from the run ids given (method and arm from the config name).

    python3 tools/method_table.py R0841 R0856 R0864 R0832 R0870 R0871 ...

SIL arms: ring height error over the last 8 s before LAND (cm). Gazebo arms (tools/method_metrics.py):
hold z error; push and circle xy error; map z / xy / tilt.
"""
import csv
import glob
import os
import re
import sys

import numpy as np
import pandas as pd

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from method_metrics import metrics  # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
METHODS = [('', 'z_ki 0 (none)'), ('zki', 'z_ki 0.4'), ('A', 'A (force balance)'), ('kt', 'kt_trim'),
           ('of', 'offset-free observer'), ('ofA', 'observer + A'), ('meas', 'INDI f_ext'),
           ('measzki', 'INDI f_ext + z_ki 0.4'), ('ofw', 'observer, window 1.0 s, tau 0.5'),
           ('pin', 'rod rates pinned'), ('pinA', 'rod rates pinned + A'), ('Azki', 'A + z_ki 0.4')]
SUFFIX = re.compile(r'_(zki|A|kt|of|ofA|meas|measzki|ofw|ofw10t05|ofw10t05_long|pin|pinA|Azki)$')


def method_arm(name):
    name = re.sub(r'^R\d{4}_(sim_gz|sil)_', '', name)
    m = SUFFIX.search(name)
    meth = m.group(1) if m else ''
    meth = 'ofw' if meth.startswith('ofw10t05') else meth
    base = name[:m.start()] if m else name
    if base.startswith('b4_wind_circle_n3'):
        base = 'b4_wind_circle_n3'
        meth = {'full': 'A'}.get(name.rsplit('_', 1)[-1], meth)
    return meth, base


def sil_err(d):
    ev = {e['event']: float(e['sim_time']) for e in csv.DictReader(open(os.path.join(d, 'logs', 'events.csv')))}
    s = pd.read_csv(os.path.join(d, 'logs', 'sil.csv'))
    w = s[(s.t >= ev['LAND'] - 8) & (s.t < ev['LAND'])]
    return 100 * float((w.payload_z - w.payload_ref_z).mean())


def cell(rid):
    d = glob.glob(os.path.join(REPO, 'results', '*', f'{rid}_*'))[0]
    meth, arm = method_arm(os.path.basename(d))
    if '_sil_' in os.path.basename(d):
        return meth, 'SIL ' + arm.replace('carry_', '').replace('_long', ''), f'{sil_err(d):+.1f}'
    m = metrics(rid)
    if m.get('arm') == 'void':
        return meth, arm, 'void'
    ab = ' ABORT' if m['aborted'] else ''
    if m['arm'] == 'hold' and arm.endswith(('_map', '_mapneg')):
        return meth, ('map- ' if arm.endswith('neg') else 'map+ ') + '(z / xy / tilt)', f"{m['z_err_cm']:+.1f} / {m['err_cm']:.1f} / {m['tilt']:.1f}{ab}"
    if m['arm'] == 'hold':
        return meth, 'hold z', f"{m['z_err_cm']:+.1f}{ab}"
    if m['arm'] == 'push':
        return meth, 'push xy', f"{m['err_cm']:.1f}{ab}"
    lab = 'wind circle xy' if 'wind' in arm else 'orbit xy / z'
    v = f"{m['err_cm']:.1f}" + ('' if 'wind' in arm else f" / {m['z_err_cm']:+.1f}")
    return meth, lab, v + ab


def main():
    """Arguments: run ids, each optionally 'R0887@method label' to override the method row."""
    cells, order = {}, [label for _, label in METHODS]
    for arg in sys.argv[1:]:
        rid, _, label = arg.partition('@')
        meth, arm, v = cell(rid)
        label = label or dict(METHODS).get(meth, meth)
        if label not in order:
            order.append(label)
        cells.setdefault((label, arm), []).append(f'{v} ({rid})')
    arms = sorted({a for _, a in cells}, key=lambda a: (not a.startswith('SIL'), a))
    print('| method | ' + ' | '.join(arms) + ' |')
    print('|' + '---|' * (len(arms) + 1))
    for label in order:
        if not any(m == label for m, _ in cells):
            continue
        print(f'| {label} | ' + ' | '.join('<br>'.join(cells.get((label, a), [''])) for a in arms) + ' |')


if __name__ == '__main__':
    main()

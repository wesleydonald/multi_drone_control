#!/usr/bin/env python3
"""The tip's motion against plate 9 before the first contact, per twin run (card 2026-10-16_plate_height_filter rev 2).

    python3 tools/n6_mechanism.py results/2026-10-16/R17*_sim_gz_n6_arm_* [...]

Window: his first DESCEND_TO_ATTACHMENT to the first CONTACT (or 10 s if none). rel = tip z - plate-9 point z (the
plate point through the full ring pose, ring frame (0, 0.225, 0.02)); its 0.3-3 Hz sd is the bar's metric. Also: QUAD4's
height sd in 1.2-2.5 Hz, the time inside the catch (xy <= 24 mm) and slow (|d rel/dt| <= 0.03 m/s, 0.1 s mean),
CONTACTs per bottom second (his ATTACH_READY), the weld and the plate's own bounce in the window.
"""
import csv
import os
import sys

import numpy as np
from scipy.signal import butter, filtfilt
from scipy.spatial.transform import Rotation as Rot

PLATE = (0.0, 0.225, 0.02)


def score(d):
    rows = list(csv.DictReader(open(os.path.join(d, 'logs', 'run.csv'))))
    col = lambda k: np.array([float(r[k]) if r[k] not in ('', 'nan') else np.nan for r in rows])  # noqa: E731
    t = col('t')
    ev = list(csv.DictReader(open(os.path.join(d, 'logs', 'events.csv'))))
    times = lambda name: [float(e['sim_time']) for e in ev if e['event'] == name and e['sim_time'] not in ('nan', '')]  # noqa: E731
    desc, cont = times('PHASE:DESCEND_TO_ATTACHMENT'), times('CONTACT')
    if not desc:
        return None
    a = desc[0]
    b = next((c for c in cont if c > a), a + 10.0)
    sel = (t >= a) & (t <= b)
    q = np.column_stack([col('payload_qx'), col('payload_qy'), col('payload_qz'), col('payload_qw')])
    pc = np.column_stack([col('payload_x'), col('payload_y'), col('payload_z')])
    ok = sel & np.all(np.isfinite(q), axis=1) & np.isfinite(col('tip_z'))
    plate = pc[ok] + Rot.from_quat(q[ok]).apply(PLATE)
    tip = np.column_stack([col('tip_x'), col('tip_y'), col('tip_z')])[ok]
    tt = t[ok]
    fs = 1.0 / np.median(np.diff(tt))
    rel = tip[:, 2] - plate[:, 2]
    band = lambda x, lo, hi: filtfilt(*butter(2, [lo / (fs / 2), hi / (fs / 2)], btype='band'), x)  # noqa: E731
    n = max(3, int(round(0.1 * fs)))
    v = np.convolve(np.gradient(rel, tt), np.ones(n) / n, 'same')
    xy = np.linalg.norm(tip[:, :2] - plate[:, :2], axis=1)
    ar = [(float(e['sim_time']), e['event']) for e in ev if e['event'].startswith('PHASE:') and e['sim_time'] not in ('nan', '')]
    bottom = 0.0
    for i, (s, ph) in enumerate(ar):
        if ph == 'PHASE:ATTACH_READY':
            bottom += (ar[i + 1][0] if i + 1 < len(ar) else s) - s
    return {'run': os.path.basename(d)[:5], 'arm': os.path.basename(d).split('_sim_gz_')[1], 'window_s': b - a,
            'rel_sd_cm': 100 * band(rel, 0.3, 3.0).std(), 'plate_bounce_cm': 100 * band(plate[:, 2], 1.2, 2.5).std() if b - a > 2 else float('nan'),
            'quad4_z_sd_cm': 100 * band(col('d3_z')[ok], 1.2, 2.5).std() if b - a > 2 else float('nan'),
            'in_slow_pct': 100 * float(np.mean((xy <= 0.024) & (np.abs(v) <= 0.03))),
            'contacts_per_bottom_s': len(cont) / bottom if bottom > 0 else float('nan'), 'weld': bool(times('WELD'))}


def main(argv=None):
    runs = argv if argv is not None else sys.argv[1:]
    print('run   arm             window  rel sd  plate  QUAD4 z  in+slow  contacts/s  weld')
    for d in sorted(runs):
        r = score(d)
        if r is None:
            print(f'{os.path.basename(d)[:5]} no descent')
            continue
        print(f"{r['run']} {r['arm']:15s} {r['window_s']:5.1f} s {r['rel_sd_cm']:5.2f} cm {r['plate_bounce_cm']:5.2f} {r['quad4_z_sd_cm']:6.2f} cm "
              f"{r['in_slow_pct']:5.0f} % {r['contacts_per_bottom_s']:8.2f}  {r['weld']}")
    return 0


if __name__ == '__main__':
    sys.exit(main())

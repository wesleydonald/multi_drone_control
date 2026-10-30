#!/usr/bin/env python3
"""W4b additions to descents.py: the cut (the ring first below 0.9 m after 40 s), and at the bottom (ATTACH_READY, and
descent samples at or below the contact height) the longest continuous stretch with the tip more than GATE off the plate
centre before a restart, the tip's height then, and ring-tilt kicks (largest 0.2 s tilt change) while the tip was off
the plate against the rest of the hold.

    bottom.py <run dir> <gate m>
"""
import json
import os
import sys

import numpy as np
import pandas as pd


def main(run, gate):
    ev = pd.read_csv(os.path.join(run, 'logs', 'events.csv'))
    ph = [(float(r.sim_time), r.event.split(':', 1)[1]) for r in ev.itertuples()
          if isinstance(r.event, str) and r.event.startswith('PHASE:')]
    x = pd.read_csv(os.path.join(run, 'logs', 'run.csv'))
    t = x.t.values
    low = x[(x.t > 40) & (x.payload_z < 0.9)]
    cut = float(low.t.iloc[0]) if len(low) else float(t[-1])
    err = np.hypot(x.tip_x - x.tip_target_x, x.tip_y - x.tip_target_y).values
    dz = (x.tip_z - x.tip_target_z).values
    tilt = x.payload_tilt_deg.values
    k = 6                                     # 0.2 s at 30 Hz
    kick = np.full(len(t), np.nan)
    kick[k:] = np.abs(tilt[k:] - tilt[:-k])
    out = {'cut_s': round(cut, 1), 'holds': []}
    for i, (t0, p) in enumerate(ph):
        if p != 'ATTACH_READY' or t0 >= cut:
            continue
        t1, nxt = ph[i + 1] if i + 1 < len(ph) else (t[-1], 'END')
        m = (t >= t0) & (t < min(t1, cut))
        off = m & (err > gate) & (dz <= 0.0)
        idx = np.flatnonzero(m)
        best, run_start, run_len = 0.0, None, 0.0
        for j in idx:
            if off[j]:
                run_start = t[j] if run_start is None else run_start
                best = max(best, t[j] - run_start)
            else:
                run_start = None
        h = {'t0': round(t0, 2), 'dur_s': round(min(t1, cut) - t0, 2), 'next': nxt if t1 <= cut else 'CUT',
             'longest_off_plate_low_s': round(best, 2), 'frac_off_plate_low': round(float(off[m].mean()), 3),
             'tip_dz_when_off_cm': round(float(np.median(dz[off])) * 100, 1) if off.any() else None,
             'tilt_kick_off_deg': round(float(np.nanmax(kick[off])), 2) if off.any() else None,
             'tilt_kick_on_deg_p95': round(float(np.nanpercentile(kick[m & ~off], 95)), 2) if (m & ~off).any() else None}
        out['holds'].append(h)
    ce = ev[ev.event.astype(str) == 'CONTACT']
    out['contact'] = float(ce.sim_time.iloc[0]) if len(ce) else None
    print(json.dumps(out))
    return out


if __name__ == '__main__':
    main(sys.argv[1], float(sys.argv[2]))

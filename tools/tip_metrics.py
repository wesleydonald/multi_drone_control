#!/usr/bin/env python3
"""tools/tip_metrics.py -- the measured-tip approach, scored for card 2026-10-09_approach_tip_feedback.

    python3 tools/tip_metrics.py R1330 R1331 ...

From the dissipative planner's tip.csv (node clock; rows end when the approach ends, at LAND or the weld):
  hold_t        the first hold row, s after the first row (None: never held)
  goal_z_mean   mean |tip z - setpoint z| over the last 5 s of rows with a valid tip (m; bar <= 0.01)
  conv_s        hold entry to the first time the 0.5 s mean |tip - setpoint| < 0.01 m (None: never)
  c_end, c_min, c_max   the correction (m)
  invalid_share share of hold rows with an invalid tip
  frozen_dc     the correction's range over rows marked 'frozen' (arm F; m)
  contact       the contact freeze latched
"""
import glob
import json
import os
import sys

import numpy as np
import pandas as pd

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WINDOW_S = 5.0


def run_dir(rid):
    if os.path.isdir(rid):
        return rid
    return glob.glob(os.path.join(REPO, 'results', '*', f'{rid}_*'))[0]


def tip_csv(d):
    hits = sorted(glob.glob(os.path.join(d, 'logs', '**', 'tip.csv'), recursive=True))
    return hits[-1] if hits else None


def analyse(rid):
    d = run_dir(rid)
    f = tip_csv(d)
    out = {'run': os.path.basename(d).split('_')[0]}
    if f is None:
        out['error'] = 'no tip.csv'
        return out
    r = pd.read_csv(f)
    t = r.t.to_numpy() - float(r.t.iloc[0])
    err = (r.tip_z - r.setpoint_z).to_numpy()            # tip above its setpoint is positive
    valid = r.valid.to_numpy() > 0
    hold = r.hold.to_numpy() > 0
    c = r.c_z.to_numpy()
    win = (t >= t[-1] - WINDOW_S) & valid & np.isfinite(err)
    out['goal_z_mean'] = round(float(np.mean(np.abs(err[win]))), 4) if win.any() else None
    out['goal_z_signed'] = round(float(np.mean(err[win])), 4) if win.any() else None
    if hold.any():
        h0 = int(np.argmax(hold))
        out['hold_t'] = round(float(t[h0]), 2)
        roll = pd.Series(np.abs(err)).rolling(5, min_periods=5).mean().to_numpy()
        ok = np.where((np.arange(len(t)) >= h0) & (roll < 0.01))[0]
        out['conv_s'] = round(float(t[ok[0]] - t[h0]), 2) if len(ok) else None
        out['invalid_share'] = round(float(np.mean(~valid[hold])), 3)
    else:
        out['hold_t'] = out['conv_s'] = None
    out['c_end'] = round(float(c[-1]), 4)
    out['c_min'] = round(float(np.min(c)), 4)
    out['c_max'] = round(float(np.max(c)), 4)
    fz = r.why.astype(str).to_numpy() == 'frozen'
    out['frozen_dc'] = round(float(np.ptp(c[fz])), 4) if fz.any() else None
    out['contact'] = bool((r.reason.astype(str) == 'contact').any())
    out['rows'] = int(len(r))
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

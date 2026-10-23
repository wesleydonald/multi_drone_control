#!/usr/bin/env python3
"""tools/tip_metrics.py -- the measured-tip approach, scored for card 2026-10-09_approach_tip_feedback.

    python3 tools/tip_metrics.py R1330 R1331 ...

From the dissipative planner's tip.csv (node clock; rows end when the approach ends, at LAND or the weld):
  hold_t        the first hold row, s after the first row (None: never held)
  goal_z_mean   mean |tip z - setpoint z| over the last 5 s of rows with a valid tip (m; bar <= 0.01)
  conv_s        hold entry to the first time |tip - setpoint| < 0.01 m (instantaneous, as the card words it)
  conv_mean_s   the same on the 0.5 s mean; conv_stay_s: from when it stays inside to the end
  c_end, c_min, c_max   the correction (m)
  invalid_share share of hold rows with an invalid tip
  frozen_dc     the correction's change over the runner's TIP_FREEZE window [t, t + its seconds] (arm F; m), the
                tip.csv clock aligned on the approach's end (LAND or WELD in events.csv = tip.csv's last row)
  push_dc       c's range from the runner's first nonzero WRENCH (force or torque) to + 6 s (arm S; m)
  contact       the contact freeze latched
"""
import glob
import json
import os
import sys

import numpy as np
import pandas as pd

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from experiment.config import wrench_arg  # noqa: E402  (the runner's own WRENCH grammar)

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WINDOW_S = 5.0


def run_dir(rid):
    if os.path.isdir(rid):
        return rid
    return glob.glob(os.path.join(REPO, 'results', '*', f'{rid}_*'))[0]


def is_push(arg):
    """A WRENCH that applies something: any of fx fy fz tx ty tz nonzero. '0 0 0 0.15 0 0 link' is a torque push,
    not the push's removal ('0 0 0' or '0 0 0 0 0 0' [link])."""
    try:
        force, torque, _ = wrench_arg(arg)
    except ValueError:
        return False
    return any(abs(x) > 0 for x in (*force, *torque))


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
        after = np.arange(len(t)) >= h0
        inside = np.abs(err) < 0.01
        ok = np.where(after & inside)[0]
        out['conv_s'] = round(float(t[ok[0]] - t[h0]), 2) if len(ok) else None
        roll = pd.Series(np.abs(err)).rolling(5, min_periods=5).mean().to_numpy()
        ok = np.where(after & (roll < 0.01))[0]
        out['conv_mean_s'] = round(float(t[ok[0]] - t[h0]), 2) if len(ok) else None
        out_rows = np.where(after & ~inside)[0]
        stay = (out_rows[-1] + 1) if len(out_rows) else h0
        out['conv_stay_s'] = round(float(t[stay] - t[h0]), 2) if stay < len(t) else None
        out['invalid_share'] = round(float(np.mean(~valid[hold])), 3)
    else:
        out['hold_t'] = out['conv_s'] = out['conv_mean_s'] = out['conv_stay_s'] = None
    out['c_end'] = round(float(c[-1]), 4)
    out['c_min'] = round(float(np.min(c)), 4)
    out['c_max'] = round(float(np.max(c)), 4)
    out['frozen_dc'] = out['push_dc'] = None
    ev_f = os.path.join(d, 'logs', 'events.csv')
    if os.path.exists(ev_f):
        ev = pd.read_csv(ev_f)
        end = ev[ev.event.isin(['WELD', 'LAND'])]
        if len(end):
            off = float(r.t.iloc[-1]) - float(end.sim_time.iloc[0])     # tip.csv clock minus runner clock
            lg = ev[ev.event == 'LOGGED']
            if hold.any() and len(lg):
                # a WAIT_LOG on the tip-hold line matches the first hold row within ms (reviewer 9 Oct): finer
                off_h = float(r.t[hold].iloc[0]) - float(lg.sim_time.iloc[-1])
                if abs(off_h - off) < 0.5:
                    off = off_h
            tn = r.t.to_numpy()
            fr = ev[ev.event == 'TIP_FREEZE']
            if len(fr):
                a0 = float(fr.sim_time.iloc[0]) + off
                secs = float(fr.arg.iloc[0]) if str(fr.arg.iloc[0]).strip() not in ('', 'nan') else 2.0
                before = np.where(tn < a0 - 0.005)[0]           # from the last row before the freeze
                upto = np.where(tn <= a0 + secs + 0.05)[0]
                if len(before) and len(upto):
                    out['frozen_dc'] = round(float(c[upto[-1]] - c[before[-1]]), 4)
            pw = ev[(ev.event == 'WRENCH') & ev.arg.map(is_push)]
            if len(pw):
                a0 = float(pw.sim_time.iloc[0]) + off
                w = (tn >= a0) & (tn <= a0 + 6.0)
                if w.any():
                    out['push_dc'] = round(float(np.ptp(c[w])), 4)
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

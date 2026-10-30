#!/usr/bin/env python3
"""His contact descents in a twin run, one line each: start, length, how it ended (ATTACH_READY = the bottom, or a
restart back to APPROACH), the deepest tip height relative to the plate's contact target, and for a restart its cause
(position: the tip-to-target xy error over 8 cm when it fired; speed otherwise), the continuous time over 8 cm before
it, and the extra descent while misaligned: the drop of his reference (his MPC's ref_z, the body reference) and of the
measured tip over that continuous over-8 cm stretch, with the tip's height above the target and its radial / tangential
offset at the restart. The bottom holds (ATTACH_READY) are scored the same way.

    descents.py <run dir> [--json out.json]
"""
import glob
import json
import os
import sys

import numpy as np
import pandas as pd

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from align import sim_time_of  # noqa: E402

GATE = float(os.environ.get('DESCENT_GATE', '0.08'))


def main(run, out=None):
    ev = pd.read_csv(os.path.join(run, 'logs', 'events.csv'))
    ph = [(float(r.sim_time), r.event.split(':', 1)[1]) for r in ev.itertuples()
          if isinstance(r.event, str) and r.event.startswith('PHASE:')]
    evs = ev.event.astype(str)
    x = pd.read_csv(os.path.join(run, 'logs', 'run.csv'))
    t = x.t.values
    err = np.hypot(x.tip_x - x.tip_target_x, x.tip_y - x.tip_target_y).values
    dz = (x.tip_z - x.tip_target_z).values
    # the tip's xy speed, a 0.1 s central difference (the target's twist is zeroed in these runs)
    k = 3
    vx = np.full(len(t), np.nan)
    vx[k:-k] = np.hypot(x.tip_x.values[2 * k:] - x.tip_x.values[:-2 * k],
                        x.tip_y.values[2 * k:] - x.tip_y.values[:-2 * k]) / (t[2 * k:] - t[:-2 * k])
    # radial / tangential offset of the tip from plate 9's target in the ring frame (plate 9 on the ring's +y)
    rx, ry = x.payload_x.values, x.payload_y.values
    radial = np.hypot(x.tip_x - rx, x.tip_y - ry).values - np.hypot(x.tip_target_x - rx, x.tip_target_y - ry).values
    mfile = glob.glob(os.path.join(run, 'his_mpc', '*', 'log.csv'))
    mpc = pd.read_csv(mfile[0]) if mfile else None
    if mpc is not None:
        to_sim, _, _ = sim_time_of(x, mpc)
        mt, mref = to_sim(mpc.timestamp.values), mpc.ref_z.values
    rows = []
    for i, (t0, p) in enumerate(ph):
        if p not in ('DESCEND_TO_ATTACHMENT', 'ATTACH_READY'):
            continue
        t1, nxt = (ph[i + 1] if i + 1 < len(ph) else (t[-1], 'END'))
        m = (t >= t0) & (t < t1)
        if m.sum() < 2:
            continue
        r = {'phase': p, 't0': round(t0, 2), 'dur_s': round(t1 - t0, 2), 'next': nxt,
             'min_tip_above_target_cm': round(float(dz[m].min()) * 100, 1),
             'err_median_cm': round(float(np.median(err[m])) * 100, 1)}
        if nxt == 'APPROACH_ABOVE_TARGET':
            j = np.flatnonzero(m)[-1]
            r['err_at_restart_cm'] = round(float(err[j]) * 100, 1)
            r['tip_speed_at_restart'] = round(float(np.nanmax(vx[max(0, j - 2):j + 1])), 3)
            last = [q for q in range(max(0, j - 3), j + 1) if err[q] > GATE - 0.005]  # his tick may see a sample run.csv misses
            speed = float(np.nanmax(vx[max(0, j - 3):j + 1]))
            r['cause'] = 'position' if last else ('speed' if speed > 0.15 else 'unclear')
            if last:
                jj = last[-1]
                a = jj
                while a - 1 >= 0 and err[a - 1] > GATE and t[a - 1] >= t0:
                    a -= 1
                r['over8_s'] = round(float(t[jj] - t[a]), 2)
                r['tip_drop_over8_cm'] = round(float(x.tip_z.values[a] - x.tip_z.values[jj]) * 100, 1)
                if mpc is not None:
                    mm = (mt >= t[a] - 0.02) & (mt <= t[jj] + 0.02)
                    if mm.sum() >= 1:
                        r['ref_drop_over8_cm'] = round(float(mref[mm][0] - mref[mm].min()) * 100, 1)
            r['tip_above_target_at_restart_cm'] = round(float(dz[j]) * 100, 1)
            r['radial_cm'] = round(float(radial[j]) * 100, 1)
            r['tangential_cm'] = round(float(np.sqrt(max(err[j] ** 2 - radial[j] ** 2, 0.0))) * 100, 1)
        rows.append(r)
        print(json.dumps(r))
    names = [p for _, p in ph]
    summary = {
        'descents': names.count('DESCEND_TO_ATTACHMENT'),
        'reached_bottom': sum(1 for r in rows if r['phase'] == 'DESCEND_TO_ATTACHMENT' and r['next'] == 'ATTACH_READY'),
        'restarts_position': sum(1 for r in rows if r.get('cause') == 'position'),
        'restarts_speed': sum(1 for r in rows if r.get('cause') == 'speed'),
        'contact': float(ev[evs == 'CONTACT'].sim_time.iloc[0]) if (evs == 'CONTACT').any() else None,
        'weld': float(ev[evs == 'WELD'].sim_time.iloc[0]) if (evs == 'WELD').any() else None,
        'give_up': float(ev[evs == 'GIVE_UP'].sim_time.iloc[0]) if (evs == 'GIVE_UP').any() else None,
        'take_back': float(ev[evs == 'TAKE_BACK'].sim_time.iloc[0]) if (evs == 'TAKE_BACK').any() else None,
        'deepest_cm': min((r['min_tip_above_target_cm'] for r in rows), default=None),
        'max_ref_drop_over8_cm': max((r.get('ref_drop_over8_cm', 0.0) for r in rows), default=None),
        'max_tip_drop_over8_cm': max((r.get('tip_drop_over8_cm', 0.0) for r in rows), default=None)}
    print('SUMMARY', json.dumps(summary))
    if out:
        with open(out, 'w') as f:
            json.dump({'descents': rows, 'summary': summary}, f, indent=1)


if __name__ == '__main__':
    a = sys.argv[1:]
    main(a[0], a[a.index('--json') + 1] if '--json' in a else None)

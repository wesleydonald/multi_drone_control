#!/usr/bin/env python3
"""tools/app_demo_figure.py -- one check figure and one summary line per run of Te-Jen's application demonstrations
(card docs/experiments/2026-11-11_tejen_app_demos.md): pickup, drop into the ring's net, attach, ring hovering,
circling or flying a figure-eight.

    python3 tools/app_demo_figure.py results/2026-11-11/R4254_sim_gz_tejen_app_hover_twin [more run dirs]

Writes <run>/plots/app_demo.{pdf,png} and prints the summary. Figure: (a) top view, the ring's desired and actual path,
QUAD4's path, the ball, the pickup, drop and attach marked; (b) the ring's x, y, z desired and actual; (c) the ring's
tilt; (d) the ring's path error (desired to actual, horizontal) and QUAD4's and the ball's height. Times are sim seconds.
"""
import csv
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import plot_style  # noqa: E402

MARKS = (('PHASE:TAKEOFF', 'QUAD4 takeoff'), ('PHASE:LIFT_OBJECT', 'pickup'), ('PHASE:DROP_OBJECT', 'drop'),
         ('WELD', 'attach confirmed'), ('LAND', 'LAND'))


def load(run):
    rows = list(csv.DictReader(open(os.path.join(run, 'logs', 'run.csv'))))

    def col(name):
        return np.array([float(r[name]) if r.get(name) not in (None, '') else np.nan for r in rows])
    d = {k: col(k) for k in ('t', 'payload_x', 'payload_y', 'payload_z', 'payload_ref_x', 'payload_ref_y',
                             'payload_ref_z', 'payload_tilt_deg', 'd3_x', 'd3_y', 'd3_z', 'ball_x', 'ball_y', 'ball_z')}
    ev = []
    for r in csv.reader(open(os.path.join(run, 'logs', 'events.csv'))):
        if len(r) >= 2 and r[0] not in ('sim_time', 'nan', ''):
            ev.append((float(r[0]), r[1]))
    return d, ev


def first(ev, name):
    if name == 'WELD':          # the first weld with a sim time; a welded start's own weld has none
        return next((t for t, e in ev if e in ('WELD', 'REWELD')), first(ev, 'PHASE:RING_ATTACHED'))
    return next((t for t, e in ev if e == name), None)


def summary(run, d, ev):
    t = d['t']
    phases = [e for _, e in ev if e.startswith('PHASE:')]
    out = {'run': os.path.basename(run)[:5]}
    out['pickup descents'] = phases.count('PHASE:DESCEND_TO_PICKUP')
    out['attach descents'] = phases.count('PHASE:DESCEND_TO_ATTACHMENT')
    t_to, t_pick, t_drop = first(ev, 'PHASE:TAKEOFF'), first(ev, 'PHASE:LIFT_OBJECT'), first(ev, 'PHASE:DROP_OBJECT')
    t_weld = first(ev, 'WELD')
    t_land = first(ev, 'LAND')
    out['phases in order'] = all(x is not None for x in (t_to, t_pick, t_drop, t_weld))
    for k, v in (('QUAD4 takeoff', t_to), ('pickup', t_pick), ('drop', t_drop), ('attach confirmed', t_weld),
                 ('LAND', t_land)):
        out[k + ' (s)'] = None if v is None else round(v, 1)
    if t_to is not None and t_weld is not None:
        out['takeoff to attach (s)'] = round(t_weld - t_to, 1)
    if t_drop is not None:
        i = np.searchsorted(t, t_drop + 5.0)
        if i < len(t):
            out['ball from ring centre 5 s after drop (cm)'] = round(100 * math.hypot(
                d['ball_x'][i] - d['payload_x'][i], d['ball_y'][i] - d['payload_y'][i]), 1)
            out['ball above ring plane (cm)'] = round(100 * (d['ball_z'][i] - d['payload_z'][i]), 1)
    vx = np.gradient(d['payload_x'], t)
    vy = np.gradient(d['payload_y'], t)
    for k, v in (('drop', t_drop), ('attach confirmed', t_weld)):
        if v is not None:
            m = (t > v - 0.5) & (t < v + 0.5)
            out[f'ring speed at {k} (m/s)'] = round(float(np.nanmean(np.hypot(vx[m], vy[m]))), 3)
    up = (d['payload_z'] > 0.9 * np.nanmax(d['payload_z']))
    end = t_land if t_land is not None else t[-1]
    fly = up & (t < end)
    err = np.hypot(d['payload_x'] - d['payload_ref_x'], d['payload_y'] - d['payload_ref_y'])
    if fly.any():
        out['ring path error rms / max (cm)'] = (round(100 * float(np.sqrt(np.nanmean(err[fly] ** 2))), 1),
                                                round(100 * float(np.nanmax(err[fly])), 1))
        out['ring tilt peak in flight (deg)'] = round(float(np.nanmax(d['payload_tilt_deg'][fly])), 1)
    return out


def figure(run, d, ev, s):
    plot_style.use()
    import matplotlib.pyplot as plt
    t_land = first(ev, 'LAND')
    keep = d['t'] <= (t_land + 8.0 if t_land is not None else d['t'][-1])     # to just after LAND
    d = {k: v[keep] for k, v in d.items()}
    t = d['t']
    fig, ax = plt.subplots(2, 2, figsize=(12, 8.5))
    a = ax[0, 0]
    a.plot(d['payload_ref_x'], d['payload_ref_y'], '--', color='k', lw=1.2, label='ring desired')
    a.plot(d['payload_x'], d['payload_y'], '-', color='tab:blue', lw=1.4, label='ring actual')
    a.plot(d['d3_x'], d['d3_y'], '-', color=plot_style.drone_colour(3), lw=0.8, alpha=0.7, label='QUAD4')
    a.plot(d['ball_x'], d['ball_y'], ':', color='tab:orange', lw=1.2, label='object')
    for key, lab, mk in (('PHASE:LIFT_OBJECT', 'pickup', 'o'), ('PHASE:DROP_OBJECT', 'drop', 's'), ('WELD', 'attach', '^')):
        v = first(ev, key)
        if v is not None:
            i = min(np.searchsorted(t, v), len(t) - 1)
            a.plot(d['d3_x'][i], d['d3_y'][i], mk, color='k', ms=7, label=f'QUAD4 at {lab}')
    a.set_aspect('equal')
    a.set_xlabel('x (m)')
    a.set_ylabel('y (m)')
    a.set_title('(a) top view')
    a.legend(fontsize=7, loc='best')
    a = ax[0, 1]
    for k, c in (('x', 'tab:red'), ('y', 'tab:green'), ('z', 'tab:blue')):
        a.plot(t, d[f'payload_ref_{k}'], '--', color=c, lw=1.0)
        a.plot(t, d[f'payload_{k}'], '-', color=c, lw=1.2, label=f'ring {k}')
    a.set_title('(b) ring position: desired dashed, actual solid')
    a.set_ylabel('m')
    a.legend(fontsize=7)
    a = ax[1, 0]
    a.plot(t, d['payload_tilt_deg'], color='tab:blue', lw=1.2)
    a.set_title('(c) ring tilt')
    a.set_ylabel('deg')
    a = ax[1, 1]
    a.plot(t, 100 * np.hypot(d['payload_x'] - d['payload_ref_x'], d['payload_y'] - d['payload_ref_y']),
           color='tab:blue', lw=1.2, label='ring path error (cm)')
    a2 = a.twinx()
    a2.plot(t, d['d3_z'], color=plot_style.drone_colour(3), lw=1.0, label='QUAD4 height (m)')
    a2.plot(t, d['ball_z'], ':', color='tab:orange', lw=1.2, label='object height (m)')
    a2.set_ylabel('m')
    a.set_ylabel('cm')
    a.set_title('(d) ring path error; QUAD4 and object height')
    h1, l1 = a.get_legend_handles_labels()
    h2, l2 = a2.get_legend_handles_labels()
    a.legend(h1 + h2, l1 + l2, fontsize=7, loc='upper left')
    for a in (ax[0, 1], ax[1, 0], ax[1, 1]):
        a.set_xlabel('time (s)')
        for key, lab in MARKS:
            v = first(ev, key)
            if v is not None:
                a.axvline(v, color='0.4', ls=':', lw=0.9)
                a.text(v, a.get_ylim()[1], ' ' + lab, rotation=90, va='top', ha='left', fontsize=7, color='0.3')
    fig.suptitle(f"{os.path.basename(run)}  (simulation)", fontsize=10)
    fig.tight_layout()
    out = os.path.join(run, 'plots')
    os.makedirs(out, exist_ok=True)
    for ext in ('pdf', 'png'):
        fig.savefig(os.path.join(out, f'app_demo.{ext}'), dpi=130)
    plt.close(fig)
    return os.path.join(out, 'app_demo.png')


def main():
    for run in sys.argv[1:]:
        run = run.rstrip('/')
        d, ev = load(run)
        s = summary(run, d, ev)
        png = figure(run, d, ev, s)
        print('; '.join(f'{k}: {v}' for k, v in s.items()))
        print('   figure:', png)


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Print the Q013 item 3 and 4 tables as markdown from the CSVs beside this file (drops.py, pickups.py, mission.py
first).

    cd ~/multi_drone_control && python3 results/analysis/rig_results_2026-11-13/tables.py
"""
import os

import pandas as pd

HERE = os.path.dirname(os.path.abspath(__file__))


def md(df):
    cols = list(df.columns)
    out = ['| ' + ' | '.join(cols) + ' |', '|' + '---|' * len(cols)]
    for _, r in df.iterrows():
        out.append('| ' + ' | '.join('' if pd.isna(v) else str(v) for v in r.values) + ' |')
    return '\n'.join(out)


def f(v, n=1):
    return '' if pd.isna(v) else f'{v:.{n}f}'


def drops():
    d = pd.read_csv(os.path.join(HERE, 'drop_disturbance.csv'))
    rows = []
    for _, r in d.iterrows():
        if r.released != 'yes':
            continue
        obj = f(r.obj_settled_cm) if not pd.isna(r.obj_settled_cm) else (f'({f(r.impact_xy_cm)} at impact)'
                                                                          if not pd.isna(r.impact_xy_cm) else '')
        if not pd.isna(r.obj_settled_cm) and r.obj_settled_cm > 50:
            obj = f'on the floor (hit {f(r.impact_xy_cm)} out)'
        rows.append({'flight': r.flight, 'day': r.day[5:], 'ring': r.motion, 'transfer': r.transfer,
                     'ring / ref speed m/s': f'{f(r.ring_speed_mps, 3)} / {f(r.ref_speed_mps, 3)}',
                     'object over plane at release cm': f(r.obj_dz_release_cm, 0),
                     'object from centre cm': obj, 'in net': r.in_net if not pd.isna(r.in_net) else '',
                     'window s (end)': f'{f(r.window_s, 1)} ({r.window_end.split(":")[0]})',
                     'tilt before mean / max -> peak deg':
                         f'{f(r.tilt_pre_deg)} / {f(r.tilt_pre_max_deg)} -> {f(r.tilt_peak_deg)}',
                     'swing deg': f(r.swing_deg),
                     'dip before -> after mm': f'{f(r.dip_pre_mm, 0)} -> {f(r.dip_mm, 0)}',
                     'xy error before mean / max -> peak cm':
                         f'{f(r.xy_err_pre_mean_cm)} / {f(r.xy_err_pre_max_cm)} -> {f(r.xy_err_peak_cm)}',
                     'recover s': f(r.recover_s) if not pd.isna(r.recover_s) else 'not in window',
                     'tilt settle s': f(r.settle_tilt_s)})
    print(md(pd.DataFrame(rows)))


def pickups():
    d = pd.read_csv(os.path.join(HERE, 'pickups.csv'))
    d['try_held'] = d.try_held.map(lambda v: '' if pd.isna(v) else str(int(v)))
    print(md(d[['flight', 'day', 'object', 'carriers', 'ring_during_pickup', 'tries', 'lift_started', 'held',
                'held_then_fell', 'try_held']].fillna('')))


def mission(fn):
    d = pd.read_csv(os.path.join(HERE, fn))
    d = d[['flight', 'phase', 'start_s', 'end_s', 'duration_s', 'n_ours', 'quad4_by', 'ring', 'detail']].fillna('')
    print(md(d))


if __name__ == '__main__':
    drops()
    print()
    pickups()
    print()
    mission('full_mission_fm9_5.csv')
    print()
    mission('moving_half_fm9hc.csv')

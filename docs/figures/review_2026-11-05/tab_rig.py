#!/usr/bin/env python3
"""Rows of the real-world tables of 4.2 (Setup 3), printed as LaTeX (Wesley 13 Nov: the real world event by event up to
the full mission; the mean of the flights in each row, the count in a column).

    drop     tab:res_drop     results/analysis/rig_results_2026-11-13/drop_disturbance.csv (F113, F116, F117): the 13
             drops into the net, grouped by the ring's motion at the release (measured speed, the 1 s chord, under
             0.05 m/s = at rest). Before = the largest value over the 5 s before the release; after = the peak over
             0-5 s (F116's equal windows). fm9h8_13's dip is taken on a 5 s baseline, 35 mm (F116; its table value
             45 mm stands on a 1.43 s baseline). Recovered = recover_0to5 (F117).
    mission  tab:res_mission  full_mission_fm9_5.csv (F115, F116): the phases of RIG-1112-fm9_5 merged as in the text.
"""
import os
import sys

import pandas as pd

DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..', 'results', 'analysis',
                   'rig_results_2026-11-13')
DIP_5S = {('fm9h8_13', '2026-11-13'): 35.0}


def drop():
    d = pd.read_csv(os.path.join(DIR, 'drop_disturbance.csv'))
    d = d[d.in_net == 'yes'].copy()
    d['dip'] = [DIP_5S.get((f, day), v) for f, day, v in zip(d.flight, d.day, d.dip_0to5_mm)]
    d['group'] = ['At rest' if v < 0.05 else ('Circle' if m == 'circle' else 'Figure-eight')
                  for v, m in zip(d.ring_speed_mps, d.motion)]
    groups = ('At rest', 'Circle', 'Figure-eight')
    xs = [d[d.group == g] for g in groups]
    for g, x in zip(groups, xs):
        print('%', g, list(zip(x.flight, x.day, x.transfer)))
    rows = (('Drops', lambda x: f'{len(x)}'),
            ('Ring speed (m/s)', lambda x: f'{x.ring_speed_mps.mean():.2f}'),
            ("Object from the ring's centre (cm)", lambda x: f'{x.obj_settled_cm.mean():.1f}'),
            (r'Tilt, largest before ($^\circ$)', lambda x: f'{x.tilt_pre_max_deg.mean():.1f}'),
            (r'Tilt, peak after ($^\circ$)', lambda x: f'{x.tilt_peak_0to5_deg.mean():.1f}'),
            ('Height dip (mm)', lambda x: f'{x.dip.mean():.0f}'),
            ('Horizontal error, largest before (cm)', lambda x: f'{x.xy_err_pre_max_cm.mean():.1f}'),
            ('Horizontal error, peak after (cm)', lambda x: f'{x.xy_err_peak_0to5_cm.mean():.1f}'),
            (r'Recovered within 5\,s', lambda x: f"{int((x.recover_0to5 == 'yes').sum())} of {len(x)}"))
    for name, f in rows:
        print(name + ' & ' + ' & '.join(f(x) for x in xs) + r' \\ \hline')
    print('% all 13: transfer', d.transfer.value_counts().to_dict(), 'speed moving', d[d.group != 'At rest'].ring_speed_mps
          .agg(['min', 'max']).round(3).to_dict(), 'object', d.obj_settled_cm.agg(['min', 'max']).round(1).to_dict(),
          'release dz', d.obj_dz_release_cm.agg(['min', 'max']).round(1).to_dict(),
          'tilt after <= before', int((d.tilt_peak_0to5_deg <= d.tilt_pre_max_deg).sum()),
          'recovered 0-5', int((d.recover_0to5 == 'yes').sum()), 'recovered 10 s', int(d.recover_s.notna().sum()))


def mission():
    p = pd.read_csv(os.path.join(DIR, 'full_mission_fm9_5.csv'))
    s = dict(zip(p.phase, p.start_s))
    e = dict(zip(p.phase, p.end_s))
    rows = (('Takeoff and lift', 0.0, s['detach: unload over 3 s, our magnet OFF'], '4'),
            ('Detachment, step out', s['detach: unload over 3 s, our magnet OFF'],
             e['QUAD4 steps out'], r'4 to 3'),
            ('Pickup, three tries', s['his takeoff and approach over the object'], s['object lift'], '3'),
            ('Lift and transit', s['object lift'], s['settle and descent over the ring'], '3'),
            ('Descent and drop', s['settle and descent over the ring'], e['drop (release dwell)'], '3'),
            ('Reattachment, two contacts', e['drop (release dwell)'],
             e['fold-in (proof -> FLEET RESIZED n=4)'], '3 to 4'),
            ('Hover with four', e['fold-in (proof -> FLEET RESIZED n=4)'], s['LAND (command -> fleet landed and disarmed)'],
             '4'))
    for name, a, b, n in rows:
        print(f'{name} & {a:.1f} & {b - a:.1f} & {n} \\\\ \\hline')


if __name__ == '__main__':
    for name in sys.argv[1:] or ('drop', 'mission'):
        globals()[name]()

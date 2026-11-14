#!/usr/bin/env python3
"""Print the Q013 item 1 and 2 tables as markdown from detach_planned.csv, reattach_attempts.csv and
reattach_resize.csv (detach_planned.py and reattach.py first).

    cd ~/multi_drone_control && python3 results/analysis/rig_results_2026-11-13/detach_reattach_tables.py
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
    try:
        return '' if pd.isna(v) else f'{float(v):.{n}f}'
    except (TypeError, ValueError):
        return str(v)


def detach():
    d = pd.read_csv(os.path.join(HERE, 'detach_planned.csv'))
    rows = []
    for _, r in d.iterrows():
        at_off = r.motion_at_off
        if isinstance(at_off, str) and at_off.startswith('circle paused'):
            at_off = f'circle paused by the unload, moves again +{f(r.setpoint_moves_after_off_s)} s'
        elif isinstance(at_off, str) and at_off.startswith('hover hold before'):
            at_off = f'hover hold, set-point moves +{f(r.setpoint_moves_after_off_s)} s'
        rows.append({
            'flight': r.flight, 'day': r.day[5:], 'setup': r.setup,
            'event': 'planned' if r.setup == 3 else 'unannounced',
            'ring at the OFF (config)': f'{at_off} ({r.motion})' if at_off != r.motion else r.motion,
            'ring speed m/s': f(r.ring_speed_mps, 3),
            'resize s': (f'{f(r.unload_to_resize_s, 2)} from UNLOAD' if r.setup == 3
                         else f'{f(r.resize_s, 2)} after OFF (detected {f(r.detect_delay_s, 2)})'),
            'before s': f(r.pre_s, 1),
            'tilt before deg': f(r.tilt_pre_deg), 'swing deg': f(r.swing_deg), 'tilt peak deg': f(r.tilt_peak_deg),
            'settle (old / new tilt) s': f'{f(r.settle_s) or "-"} / {f(r.settle_new_s) or "-"}',
            'dip mm': f(r.dip_mm, 0), 'horiz cm': f(r.horiz_offset_cm),
            'window s (end)': f'{f(r.window_s)} ({r.window_end})',
            'note': ('planner at 1.15 kg' if r.planner_ring_kg > 1.0 else '') +
                    ('square (T after)' if r.formation == 'square' else '') +
                    ('plates 3/6/11' if r.formation == 'plates 3/6/11' else '')})
    print(md(pd.DataFrame(rows)))


def attempts():
    d = pd.read_csv(os.path.join(HERE, 'reattach_attempts.csv'))
    rows = []
    for _, r in d.iterrows():
        rows.append({'flight': r.flight, 'day': r.day[5:], 'kind': r.kind, 'plate': r.plate,
                     'ring': (r.ring_during_reattach if isinstance(r.ring_during_reattach, str) else
                              f'({r.motion_config}; no descent)'),
                     'funnel': r.funnel, 'descents': r.descents, 'at plate height': r.descents_at_plate_height,
                     'contacts': r.contacts, 'proofs failed': r.proofs_failed, 'proven': r.proven,
                     'held': r.held + (f' ({f(r.held_s)} s)' if r.proven == 'yes' else ''),
                     'start -> proven s': f(r.start_to_proven_s),
                     'why not': '' if pd.isna(r.fail_reason) else r.fail_reason,
                     'excluded': '' if pd.isna(r.exclude) else r.exclude})
    print(md(pd.DataFrame(rows)))


def resize():
    d = pd.read_csv(os.path.join(HERE, 'reattach_resize.csv'))
    rows, trows = [], []
    for _, r in d.iterrows():
        rows.append({'flight': r.flight, 'day': r.day[5:], 'funnel': r.funnel,
                     'contact / proof / fold-in / blend end s': f'{f(r.t_contact_s)} / {f(r.t_proof_s)} / '
                                                                f'{f(r.t_foldin_s)} / {f(r.t_blend_end_s)}',
                     'contact->proof s': f(r.contact_to_proof_s, 2), 'proof->fold-in s': f(r.proof_to_foldin_s, 2),
                     'LAND after fold-in s': f(r.land_after_foldin_s), 'window s (end)':
                         f'{f(r.window_s)} ({r.window_end})',
                     'tilt before / peak deg': f'{f(r.tilt_pre_deg)} / {f(r.tilt_peak_deg)}',
                     'swing deg': f(r.swing_deg), 'dip mm': f(r.dip_mm, 0), 'horiz cm': f(r.horiz_offset_cm),
                     'held': r.held})
        tr = {'flight': r.flight}
        car = sorted(range(3), key=lambda k: r[f'plate_d{k}'])
        for j, k in enumerate(car + [3]):
            lab = 'QUAD4' if k == 3 else f'carrier {j + 1}'
            where = (f"{r.quad4_catch_angle_cw_deg:.0f} deg" if k == 3 else f"plate {int(r[f'plate_d{k}'])}")
            pull = f(r[f'Tpull_blend_end_d{k}_N'], 2) or 'LAND first'
            tr[lab] = (f"{where}: {f(r[f'T_start_d{k}_N'], 2)} -> {f(r[f'T_end_d{k}_N'], 2)}; "
                       f"{f(r[f'Tplan0_fold_d{k}_N'], 2)} -> {f(r[f'Tplan0_blend_end_d{k}_N'], 2)}; {pull}")
        trows.append(tr)
    print(md(pd.DataFrame(rows)))
    print()
    print(md(pd.DataFrame(trows)))


if __name__ == '__main__':
    detach()
    print()
    attempts()
    print()
    resize()

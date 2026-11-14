#!/usr/bin/env python3
"""Averaged rows for the results tables (Wesley 9 Nov: averages, not ranges; the caption gives the runs; no Runs column in
the simulation tables).

Each table's numbers are the mean over its runs, read from the lab's scored files (results/analysis/) or, where none
holds the number, from the run logs. Every source reproduces the range the table gave before (checked 9 Nov).
  tab:res_sim_fixed s41_s42_2026-11-07.json (R4014-R4031)
  tab:res_gap       x11_gap_2026-11-07.json (120 deg: R4032, R4033, R4149; R4148's detection 0.29 s from F084)
  tab:res_join      x11_joins_m2a_2026-11-09.json (R4214-R4227, Setup 3's ring; Q011, F097/F098; the 1.15 kg runs
                    R4094-R4107 replaced); the tilt from 1 s after the weld, from run.csv
  tab:res_rejoin    rejoin_oldlinks_2026-11-07.json (R3992-R3999)
  tab:res_s8        x3b_oneModel_2026-11-07.json runs R3974/R3975 (hover), R3972/R3973 (sequence)
  tab:res_x3b       x3b_oneModel_2026-11-07.json (P rows, verdicts, full-size solve p95 from ticks.per_n)
  tab:res_x4        x4_oldlinks_2026-11-06.json, without R3912 (the 1.15 kg relay)
  tab:res_compare   x11_quan_2026-11-07.json, x8_2026-11-07.json, x8_hub_noxy_2026-11-07.json
  tab:res_both      x11_gap_2026-11-07.json (R4032/R4033); recovery 2.63 s in both runs (F077); layout (e) while away:
                    mean tilt from the magnet off to the re-attachment, RIG-1105-sq8_2 (log: magnet OFF to FLEET RESIZED
                    n=4) and R3708, R3712, R3714, R3715 (RELEASED to REWELD; their medians reproduce F049's)
  tab:res_descents  rig_joins_2026-11-04_05/joins.csv (miss at plate or target height, tip_xy_cm)
"""
import csv
import glob
import json
import os
import re

import numpy as np
import pandas as pd

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
A = os.path.join(REPO, 'results', 'analysis')


def load(name):
    return json.load(open(os.path.join(A, name)))


def mean(v):
    return float(np.mean(v))


def join():
    """Q011 re-fly on Setup 3's ring (R4214-R4227, F097/F098). The ring tilt is the absolute tilt's peak from 1 s after
    the weld to +15 s: in layout (e) the ring already leans 2.1-2.7 deg as the weld happens (F098 item 1)."""
    d = load('x11_joins_m2a_2026-11-09.json')['runs']
    rows = (('From the floor, layout (e)', ['R4216', 'R4217', 'R4218', 'R4219']),
            ('From the floor, layout (d)', ['R4224', 'R4225', 'R4226', 'R4227']),
            ('Frozen magnet position, layout (d)', ['R4220', 'R4221', 'R4222', 'R4223']),
            ('Pickup and drop into the ring, then attachment', ['R4214', 'R4215']))

    def tilt_after(rid):
        (rd,) = glob.glob(os.path.join(REPO, 'results', '2026-11-*', f'{rid}_*'))
        r = pd.read_csv(os.path.join(rd, 'logs', 'run.csv'), usecols=['t', 'payload_tilt_deg']).drop_duplicates('t')
        ev = pd.read_csv(os.path.join(rd, 'logs', 'events.csv'))
        ev = ev[pd.to_numeric(ev.sim_time, errors='coerce').notna()]
        tw = float(ev.loc[ev.event == 'WELD', 'sim_time'].astype(float).iloc[0])
        return float(r.payload_tilt_deg[(r.t >= tw + 1.0) & (r.t <= tw + 15.0)].max())

    out = []
    for name, rr in rows:
        t = mean([d[r]['his_takeoff_to_weld_s'] for r in rr])
        c = mean([d[r]['contacts'] for r in rr])
        tilt = mean([tilt_after(r) for r in rr])
        out.append(f'{name} & {t:.1f} & {c:g} & {tilt:.1f} \\\\ \\hline')
    return out


def sim_fixed():
    d = load('s41_s42_2026-11-07.json')
    out = []
    for n in (2, 3, 4):
        cells = []
        for kind, fmt in (('hover', '{:.1f}'), ('figure-8', '{:.1f}'), ('spinny line', '{:.1f}')):
            rr = [r for r in d if r.get('carriers') == n and r['kind'] == kind]
            assert len(rr) == 2
            cells += [fmt.format(mean([r['err_mm'] for r in rr])), f"{mean([r['tilt_peak_deg'] for r in rr]):.2f}"]
        out.append(f'{n} & ' + ' & '.join(cells) + ' \\\\ \\hline')
    return out


def gap():
    d = load('x11_gap_2026-11-07.json')
    arms = d['arms']
    out = []
    for key, name, margin, tilt, outcome in (
            ('G120 (x11_r1m_ta_a1 r1/r2/r3)', '120$^\\circ$', 112, None, 'Held'),
            ('G135', '135$^\\circ$', 86, None, 'Held'),
            ('G150', '150$^\\circ$', 58, None, 'Held'),
            ('G165', '165$^\\circ$', 29, 'Over 30', 'One held, one dropped'),
            ('G180, drop rule off', '180$^\\circ$, drop rule off', 0, 'Over 60', 'Tipped over'),
            ('G180d (drop rule on, optional, n=1)', '180$^\\circ$, drop rule on', 0, 'Over 21', 'Ring dropped')):
        rr = arms[key]['ids']
        det = [d['runs'][r]['detection_s_planner_log_10Hz'] for r in rr]
        det = [0.29 if x is None else x for x in det]                 # R4148: F084, dropped at +0.29 s
        t = tilt or f"{mean([d['runs'][r]['abs_peak'] for r in rr]):.1f}"     # Wesley 12 Nov: the absolute peak tilt
        out.append(f'{name} & {margin} & {mean(det):.2f} & {t} & {outcome} \\\\ \\hline')
    return out


def rejoin():
    d = load('rejoin_oldlinks_2026-11-07.json')
    names = (('Square', 'Four to three and back, layout (e)'), ('T', 'Three to two and back'),
             ('T, leaning toward', 'Three to two and back, leaning toward'),
             ('T, leaning away', 'Three to two and back, leaning away'))
    out = []
    for case, name in names:
        rr = [x for x in d if x['case'] == case]
        assert len(rr) == 2
        j = [x['rejoins'][0] for x in rr]
        out.append(f"{name} & {mean([e['plate_s'] for e in j]):.1f} & {mean([e['fold_peak_deg'] for e in j]):.2f} & "
                   f"{mean([e['rod_end_cm'] for e in j]):.2f} & {mean([x['lean_deg'] for x in rr]):.2f} \\\\ \\hline")
    return out


def s8():
    runs = load('x3b_oneModel_2026-11-07.json')['runs']
    H, C = ('R3974', 'R3975'), ('R3972', 'R3973')
    m = lambda rr, f: mean([f(runs[r]) for r in rr])  # noqa: E731
    rows = [('Real-time factor', lambda r: r['manifest']['mean_rtf'], '{:.3f}', True),
            ('Late control steps (\\%)', lambda r: 100 * r['planner']['late'], '{:.2f}', True),
            (None, 'Eight quadrotors, before any event'),
            ('Position error, RMS (mm)', lambda r: 1e3 * r['n1']['pos_rmse_m'], '{:.2f}', True),
            ('Tilt peak ($^\\circ$)', lambda r: r['n1']['tilt_peak_deg'], '{:.2f}', True),
            (None, 'Detachments, mean of four'),        # Wesley 12 Nov: the mean over all events, as Table 18
            ('Detection (s)', lambda r: mean([e['resize_s'] for e in r['releases']]), '{:.2f}', False),
            ('Tilt rise ($^\\circ$)', lambda r: mean([e['peak_deg'] for e in r['releases']]), '{:.2f}', False),
            ('Tilt before the next event ($^\\circ$)', lambda r: mean([e['settled_deg'] for e in r['releases']]),
             '{:.2f}', False),
            (None, 'Reattachments, mean of four'),
            ('Time from start to resizing (s)', lambda r: mean([e['bigger_s'] for e in r['rejoins']]), '{:.1f}', False),
            ('Tilt peak ($^\\circ$)', lambda r: mean([e['fold_peak_deg'] for e in r['rejoins']]), '{:.2f}', False),
            ('Tilt before the next event ($^\\circ$)', lambda r: mean([e['settled_deg'] for e in r['rejoins']]),
             '{:.2f}', False),
            (None, 'Eight again, from 2\\,s after the tensions were last blended'),
            ('Position error, RMS (mm)', lambda r: 1e3 * r['n4']['pos_rmse_m'], '{:.2f}', False),
            ('Tilt peak ($^\\circ$)', lambda r: r['n4']['tilt_peak_deg'], '{:.2f}', False)]
    out = []
    for row in rows:
        if row[0] is None:
            out.append(f'\\multicolumn{{3}}{{|l|}}{{\\textbf{{{row[1]}}}}} \\\\ \\hline')
            continue
        name, f, fmt, hover = row
        h = fmt.format(m(H, f)) if hover else 'N/A'
        out.append(f'{name} & {h} & {fmt.format(m(C, f))} \\\\ \\hline')
    return out


def x3b():
    d = load('x3b_oneModel_2026-11-07.json')
    P = {(k, m['metric']): m for k, lst in d['P'].items() for m in lst}
    runs, ver = d['runs'], d['verdicts']
    pm = lambda cell, metric, arm: mean(list(P[(cell, metric)][arm].values()))  # noqa: E731
    ev = lambda rr, key, f: mean([f(e) for r in rr for e in ver[r][key]])  # noqa: E731
    full = lambda rr: mean([runs[r]['ticks']['per_n'][max(runs[r]['ticks']['per_n'], key=int)]['solve_ms']['p95']  # noqa: E731
                            for r in rr])
    n1 = lambda rr, k: mean([(1e3 if k == 'pos_rmse_m' else 1) * runs[r]['n1'][k] for r in rr])  # noqa: E731
    one8, five8 = ['R4196', 'R4197', 'R4198', 'R4199', 'R3984', 'R3985'], ['R3974', 'R3975', 'R3976', 'R3977',
                                                                           'R3972', 'R3973']
    return [
        f"\\multirow{{2}}{{*}}{{Eight, hover}} & Position error (mm) & {pm('hover', 'N1 RMSE (mm)', 'rt'):.1f} & "
        f"{pm('hover', 'N1 RMSE (mm)', 'it5'):.1f} \\\\ \\cline{{2-4}}",
        f" & Tilt peak ($^\\circ$) & {pm('hover', 'N1 tilt peak (deg)', 'rt'):.2f} & "
        f"{pm('hover', 'N1 tilt peak (deg)', 'it5'):.2f} \\\\ \\hline",
        f"\\multirow{{2}}{{*}}{{Eight, figure-eight}} & Contour error (mm) & "
        f"{pm('fig8', 'clock-free path RMSE (mm)', 'rt'):.1f} & {pm('fig8', 'clock-free path RMSE (mm)', 'it5'):.1f} "
        f"\\\\ \\cline{{2-4}}",
        f" & Tilt peak ($^\\circ$) & {pm('fig8', 'path tilt peak (deg)', 'rt'):.2f} & "
        f"{pm('fig8', 'path tilt peak (deg)', 'it5'):.2f} \\\\ \\hline",
        f"\\multirow{{2}}{{*}}{{Eight to four and back}} & Tilt rise at a detachment ($^\\circ$) & "
        f"{ev(['R3984', 'R3985'], 'L', lambda e: e['peak_deg']):.2f} & "
        f"{np.mean([e['peak_deg'] for r in ('R3972', 'R3973') for e in runs[r]['releases']]):.2f} \\\\ \\cline{{2-4}}",
        f" & Tilt peak at a reattachment ($^\\circ$) & {ev(['R3984', 'R3985'], 'J', lambda e: e['fold_peak_deg']):.2f} & "
        f"{np.mean([e['fold_peak_deg'] for r in ('R3972', 'R3973') for e in runs[r]['rejoins']]):.2f} \\\\ \\hline",
        f"Eight, every case & Solve time, 95th percentile (ms) & {full(one8):.0f} & {full(five8):.0f} \\\\ \\hline",
        f"\\multirow{{2}}{{*}}{{Nine, hover}} & Position error (mm) & {n1(['R4194', 'R4195'], 'pos_rmse_m'):.1f} & N/A "
        f"\\\\ \\cline{{2-4}}",
        f" & Tilt peak ($^\\circ$) & {n1(['R4194', 'R4195'], 'tilt_peak_deg'):.2f} & N/A \\\\ \\hline",
        f"\\multirow{{2}}{{*}}{{Nine to five and back}} & Tilt rise at a detachment ($^\\circ$) & "
        f"{ev(['R3980', 'R3981'], 'L', lambda e: e['peak_deg']):.2f} & N/A \\\\ \\cline{{2-4}}",
        f" & Tilt peak at a reattachment ($^\\circ$) & {ev(['R3980', 'R3981'], 'J', lambda e: e['fold_peak_deg']):.2f} "
        f"& N/A \\\\ \\hline",
        f"Nine, every case & Solve time, 95th percentile (ms) & {full(['R4194', 'R4195', 'R3980', 'R3981']):.0f} & N/A "
        f"\\\\ \\hline"]


def x4():
    d = {r['run'][:5]: r for r in load('x4_oldlinks_2026-11-06.json')}
    arms = (['R3910', 'R3911'], ['R3913', 'R3914'])          # relay, fixed formation (R3912, the 1.15 kg relay, out)
    lap = lambda rr, f: mean([f([1e3 * l['path_rms_m'] for l in d[r]['laps']]) for r in rr])  # noqa: E731
    val = lambda rr, k: mean([d[r][k] for r in rr])  # noqa: E731
    rows = [('Flight time (s)', [f"{val(rr, 'mission_s'):.0f}" for rr in arms]),
            ('Battery budgets', [f"{val(rr, 'mission_over_b'):.2f}" for rr in arms]),
            ('Longest flight on one battery (s)', [f"{val(rr, 'max_pack_s'):.0f}" for rr in arms]),
            ('Largest tilt at an event ($^\\circ$)', [f"{val(arms[0], 'event_tilt_max_deg'):.2f}", 'N/A']),
            ('Contour error, first lap (mm)', [f"{lap(rr, lambda v: v[0]):.1f}" for rr in arms]),
            ('Contour error, last lap (mm)', [f"{lap(rr, lambda v: v[-1]):.1f}" for rr in arms]),
            ('Contour error, worst lap (mm)', [f"{lap(rr, max):.1f}" for rr in arms])]
    return [f'{name} & {a} & {b} \\\\ \\hline' for name, (a, b) in rows]


def compare():
    q = load('x11_quan_2026-11-07.json')
    x8 = {r['run']: r for r in load('x8_2026-11-07.json')['rows']}
    x8n = {r['run']: r for r in load('x8_hub_noxy_2026-11-07.json')['rows']}
    fe = lambda v: f'{v:.1f}' if v < 100 else f'{v:.0f}'  # noqa: E731
    ft = lambda v: f'{v:.2f}' if v < 1 else f'{v:.1f}'  # noqa: E731
    out = []
    for w, cab in (('rim', 'Rim'), ('hub', 'Hub')):
        for cell, name, err in (('hover', 'Hover', 'err_rms_mm'), ('linex_v200', 'Line, 0.2\\,m/s', 'headline_mm'),
                                ('linex_v400', 'Line, 0.4\\,m/s', 'headline_mm')):
            c = q[w]['cells'][cell]
            out.append(f"{name} & {cab} & {fe(c['planner'][err]['mean'])} & {ft(c['planner']['tilt_peak_deg']['mean'])} & "
                       f"{fe(c['port'][err]['mean'])} & {ft(c['port']['tilt_peak_deg']['mean'])} \\\\ \\hline")
    det = lambda src, rr, k: mean([src[r][k] for r in rr])  # noqa: E731
    for name, cab, ours, src, port in (('Detachment, estimate off', 'Rim', ['R4002', 'R4003'], x8, ['R4004', 'R4005']),
                                       ('Detachment, estimate on', 'Centre', ['R4008', 'R4009'], x8, ['R4010', 'R4011']),
                                       ('Detachment, estimate off', 'Centre', ['R4200', 'R4201'], x8n,
                                        ['R4010', 'R4011'])):
        out.append(f"{name} & {cab} & {det(src, ours, 'E_mm'):.0f} & {det(src, ours, 'tilt_abs_peak'):.1f} & "
                   f"{det(x8, port, 'E_mm'):.0f} & {det(x8, port, 'tilt_abs_peak'):.1f} \\\\ \\hline")
    c = q['world_c']['cells']['hover']['quan']
    out.append(f"Per-robot law as printed, hover & Centre & N/A & N/A & {c['err_rms_mm']['mean']:.0f} & "
               f"{c['tilt_peak_deg']['mean']:.1f} \\\\ \\hline")
    return out


def tilt_away_sim(rid):
    (d,) = glob.glob(os.path.join(REPO, 'results', '2026-11-05', f'{rid}_*'))
    r = pd.read_csv(os.path.join(d, 'logs', 'run.csv'), usecols=['t', 'payload_tilt_deg']).drop_duplicates('t')
    ev = pd.read_csv(os.path.join(d, 'logs', 'events.csv'))
    ev = ev[pd.to_numeric(ev.sim_time, errors='coerce').notna()]
    g = lambda k: float(ev.loc[ev.event == k, 'sim_time'].astype(float).iloc[0])  # noqa: E731
    m = (r.t >= g('RELEASED')) & (r.t <= g('REWELD'))
    return float(r.payload_tilt_deg[m].mean()), float(r.payload_tilt_deg[m].median())


def tilt_away_rig():
    base = os.path.join(REPO, 'results', 'rig', '2026-11-05')
    log = open(os.path.join(base, 'sq8_2.log')).read().splitlines()
    stamp = lambda pat, k=0: float(re.search(r'\[(\d+\.\d+)\]', [l for l in log if re.search(pat, l)][k]).group(1))  # noqa: E731
    t_off, t_n4 = stamp(r'ELRS magnet OFF: /drone_3'), stamp(r'FLEET RESIZED to n=4', 1)
    r = pd.read_csv(os.path.join(base, 'sq8_2_run', 'logs', 'run.csv'), usecols=['t', 'payload_tilt_deg'])
    ev = list(csv.DictReader(open(os.path.join(base, 'sq8_2_run', 'logs', 'events.csv')), fieldnames=['t', 'e', 'a']))
    t0 = float([e['t'] for e in ev if e['e'] == 'DETACH'][0])     # the events file's DETACH is the magnet off
    m = (r.t >= t0) & (r.t <= t0 + t_n4 - t_off)
    return float(r.payload_tilt_deg[m].mean()), float(r.payload_tilt_deg[m].median()), t_n4 - t_off


def both():
    g = load('x11_gap_2026-11-07.json')['runs']
    rr = ['R4032', 'R4033']
    sims = {r: tilt_away_sim(r) for r in ('R3708', 'R3712', 'R3714', 'R3715')}
    rig = tilt_away_rig()
    print('  layout (e) while away, sim (mean, median):', {k: (round(a, 2), round(b, 2)) for k, (a, b) in sims.items()},
          ' rig:', tuple(round(x, 2) for x in rig))
    return [f"rise {mean([g[r]['dtheta'] for r in rr]):.1f}, detection "
            f"{mean([g[r]['detection_s_planner_log_10Hz'] for r in rr]):.2f}, recovery 2.63 (F077)",
            f"layout (e): rig mean {rig[0]:.1f}, sim mean of 4 {mean([a for a, _ in sims.values()]):.1f}"]


def descents():
    rows = list(csv.DictReader(open(os.path.join(A, 'rig_joins_2026-11-04_05', 'joins.csv'))))
    g = {}
    for r in rows:
        if r['eval'] in ('plate height', 'target height'):
            g.setdefault((r['kind'], r['formation'], r['ring_motion']), []).append(float(r['tip_xy_cm']))
    return [f'{k}: n {len(v)}, mean {np.mean(v):.1f}' for k, v in sorted(g.items())]


if __name__ == '__main__':
    for name, f in (('tab:res_sim_fixed', sim_fixed), ('tab:res_gap', gap), ('tab:res_join', join), ('tab:res_rejoin', rejoin), ('tab:res_s8', s8), ('tab:res_x3b', x3b),
                    ('tab:res_x4', x4), ('tab:res_compare', compare), ('tab:res_both', both),
                    ('tab:res_descents', descents)):
        print(f'%% {name}')
        print('\n'.join(f()))

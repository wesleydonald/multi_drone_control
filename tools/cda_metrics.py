#!/usr/bin/env python3
"""tools/cda_metrics.py -- the carriers_down_alone runs, scored for the card's bars H1-H5
(docs/experiments/2026-11-05_carriers_down_alone.md, section 6).

    python3 tools/cda_metrics.py R3755 R3756 ...     # B and D runs; pairs are matched by scenario (cda_<arm>_<scen>)

Per run, from run.csv / events.csv (sim time) and launch.log (the planner's touchdown and release lines, the
manager's disarm-alone lines; their wall stamps are put on sim time through the LAND, TAKE_BACK, disarm and LANDED
anchors). A carrier's rest is its median z over the first 20 samples; its disarm is its first armed -> 0 after LAND.
"""
import csv
import os
import re
import sys

import numpy as np
import pandas as pd

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from departed_metrics import run_dir  # noqa: E402

STAMP = re.compile(r'\[(\d{10}\.\d+)\]')
FAULTS = ('FLEET ABORT', 'EMERGENCY STOP', 'ENVELOPE FAULT', 'DROP AND LAND', 'UNANNOUNCED DETACH', 'POSITION LIMIT')
STILL = 0.05            # m/s, CARRIERS_STILL_MPS
STILL_S = 0.5           # s, CARRIERS_STILL_S
BAND = (-0.03, 0.15)    # m over rest, the carriers' band


def _lines(d):
    out = []
    for ln in open(os.path.join(d, 'logs', 'launch.log'), errors='replace'):
        m = STAMP.search(ln)
        if m:
            out.append((float(m.group(1)), ln.rstrip()))
    return out


def _first(lines, text):
    return next((t for t, ln in lines if text in ln), None)


def _touchdown_sim(d, t_land):
    """The touchdown on the runner's clock: the planner log's lift_progress freezes there (its own clock, offset by
    its LAND tick)."""
    import glob
    f = glob.glob(os.path.join(d, 'logs', 'dissipative_planner', '*', 'log.csv'))
    if not f or t_land is None:
        return None
    pl = pd.read_csv(f[0])
    w = pl[pl.land == 1]
    if w.empty:
        return None
    off = float(w.sim_time.iloc[0]) - t_land
    lp, ts = w.lift_progress.to_numpy(), w.sim_time.to_numpy()
    for i in range(1, len(lp) - 5):
        if np.all(lp[i:i + 6] == lp[i - 1]):
            return float(ts[i - 1]) - off
    return None


def _earliest_still(df, t0, carriers):
    """First time after t0 at which every carrier and the ring have been at most STILL for STILL_S (run.csv)."""
    w = df[df.t >= t0]
    if len(w) < 3:
        return None
    tt = w.t.to_numpy()
    v = [np.linalg.norm(w[[f'd{i}_vx', f'd{i}_vy', f'd{i}_vz']].to_numpy(), axis=1) for i in carriers]
    v.append(np.linalg.norm(np.gradient(w[['payload_x', 'payload_y', 'payload_z']].to_numpy(), tt, axis=0), axis=1))
    still = np.all(np.vstack(v) <= STILL, axis=0)
    start = None
    for k, ok in enumerate(still):
        start = (tt[k] if start is None else start) if ok else None
        if start is not None and tt[k] - start >= STILL_S:
            return float(tt[k])
    return None


def score(rid):
    d = run_dir(rid)
    name = re.sub(r'^R\d{4}_sim_gz_', '', os.path.basename(d))
    m = re.match(r'cda_([abd])_(\w+)$', name)
    arm, scen = (m.group(1), m.group(2)) if m else ('?', name)
    ev = [e for e in csv.DictReader(open(os.path.join(d, 'logs', 'events.csv'))) if e['sim_time'] != 'nan']
    t_of = {}
    for e in ev:
        t_of.setdefault(e['event'], float(e['sim_time']))
    df = pd.read_csv(os.path.join(d, 'logs', 'run.csv')).drop_duplicates('t').reset_index(drop=True)
    t = df.t.to_numpy()
    n = len([c for c in df.columns if re.match(r'd\d+_armed$', c)])
    lines = _lines(d)
    t_land, t_landed = t_of.get('LAND'), t_of.get('LANDED')
    out = dict(run=rid, config=name, arm=arm, scen=scen, t_land=t_land, t_landed=t_landed)

    # wall stamps -> sim time
    anchors = [(_first(lines, '[planner] LAND - descending'), t_land),
               (_first(lines, 'Landed - disarming the fleet'), t_landed)]
    if 'TAKE_BACK' in t_of:
        anchors.append((_first(lines, '[dissipative] TAKE-BACK of'), t_of['TAKE_BACK']))
    disarm = {}
    rest = {}
    for i in range(n):
        rest[i] = float(df[f'd{i}_z'].iloc[:20].median())
        a = df[f'd{i}_armed'].to_numpy() > 0.5
        k = np.nonzero((t >= t_land) & ~a)[0] if t_land is not None else []
        disarm[i] = float(t[k[0]]) if len(k) else None
        w = _first(lines, f'Drone {i + 1} is down at its rest on the landed ring')
        if w is not None and disarm[i] is not None:
            anchors.append((w, disarm[i]))
    out['t_td'] = _touchdown_sim(d, t_land)
    anchors.append((_first(lines, '[planner] touchdown by'), out['t_td']))
    anchors = sorted((a, b) for a, b in anchors if a is not None and b is not None)
    wx, sx = np.array([a for a, _ in anchors]), np.array([b for _, b in anchors])

    def sim(wall):
        if wall is None or len(wx) < 2:
            return None
        return float(np.interp(wall, wx, sx, left=np.nan, right=np.nan)) if wx[0] <= wall <= wx[-1] else \
            float(sx[0] + (wall - wx[0]) * (sx[-1] - sx[0]) / (wx[-1] - wx[0]))

    td_line = next((ln for _, ln in lines if '[planner] touchdown by' in ln), '')
    out['exit'] = 'stall' if 'touchdown by stall' in td_line else 'max drop' if 'max drop' in td_line else '-'
    out['td_heights'] = ', '.join(re.findall(r'\(([+-]+?[\d.]+)\)', td_line))
    out['waited'] = any('waiting for the departed' in ln for _, ln in lines)
    out['t_release'] = sim(_first(lines, '[planner] carriers_down_alone: ring down'))
    out['held'] = any('carriers held armed while' in ln for _, ln in lines)
    out['quad4_down'] = any('departed drone 4' in ln and 'is down' in ln for _, ln in lines)
    out['not_down_15s'] = any('not down after' in ln for _, ln in lines)
    out['faults'] = {f: sum(f in ln for _, ln in lines) for f in FAULTS}

    # the ring: touchdown (first sample after LAND within 5 mm of its lowest), its xy from the release to LANDED
    after = df[df.t >= t_land]
    zmin = float(after.payload_z.min())
    out['t_ring_down'] = float(after.t[after.payload_z <= zmin + 0.005].iloc[0])
    t0 = out['t_release'] if out['t_release'] is not None else out['t_td']
    seg = df[(df.t >= (t0 or t_landed)) & (df.t <= t_landed)]
    xy = seg[['payload_x', 'payload_y']].to_numpy()
    out['ring_xy_cm'] = 100.0 * float(np.max(np.linalg.norm(xy - xy[0], axis=1))) if len(xy) else float('nan')

    # the earliest the planner could release (run.csv, every carrier and the ring still for STILL_S after the
    # touchdown) against the release line on the same clock (wall stamps mapped: ~0.1-0.2 s)
    carriers = [i for i in range(n) if i < 3 or scen == 'nr9']           # QUAD4 is a carrier when folded in (S1)
    out['t_still'] = _earliest_still(df, out['t_td'], carriers) if out['t_td'] is not None else None

    # per carrier
    per = []
    for i in carriers:
        ti = disarm[i]
        alone = any(f'Drone {i + 1} is down at its rest on the landed ring' in ln for _, ln in lines)
        row = dict(d=i, alone=alone, t_disarm=ti)
        if ti is not None:
            k = int(np.argmin(np.abs(t - ti - 0.0)))
            kb = max(k - 1, 0)                                            # the last armed sample
            row['dz_cm'] = 100.0 * (float(df[f'd{i}_z'].iloc[kb]) - rest[i])
            row['speed'] = float(np.linalg.norm(df.loc[kb, [f'd{i}_vx', f'd{i}_vy', f'd{i}_vz']].to_numpy(float)))
            row['tilt'] = float(df[f'd{i}_tilt_deg'].iloc[kb])
            tail = df.iloc[kb:]
            row['tilt_after'] = float(tail[f'd{i}_tilt_deg'].max())
            p = tail[[f'd{i}_x', f'd{i}_y']].to_numpy()
            row['xy_after_cm'] = 100.0 * float(np.max(np.linalg.norm(p - p[0], axis=1)))
            row['td_to_disarm'] = None if out['t_td'] is None else ti - out['t_td']
        kL = int(np.argmin(np.abs(t - t_landed)))
        row['dz_landed_cm'] = 100.0 * (float(df[f'd{i}_z'].iloc[max(kL - 1, 0)]) - rest[i])
        per.append(row)
    out['carriers'] = per
    return out


def judge(b, dd):
    """H1-H5 for a D run against its scenario's B run: (bar, holds, numbers)."""
    res = []
    waits = dd['waited']
    if dd['scen'] == 'nr9':
        h1 = ('H1', None, 'S1: nothing waits')
    elif dd['exit'] != 'stall':
        h1 = ('H1', None, f'{dd["exit"]} exit: not scored')
    else:
        late = [c['td_to_disarm'] for c in dd['carriers']]
        ok = all(c['alone'] for c in dd['carriers']) and all(x is not None and x <= 3.0 for x in late)
        h1 = ('H1', ok, 'touchdown -> disarm alone ' + ', '.join('-' if x is None or not c['alone'] else f'{x:.2f}'
                                                              for x, c in zip(late, dd['carriers'])) + ' s')
    if dd['scen'] != 'nr9' and dd['exit'] == 'stall' and not all(c['alone'] for c in dd['carriers']):
        tops = [c.get('dz_cm') for c in dd['carriers']]
        within = [BAND[0] * 100 <= (x if x is not None else 999) <= BAND[1] * 100 for x in tops]
        if not any(within):
            h1 = ('H1', None, 'no carrier within its band at the release (' + h1[2] + ')')
    res.append(h1)
    alone = [c for c in dd['carriers'] if c['alone']]
    ok2 = all(BAND[0] * 100 <= c['dz_cm'] <= BAND[1] * 100 and c['speed'] <= STILL and c['tilt'] <= 10.0
              for c in alone) and (dd['t_release'] is None or (dd['t_still'] is not None
                                                                and dd['t_release'] >= dd['t_still'] - 0.2))
    res.append(('H2', ok2 if (alone or dd['t_release'] is not None) else None,
                'at disarm alone: dz ' + ', '.join(f'{c["dz_cm"]:+.1f}' for c in alone) + ' cm, speed max '
                + (f'{max(c["speed"] for c in alone):.3f}' if alone else '-') + ' m/s, tilt max '
                + (f'{max(c["tilt"] for c in alone):.1f}' if alone else '-') + ' deg; release '
                + ('-' if dd['t_release'] is None else f'{dd["t_release"] - dd["t_td"]:.2f} s after the touchdown, '
                   f'all still for 0.5 s from {dd["t_still"] - dd["t_td"]:.2f} s')
                + (f'; released with the carriers {dd["td_heights"]} m over their rests' if dd['t_release'] is not None and not alone
                   else '')
                + ('; held by a take-back' if dd['held'] else '')))
    if dd['scen'] == 'nr9':
        gap_b, gap_d = b['t_landed'] - b['t_ring_down'], dd['t_landed'] - dd['t_ring_down']
        dh = max(abs(x['dz_landed_cm'] - y['dz_landed_cm']) for x, y in zip(b['carriers'], dd['carriers']))
        ok3 = (dd['t_release'] is None and not alone and abs(gap_d - gap_b) <= 1.0 and dh <= 1.0)
        res.append(('H3', ok3, f'release line {"yes" if dd["t_release"] is not None else "no"}, disarmed alone '
                               f'{len(alone)}; ring down -> LANDED {gap_b:.2f} / {gap_d:.2f} s; heights at LANDED '
                               f'max diff {dh:.1f} cm'))
    else:
        res.append(('H3', None, 'not S1'))
    if dd['scen'] != 'nr9':
        lb, ld = b['t_landed'] - b['t_land'], dd['t_landed'] - dd['t_land']
        ok4 = dd['quad4_down'] and abs(ld - lb) <= 2.0 and not (dd['not_down_15s'] and not b['not_down_15s'])
        res.append(('H4', ok4, f'LAND -> LANDED {lb:.2f} / {ld:.2f} s; QUAD4 down line {b["quad4_down"]} / '
                               f'{dd["quad4_down"]}; 15 s timeout {b["not_down_15s"]} / {dd["not_down_15s"]}'))
    else:
        res.append(('H4', None, 'not a take-back'))
    new = [f for f in FAULTS if dd['faults'][f] > b['faults'][f]]

    def worst(r):                    # every carrier, from its disarm (alone or the fleet's) to the end of the log
        return (max((c.get('tilt_after', 0.0) for c in r['carriers']), default=0.0),
                max((c.get('xy_after_cm', 0.0) for c in r['carriers']), default=0.0))
    (tilt, slide), (tilt_b, slide_b) = worst(dd), worst(b)
    ok5 = not new and tilt <= 30.0 and slide <= 3.0 and (dd['t_release'] is None or dd['ring_xy_cm'] <= 2.0)
    res.append(('H5', ok5, f'new faults {new or "none"}; from each disarm to the end: tilt max {tilt:.1f} deg, xy '
                           f'max {slide:.1f} cm (B {tilt_b:.1f} deg, {slide_b:.1f} cm); ring xy from the release '
                           f'{dd["ring_xy_cm"]:.1f} cm'))
    return res


def main(argv):
    runs = [score(r) for r in argv]
    for r in runs:
        print(f"{r['run']} {r['config']}: exit {r['exit']} (touchdown heights {r['td_heights']}), waited {r['waited']}, "
              f"LAND {r['t_land']:.2f} ring down {r['t_ring_down']:.2f} touchdown "
              f"{'-' if r['t_td'] is None else format(r['t_td'], '.2f')} release "
              f"{'-' if r['t_release'] is None else format(r['t_release'], '.2f')} LANDED {r['t_landed']:.2f}; "
              f"faults {[f for f, k in r['faults'].items() if k]}")
        for c in r['carriers']:
            print(f"   d{c['d']}: disarm {'-' if c['t_disarm'] is None else format(c['t_disarm'], '.2f')} "
                  f"({'alone' if c['alone'] else 'fleet'}), dz {c.get('dz_cm', float('nan')):+.1f} cm, "
                  f"speed {c.get('speed', float('nan')):.3f}, tilt {c.get('tilt', float('nan')):.1f}, "
                  f"tilt after {c.get('tilt_after', float('nan')):.1f}, xy after {c.get('xy_after_cm', float('nan')):.1f} cm, "
                  f"dz at LANDED {c['dz_landed_cm']:+.1f} cm")
    by = {(r['scen'], r['arm']): r for r in runs}
    for scen in sorted({r['scen'] for r in runs}):
        b, dd = by.get((scen, 'b')), by.get((scen, 'd'))
        if b is None or dd is None:
            continue
        print(f'\n{scen}: B {b["run"]} vs D {dd["run"]}')
        for bar, ok, text in judge(b, dd):
            print(f'   {bar} {"-" if ok is None else "holds" if ok else "FAILS"}: {text}')


if __name__ == '__main__':
    main(sys.argv[1:] or ['R3755', 'R3756', 'R3757', 'R3758', 'R3759', 'R3760', 'R3761', 'R3762', 'R3763', 'R3764'])

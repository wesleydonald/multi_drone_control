#!/usr/bin/env python3
"""tools/refly_score.py -- thesis R (card docs/experiments/2026-10-23_refly_1m.md): each re-flown table scored against
its 0.55 m original in the original's own metric, with the original card's own bars.

    python3 tools/refly_score.py s R2500 R2501              # S per run (the appendix card's S1/S2)
    python3 tools/refly_score.py T7t tier_a                 # tables: metric per run at 0.55 m and 1 m, A, C, rows
    python3 tools/refly_score.py --all --out rows.csv

Per table (tools/thesis_runsets.py: its cells, the 0.55 m runs as cited, the r1m_ runs found):
- S, every 1 m run: no fault or abort line, no drop or unannounced detach the arm does not expect, LANDED reached, every
  drone disarmed at the end, planner solve failures <= 1;
- A, every cell: the 1 m mean within [0.5, 2] x the 0.55 m mean or within the floor (1.0 cm, 0.5 deg, 1.0 s); reported;
- C, the original card's claim with its frozen thresholds, computed on both sets (the 0.55 m line must reproduce the
  card's verdict; the 1 m line is the re-fly's).
Registry-style rows (one per 1 m run) go to --out, never to results/registry.csv; the tree and host are left to fill.
K1 (landing lean), Tier B and the push rows are not scored here (release_transient.py; no 1 m worlds).
"""
import argparse
import csv
import glob
import json
import math
import os
import re
import sys
import tempfile

import numpy as np
import pandas as pd

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, 'tools'))
import thesis_runsets as RS  # noqa: E402

FLOOR = {'cm': 1.0, 'mm': 10.0, 'deg': 0.5, 's': 1.0}
FAULT = re.compile(r'ENVELOPE FAULT|FLEET ABORT|EMERGENCY STOP|disarmed in flight')
EXPECT_DROP = ('dl_d1_second_loss', 'dl_d4_gap240')
DROP_IS_SUPPORT = ('ta_a6_', 'ta_a2_')       # Tier A: a planned-pull drop is T3/T8 support, scored ">=" (freeze 1.2)
DROP_TILT_DEG = 30.0
FALSE_TOUCHDOWN_CM = 10.0                    # a drone this high above rest at LANDED (0.55 m: <= 5.1; false 16-44)
# detection off, planned pull: the ring lands at 28 deg; 3 of 3 aborted in the LAND at 0.55 m (R1890/R1894/R1898),
# outside the scored window
LAND_ABORT_AS_FLOWN = ('ta_a6_',)
HEADER = ['run', 'date', 'config', 'card', 'variable', 'baseline', 'metric', 'value', 'repeat_of', 'verdict',
          'reviewer', 'notes']


def run_dir(rid):
    hits = glob.glob(os.path.join(RS.RESULTS, '2*', f'{rid}_*'))
    if len(hits) != 1:
        raise FileNotFoundError(f'{rid}: {len(hits)} run directories under {RS.RESULTS}')
    return hits[0]


def config_of(rid):
    return os.path.basename(run_dir(rid)).split('_sim_gz_', 1)[1]


def events(d):
    ev = list(csv.DictReader(open(os.path.join(d, 'logs', 'events.csv'))))
    first = {}
    for e in ev:
        first.setdefault(e['event'], float(e['sim_time']))
    return ev, first


def launch_log(d):
    p = os.path.join(d, 'logs', 'launch.log')
    return open(p, errors='replace').read() if os.path.exists(p) else ''


def solve_failures(d):
    logs = glob.glob(os.path.join(d, 'logs', '*planner', '*', 'log.csv'))
    if not logs:
        return 0
    L = pd.read_csv(logs[0])
    st = pd.to_numeric(L.solve_status, errors='coerce')
    return int(((L.phase == 'planner') & st.notna() & (st != 0)).sum())


def safety(rid):
    """S1/S2 for one run: {'ok', 'fails', 'notes', 'dropped'}. A fault after LAND fails S unless the arm did the same
    in every 0.55 m run (LAND_ABORT_AS_FLOWN): then it is a note."""
    d = run_dir(rid)
    cfg = config_of(rid)
    src = cfg[len(RS.PREFIX):] if cfg.startswith(RS.PREFIX) else cfg
    ev, first = events(d)
    log = launch_log(d)
    fails, notes = [], []
    faults = [ln for ln in log.splitlines() if FAULT.search(ln)]
    t_abort = min((float(e['sim_time']) for e in ev if 'ABORT' in e['event']), default=None)
    if faults or t_abort is not None:
        after_land = t_abort is not None and t_abort > first.get('LAND', math.inf)
        what = f"{'landing' if after_land else 'in-flight'} fault/abort ({len(faults)} lines)"
        (notes if after_land and src.startswith(LAND_ABORT_AS_FLOWN) else fails).append(what)
    dropped = 'DROP AND LAND' in log
    if dropped and not src.startswith(EXPECT_DROP + DROP_IS_SUPPORT):
        fails.append('drop not expected')
    if src in EXPECT_DROP and not dropped:
        fails.append('expected drop missing')
    if 'UNANNOUNCED DETACH' in log and 'RELEASE' not in first:
        fails.append('unannounced detach with no release')
    if 'LANDED' not in first:
        (notes if notes else fails).append('no LANDED')
    elif not dropped and 'LAND' in first:
        dz = m_land(rid).get('landed_dz_cm', 0.0)
        if dz > FALSE_TOUCHDOWN_CM:       # the appendix reviewer's correction (17 Oct, R2032c-R2045c)
            fails.append(f'false touchdown (a drone {dz:.0f} cm above its rest at LANDED)')
    run = pd.read_csv(os.path.join(d, 'logs', 'run.csv'))
    armed = [c for c in run.columns if re.fullmatch(r'd\d+_armed', c)]
    if armed and run[armed].iloc[-1].sum() > 0 and t_abort is None:
        fails.append('armed at the end')
    n_fail = solve_failures(d)
    if n_fail > 1:
        fails.append(f'{n_fail} solve failures')
    return dict(run=rid, ok=not fails, fails=fails, notes=notes, dropped=dropped)


def resolved(a, b, sign, floor):
    """Tier A's freeze 1.2 (the appendix card's 'resolved'): (1) sign(mean b - mean a) is the frozen sign (0: any),
    (2) the ranges do not overlap, (3) |gap| >= max(floor, 2 x the larger range)."""
    gap = float(np.mean(b) - np.mean(a))
    c1 = sign == 0 or np.sign(gap) == sign
    c2 = max(a) < min(b) or max(b) < min(a)
    need = max(floor, 2 * max(np.ptp(a), np.ptp(b)))
    return dict(gap=gap, need=need, c1=bool(c1), c2=bool(c2), c3=abs(gap) >= need,
                resolved=bool(c1 and c2 and abs(gap) >= need))


def agrees(old, new, floor):
    """A: the 1 m mean within [0.5, 2] x the 0.55 m mean, or within the floor of it; None without both."""
    if not len(old) or not len(new):
        return None
    o, n = float(np.mean(old)), float(np.mean(new))
    return bool(abs(n - o) <= floor or (o != 0 and 0.5 <= n / o <= 2.0))


# ── per-run metrics, each the original card's tool ───────────────────────────

def m_carry(rid):
    import n2_bands
    d = run_dir(rid)
    m = json.load(open(os.path.join(d, 'metrics.json')))
    err, tilt = n2_bands.path_metrics_clock_free(d)
    return dict(rmse_mm=1000 * m['payload_rmse_m'], tilt_deg=m['payload_tilt_peak_deg'],
                ratio=m['payload_radius_ratio'], cf_mm=1000 * err, cf_tilt_deg=tilt)


def m_traj(rid, cell):
    import thesis_sim_tables as T
    window = 'hover' if cell == 'hover' else 'lap' if cell.startswith('orbit') else 'all'
    s = T.traj_stats(rid, window)
    return dict(err_mean_cm=s['mean'], err_max_cm=s['max'], tilt_deg=s['tilt'])


def m_release(rid):
    """Delta-theta, event to +15 s (release_bands); a run that drops is scored at the drop (tilt past 30 deg), '>='."""
    import release_bands as rb
    r = rb.Run(rid)
    dropped = 'DROP AND LAND' in launch_log(run_dir(rid))
    hz = rb.BAND_HORIZON_S
    if dropped:
        past = np.flatnonzero((r.t >= r.te) & (np.linalg.norm(r.th, axis=1) >= DROP_TILT_DEG))
        hz = float(r.t[past[0]] - r.te) if len(past) else hz
    return dict(dtheta_deg=r.metrics(hz)['dtheta'], at_drop=dropped)


def m_t15(rid):
    import rig_night
    d = run_dir(rid)
    run = pd.read_csv(os.path.join(d, 'logs', 'run.csv')).drop_duplicates('t').reset_index(drop=True)
    return dict(exy_mean_cm=rig_night.carry_metrics(run, events(d)[1])['plus4_to_land']['exy_mean_cm'])


def _disarm_edges(run, n):
    out = {}
    for i in range(n):
        a = run[f'd{i}_armed'].to_numpy() > 0.5
        e = np.flatnonzero(a[:-1] & ~a[1:])
        if len(e):
            out[i] = (float(run.t.iloc[e[0] + 1]), float(run[f'd{i}_z'].iloc[e[0] + 1]), e[0] + 1)
    return out


def m_departed(rid):
    """P1-P4 (departed_disarm_alone, the 16 Oct reading: disarm edge minus first at rest) and Delta-theta."""
    import departed_metrics as dm
    d = run_dir(rid)
    ev, first = events(d)
    left = {int(e['arg']) for e in ev if e['event'] == 'RELEASE'}
    a = dm.analyse(rid)
    run = pd.read_csv(os.path.join(d, 'logs', 'run.csv')).drop_duplicates('t').reset_index(drop=True)
    edges = _disarm_edges(run, len(a['drones']))
    lt = a['landed_t']
    p1, p3 = [], []
    for i in left:
        v = a['drones'][i]
        if v.get('disarm_t') is None or v.get('at_rest_t') is None:
            p1.append(math.inf)
            continue
        ok = v['before_landed'] and abs(v['disarm_dz']) <= 0.03
        p1.append(v['disarm_t'] - v['at_rest_t'] if ok else math.inf)
        k = edges[i][2]
        p3.append(float(np.max(np.abs(run[f'd{i}_z'].iloc[k:] - v['rest']))))
    p4 = sum(1 for i, (t, _, _) in edges.items() if i not in left and (lt is None or t < lt))
    return dict(p1_s=max(p1) if p1 else math.inf, p3_m=max(p3) if p3 else math.inf, p4=p4,
                p2=not any('ABORT' in e['event'] for e in ev) and 'LANDED' in first, **m_release(rid))


def m_drop(rid):
    import drop_metrics
    a = drop_metrics.analyse(rid)
    run = pd.read_csv(os.path.join(run_dir(rid), 'logs', 'run.csv')).drop_duplicates('t').reset_index(drop=True)
    n = len([c for c in run.columns if re.fullmatch(r'd\d+_armed', c)])
    downs = list((a.get('down after (s)') or {}).values())
    return dict(dropped=a['dropped'], after_s=a.get('after last event (s)'), faults=a['fault count'],
                tilt_deg=max((a.get('drone tilt max (deg)') or {0: math.inf}).values()),
                climb_m=max((a.get('climb (m)') or {0: math.inf}).values()),
                down_s=max(downs) if downs and None not in downs else math.inf, landed=a.get('LANDED event', False),
                disarm_z_m=max((z for _, z, _ in _disarm_edges(run, n).values()), default=math.inf))


def m_method(rid):
    import method_metrics
    return method_metrics.metrics(rid)


def m_land(rid):
    import land_touchdown
    return land_touchdown.land(rid)


# ── tables ───────────────────────────────────────────────────────────────────

def _vals(per_run, cell, key):
    return [m[key] for m in per_run.get(cell, []) if m.get(key) is not None and np.isfinite(m[key])]


def c_carry(P):
    """Carry card bars 1-4 on the 22 Oct clock-free path-only ruler (push answer 8), every run; the 16 Oct metric
    against the same values beside."""
    out = []
    for cell, ms in P.items():
        for m in ms:
            cf = m['cf_mm'] <= 25 and m['cf_tilt_deg'] <= 6 and m['ratio'] >= 0.95
            old = m['rmse_mm'] <= 25 and m['tilt_deg'] <= 6 and m['ratio'] >= 0.95
            out.append((f'T7t {cell} {m["run"]}', cf and m['S'], f'clock-free {m["cf_mm"]:.1f} mm, '
                        f'{m["cf_tilt_deg"]:.2f} deg; 16 Oct metric {m["rmse_mm"]:.1f} mm, {m["tilt_deg"]:.2f} deg '
                        f'({"pass" if old else "miss"}); ratio {m["ratio"]:.3f}'))
    return out


def c_tier_a(P):
    arm = lambda a: [m['dtheta_deg'] for c, ms in P.items() if c.startswith(a) for m in ms]  # noqa: E731
    out = []
    for name, a, b, sign in (('T3 measured < planned (A1, A2)', 'A1', 'A2', 1),
                             ('T8 on < off (A2, A6)', 'A2', 'A6', 1)):
        if arm(a) and arm(b):
            r = resolved(arm(a), arm(b), sign, 1.0)
            out.append((name, r['resolved'], f"gap {r['gap']:+.2f} need {r['need']:.2f} "
                                             f"({r['c1']}/{r['c2']}/{r['c3']})"))
    if arm('A1') and arm('A5'):
        r = resolved(arm('A1'), arm('A5'), 0, 1.0)
        out.append(('T9 no sign (A1, A5)', not (r['c2'] and r['c3']), f"gap {r['gap']:+.2f} need {r['need']:.2f}"))
    drops = [m['run'] for ms in P.values() for m in ms if m['at_drop']]
    if drops:
        out.append(('drops scored ">=" at the drop', True, ', '.join(drops)))
    return out


def c_t15(P):
    a, b = _vals(P, 'A8', 'exy_mean_cm'), _vals(P, 'A7', 'exy_mean_cm')
    if not (a and b):
        return []
    r = resolved(a, b, 1, 0.3)
    return [('T15 planned < measured (A8, A7)', r['resolved'], f"gap {r['gap']:+.2f} cm need {r['need']:.2f}")]


BANDS_055 = {'W4': (1.75, 14.51), 'W5': (1.00, 13.20)}


def c_departed(P):
    out = []
    for cell, ms in P.items():
        for m in ms:
            ok = m['p1_s'] <= 0.5 and m['p2'] and m['p3_m'] <= 0.05 and m['p4'] == 0
            lo, hi = BANDS_055[cell]
            out.append((f'{cell} P1-P4 {m["run"]}', ok, f"P1 {m['p1_s']:.2f} s, P3 {m['p3_m']:.3f} m, P4 {m['p4']}; "
                        f"dtheta {m['dtheta_deg']:.2f} (0.55 m band [{lo}, {hi}] "
                        f"{'inside' if lo <= m['dtheta_deg'] <= hi else 'outside'}, reported)"))
    return out


def c_drop(P):
    out = []
    for cell, ms in P.items():
        for m in ms:
            fired = m['dropped'] and m['after_s'] is not None and 0.0 <= m['after_s'] <= 1.0
            ok = (fired and m['faults'] == 0 and m['tilt_deg'] <= 30 and m['climb_m'] <= 0.3 and m['down_s'] <= 15
                  and m['landed'] and m['disarm_z_m'] <= 0.2)
            out.append((f'{cell} v2 bars {m["run"]}', ok, f"drop {m['after_s']} s after the last loss, faults "
                        f"{m['faults']}, tilt {m['tilt_deg']}, climb {m['climb_m']}, down {m['down_s']} s, "
                        f"disarm z {m['disarm_z_m']:.2f} m"))
    return out


def c_x0(P):
    out = []
    h0, h1 = _vals(P, 'x0_hold_eps0', 'z_err_cm'), _vals(P, 'x0_hold_eps025', 'z_err_cm')
    if h0 and h1:
        r = resolved(h0, h1, 1, 1.0)
        out.append(('C-X1 relaxed box raises the hover >= 2 cm', r['resolved'] and r['gap'] >= 2.0,
                    f"gap {r['gap']:+.2f} cm need {max(2.0, r['need']):.2f}"))
    o0, o1 = _vals(P, 'x0_orbit_eps0', 'err_cm'), _vals(P, 'x0_orbit_eps025', 'err_cm')
    if o0 and o1:
        r = resolved(o0, o1, 1, 1.0)
        ratio = np.mean(o0) / np.mean(o1)
        out.append(('C-X2 pinning halves the orbit error', r['resolved'] and ratio <= 0.5,
                    f"ratio {ratio:.2f}, gap {r['gap']:+.2f} need {r['need']:.2f}"))
    return out


def c_robust(P):
    z = lambda c: _vals(P, c, 'z_err_cm')  # noqa: E731
    out = []
    r1 = [c for c in ('rob_none_int', 'rob_m+10_int', 'rob_m-20_int', 'rob_rod+5_int', 'rob_r+2_int') if z(c)]
    if r1:
        worst = max(abs(v) for c in r1 for v in z(c))
        out.append(('C-R1 integral removes the error (|z| <= 1 cm, every run)', worst <= 1.0, f'worst {worst:.2f} cm'))
    for arm in ('m+20', 'rod+5nomeas'):
        a, b = z(f'rob_{arm}_int'), z(f'rob_{arm}_z0')
        if a and b:
            g = abs(np.mean(a) - np.mean(b))
            out.append((f'C-R2 {arm}: integral within 1 cm of no integral', g <= 1.0, f'{g:.2f} cm'))
    a, b = z('rob_rod+5_z0'), z('x0_hold_eps0')
    if a and b:
        g = abs(np.mean(a) - np.mean(b))
        out.append(('C-R3 rod +5 cm measured: within 1 cm of X0-H0', g <= 1.0, f'{g:.2f} cm'))
    landed = [(m['run'], m.get('outcome') == 'landed' and m.get('tilt_deg', 90) <= 45)
              for ms in P.values() for m in ms if 'outcome' in m]
    if landed:
        bad = [r for r, ok in landed if not ok]
        out.append(('C-R4 every flight landed, no tip past 45 deg', not bad,
                    f'{len(landed) - len(bad)}/{len(landed)}' + (f' (not: {", ".join(bad)})' if bad else '')))
    return out


def c_land(P):
    L = [m for c, ms in P.items() if c != 'land_straight' for m in ms if 'outcome' in m]
    out = []
    if L:
        tips = [m['run'] for m in L if 'ABORT' in m['outcome'] or m.get('tilt_deg', 0) > 45]
        false = [m['run'] for m in L if m.get('landed_dz_cm', 0.0) > FALSE_TOUCHDOWN_CM]
        clean = [m for m in L if m['outcome'] == 'landed' and 'outside_cm' in m and m['tilt_deg'] <= 45]
        o = max((m['outside_cm'] for m in clean), default=math.nan)
        t = max((m['tilt_deg'] for m in clean), default=math.nan)
        out.append(('C-L1 default landing: 0 tips, outside <= 6.0 cm, tilt <= 8.0 deg (and no false touchdown, '
                    'the 17 Oct reviewer)', not tips and not false and o <= 6.0 and t <= 8.0,
                    f'{len(tips)} tips of {len(L)}{" (" + ", ".join(tips) + ")" if tips else ""}; '
                    f'{len(false)} false touchdowns{" (" + ", ".join(false) + ")" if false else ""}; outside max '
                    f'{o:.1f} cm, tilt max {t:.1f} deg'))
    ls, h0 = P.get('land_straight', []), P.get('x0_hold_eps0', [])
    if ls and h0:
        ref = max(m.get('outside_cm', math.nan) for m in h0)
        vals = [m.get('outside_cm', math.nan) for m in ls]
        out.append(('C-L2 straight descent pushes outward more than X0-H0', all(v > ref for v in vals),
                    f"L-S {' / '.join(f'{v:.1f}' for v in vals)} against {ref:.1f} cm"))
    return out


def c_mapneg(P):
    a, b = _vals(P, 'mapneg_model', 'err_cm'), _vals(P, 'mapneg_ref', 'err_cm')
    if not (a and b):
        return []
    r = resolved(a, b, 1, 1.0)
    return [('C-D5 model integral removes the thrust-map error', r['resolved'],
             f"model {np.mean(a):.2f} against reference {np.mean(b):.2f} cm, need {r['need']:.2f}")]


# scorer: (per-run metric, A keys with their floor unit, C)
SCORERS = {
    'carry': (m_carry, {'cf_mm': 'mm', 'rmse_mm': 'mm', 'tilt_deg': 'deg', 'cf_tilt_deg': 'deg'}, c_carry),
    'traj': (m_traj, {'err_mean_cm': 'cm', 'err_max_cm': 'cm', 'tilt_deg': 'deg'}, None),
    'tier_a': (m_release, {'dtheta_deg': 'deg'}, c_tier_a),
    't15': (m_t15, {'exy_mean_cm': 'cm'}, c_t15),
    'departed': (m_departed, {'dtheta_deg': 'deg', 'p1_s': 's'}, c_departed),
    'drop': (m_drop, {'tilt_deg': 'deg', 'down_s': 's'}, c_drop),
    'x0': (m_method, {'z_err_cm': 'cm', 'err_cm': 'cm'}, c_x0),
    'robust': (None, {'z_err_cm': 'cm'}, c_robust),
    'land': (m_land, {'outside_cm': 'cm', 'tilt_deg': 'deg'}, c_land),
    'mapneg': (m_method, {'err_cm': 'cm', 'z_err_cm': 'cm'}, c_mapneg),
}


def per_run(scorer, rid, cell):
    if scorer == 'robust':
        return {**m_method(rid), **{k: v for k, v in m_land(rid).items() if k in ('outcome', 'tilt_deg')}}
    if scorer == 'traj':
        return m_traj(rid, cell)
    return SCORERS[scorer][0](rid)


def collect(key, which):
    """{cell: [metrics + run + S]} for one run set; a run that cannot be read is listed with its error."""
    scorer = RS.TABLES[key]['scorer']
    cells = RS.runset(key, which)
    if scorer == 'tier_a':     # A1_r1, A1_r2 ... pooled per arm
        merged = {}
        for c, rr in cells.items():
            merged.setdefault(c.split('_')[0], []).extend(rr)
        cells = merged
    out = {}
    for cell, runs in cells.items():
        for rid in runs:
            try:
                m = dict(per_run(scorer, rid, cell), run=rid)
                m['S'] = safety(rid)['ok'] if which == '1m' else True
            except (Exception, SystemExit) as e:  # noqa: B902 -- a run that cannot be read is listed, not fatal
                m = dict(run=rid, error=str(e)[:120], S=False)
            out.setdefault(cell, []).append(m)
    return out


def score(key):
    scorer = RS.TABLES[key]['scorer']
    if scorer not in SCORERS:
        return None
    _, akeys, cfn = SCORERS[scorer]
    old, new = collect(key, 'old'), collect(key, '1m')
    agreement = []
    for cell in old:
        for k, unit in akeys.items():
            o, n = _vals(old, cell, k), _vals(new, cell, k)
            if o:
                agreement.append((cell, k, o, n, agrees(o, n, FLOOR[unit])))
    return dict(table=key, old=old, new=new, A=agreement,
                C_old=cfn(old) if cfn else [], C_new=cfn(new) if cfn and new else [])


def fmt_vals(v):
    return ' / '.join(f'{x:.2f}' for x in v) if v else '-'


def report(res):
    t = RS.TABLES[res['table']]
    print(f"\n== {res['table']}: {t['thesis']}\n   card {t['card']}")
    print('| cell | metric | 0.55 m | 1 m | A |\n|---|---|---|---|---|')
    for cell, k, o, n, ok in res['A']:
        print(f"| {cell} | {k} | {fmt_vals(o)} | {fmt_vals(n)} | {'-' if ok is None else 'agrees' if ok else 'MISS'} |")
    for label, rows in (('0.55 m (must reproduce the card)', res['C_old']), ('1 m', res['C_new'])):
        for name, ok, detail in rows:
            print(f"C {label}: {name}: {'CONFIRMED' if ok else 'NOT CONFIRMED'} ({detail})")
    errs = [m for ms in list(res['old'].values()) + list(res['new'].values()) for m in ms if 'error' in m]
    for m in errs:
        print(f"unread {m['run']}: {m['error']}")


def rows(res):
    """One registry-style row per 1 m run of the table."""
    t = RS.TABLES[res['table']]
    verdict_c = '; '.join(f"{n} {'CONFIRMED' if ok else 'NOT CONFIRMED'}" for n, ok, _ in res['C_new']
                          if not n.startswith(('T7t ', 'W4 ', 'W5 ', 'D1 ', 'D4 ')))
    out, first = [], {}
    for cell, ms in res['new'].items():
        for m in ms:
            d = run_dir(m['run'])
            cfg = config_of(m['run'])
            first.setdefault(cfg, m['run'])
            base = [r for c in t['cells'] if c['cell'] == cell or c['cell'].startswith(cell + '_') for r in c['old']]
            mine = [f"{'pass' if ok else 'FAIL'} ({detail})" for n, ok, detail in res['C_new'] if m['run'] in n]
            a = [f"{k} {'agrees' if ok else 'MISS'}" for c, k, _, n, ok in res['A'] if c == cell and ok is not None]
            vals = ', '.join(f'{k} {v:.3g}' for k, v in m.items() if isinstance(v, (int, float)) and k != 'S'
                             and not isinstance(v, bool))
            s = safety(m['run'])
            src = cfg[len(RS.PREFIX):] if cfg.startswith(RS.PREFIX) else cfg
            out.append([m['run'], os.path.basename(os.path.dirname(d)), cfg, f'{RS.CARD} (table {res["table"]})',
                        f'1 m re-fly of {src} ({cell})', ', '.join(base),
                        ', '.join(SCORERS[t['scorer']][1]), vals,
                        '' if first[cfg] == m['run'] else first[cfg],
                        '; '.join([f"S {'PASS' if s['ok'] else 'FAIL ' + ', '.join(s['fails'])}"]
                                  + (['A ' + ', '.join(a)] if a else []) + mine + ([verdict_c] if verdict_c else [])),
                        '', 'tree and host to fill; scored by tools/refly_score.py'])
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('what', nargs='*', help="'s RUN...' or table keys")
    ap.add_argument('--all', action='store_true', help='every table with a scorer')
    ap.add_argument('--out', default=os.path.join(tempfile.gettempdir(), 'refly_rows.csv'))
    o = ap.parse_args(argv)
    if o.what[:1] == ['s']:
        for rid in o.what[1:]:
            s = safety(rid)
            print(f"{rid}: S {'PASS' if s['ok'] else 'FAIL'} {', '.join(s['fails'] + s['notes'])}")
        return 0
    keys = [k for k in RS.TABLES if RS.TABLES[k]['scorer'] in SCORERS] if o.all else o.what
    bad = [k for k in keys if k not in RS.TABLES or RS.TABLES[k]['scorer'] not in SCORERS]
    if bad or not keys:
        ap.error(f'tables with a scorer: {", ".join(k for k in RS.TABLES if RS.TABLES[k]["scorer"] in SCORERS)}')
    out = []
    for k in keys:
        res = score(k)
        report(res)
        out += rows(res)
    with open(o.out, 'w', newline='') as fh:
        w = csv.writer(fh, lineterminator='\n')
        w.writerow(HEADER)
        w.writerows(out)
    print(f'\n{len(out)} rows -> {o.out} (review, then append to the registry by hand)')
    return 0


if __name__ == '__main__':
    sys.exit(main())

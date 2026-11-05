#!/usr/bin/env python3
"""tools/x8_metrics.py -- score X8, the paper's method through a loss (card docs/experiments/2026-10-30_x8_network_loss.md).

    python3 tools/x8_metrics.py R3592 R3593 R3636 ... [--json results/analysis/x8_<date>.json]

Cell and arm come from the run's name (x8_<cell>_<arm>: hub or rim; ours, ours_noxy, port, portu, quanu). Section 5's
metrics, as the reviewer's independent scorer computed them on 4 Nov (results/analysis/review_x2_x8_2026-11-04/rev_x8/
x8_check.py; t_e = RELEASE):
  E           the ring centre's largest 3-D distance from its mean over the 5 s before t_e, t_e .. t_e + 20 s (mm)
  dip         the ring's mean height over the 5 s before t_e minus its lowest in t_e .. t_e + 10 s, cut at LAND (mm)
  shift       the ring's horizontal mean over the last 5 s before LAND against the 5 s before t_e (mm)
  t_s         t_e to the last time the ring centre is outside max(20 mm, 3 sigma_pre) of its mean over the last 5 s
              before LAND (censored when still outside at LAND); sigma_pre the spread of its distance from the pre mean
  quad tilt   the survivors' peak attitude, t_e .. t_e + 10 s; ring tilt rise (from the 5 s mean) and abs peak to LAND
  loss        a fleet abort, an ENVELOPE FAULT line, DROP AND LAND, the ring under 0.3 m before LAND; rim: the ring's tilt
              over 80 deg on 2 samples (applied offline to every arm); hub: over 300 mm from the pre mean for 1 s
  leaver      its farthest distance from the ring, its peak speed, its largest distance in the first 1 s (validity)
  gap         the survivors' largest azimuth gap about the ring centre over the last 5 s before LAND
Bars (frozen 30 Oct 16:26): L1 (hub) and L2 (rim): CONFIRMED if in both port repeats E > ours_max + 20 mm or the port
loses the ring where ours does not; FALSIFIED if the port's mean E <= ours' and the port loses nothing; else
INCONCLUSIVE. L3: every ours run without a loss, and on the hub t_s <= 10 s. The never-told arms and the exact law's
smoke are reported, not barred. Validity (section 5) per run, from the same logs and s_metrics' late ticks.
"""
import argparse
import json
import os
import re
import sys

import numpy as np
import pandas as pd

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import release_bands as RB  # noqa: E402
import s_metrics as S  # noqa: E402

PRE_S, E_WIN_S, DIP_WIN_S, LAST_S = 5.0, 20.0, 10.0, 5.0
SETTLE_MIN_M = 0.020
HUB_LOSS_M, HUB_LOSS_S, RIM_LOSS_DEG = 0.300, 1.0, 80.0
L_MARGIN_MM, L3_SETTLE_S = 20.0, 10.0
LEAVER_OUT_M, OURS_RESIZE_S, PORT_DETACH_S, HANDOVER_MIN_S, ROWS_MIN = 1.06, 0.5, 0.6, 15.0, 0.95
ARMS = {'ours': 'ours', 'ours_noxy': 'ours', 'port': 'port', 'portu': 'portu', 'quanu': 'quanu'}


def cell_arm(name):
    m = re.search(r'x8_(hub|rim)_([a-z_]+?)$', name)
    return (m.group(1), ARMS.get(m.group(2), m.group(2))) if m else (None, None)


def score(d):
    name = os.path.basename(d.rstrip('/'))
    cell, arm = cell_arm(name)
    rim = cell == 'rim'
    df = pd.read_csv(f'{d}/logs/run.csv').drop_duplicates('t').reset_index(drop=True)
    ev = pd.read_csv(f'{d}/logs/events.csv')
    man = json.load(open(f'{d}/manifest.json'))
    txt = open(f'{d}/logs/launch.log', errors='replace').read()
    t = df.t.to_numpy(float)
    P = df[['payload_x', 'payload_y', 'payload_z']].to_numpy(float)
    q = df[['payload_qw', 'payload_qx', 'payload_qy', 'payload_qz']].to_numpy(float)
    ok = np.all(np.isfinite(q), 1)
    th = np.full((len(t), 2), np.nan)
    th[ok] = RB.tilt_vector(q[ok])
    mag = np.linalg.norm(th, axis=1)
    rel = ev[ev.event == 'RELEASE']
    te = float(rel.sim_time.iloc[0])
    leaver = int(float(rel.arg.iloc[0]))
    n_quads = len([c for c in df.columns if re.fullmatch(r'd\d+_x', c)])
    surv = [i for i in range(n_quads) if i != leaver]
    land = ev[ev.event == 'LAND'].sim_time
    tl = float(land.iloc[0]) if len(land) else None
    ab = ev[ev.event == 'FLEET_ABORT']
    t_ab = float(ab.sim_time.iloc[0]) if len(ab) else None
    tend = tl if tl is not None else (t_ab if t_ab is not None else t[-1])
    pre = (t >= te - PRE_S) & (t < te)
    mu = np.nanmean(P[pre], 0)
    dist = np.linalg.norm(P - mu, axis=1)
    sig_pre = float(np.nanstd(dist[pre]))
    wE = (t >= te) & (t <= te + E_WIN_S)
    kE = np.nanargmax(np.where(wE, dist, -1))
    aft = (t >= te) & (t <= min(te + DIP_WIN_S, tend))
    out = dict(run=name.split('_')[0], name=name.split('sim_gz_')[-1], cell=cell, arm=arm, leaver=leaver, te=te, tl=tl,
               t_abort=(t_ab - te) if t_ab else None, E_mm=1e3 * dist[kE], tE=t[kE] - te,
               dip_mm=1e3 * (np.nanmean(P[pre, 2]) - np.nanmin(P[aft, 2])), rtf=man.get('mean_rtf'), rows=len(t),
               diff=man.get('diff_sha256', '')[:8])
    if tl is not None:
        last = (t >= tl - LAST_S) & (t < tl)
        mul = np.nanmean(P[last], 0)
        out['shift_mm'] = 1e3 * float(np.linalg.norm(mul[:2] - mu[:2]))
        band = max(SETTLE_MIN_M, 3 * sig_pre)
        dl = np.linalg.norm(P - mul, axis=1)
        idx = np.nonzero((t >= te) & (t <= tl))[0]
        outside = idx[dl[idx] > band]
        out['ts'] = (t[outside[-1]] - te) if outside.size else 0.0
        out['ts_band_mm'] = 1e3 * band
        out['ts_cens'] = bool(outside.size and t[outside[-1]] >= t[idx[-1]] - 0.05)
    wq = (t >= te) & (t <= te + 10)
    out['quad_tilt_peak'] = float(np.nanmax(df[[f'd{i}_tilt_deg' for i in surv]].to_numpy(float)[wq]))
    wr = (t >= te) & (t <= tend)
    out['tilt_rise'] = float(np.nanmax(np.linalg.norm(th[wr] - np.nanmean(th[pre], 0), axis=1)))
    out['tilt_abs_peak'] = float(np.nanmax(mag[wr]))
    loss = []
    if t_ab is not None:
        loss.append(f'fleet abort {t_ab - te:+.2f}')
    if re.search(r'DROP AND LAND', txt):
        loss.append('DROP AND LAND')
    if re.search(r'ENVELOPE FAULT', txt):
        loss.append('ENVELOPE FAULT line')
    wl = (t > te) & (t < tend)
    if np.any(P[wl, 2] < 0.3):
        loss.append(f'ring z<0.3 at {t[wl][np.argmax(P[wl, 2] < 0.3)] - te:+.2f}')
    if rim:
        o = np.nan_to_num(mag) > RIM_LOSS_DEG
        k = np.nonzero(o & np.r_[o[1:], False] & wl)[0]
        if k.size:
            loss.append(f'tilt>80 2 samples at {t[k[0]] - te:+.2f}')
    else:
        o = (dist > HUB_LOSS_M) & wl
        best, start, bs, step = 0.0, None, None, np.median(np.diff(t))
        for i in range(len(t)):
            if o[i]:
                start = i if start is None else start
                if t[i] - t[start] + step > best:
                    best, bs = t[i] - t[start] + step, t[start] - te
            else:
                start = None
        out['over300_s'], out['over300_from'] = best, bs
        if best >= HUB_LOSS_S:
            loss.append(f'over 300 mm for {best:.2f} s from {bs:+.2f}')
    out['loss'] = loss
    pl = df[[f'd{leaver}_x', f'd{leaver}_y', f'd{leaver}_z']].to_numpy(float)
    vl = df[[f'd{leaver}_vx', f'd{leaver}_vy', f'd{leaver}_vz']].to_numpy(float)
    out['leaver_far_m'] = float(np.nanmax(np.linalg.norm(pl[wr] - P[wr], axis=1)))
    out['leaver_vmax'] = float(np.nanmax(np.linalg.norm(vl[wr], axis=1)))
    w1 = (t >= te) & (t <= te + 1.0)
    out['leaver_dist_1s_max'] = float(np.nanmax(np.linalg.norm(pl[w1] - P[w1], axis=1)))
    out['leaver_closest_survivor_m'] = float(min(np.nanmin(np.linalg.norm(
        pl[wr] - df[[f'd{i}_x', f'd{i}_y', f'd{i}_z']].to_numpy(float)[wr], axis=1)) for i in surv))
    if tl is not None:
        last = (t >= tl - LAST_S) & (t < tl)
        az = []
        for i in surv:
            pi = df[[f'd{i}_x', f'd{i}_y']].to_numpy(float)[last] - P[last, :2]
            az.append(np.degrees(np.arctan2(np.nanmean(pi[:, 1]), np.nanmean(pi[:, 0]))) % 360)
        az = sorted(az)
        out['gap_deg'] = float(max([b - a for a, b in zip(az[:-1], az[1:])] + [360 - az[-1] + az[0]]))
    out['unannounced'] = re.findall(rf'UNANNOUNCED DETACH: drone \d+ \(/drone_{leaver}\)[^\n]*', txt)[:1]
    out['detach_lines'] = re.findall(rf'DETACH drone \d+ \(/drone_{leaver}\)[^\n]*', txt)[:2]
    out['resize_ev'] = [float(x) - te for x in ev[ev.event == 'DETACH_RESIZE'].sim_time]
    out['detach_ev'] = [float(x) - te for x in ev[ev.event == 'DETACH'].sim_time]
    ho = ev[ev.event == 'NET_HANDOVER'].sim_time
    out['handover_before_s'] = (te - float(ho.iloc[0])) if len(ho) else None
    out['failed_lines'] = len(re.findall(r'(?i)solve failed|failed solve|SOLVER FAIL|status [1-9]', txt))
    out['slow'] = re.findall(r'SLOW TICK ([\d.]+) s', txt)
    out['events'] = ev.event.tolist()
    out['validity'] = validity(d, out, t, ev, txt, man)
    return out


def validity(d, o, t, ev, txt, man):
    """Section 5's validity items; 'reasons' empty means valid (a loss after t_e is an outcome, not void)."""
    sm = S.score(d)
    pl = sm.get('planner', {})
    late = max(v for v in (pl.get('late', 0.0), sm.get('tracker_late_max', 0.0)) if np.isfinite(v))
    step = float(np.median(np.diff(t)))
    rows_frac = len(t) / ((t[-1] - t[0]) / step + 1.0)
    before = [k for k, ts in zip(ev.event, ev.sim_time) if float(ts) < o['te'] and k in ('FLEET_ABORT', 'DROP')]
    why = []
    if o['leaver_dist_1s_max'] <= LEAVER_OUT_M:
        why.append(f"the joint did not let go (leaver {o['leaver_dist_1s_max']:.2f} m out in 1 s)")
    if o['arm'] == 'ours':
        if not o['unannounced']:
            why.append('no UNANNOUNCED DETACH line')
        if not o['resize_ev'] or o['resize_ev'][0] > OURS_RESIZE_S:
            why.append('planner not at n-1 within 0.5 s')
    else:
        if o['handover_before_s'] is None or o['handover_before_s'] < HANDOVER_MIN_S:
            why.append('NET_HANDOVER not 15 s before the release')
        if o['arm'] == 'port' and (not o['detach_ev'] or o['detach_ev'][0] > PORT_DETACH_S
                                   or not any('remain on the load' in ln for ln in o['detach_lines'])):
            why.append("the port's DETACH line not within 0.6 s")
        if o['arm'] in ('portu', 'quanu') and o['detach_ev']:
            why.append('a never-told arm was told (DETACH)')
    if before:
        why.append('abort or drop before the release')
    if pl.get('failed'):
        why.append('failed planner solve')
    if late > S.LATE_VOID:
        why.append('late ticks over 1 %')
    if rows_frac < ROWS_MIN:
        why.append(f'{100 * rows_frac:.0f} % of the rows')
    rb = {}
    pf = os.path.join(d, 'params', 'dissipative_planner.yaml')
    if os.path.isfile(pf):
        ptxt = open(pf).read()
        for k in ('mode', 'net_law', 'drop_gap_deg', 'reconfig_mode'):
            m = re.search(rf'^\s+{k}: (\S+)', ptxt, re.M)
            rb[k] = m.group(1) if m else None
    rb['drop_tilt_deg_trackers'] = sorted({m.group(1) for f in os.listdir(os.path.join(d, 'params'))
                                           if f.startswith('tracker_') for m in
                                           [re.search(r'drop_tilt_deg: (\S+)', open(os.path.join(d, 'params', f)).read())]
                                           if m}) if os.path.isdir(os.path.join(d, 'params')) else []
    return {'reasons': why, 'late_max': late, 'failed': pl.get('failed'), 'rows_frac': rows_frac, 'readback': rb}


def bars(rows):
    """L1 (hub), L2 (rim), L3 over the valid runs."""
    out = {}
    for cell, key in (('hub', 'L1'), ('rim', 'L2')):
        ours = [r for r in rows if r['cell'] == cell and r['arm'] == 'ours' and not r['validity']['reasons']]
        port = [r for r in rows if r['cell'] == cell and r['arm'] == 'port' and not r['validity']['reasons']]
        if not ours or not port:
            continue
        top = max(r['E_mm'] for r in ours)
        ours_loss = any(r['loss'] for r in ours)
        each = [r['E_mm'] > top + L_MARGIN_MM or (bool(r['loss']) and not ours_loss) for r in port]
        m_o, m_p = float(np.mean([r['E_mm'] for r in ours])), float(np.mean([r['E_mm'] for r in port]))
        conf, fals = all(each), m_p <= m_o and not any(r['loss'] for r in port)
        out[key] = {'cell': cell, 'ours': [r['run'] for r in ours], 'port': [r['run'] for r in port],
                    'ours_E_mm': [r['E_mm'] for r in ours], 'port_E_mm': [r['E_mm'] for r in port],
                    'bar_mm': top + L_MARGIN_MM, 'ours_mean_mm': m_o, 'port_mean_mm': m_p, 'each_port_worse': each,
                    'port_loss': [r['loss'] for r in port],
                    'verdict': 'CONFIRMED' if conf else 'FALSIFIED' if fals else 'INCONCLUSIVE'}
    ours = [r for r in rows if r['arm'] == 'ours' and not r['validity']['reasons']]
    if ours:
        miss = {r['run']: ([f'loss: {r["loss"]}'] if r['loss'] else [])
                + ([f"t_s {r.get('ts')}"] if r['cell'] == 'hub' and (r.get('ts') is None or r.get('ts_cens')
                                                                      or r['ts'] > L3_SETTLE_S) else [])
                for r in ours}
        out['L3'] = {'runs': miss, 'verdict': 'MISS' if any(miss.values()) else 'HOLDS'}
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('runs', nargs='+', help='R#### or run directories')
    ap.add_argument('--json', help='write the rows and the bars here')
    a = ap.parse_args()
    rows = []
    for r in a.runs:
        d = S.locate(r)
        if not os.path.isfile(os.path.join(d, 'logs', 'run.csv')):
            print(f'{r}: no logs/run.csv, not scored (void?)')
            continue
        rows.append(score(d))
    f = lambda v, nd=1: '-' if v is None else f'{v:.{nd}f}'  # noqa: E731
    print('| run | cell | arm | E (mm) | dip (mm) | shift (mm) | t_s (s) | quad tilt (deg) | ring rise / abs (deg) | loss '
          '| gap (deg) | leaver far, vmax, closest | valid |')
    print('|' + '---|' * 13)
    for o in rows:
        ts = '-' if o.get('ts') is None else f"{o['ts']:.2f}" + (' cens' if o.get('ts_cens') else '')
        print(f"| {o['run']} | {o['cell']} | {o['arm']} | {f(o['E_mm'])} | {f(o['dip_mm'])} | {f(o.get('shift_mm'))} "
              f"| {ts} | {f(o['quad_tilt_peak'])} | {f(o['tilt_rise'], 2)} / {f(o['tilt_abs_peak'], 2)} "
              f"| {'; '.join(o['loss']) or 'none'} | {f(o.get('gap_deg'), 0)} | {o['leaver_far_m']:.2f} m, "
              f"{o['leaver_vmax']:.2f} m/s, {o['leaver_closest_survivor_m']:.2f} m "
              f"| {'yes' if not o['validity']['reasons'] else 'VOID: ' + '; '.join(o['validity']['reasons'])} |")
    b = bars(rows)
    for k in ('L1', 'L2'):
        if k in b:
            x = b[k]
            print(f"{k} ({x['cell']}) {x['verdict']}: port E {', '.join(f'{v:.1f}' for v in x['port_E_mm'])} mm against "
                  f"ours_max + 20 = {x['bar_mm']:.1f} mm (ours {', '.join(f'{v:.1f}' for v in x['ours_E_mm'])}); means "
                  f"port {x['port_mean_mm']:.1f}, ours {x['ours_mean_mm']:.1f}; port losses {x['port_loss']}")
    if 'L3' in b:
        print(f"L3 {b['L3']['verdict']}: " + '; '.join(f"{k} {', '.join(v) or 'ok'}" for k, v in b['L3']['runs'].items()))
    if a.json:
        os.makedirs(os.path.dirname(os.path.abspath(a.json)), exist_ok=True)
        with open(a.json, 'w') as fh:
            json.dump({'card': 'docs/experiments/2026-10-30_x8_network_loss.md', 'rows': rows, 'bars': b}, fh,
                      indent=1, default=float)
        print('wrote', a.json)
    return 0


if __name__ == '__main__':
    sys.exit(main())

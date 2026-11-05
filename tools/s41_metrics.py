#!/usr/bin/env python3
"""tools/s41_metrics.py -- score the thesis's 4.1 fixed-team cells and 4.2's 4 -> 3 release in simulation with the tools
the real flights were scored with (feed F031, F033; Q007 items 1 and 2).

    python3 tools/s41_metrics.py R3489 ... R3508 [--json results/analysis/s41_<date>.json]

Fixed team (b<n>_hover / _fig8s075 / _linespinc075 runs): docs/figures/review_2026-10-23_best/best_data.py's score(), the
real flights' scorer (hover: the ring's 3-D distance to its reference from 2 s after the reference tops out to LAND; a
path: the clock-free distance to the path from its first move to the planner's descent, and the lag on the schedule
reported apart; its height part beside), the ring's tilt peak and mean over the same window, and on the spinny line
tools/yaw_track.py (the yaw error's RMS and delay). The table is F031's: one line per carrier count, two runs as a / b.
Release (r1m_ta_* runs, the 4 -> 3 unannounced release in hover): tools/x1_metrics.score() (detection, tilt rise from the
5 s mean, the absolute peak, release_bands' T6 back in band, the dip, the settled tilt) and the thesis table's back in
band (fig_reaction.py's reaction(), F033: the last time after the release the tilt is over the highest tilt of the 5 s
before it + 0.5 deg, looking to LAND or 15 s).
"""
import argparse
import json
import os
import re
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(REPO, 'docs', 'figures', 'review_2026-10-23_best'))
import best_data as B  # noqa: E402
import s_metrics as S  # noqa: E402
import x1_metrics as X  # noqa: E402

REACTION_BAND_DEG, REACTION_HORIZON_S = 0.5, 15.0


def reaction(f):
    """fig_reaction.py's reaction() (the thesis's 'back in band'), without its figure imports."""
    d = f.run()
    t = d.t.to_numpy()
    te = f.t_event('RELEASE')
    end = min(f.t_event('LAND', te) or np.inf, te + REACTION_HORIZON_S)
    tilt = d.payload_tilt_deg.to_numpy()
    band = float(np.nanmax(tilt[(t >= te - 5) & (t < te)])) + REACTION_BAND_DEG
    post = (t >= te) & (t <= end)
    over = np.flatnonzero(post & (tilt > band))
    return {'back_s': float(t[over[-1]] - te) if over.size else 0.0, 'peak_deg': float(np.nanmax(tilt[post])),
            'band_deg': band, 'window_s': end - te}


def score(d):
    rid = os.path.basename(d.rstrip('/')).split('_')[0]
    name = os.path.basename(d.rstrip('/')).split('_sim_gz_')[-1]
    f = B.Flight(rid)
    if 'r1m_ta' in name:
        x = X.score(d)
        return {'run': rid, 'kind': 'release', 'name': name, 'detect_s': x['detect_s'], 'resize_s': x['resize_s'],
                'tilt_rise_deg': x['tilt_rise'], 'abs_peak_deg': x['abs_peak'], 't6_back_s': x['t_back_t6'],
                'dip_mm': x['dip_mm'], 'settled_deg': x['settled_tilt'], 'dropped': x['dropped'], 'loss': x['loss'],
                'reaction': reaction(f), 'void_late': x['void_late']}
    m = re.match(r'(?:x11_)?b(\d)_', name)
    sc = B.score(f)
    tilt = f.run().payload_tilt_deg.to_numpy()
    out = {'run': rid, 'kind': f.motion, 'name': name, 'carriers': int(m.group(1)) if m else f.n,
           'err_mm': 1e3 * sc['err'], 'tilt_peak_deg': sc['tilt_peak'], 'tilt_mean_deg': float(np.nanmean(tilt[sc['w']]))}
    if f.motion != 'hover':
        out.update(lag_s=sc['lag_s'], dz_rms_mm=1e3 * sc['dz_rms'], across_rms_mm=1e3 * sc['across_rms'])
    if f.motion == 'spinny line':
        import yaw_track
        y = yaw_track.track(d)
        out.update(yaw_err_rms_deg=y['err_rms'], yaw_delay_s=y['delay_s'])
    sm = S.score(d)
    out['void_late'], out['dropped'] = sm['void'], sm['dropped']
    return out


def table(rows):
    ab = lambda v, nd: ' / '.join('-' if x is None else f'{x:.{nd}f}' for x in v) or '-'  # noqa: E731
    lines = ['| Carriers | Hover error (mm) | Figure-eight path error (mm), lag (s) | Spinny line path error (mm), yaw error '
             'RMS (deg), yaw delay (s) | Tilt peak, hover / figure-eight / spinny line (deg) |', '|---|---|---|---|---|']
    fixed = [r for r in rows if r['kind'] != 'release']
    for n in sorted({r['carriers'] for r in fixed}):
        k = lambda kind, key: [r[key] for r in fixed if r['carriers'] == n and r['kind'] == kind]  # noqa: E731
        lines.append(f"| {n} | {ab(k('hover', 'err_mm'), 1)} | {ab(k('figure-8', 'err_mm'), 1)}, "
                     f"{ab(k('figure-8', 'lag_s'), 2)} | {ab(k('spinny line', 'err_mm'), 1)}, "
                     f"{ab(k('spinny line', 'yaw_err_rms_deg'), 1)}, {ab(k('spinny line', 'yaw_delay_s'), 2)} | "
                     f"{ab(k('hover', 'tilt_peak_deg'), 2)}, {ab(k('figure-8', 'tilt_peak_deg'), 2)}, "
                     f"{ab(k('spinny line', 'tilt_peak_deg'), 2)} |")
    for kind in ('hover', 'figure-8', 'spinny line'):
        v = [r['tilt_mean_deg'] for r in fixed if r['kind'] == kind]
        h = [r['dz_rms_mm'] for r in fixed if r['kind'] == kind and 'dz_rms_mm' in r]
        if v:
            lines.append(f"{kind}: tilt mean {min(v):.2f}-{max(v):.2f} deg"
                         + (f", height RMS {min(h):.1f}-{max(h):.1f} mm" if h else ''))
    rel = [r for r in rows if r['kind'] == 'release']
    if rel:
        g = lambda key, nd=2: ' / '.join('-' if r[key] is None else f'{r[key]:.{nd}f}' for r in rel)  # noqa: E731
        lines.append(f"release ({', '.join(r['run'] for r in rel)}): detection {g('detect_s')} s; tilt rise {g('tilt_rise_deg')}"
                     f" deg; absolute peak {g('abs_peak_deg')} deg; back in band, release_bands' T6 {g('t6_back_s')} s, "
                     f"the thesis table's {' / '.join('%.2f' % r['reaction']['back_s'] for r in rel)} s; dip "
                     f"{g('dip_mm', 1)} mm; settled {g('settled_deg')} deg; loss {[r['loss'] for r in rel]}")
    voids = [r['run'] for r in rows if r['void_late']]
    if voids:
        lines.append(f"VOID (late ticks over 1 %): {', '.join(voids)}")
    return '\n'.join(lines)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('runs', nargs='+', help='R#### or run directories')
    ap.add_argument('--json', help="write every run's score here")
    a = ap.parse_args()
    rows = []
    for r in a.runs:
        d = S.locate(r)
        if not os.path.isfile(os.path.join(d, 'logs', 'run.csv')):
            print(f'{r}: no logs/run.csv, not scored (void?)')
            continue
        rows.append(score(d))
    print(table(rows))
    if a.json:
        os.makedirs(os.path.dirname(os.path.abspath(a.json)), exist_ok=True)
        with open(a.json, 'w') as fh:
            json.dump(rows, fh, indent=1, default=float)
        print('wrote', a.json)
    return 0


if __name__ == '__main__':
    sys.exit(main())

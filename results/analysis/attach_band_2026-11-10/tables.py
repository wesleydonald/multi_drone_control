#!/usr/bin/env python3
"""The markdown tables of summary.md, from the CSVs bands.py writes (run bands.py first).

    python3 results/analysis/attach_band_2026-11-10/tables.py      # -> tables.md
"""
import math
import os

import numpy as np
import pandas as pd

HERE = os.path.dirname(os.path.abspath(__file__))


def rd(fn):
    return pd.read_csv(os.path.join(HERE, fn))


def cm(v, sign=False):
    if v is None or (isinstance(v, float) and not math.isfinite(v)):
        return '-'
    return f'{100 * v:+.1f}' if sign else f'{100 * v:.1f}'


def table(head, rows):
    out = ['| ' + ' | '.join(head) + ' |', '|' + '---|' * len(head)]
    out += ['| ' + ' | '.join(str(c) for c in r) + ' |' for r in rows]
    return '\n'.join(out)


def main():
    D = rd('descents.csv')
    M = D[(D.day != '2026-10-29') & (D.frm == 'MATCH_VELOCITY')]
    L = []

    # ---- T1 every plate descent
    rows = []
    for _, r in D.iterrows():
        def at(f):
            st = r[f + '_status']
            if st != 'scored':
                return {'started below': 'started below', 'never reached': 'not reached'}.get(st, st), '', ''
            return (f"{cm(r[f + '_r'], True)} / {cm(r[f + '_s'], True)}",
                    'in' if r[f + '_inside'] == 'Y' else ('mouth' if r[f + '_mouth'] == 'Y' else 'out'), '')
        v4, v4c, _ = at('v4')
        s3, s3c, _ = at('sup35')
        rows.append([r.day[5:], r.flight, int(r.n), r.kind.replace('attach from the mat', 'mat').replace('re-attach', 're'),
                     'MATCH' if r.frm == 'MATCH_VELOCITY' else 'proof', r.outcome, int(r.held),
                     cm(r.match_xy), f"{r.match_dur:.2f}" if math.isfinite(r.match_dur) else '-',
                     cm(r.start_xy), cm(r.start_n), v4, v4c, s3, s3c,
                     f"{float(r.joins_tip_xy_cm):.1f}" if isinstance(r.joins_tip_xy_cm, (int, float)) and math.isfinite(r.joins_tip_xy_cm) else '-'])
    L.append('### T1. Every plate descent (cm; r radial + outward, s sideways + anticlockwise, ring plane)\n')
    L.append(table(['day', 'flight', '#', 'kind', 'from', 'outcome', 'holds', 'xy at MATCH entry', 'MATCH s',
                    'xy at descent start', 'tip height at start', 'v4 plane r / s', 'v4', 'sup. 35 mm plane r / s',
                    'sup. 35', 'xy at plate height (rig_joins)'], rows))

    # ---- T2 drift: start vs plane
    L.append('\n### T2. The descent start against the mouth plane (4-5 Nov, descents from MATCH that reached the plane)\n')
    rows = []
    for f, lab in (('v4', 'v4 (68 mm up)'), ('sup30', 'sup. 30 mm (13 mm up)'), ('sup35', 'sup. 35 mm (18 mm up)'),
                   ('sup40', 'sup. 40 mm (23 mm up)')):
        s = M[M[f + '_status'] == 'scored']
        rows.append([lab, len(s), f"{s[f + '_t'].median():.2f} ({s[f + '_t'].min():.2f}-{s[f + '_t'].max():.2f})",
                     f"{100 * s.start_xy.median():.1f}", f"{100 * s[f + '_xy'].median():.1f}",
                     f"{100 * np.sqrt((s[f + '_r'] ** 2).mean()):.1f} / {100 * np.sqrt((s[f + '_s'] ** 2).mean()):.1f}",
                     f"{np.corrcoef(s.start_r, s[f + '_r'])[0, 1]:+.2f} / {np.corrcoef(s.start_s, s[f + '_s'])[0, 1]:+.2f}",
                     f"{np.corrcoef(s.start_xy, s[f + '_xy'])[0, 1]:+.2f}",
                     f"{100 * np.sqrt(((s[f + '_r'] - s.start_r) ** 2).mean()):.1f} / "
                     f"{100 * np.sqrt(((s[f + '_s'] - s.start_s) ** 2).mean()):.1f}",
                     f"{(s[f + '_inside'] == 'Y').sum()}/{len(s)}", f"{(s[f + '_mouth'] == 'Y').sum()}/{len(s)}"])
    L.append(table(['plane', 'descents', 'start to plane, s: median (range)', 'xy at start, median', 'xy at plane, median',
                    'rms at plane r / s', 'corr start-plane r / s', 'corr of the distances', 'rms change r / s',
                    'fully inside', 'into the mouth'], rows))

    # ---- T3 horizon
    H = rd('horizon.csv')
    L.append('\n### T3. How long before the plane the offset still says where the disc arrives\n')
    rows = []
    for dt in sorted(H.dt.unique()):
        a = H[(H.funnel == 'v4') & (H.dt == dt)].iloc[0]
        b = H[(H.funnel == 'sup35') & (H.dt == dt)].iloc[0]
        rows.append([f'{dt:.2f}', f'{a.corr_r:+.2f} / {a.corr_s:+.2f}', f'{a.rms_change_r_cm:.1f} / {a.rms_change_s_cm:.1f}',
                     f'{b.corr_r:+.2f} / {b.corr_s:+.2f}', f'{b.rms_change_r_cm:.1f} / {b.rms_change_s_cm:.1f}'])
    L.append(table(['s before the plane', 'v4: corr r / s', 'v4: rms change r / s, cm', 'sup. 35: corr r / s',
                    'sup. 35: rms change r / s, cm'], rows))

    # ---- T4 start-gate selection
    L.append('\n### T4. Would a tighter start have arrived better? (the recorded descents grouped by their offset at the start)\n')
    rows = []
    for B in (0.03, 0.04, 0.05, 0.06, 0.08, 1.0):
        cells = ['all' if B == 1.0 else f'< {100 * B:.0f}']
        for f in ('v4', 'sup35'):
            s = M[(M[f + '_status'] == 'scored') & (M.start_xy < B)]
            cells += [len(s), f"{(s[f + '_inside'] == 'Y').sum()} ({100 * (s[f + '_inside'] == 'Y').mean():.0f} %)",
                      f"{(s[f + '_mouth'] == 'Y').sum()} ({100 * (s[f + '_mouth'] == 'Y').mean():.0f} %)"]
        rows.append(cells)
    L.append(table(['xy at descent start, cm', 'v4: descents', 'v4: fully inside', 'v4: into the mouth',
                    'sup. 35: descents', 'sup. 35: fully inside', 'sup. 35: into the mouth'], rows))

    # ---- T5 the MATCH wait
    W = rd('wait.csv')
    L.append('\n### T5. The MATCH wait by band and dwell (speed gate 0.20 m/s; all pools: 120.6 s, 78 segments)\n')
    rows = []
    A = W[W.pool == 'all']
    for B in sorted(A.B.unique()):
        for d in (0.3, 0.5, 1.0):
            w = A[(A.B == B) & (A.dwell == d)].iloc[0]
            per = []
            for p in ('hover 4-5 Nov', 'hover 29 Oct', 'descent 4-5 Nov'):
                x = W[(W.pool == p) & (W.B == B) & (W.dwell == d)]
                per.append(f"{x.iloc[0].frac_in:.2f}" if len(x) else '-')
            wl = '-' if not math.isfinite(w.wait_s_lo) else f'{w.wait_s_lo:.1f}'
            wh = 'never seen' if not math.isfinite(w.wait_s_hi) else f'{w.wait_s_hi:.1f}'
            rows.append([f'{100 * B:.0f}', d, ' / '.join(per), int(w.windows), int(w.cut_short), f'{w.longest_run_s:.2f}',
                         f'{w.windows_per_min_lo:.1f}-{w.windows_per_min_hi:.1f}', f'{wl}-{wh}',
                         f'{100 * w.p_none_30_lo:.0f}-{100 * w.p_none_30_hi:.0f}',
                         f'{100 * w.p_none_60_lo:.0f}-{100 * w.p_none_60_hi:.0f}'])
    L.append(table(['match_xy, cm', 'dwell, s', 'time inside the band: hover 4-5 Nov / hover 29 Oct / descent',
                    'windows seen', 'runs cut short', 'longest run, s', 'windows per min', 'expected wait, s',
                    'no attempt in 30 s, %', 'no attempt in 60 s, %'], rows))

    # ---- T6 abort replay
    Ab = rd('abort_replay.csv')
    L.append('\n### T6. descent_hold OFF (a real abort): the recorded descents replayed to each mouth plane\n')
    rows = []
    for B in sorted(Ab.B.unique()):
        for deb in (0.0, 1.0):
            a = Ab[(Ab.B == B) & (Ab.debounce == deb) & (Ab.funnel == 'v4')].iloc[0]
            b = Ab[(Ab.B == B) & (Ab.debounce == deb) & (Ab.funnel == 'sup35')].iloc[0]
            rows.append([f'{100 * B:.0f}', deb, f'{a.aborted}/{a.scored}', f'{a.aborted_inside} / {a.aborted_mouth}',
                         f'{a.kept_inside}/{a.kept} / {a.kept_mouth}/{a.kept}', f'{b.aborted}/{b.scored}',
                         f'{b.aborted_inside} / {b.aborted_mouth}', f'{b.kept_inside}/{b.kept} / {b.kept_mouth}/{b.kept}'])
    L.append(table(['band, cm', 'debounce, s', 'v4: aborted', 'v4: of those, would have been fully inside / in the mouth',
                    'v4: kept, fully inside / in the mouth', 'sup. 35: aborted',
                    'sup. 35: of those, fully inside / in the mouth', 'sup. 35: kept, fully inside / in the mouth'], rows))

    # ---- T7 hold estimate
    E = rd('hold_estimate.csv')
    L.append('\n### T7. descent_hold ON (as flown): the estimated arrival for a hold band B (lag 0.28 s; range: lag 0.14-0.43 s)\n')
    rows = []
    for B in (0.10, 0.08, 0.06, 0.05, 0.04, 0.03):
        cells = [f'{100 * B:.0f}']
        for f in ('v4', 'sup30', 'sup35', 'sup40'):
            x = E[(E.funnel == f) & (np.isclose(E.B, B))]
            m = x[x.lag_case == 'main'].iloc[0]
            lo, hi = x.p_inside.min(), x.p_inside.max()
            mlo, mhi = x.p_mouth.min(), x.p_mouth.max()
            cells.append(f'{100 * m.p_inside:.0f} ({100 * lo:.0f}-{100 * hi:.0f}) / {100 * m.p_mouth:.0f} '
                         f'({100 * mlo:.0f}-{100 * mhi:.0f})')
        cells.append(f"{x.iloc[0].frac_time_in_band:.2f}")
        rows.append(cells)
    L.append(table(['band B, cm', 'v4: fully inside / mouth, %', 'sup. 30 mm', 'sup. 35 mm', 'sup. 40 mm',
                    'share of descent time inside B'], rows))
    L.append('\nMeasured as flown (B 0.10, 4-5 Nov): v4 fully inside 19/32, mouth 29/32; sup. 35 mm 6/29, 13/29; '
             'sup. 30 mm and 40 mm in T2.')

    # ---- T8 recommendation grid (selected rows)
    R = rd('recommend.csv')
    L.append('\n### T8. Attempts per minute and arrival per attempt (descent_abort_xy is raised to match_xy by his code)\n')
    rows = []
    pick = [(0.08, 0.5, 0.10), (0.08, 0.3, 0.10), (0.08, 0.5, 0.08), (0.06, 0.5, 0.06), (0.06, 0.3, 0.06), (0.05, 0.5, 0.05),
            (0.05, 0.3, 0.05), (0.04, 0.3, 0.04), (0.04, 0.5, 0.04), (0.03, 0.3, 0.03), (0.03, 0.5, 0.03)]
    for f in ('v4', 'sup35'):
        for Bm, d, Bh in pick:
            x = R[(R.funnel == f) & np.isclose(R.match_xy, Bm) & np.isclose(R.match_dwell, d) & np.isclose(R.descent_abort_xy, Bh)]
            if not len(x):
                continue
            x = x.iloc[0]
            rows.append([f, f'{Bm:.2f}', d, f'{Bh:.2f}', f'{x.match_wait_s:.1f}', f'{x.descent_to_plane_s:.1f}',
                         f'{x.attempt_s:.1f}', f'{x.attempts_per_min:.1f}', f'{100 * x.p_inside:.0f}', f'{100 * x.p_mouth:.0f}',
                         f'{x.inside_per_min:.1f}', f'{x.mouth_per_min:.1f}', f'{100 * x.p_no_attempt_30:.0f}'])
    L.append(table(['funnel', 'match_xy', 'match_dwell', 'descent_abort_xy', 'MATCH wait, s', 'descent to the plane, s',
                    'one attempt, s', 'attempts per min', 'fully inside, %', 'into the mouth, %', 'fully inside per min',
                    'into the mouth per min', 'no attempt in 30 s, %'], rows))

    # ---- checks
    G = rd('gate_check.csv')
    m = G[(G.kind == 'match') & (G.to == 'DESCEND_TO_ATTACHMENT')]
    m45 = m[m.day != '2026-10-29']
    d45 = (m45.recon_s - m45.his_s)
    h = G[G.kind == 'held']
    dh = (h.recon_s - h.his_s)
    hs = rd('hold_sinks.csv')
    hs8 = hs[hs.n_at_hold_cm > 8]
    L.append('\n### T9. Checks\n')
    L.append(table(['check', 'result'], [
        ['his MATCH -> DESCEND time vs the reconstructed gate, 4-5 Nov', f'{len(m45)} descents: reconstructed '
         f'{-d45.max():.2f}-{-d45.min():.2f} s earlier (his 30 Hz tick)'],
        ['the same, 29 Oct sq2_5 (his gates 0.05 m / 0.15 m/s / 1.0 s)', 'reconstructed 12.09 s, his 12.30 s (with the '
         'body speed only; the bag\'s plate twist left out)'],
        ['the same, 29 Oct sq5_2 (0.08 / 0.15 / 0.5)', '4 of 6 within 0.06 s; #4 1.6 s early, #6 not reproduced'],
        ['his logged hold xy vs the reconstructed xy (max of the last 0.15 s)', f'{len(h)} holds: median {dh.median():+.2f} cm '
         f'({dh.min():+.2f} to {dh.max():+.2f})'],
        ['the abort replay vs his real restarts, 29 Oct sq5_2 (0.08 m, one tick)', 'the 5 descents with tip samples: '
         'within 0.01-0.05 s (abort_check_sq5_2.csv)'],
        ['the tip\'s further sink after a hold starts (4-5 Nov, tip > 8 cm up)', f'{len(hs8)} holds: median '
         f'{hs8.sink_cm.median():.1f} cm ({hs8.sink_cm.min():.1f}-{hs8.sink_cm.max():.1f}), hold median {hs8.dur.median():.2f} s'],
        ['as-flown MATCH duration (0.08 / 0.5), 4-5 Nov, descents from MATCH', f'median {M.match_dur.median():.2f} s, mean '
         f'{M.match_dur.mean():.2f} s, max {M.match_dur.max():.2f} s (T5\'s estimate from a random start: 2.0-2.4 s)'],
        ['as-flown attempt (descent start to the next), 4-5 Nov', f'median {M.cycle_s.median():.1f} s ({int(M.cycle_s.notna().sum())} '
         f'pairs): {60 / M.cycle_s.median():.1f} per min'],
        ['tip height above the weld point at the descent start', f'median {100 * M.start_n.median():.0f} cm '
         f'({100 * M.start_n.min():.0f}-{100 * M.start_n.max():.0f}); approach_offset_z 30 cm'],
    ]))
    with open(os.path.join(HERE, 'tables.md'), 'w') as f:
        f.write('\n'.join(L) + '\n')
    print('wrote tables.md')


if __name__ == '__main__':
    main()

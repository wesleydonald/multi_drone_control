#!/usr/bin/env python3
"""tools/thesis_join_figures.py -- the join chapter's figures (thesis plan of 11 Oct, section 4):

    python3 tools/thesis_join_figures.py proof     # F14 the contact-and-lift proof in the twin

Run ids come from configs/thesis_figures.yaml (tools/figure_runs.py); outputs go to docs/figures/<ID>/ at print size.

Brief (thesis-figure skill, step 1), F14:
    Sentence: the catch is proven from motion capture alone, in under a second, by the plate rising with the tip
    while the tip keeps the vector it touched with; authority switches on the proof tick and the newcomer's throttle
    does not step. Hero: the heights relative to the contact (the newcomer's lift, the tip, the plate point), with the
    proof thresholds drawn. Support: the tip's drift from its contact vector, the newcomer's throttle across the
    switch, the ring's tilt. Left out: his descent, the fold-in's later blend, the landing. Window: 1 s before the
    contact to 6 s after it. Two runs of the same arm, aligned on the contact.
"""
import csv
import os
import sys

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import plot_style as ps                      # noqa: E402
import figure_runs as fr                     # noqa: E402
import release_bands as rb                   # noqa: E402
from thesis_leave_figures import load_run, rotm, GREY_TEXT   # noqa: E402

OUT = os.path.join(fr.REPO, 'docs', 'figures')


def launch_params(d):
    """The thresholds the manager logged at start (proof d_ok, d_fail, h_plate) and the plate's weld point."""
    import re
    out = {}
    for line in open(os.path.join(d, 'logs', 'launch.log'), errors='replace'):
        m = re.search(r'confirm_mode=contact_lift: contact ([\d.]+) m / ([\d.]+) m/s for ([\d.]+) s, proof d_ok=([\d.]+) '
                      r'd_fail=([\d.]+) h_plate=([\d.]+)', line)
        if m:
            out.update(radius=float(m[1]), speed=float(m[2]), dwell=float(m[3]), d_ok=float(m[4]), d_fail=float(m[5]),
                       h_plate=float(m[6]))
        m = re.search(r'ATTACH: proven \(try (\d+)\): drift ([\d.]+) mm, plate rise ([\d.]+) mm', line)
        if m:
            out.update(tries=int(m[1]), drift_mm=float(m[2]), rise_mm=float(m[3]))
    import json
    la = json.load(open(os.path.join(d, 'manifest.json'))).get('launch_args', {})
    if not isinstance(la, dict):
        la = dict(a.split(':=', 1) for a in la if ':=' in a)
    out['weld_point_z'] = float(la.get('weld_point_z', 0.0))
    out['attach_y_offset'] = float(la.get('attach_y_offset', 0.225))
    out['attach_x_offset'] = float(la.get('attach_x_offset', 0.0))
    out['proof_rise_max'] = float(la.get('proof_rise_max', 0.01))
    return out


def proof_trace(rid, newcomer=3):
    run, ev, d = load_run(rid)
    par = launch_params(d)
    t = run.t.to_numpy(float)
    tc = ev['CONTACT'][0]
    t_proof = ev['WELD'][0]
    t_mux = ev['MUX:ours'][0]
    t_fold = ev.get('FOLD_IN', [np.nan])[0]
    ring = run[['payload_x', 'payload_y', 'payload_z']].to_numpy(float)
    R = rotm(run[['payload_qw', 'payload_qx', 'payload_qy', 'payload_qz']].to_numpy(float))
    rho = np.array([par['attach_x_offset'], par['attach_y_offset'], par['weld_point_z']])
    plate = ring + np.einsum('nij,j->ni', R, rho)
    tip = run[['tip_x', 'tip_y', 'tip_z']].to_numpy(float)
    body = run[[f'd{newcomer}_x', f'd{newcomer}_y', f'd{newcomer}_z']].to_numpy(float)
    kc = int(np.searchsorted(t, tc))
    # the contact vector in the ring frame, and the tip's drift from it
    rel = np.einsum('nji,nj->ni', R, tip - plate)
    drift = np.linalg.norm(rel - rel[kc], axis=1)
    thr = run[f'd{newcomer}_thr'].to_numpy(float)
    q = run[['payload_qw', 'payload_qx', 'payload_qy', 'payload_qz']].to_numpy(float)
    th = rb.tilt_vector(q)
    pre = (t >= tc - rb.PRE_S) & (t < tc)
    dev = np.linalg.norm(th - th[pre].mean(0), axis=1)
    w = (t >= tc - 8.0) & (t <= tc + 8.0)
    k_hover = (t >= tc - 1.0) & (t < tc - 0.2)
    thr_hover = float(np.nanmean(thr[k_hover]))
    k_pre = (t >= t_mux - 0.1) & (t < t_mux)
    k_post = (t >= t_mux) & (t < t_mux + 0.3)
    step = float(np.nanmean(thr[k_post]) - np.nanmean(thr[k_pre]))
    win = (t >= tc) & (t <= t_mux + 8.0)
    stats = dict(contact_to_proof=t_proof - tc, proof_to_mux=t_mux - t_proof, fold_after_mux=t_fold - t_mux,
                 lift_body_mm=1e3 * float(body[(t >= tc) & (t <= t_proof), 2].max() - body[kc, 2]),
                 tip_rise_mm=1e3 * float(tip[(t >= tc) & (t <= t_proof), 2].max() - tip[kc, 2]),
                 plate_rise_mm=1e3 * float(plate[(t >= tc) & (t <= t_proof), 2].max() - plate[kc, 2]),
                 drift_at_proof_mm=1e3 * float(np.interp(t_proof, t, drift)),
                 thr_hover=thr_hover, thr_step_pct=100 * step / thr_hover,
                 tilt_rise=float(dev[win].max()), tilt_abs_peak=float(np.linalg.norm(th, axis=1)[win].max()),
                 dip_mm=1e3 * float(ring[kc, 2] - ring[win, 2].min()), logged=par)
    return dict(t=t[w] - tc, body=1e3 * (body[w, 2] - body[kc, 2]), tip=1e3 * (tip[w, 2] - tip[kc, 2]),
                plate=1e3 * (plate[w, 2] - plate[kc, 2]), drift=1e3 * drift[w], thr=thr[w], dev=dev[w],
                t_proof=t_proof - tc, t_mux=t_mux - tc, t_fold=t_fold - tc, par=par, stats=stats, rid=rid)


def fig_proof():
    ps.thesis()
    spec = fr.figure('F_proof_timeline')
    tr = [proof_trace(r) for r in spec['runs']]
    for d in tr:
        print(d['rid'], {k: (round(v, 3) if isinstance(v, float) else v) for k, v in d['stats'].items() if k != 'logged'},
              d['stats']['logged'])
    T0, T1 = -1.0, 6.0
    fig, axs = plt.subplots(4, 1, figsize=ps.size('full', height=4.0), sharex=True,
                            gridspec_kw=dict(height_ratios=[1.3, 0.9, 0.8, 0.9]))
    q4 = ps.drone_colour(3)
    styles = [dict(lw=1.2, alpha=1.0), dict(lw=0.9, alpha=0.7)]
    par = tr[0]['par']
    for d, st in zip(tr, styles):
        w = (d['t'] >= T0) & (d['t'] <= T1)
        t = d['t'][w]
        # (a) heights relative to the contact: body (newcomer), tip, plate
        axs[0].plot(t, d['body'][w], color=q4, zorder=3, **st)
        axs[0].plot(t, d['tip'][w], color='0.3', zorder=3, **st)
        axs[0].plot(t, d['plate'][w], color=ps.PAYLOAD_LINE, zorder=3, **st)
        # (b) drift, defined from the contact on
        wc = w & (d['t'] >= 0)
        axs[1].plot(d['t'][wc], d['drift'][wc], color='0.3', zorder=3, **st)
        # (c) throttle as a percentage of the hover throttle before the contact
        axs[2].plot(t, 100 * (d['thr'][w] / d['stats']['thr_hover'] - 1), color=q4, zorder=3, **st)
        # (d) ring tilt departure
        axs[3].plot(t, d['dev'][w], color=ps.PAYLOAD_LINE, zorder=3, **st)
        for ax in axs:
            ax.axvline(d['t_proof'], color='0.5' if st['alpha'] == 1.0 else '0.75', lw=0.6, zorder=1)
    # thresholds and labels
    a0 = axs[0]
    a0.axhline(1e3 * par['h_plate'], color='0.6', lw=0.6, ls=(0, (3, 2)), zorder=1)
    a0.annotate(f"plate rise that proves the catch, {1e3 * par['h_plate']:.1f}\\,mm", (T1, 1e3 * par['h_plate']),
                xytext=(-2, 2), textcoords='offset points', ha='right', va='bottom', fontsize=7, color=GREY_TEXT)
    a0.axhline(1e3 * par['proof_rise_max'], color='0.6', lw=0.6, ls=(0, (3, 2)), zorder=1)
    a0.annotate(f"the collaborator's lift target, +{1e3 * par['proof_rise_max']:.0f}\\,mm", (T1, 1e3 * par['proof_rise_max']),
                xytext=(-2, 2), textcoords='offset points', ha='right', va='bottom', fontsize=7, color=GREY_TEXT)
    d = tr[0]
    # the three series named once, in the empty corner before the contact
    for yy, word, col in ((-9, 'newcomer body', q4), (-16, 'magnet tip', '0.3'), (-23, 'plate point', ps.PAYLOAD_LINE)):
        a0.annotate(word, (T0 + 0.05, yy), fontsize=7, color=col, va='center', ha='left')
    a0.set_ylabel('Height above\nthe contact (mm)')
    a0.set_yticks([-20, -10, 0, 10])
    a0.set_ylim(-30, 16)
    a1 = axs[1]
    for key, lab in (('d_ok', 'kept within'), ('d_fail', 'lost at')):
        a1.axhline(1e3 * par[key], color='0.6', lw=0.6, ls=(0, (3, 2)), zorder=1)
        a1.annotate(f"{lab} {1e3 * par[key]:.0f}\\,mm", (T1, 1e3 * par[key]), xytext=(-2, 2), textcoords='offset points',
                    ha='right', va='bottom', fontsize=7, color=GREY_TEXT)
    a1.set_ylabel('Tip drift (mm)')
    a1.set_ylim(0, 12)
    a2 = axs[2]
    a2.set_ylabel('Newcomer throttle\n(\\% of hover)')
    a2.set_yticks([0, 5, 10])
    s = tr[0]['stats']
    steps = ' and '.join(f"{x['stats']['thr_step_pct']:+.1f}" for x in tr)
    a2.annotate(f"0.3\\,s mean after the switch against 0.1\\,s before: {steps}\\,\\%", (0.02, 0.95), xycoords='axes fraction',
                ha='left', va='top', fontsize=7, color=GREY_TEXT)
    a3 = axs[3]
    a3.axhline(5.0, color='0.6', lw=0.6, ls=(0, (3, 2)), zorder=1)
    a3.annotate('bar, 5$^\\circ$', (T1, 5.0), xytext=(-2, 2), textcoords='offset points', ha='right', va='bottom',
                fontsize=7, color=GREY_TEXT)
    a3.set_ylabel('Ring tilt\ndeparture (\\si{\\degree})')
    a3.set_ylim(0, 6.5)
    a3.set_xlabel('$t$ after the contact (s)')
    a3.set_xlim(T0, T1)
    for ax in axs:
        ax.axvline(0.0, color='0.5', lw=0.5, zorder=1)
    axs[0].annotate('contact', (0.0, 1.0), xycoords=('data', 'axes fraction'), xytext=(2, 2), textcoords='offset points',
                    fontsize=8, color=GREY_TEXT, ha='left', va='bottom')
    proofs = ' (dark) and '.join(f"{x['stats']['contact_to_proof']:.2f}" for x in tr) + ' (light)'
    axs[0].annotate(f"proven, the radio switched: {proofs}\\,s", (max(x['t_proof'] for x in tr), 1.0),
                    xycoords=('data', 'axes fraction'), xytext=(2, 2), textcoords='offset points', fontsize=8,
                    color=GREY_TEXT, ha='left', va='bottom')
    for ax, letter in zip(axs, 'abcd'):
        ps.panel_label(ax, letter)
    fig.align_ylabels(axs)
    paths = ps.save_print(fig, os.path.join(OUT, 'F_proof_timeline', 'proof_timeline'))
    print('\n'.join(paths))


FIGS = {'proof': fig_proof}

if __name__ == '__main__':
    for name in sys.argv[1:] or FIGS:
        FIGS[name]()

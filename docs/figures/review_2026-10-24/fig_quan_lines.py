#!/usr/bin/env python3
"""Quan et al.'s law against our planner on the centre-of-mass hub world (card docs/experiments/2026-10-23_quan_exact.md).

Message: on the 0.9 m out-and-back line our planner follows the schedule; the paper's law, as read, keeps the pace but runs
past each end; reading R7c (the path's acceleration fed forward once) takes most of that overshoot out.
Hero: the ring's position along the line against the schedule, from the path's start, two cycles; our planner blue, the
law red (R6c), R7c light red where flown, the schedule grey dashed. Left out: the port (it lags 1.1-1.2 s, in the table),
the sideways error, the hover. Data: R2806/R2807 (ours), R2814/R2815 (R6c), R2818 (R7c, line 0.4) via tools/metrics.load_run.
"""
import glob
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import review_style as S  # noqa: E402
import metrics as M  # noqa: E402

REPO = S.REPO
CASES = (('0.2 m/s', {'ours': 'R2806', 'law': 'R2814'}),
         ('0.4 m/s', {'ours': 'R2807', 'law': 'R2815', 'r7c': 'R2818'}))
WINDOW_S = {'0.2 m/s': 20.0, '0.4 m/s': 10.0}            # two out-and-back cycles


def track(rid):
    hits = glob.glob(os.path.join(REPO, 'results', '2026-10-2*', rid + '_*'))
    if not hits:
        return None
    d = M.load_run(hits[0])['data']
    ev = json.load(open(os.path.join(hits[0], 'metrics.json')))['events']
    t = np.asarray(d['t'], float) - float(ev['PATH_START'])
    x = np.asarray(d['payload_x'], float)
    i0 = np.searchsorted(t, 0.0)
    return t, x - x[i0], np.asarray(d['payload_ref_x'], float) - x[i0]


def main():
    S.setup()
    fig = S.figure('full', 2.5)
    axs = fig.subplots(1, 2, sharey=True)
    for ax, (case, runs) in zip(axs, CASES):
        w = WINDOW_S[case]
        t, x, ref = track(runs['ours'])
        m = (t >= 0) & (t <= w)
        ax.plot(t[m], ref[m], **S.REF)
        ax.plot(t[m], x[m], color=S.BLUE, lw=S.LW_HERO, label='our planner')
        t, x, _ = track(runs['law'])
        m = (t >= 0) & (t <= w)
        ax.plot(t[m], x[m], color=S.RED, lw=S.LW_HERO, label='Quan et al., as read')
        if 'r7c' in runs and track(runs['r7c']) is not None:
            t, x, _ = track(runs['r7c'])
            m = (t >= 0) & (t <= w)
            ax.plot(t[m], x[m], color=S.RED_LIGHT, lw=S.LW_HERO, label='Quan et al., acceleration once (R7c)')
        ax.axhline(0.9, color=S.LIGHT, lw=0.5, zorder=0)
        ax.axhline(0.0, color=S.LIGHT, lw=0.5, zorder=0)
        ax.set_xlim(0, w)
        ax.set_xlabel('Time from the path start (s)')
        S.panel(ax, 'ab'[CASES.index((case, runs))], f'line at {case}')
    axs[0].set_ylabel('Ring along the line (m)')
    h, l = axs[1].get_legend_handles_labels()
    axs[0].legend(h, l, loc='lower left', fontsize=7, frameon=False)
    S.save(fig, 'fig_quan_lines')


if __name__ == '__main__':
    main()

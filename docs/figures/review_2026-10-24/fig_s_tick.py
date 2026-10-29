#!/usr/bin/env python3
"""The planner's whole tick against fleet size, idle rig laptop (bench B, 24 Oct), at 5, 2 and 1 SQP iterations.

Message: the tick grows with the fleet; at 5 iterations eight carriers take ~256 ms against the 100 ms period, at 1
iteration ~83 ms. The grey dotted line is the rig projection (the in-flight factor 3.5): ticks under it fit in flight.
Data: results/analysis/s_bench_B_2026-10-24.json (tick_ms p95, plan_fast_refs on).
"""
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import review_style as S  # noqa: E402

COLS = {5: '#0d0887', 2: '#7e03a8', 1: '#cc4778'}


def main():
    d = json.load(open(os.path.join(S.REPO, 'results', 'analysis', 's_bench_B_2026-10-24.json')))
    S.setup()
    fig = S.figure('full', 2.6)
    ax = fig.subplots()
    # one iteration count only, unlabelled, no rig projection (Wesley 27 Oct)
    rows = sorted((r for r in d['rows'] if r['iters'] == 1), key=lambda r: r['n'])
    ax.plot([r['n'] for r in rows], [r['tick_ms']['p95'] for r in rows], '-o', color=S.BLUE, lw=S.LW_HERO, ms=S.MS,
            mec='white', mew=0.4)
    S.ref_line(ax, 100.0, '100 ms planner period', x=0.02, ha='left')
    ax.set_xlabel('Number of carriers')
    ax.set_ylabel('Planner tick, 95th percentile (ms)')
    ax.set_xticks(range(2, 11))
    ax.set_xlim(1.6, 10.4)
    S.save(fig, 'fig_s_tick')


if __name__ == '__main__':
    main()

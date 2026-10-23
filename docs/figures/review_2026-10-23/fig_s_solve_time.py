#!/usr/bin/env python3
"""S: our planner's solve time against the fleet size at 5, 2 and 1 SQP iterations a tick (card
docs/experiments/2026-10-23_scalability.md, section 4, baseline B).

Message: the solve time grows steeply with the fleet; fewer iterations a tick move the largest fleet that fits the
60 ms solve budget from 4 quads (5 iterations) to 5 (2) and 6 (1), on the idle laptop.
Hero: the solve time p95 per tick against n, one line per iteration count, the 60 ms budget dashed. Data: the 22 Oct
bench (results/analysis/solve_scaling_tuf_2026-10-22.json: 200 warm ticks after 20, the rig laptop idle; before the
speed-ups: the residual inside the solve, node-by-node extraction, no pivot). Left out: the median (within 1 ms of the
p95 below 9 quads), the whole tick, the rig factor (bar B's claim, still to run).
Preliminary: bar B (the bench with the speed-ups, the rig factor) replaces this.
"""
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import review_style as S  # noqa: E402

BENCH = os.path.join(S.REPO, 'results', 'analysis', 'solve_scaling_tuf_2026-10-22.json')


def main():
    b = json.load(open(BENCH))
    budget = float(b['args']['budget'])
    S.setup()
    fig = S.figure('half', 2.3)
    ax = fig.subplots()
    S.ref_line(ax, budget, rf'{budget:.0f}\,ms budget')
    styles = {5: ('0.1', 'o'), 2: ('0.38', 's'), 1: ('0.56', '^')}
    for it in b['args']['iters']:
        rows = sorted((r for r in b['rows'] if r['iters'] == it and not r['fails']), key=lambda r: r['n'])
        n = [r['n'] for r in rows]
        p95 = [r['solve_ms']['p95'] for r in rows]
        col, mk = styles[it]
        ax.plot(n, p95, color=col, lw=S.LW_HERO, marker=mk, ms=S.MS - 0.4, mec='white', mew=0.4)
        fit = [r['n'] for r in rows if r['solve_ms']['p95'] <= budget]
        print(f'{it} iterations: largest n within {budget:.0f} ms p95 = {max(fit)}')
        ax.annotate(f'{it} iteration' + ('s' if it > 1 else ''), xy=(n[-1], p95[-1]), xytext=(3, 0),
                    textcoords='offset points', ha='left', va='center', fontsize=8, color=col)
    ax.set_xlabel(r'Quads in the fleet')
    ax.set_ylabel(r'Solve time, p95 (ms)')
    ax.set_xticks(range(2, 10))
    ax.set_xlim(1.6, 11.2)
    ax.set_ylim(0, 340)
    ax.set_yticks([0, 100, 200, 300])
    ax.spines['bottom'].set_bounds(2, 9)
    S.save(fig, 'fig_s_solve_time', width='half')


if __name__ == '__main__':
    main()

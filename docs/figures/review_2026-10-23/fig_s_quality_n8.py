#!/usr/bin/env python3
"""S, Q6: how far each SQP iteration count leaves the 8-quad plan from the converged one (card
docs/experiments/2026-10-23_scalability.md, "Results, Q6").

Message: at 8 quads one iteration a tick leaves the plan outside every bar (dynamics residual, references,
feed-forward); two miss only the feed-forward, by 0.016 m/s^2; five pass them all, so the 8-quad claims fly at five.
Hero: the p95 over 200 noisy warm ticks of each quality measure against the iteration count, its frozen bar dashed.
Data: results/analysis/s_quality_bench_n8_2026-10-23.json (tools/solve_scaling.py --n 8 --quality --noise 0.01,
untimed). Left out: the cost ratio (Q1: within 1 % at 2 and 5 iterations; at 1 iteration 170 of 200 ticks miss it
through the residual), node 5's references (the same order: 0.9 / 5.1 / 14.5 mm against 10), the medians.
"""
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import review_style as S  # noqa: E402

DATA = os.path.join(S.REPO, 'results', 'analysis', 's_quality_bench_n8_2026-10-23.json')
# the card's frozen bars (section 5, Q2-Q4), per tick p95
PANELS = (('res_eq', 1.0, r'Dynamics residual', 1e-3, r'$10^{-3}$', True),
          ('ref1', 1000.0, r'Reference error (mm)', 5.0, r'5\,mm', False),
          ('ff0', 1.0, r'Feed-forward error (m\,s$^{-2}$)', 0.3, r'0.3', False))


def main():
    q = json.load(open(DATA))
    rows = sorted(q['rows'], key=lambda r: r['iters'])
    its = [r['iters'] for r in rows]
    S.setup()
    fig = S.figure('full', 1.85)
    axs = fig.subplots(1, 3)
    for k, (ax, (key, scale, label, bar, bar_text, log)) in enumerate(zip(axs, PANELS)):
        p95 = [scale * r['quality'][key]['p95'] for r in rows]
        x = range(len(its))
        S.ref_line(ax, bar, bar_text)
        ax.plot(x, p95, color=S.LIGHT, lw=S.LW_CTX, zorder=2)
        ax.plot(x, p95, 'o', ms=S.MS, mfc='#0d0887', mec='white', mew=0.5, zorder=3)
        print(key, dict(zip(its, [f'{v:.3g}' for v in p95])), 'bar', bar)
        ax.set_xticks(list(x))
        ax.set_xticklabels([str(i) for i in its])
        ax.set_xlim(-0.4, len(its) - 0.6)
        ax.set_ylabel(label)
        if log:
            ax.set_yscale('log')
            ax.set_ylim(1e-7, 1e-1)
            ax.set_yticks([1e-7, 1e-5, 1e-3, 1e-1])
            ax.minorticks_off()
        else:
            ax.set_ylim(0, 1.2 * max(max(p95), 2 * bar))
            ax.yaxis.set_major_locator(S.plt.MaxNLocator(4))
        S.ps.panel_label(ax, 'abc'[k])
    axs[1].set_xlabel(r'SQP iterations per tick')
    S.save(fig, 'fig_s_quality_n8')


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""S claim: how far each release and each rejoin moves the ring, at every team size (the chain of fig_s8_chain).

Message: the fewer carriers share the ring, the further one event moves it: a release moves it about 6 cm with eight or
seven carrying and about 8 cm with six or five, a rejoin's fold-in 4.6 cm into five down to 3.7 cm into eight, and the
ring is back within a few millimetres about 5 s after a release and 4 s after a fold-in ends.
Hero: the ring's distance from its hover point (run.csv, as recorded), (a) aligned at each RELEASE, (b) at each
REJOIN_WELD, with the fold-in (REJOIN_WELD to REJOIN_BLEND_OVER, events.csv) shaded. One time scale for both panels.
Shade: the team size after the event (darker = fewer). Both runs of the cell drawn (they overlay within 3 mm).
Left out: the tilt (fig_s8_chain (b); its settled value follows the layout's symmetry, not the team size), the
approach before each weld (the ring does not move), the bars.
Data: R2766, R2767 (s8_chain4back_R50_L100_it5, lab batch 18, week7 63b504f, 5 SQP iterations, ema).
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from review30_style import S, TEXT  # noqa: E402
from fig_s8_chain import load  # noqa: E402

RUNS = ('R2766', 'R2767')
WIN = {'RELEASE': (-2.0, 10.0), 'REJOIN_WELD': (-2.0, 16.0)}


def shade(n_after):
    """Team size after the event, 4 (dark) to 8 (light), in the figure's blue (range widened: critic 30 Oct)."""
    return S.plt.cm.Blues(1.0 - 0.12 * (n_after - 4))


def traces(r, kind):
    t, err, tn, n = r['t'], 1e3 * r['err'], r['tn'], r['n']
    lo, hi = WIN[kind]
    out = []
    for te in (te for te, name, _ in r['ev'] if name == kind):
        w = (t >= te + lo) & (t <= te + hi)
        after = n[(tn > te + 0.05) & (tn < te + 2.0)]
        # a rejoin's planner grows on the weld's tick: its size after is the first new value logged
        n_after = int(after[after != n[tn < te][-1]][0]) if (after != n[tn < te][-1]).any() else int(n[tn < te][-1])
        out.append((te, n_after, t[w] - te, err[w]))
    return out


def main():
    runs = [load(rid) for rid in RUNS]
    S.setup()
    fig = S.figure('full', 2.55)        # 2.3 in of panels plus the legend below
    spans = [WIN[k][1] - WIN[k][0] for k in WIN]
    gs = fig.add_gridspec(1, 2, width_ratios=spans)
    ax_r = fig.add_subplot(gs[0])
    ax_f = fig.add_subplot(gs[1], sharey=ax_r)

    peaks = {}
    for r in runs:
        for ax, kind in ((ax_r, 'RELEASE'), (ax_f, 'REJOIN_WELD')):
            for te, n_after, tt, e in traces(r, kind):
                ax.plot(tt, e, color=shade(n_after), lw=S.LW_HERO, solid_capstyle='round')
                k = int(np.argmax(e))
                peaks.setdefault((kind, n_after), []).append((float(e[k]), float(tt[k])))
                # back within 5 mm: the first sample after the peak (and, for a fold-in, after the blend) under 5 mm
                after = (np.arange(e.size) > k) & (e < 5.0)
                tb = float(tt[after][0]) if after.any() else float('nan')
                print(f'{kind:12s} n_after {n_after}: peak {e[k]:.1f} mm at {tt[k]:+.2f} s, back under 5 mm at {tb:+.2f} s')
    for (kind, n_after), v in sorted(peaks.items()):
        print(kind, n_after, 'peaks', [round(p, 1) for p, _ in v])

    ax_r.axvline(0, color=S.LIGHT, lw=0.5, zorder=0)
    over = np.mean([te_o - te_w for r in runs for te_w, te_o in
                    zip([te for te, nm, _ in r['ev'] if nm == 'REJOIN_WELD'],
                        [te for te, nm, _ in r['ev'] if nm == 'REJOIN_BLEND_OVER'])])
    ax_f.axvspan(0, over, color=S.FILL, lw=0, zorder=0)
    ax_f.annotate('fold-in', xy=(over / 2, 1.0), xycoords=('data', 'axes fraction'), xytext=(0, -3),
                  textcoords='offset points', ha='center', va='top', fontsize=8, color=TEXT)

    # the key, below the panels (Wesley 30 Oct): the shade is the team size after the event
    from matplotlib.lines import Line2D
    sizes = sorted({n for _, n in peaks})
    keys = [Line2D([], [], ls='none')] + [Line2D([], [], color=shade(n), lw=S.LW_HERO) for n in sizes]
    S.ps.legend_below(fig, keys, ['Carriers after the event:'] + [str(n) for n in sizes], ncol=len(keys))

    ax_r.set_ylim(0, 100)
    ax_r.set_yticks([0, 50, 100])
    ax_r.set_ylabel('Ring offset (mm)')
    ax_r.set_xlim(*WIN['RELEASE'])
    ax_r.set_xticks([0, 5, 10])
    ax_r.set_xlabel(r'$t$ after the release (s)')
    ax_f.set_xlim(*WIN['REJOIN_WELD'])
    ax_f.set_xticks([0, 5, 10, 15])
    ax_f.set_xlabel(r'$t$ after the fold-in starts (s)')
    ax_f.tick_params(labelleft=False)
    S.ps.panel_label(ax_r, 'a')
    S.ps.panel_label(ax_f, 'b')
    S.save(fig, 'fig_s8_events')


if __name__ == '__main__':
    main()

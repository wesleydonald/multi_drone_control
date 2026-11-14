"""Fig 7: which carrier-component vs ego-component pair his Separator reports as blocking, on the real 12 Nov transit +
settle samples (out/, all four flights pooled): (a) for the drop point, (b) for QUAD4's own position on the flown path."""
import collections

import numpy as np

from common import *

flights = ['fm9_5', 'fm9hc_2', 'fm9hc_4', 'fm9hc_5']
A = ('TRANSIT_TO_DROP_POINT', 'SETTLE_ABOVE_DROP_POINT')
cases = [('legacy', '0'), ('octagon', '0'), ('octagon', '1')]
fig, axs = plt.subplots(1, 2, figsize=(15, 5.6), sharey=False)
for ax, col_in, col_who, what in [(axs[0], 4, 5, 'drop point'), (axs[1], 2, 3, 'QUAD4\'s own position')]:
    counts, ns = {}, {}
    for mode, d in cases:
        c, n = collections.Counter(), 0
        for fl in flights:
            for r in [l.split() for l in open(f'{RT}/out/{fl}_{mode}_{d}.txt')][1:]:
                if r[1] in A:
                    n += 1
                    if r[col_in] == '1':
                        c[r[col_who]] += 1
        counts[(mode, d)], ns[(mode, d)] = c, n
    pairs = sorted({k for c in counts.values() for k in c}, key=lambda k: -sum(c[k] for c in counts.values()))[:7]
    y = np.arange(len(pairs))
    w = 0.27
    for j, ((mode, d), col) in enumerate(zip(cases, [C_LEG, C_OCT, '#85c1e9'])):
        v = [100 * counts[(mode, d)][p] / ns[(mode, d)] for p in pairs]
        ax.barh(y + (1 - j) * w, v, height=w, color=col,
                label=f'{mode}, {"static" if d == "0" else "swept 1 s"} ({ns[(mode, d)]} samples)')
        for yi, vi in zip(y + (1 - j) * w, v):
            if vi > 0:
                ax.text(vi + 0.5, yi, f'{vi:.0f}', va='center', fontsize=7.5)
    ax.set_yticks(y); ax.set_yticklabels([p.replace('_vs_', '\nvs ') for p in pairs], fontsize=8)
    ax.invert_yaxis()
    ax.set_xlabel(f'transit + settle samples where this pair blocks the {what} (%)')
    ax.set_title(f'Who blocks the {what}'.replace('the QUAD4', 'QUAD4'))
    ax.legend(loc='lower right', fontsize=8)
fig.suptitle('Fig 7. Real 12 Nov states (4 flights pooled): what blocks is a carrier\'s ATTACHED TETHER against QUAD4\'s '
             'cable, magnet or payload (a carrier body only 27 times in all saved outputs).\nThe probe names the FIRST '
             'hull that contains the point, in his obstacle list order: drone_0 is likely over-counted, the other tethers may '
             'contain it too.', fontsize=10.5)
fig.tight_layout(rect=(0, 0, 1, 0.92))
save(fig, 'fig7_which_pair_blocks.png')
for k, c in counts.items():
    print(k, c.most_common(5))

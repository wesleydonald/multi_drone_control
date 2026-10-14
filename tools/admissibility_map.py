#!/usr/bin/env python3
"""tools/admissibility_map.py -- which carrier may leave a layout, and where a newcomer should attach (statics only).

    python3 tools/admissibility_map.py [--out results/analysis/admissibility] [--elev 60] [--radius 0.225]

A design-time check on the M2A ring (12 plates every 30 deg, attachment radius 0.225 m), with no flight and no
simulation. Angles are anticlockwise from the ring's +x (plate k at 30 k deg, as the thesis and the rig number them).

For each 4-carrier layout of record, every single removal:
  * the survivors' support margin, from metrics.cog_margin: the distance from the ring's centre of mass (the ring
    centre) to the nearest edge of the survivors' polygon, negative when the centre lies outside, and the largest
    angular gap the leaver opens;
  * the level-hover tension shares before and after, from the planner's own static balance
    (mpc_planner.geometry.balanced_tensions with the rods at the planned elevation), so each survivor's planned
    step at the resize is the share it must take on.
For each 3-carrier layout of record, every free plate a newcomer could take: the share spread of the 4-layout it
makes, the newcomer's own share, and the worst single-removal margin afterwards (the N-1 rule).
Finally, every one of the 495 four-plate layouts is scored by its worst single-removal margin, to show which
layouts survive the loss of any one carrier and which (the even ring) survive none.
"""
import argparse
import csv
import itertools
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(REPO, 'src', 'mpc_planner'))
from metrics import cog_margin                                           # noqa: E402
from mpc_planner.geometry import balanced_tensions, nominal_cable_dirs   # noqa: E402

RING_KG = 0.86
G = 9.81
N_PLATES = 12
PITCH = 360.0 / N_PLATES

# Layouts of record, as plate angles (deg, anticlockwise from +x). Names are the thesis plate numbers.
LAYOUTS_4 = {
    '1/3/5/9': (30, 90, 150, 270),      # the M1 carry layout (3/11/7/9 on the 8-14 Oct sheets); the 8 Oct detaches
    '0/3/6/9': (0, 90, 180, 270),       # the even ring: zero margin after any loss
    '1/3/6/9': (30, 90, 180, 270),      # the 150 deg gap case (rig r304_3) and the 7 Oct capsize r0013
}
LAYOUTS_3 = {
    '1/5/9': (30, 150, 270),            # 1/3/5/9 after plate 3 leaves; the newcomer's plate in M1 is 3
    '0/4/8': (0, 120, 240),             # the even triple
    '1/6/9': (30, 180, 270),            # 1/3/6/9 after plate 3 leaves
}


def rho_of(angles_deg, radius):
    return [np.array([radius * np.cos(np.deg2rad(a)), radius * np.sin(np.deg2rad(a)), 0.0]) for a in angles_deg]


def shares(angles_deg, radius, elev_deg, mass=RING_KG):
    """Level-hover tension shares (fraction of the ring weight carried vertically) for a layout."""
    rho = rho_of(angles_deg, radius)
    s = nominal_cable_dirs(rho, elev_deg)
    t = np.array(balanced_tensions(rho, s, mass))
    vert = t * np.sin(np.deg2rad(elev_deg))
    return t, vert / (mass * G)


def removal_rows(name, angles, radius, elev):
    t4, sh4 = shares(angles, radius, elev)
    rows = []
    for k, leaver in enumerate(angles):
        surv = [a for a in angles if a != leaver]
        margin, gap, ok = cog_margin(rho_of(surv, radius))
        t3, sh3 = shares(surv, radius, elev)
        before = {a: sh4[i] for i, a in enumerate(angles)}
        after = {a: sh3[i] for i, a in enumerate(surv)}
        step = {a: after[a] - before[a] for a in surv}
        rows.append({
            'layout': name, 'leaver_deg': leaver, 'survivors_deg': ' '.join(str(a) for a in surv),
            'largest_gap_deg': round(gap, 1), 'margin_mm': round(1000 * margin, 1), 'level_ok': int(ok),
            'leaver_share_before': round(before[leaver], 3),
            'survivor_step_max': round(max(step.values()), 3),
            'survivor_step_sum': round(sum(step.values()), 3),
            'survivor_steps': ' '.join(f'{a}:{step[a]:+.3f}' for a in surv),
            'shares_after': ' '.join(f'{a}:{after[a]:.3f}' for a in surv),
            'spread_after': round(max(sh3) - min(sh3), 3),
        })
    return rows


def newcomer_rows(name, angles, radius, elev):
    rows = []
    free = [k * PITCH for k in range(N_PLATES) if k * PITCH not in angles]
    for plate in free:
        new = tuple(sorted(list(angles) + [plate]))
        t4, sh4 = shares(new, radius, elev)
        margins = {l: cog_margin(rho_of([a for a in new if a != l], radius))[0] for l in new}
        admissible = [l for l, m in margins.items() if m > 1e-9]
        rows.append({
            'layout3': name, 'newcomer_deg': plate, 'plate': int(plate / PITCH),
            'layout4': '/'.join(str(int(a / PITCH)) for a in new),
            'newcomer_share': round(sh4[list(new).index(plate)], 3),
            'spread': round(max(sh4) - min(sh4), 3),
            'n_admissible_leavers': len(admissible),
            'admissible_leavers_deg': ' '.join(str(int(l)) for l in admissible),
            'best_removal_margin_mm': round(1000 * max(margins.values()), 1),
            'newcomer_leaves_again_margin_mm': round(1000 * margins[plate], 1),
        })
    return rows


def all_layouts(radius):
    rows = []
    for combo in itertools.combinations(range(N_PLATES), 4):
        angles = [k * PITCH for k in combo]
        margins = [cog_margin(rho_of([a for a in angles if a != l], radius))[0] for l in angles]
        m4 = cog_margin(rho_of(angles, radius))[0]
        rows.append({'layout': '/'.join(str(k) for k in combo), 'margin4_mm': round(1000 * m4, 1),
                     'n_admissible_leavers': sum(1 for m in margins if m > 1e-9),
                     'best_removal_margin_mm': round(1000 * max(margins), 1),
                     'worst_removal_margin_mm': round(1000 * min(margins), 1)})
    rows.sort(key=lambda r: (-r['n_admissible_leavers'], -r['best_removal_margin_mm']))
    return rows


def write_csv(path, rows):
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)


def plot(out, removals, newcomers, radius):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    names = list(LAYOUTS_4)
    fig, axes = plt.subplots(1, len(names) + 1, figsize=(3.2 * (len(names) + 1), 3.4),
                             subplot_kw={'projection': 'polar'})
    for ax, name in zip(axes, names):
        angles = LAYOUTS_4[name]
        th = np.deg2rad(list(angles) + [angles[0]])
        ax.plot(th, [1.0] * len(th), '-', color='0.6', lw=1)
        ax.plot(np.deg2rad(angles), [1.0] * len(angles), 'o', color='k', ms=6)
        for r in removals:
            if r['layout'] != name:
                continue
            a = np.deg2rad(r['leaver_deg'])
            txt = f"{r['margin_mm']:.0f} mm\n{r['largest_gap_deg']:.0f} deg"
            ax.annotate(txt, (a, 1.0), xytext=(a, 1.55), textcoords='data', ha='center', va='center', fontsize=7,
                        color='C3' if r['margin_mm'] <= 0 else 'C2')
        ax.set_ylim(0, 1.8)
        ax.set_yticks([])
        ax.set_xticks(np.deg2rad(np.arange(0, 360, 30)))
        ax.set_xticklabels([str(k) for k in range(12)], fontsize=7)
        ax.set_title(f'{name}: margin after each leaver', fontsize=8)
    ax = axes[-1]
    ax.remove()
    ax = fig.add_subplot(1, len(names) + 1, len(names) + 1)
    for name in LAYOUTS_3:
        rs = [r for r in newcomers if r['layout3'] == name]
        ax.plot([r['newcomer_deg'] for r in rs], [r['spread'] for r in rs], 'o-', ms=3, label=f'from {name}')
    ax.set_xlabel('newcomer plate angle (deg)')
    ax.set_ylabel('level-hover share spread of the four')
    ax.set_title('where a newcomer should attach', fontsize=8)
    ax.legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(os.path.join(out, 'admissibility.png'), dpi=150)


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default=os.path.join(REPO, 'results', 'analysis', 'admissibility'))
    ap.add_argument('--elev', type=float, default=60.0, help='planned rod elevation (deg); 60 since 7 Oct')
    ap.add_argument('--radius', type=float, default=0.225)
    ap.add_argument('--no-plot', action='store_true')
    a = ap.parse_args(argv)
    os.makedirs(a.out, exist_ok=True)
    removals = [r for n, ang in LAYOUTS_4.items() for r in removal_rows(n, ang, a.radius, a.elev)]
    newcomers = [r for n, ang in LAYOUTS_3.items() for r in newcomer_rows(n, ang, a.radius, a.elev)]
    ranking = all_layouts(a.radius)
    write_csv(os.path.join(a.out, 'removals.csv'), removals)
    write_csv(os.path.join(a.out, 'newcomer.csv'), newcomers)
    write_csv(os.path.join(a.out, 'layouts_ranked.csv'), ranking)
    print('single removals (layout, leaver, gap, margin, leaver share, max survivor step):')
    for r in removals:
        print(f"  {r['layout']:7s} leaver {r['leaver_deg']:3.0f}  gap {r['largest_gap_deg']:5.1f}  "
              f"margin {r['margin_mm']:6.1f} mm  share {r['leaver_share_before']:.3f}  "
              f"step max {r['survivor_step_max']:+.3f} sum {r['survivor_step_sum']:+.3f}  steps {r['survivor_steps']}")
    print('newcomer plates (3-layout, plate, newcomer share, spread, admissible leavers afterwards, best margin):')
    for r in newcomers:
        print(f"  {r['layout3']:6s} plate {r['plate']:2d} ({r['newcomer_deg']:3.0f} deg) -> {r['layout4']:10s}  "
              f"share {r['newcomer_share']:.3f}  spread {r['spread']:.3f}  admissible {r['n_admissible_leavers']} "
              f"({r['admissible_leavers_deg']})  best margin {r['best_removal_margin_mm']:6.1f} mm")
    print('four-plate layouts ranked by admissible leavers, then the best margin:')
    for r in ranking[:8]:
        print(f"  {r['layout']:12s} margin4 {r['margin4_mm']:6.1f} mm  admissible leavers {r['n_admissible_leavers']}  "
              f"best {r['best_removal_margin_mm']:6.1f} mm  worst {r['worst_removal_margin_mm']:6.1f} mm")
    n_none = sum(1 for r in ranking if r['n_admissible_leavers'] == 0)
    n_any = sum(1 for r in ranking if r['worst_removal_margin_mm'] > 0)
    print(f"  {n_any} of {len(ranking)} four-plate layouts can lose ANY one carrier and stay level "
          f"(none can: the worst removal merges two gaps into at least 180 deg); {n_none} can lose none")
    if not a.no_plot:
        plot(a.out, removals, newcomers, a.radius)
    return 0


if __name__ == '__main__':
    sys.exit(main())

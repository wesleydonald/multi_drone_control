"""Fig 1: his free-space map at R4374's stall (x = 0 slice, body-centre positions), legacy_square vs outer_octagon.
Source: the probe's output rows quoted in the stall README (the full probe printout was not saved).
Legacy: the README states every row 2.20-3.40 m is identical, so that band is filled from the quoted row."""
import re

import numpy as np
from matplotlib.patches import Patch, Rectangle
from matplotlib.lines import Line2D

from common import *

txt = open(f'{STALL}/README.md').read().splitlines()
blocks, cur = {}, None
for l in txt:
    if l.startswith('legacy_square'):
        cur = 'legacy'
    elif l.startswith('outer_octagon'):
        cur = 'octagon'
    m = re.match(r'^z (\d\.\d\d) ([#.Ggq]+)$', l)
    if m and cur:
        blocks.setdefault(cur, {})[round(float(m.group(1)), 2)] = m.group(2)
leg = blocks['legacy']
for z in np.arange(2.20, 3.401, 0.05):          # "every row from 2.20 to 3.40 m is the same"
    leg.setdefault(round(z, 2), leg[2.20])
Y0, DY, DZ = -0.40, 0.025, 0.05
ZS = np.round(np.arange(2.20, 3.901, 0.05), 2)

run = load_run(f'{R4374}/run.csv')
s = at(run, 54.83)
P = plates(s)
q4 = np.array([s['d3_x'], s['d3_y'], s['d3_z']])
ball = np.array([s['ball_x'], s['ball_y'], s['ball_z']])

fig, axs = plt.subplots(1, 2, figsize=(11.5, 6.2), sharey=True)
for ax, mode, title in [(axs[0], 'legacy', 'legacy_square (his default, what flew)'),
                        (axs[1], 'octagon', 'outer_octagon (slanted tether prism)')]:
    rows = blocks[mode]
    for z in ZS:
        r = rows.get(round(z, 2))
        if r is None:
            ax.add_patch(Rectangle((Y0 - DY / 2, z - DZ / 2), 57 * DY, DZ, fc=C_UNK, ec='none', hatch='////',
                                   lw=0, alpha=0.6))
            continue
        for i, c in enumerate(r):
            if c in '#G':
                ax.add_patch(Rectangle((Y0 + i * DY - DY / 2, z - DZ / 2), DY, DZ, fc=C_BLOCK, ec='none', alpha=0.85))
    # the carriers drone_1 / drone_2 (plates 11 and 7) sit at x = +-0.63: their tethers projected onto this y-z plane
    for i in (1, 2):
        b = np.array([s[f'd{i}_x'], s[f'd{i}_y'], s[f'd{i}_z']])
        ax.plot([P[i][1], b[1]], [P[i][2], b[2]], '--', color=C_CARRIER, lw=1.8)
        ax.plot(b[1], b[2], 's', color=C_CARRIER, ms=8)
    b0 = np.array([s['d0_x'], s['d0_y'], s['d0_z']])
    ax.plot([P[0][1], b0[1]], [P[0][2], b0[2]], '--', color=C_CARRIER, lw=1.8)
    ax.plot(b0[1], b0[2], 's', color=C_CARRIER, ms=8)
    ax.text(b0[1] + 0.02, b0[2] + 0.07, 'drone_0 (x 0)', color=C_CARRIER, fontsize=8.5, clip_on=True)
    ax.text(0.39, 2.06, 'drone_1 / drone_2\n(x = +-0.63, overlap here)', color=C_CARRIER, fontsize=8.5, va='top')
    # the ring (plates at r 0.225)
    ax.plot([-PLATE_R, PLATE_R], [s['payload_z']] * 2, color='k', lw=4, solid_capstyle='butt')
    ax.text(-0.38, s['payload_z'] - 0.11, 'ring, z 1.20', fontsize=8.5)
    # QUAD4 at its stall with its cable and object; the drop point (body target over the ring centre)
    ax.plot([q4[1], ball[1]], [q4[2], ball[2]], color=C_Q4, lw=1.6)
    ax.plot(q4[1], q4[2], 'o', color=C_Q4, ms=9, mec='k', zorder=5)
    ax.plot(ball[1], ball[2], 'o', color=C_Q4, ms=7, mec='k', zorder=5)
    ax.plot(0, 2.90, '*', color=C_DROP, ms=16, mec='k', zorder=6)
    ax.plot([0, 0], [2.90, 1.79], ':', color=C_DROP, lw=1.4)
    ax.annotate('QUAD4 stalled\ny 0.677, z 2.90', (q4[1], q4[2]), (0.73, 3.42), fontsize=8.5,
                arrowprops=dict(arrowstyle='->', lw=0.8))
    ax.annotate('its object\nz 1.79', (ball[1], ball[2]), (0.78, 1.55), fontsize=8.5,
                arrowprops=dict(arrowstyle='->', lw=0.8))
    ax.annotate('drop point\n(body target)', (0, 2.90), (-0.36, 3.62), fontsize=8.5, color=C_DROP,
                arrowprops=dict(arrowstyle='->', lw=0.8, color=C_DROP))
    ax.axhspan(2.175, 3.925, fc='none', ec='0.5', lw=0.6)
    ax.set_xlim(-0.42, 1.03)
    ax.set_ylim(1.0, 3.95)
    ax.set_xlabel('y (m), x = 0 slice  [QUAD4 comes in from +y]')
    ax.set_title(title)
axs[0].set_ylabel('z (m): QUAD4 BODY-CENTRE height for the map rows (2.20-3.90)')
axs[0].text(-0.40, 2.55, 'blocked from\ny -0.40 to 0.65\nat every height\n2.20-3.40 m', fontsize=9, color='white',
            weight='bold')
axs[1].text(0.47, 2.62, 'z 2.75 / 2.50:\nblocked to y 0.40-0.43', fontsize=8.5, color='k')
axs[1].text(0.05, 3.08, 'free band at 2.90 m:\nonly x = +-0.05 m wide\nat y 0-0.4 (README)', fontsize=8.5)
leg_items = [Patch(fc=C_BLOCK, alpha=0.85, label='body centre here collides (his hulls)'),
             Patch(fc='white', ec='0.6', label='free'),
             Patch(fc=C_UNK, hatch='////', label='row not in the saved probe output'),
             Line2D([], [], color=C_CARRIER, ls='--', marker='s', label='carrier body + tether (projected)'),
             Line2D([], [], color=C_Q4, marker='o', mec='k', label='QUAD4 body, cable, object (R4374 t 54.83 s)'),
             Line2D([], [], color=C_DROP, marker='*', ls='', ms=12, mec='k', label='drop point')]
fig.legend(handles=leg_items, loc='lower center', ncol=3, bbox_to_anchor=(0.5, -0.06), frameon=True)
fig.suptitle('Fig 1. R4374 stall (twin hover, swing 5 deg, tracking 0.06): where his obstacle set lets QUAD4\'s body be '
             '(x = 0 slice). Legacy boxes swallow the whole ring; the octagon frees a thin layer at 2.9 m.',
             fontsize=10.5, y=1.0)
fig.tight_layout(rect=(0, 0.03, 1, 0.97))
save(fig, 'fig1_free_space_stall_R4374.png')

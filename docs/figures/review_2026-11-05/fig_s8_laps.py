#!/usr/bin/env python3
"""5 Nov copy of review_2026-10-30/fig_s8_laps.py: the thesis' words in the figure (quadrotors, rejoin, distance from the hover point).

Eight carriers on the figure-eight in simulation, lap by lap: eight, six, four, eight again (replaces the four
Gazebo screenshots shot_s8_fig8laps_1-4.png of fig:res_s8).

Message: the team shrinks from eight to four by releases on the move and grows back to eight, the ring on its path.
Hero: four snapshots from one fixed orthographic camera, rebuilt from the logged poses by tools/snapshot_render.py:
the ring black (far half grey), its plates, each UAV coloured by its role (carrying blue, departed grey), each cable
as the state log has it, the ring's reference path light grey, a 1 m scale bar.
Left out: the floor, the lift and the landing, the rejoins themselves (fig_rejoin and fig_s8_chain show them).
Data: R2779 (s8_chain4back_fig8laps_R50_L100_it5, exploratory, lab batch 18, week7 63b504f): run.csv, events.csv
(the joint events give the roles), the planner's params.json, gz_record/state.tlog (the rods).
"""
import glob
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'review_2026-10-30'))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))     # this folder's fig_s8_chain first
from review30_style import S  # noqa: E402
S.HERE = os.path.dirname(os.path.abspath(__file__))     # save beside this 5 Nov copy

sys.path.insert(0, S.TOOLS)
import snapshot_render as SR  # noqa: E402

RUN = 'R3924'     # 7 Nov: the figure-eight chain on the earlier masses (F065); was R3782, R2779
TIMES = (44.0, 100.0, 145.0, 325.0)       # run clock (R3924): eight, six, four, eight again after the last weld (307 s)
AZIM, ELEV = -62.0, 32.0


def main():
    (d,) = glob.glob(os.path.join(S.REPO, 'results', '*', f'{RUN}_*'))
    run = SR.Run(d)
    t_path = [t for t, e, a in run.events if e == 'PATH_START'][0]
    t_land = [t for t, e, a in run.events if e == 'LAND'][0]
    laps = [t for t, e, a in run.events if e == 'LAP_DONE']
    path = SR.reference_path(run, t_path, t_land)
    cam = SR.Camera(AZIM, ELEV)
    xl, yl = SR.extent(run, TIMES, cam, path, pad=0.03)

    S.setup()
    w = S.WIDTH['full']
    aspect = (yl[1] - yl[0]) / (xl[1] - xl[0])
    fig = S.figure('full', 2 * (w / 2) * aspect + 0.75)
    axes = fig.subplots(2, 2)
    seen = set()
    for k, (ax, t) in enumerate(zip(axes.flat, TIMES)):
        roles = SR.draw_snapshot(ax, run, t, cam, path=path)
        seen |= set(roles)
        ax.set_xlim(*xl)
        ax.set_ylim(*yl)
        ax.set_aspect('equal')
        ax.set_xticks([])
        ax.set_yticks([])
        lap = 1 + sum(1 for te in laps if te <= t)
        S.panel(ax, 'abcd'[k], f'{roles.count("carrying")} quadrotors, lap {lap}, $t = {t:.0f}$\\,s')
        print(f'({"abcd"[k]}) t {t:.0f} s: {roles.count("carrying")} carrying, '
              f'{roles.count("departed")} departed, {roles.count("rejoining")} rejoining')
    order = [r for r in SR.ROLE_ORDER if r in seen]
    handles = [S.plt.Line2D([], [], ls='none', marker='o', ms=5,
                            mfc='white' if r == 'departed' else SR.ROLE_COLOUR[r],
                            mec=SR.ROLE_COLOUR[r] if r == 'departed' else 'white', mew=0.9) for r in order]
    handles += [S.plt.Line2D([], [], ls='none', marker='o', ms=3.4, mfc='white', mec='black', mew=0.6),
                S.plt.Line2D([], [], color='black', lw=1.6), S.plt.Line2D([], [], color='0.82', lw=0.9)]
    S.ps.legend_below(fig, handles, [{'carrying': 'Carrying', 'departed': 'Departed', 'rejoining': 'Rejoining'}.get(r, r) for r in order] + ['Empty plate', 'Payload', 'Payload path'], ncol=len(handles))
    S.save(fig, 'fig_s8_laps')


if __name__ == '__main__':
    main()

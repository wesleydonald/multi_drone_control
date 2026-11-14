#!/usr/bin/env python3
"""A planned detachment and the reattachment of the new agent in one flight, Setup 3 (RIG-1111-nrtfc_5, the
supervisor's funnel; feed F119-F122; Wesley 13 Nov: the reattachment shows how resizing works, the attachment itself is
the collaborator's).

The one sentence: at the detachment the planner unloads the new agent over about 3 s and resizes to three quadrotors as
its magnet is switched off; at the reattachment it resizes to four within 0.1 s of the confirmation and blends the new
agent's tension in over 10 s, while the ring stays within a few degrees of its tilt before each event.
Hero: 2 x 2, columns the detachment (time from the magnet OFF, which is the resize) and the reattachment (time from the
confirmation, which the resize follows within 0.08 s); top row the planner's commanded tension per quadrotor (the OCP's
node-0 tension, which includes the disturbance estimate: F122 says to call it commanded, never measured), bottom row the
ring's tilt. The unloading and the blend are light bands; the contact is a thin line in (b).
Left out: the commanded (node-0) tensions and the reference split, the
new agent's transit between the two events, the landing (LAND at +12.4 s after the confirmation ends the right column).
Data: results/analysis/rig_results_2026-11-13/nrtfc_5_poses.csv (F121, ~19.4 Hz sample-and-hold; t_from_takeoff,
Tplan_d0..3, r_q*, event). Drone to plate on 11 Nov (F122): d1 plate 3, d2 plate 7, d0 plate 11, d3 the new agent on 9.
"""
import os
import sys

import numpy as np
import pandas as pd

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import review_style as S  # noqa: E402

S.HERE = HERE
CSV = os.path.join(S.REPO, 'results', 'analysis', 'rig_results_2026-11-13', 'nrtfc_5_poses.csv')
PLATES = ((1, 'Plate 3'), (2, 'Plate 7'), (0, 'Plate 11'), (3, 'New agent'))
NEW_ID = 3


def ev(d, text, after=-1e9):
    """The first event containing text after the time `after` (the setup's FLEET RESIZED n=4 comes before takeoff)."""
    m = d.event.fillna('').str.contains(text, regex=False) & (d.t_from_takeoff > after)
    return float(d.loc[m, 't_from_takeoff'].iloc[0])


def main():
    d = pd.read_csv(CSV)
    t = d.t_from_takeoff.to_numpy()
    tilt = np.degrees(np.arccos(np.clip(1 - 2 * (d.r_qx ** 2 + d.r_qy ** 2), -1, 1))).to_numpy()
    t_unload, t_off = ev(d, 'UNLOAD (soft'), ev(d, 'UNLOADED; FLEET RESIZED n=3')
    t_contact, t_proof = ev(d, 'CONTACT try 1'), ev(d, 'ATTACH proven')
    t_fold, t_land = ev(d, 'FLEET RESIZED n=4', t_proof), ev(d, 'fleet LAND')
    print(f'unload {t_unload:.2f} off {t_off:.2f} | contact {t_contact:.2f} proof {t_proof:.2f} fold {t_fold:.2f} '
          f'land {t_land:.2f}')
    cols = ((t_off, (-6.0, 12.0), (t_unload - t_off, 0.0), None),
            (t_proof, (-4.0, t_land - t_proof), (t_fold - t_proof, t_fold - t_proof + 10.0), t_contact - t_proof))

    S.setup()
    fig = S.figure('full', 3.3)
    axs = fig.subplots(2, 2, sharex='col', sharey='row')
    handles = {}
    for j, (t0, win, band, t_c) in enumerate(cols):
        tt = t - t0
        m = (tt >= win[0]) & (tt <= win[1])
        for ax in axs[:, j]:
            ax.axvspan(*band, color=S.FILL, lw=0, zorder=0)
            ax.axvline(0, color=S.LIGHT, lw=0.6, zorder=0)
            if t_c is not None:
                ax.axvline(t_c, color=S.LIGHT, lw=0.6, ls=(0, (2, 2)), zorder=0)
        for k, name in PLATES:
            y = d[f'Tpull_d{k}'].to_numpy().copy()
            if k == NEW_ID:
                y[d.d3_mux.to_numpy() != 'ours'] = np.nan     # frozen while the collaborator's controller flies it (F122)
            h, = axs[0, j].plot(tt[m], y[m], color=S.ps.drone_colour(k), drawstyle='steps-post',
                                lw=S.LW_HERO if k == NEW_ID else S.LW_CTX + 0.2)
            handles[name] = h
        axs[1, j].plot(tt[m], tilt[m], color=S.INK, lw=S.LW_CTX + 0.2, drawstyle='steps-post')
        axs[1, j].set_xlim(*win)
        pre = (tt >= win[0]) & (tt < 0)
        post = (tt >= 0) & (tt <= win[1])
        print(f'column {j}: tilt before max {np.nanmax(tilt[pre]):.1f}, after max {np.nanmax(tilt[post]):.1f} deg')
    for ax in axs[1]:
        ax.set_xticks([-5, 0, 5, 10] if ax is axs[1, 0] else [0, 5, 10])
    axs[0, 0].set_ylabel('Measured\ntension (N)')
    axs[1, 0].set_ylabel(r'Payload tilt ($^\circ$)')
    axs[1, 0].set_xlabel('Time after the magnet switch-off (s)')
    axs[1, 1].set_xlabel('Time after the attachment is confirmed (s)')
    axs[0, 0].set_ylim(0, None)
    axs[1, 0].set_ylim(0, None)
    axs[1, 0].set_yticks([0, 2, 4, 6, 8])
    fig.align_ylabels(axs[:, 0])
    names = ('Tension at the detachment', 'Tension at the reattachment', 'Tilt at the detachment', 'Tilt at the reattachment')
    for ax, letter, name in zip(axs.flat, 'abcd', names):
        S.panel(ax, letter, name)
    names = [n for _, n in PLATES]
    band = S.plt.Rectangle((0, 0), 1, 1, color=S.FILL)
    contact = S.plt.Line2D([], [], color=S.LIGHT, lw=0.6, ls=(0, (2, 2)))
    S.ps.legend_below(fig, [handles[n] for n in names] + [band, contact], names + ['Unloading or blending', 'Contact'], ncol=3)
    S.save(fig, 'fig_detach_reattach')


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Reaction times on every rig flight that lost a carrier as planned (4 -> 3, 8 Oct): the alternative to fig_release.

Message: on every rig release the planner detected the loose cable 0.33-0.42 s after the magnet opened and resized
0.09-0.11 s later; the ring was back in its tilt band 1.8-3.8 s after the release, its tilt peaking at 6.5-17.8 deg
(the 150 deg gap the highest). The announced detach resized at once but tilted the ring to 15.4 deg.
One row per flight (best_data.RELEASES) on one clock from the event (the operator's magnet OFF; the planner's DETACH
command for the announced detach), in the four-carrier colour: open diamond = DETECTED, bar = RESIZE (the planner's DETACH line: OCP resized to n = 3, for the
announced detach the command itself), the grey line runs to back in
band (the ring's tilt under its 5 s pre-event peak + 0.5 deg for good, as fig_release), the tilt peak printed at its end.
Flights: r303_4 (hover, plates 3/11/7/9 -> 3/11/7), r304_3 (hover, 150 deg gap left), r305 (on the 0.125 m/s circle),
r302_3 (announced). Left out: the twin's 1 m release R2486 (resize 0.37 s, back in band 3.4 s, peak 5.9 deg: in the
index; its log has no sim-time detection stamp); r0012 (7 Oct announced, LAND 8 s after with the ring 15 cm high).
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import best_style as S  # noqa: E402
import best_data as B  # noqa: E402

ROWS = B.RELEASES
C = S.FLEET_COLOUR[4]               # every row is a four-carrier flight that went to three


def reaction(key):
    f = B.Flight(key)
    d = f.run()
    t = d.t.to_numpy()
    te = f.t_event('RELEASE')
    announced = te is None
    if announced:
        te = f.t_event('DETACH')
    det = None if announced else f.t_event('DETECTED', te - 0.01) - te
    resize = f.t_event('DETACH', te - 0.01) - te         # the planner's 'DETACH ... (OCP resize)' line
    end = min(f.t_event('LAND', te) or np.inf, te + 15.0)
    tilt = d.payload_tilt_deg.to_numpy()
    band = float(np.nanmax(tilt[(t >= te - 5) & (t < te)])) + 0.5
    post = (t >= te) & (t <= end)
    back = float(t[np.flatnonzero(post & (tilt > band))[-1]] - te)
    return dict(det=det, resize=resize, back=back, peak=float(np.nanmax(tilt[post])), window=end - te)


def main():
    rows = [(label, reaction(k)) for k, label in ROWS]
    for (k, _), (label, r) in zip(ROWS, rows):
        print(k, label, {a: (round(v, 2) if v is not None else None) for a, v in r.items()})
    S.setup()
    fig = S.figure('full', 1.75)
    ax = fig.subplots()
    for j, (label, r) in enumerate(rows):
        y = -j
        ax.plot([0, r['back']], [y, y], color=S.tint(C, 0.35), lw=2.6, solid_capstyle='butt', zorder=1)
        if r['det'] is not None:
            ax.plot(r['det'], y, 'D', ms=3.4, color=C, mfc='white', mew=0.9, zorder=3)
        ax.plot([r['resize']] * 2, [y - 0.22, y + 0.22], color=C, lw=S.LW_HERO, zorder=3)
        ax.annotate(f"{r['peak']:.1f}" + r'$^\circ$', xy=(r['back'], y), xytext=(4, 0), textcoords='offset points',
                    ha='left', va='center', fontsize=8, color=S.GREY)
    ax.annotate('detected', xy=(rows[0][1]['det'], 0), xytext=(-1, 6), textcoords='offset points', ha='right',
                va='bottom', fontsize=8, color=C)
    ax.annotate('resized', xy=(rows[0][1]['resize'], 0), xytext=(2, 6), textcoords='offset points', ha='left',
                va='bottom', fontsize=8, color=C)
    ax.annotate(r'back in band, peak tilt', xy=(rows[0][1]['back'], 0), xytext=(4, 6), textcoords='offset points',
                ha='left', va='bottom', fontsize=8, color=S.GREY)
    ax.axvline(0, color=S.LIGHT, lw=0.5, zorder=0)
    ax.set_yticks([-j for j in range(len(rows))], [label for label, _ in rows])
    ax.tick_params(axis='y', length=0)
    ax.set_ylim(-len(rows) + 0.4, 0.75)
    ax.set_xlim(-0.1, 5)
    ax.set_xticks([0, 1, 2, 3, 4, 5])
    ax.set_xlabel(r'$t$ after the release (s)')
    for side in ('left',):
        ax.spines[side].set_visible(False)
    S.save(fig, 'fig_reaction')


if __name__ == '__main__':
    main()

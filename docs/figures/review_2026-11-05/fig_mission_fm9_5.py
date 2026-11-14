#!/usr/bin/env python3
"""The first complete full mission on Setup 3 (RIG-1112-fm9_5, 12 Nov; feed F103, F115/F116; Wesley 13 Nov: the real
world told event by event up to the full mission).

The one sentence: the planner flew four quadrotors, then three while the new agent detached, picked up the object and
dropped it into the net, then four again after the reattachment, and the ring held its height and stayed within about
10 deg of level throughout.
Hero: (a) heights against time from takeoff: the ring (black), the new agent (its drone colour), the object (grey);
(b) the ring's tilt. The time with three quadrotors in the planner is a light band; the four events are thin lines
named once above (a).
Left out: the landing (the window ends at LAND), the failed pickup tries and the failed first proof (the text and the
phase table give them), the carriers' own heights.
Data: results/rig/2026-11-12/fm9_5_run/logs/run.csv (payload_z, payload_tilt_deg, d3_z; TAKEOFF at run t 4.317 s from
events.csv) and results/analysis/rig_results_2026-11-13/fm9_5_full/obj.csv (the object, epoch; TAKEOFF at epoch
1794434730.816 from mission_events.csv). Event times after TAKEOFF from full_mission_fm9_5.csv (F115, reviewed F116):
our magnet OFF +20.99, the object lifted +109.8, the drop +154.5, the fold-in +171.0, LAND +186.7.
"""
import os
import sys

import numpy as np
import pandas as pd

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import review_style as S  # noqa: E402

S.HERE = HERE
RUN = os.path.join(S.REPO, 'results', 'rig', '2026-11-12', 'fm9_5_run', 'logs')
FULL = os.path.join(S.REPO, 'results', 'analysis', 'rig_results_2026-11-13', 'fm9_5_full')
NEW_ID = 3


def main():
    ev = pd.read_csv(os.path.join(RUN, 'events.csv'))
    t_to = float(ev.loc[ev.event == 'TAKEOFF', 'sim_time'].iloc[0])
    t_det = float(ev.loc[ev.event == 'DETACH', 'sim_time'].iloc[0]) - t_to       # our magnet OFF
    t_land = float(ev.loc[ev.event == 'LAND', 'sim_time'].iloc[0]) - t_to
    me = pd.read_csv(os.path.join(S.REPO, 'results', 'analysis', 'rig_results_2026-11-13', 'mission_events.csv'))
    me = me[me.flight == 'fm9_5']
    epoch0 = float(me.loc[me.event == 'fleet TAKEOFF', 'epoch'].iloc[0])
    ph = pd.read_csv(os.path.join(S.REPO, 'results', 'analysis', 'rig_results_2026-11-13', 'full_mission_fm9_5.csv'))
    start = dict(zip(ph.phase, ph.start_s))
    t_lift = start['object lift']
    t_drop = start['drop (release dwell)']
    t_fold = start['fold-in hold (resize -> trajectory resumes)']
    print(f'events after TAKEOFF: detach {t_det:.2f}, lift {t_lift}, drop {t_drop}, fold-in {t_fold}, LAND {t_land:.2f}')

    run = pd.read_csv(os.path.join(RUN, 'run.csv'), usecols=['t', 'payload_z', 'payload_tilt_deg', 'd3_z'])
    run = run.drop_duplicates('t')
    t = run.t.to_numpy() - t_to
    m = (t >= 0) & (t <= t_land)
    obj = pd.read_csv(os.path.join(FULL, 'obj.csv'), usecols=['t', 'z'])
    to = obj.t.to_numpy() - epoch0
    mo = (to >= 0) & (to <= t_land)

    VAR = os.environ.get('FIG20_VARIANT', 'C')       # Wesley 14 Nov: options A (markers), B (A + count), C (phase strip,
    #                                                  his pick: the strip between the title of (a) and its plot)
    AGENT = os.environ.get('FIG20_AGENT', 'New agent')
    if VAR:
        return variant(VAR, AGENT, t, m, run, to, mo, obj, t_det, t_lift, t_drop, t_fold, t_land, ph)
    S.setup()
    fig = S.figure('full', 3.2)
    a, b = fig.subplots(2, 1, sharex=True, gridspec_kw=dict(height_ratios=[1.5, 1]))
    for ax in (a, b):
        ax.axvspan(t_det, t_fold, color=S.FILL, lw=0, zorder=0)
        for te in (t_det, t_lift, t_drop, t_fold):
            ax.axvline(te, color=S.LIGHT, lw=0.6, zorder=0)
    a.plot(to[mo], obj.z.to_numpy()[mo], color=S.BLUE, lw=S.LW_CTX, label='Object', rasterized=True)   # not grey: the events are
    a.plot(t[m], run.d3_z.to_numpy()[m], color=S.ps.drone_colour(NEW_ID), lw=S.LW_CTX + 0.2, label='New agent')
    a.plot(t[m], run.payload_z.to_numpy()[m], color=S.INK, lw=S.LW_HERO, label='Payload')
    b.plot(t[m], run.payload_tilt_deg.to_numpy()[m], color=S.INK, lw=S.LW_CTX)          # dense: thin, as recorded
    tilt_peak = float(np.nanmax(run.payload_tilt_deg.to_numpy()[m & (t >= 5)]))
    print(f'ring tilt peak from +5 s to LAND: {tilt_peak:.1f} deg; ring z median in hover '
          f'{np.nanmedian(run.payload_z.to_numpy()[m & (t > 25)]):.2f} m')

    # Wesley 14 Nov: panel titles above the axes, so the event names sit just inside the top edge of (a), each beside its
    # line ("Drop" to the left of its line, the others to the right); the y limit leaves room for them above the traces
    for te, name, ha in ((t_det, 'Detachment', 'left'), (t_lift, 'Pickup', 'left'), (t_drop, 'Drop', 'right'),
                         (t_land, 'Reattachment', 'right')):          # the last ends at the axes' right edge
        a.annotate(name, xy=(te, 1.0), xycoords=('data', 'axes fraction'), xytext=(2 if ha == 'left' else -2, -2),
                   textcoords='offset points', ha=ha, va='top', fontsize=8, color=S.INK)
    b.annotate('Three quadrotors', xy=(0.5 * (t_det + t_lift), 0.93), xycoords=('data', 'axes fraction'),
               ha='center', va='top', fontsize=8, color=S.INK)
    a.set_ylabel('Height (m)')
    b.set_ylabel(r'Payload tilt ($^\circ$)')
    b.set_xlabel('Time after takeoff (s)')
    b.set_xlim(0, t_land)
    b.set_ylim(0, None)
    a.set_ylim(0, 3.35)
    fig.align_ylabels((a, b))
    S.panel(a, 'a', 'Heights')
    S.panel(b, 'b', 'Payload tilt')
    h, lab = a.get_legend_handles_labels()
    order = [2, 1, 0]
    S.ps.legend_below(fig, [h[i] for i in order], [lab[i] for i in order], ncol=3)
    S.save(fig, 'fig_mission_fm9_5')


def variant(VAR, AGENT, t, m, run, to, mo, obj, t_det, t_lift, t_drop, t_fold, t_land, ph):
    """The options of 14 Nov: no grey band and no words inside the plot boxes."""
    S.setup()
    if VAR == 'B':
        fig = S.figure('full', 3.6)
        a, b, c = fig.subplots(3, 1, sharex=True, gridspec_kw=dict(height_ratios=[1.5, 1, 0.55]))
    elif VAR == 'C':
        fig = S.figure('full', 3.3)
        a, b = fig.subplots(2, 1, sharex=True, gridspec_kw=dict(height_ratios=[1.5, 1]))
        st = a.inset_axes([0, 1.0, 1, 0.15], sharex=a)       # the phase strip sits on top of (a), under its title
        st.tick_params(axis='x', labelbottom=False, bottom=False)
    else:
        fig = S.figure('full', 3.2)
        a, b = fig.subplots(2, 1, sharex=True, gridspec_kw=dict(height_ratios=[1.5, 1]))
    zo = obj.z.to_numpy()
    za = run.d3_z.to_numpy()
    ho, = a.plot(to[mo], zo[mo], color=S.BLUE, lw=S.LW_CTX, rasterized=True)
    hn, = a.plot(t[m], za[m], color=S.ps.drone_colour(NEW_ID), lw=S.LW_CTX + 0.2)
    hp, = a.plot(t[m], run.payload_z.to_numpy()[m], color=S.INK, lw=S.LW_HERO)
    b.plot(t[m], run.payload_tilt_deg.to_numpy()[m], color=S.INK, lw=S.LW_CTX)
    handles, names = [hp, hn, ho], ['Payload', AGENT, 'Object']
    if VAR in ('A', 'B'):
        at = lambda tt, x, y: float(np.interp(x, tt, y))
        marks = ((t_det, t[m], za[m], 'v', 'Detachment'), (t_lift, to[mo], zo[mo], 'o', 'Pickup'),
                 (t_drop, to[mo], zo[mo], 's', 'Drop'), (t_fold, t[m], za[m], '^', 'Reattachment'))
        for x, tt, yy, mk, name in marks:
            h, = a.plot([x], [at(tt, x, yy)], mk, ms=5, mfc='white', mec=S.INK, mew=1.0, zorder=5, ls='')
            handles.append(h); names.append(name)
    if VAR == 'B':
        nq = np.where((t >= t_det) & (t < t_fold), 3, 4)
        c.step(t[m], nq[m], where='post', color=S.INK, lw=S.LW_HERO)
        c.set_ylim(2.5, 4.5)
        c.set_yticks([3, 4])
        c.set_ylabel('No. of\nquadrotors')
    if VAR == 'C':
        s_, e_ = dict(zip(ph.phase, ph.start_s)), dict(zip(ph.phase, ph.end_s))
        ph = {'pickup': (s_['his takeoff and approach over the object'], s_['object lift']),
              'transit': (s_['object lift'], s_['settle and descent over the ring']),
              'drop': (s_['settle and descent over the ring'], e_['drop (release dwell)']),
              'reattach': (e_['drop (release dwell)'], e_['fold-in (proof -> FLEET RESIZED n=4)'])}
        st.set_ylim(0, 1)
        st.set_yticks([])
        st.tick_params(axis='x', length=0)
        rows = (('Lift, detach', 0.0, ph['pickup'][0]), ('Pickup', ph['pickup'][0], ph['transit'][0]),
                ('Transit', ph['transit'][0], ph['drop'][0]), ('Drop', ph['drop'][0], ph['reattach'][0]),
                ('Reattach', ph['reattach'][0], ph['reattach'][1]), ('Hover', ph['reattach'][1], t_land))
        for k, (name, x0, x1) in enumerate(rows):
            if k:
                st.axvline(x0, color=S.INK, lw=0.6)
            st.annotate(name, xy=(0.5 * (x0 + x1), 0.5), ha='center', va='center', fontsize=7, color=S.INK)
        for ax in (a, b):
            for x0 in [r[1] for r in rows[1:]]:
                ax.axvline(x0, color=S.LIGHT, lw=0.5, zorder=0)
    a.set_ylabel('Height (m)')
    b.set_ylabel(r'Payload tilt ($^\circ$)')
    last = c if VAR == 'B' else b
    last.set_xlabel('Time after takeoff (s)')
    last.set_xlim(0, t_land)
    b.set_ylim(0, None)
    a.set_ylim(0, 3.0)
    axes = [a, b] + ([c] if VAR == 'B' else [])
    fig.align_ylabels(axes)
    titles = ('Heights', 'Payload tilt') + (('Quadrotors carrying',) if VAR == 'B' else ())
    for ax, letter, name in zip([st if VAR == 'C' else a] + axes[1:], 'abcd', titles):
        S.panel(ax, letter, name)
    if len(handles) == 7:        # lines on the first row, the four event markers on the second (column-major order)
        blank = S.plt.Line2D([], [], ls='', marker='')
        order = [(handles[0], names[0]), (handles[3], names[3]), (handles[1], names[1]), (handles[4], names[4]),
                 (handles[2], names[2]), (handles[5], names[5]), (blank, ''), (handles[6], names[6])]
        handles, names = [h for h, _ in order], [n for _, n in order]
    S.ps.legend_below(fig, handles, names, ncol=min(len(handles), 4))
    S.save(fig, ('fig_mission_fm9_5' if VAR == 'C' else f'fig_mission_fm9_5_{VAR}') + ('_tasked' if AGENT != 'New agent' else ''))


if __name__ == '__main__':
    main()

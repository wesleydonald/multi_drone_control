#!/usr/bin/env python3
"""Eight carriers in the twin (5 SQP iterations): the ring's path on the figure-8 and the orbit against the reference.

Message: with eight carriers the ring follows both paths within about 2 cm (17.5 and 20.1 mm clock-free RMS).
Data: R2771 (s8_fig8_h5_v200_R50_L100_it5), R2769 (s8_orbit_r30_v200_R50_L100_it5); PATH_START to LAND.
"""
import glob
import os
import sys

import numpy as np
import pandas as pd

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import review_style as S  # noqa: E402
import metrics as M  # noqa: E402

RUNS = (('R2771', 'figure-8', '17.5 mm RMS'), ('R2769', 'orbit', '20.1 mm RMS'))
EIGHT = '#0d0887'                                    # plasma 0.0: one step darker than the 4-carrier purple


def main():
    S.setup()
    fig = S.figure('full', 2.6)
    axs = fig.subplots(1, 2)
    for ax, (rid, name, rms) in zip(axs, RUNS):
        run = glob.glob(os.path.join(S.REPO, 'results', '2026-10-2*', rid + '_*'))[0]
        ev = pd.read_csv(os.path.join(run, 'logs', 'events.csv'))
        t0 = float(ev.loc[ev.event == 'PATH_START', 'sim_time'].iloc[0]) if (ev.event == 'PATH_START').any() \
            else float(ev.loc[ev.event == 'LIFTED', 'sim_time'].iloc[0]) + 10.0
        t1 = float(ev.loc[ev.event == 'LAND', 'sim_time'].iloc[0])
        d = M.load_run(run)['data']
        t = np.asarray(d['t'], float)
        m = (t >= t0) & (t <= t1)
        ax.plot(np.asarray(d['payload_ref_x'], float)[m], np.asarray(d['payload_ref_y'], float)[m], **S.REF)
        ax.plot(np.asarray(d['payload_x'], float)[m], np.asarray(d['payload_y'], float)[m], color=EIGHT,
                lw=S.LW_HERO)
        ax.set_aspect('equal')
        ax.set_xlabel('$x$ (m)')
        S.panel(ax, 'ab'[RUNS.index((rid, name, rms))], name)
        ax.annotate(rms, xy=(1, 1), xycoords='axes fraction', xytext=(0, 4), textcoords='offset points',
                    ha='right', va='bottom', fontsize=9, color=S.GREY)
    axs[0].set_ylabel('$y$ (m)')
    S.save(fig, 'fig_s8_paths')


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""The thesis's flight, trajectory and detach figures re-rendered on the 1 m re-fly runs, with the thesis's own figure
code (tools/thesis_sim_figures.py: fig_flight, fig_trajectories, fig_detach) and only the runs and the ring's colour
changed: the ring in black instead of the gold (Wesley 23 Oct; plot_style.PAYLOAD_LINE).

  flight      R2544 (r1m_app_rob_none_int r1; was R2034 at 0.55 m)
  trajectory  R2511 circle (r1m_traj3_orbit_v125 r1; was R1853), R2509 figure-8 (r1m_traj3_fig8_v125 r1; was R1849)
  detach      R2486 (r1m_ta_a1_r1, Tier A A1 at 1 m; was R1887)
The thesis files are untouched: the functions write into a scratch folder and the PDFs and PNGs are copied here.
Two layout fixes the 1 m data need (applied just before the save, in fix()): the flight's height axis to 2.4 m with the
UAV label at the UAVs' hold height (they hover at 1.9 m now, 1.5 m at 0.55 m), and the figure-8's plan view cropped to
its path (the thesis code's ticks are for a wider eight).
"""
import os
import shutil
import subprocess
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import review_style as S  # noqa: E402
import thesis_sim_figures as T  # noqa: E402

T.FLIGHT_RUN = 'R2544'
T.TRAJ_RUNS = dict(circle='R2511', eight='R2509')
T.DETACH_RUN = 'R2486'
T.OUT = os.path.join(S.SCRATCH, 'rerender_1m')

_save = T.ps.save_print
CURRENT = {}


def fix(fig, stem):
    """Per-figure layout fixes for the 1 m data, then the thesis code's own save."""
    axs = fig.get_axes()
    if CURRENT.get('name') == 'fig_flight_1m':
        ax = axs[0]
        top = max(float(np.nanmax(ln.get_ydata())) for ln in ax.get_lines() if len(ln.get_ydata()) > 10)
        ax.set_ylim(0, 2.45)
        ax.set_yticks([0, 0.5, 1.0, 1.5, 2.0])
        for tx in ax.texts:
            if tx.get_text() == 'UAVs':
                tx.xy = (tx.xy[0], top)
                tx.set_position((0, -3))
                tx.set_va('top')
    elif CURRENT.get('name') == 'fig_traj_1m':
        ax = axs[2]                        # the figure-8's plan view (the axes are made circle, its strip, eight)
        x = np.concatenate([ln.get_xdata() for ln in ax.get_lines() if len(ln.get_xdata()) > 10])
        y = np.concatenate([ln.get_ydata() for ln in ax.get_lines() if len(ln.get_ydata()) > 10])
        pad = 0.08
        ax.set_xlim(float(np.min(x)) - pad, float(np.max(x)) + pad)
        ax.set_ylim(float(np.min(y)) - pad, float(np.max(y)) + pad)
        ax.set_xticks([-0.5, 0.0, 0.5])
        ax.set_yticks([-0.25, 0.0, 0.25])
    return _save(fig, stem)


T.ps.save_print = fix

JOBS = ((T.fig_flight, 'fig_flight_1m', 'full'), (T.fig_trajectories, 'fig_traj_1m', 'full'),
        (T.fig_detach, 'fig_detach_1m', 'full'))


def main():
    for fn, name, width in JOBS:
        CURRENT['name'] = name
        stem, stats = fn()
        print(name, {k: (tuple(round(v, 2) for v in s) if isinstance(s, tuple) else round(s, 3))
                     for k, s in stats.items()})
        for ext in ('pdf', 'png'):
            shutil.copy(f'{stem}.{ext}', os.path.join(S.HERE, f'{name}.{ext}'))
        tmp = os.path.join(S.SCRATCH, name + '.pdf')
        shutil.copy(f'{stem}.pdf', tmp)
        out = subprocess.run([sys.executable, S.AUDIT, tmp, '--width', f'{S.WIDTH[width]:.3f}'],
                             capture_output=True, text=True)
        print(out.stdout.strip().splitlines()[-1])


if __name__ == '__main__':
    main()

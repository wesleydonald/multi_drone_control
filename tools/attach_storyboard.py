#!/usr/bin/env python3
"""
tools/attach_storyboard.py — the attach-during-trajectory demo as one thesis figure.

    ./tools/attach_storyboard.py R0209            # -> docs/figures/attach_storyboard_R0209.{png,pdf}
    ./tools/attach_storyboard.py R0209 --out /tmp

Three panels from one run directory (tools/run_experiment.py output):
  (a) plan view: desired vs actual payload path, drone paths, the newcomer's approach,
      with ATTACH / WELD / RESUME marked on the payload path;
  (b) payload tilt and height vs time with the approach-hold and reconfiguration-hold
      bands shaded and the same events as vertical lines;
  (c) the newcomer: height and radial distance from the payload centre vs time — the
      transit that every earlier attempt lost.
Events come from logs/events.csv and the planner's hold messages in logs/launch.log,
so the figure cannot disagree with the run it cites.
"""
import argparse, csv, os, re, sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
try:
    from plot_style import DRONE_COLOURS, apply as apply_style      # noqa: F401
except Exception:                                                     # pragma: no cover
    DRONE_COLOURS = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728']
    def apply_style(): pass

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def find_run(run_id):
    for root in ('results_archive', 'results'):
        for day in sorted(os.listdir(os.path.join(REPO, root)) if os.path.isdir(os.path.join(REPO, root)) else []):
            d = os.path.join(REPO, root, day)
            if not os.path.isdir(d):
                continue
            for name in os.listdir(d):
                if name.startswith(run_id + '_'):
                    return os.path.join(d, name)
    raise SystemExit(f'{run_id}: no run directory under results_archive/ or results/')


def load(run_dir):
    rows = list(csv.DictReader(open(os.path.join(run_dir, 'logs', 'run.csv'))))
    f = lambda k: np.array([float(r[k]) if r.get(k) not in (None, '', 'nan') else np.nan for r in rows])
    t = f('t')
    ev = {}
    for line in open(os.path.join(run_dir, 'logs', 'events.csv')):
        parts = line.strip().split(',')
        if parts and parts[0] != 'sim_time' and parts[0]:
            ev.setdefault(parts[1], float(parts[0]))
    hold_s = None
    lp = os.path.join(run_dir, 'logs', 'launch.log')
    if os.path.exists(lp):
        for line in open(lp):
            m = re.search(r'trajectory held ([0-9.]+) s for the reconfiguration', line)
            if m:
                hold_s = float(m.group(1)); break
    return rows, t, f, ev, hold_s


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('run')
    ap.add_argument('--out', default=os.path.join(REPO, 'docs', 'figures'))
    a = ap.parse_args()
    run_dir = find_run(a.run)
    rows, t, f, ev, hold_s = load(run_dir)
    apply_style()
    n_drones = len([k for k in rows[0] if re.fullmatch(r'd\d+_x', k)])
    t_attach, t_weld = ev.get('MAGNET'), ev.get('WELD')
    t_resume = (t_weld + hold_s) if (t_weld is not None and hold_s) else None

    fig = plt.figure(figsize=(11, 8.5))
    gs = fig.add_gridspec(2, 2, height_ratios=[1.25, 1.0])
    ax = fig.add_subplot(gs[0, :])
    ax.plot(f('payload_ref_x'), f('payload_ref_y'), '--', color='0.4', lw=1.2, label='payload target')
    ax.plot(f('payload_x'), f('payload_y'), color='k', lw=2.0, label='payload')
    for i in range(n_drones):
        ax.plot(f(f'd{i}_x'), f(f'd{i}_y'), color=DRONE_COLOURS[i % len(DRONE_COLOURS)], lw=0.9, alpha=0.8,
                label=f'drone {i}' + (' (newcomer)' if i == n_drones - 1 else ''))
    # the payload is held still through ATTACH/WELD/RESUME, so the three markers sit on
    # top of each other: stagger the labels and draw a leader line to each
    for j, (name, tt, mk) in enumerate((('ATTACH', t_attach, 's'), ('WELD', t_weld, '*'), ('RESUME', t_resume, '^'))):
        if tt is None:
            continue
        k = int(np.nanargmin(np.abs(t - tt)))
        ax.plot(f('payload_x')[k], f('payload_y')[k], mk, ms=12 if mk == '*' else 8, color='k', mfc='w', zorder=5)
        ax.annotate(f'{name} {tt:.0f} s', (f('payload_x')[k], f('payload_y')[k]), textcoords='offset points',
                    xytext=(40, -34 + 16 * j), fontsize=9,
                    arrowprops=dict(arrowstyle='-', color='0.4', lw=0.7))
    ax.set_aspect('equal'); ax.set_xlabel('x [m]'); ax.set_ylabel('y [m]')
    ax.set_title(f'{os.path.basename(run_dir)} — attach during the circle (plan view)')
    ax.legend(loc='upper left', fontsize=8, ncol=2)

    def bands(axis):
        if t_attach is not None and t_weld is not None:
            axis.axvspan(t_attach, t_weld, color='#ffd27f', alpha=0.35, lw=0, label='hold: approach')
        if t_weld is not None and t_resume is not None:
            axis.axvspan(t_weld, t_resume, color='#9fd39f', alpha=0.35, lw=0, label='hold: reconfiguration')
        for tt, ls in ((t_attach, ':'), (t_weld, '-'), (t_resume, '--')):
            if tt is not None:
                axis.axvline(tt, color='k', lw=0.9, ls=ls)

    ax2 = fig.add_subplot(gs[1, 0])
    ax2.plot(t, f('payload_tilt_deg'), color='k', lw=1.4, label='payload tilt [deg]')
    ax2.set_ylabel('tilt [deg]'); ax2.set_xlabel('t [s]')
    ax2b = ax2.twinx()
    ax2b.plot(t, f('payload_z'), color='#1f77b4', lw=1.2, label='payload z [m]')
    ax2b.plot(t, f('payload_ref_z'), ':', color='#1f77b4', lw=1.0)
    ax2b.set_ylabel('z [m]', color='#1f77b4')
    bands(ax2); ax2.set_title('load attitude and height'); ax2.legend(loc='upper right', fontsize=8)

    ax3 = fig.add_subplot(gs[1, 1])
    i = n_drones - 1
    r = np.hypot(f(f'd{i}_x') - f('payload_x'), f(f'd{i}_y') - f('payload_y'))
    ax3.plot(t, f(f'd{i}_z'), color=DRONE_COLOURS[i % len(DRONE_COLOURS)], lw=1.4, label=f'drone {i} z [m]')
    ax3.plot(t, r, color=DRONE_COLOURS[i % len(DRONE_COLOURS)], lw=1.2, ls='--', label=f'drone {i} radius from load [m]')
    ax3.set_xlabel('t [s]'); ax3.set_ylabel('[m]'); bands(ax3)
    ax3.set_title('the newcomer: descent, weld, hand-out'); ax3.legend(loc='upper right', fontsize=8)

    os.makedirs(a.out, exist_ok=True)
    stem = os.path.join(a.out, f'attach_storyboard_{a.run}')
    fig.tight_layout()
    fig.savefig(stem + '.png', dpi=160); fig.savefig(stem + '.pdf')
    print('wrote', stem + '.png')


if __name__ == '__main__':
    main()

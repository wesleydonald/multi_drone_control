#!/usr/bin/env python3
"""One panel per contact event: the tip's height over the plate (plate normal, mm) from -4 s to +2 s around the
logged contact, with the 40 / 20 / 10 mm gate tops and the 0.30 s contact dwell. usage: plot_events.py"""
import csv, os
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
ev = list(csv.DictReader(open(os.path.join(HERE, 'events.csv'))))
n = len(ev)
cols = 5
rows = (n + cols - 1) // cols
fig, axs = plt.subplots(rows, cols, figsize=(3.2 * cols, 2.3 * rows), sharex=True, squeeze=False)
for ax, e in zip(axs.flat, ev):
    w = list(csv.DictReader(open(os.path.join(HERE, 'windows', f"{e['flight']}_try{e['try']}.csv"))))
    t = [float(r['t_rel']) for r in w if -4.0 <= float(r['t_rel']) <= 2.0]
    z = [float(r['normal_mm']) for r in w if -4.0 <= float(r['t_rel']) <= 2.0]
    ok = e['outcome'].startswith('proven')
    ax.plot(t, z, color='tab:blue' if ok else 'tab:red', lw=0.9)
    for g, ls in ((40, '-'), (20, '--'), (10, ':')):
        ax.axhline(g, color='0.4', lw=0.7, ls=ls)
    ax.axhline(0, color='k', lw=0.6)
    ax.axvspan(-0.3, 0.0, color='0.85', zorder=0)
    ax.axvline(0, color='k', lw=0.6)
    ax.set_ylim(-20, 80)
    ax.set_xlim(-4, 2)
    ax.set_title(f"{e['flight'][5:]} try {e['try']}: {e['outcome'].split(' +')[0]}\n"
                 f"contact {float(e['normal_log_mm']):+.1f} mm, 10 mm gate: {e['fires_0.01'].split(' (')[0]}",
                 fontsize=7)
    ax.tick_params(labelsize=7)
for ax in axs.flat[n:]:
    ax.axis('off')
for ax in axs[:, 0]:
    ax.set_ylabel('tip over plate (mm)', fontsize=7)
for ax in axs[-1, :]:
    ax.set_xlabel('time from the logged contact (s)', fontsize=7)
fig.suptitle('Every rig contact under the co-motion rule (blue: proven, red: proof failed); lines: gate top 40 / 20 / '
             '10 mm; grey: the 0.30 s dwell', fontsize=9)
fig.tight_layout(rect=(0, 0, 1, 0.97))
out = os.path.join(HERE, 'contact_events.png')
fig.savefig(out, dpi=130)
print(out)

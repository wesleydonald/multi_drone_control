"""Real-time replays of the 12 Nov circle flights through Te-Jen's backend, by setting: time to reach the drop point
(capture), the ring's speed then, QUAD4's highest point, the closest carrier tether before capture, and how long QUAD4
stood still near a tether before capture (full/dist.py's stranding). Run from results/analysis/cpp_realtime_2026-11-13/:
python3 ../cpp_failure_figures_2026-11-13/swing3_spline2/compare_settings.py"""
import csv, os, re, subprocess
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
OUT = os.path.dirname(os.path.abspath(__file__))
OFF = {'nro': 66.0, 'fm9hc_2': 55.0, 'fm9hc_4': 70.0, 'fm9hc_5': 47.0, 'fm9_5': 135.0}
SETS = [('swing 5, spline 4, splice 0.10 (refresh 0.2)', ['nro_p02', 'fm9hc_2_r02', 'fm9hc_4_r02', 'fm9hc_5_r02']),
        ('swing 5, spline 4, splice 0.20', ['nro_mv_z29s20', 'nro_mv_z38s20', 'nro_mv_z38s20_r2', 'fm9hc_2_mv_z38s20', 'fm9hc_4_mv_z38s20', 'fm9hc_5_mv_z38s20']),
        ('swing 5, spline 3, splice 0.20', ['nro_mv_z29s20sp3', 'nro_mv_z29s20sp3_r2', 'nro_mv_z29s20sp3_load8', 'nro_mv_z38s20sp3', 'fm9hc_2_mv_z29s20sp3', 'fm9hc_4_mv_z29s20sp3', 'fm9hc_5_mv_z29s20sp3']),
        ('swing 5, spline 2, splice 0.20', ['nro_mv_z38s20sp2']),
        ('swing 3, spline 2.5, splice 0.20 (Te-Jen)', ['nro_tj_sw3_sp25', 'nro_tj_sw3_sp25_r2', 'fm9hc_2_tj_sw3_sp25', 'fm9hc_4_tj_sw3_sp25', 'fm9hc_5_tj_sw3_sp25']),
        ('swing 3, spline 2, splice 0.20 (Te-Jen)', ['nro_tj_sw3_sp20', 'nro_tj_sw3_sp20_r2', 'fm9hc_2_tj_sw3_sp20', 'fm9hc_4_tj_sw3_sp20', 'fm9hc_5_tj_sw3_sp20',
                                                    'nro_tj_sw3_sp20_load8', 'fm9hc_4_tj_sw3_sp20_load8', 'fm9hc_5_tj_sw3_sp20_load8'])]
def one(run):
    d = f'full/runs/{run}'
    if not os.path.exists(f'{d}/driver.csv'):
        return None
    rows = list(csv.DictReader(open(f'{d}/driver.csv')))
    t = np.array([float(r['t']) for r in rows])
    auth = next((float(r['t']) for r in rows if r['auth'] == '1'), None)
    cap = next((float(r['t']) for r in rows if r['captured'] == '1'), None)
    end = cap or t[-1]
    m = (t >= auth) & (t <= end)
    z = np.array([float(r['z']) for r in rows])
    ct = np.array([float(r['min_cable_tether_m']) if r['min_cable_tether_m'] not in ('', 'nan') else np.nan for r in rows])
    rx = np.array([[float(r['ring_x']), float(r['ring_y'])] for r in rows])
    v = np.r_[0, np.linalg.norm(np.diff(rx, axis=0), axis=1) / np.maximum(np.diff(t), 1e-3)]
    v = np.convolve(v, np.ones(25) / 25, mode='same')
    flight = next(f for f in OFF if run.startswith(f + '_') or run == f)
    out = subprocess.run(['python3', 'full/dist.py', run, flight, str(OFF[flight])], capture_output=True, text=True).stdout
    mm = re.search(r'before capture ([0-9.]+) s(?: from \+[0-9.]+ s, min ([0-9.]+) m)?', out)
    still = float(mm.group(1)) if mm else float('nan')
    still_min = float(mm.group(2)) if mm and mm.group(2) else float('nan')
    return dict(run=run, flight=flight, captured=cap is not None, t_cap=(cap - auth) if cap else (end - auth),
                ring_v=float(v[np.argmin(np.abs(t - cap))]) if cap else float('nan'), z_max=float(z[m].max()),
                tether_min=float(np.nanmin(ct[m])), still_s=still, still_min=still_min, load='load8' in run)
res = [(name, [r for r in (one(x) for x in runs) if r]) for name, runs in SETS]
with open(os.path.join(OUT, 'compare_settings.csv'), 'w', newline='') as f:
    w = csv.writer(f, lineterminator='\n'); w.writerow(['setting', 'run', 'captured', 'time_s', 'ring_speed_at_capture', 'z_max', 'closest_tether_before_capture_m', 'still_near_tether_before_capture_s', 'still_min_m', 'cpu_load'])
    for name, rs in res:
        for r in rs:
            w.writerow([name, r['run'], r['captured'], round(r['t_cap'], 1), round(r['ring_v'], 3), round(r['z_max'], 2), round(r['tether_min'], 2), r['still_s'], r['still_min'], r['load']])
fig, axs = plt.subplots(1, 3, figsize=(16, 5.5), sharey=True)
for y, (name, rs) in enumerate(res):
    for j, r in enumerate(rs):
        yy = y + (j - len(rs) / 2) * 0.07
        moving = r['captured'] and r['ring_v'] > 0.08
        col = '#1b9e77' if moving else ('#e6ab02' if r['captured'] else '#d95f02')
        mk = 'D' if r['load'] else ('o' if r['flight'] == 'nro' else 's')
        axs[0].scatter(r['t_cap'] if r['captured'] else 95, yy, c=col, marker=mk, edgecolors='k', s=45, zorder=3)
        axs[1].scatter(r['tether_min'], yy, c=col, marker=mk, edgecolors='k', s=45, zorder=3)
        axs[2].scatter(r['still_s'] if r['still_s'] == r['still_s'] else 0, yy, c=col, marker=mk, edgecolors='k', s=45, zorder=3)
axs[0].axvline(90, color='k', ls='--', lw=0.8); axs[0].text(91, len(res) - 0.6, 'never', fontsize=8)
axs[0].set_xlim(0, 100); axs[0].set_xlabel('time from his authority to capture (s, real time)')
axs[1].set_xlabel('closest carrier tether to QUAD4 hanging line before capture (m, centre lines)'); axs[1].axvline(0.1, color='r', ls=':', lw=0.8)
axs[2].set_xlabel('QUAD4 standing still near a tether before capture (s)')
axs[0].set_yticks(range(len(res))); axs[0].set_yticklabels([n for n, _ in res], fontsize=9)
for a in axs: a.grid(alpha=0.3)
from matplotlib.lines import Line2D
h = [Line2D([], [], marker='o', ls='', color='#1b9e77', markeredgecolor='k', label='captured while the ring moved (> 0.08 m/s)'),
     Line2D([], [], marker='o', ls='', color='#e6ab02', markeredgecolor='k', label='captured after the ring slowed'),
     Line2D([], [], marker='o', ls='', color='#d95f02', markeredgecolor='k', label='never captured'),
     Line2D([], [], marker='o', ls='', color='w', markeredgecolor='k', label='nro (90 s continuous circle)'),
     Line2D([], [], marker='s', ls='', color='w', markeredgecolor='k', label='fm9hc_2/_4/_5 (circle windows)'),
     Line2D([], [], marker='D', ls='', color='w', markeredgecolor='k', label='with 8 busy CPU loops')]
fig.legend(handles=h, loc='lower center', ncol=3, fontsize=8, bbox_to_anchor=(0.55, -0.06))
fig.suptitle('Moving ring, real-time replays of the 12 Nov circle flights through his backend (open loop: no swing, perfect tracking)\n'
             'all with outer_octagon, tracking 0.06, 30 steps, refresh 0.2 s; one dot per replay', fontsize=10)
fig.tight_layout(rect=(0, 0.04, 1, 0.95))
fig.savefig(os.path.join(OUT, 'moving_ring_by_setting.png'), dpi=150, bbox_inches='tight')
print('ok')

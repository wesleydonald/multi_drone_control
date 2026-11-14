"""Every twin run of Te-Jen's C++ planner on 12-13 Nov (t13_*cpp* in results/2026-11-12 and 2026-11-13): its settings
(from the LAUNCH line in events.csv) against what happened (his phases, run.csv). One row per run -> runs_table.csv.
Run from the repo root: python3 results/analysis/cpp_failure_figures_2026-11-13/overview/runs_table.py"""
import csv, glob, os, re, sys
import numpy as np
sys.path.insert(0, 'tools')
import attach_clearance as A
import m1_metrics as M
OUT = os.path.dirname(os.path.abspath(__file__))
def arg(line, k, default=''):
    m = re.search(rf'\b{k}:=(\S+)', line)
    return m.group(1) if m else default
rows = []
for d in sorted(glob.glob('results/2026-11-1[23]/R4*_sim_gz_t13_*cpp*')):
    rid = os.path.basename(d).split('_')[0]
    if not os.path.exists(os.path.join(d, 'logs', 'events.csv')):
        print('skip', d, 'no events.csv'); continue
    ev = [l.rstrip('\n').split(',', 2) for l in open(os.path.join(d, 'logs', 'events.csv'))][1:]
    launch = next((e[2] for e in ev if len(e) > 2 and e[1] == 'LAUNCH'), '')
    name = os.path.basename(d).split('_sim_gz_')[1]
    try:
        r = M.Run(rid)
    except Exception as exc:
        print('skip', rid, exc); continue
    t = r.tt
    tk = r.t('PHASE:TAKEOFF')
    tr = r.t('PHASE:TRANSIT_TO_DROP_POINT', after=tk) if tk else None
    st = r.t('PHASE:SETTLE_ABOVE_DROP_POINT', after=tr) if tr else None
    end = st or r.t('LAND', after=tk) or float(t[-1])
    ref = r.xyz('payload_ref_')[:, :2]
    v = np.r_[0, np.linalg.norm(np.diff(ref, axis=0), axis=1) / np.maximum(np.diff(t), 1e-6)]
    k = max(1, int(round(0.5 / max(np.median(np.diff(t)), 1e-3)))); v = np.convolve(v, np.ones(k) / k, mode='same')
    win = (t >= tr) & (t < end) if tr else np.zeros(len(t), bool)
    z = r.xyz(r.k)[:, 2]
    c = A.clearance(r, win) if win.any() else None
    q4 = r.xyz(r.k)[:, :2]; ring = r.xyz('payload_')[:, :2]
    dist_end = float(np.linalg.norm(q4[win][-1] - ring[win][-1])) if win.any() else float('nan')
    traj = name.split('_')[1]
    rows.append(dict(run=rid, name=name, traj=traj, start=name.split('_')[2],
        shape=arg(launch, 'swing_envelope', 'legacy_square (default)'),
        swing_deg=round(float(arg(launch, 'max_swing_angle_rad', 'nan')) * 57.2958, 1) if arg(launch, 'max_swing_angle_rad') else 'default',
        spline=arg(launch, 'spline_time_factor', 'default'), tracking_m=arg(launch, 'cooperative_tracking_half_m', 'default'),
        cap_m=arg(launch, 'cpp_max_z', 'none'), workspace=arg(launch, 'workspace_xy', 'none'), refresh_s=arg(launch, 'cpp_world_refresh', 'none'),
        splice_s=arg(launch, 'cpp_splice', '0.10 (default)'), tti=arg(launch, 'target_time_iterations', 'default'),
        paused='yes' if 'pausetakeoff' in name else 'no',
        ring_speed_at_transit=round(float(v[np.argmin(np.abs(t - tr))]), 3) if tr else '',
        cpp_started='yes' if tr else 'no',
        reached_drop='yes' if st else 'no',
        transit_s=round(st - tr, 1) if (tr and st) else '',
        z_max_transit=round(float(z[win].max()), 2) if win.any() else '',
        closest_in_transit_m=c['min_m'] if c else '', closest_pair=(c['pair'] if c else ''),
        quad4_to_ring_centre_at_end_m=round(dist_end, 2),
        rtf=round(float(r.man.get('mean_rtf') or 0), 3) if hasattr(r, 'man') else ''))
with open(os.path.join(OUT, 'runs_table.csv'), 'w', newline='') as f:
    w = csv.DictWriter(f, fieldnames=list(rows[0].keys()), lineterminator='\n'); w.writeheader(); w.writerows(rows)
print(len(rows), 'runs')

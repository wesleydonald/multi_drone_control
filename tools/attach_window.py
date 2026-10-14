#!/usr/bin/env python3
"""How well QUAD4's swung magnet sits over its plate: the attach window of a rig flight, per phase of his planner.

    python3 tools/attach_window.py results/rig/2026-10-14/t6h_4 [more runs]
    python3 tools/attach_window.py RUN --from T --to T                # a window by hand (bags without his phase)
    python3 tools/attach_window.py RUN --csv out.csv

Per window (one per visit of his MATCH_VELOCITY, DESCEND_TO_ATTACHMENT, ATTACH_READY or LIFT_PROOF):
- tip error: the tip marker against the plate point, horizontal in the level frame (ring yaw only), mean offset,
  median and p90; inside the catch radius (K) and inside-and-slow (relative speed under --slow, 9-sample mean),
  the fraction of time the magnet could catch;
- body wander: QUAD4's body xy sd; swing: tip about the body, rms about its mean; plate motion: the plate point's xy sd;
- ring tilt mean and max;
- forcing (a pendulum, tip hanging L_eff below the body): body acceleration rms, and the rest of the tip's
  acceleration (air on the tip, rms) from r'' + (g/L) r + a_body, both after a 4 Hz low-pass.
The definitions are the 14 Oct analyses' (docs/plan_after_rig_2026-10-14.md, "What the data says").
"""
import argparse
import csv
import glob
import math
import os
import sqlite3
import sys

import numpy as np

G = 9.81
PLATE_XY = (0.013, 0.210)       # QUAD4's plate (plate 9, Tejen's 3) in the ring frame: test 3's catch centre (RIG-1014-t3)
CATCH_K = 0.024                 # m, the catch radius K (RIG-1014-t3)
SLOW = 0.10                     # m/s, tip speed relative to the plate for a catch
HOLD_PHASES = ('MATCH_VELOCITY', 'DESCEND_TO_ATTACHMENT', 'ATTACH_READY', 'LIFT_PROOF')
MIN_WINDOW_S = 0.5


def _quat_yaw(q):
    x, y, z, w = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def _tilt_deg(q):
    x, y = q[0], q[1]
    return math.degrees(math.acos(max(-1.0, min(1.0, 1.0 - 2.0 * (x * x + y * y)))))


def read_bag(run):
    """{'tip', 'ring', 'body': arrays [t, x, y, z(, qx, qy, qz, qw)], 'phases': [(t, phase)]} from the run's bag."""
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    db = sorted(glob.glob(os.path.join(run, '*.db3')))
    if not db:
        raise SystemExit(f'no bag in {run}')
    c = sqlite3.connect(db[0])
    tops = {n: (i, ty) for i, n, ty in c.execute('select id,name,type from topics')}

    def poses(name, quat=False):
        if name not in tops:
            return np.zeros((0, 8 if quat else 4))
        i, ty = tops[name]
        cls = get_message(ty)
        out = []
        for ts, d in c.execute('select timestamp,data from messages where topic_id=? order by timestamp', (i,)):
            m = deserialize_message(d, cls)
            p, q = m.pose.position, m.pose.orientation
            out.append((ts * 1e-9, p.x, p.y, p.z) + ((q.x, q.y, q.z, q.w) if quat else ()))
        return np.array(out) if out else np.zeros((0, 8 if quat else 4))

    phases = []
    if '/join_planner/phase' in tops:
        i, ty = tops['/join_planner/phase']
        cls = get_message(ty)
        for ts, d in c.execute('select timestamp,data from messages where topic_id=? order by timestamp', (i,)):
            p = deserialize_message(d, cls).data
            if not phases or phases[-1][1] != p:
                phases.append((ts * 1e-9, p))
    return {'tip': poses('/magnet_tip_pose'), 'ring': poses('/payload/motion_capture_state', True),
            'body': poses('/drone_3/motion_capture_state'), 'phases': phases}


def phase_windows(phases, end_t, wanted=HOLD_PHASES, min_s=MIN_WINDOW_S):
    """[(t0, t1, phase)] for each visit of a wanted phase, at least min_s long."""
    out = []
    for k, (t, p) in enumerate(phases):
        t1 = phases[k + 1][0] if k + 1 < len(phases) else end_t
        if p in wanted and t1 - t >= min_s:
            out.append((t, t1, p))
    return out


def _lowpass(x, dt, hz=4.0):
    from scipy.signal import butter, filtfilt
    b, a = butter(2, hz / (0.5 / dt))
    return filtfilt(b, a, x, axis=0) if len(x) > 12 else x


def window_metrics(data, t0, t1, plate_xy=PLATE_XY, catch=CATCH_K, slow=SLOW):
    """The window's numbers (dict; None where a stream is missing)."""
    tip, ring, body = data['tip'], data['ring'], data['body']
    tw = tip[(tip[:, 0] >= t0) & (tip[:, 0] <= t1)] if len(tip) else tip
    rw = ring[(ring[:, 0] >= t0 - 0.2) & (ring[:, 0] <= t1 + 0.2)] if len(ring) else ring
    out = {'t0': round(t0, 2), 'dur_s': round(t1 - t0, 2), 'n_tip': int(len(tw))}
    if len(tw) < 10 or len(rw) < 10:
        return out
    j = np.clip(np.searchsorted(rw[:, 0], tw[:, 0]), 0, len(rw) - 1)
    yaw = np.array([_quat_yaw(q) for q in rw[j, 4:8]])
    cy, sy = np.cos(yaw), np.sin(yaw)
    px = rw[j, 1] + cy * plate_xy[0] - sy * plate_xy[1]
    py = rw[j, 2] + sy * plate_xy[0] + cy * plate_xy[1]
    d = np.stack([tw[:, 1] - px, tw[:, 2] - py], 1)
    e = np.hypot(d[:, 0], d[:, 1])
    dt = np.diff(tw[:, 0])
    ok = dt > 1e-3
    v = np.r_[0.0, np.linalg.norm(np.diff(d, axis=0) / np.where(ok, dt, 1.0)[:, None], axis=1)]
    v = np.convolve(v, np.ones(9) / 9, mode='same')
    tilt = np.array([_tilt_deg(q) for q in rw[:, 4:8]])
    plate = np.stack([px, py], 1)
    out.update({'offset_x_cm': 100 * d[:, 0].mean(), 'offset_y_cm': 100 * d[:, 1].mean(),
                'err_median_cm': 100 * float(np.median(e)), 'err_p90_cm': 100 * float(np.percentile(e, 90)),
                'inside_pct': 100 * float(np.mean(e < catch)), 'inside_slow_pct': 100 * float(np.mean((e < catch) & (v < slow))),
                'rel_speed_median': float(np.median(v)), 'plate_sd_cm': 100 * float(np.mean(plate.std(0))),
                'tilt_mean_deg': float(tilt.mean()), 'tilt_max_deg': float(tilt.max())})
    bw = body[(body[:, 0] >= t0 - 2.0) & (body[:, 0] <= t1 + 2.0)] if len(body) else body
    if len(bw) < 20:
        return out
    t = np.arange(max(tw[0, 0], bw[0, 0], t0 - 2.0), min(tip[-1, 0], bw[-1, 0], t1 + 2.0), 0.01)
    tt = tip[(tip[:, 0] >= t0 - 2.0) & (tip[:, 0] <= t1 + 2.0)]
    if len(t) < 50 or len(tt) < 20:
        return out
    T = _lowpass(np.stack([np.interp(t, tt[:, 0], tt[:, k]) for k in (1, 2, 3)], 1), 0.01)
    B = _lowpass(np.stack([np.interp(t, bw[:, 0], bw[:, k]) for k in (1, 2, 3)], 1), 0.01)
    L = float(np.median(B[:, 2] - T[:, 2]))
    r = T[:, :2] - B[:, :2]

    def d2(x):
        return np.gradient(np.gradient(x, 0.01, axis=0), 0.01, axis=0)
    ab = d2(B[:, :2])
    aero = d2(r) + (G / max(L, 0.05)) * r + ab
    w = (t >= t0) & (t <= t1)

    def rms(x):
        return float(np.sqrt((np.linalg.norm(x[w] - x[w].mean(0), axis=1) ** 2).mean())) if w.any() else math.nan
    out.update({'body_sd_cm': 100 * float(np.mean(B[w, :2].std(0))) if w.any() else math.nan,
                'swing_rms_cm': 100 * rms(r), 'l_eff_m': L, 'body_acc_rms': rms(ab), 'air_acc_rms': rms(aero)})
    return out


COLUMNS = [('phase', 'phase', '{}'), ('dur_s', 's', '{:.1f}'), ('err_median_cm', 'err med cm', '{:.1f}'),
           ('err_p90_cm', 'p90', '{:.1f}'), ('offset_x_cm', 'off x', '{:+.1f}'), ('offset_y_cm', 'off y', '{:+.1f}'),
           ('inside_pct', 'in K %', '{:.0f}'), ('inside_slow_pct', 'in+slow %', '{:.0f}'),
           ('body_sd_cm', 'body sd', '{:.1f}'), ('swing_rms_cm', 'swing', '{:.1f}'), ('plate_sd_cm', 'plate sd', '{:.1f}'),
           ('tilt_mean_deg', 'tilt', '{:.1f}'), ('tilt_max_deg', 'max', '{:.1f}'),
           ('body_acc_rms', 'a body', '{:.2f}'), ('air_acc_rms', 'a air', '{:.2f}')]


def _fmt(row):
    cells = []
    for key, _, f in COLUMNS:
        v = row.get(key)
        cells.append('-' if v is None or (isinstance(v, float) and not math.isfinite(v)) else f.format(v))
    return cells


def score(run, t_from=None, t_to=None, plate_xy=PLATE_XY, catch=CATCH_K, slow=SLOW):
    data = read_bag(run)
    end = max(a[-1, 0] for a in (data['tip'], data['ring'], data['body']) if len(a)) if any(
        len(data[k]) for k in ('tip', 'ring', 'body')) else 0.0
    if t_from is not None or t_to is not None:
        start = data['tip'][0, 0] if len(data['tip']) else 0.0
        windows = [(t_from if t_from is not None else start, t_to if t_to is not None else end, 'by hand')]
    else:
        windows = phase_windows(data['phases'], end)
    rows = []
    for t0, t1, p in windows:
        m = window_metrics(data, t0, t1, plate_xy, catch, slow)
        m['phase'] = p
        m['run'] = os.path.basename(os.path.normpath(run))
        rows.append(m)
    return rows


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('run', nargs='+', help='rig run folder(s) with the bag')
    ap.add_argument('--from', dest='t_from', type=float)
    ap.add_argument('--to', dest='t_to', type=float)
    ap.add_argument('--plate', nargs=2, type=float, default=PLATE_XY, metavar=('X', 'Y'),
                    help='the plate point in the ring frame (m)')
    ap.add_argument('--catch', type=float, default=CATCH_K, help='the catch radius K (m)')
    ap.add_argument('--slow', type=float, default=SLOW, help='relative speed for a catch (m/s)')
    ap.add_argument('--csv', help='also write every window to this CSV')
    a = ap.parse_args(argv)
    all_rows = []
    for run in a.run:
        rows = score(run, a.t_from, a.t_to, tuple(a.plate), a.catch, a.slow)
        all_rows += rows
        print(f'== {run}: {len(rows)} window(s); plate ({a.plate[0]:.3f}, {a.plate[1]:.3f}) m, K {100 * a.catch:.1f} cm, '
              f'slow < {a.slow:.2f} m/s')
        if not rows:
            print('   no window (no hold phase in the bag: give --from/--to)')
            continue
        table = [[h for _, h, _ in COLUMNS]] + [_fmt(r) for r in rows]
        widths = [max(len(r[k]) for r in table) for k in range(len(COLUMNS))]
        for r in table:
            print('   ' + '  '.join(c.rjust(wd) for c, wd in zip(r, widths)))
    if a.csv and all_rows:
        keys = ['run', 'phase', 't0'] + [k for k, _, _ in COLUMNS if k != 'phase'] + ['rel_speed_median', 'l_eff_m',
                                                                                        'n_tip']
        with open(a.csv, 'w', newline='') as f:
            wr = csv.DictWriter(f, fieldnames=keys, extrasaction='ignore', lineterminator='\n')
            wr.writeheader()
            wr.writerows(all_rows)
        print(f'wrote {a.csv}')
    return 0


if __name__ == '__main__':
    sys.exit(main())

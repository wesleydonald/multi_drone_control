#!/usr/bin/env python3
"""W2b: RIG-1029-k1_4's SETTLE_ABOVE_PICKUP (26 s, never captured), his capture predicate replayed per 30 Hz tick
as flown (xy 0.025 m, speed 0.07 m/s, dwell 0.5 s) and with the typed next trial (xy 0.06, speed 0.15, dwell 0.5).

His predicate (online_join_planner.update_pickup_tier_alignment -> measured_tip_alignment.TipAlignment.step, phase
'capture'), all at once, unbroken for the dwell on his planner clock, reset on any failing tick:
  * xy:    |pickup point xy - magnet face xy| <= pickup_alignment_capture_xy_m; face = measured tip marker
           (/tejen/magnet_tip_pose) + marker_to_contact_face (vertical, -0.0223 m here), pickup point = the object
           (/pickup/motion_capture_state) + 0.012 m up
  * speed: |v| <= pickup_alignment_max_relative_speed_mps, v the 3-D velocity of (face - pickup point), updated only
           when the tip stamp has advanced: v = 0.35 (delta / dt) + 0.65 v (his .35/.65 filter, per planner tick)
  * clearance: face z - pickup point z >= pickup_alignment_min_normal_m 0.08 (0.26-0.33 m here: always true)
  * fresh: once a velocity sample exists; the magnet is not ON in SETTLE.
Checked against his 5 Hz CSV (pickup_alignment_xy_m, _clearance_m, and the waiting reason).
"""
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import OUT, hold, join_planner_csv, pose_array, read_bag  # noqa: E402

RUN, JP = 'k1_4', '2026-10-29_10-39-12'
FACE_DZ, POINT_DZ, MIN_NORMAL = -0.0223, 0.012, 0.08
SETTINGS = {'as flown (0.025 m, 0.07 m/s, 0.5 s)': (0.025, 0.07, 0.5),
            'next trial (0.06 m, 0.15 m/s, 0.5 s)': (0.06, 0.15, 0.5),
            'xy 0.06 only (speed 0.07)': (0.06, 0.07, 0.5),
            'speed 0.15 only (xy 0.025)': (0.025, 0.15, 0.5)}


def replay(ticks, tip, obj, settings):
    rel_all, xy, clr, spd, vh, vz = [], [], [], [], [], []
    v, last = np.zeros(3), None
    for t in ticks:
        j = int(np.searchsorted(tip[:, 0], t, side='right') - 1)
        k = int(np.searchsorted(obj[:, 0], t, side='right') - 1)
        face = tip[j, 1:4] + np.array([0, 0, FACE_DZ])
        point = obj[k, 1:4] + np.array([0, 0, POINT_DZ])
        rel = face - point
        stamp = tip[j, 0]
        if last is not None and stamp > last[0] + 1e-6:
            v = 0.35 * (rel - last[1]) / (stamp - last[0]) + 0.65 * v
        if last is None or stamp > last[0] + 1e-6:
            last = (stamp, rel.copy())
        rel_all.append(rel)
        xy.append(float(np.linalg.norm(rel[:2])))
        clr.append(float(rel[2]))
        spd.append(float(np.linalg.norm(v)))
        vh.append(float(np.linalg.norm(v[:2])))
        vz.append(float(abs(v[2])))
    xy, clr, spd = map(np.array, (xy, clr, spd))
    out = {}
    for name, (cxy, cv, dwell) in settings.items():
        ok = (xy <= cxy) & (spd <= cv) & (clr >= MIN_NORMAL)
        ok[0] = False                              # no velocity sample yet
        first, since, closes, held_s = None, None, 0, []
        run_start = None
        for i, t in enumerate(ticks):
            if ok[i]:
                since = t if since is None else since
                run_start = t if run_start is None else run_start
                if t - since >= dwell - 1e-9:
                    first = t - ticks[0] if first is None else first
                    closes += 1
                    since = None                   # count separate closures: restart the dwell after each
            else:
                if run_start is not None:
                    held_s.append(ticks[i - 1] - run_start + 1 / 30)
                since, run_start = None, None
        if run_start is not None:
            held_s.append(ticks[-1] - run_start + 1 / 30)
        held_s = np.array(held_s) if held_s else np.zeros(1)
        out[name] = dict(xy=cxy, speed=cv, dwell=dwell, first_close_s=first, closures=closes,
                         all_true_frac=float(ok.mean()), xy_true_frac=float(np.mean(xy <= cxy)),
                         speed_true_frac=float(np.mean(spd <= cv)),
                         runs_ge_dwell=int(np.sum(held_s >= dwell - 1e-9)), longest_run_s=float(held_s.max()),
                         median_run_s=float(np.median(held_s)))
    replay.vh, replay.vz = np.array(vh), np.array(vz)
    return out, xy, clr, spd


def main():
    b = read_bag(RUN, ['/tejen/magnet_tip_pose', '/pickup/motion_capture_state', '/join_planner/reference',
                       '/join_planner/phase'])
    tip, obj = pose_array(b['/tejen/magnet_tip_pose']), pose_array(b['/pickup/motion_capture_state'])
    phases = []
    for t, m in b['/join_planner/phase']:
        if not phases or phases[-1][1] != m.data:
            phases.append((t, m.data))
    a = next(t for t, p in phases if p == 'SETTLE_ABOVE_PICKUP')
    z = next(t for t, p in phases if t > a and p != 'SETTLE_ABOVE_PICKUP')
    ticks = np.array([t for t, _ in b['/join_planner/reference'] if a <= t <= z])
    res, xy, clr, spd = replay(ticks, tip, obj, SETTINGS)
    # check against his 5 Hz CSV
    jp = join_planner_csv(JP)
    x = jp[jp.phase == 'SETTLE_ABOVE_PICKUP']
    idx = np.clip(np.searchsorted(ticks, x.time_sec.to_numpy(float), side='right') - 1, 0, len(ticks) - 1)
    dxy = np.abs(xy[idx] - x.pickup_alignment_xy_m.to_numpy(float))
    dcl = np.abs(clr[idx] - x.pickup_alignment_clearance_m.to_numpy(float))
    reason = x.pickup_alignment_waiting_reason.astype(str).to_numpy()
    mine = np.where(spd[idx] > 0.07, 'relative_speed', np.where(xy[idx] > 0.025, 'capture_xy', 'capture_dwell'))
    sel = np.isin(reason, ['relative_speed', 'capture_xy', 'capture_dwell'])
    check = dict(n=int(len(x)), xy_diff_median_cm=100 * float(np.median(dxy)), xy_diff_p95_cm=100 * float(np.quantile(dxy, .95)),
                 clearance_diff_median_cm=100 * float(np.median(dcl)),
                 reason_agree=float(np.mean(mine[sel] == reason[sel])),
                 his_reasons={k: int(v) for k, v in zip(*np.unique(reason, return_counts=True))})
    stats = dict(settle_s=float(z - a), xy_median_cm=100 * float(np.median(xy)), xy_p90_cm=100 * float(np.quantile(xy, .9)),
                 speed_median=float(np.median(spd)), speed_p90=float(np.quantile(spd, .9)),
                 clearance_min=float(clr.min()), speed_xy_part_median=float(np.median(replay.vh)),
                 speed_z_part_median=float(np.median(replay.vz)), corr_xy_speed=float(np.corrcoef(xy, spd)[0, 1]),
                 speed_median_when_xy_le_2p5cm=float(np.median(spd[xy <= 0.025])) if np.any(xy <= 0.025) else None,
                 speed_median_when_xy_gt_4cm=float(np.median(spd[xy > 0.04])) if np.any(xy > 0.04) else None)
    out = dict(check=check, stats=stats, settings=res)
    with open(os.path.join(OUT, 'w2b_pickup_capture.json'), 'w') as f:
        json.dump(out, f, indent=1)
    print('check', check)
    print('stats', {k: (round(v, 3) if isinstance(v, float) else v) for k, v in stats.items()})
    for k, v in res.items():
        print(k, {kk: (round(vv, 3) if isinstance(vv, float) else vv) for kk, vv in v.items()})


if __name__ == '__main__':
    main()

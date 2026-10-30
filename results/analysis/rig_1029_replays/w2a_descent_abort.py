#!/usr/bin/env python3
"""W2a: RIG-1029-sq5_2's six DESCEND_TO_ATTACHMENT aborts, rebuilt per 30 Hz tick, and a 0.30 s debounce replayed.

His contact-mode abort (online_join_planner, the flown tree: no debounce):
    too_far = xy_error > contact_abort_xy_threshold (rig 0.08) or xy_relative_velocity > ..._velocity_threshold (0.25)
from tracking_errors(): xy_error = |measured tip - (target + desired_tip_offset)| in xy, the target the ring bridge's
attachment point (ring pose + R(q) (X, Y, W), X -0.005, Y 0.197, W 0.028, sampled and held at 50 Hz), the offset
approach_offset_x/y (0; his measured-tip trim moves the reference, not this target: his logged tip_target is the
plain plate point to 0.25 cm p95); xy_relative_velocity =
|QUAD4's mocap velocity - the attachment twist| in xy, the twist zero on the rig (attach_twist zero).
Tip = /tejen/magnet_tip_pose (his relay of /magnet_tip_pose), held. Ticks = his /join_planner/reference messages
(one per 30 Hz update). Checked against his own 5 Hz CSV (tip_error_xy, relative_speed_xy) and his six abort times.

Debounce replay: abort only after 0.30 s of continuous violation, timed on the tip samples, reset on recovery (as his
new contact_position_lost). Past the real abort QUAD4 flew his re-approach (up 0.3 m, same xy target), so what the
error does after the real abort is the recorded swing, not what a continued descent would have flown.
"""
import json
import os
import sys

import numpy as np
from scipy.spatial.transform import Rotation as Rot

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import OUT, hold, join_planner_csv, pose_array, read_bag  # noqa: E402

RUN, JP = 'sq5_2', '2026-10-29_10-28-45'
PLATE = np.array([-0.005, 0.197, 0.028])
XY_ABORT, V_ABORT, DEBOUNCE, BRIDGE_DT = 0.08, 0.25, 0.30, 0.02
DESCENT_RATE, START_Z, END_Z = 0.08, 0.30, -0.01      # his rate profile (rate_profile_duration_s 3.875 in his CSV)


def main():
    b = read_bag(RUN, ['/tejen/magnet_tip_pose', '/payload/motion_capture_state', '/drone_3/motion_capture_state',
                       '/join_planner/reference', '/join_planner/phase'])
    tip, ring, q4 = (pose_array(b[k]) for k in ('/tejen/magnet_tip_pose', '/payload/motion_capture_state',
                                                 '/drone_3/motion_capture_state'))
    ticks = np.array([t for t, _ in b['/join_planner/reference']])
    phases = []
    for t, m in b['/join_planner/phase']:
        if not phases or phases[-1][1] != m.data:
            phases.append((t, m.data))
    jp = join_planner_csv(JP)
    # the bridge: latest ring pose every 20 ms, plate point through the full attitude
    tb = np.arange(ring[0, 0], ring[-1, 0], BRIDGE_DT)
    rr = hold(ring, tb)
    plate = rr[:, 1:4] + Rot.from_quat(rr[:, 4:8]).apply(PLATE)
    bridge = np.column_stack([tb, plate])

    def errors(tq):
        tgt = hold(bridge, tq)[:, 1:3]
        e = np.linalg.norm(hold(tip, tq)[:, 1:3] - tgt, axis=1)
        v = np.linalg.norm(hold(q4, tq)[:, 8:10], axis=1)
        tip_t = hold(tip, tq)[:, 0]
        return e, v, tip_t

    # check against his 5 Hz CSV
    e5, v5, _ = errors(jp.time_sec.to_numpy(float))
    ok = np.isfinite(jp.tip_error_xy.to_numpy(float)) & (jp.phase.isin(['MATCH_VELOCITY', 'DESCEND_TO_ATTACHMENT']))
    de = e5[ok] - jp.tip_error_xy.to_numpy(float)[ok]
    dv = v5[ok] - jp.relative_speed_xy.to_numpy(float)[ok]
    check = dict(n=int(ok.sum()), err_diff_median_cm=100 * float(np.median(np.abs(de))),
                 err_diff_p95_cm=100 * float(np.quantile(np.abs(de), 0.95)),
                 speed_diff_median=float(np.median(np.abs(dv))), speed_diff_p95=float(np.quantile(np.abs(dv), 0.95)))
    print('check vs his CSV:', check)

    t_take = next(t for t, p in phases if p == 'TAKEN_BACK')
    t_his_takeoff = next(t for t, p in phases if p == 'TAKEOFF')
    descents = [(t, phases[i + 1][0]) for i, (t, p) in enumerate(phases) if p == 'DESCEND_TO_ATTACHMENT']
    # tip holds: samples whose value repeats the previous one (stamps still advance)
    held = np.r_[False, np.all(np.diff(tip[:, 1:4], axis=0) == 0, axis=1)]
    holds, i = [], 1
    while i < len(tip):                          # (first index of the held value, index of the first new value)
        if held[i]:
            s0 = i - 1
            while i < len(tip) and held[i]:
                i += 1
            if i < len(tip):
                holds.append((s0, i))
        i += 1
    full = (START_Z - END_Z) / DESCENT_RATE
    out = []
    for k, (a, z) in enumerate(descents):
        # the abort tick publishes its reference just after the phase message: the descent's ticks run to z + 34 ms
        tk = ticks[(ticks >= a) & (ticks <= t_take)]
        e, v, tip_t = errors(tk)
        inside = tk <= z + 0.034
        first_bad = next((i for i in range(len(tk)) if inside[i] and (e[i] > XY_ABORT or v[i] > V_ABORT)), None)
        why = None if first_bad is None else ('position' if e[first_bad] > XY_ABORT else 'speed')
        # did a tip hold (>= 0.15 s, one value with advancing stamps) end within 0.1 s before the abort tick?
        after_hold = None
        if first_bad is not None:
            T = tk[first_bad]
            for hs, he in holds:
                if tip[he, 0] <= T + 1e-6 and T - tip[he, 0] <= 0.1 and tip[he, 0] - tip[hs, 0] >= 0.15:
                    after_hold = dict(hold_s=float(tip[he, 0] - tip[hs, 0]), ended_s_before_abort=float(T - tip[he, 0]),
                                      jump_cm=100 * float(np.linalg.norm(tip[he, 1:3] - tip[hs, 1:3])))
        # runs above 8 cm on the continuous record (the descent and, after the real abort, his re-approach)
        runs, cur = [], None
        for i in range(len(tk)):
            if e[i] > XY_ABORT:
                cur = [i, i] if cur is None else [cur[0], i]
            elif cur is not None:
                runs.append(cur)
                cur = None
        if cur is not None:
            runs.append(cur)
        inside_runs = [r for r in runs if inside[r[0]]]
        last = first_bad if first_bad is not None else int(np.flatnonzero(inside)[-1])
        longest_inside = max([(min(r[1], last) - r[0] + 1) / 30.0 for r in inside_runs if r[0] <= last], default=0.0)
        abort_run = next((r for r in runs if first_bad is not None and r[0] <= first_bad <= r[1]), None)
        abort_run_len = None if abort_run is None else float(tk[abort_run[1]] - tk[abort_run[0]]) + 1 / 30.0
        # 0.30 s debounce on the continuous record, timed on the tip samples (as contact_position_lost)
        deb, since = None, None
        for i in range(len(tk)):
            if e[i] > XY_ABORT:
                since = tip_t[i] if since is None else since
                if tip_t[i] - since >= DEBOUNCE - 1e-9:
                    deb = float(tk[i] - a)
                    break
            else:
                since = None
        out.append(dict(descent=k + 1, start_s_after_his_takeoff=float(a - t_his_takeoff), flown_s=float(z - a),
                        abort_tick_s=None if first_bad is None else float(tk[first_bad] - a), abort_cause=why,
                        err_at_abort_cm=None if first_bad is None else 100 * float(e[first_bad]),
                        speed_at_abort=None if first_bad is None else float(v[first_bad]),
                        peak_err_before_abort_cm=100 * float(e[:first_bad].max()) if first_bad else None,
                        median_err_inside_cm=100 * float(np.median(e[inside])), after_hold=after_hold,
                        longest_run_inside_s=float(longest_inside), abort_run_len_s=abort_run_len,
                        debounced_abort_s=deb, debounced_before_contact=bool(deb is not None and deb < full),
                        recorded_until_s=float(t_take - a)))
    # how often a full descent (3.875 s) fits the recorded swing: from every tick between his first MATCH and
    # TAKEN_BACK - 3.875 s, would the abort (threshold, debounce) stay quiet for 3.875 s?
    t_match = next(t for t, p in phases if p == 'MATCH_VELOCITY')
    tk = ticks[(ticks >= t_match) & (ticks <= t_take)]
    e, v, tip_t = errors(tk)
    sweep = []
    for thr in (0.08, 0.10, 0.12):
        for db in (0.0, 0.3, 0.5, 1.0):
            ok_n, n = 0, 0
            for s0 in range(0, len(tk)):
                if tk[s0] + full > tk[-1]:
                    break
                n += 1
                since, fired = None, False
                for i in range(s0, len(tk)):
                    if tk[i] - tk[s0] > full:
                        break
                    if e[i] > thr:
                        since = tip_t[i] if since is None else since
                        if tip_t[i] - since >= db - 1e-9:
                            fired = True
                            break
                    else:
                        since = None
                ok_n += not fired
            sweep.append(dict(xy=thr, debounce_s=db, starts=n, full_descent_fits=ok_n / max(n, 1)))
    # how much of the error is the plate moving: rms about the window mean, MATCH to TAKEN_BACK
    tgt = hold(bridge, tk)[:, 1:3]
    tipxy = hold(tip, tk)[:, 1:3]
    def rms(x):
        return 100 * float(np.sqrt(np.mean(np.sum((x - x.mean(axis=0)) ** 2, axis=1))))
    split = dict(plate_rms_cm=rms(tgt), tip_rms_cm=rms(tipxy), err_rms_cm=100 * float(np.sqrt(np.mean(e ** 2))),
                 err_median_cm=100 * float(np.median(e)), err_over_8cm_frac=float(np.mean(e > 0.08)),
                 tip_minus_plate_mean_cm=[100 * float(v) for v in (tipxy - tgt).mean(axis=0)])
    print('split', split)
    res = dict(check=check, descents=out, split=split, descent_rate=DESCENT_RATE, start_height=START_Z, end_height=END_Z,
               full_descent_s=full, sweep=sweep,
               tip_held_frac=float(held.mean()))
    with open(os.path.join(OUT, 'w2a_descent_abort.json'), 'w') as f:
        json.dump(res, f, indent=1)
    print(f'full descent {START_Z:.2f} -> {END_Z:.2f} m at {DESCENT_RATE} m/s: {full:.2f} s; tip held {100 * held.mean():.1f} %')
    for r in out:
        print({k: (round(v, 3) if isinstance(v, float) else v) for k, v in r.items()})
    for r in sweep:
        print(r)


if __name__ == '__main__':
    main()

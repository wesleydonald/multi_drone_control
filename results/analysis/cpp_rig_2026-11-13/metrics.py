#!/usr/bin/env python3
"""One metrics row per 13 Nov flight that ran his C++ planner -> metrics.csv, plus per-flight clearance series
(series/<flight>.npz) for the figures.

Times are seconds after his TAKEOFF (his phase WAIT_FOR_TAKEOFF -> TAKEOFF, ROS wall stamp from the .log; the bag's
receive stamps are on the same clock). Windows:
  C++ transit   = his phase TRANSIT_TO_DROP_POINT entry (the authority switch to C++) -> the next phase
                  (SETTLE_ABOVE_DROP_POINT = the moving rendezvous captured; TAKEN_BACK = our LAND took QUAD4 back)
  settle+descent = SETTLE_ABOVE_DROP_POINT entry -> DROP_OBJECT entry (his Python, drop-point relative)
Clearances are centre-line distances (no radii) between segments: QUAD4's cable = QUAD4 body - 0.04 m -> the magnet
tip marker (/magnet_tip_pose); each carrier's cable = carrier body - 0.04 m -> its plate (radius 0.225, +0.018 m in
the ring frame, the plate found from the data, see 'plates'); the object = its marker as a point, minus its 0.0525 m
half-size, only while it is within 0.15 m of the tip (on the magnet)."""
import csv
import json
import os

import numpy as np
import pandas as pd

from common import (FLIGHTS, FROZEN, HERE, OBJ_HALF, OBJ_ON_TIP, RIG, cable_top, grid, load, next_phase, parse_log,
                    phase_t, plate_pos, rot_inv, seg_seg, stamp_local)


def launch_args():
    out = {}
    for r in csv.DictReader(open(os.path.join(RIG, 'launches.csv'))):
        kv = {}
        for tok in r['launch'].split():
            if ':=' in tok:
                k, v = tok.split(':=', 1)
                kv.setdefault(k, v)
        out[r['name']] = kv
    return out


def find_plates(g, mask):
    """For each carrier, the plate whose distance to its cable top stays most constant (std), with the mean."""
    res = []
    for i in range(3):
        top = cable_top(g, i)
        best = []
        for k in range(12):
            d = np.linalg.norm(top - plate_pos(g['ring'], k), axis=1)[mask]
            best.append((np.nanstd(d), np.nanmean(d), k))
        best.sort()
        res.append((best[0][2], best[0][1], best[0][0], best[1][0]))
    return res


def carrier_azimuth(g, mask):
    """Each carrier body's median azimuth about the ring centre in the ring frame, clockwise from +x (deg); its taut
    cable runs out radially from its plate, so azimuth / 30 = the plate number."""
    out = []
    for i in range(3):
        rel = rot_inv(g['ring'][mask, 3:7], g[f'd{i}'][mask, :3] - g['ring'][mask, :3])
        out.append(float(np.nanmedian((-np.degrees(np.arctan2(rel[:, 1], rel[:, 0]))) % 360.0)))
    return out


def ring_speed(g):
    p = g['ring'][:, :2]
    n = 10  # +-0.1 s central difference
    v = np.full(len(p), np.nan)
    v[n:-n] = np.linalg.norm(p[2 * n:] - p[:-2 * n], axis=1) / (2 * n * (g['t'][1] - g['t'][0]))
    return v


def clearances(g, plates):
    """Per carrier: QUAD4 cable - carrier cable, object - carrier cable (object only while on the magnet)."""
    body = g['d3'][:, :3]
    top4 = body - np.array([0.0, 0.0, 0.04])
    tip = g['tip'][:, :3]
    obj = g['obj'][:, :3]
    on = np.linalg.norm(obj - tip, axis=1) < OBJ_ON_TIP
    cab, ob = [], []
    for i in range(3):
        top = cable_top(g, i)
        pl = plate_pos(g['ring'], plates[i])
        cab.append(seg_seg(top4, tip, top, pl))
        o = seg_seg(obj, obj, top, pl) - OBJ_HALF
        o[~on] = np.nan
        ob.append(o)
    return np.stack(cab, 1), np.stack(ob, 1)


def wmin(t, D, a, b, tk):
    """Min over the window [a, b] of a (N, 3) distance array: value, carrier index, time after TAKEOFF."""
    if a is None or b is None:
        return np.nan, '', np.nan
    m = (t >= a) & (t <= b)
    if not m.any() or np.all(np.isnan(D[m])):
        return np.nan, '', np.nan
    sub = np.where(np.isnan(D[m]), np.inf, D[m])
    j, i = np.unravel_index(np.argmin(sub), sub.shape)
    return float(sub[j, i]), f'd{i}', float(t[m][j] - tk)


def backend(cpp_dir, t_on, t_off):
    r = pd.read_csv(os.path.join(cpp_dir, 'replans.csv'))
    b = pd.read_csv(os.path.join(cpp_dir, 'backend.csv'))
    rt = r.replan_runtime_ms
    term = r.octopus_termination.fillna('').astype(str).value_counts().to_dict()
    be = b[b.enabled == 1]
    fail = r[r.accepted == 0]
    fterm = fail.octopus_termination.fillna('').astype(str).value_counts().to_dict()
    return {'replans': len(r), 'installed': int(r.authority_installed.sum()), 'late': int(r.candidate_late.sum()),
            'replan_ms_median': float(rt.median()) if len(r) else np.nan,
            'replan_ms_p95': float(rt.quantile(0.95)) if len(r) else np.nan,
            'replan_ms_max': float(rt.max()) if len(r) else np.nan,
            'runtime_reached': int(term.get('RUNTIME_REACHED', 0)), 'empty_open_list': int(term.get('EMPTY_OPEN_LIST', 0)),
            'goal_reached': int(term.get('GOAL_REACHED', 0)), 'accepted': int(r.accepted.sum()),
            'failed': len(fail), 'failed_runtime_reached': int(fterm.get('RUNTIME_REACHED', 0)),
            'failed_empty_open_list': int(fterm.get('EMPTY_OPEN_LIST', 0)),
            'max_state_age_ms_1hz': float(be.cooperative_max_state_age_ms.max()) if len(be) else np.nan,
            'fallback_count_backend': int(b.execution_fallback_count.max()) if len(b) else 0}


def object_after_drop(g, td):
    """The object in the ring frame 2.5-3.5 s after DROP_OBJECT, and whether it stays inside the ring (DROP+1.5..+6 s)."""
    t = g['t']
    rel = rot_inv(g['ring'][:, 3:7], g['obj'][:, :3] - g['ring'][:, :3])
    h, z = np.hypot(rel[:, 0], rel[:, 1]), rel[:, 2]
    m3 = (t >= td + 2.5) & (t <= td + 3.5)
    ms = (t >= td + 1.5) & (t <= td + 6.0)
    fin_h, fin_z = float(np.nanmean(h[m3])), float(np.nanmean(z[m3]))
    stay = ms & np.isfinite(h)
    inside = (h[stay] < 0.225) & (z[stay] > -0.30) & (z[stay] < 0.10)
    return fin_h, fin_z, float(inside.mean()) if stay.any() else np.nan, float(np.nanmin(g['obj'][ms, 2]))


def frozen(a, t0, t1):
    """Longest run (s) of bit-identical consecutive positions in [t0, t1] (a lost body keeps its last pose)."""
    a = a[(a[:, 0] >= t0) & (a[:, 0] <= t1)]
    if len(a) < 2:
        return 0.0
    same = np.all(np.diff(a[:, 1:4], axis=0) == 0.0, axis=1)
    best, start = 0.0, None
    for i, s in enumerate(same):
        if s and start is None:
            start = a[i, 0]
        elif not s and start is not None:
            best, start = max(best, a[i, 0] - start), None
    if start is not None:
        best = max(best, a[-1, 0] - start)
    return best


def outcome(row, L, g, tk, t_tr, t_end_tr):
    """PASS = dropped into the net and QUAD4 at its start; PARTIAL = dropped but not into the net; FAIL = the C++
    transit never captured; ABORT = no C++ hand-over."""
    if t_tr is None:
        if L['pos_limit']:
            tp = L['pos_limit'][0][0]
            j = np.argmin(np.abs(g['t'] - tp))
            far = max(range(4), key=lambda i: np.nanmax(np.abs(g[f'd{i}'][j, :2])))
            x, y = g[f'd{far}'][j, :2]
            return 'ABORT', (f'position limit {tp - tk:.1f} s after his TAKEOFF (d{far} at x {x:+.2f}, y {y:+.2f}), '
                             'before the C++ hand-over; every drone landed')
        return 'ABORT', 'no C++ hand-over'
    if not row['captured']:
        bits = []
        dis = [(tb, why) for tb, why in L['backend_disabled'] if t_tr <= tb < t_end_tr - 0.2]
        if dis:
            bits.append(f'C++ backend disabled {dis[0][0] - t_tr:.1f} s after the grant ({dis[0][1]}), never re-granted')
        fb = []
        if row['fb_smooth_brake']:
            fb.append(f"SMOOTH_BRAKE x{row['fb_smooth_brake']}")
        if row['fb_hard_hover']:
            fb.append(f"UNCERTIFIED_HARD_HOVER x{row['fb_hard_hover']}")
        if fb:
            bits.append(', '.join(fb))
        if row.get('failed'):
            bits.append(f"{row['failed']} of {row['replans']} replans failed (search RUNTIME_REACHED "
                        f"x{row['failed_runtime_reached']}, EMPTY_OPEN_LIST x{row['failed_empty_open_list']})")
        land = [x for x in L['land_cmd'] if x >= t_tr]
        tail = f'our LAND {land[0] - tk:.1f} s after his TAKEOFF' if land else 'ended without our LAND'
        return 'FAIL', f'no capture in {row["cpp_transit_s"]} s: ' + '; '.join(bits + [tail])
    if row.get('drop_t', '') == '':
        return 'FAIL', 'captured, no drop'
    if not row.get('in_net', False):
        where = 'on the floor' if row['obj_min_world_z_after_m'] < 0.3 else 'outside the ring'
        return 'PARTIAL', (f"captured after {row['cpp_transit_s']} s; dropped, missed the net: object {where}, "
                           f"{row['obj_final_h_m']:.2f} m from the ring centre 3 s later")
    if row.get('at_start_t', '') == '':
        return 'PARTIAL', 'dropped into the net, no AT_START'
    return 'PASS', 'dropped into the net, Python return to AT_START'


def main(flights):
    la = launch_args()
    rows = []
    os.makedirs(os.path.join(HERE, 'series'), exist_ok=True)
    for name in flights:
        L = parse_log(name)
        d = load(name)
        a = la.get(name, {})
        tk = phase_t(L, 'TAKEOFF')
        t_tr = phase_t(L, 'TRANSIT_TO_DROP_POINT')
        t_settle = phase_t(L, 'SETTLE_ABOVE_DROP_POINT')
        t_drop = phase_t(L, 'DROP_OBJECT')
        t_ret = phase_t(L, 'RETURN_TO_START')
        t_at = phase_t(L, 'AT_START')
        t_back = phase_t(L, 'TAKEN_BACK')
        t_end_tr, ph_after, why_after = next_phase(L, t_tr) if t_tr else (None, None, None)
        captured = ph_after == 'SETTLE_ABOVE_DROP_POINT'
        t_stop = next((x for x in L['disarm'] if x > tk), d['d3'][-1, 0])
        g = grid(d, tk - 2.0, t_stop)
        t = g['t']
        air = (g['ring'][:, 2] > 0.6) & (t >= tk) & (t <= (t_back or t_stop))
        plates = find_plates(g, air)
        P = [p[0] for p in plates]
        cab, ob = clearances(g, P)
        vr = ring_speed(g)
        # bag start (first message of any extracted topic)
        t_bag0 = min(d[k][0, 0] for k in ('d0', 'd1', 'd2', 'd3', 'ring') if len(d[k]))
        row = {'flight': name, 'frozen_line': name in FROZEN, 'traj': a.get('load_traj', ''),
               'swing_deg': round(np.degrees(float(a['max_swing_angle_rad'])), 1) if 'max_swing_angle_rad' in a else '',
               'spline_factor': a.get('spline_time_factor', ''), 'cpp_splice_s': a.get('cpp_splice', ''),
               'cpp_max_z': a.get('cpp_max_z', ''), 'drop_height': a.get('drop_height', ''),
               'max_body_z': a.get('max_body_z', ''),
               'args_log_match': all(L['args'].get(k) == a.get(k) for k in L['args']),
               'takeoff_local': stamp_local(tk), 'takeoff_s_after_bag_start': round(tk - t_bag0, 1),
               'plates_d0_d1_d2': '/'.join(str(p[0]) for p in plates),
               'plate_len_mean_m': '/'.join(f'{p[1]:.3f}' for p in plates),
               'plate_len_std_m': '/'.join(f'{p[2]:.4f}' for p in plates),
               'plate_runnerup_std_m': '/'.join(f'{p[3]:.4f}' for p in plates),
               'carrier_az_cw_deg': '/'.join(f'{a:.1f}' for a in carrier_azimuth(g, air))}
        rel = lambda x: round(x - tk, 1) if x is not None else ''
        row['cpp_start_s'] = rel(t_tr)
        row['cpp_end_phase'] = ph_after or ''
        row['captured'] = captured
        row['cpp_transit_s'] = round(t_end_tr - t_tr, 1) if t_tr else ''
        if t_tr:
            m = (t >= t_tr) & (t <= t_end_tr)
            row['ring_speed_mean_mps'] = round(float(np.nanmean(vr[m])), 3)
            row['ring_speed_at_end_mps'] = round(float(np.nanmean(vr[(t >= t_end_tr - 0.25) & (t <= t_end_tr)])), 3)
            row['q4_max_z_cpp_m'] = round(float(np.nanmax(g['d3'][m, 2])), 2)
        mf = (t >= tk) & (t <= (t_back or t_stop))
        row['q4_max_z_flight_m'] = round(float(np.nanmax(g['d3'][mf, 2])), 2)
        rf = d['ref'][(d['ref'][:, 0] >= tk) & (d['ref'][:, 0] <= (t_back or t_stop))]
        row['ref_max_z_flight_m'] = round(float(rf[:, 3].max()), 2) if len(rf) else ''
        for tag, (wa, wb) in (('cpp', (t_tr, t_end_tr)), ('desc', (t_settle, t_drop))):
            for kind, D in (('q4cable', cab), ('obj', ob)):
                v, c, tt = wmin(t, D, wa, wb, tk)
                row[f'min_{kind}_{tag}_m'] = round(v, 3) if np.isfinite(v) else ''
                row[f'min_{kind}_{tag}_carrier'] = c
                row[f'min_{kind}_{tag}_t'] = round(tt, 1) if np.isfinite(tt) else ''
                for i in range(3):
                    vi, _, _ = wmin(t, D[:, i:i + 1], wa, wb, tk)
                    row[f'min_{kind}_{tag}_d{i}_m'] = round(vi, 3) if np.isfinite(vi) else ''
        row['settle_s'] = round(phase_t(L, 'DESCEND_TO_DROP_HEIGHT') - t_settle, 1) if t_settle and t_drop else ''
        row['descent_s'] = round(t_drop - phase_t(L, 'DESCEND_TO_DROP_HEIGHT'), 1) if t_drop else ''
        row['drop_t'] = rel(t_drop)
        row['drop_reason'] = L['drop_reason'] or ''
        if t_drop:
            fh, fz, fin, zmin = object_after_drop(g, t_drop)
            row['obj_final_h_m'] = round(fh, 3)
            row['obj_final_dz_m'] = round(fz, 3)
            row['obj_inside_frac'] = round(fin, 2)
            row['obj_min_world_z_after_m'] = round(zmin, 2)
            row['in_net'] = bool(fin > 0.9)
            j = np.argmin(np.abs(t - t_drop))
            rel0 = rot_inv(g['ring'][j:j + 1, 3:7], g['obj'][j:j + 1, :3] - g['ring'][j:j + 1, :3])[0]
            row['obj_h_at_drop_m'] = round(float(np.hypot(rel0[0], rel0[1])), 3)
            row['obj_dz_at_drop_m'] = round(float(rel0[2]), 3)
        row['return_s'] = round(t_at - t_ret, 1) if t_at and t_ret else ''
        row['at_start_t'] = rel(t_at)
        if t_at:
            j0, j1 = np.argmin(np.abs(t - tk)), np.argmin(np.abs(t - t_at))
            row['q4_at_start_xy_offset_m'] = round(float(np.hypot(*(g['d3'][j1, :2] - g['d3'][j0, :2]))), 3)
            row['q4_at_start_z_m'] = round(float(g['d3'][j1, 2]), 2)
        row['our_land_t'] = rel(L['land_cmd'][0]) if L['land_cmd'] else ''
        row['pos_limit'] = L['pos_limit'][0][1] if L['pos_limit'] else ''
        row['pos_limit_t'] = rel(L['pos_limit'][0][0]) if L['pos_limit'] else ''
        if L['cpp_dir']:
            row.update(backend(L['cpp_dir'], t_tr, t_end_tr))
        fb = {}
        for tf, mode, why in L['fallbacks']:
            fb[mode] = fb.get(mode, 0) + 1
        row['fb_smooth_brake'] = fb.get('SMOOTH_BRAKE', 0)
        row['fb_hard_hover'] = fb.get('UNCERTIFIED_HARD_HOVER', 0)
        row['fb_other'] = sum(v for k, v in fb.items() if k not in ('SMOOTH_BRAKE', 'UNCERTIFIED_HARD_HOVER'))
        row['fb_outside_cpp'] = sum(1 for tf, *_ in L['fallbacks'] if not (t_tr and t_tr <= tf <= t_end_tr))
        row['stale_ref_lines'] = len(L['stale_ref'])
        # mocap dropouts on the grid over the C++ transit + descent (QUAD4 body, tip, object, ring, carriers)
        if t_tr:
            m = (t >= t_tr) & (t <= (t_drop or t_end_tr))
            row['nan_frac_q4_tip_ring'] = round(float(np.mean(np.isnan(g['d3'][m, 0]) | np.isnan(g['tip'][m, 0])
                                                              | np.isnan(g['ring'][m, 0]))), 3)
            row['max_frozen_pose_s'] = round(max(frozen(d[k], t_tr, t_drop or t_end_tr) for k in
                                                 ('d0', 'd1', 'd2', 'd3', 'ring', 'tip')), 2)
        row['outcome'], row['reason'] = outcome(row, L, g, tk, t_tr, t_end_tr)
        rows.append(row)
        np.savez_compressed(os.path.join(HERE, 'series', name + '.npz'), t=t - tk, cab=cab, ob=ob, vr=vr,
                            d0=g['d0'][:, :3], d1=g['d1'][:, :3], d2=g['d2'][:, :3], d3=g['d3'][:, :3],
                            ring=g['ring'], tip=g['tip'][:, :3], obj=g['obj'][:, :3], plates=np.array(P),
                            ev=json.dumps({k: (v - tk if v else None) for k, v in
                                           (('transit', t_tr), ('cpp_end', t_end_tr), ('settle', t_settle),
                                            ('drop', t_drop), ('ret', t_ret), ('at_start', t_at),
                                            ('back', t_back))}),
                            cpp_end_phase=ph_after or '')
        print(name, 'done')
    keys = []
    for r in rows:
        keys += [k for k in r if k not in keys]
    with open(os.path.join(HERE, 'metrics.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, keys, lineterminator='\n')
        w.writeheader()
        w.writerows(rows)


if __name__ == '__main__':
    import sys
    main(sys.argv[1:] or FLIGHTS)

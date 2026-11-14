#!/usr/bin/env python3
"""Q013 item 3: the drop into the net as a disturbance of the ring, one row per drop on Setup 3 (the M2A ring, 1 m
cables, 4 Nov on). Writes drop_disturbance.csv beside this file.

    source ~/ros2_humble/install/setup.bash && source ~/multi_drone_control/install/setup.bash
    cd ~/multi_drone_control && nice -n 10 python3 results/analysis/rig_results_2026-11-13/drops.py

Flights: every flight from 4 Nov on whose launch log has his "Planner phase: ... -> DROP_OBJECT" line and that our
planner flew (a geometry line); a flight whose object never left the magnet is listed in the output with no metrics.

Definitions (paper_rig's where it has one; judged after the flights, no bars):
- release t_r: his DROP_OBJECT phase line (his OFF reaches the coil at once since 11 Nov 09:41).
- window: t_r to the first of t_r + 10 s, the operator's LAND, a loss (drop, fault stop, position limit, the ring
  under 0.3 m), the next fleet resize, QUAD4's next descent onto the ring (DESCEND_TO_ATTACHMENT / LIFT_PROOF:
  paper_rig's detach rule) and the end of the record. The *_to10 columns repeat the peaks to t_r + 10 s without the
  descent cut.
- before: paper_rig's 5 s before t_r, cut at the set-point's last change (release_bands' ref_still).
- tilt (deg): the ring normal from vertical; tilt peak = the largest in the window; swing = the tilt vector's largest
  departure from its mean before.
- dip (mm): the mean height before minus the lowest height in the window (+ = down); dip_pre_mm: the same for the
  lowest height before (the baseline), tilt_pre_max_deg the largest tilt before.
- horizontal error (cm): the ring's horizontal distance to the path its reference traces within +-5 s (clock-free; in
  a hover this is the distance to the set-point); peak in the window, mean and max before.
- ring speed at the release: the ring's horizontal chord over the 1 s before t_r (paper_rig).
- recovery: from the object's first impact in the net (else the object leaving the magnet, else t_r), the first time
  from which, held 1 s, the tilt vector is within max(2 sigma, 1 deg) of its mean before, the height within
  max(2 sd, 1 cm) of its mean before and the horizontal error under its mean before + max(2 sd, 1 cm) (sigma, sd: the
  before window). Searched to t_r + 15 s (cut as the window). settle_tilt_s: paper_rig's tilt-only t_level (from
  the swing's peak, held 1 s), the detach table's rule.
- object (ring frame, mocap): obj_dz_release = its height over the ring's plane at t_r; obj_left_s = its marker 2 cm
  further from the magnet marker than in the 1 s before t_r; impact (joins.drop_metrics, F052: the fall relative to
  the ring stops); settled distance and depth = the mean over t_r + 2.5..3.5 s; in the net = inside (horizontal
  < 0.225 m, -0.30 < depth < 0.10 m) for over 90 % of t_r + 1.5..6 s (results/analysis/cpp_rig_2026-11-13 rules).
"""
import glob
import math
import os
import re
import sys
import warnings

import numpy as np
from scipy.spatial.transform import Rotation as Rot

import common as C

PR = C.PR
joins = C.joins
AFTER = 10.0
HORIZON = 15.0
HOLD = 1.0
HALF_PATH = 5.0
FIRST_DAY = '2026-11-04'
OBJECT_BY_DAY = {'2026-11-05': 'small, bare (60 g)', '2026-11-11': 'small, bare (60 g)',
                 '2026-11-12': 'small + funnel', '2026-11-13': 'small + funnel'}
OBJECT_FLIGHT = {'2026-11-05/k1ob_2': 'big (0.15 kg)'}
NOTES = {
    '2026-11-05/k1d': 'Motive recording on (every pose ~2x late, 5 Nov before ~09:00)',
    '2026-11-05/k1ob_2': 'big object fell 0.53 m; the impact pulled off plate 7, 3 then 11 magnets (RIG-1105-k1ob_2c)',
    '2026-11-11/k1zo': 'release 0.14 m over the drop point; the impact pulled a carrier magnet off (RIG-1111-k1zo)',
    '2026-11-11/fm9hc_8': 'the object rode the magnet 0.8-0.9 s after the OFF (RIG-1111-fm9hc_8)',
    '2026-11-13/fm9hc': 'released 0.16 m off centre while drifting; hit the ring edge, fell to the floor',
    '2026-11-11/k1zo_3': 'no release: our magnet_gate held his OFF and the coil stayed ON (fixed 09:41, RIG-1111-k1zo_3)',
}


def drop_flights():
    out = []
    for day in sorted(d for d in os.listdir(C.RIG) if re.fullmatch(r'\d{4}-\d\d-\d\d', d) and d >= FIRST_DAY):
        for fn in sorted(glob.glob(os.path.join(C.RIG, day, '*.log'))):
            txt = open(fn, errors='replace').read()
            if re.search(r'Planner phase: \w+ -> DROP_OBJECT\.', txt) and 'geometry: n=' in txt:
                out.append(f'{day}/{os.path.basename(fn)[:-4]}')
    return out


def hold_series(arr, ts, max_age=0.1):
    """arr (t, ...) sampled and held at ts; rows older than max_age are NaN."""
    k = np.searchsorted(arr[:, 0], ts, side='right') - 1
    ok = (k >= 0) & (ts - arr[np.clip(k, 0, None), 0] <= max_age)
    out = arr[np.clip(k, 0, None)].copy()
    out[~ok, 1:] = np.nan
    return out


def frozen_mask(arr, min_run=0.2):
    """Rows of a pose array whose position repeats bit for bit for min_run s or more (a lost body keeps arriving with
    its last pose, CLAUDE.md 11 Nov)."""
    out = np.zeros(len(arr), bool)
    if len(arr) < 2:
        return out
    same = np.r_[False, np.all(np.diff(arr[:, 1:4], axis=0) == 0.0, axis=1)]
    i = 0
    while i < len(arr):
        if same[i]:
            j = i
            while j < len(arr) and same[j]:
                j += 1
            if arr[j - 1, 0] - arr[i - 1, 0] >= min_run:
                out[i - 1:j] = True
            i = j
        else:
            i += 1
    return out


def ring_frozen_s(ring, t0, t1):
    """The longest frozen run of the ring's pose in [t0, t1] (s)."""
    w = (ring[:, 0] >= t0) & (ring[:, 0] <= t1)
    a = ring[w]
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
    return float(best)


def path_err(t, p, r, idx, half=HALF_PATH):
    """Horizontal distance of p[i] to the polyline r traces over t[i] +- half (clock-free), for i in idx."""
    okr = np.all(np.isfinite(r), 1)
    out = np.full(len(idx), np.nan)
    for j, i in enumerate(idx):
        if not np.all(np.isfinite(p[i])):
            continue
        R = r[okr & (t >= t[i] - half) & (t <= t[i] + half)]
        if len(R) == 0:
            continue
        if len(R) == 1:
            out[j] = float(np.linalg.norm(p[i] - R[0]))
            continue
        A, AB = R[:-1], R[1:] - R[:-1]
        L2 = np.sum(AB ** 2, 1)
        u = np.clip(np.sum((p[i] - A) * AB, 1) / np.maximum(L2, 1e-12), 0.0, 1.0)
        out[j] = float(np.min(np.linalg.norm(A + u[:, None] * AB - p[i], axis=1)))
    return out


def chord_speed(t, xy, te, span=1.0):
    w = (t >= te - span) & (t < te) & np.all(np.isfinite(xy), 1)
    if w.sum() < 10:
        return None
    a, b = np.nonzero(w)[0][[0, -1]]
    return float(np.linalg.norm(xy[b] - xy[a]) / max(t[b] - t[a], 1e-3))


def window_label(F, te, tend, desc_t):
    if F.t_land and abs(tend - F.t_land) < 1e-6:
        return 'LAND'
    lo = F.loss_between(te, math.inf)
    if lo and abs(tend - lo[0]) < 1e-6:
        return 'loss: ' + lo[1]
    if desc_t is not None and abs(tend - desc_t) < 1e-6:
        return 'QUAD4 descent'
    if abs(tend - F.t_end) < 1e-6:
        return 'end of record'
    if abs(tend - te - AFTER) < 1e-6:
        return '10 s'
    return 'next resize'


def object_metrics(B, te, t_loss):
    out = {}
    obj, ring, tip = B['obj'], B['ring'], B['tip']
    if len(obj) < 20 or len(ring) < 20:
        out['obj_note'] = 'no object or ring pose'
        return out
    R = hold_series(ring, obj[:, 0])
    ok = np.all(np.isfinite(R[:, 1:8]), 1)
    rel = np.full((len(obj), 3), np.nan)
    rel[ok] = Rot.from_quat(R[ok, 4:8]).inv().apply(obj[ok, 1:4] - R[ok, 1:4])
    fz = frozen_mask(obj)
    rel[fz] = np.nan
    h, dz, t = np.hypot(rel[:, 0], rel[:, 1]), rel[:, 2], obj[:, 0]
    fr = np.nonzero(fz & (t > te) & (t <= te + 6.0))[0]
    if fr.size:
        out['obj_frozen_from_s'] = round(float(t[fr[0]] - te), 2)
    k0 = np.nonzero((t <= te) & np.isfinite(dz))[0]
    if k0.size:
        out['obj_dz_release_cm'] = 100.0 * float(dz[k0[-1]])
        out['obj_h_release_cm'] = 100.0 * float(h[k0[-1]])
    if len(tip) > 20:
        T = hold_series(tip, t)
        d = np.linalg.norm(obj[:, 1:4] - T[:, 1:4], axis=1)
        w0 = (t >= te - 1.0) & (t <= te) & np.isfinite(d)
        if w0.sum() > 5:
            d0 = float(np.median(d[w0]))
            far = np.nonzero((t >= te - 0.3) & (t <= te + 5.0) & (np.abs(d - d0) > 0.02))[0]
            out['obj_left_s'] = float(t[far[0]] - te) if far.size else None
    dm = joins.drop_metrics(B, te)
    out['impact_s'] = dm.get('impact_after_s')
    out['impact_xy_cm'] = dm.get('impact_xy_cm')
    out['impact_depth_cm'] = -dm['impact_height_cm'] if dm.get('impact_height_cm') is not None else None
    stop = t_loss if t_loss is not None else math.inf
    m3 = (t >= te + 2.5) & (t <= min(te + 3.5, stop)) & np.isfinite(h)
    if m3.sum() >= 5:
        out['obj_settled_cm'] = 100.0 * float(np.mean(h[m3]))
        out['obj_settled_depth_cm'] = -100.0 * float(np.mean(dz[m3]))
    ms = (t >= te + 1.5) & (t <= min(te + 6.0, stop)) & np.isfinite(h)
    n_all = int(((t >= te + 1.5) & (t <= min(te + 6.0, stop))).sum())
    if n_all and ms.sum() < 0.5 * n_all:
        out['in_net'] = 'unknown (object pose lost)'
        out['obj_inside_frac'] = float(((h[ms] < 0.225) & (dz[ms] > -0.30) & (dz[ms] < 0.10)).mean()) if ms.any() else None
    elif ms.sum() >= 10:
        inside = (h[ms] < 0.225) & (dz[ms] > -0.30) & (dz[ms] < 0.10)
        out['obj_inside_frac'] = float(inside.mean())
        out['in_net'] = 'yes' if inside.mean() > 0.9 else 'no'
    if t_loss is not None and t_loss < te + 6.0:
        out['obj_note'] = f'the ring was lost at +{t_loss - te:.2f} s: settled / in-net read to then'
    return out


def one(fid):
    F = C.flight(fid)
    row = {'flight': F.name, 'day': F.day, 'registry_id': F.rid}
    drops = [p for p in F.J['phases'] if p[2] == 'DROP_OBJECT']
    te, reason = drops[0][0], drops[0][3]
    traj = F.args.get('load_traj') or (F.geo or {}).get('load_traj')
    row['motion'] = C.MOTION.get(traj, traj)
    row['object'] = OBJECT_FLIGHT.get(fid, OBJECT_BY_DAY.get(F.day, '?'))
    log = open(os.path.join(C.RIG, F.day, F.name + '.log'), errors='replace').read()
    row['transfer'] = 'C++' if 'C++ transfer prepared' in log else 'Python'
    row['release_rule'] = ('drop on arrival' if reason.startswith('drop on arrival') else 'settle drop')
    row['drop_reason'] = reason
    row['t_release_s'] = F.rel(te)
    row['carriers'] = F.n_at(te)
    nxt = next((p for p in F.J['phases'] if p[0] > te and p[1] == 'DROP_OBJECT'), None)
    row['after_drop_phase'] = nxt[2] if nxt else ''
    pause = C.log_lines(F.day, F.name, r'attach pause: the orbit slows to rest')
    pz = [t for t, _ in pause if t > te - 1.0]
    row['orbit_pause_s'] = round(pz[0] - te, 2) if pz else None
    desc = next((p[0] for p in F.J['phases'] if p[0] > te and p[2] in PR.DESCENTS), None)
    row['q4_next_descent_s'] = round(desc - te, 2) if desc else None
    tend15 = F.window_end(te, 'detach')
    tend10 = min(te + AFTER, tend15)
    full15 = F.window_end(te, 'drop')
    full10 = min(te + AFTER, full15)
    row['window_s'] = round(tend10 - te, 2)
    row['window_end'] = window_label(F, te, tend10, desc)
    lo = F.loss_between(te - 0.05, te + HORIZON)
    row['loss'] = f'{lo[1]} at +{lo[0] - te:.2f} s' if lo else ''
    row['note'] = NOTES.get(fid, '')
    if F.B is None or F.G is None:
        row['note'] += '; no bag'
        return row
    row.update(object_metrics(F.B, te, lo[0] if lo else None))
    if row.get('obj_left_s') is None and row.get('impact_s') is None:
        row['released'] = 'no'
        row['in_net'] = 'no (still on the magnet)'
        return row
    row['released'] = 'yes'
    moving = row['motion'] != 'hover'
    G = F.G
    t = G.t.to_numpy(float)
    th = G.attrs['th']
    okt = np.all(np.isfinite(th), 1)
    mag = np.linalg.norm(th, axis=1)
    z = G.payload_z.to_numpy(float)
    xy = G[['payload_x', 'payload_y']].to_numpy(float)
    ref = G[['payload_ref_x', 'payload_ref_y']].to_numpy(float)
    row['ring_speed_mps'] = chord_speed(t, xy, te)
    row['ref_speed_mps'] = chord_speed(t, ref, te)
    row['ring_frozen_max_s'] = ring_frozen_s(F.B['ring'], te - PR.PRE_S, tend15)
    # paper_rig's "before": the 5 s before, cut at the set-point's last change
    lo_pre = max(te - PR.PRE_S, PR.ref_still_from(G, te))
    pre = (t >= lo_pre) & (t < te) & okt
    row['pre_s'] = round(te - lo_pre, 2)
    win = np.nonzero((t >= te) & (t < tend10) & okt)[0]
    if pre.sum() < 1.0 * PR.HZ or win.size < 5:
        row['resp_note'] = f'before {pre.sum() / PR.HZ:.1f} s, after {win.size / PR.HZ:.1f} s of ring pose'
        return row
    mean_th = th[pre].mean(0)
    sig = float(np.sqrt(np.mean(np.sum((th[pre] - mean_th) ** 2, 1))))
    dev_th = np.linalg.norm(th - mean_th, axis=1)
    row['tilt_pre_deg'] = float(mag[pre].mean())
    row['tilt_pre_max_deg'] = float(np.nanmax(mag[pre]))
    row['sigma_pre_deg'] = sig
    row['tilt_peak_deg'] = float(np.nanmax(mag[win]))
    k = win[int(np.argmax(dev_th[win]))]
    row['swing_deg'] = float(dev_th[k])
    row['t_swing_peak_s'] = float(t[k] - te)
    aft = (t >= te) & (t <= min(te + AFTER, tend10))
    row['ring_z_pre_m'] = float(np.nanmean(z[pre]))
    row['z_sd_pre_mm'] = 1000.0 * float(np.nanstd(z[pre]))
    row['dip_mm'] = 1000.0 * float(np.nanmean(z[pre]) - np.nanmin(z[aft]))
    row['dip_pre_mm'] = 1000.0 * float(np.nanmean(z[pre]) - np.nanmin(z[pre]))
    kz = np.nonzero(aft)[0][int(np.nanargmin(z[aft]))]
    row['t_dip_s'] = float(t[kz] - te)
    if not moving:
        row['xy_exc_from_pre_cm'] = 100.0 * float(np.nanmax(np.linalg.norm(xy[aft] - np.nanmean(xy[pre], 0), axis=1)))
    # paper_rig's own numbers on the same window (a check) and its tilt-only settle time
    R = PR.response(G, te, tend10, moving)
    R15 = PR.response(G, te, tend15, moving)
    row['settle_tilt_s'] = R15.get('settle_s')
    for a, b in (('tilt_peak_deg', 'tilt_peak_deg'), ('swing_deg', 'swing_deg'), ('dip_mm', 'dip_mm')):
        if R.get(b) is not None and abs(R[b] - row[a]) > 1e-6:
            row['check'] = row.get('check', '') + f'{a} {row[a]:.3f} vs paper_rig {R[b]:.3f}; '
    ipre = np.nonzero(pre)[0]
    win15 = np.nonzero((t >= te) & (t < tend15))[0]
    e_pre = path_err(t, xy, ref, ipre)
    e = np.full(len(t), np.nan)
    e[ipre] = e_pre
    e[win15] = path_err(t, xy, ref, win15)
    row['xy_err_pre_mean_cm'] = 100.0 * float(np.nanmean(e_pre))
    row['xy_err_pre_max_cm'] = 100.0 * float(np.nanmax(e_pre))
    row['xy_err_peak_cm'] = 100.0 * float(np.nanmax(e[win]))
    row['xy_err_rise_cm'] = row['xy_err_peak_cm'] - row['xy_err_pre_mean_cm']
    row['t_xy_peak_s'] = float(t[win[int(np.nanargmax(e[win]))]] - te)
    # the same peaks to t_r + 10 s without the descent cut
    w10 = np.nonzero((t >= te) & (t < full10))[0]
    if full10 > tend10 + 0.05:
        e10 = path_err(t, xy, ref, w10)
        row['tilt_peak_to10_deg'] = float(np.nanmax(mag[w10]))
        row['dip_to10_mm'] = 1000.0 * float(np.nanmean(z[pre]) - np.nanmin(z[w10]))
        row['xy_err_peak_to10_cm'] = 100.0 * float(np.nanmax(e10))
        row['window_to10_s'] = round(full10 - te, 2)
    # recovery
    band_th = max(2.0 * sig, PR.BAND_MIN)
    z_m, z_sd = float(np.nanmean(z[pre])), float(np.nanstd(z[pre]))
    band_z = max(2.0 * z_sd, 0.01)
    e_m, e_sd = float(np.nanmean(e_pre)), float(np.nanstd(e_pre))
    band_e = max(2.0 * e_sd, 0.01)
    inb = (dev_th <= band_th) & (np.abs(z - z_m) <= band_z) & (e - e_m <= band_e)
    inb &= np.all(np.isfinite(th), 1) & np.isfinite(z) & np.isfinite(e)
    row['band_tilt_deg'], row['band_z_cm'], row['band_xy_cm'] = band_th, 100 * band_z, 100 * band_e
    norm = np.nanmax(np.stack([dev_th / band_th, np.abs(z - z_m) / band_z, (e - e_m) / band_e]), 0)
    row['worst_over_band'] = float(np.nanmax(norm[win])) if win.size else None
    which = np.nanargmax(np.stack([dev_th[win] / band_th, np.abs(z[win] - z_m) / band_z,
                                   (e[win] - e_m) / band_e]).max(1))
    row['worst_signal'] = ('tilt', 'height', 'horizontal')[int(which)]
    start = te + (row.get('impact_s') if row.get('impact_s') is not None else (row.get('obj_left_s') or 0.0))
    row['recover_from_s'] = round(start - te, 2)
    rec = None
    nh = int(round(HOLD * PR.HZ))
    for i in np.nonzero((t >= start) & (t + HOLD <= tend15))[0]:
        if inb[i:i + nh].all():
            rec = float(t[i] - te)
            break
    row['recover_s'] = rec
    # the same peaks on an "after" as long as "before" (the reviewer's equal windows, F116): release to +5 s, with the
    # same cuts; recovered within it only if back in band and held by +5 s
    end5 = min(te + PR.PRE_S, tend10)
    w5 = np.nonzero((t >= te) & (t < end5) & okt)[0]
    if w5.size >= 5:
        row['window_0to5_s'] = round(end5 - te, 2)
        row['tilt_peak_0to5_deg'] = float(np.nanmax(mag[w5]))
        row['dip_0to5_mm'] = 1000.0 * float(np.nanmean(z[pre]) - np.nanmin(z[w5]))
        row['xy_err_peak_0to5_cm'] = 100.0 * float(np.nanmax(e[w5]))
        row['recover_0to5'] = 'yes' if rec is not None and rec + HOLD <= end5 - te + 1e-6 else 'no'
    # each signal on its own (same start, band and hold)
    okall = np.all(np.isfinite(th), 1) & np.isfinite(z) & np.isfinite(e)
    for name, ok in (('tilt', dev_th <= band_th), ('z', np.abs(z - z_m) <= band_z), ('xy', e - e_m <= band_e)):
        ok = ok & okall
        row[f'recover_{name}_s'] = next((float(t[i] - te) for i in np.nonzero((t >= start) & (t + HOLD <= tend15))[0]
                                         if ok[i:i + nh].all()), None)
    if rec is None:
        row['recover_note'] = f'not back in band before the window end (+{tend15 - te:.1f} s, ' \
                              f'{window_label(F, te, tend15, desc) if tend15 < te + HORIZON - 1e-6 else "15 s"})'
    return row


def main():
    warnings.simplefilter('ignore', RuntimeWarning)
    fids = sys.argv[1:] or drop_flights()
    rows = []
    for fid in fids:
        print(fid, flush=True)
        rows.append(one(fid))
    head = ['flight', 'day', 'registry_id', 'motion', 'transfer', 'object', 'release_rule', 'released', 'carriers',
            'ring_speed_mps', 'ref_speed_mps', 'obj_dz_release_cm', 'obj_left_s', 'impact_s', 'impact_xy_cm',
            'obj_settled_cm', 'obj_settled_depth_cm', 'in_net', 'window_s', 'window_end', 'pre_s',
            'tilt_pre_deg', 'tilt_pre_max_deg', 'sigma_pre_deg', 'tilt_peak_deg', 'swing_deg', 'ring_z_pre_m',
            'z_sd_pre_mm', 'dip_pre_mm', 'dip_mm',
            'xy_err_pre_mean_cm', 'xy_err_pre_max_cm', 'xy_err_peak_cm', 'xy_err_rise_cm', 'recover_s',
            'settle_tilt_s', 'loss', 'note']
    out = 'drop_disturbance.csv' if len(sys.argv) == 1 else 'drop_disturbance_test.csv'
    C.write_csv(os.path.join(C.HERE, out), rows, head)
    print(f'wrote {len(rows)} rows')


if __name__ == '__main__':
    main()

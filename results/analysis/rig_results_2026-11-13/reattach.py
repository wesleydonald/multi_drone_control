#!/usr/bin/env python3
"""Q013 item 2: the re-attachments on Setup 3 (the M2A ring on 1 m cables, 4 Nov on), before and after the funnel.
Writes beside this file:
- reattach_attempts.csv: one row per attempt (a flight in which QUAD4 went back to the flying ring: after our
  DETACH and PARTNER RELEASE, after a drop into the net, or from its mat), with paper_rig's attach columns
  (Flight.attaches: descents, at plate height, contacts, misses, proven, folded in, held) and the funnel.
- reattach_resize.csv: one row per proven attachment that was folded in: contact -> proof -> fold-in -> blend end,
  the four planned tensions at the blend's start and end (our planner's RESIZE_INPUTS and ATTACH lines), the
  planner's node-0 tension and the trackers' estimated pull at the fold-in and at the blend's end, and the ring
  from the fold-in to +15 s (cut at LAND, a loss or the end of the record).
- reattach_tensions.csv: the same four tensions every 0.5 s from 2 s before the fold-in to +15 s (or LAND).
- joins_q013/: joins.py (rig_joins_2026-11-04_05, unchanged) re-run on these flights: every descent.

    source ~/ros2_humble/install/setup.bash && source ~/multi_drone_control/install/setup.bash
    cd ~/multi_drone_control && nice -n 10 python3 results/analysis/rig_results_2026-11-13/reattach.py

Rules (paper_rig's where it has one; judged after the flights, no bars):
- an attempt starts at PARTNER RELEASE (re-attach), CLEAR_DROP_ZONE (after a drop) or his TAKEOFF (from the mat);
  a flight whose drop was followed by no approach to the ring (TAKEN_BACK, RETURN_TO_START) is not an attempt.
  Not attempts: the pickup-and-drop flights from the mat (k1*), the missions whose pickup failed (5 Nov fm9_2, fm9_4,
  fm9s, fm9y; 11 Nov fm9c*; 12 Nov fm9_2, fm9c_3, fm9c_5), 11 Nov fm9hc_2/_4/_5 (object lost in his takeoff / landed
  in the creep), 11 Nov fm9hc_8 (DROP_OBJECT -> TAKEN_BACK), the 13 Nov C++ flights (RETURN_TO_START by design).
  sq6b (4 Nov) flew the 1.15 kg ring (Setup 2) and is left out; nr9 and nr1_2 (4 Nov) flew the M2A ring with the
  planner built for 1.15 kg (planner_ring_kg).
- descents: joins.py's (each entry into DESCEND_TO_ATTACHMENT) up to the proof; at plate height: the magnet got
  within 10 mm of the weld point's height; contact: our magnet manager's CONTACT line; proven: ATTACH: proven;
  held (paper_rig's success): proven, FLEET RESIZED back to 4, nothing lost and the magnet marker never 2 cm off its
  plate seat until LAND or 15 s.
- fold-in: our FLEET RESIZED to n=4 after the proof (the OCP resize; ATTACH drone 3 follows it by ~0.15 s). Blend
  end: fold-in + the tension blend's length (attach_traj_hold_s in the flight's planner params.json, 10 s on every
  flight here; the ATTACH line's "trajectory held 1.5 s" is the path's hold only). In the blend our planner moves
  the tension references linearly from (the three carriers at their shares before, QUAD4 at attach_t_start_new
  0.1 N) to the new balanced split (t_nom).
- planned tensions: t_from / t_nom of the RESIZE_INPUTS line (slot order, mapped to drones by the FLEET RESIZED
  line's slot->drone); node-0 tension: the planner log's t<slot> (the first node of the published plan, 10 Hz).
  Estimated pull: the tracker's raw measured cable acceleration |acm| (mocap acceleration minus the thrust map's
  thrust and gravity) x 0.55 kg, averaged over +-0.25 s. run.csv's d<i>_tension columns are empty on the rig.
- the ring from the fold-in: before = the 5 s before the fold-in; tilt peak = the largest tilt in the window;
  swing = the tilt vector's largest departure from its mean before; dip = the mean height before minus the lowest
  height in the window (+ = down); horizontal offset = the largest distance from the mean position before when the
  set-point stays put, else the peak horizontal distance to the set-point's path (clock-free, drops.py). Window:
  fold-in to the first of +15 s, LAND, a loss, the end of the record.
"""
import csv
import glob
import json
import math
import os
import re
import sys
import warnings

import numpy as np
import pandas as pd

import common as C
import drops as D

PR = C.PR
joins = C.joins
DAYS = ('2026-11-04', '2026-11-05', '2026-11-11', '2026-11-12', '2026-11-13')
SETUP2 = ('2026-11-04/sq6b',)          # the 1.15 kg ring (Setup 2)
DRONE_KG = 0.55
WIN = 15.0
FUNNEL = {   # (funnel, source); paper_rig's conditions.csv has no 12-13 Nov lines
    '2026-11-04': ('none', 'no funnel before 11 Nov (conditions.csv)'),
    '2026-11-05': ('none', 'no funnel before 11 Nov (conditions.csv)'),
    '2026-11-11/nrtfc': ('v4 (TPU)', 'conditions.csv; RIG-1111-funnel_c'),
    '2026-11-11/nrtfc_2': ('v4 (TPU)', 'conditions.csv; RIG-1111-funnel_c'),
    '2026-11-11/nrtfc_3': ('v4 (TPU)', 'conditions.csv; RIG-1111-funnel_c'),
    '2026-11-11/nrtfc_4': ('v4 (TPU)', 'conditions.csv; RIG-1111-funnel_c'),
    '2026-11-11': ("the supervisor's (105 mm)", 'RIG-1111-funnel_c: from nrtfc_5 (09:57) on'),
    '2026-11-12': ("the supervisor's (105 mm)", 'RIG-1112-fm9_5 / fm9hc_2 rows; sheet thu_2026-11-12'),
    '2026-11-13': ("the supervisor's (105 mm)", 'sheet fri_2026-11-13 ("funnel on plate 9"; not swapped on 12-13 Nov)'),
}


def funnel(fid):
    return FUNNEL.get(fid) or FUNNEL[fid[:10]]


def attempt_flights():
    """(fid, kind) for every flight of these days in which QUAD4 went back to the flying ring."""
    out = []
    for day in DAYS:
        for L in PR.launches(day):
            fid = f"{L['day']}/{L['name']}"
            if L['flew'] != 'yes' or L['args'].get('mode') != 'attach' or fid in SETUP2:
                continue
            a = L['args']
            welded = a.get('partner_attached', 'false') == 'true'
            mm = a.get('mission_mode') or 'floor_join'
            if mm.startswith('pickup') or mm.startswith('delivery'):
                J = joins.parse_log(os.path.join(C.RIG, day, L['name'] + '.log'))
                cz = next((p[0] for p in J['phases'] if p[1] == 'DROP_OBJECT'), None)
                back = cz is not None and any(p[0] > cz and p[2] in ('TRANSIT_TO_REATTACH', 'APPROACH_ABOVE_TARGET',
                                                                     'MATCH_VELOCITY', 'DESCEND_TO_ATTACHMENT')
                                              for p in J['phases'])
                if back:
                    out.append(fid)
            else:
                out.append(fid)
    return out


def run_joins(fids):
    outdir = os.path.join(C.HERE, 'joins_q013')
    os.makedirs(outdir, exist_ok=True)
    joins.OUT = outdir
    joins.DAYS = tuple(sorted({f[:10] for f in fids}))
    argv = sys.argv
    sys.argv = ['joins.py'] + list(fids)
    try:
        joins.main()
    finally:
        sys.argv = argv
    with open(os.path.join(outdir, 'joins.csv'), newline='') as f:
        return list(csv.DictReader(f))


def verdict(rid):
    best = ''
    with open(os.path.join(C.REPO, 'results', 'registry.csv'), newline='') as f:
        for r in csv.reader(f):
            if r and r[0] == rid:
                best = (r[9] if len(r) > 9 else '').strip()
    return best


def ring_speed_at(G, ts):
    """The median over ts of the ring's horizontal speed (mocap, the 1 s chord after each t)."""
    t = G.t.to_numpy(float)
    xy = G[['payload_x', 'payload_y']].to_numpy(float)
    v = [D.chord_speed(t, xy, x + 1.0) for x in ts]
    v = [x for x in v if x is not None]
    return float(np.median(v)) if v else None


def ring_during(F, descs, vs):
    motion = C.MOTION.get(F.args.get('load_traj') or (F.geo or {}).get('load_traj'))
    if vs is None:
        return None
    if motion == 'hover':
        return 'hover'
    paused = [t for t, _ in C.log_lines(F.day, F.name, r'attach pause: the orbit slows to rest')
              if descs and t < descs[0]]
    if vs < 0.05:
        return f'{motion} paused for the re-attach (ring at rest)' if paused else f'{motion} config, ring at rest'
    return f'{motion}, ring moving {vs:.2f} m/s (no pause)'


def attempt_row(F, descents):
    rows = F.attaches(descents)
    if not rows:
        return None
    r = rows[0]
    fn, fsrc = funnel(F.fid)
    start = F.t_takeoff + r['t_start_s'] - 0.01 if r.get('t_start_s') is not None else None
    pv = next((p for p in F.J['proven'] if start is None or p[0] >= start), None)
    stop = pv[0] if pv else (F.t_land or math.inf)
    descs = [p[0] for p in F.J['phases'] if p[2] == 'DESCEND_TO_ATTACHMENT' and (start or -math.inf) <= p[0] <= stop]
    pf = [p for p in F.J['proof_failed'] if (start or -math.inf) <= p[0] <= stop]
    exclude = ('VOID in the registry' if F.void else
               ('excluded in conditions.csv: ' + F.cond['note']) if F.cond['exclude'] else '')
    vs = ring_speed_at(F.G, descs) if (descs and F.G is not None) else None
    row = {'flight': F.name, 'day': F.day, 'registry_id': F.rid, 'kind': r['flight_kind'],
           'started_from': r.get('start_from'), 'planner_ring_kg': (F.geo or {}).get('load_mass'),
           'formation': r.get('formation'), 'plate': r.get('plate'), 'motion_config': C.MOTION.get(
               F.args.get('load_traj') or (F.geo or {}).get('load_traj')),
           'ring_speed_at_descents_mps': vs, 'ring_during_reattach': ring_during(F, descs, vs),
           'funnel': fn, 'funnel_source': fsrc, 't_start_s': r.get('t_start_s'),
           'descents': r.get('descents'), 'descents_at_plate_height': r.get('descents_at_plate_height'),
           'contacts': r.get('contacts'), 'proofs_failed': len(pf), 'proven': r.get('proven'),
           'folded_in': r.get('folded_in'), 'held': r.get('success'),
           'held_s': r.get('carried_on_s') if r.get('proven') == 'yes' else None,
           'miss_median_cm': r.get('miss_median_cm'), 'miss_min_cm': r.get('miss_min_cm'),
           'miss_max_cm': r.get('miss_max_cm'), 'start_to_proven_s': r.get('start_to_proven_s'),
           'start_to_foldin_s': r.get('start_to_foldin_s'), 'contact_to_foldin_s': r.get('contact_to_foldin_s'),
           'contact_xy_mm': r.get('contact_xy_mm'), 'fail_reason': r.get('fail_reason', ''),
           'exclude': exclude, 'verdict': verdict(F.rid)}
    return row


# ------------------------------------------------------------------------------------------- the resize at a proof
def planner_log(F):
    hits = sorted(glob.glob(os.path.join(C.RIG, F.day, F.name + '_logs', 'logs', 'dissipative_planner', '*',
                                         'log.csv')))
    if not hits:
        return None, {}
    d = pd.read_csv(hits[-1], low_memory=False).sort_values('sim_time').reset_index(drop=True)
    pj = os.path.join(os.path.dirname(hits[-1]), 'params.json')
    p = json.load(open(pj)) if os.path.exists(pj) else {}
    return d, p.get('params', p)


def trackers(F):
    out = {}
    for i in range(4):
        hits = sorted(glob.glob(os.path.join(C.RIG, F.day, F.name + '_logs', 'logs', 'tracker', f'drone{i}_*',
                                             'log.csv')))
        if hits:
            d = pd.read_csv(hits[-1], usecols=['sim_time', 'acm_x', 'acm_y', 'acm_z']).sort_values('sim_time')
            d['pull'] = DRONE_KG * np.sqrt(d.acm_x ** 2 + d.acm_y ** 2 + d.acm_z ** 2)
            out[i] = d[['sim_time', 'pull']].to_numpy(float)
    return out


def planned_by_drone(P, t, max_age=0.25, first_after=False):
    """{drone: node-0 planned tension} from the planner log row at or before t (first_after: the first row at or
    after t, so the fold-in reads the first n=4 plan)."""
    if P is None:
        return {}
    ts = P.sim_time.to_numpy(float)
    if first_after:
        k = int(np.searchsorted(ts, t, side='left'))
        if k >= len(ts) or ts[k] - t > max_age:
            return {}
    else:
        k = int(np.searchsorted(ts, t, side='right')) - 1
        if k < 0 or t - ts[k] > max_age:
            return {}
    s2d = [int(x) for x in str(P.slot2drone.iloc[k]).split()]
    out = {}
    for s, d in enumerate(s2d):
        v = P[f't{s}'].iloc[k]
        if pd.notna(v):
            out[d] = float(v)
    return out


def pull_at(T, i, t, half=0.25):
    a = T.get(i)
    if a is None:
        return None
    w = (a[:, 0] >= t - half) & (a[:, 0] <= t + half) & np.isfinite(a[:, 1])
    return float(np.mean(a[w, 1])) if w.sum() >= 3 else None


def resize_inputs(F, t_fold):
    """The RESIZE_INPUTS JSON of the attach at t_fold and the slot->drone of its FLEET RESIZED line."""
    ri = [(t, m) for t, m in C.log_lines(F.day, F.name, r'\[dissipative\] RESIZE_INPUTS (\{.*\})$')
          if t_fold - 0.5 <= t <= t_fold + 1.0]
    rs = next((r for r in F.L['resize'] if abs(r[0] - t_fold) < 0.02), None)
    s2d = [int(x) for x in rs[2].split(',')] if rs else None
    if not ri:
        return None, s2d
    return json.loads(ri[0][1].group(1)), s2d


def ring_after(F, te, tend):
    G = F.G
    t = G.t.to_numpy(float)
    th = G.attrs['th']
    z = G.payload_z.to_numpy(float)
    xy = G[['payload_x', 'payload_y']].to_numpy(float)
    ref = G[['payload_ref_x', 'payload_ref_y']].to_numpy(float)
    pre = (t >= te - PR.PRE_S) & (t < te) & np.all(np.isfinite(th), 1)
    win = np.nonzero((t >= te) & (t <= tend) & np.all(np.isfinite(th), 1))[0]
    out = {}
    if pre.sum() < PR.HZ or win.size < 5:
        out['ring_note'] = 'too little ring pose'
        return out
    mag = np.linalg.norm(th, axis=1)
    mean = th[pre].mean(0)
    dev = np.linalg.norm(th - mean, axis=1)
    out['tilt_pre_deg'] = float(mag[pre].mean())
    out['tilt_peak_deg'] = float(np.nanmax(mag[win]))
    out['t_tilt_peak_s'] = float(t[win[int(np.nanargmax(mag[win]))]] - te)
    out['swing_deg'] = float(np.nanmax(dev[win]))
    out['dip_mm'] = 1000.0 * float(np.nanmean(z[pre]) - np.nanmin(z[win]))
    out['t_dip_s'] = float(t[win[int(np.nanargmin(z[win]))]] - te)
    vref = [D.chord_speed(t, ref, t[i]) for i in win[::10]]
    moving = max([v for v in vref if v is not None], default=0.0) > 0.03
    if moving:
        e = D.path_err(t, xy, ref, win)
        out['horiz_offset_cm'] = 100.0 * float(np.nanmax(e))
        out['horiz_rule'] = ('peak horizontal distance to the set-point path (clock-free): ' +
                             ('the circle resumed' if C.MOTION.get(F.args.get('load_traj') or (F.geo or {}).get(
                                 'load_traj')) != 'hover' else 'the set-point moved in the window'))
    else:
        out['horiz_offset_cm'] = 100.0 * float(np.nanmax(np.linalg.norm(xy[win] - np.nanmean(xy[pre], 0), axis=1)))
        out['horiz_rule'] = 'peak distance from the mean position before'
    if not moving:
        e = D.path_err(t, xy, ref, win)
    out['xy_err_peak_cm'] = 100.0 * float(np.nanmax(e))
    return out


def resize_row(F, att, P, params, T):
    start = F.t_takeoff + att['t_start_s'] - 0.01 if att.get('t_start_s') is not None else -math.inf
    pv = next(p for p in F.J['proven'] if p[0] >= start)
    t_fold = next(r[0] for r in F.L['resize'] if pv[0] - 0.5 <= r[0] <= pv[0] + 5.0 and int(r[1]) >= 4)
    con = [c for c in F.J['contacts'] if start <= c[0] <= pv[0]]
    blend_s = float(params.get('attach_traj_hold_s', 0.0) or 0.0) or float(params.get('reconfig_hold_s', 1.5))
    t_blend = t_fold + blend_s
    lo = F.loss_between(t_fold, math.inf)
    ends = [t_fold + WIN, F.t_end] + ([F.t_land] if F.t_land and F.t_land > t_fold else []) + ([lo[0]] if lo else [])
    tend = min(ends)
    rx_att = r'\[dissipative\] ATTACH drone 3 \(OCP resize.*?trajectory (held [\d.]+ s|held until[^;]*)'
    att_line = next((m for t, m in C.log_lines(F.day, F.name, rx_att) if t >= t_fold - 0.5), None)
    hold_over = next((t for t, _ in C.log_lines(F.day, F.name, r'reconfiguration hold over') if t > t_fold), None)
    row = {'flight': F.name, 'day': F.day, 'registry_id': F.rid, 'kind': att['flight_kind'],
           'funnel': funnel(F.fid)[0], 'ring_during_reattach': None,
           't_contact_s': F.rel(con[-1][0]) if con else None, 'contact_try': con[-1][1] if con else None,
           't_proof_s': F.rel(pv[0]), 't_foldin_s': F.rel(t_fold), 't_blend_end_s': F.rel(t_blend),
           'contact_to_proof_s': round(pv[0] - con[-1][0], 2) if con else None,
           'proof_to_foldin_s': round(t_fold - pv[0], 3), 'blend_s': blend_s,
           'path_hold': att_line.group(1) if att_line else None,
           'path_resumes_s': round(hold_over - t_fold, 2) if hold_over else None,
           't_land_s': F.rel(F.t_land), 'land_after_foldin_s': round(F.t_land - t_fold, 2) if F.t_land else None,
           'blend_done_before_land': ('yes' if (F.t_land or math.inf) >= t_blend else 'no'),
           'window_s': round(tend - t_fold, 2),
           'window_end': ('LAND' if F.t_land and abs(tend - F.t_land) < 1e-6 else
                          ('loss: ' + lo[1]) if lo and abs(tend - lo[0]) < 1e-6 else
                          'end of record' if abs(tend - F.t_end) < 1e-6 else f'{WIN:.0f} s'),
           'held': att.get('success'), 'fail_reason': att.get('fail_reason', '')}
    ri, s2d = resize_inputs(F, t_fold)
    rs = next((r for r in F.L['resize'] if abs(r[0] - t_fold) < 0.02), None)
    if rs and s2d:      # each drone's plate (clockwise numbering) from the line's angles (cw)
        ang = [float(a) * (1.0 if rs[3].startswith('angles') else -1.0) for a in rs[4].split(',') if a.strip()]
        for s, d in enumerate(s2d):
            row[f'plate_d{d}'] = int(round(ang[s] / 30.0)) % 12
            if d == 3:
                row['quad4_catch_angle_cw_deg'] = ang[s] % 360.0
    if ri and s2d:
        for s, d in enumerate(s2d):
            row[f'T_start_d{d}_N'] = ri['t_from'][s]
            row[f'T_end_d{d}_N'] = ri['t_nom'][s]
    for lab, tt in (('fold', t_fold), ('blend_end', t_blend)):
        pl = planned_by_drone(P, tt, max_age=0.5 if lab == 'fold' else 0.25, first_after=(lab == 'fold'))
        for d in range(4):
            row[f'Tplan0_{lab}_d{d}_N'] = pl.get(d)
            row[f'Tpull_{lab}_d{d}_N'] = pull_at(T, d, tt) if tt <= tend + 1e-6 else None
    row.update(ring_after(F, t_fold, tend))
    for k in ('tilt_peak_deg', 'dip_mm', 'xy_excursion_cm', 'path_rms_cm', 'settle_new_s', 'tilt_after_deg'):
        row[f'paper_rig_{k}'] = att.get(k)
    return row, t_fold, tend, s2d


def tension_series(F, t_fold, tend, P, T):
    out = []
    for tt in np.arange(t_fold - 2.0, min(t_fold + WIN, F.t_land or math.inf, F.t_end) + 1e-6, 0.5):
        pl = planned_by_drone(P, tt)
        for d in range(4):
            out.append({'flight': F.name, 'day': F.day, 't_from_foldin_s': round(tt - t_fold, 2),
                        't_s': F.rel(tt), 'drone': d, 'n_ours': F.n_at(tt), 'T_plan_node0_N': pl.get(d),
                        'T_pull_est_N': pull_at(T, d, tt)})
    return out


def main():
    warnings.simplefilter('ignore', RuntimeWarning)
    fids = attempt_flights()
    print('attempts:', ' '.join(fids), flush=True)
    descents = run_joins(fids)
    att_rows, rz_rows, ts_rows = [], [], []
    for fid in fids:
        print(fid, flush=True)
        F = C.flight(fid)
        row = attempt_row(F, descents)
        if row is None:
            print(f'  {fid}: paper_rig has no attach row', flush=True)
            continue
        att_rows.append(row)
        att = F.attaches(descents)[0]
        if att.get('folded_in') != 'yes':
            continue
        P, params = planner_log(F)
        T = trackers(F)
        rz, t_fold, tend, _ = resize_row(F, att, P, params, T)
        rz['ring_during_reattach'] = row['ring_during_reattach']
        rz['exclude'] = row['exclude']
        rz_rows.append(rz)
        ts_rows += tension_series(F, t_fold, tend, P, T)
    head_a = ['flight', 'day', 'registry_id', 'kind', 'started_from', 'planner_ring_kg', 'formation', 'plate',
              'motion_config', 'ring_during_reattach', 'funnel', 't_start_s', 'descents', 'descents_at_plate_height',
              'contacts', 'proofs_failed', 'proven', 'folded_in', 'held', 'held_s', 'miss_median_cm', 'miss_min_cm',
              'miss_max_cm', 'start_to_proven_s', 'start_to_foldin_s', 'contact_to_foldin_s', 'contact_xy_mm',
              'fail_reason', 'exclude', 'verdict', 'funnel_source', 'ring_speed_at_descents_mps']
    C.write_csv(os.path.join(C.HERE, 'reattach_attempts.csv'), att_rows, head_a)
    head_r = ['flight', 'day', 'registry_id', 'kind', 'funnel', 'ring_during_reattach', 't_contact_s',
              'contact_try', 't_proof_s', 't_foldin_s', 't_blend_end_s', 'contact_to_proof_s', 'proof_to_foldin_s',
              'blend_s', 'path_hold', 'path_resumes_s', 't_land_s', 'land_after_foldin_s', 'blend_done_before_land']
    head_r += [f'plate_d{d}' for d in range(4)] + ['quad4_catch_angle_cw_deg']
    head_r += [f'T_start_d{d}_N' for d in range(4)] + [f'T_end_d{d}_N' for d in range(4)]
    head_r += ['window_s', 'window_end', 'tilt_pre_deg', 'tilt_peak_deg', 't_tilt_peak_s', 'swing_deg', 'dip_mm',
               't_dip_s', 'horiz_offset_cm', 'horiz_rule', 'xy_err_peak_cm', 'held', 'fail_reason', 'exclude']
    C.write_csv(os.path.join(C.HERE, 'reattach_resize.csv'), rz_rows, head_r)
    C.write_csv(os.path.join(C.HERE, 'reattach_tensions.csv'), ts_rows)
    print(f'wrote {len(att_rows)} attempts, {len(rz_rows)} resizes, {len(ts_rows)} tension samples')


if __name__ == '__main__':
    main()

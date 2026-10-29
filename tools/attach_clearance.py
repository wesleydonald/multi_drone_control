#!/usr/bin/env python3
"""tools/attach_clearance.py -- the metrics of card 2026-10-28_attach_ambitious, one run directory at a time, so every
arm (baseline, V1, V2+V3, V4, combined) is scored the same way.

    python3 tools/attach_clearance.py RUN... [--json] [--bar 0.25]

RUN is a run directory or a run id (tools/m1_metrics.run_dir). Times are the runner's sim time (events.csv, run.csv).

Window: his TAKEOFF (the first PHASE:TAKEOFF) to the weld (the first timed WELD, else REWELD), else to LAND, the fleet
abort or the end of the log.

1. Clearance, the minimum over the window of every pair between QUAD4 {body, cable, object} and each carrier {body,
   cable}, on centres and centrelines (no radii taken off):
   QUAD4's body = d<n>_* (n = the carriers), its cable the segment body -> magnet tip (tip_*); the object = the ball's
   surface while he carries it (PICKUP .. DROP: its centre distance less BALL_R); a carrier's body = d<i>_*, its cable
   the segment from its pivot (pivot_offset_z under the body) to its plate (attach_radius, attach_z, attach_azimuths_deg
   in the ring frame, the ring's measured attitude applied). Reported: the minimum, which pair, which carrier (1 =
   /drone_0), when, and his phase then; the minimum per pair kind; the same from his TAKEOFF to his first descent
   (clearance_to_descend: the legs before the tip goes down to the plate, where plate 9's neighbours bound it).
2. Time his TAKEOFF -> weld; MATCH_VELOCITY start -> DESCEND_TO_ATTACHMENT start (the first MATCH after his TAKEOFF
   and the first descent after it; also each MATCH entry's wait, and the MATCH entries counted).
3. Tries (CONTACT events up to the weld) and the proof: launch.log's 'ATTACH: proven (try k)' / 'PROOF FAILED' lines.
4. Ring tilt peak over the window (and the mean over the 5 s before his TAKEOFF: the carriers' lean); aborts:
   FLEET_ABORT, GIVE_UP, TAKE_BACK, WELD_TIMEOUT / REWELD_TIMEOUT events, launch.log's ENVELOPE FAULT and POSITION
   LIMIT lines. A fleet abort after LAND (the touchdown) is reported apart with tools/m1_metrics.tb14's reading, and the
   safety bar column counts aborts from his TAKEOFF to LAND only.
Also, for V1: the magnet tip's height above plate 9's face (the logged contact target tip_target_z less the 0.012 tip
sphere: the tilted ring's plate; else ring z + attach_z) when MATCH and the first descent start, and its lowest over
the transits (TRANSIT_TO_REATTACH, TRANSIT_TO_DROP_POINT) while the tip is over the ring area (within the carriers'
circle, attach_radius + cable_len cos 60 deg of the ring's centre); the pickup's ball in the net (metrics.json); the
time split from his TAKEOFF to the weld (critic finding 13); the revision-2 safety window, tools/m1_metrics.flight's
clearance (QUAD4's rod until his first CONTACT, the ball while carried; ring attitude not applied).
"""
import argparse
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import m1_metrics as M  # noqa: E402

BALL_R = M.BALL_R
TIP_R = 0.012                     # the twin's magnet sphere (tools/make_attach_worlds.TIP_R)
SPLIT = (('TAKEOFF', 'climb and install wait'), ('TRANSIT_TO_REATTACH', 'transit'),
         ('APPROACH_ABOVE_TARGET', 'approach'), ('MATCH_VELOCITY', 'MATCH'), ('DESCEND_TO_ATTACHMENT', 'descent'))
BAR_M = 0.25
TRANSITS = ('PHASE:TRANSIT_TO_REATTACH', 'PHASE:TRANSIT_TO_DROP_POINT')
CARRIER_ELEV_DEG = 60.0


def rotate(q, v):
    """v (3,) or (N, 3) rotated by quaternions q (N, 4) as w, x, y, z."""
    w, u = q[:, :1], q[:, 1:]
    t = 2.0 * np.cross(u, v)
    return v + w * t + np.cross(u, t)


def window(r):
    a = r.t('PHASE:TAKEOFF')
    tw, kind = r.attach()
    if tw is not None and a is not None and tw < a:
        tw, kind = None, None
    ends = [x for x in (r.t('LAND', after=a), r.t('FLEET_ABORT', after=a)) if x is not None] if a is not None else []
    b = tw if tw is not None else (min(ends) if ends else float(r.tt[-1]))
    return a, b, tw, kind


def phase_at(r, t):
    ph = r.ev[(r.ev.event.str.startswith('PHASE:')) & (r.ev.sim_time <= t)]
    return ph.event.iloc[-1][6:] if len(ph) else ''


def clearance(r, mask):
    t = r.tt
    body, tip, ball = r.xyz(r.k), r.xyz('tip_'), r.xyz('ball_')
    ring = r.xyz('payload_')
    q = np.stack([r.col('payload_q' + c) for c in 'wxyz'], -1)
    q = np.where(np.isfinite(q).all(1, keepdims=True), q, np.array([1.0, 0, 0, 0]))
    carried = M.ball_carried(r, t) & np.isfinite(ball).all(1)
    pivot_dz = float(r.man.get('pivot_offset_z') or -0.04)
    kinds = {}

    def put(kind, i, d):
        d = np.where(mask, d, np.inf)
        cur = kinds.get(kind)
        if cur is None or np.nanmin(d) < cur[0]:
            j = int(np.nanargmin(d))
            kinds[kind] = (float(d[j]), i, float(t[j]))

    for i, az in enumerate(r.az):
        cb = r.xyz(f'd{i}_')
        piv = cb + np.array([0.0, 0.0, pivot_dz])
        off = np.array([r.r_att * math.cos(math.radians(az)), r.r_att * math.sin(math.radians(az)), r.z_att])
        plate = ring + rotate(q, np.broadcast_to(off, ring.shape))
        ok_tip = np.isfinite(tip).all(1)
        put('body-body', i, np.linalg.norm(body - cb, axis=1))
        put('body-cable', i, M.seg_seg(body, body, piv, plate))
        put('cable-body', i, np.where(ok_tip, M.seg_seg(body, tip, cb, cb), np.inf))
        put('cable-cable', i, np.where(ok_tip, M.seg_seg(body, tip, piv, plate), np.inf))
        if carried.any():
            put('object-body', i, np.where(carried, np.linalg.norm(ball - cb, axis=1) - BALL_R, np.inf))
            put('object-cable', i, np.where(carried, M.seg_seg(ball, ball, piv, plate) - BALL_R, np.inf))
    kinds = {k: v for k, v in kinds.items() if np.isfinite(v[0])}
    if not kinds:
        return None
    kind, (d, i, tm) = min(kinds.items(), key=lambda kv: kv[1][0])
    return {'min_m': round(d, 3), 'pair': kind, 'carrier': i + 1, 't': round(tm, 2), 'phase': phase_at(r, tm),
            'by_pair_m': {k: round(v[0], 3) for k, v in sorted(kinds.items())}}


def tip_heights(r, a, b):
    """(tip above the plate plane when MATCH starts, when the first descent starts, the transits' lowest tip over the
    ring area) in m."""
    t, tip, ring = r.tt, r.xyz('tip_'), r.xyz('payload_')
    # the plate face: the logged contact target (plate 9 on the tilted ring) less the tip sphere, else the level plane
    face = r.col('tip_target_z') - TIP_R
    above = tip[:, 2] - np.where(np.isfinite(face), face, ring[:, 2] + r.z_att)

    def at(name):
        tm = r.t(name, after=a)
        return None if tm is None else M.r2(above[int(np.argmin(np.abs(t - tm)))], 3)
    cable = float(r.man.get('cable_len') or 1.0)
    reach = r.r_att + cable * math.cos(math.radians(CARRIER_ELEV_DEG))
    over = np.linalg.norm(tip[:, :2] - ring[:, :2], axis=1) <= reach
    ph = r.ev[r.ev.event.str.startswith('PHASE:')].reset_index(drop=True)
    transit = np.zeros(len(t), bool)
    for k, row in ph.iterrows():
        if row.event in TRANSITS:
            end = ph.sim_time.iloc[k + 1] if k + 1 < len(ph) else b
            transit |= (t >= row.sim_time) & (t < end)
    m = transit & over & (t >= a) & (t <= b) & np.isfinite(above)
    return at('PHASE:MATCH_VELOCITY'), at('PHASE:DESCEND_TO_ATTACHMENT'), (M.r2(float(above[m].min()), 3) if m.any()
                                                                            else None)


def match_waits(r, a, b):
    """[MATCH start -> the next DESCEND start (s), or None if he left MATCH another way] for each MATCH entry."""
    ph = r.ev[r.ev.event.str.startswith('PHASE:') & (r.ev.sim_time >= a) & (r.ev.sim_time <= b + 1e-6)]
    ph = ph.reset_index(drop=True)
    out = []
    for k, row in ph.iterrows():
        if row.event == 'PHASE:MATCH_VELOCITY':
            nxt = ph.iloc[k + 1] if k + 1 < len(ph) else None
            out.append(round(float(nxt.sim_time - row.sim_time), 2)
                       if nxt is not None and nxt.event == 'PHASE:DESCEND_TO_ATTACHMENT' else None)
    return out


def time_split(r, a, b):
    """Seconds per stage from his TAKEOFF to the weld (critic finding 13): his TAKEOFF split at the end of the climb
    (QUAD4's body first within 5 cm of its highest point in that phase) into climb and his install wait, then each
    phase's total time, the bottom (ATTACH_READY, LIFT_PROOF) to the weld, and the pickup/drop phases as 'object'."""
    ph = r.ev[r.ev.event.str.startswith('PHASE:') & (r.ev.sim_time >= a) & (r.ev.sim_time <= b + 1e-6)]
    ph = ph.reset_index(drop=True)
    names = dict(SPLIT)
    out = {}
    t, z = r.tt, r.col(r.k + 'z')
    for k, row in ph.iterrows():
        end = float(ph.sim_time.iloc[k + 1]) if k + 1 < len(ph) else b
        p = row.event[6:]
        if p == 'TAKEOFF' and 'climb' not in out:
            m = (t >= row.sim_time) & (t <= end) & np.isfinite(z)
            top = float(t[m][np.argmax(z[m] >= z[m].max() - 0.05)]) if m.any() else end
            out['climb'] = round(top - row.sim_time, 1)
            out['install wait'] = round(end - top, 1)
            continue
        key = names.get(p, 'bottom' if p in ('ATTACH_READY', 'LIFT_PROOF') else 'object')
        out[key] = round(out.get(key, 0.0) + end - float(row.sim_time), 1)
    return out


def score(rid, bar=BAR_M):
    r = M.Run(rid)
    a, b, tw, kind = window(r)
    out = {'run': r.id, 'config': r.config, 'his_takeoff': a, 'weld': tw, 'weld_event': kind, 'window_end': M.r2(b, 2)}
    if a is None:
        out['note'] = 'no PHASE:TAKEOFF'
        return out
    t = r.tt
    mask = (t >= a) & (t <= b)
    out['clearance'] = clearance(r, mask)
    d = r.t('PHASE:DESCEND_TO_ATTACHMENT', after=a)
    out['clearance_to_descend'] = clearance(r, (t >= a) & (t <= d)) if d is not None and d <= b else None
    out['takeoff_to_weld_s'] = M.r2(tw - a, 1) if tw is not None else None
    out['time_split_s'] = time_split(r, a, b)
    f5 = M.flight(r, legacy=False)
    out['m1_flight'] = {k: f5[k] for k in ('rod_min', 'rod_at', 'rod_carrier', 'body_min', 'ball_min', 'rod_to')}
    waits = match_waits(r, a, b)
    out['match_entries'] = len(waits)
    out['match_to_descend_s'] = next((w for w in waits if w is not None), None)
    out['match_waits_s'] = waits
    out['tries'] = int(len(r.times('CONTACT', after=a, before=b + 1e-6)))
    log = r.log
    out['proof'] = ([ln.split('ATTACH: ', 1)[1][:60] for ln in log if 'ATTACH: proven' in ln][:1] or [None])[0]
    out['proof_failed'] = sum('PROOF FAILED' in ln for ln in log)
    tilt = r.df.payload_tilt_deg.to_numpy(float)
    out['ring_tilt_peak_deg'] = M.r2(float(np.nanmax(tilt[mask])), 2)
    j = int(np.nanargmax(np.where(mask, tilt, -np.inf)))
    out['ring_tilt_peak_at'] = {'t': M.r2(float(t[j]), 2), 'phase': phase_at(r, t[j])}
    out['ring_tilt_before_takeoff_deg'] = M.r2(float(np.nanmean(tilt[(t >= a - 5.0) & (t < a)])), 2)
    (out['tip_above_plate_at_match_m'], out['tip_above_plate_at_descend_m'],
     out['transit_tip_min_above_plate_m']) = tip_heights(r, a, b)
    out['events'] = {e: r.t(e) for e in ('FLEET_ABORT', 'GIVE_UP', 'TAKE_BACK', 'WELD_TIMEOUT', 'REWELD_TIMEOUT',
                                         'PICKUP', 'DROP', 'LAND', 'LANDED') if r.t(e) is not None}
    out['envelope_faults'] = sum('ENVELOPE FAULT' in ln for ln in log)
    out['position_limit'] = sum('POSITION LIMIT' in ln for ln in log)
    mpath = os.path.join(r.d, 'metrics.json')
    if os.path.exists(mpath) and r.t('PICKUP') is not None:
        at = json.load(open(mpath)).get('m1', {}).get('ball_at', {}).get('LAND', {})
        out['ball_in_net_at_land'] = at.get('in_net')
    # an abort after LAND (the touchdown) is reported apart, with the TB14 reading (QUAD4's rigid twin rod tipping it
    # at touchdown: a twin artefact, decisions 9 and 22 Oct); its envelope fault is the abort's
    land, abort = r.t('LAND', after=a), r.t('FLEET_ABORT', after=a)
    late = abort is not None and land is not None and abort > land
    out['abort_after_land'] = ({'t': abort, 'after_land_s': M.r2(abort - land, 2), 'tb14': bool(M.tb14(r).get('tb14'))}
                               if late else None)
    c = out['clearance']
    out['safety_bar'] = bool(c and c['min_m'] >= bar and tw is not None and out['proof'] is not None
                             and (abort is None or late) and (not out['envelope_faults'] or late)
                             and not out['position_limit'])
    return out


def row(o):
    c = o.get('clearance') or {}
    f = lambda v, u='': '-' if v is None else f'{v}{u}'  # noqa: E731
    ev = o.get('events', {})
    late = o.get('abort_after_land')
    aborts = [k for k in ('FLEET_ABORT', 'GIVE_UP', 'TAKE_BACK', 'WELD_TIMEOUT', 'REWELD_TIMEOUT') if k in ev
              and not (late and k == 'FLEET_ABORT')]
    if late:
        aborts.append(f"after LAND +{late['after_land_s']} s{' (TB14 tip-over)' if late['tb14'] else ''}")
    elif o.get('envelope_faults'):
        aborts.append(f"ENVELOPE x{o['envelope_faults']}")
    if o.get('position_limit'):
        aborts.append('POSITION LIMIT')
    clr = (f"{c['min_m']} ({c['pair']}, carrier {c['carrier']}, {c['phase']} at {c['t']} s)" if c else '-')
    return (f"| {o['run']} | {clr} | {f(o.get('takeoff_to_weld_s'), ' s')} | {f(o.get('match_to_descend_s'), ' s')} "
            f"({o.get('match_entries', 0)} MATCH) | {o.get('tries', '-')} | {o.get('proof') or 'none'} | "
            f"{f(o.get('ring_tilt_peak_deg'), ' deg')} | {', '.join(aborts) or 'none'} | "
            f"{'PASS' if o.get('safety_bar') else 'FAIL'} |")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('runs', nargs='+')
    ap.add_argument('--json', action='store_true', help='one JSON line per run instead of the table')
    ap.add_argument('--bar', type=float, default=BAR_M, help='the safety bar\'s clearance (m)')
    a = ap.parse_args()
    out = [score(rid, a.bar) for rid in a.runs]
    if a.json:
        for o in out:
            print(json.dumps(o, default=float))
        return 0
    print('| run | clearance min (pair, carrier, phase, t) | his TAKEOFF -> weld | MATCH -> DESCEND | tries | proof '
          '| ring tilt peak | aborts | safety bar |')
    print('|---|---|---|---|---|---|---|---|---|')
    for o in out:
        print(row(o))
    return 0


if __name__ == '__main__':
    sys.exit(main())

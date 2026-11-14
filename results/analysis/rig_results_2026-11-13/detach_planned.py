#!/usr/bin/env python3
"""Q013 item 1: every planned detachment on Setup 3 (the M2A ring on 1 m cables, 4 Nov on), one row each, with
paper_rig's detach columns, plus the Setup 1 unannounced detections of 8 Oct copied from paper_rig's events.csv.
Writes detach_planned.csv beside this file.

    source ~/ros2_humble/install/setup.bash && source ~/multi_drone_control/install/setup.bash
    cd ~/multi_drone_control && nice -n 10 python3 results/analysis/rig_results_2026-11-13/detach_planned.py

Flights: every launch log of 4-6 and 11-13 Nov with our planner's "UNLOAD drone" line followed by "UNLOADED"
(the soft detach: QUAD4's planned tension blended to 0.5 N over 3 s, our magnet OFF, FLEET RESIZED to n=3,
DETACH). An UNANNOUNCED DETACH line is not a planned detach: RIG-1104-nr9m_3's (QUAD4's start weld read long 8 s
into the takeoff, LAND at once; VOID) and the ones logged after LAND (sq8_2, fm9c_6, nrtfc_5: carriers on the
floor) are listed in the report, not here.

Every metric is paper_rig's (Flight.detaches -> response; its docstring has the definitions), on paper_rig's
window: from our magnet OFF to the first of LAND, a loss, the next resize, QUAD4's next descent onto the ring or the
object, the end of the record and +15 s. Dip and the hover excursion use OFF .. min(OFF + 10 s, window end). Added
here: the unload's start to the resize, the ring's and its set-point's speed at the OFF (1 s chord), and the
horizontal error peak of drops.py (the ring's horizontal distance to the path its set-point traces within +-5 s,
clock-free; in a hover the distance to the set-point) on the same window as the dip, with its mean before.
horiz_offset_cm: paper_rig's excursion from the mean position before when the set-point stays put in OFF .. +10 s,
else the path error peak.

The ring at the OFF (motion_at_off): every planned detach on Setup 3 came with the ring at rest. The unload holds the
path (the set-point stops at UNLOAD; the DETACH holds it 1.5 s more), so nr9o_2 and nro, which began the unload on
the circle at 0.12-0.13 m/s, had coasted to 0.02 m/s by the OFF; the fm9c* detaches came in the hover hold before
the circle. setpoint_moves_after_off_s: when the set-point first moves after the OFF (1 s chord over 0.03 m/s).
paper_rig's tab:detach_performance counts the circle flights as "Planned, moving" (by the launch's load_traj).
"""
import csv
import glob
import math
import os
import re
import sys
import warnings

import numpy as np

import common as C
import drops as D

PR = C.PR
DAYS = ('2026-11-04', '2026-11-05', '2026-11-06', '2026-11-11', '2026-11-12', '2026-11-13')
SETUP1 = ('2026-10-08/r303_4', '2026-10-08/r305', '2026-10-08/r304_3')
NOTES = {
    '2026-11-04/nr9': 'planner built for the 1.15 kg ring (attach_m1_1m.yaml) while the M2A ring flew',
    '2026-11-04/nr1_2': 'planner built for the 1.15 kg ring (attach_m1_1m.yaml) while the M2A ring flew',
    '2026-11-05/sq8_2': 'square: QUAD1-3 on 11/2/5, QUAD4 on 8; the survivors span 180 deg (the T)',
    '2026-11-11/fm9c_4': 'the bigger object (OBJ=big); the circle paused for the pickup',
    '2026-11-11/fm9c_6': 'the bigger object; it fell off in the lift; QUAD4 flew on a frozen pose (RIG-1111-fm9c_6c)',
}


def planned_flights():
    out = []
    for day in DAYS:
        for fn in sorted(glob.glob(os.path.join(C.RIG, day, '*.log'))):
            txt = open(fn, errors='replace').read()
            if re.search(r'\[dissipative\] UNLOAD drone \d', txt) and 'UNLOADED drone' in txt:
                out.append(f'{day}/{os.path.basename(fn)[:-4]}')
    return out


def verdict(rid):
    with open(os.path.join(C.REPO, 'results', 'registry.csv'), newline='') as f:
        for r in csv.reader(f):
            if r and r[0] == rid:
                return (r[9] if len(r) > 9 else '').split(':')[0].strip()
    return ''


def one(fid):
    F = C.flight(fid)
    rows = [r for r in F.detaches() if r['event'] == 'detach (planned)']
    out = []
    G = F.G
    t = G.t.to_numpy(float)
    xy = G[['payload_x', 'payload_y']].to_numpy(float)
    ref = G[['payload_ref_x', 'payload_ref_y']].to_numpy(float)
    for u, r in zip([u for u in F.L['unload'] if any(x for x in F.L['unloaded'] if u[0] <= x[0] <= u[0] + 10.0)],
                    rows):
        te = F.t_takeoff + r['t_event_s']
        rs = next((x[0] for x in F.L['resize'] if te - 0.5 <= x[0] <= te + 2.0), None)
        traj = F.args.get('load_traj') or (F.geo or {}).get('load_traj')
        row = {'flight': F.name, 'day': F.day, 'registry_id': F.rid, 'setup': 3,
               'planner_ring_kg': (F.geo or {}).get('load_mass'), 'motion': C.MOTION.get(traj, traj),
               'traj_radius_m': (F.geo or {}).get('traj_radius'), 'formation': r.get('formation'),
               'event': 'planned (soft) detach of QUAD4'}
        row['ring_speed_mps'] = D.chord_speed(t, xy, te)
        row['ref_speed_mps'] = D.chord_speed(t, ref, te)
        row['ring_moving_at_off'] = ('yes' if (row['ref_speed_mps'] or 0.0) > 0.03 else 'no')
        row['t_unload_s'] = F.rel(u[0])
        row['t_off_s'] = r['t_event_s']
        row['unload_s'] = r.get('unload_s')
        row['unload_to_resize_s'] = round(rs - u[0], 3) if rs else None
        for k in ('resize_s', 'separation_s', 'unloaded_tilt_deg', 'window_s', 'window_end', 'pre_s', 'tilt_pre_deg',
                  'sigma_pre_deg', 'swing_deg', 't_swing_peak_s', 'tilt_peak_deg', 'settle_band_deg', 'settle_s',
                  'settle_2s_s', 'settle_new_s', 'tilt_after_deg', 'tilt_shift_deg', 'dip_mm', 'ring_z_min_m',
                  'xy_excursion_cm', 'sp_pre_rms_cm', 'sp_rms_cm', 'path_kind', 'path_rms_cm', 'success', 'loss',
                  'carried_on_s', 'fail_reason', 'resp_note'):
            row[k] = r.get(k)
        # drops.py's horizontal error (clock-free) on the dip's window, and its mean before
        tend = F.window_end(te, 'detach')
        aft = np.nonzero((t >= te) & (t <= min(te + PR.AFTER_S, tend)))[0]
        lo_pre = max(te - PR.PRE_S, PR.ref_still_from(G, te))
        ipre = np.nonzero((t >= lo_pre) & (t < te))[0]
        if aft.size > 5:
            e = D.path_err(t, xy, ref, aft)
            row['xy_err_peak_cm'] = 100.0 * float(np.nanmax(e))
            row['t_xy_err_peak_s'] = float(t[aft[int(np.nanargmax(e))]] - te)
        if ipre.size >= PR.HZ:
            row['xy_err_pre_mean_cm'] = 100.0 * float(np.nanmean(D.path_err(t, xy, ref, ipre)))
        # the set-point moving at any time in the window (a circle held 1.5 s at the resize, then resumed): the
        # path error; a set-point that stays put: paper_rig's excursion from the mean position before
        w = np.nonzero((t >= te - 1.0) & (t <= min(te + PR.AFTER_S, tend)))[0]
        vref = [D.chord_speed(t, ref, t[i]) for i in w[::10] if t[i] >= te]
        row['ref_speed_max_window_mps'] = max([v for v in vref if v is not None], default=None)
        moving = (row['ref_speed_max_window_mps'] or 0.0) > 0.03
        # a circle flight: was the circle running when the unload began? (the unload holds the path: the set-point
        # stops at UNLOAD and the circle resumes after the DETACH's 1.5 s hold)
        row['ring_speed_at_unload_mps'] = D.chord_speed(t, xy, u[0])
        row['ref_speed_at_unload_mps'] = D.chord_speed(t, ref, u[0])
        orb = [x for x, _ in C.log_lines(F.day, F.name, r'hover hold done: orbit starts')]
        row['orbit_start_after_off_s'] = round(orb[0] - te, 2) if orb else None
        # when the set-point first moves after the OFF (its 1 s chord over 0.03 m/s), searched to +20 s
        mv = next((float(t[i] - te) for i in np.nonzero((t > te) & (t <= te + 20.0))[0][::5]
                   if (D.chord_speed(t, ref, t[i]) or 0.0) > 0.03), None)
        row['setpoint_moves_after_off_s'] = round(mv, 1) if mv is not None else None
        if row['ring_moving_at_off'] == 'yes':
            row['motion_at_off'] = row['motion']
        elif row['motion'] == 'hover':
            row['motion_at_off'] = 'hover'
        elif (row['ref_speed_at_unload_mps'] or 0.0) > 0.03:
            row['motion_at_off'] = (f"{row['motion']} paused by the unload (ring {row['ring_speed_at_unload_mps']:.2f}"
                                    f" m/s at its start); the set-point moves again {mv:+.1f} s from the OFF")
        else:
            row['motion_at_off'] = (f"hover hold before the {row['motion']} (the set-point first moves "
                                    f"{mv:+.1f} s from the OFF)")
        row['horiz_offset_cm'] = row.get('xy_err_peak_cm') if moving else (
            row.get('xy_excursion_cm') if row.get('xy_excursion_cm') is not None else row.get('xy_err_peak_cm'))
        row['horiz_rule'] = ('peak horizontal distance to the set-point path (clock-free)' if moving else
                             'peak distance from the mean position before' if row.get('xy_excursion_cm') is not None
                             else 'peak horizontal distance to the set-point (paper_rig gave no excursion)')
        row['verdict'] = verdict(F.rid)
        row['funnel'] = F.cond['funnel']
        row['note'] = NOTES.get(fid, '')
        out.append(row)
    if len(rows) != len(out):
        print(f'  {fid}: {len(rows)} paper_rig rows, {len(out)} matched', flush=True)
    return out


def setup1_rows():
    """The 8 Oct rows of paper_rig's events.csv as they are (tab:detach_context_0p55)."""
    out = []
    with open(os.path.join(C.REPO, 'results', 'analysis', 'paper_rig_2026-11', 'events.csv'), newline='') as f:
        ev = {f"{r['day']}/{r['flight']}": r for r in csv.DictReader(f)}
    for fid in SETUP1:
        r = ev[fid]
        row = {'flight': r['flight'], 'day': r['day'], 'registry_id': r['registry_id'], 'setup': 1,
               'planner_ring_kg': r['load_kg_config'], 'motion': 'circle' if r['motion'] == 'moving' else 'hover',
               'formation': r['formation'], 'event': 'unannounced (operator magnet OFF, detected)',
               'ring_speed_mps': r['ring_speed_mps'], 't_off_s': r['t_event_s'],
               'detect_delay_s': r['detect_delay_s']}
        row['motion_at_off'] = row['motion']
        for k in ('resize_s', 'separation_s', 'window_s', 'window_end', 'pre_s', 'tilt_pre_deg', 'sigma_pre_deg',
                  'swing_deg', 't_swing_peak_s', 'tilt_peak_deg', 'settle_band_deg', 'settle_s', 'settle_2s_s',
                  'settle_new_s', 'tilt_after_deg', 'tilt_shift_deg', 'dip_mm', 'ring_z_min_m', 'xy_excursion_cm',
                  'sp_pre_rms_cm', 'sp_rms_cm', 'path_kind', 'path_rms_cm', 'success', 'loss', 'carried_on_s',
                  'fail_reason', 'resp_note'):
            row[k] = r.get(k, '')
        row['horiz_offset_cm'] = r['path_rms_cm'] if r['motion'] == 'moving' else r['xy_excursion_cm']
        row['horiz_rule'] = ('path deviation RMS 10 s after (paper_rig; no peak in events.csv)'
                             if r['motion'] == 'moving' else 'peak distance from the mean position before')
        row['note'] = ('0.55 m cables; as in paper_rig events.csv (tab:detach_context_0p55)' +
                       ('; plates 3/6/11: the survivors leave a 150 deg gap' if fid.endswith('r304_3') else ''))
        out.append(row)
    return out


HEAD = ['flight', 'day', 'registry_id', 'setup', 'event', 'planner_ring_kg', 'motion', 'traj_radius_m',
        'motion_at_off', 'ring_moving_at_off', 'ring_speed_mps', 'ring_speed_at_unload_mps',
        'orbit_start_after_off_s', 'setpoint_moves_after_off_s', 'ref_speed_max_window_mps', 'ref_speed_mps',
        'formation', 't_unload_s', 't_off_s', 'unload_s',
        'unload_to_resize_s', 'resize_s', 'detect_delay_s', 'separation_s', 'unloaded_tilt_deg', 'window_s',
        'window_end', 'pre_s', 'tilt_pre_deg', 'swing_deg', 'tilt_peak_deg', 'settle_s', 'settle_new_s',
        'tilt_shift_deg', 'dip_mm', 'horiz_offset_cm', 'horiz_rule', 'xy_excursion_cm', 'xy_err_pre_mean_cm',
        'xy_err_peak_cm', 'path_rms_cm', 'success', 'loss', 'verdict', 'note']


def main():
    warnings.simplefilter('ignore', RuntimeWarning)
    fids = sys.argv[1:] or planned_flights()
    rows = []
    for fid in fids:
        print(fid, flush=True)
        rows += one(fid)
    rows += setup1_rows()
    out = 'detach_planned.csv' if len(sys.argv) == 1 else 'detach_planned_test.csv'
    C.write_csv(os.path.join(C.HERE, out), rows, HEAD)
    print(f'wrote {len(rows)} rows to {out}')


if __name__ == '__main__':
    main()

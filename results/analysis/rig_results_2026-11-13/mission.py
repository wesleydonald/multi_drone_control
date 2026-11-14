#!/usr/bin/env python3
"""Q013 item 4: the full mission RIG-1112-fm9_5 phase by phase, and the moving-ring half of RIG-1112-fm9hc_2, _4, _5
(the drop into the circling ring, the pause, the re-attach), from the launch logs. Writes beside this file:
full_mission_fm9_5.csv and moving_half_fm9hc.csv (one row per phase), and mission_events.csv (every timed line used).

    source ~/ros2_humble/install/setup.bash && source ~/multi_drone_control/install/setup.bash
    cd ~/multi_drone_control && python3 results/analysis/rig_results_2026-11-13/mission.py

Times are seconds after the fleet's TAKEOFF command (the fleet_manager line). Each phase runs from the end of the one
before. n_ours = the quadrotors our planner flew on the ring (the last FLEET RESIZED line at the phase's start; the
start weld's resize puts QUAD4 in as the fourth carrier at launch); quad4_by = who flew QUAD4. The ring at the top =
paper_rig's ring_up (the set-point at the top of the lift and the ring within 10 cm of it). Ring speed: the
horizontal chord of the ring's mocap position over the 1 s before the time given.
"""
import math
import os
import re

import numpy as np

import common as C

PR = C.PR

EXTRA = {
    'handover_done': r'\[planner\] handover settle complete',
    'pretension_done': r'\[planner\] pretension done',
    'hold_over': r'\[dissipative\] reconfiguration hold over',
    'orbit_hold': r'\[planner\] hover hold [\d.]+ s before the orbit',
    'orbit_start': r'\[planner\] hover hold done: orbit starts',
    'pause': r'\[dissipative\] attach pause: the orbit slows to rest over ([\d.]+) s',
    'pause_over': r'\[dissipative\] attach pause over: the orbit speeds back up over ([\d.]+) s',
    'descent_held': r'Contact descent held: tip ([\d.]+) cm off the plate',
    'start_weld': r'\[dissipative\] ATTACH drone 3 \(start weld',
}


class Mission:
    def __init__(self, fid):
        self.F = F = C.flight(fid)
        self.t0 = F.t_takeoff
        self.ev = []           # (t, event, detail)
        self.x = {k: C.log_lines(F.day, F.name, rx) for k, rx in EXTRA.items()}
        for t, c in F.L['cmd']:
            self.ev.append((t, f'fleet {c}', 'fleet_manager'))
        for x in F.L['landed']:
            self.ev.append((x[0], 'Landed - disarming the fleet', 'fleet_manager'))
        for r in F.L['resize']:
            self.ev.append((r[0], f'FLEET RESIZED to n={r[1]}', f'plates (cw) [{r[4]}]'))
        for p in F.J['phases']:
            self.ev.append((p[0], f'his {p[1]} -> {p[2]}', p[3]))
        for c in F.J['contacts']:
            self.ev.append((c[0], f'CONTACT (try {c[1]})', f'xy {c[2]} mm, normal {c[3]} mm'))
        for p in F.J['proof_failed']:
            self.ev.append((p[0], f'PROOF FAILED (try {p[1]}, {p[2]})', f'drift {p[3]} mm, plate rise {p[4]} mm'))
        for p in F.J['proven']:
            self.ev.append((p[0], f'ATTACH: proven (try {p[1]})', f'drift {p[2]} mm'))
        for p in F.J['pick_failed']:
            self.ev.append((p[0], f'pickup attempt {p[1]} failed', p[2]))
        for k, v in self.x.items():
            for t, m in v:
                self.ev.append((t, k, m.group(0)))
        for u in F.L['unload']:
            self.ev.append((u[0], 'UNLOAD', f'drone {u[1]}'))
        for u in F.L['unloaded']:
            self.ev.append((u[0], 'UNLOADED', f'planned {u[2]} N, ring tilt {u[3]} deg'))
        for d in F.L['detach']:
            self.ev.append((d[0], 'DETACH (OCP resize)', f'drone {d[1]}'))
        for t in F.J['release']:
            self.ev.append((t, 'PARTNER RELEASE', ''))
        for f in F.L['fold']:
            self.ev.append((f[0], 'ATTACH drone 3 (OCP resize)', ''))
        for tb in F.L['takeback']:
            self.ev.append((tb[0], 'TAKE-BACK', ''))
        top = F.ring_up()
        if top:
            self.ev.append((top, 'ring at the top', 'set-point at the top, ring within 10 cm'))
        self.ev.sort(key=lambda e: e[0])
        G = F.G
        self.gt = G.t.to_numpy(float)
        self.gxy = G[['payload_x', 'payload_y']].to_numpy(float)
        self.gz = G.payload_z.to_numpy(float)

    def first(self, name, after=-math.inf, contains=None):
        for t, e, d in self.ev:
            if t > after and (e == name if contains is None else (name in e and contains in (e + ' ' + d))):
                return t
        return None

    def his(self, to, after=-math.inf, frm=None):
        for p in self.F.J['phases']:
            if p[0] > after and p[2] == to and (frm is None or p[1] == frm):
                return p[0]
        return None

    def rel(self, t):
        return None if t is None else round(t - self.t0, 2)

    def speed(self, t):
        if t is None:
            return None
        w = (self.gt >= t - 1.0) & (self.gt < t) & np.all(np.isfinite(self.gxy), 1)
        if w.sum() < 10:
            return None
        a, b = np.nonzero(w)[0][[0, -1]]
        return round(float(np.linalg.norm(self.gxy[b] - self.gxy[a]) / (self.gt[b] - self.gt[a])), 3)

    def quad4_by(self, t):
        F = self.F
        n = F.n_at(t)
        prel = F.J['release'][0] if F.J['release'] else None
        his_to = F.J['his_takeoff'][0] if F.J['his_takeoff'] else None
        if t >= (F.t_land or math.inf):
            return 'LAND (every drone)'
        if n == 4:
            return 'ours (carrier)'
        if prel is not None and t < prel:
            return 'ours (stepping out)'
        if his_to is not None and t >= his_to - 0.2:
            return 'his planner'
        return 'his stack, on its mat'

    def span(self, name, a, b, detail='', ring=''):
        return {'flight': self.F.name, 'registry_id': self.F.rid, 'phase': name, 'start_s': self.rel(a),
                'end_s': self.rel(b), 'duration_s': round(b - a, 2) if (a is not None and b is not None) else None,
                'n_ours': self.F.n_at(a) if a is not None else None,
                'quad4_by': self.quad4_by(a) if a is not None else '', 'ring': ring,
                'ring_speed_end_mps': self.speed(b), 'detail': detail}

    def heights(self, t):
        """The ring's and the quads' heights at t, from the flight's run.csv (tools/rig_to_run.py), as text."""
        import json
        import pandas as pd
        run = os.path.join(C.RIG, self.F.day, self.F.name + '_run')
        if not os.path.exists(os.path.join(run, 'logs', 'run.csv')):
            return ''
        t0 = json.load(open(os.path.join(run, 'manifest.json')))['t0_epoch']
        d = pd.read_csv(os.path.join(run, 'logs', 'run.csv'), usecols=['t', 'payload_z'] + [f'd{i}_z' for i in range(4)])
        k = int(np.argmin(np.abs(d.t.to_numpy() + t0 - t)))
        q = [d[f'd{i}_z'].iloc[k] for i in range(4)]
        return f' (ring z {d.payload_z.iloc[k]:.2f} m, quads z {min(q):.2f}-{max(q):.2f} m)'

    def contact_loop(self, start, end_t):
        """Spans from the re-attach's first descent: each descent to its contact, each proof, up to the proof that
        held. Returns (rows, t_proven)."""
        F = self.F
        rows = []
        pv = next((p for p in F.J['proven'] if p[0] > start), None)
        t_pv = pv[0] if pv else None
        cons = [c for c in F.J['contacts'] if start < c[0] <= (t_pv or end_t)]
        fails = [p for p in F.J['proof_failed'] if start < p[0] <= (t_pv or end_t)]
        cur = start
        for i, c in enumerate(cons):
            descs = [p for p in F.J['phases'] if cur <= p[0] < c[0] and p[2] == 'DESCEND_TO_ATTACHMENT']
            ends = [f'{p[1]} -> {p[2]}: {p[3]}' for p in F.J['phases'] if cur < p[0] < c[0]
                    and p[1] in ('DESCEND_TO_ATTACHMENT', 'ATTACH_READY') and p[2] == 'APPROACH_ABOVE_TARGET']
            held = [t for t, _ in self.x['descent_held'] if cur <= t < c[0]]
            name = 'descent to contact' if i == 0 else 're-contact (descent again)'
            det = f'{len(descs)} descent(s); CONTACT try {c[1]} xy {c[2]:.1f} mm, normal {c[3]:+.1f} mm'
            if held:
                det += f'; {len(held)} hold(s) at the 5 cm band'
            if ends:
                det += '; retried: ' + ' | '.join(ends)
            rows.append(self.span(name, cur, c[0], det))
            f = next((p for p in fails if p[0] > c[0]), None)
            nxt_c = cons[i + 1][0] if i + 1 < len(cons) else math.inf
            if f is not None and f[0] < nxt_c:
                rows.append(self.span(f'proof {c[1]} (failed)', c[0], f[0],
                                      f'PROOF FAILED ({f[2]}): drift {f[3]:.1f} mm, plate rise {f[4]:.1f} mm'))
                cur = f[0]
            elif t_pv is not None and t_pv < nxt_c:
                rows.append(self.span(f'proof {c[1]} (held)', c[0], t_pv,
                                      f'ATTACH: proven (try {pv[1]}), drift {pv[2]:.1f} mm'))
                cur = t_pv
        if not cons:
            descs = [p for p in F.J['phases'] if p[0] >= start and p[2] == 'DESCEND_TO_ATTACHMENT']
            rows.append(self.span('re-attach descents (no contact)', start, end_t, f'{len(descs)} descent(s)'))
        return rows, t_pv

    def tail(self, t_pv, ring_after):
        """Fold-in, hold, the hover / circle on four, LAND."""
        F = self.F
        rows = []
        fold = next((r[0] for r in F.L['resize'] if t_pv - 0.5 <= r[0] <= t_pv + 5.0 and int(r[1]) >= 4), None)
        hold = self.first('hold_over', after=fold) if fold else None
        rows.append(self.span('fold-in (proof -> FLEET RESIZED n=4)', t_pv, fold, 'our planner takes QUAD4 over'))
        po = self.first('pause_over', after=fold - 0.5) if fold else None
        det = 'the OCP resize; trajectory held 1.5 s'
        if po:
            det += f'; the orbit speeds back up over 3 s from +{po - self.t0:.2f} s'
        rows.append(self.span('fold-in hold (resize -> trajectory resumes)', fold, hold, det))
        rows.append(self.span(f'{ring_after} on four', hold, F.t_land, ''))
        dis = [t for t, c in F.L['cmd'] if c == 'DISARM' and F.t_land < t <= (F.t_landed or math.inf) + 0.01]
        how = (f'the operator DISARM at +{dis[0] - self.t0:.2f} s{self.heights(dis[0] - 0.2)}' if dis
               else 'the fleet disarmed by its own landed test')
        rows.append(self.span('LAND (command -> fleet landed and disarmed)', F.t_land, F.t_landed,
                              f'every drone lands; {how}', 'landing'))
        return rows


def fm9_5():
    M = Mission('2026-11-12/fm9_5')
    F = M.F
    top = F.ring_up()
    unload = F.L['unload'][0][0]
    rs3 = next(r[0] for r in F.L['resize'] if r[0] > unload and int(r[1]) == 3)
    prel = F.J['release'][0]
    his_to = F.J['his_takeoff'][0]
    zu = float(np.interp(unload, M.gt, F.G.payload_ref_z.to_numpy(float)))
    rows = [M.span('takeoff, creep, pretension and lift ramp (QUAD4 welded on plate 9)', M.t0, unload,
                   f"hand-over settle done +{M.first('handover_done') - M.t0:.2f} s, pretension done and the lift "
                   f"ramp starts +{M.first('pretension_done') - M.t0:.2f} s; the unload starts with the ring's "
                   f"set-point at {zu:.2f} m of 1.20", 'lift'),
            M.span('detach: unload over 3 s, our magnet OFF', unload, rs3,
                   f'UNLOAD -> UNLOADED (planned 0.48 N, ring tilt 3.1 deg) -> FLEET RESIZED n=3; the set-point '
                   f'reaches 1.20 m at +{top - M.t0:.2f} s', 'lift'),
            M.span('QUAD4 steps out', rs3, prel, 'DETACH (OCP resize, trajectory held 1.5 s) -> PARTNER RELEASE',
                   'hover')]
    # the pickup: his TAKEOFF, approach, then each try
    descs = [p for p in F.J['phases'] if p[2] == 'DESCEND_TO_PICKUP']
    attached = M.his('LIFT_OBJECT')
    rows.append(M.span('his takeoff and approach over the object', prel, descs[0][0],
                       f'his TAKEOFF +{his_to - M.t0:.2f} s', 'hover'))
    for i, d in enumerate(descs):
        nxt = descs[i + 1][0] if i + 1 < len(descs) else None
        fail = next((p for p in F.J['pick_failed'] if d[0] < p[0] < (nxt or math.inf)), None)
        if fail:
            back = M.his('APPROACH_ABOVE_PICKUP', after=fail[0])
            rows.append(M.span(f'pickup try {i + 1} (failed)', d[0], nxt,
                               f'failed +{fail[0] - M.t0:.2f} s: {fail[2]}; retry from +{back - M.t0:.2f} s', 'hover'))
        else:
            att = next(p for p in F.J['pick_attached'] if p[0] > d[0])
            rows.append(M.span(f'pickup try {i + 1} (attached)', d[0], attached,
                               f'object attached with xy error {100 * att[1]:.1f} cm', 'hover'))
    lifted = M.his('TRANSIT_TO_DROP_POINT', frm='LIFT_OBJECT')
    lift_m = F.J['lifted'][0][1] if F.J['lifted'] else None
    settle = M.his('SETTLE_ABOVE_DROP_POINT')
    t_drop = M.his('DROP_OBJECT')
    t_clear = M.his('CLEAR_DROP_ZONE', frm='DROP_OBJECT')
    rows += [M.span('object lift', attached, lifted, f'object lifted {lift_m} m', 'hover'),
             M.span('transit to the drop point', lifted, settle, '', 'hover'),
             M.span('settle and descent over the ring', settle, t_drop,
                    f"descent from +{M.his('DESCEND_TO_DROP_HEIGHT') - M.t0:.2f} s", 'hover'),
             M.span('drop (release dwell)', t_drop, t_clear, 'object measured away from the magnet', 'hover')]
    d1 = M.his('DESCEND_TO_ATTACHMENT', after=t_clear)
    rows.append(M.span('to the re-attach (clear, transit, approach, match)', t_clear, d1, '', 'hover'))
    loop, t_pv = M.contact_loop(d1, F.t_land)
    rows += loop
    rows += M.tail(t_pv, 'hover')
    for r in rows:
        r['ring'] = r.get('ring') or 'hover'
    return rows, M


def fm9hc(fid):
    M = Mission(fid)
    F = M.F
    top = F.ring_up()
    o_start = M.first('orbit_start')
    his_to = F.J['his_takeoff'][0]
    climbed = M.his('TRANSIT_TO_DROP_POINT')
    settle = M.his('SETTLE_ABOVE_DROP_POINT')
    t_drop = M.his('DROP_OBJECT')
    t_clear = M.his('CLEAR_DROP_ZONE', frm='DROP_OBJECT')
    p = M.x['pause'][0]
    t_pause, slow = p[0], float(p[1].group(1))
    d1 = M.his('DESCEND_TO_ATTACHMENT', after=t_clear)
    rows = [M.span('takeoff and lift on three', M.t0, top,
                   f'his TAKEOFF (QUAD4 off its mat holding the object) +{his_to - M.t0:.2f} s', 'lift'),
            M.span('hover hold 5 s, then the orbit starts', top, o_start, '', 'hover'),
            M.span('his takeoff and loaded climb (QUAD4 track)', his_to, climbed,
                   'overlaps the two rows above and below', 'lift / hover / circle'),
            M.span('transit to the moving drop point', climbed, settle,
                   f'the orbit starts +{o_start - M.t0:.2f} s', 'hover / circle'),
            M.span('settle and descent over the moving ring', settle, t_drop,
                   f"descent from +{M.his('DESCEND_TO_DROP_HEIGHT') - M.t0:.2f} s", 'circle'),
            M.span('drop (release dwell)', t_drop, t_clear, f'ring speed at the drop {M.speed(t_drop)} m/s',
                   'circle'),
            M.span('clear the drop zone', t_clear, t_pause, 'his TRANSIT_TO_REATTACH starts the attach pause',
                   'circle'),
            M.span(f'the orbit slows to rest ({slow:.0f} s)', t_pause, t_pause + slow, '', 'slowing'),
            M.span('approach and match over plate 9', t_pause + slow, d1, '', 'at rest')]
    loop, t_pv = M.contact_loop(d1, F.t_land)
    for r in loop:
        r['ring'] = 'at rest'
    rows += loop
    tl = M.tail(t_pv, 'circle')
    tl[0]['ring'] = tl[1]['ring'] = 'at rest'
    tl[2]['ring'] = 'circle'
    rows += tl
    return rows, M


def main():
    rows, M = fm9_5()
    head = ['flight', 'registry_id', 'phase', 'start_s', 'end_s', 'duration_s', 'n_ours', 'quad4_by', 'ring',
            'ring_speed_end_mps', 'detail']
    C.write_csv(os.path.join(C.HERE, 'full_mission_fm9_5.csv'), rows, head)
    evs = [{'flight': M.F.name, 't_s': round(t - M.t0, 3), 'epoch': f'{t:.3f}', 'event': e, 'detail': d,
            'n_ours': M.F.n_at(t)} for t, e, d in M.ev]
    mv = []
    for fid in ('2026-11-12/fm9hc_2', '2026-11-12/fm9hc_4', '2026-11-12/fm9hc_5'):
        r, Mh = fm9hc(fid)
        mv += r
        evs += [{'flight': Mh.F.name, 't_s': round(t - Mh.t0, 3), 'epoch': f'{t:.3f}', 'event': e, 'detail': d,
                 'n_ours': Mh.F.n_at(t)} for t, e, d in Mh.ev]
    C.write_csv(os.path.join(C.HERE, 'moving_half_fm9hc.csv'), mv, head)
    C.write_csv(os.path.join(C.HERE, 'mission_events.csv'), evs, ['flight', 't_s', 'epoch', 'event', 'n_ours',
                                                                    'detail'])
    print(f'wrote {len(rows)} + {len(mv)} phase rows, {len(evs)} events')


if __name__ == '__main__':
    main()

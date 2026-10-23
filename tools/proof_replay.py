#!/usr/bin/env python3
"""Replay a rig bag through the attach confirmation (drone_magnet/comotion_proof.py): every contact, proof and
failure the co-motion rules would have produced, with the numbers behind them.

    python3 tools/proof_replay.py results/rig/2026-10-14/t6h_4 [--weld 0.013 0.210 0.024] [--from T --to T]

Reads /magnet_tip_pose, the ring (/payload/motion_capture_state) and QUAD4's body (/drone_3/motion_capture_state).
The magnet counts as ON from the first sample (the ring span is ON over the whole rejoin). The rules run with the rig's
values (--contact-xy = the sheet's K, --taut-fallback = its P + 3 mm, as graph_attach passes them). Bags from 15 Oct on
carry his lift-done signal (/join_planner/proof_lift_complete) and his phase (/join_planner/phase): the lift is his,
and a phase outside the contact phases voids a proof, as in the manager. Older bags time the lift from the contact
(--lift-s, his 3 cm at 2 cm/s plus the start).
"""
import argparse
import glob
import math
import sqlite3
import sys

import numpy as np


def quat_rot(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def load(cur, tops, name, quat=False, t_from=None, t_to=None):
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    if name not in tops:
        return np.zeros((0, 8 if quat else 4))
    i, ty = tops[name]
    cls = get_message(ty)
    lo = int((t_from or 0) * 1e9)
    hi = int((t_to or 4e9) * 1e9)
    out = []
    for ts, d in cur.execute('select timestamp,data from messages where topic_id=? and timestamp between ? and ? '
                             'order by timestamp', (i, lo, hi)):
        m = deserialize_message(d, cls)
        p, q = m.pose.position, m.pose.orientation
        out.append((ts * 1e-9, p.x, p.y, p.z) + ((q.x, q.y, q.z, q.w) if quat else ()))
    return np.array(out) if out else np.zeros((0, 8 if quat else 4))


def load_values(cur, tops, name, t_from=None, t_to=None):
    """[(t, msg.data)] of a std_msgs topic, or [] when the bag does not have it."""
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    if name not in tops:
        return []
    i, ty = tops[name]
    cls = get_message(ty)
    lo, hi = int((t_from or 0) * 1e9), int((t_to or 4e9) * 1e9)
    return [(ts * 1e-9, deserialize_message(d, cls).data) for ts, d in cur.execute(
        'select timestamp,data from messages where topic_id=? and timestamp between ? and ? order by timestamp',
        (i, lo, hi))]


def _latest(series, t, default=None):
    """The value of a [(t, value)] series at time t."""
    k = np.searchsorted([s[0] for s in series], t, side='right') - 1 if series else -1
    return series[k][1] if k >= 0 else default


def _plate_frame(ring, weld, t):
    """(rotation, plate point) of the ring sample nearest t, or (None, None) when it is over 0.1 s away."""
    j = min(np.searchsorted(ring[:, 0], t), len(ring) - 1)
    if abs(ring[j, 0] - t) > 0.1:
        return None, None
    rot = quat_rot(ring[j, 4:8])
    return rot, ring[j, 1:4] + rot @ np.asarray(weld)


def _nearest(arr, t, tol=0.1):
    if not len(arr):
        return None
    k = min(np.searchsorted(arr[:, 0], t), len(arr) - 1)
    return arr[k, 1:4] if abs(arr[k, 0] - t) <= tol else None


def replay_full(bag, weld, lift_s, t_from=None, t_to=None, **cfg_kw):
    """(events, data): the rules on the bag as the manager runs them. The magnet follows our bagged /magnet/command
    (ON from the first sample in bags without it); his lift-done signal and phase when the bag has them, with the
    manager's 2.5 s lift fallback; older bags time the lift from the contact (lift_s)."""
    from drone_magnet.comotion_proof import ComotionConfig, ComotionProof
    from drone_magnet.magnet_attachment_manager import CONTACT_PHASES
    db = sorted(glob.glob(f'{bag}/*.db3'))
    if not db:
        raise SystemExit(f'no bag in {bag}')
    cur = sqlite3.connect(db[0])
    tops = {n: (i, ty) for i, n, ty in cur.execute('select id,name,type from topics')}
    data = {'tip': load(cur, tops, '/magnet_tip_pose', t_from=t_from, t_to=t_to),
            'ring': load(cur, tops, '/payload/motion_capture_state', quat=True, t_from=t_from, t_to=t_to),
            'body': load(cur, tops, '/drone_3/motion_capture_state', t_from=t_from, t_to=t_to),
            'lift': load_values(cur, tops, '/join_planner/proof_lift_complete', t_from, t_to),
            'phases': load_values(cur, tops, '/join_planner/phase', t_from, t_to),
            'magnet': load_values(cur, tops, '/magnet/command', t_from, t_to),
            'mux': load_values(cur, tops, '/drone_3/mux_state', t_from, t_to)}
    tip, ring, body = data['tip'], data['ring'], data['body']
    if not len(tip) or not len(ring):
        return [], data
    # with his signal in the bag, the signal only (the manager's wiring); older bags time the lift from the contact
    proof = ComotionProof(ComotionConfig(lift_fallback_s=0.0 if data['lift'] else lift_s, **cfg_kw))
    events, phase_was, mag_was = [], None, None
    for t, x, y, z in tip:
        word = str(_latest(data['magnet'], t, 'ON' if not data['magnet'] else 'OFF')).strip().upper()
        on = word in ('ON', '1', 'TRUE')
        if on != mag_was:
            proof.magnet(on)
            mag_was = on
        phase = _latest(data['phases'], t)
        if phase != phase_was and phase is not None and str(phase).strip().upper() not in CONTACT_PHASES:
            ev = proof.abort(f'aborted ({phase})', t)
            if ev:
                rec = proof.report(t)
                rec.update({k2: proof.last_fail.get(k2) for k2 in ('reason', 'drift_m', 'xy_m', 'normal_m')})
                events.append((t, ev, rec))
        phase_was = phase
        rot, plate = _plate_frame(ring, weld, t)
        if rot is None:
            continue
        ev = proof.step(t, (x, y, z), plate, body=_nearest(body, t), rotation=rot,
                        lift_done=bool(_latest(data['lift'], t, False))) or proof.tick(t)
        if ev:
            rec = proof.report(t)
            if ev == 'failed':
                rec.update({k2: proof.last_fail.get(k2) for k2 in ('reason', 'drift_m', 'xy_m', 'normal_m')})
            events.append((t, ev, rec))
    return events, data


def replay(bag, weld, lift_s, t_from=None, t_to=None, **cfg_kw):
    return replay_full(bag, weld, lift_s, t_from, t_to, **cfg_kw)[0]


CATCH_MM, CATCH_HOLD_S, LOSS_MM, LOSS_S, CLIMB_M = 10.0, 0.3, 20.0, 0.12, 0.03


def _track(data, weld, t0, t1):
    """[(t, tip in the plate frame, body-to-tip, plate z, body z)] between t0 and t1."""
    out = []
    for t, x, y, z in data['tip'][(data['tip'][:, 0] >= t0) & (data['tip'][:, 0] <= t1)]:
        rot, plate = _plate_frame(data['ring'], weld, t)
        b = _nearest(data['body'], t)
        if rot is None or b is None:
            continue
        tip = np.array([x, y, z])
        out.append((t, rot.T @ (tip - plate), float(np.linalg.norm(tip - b)), float(plate[2]), float(b[2])))
    return out


def _longest(times, flags):
    """Longest run (s) of consecutive True flags."""
    best, start = 0.0, None
    for t, f in zip(times, flags):
        start = (t if start is None else start) if f else None
        if start is not None:
            best = max(best, t - start)
    return best


def catch_report(events, data, weld, taut_margin=0.003):
    """The card's bars from the bag, independent of the rules: per contact, the tip's largest excursion from its
    contact point (plate frame) until 5 s after a take-over (else until the proof ended, at most 4 s), the time taut,
    the plate's rise, and a catch = within 10 mm of the contact point while taut for 0.3 s (bars 1, 2); per climb back
    after a descent, whether the plate came up: taut, the tip within 10 mm of its place on the plate, QUAD4 3 cm up,
    for 0.3 s (bar 3)."""
    lines = []
    ours = [t for t, v in data['mux'] if str(v).strip().lower() == 'ours']
    contacts = [(t, rec) for t, ev, rec in events if ev == 'contact']
    for n, (tc, rec) in enumerate(contacts):
        nxt = [t for t, ev, _ in events if t > tc and ev in ('failed', 'contact')]
        t_proven = next((t for t, ev, _ in events if ev == 'proven' and t >= tc and (not nxt or t <= nxt[0])), None)
        # the take-over that follows the proof (not a later take-back); an OFF of ours ends the attached span
        took = [t for t in ours if t_proven is not None and t_proven <= t <= t_proven + 2.0]
        t_end = ((took[0] if took else t_proven) + 5.0 if t_proven is not None
                 else min(nxt[0] if nxt else tc + 4.0, tc + 4.0))
        offs = [t for t, v in data['magnet'] if t > tc and str(v).strip().upper() == 'OFF']
        t_end = min(t_end, offs[0]) if offs else t_end
        tr = _track(data, weld, tc, t_end)
        if len(tr) < 5:
            lines.append(f'  contact {n + 1} (+{tc - contacts[0][0]:.2f} s): too few samples')
            continue
        L = rec.get('l_hang_m')
        rel0 = tr[0][1]
        exc = [1000.0 * float(np.linalg.norm(r[1] - rel0)) for r in tr]
        taut = [L is not None and r[2] >= L - taut_margin for r in tr]
        times = [r[0] for r in tr]
        held = _longest(times, [e <= CATCH_MM and k for e, k in zip(exc, taut)])
        lost = _longest(times, [e > LOSS_MM for e in exc])
        rise = 1000.0 * (max(r[3] for r in tr) - tr[0][3])
        catch = held >= CATCH_HOLD_S
        proven = t_proven is not None
        verdict = ('CATCH' if catch else 'no catch') + (', proven' if proven else ', not proven')
        if proven and (max(exc) > CATCH_MM or lost >= LOSS_S):
            verdict += ': BAR 1 FAIL (the tip left its contact point after the proof)'
        elif proven != catch:
            verdict += ': BAR 2 MISMATCH'
        lines.append(f'  contact {n + 1} (+{tc - contacts[0][0]:.2f} s, to {"take-over + 5 s" if took else "proof + 5 s" if proven else "the proof end"} '
                     f'+{t_end - tc:.1f} s): excursion max {max(exc):.1f} mm, taut {100 * np.mean(taut):.0f} %, '
                     f'held {held:.2f} s, plate rise {rise:+.1f} mm -> {verdict}')
    climbs = [(t, data['phases'][k - 1][1]) for k, (t, p) in enumerate(data['phases'])
              if k and str(p).upper() == 'APPROACH_ABOVE_TARGET'
              and str(data['phases'][k - 1][1]).upper() in ('ATTACH_READY', 'DESCEND_TO_ATTACHMENT', 'LIFT_PROOF')]
    L = next((rec.get('l_hang_m') for _, rec in contacts if rec.get('l_hang_m')), None)
    for tc, was in climbs:
        tr = _track(data, weld, tc, tc + 5.0)
        if len(tr) < 5:
            continue
        rel0, z0 = tr[0][1], tr[0][4]
        flags = [L is not None and r[2] >= L - taut_margin and 1000.0 * float(np.linalg.norm(r[1] - rel0)) <= CATCH_MM
                 and r[4] - z0 >= CLIMB_M for r in tr]
        up = _longest([r[0] for r in tr], flags)
        lines.append(f'  climb back after {was} (+{tc - (contacts[0][0] if contacts else tc):.2f} s): QUAD4 up '
                     f'{100 * (max(r[4] for r in tr) - z0):.1f} cm, plate up {1000 * (max(r[3] for r in tr) - tr[0][3]):+.1f} mm'
                     f' -> {"BAR 3 FAIL (the plate came up with QUAD4)" if up >= CATCH_HOLD_S else "ok"}')
    return lines


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('bag', nargs='+')
    ap.add_argument('--weld', nargs=3, type=float, default=(0.013, 0.210, 0.024),
                    help='the weld point in the ring frame (m): the tip marker at contact')
    ap.add_argument('--lift-s', type=float, default=1.8, help='s from contact to his lift done (no signal in old bags)')
    ap.add_argument('--contact-xy', type=float, default=0.024, help="the contact radius (m): the sheet's K")
    ap.add_argument('--taut-fallback', type=float, default=0.575,
                    help="the typed taut length if no free hang is seen (m): the sheet's P + 0.003; 0 = none")
    ap.add_argument('--no-motion', action='store_true', help="the twin's rule: no plate motion needed")
    ap.add_argument('--from', dest='t_from', type=float)
    ap.add_argument('--to', dest='t_to', type=float)
    ap.add_argument('--catch', action='store_true',
                    help="also the card's bars from the bag: each contact's excursion, taut time and plate rise, and "
                         'whether the plate came up on a climb back')
    a = ap.parse_args(argv)
    for bag in a.bag:
        events, data = replay_full(bag, a.weld, a.lift_s, a.t_from, a.t_to, contact_xy=a.contact_xy,
                                   taut_fallback=a.taut_fallback, require_motion=0.0 if a.no_motion else 1.0)
        print(f'== {bag}: {sum(e[1] == "contact" for e in events)} contact(s), '
              f'{sum(e[1] == "proven" for e in events)} proven, {sum(e[1] == "failed" for e in events)} failed')
        t0 = None
        for t, ev, rec in events:
            t0 = t0 or t
            extra = (f" reason {rec.get('reason')}" if ev == 'failed' else '')
            print(f'  +{t - t0:6.2f} s  {ev:8s} xy {_mm(rec.get("xy_m"))} normal {_mm(rec.get("normal_m"))} '
                  f'drift {_mm(rec.get("drift_m"))} body-to-tip {rec.get("body_to_tip_m")} '
                  f'l_hang {rec.get("l_hang_m")} ({rec.get("l_hang_source")}) comotion {_mm(rec.get("comotion_m"))}{extra}')
        if a.catch:
            print('\n'.join(catch_report(events, data, a.weld)) or '  no contact and no climb back')
    return 0


def _mm(v):
    return '   -  ' if v is None or not math.isfinite(v) else f'{1000 * v:6.1f}mm'


if __name__ == '__main__':
    sys.exit(main())

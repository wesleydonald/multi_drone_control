#!/usr/bin/env python3
"""An independent recomputation of paper_rig.py's metrics for two flights, straight from the raw files: the bag's sqlite
rows, the launch log's lines and the drone-0 tracker CSV. It shares no code with paper_rig.py (its own bag read, its
own quaternion arithmetic on the raw mocap samples, no 50 Hz grid), so a slip in either shows as a difference.

    source ~/ros2_humble/install/setup.bash && source ~/multi_drone_control/install/setup.bash
    nice -n 10 python3 results/analysis/paper_rig_2026-11/check_by_hand.py      # -> check_by_hand.txt

Flights: 2026-10-08/r303_4 (the unannounced release in hover: detection, resize, tilt, dip, set-point, settle) and
2026-11-05/sq8_2 (a planned detach, then the one re-attach caught: the attach timeline, descents, the fold-in).
"""
import csv
import glob
import math
import os
import re
import sqlite3

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
RIG = os.path.expanduser('~/multi_drone_control/results/rig')
OUT = []


def say(s=''):
    print(s)
    OUT.append(s)


def bag(folder, names):
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    out = {n: [] for n in names}
    for db in sorted(glob.glob(os.path.join(folder, '*.db3'))):
        c = sqlite3.connect(db)
        for i, n, ty in c.execute('select id,name,type from topics'):
            if n not in names:
                continue
            cls = get_message(ty)
            for ts, d in c.execute('select timestamp,data from messages where topic_id=? order by timestamp', (i,)):
                m = deserialize_message(d, cls)
                if hasattr(m, 'pose'):
                    p, q = m.pose.position, m.pose.orientation
                    out[n].append((ts * 1e-9, p.x, p.y, p.z, q.w, q.x, q.y, q.z))
                else:
                    out[n].append((ts * 1e-9, m.data))
    return out


def stamps(log, pattern):
    rx = re.compile(pattern)
    hits = []
    with open(log, errors='replace') as f:
        for ln in f:
            if rx.search(ln):
                hits.append((float(re.search(r'\[(\d{9,}\.\d+)\]', ln).group(1)), ln.strip()))
    return hits


def ring_arrays(raw):
    a = np.array([r for r in raw])
    t, p = a[:, 0], a[:, 1:4]
    w, x, y, z = a[:, 4], a[:, 5], a[:, 6], a[:, 7]
    # the body z axis in the world frame: third column of the rotation matrix
    zx, zy, zz = 2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)
    ang = np.degrees(np.arccos(np.clip(zz, -1, 1)))
    h = np.hypot(zx, zy)
    tv = np.stack([zx, zy], 1) * np.where(h > 1e-12, ang / np.maximum(h, 1e-12), 0)[:, None]
    return t, p, ang, tv


def tracker_ref(day, name):
    fn = sorted(glob.glob(os.path.join(RIG, day, name + '_logs/logs/tracker/drone0_*/log.csv')))[-1]
    rows = []
    with open(fn) as f:
        for r in csv.DictReader(f):
            try:
                rows.append((float(r['sim_time']), float(r['payload_ref_x']), float(r['payload_ref_y']),
                             float(r['payload_ref_z'])))
            except ValueError:
                pass
    return np.array(sorted(rows))


def ring_numbers(t, p, ang, tv, ref, te, t_pre0, t_end):
    """The X1 numbers by hand on the raw samples."""
    pre = (t >= t_pre0) & (t < te)
    post = (t >= te) & (t < t_end)
    aft = (t >= te) & (t <= min(te + 10.0, t_end))
    m = tv[pre].mean(0)
    sig = math.sqrt(np.mean(np.sum((tv[pre] - m) ** 2, 1)))
    dev = np.linalg.norm(tv - m, axis=1)
    k = np.nonzero(post)[0][np.argmax(dev[post])]
    band = max(2 * sig, 1.0)
    settle = None
    for i in np.nonzero((t >= t[k]) & (t + 1.0 <= t_end))[0]:
        j = (t >= t[i]) & (t < t[i] + 1.0)
        if np.all(dev[j] <= band):
            settle = t[i] - te
            break
    kr = np.searchsorted(ref[:, 0], t, side='right') - 1
    e = np.linalg.norm(p - ref[np.clip(kr, 0, None), 1:4], axis=1)
    last = (t >= t_end - 5.0) & (t < t_end)
    after_level = np.linalg.norm(tv[last].mean(0)) if (t_end - te >= 10) else None
    settle_new = None
    if after_level is not None:
        lv = tv[last].mean(0)
        dn = np.linalg.norm(tv - lv, axis=1)
        for i in np.nonzero((t >= te) & (t + 1.0 <= t_end))[0]:
            j = (t >= t[i]) & (t < t[i] + 1.0)
            if np.all(dn[j] <= band):
                settle_new = t[i] - te
                break
    return {'tilt_pre_deg': ang[pre].mean(), 'sigma_pre_deg': sig, 'swing_deg': dev[k], 't_swing_peak_s': t[k] - te,
            'tilt_peak_deg': ang[post].max(), 'settle_s': settle, 'settle_new_s': settle_new,
            'tilt_after_deg': after_level, 'dip_mm': 1000 * (p[pre, 2].mean() - p[aft, 2].min()),
            'sp_pre_rms_cm': 100 * math.sqrt(np.mean(e[pre] ** 2)), 'sp_rms_cm': 100 * math.sqrt(np.mean(e[aft] ** 2))}


def compare(name, event, mine):
    with open(os.path.join(HERE, 'events.csv'), newline='') as f:
        row = next((r for r in csv.DictReader(f) if r['flight'] == name and r['event'] == event), None)
    say(f'  {"metric":24s} {"by hand":>10s} {"paper_rig":>10s}')
    worst = 0.0
    for k, v in mine.items():
        pr = row.get(k, '') if row else ''
        hv = '-' if v is None else f'{v:.3f}'
        say(f'  {k:24s} {hv:>10s} {pr or "-":>10s}')
        if v is not None and pr not in ('', None):
            worst = max(worst, abs(v - float(pr)))
    return worst


def r303_4():
    day, name = '2026-10-08', 'r303_4'
    say(f'== {day}/{name}: the unannounced release in hover (0.55 m links)')
    log = os.path.join(RIG, day, name + '.log')
    B = bag(os.path.join(RIG, day, name), ['/drone_3/magnet', '/payload/motion_capture_state'])
    offs = [m for m in B['/drone_3/magnet'] if m[1] == 'OFF']
    t_off = offs[0][0]
    t_det = stamps(log, r'UNANNOUNCED DETACH: drone 4')[0][0]
    t_rs = stamps(log, r'FLEET RESIZED to n=3')[0][0]
    t_land = stamps(log, r"Fleet command received: 'LAND'")[0][0]
    t_to = stamps(log, r"Fleet command received: 'TAKEOFF'")[0][0]
    say(f'  the bag\'s first /drone_3/magnet OFF {t_off:.3f}; the planner\'s own OFF line '
        f'{stamps(log, "magnet OFF -> /drone_3/magnet")[0][0]:.3f} (0.51 s later: the operator\'s OFF came first)')
    t, p, ang, tv = ring_arrays(B['/payload/motion_capture_state'])
    ref = tracker_ref(day, name)
    mine = {'t_event_s': t_off - t_to, 'detect_delay_s': t_det - t_off, 'resize_s': t_rs - t_off}
    mine.update(ring_numbers(t, p, ang, tv, ref, t_off, t_off - 5.0, t_land))
    say(f'  ring samples {len(t)} ({len(t) / (t[-1] - t[0]):.0f} Hz); window OFF .. LAND = {t_land - t_off:.2f} s')
    return compare(name, 'detach (unannounced)', mine)


def sq8_2():
    day, name = '2026-11-05', 'sq8_2'
    say(f'== {day}/{name}: the planned detach, then the re-attach caught on plate 8 (the square)')
    log = os.path.join(RIG, day, name + '.log')
    B = bag(os.path.join(RIG, day, name), ['/payload/motion_capture_state', '/magnet_tip_pose'])
    t_to = stamps(log, r"Fleet command received: 'TAKEOFF'")[0][0]
    t_land = stamps(log, r"Fleet command received: 'LAND'")[0][0]
    t_unload = stamps(log, r'UNLOAD drone 4')[0][0]
    t_off = stamps(log, r'magnet_attachment_manager\]: DETACH: magnet command OFF')[0][0]
    t_rs3 = stamps(log, r'FLEET RESIZED to n=3')[0][0]
    t_prel = stamps(log, r'PARTNER RELEASE')[0][0]
    t_proven = stamps(log, r'ATTACH: proven')[0][0]
    t_fold = [s for s, _ in stamps(log, r'FLEET RESIZED to n=4') if s > t_proven][0]
    t_contact = [s for s, _ in stamps(log, r'CONTACT \(try') if s < t_proven][-1]
    desc = [s for s, _ in stamps(log, r'Planner phase: \w+ -> DESCEND_TO_ATTACHMENT') if t_prel < s < t_proven]
    first_desc = desc[0]
    t, p, ang, tv = ring_arrays(B['/payload/motion_capture_state'])
    ref = tracker_ref(day, name)
    # the planned detach: the window ends at his first descent; the 5 s before from the set-point's lift end
    zr = ref[:, 3]
    top = ref[np.nonzero((ref[:, 0] < t_off) & (np.abs(zr - zr[np.searchsorted(ref[:, 0], t_off) - 1]) > 0.005))[0][-1], 0]
    pre0 = max(t_off - 5.0, top)
    # the magnet marker leaving its plate: 2 cm in the ring frame from where it sat over the 1 s before
    tip = np.array([r[:4] for r in B['/magnet_tip_pose']])
    ring = np.array(B['/payload/motion_capture_state'])

    def ring_frame(ts, pts):
        k = np.clip(np.searchsorted(ring[:, 0], ts, side='right') - 1, 0, len(ring) - 1)
        w, x, y, z = ring[k, 4], ring[k, 5], ring[k, 6], ring[k, 7]
        R = np.stack([np.stack([1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)], -1),
                      np.stack([2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)], -1),
                      np.stack([2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)], -1)], 1)
        return np.einsum('nji,nj->ni', R, pts - ring[k, 1:4])
    w0 = (tip[:, 0] >= t_off - 1) & (tip[:, 0] < t_off)
    seat = np.median(ring_frame(tip[w0, 0], tip[w0, 1:4]), 0)
    w1 = (tip[:, 0] >= t_off) & (tip[:, 0] < t_off + 5)
    d = np.linalg.norm(ring_frame(tip[w1, 0], tip[w1, 1:4]) - seat, axis=1)
    t_sep = tip[w1][np.nonzero(d > 0.02)[0][0], 0]
    mine = {'t_event_s': t_off - t_to, 'unload_s': t_off - t_unload, 'resize_s': t_rs3 - t_off,
            'separation_s': t_sep - t_off}
    mine.update(ring_numbers(t, p, ang, tv, ref, t_off, pre0, first_desc))
    say(f'  planned detach: UNLOAD {t_unload - t_to:.2f} s, OFF {t_off - t_to:.2f} s after TAKEOFF; the 5 s before cut '
        f'to {t_off - pre0:.2f} s (the set-point last moved then); window to his first descent, {first_desc - t_off:.2f} s')
    worst = compare(name, 'detach (planned)', mine)
    say('  the attach:')
    mine = {'t_event_s': t_fold - t_to, 'start_to_proven_s': t_proven - t_prel, 'start_to_foldin_s': t_fold - t_prel,
            'contact_to_foldin_s': t_fold - t_contact, 'descents': float(len(desc))}
    mine.update(ring_numbers(t, p, ang, tv, ref, t_fold, t_fold - 5.0, t_land))
    worst = max(worst, compare(name, 'attach', mine))
    # the miss per descent against feed F052's table (the original joins.csv, untouched)
    orig = os.path.expanduser('~/multi_drone_control/results/analysis/rig_joins_2026-11-04_05/joins.csv')
    with open(orig, newline='') as f:
        o = [r for r in csv.DictReader(f) if r['flight'] == name and r['day'] == day and r['kind'] == 're-attach']
    with open(os.path.join(HERE, 'joins', 'joins.csv'), newline='') as f:
        n = [r for r in csv.DictReader(f) if r['flight'] == name and r['day'] == day and r['kind'] == 're-attach']
    say('  descents (F052\'s joins.csv | re-run): ' + '; '.join(
        f"#{a['n']} {a['eval']} {a['tip_xy_cm']} cm {a['outcome']} | {b['eval']} {b['tip_xy_cm']} cm {b['outcome']}"
        for a, b in zip(o, n)))
    return worst


def main():
    w1 = r303_4()
    say()
    w2 = sq8_2()
    say()
    say(f'largest difference by hand vs paper_rig: r303_4 {w1:.3f}, sq8_2 {w2:.3f} (in each metric\'s unit)')
    with open(os.path.join(HERE, 'check_by_hand.txt'), 'w') as f:
        f.write('\n'.join(OUT) + '\n')


if __name__ == '__main__':
    main()

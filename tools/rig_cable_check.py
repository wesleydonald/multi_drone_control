#!/usr/bin/env python3
"""tools/rig_cable_check.py -- one line per rig flight on QUAD4's cable and the ring's level (11 Nov 2026).

    source ~/ros2_humble/install/setup.bash && source install/setup.bash
    python3 tools/rig_cable_check.py nrtf nrtf_2                 # today's flights by name
    python3 tools/rig_cable_check.py 2026-11-11/fm9c_3           # another day's

While QUAD4 is on the ring (pretension to DETACH, and the fold-in to LAND after a re-attach) it reads from the bag:
- the cable: body 14 to the magnet marker (body 24). Taut = that distance's 98th percentile in the window; slack =
  more than 4 mm shorter. Share of the window slack, the deepest slack, and the main frequency of the length (0.4-10 Hz);
- the ring's level: the rim's height at plate 9 and at plate 3 against the ring's centre (mm, + = high);
- frozen poses: the longest run of bit-identical QUAD4 poses from TAKEOFF to the disarm (a lost body repeats).
Background: docs/analysis/quad4_bounce_2026-11-11.md (deep slack 22-36 mm in the three bouncy flights, 5-7 mm in the calm).
"""
import argparse
import datetime
import os
import re
import sqlite3
import sys

import numpy as np

RIG = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'results', 'rig')
SLACK_M = 0.004


def _rot(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def cable_stats(t, length):
    """(taut m, slack share, deepest slack m, main frequency Hz) of a cable-length series."""
    taut = float(np.percentile(length, 98))
    slack = taut - length
    dt = float(np.median(np.diff(t)))
    n = max(3, int(round(1.0 / dt)))
    hp = (length - np.convolve(length, np.ones(n) / n, mode='same'))[n:-n]
    freq = float('nan')
    if len(hp) > 4 * n:
        f = np.fft.rfftfreq(len(hp), dt)
        s = np.abs(np.fft.rfft(hp))
        band = (f > 0.4) & (f < 10.0)
        freq = float(f[np.argmax(np.where(band, s, 0.0))])
    return taut, float(np.mean(slack > SLACK_M)), float(slack.max()), freq


def longest_repeat(t, raw):
    """Longest run (s) of identical consecutive entries in raw (bytes per sample)."""
    best = cur = 0.0
    start = None
    for i in range(1, len(raw)):
        if raw[i] == raw[i - 1]:
            start = t[i - 1] if start is None else start
            cur = t[i] - start
            best = max(best, cur)
        else:
            start = None
    return best


def check(day, name):
    from rclpy.serialization import deserialize_message
    from geometry_msgs.msg import PoseStamped
    from interfaces.msg import MotionCaptureState
    d = os.path.join(RIG, day)
    log = open(os.path.join(d, name + '.log'), errors='ignore').read()

    def times(pat):
        return [float(m) for m in re.findall(pat, log)]
    to = times(r"\[(\d+\.\d+)\] \[fleet_manager\]: Fleet command received: 'TAKEOFF'")
    pre = times(r"\[(\d+\.\d+)\] \[dissipative_planner\]: \[planner\] pretension done")
    n3 = times(r"\[(\d+\.\d+)\] \[dissipative_planner\]: \[planner\] FLEET RESIZED to n=3")
    n4 = times(r"\[(\d+\.\d+)\] \[dissipative_planner\]: \[planner\] FLEET RESIZED to n=4")
    land = times(r"\[(\d+\.\d+)\] \[fleet_manager\]: Fleet command received: 'LAND'")
    end = times(r"\[(\d+\.\d+)\] \[fleet_manager\]: (?:Disarm complete|Landed - disarming)")
    if not to:
        return f'{name}: no TAKEOFF'
    db = sqlite3.connect(os.path.join(d, name, name + '_0.db3'))
    tid = dict(db.execute('select name, id from topics'))

    def rows(topic, a, b):
        return db.execute('select timestamp, data from messages where topic_id=? and timestamp between ? and ? order by '
                          'timestamp', (tid[topic], int(a * 1e9), int(b * 1e9))).fetchall()

    def window(label, a, b):
        if b - a < 2.0 or '/magnet_tip_pose' not in tid:
            return None
        tip = rows('/magnet_tip_pose', a, b)
        q4 = rows('/drone_3/motion_capture_state', a, b)
        ring = rows('/payload/motion_capture_state', a, b)
        if min(len(tip), len(q4), len(ring)) < 50:
            return None
        tt = np.array([r[0] for r in tip]) * 1e-9
        P = np.array([[m.pose.position.x, m.pose.position.y, m.pose.position.z]
                      for m in (deserialize_message(r[1], PoseStamped) for r in tip)])
        qt = np.array([r[0] for r in q4]) * 1e-9
        Q = np.array([[m.pose.position.x, m.pose.position.y, m.pose.position.z]
                      for m in (deserialize_message(r[1], MotionCaptureState) for r in q4)])
        length = np.linalg.norm(Q[np.clip(np.searchsorted(qt, tt), 0, len(qt) - 1)] - P, axis=1)
        taut, share, deep, freq = cable_stats(tt, length)
        rim9, rim3 = [], []
        for r in ring[::5]:
            o = deserialize_message(r[1], MotionCaptureState).pose.orientation
            R = _rot((o.w, o.x, o.y, o.z))
            rim9.append(1000.0 * (R @ np.array([0.0, 0.225, 0.0]))[2])      # plate 9: the ring's +y
            rim3.append(1000.0 * (R @ np.array([0.0, -0.225, 0.0]))[2])
        return (f'{label} {b - a:.0f} s: cable slack {100 * share:.0f} %, deepest {1000 * deep:.0f} mm'
                + (f', {freq:.1f} Hz' if deep > 0.010 and np.isfinite(freq) else '')
                + f'; plate 9 {np.mean(rim9):+.0f} mm, plate 3 {np.mean(rim3):+.0f} mm')

    out = []
    if pre and n3:
        out.append(window('on four', pre[0] + 1.0, n3[0] - 0.3))
    later = [x for x in n4 if n3 and x > n3[0]]
    if later and land and land[-1] > later[0]:
        out.append(window('re-attached', later[0] + 1.0, land[-1] - 0.3))
    q4 = rows('/drone_3/motion_capture_state', to[0], (end[0] if end else to[0] + 600.0))
    frozen = longest_repeat(np.array([r[0] for r in q4]) * 1e-9, [r[1][-104:-48] for r in q4]) if len(q4) > 10 else 0.0
    out.append(f'QUAD4 pose frozen at most {frozen:.2f} s' + (' <-- LOST' if frozen > 0.5 else ''))
    return f'{name}: ' + ' | '.join(x for x in out if x)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('flights', nargs='+', help='<name> (today) or <YYYY-MM-DD>/<name>')
    a = ap.parse_args()
    today = datetime.date.today().isoformat()
    for f in a.flights:
        day, name = f.split('/') if '/' in f else (today, f)
        try:
            print(check(day, name))
        except Exception as e:                          # one unreadable flight must not hide the others
            print(f'{name}: not read ({type(e).__name__}: {e})')


if __name__ == '__main__':
    sys.exit(main())

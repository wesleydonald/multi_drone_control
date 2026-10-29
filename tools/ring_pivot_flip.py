#!/usr/bin/env python3
"""tools/ring_pivot_flip.py -- find the ring's true centre with a 180 deg turn (the 21-22 Oct pivot procedure).

    python3 tools/ring_pivot_flip.py read A        # the ring resting, tape marks on the floor at its rim
    python3 tools/ring_pivot_flip.py read B        # the ring turned 180 deg about its centre, back in the same marks
    python3 tools/ring_pivot_flip.py compute       # the pivot's offset from the centre, in the ring's axes

Reads /payload/motion_capture_state (body 8) for 3 s per reading and keeps them in /tmp/ring_pivot_flip.json.
A pivot that sits d off the ring's centre moves 2 d when the ring turns 180 deg about its centre, so the centre is
the midpoint and the offset is half the difference, expressed in the ring frame of reading A. Motive's rigid-body
axes (22 Oct probes): Motive X = ring +x (towards plate 0), Motive Z = ring -y (towards plate 3), Motive Y = up.
"""
import json
import math
import sys
import time

import numpy as np

from tip_calibration import quat_to_rot

STORE = '/tmp/ring_pivot_flip.json'


def read(seconds=3.0):
    import rclpy
    from interfaces.msg import MotionCaptureState
    rclpy.init()
    node = rclpy.create_node('ring_pivot_flip')
    got = []

    def cb(m):
        p, q = m.pose.position, m.pose.orientation
        got.append([p.x, p.y, p.z, q.w, q.x, q.y, q.z])

    node.create_subscription(MotionCaptureState, '/payload/motion_capture_state', cb, 10)
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    node.destroy_node()
    rclpy.shutdown()
    if not got:
        raise SystemExit('no ring pose: is T1 up with the payload as body 8?')
    g = np.array(got)
    q = g[-1, 3:]
    yaw = math.degrees(math.atan2(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] ** 2 + q[3] ** 2)))
    return {'pos': g[:, :3].mean(axis=0).tolist(), 'quat': q.tolist(), 'yaw_deg': yaw, 'n': len(g),
            'sd_mm': (g[:, :3].std(axis=0) * 1000).tolist()}


def main(argv):
    if argv[:1] == ['read'] and len(argv) == 2:
        try:
            store = json.load(open(STORE))
        except (OSError, ValueError):
            store = {}
        r = read()
        store[argv[1]] = r
        json.dump(store, open(STORE, 'w'))
        x, y, z = r['pos']
        print(f"{argv[1]}: pivot x {x * 100:+.2f} y {y * 100:+.2f} z {z * 100:.2f} cm, yaw {r['yaw_deg']:+.1f} deg "
              f"({r['n']} frames, sd {max(r['sd_mm']):.1f} mm)")
        return 0
    if argv[:1] == ['compute']:
        s = json.load(open(STORE))
        a, b = np.array(s['A']['pos']), np.array(s['B']['pos'])
        turn = (s['B']['yaw_deg'] - s['A']['yaw_deg'] + 180.0) % 360.0 - 180.0
        centre = (a + b) / 2.0
        off_world = a - centre
        off_ring = quat_to_rot(*s['A']['quat']).T @ off_world
        print(f"turn between the readings {turn:+.1f} deg (want +-180); centre x {centre[0] * 100:+.2f} y "
              f"{centre[1] * 100:+.2f} cm; z A {a[2] * 100:.2f} B {b[2] * 100:.2f} cm")
        print(f"pivot offset from the centre, ring axes: x {off_ring[0] * 1000:+.1f} mm (towards plate 0), "
              f"y {off_ring[1] * 1000:+.1f} mm (towards plate 9), {np.hypot(*off_ring[:2]) * 1000:.1f} mm in all")
        print(f"to centre it, move the pivot in Motive by X {-off_ring[0] * 1000:+.1f} mm, Z {off_ring[1] * 1000:+.1f} mm "
              f"(Motive Z = ring -y); for a 1 cm rest height also Y {(0.010 - a[2]) * 1000:+.1f} mm")
        return 0
    print(__doc__)
    return 1


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))

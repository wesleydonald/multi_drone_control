#!/usr/bin/env python3
"""tools/payload_geometry.py -- props off, a new payload's geometry from motion capture (Wesley 28 Oct, 1.15 kg ring).

    python3 tools/payload_geometry.py [--seconds 5] [--drones 4]

With T1 up (the payload as body 8 on /payload/motion_capture_state, each magnet marker on
/drone_<i>/magnet_tip_state) and every magnet face resting on its plate, prints the ring's rest height (the pivot's
z: payload_rest_z) and each magnet marker in the ring frame: its radius and azimuth (counter-clockwise from the
ring's +x, and the plate it sits on counted clockwise) and its height above the pivot (the marker on the plate face:
attach_z, or weld_point_z for a newcomer), and each drone's body-to-marker distance (held up with its cable taut:
the newcomer's L).
"""
import argparse
import math
import sys
import time

import numpy as np

from tip_calibration import in_ring_frame


def main(argv):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--seconds', type=float, default=5.0)
    ap.add_argument('--drones', type=int, default=4)
    a = ap.parse_args(argv)
    import rclpy
    from interfaces.msg import MotionCaptureState
    rclpy.init()
    node = rclpy.create_node('payload_geometry')
    ring, tips = [], {i: [] for i in range(a.drones)}
    bodies, reach = {}, {i: [] for i in range(a.drones)}
    last = {}

    def on_ring(m):
        p, q = m.pose.position, m.pose.orientation
        last['ring'] = (np.array([p.x, p.y, p.z]), (q.w, q.x, q.y, q.z))
        ring.append(last['ring'])

    def on_tip(i):
        def cb(m):
            p = m.pose.position
            tip = np.array([p.x, p.y, p.z])
            if 'ring' in last:
                tips[i].append(in_ring_frame(tip, *last['ring']))
            if i in bodies:
                reach[i].append(float(np.linalg.norm(tip - bodies[i])))
        return cb

    def on_body(i):
        def cb(m):
            p = m.pose.position
            bodies[i] = np.array([p.x, p.y, p.z])
        return cb

    node.create_subscription(MotionCaptureState, '/payload/motion_capture_state', on_ring, 10)
    for i in range(a.drones):
        node.create_subscription(MotionCaptureState, f'/drone_{i}/magnet_tip_state', on_tip(i), 10)
        node.create_subscription(MotionCaptureState, f'/drone_{i}/motion_capture_state', on_body(i), 10)
    end = time.monotonic() + a.seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    node.destroy_node()
    rclpy.shutdown()
    if not ring:
        print('no ring pose: is T1 up with the payload as body 8?')
        return 1
    z = np.array([r[0][2] for r in ring])
    print(f'ring pivot at rest: z {z.mean():.4f} m (sd {z.std() * 1000:.1f} mm, {len(z)} frames) -> payload_rest_z')
    radii, heights = [], []
    for i, s in tips.items():
        if not s:
            print(f'drone_{i}: no magnet marker frames')
            continue
        s = np.array(s)
        x, y, h = s.mean(axis=0)
        r = math.hypot(x, y)
        az = math.degrees(math.atan2(y, x)) % 360.0
        plate = round(((360.0 - az) % 360.0) / 30.0) % 12
        radii.append(r)
        heights.append(h)
        print(f'drone_{i}: marker at x {x:+.4f} y {y:+.4f} -> radius {r:.4f} m, {az:5.1f} deg ccw (plate {plate}), '
              f'{h * 100:+.2f} cm above the pivot (sd {s.std(axis=0).max() * 1000:.1f} mm, {len(s)} frames)'
              + (f'; body to marker {np.mean(reach[i]):.3f} m' if reach[i] else ''))
    if radii:
        print(f'mean radius {np.mean(radii):.4f} m (attach_radius), mean height {np.mean(heights) * 100:+.2f} cm '
              f'(the magnet marker on the plate face; attach_z is the plate face, the marker sits on the magnet)')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))

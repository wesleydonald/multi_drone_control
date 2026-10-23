#!/usr/bin/env python3
"""tools/pickup_geometry.py -- the two measured values m1_irl_pickup.launch.py needs (Wesley 21 Oct).

    ros2 launch tejen_mission m1_irl_pickup.launch.py measure:=true magnet_body_id:=<id>   # T1, props off
    python3 tools/pickup_geometry.py [--seconds 5]                                           # T2

Reads his IRL mocap publisher's /motion_capture_state (the quad), /magnet_tip_pose (its magnet marker) and
/irl/pickup_object/pose (the object) for --seconds, then prints, from the means:
  * magnet_drop_below_quad = quad z - marker z     (hold the quad up with the cable hanging taut and still)
  * contact_height         = marker z - object z   (the magnet face resting flat on the object, magnet ON)
and the marker's xy offset from the object, the sample counts and the spread. Run it once per measurement.
"""
import argparse
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseArray, PoseStamped
from interfaces.msg import MotionCaptureState


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--seconds', type=float, default=5.0)
    a = ap.parse_args()
    rclpy.init()
    node = rclpy.create_node('pickup_geometry')
    got = {'quad': [], 'tip': [], 'object': []}
    pos = lambda p: [p.position.x, p.position.y, p.position.z]  # noqa: E731
    node.create_subscription(MotionCaptureState, '/motion_capture_state', lambda m: got['quad'].append(pos(m.pose)), 50)
    node.create_subscription(PoseStamped, '/magnet_tip_pose', lambda m: got['tip'].append(pos(m.pose)), 50)
    node.create_subscription(PoseArray, '/irl/pickup_object/pose',
                             lambda m: got['object'].append(pos(m.poses[0])) if m.poses else None, 50)
    end = time.time() + a.seconds
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    mean = {k: np.mean(v, axis=0) if v else None for k, v in got.items()}
    for k, v in got.items():
        sd = np.std(v, axis=0) * 1000 if v else None
        print(f'{k:7s} {len(v):4d} samples' + (f', mean {np.round(mean[k], 4)}, sd {np.round(sd, 1)} mm' if v else
                                               ' -- NONE (check the Motive id and that it is tracked)'))
    if mean['quad'] is not None and mean['tip'] is not None:
        print(f'\nmagnet_drop_below_quad (cable hanging taut): {mean["quad"][2] - mean["tip"][2]:.4f} m'
              f'   (xy offset {np.linalg.norm(mean["quad"][:2] - mean["tip"][:2]) * 100:.1f} cm: near 0 when hanging)')
    if mean['tip'] is not None and mean['object'] is not None:
        print(f'contact_height (magnet face flat on the object): {mean["tip"][2] - mean["object"][2]:.4f} m'
              f'   (marker {np.linalg.norm(mean["tip"][:2] - mean["object"][:2]) * 100:.1f} cm off the object centre)')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

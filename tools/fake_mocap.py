#!/usr/bin/env python3
"""
fake_mocap.py -- static MotionCaptureState feed for desk-testing the HARDWARE launches.

The real launches block until mocap streams (each tracker waits for its first pose
while holding the acados compile lock, so without mocap only one tracker ever comes
up). This publishes fixed poses for num_drones drones (+ one newcomer with --attach)
and the payload at 50 Hz, so real_*_launch.py can be brought up whole, parameters read
back (tools/preflight.py), and topics inspected, with no rig. Nothing here flies: the
poses never move, so ARM/TAKEOFF must not be sent against it.

    tools/fake_mocap.py --num-drones 3 --attach          # 3 tethered + drone 3
"""
import argparse
import math

import rclpy
from rclpy.node import Node
from interfaces.msg import MotionCaptureState


class FakeMocap(Node):
    def __init__(self, n, attach, cable_len, radius, elev_deg, load_z):
        super().__init__('fake_mocap')
        self.poses = {}
        el = math.radians(elev_deg)
        for i in range(n):
            az = 2 * math.pi * i / n
            r = radius + cable_len * math.cos(el)
            self.poses[f'/drone_{i}/motion_capture_state'] = (
                r * math.cos(az), r * math.sin(az), load_z + cable_len * math.sin(el))
        if attach:
            self.poses[f'/drone_{n}/motion_capture_state'] = (0.0, -1.2, 0.12)
        self.poses['/payload/motion_capture_state'] = (0.0, 0.0, load_z)
        self.pubs = {t: self.create_publisher(MotionCaptureState, t, 10) for t in self.poses}
        self.create_timer(0.02, self._tick)
        self.get_logger().info(f'fake mocap: {len(self.poses)} bodies at 50 Hz (static)')

    def _tick(self):
        now = self.get_clock().now().to_msg()
        for t, (x, y, z) in self.poses.items():
            m = MotionCaptureState()
            m.header.stamp = now
            m.header.frame_id = 'map'
            m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, z
            m.pose.orientation.w = 1.0
            self.pubs[t].publish(m)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--num-drones', type=int, default=3)
    ap.add_argument('--attach', action='store_true', help='also publish the newcomer (id num_drones)')
    ap.add_argument('--cable-len', type=float, default=0.5)
    ap.add_argument('--attach-radius', type=float, default=0.25)
    ap.add_argument('--elev-deg', type=float, default=45.0)
    ap.add_argument('--load-z', type=float, default=0.05)
    a = ap.parse_args()
    rclpy.init()
    node = FakeMocap(a.num_drones, a.attach, a.cable_len, a.attach_radius, a.elev_deg, a.load_z)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Read-only props-off geometry gate for the opt-in M1 ground-ring trial."""

from collections import deque
import math
from pathlib import Path
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src/tejen_mission'))
from tejen_mission.m1_ground_ring import minimum_face_gap


def params(path, node):
    return yaml.safe_load(path.read_text(encoding='utf-8'))[node]['ros__parameters']


def yaw_tilt(quat):
    x, y, z, w = np.asarray(quat, dtype=float)
    norm = math.sqrt(x*x + y*y + z*z + w*w)
    if norm < 1e-9 or not math.isfinite(norm):
        raise ValueError('invalid ring orientation')
    x, y, z, w = x/norm, y/norm, z/norm, w/norm
    tilt = math.degrees(math.acos(max(-1., min(1., 1. - 2.*(x*x+y*y)))))
    yaw = math.atan2(2.*(w*z+x*y), 1.-2.*(y*y+z*z))
    return yaw, tilt


class Check(Node):
    def __init__(self):
        super().__init__('m1_ground_ring_props_off_check')
        self.ring = deque(maxlen=300)
        self.create_subscription(PoseStamped, '/irl/ring/mocap_pose', self.on_ring, 10)

    def on_ring(self, msg):
        p, q = msg.pose.position, msg.pose.orientation
        try:
            yaw, tilt = yaw_tilt((q.x, q.y, q.z, q.w))
            xyz = np.array([p.x, p.y, p.z], dtype=float)
            if np.all(np.isfinite(xyz)):
                self.ring.append((time.monotonic(), xyz, yaw, tilt))
        except ValueError:
            pass


def main():
    base = params(ROOT / 'src/tejen_mission/config/irl_commissioning.yaml',
                  'online_join_planner')
    ground = params(ROOT / 'src/tejen_mission/config/m1_ground_ring.yaml',
                    'online_join_planner')
    rclpy.init()
    node = Check()
    try:
        deadline = time.monotonic() + 4.0
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        now = time.monotonic()
        ring = [item for item in node.ring if now-item[0] <= 2.]
        if len(ring) < 20 or now-ring[-1][0] > .5:
            raise ValueError('STOP: ring ID8 pose missing/stale')
        points = np.array([item[1] for item in ring])
        centre = np.mean(points, axis=0)
        drift = float(np.max(np.linalg.norm(points-centre, axis=1)))
        yaw = ring[-1][2]
        max_tilt = max(item[3] for item in ring)
        yaw_drift = max(abs(math.degrees(math.atan2(
            math.sin(item[2]-yaw), math.cos(item[2]-yaw)))) for item in ring)
        gap = minimum_face_gap(float(centre[2]), float(ground['min_reference_z']),
                               float(base['magnet_drop_below_quad']),
                               float(ground['ground_ring_marker_to_face_m']))
        print(f'ring centre={centre.round(3)}, plate0 yaw={math.degrees(yaw):.1f} deg, '
              f'tilt_max={max_tilt:.1f} deg, placement drift={drift:.3f} m')
        print(f'min achievable face gap={gap:.4f} m')
        if max_tilt > 5. or drift > .01 or yaw_drift > 2.:
            raise ValueError('STOP: ring is tilted or unstable; place/recalibrate before ARM')
        if gap > float(ground['ground_ring_max_min_face_gap_m']):
            raise ValueError(
                f"STOP: nominal magnet face cannot reach the plate at the "
                f"{float(ground['min_reference_z']):.2f} m body-reference floor"
            )
        print('PASS: props-off placement geometry. Attachment remains UNVERIFIED; inspect hardware.')
        return 0
    except ValueError as exc:
        print(exc, file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())

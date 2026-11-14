"""Open-loop replay driver for Te-Jen's transfer backend (nothing flies).

A virtual QUAD4 starts at QUAD4's recorded pose at the start of TRANSIT_TO_DROP_POINT, holds still until the backend
has its prepared plan, is granted authority, then follows the backend's own reference exactly (perfect tracking).
The world (ring pose and trajectory state -> ring_bridge -> the carriers' shared trajectories, plates, ring pose) comes
from the 12 Nov recording through ros2 bag play. Publishes the mission's inputs the recording lacks: the body target
over the drop point (ring + 0.15 + 0.38 + 1.034 m, online_join_planner's c1f2 transfer target), its velocity (the
ring's measured twist), enable, authority and object_attached.
params: x0 y0 z0, log, capture_xy 0.12, capture_z 0.15, capture_hold 0.5, run_s"""
import math

import rclpy
from geometry_msgs.msg import PoseArray, PoseStamped, TwistStamped
from interfaces.msg import MotionCaptureState
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from trajectory_msgs.msg import MultiDOFJointTrajectory

TARGET_DZ = 0.15 + 0.38 + 1.034


def seg_dist(p1, q1, p2, q2):
    """Distance between segments p1-q1 and p2-q2 (3D)."""
    import numpy as np
    p1, q1, p2, q2 = (np.asarray(v, float) for v in (p1, q1, p2, q2))
    d1, d2, r = q1 - p1, q2 - p2, p1 - p2
    a, e, f = d1 @ d1, d2 @ d2, d2 @ r
    c, b = d1 @ r, d1 @ d2
    den = a * e - b * b
    s = min(max((b * f - c * e) / den, 0.0), 1.0) if den > 1e-12 else 0.0
    t = (b * s + f) / e
    if t < 0.0:
        t, s = 0.0, min(max(-c / a, 0.0), 1.0)
    elif t > 1.0:
        t, s = 1.0, min(max((b - c) / a, 0.0), 1.0)
    return float(np.linalg.norm((p1 + d1 * s) - (p2 + d2 * t)))


class Driver(Node):
    def __init__(self):
        super().__init__('replay_driver')
        g = lambda n, d: self.declare_parameter(n, d).value
        self.pos = [g('x0', 0.0), g('y0', 0.0), g('z0', 2.7)]
        self.vel = [0.0, 0.0, 0.0]
        self.cap_xy, self.cap_z, self.cap_hold = g('capture_xy', 0.12), g('capture_z', 0.15), g('capture_hold', 0.5)
        self.run_s = g('run_s', 60.0)
        self.latch = g('latch', False)
        self.latched = None
        self.relay_hz = g('relay_hz', 0.0)
        self.relay_msg = None
        if self.relay_hz > 0.0:
            # diagnostic: ring_bridge's 20 Hz ring pose relayed to his backend at relay_hz (fewer world versions)
            self.create_subscription(PoseArray, '/rb/pose_array', lambda m: setattr(self, 'relay_msg', m), 10)
            self.relay_pub = self.create_publisher(PoseArray, '/fake_payload/pose_array', 10)
            self.create_timer(1.0 / self.relay_hz, self._relay_tick)
        self.log = open(g('log', 'driver.csv'), 'w')
        self.log.write('t,auth,x,y,z,vx,vy,vz,tx,ty,tz,ring_x,ring_y,ring_z,dxy,dz,captured,min_body_m,min_cable_tether_m,'
                       'status\n')
        self.ring = None
        self.ring_v = (0.0, 0.0, 0.0)
        self.carriers = {}
        self.ref = None
        self.status = ''
        self.auth_ack = False
        self.auth_t = None
        self.cap_since = None
        self.captured_t = None
        self.t_start = None
        self.events = []
        self.create_subscription(MotionCaptureState, '/payload/motion_capture_state', self._ring_cb, 20)
        for i in range(3):
            self.create_subscription(MotionCaptureState, f'/drone_{i}/motion_capture_state',
                                     lambda m, i=i: self.carriers.__setitem__(i, m), 20)
        self.create_subscription(MultiDOFJointTrajectory, '/dynamic_planner/transfer_reference', self._ref_cb, 10)
        self.create_subscription(String, '/dynamic_planner/transfer_status', self._status_cb, 10)
        self.create_subscription(Bool, '/dynamic_planner/transfer_authority_ack', self._ack_cb, 10)
        self.state_pub = self.create_publisher(MotionCaptureState, '/virtual_q4/state', 10)
        self.target_pub = self.create_publisher(PoseStamped, '/dynamic_planner/transfer_target', 10)
        self.tvel_pub = self.create_publisher(TwistStamped, '/dynamic_planner/transfer_target_velocity', 10)
        self.enable_pub = self.create_publisher(Bool, '/dynamic_planner/transfer_enable', 10)
        self.auth_pub = self.create_publisher(Bool, '/dynamic_planner/transfer_authority', 10)
        self.attached_pub = self.create_publisher(Bool, '/tejen/object_attached', 10)
        self.create_timer(0.01, self._state_tick)
        self.create_timer(0.05, self._input_tick)
        self.create_timer(0.1, self._log_tick)

    def _relay_tick(self):
        if self.relay_msg is not None:
            self.relay_msg.header.stamp = self.get_clock().now().to_msg()
            self.relay_pub.publish(self.relay_msg)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _ring_cb(self, m):
        self.ring = (m.pose.position.x, m.pose.position.y, m.pose.position.z)
        self.ring_v = (m.twist.linear.x, m.twist.linear.y, m.twist.linear.z)

    def _ref_cb(self, m):
        self.ref = m

    def _ack_cb(self, m):
        if m.data and not self.auth_ack:
            self.auth_ack = True
            self.events.append((self.now(), 'AUTHORITY_ACK'))
            self.get_logger().info('authority acknowledged')

    def _status_cb(self, m):
        for line in m.data.splitlines():
            if line.startswith('latest_status:'):
                s = line.split(':', 1)[1].strip()
                if s != self.status:
                    self.events.append((self.now(), s))
                self.status = s

    def _state_tick(self):
        t = self.now()
        if t <= 0.0:
            return
        if self.t_start is None:
            self.t_start = t
        if self.auth_ack and self.ref is not None and self.ref.points:
            st = self.ref.header.stamp.sec + self.ref.header.stamp.nanosec * 1e-9
            tq = t - st
            pts = self.ref.points
            tf = [p.time_from_start.sec + p.time_from_start.nanosec * 1e-9 for p in pts]
            k = 0
            while k + 1 < len(pts) and tf[k + 1] <= tq:
                k += 1
            p0 = pts[k]
            if k + 1 < len(pts) and tf[k + 1] > tf[k]:
                a = min(max((tq - tf[k]) / (tf[k + 1] - tf[k]), 0.0), 1.0)
                p1 = pts[k + 1]
            else:
                a, p1 = 0.0, p0
            tr0, tr1 = p0.transforms[0].translation, p1.transforms[0].translation
            self.pos = [tr0.x + a * (tr1.x - tr0.x), tr0.y + a * (tr1.y - tr0.y), tr0.z + a * (tr1.z - tr0.z)]
            if p0.velocities:
                v0, v1 = p0.velocities[0].linear, (p1.velocities[0].linear if p1.velocities else p0.velocities[0].linear)
                self.vel = [v0.x + a * (v1.x - v0.x), v0.y + a * (v1.y - v0.y), v0.z + a * (v1.z - v0.z)]
        m = MotionCaptureState()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'map'
        m.child_frame_id = 'drone_3'
        m.pose.position.x, m.pose.position.y, m.pose.position.z = self.pos
        m.pose.orientation.w = 1.0
        m.twist.linear.x, m.twist.linear.y, m.twist.linear.z = self.vel
        self.state_pub.publish(m)

    def target(self):
        if self.ring is None:
            return None
        if self.latch:
            # latched once (the hovering ring's pose at the start), no velocity: removes the ring's measured jitter
            if self.latched is None:
                self.latched = (self.ring[0], self.ring[1], self.ring[2] + TARGET_DZ)
            return self.latched
        return (self.ring[0], self.ring[1], self.ring[2] + TARGET_DZ)

    def _input_tick(self):
        t = self.now()
        if t <= 0.0 or self.ring is None:
            return
        tg = self.target()
        stamp = self.get_clock().now().to_msg()
        p = PoseStamped()
        p.header.stamp, p.header.frame_id = stamp, 'map'
        p.pose.position.x, p.pose.position.y, p.pose.position.z = tg
        p.pose.orientation.w = 1.0
        self.target_pub.publish(p)
        v = TwistStamped()
        v.header.stamp, v.header.frame_id = stamp, 'map'
        v.twist.linear.x, v.twist.linear.y, v.twist.linear.z = (0.0, 0.0, 0.0) if self.latch else (self.ring_v[0], self.ring_v[1], 0.0)
        self.tvel_pub.publish(v)
        self.attached_pub.publish(Bool(data=True))
        if self.t_start is not None and t - self.t_start > 1.0:
            self.enable_pub.publish(Bool(data=True))
        if not self.auth_ack and self.status == 'PREPARED_WAITING_AUTHORITY':
            self.auth_pub.publish(Bool(data=True))
        # capture: body within capture_xy of the target in xy and capture_z in z, held capture_hold
        dxy = math.hypot(self.pos[0] - tg[0], self.pos[1] - tg[1])
        dz = self.pos[2] - tg[2]
        if self.auth_ack and dxy < self.cap_xy and abs(dz) < self.cap_z:
            self.cap_since = self.cap_since or t
            if self.captured_t is None and t - self.cap_since >= self.cap_hold:
                self.captured_t = t
                self.events.append((t, 'CAPTURED'))
                self.get_logger().info(f'CAPTURED at +{t - self.t_start:.1f} s')
        else:
            self.cap_since = None
        if self.t_start is not None and t - self.t_start > self.run_s:
            raise SystemExit

    def _log_tick(self):
        t = self.now()
        if t <= 0.0 or self.ring is None:
            return
        tg = self.target()
        min_body, min_ct = float('nan'), float('nan')
        if len(self.carriers) == 3:
            bodies, cts = [], []
            cab_top = self.pos
            cab_bot = (self.pos[0], self.pos[1], self.pos[2] - 1.034 - 0.05)
            for i, m in self.carriers.items():
                b = (m.pose.position.x, m.pose.position.y, m.pose.position.z)
                bodies.append(math.dist(b, self.pos))
                dx, dy = b[0] - self.ring[0], b[1] - self.ring[1]
                n = math.hypot(dx, dy) or 1.0
                plate = (self.ring[0] + 0.225 * dx / n, self.ring[1] + 0.225 * dy / n, self.ring[2])
                cts.append(seg_dist(cab_top, cab_bot, (b[0], b[1], b[2] - 0.04), plate))
            min_body, min_ct = min(bodies), min(cts)
        dxy = math.hypot(self.pos[0] - tg[0], self.pos[1] - tg[1])
        self.log.write(f'{t:.3f},{int(self.auth_ack)},{self.pos[0]:.4f},{self.pos[1]:.4f},{self.pos[2]:.4f},'
                       f'{self.vel[0]:.3f},{self.vel[1]:.3f},{self.vel[2]:.3f},{tg[0]:.4f},{tg[1]:.4f},{tg[2]:.4f},'
                       f'{self.ring[0]:.4f},{self.ring[1]:.4f},{self.ring[2]:.4f},{dxy:.3f},{self.pos[2]-tg[2]:.3f},'
                       f'{int(self.captured_t is not None)},{min_body:.3f},{min_ct:.3f},{self.status}\n')
        self.log.flush()


def main():
    rclpy.init()
    n = Driver()
    try:
        rclpy.spin(n)
    except SystemExit:
        pass
    finally:
        for t, e in n.events:
            n.get_logger().info(f'EVENT +{t - (n.t_start or t):.2f} {e}')
        n.log.close()
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

"""His M1 inputs on the rig, from OUR motion capture (combined-stack H3; card 2026-10-10_combined_m1_contact).

On the combined rig our mocap node is the only Motive owner (one socket on UDP 1511). His mission still
needs what his own IRL publisher (motion_capture_publisher_irl) derives from the same bodies, under /tejen:

  /drone_3/motion_capture_state (QUAD4, body 14) + /magnet_tip_pose (its magnet marker, body 24)
      -> /tejen/magnet_tip_pose        (PoseStamped, the marker as measured)
      -> /tejen/pendulum_swing_state   (his MPC's [phi, theta, phi_dot, theta_dot], pose.z = the length)
  /pickup/motion_capture_state (his ball, body 6)
      -> /tejen/pickup_object/pose     (PoseArray, the ball at index 0, as his IRL publisher)
      -> /tejen/payload_world_state    (MotionCaptureState, his supervisor's readiness input)

The pendulum is his IRL math, unchanged (motion_capture_publisher_irl.compute_pendulum_state): the anchor
is the body pose plus R(q) @ (0, 0, anchor_offset_z) (his IRL -0.030), r = marker - anchor,
phi = atan2(r_x, -r_z), theta = atan2(r_y, -r_z), the rates over the marker's sample times through an
8-sample mean, and no estimate from a body/marker pair older than max_pair_age_s (his 0.20 s).
"""
from collections import deque
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseArray, PoseStamped
from interfaces.msg import MotionCaptureState
from rclpy.node import Node


def _rot(q_wxyz):
    w, x, y, z = q_wxyz
    n = math.sqrt(w * w + x * x + y * y + z * z) or 1.0
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def pendulum_angles(body_pos, body_q_wxyz, tip_pos, anchor_offset_body):
    """(phi, theta, length) of the marker below the anchor, his convention; None when degenerate."""
    anchor = np.asarray(body_pos, float) + _rot(body_q_wxyz) @ np.asarray(anchor_offset_body, float)
    r = np.asarray(tip_pos, float) - anchor
    length = float(np.linalg.norm(r))
    if length < 1e-3:
        return None
    return float(math.atan2(r[0], -r[2])), float(math.atan2(r[1], -r[2])), length


class PendulumRates:
    """His rate estimate: angle differences over the marker sample times, then an 8-sample mean."""

    def __init__(self, n=8):
        self._last = None
        self._phi = deque(maxlen=n)
        self._theta = deque(maxlen=n)

    def step(self, t, phi, theta):
        if self._last is None or t - self._last[0] <= 1e-6:
            rates = (0.0, 0.0)
        else:
            dt = t - self._last[0]
            self._phi.append((phi - self._last[1]) / dt)
            self._theta.append((theta - self._last[2]) / dt)
            rates = (float(np.mean(self._phi)), float(np.mean(self._theta)))
        self._last = (t, phi, theta)
        return rates


class PartnerRigState(Node):
    def __init__(self):
        super().__init__('partner_rig_state')
        p = self.declare_parameter
        ns = str(p('ns', '/tejen').value).rstrip('/')
        self.anchor = (0.0, 0.0, float(p('anchor_offset_z', -0.030).value))
        self.max_pair_age_s = max(0.01, float(p('max_pair_age_s', 0.20).value))
        self._body = None             # (t, position, q_wxyz)
        self._rates = PendulumRates()
        self.tip_pub = self.create_publisher(PoseStamped, f'{ns}/magnet_tip_pose', 10)
        self.pen_pub = self.create_publisher(MotionCaptureState, f'{ns}/pendulum_swing_state', 10)
        self.ball_pose_pub = self.create_publisher(PoseArray, f'{ns}/pickup_object/pose', 10)
        self.ball_state_pub = self.create_publisher(MotionCaptureState, f'{ns}/payload_world_state', 10)
        body = str(p('body_topic', '/drone_3/motion_capture_state').value)
        tip = str(p('tip_topic', '/magnet_tip_pose').value)
        ball = str(p('ball_topic', '/pickup/motion_capture_state').value)
        self.create_subscription(MotionCaptureState, body, self._body_cb, 10)
        self.create_subscription(PoseStamped, tip, self._tip_cb, 10)
        self.create_subscription(MotionCaptureState, ball, self._ball_cb, 10)
        self.get_logger().info(
            f'[partner_rig_state] {body} + {tip} -> {ns}/magnet_tip_pose, {ns}/pendulum_swing_state '
            f'(anchor z {self.anchor[2]:+.3f}, pair age {self.max_pair_age_s:.2f} s); {ball} -> '
            f'{ns}/pickup_object/pose, {ns}/payload_world_state')

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _body_cb(self, msg):
        p, q = msg.pose.position, msg.pose.orientation
        self._body = (self._now(), (p.x, p.y, p.z), (q.w, q.x, q.y, q.z))

    def _tip_cb(self, msg):
        out = PoseStamped()
        out.header = msg.header
        out.pose = msg.pose
        self.tip_pub.publish(out)
        t = self._now()
        if self._body is None or t - self._body[0] > self.max_pair_age_s:
            self.get_logger().warn('[partner_rig_state] no pendulum: QUAD4 body and magnet marker too far apart '
                                   'in time', throttle_duration_sec=2.0)
            return
        tp = msg.pose.position
        ang = pendulum_angles(self._body[1], self._body[2], (tp.x, tp.y, tp.z), self.anchor)
        if ang is None:
            return
        phi, theta, length = ang
        phi_dot, theta_dot = self._rates.step(t, phi, theta)
        m = MotionCaptureState()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'pendulum_state_from_mocap'
        m.pose.position.x, m.pose.position.y, m.pose.position.z = phi, theta, length
        m.pose.orientation.w = 1.0
        # his MPC reads the rates from angular.x/y; linear.x/y as his IRL publisher fills them
        m.twist.linear.x, m.twist.linear.y = phi_dot, theta_dot
        m.twist.angular.x, m.twist.angular.y = phi_dot, theta_dot
        self.pen_pub.publish(m)

    def _ball_cb(self, msg):
        arr = PoseArray()
        arr.header.stamp = msg.header.stamp
        arr.header.frame_id = 'map'
        arr.poses = [msg.pose]
        self.ball_pose_pub.publish(arr)
        self.ball_state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PartnerRigState()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

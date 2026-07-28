#!/usr/bin/env python3
"""
Publish pendulum swing state from the simulated electromagnet/magnet-tip pose,
and publish /magnet_tip_pose in the WORLD frame.

Current IRL mocap PoseArray convention:
  drone_index      = 7   # quad body pose in world frame
  magnet_index     = 8   # electromagnet/magnet-tip pose in world frame

For IRL tests, the attachment/anchor point can be estimated from the quad pose
plus a configurable static body-frame offset instead of requiring a separate
attachment marker.

This node publishes:
  /pendulum_swing_state
      pose.position.x = phi
      pose.position.y = theta
      pose.position.z = cable length
      twist.angular.x = phi_dot
      twist.angular.y = theta_dot

  /magnet_tip_pose
      PoseStamped magnet-tip pose in frame_id, default "map"

  /payload_world_state
      pickup object / red-ball world state from /model/payload_model/pose[object_index]

Important frame convention:
  The magnet-tip and attachment entries from /model/x3/pose are treated as
  drone-local coordinates by default. Therefore /magnet_tip_pose is computed as

      p_tip_world = p_drone_world + R_world_drone * p_tip_local

  while pendulum angles are computed from the local relative vector

      r_local = p_tip_local - p_attachment_local.
"""

from __future__ import annotations

import math
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose, PoseArray, PoseStamped, Twist
from interfaces.msg import MotionCaptureState


class PendulumStatePublisher(Node):
    def __init__(self) -> None:
        super().__init__('pendulum_state_publisher')

        # Source containing x3 link/model poses. ROS PoseArray loses link names,
        # so the indices are parameters.
        self.x3_pose_topic = str(self.declare_parameter('x3_pose_topic', '/model/x3/pose').value)
        # Backwards-compatible alias. If someone passes -p drone_topic:=..., use it.
        self.drone_topic = str(self.declare_parameter('drone_topic', self.x3_pose_topic).value)
        if self.drone_topic != self.x3_pose_topic:
            self.x3_pose_topic = self.drone_topic

        self.magnet_index = int(self.declare_parameter('magnet_index', 8).value)
        self.attachment_index = int(self.declare_parameter('attachment_index', 5).value)
        self.drone_index = int(self.declare_parameter('drone_index', 7).value)

        # Sim mode: magnet/attachment indices may be local to the drone.
        # IRL mocap mode: quad and magnet markers are already world-frame.
        self.x3_link_poses_are_relative = bool(
            self.declare_parameter('x3_link_poses_are_relative', False).value
        )
        self.use_drone_as_attachment_anchor = bool(
            self.declare_parameter('use_drone_as_attachment_anchor', True).value
        )
        self.attachment_offset_x = float(self.declare_parameter('attachment_offset_x', 0.0).value)
        self.attachment_offset_y = float(self.declare_parameter('attachment_offset_y', 0.0).value)
        self.attachment_offset_z = float(self.declare_parameter('attachment_offset_z', 0.0).value)

        # Pickup object / red ball. This is NOT used for pendulum angles before
        # pickup; it is only republished as an object/world-state topic.
        self.object_pose_topic = str(self.declare_parameter('object_pose_topic', '/model/payload_model/pose').value)
        # Backwards-compatible alias for old launch commands.
        self.payload_topic = str(self.declare_parameter('payload_topic', self.object_pose_topic).value)
        if self.payload_topic != self.object_pose_topic:
            self.object_pose_topic = self.payload_topic
        self.object_index = int(self.declare_parameter('object_index', 1).value)
        self.payload_index = int(self.declare_parameter('payload_index', self.object_index).value)
        if self.payload_index != self.object_index:
            self.object_index = self.payload_index

        self.frame_id = str(self.declare_parameter('frame_id', 'map').value)
        self.publish_rate_hz = max(1.0, float(self.declare_parameter('publish_rate_hz', 500.0).value))
        self.warn_throttle_s = float(self.declare_parameter('warn_throttle_s', 2.0).value)
        self.publish_payload_world_state = bool(self.declare_parameter('publish_payload_world_state', False).value)
        self.publish_magnet_tip_pose = bool(self.declare_parameter('publish_magnet_tip_pose', True).value)

        self.magnet_pose_local_or_world: Optional[Pose] = None
        self.attachment_pose_local_or_world: Optional[Pose] = None
        self.drone_pose_world: Optional[Pose] = None
        self.object_pose_world: Optional[Pose] = None

        self.last_phi: Optional[float] = None
        self.last_theta: Optional[float] = None
        self.last_tip_world_position: Optional[np.ndarray] = None
        self.last_object_position: Optional[np.ndarray] = None
        self.last_time: Optional[float] = None
        self.last_object_time: Optional[float] = None

        self.create_subscription(PoseArray, self.x3_pose_topic, self.x3_pose_callback, 10)
        self.create_subscription(PoseArray, self.object_pose_topic, self.object_pose_callback, 10)

        self.pendulum_pub = self.create_publisher(MotionCaptureState, '/pendulum_swing_state', 10)
        self.payload_world_pub = self.create_publisher(MotionCaptureState, '/payload_world_state', 10)
        self.magnet_tip_pub = self.create_publisher(PoseStamped, '/magnet_tip_pose', 10)

        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self.publish_state)

        self.get_logger().info(
            'PendulumStatePublisher started with electromagnet-based state and world-frame magnet tip. '
            f'x3_pose_topic={self.x3_pose_topic}, magnet_index={self.magnet_index}, '
            f'attachment_index={self.attachment_index}, drone_index={self.drone_index}, '
            f'x3_link_poses_are_relative={self.x3_link_poses_are_relative}, '
            f'use_drone_as_attachment_anchor={self.use_drone_as_attachment_anchor}, '
            f'attachment_offset=({self.attachment_offset_x:.3f},{self.attachment_offset_y:.3f},{self.attachment_offset_z:.3f}), '
            f'object_pose_topic={self.object_pose_topic}, object_index={self.object_index}'
        )

    @staticmethod
    def _pose_position(pose: Pose) -> np.ndarray:
        return np.array([pose.position.x, pose.position.y, pose.position.z], dtype=float)

    @staticmethod
    def _select_pose(msg: PoseArray, index: int) -> Optional[Pose]:
        if not msg.poses:
            return None
        if index < 0:
            index = len(msg.poses) + index
        if index < 0 or index >= len(msg.poses):
            return None
        return msg.poses[index]

    @staticmethod
    def _quat_xyzw(pose: Pose) -> np.ndarray:
        return np.array(
            [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w],
            dtype=float,
        )

    @staticmethod
    def _normalize_quat(q: np.ndarray) -> np.ndarray:
        n = float(np.linalg.norm(q))
        if n < 1e-12:
            return np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
        return q / n

    @classmethod
    def _rotate_vector_by_quat(cls, v: np.ndarray, q_xyzw: np.ndarray) -> np.ndarray:
        """Rotate vector v by quaternion q = [x, y, z, w]."""
        q = cls._normalize_quat(q_xyzw)
        q_vec = q[:3]
        q_w = q[3]
        # Numerically stable equivalent of q * [v,0] * q_conjugate.
        t = 2.0 * np.cross(q_vec, v)
        return v + q_w * t + np.cross(q_vec, t)

    @classmethod
    def _quat_multiply(cls, q1_xyzw: np.ndarray, q2_xyzw: np.ndarray) -> np.ndarray:
        """Return q1*q2 for quaternions stored as [x, y, z, w]."""
        x1, y1, z1, w1 = cls._normalize_quat(q1_xyzw)
        x2, y2, z2, w2 = cls._normalize_quat(q2_xyzw)
        return cls._normalize_quat(np.array([
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        ], dtype=float))

    @classmethod
    def _compose_local_pose_with_parent_world(cls, local_pose: Pose, parent_world_pose: Pose) -> Pose:
        local_p = cls._pose_position(local_pose)
        parent_p = cls._pose_position(parent_world_pose)
        parent_q = cls._quat_xyzw(parent_world_pose)
        local_q = cls._quat_xyzw(local_pose)

        world_p = parent_p + cls._rotate_vector_by_quat(local_p, parent_q)
        world_q = cls._quat_multiply(parent_q, local_q)

        out = Pose()
        out.position.x = float(world_p[0])
        out.position.y = float(world_p[1])
        out.position.z = float(world_p[2])
        out.orientation.x = float(world_q[0])
        out.orientation.y = float(world_q[1])
        out.orientation.z = float(world_q[2])
        out.orientation.w = float(world_q[3])
        return out

    def x3_pose_callback(self, msg: PoseArray) -> None:
        magnet_pose = self._select_pose(msg, self.magnet_index)
        attachment_pose = self._select_pose(msg, self.attachment_index)
        drone_pose = self._select_pose(msg, self.drone_index)

        if magnet_pose is None:
            self.get_logger().warn(
                f'magnet_index={self.magnet_index} out of range for {self.x3_pose_topic}; length={len(msg.poses)}',
                throttle_duration_sec=self.warn_throttle_s,
            )
            return
        if attachment_pose is None and not self.use_drone_as_attachment_anchor:
            self.get_logger().warn(
                f'attachment_index={self.attachment_index} out of range for {self.x3_pose_topic}; length={len(msg.poses)}',
                throttle_duration_sec=self.warn_throttle_s,
            )
            return
        if (self.x3_link_poses_are_relative or self.use_drone_as_attachment_anchor) and drone_pose is None:
            self.get_logger().warn(
                f'drone_index={self.drone_index} out of range for {self.x3_pose_topic}; length={len(msg.poses)}. '
                'Need quad world pose to publish world-frame /magnet_tip_pose.',
                throttle_duration_sec=self.warn_throttle_s,
            )
            return

        self.magnet_pose_local_or_world = magnet_pose
        self.attachment_pose_local_or_world = attachment_pose
        self.drone_pose_world = drone_pose

    def object_pose_callback(self, msg: PoseArray) -> None:
        object_pose = self._select_pose(msg, self.object_index)
        if object_pose is None:
            self.get_logger().warn(
                f'object_index={self.object_index} out of range for {self.object_pose_topic}; length={len(msg.poses)}',
                throttle_duration_sec=self.warn_throttle_s,
            )
            return
        self.object_pose_world = object_pose

    def _current_tip_world_pose(self) -> Optional[Pose]:
        if self.magnet_pose_local_or_world is None:
            return None
        if self.x3_link_poses_are_relative:
            if self.drone_pose_world is None:
                return None
            return self._compose_local_pose_with_parent_world(
                self.magnet_pose_local_or_world,
                self.drone_pose_world,
            )
        return self.magnet_pose_local_or_world

    def _attachment_anchor_world_pose(self) -> Optional[Pose]:
        if self.use_drone_as_attachment_anchor:
            if self.drone_pose_world is None:
                return None
            local = Pose()
            local.position.x = float(self.attachment_offset_x)
            local.position.y = float(self.attachment_offset_y)
            local.position.z = float(self.attachment_offset_z)
            local.orientation.w = 1.0
            return self._compose_local_pose_with_parent_world(local, self.drone_pose_world)
        if self.attachment_pose_local_or_world is None:
            return None
        if self.x3_link_poses_are_relative:
            if self.drone_pose_world is None:
                return None
            return self._compose_local_pose_with_parent_world(
                self.attachment_pose_local_or_world,
                self.drone_pose_world,
            )
        return self.attachment_pose_local_or_world

    def publish_state(self) -> None:
        if self.magnet_pose_local_or_world is None:
            return

        now = self.get_clock().now()
        t = now.nanoseconds * 1e-9

        tip_world_pose = self._current_tip_world_pose()
        attachment_world_pose = self._attachment_anchor_world_pose()
        if tip_world_pose is None or attachment_world_pose is None:
            return

        tip_world_position = self._pose_position(tip_world_pose)
        attach_world_position = self._pose_position(attachment_world_pose)

        if self.x3_link_poses_are_relative and not self.use_drone_as_attachment_anchor:
            p_magnet_for_angle = self._pose_position(self.magnet_pose_local_or_world)
            p_attach_for_angle = self._pose_position(self.attachment_pose_local_or_world)
            r = p_magnet_for_angle - p_attach_for_angle
        else:
            r = tip_world_position - attach_world_position

        cable_len = float(np.linalg.norm(r))
        if cable_len < 1e-6:
            return

        # At rest, magnet below attachment gives r ~= [0, 0, -L].
        phi = math.atan2(float(r[0]), float(-r[2]))
        theta = math.atan2(float(r[1]), float(-r[2]))

        tip_world_velocity = np.zeros(3, dtype=float)
        if self.last_time is None or self.last_phi is None or self.last_theta is None:
            phi_dot = 0.0
            theta_dot = 0.0
        else:
            dt = max(1e-6, t - self.last_time)
            phi_dot = (phi - self.last_phi) / dt
            theta_dot = (theta - self.last_theta) / dt
            if self.last_tip_world_position is not None and tip_world_position is not None:
                tip_world_velocity = (tip_world_position - self.last_tip_world_position) / dt

        self.last_phi = phi
        self.last_theta = theta
        if tip_world_position is not None:
            self.last_tip_world_position = tip_world_position.copy()
        self.last_time = t

        swing_msg = MotionCaptureState()
        try:
            swing_msg.header.stamp = now.to_msg()
            swing_msg.header.frame_id = self.frame_id
        except Exception:
            pass
        swing_msg.pose = Pose()
        swing_msg.twist = Twist()
        swing_msg.pose.position.x = float(phi)
        swing_msg.pose.position.y = float(theta)
        swing_msg.pose.position.z = float(cable_len)
        swing_msg.twist.angular.x = float(phi_dot)
        swing_msg.twist.angular.y = float(theta_dot)
        swing_msg.twist.angular.z = 0.0
        if tip_world_position is not None:
            swing_msg.twist.linear.x = float(tip_world_velocity[0])
            swing_msg.twist.linear.y = float(tip_world_velocity[1])
            swing_msg.twist.linear.z = float(tip_world_velocity[2])
        self.pendulum_pub.publish(swing_msg)

        if self.publish_magnet_tip_pose and tip_world_pose is not None:
            tip_msg = PoseStamped()
            tip_msg.header.stamp = now.to_msg()
            tip_msg.header.frame_id = self.frame_id
            tip_msg.pose = tip_world_pose
            self.magnet_tip_pub.publish(tip_msg)

        if self.publish_payload_world_state and self.object_pose_world is not None:
            object_p = self._pose_position(self.object_pose_world)
            object_velocity = np.zeros(3, dtype=float)
            if self.last_object_time is not None and self.last_object_position is not None:
                dt_obj = max(1e-6, t - self.last_object_time)
                object_velocity = (object_p - self.last_object_position) / dt_obj
            self.last_object_time = t
            self.last_object_position = object_p.copy()

            payload_msg = MotionCaptureState()
            try:
                payload_msg.header.stamp = now.to_msg()
                payload_msg.header.frame_id = self.frame_id
            except Exception:
                pass
            payload_msg.pose = Pose()
            payload_msg.twist = Twist()
            payload_msg.pose.position.x = float(object_p[0])
            payload_msg.pose.position.y = float(object_p[1])
            payload_msg.pose.position.z = float(object_p[2])
            payload_msg.pose.orientation = self.object_pose_world.orientation
            payload_msg.twist.linear.x = float(object_velocity[0])
            payload_msg.twist.linear.y = float(object_velocity[1])
            payload_msg.twist.linear.z = float(object_velocity[2])
            self.payload_world_pub.publish(payload_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PendulumStatePublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

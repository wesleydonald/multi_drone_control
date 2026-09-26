#!/usr/bin/env python3
"""Publish /magnet_tip_pose in world frame from /model/x3/pose.

Current /model/x3/pose convention after adding magnet_tip_link:
  magnet_index = 0   # magnet tip local/relative pose
  drone_index  = 7   # quad body world pose

World-frame output:
  p_tip_world = p_drone_world + R_world_drone * p_tip_local

This standalone node is optional if pendulum_state_publisher is already running
with publish_magnet_tip_pose:=true.
"""

from __future__ import annotations

from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray, PoseStamped


class MagnetTipPublisher(Node):
    def __init__(self) -> None:
        super().__init__('magnet_tip_publisher')
        self.x3_pose_topic = str(self.declare_parameter('x3_pose_topic', '/model/x3/pose').value)
        self.magnet_index = int(self.declare_parameter('magnet_index', 0).value)
        self.drone_index = int(self.declare_parameter('drone_index', 7).value)
        self.x3_link_poses_are_relative = bool(
            self.declare_parameter('x3_link_poses_are_relative', True).value
        )
        self.magnet_tip_topic = str(self.declare_parameter('magnet_tip_topic', '/magnet_tip_pose').value)
        self.frame_id = str(self.declare_parameter('frame_id', 'map').value)
        self.warn_throttle_s = float(self.declare_parameter('warn_throttle_s', 2.0).value)

        self.pub = self.create_publisher(PoseStamped, self.magnet_tip_topic, 10)
        self.create_subscription(PoseArray, self.x3_pose_topic, self.pose_callback, 10)
        self.get_logger().info(
            f'MagnetTipPublisher started: {self.x3_pose_topic}[{self.magnet_index}] -> {self.magnet_tip_topic}, '
            f'drone_index={self.drone_index}, x3_link_poses_are_relative={self.x3_link_poses_are_relative}'
        )

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
    def _pose_position(pose: Pose) -> np.ndarray:
        return np.array([pose.position.x, pose.position.y, pose.position.z], dtype=float)

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
        q = cls._normalize_quat(q_xyzw)
        q_vec = q[:3]
        q_w = q[3]
        t = 2.0 * np.cross(q_vec, v)
        return v + q_w * t + np.cross(q_vec, t)

    @classmethod
    def _quat_multiply(cls, q1_xyzw: np.ndarray, q2_xyzw: np.ndarray) -> np.ndarray:
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

    def pose_callback(self, msg: PoseArray) -> None:
        tip_pose = self._select_pose(msg, self.magnet_index)
        if tip_pose is None:
            self.get_logger().warn(
                f'magnet_index={self.magnet_index} out of range for {self.x3_pose_topic}; length={len(msg.poses)}',
                throttle_duration_sec=self.warn_throttle_s,
            )
            return

        if self.x3_link_poses_are_relative:
            drone_pose = self._select_pose(msg, self.drone_index)
            if drone_pose is None:
                self.get_logger().warn(
                    f'drone_index={self.drone_index} out of range for {self.x3_pose_topic}; length={len(msg.poses)}',
                    throttle_duration_sec=self.warn_throttle_s,
                )
                return
            out_pose = self._compose_local_pose_with_parent_world(tip_pose, drone_pose)
        else:
            out_pose = tip_pose

        out = PoseStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.frame_id
        out.pose = out_pose
        self.pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MagnetTipPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

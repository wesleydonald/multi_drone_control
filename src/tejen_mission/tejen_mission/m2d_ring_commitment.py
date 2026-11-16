#!/usr/bin/env python3
"""Publish the measured stationary M2D ring pose as an authoritative commitment."""

from __future__ import annotations

import math

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseArray
from interfaces.msg import CommittedTrajectory, CubicTrajectoryPiece
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from .m2_fleet_commitment import commitment_sequence_seed


def _latched_qos() -> QoSProfile:
    qos = QoSProfile(depth=1)
    qos.reliability = ReliabilityPolicy.RELIABLE
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    return qos


class M2DRingCommitment(Node):
    """Bridge measured ring centre pose to the existing planner commitment seam.

    M2D intentionally keeps the ring stationary.  The node still consumes the
    measured PoseArray rather than a hard-coded fixture pose, so a deliberately
    rotated/translated commissioning ring remains authoritative.
    """

    def __init__(self) -> None:
        super().__init__("m2d_ring_commitment")
        self.pose_topic = str(
            self.declare_parameter("ring_pose_topic", "/model/payload_model/pose").value
        )
        self.pose_index = int(self.declare_parameter("ring_pose_index", 1).value)
        self.output_topic = str(
            self.declare_parameter(
                "committed_trajectory_topic", "/m2d/ring/committed_trajectory"
            ).value
        )
        self.vehicle_id = str(self.declare_parameter("vehicle_id", "m2d_ring").value)
        self.frame_id = str(self.declare_parameter("frame_id", "map").value)
        self.publish_rate_hz = max(
            1.0, float(self.declare_parameter("publish_rate_hz", 10.0).value)
        )
        self.horizon_s = max(
            1.0, float(self.declare_parameter("commitment_horizon_s", 10.0).value)
        )
        self.pose_timeout_s = max(
            0.05, float(self.declare_parameter("ring_pose_timeout_s", 0.50).value)
        )
        if self.pose_index < 0:
            raise ValueError("ring_pose_index must be non-negative")

        self._position: np.ndarray | None = None
        self._last_pose_s: float | None = None
        self._sequence = commitment_sequence_seed()

        self.create_subscription(PoseArray, self.pose_topic, self._pose_callback, 10)
        self.publisher = self.create_publisher(
            CommittedTrajectory, self.output_topic, _latched_qos()
        )
        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self._tick)

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _pose_callback(self, msg: PoseArray) -> None:
        if self.pose_index >= len(msg.poses):
            self.get_logger().warning(
                f"Ring pose index {self.pose_index} unavailable in PoseArray size {len(msg.poses)}"
            )
            return
        pose = msg.poses[self.pose_index]
        position = np.array([pose.position.x, pose.position.y, pose.position.z], dtype=float)
        if not np.all(np.isfinite(position)):
            self.get_logger().warning("Rejected non-finite measured ring pose")
            return
        self._position = position
        self._last_pose_s = self._now_s()

    def _tick(self) -> None:
        if self._position is None or self._last_pose_s is None:
            return
        now = self.get_clock().now()
        now_s = now.nanoseconds * 1e-9
        age_s = now_s - self._last_pose_s
        if not math.isfinite(age_s) or age_s < -0.02 or age_s > self.pose_timeout_s:
            return

        t0 = now_s
        t1 = t0 + self.horizon_s
        msg = CommittedTrajectory()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.frame_id
        msg.vehicle_id = self.vehicle_id
        self._sequence += 1
        msg.sequence = self._sequence
        msg.terminal_hold = True

        piece = CubicTrajectoryPiece()
        piece.valid_from_s = t0
        piece.valid_until_s = t1
        piece.knots = [t0, t0, t0, t0, t1, t1, t1, t1]
        piece.control_points = []
        for _ in range(4):
            point = Point()
            point.x = float(self._position[0])
            point.y = float(self._position[1])
            point.z = float(self._position[2])
            piece.control_points.append(point)
        msg.pieces = [piece]
        self.publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = M2DRingCommitment()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

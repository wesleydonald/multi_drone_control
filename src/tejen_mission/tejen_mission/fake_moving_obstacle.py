#!/usr/bin/env python3
"""Deterministic ROS 2 moving-obstacle test publisher for M1 prediction.

This node is intentionally a test source, not planner logic.  It publishes a
single spherical moving obstacle using the same MotionCaptureState convention as
`fake_cooperative_transport_world`, plus PoseStamped/TwistStamped convenience
topics and RViz markers for the predicted widening reachable tube.

The predictor itself lives in `moving_obstacle_prediction.py` and is ROS
independent, so the same maths can later consume real mocap states.
"""

from __future__ import annotations

import math
from typing import Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Quaternion, Twist, TwistStamped
from visualization_msgs.msg import Marker, MarkerArray
from interfaces.msg import MotionCaptureState

from .moving_obstacle_prediction import (
    MovingObstaclePrediction,
    MovingObstaclePredictionConfig,
    MovingObstaclePredictor,
    MovingObstacleState,
)


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(0.5 * yaw)
    q.w = math.cos(0.5 * yaw)
    return q


class FakeMovingObstacle(Node):
    """Publish one deterministic moving sphere and its M1 prediction tube."""

    def __init__(self) -> None:
        super().__init__("fake_moving_obstacle")

        self.declare_parameter("frame_id", "map")
        self.declare_parameter("state_topic", "/fake_moving_obstacle/state")
        self.declare_parameter("pose_topic", "/fake_moving_obstacle/pose")
        self.declare_parameter("twist_topic", "/fake_moving_obstacle/twist")
        self.declare_parameter("marker_topic", "/fake_moving_obstacle/markers")
        self.declare_parameter("publish_rate_hz", 50.0)

        # Test trajectory.  Defaults are chosen to give an obvious crossing motion.
        # Supported: stationary, constant_velocity, line, circle, figure8.
        self.declare_parameter("trajectory_type", "constant_velocity")
        self.declare_parameter("center_x", 0.0)
        self.declare_parameter("center_y", -1.0)
        self.declare_parameter("center_z", 1.5)

        self.declare_parameter("velocity_x", 0.0)
        self.declare_parameter("velocity_y", 0.25)
        self.declare_parameter("velocity_z", 0.0)

        self.declare_parameter("line_amplitude", 0.8)
        self.declare_parameter("radius", 0.8)
        self.declare_parameter("omega", 0.35)
        self.declare_parameter("z_amplitude", 0.0)
        self.declare_parameter("z_omega", 0.35)

        self.declare_parameter("obstacle_radius_m", 0.18)

        # M1 reachable-set predictor parameters.
        self.declare_parameter("prediction_horizon_s", 3.0)
        self.declare_parameter("prediction_time_step_s", 0.25)
        self.declare_parameter("velocity_filter_tau_s", 0.20)
        self.declare_parameter("position_uncertainty_m", 0.03)
        self.declare_parameter("velocity_uncertainty_mps", 0.10)
        self.declare_parameter("acceleration_uncertainty_mps2", 0.15)

        self.frame_id = str(self.get_parameter("frame_id").value)
        self.state_topic = str(self.get_parameter("state_topic").value)
        self.pose_topic = str(self.get_parameter("pose_topic").value)
        self.twist_topic = str(self.get_parameter("twist_topic").value)
        self.marker_topic = str(self.get_parameter("marker_topic").value)

        config = MovingObstaclePredictionConfig(
            horizon_s=self._float("prediction_horizon_s"),
            time_step_s=self._float("prediction_time_step_s"),
            velocity_filter_tau_s=self._float("velocity_filter_tau_s"),
            position_uncertainty_m=self._float("position_uncertainty_m"),
            velocity_uncertainty_mps=self._float("velocity_uncertainty_mps"),
            acceleration_uncertainty_mps2=self._float(
                "acceleration_uncertainty_mps2"
            ),
        )
        self.predictor = MovingObstaclePredictor(config)

        self.state_pub = self.create_publisher(
            MotionCaptureState,
            self.state_topic,
            10,
        )
        self.pose_pub = self.create_publisher(
            PoseStamped,
            self.pose_topic,
            10,
        )
        self.twist_pub = self.create_publisher(
            TwistStamped,
            self.twist_topic,
            10,
        )
        self.marker_pub = self.create_publisher(
            MarkerArray,
            self.marker_topic,
            10,
        )

        publish_rate_hz = max(
            1.0,
            self._float("publish_rate_hz"),
        )
        self.timer_period_s = 1.0 / publish_rate_hz
        self.start_time = self.get_clock().now()
        self.last_time_s = None
        self.timer = self.create_timer(
            self.timer_period_s,
            self._timer_callback,
        )

        self.get_logger().info(
            "Fake moving obstacle started: "
            f"state={self.state_topic}, markers={self.marker_topic}, "
            f"trajectory={self._string('trajectory_type')}, "
            f"prediction horizon={config.horizon_s:.2f}s"
        )

    def _float(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def _string(self, name: str) -> str:
        return str(self.get_parameter(name).value).strip().lower()

    def _time_since_start_s(self) -> float:
        return (
            self.get_clock().now() - self.start_time
        ).nanoseconds * 1e-9

    def _trajectory_state(
        self,
        time_s: float,
    ) -> Tuple[Tuple[float, float, float], Tuple[float, float, float], float]:
        trajectory = self._string("trajectory_type")

        cx = self._float("center_x")
        cy = self._float("center_y")
        cz = self._float("center_z")

        vx0 = self._float("velocity_x")
        vy0 = self._float("velocity_y")
        vz0 = self._float("velocity_z")

        omega = self._float("omega")
        radius = self._float("radius")
        line_amplitude = self._float("line_amplitude")
        z_amplitude = self._float("z_amplitude")
        z_omega = self._float("z_omega")

        if trajectory == "stationary":
            x, y = cx, cy
            vx, vy = 0.0, 0.0

        elif trajectory == "constant_velocity":
            x = cx + vx0 * time_s
            y = cy + vy0 * time_s
            vx, vy = vx0, vy0

        # elif trajectory == "line":
        #     x = cx + line_amplitude * math.sin(omega * time_s)
        #     y = cy
        #     vx = line_amplitude * omega * math.cos(omega * time_s)
        #     vy = 0.0

        elif trajectory == "line":
            x = cx
            y = cy + line_amplitude * math.sin(omega * time_s)
            vx = 0.0
            vy = line_amplitude * omega * math.cos(omega * time_s)

        elif trajectory == "circle":
            x = cx + radius * math.cos(omega * time_s)
            y = cy + radius * math.sin(omega * time_s)
            vx = -radius * omega * math.sin(omega * time_s)
            vy = radius * omega * math.cos(omega * time_s)

        elif trajectory == "figure8":
            x = cx + radius * math.sin(omega * time_s)
            y = cy + 0.5 * radius * math.sin(2.0 * omega * time_s)
            vx = radius * omega * math.cos(omega * time_s)
            vy = radius * omega * math.cos(2.0 * omega * time_s)

        else:
            self.get_logger().warn(
                f"Unknown trajectory_type='{trajectory}', using stationary."
            )
            x, y = cx, cy
            vx, vy = 0.0, 0.0

        if trajectory == "constant_velocity":
            z = cz + vz0 * time_s
            vz = vz0
        else:
            z = cz + z_amplitude * math.sin(z_omega * time_s)
            vz = z_amplitude * z_omega * math.cos(z_omega * time_s)

        yaw = math.atan2(vy, vx) if abs(vx) + abs(vy) > 1e-9 else 0.0

        return (
            (x, y, z),
            (vx, vy, vz),
            yaw,
        )

    def _make_pose_stamped(
        self,
        position: Tuple[float, float, float],
        yaw: float,
        stamp,
    ) -> PoseStamped:
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.pose.position.x = position[0]
        msg.pose.position.y = position[1]
        msg.pose.position.z = position[2]
        msg.pose.orientation = yaw_to_quaternion(yaw)
        return msg

    def _make_twist_stamped(
        self,
        velocity: Tuple[float, float, float],
        stamp,
    ) -> TwistStamped:
        msg = TwistStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.twist.linear.x = velocity[0]
        msg.twist.linear.y = velocity[1]
        msg.twist.linear.z = velocity[2]
        return msg

    def _make_motion_capture_state(
        self,
        pose: PoseStamped,
        twist: TwistStamped,
    ) -> MotionCaptureState:
        msg = MotionCaptureState()
        msg.header = Header()
        msg.header.stamp = pose.header.stamp
        msg.header.frame_id = self.frame_id
        msg.pose = pose.pose
        msg.twist = Twist()
        msg.twist.linear.x = twist.twist.linear.x
        msg.twist.linear.y = twist.twist.linear.y
        msg.twist.linear.z = twist.twist.linear.z
        return msg

    def _common_marker(
        self,
        marker_id: int,
        namespace: str,
        marker_type: int,
        stamp,
    ) -> Marker:
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.frame_id
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0
        return marker

    def _prediction_markers(
        self,
        actual_position: Tuple[float, float, float],
        prediction: MovingObstaclePrediction,
        stamp,
    ) -> MarkerArray:
        array = MarkerArray()

        radius = self._float("obstacle_radius_m")
        actual = self._common_marker(
            0,
            "fake_moving_obstacle_actual",
            Marker.SPHERE,
            stamp,
        )
        actual.pose.position.x = actual_position[0]
        actual.pose.position.y = actual_position[1]
        actual.pose.position.z = actual_position[2]
        actual.scale.x = 2.0 * radius
        actual.scale.y = 2.0 * radius
        actual.scale.z = 2.0 * radius
        actual.color.r = 1.0
        actual.color.g = 0.25
        actual.color.b = 0.05
        actual.color.a = 0.95
        array.markers.append(actual)

        # Each future sphere is one time slice of the expanding reachable set.
        # Together they make the widening bugle intuitive in RViz.
        last_index = max(
            1,
            len(prediction.sample_times_s) - 1,
        )
        for index, (center, reach_radius, future_time) in enumerate(
            zip(
                prediction.sample_centers,
                prediction.sample_radii_m,
                prediction.sample_times_s,
            )
        ):
            sphere = self._common_marker(
                100 + index,
                "moving_obstacle_reachable_slices",
                Marker.SPHERE,
                stamp,
            )
            sphere.pose.position.x = float(center[0])
            sphere.pose.position.y = float(center[1])
            sphere.pose.position.z = float(center[2])
            sphere.scale.x = 2.0 * float(reach_radius)
            sphere.scale.y = 2.0 * float(reach_radius)
            sphere.scale.z = 2.0 * float(reach_radius)
            sphere.color.r = 1.0
            sphere.color.g = 0.75
            sphere.color.b = 0.05
            sphere.color.a = 0.08 + 0.12 * (1.0 - index / last_index)
            array.markers.append(sphere)

            label = self._common_marker(
                500 + index,
                "moving_obstacle_prediction_time",
                Marker.TEXT_VIEW_FACING,
                stamp,
            )
            label.pose.position.x = float(center[0])
            label.pose.position.y = float(center[1])
            label.pose.position.z = float(center[2] + reach_radius + 0.08)
            label.scale.z = 0.08
            label.color.r = 1.0
            label.color.g = 0.9
            label.color.b = 0.2
            label.color.a = 0.8
            label.text = f"{float(future_time):.2f}s"
            array.markers.append(label)

        centreline = self._common_marker(
            10,
            "moving_obstacle_prediction_centreline",
            Marker.LINE_STRIP,
            stamp,
        )
        centreline.scale.x = 0.015
        centreline.color.r = 1.0
        centreline.color.g = 0.8
        centreline.color.b = 0.1
        centreline.color.a = 0.9

        for center in prediction.sample_centers:
            from geometry_msgs.msg import Point

            point = Point()
            point.x = float(center[0])
            point.y = float(center[1])
            point.z = float(center[2])
            centreline.points.append(point)

        array.markers.append(centreline)
        return array

    def _timer_callback(self) -> None:
        time_s = self._time_since_start_s()
        stamp = self.get_clock().now().to_msg()

        position, velocity, yaw = self._trajectory_state(time_s)

        state = MovingObstacleState(
            obstacle_id="fake_moving_obstacle",
            position=position,
            velocity=velocity,
            geometry_radius_m=self._float("obstacle_radius_m"),
        )

        dt_s = None
        if self.last_time_s is not None:
            dt_s = max(
                1e-6,
                time_s - self.last_time_s,
            )

        prediction = self.predictor.update(
            state,
            dt_s=dt_s,
        )

        pose = self._make_pose_stamped(
            position,
            yaw,
            stamp,
        )
        twist = self._make_twist_stamped(
            velocity,
            stamp,
        )
        mocap = self._make_motion_capture_state(
            pose,
            twist,
        )

        self.pose_pub.publish(pose)
        self.twist_pub.publish(twist)
        self.state_pub.publish(mocap)
        self.marker_pub.publish(
            self._prediction_markers(
                position,
                prediction,
                stamp,
            )
        )

        self.last_time_s = time_s


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FakeMovingObstacle()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

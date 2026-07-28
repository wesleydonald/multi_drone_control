#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseArray, Pose, Twist
from interfaces.msg import MotionCaptureState


class PendulumStatePublisher(Node):
    def __init__(self):
        super().__init__("pendulum_state_publisher")

        self.declare_parameter("drone_topic", "/model/x3/pose")
        self.declare_parameter("payload_topic", "/model/pendulum/pose")
        self.declare_parameter("drone_index", 6)
        self.declare_parameter("payload_index", 1)

        self.drone_topic = self.get_parameter("drone_topic").value
        self.payload_topic = self.get_parameter("payload_topic").value
        self.drone_index = self.get_parameter("drone_index").value
        self.payload_index = self.get_parameter("payload_index").value

        self.drone_pose = None
        self.payload_pose = None

        self.last_phi = None
        self.last_theta = None
        self.last_time = None

        self.drone_sub = self.create_subscription(
            PoseArray,
            self.drone_topic,
            self.drone_callback,
            10,
        )

        self.payload_sub = self.create_subscription(
            PoseArray,
            self.payload_topic,
            self.payload_callback,
            10,
        )

        self.pendulum_pub = self.create_publisher(
            MotionCaptureState,
            "/pendulum_swing_state",
            10,
        )

        self.payload_world_pub = self.create_publisher(
            MotionCaptureState,
            "/payload_world_state",
            10,
        )

        self.timer = self.create_timer(0.002, self.publish_state)  # 500 Hz max

        self.get_logger().info(
            f"PendulumStatePublisher using drone {self.drone_topic}[{self.drone_index}] "
            f"and payload {self.payload_topic}[{self.payload_index}]"
        )

    def drone_callback(self, msg: PoseArray):
        if len(msg.poses) <= self.drone_index:
            self.get_logger().warn(
                f"Drone index {self.drone_index} out of range for {self.drone_topic}, "
                f"length={len(msg.poses)}"
            )
            return

        self.drone_pose = msg.poses[self.drone_index]

    def payload_callback(self, msg: PoseArray):
        if len(msg.poses) <= self.payload_index:
            self.get_logger().warn(
                f"Payload index {self.payload_index} out of range for {self.payload_topic}, "
                f"length={len(msg.poses)}"
            )
            return

        self.payload_pose = msg.poses[self.payload_index]

    def publish_state(self):
        if self.drone_pose is None or self.payload_pose is None:
            return

        now = self.get_clock().now()
        t = now.nanoseconds * 1e-9

        drone_p = np.array([
            self.drone_pose.position.x,
            self.drone_pose.position.y,
            self.drone_pose.position.z,
        ], dtype=float)

        payload_p = np.array([
            self.payload_pose.position.x,
            self.payload_pose.position.y,
            self.payload_pose.position.z,
        ], dtype=float)

        r = payload_p - drone_p
        cable_len = np.linalg.norm(r)

        if cable_len < 1e-6:
            return

        # Small-angle-ish convention:
        # payload below drone at rest gives r ~= [0, 0, -L]
        # phi/theta are lateral displacement divided by vertical drop.
        vertical_drop = max(-r[2], 1e-6)

        phi = r[0] / vertical_drop
        theta = r[1] / vertical_drop

        if self.last_time is None:
            phi_dot = 0.0
            theta_dot = 0.0
        else:
            dt = t - self.last_time
            if dt <= 0:
                phi_dot = 0.0
                theta_dot = 0.0
            else:
                phi_dot = (phi - self.last_phi) / dt
                theta_dot = (theta - self.last_theta) / dt

        self.last_phi = phi
        self.last_theta = theta
        self.last_time = t

        # Publish swing state
        swing_msg = MotionCaptureState()
        swing_msg.pose = Pose()
        swing_msg.twist = Twist()

        swing_msg.pose.position.x = float(phi)
        swing_msg.pose.position.y = float(theta)
        swing_msg.pose.position.z = float(cable_len)

        swing_msg.twist.angular.x = float(phi_dot)
        swing_msg.twist.angular.y = float(theta_dot)
        swing_msg.twist.angular.z = 0.0

        self.pendulum_pub.publish(swing_msg)

        # Publish payload world state
        payload_msg = MotionCaptureState()
        payload_msg.pose = Pose()
        payload_msg.twist = Twist()

        payload_msg.pose.position.x = float(payload_p[0])
        payload_msg.pose.position.y = float(payload_p[1])
        payload_msg.pose.position.z = float(payload_p[2])

        payload_msg.pose.orientation = self.payload_pose.orientation

        # No velocity here yet unless we also finite-difference payload world position.
        # Fine for now.
        self.payload_world_pub.publish(payload_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PendulumStatePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
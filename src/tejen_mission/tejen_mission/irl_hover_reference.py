#!/usr/bin/env python3
"""IRL commissioning hover-reference publisher.

This is the same simple 61-sample, 30 Hz minimum-jerk hover helper used during
earlier dynamic-planner commissioning, packaged here so the hardware bring-up is
self-contained.

Publishes /join_planner/reference at 30 Hz using the MPC's rolling
MultiDOFJointTrajectory contract (61 samples, 1/30 s spacing).

Behaviour:
- Before TAKEOFF: hold the first measured position.
- On receiving std_msgs/String "TAKEOFF" on /drone_command: smoothly rise from
  the captured start position to hover_z over takeoff_duration_s using a
  quintic minimum-jerk profile, then hold.
- This is ONLY a commissioning helper. It is not part of the dynamic planner.
"""

import math
import sys

import rclpy
from builtin_interfaces.msg import Duration
from interfaces.msg import MotionCaptureState
from rclpy.node import Node
from std_msgs.msg import String
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Transform, Twist


RATE_HZ = 30.0
DT = 1.0 / RATE_HZ
SAMPLES = 61


def duration_msg(seconds: float) -> Duration:
    seconds = max(0.0, float(seconds))
    sec = int(math.floor(seconds))
    nanosec = int(round((seconds - sec) * 1e9))
    if nanosec >= 1_000_000_000:
        sec += 1
        nanosec -= 1_000_000_000
    msg = Duration()
    msg.sec = sec
    msg.nanosec = nanosec
    return msg


def min_jerk(s: float):
    """Return h, dh/ds, d2h/ds2 for 10s^3 - 15s^4 + 6s^5."""
    s = min(1.0, max(0.0, s))
    h = 10.0*s**3 - 15.0*s**4 + 6.0*s**5
    hp = 30.0*s**2 - 60.0*s**3 + 30.0*s**4
    hpp = 60.0*s - 180.0*s**2 + 120.0*s**3
    return h, hp, hpp


class HoverReferencePublisher(Node):
    def __init__(self):
        super().__init__('irl_hover_reference')
        self.declare_parameter('state_topic', '/motion_capture_state')
        self.declare_parameter('reference_topic', '/join_planner/reference')
        self.declare_parameter('command_topic', '/drone_command')
        self.declare_parameter('hover_z', 1.20)
        self.declare_parameter('takeoff_duration_s', 4.0)
        self.declare_parameter('frame_id', 'map')

        self.state_topic = str(self.get_parameter('state_topic').value)
        self.reference_topic = str(self.get_parameter('reference_topic').value)
        self.command_topic = str(self.get_parameter('command_topic').value)
        self.hover_z = float(self.get_parameter('hover_z').value)
        self.takeoff_duration_s = max(0.5, float(self.get_parameter('takeoff_duration_s').value))
        self.frame_id = str(self.get_parameter('frame_id').value)

        self.start_position = None
        self.takeoff_start_time = None
        self.have_takeoff_command = False

        self.reference_pub = self.create_publisher(
            MultiDOFJointTrajectory, self.reference_topic, 10
        )
        self.create_subscription(
            MotionCaptureState, self.state_topic, self.state_callback, 10
        )
        self.create_subscription(String, self.command_topic, self.command_callback, 10)
        self.create_timer(DT, self.publish_reference)

        self.get_logger().info(
            f'IRL hover reference helper started: reference={self.reference_topic}, '
            f'hover_z={self.hover_z:.2f} m, takeoff_duration={self.takeoff_duration_s:.1f} s.'
        )

    def state_callback(self, msg: MotionCaptureState):
        if self.start_position is None:
            self.start_position = (
                float(msg.pose.position.x),
                float(msg.pose.position.y),
                float(msg.pose.position.z),
            )
            self.get_logger().info(
                'Captured initial position: '
                f'({self.start_position[0]:.3f}, {self.start_position[1]:.3f}, '
                f'{self.start_position[2]:.3f})'
            )

    def command_callback(self, msg: String):
        if msg.data.strip().upper() == 'TAKEOFF' and not self.have_takeoff_command:
            if self.start_position is None:
                self.get_logger().warn('TAKEOFF received before first state sample; waiting for state.')
                self.have_takeoff_command = True
                return
            self.have_takeoff_command = True
            self.takeoff_start_time = self.get_clock().now()
            self.get_logger().info('TAKEOFF received: starting smooth hover ramp.')

    def desired_state(self, t_ros_s: float):
        x0, y0, z0 = self.start_position
        if not self.have_takeoff_command:
            return (x0, y0, z0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)

        if self.takeoff_start_time is None:
            self.takeoff_start_time = self.get_clock().now()

        t0 = self.takeoff_start_time.nanoseconds * 1e-9
        tau = max(0.0, t_ros_s - t0)
        T = self.takeoff_duration_s
        dz = self.hover_z - z0

        if tau >= T:
            return (x0, y0, self.hover_z), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)

        s = tau / T
        h, hp, hpp = min_jerk(s)
        z = z0 + dz * h
        vz = dz * hp / T
        az = dz * hpp / (T*T)
        return (x0, y0, z), (0.0, 0.0, vz), (0.0, 0.0, az)

    def publish_reference(self):
        if self.start_position is None:
            return

        now = self.get_clock().now()
        now_s = now.nanoseconds * 1e-9

        msg = MultiDOFJointTrajectory()
        msg.header.stamp = now.to_msg() if hasattr(now, 'to_msg') else now.to_msg()  # ROS2 Python Time supports to_msg()
        msg.header.frame_id = self.frame_id
        msg.joint_names = ['drone_0']

        for k in range(SAMPLES):
            tk = now_s + k * DT
            p, v, a = self.desired_state(tk)

            point = MultiDOFJointTrajectoryPoint()
            transform = Transform()
            transform.translation.x = p[0]
            transform.translation.y = p[1]
            transform.translation.z = p[2]
            transform.rotation.w = 1.0
            point.transforms = [transform]

            vel = Twist()
            vel.linear.x = v[0]
            vel.linear.y = v[1]
            vel.linear.z = v[2]
            point.velocities = [vel]

            acc = Twist()
            acc.linear.x = a[0]
            acc.linear.y = a[1]
            acc.linear.z = a[2]
            point.accelerations = [acc]

            point.time_from_start = duration_msg(k * DT)
            msg.points.append(point)

        self.reference_pub.publish(msg)


def main():
    rclpy.init()
    node = HoverReferencePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

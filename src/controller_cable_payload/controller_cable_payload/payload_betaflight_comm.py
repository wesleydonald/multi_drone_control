"""
payload_betaflight_comm.py
--------------------------
Betaflight inner-loop interface for drones nested inside the lift_system model.

Differences from betaflight_communication.py:
  - Pose topic:  /model/{parent_model}/model/{drone_name}/pose  (nested path)
  - Motor topic: /{parent_model}/{drone_name}/gazebo/command/motor_speed
  - Both are parameterised so we can reuse this node for different worlds.

Parameters (ROS):
  drone_id      int   0-3
  drone_name    str   x3_drone0  (model name inside parent)
  parent_model  str   lift_system
  rates_d_val   float 70.0
  rates_f_val   float 670.0
  rates_g_val   float 0.5
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from actuator_msgs.msg import Actuators
from geometry_msgs.msg import PoseArray
from interfaces.msg import ELRSCommand
from tf_transformations import quaternion_multiply, quaternion_inverse, quaternion_matrix


class PayloadBetaflightComm(Node):
    def __init__(self):
        super().__init__('payload_betaflight_comm')

        self.declare_parameter('drone_id', 0)
        self.declare_parameter('drone_name', 'x3_drone0')
        self.declare_parameter('parent_model', 'lift_system')
        self.declare_parameter('rates_d_val', 70.0)
        self.declare_parameter('rates_f_val', 670.0)
        self.declare_parameter('rates_g_val', 0.5)

        self.drone_id = self.get_parameter('drone_id').value
        drone_name = self.get_parameter('drone_name').value
        parent = self.get_parameter('parent_model').value
        self.rates_d = self.get_parameter('rates_d_val').value
        self.rates_f = self.get_parameter('rates_f_val').value
        self.rates_g = self.get_parameter('rates_g_val').value

        pose_topic = f'/model/{parent}/model/{drone_name}/pose'
        motor_topic = f'/{parent}/{drone_name}/gazebo/command/motor_speed'

        self.get_logger().info(
            f'[BF{self.drone_id}] pose={pose_topic}  motor={motor_topic}')

        self.create_subscription(PoseArray, pose_topic, self._pose_cb, 10)
        self.create_subscription(
            ELRSCommand, f'/drone_{self.drone_id}/ELRSCommand', self._cmd_cb, 10)
        self.motor_pub = self.create_publisher(Actuators, motor_topic, 10)

        self.set_point = None
        self.last_pose = None
        self.last_orientation = None
        self.last_time = None

        self.kp = 0.5
        self.ki = 0.0
        self.kd = 0.0
        self.integral_error = None
        self.previous_error = None

    # ------------------------------------------------------------------
    def _betaflight_rates(self, x):
        x = max(-1.0, min(1.0, x))
        ax = math.sqrt(x * x + 1e-6)
        sgn = x / ax
        h = ax * (ax ** 5 * self.rates_g + ax * (1.0 - self.rates_g))
        j = self.rates_d * ax + (self.rates_f - self.rates_d) * h
        return sgn * j

    @staticmethod
    def _normalize_quat(x, y, z, w):
        if w < 0:
            return -x, -y, -z, -w
        return x, y, z, w

    # ------------------------------------------------------------------
    def _pose_cb(self, msg: PoseArray):
        if not msg.poses:
            return
        pos = msg.poses[-1].position
        ori = msg.poses[-1].orientation
        ori.x, ori.y, ori.z, ori.w = self._normalize_quat(ori.x, ori.y, ori.z, ori.w)

        now = self.get_clock().now().to_msg()

        if self.last_pose is None:
            self.last_pose = pos
            self.last_orientation = ori
            self.last_time = now
            return

        dt = (now.sec + now.nanosec * 1e-9) - (self.last_time.sec + self.last_time.nanosec * 1e-9)
        if dt <= 0:
            self.last_pose, self.last_orientation, self.last_time = pos, ori, now
            return

        q1 = [self.last_orientation.x, self.last_orientation.y,
              self.last_orientation.z, self.last_orientation.w]
        q2 = [ori.x, ori.y, ori.z, ori.w]
        q_rel = quaternion_multiply(q2, quaternion_inverse(q1))
        ang_vel = 2 * np.array([q_rel[0], q_rel[1], q_rel[2]]) / dt
        R = quaternion_matrix(q1)[:3, :3]
        ang_vel_body = R.T @ ang_vel
        ang_vel_body_deg = np.degrees(ang_vel_body)

        self.last_pose, self.last_orientation, self.last_time = pos, ori, now

        if self.set_point is not None:
            speeds = self._compute_motor_speeds(ang_vel_body_deg)
            msg_out = Actuators()
            msg_out.header.stamp = self.get_clock().now().to_msg()
            msg_out.velocity = speeds.tolist()
            self.motor_pub.publish(msg_out)

    def _compute_motor_speeds(self, ang_vel_deg):
        error = np.array([self.set_point[0], self.set_point[1], self.set_point[3]]) - ang_vel_deg

        if self.integral_error is None:
            self.integral_error = np.zeros(3)
        self.integral_error += error

        if self.previous_error is None:
            self.previous_error = np.zeros(3)
        derivative = self.kd * (error - self.previous_error)
        self.previous_error = error

        offset = self.kp * error + self.ki * self.integral_error + derivative
        throttle = self.set_point[2]

        speeds = np.array([
            throttle - offset[0] + offset[1] + offset[2],
            throttle - offset[0] - offset[1] - offset[2],
            throttle + offset[0] + offset[1] - offset[2],
            throttle + offset[0] - offset[1] + offset[2],
        ])
        return np.clip(speeds, 0, 4631)

    # ------------------------------------------------------------------
    def _cmd_cb(self, msg: ELRSCommand):
        roll_rate = self._betaflight_rates(msg.channel_0)
        pitch_rate = self._betaflight_rates(msg.channel_1)
        yaw_rate = self._betaflight_rates(-msg.channel_3)
        throttle = (msg.channel_2 + 1) / 2 * 4631

        if msg.armed and throttle < 0.05 * 4631:
            throttle = 0.05 * 4631

        self.set_point = [roll_rate, pitch_rate, throttle, yaw_rate]


def main(args=None):
    rclpy.init(args=args)
    node = PayloadBetaflightComm()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

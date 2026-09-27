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
  rates_d_val   float 100.0
  rates_f_val   float 100.0
  rates_g_val   float 0.5
  rate_source   str   'pose' (difference the gz poses) | 'imu' (the IMU gyro)
  imu_topic     str   /drone_{drone_id}/imu (sensor_msgs/Imu), used when rate_source is imu
"""

import math
import os
import numpy as np

from simulation_communication.rate_pid import RatePid, gyro_sample, integrate_active
import rclpy
from rclpy.node import Node
from actuator_msgs.msg import Actuators
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import Imu
from interfaces.msg import ELRSCommand
from tf_transformations import quaternion_multiply, quaternion_inverse, quaternion_matrix


class PayloadBetaflightComm(Node):
    def __init__(self):
        super().__init__('payload_betaflight_comm')

        self.declare_parameter('drone_id', 0)
        self.declare_parameter('drone_name', 'x3_drone0')
        self.declare_parameter('parent_model', 'lift_system')
        self.declare_parameter('rates_d_val', 100.0)
        self.declare_parameter('rates_f_val', 100.0)
        self.declare_parameter('rates_g_val', 0.5)

        self.drone_id = self.get_parameter('drone_id').value
        drone_name = self.get_parameter('drone_name').value
        parent = self.get_parameter('parent_model').value
        self.rates_d = self.get_parameter('rates_d_val').value
        self.rates_f = self.get_parameter('rates_f_val').value
        self.rates_g = self.get_parameter('rates_g_val').value

        pose_topic = f'/model/{parent}/model/{drone_name}/pose'
        # Motor topic uses only the drone model name (not the nested hierarchy path).
        # Verified with: gz topic --list | grep motor_speed
        motor_topic = f'/{drone_name}/gazebo/command/motor_speed'

        self.get_logger().info(
            f'[BF{self.drone_id}] pose  = {pose_topic}')
        self.get_logger().info(
            f'[BF{self.drone_id}] motor = {motor_topic}')

        # 'imu' runs the rate loop on the gyro, as a real Betaflight does; the pose path
        # differences unstamped poses, so bunched samples under load become rate spikes (G4).
        self.rate_source = str(self.declare_parameter('rate_source', 'pose').value)
        if self.rate_source not in ('pose', 'imu'):
            raise ValueError(f"rate_source must be 'pose' or 'imu', got {self.rate_source!r}")
        imu_topic = str(self.declare_parameter(
            'imu_topic', f'/drone_{self.drone_id}/imu').value)
        self._imu_t_prev = None
        self._imu_n = 0
        self._imu_time_base = None
        self._warned_no_imu = False
        if self.rate_source == 'imu':
            self.get_logger().info(f'[BF{self.drone_id}] rate  = gyro {imu_topic}')
            self.create_subscription(Imu, imu_topic, self._imu_cb, 10)
        else:
            self.create_subscription(PoseArray, pose_topic, self._pose_cb, 10)
        self.create_subscription(
            ELRSCommand, f'/drone_{self.drone_id}/ELRSCommand', self._cmd_cb, 10)
        self.motor_pub = self.create_publisher(Actuators, motor_topic, 10)

        self.set_point = None
        self.last_pose = None
        self.last_orientation = None
        self.last_time = None
        self._last_stamp_t = None
        self._win_t0, self._win_n, self._period = None, 0, None

        self._pid = RatePid(
            kp=float(self.declare_parameter('rate_kp', 0.5).value),
            ki=float(self.declare_parameter(
                'rate_ki', float(os.environ.get('SIM_RATE_KI', '5.0'))).value),
            kd=float(self.declare_parameter('rate_kd', 0.0).value),
            i_limit=float(self.declare_parameter('rate_i_limit', 200.0).value))
        self._i_min_u = float(self.declare_parameter('rate_i_min_u', 0.09).value)
        self.add_on_set_parameters_callback(self._on_rate_params)
        self._active = False

    # ------------------------------------------------------------------
    def _on_rate_params(self, params):
        """rate_kp / rate_ki settable live (M2 hand-over: his join flies the old loop, the
        I-term comes on just before our takeover); the integral restarts from zero."""
        from rcl_interfaces.msg import SetParametersResult
        for prm in params:
            if prm.name == 'rate_ki':
                self._pid.ki = float(prm.value)
                self._pid.integral[:] = 0.0
                self.get_logger().info(f'rate_ki -> {self._pid.ki}')
            elif prm.name == 'rate_kp':
                self._pid.kp = float(prm.value)
                self.get_logger().info(f'rate_kp -> {self._pid.kp}')
        return SetParametersResult(successful=True)

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
    def _pose_dt(self, msg):
        """dt between this pose sample and the previous one, for the body-rate difference.

        The ROS clock is /clock, throttled to 100 Hz since 2026-09-23, while poses arrive
        at 500 Hz: differencing against it gave dt = 0 for four of five pairs (skipped)
        and a rate ~5x too low on the fifth. Cables damped the tethered drones; a free
        drone tipped at 1.4 Hz under every outer controller (R0522-R0534; R0535 flew with
        the clock unthrottled). The bridged PoseArray carries no stamp (R0536), so the
        sample period is estimated from the clock over a window and every sample gets it.
        Returns None until the first period estimate exists."""
        st = msg.header.stamp
        now = self.get_clock().now().nanoseconds * 1e-9
        if int(st.sec) > 0 or int(st.nanosec) > 0:
            t = st.sec + st.nanosec * 1e-9
            dt = (t - self._last_stamp_t) if self._last_stamp_t is not None else None
            self._last_stamp_t = t
            return dt if (dt is not None and dt > 0.0) else None
        if self._win_t0 is None:
            self._win_t0, self._win_n = now, 0
        self._win_n += 1
        if now - self._win_t0 >= 0.25 and self._win_n >= 2:
            self._period = (now - self._win_t0) / self._win_n
            self._win_t0, self._win_n = now, 0
        return self._period

    def _pose_cb(self, msg: PoseArray):
        if not msg.poses:
            return
        pos = msg.poses[-1].position
        ori = msg.poses[-1].orientation
        ori.x, ori.y, ori.z, ori.w = self._normalize_quat(ori.x, ori.y, ori.z, ori.w)

        dt = self._pose_dt(msg)

        if self.last_pose is None or dt is None:
            self.last_pose = pos
            self.last_orientation = ori
            return

        q1 = [self.last_orientation.x, self.last_orientation.y,
              self.last_orientation.z, self.last_orientation.w]
        q2 = [ori.x, ori.y, ori.z, ori.w]
        q_rel = quaternion_multiply(q2, quaternion_inverse(q1))
        ang_vel = 2 * np.array([q_rel[0], q_rel[1], q_rel[2]]) / dt
        R = quaternion_matrix(q1)[:3, :3]
        ang_vel_body = R.T @ ang_vel
        ang_vel_body_deg = np.degrees(ang_vel_body)

        self.last_pose, self.last_orientation = pos, ori

        if self.set_point is not None:
            speeds = self._compute_motor_speeds(ang_vel_body_deg, dt)
            msg_out = Actuators()
            msg_out.header.stamp = self.get_clock().now().to_msg()
            msg_out.velocity = speeds.tolist()
            self.motor_pub.publish(msg_out)

    def _imu_cb(self, msg: Imu):
        """One rate-loop step per gyro sample, dt from the sensor stamps (sim time).

        Axes: gz gives angular_velocity in the sensor frame, which sits on base_link at
        identity, i.e. body FLU (the same IMU reads +9.81 on z at rest). The pose path's
        R^T * world rate is that frame too, so the axes map one to one, no sign flips."""
        st = msg.header.stamp
        if int(st.sec) > 0 or int(st.nanosec) > 0:
            t, base = st.sec + st.nanosec * 1e-9, 'IMU header stamps'
        else:
            t, base = self.get_clock().now().nanoseconds * 1e-9, 'node clock (IMU stamps are zero)'
        if base != self._imu_time_base:
            self._imu_time_base = base
            self.get_logger().info(f'[BF{self.drone_id}] gyro dt from {base}')
        w = msg.angular_velocity
        dt, ang_vel_deg, self._imu_t_prev = gyro_sample(t, self._imu_t_prev, (w.x, w.y, w.z))
        self._imu_n += 1
        if dt is None or self.set_point is None:
            return
        speeds = self._compute_motor_speeds(ang_vel_deg, dt)
        msg_out = Actuators()
        msg_out.header.stamp = self.get_clock().now().to_msg()
        msg_out.velocity = speeds.tolist()
        self.motor_pub.publish(msg_out)

    def _compute_motor_speeds(self, ang_vel_deg, dt):
        error = np.array([self.set_point[0], self.set_point[1], self.set_point[3]]) - ang_vel_deg
        offset = self._pid.step(error, dt, self._active)
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
        # Linear command -> THRUST (2026-09-24, Wesley's word). Gazebo rotor thrust is
        # motor-speed squared, so a speed proportional to the command gave a = 88.6 u^2
        # and one fixed linear gain matched a single operating point (hover height moved
        # with mass; free drones could not follow the creep arc, R0466/R0468). sqrt(u)
        # makes a = 88.6 u at every throttle, which is what the rig's Betaflight gives.
        # This is the node every OCP launch runs per drone (mpc_quad_load_launch:264).
        u = max(0.0, min(1.0, (msg.channel_2 + 1) / 2))
        throttle = math.sqrt(u) * 4631

        if msg.armed and throttle < 0.05 * 4631:
            throttle = 0.05 * 4631

        self.set_point = [roll_rate, pitch_rate, throttle, yaw_rate]
        self._active = integrate_active(msg.armed, u, self._i_min_u)
        if (self.rate_source == 'imu' and msg.armed and self._imu_n == 0
                and not self._warned_no_imu):
            self._warned_no_imu = True
            self.get_logger().warn(
                f'[BF{self.drone_id}] armed with rate_source imu but no gyro sample yet: '
                'no motor command is sent until one arrives (is the IMU bridged?)')


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

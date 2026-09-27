import os
import socket
import struct
import rclpy
import numpy as np

from simulation_communication.rate_pid import RatePid, gyro_sample, integrate_active
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Imu
from actuator_msgs.msg import Actuators
from interfaces.msg import ELRSCommand, Telemetry
from geometry_msgs.msg import Twist, PoseArray, Pose, PoseStamped
from tf_transformations import euler_from_quaternion, quaternion_multiply, quaternion_inverse, quaternion_matrix
from builtin_interfaces.msg import Time

class BetaflightInterfaceNode(Node):
    def __init__(self):
        super().__init__('betaflight_interface')



        self.declare_parameter('pose_topic', '/model/x3/pose')
        self.declare_parameter('pose_index', 7)
        self.declare_parameter('elrs_command_topic', 'ELRSCommand')
        self.declare_parameter('motor_command_topic', '/X3/gazebo/command/motor_speed')
        self.declare_parameter('publish_telemetry', False)
        self.declare_parameter('telemetry_topic', 'telemetry')
        self.declare_parameter('sim_battery_voltage', 16.0)
        self.pose_topic = str(self.get_parameter('pose_topic').value)
        self.pose_index = int(self.get_parameter('pose_index').value)
        self.elrs_command_topic = str(self.get_parameter('elrs_command_topic').value)
        self.motor_command_topic = str(self.get_parameter('motor_command_topic').value)
        self.publish_telemetry = bool(self.get_parameter('publish_telemetry').value)
        self.telemetry_topic = str(self.get_parameter('telemetry_topic').value)
        self.sim_battery_voltage = float(self.get_parameter('sim_battery_voltage').value)

        # 'imu' runs the rate loop on the gyro, as a real Betaflight does; the pose path
        # differences unstamped poses, so bunched samples under load become rate spikes
        # (multi_drone_control 2026-09-27, the load-dependent M2 ring rock).
        self.rate_source = str(self.declare_parameter('rate_source', 'pose').value)
        if self.rate_source not in ('pose', 'imu'):
            raise ValueError(f"rate_source must be 'pose' or 'imu', got {self.rate_source!r}")
        self.imu_topic = str(self.declare_parameter('imu_topic', 'imu').value)
        self._imu_t_prev = None
        self._imu_n = 0
        self._imu_time_base = None
        self._warned_no_imu = False
        if self.rate_source == 'imu':
            self.get_logger().info(f'rate loop on the gyro: {self.imu_topic}')
            self.subscription_imu = self.create_subscription(Imu, self.imu_topic, self._imu_cb, 10)
        else:
            self.subscription_motion_capture = self.create_subscription(PoseArray, self.pose_topic, self.pose_callback, 10)
        self.subscription_control = self.create_subscription(ELRSCommand, self.elrs_command_topic, self.controller_commands_callback, 10)
        self.publisher = self.create_publisher(Actuators, self.motor_command_topic, 10)
        self.telemetry_publisher = (
            self.create_publisher(Telemetry, self.telemetry_topic, 5)
            if self.publish_telemetry
            else None
        )
        self.telemetry_timer = (
            self.create_timer(0.10, self.publish_sim_telemetry)
            if self.telemetry_publisher is not None
            else None
        )

        self.set_point = None
        self.current_pose = None

        self._pid = RatePid(
            kp=float(self.declare_parameter('rate_kp', 0.5).value),
            ki=float(self.declare_parameter(
                'rate_ki', float(os.environ.get('SIM_RATE_KI', '5.0'))).value),
            kd=float(self.declare_parameter('rate_kd', 0.0).value),
            i_limit=float(self.declare_parameter('rate_i_limit', 200.0).value))
        self._i_min_u = float(self.declare_parameter('rate_i_min_u', 0.09).value)
        self.add_on_set_parameters_callback(self._on_rate_params)
        self._active = False
        self._dt = None
        
        self.last_pose = None
        self.last_orientation = None
        self.last_time = None
        self._last_stamp_t = None
        self._win_t0, self._win_n, self._period = None, 0, None

        self.databuffer = []
        
        # Betaflight rates parameters (settable via launch file)
        self.declare_parameter('rates_d_val', 100.0)
        self.declare_parameter('rates_f_val', 100.0)
        self.declare_parameter('rates_g_val', 0.5)
        self.rates_d_val = self.get_parameter('rates_d_val').get_parameter_value().double_value
        self.rates_f_val = self.get_parameter('rates_f_val').get_parameter_value().double_value
        self.rates_g_val = self.get_parameter('rates_g_val').get_parameter_value().double_value
        
    def publish_sim_telemetry(self):
        """Publish a minimal deterministic simulator telemetry heartbeat when enabled."""
        if self.telemetry_publisher is None:
            return
        msg = Telemetry()
        msg.battery_voltage = float(self.sim_battery_voltage)
        msg.battery_mah_used = 0
        msg.rssi = -40
        msg.mode = 'SIM'
        self.telemetry_publisher.publish(msg)

    def betaflight_rates(self, x):
        """
        Betaflight rates formula:
        h = x * (x^5 * g + x * (1-g))
        j = (d * x) + ((f-d) * h) for x in [-1, 1]
        The mapping from -1 to 0 is the inverted version of 0 to 1
        """
        import math

        x = max(-1.0, min(1.0, x))
        ax = math.sqrt(x*x + 1e-6)
        sgn = x / ax if ax > 0 else 0
        h_abs = ax * (pow(ax, 5) * self.rates_g_val + ax * (1.0 - self.rates_g_val))
        j_abs = self.rates_d_val * ax + (self.rates_f_val - self.rates_d_val) * h_abs
        j = sgn * j_abs
        
        return j
        
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

    def normalize_quaternion_positive_w(self, x, y, z, w):
        if w < 0:
            return -x, -y, -z, -w
        return x, y, z, w

    def _pose_dt(self, msg):
        """Sample period for the body-rate difference (multi_drone_control 2026-09-25): the
        pose stamp when the bridge sets one, else a 0.25 s clock-window estimate. The ROS
        clock may be throttled below the pose rate, which made most dt zero and the rate
        estimate ~5x low (a free drone was un-flyable, R0522-R0538)."""
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

    def pose_callback(self, msg):
        if self.pose_index < 0 or self.pose_index >= len(msg.poses):
            self.get_logger().warn(
                f'Pose index {self.pose_index} unavailable on {self.pose_topic} '
                f'(length={len(msg.poses)})'
            )
            return
        current_position = msg.poses[self.pose_index].position
        current_orientation = msg.poses[self.pose_index].orientation
       
        current_orientation.x, current_orientation.y, current_orientation.z, current_orientation.w = self.normalize_quaternion_positive_w(
            current_orientation.x, current_orientation.y, current_orientation.z, current_orientation.w
        )
        
        dt = self._pose_dt(msg)
        self._dt = dt

        if self.last_pose is None or dt is None:
            self.last_pose = current_position
            self.last_orientation = current_orientation
            return


        q1 = [self.last_orientation.x, self.last_orientation.y, self.last_orientation.z, self.last_orientation.w]
        q2 = [current_orientation.x, current_orientation.y, current_orientation.z, current_orientation.w]
        q_relative = quaternion_multiply(q2, quaternion_inverse(q1))
        angular_velocity = 2 * np.array([q_relative[0], q_relative[1], q_relative[2]]) / dt 
        rotation_matrix = quaternion_matrix(q1)[:3, :3]

        angular_velocity_body = np.dot(rotation_matrix.T, angular_velocity)

      
        self.last_pose = current_position
        self.last_orientation = current_orientation


        angular_velocity_body_deg = np.degrees(angular_velocity_body)
        if self.set_point is not None:
            motor_speeds = self.calculate_motor_speeds(angular_velocity_body_deg)

            actuator_msg = Actuators()
            actuator_msg.header.stamp = self.get_clock().now().to_msg()
            actuator_msg.velocity = motor_speeds.tolist() 
            
            self.publisher.publish(actuator_msg)

            




    def _imu_cb(self, msg):
        """
        One rate-loop step per gyro sample, dt from the sensor stamps (sim time).

        The sensor sits on X3/base_link at identity, so its angular_velocity is the body FLU
        rate the pose path computes as R^T * world rate: same axes, no sign flips.
        """
        st = msg.header.stamp
        if int(st.sec) > 0 or int(st.nanosec) > 0:
            t, base = st.sec + st.nanosec * 1e-9, 'IMU header stamps'
        else:
            t, base = self.get_clock().now().nanoseconds * 1e-9, 'node clock (IMU stamps are zero)'
        if base != self._imu_time_base:
            self._imu_time_base = base
            self.get_logger().info(f'gyro dt from {base}')
        w = msg.angular_velocity
        dt, angular_velocity_body_deg, self._imu_t_prev = gyro_sample(
            t, self._imu_t_prev, (w.x, w.y, w.z))
        self._imu_n += 1
        if dt is None or self.set_point is None:
            return
        self._dt = dt
        motor_speeds = self.calculate_motor_speeds(angular_velocity_body_deg)
        actuator_msg = Actuators()
        actuator_msg.header.stamp = self.get_clock().now().to_msg()
        actuator_msg.velocity = motor_speeds.tolist()
        self.publisher.publish(actuator_msg)

    def calculate_motor_speeds(self, angular_velocity_body_deg):
        error = np.array([self.set_point[0], self.set_point[1], self.set_point[3]]) - angular_velocity_body_deg
        offset = self._pid.step(error, self._dt, self._active)

        throttle = self.set_point[2]  

        motor_speeds = np.zeros(4)
        motor_speeds[0] = throttle - offset[0] + offset[1] + offset[2]
        motor_speeds[1] = throttle - offset[0] - offset[1] - offset[2]
        motor_speeds[2] = throttle + offset[0] + offset[1] - offset[2]
        motor_speeds[3] = throttle + offset[0] - offset[1] + offset[2]

        motor_speeds = np.clip(motor_speeds, 0, 4631)

        return motor_speeds


    def controller_commands_callback(self, msg):
        roll_rate = self.betaflight_rates(msg.channel_0)
        pitch_rate = self.betaflight_rates(msg.channel_1)
        yaw_rate = self.betaflight_rates(-msg.channel_3)
        
        # linear command -> thrust (multi_drone_control 2026-09-24): rotor thrust is speed^2
        u = max(0.0, min(1.0, (msg.channel_2 + 1.0) * 0.5))
        throttle = (u ** 0.5) * 4631.0
        
        #if msg.armed and throttle < (0.05 * 4631):
        #    throttle = 0.0 * 4631
        
        self.set_point = [roll_rate, pitch_rate, throttle, yaw_rate]
        self._active = integrate_active(msg.armed, u, self._i_min_u)
        if (self.rate_source == 'imu' and msg.armed and self._imu_n == 0
                and not self._warned_no_imu):
            self._warned_no_imu = True
            self.get_logger().warn(
                f'armed with rate_source imu but no gyro sample on {self.imu_topic} yet: '
                'no motor command is sent until one arrives (is the IMU bridged?)')





def main(args=None):
    rclpy.init(args=args)
    node = BetaflightInterfaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

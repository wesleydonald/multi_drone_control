import socket
import struct
import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Imu
from actuator_msgs.msg import Actuators
from interfaces.msg import MotionCaptureState, ELRSCommand, Telemetry  # Import the Telemetry message
from actuator_msgs.msg import Actuators
from geometry_msgs.msg import Twist, PoseArray, Pose, PoseStamped
from tf_transformations import euler_from_quaternion, quaternion_multiply, quaternion_inverse, quaternion_matrix
from builtin_interfaces.msg import Time

class BetaflightInterfaceNode(Node):
    def __init__(self):
        super().__init__('betaflight_interface')

        self.declare_parameter('drone_id', 0)
        self.drone_id = self.get_parameter('drone_id').get_parameter_value().integer_value
        
        self.subscription_motion_capture = self.create_subscription(PoseArray, f'/model/x3_drone{self.drone_id}/pose', self.pose_callback, 10)
        self.subscription_control = self.create_subscription(ELRSCommand, f'/drone_{self.drone_id}/ELRSCommand', self.controller_commands_callback, 10)
        self.publisher = self.create_publisher(Actuators, f'/x3_drone{self.drone_id}/gazebo/command/motor_speed', 10)

        self.declare_parameter('pose_index', 5)
        self.pose_index = self.get_parameter('pose_index').get_parameter_value().integer_value

        self.set_point = None
        self.current_pose = None

        self.kp = 0.5
        self.ki = 0.0
        self.kd = 0.0
        
        self.last_pose = None
        self.last_orientation = None
        self.last_time = None

        self.databuffer = []
        
        # Betaflight rates parameters (settable via launch file)
        self.declare_parameter('rates_d_val', 70.0)
        self.declare_parameter('rates_f_val', 670.0)
        self.declare_parameter('rates_g_val', 0.5)
        self.rates_d_val = self.get_parameter('rates_d_val').get_parameter_value().double_value
        self.rates_f_val = self.get_parameter('rates_f_val').get_parameter_value().double_value
        self.rates_g_val = self.get_parameter('rates_g_val').get_parameter_value().double_value
        
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
        
    def normalize_quaternion_positive_w(self, x, y, z, w):
        if w < 0:
            return -x, -y, -z, -w
        return x, y, z, w

    def pose_callback(self, msg):
        current_position = msg.poses[self.pose_index].position
        current_orientation = msg.poses[self.pose_index].orientation
       
        current_orientation.x, current_orientation.y, current_orientation.z, current_orientation.w = self.normalize_quaternion_positive_w(
            current_orientation.x, current_orientation.y, current_orientation.z, current_orientation.w
        )
        
        current_time = self.get_clock().now().to_msg()

        if self.last_pose is None:
            self.last_pose = current_position
            self.last_orientation = current_orientation
            self.last_time = current_time
            return

        dt = (current_time.sec + current_time.nanosec * 1e-9) - (self.last_time.sec + self.last_time.nanosec * 1e-9)
        
        if dt <= 0:
            self.last_pose = current_position
            self.last_orientation = current_orientation
            self.last_time = current_time
            return


        q1 = [self.last_orientation.x, self.last_orientation.y, self.last_orientation.z, self.last_orientation.w]
        q2 = [current_orientation.x, current_orientation.y, current_orientation.z, current_orientation.w]
        q_relative = quaternion_multiply(q2, quaternion_inverse(q1))
        angular_velocity = 2 * np.array([q_relative[0], q_relative[1], q_relative[2]]) / dt 
        rotation_matrix = quaternion_matrix(q1)[:3, :3]

        angular_velocity_body = np.dot(rotation_matrix.T, angular_velocity)

      
        self.last_pose = current_position
        self.last_orientation = current_orientation
        self.last_time = current_time


        angular_velocity_body_deg = np.degrees(angular_velocity_body)
        if self.set_point is not None:
            motor_speeds = self.calculate_motor_speeds(angular_velocity_body_deg)

            actuator_msg = Actuators()
            actuator_msg.header.stamp = self.get_clock().now().to_msg()
            actuator_msg.velocity = motor_speeds.tolist() 
            
            self.publisher.publish(actuator_msg)

            




    def calculate_motor_speeds(self, angular_velocity_body_deg):
        error = [self.set_point[0],self.set_point[1],self.set_point[3]] - angular_velocity_body_deg
        proportional = self.kp * error

        if not hasattr(self, 'integral_error'):
            self.integral_error = np.zeros_like(error)  
        self.integral_error += error 
        integral = self.ki * self.integral_error

        if not hasattr(self, 'previous_error'):
            self.previous_error = np.zeros_like(error)
        derivative = self.kd * (error - self.previous_error)
        self.previous_error = error 

        offset = proportional + integral + derivative

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
        
        throttle = (msg.channel_2 + 1) / 2 * 4631
        
        if msg.armed and throttle < (0.05 * 4631):
            throttle = 0.05 * 4631
        
        self.set_point = [roll_rate, pitch_rate, throttle, yaw_rate]





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

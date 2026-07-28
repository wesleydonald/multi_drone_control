#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from interfaces.msg import InvertedPendulumStates
from geometry_msgs.msg import PoseStamped, TwistStamped, PoseArray, Pose, Twist
from sensor_msgs.msg import Imu
from tf_transformations import euler_from_quaternion, quaternion_multiply, quaternion_inverse, quaternion_matrix
import numpy as np
from scipy.spatial.transform import Rotation as R
from tf_transformations import euler_from_quaternion, quaternion_multiply, quaternion_inverse, quaternion_matrix
import math 
class pendulumStateListener(Node):
    def __init__(self):
        super().__init__('pendulum_state_listener')
        #self.subscription = self.create_subscription(Imu,'/imu',self.listener_callback,10)
        
        self.worldPoseSub_ = self.create_subscription(PoseArray, '/world/quadcopter/dynamic_pose/info', self.listener_callback, 10)
        #self.worldPoseSub_ = self.create_subscription(PoseArray, '/model/pendulum/pose', self.listener_callback, 10)
        #self.worldPoseSub_ = self.create_subscription(PoseArray, '/model/x3/pose', self.listener_callback, 10)
        #self.subscription = self.create_subscription(Imu,'/imu',self.listener_callback,10)
        self.publisher = self.create_publisher(InvertedPendulumStates, '/pendulum_state_publisher', 10)
        #self.publisher = self.create_publisher(inverted_pendulum_states, '/motion_capture_state', 10)
        #self.pose_publisher = self.create_publisher(PoseStamped, '/pendulum_pose', 10)
        #self.vel_publisher = self.create_publisher(TwistStamped, '/pendulum_vel', 10)
        self.last_pose = None
        self.last_orientation = None
        self.last_time = None

        self.databuffer = []
    def normalize_quaternion_positive_w(self, x, y, z, w):
        """Normalize quaternion and ensure w is positive."""
        # If w is negative, negate the quaternion
        if w < 0:
            return -x, -y, -z, -w
        return x, y, z, w

    def quaternion_to_euler(self, w, x, y, z):
        # Roll (x-axis rotation)
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)

        # Pitch (y-axis rotation)
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)

        # Yaw (z-axis rotation)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)

        return roll, pitch, yaw

    def listener_callback(self, msg):
        print("START")
        # Extract current pose and time
        #"x3::X3/pendulum::imu_sensor"

        #if msg[-1]["name"] != "X3/pendulum":
        #    raise Exception("Pendulum link not found")

        currentQuad_position = msg.poses[0].position
        currentQuad_orientation = msg.poses[0].orientation
        #base_orientation = msg.poses[-1].orientation
        current_position = msg.poses[-1].position
        current_orientation = msg.poses[-1].orientation
        # pendulum index is 5
        #current_position = msg.poses[6].position
        #current_orientation = msg.poses[6].orientation
       
        # Ensure w is positive
        current_orientation.x, current_orientation.y, current_orientation.z, current_orientation.w = self.normalize_quaternion_positive_w(
            current_orientation.x, current_orientation.y, current_orientation.z, current_orientation.w
        )
        currentQuad_orientation.x, currentQuad_orientation.y, currentQuad_orientation.z, currentQuad_orientation.w = self.normalize_quaternion_positive_w(
            currentQuad_orientation.x, currentQuad_orientation.y, currentQuad_orientation.z, currentQuad_orientation.w
        )
        

        r, p, yaw = self.quaternion_to_euler(currentQuad_orientation.w, currentQuad_orientation.x, currentQuad_orientation.y, currentQuad_orientation.z)
        rotate = R.from_euler('zyx', [yaw, p, r], degrees=False)
        rotationMatrix = rotate.as_matrix()
       
        [[a], [b], [eta]] = rotationMatrix@np.array([[current_position.x],[current_position.y],[current_position.z]])
        #[[a_dot], [b_dot], [eta_dot]] = rotationMatrix@np.array([[a_dot], [b_dot], [eta_dot]])
        current_position.x, current_position.y, current_position.z = a, b, eta
        
        current_time = msg.header.stamp

        if self.last_pose is None:
            # Initialize the last pose, orientation, and time
            self.last_pose = current_position
            self.last_orientation = current_orientation
            self.last_time = current_time
            return

        # Calculate time difference (dt)
        dt = (current_time.sec + current_time.nanosec * 1e-9) - (self.last_time.sec + self.last_time.nanosec * 1e-9)

        print(f"Time difference (dt): {dt:.6f} seconds")

        if dt <= 0:

            print("Time difference is non-positive, skipping callback.")
            # Skip this callback if the time difference is non-positive
            self.last_pose = current_position
            self.last_orientation = current_orientation
            self.last_time = current_time
            return


        # Calculate linear velocity in the world frame
        dx = current_position.x - self.last_pose.x
        dy = current_position.y - self.last_pose.y
        dz = current_position.z - self.last_pose.z
        linear_velocity_world = np.array([dx / dt, dy / dt, dz / dt])

        # Calculate angular velocity
        q1 = [self.last_orientation.x, self.last_orientation.y, self.last_orientation.z, self.last_orientation.w]
        q2 = [current_orientation.x, current_orientation.y, current_orientation.z, current_orientation.w]
        q_relative = quaternion_multiply(q2, quaternion_inverse(q1))  # Relative rotation
        angular_velocity = 2 * np.array([q_relative[0], q_relative[1], q_relative[2]]) / dt  # Angular velocity
        q_current = [current_orientation.x, current_orientation.y, current_orientation.z, current_orientation.w]
        rotation_matrix = quaternion_matrix(q1)[:3, :3]  # Extract 3x3 rotation part

        # Transform  velocity from world frame to body frame
        angular_velocity_body = np.dot(rotation_matrix.T, angular_velocity)

        # Publish MotionCaptureState
        mcs = InvertedPendulumStates()
        mcs.pose = Pose()
        mcs.pose.position.x = round(float(current_position.x), 3)
        mcs.pose.position.y = round(float(current_position.y), 3)
        mcs.pose.position.z = round(float(current_position.z), 3)
        mcs.pose.orientation.x = round(float(current_orientation.x), 3)
        mcs.pose.orientation.y = round(float(current_orientation.y), 3)
        mcs.pose.orientation.z = round(float(current_orientation.z), 3)
        mcs.pose.orientation.w = round(float(current_orientation.w), 3)

        mcs.twist = Twist()
        mcs.twist.linear.x = round(float(linear_velocity_world[0]), 3)  # Use body frame velocity
        mcs.twist.linear.y = round(float(linear_velocity_world[1]), 3)  # Use body frame velocity
        mcs.twist.linear.z = round(float(linear_velocity_world[2]), 3)  # Use body frame velocity
        mcs.twist.angular.x = round(float(angular_velocity_body[0]), 3)
        mcs.twist.angular.y = round(float(angular_velocity_body[1]), 3)
        mcs.twist.angular.z = round(float(angular_velocity_body[2]), 3)

        # Print mcs in a nicely formatted and normalized spacing
        print(f"Pose: Position(x={mcs.pose.position.x:7.3f}, y={mcs.pose.position.y:7.3f}, z={mcs.pose.position.z:7.3f}), ")
        print(f"Orientation(x={mcs.pose.orientation.x:7.3f}, y={mcs.pose.orientation.y:7.3f}, z={mcs.pose.orientation.z:7.3f}, w={mcs.pose.orientation.w:7.3f})")
        print(f"Twist: Linear(x={mcs.twist.linear.x:7.3f}, y={mcs.twist.linear.y:7.3f}, z={mcs.twist.linear.z:7.3f}), ")
        print(f"Angular(x={mcs.twist.angular.x:7.3f}, y={mcs.twist.angular.y:7.3f}, z={mcs.twist.angular.z:7.3f})")

        '''current_orientation = msg.orientation
        angular_velocity = msg.angular_velocity
        linear_acceleration = msg.linear_acceleration'''

       
        # Ensure w is positive
        '''current_orientation.x, current_orientation.y, current_orientation.z, current_orientation.w = self.normalize_quaternion_positive_w(
            current_orientation.x, current_orientation.y, current_orientation.z, current_orientation.w
        )
        
        

        # Publish MotionCaptureState
        mcs = InvertedPendulumStates()
    
        mcs.orientation_x = round(float(current_orientation.x), 3)
        mcs.orientation_y = round(float(current_orientation.y), 3)
        mcs.orientation_z= round(float(current_orientation.z), 3)
        mcs.orientation_w = round(float(current_orientation.w), 3)
        
        mcs.angular_velocity_x = round(float(angular_velocity.x), 3)
        mcs.angular_velocity_y = round(float(angular_velocity.y), 3)
        mcs.angular_velocity_z = round(float(angular_velocity.z), 3)

        mcs.linear_acceleration_x = round(float(linear_acceleration.x), 3)
        mcs.linear_acceleration_y = round(float(linear_acceleration.y), 3)
        mcs.linear_acceleration_z = round(float(linear_acceleration.z), 3)'''
       

        

        simulate_delay = False  # Set to True to simulate delay
        if simulate_delay == True:

            self.databuffer.append(mcs)
            if len(self.databuffer) > 5 :
                mcs = self.databuffer.pop(0)
                self.publisher.publish(mcs)
        else:
            self.publisher.publish(mcs)

        
        self.last_orientation = current_orientation
        print("END")

        # Update last pose, orientation, and time
        self.last_pose = current_position
        self.last_orientation = current_orientation
        self.last_time = current_time
   




def main(args=None):
    rclpy.init(args=args)
    node = pendulumStateListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


'''
#!/usr/bin/env python3

import numpy as np

# Patch np.float to avoid issues with deprecated alias
if not hasattr(np, 'float'):
    np.float = float

import rclpy
from rclpy.node import Node
from actuator_msgs.msg import Actuators
from geometry_msgs.msg import Twist, PoseArray, Pose, PoseStamped
from tf_transformations import euler_from_quaternion, quaternion_multiply, quaternion_inverse, quaternion_matrix
from builtin_interfaces.msg import Time
import signal
import pandas as pd
import time
import math
import matplotlib.pyplot as plt
from interfaces.msg import MotionCaptureState 
from interfaces.msg import InvertedPendulumStates

from scipy.spatial.transform import Rotation as R

class pendulumStateListener(Node):
    def __init__(self):
        super().__init__('pendulum_state_listener')
        #self.worldPoseSub_ = self.create_subscription(PoseArray, '/model/pendulum/pose', self.worldPoseCallback, 10)
        #self.worldPoseSub_ = self.create_subscription(PoseArray, '/model/x3/pose', self.worldPoseCallback, 10)
        #self.worldPoseSub_ = self.create_subscription(PoseArray, '/world/quadcopter/dynamic_pose/info', self.listener_callback, 10)
        self.worldPoseSub_ = self.create_subscription(PoseArray, '/model/pendulum/pose', self.worldPoseCallback, 10)
        self.publisher = self.create_publisher(InvertedPendulumStates,'/pendulum_state_publisher', 10)
        #self.pose_publisher = self.create_publisher(PoseStamped, '/rviz_pose', 10)

        self.last_pose = None
        self.last_orientation = None
        self.last_time = None

        self.databuffer = []
        
    def normalize_quaternion_positive_w(self, x, y, z, w):
        """Normalize quaternion and ensure w is positive."""
        # If w is negative, negate the quaternion
        if w < 0:
            return -x, -y, -z, -w
        return x, y, z, w

    def quaternion_to_euler(self, w, x, y, z):
        # Roll (x-axis rotation)
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)

        # Pitch (y-axis rotation)
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)

        # Yaw (z-axis rotation)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)

        return roll, pitch, yaw

    def worldPoseCallback(self, msg):
        print("START")
        # Extract current pose and time # 9 for omnicopter, 5 for quadcopter
        # when using world_large.sdf quadcopter is 5
        # when using world_inv_pen.sdf quadcopter is 6 and pendulum is 5
        #current_position = msg.poses[5].position
        #current_orientation = msg.poses[5].orientation

        #current_position = msg.poses[0].position
        #current_orientation = msg.poses[0].orientation
        #current_position = msg.poses[5].position
        #current_orientation = msg.poses[5].orientation
        #current_position = msg.poses[-1].position
        #current_orientation = msg.poses[-1].orientation
        current_position = msg.poses[0].position
        current_orientation = msg.poses[0].orientation
        base_orientation = msg.poses[-1].orientation
        
        # Ensure w is positive
        current_orientation.x, current_orientation.y, current_orientation.z, current_orientation.w = self.normalize_quaternion_positive_w(
            current_orientation.x, current_orientation.y, current_orientation.z, current_orientation.w
        )

        base_orientation.x, base_orientation.y, base_orientation.z, base_orientation.w = self.normalize_quaternion_positive_w(
            base_orientation.x, base_orientation.y, base_orientation.z, base_orientation.w
        )
        r, p, yaw = self.quaternion_to_euler(base_orientation.x, base_orientation.y, base_orientation.z, base_orientation.w)
        rotate = R.from_euler('zyx', [yaw, p, r], degrees=False)
        rotationMatrix = rotate.as_matrix()
       
        [[a], [b], [eta]] = rotationMatrix@np.array([[current_position.x],[current_position.y],[current_position.z]])
        #[[a_dot], [b_dot], [eta_dot]] = rotationMatrix@np.array([[a_dot], [b_dot], [eta_dot]])
        current_position.x, current_position.y, current_position.z = a, b, eta
        current_time = self.get_clock().now().to_msg()

        if self.last_pose is None:
            # Initialize the last pose, orientation, and time
            self.last_pose = current_position
            self.last_orientation = current_orientation
            self.last_time = current_time
            return

        # Calculate time difference (dt)
        dt = (current_time.sec + current_time.nanosec * 1e-9) - (self.last_time.sec + self.last_time.nanosec * 1e-9)

        print(f"Time difference (dt): {dt:.6f} seconds")

        if dt <= 0:

            print("Time difference is non-positive, skipping callback.")
            # Skip this callback if the time difference is non-positive
            self.last_pose = current_position
            self.last_orientation = current_orientation
            self.last_time = current_time
            return


        # Calculate linear velocity in the world frame
        dx = current_position.x - self.last_pose.x
        dy = current_position.y - self.last_pose.y
        dz = current_position.z - self.last_pose.z
        linear_velocity_world = np.array([dx / dt, dy / dt, dz / dt])

        # Calculate angular velocity
        q1 = [self.last_orientation.x, self.last_orientation.y, self.last_orientation.z, self.last_orientation.w]
        q2 = [current_orientation.x, current_orientation.y, current_orientation.z, current_orientation.w]
        q_relative = quaternion_multiply(q2, quaternion_inverse(q1))  # Relative rotation
        angular_velocity = 2 * np.array([q_relative[0], q_relative[1], q_relative[2]]) / dt  # Angular velocity
        q_current = [current_orientation.x, current_orientation.y, current_orientation.z, current_orientation.w]
        rotation_matrix = quaternion_matrix(q1)[:3, :3]  # Extract 3x3 rotation part

        # Transform  velocity from world frame to body frame
        angular_velocity_body = np.dot(rotation_matrix.T, angular_velocity)

        # Publish MotionCaptureState
        mcs = InvertedPendulumStates()
        mcs.pose = Pose()
        mcs.pose.position.x = float(current_position.x)
        mcs.pose.position.y = float(current_position.y)
        mcs.pose.position.z = float(current_position.z)
        mcs.pose.orientation.x = float(current_orientation.x)
        mcs.pose.orientation.y = float(current_orientation.y)
        mcs.pose.orientation.z = float(current_orientation.z)
        mcs.pose.orientation.w = float(current_orientation.w)

        mcs.twist = Twist()
        mcs.twist.linear.x = float(linear_velocity_world[0])
        mcs.twist.linear.y = float(linear_velocity_world[1])
        mcs.twist.linear.z = float(linear_velocity_world[2])
        mcs.twist.angular.x = float(angular_velocity_body[0])
        mcs.twist.angular.y = float(angular_velocity_body[1])
        mcs.twist.angular.z = float(angular_velocity_body[2])

        # Print mcs in a nicely formatted and normalized spacing
        #print(f"Pose: Position(x={mcs.pose.position.x:7.3f}, y={mcs.pose.position.y:7.3f}, z={mcs.pose.position.z:7.3f}), ")
        #print(f"Orientation(x={mcs.pose.orientation.x:7.3f}, y={mcs.pose.orientation.y:7.3f}, z={mcs.pose.orientation.z:7.3f}, w={mcs.pose.orientation.w:7.3f})")
        #print(f"Twist: Linear(x={mcs.twist.linear.x:7.3f}, y={mcs.twist.linear.y:7.3f}, z={mcs.twist.linear.z:7.3f}), ")
        #print(f"Angular(x={mcs.twist.angular.x:7.3f}, y={mcs.twist.angular.y:7.3f}, z={mcs.twist.angular.z:7.3f})")


        self.publisher.publish(mcs)


        # Update last pose, orientation, and time
        self.last_pose = current_position
        self.last_orientation = current_orientation
        self.last_time = current_time

def main(args=None):
    rclpy.init(args=args)
    pentaVerify = pendulumStateListener()
    rclpy.spin(pentaVerify)
    pentaVerify.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
'''
#!/usr/bin/env python3

import numpy as np

# Patch np.float to avoid issues with deprecated alias
if not hasattr(np, 'float'):
    np.float = float

import rclpy
from rclpy.node import Node
from actuator_msgs.msg import Actuators
from geometry_msgs.msg import Twist, PoseArray, Pose, PoseStamped
from tf_transformations import (
    euler_from_quaternion,
    quaternion_from_euler,
    quaternion_multiply,
    quaternion_inverse,
    quaternion_matrix,
)
from builtin_interfaces.msg import Time
import signal
import pandas as pd
import time
import math
import matplotlib.pyplot as plt
from interfaces.msg import MotionCaptureState 

class PentaVerify(Node):
    def __init__(self):
        super().__init__('penta_verify')
        
        # Declare parameter for target object ID
        # self.declare_parameter('target_object_id', 7)
        # self.declare_parameter('target_object_id', 5)
        self.declare_parameter('target_object_id', 7)
        self.target_object_id = self.get_parameter('target_object_id').get_parameter_value().integer_value

        # Optional fixed orientation error applied only to the emulated mocap
        # measurement. Gazebo's true vehicle pose is left unchanged. The bias is
        # right-multiplied so it represents a mocap rigid-body frame that is fixed
        # at a small angle relative to the drone's true body/thrust frame.
        self.declare_parameter('enable_orientation_bias', False)
        self.declare_parameter('orientation_bias_roll_deg', -1.3)
        self.declare_parameter('orientation_bias_pitch_deg', -1.3)
        self.declare_parameter('orientation_bias_yaw_deg', 0.0)

        self.enable_orientation_bias = bool(
            self.get_parameter('enable_orientation_bias').value
        )
        self.orientation_bias_roll_deg = float(
            self.get_parameter('orientation_bias_roll_deg').value
        )
        self.orientation_bias_pitch_deg = float(
            self.get_parameter('orientation_bias_pitch_deg').value
        )
        self.orientation_bias_yaw_deg = float(
            self.get_parameter('orientation_bias_yaw_deg').value
        )
        self.orientation_bias_quaternion = quaternion_from_euler(
            math.radians(self.orientation_bias_roll_deg),
            math.radians(self.orientation_bias_pitch_deg),
            math.radians(self.orientation_bias_yaw_deg),
        )
        
        self.get_logger().info(f'Motion capture emulator tracking object ID: {self.target_object_id}')
        self.get_logger().info(
            'Mocap orientation bias: '
            f'enabled={self.enable_orientation_bias}, '
            f'roll={self.orientation_bias_roll_deg:.3f} deg, '
            f'pitch={self.orientation_bias_pitch_deg:.3f} deg, '
            f'yaw={self.orientation_bias_yaw_deg:.3f} deg'
        )
        
        self.worldPoseSub_ = self.create_subscription(PoseArray, '/model/x3/pose', self.worldPoseCallback, 10)
        self.publisher = self.create_publisher(MotionCaptureState, '/motion_capture_state', 10)
        self.pose_publisher = self.create_publisher(PoseStamped, '/rviz_pose', 10)

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

    def worldPoseCallback(self, msg):
        print("START")
        # Extract current pose and time using the parameterized object ID
        # when using world_large.sdf quadcopter is 5
        # when using world_inv_pen.sdf quadcopter is 6 and pendulum is 5
        if len(msg.poses) <= self.target_object_id:
            self.get_logger().warn(f'Target object ID {self.target_object_id} not available in pose array (length: {len(msg.poses)})')
            return
            
        self.get_logger().info(f"PoseArray length: {len(msg.poses)}")

        for i, pose in enumerate(msg.poses):
            self.get_logger().info(
                f"[{i}] x={pose.position.x:.3f}, "
                f"y={pose.position.y:.3f}, "
                f"z={pose.position.z:.3f}, "
                f"qw={pose.orientation.w:.3f}, "
                f"qx={pose.orientation.x:.3f}, "
                f"qy={pose.orientation.y:.3f}, "
                f"qz={pose.orientation.z:.3f}"
            )
        current_position = msg.poses[self.target_object_id].position
        true_orientation = msg.poses[self.target_object_id].orientation

        # Work on a copy so the incoming Gazebo PoseArray remains untouched. ROS and
        # tf_transformations use quaternion order [x, y, z, w].
        q_true = np.array([
            float(true_orientation.x),
            float(true_orientation.y),
            float(true_orientation.z),
            float(true_orientation.w),
        ], dtype=float)
        q_norm = float(np.linalg.norm(q_true))
        if q_norm <= 1e-12:
            self.get_logger().warn('Received near-zero Gazebo quaternion; skipping pose sample.')
            return
        q_true /= q_norm

        if self.enable_orientation_bias:
            # Body-fixed rigid-body-frame error: q_reported = q_true * q_bias.
            q_reported = quaternion_multiply(
                q_true, self.orientation_bias_quaternion
            )
        else:
            q_reported = q_true

        q_reported = np.asarray(q_reported, dtype=float)
        q_reported /= np.linalg.norm(q_reported)
        q_reported = np.array(
            self.normalize_quaternion_positive_w(*q_reported),
            dtype=float,
        )

        current_orientation = Pose().orientation
        current_orientation.x = float(q_reported[0])
        current_orientation.y = float(q_reported[1])
        current_orientation.z = float(q_reported[2])
        current_orientation.w = float(q_reported[3])
        
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
        mcs = MotionCaptureState()
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
    pentaVerify = PentaVerify()
    rclpy.spin(pentaVerify)
    pentaVerify.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

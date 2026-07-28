import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist
from std_msgs.msg import Header
from interfaces.msg import MotionCaptureState  # Import the new message type
from geometry_msgs.msg import Twist, PoseArray, Pose, PoseStamped
from tf_transformations import quaternion_multiply, quaternion_inverse, quaternion_matrix
import socket
from dataclasses import dataclass
from typing import Tuple, Optional
import sys
import time
import numpy as np
from collections import deque  # Import deque for the rolling average filter
import csv
from pathlib import Path
from datetime import datetime
import scipy.signal  # Import scipy.signal for Butterworth filter
import re 

# Rigid-body IDs from the motion-capture UDP stream.
QUAD_ID = "7"
MAGNET_ID = "8"

# Offset from the quad mocap/body origin to the cable attachment point,
# expressed in the quad body frame. Set to [0, 0, 0] if ID 7 is already
# located at the cable attachment point.
ATTACH_OFFSET_BODY = np.array([0.0, 0.0, -0.03], dtype=float)

# Used only as a sanity check. The published angles use the measured cable
# vector length at each instant.
NOMINAL_CABLE_LENGTH_M = 0.50

# Ignore pendulum estimates if the most recent quad and payload packets are
# too far apart in time.
MAX_PAIR_AGE_S = 0.20

@dataclass
class ObjectData:
    id: str
    position: Tuple[float, float, float]
    rotation: Tuple[float, float, float, float]
    velocity: Tuple[float, float, float]
    angular_velocity: Tuple[float, float, float]
    timestamp: float

class ParseData():
    def __init__(self):
        self.last_pose = np.array([0, 0, 0])
        self.last_orientation = np.array([0, 0, 0, 1])
        self.last_time = time.time()

        '''self.last_pen_pose = np.array([0, 0, 0])
        self.last_pen_orientation = np.array([0, 0, 0, 1])
        self.last_pen_time = time.time()'''
        # Rolling average buffers for position, orientation, velocity, and angular velocity

        self.velocity_buffers = {
            'x': deque(maxlen=8),
            'y': deque(maxlen=8),
            'z': deque(maxlen=8)
        }
        self.angular_velocity_buffers = {
            'x': deque(maxlen=8),
            'y': deque(maxlen=8),
            'z': deque(maxlen=8)
        }

        # Rolling average buffers for velocitykpz, kiz, kdz = 15.0, 10.0, 10.0 
        self.rolling_velocity_buffers = {
            'x': deque(maxlen=8),
            'y': deque(maxlen=8),
            'z': deque(maxlen=8)
        }

        # Low-pass filter buffers for velocity
        self.low_pass_velocity_buffers = {
            'x': deque(maxlen=8),
            'y': deque(maxlen=8),
            'z': deque(maxlen=8)
        }

        # Butterworth filter buffers for velocity
        self.butter_velocity_buffers = {
            'x': deque(maxlen=8),
            'y': deque(maxlen=8),
            'z': deque(maxlen=8)
        }

        # Rolling average buffers for angular velocity
        self.rolling_angular_velocity_buffers = {
            'x': deque(maxlen=8),
            'y': deque(maxlen=8),
            'z': deque(maxlen=8)
        }

        # Low-pass filter buffers for angular velocity
        self.low_pass_angular_velocity_buffers = {
            'x': deque(maxlen=8),
            'y': deque(maxlen=8),
            'z': deque(maxlen=8)
        }

        # Butterworth filter buffers for angular velocity
        self.butter_angular_velocity_buffers = {
            'x': deque(maxlen=8),
            'y': deque(maxlen=8),
            'z': deque(maxlen=8)
        }

        # Low-pass filter parameters
        self.alpha = 0.2  # Smoothing factor (0 < alpha <= 1)


        # Butterworth filter parameters
        self.butter_cutoff = 0.1  # Cutoff frequency (normalized, 0 < butter_cutoff < 0.5)
        self.butter_order = 2    # Order of the Butterworth filter


        # Precompute Butterworth filter coefficients
        self.butter_b, self.butter_a = scipy.signal.butter(
            self.butter_order, self.butter_cutoff, btype='low', analog=False
        )

        # Initialize filtered values for velocity and angular velocity
        self.filtered_velocity = np.array([0.0, 0.0, 0.0])
        self.filtered_angular_velocity = np.array([0.0, 0.0, 0.0])
       


    '''def clean_message(self, message: str) -> str:
        message = message.replace('-(', '|').replace(')-', '|')
        message = message.replace('(', '').replace(')', '')
        message = message.strip()
        message = message.replace('||', '|')
        return message'''
    
    def normalize_quaternion_positive_w(self, x, y, z, w):
        """Normalize quaternion and ensure w is positive."""
        print(f"Normalizing quaternion: {x}, {y}, {z}, {w}")
        if w < 0:
            print(f"Quaternion w is negative, negating all components.")
            return -x, -y, -z, -w
        print(f"Quaternion w is positive, no change needed.")
        return x, y, z, w

    def apply_rolling_average(self, buffers, *values):
        """Apply a rolling average filter to the given components."""
        for key, value in zip(buffers.keys(), values):
            buffers[key].append(value)

        averages = tuple(sum(buffers[key]) / len(buffers[key]) for key in buffers.keys())
        return averages

    def apply_low_pass_filter(self, buffers, *values):
        """Apply a low-pass filter to the given components."""
        filtered_values = []
        for key, value in zip(buffers.keys(), values):
            if len(buffers[key]) == 0:
                # Initialize the buffer with the first value
                buffers[key].append(value)
            else:
                # Apply low-pass filter
                filtered_value = self.alpha * value + (1 - self.alpha) * buffers[key][-1]
                buffers[key].append(filtered_value)
            filtered_values.append(buffers[key][-1])
        return tuple(filtered_values)

    def apply_butterworth_filter(self, buffers, *values):
        """Apply a Butterworth filter to the given components."""
        filtered_values = []
        for key, value in zip(buffers.keys(), values):
            buffers[key].append(value)
            if len(buffers[key]) < len(self.butter_b):
                # Not enough data to apply the filter yet
                filtered_values.append(value)
            else:
                # Apply the Butterworth filter
                filtered_value = scipy.signal.lfilter(
                    self.butter_b, self.butter_a, list(buffers[key])
                )[-1]
                filtered_values.append(filtered_value)
        return tuple(filtered_values)

    def parse_packet(self, data: str) -> Optional[ObjectData]:
        try:
            if not data or '|' not in data:
                print("Invalid data format, skipping packet.")
                return None

            parts = [p.strip() for p in data.split('|') if p.strip()]
            if len(parts) != 3:
                print("Invalid parts length.")
                return None

            obj_id, pos_str, rot_str = parts

            
            # Parse position
            pos_parts = [p.strip() for p in pos_str.split(',')]
            if len(pos_parts) != 3:
                print("Invalid position format.")
                return None
            r_x, r_y, r_z = map(float, pos_parts)
            
            # Apply rolling average filter to position
            #x, y, z = self.apply_rolling_average(self.position_buffers, x, y, z)
            
            # Parse orientation
            rot_parts = [p.strip() for p in rot_str.split(',')]
            if len(rot_parts) != 4:
                print("Invalid rotation format.")
                return None
            r_qx, r_qy, r_qz, r_qw = map(float, rot_parts)
            
            # Apply rolling average filter to orientation
            #qx, qy, qz, qw = self.apply_rolling_average(self.orientation_buffers, qx, qy, qz, qw)

            current_time = time.time()
            dt = current_time - self.last_time
            self.last_time = current_time
            

            if dt <= 0:
                self.last_pose = np.array([r_x, r_y, r_z])
                self.last_orientation = np.array([r_qx, r_qy, r_qz, r_qw])
                self.last_time = current_time
                return None

            # Calculate linear velocity in the world frame
            dx, dy, dz = r_x - self.last_pose[0], r_y - self.last_pose[1], r_z - self.last_pose[2]
            linear_velocity_world = np.array([dx / dt, dy / dt, dz / dt])


            # Apply low-pass filter to velocity
            low_pass_vx, low_pass_vy, low_pass_vz = self.apply_low_pass_filter(
                self.low_pass_velocity_buffers, *linear_velocity_world
            )


            # Calculate angular velocity
            q1 = self.last_orientation
            q2 = np.array([r_qx, r_qy,r_qz, r_qw])
            q_relative = quaternion_multiply(q2, quaternion_inverse(q1))  # Relative rotation
            angular_velocity = 2 * np.array([q_relative[0], q_relative[1], q_relative[2]]) / dt  # Angular velocity
            rotation_matrix = quaternion_matrix(q2)[:3, :3]  # Extract 3x3 rotation part

            # Transform velocity from world frame to body frame
            angular_velocity_body = np.dot(rotation_matrix.T, angular_velocity)

        
            low_pass_wx, low_pass_wy, low_pass_wz = self.apply_low_pass_filter(
                self.low_pass_angular_velocity_buffers, *angular_velocity_body
            )


            # Update last pose, orientation, and time
            self.last_pose = np.array([r_x, r_y, r_z])
            self.last_orientation = np.array([r_qx, r_qy, r_qz, r_qw])


            # Round angular velocity (rad/s) to 1 decimal place
            low_pass_wx = round(low_pass_wx, 1)
            low_pass_wy = round(low_pass_wy, 1)
            low_pass_wz = round(low_pass_wz, 1)

            # Convert angular velocity from rad/s to deg/s for logging
            deg_wx = np.degrees(low_pass_wx)
            deg_wy = np.degrees(low_pass_wy)
            deg_wz = np.degrees(low_pass_wz)
            # Format with constant width, including sign and padding
            # print(
            #     f"Angular velocity (body frame): "
            #     f"wx={low_pass_wx:+08.4f} rad/s ({deg_wx:+08.2f}°/s), "
            #     f"wy={low_pass_wy:+08.4f} rad/s ({deg_wy:+08.2f}°/s), "
            #     f"wz={low_pass_wz:+08.4f} rad/s ({deg_wz:+08.2f}°/s)"
            # )




            
            return ObjectData(
                id=obj_id,
                position=(r_x, r_y, r_z),
                rotation=(r_qw, r_qx, r_qy, r_qz),
                velocity=(low_pass_vx, low_pass_vy, low_pass_vz),  # Use Butterworth-filtered velocity
                angular_velocity=(low_pass_wx, low_pass_wy, low_pass_wz),  # Use Butterworth-filtered angular velocity
                timestamp=current_time
            )
        except Exception:
            return None
        
    

class MotionCapturePublisher(Node):
    def __init__(self):
        super().__init__('udp_to_pose_node')
        
        # ROS2 Publisher
        self.publisher = self.create_publisher(MotionCaptureState, 'motion_capture_state', 10)
        self.pose_publisher = self.create_publisher(PoseStamped, '/rviz_pose', 10)
        self.pen_publisher = self.create_publisher(MotionCaptureState, '/pendulum_swing_state', 10)
        self.magnet_tip_publisher = self.create_publisher(PoseStamped, '/magnet_tip_pose', 10)
        # UDP Setup
        #self.HOST = "192.168.1.105"
        #self.PORT = 1511
        self.HOST = "192.168.0.87" #"192.168.1.105"
        self.PORT = 1511
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.HOST, self.PORT))
        
        self.get_logger().info(f'Listening for UDP on {self.HOST}:{self.PORT}')


        # CSV logging setup
        self.csv_file = None
        self.csv_writer = None
        self.csv_path = None
        self.csv_row_count = 0
        self.setup_csv_logger()

        # Store latest raw quad and magnet samples so that the pendulum
        # angle state can be computed from their relative positions.
        self.latest_quad_data: Optional[ObjectData] = None
        self.latest_magnet_data: Optional[ObjectData] = None
        self.last_pendulum_angles: Optional[np.ndarray] = None
        self.last_pendulum_time: Optional[float] = None
        self.last_published_magnet_time: Optional[float] = None
        self.pendulum_rate_buffers = {
            'phi_dot': deque(maxlen=8),
            'theta_dot': deque(maxlen=8),
        }

    def write_csv_row(self, object_type: str, obj_data: ObjectData):
        ros_now = self.get_clock().now()
        ros_time_sec = ros_now.nanoseconds * 1e-9
        unix_time_sec = time.time()

        row = [
            ros_time_sec,
            unix_time_sec,
            object_type,
            obj_data.id,
            obj_data.position[0],
            obj_data.position[1],
            obj_data.position[2],
            obj_data.rotation[0],
            obj_data.rotation[1],
            obj_data.rotation[2],
            obj_data.rotation[3],
            obj_data.velocity[0],
            obj_data.velocity[1],
            obj_data.velocity[2],
            obj_data.angular_velocity[0],
            obj_data.angular_velocity[1],
            obj_data.angular_velocity[2],
        ]

        self.csv_writer.writerow(row)
        self.csv_file.flush()

    def setup_csv_logger(self):
        """Create a timestamped CSV log file for incoming motion-capture packets."""
        log_dir = Path.cwd() / "logs" / "motion_capture_udp"
        log_dir.mkdir(parents=True, exist_ok=True)

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_path = log_dir / f"motion_capture_udp_{timestamp}.csv"

        self.csv_file = open(self.csv_path, mode="w", newline="")
        self.csv_writer = csv.writer(self.csv_file)

        self.csv_writer.writerow([
            "ros_time_sec",
            "unix_time_sec",
            "object_type",
            "object_id",
            "x",
            "y",
            "z",
            "qw",
            "qx",
            "qy",
            "qz",
            "vx",
            "vy",
            "vz",
            "wx",
            "wy",
            "wz",
        ])
        self.csv_file.flush()

        self.get_logger().info(f"Logging motion-capture data to {self.csv_path}")

    def log_object_data(self, object_type: str, obj_data: ObjectData, msg: MotionCaptureState):
        """Append one parsed object sample to the CSV log."""
        if self.csv_writer is None:
            return

        ros_time_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        self.csv_writer.writerow([
            f"{ros_time_sec:.9f}",
            f"{time.time():.9f}",
            object_type,
            obj_data.id,
            f"{obj_data.position[0]:.9f}",
            f"{obj_data.position[1]:.9f}",
            f"{obj_data.position[2]:.9f}",
            f"{obj_data.rotation[0]:.9f}",
            f"{obj_data.rotation[1]:.9f}",
            f"{obj_data.rotation[2]:.9f}",
            f"{obj_data.rotation[3]:.9f}",
            f"{obj_data.velocity[0]:.9f}",
            f"{obj_data.velocity[1]:.9f}",
            f"{obj_data.velocity[2]:.9f}",
            f"{obj_data.angular_velocity[0]:.9f}",
            f"{obj_data.angular_velocity[1]:.9f}",
            f"{obj_data.angular_velocity[2]:.9f}",
        ])

        self.csv_row_count += 1
        if self.csv_row_count % 25 == 0:
            self.csv_file.flush()

    def close_csv_logger(self):
        """Flush and close the CSV log file on shutdown."""
        if self.csv_file is not None:
            self.csv_file.flush()
            self.csv_file.close()
            self.get_logger().info(f"Closed motion-capture CSV log: {self.csv_path}")
            self.csv_file = None
            self.csv_writer = None

        

    def clean_message(self, message: str) -> str:
        message = message.replace('-(', '|').replace(')-', '|')
        message = message.replace('(', '').replace(')', '')
        message = message.strip()
        message = message.replace('||', '|')
        return message
    
    def create_motion_capture_state_msg(self, obj_data: ObjectData) -> MotionCaptureState:
        msg = MotionCaptureState()
        
        # Set header
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = obj_data.id
        
        # Set pose
        msg.pose = Pose()
        msg.pose.position.x = obj_data.position[0]
        msg.pose.position.y = obj_data.position[1]
        msg.pose.position.z = obj_data.position[2]
        msg.pose.orientation.w = obj_data.rotation[0]
        msg.pose.orientation.x = obj_data.rotation[1]
        msg.pose.orientation.y = obj_data.rotation[2]
        msg.pose.orientation.z = obj_data.rotation[3]
        
        # Set twist (velocity)
        msg.twist = Twist()
        msg.twist.linear.x = obj_data.velocity[0]
        msg.twist.linear.y = obj_data.velocity[1]
        msg.twist.linear.z = obj_data.velocity[2]
        msg.twist.angular.x = obj_data.angular_velocity[0]
        msg.twist.angular.y = obj_data.angular_velocity[1]
        msg.twist.angular.z = obj_data.angular_velocity[2]
        
        return msg

    def create_pose_stamped_msg(self, obj_data: ObjectData, frame_id: str = "map") -> PoseStamped:
        """Create a world-frame PoseStamped from one mocap rigid body."""
        msg = PoseStamped()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id

        msg.pose = Pose()
        msg.pose.position.x = obj_data.position[0]
        msg.pose.position.y = obj_data.position[1]
        msg.pose.position.z = obj_data.position[2]
        msg.pose.orientation.w = obj_data.rotation[0]
        msg.pose.orientation.x = obj_data.rotation[1]
        msg.pose.orientation.y = obj_data.rotation[2]
        msg.pose.orientation.z = obj_data.rotation[3]

        return msg


    def object_id_matches(self, obj_id: str, target_id: str) -> bool:
        """Return True when obj_id contains target_id as a standalone number."""
        return re.search(rf'(?<!\d){re.escape(target_id)}(?!\d)', obj_id.strip()) is not None

    def quad_attachment_position(self, quad_data: ObjectData) -> np.ndarray:
        """Compute cable attachment position from the quad mocap pose."""
        p_quad = np.asarray(quad_data.position, dtype=float)

        # ObjectData stores quaternion as [qw, qx, qy, qz], while
        # tf_transformations.quaternion_matrix expects [qx, qy, qz, qw].
        qw, qx, qy, qz = quad_data.rotation
        rot_mat = quaternion_matrix([qx, qy, qz, qw])[:3, :3]

        return p_quad + rot_mat.dot(ATTACH_OFFSET_BODY)

    def compute_pendulum_state(self) -> Optional[Tuple[float, float, float, float, float]]:
        """
        Compute [phi, theta, phi_dot, theta_dot] from latest quad/magnet samples.

        Convention used here:
            r = p_magnet - p_attach
            phi   = atan2(r_x, -r_z)
            theta = atan2(r_y, -r_z)

        This gives phi ~= r_x / L and theta ~= r_y / L for small angles,
        matching the small-angle payload state used by the controller.
        """
        if self.latest_quad_data is None or self.latest_magnet_data is None:
            return None

        now = time.time()
        oldest_sample_time = min(self.latest_quad_data.timestamp, self.latest_magnet_data.timestamp)
        if now - oldest_sample_time > MAX_PAIR_AGE_S:
            self.get_logger().warn(
                f"Skipping pendulum estimate: quad/magnet samples too stale "
                f"({now - oldest_sample_time:.3f} s old)."
            )
            return None

        p_attach = self.quad_attachment_position(self.latest_quad_data)
        p_magnet = np.asarray(self.latest_magnet_data.position, dtype=float)
        # p_magnet = p_attach + np.array([0.0, 0.0, -NOMINAL_CABLE_LENGTH_M], dtype=float)
        r = p_magnet - p_attach
        cable_length = float(np.linalg.norm(r))

        if cable_length < 1e-3:
            self.get_logger().warnpendulum("Skipping pendulum estimate: cable vector is too small.")
            return None

        # True geometric angles. For small angles these are approximately
        # lateral displacement divided by cable length.
        phi = float(np.arctan2(r[0], -r[2]))
        theta = float(np.arctan2(r[1], -r[2]))
        angles = np.array([phi, theta], dtype=float)

        sample_time = self.latest_magnet_data.timestamp
        if self.last_pendulum_angles is None or self.last_pendulum_time is None:
            phi_dot = 0.0
            theta_dot = 0.0
        else:
            dt = sample_time - self.last_pendulum_time
            if dt <= 1e-6:
                phi_dot = 0.0
                theta_dot = 0.0
            else:
                raw_rates = (angles - self.last_pendulum_angles) / dt
                phi_dot, theta_dot = self.apply_pendulum_rate_filter(raw_rates[0], raw_rates[1])

        self.last_pendulum_angles = angles
        self.last_pendulum_time = sample_time

        return phi, theta, float(phi_dot), float(theta_dot), cable_length

    def apply_pendulum_rate_filter(self, phi_dot: float, theta_dot: float) -> Tuple[float, float]:
        """Small rolling average filter for differentiated pendulum rates."""
        self.pendulum_rate_buffers['phi_dot'].append(phi_dot)
        self.pendulum_rate_buffers['theta_dot'].append(theta_dot)

        return (
            float(np.mean(self.pendulum_rate_buffers['phi_dot'])),
            float(np.mean(self.pendulum_rate_buffers['theta_dot'])),
        )

    def create_pendulum_state_msg(self, phi: float, theta: float, phi_dot: float, theta_dot: float, cable_length: float) -> MotionCaptureState:
        """Publish controller-friendly pendulum state on /pendulum_swing_state."""
        msg = MotionCaptureState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "pendulum_state_from_mocap_7_8"

        msg.pose = Pose()
        msg.pose.position.x = float(phi)
        msg.pose.position.y = float(theta)
        msg.pose.position.z = float(cable_length)
        msg.pose.orientation.w = 1.0
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0

        msg.twist = Twist()

        # Controller callback expects phi_dot/theta_dot in angular.x/angular.y.
        # Keep linear.x/linear.y populated as well for backwards compatibility
        # with older debugging scripts that read the rates from linear velocity.
        msg.twist.linear.x = float(phi_dot)
        msg.twist.linear.y = float(theta_dot)
        msg.twist.linear.z = 0.0
        msg.twist.angular.x = float(phi_dot)
        msg.twist.angular.y = float(theta_dot)
        msg.twist.angular.z = 0.0

        return msg

    def maybe_publish_pendulum_state(self):
        """Publish one pendulum-angle estimate when a new magnet sample arrives."""
        if self.latest_magnet_data is None:
            return

        if self.last_published_magnet_time == self.latest_magnet_data.timestamp:
            return

        pendulum_state = self.compute_pendulum_state()
        if pendulum_state is None:
            return

        phi, theta, phi_dot, theta_dot, cable_length = pendulum_state
        msg = self.create_pendulum_state_msg(phi, theta, phi_dot, theta_dot, cable_length)
        self.pen_publisher.publish(msg)
        self.last_published_magnet_time = self.latest_magnet_data.timestamp

        self.get_logger().info(
            f"Published pendulum state: phi={phi:+.4f}, theta={theta:+.4f}, "
            f"phi_dot={phi_dot:+.4f}, theta_dot={theta_dot:+.4f}, L={cable_length:.4f}"
        )


    def run(self):
        self.penParseData = ParseData()
        self.quadParseData = ParseData()
        try:
            while rclpy.ok():
                data, _ = self.sock.recvfrom(255)
                message = data.decode().strip()
                
                data = self.clean_message(message)
                try:
                    if not data or '|' not in data:
                        print("Invalid data format, skipping packet.")
                        return None

                    parts = [p.strip() for p in data.split('|') if p.strip()]
                    if len(parts) != 3:
                        print("Invalid parts length.")
                        return None

                    obj_id, pos_str, rot_str = parts
                except Exception:
                    print("Error")
                    continue

                # self.get_logger().info(f"Received obj_id: {obj_id}")
                if self.object_id_matches(obj_id, MAGNET_ID):
                    obj_data = self.penParseData.parse_packet(data)

                    if obj_data is None:
                        continue

                    self.latest_magnet_data = obj_data
                    self.write_csv_row("magnet_raw", obj_data)

                    # ID 8 is the electromagnet/magnet tip. Publish its world-frame
                    # pose directly for the planner.
                    magnet_tip_msg = self.create_pose_stamped_msg(obj_data)
                    self.magnet_tip_publisher.publish(magnet_tip_msg)

                    # Publish controller-friendly [phi, theta, phi_dot, theta_dot]
                    # on /pendulum_swing_state once both ID 7 and ID 8 exist.
                    self.maybe_publish_pendulum_state()

                elif self.object_id_matches(obj_id, QUAD_ID):
                    obj_data = self.quadParseData.parse_packet(data)

                    if obj_data is None:
                        continue

                    self.latest_quad_data = obj_data
                    self.write_csv_row("quad_raw", obj_data)

                    motion_capture_msg = self.create_motion_capture_state_msg(obj_data)
                    self.publisher.publish(motion_capture_msg)

                else:
                    self.get_logger().warn(f"Ignoring unrecognised mocap object ID: {obj_id}")

                
        except KeyboardInterrupt:
            self.get_logger().info('Shutting down...')
        finally:
            self.close_csv_logger()
            self.sock.close()

def main(args=None):
    rclpy.init(args=args)
    node = MotionCapturePublisher()
    
    try:
        node.run()
    except Exception as e:
        node.get_logger().error(f'Error: {str(e)}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
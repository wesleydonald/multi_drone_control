import rclpy
import signal
import sys
import numpy as np
import copy
import math
import os
import threading
import time
from rclpy.node import Node
from datetime import datetime
from scipy.spatial.transform import Rotation as R
from scipy.linalg import cholesky

# Local imports
from .acados import (generate_ocp_controller, set_initial_guess, 
                     warm_start_from_previous_solution, 
                     set_trajectory_reference_aligned, update_ocp_parameters)
from .trajectories import circle_trajectory # Default backup
from utility_objects.visualization import TrajectoryVisualizer
from utility_objects.data_logger import DataLogger
from utility_objects.callback_manager import CallbackManager
from interfaces.msg import MotionCaptureState, ELRSCommand, Telemetry, ControlApplied

# ROS2 Imports
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32MultiArray, Int32

POSE_TIMEOUT_THRESHOLD = 0.25  # seconds
USE_MOTION_CAPTURE = True
FREQUENCY_HZ = 30.0
DT = 1.0 / FREQUENCY_HZ
LOGGING_NAME = 'controller_graffiti'

class Controller(Node):
    def __init__(self):
        super().__init__('controller')

        # General Settings
        self.cb = CallbackManager(self, USE_MOTION_CAPTURE)
        self.trajectory_visualizer = TrajectoryVisualizer(self, frame_id="map")

        # Initial default state (Hover at 0,0,0)
        self.traj = np.zeros((13, 2))
        self.traj[3, :] = 1.0 # qw
        self.trajectory_ready = False
        
        self.timer = self.create_timer(DT, self.control_loop)
        self.step_counter = 0
        self.steps = self.traj.shape[1] - 1

        # Control State
        self.armed = False
        self.takeoff_requested = True
        self.shutdown_requested = False
        self.target_speed = 0.1  # m/s

        # MPC settings
        self.N = 20
        self.skip_steps = 3
        self.first_solve = True
        self.ocp, self.sim_integrator = generate_ocp_controller()

        # Graffiti Subscriber
        self.stroke_sub = self.create_subscription(
            Float32MultiArray, 
            'drone_graffiti_strokes', 
            self.graffiti_callback, 
            10)

        # ORB-Slam interface 
        self.orb_slam_pose = [0,0,0,1,0,0,0,0,0,0,0,0,0]
        self.orb_slam_state_subscription_ = self.create_subscription(
            MotionCaptureState, '/orb_slam_state', self.orb_slam_state_callback, 10)
        
        # UKF settings
        self.est_params = np.array([42.0, 0.2, 0.12, 100.0, 100.0, 0.0])
        self.alpha, self.beta, self.kappa = 0.1, 2, 0
        self.x_est = np.array([0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                               self.est_params[0], self.est_params[1], self.est_params[2], 
                               self.est_params[3], self.est_params[4], self.est_params[5]])
        self.P = np.diag([0.1]*19)
        self.Q = np.diag([1e-4, 1e-4, 1e-4, 1e-5, 1e-5, 1e-5, 1e-5, 1e-3, 1e-3, 1e-3, 1e-3, 1e-3, 1e-3, 
                          1e-4, 1e-5, 1e-5, 1, 1, 0.1])
        self.R = np.diag([0.05]*13)

        qos1 = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        self.pub_ctrl_applied = self.create_publisher(ControlApplied, '/control_applied', qos1)

        log_headers = [
            'step', 'timestamp', 'u0','u1','u2','u3','u0_rate', 'u1_rate', 'u2_rate', 'u3_rate',
            'pose_x', 'pose_y', 'pose_z', 'pose_qw', 'pose_qx', 'pose_qy', 'pose_qz',
            'est_pose_x', 'est_pose_y', 'est_pose_z', 'traj_x_ref', 'traj_y_ref', 'traj_z_ref'
        ]
        self.data_logger = DataLogger(LOGGING_NAME, "graffiti_trajectory", log_headers)
        self.control_history = []

    def graffiti_callback(self, msg: Float32MultiArray):
        """Processes incoming stroke data into a continuous drone trajectory."""
        self.get_logger().info("New Graffiti Message Received.")
        data = list(msg.data)
        if not data: return
            
        num_strokes = int(data.pop(0))
        all_strokes = []

        # 1. Decode and Interpolate Strokes
        for _ in range(num_strokes):
            num_pts = int(data.pop(0))
            pts = []
            for _ in range(num_pts):
                pts.append(np.array([data.pop(0), data.pop(0), 1.5]))
            all_strokes.append(self.interpolate_path(pts, self.target_speed))

        full_path = []

        # 2. Add Takeoff Ramp from current position to first stroke
        if self.current_pose is not None:
            curr_pos = np.array([self.current_pose[0], self.current_pose[1], self.current_pose[2]])
            takeoff_ramp = self.interpolate_path([curr_pos, all_strokes[0][0]], self.target_speed)
            for pt in takeoff_ramp:
                full_path.append(self.format_state(pt))

        # 3. Build continuous path with travel segments
        for i, stroke in enumerate(all_strokes):
            for pt in stroke:
                full_path.append(self.format_state(pt))
            
            if i < len(all_strokes) - 1:
                travel = self.interpolate_path([all_strokes[i][-1], all_strokes[i+1][0]], self.target_speed)
                for pt in travel: full_path.append(self.format_state(pt))

        # 4. Add Return to Origin and Land
        last_pt = np.array([full_path[-1][0], full_path[-1][1], 1.5])
        home_high = np.array([0.0, 0.0, 1.5])
        home_land = np.array([0.0, 0.0, 0.0])
        
        for pt in self.interpolate_path([last_pt, home_high], self.target_speed):
            full_path.append(self.format_state(pt))
        for pt in self.interpolate_path([home_high, home_land], self.target_speed * 0.5):
            full_path.append(self.format_state(pt))

        # 5. Finalize Trajectory
        self.traj = np.array(full_path).T
        self.steps = self.traj.shape[1] - 1
        self.step_counter = 0
        self.first_solve = True
        self.trajectory_ready = True
        self.trajectory_visualizer.publish_all_visualizations(self.traj)
        self.get_logger().info(f"Trajectory Loaded: {self.steps} waypoints.")

    def interpolate_path(self, points, speed):
        interp = []
        for i in range(len(points) - 1):
            p1, p2 = points[i], points[i+1]
            dist = np.linalg.norm(p2 - p1)
            num_steps = max(1, int((dist / speed) * FREQUENCY_HZ))
            for s in range(num_steps):
                interp.append(p1 + (s / num_steps) * (p2 - p1))
        interp.append(points[-1])
        return interp

    def format_state(self, pos):
        s = np.zeros(13); s[0:3] = pos; s[3] = 1.0
        return s

    def orb_slam_state_callback(self, msg: MotionCaptureState):
        p, o, lv, av = msg.pose.position, msg.pose.orientation, msg.twist.linear, msg.twist.angular
        self.orb_slam_pose = np.round(np.array([p.x, p.y, p.z, o.w, o.x, o.y, o.z, lv.x, lv.y, lv.z, av.x, av.y, av.z]), 3)
        if not USE_MOTION_CAPTURE:
            self.current_pose = self.orb_slam_pose
            self.last_pose_update_time = time.time()

    def control_loop(self):
        if self.shutdown_requested or not self.trajectory_ready:
            return

        if self.armed and (time.time() - self.last_pose_update_time) > POSE_TIMEOUT_THRESHOLD:
            self.cb.disarm(ELRSCommand(armed=False, channel_2=-1.0))
            return 

        if self.armed and self.current_pose is not None:
            if self.step_counter + (self.N * self.skip_steps) > self.steps:
                self.cb.disarm(ELRSCommand(armed=False, channel_2=-1.0))
                self.cb.request_shutdown()
                return
            
            # MPC Prep
            update_ocp_parameters(self.ocp, self.est_params, self.N)
            set_trajectory_reference_aligned(self.ocp, self.traj, self.N, self.step_counter, self.skip_steps, self.est_params)

            # Build initial state: [pose(13), u(4)]
            estimated_state = copy.deepcopy(self.current_pose[:13])
            if len(self.control_history) == 0:
                initial_x_with_u = np.concatenate((estimated_state, np.array([0.0, 0.0, 0.0, 0.0]))) 
            else:
                last_u = np.array(self.control_history[-1][0:4])
                initial_x_with_u = np.concatenate((estimated_state, last_u)) 

            # Set constraints on 0-th state
            self.ocp.set(0, "lbx", initial_x_with_u)
            self.ocp.set(0, "ubx", initial_x_with_u)

            if self.first_solve:
                set_initial_guess(self.ocp, self.N)
                self.first_solve = False
            else:
                warm_start_from_previous_solution(self.ocp, self.N)

            status = self.ocp.solve()
            if status != 0:
                self.get_logger().error(f"MPC Solve failed: {status}")
                return

            x_sol = self.ocp.get(1, "x")
            u, u_rate = x_sol[-4:], self.ocp.get(0, "u")

            if self.takeoff_requested:
                msg = ELRSCommand(armed=True, channel_0=round(u[0], 3), channel_1=round(u[1], 3), 
                                  channel_2=round((u[2]*2)-1, 3), channel_3=round(u[3], 3))
                self.cb.cmd_publisher_.publish(msg)
                self.step_counter += 1
            
            self.trajectory_visualizer.publish_actual_path(self.current_pose)
            self.control_history.append(np.concatenate((u, u_rate)).tolist())

    def signal_handler(self, sig, frame):
        self.on_close(); sys.exit(0)

    def on_close(self):
        if getattr(self, 'on_close_called', False): return
        self.on_close_called = True
        self.cb.cmd_publisher_.publish(ELRSCommand(armed=False, channel_2=-1.0))
        self.data_logger.close()

def main(args=None): 
    rclpy.init(args=args)
    controller = Controller()
    signal.signal(signal.SIGINT, controller.signal_handler)
    try: rclpy.spin(controller)
    finally: controller.on_close(); controller.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()
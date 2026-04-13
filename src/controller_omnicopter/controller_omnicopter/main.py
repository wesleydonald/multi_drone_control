import rclpy
import signal
import sys
import numpy as np
import math
import csv
import os
import datetime as date
from rclpy.node import Node
from datetime import datetime
from scipy.spatial.transform import Rotation as R
import time
# NOTE: import the new helper we added to acados.py
from .acados import (
    generate_ocp_controller,
    set_initial_guess,
    warm_start_from_previous_solution,
    set_trajectory_reference_aligned,   # <-- NEW
    set_adaptive_parameters,            # <-- NEW for adaptive parameters
)
from .ukf_estimator import UKFEstimator  # <-- UKF for adaptive parameter estimation
from .gui import GUI
from .trajectories import hover_trajectory, circle_trajectory, power_loop_trajectory, hover_and_rotate, sine_wave_trajectory, hover_and_yaw, four_roll_rotations_trajectory, zsine_trajectory
from interfaces.msg import MotionCaptureState, ELRSCommand
from geometry_msgs.msg import Pose, PoseArray


def _norm_quat_np(q):
    n = float(np.linalg.norm(q))
    return q if n == 0.0 else (q / n)


class Controller(Node):
    def __init__(self):
        super().__init__('controller')

        self.cmd_publisher_ = self.create_publisher(ELRSCommand, '/ELRSCommand', 10)
        self.pose_subscription_ = self.create_subscription(MotionCaptureState, '/motion_capture_state', self.pose_callback, 10)
        self.trajectory_publisher_ = self.create_publisher(PoseArray, '/planned_trajectory', 10)
        self.current_pose = None

        self.ocp, self.sim_integrator = generate_ocp_controller()

        self.dt = 1.0 / 60.0
        self.step_counter = 0
        self.timer = self.create_timer(self.dt, self.control_loop)

        # self.timer_test_angular = self.create_timer(self.dt, self.angular_velocity_test)

        self.traj = zsine_trajectory(self.dt)
        #self.traj = hover_and_yaw(self.dt)
        #self.traj = zsine_trajectory(self.dt)

        self.steps = self.traj.shape[1] - 1

        self.gui = GUI(self)
        self.armed = False

        self.pre_start_duration = 2.0
        self.pre_start_counter = 0
        self.pre_start_steps = int(self.pre_start_duration / self.dt)

        self.N = 30
        self.skip_steps = 3
        # Parameters: [theta_roll, theta_pitch, theta_yaw, thrust_base, motor_time_constant, ixx, iyy, izz]
        # thrust_base = 7.42678162 (will be multiplied by 1e-7 in dynamics)
        # mass and motor_distance are now known constants in the dynamics model
        # on roll 0.031
        self.params = np.array([0.031, 0.0, 0.0, 7.02678162, 0.04])
        
        # Initialize UKF estimator for adaptive parameter estimation (always enabled)
        self.ukf = UKFEstimator(self.sim_integrator, self.dt, initial_params=self.params)
    
        # Set initial adaptive parameters for both solvers (only once)
        set_adaptive_parameters(self.ocp, self.sim_integrator, self.params, self.N)
        self.predicted_next_state = None
        self.last_pose = None
        self.last_control = None
        self.last_predicted_state = None  # Store predicted state for error calculation

        self.sent_command = False
        self.initial_guess_set = False
        self.last_actual_actuators = None  # Track last actual actuator states
        self.last_desired_actuators = None  # Track last desired actuator states
        self.sd = 0.2
        self.IG = 0.04

        # CSV init
        if not hasattr(self, 'csv_initialized'):
            self.csv_initialized = True
            name = date.datetime.now().strftime("%Y%m%d_%H%M%S")
            self.csv_file = open(f'control_results_{name}.csv', mode='w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            
            # Create descriptive headers for x0 (29D current state)
            x0_headers = [
                'x0_px', 'x0_py', 'x0_pz',  # position (0-2)
                'x0_qw', 'x0_qx', 'x0_qy', 'x0_qz',  # quaternion (3-6)
                'x0_vx', 'x0_vy', 'x0_vz',  # linear velocity (7-9)
                'x0_wx', 'x0_wy', 'x0_wz',  # angular velocity (10-12)
                'x0_u0_act', 'x0_u1_act', 'x0_u2_act', 'x0_u3_act',  # actual actuators (13-16)
                'x0_u4_act', 'x0_u5_act', 'x0_u6_act', 'x0_u7_act',  # actual actuators (17-20)
                'x0_u0_des', 'x0_u1_des', 'x0_u2_des', 'x0_u3_des',  # desired actuators (21-24)
                'x0_u4_des', 'x0_u5_des', 'x0_u6_des', 'x0_u7_des'   # desired actuators (25-28)
            ]
            
            # Create descriptive headers for x_next (29D next state)  
            x_next_headers = [
                'x_next_px', 'x_next_py', 'x_next_pz',  # position (0-2)
                'x_next_qw', 'x_next_qx', 'x_next_qy', 'x_next_qz',  # quaternion (3-6)
                'x_next_vx', 'x_next_vy', 'x_next_vz',  # linear velocity (7-9)
                'x_next_wx', 'x_next_wy', 'x_next_wz',  # angular velocity (10-12)
                'x_next_u0_act', 'x_next_u1_act', 'x_next_u2_act', 'x_next_u3_act',  # actual actuators (13-16)
                'x_next_u4_act', 'x_next_u5_act', 'x_next_u6_act', 'x_next_u7_act',  # actual actuators (17-20)
                'x_next_u0_des', 'x_next_u1_des', 'x_next_u2_des', 'x_next_u3_des',  # desired actuators (21-24)
                'x_next_u4_des', 'x_next_u5_des', 'x_next_u6_des', 'x_next_u7_des'   # desired actuators (25-28)
            ]
            
            # Create headers for u_dot_rates (control rates)
            u_dot_headers = [
                'u_dot_0', 'u_dot_1', 'u_dot_2', 'u_dot_3',
                'u_dot_4', 'u_dot_5', 'u_dot_6', 'u_dot_7'
            ]
            
            # Create headers for prediction errors
            error_headers = [
                'pos_error', 'quat_error', 'vel_error', 'angvel_error', 
                'act_error', 'des_act_error'
            ]
            
            self.csv_writer.writerow([
                'Step'
            ] + x0_headers + u_dot_headers + x_next_headers + error_headers)

    def pose_callback(self, msg: MotionCaptureState):
        p, o, lv, av = msg.pose.position, msg.pose.orientation, msg.twist.linear, msg.twist.angular
        q = np.array([o.w, o.x, o.y, o.z], dtype=float)
        q = _norm_quat_np(q)  # keep unit quaternion
        self.current_pose = np.array([
            p.x, p.y, p.z, q[0], q[1], q[2], q[3], lv.x, lv.y, lv.z, av.x, av.y, av.z
        ])

    def expand_state_to_29d(self, state_13d, actual_actuators=None, desired_actuators=None):
        if actual_actuators is None:
            actual_actuators = np.array([-self.IG, self.IG, -self.IG, self.IG, self.IG, -self.IG, self.IG, -self.IG])
        if desired_actuators is None:
            desired_actuators = np.array([-self.IG, self.IG, -self.IG, self.IG, self.IG, -self.IG, self.IG, -self.IG])
        return np.concatenate([state_13d, actual_actuators, desired_actuators])

    def get_current_state_29d(self):
        if hasattr(self, 'last_actual_actuators') and self.last_actual_actuators is not None:
            actual_actuators = self.last_actual_actuators
        else:
            actual_actuators = np.array([-self.IG, self.IG, -self.IG, self.IG, self.IG, -self.IG, self.IG, -self.IG])
            
        if hasattr(self, 'last_desired_actuators') and self.last_desired_actuators is not None:
            desired_actuators = self.last_desired_actuators
        else:
            desired_actuators = np.array([-self.IG, self.IG, -self.IG, self.IG, self.IG, -self.IG, self.IG, -self.IG])
        
        return self.expand_state_to_29d(self.current_pose, actual_actuators, desired_actuators)



    def motor_actuators_from_wrench(self,v_cmd):
        """
        Input:  v_cmd = [Fx,Fy,Fz, tx,ty,tz]
        Output: actuators a[8] in [-1, 1], where thrust_i = thrust_c * (max_rpm * a_i)^2 (with sign).
        """
        # ---- constants (edit to your hardware) ----
        P = 0.17 * np.array([[ 1,-1, 1],[-1,-1, 1],[ 1,-1,-1],[-1,-1,-1],
                            [ 1, 1,-1],[-1, 1,-1],[ 1, 1, 1],[-1, 1, 1]], float)
        N = np.array([[-0.211325,-0.788675,-0.57735 ],
                    [ 0.788675,-0.211325, 0.57735 ],
                    [ 0.211325, 0.788675,-0.57735 ],
                    [-0.788675, 0.211325, 0.57735 ],
                    [ 0.788675,-0.211325, 0.57735 ],
                    [-0.211325,-0.788675,-0.57735 ],
                    [-0.788675, 0.211325, 0.57735 ],
                    [ 0.211325, 0.788675,-0.57735 ]], float)
        s      = np.array([-1, 1, 1,-1,-1, 1, 1,-1], float)  # -1=CCW, +1=CW
        kappa  = 0.05                                        # m; set 0.0 to ignore drag torque
        thrust_c = 7.42678162e-07                            # N / (RPM^2)
        max_rpm  = 4631.0

        # ---- build B ----
        B = np.zeros((6, 8))
        for i in range(8):
            Ni, ri = N[i], P[i]
            B[0:3, i] = Ni
            B[3:6, i] = np.cross(ri, Ni) + (kappa * s[i]) * Ni

        # ---- left pseudoinverse and base thrusts ----
        B_dag = B.T @ np.linalg.inv(B @ B.T)
        f = B_dag @ np.asarray(v_cmd, float)   # rotor thrusts in N (can be +/-)

        # ---- thrust -> actuator in [-1,1] ----
        # a_i = sgn(f_i) * sqrt(|f_i| / (thrust_c * max_rpm^2))
        denom = thrust_c * (max_rpm ** 2)
        a = np.sign(f) * np.sqrt(np.maximum(0.0, np.abs(f)) / denom)

        # saturate to [-1, 1]
        a = np.clip(a, -1.0, 1.0)

        return a  # length-8 array in [-1,1]


    def control_loop(self):

        if self.armed and self.pre_start_counter < self.pre_start_steps:
            print(f"Pre-start phase: {self.pre_start_counter + 1}/{self.pre_start_steps}")

            msg = ELRSCommand(armed=True, channel_0=-self.sd, channel_1=self.sd, channel_2=-self.sd, channel_3=self.sd, channel_4=self.sd, channel_5=-self.sd, channel_6=self.sd, channel_7=-self.sd)
            self.cmd_publisher_.publish(msg)
            self.pre_start_counter += 1

        elif self.armed and self.current_pose is not None:





            v_cmd = np.array([0.1, 0.1, 9.81, 0.0, 0.0, 0.1])  # e.g., 10 N upward
            motor_speeds = self.motor_actuators_from_wrench(v_cmd)
            print("Motor speeds [rad/s]:\n", np.round(motor_speeds, 2))



            # Send the converted motor speeds to the motors
            msg = ELRSCommand(
                armed=True,
                channel_0=round(motor_speeds[0], 3),
                channel_1=round(motor_speeds[1], 3),
                channel_2=round(motor_speeds[2], 3),
                channel_3=round(motor_speeds[3], 3),
                channel_4=round(motor_speeds[4], 3),
                channel_5=round(motor_speeds[5], 3),
                channel_6=round(motor_speeds[6], 3),
                channel_7=round(motor_speeds[7], 3)
            )
            self.cmd_publisher_.publish(msg)
            self.step_counter += 1




        else:
            print("Controller is not armed or current pose is not available.")
            msg = ELRSCommand(armed=True, channel_0=0.0, channel_1=0.0, channel_2=0.0, channel_3=0.0, channel_4=0.0, channel_5=0.0, channel_6=0.0, channel_7=0.0)
            self.cmd_publisher_.publish(msg)
            self.step_counter = 0



    def angular_velocity_test(self):

        if self.armed and self.sent_command == False:
            print("Sending initial command to arm the controller.")
            print(f"Current pose: {self.current_pose[10:13]}")
            u = np.array([0.3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

            # Use 29D state for simulation
            x_29d = self.get_current_state_29d()
            self.sim_integrator.set("x", x_29d)
            self.sim_integrator.set("u", u)
            self.sim_integrator.solve()
            predicted_state = self.sim_integrator.get("x")
            print("Predicted state:", predicted_state[10:13])

            msg = ELRSCommand(armed=True, channel_0=u[0], channel_1=u[1], channel_2=u[2], channel_3=u[3], channel_4=u[4], channel_5=u[5], channel_6=u[6], channel_7=u[7])
            self.cmd_publisher_.publish(msg)
            self.sent_command = True

        elif self.armed and self.sent_command == True:
            print(f"Current pose: {self.current_pose[10:13]}")
        else:
            u = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            msg = ELRSCommand(armed=True, channel_0=u[0], channel_1=u[1], channel_2=u[2], channel_3=u[3], channel_4=u[4], channel_5=u[5], channel_6=u[6], channel_7=u[7])
            self.cmd_publisher_.publish(msg)

    def signal_handler(self, sig, frame):
        self.on_close()

    def on_close(self):
        self.gui.quit()
        rclpy.shutdown()
        sys.exit(0)


def main(args=None):
    rclpy.init(args=args)
    controller = Controller()
    signal.signal(signal.SIGINT, controller.signal_handler)
    while rclpy.ok():
        rclpy.spin_once(controller, timeout_sec=0.1)
        controller.gui.handle_events()
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

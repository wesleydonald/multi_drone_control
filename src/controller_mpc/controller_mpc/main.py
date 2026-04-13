import rclpy
import signal
import sys
import numpy as np
import copy
import math
import os
import threading
from rclpy.node import Node
from datetime import datetime
from scipy.spatial.transform import Rotation as R
import time
from .acados import generate_ocp_controller, set_initial_guess, warm_start_from_previous_solution, set_trajectory_reference_aligned, update_ocp_parameters
from .trajectories import hover_trajectory, z_sin_trajectory, xyz_sine_trajectory, circle_trajectory, power_loop_trajectory, figure8_zsine_trajectory, fast_xyz_sine_trajectory
from utility_objects.visualization import TrajectoryVisualizer
from utility_objects.data_logger import DataLogger
from utility_objects.callback_manager import CallbackManager
from interfaces.msg import MotionCaptureState, ELRSCommand, Telemetry
from scipy.linalg import cholesky


POSE_TIMEOUT_THRESHOLD = 0.25  # seconds
USE_MOTION_CAPTURE = True  # Set to False to use ORB-SLAM data instead
FREQUENCY_HZ = 30.0
DT = 1.0 / FREQUENCY_HZ

LOGGING_NAME = 'controller_mpc'

class Controller(Node):
    def __init__(self):
        super().__init__('controller')

        # General Settings
        self.cb = CallbackManager(self)

        self.traj, trajectory_name = xyz_sine_trajectory(DT)
        self.trajectory_visualizer = TrajectoryVisualizer(self, frame_id="map")
        self.trajectory_visualizer.publish_all_visualizations(self.traj,  pose_subsample=15, show_velocity=False,  velocity_scale=0.3, color_by_time=True )

        self.timer = self.create_timer(DT, self.control_loop)
        self.step_counter = 0
        self.steps = self.traj.shape[1] - 1

        self.armed = False
        self.takeoff_requested = False
        self.shutdown_requested = False


        # MPC settings
        self.N = 20
        self.skip_steps = 3
        self.first_solve = True
        self.ocp, self.sim_integrator = generate_ocp_controller()

        # Parameters: [Thrust ratio, drag ratio, angular velocity tau, centre rate, max rate, expo]
        self.est_params = np.array([38.0, 0.0, 0.12, 70.0, 670.0, 0.5])

        # Logging
        log_headers = [
            'step', 'timestamp', 'u0', 'u1', 'u2', 'u3',
            'pose_x', 'pose_y', 'pose_z', 'pose_qw', 'pose_qx', 'pose_qy', 'pose_qz',
        ]
        self.data_logger = DataLogger(LOGGING_NAME, trajectory_name, log_headers)

        self.observed_state_history = []       
        self.control_history = []
        self.estimated_state_history = []


    def control_loop(self):


        if self.shutdown_requested:
            self.cb.request_shutdown()
            return

        if self.armed and (time.time() - self.last_pose_update_time) > POSE_TIMEOUT_THRESHOLD:
            msg = ELRSCommand(armed=False, channel_0=0.0, channel_1=0.0, channel_2=-1.0, channel_3=0.0)
            self.cb.disarm(msg)
            return 

        if self.armed and self.current_pose is not None:
            if self.step_counter + self.N * self.skip_steps > self.steps:
                msg = ELRSCommand(armed=False, channel_0=0.0, channel_1=0.0, channel_2=-1.0, channel_3=0.0)
                self.cb.disarm(msg)
                self.cb.request_shutdown()
                return

            set_trajectory_reference_aligned(self.ocp, self.traj, self.N, self.step_counter, self.skip_steps, self.est_params)
            estimated_state = copy.deepcopy(self.current_pose[:13])

            if len(self.control_history) == 0:
                self.get_logger().warn("Control history is empty - cannot set state bounds accurately")
                estimated_state_with_control = np.concatenate((estimated_state, np.array([0.0, 0.0, 0.0, 0.0]))) 
            else:
                estimated_state_with_control = np.concatenate((estimated_state, np.array(self.control_history[-1][0:4]))) 

            relaxation_factor = 0.025
            relaxed_lbx = estimated_state_with_control * (1 - relaxation_factor)
            relaxed_ubx = estimated_state_with_control * (1 + relaxation_factor)
            self.ocp.set(0, "lbx", relaxed_lbx)
            self.ocp.set(0, "ubx", relaxed_ubx)

            ### MPC WARM START
            if self.first_solve:
                set_initial_guess(self.ocp, self.N)
                self.first_solve = False
            else:
                warm_start_from_previous_solution(self.ocp, self.N)

            ### SOLVE OCP
            status = self.ocp.solve()
            if status != 0:
                raise Exception(f'acados returned status {status}.')
            x = self.ocp.get(1, "x")
            u = x[-4:]
            u_rate = self.ocp.get(0, "u")

            ### SEND COMMANDS
            # Only execute trajectory if takeoff has been requested
            if self.takeoff_requested:
                msg = ELRSCommand(armed=True, channel_0=round(u[0], 3), channel_1=round(u[1], 3), channel_2=round((u[2]*2)-1, 3), channel_3=round(u[3], 3))
                print(f"r: {round(u[0], 3)}, p: {round(u[1], 3)}, t: {round((u[2]), 3)}, y: {round(u[3], 3)}")
                print(f"EST. params - TR: {round(self.est_params[0],2)}, DC z: {round(self.est_params[1],3)}, Tau: {round(self.est_params[2],3)}, Centre deg: {round(self.est_params[3],1)}, Max deg: {round(self.est_params[4],1)}, expo: {round(self.est_params[5],3)}")
            else:
                # Stay armed but don't send thrust commands until takeoff
                msg = ELRSCommand(armed=True, channel_0=0.0, channel_1=0.0, channel_2=-1.0, channel_3=0.0)
                print("Armed - Waiting for TAKEOFF command")
            
            self.cb.cmd_publisher_.publish(msg)
            
            # Extract MPC trajectory for visualization
            mpc_trajectory = np.zeros((13, self.N))
            for i in range(self.N):
                x_i = self.ocp.get(i, "x")
                mpc_trajectory[:, i] = x_i[:13]
            
            # Replace the first state with the current measured pose to eliminate offset
            mpc_trajectory[:, 0] = self.current_pose[:13]
            
            # Publish MPC plan visualization
            self.trajectory_visualizer.publish_mpc_plan(mpc_trajectory)
            self.trajectory_visualizer.publish_transform_frame(self.current_pose, "drone_mocap")

            log_row = [
                self.step_counter,
                time.time(),
                float(u[0]), float(u[1]), float(u[2]), float(u[3]),
                float(self.current_pose[0]), float(self.current_pose[1]), float(self.current_pose[2]),
                float(self.current_pose[3]), float(self.current_pose[4]), float(self.current_pose[5]), float(self.current_pose[6])
            ]
            self.data_logger.append_row(log_row)


            # Publish actual path visualization (dotted red line)
            self.trajectory_visualizer.publish_actual_path(self.current_pose)
            
            # Only increment step counter if takeoff was requested
            if self.takeoff_requested:
                self.step_counter += 1

            self.control_history.append( np.concatenate( (u, u_rate) ).tolist() )
            self.observed_state_history.append( self.current_pose[:13].tolist() )
            self.estimated_state_history.append( estimated_state[:13].tolist() )

        else:
            msg = ELRSCommand(armed=False, channel_0=0.0, channel_1=0.0, channel_2=-1.0, channel_3=0.0)
            self.cb.cmd_publisher_.publish(msg)
            self.step_counter = 0




    def signal_handler(self, sig, frame):
        print("Interrupt received, shutting down...")
        self.on_close()
        sys.exit(0)

    def on_close(self):
        if getattr(self, 'on_close_called', False):
            return
        self.on_close_called = True
        msg = ELRSCommand(armed=False, channel_0=0.0, channel_1=0.0, channel_2=-1.0, channel_3=0.0)
        self.cb.cmd_publisher_.publish(msg)
        self.data_logger.close()


def main(args=None): 
    rclpy.init(args=args)
    controller = Controller()
    signal.signal(signal.SIGINT, controller.signal_handler)
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        print("Keyboard interrupt received")
    except Exception as e:
        print(f"Exception occurred: {e}")
    finally:
        controller.on_close()
        # Final cleanup
        try:
            controller.destroy_node()
        except:
            pass
        try:
            rclpy.shutdown()
        except:
            pass

if __name__ == '__main__':
    main()

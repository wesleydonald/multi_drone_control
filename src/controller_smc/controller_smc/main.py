import rclpy
import signal
import sys
import numpy as np
import math
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
import time
from .trajectories import hover_trajectory, z_sin_trajectory, xyz_sine_trajectory, pickup_object_trajectory
from utility_objects.visualization import TrajectoryVisualizer
from utility_objects.data_logger import DataLogger
from utility_objects.callback_manager import CallbackManager
from interfaces.msg import ELRSCommand


POSE_TIMEOUT_THRESHOLD = 0.25  # seconds
FREQUENCY_HZ = 120.0
DT = 1.0 / FREQUENCY_HZ

LOGGING_NAME = 'controller_smc'

class Controller(Node):
    def __init__(self):
        super().__init__('controller')

        # General Settings
        self.last_pose_update_time = time.time()
        self.cb = CallbackManager(self)

        self.traj, trajectory_name = pickup_object_trajectory(DT)
        self.trajectory_visualizer = TrajectoryVisualizer(self, frame_id="map")
        self.trajectory_visualizer.publish_all_visualizations(self.traj,  pose_subsample=15, show_velocity=False,  velocity_scale=0.3, color_by_time=True )

        self.timer = self.create_timer(DT, self.control_loop)
        self.step_counter = 0
        self.steps = self.traj.shape[1] - 1
        

        self.armed = False
        self.takeoff_requested = False
        self.shutdown_requested = False

        # Logging
        log_headers = [
            'step', 'timestamp', 'u0', 'u1', 'u2', 'u3',
            'pose_x', 'pose_y', 'pose_z', 'pose_qw', 'pose_qx', 'pose_qy', 'pose_qz',
        ]
        self.data_logger = DataLogger(LOGGING_NAME, trajectory_name, log_headers)

        self.M = 0.3  # CHANGE THIS TO ADJUST THRUST GAIN
        self.g = 9.81
        self.adaptive_throttle = 1



    def control_loop(self):

        if self.shutdown_requested:
            self.cb.request_shutdown()
            return

        if self.armed and (time.time() - self.last_pose_update_time) > POSE_TIMEOUT_THRESHOLD:
            msg = ELRSCommand(armed=False, channel_0=0.0, channel_1=0.0, channel_2=-1.0, channel_3=0.0)
            self.cb.disarm(msg)
            return 
        
        if self.armed and not self.takeoff_requested and self.current_pose is not None:
            msg = ELRSCommand(armed=True, channel_0=0.0, channel_1=0.0, channel_2=-1.0, channel_3=0.0)
            self.cb.cmd_publisher_.publish(msg)

        elif self.armed and self.takeoff_requested and self.current_pose is not None:

            if self.step_counter > self.steps:
                msg = ELRSCommand(armed=False, channel_0=0.0, channel_1=0.0, channel_2=-1.0, channel_3=0.0)
                self.cb.disarm(msg)
                self.cb.request_shutdown()
                return

            xd, yd, zd = self.traj[0:3, self.step_counter]
            yawd = 0.0
            x, y, z = self.current_pose[0:3]
            r, p, yaw = self.quaternion_to_euler(*self.current_pose[3:7])
            vx, vy, vz = self.current_pose[7:10]
        
            wy = -1*np.array([12.6320, 125.3600, 25.0785])@np.array([[x-xd], [p], [vx]])
            wy = (( wy[0]))/100.0
            wx = -1*np.array([-12.6320, 125.3600, -25.0785])@np.array([[y-yd], [r], [vy]]) # roll control
            wx = (( wx[0]))/100.0

            kpz, kiz, kdz = 15, 10, 10
            max_motor_speed = 4631.0
            Cf = 1.42e-06
            
            lambda_z = 0.8      # Sliding surface slope
            k_z = 0.8           # Switching gain
            phi = 0.1          # Boundary layer width

            ez = zd - z # position error
            edot_z = self.traj[9, self.step_counter] - vz # velocity error

            s = edot_z + lambda_z * ez
            sat_s = np.clip(s/phi, -1.0, 1.0)
            u_z = lambda_z * edot_z + k_z * sat_s + self.g

            force = self.M * u_z

            throttle = np.sqrt(force/(4*Cf))/max_motor_speed
            throttle = throttle * 2 - 1
            throttle = float(max(-1.0, min(1.0, throttle)))
  
            wz =  -0.5*(yawd-yaw) 
            u = [wx, wy, throttle, wz]


            print(f"Control Inputs: wx: {wx:.3f}, wy: {wy:.3f}, throttle: {throttle:.3f}, wz: {wz:.3f}")

            # Activate gripper after 30 seconds
            if (self.step_counter <= 120 * 30):
                msg = ELRSCommand(armed=True, channel_0=round(u[0], 3), channel_1=round(u[1], 3), channel_2=round(u[2], 3), channel_3=round(u[3], 3), channel_6 = -1.0)
            else:
                msg = ELRSCommand(armed=True, channel_0=round(u[0], 3), channel_1=round(u[1], 3), channel_2=round(u[2], 3), channel_3=round(u[3], 3), channel_6 = 0.0)
                print("grab")
            
            self.cb.cmd_publisher_.publish(msg)

            log_row = [
                self.step_counter,
                time.time(),
                float(u[0]), float(u[1]), float(u[2]), float(u[3]),
                float(self.current_pose[0]), float(self.current_pose[1]), float(self.current_pose[2]),
                float(self.current_pose[3]), float(self.current_pose[4]), float(self.current_pose[5]), float(self.current_pose[6])
            ]
            self.data_logger.append_row(log_row)
            self.trajectory_visualizer.publish_transform_frame(self.current_pose, "drone_mocap")
            self.trajectory_visualizer.publish_actual_path(self.current_pose)
            if self.takeoff_requested:
                self.step_counter += 1

        else:
            msg = ELRSCommand(armed=False, channel_0=0.0, channel_1=0.0, channel_2=-1.0, channel_3=0.0)
            self.cb.cmd_publisher_.publish(msg)
            self.step_counter = 0

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

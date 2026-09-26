# tejen_utility_objects/callbacks.py
import rclpy
import time
import threading
import numpy as np
import os
import signal


from std_msgs.msg import String, Bool
from interfaces.msg import MotionCaptureState, Telemetry, ELRSCommand
from interfaces.srv import SetArming

from .arm_permission import arm_permission_for_node

class CallbackManager:
    def __init__( self,node,USE_MOTION_CAPTURE: bool = True):      
        self.node = node

        self.cmd_publisher_ = self.node.create_publisher(ELRSCommand, 'ELRSCommand', 1)

        self.pose_subscription_ = self.node.create_subscription(MotionCaptureState, 'motion_capture_state', self.pose_callback, 5)
        self.telemetry_subscription_ = self.node.create_subscription(Telemetry, 'telemetry', self.telemetry_callback, 5)

        self.arming_service_ = self.node.create_service(SetArming, 'drone_arming_service', self.handle_arming_service)
        self.command_subscription_ = self.node.create_subscription(String, 'drone_command', self.command_callback, 5)
        self.arming_state_publisher_ = self.node.create_publisher(Bool, 'drone_arming_state_feedback', 5)

        self.motion_capture_pose = [0,0,0,1,0,0,0,0,0,0,0,0,0]  # x,y,z,qw,qx,qy,qz,vx,vy,vz,avx,avy,avz
        self.use_motion_capture = USE_MOTION_CAPTURE

        self.pendulum_subscription_ = self.node.create_subscription(
            MotionCaptureState,
            'pendulum_swing_state',
            self.pendulum_callback,
            5
        )

        self.pendulum_state = np.zeros(4)  # phi, theta, phi_dot, theta_dot


    def pendulum_callback(self, msg: MotionCaptureState):
        phi = msg.pose.position.x
        theta = msg.pose.position.y
        phi_dot = msg.twist.angular.x
        theta_dot = msg.twist.angular.y

        self.pendulum_state = np.array([
            phi,
            theta,
            phi_dot,
            theta_dot
        ])

        self.node.pendulum_state = self.pendulum_state
        # Payload-state freshness is physical time. In simulation the node clock
        # follows Gazebo /clock; on hardware it remains the normal ROS/system clock.
        self.node.last_pendulum_update_time = (
            self.node.get_clock().now().nanoseconds * 1e-9
        )

        
    def pose_callback(self, msg: MotionCaptureState):
        p, o, lv, av = msg.pose.position, msg.pose.orientation, msg.twist.linear, msg.twist.angular
        arr = np.round(np.array([
            p.x, p.y, p.z,
            o.w, o.x, o.y, o.z,
            lv.x, lv.y, lv.z,
            av.x, av.y, av.z
        ]), 3)

        self.motion_capture_pose = arr

        if self.use_motion_capture:
            self.node.current_pose = self.motion_capture_pose
            # Pose freshness is ROS-node time. In simulation this follows Gazebo
            # /clock; on hardware the same node clock is ordinary system time.
            self.node.last_pose_update_time = (
                self.node.get_clock().now().nanoseconds * 1e-9
            )

            if hasattr(self.node, "ukf_update_from_current_pose"):
                self.node.ukf_update_from_current_pose()

    def handle_arming_service(self, request, response):
        if request.arm:
            arm_allowed, arm_reason = arm_permission_for_node(self.node)
            if self.node.current_pose is not None and arm_allowed:
                self.node.armed = True
                response.success = True
                response.message = "Drone armed successfully"
                self.node.get_logger().info("Drone armed via service")
            elif self.node.current_pose is None:
                response.success = False
                response.message = "Cannot arm: No pose data available"
                self.node.get_logger().warn("Arming failed: No pose data")
            else:
                response.success = False
                detail = arm_reason or "external arm interlock is not satisfied"
                response.message = f"Cannot arm: {detail}"
                self.node.get_logger().warn(f"Arming blocked: {detail}")
        else:
            self.node.armed = False
            self.node.takeoff_requested = False
            self.node.shutdown_requested = True
            response.success = True
            response.message = "Drone disarmed successfully - shutting down controller"
            self.node.get_logger().info("Drone disarmed via service - initiating shutdown")
        
        self.publish_arming_state()
        return response
    

    def command_callback(self, msg: String):
        command = msg.data.upper()
        
        if command == "ARM":
            arm_allowed, arm_reason = arm_permission_for_node(self.node)
            if self.node.current_pose is not None and arm_allowed:
                self.node.armed = True
                self.node.get_logger().info("Drone armed via command")
                self.publish_arming_state()
            elif self.node.current_pose is None:
                self.node.get_logger().warn("Cannot arm: No pose data available")
            else:
                detail = arm_reason or "external arm interlock is not satisfied"
                self.node.get_logger().warn(f"Cannot arm: {detail}")
        
        elif command == "DISARM":
            self.node.armed = False
            self.node.takeoff_requested = False
            self.node.shutdown_requested = True
            self.node.get_logger().info("Drone disarmed via command - initiating shutdown")
            self.publish_arming_state()
        
        elif command == "TAKEOFF":
            if self.node.armed:
                self.node.takeoff_requested = True
                self.node.get_logger().info("Takeoff requested")
            else:
                self.node.get_logger().warn("Cannot takeoff: Drone not armed")

    def publish_arming_state(self):
        msg = Bool()
        msg.data = self.node.armed
        self.arming_state_publisher_.publish(msg)
    
    def telemetry_callback(self, msg: Telemetry):
        self.node.battery_voltage = msg.battery_voltage

    def request_shutdown(self):
        self.node.get_logger().info("Shutdown requested - closing controller")
        def shutdown_thread():
            time.sleep(0.1)
            self.node.on_close()
            try:
                self.node.destroy_node()
            except:
                pass
            try:
                rclpy.shutdown()
            except:
                pass
            os.kill(os.getpid(), signal.SIGTERM)
        thread = threading.Thread(target=shutdown_thread, daemon=True)
        thread.start()

    def safety_disarm(self, msg: ELRSCommand):
        self.node.armed = False
        self.node.takeoff_requested = False
        self.node.get_logger().warn("Safety disarm triggered - disarming drone")
        self.cmd_publisher_.publish(msg)
        self.publish_arming_state()

    def disarm(self, msg: ELRSCommand):
        self.node.armed = False
        self.node.takeoff_requested = False
        self.node.get_logger().warn("disarm triggered - disarming drone")
        self.cmd_publisher_.publish(msg)
        self.publish_arming_state()
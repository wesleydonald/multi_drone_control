"""
utility_objects/callback_manager.py
────────────────────────────────────
Per-drone scoped callbacks.  All topics and services are namespaced
under /drone_<id>/ so the fleet manager can target each drone individually
and the global broadcast bug from the original is fixed.
"""

import rclpy
import time
import threading
import numpy as np
import os
import signal

from rclpy.clock import Clock, ClockType
from std_msgs.msg import String, Bool
from interfaces.msg import MotionCaptureState, Telemetry, ELRSCommand
from interfaces.srv import SetArming


class CallbackManagerMulti:
    def __init__(self, node, drone_id: int = 0, USE_MOTION_CAPTURE: bool = True):
        self.node = node
        self.drone_id = drone_id
        self.use_motion_capture = USE_MOTION_CAPTURE

        # Wall clock for pose-timeout watchdog (independent of sim time)
        self._wall_clock = Clock(clock_type=ClockType.SYSTEM_TIME)

        # ── Publishers ────────────────────────────────────────────────────
        self.cmd_publisher_ = self.node.create_publisher(
            ELRSCommand, f'/drone_{drone_id}/ELRSCommand', 1)

        self.arming_state_publisher_ = self.node.create_publisher(
            Bool, f'/drone_{drone_id}/arming_state_feedback', 5)

        # ── Subscriptions ─────────────────────────────────────────────────
        self.pose_subscription_ = self.node.create_subscription(
            MotionCaptureState,
            f'/drone_{drone_id}/motion_capture_state',
            self.pose_callback, 5)

        self.telemetry_subscription_ = self.node.create_subscription(
            Telemetry, f'/drone_{drone_id}/telemetry',
            self.telemetry_callback, 5)

        # Per-drone command topic (fleet manager writes here)
        self.command_subscription_ = self.node.create_subscription(
            String,
            f'/drone_{drone_id}/command',
            self.command_callback, 5)

        # ── Services ──────────────────────────────────────────────────────
        # Scoped per drone so the fleet manager can arm/disarm individually
        self.arming_service_ = self.node.create_service(
            SetArming,
            f'/drone_{drone_id}/arming_service',
            self.handle_arming_service)

        # ── Internal state ────────────────────────────────────────────────
        self.motion_capture_pose = [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0]

    # ─────────────────────────────────────────────────────────────────────
    # Pose callback
    # ─────────────────────────────────────────────────────────────────────

    def pose_callback(self, msg: MotionCaptureState):
        p, o = msg.pose.position, msg.pose.orientation
        lv, av = msg.twist.linear, msg.twist.angular

        arr = np.round(np.array([
            p.x, p.y, p.z,
            o.w, o.x, o.y, o.z,
            lv.x, lv.y, lv.z,
            av.x, av.y, av.z
        ]), 3)

        self.motion_capture_pose = arr

        if self.use_motion_capture:
            self.node.current_pose = arr
            # Update wall-clock timestamp for pose-timeout watchdog
            self.node.last_pose_update_time = self._wall_clock.now()

            if hasattr(self.node, "ukf_update_from_current_pose"):
                self.node.ukf_update_from_current_pose()

    # ─────────────────────────────────────────────────────────────────────
    # Arming service handler
    # ─────────────────────────────────────────────────────────────────────

    def handle_arming_service(self, request, response):
        if request.arm:
            if self.node.current_pose is not None:
                self.node.armed = True
                response.success = True
                response.message = f"Drone {self.drone_id} armed successfully"
                self.node.get_logger().info(
                    f"[Drone {self.drone_id}] Armed via service.")
            else:
                response.success = False
                response.message = f"Drone {self.drone_id}: cannot arm — no pose data"
                self.node.get_logger().warn(
                    f"[Drone {self.drone_id}] Arming failed: no pose data.")
        else:
            self.node.armed = False
            self.node.takeoff_requested = False
            self.node.shutdown_requested = True
            response.success = True
            response.message = f"Drone {self.drone_id} disarmed — shutting down controller"
            self.node.get_logger().info(
                f"[Drone {self.drone_id}] Disarmed via service — initiating shutdown.")

        self.publish_arming_state()
        return response

    # ─────────────────────────────────────────────────────────────────────
    # String command callback
    # ─────────────────────────────────────────────────────────────────────

    def command_callback(self, msg: String):
        command = msg.data.strip().upper()

        if command == "ARM":
            if self.node.current_pose is not None:
                self.node.armed = True
                self.node.get_logger().info(
                    f"[Drone {self.drone_id}] Armed via command topic.")
                self.publish_arming_state()
            else:
                self.node.get_logger().warn(
                    f"[Drone {self.drone_id}] Cannot arm: no pose data.")

        elif command == "DISARM":
            self.node.armed = False
            self.node.takeoff_requested = False
            self.node.shutdown_requested = True
            self.node.get_logger().info(
                f"[Drone {self.drone_id}] Disarmed via command topic.")
            self.publish_arming_state()

        elif command == "TAKEOFF":
            if self.node.armed:
                self.node.takeoff_requested = True
                self.node.get_logger().info(
                    f"[Drone {self.drone_id}] Takeoff requested.")
            else:
                self.node.get_logger().warn(
                    f"[Drone {self.drone_id}] Cannot takeoff: not armed.")

        else:
            self.node.get_logger().warn(
                f"[Drone {self.drone_id}] Unknown command: '{command}'")

    # ─────────────────────────────────────────────────────────────────────
    # Helpers
    # ─────────────────────────────────────────────────────────────────────

    def publish_arming_state(self):
        msg = Bool()
        msg.data = self.node.armed
        self.arming_state_publisher_.publish(msg)

    def telemetry_callback(self, msg: Telemetry):
        self.node.battery_voltage = msg.battery_voltage

    def request_shutdown(self):
        self.node.get_logger().info(
            f"[Drone {self.drone_id}] Shutdown requested — closing controller.")

        def shutdown_thread():
            time.sleep(0.1)
            self.node.on_close()
            try:
                self.node.destroy_node()
            except Exception:
                pass
            try:
                rclpy.shutdown()
            except Exception:
                pass
            os.kill(os.getpid(), signal.SIGTERM)

        thread = threading.Thread(target=shutdown_thread, daemon=True)
        thread.start()

    def safety_disarm(self, msg: ELRSCommand):
        self.node.armed = False
        self.node.takeoff_requested = False
        self.node.get_logger().warn(
            f"[Drone {self.drone_id}] Safety disarm triggered.")
        self.cmd_publisher_.publish(msg)
        self.publish_arming_state()

    def disarm(self, msg: ELRSCommand):
        self.node.armed = False
        self.node.takeoff_requested = False
        self.node.get_logger().warn(
            f"[Drone {self.drone_id}] Disarm triggered.")
        self.cmd_publisher_.publish(msg)
        self.publish_arming_state()
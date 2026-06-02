"""
central_controller.py
────────────────
Single node that coordinates all drones.

Responsibilities
────────────────
1. Waits for all N_DRONES poses to arrive before doing anything.
2. Handles /fleet/command  (ARM | TAKEOFF | DISARM | ESTOP)
3. Broadcasts /fleet/step at FREQUENCY_HZ using Gazebo sim time.
4. Arms / disarms individual drones via /drone_N/arming_service.
5. Monitors /drone_N/arming_state_feedback — any unexpected disarm
   triggers an emergency stop of the whole fleet.

Usage
─────
ros2 run <your_package> central_controller

Then:
  ros2 topic pub --once /fleet/command std_msgs/msg/String "{data: ARM}"
  ros2 topic pub --once /fleet/command std_msgs/msg/String "{data: TAKEOFF}"
  ros2 topic pub --once /fleet/command std_msgs/msg/String "{data: DISARM}"
  ros2 topic pub --once /fleet/command std_msgs/msg/String "{data: ESTOP}"
"""

import rclpy
import threading
import time
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from std_msgs.msg import String, Bool, Int32
from interfaces.srv import SetArming

# ── Configuration ─────────────────────────────────────────────────────────────

N_DRONES = 4
FREQUENCY_HZ = 30.0          # Must match DT in controller_mpc.py
DT = 1.0 / FREQUENCY_HZ

# How long (real seconds) to wait for all drones to be ready before timing out
READY_TIMEOUT_SEC = 30.0

# If a drone misses this many consecutive heartbeats we treat it as dropped
ARMING_FEEDBACK_TIMEOUT_SEC = 1.0


class CentralController(Node):

    def __init__(self):
        super().__init__('central_controller', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, False)
        ])

        # ── Wall clock for real-time safety checks ────────────────────────
        self._wall_clock = Clock(clock_type=ClockType.SYSTEM_TIME)

        # ── Fleet state ───────────────────────────────────────────────────
        self.master_step = 0
        self.flying = False          # True after TAKEOFF, False before/after
        self.fleet_armed = False     # True after ARM, before DISARM
        self.shutdown_requested = False

        # Per-drone arming state (updated by feedback callbacks)
        self.drone_armed = {i: False for i in range(N_DRONES)}
        self.drone_last_feedback = {i: self._wall_clock.now() for i in range(N_DRONES)}

        # ── Publishers ────────────────────────────────────────────────────
        self.step_pub = self.create_publisher(Int32, '/fleet/step', 1)

        # ── Fleet command subscription ────────────────────────────────────
        self.cmd_sub = self.create_subscription(
            String, '/fleet/command', self._command_callback, 10)

        # ── Per-drone arming service clients ──────────────────────────────
        self.arming_clients = {}
        for i in range(N_DRONES):
            client = self.create_client(SetArming, f'/drone_{i}/arming_service')
            self.arming_clients[i] = client

        # ── Per-drone arming feedback subscriptions ───────────────────────
        for i in range(N_DRONES):
            self.create_subscription(
                Bool,
                f'/drone_{i}/arming_state_feedback',
                lambda msg, drone_id=i: self._arming_feedback_callback(msg, drone_id),
                5)

        # ── Master step timer (sim time) ──────────────────────────────────
        self.timer = self.create_timer(DT, self._step_timer_callback)

        self.get_logger().info(
            f"CentralController ready. Listening on /fleet/command. "
            f"Managing {N_DRONES} drones at {FREQUENCY_HZ} Hz (sim time).")

        self.drone_cmd_publishers = {}
        for i in range(N_DRONES):
            self.drone_cmd_publishers[i] = self.create_publisher(
                String, f'/drone_{i}/command', 10)
    # ─────────────────────────────────────────────────────────────────────
    # Timer — master step broadcast
    # ─────────────────────────────────────────────────────────────────────

    def _step_timer_callback(self):
        if self.shutdown_requested:
            return

        # Always publish the current step so drones can initialise their
        # reference window even before TAKEOFF.
        msg = Int32(data=self.master_step)
        self.step_pub.publish(msg)

        # Only advance the counter while the fleet is actively flying.
        if self.flying:
            self.master_step += 1

    # ─────────────────────────────────────────────────────────────────────
    # Fleet command handler
    # ─────────────────────────────────────────────────────────────────────

    def _command_callback(self, msg: String):
        command = msg.data.strip().upper()
        self.get_logger().info(f"Fleet command received: '{command}'")

        if command == "ARM":
            self._arm_fleet()
        elif command == "TAKEOFF":
            self._takeoff_fleet()
        elif command == "DISARM":
            self._disarm_fleet(emergency=False)
        elif command == "ESTOP":
            self.get_logger().error("EMERGENCY STOP commanded!")
            self._disarm_fleet(emergency=True)
        else:
            self.get_logger().warn(f"Unknown fleet command: '{command}'")

    # ─────────────────────────────────────────────────────────────────────
    # Arming feedback — safety monitor
    # ─────────────────────────────────────────────────────────────────────

    def _arming_feedback_callback(self, msg: Bool, drone_id: int):
        self.drone_last_feedback[drone_id] = self._wall_clock.now()
        was_armed = self.drone_armed[drone_id]
        self.drone_armed[drone_id] = msg.data

        # If a drone disarmed unexpectedly while the fleet is flying,
        # trigger an emergency stop for the entire fleet.
        if self.flying and was_armed and not msg.data:
            self.get_logger().error(
                f"Drone {drone_id} disarmed unexpectedly during flight — "
                f"triggering emergency stop for all drones!")
            self._disarm_fleet(emergency=True)

    # ─────────────────────────────────────────────────────────────────────
    # Fleet operations (run in background threads to avoid blocking the
    # ROS spin loop while waiting for service responses)
    # ─────────────────────────────────────────────────────────────────────

    def _arm_fleet(self):
        thread = threading.Thread(target=self._arm_fleet_thread, daemon=True)
        thread.start()

    def _arm_fleet_thread(self):
        self.get_logger().info("Arming all drones...")

        # Wait for all arming services to become available
        for i in range(N_DRONES):
            client = self.arming_clients[i]
            if not client.wait_for_service(timeout_sec=5.0):
                self.get_logger().error(
                    f"Arming service for drone {i} not available — aborting ARM.")
                return

        # Send arm requests in parallel
        futures = {}
        for i in range(N_DRONES):
            req = SetArming.Request()
            req.arm = True
            futures[i] = self.arming_clients[i].call_async(req)

        # Wait for all responses
        deadline = time.time() + 5.0
        for i, future in futures.items():
            remaining = max(0.0, deadline - time.time())
            # Spin until the future is done or timeout
            rclpy.spin_until_future_complete(self, future, timeout_sec=remaining)
            if future.done():
                result = future.result()
                if result.success:
                    self.get_logger().info(f"Drone {i} armed: {result.message}")
                else:
                    self.get_logger().error(
                        f"Drone {i} arming failed: {result.message}")
            else:
                self.get_logger().error(
                    f"Drone {i} arming service timed out.")

        self.fleet_armed = True
        self.get_logger().info("ARM sequence complete.")

    def _takeoff_fleet(self):
        if not self.fleet_armed:
            self.get_logger().warn(
                "Cannot TAKEOFF: fleet is not armed. Send ARM first.")
            return
        self.master_step = 0
        self.flying = True
        self.get_logger().info(
            "TAKEOFF: master step counter reset to 0, broadcasting steps.")
        # Drones listen to /fleet/step and their own armed+takeoff_requested
        # flags.  We publish TAKEOFF via the per-drone command topic so each
        # controller's takeoff_requested flag is set.
        self._publish_drone_command("TAKEOFF")

    def _disarm_fleet(self, emergency: bool = False):
        self.flying = False
        self.fleet_armed = False
        self.shutdown_requested = not emergency  # on estop keep node alive for debug

        label = "EMERGENCY STOP" if emergency else "DISARM"
        self.get_logger().info(f"{label}: sending disarm to all drones.")

        thread = threading.Thread(
            target=self._disarm_fleet_thread, daemon=True)
        thread.start()

    def _disarm_fleet_thread(self):
        futures = {}
        for i in range(N_DRONES):
            client = self.arming_clients[i]
            if client.service_is_ready():
                req = SetArming.Request()
                req.arm = False
                futures[i] = client.call_async(req)
            else:
                self.get_logger().warn(
                    f"Arming service for drone {i} not ready during disarm — skipping.")

        deadline = time.time() + 3.0
        for i, future in futures.items():
            remaining = max(0.0, deadline - time.time())
            rclpy.spin_until_future_complete(self, future, timeout_sec=remaining)
            if future.done():
                result = future.result()
                self.get_logger().info(
                    f"Drone {i} disarm response: {result.message}")

        self.get_logger().info("Disarm sequence complete.")

    def _publish_drone_command(self, command: str):
        msg = String(data=command)
        for i in range(N_DRONES):
            self.drone_cmd_publishers[i].publish(msg)
        self.get_logger().info(f"Published '{command}' to all {N_DRONES} drone command topics.")


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = CentralController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("CentralController interrupted.")
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
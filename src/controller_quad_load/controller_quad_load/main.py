"""
central_controller.py
────────────────
Single node that coordinates all drones.

Responsibilities
────────────────
1. Waits for all N_DRONES poses to arrive before doing anything.
2. Handles /fleet/command  (ARM | TAKEOFF | LAND | DISARM | ESTOP)
   LAND: planner lowers the load back down, then we disarm once it reports done.
3. Broadcasts /fleet/step at FREQUENCY_HZ using Gazebo sim time.
4. Arms / disarms individual drones via /drone_N/arming_service.
5. Monitors /drone_N/arming_state_feedback - any unexpected disarm in flight
   triggers an emergency stop of the whole fleet; one between ARM and TAKEOFF
   disarms the fleet through the services and refuses TAKEOFF, like a failed ARM.

Usage
─────
ros2 run <your_package> central_controller

Then:
  ros2 topic pub --once /fleet/command std_msgs/msg/String "{data: ARM}"
  ros2 topic pub --once /fleet/command std_msgs/msg/String "{data: TAKEOFF}"
  ros2 topic pub --once /fleet/command std_msgs/msg/String "{data: LAND}"
  ros2 topic pub --once /fleet/command std_msgs/msg/String "{data: DISARM}"
  ros2 topic pub --once /fleet/command std_msgs/msg/String "{data: ESTOP}"
"""

import rclpy
import threading
import time
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String, Bool, Int32
from interfaces.srv import SetArming
from interfaces.msg import ELRSCommand

# ── Configuration ─────────────────────────────────────────────────────────────

N_DRONES_DEFAULT = 4         # overridable via the 'num_drones' ROS param
FREQUENCY_HZ = 50.0          # Must match DT in controller_mpc.py
DT = 1.0 / FREQUENCY_HZ

# How long (real seconds) to wait for all drones to be ready before timing out
READY_TIMEOUT_SEC = 30.0

# If a drone misses this many consecutive heartbeats we treat it as dropped
ARMING_FEEDBACK_TIMEOUT_SEC = 1.0

# elrs_mux's /drone_i/mux_state ('partner' | 'ours' | 'latched'), latched on both ends so
# the state is read even when the manager starts after the mux. No publisher (no mux on
# that drone) = ours and not latched.
MUX_STATE_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL)


class CentralController(Node):

    def __init__(self):
        # Same clock-domain fix as controller_mpc.Controller: use_sim_time is the
        # LAUNCH's call. This was pinned False here, contradicting the module docstring
        # ("Broadcasts /fleet/step at FREQUENCY_HZ using Gazebo sim time") and putting
        # the step broadcast on a different clock from the sim-time planner. Since
        # /fleet/step paces the trackers, that made the step-to-physics ratio depend on
        # Gazebo's real-time factor. Hardware launches set it false explicitly.
        super().__init__('central_controller')

        # ── Fleet size (configurable so the same node serves 2- or 4-drone
        #    worlds via `num_drones` launch arg / ROS param) ───────────────
        self.declare_parameter('num_drones', N_DRONES_DEFAULT)
        self.num_drones = self.get_parameter('num_drones').value

        # ── Wall clock for real-time safety checks ────────────────────────
        self._wall_clock = Clock(clock_type=ClockType.SYSTEM_TIME)

        # ── Fleet state ───────────────────────────────────────────────────
        self.master_step = 0
        self.flying = False          # True after TAKEOFF, False before/after
        self.fleet_armed = False     # True after ARM, before DISARM
        self.landing = False         # True after LAND, until the descent finishes
        self.shutdown_requested = False

        # Per-drone arming state (updated by feedback callbacks)
        self.drone_armed = {i: False for i in range(self.num_drones)}
        self.drone_last_feedback = {i: self._wall_clock.now() for i in range(self.num_drones)}

        # ── Publishers ────────────────────────────────────────────────────
        self.step_pub = self.create_publisher(Int32, '/fleet/step', 1)

        # ── Fleet command subscription ────────────────────────────────────
        self.cmd_sub = self.create_subscription(
            String, '/fleet/command', self._command_callback, 10)
        # planner signals here once the LAND descent has finished, so we disarm.
        self.landed_sub = self.create_subscription(
            Bool, '/fleet/landed', self._landed_callback, 1)

        # ── Per-drone arming service clients ──────────────────────────────
        self.arming_clients = {}
        for i in range(self.num_drones):
            client = self.create_client(SetArming, f'/drone_{i}/arming_service')
            self.arming_clients[i] = client

        # ── Per-drone arming feedback subscriptions ───────────────────────
        for i in range(self.num_drones):
            self.create_subscription(
                Bool,
                f'/drone_{i}/arming_state_feedback',
                lambda msg, drone_id=i: self._arming_feedback_callback(msg, drone_id),
                5)

        # ── Per-drone ELRS mux state (drones with a mux only) ─────────────
        self.mux_state = {}
        for i in range(self.num_drones):
            self.create_subscription(
                String, f'/drone_{i}/mux_state',
                lambda msg, drone_id=i: self._mux_state_callback(msg, drone_id),
                MUX_STATE_QOS)

        # ── Master step timer (sim time) ──────────────────────────────────
        self.timer = self.create_timer(DT, self._step_timer_callback)

        self.get_logger().info(
            f"CentralController ready: {self.num_drones} drones, waiting for ARM on /fleet/command.")

        self.drone_cmd_publishers = {}
        for i in range(self.num_drones):
            self.drone_cmd_publishers[i] = self.create_publisher(
                String, f'/drone_{i}/command', 10)

        # ── Emergency stop path (finding F4) ──────────────────────────────
        # The normal disarm goes through each drone's SetArming *service* with a
        # 3 s deadline, and SKIPS any drone whose service is not ready. That is
        # fine for an orderly landing and wrong for an emergency: it depends on
        # service responsiveness at exactly the moment the system is misbehaving.
        # So an emergency additionally (a) broadcasts /fleet/abort, which every
        # tracker acts on immediately, and (b) publishes a disarm straight to each
        # drone's ELRSCommand topic, which reaches the radio even if a tracker
        # process is wedged and no longer publishing.
        self.abort_pub = self.create_publisher(String, '/fleet/abort', 5)
        self.status_pub = self.create_publisher(String, '/fleet/manager_status', MUX_STATE_QOS)
        self.elrs_publishers = {
            i: self.create_publisher(ELRSCommand, f'/drone_{i}/ELRSCommand', 1)
            for i in range(self.num_drones)
        }
    # ─────────────────────────────────────────────────────────────────────
    # Timer - master step broadcast
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
        elif command == "LAND":
            self._land_fleet()
        elif command == "DISARM":
            # The operator's disarm is the rig kill switch: the fast path too (a disarm straight
            # to every radio, reaching a drone whose tracker is wedged) and /fleet/abort, which
            # latches the muxes of drones on the partner's stream (Wesley 2026-09-28/29).
            self._disarm_fleet(emergency=True, reason="operator DISARM")
        elif command == "ESTOP":
            self._disarm_fleet(emergency=True, reason="operator ESTOP")
        else:
            self.get_logger().warn(f"Unknown fleet command: '{command}'")

    # ─────────────────────────────────────────────────────────────────────
    # Arming feedback - safety monitor
    # ─────────────────────────────────────────────────────────────────────

    def _mux_state_callback(self, msg: String, drone_id: int):
        state = msg.data.strip().lower()
        if self.mux_state.get(drone_id) != state:
            self.get_logger().info(f"Drone {drone_id} mux: {state}")
        self.mux_state[drone_id] = state

    def _arming_feedback_callback(self, msg: Bool, drone_id: int):
        self.drone_last_feedback[drone_id] = self._wall_clock.now()
        was_armed = self.drone_armed[drone_id]
        self.drone_armed[drone_id] = msg.data

        # Between ARM and TAKEOFF (R0749: every tracker pose-timed-out and TAKEOFF was still
        # accepted): nothing of ours flies yet, so ground our fleet like a failed ARM (Q11)
        # rather than abort, which would latch the partner's drones behind the muxes.
        if self.fleet_armed and not self.flying and was_armed and not msg.data:
            _announce(self, 'error',
                      f"Drone {drone_id} disarmed before TAKEOFF: fleet disarmed, TAKEOFF refused. "
                      f"Usually a pose timeout (mocap lost > 0.25 s) or a solver failure: see drone "
                      f"{drone_id}'s lines above. Relaunch T2 to retry.")
            self._disarm_fleet(emergency=False,
                               reason=f"drone {drone_id} disarmed before TAKEOFF")
            return

        # If a drone disarmed unexpectedly while the fleet is flying,
        # trigger an emergency stop for the entire fleet.
        if self.flying and was_armed and not msg.data:
            if self.mux_state.get(drone_id) == 'partner':
                # Not under our command (its mux forwards the partner's stream): the
                # tracker's own disarm does not reach the radio, and grounding everyone
                # for it would drop a drone we do not fly (ruling F5, R0618).
                self.get_logger().warn(
                    f"Drone {drone_id} tracker disarmed while its mux forwards the "
                    f"partner - not escalating to a fleet abort.")
                return
            _announce(self, 'error',
                      f"Drone {drone_id} disarmed in flight: EMERGENCY STOP for the whole fleet. "
                      f"See drone {drone_id}'s lines above for why (pose timeout, envelope, "
                      f"reference stale).")
            self._disarm_fleet(
                emergency=True,
                reason=f"drone {drone_id} disarmed unexpectedly")

    # ─────────────────────────────────────────────────────────────────────
    # Fleet operations (run in background threads to avoid blocking the
    # ROS spin loop while waiting for service responses)
    # ─────────────────────────────────────────────────────────────────────

    def _arm_fleet(self):
        thread = threading.Thread(target=self._arm_fleet_thread, daemon=True)
        thread.start()

    def _arm_fleet_thread(self):
        # A latched mux holds its drone's motors off until the mux restarts: armed
        # trackers behind it would wind up their integrators against dead motors.
        latched = sorted(i for i, s in self.mux_state.items() if s == 'latched')
        if latched:
            self.fleet_armed = False
            for i in latched:
                _announce(self, 'error', f"ARM REFUSED: mux latched on drone {i} (relaunch the muxes)")
            return
        self.get_logger().debug("Arming all drones...")
        self.fleet_armed = False          # a repeat ARM must re-earn it

        # Wait for all arming services to become available
        for i in range(self.num_drones):
            client = self.arming_clients[i]
            if not client.wait_for_service(timeout_sec=5.0):
                _announce(self, 'error',
                          f"ARM FAILED for drone(s) [{i}]: its tracker is not up (not started, or "
                          f"still building its solver). Fleet disarmed; TAKEOFF refused. Relaunch T2.")
                self._disarm_fleet(emergency=False,
                                   reason=f"arming service for drone {i} not available")
                return

        # Send arm requests in parallel
        futures = {}
        for i in range(self.num_drones):
            req = SetArming.Request()
            req.arm = True
            futures[i] = self.arming_clients[i].call_async(req)

        # Wait for all responses
        deadline = time.time() + 5.0
        failed = []
        for i, future in futures.items():
            remaining = max(0.0, deadline - time.time())
            # Spin until the future is done or timeout
            rclpy.spin_until_future_complete(self, future, timeout_sec=remaining)
            if future.done():
                result = future.result()
                if result.success:
                    self.get_logger().debug(f"Drone {i} armed: {result.message}")
                else:
                    failed.append(i)
                    self.get_logger().error(f"Drone {i} refused ARM: {result.message}")
            else:
                failed.append(i)
                self.get_logger().error(
                    f"Drone {i} did not answer ARM within 5 s (tracker busy, e.g. building its solver).")

        if failed:
            # A fleet missing a drone must not take off: the others would lift and
            # drag the payload (R0560: drone 0's tracker died before ARM, the manager
            # declared the ARM complete and three drones hauled the ring to 44 deg).
            # Service disarm only, no /fleet/abort and no direct ELRS (Q6b): nothing of
            # ours is flying yet, and in M2 the partner's drones hold the ring behind
            # the muxes, which an abort would latch down.
            _announce(self, 'error',
                      f"ARM FAILED for drone(s) {failed}: fleet disarmed; TAKEOFF refused. "
                      f"Relaunch T2 to retry (the disarm shuts the trackers down).")
            self._disarm_fleet(emergency=False,
                               reason=f"ARM failed for drone(s) {failed}")
            return
        self.fleet_armed = True
        _announce(self, 'info', f"ARM sequence complete: all {self.num_drones} armed, ready for TAKEOFF.")

    def _takeoff_fleet(self):
        if not self.fleet_armed:
            _announce(self, 'warn', "Cannot TAKEOFF: fleet is not armed. Send ARM first.")
            return
        # the feedback path can miss a disarm that raced the ARM thread (a False handled
        # before fleet_armed was set): check every managed drone's last reported state
        not_armed = [i for i in range(self.num_drones) if not self.drone_armed.get(i)]
        if not_armed:
            _announce(self, 'error',
                      f"TAKEOFF REFUSED: drone(s) {not_armed} not armed: fleet disarmed. "
                      f"Relaunch T2 to retry.")
            self._disarm_fleet(emergency=False,
                               reason=f"drone(s) {not_armed} not armed at TAKEOFF")
            return
        self.master_step = 0
        self.flying = True
        _announce(self, 'info', "TAKEOFF: flying.")
        # Drones listen to /fleet/step and their own armed+takeoff_requested
        # flags.  We publish TAKEOFF via the per-drone command topic so each
        # controller's takeoff_requested flag is set.
        self._publish_drone_command("TAKEOFF")

    def _land_fleet(self):
        if not self.flying:
            _announce(self, 'warn', "Cannot LAND: fleet is not flying.")
            return
        self.landing = True
        _announce(self, 'info', "LAND: descending; the fleet disarms when the load is down.")
        # The planner also subscribes to /fleet/command and starts the descent.
        # The drones keep tracking the (now descending) reference until we disarm
        # on /fleet/landed below.

    def _landed_callback(self, msg: Bool):
        # planner reports the descent is complete; disarm to settle on the ground.
        if not (self.landing and msg.data):
            return
        self.landing = False
        _announce(self, 'info', "Landed - disarming the fleet.")
        self._disarm_fleet(emergency=False)

    def _disarm_fleet(self, emergency: bool = False, reason: str = ''):
        self.flying = False
        self.fleet_armed = False
        self.shutdown_requested = not emergency  # on estop keep node alive for debug

        if not emergency:
            self.get_logger().info("DISARM: disarming all drones" + (f" ({reason})" if reason else ""))

        if emergency:
            # Fast path FIRST, synchronously, before spawning any thread or
            # touching a service. Both of these are fire-and-forget publishes.
            self.abort_pub.publish(String(data=reason or 'emergency stop'))
            stop = ELRSCommand(armed=False, channel_0=0.0, channel_1=0.0,
                               channel_2=-1.0, channel_3=0.0)
            for i, pub in self.elrs_publishers.items():
                pub.publish(stop)
            _announce(self, 'error',
                      f"EMERGENCY STOP broadcast to {len(self.elrs_publishers)} drones"
                      + (f": {reason}" if reason else ""))

        # Services still run, as the authoritative/acknowledged disarm.
        thread = threading.Thread(
            target=self._disarm_fleet_thread, daemon=True)
        thread.start()

    def _disarm_fleet_thread(self):
        futures = {}
        for i in range(self.num_drones):
            client = self.arming_clients[i]
            if client.service_is_ready():
                req = SetArming.Request()
                req.arm = False
                futures[i] = client.call_async(req)
            else:
                self.get_logger().warn(
                    f"Drone {i}: tracker not answering, its disarm is not confirmed (the radio "
                    f"disarm and /fleet/abort still apply on an emergency stop).")

        deadline = time.time() + 3.0
        for i, future in futures.items():
            remaining = max(0.0, deadline - time.time())
            rclpy.spin_until_future_complete(self, future, timeout_sec=remaining)
            if future.done():
                result = future.result()
                self.get_logger().debug(
                    f"Drone {i} disarm response: {result.message}")

        self.get_logger().info("Disarm complete.")

    def _publish_drone_command(self, command: str):
        msg = String(data=command)
        for i in range(self.num_drones):
            self.drone_cmd_publishers[i].publish(msg)
        self.get_logger().debug(f"Published '{command}' to all {self.num_drones} drone command topics.")


def _announce(node, level, text):
    """Log text at level and repeat it on the latched /fleet/manager_status, which the RViz
    panel shows: the operator sees refusals and state changes without reading T2."""
    # rclpy fixes the severity per calling line, so each level needs its own call
    log = node.get_logger()
    if level == 'error':
        log.error(text)
    elif level == 'warn':
        log.warning(text)
    else:
        log.info(text)
    pub = getattr(node, 'status_pub', None)
    if pub is not None:
        pub.publish(String(data=text))


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
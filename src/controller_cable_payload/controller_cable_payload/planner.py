"""
planner.py — Central hover planner (TU Delft simplified)
---------------------------------------------------------
Implements the kinematic planning layer from the paper.  For a hover
setpoint this reduces to: given the desired load position p_L and
identity attitude, compute each drone's desired hover position using the
kinematic constraint

    p_i = p_L + R(q_L) * rho_i - l * s_i

where for vertical cables (hover), s_i = [0,0,-1], R(q_L) = I, so

    p_i = p_L + rho_i + [0, 0, l]

rho_i is the attachment-point offset on the payload (in payload frame).
Because the payload in world_quad_payload.sdf has all cables attaching at
the centre of mass, rho_i = (0, 0, 0) for all i, and the drones should
hover directly above the payload at height  p_L.z + l.

In practice the drones spread out horizontally in a square formation (set
by DRONE_OFFSETS_XY), so each drone targets:

    p_i_des = (payload_x + offset_x_i,
               payload_y + offset_y_i,
               payload_z_desired + CABLE_LENGTH)

State machine:  /fleet/command  (ARM | TAKEOFF | DISARM | ESTOP)
  Same interface as controller_mpc_multi so run_multi.sh commands work.

Publishes:
  /drone_{i}/setpoint   Float64MultiArray [x, y, z, vx, vy, vz]
"""

import threading
import time
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Float64MultiArray
from interfaces.msg import MotionCaptureState
from interfaces.srv import SetArming

# ── Configuration ──────────────────────────────────────────────────────────────

N_DRONES = 4
FREQUENCY_HZ = 30.0

# Defaults match world_quad_payload.sdf (overridden via ROS params for other worlds).
_DEFAULT_CABLE_LENGTH   = 0.75
_DEFAULT_PAYLOAD_HOVER_Z = 0.84
# Drone XY offsets from payload centre, indexed 0-3.
_DEFAULT_OFFSETS_X = [ 0.25, -0.25, -0.25,  0.25]
_DEFAULT_OFFSETS_Y = [ 0.25,  0.25, -0.25, -0.25]


class Planner(Node):

    def __init__(self):
        super().__init__('planner', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, False)
        ])

        self.declare_parameter('cable_length',    _DEFAULT_CABLE_LENGTH)
        self.declare_parameter('payload_hover_z', _DEFAULT_PAYLOAD_HOVER_Z)
        self.declare_parameter('drone_offsets_x', _DEFAULT_OFFSETS_X)
        self.declare_parameter('drone_offsets_y', _DEFAULT_OFFSETS_Y)
        # tether_length > 0 enables sphere projection for rigid-cable worlds.
        # Set to the physical SDF joint length. 0.0 = disabled (spring-damper worlds).
        self.declare_parameter('tether_length', 0.0)

        cable_length    = self.get_parameter('cable_length').value
        payload_hover_z = self.get_parameter('payload_hover_z').value
        offsets_x       = list(self.get_parameter('drone_offsets_x').value)
        offsets_y       = list(self.get_parameter('drone_offsets_y').value)
        tether_length   = float(self.get_parameter('tether_length').value)

        self._cable_length    = float(cable_length)
        self._payload_hover_z = float(payload_hover_z)
        self._offsets_xy = {i: (float(offsets_x[i]), float(offsets_y[i]))
                            for i in range(N_DRONES)}
        self._tether_length = tether_length  # 0 = no projection

        self.get_logger().info(
            f'Planner: cable_length={self._cable_length:.3f}m  '
            f'payload_hover_z={self._payload_hover_z:.3f}m  '
            f'tether_length={self._tether_length:.3f}m  '
            f'offsets={self._offsets_xy}')

        # Fleet state
        self.fleet_armed = False
        self.flying = False

        # Payload state (updated from /payload/motion_capture_state)
        self.payload_pos = np.array([0.0, 0.0, self._payload_hover_z])

        # Publishers — one setpoint topic per drone
        self._setpoint_pubs = {}
        for i in range(N_DRONES):
            self._setpoint_pubs[i] = self.create_publisher(
                Float64MultiArray, f'/drone_{i}/setpoint', 1)

        # Fleet command subscription
        self.create_subscription(String, '/fleet/command', self._fleet_cmd_cb, 10)

        # Payload state subscription
        self.create_subscription(
            MotionCaptureState, '/payload/motion_capture_state', self._payload_cb, 5)

        # Per-drone arming service clients
        self._arming_clients = {}
        for i in range(N_DRONES):
            self._arming_clients[i] = self.create_client(
                SetArming, f'/drone_{i}/arming_service')

        # Per-drone command publishers (for TAKEOFF / DISARM)
        self._drone_cmd_pubs = {}
        for i in range(N_DRONES):
            self._drone_cmd_pubs[i] = self.create_publisher(
                String, f'/drone_{i}/command', 10)

        # Timer — publish setpoints at FREQUENCY_HZ
        self.create_timer(1.0 / FREQUENCY_HZ, self._publish_setpoints)

        self.get_logger().info(
            f'Planner ready. Send ARM → TAKEOFF to /fleet/command.')

    # ── Callbacks ────────────────────────────────────────────────────────────

    def _payload_cb(self, msg: MotionCaptureState):
        self.payload_pos = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        ])

    def _fleet_cmd_cb(self, msg: String):
        cmd = msg.data.strip().upper()
        self.get_logger().info(f'Fleet command: {cmd}')
        if cmd == 'ARM':
            threading.Thread(target=self._arm_all, daemon=True).start()
        elif cmd == 'TAKEOFF':
            if self.fleet_armed:
                self.flying = True
                self._publish_drone_cmd('TAKEOFF')
            else:
                self.get_logger().warn('Cannot TAKEOFF — fleet not armed.')
        elif cmd in ('DISARM', 'ESTOP'):
            self.flying = False
            self.fleet_armed = False
            threading.Thread(target=self._disarm_all, daemon=True).start()

    # ── Arming helpers ────────────────────────────────────────────────────────

    def _arm_all(self):
        for i in range(N_DRONES):
            if not self._arming_clients[i].wait_for_service(timeout_sec=5.0):
                self.get_logger().error(f'Arming service drone {i} not available.')
                return

        futures = {}
        for i in range(N_DRONES):
            req = SetArming.Request()
            req.arm = True
            futures[i] = self._arming_clients[i].call_async(req)

        deadline = time.time() + 5.0
        for i, fut in futures.items():
            rclpy.spin_until_future_complete(self, fut,
                                             timeout_sec=max(0.0, deadline - time.time()))
            if fut.done() and fut.result().success:
                self.get_logger().info(f'Drone {i} armed.')
            else:
                self.get_logger().error(f'Drone {i} arm failed.')

        self.fleet_armed = True

    def _disarm_all(self):
        for i in range(N_DRONES):
            client = self._arming_clients[i]
            if client.service_is_ready():
                req = SetArming.Request()
                req.arm = False
                fut = client.call_async(req)
                rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)

    def _publish_drone_cmd(self, cmd: str):
        msg = String(data=cmd)
        for i in range(N_DRONES):
            self._drone_cmd_pubs[i].publish(msg)

    # ── Sphere projection (rigid-cable worlds only) ───────────────────────────

    def _project_to_sphere(self, p_des: np.ndarray) -> np.ndarray:
        """
        Project p_des onto the sphere of radius tether_length centred on the
        current payload position.  This guarantees the setpoint is reachable
        under a rigid cable constraint, preventing the controller from fighting
        the physics engine.

        With no rigid constraint (tether_length == 0) the raw p_des is returned.
        """
        if self._tether_length <= 0.0:
            return p_des
        direction = p_des - self.payload_pos
        dist = np.linalg.norm(direction)
        if dist < 1e-6:
            return self.payload_pos + np.array([0.0, 0.0, self._tether_length])
        return self.payload_pos + self._tether_length * direction / dist

    # ── Setpoint publisher ────────────────────────────────────────────────────

    def _publish_setpoints(self):
        """Compute and publish desired position for each drone."""
        for i in range(N_DRONES):
            if not self.flying:
                # Hover at starting position when not flying (z=0.1m)
                p_des = np.array([0.5, 0.5, 0.1])
            else:
                ox, oy = self._offsets_xy[i]
                p_des = np.array([
                    self.payload_pos[0] + ox,
                    self.payload_pos[1] + oy,
                    self._payload_hover_z + self._cable_length,
                ])
                # For rigid-cable worlds: project onto the feasible sphere so the
                # controller never commands a position the tether cannot reach.
                p_des = self._project_to_sphere(p_des)

            msg = Float64MultiArray()
            msg.data = [float(p_des[0]), float(p_des[1]), float(p_des[2]),
                        0.0, 0.0, 0.0]
            self._setpoint_pubs[i].publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Planner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
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

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
CABLE_LENGTH = 0.75          # m — must match world_quad_payload.sdf
FREQUENCY_HZ = 30.0

# XY offsets of each drone from payload centre (formation layout, m).
# These match the drone positions in world_quad_payload.sdf.
DRONE_OFFSETS_XY = {
    0: ( 0.25,  0.25),
    1: (-0.25,  0.25),
    2: (-0.25, -0.25),
    3: ( 0.25, -0.25),
}

# Desired payload hover height (above ground, m).
PAYLOAD_HOVER_Z = 0.84       # ≈ equilibrium with drones at 1.5 m


class Planner(Node):

    def __init__(self):
        super().__init__('planner', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, False)
        ])

        # Fleet state
        self.fleet_armed = False
        self.flying = False

        # Payload state (updated from /payload/motion_capture_state)
        self.payload_pos = np.array([0.0, 0.0, PAYLOAD_HOVER_Z])

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

    # ── Setpoint publisher ────────────────────────────────────────────────────

    def _publish_setpoints(self):
        """Compute and publish desired position for each drone."""
        for i in range(N_DRONES):
            ox, oy = DRONE_OFFSETS_XY[i]
            x_des = self.payload_pos[0] + ox
            y_des = self.payload_pos[1] + oy
            # Target drone z = desired payload z + cable_length
            z_des = PAYLOAD_HOVER_Z + CABLE_LENGTH

            msg = Float64MultiArray()
            msg.data = [x_des, y_des, z_des, 0.0, 0.0, 0.0]
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

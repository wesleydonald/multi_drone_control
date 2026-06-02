"""
cable_tension_node.py — Tension-only cable simulation via gz transport
----------------------------------------------------------------------
Replaces the rigid-rod Gazebo joint with a spring-damper cable model that
can go slack.  Every control tick:

  1. Read each drone's distance to the payload (from MotionCaptureState).
  2. If dist > CABLE_LENGTH: compute spring-damper tension force.
  3. Publish force on drone (toward payload) and equal-opposite force on
     payload via the gz ApplyLinkWrench plugin's  /wrench/persistent  topic.
  4. If cable becomes slack: publish to  /wrench/clear  to remove the
     persistent force.

Persistent wrenches are applied every Gazebo physics step (~1 ms) so there
is no aliasing from the 30 Hz ROS control rate.

Entity name format used (full scoped names within the world):
  Drones:  lift_system::x3_drone{i}::base_link
  Payload: lift_system::payload::body
Adjust DRONE_LINK_NAMES / PAYLOAD_LINK_NAME if the world structure differs.

Requires gz-transport13 Python bindings at:
  /usr/lib/python3/dist-packages
"""

import sys
import numpy as np

# gz Python bindings are not on the default PYTHONPATH
sys.path.insert(0, '/usr/lib/python3/dist-packages')
from gz.transport13 import Node as GzNode                           # noqa: E402
from gz.msgs10.entity_wrench_pb2 import EntityWrench               # noqa: E402
from gz.msgs10.entity_pb2 import Entity                            # noqa: E402

import rclpy
from rclpy.node import Node
from interfaces.msg import MotionCaptureState

# ── Parameters ────────────────────────────────────────────────────────────────

CABLE_LENGTH = 0.75     # m — cable natural length
K_SPRING = 300.0        # N/m — spring constant (stiffness when taut)
K_DAMPER = 15.0         # Ns/m — damping coefficient

N_DRONES = 4
WORLD_NAME = 'quad_payload'
FREQUENCY_HZ = 30.0

# Full Gazebo entity names (lift_system > sub-model > link)
DRONE_LINK_NAMES = [f'lift_system::x3_drone{i}::base_link' for i in range(N_DRONES)]
PAYLOAD_LINK_NAME = 'lift_system::payload::body'
LINK_TYPE = 3   # gz.msgs.Entity.LINK


class CableTensionNode(Node):

    def __init__(self):
        super().__init__('cable_tension_node')

        # gz transport publishers
        self._gz = GzNode()
        self._wrench_pub = self._gz.advertise(
            f'/world/{WORLD_NAME}/wrench/persistent', EntityWrench)
        self._clear_pub = self._gz.advertise(
            f'/world/{WORLD_NAME}/wrench/clear', Entity)

        # Per-drone state
        self._drone_pos = [None] * N_DRONES
        self._drone_vel = [None] * N_DRONES
        self._taut = [False] * N_DRONES

        # Payload state
        self._payload_pos = None
        self._payload_vel = None

        # ROS 2 subscriptions
        for i in range(N_DRONES):
            self.create_subscription(
                MotionCaptureState,
                f'/drone_{i}/motion_capture_state',
                lambda msg, idx=i: self._drone_cb(msg, idx),
                5)
        self.create_subscription(
            MotionCaptureState,
            '/payload/motion_capture_state',
            self._payload_cb, 5)

        self.create_timer(1.0 / FREQUENCY_HZ, self._update)
        self.get_logger().info(
            f'Cable tension node ready. '
            f'L={CABLE_LENGTH} m  K={K_SPRING} N/m  D={K_DAMPER} Ns/m')

    # ── ROS callbacks ─────────────────────────────────────────────────────────

    def _drone_cb(self, msg: MotionCaptureState, i: int):
        self._drone_pos[i] = np.array([
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        self._drone_vel[i] = np.array([
            msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z])

    def _payload_cb(self, msg: MotionCaptureState):
        self._payload_pos = np.array([
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        self._payload_vel = np.array([
            msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z])

    # ── Cable physics update ──────────────────────────────────────────────────

    def _update(self):
        if self._payload_pos is None:
            return

        total_payload_force = np.zeros(3)

        for i in range(N_DRONES):
            if self._drone_pos[i] is None:
                continue

            diff = self._payload_pos - self._drone_pos[i]
            dist = float(np.linalg.norm(diff))

            if dist < 1e-6 or dist <= CABLE_LENGTH:
                # Cable slack — clear any previously applied persistent force
                if self._taut[i]:
                    self._clear_entity(DRONE_LINK_NAMES[i])
                    self._taut[i] = False
                continue

            # ── Cable is taut: compute spring-damper tension ──────────────────
            direction = diff / dist   # unit vector: drone → payload

            extension = dist - CABLE_LENGTH

            # Rate of extension (positive = cable stretching)
            d_ext = 0.0
            if self._drone_vel[i] is not None:
                d_ext = float(np.dot(
                    self._payload_vel - self._drone_vel[i], direction))

            T = max(0.0, K_SPRING * extension + K_DAMPER * d_ext)

            force_on_drone = T * direction      # pulls drone toward payload
            total_payload_force += -force_on_drone  # equal-opposite on payload

            self._apply_force(DRONE_LINK_NAMES[i], force_on_drone)
            self._taut[i] = True

        # Apply summed cable tension to payload
        self._apply_force(PAYLOAD_LINK_NAME, total_payload_force)

    # ── gz transport helpers ──────────────────────────────────────────────────

    def _apply_force(self, link_name: str, force: np.ndarray):
        ew = EntityWrench()
        ew.entity.name = link_name
        ew.entity.type = LINK_TYPE
        ew.wrench.force.x = float(force[0])
        ew.wrench.force.y = float(force[1])
        ew.wrench.force.z = float(force[2])
        self._wrench_pub.publish(ew)

    def _clear_entity(self, link_name: str):
        e = Entity()
        e.name = link_name
        e.type = LINK_TYPE
        self._clear_pub.publish(e)

    def on_shutdown(self):
        """Clear all persistent forces cleanly."""
        for i in range(N_DRONES):
            self._clear_entity(DRONE_LINK_NAMES[i])
        self._clear_entity(PAYLOAD_LINK_NAME)


def main(args=None):
    rclpy.init(args=args)
    node = CableTensionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.on_shutdown()
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

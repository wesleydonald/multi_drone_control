"""
free_hover.py -- reference generator for UNTETHERED drones: each drone climbs straight
up from where it sits, hovers at hover_z, and descends on LAND. No payload, no cables.

Publishes the same 12-field wire the planners do (see planner_node.py), with a_cable = 0
and a_ff = [0, 0, g], so the unchanged per-drone tracker flies it as free-flight dynamics.
The point: fly each new airframe on the SAME tracker, safety envelope and fleet manager
that will carry the payload, and read its hover throttle (kT) before anything is tethered.

    ros2 launch controller_quad_load real_hover_launch.py num_drones:=3 hover_z:=0.8
    ARM -> TAKEOFF -> hover -> LAND    (RViz panel or /fleet/command)

Each drone's xy is latched from its first mocap pose; z0 is its resting height. On
TAKEOFF the reference climbs at climb_vel to hover_z; on LAND it descends at land_vel to
z0 and, once every drone measures within land_tol of the ground, /fleet/landed is
published so the fleet manager disarms (the same contract the load planner honours).
"""
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64MultiArray, String
from interfaces.msg import MotionCaptureState

G = 9.81
RATE_HZ = 10.0
N_NODES = 21          # tracker wants >= its N + 1 = 21
DT = 0.1


class FreeHover(Node):
    def __init__(self):
        super().__init__('free_hover')
        p = self.declare_parameter
        self.n = int(p('num_drones', 3).value)
        self.hover_z = float(p('hover_z', 0.8).value)
        self.climb_vel = float(p('climb_vel', 0.15).value)
        self.land_vel = float(p('land_vel', 0.15).value)
        self.land_tol = float(p('land_tol', 0.10).value)

        self.z0 = [None] * self.n          # resting height per drone, latched on first pose
        self.xy = [None] * self.n          # hover xy per drone, latched on first pose
        self.z_meas = [None] * self.n
        self.z_ref = [None] * self.n       # current commanded height (None until posed)
        self.phase = 'ground'              # ground | climb | land
        self._landed_sent = False

        self.ref_pub = [self.create_publisher(Float64MultiArray,
                                              f'/drone_{i}/reference_trajectory', 5)
                        for i in range(self.n)]
        self.landed_pub = self.create_publisher(Bool, '/fleet/landed', 1)
        for i in range(self.n):
            self.create_subscription(MotionCaptureState, f'/drone_{i}/motion_capture_state',
                                     lambda m, i=i: self._pose_cb(m, i), 5)
        self.create_subscription(String, '/fleet/command', self._cmd_cb, 10)
        self.create_timer(1.0 / RATE_HZ, self._tick)
        self.get_logger().info(
            f'[free_hover] n={self.n} hover_z={self.hover_z:.2f} climb {self.climb_vel:.2f} '
            f'land {self.land_vel:.2f} m/s -- waiting for poses')

    def _pose_cb(self, msg, i):
        pos = msg.pose.position
        self.z_meas[i] = float(pos.z)
        if self.z0[i] is None:
            self.z0[i] = float(pos.z)
            self.xy[i] = (float(pos.x), float(pos.y))
            self.z_ref[i] = float(pos.z)
            self.get_logger().info(
                f'[free_hover] drone {i} latched at ({pos.x:.2f}, {pos.y:.2f}, z0 {pos.z:.2f})')

    def _cmd_cb(self, msg):
        c = msg.data.strip().upper()
        if c == 'TAKEOFF':
            self.phase = 'climb'
            self._landed_sent = False
            self.get_logger().info('[free_hover] TAKEOFF: climbing to hover_z')
        elif c == 'LAND':
            self.phase = 'land'
            self.get_logger().info('[free_hover] LAND: descending to the resting height')
        elif c in ('DISARM', 'ESTOP'):
            self.phase = 'ground'
            for i in range(self.n):
                if self.z0[i] is not None:
                    self.z_ref[i] = self.z0[i]

    def _rate(self):
        if self.phase == 'climb':
            return self.climb_vel
        if self.phase == 'land':
            return -self.land_vel
        return 0.0

    def _z_at(self, i, z_now, tau):
        """Commanded height tau seconds ahead: the ramp, clamped to its target."""
        z = z_now + self._rate() * tau
        if self.phase == 'climb':
            return min(z, self.hover_z)
        if self.phase == 'land':
            return max(z, self.z0[i])
        return z_now

    def _tick(self):
        for i in range(self.n):
            if self.z_ref[i] is None:
                continue                       # no pose yet: publish nothing, tracker idles
            self.z_ref[i] = self._z_at(i, self.z_ref[i], DT)
            x, y = self.xy[i]
            rate = self._rate()
            data = [float(N_NODES), DT]
            for k in range(N_NODES):
                zk = self._z_at(i, self.z_ref[i], DT * k)
                vz = rate if (self.phase == 'climb' and zk < self.hover_z) or \
                             (self.phase == 'land' and zk > self.z0[i]) else 0.0
                data += [x, y, zk, 0.0, 0.0, vz, 0.0, 0.0, G, 0.0, 0.0, 0.0]
            msg = Float64MultiArray()
            msg.data = data
            self.ref_pub[i].publish(msg)

        if self.phase == 'land' and not self._landed_sent:
            ready = [self.z0[i] is not None and self.z_meas[i] is not None
                     and self.z_ref[i] <= self.z0[i] + 1e-6
                     and self.z_meas[i] <= self.z0[i] + self.land_tol
                     for i in range(self.n)]
            if ready and all(ready):
                self._landed_sent = True
                self.landed_pub.publish(Bool(data=True))
                self.get_logger().info('[free_hover] all drones down - announcing /fleet/landed')
                self.phase = 'ground'


def main(args=None):
    rclpy.init(args=args)
    node = FreeHover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

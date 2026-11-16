"""
free_hover.py -- reference generator for UNTETHERED drones: each drone climbs straight
up from where it sits, hovers at hover_z, and descends on LAND. No payload, no cables.

Publishes the same 12-field wire the planners do (see planner_node.py), with a_cable = 0
and a_ff = [0, 0, g], so the unchanged per-drone tracker flies it as free-flight dynamics.
The point: fly each new airframe on the SAME tracker, safety envelope and fleet manager
that will carry the payload, and read its hover throttle (kT) before anything is tethered.

    ros2 launch bringup real_control_launch.py mode:=free_hover num_drones:=3 hover_z:=0.8
    ARM -> TAKEOFF -> hover -> LAND    (RViz panel or /fleet/command)

Each drone's xy is latched from its first mocap pose; z0 is its resting height. On
TAKEOFF the reference climbs at climb_vel to hover_z; on LAND it descends at land_vel to
z0 and, once every drone measures within land_tol of the ground, /fleet/landed is
published so the fleet manager disarms (the same contract the load planner honours).

circle_radius > 0 (one drone only; 0 = off): circle_hold_s after reaching hover_z the drone
circles its takeoff point counter-clockwise seen from above (circle_offset), then hovers there
again. LAND during the circle stops it where it is and descends.
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
DIFF_H = 1e-3         # s, central differences of circle_offset


def _smooth(x):
    return x * x * (3.0 - 2.0 * x)


def circle_offset(t, radius, period, laps):
    """(dx, dy) from the centre t s into the circle: one period spiralling out while the speed
    ramps up, `laps` laps at radius, one period spiralling back in to rest at the centre."""
    w = 2.0 * np.pi / period
    run = laps * period
    if t <= 0.0 or t >= 2.0 * period + run:
        return np.zeros(2)
    if t < period:
        th, rho = 0.5 * w * t * t / period, _smooth(t / period)
    elif t < period + run:
        th, rho = w * (t - 0.5 * period), 1.0
    else:
        u = t - period - run
        th, rho = w * (0.5 * period + run + u - 0.5 * u * u / period), _smooth(1.0 - u / period)
    return radius * rho * np.array([np.cos(th), np.sin(th)])


def circle_state(t, radius, period, laps):
    """Offset, velocity and acceleration of the circle at t (central differences)."""
    a, b, c = (circle_offset(t + d, radius, period, laps) for d in (-DIFF_H, 0.0, DIFF_H))
    return b, (c - a) / (2.0 * DIFF_H), (c - 2.0 * b + a) / DIFF_H ** 2


class FreeHover(Node):
    def __init__(self):
        super().__init__('free_hover')
        p = self.declare_parameter
        self.n = int(p('num_drones', 3).value)
        self.hover_z = float(p('hover_z', 0.8).value)
        self.climb_vel = float(p('climb_vel', 0.15).value)
        self.land_vel = float(p('land_vel', 0.15).value)
        self.land_tol = float(p('land_tol', 0.10).value)
        self.circle_radius = float(p('circle_radius', 0.0).value)
        self.circle_period = float(p('circle_period_s', 6.0).value)
        self.circle_laps = int(p('circle_laps', 3).value)
        self.circle_hold_s = float(p('circle_hold_s', 5.0).value)
        if self.circle_radius > 0.0 and self.n != 1:
            self.get_logger().warn(f'[free_hover] circle_radius {self.circle_radius:.2f} ignored: one drone only '
                                   f'(n={self.n}: each would circle its own takeoff point)')
            self.circle_radius = 0.0

        self.z0 = [None] * self.n          # resting height per drone, latched on first pose
        self.xy = [None] * self.n          # hover xy per drone, latched on first pose
        self.z_meas = [None] * self.n
        self.z_ref = [None] * self.n       # current commanded height (None until posed)
        self.ref_xy = [None] * self.n      # current commanded xy: the latched xy, or on the circle
        self.phase = 'ground'              # ground | climb | circle | land
        self._landed_sent = False
        self._t_at_hover = None            # first tick at hover_z after this TAKEOFF
        self._t_circle = None              # circle start
        self._circle_done = False

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
        if self.circle_radius > 0.0:
            w = 2.0 * np.pi / self.circle_period
            self.get_logger().info(
                f'[free_hover] CIRCLE armed: radius {self.circle_radius:.2f} m, {self.circle_period:.1f} s laps '
                f'({self.circle_radius * w:.2f} m/s, {self.circle_radius * w * w:.2f} m/s^2), {self.circle_laps} '
                f'laps + one period in and out, {self.circle_hold_s:.1f} s after reaching hover_z')

    def _pose_cb(self, msg, i):
        pos = msg.pose.position
        self.z_meas[i] = float(pos.z)
        if self.z0[i] is None:
            self.z0[i] = float(pos.z)
            self.xy[i] = (float(pos.x), float(pos.y))
            self.ref_xy[i] = self.xy[i]
            self.z_ref[i] = float(pos.z)
            self.get_logger().info(
                f'[free_hover] drone {i} latched at ({pos.x:.2f}, {pos.y:.2f}, z0 {pos.z:.2f})')

    def _cmd_cb(self, msg):
        c = msg.data.strip().upper()
        if c == 'TAKEOFF':
            if self.phase == 'circle':
                return
            self.phase = 'climb'
            self._landed_sent = False
            self._t_at_hover = None
            self._circle_done = False
            self.get_logger().info('[free_hover] TAKEOFF: climbing to hover_z')
        elif c == 'LAND':
            if self.phase == 'circle':
                self.get_logger().info(f'[free_hover] CIRCLE stopped by LAND at ({self.ref_xy[0][0]:.2f}, '
                                       f'{self.ref_xy[0][1]:.2f})')
            self.phase = 'land'
            self.get_logger().info('[free_hover] LAND: descending to the resting height')
        elif c in ('DISARM', 'ESTOP'):
            self.phase = 'ground'
            for i in range(self.n):
                if self.z0[i] is not None:
                    self.z_ref[i] = self.z0[i]
                    self.ref_xy[i] = self.xy[i]

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

    def _circle_tick(self, now):
        """Start the circle circle_hold_s after hover_z is reached; end it when it is back at the centre."""
        if self.circle_radius <= 0.0 or self._circle_done:
            return
        if self.phase == 'climb' and all(z is not None and z >= self.hover_z - 1e-6 for z in self.z_ref):
            if self._t_at_hover is None:
                self._t_at_hover = now
            elif now - self._t_at_hover >= self.circle_hold_s:
                self.phase, self._t_circle = 'circle', now
                self.get_logger().info(f'[free_hover] CIRCLE start around ({self.xy[0][0]:.2f}, {self.xy[0][1]:.2f})')
        elif self.phase == 'circle' and now - self._t_circle >= (self.circle_laps + 2) * self.circle_period:
            self.phase, self._circle_done = 'climb', True
            self.ref_xy[0] = self.xy[0]
            self.get_logger().info('[free_hover] CIRCLE done: hovering at the takeoff point')

    def _tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        self._circle_tick(now)
        for i in range(self.n):
            if self.z_ref[i] is None:
                continue                       # no pose yet: publish nothing, tracker idles
            self.z_ref[i] = self._z_at(i, self.z_ref[i], DT)
            rate = self._rate()
            data = [float(N_NODES), DT]
            for k in range(N_NODES):
                zk = self._z_at(i, self.z_ref[i], DT * k)
                vz = rate if (self.phase == 'climb' and zk < self.hover_z) or \
                             (self.phase == 'land' and zk > self.z0[i]) else 0.0
                (x, y), (vx, vy), (ax, ay) = self.ref_xy[i], (0.0, 0.0), (0.0, 0.0)
                if self.phase == 'circle':
                    d, v, a = circle_state(now - self._t_circle + DT * k, self.circle_radius,
                                           self.circle_period, self.circle_laps)
                    (x, y), (vx, vy), (ax, ay) = np.add(self.xy[i], d), v, a
                    if k == 0:
                        self.ref_xy[i] = (float(x), float(y))
                data += [float(x), float(y), zk, float(vx), float(vy), vz, float(ax), float(ay), G, 0.0, 0.0, 0.0]
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

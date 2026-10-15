"""Our carried ring and its carrying drones, published in Tejen's fake-world formats.

A drop-in replacement for fake_cooperative_transport_world (same topics, same message
types, same committed-trajectory builder), fed by the multi_drone_control stack instead
of an analytic path:

* the ring's CURRENT state comes from its motion capture (/payload/motion_capture_state);
* the ring's FUTURE (the rolling committed trajectory the C++ planner needs) is predicted
  from the load planner's own trajectory (/payload/trajectory_state: kind, speed, radius,
  datum, clock, hold), so the commitment is the path the ring is being flown along, not a
  fit to noisy mocap;
* the carrying drones are fixed points in the ring frame (Tejen's 'three_attached' model):
  attach point + cable at the flight elevation, per carrier azimuth.

The drone flying Tejen's mission is simply not listed as a carrier.
"""
import json
import math
from typing import List, Tuple

import rclpy
from interfaces.msg import MotionCaptureState
from std_msgs.msg import String

from tejen_mission.fake_cooperative_transport_world import (
    FakeCooperativeTransportWorld, RigidBodyState)
from mpc_planner.geometry import cw_text, tejen_plate
from mpc_planner.load_trajectory import LoadTrajectory


def _yaw_from_quat(w, x, y, z):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _rotate(q, v):
    """v rotated by the unit quaternion q = (w, x, y, z)."""
    w, x, y, z = q
    n = math.sqrt(w * w + x * x + y * y + z * z) or 1.0
    w, x, y, z = w / n, x / n, y / n, z / n
    vx, vy, vz = v
    cx, cy, cz = y * vz - z * vy, z * vx - x * vz, x * vy - y * vx
    ccx, ccy, ccz = y * cz - z * cy, z * cx - x * cz, x * cy - y * cx
    return (vx + 2.0 * (w * cx + ccx), vy + 2.0 * (w * cy + ccy), vz + 2.0 * (w * cz + ccz))


class RingBridge(FakeCooperativeTransportWorld):
    def __init__(self) -> None:
        self._traj_state = None          # latest /payload/trajectory_state (dict)
        self._traj_rx_s = None           # node time it arrived
        self._ring = None                # latest measured ring RigidBodyState
        self._ring_q = None              # its attitude (w, x, y, z)
        self._zf = None                  # attach_z_lowpass_hz: the two filter stages, and when they ran
        self._zf_t = None
        super().__init__()
        self.create_subscription(String, '/payload/trajectory_state', self._traj_cb, 5)
        self.create_subscription(MotionCaptureState, '/payload/motion_capture_state',
                                 self._ring_cb, 10)
        # shown as typed (clockwise degrees, our plate number); the parameters are his (counter-clockwise)
        self.get_logger().info(
            f'[ring_bridge] carriers (cw) {cw_text(self._carrier_azimuths())} deg, cable '
            f'{self.get_float_param("bridge_cable_len_m"):.2f} m at '
            f'{self.get_float_param("bridge_cable_elev_deg"):.0f} deg, target plate '
            f'{tejen_plate(self.ring_attachment_plate_index)} (his {self.ring_attachment_plate_index})')
        # the typed switches, so a run's log says which arm it flew (RIG-1015-a2_2 flew without its intended filter)
        self.get_logger().info(
            f"[ring_bridge] attachment twist: {str(self._param('attach_twist', '')).strip() or 'raw'}; plate height "
            f"low-pass: {float(self._param('attach_z_lowpass_hz', 0.0)) or 'off'}")

    # ── inputs ────────────────────────────────────────────────────────────────
    def _traj_cb(self, msg: String) -> None:
        try:
            self._traj_state = json.loads(msg.data)
            self._traj_rx_s = self.get_clock().now().nanoseconds * 1e-9
        except ValueError:
            pass

    def _ring_cb(self, msg: MotionCaptureState) -> None:
        p, q, v = msg.pose.position, msg.pose.orientation, msg.twist.linear
        self._ring = RigidBodyState(x=p.x, y=p.y, z=p.z, yaw=_yaw_from_quat(q.w, q.x, q.y, q.z),
                                    vx=v.x, vy=v.y, vz=v.z, yaw_rate=0.0)
        self._ring_q = (q.w, q.x, q.y, q.z)

    # ── parameters (declared lazily: the parent calls the offsets in its __init__) ─
    def _param(self, name, default):
        if not self.has_parameter(name):
            self.declare_parameter(name, default)
        return self.get_parameter(name).value

    def _carrier_azimuths(self) -> List[float]:
        return [float(a) for a in self._param('bridge_carrier_azimuths_deg', [30.0, 150.0, 270.0])]

    # ── the three overrides ──────────────────────────────────────────────────
    def get_fake_drone_offsets(self) -> List[Tuple[float, float, float]]:
        """Ring-frame position of each carrying drone: attach point on the ring radius,
        then the cable outward and up at the flight elevation."""
        r = float(self._param('bridge_attach_radius_m', 0.25))
        z0 = float(self._param('bridge_attach_z_m', 0.025))
        length = float(self._param('bridge_cable_len_m', 0.5))
        elev = math.radians(float(self._param('bridge_cable_elev_deg', 45.0)))
        out = []
        for az in self._carrier_azimuths():
            a = math.radians(az)
            rad = r + length * math.cos(elev)
            out.append((rad * math.cos(a), rad * math.sin(a), z0 + length * math.sin(elev)))
        return out

    def get_obstacle_scenario(self) -> str:
        return 'three_attached'            # carriers move rigidly with the ring

    def compute_payload_state(self, t: float) -> RigidBodyState:
        """Measured for the present; predicted from the load planner's trajectory for the
        future (the committed trajectory)."""
        now_rel = self.now_sec_since_start()
        ahead = float(t) - now_rel
        if abs(ahead) < 0.05 or self._traj_state is None:
            if self._ring is not None:
                return self._ring
            return RigidBodyState(x=0.0, y=0.0, z=0.0, yaw=0.0, vx=0.0, vy=0.0, vz=0.0,
                                  yaw_rate=0.0)
        s = self._traj_state
        traj = LoadTrajectory(s['kind'], s['speed'], s.get('distance', 1.0), s['radius'])
        age = max(0.0, self.get_clock().now().nanoseconds * 1e-9 - float(self._traj_rx_s))
        run = max(0.0, ahead + age - float(s.get('hold_s', 0.0)))
        moving = bool(s.get('running', False))
        tt = float(s['traj_t']) + (run if moving else 0.0)
        dx, dy, vx, vy = traj.offset_at(tt)
        if not moving or run <= 0.0:
            vx, vy = 0.0, 0.0
        yaw = self._ring.yaw if self._ring is not None else 0.0
        # hold the ring's MEASURED height: in a trajectory it rides ~6 cm above its
        # target (the height integral is gated off while moving), and the target was
        # the whole of the 10 s prediction error (bridge check, 2026-09-25)
        z = self._ring.z if (self._ring is not None and moving) else float(s['z'])
        return RigidBodyState(x=float(s['hover_x']) + dx, y=float(s['hover_y']) + dy,
                              z=z, yaw=yaw, vx=vx, vy=vy, vz=0.0, yaw_rate=0.0)

    def _plate_xy(self):
        """The attach plate's point in the ring frame: bridge_attach_xy_m when set (our weld point, so both stacks aim
        at one point: on 14 Oct his nominal plate and our measured point were 2.4 cm apart), else his nominal."""
        xy = [float(v) for v in self._param('bridge_attach_xy_m', [0.0])]
        if len(xy) == 2:
            return xy
        plate = self.ring_geometry.plate_body_offset(self.ring_attachment_plate_index)
        return [float(plate[0]), float(plate[1])]

    def compute_attachment_xyz(self, payload_state: RigidBodyState) -> Tuple[float, float, float]:
        z0 = float(self._param('bridge_attach_z_m', 0.025))
        if self._param('full_pose_attachment', False) and self._ring_q is not None:
            # the plate point as our magnet manager computes it (ring pose + R(q) @ offset): a
            # tilted ring moves the plate by r*sin(tilt), which a yaw-only target misses
            px, py = self._plate_xy()
            ox, oy, oz = _rotate(self._ring_q, (px, py, z0))
            x, y, z = payload_state.x + ox, payload_state.y + oy, payload_state.z + oz
        else:
            x, y, z = super().compute_attachment_xyz(payload_state)
            z += z0
        if payload_state is getattr(self, "_ring", None):   # the present, measured plate (not the predicted path)
            z = self._lowpass_z(z)
        return (x, y, z)

    def finite_difference_attachment_twist(self, t, payload_state, attachment_xyz):
        """attach_twist 'zero' (typed only; '' = the base's 50 Hz finite difference): a zero attachment twist, as his
        measured-ring mode publishes. His reference extrapolates this twist over its horizon and his match gate compares
        QUAD4's velocity with it; differenced from the raw ring pose it carried the 1.8 Hz bounce (+-0.13 m/s, RIG-1015-a1)
        and 0.09 m/s of sideways jitter (a3_2). Hover only: a moving ring needs its velocity."""
        moving = bool((getattr(self, '_traj_state', None) or {}).get('running', False))
        if str(self._param('attach_twist', '')).strip().lower() == 'zero' and not moving:
            return 0.0, 0.0, 0.0, 0.0
        return FakeCooperativeTransportWorld.finite_difference_attachment_twist(self, t, payload_state, attachment_xyz)

    def _lowpass_z(self, z: float) -> float:
        """attach_z_lowpass_hz > 0 (0 = off): the plate height through two first-order lags at that cutoff. On
        15 Oct (a1) the ring bounced ~0.8 cm at 1.8 Hz, his reference copied it and QUAD4 followed half a cycle late,
        doubling the tip's bounce against the plate; x and y stay raw (QUAD4 barely followed them)."""
        fc = float(self._param('attach_z_lowpass_hz', 0.0))
        if fc <= 0.0:
            return z
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._zf is None or now - self._zf_t > 0.5:
            self._zf, self._zf_t = [z, z], now
            return z
        dt = now - self._zf_t
        a = dt / (1.0 / (2.0 * math.pi * fc) + dt)
        self._zf[0] += a * (z - self._zf[0])
        self._zf[1] += a * (self._zf[0] - self._zf[1])
        self._zf_t = now
        return self._zf[1]


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RingBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""
fake_mocap.py -- MotionCaptureState feed for desk-testing the HARDWARE launches.

The real launches block until mocap streams (each tracker waits for its first pose
while holding the acados compile lock, so without mocap only one tracker ever comes
up). This publishes fixed poses for num_drones drones (+ one newcomer with --attach)
and the payload at 50 Hz, so real_*_launch.py can be brought up whole, parameters read
back (tools/preflight.py), and topics inspected, with no rig. Nothing here flies: the
poses never move, so ARM/TAKEOFF must not be sent against it, except with --script against
real_io:=false (no radios), which is what --script is for.

    tools/fake_mocap.py --num-drones 3 --attach          # 3 tethered + drone 3
    tools/fake_mocap.py --num-drones 3 --cable-len 0.47 --azimuths-deg 30,150,270 \
        --yaw-deg 0 --drone-z 0.08 --load-z 0.05 --dry-run   # rig-like creep start, print only
    tools/fake_mocap.py --rig --num-drones 3 --attach --tip --azimuths-deg 270,30,150 --drone-z 0.08
        # the M1 desk rehearsal: rig geometry, QUAD4 on the floor with its magnet tip (/magnet_tip_pose)
    tools/fake_mocap.py --rig --num-drones 3 --attach --tip --azimuths-deg 270,30,150 --drone-z 0.08 \
        --tip-plate 3 --tip-dz 0.02     # the props-off attach check: QUAD4 held, its tip 2 cm above
                                        # plate 3's weld point (--tip-dz <weld_point_z>: on the plate)
    tools/fake_mocap.py --rig --num-drones 3 --attach --tip --azimuths-deg 270,30,150 --load-z 1.0 \
        --tip-plate 3 --script caught --script-delay 1e9 --ball 1.0,1.6
        # the combined M1 contact attach (desk): carriers hovering on the cone, the ring at 1.0 m, QUAD4
        # hanging over plate 3; `kill -USR1 <pid>` starts the contact script (and restarts it)

Azimuths are anticlockwise degrees in the LOAD frame, plate 0 on the payload's +x and plate k at
+30*k deg, counter-clockwise like the azimuths (so 30/90/150/270 are plates 1/3/5/9; clockwise
8-14 Oct). Drone i sits at the i-th azimuth; --yaw-deg rotates the payload and the whole ring
with it. --drone-z puts the drones at that height with the rod still cable_len long (rods near
flat, the creep floor start) instead of on the --elev-deg cone. --rig takes the rig geometry
(rod 0.55, magnets at r 0.225, 60 deg); --tip adds the newcomer's magnet tip resting on the floor
--tip-len from its body (0.58: QUAD4's pivot-to-plate 0.54 m in the 8 Oct logs + the 4 cm pivot);
--tip-plate k puts that tip over plate k instead, --tip-dz above the ring origin plane, QUAD4 hanging
straight above it.

--script caught|uncaught (needs --tip-plate): the combined M1 contact attach as the rig's slack tether
makes it (card 2026-10-10_combined_m1_contact; the twin's rod is rigid). QUAD4 hangs with the tip
--approach-dz above the plate point (tether taut at --taut-len), descends at --descend-rate until the
tip touches the plate, goes on --overtravel lower with the tip resting on the plate (the tether slack:
Tejen aims the face 1 cm below the plate), rests --rest-s, then rises at --rise-rate. The tether is taut
again after --overtravel of rise; then caught: the plate rises with the tip (the ring tilts about its
centre) by --lift-mm and the body stops there; uncaught: the tip lifts off with the body, up to
--rise-max. --noise-mm adds Gaussian noise to every published position (seeded). --ball X,Y publishes
his pickup ball (/pickup/motion_capture_state) resting on the floor there.
"""
import argparse
import math
import signal
import time


def parse_azimuths(spec, n):
    """'30,150,270' -> [30.0, 150.0, 270.0]; None/'' -> the even ring 360*i/n."""
    if spec is None or not str(spec).strip():
        return [360.0 * i / n for i in range(n)]
    az = [float(t) for t in str(spec).replace(' ', '').split(',') if t]
    if len(az) != n:
        raise ValueError(f'--azimuths-deg has {len(az)} entries for --num-drones {n}')
    return az


TIP_TOPIC = '/magnet_tip_pose'
LOAD_TOPIC = '/payload/motion_capture_state'
BALL_TOPIC = '/pickup/motion_capture_state'
BALL_Z = 0.011                # his 22 mm puck's body origin resting on the floor
RIG = {'cable_len': 0.55, 'attach_radius': 0.225, 'elev_deg': 60.0}


def compute_poses(n, attach=False, cable_len=0.5, radius=0.25, elev_deg=45.0, load_z=0.05,
                  azimuths_deg=None, yaw_deg=0.0, drone_z=None, tip_len=None, tip_plate=None, tip_dz=0.0):
    """Static poses {topic: ((x, y, z), (qw, qx, qy, qz))}. The defaults reproduce the
    original even ring on the 45-deg cone with the payload at yaw 0."""
    az = parse_azimuths(azimuths_deg, n)
    yaw = math.radians(yaw_deg)
    if drone_z is None:
        el = math.radians(elev_deg)
        horiz, z = cable_len * math.cos(el), load_z + cable_len * math.sin(el)
    else:
        dz = drone_z - load_z
        if abs(dz) > cable_len:
            raise ValueError(f'--drone-z {drone_z} is more than cable_len above/below the load')
        horiz, z = math.sqrt(cable_len ** 2 - dz ** 2), drone_z
    level = (1.0, 0.0, 0.0, 0.0)
    poses = {}
    for i in range(n):
        a = math.radians(az[i]) + yaw
        r = radius + horiz
        poses[f'/drone_{i}/motion_capture_state'] = ((r * math.cos(a), r * math.sin(a), z), level)
    if attach:
        poses[f'/drone_{n}/motion_capture_state'] = ((0.0, -1.2, 0.12), level)
        if tip_len and tip_plate is not None:
            a = math.radians(30.0 * tip_plate) + yaw
            tip = (radius * math.cos(a), radius * math.sin(a), load_z + tip_dz)
            poses[TIP_TOPIC] = (tip, level)
            poses[f'/drone_{n}/motion_capture_state'] = ((tip[0], tip[1], tip[2] + tip_len), level)
        elif tip_len:
            dz = 0.12 - 0.02                    # the tip rests on the floor, outward of the body
            poses[TIP_TOPIC] = ((0.0, -1.2 - math.sqrt(max(tip_len ** 2 - dz ** 2, 0.0)), 0.02), level)
    poses[LOAD_TOPIC] = (
        (0.0, 0.0, load_z), (math.cos(0.5 * yaw), 0.0, 0.0, math.sin(0.5 * yaw)))
    return poses


def _qmul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz, aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx, aw * bz + ax * by - ay * bx + az * bw)


def _qrot(q, v):
    w, x, y, z = q
    vx, vy, vz = v
    cx, cy, cz = y * vz - z * vy, z * vx - x * vz, x * vy - y * vx
    ccx, ccy, ccz = y * cz - z * cy, z * cx - x * cz, x * cy - y * cx
    return (vx + 2.0 * (w * cx + ccx), vy + 2.0 * (w * cy + ccy), vz + 2.0 * (w * cz + ccz))


class ContactScript:
    """QUAD4's body and magnet marker and the ring's attitude over a contact attach on a slack tether.

    The plate point is the magnet manager's: ring centre + R(q) @ (r cos a, r sin a, plate_dz) in the ring
    frame (a = the plate's load-frame azimuth). poses(t) -> (body xyz, tip xyz, ring quaternion wxyz);
    t < 0 is the hanging start."""

    def __init__(self, kind, center, yaw, radius, plate_az, plate_dz, taut_len, approach_dz=0.10,
                 descend_rate=0.08, overtravel=0.01, rest_s=1.0, rise_rate=0.02, lift=0.005, rise_max=0.03):
        if kind not in ('caught', 'uncaught'):
            raise ValueError(f'script {kind!r}: caught or uncaught')
        self.kind, self.center = kind, tuple(center)
        self.q_yaw = (math.cos(0.5 * yaw), 0.0, 0.0, math.sin(0.5 * yaw))
        self.offset = (radius * math.cos(plate_az), radius * math.sin(plate_az), plate_dz)
        # tilting about the horizontal axis across the plate's direction raises the plate (world frame)
        u = _qrot(self.q_yaw, (math.cos(plate_az), math.sin(plate_az), 0.0))
        self.axis = (u[1], -u[0], 0.0)
        self.radius, self.taut = radius, taut_len
        self.approach_dz, self.descend_rate = approach_dz, descend_rate
        self.overtravel, self.rest_s, self.rise_rate = overtravel, rest_s, rise_rate
        self.lift, self.rise_max = lift, rise_max
        self.t_contact = approach_dz / descend_rate                 # the tip reaches the plate
        self.t_rest = self.t_contact + overtravel / descend_rate    # the body at its lowest
        self.t_lift = self.t_rest + rest_s                          # the rise starts

    def ring_q(self, rise):
        """The ring attitude that raises the plate point by `rise` (m)."""
        th = math.asin(max(-1.0, min(1.0, rise / self.radius)))
        s = math.sin(0.5 * th)
        return _qmul((math.cos(0.5 * th), self.axis[0] * s, self.axis[1] * s, 0.0), self.q_yaw)

    def plate(self, q):
        o = _qrot(q, self.offset)
        return tuple(c + d for c, d in zip(self.center, o))

    def poses(self, t):
        q = self.q_yaw
        p = self.plate(q)
        up = lambda pt, dz: (pt[0], pt[1], pt[2] + dz)  # noqa: E731
        if t < self.t_contact:                                      # hanging, taut, coming down
            tip = up(p, self.approach_dz - self.descend_rate * max(t, 0.0))
            return up(tip, self.taut), tip, q
        if t < self.t_lift:                                         # tip on the plate, the body lower
            low = min(t, self.t_rest) - self.t_contact
            return up(p, self.taut - self.descend_rate * low), p, q
        rise = self.rise_rate * (t - self.t_lift)                   # the body rises from its lowest
        body_rest = p[2] + self.taut - self.overtravel
        if self.kind == 'caught':
            rise = min(rise, self.overtravel + self.lift)
            h = max(0.0, rise - self.overtravel)
            q = self.ring_q(h)
            tip = self.plate(q)
            # slack: the body rises alone; taut: it carries the plate (and the tip on it) up
            return (tip[0], tip[1], body_rest + rise if h <= 0.0 else tip[2] + self.taut), tip, q
        rise = min(rise, self.rise_max)
        tip = up(p, max(0.0, rise - self.overtravel))
        return (p[0], p[1], body_rest + rise), tip, q


def run(poses, hz=50.0, script=None, newcomer=None, noise_m=0.0, seed=1, script_delay=0.0):
    import random

    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import PoseStamped
    from interfaces.msg import MotionCaptureState

    rng = random.Random(seed)

    class FakeMocap(Node):
        def __init__(self):
            super().__init__('fake_mocap')
            self.pubs = {t: self.create_publisher(PoseStamped if t == TIP_TOPIC else MotionCaptureState, t, 10)
                         for t in poses}
            self.t0 = time.monotonic() + script_delay
            self.last = {}
            self.stage = None
            self.create_timer(1.0 / hz, self._tick)
            self.get_logger().info(
                f'fake mocap: {len(poses)} bodies at {hz:.0f} Hz '
                + (f'(script {script.kind}: starts in {script_delay:.0f} s or on SIGUSR1; contact at '
                   f'{script.t_contact:.1f} s, lift from {script.t_lift:.1f} s)' if script else '(static)'))

        def restart(self):
            self.t0 = time.monotonic()
            self.get_logger().info(f'script {script.kind}: (re)started')

        def _current(self):
            if script is None:
                return poses
            t = time.monotonic() - self.t0
            body, tip, q = script.poses(t)
            stage = ('hanging' if t < 0 else 'descending' if t < script.t_contact else
                     'on the plate' if t < script.t_lift else 'lifting')
            if stage != self.stage:
                self.stage = stage
                self.get_logger().info(f'script {script.kind}: {stage} (t {max(t, 0.0):.2f} s)')
            out = dict(poses)
            out[newcomer] = (body, (1.0, 0.0, 0.0, 0.0))
            out[TIP_TOPIC] = (tip, (1.0, 0.0, 0.0, 0.0))
            out[LOAD_TOPIC] = (poses[LOAD_TOPIC][0], q)
            return out

        def _tick(self):
            now = self.get_clock().now()
            stamp, t = now.to_msg(), now.nanoseconds * 1e-9
            for topic, (xyz, (qw, qx, qy, qz)) in self._current().items():
                x, y, z = (c + (rng.gauss(0.0, noise_m) if noise_m > 0.0 else 0.0) for c in xyz)
                m = PoseStamped() if topic == TIP_TOPIC else MotionCaptureState()
                m.header.stamp = stamp
                m.header.frame_id = 'map'
                m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, z
                m.pose.orientation.w, m.pose.orientation.x = qw, qx
                m.pose.orientation.y, m.pose.orientation.z = qy, qz
                if topic != TIP_TOPIC:
                    prev = self.last.get(topic)
                    if prev is not None and t > prev[0]:
                        dt = t - prev[0]
                        m.twist.linear.x, m.twist.linear.y, m.twist.linear.z = (
                            (c - pc) / dt for c, pc in zip(xyz, prev[1]))
                    self.last[topic] = (t, xyz)
                self.pubs[topic].publish(m)

    rclpy.init()
    node = FakeMocap()
    if script is not None:
        signal.signal(signal.SIGUSR1, lambda *_: node.restart())
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--num-drones', type=int, default=3)
    ap.add_argument('--attach', action='store_true', help='also publish the newcomer (id num_drones)')
    ap.add_argument('--cable-len', type=float, default=None, help='rod length (m); default 0.5, --rig 0.55')
    ap.add_argument('--attach-radius', type=float, default=None, help='plate radius (m); default 0.25, --rig 0.225')
    ap.add_argument('--elev-deg', type=float, default=None, help='rod elevation (deg); default 45, --rig 60')
    ap.add_argument('--rig', action='store_true', help='rig geometry: rod 0.55, plates at r 0.225, 60 deg')
    ap.add_argument('--tip', action='store_true', help='with --attach: also publish the newcomer magnet tip')
    ap.add_argument('--tip-len', type=float, default=0.58, help='body-to-tip length of the newcomer (m)')
    ap.add_argument('--tip-plate', type=int, default=None,
                    help='with --tip: the tip over this plate (0-11, counter-clockwise from +x), QUAD4 above it')
    ap.add_argument('--tip-dz', type=float, default=0.0,
                    help='with --tip-plate: tip height above the ring origin plane (m)')
    ap.add_argument('--load-z', type=float, default=0.05)
    ap.add_argument('--azimuths-deg', default='',
                    help="load-frame azimuth per drone, e.g. 30,150,270 (plates 1/5/9); '' = even ring")
    ap.add_argument('--yaw-deg', type=float, default=0.0, help='payload yaw (the ring rotates with it)')
    ap.add_argument('--drone-z', type=float, default=None,
                    help='drone height (m) with the rod kept at cable_len, e.g. resting on the floor '
                         'for the creep start; default: on the --elev-deg cone')
    ap.add_argument('--hz', type=float, default=None, help='publish rate (Hz); default 50, --script 100')
    ap.add_argument('--ball', default='', help="X,Y: his pickup ball resting on the floor there")
    ap.add_argument('--script', choices=('caught', 'uncaught'), default=None,
                    help='with --tip-plate: the contact attach on a slack tether (see above)')
    ap.add_argument('--script-delay', type=float, default=5.0, help='s before the script starts (SIGUSR1 restarts)')
    ap.add_argument('--taut-len', type=float, default=None, help='taut body-to-tip (m); default --tip-len')
    ap.add_argument('--approach-dz', type=float, default=0.10, help='tip above the plate at the start (m)')
    ap.add_argument('--descend-rate', type=float, default=0.08, help='m/s (his descend_rate default)')
    ap.add_argument('--overtravel', type=float, default=0.01, help='body below the contact height (m)')
    ap.add_argument('--rest-s', type=float, default=1.0, help='s on the plate before the rise')
    ap.add_argument('--rise-rate', type=float, default=0.02, help='m/s (his LIFT_PROOF)')
    ap.add_argument('--lift-mm', type=float, default=5.0, help='caught: the plate rise (mm)')
    ap.add_argument('--rise-max', type=float, default=0.03, help='uncaught: the body rise cap (m)')
    ap.add_argument('--noise-mm', type=float, default=0.0, help='Gaussian position noise (mm sd)')
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--dry-run', action='store_true', help='print the poses and exit (no ROS)')
    a = ap.parse_args(argv)
    base = RIG if a.rig else {'cable_len': 0.5, 'attach_radius': 0.25, 'elev_deg': 45.0}
    for k, v in base.items():
        if getattr(a, k) is None:
            setattr(a, k, v)
    if a.tip and not a.attach:
        ap.error('--tip needs --attach (the tip belongs to the newcomer)')
    if a.tip_plate is not None and not a.tip:
        ap.error('--tip-plate needs --tip')
    if a.script and a.tip_plate is None:
        ap.error('--script needs --tip --tip-plate')
    poses = compute_poses(a.num_drones, a.attach, a.cable_len, a.attach_radius, a.elev_deg,
                          a.load_z, a.azimuths_deg, a.yaw_deg, a.drone_z,
                          tip_len=a.tip_len if a.tip else None, tip_plate=a.tip_plate, tip_dz=a.tip_dz)
    if a.ball:
        bx, by = (float(v) for v in a.ball.split(','))
        poses[BALL_TOPIC] = ((bx, by, BALL_Z), (1.0, 0.0, 0.0, 0.0))
    script = None
    if a.script:
        script = ContactScript(
            a.script, poses[LOAD_TOPIC][0], math.radians(a.yaw_deg), a.attach_radius,
            math.radians(30.0 * a.tip_plate), a.tip_dz, a.taut_len or a.tip_len,
            approach_dz=a.approach_dz, descend_rate=a.descend_rate, overtravel=a.overtravel,
            rest_s=a.rest_s, rise_rate=a.rise_rate, lift=a.lift_mm / 1000.0, rise_max=a.rise_max)
    if a.dry_run:
        for t, ((x, y, z), q) in poses.items():
            print(f'{t:36s} xyz ({x:+.3f}, {y:+.3f}, {z:+.3f})  q(wxyz) '
                  f'({q[0]:+.4f}, {q[1]:+.4f}, {q[2]:+.4f}, {q[3]:+.4f})')
        if script:
            for t in (-1.0, script.t_contact, script.t_rest, script.t_lift, script.t_lift + 1.0,
                      script.t_lift + 2.0):
                b, tip, q = script.poses(t)
                print(f'script {a.script} t {t:5.2f}: body z {b[2]:.4f}, tip z {tip[2]:.4f}, '
                      f'body-to-tip {b[2] - tip[2]:.4f}')
        return
    run(poses, hz=a.hz or (100.0 if script else 50.0), script=script,
        newcomer=f'/drone_{a.num_drones}/motion_capture_state', noise_m=a.noise_mm / 1000.0, seed=a.seed,
        script_delay=a.script_delay)


if __name__ == '__main__':
    main()

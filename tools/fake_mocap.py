#!/usr/bin/env python3
"""
fake_mocap.py -- static MotionCaptureState feed for desk-testing the HARDWARE launches.

The real launches block until mocap streams (each tracker waits for its first pose
while holding the acados compile lock, so without mocap only one tracker ever comes
up). This publishes fixed poses for num_drones drones (+ one newcomer with --attach)
and the payload at 50 Hz, so real_*_launch.py can be brought up whole, parameters read
back (tools/preflight.py), and topics inspected, with no rig. Nothing here flies: the
poses never move, so ARM/TAKEOFF must not be sent against it.

    tools/fake_mocap.py --num-drones 3 --attach          # 3 tethered + drone 3
    tools/fake_mocap.py --num-drones 3 --cable-len 0.47 --azimuths-deg 30,150,270 \
        --yaw-deg 0 --drone-z 0.08 --load-z 0.05 --dry-run   # rig-like creep start, print only
    tools/fake_mocap.py --rig --num-drones 3 --attach --tip --azimuths-deg 270,30,150 --drone-z 0.08
        # the M1 desk rehearsal: rig geometry, QUAD4 on the floor with its magnet tip (/magnet_tip_pose)

Azimuths are anticlockwise degrees in the LOAD frame, plate 0 on the payload's +x; the plates
are numbered CLOCKWISE since 8 Oct (plate k at -30*k deg, so 30/90/150/270 are plates
11/9/7/3). Drone i sits at the i-th azimuth; --yaw-deg rotates the payload and the whole ring
with it. --drone-z puts the drones at that height with the rod still cable_len long (rods near
flat, the creep floor start) instead of on the --elev-deg cone. --rig takes the rig geometry
(rod 0.55, magnets at r 0.225, 60 deg); --tip adds the newcomer's magnet tip resting on the floor
--tip-len from its body (0.58: QUAD4's pivot-to-plate 0.54 m in the 8 Oct logs + the 4 cm pivot).
"""
import argparse
import math


def parse_azimuths(spec, n):
    """'30,150,270' -> [30.0, 150.0, 270.0]; None/'' -> the even ring 360*i/n."""
    if spec is None or not str(spec).strip():
        return [360.0 * i / n for i in range(n)]
    az = [float(t) for t in str(spec).replace(' ', '').split(',') if t]
    if len(az) != n:
        raise ValueError(f'--azimuths-deg has {len(az)} entries for --num-drones {n}')
    return az


TIP_TOPIC = '/magnet_tip_pose'
RIG = {'cable_len': 0.55, 'attach_radius': 0.225, 'elev_deg': 60.0}


def compute_poses(n, attach=False, cable_len=0.5, radius=0.25, elev_deg=45.0, load_z=0.05,
                  azimuths_deg=None, yaw_deg=0.0, drone_z=None, tip_len=None):
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
        if tip_len:
            dz = 0.12 - 0.02                    # the tip rests on the floor, outward of the body
            poses[TIP_TOPIC] = ((0.0, -1.2 - math.sqrt(max(tip_len ** 2 - dz ** 2, 0.0)), 0.02), level)
    poses['/payload/motion_capture_state'] = (
        (0.0, 0.0, load_z), (math.cos(0.5 * yaw), 0.0, 0.0, math.sin(0.5 * yaw)))
    return poses


def run(poses):
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import PoseStamped
    from interfaces.msg import MotionCaptureState

    class FakeMocap(Node):
        def __init__(self):
            super().__init__('fake_mocap')
            self.pubs = {t: self.create_publisher(PoseStamped if t == TIP_TOPIC else MotionCaptureState, t, 10)
                         for t in poses}
            self.create_timer(0.02, self._tick)
            self.get_logger().info(f'fake mocap: {len(poses)} bodies at 50 Hz (static)')

        def _tick(self):
            now = self.get_clock().now().to_msg()
            for t, ((x, y, z), (qw, qx, qy, qz)) in poses.items():
                m = PoseStamped() if t == TIP_TOPIC else MotionCaptureState()
                m.header.stamp = now
                m.header.frame_id = 'map'
                m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, z
                m.pose.orientation.w, m.pose.orientation.x = qw, qx
                m.pose.orientation.y, m.pose.orientation.z = qy, qz
                self.pubs[t].publish(m)

    rclpy.init()
    node = FakeMocap()
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
    ap.add_argument('--load-z', type=float, default=0.05)
    ap.add_argument('--azimuths-deg', default='',
                    help="load-frame azimuth per drone, e.g. 30,150,270 (plates 1/5/9); '' = even ring")
    ap.add_argument('--yaw-deg', type=float, default=0.0, help='payload yaw (the ring rotates with it)')
    ap.add_argument('--drone-z', type=float, default=None,
                    help='drone height (m) with the rod kept at cable_len, e.g. resting on the floor '
                         'for the creep start; default: on the --elev-deg cone')
    ap.add_argument('--dry-run', action='store_true', help='print the poses and exit (no ROS)')
    a = ap.parse_args(argv)
    base = RIG if a.rig else {'cable_len': 0.5, 'attach_radius': 0.25, 'elev_deg': 45.0}
    for k, v in base.items():
        if getattr(a, k) is None:
            setattr(a, k, v)
    if a.tip and not a.attach:
        ap.error('--tip needs --attach (the tip belongs to the newcomer)')
    poses = compute_poses(a.num_drones, a.attach, a.cable_len, a.attach_radius, a.elev_deg,
                          a.load_z, a.azimuths_deg, a.yaw_deg, a.drone_z,
                          tip_len=a.tip_len if a.tip else None)
    if a.dry_run:
        for t, ((x, y, z), q) in poses.items():
            print(f'{t:36s} xyz ({x:+.3f}, {y:+.3f}, {z:+.3f})  q(wxyz) '
                  f'({q[0]:+.4f}, {q[1]:+.4f}, {q[2]:+.4f}, {q[3]:+.4f})')
        return
    run(poses)


if __name__ == '__main__':
    main()

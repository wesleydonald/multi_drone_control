#!/usr/bin/env python3
"""Stage-1 torque test (2026-09-26): can the sim rate loop hold the tether-lever torque?

One X3 (his model, tether hanging free) hovers at 1.3 m under his sim Betaflight bridge and
a plain attitude/position hold. A constant body torque about x is applied through Gazebo's
ApplyLinkWrench: 0.031 N m (the lever at 70 deg rods), then 0.084 N m (at 45 deg), then off.

    python3 tools/sim_test/stage1_torque_test.py OUTDIR --kp 0.5 --ki 0.0

Needs both setup files sourced. Writes OUTDIR/log.csv and prints one summary line per phase.
"""
import argparse
import csv
import math
import os
import re
import signal
import subprocess
import time

import rclpy
from geometry_msgs.msg import PoseArray
from rclpy.node import Node

from interfaces.msg import ELRSCommand

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
WORLD = os.path.join(REPO, 'simulation_assets/tejen/stage1_torque_test.sdf')
U_HOVER = 0.643 * 9.81 / (4 * 0.62e-6 * 4631.0 ** 2)
PHASES = [(0.0, 6.0, 0.0), (6.0, 14.0, 0.031), (14.0, 22.0, 0.084), (22.0, 26.0, 0.0)]

ap = argparse.ArgumentParser()
ap.add_argument('outdir')
ap.add_argument('--kp', type=float, default=0.5)
ap.add_argument('--ki', type=float, default=0.0)
ap.add_argument('--rate-source', default='imu', choices=['imu', 'pose'],
                help="bridge rate loop: the X3 gyro (sim default since 2026-09-28) or poses")
ap.add_argument('--axis', default='x')
ap.add_argument('--model', default='modelLargeM2BallMagnet_free.sdf',
                help='X3 variant in simulation_assets/tejen')
ap.add_argument('--mode', default='torque', choices=['torque', 'swing'])
ap.add_argument('--pivot-z', type=float, default=-0.04, help='tether pivot in the body frame')
args = ap.parse_args()
os.makedirs(args.outdir, exist_ok=True)
if args.mode == 'swing':      # stage 2: kick the tether tip (0.006 N along x for 0.2 s: ~0.4 m/s at the 3 g tip), watch it settle
    PHASES = [(0.0, 6.0, 0.0), (6.0, 6.2, 0.006), (6.2, 20.0, 0.0)]
world_txt = open(WORLD).read().replace('modelLargeM2BallMagnet_free.sdf', args.model)
WORLD = os.path.join(os.path.abspath(args.outdir), 'world.sdf')
open(WORLD, 'w').write(world_txt)
procs = []


def spawn(cmd, name):
    fh = open(os.path.join(args.outdir, f'{name}.log'), 'w')
    p = subprocess.Popen(cmd, stdout=fh, stderr=subprocess.STDOUT, start_new_session=True)
    procs.append(p)
    return p


def stop_all():
    for p in procs:
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGINT)
        except Exception:
            pass
    for p in procs:
        try:
            p.wait(timeout=15)
        except Exception:
            os.killpg(os.getpgid(p.pid), signal.SIGKILL)


def gz(*a, timeout=10):
    return subprocess.run(['gz', *a], capture_output=True, text=True, timeout=timeout).stdout


env_path = ':'.join([os.path.join(REPO, 'simulation_assets/tejen'),
                     os.path.join(REPO, 'simulation_assets'),
                     os.environ.get('GZ_SIM_RESOURCE_PATH', '')])
os.environ['GZ_SIM_RESOURCE_PATH'] = env_path
spawn(['gz', 'sim', '-s', WORLD], 'gz')                  # paused until the loop is up
for _ in range(60):
    if 'x3' in gz('topic', '-l'):
        break
    time.sleep(1.0)
time.sleep(2.0)

# base_link index in the pose vector (ROS PoseArray keeps the order, drops the names)
echo = subprocess.Popen(['gz', 'topic', '-e', '-t', '/model/x3/pose', '-n', '1'],
                        stdout=subprocess.PIPE, text=True)
time.sleep(1.0)
gz('service', '-s', '/world/quadcopter/control', '--reqtype', 'gz.msgs.WorldControl',
   '--reptype', 'gz.msgs.Boolean', '--timeout', '3000', '--req', 'multi_step: 5')
names = re.findall(r'name: "([^"]+)"', echo.communicate(timeout=30)[0])
idx = names.index('x3')                  # link poses are model-relative; the model pose is world
tip_idx = next(i for i, n in enumerate(names) if n.endswith('magnet_tip_link'))

GZ_IMU = '/world/quadcopter/model/x3/link/X3/base_link/sensor/imu_sensor/imu'
spawn(['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
       '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
       '/model/x3/pose@geometry_msgs/msg/PoseArray[gz.msgs.Pose_V',
       '/X3/gazebo/command/motor_speed@actuator_msgs/msg/Actuators]gz.msgs.Actuators',
       GZ_IMU + '@sensor_msgs/msg/Imu[gz.msgs.IMU'], 'bridge')
spawn(['ros2', 'run', 'simulation_communication', 'tejen_betaflight_communication',
       '--ros-args', '-r', '__ns:=/t1', '-p', 'use_sim_time:=true',
       '-p', 'pose_topic:=/model/x3/pose', '-p', f'pose_index:={idx}',
       '-p', 'elrs_command_topic:=ELRSCommand',
       '-p', 'motor_command_topic:=/X3/gazebo/command/motor_speed',
       '-p', 'rates_d_val:=100.0', '-p', 'rates_f_val:=100.0', '-p', 'rates_g_val:=0.0',
       '-p', f'rate_kp:={args.kp}', '-p', f'rate_ki:={args.ki}',
       '-p', f'rate_source:={args.rate_source}', '-p', f'imu_topic:={GZ_IMU}'], 'bf')


def quat_rpy(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


class Hold(Node):
    """Attitude P (rate command = 5 x angle error, deg/s) under a soft position hold."""

    def __init__(self):
        super().__init__('stage1_hold', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.pub = self.create_publisher(ELRSCommand, '/t1/ELRSCommand', 10)
        self.create_subscription(PoseArray, '/model/x3/pose', self.cb, 10)
        self.p0 = None
        self.prev = None
        self.iz = 0.0
        self.rows = []
        self.done = False

    def cb(self, msg):
        if len(msg.poses) <= idx:
            return
        t = self.get_clock().now().nanoseconds * 1e-9
        p = msg.poses[idx].position
        roll, pitch, yaw = quat_rpy(msg.poses[idx].orientation)
        if self.p0 is None:
            self.p0 = (p.x, p.y, 1.3)
        if self.prev is None or t - self.prev[0] <= 0.0:
            self.prev = (t, p.x, p.y, p.z)
            return
        dt = t - self.prev[0]
        vx, vy, vz = ((p.x - self.prev[1]) / dt, (p.y - self.prev[2]) / dt,
                      (p.z - self.prev[3]) / dt)
        self.prev = (t, p.x, p.y, p.z)
        ax = 1.0 * (self.p0[0] - p.x) - 1.2 * vx
        ay = 1.0 * (self.p0[1] - p.y) - 1.2 * vy
        lim = math.radians(10.0)
        roll_des = max(-lim, min(lim, -ay / 9.81))
        pitch_des = max(-lim, min(lim, ax / 9.81))
        ez = self.p0[2] - p.z
        self.iz = max(-0.05, min(0.05, self.iz + 0.3 * ez * dt))
        u = U_HOVER * (1.0 + 1.5 * ez - 1.0 * vz) + self.iz
        u /= max(0.5, math.cos(roll) * math.cos(pitch))
        k = 5.0
        cmd = ELRSCommand()
        cmd.armed = True
        cmd.channel_0 = max(-1.0, min(1.0, k * math.degrees(roll_des - roll) / 100.0))
        cmd.channel_1 = max(-1.0, min(1.0, k * math.degrees(pitch_des - pitch) / 100.0))
        cmd.channel_2 = 2.0 * max(0.0, min(1.0, u)) - 1.0
        cmd.channel_3 = -max(-1.0, min(1.0, k * math.degrees(-yaw) / 100.0))
        self.pub.publish(cmd)
        tp = msg.poses[tip_idx].position
        dx, dy, dz = tp.x, tp.y, tp.z - args.pivot_z
        teth = math.degrees(math.atan2(math.hypot(dx, dy), -dz))
        self.rows.append((t, math.degrees(roll), math.degrees(pitch), math.degrees(yaw),
                          p.x, p.y, p.z, cmd.channel_0, cmd.channel_2, teth))
        ts = t - self.rows[0][0]
        want = next((tau for a, b, tau in PHASES if a <= ts < b), None)
        if want is None:
            self.done = True
        elif want != _applied_total[0]:
            set_torque(want)
            print(f'  t={ts:5.2f} s torque {args.axis} = {want} N m', flush=True)


# one long-lived gz publisher: a one-shot `gz topic -p` can drop its message before the
# connection forms (the first stage-1 attempt applied its torques late or never)
from gz.msgs10.entity_pb2 import Entity  # noqa: E402
from gz.msgs10.entity_wrench_pb2 import EntityWrench  # noqa: E402
from gz.transport13 import Node as GzNode  # noqa: E402
gzn = GzNode()
wrench_pub = gzn.advertise('/world/quadcopter/wrench/persistent', EntityWrench)
clear_pub = gzn.advertise('/world/quadcopter/wrench/clear', Entity)


_applied_total = [0.0]


def set_torque(value):
    """Persistent wrenches ADD up, and a clear sent with the new one can be processed after
    it (wiping it): publish only the change."""
    delta = value - _applied_total[0]
    if delta == 0.0:
        return
    w = EntityWrench()
    w.entity.name = 'x3::X3/base_link'
    w.entity.type = Entity.LINK
    if args.mode == 'swing':
        w.entity.name = 'x3::magnet_tip_link'
        setattr(w.wrench.force, args.axis, delta)
    else:
        setattr(w.wrench.torque, args.axis, delta)
    wrench_pub.publish(w)
    _applied_total[0] = value


w0 = EntityWrench()
w0.entity.name = 'x3::X3/base_link'
w0.entity.type = Entity.LINK
for _ in range(3):                                      # zero wrench: forms the connection
    wrench_pub.publish(w0)
    time.sleep(0.5)

rclpy.init()
node = Hold()
t_end = time.time() + 20
while time.time() < t_end:                              # nodes up, bf has a pose
    rclpy.spin_once(node, timeout_sec=0.1)
gz('service', '-s', '/world/quadcopter/control', '--reqtype', 'gz.msgs.WorldControl',
   '--reptype', 'gz.msgs.Boolean', '--timeout', '3000', '--req', 'pause: false')

wall_limit = time.time() + 600
while time.time() < wall_limit and not node.done:
    rclpy.spin_once(node, timeout_sec=0.02)
t0 = node.rows[0][0]
rows = node.rows
node.destroy_node()
rclpy.shutdown()
stop_all()

with open(os.path.join(args.outdir, 'log.csv'), 'w', newline='') as fh:
    w = csv.writer(fh)
    w.writerow(['t', 'roll', 'pitch', 'yaw', 'x', 'y', 'z', 'ch0', 'ch2', 'tether_deg'])
    w.writerows(rows)
print(f'kp {args.kp} ki {args.ki} (U_hover {U_HOVER:.3f}, base_link index {idx}, '
      f'{len(rows)} samples)')
for a, b, tau in PHASES:
    seg = [r for r in rows if a + 0.0 <= r[0] - t0 < b]
    if not seg:
        continue
    tail = [r for r in seg if r[0] - t0 >= b - 2.0]
    tilt = [math.degrees(math.acos(max(-1, min(1, math.cos(math.radians(r[1]))
                                                 * math.cos(math.radians(r[2]))))))
            for r in seg]
    print(f'  {a:4.0f}-{b:4.0f} s torque {tau:.3f}: peak tilt {max(tilt):6.1f} deg, '
          f'roll last 2 s {sum(r[1] for r in tail) / max(1, len(tail)):+7.2f} deg, '
          f'z {min(r[6] for r in seg):.2f}-{max(r[6] for r in seg):.2f} m, '
          f'max |roll cmd| {max(abs(r[7]) for r in seg):.2f}')

if args.mode == 'swing':
    after = [r for r in rows if r[0] - t0 >= 6.2]
    peak = max(r[9] for r in after)
    settle = next((r[0] - t0 for i, r in enumerate(after)
                   if all(q[9] < 2.0 for q in after[i:])), None)
    tail = [r[9] for r in after if r[0] - t0 >= 17.0]
    print(f'  swing: tether peak {peak:.1f} deg after the kick, stays < 2 deg from '
          f'{"never" if settle is None else f"{settle:.1f} s"}, last 3 s mean {sum(tail)/len(tail):.2f} deg; '
          f'drone peak tilt after the kick '
          f'{max(math.degrees(math.acos(max(-1, min(1, math.cos(math.radians(r[1])) * math.cos(math.radians(r[2])))))) for r in after):.1f} deg')

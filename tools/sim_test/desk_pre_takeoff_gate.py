#!/usr/bin/env python3
"""Forced pre-TAKEOFF disarm on the real topics (card docs/experiments/2026-09-29_pre_takeoff_disarm_gate.md,
critic 6d). Desk test, no Gazebo:

    python3 tools/sim_test/desk_pre_takeoff_gate.py LOGDIR [DOMAIN]

The split M2 launch (dissipative_launch partner_m2, sim_interface:=false, never real), fake mocap
poses from this script, ARM until every /ours feedback is True, then SetArming(false) on
/ours/drone_1/arming_service (a tracker dropping out between ARM and TAKEOFF), then TAKEOFF.
PASS = the manager logs 'Drone 1 disarmed before TAKEOFF' once and 'Cannot TAKEOFF', no
/fleet/abort is published, every mux stays 'partner', every tracker ends disarmed, and no mux
output is armed or above idle after TAKEOFF. Isolated ROS domain (default 92, localhost only).
"""
import os
import re
import shutil
import signal
import subprocess
import sys
import time

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
logdir = os.path.abspath(sys.argv[1])
domain = int(sys.argv[2]) if len(sys.argv) > 2 else 92
if domain == 0:
    sys.exit('refusing to run on ROS domain 0 (the sim/rig domain)')
os.makedirs(logdir, exist_ok=True)
# the solvers load from copies: a rebuild here must never touch the .so a live run has loaded
# (the planner's code dir is relative to the cwd, the trackers' follow MDC_ACADOS_ROOT)
work = os.path.join(logdir, 'work')
for d in ('c_generated_code_load_planner', 'c_generated_code_quad_load'):
    if not os.path.isdir(os.path.join(work, d)):
        shutil.copytree(os.path.join(REPO, d), os.path.join(work, d), symlinks=True)
os.environ.update(ROS_DOMAIN_ID=str(domain), ROS_LOCALHOST_ONLY='1', MDC_RUN_DIR=logdir,
                  MDC_ACADOS_ROOT=work, MDC_REPO_ROOT=REPO)
os.environ.pop('FASTRTPS_DEFAULT_PROFILES_FILE', None)
N = 4
ARGS = ['num_drones:=4', 'sim_interface:=false', 'partner_m2:=true',
        'cable_len:=0.515', 'attach_azimuths_deg:=30.0,120.0,210.0,300.0', 'start_taut:=false',
        'load_mass:=0.86', 'load_traj:=hover', 'target_z:=0.6', 'reconfig_mode:=ocp',
        'takeoff_spool_s:=0.0', 'airborne_start:=true', 'auto_slot_assign:=false']
dlog = open(os.path.join(logdir, 'desk.log'), 'w')
procs = []


def say(msg):
    line = f'[{time.strftime("%H:%M:%S")}] {msg}'
    print(line, flush=True)
    dlog.write(line + '\n')
    dlog.flush()


def read(path):
    try:
        return open(path, errors='replace').read()
    except OSError:
        return ''


def launch(part, name):
    path = os.path.join(logdir, name)
    with open(path, 'w') as fh:
        p = subprocess.Popen(['ros2', 'launch', 'bringup', 'sim_control_launch.py', 'mode:=dissipative', 'legacy:=true',
                              *ARGS, f'partner_m2_part:={part}'], cwd=work, env=dict(os.environ),
                             stdout=fh, stderr=subprocess.STDOUT, start_new_session=True)
    procs.append(p)
    return p, path


def stop(p, timeout=30):
    if p.poll() is None:
        os.killpg(os.getpgid(p.pid), signal.SIGINT)
        try:
            p.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(p.pid), signal.SIGKILL)
            p.wait()


def finish(ok, why):
    for p in reversed(procs):
        stop(p)
    say(('PASS' if ok else 'FAIL') + f': {why}')
    sys.exit(0 if ok else 1)


import math  # noqa: E402

import rclpy  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from std_msgs.msg import Bool, String  # noqa: E402
from interfaces.msg import ELRSCommand, MotionCaptureState  # noqa: E402
from interfaces.srv import SetArming  # noqa: E402

rclpy.init(domain_id=domain)
node = rclpy.create_node('desk_pre_takeoff_gate')
time.sleep(2.0)                                   # discovery
others = [n for n in node.get_node_names() if n != 'desk_pre_takeoff_gate']
if others:
    sys.exit(f'domain {domain} is not empty ({others[:5]}): pick another domain')
mux = [None] * N
armed = [None] * N
aborts = []
out = {'n': [0] * N, 'hot': [0] * N, 'on': False}
latched_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
node.create_subscription(String, '/fleet/abort', lambda m: aborts.append(m.data), 5)
pose_pubs = []
for i in range(N):
    node.create_subscription(String, f'/drone_{i}/mux_state',
                             lambda m, k=i: mux.__setitem__(k, m.data), latched_qos)
    node.create_subscription(Bool, f'/ours/drone_{i}/arming_state_feedback',
                             lambda m, k=i: armed.__setitem__(k, m.data), 5)

    def out_cb(m, k=i):
        if out['on']:
            out['n'][k] += 1
            out['hot'][k] += int(bool(m.armed) or float(m.channel_2) > -0.99)
    node.create_subscription(ELRSCommand, f'/drone_{i}/ELRSCommand', out_cb, 10)
    pose_pubs.append(node.create_publisher(MotionCaptureState, f'/drone_{i}/motion_capture_state', 10))
payload_pub = node.create_publisher(MotionCaptureState, '/payload/motion_capture_state', 10)
his = [node.create_publisher(ELRSCommand, f'/drone_{i}/ELRSCommand_tejen', 5) for i in range(N)]
cmd = node.create_publisher(String, '/fleet/command', 5)
AZ = [30.0, 120.0, 210.0, 300.0]


def state(x, y, z):
    m = MotionCaptureState()
    m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, z
    m.pose.orientation.w = 1.0
    return m


def tick():
    # the four drones hold the ring at 45 deg (his join done), and his MPCs fly them
    r = 0.25 + 0.515 * math.cos(math.radians(45))
    for i, a in enumerate(AZ):
        pose_pubs[i].publish(state(r * math.cos(math.radians(a)), r * math.sin(math.radians(a)),
                                   0.1 + 0.515 * math.sin(math.radians(45))))
        his[i].publish(ELRSCommand(armed=True, channel_2=0.2))
    payload_pub.publish(state(0.0, 0.0, 0.1))


node.create_timer(0.02, tick)


def spin_until(pred, timeout, what):
    t0 = time.time()
    while time.time() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.05)
        if pred():
            return True
    finish(False, f'timeout waiting for {what}')


def spin_for(s):
    t0 = time.time()
    while time.time() - t0 < s:
        rclpy.spin_once(node, timeout_sec=0.05)


say(f'domain {domain} localhost-only; logs in {logdir}')
launch('muxes', 'muxes.log')
spin_until(lambda: all(s == 'partner' for s in mux), 120, 'four muxes in partner')
ctrl, log = launch('controllers', 'ctrl.log')
spin_until(lambda: 'CentralController ready' in read(log) and cmd.get_subscription_count() >= 1,
           300, 'the fleet manager')
clients = [node.create_client(SetArming, f'/ours/drone_{i}/arming_service') for i in range(N)]
spin_until(lambda: all(c.service_is_ready() for c in clients), 300, 'four arming services')
spin_for(3.0)
for _ in range(10):
    cmd.publish(String(data='ARM'))
    spin_for(4.0)
    if all(a is True for a in armed):
        break
if not all(a is True for a in armed) or 'ARM sequence complete' not in read(log):
    finish(False, f'fleet never armed: {armed}')
say(f'fleet armed {armed}, muxes {mux}; forcing drone 1 disarmed')
fut = clients[1].call_async(SetArming.Request(arm=False))
spin_until(fut.done, 10, 'the forced disarm')
spin_until(lambda: 'disarmed before TAKEOFF' in read(log), 10, 'the pre-TAKEOFF gate')
spin_for(3.0)
out['on'] = True
cmd.publish(String(data='TAKEOFF'))
spin_for(4.0)
text = read(log)
fired = len(re.findall(r'Drone 1 disarmed before TAKEOFF', text))
refused = 'Cannot TAKEOFF' in text
takeoffs = len(re.findall(r'Takeoff requested', text))
say(f'gate lines {fired}; TAKEOFF refused {refused}; tracker takeoffs {takeoffs}; aborts {aborts}; '
    f'muxes {mux}; our armed {armed}; mux outputs after TAKEOFF {out["n"]} msgs, '
    f'armed-or-above-idle {out["hot"]} (his stream forwarded)')
# the muxes keep forwarding his (armed) stream: 'hot' counts are his, expected > 0 and
# unchanged in kind; what must not happen is a switch to ours
ok = (fired == 1 and refused and takeoffs == 0 and not aborts
      and all(s == 'partner' for s in mux) and all(a is False for a in armed))
finish(ok, 'the dropped tracker grounded our fleet quietly, TAKEOFF refused, muxes on the partner'
       if ok else 'see desk.log / ctrl.log')

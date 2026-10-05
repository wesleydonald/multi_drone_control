#!/usr/bin/env python3
"""Arm D of the mux abort latch card (docs/experiments/2026-09-28_mux_abort_latch.md v3): a
re-ARM after a latch without relaunching the muxes must be refused. Desk test, no Gazebo:

    python3 tools/sim_test/desk_m2_rearm_latch.py LOGDIR [DOMAIN]

The split M2 launch (dissipative_launch partner_m2, sim_interface:=false, never real): the
muxes part, then the controllers part; ESTOP; the controllers part stopped and relaunched
with the muxes left running; ARM. PASS = every mux reports 'latched', the relaunched manager
logs 'ARM REFUSED: mux latched on drone i' and never 'Arming all drones', and no mux output
is armed or above idle after the abort. Runs on an isolated ROS domain (default 91,
localhost only); no clock, no serial port, no mocap UDP. Needs both setup files sourced.
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
domain = int(sys.argv[2]) if len(sys.argv) > 2 else 91
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
        p = subprocess.Popen(['ros2', 'launch', 'bringup', 'sim_control_launch.py', 'mode:=dissipative',
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


import rclpy  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from interfaces.msg import ELRSCommand  # noqa: E402

rclpy.init(domain_id=domain)
node = rclpy.create_node('desk_rearm_latch')
time.sleep(2.0)                                   # discovery
others = [n for n in node.get_node_names() if n != 'desk_rearm_latch']
if others:
    sys.exit(f'domain {domain} is not empty ({others[:5]}): pick another domain')
mux = [None] * N
out = {'n': [0] * N, 'hot': [0] * N, 'armed_phase': False}
latched_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
for i in range(N):
    node.create_subscription(String, f'/drone_{i}/mux_state',
                             lambda m, k=i: mux.__setitem__(k, m.data), latched_qos)

    def out_cb(m, k=i):
        if out['armed_phase']:
            out['n'][k] += 1
            out['hot'][k] += int(bool(m.armed) or float(m.channel_2) > -0.99)
    node.create_subscription(ELRSCommand, f'/drone_{i}/ELRSCommand', out_cb, 10)
cmd = node.create_publisher(String, '/fleet/command', 5)


def spin_until(pred, timeout, what):
    t0 = time.time()
    while time.time() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.2)
        if pred():
            return True
    finish(False, f'timeout waiting for {what}')


def manager_up(path):
    return ('CentralController ready' in read(path)
            and cmd.get_subscription_count() >= 1)


say(f'domain {domain} localhost-only; logs in {logdir}')
muxes, mux_log = launch('muxes', 'muxes.log')
spin_until(lambda: all(s == 'partner' for s in mux), 120, 'four muxes in partner')
mux_pids = sorted(int(x) for x in re.findall(r'elrs_mux-\d+\]: process started with pid \[(\d+)\]',
                                               read(mux_log)))
say(f'muxes up, states {mux}, pids {mux_pids}')

ctrl1, log1 = launch('controllers', 'ctrl1.log')
spin_until(lambda: manager_up(log1), 300, 'the first fleet manager')
spin_until(lambda: len(re.findall(r'Drone \d mux: partner', read(log1))) >= N, 30,
           'the manager to read the four mux states')
say('controllers part 1 up; ESTOP')
cmd.publish(String(data='ESTOP'))
spin_until(lambda: all(s == 'latched' for s in mux), 20, 'four latched muxes')
out['armed_phase'] = True
say(f'muxes latched: {mux}')
t0 = time.time()
while time.time() - t0 < 2.0:
    rclpy.spin_once(node, timeout_sec=0.1)
stop(ctrl1)
procs.remove(ctrl1)
if muxes.poll() is not None:
    finish(False, 'the muxes launch exited with the controllers part')
say('controllers part 1 stopped; muxes still running')

ctrl2, log2 = launch('controllers', 'ctrl2.log')
spin_until(lambda: manager_up(log2), 300, 'the relaunched fleet manager')
spin_until(lambda: len(re.findall(r'Drone \d mux: latched', read(log2))) >= N, 30,
           'the relaunched manager to read the latched states (TRANSIENT_LOCAL)')
say('controllers part 2 up; ARM')
cmd.publish(String(data='ARM'))
spin_until(lambda: 'ARM REFUSED' in read(log2), 20, 'ARM REFUSED on the gate log')
t0 = time.time()
while time.time() - t0 < 3.0:
    rclpy.spin_once(node, timeout_sec=0.1)
refused = sorted(set(int(d) - 1 for d in re.findall(r'ARM REFUSED: mux latched on drone (\d)', read(log2))))
armed_attempt = 'Arming all drones' in read(log2)
pids_now = sorted(int(x) for x in re.findall(r'elrs_mux-\d+\]: process started with pid \[(\d+)\]',
                                             read(mux_log)))
say(f'refused for drones {refused}; manager tried to arm: {armed_attempt}; '
    f'mux outputs after the abort {out["n"]} msgs, armed-or-above-idle {out["hot"]}; '
    f'mux pids {pids_now} (unchanged: {pids_now == mux_pids})')
ok = (refused == list(range(N)) and not armed_attempt and sum(out['hot']) == 0
      and min(out['n']) > 0 and pids_now == mux_pids and muxes.poll() is None)
finish(ok, 'ARM refused on every latched mux, outputs held at disarm' if ok else
       'see desk.log / ctrl2.log')

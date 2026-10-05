#!/usr/bin/env python3
"""R0a at home (readiness review 2026-09-29): Wednesday's R3a lines against fake mocap, no rig.

    python3 tools/sim_test/desk_r0a.py LOGDIR [DOMAIN]

Runs the sheet's exact R3a T1 (real_io_launch, rviz:=false, no radios plugged: the radio nodes
retry) and T2 (real_control_launch with RING 0.86, DM3 0.64, ROD 0.47, KT 24) on an isolated ROS
domain, publishing the rig's resting poses (tools/fake_mocap.compute_poses: 1/5/9, rods near flat,
drones at 0.08, ring at 0.05). Then: the sheet's T3 preflight (read-back = typed); ARM; drone 1's
pose stopped -> its 0.25 s watchdog disarms it -> the manager grounds the fleet and refuses TAKEOFF;
the between-flight reset (Ctrl-C T2, clean_slate --rig) keeps T1 alive; T2 relaunched; preflight
again. Never sends a flying TAKEOFF against a stationary feed except after the refusal."""
import os
import re
import signal
import subprocess
import sys
import time

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(REPO, 'tools'))
from fake_mocap import compute_poses  # noqa: E402

logdir = os.path.abspath(sys.argv[1])
domain = int(sys.argv[2]) if len(sys.argv) > 2 else 93
if domain == 0:
    sys.exit('refusing to run on ROS domain 0 (the rig domain)')
os.makedirs(logdir, exist_ok=True)
os.environ.update(ROS_DOMAIN_ID=str(domain), ROS_LOCALHOST_ONLY='1')
RING, DM3, ROD, KT = '0.86', '0.64', '0.47', '24.0'
T1 = ('real_io_launch.py num_drones:=3 drone0_serial:=/dev/QUAD1 drone1_serial:=/dev/QUAD2 '
      'drone2_serial:=/dev/QUAD3 magnet_channel:=6 magnet_initial:=ON rviz:=false').split()
T2 = (f'real_control_launch.py num_drones:=3 load_mass:={RING} drone_mass:={DM3} cable_len:={ROD} '
      'attach_radius:=0.25 attach_z:=0.0 attach_azimuths_deg:=30,150,270 '
      f'thrust_ratio:={KT} kt_trim:=false z_ki:=0.4 start_taut:=false handover_elev_deg:=45.0 '
      'handover_settle_s:=1.0 creep_vel:=0.2 target_z:=0.6 lift_ramp_vel:=0.0 load_traj:=hover').split()
TYPED = {'kt_trim': 'False', 'lift_ramp_vel': '0.0', 'start_taut': 'False', 'handover_elev_deg': '45.0',
         'creep_vel': '0.2', 'z_ki': '0.4', 'drone_mass': DM3, 'thrust_ratio': KT}
dlog = open(os.path.join(logdir, 'desk.log'), 'w')
procs = {}
results = {}


def say(msg):
    line = f'[{time.strftime("%H:%M:%S")}] {msg}'
    print(line, flush=True)
    dlog.write(line + '\n')
    dlog.flush()


def read(name):
    try:
        return open(os.path.join(logdir, name), errors='replace').read()
    except OSError:
        return ''


def launch(key, args, name):
    fh = open(os.path.join(logdir, name), 'w')
    p = subprocess.Popen(['ros2', 'launch', 'bringup', *args], cwd=REPO,
                         env=dict(os.environ), stdout=fh, stderr=subprocess.STDOUT, start_new_session=True)
    procs[key] = p
    return p


def stop(key, timeout=30):
    p = procs.pop(key, None)
    if p is None or p.poll() is not None:
        return
    os.killpg(os.getpgid(p.pid), signal.SIGINT)
    try:
        p.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        os.killpg(os.getpgid(p.pid), signal.SIGKILL)
        p.wait()


def finish(ok, why):
    for k in list(procs)[::-1]:
        stop(k)
    say(('PASS' if ok else 'FAIL') + f': {why}')
    for k, v in results.items():
        say(f'  {k}: {v}')
    sys.exit(0 if ok else 1)


import rclpy  # noqa: E402
from std_msgs.msg import Bool, String  # noqa: E402
from interfaces.msg import MotionCaptureState  # noqa: E402

rclpy.init(domain_id=domain)
node = rclpy.create_node('desk_r0a')
time.sleep(2.0)
others = [n for n in node.get_node_names() if n != 'desk_r0a']
if others:
    sys.exit(f'domain {domain} is not empty ({others[:5]}): pick another domain')
poses = compute_poses(3, cable_len=float(ROD), azimuths_deg='30,150,270', drone_z=0.08, load_z=0.05)
pubs = {t: node.create_publisher(MotionCaptureState, t, 10) for t in poses}
dropped = set()
armed = [None] * 3


def tick():
    for t, ((x, y, z), (qw, qx, qy, qz)) in poses.items():
        if t in dropped:
            continue
        m = MotionCaptureState()
        m.header.stamp = node.get_clock().now().to_msg()
        m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, z
        m.pose.orientation.w, m.pose.orientation.x = qw, qx
        m.pose.orientation.y, m.pose.orientation.z = qy, qz
        pubs[t].publish(m)


node.create_timer(0.02, tick)
for i in range(3):
    node.create_subscription(Bool, f'/drone_{i}/arming_state_feedback',
                             lambda msg, k=i: armed.__setitem__(k, msg.data), 5)
cmd = node.create_publisher(String, '/fleet/command', 5)


def spin_for(s):
    t0 = time.time()
    while time.time() - t0 < s:
        rclpy.spin_once(node, timeout_sec=0.05)


def spin_until(pred, timeout, what):
    t0 = time.time()
    while time.time() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.05)
        if pred():
            return True
    finish(False, f'timeout waiting for {what}')


def up(logname):
    text = read(logname)
    return 'CentralController ready' in text and len(re.findall(r'Controller ready', text)) >= 3


def preflight(tag):
    # keep publishing the fake poses while preflight measures them (a blocking call starves the feed)
    path = os.path.join(logdir, f'preflight_{tag}.txt')
    with open(path, 'w') as fh:
        p = subprocess.Popen([sys.executable, 'tools/preflight.py', '--drones', '3', '--real', '--planner',
                              'mpc_planner', '--max-ground-z', '0.5'], cwd=REPO, env=dict(os.environ),
                             stdout=fh, stderr=subprocess.STDOUT)
        t0 = time.time()
        while p.poll() is None and time.time() - t0 < 180:
            rclpy.spin_once(node, timeout_sec=0.02)
        if p.poll() is None:
            p.kill()
    text = read(f'preflight_{tag}.txt')
    missing = [f'{k}={v}' for k, v in TYPED.items() if not re.search(rf'{k}\W[^\n]*{re.escape(v)}', text)]
    fails = [ln.strip() for ln in text.splitlines() if 'FAIL' in ln]
    return missing, fails


say(f'domain {domain} localhost-only; logs in {logdir}')
launch('t1', T1, 't1.log')
spin_for(8.0)
launch('t2', T2, 't2.log')
spin_until(lambda: up('t2.log'), 420, 'T2 up (manager + three trackers ready)')
t1_pid = procs['t1'].pid
say('T1 and T2 up; preflight #1')
spin_for(10.0)
miss, fails = preflight('1')
results['preflight 1 read-back missing'] = miss or 'none'
results['preflight 1 FAIL lines'] = fails
say('ARM')
for _ in range(3):
    cmd.publish(String(data='ARM'))
    spin_for(3.0)
    if all(a is True for a in armed):
        break
if 'ARM sequence complete' not in read('t2.log') or not all(a is True for a in armed):
    finish(False, f'fleet never armed: {armed}')
say(f'armed {armed}; dropping drone 1 pose')
dropped.add('/drone_1/motion_capture_state')
spin_until(lambda: 'disarmed before TAKEOFF' in read('t2.log'), 15, 'the pre-TAKEOFF gate')
cmd.publish(String(data='TAKEOFF'))
spin_for(3.0)
t2 = read('t2.log')
results['gate line'] = re.findall(r'Drone \d disarmed before TAKEOFF[^\n]*', t2)[:1]
results['pose timeout line'] = re.findall(r'\[Drone 1\] Pose timeout[^\n]*', t2)[:1]
results['TAKEOFF refused'] = 'Cannot TAKEOFF' in t2
results['tracker takeoffs'] = len(re.findall(r'Takeoff requested', t2))
results['fleet abort'] = 'FLEET ABORT' in t2
dropped.clear()
say('between-flight reset: Ctrl-C T2, clean_slate --rig')
stop('t2')
cs = subprocess.run(['bash', 'tools/clean_slate.sh', '--rig'], cwd=REPO, env=dict(os.environ),
                    capture_output=True, text=True, timeout=120)
open(os.path.join(logdir, 'clean_slate_rig.txt'), 'w').write(cs.stdout + cs.stderr)
t1_alive = procs['t1'].poll() is None
spin_for(3.0)
nodes_now = node.get_node_names()
results['clean_slate --rig exit'] = cs.returncode
results['T1 alive after reset'] = t1_alive
results['T1 nodes seen'] = sorted(n for n in nodes_now if any(k in n for k in ('elrs', 'motion', 'mocap', 'fleet_viz')))[:8]
say('relaunch T2')
launch('t2', T2, 't2_second.log')
spin_until(lambda: up('t2_second.log'), 420, 'T2 up again')
spin_for(10.0)
miss2, fails2 = preflight('2')
results['preflight 2 read-back missing'] = miss2 or 'none'
results['preflight 2 FAIL lines'] = fails2
ok = (not miss and not miss2 and results['gate line'] and results['TAKEOFF refused']
      and results['tracker takeoffs'] == 0 and not results['fleet abort'] and t1_alive
      and cs.returncode == 0)
finish(ok, 'R0a: lines start, read-back = typed, the gate refuses, the reset keeps T1' if ok
       else 'see desk.log, t2.log, preflight_*.txt')

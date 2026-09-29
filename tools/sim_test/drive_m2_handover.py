#!/usr/bin/env python3
"""M2 handover demo (multi_drone_control, 2026-09-26): Tejen's M2D joins four drones to
the (dynamic, 0.86 kg) M2A ring, all at once, and holds them at 15 deg; our stack then
takes the fleet over (per-drone ELRS mux), creeps the rods to 45 deg and lifts.

    python3 tools/sim_test/drive_m2_handover.py LOGDIR [GUI true|false] [HOVER_S]

HOVER_S is SIM seconds at the target height before LAND. Needs both setup files sourced.
Writes LOGDIR/runner.log, ours*.log, driver.log, our nodes' logs under LOGDIR/logs, and
LOGDIR/metrics.json (tools/metrics.py summarise_m2) on exit.
M2_LIVE_KI=<ki>: also set rate_ki live on his four bridges at the success hold (they
start at M2_RATE_KI, else SIM_RATE_KI, else 5).
M2_RATE_SOURCE: his four bridges' rate loops run on the X3 gyro (imu, the default since
2026-09-28: the runner adds the Imu system to his world, his launch bridges /drone_i/imu) or
on differenced poses (pose).
M2_ESTOP_BEFORE_HANDOVER=1 (mux abort latch card 2026-09-28, arms A/B): after our fleet
arms, ESTOP on /fleet/command instead of the hand-over, record ESTOP_RECORD_S (10) sim
seconds with a bag of /drone_*/ELRSCommand* and the mux states under LOGDIR/bag, then stop.
M2_ABORT_LATCH=0: set abort_latch false on the four muxes once they are up (arm A).
"""
import json
import os
import re
import signal
import subprocess
import sys
import time

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(REPO, 'tools'))
from metrics import summarise_m2  # noqa: E402
logdir = os.path.abspath(sys.argv[1])
gui = (sys.argv[2] if len(sys.argv) > 2 else 'false').lower()
hover_s = float(sys.argv[3]) if len(sys.argv) > 3 else 10.0
os.makedirs(logdir, exist_ok=True)
runner_log = os.path.join(logdir, 'runner.log')
ours_log = os.path.join(logdir, 'ours.log')
# M2_RESUME=1: pick up an earlier driver's run at his success hold (the runner, his fleet and our
# muxes are still up): skip the runner, the muxes and his operator prompts
RESUME = os.environ.get('M2_RESUME') == '1'
dlog = open(os.path.join(logdir, 'driver.log'), 'a' if RESUME else 'w')


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


os.environ['FASTRTPS_DEFAULT_PROFILES_FILE'] = os.path.join(REPO, 'configs/dds/fastdds_udp_only.xml')
env = dict(os.environ, M2D_REPO=REPO, M2D_GOAL_ATTACHMENTS='4', M2D_SIMULTANEOUS='true',
           M2D_HANDOVER='true')
if os.environ.get('M2D_SOURCE_X3'):
    say_later = f"x3 model {os.environ['M2D_SOURCE_X3']}"
else:
    say_later = 'x3 model: his default'
# reaches the runner, and through it his launch, in env (validated there)
say(f"rate source {os.environ.get('M2_RATE_SOURCE') or 'imu'} (his four bridges)")
procs = []
if RESUME:
    class _Alive:                          # the earlier driver's runner, not ours to poll
        returncode = None

        def poll(self):
            return None
    runner = _Alive()
    say('RESUME at the success hold')
else:
    with open(runner_log, 'w') as fh:
        runner = subprocess.Popen(
            ['bash', os.path.join(REPO, 'tools/sim_test/run_m2d_sequential_attachment.sh'), gui],
            cwd=REPO, env=env, stdout=fh, stderr=subprocess.STDOUT, start_new_session=True)
    procs.append(runner)
    say(f'runner started (handover, simultaneous; {say_later})')


def stop_all(code):
    for p in reversed(procs):
        if p.poll() is None:
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGINT)
            except Exception:
                pass
    for p in procs:
        try:
            p.wait(timeout=40)
        except Exception:
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGKILL)
            except Exception:
                pass
    try:
        m = summarise_m2(logdir)
        with open(os.path.join(logdir, 'metrics.json'), 'w') as fh:
            json.dump(m, fh, indent=2, default=str)
        say(f"metrics: join {m.get('join_attached')}/4, ring tilt peak "
            f"{m.get('ring_tilt_peak_deg')} deg, load_z {m.get('load_z_min_m')}-"
            f"{m.get('load_z_max_m')} m, faults {m.get('envelope_faults')}, "
            f"landed {m.get('landed')}")
    except Exception as e:                  # noqa: BLE001
        say(f'metrics failed: {type(e).__name__}: {e}')
    say(f'driver exit {code}')
    sys.exit(code)


def wait_for(pred, timeout, what):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if runner.poll() is not None:
            say(f'runner exited ({runner.returncode}) while waiting for {what}')
            stop_all(1)
        if pred():
            return True
        time.sleep(1.0)
    say(f'TIMEOUT waiting for {what}')
    stop_all(1)


# 1. the frozen plate assignment -> our attach azimuths
wait_for(lambda: 'Evidence:' in read(runner_log), 120, 'evidence dir')
evidence = re.search(r'Evidence:\s+(\S+)', read(runner_log)).group(1)
rt = os.path.join(evidence, 'runtime.log')
wait_for(lambda: 'M2C assignment frozen' in read(rt), 900, 'M2C assignment')
m = re.search(r'M2C assignment frozen: (\{.*\})', read(rt))
plates = json.loads(m.group(1))['vehicle_to_plate']
az = [30.0 * int(plates[f'drone_{i}']) for i in range(4)]
say(f'assignment {plates} -> azimuths {az}')

# 2. our stack (muxes, trackers, fleet manager, OCP planner, ring mocap)
args = ['num_drones:=4', 'sim_interface:=false', 'partner_m2:=true', 'cable_len:=' + os.environ.get('M2_CABLE_LEN', '0.515'),
        'attach_azimuths_deg:=' + ','.join(f'{a:.1f}' for a in az), 'attach_z:=' + os.environ.get('M2_ATTACH_Z', '-0.005'),
        'start_taut:=false', 'handover_elev_deg:=' + os.environ.get('M2_ELEV_DEG', '45.0'),
        'cable_elev_deg:=' + os.environ.get('M2_ELEV_DEG', '45.0'), 'handover_settle_s:=1.0',
        'creep_vel:=0.2', 'load_mass:=0.86', 'load_traj:=hover', 'target_z:=0.6',
        'reconfig_mode:=ocp', 'pose_timeout_s:=3.0', 'safety_ref_timeout_s:=3.0',
        # his MPC is holding them in the air: no spool from half throttle at the switch
        'takeoff_spool_s:=0.0', 'airborne_start:=true',
        # the plates come from his frozen assignment, in drone order
        'auto_slot_assign:=false']
# our stack in two parts: the muxes now (his MPCs fly through them), the controllers only at
# his success hold (M2_LATE_CONTROLLERS, default on: they idle through his join otherwise)
late = os.environ.get('M2_LATE_CONTROLLERS', '1') != '0'
# our nodes log into this run's directory (summarise_m2 reads them there)
os.environ['MDC_RUN_DIR'] = logdir


def launch_ours(part, log_path):
    with open(log_path, 'w') as fh:
        full = args + [f'partner_m2_part:={part}']
        fh.write('$ ros2 launch controller_quad_load dissipative_launch.py ' + ' '.join(full) + '\n')
        fh.flush()
        p = subprocess.Popen(['ros2', 'launch', 'controller_quad_load', 'dissipative_launch.py',
                              *full], cwd=REPO, env=dict(os.environ), stdout=fh,
                             stderr=subprocess.STDOUT, start_new_session=True)
    procs.append(p)
    return p


if not RESUME:
    launch_ours('muxes' if late else 'all', ours_log)
    say('our muxes launched' + ('' if late else ' (with the controllers)'))


def muxes_up():
    for i in range(4):
        try:
            out = subprocess.run(['ros2', 'topic', 'info', f'/drone_{i}/ELRSCommand'],
                                 capture_output=True, text=True, timeout=20).stdout
        except Exception:
            return False
        c = re.search(r'Publisher count:\s*(\d+)', out)
        if not c or int(c.group(1)) < 1:
            return False
    return True


if not RESUME:
    wait_for(muxes_up, 300, 'the four muxes')
    say('muxes up')

ESTOP_ARM = os.environ.get('M2_ESTOP_BEFORE_HANDOVER') == '1'
if os.environ.get('M2_ABORT_LATCH') == '0':
    for i in range(4):
        r = subprocess.run(['ros2', 'param', 'set', f'/elrs_mux_{i}', 'abort_latch', 'false'],
                           capture_output=True, text=True, timeout=90)
        say(f'elrs_mux_{i} abort_latch false: {(r.stdout or r.stderr).strip()}')

# 3. play his operator: ARM/TAKEOFF prompts, re-sent until acknowledged
pat = re.compile(r"^ros2 topic pub --once -w 2 (/drone_\d/command) std_msgs/msg/String "
                 r"'\{data: (ARM|TAKEOFF)\}'$", re.M)
done = 0
t_start = time.time()
while not RESUME and 'SUCCESS HOLD' not in read(runner_log):
    if runner.poll() is not None or time.time() - t_start > 3600:
        say('runner ended / timed out before the success hold')
        stop_all(1)
    cmds = pat.findall(read(runner_log))
    while done < len(cmds):
        topic, what = cmds[done]
        drone = topic.split('/')[1]
        ack = (f'{drone} armed-state feedback: CONFIRMED' if what == 'ARM'
               else f'{drone} TAKEOFF acknowledgement: CONTROLLER TRUE')
        for attempt in range(5):
            time.sleep(3.0)
            say(f'his {what} -> {topic} (try {attempt + 1})')
            subprocess.run(['ros2', 'topic', 'pub', '--once', '-w', '2', topic,
                            'std_msgs/msg/String', f'{{data: {what}}}'], env=env,
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=60)
            t0 = time.time()
            while time.time() - t0 < 15.0 and ack not in read(runner_log):
                time.sleep(1.0)
            if ack in read(runner_log):
                break
        done += 1
    time.sleep(2.0)
say('M2D SUCCESS HOLD: all four attached and holding')

# optional override: rate_ki set live on his bridges for the rest of the run
live_ki = os.environ.get('M2_LIVE_KI')
if live_ki:
    for i in range(4):
        out = 'not set'
        for attempt in range(4):
            try:
                r = subprocess.run(['ros2', 'param', 'set', f'/drone_{i}/betaflight_communication',
                                    'rate_ki', live_ki], capture_output=True, text=True, timeout=90)
                out = r.stdout.strip() or r.stderr.strip()
                if 'successful' in out:
                    break
            except subprocess.TimeoutExpired:
                out = f'timed out (try {attempt + 1})'
        say(f'drone_{i} rate_ki {live_ki}: {out}')
    time.sleep(3.0)

if late:
    launch_ours('controllers', ours_log.replace('ours.log', 'ours_ctrl.log'))
    say('our controllers launched')

# 4. our takeover
import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import QoSProfile, ReliabilityPolicy  # noqa: E402
from rosgraph_msgs.msg import Clock  # noqa: E402
from std_msgs.msg import Bool, String  # noqa: E402
from interfaces.msg import MotionCaptureState  # noqa: E402

rclpy.init()
node = Node('m2_handover_driver')
state = {'z': None, 'landed': False, 'armed': [None] * 4, 'sim_t': None}
# read-only: the hover is counted in sim seconds (his world's real-time factor varies)
node.create_subscription(
    Clock, '/clock', lambda m: state.__setitem__('sim_t', m.clock.sec + m.clock.nanosec * 1e-9),
    QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
node.create_subscription(MotionCaptureState, '/payload/motion_capture_state',
                         lambda m: state.__setitem__('z', m.pose.position.z), 10)
node.create_subscription(Bool, '/fleet/landed',
                         lambda m: state.__setitem__('landed', state['landed'] or m.data), 5)
for i in range(4):
    node.create_subscription(Bool, f'/ours/drone_{i}/arming_state_feedback',
                             lambda m, k=i: state['armed'].__setitem__(k, m.data), 5)
cmd = node.create_publisher(String, '/fleet/command', 5)
handover = node.create_publisher(Bool, '/fleet/handover', 5)


def spin(seconds):
    t0 = time.time()
    while time.time() - t0 < seconds:
        rclpy.spin_once(node, timeout_sec=0.1)


# ARM only once every tracker serves its arming service: the fleet manager treats a failed ARM
# as an abort and then refuses TAKEOFF
from interfaces.srv import SetArming  # noqa: E402
clients = [node.create_client(SetArming, f'/ours/drone_{i}/arming_service') for i in range(4)]
t_wait = time.time()
while not all(c.service_is_ready() for c in clients) and time.time() - t_wait < 300:
    spin(1.0)
say(f'arming services ready: {[c.service_is_ready() for c in clients]} '
    f'({time.time() - t_wait:.0f} s)')
spin(5.0)                          # fleet manager + planner finish their start-up
z0 = state['z']
say(f'ring z at hand-over {z0}')
GROUNDED = ('disarmed before TAKEOFF', 'TAKEOFF REFUSED', 'ARM FAILED')


def manager_grounded():
    """The fleet manager grounded our fleet (it shuts the trackers down: re-ARM cannot help)."""
    try:
        with open(ours_log.replace('ours.log', 'ours_ctrl.log'), errors='replace') as fh:
            text = fh.read()
    except OSError:
        return None
    return next((g for g in GROUNDED if g in text), None)


for attempt in range(25):          # the late controllers need ~20-60 s wall to come up
    cmd.publish(String(data='ARM'))
    spin(6.0)
    if all(a is True for a in state['armed']):
        break
    if manager_grounded():
        say(f"fleet manager grounded our fleet ('{manager_grounded()}') - stopping")
        stop_all(1)
say(f'our fleet armed: {state["armed"]}')


def estop_before_handover():
    """Arms A/B: his MPCs still fly all four through the muxes; ESTOP, then count what the
    radios got. Falsifier (latch on): any mux output armed or above idle after the abort."""
    from interfaces.msg import ELRSCommand
    from rclpy.qos import DurabilityPolicy
    idle = -0.99
    ev = {'t_abort': None, 'out_hot': [0] * 4, 'out_n': [0] * 4, 'his_hot': [0] * 4,
          'mux': [None] * 4}
    latched_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)

    def out_cb(m, i):
        # from this mux's own latch on: a command it forwarded before it read the abort
        # can still arrive after the driver's /fleet/abort and is not a latch failure
        if ev['mux'][i] == 'latched':
            ev['out_n'][i] += 1
            ev['out_hot'][i] += int(bool(m.armed) or float(m.channel_2) > idle)

    def his_cb(m, i):
        if ev['t_abort'] is not None:
            ev['his_hot'][i] += int(bool(m.armed) and float(m.channel_2) > idle)

    for i in range(4):
        node.create_subscription(ELRSCommand, f'/drone_{i}/ELRSCommand',
                                 lambda m, k=i: out_cb(m, k), 10)
        node.create_subscription(ELRSCommand, f'/drone_{i}/ELRSCommand_tejen',
                                 lambda m, k=i: his_cb(m, k), 10)
        node.create_subscription(String, f'/drone_{i}/mux_state',
                                 lambda m, k=i: ev['mux'].__setitem__(k, m.data), latched_qos)
    node.create_subscription(String, '/fleet/abort',
                             lambda m: ev.__setitem__('t_abort', ev['t_abort'] or state['sim_t']), 5)
    topics = [t for i in range(4) for t in (f'/drone_{i}/ELRSCommand', f'/drone_{i}/ELRSCommand_tejen',
                                           f'/drone_{i}/ELRSCommand_diss', f'/drone_{i}/mux_state',
                                           f'/drone_{i}/motion_capture_state')]
    topics += ['/fleet/abort', '/fleet/command', '/payload/motion_capture_state']
    with open(os.path.join(logdir, 'bag.log'), 'w') as fh:
        procs.append(subprocess.Popen(['ros2', 'bag', 'record', '-o', os.path.join(logdir, 'bag'),
                                       *topics], cwd=REPO, env=dict(os.environ), stdout=fh,
                                      stderr=subprocess.STDOUT, start_new_session=True))
    spin(5.0)                                  # the recorder discovers the topics
    say(f'mux states before the ESTOP: {ev["mux"]}')
    cmd.publish(String(data='ESTOP'))
    say('ESTOP sent (before the hand-over)')
    rec_s = float(os.environ.get('ESTOP_RECORD_S', '10'))
    t0 = time.time()
    while time.time() - t0 < 60.0 + 40.0 * rec_s:
        spin(0.5)
        if (ev['t_abort'] is not None and state['sim_t'] is not None
                and state['sim_t'] - ev['t_abort'] >= rec_s):
            break
    say(f"abort at sim {ev['t_abort']}; mux states {ev['mux']}; our armed {state['armed']}")
    say(f"mux output after the abort: {ev['out_n']} msgs, armed-or-above-idle {ev['out_hot']} "
        f"(latch on: all 0); his streams armed above idle {ev['his_hot']} (must be > 0)")
    stop_all(0)


if ESTOP_ARM:
    estop_before_handover()
spin(1.0)
if not all(a is True for a in state['armed']):
    # the manager grounds our fleet when a tracker drops between ARM and TAKEOFF (Q11): do not
    # hand over (a mux would switch to a dead tracker) and do not wait for a lift
    say(f"our fleet no longer armed before the hand-over: {state['armed']} - stopping")
    stop_all(1)
for _ in range(5):
    handover.publish(Bool(data=True))
    spin(0.3)
cmd.publish(String(data='TAKEOFF'))
say('HANDOVER + TAKEOFF sent')
t0 = time.time()
while time.time() - t0 < 1200:
    spin(1.0)
    if state['z'] is not None and z0 is not None and state['z'] > z0 + 0.3:
        break
    if not all(a is True for a in state['armed']):
        say(f"our fleet disarmed before the lift: {state['armed']} - stopping")
        stop_all(1)
say(f'ring lifted to {state["z"]} ({time.time() - t0:.0f} s wall)')
t_sim0, t0 = state['sim_t'], time.time()
while time.time() - t0 < 60.0 + 40.0 * hover_s:     # wall guard against a stalled /clock
    spin(0.5)
    if t_sim0 is None:
        t_sim0 = state['sim_t']
    elif state['sim_t'] is not None and state['sim_t'] - t_sim0 >= hover_s:
        break
hovered = (f"{state['sim_t'] - t_sim0:.1f} s sim"
           if t_sim0 is not None and state['sim_t'] is not None else 'no /clock')
say(f'hover done ({hovered}, {time.time() - t0:.0f} s wall), ring z {state["z"]}; LAND')
cmd.publish(String(data='LAND'))
t0 = time.time()
while time.time() - t0 < 900 and not state['landed']:
    spin(1.0)
say(f'landed={state["landed"]}')
spin(10.0)
node.destroy_node()
rclpy.shutdown()
stop_all(0)

#!/usr/bin/env python3
"""tools/frozen_pose_desk_check.py -- the mocap node's frozen-pose guard, end to end at the desk (rig stack DOWN).

    source ~/ros2_humble/install/setup.bash && source install/setup.bash
    python3 tools/frozen_pose_desk_check.py            # the guard at 0.5 s, then off

Starts the real mocap node (UDP 1511) on a private ROS domain with made-up body ids (91-94 the quads, 98 the ring, so
packets from the mocap PC are ignored), sends it packets at 240 Hz and reads what it publishes:
  3.0-5.0 s  QUAD4 and the ring send one unchanged pose (lost bodies); 5.0 s QUAD4 comes back 1 m away
  7.0-8.2 s  QUAD1 sends one unchanged pose
With the guard: QUAD4's and QUAD1's poses stop 0.5 s into their freeze and resume when the pose changes, with no speed
read from the jump; QUAD2, QUAD3 and the ring (never withheld: card 2026-11-11_frozen_pose_guard, critic F1/F2) keep
arriving. With the guard off nothing stops. What a quad's readers do with the gap (the 0.25 s pose rule) is theirs.
Exit 0 when every check holds.
"""
import os
import random
import signal
import socket
import subprocess
import sys
import threading
import time

WINDOW = 0.5
QUADS, RING = (91, 92, 93, 94), 98
FREEZES = {94: (3.0, 5.0), 98: (3.0, 5.0), 91: (7.0, 8.2)}
RUN_S = 10.0


def send(t0, stop):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rng = random.Random(1)
    held = {}
    while not stop.is_set():
        t = time.time() - t0
        for k, body in enumerate(QUADS + (RING,)):
            x = 0.5 * k + (1.0 if body == 94 and t >= 5.0 else 0.0)
            text = f'{body}|({x + rng.gauss(0, 2e-5):.6f}, {0.2 + rng.gauss(0, 2e-5):.6f}, {1.0 + rng.gauss(0, 2e-5):.6f})|(0.0, 0.0, 0.0, 1.0)'
            a, b = FREEZES.get(body, (1e9, 1e9))
            if a <= t < b:
                text = held.setdefault(body, text)
            else:
                held.pop(body, None)
            s.sendto(text.encode(), ('127.0.0.1', 1511))
        time.sleep(1.0 / 240.0)


def run(window):
    import rclpy
    from interfaces.msg import MotionCaptureState
    env = dict(os.environ, ROS_DOMAIN_ID='47', ROS_LOCALHOST_ONLY='1', MDC_MOCAP_FROZEN_S=str(window))
    os.environ.update(ROS_DOMAIN_ID='47', ROS_LOCALHOST_ONLY='1')
    node_proc = subprocess.Popen(
        ['ros2', 'run', 'drone_communication', 'motion_capture_publisher_node', '--ros-args',
         '-p', f'drone_body_ids:=[{",".join(map(str, QUADS))}]', '-p', f'payload_body_id:={RING}',
         '-p', 'magnet_tip_body_id:=-1'],
        env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, start_new_session=True)
    lines = []
    threading.Thread(target=lambda: lines.extend(node_proc.stdout), daemon=True).start()
    rclpy.init()
    n = rclpy.create_node('frozen_pose_desk_check')
    got = {}
    t0 = [None]

    def cb(msg, key):
        if t0[0] is not None:
            v = msg.twist.linear
            got.setdefault(key, []).append((time.time() - t0[0], (v.x * v.x + v.y * v.y + v.z * v.z) ** 0.5))
    for d in range(4):
        n.create_subscription(MotionCaptureState, f'/drone_{d}/motion_capture_state', lambda m, d=d: cb(m, d), 50)
    n.create_subscription(MotionCaptureState, '/payload/motion_capture_state', lambda m: cb(m, 'ring'), 50)
    time.sleep(2.0)                                   # the node up and bound
    stop = threading.Event()
    t0[0] = time.time()
    threading.Thread(target=send, args=(t0[0], stop), daemon=True).start()
    while time.time() - t0[0] < RUN_S:
        rclpy.spin_once(n, timeout_sec=0.01)
    stop.set()
    n.destroy_node()
    rclpy.shutdown()
    os.killpg(node_proc.pid, signal.SIGINT)
    try:
        node_proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        os.killpg(node_proc.pid, signal.SIGKILL)
    time.sleep(0.3)
    return got, lines


def gaps(rows, least=0.08):
    return [(round(a[0], 2), round(b[0], 2)) for a, b in zip(rows, rows[1:]) if b[0] - a[0] > least]


def main():
    ok = True

    def check(what, cond, detail=''):
        nonlocal ok
        ok &= bool(cond)
        print(f"  {'ok  ' if cond else 'FAIL'} {what}" + (f'   [{detail}]' if detail else ''))

    print(f'guard at {WINDOW} s')
    got, lines = run(WINDOW)
    if any(len(got.get(k, [])) < 200 for k in (0, 1, 2, 3, 'ring')):
        print('  FAIL the node published too little: is UDP 1511 free (rig stack down)?', {k: len(v) for k, v in got.items()})
        print(''.join(lines[-15:]))
        return 1
    for d, (a, b) in ((3, FREEZES[94]), (0, FREEZES[91])):
        g = gaps(got[d])
        check(f'QUAD{d + 1}: one gap, from {WINDOW} s into its freeze to its end', len(g) == 1
              and abs(g[0][0] - (a + WINDOW)) < 0.06 and abs(g[0][1] - b) < 0.06, f'gaps {g}')
        after = [v for t, v in got[d] if b <= t < b + 0.5]
        check(f'QUAD{d + 1}: no speed read from the jump on resume', after and max(after) < 0.1,
              f'first {after[0]:.3f} m/s, peak {max(after):.3f} m/s in 0.5 s' if after else 'no poses')
    for d in (1, 2):
        check(f'QUAD{d + 1}: never interrupted', not gaps(got[d]), f'gaps {gaps(got[d])}')
    check('the ring: never interrupted, though its pose stood still for 2 s', not gaps(got['ring']), f"gaps {gaps(got['ring'])}")
    log = ''.join(lines)
    check('the node said so, once each way per quad', log.count('MOCAP FROZEN: QUAD4') == 1 and log.count('MOCAP FROZEN: QUAD1') == 1
          and log.count('mocap live again: QUAD4') == 1 and log.count('mocap live again: QUAD1') == 1 and 'QUAD2' not in log
          and 'QUAD3' not in log)
    for ln in lines:
        if 'MOCAP FROZEN' in ln or 'live again' in ln or 'frozen-pose guard' in ln:
            print('    ' + ln.strip()[:210])

    print('guard off (MDC_MOCAP_FROZEN_S=0)')
    got, lines = run(0)
    for d in range(4):
        check(f'QUAD{d + 1}: never interrupted', len(got.get(d, [])) > 200 and not gaps(got[d]), f'gaps {gaps(got.get(d, []))}')
    check('no guard lines', 'MOCAP FROZEN' not in ''.join(lines) and 'frozen-pose guard' not in ''.join(lines))
    print('OK' if ok else 'NOT OK')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())

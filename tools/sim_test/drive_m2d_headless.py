#!/usr/bin/env python3
"""Run Tejen's M2D sequential four-drone runner headless and play the operator: every
ARM / TAKEOFF CLI fallback it prints is executed once, in order (multi_drone_control).

    python3 tools/sim_test/drive_m2d_headless.py LOGFILE [GOAL_ATTACHMENTS] [SIMULTANEOUS true|false] [GUI true|false]
"""
import os, re, subprocess, sys, time

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
log_path = sys.argv[1]
goal = sys.argv[2] if len(sys.argv) > 2 else '4'
simultaneous = (sys.argv[3] if len(sys.argv) > 3 else 'false').lower()
gui = (sys.argv[4] if len(sys.argv) > 4 else 'false').lower()   # 'true' opens Gazebo + RViz
env = dict(os.environ, M2D_REPO=REPO, M2D_GOAL_ATTACHMENTS=goal, M2D_SIMULTANEOUS=simultaneous)
with open(log_path, 'w') as fh:
    runner = subprocess.Popen(['bash', os.path.join(REPO, 'tools/sim_test/run_m2d_sequential_attachment.sh'), gui],
                              cwd=REPO, env=env, stdout=fh, stderr=subprocess.STDOUT, start_new_session=True)
pat = re.compile(r"^ros2 topic pub --once -w 2 (/drone_\d/command) std_msgs/msg/String '\{data: (ARM|TAKEOFF)\}'$", re.M)
done = 0
while runner.poll() is None:
    time.sleep(2.0)
    try:
        text = open(log_path, errors='replace').read()
    except OSError:
        continue
    cmds = pat.findall(text)
    while done < len(cmds):
        topic, what = cmds[done]
        drone = topic.split('/')[1]
        # the runner's acknowledgement for this command; re-send until it appears (a single
        # publish was lost for drone_1's TAKEOFF in the first sequential run)
        ack = (f'{drone} armed-state feedback: CONFIRMED' if what == 'ARM'
               else f'{drone} TAKEOFF acknowledgement: CONTROLLER TRUE')
        for attempt in range(5):
            time.sleep(3.0)
            print(f'[driver] {what} -> {topic} (try {attempt + 1})', flush=True)
            subprocess.run(['ros2', 'topic', 'pub', '--once', '-w', '2', topic,
                            'std_msgs/msg/String', f'{{data: {what}}}'], env=env,
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=60)
            t0 = time.time()
            while time.time() - t0 < 15.0 and runner.poll() is None:
                if ack in open(log_path, errors='replace').read():
                    break
                time.sleep(1.0)
            if ack in open(log_path, errors='replace').read() or runner.poll() is not None:
                break
        done += 1
print(f'[driver] runner exited {runner.returncode}; commands sent {done}', flush=True)
sys.exit(runner.returncode)

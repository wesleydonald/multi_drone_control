#!/usr/bin/env python3
"""tools/his_console.py -- one-terminal keys for his standalone M1 IRL stack (m1_irl_pickup.launch.py), Wesley 21 Oct.

    a = ARM   t = TAKEOFF   l = LAND (/join_planner/land_now)   SPACE = DISARM (instant)   q = quit

ARM, TAKEOFF and DISARM go to /drone_command (his controller and planner); the line shows his planner's phase.
"""
import select
import sys
import termios
import time
import tty

import rclpy
from std_msgs.msg import Bool, String

KEYS = {'a': ('ARM', None), 't': ('TAKEOFF', None), 'l': (None, True), ' ': ('DISARM', None)}


def main():
    rclpy.init()
    node = rclpy.create_node('his_console')
    cmd = node.create_publisher(String, '/drone_command', 10)
    land = node.create_publisher(Bool, '/join_planner/land_now', 10)
    phase = {'v': '?'}
    node.create_subscription(String, '/join_planner/phase', lambda m: phase.__setitem__('v', m.data), 10)
    old = termios.tcgetattr(sys.stdin)
    last, shown = '', ''
    try:
        tty.setcbreak(sys.stdin.fileno())
        print(__doc__.strip().splitlines()[2].strip() + '\n')
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            if select.select([sys.stdin], [], [], 0)[0]:
                k = sys.stdin.read(1).lower()
                if k == 'q':
                    break
                if k in KEYS:
                    word, land_now = KEYS[k]
                    for _ in range(3 if word == 'DISARM' else 1):   # a kill is never lost to one dropped message
                        if word:
                            cmd.publish(String(data=word))
                        else:
                            land.publish(Bool(data=True))
                    last = f'sent {word or "LAND"} at {time.strftime("%H:%M:%S")}'
            line = f'\r phase {phase["v"]:<26s} {last:<30s}'
            if line != shown:
                sys.stdout.write(line)
                sys.stdout.flush()
                shown = line
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        print()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

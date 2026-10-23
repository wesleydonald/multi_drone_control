#!/usr/bin/env python3
"""
tools/fleet_reach.py -- is this terminal in the fleet's DDS domain? (critic F2, combined M1 on the rig)

A terminal with another ROS_DOMAIN_ID or without ROS_LOCALHOST_ONLY publishes into a graph nobody listens to: a
`fleet disarm` from it goes nowhere, silently. The fleet manager (and the planner) keep /fleet/manager_status
latched, so a terminal that reaches the fleet gets it at once.

    tools/fleet_reach.py                  # exit 0 if /fleet/manager_status answers within 2 s, else 1
    tools/fleet_reach.py --wait 120       # keep asking up to 120 s (rig_flight.sh, once its launch is starting)
    tools/fleet_reach.py --absent         # exit 0 if NOTHING answers (rig_flight.sh, before its launch)
    tools/fleet_reach.py --send DISARM    # the fleet CLI: refuse unless it answers, then publish it 3 times

`--send` is what the shell's `fleet` helper calls (its replacement is in the rig sheet; shortcuts.md is Wesley's).
"""
import argparse
import os
import sys
import time

COMMANDS = ('ARM', 'TAKEOFF', 'LAND', 'DISARM', 'ESTOP')
TOPIC = '/fleet/manager_status'


def env_line():
    return (f"ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', '(unset)')} "
            f"ROS_LOCALHOST_ONLY={os.environ.get('ROS_LOCALHOST_ONLY', '(unset)')}")


def probe(node, timeout_s=2.0):
    """The latched /fleet/manager_status text if it arrives within timeout_s, else None."""
    import rclpy
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String
    got = []
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    sub = node.create_subscription(String, TOPIC, lambda m: got.append(m.data), qos)
    t_end = time.monotonic() + timeout_s
    while not got and time.monotonic() < t_end:
        rclpy.spin_once(node, timeout_sec=0.05)
    node.destroy_subscription(sub)
    return got[0] if got else None


def send(node, command, repeats=3):
    """Publish the fleet command `repeats` times once the fleet manager subscribes (as `ros2 topic pub -t 3`)."""
    import rclpy
    from std_msgs.msg import String
    pub = node.create_publisher(String, '/fleet/command', 10)
    t_end = time.monotonic() + 2.0
    while pub.get_subscription_count() == 0 and time.monotonic() < t_end:
        rclpy.spin_once(node, timeout_sec=0.05)
    for _ in range(repeats):
        pub.publish(String(data=command))
        rclpy.spin_once(node, timeout_sec=0.1)
    return pub.get_subscription_count()


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    ap.add_argument('--timeout', type=float, default=2.0, help='s per attempt (default 2)')
    ap.add_argument('--wait', type=float, default=0.0, help='keep asking up to this long (s)')
    ap.add_argument('--absent', action='store_true', help='succeed only if nothing answers')
    ap.add_argument('--send', default='', help='fleet command to publish once the fleet answers')
    a = ap.parse_args(argv)
    cmd = a.send.strip().upper()
    if a.send and cmd not in COMMANDS:
        print(f"fleet: unknown command '{a.send}' ({' '.join(COMMANDS)})", file=sys.stderr)
        return 2
    import rclpy
    rclpy.init()
    node = rclpy.create_node('fleet_reach')
    try:
        t_end = time.monotonic() + a.wait
        status = probe(node, a.timeout)
        while status is None and not a.absent and time.monotonic() < t_end:
            status = probe(node, a.timeout)
        if a.absent:
            if status is not None:
                print(f'fleet_reach: a fleet manager already answers in this domain ({env_line()}): "{status}". '
                      f'Another stack is up: stop it first.', file=sys.stderr)
                return 1
            return 0
        if status is None:
            print(f'fleet_reach: REFUSED: no fleet manager answered on {TOPIC} within {a.timeout:g} s from this '
                  f'terminal ({env_line()}). Its commands would go nowhere: export the T2 domain '
                  f'(ROS_LOCALHOST_ONLY=1, ROS_DOMAIN_ID as T2) or check that T2 is up.', file=sys.stderr)
            return 1
        if cmd:
            n = send(node, cmd)
            print(f'fleet: {cmd} sent ({n} subscriber(s); fleet manager says "{status}")')
        else:
            print(f'fleet_reach: ok ({env_line()}): "{status}"')
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())

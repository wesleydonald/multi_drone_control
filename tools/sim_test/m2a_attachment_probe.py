#!/usr/bin/env python3
"""Test-only staged M2A contact/proof probe.

The actual thesis X3 model is left unchanged. This probe moves only the invisible
``m2a_x3_support`` scaffold to which the real X3 base is fixed for M2A bench
commissioning. It never publishes planner or MPC reference commands.
"""

from __future__ import annotations

import argparse
import math
import subprocess
import time


def run(cmd):
    result = subprocess.run(cmd, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False)
    if result.returncode != 0:
        raise RuntimeError(f"command failed: {' '.join(cmd)}\n{result.stderr}")
    return result.stdout


def set_support_pose(x, y, z, yaw=0.0):
    half = 0.5 * yaw
    request = (
        f'name: "m2a_x3_support" position {{ x: {x:.6f} y: {y:.6f} z: {z:.6f} }} '
        f'orientation {{ z: {math.sin(half):.9f} w: {math.cos(half):.9f} }}'
    )
    run([
        "gz", "service", "-s", "/world/quadcopter/set_pose",
        "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean",
        "--timeout", "1000", "--req", request,
    ])


def ros_string(topic, value):
    run(["ros2", "topic", "pub", "--once", topic, "std_msgs/msg/String", f"{{data: {value}}}"])


def ros_bool(topic, value):
    run(["ros2", "topic", "pub", "--once", topic, "std_msgs/msg/Bool", f"{{data: {str(value).lower()}}}"])


def phase(name):
    print(f"M2A probe phase: {name}", flush=True)
    ros_string("/m2a/probe/phase", name)


def move_linear(start, goal, duration=2.0, rate_hz=25.0):
    steps = max(2, int(duration * rate_hz))
    for k in range(steps + 1):
        a = k / steps
        pose = [(1.0 - a) * s + a * g for s, g in zip(start, goal)]
        set_support_pose(*pose)
        time.sleep(duration / steps)


def hold_pose(pose, duration, rate_hz=25.0):
    steps = max(1, int(duration * rate_hz))
    for _ in range(steps):
        set_support_pose(*pose)
        time.sleep(duration / steps)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("scenario", choices=["success", "miss", "magnet_off", "forced_separation"], nargs="?", default="success")
    parser.add_argument("--start-x", type=float, default=0.55)
    parser.add_argument("--start-y", type=float, default=0.0)
    # Current real M2 X3 geometry: base -> tether anchor 0.05 m downward,
    # tether 0.50 m, magnet contact offset 0.025 m downward. With ring plane
    # z=0.10, base contact z=0.675. Staging z=0.855 gives 0.18 m clearance.
    parser.add_argument("--start-z", type=float, default=0.855)
    parser.add_argument("--plate-x", type=float, default=0.25)
    parser.add_argument("--plate-y", type=float, default=0.0)
    parser.add_argument("--precontact-z", type=float, default=0.715)
    parser.add_argument("--contact-z", type=float, default=0.675)
    args = parser.parse_args()

    start = (args.start_x, args.start_y, args.start_z)
    target_y = args.plate_y + (0.07 if args.scenario == "miss" else 0.0)
    above = (args.plate_x, target_y, args.start_z)
    precontact = (args.plate_x, target_y, args.precontact_z)
    contact = (args.plate_x, target_y, args.contact_z)

    phase("RESET")
    ros_bool("/m2a/attachment/proof_requested", False)
    ros_string("/magnet/command", "OFF")
    hold_pose(start, 2.5)

    phase("HORIZONTAL_APPROACH")
    move_linear(start, above, duration=3.0)
    phase("ABOVE_PLATE_SETTLE")
    hold_pose(above, 2.0)

    phase("PRECONTACT_DESCENT")
    move_linear(above, precontact, duration=2.5)
    hold_pose(precontact, 1.0)

    if args.scenario != "magnet_off":
        phase("MAGNET_ON")
        ros_string("/magnet/command", "ON")
        hold_pose(precontact, 0.8)
    else:
        phase("MAGNET_REMAINS_OFF")

    phase("CONTACT_DESCENT")
    move_linear(precontact, contact, duration=2.0)
    phase("CONTACT_SETTLE")
    hold_pose(contact, 1.0)

    if args.scenario == "magnet_off":
        phase("DONE_MAGNET_OFF")
        return

    phase("PROOF")
    ros_bool("/m2a/attachment/proof_requested", True)
    proof_goal = (contact[0] + 0.006, contact[1], contact[2] + 0.024)
    move_linear(contact, proof_goal, duration=1.5)
    phase("PROOF_HOLD")
    hold_pose(proof_goal, 2.0)

    if args.scenario == "forced_separation":
        phase("FORCED_SEPARATION")
        ros_string("/magnet/command", "OFF")
        hold_pose(proof_goal, 1.5)

    ros_bool("/m2a/attachment/proof_requested", False)
    phase("DONE")


if __name__ == "__main__":
    main()

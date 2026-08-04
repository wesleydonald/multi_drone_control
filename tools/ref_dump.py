#!/usr/bin/env python3
"""
ref_dump.py - capture what the planner actually sends the tracker.

Every offline reproduction so far used a GUESSED reference (hover, 1g vertical)
and produced a clean zero command, while the real system commands a large
standing rate from the same apparent state. So the guess is wrong somewhere.
This records the real /drone_{id}/reference_trajectory plus the drone's measured
state, so the solver can be replayed offline on the true inputs.

    python3 ref_dump.py --drone 0 --secs 20 --out /tmp/ref0.npz
"""
import argparse
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from interfaces.msg import MotionCaptureState


class RefDump(Node):
    def __init__(self, drone):
        super().__init__("ref_dump")
        self.refs = []
        self.states = []
        self.create_subscription(
            Float64MultiArray, f"/drone_{drone}/reference_trajectory",
            self._ref, 10)
        self.create_subscription(
            MotionCaptureState, f"/drone_{drone}/motion_capture_state",
            self._state, 10)

    def _ref(self, m):
        self.refs.append((self.get_clock().now().nanoseconds * 1e-9,
                          np.array(m.data, dtype=float)))

    def _state(self, m):
        p, o = m.pose.position, m.pose.orientation
        lv, av = m.twist.linear, m.twist.angular
        self.states.append((self.get_clock().now().nanoseconds * 1e-9,
                            np.array([p.x, p.y, p.z, o.w, o.x, o.y, o.z,
                                      lv.x, lv.y, lv.z, av.x, av.y, av.z])))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--drone", type=int, default=0)
    ap.add_argument("--secs", type=float, default=20.0)
    ap.add_argument("--out", default="/tmp/ref0.npz")
    a = ap.parse_args()

    rclpy.init()
    n = RefDump(a.drone)
    end = None
    while rclpy.ok():
        rclpy.spin_once(n, timeout_sec=0.05)
        t = n.get_clock().now().nanoseconds * 1e-9
        if end is None:
            end = t + a.secs
        if t > end:
            break

    if not n.refs:
        print("NO REFERENCE MESSAGES - is the planner running?")
    else:
        rt = np.array([r[0] for r in n.refs])
        rd = np.array([r[1] for r in n.refs], dtype=object)
        L = min(len(x) for x in rd)
        np.savez(a.out,
                 ref_t=rt, ref=np.stack([x[:L] for x in rd]),
                 st_t=np.array([s[0] for s in n.states]) if n.states else np.zeros(0),
                 st=np.stack([s[1] for s in n.states]) if n.states else np.zeros((0, 13)))
        print(f"saved {len(n.refs)} refs ({L} floats each) and "
              f"{len(n.states)} states -> {a.out}")
        r0 = n.refs[len(n.refs) // 2][1]
        nn, dt = int(r0[0]), r0[1]
        print(f"\nmid-run message: n_nodes={nn} dt={dt}")
        print("node  pos                    vel                    "
              "acc                    cable")
        for j in range(min(3, nn)):
            b = 2 + 12 * j
            f = lambda s: "(" + ",".join(f"{v:+.3f}" for v in r0[s:s + 3]) + ")"
            print(f"  {j}  {f(b):22s} {f(b+3):22s} {f(b+6):22s} {f(b+9)}")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
rate_probe.py - is the rate loop actually inverted?

Every previous answer to this came from differentiating the logged pose, which
updates at ~17 Hz and is rounded to 3 dp, so it can't be trusted. This records
the IMU GYRO directly against the commanded rate - no differentiation, no
rounding - and reports the correlation per axis.

Two modes:

  passive (default)   just listen during a normal flight and correlate
      python3 rate_probe.py --drone 0 --secs 30

  openloop            take over one drone and command a known constant rate,
                      then report which way it actually rotated. This is the
                      decisive test: no MPC, no planner, no cables in the loop.
      python3 rate_probe.py --drone 0 --openloop pitch --value 0.3 \
              --throttle 0.25 --secs 4

Run the openloop test in a world with no cables (three_no_cables.sdf) and with
the controller stack NOT running, or it will fight you for the command topic.
"""
import argparse
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu
from interfaces.msg import ELRSCommand

AXES = {"roll": 0, "pitch": 1, "yaw": 2}
# ELRSCommand channel carrying each axis' rate command
CHAN = {"roll": "channel_0", "pitch": "channel_1", "yaw": "channel_3"}


class RateProbe(Node):
    def __init__(self, drone, openloop, value, throttle, secs):
        super().__init__("rate_probe")
        self.openloop = openloop
        self.value = value
        self.throttle = throttle
        self.secs = secs
        self.gyro = []          # (t, wx, wy, wz) deg/s, straight from the imu
        self.cmd = []           # (t, ch0, ch1, ch3) raw stick units
        self.t0 = None

        # imu is sensor data - best effort, or we may silently receive nothing
        qos = QoSProfile(depth=50, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Imu, f"/drone_{drone}/imu", self._imu, qos)
        self.create_subscription(
            ELRSCommand, f"/drone_{drone}/ELRSCommand", self._cmd, 10)

        if openloop:
            self.pub = self.create_publisher(
                ELRSCommand, f"/drone_{drone}/ELRSCommand", 10)
            self.create_timer(0.02, self._drive)
            self.get_logger().warn(
                f"OPEN LOOP: commanding {openloop}={value} throttle={throttle} "
                f"for {secs}s on drone {drone}")

    def _t(self, stamp):
        t = stamp.sec + stamp.nanosec * 1e-9
        if self.t0 is None:
            self.t0 = t
        return t - self.t0

    def _imu(self, m):
        w = m.angular_velocity
        self.gyro.append((self._t(m.header.stamp),
                          *np.degrees([w.x, w.y, w.z])))

    def _cmd(self, m):
        self.cmd.append((self._t(self.get_clock().now().to_msg()),
                         m.channel_0, m.channel_1, m.channel_3))

    def _drive(self):
        msg = ELRSCommand(armed=True, channel_0=0.0, channel_1=0.0,
                          channel_2=float(self.throttle * 2 - 1), channel_3=0.0)
        setattr(msg, CHAN[self.openloop], float(self.value))
        self.pub.publish(msg)


def report(gyro, cmd, openloop, value):
    if not gyro:
        print("NO IMU DATA - check the topic name and that the bridge is up.")
        return
    g = np.array(gyro)
    print(f"\ngyro samples: {len(g)} over {g[-1,0]-g[0,0]:.1f}s "
          f"({len(g)/max(1e-6,g[-1,0]-g[0,0]):.0f} Hz)")

    if openloop:
        j = AXES[openloop] + 1
        w = g[:, j]
        settled = w[len(w) // 2:]          # ignore the spin-up transient
        print(f"commanded {openloop} rate stick = {value:+.3f}")
        print(f"measured  {openloop} rate       = {settled.mean():+.1f} deg/s "
              f"(sd {settled.std():.1f})")
        same = np.sign(settled.mean()) == np.sign(value)
        print(f"\n  => sign {'MATCHES' if same else 'IS INVERTED'}"
              f"  ({'correct' if same else 'THIS IS THE BUG'})")
        for nm, k in AXES.items():
            if k != AXES[openloop]:
                o = g[len(g) // 2:, k + 1]
                print(f"     cross-axis {nm}: {o.mean():+.1f} deg/s")
        return

    if not cmd:
        print("NO COMMAND DATA - is the controller stack running?")
        return
    c = np.array(cmd)
    print(f"cmd samples : {len(c)}")
    for nm, k in AXES.items():
        ci = {"roll": 1, "pitch": 2, "yaw": 3}[nm]
        stick = np.interp(g[:, 0], c[:, 0], c[:, ci])
        w = g[:, k + 1]
        m = np.abs(stick) > 0.05
        if m.sum() < 50:
            print(f"  {nm:5s}: n={m.sum()} insufficient command activity")
            continue
        cc = np.corrcoef(stick[m], w[m])[0, 1]
        tag = ("INVERTED" if cc < -0.2 else
               "ok" if cc > 0.2 else "no response")
        print(f"  {nm:5s}: n={m.sum():6d} corr={cc:+.3f}  {tag}")


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--drone", type=int, default=0)
    ap.add_argument("--secs", type=float, default=30.0)
    ap.add_argument("--openloop", choices=list(AXES), default=None)
    ap.add_argument("--value", type=float, default=0.3)
    ap.add_argument("--throttle", type=float, default=0.25)
    a = ap.parse_args()

    rclpy.init()
    n = RateProbe(a.drone, a.openloop, a.value, a.throttle, a.secs)
    try:
        end = n.get_clock().now().nanoseconds * 1e-9 + a.secs
        while rclpy.ok() and n.get_clock().now().nanoseconds * 1e-9 < end:
            rclpy.spin_once(n, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    report(n.gyro, n.cmd, a.openloop, a.value)
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

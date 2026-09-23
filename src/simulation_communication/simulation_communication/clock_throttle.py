"""
clock_throttle.py -- republish Gazebo's /clock at a bounded rate.

Gazebo publishes /clock every physics step (~700-1000 Hz here) and rclpy handles the
sim-time clock in a PYTHON callback, so every node on use_sim_time burns CPU on it: with
~20 nodes that was ~14k callbacks/s and the RTF sat at 0.35 before anything flew
(2026-09-23). The bridge is remapped to /clock_gz and this one node forwards it to /clock
at `rate_hz`. Sim time still comes from Gazebo; only its granularity changes.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rosgraph_msgs.msg import Clock


class ClockThrottle(Node):
    def __init__(self):
        super().__init__('clock_throttle')
        self.rate_hz = float(self.declare_parameter('rate_hz', 100.0).value)
        self._period = 1.0 / max(self.rate_hz, 1e-3)
        self._last = None
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self._pub = self.create_publisher(Clock, '/clock', qos)
        self.create_subscription(Clock, '/clock_gz', self._cb, qos)
        self.get_logger().info(f'clock_throttle: /clock_gz -> /clock at {self.rate_hz:.0f} Hz')

    def _cb(self, msg):
        t = msg.clock.sec + msg.clock.nanosec * 1e-9
        if self._last is None or t - self._last >= self._period or t < self._last:
            self._last = t
            self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ClockThrottle()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

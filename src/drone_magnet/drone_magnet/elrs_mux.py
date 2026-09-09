"""
elrs_mux.py
-----------
Two-input ELRSCommand multiplexer for the ATTACH handoff. The approach drone is flown by
the collaborator's MPC (controller_mpc_payload) until its magnet welds to the payload, then
by our dissipative tracker (controller_quad_load). Both stacks emit an ELRSCommand on
`/drone_{id}/ELRSCommand` through the shared CallbackManager; in the launch each is remapped
to a pre-mux topic so they don't collide:

    approach  -> /drone_{id}/ELRSCommand_tejen   (forwarded BEFORE the weld)
    ours      -> /drone_{id}/ELRSCommand_diss    (forwarded AFTER  the weld)

This node forwards the selected stream to the real `/drone_{id}/ELRSCommand`, which the
betaflight inner-loop (payload_betaflight_comm) turns into motor speeds. When to switch is
`handover_policy.HandoverPolicy` -- read it, that is where the rule and its history live.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from interfaces.msg import ELRSCommand

from drone_magnet.handover_policy import HandoverPolicy


class ElrsMux(Node):
    def __init__(self):
        super().__init__('elrs_mux')
        drone_id = int(self.declare_parameter('drone_id', 3).value)
        self.policy = HandoverPolicy(
            latch=bool(self.declare_parameter('latch', True).value),
            # Off restores the unconditional switch-on-weld, which is only safe if the
            # tracker is warm before the weld (dissipative_node._publish_pending_attach_refs).
            require_live=bool(
                self.declare_parameter('require_live_takeover', True).value),
            live_timeout_s=float(self.declare_parameter('live_timeout_s', 0.5).value))
        self._attached = False

        self.pub = self.create_publisher(ELRSCommand, f'/drone_{drone_id}/ELRSCommand', 1)
        self.create_subscription(ELRSCommand, f'/drone_{drone_id}/ELRSCommand_tejen',
                                 self._tejen_cb, 1)
        self.create_subscription(ELRSCommand, f'/drone_{drone_id}/ELRSCommand_diss',
                                 self._diss_cb, 1)
        self.create_subscription(Bool, '/magnet/object_attached', self._attached_cb, 10)
        self.get_logger().info(
            f'[elrs_mux] drone {drone_id}: forwarding APPROACH until /magnet/object_attached'
            f' (latch={self.policy.latch}, require_live={self.policy.require_live})')

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _apply(self, attached):
        if attached == self._attached:
            if self.policy.welded and not attached:
                self.get_logger().error(
                    '[elrs_mux] welded but the dissipative tracker is idle/silent - '
                    'HOLDING approach authority (switching now would cut the motors)',
                    throttle_duration_sec=1.0)
            return
        self._attached = attached
        self.get_logger().info(
            f"[elrs_mux] -> {'DISSIPATIVE tracker' if attached else 'APPROACH controller'}")

    def _attached_cb(self, msg: Bool):
        self._apply(self.policy.weld(bool(msg.data), self._now()))

    def _tejen_cb(self, msg: ELRSCommand):
        if not self._attached:
            self.pub.publish(msg)

    def _diss_cb(self, msg: ELRSCommand):
        # Evaluated on the incoming stream, so authority transfers on the first flying
        # command after the weld rather than one weld-signal period later.
        self._apply(self.policy.tracker_command(
            bool(msg.armed), float(msg.channel_2), self._now()))
        if self._attached:
            self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ElrsMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

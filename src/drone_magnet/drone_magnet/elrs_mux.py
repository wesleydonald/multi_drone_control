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
from std_msgs.msg import Bool, String
from interfaces.msg import ELRSCommand

from drone_magnet.handover_policy import HandoverPolicy, IDLE_THROTTLE


def merge_magnet_channel(msg, channel, value):
    """Write the magnet aux channel into a command about to be forwarded. On hardware the
    magnet manager publishes its ON/OFF on a separate ELRSCommand topic that never reaches
    the radio by itself; the mux is the one place every forwarded command passes through.
    None = no magnet command seen yet, leave the field alone."""
    if value is None:
        return msg
    field = f'channel_{int(channel)}'
    if hasattr(msg, field):
        setattr(msg, field, float(max(-1.0, min(1.0, value))))
    return msg


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
        # partner handover (his /join_planner/handoff_ready at ATTACH_READY): our tracker
        # takes over for the last descent onto the plate, through the same live-takeover
        # rule as the weld ('' = off)
        handoff_topic = str(self.declare_parameter('handoff_topic', '').value)
        if handoff_topic:
            self.create_subscription(Bool, handoff_topic, self._handoff_cb, 10)
        # partner release (a start-welded drone after its detach and step-out): hand it to
        # the approach stream on that stream's first flying command, never before
        self._release_pending = False
        self._released = False
        release_topic = str(self.declare_parameter('release_topic', '').value)
        if release_topic:
            self.create_subscription(String, release_topic, self._release_cb, 5)
        # magnet aux channel merge ('' = off, the sim has no magnet radio)
        self._magnet_channel = int(self.declare_parameter('magnet_channel', 10).value)
        # no approach controller in the loop (the tracker flies the approach itself):
        # forward OURS from the start, the weld changes nothing here
        self._ours_only = bool(self.declare_parameter('approach_stream', True).value) is False
        self._magnet_value = None
        magnet_topic = str(self.declare_parameter('magnet_command_topic', '').value)
        if magnet_topic:
            self.create_subscription(ELRSCommand, magnet_topic, self._magnet_cb, 5)
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
        if self._ours_only:
            return                        # nothing to hand over: ours flies it throughout
        self._apply(self.policy.weld(bool(msg.data), self._now()))

    def _release_cb(self, msg: String):
        if self._released or self._ours_only or '"running": true' not in msg.data:
            return
        self._release_pending = True

    def _handoff_cb(self, msg: Bool):
        if self._ours_only or not msg.data:
            return
        self._apply(self.policy.weld(True, self._now()))

    def _magnet_cb(self, msg: ELRSCommand):
        self._magnet_value = float(getattr(msg, f'channel_{self._magnet_channel}', 0.0))

    def _forward(self, msg):
        self.pub.publish(merge_magnet_channel(msg, self._magnet_channel, self._magnet_value))

    def _tejen_cb(self, msg: ELRSCommand):
        if self._release_pending and msg.armed and float(msg.channel_2) > IDLE_THROTTLE:
            self._release_pending = False
            self._released = True
            self.policy = HandoverPolicy(latch=self.policy.latch,
                                         require_live=self.policy.require_live,
                                         live_timeout_s=self.policy.live_timeout_s)
            self._attached = False
            self.get_logger().info('[elrs_mux] RELEASED -> APPROACH controller (partner mission)')
        if not self._attached and not self._ours_only:
            self._forward(msg)

    def _diss_cb(self, msg: ELRSCommand):
        if self._ours_only:
            self._forward(msg)
            return
        # Evaluated on the incoming stream, so authority transfers on the first flying
        # command after the weld rather than one weld-signal period later.
        self._apply(self.policy.tracker_command(
            bool(msg.armed), float(msg.channel_2), self._now()))
        if self._attached:
            self._forward(msg)


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

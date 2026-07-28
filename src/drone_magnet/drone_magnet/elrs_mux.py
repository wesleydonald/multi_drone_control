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
betaflight inner-loop (payload_betaflight_comm) turns into motor speeds. The switch is keyed
on `/magnet/object_attached` (Bool) -- the same weld signal that folds the drone into the
dissipative network -- so control authority and network membership flip together. The switch
LATCHES on the first weld (default) so a transient drop of the signal cannot hand the drone
back to the approach controller mid-flight.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from interfaces.msg import ELRSCommand


class ElrsMux(Node):
    def __init__(self):
        super().__init__('elrs_mux')
        drone_id = int(self.declare_parameter('drone_id', 3).value)
        self._latch = bool(self.declare_parameter('latch', True).value)

        self._attached = False           # False -> forward approach; True -> forward ours
        self.pub = self.create_publisher(ELRSCommand, f'/drone_{drone_id}/ELRSCommand', 1)
        self.create_subscription(ELRSCommand, f'/drone_{drone_id}/ELRSCommand_tejen',
                                 self._tejen_cb, 1)
        self.create_subscription(ELRSCommand, f'/drone_{drone_id}/ELRSCommand_diss',
                                 self._diss_cb, 1)
        self.create_subscription(Bool, '/magnet/object_attached', self._attached_cb, 10)
        self.get_logger().info(
            f'[elrs_mux] drone {drone_id}: forwarding APPROACH until /magnet/object_attached'
            f' (latch={self._latch})')

    def _attached_cb(self, msg: Bool):
        if msg.data and not self._attached:
            self._attached = True
            self.get_logger().info('[elrs_mux] weld detected -> switching to DISSIPATIVE tracker')
        elif not msg.data and self._attached and not self._latch:
            self._attached = False
            self.get_logger().info('[elrs_mux] weld released -> switching back to APPROACH')

    def _tejen_cb(self, msg: ELRSCommand):
        if not self._attached:
            self.pub.publish(msg)

    def _diss_cb(self, msg: ELRSCommand):
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

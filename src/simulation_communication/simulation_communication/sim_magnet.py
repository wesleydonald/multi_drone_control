"""
sim_magnet.py — the sim stand-in for the radio's magnet latch, so the RViz MAGNET
toggles do in Gazebo what they do on the rig.

On the rig, /drone_i/magnet (String ON|OFF) is latched by elrs_interface into aux
channel 6: OFF drops the ring plate from the magnet at the tip of drone i's rod. Here
OFF releases tether i's DetachableJoint (stub <-> rod, generate_rigid_world.py), the
same release the dissipative node's DETACH uses. The launch remaps the bridge's ROS side
to /drone_i/magnet_release so it never shares /drone_i/detach with the control launch.

ON after an OFF cannot be modelled: Gazebo re-welds at the current relative pose, a
rigid bar across whatever gap there is, so the node warns and does nothing (restart
Gazebo). Drones at or above num_tethers (the attach newcomer) have no joint here; their
magnet belongs to the magnet manager.

    ros2 run simulation_communication sim_magnet --ros-args -p num_drones:=4 -p num_tethers:=3
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, String


def magnet_action(state, tether, attached):
    """What an ON|OFF on one drone's magnet does: 'release', 'none', 'cannot_reattach',
    'newcomer' (no joint in this world) or 'ignore' (not ON/OFF)."""
    state = state.strip().upper()
    if state not in ('ON', 'OFF'):
        return 'ignore'
    if not tether:
        return 'newcomer'
    if state == 'OFF':
        return 'release' if attached else 'none'
    return 'none' if attached else 'cannot_reattach'


class SimMagnet(Node):
    def __init__(self):
        super().__init__('sim_magnet')
        self.num_drones = int(self.declare_parameter('num_drones', 2).value)
        self.num_tethers = int(self.declare_parameter('num_tethers', self.num_drones).value)
        # every tether spawns welded; the joint's own echo keeps this true after a DETACH
        self.attached = [True] * self.num_tethers
        self._unconfirmed = {}   # tether -> release time, until the joint echoes 'detached'
        self.release_pubs = [self.create_publisher(Empty, f'/drone_{i}/magnet_release', 1)
                             for i in range(self.num_tethers)]
        for i in range(self.num_drones):
            self.create_subscription(String, f'/drone_{i}/magnet',
                                     lambda m, i=i: self._on_magnet(i, m.data), 10)
        for i in range(self.num_tethers):
            self.create_subscription(String, f'/drone_{i}/detachable_joint_state',
                                     lambda m, i=i: self._on_joint(i, m.data), 10)
        self.create_timer(1.0, self._check_released)
        self.get_logger().info(
            f'sim_magnet: MAGNET toggles release tethers 0-{self.num_tethers - 1} in Gazebo.')

    def _on_joint(self, i, data):
        self.attached[i] = data.strip().lower() != 'detached'
        if not self.attached[i]:
            self._unconfirmed.pop(i, None)

    def _check_released(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        for i, t in list(self._unconfirmed.items()):
            if now - t > 2.0:
                del self._unconfirmed[i]
                self.get_logger().warn(f'Drone {i}: no release echo from Gazebo; this '
                                       'world may have no DetachableJoint on that tether.')

    def _on_magnet(self, i, data):
        tether = i < self.num_tethers
        act = magnet_action(data, tether, self.attached[i] if tether else False)
        log = self.get_logger()
        if act == 'release':
            self.release_pubs[i].publish(Empty())
            self.attached[i] = False
            self._unconfirmed[i] = self.get_clock().now().nanoseconds * 1e-9
            log.warn(f'Drone {i} magnet OFF: tether released from the ring. '
                     'The sim cannot re-seat it; restart Gazebo to re-weld.')
        elif act == 'cannot_reattach':
            log.warn(f'Drone {i} magnet ON: the sim cannot re-weld a released tether '
                     '(restart Gazebo). On the rig the magnet would grab on contact.')
        elif act == 'newcomer':
            log.info(f'Drone {i} magnet {data.strip().upper()}: no tether joint in this '
                     'world; the magnet manager owns this magnet.')
        elif act == 'ignore':
            log.warn(f"Drone {i} magnet: '{data}' is not ON or OFF, ignored.")


def main(args=None):
    rclpy.init(args=args)
    node = SimMagnet()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

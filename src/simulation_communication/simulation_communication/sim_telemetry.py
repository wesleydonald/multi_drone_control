"""
sim_telemetry.py — placeholder Telemetry for simulation (finding F7).

On hardware, `drone_communication/elrs_interface` publishes `/drone_i/telemetry`
(battery voltage, mAh used, RSSI, mode) and the RViz ArmPanel shows a per-drone row.
**Nothing published it in simulation**, so those rows stayed blank off-hardware and the
panel was effectively untested until you were standing at the rig with four drones
armed. This node fills that gap.

Values are CONSTANT placeholders, by decision (Wesley, 2026-08-04): the point is to
exercise the display and the warn path, not to model a battery. A discharge model would
be more realistic and would buy nothing the panel needs.

Because the values are parameters, a test can drive one drone's voltage below the
warn threshold and confirm the operator actually sees it -- which is the only way to
check that path without waiting for a real pack to sag.

    ros2 run simulation_communication sim_telemetry --ros-args -p num_drones:=3
    # prove the low-battery warning is visible:
    ros2 param set /sim_telemetry voltage_drone_1 13.2
"""
import rclpy
from rclpy.node import Node

from interfaces.msg import Telemetry


class SimTelemetry(Node):
    def __init__(self):
        super().__init__('sim_telemetry')

        self.num_drones = int(self.declare_parameter('num_drones', 2).value)
        self.rate_hz = float(self.declare_parameter('rate_hz', 2.0).value)
        # One nominal voltage for the fleet, overridable per drone so a single
        # drone can be driven low to exercise the warning path.
        self.default_v = float(self.declare_parameter('battery_voltage', 16.4).value)
        self.mah = int(self.declare_parameter('battery_mah_used', 0).value)
        self.rssi = int(self.declare_parameter('rssi', -55).value)
        self.mode = str(self.declare_parameter('mode', 'SIM').value)

        self._per_drone_v = []
        for i in range(self.num_drones):
            p = self.declare_parameter(f'voltage_drone_{i}', self.default_v)
            self._per_drone_v.append(p)

        self.pubs = [
            self.create_publisher(Telemetry, f'/drone_{i}/telemetry', 5)
            for i in range(self.num_drones)
        ]
        self.create_timer(1.0 / max(self.rate_hz, 0.1), self._publish)
        self.get_logger().info(
            f'sim_telemetry: {self.num_drones} drones at {self.rate_hz:.1f} Hz, '
            f'{self.default_v:.2f} V placeholder '
            f'(override per drone with voltage_drone_<i>)')

    def _publish(self):
        for i, pub in enumerate(self.pubs):
            msg = Telemetry()
            # Re-read each tick so `ros2 param set` takes effect live -- that is
            # what makes the warning path testable without a real battery.
            msg.battery_voltage = float(
                self.get_parameter(f'voltage_drone_{i}').value)
            msg.battery_mah_used = self.mah
            msg.rssi = self.rssi
            msg.mode = self.mode
            pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimTelemetry()
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

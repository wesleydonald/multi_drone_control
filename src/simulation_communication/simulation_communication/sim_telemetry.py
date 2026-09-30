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
    ros2 param set /sim_telemetry voltage_drone_1 19.8

pack_model true (the rig thrust plant, thrust_map 'rig' on payload_betaflight_comm) replaces
the placeholder with rig_thrust.PackModel per drone, driven by that drone's ELRSCommand
throttle. The true voltage goes on /drone_i/sim/pack_v every step, for the bridge's thrust;
the telemetry carries it quantised, as the rig's link does.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

from interfaces.msg import ELRSCommand, Telemetry
from simulation_communication.rig_thrust import PackModel


class SimTelemetry(Node):
    def __init__(self):
        super().__init__('sim_telemetry')

        self.num_drones = int(self.declare_parameter('num_drones', 2).value)
        self.rate_hz = float(self.declare_parameter('rate_hz', 2.0).value)
        # One nominal voltage for the fleet, overridable per drone so a single
        # drone can be driven low to exercise the warning path.
        self.default_v = float(self.declare_parameter('battery_voltage', 24.6).value)
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
        self.pack_model = bool(self.declare_parameter('pack_model', False).value)
        if self.pack_model:
            self._setup_packs()
        self.create_timer(1.0 / max(self.rate_hz, 0.1), self._publish)
        self.get_logger().info(
            f'sim_telemetry: {self.num_drones} drones at {self.rate_hz:.1f} Hz, '
            + ('pack model (rig thrust plant), /drone_<i>/sim/pack_v' if self.pack_model else
               f'{self.default_v:.2f} V placeholder (override per drone with voltage_drone_<i>)'))

    def _setup_packs(self):
        kw = dict(
            v0=float(self.declare_parameter('pack_v0', 24.4).value),
            r_sag=float(self.declare_parameter('pack_r_sag', 1.0).value),
            drain=float(self.declare_parameter('pack_drain', 0.022).value),
            tau=float(self.declare_parameter('pack_tau', 5.0).value),
            quant=float(self.declare_parameter('pack_quant', 0.1).value))
        step_hz = float(self.declare_parameter('pack_step_hz', 50.0).value)
        self.packs = [PackModel(**kw) for _ in range(self.num_drones)]
        self._u = [0.0] * self.num_drones
        self._t_prev = None
        self.pack_pubs = [
            self.create_publisher(Float32, f'/drone_{i}/sim/pack_v', 10)
            for i in range(self.num_drones)]
        for i in range(self.num_drones):
            self.create_subscription(
                ELRSCommand, f'/drone_{i}/ELRSCommand',
                lambda m, i=i: self._cmd_cb(i, m), 10)
        self.create_timer(1.0 / max(step_hz, 1.0), self._step_packs)

    def _cmd_cb(self, i, msg):
        self._u[i] = max(0.0, min(1.0, (msg.channel_2 + 1) / 2)) if msg.armed else 0.0

    def _step_packs(self):
        t = self.get_clock().now().nanoseconds * 1e-9
        dt = 0.0 if self._t_prev is None else t - self._t_prev
        self._t_prev = t
        if dt < 0.0 or dt > 1.0:        # clock reset or a stall: hold the pack
            dt = 0.0
        for pack, u, pub in zip(self.packs, self._u, self.pack_pubs):
            pub.publish(Float32(data=float(pack.step(u, dt))))

    def _publish(self):
        for i, pub in enumerate(self.pubs):
            msg = Telemetry()
            if self.pack_model:
                msg.battery_voltage = float(self.packs[i].measured())
            else:
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

"""
magnet_gate.py
--------------
The one writer of QUAD4's coil on the rig in the combined M1 contact attach (combined-stack O5;
card 2026-10-10_combined_m1_contact, critic round 3 finding 6).

QUAD4 has ONE electromagnet, on its aux channel, set by its radio's String latch
/drone_3/magnet. Two controllers want it: Tejen's mission (his ball: pickup ON, drop OFF, his
rejoin's OFF/ON) and our ring-magnet manager (the attach: ON from our span, OFF only on our
take-back after the span). This node forwards exactly one of them:

  * his object-magnet command (/tejen/magnet/command) until OUR span begins;
  * from the first ON our manager sends (its latch output, /drone_3/magnet_ours) on: ours only,
    for good. Before that ON, our manager's OFF keepalive is ignored, so a take-back before
    the span never drops his ball, and his OFFs after it never drop QUAD4 off the ring.

The switch is one-way: nothing hands the coil back to him. An operator /magnet/command ON
(the RViz ATTACH button) is our span too, so it must not be pressed before his drop.

His puck (critic F9; /tejen/object_attached, /join_planner/phase):
  * once his puck has been attached, his OFF reaches the coil only in DROP_OBJECT / CLEAR_DROP_ZONE: a LANDING
    (land_now) or any other OFF of his before the drop does not drop the puck over the ring;
  * our first ON takes the coil only while the puck reads released: an ON before the drop (RViz ATTACH) is held.
On a restart it forwards nothing for restore_s while it listens to its own latched state (a previous instance's
'ours', if still held) and to our manager's ON keepalive (2 Hz), so a restarted gate does not hand the coil back.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

PARTNER, OURS = 'partner', 'ours'
DROP_PHASES = ('DROP_OBJECT', 'CLEAR_DROP_ZONE')
ON_WORDS = ('ON', '1', 'TRUE', 'ENABLE', 'ENABLED')
OFF_WORDS = ('OFF', '0', 'FALSE', 'DISABLE', 'DISABLED')
GATE_STATE_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                            durability=DurabilityPolicy.TRANSIENT_LOCAL)


def magnet_word(text):
    """'ON' / 'OFF' for a magnet command string, None for anything else (his NONE, a typo)."""
    w = str(text).strip().upper()
    return 'ON' if w in ON_WORDS else 'OFF' if w in OFF_WORDS else None


class MagnetGate:
    """Which input drives the coil, and what to forward. No ROS: the node feeds it messages.

    partner(word) / ours(word) return the word to publish on the coil latch, or None (dropped).
    `events` collects one-line descriptions of every state change and first drop, for the log."""

    def __init__(self):
        self.source = PARTNER
        self.last = None              # the last word forwarded to the coil
        self.events = []
        self.phase = ''               # his mission phase
        self.puck_now = False         # /tejen/object_attached
        self.puck_ever = False        # his puck has been attached in this flight
        self._dropped_ours_off = False
        self._dropped_partner = False
        self._held_off_in = None
        self._held_on = False

    def his_phase(self, phase):
        self.phase = str(phase).strip().upper()

    def puck(self, attached):
        attached = bool(attached)
        if attached != self.puck_now:
            self.events.append(f'his puck {"on" if attached else "released"}')
        self.puck_now = attached
        self.puck_ever = self.puck_ever or attached

    def partner(self, text):
        w = magnet_word(text)
        if w is None:
            return None
        if self.source != PARTNER:
            if not self._dropped_partner:
                self._dropped_partner = True
                self.events.append(f'his {w} ignored: the coil is ours since our span began')
            return None
        if w == 'OFF' and self.puck_ever and self.phase not in DROP_PHASES:
            if self._held_off_in != self.phase:
                self._held_off_in = self.phase
                self.events.append(f'his OFF held in {self.phase or "an unknown phase"}: after the pickup his OFF '
                                   f'reaches the coil only in {" / ".join(DROP_PHASES)}')
            return None
        return self._forward(w, 'his')

    def ours(self, text):
        w = magnet_word(text)
        if w is None:
            return None
        if self.source == PARTNER:
            if w == 'OFF':
                if not self._dropped_ours_off:
                    self._dropped_ours_off = True
                    self.events.append('our OFF ignored before our span: the coil stays his (ball)')
                return None
            if self.puck_now:
                if not self._held_on:
                    self._held_on = True
                    self.events.append('our ON held: his puck is still on (the coil is ours once '
                                       '/tejen/object_attached reads false)')
                return None
            self.source = OURS
            self.events.append('OURS from now (first ON from the ring-magnet manager): his commands '
                               'are ignored')
        return self._forward(w, 'our')

    def restore(self, state):
        """A previous instance's latched state: 'ours' is taken back at once (never handed to him)."""
        if str(state).strip().lower() == OURS and self.source != OURS:
            self.source = OURS
            self.events.append('OURS restored from the latched gate state (restart)')

    def _forward(self, w, who):
        if w != self.last:
            self.events.append(f'coil {w} ({who} command)')
        self.last = w
        return w


class MagnetGateNode(Node):
    def __init__(self):
        super().__init__('magnet_gate')
        d = int(self.declare_parameter('drone_id', 3).value)
        partner_topic = str(self.declare_parameter('partner_topic', '/tejen/magnet/command').value)
        ours_topic = str(self.declare_parameter('ours_topic', f'/drone_{d}/magnet_ours').value)
        out_topic = str(self.declare_parameter('out_topic', f'/drone_{d}/magnet').value)
        state_topic = str(self.declare_parameter('state_topic', f'/drone_{d}/magnet_gate').value)
        phase_topic = str(self.declare_parameter('phase_topic', '').value)
        puck_topic = str(self.declare_parameter('puck_topic', '').value)
        restore_s = float(self.declare_parameter('restore_s', 1.0).value)
        self.gate = MagnetGate()
        self._t0 = self.get_clock().now().nanoseconds * 1e-9
        self._restore_s = restore_s
        self._restoring = restore_s > 0.0
        # its own latched state, before its publisher exists: a previous instance's sample if one is still held
        self.create_subscription(String, state_topic, self._state_cb, GATE_STATE_QOS)
        self.pub = self.create_publisher(String, out_topic, 10)
        self.state_pub = self.create_publisher(String, state_topic, GATE_STATE_QOS)
        self.create_subscription(String, partner_topic, lambda m: self._on(self.gate.partner, m.data), 10)
        self.create_subscription(String, ours_topic, lambda m: self._on(self.gate.ours, m.data), 10)
        if phase_topic:
            self.create_subscription(String, phase_topic, lambda m: self._on(self.gate.his_phase, m.data),
                                     GATE_STATE_QOS)
        if puck_topic:
            self.create_subscription(Bool, puck_topic, lambda m: self._on(self.gate.puck, m.data), 10)
        if self._restoring:
            self._restore_timer = self.create_timer(restore_s, self._end_restore)
        else:
            self.state_pub.publish(String(data=self.gate.source))
        self.get_logger().info(
            f'[magnet_gate] drone {d}: {out_topic} follows his {partner_topic} until the first ON on '
            f'{ours_topic} (our span), then ours only' + (f'; his puck from {puck_topic}' if puck_topic else ''))

    def _state_cb(self, msg):
        if self._restoring and msg.data.strip().lower() == OURS:
            self._on(self.gate.restore, msg.data)

    def _end_restore(self):
        self._restore_timer.cancel()
        self._restoring = False
        self.state_pub.publish(String(data=self.gate.source))
        if self.gate.last is not None:
            self.pub.publish(String(data=self.gate.last))

    def _on(self, feed, value):
        before = self.gate.source
        out = feed(value)
        for e in self.gate.events:
            # one severity per call site (rclpy refuses a logger call that changes it)
            if 'OURS' in e or 'held' in e:
                self.get_logger().warn(f'[magnet_gate] {e}')
            else:
                self.get_logger().info(f'[magnet_gate] {e}')
        self.gate.events.clear()
        if self._restoring:
            return               # restoring: the state moves, nothing reaches the coil yet
        if out is not None:
            self.pub.publish(String(data=out))
        if self.gate.source != before:
            self.state_pub.publish(String(data=self.gate.source))


def main(args=None):
    rclpy.init(args=args)
    node = MagnetGateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

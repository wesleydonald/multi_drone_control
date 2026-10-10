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
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

PARTNER, OURS = 'partner', 'ours'
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
        self._dropped_ours_off = False
        self._dropped_partner = False

    def partner(self, text):
        w = magnet_word(text)
        if w is None:
            return None
        if self.source != PARTNER:
            if not self._dropped_partner:
                self._dropped_partner = True
                self.events.append(f'his {w} ignored: the coil is ours since our span began')
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
            self.source = OURS
            self.events.append('OURS from now (first ON from the ring-magnet manager): his commands '
                               'are ignored')
        return self._forward(w, 'our')

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
        self.gate = MagnetGate()
        self.pub = self.create_publisher(String, out_topic, 10)
        self.state_pub = self.create_publisher(String, state_topic, GATE_STATE_QOS)
        self.create_subscription(String, partner_topic, lambda m: self._on(self.gate.partner, m), 10)
        self.create_subscription(String, ours_topic, lambda m: self._on(self.gate.ours, m), 10)
        self.state_pub.publish(String(data=self.gate.source))
        self.get_logger().info(
            f'[magnet_gate] drone {d}: {out_topic} follows his {partner_topic} until the first ON on '
            f'{ours_topic} (our span), then ours only')

    def _on(self, feed, msg):
        before = self.gate.source
        out = feed(msg.data)
        for e in self.gate.events:
            # one severity per call site (rclpy refuses a logger call that changes it)
            if 'OURS' in e:
                self.get_logger().warn(f'[magnet_gate] {e}')
            else:
                self.get_logger().info(f'[magnet_gate] {e}')
        self.gate.events.clear()
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

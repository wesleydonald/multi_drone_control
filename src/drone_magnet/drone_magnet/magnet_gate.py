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
  * from his puck's pickup to his drop, his OFF reaches the coil only in his pickup phases (at the pickup site: his
    retry releases there) and in DROP_OBJECT / CLEAR_DROP_ZONE: a LANDING (land_now) or any other OFF of his in the
    transit or over the ring does not drop the puck; after the drop his OFFs pass again (his TAKEN_BACK holds his
    magnet OFF so it cannot re-catch the puck in the net, R1520);
  * our first ON takes the coil only while the puck reads released: an ON before the drop (RViz ATTACH) is held.
On a restart it forwards nothing for restore_s while it listens to its own latched state (a previous instance's
'ours', if still held) and to our manager's ON keepalive (2 Hz), so a restarted gate does not hand the coil back.

The full M1 (start_ours, QUAD4 starts welded as a carrier): the coil is ours from boot (our detach releases it),
his from /partner/release on (his pickup), and ours again from our span's first ON.

The tilt cut (card docs/experiments/2026-10-11_quad4_tilt_cut.md, critic round 2 H-B; cut_state_topic '' = off): when
QUAD4's mux reads 'cut' (it has disarmed the vehicle), the coil goes OFF at once and stays OFF, re-sent at 2 Hz, and
both inputs are dropped until this node restarts: a dead QUAD4 must not hang on plate 3, and neither his rejoin's ON
nor ours may reach it. A restarted gate reads the mux's latched state and cuts again while the mux still reads 'cut'.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

PARTNER, OURS, CUT = 'partner', 'ours', 'cut'
CUT_RESEND_S = 0.5
DROP_PHASES = ('DROP_OBJECT', 'CLEAR_DROP_ZONE')
# his pickup phases, at the pickup site away from the ring: his bounded retry releases there (m1-final 6143989: also
# an object attached but not aligned), so his OFF passes there too
PICKUP_PHASES = ('APPROACH_ABOVE_PICKUP', 'SETTLE_ABOVE_PICKUP', 'DESCEND_TO_PICKUP', 'MAGNET_ATTACH_WAIT',
                 'LIFT_OBJECT')
OFF_PHASES = PICKUP_PHASES + DROP_PHASES
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

    def __init__(self, start_ours=False):
        self.source = OURS if start_ours else PARTNER
        self._release_pending = bool(start_ours)     # the welded start hands the coil to him once
        self.last = None              # the last word forwarded to the coil
        self.events = []
        self.phase = ''               # his mission phase
        self.puck_now = False         # /tejen/object_attached
        self.puck_ever = False        # his puck has been attached in this flight
        self.dropped = False          # his mission has reached the drop
        self._dropped_ours_off = False
        self._dropped_partner = False
        self._held_off_in = None
        self._held_on = False
        self._dropped_after_cut = False

    def his_phase(self, phase):
        self.phase = str(phase).strip().upper()
        if self.phase in DROP_PHASES:
            self.dropped = True

    def puck(self, attached):
        attached = bool(attached)
        if attached != self.puck_now:
            self.events.append(f'his puck {"on" if attached else "released"}')
        self.puck_now = attached
        self.puck_ever = self.puck_ever or attached

    def cut(self):
        """QUAD4's mux has cut it: the coil OFF now and for good; returns the word to publish (always 'OFF')."""
        if self.source != CUT:
            self.source = CUT
            self._release_pending = False
            self.events.append('CUT: the mux has disarmed QUAD4: coil OFF, his and our commands ignored from now')
        self.last = 'OFF'
        return 'OFF'

    def _dropped_cut(self, who, w):
        if not self._dropped_after_cut:
            self._dropped_after_cut = True
            self.events.append(f'{who} {w} ignored: the coil is cut')
        return None

    def partner(self, text):
        w = magnet_word(text)
        if w is None:
            return None
        if self.source == CUT:
            return self._dropped_cut('his', w)
        if self.source != PARTNER:
            if not self._dropped_partner:
                self._dropped_partner = True
                self.events.append(f'his {w} ignored: the coil is ours since our span began')
            return None
        if w == 'OFF' and self.puck_ever and not self.dropped and self.phase not in OFF_PHASES:
            if self._held_off_in != self.phase:
                self._held_off_in = self.phase
                self.events.append(f'his OFF held in {self.phase or "an unknown phase"}: from the pickup to the drop '
                                   f'his OFF reaches the coil only at the pickup or in {" / ".join(DROP_PHASES)}')
            return None
        return self._forward(w, 'his')

    def ours(self, text):
        w = magnet_word(text)
        if w is None:
            return None
        if self.source == CUT:
            return self._dropped_cut('our', w)
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

    def release(self):
        """/partner/release after a welded start (start_ours): the coil is his until our span's first ON."""
        if not self._release_pending or self.source == CUT:
            return
        self._release_pending = False
        self.source = PARTNER
        self.events.append('PARTNER from now (/partner/release: the welded start was released to his mission)')

    def restore(self, state):
        """A previous instance's latched state: 'ours' is taken back at once (never handed to him)."""
        if str(state).strip().lower() == OURS and self.source not in (OURS, CUT):
            self.source = OURS
            self._release_pending = False
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
        start_ours = bool(self.declare_parameter('start_ours', False).value)
        release_topic = str(self.declare_parameter('release_topic', '').value)
        cut_state_topic = str(self.declare_parameter('cut_state_topic', '').value)
        self.gate = MagnetGate(start_ours=start_ours)
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
        if release_topic:
            self.create_subscription(
                String, release_topic,
                lambda m: self._on(lambda _v: self.gate.release(), None) if '"running": true' in m.data else None, 5)
        self._cut_timer = None
        if cut_state_topic:
            # the mux's latched state ('partner' | 'ours' | 'cut' | 'latched'): a cut acts at once, restoring or not
            self.create_subscription(String, cut_state_topic, self._mux_state_cb, GATE_STATE_QOS)
        if self._restoring:
            self._restore_timer = self.create_timer(restore_s, self._end_restore)
        else:
            self.state_pub.publish(String(data=self.gate.source))
        self.get_logger().info(
            f'[magnet_gate] drone {d}: {out_topic} follows his {partner_topic} until the first ON on '
            f'{ours_topic} (our span), then ours only' + (f'; his puck from {puck_topic}' if puck_topic else ''))

    def _mux_state_cb(self, msg):
        if msg.data.strip().lower() != CUT or self.gate.source == CUT:
            return
        self.pub.publish(String(data=self.gate.cut()))
        self.state_pub.publish(String(data=CUT))
        for e in self.gate.events:
            self.get_logger().error(f'[magnet_gate] {e}')
        self.gate.events.clear()
        self._cut_timer = self.create_timer(CUT_RESEND_S, lambda: self.pub.publish(String(data='OFF')))

    def _state_cb(self, msg):
        if self._restoring and msg.data.strip().lower() == OURS:
            self._on(self.gate.restore, msg.data)

    def _end_restore(self):
        self._restore_timer.cancel()
        self._restoring = False
        if self.gate.source == CUT:
            return                   # cut while restoring: published then, and re-sent by its own timer
        self.state_pub.publish(String(data=self.gate.source))
        if self.gate.last is not None:
            self.pub.publish(String(data=self.gate.last))

    def _on(self, feed, value):
        before = self.gate.source
        out = feed(value)
        for e in self.gate.events:
            # one severity per call site (rclpy refuses a logger call that changes it)
            if 'OURS' in e or 'PARTNER' in e or 'held' in e:
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

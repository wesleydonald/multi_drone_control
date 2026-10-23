"""
elrs_mux.py
-----------
Two-input ELRSCommand multiplexer for the ATTACH handoff. The approach drone is flown by
the collaborator's MPC (controller_mpc_payload) until its magnet welds to the payload, then
by our tracker (package tracker). Both stacks emit an ELRSCommand on
`/drone_{id}/ELRSCommand` through the shared CallbackManager; in the launch each is remapped
to a pre-mux topic so they don't collide:

    approach  -> /drone_{id}/ELRSCommand_tejen   (forwarded BEFORE the weld)
    ours      -> /drone_{id}/ELRSCommand_diss    (forwarded AFTER  the weld)

This node forwards the selected stream to the real `/drone_{id}/ELRSCommand`, which the
betaflight inner-loop (payload_betaflight_comm) turns into motor speeds. When to switch is
`handover_policy.HandoverPolicy` -- read it, that is where the rule and its history live.

Fleet abort (card docs/experiments/2026-09-28_mux_abort_latch.md): the partner's stream
ignores /fleet/abort, so while it is selected its next command would re-arm a drone our
abort just disarmed. On the first /fleet/abort the mux latches and forwards only disarmed
idle commands, whichever input is selected, until the node restarts.

Tilt cut (card docs/experiments/2026-10-11_quad4_tilt_cut.md; '' = off): our drone's tilt fault, or the fleet
manager's cut request at LANDED, while the mux forwards the PARTNER's stream: this drone alone is latched to disarmed
idle (state 'cut'), its magnet aux channel OFF in the same frame. The rig coil is the radio's latch, written by
magnet_gate alone, which turns it OFF on this node's 'cut' (critic round 2, H-B). While ours flies it, our tracker's
own disarm already reaches the radio, so nothing changes.
"""
import copy

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from interfaces.msg import ELRSCommand

from drone_magnet.handover_policy import HandoverPolicy, IDLE_THROTTLE

# /drone_i/mux_state ('partner' | 'ours' | 'latched'): latched so a manager that starts
# after the mux still reads it (the fleet manager's ARM gate and fault scoping)
MUX_STATE_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL)
LATCH_REPUBLISH_S = 0.05


def abort_latch_command(msg=None):
    """What a latched mux forwards in place of `msg`: a copy disarmed, throttle idle and
    zero rates. Aux channels (the magnet) are kept. Never mutates `msg`."""
    out = ELRSCommand()
    if msg is not None:
        for name in out.get_fields_and_field_types():
            setattr(out, name, getattr(msg, name))
    out.armed = False
    out.channel_0 = 0.0
    out.channel_1 = 0.0
    out.channel_2 = -1.0
    out.channel_3 = 0.0
    return out


# the partner MPC sends +1 on aux channels 5 and 7-10 when armed; on QUAD4 any of them could be
# mapped to a Betaflight mode, so they are zeroed on his stream (combined-stack O6, Wesley 8 Oct:
# always), as our tracker sends them. Channel 6 is the magnet, set by its own writer.
PARTNER_AUX_ZEROED = (5, 7, 8, 9, 10)


def zero_partner_aux(msg, keep=()):
    """A copy of msg with the partner's aux channels zeroed, except `keep` (the magnet channel,
    which has its own writer). Never mutates msg."""
    out = copy.copy(msg)
    for ch in PARTNER_AUX_ZEROED:
        if ch in keep:
            continue
        field = f'channel_{ch}'
        if hasattr(out, field):
            setattr(out, field, 0.0)
    return out


def clamp_partner_idle(msg):
    """A copy of his command at armed idle: throttle -1 and zero rates (with airmode a rate command spins
    the motors on the mat), armed and aux as sent. Never mutates msg."""
    out = copy.copy(msg)
    out.channel_0 = 0.0
    out.channel_1 = 0.0
    out.channel_2 = -1.0
    out.channel_3 = 0.0
    return out


def planner_flying(status):
    """True when our planner's /fleet/status line ('PLANNER | ...', 'NETWORK | ...') says it flies the ring."""
    return str(status).split('|', 1)[0].strip().upper() in ('PLANNER', 'NETWORK')


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
        self.drone_id = drone_id
        # a drone that starts welded as a carrier is OURS from boot (Wesley 8 Oct, combined-stack O1):
        # it arms at our idle throttle and nothing of the partner's is forwarded before /partner/release
        self._start_ours = bool(self.declare_parameter('start_ours', False).value)
        if self._start_ours:
            self.policy.welded = True
            self.policy._switched = True
            self._attached = True
        # rig, combined M1: zero the partner's aux channels on his stream (the sim plant ignores them)
        self._zero_aux = bool(self.declare_parameter('zero_partner_aux', False).value)
        # rig, combined M1 (critic F3): his stream is held at armed idle until our planner flies the ring (its
        # /fleet/status reads PLANNER or NETWORK): a TAKEOFF of his before ours could not be taken back
        self._hold_partner = bool(self.declare_parameter('partner_hold_until_planner', False).value)
        self._planner_flying = False
        if self._hold_partner:
            self.create_subscription(
                String, str(self.declare_parameter('planner_status_topic', '/fleet/status').value),
                self._planner_status_cb, 5)

        self.pub = self.create_publisher(ELRSCommand, f'/drone_{drone_id}/ELRSCommand', 1)
        self._state = None
        self._state_pub = self.create_publisher(String, f'/drone_{drone_id}/mux_state',
                                                MUX_STATE_QOS)
        # read at the abort, so `ros2 param set` can switch it off for a latch-off arm
        self.declare_parameter('abort_latch', True)
        self._abort_latched = False
        self._last_cmd = None
        self._latch_timer = None
        self.create_subscription(
            String, str(self.declare_parameter('abort_topic', '/fleet/abort').value),
            self._abort_cb, 5)
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
        # take-back (combined M1 contact attach: no attach, our tracker flies it away and lands
        # it), latched by its publisher ('' = off)
        take_back_topic = str(self.declare_parameter('take_back_topic', '').value)
        if take_back_topic:
            self.create_subscription(Bool, take_back_topic, self._take_back_cb, MUX_STATE_QOS)
        # partner release (a start-welded drone after its detach and step-out): hand it to
        # the approach stream on that stream's first flying command, never before
        self._release_pending = False
        self._released = False
        self._taken_back = False
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
        # tilt cut ('' = off): the tracker's fault and the manager's request, volatile and reliable (an old fault
        # must not cut a relaunched mux)
        self._cut = False
        self._cut_timer = None
        self._magnet_off_value = float(self.declare_parameter('magnet_off_value', -1.0).value)
        cut_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.VOLATILE)
        for name in ('tilt_fault_topic', 'cut_topic'):
            topic = str(self.declare_parameter(name, '').value)
            if topic:
                self.create_subscription(Bool, topic, lambda m, why=name: self._cut_cb(m, why), cut_qos)
        self._publish_state()
        self.get_logger().info(
            f'[elrs_mux] drone {drone_id}: '
            + ('OURS from boot until /partner/release' if self._start_ours
               else 'forwarding APPROACH until /magnet/object_attached')
            + f' (latch={self.policy.latch}, require_live={self.policy.require_live})')

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _publish_state(self):
        if self._abort_latched:
            state = 'latched'
        elif self._cut:
            state = 'cut'
        elif self._ours_only or self._attached:
            state = 'ours'
        else:
            state = 'partner'
        if state != self._state:
            self._state = state
            self._state_pub.publish(String(data=state))

    def _abort_cb(self, msg: String):
        if self._abort_latched:
            return
        if not bool(self.get_parameter('abort_latch').value):
            self.get_logger().warn(
                f'[elrs_mux] drone {self.drone_id}: /fleet/abort ({msg.data}) ignored: '
                f'abort_latch is false')
            return
        self._abort_latched = True
        self._publish_disarm()
        self._publish_state()
        # the sim bridges hold the last command, and a wedged input would send nothing
        self._latch_timer = self.create_timer(
            LATCH_REPUBLISH_S, self._publish_disarm,
            clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().error(
            f'[elrs_mux] drone {self.drone_id}: FLEET ABORT LATCHED ({msg.data}) - '
            f'forwarding DISARM only (armed false, throttle idle) from every input until '
            f'this node is restarted')

    def _publish_disarm(self):
        self.pub.publish(merge_magnet_channel(abort_latch_command(self._last_cmd),
                                              self._magnet_channel, self._magnet_value))

    def _cut_cb(self, msg: Bool, why: str):
        if not msg.data or self._cut or self._abort_latched:
            return
        if self._ours_only or self._attached:
            self.get_logger().warn(f'[elrs_mux] drone {self.drone_id}: {why} while ours flies it: not cut (our '
                                   'tracker\'s own disarm reaches the radio)')
            return
        self._cut = True
        self._release_pending = False
        self._magnet_value = self._magnet_off_value       # OFF in every frame from now on
        self._publish_disarm()
        self._publish_state()                             # 'cut': magnet_gate turns the rig coil OFF
        self._cut_timer = self.create_timer(LATCH_REPUBLISH_S, self._publish_disarm,
                                            clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().error(
            f'[elrs_mux] drone {self.drone_id}: CUT ({why}) while the partner flies it - forwarding DISARM only '
            f'(armed false, throttle idle, magnet OFF) for this drone until this node is restarted')

    def _apply(self, attached):
        if attached == self._attached:
            if self.policy.welded and not attached:
                self.get_logger().error(
                    '[elrs_mux] welded but the dissipative tracker is idle/silent - '
                    'HOLDING approach authority (switching now would cut the motors)',
                    throttle_duration_sec=1.0)
            elif self.policy.taken_back and not attached:
                self.get_logger().warn(
                    '[elrs_mux] taken back, but our tracker is not flying yet: the approach '
                    'stream stays on until it is', throttle_duration_sec=2.0)
            return
        self._attached = attached
        self._publish_state()
        self.get_logger().info(
            f"[elrs_mux] -> {'DISSIPATIVE tracker' if attached else 'APPROACH controller'}")

    def _attached_cb(self, msg: Bool):
        if self._ours_only:
            return                        # nothing to hand over: ours flies it throughout
        self._apply(self.policy.weld(bool(msg.data), self._now()))

    def _release_cb(self, msg: String):
        if self._released or self._ours_only or self._taken_back or '"running": true' not in msg.data:
            return
        self._release_pending = True

    def _handoff_cb(self, msg: Bool):
        if self._ours_only or not msg.data:
            return
        self._apply(self.policy.weld(True, self._now()))

    def _take_back_cb(self, msg: Bool):
        if self._ours_only or not msg.data:
            return
        self._taken_back = True
        if self._release_pending:
            # taken back before his stream took over (a welded start released, then our LAND): it stays ours
            self._release_pending = False
            self.get_logger().warn('[elrs_mux] TAKE-BACK before the partner flew it: the release is cancelled')
        if not self.policy.taken_back:
            self.get_logger().warn('[elrs_mux] TAKE-BACK: our tracker takes the drone (no attach)')
        self._apply(self.policy.take_back(self._now()))

    def _planner_status_cb(self, msg: String):
        if not self._planner_flying and planner_flying(msg.data):
            self._planner_flying = True
            self.get_logger().info(f'[elrs_mux] our planner flies the ring ({msg.data.split("|")[0].strip()}): '
                                   f'the partner\'s throttle passes from now')

    def _magnet_cb(self, msg: ELRSCommand):
        if self._cut:
            return                                        # OFF from the cut on (critic round 2, L-A)
        self._magnet_value = float(getattr(msg, f'channel_{self._magnet_channel}', 0.0))

    def _forward(self, msg):
        self._last_cmd = msg
        if self._abort_latched or self._cut:
            msg = abort_latch_command(msg)
        self.pub.publish(merge_magnet_channel(msg, self._magnet_channel, self._magnet_value))

    def _tejen_cb(self, msg: ELRSCommand):
        if self._release_pending and not self._cut and msg.armed and float(msg.channel_2) > IDLE_THROTTLE:
            self._release_pending = False
            self._released = True
            self.policy = HandoverPolicy(latch=self.policy.latch,
                                         require_live=self.policy.require_live,
                                         live_timeout_s=self.policy.live_timeout_s)
            self._attached = False
            self._publish_state()
            self.get_logger().info('[elrs_mux] RELEASED -> APPROACH controller (partner mission)')
        if not self._attached and not self._ours_only:
            if self._hold_partner and not self._planner_flying and msg.armed and (
                    float(msg.channel_2) > -1.0 or msg.channel_0 or msg.channel_1 or msg.channel_3):
                msg = clamp_partner_idle(msg)
                self.get_logger().warn('[elrs_mux] the partner\'s throttle held at armed idle: our planner is not '
                                       'flying the ring yet (fleet takeoff first, then his TAKEOFF)',
                                       throttle_duration_sec=2.0)
            self._forward(zero_partner_aux(msg, keep=(self._magnet_channel,)) if self._zero_aux else msg)

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

"""
tools/experiment/runner_node.py — the ROS side of a headless Gazebo run (§9.1).

It is the operator: it presses ARM / TAKEOFF / ATTACH / LAND on a SIM-TIME schedule,
watches for the failures that make a run worthless, and records the state trace.

THE RECORDING SCHEMA IS THE SIL BENCH'S SCHEMA, ON PURPOSE. `logs/run.csv` here uses the
same column names as `logs/sil.csv` there (t, payload_*, dN_*). §9.3's whole claim is a
comparison between the bench and Gazebo, and a comparison whose two sides need different
loaders is one nobody runs twice. Columns Gazebo cannot supply (per-drone cable
acceleration, tension) are written as NaN rather than omitted, so the schema is stable
and `metrics.py` sees a missing quantity instead of a missing column.

SIM TIME, NEVER WALL TIME. Every deadline in here is measured on /clock. Gazebo's
real-time factor moves with machine load, so a wall-clock schedule makes two runs of the
"same" experiment cover different amounts of flight -- §9.1 lists that as having already
invalidated comparisons once. Wall time appears exactly once, as the startup timeout,
because before /clock is flowing there is nothing else to measure.
"""
import math
import os
import re
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseArray
from rcl_interfaces.msg import Log
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Bool, Empty, Float64MultiArray, Int32, String

from interfaces.msg import ELRSCommand, MotionCaptureState

from .config import wait_lift_arg, wrench_arg

ARM_FAIL = 'fleet manager ARM failed or refused'


def _tilt_deg(q):
    """Angle of body +z from world +z, from a (w, x, y, z) quaternion."""
    w, x, y, z = q
    n = math.sqrt(w * w + x * x + y * y + z * z)
    if n < 1e-9:
        return math.nan
    w, x, y, z = w / n, x / n, y / n, z / n
    cz = 1.0 - 2.0 * (x * x + y * y)       # R[2,2]
    return math.degrees(math.acos(max(-1.0, min(1.0, cz))))


class ExperimentRunner(Node):
    def __init__(self, cfg, run_dir):
        super().__init__('experiment_runner')
        self.cfg = cfg
        self.run_dir = run_dir
        self.n = cfg.n_total

        # Sim clock. Tracked from /clock directly rather than via use_sim_time: the
        # node's own timers must keep ticking even while sim time is stalled, or a
        # stalled Gazebo would freeze the watchdog that exists to catch it.
        self.sim_t = None
        self.t_ready = None
        self.t_weld = None

        self.pose = [None] * self.n            # (x,y,z, qw,qx,qy,qz)
        self.vel = [None] * self.n
        self.pose_stamp = [None] * self.n      # sim time of last mocap
        self.ref = [None] * self.n
        self.cmd = [None] * self.n             # (throttle, armed)
        self.payload = None
        self.payload_ref = None
        self.ball = None                       # partner's pickup object (x, y, z)
        self.attached = False
        self.wall0 = None

        self.rows = []
        self.event_log = []
        self.aborts = []
        self.failures = []                     # hard failures -> nonzero exit
        self.step_i = 0
        self._pending = sorted(cfg.events, key=lambda e: e.t)
        self.launch_requests = []           # LAUNCH events not yet started
        self._subs_ok_since = None
        self._lift_wait = None              # (threshold_z, t_blocked) while WAIT_LIFT holds
        self._waiting_for_weld = False
        self._released = False
        self.t_reweld = None
        self._reweld_wait = None            # t the WAIT_REWELD event fired
        self._last_log_t = None
        self._repeats = []                  # (until, pub, msg) re-sent by resend_pending
        self._phase = None
        self._handoff = False
        self._handoff_fired = None          # t of our own HANDOFF publish (not an observation)
        self._object_attached = False
        self.t_partner_release = None
        self._partner_release_wait = None   # t the WAIT_PARTNER_RELEASE event fired
        self.stop_reason = None            # set when the run ends early, for the manifest

        best = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                          history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Clock, '/clock', self._clock_cb, 10)
        for i in range(self.n):
            self.create_subscription(
                MotionCaptureState, f'/drone_{i}/motion_capture_state',
                lambda m, i=i: self._mocap_cb(m, i), best)
            self.create_subscription(
                ELRSCommand, cfg.cmd_ready_topic.format(i=i),
                lambda m, i=i: self._cmd_cb(m, i), best)
            self.create_subscription(
                Float64MultiArray, f'/drone_{i}/reference_trajectory',
                lambda m, i=i: self._ref_cb(m, i), 1)
        self.create_subscription(
            MotionCaptureState, '/payload/motion_capture_state',
            self._payload_cb, best)
        self.create_subscription(
            Float64MultiArray, '/payload/desired_position',
            self._payload_ref_cb, 5)
        self.create_subscription(Bool, '/magnet/object_attached',
                                 self._attached_cb, 10)
        self.t_landed = None
        self.create_subscription(Bool, '/fleet/landed', self._landed_cb, 5)
        self.create_subscription(String, '/fleet/abort', self._abort_cb, 10)

        # partner-mission stages, stamped on receipt at the runner's sim_t (a /rosout
        # stamp is wall time). His phase is latched: RELIABLE + TRANSIENT_LOCAL to match.
        latched = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/join_planner/phase', self._phase_cb, latched)
        self.create_subscription(Bool, '/join_planner/handoff_ready', self._handoff_cb, 10)
        self.create_subscription(Bool, '/tejen/object_attached', self._object_cb, 10)
        self.create_subscription(Log, '/rosout', self._rosout_cb, 100)
        if cfg.ball_start is not None:
            self.create_subscription(PoseArray, '/model/payload_model/pose',
                                     self._ball_cb, best)

        self.fleet_pub = self.create_publisher(String, '/fleet/command', 10)
        self.magnet_pub = self.create_publisher(String, '/magnet/command', 10)
        self.attach_pub = self.create_publisher(Int32, '/fleet/attach', 10)
        self.detach_pub = self.create_publisher(Int32, '/fleet/detach', 10)
        if any(e.do == 'WRENCH' for e in cfg.events):
            self._wrench_publisher()
        self.handoff_pub = self.create_publisher(Bool, '/join_planner/handoff_ready', 10)
        self.fleet_handover_pub = self.create_publisher(Bool, '/fleet/handover', 10)
        # HANGER_RELEASE publishers exist from the start: one created at the event has no
        # matched subscriber yet, and _publish would fail the run on its first send
        self._hanger_pubs = {}
        for e in cfg.events:
            if e.do == 'HANGER_RELEASE':
                tpl = str(e.arg) if e.arg else '/bench/hanger_{i}/detach'
                for i in range(self.n):
                    topic = tpl.format(i=i)
                    if topic not in self._hanger_pubs:
                        self._hanger_pubs[topic] = self.create_publisher(Empty, topic, 10)
        # WAIT_THRUST: per topic template, which drones have sent an armed, above-idle command
        self._thrust_seen = {}
        self._thrust_wait = None            # (template, t the WAIT_THRUST event fired)
        for e in cfg.events:
            tpl = str(e.arg) if e.do == 'WAIT_THRUST' and e.arg else cfg.cmd_ready_topic
            if e.do != 'WAIT_THRUST' or tpl in self._thrust_seen:
                continue
            self._thrust_seen[tpl] = [False] * self.n
            for i in range(self.n):
                self.create_subscription(
                    ELRSCommand, tpl.format(i=i),
                    lambda m, i=i, tpl=tpl: self._thrust_cb(m, i, tpl), best)

    # ── inputs ───────────────────────────────────────────────────────────────

    def _clock_cb(self, msg):
        self.sim_t = msg.clock.sec + msg.clock.nanosec * 1e-9

    def _mocap_cb(self, msg, i):
        p, o = msg.pose.position, msg.pose.orientation
        self.pose[i] = (p.x, p.y, p.z, o.w, o.x, o.y, o.z)
        t = msg.twist.linear
        self.vel[i] = (t.x, t.y, t.z)
        self.pose_stamp[i] = self.sim_t

    def _payload_cb(self, msg):
        p, o = msg.pose.position, msg.pose.orientation
        self.payload = (p.x, p.y, p.z, o.w, o.x, o.y, o.z)

    def _ref_cb(self, msg, i):
        # [n_nodes, dt, px,py,pz, vx,vy,vz, ax,ay,az, cx,cy,cz, ...] -- node 0's
        # position is the reference the drone is being asked to be at right now.
        if len(msg.data) >= 5:
            self.ref[i] = (float(msg.data[2]), float(msg.data[3]),
                           float(msg.data[4]))

    def _payload_ref_cb(self, msg):
        if len(msg.data) >= 3:
            self.payload_ref = (float(msg.data[0]), float(msg.data[1]),
                                float(msg.data[2]))

    def _cmd_cb(self, msg, i):
        # channel_2 is throttle mapped to [-1, 1] by the tracker; decode it the way
        # payload_betaflight_comm and tools/sil/plant.py both do, so a throttle column
        # means the same number in a Gazebo run and a bench run.
        self.cmd[i] = ((float(msg.channel_2) + 1.0) * 0.5, bool(msg.armed))

    def _thrust_cb(self, msg, i, tpl):
        # -0.99: the trackers' armed-idle is channel_2 = -1.0 (drone_magnet.handover_policy)
        if bool(msg.armed) and float(msg.channel_2) > -0.99:
            self._thrust_seen[tpl][i] = True

    def _landed_cb(self, msg):
        if bool(msg.data) and self.t_landed is None and not math.isnan(self._rel()):
            self.t_landed = self._rel()
            self._log_event('LANDED', '/fleet/landed')

    def _attached_cb(self, msg):
        # release / re-weld tracking (a drone that starts welded, leaves and rejoins)
        if not bool(msg.data) and self.attached and not self._released:
            self._released = True
            self._log_event('RELEASED', 'magnet/object_attached')
        if bool(msg.data) and self._released and self.t_reweld is None:
            self.t_reweld = self._rel()
            self._log_event('REWELD', 'magnet/object_attached')
            self.get_logger().warn(f'[exp] RE-WELD detected at t={self.t_reweld:.2f}')
        if bool(msg.data) and not self.attached:
            self.attached = True
            self.t_weld = self._rel()
            self._log_event('WELD', 'magnet/object_attached')
            self.get_logger().warn(f'[exp] WELD detected at t={self.t_weld:.2f}')

    def _phase_cb(self, msg):
        if msg.data != self._phase:
            self._phase = msg.data
            self._log_event(f'PHASE:{msg.data}', '/join_planner/phase')

    def _handoff_cb(self, msg):
        rising = bool(msg.data) and not self._handoff
        self._handoff = bool(msg.data)
        own = (self._handoff_fired is not None
               and self._rel() - self._handoff_fired < 3.0)
        if rising and not own:
            self._log_event('HANDOFF', '/join_planner/handoff_ready')
            self.partner_handoff_seen = True

    def _object_cb(self, msg):
        if bool(msg.data) != self._object_attached:
            self._object_attached = bool(msg.data)
            self._log_event('PICKUP' if msg.data else 'DROP', '/tejen/object_attached')

    _ROSOUT = ((re.compile(r'\] ATTACH drone (\d+)'), 'FOLD_IN'),
               (re.compile(r'PARTNER RELEASE: drone (\d+)'), 'PARTNER_RELEASE'),
               (re.compile(r'PARTNER HANDOFF()'), 'PARTNER_HANDOFF'))

    def _rosout_cb(self, msg):
        # a failed or refused ARM no longer raises /fleet/abort (Q6b, mux latch card v3), so
        # the manager's log is the only sign of it: fail the run instead of flying nothing
        if msg.name.endswith('fleet_manager'):
            if (('ARM FAILED' in msg.msg or 'ARM REFUSED' in msg.msg)
                    and not any(f.startswith(ARM_FAIL) for f in self.failures)):
                self.failures.append(f'{ARM_FAIL} at t={self._rel():.2f}: {msg.msg}')
            return
        if not msg.name.endswith('dissipative_planner'):
            return
        for pat, name in self._ROSOUT:
            m = pat.search(msg.msg)
            if m:
                self._log_event(name, m.group(1) or None)
                if name == 'PARTNER_RELEASE' and self.t_partner_release is None:
                    self.t_partner_release = self._rel()
                return

    def _ball_cb(self, msg):
        if len(msg.poses) < 2:
            return
        p = msg.poses[1].position
        first = self.ball is None
        self.ball = (p.x, p.y, p.z)
        if first:
            off = max(abs(a - b) for a, b in zip(self.ball, self.cfg.ball_start))
            if off > 0.02:
                self.failures.append(
                    f'pickup object starts at ({p.x:.3f}, {p.y:.3f}, {p.z:.3f}), not '
                    f'{self.cfg.ball_start} +- 0.02: wrong world or pose index')

    def _abort_cb(self, msg):
        self.aborts.append((self._rel(), str(msg.data)))
        self.get_logger().error(f'[exp] /fleet/abort at t={self._rel():.2f}: {msg.data}')

    # ── time ─────────────────────────────────────────────────────────────────

    def _rel(self):
        """Sim seconds since the run clock started (t_ready), or nan."""
        if self.sim_t is None or self.t_ready is None:
            return math.nan
        return self.sim_t - self.t_ready

    def stack_ready(self):
        """Every drone's mocap is flowing and every tracker is commanding.

        Both halves matter. Mocap alone means Gazebo is up but the trackers may still be
        compiling acados; a command alone cannot happen without mocap. Starting the
        schedule early is how a run ends up pressing ARM into the void.
        """
        ok = (self.sim_t is not None
              and all(p is not None for p in self.pose)
              and all(c is not None for c in self.cmd[:self._n_cmd_ready()])
              and self.payload is not None)
        # the fleet manager must be discovered on /fleet/command, and stay so for 2 s:
        # R0602's ARM reached drone 3's tracker but not the fleet manager
        import time as _t
        subs = {i.node_name for i in self.get_subscriptions_info_by_topic('/fleet/command')}
        if ok and 'fleet_manager' in subs:
            if self._subs_ok_since is None:
                self._subs_ok_since = _t.time()
            return _t.time() - self._subs_ok_since >= 2.0
        self._subs_ok_since = None
        return False

    def _n_cmd_ready(self):
        # partner mode: the newcomer is silent until the partner's mission arms it
        partner = str(self.cfg.launch_args.get('partner', '')).lower() in ('true', '1')
        return self.cfg.num_drones if partner else self.n

    def start_clock(self):
        self.t_ready = self.sim_t
        self.wall0 = time.time()
        self.get_logger().warn(
            f'[exp] stack ready at sim t={self.sim_t:.2f}; run clock starts now')

    # ── the schedule ─────────────────────────────────────────────────────────

    def _log_event(self, name, arg=None):
        self.event_log.append((self._rel(), name, '' if arg is None else str(arg)))

    def fire_due_events(self):
        t = self._rel()
        if math.isnan(t):
            return
        if self._waiting_for_weld:
            if self.attached:
                self._waiting_for_weld = False
            else:
                return                      # WAIT_WELD blocks the rest of the schedule
        if self._reweld_wait is not None:
            if self.t_reweld is None:
                return                      # WAIT_REWELD blocks the rest of the schedule
            shift = t - self._reweld_wait
            for e in self._pending:
                e.t += shift
            self._reweld_wait = None
            self.get_logger().warn(f'[exp] t={t:6.2f} re-welded; later events shifted {shift:+.2f} s')
        if self._partner_release_wait is not None:
            if self.t_partner_release is None:
                return                      # WAIT_PARTNER_RELEASE blocks the rest
            shift = t - self._partner_release_wait
            for e in self._pending:
                e.t += shift
            self._partner_release_wait = None
            self.get_logger().warn(
                f'[exp] t={t:6.2f} partner released; later events shifted {shift:+.2f} s')
        if self._thrust_wait is not None:
            tpl, t0 = self._thrust_wait
            if not all(self._thrust_seen[tpl]):
                return                      # WAIT_THRUST blocks the rest of the schedule
            shift = t - t0
            for e in self._pending:
                e.t += shift
            self._thrust_wait = None
            self._log_event('THRUST', f'{shift:.2f}')
            self.get_logger().warn(f'[exp] t={t:6.2f} all drones under thrust; later events shifted {shift:+.2f} s')
        if self._lift_wait is not None:
            z_th, t0, timeout = self._lift_wait
            lifted = self.payload is not None and self.payload[2] >= z_th
            if not lifted and not (timeout and t - t0 >= timeout):
                return                      # WAIT_LIFT blocks the rest of the schedule
            shift = t - t0
            for e in self._pending:
                e.t += shift
            self._lift_wait = None
            self._log_event('LIFTED' if lifted else 'LIFT_TIMEOUT', f'{shift:.2f}')
            self.get_logger().warn(f'[exp] t={t:6.2f} payload {"lifted" if lifted else "NOT lifted (timeout)"}; '
                                   f'later events shifted {shift:+.2f} s')
        while self._pending and self._pending[0].t <= t:
            ev = self._pending.pop(0)
            self._fire(ev)
            if ev.do == 'WAIT_WELD' and not self.attached:
                self._waiting_for_weld = True
                return
            if ev.do == 'WAIT_LIFT':
                z_th, timeout = wait_lift_arg(ev.arg)
                self._lift_wait = (z_th, t, timeout)
                return
            if ev.do == 'WAIT_THRUST':
                self._thrust_wait = (str(ev.arg) if ev.arg else self.cfg.cmd_ready_topic, t)
                return
            if ev.do == 'WAIT_REWELD' and self.t_reweld is None:
                self._reweld_wait = t
                return
            if ev.do == 'WAIT_PARTNER_RELEASE' and self.t_partner_release is None:
                self._partner_release_wait = t
                return

    def _publish(self, pub, msg, what, exclude_self=False):
        """Publish, and SAY SO if nobody is listening.

        A single publish into a topic with zero matched subscribers is silently
        discarded, and the run then proceeds looking completely normal with the
        interesting event never having happened. That is not hypothetical: R0027 fired
        MAGNET ON at t=25, recorded it in events.csv, and the magnet manager never saw
        it -- 70 seconds of Gazebo that could not answer the question it was run for.
        """
        if exclude_self:              # a topic the runner also listens to
            n = sum(1 for s in self.get_subscriptions_info_by_topic(pub.topic_name)
                    if s.node_name != self.get_name())
        else:
            n = pub.get_subscription_count()
        if n == 0:
            self.get_logger().error(
                f'[exp] {what} published to a topic with NO SUBSCRIBERS — this event '
                f'will have no effect. Check the topic name and that the node owning '
                f'it is alive.')
            self.failures.append(f'{what}: no subscriber on the topic at publish time')
        pub.publish(msg)
        return n

    def _wrench_publisher(self):
        """Advertise the gz wrench topic. Done when the runner starts, long before any
        WRENCH: a publisher created at the push itself lost its first message while the
        connection formed, so only the "off" step reached Gazebo (R0855)."""
        if getattr(self, '_gz_wrench_pub', None) is None:
            from gz.msgs10.entity_wrench_pb2 import EntityWrench
            from gz.transport13 import Node as GzNode
            sdf = self.cfg.world if os.path.isabs(self.cfg.world) else os.path.join(
                os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
                'simulation_assets', self.cfg.world)
            world = re.search(r'<world name="([^"]+)"', open(sdf).read()).group(1)
            self._gz_node = GzNode()
            self._gz_wrench_pub = self._gz_node.advertise(f'/world/{world}/wrench/persistent',
                                                          EntityWrench)
            self._wrench_applied = {}
        return self._gz_wrench_pub

    def _set_wrench(self, force, link):
        """Make the persistent force on `link` equal `force`. Persistent wrenches add up in
        Gazebo, so only the change is sent."""
        from gz.msgs10.entity_pb2 import Entity
        from gz.msgs10.entity_wrench_pb2 import EntityWrench
        pub = self._wrench_publisher()
        prev = self._wrench_applied.get(link, (0.0, 0.0, 0.0))
        w = EntityWrench()
        w.entity.name = link
        w.entity.type = Entity.LINK
        w.wrench.force.x, w.wrench.force.y, w.wrench.force.z = (
            float(f - p) for f, p in zip(force, prev))
        pub.publish(w)
        self._wrench_applied[link] = tuple(force)

    def _fire(self, ev):
        if ev.do in ('ARM', 'TAKEOFF', 'LAND', 'ESTOP', 'DISARM'):
            self._publish(self.fleet_pub, String(data=ev.do), f'/fleet/command {ev.do}')
        elif ev.do == 'MAGNET':
            # Event.__init__ has already normalised this to the string 'ON'/'OFF';
            # see the YAML-boolean note there. Repeated for ~2 s (the documented usage
            # is `ros2 topic pub -t 3`) because this one command decides whether the
            # run's whole subject happens, and re-sending it is free.
            self._publish(self.magnet_pub, String(data=str(ev.arg)),
                          f'/magnet/command {ev.arg}')
            self._repeats.append((self._rel() + 2.0, self.magnet_pub,
                                  String(data=str(ev.arg))))
        elif ev.do == 'ATTACH':
            self._publish(self.attach_pub, Int32(data=int(ev.arg)),
                          f'/fleet/attach {ev.arg}')
        elif ev.do == 'DETACH':
            self._publish(self.detach_pub, Int32(data=int(ev.arg)),
                          f'/fleet/detach {ev.arg}')
        elif ev.do == 'HANDOFF':
            # latched True, as his planner does while ATTACH_READY; the consumers act on
            # the first True only
            self._handoff_fired = self._rel()
            self._publish(self.handoff_pub, Bool(data=True), '/join_planner/handoff_ready',
                          exclude_self=True)
            self._repeats.append((self._rel() + 2.0, self.handoff_pub, Bool(data=True)))
        elif ev.do == 'FLEET_HANDOVER':
            # the driver's 5 x 0.3 s (drive_m2_handover.py); the muxes latch the first True
            self._publish(self.fleet_handover_pub, Bool(data=True), '/fleet/handover')
            self._repeats.append((self._rel() + 1.5, self.fleet_handover_pub, Bool(data=True)))
        elif ev.do == 'HANGER_RELEASE':
            tpl = str(ev.arg) if ev.arg else '/bench/hanger_{i}/detach'
            for i in range(self.n):
                topic = tpl.format(i=i)
                if topic not in self._hanger_pubs:
                    self._hanger_pubs[topic] = self.create_publisher(Empty, topic, 10)
                pub = self._hanger_pubs[topic]
                self._publish(pub, Empty(), topic)
                self._repeats.append((self._rel() + 1.0, pub, Empty()))
        elif ev.do == 'GZ_PUB':
            import subprocess
            for _ in range(3):
                subprocess.run(['gz', 'topic', '-t', str(ev.arg), '-m', 'gz.msgs.Empty', '-p', ''],
                               timeout=10, check=False)
            self.get_logger().warn(f'[exp] gz {ev.arg} sent x3')
        elif ev.do == 'WRENCH':
            self._set_wrench(*wrench_arg(ev.arg))
        elif ev.do in ('WAIT_WELD', 'WAIT_LIFT', 'WAIT_REWELD', 'WAIT_PARTNER_RELEASE',
                       'WAIT_THRUST'):
            pass
        elif ev.do == 'LAUNCH':
            self.launch_requests.append(str(ev.arg))     # started by run_experiment's loop
        self._log_event(ev.do, ev.arg)
        self.get_logger().warn(f'[exp] t={self._rel():6.2f} EVENT {ev.do} '
                               f'{ev.arg if ev.arg is not None else ""}')

    def resend_pending(self):
        """Re-publish sticky commands until their windows close (see _fire/MAGNET)."""
        t = self._rel()
        self._repeats = [r for r in self._repeats if t <= r[0]]
        for _, pub, msg in self._repeats:
            pub.publish(msg)

    # ── watchdogs ────────────────────────────────────────────────────────────

    def check_health(self):
        """Fail loudly (§9.1). Returns True if the run should be stopped now."""
        t = self._rel()
        if math.isnan(t):
            return False
        # Grace period. Any long blocking call on the runner's thread (the parameter
        # read-back is the obvious one) leaves every pose_stamp older than the clock by
        # however long it took, and a watchdog armed instantly would call that a dead
        # fleet. One timeout's worth of grace lets the stamps refresh first; a genuinely
        # dead mocap still trips it, just one pose_timeout_s later.
        if t < self.cfg.pose_timeout_s:
            return False
        for i in range(self.n):
            st = self.pose_stamp[i]
            if st is None:
                continue
            age = self.sim_t - st
            if age > self.cfg.pose_timeout_s:
                self.failures.append(
                    f'drone {i} mocap stale for {age:.2f}s sim at t={t:.2f} '
                    f'(pose_timeout_s={self.cfg.pose_timeout_s})')
                return True
        if any(f.startswith(('pickup object starts', ARM_FAIL)) for f in self.failures):
            return True
        if self.aborts and self.cfg.criteria and self.cfg.criteria.forbid_abort:
            self.failures.append(
                f'/fleet/abort fired: {self.aborts[0][1]} at t={self.aborts[0][0]:.2f}')
            return True
        return False

    # ── recording ────────────────────────────────────────────────────────────

    def log_row(self, min_dt=0.02):
        """One row per `min_dt` of SIM time, so the log rate does not depend on how
        fast the machine happened to be running Gazebo."""
        t = self._rel()
        if math.isnan(t) or self.payload is None:
            return
        if self._last_log_t is not None and t - self._last_log_t < min_dt:
            return
        self._last_log_t = t
        p = self.payload
        row = {'t': t, 'wall': time.time() - self.wall0, 'step': self.step_i,
               'payload_x': p[0], 'payload_y': p[1], 'payload_z': p[2],
               'payload_qw': p[3], 'payload_qx': p[4],
               'payload_qy': p[5], 'payload_qz': p[6],
               'payload_tilt_deg': _tilt_deg(p[3:7])}
        pr = self.payload_ref
        row.update(payload_ref_x=pr[0] if pr else math.nan,
                   payload_ref_y=pr[1] if pr else math.nan,
                   payload_ref_z=pr[2] if pr else math.nan)
        for i in range(self.n):
            q = self.pose[i]
            v = self.vel[i] or (math.nan,) * 3
            c = self.cmd[i] or (math.nan, False)
            r = self.ref[i]
            pre = f'd{i}_'
            row[pre + 'x'] = q[0] if q else math.nan
            row[pre + 'y'] = q[1] if q else math.nan
            row[pre + 'z'] = q[2] if q else math.nan
            row[pre + 'vx'], row[pre + 'vy'], row[pre + 'vz'] = v
            row[pre + 'tilt_deg'] = _tilt_deg(q[3:7]) if q else math.nan
            row[pre + 'ref_x'] = r[0] if r else math.nan
            row[pre + 'ref_y'] = r[1] if r else math.nan
            row[pre + 'ref_z'] = r[2] if r else math.nan
            if q and r:
                row[pre + 'track_err'] = float(np.linalg.norm(
                    np.array(q[:3]) - np.array(r[:3])))
            else:
                row[pre + 'track_err'] = math.nan
            # Not observable from outside the tracker in a Gazebo run. Kept as columns
            # so the schema matches the bench's -- see the module docstring.
            row[pre + 'acm'] = math.nan
            row[pre + 'tension'] = math.nan
            row[pre + 'elev_deg'] = math.nan
            row[pre + 'thr'] = c[0]
            row[pre + 'armed'] = int(bool(c[1]))
            row[pre + 'attached'] = int(self.attached and i == self.n - 1)
        if self.cfg.ball_start is not None:
            b = self.ball or (math.nan,) * 3
            row['ball_x'], row['ball_y'], row['ball_z'] = b
        self.rows.append(row)
        self.step_i += 1

    def finished(self):
        t = self._rel()
        if math.isnan(t):
            return False
        if t >= self.cfg.duration_s:
            return True
        # Early stop once the outcome is decided: an abort disarms the whole fleet, so
        # the remaining sim time only records it lying on the floor (see
        # cfg.stop_after_abort_s). The grace window still captures the fall.
        grace = getattr(self.cfg, 'stop_after_abort_s', 0.0)
        if grace > 0.0 and self.aborts:
            t_ab = self.aborts[0][0]
            # Never stop before the criteria window has closed. Otherwise a run that
            # aborts early would be judged on a truncated window and could "pass" a
            # bound it simply was not observed long enough to violate -- an
            # optimisation silently weakening the acceptance test.
            c = self.cfg.criteria
            if c is not None and c.window_s:
                base = self.t_weld if c.window_from == 'weld' else 0.0
                if base is not None and t < base + float(c.window_s):
                    return False
            if math.isfinite(t_ab) and t >= t_ab + grace:
                self.stop_reason = (
                    f'stopped {grace:.0f}s after /fleet/abort at t={t_ab:.1f} '
                    f'(the fleet is disarmed; nothing further to record)')
                return True
        # early stop after a normal landing (timing.stop_after_landed_s > 0)
        land_grace = getattr(self.cfg, 'stop_after_landed_s', 0.0)
        if land_grace > 0.0 and self.t_landed is not None and t >= self.t_landed + land_grace:
            self.stop_reason = f'stopped {land_grace:.0f}s after /fleet/landed at t={self.t_landed:.1f}'
            return True
        return False

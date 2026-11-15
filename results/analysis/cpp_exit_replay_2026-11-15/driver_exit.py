"""The C++ exit's mission side over a 13 Nov recording (nothing flies).

His planner (tejen_mission.online_join_planner.OnlineJoinPlanner, the integrated code as built, uncommitted) runs
without its ROS node: the object is the mission tests' fixture (test_cpp_exit_mission.exit_planner) with the trial
line's resolved values typed over it (planner_params.json, launch_dump_trial_fm9h8.json), driven in timer_callback's
order at 30 Hz on the bag's clock: update_state_machine, build_reference, update_c1f1_shadow_transfer,
select_c1f2_authority_reference, publish_reference. Every exit method it calls is the mission's own (exit_select,
exit_build_prefix, refresh_moving_prefix, MovingExitHandoff, exit_shadow_step, exit_relatively_ready, exit_try_accept,
exit_fail, the 7 s deadline), and its publishers and callbacks are wired to the real topics of his real backend binary.
Not run: inputs_ready(), the safety planner (diagnostic, returns the reference unchanged), markers, CSV logging.

The world: the recorded ring, carriers and ring trajectory state (bag -> ring_bridge -> his backend). QUAD4 is
virtual: its state is the reference the mission last published, evaluated at now from that publication's stamp
(the clearance at the 2.9 m hold -> the prefix -> the C++ reference after the switch), plus, with err_csv, the
recorded tracking error of the flight from its CLEAR_DROP_ZONE entry on (mocap minus /join_planner/reference;
ping-ponged after the recorded LAND). /tejen/object_attached reads false at 30 Hz (the object is released).

params: t_start (bag time of the replay's CLEAR_DROP_ZONE entry), goal_x, goal_y, err_csv ('' = perfect tracking),
err_cut_s, run_s, out (directory)."""
import json
import math
import os
import sys
import time
import traceback

import numpy as np

sys.path.insert(0, os.environ.get('MDC_TEST_DIR', '/home/wesley/multi_drone_control/src/tejen_mission/test'))

import rclpy  # noqa: E402
from geometry_msgs.msg import PoseStamped, TwistStamped  # noqa: E402
from interfaces.msg import CommittedTrajectory, MotionCaptureState  # noqa: E402
from rcl_interfaces.msg import ParameterDescriptor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from std_msgs.msg import Bool, String  # noqa: E402
from trajectory_msgs.msg import MultiDOFJointTrajectory  # noqa: E402

from tejen_mission import c1f2_handoff as h  # noqa: E402
from tejen_mission.mission_types import MissionPhase as P  # noqa: E402

import test_cpp_exit_mission as fixture  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
PARAMS = json.load(open(os.environ.get('PLANNER_PARAMS', os.path.join(HERE, 'planner_params.json'))))
ROD = float(PARAMS['magnet_drop_below_quad'])          # 1.034


class Pub:
    """A planner publisher wired to a real one; keeps the last message."""

    def __init__(self, node, pub, tap=None):
        self.node, self.pub, self.tap, self.messages = node, pub, tap, []

    def publish(self, msg):
        if self.tap is not None:
            self.tap(msg)
        self.pub.publish(msg)


class Log:
    def __init__(self, node):
        self.node = node

    def _w(self, level, msg):
        self.node.plog.write(f'{self.node.now():.3f} {level} {msg}\n')
        self.node.plog.flush()

    def info(self, msg, *a, **k):
        self._w('I', msg)

    def warning(self, msg, *a, **k):
        self._w('W', msg)

    warn = warning

    def error(self, msg, *a, **k):
        self._w('E', msg)

    def debug(self, msg, *a, **k):
        pass


def ping_pong(t, cut):
    if t <= cut:
        return t
    k = math.floor(t / cut)
    r = t - k * cut
    return r if k % 2 == 0 else cut - r


class Driver(Node):
    def __init__(self):
        super().__init__('exit_replay_driver')
        g = lambda n, d: self.declare_parameter(n, d, ParameterDescriptor(dynamic_typing=True)).value
        self.t_start = float(g('t_start', 0.0))
        self.goal = (float(g('goal_x', 0.0)), float(g('goal_y', 1.55)))
        self.run_s = float(g('run_s', 32.0))
        self.out = g('out', HERE)
        err_csv = g('err_csv', '')
        self.err_cut = float(g('err_cut_s', 1e9))
        self.err = None
        if err_csv:
            a = np.loadtxt(err_csv, delimiter=',', skiprows=1)
            a = a[(a[:, 0] >= 0.0) & (a[:, 0] <= self.err_cut)]
            self.err = a
        os.makedirs(self.out, exist_ok=True)
        self.plog = open(os.path.join(self.out, 'planner.log'), 'w')
        self.events = open(os.path.join(self.out, 'events.txt'), 'w')
        self.ticks = open(os.path.join(self.out, 'ticks.csv'), 'w')
        self.ticks.write('t,phase,hstate,seq,source,stamp_ns,px,py,pz,vx,vy,vz,ax,ay,az,step_p,step_v,step_a,'
                         'q4x,q4y,q4z,q4vx,q4vy,q4vz,q4ax,q4ay,q4az,ringx,ringy,ringz,cpp_auth,waiting,tick_ms,'
                         'status_lag_ms,ref_lag_ms\n')
        self.q4log = open(os.path.join(self.out, 'q4.csv'), 'w')
        self.q4log.write('t,cx,cy,cz,x,y,z,vx,vy,vz,ringx,ringy,ringz,rqw,rqx,rqy,rqz\n')
        self.q4n = 0
        self.status_lag = float('nan')
        self.ref_lag = float('nan')
        self.statuslog = open(os.path.join(self.out, 'status.jsonl'), 'w')
        self.reqlog = open(os.path.join(self.out, 'requests.txt'), 'w')
        self.prefixlog = open(os.path.join(self.out, 'prefixes.jsonl'), 'w')
        self.bstatus = open(os.path.join(self.out, 'backend_status.txt'), 'w')
        self.bstatus.write(f'# mission code: {fixture.ojp.__file__} {h.__file__}\n')
        self.reflog = open(os.path.join(self.out, 'cpp_refs.txt'), 'w')

        self.ring = None
        self.ring_q = (1.0, 0.0, 0.0, 0.0)
        self.ring_v = np.zeros(3)
        self.ring_t = None
        self.carriers = {}
        self.started = False
        self.done_t = None
        self.errors = 0
        self.last_backend_status = ''
        self.prev_pub = None            # (stamp_s, TrajectoryReference) of the previous publication
        self.q4_acc = np.zeros(3)
        self.q4_prev = None
        self.mark = {}

        # the planner: the mission tests' fixture, the trial line's values over it
        p = fixture.exit_planner(switch=True)
        self.p = p
        p.get_clock = self.get_clock
        p._physical_now_s = self.now
        p._wall_now_s = time.monotonic
        p.get_logger = lambda: self.logger
        self.logger = Log(self)
        p.dt = 1.0 / 30.0
        p.horizon_samples = max(2, int(math.ceil(3.0 / p.dt)))          # horizon_seconds 3.0 (default) -> 90
        p.min_reference_z, p.max_reference_speed = float(PARAMS['min_reference_z']), float(PARAMS['max_reference_speed'])
        for k in ('exit_prefix_horizon_s', 'exit_prefix_refresh_s', 'exit_blend_s', 'exit_protected_s',
                  'exit_handoff_timeout_s', 'exit_hold_z_m', 'at_start_land_after_s'):
            setattr(p, k, float(PARAMS[k]))
        p.exit_ring_commitment_max_age_s = 2.0
        p.exit_ring_commitment, p.exit_ring_commitment_sequence, p.exit_ring_commitment_receive_s = None, -1, None
        p.c1f2_bridge_max_velocity = np.array([PARAMS[f'c1f2_bridge_max_velocity_{a}_mps'] for a in 'xyz'], float)
        p.c1f2_bridge_max_acceleration = np.array([PARAMS[f'c1f2_bridge_max_acceleration_{a}_mps2'] for a in 'xyz'],
                                                  float)
        p.c1f2_bridge_max_jerk = np.array([PARAMS[f'c1f2_bridge_max_jerk_{a}_mps3'] for a in 'xyz'], float)
        p.c1f2_bridge_feasibility_sample_dt_s = float(PARAMS['c1f2_bridge_feasibility_sample_dt_s'])
        p.c1f2_prepare_max_lift_drift_m = float(PARAMS['c1f2_prepare_max_lift_drift_m'])
        p.c1f2_prepare_settle_speed_mps = float(PARAMS['c1f2_prepare_settle_speed_mps'])
        p.c1f2_prepare_settle_acceleration_mps2 = float(PARAMS['c1f2_prepare_settle_acceleration_mps2'])
        p.c1f2_prepare_settle_dwell_s = float(PARAMS['c1f2_prepare_settle_dwell_s'])
        p.c1f2_handoff_position_tolerance_m = float(PARAMS['c1f2_handoff_position_tolerance_m'])
        p.c1f2_handoff_velocity_tolerance_mps = float(PARAMS['c1f2_handoff_velocity_tolerance_mps'])
        p.c1f2_handoff_acceleration_tolerance_mps2 = float(PARAMS['c1f2_handoff_acceleration_tolerance_mps2'])
        p.c1f1_shadow_reference_timeout_s = float(PARAMS['c1f1_shadow_reference_timeout_s'])
        p.c1f2_nav_capture_xy_threshold_m = float(PARAMS['c1f2_nav_capture_xy_threshold_m'])
        p.c1f2_nav_capture_z_threshold_m = float(PARAMS['c1f2_nav_capture_z_threshold_m'])
        p.drop_point_source, p.drop_offset, p.payload_yaw = 'payload', np.array([0.0, 0.0, 0.15]), 0.0
        p.drop_release_clearance = float(PARAMS['drop_release_clearance'])
        p.drop_clear_height = float(PARAMS['drop_clear_height'])
        p.drop_approach_clearance = float(PARAMS['drop_approach_clearance'])
        p.drop_clear_speed = 0.08                                         # the node's default (not typed)
        p.drop_hold_offset = None
        p.pickup_z_tolerance = float(PARAMS['pickup_z_tolerance'])
        p.magnet_drop_below_quad = ROD
        # the drop trim (target_alignment_mode measured_tip_trim) is taken as zero: the body ROD over the tip
        p.quad_reference_offset_from_tip = lambda: np.array([0.0, 0.0, ROD])
        p.frame_id, p.vehicle_id = 'map', str(PARAMS['vehicle_id'])
        p.takeoff_start_position = np.array([self.goal[0], self.goal[1], 0.06])
        p.input_timeout_s = 0.5
        p.visual_trajectory_acceleration_filter_tau_s = 0.15
        p.c1f1_shadow_reference_count = 0
        p.c1f1_shadow_reference_last_period_ms = float('nan')
        p.c1f1_shadow_reference_terminal = np.full(3, np.nan)
        p.c1f1_shadow_status_last_receive_time = None
        p.c1f1_shadow_backend_status = ''
        p.c1f2_authority_ack_time = None
        p.c1f2_commanded_authority = False
        p.object_attached = False
        p.last_reference = p.last_reference_time_s = None
        p.last_reference_stamp_ns = None
        p.exit_handoff = h.MovingExitHandoff()
        self._instrument(p.exit_handoff)
        p.return_prepare_started, p.return_hover = False, None
        p.exit_climb_tip_z = None
        if os.environ.get('RIG_TRANSFER', '0') == '1':
            # the node's TransferConfig at the trial line's resolved values (online_join_planner.py ~1121-1162), in
            # place of the test fixture's; it flies the start hover (AT_START) and the LAND escape's direct return
            from tejen_mission.reference_generators import TransferConfig, VirtualTransferGenerator
            g2 = lambda k, d: float(PARAMS.get(k, d))
            p.transfer_generator = VirtualTransferGenerator(TransferConfig(
                dt=p.dt, horizon_samples=p.horizon_samples,
                nominal_speed=max(0.05, g2('reference_nominal_speed', 0.30)),
                duration_scale=max(1.0, g2('reference_duration_scale', 1.5)),
                min_duration=max(0.25, g2('reference_min_duration_s', 1.0)),
                max_duration=max(g2('reference_min_duration_s', 1.0), g2('reference_max_duration_s', 6.0)),
                tracking_error_soft=max(0.0, g2('reference_tracking_error_soft_m', 0.10)),
                tracking_error_hard=max(0.02, g2('reference_tracking_error_hard_m', 0.35)),
                minimum_progress_scale=float(np.clip(g2('reference_min_progress_scale', 0.10), 0.0, 1.0)),
                min_reference_z=p.min_reference_z, max_reference_speed=p.max_reference_speed))

        rel = QoSProfile(depth=1)
        rel.reliability, rel.durability = ReliabilityPolicy.RELIABLE, DurabilityPolicy.TRANSIENT_LOCAL
        p.reference_publisher = Pub(self, self.create_publisher(MultiDOFJointTrajectory, '/join_planner/reference', 1))
        p.exit_prefix_publisher = Pub(self, self.create_publisher(CommittedTrajectory,
                                                                  '/dynamic_planner/transfer_prefix', rel),
                                      self._tap_prefix)
        p.c1f2_recovery_request_pub = Pub(self, self.create_publisher(
            String, '/dynamic_planner/reference_recovery_request', 10), self._tap_request)
        p.c1f1_shadow_enable_publisher = Pub(self, self.create_publisher(Bool, '/dynamic_planner/transfer_enable', 10),
                                             lambda m: self.on_change('enable', m.data))
        p.c1f2_authority_publisher = Pub(self, self.create_publisher(Bool, '/dynamic_planner/transfer_authority', 10),
                                         lambda m: self.on_change('authority', m.data))
        p.c1f1_shadow_target_publisher = Pub(self, self.create_publisher(PoseStamped,
                                                                         '/dynamic_planner/transfer_target', 10))
        p.c1f2_target_velocity_publisher = Pub(self, self.create_publisher(
            TwistStamped, '/dynamic_planner/transfer_target_velocity', 10))
        p.c1f2_static_target_publisher = Pub(self, self.create_publisher(Bool, '/dynamic_planner/static_target', 10))

        self.state_pub = self.create_publisher(MotionCaptureState, '/virtual_q4/state', 10)
        self.attached_pub = self.create_publisher(Bool, '/tejen/object_attached', 10)
        # QoS depths as the mission's own subscriptions (10); the carriers are not subscribed (their poses for the
        # geometry come from the bag afterwards: analyze.py), keeping this process light under load
        self.create_subscription(MotionCaptureState, '/payload/motion_capture_state', self._ring_cb, 10)
        self.create_subscription(CommittedTrajectory, '/fake_payload/committed_trajectory', self._ring_commit_cb, rel)
        self.create_subscription(String, '/dynamic_planner/reference_recovery_status', self._recovery_status_cb, 10)
        self.create_subscription(MultiDOFJointTrajectory, '/dynamic_planner/transfer_reference', self._shadow_ref_cb,
                                 10)
        self.create_subscription(String, '/dynamic_planner/transfer_status', self._backend_status_cb, 10)
        self.create_subscription(Bool, '/dynamic_planner/transfer_authority_ack', self._ack_cb, 10)
        self.create_timer(0.01, self._q4_tick)
        self.create_timer(1.0 / 30.0, self._tick)
        self.create_timer(1.0 / 30.0, lambda: self.attached_pub.publish(Bool(data=False)))

    # ---- helpers -------------------------------------------------------------------------------------------------
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def event(self, what):
        t = self.now()
        self.events.write(f'{t:.4f} {t - self.t_start:+.3f} {what}\n')
        self.events.flush()

    def _instrument(self, hs):
        """Log-only wrappers (behaviour unchanged): why the mission did or did not join a granted window."""
        self.acclog = open(os.path.join(self.out, 'accept.txt'), 'w')
        self.acclog.write('t,what,ref_stamp,splice,in_refs,recv_age_ms,src_age_ms,result\n')
        orig_mr, orig_wm = hs.matching_reference, hs.window_matches

        def mr(**kw):
            out = orig_mr(**kw)
            st = hs.status
            if st is not None and st.get('granted') and self.acc_n < 400:
                self.acc_n += 1
                stamp = st.get('reference_stamp_ns', 0)
                item = hs.references.get(stamp)
                now = kw.get('now_s', float('nan'))
                recv = 1e3 * (now - item[1]) if item is not None else float('nan')
                src = 1e3 * (now - hs.started_s - (stamp - hs.started_ros_ns) * 1e-9)
                self.acclog.write(f'{self.now():.4f},match,{stamp * 1e-9:.4f},{st.get("splice_time_ns", 0) * 1e-9:.4f},'
                                  f'{int(item is not None)},{recv:.2f},{src:.3f},{"none" if out is None else "found"}\n')
            return out

        def wm(window, **kw):
            out = orig_wm(window, **kw)
            st = hs.status or {}
            self.acclog.write(f'{self.now():.4f},window,{kw.get("stamp_ns", 0) * 1e-9:.4f},'
                              f'{st.get("splice_time_ns", 0) * 1e-9:.4f},,,,{out[0]} {out[1]}\n')
            return out
        self.acc_n = 0
        hs.matching_reference, hs.window_matches = mr, wm

    def on_change(self, what, value):
        if self.mark.get('_' + what) != value:
            self.mark['_' + what] = value
            self.event(f'{what} {value}')

    def guard(self, name, fn, *args):
        try:
            return fn(*args)
        except Exception:                                   # noqa: BLE001 -- recorded, the replay goes on
            self.errors += 1
            self.event(f'EXCEPTION in {name}: ' + traceback.format_exc().replace('\n', ' | '))
            return None

    def _tap_prefix(self, msg):
        hs = self.p.exit_handoff
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        self.prefixlog.write(json.dumps(dict(t=self.now(), seq=int(msg.sequence), stamp_ns=stamp,
                                             start=msg.pieces[0].valid_from_s, end=msg.pieces[-1].valid_until_s,
                                             pieces=len(msg.pieces), deadline_ns=hs.deadline_ns)) + '\n')
        self.prefixlog.flush()
        self.event(f'prefix {msg.sequence} published ({len(msg.pieces)} piece(s), '
                   f'[{msg.pieces[0].valid_from_s - self.t_start:+.3f}, {msg.pieces[-1].valid_until_s - self.t_start:+.3f}])')

    def _tap_request(self, msg):
        self.reqlog.write(f'{self.now():.4f} {msg.data}\n')
        self.reqlog.flush()
        if ' seedtraj ' in msg.data or ' cancel' in msg.data or ' reset' in msg.data:
            self.event('request ' + msg.data.split(' ', 1)[1])
        elif ' grant ' in msg.data and 'grant' not in self.mark:
            self.mark['grant'] = self.now()
            self.event('request grant (first)')

    # ---- inputs --------------------------------------------------------------------------------------------------
    def _ring_cb(self, m):
        self.ring = np.array([m.pose.position.x, m.pose.position.y, m.pose.position.z])
        o = m.pose.orientation
        self.ring_q = (o.w, o.x, o.y, o.z)
        self.ring_v = np.array([m.twist.linear.x, m.twist.linear.y, m.twist.linear.z])
        self.ring_t = self.now()
        p = self.p
        p.payload_position, p.payload_velocity = self.ring.copy(), self.ring_v.copy()
        p.payload_yaw = math.atan2(2.0 * (o.w * o.z + o.x * o.y), 1.0 - 2.0 * (o.y * o.y + o.z * o.z))
        p.last_payload_pose_time = p.last_payload_twist_time = self.ring_t

    def _ring_commit_cb(self, m):
        if self.started:
            self.guard('exit_ring_commitment_callback', self.p.exit_ring_commitment_callback, m)
        else:
            self.pending_commit = m

    def _recovery_status_cb(self, m):
        t = self.now()
        try:
            st = json.loads(m.data)
        except ValueError:
            return
        st['_t'] = t
        if type(st.get('stamp_ns')) is int:
            self.status_lag = t - st['stamp_ns'] * 1e-9
        self.statuslog.write(json.dumps(st) + '\n')
        for key, cond in (('exit_context', st.get('exit_context') is True),
                          ('seed_prefix', st.get('seed_kind') == 'prefix' and st.get('prefix_sequence', 0) >= 1),
                          ('prepared', st.get('prepared') is True), ('granted', st.get('granted') is True)):
            if cond and key not in self.mark:
                self.mark[key] = t
                self.event(f'backend status {key} (seq {st.get("prefix_sequence")}, reason {st.get("reason")!r})')
        rr = st.get('exit_reject_reason', '')
        if rr and self.mark.get('reject_reason') != rr:
            self.mark['reject_reason'] = rr
            self.event(f'backend exit_reject_reason {rr}')
        if self.started:
            self.guard('c1f2_recovery_status_callback', self.p.c1f2_recovery_status_callback, m)

    def _shadow_ref_cb(self, m):
        self.ref_lag = self.now() - (m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec)
        if self.started:
            self.reflog.write(f'{self.now():.4f},{m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec:.4f}\n')
        if self.started:
            self.guard('c1f1_shadow_reference_callback', self.p.c1f1_shadow_reference_callback, m)

    def _backend_status_cb(self, m):
        for line in m.data.splitlines():
            if line.startswith('latest_status:'):
                s = line.split(':', 1)[1].strip()
                if s != self.last_backend_status:
                    self.bstatus.write(f'{self.now():.4f} {self.now() - self.t_start:+.3f} {s}\n')
                    self.bstatus.flush()
                self.last_backend_status = s
        if self.started:
            self.guard('c1f1_shadow_status_callback', self.p.c1f1_shadow_status_callback, m)

    def _ack_cb(self, m):
        if m.data and 'ack' not in self.mark:
            self.mark['ack'] = self.now()
            self.event('backend authority ack true')
        if self.started:
            self.guard('c1f2_authority_ack_callback', self.p.c1f2_authority_ack_callback, m)

    # ---- the virtual QUAD4 ---------------------------------------------------------------------------------------
    def error_at(self, t_rel):
        if self.err is None or t_rel < 0.0:
            return np.zeros(3), np.zeros(3)
        tq = ping_pong(t_rel, self.err[-1, 0])
        e = np.array([np.interp(tq, self.err[:, 0], self.err[:, j]) for j in (1, 2, 3)])
        ev = np.array([np.interp(tq, self.err[:, 0], self.err[:, j]) for j in (4, 5, 6)])
        return e, ev

    def commanded(self, t):
        """The reference the mission last published, at t (its samples at stamp + i dt)."""
        p = self.p
        if p.last_reference is None or p.last_reference_stamp_ns is None:
            return None
        w = h.advance_reference_window(p.last_reference, elapsed_s=max(0.0, t - p.last_reference_stamp_ns * 1e-9),
                                       sample_dt_s=p.dt, output_count=1)
        return w.positions[0], w.velocities[0]

    def _q4_tick(self):
        t = self.now()
        if t <= 0.0 or self.ring is None:
            return
        c = self.commanded(t) if self.started else None
        if c is None:
            # before the clearance: the clearance reference's start over the ring (release height, ring velocity)
            cp = self.ring + np.array([0.0, 0.0, 0.15 + float(PARAMS['drop_release_clearance']) + ROD])
            cv = self.ring_v.copy()
        else:
            cp, cv = c
        e, ev = self.error_at(t - self.t_start)
        pos, vel = cp + e, cv + ev
        m = MotionCaptureState()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'map'
        m.child_frame_id = 'drone_3'
        m.pose.position.x, m.pose.position.y, m.pose.position.z = (float(x) for x in pos)
        m.pose.orientation.w = 1.0
        m.twist.linear.x, m.twist.linear.y, m.twist.linear.z = (float(x) for x in vel)
        self.state_pub.publish(m)
        # the mission's drone_state_callback: p, v and its first-order filtered acceleration (tau 0.15 s)
        p = self.p
        if self.q4_prev is not None:
            dt = max(1e-4, t - self.q4_prev[0])
            raw = (vel - self.q4_prev[1]) / dt
            alpha = 1.0 - math.exp(-dt / p.visual_trajectory_acceleration_filter_tau_s)
            self.q4_acc = self.q4_acc + alpha * (raw - self.q4_acc)
        self.q4_prev = (t, vel.copy())
        p.drone_position, p.drone_velocity, p.drone_acceleration = pos.copy(), vel.copy(), self.q4_acc.copy()
        p.last_drone_time = t
        p.measured_magnet_tip_position = pos - np.array([0.0, 0.0, ROD])
        p.last_magnet_tip_pose_time = p.last_attachment_pose_time = p.last_object_attached_time = t
        self.q4n += 1
        if self.started and self.q4n % 4 == 0:
            row = [t, *cp, *pos, *vel, *self.ring, *self.ring_q]
            self.q4log.write(','.join(f'{x:.5f}' for x in row) + '\n')

    # ---- the planner tick (timer_callback's order) -------------------------------------------------------------------
    def _tick(self):
        t = self.now()
        if t <= 0.0 or self.ring is None:
            return
        p = self.p
        if not self.started:
            if t < self.t_start:
                return
            self.started = True
            self.t_start = t
            p.phase, p.phase_entry_time, p.condition_start_time = P.CLEAR_DROP_ZONE, t, None
            self.event(f'CLEAR_DROP_ZONE (replay start), goal ({self.goal[0]:.3f}, {self.goal[1]:.3f}), '
                       f'error {"recorded" if self.err is not None else "none (perfect tracking)"}')
            if getattr(self, 'pending_commit', None) is not None:
                self.guard('exit_ring_commitment_callback', p.exit_ring_commitment_callback, self.pending_commit)
        w0 = time.perf_counter()
        phase0, hstate0, seq0 = p.phase, p.exit_handoff.state, p.exit_handoff.sequence
        self.guard('update_state_machine', p.update_state_machine)
        python = None
        if not self.guard('c1f2_cpp_transit_authority_active', p.c1f2_cpp_transit_authority_active):
            python = self.guard('build_reference', p.build_reference)
        self.guard('update_c1f1_shadow_transfer', p.update_c1f1_shadow_transfer)
        ref = self.guard('select_c1f2_authority_reference', p.select_c1f2_authority_reference, python)
        if ref is None:
            ref = python if python is not None else p.last_reference
        if ref is None:
            return
        p.last_reference, p.last_reference_time_s = ref, t
        self.guard('publish_reference', p.publish_reference, ref)
        tick_ms = 1e3 * (time.perf_counter() - w0)
        hs = p.exit_handoff
        if p.phase != phase0:
            self.event(f'phase {phase0.value} -> {p.phase.value}')
        if hs.state != hstate0:
            self.event(f'handoff {hstate0} -> {hs.state}' + (f' ({hs.failure})' if hs.failure else ''))
            if hs.state == 'preparing' and hs.t0_s is not None:
                self.event(f'clearance complete: t0 {hs.t0_s - self.t_start:+.3f}, deadline_ns {hs.deadline_ns}')
        # the join: this publication's sample 0 against the previous publication evaluated at this stamp
        stamp = p.last_reference_stamp_ns * 1e-9
        step = (float('nan'),) * 3
        if self.prev_pub is not None:
            ps, pref = self.prev_pub
            w = h.advance_reference_window(pref, elapsed_s=max(0.0, stamp - ps), sample_dt_s=p.dt, output_count=1)
            step = (float(np.linalg.norm(ref.positions[0] - w.positions[0])),
                    float(np.linalg.norm(ref.velocities[0] - w.velocities[0])),
                    float(np.linalg.norm(ref.accelerations[0] - w.accelerations[0])))
        self.prev_pub = (stamp, ref)
        q = p.drone_position if p.drone_position is not None else np.full(3, np.nan)
        qv = p.drone_velocity if p.drone_velocity is not None else np.full(3, np.nan)
        qa = p.drone_acceleration if p.drone_acceleration is not None else np.full(3, np.nan)
        src = getattr(p, 'exit_last_source', 'other')
        if p.phase == P.RETURN_TO_START and p.c1f2_cpp_authority_active:
            src = 'cpp'
        row = [t, p.phase.value, hs.state, hs.sequence, src, p.last_reference_stamp_ns, *ref.positions[0],
               *ref.velocities[0], *ref.accelerations[0], *step, *q, *qv, *qa, *self.ring,
               int(bool(p.c1f2_cpp_authority_active)), hs.waiting_reason, round(tick_ms, 2),
               round(1e3 * self.status_lag, 1), round(1e3 * self.ref_lag, 1)]
        self.ticks.write(','.join(str(x) if isinstance(x, str) else f'{x:.6f}' if isinstance(x, float) else str(x)
                                  for x in row) + '\n')
        self.ticks.flush()
        if self.done_t is None and (p.phase in (P.LANDING, P.LANDED_DISARMED) or hs.state in ('failed', 'landed')):
            self.done_t = t
            self.event(f'done ({p.phase.value}, handoff {hs.state})')
        if (self.done_t is not None and t - self.done_t > 3.0) or t - self.t_start > self.run_s:
            raise SystemExit


def main():
    rclpy.init()
    n = Driver()
    try:
        rclpy.spin(n)
    except SystemExit:
        pass
    finally:
        n.event(f'END exceptions {n.errors}; marks ' + json.dumps({k: (round(v - n.t_start, 3) if isinstance(v, float)
                                                                      else v) for k, v in n.mark.items()
                                                                  if not k.startswith('_')}))
        for f in (n.plog, n.events, n.ticks, n.q4log, n.statuslog, n.reqlog, n.prefixlog, n.bstatus, n.reflog,
                  n.acclog):
            f.close()
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

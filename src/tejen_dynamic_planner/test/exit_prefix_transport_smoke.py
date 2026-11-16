"""C++ exit hand-off, backend half (multi_drone_control 15 Nov): isolated backend-only ROS smoke of the real binary.

Synthetic inputs, no controller or flight: QUAD4 is put exactly on the prefix the "mission" publishes. Run explicitly
after the scoped build, in an unused localhost-only ROS domain. Transport/contract evidence, not a replay or a twin.
"""
import json
import math
import os
from pathlib import Path
import subprocess
import time

import rclpy
from geometry_msgs.msg import Point, PoseStamped
from interfaces.msg import CommittedTrajectory, CubicTrajectoryPiece, MotionCaptureState
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool, String
from trajectory_msgs.msg import MultiDOFJointTrajectory

NS = '/exit_prefix_smoke'
KNOT_S = 0.25
HORIZON_S = 8.0
GOAL = (0.0, 1.55, 2.5)


def ring_following(t, t0):
    s = t - t0
    return (-0.4 + 0.30 * s, 0.6 + 0.20 * math.sin(0.8 * s), 2.9)


class Prefix:
    """A uniform cubic B-spline on the grid t0 + k*KNOT_S, sent as one piece per knot interval (C2 by construction).
    Control points from `free_from` on may be moved (a refresh's new pieces)."""

    def __init__(self, t0, start, horizon_s, shift_from=None, shift=(0., 0., 0.), base=None):
        self.t0, self.start = t0, start
        self.first = int(math.floor((start - t0) / KNOT_S))
        self.last = int(math.ceil((start + horizon_s - t0) / KNOT_S))
        self.end = t0 + self.last * KNOT_S
        self.points = {}
        for i in range(self.first, self.last + 3):
            p = base.points[i] if base is not None and i in base.points else ring_following(t0 + (i - 1) * KNOT_S, t0)
            if shift_from is not None and i >= shift_from:
                p = tuple(a + b for a, b in zip(p, shift))
            self.points[i] = p

    def evaluate(self, t):
        s = min(max(int(math.floor((t - self.t0) / KNOT_S)), self.first), self.last - 1)
        u = (t - (self.t0 + s * KNOT_S)) / KNOT_S
        basis = ((1 - u) ** 3 / 6, (3 * u ** 3 - 6 * u ** 2 + 4) / 6, (-3 * u ** 3 + 3 * u ** 2 + 3 * u + 1) / 6,
                 u ** 3 / 6)
        d1 = (-(1 - u) ** 2 / 2, (9 * u ** 2 - 12 * u) / 6, (-9 * u ** 2 + 6 * u + 3) / 6, u ** 2 / 2)
        d2 = (1 - u, 3 * u - 2, -3 * u + 1, u)
        out = []
        for weights, scale in ((basis, 1.), (d1, 1 / KNOT_S), (d2, 1 / KNOT_S ** 2)):
            out.append(tuple(scale * sum(w * self.points[s + j][axis] for j, w in enumerate(weights))
                             for axis in range(3)))
        return out

    def message(self, sequence, stamp, vehicle_id='drone_0'):
        msg = CommittedTrajectory()
        msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = 'map'
        msg.vehicle_id = vehicle_id
        msg.sequence = sequence
        msg.terminal_hold = False
        for s in range(self.first, self.last):
            piece = CubicTrajectoryPiece()
            piece.valid_from_s = max(self.start, self.t0 + s * KNOT_S)
            piece.valid_until_s = self.t0 + (s + 1) * KNOT_S
            piece.knots = [self.t0 + (s + j - 3) * KNOT_S for j in range(8)]
            piece.control_points = [Point(x=p[0], y=p[1], z=p[2]) for p in
                                    (self.points[s + j] for j in range(4))]
            msg.pieces.append(piece)
        return msg


def backend(feature_on):
    root = Path(__file__).resolve().parents[3]
    topics = dict(state_topic='state', target_topic='target', enable_topic='enable', authority_topic='authority',
                  authority_ack_topic='ack', recovery_request_topic='request', recovery_status_topic='status',
                  shadow_reference_topic='window', recovery_continuation_topic='continuation',
                  diagnostics_topic='diagnostics', marker_topic='markers', object_attached_topic='object_attached')
    params = {k: NS + '/' + v for k, v in topics.items()}
    params.update(reference_recovery_enabled=True, seed_stationary_on_activation=False,
                  static_scene_enabled=False, require_scene_witness=False, fixed_splice_lookahead_s=.30,
                  octopus_max_runtime_s=.10, planning_policy_profile='simulation_cage',
                  require_terminal_hold_for_nominal=False, enable_octopus_useful_deadline=False,
                  ego_tracking_half_x_m=.05, ego_tracking_half_y_m=.05, ego_tracking_half_z_m=.05,
                  payload_collision_enabled=True, require_payload_attached_for_enable=True)
    if feature_on:
        params.update(moving_prefix_topic=NS + '/prefix', exit_max_swing_angle_rad=math.radians(5.),
                      exit_workspace_min='[-1.8,-1.8,0.0]', exit_workspace_max='[1.8,1.8,3.26]',
                      exit_max_search_z_m=3.2, exit_protected_s=1.5)
    args = [str(root / 'install/tejen_dynamic_planner/lib/tejen_dynamic_planner/dynamic_planner_transfer_backend'),
            '--ros-args', '--log-level', 'error', '-r', '__node:=exit_prefix_smoke_backend']
    for key, value in params.items():
        args += ['-p', key + ':=' + (str(value).lower() if isinstance(value, bool) else str(value))]
    return subprocess.Popen(args, stdout=subprocess.DEVNULL)


class Driver:
    def __init__(self, node, process):
        self.node, self.process = node, process
        self.statuses, self.references, self.diagnostics = [], [], []
        node.create_subscription(String, NS + '/status', lambda m: self.statuses.append(json.loads(m.data)), 50)
        node.create_subscription(String, NS + '/diagnostics', lambda m: self.diagnostics.append(m.data), 10)
        node.create_subscription(MultiDOFJointTrajectory, NS + '/window', self.references.append, 50)
        self.state_pub = node.create_publisher(MotionCaptureState, NS + '/state', 10)
        self.target_pub = node.create_publisher(PoseStamped, NS + '/target', 10)
        self.object_pub = node.create_publisher(Bool, NS + '/object_attached', 10)
        self.request_pub = node.create_publisher(String, NS + '/request', 10)
        self.enable_pub = node.create_publisher(Bool, NS + '/enable', 10)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.prefix_pub = node.create_publisher(CommittedTrajectory, NS + '/prefix', qos)
        self.request_id, self.action, self.object_attached = 'a' * 32, None, False
        self.executing = []          # (start, Prefix): QUAD4 follows the newest one it has reached
        self.seedtraj = None         # (sequence, stamp_ns, deadline_ns)

    def now(self):
        return self.node.get_clock().now()

    def publish_prefix(self, prefix, sequence, vehicle_id='drone_0'):
        stamp = self.now()
        self.prefix_pub.publish(prefix.message(sequence, stamp, vehicle_id))
        self.executing.append((prefix.start, prefix))
        return stamp.nanoseconds

    def send(self, command):
        self.request_pub.publish(String(data=self.request_id + ' ' + command))

    def status(self):
        mine = [s for s in self.statuses if s['request_id'] == self.request_id]
        return mine[-1] if mine else {}

    def pump(self, seconds, until=None):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            assert self.process.poll() is None, 'backend exited'
            stamp = self.now()
            t = stamp.nanoseconds * 1e-9
            state = MotionCaptureState()
            state.header.stamp = stamp.to_msg()
            state.header.frame_id = 'map'
            state.pose.orientation.w = 1.
            reached = [p for start, p in self.executing if start <= t]
            if reached:
                (x, y, z), (vx, vy, vz), _ = reached[-1].evaluate(t)
            else:
                (x, y, z), (vx, vy, vz) = (0., 0., 1.), (0., 0., 0.)
            state.pose.position.x, state.pose.position.y, state.pose.position.z = x, y, z
            state.twist.linear.x, state.twist.linear.y, state.twist.linear.z = vx, vy, vz
            self.state_pub.publish(state)
            target = PoseStamped()
            target.header = state.header
            target.pose.orientation.w = 1.
            target.pose.position.x, target.pose.position.y, target.pose.position.z = GOAL
            self.target_pub.publish(target)
            self.object_pub.publish(Bool(data=self.object_attached))
            command = self.action
            if self.action == 'seedtraj':
                command = 'seedtraj exit %d %d %d' % self.seedtraj
            elif self.action == 'grant':
                command = 'grant %d' % stamp.nanoseconds
            if command is not None:
                self.request_pub.publish(String(data=self.request_id + ' ' + command))
            rclpy.spin_once(self.node, timeout_sec=.02)
            if until and until():
                return
        if until:
            raise AssertionError('timeout; status=' + str(self.status()))

    def new_transaction(self, request_id):
        self.request_id, self.action, self.executing = request_id, 'reset', self.executing[-1:]
        self.pump(10., lambda: self.status().get('reason') == 'RECOVERY_WAITING_COMMON_STOPPED_REFERENCE')
        assert self.status()['exit_reject_reason'] == '' and not self.status()['exit_context']

    def open_exit(self, deadline_s=7., horizon_s=HORIZON_S):
        start = self.now().nanoseconds * 1e-9
        prefix = Prefix(start - 0.1, start, horizon_s)
        stamp_ns = self.publish_prefix(prefix, 1)
        self.seedtraj = (1, stamp_ns, stamp_ns + int(deadline_s * 1e9))
        self.action = 'seedtraj'
        return prefix

    def swing_deg(self):
        lines = [d for d in self.diagnostics if 'exit context/swing_deg' in d]
        return float(lines[-1].split('invalidated: ')[1].split(' / ')[1]) if lines else float('nan')


def run_feature_on():
    process = backend(True)
    rclpy.init()
    node = rclpy.create_node('exit_prefix_smoke_driver')
    d = Driver(node, process)
    try:
        d.action = 'reset'
        d.pump(.8)
        # A fresh attached reading refuses the exit context: no new gate, the existing release evidence.
        d.new_transaction('a' * 32)
        d.object_attached = True
        d.open_exit()
        d.pump(5., lambda: d.status().get('exit_reject_reason') == 'object_attached_or_stale')
        assert not d.status()['exit_context'] and not d.status()['prepared']

        # The seedtraj may overtake its prefix: it waits, without a reject reason.
        d.object_attached = False
        d.new_transaction('b' * 32)
        start = d.now().nanoseconds * 1e-9
        prefix1 = Prefix(start - 0.1, start, HORIZON_S)
        stamp_ns = d.now().nanoseconds
        d.seedtraj, d.action = (1, stamp_ns, stamp_ns + int(9e9)), 'seedtraj'
        d.pump(.5)
        assert not d.status()['exit_context'] and d.status()['exit_reject_reason'] == ''
        msg = prefix1.message(1, Time(nanoseconds=stamp_ns, clock_type=d.now().clock_type))
        d.prefix_pub.publish(msg)
        d.executing.append((prefix1.start, prefix1))
        # Moving at ~0.3 m/s, the stationary gate (0.10 m/s) would refuse; relative readiness activates.
        d.pump(10., lambda: d.status().get('exit_context') and d.status().get('prepared'))
        s = d.status()
        assert s['seed_kind'] == 'prefix' and s['prefix_sequence'] == 1 and s['candidates_accepted'] >= 1
        assert s['deadline_ns'] == stamp_ns + int(9e9) and abs(s['prefix_coverage_end_ns'] * 1e-9 - prefix1.end) < 1e-6
        d.pump(10., lambda: d.swing_deg() == d.swing_deg() and abs(d.swing_deg() - 5.) < 1e-6)

        # Refresh, protected window identical (verbatim pieces past t_R + 1.5 s): no epoch bump.
        epoch, invalidated = s['epoch'], s['candidates_invalidated']
        t_r = d.now().nanoseconds * 1e-9 + 0.01
        free = int(math.floor((t_r + 1.5 - prefix1.t0) / KNOT_S)) + 4
        prefix2 = Prefix(prefix1.t0, t_r, HORIZON_S, shift_from=free, shift=(0., .2, 0.), base=prefix1)
        stamp2 = d.publish_prefix(prefix2, 2)
        d.seedtraj = (2, stamp2, s['deadline_ns'])
        d.pump(10., lambda: d.status().get('prefix_sequence') == 2 and d.status().get('prepared'))
        assert d.status()['epoch'] == epoch, 'identical protected window bumped the epoch'
        assert d.status()['deadline_ns'] == s['deadline_ns']

        # Refresh, protected window differs (inside it): candidates invalidated (epoch bump) before rebinding.
        s = d.status()
        epoch, invalidated = s['epoch'], s['candidates_invalidated']
        t_r = d.now().nanoseconds * 1e-9 + 0.01
        inside = int(math.floor((t_r - prefix1.t0) / KNOT_S)) + 6
        prefix3 = Prefix(prefix1.t0, t_r, HORIZON_S, shift_from=inside, shift=(0., 0., -.05), base=prefix2)
        stamp3 = d.publish_prefix(prefix3, 3)
        d.seedtraj = (3, stamp3, s['deadline_ns'])
        d.pump(10., lambda: d.status().get('prefix_sequence') == 3 and d.status().get('epoch', 0) > epoch)
        assert d.status()['candidates_invalidated'] > invalidated
        d.pump(10., lambda: d.status().get('prepared'))

        # A refresh landing while the post-grant solve is in flight (sent in the grant's tick). Differing window:
        # that solve is invalidated before rebinding and the pending grant reverted.
        s = d.status()
        epoch, invalidated, seen = s['epoch'], s['candidates_invalidated'], len(d.statuses)
        t_r = d.now().nanoseconds * 1e-9 + 0.01
        inside = int(math.floor((t_r - prefix1.t0) / KNOT_S)) + 6
        prefix4 = Prefix(prefix1.t0, t_r, HORIZON_S, shift_from=inside, shift=(0., 0., .03), base=prefix3)
        stamp4 = d.publish_prefix(prefix4, 4)
        d.send('grant %d' % d.now().nanoseconds)
        d.seedtraj = (4, stamp4, s['deadline_ns'])
        d.send('seedtraj exit %d %d %d' % d.seedtraj)
        d.action = 'seedtraj'
        d.pump(10., lambda: d.status().get('prefix_sequence') == 4 and d.status().get('epoch', 0) > epoch)
        reasons = [x['reason'] for x in d.statuses[seen:] if x['request_id'] == d.request_id]
        assert 'AUTHORITY_REQUESTED_WAITING_FRESH_COMMIT' in reasons, reasons
        assert not d.status()['granted'] and d.status()['candidates_invalidated'] == invalidated + 1
        d.pump(10., lambda: d.status().get('prepared'))

        # Identical window: the in-flight post-grant solve stays valid, installs, and C++ takes authority.
        s = d.status()
        epoch, invalidated = s['epoch'], s['candidates_invalidated']
        t_r = d.now().nanoseconds * 1e-9 + 0.01
        free = int(math.floor((t_r + 1.5 - prefix1.t0) / KNOT_S)) + 4
        prefix5 = Prefix(prefix1.t0, t_r, HORIZON_S, shift_from=free, shift=(0., -.2, 0.), base=prefix4)
        stamp5 = d.publish_prefix(prefix5, 5)
        d.references.clear()
        d.send('grant %d' % d.now().nanoseconds)
        d.seedtraj = (5, stamp5, s['deadline_ns'])
        d.send('seedtraj exit %d %d %d' % d.seedtraj)
        d.action = 'grant'
        d.pump(10., lambda: d.status().get('granted') and d.status().get('reference_stamp_ns'))
        s = d.status()
        assert s['prefix_sequence'] == 5, 'the refresh did not land while the solve was in flight'
        assert s['epoch'] == epoch and s['candidates_invalidated'] == invalidated, 'in-flight candidate dropped'
        splice_s = s['splice_time_ns'] * 1e-9
        wanted = s['reference_stamp_ns']
        d.pump(.3)
        publication = next(r for r in reversed(d.references)
                           if r.header.stamp.sec * 1_000_000_000 + r.header.stamp.nanosec == wanted)
        sampled = wanted * 1e-9
        assert sampled < splice_s, 'granted window starts after the join'
        compared = 0
        for k, point in enumerate(publication.points):
            t = sampled + k / 30.
            if t >= splice_s - 1e-6:
                break
            p, v, a = prefix5.evaluate(t)
            got = ((point.transforms[0].translation.x, point.transforms[0].translation.y,
                    point.transforms[0].translation.z),
                   (point.velocities[0].linear.x, point.velocities[0].linear.y, point.velocities[0].linear.z),
                   (point.accelerations[0].linear.x, point.accelerations[0].linear.y,
                    point.accelerations[0].linear.z))
            for want, have, tol in ((p, got[0], 1e-6), (v, got[1], 1e-6), (a, got[2], 1e-5)):
                assert max(abs(x - y) for x, y in zip(want, have)) <= tol, (k, want, have)
            compared += 1
        assert compared >= 1, 'no executed sample before the joining time'
        print('  granted: %d samples before the join equal the prefix; splice %.3f s after the window start'
              % (compared, splice_s - sampled))

        # Cleanup paths: each restores the loaded geometry (15 deg default swing) and ends the context.
        d.action = 'cancel'
        d.pump(5., lambda: d.status().get('reason') == 'RECOVERY_CANCELLED' and not d.status().get('exit_context'))
        assert not d.status()['granted']
        d.pump(10., lambda: abs(d.swing_deg() - 15.) < 1e-6)

        # A grant 0.25 s before the deadline: its solve splices (now + 0.30 s) after the deadline, so it is dropped
        # and counted late; then the deadline ends the context without a grant.
        d.new_transaction('c' * 32)
        d.open_exit(deadline_s=2.5)
        deadline_s = d.seedtraj[2] * 1e-9
        d.pump(10., lambda: d.status().get('exit_context') and d.status().get('prepared'))
        d.pump(10., lambda: d.now().nanoseconds * 1e-9 >= deadline_s - 0.25)
        d.action = 'grant'
        d.pump(10., lambda: d.status().get('exit_reject_reason') == 'deadline_passed')
        s = d.status()
        assert not s['exit_context'] and not s['granted'] and s['reason'] == 'EXIT_DEADLINE_PASSED'
        assert s['candidates_late'] >= 1, s

        d.new_transaction('d' * 32)
        d.open_exit()
        d.pump(10., lambda: d.status().get('exit_context'))
        d.enable_pub.publish(Bool(data=False))
        d.pump(5., lambda: d.status().get('reason') == 'DISABLED_BY_COMMAND' and not d.status().get('exit_context'))

        d.new_transaction('e' * 32)
        d.open_exit()
        d.pump(10., lambda: d.status().get('exit_context'))
        d.new_transaction('f' * 32)  # asserts exit_context false for the new id

        # Failed requirements: no context, a reason.
        start = d.now().nanoseconds * 1e-9
        bad = Prefix(start - 0.1, start, HORIZON_S)
        stamp_ns = d.publish_prefix(bad, 1, vehicle_id='drone_9')
        d.seedtraj, d.action = (1, stamp_ns, stamp_ns + int(7e9)), 'seedtraj'
        d.pump(5., lambda: d.status().get('exit_reject_reason') == 'prefix_invalid')
        d.new_transaction('1' * 32)
        d.open_exit(horizon_s=1.5)
        d.pump(5., lambda: d.status().get('exit_reject_reason') == 'prefix_coverage_short')
        assert not d.status()['exit_context']
        d.pump(10., lambda: abs(d.swing_deg() - 15.) < 1e-6)
        print('PASS: exit context opens on validated evidence, prefix seed, relative readiness, refresh keep/'
              'invalidate (also in flight), grant joins on the prefix, late splice dropped, cancel/deadline/'
              'enable=false/reset restore')
    finally:
        process.terminate()
        try:
            process.wait(timeout=5.)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
        node.destroy_node()
        rclpy.shutdown()


def run_feature_off():
    process = backend(False)
    rclpy.init()
    node = rclpy.create_node('exit_prefix_smoke_driver_off')
    d = Driver(node, process)
    try:
        d.action = 'reset'
        d.pump(.8)
        d.request_id, d.action = '2' * 32, 'reset'
        d.pump(10., lambda: d.statuses and d.statuses[-1]['request_id'] == d.request_id)
        d.action = None
        d.pump(.5)
        assert all('seed_kind' not in s and 'exit_context' not in s for s in d.statuses)
        count = len(d.statuses)
        start = d.now().nanoseconds * 1e-9
        d.seedtraj = (1, d.now().nanoseconds, d.now().nanoseconds + int(7e9))
        d.executing.append((start, Prefix(start - 0.1, start, HORIZON_S)))
        d.action = 'seedtraj'
        d.pump(1.)
        assert len(d.statuses) == count, 'a seedtraj request changed anything with the feature off'
        print('PASS (feature off): seedtraj ignored like an unparsed request; no new status keys')
    finally:
        process.terminate()
        try:
            process.wait(timeout=5.)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    if os.environ.get('ROS_LOCALHOST_ONLY') != '1' or not os.environ.get('ROS_DOMAIN_ID'):
        raise SystemExit('Use a dedicated ROS_DOMAIN_ID with ROS_LOCALHOST_ONLY=1; backend only.')
    run_feature_off()
    run_feature_on()

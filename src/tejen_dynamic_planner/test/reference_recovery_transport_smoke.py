"""Isolated backend-only ROS smoke; synthetic inputs, no controller or flight.

Run explicitly after the scoped build, in an unused localhost-only ROS domain.
This is transport/authority evidence, not a simulator or physical tracking test.
"""
import json
import os
from pathlib import Path
import subprocess
import time

import rclpy
from geometry_msgs.msg import Point, PoseStamped
from interfaces.msg import CommittedTrajectory, CubicTrajectoryPiece, MotionCaptureState
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import Bool, String
from trajectory_msgs.msg import MultiDOFJointTrajectory


def main(reference_recovery_enabled=False, cooperative_tracking_safety_enabled=True):
    root = Path(__file__).resolve().parents[3]
    ns = '/reference_recovery_smoke'
    topics = dict(state_topic='state', target_topic='target', enable_topic='enable',
                  authority_topic='authority', authority_ack_topic='ack',
                  recovery_request_topic='request', recovery_status_topic='status',
                  shadow_reference_topic='window', recovery_continuation_topic='continuation',
                  diagnostics_topic='diagnostics', marker_topic='markers')
    params = {k: ns+'/'+v for k, v in topics.items()}
    params.update(reference_recovery_enabled=reference_recovery_enabled, seed_stationary_on_activation=True,
                  static_scene_enabled=False, require_scene_witness=False,
                  fixed_splice_lookahead_s=.10, octopus_max_runtime_s=.05,
                  planning_policy_profile='simulation_cage',
                  require_terminal_hold_for_nominal=False,
                  ego_tracking_half_x_m=.05, ego_tracking_half_y_m=.05, ego_tracking_half_z_m=.05,
                  enable_octopus_useful_deadline=False, cooperative_scene_enabled=True,
                  cooperative_prediction_mode='shared_trajectory',
                  cooperative_tracking_safety_enabled=cooperative_tracking_safety_enabled,
                  cooperative_tracking_half_x_m=.05, cooperative_tracking_half_y_m=.05,
                  cooperative_tracking_half_z_m=.05,
                  cooperative_drone_state_topics=[ns+'/peer_state'],
                  cooperative_committed_trajectory_topics=[ns+'/peer_future'],
                  cooperative_vehicle_ids=['peer'])
    args = [str(root/'install/tejen_dynamic_planner/lib/tejen_dynamic_planner/dynamic_planner_transfer_backend'),
            '--ros-args', '--log-level', 'error', '-r', '__node:=reference_recovery_smoke_backend']
    for key, value in params.items():
        encoded = str(value).lower() if isinstance(value, bool) else str(value)
        args += ['-p', key+':='+encoded]
    rclpy.init()
    node = rclpy.create_node('reference_recovery_smoke_driver')
    statuses, references, diagnostics = [], [], []
    node.create_subscription(String, ns+'/status', lambda m: statuses.append(json.loads(m.data)), 10)
    node.create_subscription(String, ns+'/diagnostics', lambda m: diagnostics.append(m.data), 10)
    reference_topic = '/continuation' if reference_recovery_enabled else '/window'
    node.create_subscription(MultiDOFJointTrajectory, ns+reference_topic, references.append, 10)
    state_pub = node.create_publisher(MotionCaptureState, ns+'/state', 10)
    peer_pub = node.create_publisher(MotionCaptureState, ns+'/peer_state', 10)
    target_pub = node.create_publisher(PoseStamped, ns+'/target', 10)
    request_pub = node.create_publisher(String, ns+'/request', 10)
    enable_pub = node.create_publisher(Bool, ns+'/enable', 10)
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    future_pub = node.create_publisher(CommittedTrajectory, ns+'/peer_future', qos)
    request_id = 'a'*32
    process = subprocess.Popen(args, stdout=subprocess.DEVNULL)
    peer_mode, action = 'missing', 'reset'
    sequence = 0
    stationary_future_sent = False
    seed_position = (0., 0., 1.)

    def granted_publication():
        if not statuses or not statuses[-1]['granted']:
            return None
        wanted = statuses[-1]['reference_stamp_ns']
        return next((ref for ref in reversed(references)
                     if ref.header.stamp.sec*1_000_000_000 + ref.header.stamp.nanosec == wanted), None)

    def pump(seconds, until=None):
        nonlocal sequence, stationary_future_sent
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            assert process.poll() is None, 'backend exited during recovery check'
            stamp = node.get_clock().now()
            now = stamp.nanoseconds*1e-9
            state = MotionCaptureState()
            state.header.stamp = stamp.to_msg(); state.header.frame_id = 'map'
            state.pose.orientation.w = 1.; state.pose.position.x = .01; state.pose.position.z = .954
            state_pub.publish(state)
            target = PoseStamped(); target.header = state.header
            target.pose.orientation.w = 1.; target.pose.position.x = .30; target.pose.position.z = 1.
            target_pub.publish(target)
            if peer_mode != 'missing':
                state.pose.position.x = 2.08 if peer_mode == 'mismatch' else 2.
                peer_pub.publish(state)
                if peer_mode == 'gap' or not stationary_future_sent:
                    sequence += 1
                    future = CommittedTrajectory(); future.header = state.header
                    future.vehicle_id = 'peer'; future.sequence = sequence
                    future.terminal_hold = peer_mode == 'valid'
                    piece = CubicTrajectoryPiece()
                    piece.valid_from_s = now-.01
                    piece.valid_until_s = now+(.04 if peer_mode == 'gap' else 10.)
                    piece.knots = [piece.valid_from_s]*4 + [piece.valid_until_s]*4
                    piece.control_points = [Point(x=2.+(.01*k/3 if peer_mode == 'gap' else 0.),
                                                  z=1.) for k in range(4)]
                    future.pieces = [piece]; future_pub.publish(future)
                    stationary_future_sent = peer_mode == 'valid'
            command = action
            if action == 'seed': command = 'seed '+' '.join(map(str, seed_position))+' '+str(stamp.nanoseconds)
            if action == 'grant': command = 'grant '+str(stamp.nanoseconds)
            if command is not None:
                request_pub.publish(String(data=request_id+' '+command))
            rclpy.spin_once(node, timeout_sec=.02)
            if until and until(): return
        if until: raise AssertionError('timeout; status='+str(statuses[-1:] ))

    try:
        pump(.6)
        action = 'seed'
        pump(.5)
        assert statuses and not any(s['granted'] for s in statuses)
        assert 'commitment_missing' in statuses[-1]['blockers']
        peer_mode = 'gap'
        pump(.5)
        assert not any(s['granted'] for s in statuses), 'future gap granted authority'
        peer_mode = 'valid'
        pump(10., lambda: bool(statuses and statuses[-1]['prepared']))
        assert references and all(not s['granted'] for s in statuses)
        assert abs(references[-1].points[0].transforms[0].translation.x) < 1e-6, 'seed uses measured origin'
        references.clear()
        action = 'grant'
        pump(10., lambda: granted_publication() is not None)
        assert statuses[-1]['request_id'] == request_id
        publication = granted_publication()
        assert publication is not None and len(publication.points) >= 61
        source_s = publication.header.stamp.sec + publication.header.stamp.nanosec*1e-9
        assert 0. <= node.get_clock().now().nanoseconds*1e-9-source_s <= .20
        epoch = statuses[-1]['epoch']
        # Same 80 mm peer error / 50 mm inflation in both policies. Diagnostic
        # mode must keep the epoch/grant while still reporting ratio > 1.
        action, peer_mode = None, 'mismatch'
        if not cooperative_tracking_safety_enabled:
            pump(1.3)
            assert statuses[-1]['granted'] and statuses[-1]['epoch'] == epoch, \
                'diagnostic-only peer mismatch withdrew authority'
            assert not statuses[-1]['blockers']
            ratios = [float(text.split('max_track_ratio=')[1].split()[0])
                      for text in diagnostics if 'max_track_ratio=' in text]
            assert ratios and max(ratios) > 1.5, 'peer tracking diagnostic was lost'
            # Actual missing/stale peer measurements still withdraw authority.
            peer_mode = 'missing'
            blocker = 'peer:state_stale'
        else:
            blocker = 'peer:tracking_mismatch'
        pump(3., lambda: bool(statuses and not statuses[-1]['granted'] and
                              blocker in statuses[-1]['blockers']))
        assert statuses[-1]['epoch'] > epoch
        peer_mode = 'valid'
        pump(.35)
        assert not statuses[-1]['granted'], 'peer validity silently restored old authority'
        request_id, action, seed_position = 'b'*32, 'reset', (.02, 0., 1.)
        pump(.3)
        action = 'seed'
        pump(10., lambda: bool(statuses and statuses[-1]['prepared'] and
                               statuses[-1]['request_id'] == request_id))
        references.clear()
        action = 'grant'
        pump(10., lambda: granted_publication() is not None)
        assert statuses[-1]['request_id'] == request_id and statuses[-1]['epoch'] > epoch
        assert abs(granted_publication().points[0].transforms[0].translation.x-.02) < 1e-6, \
            'recovery reused the stale initial seed'
        epoch = statuses[-1]['epoch']
        action = 'cancel'
        pump(.3)
        action = 'grant'  # simulate a queued obsolete grant and bare enable after LAND
        enable_pub.publish(Bool(data=True))
        pump(.5)
        assert not statuses[-1]['granted'] and statuses[-1]['epoch'] >= epoch
        assert statuses[-1]['reason'] == 'RECOVERY_CANCELLED'
        print('PASS: missing/gapped/stale peer blocks; tracking enforcement='+
              str(cooperative_tracking_safety_enabled)+'; fresh replacement seed/grant; LAND cancels')
    finally:
        process.terminate()
        try: process.wait(timeout=5.)
        except subprocess.TimeoutExpired:
            process.kill(); process.wait()
        node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    if os.environ.get('ROS_LOCALHOST_ONLY') != '1' or not os.environ.get('ROS_DOMAIN_ID'):
        raise SystemExit('Use a dedicated ROS_DOMAIN_ID with ROS_LOCALHOST_ONLY=1; backend only.')
    main()

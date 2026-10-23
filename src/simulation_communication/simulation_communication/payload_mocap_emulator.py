"""
payload_mocap_emulator.py
-------------------------
Reads Gazebo poses for a drone (nested model) and the cable-suspended
payload, then publishes MotionCaptureState for each.

Parameters (ROS):
  drone_id      int   0
  drone_name    str   x3_drone0
  parent_model  str   lift_system
  publish_payload bool  True (only drone_id==0 need publish; others False)

Topics produced:
  /drone_{drone_id}/motion_capture_state   — for the drone_controller
  /payload/motion_capture_state            — for the planner (drone 0 only)
  /drone_{drone_id}/magnet_tip_state       — tether_pose_topic set: the rig's magnet marker, the payload end of
                                             this drone's rod (card 2026-10-21_tip_detect)
"""

import collections
import os

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose, Twist, PoseStamped
from std_msgs.msg import Float64, Float64MultiArray
from interfaces.msg import MotionCaptureState
from tf_transformations import quaternion_multiply, quaternion_inverse, quaternion_matrix
from drone_communication.velocity_filter import VelocityFilter


class PayloadMocapEmulator(Node):
    def __init__(self):
        super().__init__('payload_mocap_emulator')

        self.declare_parameter('drone_id', 0)
        self.declare_parameter('drone_name', 'x3_drone0')
        self.declare_parameter('parent_model', 'lift_system')
        self.declare_parameter('publish_payload', True)
        # Overrides for a STANDALONE (non-nested) drone model, e.g. the free approach drone:
        #   pose_topic  non-empty -> use this pose topic instead of the nested-path default
        #               (a standalone model publishes on /model/{drone_name}/pose).
        #   pose_index  which PoseArray entry is the drone body (-1 = last, the nested-drone
        #               default). A model with extra links (a magnet arm) may need a fixed
        #               index -- inspect `gz topic -e -t <pose_topic>` to pick base_link.
        self.declare_parameter('pose_topic', '')
        self.declare_parameter('pose_index', -1)
        # Publish ceiling (Hz) per body, 0 = every Gazebo pose (~300-500 Hz). The trackers
        # run at 50 Hz and every message costs a Python callback in each subscriber; the
        # real mocap node throttles the same way (MOCAP_MAX_PUBLISH_HZ).
        self.declare_parameter('max_publish_hz', 0.0)
        # > 0: each sample is published this long after Gazebo produced it, standing in for the
        # rig's mocap, relay and network latency (default from SIM_MOCAP_DELAY_S; 0 = off)
        self.declare_parameter('mocap_delay_s', float(os.environ.get('SIM_MOCAP_DELAY_S', '0.0')))
        # the rig mocap node's velocity filter (card 2026-10-14_tracker_timing): 'none' keeps the plain difference,
        # 'ema' is the rig's alpha 0.2 (its ~45 ms lag), 'slope' the least-squares slope. The twin flies the rig's
        # filter, so 'ema' by default (Wesley 22 Oct; 'none' before, which hid the rig's pair swing)
        self.declare_parameter('velocity_filter', os.environ.get('SIM_MOCAP_VEL_FILTER', 'ema'))
        # the ring's own filter when the carriers are filtered: the rig's ema, or slope (card 2026-10-22_velocity_filter,
        # the ring arm; default ema)
        self.declare_parameter('ring_velocity_filter', os.environ.get('SIM_MOCAP_VEL_RING', 'ema'))
        # sim time (s) from which the delay and the filter apply (default from SIM_MOCAP_DELAY_FROM_S; 0 = from the
        # start). The drones flip in the takeoff creep with the delay on (R1100, R2117): starting it after the lift
        # lets the rig-latency twin carry.
        self.declare_parameter('mocap_delay_from_s', float(os.environ.get('SIM_MOCAP_DELAY_FROM_S', '0.0')))

        drone_id = self.get_parameter('drone_id').value
        drone_name = self.get_parameter('drone_name').value
        parent = self.get_parameter('parent_model').value
        publish_payload = self.get_parameter('publish_payload').value
        self._pose_index = int(self.get_parameter('pose_index').value)
        hz = float(self.get_parameter('max_publish_hz').value)
        self._min_dt = (1.0 / hz) if hz > 0 else 0.0
        self._drone_last_pub = None
        self._payload_last_pub = None
        self._delay = float(self.get_parameter('mocap_delay_s').value)
        self._delay_from = float(self.get_parameter('mocap_delay_from_s').value)
        vmode = str(self.get_parameter('velocity_filter').value)
        self._vfilt = None
        if vmode != 'none':
            # as the rig: the chosen filter on the listed carriers, the rig's ema on the ring and QUAD4
            carriers = [int(v) for v in os.environ.get('MDC_MOCAP_VEL_DRONES', '0,1,2').split(',') if v.strip()]
            dmode = vmode if (vmode == 'ema' or int(drone_id) in carriers) else 'ema'
            rmode = str(self.get_parameter('ring_velocity_filter').value)
            self._vfilt = {'drone': VelocityFilter(dmode), 'payload': VelocityFilter(rmode)}
            self.get_logger().warn(f'[MOCAP{drone_id}] velocity filter {dmode} (ring {rmode})')
        else:
            self.get_logger().warn(f'[MOCAP{drone_id}] velocity filter none (plain difference, ring too)')
        self._queues = {}
        if self._delay > 0.0:
            self.get_logger().warn(f'[MOCAP{drone_id}] mocap delay {self._delay:.3f} s from t {self._delay_from:.1f} s')

        pose_topic_override = str(self.get_parameter('pose_topic').value)
        drone_pose_topic = (pose_topic_override if pose_topic_override
                            else f'/model/{parent}/model/{drone_name}/pose')
        payload_pose_topic = f'/model/{parent}/model/payload/pose'

        self.get_logger().info(
            f'[MOCAP{drone_id}] drone={drone_pose_topic}  '
            f'payload={payload_pose_topic if publish_payload else "(skipped)"}')

        # Drone state
        self._drone_last_pos = None
        self._drone_last_ori = None
        self._drone_last_time = None
        self._drone_pub = self.create_publisher(
            MotionCaptureState, f'/drone_{drone_id}/motion_capture_state', 5)
        self.create_subscription(PoseArray, drone_pose_topic, self._drone_cb, 10)
        # test fault: the runner's MOCAP_LOSS silences this drone's state for data s (sim time), as a
        # lost marker set does on the rig (card 2026-10-09_newcomer_fault_scope)
        self._dropout_until = None
        self.create_subscription(Float64, f'/drone_{drone_id}/mocap_dropout', self._dropout_cb, 5)
        # test fault: the runner's MOCAP_TILT [deg, s, ramp_s] rotates this drone's published attitude about its body
        # x, in one step (a mis-solve) or over ramp_s (a body tipping), then holds it s; the tracker and the partner's
        # MPC see it (card 2026-10-11_quad4_tilt_cut)
        self._tilt_until = None
        self._tilt = None
        self.create_subscription(Float64MultiArray, f'/drone_{drone_id}/mocap_tilt', self._tilt_cb, 5)

        # Payload state (published by the node whose drone_id is 0)
        self._payload_last_pos = None
        self._payload_last_ori = None
        self._payload_last_time = None
        # the magnet marker the rig has on every carrier (Motive bodies 21-24): the payload end of this drone's rod,
        # from the rod model's pose (the world publishes it for the chain's worlds); off with tether_pose_topic ''
        tether_topic = str(self.declare_parameter('tether_pose_topic', '').value)
        # the world's rods (1 m since 22 Oct)
        self._tether_half = 0.5 * float(self.declare_parameter('tether_len', 1.0).value)
        self._tip_last_pub = None
        self._tip_noise = float(self.declare_parameter('tether_tip_noise_m', 0.0005).value)   # Motive marker jitter
        self._tip_rng = np.random.default_rng(int(drone_id))
        if tether_topic:
            self._tip_pub = self.create_publisher(MotionCaptureState, f'/drone_{drone_id}/magnet_tip_state', 5)
            self.create_subscription(PoseArray, tether_topic, self._tether_cb, 10)
        if publish_payload:
            self._payload_pub = self.create_publisher(
                MotionCaptureState, '/payload/motion_capture_state', 5)
            self.create_subscription(
                PoseArray, payload_pose_topic, self._payload_cb, 10)
            # test fault: the runner's MOCAP_TILT -1 rotates the RING's published attitude about its body y (the T's
            # 3-9 axis), as a Motive mis-solve would (card 2026-10-21_tip_detect rev 3)
            self._ring_tilt_until = None
            self._ring_tilt = None
            self.create_subscription(Float64MultiArray, '/payload/mocap_tilt', self._ring_tilt_cb, 5)
        else:
            self._payload_pub = None

    # ------------------------------------------------------------------
    @staticmethod
    def _normalize_quat(x, y, z, w):
        if w < 0:
            return -x, -y, -z, -w
        return x, y, z, w

    def _compute_mcs(self, pos, ori, last_pos, last_ori, last_time, body=None):
        """Return (MotionCaptureState, pos, ori, time) or (None,...) on first call."""
        now = self.get_clock().now().to_msg()
        if last_pos is None:
            return None, pos, ori, now

        dt = (now.sec + now.nanosec * 1e-9) - (last_time.sec + last_time.nanosec * 1e-9)
        if dt <= 0:
            return None, pos, ori, now

        # Linear velocity (world frame)
        lin_vel = np.array([
            (pos.x - last_pos.x) / dt,
            (pos.y - last_pos.y) / dt,
            (pos.z - last_pos.z) / dt,
        ])
        if self._vfilt is not None and body in self._vfilt:
            v = self._vfilt[body].update(now.sec + now.nanosec * 1e-9, (pos.x, pos.y, pos.z))
            lin_vel = lin_vel if v is None or not self._timing_on() else v

        # Angular velocity (body frame)
        q1 = [last_ori.x, last_ori.y, last_ori.z, last_ori.w]
        q2 = [ori.x, ori.y, ori.z, ori.w]
        q_rel = quaternion_multiply(q2, quaternion_inverse(q1))
        ang_vel_world = 2 * np.array([q_rel[0], q_rel[1], q_rel[2]]) / dt
        R = quaternion_matrix(q1)[:3, :3]
        ang_vel_body = R.T @ ang_vel_world

        mcs = MotionCaptureState()
        mcs.header.stamp = now
        mcs.header.frame_id = 'world'
        mcs.pose = Pose()
        mcs.pose.position.x = float(pos.x)
        mcs.pose.position.y = float(pos.y)
        mcs.pose.position.z = float(pos.z)
        mcs.pose.orientation.x = float(ori.x)
        mcs.pose.orientation.y = float(ori.y)
        mcs.pose.orientation.z = float(ori.z)
        mcs.pose.orientation.w = float(ori.w)
        mcs.twist = Twist()
        mcs.twist.linear.x = float(lin_vel[0])
        mcs.twist.linear.y = float(lin_vel[1])
        mcs.twist.linear.z = float(lin_vel[2])
        mcs.twist.angular.x = float(ang_vel_body[0])
        mcs.twist.angular.y = float(ang_vel_body[1])
        mcs.twist.angular.z = float(ang_vel_body[2])
        return mcs, pos, ori, now

    # ------------------------------------------------------------------
    def _due(self, last_pub):
        """(due, now_s): rate ceiling on the node clock (sim time)."""
        n = self.get_clock().now().to_msg()
        now = n.sec + n.nanosec * 1e-9
        if self._min_dt and last_pub is not None and 0.0 <= now - last_pub < self._min_dt:
            return False, last_pub
        return True, now

    def _dropout_cb(self, msg):
        now = self.get_clock().now().nanoseconds * 1e-9
        self._dropout_until = now + max(0.0, float(msg.data))
        self.get_logger().warn(f'[mocap] state silenced for {float(msg.data):.1f} s (MOCAP_LOSS test fault)')

    def _tilt_cb(self, msg):
        deg, secs, ramp = (float(v) for v in (list(msg.data) + [0.0, 0.0, 0.0])[:3])
        t0 = self.get_clock().now().nanoseconds * 1e-9
        self._tilt = (t0, deg, max(0.0, ramp))
        self._tilt_until = t0 + max(0.0, ramp) + max(0.0, secs)
        self.get_logger().warn(f'[mocap] attitude tilted {deg:.0f} deg over {ramp:.2f} s, held {secs:.2f} s '
                               f'(MOCAP_TILT test fault)')

    def _ring_tilt_cb(self, msg):
        deg, secs, ramp = (float(v) for v in (list(msg.data) + [0.0, 0.0, 0.0])[:3])
        t0 = self.get_clock().now().nanoseconds * 1e-9
        self._ring_tilt = (t0, deg, max(0.0, ramp))
        self._ring_tilt_until = t0 + max(0.0, ramp) + max(0.0, secs)
        self.get_logger().warn(f'[mocap] ring attitude tilted {deg:.0f} deg about its y over {ramp:.2f} s, held '
                               f'{secs:.2f} s (MOCAP_TILT -1 test fault)')

    @staticmethod
    def _axis_q(tilt, now, axis):
        """The injected rotation about body axis (0 x, 1 y) at now, (x, y, z, w)."""
        t0, deg, ramp = tilt
        h = 0.5 * np.radians(deg * (min(1.0, (now - t0) / ramp) if ramp > 0.0 else 1.0))
        q = [0.0, 0.0, 0.0, np.cos(h)]
        q[axis] = np.sin(h)
        return q

    def _tilt_q(self, now):
        """The injected rotation about body x at now, (x, y, z, w)."""
        return PayloadMocapEmulator._axis_q(self._tilt, now, 0)

    def _drone_cb(self, msg: PoseArray):
        if not msg.poses:
            return
        if self._dropout_until is not None and \
                self.get_clock().now().nanoseconds * 1e-9 < self._dropout_until:
            return
        due, self._drone_last_pub = self._due(self._drone_last_pub)
        if not due:
            return
        idx = self._pose_index if -len(msg.poses) <= self._pose_index < len(msg.poses) else -1
        pos = msg.poses[idx].position
        ori = msg.poses[idx].orientation
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._tilt_until is not None and now < self._tilt_until:
            ori.x, ori.y, ori.z, ori.w = (float(v) for v in quaternion_multiply(
                [ori.x, ori.y, ori.z, ori.w], self._tilt_q(now)))
        ori.x, ori.y, ori.z, ori.w = self._normalize_quat(ori.x, ori.y, ori.z, ori.w)

        mcs, self._drone_last_pos, self._drone_last_ori, self._drone_last_time = \
            self._compute_mcs(pos, ori,
                              self._drone_last_pos, self._drone_last_ori, self._drone_last_time, body='drone')
        if mcs is not None:
            self._publish(self._drone_pub, mcs)

    @staticmethod
    def rod_world(poses):
        """The rod link's pose in lift_system (world-anchored), (xyz, xyzw): the tether's PosePublisher sends [the rod
        in its model, the model in lift_system] (generate_rigid_world._pose_publisher), so model x rod; a lone pose
        is taken as is."""
        def xyz_q(p):
            return (np.array([p.position.x, p.position.y, p.position.z], float),
                    np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w], float))
        if len(poses) < 2:
            return xyz_q(poses[-1])
        (lp, lq), (mp, mq) = xyz_q(poses[0]), xyz_q(poses[-1])
        return mp + quaternion_matrix(mq)[:3, :3] @ lp, np.asarray(quaternion_multiply(mq, lq), float)

    @staticmethod
    def rod_end(pos, ori_xyzw, half):
        """The payload end of a rod whose link frame sits at its middle with z along it (generate_rigid_world: the
        magnet is at -L/2 on the rod's z)."""
        R = quaternion_matrix(ori_xyzw)[:3, :3]
        return np.asarray(pos, float) + R @ np.array([0.0, 0.0, -half])

    def _tether_cb(self, msg: PoseArray):
        if not msg.poses:
            return
        due, self._tip_last_pub = self._due(self._tip_last_pub)
        if not due:
            return
        pos, ori = self.rod_world(msg.poses)
        tip = self.rod_end(pos, ori, self._tether_half) + self._tip_rng.normal(0.0, self._tip_noise, 3)
        mcs = MotionCaptureState()
        mcs.header.stamp = self.get_clock().now().to_msg()
        mcs.header.frame_id = 'map'
        mcs.pose.position.x, mcs.pose.position.y, mcs.pose.position.z = (float(v) for v in tip)
        (mcs.pose.orientation.x, mcs.pose.orientation.y, mcs.pose.orientation.z,
         mcs.pose.orientation.w) = (float(v) for v in ori)
        self._publish(self._tip_pub, mcs)

    def _payload_cb(self, msg: PoseArray):
        if not msg.poses or self._payload_pub is None:
            return
        due, self._payload_last_pub = self._due(self._payload_last_pub)
        if not due:
            return
        # payload sub-model: index 0 is the model pose, index 1 is the 'body' link
        idx = 1 if len(msg.poses) > 1 else 0
        pos = msg.poses[idx].position
        ori = msg.poses[idx].orientation
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._ring_tilt_until is not None and now < self._ring_tilt_until:
            ori.x, ori.y, ori.z, ori.w = (float(v) for v in quaternion_multiply(
                [ori.x, ori.y, ori.z, ori.w], PayloadMocapEmulator._axis_q(self._ring_tilt, now, 1)))
        ori.x, ori.y, ori.z, ori.w = self._normalize_quat(ori.x, ori.y, ori.z, ori.w)

        mcs, self._payload_last_pos, self._payload_last_ori, self._payload_last_time = \
            self._compute_mcs(pos, ori,
                              self._payload_last_pos, self._payload_last_ori, self._payload_last_time, body='payload')
        if mcs is not None:
            self._publish(self._payload_pub, mcs)

    def _timing_on(self):
        return self._delay_from <= 0.0 or self.get_clock().now().nanoseconds * 1e-9 >= self._delay_from

    def _publish(self, pub, mcs):
        """Publish now, or after mocap_delay_s (restamped on release, as the rig stamps on receipt)."""
        if self._delay <= 0.0:
            pub.publish(mcs)
            return
        now = self.get_clock().now()
        q = self._queues.setdefault(id(pub), collections.deque())
        q.append((now.nanoseconds * 1e-9, mcs))
        # ramped in over 2 s from mocap_delay_from_s, so the stream slows instead of stopping for the delay
        delay = self._delay * min(1.0, max(0.0, (now.nanoseconds * 1e-9 - self._delay_from) / 2.0)) \
            if self._delay_from > 0.0 else self._delay
        while q and q[0][0] <= now.nanoseconds * 1e-9 - delay:
            _, m = q.popleft()
            m.header.stamp = now.to_msg()
            pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = PayloadMocapEmulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

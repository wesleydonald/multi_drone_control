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
"""

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose, Twist, PoseStamped
from interfaces.msg import MotionCaptureState
from tf_transformations import quaternion_multiply, quaternion_inverse, quaternion_matrix


class PayloadMocapEmulator(Node):
    def __init__(self):
        super().__init__('payload_mocap_emulator')

        self.declare_parameter('drone_id', 0)
        self.declare_parameter('drone_name', 'x3_drone0')
        self.declare_parameter('parent_model', 'lift_system')
        self.declare_parameter('publish_payload', True)

        drone_id = self.get_parameter('drone_id').value
        drone_name = self.get_parameter('drone_name').value
        parent = self.get_parameter('parent_model').value
        publish_payload = self.get_parameter('publish_payload').value

        drone_pose_topic = f'/model/{parent}/model/{drone_name}/pose'
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

        # Payload state (published by the node whose drone_id is 0)
        self._payload_last_pos = None
        self._payload_last_ori = None
        self._payload_last_time = None
        if publish_payload:
            self._payload_pub = self.create_publisher(
                MotionCaptureState, '/payload/motion_capture_state', 5)
            self.create_subscription(
                PoseArray, payload_pose_topic, self._payload_cb, 10)
        else:
            self._payload_pub = None

    # ------------------------------------------------------------------
    @staticmethod
    def _normalize_quat(x, y, z, w):
        if w < 0:
            return -x, -y, -z, -w
        return x, y, z, w

    def _compute_mcs(self, pos, ori, last_pos, last_ori, last_time):
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
    def _drone_cb(self, msg: PoseArray):
        if not msg.poses:
            return
        pos = msg.poses[-1].position
        ori = msg.poses[-1].orientation
        ori.x, ori.y, ori.z, ori.w = self._normalize_quat(ori.x, ori.y, ori.z, ori.w)

        mcs, self._drone_last_pos, self._drone_last_ori, self._drone_last_time = \
            self._compute_mcs(pos, ori,
                              self._drone_last_pos, self._drone_last_ori, self._drone_last_time)
        if mcs is not None:
            self._drone_pub.publish(mcs)

    def _payload_cb(self, msg: PoseArray):
        if not msg.poses or self._payload_pub is None:
            return
        # payload sub-model: index 0 is the model pose, index 1 is the 'body' link
        idx = 1 if len(msg.poses) > 1 else 0
        pos = msg.poses[idx].position
        ori = msg.poses[idx].orientation
        ori.x, ori.y, ori.z, ori.w = self._normalize_quat(ori.x, ori.y, ori.z, ori.w)

        mcs, self._payload_last_pos, self._payload_last_ori, self._payload_last_time = \
            self._compute_mcs(pos, ori,
                              self._payload_last_pos, self._payload_last_ori, self._payload_last_time)
        if mcs is not None:
            self._payload_pub.publish(mcs)


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

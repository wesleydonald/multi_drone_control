#!/usr/bin/env python3
"""IRL electromagnet manager with explicit *assumed* attachment semantics.

This node never calls Gazebo and never claims to sense physical attachment. It
commands the electromagnet through a magnet-only ELRSCommand override and
publishes /magnet/object_attached=True only after all of the following hold:
  - high-level /magnet/command is ON,
  - the tracked pickup object and measured magnet tip are both fresh,
  - their configured physical contact points are sufficiently close,
  - relative contact-point speed is below a configurable threshold,
  - the condition persists for a configurable dwell.

The Bool is therefore inferred commissioning state, not a measured contact
sensor. After attachment is declared, live geometry is re-checked continuously;
sustained separation or stale pose data de-latches the state. A declared loss
remains false until magnet OFF rearms the next pickup attempt.
"""
from __future__ import annotations

import time
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import PoseArray, PoseStamped
from rclpy.node import Node
from std_msgs.msg import Bool, String

from interfaces.msg import ELRSCommand
from .irl_pickup_geometry import transform_level_yaw_point


class MagnetAttachmentManagerIRL(Node):
    def __init__(self) -> None:
        super().__init__('magnet_attachment_manager_irl')

        self.magnet_command_topic = str(
            self.declare_parameter('magnet_command_topic', '/magnet/command').value
        )
        self.object_attached_topic = str(
            self.declare_parameter('object_attached_topic', '/magnet/object_attached').value
        )
        self.attachment_state_topic = str(
            self.declare_parameter('attachment_state_topic', '/magnet/attachment_state').value
        )
        self.magnet_tip_pose_topic = str(
            self.declare_parameter('magnet_tip_pose_topic', '/magnet_tip_pose').value
        )
        self.pickup_object_pose_topic = str(
            self.declare_parameter(
                'pickup_object_pose_topic', '/irl/pickup_object/pose'
            ).value
        )
        self.pickup_object_index = int(
            self.declare_parameter('pickup_object_index', 0).value
        )

        # Keep these values identical to the online_join_planner geometry fields
        # in config/irl_commissioning.yaml. The pickup point is object-frame/yaw-only
        # because PayloadGeometryProfile assumes a level object. The magnet offset is
        # deliberately world-frame because the existing planner uses that convention.
        self.pickup_point_offset = np.array([
            float(self.declare_parameter('pickup_point_offset_x', 0.0).value),
            float(self.declare_parameter('pickup_point_offset_y', 0.0).value),
            float(self.declare_parameter('pickup_point_offset_z', 0.0).value),
        ], dtype=float)
        self.magnet_marker_to_contact_face = np.array([
            float(self.declare_parameter('magnet_marker_to_contact_face_x', 0.0).value),
            float(self.declare_parameter('magnet_marker_to_contact_face_y', 0.0).value),
            float(self.declare_parameter('magnet_marker_to_contact_face_z', -0.03).value),
        ], dtype=float)

        self.attach_radius_m = max(
            0.0, float(self.declare_parameter('attach_radius_m', 0.12).value)
        )
        self.attach_speed_threshold_mps = max(
            0.0, float(self.declare_parameter('attach_speed_threshold_mps', 0.25).value)
        )
        self.attach_dwell_s = max(
            0.0, float(self.declare_parameter('attach_dwell_s', 0.15).value)
        )
        self.detach_dwell_s = max(
            0.0, float(self.declare_parameter('detach_dwell_s', 0.20).value)
        )
        self.pose_timeout_s = max(
            0.01, float(self.declare_parameter('pose_timeout_s', 0.25).value)
        )

        # Magnet-only ELRS output. Until the physical Betaflight AUX mapping is
        # positively identified, fan the same magnet command across the known-unused
        # logical ELRS channels 5..10. elrs_interface_irl applies the same channel
        # list to the live CRSF packet; flight channels and the arm channel are not
        # touched. The legacy scalar remains declared for old launch commands.
        self.enable_elrs_magnet_output = bool(
            self.declare_parameter('enable_elrs_magnet_output', True).value
        )
        self.elrs_magnet_command_topic = str(
            self.declare_parameter('elrs_magnet_command_topic', '/magnet/ELRSCommand').value
        )
        self.elrs_magnet_channel = int(
            self.declare_parameter('elrs_magnet_channel', 10).value
        )
        requested_channels = [
            int(value)
            for value in self.declare_parameter(
                'elrs_magnet_channels', [self.elrs_magnet_channel]
            ).value
        ]
        self.elrs_magnet_channels = []
        for channel in requested_channels:
            if channel < 5 or channel > 10:
                raise ValueError(
                    f'elrs_magnet_channels contains unsafe channel {channel}; '
                    'IRL fan-out is restricted to known-unused logical channels 5..10.'
                )
            if channel not in self.elrs_magnet_channels:
                self.elrs_magnet_channels.append(channel)
        if not self.elrs_magnet_channels:
            raise ValueError('elrs_magnet_channels must contain at least one channel.')
        self.elrs_magnet_on_value = float(
            self.declare_parameter('elrs_magnet_on_value', 1.0).value
        )
        self.elrs_magnet_off_value = float(
            self.declare_parameter('elrs_magnet_off_value', -1.0).value
        )
        self.elrs_repeat_rate_hz = max(
            0.1, float(self.declare_parameter('elrs_repeat_rate_hz', 2.0).value)
        )

        self.magnet_on = False
        self.assumed_attached = False
        self.attachment_loss_latched = False
        self.attach_condition_start: Optional[float] = None
        self.detach_condition_start: Optional[float] = None

        self.tip_contact_position: Optional[np.ndarray] = None
        self.tip_contact_velocity = np.zeros(3, dtype=float)
        self.last_tip_time: Optional[float] = None

        self.pickup_contact_position: Optional[np.ndarray] = None
        self.pickup_contact_velocity = np.zeros(3, dtype=float)
        self.last_pickup_time: Optional[float] = None

        self._last_elrs_state: Optional[bool] = None
        self._last_elrs_publish_time = 0.0

        self.create_subscription(String, self.magnet_command_topic, self.command_cb, 10)
        self.create_subscription(PoseStamped, self.magnet_tip_pose_topic, self.tip_cb, 10)
        self.create_subscription(
            PoseArray, self.pickup_object_pose_topic, self.pickup_cb, 10
        )
        self.attached_pub = self.create_publisher(Bool, self.object_attached_topic, 10)
        self.state_pub = self.create_publisher(String, self.attachment_state_topic, 10)
        self.elrs_pub = self.create_publisher(ELRSCommand, self.elrs_magnet_command_topic, 5)
        self.timer = self.create_timer(1.0 / 30.0, self.timer_cb)

        self.get_logger().warn(
            'IRL attachment state is INFERRED, not sensed. '
            f'pickup_topic={self.pickup_object_pose_topic}[{self.pickup_object_index}], '
            f'radius={self.attach_radius_m:.3f} m, '
            f'rel_speed<={self.attach_speed_threshold_mps:.3f} m/s, '
            f'attach_dwell={self.attach_dwell_s:.2f} s, '
            f'detach_dwell={self.detach_dwell_s:.2f} s. '
            'Missing/stale pickup mocap fails closed. '
            f'ELRS magnet fan-out channels={self.elrs_magnet_channels}.'
        )

    @staticmethod
    def _set_channel(msg: ELRSCommand, channel_index: int, value: float) -> bool:
        name = f'channel_{channel_index}'
        if not hasattr(msg, name):
            return False
        setattr(msg, name, float(np.clip(value, -1.0, 1.0)))
        return True

    @staticmethod
    def _pose_components(pose):
        position = np.array([
            pose.position.x,
            pose.position.y,
            pose.position.z,
        ], dtype=float)
        quaternion_wxyz = np.array([
            pose.orientation.w,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
        ], dtype=float)
        return position, quaternion_wxyz

    @staticmethod
    def _filtered_velocity(
        previous_position: Optional[np.ndarray],
        previous_time: Optional[float],
        previous_velocity: np.ndarray,
        position: np.ndarray,
        now: float,
    ) -> np.ndarray:
        if previous_position is None or previous_time is None:
            return np.zeros(3, dtype=float)
        dt = now - previous_time
        if dt <= 1e-6:
            return previous_velocity
        raw_v = (position - previous_position) / dt
        return 0.35 * raw_v + 0.65 * previous_velocity

    def publish_elrs(self, force: bool = False) -> None:
        if not self.enable_elrs_magnet_output:
            return
        now = time.time()
        period = 1.0 / self.elrs_repeat_rate_hz
        if (
            not force
            and self._last_elrs_state is self.magnet_on
            and now - self._last_elrs_publish_time < period
        ):
            return
        msg = ELRSCommand()
        msg.armed = False
        # Safe/inspectable defaults. The dedicated topic is consumed only for the
        # configured magnet fan-out channels by elrs_interface_irl.
        for idx in range(0, 11):
            if idx == 4:
                continue
            self._set_channel(msg, idx, 0.0)
        self._set_channel(msg, 2, -1.0)
        value = self.elrs_magnet_on_value if self.magnet_on else self.elrs_magnet_off_value
        for channel in self.elrs_magnet_channels:
            if not self._set_channel(msg, channel, value):
                self.get_logger().error(
                    f'ELRSCommand has no channel_{channel}; magnet command not sent.'
                )
                return
        self.elrs_pub.publish(msg)
        self._last_elrs_state = self.magnet_on
        self._last_elrs_publish_time = now

    def command_cb(self, msg: String) -> None:
        command = str(msg.data).strip().upper()
        if command in {'ON', '1', 'TRUE', 'ENABLE', 'ENABLED'}:
            self.magnet_on = True
            self.publish_elrs(force=True)
        elif command in {'OFF', '0', 'FALSE', 'DISABLE', 'DISABLED'}:
            self.magnet_on = False
            self.assumed_attached = False
            self.attachment_loss_latched = False
            self.attach_condition_start = None
            self.detach_condition_start = None
            self.publish_elrs(force=True)
        else:
            self.get_logger().warn(f'Ignoring unknown magnet command {msg.data!r}')

    def tip_cb(self, msg: PoseStamped) -> None:
        now = time.time()
        position, quaternion_wxyz = self._pose_components(msg.pose)
        # Match PayloadGeometryProfile.pickup_target_geometry(): the tracked
        # magnet marker has no trusted full attitude, so this contact-face offset
        # is a configured world-frame vector rather than a rotated body vector.
        contact = position + self.magnet_marker_to_contact_face
        self.tip_contact_velocity = self._filtered_velocity(
            self.tip_contact_position,
            self.last_tip_time,
            self.tip_contact_velocity,
            contact,
            now,
        )
        self.tip_contact_position = contact
        self.last_tip_time = now

    def pickup_cb(self, msg: PoseArray) -> None:
        if self.pickup_object_index < 0:
            index = len(msg.poses) + self.pickup_object_index
        else:
            index = self.pickup_object_index
        if index < 0 or index >= len(msg.poses):
            self.get_logger().warn(
                f'pickup_object_index={self.pickup_object_index} out of range for '
                f'{self.pickup_object_pose_topic}; length={len(msg.poses)}',
                throttle_duration_sec=2.0,
            )
            return

        now = time.time()
        position, quaternion_wxyz = self._pose_components(msg.poses[index])
        # Match PayloadGeometryProfile.pickup_point_world(): the pickup object
        # is treated as level and only its measured yaw rotates the local offset.
        contact = transform_level_yaw_point(
            position, quaternion_wxyz, self.pickup_point_offset
        )
        self.pickup_contact_velocity = self._filtered_velocity(
            self.pickup_contact_position,
            self.last_pickup_time,
            self.pickup_contact_velocity,
            contact,
            now,
        )
        self.pickup_contact_position = contact
        self.last_pickup_time = now

    def timer_cb(self) -> None:
        now = time.time()
        self.publish_elrs(force=False)

        tip_fresh = (
            self.tip_contact_position is not None
            and self.last_tip_time is not None
            and now - self.last_tip_time <= self.pose_timeout_s
        )
        pickup_fresh = (
            self.pickup_contact_position is not None
            and self.last_pickup_time is not None
            and now - self.last_pickup_time <= self.pose_timeout_s
        )
        both_fresh = tip_fresh and pickup_fresh

        distance = float('nan')
        relative_speed = float('nan')
        close = False
        slow = False
        if both_fresh:
            distance = float(
                np.linalg.norm(self.tip_contact_position - self.pickup_contact_position)
            )
            relative_speed = float(
                np.linalg.norm(self.tip_contact_velocity - self.pickup_contact_velocity)
            )
            close = distance <= self.attach_radius_m
            slow = relative_speed <= self.attach_speed_threshold_mps

        attachment_maintained = self.magnet_on and both_fresh and close
        if self.assumed_attached:
            self.attach_condition_start = None
            if attachment_maintained:
                self.detach_condition_start = None
            else:
                if self.detach_condition_start is None:
                    self.detach_condition_start = now
                if now - self.detach_condition_start >= self.detach_dwell_s:
                    self.assumed_attached = False
                    self.attachment_loss_latched = True
                    self.get_logger().warn(
                        'ASSUMED ATTACHMENT LOST: live pickup geometry was invalid '
                        f'for {self.detach_dwell_s:.2f} s. Waiting for magnet OFF '
                        'before another attachment attempt.'
                    )
        elif not self.attachment_loss_latched:
            self.detach_condition_start = None
            if self.magnet_on and both_fresh and close and slow:
                if self.attach_condition_start is None:
                    self.attach_condition_start = now
                if now - self.attach_condition_start >= self.attach_dwell_s:
                    self.assumed_attached = True
                    self.attach_condition_start = None
                    self.detach_condition_start = None
                    self.get_logger().warn(
                        'ASSUMED ATTACHED: live pickup geometry/relative-speed/dwell gate passed. '
                        'This is not physical attachment feedback.'
                    )
            else:
                self.attach_condition_start = None
        else:
            self.attach_condition_start = None

        attached = Bool()
        attached.data = bool(self.assumed_attached)
        self.attached_pub.publish(attached)

        attach_dwell = (
            0.0 if self.attach_condition_start is None else now - self.attach_condition_start
        )
        detach_dwell = (
            0.0 if self.detach_condition_start is None else now - self.detach_condition_start
        )
        tip_age = float('nan') if self.last_tip_time is None else now - self.last_tip_time
        pickup_age = (
            float('nan') if self.last_pickup_time is None else now - self.last_pickup_time
        )
        state = String()
        state.data = (
            f'mode=physical_assumed magnet_on={self.magnet_on} '
            f'assumed_attached={self.assumed_attached} tip_fresh={tip_fresh} '
            f'pickup_fresh={pickup_fresh} tip_age_s={tip_age:.3f} '
            f'pickup_age_s={pickup_age:.3f} distance_m={distance:.3f} '
            f'relative_speed_mps={relative_speed:.3f} close={close} slow={slow} '
            f'attachment_maintained={attachment_maintained} '
            f'attach_dwell_s={attach_dwell:.2f}/{self.attach_dwell_s:.2f} '
            f'detach_dwell_s={detach_dwell:.2f}/{self.detach_dwell_s:.2f} '
            f'loss_latched={self.attachment_loss_latched} '
            'ATTACHMENT_IS_NOT_SENSOR_VERIFIED'
        )
        self.state_pub.publish(state)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MagnetAttachmentManagerIRL()
    try:
        rclpy.spin(node)
    finally:
        # Command the magnet OFF on orderly shutdown. The flight controller still
        # owns arm/disarm separately.
        node.magnet_on = False
        node.publish_elrs(force=True)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

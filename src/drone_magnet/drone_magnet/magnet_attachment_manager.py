#!/usr/bin/env python3
"""Magnet attachment manager for the detachable payload simulation.

This node provides a ROS-side "magnet" abstraction in front of Gazebo's raw
DetachableJoint attach/detach topics. The raw Gazebo attach topic creates the
joint immediately, regardless of where the payload currently is. This manager
only commands attachment when the simulated magnet tip is close to the pickup
object and the relative speed is low enough.

Typical first test:

  ros2 run drone_communication magnet_attachment_manager --ros-args \
    -p command_backend:=gz_cli \
    -p magnet_tip_pose_index:=-1 \
    -p object_pose_index:=0

Then toggle the magnet with:

  ros2 topic pub --once /magnet/command std_msgs/msg/String "{data: ON}"
  ros2 topic pub --once /magnet/command std_msgs/msg/String "{data: OFF}"

The default fixed-joint mode uses the existing Gazebo DetachableJoint plugin.
An experimental kinematic mode is included as a debug fallback; it repeatedly
sets the pickup object's pose to the magnet tip while attached, using the Gazebo
/world/<world>/set_pose service.
"""

from __future__ import annotations

import math
import subprocess
import time
from typing import Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from std_msgs.msg import Bool, Empty, String
from interfaces.msg import ELRSCommand


class MagnetAttachmentManager(Node):
    def __init__(self) -> None:
        super().__init__('magnet_attachment_manager')

        # High-level magnet interface.
        self.magnet_command_topic = self.declare_parameter('magnet_command_topic', '/magnet/command').value
        self.object_attached_topic = self.declare_parameter('object_attached_topic', '/magnet/object_attached').value
        self.attachment_state_topic = self.declare_parameter('attachment_state_topic', '/magnet/attachment_state').value

        # Pose sources. Gazebo PosePublisher often publishes PoseArray without
        # names on the ROS side, so the pose index is parameterised.
        self.magnet_tip_pose_topic = self.declare_parameter('magnet_tip_pose_topic', '/magnet_tip_pose').value
        self.object_pose_topic = self.declare_parameter('object_pose_topic', '/model/payload_model/pose').value
        self.magnet_tip_pose_index = int(self.declare_parameter('magnet_tip_pose_index', -1).value)
        self.object_pose_index = int(self.declare_parameter('object_pose_index', 0).value)
        # OFF-CENTRE weld: shift the weld-reference point off the payload centre by this
        # world-frame XY (MUST match attach_target_publisher's x/y offset). The weld triggers
        # on tip-distance to THIS point, so with the offset the tip welds at a ring point
        # (moment arm != 0) instead of the centre -- what lets the dissipative network's
        # balanced-tension mode reconfigure the fleet. 0,0 = centre weld (central lifter).
        self.object_x_offset = float(self.declare_parameter('object_x_offset', 0.0).value)
        self.object_y_offset = float(self.declare_parameter('object_y_offset', 0.0).value)
        self.magnet_tip_pose_msg_type = str(
            self.declare_parameter('magnet_tip_pose_msg_type', 'pose_stamped').value
        ).strip().lower()

        # Fallback object pose if the payload_model Gazebo pose is not bridged
        # into ROS yet. This matches the current world spawn pose for the red ball.
        self.use_fallback_object_pose = bool(self.declare_parameter('use_fallback_object_pose', True).value)
        self.fallback_object_x = float(self.declare_parameter('fallback_object_x', 0.0).value)
        self.fallback_object_y = float(self.declare_parameter('fallback_object_y', 0.0).value)
        self.fallback_object_z = float(self.declare_parameter('fallback_object_z', 0.08).value)

        # Attachment conditions.
        self.attach_radius = float(self.declare_parameter('attach_radius', 0.12).value)
        self.attach_speed_threshold = float(self.declare_parameter('attach_speed_threshold', 0.35).value)
        self.attach_dwell_time_s = float(self.declare_parameter('attach_dwell_time_s', 0.15).value)
        self.pose_timeout_s = float(self.declare_parameter('pose_timeout_s', 1.0).value)
        self.command_timeout_s = float(self.declare_parameter('command_timeout_s', 0.0).value)  # 0 disables timeout
        self.detach_when_magnet_off = bool(self.declare_parameter('detach_when_magnet_off', True).value)

        # Backend for low-level attach/detach. Since /payload/attach is a Gazebo
        # Transport topic in your current sim, gz_cli works without a ROS bridge.
        # Valid: 'gz_cli', 'ros_topic', 'both'.
        self.attachment_mode = str(self.declare_parameter('attachment_mode', 'fixed_joint').value).lower()
        self.command_backend = str(self.declare_parameter('command_backend', 'gz_cli').value).lower()
        self.gz_attach_topic = self.declare_parameter('gz_attach_topic', '/payload/attach').value
        self.gz_detach_topic = self.declare_parameter('gz_detach_topic', '/payload/detach').value
        self.ros_attach_topic = self.declare_parameter('ros_attach_topic', '/payload/attach').value
        self.ros_detach_topic = self.declare_parameter('ros_detach_topic', '/payload/detach').value
        self.gz_command_timeout_s = float(self.declare_parameter('gz_command_timeout_s', 1.0).value)

        # IRL electromagnet output over ELRS. This publishes a small magnet-only
        # ELRSCommand on a separate topic; the patched elrs_interface subscribes
        # to it and only copies the selected aux channel into the live CRSF packet,
        # so flight channels from the controller are preserved.
        self.enable_elrs_magnet_output = bool(
            self.declare_parameter('enable_elrs_magnet_output', True).value
        )
        self.elrs_magnet_command_topic = self.declare_parameter(
            'elrs_magnet_command_topic', '/magnet/ELRSCommand'
        ).value
        self.elrs_magnet_channel = int(self.declare_parameter('elrs_magnet_channel', 10).value)
        self.elrs_magnet_on_value = float(self.declare_parameter('elrs_magnet_on_value', 1.0).value)
        self.elrs_magnet_off_value = float(self.declare_parameter('elrs_magnet_off_value', -1.0).value)
        self.elrs_magnet_repeat_rate_hz = float(
            self.declare_parameter('elrs_magnet_repeat_rate_hz', 2.0).value
        )
        self._last_elrs_magnet_state: Optional[bool] = None
        self._last_elrs_publish_time = 0.0

        # Optional debug-only kinematic fallback. This does not use the fixed
        # joint. It is useful only if the detachable joint plugin or bridge is
        # temporarily broken.
        self.world_name = self.declare_parameter('world_name', 'quadcopter').value
        self.kinematic_object_name = self.declare_parameter('kinematic_object_name', 'payload_model').value
        self.kinematic_z_offset = float(self.declare_parameter('kinematic_z_offset', 0.0).value)
        self.kinematic_update_rate_hz = float(self.declare_parameter('kinematic_update_rate_hz', 30.0).value)
        self.kinematic_set_pose_timeout_s = float(self.declare_parameter('kinematic_set_pose_timeout_s', 0.2).value)
        self._last_kinematic_update = 0.0
        self._last_kinematic_warning = 0.0

        # State.
        self.magnet_on = False
        self.last_magnet_command_time: Optional[float] = None
        self.attached = False
        self.attach_command_sent = False
        self.detach_command_sent = False
        self.attach_condition_start: Optional[float] = None
        self.last_distance = float('nan')
        self.last_relative_speed = float('nan')
        self.last_state_text = ''

        self.magnet_tip_position: Optional[np.ndarray] = None
        self.object_position: Optional[np.ndarray] = None
        self.prev_magnet_tip_position: Optional[np.ndarray] = None
        self.prev_object_position: Optional[np.ndarray] = None
        self.magnet_tip_velocity = np.zeros(3, dtype=float)
        self.object_velocity = np.zeros(3, dtype=float)
        self.last_magnet_tip_time: Optional[float] = None
        self.last_object_time: Optional[float] = None

        self.create_subscription(String, self.magnet_command_topic, self.magnet_command_callback, 10)
        if self.magnet_tip_pose_msg_type in ('pose_stamped', 'posestamped', 'stamped'):
            self.create_subscription(PoseStamped, self.magnet_tip_pose_topic, self.magnet_tip_pose_stamped_callback, 10)
        elif self.magnet_tip_pose_msg_type in ('pose_array', 'posearray', 'array'):
            self.create_subscription(PoseArray, self.magnet_tip_pose_topic, self.magnet_tip_pose_callback, 10)
        else:
            self.get_logger().warn(
                f"Unknown magnet_tip_pose_msg_type={self.magnet_tip_pose_msg_type!r}; "
                "defaulting to PoseStamped."
            )
            self.create_subscription(PoseStamped, self.magnet_tip_pose_topic, self.magnet_tip_pose_stamped_callback, 10)
        self.create_subscription(PoseArray, self.object_pose_topic, self.object_pose_callback, 10)

        self.attach_pub = self.create_publisher(Empty, self.ros_attach_topic, 5)
        self.detach_pub = self.create_publisher(Empty, self.ros_detach_topic, 5)
        self.attached_pub = self.create_publisher(Bool, self.object_attached_topic, 5)
        self.state_pub = self.create_publisher(String, self.attachment_state_topic, 5)
        self.elrs_magnet_pub = self.create_publisher(ELRSCommand, self.elrs_magnet_command_topic, 5)

        # Gazebo's DetachableJoint is created ATTACHED at spawn, which would rigidly weld the
        # magnet to the payload from t=0 (before any ATTACH) and drag the payload around. Send
        # a one-shot detach shortly after start so the magnet begins FREE; it then only welds
        # when magnet_on + proximity trigger command_attach_once. Delay lets the joint exist.
        self.detach_on_start = bool(self.declare_parameter('detach_on_start', True).value)
        self.detach_on_start_delay_s = float(
            self.declare_parameter('detach_on_start_delay_s', 1.0).value)
        # Re-send detach at this rate (not a one-shot / fixed window): the joint may spawn AFTER
        # this node starts, so we keep freeing it until the magnet is first commanded ON.
        self.detach_on_start_period_s = float(
            self.declare_parameter('detach_on_start_period_s', 1.0).value)
        self._start_time = time.time()
        self._last_start_detach = 0.0
        self._ever_magnet_on = False

        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)

        self.get_logger().info(
            'Magnet attachment manager started. '
            f'mode={self.attachment_mode}, backend={self.command_backend}, '
            f'magnet_tip_topic={self.magnet_tip_pose_topic}[{self.magnet_tip_pose_index}], '
            f'magnet_tip_type={self.magnet_tip_pose_msg_type}, '
            f'object_topic={self.object_pose_topic}[{self.object_pose_index}], '
            f'attach_radius={self.attach_radius:.3f} m, '
            f'attach_speed_threshold={self.attach_speed_threshold:.3f} m/s, '
            f'elrs_output={self.enable_elrs_magnet_output}, '
            f'elrs_topic={self.elrs_magnet_command_topic}, '
            f'elrs_channel={self.elrs_magnet_channel}'
        )

    @staticmethod
    def _pose_to_position(pose: Pose) -> np.ndarray:
        return np.array([pose.position.x, pose.position.y, pose.position.z], dtype=float)

    @staticmethod
    def _select_pose(msg: PoseArray, index: int) -> Optional[Pose]:
        if not msg.poses:
            return None
        if index < 0:
            index = len(msg.poses) + index
        if index < 0 or index >= len(msg.poses):
            return None
        return msg.poses[index]

    def _update_position_and_velocity(
        self,
        new_position: np.ndarray,
        now: float,
        old_position: Optional[np.ndarray],
        old_time: Optional[float],
    ) -> Tuple[np.ndarray, np.ndarray]:
        if old_position is None or old_time is None:
            return new_position, np.zeros(3, dtype=float)
        dt = max(1e-6, now - old_time)
        return new_position, (new_position - old_position) / dt

    def magnet_tip_pose_callback(self, msg: PoseArray) -> None:
        pose = self._select_pose(msg, self.magnet_tip_pose_index)
        if pose is None:
            self.get_logger().warn(
                f'No pose at magnet_tip_pose_index={self.magnet_tip_pose_index} in {self.magnet_tip_pose_topic}; '
                f'PoseArray length={len(msg.poses)}',
                throttle_duration_sec=2.0,
            )
            return
        now = time.time()
        new_pos = self._pose_to_position(pose)
        self.prev_magnet_tip_position = self.magnet_tip_position
        self.magnet_tip_position, self.magnet_tip_velocity = self._update_position_and_velocity(
            new_pos, now, self.magnet_tip_position, self.last_magnet_tip_time
        )
        self.last_magnet_tip_time = now

    def magnet_tip_pose_stamped_callback(self, msg: PoseStamped) -> None:
        """Update magnet tip from a world-frame PoseStamped, e.g. /magnet_tip_pose."""
        now = time.time()
        new_pos = self._pose_to_position(msg.pose)
        self.prev_magnet_tip_position = self.magnet_tip_position
        self.magnet_tip_position, self.magnet_tip_velocity = self._update_position_and_velocity(
            new_pos, now, self.magnet_tip_position, self.last_magnet_tip_time
        )
        self.last_magnet_tip_time = now

    def object_pose_callback(self, msg: PoseArray) -> None:
        pose = self._select_pose(msg, self.object_pose_index)
        if pose is None:
            self.get_logger().warn(
                f'No pose at object_pose_index={self.object_pose_index} in {self.object_pose_topic}; '
                f'PoseArray length={len(msg.poses)}',
                throttle_duration_sec=2.0,
            )
            return
        now = time.time()
        new_pos = self._pose_to_position(pose)
        new_pos[0] += self.object_x_offset          # off-centre weld reference (see __init__)
        new_pos[1] += self.object_y_offset
        self.prev_object_position = self.object_position
        self.object_position, self.object_velocity = self._update_position_and_velocity(
            new_pos, now, self.object_position, self.last_object_time
        )
        self.last_object_time = now

    def _set_elrs_channel(self, msg: ELRSCommand, channel_index: int, value: float) -> bool:
        """Set one normalised ELRS channel field, e.g. channel_10 = +1.0."""
        field_name = f'channel_{channel_index}'
        if not hasattr(msg, field_name):
            self.get_logger().warn(
                f'Cannot set {field_name} on ELRSCommand; check the message definition.',
                throttle_duration_sec=2.0,
            )
            return False
        setattr(msg, field_name, float(max(-1.0, min(1.0, value))))
        return True

    def publish_elrs_magnet_command(self, magnet_on: bool, reason: str = '', force: bool = False) -> None:
        """Publish the magnet ON/OFF aux channel for the patched ELRS interface."""
        if not self.enable_elrs_magnet_output:
            return

        now = time.time()
        repeat_period = 1.0 / max(0.01, self.elrs_magnet_repeat_rate_hz)
        if (
            not force
            and self._last_elrs_magnet_state is magnet_on
            and now - self._last_elrs_publish_time < repeat_period
        ):
            return

        msg = ELRSCommand()
        # These fields are ignored by the patched elrs_interface magnet callback,
        # but fill safe defaults so the message is still inspectable in ROS echo.
        msg.armed = False
        for idx in [0, 1, 3, 4, 5, 6, 7, 8, 9, 10]:
            self._set_elrs_channel(msg, idx, 0.0)
        self._set_elrs_channel(msg, 2, -1.0)

        value = self.elrs_magnet_on_value if magnet_on else self.elrs_magnet_off_value
        if not all(self._set_elrs_channel(msg, ch, value) for ch in range(5, 11)):
            return

        self.elrs_magnet_pub.publish(msg)
        self._last_elrs_publish_time = now
        changed = self._last_elrs_magnet_state is not magnet_on
        self._last_elrs_magnet_state = magnet_on
        if changed or force:
            state = 'ON' if magnet_on else 'OFF'
            suffix = f' ({reason})' if reason else ''
            self.get_logger().info(
                f'ELRS magnet {state}: channel_{self.elrs_magnet_channel}={value:+.2f}{suffix}'
            )

    def magnet_command_callback(self, msg: String) -> None:
        command = str(msg.data).strip().upper()
        self.last_magnet_command_time = time.time()

        if command in ('ON', '1', 'TRUE', 'ENABLE', 'ENABLED'):
            if not self.magnet_on:
                self.get_logger().info('Magnet command: ON')
            self.magnet_on = True
            self._ever_magnet_on = True  # stop the startup keep-free retry; a real attach is wanted
            self.detach_command_sent = False
            self.publish_elrs_magnet_command(True, 'magnet command ON', force=True)

        elif command in ('OFF', '0', 'FALSE', 'DISABLE', 'DISABLED'):
            if self.magnet_on:
                self.get_logger().info('Magnet command: OFF')
            self.magnet_on = False
            self.attach_condition_start = None
            self.attach_command_sent = False
            self.publish_elrs_magnet_command(False, 'magnet command OFF', force=True)
            if self.detach_when_magnet_off:
                self.command_detach_once('magnet command OFF')

        else:
            self.get_logger().warn(f'Ignoring unknown magnet command: {msg.data!r}')

    def current_object_position(self) -> Optional[np.ndarray]:
        if self.object_position is not None:
            return self.object_position.copy()
        if self.use_fallback_object_pose:
            return np.array([self.fallback_object_x, self.fallback_object_y, self.fallback_object_z], dtype=float)
        return None

    def poses_fresh(self) -> Tuple[bool, str]:
        now = time.time()
        if self.magnet_tip_position is None or self.last_magnet_tip_time is None:
            return False, 'waiting for magnet tip pose'
        if now - self.last_magnet_tip_time > self.pose_timeout_s:
            return False, f'magnet tip pose stale ({now - self.last_magnet_tip_time:.2f}s)'
        if self.object_position is not None and self.last_object_time is not None:
            if now - self.last_object_time > self.pose_timeout_s:
                if not self.use_fallback_object_pose:
                    return False, f'object pose stale ({now - self.last_object_time:.2f}s)'
        elif not self.use_fallback_object_pose:
            return False, 'waiting for object pose'
        return True, 'poses fresh'

    def command_attach_once(self, reason: str) -> None:
        if self.attach_command_sent and self.attached:
            return
        self.attach_command_sent = True
        self.detach_command_sent = False
        self.attached = True  # local latch; may be overwritten later if you bridge joint state
        self.get_logger().info(f'ATTACH: {reason}')
        self.publish_elrs_magnet_command(True, f'attach: {reason}', force=True)

        if self.command_backend in ('ros_topic', 'both'):
            self.attach_pub.publish(Empty())
        if self.command_backend in ('gz_cli', 'both'):
            self.publish_gz_empty(self.gz_attach_topic)

    def command_detach_once(self, reason: str) -> None:
        if self.detach_command_sent and not self.attached:
            return
        self.detach_command_sent = True
        self.attach_command_sent = False
        self.attached = False
        self.attach_condition_start = None
        self.get_logger().info(f'DETACH: {reason}')
        self.publish_elrs_magnet_command(False, f'detach: {reason}', force=True)

        if self.command_backend in ('ros_topic', 'both'):
            self.detach_pub.publish(Empty())
        if self.command_backend in ('gz_cli', 'both'):
            self.publish_gz_empty(self.gz_detach_topic)

    def publish_gz_empty(self, topic: str) -> bool:
        try:
            result = subprocess.run(
                ['gz', 'topic', '-t', topic, '-m', 'gz.msgs.Empty', '-p', ''],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                timeout=self.gz_command_timeout_s,
                check=False,
            )
        except Exception as exc:  # noqa: BLE001 - this is a sim utility node
            self.get_logger().error(f'Failed to call gz topic for {topic}: {exc}')
            return False

        if result.returncode != 0:
            self.get_logger().error(
                f'gz topic command failed for {topic}: returncode={result.returncode}, stderr={result.stderr.strip()}'
            )
            return False
        return True

    def update_kinematic_attachment(self, now: float) -> None:
        if self.attachment_mode != 'kinematic' or not self.attached:
            return
        if self.magnet_tip_position is None:
            return
        period = 1.0 / max(1.0, self.kinematic_update_rate_hz)
        if now - self._last_kinematic_update < period:
            return
        self._last_kinematic_update = now

        p = self.magnet_tip_position.copy()
        p[2] += self.kinematic_z_offset
        req = (
            f'name: "{self.kinematic_object_name}" '
            f'position {{ x: {p[0]:.6f} y: {p[1]:.6f} z: {p[2]:.6f} }} '
            'orientation { w: 1.0 }'
        )
        service = f'/world/{self.world_name}/set_pose'
        try:
            result = subprocess.run(
                [
                    'gz', 'service', '-s', service,
                    '--reqtype', 'gz.msgs.Pose',
                    '--reptype', 'gz.msgs.Boolean',
                    '--timeout', str(int(1000.0 * self.kinematic_set_pose_timeout_s)),
                    '--req', req,
                ],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                timeout=max(0.2, self.kinematic_set_pose_timeout_s + 0.1),
                check=False,
            )
            if result.returncode != 0 and now - self._last_kinematic_warning > 2.0:
                self._last_kinematic_warning = now
                self.get_logger().warn(
                    f'Kinematic set_pose failed: returncode={result.returncode}, stderr={result.stderr.strip()}'
                )
        except Exception as exc:  # noqa: BLE001
            if now - self._last_kinematic_warning > 2.0:
                self._last_kinematic_warning = now
                self.get_logger().warn(f'Kinematic set_pose exception: {exc}')

    def publish_state(self, text: str) -> None:
        attached_msg = Bool()
        attached_msg.data = bool(self.attached)
        self.attached_pub.publish(attached_msg)

        state_msg = String()
        state_msg.data = text
        self.state_pub.publish(state_msg)
        self.last_state_text = text

    def timer_callback(self) -> None:
        now = time.time()

        # Keep the magnet FREE until it is first commanded ON. gz's DetachableJoint is created
        # ATTACHED at spawn, so without this the magnet is welded to the payload from t=0 and
        # drags it around (the pendulum gets pulled sideways instead of hanging straight down).
        # We re-send detach at a low rate -- not a one-shot / fixed window -- because the joint
        # may spawn AFTER this node starts, so a single early detach can be lost. Stops the
        # instant the magnet is first commanded ON, so it never fights a real attach.
        elapsed = now - self._start_time
        if (self.detach_on_start and not self._ever_magnet_on and not self.magnet_on
                and elapsed >= self.detach_on_start_delay_s
                and now - self._last_start_detach >= self.detach_on_start_period_s):
            self._last_start_detach = now
            self.attached = False
            self.get_logger().info(
                'Startup detach: keeping the auto-created magnet joint FREE until ATTACH',
                throttle_duration_sec=3.0)
            if self.command_backend in ('ros_topic', 'both'):
                self.detach_pub.publish(Empty())
            if self.command_backend in ('gz_cli', 'both'):
                self.publish_gz_empty(self.gz_detach_topic)

        if self.command_timeout_s > 0.0 and self.last_magnet_command_time is not None:
            if now - self.last_magnet_command_time > self.command_timeout_s:
                if self.magnet_on:
                    self.get_logger().warn('Magnet command timed out; turning magnet OFF')
                self.magnet_on = False
                self.publish_elrs_magnet_command(False, 'magnet command timeout')
                if self.detach_when_magnet_off:
                    self.command_detach_once('magnet command timeout')

        # Keep the physical ELRS magnet channel alive at a low rate even if the
        # high-level command is latched. This helps if the ELRS interface starts
        # after this node or misses the first command.
        self.publish_elrs_magnet_command(self.magnet_on)

        fresh, freshness_reason = self.poses_fresh()
        object_pos = self.current_object_position()
        if not fresh or object_pos is None or self.magnet_tip_position is None:
            self.publish_state(
                f'magnet_on={self.magnet_on} attached={self.attached} status={freshness_reason}'
            )
            return

        rel = self.magnet_tip_position - object_pos
        distance = float(np.linalg.norm(rel))
        rel_vel = self.magnet_tip_velocity - self.object_velocity
        relative_speed = float(np.linalg.norm(rel_vel))
        self.last_distance = distance
        self.last_relative_speed = relative_speed

        close_enough = distance <= self.attach_radius
        slow_enough = relative_speed <= self.attach_speed_threshold
        can_attach_now = self.magnet_on and close_enough and slow_enough

        if self.magnet_on and not self.attached:
            if can_attach_now:
                if self.attach_condition_start is None:
                    self.attach_condition_start = now
                dwell = now - self.attach_condition_start
                if dwell >= self.attach_dwell_time_s:
                    if self.attachment_mode == 'fixed_joint':
                        self.command_attach_once(
                            f'd={distance:.3f} m, rel_speed={relative_speed:.3f} m/s, dwell={dwell:.2f} s'
                        )
                    elif self.attachment_mode == 'kinematic':
                        self.attached = True
                        self.attach_command_sent = True
                        self.detach_command_sent = False
                        self.get_logger().info(
                            f'KINEMATIC ATTACH: d={distance:.3f} m, rel_speed={relative_speed:.3f} m/s, dwell={dwell:.2f} s'
                        )
                    else:
                        self.get_logger().warn(f'Unknown attachment_mode={self.attachment_mode!r}; not attaching')
            else:
                self.attach_condition_start = None

        if not self.magnet_on and self.attached and self.detach_when_magnet_off:
            self.command_detach_once('magnet OFF while attached')

        self.update_kinematic_attachment(now)

        dwell_text = 0.0 if self.attach_condition_start is None else now - self.attach_condition_start
        state = (
            f'magnet_on={self.magnet_on} attached={self.attached} '
            f'mode={self.attachment_mode} backend={self.command_backend} '
            f'distance={distance:.3f}m radius={self.attach_radius:.3f}m '
            f'rel_speed={relative_speed:.3f}m/s threshold={self.attach_speed_threshold:.3f}m/s '
            f'close={close_enough} slow={slow_enough} dwell={dwell_text:.2f}/{self.attach_dwell_time_s:.2f}s'
        )
        self.publish_state(state)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MagnetAttachmentManager()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

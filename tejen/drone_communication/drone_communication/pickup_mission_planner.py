#!/usr/bin/env python3
"""Isolated pickup mission planner for the suspended electromagnet drone.

This node is intentionally separate from online_join_planner.py so the pickup
sequence can be tested without touching the already-working reattachment / moving
obstacle pipeline.

It publishes a rolling quad-body MultiDOFJointTrajectory to /join_planner/reference
using magnet-tip feedback. The pickup object is the Gazebo payload_model red ball
published as /model/payload_model/pose. The magnet itself is commanded through
/magnet/command; the magnet_attachment_manager is responsible for turning that
high-level ON/OFF command into Gazebo detachable-joint attach/detach commands.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point, PoseArray, PoseStamped, Transform, Twist, Vector3
from interfaces.msg import MotionCaptureState
from rclpy.node import Node
from std_msgs.msg import Bool, String
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from visualization_msgs.msg import Marker, MarkerArray


APPROACH_ABOVE_PICKUP = 'APPROACH_ABOVE_PICKUP'
SETTLE_ABOVE_PICKUP = 'SETTLE_ABOVE_PICKUP'
DESCEND_TO_PICKUP = 'DESCEND_TO_PICKUP'
MAGNET_ATTACH_WAIT = 'MAGNET_ATTACH_WAIT'
LIFT_OBJECT = 'LIFT_OBJECT'
PICKUP_COMPLETE = 'PICKUP_COMPLETE'


def _pos_from_pose(pose) -> np.ndarray:
    return np.array([pose.position.x, pose.position.y, pose.position.z], dtype=float)


def _pos_from_motion_state(msg: MotionCaptureState) -> np.ndarray:
    return np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z], dtype=float)


def _vel_from_motion_state(msg: MotionCaptureState) -> np.ndarray:
    return np.array([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z], dtype=float)


def _clip_norm(v: np.ndarray, max_norm: float) -> np.ndarray:
    n = float(np.linalg.norm(v))
    if n <= max_norm or n < 1e-9:
        return v
    return v * (max_norm / n)


def _smoothstep(s: float) -> float:
    s = max(0.0, min(1.0, s))
    return s * s * (3.0 - 2.0 * s)


def _make_point(v: np.ndarray) -> Point:
    p = Point()
    p.x = float(v[0])
    p.y = float(v[1])
    p.z = float(v[2])
    return p


@dataclass
class TimedVector:
    position: np.ndarray
    velocity: np.ndarray
    stamp: float


class PickupMissionPlanner(Node):
    def __init__(self) -> None:
        super().__init__('pickup_mission_planner')

        # Topics
        self.drone_state_topic = self.declare_parameter('drone_state_topic', '/motion_capture_state').value
        self.magnet_tip_pose_topic = self.declare_parameter('magnet_tip_pose_topic', '/magnet_tip_pose').value
        self.object_pose_topic = self.declare_parameter('object_pose_topic', '/model/payload_model/pose').value
        self.object_attached_topic = self.declare_parameter('object_attached_topic', '/magnet/object_attached').value
        self.magnet_command_topic = self.declare_parameter('magnet_command_topic', '/magnet/command').value
        self.reference_topic = self.declare_parameter('reference_topic', '/join_planner/reference').value
        self.state_topic = self.declare_parameter('state_topic', '/pickup_planner/state').value
        self.marker_topic = self.declare_parameter('marker_topic', '/pickup_planner/markers').value

        # Pose indices
        self.object_index = int(self.declare_parameter('object_index', 1).value)

        # Timing / trajectory parameters
        self.frame_id = self.declare_parameter('frame_id', 'map').value
        self.publish_rate_hz = float(self.declare_parameter('publish_rate_hz', 30.0).value)
        self.horizon_seconds = float(self.declare_parameter('horizon_seconds', 3.0).value)
        # Keep the published reference contract close to online_join_planner:
        # a smooth Hermite intercept to the current target, then a hold/predict
        # segment for the rest of the horizon.
        self.intercept_time = float(self.declare_parameter('intercept_time', 2.0).value)
        # Diagnostic/compatibility mode: during the first pickup approach, publish
        # every point in the MPC horizon at the quad-body target instead of
        # starting point[0] at the current measured quad pose.  This tests whether
        # the controller is only reacting to the early part of the rolling horizon.
        self.step_reference_in_approach = bool(
            self.declare_parameter('step_reference_in_approach', True).value
        )
        self.max_reference_speed = float(self.declare_parameter('max_reference_speed', 1.0).value)
        self.max_reference_step_xy = float(self.declare_parameter('max_reference_step_xy', 0.45).value)
        self.min_reference_z = float(self.declare_parameter('min_reference_z', 0.35).value)
        self.input_timeout_s = float(self.declare_parameter('input_timeout_s', 0.5).value)

        # Pickup behaviour. Heights are magnet-tip heights relative to object z.
        self.pickup_approach_clearance = float(self.declare_parameter('pickup_approach_clearance', 0.35).value)
        self.pickup_attach_clearance = float(self.declare_parameter('pickup_attach_clearance', 0.045).value)
        self.pickup_lift_height = float(self.declare_parameter('pickup_lift_height', 0.55).value)
        self.pickup_descent_speed = float(self.declare_parameter('pickup_descent_speed', 0.12).value)
        self.pickup_lift_speed = float(self.declare_parameter('pickup_lift_speed', 0.18).value)
        self.pickup_settle_time_s = float(self.declare_parameter('pickup_settle_time_s', 1.5).value)
        self.pickup_xy_tolerance = float(self.declare_parameter('pickup_xy_tolerance', 0.08).value)
        self.pickup_z_tolerance = float(self.declare_parameter('pickup_z_tolerance', 0.08).value)
        self.pickup_tip_speed_tolerance = float(self.declare_parameter('pickup_tip_speed_tolerance', 0.20).value)
        self.attach_wait_timeout_s = float(self.declare_parameter('attach_wait_timeout_s', 8.0).value)

        # Magnet-tip feedback. The desired quad-body reference is generated by
        # commanding the current quad position plus a correction based on the
        # measured magnet-tip error. This is deliberately gentle to avoid adding
        # swing during final descent.
        self.tip_xy_feedback_gain = float(self.declare_parameter('tip_xy_feedback_gain', 0.75).value)
        self.tip_xy_feedback_max_step = float(self.declare_parameter('tip_xy_feedback_max_step', 0.35).value)
        self.use_current_quad_tip_offset_for_z = bool(
            self.declare_parameter('use_current_quad_tip_offset_for_z', True).value
        )
        self.nominal_magnet_drop_below_quad = float(
            self.declare_parameter('nominal_magnet_drop_below_quad', 0.50).value
        )

        # Optional automatic start. This is useful for isolated testing; later a
        # mission supervisor can trigger this planner explicitly.
        self.auto_start = bool(self.declare_parameter('auto_start', True).value)

        self.dt = 1.0 / max(1.0, self.publish_rate_hz)
        self.horizon_samples = max(2, int(math.ceil(self.horizon_seconds / self.dt)))
        self.intercept_time = max(self.dt, float(self.intercept_time))

        self.state = APPROACH_ABOVE_PICKUP if self.auto_start else APPROACH_ABOVE_PICKUP
        self.state_entry_time = time.time()
        self.condition_start_time: Optional[float] = None
        self.descend_start_tip_z: Optional[float] = None
        self.lift_start_tip_z: Optional[float] = None
        self.latched_object_xy: Optional[np.ndarray] = None
        self.last_magnet_command: Optional[str] = None
        self.object_attached = False
        self.attach_wait_start_time: Optional[float] = None

        self.drone: Optional[TimedVector] = None
        self.magnet_tip: Optional[TimedVector] = None
        self.object: Optional[TimedVector] = None
        self._last_tip_position: Optional[np.ndarray] = None
        self._last_tip_stamp: Optional[float] = None
        self._last_object_position: Optional[np.ndarray] = None
        self._last_object_stamp: Optional[float] = None
        self.last_desired_tip = np.array([float('nan'), float('nan'), float('nan')], dtype=float)
        self.last_quad_target = np.array([float('nan'), float('nan'), float('nan')], dtype=float)

        self.create_subscription(MotionCaptureState, self.drone_state_topic, self.drone_callback, 10)
        self.create_subscription(PoseStamped, self.magnet_tip_pose_topic, self.magnet_tip_callback, 10)
        self.create_subscription(PoseArray, self.object_pose_topic, self.object_pose_callback, 10)
        self.create_subscription(Bool, self.object_attached_topic, self.object_attached_callback, 10)

        self.reference_pub = self.create_publisher(MultiDOFJointTrajectory, self.reference_topic, 1)
        self.magnet_command_pub = self.create_publisher(String, self.magnet_command_topic, 5)
        self.state_pub = self.create_publisher(String, self.state_topic, 5)
        self.marker_pub = self.create_publisher(MarkerArray, self.marker_topic, 1)

        self.timer = self.create_timer(self.dt, self.timer_callback)
        self.get_logger().info(
            'PickupMissionPlanner started. '
            f'Publishing reference to {self.reference_topic}; '
            f'object={self.object_pose_topic}[{self.object_index}], magnet_tip={self.magnet_tip_pose_topic}. '
            'Run only one reference planner at a time.'
        )

    def drone_callback(self, msg: MotionCaptureState) -> None:
        now = time.time()
        self.drone = TimedVector(_pos_from_motion_state(msg), _vel_from_motion_state(msg), now)

    def magnet_tip_callback(self, msg: PoseStamped) -> None:
        now = time.time()
        pos = _pos_from_pose(msg.pose)
        if self._last_tip_position is None or self._last_tip_stamp is None:
            vel = np.zeros(3, dtype=float)
        else:
            dt = max(1e-6, now - self._last_tip_stamp)
            vel = (pos - self._last_tip_position) / dt
        self._last_tip_position = pos.copy()
        self._last_tip_stamp = now
        self.magnet_tip = TimedVector(pos, vel, now)

    def object_pose_callback(self, msg: PoseArray) -> None:
        if self.object_index < 0:
            index = len(msg.poses) + self.object_index
        else:
            index = self.object_index
        if index < 0 or index >= len(msg.poses):
            self.get_logger().warn(
                f'object_index={self.object_index} out of range for {self.object_pose_topic}; length={len(msg.poses)}',
                throttle_duration_sec=2.0,
            )
            return
        now = time.time()
        pos = _pos_from_pose(msg.poses[index])
        if self._last_object_position is None or self._last_object_stamp is None:
            vel = np.zeros(3, dtype=float)
        else:
            dt = max(1e-6, now - self._last_object_stamp)
            vel = (pos - self._last_object_position) / dt
        self._last_object_position = pos.copy()
        self._last_object_stamp = now
        self.object = TimedVector(pos, vel, now)

    def object_attached_callback(self, msg: Bool) -> None:
        self.object_attached = bool(msg.data)

    def set_state(self, new_state: str) -> None:
        if new_state == self.state:
            return
        self.get_logger().info(f'{self.state} -> {new_state}')
        self.state = new_state
        self.state_entry_time = time.time()
        self.condition_start_time = None
        if new_state == DESCEND_TO_PICKUP:
            self.descend_start_tip_z = None
        if new_state == LIFT_OBJECT:
            self.lift_start_tip_z = None
        if new_state == MAGNET_ATTACH_WAIT:
            self.attach_wait_start_time = time.time()

    def publish_magnet_command(self, command: str) -> None:
        command = command.upper()
        if self.last_magnet_command == command:
            return
        msg = String()
        msg.data = command
        self.magnet_command_pub.publish(msg)
        self.last_magnet_command = command
        self.get_logger().info(f'Magnet command: {command}')

    def inputs_ready(self) -> Tuple[bool, str]:
        now = time.time()
        for name, value in [('drone', self.drone), ('magnet_tip', self.magnet_tip), ('object', self.object)]:
            if value is None:
                return False, f'waiting for {name}'
            if now - value.stamp > self.input_timeout_s:
                return False, f'{name} stale ({now - value.stamp:.2f}s)'
        return True, 'inputs ready'

    def object_target_xy(self) -> np.ndarray:
        if self.latched_object_xy is not None and self.state in (DESCEND_TO_PICKUP, MAGNET_ATTACH_WAIT, LIFT_OBJECT, PICKUP_COMPLETE):
            return self.latched_object_xy.copy()
        assert self.object is not None
        return self.object.position[:2].copy()

    def desired_tip_for_state(self) -> np.ndarray:
        assert self.object is not None
        now = time.time()
        obj = self.object.position.copy()
        xy = self.object_target_xy()

        if self.state in (APPROACH_ABOVE_PICKUP, SETTLE_ABOVE_PICKUP):
            z = obj[2] + self.pickup_approach_clearance

        elif self.state == DESCEND_TO_PICKUP:
            if self.descend_start_tip_z is None:
                assert self.magnet_tip is not None
                self.descend_start_tip_z = float(self.magnet_tip.position[2])
                self.latched_object_xy = obj[:2].copy()
            elapsed = now - self.state_entry_time
            target_z = obj[2] + self.pickup_attach_clearance
            z = max(target_z, self.descend_start_tip_z - self.pickup_descent_speed * elapsed)

        elif self.state == MAGNET_ATTACH_WAIT:
            z = obj[2] + self.pickup_attach_clearance

        elif self.state in (LIFT_OBJECT, PICKUP_COMPLETE):
            if self.lift_start_tip_z is None:
                assert self.magnet_tip is not None
                self.lift_start_tip_z = float(self.magnet_tip.position[2])
            elapsed = now - self.state_entry_time
            target_z = obj[2] + self.pickup_lift_height
            z = min(target_z, self.lift_start_tip_z + self.pickup_lift_speed * elapsed)

        else:
            z = obj[2] + self.pickup_approach_clearance

        return np.array([xy[0], xy[1], z], dtype=float)

    def quad_target_from_tip_target(self, desired_tip: np.ndarray) -> np.ndarray:
        """Convert a desired world-frame magnet-tip target to a quad-body target.

        The old join planner publishes quad-body references, not magnet-tip
        references.  This function keeps that convention.  In simulation we
        have a measured world-frame magnet tip, so the safest conversion is to
        preserve the current quad-to-tip offset:

            quad_target = desired_tip - (tip_world - quad_world)

        That means if the magnet is currently hanging 0.55 m below the quad, a
        desired tip target at z=0.35 becomes a quad target at z=0.90.  It also
        compensates small horizontal swing without making /join_planner/reference
        a magnet-tip trajectory by accident.
        """
        assert self.drone is not None and self.magnet_tip is not None
        quad = self.drone.position.copy()
        tip = self.magnet_tip.position.copy()

        quad_to_tip = tip - quad
        quad_target = desired_tip - quad_to_tip

        if not self.use_current_quad_tip_offset_for_z:
            quad_target[2] = desired_tip[2] + self.nominal_magnet_drop_below_quad

        quad_target[2] = max(self.min_reference_z, quad_target[2])
        return quad_target

    def update_state_machine(self, desired_tip: np.ndarray) -> None:
        assert self.magnet_tip is not None and self.object is not None
        now = time.time()
        tip = self.magnet_tip.position
        tip_vel = self.magnet_tip.velocity
        tip_error = desired_tip - tip
        tip_error_xy = float(np.linalg.norm(tip_error[:2]))
        tip_error_z = abs(float(tip_error[2]))
        tip_speed = float(np.linalg.norm(tip_vel))
        tip_xy_speed = float(np.linalg.norm(tip_vel[:2]))

        if self.state in (APPROACH_ABOVE_PICKUP, SETTLE_ABOVE_PICKUP):
            self.publish_magnet_command('OFF')

        if self.state == APPROACH_ABOVE_PICKUP:
            good = (
                tip_error_xy <= self.pickup_xy_tolerance
                and tip_error_z <= self.pickup_z_tolerance
                and tip_xy_speed <= self.pickup_tip_speed_tolerance
            )
            if good:
                if self.condition_start_time is None:
                    self.condition_start_time = now
                if now - self.condition_start_time >= 0.5:
                    self.set_state(SETTLE_ABOVE_PICKUP)
            else:
                self.condition_start_time = None

        elif self.state == SETTLE_ABOVE_PICKUP:
            good = (
                tip_error_xy <= self.pickup_xy_tolerance
                and tip_error_z <= self.pickup_z_tolerance
                and tip_speed <= self.pickup_tip_speed_tolerance
            )
            if good:
                if self.condition_start_time is None:
                    self.condition_start_time = now
                if now - self.condition_start_time >= self.pickup_settle_time_s:
                    self.set_state(DESCEND_TO_PICKUP)
                    self.publish_magnet_command('ON')
            else:
                self.condition_start_time = None

        elif self.state == DESCEND_TO_PICKUP:
            self.publish_magnet_command('ON')
            target_z = self.object.position[2] + self.pickup_attach_clearance
            if desired_tip[2] <= target_z + 1e-3:
                self.set_state(MAGNET_ATTACH_WAIT)

        elif self.state == MAGNET_ATTACH_WAIT:
            self.publish_magnet_command('ON')
            if self.object_attached:
                self.set_state(LIFT_OBJECT)
            elif self.attach_wait_start_time is not None and now - self.attach_wait_start_time > self.attach_wait_timeout_s:
                self.get_logger().warn(
                    'Still waiting for object attachment. Check /magnet/attachment_state distance/radius/speed.',
                    throttle_duration_sec=2.0,
                )

        elif self.state == LIFT_OBJECT:
            self.publish_magnet_command('ON')
            target_z = self.object.position[2] + self.pickup_lift_height
            if tip[2] >= target_z - self.pickup_z_tolerance:
                self.set_state(PICKUP_COMPLETE)

        elif self.state == PICKUP_COMPLETE:
            self.publish_magnet_command('ON')

    def clamp_velocity(self, v: np.ndarray) -> np.ndarray:
        speed = float(np.linalg.norm(v))
        if speed > self.max_reference_speed and speed > 1e-9:
            return v * (self.max_reference_speed / speed)
        return v

    def hermite_segment(
        self,
        t: float,
        T: float,
        p0: np.ndarray,
        v0: np.ndarray,
        pT: np.ndarray,
        vT: np.ndarray,
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Cubic Hermite segment matching online_join_planner's reference style."""
        T = max(float(T), self.dt)
        s = float(np.clip(t / T, 0.0, 1.0))

        h00 = 2.0 * s**3 - 3.0 * s**2 + 1.0
        h10 = s**3 - 2.0 * s**2 + s
        h01 = -2.0 * s**3 + 3.0 * s**2
        h11 = s**3 - s**2

        p = h00 * p0 + h10 * T * v0 + h01 * pT + h11 * T * vT

        dh00 = (6.0 * s**2 - 6.0 * s) / T
        dh10 = 3.0 * s**2 - 4.0 * s + 1.0
        dh01 = (-6.0 * s**2 + 6.0 * s) / T
        dh11 = 3.0 * s**2 - 2.0 * s
        v = dh00 * p0 + dh10 * v0 + dh01 * pT + dh11 * vT

        ddh00 = (12.0 * s - 6.0) / (T * T)
        ddh10 = (6.0 * s - 4.0) / T
        ddh01 = (6.0 - 12.0 * s) / (T * T)
        ddh11 = (6.0 * s - 2.0) / T
        a = ddh00 * p0 + ddh10 * v0 + ddh01 * pT + ddh11 * vT
        return p, v, a

    def _append_reference_point(
        self,
        msg: MultiDOFJointTrajectory,
        pos: np.ndarray,
        vel: np.ndarray,
        acc: np.ndarray,
        t: float,
    ) -> None:
        """Append one quad-body reference point with the old planner's message contract."""
        transform = Transform()
        transform.translation.x = float(pos[0])
        transform.translation.y = float(pos[1])
        transform.translation.z = float(pos[2])
        transform.rotation.x = 0.0
        transform.rotation.y = 0.0
        transform.rotation.z = 0.0
        transform.rotation.w = 1.0

        velocity = Twist()
        velocity.linear.x = float(vel[0])
        velocity.linear.y = float(vel[1])
        velocity.linear.z = float(vel[2])

        acceleration = Twist()
        acceleration.linear.x = float(acc[0])
        acceleration.linear.y = float(acc[1])
        acceleration.linear.z = float(acc[2])

        point = MultiDOFJointTrajectoryPoint()
        point.transforms.append(transform)
        point.velocities.append(velocity)
        point.accelerations.append(acceleration)
        total_nanoseconds = int(round(t * 1e9))
        point.time_from_start.sec = total_nanoseconds // 1_000_000_000
        point.time_from_start.nanosec = total_nanoseconds % 1_000_000_000
        msg.points.append(point)

    def make_reference(self, quad_target: np.ndarray) -> Tuple[MultiDOFJointTrajectory, np.ndarray]:
        """Build the rolling quad-body MultiDOFJointTrajectory.

        During APPROACH_ABOVE_PICKUP, step_reference_in_approach=True publishes
        all horizon points at the current quad target.  This is deliberately
        less elegant than the Hermite path, but it is useful for checking
        whether the controller is only responding to the first few horizon
        points.  Later pickup phases still use the Hermite intercept path.
        """
        assert self.drone is not None
        p0 = self.drone.position.copy()
        v0 = self.clamp_velocity(self.drone.velocity.copy())
        pT = quad_target.copy()
        vT = np.zeros(3, dtype=float)

        if pT[2] < self.min_reference_z:
            pT[2] = self.min_reference_z
            vT[2] = 0.0

        msg = MultiDOFJointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.joint_names = ['drone']

        positions = np.zeros((self.horizon_samples, 3), dtype=float)

        # Diagnostic step mode for the initial above-object approach.  This makes
        # point[0] equal to the intended quad target instead of the measured
        # current pose, so the controller cannot ignore the climb by repeatedly
        # consuming only the first/early horizon points.
        if self.step_reference_in_approach and self.state == APPROACH_ABOVE_PICKUP:
            for i in range(self.horizon_samples):
                t = i * self.dt
                pos = pT.copy()
                vel = np.zeros(3, dtype=float)
                acc = np.zeros(3, dtype=float)
                positions[i, :] = pos
                self._append_reference_point(msg, pos, vel, acc, t)
            return msg, positions

        # Normal smooth reference for all later pickup phases.
        # Do not ask the vehicle to cover more than max_reference_speed allows
        # over the intercept window.
        max_travel = max(0.05, self.max_reference_speed * self.intercept_time)
        delta = _clip_norm(pT - p0, max_travel)
        pT = p0 + delta
        if pT[2] < self.min_reference_z:
            pT[2] = self.min_reference_z
            vT[2] = 0.0

        for i in range(self.horizon_samples):
            t = i * self.dt
            if t <= self.intercept_time:
                pos, vel, acc = self.hermite_segment(t, self.intercept_time, p0, v0, pT, vT)
            else:
                pos = pT.copy()
                vel = np.zeros(3, dtype=float)
                acc = np.zeros(3, dtype=float)

            if pos[2] < self.min_reference_z:
                pos[2] = self.min_reference_z
                vel[2] = 0.0
                acc[2] = 0.0

            vel = self.clamp_velocity(vel)
            positions[i, :] = pos
            self._append_reference_point(msg, pos, vel, acc, t)

        return msg, positions

    def publish_state_text(self, desired_tip: np.ndarray, quad_target: np.ndarray) -> None:
        assert self.magnet_tip is not None and self.object is not None
        tip_error = desired_tip - self.magnet_tip.position
        text = (
            f'{self.state}\n'
            f'tip_err_xy={np.linalg.norm(tip_error[:2]):.3f} m, tip_err_z={tip_error[2]:+.3f} m\n'
            f'tip_speed={np.linalg.norm(self.magnet_tip.velocity):.3f} m/s, attached={self.object_attached}\n'
            f'object=({self.object.position[0]:+.2f},{self.object.position[1]:+.2f},{self.object.position[2]:+.2f}) '
            f'des_tip=({desired_tip[0]:+.2f},{desired_tip[1]:+.2f},{desired_tip[2]:+.2f}) '
            f'quad_ref=({quad_target[0]:+.2f},{quad_target[1]:+.2f},{quad_target[2]:+.2f})'
        )
        msg = String()
        msg.data = text
        self.state_pub.publish(msg)

    def publish_markers(self, desired_tip: np.ndarray, quad_target: np.ndarray, positions: np.ndarray) -> None:
        assert self.object is not None and self.magnet_tip is not None
        now = self.get_clock().now().to_msg()
        markers = MarkerArray()

        def sphere(marker_id: int, ns: str, pos: np.ndarray, scale: float) -> Marker:
            m = Marker()
            m.header.frame_id = self.frame_id
            m.header.stamp = now
            m.ns = ns
            m.id = marker_id
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position = _make_point(pos)
            m.pose.orientation.w = 1.0
            m.scale.x = scale
            m.scale.y = scale
            m.scale.z = scale
            m.color.a = 0.8
            return m

        object_m = sphere(0, 'pickup_object', self.object.position, 0.10)
        object_m.color.r = 0.9
        object_m.color.g = 0.1
        object_m.color.b = 0.1
        markers.markers.append(object_m)

        desired_m = sphere(1, 'desired_magnet_tip', desired_tip, 0.07)
        desired_m.color.r = 0.1
        desired_m.color.g = 0.9
        desired_m.color.b = 0.2
        markers.markers.append(desired_m)

        actual_m = sphere(2, 'actual_magnet_tip', self.magnet_tip.position, 0.06)
        actual_m.color.r = 0.1
        actual_m.color.g = 0.5
        actual_m.color.b = 1.0
        markers.markers.append(actual_m)

        quad_m = sphere(3, 'quad_reference_target', quad_target, 0.08)
        quad_m.color.r = 1.0
        quad_m.color.g = 0.6
        quad_m.color.b = 0.0
        markers.markers.append(quad_m)

        path = Marker()
        path.header.frame_id = self.frame_id
        path.header.stamp = now
        path.ns = 'pickup_reference_path'
        path.id = 4
        path.type = Marker.LINE_STRIP
        path.action = Marker.ADD
        path.pose.orientation.w = 1.0
        path.scale.x = 0.025
        path.color.r = 1.0
        path.color.g = 0.6
        path.color.b = 0.0
        path.color.a = 0.9
        path.points = [_make_point(p) for p in positions]
        markers.markers.append(path)

        # In step-reference mode the real reference path collapses to a point.
        # Keep a visible debug line from the current quad pose to the target so
        # RViz still shows where the pickup planner is trying to go.
        if (
            self.drone is not None
            and positions.shape[0] > 0
            and float(np.linalg.norm(positions[-1] - positions[0])) < 1e-6
        ):
            debug_line = Marker()
            debug_line.header.frame_id = self.frame_id
            debug_line.header.stamp = now
            debug_line.ns = 'pickup_debug_current_to_target'
            debug_line.id = 5
            debug_line.type = Marker.LINE_STRIP
            debug_line.action = Marker.ADD
            debug_line.pose.orientation.w = 1.0
            debug_line.scale.x = 0.018
            debug_line.color.r = 1.0
            debug_line.color.g = 0.8
            debug_line.color.b = 0.0
            debug_line.color.a = 0.75
            debug_line.points = [_make_point(self.drone.position), _make_point(quad_target)]
            markers.markers.append(debug_line)

        def text(marker_id: int, ns: str, pos: np.ndarray, label: str) -> Marker:
            m = Marker()
            m.header.frame_id = self.frame_id
            m.header.stamp = now
            m.ns = ns
            m.id = marker_id
            m.type = Marker.TEXT_VIEW_FACING
            m.action = Marker.ADD
            m.pose.position = _make_point(pos)
            m.pose.position.z += 0.15
            m.pose.orientation.w = 1.0
            m.scale.z = 0.12
            m.color.r = 1.0
            m.color.g = 1.0
            m.color.b = 1.0
            m.color.a = 0.95
            m.text = label
            return m

        markers.markers.append(text(20, 'pickup_label_state', quad_target, self.state))
        markers.markers.append(text(21, 'pickup_label_object', self.object.position, 'pickup object'))
        markers.markers.append(text(22, 'pickup_label_tip', self.magnet_tip.position, 'magnet tip'))
        markers.markers.append(text(23, 'pickup_label_quad_ref', quad_target, 'quad ref'))

        self.marker_pub.publish(markers)

    def timer_callback(self) -> None:
        ready, reason = self.inputs_ready()
        if not ready:
            msg = String()
            msg.data = f'WAITING_FOR_INPUTS: {reason}'
            self.state_pub.publish(msg)
            return

        desired_tip = self.desired_tip_for_state()
        quad_target = self.quad_target_from_tip_target(desired_tip)
        self.last_desired_tip = desired_tip.copy()
        self.last_quad_target = quad_target.copy()

        self.update_state_machine(desired_tip)
        # Recompute after possible state transition so z ramps start cleanly.
        desired_tip = self.desired_tip_for_state()
        quad_target = self.quad_target_from_tip_target(desired_tip)

        ref_msg, positions = self.make_reference(quad_target)
        self.reference_pub.publish(ref_msg)
        self.publish_state_text(desired_tip, quad_target)
        self.publish_markers(desired_tip, quad_target, positions)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PickupMissionPlanner()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""
Fake cooperative transport world publisher.

This node is intended as a development stand-in for your groupmate's drone/payload
system. It publishes:

  - a moving large-payload pose/twist,
  - a rigidly attached attachment-point pose/twist,
  - fake other-drone obstacle poses/states,
  - RViz markers generated from the same internal state.

For C1F.5 cooperative planning, the fake obstacles publish authoritative committed
cubic B-splines as well as live MotionCaptureState samples. The C++ planner uses the
commitments for future motion and the live state to verify tracking. MarkerArray output
is only for visualisation/debugging.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
from typing import Deque, List, Sequence, Tuple
from bisect import bisect_right

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Header
from geometry_msgs.msg import (
    Point,
    Pose,
    PoseArray,
    PoseStamped,
    Quaternion,
    Twist,
    TwistStamped,
)
from visualization_msgs.msg import Marker, MarkerArray
from interfaces.msg import MotionCaptureState, CommittedTrajectory, CubicTrajectoryPiece
from tejen_mission.cooperative_trajectory import (
    RingNetGeometry,
    RigidBodyState as CooperativeRigidBodyState,
    rigid_body_offset_state,
)
from tejen_mission.m2_fleet_commitment import commitment_sequence_seed


# M2 preview assignment only. Physical vehicle/plate ownership is deliberately not
# finalized by this constant; it exists so the diagnostic environment has one
# explicit source of truth for the three already-attached companion previews.
THREE_ATTACHED_COMPANION_PLATES = (3, 6, 9)


@dataclass
class SharedSpline:
    knots: List[float]
    control_points: List[Tuple[float, float, float]]
    valid_from_s: float
    valid_until_s: float


def _open_uniform_knots(t0: float, t1: float, num_segments: int, degree: int = 3) -> List[float]:
    if num_segments < 1 or not t1 > t0:
        raise ValueError('invalid shared-trajectory knot request')
    num_control_points = num_segments + degree
    knot_count = num_control_points + degree + 1
    knots = [t0] * knot_count
    for i in range(knot_count - degree - 1, knot_count):
        knots[i] = t1
    if num_segments > 1:
        dt = (t1 - t0) / float(num_segments)
        for i in range(1, num_segments):
            knots[degree + i] = t0 + dt * float(i)
    return knots


def _deboor_eval(control_points: Sequence[Tuple[float, float, float]], knots: Sequence[float], degree: int, t: float) -> Tuple[float, float, float]:
    n = len(control_points) - 1
    if n < degree or len(knots) != len(control_points) + degree + 1:
        raise ValueError('invalid B-spline shape')
    lo = knots[degree]
    hi = knots[n + 1]
    tt = min(max(float(t), lo), hi)
    if tt >= hi:
        span = n
    else:
        span = max(degree, min(n, bisect_right(knots, tt) - 1))
    d = [list(control_points[span - degree + j]) for j in range(degree + 1)]
    for r in range(1, degree + 1):
        for j in range(degree, r - 1, -1):
            i = span - degree + j
            denom = knots[i + degree - r + 1] - knots[i]
            alpha = 0.0 if abs(denom) < 1e-15 else (tt - knots[i]) / denom
            for axis in range(3):
                d[j][axis] = (1.0 - alpha) * d[j - 1][axis] + alpha * d[j][axis]
    return tuple(d[degree])


def _derivative_spline(control_points: Sequence[Tuple[float, float, float]], knots: Sequence[float], degree: int) -> Tuple[List[Tuple[float, float, float]], List[float], int]:
    if degree <= 0:
        return [(0.0, 0.0, 0.0)], [knots[0], knots[-1]], 0
    derived: List[Tuple[float, float, float]] = []
    for i in range(len(control_points) - 1):
        denom = knots[i + degree + 1] - knots[i + 1]
        scale = 0.0 if abs(denom) < 1e-15 else degree / denom
        derived.append(tuple(scale * (control_points[i + 1][axis] - control_points[i][axis]) for axis in range(3)))
    return derived, list(knots[1:-1]), degree - 1


def _eval_spline_state(spline: SharedSpline, t: float) -> Tuple[Tuple[float, float, float], Tuple[float, float, float]]:
    position = _deboor_eval(spline.control_points, spline.knots, 3, t)
    velocity_cp, velocity_knots, velocity_degree = _derivative_spline(spline.control_points, spline.knots, 3)
    velocity = _deboor_eval(velocity_cp, velocity_knots, velocity_degree, t)
    return position, velocity


@dataclass
class RigidBodyState:
    x: float
    y: float
    z: float
    yaw: float
    vx: float
    vy: float
    vz: float
    yaw_rate: float


@dataclass
class ObstacleState:
    name: str
    state: RigidBodyState
    radius: float
    safety_radius: float


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(0.5 * yaw)
    q.w = math.cos(0.5 * yaw)
    return q


def rotate_yaw(offset: Tuple[float, float, float], yaw: float) -> Tuple[float, float, float]:
    ox, oy, oz = offset
    c = math.cos(yaw)
    s = math.sin(yaw)
    return (
        c * ox - s * oy,
        s * ox + c * oy,
        oz,
    )


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class FakeCooperativeTransportWorld(Node):
    def __init__(self) -> None:
        super().__init__('fake_cooperative_transport_world')

        # Topic parameters
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('payload_pose_topic', '/fake_payload/pose')
        self.declare_parameter('payload_twist_topic', '/fake_payload/twist')
        self.declare_parameter('attachment_pose_topic', '/fake_attachment_point/pose')
        self.declare_parameter('attachment_twist_topic', '/fake_attachment_point/twist')
        self.declare_parameter('obstacle_pose_array_topic', '/fake_obstacles/poses')
        self.declare_parameter('obstacle_state_topic_prefix', '/fake_obstacles/drone')
        self.declare_parameter('marker_topic', '/fake_world/markers')
        self.declare_parameter('committed_trajectory_topic_prefix', '/fake_obstacles/drone')
        # The moving drop basket is the large fake payload itself. Publishing
        # its future through the same commitment message keeps C++ transit
        # collision avoidance independent of simulator-only ground truth.
        self.declare_parameter('payload_committed_trajectory_topic', '/fake_payload/committed_trajectory')

        # Timing / trajectory parameters
        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('trajectory_type', 'circle')  # stationary, circle, line, figure8
        self.declare_parameter('center_x', 0.0)
        self.declare_parameter('center_y', 0.0)
        self.declare_parameter('center_z', 1.5)
        self.declare_parameter('radius', 0.5)
        self.declare_parameter('line_amplitude', 0.6)
        self.declare_parameter('omega', 0.25)  # rad/s for circle/line/figure8
        self.declare_parameter('z_amplitude', 0.0)
        self.declare_parameter('z_omega', 0.25)

        # Payload yaw behaviour
        self.declare_parameter('payload_yaw_mode', 'fixed')  # fixed, spin, tangent, oscillate
        self.declare_parameter('payload_yaw0', 0.0)
        self.declare_parameter('payload_yaw_rate', 0.0)
        self.declare_parameter('payload_yaw_amplitude', 0.0)
        self.declare_parameter('payload_yaw_omega', 0.25)

        # Physical ring/net geometry. The ring frame origin is the centre of the
        # steel attachment-plate plane. Plate 0 is on +x_R and is the current
        # single-drone M1 reattachment target; C++ delivery still targets the
        # true ring centre via /fake_payload/committed_trajectory.
        self.declare_parameter('ring_plate_count', 12)
        self.declare_parameter('ring_plate_pitch_diameter_m', 0.50)
        self.declare_parameter('ring_plate_diameter_m', 0.06)
        self.declare_parameter('ring_attachment_plate_index', 0)
        self.declare_parameter('ring_collision_outer_diameter_m', 0.56)
        self.declare_parameter('ring_collision_top_offset_m', 0.0)
        self.declare_parameter('ring_collision_bottom_offset_m', -0.27)
        # Visual-only approximations of the printed truss and steel plate thickness.
        # Collision checking uses the explicit conservative envelope above.
        self.declare_parameter('ring_structure_height_m', 0.07)
        self.declare_parameter('ring_plate_visual_thickness_m', 0.006)

        # Fake other drones / moving obstacles expressed in the large-payload body frame.
        # These are modelling obstacles for the planner, not physics objects.
        self.declare_parameter('enable_fake_obstacles', True)
        # Set to 'custom' to use the explicit fake_drone_offsets_* lists below.
        # Built-in scenarios make repeatable obstacle-avoidance tests easier:
        #   custom, clear, transport_default, blocking_center, blocking_left,
        #   blocking_right, narrow_gap, three_attached
        self.declare_parameter('obstacle_scenario', 'custom')
        self.declare_parameter('scenario_drone_offset_z', 0.50)
        # M2-preview geometry only. These parameters describe the ideal straight
        # cable from a ring attachment plate to the companion's upper endpoint.
        # They do not enable cooperative cable collision geometry.
        self.declare_parameter('three_attached_cable_length_m', 0.50)
        self.declare_parameter('three_attached_cable_angle_deg', 45.0)
        self.declare_parameter('fake_drone_count', 2)
        self.declare_parameter('fake_drone_offsets_x', [-0.30, -0.30])
        self.declare_parameter('fake_drone_offsets_y', [0.35, -0.35])
        self.declare_parameter('fake_drone_offsets_z', [0.20, 0.20])
        self.declare_parameter('fake_drone_radius', 0.12)
        self.declare_parameter('fake_drone_safety_radius', 0.35)
        self.declare_parameter('publish_shared_trajectories', True)
        self.declare_parameter('shared_trajectory_duration_s', 180.0)
        self.declare_parameter('shared_trajectory_control_interval_s', 0.50)
        # Optional rolling renewal for finite-horizon cooperative commitments.
        # Zero preserves the historical one-shot fake-world behaviour.
        self.declare_parameter('shared_trajectory_refresh_period_s', 0.0)

        # Marker sizing. Physical ring/net markers are visual only; the C++
        # planner publishes the authoritative collision envelope separately.
        self.declare_parameter('attachment_marker_diameter', 0.08)
        self.declare_parameter('trail_seconds', 20.0)

        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.payload_pose_topic = self.get_parameter('payload_pose_topic').get_parameter_value().string_value
        self.payload_twist_topic = self.get_parameter('payload_twist_topic').get_parameter_value().string_value
        self.attachment_pose_topic = self.get_parameter('attachment_pose_topic').get_parameter_value().string_value
        self.attachment_twist_topic = self.get_parameter('attachment_twist_topic').get_parameter_value().string_value
        self.obstacle_pose_array_topic = self.get_parameter('obstacle_pose_array_topic').get_parameter_value().string_value
        self.obstacle_state_topic_prefix = self.get_parameter('obstacle_state_topic_prefix').get_parameter_value().string_value
        self.marker_topic = self.get_parameter('marker_topic').get_parameter_value().string_value
        self.committed_trajectory_topic_prefix = self.get_parameter('committed_trajectory_topic_prefix').get_parameter_value().string_value
        self.payload_committed_trajectory_topic = self.get_parameter(
            'payload_committed_trajectory_topic'
        ).get_parameter_value().string_value

        self.ring_geometry = RingNetGeometry(
            plate_count=int(self.get_parameter('ring_plate_count').value),
            plate_pitch_diameter_m=self.get_float_param('ring_plate_pitch_diameter_m'),
            plate_diameter_m=self.get_float_param('ring_plate_diameter_m'),
            collision_outer_diameter_m=self.get_float_param('ring_collision_outer_diameter_m'),
            collision_top_offset_m=self.get_float_param('ring_collision_top_offset_m'),
            collision_bottom_offset_m=self.get_float_param('ring_collision_bottom_offset_m'),
        )
        self.ring_attachment_plate_index = int(
            self.get_parameter('ring_attachment_plate_index').value
        )
        # Validate the configured attachment index immediately.
        self.ring_geometry.plate_body_offset(self.ring_attachment_plate_index)

        self.payload_pose_pub = self.create_publisher(PoseStamped, self.payload_pose_topic, 10)
        self.payload_twist_pub = self.create_publisher(TwistStamped, self.payload_twist_topic, 10)
        self.attachment_pose_pub = self.create_publisher(PoseStamped, self.attachment_pose_topic, 10)
        self.attachment_twist_pub = self.create_publisher(TwistStamped, self.attachment_twist_topic, 10)
        self.obstacle_pose_array_pub = self.create_publisher(PoseArray, self.obstacle_pose_array_topic, 10)
        self.marker_pub = self.create_publisher(MarkerArray, self.marker_topic, 10)

        self.fake_drone_count = len(self.get_fake_drone_offsets())
        self.obstacle_state_pubs = [
            self.create_publisher(
                MotionCaptureState,
                f'{self.obstacle_state_topic_prefix}_{i}/state',
                10,
            )
            for i in range(self.fake_drone_count)
        ]
        trajectory_qos = QoSProfile(depth=1)
        trajectory_qos.reliability = ReliabilityPolicy.RELIABLE
        trajectory_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.committed_trajectory_pubs = [
            self.create_publisher(
                CommittedTrajectory,
                f'{self.committed_trajectory_topic_prefix}_{i}/committed_trajectory',
                trajectory_qos,
            )
            for i in range(self.fake_drone_count)
        ]
        self.payload_committed_trajectory_pub = self.create_publisher(
            CommittedTrajectory,
            self.payload_committed_trajectory_topic,
            trajectory_qos,
        )

        self.start_time = self.get_clock().now()
        self.start_time_s = self.start_time.nanoseconds * 1e-9
        self.shared_obstacle_splines: List[SharedSpline] = []
        self.shared_payload_spline: SharedSpline | None = None
        self.shared_trajectory_sequence = commitment_sequence_seed()
        self.last_shared_commitment_publish_s: float | None = None
        if self.get_bool_param('publish_shared_trajectories'):
            self.refresh_shared_commitments(self.start_time_s)
        self.last_payload_state: RigidBodyState | None = None
        self.last_attachment_xyz: Tuple[float, float, float] | None = None
        self.last_obstacle_xyzs: List[Tuple[float, float, float]] | None = None
        self.last_time_sec: float | None = None
        self.attachment_trail: Deque[Point] = deque()

        publish_rate_hz = max(1.0, self.get_float_param('publish_rate_hz'))
        self.timer_period = 1.0 / publish_rate_hz
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.get_logger().info('Fake cooperative transport world publisher started.')
        self.get_logger().info(f'Frame: {self.frame_id}')
        self.get_logger().info(f'Payload pose topic: {self.payload_pose_topic}')
        self.get_logger().info(f'Attachment point topic: {self.attachment_pose_topic}')
        self.get_logger().info(f'Obstacle scenario: {self.get_obstacle_scenario()}')
        self.get_logger().info(f'Obstacle offsets: {self.get_fake_drone_offsets()}')
        self.get_logger().info(f'Obstacle pose array topic: {self.obstacle_pose_array_topic}')
        for i in range(self.fake_drone_count):
            self.get_logger().info(f'Obstacle {i} state topic: {self.obstacle_state_topic_prefix}_{i}/state')
            if self.get_bool_param('publish_shared_trajectories'):
                self.get_logger().info(f'Obstacle {i} commitment topic: {self.committed_trajectory_topic_prefix}_{i}/committed_trajectory')
        if self.get_bool_param('publish_shared_trajectories'):
            self.get_logger().info(
                f'Payload/basket commitment topic: {self.payload_committed_trajectory_topic}'
            )
        self.get_logger().info(f'Marker topic: {self.marker_topic}')

    def now_sec_since_start(self) -> float:
        now = self.get_clock().now()
        return (now - self.start_time).nanoseconds * 1e-9

    def get_float_param(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def get_bool_param(self, name: str) -> bool:
        return bool(self.get_parameter(name).value)

    def get_string_param(self, name: str) -> str:
        return str(self.get_parameter(name).value)

    def get_float_list_param(self, name: str) -> List[float]:
        value = self.get_parameter(name).value
        if isinstance(value, (list, tuple)):
            return [float(v) for v in value]
        try:
            return [float(v) for v in list(value)]
        except TypeError:
            return [float(value)]

    def compute_payload_state(self, t: float) -> RigidBodyState:
        trajectory_type = self.get_string_param('trajectory_type').lower()
        cx = self.get_float_param('center_x')
        cy = self.get_float_param('center_y')
        cz = self.get_float_param('center_z')
        radius = self.get_float_param('radius')
        line_amp = self.get_float_param('line_amplitude')
        omega = self.get_float_param('omega')
        z_amp = self.get_float_param('z_amplitude')
        z_omega = self.get_float_param('z_omega')

        if trajectory_type == 'stationary':
            x, y = cx, cy
            vx, vy = 0.0, 0.0
        elif trajectory_type == 'line':
            x = cx + line_amp * math.sin(omega * t)
            y = cy
            vx = line_amp * omega * math.cos(omega * t)
            vy = 0.0
        elif trajectory_type == 'figure8':
            x = cx + radius * math.sin(omega * t)
            y = cy + 0.5 * radius * math.sin(2.0 * omega * t)
            vx = radius * omega * math.cos(omega * t)
            vy = radius * omega * math.cos(2.0 * omega * t)
        else:  # circle default
            x = cx + radius * math.cos(omega * t)
            y = cy + radius * math.sin(omega * t)
            vx = -radius * omega * math.sin(omega * t)
            vy = radius * omega * math.cos(omega * t)

        z = cz + z_amp * math.sin(z_omega * t)
        vz = z_amp * z_omega * math.cos(z_omega * t)

        yaw0 = self.get_float_param('payload_yaw0')
        yaw_mode = self.get_string_param('payload_yaw_mode').lower()
        if yaw_mode == 'spin':
            yaw_rate = self.get_float_param('payload_yaw_rate')
            yaw = yaw0 + yaw_rate * t
        elif yaw_mode == 'tangent':
            if abs(vx) + abs(vy) > 1e-9:
                yaw = math.atan2(vy, vx)
            else:
                yaw = yaw0
            yaw_rate = 0.0  # finite-diff is used for offset points when needed
        elif yaw_mode == 'oscillate':
            yaw_amp = self.get_float_param('payload_yaw_amplitude')
            yaw_omega = self.get_float_param('payload_yaw_omega')
            yaw = yaw0 + yaw_amp * math.sin(yaw_omega * t)
            yaw_rate = yaw_amp * yaw_omega * math.cos(yaw_omega * t)
        else:
            yaw = yaw0
            yaw_rate = 0.0

        return RigidBodyState(x=x, y=y, z=z, yaw=yaw, vx=vx, vy=vy, vz=vz, yaw_rate=yaw_rate)

    def compute_offset_xyz(self, payload_state: RigidBodyState, offset: Tuple[float, float, float]) -> Tuple[float, float, float]:
        ox, oy, oz = rotate_yaw(offset, payload_state.yaw)
        return (
            payload_state.x + ox,
            payload_state.y + oy,
            payload_state.z + oz,
        )

    def compute_attachment_xyz(self, payload_state: RigidBodyState) -> Tuple[float, float, float]:
        # Current M1 is level/yaw-only, but the body-frame plate definition comes
        # from the shared RingNetGeometry contract used by future 3-D transforms.
        offset = self.ring_geometry.plate_body_offset(self.ring_attachment_plate_index)
        return self.compute_offset_xyz(payload_state, tuple(float(v) for v in offset))

    def get_obstacle_scenario(self) -> str:
        return self.get_string_param('obstacle_scenario').strip().lower()

    def get_fake_drone_offsets(self) -> List[Tuple[float, float, float]]:
        """Return fake drone offsets in the large-payload body frame.

        The 'custom' scenario uses the explicit fake_drone_offsets_* parameter
        lists. Other scenarios are deterministic presets intended for repeatable
        obstacle-avoidance tests.
        """
        scenario = self.get_obstacle_scenario()

        attachment_offset = self.ring_geometry.plate_body_offset(
            self.ring_attachment_plate_index
        )
        attach_x = float(attachment_offset[0])
        attach_y = float(attachment_offset[1])
        z = self.get_float_param('scenario_drone_offset_z')

        # The far obstacle keeps the topic/publisher layout stable while staying
        # irrelevant for most tests.
        far_back_left = (-1.20, 0.80, z)
        far_back_right = (-1.20, -0.80, z)

        if scenario == 'clear':
            return [far_back_left, far_back_right]

        if scenario == 'transport_default':
            return [(-0.30, 0.35, z), (-0.30, -0.35, z)]

        if scenario == 'blocking_center':
            return [(attach_x, attach_y, z), far_back_right]

        if scenario == 'blocking_left':
            return [(attach_x, attach_y + 0.25, z), far_back_right]

        if scenario == 'blocking_right':
            return [(attach_x, attach_y - 0.25, z), far_back_left]

        if scenario == 'narrow_gap':
            # Two drones flank the desired attachment/quad-body line. The 0.47 m
            # centre offset is deliberately only a few centimetres outside the
            # current conservative ego-magnet/cooperative-body terminal envelope:
            # tight enough to stress the planner without making the rendezvous
            # structurally impossible by construction.
            return [(attach_x, attach_y + 0.47, z), (attach_x, attach_y - 0.47, z)]

        if scenario == 'three_attached':
            # M2-preview environment only: ego is associated with plate 0 and the
            # three companion bodies correspond to plates 3/6/9.  Approximate the
            # visible tensioned configuration using an ideal straight 0.5 m cable
            # pulled radially outward at a configurable angle (45 deg in the case).
            # This positions body centres/upper endpoints for the preview only; the
            # current ROS transfer backend still uses body-only companion collision
            # geometry and does NOT model these diagonal cables yet.
            cable_length = self.get_float_param('three_attached_cable_length_m')
            cable_angle = math.radians(
                self.get_float_param('three_attached_cable_angle_deg')
            )
            offsets: List[Tuple[float, float, float]] = []
            for plate_index in THREE_ATTACHED_COMPANION_PLATES:
                endpoint = self.ring_geometry.radially_tensioned_endpoint_body(
                    plate_index,
                    cable_length_m=cable_length,
                    angle_from_vertical_rad=cable_angle,
                )
                offsets.append(tuple(float(v) for v in endpoint))
            return offsets

        if scenario != 'custom':
            self.get_logger().warn(
                f"Unknown obstacle_scenario='{scenario}', falling back to custom offsets."
            )

        count = max(0, int(self.get_parameter('fake_drone_count').value))
        xs = self.get_float_list_param('fake_drone_offsets_x')
        ys = self.get_float_list_param('fake_drone_offsets_y')
        zs = self.get_float_list_param('fake_drone_offsets_z')

        offsets: List[Tuple[float, float, float]] = []
        for i in range(count):
            # If a list is shorter than the requested count, keep reusing the last entry.
            ix = min(i, len(xs) - 1)
            iy = min(i, len(ys) - 1)
            iz = min(i, len(zs) - 1)
            offsets.append((xs[ix], ys[iy], zs[iz]))
        return offsets

    def desired_obstacle_xyz(self, t: float, offset: Tuple[float, float, float]) -> Tuple[float, float, float]:
        return self.compute_offset_xyz(self.compute_payload_state(t), offset)

    def _build_shared_spline_for_offset(
        self, offset: Tuple[float, float, float], valid_from_s: float
    ) -> SharedSpline:
        duration = max(5.0, self.get_float_param('shared_trajectory_duration_s'))
        control_interval = max(0.05, self.get_float_param('shared_trajectory_control_interval_s'))
        num_segments = max(1, int(math.ceil(duration / control_interval)))
        t0 = float(valid_from_s)
        t1 = t0 + duration
        knots = _open_uniform_knots(t0, t1, num_segments, 3)
        num_control_points = num_segments + 3
        control_points: List[Tuple[float, float, float]] = []
        for i in range(num_control_points):
            # Greville abscissae closely approximate the analytic scripted path
            # while keeping one exact C2 commitment for the planner.  Evaluate
            # against the ORIGINAL node time origin so rolling commitment windows
            # continue the same motion phase instead of restarting the circle.
            greville_abs = (knots[i + 1] + knots[i + 2] + knots[i + 3]) / 3.0
            control_points.append(
                self.desired_obstacle_xyz(greville_abs - self.start_time_s, offset)
            )
        return SharedSpline(list(knots), control_points, t0, t1)

    def build_shared_obstacle_splines(self, valid_from_s: float) -> List[SharedSpline]:
        return [
            self._build_shared_spline_for_offset(offset, valid_from_s)
            for offset in self.get_fake_drone_offsets()
        ]

    def build_shared_payload_spline(self, valid_from_s: float) -> SharedSpline:
        # Zero body-frame offset is exactly the large moving payload/basket
        # centre. C++ uses this only during TRANSIT_TO_DROP_POINT authority.
        return self._build_shared_spline_for_offset((0.0, 0.0, 0.0), valid_from_s)

    def _commitment_message(
        self, spline: SharedSpline, vehicle_id: str, stamp, sequence: int
    ) -> CommittedTrajectory:
        msg = CommittedTrajectory()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.vehicle_id = vehicle_id
        msg.sequence = int(sequence)
        msg.terminal_hold = False
        piece = CubicTrajectoryPiece()
        piece.valid_from_s = spline.valid_from_s
        piece.valid_until_s = spline.valid_until_s
        piece.knots = list(spline.knots)
        piece.control_points = []
        for xyz in spline.control_points:
            point = Point()
            point.x, point.y, point.z = xyz
            piece.control_points.append(point)
        msg.pieces = [piece]
        return msg

    def publish_shared_commitments(self) -> None:
        stamp = self.get_clock().now().to_msg()
        self.shared_trajectory_sequence += 1
        sequence = self.shared_trajectory_sequence
        for i, spline in enumerate(self.shared_obstacle_splines):
            if i >= len(self.committed_trajectory_pubs):
                continue
            self.committed_trajectory_pubs[i].publish(
                self._commitment_message(spline, f'fake_drone_{i}', stamp, sequence)
            )
        if self.shared_payload_spline is not None:
            self.payload_committed_trajectory_pub.publish(
                self._commitment_message(
                    self.shared_payload_spline, 'fake_drop_basket', stamp, sequence
                )
            )

    def refresh_shared_commitments(self, valid_from_s: float) -> None:
        self.shared_obstacle_splines = self.build_shared_obstacle_splines(valid_from_s)
        self.shared_payload_spline = self.build_shared_payload_spline(valid_from_s)
        self.publish_shared_commitments()
        self.last_shared_commitment_publish_s = float(valid_from_s)

    def maybe_refresh_shared_commitments(self, now_abs_s: float) -> None:
        if not self.get_bool_param('publish_shared_trajectories'):
            return
        refresh_period_s = self.get_float_param('shared_trajectory_refresh_period_s')
        if refresh_period_s <= 0.0:
            return
        if (
            self.last_shared_commitment_publish_s is None
            or now_abs_s - self.last_shared_commitment_publish_s >= refresh_period_s
        ):
            self.refresh_shared_commitments(now_abs_s)

    def compute_obstacle_states(self, t: float, payload_state: RigidBodyState) -> List[ObstacleState]:
        if not self.get_bool_param('enable_fake_obstacles'):
            return []

        offsets = self.get_fake_drone_offsets()
        radius = self.get_float_param('fake_drone_radius')
        safety_radius = self.get_float_param('fake_drone_safety_radius')

        yaw_rate = payload_state.yaw_rate
        if self.last_payload_state is not None and self.last_time_sec is not None:
            dt = max(1e-6, t - self.last_time_sec)
            yaw_rate = normalize_angle(payload_state.yaw - self.last_payload_state.yaw) / dt

        if self.get_obstacle_scenario() == 'three_attached':
            # The already-attached companions are fixed points in the ring/body
            # frame.  Evaluate their live state directly from the instantaneous
            # basket transform so their visible/current motion is exactly rigid:
            # translation with the basket plus omega x r tangential velocity.
            # Their advertised future remains the shared B-spline generated from
            # the same body-frame offsets by desired_obstacle_xyz().
            rigid_payload = CooperativeRigidBodyState(
                position=(payload_state.x, payload_state.y, payload_state.z),
                velocity=(payload_state.vx, payload_state.vy, payload_state.vz),
                acceleration=(0.0, 0.0, 0.0),
                yaw=payload_state.yaw,
                yaw_rate=yaw_rate,
                yaw_acceleration=0.0,
            )
            rigid_states = [rigid_body_offset_state(rigid_payload, offset) for offset in offsets]
            obstacle_xyzs = [tuple(float(value) for value in state.position) for state in rigid_states]
            vels = [tuple(float(value) for value in state.velocity) for state in rigid_states]
        elif self.shared_obstacle_splines and len(self.shared_obstacle_splines) == len(offsets):
            now_abs_s = self.start_time_s + t
            states = [_eval_spline_state(spline, now_abs_s) for spline in self.shared_obstacle_splines]
            obstacle_xyzs = [state[0] for state in states]
            vels = [state[1] for state in states]
        else:
            obstacle_xyzs = [self.compute_offset_xyz(payload_state, offset) for offset in offsets]
            if (
                self.last_obstacle_xyzs is not None
                and self.last_time_sec is not None
                and len(self.last_obstacle_xyzs) == len(obstacle_xyzs)
            ):
                dt = max(1e-6, t - self.last_time_sec)
                vels = [
                    (
                        (xyz[0] - last_xyz[0]) / dt,
                        (xyz[1] - last_xyz[1]) / dt,
                        (xyz[2] - last_xyz[2]) / dt,
                    )
                    for xyz, last_xyz in zip(obstacle_xyzs, self.last_obstacle_xyzs)
                ]
            else:
                vels = [(payload_state.vx, payload_state.vy, payload_state.vz) for _ in obstacle_xyzs]

        obstacles: List[ObstacleState] = []
        for i, (xyz, vel) in enumerate(zip(obstacle_xyzs, vels)):
            body_state = RigidBodyState(
                x=xyz[0],
                y=xyz[1],
                z=xyz[2],
                yaw=payload_state.yaw,
                vx=vel[0],
                vy=vel[1],
                vz=vel[2],
                yaw_rate=yaw_rate,
            )
            obstacles.append(
                ObstacleState(
                    name=f'fake_drone_{i}',
                    state=body_state,
                    radius=radius,
                    safety_radius=safety_radius,
                )
            )
        return obstacles

    def make_pose(self, x: float, y: float, z: float, yaw: float) -> Pose:
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation = yaw_to_quaternion(yaw)
        return pose

    def make_pose_stamped(self, x: float, y: float, z: float, yaw: float, stamp) -> PoseStamped:
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.pose = self.make_pose(x, y, z, yaw)
        return msg

    def make_twist_stamped(self, vx: float, vy: float, vz: float, yaw_rate: float, stamp) -> TwistStamped:
        msg = TwistStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.twist.linear.x = vx
        msg.twist.linear.y = vy
        msg.twist.linear.z = vz
        msg.twist.angular.z = yaw_rate
        return msg

    def make_motion_capture_state(self, obstacle: ObstacleState, stamp) -> MotionCaptureState:
        state = obstacle.state
        msg = MotionCaptureState()
        msg.header = Header()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id

        msg.pose = self.make_pose(state.x, state.y, state.z, state.yaw)

        msg.twist = Twist()
        msg.twist.linear.x = state.vx
        msg.twist.linear.y = state.vy
        msg.twist.linear.z = state.vz
        msg.twist.angular.z = state.yaw_rate
        return msg

    def make_obstacle_pose_array(self, obstacles: Sequence[ObstacleState], stamp) -> PoseArray:
        msg = PoseArray()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.poses = [
            self.make_pose(obs.state.x, obs.state.y, obs.state.z, obs.state.yaw)
            for obs in obstacles
        ]
        return msg

    def finite_difference_attachment_twist(
        self,
        t: float,
        payload_state: RigidBodyState,
        attachment_xyz: Tuple[float, float, float],
    ) -> Tuple[float, float, float, float]:
        if self.last_attachment_xyz is None or self.last_payload_state is None or self.last_time_sec is None:
            return payload_state.vx, payload_state.vy, payload_state.vz, payload_state.yaw_rate

        dt = max(1e-6, t - self.last_time_sec)
        vx = (attachment_xyz[0] - self.last_attachment_xyz[0]) / dt
        vy = (attachment_xyz[1] - self.last_attachment_xyz[1]) / dt
        vz = (attachment_xyz[2] - self.last_attachment_xyz[2]) / dt
        yaw_rate = normalize_angle(payload_state.yaw - self.last_payload_state.yaw) / dt
        return vx, vy, vz, yaw_rate

    def make_marker_common(self, marker_id: int, ns: str, marker_type: int, stamp) -> Marker:
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.frame_id
        marker.ns = ns
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0
        return marker

    def append_ring_markers(
        self,
        markers: MarkerArray,
        payload_state: RigidBodyState,
        stamp,
    ) -> None:
        """Draw a physical-ish ring/truss/net model from the shared ring geometry.

        These are ground-truth visualization markers only. The C++ planner publishes
        its conservative collision envelope separately from the collision-translated
        committed trajectory.
        """

        plate_count = int(self.ring_geometry.plate_count)
        pitch_radius = self.ring_geometry.plate_pitch_radius_m
        plate_diameter = float(self.ring_geometry.plate_diameter_m)
        plate_thickness = max(0.001, self.get_float_param('ring_plate_visual_thickness_m'))
        structure_height = max(0.001, self.get_float_param('ring_structure_height_m'))

        # Approximate the printed truss with tangent boxes around the pitch circle.
        # Keep its radial width within the steel-plate outer diameter.
        truss_radial_width = min(0.05, plate_diameter)
        tangent_length = 2.0 * pitch_radius * math.sin(math.pi / plate_count)
        for i in range(plate_count):
            theta = 2.0 * math.pi * i / plate_count
            local = (
                pitch_radius * math.cos(theta),
                pitch_radius * math.sin(theta),
                -0.5 * structure_height,
            )
            wx, wy, wz = self.compute_offset_xyz(payload_state, local)
            truss = self.make_marker_common(1000 + i, 'fake_ring_truss', Marker.CUBE, stamp)
            truss.pose = self.make_pose(wx, wy, wz, payload_state.yaw + theta + 0.5 * math.pi)
            truss.scale.x = tangent_length
            truss.scale.y = truss_radial_width
            truss.scale.z = structure_height
            truss.color.r = 0.25
            truss.color.g = 0.25
            truss.color.b = 0.30
            truss.color.a = 0.90
            markers.markers.append(truss)

        # Steel attachment plates. Plate 0 is highlighted because it is the active
        # single-drone M1 reattachment point; M2 can later assign 0/3/6/9 online.
        for i in range(plate_count):
            local = self.ring_geometry.plate_body_offset(i)
            wx, wy, wz = self.compute_offset_xyz(
                payload_state, tuple(float(v) for v in local)
            )
            plate = self.make_marker_common(1100 + i, 'fake_ring_plates', Marker.CYLINDER, stamp)
            plate.pose = self.make_pose(wx, wy, wz, payload_state.yaw)
            plate.scale.x = plate_diameter
            plate.scale.y = plate_diameter
            plate.scale.z = plate_thickness
            if i == self.ring_attachment_plate_index:
                plate.color.r = 0.15
                plate.color.g = 0.95
                plate.color.b = 0.25
            else:
                plate.color.r = 0.65
                plate.color.g = 0.68
                plate.color.b = 0.72
            plate.color.a = 1.0
            markers.markers.append(plate)

        # Visual-only net: twelve parabolic-ish radial strands from the bottom of
        # the printed truss to the conservative lower net depth. A deformable net
        # is intentionally not part of the planner dynamics; its conservative AABB
        # is the planner-facing representation.
        net_bottom = float(self.ring_geometry.collision_bottom_offset_m)
        net_edge_z = -structure_height
        segments_per_strand = 8
        net = self.make_marker_common(1200, 'fake_ring_net', Marker.LINE_LIST, stamp)
        net.scale.x = 0.006
        net.color.r = 0.75
        net.color.g = 0.75
        net.color.b = 0.78
        net.color.a = 0.75
        for i in range(plate_count):
            theta = 2.0 * math.pi * i / plate_count
            previous = None
            for j in range(segments_per_strand + 1):
                s = j / float(segments_per_strand)
                radial_fraction = 1.0 - s
                radius = pitch_radius * radial_fraction
                # z(r) is quadratic in radius: shallow at the rim and deepest at centre.
                z_local = net_bottom + (net_edge_z - net_bottom) * radial_fraction**2
                local = (radius * math.cos(theta), radius * math.sin(theta), z_local)
                world = self.compute_offset_xyz(payload_state, local)
                point = Point()
                point.x, point.y, point.z = world
                if previous is not None:
                    net.points.append(previous)
                    net.points.append(point)
                previous = point
        markers.markers.append(net)

    def make_markers(
        self,
        payload_state: RigidBodyState,
        payload_pose: PoseStamped,
        attachment_pose: PoseStamped,
        obstacles: Sequence[ObstacleState],
        stamp,
    ) -> MarkerArray:
        markers = MarkerArray()

        self.append_ring_markers(markers, payload_state, stamp)

        attachment_marker = self.make_marker_common(1, 'fake_attachment_point', Marker.SPHERE, stamp)
        attachment_marker.pose = attachment_pose.pose
        diameter = self.get_float_param('attachment_marker_diameter')
        attachment_marker.scale.x = diameter
        attachment_marker.scale.y = diameter
        attachment_marker.scale.z = diameter
        attachment_marker.color.r = 0.0
        attachment_marker.color.g = 1.0
        attachment_marker.color.b = 0.2
        attachment_marker.color.a = 1.0
        markers.markers.append(attachment_marker)

        label_marker = self.make_marker_common(2, 'fake_attachment_label', Marker.TEXT_VIEW_FACING, stamp)
        label_marker.pose = attachment_pose.pose
        label_marker.pose.position.z += 0.12
        label_marker.scale.z = 0.12
        label_marker.color.r = 0.0
        label_marker.color.g = 1.0
        label_marker.color.b = 0.2
        label_marker.color.a = 1.0
        label_marker.text = f'attach plate {self.ring_attachment_plate_index}'
        markers.markers.append(label_marker)

        if len(self.attachment_trail) >= 2:
            trail_marker = self.make_marker_common(3, 'fake_attachment_trail', Marker.LINE_STRIP, stamp)
            trail_marker.points = list(self.attachment_trail)
            trail_marker.scale.x = 0.015
            trail_marker.color.r = 0.0
            trail_marker.color.g = 1.0
            trail_marker.color.b = 0.2
            trail_marker.color.a = 0.6
            markers.markers.append(trail_marker)

        for i, obstacle in enumerate(obstacles):
            state = obstacle.state
            pose = self.make_pose(state.x, state.y, state.z, state.yaw)

            drone_marker = self.make_marker_common(100 + i, 'fake_obstacle_ground_truth', Marker.SPHERE, stamp)
            drone_marker.pose = pose
            drone_marker.scale.x = 2.0 * obstacle.radius
            drone_marker.scale.y = 2.0 * obstacle.radius
            drone_marker.scale.z = 2.0 * obstacle.radius
            drone_marker.color.r = 1.0
            drone_marker.color.g = 0.35
            drone_marker.color.b = 0.0
            drone_marker.color.a = 0.95
            markers.markers.append(drone_marker)

            safety_marker = self.make_marker_common(200 + i, 'fake_obstacle_legacy_visual_safety_radius', Marker.SPHERE, stamp)
            safety_marker.pose = pose
            safety_marker.scale.x = 2.0 * obstacle.safety_radius
            safety_marker.scale.y = 2.0 * obstacle.safety_radius
            safety_marker.scale.z = 2.0 * obstacle.safety_radius
            safety_marker.color.r = 1.0
            safety_marker.color.g = 0.35
            safety_marker.color.b = 0.0
            safety_marker.color.a = 0.16
            markers.markers.append(safety_marker)

            label_marker = self.make_marker_common(300 + i, 'fake_obstacle_label', Marker.TEXT_VIEW_FACING, stamp)
            label_marker.pose = pose
            label_marker.pose.position.z += obstacle.safety_radius + 0.08
            label_marker.scale.z = 0.10
            label_marker.color.r = 1.0
            label_marker.color.g = 0.55
            label_marker.color.b = 0.0
            label_marker.color.a = 1.0
            label_marker.text = f'drone_{i}'
            markers.markers.append(label_marker)

        return markers

    def update_trail(self, attachment_pose: PoseStamped) -> None:
        p = Point()
        p.x = attachment_pose.pose.position.x
        p.y = attachment_pose.pose.position.y
        p.z = attachment_pose.pose.position.z
        self.attachment_trail.append(p)

        trail_seconds = max(0.0, self.get_float_param('trail_seconds'))
        max_points = max(2, int(trail_seconds / self.timer_period))
        while len(self.attachment_trail) > max_points:
            self.attachment_trail.popleft()

    def timer_callback(self) -> None:
        t = self.now_sec_since_start()
        self.maybe_refresh_shared_commitments(self.start_time_s + t)
        stamp = self.get_clock().now().to_msg()

        payload_state = self.compute_payload_state(t)
        attachment_xyz = self.compute_attachment_xyz(payload_state)
        obstacles = self.compute_obstacle_states(t, payload_state)
        avx, avy, avz, ayaw_rate = self.finite_difference_attachment_twist(t, payload_state, attachment_xyz)

        payload_pose = self.make_pose_stamped(
            payload_state.x,
            payload_state.y,
            payload_state.z,
            payload_state.yaw,
            stamp,
        )
        payload_twist = self.make_twist_stamped(
            payload_state.vx,
            payload_state.vy,
            payload_state.vz,
            payload_state.yaw_rate,
            stamp,
        )

        attachment_pose = self.make_pose_stamped(
            attachment_xyz[0],
            attachment_xyz[1],
            attachment_xyz[2],
            payload_state.yaw,
            stamp,
        )
        attachment_twist = self.make_twist_stamped(avx, avy, avz, ayaw_rate, stamp)
        obstacle_pose_array = self.make_obstacle_pose_array(obstacles, stamp)

        self.update_trail(attachment_pose)
        markers = self.make_markers(payload_state, payload_pose, attachment_pose, obstacles, stamp)

        self.payload_pose_pub.publish(payload_pose)
        self.payload_twist_pub.publish(payload_twist)
        self.attachment_pose_pub.publish(attachment_pose)
        self.attachment_twist_pub.publish(attachment_twist)
        self.obstacle_pose_array_pub.publish(obstacle_pose_array)
        for i, obstacle in enumerate(obstacles):
            if i < len(self.obstacle_state_pubs):
                self.obstacle_state_pubs[i].publish(self.make_motion_capture_state(obstacle, stamp))
        self.marker_pub.publish(markers)

        self.last_payload_state = payload_state
        self.last_attachment_xyz = attachment_xyz
        self.last_obstacle_xyzs = [(obs.state.x, obs.state.y, obs.state.z) for obs in obstacles]
        self.last_time_sec = t


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FakeCooperativeTransportWorld()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""
Fake cooperative transport world publisher.

This node is intended as a development stand-in for your groupmate's drone/payload
system. It publishes:

  - a moving large-payload pose/twist,
  - a rigidly attached attachment-point pose/twist,
  - fake other-drone obstacle poses/states,
  - RViz markers generated from the same internal state.

The planner should consume the PoseStamped/TwistStamped and obstacle state topics.
The MarkerArray is only for visualisation/debugging.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
from typing import Deque, List, Sequence, Tuple

import rclpy
from rclpy.node import Node
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
from interfaces.msg import MotionCaptureState


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

        # Attachment point expressed in the large-payload body frame.
        self.declare_parameter('attachment_offset_x', 0.30)
        self.declare_parameter('attachment_offset_y', 0.00)
        self.declare_parameter('attachment_offset_z', -0.10)

        # Fake other drones / moving obstacles expressed in the large-payload body frame.
        # These are modelling obstacles for the planner, not physics objects.
        self.declare_parameter('enable_fake_obstacles', True)
        # Set to 'custom' to use the explicit fake_drone_offsets_* lists below.
        # Built-in scenarios make repeatable obstacle-avoidance tests easier:
        #   custom, clear, transport_default, blocking_center, blocking_left,
        #   blocking_right, narrow_gap
        self.declare_parameter('obstacle_scenario', 'custom')
        self.declare_parameter('scenario_drone_offset_z', 0.50)
        self.declare_parameter('fake_drone_count', 2)
        self.declare_parameter('fake_drone_offsets_x', [-0.30, -0.30])
        self.declare_parameter('fake_drone_offsets_y', [0.35, -0.35])
        self.declare_parameter('fake_drone_offsets_z', [0.20, 0.20])
        self.declare_parameter('fake_drone_radius', 0.12)
        self.declare_parameter('fake_drone_safety_radius', 0.35)

        # Marker sizing
        self.declare_parameter('payload_size_x', 0.80)
        self.declare_parameter('payload_size_y', 0.30)
        self.declare_parameter('payload_size_z', 0.12)
        self.declare_parameter('payload_safety_margin_xy', 0.20)
        self.declare_parameter('payload_safety_margin_z', 0.10)
        self.declare_parameter('show_payload_safety_box', True)
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

        self.start_time = self.get_clock().now()
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
        offset = (
            self.get_float_param('attachment_offset_x'),
            self.get_float_param('attachment_offset_y'),
            self.get_float_param('attachment_offset_z'),
        )
        return self.compute_offset_xyz(payload_state, offset)

    def get_obstacle_scenario(self) -> str:
        return self.get_string_param('obstacle_scenario').strip().lower()

    def get_fake_drone_offsets(self) -> List[Tuple[float, float, float]]:
        """Return fake drone offsets in the large-payload body frame.

        The 'custom' scenario uses the explicit fake_drone_offsets_* parameter
        lists. Other scenarios are deterministic presets intended for repeatable
        obstacle-avoidance tests.
        """
        scenario = self.get_obstacle_scenario()

        attach_x = self.get_float_param('attachment_offset_x')
        attach_y = self.get_float_param('attachment_offset_y')
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
            # Two drones flank the desired attachment/quad-body line. With the
            # default 0.35 m safety radius this creates a deliberately tight
            # corridor for the deformed reference to thread or reject.
            return [(attach_x, attach_y + 0.42, z), (attach_x, attach_y - 0.42, z)]

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

    def compute_obstacle_states(self, t: float, payload_state: RigidBodyState) -> List[ObstacleState]:
        if not self.get_bool_param('enable_fake_obstacles'):
            return []

        offsets = self.get_fake_drone_offsets()
        obstacle_xyzs = [self.compute_offset_xyz(payload_state, offset) for offset in offsets]
        radius = self.get_float_param('fake_drone_radius')
        safety_radius = self.get_float_param('fake_drone_safety_radius')

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

        yaw_rate = payload_state.yaw_rate
        if self.last_payload_state is not None and self.last_time_sec is not None:
            dt = max(1e-6, t - self.last_time_sec)
            yaw_rate = normalize_angle(payload_state.yaw - self.last_payload_state.yaw) / dt

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

    def make_markers(
        self,
        payload_pose: PoseStamped,
        attachment_pose: PoseStamped,
        obstacles: Sequence[ObstacleState],
        stamp,
    ) -> MarkerArray:
        markers = MarkerArray()

        payload_marker = self.make_marker_common(0, 'fake_payload', Marker.CUBE, stamp)
        payload_marker.pose = payload_pose.pose
        payload_marker.scale.x = self.get_float_param('payload_size_x')
        payload_marker.scale.y = self.get_float_param('payload_size_y')
        payload_marker.scale.z = self.get_float_param('payload_size_z')
        payload_marker.color.r = 0.1
        payload_marker.color.g = 0.5
        payload_marker.color.b = 1.0
        payload_marker.color.a = 0.35
        markers.markers.append(payload_marker)

        if self.get_bool_param('show_payload_safety_box'):
            safety_marker = self.make_marker_common(4, 'fake_payload_safety_volume', Marker.CUBE, stamp)
            safety_marker.pose = payload_pose.pose
            margin_xy = self.get_float_param('payload_safety_margin_xy')
            margin_z = self.get_float_param('payload_safety_margin_z')
            safety_marker.scale.x = self.get_float_param('payload_size_x') + 2.0 * margin_xy
            safety_marker.scale.y = self.get_float_param('payload_size_y') + 2.0 * margin_xy
            safety_marker.scale.z = self.get_float_param('payload_size_z') + 2.0 * margin_z
            safety_marker.color.r = 0.1
            safety_marker.color.g = 0.5
            safety_marker.color.b = 1.0
            safety_marker.color.a = 0.12
            markers.markers.append(safety_marker)

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
        label_marker.text = 'attach'
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

            drone_marker = self.make_marker_common(100 + i, 'fake_obstacle_drone', Marker.SPHERE, stamp)
            drone_marker.pose = pose
            drone_marker.scale.x = 2.0 * obstacle.radius
            drone_marker.scale.y = 2.0 * obstacle.radius
            drone_marker.scale.z = 2.0 * obstacle.radius
            drone_marker.color.r = 1.0
            drone_marker.color.g = 0.35
            drone_marker.color.b = 0.0
            drone_marker.color.a = 0.95
            markers.markers.append(drone_marker)

            safety_marker = self.make_marker_common(200 + i, 'fake_obstacle_safety_radius', Marker.SPHERE, stamp)
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
        markers = self.make_markers(payload_pose, attachment_pose, obstacles, stamp)

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

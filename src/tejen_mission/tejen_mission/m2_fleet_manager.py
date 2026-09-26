#!/usr/bin/env python3
"""M2C ground-only fleet registry, frozen assignment, and readiness authority.

This node intentionally does not sequence flight.  It proves that four physical
vehicle identities, four truthful stationary commitments, and the measured ring
pose are mutually consistent before M2D is allowed to add motion.
"""

from __future__ import annotations

import json
import math
import time
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseArray, PoseStamped
from interfaces.msg import CommittedTrajectory, MotionCaptureState
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker, MarkerArray

from .cooperative_trajectory import RingNetGeometry
from .m2a_attachment_runtime import quaternion_xyzw_to_rotation
from .m2_fleet_assignment import (
    FleetAssignmentConfig,
    FleetAssignmentLatch,
    FleetVehicleState,
    attached_hold_body_target_world,
    choose_fleet_assignment,
)
from .m2_fleet_commitment import validate_stationary_cubic_hold
from .m2_fleet_topics import canonical_vehicle_topics
from .m2c_ground_readiness import ContinuousSettleGate


DRONE_IDS = (0, 1, 2, 3)


def _yaw_from_quaternion(q) -> float:
    x, y, z, w = float(q.x), float(q.y), float(q.z), float(q.w)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1e-12:
        raise ValueError("zero quaternion")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _position(pose) -> np.ndarray:
    return np.array(
        [pose.position.x, pose.position.y, pose.position.z], dtype=float
    )


def _quaternion(pose) -> np.ndarray:
    return np.array(
        [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w],
        dtype=float,
    )


class M2CFleetManager(Node):
    def __init__(self) -> None:
        super().__init__("m2c_fleet_manager")
        self.vehicle_state_timeout_s = max(
            0.05, float(self.declare_parameter("vehicle_state_timeout_s", 0.25).value)
        )
        self.ring_state_timeout_s = max(
            0.05, float(self.declare_parameter("ring_state_timeout_s", 0.25).value)
        )
        self.commitment_timeout_s = max(
            0.05, float(self.declare_parameter("commitment_timeout_s", 0.50).value)
        )
        self.joint_truth_timeout_s = max(
            0.05, float(self.declare_parameter("joint_truth_timeout_s", 0.25).value)
        )
        self.ground_settle_dwell_s = max(
            0.0, float(self.declare_parameter("ground_settle_dwell_s", 1.0).value)
        )
        self.ring_pose_topic = str(
            self.declare_parameter("ring_pose_topic", "/model/payload_model/pose").value
        )
        self.ring_pose_index = int(self.declare_parameter("ring_pose_index", 1).value)
        self.frame_id = str(self.declare_parameter("frame_id", "map").value)
        rigid_ids = tuple(
            int(value)
            for value in self.declare_parameter(
                "mocap_rigid_body_ids", [100, 101, 102, 103]
            ).value
        )
        if len(rigid_ids) != 4 or len(set(rigid_ids)) != 4 or min(rigid_ids) < 0:
            raise ValueError("mocap_rigid_body_ids must contain four unique non-negative IDs")
        self.rigid_ids = rigid_ids

        self.geometry = RingNetGeometry()
        self.assignment_config = FleetAssignmentConfig(
            state_timeout_s=self.vehicle_state_timeout_s,
            ring_state_timeout_s=self.ring_state_timeout_s,
            tether_length_m=0.475,
            tether_anchor_body=np.array([0.0, 0.0, -0.04], dtype=float),
            attached_hold_angle_deg=15.0,
        )
        self.assignment_latch = FleetAssignmentLatch()
        self.vehicle_states: dict[int, FleetVehicleState] = {}
        self.initial_yaws: dict[int, float] = {}
        self.commitments: dict[int, tuple[CommittedTrajectory, float]] = {}
        self.vehicle_speeds: dict[int, float] = {}
        self.magnet_floor_samples: dict[int, tuple[float, float]] = {}
        self.joint_detached: dict[int, bool] = {}
        self.joint_truth_stamps: dict[int, float] = {}
        self.arming_states: dict[int, bool] = {}
        self.ground_settle_gate = ContinuousSettleGate(self.ground_settle_dwell_s)
        self.last_commitment_sequence: dict[int, int] = {}
        self.commitment_rejections: dict[int, int] = {i: 0 for i in DRONE_IDS}
        self.ring_position: Optional[np.ndarray] = None
        self.ring_rotation: Optional[np.ndarray] = None
        self.ring_stamp_s: Optional[float] = None
        self.pass_latched = False
        self.last_status_state = "WAITING_FOR_FLEET"

        for drone_id in DRONE_IDS:
            topics = canonical_vehicle_topics(drone_id)
            self.create_subscription(
                MotionCaptureState,
                topics["motion_capture_state"],
                lambda msg, i=drone_id: self._vehicle_callback(i, msg),
                10,
            )
            self.create_subscription(
                CommittedTrajectory,
                topics["committed_trajectory"],
                lambda msg, i=drone_id: self._commitment_callback(i, msg),
                10,
            )
            self.create_subscription(
                PoseStamped,
                topics["magnet_tip_pose"],
                lambda msg, i=drone_id: self._magnet_tip_callback(i, msg),
                10,
            )
            self.create_subscription(
                Bool,
                topics["joint_state"],
                lambda msg, i=drone_id: self._joint_truth_callback(i, msg),
                10,
            )
            self.create_subscription(
                Bool,
                topics["arming_state_feedback"],
                lambda msg, i=drone_id: self._arming_callback(i, msg),
                10,
            )

        self.create_subscription(PoseArray, self.ring_pose_topic, self._ring_callback, 20)
        self.status_pub = self.create_publisher(String, "/m2c/status", 10)
        assignment_qos = QoSProfile(depth=1)
        assignment_qos.reliability = ReliabilityPolicy.RELIABLE
        assignment_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.assignment_pub = self.create_publisher(
            String, "/m2c/assignment", assignment_qos
        )
        self.ready_pub = self.create_publisher(Bool, "/m2c/ready", 10)
        self.markers_pub = self.create_publisher(MarkerArray, "/m2c/assignment_markers", 5)
        self.timer = self.create_timer(0.10, self._tick)
        self.get_logger().info(
            "M2C fleet manager started: four vehicles, ground-only, frozen 72-case assignment."
        )

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _vehicle_callback(self, drone_id: int, msg: MotionCaptureState) -> None:
        now = self._now_s()
        try:
            position = _position(msg.pose)
            if not np.all(np.isfinite(position)):
                return
            yaw = _yaw_from_quaternion(msg.pose.orientation)
        except (ValueError, TypeError):
            return
        if drone_id not in self.initial_yaws:
            self.initial_yaws[drone_id] = yaw
        velocity = np.array(
            [msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z], dtype=float
        )
        self.vehicle_speeds[drone_id] = (
            float(np.linalg.norm(velocity)) if np.all(np.isfinite(velocity)) else math.inf
        )
        self.vehicle_states[drone_id] = FleetVehicleState(
            vehicle_id=f"drone_{drone_id}",
            physical_drone_id=drone_id,
            namespace=f"/drone_{drone_id}",
            mocap_rigid_body_id=self.rigid_ids[drone_id],
            position_world=position,
            yaw_rad=self.initial_yaws[drone_id],
            stamp_s=now,
        )

    def _magnet_tip_callback(self, drone_id: int, msg: PoseStamped) -> None:
        z = float(msg.pose.position.z)
        if math.isfinite(z):
            self.magnet_floor_samples[drone_id] = (z, self._now_s())

    def _joint_truth_callback(self, drone_id: int, msg: Bool) -> None:
        self.joint_detached[drone_id] = bool(msg.data)
        self.joint_truth_stamps[drone_id] = self._now_s()

    def _arming_callback(self, drone_id: int, msg: Bool) -> None:
        self.arming_states[drone_id] = bool(msg.data)

    def _commitment_callback(self, drone_id: int, msg: CommittedTrajectory) -> None:
        # M2C is a ground-only commissioning gate. Once that gate has passed, M2D
        # legitimately publishes moving commitments and M2C must not reinterpret
        # them through its stationary-ground contract. The M2D supervisor already
        # latches the initial M2C ready edge and the assignment remains frozen.
        if self.pass_latched:
            return

        expected = f"drone_{drone_id}"
        if msg.vehicle_id != expected:
            self.commitment_rejections[drone_id] += 1
            self.get_logger().error(
                f"Rejected commitment on {expected} route: message vehicle_id={msg.vehicle_id!r}"
            )
            return

        # DDS startup ordering can deliver a commitment before this manager has
        # received the first mocap sample.  Ignore that pre-state sample rather
        # than permanently poisoning readiness; the 10 Hz publisher will retry.
        state = self.vehicle_states.get(drone_id)
        if state is None:
            return

        sequence = int(msg.sequence)
        previous = self.last_commitment_sequence.get(drone_id)
        if previous is not None and sequence <= previous:
            self.commitment_rejections[drone_id] += 1
            self.get_logger().error(
                f"Rejected non-monotonic {expected} commitment sequence {sequence} <= {previous}"
            )
            return
        if not msg.terminal_hold or len(msg.pieces) != 1:
            self.commitment_rejections[drone_id] += 1
            self.get_logger().error(
                f"Rejected non-stationary/non-cubic M2C commitment from {expected}"
            )
            return
        if msg.header.frame_id != self.frame_id:
            self.commitment_rejections[drone_id] += 1
            self.get_logger().error(
                f"Rejected {expected} commitment frame {msg.header.frame_id!r}; expected {self.frame_id!r}"
            )
            return

        piece = msg.pieces[0]
        try:
            validate_stationary_cubic_hold(
                control_points=[
                    [point.x, point.y, point.z] for point in piece.control_points
                ],
                knots=piece.knots,
                valid_from_s=piece.valid_from_s,
                valid_until_s=piece.valid_until_s,
                expected_position_world=state.position_world,
                now_s=self._now_s(),
                position_tolerance_m=0.04,
            )
        except (TypeError, ValueError) as exc:
            self.commitment_rejections[drone_id] += 1
            self.get_logger().error(
                f"Rejected untruthful {expected} stationary commitment: {exc}"
            )
            return

        self.last_commitment_sequence[drone_id] = sequence
        self.commitments[drone_id] = (msg, self._now_s())

    def _ring_callback(self, msg: PoseArray) -> None:
        if self.ring_pose_index < 0 or self.ring_pose_index >= len(msg.poses):
            return
        pose = msg.poses[self.ring_pose_index]
        try:
            position = _position(pose)
            rotation = quaternion_xyzw_to_rotation(_quaternion(pose))
        except (ValueError, TypeError):
            return
        if not np.all(np.isfinite(position)) or not np.all(np.isfinite(rotation)):
            return
        self.ring_position = position
        self.ring_rotation = rotation
        self.ring_stamp_s = self._now_s()

    def _try_assignment(self, now: float) -> None:
        if self.assignment_latch.frozen:
            return
        if len(self.vehicle_states) != 4 or self.ring_position is None or self.ring_rotation is None:
            return
        try:
            candidate = choose_fleet_assignment(
                vehicles=list(self.vehicle_states.values()),
                ring_position_world=self.ring_position,
                rotation_world_from_ring=self.ring_rotation,
                now_s=now,
                ring_stamp_s=self.ring_stamp_s,
                geometry=self.geometry,
                config=self.assignment_config,
            )
        except (ValueError, RuntimeError):
            return
        self.assignment_latch.freeze(candidate)
        payload = {
            "candidate_count": candidate.candidate_count,
            "plate_set_index": candidate.plate_set_index,
            "total_cost": candidate.total_cost,
            "vehicle_to_plate": dict(candidate.vehicle_to_plate),
        }
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True)
        self.assignment_pub.publish(msg)
        self.get_logger().info(f"M2C assignment frozen: {msg.data}")

    def _fresh_state_count(self, now: float) -> int:
        return sum(
            1
            for state in self.vehicle_states.values()
            if -1e-6 <= now - state.stamp_s <= self.vehicle_state_timeout_s
        )

    def _fresh_commitment_count(self, now: float) -> int:
        return sum(
            1
            for _, stamp in self.commitments.values()
            if -1e-6 <= now - stamp <= self.commitment_timeout_s
        )

    def _ring_fresh(self, now: float) -> bool:
        return (
            self.ring_stamp_s is not None
            and -1e-6 <= now - self.ring_stamp_s <= self.ring_state_timeout_s
        )

    def _fresh_detached_count(self, now: float) -> int:
        return sum(
            1
            for drone_id in DRONE_IDS
            if self.joint_detached.get(drone_id, False)
            and drone_id in self.joint_truth_stamps
            and -1e-6 <= now - self.joint_truth_stamps[drone_id] <= self.joint_truth_timeout_s
        )

    def _disarmed_count(self) -> int:
        return sum(
            1 for drone_id in DRONE_IDS if self.arming_states.get(drone_id) is False
        )

    def _grounded_count(self, now: float) -> int:
        count = 0
        for drone_id in DRONE_IDS:
            state = self.vehicle_states.get(drone_id)
            tip = self.magnet_floor_samples.get(drone_id)
            speed = self.vehicle_speeds.get(drone_id, math.inf)
            if state is None or tip is None:
                continue
            tip_z, tip_stamp = tip
            if (
                -1e-6 <= now - state.stamp_s <= self.vehicle_state_timeout_s
                and 0.08 <= float(state.position_world[2]) <= 0.15
                and speed <= 0.05
                and abs(tip_z - 0.025) <= 0.02
                and 0.0 <= now - tip_stamp <= 0.25
            ):
                count += 1
        return count

    def _publish_markers(self) -> None:
        assignment = self.assignment_latch.assignment
        if assignment is None or self.ring_position is None or self.ring_rotation is None:
            return
        markers = MarkerArray()
        now = self.get_clock().now().to_msg()
        for index, vehicle_id in enumerate(sorted(assignment.vehicle_to_plate)):
            drone_id = int(vehicle_id.rsplit("_", 1)[1])
            state = self.vehicle_states.get(drone_id)
            if state is None:
                continue
            plate = int(assignment.vehicle_to_plate[vehicle_id])
            target = attached_hold_body_target_world(
                geometry=self.geometry,
                ring_position_world=self.ring_position,
                rotation_world_from_ring=self.ring_rotation,
                plate_index=plate,
                vehicle_yaw_rad=state.yaw_rad,
                tether_length_m=self.assignment_config.tether_length_m,
                tether_anchor_body=self.assignment_config.tether_anchor_body,
                attached_hold_angle_deg=self.assignment_config.attached_hold_angle_deg,
            )
            line = Marker()
            line.header.frame_id = self.frame_id
            line.header.stamp = now
            line.ns = "m2c_assignment"
            line.id = 10 + index
            line.type = Marker.LINE_LIST
            line.action = Marker.ADD
            line.scale.x = 0.012
            line.color.r = 1.0
            line.color.g = 1.0
            line.color.b = 1.0
            line.color.a = 0.8
            a = Point()
            a.x, a.y, a.z = (float(v) for v in state.position_world)
            b = Point()
            b.x, b.y, b.z = (float(v) for v in target)
            line.points = [a, b]
            markers.markers.append(line)

            text = Marker()
            text.header.frame_id = self.frame_id
            text.header.stamp = now
            text.ns = "m2c_assignment_labels"
            text.id = 100 + index
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x, text.pose.position.y, text.pose.position.z = (
                float(state.position_world[0]),
                float(state.position_world[1]),
                float(state.position_world[2] + 0.25),
            )
            text.pose.orientation.w = 1.0
            text.scale.z = 0.12
            text.color.r = text.color.g = text.color.b = text.color.a = 1.0
            text.text = f"{vehicle_id} -> plate {plate}"
            markers.markers.append(text)
        self.markers_pub.publish(markers)

    def _tick(self) -> None:
        now = self._now_s()
        state_count = self._fresh_state_count(now)
        commitment_count = self._fresh_commitment_count(now)
        ring_fresh = self._ring_fresh(now)
        detached_count = self._fresh_detached_count(now)
        grounded_count = self._grounded_count(now)
        disarmed_count = self._disarmed_count()

        # Assignment must represent the free, settled starting geometry rather than
        # the artificial Gazebo startup fixture. One bad sample resets the dwell.
        ground_settled = self.ground_settle_gate.update(
            condition=(state_count == 4 and detached_count == 4 and grounded_count == 4),
            now_s=now,
        )
        if ground_settled:
            self._try_assignment(now)

        assignment = self.assignment_latch.assignment
        ready = (
            assignment is not None
            and state_count == 4
            and commitment_count == 4
            and ring_fresh
            and detached_count == 4
            and grounded_count == 4
            and ground_settled
            and disarmed_count == 4
            and not any(self.commitment_rejections.values())
        )
        if ready:
            self.pass_latched = True
            state = "M2C_PASS"
        elif not ground_settled:
            state = "WAITING_FOR_GROUND_SETTLE"
        elif assignment is None:
            state = "WAITING_FOR_ASSIGNMENT"
        elif commitment_count < 4:
            state = "WAITING_FOR_COMMITMENTS"
        elif disarmed_count < 4:
            state = "WAITING_FOR_DISARMED_CONTROLLERS"
        else:
            state = "WAITING_FOR_FRESH_FLEET"
        self.last_status_state = state

        status = {
            "state": state,
            "ground_only": True,
            "fresh_vehicle_states": state_count,
            "fresh_commitments": commitment_count,
            "ring_fresh": ring_fresh,
            "joint_detached_count": detached_count,
            "grounded_vehicle_count": grounded_count,
            "ground_settled": ground_settled,
            "ground_settle_elapsed_s": self.ground_settle_gate.elapsed_s(now),
            "ground_settle_dwell_s": self.ground_settle_dwell_s,
            "disarmed_controller_count": disarmed_count,
            "assignment_frozen": assignment is not None,
            "candidate_count": None if assignment is None else assignment.candidate_count,
            "plate_set_index": None if assignment is None else assignment.plate_set_index,
            "vehicle_to_plate": None if assignment is None else dict(assignment.vehicle_to_plate),
            "commitment_rejections": dict(self.commitment_rejections),
            "initial_yaw_rad": {f"drone_{i}": self.initial_yaws[i] for i in sorted(self.initial_yaws)},
        }
        text = String()
        text.data = json.dumps(status, sort_keys=True)
        self.status_pub.publish(text)
        flag = Bool()
        flag.data = bool(ready)
        self.ready_pub.publish(flag)
        if assignment is not None:
            self._publish_markers()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = M2CFleetManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

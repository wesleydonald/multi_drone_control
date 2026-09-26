#!/usr/bin/env python3
"""Two-drone IRL readiness and frozen opposite-plate assignment manager."""

from __future__ import annotations

import json
import math
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import PoseArray, PoseStamped
from interfaces.msg import MotionCaptureState, Telemetry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Empty, String

from .m2_fleet_assignment import (
    FleetAssignmentConfig,
    FleetVehicleState,
    TwoDroneAssignmentLatch,
    choose_two_drone_assignment,
)
from .m2_irl_readiness import TwoDroneIRLReadinessGate
from .m2_irl_launch_config import parse_identity_json
from .m2a_attachment_runtime import quaternion_xyzw_to_rotation


VEHICLE_IDS = ("drone_0", "drone_1")


def _yaw(q) -> float:
    x, y, z, w = float(q.x), float(q.y), float(q.z), float(q.w)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1e-12:
        raise ValueError("zero quaternion")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class M2IRLFleetManager(Node):
    def __init__(self) -> None:
        super().__init__("m2_irl_fleet_manager")
        self.timeout_s = float(self.declare_parameter("readiness_timeout_s", 0.50).value)
        self.gate = TwoDroneIRLReadinessGate(
            timeout_s=self.timeout_s,
            status_timeout_s=float(
                self.declare_parameter("status_timeout_s", 1.25).value
            ),
            stationary_dwell_s=float(
                self.declare_parameter("stationary_dwell_s", 1.0).value
            ),
            max_stationary_speed_mps=float(
                self.declare_parameter("max_stationary_speed_mps", 0.05).value
            ),
        )
        self.identity = parse_identity_json(
            str(self.declare_parameter("hardware_identity_json", "").value)
        )
        self.hardware = self.identity.vehicles
        self.rigid_ids = {vehicle_id: item.body_id for vehicle_id, item in self.hardware.items()}
        self.physical_quads = {vehicle_id: item.physical_quad for vehicle_id, item in self.hardware.items()}
        self.assignment_config = FleetAssignmentConfig(
            state_timeout_s=self.timeout_s,
            ring_state_timeout_s=self.timeout_s,
            tether_length_m=float(self.declare_parameter("tether_length_m", 0.475).value),
            tether_anchor_body=np.array(
                self.declare_parameter("tether_anchor_body", [0.0, 0.0, -0.04]).value,
                dtype=float,
            ),
            attached_hold_angle_deg=15.0,
        )
        self.assignment_latch = TwoDroneAssignmentLatch()
        self.states: dict[str, FleetVehicleState] = {}
        self.speeds: dict[str, float] = {}
        self.body_stamps: dict[str, float] = {}
        self.magnet_stamps: dict[str, float] = {}
        self.armed: dict[str, bool] = {}
        self.arming_stamps: dict[str, float] = {}
        self.planner_stamps: dict[str, float] = {}
        self.backend_stamps: dict[str, float] = {}
        self.controller_stamps: dict[str, float] = {}
        self.telemetry_stamps: dict[str, float] = {}
        self.ring_position: Optional[np.ndarray] = None
        self.ring_rotation: Optional[np.ndarray] = None
        self.ring_stamp_s: Optional[float] = None

        for vehicle_id in VEHICLE_IDS:
            ns = f"/{vehicle_id}"
            self.create_subscription(
                MotionCaptureState, f"{ns}/motion_capture_state",
                lambda msg, v=vehicle_id: self._body(v, msg), 10
            )
            self.create_subscription(
                PoseStamped, f"{ns}/magnet_tip_pose",
                lambda msg, v=vehicle_id: self._magnet(v, msg), 10
            )
            self.create_subscription(
                Bool, f"{ns}/arming_state_feedback",
                lambda msg, v=vehicle_id: self._arming(v, msg), 10
            )
            # The planner publishes this fail-closed Bool every tick before it
            # receives an assignment.  Treat receipt, not its value, as the fresh
            # planner-path heartbeat so assignment readiness cannot depend on the
            # post-assignment committed trajectory.
            self.create_subscription(
                Bool, f"{ns}/join_planner/arm_permission",
                lambda msg, v=vehicle_id: self._stamp(self.planner_stamps, v), 10
            )
            self.create_subscription(
                String, f"{ns}/dynamic_planner/transfer_status",
                lambda msg, v=vehicle_id: self._stamp(self.backend_stamps, v), 10
            )
            self.create_subscription(
                String, f"{ns}/tejen_mpc/c1d_status",
                lambda msg, v=vehicle_id: self._stamp(self.controller_stamps, v), 10
            )
            self.create_subscription(
                Telemetry, f"{ns}/telemetry",
                lambda msg, v=vehicle_id: self._stamp(self.telemetry_stamps, v), 10
            )
            self.create_subscription(
                Empty, f"{ns}/elrs/link_statistics",
                lambda msg, v=vehicle_id: self.gate.note_link_statistics(v, stamp_s=self._now()), 10
            )
        self.create_subscription(PoseArray, "/m2/ring/pose", self._ring, 10)

        latched = QoSProfile(depth=1)
        latched.reliability = ReliabilityPolicy.RELIABLE
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.assignment_pub = self.create_publisher(String, "/m2c/assignment", latched)
        self.ready_pub = self.create_publisher(Bool, "/m2c/ready", 10)
        self.status_pub = self.create_publisher(String, "/m2c/status", 10)
        self.create_timer(0.1, self._tick)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _stamp(self, destination: dict[str, float], vehicle_id: str) -> None:
        destination[vehicle_id] = self._now()

    def _body(self, vehicle_id: str, msg: MotionCaptureState) -> None:
        now = self._now()
        position = np.array(
            [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z], dtype=float
        )
        velocity = np.array(
            [msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z], dtype=float
        )
        if not np.all(np.isfinite(position)) or not np.all(np.isfinite(velocity)):
            return
        self.speeds[vehicle_id] = float(np.linalg.norm(velocity))
        self.body_stamps[vehicle_id] = now
        self.states[vehicle_id] = FleetVehicleState(
            vehicle_id=vehicle_id,
            physical_drone_id=self.physical_quads[vehicle_id],
            namespace=f"/{vehicle_id}",
            mocap_rigid_body_id=self.rigid_ids[vehicle_id],
            position_world=position,
            yaw_rad=_yaw(msg.pose.orientation),
            stamp_s=now,
        )

    def _magnet(self, vehicle_id: str, _msg: PoseStamped) -> None:
        self.magnet_stamps[vehicle_id] = self._now()

    def _arming(self, vehicle_id: str, msg: Bool) -> None:
        self.armed[vehicle_id] = bool(msg.data)
        self.arming_stamps[vehicle_id] = self._now()

    def _ring(self, msg: PoseArray) -> None:
        if not msg.poses:
            return
        pose = msg.poses[0]
        position = np.array([pose.position.x, pose.position.y, pose.position.z])
        quaternion = np.array(
            [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        )
        try:
            rotation = quaternion_xyzw_to_rotation(quaternion)
        except ValueError:
            return
        if not np.all(np.isfinite(position)):
            return
        now = self._now()
        self.ring_position = position
        self.ring_rotation = rotation
        self.ring_stamp_s = now
        self.gate.update_ring(stamp_s=now)

    def _update_gate(self) -> None:
        for vehicle_id in VEHICLE_IDS:
            tables = (
                self.body_stamps, self.magnet_stamps, self.speeds, self.armed,
                self.arming_stamps, self.planner_stamps, self.backend_stamps,
                self.controller_stamps, self.telemetry_stamps,
            )
            if not all(vehicle_id in table for table in tables):
                continue
            self.gate.update_vehicle(
                vehicle_id,
                body_stamp_s=self.body_stamps[vehicle_id],
                magnet_stamp_s=self.magnet_stamps[vehicle_id],
                speed_mps=self.speeds[vehicle_id],
                armed=self.armed[vehicle_id],
                arming_stamp_s=self.arming_stamps[vehicle_id],
                planner_stamp_s=self.planner_stamps[vehicle_id],
                backend_stamp_s=self.backend_stamps[vehicle_id],
                controller_stamp_s=self.controller_stamps[vehicle_id],
                telemetry_stamp_s=self.telemetry_stamps[vehicle_id],
            )

    def _tick(self) -> None:
        now = self._now()
        self._update_gate()
        snapshot = self.gate.evaluate(now)
        if snapshot.ready and not self.assignment_latch.frozen:
            if self.ring_position is not None and self.ring_rotation is not None:
                try:
                    chosen = choose_two_drone_assignment(
                        vehicles=[self.states[v] for v in VEHICLE_IDS],
                        ring_position_world=self.ring_position,
                        rotation_world_from_ring=self.ring_rotation,
                        ring_stamp_s=self.ring_stamp_s,
                        now_s=now,
                        config=self.assignment_config,
                    )
                    self.assignment_latch.freeze(chosen)
                except (KeyError, RuntimeError, ValueError) as exc:
                    self.get_logger().error(f"assignment refused: {exc}")
        ready = bool(snapshot.ready and self.assignment_latch.frozen)
        ready_msg = Bool(); ready_msg.data = ready; self.ready_pub.publish(ready_msg)
        assignment = self.assignment_latch.assignment
        if assignment is not None:
            msg = String()
            msg.data = json.dumps(
                {
                    "vehicle_to_plate": dict(assignment.vehicle_to_plate),
                    "candidate_count": assignment.candidate_count,
                    "total_cost": assignment.total_cost,
                }, sort_keys=True
            )
            self.assignment_pub.publish(msg)
        status = String()
        status.data = json.dumps(
            {
                "ready": ready,
                "reason": snapshot.reason,
                "stationary_dwell_s": snapshot.stationary_dwell_s,
                "vehicle_ready": dict(snapshot.vehicle_ready),
                "vehicle_reasons": dict(snapshot.vehicle_reasons),
                "assignment_frozen": self.assignment_latch.frozen,
            }, sort_keys=True
        )
        self.status_pub.publish(status)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = M2IRLFleetManager()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

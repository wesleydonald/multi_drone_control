#!/usr/bin/env python3
"""Thin ROS wrapper around the M2D sequential fleet supervision policy."""

from __future__ import annotations

import json

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Int32, String

from .m2d_sequential_supervision import (
    ATTACHED_PHASE,
    DEFAULT_VEHICLE_IDS,
    M2DSequentialSupervisorCore,
)


def _latched_qos() -> QoSProfile:
    qos = QoSProfile(depth=1)
    qos.reliability = ReliabilityPolicy.RELIABLE
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    return qos


class M2DFleetSupervisor(Node):
    def __init__(self) -> None:
        super().__init__("m2d_fleet_supervisor")
        self.vehicle_ids = tuple(
            str(value)
            for value in self.declare_parameter(
                "vehicle_ids", list(DEFAULT_VEHICLE_IDS)
            ).value
        )
        if not self.vehicle_ids or len(set(self.vehicle_ids)) != len(self.vehicle_ids):
            raise ValueError("vehicle_ids must contain unique non-empty names")
        self.target_attachment_count = int(
            self.declare_parameter("target_attachment_count", len(self.vehicle_ids)).value
        )
        self.operator_hold_after_goal = bool(
            self.declare_parameter("operator_hold_after_goal", False).value
        )
        self.core = M2DSequentialSupervisorCore(
            vehicle_ids=self.vehicle_ids,
            evidence_timeout_s=float(
                self.declare_parameter("attachment_evidence_timeout_s", 0.75).value
            ),
            target_attachment_count=self.target_attachment_count,
            operator_hold_after_goal=self.operator_hold_after_goal,
            simultaneous=bool(self.declare_parameter("simultaneous", False).value),
        )
        self.assignment_topic = str(
            self.declare_parameter("m2c_assignment_topic", "/m2c/assignment").value
        )
        self.ready_topic = str(
            self.declare_parameter("m2c_ready_topic", "/m2c/ready").value
        )
        self.status_topic = str(
            self.declare_parameter("status_topic", "/m2d/status").value
        )
        self.pass_topic = str(
            self.declare_parameter("pass_topic", "/m2d/pass").value
        )
        self.operator_land_request_topic = str(
            self.declare_parameter(
                "operator_land_request_topic", "/m2d/operator/land_request"
            ).value
        )
        self.publish_rate_hz = max(
            2.0, float(self.declare_parameter("publish_rate_hz", 10.0).value)
        )

        latched = _latched_qos()
        self.create_subscription(String, self.assignment_topic, self._assignment_callback, latched)
        self.create_subscription(Bool, self.ready_topic, self._ready_callback, 10)
        self.create_subscription(
            Bool,
            self.operator_land_request_topic,
            self._operator_land_request_callback,
            10,
        )

        self.permission_pubs = {}
        self.assignment_pubs = {}
        self.attached_plate_pubs = {}
        self.land_request_pubs = {}
        for vehicle_id in self.vehicle_ids:
            ns = f"/{vehicle_id}"
            self.create_subscription(
                String,
                f"{ns}/join_planner/phase",
                lambda msg, vehicle_id=vehicle_id: self._phase_callback(vehicle_id, msg),
                latched,
            )
            self.create_subscription(
                String,
                f"{ns}/attachment/diagnostics",
                lambda msg, vehicle_id=vehicle_id: self._attachment_callback(vehicle_id, msg),
                10,
            )
            self.permission_pubs[vehicle_id] = self.create_publisher(
                Bool, f"{ns}/m2d/mission_permission", latched
            )
            self.assignment_pubs[vehicle_id] = self.create_publisher(
                Int32, f"{ns}/m2d/assigned_plate_id", latched
            )
            self.attached_plate_pubs[vehicle_id] = self.create_publisher(
                Int32, f"{ns}/m2d/attached_plate_id", latched
            )
            self.land_request_pubs[vehicle_id] = self.create_publisher(
                Bool, f"{ns}/join_planner/land_now", 10
            )

        # one C++ transit at a time: the lowest-index unattached vehicle (simultaneous
        # mode) or the active one (sequential); see online_join_planner m2d_transit_owner
        self.transit_owner_pub = self.create_publisher(String, "/m2d/transit_owner", latched)
        self.status_pub = self.create_publisher(String, self.status_topic, latched)
        self.pass_pub = self.create_publisher(Bool, self.pass_topic, latched)
        self._last_state = ""
        self._last_landing_vehicle = None
        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self._tick)
        self.get_logger().info(
            "M2D fleet supervisor started: frozen M2C assignment, manual ARM/TAKEOFF, "
            f"order={'->'.join(self.vehicle_ids)}, target_attachments={self.target_attachment_count}, "
            f"operator_hold_after_goal={self.operator_hold_after_goal}."
        )

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _assignment_callback(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            mapping = payload.get("vehicle_to_plate", payload)
            self.core.freeze_assignment(mapping)
        except Exception as exc:
            self.get_logger().error(f"Rejected M2C assignment for M2D: {exc}")

    def _ready_callback(self, msg: Bool) -> None:
        self.core.update_m2c_ready(bool(msg.data))

    def _phase_callback(self, vehicle_id: str, msg: String) -> None:
        self.core.update_phase(vehicle_id, msg.data.strip())

    def _attachment_callback(self, vehicle_id: str, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            self.core.update_attachment_evidence(
                vehicle_id,
                confirmed=bool(payload.get("confirmed", False)),
                lost=bool(payload.get("lost", False)),
                stamp_s=self._now_s(),
            )
        except Exception as exc:
            self.get_logger().warn(f"Ignored malformed {vehicle_id} attachment diagnostics: {exc}")

    def _operator_land_request_callback(self, msg: Bool) -> None:
        if not bool(msg.data):
            return
        if self.core.request_landing():
            order = " -> ".join(self.core.landing_order)
            self.get_logger().warning(
                f"Operator fleet LAND accepted; reverse landing order: {order}."
            )
        else:
            self.get_logger().warning(
                "Operator fleet LAND ignored: request is only accepted from M2D_SUCCESS_HOLD."
            )

    def _publish_current_landing_request(self, snapshot) -> None:
        vehicle_id = snapshot.landing_vehicle_id
        if vehicle_id is None:
            self._last_landing_vehicle = None
            return

        # Re-publish while the vehicle is still in ATTACHED_HOLD. This makes the
        # operator request robust to one dropped volatile message without changing
        # per-vehicle landing authority. Once the planner acknowledges by entering
        # M2_LANDING_STAGE/LANDING, it owns the remainder of the sequence.
        if self.core.phases[vehicle_id] == ATTACHED_PHASE:
            request = Bool()
            request.data = True
            self.land_request_pubs[vehicle_id].publish(request)

        if vehicle_id != self._last_landing_vehicle:
            self.get_logger().warning(
                f"Fleet landing authority -> {vehicle_id}; requesting release-before-land."
            )
            self._last_landing_vehicle = vehicle_id

    def _tick(self) -> None:
        snapshot = self.core.step(self._now_s())
        self._publish_current_landing_request(snapshot)
        owner = String()
        if self.core.simultaneous and not snapshot.abort_reason:
            owner.data = next((v for v in self.vehicle_ids if v not in self.core.attached), "")
        else:
            owner.data = snapshot.active_vehicle_id or ""
        self.transit_owner_pub.publish(owner)

        for vehicle_id in self.vehicle_ids:
            permission = Bool()
            permission.data = bool(snapshot.permissions[vehicle_id])
            self.permission_pubs[vehicle_id].publish(permission)

            assigned = Int32()
            assigned.data = int(snapshot.assigned_plate_ids.get(vehicle_id, -1))
            self.assignment_pubs[vehicle_id].publish(assigned)

            attached = Int32()
            attached.data = int(snapshot.attached_plate_ids.get(vehicle_id, -1))
            self.attached_plate_pubs[vehicle_id].publish(attached)

        payload = {
            "state": snapshot.state,
            "active_vehicle_id": snapshot.active_vehicle_id,
            "permissions": dict(snapshot.permissions),
            "assigned_plate_ids": dict(snapshot.assigned_plate_ids),
            "attached_plate_ids": dict(snapshot.attached_plate_ids),
            "abort_reason": snapshot.abort_reason,
            "target_attachment_count": self.target_attachment_count,
            "operator_hold_after_goal": self.operator_hold_after_goal,
            "landing_vehicle_id": snapshot.landing_vehicle_id,
            "landed_vehicle_ids": list(snapshot.landed_vehicle_ids),
        }
        status = String()
        status.data = json.dumps(payload, separators=(",", ":"), sort_keys=True)
        self.status_pub.publish(status)
        passed = Bool()
        passed.data = snapshot.passed
        self.pass_pub.publish(passed)

        if snapshot.state != self._last_state:
            self.get_logger().info(
                f"M2D state={snapshot.state} active={snapshot.active_vehicle_id} "
                f"attached={list(snapshot.attached_plate_ids)} "
                f"landing={snapshot.landing_vehicle_id or 'none'} "
                f"landed={list(snapshot.landed_vehicle_ids)} "
                f"abort={snapshot.abort_reason or 'none'}"
            )
            self._last_state = snapshot.state


def main(args=None) -> None:
    rclpy.init(args=args)
    node = M2DFleetSupervisor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

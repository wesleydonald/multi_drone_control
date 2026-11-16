"""Canonical identity-bound ROS routes for M2 fleet vehicles.

The partner multi-drone stack established the ``/drone_<id>/...`` platform
convention.  This module keeps that convention in one ROS-independent place so
launch files, fleet supervision and tests cannot silently invent incompatible
parallel topic lists.
"""

from __future__ import annotations

from dataclasses import dataclass, asdict


@dataclass(frozen=True)
class VehicleTopics:
    namespace: str
    motion_capture_state: str
    telemetry: str
    elrs_command: str
    command: str
    arming_service: str
    arming_state_feedback: str
    reference: str
    pendulum_swing_state: str
    magnet_tip_pose: str
    magnet_command: str
    object_attached: str
    attachment_diagnostics: str
    arm_permission: str
    mpc_mode: str
    committed_trajectory: str
    joint_attach: str
    joint_detach: str
    joint_state: str


def vehicle_namespace(drone_id: int) -> str:
    value = int(drone_id)
    if value < 0:
        raise ValueError("drone_id must be non-negative")
    return f"/drone_{value}"


def canonical_vehicle_topics(drone_id: int) -> dict[str, str]:
    """Return the canonical M2 topic/service map for one physical vehicle."""

    ns = vehicle_namespace(drone_id)
    topics = VehicleTopics(
        namespace=ns,
        motion_capture_state=f"{ns}/motion_capture_state",
        telemetry=f"{ns}/telemetry",
        elrs_command=f"{ns}/ELRSCommand",
        command=f"{ns}/command",
        arming_service=f"{ns}/arming_service",
        arming_state_feedback=f"{ns}/arming_state_feedback",
        reference=f"{ns}/join_planner/reference",
        pendulum_swing_state=f"{ns}/pendulum_swing_state",
        magnet_tip_pose=f"{ns}/magnet_tip_pose",
        magnet_command=f"{ns}/magnet/command",
        object_attached=f"{ns}/magnet/object_attached",
        attachment_diagnostics=f"{ns}/attachment/diagnostics",
        arm_permission=f"{ns}/join_planner/arm_permission",
        mpc_mode=f"{ns}/join_planner/mpc_mode",
        committed_trajectory=f"{ns}/committed_trajectory",
        joint_attach=f"{ns}/magnet/attach",
        joint_detach=f"{ns}/magnet/detach",
        joint_state=f"{ns}/magnet/joint_detached_truth",
    )
    return asdict(topics)

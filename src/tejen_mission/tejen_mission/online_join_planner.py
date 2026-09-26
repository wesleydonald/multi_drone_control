#!/usr/bin/env python3
"""Barebones, reusable per-drone mission and trajectory planner.

This planner deliberately contains no collision avoidance.  It separates:

* a declarative mission/phase description;
* four reusable reference primitives;
* mission transition guards; and
* the ROS interface used by the existing payload-aware NMPC.

The node remains responsible for one vehicle only.  A future multi-drone or
swarm coordinator can assign targets and consume ``handoff_ready`` without
being embedded in this planner.
"""

from __future__ import annotations

import csv
import json
from collections import deque
import math
import os
import time
from concurrent.futures import Future, ThreadPoolExecutor
from dataclasses import dataclass
from datetime import datetime
from typing import Dict, Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseArray, PoseStamped, Transform, Twist, TwistStamped
from interfaces.msg import MotionCaptureState, CommittedTrajectory, CubicTrajectoryPiece
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Empty, Int32, String
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from visualization_msgs.msg import Marker, MarkerArray

from .mission_definitions import mission_for_mode
from .m2a_attachment_runtime import quaternion_xyzw_to_rotation
from .m2_irl_readiness import link_statistics_fresh
from .mission_types import MissionPhase, ReferenceType, TargetSource
from .cooperative_trajectory import RingNetGeometry
from .m2_fleet_commitment import commitment_sequence_seed
from .m2b_attachment_mission import (
    BootstrapStatus,
    M2BConfig,
    M2BBootstrapGate,
    M2BDescentProgress,
    M2BEvidenceFreshnessGate,
    M2BLatchStabilityGate,
    M2BRetryTracker,
    M2BSettleGate,
    attached_hold_direction_plate,
    body_reference_from_contact,
    contact_capture_position,
    evaluate_takeoff_magnet_clearance,
    is_m2b_b1_local_reference_phase,
    m2b_mpc_mode_for_phase,
    proof_command_distance,
    proof_direction_plate,
    proof_success,
    should_enable_magnet,
    should_enter_capture_wait,
    yaw_quaternion_xyzw,
)
from .payload_geometry import PayloadGeometryProfile, PickupTargetGeometry
from .online_safety_planner import (
    OnlineSafetyPlanner,
    SafetyPlannerRequest,
    SafetyPlannerResult,
    StaticSphereObstacle,
)
from .reference_generators import (
    TargetState,
    TrajectoryReference,
    TransferConfig,
    VirtualTransferGenerator,
    rate_controlled_manoeuvre,
    smooth_rate_controlled_manoeuvre,
    stationary_regulation,
    target_relative_tracking,
)
from .visual_astar_planner import (
    VisualAStarConfig,
    VisualAStarRequest,
    VisualAStarResult,
    compute_visual_astar,
    idle_visual_astar_result,
)
from .convex_trajectory_smoother import (
    ConvexTrajectoryConfig,
)
from .convex_corridor import polyhedron_triangles
from .moving_obstacle_prediction import (
    MovingObstaclePrediction,
    MovingObstaclePredictionConfig,
    MovingObstaclePredictor,
    MovingObstacleState,
)
from .c1f2_handoff import (
    TargetRelativeBridgeFeasibility,
    TargetRelativeBridgeState,
    advance_reference_window,
    handoff_allowed,
    loaded_lift_override_allowed,
    loaded_lift_ready_for_cpp,
    post_grant_reference_is_fresh,
    stationary_grant_state,
    stage0_errors,
    target_relative_bridge_feasibility,
    target_relative_bridge_initial_state,
    target_relative_bridge_reference,
)


# Backwards-compatible string aliases for scripts that imported the old module.
WAIT_FOR_TAKEOFF = MissionPhase.WAIT_FOR_TAKEOFF.value
TAKEOFF = MissionPhase.TAKEOFF.value
APPROACH_ABOVE_PICKUP = MissionPhase.APPROACH_ABOVE_PICKUP.value
SETTLE_ABOVE_PICKUP = MissionPhase.SETTLE_ABOVE_PICKUP.value
DESCEND_TO_PICKUP = MissionPhase.DESCEND_TO_PICKUP.value
MAGNET_ATTACH_WAIT = MissionPhase.MAGNET_ATTACH_WAIT.value
LIFT_OBJECT = MissionPhase.LIFT_OBJECT.value
TRANSIT_TO_DROP_POINT = MissionPhase.TRANSIT_TO_DROP_POINT.value
SETTLE_ABOVE_DROP_POINT = MissionPhase.SETTLE_ABOVE_DROP_POINT.value
DESCEND_TO_DROP_HEIGHT = MissionPhase.DESCEND_TO_DROP_HEIGHT.value
DROP_OBJECT = MissionPhase.DROP_OBJECT.value
CLEAR_DROP_ZONE = MissionPhase.CLEAR_DROP_ZONE.value
TRANSIT_TO_REATTACH = MissionPhase.TRANSIT_TO_REATTACH.value
APPROACH_ABOVE_TARGET = MissionPhase.APPROACH_ABOVE_TARGET.value
MATCH_VELOCITY = MissionPhase.MATCH_VELOCITY.value
DESCEND_TO_ATTACHMENT = MissionPhase.DESCEND_TO_ATTACHMENT.value
ATTACH_READY = MissionPhase.ATTACH_READY.value
LANDING = MissionPhase.LANDING.value
LANDED_DISARMED = MissionPhase.LANDED_DISARMED.value


def _vec3_from_position(position) -> np.ndarray:
    return np.array(
        [float(position.x), float(position.y), float(position.z)],
        dtype=float,
    )


def _vec3_from_linear(vector) -> np.ndarray:
    return np.array([float(vector.x), float(vector.y), float(vector.z)], dtype=float)


def _make_point(vector: np.ndarray) -> Point:
    point = Point()
    point.x = float(vector[0])
    point.y = float(vector[1])
    point.z = float(vector[2])
    return point


def mission_phase_qos_profile() -> QoSProfile:
    """QoS contract for the latched machine-readable mission phase."""
    profile = QoSProfile(depth=1)
    profile.reliability = ReliabilityPolicy.RELIABLE
    profile.durability = DurabilityPolicy.TRANSIENT_LOCAL
    return profile


def _yaw_from_quaternion(quaternion) -> float:
    w = float(quaternion.w)
    x = float(quaternion.x)
    y = float(quaternion.y)
    z = float(quaternion.z)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _rotate_yaw(vector: np.ndarray, yaw: float) -> np.ndarray:
    c = math.cos(float(yaw))
    s = math.sin(float(yaw))
    return np.array(
        [
            c * vector[0] - s * vector[1],
            s * vector[0] + c * vector[1],
            vector[2],
        ],
        dtype=float,
    )


def _set_marker_yaw(marker: Marker, yaw: float) -> None:
    """Set a marker orientation for a level body with the supplied yaw."""

    half_yaw = 0.5 * float(yaw)
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = math.sin(half_yaw)
    marker.pose.orientation.w = math.cos(half_yaw)


@dataclass(frozen=True)
class RateProfile:
    start_tip_offset: np.ndarray
    goal_tip_offset: np.ndarray
    rate: float

    def __post_init__(self) -> None:
        start = np.asarray(self.start_tip_offset, dtype=float).reshape(3)
        goal = np.asarray(self.goal_tip_offset, dtype=float).reshape(3)
        object.__setattr__(self, "start_tip_offset", start.copy())
        object.__setattr__(self, "goal_tip_offset", goal.copy())
        object.__setattr__(self, "rate", max(0.0, float(self.rate)))

    @property
    def duration(self) -> float:
        distance = float(np.linalg.norm(self.goal_tip_offset - self.start_tip_offset))
        if distance <= 1e-12:
            return 0.0
        if self.rate <= 1e-12:
            return float("inf")
        return distance / self.rate

    def tip_offset_at(self, elapsed: float) -> np.ndarray:
        delta = self.goal_tip_offset - self.start_tip_offset
        distance = float(np.linalg.norm(delta))
        if distance <= 1e-12 or self.rate <= 1e-12:
            return self.start_tip_offset.copy()
        direction = delta / distance
        travel = min(distance, self.rate * max(0.0, float(elapsed)))
        return self.start_tip_offset + direction * travel

    def complete(self, elapsed: float) -> bool:
        return bool(np.isfinite(self.duration) and elapsed >= self.duration - 1e-9)


class OnlineJoinPlanner(Node):
    """Per-drone mission executive and rolling reference publisher."""

    def _physical_now_s(self) -> float:
        """Physical mission time from the ROS node clock.

        Simulation launches set use_sim_time=true so this follows Gazebo /clock;
        hardware/default launches continue to use the normal ROS/system clock.
        """
        return self.get_clock().now().nanoseconds * 1e-9

    def _wall_now_s(self) -> float:
        """Host monotonic time for bootstrap / worker supervision."""
        return time.monotonic()

    def __init__(self) -> None:
        super().__init__("online_join_planner")

        # ------------------------------------------------------------------
        # Identity and topics
        # ------------------------------------------------------------------
        self.vehicle_id = str(self.declare_parameter("vehicle_id", "drone_0").value)
        self.assigned_attachment_id = str(
            self.declare_parameter("assigned_attachment_id", "attachment_0").value
        )
        self.frame_id = str(self.declare_parameter("frame_id", "map").value)

        # M2C cooperative-future publication. Disabled by default so the
        # commissioned M1/M2B paths are byte-for-behaviour compatible. M2C
        # enables this while vehicles are grounded and publishes a truthful
        # stationary terminal_hold commitment. Moving-authority publication is
        # intentionally deferred to the M2D execution layer.
        self.publish_committed_trajectory = bool(
            self.declare_parameter("publish_committed_trajectory", False).value
        )
        self.committed_trajectory_topic = str(
            self.declare_parameter("committed_trajectory_topic", "committed_trajectory").value
        )
        self.committed_trajectory_horizon_s = max(
            1.0, float(self.declare_parameter("committed_trajectory_horizon_s", 10.0).value)
        )
        self.committed_trajectory_stationary_speed_mps = max(
            0.0,
            float(
                self.declare_parameter(
                    "committed_trajectory_stationary_speed_mps", 0.05
                ).value
            ),
        )
        self.m2c_ground_reference_enabled = bool(
            self.declare_parameter("m2c_ground_reference_enabled", False).value
        )
        # Ground-only M2C needs only the controller's current 20-stage,
        # skip_steps=3 rolling-reference contract: 20*3 + sample-0 = 61 points.
        # Keep this separate from the general 3 s planner horizon so B1/B2 and
        # future moving-reference behavior remain unchanged.
        self.m2c_ground_reference_samples = max(
            2, int(self.declare_parameter("m2c_ground_reference_samples", 61).value)
        )
        self.committed_trajectory_sequence = commitment_sequence_seed()

        self.drone_state_topic = str(
            self.declare_parameter("drone_state_topic", "/motion_capture_state").value
        )
        self.attachment_pose_topic = str(
            self.declare_parameter(
                "attachment_pose_topic", "/fake_attachment_point/pose"
            ).value
        )
        self.attachment_twist_topic = str(
            self.declare_parameter(
                "attachment_twist_topic", "/fake_attachment_point/twist"
            ).value
        )
        self.payload_pose_topic = str(
            self.declare_parameter("payload_pose_topic", "/fake_payload/pose").value
        )
        self.payload_twist_topic = str(
            self.declare_parameter("payload_twist_topic", "/fake_payload/twist").value
        )
        self.pickup_object_pose_topic = str(
            self.declare_parameter(
                "pickup_object_pose_topic", "/model/payload_model/pose"
            ).value
        )
        self.object_attached_topic = str(
            self.declare_parameter(
                "object_attached_topic", "/magnet/object_attached"
            ).value
        )
        self.magnet_tip_pose_topic = str(
            self.declare_parameter("magnet_tip_pose_topic", "/magnet_tip_pose").value
        )
        self.reference_topic = str(
            self.declare_parameter("reference_topic", "/join_planner/reference").value
        )
        self.marker_topic = str(
            self.declare_parameter("marker_topic", "/join_planner/markers").value
        )
        self.state_topic = str(
            self.declare_parameter("state_topic", "/join_planner/state").value
        )
        self.phase_topic = str(
            self.declare_parameter("phase_topic", "/join_planner/phase").value
        )
        self.handoff_ready_topic = str(
            self.declare_parameter(
                "handoff_ready_topic", "/join_planner/handoff_ready"
            ).value
        )
        self.safety_status_topic = str(
            self.declare_parameter(
                "obstacle_diagnostics_topic", "/join_planner/obstacle_diagnostics"
            ).value
        )
        self.drone_command_topic = str(
            self.declare_parameter("drone_command_topic", "drone_command").value
        )
        self.magnet_command_topic = str(
            self.declare_parameter("magnet_command_topic", "/magnet/command").value
        )
        self.external_landing_topic = str(
            self.declare_parameter(
                "external_landing_topic", "/join_planner/land_now"
            ).value
        )
        self.operator_advance_override_topic = str(
            self.declare_parameter(
                "operator_advance_override_topic", "/join_planner/advance_override"
            ).value
        )

        # ------------------------------------------------------------------
        # C1F.1 passive C++ transfer-backend shadow integration
        # ------------------------------------------------------------------
        # This commissioning bridge is deliberately shadow-only.  The existing
        # Python reference remains the sole controller reference in C1F.1; these
        # topics only provide the already-computed fixed body target to the C++
        # backend and observe its internal shadow reference/status.
        self.c1f1_shadow_transfer_enabled = bool(
            self.declare_parameter("c1f1_shadow_transfer_enabled", False).value
        )
        self.c1f1_shadow_target_topic = str(
            self.declare_parameter(
                "c1f1_shadow_target_topic",
                "/dynamic_planner/transfer_target",
            ).value
        )
        self.c1f1_shadow_enable_topic = str(
            self.declare_parameter(
                "c1f1_shadow_enable_topic",
                "/dynamic_planner/transfer_enable",
            ).value
        )
        self.c1f1_shadow_reference_topic = str(
            self.declare_parameter(
                "c1f1_shadow_reference_topic",
                "/dynamic_planner/transfer_reference",
            ).value
        )
        self.c1f1_shadow_status_topic = str(
            self.declare_parameter(
                "c1f1_shadow_status_topic",
                "/dynamic_planner/transfer_status",
            ).value
        )
        self.c1f1_shadow_reference_timeout_s = max(
            0.05,
            float(
                self.declare_parameter(
                    "c1f1_shadow_reference_timeout_s", 0.20
                ).value
            ),
        )
        self.c1f1_shadow_target_drift_warn_m = max(
            0.0,
            float(
                self.declare_parameter(
                    "c1f1_shadow_target_drift_warn_m", 0.02
                ).value
            ),
        )
        self.c1f2_cpp_authority_enabled = bool(
            self.declare_parameter("c1f2_cpp_authority_enabled", False).value
        )
        self.c1f2_handoff_position_tolerance_m = max(
            0.0,
            float(
                self.declare_parameter(
                    "c1f2_handoff_position_tolerance_m", 0.03
                ).value
            ),
        )
        self.c1f2_handoff_velocity_tolerance_mps = max(
            0.0,
            float(
                self.declare_parameter(
                    "c1f2_handoff_velocity_tolerance_mps", 0.10
                ).value
            ),
        )
        self.c1f2_handoff_acceleration_tolerance_mps2 = max(
            0.0,
            float(
                self.declare_parameter(
                    "c1f2_handoff_acceleration_tolerance_mps2", 0.50
                ).value
            ),
        )
        self.c1f2_prepare_max_lift_drift_m = max(
            0.0,
            float(
                self.declare_parameter(
                    "c1f2_prepare_max_lift_drift_m", 0.15
                ).value
            ),
        )
        self.c1f2_prepare_settle_speed_mps = max(
            0.0,
            float(
                self.declare_parameter(
                    "c1f2_prepare_settle_speed_mps", 0.08
                ).value
            ),
        )
        self.c1f2_prepare_settle_dwell_s = max(
            0.0,
            float(
                self.declare_parameter(
                    "c1f2_prepare_settle_dwell_s", 0.30
                ).value
            ),
        )
        self.c1f2_target_velocity_topic = str(
            self.declare_parameter(
                "c1f2_target_velocity_topic",
                "/dynamic_planner/transfer_target_velocity",
            ).value
        )
        self.c1f2_authority_topic = str(
            self.declare_parameter(
                "c1f2_authority_topic",
                "/dynamic_planner/transfer_authority",
            ).value
        )
        self.c1f2_authority_ack_topic = str(
            self.declare_parameter(
                "c1f2_authority_ack_topic",
                "/dynamic_planner/transfer_authority_ack",
            ).value
        )

        # ------------------------------------------------------------------
        # Mission definition
        # ------------------------------------------------------------------
        self.mission_mode = str(
            self.declare_parameter("mission_mode", "join").value
        ).strip().lower()
        self.mission = mission_for_mode(self.mission_mode)
        self.phase = self.mission.initial_phase
        self.phase_entry_time = self._physical_now_s()
        self.condition_start_time: Optional[float] = None
        self.last_transition_reason = "mission initialised"

        # M2B is a normal mission in the shared planner framework. B0 retains the
        # commissioned bootstrap/takeoff path; B1 adds the local attachment phases,
        # and B2 inserts the established C++ safe-transit authority before that same B1 logic.
        self.is_m2b = self.mission.name == "m2b_single_attachment"
        self.m2b_commissioning_stage = str(
            self.declare_parameter("m2b_commissioning_stage", "b0").value
        ).strip().lower()
        if self.is_m2b and self.m2b_commissioning_stage not in {"b0", "b1", "b2"}:
            raise ValueError("m2b_commissioning_stage must be 'b0', 'b1', or 'b2'.")
        self.m2b_b1_attachment_enabled = bool(
            self.declare_parameter("m2b_b1_attachment_enabled", True).value
        )
        self.m2b_b1_proof_enabled = bool(
            self.declare_parameter("m2b_b1_proof_enabled", True).value
        )
        self.m2b_assigned_plate_id = int(
            self.declare_parameter("m2b_assigned_plate_id", 0).value
        )
        if self.m2b_assigned_plate_id < 0:
            raise ValueError("m2b_assigned_plate_id must be non-negative")
        self.m2d_enabled = bool(
            self.declare_parameter("m2d_enabled", False).value
        )
        self.m2d_mission_permission_topic = str(
            self.declare_parameter(
                "m2d_mission_permission_topic", "/m2d/mission_permission"
            ).value
        )
        self.m2d_assigned_plate_topic = str(
            self.declare_parameter(
                "m2d_assigned_plate_topic", "/m2d/assigned_plate_id"
            ).value
        )
        self.m2d_mission_permission = not self.m2d_enabled
        self.m2d_assignment_received = not self.m2d_enabled
        # one C++ transit at a time (multi_drone_control): the safety checker hulls a
        # peer's advertised trajectory over the ego horizon and throws if a moving peer's
        # window ends early, which stalled simultaneous joins (T0009, T0012-T0014).
        # '' = off (his sequential M2D never has two vehicles in transit)
        self.m2d_transit_owner_topic = str(
            self.declare_parameter("m2d_transit_owner_topic", "").value
        )
        self.m2d_transit_owner = ""
        self._m2d_transit_wait_logged = False
        self.m2b_joint_truth_topic = str(
            self.declare_parameter(
                "m2b_joint_truth_topic", "/m2a/sim/joint_detached_truth"
            ).value
        )
        self.m2b_attachment_truth_source = str(self.declare_parameter("m2b_attachment_truth_source", "joint_truth").value).strip().lower()
        if self.m2b_attachment_truth_source not in {"joint_truth", "measured_detector"}:
            raise ValueError(
                "m2b_attachment_truth_source must be one of "
                '"joint_truth", "measured_detector"'
            )
        self.m2b_bootstrap_release_required = bool(
            self.declare_parameter("m2b_bootstrap_release_required", False).value
        )
        self.m2b_bootstrap_release_topic = str(
            self.declare_parameter("m2b_bootstrap_release_topic", "/m2b/bootstrap/release").value
        )
        self.m2b_raw_detach_request_topic = str(
            self.declare_parameter(
                "m2b_raw_detach_request_topic", "/m2b/sim/raw_detach_request"
            ).value
        )
        self.m2b_arm_permission_topic = str(
            self.declare_parameter(
                "m2b_arm_permission_topic", "/join_planner/arm_permission"
            ).value
        )
        self.m2b_require_recent_link_statistics = bool(
            self.declare_parameter("m2b_require_recent_link_statistics", False).value
        )
        self.m2b_link_statistics_topic = str(
            self.declare_parameter("m2b_link_statistics_topic", "elrs/link_statistics").value
        )
        self.m2b_link_statistics_timeout_s = float(
            self.declare_parameter("m2b_link_statistics_timeout_s", 0.5).value
        )
        if self.m2b_link_statistics_timeout_s <= 0.0:
            raise ValueError("m2b_link_statistics_timeout_s must be positive")
        self.m2b_last_link_statistics_s = None
        self.m2b_mpc_mode_topic = str(
            self.declare_parameter("m2b_mpc_mode_topic", "/join_planner/mpc_mode").value
        )
        self.m2b_arming_state_topic = str(
            self.declare_parameter(
                "m2b_arming_state_topic", "drone_arming_state_feedback"
            ).value
        )
        self.m2b_attachment_diagnostics_topic = str(
            self.declare_parameter(
                "m2b_attachment_diagnostics_topic", "/m2a/attachment/diagnostics"
            ).value
        )
        self.m2b_proof_requested_topic = str(
            self.declare_parameter(
                "m2b_proof_requested_topic", "/m2a/attachment/proof_requested"
            ).value
        )
        self.m2b_detector_reset_topic = str(
            self.declare_parameter(
                "m2b_detector_reset_topic", "/m2a/attachment/reset"
            ).value
        )
        self.m2b_attempt_topic = str(
            self.declare_parameter("m2b_attempt_topic", "/m2b/attempt_number").value
        )
        self.m2b_latch_stable_topic = str(
            self.declare_parameter("m2b_latch_stable_topic", "/m2b/latch_stable").value
        )
        self.m2b_latch_stability_topic = str(
            self.declare_parameter("m2b_latch_stability_topic", "/m2b/latch_stability").value
        )
        self.m2b_pendulum_topic = str(
            self.declare_parameter("m2b_pendulum_topic", "/pendulum_swing_state").value
        )
        self.m2b_ring_pose_source = str(
            self.declare_parameter("m2b_ring_pose_source", "fixed").value
        ).strip().lower()
        self.m2b_ring_pose_topic = str(
            self.declare_parameter(
                "m2b_ring_pose_topic", "/model/payload_model/pose"
            ).value
        )
        self.m2b_ring_pose_index = int(
            self.declare_parameter("m2b_ring_pose_index", 1).value
        )
        self.m2b_ring_pose_timeout_s = max(
            0.02, float(self.declare_parameter("m2b_ring_pose_timeout_s", 0.25).value)
        )
        if self.m2b_ring_pose_source not in {"fixed", "pose_array"}:
            raise ValueError("m2b_ring_pose_source must be 'fixed' or 'pose_array'")
        self.m2b_fixed_ring_position = np.array([
            float(self.declare_parameter("m2b_fixed_ring_x", 0.0).value),
            float(self.declare_parameter("m2b_fixed_ring_y", 0.0).value),
            float(self.declare_parameter("m2b_fixed_ring_z", 0.035).value),
        ], dtype=float)
        self.m2b_fixed_ring_yaw = float(
            self.declare_parameter("m2b_fixed_ring_yaw", 0.0).value
        )
        self.m2b_tether_anchor_body = np.array([
            float(self.declare_parameter("m2b_tether_anchor_body_x", 0.0).value),
            float(self.declare_parameter("m2b_tether_anchor_body_y", 0.0).value),
            float(self.declare_parameter("m2b_tether_anchor_body_z", -0.04).value),
        ], dtype=float)
        self.m2b_anchor_to_contact_length_m = float(
            self.declare_parameter("m2b_anchor_to_contact_length_m", 0.475).value
        )
        if self.m2b_anchor_to_contact_length_m <= 0.0:
            raise ValueError("m2b_anchor_to_contact_length_m must be positive")
        self.m2b_ring_geometry = RingNetGeometry()
        self.m2b_joint_truth_timeout_s = max(
            0.02,
            float(self.declare_parameter("m2b_joint_truth_timeout_s", 0.35).value),
        )
        self.m2b_config = M2BConfig(
            bootstrap_detach_dwell_s=float(
                self.declare_parameter("m2b_bootstrap_detach_dwell_s", 0.15).value
            ),
            bootstrap_settle_s=float(
                self.declare_parameter("m2b_bootstrap_settle_s", 0.50).value
            ),
            bootstrap_timeout_s=float(
                self.declare_parameter("m2b_bootstrap_timeout_s", 3.0).value
            ),
            capture_height_m=float(self.declare_parameter("m2b_capture_height_m", 0.30).value),
            settle_position_tolerance_m=float(self.declare_parameter("m2b_settle_position_tolerance_m", 0.05).value),
            settle_speed_mps=float(self.declare_parameter("m2b_settle_speed_mps", 0.03).value),
            settle_swing_deg=float(self.declare_parameter("m2b_settle_swing_deg", 3.0).value),
            settle_dwell_s=float(self.declare_parameter("m2b_settle_dwell_s", 0.75).value),
            evidence_timeout_s=float(self.declare_parameter("m2b_evidence_timeout_s", 0.35).value),
            approach_evidence_loss_abort_s=float(
                self.declare_parameter("m2b_approach_evidence_loss_abort_s", 0.75).value
            ),
            high_low_split_m=float(self.declare_parameter("m2b_high_low_split_m", 0.08).value),
            high_xy_tolerance_m=float(self.declare_parameter("m2b_high_xy_tolerance_m", 0.05).value),
            low_xy_tolerance_m=float(self.declare_parameter("m2b_low_xy_tolerance_m", 0.025).value),
            gross_xy_abort_m=float(self.declare_parameter("m2b_gross_xy_abort_m", 0.10).value),
            alignment_pause_timeout_s=float(self.declare_parameter("m2b_alignment_pause_timeout_s", 2.0).value),
            descent_speed_mps=float(self.declare_parameter("m2b_descent_speed_mps", 0.04).value),
            magnet_enable_height_m=float(self.declare_parameter("m2b_magnet_enable_height_m", 0.04).value),
            magnet_enable_relative_speed_mps=float(self.declare_parameter("m2b_magnet_enable_relative_speed_mps", 0.12).value),
            capture_hold_normal_m=float(self.declare_parameter("m2b_capture_hold_normal_m", 0.020).value),
            physical_latch_timeout_s=float(self.declare_parameter("m2b_physical_latch_timeout_s", 0.75).value),
            latch_settle_dwell_s=float(self.declare_parameter("m2b_latch_settle_dwell_s", 0.50).value),
            latch_settle_timeout_s=float(self.declare_parameter("m2b_latch_settle_timeout_s", 1.50).value),
            latch_settle_speed_mps=float(self.declare_parameter("m2b_latch_settle_speed_mps", 0.08).value),
            latch_settle_accel_mps2=float(self.declare_parameter("m2b_latch_settle_accel_mps2", 2.0).value),
            latch_abort_speed_mps=float(self.declare_parameter("m2b_latch_abort_speed_mps", 0.50).value),
            latch_abort_accel_mps2=float(self.declare_parameter("m2b_latch_abort_accel_mps2", 8.0).value),
            latch_abort_displacement_m=float(self.declare_parameter("m2b_latch_abort_displacement_m", 0.05).value),
            proof_timeout_s=float(self.declare_parameter("m2b_proof_timeout_s", 2.0).value),
            proof_direction_plate_xyz=(
                float(self.declare_parameter("m2b_proof_direction_radial", 1.0).value),
                float(self.declare_parameter("m2b_proof_direction_tangential", 0.0).value),
                float(self.declare_parameter("m2b_proof_direction_normal", 0.0).value),
            ),
            attached_hold_angle_deg=float(self.declare_parameter("m2b_attached_hold_angle_deg", 15.0).value),
            proof_command_m=float(self.declare_parameter("m2b_proof_command_m", 0.025).value),
            proof_speed_mps=float(self.declare_parameter("m2b_proof_speed_mps", 0.03).value),
            proof_min_measured_excitation_m=float(
                self.declare_parameter("m2b_proof_min_measured_excitation_m", 0.012).value
            ),
            attached_loss_grace_s=float(self.declare_parameter("m2b_attached_loss_grace_s", 0.50).value),
            detach_verify_timeout_s=float(self.declare_parameter("m2b_detach_verify_timeout_s", 1.50).value),
            retreat_rise_m=float(self.declare_parameter("m2b_retreat_rise_m", 0.20).value),
            max_attempts=int(self.declare_parameter("m2b_max_attempts", 3).value),
        )

        # ------------------------------------------------------------------
        # Planning and reference settings
        # ------------------------------------------------------------------
        self.publish_rate_hz = max(
            1.0, float(self.declare_parameter("publish_rate_hz", 30.0).value)
        )
        self.dt = 1.0 / self.publish_rate_hz
        self.c1f2_exit_bridge_duration_s = max(
            self.dt,
            float(
                self.declare_parameter(
                    "c1f2_exit_bridge_duration_s", 1.50
                ).value
            ),
        )
        # C1F.6: the 1.5 s bridge duration is a configurable commissioning
        # value, not a literature-derived universal constant.  The transition
        # gate now checks the exact quintic bridge against the same dynamic
        # envelopes used by the C++ planner instead of demanding a magic
        # relative-velocity threshold.
        self.c1f2_bridge_feasibility_enabled = bool(
            self.declare_parameter("c1f2_bridge_feasibility_enabled", True).value
        )
        self.c1f2_bridge_max_velocity = np.array([
            max(0.05, float(self.declare_parameter("c1f2_bridge_max_velocity_x_mps", 1.0).value)),
            max(0.05, float(self.declare_parameter("c1f2_bridge_max_velocity_y_mps", 1.0).value)),
            max(0.05, float(self.declare_parameter("c1f2_bridge_max_velocity_z_mps", 1.0).value)),
        ], dtype=float)
        self.c1f2_bridge_max_acceleration = np.array([
            max(0.05, float(self.declare_parameter("c1f2_bridge_max_acceleration_x_mps2", 1.0).value)),
            max(0.05, float(self.declare_parameter("c1f2_bridge_max_acceleration_y_mps2", 1.0).value)),
            max(0.05, float(self.declare_parameter("c1f2_bridge_max_acceleration_z_mps2", 1.5).value)),
        ], dtype=float)
        self.c1f2_bridge_max_jerk = np.array([
            max(0.05, float(self.declare_parameter("c1f2_bridge_max_jerk_x_mps3", 4.0).value)),
            max(0.05, float(self.declare_parameter("c1f2_bridge_max_jerk_y_mps3", 4.0).value)),
            max(0.05, float(self.declare_parameter("c1f2_bridge_max_jerk_z_mps3", 4.0).value)),
        ], dtype=float)
        self.c1f2_bridge_feasibility_sample_dt_s = max(
            0.005,
            float(self.declare_parameter("c1f2_bridge_feasibility_sample_dt_s", 0.02).value),
        )
        self.horizon_seconds = max(
            0.5, float(self.declare_parameter("horizon_seconds", 3.0).value)
        )
        self.horizon_samples = max(
            2, int(math.ceil(self.horizon_seconds / self.dt))
        )
        self.min_reference_z = float(
            self.declare_parameter("min_reference_z", 0.60).value
        )
        self.max_reference_speed = max(
            0.05, float(self.declare_parameter("max_reference_speed", 2.0).value)
        )

        reference_min_duration = max(
            0.25,
            float(self.declare_parameter("reference_min_duration_s", 1.0).value),
        )
        reference_max_duration = max(
            reference_min_duration,
            float(self.declare_parameter("reference_max_duration_s", 6.0).value),
        )
        transfer_config = TransferConfig(
            dt=self.dt,
            horizon_samples=self.horizon_samples,
            nominal_speed=max(
                0.05,
                float(self.declare_parameter("reference_nominal_speed", 0.30).value),
            ),
            duration_scale=max(
                1.0,
                float(self.declare_parameter("reference_duration_scale", 1.5).value),
            ),
            min_duration=reference_min_duration,
            max_duration=reference_max_duration,
            tracking_error_soft=max(
                0.0,
                float(
                    self.declare_parameter(
                        "reference_tracking_error_soft_m", 0.10
                    ).value
                ),
            ),
            tracking_error_hard=max(
                0.02,
                float(
                    self.declare_parameter(
                        "reference_tracking_error_hard_m", 0.35
                    ).value
                ),
            ),
            minimum_progress_scale=float(
                np.clip(
                    self.declare_parameter(
                        "reference_min_progress_scale", 0.10
                    ).value,
                    0.0,
                    1.0,
                )
            ),
            min_reference_z=self.min_reference_z,
            max_reference_speed=self.max_reference_speed,
        )
        self.transfer_generator = VirtualTransferGenerator(transfer_config)

        self.attach_ready_target_lead_time = max(
            0.0,
            float(
                self.declare_parameter("attach_ready_target_lead_time", 0.20).value
            ),
        )

        # ------------------------------------------------------------------
        # Diagnostics-only static collision checking
        # ------------------------------------------------------------------
        self.collision_diagnostics_enabled = bool(
            self.declare_parameter("collision_diagnostics_enabled", True).value
        )
        self.drone_collision_radius = max(
            0.0,
            float(self.declare_parameter("drone_collision_radius", 0.20).value),
        )
        self.collision_safety_margin = max(
            0.0,
            float(self.declare_parameter("collision_safety_margin", 0.10).value),
        )
        self.magnet_collision_radius = max(
            0.0,
            float(self.declare_parameter("magnet_collision_radius", 0.05).value),
        )
        self.magnet_collision_margin = max(
            0.0,
            float(self.declare_parameter("magnet_collision_margin", 0.05).value),
        )
        self.cable_collision_radius = max(
            0.0,
            float(self.declare_parameter("cable_collision_radius", 0.005).value),
        )
        self.cable_collision_margin = max(
            0.0,
            float(self.declare_parameter("cable_collision_margin", 0.05).value),
        )

        # V5A carried-payload profile.  These parameters define one shared source
        # of truth for passive collision geometry now and geometry-aware pickup in
        # V5B.  V5A does not alter any pickup target or reference.
        self.payload_profile_name = str(
            self.declare_parameter("payload_name", "simulation_payload").value
        ).strip()
        self.payload_collision_enabled = bool(
            self.declare_parameter("payload_collision_enabled", True).value
        )
        self.payload_collision_dimensions = np.array(
            [
                float(self.declare_parameter("payload_collision_size_x", 0.16).value),
                float(self.declare_parameter("payload_collision_size_y", 0.16).value),
                float(self.declare_parameter("payload_collision_size_z", 0.16).value),
            ],
            dtype=float,
        )
        self.payload_collision_margin = max(
            0.0,
            float(self.declare_parameter("payload_collision_margin", 0.05).value),
        )
        self.payload_pose_reference = str(
            self.declare_parameter("payload_pose_reference", "centre").value
        ).strip().lower()
        self.payload_geometry_origin_from_pose = np.array(
            [
                float(self.declare_parameter("payload_geometry_origin_from_pose_x", 0.0).value),
                float(self.declare_parameter("payload_geometry_origin_from_pose_y", 0.0).value),
                float(self.declare_parameter("payload_geometry_origin_from_pose_z", 0.0).value),
            ],
            dtype=float,
        )
        self.payload_pickup_point = np.array(
            [
                float(self.declare_parameter("payload_pickup_point_x", 0.0).value),
                float(self.declare_parameter("payload_pickup_point_y", 0.0).value),
                float(self.declare_parameter("payload_pickup_point_z", 0.08).value),
            ],
            dtype=float,
        )
        self.magnet_marker_to_contact_face = np.array(
            [
                float(self.declare_parameter("magnet_marker_to_contact_face_x", 0.0).value),
                float(self.declare_parameter("magnet_marker_to_contact_face_y", 0.0).value),
                float(self.declare_parameter("magnet_marker_to_contact_face_z", -0.03).value),
            ],
            dtype=float,
        )
        self.payload_assume_level = bool(
            self.declare_parameter("payload_assume_level", True).value
        )
        self.payload_pose_timeout_s = max(
            0.05,
            float(self.declare_parameter("payload_pose_timeout_s", 0.50).value),
        )
        self.payload_profile = PayloadGeometryProfile(
            name=self.payload_profile_name,
            dimensions=self.payload_collision_dimensions,
            geometry_origin_from_pose=self.payload_geometry_origin_from_pose,
            pickup_point=self.payload_pickup_point,
            magnet_marker_to_contact_face=self.magnet_marker_to_contact_face,
            pose_reference=self.payload_pose_reference,
            assume_level=self.payload_assume_level,
            collision_enabled=self.payload_collision_enabled,
            collision_margin=self.payload_collision_margin,
        )

        # V5B geometry-aware pickup.  V5B1 added passive diagnostics.  V5B2 can
        # explicitly activate the same physical target during pickup, but defaults
        # to the exact legacy fixed-clearance behaviour.
        self.use_geometry_aware_pickup = bool(
            self.declare_parameter("use_geometry_aware_pickup", False).value
        )
        self.pickup_contact_gap = max(
            0.0,
            float(self.declare_parameter("pickup_contact_gap", 0.0).value),
        )
        self.pickup_overtravel = max(
            0.0,
            float(self.declare_parameter("pickup_overtravel", 0.002).value),
        )
        self.pickup_max_overtravel = max(
            0.0,
            float(self.declare_parameter("pickup_max_overtravel", 0.005).value),
        )
        self.support_surface_z = float(
            self.declare_parameter("support_surface_z", 0.0).value
        )

        self.static_obstacle_count = max(
            0, int(self.declare_parameter("static_obstacle_count", 0).value)
        )
        self.static_obstacles = []
        for obstacle_index in range(self.static_obstacle_count):
            prefix = f"static_obstacle_{obstacle_index}"
            obstacle_id = str(
                self.declare_parameter(
                    f"{prefix}_id", f"obstacle_{obstacle_index}"
                ).value
            )
            centre = np.array(
                [
                    float(self.declare_parameter(f"{prefix}_x", 0.0).value),
                    float(self.declare_parameter(f"{prefix}_y", 0.0).value),
                    float(self.declare_parameter(f"{prefix}_z", 0.0).value),
                ],
                dtype=float,
            )
            radius = max(
                0.0,
                float(self.declare_parameter(f"{prefix}_radius", 0.25).value),
            )
            self.static_obstacles.append(
                StaticSphereObstacle(
                    obstacle_id=obstacle_id,
                    centre=centre,
                    radius=radius,
                )
            )

        # M2A moving-obstacle prediction. This stage is deliberately passive:
        # it consumes one obstacle state, runs the ROS-independent M1 predictor,
        # and publishes the resulting reachable tube in RViz. It does not yet
        # modify A*, occupancy, the safe corridor, or the controller reference.
        self.moving_obstacle_prediction_enabled = bool(
            self.declare_parameter(
                "moving_obstacle_prediction_enabled",
                True,
            ).value
        )
        self.moving_obstacle_state_topic = str(
            self.declare_parameter(
                "moving_obstacle_state_topic",
                "/fake_moving_obstacle/state",
            ).value
        )
        self.moving_obstacle_id = str(
            self.declare_parameter(
                "moving_obstacle_id",
                "moving_obstacle_0",
            ).value
        )
        self.moving_obstacle_geometry_radius_m = max(
            0.0,
            float(
                self.declare_parameter(
                    "moving_obstacle_geometry_radius_m",
                    0.18,
                ).value
            ),
        )
        self.moving_obstacle_state_timeout_s = max(
            0.05,
            float(
                self.declare_parameter(
                    "moving_obstacle_state_timeout_s",
                    0.50,
                ).value
            ),
        )
        self.moving_obstacle_show_prediction_markers = bool(
            self.declare_parameter(
                "moving_obstacle_show_prediction_markers",
                True,
            ).value
        )

        self.moving_obstacle_prediction_config = MovingObstaclePredictionConfig(
            horizon_s=max(
                0.05,
                float(
                    self.declare_parameter(
                        "moving_obstacle_prediction_horizon_s",
                        self.horizon_seconds,
                    ).value
                ),
            ),
            time_step_s=max(
                0.01,
                float(
                    self.declare_parameter(
                        "moving_obstacle_prediction_time_step_s",
                        0.25,
                    ).value
                ),
            ),
            velocity_filter_tau_s=max(
                0.0,
                float(
                    self.declare_parameter(
                        "moving_obstacle_velocity_filter_tau_s",
                        0.20,
                    ).value
                ),
            ),
            position_uncertainty_m=max(
                0.0,
                float(
                    self.declare_parameter(
                        "moving_obstacle_position_uncertainty_m",
                        0.03,
                    ).value
                ),
            ),
            velocity_uncertainty_mps=max(
                0.0,
                float(
                    self.declare_parameter(
                        "moving_obstacle_velocity_uncertainty_mps",
                        0.10,
                    ).value
                ),
            ),
            acceleration_uncertainty_mps2=max(
                0.0,
                float(
                    self.declare_parameter(
                        "moving_obstacle_acceleration_uncertainty_mps2",
                        0.15,
                    ).value
                ),
            ),
        )

        # Passive visual-only 3D voxel A*.  Search is deliberately decoupled
        # from the 30 Hz reference callback and never replaces the nominal path.
        self.visual_astar_enabled = bool(
            self.declare_parameter("visual_astar_enabled", True).value
        )
        self.visual_astar_resolution = max(
            0.01, float(self.declare_parameter("visual_astar_resolution", 0.08).value)
        )
        self.visual_astar_replan_rate_hz = max(
            0.1,
            float(self.declare_parameter("visual_astar_replan_rate_hz", 2.0).value),
        )
        self.visual_astar_max_expansions = max(
            1,
            int(self.declare_parameter("visual_astar_max_expansions", 150000).value),
        )
        self.visual_astar_max_planning_time_s = max(
            0.001,
            float(
                self.declare_parameter(
                    "visual_astar_max_planning_time_s", 0.25
                ).value
            ),
        )
        self.visual_astar_nearest_free_radius_m = max(
            0.0,
            float(
                self.declare_parameter(
                    "visual_astar_nearest_free_radius_m", 0.40
                ).value
            ),
        )
        self.visual_astar_local_margin_xy_m = max(
            0.10,
            float(
                self.declare_parameter(
                    "visual_astar_local_margin_xy_m", 1.25
                ).value
            ),
        )
        self.visual_astar_local_margin_z_m = max(
            0.10,
            float(
                self.declare_parameter(
                    "visual_astar_local_margin_z_m", 0.75
                ).value
            ),
        )
        self.visual_astar_bounds_min = np.array(
            [
                float(self.declare_parameter("visual_astar_bounds_min_x", -5.0).value),
                float(self.declare_parameter("visual_astar_bounds_min_y", -5.0).value),
                float(self.declare_parameter("visual_astar_bounds_min_z", 0.0).value),
            ],
            dtype=float,
        )
        self.visual_astar_bounds_max = np.array(
            [
                float(self.declare_parameter("visual_astar_bounds_max_x", 5.0).value),
                float(self.declare_parameter("visual_astar_bounds_max_y", 5.0).value),
                float(self.declare_parameter("visual_astar_bounds_max_z", 4.0).value),
            ],
            dtype=float,
        )
        self.visual_astar_rejoin_min_lookahead_s = max(
            0.0,
            float(
                self.declare_parameter(
                    "visual_astar_rejoin_min_lookahead_s", 0.35
                ).value
            ),
        )
        self.visual_astar_rejoin_preferred_lookahead_s = max(
            self.visual_astar_rejoin_min_lookahead_s,
            float(
                self.declare_parameter(
                    "visual_astar_rejoin_preferred_lookahead_s", 1.00
                ).value
            ),
        )
        self.visual_astar_rejoin_min_separation_m = max(
            0.0,
            float(
                self.declare_parameter(
                    "visual_astar_rejoin_min_separation_m", 0.20
                ).value
            ),
        )
        self.visual_astar_candidate_speed_mps = max(
            0.01,
            float(
                self.declare_parameter(
                    "visual_astar_candidate_speed_mps", 0.35
                ).value
            ),
        )
        self.declare_parameter(
            "visual_astar_show_occupancy_voxels",
            False,
        )

        # Passive convex-polyhedral Bernstein trajectory smoothing.  This is
        # visual/logging only at the M2A restart; NMPC still receives the nominal
        # mission reference.  Keep only parameters consumed by the active smoother
        # or by the measured-acceleration estimate supplied to it.
        self.visual_trajectory_enabled = bool(
            self.declare_parameter("visual_trajectory_enabled", True).value
        )
        self.visual_trajectory_clearance_margin_m = max(
            0.0,
            float(
                self.declare_parameter(
                    "visual_trajectory_clearance_margin_m", 0.05
                ).value
            ),
        )
        self.visual_trajectory_max_speed_mps = max(
            0.05,
            float(
                self.declare_parameter(
                    "visual_trajectory_max_speed_mps", 1.0
                ).value
            ),
        )
        self.visual_trajectory_max_acceleration_mps2 = max(
            0.05,
            float(
                self.declare_parameter(
                    "visual_trajectory_max_acceleration_mps2", 1.5
                ).value
            ),
        )
        self.visual_trajectory_validation_headroom_ratio = max(
            1.0,
            float(
                self.declare_parameter(
                    "visual_trajectory_validation_headroom_ratio", 1.10
                ).value
            ),
        )
        self.visual_trajectory_use_target_acceleration = bool(
            self.declare_parameter(
                "visual_trajectory_use_target_acceleration", False
            ).value
        )
        self.visual_trajectory_acceleration_filter_tau_s = max(
            0.01,
            float(
                self.declare_parameter(
                    "visual_trajectory_acceleration_filter_tau_s", 0.15
                ).value
            ),
        )
        self.visual_trajectory_time_stretch_safety_factor = max(
            1.0,
            float(
                self.declare_parameter(
                    "visual_trajectory_time_stretch_safety_factor", 1.05
                ).value
            ),
        )
        self.visual_trajectory_baseline_nominal_speed_mps = max(
            0.05,
            float(
                self.declare_parameter(
                    "visual_trajectory_baseline_nominal_speed_mps", 0.50
                ).value
            ),
        )
        self.visual_trajectory_minimum_segment_time_s = max(
            0.05,
            float(
                self.declare_parameter(
                    "visual_trajectory_minimum_segment_time_s", 0.18
                ).value
            ),
        )
        self.visual_trajectory_maximum_segment_time_s = max(
            self.visual_trajectory_minimum_segment_time_s,
            float(
                self.declare_parameter(
                    "visual_trajectory_maximum_segment_time_s", 6.0
                ).value
            ),
        )
        # These two parameter names are retained for command/config compatibility.
        # They now configure the active convex Bernstein/OSQP smoother only.
        self.visual_trajectory_direct_bezier_solver_iterations = max(
            20,
            int(
                self.declare_parameter(
                    "visual_trajectory_direct_bezier_solver_iterations", 3000
                ).value
            ),
        )
        self.visual_trajectory_direct_bezier_solver_time_s = max(
            0.05,
            float(
                self.declare_parameter(
                    "visual_trajectory_direct_bezier_solver_time_s", 0.75
                ).value
            ),
        )

        # ------------------------------------------------------------------
        # Geometry and target sources
        # ------------------------------------------------------------------
        self.magnet_drop_below_quad = max(
            0.0,
            float(self.declare_parameter("magnet_drop_below_quad", 0.50).value),
        )
        self.approach_offset_x = float(
            self.declare_parameter("approach_offset_x", 0.0).value
        )
        self.approach_offset_y = float(
            self.declare_parameter("approach_offset_y", 0.0).value
        )
        self.approach_offset_z = float(
            self.declare_parameter("approach_offset_z", 0.30).value
        )
        self.final_attach_offset_z = float(
            self.declare_parameter("final_attach_offset_z", 0.05).value
        )

        self.use_static_pickup_object = bool(
            self.declare_parameter("use_static_pickup_object", True).value
        )
        self.static_pickup_object = np.array(
            [
                float(self.declare_parameter("static_pickup_object_x", 0.0).value),
                float(self.declare_parameter("static_pickup_object_y", 0.0).value),
                float(self.declare_parameter("static_pickup_object_z", 0.0).value),
            ],
            dtype=float,
        )
        self.static_pickup_object_yaw = float(
            self.declare_parameter("static_pickup_object_yaw", 0.0).value
        )
        self.pickup_object_index = int(
            self.declare_parameter("pickup_object_index", 1).value
        )

        self.drop_point_source = str(
            self.declare_parameter("drop_point_source", "static").value
        ).strip().lower()
        self.static_drop_point = np.array(
            [
                float(self.declare_parameter("static_drop_point_x", 0.0).value),
                float(self.declare_parameter("static_drop_point_y", 0.0).value),
                float(self.declare_parameter("static_drop_point_z", 0.0).value),
            ],
            dtype=float,
        )
        self.drop_offset = np.array(
            [
                float(self.declare_parameter("drop_offset_x", 0.0).value),
                float(self.declare_parameter("drop_offset_y", 0.0).value),
                float(self.declare_parameter("drop_offset_z", 0.15).value),
            ],
            dtype=float,
        )

        # ------------------------------------------------------------------
        # Phase motion settings
        # ------------------------------------------------------------------
        # Takeoff parameters
        self.takeoff_height = float(
            self.declare_parameter("takeoff_height", 0.50).value
        )
        self.takeoff_speed = float(
            self.declare_parameter("takeoff_speed", 0.15).value
        )
        # Takeoff is the one commissioned RATE_CONTROLLED phase where the old
        # instantaneous velocity step has repeatedly shown transient sag.  Keep
        # the existing 0.15 m/s speed cap but ramp into/out of it with a smooth,
        # acceleration-bounded profile.  Other low-speed attachment phases retain
        # their already-commissioned constant-rate semantics.
        self.takeoff_reference_acceleration_limit_mps2 = max(
            0.05,
            float(
                self.declare_parameter(
                    "takeoff_reference_acceleration_limit_mps2", 0.50
                ).value
            ),
        )

        self.takeoff_start_position = None

        # Pickup phase settings
        self.pickup_approach_clearance = float(
            self.declare_parameter("pickup_approach_clearance", 0.35).value
        )
        self.pickup_attach_clearance = float(
            self.declare_parameter("pickup_attach_clearance", 0.03).value
        )
        self.pickup_lift_height = float(
            self.declare_parameter("pickup_lift_height", 0.45).value
        )
        self.pickup_descent_speed = max(
            0.001,
            float(self.declare_parameter("pickup_descent_speed", 0.04).value),
        )
        self.pickup_lift_speed = max(
            0.001,
            float(self.declare_parameter("pickup_lift_speed", 0.06).value),
        )
        self.pickup_settle_time_s = max(
            0.0,
            float(self.declare_parameter("pickup_settle_time_s", 1.0).value),
        )
        self.pickup_xy_tolerance = max(
            0.0, float(self.declare_parameter("pickup_xy_tolerance", 0.08).value)
        )
        self.pickup_lift_xy_tolerance = max(
            0.0,
            float(
                self.declare_parameter("pickup_lift_xy_tolerance", 0.08).value
            ),
        )
        self.pickup_lift_xy_dwell_s = max(
            0.0,
            float(
                self.declare_parameter("pickup_lift_xy_dwell_s", 0.30).value
            ),
        )
        self.pickup_z_tolerance = max(
            0.0, float(self.declare_parameter("pickup_z_tolerance", 0.10).value)
        )
        self.loaded_lift_z_tolerance = max(
            0.0,
            float(
                self.declare_parameter(
                    "loaded_lift_z_tolerance", self.pickup_z_tolerance
                ).value
            ),
        )
        self.pickup_tip_speed_tolerance = max(
            0.0,
            float(
                self.declare_parameter("pickup_tip_speed_tolerance", 0.25).value
            ),
        )
        self.attach_wait_timeout_s = max(
            0.0,
            float(self.declare_parameter("attach_wait_timeout_s", 8.0).value),
        )
        self.lift_complete_object_clearance = max(
            0.0,
            float(
                self.declare_parameter(
                    "lift_complete_object_clearance", 0.10
                ).value
            ),
        )

        self.drop_approach_clearance = float(
            self.declare_parameter("drop_approach_clearance", 0.35).value
        )
        self.drop_release_clearance = float(
            self.declare_parameter("drop_release_clearance", 0.14).value
        )
        self.drop_clear_height = float(
            self.declare_parameter("drop_clear_height", 0.45).value
        )
        self.drop_descent_speed = max(
            0.001,
            float(self.declare_parameter("drop_descent_speed", 0.04).value),
        )
        self.drop_clear_speed = max(
            0.001,
            float(self.declare_parameter("drop_clear_speed", 0.08).value),
        )
        self.drop_settle_time_s = max(
            0.0,
            float(self.declare_parameter("drop_settle_time_s", 1.0).value),
        )
        self.drop_wait_time_s = max(
            0.0, float(self.declare_parameter("drop_wait_time_s", 0.40).value)
        )

        self.descend_rate = max(
            0.001, float(self.declare_parameter("descend_rate", 0.08).value)
        )
        self.auto_descend = bool(
            self.declare_parameter("auto_descend", False).value
        )

        # ------------------------------------------------------------------
        # Transition guards
        # ------------------------------------------------------------------
        self.approach_xy_threshold = max(
            0.0,
            float(self.declare_parameter("approach_xy_threshold", 0.15).value),
        )
        # Coarse capture radius used specifically when the C++ loaded-transfer
        # planner approaches the moving drop target.  Keep this distinct from
        # both the generic approach threshold and the tighter precision-match
        # threshold: C++ only needs to enter a useful rendezvous neighbourhood,
        # after which the C2 target-frame bridge acquires the target motion.
        self.c1f2_nav_capture_xy_threshold_m = max(
            0.0,
            float(
                self.declare_parameter(
                    "c1f2_nav_capture_xy_threshold_m", 0.18
                ).value
            ),
        )
        self.c1f2_nav_capture_z_threshold_m = max(
            0.0,
            float(
                self.declare_parameter(
                    "c1f2_nav_capture_z_threshold_m", 0.15
                ).value
            ),
        )
        self.reattach_transit_xy_threshold = max(
            self.approach_xy_threshold,
            float(
                self.declare_parameter(
                    "reattach_transit_xy_threshold", 0.40
                ).value
            ),
        )
        self.match_xy_threshold = max(
            0.0, float(self.declare_parameter("match_xy_threshold", 0.10).value)
        )
        self.match_xy_velocity_threshold = max(
            0.0,
            float(
                self.declare_parameter(
                    "match_xy_velocity_threshold", 0.10
                ).value
            ),
        )
        self.dwell_time_s = max(
            0.0, float(self.declare_parameter("dwell_time_s", 1.0).value)
        )
        self.descend_abort_xy_threshold = max(
            self.match_xy_threshold,
            float(
                self.declare_parameter(
                    "descend_abort_xy_threshold", 0.25
                ).value
            ),
        )
        self.descend_abort_xy_velocity_threshold = max(
            self.match_xy_velocity_threshold,
            float(
                self.declare_parameter(
                    "descend_abort_xy_velocity_threshold", 0.30
                ).value
            ),
        )
        self.takeoff_detection_height = max(
            0.0,
            float(self.declare_parameter("takeoff_detection_height", 0.12).value),
        )
        self.input_timeout_s = max(
            0.05, float(self.declare_parameter("input_timeout_s", 0.50).value)
        )

        # ------------------------------------------------------------------
        # Measured magnet-tip compensation
        # ------------------------------------------------------------------
        self.use_measured_magnet_tip = bool(
            self.declare_parameter("use_measured_magnet_tip", False).value
        )
        self.m2b_takeoff_require_magnet_clearance = bool(
            self.declare_parameter(
                "m2b_takeoff_require_magnet_clearance", False
            ).value
        )
        self.m2b_takeoff_magnet_clearance_m = max(
            0.0,
            float(self.declare_parameter("m2b_takeoff_magnet_clearance_m", 0.03).value),
        )
        self.m2b_takeoff_hover_timeout_s = max(
            0.0,
            float(self.declare_parameter("m2b_takeoff_hover_timeout_s", 0.0).value),
        )
        self.enable_pickup_tip_xy_compensation = bool(
            self.declare_parameter(
                "enable_pickup_tip_xy_compensation", True
            ).value
        )
        self.pickup_tip_xy_compensation_gain = max(
            0.0,
            float(
                self.declare_parameter(
                    "pickup_tip_xy_compensation_gain", 0.50
                ).value
            ),
        )
        self.pickup_tip_xy_compensation_time_constant_s = max(
            0.0,
            float(
                self.declare_parameter(
                    "pickup_tip_xy_compensation_time_constant_s", 1.50
                ).value
            ),
        )
        self.pickup_tip_xy_compensation_max_m = max(
            0.0,
            float(
                self.declare_parameter(
                    "pickup_tip_xy_compensation_max_m", 0.10
                ).value
            ),
        )
        self.pickup_tip_xy_compensation_raw = np.zeros(2, dtype=float)
        self.pickup_tip_xy_compensation_filtered = np.zeros(2, dtype=float)
        self.pickup_tip_xy_compensation_measurement_active = False

        # ------------------------------------------------------------------
        # Landing and simulation safety
        # ------------------------------------------------------------------
        self.enable_external_landing_trigger = bool(
            self.declare_parameter("enable_external_landing_trigger", True).value
        )
        # Commissioning-only progression aid. Disabled by default so simulation
        # and unattended mission semantics are unchanged unless the operator
        # explicitly opts in at launch.
        self.enable_operator_advance_override = bool(
            self.declare_parameter("enable_operator_advance_override", False).value
        )
        # IRL commissioning gates. Defaults are false so simulation and the
        # commissioned mission remain unchanged unless explicitly enabled.
        # hold_after_lift keeps the loaded vehicle at the completed lift endpoint.
        # hold_before_drop allows C++ transfer/precision settle but prevents descent
        # and magnet release when no physical basket is present.
        self.commissioning_hold_after_lift = bool(
            self.declare_parameter("commissioning_hold_after_lift", False).value
        )
        self.commissioning_hold_before_drop = bool(
            self.declare_parameter("commissioning_hold_before_drop", False).value
        )
        self.enable_attach_ready_timeout_landing = bool(
            self.declare_parameter(
                "enable_attach_ready_timeout_landing", True
            ).value
        )
        self.attach_ready_land_after_s = max(
            0.0,
            float(
                self.declare_parameter("attach_ready_land_after_s", 30.0).value
            ),
        )
        self.enable_pickup_timeout_landing = bool(
            self.declare_parameter("enable_pickup_timeout_landing", True).value
        )
        self.pickup_land_after_s = max(
            0.0,
            float(self.declare_parameter("pickup_land_after_s", 20.0).value),
        )
        self.landing_descent_speed = max(
            0.001,
            float(self.declare_parameter("landing_descent_speed", 0.25).value),
        )
        self.landing_disarm_height_above_start = max(
            0.0,
            float(
                self.declare_parameter(
                    "landing_disarm_height_above_start", 0.20
                ).value
            ),
        )
        self.landing_min_time_before_disarm_s = max(
            0.0,
            float(
                self.declare_parameter(
                    "landing_min_time_before_disarm_s", 1.0
                ).value
            ),
        )
        self.landing_disarm_command = str(
            self.declare_parameter("landing_disarm_command", "DISARM").value
        ).strip().upper()

        # ------------------------------------------------------------------
        # Visualisation and logging
        # ------------------------------------------------------------------
        self.show_debug_text = bool(
            self.declare_parameter("show_debug_text", True).value
        )
        self.show_collision_markers = bool(
            self.declare_parameter("show_collision_markers", True).value
        )
        self.enable_csv_logging = bool(
            self.declare_parameter("enable_csv_logging", True).value
        )
        self.log_directory = str(
            self.declare_parameter("log_directory", "logs/join_planner").value
        )
        self.log_run_label = str(
            self.declare_parameter("log_run_label", "").value
        ).strip()
        self.log_every_n_updates = max(
            1, int(self.declare_parameter("log_every_n_updates", 1).value)
        )
        self.update_counter = 0
        self.csv_file = None
        self.csv_writer = None
        self.csv_path = ""

        # ------------------------------------------------------------------
        # Runtime state
        # ------------------------------------------------------------------
        self.drone_position: Optional[np.ndarray] = None
        self.drone_velocity: Optional[np.ndarray] = None
        self.drone_acceleration: Optional[np.ndarray] = None
        self.last_drone_time: Optional[float] = None
        self.initial_quad_z: Optional[float] = None
        self.takeoff_command_time: Optional[float] = None
        self.takeoff_detected_time: Optional[float] = None

        # M2B B0 runtime state. Raw joint truth is intentionally isolated to the
        # pre-arm bootstrap gate and never feeds AttachmentDetector authority.
        self.m2b_joint_detached_truth: Optional[bool] = None
        self.m2b_last_joint_truth_time_s: Optional[float] = None
        self.m2b_controller_armed = False
        self.m2b_initial_yaw: Optional[float] = None
        self.m2c_initial_yaw: Optional[float] = None
        self.m2b_live_ring_position: Optional[np.ndarray] = None
        self.m2b_live_ring_rotation: Optional[np.ndarray] = None
        self.m2b_last_ring_pose_time_s: Optional[float] = None
        self.m2b_bootstrap_arm_permitted = False
        self.m2b_last_bootstrap_reason = "not started"
        self.m2b_last_raw_detach_publish_time_s = float("-inf")
        self.m2b_bootstrap_gate = M2BBootstrapGate(
            self.m2b_config,
            start_time_s=self._physical_now_s(),
            release_required=self.m2b_bootstrap_release_required,
        )
        self.m2b_takeoff_settle_gate = M2BSettleGate(self.m2b_config)
        self.m2b_capture_settle_gate = M2BSettleGate(self.m2b_config)
        self.m2b_capture_settle_position_error_m = float("nan")
        self.m2b_capture_settle_speed_mps = float("nan")
        self.m2b_capture_settle_swing_deg = float("nan")
        self.m2b_capture_settle_dwell_s = 0.0
        self.m2b_capture_settle_reason = "NOT_EVALUATED"
        self.m2b_capture_settle_settled = False
        self.m2b_approach_evidence_gate = M2BEvidenceFreshnessGate(self.m2b_config)
        self.m2b_retry_tracker = M2BRetryTracker(
            assigned_plate_id=self.m2b_assigned_plate_id,
            max_attempts=self.m2b_config.max_attempts,
        )
        self.m2b_descent_progress: Optional[M2BDescentProgress] = None
        self.m2b_attachment_diagnostics: Optional[dict] = None
        self.m2b_last_attachment_diagnostics_time_s: Optional[float] = None
        self.m2b_pendulum_phi = float("nan")
        self.m2b_pendulum_theta = float("nan")
        self.m2b_last_pendulum_time_s: Optional[float] = None
        self.m2b_detector_reset_until_time_s = float("-inf")
        self.m2b_magnet_on_requested = False
        self.m2b_proof_start_position: Optional[np.ndarray] = None
        self.m2b_capture_wait_contact_height_m: Optional[float] = None
        self.m2b_latch_hold_position: Optional[np.ndarray] = None
        self.m2b_latch_stability_gate: Optional[M2BLatchStabilityGate] = None
        self.m2b_latch_stable = False
        self.m2b_last_latch_stability_decision = None
        self.m2b_attached_bad_since_time_s: Optional[float] = None
        self.m2b_retreat_stage = "idle"
        self.m2b_retreat_target: Optional[np.ndarray] = None
        self.m2b_landing_stage_target: Optional[np.ndarray] = None
        self.m2b_landing_detach_hold_position: Optional[np.ndarray] = None
        self.m2b_fault_hold_position: Optional[np.ndarray] = None

        self.attachment_position: Optional[np.ndarray] = None
        self.attachment_velocity: Optional[np.ndarray] = None
        self.last_attachment_pose_time: Optional[float] = None
        self.last_attachment_twist_time: Optional[float] = None

        self.payload_position: Optional[np.ndarray] = None
        self.payload_velocity: Optional[np.ndarray] = None
        self.payload_yaw = 0.0
        self.last_payload_pose_time: Optional[float] = None
        self.last_payload_twist_time: Optional[float] = None

        if self.use_static_pickup_object:
            self.pickup_object_position: Optional[np.ndarray] = (
                self.static_pickup_object.copy()
            )
            self.pickup_object_velocity: Optional[np.ndarray] = np.zeros(3, dtype=float)
            self.pickup_object_yaw = self.static_pickup_object_yaw
            self.last_pickup_object_pose_time: Optional[float] = self._physical_now_s()
        else:
            self.pickup_object_position = None
            self.pickup_object_velocity = None
            self.pickup_object_yaw = 0.0
            self.last_pickup_object_pose_time = None

        self.latched_pickup_position: Optional[np.ndarray] = None
        self.latched_pickup_yaw: Optional[float] = None
        self.last_pickup_target_geometry: Optional[PickupTargetGeometry] = None
        self.last_fixed_clearance_pickup_target: Optional[np.ndarray] = None
        self.last_geometry_pickup_target_delta: Optional[np.ndarray] = None
        self.last_geometry_pickup_approach_target: Optional[np.ndarray] = None
        self.latched_geometry_pickup_contact_target: Optional[np.ndarray] = None
        self.latched_geometry_pickup_approach_target: Optional[np.ndarray] = None
        self.latched_geometry_pickup_contact_offset: Optional[np.ndarray] = None
        self.geometry_pickup_latch_source = "unavailable"
        self.geometry_pickup_latch_error = ""
        self.geometry_aware_pickup_active = False
        self.geometry_aware_pickup_mode = "disabled"
        self.geometry_aware_pickup_fallback_reason = ""
        self.last_active_geometry_pickup_target: Optional[np.ndarray] = None
        self.pickup_geometry_diagnostics_available = False
        self.pickup_geometry_diagnostics_source = "unavailable"
        self.pickup_geometry_pose_age_s: Optional[float] = None
        self.pickup_geometry_diagnostics_error = ""
        self.attached_payload_offset_from_magnet: Optional[np.ndarray] = None
        self.attached_payload_yaw: Optional[float] = None
        self.attached_payload_geometry_source = "unavailable"
        self.object_attached = False
        self.last_magnet_command: Optional[str] = None
        self.attach_wait_start_time: Optional[float] = None
        self.drop_start_time: Optional[float] = None

        self.measured_magnet_tip_position: Optional[np.ndarray] = None
        self.last_magnet_tip_pose_time: Optional[float] = None
        self.m2b_takeoff_magnet_start_z: Optional[float] = None

        self.external_landing_pending = False
        self.landing_target_xy: Optional[np.ndarray] = None
        self.landing_start_z: Optional[float] = None
        self.landing_disarm_z: Optional[float] = None
        self.landing_disarm_sent = False

        # M2A dynamic-obstacle runtime state. The predictor owns the filtered
        # velocity state; the ROS node only caches the latest complete prediction
        # and measurement arrival time for passive visualization.
        self.moving_obstacle_predictor = MovingObstaclePredictor(
            self.moving_obstacle_prediction_config
        )
        self.latest_moving_obstacle_prediction: Optional[
            MovingObstaclePrediction
        ] = None
        self.last_moving_obstacle_state_time: Optional[float] = None

        self.safety_planner = OnlineSafetyPlanner(
            obstacles=self.static_obstacles,
            drone_radius=self.drone_collision_radius,
            safety_margin=self.collision_safety_margin,
            magnet_radius=self.magnet_collision_radius,
            magnet_safety_margin=self.magnet_collision_margin,
            cable_radius=self.cable_collision_radius,
            cable_safety_margin=self.cable_collision_margin,
            payload_profile=self.payload_profile,
            payload_pose_timeout_s=self.payload_pose_timeout_s,
            collision_diagnostics_enabled=self.collision_diagnostics_enabled,
        )
        self.visual_astar_config = VisualAStarConfig(
            enabled=self.visual_astar_enabled,
            resolution=self.visual_astar_resolution,
            max_expansions=self.visual_astar_max_expansions,
            max_planning_time_s=self.visual_astar_max_planning_time_s,
            nearest_free_radius_m=self.visual_astar_nearest_free_radius_m,
            local_margin_xy_m=self.visual_astar_local_margin_xy_m,
            local_margin_z_m=self.visual_astar_local_margin_z_m,
            global_bounds_min=self.visual_astar_bounds_min,
            global_bounds_max=self.visual_astar_bounds_max,
            rejoin_min_lookahead_s=self.visual_astar_rejoin_min_lookahead_s,
            rejoin_preferred_lookahead_s=(
                self.visual_astar_rejoin_preferred_lookahead_s
            ),
            rejoin_min_separation_m=self.visual_astar_rejoin_min_separation_m,
            candidate_speed_mps=self.visual_astar_candidate_speed_mps,
        )
        self.visual_trajectory_config = (
            ConvexTrajectoryConfig(
                enabled=self.visual_trajectory_enabled,

                planning_clearance_margin_m=(
                    self.visual_trajectory_clearance_margin_m
                ),

                max_speed_mps=(
                    self.visual_trajectory_max_speed_mps
                ),

                max_acceleration_mps2=(
                    self.visual_trajectory_max_acceleration_mps2
                ),

                validation_headroom_ratio=(
                    self.visual_trajectory_validation_headroom_ratio
                ),

                nominal_speed_mps=(
                    self.visual_trajectory_baseline_nominal_speed_mps
                ),

                minimum_segment_time_s=(
                    self.visual_trajectory_minimum_segment_time_s
                ),

                maximum_segment_time_s=(
                    self.visual_trajectory_maximum_segment_time_s
                ),

                max_time_scaling_retries=1,

                time_stretch_safety_factor=(
                    self.visual_trajectory_time_stretch_safety_factor
                ),

                solver_time_s=(
                    self.visual_trajectory_direct_bezier_solver_time_s
                ),

                solver_iterations=(
                    self.visual_trajectory_direct_bezier_solver_iterations
                ),

                use_target_acceleration=(
                    self.visual_trajectory_use_target_acceleration
                ),
            )
        )
        self.visual_astar_executor = ThreadPoolExecutor(
            max_workers=1,
            thread_name_prefix=f"{self.vehicle_id}_visual_astar",
        )
        self.visual_astar_future: Optional[Future] = None
        self.visual_astar_last_submit_time: Optional[float] = None
        self.visual_astar_last_submit_wall_time: Optional[float] = None
        self.visual_astar_request_id = 0
        self.visual_astar_completed_count = 0
        self.visual_astar_discarded_count = 0
        self.last_visual_astar_worker_turnaround_ms = float("nan")
        self.visual_trajectory_target_velocity_history = deque(maxlen=15)
        self.visual_trajectory_velocity_history_phase: Optional[str] = None
        self.last_visual_astar_result: VisualAStarResult = idle_visual_astar_result(
            phase_name=self.phase.value
        )
        self.last_nominal_reference: Optional[TrajectoryReference] = None
        self.last_safety_result: Optional[SafetyPlannerResult] = None
        self.last_reference: Optional[TrajectoryReference] = None
        self.last_rate_profile_duration = float("nan")
        self.last_rate_profile_complete = False
        self.last_status_text = ""

        # Completed-callback timing is reported on the next planner update so the
        # measurement includes CSV, RViz, and status publication from the prior tick.
        self.planner_callback_budget_ms = 1000.0 / self.publish_rate_hz
        self.last_planner_callback_time_ms = float("nan")
        self.last_planner_callback_period_ms = float("nan")
        self.planner_deadline_miss_count = 0
        self._last_planner_callback_start: Optional[float] = None

        # C1F.1 shadow state.  The body target is intentionally latched on entry
        # to APPROACH_ABOVE_PICKUP because the C1F.0 backend is fixed-target-only.
        # We continue computing the live Python body target for diagnostics so any
        # pickup-pose / magnet-compensation drift is visible before C1F.2 authority.
        self.c1f1_shadow_commanded_enable = False
        self.c1f1_shadow_latched_body_target: Optional[np.ndarray] = None
        self.c1f1_shadow_live_body_target: Optional[np.ndarray] = None
        self.c1f1_shadow_target_latch_time: Optional[float] = None
        self.c1f1_shadow_target_drift_m = float("nan")
        self.c1f1_shadow_reference_last_receive_time: Optional[float] = None
        self.c1f1_shadow_reference_source_stamp_s: Optional[float] = None
        self.c1f1_shadow_reference_last_period_ms = float("nan")
        self.c1f1_shadow_reference_count = 0
        self.c1f1_shadow_reference_sample_count = 0
        self.c1f1_shadow_reference_terminal = np.full(3, np.nan)
        self.c1f1_shadow_reference: Optional[TrajectoryReference] = None
        self.c1f1_shadow_status_last_receive_time: Optional[float] = None
        self.c1f1_shadow_backend_status = "not_received"
        self.c1f2_cpp_authority_active = False
        # C1F.2b one-time ownership handshake. The mission remains in the
        # completed LIFT_OBJECT hold while C++ prepares its first committed
        # TRANSIT_TO_DROP_POINT trajectory. Python does not generate an
        # authoritative moving transfer during this interval.
        self.c1f2_prepare_active = False
        self.c1f2_prepare_settled = False
        self.c1f2_prepare_lift_end_position: Optional[np.ndarray] = None
        self.c1f2_prepare_hold_position: Optional[np.ndarray] = None
        self.c1f2_prepare_settle_start_time: Optional[float] = None
        self.c1f2_prepare_drift_m = float("nan")
        self.c1f2_prepare_speed_mps = float("nan")
        self.c1f2_handoff_ready = False
        self.c1f2_prepare_start_time: Optional[float] = None
        self.c1f2_prepare_count = 0
        self.c1f2_prepare_wait_s = float("nan")

        # Operator advance is deliberately narrow: it can waive only the final
        # loaded-lift ideal body-Z entry check.  The normal stationary p/v/a
        # settle, C++ preparation, authority ACK and collision certification all
        # remain mandatory afterward.
        self.operator_advance_override_pending = False
        self.operator_override_used = False
        self.operator_override_request_count = 0
        self.operator_override_accept_count = 0
        self.operator_override_last_from_phase = ""
        self.operator_override_last_action = ""
        self.operator_override_last_result = "NOT_REQUESTED"

        self.c1f2_handoff_position_error_m = float("nan")
        self.c1f2_handoff_velocity_error_mps = float("nan")
        self.c1f2_handoff_acceleration_error_mps2 = float("nan")
        self.c1f2_authority_switch_count = 0
        self.c1f2_commanded_authority = False
        self.c1f2_authority_grant_pending = False
        self.c1f2_authority_acknowledged = False
        self.c1f2_authority_grant_count = 0
        self.c1f2_authority_grant_time: Optional[float] = None
        self.c1f2_authority_grant_ros_time_s: Optional[float] = None
        self.c1f2_authority_ack_time: Optional[float] = None
        # Retained for backwards-compatible CSV analysis. C1F.2a no longer
        # falls back directly to Python on a stale C++ reference.
        self.c1f2_stale_fallback_count = 0
        self.c1f2_cached_cpp_reference: Optional[TrajectoryReference] = None
        self.c1f2_cached_cpp_reference_receive_time: Optional[float] = None
        self.c1f2_cached_continuation_active = False
        self.c1f2_cached_continuation_count = 0
        self.c1f2_recovery_reject_count = 0
        self.c1f2_exit_switch_count = 0
        self.c1f2_cached_reference_age_ms = float("nan")
        # Generic C++ -> Python target-frame bridge. C1F.2b initially uses it
        # for TRANSIT_TO_DROP_POINT -> SETTLE_ABOVE_DROP_POINT; the helper is
        # intentionally reusable for APPROACH_ABOVE_TARGET -> MATCH_VELOCITY.
        self.c1f2_exit_bridge_state: Optional[TargetRelativeBridgeState] = None
        self.c1f2_exit_bridge_start_time: Optional[float] = None
        self.c1f2_exit_bridge_active = False
        self.c1f2_exit_bridge_complete = False
        self.c1f2_exit_bridge_initial_position_error_m = float("nan")
        self.c1f2_exit_bridge_initial_velocity_error_mps = float("nan")
        self.c1f2_exit_bridge_initial_acceleration_error_mps2 = float("nan")
        self.c1f2_exit_bridge_pending_state: Optional[TargetRelativeBridgeState] = None
        self.c1f2_exit_bridge_pending_feasibility: Optional[TargetRelativeBridgeFeasibility] = None
        self.c1f2_exit_bridge_feasible = False
        self.c1f2_exit_bridge_feasibility_reason = "NOT_EVALUATED"
        self.c1f2_exit_bridge_peak_velocity_mps = float("nan")
        self.c1f2_exit_bridge_peak_acceleration_mps2 = float("nan")
        self.c1f2_exit_bridge_peak_jerk_mps3 = float("nan")

        # ------------------------------------------------------------------
        # ROS interfaces
        # ------------------------------------------------------------------
        self.create_subscription(
            MotionCaptureState, self.drone_state_topic, self.drone_state_callback, 10
        )
        if self.moving_obstacle_prediction_enabled:
            self.create_subscription(
                MotionCaptureState,
                self.moving_obstacle_state_topic,
                self.moving_obstacle_state_callback,
                10,
            )
        self.create_subscription(
            PoseStamped,
            self.attachment_pose_topic,
            self.attachment_pose_callback,
            10,
        )
        self.create_subscription(
            TwistStamped,
            self.attachment_twist_topic,
            self.attachment_twist_callback,
            10,
        )
        self.create_subscription(
            PoseStamped, self.payload_pose_topic, self.payload_pose_callback, 10
        )
        self.create_subscription(
            TwistStamped, self.payload_twist_topic, self.payload_twist_callback, 10
        )
        if not self.use_static_pickup_object:
            self.create_subscription(
                PoseArray,
                self.pickup_object_pose_topic,
                self.pickup_object_pose_callback,
                10,
            )
        self.create_subscription(
            Bool,
            self.object_attached_topic,
            self.object_attached_callback,
            10,
        )
        if self.use_measured_magnet_tip:
            self.create_subscription(
                PoseStamped,
                self.magnet_tip_pose_topic,
                self.magnet_tip_pose_callback,
                10,
            )
        self.create_subscription(
            String, self.drone_command_topic, self.drone_command_callback, 10
        )
        if self.is_m2b:
            if self.m2b_require_recent_link_statistics:
                self.create_subscription(
                    Empty,
                    self.m2b_link_statistics_topic,
                    self.m2b_link_statistics_callback,
                    10,
                )
            if self.m2d_enabled:
                self.create_subscription(
                    Bool,
                    self.m2d_mission_permission_topic,
                    self.m2d_mission_permission_callback,
                    10,
                )
                self.create_subscription(
                    Int32,
                    self.m2d_assigned_plate_topic,
                    self.m2d_assigned_plate_callback,
                    10,
                )
                if self.m2d_transit_owner_topic:
                    self.create_subscription(
                        String,
                        self.m2d_transit_owner_topic,
                        lambda msg: setattr(self, "m2d_transit_owner", str(msg.data)),
                        10,
                    )
            if self.m2b_ring_pose_source == "pose_array":
                self.create_subscription(
                    PoseArray,
                    self.m2b_ring_pose_topic,
                    self.m2b_ring_pose_callback,
                    10,
                )
            if self.m2b_attachment_truth_source == "joint_truth":
                self.create_subscription(
                    Bool,
                    self.m2b_joint_truth_topic,
                    self.m2b_joint_truth_callback,
                    10,
                )
            if self.m2b_bootstrap_release_required:
                self.create_subscription(
                    Bool,
                    self.m2b_bootstrap_release_topic,
                    self.m2b_bootstrap_release_callback,
                    10,
                )
            self.create_subscription(
                Bool,
                self.m2b_arming_state_topic,
                self.m2b_arming_state_callback,
                10,
            )
            if self.m2b_commissioning_stage in {"b1", "b2"}:
                self.create_subscription(
                    String,
                    self.m2b_attachment_diagnostics_topic,
                    self.m2b_attachment_diagnostics_callback,
                    10,
                )
                self.create_subscription(
                    MotionCaptureState,
                    self.m2b_pendulum_topic,
                    self.m2b_pendulum_callback,
                    10,
                )
        if self.enable_external_landing_trigger:
            self.create_subscription(
                Bool,
                self.external_landing_topic,
                self.external_landing_callback,
                10,
            )
        self.create_subscription(
            Bool,
            self.operator_advance_override_topic,
            self.operator_advance_override_callback,
            10,
        )

        self.c1f1_shadow_target_publisher = None
        self.c1f1_shadow_enable_publisher = None
        self.c1f2_target_velocity_publisher = None
        self.c1f2_authority_publisher = None
        if self.c1f1_shadow_transfer_enabled:
            if self.c1f1_shadow_reference_topic == self.reference_topic:
                raise ValueError(
                    "C1F.1 shadow_reference_topic must not equal the MPC authority topic"
                )
            self.c1f1_shadow_target_publisher = self.create_publisher(
                PoseStamped, self.c1f1_shadow_target_topic, 5
            )
            self.c1f1_shadow_enable_publisher = self.create_publisher(
                Bool, self.c1f1_shadow_enable_topic, 5
            )
            if self.c1f2_cpp_authority_enabled:
                self.c1f2_target_velocity_publisher = self.create_publisher(
                    TwistStamped, self.c1f2_target_velocity_topic, 5
                )
                self.c1f2_authority_publisher = self.create_publisher(
                    Bool, self.c1f2_authority_topic, 5
                )
            self.create_subscription(
                MultiDOFJointTrajectory,
                self.c1f1_shadow_reference_topic,
                self.c1f1_shadow_reference_callback,
                10,
            )
            self.create_subscription(
                String,
                self.c1f1_shadow_status_topic,
                self.c1f1_shadow_status_callback,
                10,
            )
            if self.c1f2_cpp_authority_enabled:
                self.create_subscription(
                    Bool,
                    self.c1f2_authority_ack_topic,
                    self.c1f2_authority_ack_callback,
                    10,
                )

        self.reference_publisher = self.create_publisher(
            MultiDOFJointTrajectory, self.reference_topic, 1
        )
        self.committed_trajectory_publisher = None
        if self.publish_committed_trajectory:
            # Match the C++ shared-trajectory consumer.  Reliable + transient-local
            # makes the latest authoritative future available across startup order
            # without weakening the existing peer subscription contract.
            committed_qos = QoSProfile(depth=1)
            committed_qos.reliability = ReliabilityPolicy.RELIABLE
            committed_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            self.committed_trajectory_publisher = self.create_publisher(
                CommittedTrajectory, self.committed_trajectory_topic, committed_qos
            )
        self.marker_publisher = self.create_publisher(
            MarkerArray, self.marker_topic, 1
        )
        self.state_publisher = self.create_publisher(String, self.state_topic, 5)
        self.phase_publisher = self.create_publisher(
            String, self.phase_topic, mission_phase_qos_profile()
        )
        self.handoff_ready_publisher = self.create_publisher(
            Bool, self.handoff_ready_topic, 5
        )
        self.safety_status_publisher = self.create_publisher(
            String, self.safety_status_topic, 5
        )
        self.drone_command_publisher = self.create_publisher(
            String, self.drone_command_topic, 5
        )
        self.magnet_command_publisher = self.create_publisher(
            String, self.magnet_command_topic, 5
        )
        self.m2b_raw_detach_request_publisher = None
        self.m2b_arm_permission_publisher = None
        self.m2b_mpc_mode_publisher = None
        self.m2b_proof_requested_publisher = None
        self.m2b_detector_reset_publisher = None
        self.m2b_attempt_publisher = None
        self.m2b_latch_stable_publisher = None
        self.m2b_latch_stability_publisher = None
        if self.is_m2b:
            self.m2b_raw_detach_request_publisher = self.create_publisher(
                Bool, self.m2b_raw_detach_request_topic, 10
            )
            self.m2b_arm_permission_publisher = self.create_publisher(
                Bool, self.m2b_arm_permission_topic, 10
            )
            self.m2b_mpc_mode_publisher = self.create_publisher(
                String, self.m2b_mpc_mode_topic, 10
            )
            if self.m2b_commissioning_stage in {"b1", "b2"}:
                self.m2b_proof_requested_publisher = self.create_publisher(
                    Bool, self.m2b_proof_requested_topic, 10
                )
                self.m2b_detector_reset_publisher = self.create_publisher(
                    Bool, self.m2b_detector_reset_topic, 10
                )
                self.m2b_attempt_publisher = self.create_publisher(
                    Int32, self.m2b_attempt_topic, 10
                )
                self.m2b_latch_stable_publisher = self.create_publisher(
                    Bool, self.m2b_latch_stable_topic, 10
                )
                self.m2b_latch_stability_publisher = self.create_publisher(
                    String, self.m2b_latch_stability_topic, 10
                )

        self.initialise_csv_logger()
        self.publish_phase()
        self.timer = self.create_timer(self.dt, self.timer_callback)

        self.get_logger().info(
            "Barebones per-drone planner started: "
            f"vehicle_id={self.vehicle_id}, mission={self.mission.name}, "
            f"initial_phase={self.phase.value}, publish_rate={self.publish_rate_hz:.1f} Hz, "
            "safety_planner=vectorised_whole_body_diagnostics_only, "
            f"static_obstacles={len(self.static_obstacles)}, "
            f"magnet_radius={self.magnet_collision_radius:.3f} m, "
            f"cable_radius={self.cable_collision_radius:.3f} m, "
            f"payload_profile={self.payload_profile.name}, "
            f"payload_size={self.payload_profile.dimensions.tolist()} m."
        )
        if self.c1f1_shadow_transfer_enabled:
            self.get_logger().warn(
                "C1F.1 SHADOW ONLY enabled for APPROACH_ABOVE_PICKUP: "
                f"target={self.c1f1_shadow_target_topic}, "
                f"enable={self.c1f1_shadow_enable_topic}, "
                f"shadow_reference={self.c1f1_shadow_reference_topic}. "
                f"Controller authority remains {self.reference_topic}."
            )
        if self.c1f2_cpp_authority_enabled:
            self.get_logger().warn(
                "C1F.2 SIM authority enabled for TRANSIT_TO_DROP_POINT: "
                "online_join_planner remains the sole /join_planner/reference publisher, "
                "but selects the C++ transfer reference after a stage-0 p/v/a handoff gate; "
                "brief backend dropouts continue the last committed C++ window instead of jumping back to Python."
            )
        if self.moving_obstacle_prediction_enabled:
            self.get_logger().info(
                "M2A moving-obstacle prediction enabled: "
                f"topic={self.moving_obstacle_state_topic}, "
                f"radius={self.moving_obstacle_geometry_radius_m:.3f} m, "
                f"horizon={self.moving_obstacle_prediction_config.horizon_s:.2f} s, "
                f"dt={self.moving_obstacle_prediction_config.time_step_s:.2f} s, "
                "planner influence=none (RViz only)."
            )

    # ----------------------------------------------------------------------
    # Basic callbacks and input validation
    # ----------------------------------------------------------------------
    def drone_state_callback(self, msg: MotionCaptureState) -> None:
        now = self._physical_now_s()

        position = _vec3_from_position(msg.pose.position)

        velocity = _vec3_from_linear(msg.twist.linear)

        if self.is_m2b and self.m2b_initial_yaw is None:
            self.m2b_initial_yaw = _yaw_from_quaternion(msg.pose.orientation)
            self.get_logger().info(
                f"M2B initial yaw latched at {self.m2b_initial_yaw:.4f} rad"
            )
        if self.m2c_ground_reference_enabled and self.m2c_initial_yaw is None:
            self.m2c_initial_yaw = _yaw_from_quaternion(msg.pose.orientation)
            self.get_logger().info(
                f"M2C initial yaw latched at {self.m2c_initial_yaw:.4f} rad"
            )

        if (
            self.drone_velocity is None
            or self.last_drone_time is None
        ):
            filtered_acceleration = np.zeros(
                3,
                dtype=float,
            )

        else:
            sample_dt = max(
                1e-4,
                now - self.last_drone_time,
            )

            raw_acceleration = (velocity - self.drone_velocity) / sample_dt

            # First-order low-pass filter:
            # alpha = 1 - exp(-dt / tau)
            alpha = 1.0 - math.exp(
                -sample_dt
                / self.visual_trajectory_acceleration_filter_tau_s
            )

            previous_acceleration = (
                np.zeros(3, dtype=float)
                if self.drone_acceleration is None
                else self.drone_acceleration
            )

            filtered_acceleration = (
                previous_acceleration
                + alpha
                * (
                    raw_acceleration
                    - previous_acceleration
                )
            )

        self.drone_position = position
        self.drone_velocity = velocity
        self.drone_acceleration = filtered_acceleration
        self.last_drone_time = now

        if self.initial_quad_z is None:
            self.initial_quad_z = float(self.drone_position[2])

        height_above_start = float(self.drone_position[2]) - self.initial_quad_z
        if (
            self.phase == MissionPhase.TAKEOFF
            and height_above_start >= self.takeoff_detection_height
        ):
            self.takeoff_detected_time = self.last_drone_time
            self.transition_to(
                MissionPhase.APPROACH_ABOVE_PICKUP,
                f"takeoff confirmed at {height_above_start:.2f} m above start",
            )

        if self.external_landing_pending and self.phase not in {
            MissionPhase.LANDING,
            MissionPhase.LANDED_DISARMED,
        }:
            self.external_landing_pending = False
            self.transition_to(
                MissionPhase.LANDING,
                "landing request received before first drone-state sample",
            )

    def moving_obstacle_state_callback(self, msg: MotionCaptureState) -> None:
        """Consume one obstacle measurement and cache its passive M2A prediction."""

        now = self._physical_now_s()

        dt_s = None
        if self.last_moving_obstacle_state_time is not None:
            dt_s = max(
                1e-6,
                now - self.last_moving_obstacle_state_time,
            )

        try:
            state = MovingObstacleState(
                obstacle_id=self.moving_obstacle_id,
                position=_vec3_from_position(msg.pose.position),
                velocity=_vec3_from_linear(msg.twist.linear),
                geometry_radius_m=self.moving_obstacle_geometry_radius_m,
            )
            prediction = self.moving_obstacle_predictor.update(
                state,
                dt_s=dt_s,
            )
        except ValueError as exc:
            self.get_logger().warning(
                f"moving-obstacle prediction rejected measurement: {exc}"
            )
            return

        self.latest_moving_obstacle_prediction = prediction
        self.last_moving_obstacle_state_time = now

    def attachment_pose_callback(self, msg: PoseStamped) -> None:
        self.attachment_position = _vec3_from_position(msg.pose.position)
        self.last_attachment_pose_time = self._physical_now_s()

    def attachment_twist_callback(self, msg: TwistStamped) -> None:
        self.attachment_velocity = _vec3_from_linear(msg.twist.linear)
        self.last_attachment_twist_time = self._physical_now_s()

    def payload_pose_callback(self, msg: PoseStamped) -> None:
        self.payload_position = _vec3_from_position(msg.pose.position)
        self.payload_yaw = _yaw_from_quaternion(msg.pose.orientation)
        self.last_payload_pose_time = self._physical_now_s()

    def payload_twist_callback(self, msg: TwistStamped) -> None:
        self.payload_velocity = _vec3_from_linear(msg.twist.linear)
        self.last_payload_twist_time = self._physical_now_s()

    def magnet_tip_pose_callback(self, msg: PoseStamped) -> None:
        self.measured_magnet_tip_position = _vec3_from_position(msg.pose.position)
        self.last_magnet_tip_pose_time = self._physical_now_s()

    def pickup_object_pose_callback(self, msg: PoseArray) -> None:
        if self.use_static_pickup_object:
            return
        index = self.pickup_object_index
        if index < 0:
            index = len(msg.poses) + index
        if index < 0 or index >= len(msg.poses):
            self.get_logger().warning(
                f"pickup_object_index={self.pickup_object_index} is out of range "
                f"for PoseArray length {len(msg.poses)}"
            )
            return

        now = self._physical_now_s()
        pose = msg.poses[index]
        position = _vec3_from_position(pose.position)
        if self.pickup_object_position is None or self.last_pickup_object_pose_time is None:
            velocity = np.zeros(3, dtype=float)
        else:
            sample_dt = max(1e-6, now - self.last_pickup_object_pose_time)
            velocity = (position - self.pickup_object_position) / sample_dt
        self.pickup_object_position = position
        self.pickup_object_velocity = velocity
        self.pickup_object_yaw = _yaw_from_quaternion(pose.orientation)
        self.last_pickup_object_pose_time = now

    def object_attached_callback(self, msg: Bool) -> None:
        attached = bool(msg.data)
        if not attached and self.object_attached:
            self.attached_payload_offset_from_magnet = None
            self.attached_payload_yaw = None
            self.attached_payload_geometry_source = "unavailable"
        self.object_attached = attached

    def attached_payload_geometry_for_safety(
        self,
    ) -> Tuple[bool, Optional[np.ndarray], Optional[float], Optional[float]]:
        """Return the carried-payload transform used by passive V5A checks.

        A configured static pickup object represents a known rigid payload for
        real-world tests, so its magnet-to-pose transform is latched once after
        attachment.  A topic-backed simulated payload is used only while its pose
        is fresh; stale data is reported as unavailable rather than silently safe.
        """

        if not self.object_attached:
            self.attached_payload_geometry_source = "inactive"
            return False, None, None, None
        if self.pickup_object_position is None:
            self.attached_payload_geometry_source = "missing_pickup_pose"
            return False, None, None, None

        now = self._physical_now_s()
        pose_age = (
            float("inf")
            if self.last_pickup_object_pose_time is None
            else max(0.0, now - self.last_pickup_object_pose_time)
        )

        if self.use_static_pickup_object:
            if self.attached_payload_offset_from_magnet is None:
                try:
                    magnet_position = self.current_magnet_tip_position()
                except RuntimeError:
                    self.attached_payload_geometry_source = "missing_magnet_pose"
                    return False, None, None, None
                self.attached_payload_offset_from_magnet = (
                    self.pickup_object_position - magnet_position
                )
                self.attached_payload_yaw = float(self.pickup_object_yaw)
            self.attached_payload_geometry_source = "latched_static_profile"
            return (
                True,
                self.attached_payload_offset_from_magnet.copy(),
                float(self.attached_payload_yaw),
                0.0,
            )

        if pose_age > self.payload_pose_timeout_s:
            self.attached_payload_geometry_source = "stale_pickup_pose"
            return False, None, None, pose_age

        try:
            magnet_position = self.current_magnet_tip_position()
        except RuntimeError:
            self.attached_payload_geometry_source = "missing_magnet_pose"
            return False, None, None, pose_age

        self.attached_payload_offset_from_magnet = (
            self.pickup_object_position - magnet_position
        )
        self.attached_payload_yaw = float(self.pickup_object_yaw)
        self.attached_payload_geometry_source = "fresh_pickup_pose"
        return (
            True,
            self.attached_payload_offset_from_magnet.copy(),
            float(self.attached_payload_yaw),
            pose_age,
        )

    def m2b_joint_truth_callback(self, msg: Bool) -> None:
        self.m2b_joint_detached_truth = bool(msg.data)
        self.m2b_last_joint_truth_time_s = self._physical_now_s()

    def m2b_bootstrap_release_callback(self, msg: Bool) -> None:
        if not bool(msg.data) or not self.m2b_bootstrap_release_required:
            return
        if self.m2b_controller_armed:
            self.get_logger().warning(
                "Ignoring M2B bootstrap release after controller is armed",
                throttle_duration_sec=1.0,
            )
            return
        if self.phase not in {
            MissionPhase.M2_BOOTSTRAP_DETACH,
            MissionPhase.M2_BOOTSTRAP_SETTLE,
        }:
            return
        self.m2b_bootstrap_gate.release(now_s=self._physical_now_s())
        self.m2b_last_bootstrap_reason = "supervised bootstrap release received"
        self.get_logger().info("M2B supervised bootstrap release received; raw detach is now permitted")

    def m2b_link_statistics_callback(self, _msg: Empty) -> None:
        self.m2b_last_link_statistics_s = self._physical_now_s()

    def m2d_mission_permission_callback(self, msg: Bool) -> None:
        previous = bool(self.m2d_mission_permission)
        granted = bool(msg.data)
        self.m2d_mission_permission = granted
        if (
            self.m2d_enabled
            and previous
            and not granted
            and self.phase not in {
                MissionPhase.M2_BOOTSTRAP_DETACH,
                MissionPhase.M2_BOOTSTRAP_SETTLE,
                MissionPhase.M2_WAIT_FOR_ARM,
                MissionPhase.M2_ATTACHED_HOLD,
                MissionPhase.M2_FAULT,
                MissionPhase.LANDED_DISARMED,
            }
        ):
            self.transition_to(
                MissionPhase.M2_FAULT,
                "M2D fleet permission revoked while vehicle mission was active",
            )

    def m2d_assigned_plate_callback(self, msg: Int32) -> None:
        plate = int(msg.data)
        if plate < 0 or plate >= self.m2b_ring_geometry.plate_count:
            return
        if self.m2d_assignment_received and plate != self.m2b_assigned_plate_id:
            self.get_logger().error(
                f"Ignoring M2D reassignment {self.m2b_assigned_plate_id} -> {plate}"
            )
            return
        if self.phase not in {
            MissionPhase.M2_BOOTSTRAP_DETACH,
            MissionPhase.M2_BOOTSTRAP_SETTLE,
            MissionPhase.M2_WAIT_FOR_ARM,
        } and plate != self.m2b_assigned_plate_id:
            self.get_logger().error("Ignoring M2D plate assignment after flight progression")
            return
        self.m2b_assigned_plate_id = plate
        self.m2b_retry_tracker = M2BRetryTracker(
            assigned_plate_id=plate, max_attempts=self.m2b_config.max_attempts
        )
        self.m2d_assignment_received = True

    def m2b_ring_pose_callback(self, msg: PoseArray) -> None:
        if not msg.poses:
            return
        index = self.m2b_ring_pose_index
        if index < 0:
            index += len(msg.poses)
        if index < 0 or index >= len(msg.poses):
            return
        pose = msg.poses[index]
        position = np.array(
            [pose.position.x, pose.position.y, pose.position.z], dtype=float
        )
        quaternion = np.array(
            [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w],
            dtype=float,
        )
        if not np.all(np.isfinite(position)) or not np.all(np.isfinite(quaternion)):
            return
        try:
            rotation = quaternion_xyzw_to_rotation(quaternion)
        except ValueError:
            return
        self.m2b_live_ring_position = position
        self.m2b_live_ring_rotation = rotation
        self.m2b_last_ring_pose_time_s = self._physical_now_s()

    def m2b_arming_state_callback(self, msg: Bool) -> None:
        self.m2b_controller_armed = bool(msg.data)

    def m2b_attachment_diagnostics_callback(self, msg: String) -> None:
        try:
            data = json.loads(str(msg.data))
        except (TypeError, ValueError, json.JSONDecodeError):
            self.get_logger().warning(
                "Ignoring malformed M2B attachment diagnostics",
                throttle_duration_sec=1.0,
            )
            return
        if not isinstance(data, dict):
            return
        try:
            plate = int(data.get("assigned_plate_id", -1))
        except (TypeError, ValueError):
            return
        if plate != self.m2b_assigned_plate_id:
            self.get_logger().warning(
                f"Ignoring attachment diagnostics for plate {plate}; assigned={self.m2b_assigned_plate_id}",
                throttle_duration_sec=1.0,
            )
            return
        self.m2b_attachment_diagnostics = data
        self.m2b_last_attachment_diagnostics_time_s = self._physical_now_s()

    def m2b_pendulum_callback(self, msg: MotionCaptureState) -> None:
        self.m2b_pendulum_phi = float(msg.pose.position.x)
        self.m2b_pendulum_theta = float(msg.pose.position.y)
        self.m2b_last_pendulum_time_s = self._physical_now_s()

    def m2b_attachment_evidence_fresh(self, now_s: Optional[float] = None) -> bool:
        if self.m2b_attachment_diagnostics is None or self.m2b_last_attachment_diagnostics_time_s is None:
            return False
        now = self._physical_now_s() if now_s is None else float(now_s)
        age = now - self.m2b_last_attachment_diagnostics_time_s
        return bool(
            0.0 <= age <= self.m2b_config.evidence_timeout_s
            and self.m2b_attachment_diagnostics.get("fresh", False)
        )

    def m2b_pendulum_fresh(self, now_s: Optional[float] = None) -> bool:
        if self.m2b_last_pendulum_time_s is None:
            return False
        now = self._physical_now_s() if now_s is None else float(now_s)
        age = now - self.m2b_last_pendulum_time_s
        return bool(0.0 <= age <= self.m2b_config.evidence_timeout_s)

    def m2b_ring_pose_world(self) -> Tuple[np.ndarray, np.ndarray]:
        if self.m2b_ring_pose_source == "pose_array":
            if self.m2b_live_ring_position is None or self.m2b_live_ring_rotation is None:
                raise RuntimeError("live M2B ring pose not available")
            return self.m2b_live_ring_position.copy(), self.m2b_live_ring_rotation.copy()
        c = math.cos(self.m2b_fixed_ring_yaw)
        s = math.sin(self.m2b_fixed_ring_yaw)
        rotation = np.array(
            [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=float
        )
        return self.m2b_fixed_ring_position.copy(), rotation

    def m2b_ring_rotation_world(self) -> np.ndarray:
        return self.m2b_ring_pose_world()[1]

    def m2b_plate_pose_world(self) -> Tuple[np.ndarray, np.ndarray]:
        ring_position, rotation = self.m2b_ring_pose_world()
        position = self.m2b_ring_geometry.plate_position_world(
            ring_position=ring_position,
            rotation_world_from_ring=rotation,
            plate_index=self.m2b_assigned_plate_id,
        )
        plate_rotation = self.m2b_ring_geometry.plate_rotation_world(
            rotation_world_from_ring=rotation,
            plate_index=self.m2b_assigned_plate_id,
        )
        return position, plate_rotation

    def m2b_body_rotation_world(self) -> np.ndarray:
        yaw = 0.0 if self.m2b_initial_yaw is None else float(self.m2b_initial_yaw)
        c = math.cos(yaw)
        s = math.sin(yaw)
        return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=float)

    def m2b_body_target_for_contact_height(self, contact_height_m: float) -> np.ndarray:
        plate, _ = self.m2b_plate_pose_world()
        contact = contact_capture_position(plate, capture_height_m=max(0.0, float(contact_height_m)))
        return body_reference_from_contact(
            contact_position_world=contact,
            direction_contact_to_vehicle_world=np.array([0.0, 0.0, 1.0]),
            tether_length_m=self.m2b_anchor_to_contact_length_m,
            rotation_world_from_body=self.m2b_body_rotation_world(),
            tether_anchor_body=self.m2b_tether_anchor_body,
        )

    def m2b_capture_body_target(self) -> np.ndarray:
        return self.m2b_body_target_for_contact_height(self.m2b_config.capture_height_m)

    def m2b_attached_hold_body_target(self) -> np.ndarray:
        plate, plate_rotation = self.m2b_plate_pose_world()
        direction_world = plate_rotation @ attached_hold_direction_plate(self.m2b_config)
        return body_reference_from_contact(
            contact_position_world=plate,
            direction_contact_to_vehicle_world=direction_world,
            tether_length_m=self.m2b_anchor_to_contact_length_m,
            rotation_world_from_body=self.m2b_body_rotation_world(),
            tether_anchor_body=self.m2b_tether_anchor_body,
        )

    def m2b_measured_settled(self, gate: M2BSettleGate, target: np.ndarray, now: float) -> bool:
        if self.drone_position is None or self.drone_velocity is None:
            gate.reset()
            return False
        pendulum_fresh = self.m2b_pendulum_fresh(now)
        swing = float(math.hypot(self.m2b_pendulum_phi, self.m2b_pendulum_theta))
        if not math.isfinite(swing):
            pendulum_fresh = False
            swing = float("inf")
        position_error_m = float(np.linalg.norm(self.drone_position - target))
        speed_mps = float(np.linalg.norm(self.drone_velocity))
        decision = gate.step(
            now_s=now,
            inputs_fresh=pendulum_fresh,
            position_error_m=position_error_m,
            speed_mps=speed_mps,
            swing_magnitude_rad=swing,
        )
        if gate is self.m2b_capture_settle_gate:
            self.m2b_capture_settle_position_error_m = position_error_m
            self.m2b_capture_settle_speed_mps = speed_mps
            self.m2b_capture_settle_swing_deg = math.degrees(swing)
            self.m2b_capture_settle_dwell_s = float(decision.dwell_s)
            self.m2b_capture_settle_reason = str(decision.reason)
            self.m2b_capture_settle_settled = bool(decision.settled)
        return bool(decision.settled)

    def m2b_takeoff_hover_settled(
        self,
        target: np.ndarray,
        now: float,
    ) -> bool:
        elapsed_s = max(0.0, now - self.phase_entry_time)
        current_tip_z = None
        if self.measured_magnet_tip_position is not None:
            current_tip_z = float(self.measured_magnet_tip_position[2])
        tip_fresh = bool(
            self.measured_magnet_tip_position is not None
            and self.last_magnet_tip_pose_time is not None
            and now - self.last_magnet_tip_pose_time <= self.input_timeout_s
        )
        clearance = evaluate_takeoff_magnet_clearance(
            enabled=self.m2b_takeoff_require_magnet_clearance,
            start_tip_z_m=self.m2b_takeoff_magnet_start_z,
            current_tip_z_m=current_tip_z,
            inputs_fresh=tip_fresh,
            minimum_rise_m=self.m2b_takeoff_magnet_clearance_m,
            hover_elapsed_s=elapsed_s,
            timeout_s=self.m2b_takeoff_hover_timeout_s,
        )
        if not clearance.ready:
            self.m2b_takeoff_settle_gate.reset()
            self.get_logger().warning(
                f"Post-takeoff settle blocked: {clearance.reason}",
                throttle_duration_sec=1.0,
            )
            if clearance.timed_out:
                self.transition_to(
                    MissionPhase.M2_FAULT,
                    f"post-takeoff hover timeout: {clearance.reason}",
                )
            return False

        settled = self.m2b_measured_settled(self.m2b_takeoff_settle_gate, target, now)
        if settled:
            return True
        if (
            self.m2b_takeoff_hover_timeout_s > 0.0
            and elapsed_s >= self.m2b_takeoff_hover_timeout_s
        ):
            self.transition_to(
                MissionPhase.M2_FAULT,
                "post-takeoff hover timeout: measured vehicle/swing settle incomplete",
            )
        return False

    def m2b_request_detector_reset(self) -> None:
        self.m2b_detector_reset_until_time_s = self._physical_now_s() + 0.15
        self.m2b_attachment_diagnostics = None
        self.m2b_last_attachment_diagnostics_time_s = None

    def m2b_physical_latched(self, now: Optional[float] = None) -> bool:
        if self.m2b_attachment_truth_source == "joint_truth":
            return bool(
                self.m2b_joint_truth_fresh(now)
                and self.m2b_joint_detached_truth is False
            )
        if self.m2b_attachment_truth_source == "measured_detector":
            if not self.m2b_attachment_evidence_fresh(now):
                return False
            diagnostics = self.m2b_attachment_diagnostics or {}
            return bool(
                not diagnostics.get("lost", False)
                and (
                    diagnostics.get("candidate_ready", False)
                    or diagnostics.get("confirmed", False)
                )
            )
        return False

    def m2b_geometry_separated(
        self, now: Optional[float] = None, *, require_lost: bool = False
    ) -> bool:
        if self.m2b_attachment_truth_source == "joint_truth":
            return bool(
                self.m2b_joint_truth_fresh(now)
                and self.m2b_joint_detached_truth is True
            )
        if not self.m2b_attachment_evidence_fresh(now):
            return False
        diagnostics = self.m2b_attachment_diagnostics or {}
        return bool(
            diagnostics.get("geometry_separated", False)
            and (not require_lost or diagnostics.get("lost", False))
        )

    def m2b_fail_attempt(self, reason: str) -> None:
        self.m2b_magnet_on_requested = False
        self.m2b_latch_hold_position = None
        self.m2b_latch_stability_gate = None
        self.m2b_latch_stable = False
        self.m2b_last_latch_stability_decision = None
        self.m2b_request_detector_reset()
        outcome = self.m2b_retry_tracker.record_failure()
        if outcome.terminal_failure:
            self.transition_to(
                MissionPhase.M2_LANDING_STAGE,
                f"M2B attempt {outcome.attempt_number} failed: {reason}; maximum attempts reached",
            )
        else:
            self.transition_to(
                MissionPhase.M2_DETACHED_RETREAT,
                f"M2B attempt failed: {reason}; retrying same plate {outcome.assigned_plate_id} as attempt {outcome.attempt_number}",
            )

    def m2b_b2_begin_cpp_prepare(self) -> None:
        """Latch a stationary post-takeoff hold for the existing C1F.2 authority handshake."""
        if self.drone_position is None or self.drone_velocity is None:
            return
        self.c1f2_prepare_active = True
        self.c1f2_prepare_settled = False
        self.c1f2_prepare_lift_end_position = self.drone_position.copy()
        self.c1f2_prepare_hold_position = self.drone_position.copy()
        self.c1f2_prepare_settle_start_time = None
        self.c1f2_prepare_drift_m = 0.0
        self.c1f2_prepare_speed_mps = float(np.linalg.norm(self.drone_velocity))
        self.c1f2_handoff_ready = False
        self.c1f2_authority_grant_pending = False
        self.c1f2_authority_acknowledged = False
        self.c1f2_authority_grant_time = None
        self.c1f2_authority_grant_ros_time_s = None
        self.c1f2_authority_ack_time = None
        self.c1f2_publish_authority(False)
        self.c1f2_prepare_start_time = self._physical_now_s()
        self.c1f2_prepare_count += 1
        self.get_logger().info(
            "M2B B2 post-takeoff hover settled; latched measured hold for C++ transit preparation."
        )

    def update_m2b_b2(self) -> None:
        """Commission C++ transit to the exact B1 capture hover, then hand back to B1."""
        if not self.is_m2b or self.m2b_commissioning_stage != "b2":
            return
        if self.drone_position is None or self.drone_velocity is None:
            return
        now = self._physical_now_s()

        if self.phase == MissionPhase.M2_TAKEOFF_HOVER:
            if self.takeoff_start_position is None:
                return
            hover = self.takeoff_start_position.copy()
            hover[2] += self.takeoff_height
            if not self.c1f2_prepare_active:
                if self.m2b_takeoff_hover_settled(hover, now):
                    if (self.m2d_transit_owner_topic
                            and self.m2d_transit_owner != self.vehicle_id):
                        if not self._m2d_transit_wait_logged:
                            self.get_logger().info(
                                "M2D transit token held by "
                                f"{self.m2d_transit_owner or 'nobody'}; holding the hover")
                            self._m2d_transit_wait_logged = True
                        return
                    self.m2b_b2_begin_cpp_prepare()
                return
            if self.c1f2_prepare_start_time is not None:
                self.c1f2_prepare_wait_s = max(
                    0.0, self._physical_now_s() - self.c1f2_prepare_start_time
                )
            if not self.c1f2_prepare_settled:
                if (
                    self.drone_acceleration is None
                    or self.c1f2_prepare_lift_end_position is None
                ):
                    self.c1f2_prepare_settle_start_time = None
                    return
                grant_state = stationary_grant_state(
                    lift_end_position=self.c1f2_prepare_lift_end_position,
                    measured_position=self.drone_position,
                    measured_velocity=self.drone_velocity,
                    measured_acceleration=self.drone_acceleration,
                    max_drift_m=self.c1f2_prepare_max_lift_drift_m,
                    settle_speed_mps=self.c1f2_prepare_settle_speed_mps,
                    settle_acceleration_mps2=self.c1f2_handoff_acceleration_tolerance_mps2,
                )
                self.c1f2_prepare_drift_m = grant_state.drift_m
                self.c1f2_prepare_speed_mps = grant_state.speed_mps
                if not grant_state.ready:
                    self.c1f2_prepare_settle_start_time = None
                    return
                if self.c1f2_prepare_settle_start_time is None:
                    self.c1f2_prepare_settle_start_time = now
                    if self.c1f2_prepare_settle_dwell_s > 0.0:
                        return
                if now - self.c1f2_prepare_settle_start_time < self.c1f2_prepare_settle_dwell_s:
                    return
                self.c1f2_prepare_hold_position = self.drone_position.copy()
                self.c1f2_prepare_settled = True
                self.get_logger().info(
                    "M2B B2 stationary handoff state settled; enabling existing C++ transfer preparation."
                )
                return
            if self.c1f2_handoff_ready:
                self.transition_to(
                    MissionPhase.M2_CPP_TRANSIT_CAPTURE,
                    "M2B B2 C++ transfer prepared from stationary post-takeoff hold",
                )
            return

        if self.phase == MissionPhase.M2_CPP_TRANSIT_CAPTURE:
            capture = self.m2b_capture_body_target()
            xy_error = float(np.linalg.norm(self.drone_position[:2] - capture[:2]))
            z_error = float(abs(self.drone_position[2] - capture[2]))
            spatial_capture = bool(
                xy_error < self.c1f2_nav_capture_xy_threshold_m
                and z_error < self.c1f2_nav_capture_z_threshold_m
            )
            bridge_feasible = False
            if spatial_capture:
                _, feasibility = self._evaluate_c1f2_target_relative_exit_bridge()
                bridge_feasible = bool(feasibility is not None and feasibility.feasible)
            else:
                self.c1f2_exit_bridge_feasible = False
                self.c1f2_exit_bridge_feasibility_reason = "OUTSIDE_NAV_CAPTURE_REGION"
                self.c1f2_exit_bridge_pending_state = None
                self.c1f2_exit_bridge_pending_feasibility = None
            if self.condition_true_for(spatial_capture and bridge_feasible, 0.50):
                self.transition_to(
                    MissionPhase.M2_SETTLE_CAPTURE,
                    "B2 C++ transit entered the dynamically-capturable B1 capture-hover bridge region",
                )
            return

    def update_m2b_b1(self) -> None:
        """Advance only B1 local attachment states using measured evidence.

        Raw Gazebo joint truth is used only to verify physical latch/release. It is
        never treated as software attachment confirmation. Detector confirmation
        comes exclusively from m2_attachment_observer diagnostics.
        """
        if not self.is_m2b or self.m2b_commissioning_stage not in {"b1", "b2"}:
            return
        if self.drone_position is None or self.drone_velocity is None:
            return
        now = self._physical_now_s()

        if self.phase == MissionPhase.M2_TAKEOFF_HOVER:
            if self.takeoff_start_position is None:
                return
            hover = self.takeoff_start_position.copy()
            hover[2] += self.takeoff_height
            if self.m2b_takeoff_hover_settled(hover, now):
                self.transition_to(
                    MissionPhase.M2_SETTLE_CAPTURE,
                    "measured post-takeoff speed/swing dwell complete; begin local capture transfer",
                )
            return

        if self.phase == MissionPhase.M2_SETTLE_CAPTURE:
            if self.m2b_commissioning_stage == "b2" and self.c1f2_exit_bridge_active:
                return
            capture = self.m2b_capture_body_target()
            if self.m2b_measured_settled(self.m2b_capture_settle_gate, capture, now):
                if not self.m2b_b1_attachment_enabled:
                    return
                self.m2b_request_detector_reset()
                self.m2b_descent_progress = M2BDescentProgress(
                    self.m2b_config,
                    initial_contact_height_m=self.m2b_config.capture_height_m,
                )
                self.m2b_capture_wait_contact_height_m = None
                self.m2b_approach_evidence_gate.reset()
                self.m2b_magnet_on_requested = False
                self.transition_to(
                    MissionPhase.M2_ATTACH_APPROACH_HIGH,
                    "capture hover reached and measured settle dwell complete",
                )
            return

        if self.phase in {MissionPhase.M2_ATTACH_APPROACH_HIGH, MissionPhase.M2_ATTACH_APPROACH_LOW}:
            if self.m2b_descent_progress is None:
                self.m2b_descent_progress = M2BDescentProgress(
                    self.m2b_config,
                    initial_contact_height_m=self.m2b_config.capture_height_m,
                )
            evidence_fresh = self.m2b_attachment_evidence_fresh(now)
            evidence_decision = self.m2b_approach_evidence_gate.step(
                now_s=now, fresh=evidence_fresh
            )
            if not evidence_fresh:
                # Never leave the magnet armed from an earlier LOW-band sample
                # after attachment evidence becomes stale.  Hold the existing
                # contact-height target and tolerate only a bounded dropout.
                self.m2b_magnet_on_requested = False
                if evidence_decision.failed:
                    self.m2b_fail_attempt("attachment evidence remained stale during approach")
                return
            diag = self.m2b_attachment_diagnostics or {}
            try:
                xy = float(diag.get("xy_error_m", float("nan")))
            except (TypeError, ValueError):
                return
            decision = self.m2b_descent_progress.step(
                now_s=now,
                measured_xy_error_m=xy,
                dt_s=self.dt,
            )
            if decision.failed:
                self.m2b_fail_attempt(decision.reason)
                return
            if (
                self.phase == MissionPhase.M2_ATTACH_APPROACH_HIGH
                and decision.band == "LOW"
            ):
                self.transition_to(
                    MissionPhase.M2_ATTACH_APPROACH_LOW,
                    "contact-height command entered LOW precision band",
                )
                return
            if self.phase == MissionPhase.M2_ATTACH_APPROACH_LOW:
                try:
                    normal = float(diag.get("normal_error_m", float("nan")))
                    rel_speed = float(diag.get("relative_speed_mps", float("nan")))
                except (TypeError, ValueError):
                    return
                self.m2b_magnet_on_requested = should_enable_magnet(
                    self.m2b_config,
                    contact_height_m=normal,
                    xy_error_m=xy,
                    detector_fresh=True,
                    relative_speed_mps=rel_speed,
                )
                if should_enter_capture_wait(
                    self.m2b_config,
                    contact_height_m=normal,
                    magnet_enabled=self.m2b_magnet_on_requested,
                ):
                    # Hold the current commanded height instead of forcing the
                    # reference to zero.  This preserves reference continuity and
                    # lets physical latch dwell close on measured geometry even
                    # when the vehicle has a small steady vertical tracking bias.
                    self.m2b_capture_wait_contact_height_m = decision.target_contact_height_m
                    self.transition_to(
                        MissionPhase.M2_ATTACH_CAPTURE_WAIT,
                        "measured contact entered physical-capture band; freeze descent for latch dwell",
                    )
            return

        if self.phase == MissionPhase.M2_ATTACH_CAPTURE_WAIT:
            self.m2b_magnet_on_requested = True
            diag = self.m2b_attachment_diagnostics or {}
            evidence_fresh = self.m2b_attachment_evidence_fresh(now)
            evidence_decision = self.m2b_approach_evidence_gate.step(
                now_s=now, fresh=evidence_fresh
            )
            if evidence_decision.failed:
                self.m2b_fail_attempt("attachment evidence remained stale during physical latch")
                return
            if bool(diag.get("lost", False)):
                self.m2b_fail_attempt("software detector lost attachment during physical latch")
                return

            physical_latched = self.m2b_physical_latched(now)
            if physical_latched and self.m2b_latch_stability_gate is None:
                # Freeze exactly the reference that was active when raw Gazebo
                # truth first reported the weld.  Do not start proof motion on
                # the same cycle as the rigid constraint is created.
                if self.last_reference is not None:
                    self.m2b_latch_hold_position = self.last_reference.positions[0].copy()
                else:
                    self.m2b_latch_hold_position = self.drone_position.copy()
                self.m2b_latch_stability_gate = M2BLatchStabilityGate(
                    self.m2b_config,
                    start_time_s=now,
                    start_position_world=self.drone_position,
                    start_velocity_world=self.drone_velocity,
                )
                self.m2b_latch_stable = False
                self.m2b_last_latch_stability_decision = None

            if self.m2b_latch_stability_gate is not None:
                if not physical_latched:
                    self.m2b_fail_attempt("physical joint separated during post-latch settle")
                    return
                decision = self.m2b_latch_stability_gate.step(
                    now,
                    position_world=self.drone_position,
                    velocity_world=self.drone_velocity,
                )
                self.m2b_last_latch_stability_decision = decision
                self.m2b_latch_stable = bool(decision.ready_for_proof)
                if decision.status.value == "FAILED":
                    self.m2b_fail_attempt(decision.reason)
                    return
                if (
                    decision.ready_for_proof
                    and evidence_fresh
                    and bool(diag.get("candidate_ready", False))
                    and self.m2b_b1_proof_enabled
                ):
                    self.transition_to(
                        MissionPhase.M2_ATTACH_PROOF,
                        "physical latch, software candidate, and measured post-latch stability verified",
                    )
                    return
                # In latch-only commissioning, remain here indefinitely after
                # measured stability is established.  Proof_requested stays false.
                return

            if self.phase_elapsed() >= self.m2b_config.physical_latch_timeout_s:
                self.m2b_fail_attempt("physical latch/candidate not verified before timeout")
            return

        if self.phase == MissionPhase.M2_ATTACH_PROOF:
            self.m2b_magnet_on_requested = True
            diag = self.m2b_attachment_diagnostics or {}
            if (
                self.m2b_attachment_truth_source == "joint_truth"
                and self.m2b_joint_truth_fresh(now)
                and self.m2b_joint_detached_truth is True
            ):
                self.m2b_fail_attempt("physical joint separated during proof")
                return
            if self.m2b_attachment_evidence_fresh(now):
                try:
                    excitation = float(diag.get("proof_excitation_m", 0.0))
                except (TypeError, ValueError):
                    excitation = 0.0
                if proof_success(
                    self.m2b_config,
                    detector_confirmed=bool(diag.get("confirmed", False)),
                    detector_fresh=True,
                    physical_latched=self.m2b_physical_latched(now),
                    measured_excitation_m=excitation,
                ):
                    self.transition_to(
                        MissionPhase.M2_ATTACHED_HOLD,
                        "software attachment detector confirmed measured proof excitation",
                    )
                    return
                if bool(diag.get("lost", False)):
                    self.m2b_fail_attempt("software detector lost attachment during proof")
                    return
            if self.phase_elapsed() >= self.m2b_config.proof_timeout_s:
                self.m2b_fail_attempt("proof did not confirm before timeout")
            return

        if self.phase == MissionPhase.M2_ATTACHED_HOLD:
            self.m2b_magnet_on_requested = True
            diag = self.m2b_attachment_diagnostics or {}
            healthy = bool(
                self.m2b_attachment_evidence_fresh(now)
                and self.m2b_physical_latched(now)
                and diag.get("confirmed", False)
                and not diag.get("lost", False)
            )
            if healthy:
                self.m2b_attached_bad_since_time_s = None
            else:
                if self.m2b_attached_bad_since_time_s is None:
                    self.m2b_attached_bad_since_time_s = now
                elif now - self.m2b_attached_bad_since_time_s >= self.m2b_config.attached_loss_grace_s:
                    if self.m2d_enabled:
                        # M2D treats a software-proven attachment as a fleet-level
                        # invariant.  Once ATTACHED_HOLD has been reached, do not
                        # let the legacy local same-plate retry path race the fleet
                        # supervisor after a sustained loss.  The supervisor sees
                        # this terminal phase and latches M2D_ABORT; the currently
                        # active vehicle separately loses permission and enters its
                        # existing controlled M2_FAULT hold.
                        self.transition_to(
                            MissionPhase.M2_FAULT,
                            "M2D proven attachment lost during attached hold",
                        )
                    else:
                        self.m2b_fail_attempt(
                            "sustained detector/physical attachment loss in hold"
                        )
            return

        if self.phase == MissionPhase.M2_DETACHED_RETREAT:
            self.m2b_magnet_on_requested = False
            if self.m2b_retreat_stage == "wait_detach":
                if self.m2b_geometry_separated(now):
                    self.m2b_retreat_stage = "rise"
                    self.m2b_retreat_target = self.drone_position.copy()
                    self.m2b_retreat_target[2] += self.m2b_config.retreat_rise_m
                    self.transfer_generator.reset()
                elif self.phase_elapsed() >= self.m2b_config.detach_verify_timeout_s:
                    self.transition_to(MissionPhase.M2_FAULT, "retreat refused: physical detach not verified")
                return
            if self.m2b_retreat_stage == "rise" and self.m2b_retreat_target is not None:
                reached = bool(
                    np.linalg.norm(self.drone_position - self.m2b_retreat_target) <= self.m2b_config.settle_position_tolerance_m
                    and np.linalg.norm(self.drone_velocity) <= 0.05
                )
                if self.condition_true_for(reached, 0.25):
                    self.transition_to(
                        MissionPhase.M2_SETTLE_CAPTURE,
                        "clear rise complete; return to same capture hover for retry",
                    )
                return

        if self.phase == MissionPhase.M2_LANDING_STAGE:
            self.m2b_magnet_on_requested = False
            if self.m2b_landing_stage_target is None:
                return
            if not self.m2b_geometry_separated(now, require_lost=True):
                if self.phase_elapsed() >= self.m2b_config.detach_verify_timeout_s:
                    self.transition_to(MissionPhase.M2_FAULT, "landing refused: physical detach not verified")
                return
            reached = bool(
                np.linalg.norm(self.drone_position - self.m2b_landing_stage_target) <= self.m2b_config.settle_position_tolerance_m
                and np.linalg.norm(self.drone_velocity) <= 0.05
            )
            if self.condition_true_for(reached, 0.25):
                self.transition_to(MissionPhase.LANDING, "terminal retry failure clear retreat complete")
            return

    def m2b_joint_truth_fresh(self, now_s: Optional[float] = None) -> bool:
        if self.m2b_last_joint_truth_time_s is None:
            return False
        now = self._physical_now_s() if now_s is None else float(now_s)
        age = now - self.m2b_last_joint_truth_time_s
        return bool(0.0 <= age <= self.m2b_joint_truth_timeout_s)

    def m2b_arm_permitted(self) -> bool:
        if not self.is_m2b:
            return False
        if self.m2b_require_recent_link_statistics and not link_statistics_fresh(
            self.m2b_last_link_statistics_s,
            now_s=self._physical_now_s(),
            timeout_s=self.m2b_link_statistics_timeout_s,
        ):
            return False
        if self.m2d_enabled and (
            not self.m2d_assignment_received or not self.m2d_mission_permission
        ):
            return False
        return self.phase not in {
            MissionPhase.M2_BOOTSTRAP_DETACH,
            MissionPhase.M2_BOOTSTRAP_SETTLE,
            MissionPhase.M2_FAULT,
            MissionPhase.LANDED_DISARMED,
        } and self.m2b_bootstrap_arm_permitted

    def update_m2b_bootstrap(self) -> None:
        if not self.is_m2b or self.phase not in {
            MissionPhase.M2_BOOTSTRAP_DETACH,
            MissionPhase.M2_BOOTSTRAP_SETTLE,
        }:
            return
        # Hardware has no Gazebo joint to detach. The measured detector remains
        # the authority after takeoff; this only enters the existing disarmed
        # manual-ARM gate without manufacturing simulated joint evidence.
        if self.m2b_attachment_truth_source == "measured_detector":
            self.m2b_bootstrap_arm_permitted = True
            self.transition_to(
                MissionPhase.M2_WAIT_FOR_ARM,
                "IRL measured-detector bootstrap: no Gazebo joint truth",
            )
            return
        now = self._physical_now_s()
        decision = self.m2b_bootstrap_gate.step(
            now,
            joint_truth_fresh=self.m2b_joint_truth_fresh(now),
            joint_detached=bool(self.m2b_joint_detached_truth),
        )
        self.m2b_bootstrap_arm_permitted = bool(decision.arm_permitted)
        self.m2b_last_bootstrap_reason = decision.reason

        if decision.request_raw_detach and self.m2b_raw_detach_request_publisher is not None:
            # Keep detach requests bounded even when the simulator-side backend is
            # asynchronous. Raw joint truth, not the command attempt, closes the gate.
            if now - self.m2b_last_raw_detach_publish_time_s >= 0.25:
                request = Bool()
                request.data = True
                self.m2b_raw_detach_request_publisher.publish(request)
                self.m2b_last_raw_detach_publish_time_s = now

        if decision.status == BootstrapStatus.FAULT:
            self.transition_to(MissionPhase.M2_FAULT, decision.reason)
        elif decision.status == BootstrapStatus.SETTLING:
            self.transition_to(MissionPhase.M2_BOOTSTRAP_SETTLE, decision.reason)
        elif decision.status == BootstrapStatus.DETACHING:
            self.transition_to(MissionPhase.M2_BOOTSTRAP_DETACH, decision.reason)
        elif decision.status == BootstrapStatus.ARMABLE:
            self.m2b_bootstrap_arm_permitted = True
            self.transition_to(MissionPhase.M2_WAIT_FOR_ARM, decision.reason)

    def publish_m2b_control_contracts(self) -> None:
        if not self.is_m2b:
            return
        if self.m2b_arm_permission_publisher is not None:
            permission = Bool()
            permission.data = self.m2b_arm_permitted()
            self.m2b_arm_permission_publisher.publish(permission)
        if self.m2b_mpc_mode_publisher is not None:
            mode = String()
            mode_name = m2b_mpc_mode_for_phase(self.phase)
            if (
                self.phase == MissionPhase.M2_ATTACH_CAPTURE_WAIT
                and self.m2b_latch_stability_gate is not None
            ):
                # Once the rigid simulated weld exists, disable pendulum-angle
                # cost exactly as proof/attached-hold do, while keeping the
                # position reference frozen during the latch stability dwell.
                mode_name = "ATTACH_PROOF"
            mode.data = mode_name
            self.m2b_mpc_mode_publisher.publish(mode)
        if self.m2b_proof_requested_publisher is not None:
            proof = Bool()
            proof.data = self.phase == MissionPhase.M2_ATTACH_PROOF
            self.m2b_proof_requested_publisher.publish(proof)
        if self.m2b_detector_reset_publisher is not None:
            reset = Bool()
            reset.data = self._physical_now_s() <= self.m2b_detector_reset_until_time_s
            self.m2b_detector_reset_publisher.publish(reset)
        if self.m2b_attempt_publisher is not None:
            attempt = Int32()
            attempt.data = int(self.m2b_retry_tracker.attempt_number)
            self.m2b_attempt_publisher.publish(attempt)
        if self.m2b_latch_stable_publisher is not None:
            stable = Bool()
            stable.data = bool(self.m2b_latch_stable)
            self.m2b_latch_stable_publisher.publish(stable)
        if self.m2b_latch_stability_publisher is not None:
            status = String()
            decision = self.m2b_last_latch_stability_decision
            if decision is None:
                payload = {
                    "active": self.m2b_latch_stability_gate is not None,
                    "ready_for_proof": bool(self.m2b_latch_stable),
                }
            else:
                payload = {
                    "active": True,
                    "status": decision.status.value,
                    "ready_for_proof": bool(decision.ready_for_proof),
                    "speed_mps": float(decision.speed_mps),
                    "acceleration_mps2": float(decision.acceleration_mps2),
                    "displacement_m": float(decision.displacement_m),
                    "stable_dwell_s": float(decision.stable_dwell_s),
                    "reason": str(decision.reason),
                }
            status.data = json.dumps(payload, separators=(",", ":"))
            self.m2b_latch_stability_publisher.publish(status)

    def drone_command_callback(self, msg: String) -> None:
        command = str(msg.data).strip().upper()
        if command == "TAKEOFF":
            takeoff_now = self._physical_now_s()
            self.takeoff_command_time = takeoff_now

            if self.is_m2b:
                if (
                    self.phase == MissionPhase.M2_WAIT_FOR_ARM
                    and self.m2b_arm_permitted()
                    and self.m2b_controller_armed
                    and self.drone_position is not None
                ):
                    magnet_tip_fresh = bool(
                        self.measured_magnet_tip_position is not None
                        and self.last_magnet_tip_pose_time is not None
                        and math.isfinite(
                            float(self.measured_magnet_tip_position[2])
                        )
                        and takeoff_now - self.last_magnet_tip_pose_time
                        <= self.input_timeout_s
                    )
                    if self.m2b_takeoff_require_magnet_clearance and not magnet_tip_fresh:
                        self.get_logger().warning(
                            "M2B TAKEOFF ignored until fresh magnet-tip ground datum is available."
                        )
                    else:
                        if self.m2b_takeoff_require_magnet_clearance:
                            assert self.measured_magnet_tip_position is not None
                            self.m2b_takeoff_magnet_start_z = float(
                                self.measured_magnet_tip_position[2]
                            )
                        else:
                            self.m2b_takeoff_magnet_start_z = None
                        self.takeoff_start_position = self.drone_position.copy()
                        self.transition_to(
                            MissionPhase.M2_VERTICAL_TAKEOFF,
                            "M2B takeoff command received after verified bootstrap and arm",
                        )
                else:
                    self.get_logger().warning(
                        "M2B TAKEOFF ignored until bootstrap permission, controller arm "
                        "feedback, and drone pose are all valid."
                    )
            elif (
                self.phase == MissionPhase.WAIT_FOR_TAKEOFF
                and self.drone_position is not None
            ):
                self.takeoff_start_position = self.drone_position.copy()
                self.transition_to(
                    MissionPhase.TAKEOFF,
                    "takeoff command received",
                )
        elif command in {"ARM", "ARMED"}:
            if self.drone_position is not None:
                self.initial_quad_z = float(self.drone_position[2])
            self.takeoff_command_time = None
            self.takeoff_detected_time = None
        elif command in {"DISARM", "DISARMED"}:
            if self.drone_position is not None:
                self.initial_quad_z = float(self.drone_position[2])
            self.takeoff_command_time = None
            self.takeoff_detected_time = None

    def external_landing_callback(self, msg: Bool) -> None:
        if not bool(msg.data):
            return
        if self.phase in {MissionPhase.LANDING, MissionPhase.LANDED_DISARMED}:
            return
        if self.drone_position is None:
            self.external_landing_pending = True
            self.get_logger().warning(
                "Landing request latched until the first drone-state sample."
            )
            return
        if self.is_m2b and self.phase == MissionPhase.M2_ATTACHED_HOLD:
            self.transition_to(
                MissionPhase.M2_LANDING_STAGE,
                f"external M2 landing request on {self.external_landing_topic}; release before landing",
            )
            return
        self.transition_to(
            MissionPhase.LANDING,
            f"external landing request on {self.external_landing_topic}",
        )

    def operator_advance_override_callback(self, msg: Bool) -> None:
        """Latch one explicit commissioning progression request.

        The callback itself never changes mission phase or authority.  The normal
        state-machine tick evaluates the physical prerequisites and either accepts
        the narrow loaded-lift override or records a rejection.
        """
        if not bool(msg.data):
            return

        self.operator_override_request_count += 1
        self.operator_override_last_from_phase = self.phase.value
        self.operator_override_last_action = "WAIVE_LOADED_LIFT_Z_GATE"

        if not self.enable_operator_advance_override:
            self.operator_override_last_result = "REJECTED_DISABLED"
            self.get_logger().warning(
                "Operator ADVANCE override rejected: commissioning override is disabled."
            )
            return
        if self.phase != MissionPhase.LIFT_OBJECT:
            self.operator_override_last_result = "REJECTED_UNSUPPORTED_PHASE"
            self.get_logger().warning(
                "Operator ADVANCE override rejected: supported only in LIFT_OBJECT; "
                f"current phase={self.phase.value}."
            )
            return
        if not self.c1f2_cpp_authority_enabled:
            self.operator_override_last_result = "REJECTED_C1F2_DISABLED"
            self.get_logger().warning(
                "Operator ADVANCE override rejected: C1F.2 C++ handoff is disabled."
            )
            return
        if self.c1f2_prepare_active:
            self.operator_override_last_result = "REJECTED_PREPARE_ALREADY_ACTIVE"
            self.get_logger().warning(
                "Operator ADVANCE override rejected: C1F.2 preparation is already active."
            )
            return

        self.operator_advance_override_pending = True
        self.operator_override_last_result = "PENDING"
        self.get_logger().warning(
            "Operator ADVANCE override requested in LIFT_OBJECT; physical loaded-lift "
            "prerequisites will be checked on the next planner tick."
        )

    def _fresh(self, value, timestamp: Optional[float], name: str) -> bool:
        if value is None or timestamp is None:
            self.get_logger().warn(
                f"Waiting for {name}...",
                throttle_duration_sec=1.0,
            )
            return False
        age = self._physical_now_s() - timestamp
        if age > self.input_timeout_s:
            self.get_logger().warn(
                f"{name} is stale ({age:.2f} s > {self.input_timeout_s:.2f} s).",
                throttle_duration_sec=1.0,
            )
            return False
        return True

    def inputs_ready(self) -> bool:
        if not self._fresh(self.drone_position, self.last_drone_time, "drone state"):
            return False
        if self.is_m2b and self.m2d_enabled and not self.m2d_assignment_received:
            return False
        if self.is_m2b and self.m2b_ring_pose_source == "pose_array":
            if not self._fresh(
                self.m2b_live_ring_position,
                self.m2b_last_ring_pose_time_s,
                "M2B ring pose",
            ):
                return False
        if self.use_measured_magnet_tip and not self._fresh(
            self.measured_magnet_tip_position,
            self.last_magnet_tip_pose_time,
            "magnet-tip pose",
        ):
            return False

        source = self.current_phase_spec().target_source
        if source == TargetSource.PICKUP_OBJECT:
            if self.use_static_pickup_object:
                self.last_pickup_object_pose_time = self._physical_now_s()
            return self._fresh(
                self.pickup_object_position,
                self.last_pickup_object_pose_time,
                "pickup-object pose",
            )
        if source == TargetSource.DROP_POINT:
            if self.drop_point_source in {"static", "fixed", "hardcoded"}:
                return True
            if self.drop_point_source == "payload":
                return self._fresh(
                    self.payload_position,
                    self.last_payload_pose_time,
                    "drop/payload pose",
                ) and self._fresh(
                    self.payload_velocity,
                    self.last_payload_twist_time,
                    "drop/payload twist",
                )
            return self._fresh(
                self.attachment_position,
                self.last_attachment_pose_time,
                "drop attachment pose",
            ) and self._fresh(
                self.attachment_velocity,
                self.last_attachment_twist_time,
                "drop attachment twist",
            )
        if source == TargetSource.ATTACHMENT_POINT:
            return self._fresh(
                self.attachment_position,
                self.last_attachment_pose_time,
                "attachment pose",
            ) and self._fresh(
                self.attachment_velocity,
                self.last_attachment_twist_time,
                "attachment twist",
            )
        return True

    # ----------------------------------------------------------------------
    # C1F.1 passive transfer-backend shadow integration
    # ----------------------------------------------------------------------
    def c1f1_shadow_reference_callback(
        self, msg: MultiDOFJointTrajectory
    ) -> None:
        if not self.c1f1_shadow_transfer_enabled:
            return
        now = self._physical_now_s()
        self.c1f1_shadow_reference_source_stamp_s = (
            float(msg.header.stamp.sec) + 1e-9 * float(msg.header.stamp.nanosec)
        )
        if self.c1f1_shadow_reference_last_receive_time is not None:
            self.c1f1_shadow_reference_last_period_ms = 1000.0 * (
                now - self.c1f1_shadow_reference_last_receive_time
            )
        self.c1f1_shadow_reference_last_receive_time = now
        self.c1f1_shadow_reference_count += 1
        self.c1f1_shadow_reference_sample_count = len(msg.points)
        positions = []
        velocities = []
        accelerations = []
        for point in msg.points:
            if not point.transforms or not point.velocities or not point.accelerations:
                self.c1f1_shadow_reference = None
                break
            translation = point.transforms[0].translation
            velocity = point.velocities[0].linear
            acceleration = point.accelerations[0].linear
            positions.append([translation.x, translation.y, translation.z])
            velocities.append([velocity.x, velocity.y, velocity.z])
            accelerations.append([acceleration.x, acceleration.y, acceleration.z])
        else:
            if positions:
                try:
                    self.c1f1_shadow_reference = TrajectoryReference(
                        np.asarray(positions, dtype=float),
                        np.asarray(velocities, dtype=float),
                        np.asarray(accelerations, dtype=float),
                    )
                except ValueError:
                    self.c1f1_shadow_reference = None
        self.c1f1_shadow_reference_terminal[:] = np.nan
        if msg.points and msg.points[-1].transforms:
            terminal = msg.points[-1].transforms[0].translation
            self.c1f1_shadow_reference_terminal = np.array(
                [terminal.x, terminal.y, terminal.z], dtype=float
            )
        self._c1f2_try_finalize_authority_handoff()

    def c1f1_shadow_status_callback(self, msg: String) -> None:
        if not self.c1f1_shadow_transfer_enabled:
            return
        self.c1f1_shadow_status_last_receive_time = self._physical_now_s()
        latest_status = ""
        for line in msg.data.splitlines():
            if line.startswith("latest_status:"):
                latest_status = line.split(":", 1)[1].strip()
                break
        self.c1f1_shadow_backend_status = latest_status or "status_received"

    def _c1f2_try_finalize_authority_handoff(self) -> None:
        """Complete the handoff only with a post-grant C++ reference.

        The backend may either freshly collision-certify an M2D rebased prepared
        trajectory or obtain the commissioned fresh solve from the stationary
        grant state. This helper does not care which path won: Python still
        requires the backend ACK, a post-grant 61-sample window, and sample-0
        continuity with the physical hold through p/v/a before leaving the hold.
        """
        if not (
            self.c1f2_prepare_phase_active()
            and self.c1f2_authority_grant_pending
            and self.c1f2_commanded_authority
            and self.c1f2_authority_acknowledged
            and self.c1f2_authority_grant_time is not None
            and self.c1f2_authority_grant_ros_time_s is not None
            and self.c1f2_authority_ack_time is not None
            and self.c1f1_shadow_reference_last_receive_time is not None
            and self.c1f1_shadow_reference_source_stamp_s is not None
            and post_grant_reference_is_fresh(
                reference_source_stamp_s=self.c1f1_shadow_reference_source_stamp_s,
                grant_ros_time_s=self.c1f2_authority_grant_ros_time_s,
                reference_receive_time=self.c1f1_shadow_reference_last_receive_time,
                ack_receive_time=self.c1f2_authority_ack_time,
            )
            and self.c1f1_shadow_reference_sample_count == 61
            and self.c1f1_shadow_reference is not None
            and self.c1f2_prepare_hold_position is not None
        ):
            return

        hold_position = np.asarray(self.c1f2_prepare_hold_position, dtype=float).reshape(3)
        hold_reference = TrajectoryReference(
            np.repeat(hold_position[None, :], 61, axis=0),
            np.zeros((61, 3), dtype=float),
            np.zeros((61, 3), dtype=float),
        )
        errors = stage0_errors(hold_reference, self.c1f1_shadow_reference)
        self.c1f2_handoff_position_error_m = errors.position_m
        self.c1f2_handoff_velocity_error_mps = errors.velocity_mps
        self.c1f2_handoff_acceleration_error_mps2 = errors.acceleration_mps2
        if not handoff_allowed(
            errors,
            position_tolerance_m=self.c1f2_handoff_position_tolerance_m,
            velocity_tolerance_mps=self.c1f2_handoff_velocity_tolerance_mps,
            acceleration_tolerance_mps2=self.c1f2_handoff_acceleration_tolerance_mps2,
        ):
            return

        self._c1f2_accept_cpp_candidate(self.c1f1_shadow_reference)
        self.c1f2_authority_grant_pending = False
        self.c1f2_handoff_ready = True
        self.get_logger().info(
            "C1F.8c certified post-grant C++ reference matches the stationary hold: "
            f"p={errors.position_m:.4f} m, v={errors.velocity_mps:.4f} m/s, "
            f"a={errors.acceleration_mps2:.4f} m/s^2. Transit handoff ready."
        )

    def c1f2_authority_ack_callback(self, msg: Bool) -> None:
        if not self.c1f2_cpp_authority_enabled:
            return
        if not msg.data:
            if not self.c1f2_cpp_authority_active:
                self.c1f2_authority_acknowledged = False
                self.c1f2_authority_ack_time = None
            return
        if not (
            self.c1f2_prepare_phase_active()
            and self.c1f2_authority_grant_pending
            and self.c1f2_commanded_authority
        ):
            return
        self.c1f2_authority_acknowledged = True
        self.c1f2_authority_ack_time = self._physical_now_s()
        self.get_logger().info(
            "C1F.8c C++ acknowledged a certified post-grant commitment; "
            "waiting for its post-grant reference window before leaving the hold."
        )
        self._c1f2_try_finalize_authority_handoff()

    def transfer_body_target(self) -> TargetState:
        """Return the exact body target used by the Python TRANSFER generator."""
        spec = self.current_phase_spec()
        if spec.reference_type != ReferenceType.TRANSFER:
            raise RuntimeError(
                f"Body transfer target requested outside TRANSFER phase {self.phase.value}."
            )
        target = self.target_state(spec.target_source)
        return TargetState(
            target.position + self.desired_tip_offset() + self.quad_reference_offset_from_tip(),
            target.velocity,
            target.acceleration,
        )

    def c1f1_shadow_phase_active(self) -> bool:
        return bool(
            self.c1f1_shadow_transfer_enabled
            and self.phase == MissionPhase.APPROACH_ABOVE_PICKUP
        )

    def c1f2_prepare_phase_active(self) -> bool:
        prepare_phase = bool(
            self.phase == MissionPhase.LIFT_OBJECT
            or (
                self.is_m2b
                and self.m2b_commissioning_stage == "b2"
                and self.phase == MissionPhase.M2_TAKEOFF_HOVER
            )
        )
        return bool(
            self.c1f1_shadow_transfer_enabled
            and self.c1f2_cpp_authority_enabled
            and prepare_phase
            and self.c1f2_prepare_active
            and self.c1f2_prepare_settled
        )

    def c1f2_cpp_phase_active(self) -> bool:
        return bool(
            self.c1f1_shadow_transfer_enabled
            and self.c1f2_cpp_authority_enabled
            and (
                self.c1f2_prepare_phase_active()
                or self.phase in {MissionPhase.TRANSIT_TO_DROP_POINT, MissionPhase.M2_CPP_TRANSIT_CAPTURE}
            )
        )

    def c1f2_cpp_transit_authority_active(self) -> bool:
        """True only while the established C++ transfer owns the transit reference."""
        return bool(
            self.c1f1_shadow_transfer_enabled
            and self.c1f2_cpp_authority_enabled
            and self.c1f2_cpp_authority_active
            and self.phase in {MissionPhase.TRANSIT_TO_DROP_POINT, MissionPhase.M2_CPP_TRANSIT_CAPTURE}
        )

    def c1f2_drop_target_inputs_ready(self) -> bool:
        """Require the future target state while holding before C++ transit."""
        if self.is_m2b and self.m2b_commissioning_stage == "b2":
            return True
        if self.drop_point_source in {"static", "fixed", "hardcoded"}:
            return True
        if self.drop_point_source == "payload":
            return self._fresh(
                self.payload_position, self.last_payload_pose_time, "drop/payload pose"
            ) and self._fresh(
                self.payload_velocity, self.last_payload_twist_time, "drop/payload twist"
            )
        return self._fresh(
            self.attachment_position, self.last_attachment_pose_time, "drop attachment pose"
        ) and self._fresh(
            self.attachment_velocity, self.last_attachment_twist_time, "drop attachment twist"
        )

    def c1f2_transfer_target_state(self) -> TargetState:
        """Return the C++ transit body target during prepare or transit."""
        if (
            self.is_m2b
            and self.m2b_commissioning_stage == "b2"
            and self.phase in {MissionPhase.M2_TAKEOFF_HOVER, MissionPhase.M2_CPP_TRANSIT_CAPTURE}
        ):
            return TargetState(self.m2b_capture_body_target(), np.zeros(3), np.zeros(3))
        if self.phase == MissionPhase.TRANSIT_TO_DROP_POINT:
            return self.transfer_body_target()
        if not self.c1f2_prepare_phase_active():
            raise RuntimeError("C1F.2b transfer target requested outside prepare/transit")
        target = self.drop_target_state()
        transit_tip_offset = np.array(
            [0.0, 0.0, self.drop_approach_clearance], dtype=float
        )
        # TRANSIT_TO_DROP_POINT does not use pickup XY compensation. Compute
        # exactly the body target that will become active after the handoff.
        body_offset = transit_tip_offset + self.nominal_quad_from_tip_offset()
        return TargetState(
            target.position + body_offset,
            target.velocity.copy(),
            target.acceleration.copy(),
        )

    def c1f1_publish_shadow_enable(self, enabled: bool) -> None:
        if self.c1f1_shadow_enable_publisher is None:
            return
        message = Bool()
        message.data = bool(enabled)
        self.c1f1_shadow_enable_publisher.publish(message)
        self.c1f1_shadow_commanded_enable = bool(enabled)

    def c1f1_publish_shadow_target(self, target: np.ndarray) -> None:
        if self.c1f1_shadow_target_publisher is None:
            return
        target = np.asarray(target, dtype=float).reshape(3)
        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.frame_id
        message.pose.position.x = float(target[0])
        message.pose.position.y = float(target[1])
        message.pose.position.z = float(target[2])
        message.pose.orientation.w = 1.0
        self.c1f1_shadow_target_publisher.publish(message)

    def c1f2_publish_target_velocity(self, velocity: np.ndarray) -> None:
        if self.c1f2_target_velocity_publisher is None:
            return
        velocity = np.asarray(velocity, dtype=float).reshape(3)
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.frame_id
        message.twist.linear.x = float(velocity[0])
        message.twist.linear.y = float(velocity[1])
        message.twist.linear.z = float(velocity[2])
        self.c1f2_target_velocity_publisher.publish(message)

    def c1f2_publish_authority(self, granted: bool) -> None:
        if self.c1f2_authority_publisher is None:
            return
        message = Bool()
        message.data = bool(granted)
        self.c1f2_authority_publisher.publish(message)
        self.c1f2_commanded_authority = bool(granted)

    def reset_c1f2_for_pickup_retry(self) -> None:
        """Cancel any in-progress C1F handoff before retrying a lost pickup."""
        self.c1f2_publish_authority(False)
        self.c1f2_cpp_authority_active = False
        self.c1f2_prepare_active = False
        self.c1f2_prepare_settled = False
        self.c1f2_prepare_lift_end_position = None
        self.c1f2_prepare_hold_position = None
        self.c1f2_prepare_settle_start_time = None
        self.c1f2_prepare_start_time = None
        self.c1f2_prepare_wait_s = float("nan")
        self.c1f2_handoff_ready = False
        self.c1f2_authority_grant_pending = False
        self.c1f2_authority_acknowledged = False
        self.c1f2_authority_grant_time = None
        self.c1f2_authority_grant_ros_time_s = None
        self.c1f2_authority_ack_time = None
        self.c1f2_cached_cpp_reference = None
        self.c1f2_cached_cpp_reference_receive_time = None
        self.c1f2_cached_continuation_active = False
        self.c1f1_shadow_latched_body_target = None
        self.c1f1_shadow_live_body_target = None
        self.c1f1_shadow_target_latch_time = None
        self.c1f1_shadow_target_drift_m = float("nan")

    def update_c1f1_shadow_transfer(self) -> None:
        """Drive C1F.1 target/enable topics without changing controller authority."""
        if not self.c1f1_shadow_transfer_enabled:
            return

        c1f1_active = self.c1f1_shadow_phase_active()
        c1f2_active = self.c1f2_cpp_phase_active()
        if not (c1f1_active or c1f2_active):
            if self.c1f2_commanded_authority:
                self.c1f2_publish_authority(False)
            if self.c1f1_shadow_commanded_enable:
                self.c1f1_publish_shadow_enable(False)
                self.get_logger().info(
                    "C++ transfer backend disabled outside its commissioned transfer phases."
                )
            self.c1f1_shadow_latched_body_target = None
            self.c1f1_shadow_live_body_target = None
            self.c1f1_shadow_target_latch_time = None
            self.c1f1_shadow_target_drift_m = float("nan")
            # C1F.2b does not keep C++ authority alive outside prepare/transit.
            # The C++ -> Python transition is handled by the explicit moving-frame
            # bridge latched at the phase boundary.
            self.c1f2_cpp_authority_active = False
            return

        if self.c1f2_prepare_phase_active() and not self.c1f2_drop_target_inputs_ready():
            self.c1f2_publish_authority(False)
            if self.c1f1_shadow_commanded_enable:
                self.c1f1_publish_shadow_enable(False)
            return

        live_target_state = (
            self.c1f2_transfer_target_state() if c1f2_active else self.transfer_body_target()
        )
        live_target = live_target_state.position.copy()
        self.c1f1_shadow_live_body_target = live_target
        if c1f2_active:
            # C1F.2b sends the moving DROP_POINT position and velocity while
            # Python either holds the completed loaded lift or C++ owns transit.
            # The backend updates its rendezvous goal only at replan boundaries,
            # preserving the currently committed trajectory between solves.
            self.c1f1_shadow_latched_body_target = live_target.copy()
            self.c1f1_shadow_target_latch_time = self._physical_now_s()
            self.c1f1_shadow_target_drift_m = 0.0
            self.c1f1_publish_shadow_target(live_target)
            self.c1f2_publish_target_velocity(live_target_state.velocity)
            self.c1f1_publish_shadow_enable(True)
            # Preparation is planning-only. Once the gate passes, the explicit
            # grant is latched and remains asserted through C++ transit authority.
            self.c1f2_publish_authority(
                self.c1f2_authority_grant_pending
                or self.c1f2_handoff_ready
                or self.c1f2_cpp_authority_active
            )
            return

        if self.c1f1_shadow_latched_body_target is None:
            self.c1f1_shadow_latched_body_target = live_target.copy()
            self.c1f1_shadow_target_latch_time = self._physical_now_s()
            self.get_logger().warn(
                "C1F.1 latched fixed shadow BODY target "
                f"[{live_target[0]:.3f}, {live_target[1]:.3f}, {live_target[2]:.3f}]. "
                "Python VirtualTransferGenerator remains controller authority."
            )

        assert self.c1f1_shadow_latched_body_target is not None
        self.c1f1_shadow_target_drift_m = float(
            np.linalg.norm(
                live_target - self.c1f1_shadow_latched_body_target
            )
        )
        if (
            self.c1f1_shadow_target_drift_warn_m > 0.0
            and self.c1f1_shadow_target_drift_m
            > self.c1f1_shadow_target_drift_warn_m
        ):
            self.get_logger().warn(
                "C1F.1 live Python body target has drifted "
                f"{self.c1f1_shadow_target_drift_m:.3f} m from the fixed C++ shadow latch.",
                throttle_duration_sec=1.0,
            )

        # Republish the same latched target and enable request at the planner's 30 Hz
        # tick.  This is intentionally redundant so a backend that starts late can
        # still join the shadow test.  The target itself never moves within the phase.
        self.c1f1_publish_shadow_target(self.c1f1_shadow_latched_body_target)
        self.c1f1_publish_shadow_enable(True)

    def _c1f2_advanced_cached_reference(self) -> Optional[TrajectoryReference]:
        if (
            self.c1f2_cached_cpp_reference is None
            or self.c1f2_cached_cpp_reference_receive_time is None
        ):
            self.c1f2_cached_reference_age_ms = float("nan")
            return None
        elapsed_s = max(
            0.0, self._physical_now_s() - self.c1f2_cached_cpp_reference_receive_time
        )
        self.c1f2_cached_reference_age_ms = 1000.0 * elapsed_s
        return advance_reference_window(
            self.c1f2_cached_cpp_reference,
            elapsed_s=elapsed_s,
            sample_dt_s=self.dt,
            output_count=61,
        )

    def _c1f2_accept_cpp_candidate(
        self, reference: TrajectoryReference
    ) -> None:
        self.c1f2_cached_cpp_reference = reference
        self.c1f2_cached_cpp_reference_receive_time = (
            self.c1f1_shadow_reference_last_receive_time
        )
        self.c1f2_cached_continuation_active = False
        self.c1f2_cached_reference_age_ms = self.c1f1_shadow_reference_age_ms()

    def _c1f2_current_authoritative_reference(self) -> Optional[TrajectoryReference]:
        """Return the best estimate of the controller reference executing *now*."""
        # Prefer the previously published post-safety reference and advance it by
        # the measured planner callback period. This preserves continuity even if
        # the Python safety layer ever modifies a C++ nominal reference.
        if self.last_reference is not None:
            period_s = (
                self.dt
                if not np.isfinite(self.last_planner_callback_period_ms)
                else max(0.0, self.last_planner_callback_period_ms / 1000.0)
            )
            return advance_reference_window(
                self.last_reference,
                elapsed_s=period_s,
                sample_dt_s=self.dt,
                output_count=int(self.last_reference.positions.shape[0]),
            )
        return self._c1f2_advanced_cached_reference()

    def c1f2_exit_bridge_target_and_offset(self) -> tuple[TargetState, np.ndarray]:
        if self.is_m2b and self.m2b_commissioning_stage == "b2":
            if self.m2d_enabled:
                ring_position = self.m2b_ring_pose_world()[0]
                target = TargetState(
                    ring_position,
                    np.zeros(3, dtype=float),
                    np.zeros(3, dtype=float),
                )
                relative_offset = self.m2b_capture_body_target() - ring_position
            else:
                # Preserve the commissioned B2 fixed-fixture bridge exactly when
                # M2D is not enabled.  Live measured ring targeting is an M2D-only
                # extension and must not silently rewrite the legacy B2 contract.
                target = TargetState(
                    self.m2b_fixed_ring_position.copy(),
                    np.zeros(3, dtype=float),
                    np.zeros(3, dtype=float),
                )
                relative_offset = (
                    self.m2b_capture_body_target() - self.m2b_fixed_ring_position
                )
            return target, relative_offset
        target = self.target_state(self.current_phase_spec().target_source)
        relative_offset = self.desired_tip_offset() + self.quad_reference_offset_from_tip()
        return target, relative_offset

    def _evaluate_c1f2_target_relative_exit_bridge(
        self,
    ) -> tuple[Optional[TargetRelativeBridgeState], Optional[TargetRelativeBridgeFeasibility]]:
        """Evaluate the exact bridge that would be used for C++ -> Python capture.

        The normal transition gate is intentionally *soft with respect to relative
        velocity*: there is no tight |dv| threshold.  It is hard only on whether
        the actual bridge can absorb the current relative p/v/a state within the
        configured absolute v/a/jerk envelopes.
        """
        outgoing = self._c1f2_current_authoritative_reference()
        if outgoing is None:
            self.c1f2_exit_bridge_feasible = False
            self.c1f2_exit_bridge_feasibility_reason = "NO_AUTHORITATIVE_REFERENCE"
            self.c1f2_exit_bridge_pending_state = None
            self.c1f2_exit_bridge_pending_feasibility = None
            return None, None
        target, relative_offset = self.c1f2_exit_bridge_target_and_offset()
        state = target_relative_bridge_initial_state(
            outgoing_position=outgoing.positions[0],
            outgoing_velocity=outgoing.velocities[0],
            outgoing_acceleration=outgoing.accelerations[0],
            target=target,
            relative_offset=relative_offset,
        )
        if self.c1f2_bridge_feasibility_enabled:
            feasibility = target_relative_bridge_feasibility(
                bridge_state=state,
                target=target,
                duration_s=self.c1f2_exit_bridge_duration_s,
                max_abs_velocity=self.c1f2_bridge_max_velocity,
                max_abs_acceleration=self.c1f2_bridge_max_acceleration,
                max_abs_jerk=self.c1f2_bridge_max_jerk,
                sample_dt_s=self.c1f2_bridge_feasibility_sample_dt_s,
            )
        else:
            zeros = np.zeros(3, dtype=float)
            feasibility = TargetRelativeBridgeFeasibility(
                feasible=True,
                reason="DISABLED",
                max_abs_velocity=zeros,
                max_abs_acceleration=zeros,
                max_abs_jerk=zeros,
                final_position_error_m=0.0,
                final_velocity_error_mps=0.0,
                final_acceleration_error_mps2=0.0,
            )
        self.c1f2_exit_bridge_pending_state = state
        self.c1f2_exit_bridge_pending_feasibility = feasibility
        self.c1f2_exit_bridge_feasible = bool(feasibility.feasible)
        self.c1f2_exit_bridge_feasibility_reason = feasibility.reason
        self.c1f2_exit_bridge_peak_velocity_mps = float(np.max(feasibility.max_abs_velocity))
        self.c1f2_exit_bridge_peak_acceleration_mps2 = float(np.max(feasibility.max_abs_acceleration))
        self.c1f2_exit_bridge_peak_jerk_mps3 = float(np.max(feasibility.max_abs_jerk))
        return state, feasibility

    def _start_c1f2_target_relative_exit_bridge(self) -> None:
        """Latch the already-certified outgoing p/v/a bridge state."""
        state = self.c1f2_exit_bridge_pending_state
        feasibility = self.c1f2_exit_bridge_pending_feasibility
        if state is None or feasibility is None or not feasibility.feasible:
            state, feasibility = self._evaluate_c1f2_target_relative_exit_bridge()
        if state is None or feasibility is None or not feasibility.feasible:
            reason = self.c1f2_exit_bridge_feasibility_reason
            raise RuntimeError(
                f"C1F.6 refused C++ -> Python bridge because it is not dynamically capturable: {reason}"
            )
        # The state was generated by the same quintic bridge model used below.
        # Using the current gate-cycle candidate avoids certifying one p/v/a state
        # and executing a different bridge after the phase switch.
        self.c1f2_exit_bridge_state = state
        self.c1f2_exit_bridge_initial_position_error_m = float(
            np.linalg.norm(state.position_error)
        )
        self.c1f2_exit_bridge_initial_velocity_error_mps = float(
            np.linalg.norm(state.velocity_error)
        )
        self.c1f2_exit_bridge_initial_acceleration_error_mps2 = float(
            np.linalg.norm(state.acceleration_error)
        )
        self.c1f2_exit_bridge_start_time = self.phase_entry_time
        self.c1f2_exit_bridge_active = True
        self.c1f2_exit_bridge_complete = False
        self.c1f2_cpp_authority_active = False
        self.c1f2_cached_continuation_active = False
        self.c1f2_exit_switch_count += 1
        self.c1f2_publish_authority(False)
        self.c1f2_cached_cpp_reference = None
        self.c1f2_cached_cpp_reference_receive_time = None
        self.c1f2_exit_bridge_pending_state = None
        self.c1f2_exit_bridge_pending_feasibility = None
        self.get_logger().info(
            "C1F.6 started dynamically-capturable C++ -> Python moving-frame bridge: "
            f"relative p={self.c1f2_exit_bridge_initial_position_error_m:.4f} m, "
            f"v={self.c1f2_exit_bridge_initial_velocity_error_mps:.4f} m/s, "
            f"a={self.c1f2_exit_bridge_initial_acceleration_error_mps2:.4f} m/s^2, "
            f"peak |v|={self.c1f2_exit_bridge_peak_velocity_mps:.3f} m/s, "
            f"peak |a|={self.c1f2_exit_bridge_peak_acceleration_mps2:.3f} m/s^2, "
            f"peak |j|={self.c1f2_exit_bridge_peak_jerk_mps3:.3f} m/s^3, "
            f"duration={self.c1f2_exit_bridge_duration_s:.2f} s."
        )

    def _build_c1f2_target_relative_exit_bridge(
        self, target: TargetState, relative_offset: np.ndarray
    ) -> Optional[TrajectoryReference]:
        if not self.c1f2_exit_bridge_active or self.c1f2_exit_bridge_state is None:
            return None
        if self.c1f2_exit_bridge_start_time is None:
            raise RuntimeError("C1F.2b exit bridge active without start time")
        elapsed_s = max(0.0, self._physical_now_s() - self.c1f2_exit_bridge_start_time)
        reference, complete = target_relative_bridge_reference(
            bridge_state=self.c1f2_exit_bridge_state,
            target=target,
            relative_offset=relative_offset,
            elapsed_s=elapsed_s,
            duration_s=self.c1f2_exit_bridge_duration_s,
            dt=self.dt,
            horizon_samples=self.horizon_samples,
        )
        if complete and not self.c1f2_exit_bridge_complete:
            self.c1f2_exit_bridge_complete = True
            self.c1f2_exit_bridge_active = False
            self.get_logger().info(
                "C1F.2b moving-frame bridge complete; ordinary Python "
                "TARGET_RELATIVE_TRACKING is now authoritative."
            )
        return reference

    def select_c1f2_authority_reference(
        self,
        python_reference: Optional[TrajectoryReference],
    ) -> TrajectoryReference:
        """Select/continue C++ authority for the commissioned loaded transfer.

        C1F.8c prepares C++ while the completed LIFT_OBJECT reference is held
        stationary. M2D may rebase and freshly collision-certify that prepared
        shape; other profiles retain the commissioned fresh post-request solve.
        In both cases a post-grant 61-sample window must match the hold through
        p/v/a before transit.
        Once transit begins, C++ remains authoritative; publication dropouts
        advance through the last committed window rather than falling back to a
        separately moving Python transfer.
        """
        prepare_active = self.c1f2_prepare_phase_active()
        transit_active = bool(
            self.c1f2_cpp_authority_enabled
            and self.phase in {MissionPhase.TRANSIT_TO_DROP_POINT, MissionPhase.M2_CPP_TRANSIT_CAPTURE}
        )

        def require_python_reference() -> TrajectoryReference:
            if python_reference is None:
                raise RuntimeError(
                    "Python reference omitted outside established C++ transit authority"
                )
            return python_reference
        if not prepare_active and not transit_active:
            self.c1f2_cpp_authority_active = False
            self.c1f2_cached_cpp_reference = None
            self.c1f2_cached_cpp_reference_receive_time = None
            self.c1f2_cached_continuation_active = False
            self.c1f2_cached_reference_age_ms = float("nan")
            return require_python_reference()

        cpp_reference = self.c1f1_shadow_reference
        fresh_candidate_ready = bool(
            self.c1f1_shadow_reference_fresh()
            and self.c1f1_shadow_reference_sample_count == 61
            and cpp_reference is not None
            and cpp_reference.positions.shape[0] == 61
        )

        if prepare_active:
            # C1F.8c2: a prepared plan can take several seconds to appear in a
            # moving world. The original settle event is therefore not a current
            # physical-state certificate. Revalidate measured p/v/a on every
            # prepare tick and keep the diagnostics live.
            if (
                self.drone_position is None
                or self.drone_velocity is None
                or self.drone_acceleration is None
                or self.c1f2_prepare_lift_end_position is None
            ):
                return require_python_reference()
            grant_state = stationary_grant_state(
                lift_end_position=self.c1f2_prepare_lift_end_position,
                measured_position=self.drone_position,
                measured_velocity=self.drone_velocity,
                measured_acceleration=self.drone_acceleration,
                max_drift_m=self.c1f2_prepare_max_lift_drift_m,
                settle_speed_mps=self.c1f2_prepare_settle_speed_mps,
                settle_acceleration_mps2=self.c1f2_handoff_acceleration_tolerance_mps2,
            )
            self.c1f2_prepare_drift_m = grant_state.drift_m
            self.c1f2_prepare_speed_mps = grant_state.speed_mps
            if self.c1f2_prepare_settled and not grant_state.ready:
                self.c1f2_prepare_settled = False
                self.c1f2_prepare_settle_start_time = None
                if self.c1f2_authority_grant_pending and not self.c1f2_cpp_authority_active:
                    # A grant attempt that has not been acknowledged is no longer
                    # valid once the physical hold leaves its prepare bounds. Clear
                    # Python bookkeeping and force one backend disable/re-enable
                    # epoch so a later re-settle can make a genuinely new request.
                    self.c1f2_authority_grant_pending = False
                    self.c1f2_authority_acknowledged = False
                    self.c1f2_authority_grant_time = None
                    self.c1f2_authority_grant_ros_time_s = None
                    self.c1f2_authority_ack_time = None
                    self.c1f2_handoff_ready = False
                    self.c1f2_cached_cpp_reference = None
                    self.c1f2_cached_cpp_reference_receive_time = None
                    self.c1f2_cached_continuation_active = False
                    self.c1f2_publish_authority(False)
                    self.c1f1_publish_shadow_enable(False)
                    self.get_logger().warning(
                        "C1F.8c2 cancelled an unacknowledged authority attempt after "
                        "the stationary prepare bounds were lost; backend epoch reset "
                        "so the next settled hold can retry cleanly."
                    )
                self.get_logger().warning(
                    "C1F.8c2 live stationary grant state left the prepare bounds; "
                    f"drift={grant_state.drift_m:.3f} m, "
                    f"speed={grant_state.speed_mps:.3f} m/s, "
                    f"accel={grant_state.acceleration_mps2:.3f} m/s^2. "
                    "Returning to the settle dwell before any authority request.",
                    throttle_duration_sec=1.0,
                )
                return require_python_reference()

            # Python deliberately keeps the completed loaded-lift hold
            # authoritative. C++ may plan for as long as required. We only set
            # handoff_ready here; update_state_machine performs the actual phase
            # transition on the next planner tick.
            if (
                self.c1f2_handoff_ready
                or self.c1f2_authority_grant_pending
                or not self.c1f2_prepare_settled
                or not fresh_candidate_ready
            ):
                return require_python_reference()
            assert cpp_reference is not None

            # The old fixed hold may accumulate a few centimetres of steady-state
            # tracking bias while C++ searches. At the instant a prepared candidate
            # proves readiness, re-latch the current measured stationary state.
            # This is the physical state from which the backend also performs its
            # fresh post-request solve.
            grant_hold_position = self.drone_position.copy()
            grant_hold_reference = stationary_regulation(
                grant_hold_position,
                horizon_samples=self.horizon_samples,
                min_reference_z=self.min_reference_z,
            )
            errors = stage0_errors(grant_hold_reference, cpp_reference)
            self.c1f2_handoff_position_error_m = errors.position_m
            self.c1f2_handoff_velocity_error_mps = errors.velocity_mps
            self.c1f2_handoff_acceleration_error_mps2 = errors.acceleration_mps2
            if not handoff_allowed(
                errors,
                position_tolerance_m=self.c1f2_handoff_position_tolerance_m,
                velocity_tolerance_mps=self.c1f2_handoff_velocity_tolerance_mps,
                acceleration_tolerance_mps2=self.c1f2_handoff_acceleration_tolerance_mps2,
            ):
                return require_python_reference()

            # From this callback onward Python commands the same freshly latched
            # stationary state used by the authority handshake. This avoids a
            # reference jump caused solely by slow prepared-plan availability.
            self.c1f2_prepare_hold_position = grant_hold_position

            # The backend decides whether the prepared trajectory can be rebased
            # and freshly collision-certified for execution (M2D), or whether it
            # must fall back to the commissioned fresh post-grant solve. Python
            # remains on the measured stationary hold until the backend ACK and a
            # post-grant 61-sample reference arrive.
            self.c1f2_cached_cpp_reference = None
            self.c1f2_cached_cpp_reference_receive_time = None
            self.c1f2_cached_continuation_active = False
            self.c1f2_authority_grant_pending = True
            self.c1f2_authority_acknowledged = False
            self.c1f2_authority_ack_time = None
            self.c1f2_authority_grant_time = self._physical_now_s()
            self.c1f2_authority_grant_ros_time_s = (
                1e-9 * float(self.get_clock().now().nanoseconds)
            )
            self.c1f2_publish_authority(True)
            self.c1f2_authority_grant_count += 1
            self.get_logger().info(
                "C1F.8c2 prepared C++ transfer matches freshly re-latched measured "
                f"grant hold: p={errors.position_m:.4f} m, "
                f"v={errors.velocity_mps:.4f} m/s, "
                f"a={errors.acceleration_mps2:.4f} m/s^2. "
                "Authority request sent; holding until C++ returns a freshly certified "
                "post-grant reference."
            )
            return grant_hold_reference

        if not self.c1f2_cpp_authority_active:
            # C1F.2b should only enter TRANSIT after the stationary prepare gate
            # has succeeded. Refuse to recreate the old moving Python->C++ chase.
            self.get_logger().error(
                "C1F.2b C++ transit phase reached without C++ authority; "
                "holding the previous reference instead of initiating a moving handoff.",
                throttle_duration_sec=1.0,
            )
            if self.last_reference is not None:
                return self.last_reference
            return require_python_reference()

        cached_now = self._c1f2_advanced_cached_reference()
        if cached_now is None:
            # This should not occur once authority has been granted. Do not silently
            # jump to an unrelated Python trajectory; retain the previously
            # authoritative window if one exists and make the fault conspicuous.
            self.c1f2_cached_continuation_active = True
            self.get_logger().error(
                "C1F.2b lost its accepted C++ cache while authority was active; "
                "retaining the previous authoritative window."
            )
            if self.last_reference is not None:
                return self.last_reference
            raise RuntimeError("C1F.2b authority active without cached or previous reference")

        if not fresh_candidate_ready:
            if not self.c1f2_cached_continuation_active:
                self.c1f2_cached_continuation_count += 1
                self.get_logger().warning(
                    "C1F.2b C++ reference is stale/unavailable; continuing the last "
                    "committed C++ window toward its stopped terminal hold."
                )
            self.c1f2_cached_continuation_active = True
            return cached_now

        assert cpp_reference is not None
        candidate_now = advance_reference_window(
            cpp_reference,
            elapsed_s=max(0.0, self.c1f1_shadow_reference_age_ms() / 1000.0),
            sample_dt_s=self.dt,
            output_count=61,
        )

        if self.c1f2_cached_continuation_active:
            recovery_errors = stage0_errors(cached_now, candidate_now)
            if not handoff_allowed(
                recovery_errors,
                position_tolerance_m=self.c1f2_handoff_position_tolerance_m,
                velocity_tolerance_mps=self.c1f2_handoff_velocity_tolerance_mps,
                acceleration_tolerance_mps2=self.c1f2_handoff_acceleration_tolerance_mps2,
            ):
                self.c1f2_recovery_reject_count += 1
                return cached_now
            self.get_logger().info(
                "C1F.2b recovered fresh C++ reference continuously after cached continuation: "
                f"p={recovery_errors.position_m:.4f} m, "
                f"v={recovery_errors.velocity_mps:.4f} m/s, "
                f"a={recovery_errors.acceleration_mps2:.4f} m/s^2."
            )

        self._c1f2_accept_cpp_candidate(cpp_reference)
        return candidate_now

    def c1f1_shadow_reference_age_ms(self) -> float:
        if self.c1f1_shadow_reference_last_receive_time is None:
            return float("nan")
        return 1000.0 * max(
            0.0, self._physical_now_s() - self.c1f1_shadow_reference_last_receive_time
        )

    def c1f1_shadow_reference_fresh(self) -> bool:
        age_ms = self.c1f1_shadow_reference_age_ms()
        return bool(
            np.isfinite(age_ms)
            and age_ms <= 1000.0 * self.c1f1_shadow_reference_timeout_s
        )

    def c1f1_shadow_status_age_ms(self) -> float:
        if self.c1f1_shadow_status_last_receive_time is None:
            return float("nan")
        return 1000.0 * max(
            0.0, self._physical_now_s() - self.c1f1_shadow_status_last_receive_time
        )

    # ----------------------------------------------------------------------
    # Mission, target and geometry abstractions
    # ----------------------------------------------------------------------
    def current_phase_spec(self):
        return self.mission.spec(self.phase)

    def phase_elapsed(self) -> float:
        return max(0.0, self._physical_now_s() - self.phase_entry_time)

    def transition_to(self, new_phase: MissionPhase, reason: str) -> None:
        if new_phase == self.phase:
            return

        # Evaluate the live geometry before changing phase.  Once DESCEND_TO_PICKUP
        # begins, the target must remain fixed even if mocap becomes stale or noisy.
        pending_geometry: Optional[PickupTargetGeometry] = None
        pending_geometry_source = ""
        pending_geometry_error = ""
        if new_phase == MissionPhase.DESCEND_TO_PICKUP and self.use_geometry_aware_pickup:
            (
                pending_geometry,
                _,
                _,
                pending_geometry_source,
                pending_geometry_error,
            ) = self.compute_pickup_target_geometry(allow_latched=False)

        old_phase = self.phase
        if old_phase == MissionPhase.LIFT_OBJECT and new_phase != MissionPhase.LIFT_OBJECT:
            if self.operator_advance_override_pending:
                self.operator_override_last_result = "REJECTED_PHASE_EXIT"
                self.get_logger().warning(
                    "Pending operator ADVANCE override cancelled because LIFT_OBJECT exited "
                    f"to {new_phase.value} before acceptance."
                )
            self.operator_advance_override_pending = False
        self.phase = new_phase
        self.phase_entry_time = self._physical_now_s()
        self.publish_phase()
        self.condition_start_time = None
        self.last_transition_reason = str(reason)
        self.transfer_generator.reset()
        self.last_rate_profile_duration = float("nan")
        self.last_rate_profile_complete = False

        if self.is_m2b:
            if new_phase == MissionPhase.M2_TAKEOFF_HOVER:
                self.m2b_takeoff_settle_gate.reset()
            elif new_phase == MissionPhase.M2_SETTLE_CAPTURE:
                self.m2b_capture_settle_gate.reset()
                self.m2b_capture_settle_position_error_m = float("nan")
                self.m2b_capture_settle_speed_mps = float("nan")
                self.m2b_capture_settle_swing_deg = float("nan")
                self.m2b_capture_settle_dwell_s = 0.0
                self.m2b_capture_settle_reason = "NOT_EVALUATED"
                self.m2b_capture_settle_settled = False
                self.m2b_magnet_on_requested = False
            elif new_phase == MissionPhase.M2_ATTACH_CAPTURE_WAIT:
                self.m2b_latch_hold_position = None
                self.m2b_latch_stability_gate = None
                self.m2b_latch_stable = False
                self.m2b_last_latch_stability_decision = None
            elif new_phase == MissionPhase.M2_ATTACH_PROOF:
                self.m2b_proof_start_position = (
                    None if self.drone_position is None else self.drone_position.copy()
                )
            elif new_phase == MissionPhase.M2_ATTACHED_HOLD:
                self.m2b_attached_bad_since_time_s = None
            elif new_phase == MissionPhase.M2_DETACHED_RETREAT:
                self.m2b_magnet_on_requested = False
                self.m2b_retreat_stage = "wait_detach"
                self.m2b_retreat_target = (
                    None if self.drone_position is None else self.drone_position.copy()
                )
                if self.m2b_attachment_truth_source == "joint_truth":
                    self.m2b_request_detector_reset()
            elif new_phase == MissionPhase.M2_LANDING_STAGE:
                self.m2b_magnet_on_requested = False
                if self.m2b_attachment_truth_source == "joint_truth":
                    self.m2b_request_detector_reset()
                if self.drone_position is not None:
                    self.m2b_landing_detach_hold_position = self.drone_position.copy()
                    target = self.drone_position.copy()
                    if self.takeoff_start_position is not None:
                        target[:2] = self.takeoff_start_position[:2]
                        target[2] = max(
                            target[2],
                            float(self.takeoff_start_position[2] + self.takeoff_height),
                        )
                    self.m2b_landing_stage_target = target
            elif new_phase == MissionPhase.M2_FAULT:
                self.m2b_magnet_on_requested = False
                self.m2b_fault_hold_position = (
                    None if self.drone_position is None else self.drone_position.copy()
                )

        if new_phase in {
            MissionPhase.WAIT_FOR_TAKEOFF,
            MissionPhase.APPROACH_ABOVE_PICKUP,
        }:
            self.latched_pickup_position = None
            self.latched_pickup_yaw = None
            self.clear_geometry_pickup_latch()
        if new_phase == MissionPhase.DESCEND_TO_PICKUP:
            if self.pickup_object_position is not None:
                self.latched_pickup_position = self.pickup_object_position.copy()
                self.latched_pickup_yaw = float(self.pickup_object_yaw)
            if self.use_geometry_aware_pickup:
                if pending_geometry is None:
                    self.clear_geometry_pickup_latch()
                    self.geometry_pickup_latch_source = pending_geometry_source
                    self.geometry_pickup_latch_error = (
                        pending_geometry_error or pending_geometry_source
                    )
                else:
                    self.latch_geometry_pickup_targets(
                        pending_geometry,
                        pending_geometry_source,
                    )
        elif new_phase == MissionPhase.MAGNET_ATTACH_WAIT:
            self.attach_wait_start_time = self._physical_now_s()
        elif new_phase == MissionPhase.M2_CPP_TRANSIT_CAPTURE:
            if self.c1f2_cpp_authority_enabled:
                if (
                    not self.c1f2_handoff_ready
                    or not self.c1f2_authority_acknowledged
                    or self.c1f2_cached_cpp_reference is None
                ):
                    raise RuntimeError(
                        "M2B B2 refused C++ transit without an acknowledged fresh post-grant C++ reference"
                    )
                self.c1f2_cpp_authority_active = True
                self.c1f2_authority_switch_count += 1
                self.c1f2_prepare_active = False
                self.get_logger().warn(
                    "M2B B2 authority source switched stationary Python hold -> C++ at capture-transit entry."
                )
        elif new_phase == MissionPhase.TRANSIT_TO_DROP_POINT:
            # The pickup-relative offset remains authoritative during loaded lift
            # so an off-centre attachment does not create a lateral target jump.
            self.clear_geometry_pickup_latch()
            if self.c1f2_cpp_authority_enabled:
                if (
                    not self.c1f2_handoff_ready
                    or not self.c1f2_authority_acknowledged
                    or self.c1f2_cached_cpp_reference is None
                ):
                    raise RuntimeError(
                        "C1F.2b refused TRANSIT_TO_DROP_POINT without an acknowledged fresh post-grant C++ reference"
                    )
                self.c1f2_cpp_authority_active = True
                self.c1f2_authority_switch_count += 1
                self.c1f2_prepare_active = False
                self.get_logger().warn(
                    "C1F.2b authority source switched stationary Python hold -> C++ "
                    "at TRANSIT_TO_DROP_POINT entry."
                )
        elif (
            (
                new_phase == MissionPhase.SETTLE_ABOVE_DROP_POINT
                and old_phase == MissionPhase.TRANSIT_TO_DROP_POINT
            )
            or (
                new_phase == MissionPhase.M2_SETTLE_CAPTURE
                and old_phase == MissionPhase.M2_CPP_TRANSIT_CAPTURE
            )
        ) and self.c1f2_cpp_authority_enabled and self.c1f2_cpp_authority_active:
            self._start_c1f2_target_relative_exit_bridge()
        elif new_phase == MissionPhase.DROP_OBJECT:
            self.drop_start_time = self._physical_now_s()
        elif new_phase == MissionPhase.LANDING:
            self.clear_geometry_pickup_latch()
            self.initialise_landing()

        self.get_logger().info(
            f"Planner phase: {old_phase.value} -> {new_phase.value}. Reason: {reason}"
        )

    def condition_true_for(self, condition: bool, duration: float) -> bool:
        if not condition:
            self.condition_start_time = None
            return False
        now = self._physical_now_s()
        if self.condition_start_time is None:
            self.condition_start_time = now
            return duration <= 0.0
        return now - self.condition_start_time >= max(0.0, float(duration))

    def geometry_pickup_approach_height(self) -> float:
        """Preserve the legacy approach-to-contact vertical separation."""

        return max(
            0.0,
            float(self.pickup_approach_clearance - self.pickup_attach_clearance),
        )

    def clear_geometry_pickup_latch(self) -> None:
        self.latched_geometry_pickup_contact_target = None
        self.latched_geometry_pickup_approach_target = None
        self.latched_geometry_pickup_contact_offset = None
        self.geometry_pickup_latch_source = "unavailable"
        self.geometry_pickup_latch_error = ""

    def latch_geometry_pickup_targets(
        self,
        geometry: PickupTargetGeometry,
        source: str,
    ) -> None:
        self.latched_geometry_pickup_contact_target = (
            geometry.magnet_marker_target_world.copy()
        )
        self.latched_geometry_pickup_approach_target = (
            geometry.approach_marker_target_world(
                self.geometry_pickup_approach_height()
            )
        )
        if self.latched_pickup_position is None:
            raise RuntimeError("Cannot latch geometry target without pickup pose")
        self.latched_geometry_pickup_contact_offset = (
            self.latched_geometry_pickup_contact_target
            - self.latched_pickup_position
        )
        self.geometry_pickup_latch_source = str(source)
        self.geometry_pickup_latch_error = ""

    def pickup_geometry_pose(
        self,
        *,
        allow_latched: bool = True,
    ) -> Tuple[Optional[np.ndarray], Optional[float], Optional[float], str]:
        """Return the fresh or committed pickup pose used by V5B."""

        if self.pickup_object_position is None:
            return None, None, None, "missing_pickup_pose"

        use_latched = (
            allow_latched
            and self.latched_pickup_position is not None
            and self.phase
            not in {
                MissionPhase.WAIT_FOR_TAKEOFF,
                MissionPhase.APPROACH_ABOVE_PICKUP,
                MissionPhase.SETTLE_ABOVE_PICKUP,
            }
        )
        if use_latched:
            yaw = (
                float(self.pickup_object_yaw)
                if self.latched_pickup_yaw is None
                else float(self.latched_pickup_yaw)
            )
            return (
                self.latched_pickup_position.copy(),
                yaw,
                0.0,
                "latched_pickup_pose",
            )

        pose_age = (
            None
            if self.last_pickup_object_pose_time is None
            else max(0.0, self._physical_now_s() - self.last_pickup_object_pose_time)
        )
        if (
            not self.use_static_pickup_object
            and (pose_age is None or pose_age > self.payload_pose_timeout_s)
        ):
            return None, None, pose_age, "stale_pickup_pose"
        return (
            self.pickup_object_position.copy(),
            float(self.pickup_object_yaw),
            0.0 if self.use_static_pickup_object else pose_age,
            "static_pickup_pose"
            if self.use_static_pickup_object
            else "fresh_pickup_pose",
        )

    def compute_pickup_target_geometry(
        self,
        *,
        allow_latched: bool = True,
    ) -> Tuple[
        Optional[PickupTargetGeometry],
        Optional[np.ndarray],
        Optional[float],
        str,
        str,
    ]:
        """Compute the shared V5B target and return availability diagnostics."""

        pose_position, yaw, pose_age, source = self.pickup_geometry_pose(
            allow_latched=allow_latched
        )
        if pose_position is None or yaw is None:
            return None, pose_position, pose_age, source, ""
        try:
            geometry = self.payload_profile.pickup_target_geometry(
                pose_position=pose_position,
                yaw=yaw,
                contact_gap=self.pickup_contact_gap,
                overtravel=self.pickup_overtravel,
                max_overtravel=self.pickup_max_overtravel,
                support_surface_z=self.support_surface_z,
            )
        except (TypeError, ValueError) as exc:
            return None, pose_position, pose_age, "invalid_profile_or_pose", str(exc)
        return geometry, pose_position, pose_age, source, ""

    def geometry_aware_pickup_target_for_phase(
        self,
        phase: Optional[MissionPhase] = None,
    ) -> Tuple[Optional[np.ndarray], str, str]:
        """Return the active absolute marker target, mode, and fallback reason."""

        phase = self.phase if phase is None else phase
        pickup_phases = {
            MissionPhase.APPROACH_ABOVE_PICKUP,
            MissionPhase.SETTLE_ABOVE_PICKUP,
            MissionPhase.DESCEND_TO_PICKUP,
            MissionPhase.MAGNET_ATTACH_WAIT,
            MissionPhase.LIFT_OBJECT,
        }
        if phase not in pickup_phases:
            return None, "not_applicable", ""
        if not self.use_geometry_aware_pickup:
            return None, "disabled", "use_geometry_aware_pickup=false"

        if phase in {
            MissionPhase.DESCEND_TO_PICKUP,
            MissionPhase.MAGNET_ATTACH_WAIT,
            MissionPhase.LIFT_OBJECT,
        }:
            if (
                self.latched_geometry_pickup_contact_target is None
                or self.latched_geometry_pickup_approach_target is None
                or self.latched_geometry_pickup_contact_offset is None
            ):
                reason = self.geometry_pickup_latch_error or "no_latched_geometry_target"
                return None, "legacy_fallback", reason
            if phase == MissionPhase.MAGNET_ATTACH_WAIT:
                return (
                    self.latched_geometry_pickup_contact_target.copy(),
                    "latched_contact",
                    "",
                )
            if self.latched_pickup_position is None:
                return None, "legacy_fallback", "missing_latched_pickup_pose"
            if phase == MissionPhase.LIFT_OBJECT:
                profile = self.rate_profile(MissionPhase.LIFT_OBJECT)
                return (
                    self.latched_pickup_position
                    + profile.tip_offset_at(self.phase_elapsed()),
                    "latched_lift",
                    "",
                )
            profile = RateProfile(
                self.latched_geometry_pickup_approach_target
                - self.latched_pickup_position,
                self.latched_geometry_pickup_contact_target
                - self.latched_pickup_position,
                self.pickup_descent_speed,
            )
            return (
                self.latched_pickup_position
                + profile.tip_offset_at(self.phase_elapsed()),
                "latched_descent",
                "",
            )

        geometry, _, _, source, error = self.compute_pickup_target_geometry(
            allow_latched=False
        )
        if geometry is None:
            return None, "legacy_fallback", error or source
        return (
            geometry.approach_marker_target_world(
                self.geometry_pickup_approach_height()
            ),
            f"live_approach:{source}",
            "",
        )

    def geometry_aware_pickup_tip_offset(
        self,
        phase: Optional[MissionPhase] = None,
    ) -> Optional[np.ndarray]:
        phase = self.phase if phase is None else phase
        target_world, _, _ = self.geometry_aware_pickup_target_for_phase(phase)
        if target_world is None:
            return None
        target_state = self.pickup_target_state()
        return target_world - target_state.position

    def update_pickup_geometry_diagnostics(self) -> None:
        """Refresh passive geometry and active V5B2 status without side effects."""

        self.last_pickup_target_geometry = None
        self.last_fixed_clearance_pickup_target = None
        self.last_geometry_pickup_target_delta = None
        self.last_geometry_pickup_approach_target = None
        self.pickup_geometry_diagnostics_available = False
        self.pickup_geometry_diagnostics_error = ""

        (
            target_geometry,
            pose_position,
            pose_age,
            source,
            error,
        ) = self.compute_pickup_target_geometry(allow_latched=True)
        self.pickup_geometry_pose_age_s = pose_age
        self.pickup_geometry_diagnostics_source = source
        self.pickup_geometry_diagnostics_error = error

        if target_geometry is not None and pose_position is not None:
            fixed_target = pose_position + np.array(
                [0.0, 0.0, self.pickup_attach_clearance],
                dtype=float,
            )
            target_delta = target_geometry.magnet_marker_target_world - fixed_target
            self.last_pickup_target_geometry = target_geometry
            self.last_fixed_clearance_pickup_target = fixed_target
            self.last_geometry_pickup_target_delta = target_delta
            self.last_geometry_pickup_approach_target = (
                target_geometry.approach_marker_target_world(
                    self.geometry_pickup_approach_height()
                )
            )
            self.pickup_geometry_diagnostics_available = True

        active_target, mode, fallback = self.geometry_aware_pickup_target_for_phase()
        self.last_active_geometry_pickup_target = (
            None if active_target is None else active_target.copy()
        )
        self.geometry_aware_pickup_mode = mode
        self.geometry_aware_pickup_fallback_reason = fallback
        self.geometry_aware_pickup_active = active_target is not None

    def pickup_geometry_markers_active(self) -> bool:
        return self.phase in {
            MissionPhase.APPROACH_ABOVE_PICKUP,
            MissionPhase.SETTLE_ABOVE_PICKUP,
            MissionPhase.DESCEND_TO_PICKUP,
            MissionPhase.MAGNET_ATTACH_WAIT,
            MissionPhase.LIFT_OBJECT,
        }

    def pickup_target_state(self) -> TargetState:
        if self.pickup_object_position is None:
            raise RuntimeError("Pickup target requested before pickup pose is available.")
        if self.latched_pickup_position is not None and self.phase not in {
            MissionPhase.WAIT_FOR_TAKEOFF,
            MissionPhase.APPROACH_ABOVE_PICKUP,
            MissionPhase.SETTLE_ABOVE_PICKUP,
        }:
            position = self.latched_pickup_position.copy()
            velocity = np.zeros(3, dtype=float)
        else:
            position = self.pickup_object_position.copy()
            velocity = (
                np.zeros(3, dtype=float)
                if self.pickup_object_velocity is None
                else self.pickup_object_velocity.copy()
            )
        return TargetState(position, velocity, np.zeros(3, dtype=float))

    def drop_target_state(self) -> TargetState:
        if self.drop_point_source in {"static", "fixed", "hardcoded"}:
            return TargetState(
                self.static_drop_point + self.drop_offset,
                np.zeros(3, dtype=float),
                np.zeros(3, dtype=float),
            )
        if self.drop_point_source == "payload":
            if self.payload_position is None:
                raise RuntimeError("Payload drop target is unavailable.")
            rotated = _rotate_yaw(
                np.array([self.drop_offset[0], self.drop_offset[1], 0.0]),
                self.payload_yaw,
            )
            rotated[2] = self.drop_offset[2]
            velocity = (
                np.zeros(3, dtype=float)
                if self.payload_velocity is None
                else self.payload_velocity.copy()
            )
            return TargetState(
                self.payload_position + rotated,
                velocity,
                np.zeros(3, dtype=float),
            )
        if self.attachment_position is None:
            raise RuntimeError("Attachment-backed drop target is unavailable.")
        velocity = (
            np.zeros(3, dtype=float)
            if self.attachment_velocity is None
            else self.attachment_velocity.copy()
        )
        return TargetState(
            self.attachment_position + self.drop_offset,
            velocity,
            np.zeros(3, dtype=float),
        )

    def attachment_target_state(self) -> TargetState:
        if self.attachment_position is None or self.attachment_velocity is None:
            raise RuntimeError("Attachment target requested before pose/twist is available.")
        return TargetState(
            self.attachment_position,
            self.attachment_velocity,
            np.zeros(3, dtype=float),
        )

    def target_state(self, source: TargetSource) -> TargetState:
        if source == TargetSource.PICKUP_OBJECT:
            return self.pickup_target_state()
        if source == TargetSource.DROP_POINT:
            return self.drop_target_state()
        if source == TargetSource.ATTACHMENT_POINT:
            return self.attachment_target_state()
        if source in {TargetSource.CURRENT_POSE, TargetSource.LANDING_POINT}:
            if self.drone_position is None:
                raise RuntimeError("Current-pose target requested before drone state.")
            velocity = (
                np.zeros(3, dtype=float)
                if self.drone_velocity is None
                else self.drone_velocity.copy()
            )
            return TargetState(
                self.drone_position,
                velocity,
                np.zeros(3, dtype=float),
            )
        raise RuntimeError(f"Unsupported target source: {source.value}")

    def nominal_quad_from_tip_offset(self) -> np.ndarray:
        return np.array([0.0, 0.0, self.magnet_drop_below_quad], dtype=float)

    def pickup_compensation_input_active(self) -> bool:
        return bool(
            self.enable_pickup_tip_xy_compensation
            and self.use_measured_magnet_tip
            and self.drone_position is not None
            and self.measured_magnet_tip_position is not None
            and self.phase
            in {
                MissionPhase.APPROACH_ABOVE_PICKUP,
                MissionPhase.SETTLE_ABOVE_PICKUP,
                MissionPhase.DESCEND_TO_PICKUP,
                MissionPhase.MAGNET_ATTACH_WAIT,
            }
        )

    def pickup_compensation_applies(self) -> bool:
        return bool(
            self.enable_pickup_tip_xy_compensation
            and self.use_measured_magnet_tip
            and self.phase
            in {
                MissionPhase.APPROACH_ABOVE_PICKUP,
                MissionPhase.SETTLE_ABOVE_PICKUP,
                MissionPhase.DESCEND_TO_PICKUP,
                MissionPhase.MAGNET_ATTACH_WAIT,
                MissionPhase.LIFT_OBJECT,
            }
        )

    def update_pickup_tip_xy_compensation(self) -> None:
        target = np.zeros(2, dtype=float)
        raw = np.zeros(2, dtype=float)
        active = self.pickup_compensation_input_active()
        if active:
            assert self.drone_position is not None
            assert self.measured_magnet_tip_position is not None
            raw = self.drone_position[:2] - self.measured_magnet_tip_position[:2]
            target = self.pickup_tip_xy_compensation_gain * raw
            norm = float(np.linalg.norm(target))
            if self.pickup_tip_xy_compensation_max_m <= 0.0:
                target[:] = 0.0
            elif norm > self.pickup_tip_xy_compensation_max_m:
                target *= self.pickup_tip_xy_compensation_max_m / norm

        tau = self.pickup_tip_xy_compensation_time_constant_s
        alpha = 1.0 if tau <= 0.0 else 1.0 - math.exp(-self.dt / tau)
        self.pickup_tip_xy_compensation_filtered += alpha * (
            target - self.pickup_tip_xy_compensation_filtered
        )
        if float(np.linalg.norm(self.pickup_tip_xy_compensation_filtered)) < 1e-6:
            self.pickup_tip_xy_compensation_filtered[:] = 0.0
        self.pickup_tip_xy_compensation_raw = raw
        self.pickup_tip_xy_compensation_measurement_active = active

    def quad_reference_offset_from_tip(self) -> np.ndarray:
        offset = self.nominal_quad_from_tip_offset()
        if self.pickup_compensation_applies():
            offset[:2] += self.pickup_tip_xy_compensation_filtered
        return offset

    def rate_profile(self, phase: Optional[MissionPhase] = None) -> RateProfile:
        phase = self.phase if phase is None else phase
        if phase in {MissionPhase.TAKEOFF, MissionPhase.M2_VERTICAL_TAKEOFF}:
            return RateProfile(
                np.array([0.0, 0.0, 0.0]),
                np.array([0.0, 0.0, self.takeoff_height]),
                self.takeoff_speed,
            )
        if phase == MissionPhase.DESCEND_TO_PICKUP:
            if (
                self.use_geometry_aware_pickup
                and self.latched_pickup_position is not None
                and self.latched_geometry_pickup_approach_target is not None
                and self.latched_geometry_pickup_contact_target is not None
            ):
                return RateProfile(
                    self.latched_geometry_pickup_approach_target
                    - self.latched_pickup_position,
                    self.latched_geometry_pickup_contact_target
                    - self.latched_pickup_position,
                    self.pickup_descent_speed,
                )
            return RateProfile(
                np.array([0.0, 0.0, self.pickup_approach_clearance]),
                np.array([0.0, 0.0, self.pickup_attach_clearance]),
                self.pickup_descent_speed,
            )
        if phase == MissionPhase.LIFT_OBJECT:
            if (
                self.use_geometry_aware_pickup
                and self.latched_geometry_pickup_contact_offset is not None
            ):
                lift_delta = max(
                    0.0,
                    float(self.pickup_lift_height - self.pickup_attach_clearance),
                )
                return RateProfile(
                    self.latched_geometry_pickup_contact_offset,
                    self.latched_geometry_pickup_contact_offset
                    + np.array([0.0, 0.0, lift_delta], dtype=float),
                    self.pickup_lift_speed,
                )
            return RateProfile(
                np.array([0.0, 0.0, self.pickup_attach_clearance]),
                np.array([0.0, 0.0, self.pickup_lift_height]),
                self.pickup_lift_speed,
            )
        if phase == MissionPhase.DESCEND_TO_DROP_HEIGHT:
            return RateProfile(
                np.array([0.0, 0.0, self.drop_approach_clearance]),
                np.array([0.0, 0.0, self.drop_release_clearance]),
                self.drop_descent_speed,
            )
        if phase == MissionPhase.CLEAR_DROP_ZONE:
            return RateProfile(
                np.array([0.0, 0.0, self.drop_release_clearance]),
                np.array([0.0, 0.0, self.drop_clear_height]),
                self.drop_clear_speed,
            )
        if phase == MissionPhase.DESCEND_TO_ATTACHMENT:
            return RateProfile(
                np.array(
                    [
                        self.approach_offset_x,
                        self.approach_offset_y,
                        self.approach_offset_z,
                    ]
                ),
                np.array(
                    [
                        self.approach_offset_x,
                        self.approach_offset_y,
                        self.final_attach_offset_z,
                    ]
                ),
                self.descend_rate,
            )
        raise RuntimeError(f"Phase {phase.value} has no relative rate profile.")

    def desired_tip_offset(self) -> np.ndarray:
        if self.phase in {
            MissionPhase.WAIT_FOR_TAKEOFF,
            MissionPhase.APPROACH_ABOVE_PICKUP,
            MissionPhase.SETTLE_ABOVE_PICKUP,
        }:
            geometry_offset = self.geometry_aware_pickup_tip_offset(self.phase)
            if geometry_offset is not None:
                return geometry_offset
            return np.array([0.0, 0.0, self.pickup_approach_clearance])
        if self.phase == MissionPhase.MAGNET_ATTACH_WAIT:
            geometry_offset = self.geometry_aware_pickup_tip_offset(self.phase)
            if geometry_offset is not None:
                return geometry_offset
            return np.array([0.0, 0.0, self.pickup_attach_clearance])
        if self.phase in {
            MissionPhase.DESCEND_TO_PICKUP,
            MissionPhase.LIFT_OBJECT,
            MissionPhase.DESCEND_TO_DROP_HEIGHT,
            MissionPhase.CLEAR_DROP_ZONE,
            MissionPhase.DESCEND_TO_ATTACHMENT,
        }:
            return self.rate_profile().tip_offset_at(self.phase_elapsed())
        if self.phase in {
            MissionPhase.TRANSIT_TO_DROP_POINT,
            MissionPhase.SETTLE_ABOVE_DROP_POINT,
        }:
            return np.array([0.0, 0.0, self.drop_approach_clearance])
        if self.phase == MissionPhase.DROP_OBJECT:
            return np.array([0.0, 0.0, self.drop_release_clearance])
        if self.phase in {
            MissionPhase.TRANSIT_TO_REATTACH,
            MissionPhase.APPROACH_ABOVE_TARGET,
            MissionPhase.MATCH_VELOCITY,
        }:
            return np.array(
                [
                    self.approach_offset_x,
                    self.approach_offset_y,
                    self.approach_offset_z,
                ]
            )
        if self.phase == MissionPhase.ATTACH_READY:
            return np.array(
                [
                    self.approach_offset_x,
                    self.approach_offset_y,
                    self.final_attach_offset_z,
                ]
            )
        return np.zeros(3, dtype=float)

    def current_magnet_tip_position(self) -> np.ndarray:
        if self.drone_position is None:
            raise RuntimeError("Magnet tip requested before drone state.")
        if self.use_measured_magnet_tip and self.measured_magnet_tip_position is not None:
            return self.measured_magnet_tip_position.copy()
        return self.drone_position - self.nominal_quad_from_tip_offset()

    def tracking_errors(self) -> Tuple[float, float, float]:
        spec = self.current_phase_spec()
        target = self.target_state(spec.target_source)
        tip_target = target.position + self.desired_tip_offset()
        tip_now = self.current_magnet_tip_position()
        drone_velocity = (
            np.zeros(3, dtype=float)
            if self.drone_velocity is None
            else self.drone_velocity
        )
        xy_error = float(np.linalg.norm(tip_now[:2] - tip_target[:2]))
        relative_velocity = float(
            np.linalg.norm(drone_velocity[:2] - target.velocity[:2])
        )
        z_error = float(tip_now[2] - tip_target[2])
        return xy_error, relative_velocity, z_error

    def loaded_lift_quad_z_error_m(self) -> float:
        """Signed quad-Z error from the completed LIFT_OBJECT body endpoint."""
        if self.drone_position is None or self.latched_pickup_position is None:
            return float("nan")
        profile = self.rate_profile(MissionPhase.LIFT_OBJECT)
        quad_offset = self.quad_reference_offset_from_tip()
        target_quad_z = float(
            self.latched_pickup_position[2]
            + profile.goal_tip_offset[2]
            + quad_offset[2]
        )
        return float(self.drone_position[2] - target_quad_z)

    # ----------------------------------------------------------------------
    # Mission transition guards
    # ----------------------------------------------------------------------
    def update_state_machine(self) -> None:
        if self.phase in {MissionPhase.LANDING, MissionPhase.LANDED_DISARMED}:
            return

        if self.is_m2b:
            if self.phase in {
                MissionPhase.M2_BOOTSTRAP_DETACH,
                MissionPhase.M2_BOOTSTRAP_SETTLE,
                MissionPhase.M2_WAIT_FOR_ARM,
                MissionPhase.M2_FAULT,
            }:
                return
            if self.phase == MissionPhase.M2_VERTICAL_TAKEOFF:
                if self.takeoff_start_position is None:
                    return
                profile = self.rate_profile()
                target_z = self.takeoff_start_position[2] + self.takeoff_height
                reached = bool(
                    profile.complete(self.phase_elapsed())
                    and self.drone_position is not None
                    and self.drone_velocity is not None
                    and abs(self.drone_position[2] - target_z) < self.pickup_z_tolerance
                    and abs(self.drone_velocity[2]) < 0.15
                )
                if self.condition_true_for(reached, 0.50):
                    self.transition_to(
                        MissionPhase.M2_TAKEOFF_HOVER,
                        "M2B B0 takeoff height reached and vertically settled",
                    )
                return
            if self.phase == MissionPhase.M2_TAKEOFF_HOVER:
                if self.m2b_commissioning_stage == "b1":
                    self.update_m2b_b1()
                elif self.m2b_commissioning_stage == "b2":
                    self.update_m2b_b2()
                return
            if self.m2b_commissioning_stage == "b2" and self.phase == MissionPhase.M2_CPP_TRANSIT_CAPTURE:
                self.update_m2b_b2()
                return
            if self.m2b_commissioning_stage in {"b1", "b2"} and self.phase in {
                MissionPhase.M2_SETTLE_CAPTURE,
                MissionPhase.M2_ATTACH_APPROACH_HIGH,
                MissionPhase.M2_ATTACH_APPROACH_LOW,
                MissionPhase.M2_ATTACH_CAPTURE_WAIT,
                MissionPhase.M2_ATTACH_PROOF,
                MissionPhase.M2_ATTACHED_HOLD,
                MissionPhase.M2_DETACHED_RETREAT,
                MissionPhase.M2_LANDING_STAGE,
            }:
                self.update_m2b_b1()
                return

        xy_error, xy_relative_velocity, z_error = self.tracking_errors()
        abs_z_error = abs(z_error)
        tip_speed = float(
            np.linalg.norm(
                np.zeros(3, dtype=float)
                if self.drone_velocity is None
                else self.drone_velocity
            )
        )
        elapsed = self.phase_elapsed()

        if self.phase == MissionPhase.WAIT_FOR_TAKEOFF:
            return

        if self.phase == MissionPhase.TAKEOFF:
            profile = self.rate_profile()

            target_z = (
                self.takeoff_start_position[2] + self.takeoff_height
            )

            reached = (
                profile.complete(elapsed)
                and abs(self.drone_position[2] - target_z) < self.pickup_z_tolerance
                and abs(self.drone_velocity[2]) < 0.15
            )

            if reached:
                self.transition_to(
                    MissionPhase.APPROACH_ABOVE_PICKUP,
                    "Takeoff height reached"
                )
            return

        if self.phase == MissionPhase.APPROACH_ABOVE_PICKUP:
            if (
                self.enable_pickup_timeout_landing
                and self.pickup_land_after_s > 0.0
                and self.takeoff_detected_time is not None
                and elapsed >= self.pickup_land_after_s
            ):
                self.transition_to(
                    MissionPhase.LANDING,
                    f"pickup approach timeout after {elapsed:.1f} s",
                )
                return
            good = (
                xy_error < self.pickup_xy_tolerance
                and abs_z_error < self.pickup_z_tolerance
                and tip_speed < self.pickup_tip_speed_tolerance
            )
            if self.condition_true_for(good, 0.50):
                self.transition_to(
                    MissionPhase.SETTLE_ABOVE_PICKUP,
                    "arrived above pickup object",
                )
            return

        if self.phase == MissionPhase.SETTLE_ABOVE_PICKUP:
            good = (
                xy_error < self.pickup_xy_tolerance
                and abs_z_error < self.pickup_z_tolerance
                and tip_speed < self.pickup_tip_speed_tolerance
            )
            if self.condition_true_for(good, self.pickup_settle_time_s):
                self.transition_to(
                    MissionPhase.DESCEND_TO_PICKUP,
                    "settled above pickup object",
                )
            return

        if self.phase == MissionPhase.DESCEND_TO_PICKUP:
            if self.rate_profile().complete(elapsed):
                self.transition_to(
                    MissionPhase.MAGNET_ATTACH_WAIT,
                    "pickup descent profile complete",
                )
            return

        if self.phase == MissionPhase.MAGNET_ATTACH_WAIT:
            aligned = self.object_attached and xy_error < self.pickup_lift_xy_tolerance
            if self.condition_true_for(aligned, self.pickup_lift_xy_dwell_s):
                self.transition_to(
                    MissionPhase.LIFT_OBJECT,
                    f"object attached with xy error {xy_error:.3f} m",
                )
            elif (
                self.attach_wait_start_time is not None
                and self.attach_wait_timeout_s > 0.0
                and self._physical_now_s() - self.attach_wait_start_time
                > self.attach_wait_timeout_s
            ):
                self.get_logger().warning(
                    "Still waiting for magnetic attachment confirmation."
                )
            return

        if self.phase == MissionPhase.LIFT_OBJECT:
            if not self.object_attached:
                self.reset_c1f2_for_pickup_retry()
                self.transition_to(
                    MissionPhase.APPROACH_ABOVE_PICKUP,
                    "attachment lost during loaded lift; retrying pickup",
                )
                return

            profile_complete = self.rate_profile().complete(elapsed)
            object_clearance = float("-inf")
            if (
                self.latched_pickup_position is not None
                and self.pickup_object_position is not None
            ):
                object_clearance = float(
                    self.pickup_object_position[2] - self.latched_pickup_position[2]
                )
            object_airborne = (
                self.object_attached
                and object_clearance >= self.lift_complete_object_clearance
            )
            tracked_lift = (
                profile_complete
                and abs_z_error < self.loaded_lift_z_tolerance
            )

            # IRL stage-5 commissioning: keep commanding the completed loaded lift
            # endpoint without preparing or granting C++ authority. Magnet remains ON.
            if (
                self.commissioning_hold_after_lift
                and self.object_attached
                and tracked_lift
            ):
                self.condition_start_time = None
                return

            if self.c1f2_cpp_authority_enabled:
                # The strict loaded-lift gate is an ENTRY certificate only. Once
                # the measured physical lift-end hold has been latched, the
                # existing stationary p/v/a settle contract owns continuation.
                # Requiring the old ideal-Z gate again would contradict the
                # measured-hold reference and can deadlock the 0.30 s settle dwell.
                if not self.c1f2_prepare_active:
                    loaded_lift_quad_z_error_m = self.loaded_lift_quad_z_error_m()
                    loaded_lift_complete = loaded_lift_ready_for_cpp(
                        object_attached=self.object_attached,
                        object_airborne=object_airborne,
                        profile_complete=profile_complete,
                        abs_quad_z_error_m=abs(loaded_lift_quad_z_error_m),
                        z_tolerance_m=self.loaded_lift_z_tolerance,
                    )

                    override_accepted = False
                    if self.operator_advance_override_pending:
                        self.operator_advance_override_pending = False
                        if loaded_lift_complete:
                            self.operator_override_last_result = "REJECTED_NOT_NEEDED"
                            self.get_logger().warning(
                                "Operator ADVANCE override not used: normal loaded-lift "
                                "entry gate already passes."
                            )
                        elif loaded_lift_override_allowed(
                            object_attached=self.object_attached,
                            object_airborne=object_airborne,
                            profile_complete=profile_complete,
                        ):
                            override_accepted = True
                            self.operator_override_used = True
                            self.operator_override_accept_count += 1
                            self.operator_override_last_result = "ACCEPTED"
                            self.get_logger().warning(
                                "OPERATOR ADVANCE OVERRIDE ACCEPTED: "
                                "phase=LIFT_OBJECT "
                                f"attachment={self.object_attached} "
                                f"object_clearance={object_clearance:.3f} m "
                                f"profile_complete={profile_complete} "
                                f"body_z_error={loaded_lift_quad_z_error_m:.3f} m "
                                f"normal_tolerance={self.loaded_lift_z_tolerance:.3f} m "
                                "action=WAIVE_LOADED_LIFT_Z_GATE_AND_BEGIN_NORMAL_C1F2_PREPARE"
                            )
                        else:
                            self.operator_override_last_result = "REJECTED_PREREQUISITES"
                            self.get_logger().warning(
                                "Operator ADVANCE override rejected: loaded-lift physical "
                                "prerequisites are not all satisfied "
                                f"(attachment={self.object_attached}, "
                                f"object_airborne={object_airborne}, "
                                f"profile_complete={profile_complete})."
                            )

                    if not loaded_lift_complete and not override_accepted:
                        return
                    if self.drone_position is None or self.drone_velocity is None:
                        return
                    self.c1f2_prepare_active = True
                    self.c1f2_prepare_settled = False
                    self.c1f2_prepare_lift_end_position = self.drone_position.copy()
                    self.c1f2_prepare_hold_position = self.drone_position.copy()
                    self.c1f2_prepare_settle_start_time = None
                    self.c1f2_prepare_drift_m = 0.0
                    self.c1f2_prepare_speed_mps = float(
                        np.linalg.norm(self.drone_velocity)
                    )
                    self.c1f2_handoff_ready = False
                    self.c1f2_authority_grant_pending = False
                    self.c1f2_authority_acknowledged = False
                    self.c1f2_authority_grant_time = None
                    self.c1f2_authority_grant_ros_time_s = None
                    self.c1f2_authority_ack_time = None
                    self.c1f2_publish_authority(False)
                    self.c1f2_prepare_start_time = self._physical_now_s()
                    self.c1f2_prepare_count += 1
                    reason = (
                        "operator override accepted; "
                        if override_accepted
                        else "loaded lift complete; "
                    )
                    self.get_logger().info(
                        "C1F.2b " + reason + "latched measured lift-end position "
                        "and holding there until the vehicle settles."
                    )
                    return
                if self.c1f2_prepare_start_time is not None:
                    self.c1f2_prepare_wait_s = max(
                        0.0, self._physical_now_s() - self.c1f2_prepare_start_time
                    )
                if not self.c1f2_prepare_settled:
                    if (
                        self.drone_position is None
                        or self.drone_velocity is None
                        or self.drone_acceleration is None
                        or self.c1f2_prepare_lift_end_position is None
                    ):
                        self.c1f2_prepare_settle_start_time = None
                        return
                    grant_state = stationary_grant_state(
                        lift_end_position=self.c1f2_prepare_lift_end_position,
                        measured_position=self.drone_position,
                        measured_velocity=self.drone_velocity,
                        measured_acceleration=self.drone_acceleration,
                        max_drift_m=self.c1f2_prepare_max_lift_drift_m,
                        settle_speed_mps=self.c1f2_prepare_settle_speed_mps,
                        settle_acceleration_mps2=self.c1f2_handoff_acceleration_tolerance_mps2,
                    )
                    self.c1f2_prepare_drift_m = grant_state.drift_m
                    self.c1f2_prepare_speed_mps = grant_state.speed_mps
                    if not grant_state.ready:
                        self.c1f2_prepare_settle_start_time = None
                        return

                    now_mono = self._physical_now_s()
                    if self.c1f2_prepare_settle_start_time is None:
                        self.c1f2_prepare_settle_start_time = now_mono
                        if self.c1f2_prepare_settle_dwell_s > 0.0:
                            return
                    if (
                        now_mono - self.c1f2_prepare_settle_start_time
                        < self.c1f2_prepare_settle_dwell_s
                    ):
                        return

                    # Re-latch the actual settled vehicle position once.  This is
                    # the physical handoff state both Python and the first C++
                    # candidate should start from.  The original lift-end position
                    # remains only the loose drift/sanity anchor.
                    self.c1f2_prepare_hold_position = self.drone_position.copy()
                    self.c1f2_prepare_settled = True
                    self.get_logger().info(
                        "C1F.8c2 stationary handoff state settled: "
                        f"drift_from_lift_end={self.c1f2_prepare_drift_m:.3f} m, "
                        f"speed={self.c1f2_prepare_speed_mps:.3f} m/s, "
                        f"accel={grant_state.acceleration_mps2:.3f} m/s^2. "
                        "Re-latched measured hold position; enabling C++ preparation."
                    )
                    return
                if self.c1f2_handoff_ready:
                    self.transition_to(
                        MissionPhase.TRANSIT_TO_DROP_POINT,
                        "C1F.2b C++ transfer prepared from stationary loaded hold",
                    )
                return

            # Legacy/Python-only behaviour remains unchanged when C1F.2 is disabled.
            if object_airborne or tracked_lift:
                reason = (
                    f"object lifted {object_clearance:.2f} m"
                    if object_airborne
                    else "loaded lift profile tracked"
                )
                self.transition_to(MissionPhase.TRANSIT_TO_DROP_POINT, reason)
            return

        if self.phase == MissionPhase.TRANSIT_TO_DROP_POINT:
            if self.c1f2_cpp_authority_enabled:
                spatial_capture = bool(
                    xy_error < self.c1f2_nav_capture_xy_threshold_m
                    and abs_z_error < self.c1f2_nav_capture_z_threshold_m
                )
                bridge_feasible = False
                if spatial_capture:
                    _, bridge_feasibility = self._evaluate_c1f2_target_relative_exit_bridge()
                    bridge_feasible = bool(
                        bridge_feasibility is not None and bridge_feasibility.feasible
                    )
                else:
                    # C1F.8: the expensive 1.5 s quintic feasibility sweep cannot
                    # affect the transition while the spatial capture gate is false.
                    # Clear any stale pending bridge certificate and defer the exact
                    # same dynamic check until it can change the decision.
                    self.c1f2_exit_bridge_feasible = False
                    self.c1f2_exit_bridge_feasibility_reason = "OUTSIDE_NAV_CAPTURE_REGION"
                    self.c1f2_exit_bridge_pending_state = None
                    self.c1f2_exit_bridge_pending_feasibility = None
                good = spatial_capture and bridge_feasible
                transition_reason = (
                    "moving rendezvous entered dynamically-capturable bridge region"
                )
            else:
                # Preserve the pre-C1F.6 Python-only mission contract. The new
                # bridge-feasibility gate is specifically the C++ -> Python
                # moving-rendezvous authority handoff, not a global replacement
                # for unrelated legacy phase logic.
                good = (
                    xy_error < self.match_xy_threshold
                    and abs_z_error < self.pickup_z_tolerance
                    and xy_relative_velocity < self.match_xy_velocity_threshold
                )
                transition_reason = "drop point reached"
            if self.condition_true_for(good, 0.50):
                self.transition_to(
                    MissionPhase.SETTLE_ABOVE_DROP_POINT,
                    transition_reason,
                )
            return

        if self.phase == MissionPhase.SETTLE_ABOVE_DROP_POINT:
            # C1F.2b hands authority to a Python moving-frame bridge immediately
            # on phase entry. Do not begin the normal settle dwell until that
            # bridge has completed and ordinary target-relative tracking is active.
            if self.c1f2_cpp_authority_active or self.c1f2_exit_bridge_active:
                self.condition_start_time = None
                return
            # IRL commissioning: after C++ has exited cleanly and Python is holding
            # the fixed target-relative staging point, stop here. This prevents the
            # automatic DESCEND_TO_DROP_HEIGHT -> DROP_OBJECT magnet-release sequence.
            if self.commissioning_hold_before_drop:
                self.condition_start_time = None
                return
            good = (
                xy_error < self.match_xy_threshold
                and abs_z_error < self.pickup_z_tolerance
                and xy_relative_velocity < self.match_xy_velocity_threshold
            )
            if self.condition_true_for(good, self.drop_settle_time_s):
                self.transition_to(
                    MissionPhase.DESCEND_TO_DROP_HEIGHT,
                    "drop-point-relative hover settled",
                )
            return

        if self.phase == MissionPhase.DESCEND_TO_DROP_HEIGHT:
            if self.rate_profile().complete(elapsed):
                self.transition_to(
                    MissionPhase.DROP_OBJECT,
                    "drop descent profile complete",
                )
            return

        if self.phase == MissionPhase.DROP_OBJECT:
            waited = (
                self.drop_start_time is not None
                and self._physical_now_s() - self.drop_start_time >= self.drop_wait_time_s
            )
            if not self.object_attached or waited:
                self.transition_to(
                    MissionPhase.CLEAR_DROP_ZONE,
                    "object released or release dwell completed",
                )
            return

        if self.phase == MissionPhase.CLEAR_DROP_ZONE:
            if self.rate_profile().complete(elapsed) and abs_z_error < self.pickup_z_tolerance:
                self.transition_to(
                    MissionPhase.TRANSIT_TO_REATTACH,
                    "drop zone cleared",
                )
            return

        if self.phase == MissionPhase.TRANSIT_TO_REATTACH:
            if self.condition_true_for(
                xy_error < self.reattach_transit_xy_threshold,
                0.20,
            ):
                self.transition_to(
                    MissionPhase.APPROACH_ABOVE_TARGET,
                    "entered attachment approach region",
                )
            return

        if self.phase == MissionPhase.APPROACH_ABOVE_TARGET:
            if self.condition_true_for(
                xy_error < self.approach_xy_threshold,
                self.dwell_time_s,
            ):
                self.transition_to(
                    MissionPhase.MATCH_VELOCITY,
                    "attachment XY approach complete",
                )
            return

        if self.phase == MissionPhase.MATCH_VELOCITY:
            matched = (
                xy_error < self.match_xy_threshold
                and xy_relative_velocity < self.match_xy_velocity_threshold
            )
            if self.condition_true_for(matched, self.dwell_time_s):
                if self.auto_descend:
                    self.transition_to(
                        MissionPhase.DESCEND_TO_ATTACHMENT,
                        "attachment position and velocity matched",
                    )
                else:
                    self.condition_start_time = self._physical_now_s()
            return

        if self.phase == MissionPhase.DESCEND_TO_ATTACHMENT:
            if (
                xy_error > self.descend_abort_xy_threshold
                or xy_relative_velocity > self.descend_abort_xy_velocity_threshold
            ):
                self.transition_to(
                    MissionPhase.APPROACH_ABOVE_TARGET,
                    "attachment descent aborted after loss of relative alignment",
                )
            elif self.rate_profile().complete(elapsed):
                self.transition_to(
                    MissionPhase.ATTACH_READY,
                    "terminal attachment profile complete",
                )
            return

        if self.phase == MissionPhase.ATTACH_READY:
            if (
                self.enable_attach_ready_timeout_landing
                and self.attach_ready_land_after_s > 0.0
                and elapsed >= self.attach_ready_land_after_s
            ):
                self.transition_to(
                    MissionPhase.LANDING,
                    f"ATTACH_READY timeout after {elapsed:.1f} s",
                )
            elif (
                xy_error > self.descend_abort_xy_threshold
                or xy_relative_velocity > self.descend_abort_xy_velocity_threshold
            ):
                self.transition_to(
                    MissionPhase.APPROACH_ABOVE_TARGET,
                    "terminal attachment alignment lost",
                )

    # ----------------------------------------------------------------------
    # Reference construction
    # ----------------------------------------------------------------------
    def build_m2b_b1_reference(self) -> TrajectoryReference:
        if self.drone_position is None or self.drone_velocity is None:
            raise RuntimeError("M2B B1 reference requested before drone state")

        if self.phase == MissionPhase.M2_SETTLE_CAPTURE:
            target_position = self.m2b_capture_body_target()
            return self.transfer_generator.generate(
                phase_key=self.phase.value,
                measured_position=self.drone_position,
                measured_velocity=self.drone_velocity,
                target=TargetState(target_position, np.zeros(3), np.zeros(3)),
                advance=True,
            )

        if self.phase in {MissionPhase.M2_ATTACH_APPROACH_HIGH, MissionPhase.M2_ATTACH_APPROACH_LOW}:
            height = (
                self.m2b_config.capture_height_m
                if self.m2b_descent_progress is None
                else self.m2b_descent_progress.target_contact_height_m
            )
            return stationary_regulation(
                self.m2b_body_target_for_contact_height(height),
                horizon_samples=self.horizon_samples,
                min_reference_z=self.min_reference_z,
            )

        if self.phase == MissionPhase.M2_ATTACH_CAPTURE_WAIT:
            if self.m2b_latch_hold_position is not None:
                return stationary_regulation(
                    self.m2b_latch_hold_position,
                    horizon_samples=self.horizon_samples,
                    min_reference_z=self.min_reference_z,
                )
            hold_height = self.m2b_capture_wait_contact_height_m
            if hold_height is None:
                hold_height = (
                    self.m2b_config.contact_height_target_m
                    if self.m2b_descent_progress is None
                    else self.m2b_descent_progress.target_contact_height_m
                )
            return stationary_regulation(
                self.m2b_body_target_for_contact_height(hold_height),
                horizon_samples=self.horizon_samples,
                min_reference_z=self.min_reference_z,
            )

        if self.phase == MissionPhase.M2_ATTACH_PROOF:
            start = self.m2b_proof_start_position
            if start is None:
                start = self.drone_position.copy()
            _, plate_rotation = self.m2b_plate_pose_world()
            direction_world = plate_rotation @ proof_direction_plate(self.m2b_config)
            target = start + direction_world * proof_command_distance(
                self.m2b_config, elapsed_s=self.phase_elapsed()
            )
            return stationary_regulation(
                target,
                horizon_samples=self.horizon_samples,
                min_reference_z=self.min_reference_z,
            )

        if self.phase == MissionPhase.M2_ATTACHED_HOLD:
            target_position = self.m2b_attached_hold_body_target()
            return self.transfer_generator.generate(
                phase_key=self.phase.value,
                measured_position=self.drone_position,
                measured_velocity=self.drone_velocity,
                target=TargetState(target_position, np.zeros(3), np.zeros(3)),
                advance=True,
            )

        if self.phase == MissionPhase.M2_DETACHED_RETREAT:
            target_position = (
                self.drone_position.copy()
                if self.m2b_retreat_target is None
                else self.m2b_retreat_target.copy()
            )
            if self.m2b_retreat_stage == "wait_detach":
                return stationary_regulation(
                    target_position,
                    horizon_samples=self.horizon_samples,
                    min_reference_z=self.min_reference_z,
                )
            return self.transfer_generator.generate(
                phase_key=f"{self.phase.value}:{self.m2b_retreat_stage}",
                measured_position=self.drone_position,
                measured_velocity=self.drone_velocity,
                target=TargetState(target_position, np.zeros(3), np.zeros(3)),
                advance=True,
            )

        if self.phase == MissionPhase.M2_LANDING_STAGE:
            # Release first. While the physical attachment still exists, hold the
            # exact entry position rather than pulling horizontally against the ring.
            # Only after detached truth is fresh may the normal clear-to-landing-column
            # transfer begin.
            detached = self.m2b_geometry_separated(require_lost=True)
            if not detached:
                hold_position = (
                    self.drone_position.copy()
                    if self.m2b_landing_detach_hold_position is None
                    else self.m2b_landing_detach_hold_position.copy()
                )
                return stationary_regulation(
                    hold_position,
                    horizon_samples=self.horizon_samples,
                    min_reference_z=self.min_reference_z,
                )
            target_position = (
                self.drone_position.copy()
                if self.m2b_landing_stage_target is None
                else self.m2b_landing_stage_target.copy()
            )
            return self.transfer_generator.generate(
                phase_key=self.phase.value,
                measured_position=self.drone_position,
                measured_velocity=self.drone_velocity,
                target=TargetState(target_position, np.zeros(3), np.zeros(3)),
                advance=True,
            )

        if self.phase == MissionPhase.M2_FAULT:
            target = self.m2b_fault_hold_position
            if target is None:
                target = self.drone_position.copy()
            return stationary_regulation(
                target,
                horizon_samples=self.horizon_samples,
                min_reference_z=self.min_reference_z,
            )

        raise RuntimeError(f"No B1 reference policy for phase {self.phase.value}")

    def build_reference(self) -> TrajectoryReference:
        if self.drone_position is None or self.drone_velocity is None:
            raise RuntimeError("Reference requested before drone state is available.")

        spec = self.current_phase_spec()
        if self.phase == MissionPhase.LANDING:
            return self.build_landing_reference()
        if self.phase == MissionPhase.LANDED_DISARMED:
            target = (
                self.drone_position
                if self.landing_disarm_z is None
                else np.array(
                    [
                        self.landing_target_xy[0]
                        if self.landing_target_xy is not None
                        else self.drone_position[0],
                        self.landing_target_xy[1]
                        if self.landing_target_xy is not None
                        else self.drone_position[1],
                        self.landing_disarm_z,
                    ]
                )
            )
            return stationary_regulation(
                target,
                horizon_samples=self.horizon_samples,
                min_reference_z=self.min_reference_z,
            )

        if (
            self.is_m2b
            and self.m2b_commissioning_stage in {"b1", "b2"}
            and is_m2b_b1_local_reference_phase(self.phase)
        ):
            if self.m2b_commissioning_stage == "b2" and self.c1f2_exit_bridge_active:
                target, relative_offset = self.c1f2_exit_bridge_target_and_offset()
                bridge_reference = self._build_c1f2_target_relative_exit_bridge(
                    target, relative_offset
                )
                if bridge_reference is not None:
                    return bridge_reference
            return self.build_m2b_b1_reference()

        if (
            (
                self.phase == MissionPhase.LIFT_OBJECT
                or (
                    self.is_m2b
                    and self.m2b_commissioning_stage == "b2"
                    and self.phase == MissionPhase.M2_TAKEOFF_HOVER
                )
            )
            and self.c1f2_cpp_authority_enabled
            and self.c1f2_prepare_active
            and self.c1f2_prepare_hold_position is not None
        ):
            # C1F.2b ownership boundary: once the full loaded lift is complete,
            # stop advancing the ideal lift profile and hold the physical lift-end
            # neighbourhood.  After the low-speed dwell this point is re-latched
            # once from the settled measured state, which becomes the common
            # Python/C++ handoff origin.
            return stationary_regulation(
                self.c1f2_prepare_hold_position,
                horizon_samples=self.horizon_samples,
                min_reference_z=self.min_reference_z,
            )

        if self.phase in {MissionPhase.TAKEOFF, MissionPhase.M2_VERTICAL_TAKEOFF}:
            if self.takeoff_start_position is None:
                self.takeoff_start_position = self.drone_position.copy()

            profile = self.rate_profile()
            target = TargetState(
                self.takeoff_start_position,
                np.zeros(3),
                np.zeros(3),
            )

            reference, duration, complete = smooth_rate_controlled_manoeuvre(
                target,
                profile.start_tip_offset,
                profile.goal_tip_offset,
                rate=profile.rate,
                acceleration_limit=self.takeoff_reference_acceleration_limit_mps2,
                elapsed=self.phase_elapsed(),
                dt=self.dt,
                horizon_samples=self.horizon_samples,
                min_reference_z=self.min_reference_z,
                max_reference_speed=self.max_reference_speed,
            )

            self.last_rate_profile_duration = duration
            self.last_rate_profile_complete = complete
            return reference

        if self.phase == MissionPhase.M2_TAKEOFF_HOVER:
            if self.takeoff_start_position is None:
                raise RuntimeError("M2B takeoff hover requested without a takeoff origin")
            hover_target = self.takeoff_start_position.copy()
            hover_target[2] += self.takeoff_height
            return stationary_regulation(
                hover_target,
                horizon_samples=self.horizon_samples,
                min_reference_z=self.min_reference_z,
            )

        if spec.target_source == TargetSource.CURRENT_POSE:
            return stationary_regulation(
                self.drone_position,
                horizon_samples=self.horizon_samples,
                min_reference_z=self.min_reference_z,
            )

        target = self.target_state(spec.target_source)
        quad_offset = self.quad_reference_offset_from_tip()

        if spec.reference_type == ReferenceType.TRANSFER:
            body_target = self.transfer_body_target()
            return self.transfer_generator.generate(
                phase_key=self.phase.value,
                measured_position=self.drone_position,
                measured_velocity=self.drone_velocity,
                target=body_target,
                advance=True,
            )

        if spec.reference_type == ReferenceType.STATIONARY_REGULATION:
            body_target_position = (
                target.position + self.desired_tip_offset() + quad_offset
            )
            return stationary_regulation(
                body_target_position,
                horizon_samples=self.horizon_samples,
                min_reference_z=self.min_reference_z,
            )

        if spec.reference_type == ReferenceType.TARGET_RELATIVE_TRACKING:
            relative_offset = self.desired_tip_offset() + quad_offset
            if self.phase == MissionPhase.SETTLE_ABOVE_DROP_POINT:
                bridge_reference = self._build_c1f2_target_relative_exit_bridge(
                    target, relative_offset
                )
                if bridge_reference is not None:
                    return bridge_reference
            lead = (
                self.attach_ready_target_lead_time
                if self.phase == MissionPhase.ATTACH_READY
                else 0.0
            )
            return target_relative_tracking(
                target,
                relative_offset,
                dt=self.dt,
                horizon_samples=self.horizon_samples,
                lead_time=lead,
                min_reference_z=self.min_reference_z,
                max_reference_speed=self.max_reference_speed,
            )

        if spec.reference_type == ReferenceType.RATE_CONTROLLED_MANOEUVRE:
            profile = self.rate_profile()
            reference, duration, complete = rate_controlled_manoeuvre(
                target,
                profile.start_tip_offset + quad_offset,
                profile.goal_tip_offset + quad_offset,
                rate=profile.rate,
                elapsed=self.phase_elapsed(),
                dt=self.dt,
                horizon_samples=self.horizon_samples,
                min_reference_z=self.min_reference_z,
                max_reference_speed=self.max_reference_speed,
            )
            self.last_rate_profile_duration = duration
            self.last_rate_profile_complete = complete
            return reference

        raise RuntimeError(
            f"Unsupported reference type {spec.reference_type.value} "
            f"for phase {self.phase.value}."
        )

    # ----------------------------------------------------------------------
    # Landing reference and disarm
    # ----------------------------------------------------------------------
    def initialise_landing(self) -> None:
        if self.drone_position is None:
            return
        if self.initial_quad_z is None:
            self.initial_quad_z = float(self.drone_position[2])
        self.landing_target_xy = self.drone_position[:2].copy()
        self.landing_start_z = float(self.drone_position[2])
        self.landing_disarm_z = float(
            self.initial_quad_z + self.landing_disarm_height_above_start
        )
        self.landing_disarm_sent = False

    def build_landing_reference(self) -> TrajectoryReference:
        if self.drone_position is None:
            raise RuntimeError("Landing reference requested before drone state.")
        if (
            self.landing_target_xy is None
            or self.landing_start_z is None
            or self.landing_disarm_z is None
        ):
            self.initialise_landing()
        assert self.landing_target_xy is not None
        assert self.landing_start_z is not None
        assert self.landing_disarm_z is not None

        start = np.array(
            [self.landing_target_xy[0], self.landing_target_xy[1], self.landing_start_z]
        )
        goal = np.array(
            [self.landing_target_xy[0], self.landing_target_xy[1], self.landing_disarm_z]
        )
        target = TargetState(
            np.zeros(3, dtype=float),
            np.zeros(3, dtype=float),
            np.zeros(3, dtype=float),
        )
        reference, duration, complete = rate_controlled_manoeuvre(
            target,
            start,
            goal,
            rate=self.landing_descent_speed,
            elapsed=self.phase_elapsed(),
            dt=self.dt,
            horizon_samples=self.horizon_samples,
            min_reference_z=min(self.min_reference_z, self.landing_disarm_z),
            max_reference_speed=self.max_reference_speed,
        )
        self.last_rate_profile_duration = duration
        self.last_rate_profile_complete = complete
        return reference

    def maybe_publish_landing_disarm(self) -> None:
        if self.phase != MissionPhase.LANDING or self.landing_disarm_sent:
            return
        if self.drone_position is None or self.landing_disarm_z is None:
            return
        if self.phase_elapsed() < self.landing_min_time_before_disarm_s:
            return
        if float(self.drone_position[2]) <= self.landing_disarm_z:
            message = String()
            message.data = self.landing_disarm_command
            self.drone_command_publisher.publish(message)
            self.landing_disarm_sent = True
            self.transition_to(
                MissionPhase.LANDED_DISARMED,
                f"published {self.landing_disarm_command} at z={self.drone_position[2]:.2f} m",
            )

    # ----------------------------------------------------------------------
    # Passive visual-only A* scheduling
    # ----------------------------------------------------------------------
    def visual_astar_phase_allowed(self) -> bool:
        return self.current_phase_spec().reference_type == ReferenceType.TRANSFER

    def visual_trajectory_target_statistics(
        self,
    ) -> Tuple[
        Optional[np.ndarray],
        Optional[np.ndarray],
        Optional[np.ndarray],
        Optional[np.ndarray],
    ]:
        """Return a lightly filtered moving-target state for passive planning diagnostics.

        This deliberately simple estimate keeps the current full-route planner interface
        supplied with target position, velocity statistics, and measured acceleration.
        """
        spec = self.current_phase_spec()
        if spec.reference_type != ReferenceType.TRANSFER:
            return None, None, None, None
        try:
            target = self.target_state(spec.target_source)
        except RuntimeError:
            return None, None, None, None

        body_position = (
            target.position
            + self.desired_tip_offset()
            + self.quad_reference_offset_from_tip()
        )
        phase_name = self.phase.value
        if self.visual_trajectory_velocity_history_phase != phase_name:
            self.visual_trajectory_target_velocity_history.clear()
            self.visual_trajectory_velocity_history_phase = phase_name
        self.visual_trajectory_target_velocity_history.append(
            target.velocity.copy()
        )
        history = np.asarray(
            self.visual_trajectory_target_velocity_history, dtype=float
        )
        mean = np.mean(history, axis=0)
        variance = np.var(history, axis=0)
        return (
            body_position,
            mean,
            variance,
            target.acceleration.copy()
        )

    def collect_visual_astar_result(self) -> None:
        future = self.visual_astar_future
        if future is None or not future.done():
            return

        completed_time = self._wall_now_s()
        if self.visual_astar_last_submit_wall_time is None:
            self.last_visual_astar_worker_turnaround_ms = float("nan")
        else:
            self.last_visual_astar_worker_turnaround_ms = (
                1000.0
                * max(
                    0.0,
                    completed_time - self.visual_astar_last_submit_wall_time,
                )
            )

        self.visual_astar_future = None
        try:
            result = future.result()
        except Exception as exc:
            self.last_visual_astar_result = idle_visual_astar_result(
                phase_name=self.phase.value,
                status="worker_failure",
                message=f"visual A* worker failed: {exc}",
            )
            self.get_logger().error(f"Visual A* worker failed: {exc}")
            return

        if result.phase_name != self.phase.value:
            self.visual_astar_discarded_count += 1
            self.last_visual_astar_result = idle_visual_astar_result(
                phase_name=self.phase.value,
                status="stale_result_discarded",
                message=(
                    f"discarded visual A* result for old phase "
                    f"{result.phase_name}"
                ),
            )
            return
        self.last_visual_astar_result = result
        self.visual_astar_completed_count += 1


    def update_visual_astar(
        self,
        nominal_reference: TrajectoryReference,
        safety_request: SafetyPlannerRequest,
    ) -> None:
        """Poll the worker and submit a new immutable snapshot when due.
        Generates the request to the a star planner to do an a star thing"""
        self.collect_visual_astar_result()
        phase_allowed = self.visual_astar_phase_allowed()

        # C1F.8: during established C++ transfer authority this worker is
        # visualization-only and cannot affect the MPC reference. Do not submit
        # new A* jobs while the authoritative C++ planner is already consuming
        # the same CPU budget. An in-flight job is allowed to finish naturally.
        if self.c1f2_cpp_transit_authority_active():
            self.last_visual_astar_result = idle_visual_astar_result(
                phase_name=self.phase.value,
                status="cpp_authority_suppressed",
                message="visual A* suppressed during established C++ transit authority",
            )
            return

        # Condition checking before altering request
        if not self.visual_astar_enabled:
            self.last_visual_astar_result = idle_visual_astar_result(
                phase_name=self.phase.value,
                status="disabled",
                message="visual A* disabled",
            )
            return
        if not phase_allowed:
            self.last_visual_astar_result = idle_visual_astar_result(
                phase_name=self.phase.value,
                status="phase_not_allowed",
                message="visual A* is restricted to transfer phases",
            )
            return
        if self.visual_astar_future is not None:
            return

        now = self._physical_now_s()

        # update without considering the 2Hz gate
        (
            target_position,
            target_velocity_mean,
            target_velocity_variance,
            target_acceleration_mean,
        ) = self.visual_trajectory_target_statistics()

        # 2Hz replan gate
        replan_period = 1.0 / self.visual_astar_replan_rate_hz
        if (
            self.visual_astar_last_submit_time is not None
            and now - self.visual_astar_last_submit_time < replan_period
        ):
            return

        if (
            self.drone_position is None
            or self.drone_velocity is None
            or self.drone_acceleration is None
        ):
            self.last_visual_astar_result = idle_visual_astar_result(
                phase_name=self.phase.value,
                status="measured_state_unavailable",
                message=(
                    "visual A* is waiting for measured "
                    "drone position/velocity/acceleration"
                ),
            )
            return

        self.visual_astar_request_id += 1
        request = VisualAStarRequest(
            request_id=self.visual_astar_request_id,
            phase_name=self.phase.value,
            phase_allowed=phase_allowed,
            nominal_positions=nominal_reference.positions.copy(),
            nominal_velocities=nominal_reference.velocities.copy(),
            nominal_accelerations=nominal_reference.accelerations.copy(),
            planning_start_position=self.drone_position.copy(),     # factor in the drone's current position
            planning_start_velocity=self.drone_velocity.copy(),     # and velocity for the request
            planning_start_acceleration=self.drone_acceleration.copy(),
            dt=self.dt,
            obstacles=tuple(self.static_obstacles),
            drone_radius=self.drone_collision_radius,
            drone_safety_margin=self.collision_safety_margin,
            magnet_offset_from_quad=safety_request.magnet_offset_from_quad.copy(),
            magnet_radius=self.magnet_collision_radius,
            magnet_safety_margin=self.magnet_collision_margin,
            cable_radius=self.cable_collision_radius,
            cable_safety_margin=self.cable_collision_margin,
            object_attached=safety_request.object_attached,
            payload_profile=self.payload_profile,
            payload_geometry_available=(
                safety_request.payload_geometry_available
            ),
            payload_offset_from_magnet=(
                None
                if safety_request.payload_offset_from_magnet is None
                else safety_request.payload_offset_from_magnet.copy()
            ),
            payload_yaw=safety_request.payload_yaw,
            target_position=(
                None if target_position is None else target_position.copy()
            ),
            target_velocity_mean=(
                None
                if target_velocity_mean is None
                else target_velocity_mean.copy()
            ),
            target_velocity_variance=(
                None
                if target_velocity_variance is None
                else target_velocity_variance.copy()
            ),
            target_acceleration_mean=(
                None
                if target_acceleration_mean is None
                else target_acceleration_mean.copy()
            ),
        )
        self.visual_astar_future = self.visual_astar_executor.submit(
            compute_visual_astar,
            self.visual_astar_config,
            request,
            self.visual_trajectory_config,
        )
        self.visual_astar_last_submit_time = now
        self.visual_astar_last_submit_wall_time = self._wall_now_s()
        if self.last_visual_astar_result.request_id < 0:
            self.last_visual_astar_result = idle_visual_astar_result(
                phase_name=self.phase.value,
                status="search_pending",
                message="visual A* search submitted",
            )

    def close_visual_astar(self) -> None:
        future = self.visual_astar_future
        if future is not None:
            future.cancel()
            self.visual_astar_future = None
        self.visual_astar_executor.shutdown(wait=False, cancel_futures=True)

    # ----------------------------------------------------------------------
    # ROS output
    # ----------------------------------------------------------------------
    def m2b_magnet_command(self) -> str:
        if not self.is_m2b or self.m2b_commissioning_stage not in {"b1", "b2"}:
            return self.current_phase_spec().magnet_command
        if self.phase in {
            MissionPhase.M2_ATTACH_CAPTURE_WAIT,
            MissionPhase.M2_ATTACH_PROOF,
            MissionPhase.M2_ATTACHED_HOLD,
        }:
            return "ON"
        if self.phase == MissionPhase.M2_ATTACH_APPROACH_LOW and self.m2b_magnet_on_requested:
            return "ON"
        return "OFF"

    def publish_magnet_command(self, command: str) -> None:
        normalized = str(command).strip().upper()
        # the current command is re-sent every second: a single on-change publish was
        # lost when the magnet manager's subscription matched late (multi_drone_control
        # R0619: the pickup ON never arrived). Both magnet managers are idempotent.
        now_wall = self._wall_now_s()
        if normalized == self.last_magnet_command:
            if now_wall - getattr(self, "_magnet_command_sent_s", 0.0) < 1.0:
                return
            message = String()
            message.data = normalized
            self.magnet_command_publisher.publish(message)
            self._magnet_command_sent_s = now_wall
            return
        message = String()
        message.data = normalized
        self.magnet_command_publisher.publish(message)
        self._magnet_command_sent_s = now_wall
        self.last_magnet_command = normalized
        self.get_logger().info(f"Magnet command: {normalized}")

    def publish_m2c_ground_reference(self) -> None:
        """Publish a real per-drone current-pose reference for ground commissioning.

        M2C does not fly, but it should prove the complete planner/reference -> MPC
        message path rather than only proving that a subscriber exists.  This
        opt-in path is disabled outside M2C and publishes only while the measured
        vehicle is effectively stationary.
        """
        if not self.m2c_ground_reference_enabled or self.drone_position is None:
            return
        position = np.asarray(self.drone_position, dtype=float).reshape(3)
        velocity = (
            np.zeros(3, dtype=float)
            if self.drone_velocity is None
            else np.asarray(self.drone_velocity, dtype=float).reshape(3)
        )
        if not np.all(np.isfinite(position)) or not np.all(np.isfinite(velocity)):
            return
        if float(np.linalg.norm(velocity)) > self.committed_trajectory_stationary_speed_mps:
            return
        positions = np.repeat(
            position.reshape(1, 3), self.m2c_ground_reference_samples, axis=0
        )
        zeros = np.zeros_like(positions)
        self.publish_reference(TrajectoryReference(positions, zeros, zeros))

    def publish_stationary_committed_trajectory(self) -> None:
        """Publish the identity-bound grounded hold future used by M2C peers.

        Four identical cubic control points form an exact stationary cubic
        B-spline. Sequence numbers are monotonic for the lifetime of this node.
        This deliberately does not claim to encode a moving C++ commitment;
        M2C is ground-only and M2D owns moving-authority commitment plumbing.
        """
        if self.committed_trajectory_publisher is None or self.drone_position is None:
            return
        position = np.asarray(self.drone_position, dtype=float).reshape(3)
        velocity = (
            np.zeros(3, dtype=float)
            if self.drone_velocity is None
            else np.asarray(self.drone_velocity, dtype=float).reshape(3)
        )
        if not np.all(np.isfinite(position)) or not np.all(np.isfinite(velocity)):
            return
        if float(np.linalg.norm(velocity)) > self.committed_trajectory_stationary_speed_mps:
            return
        now = self.get_clock().now()
        t0 = now.nanoseconds * 1e-9
        t1 = t0 + self.committed_trajectory_horizon_s
        msg = CommittedTrajectory()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.frame_id
        msg.vehicle_id = self.vehicle_id
        self.committed_trajectory_sequence += 1
        msg.sequence = self.committed_trajectory_sequence
        msg.terminal_hold = True
        piece = CubicTrajectoryPiece()
        piece.valid_from_s = t0
        piece.valid_until_s = t1
        piece.knots = [t0, t0, t0, t0, t1, t1, t1, t1]
        piece.control_points = [_make_point(position) for _ in range(4)]
        msg.pieces = [piece]
        self.committed_trajectory_publisher.publish(msg)

    def publish_m2d_reference_commitment(self, reference: TrajectoryReference) -> None:
        """Publish one truthful compact future for the current M2D command.

        A low-speed stationary phase advertises the measured position as an exact
        cubic hold. Moving phases advertise the clamped cubic that matches the
        rolling command endpoints. ``terminal_hold`` is derived from that actual
        cubic's endpoint derivatives, not from the source reference metadata, so a
        moving cubic can never falsely claim a stopped continuation.
        """
        if (
            not self.m2d_enabled
            or self.committed_trajectory_publisher is None
            or reference.positions.shape[0] < 2
        ):
            return

        stationary_phases = {
            MissionPhase.M2_BOOTSTRAP_DETACH,
            MissionPhase.M2_BOOTSTRAP_SETTLE,
            MissionPhase.M2_WAIT_FOR_ARM,
            MissionPhase.M2_TAKEOFF_HOVER,
            MissionPhase.M2_ATTACHED_HOLD,
        }
        measured_speed = (
            float("inf")
            if self.drone_velocity is None
            else float(np.linalg.norm(np.asarray(self.drone_velocity, dtype=float)))
        )
        if (
            self.phase in stationary_phases
            and measured_speed <= self.committed_trajectory_stationary_speed_mps
        ):
            self.publish_stationary_committed_trajectory()
            return

        now = self.get_clock().now()
        t0 = now.nanoseconds * 1e-9
        duration = max(self.dt, self.dt * float(reference.positions.shape[0] - 1))
        t1 = t0 + duration
        p0 = np.asarray(reference.positions[0], dtype=float)
        p3 = np.asarray(reference.positions[-1], dtype=float)
        v0 = np.asarray(reference.velocities[0], dtype=float)
        v3 = np.asarray(reference.velocities[-1], dtype=float)
        control = [
            p0,
            p0 + v0 * (duration / 3.0),
            p3 - v3 * (duration / 3.0),
            p3,
        ]

        # Exact endpoint derivatives of this clamped one-piece cubic.  The old
        # code copied terminal velocity/acceleration flags from the 61-sample
        # source reference even though this compact Hermite cubic can have a
        # non-zero terminal acceleration.  That produced commitments which said
        # terminal_hold=true but failed the C++ endsInStoppedHold() contract.
        terminal_velocity = 3.0 * (control[3] - control[2]) / duration
        terminal_acceleration = (
            6.0 * (control[3] - 2.0 * control[2] + control[1]) / (duration * duration)
        )

        msg = CommittedTrajectory()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.frame_id
        msg.vehicle_id = self.vehicle_id
        self.committed_trajectory_sequence += 1
        msg.sequence = self.committed_trajectory_sequence
        msg.terminal_hold = bool(
            np.max(np.abs(terminal_velocity)) <= 1e-6
            and np.max(np.abs(terminal_acceleration)) <= 1e-6
        )
        piece = CubicTrajectoryPiece()
        piece.valid_from_s = t0
        piece.valid_until_s = t1
        piece.knots = [t0, t0, t0, t0, t1, t1, t1, t1]
        piece.control_points = [_make_point(point) for point in control]
        msg.pieces = [piece]
        self.committed_trajectory_publisher.publish(msg)

    def publish_reference(self, reference: TrajectoryReference) -> None:
        message = MultiDOFJointTrajectory()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.frame_id
        message.joint_names = [self.vehicle_id]

        for sample in range(reference.positions.shape[0]):
            point = MultiDOFJointTrajectoryPoint()
            transform = Transform()
            transform.translation.x = float(reference.positions[sample, 0])
            transform.translation.y = float(reference.positions[sample, 1])
            transform.translation.z = float(reference.positions[sample, 2])
            reference_yaw = None
            if self.is_m2b and self.m2b_initial_yaw is not None:
                reference_yaw = self.m2b_initial_yaw
            elif self.m2c_ground_reference_enabled and self.m2c_initial_yaw is not None:
                reference_yaw = self.m2c_initial_yaw
            if reference_yaw is not None:
                q_xyzw = yaw_quaternion_xyzw(reference_yaw)
                transform.rotation.x = float(q_xyzw[0])
                transform.rotation.y = float(q_xyzw[1])
                transform.rotation.z = float(q_xyzw[2])
                transform.rotation.w = float(q_xyzw[3])
            else:
                transform.rotation.w = 1.0
            point.transforms.append(transform)

            velocity = Twist()
            velocity.linear.x = float(reference.velocities[sample, 0])
            velocity.linear.y = float(reference.velocities[sample, 1])
            velocity.linear.z = float(reference.velocities[sample, 2])
            point.velocities.append(velocity)

            acceleration = Twist()
            acceleration.linear.x = float(reference.accelerations[sample, 0])
            acceleration.linear.y = float(reference.accelerations[sample, 1])
            acceleration.linear.z = float(reference.accelerations[sample, 2])
            point.accelerations.append(acceleration)

            nanoseconds = int(round(sample * self.dt * 1e9))
            point.time_from_start.sec = nanoseconds // 1_000_000_000
            point.time_from_start.nanosec = nanoseconds % 1_000_000_000
            message.points.append(point)

        self.reference_publisher.publish(message)
        self.publish_m2d_reference_commitment(reference)

    def build_visual_trajectory_log(self) -> dict[str, object]:
        """Return diagnostics for the active passive convex trajectory smoother."""
        astar = self.last_visual_astar_result
        convex_trajectory = astar.convex_trajectory

        convex_path_length_m: object = ""
        convex_route_length_ratio: object = ""
        if (
            convex_trajectory is not None
            and convex_trajectory.success
            and convex_trajectory.positions.ndim == 2
            and convex_trajectory.positions.shape[0] >= 2
        ):
            convex_path_length_m = float(
                np.sum(
                    np.linalg.norm(
                        np.diff(convex_trajectory.positions, axis=0),
                        axis=1,
                    )
                )
            )
            route_length_m = float(astar.simplified_path_length_m)
            if np.isfinite(route_length_m) and route_length_m > 1e-9:
                convex_route_length_ratio = convex_path_length_m / route_length_m

        return {
            "visual_trajectory_enabled": str(
                bool(self.visual_trajectory_enabled)
            ).lower(),
            "visual_trajectory_clearance_margin_m": (
                self.visual_trajectory_clearance_margin_m
            ),
            "visual_convex_trajectory_status": (
                "" if convex_trajectory is None else convex_trajectory.status
            ),
            "visual_convex_trajectory_message": (
                "" if convex_trajectory is None else convex_trajectory.message
            ),
            "visual_convex_trajectory_success": (
                ""
                if convex_trajectory is None
                else str(bool(convex_trajectory.success)).lower()
            ),
            "visual_convex_trajectory_solve_time_ms": (
                "" if convex_trajectory is None else convex_trajectory.solve_time_ms
            ),
            "visual_convex_trajectory_qp_solve_count": (
                "" if convex_trajectory is None else convex_trajectory.qp_solve_count
            ),
            "visual_convex_trajectory_solver_iterations": (
                "" if convex_trajectory is None else convex_trajectory.solver_iterations
            ),
            "visual_convex_trajectory_solver_message": (
                "" if convex_trajectory is None else convex_trajectory.solver_message
            ),
            "visual_convex_trajectory_segment_count": (
                ""
                if convex_trajectory is None
                else int(convex_trajectory.segment_times.size)
            ),
            "visual_convex_trajectory_control_point_count": (
                ""
                if convex_trajectory is None
                else int(convex_trajectory.control_points.shape[0] * 8)
            ),
            "visual_convex_trajectory_total_duration_s": (
                ""
                if convex_trajectory is None
                else float(np.sum(convex_trajectory.segment_times))
            ),
            "visual_convex_trajectory_path_length_m": convex_path_length_m,
            "visual_convex_trajectory_route_length_ratio": (
                convex_route_length_ratio
            ),
            "visual_convex_trajectory_retiming_used": (
                ""
                if convex_trajectory is None
                else str(bool(convex_trajectory.qp_solve_count > 1)).lower()
            ),
            "visual_convex_trajectory_peak_speed_mps": (
                "" if convex_trajectory is None else convex_trajectory.peak_speed_mps
            ),
            "visual_convex_trajectory_peak_acceleration_mps2": (
                ""
                if convex_trajectory is None
                else convex_trajectory.peak_acceleration_mps2
            ),
            "visual_convex_trajectory_velocity_control_bound_mps": (
                ""
                if convex_trajectory is None
                else convex_trajectory.velocity_control_bound_mps
            ),
            "visual_convex_trajectory_acceleration_control_bound_mps2": (
                ""
                if convex_trajectory is None
                else convex_trajectory.acceleration_control_bound_mps2
            ),
            "visual_convex_trajectory_validation_speed_limit_mps": (
                ""
                if convex_trajectory is None
                else convex_trajectory.validation_speed_limit_mps
            ),
            "visual_convex_trajectory_validation_acceleration_limit_mps2": (
                ""
                if convex_trajectory is None
                else convex_trajectory.validation_acceleration_limit_mps2
            ),
            "visual_convex_trajectory_corridor_violation_m": (
                ""
                if convex_trajectory is None
                else convex_trajectory.corridor_violation_m
            ),
            "visual_convex_trajectory_boundary_position_error_m": (
                ""
                if convex_trajectory is None
                else convex_trajectory.boundary_position_error_m
            ),
            "visual_convex_trajectory_boundary_velocity_error_mps": (
                ""
                if convex_trajectory is None
                else convex_trajectory.boundary_velocity_error_mps
            ),
            "visual_convex_trajectory_boundary_acceleration_error_mps2": (
                ""
                if convex_trajectory is None
                else convex_trajectory.boundary_acceleration_error_mps2
            ),
            "visual_convex_trajectory_join_position_error_m": (
                ""
                if convex_trajectory is None
                else convex_trajectory.join_position_error_m
            ),
            "visual_convex_trajectory_join_velocity_error_mps": (
                ""
                if convex_trajectory is None
                else convex_trajectory.join_velocity_error_mps
            ),
            "visual_convex_trajectory_join_acceleration_error_mps2": (
                ""
                if convex_trajectory is None
                else convex_trajectory.join_acceleration_error_mps2
            ),
            "visual_convex_trajectory_join_jerk_error_mps3": (
                ""
                if convex_trajectory is None
                else convex_trajectory.join_jerk_error_mps3
            ),
        }

    def build_status_text(self) -> str:
        spec = self.current_phase_spec()
        lines = [
            f"{self.phase.value}",
            f"reference={spec.reference_type.value}, target={spec.target_source.value}",
            f"authority={spec.controller_authority.value}, handoff_ready={spec.handoff_ready}",
        ]
        if self.drone_position is not None and self.inputs_ready_for_status():
            try:
                xy_error, relative_velocity, z_error = self.tracking_errors()
                lines.append(
                    f"tip xy error={xy_error:.3f} m, relative xy speed={relative_velocity:.3f} m/s"
                )
                lines.append(f"tip z error={z_error:.3f} m")
            except RuntimeError:
                pass
        if spec.reference_type == ReferenceType.TRANSFER:
            lines.append(
                "transfer virtual time: "
                f"duration={self.transfer_generator.last_duration:.2f} s, "
                f"progress_scale={self.transfer_generator.last_progress_scale:.2f}"
            )
        if self.c1f1_shadow_transfer_enabled:
            reference_age_ms = self.c1f1_shadow_reference_age_ms()
            reference_age_text = (
                "none"
                if not np.isfinite(reference_age_ms)
                else f"{reference_age_ms:.1f} ms"
            )
            drift_text = (
                "none"
                if not np.isfinite(self.c1f1_shadow_target_drift_m)
                else f"{self.c1f1_shadow_target_drift_m:.3f} m"
            )
            lines.append(
                "C1F.1 shadow: "
                f"phase_active={self.c1f1_shadow_phase_active()}, "
                f"commanded={self.c1f1_shadow_commanded_enable}, "
                f"ref_fresh={self.c1f1_shadow_reference_fresh()}, "
                f"ref_age={reference_age_text}, "
                f"samples={self.c1f1_shadow_reference_sample_count}, "
                f"target_drift={drift_text}, "
                f"backend={self.c1f1_shadow_backend_status}"
            )
        if self.c1f2_cpp_authority_enabled:
            lines.append(
                "C1F.2b authority: "
                f"phase_active={self.c1f2_cpp_phase_active()}, "
                f"prepare={self.c1f2_prepare_active}, "
                f"settled={self.c1f2_prepare_settled}, "
                f"hold_drift={self.c1f2_prepare_drift_m:.4f}m, "
                f"hold_speed={self.c1f2_prepare_speed_mps:.4f}m/s, "
                f"ready={self.c1f2_handoff_ready}, "
                f"prepare_wait={self.c1f2_prepare_wait_s:.2f}s, "
                f"cpp_active={self.c1f2_cpp_authority_active}, "
                f"grant_cmd={self.c1f2_commanded_authority}, "
                f"grant_pending={self.c1f2_authority_grant_pending}, "
                f"grant_ack={self.c1f2_authority_acknowledged}, "
                f"grants={self.c1f2_authority_grant_count}, "
                f"switches={self.c1f2_authority_switch_count}, "
                f"cached_dropouts={self.c1f2_cached_continuation_count}, "
                f"cached_active={self.c1f2_cached_continuation_active}, "
                f"recovery_rejects={self.c1f2_recovery_reject_count}, "
                f"exit_switches={self.c1f2_exit_switch_count}, "
                f"exit_bridge={self.c1f2_exit_bridge_active}, "
                f"exit_bridge_complete={self.c1f2_exit_bridge_complete}, "
                f"entry_pva=[{self.c1f2_handoff_position_error_m:.4f}, "
                f"{self.c1f2_handoff_velocity_error_mps:.4f}, "
                f"{self.c1f2_handoff_acceleration_error_mps2:.4f}]"
            )
        if (
            self.is_m2b
            and self.m2b_commissioning_stage in {"b1", "b2"}
            and is_m2b_b1_local_reference_phase(self.phase)
        ):
            # B1 phases use build_m2b_b1_reference(), not legacy rate_profile().
            # Keep diagnostics on the same shared phase classifier as reference
            # generation so a status message can never kill the mission executive.
            local_detail = f"phase={self.phase.value}, elapsed={self.phase_elapsed():.2f} s"
            if (
                self.phase in {
                    MissionPhase.M2_ATTACH_APPROACH_HIGH,
                    MissionPhase.M2_ATTACH_APPROACH_LOW,
                }
                and self.m2b_descent_progress is not None
            ):
                local_detail += (
                    f", target_contact_height="
                    f"{self.m2b_descent_progress.target_contact_height_m:.3f} m"
                )
            lines.append(f"M2B local reference: {local_detail}")
        elif spec.reference_type == ReferenceType.RATE_CONTROLLED_MANOEUVRE:
            profile = (
                None
                if self.phase == MissionPhase.LANDING
                else self.rate_profile()
            )
            duration = (
                self.last_rate_profile_duration
                if self.phase == MissionPhase.LANDING
                else profile.duration
            )
            lines.append(
                f"rate profile={self.phase_elapsed():.2f}/{duration:.2f} s, "
                f"complete={self.last_rate_profile_complete}"
            )
        lines.append(f"object_attached={self.object_attached}")
        if self.commissioning_hold_after_lift or self.commissioning_hold_before_drop:
            lines.append(
                "IRL commissioning gates: "
                f"hold_after_lift={self.commissioning_hold_after_lift}, "
                f"hold_before_drop={self.commissioning_hold_before_drop}"
            )
        astar = self.last_visual_astar_result
        astar_pending = self.visual_astar_future is not None
        lines.append(
            f"visual A*: status={astar.status}, pending={astar_pending}, "
            f"success={astar.success}, trigger={astar.first_blocked_segment_index}, "
            f"rejoin={astar.rejoin_index}"
        )
        if astar.success:
            whole_body_text = (
                "unknown"
                if astar.candidate_whole_body_safe is None
                else str(astar.candidate_whole_body_safe).lower()
            )
            lines.append(
                f"A* plan={astar.planning_time_ms:.2f} ms, "
                f"expanded={astar.expanded_nodes}, "
                f"whole_body_safe={whole_body_text}"
            )
        convex_trajectory = (
            astar.convex_trajectory
        )

        if convex_trajectory is not None:
            lines.append(
                "convex trajectory="
                f"{convex_trajectory.status}, "
                f"solve={convex_trajectory.solve_time_ms:.1f} ms, "
                f"QP solves={convex_trajectory.qp_solve_count}, "
                f"v_bound={convex_trajectory.velocity_control_bound_mps:.2f} m/s, "
                f"a_bound={convex_trajectory.acceleration_control_bound_mps2:.2f} m/s^2"
            )
        if np.isfinite(self.last_planner_callback_time_ms):
            period_text = (
                "unknown"
                if not np.isfinite(self.last_planner_callback_period_ms)
                else f"{self.last_planner_callback_period_ms:.2f} ms"
            )
            lines.append(
                f"planner callback={self.last_planner_callback_time_ms:.2f} ms, "
                f"period={period_text}, "
                f"deadline_misses={self.planner_deadline_miss_count}"
            )
        lines.append(f"last transition: {self.last_transition_reason}")
        return "\n".join(lines)

    def inputs_ready_for_status(self) -> bool:
        """Non-logging readiness check used only while composing status text."""
        if self.drone_position is None:
            return False
        source = self.current_phase_spec().target_source
        if source == TargetSource.PICKUP_OBJECT:
            return self.pickup_object_position is not None
        if source == TargetSource.DROP_POINT:
            if self.drop_point_source in {"static", "fixed", "hardcoded"}:
                return True
            if self.drop_point_source == "payload":
                return self.payload_position is not None
            return self.attachment_position is not None
        if source == TargetSource.ATTACHMENT_POINT:
            return self.attachment_position is not None
        return True

    def publish_state(self) -> None:
        self.publish_phase()
        self.last_status_text = self.build_status_text()
        message = String()
        message.data = self.last_status_text
        self.state_publisher.publish(message)

        ready = Bool()
        ready.data = bool(self.current_phase_spec().handoff_ready)
        self.handoff_ready_publisher.publish(ready)

        safety = String()
        if self.last_safety_result is None:
            safety.data = "safety planner waiting for first nominal reference"
        else:
            safety.data = self.last_safety_result.status
        self.safety_status_publisher.publish(safety)

    def publish_phase(self) -> None:
        """Publish the exact machine-readable mission phase for supervisors."""
        message = String()
        message.data = self.phase.value
        self.phase_publisher.publish(message)

    def append_moving_obstacle_prediction_markers(
        self,
        markers: MarkerArray,
        stamp,
    ) -> None:
        """Append the cached M2A reachable tube to the planner MarkerArray.

        This is visualization only. A stale or missing obstacle state produces no
        marker and has no effect on A*, occupancy, corridor generation, smoothing,
        safety-planner output, or the published controller reference.
        """

        if (
            not self.moving_obstacle_prediction_enabled
            or not self.moving_obstacle_show_prediction_markers
            or self.latest_moving_obstacle_prediction is None
            or self.last_moving_obstacle_state_time is None
        ):
            return

        age_s = max(
            0.0,
            self._physical_now_s() - self.last_moving_obstacle_state_time,
        )
        if age_s > self.moving_obstacle_state_timeout_s:
            return

        prediction = self.latest_moving_obstacle_prediction

        # Physical measured obstacle. The reachable slices below include the
        # configured uncertainty expansion, so keep the physical sphere separate.
        actual = Marker()
        actual.header.stamp = stamp
        actual.header.frame_id = self.frame_id
        actual.ns = "moving_obstacle_prediction_actual"
        actual.id = 0
        actual.type = Marker.SPHERE
        actual.action = Marker.ADD
        actual.pose.position = _make_point(prediction.sample_centers[0])
        actual.pose.orientation.w = 1.0
        diameter = 2.0 * self.moving_obstacle_geometry_radius_m
        actual.scale.x = diameter
        actual.scale.y = diameter
        actual.scale.z = diameter
        actual.color.r = 0.05
        actual.color.g = 0.90
        actual.color.b = 1.00
        actual.color.a = 0.70
        markers.markers.append(actual)

        last_index = max(
            1,
            len(prediction.sample_times_s) - 1,
        )
        for index, (center, reach_radius) in enumerate(
            zip(
                prediction.sample_centers,
                prediction.sample_radii_m,
            )
        ):
            sphere = Marker()
            sphere.header.stamp = stamp
            sphere.header.frame_id = self.frame_id
            sphere.ns = "moving_obstacle_prediction_reachable_slices"
            sphere.id = index
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position = _make_point(center)
            sphere.pose.orientation.w = 1.0
            slice_diameter = 2.0 * float(reach_radius)
            sphere.scale.x = slice_diameter
            sphere.scale.y = slice_diameter
            sphere.scale.z = slice_diameter
            sphere.color.r = 0.05
            sphere.color.g = 0.80
            sphere.color.b = 1.00
            sphere.color.a = 0.05 + 0.10 * (1.0 - index / last_index)
            markers.markers.append(sphere)

        centreline = Marker()
        centreline.header.stamp = stamp
        centreline.header.frame_id = self.frame_id
        centreline.ns = "moving_obstacle_prediction_centreline"
        centreline.id = 0
        centreline.type = Marker.LINE_STRIP
        centreline.action = Marker.ADD
        centreline.pose.orientation.w = 1.0
        centreline.scale.x = 0.015
        centreline.color.r = 0.05
        centreline.color.g = 0.90
        centreline.color.b = 1.00
        centreline.color.a = 0.95
        centreline.points = [
            _make_point(center)
            for center in prediction.sample_centers
        ]
        markers.markers.append(centreline)

    def publish_markers(self, reference: TrajectoryReference) -> None:
        now = self.get_clock().now().to_msg()
        markers = MarkerArray()
        delete_all = Marker()
        delete_all.header.stamp = now
        delete_all.header.frame_id = self.frame_id
        delete_all.action = Marker.DELETEALL
        markers.markers.append(delete_all)

        self.append_moving_obstacle_prediction_markers(
            markers,
            now,
        )

        nominal_reference = self.last_nominal_reference or reference
        nominal_safe = (
            None
            if self.last_safety_result is None
            else self.last_safety_result.nominal_safe
        )

        path = Marker()
        path.header.stamp = now
        path.header.frame_id = self.frame_id
        path.ns = "path_nominal_reference"
        path.id = 0
        path.type = Marker.LINE_STRIP
        path.action = Marker.ADD
        path.scale.x = 0.025
        if nominal_safe is False:
            path.color.r = 1.0
            path.color.g = 0.05
            path.color.b = 0.05
        elif nominal_safe is True:
            path.color.r = 0.10
            path.color.g = 0.90
            path.color.b = 0.20
        else:
            path.color.r = 1.0
            path.color.g = 0.5
            path.color.b = 0.0
        path.color.a = 1.0
        path.points = [
            _make_point(position) for position in nominal_reference.positions
        ]
        markers.markers.append(path)

        astar = self.last_visual_astar_result
        if astar.raw_path.shape[0] >= 2:
            raw_path = Marker()
            raw_path.header.stamp = now
            raw_path.header.frame_id = self.frame_id
            raw_path.ns = "visual_astar_raw_voxel_path"
            raw_path.id = 0
            raw_path.type = Marker.LINE_STRIP
            raw_path.action = Marker.ADD
            raw_path.scale.x = 0.012
            raw_path.color.r = 0.65
            raw_path.color.g = 0.65
            raw_path.color.b = 0.70
            raw_path.color.a = 0.90
            raw_path.points = [_make_point(point) for point in astar.raw_path]
            markers.markers.append(raw_path)

        if astar.simplified_path.shape[0] >= 2:
            simplified_path = Marker()
            simplified_path.header.stamp = now
            simplified_path.header.frame_id = self.frame_id
            simplified_path.ns = "visual_astar_simplified_path"
            simplified_path.id = 0
            simplified_path.type = Marker.LINE_STRIP
            simplified_path.action = Marker.ADD
            simplified_path.scale.x = 0.032
            simplified_path.color.r = 0.10
            simplified_path.color.g = 0.55
            simplified_path.color.b = 1.00
            simplified_path.color.a = 0.95
            simplified_path.points = [
                _make_point(point) for point in astar.simplified_path
            ]
            markers.markers.append(simplified_path)

        # added aug 13 23:44 occupancy grid markers
        show_occupancy = bool(
            self.get_parameter(
                "visual_astar_show_occupancy_voxels"
            ).value
        )

        if (
            show_occupancy
            and astar.occupancy_points.shape[0] > 0
            and astar.occupancy_resolution_m > 0.0
        ):
            occupancy = Marker()
            occupancy.header.stamp = now
            occupancy.header.frame_id = self.frame_id

            occupancy.ns = (
                "visual_astar_whole_body_occupancy"
            )
            occupancy.id = 0

            occupancy.type = Marker.CUBE_LIST
            occupancy.action = Marker.ADD

            occupancy.scale.x = (
                astar.occupancy_resolution_m
            )
            occupancy.scale.y = (
                astar.occupancy_resolution_m
            )
            occupancy.scale.z = (
                astar.occupancy_resolution_m
            )

            occupancy.color.r = 1.00
            occupancy.color.g = 0.25
            occupancy.color.b = 0.05
            occupancy.color.a = 0.10

            occupancy.points = [
                _make_point(point)
                for point in astar.occupancy_points
            ]

            markers.markers.append(occupancy)

        for cell_index, cell in enumerate(astar.convex_corridor):
            triangles = polyhedron_triangles(cell)

            if triangles.shape[0] == 0:
                continue

            corridor_surface = Marker()
            corridor_surface.header.stamp = now
            corridor_surface.header.frame_id = self.frame_id
            corridor_surface.ns = "visual_astar_convex_corridor_surface"
            corridor_surface.id = cell_index
            corridor_surface.type = Marker.TRIANGLE_LIST
            corridor_surface.action = Marker.ADD

            corridor_surface.scale.x = 1.0
            corridor_surface.scale.y = 1.0
            corridor_surface.scale.z = 1.0

            corridor_surface.color.r = 0.55
            corridor_surface.color.g = 0.20
            corridor_surface.color.b = 1.00
            corridor_surface.color.a = 0.12

            corridor_surface.points = [
                _make_point(point)
                for point in triangles.reshape(-1, 3)
            ]

            markers.markers.append(corridor_surface)

            corridor_edges = Marker()
            corridor_edges.header.stamp = now
            corridor_edges.header.frame_id = self.frame_id
            corridor_edges.ns = "visual_astar_convex_corridor_edges"
            corridor_edges.id = cell_index
            corridor_edges.type = Marker.LINE_LIST
            corridor_edges.action = Marker.ADD

            corridor_edges.scale.x = 0.008

            corridor_edges.color.r = 0.65
            corridor_edges.color.g = 0.35
            corridor_edges.color.b = 1.00
            corridor_edges.color.a = 0.75

            edge_points = []

            for triangle in triangles:
                for first, second in (
                    (0, 1),
                    (1, 2),
                    (2, 0),
                ):
                    edge_points.append(
                        _make_point(triangle[first])
                    )
                    edge_points.append(
                        _make_point(triangle[second])
                    )

            corridor_edges.points = edge_points
            markers.markers.append(corridor_edges)

        if astar.candidate_positions.shape[0] >= 2:
            candidate_path = Marker()
            candidate_path.header.stamp = now
            candidate_path.header.frame_id = self.frame_id
            candidate_path.ns = "visual_astar_candidate_trajectory"
            candidate_path.id = 0
            candidate_path.type = Marker.LINE_STRIP
            candidate_path.action = Marker.ADD
            candidate_path.scale.x = 0.050
            if astar.candidate_whole_body_safe is True:
                candidate_path.color.r = 0.05
                candidate_path.color.g = 1.00
                candidate_path.color.b = 0.55
            elif astar.candidate_whole_body_safe is False:
                candidate_path.color.r = 1.00
                candidate_path.color.g = 0.05
                candidate_path.color.b = 0.05
            else:
                candidate_path.color.r = 1.00
                candidate_path.color.g = 0.55
                candidate_path.color.b = 0.05
            candidate_path.color.a = 0.95
            candidate_path.points = [
                _make_point(point) for point in astar.candidate_positions
            ]
            markers.markers.append(candidate_path)

        blocked_index = astar.first_blocked_segment_index
        if (
            blocked_index is not None
            and 0 <= blocked_index < len(nominal_reference.positions) - 1
        ):
            blocked_segment = Marker()
            blocked_segment.header.stamp = now
            blocked_segment.header.frame_id = self.frame_id
            blocked_segment.ns = "visual_astar_first_blocked_segment"
            blocked_segment.id = 0
            blocked_segment.type = Marker.LINE_LIST
            blocked_segment.action = Marker.ADD
            blocked_segment.scale.x = 0.070
            blocked_segment.color.r = 1.00
            blocked_segment.color.g = 0.05
            blocked_segment.color.b = 0.05
            blocked_segment.color.a = 1.00
            blocked_segment.points = [
                _make_point(nominal_reference.positions[blocked_index]),
                _make_point(nominal_reference.positions[blocked_index + 1]),
            ]
            markers.markers.append(blocked_segment)

        if astar.rejoin_candidate_positions.shape[0] > 0:
            candidates = Marker()
            candidates.header.stamp = now
            candidates.header.frame_id = self.frame_id
            candidates.ns = "visual_astar_rejoin_candidates"
            candidates.id = 0
            candidates.type = Marker.SPHERE_LIST
            candidates.action = Marker.ADD
            candidates.scale.x = candidates.scale.y = candidates.scale.z = 0.055
            candidates.color.r = 1.00
            candidates.color.g = 0.85
            candidates.color.b = 0.05
            candidates.color.a = 0.80
            candidates.points = [
                _make_point(point) for point in astar.rejoin_candidate_positions
            ]
            markers.markers.append(candidates)

        if astar.start_requested is not None:
            requested_start = Marker()
            requested_start.header.stamp = now
            requested_start.header.frame_id = self.frame_id
            requested_start.ns = "visual_astar_start_requested"
            requested_start.id = 0
            requested_start.type = Marker.SPHERE
            requested_start.action = Marker.ADD
            requested_start.pose.position = _make_point(astar.start_requested)
            requested_start.pose.orientation.w = 1.0
            requested_start.scale.x = requested_start.scale.y = requested_start.scale.z = 0.10
            requested_start.color.r = 1.00
            requested_start.color.g = 0.85
            requested_start.color.b = 0.05
            requested_start.color.a = 0.95
            markers.markers.append(requested_start)

        if astar.start_used is not None:
            used_start = Marker()
            used_start.header.stamp = now
            used_start.header.frame_id = self.frame_id
            used_start.ns = "visual_astar_start_used"
            used_start.id = 0
            used_start.type = Marker.SPHERE
            used_start.action = Marker.ADD
            used_start.pose.position = _make_point(astar.start_used)
            used_start.pose.orientation.w = 1.0
            used_start.scale.x = used_start.scale.y = used_start.scale.z = 0.070
            used_start.color.r = 0.10
            used_start.color.g = 1.00
            used_start.color.b = 0.10
            used_start.color.a = 1.00
            markers.markers.append(used_start)

        if astar.goal_requested is not None:
            requested_goal = Marker()
            requested_goal.header.stamp = now
            requested_goal.header.frame_id = self.frame_id
            requested_goal.ns = "visual_astar_rejoin_goal_requested"
            requested_goal.id = 0
            requested_goal.type = Marker.SPHERE
            requested_goal.action = Marker.ADD
            requested_goal.pose.position = _make_point(astar.goal_requested)
            requested_goal.pose.orientation.w = 1.0
            requested_goal.scale.x = requested_goal.scale.y = requested_goal.scale.z = 0.11
            requested_goal.color.r = 1.00
            requested_goal.color.g = 0.15
            requested_goal.color.b = 0.90
            requested_goal.color.a = 0.95
            markers.markers.append(requested_goal)

        if astar.goal_used is not None:
            used_goal = Marker()
            used_goal.header.stamp = now
            used_goal.header.frame_id = self.frame_id
            used_goal.ns = "visual_astar_rejoin_goal_used"
            used_goal.id = 0
            used_goal.type = Marker.SPHERE
            used_goal.action = Marker.ADD
            used_goal.pose.position = _make_point(astar.goal_used)
            used_goal.pose.orientation.w = 1.0
            used_goal.scale.x = used_goal.scale.y = used_goal.scale.z = 0.075
            used_goal.color.r = 0.60
            used_goal.color.g = 0.05
            used_goal.color.b = 1.00
            used_goal.color.a = 1.00
            markers.markers.append(used_goal)

        if astar.bounds_min is not None and astar.bounds_max is not None:
            bounds = Marker()
            bounds.header.stamp = now
            bounds.header.frame_id = self.frame_id
            bounds.ns = "visual_astar_local_bounds"
            bounds.id = 0
            bounds.type = Marker.CUBE
            bounds.action = Marker.ADD
            bounds.pose.position = _make_point(
                0.5 * (astar.bounds_min + astar.bounds_max)
            )
            bounds.pose.orientation.w = 1.0
            size = astar.bounds_max - astar.bounds_min
            bounds.scale.x = float(size[0])
            bounds.scale.y = float(size[1])
            bounds.scale.z = float(size[2])
            bounds.color.r = 0.20
            bounds.color.g = 0.55
            bounds.color.b = 1.00
            bounds.color.a = 0.035
            markers.markers.append(bounds)

        safety_result = self.last_safety_result
        predicted_magnet_positions = (
            None
            if safety_result is None
            else safety_result.predicted_magnet_positions
        )
        if predicted_magnet_positions is not None:
            magnet_safe = (
                safety_result.magnet_minimum_clearance is not None
                and safety_result.magnet_minimum_clearance >= -1e-9
            )

            magnet_path = Marker()
            magnet_path.header.stamp = now
            magnet_path.header.frame_id = self.frame_id
            magnet_path.ns = "path_predicted_magnet"
            magnet_path.id = 0
            magnet_path.type = Marker.LINE_STRIP
            magnet_path.action = Marker.ADD
            magnet_path.scale.x = 0.018
            if magnet_safe:
                magnet_path.color.r = 0.75
                magnet_path.color.g = 0.15
                magnet_path.color.b = 1.0
            else:
                magnet_path.color.r = 1.0
                magnet_path.color.g = 0.05
                magnet_path.color.b = 0.05
            magnet_path.color.a = 1.0
            magnet_path.points = [
                _make_point(position) for position in predicted_magnet_positions
            ]
            markers.markers.append(magnet_path)

            magnet_volume = Marker()
            magnet_volume.header.stamp = now
            magnet_volume.header.frame_id = self.frame_id
            magnet_volume.ns = "magnet_collision_volume"
            magnet_volume.id = 0
            magnet_volume.type = Marker.SPHERE
            magnet_volume.action = Marker.ADD
            magnet_volume.pose.position = _make_point(predicted_magnet_positions[0])
            magnet_volume.pose.orientation.w = 1.0
            magnet_diameter = 2.0 * self.magnet_collision_radius
            magnet_volume.scale.x = magnet_diameter
            magnet_volume.scale.y = magnet_diameter
            magnet_volume.scale.z = magnet_diameter
            magnet_volume.color.r = 0.75
            magnet_volume.color.g = 0.15
            magnet_volume.color.b = 1.0
            magnet_volume.color.a = 0.40
            markers.markers.append(magnet_volume)

            if self.show_collision_markers:
                # Show one present cable and at most one worst future cable.  The
                # previous prototype rendered the complete horizon as an array of
                # lines, which was both cluttered and unnecessarily expensive.
                current_cable = Marker()
                current_cable.header.stamp = now
                current_cable.header.frame_id = self.frame_id
                current_cable.ns = "cable_current"
                current_cable.id = 0
                current_cable.type = Marker.LINE_LIST
                current_cable.action = Marker.ADD
                current_cable.scale.x = max(0.010, 2.0 * self.cable_collision_radius)
                current_cable_unsafe = (
                    safety_result.cable_closest_sample_index == 0
                    and safety_result.cable_minimum_clearance is not None
                    and safety_result.cable_minimum_clearance < -1e-9
                )
                if current_cable_unsafe:
                    current_cable.color.r = 1.0
                    current_cable.color.g = 0.05
                    current_cable.color.b = 0.05
                else:
                    current_cable.color.r = 0.10
                    current_cable.color.g = 0.45
                    current_cable.color.b = 1.0
                current_cable.color.a = 0.95
                current_quad_position = (
                    self.drone_position
                    if self.drone_position is not None
                    else nominal_reference.positions[0]
                )
                current_offset = (
                    predicted_magnet_positions[0] - nominal_reference.positions[0]
                )
                current_cable.points = [
                    _make_point(current_quad_position),
                    _make_point(current_quad_position + current_offset),
                ]
                markers.markers.append(current_cable)

                worst_quad = safety_result.cable_closest_quad_position
                worst_magnet = safety_result.cable_closest_magnet_position
                worst_sample = safety_result.cable_closest_sample_index
                if (
                    worst_quad is not None
                    and worst_magnet is not None
                    and worst_sample is not None
                    and worst_sample > 0
                ):
                        worst_cable = Marker()
                        worst_cable.header.stamp = now
                        worst_cable.header.frame_id = self.frame_id
                        worst_cable.ns = "cable_worst_future"
                        worst_cable.id = 0
                        worst_cable.type = Marker.LINE_LIST
                        worst_cable.action = Marker.ADD
                        worst_cable.scale.x = max(
                            0.014, 2.0 * self.cable_collision_radius
                        )
                        cable_unsafe = (
                            safety_result.cable_minimum_clearance is not None
                            and safety_result.cable_minimum_clearance < -1e-9
                        )
                        if cable_unsafe:
                            worst_cable.color.r = 1.0
                            worst_cable.color.g = 0.05
                            worst_cable.color.b = 0.05
                        else:
                            worst_cable.color.r = 0.05
                            worst_cable.color.g = 0.90
                            worst_cable.color.b = 1.0
                        worst_cable.color.a = 1.0
                        worst_cable.points = [
                            _make_point(worst_quad),
                            _make_point(worst_magnet),
                        ]
                        markers.markers.append(worst_cable)

        predicted_payload_positions = (
            None
            if safety_result is None
            else safety_result.predicted_payload_positions
        )
        predicted_payload_yaw = (
            None
            if safety_result is None
            else safety_result.predicted_payload_yaw
        )
        if (
            self.show_collision_markers
            and safety_result is not None
            and safety_result.payload_check_active
            and safety_result.payload_geometry_available
            and predicted_payload_positions is not None
            and predicted_payload_yaw is not None
        ):
            payload_current = Marker()
            payload_current.header.stamp = now
            payload_current.header.frame_id = self.frame_id
            payload_current.ns = "payload_current_physical_box"
            payload_current.id = 0
            payload_current.type = Marker.CUBE
            payload_current.action = Marker.ADD
            payload_current.pose.position = _make_point(predicted_payload_positions[0])
            _set_marker_yaw(payload_current, predicted_payload_yaw[0])
            payload_current.scale.x = float(self.payload_profile.dimensions[0])
            payload_current.scale.y = float(self.payload_profile.dimensions[1])
            payload_current.scale.z = float(self.payload_profile.dimensions[2])
            payload_current.color.r = 0.15
            payload_current.color.g = 0.75
            payload_current.color.b = 0.95
            payload_current.color.a = 0.65
            markers.markers.append(payload_current)

            payload_inflated = Marker()
            payload_inflated.header.stamp = now
            payload_inflated.header.frame_id = self.frame_id
            payload_inflated.ns = "payload_current_inflated_box"
            payload_inflated.id = 0
            payload_inflated.type = Marker.CUBE
            payload_inflated.action = Marker.ADD
            payload_inflated.pose.position = _make_point(predicted_payload_positions[0])
            _set_marker_yaw(payload_inflated, predicted_payload_yaw[0])
            inflated_dimensions = self.payload_profile.inflated_dimensions
            payload_inflated.scale.x = float(inflated_dimensions[0])
            payload_inflated.scale.y = float(inflated_dimensions[1])
            payload_inflated.scale.z = float(inflated_dimensions[2])
            payload_inflated.color.r = 0.10
            payload_inflated.color.g = 0.85
            payload_inflated.color.b = 1.00
            payload_inflated.color.a = 0.16
            markers.markers.append(payload_inflated)

            worst_payload_sample = safety_result.payload_closest_sample_index
            if worst_payload_sample is not None and worst_payload_sample > 0:
                payload_worst = Marker()
                payload_worst.header.stamp = now
                payload_worst.header.frame_id = self.frame_id
                payload_worst.ns = "payload_worst_future_box"
                payload_worst.id = 0
                payload_worst.type = Marker.CUBE
                payload_worst.action = Marker.ADD
                payload_worst.pose.position = _make_point(
                    predicted_payload_positions[worst_payload_sample]
                )
                _set_marker_yaw(
                    payload_worst, predicted_payload_yaw[worst_payload_sample]
                )
                payload_worst.scale.x = float(inflated_dimensions[0])
                payload_worst.scale.y = float(inflated_dimensions[1])
                payload_worst.scale.z = float(inflated_dimensions[2])
                payload_unsafe = (
                    safety_result.payload_minimum_clearance is not None
                    and safety_result.payload_minimum_clearance < -1e-9
                )
                if payload_unsafe:
                    payload_worst.color.r = 1.0
                    payload_worst.color.g = 0.05
                    payload_worst.color.b = 0.05
                else:
                    payload_worst.color.r = 1.0
                    payload_worst.color.g = 0.65
                    payload_worst.color.b = 0.05
                payload_worst.color.a = 0.42
                markers.markers.append(payload_worst)

        start = Marker()
        start.header.stamp = now
        start.header.frame_id = self.frame_id
        start.ns = "reference_stage_zero"
        start.id = 0
        start.type = Marker.SPHERE
        start.action = Marker.ADD
        start.pose.position = _make_point(reference.positions[0])
        start.pose.orientation.w = 1.0
        start.scale.x = start.scale.y = start.scale.z = 0.08
        start.color.r = 1.0
        start.color.g = 1.0
        start.color.b = 0.0
        start.color.a = 1.0
        markers.markers.append(start)

        end = Marker()
        end.header.stamp = now
        end.header.frame_id = self.frame_id
        end.ns = "reference_horizon_end"
        end.id = 0
        end.type = Marker.SPHERE
        end.action = Marker.ADD
        end.pose.position = _make_point(reference.positions[-1])
        end.pose.orientation.w = 1.0
        end.scale.x = end.scale.y = end.scale.z = 0.10
        end.color.r = 1.0
        end.color.g = 0.2
        end.color.b = 0.0
        end.color.a = 1.0
        markers.markers.append(end)

        if self.show_collision_markers:
            closest_obstacle_id = (
                None
                if self.last_safety_result is None
                else self.last_safety_result.closest_obstacle_id
            )
            for obstacle_index, obstacle in enumerate(self.static_obstacles):
                physical = Marker()
                physical.header.stamp = now
                physical.header.frame_id = self.frame_id
                physical.ns = "static_obstacle_physical"
                physical.id = obstacle_index
                physical.type = Marker.SPHERE
                physical.action = Marker.ADD
                physical.pose.position = _make_point(obstacle.centre)
                physical.pose.orientation.w = 1.0
                physical_diameter = 2.0 * obstacle.radius
                physical.scale.x = physical_diameter
                physical.scale.y = physical_diameter
                physical.scale.z = physical_diameter
                physical.color.r = 0.20
                physical.color.g = 0.45
                physical.color.b = 0.95
                physical.color.a = 0.70
                markers.markers.append(physical)

                inflated = Marker()
                inflated.header.stamp = now
                inflated.header.frame_id = self.frame_id
                inflated.ns = "static_obstacle_inflated"
                inflated.id = obstacle_index
                inflated.type = Marker.SPHERE
                inflated.action = Marker.ADD
                inflated.pose.position = _make_point(obstacle.centre)
                inflated.pose.orientation.w = 1.0
                inflated_radius = (
                    obstacle.radius
                    + self.drone_collision_radius
                    + self.collision_safety_margin
                )
                inflated_diameter = 2.0 * inflated_radius
                inflated.scale.x = inflated_diameter
                inflated.scale.y = inflated_diameter
                inflated.scale.z = inflated_diameter
                if (
                    obstacle.obstacle_id == closest_obstacle_id
                    and nominal_safe is False
                    and self.last_safety_result is not None
                    and self.last_safety_result.critical_component == "QUAD"
                ):
                    inflated.color.r = 1.0
                    inflated.color.g = 0.05
                    inflated.color.b = 0.05
                    inflated.color.a = 0.28
                else:
                    inflated.color.r = 1.0
                    inflated.color.g = 0.75
                    inflated.color.b = 0.05
                    inflated.color.a = 0.18
                markers.markers.append(inflated)

                magnet_inflated = Marker()
                magnet_inflated.header.stamp = now
                magnet_inflated.header.frame_id = self.frame_id
                magnet_inflated.ns = "static_obstacle_inflated_magnet"
                magnet_inflated.id = obstacle_index
                magnet_inflated.type = Marker.SPHERE
                magnet_inflated.action = Marker.ADD
                magnet_inflated.pose.position = _make_point(obstacle.centre)
                magnet_inflated.pose.orientation.w = 1.0
                magnet_inflated_radius = (
                    obstacle.radius
                    + self.magnet_collision_radius
                    + self.magnet_collision_margin
                )
                magnet_inflated_diameter = 2.0 * magnet_inflated_radius
                magnet_inflated.scale.x = magnet_inflated_diameter
                magnet_inflated.scale.y = magnet_inflated_diameter
                magnet_inflated.scale.z = magnet_inflated_diameter
                if (
                    obstacle.obstacle_id == closest_obstacle_id
                    and nominal_safe is False
                    and self.last_safety_result is not None
                    and self.last_safety_result.critical_component == "MAGNET"
                ):
                    magnet_inflated.color.r = 1.0
                    magnet_inflated.color.g = 0.05
                    magnet_inflated.color.b = 0.05
                    magnet_inflated.color.a = 0.28
                else:
                    magnet_inflated.color.r = 0.70
                    magnet_inflated.color.g = 0.10
                    magnet_inflated.color.b = 1.0
                    magnet_inflated.color.a = 0.12
                markers.markers.append(magnet_inflated)

            safety_result = self.last_safety_result
            if (
                safety_result is not None
                and safety_result.closest_point is not None
                and safety_result.closest_obstacle_centre is not None
            ):
                closest_point = safety_result.closest_point
                obstacle_centre = safety_result.closest_obstacle_centre
                line_red = 1.0
                line_green = 0.05 if nominal_safe is False else 0.85
                line_blue = 0.05

                approach_line = Marker()
                approach_line.header.stamp = now
                approach_line.header.frame_id = self.frame_id
                approach_line.ns = "minimum_clearance_line"
                approach_line.id = 0
                approach_line.type = Marker.LINE_LIST
                approach_line.action = Marker.ADD
                approach_line.scale.x = 0.022
                approach_line.color.r = line_red
                approach_line.color.g = line_green
                approach_line.color.b = line_blue
                approach_line.color.a = 1.0
                approach_line.points = [
                    _make_point(obstacle_centre),
                    _make_point(closest_point),
                ]
                markers.markers.append(approach_line)

                closest_point_marker = Marker()
                closest_point_marker.header.stamp = now
                closest_point_marker.header.frame_id = self.frame_id
                closest_point_marker.ns = "minimum_clearance_point"
                closest_point_marker.id = 0
                closest_point_marker.type = Marker.SPHERE
                closest_point_marker.action = Marker.ADD
                closest_point_marker.pose.position = _make_point(closest_point)
                closest_point_marker.pose.orientation.w = 1.0
                closest_point_marker.scale.x = 0.08
                closest_point_marker.scale.y = 0.08
                closest_point_marker.scale.z = 0.08
                closest_point_marker.color.r = line_red
                closest_point_marker.color.g = line_green
                closest_point_marker.color.b = line_blue
                closest_point_marker.color.a = 1.0
                markers.markers.append(closest_point_marker)

                clearance_label = Marker()
                clearance_label.header.stamp = now
                clearance_label.header.frame_id = self.frame_id
                clearance_label.ns = "minimum_clearance_text"
                clearance_label.id = 0
                clearance_label.type = Marker.TEXT_VIEW_FACING
                clearance_label.action = Marker.ADD
                label_position = 0.5 * (closest_point + obstacle_centre)
                label_position = label_position + np.array([0.0, 0.0, 0.10])
                clearance_label.pose.position = _make_point(label_position)
                clearance_label.pose.orientation.w = 1.0
                clearance_label.scale.z = 0.09
                clearance_label.color.r = line_red
                clearance_label.color.g = line_green
                clearance_label.color.b = line_blue
                clearance_label.color.a = 1.0
                clearance = safety_result.minimum_clearance
                clearance_text = (
                    "unknown"
                    if clearance is None
                    else f"{clearance:.3f} m"
                )
                label_lines = [
                    safety_result.closest_obstacle_id or "obstacle",
                    f"component={safety_result.critical_component or 'unknown'}",
                    f"clearance={clearance_text}",
                ]
                if safety_result.closest_time_s is not None:
                    label_lines.append(f"t={safety_result.closest_time_s:.2f} s")
                clearance_label.text = "\n".join(label_lines)
                markers.markers.append(clearance_label)

        if self.inputs_ready_for_status():
            try:
                target = self.target_state(self.current_phase_spec().target_source)
                tip_target = target.position + self.desired_tip_offset()
                tip_now = self.current_magnet_tip_position()
                quad_target = tip_target + self.quad_reference_offset_from_tip()

                target_marker = Marker()
                target_marker.header.stamp = now
                target_marker.header.frame_id = self.frame_id
                target_marker.ns = "desired_magnet_tip_target"
                target_marker.id = 0
                target_marker.type = Marker.SPHERE
                target_marker.action = Marker.ADD
                target_marker.pose.position = _make_point(tip_target)
                target_marker.pose.orientation.w = 1.0
                target_marker.scale.x = target_marker.scale.y = target_marker.scale.z = 0.12
                target_marker.color.r = 0.0
                target_marker.color.g = 1.0
                target_marker.color.b = 1.0
                target_marker.color.a = 1.0
                markers.markers.append(target_marker)

                current_tip = Marker()
                current_tip.header.stamp = now
                current_tip.header.frame_id = self.frame_id
                current_tip.ns = "current_magnet_tip"
                current_tip.id = 0
                current_tip.type = Marker.SPHERE
                current_tip.action = Marker.ADD
                current_tip.pose.position = _make_point(tip_now)
                current_tip.pose.orientation.w = 1.0
                current_tip.scale.x = current_tip.scale.y = current_tip.scale.z = 0.09
                current_tip.color.r = 1.0
                current_tip.color.g = 0.0
                current_tip.color.b = 1.0
                current_tip.color.a = 1.0
                markers.markers.append(current_tip)

                cable = Marker()
                cable.header.stamp = now
                cable.header.frame_id = self.frame_id
                cable.ns = "desired_cable_line"
                cable.id = 0
                cable.type = Marker.LINE_STRIP
                cable.action = Marker.ADD
                cable.scale.x = 0.018
                cable.color.r = cable.color.g = cable.color.b = 0.8
                cable.color.a = 1.0
                cable.points = [_make_point(quad_target), _make_point(tip_target)]
                markers.markers.append(cable)
            except RuntimeError:
                pass

        if (
            self.pickup_geometry_markers_active()
            and self.pickup_geometry_diagnostics_available
            and self.last_pickup_target_geometry is not None
            and self.last_fixed_clearance_pickup_target is not None
            and self.last_geometry_pickup_target_delta is not None
        ):
            pickup_geometry = self.last_pickup_target_geometry

            fixed_target = Marker()
            fixed_target.header.stamp = now
            fixed_target.header.frame_id = self.frame_id
            fixed_target.ns = "pickup_fixed_clearance_target"
            fixed_target.id = 0
            fixed_target.type = Marker.SPHERE
            fixed_target.action = Marker.ADD
            fixed_target.pose.position = _make_point(
                self.last_fixed_clearance_pickup_target
            )
            fixed_target.pose.orientation.w = 1.0
            fixed_target.scale.x = fixed_target.scale.y = fixed_target.scale.z = 0.07
            fixed_target.color.r = 0.95
            fixed_target.color.g = 0.20
            fixed_target.color.b = 0.95
            fixed_target.color.a = 0.95
            markers.markers.append(fixed_target)

            pickup_point = Marker()
            pickup_point.header.stamp = now
            pickup_point.header.frame_id = self.frame_id
            pickup_point.ns = "pickup_point_configured"
            pickup_point.id = 0
            pickup_point.type = Marker.SPHERE
            pickup_point.action = Marker.ADD
            pickup_point.pose.position = _make_point(
                pickup_geometry.pickup_point_world
            )
            pickup_point.pose.orientation.w = 1.0
            pickup_point.scale.x = pickup_point.scale.y = pickup_point.scale.z = 0.055
            pickup_point.color.r = 1.00
            pickup_point.color.g = 0.85
            pickup_point.color.b = 0.05
            pickup_point.color.a = 1.0
            markers.markers.append(pickup_point)

            face_target = Marker()
            face_target.header.stamp = now
            face_target.header.frame_id = self.frame_id
            face_target.ns = "pickup_face_target_geometry"
            face_target.id = 0
            face_target.type = Marker.SPHERE
            face_target.action = Marker.ADD
            face_target.pose.position = _make_point(
                pickup_geometry.contact_face_target_world
            )
            face_target.pose.orientation.w = 1.0
            face_target.scale.x = face_target.scale.y = face_target.scale.z = 0.06
            face_target.color.r = 1.00
            face_target.color.g = 0.45
            face_target.color.b = 0.05
            face_target.color.a = 1.0
            markers.markers.append(face_target)

            marker_target = Marker()
            marker_target.header.stamp = now
            marker_target.header.frame_id = self.frame_id
            marker_target.ns = "pickup_marker_target_geometry"
            marker_target.id = 0
            marker_target.type = Marker.SPHERE
            marker_target.action = Marker.ADD
            marker_target.pose.position = _make_point(
                pickup_geometry.magnet_marker_target_world
            )
            marker_target.pose.orientation.w = 1.0
            marker_target.scale.x = marker_target.scale.y = marker_target.scale.z = 0.085
            marker_target.color.r = 0.10
            marker_target.color.g = 1.00
            marker_target.color.b = 0.35
            marker_target.color.a = 1.0
            markers.markers.append(marker_target)

            if self.last_geometry_pickup_approach_target is not None:
                approach_target = Marker()
                approach_target.header.stamp = now
                approach_target.header.frame_id = self.frame_id
                approach_target.ns = "pickup_marker_approach_target_geometry"
                approach_target.id = 0
                approach_target.type = Marker.SPHERE
                approach_target.action = Marker.ADD
                approach_target.pose.position = _make_point(
                    self.last_geometry_pickup_approach_target
                )
                approach_target.pose.orientation.w = 1.0
                approach_target.scale.x = approach_target.scale.y = approach_target.scale.z = 0.075
                approach_target.color.r = 0.15
                approach_target.color.g = 0.55
                approach_target.color.b = 1.00
                approach_target.color.a = 0.95
                markers.markers.append(approach_target)

            target_difference = Marker()
            target_difference.header.stamp = now
            target_difference.header.frame_id = self.frame_id
            target_difference.ns = "pickup_target_difference"
            target_difference.id = 0
            target_difference.type = Marker.LINE_LIST
            target_difference.action = Marker.ADD
            target_difference.scale.x = 0.015
            target_difference.color.r = 0.20
            target_difference.color.g = 1.00
            target_difference.color.b = 0.90
            target_difference.color.a = 1.0
            target_difference.points = [
                _make_point(self.last_fixed_clearance_pickup_target),
                _make_point(pickup_geometry.magnet_marker_target_world),
            ]
            markers.markers.append(target_difference)

            marker_to_face = Marker()
            marker_to_face.header.stamp = now
            marker_to_face.header.frame_id = self.frame_id
            marker_to_face.ns = "pickup_marker_to_face_vector"
            marker_to_face.id = 0
            marker_to_face.type = Marker.LINE_LIST
            marker_to_face.action = Marker.ADD
            marker_to_face.scale.x = 0.012
            marker_to_face.color.r = 0.20
            marker_to_face.color.g = 0.80
            marker_to_face.color.b = 1.00
            marker_to_face.color.a = 1.0
            marker_to_face.points = [
                _make_point(pickup_geometry.magnet_marker_target_world),
                _make_point(pickup_geometry.contact_face_target_world),
            ]
            markers.markers.append(marker_to_face)

            if self.last_active_geometry_pickup_target is not None:
                active_geometry_target = Marker()
                active_geometry_target.header.stamp = now
                active_geometry_target.header.frame_id = self.frame_id
                active_geometry_target.ns = "pickup_geometry_active_target"
                active_geometry_target.id = 0
                active_geometry_target.type = Marker.SPHERE
                active_geometry_target.action = Marker.ADD
                active_geometry_target.pose.position = _make_point(
                    self.last_active_geometry_pickup_target
                )
                active_geometry_target.pose.orientation.w = 1.0
                active_geometry_target.scale.x = active_geometry_target.scale.y = active_geometry_target.scale.z = 0.10
                active_geometry_target.color.r = 0.05
                active_geometry_target.color.g = 1.00
                active_geometry_target.color.b = 0.05
                active_geometry_target.color.a = 0.55
                markers.markers.append(active_geometry_target)

        if self.show_debug_text:
            convex_status = (
                "waiting"
                if self.last_visual_astar_result.convex_trajectory is None
                else self.last_visual_astar_result.convex_trajectory.status
            )
            label = Marker()
            label.header.stamp = now
            label.header.frame_id = self.frame_id
            label.ns = "planner_phase_summary"
            label.id = 0
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position = _make_point(
                reference.positions[-1] + np.array([0.0, 0.0, 0.25])
            )
            label.pose.orientation.w = 1.0
            label.scale.z = 0.11
            label.color.r = 1.0
            label.color.g = 0.5
            label.color.b = 0.0
            label.color.a = 1.0
            spec = self.current_phase_spec()
            label.text = (
                f"{self.phase.value}\n"
                f"{spec.reference_type.value}\n"
                f"target={spec.target_source.value}\n"
                f"geometry_pickup={self.geometry_aware_pickup_mode}\n"
                f"A*={self.last_visual_astar_result.status}\n"
                f"convex={convex_status}"
            )
            markers.markers.append(label)

        self.marker_publisher.publish(markers)

    # ----------------------------------------------------------------------
    # Minimal report-ready logging
    # ----------------------------------------------------------------------
    def initialise_csv_logger(self) -> None:
        if not self.enable_csv_logging:
            return
        try:
            timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            # Keep join-planner CSVs directly under logs/join_planner. The
            # existing analysis/join_planner_logs/plot_join_planner_log.py uses
            # a non-recursive logs/join_planner/*.csv discovery glob.
            run_directory = self.log_directory
            os.makedirs(run_directory, exist_ok=True)
            filename = f"join_planner_clean_{timestamp}_{self.vehicle_id}.csv"
            self.csv_path = os.path.join(run_directory, filename)
            self.csv_file = open(self.csv_path, "w", newline="")
            self.csv_writer = csv.DictWriter(
                self.csv_file,
                fieldnames=[
                    "time_sec",
                    "vehicle_id",
                    "assigned_attachment_id",
                    "mission",
                    "phase",
                    "reference_type",
                    "target_source",
                    "controller_authority",
                    "handoff_ready",
                    "object_attached",
                    "collision_diagnostics_enabled",
                    "static_obstacle_count",
                    "drone_collision_radius_m",
                    "collision_safety_margin_m",
                    "magnet_collision_radius_m",
                    "magnet_collision_margin_m",
                    "cable_collision_radius_m",
                    "cable_collision_margin_m",
                    "payload_profile_name",
                    "payload_collision_enabled",
                    "payload_collision_size_x",
                    "payload_collision_size_y",
                    "payload_collision_size_z",
                    "payload_collision_margin_m",
                    "payload_pose_reference",
                    "payload_yaw_rad",
                    "payload_geometry_source",
                    "magnet_drop_below_quad_m",
                    "pickup_object_x",
                    "pickup_object_y",
                    "pickup_object_z",
                    "pickup_object_pose_age_s",
                    "lift_object_clearance_m",
                    "loaded_lift_z_tolerance_m",
                    "loaded_lift_quad_z_error_m",
                    "loaded_lift_profile_complete",
                    "loaded_lift_object_airborne",
                    "loaded_lift_ready_for_cpp",
                    "operator_override_used",
                    "operator_override_request_count",
                    "operator_override_accept_count",
                    "operator_override_last_from_phase",
                    "operator_override_last_action",
                    "operator_override_last_result",
                    "pickup_geometry_diagnostics_available",
                    "pickup_geometry_diagnostics_source",
                    "pickup_geometry_pose_age_s",
                    "geometry_aware_pickup_enabled",
                    "geometry_aware_pickup_active",
                    "geometry_aware_pickup_mode",
                    "geometry_aware_pickup_fallback_reason",
                    "geometry_pickup_latched",
                    "geometry_pickup_latch_source",
                    "geometry_pickup_approach_height_m",
                    "geometry_pickup_active_target_x",
                    "geometry_pickup_active_target_y",
                    "geometry_pickup_active_target_z",
                    "pickup_contact_gap_m",
                    "pickup_overtravel_requested_m",
                    "pickup_overtravel_used_m",
                    "pickup_max_overtravel_m",
                    "pickup_support_surface_z_m",
                    "pickup_support_surface_clamped",
                    "pickup_fixed_target_x",
                    "pickup_fixed_target_y",
                    "pickup_fixed_target_z",
                    "pickup_point_world_x",
                    "pickup_point_world_y",
                    "pickup_point_world_z",
                    "pickup_face_target_x",
                    "pickup_face_target_y",
                    "pickup_face_target_z",
                    "pickup_marker_target_x",
                    "pickup_marker_target_y",
                    "pickup_marker_target_z",
                    "pickup_target_delta_x",
                    "pickup_target_delta_y",
                    "pickup_target_delta_z",
                    "pickup_target_delta_norm_m",
                    "pickup_geometry_diagnostics_error",
                    "c1f1_shadow_enabled_config",
                    "c1f1_shadow_phase_active",
                    "c1f1_shadow_commanded_enable",
                    "c1f1_shadow_latched_target_x",
                    "c1f1_shadow_latched_target_y",
                    "c1f1_shadow_latched_target_z",
                    "c1f1_shadow_live_target_x",
                    "c1f1_shadow_live_target_y",
                    "c1f1_shadow_live_target_z",
                    "c1f1_shadow_target_drift_m",
                    "c1f1_shadow_reference_age_ms",
                    "c1f1_shadow_reference_fresh",
                    "c1f1_shadow_reference_count",
                    "c1f1_shadow_reference_sample_count",
                    "c1f1_shadow_reference_period_ms",
                    "c1f1_shadow_reference_terminal_x",
                    "c1f1_shadow_reference_terminal_y",
                    "c1f1_shadow_reference_terminal_z",
                    "c1f1_shadow_status_age_ms",
                    "c1f1_shadow_backend_status",
                    "c1f2_cpp_authority_enabled_config",
                    "c1f2_cpp_phase_active",
                    "c1f2_prepare_active",
                    "c1f2_prepare_settled",
                    "c1f2_prepare_drift_m",
                    "c1f2_prepare_speed_mps",
                    "c1f2_handoff_ready",
                    "c1f2_prepare_count",
                    "c1f2_prepare_wait_s",
                    "c1f2_cpp_authority_active",
                    "c1f2_commanded_authority",
                    "c1f2_authority_grant_pending",
                    "c1f2_authority_acknowledged",
                    "c1f2_authority_grant_count",
                    "c1f2_authority_switch_count",
                    "c1f2_stale_fallback_count",
                    "c1f2_cached_continuation_active",
                    "c1f2_cached_continuation_count",
                    "c1f2_recovery_reject_count",
                    "c1f2_exit_switch_count",
                    "c1f2_exit_bridge_active",
                    "c1f2_exit_bridge_complete",
                    "c1f2_exit_bridge_elapsed_s",
                    "c1f2_exit_bridge_duration_s",
                    "c1f2_exit_bridge_feasible",
                    "c1f2_exit_bridge_feasibility_reason",
                    "c1f2_exit_bridge_peak_velocity_mps",
                    "c1f2_exit_bridge_peak_acceleration_mps2",
                    "c1f2_exit_bridge_peak_jerk_mps3",
                    "c1f2_exit_bridge_initial_position_error_m",
                    "c1f2_exit_bridge_initial_velocity_error_mps",
                    "c1f2_exit_bridge_initial_acceleration_error_mps2",
                    "c1f2_cached_reference_age_ms",
                    "c1f2_handoff_position_error_m",
                    "c1f2_handoff_velocity_error_mps",
                    "c1f2_handoff_acceleration_error_mps2",
                    "m2b_capture_settle_position_error_m",
                    "m2b_capture_settle_speed_mps",
                    "m2b_capture_settle_swing_deg",
                    "m2b_capture_settle_dwell_s",
                    "m2b_capture_settle_reason",
                    "m2b_capture_settle_settled",
                    "visual_astar_enabled",
                    "visual_astar_phase_allowed",
                    "visual_astar_job_pending",
                    "visual_astar_request_id",
                    "visual_astar_status",
                    "visual_astar_message",
                    "visual_astar_success",
                    "visual_astar_nominal_blocked",
                    "visual_astar_first_blocked_segment_index",
                    "visual_astar_rejoin_index",
                    "visual_astar_rejoin_time_s",
                    "visual_astar_rejoin_candidate_count",
                    "visual_astar_resolution_m",
                    "visual_astar_replan_rate_hz",
                    "visual_astar_planning_time_ms",
                    "visual_astar_worker_turnaround_ms",
                    "visual_astar_expanded_nodes",
                    "visual_astar_generated_nodes",
                    "visual_astar_raw_waypoint_count",
                    "visual_astar_simplified_waypoint_count",
                    "visual_astar_candidate_sample_count",
                    "visual_astar_raw_path_length_m",
                    "visual_astar_simplified_path_length_m",
                    "visual_astar_start_adjustment_m",
                    "visual_astar_goal_adjustment_m",
                    "visual_astar_start_requested_x",
                    "visual_astar_start_requested_y",
                    "visual_astar_start_requested_z",
                    "visual_astar_start_used_x",
                    "visual_astar_start_used_y",
                    "visual_astar_start_used_z",
                    "visual_astar_goal_requested_x",
                    "visual_astar_goal_requested_y",
                    "visual_astar_goal_requested_z",
                    "visual_astar_goal_used_x",
                    "visual_astar_goal_used_y",
                    "visual_astar_goal_used_z",
                    "visual_astar_bounds_min_x",
                    "visual_astar_bounds_min_y",
                    "visual_astar_bounds_min_z",
                    "visual_astar_bounds_max_x",
                    "visual_astar_bounds_max_y",
                    "visual_astar_bounds_max_z",
                    "visual_astar_grid_x",
                    "visual_astar_grid_y",
                    "visual_astar_grid_z",
                    "visual_astar_grid_node_count",
                    "visual_astar_candidate_whole_body_safe",
                    "visual_astar_candidate_critical_component",
                    "visual_astar_candidate_minimum_clearance_m",
                    "visual_astar_candidate_quad_clearance_m",
                    "visual_astar_candidate_magnet_clearance_m",
                    "visual_astar_candidate_cable_clearance_m",
                    "visual_astar_candidate_payload_clearance_m",
                    "visual_astar_candidate_payload_geometry_available",

                    "visual_astar_corridor_valid",
                    "visual_astar_corridor_segment_min_margin_m",
                    "visual_astar_corridor_overlap_min_margin_m",
                    "visual_astar_corridor_voxel_exclusion_min_margin_m",
                    "visual_astar_corridor_error",

                    "visual_convex_trajectory_status",
                    "visual_convex_trajectory_message",
                    "visual_convex_trajectory_success",
                    "visual_convex_trajectory_solve_time_ms",
                    "visual_convex_trajectory_qp_solve_count",
                    "visual_convex_trajectory_solver_iterations",
                    "visual_convex_trajectory_solver_message",
                    "visual_convex_trajectory_segment_count",
                    "visual_convex_trajectory_control_point_count",
                    "visual_convex_trajectory_total_duration_s",
                    "visual_convex_trajectory_path_length_m",
                    "visual_convex_trajectory_route_length_ratio",
                    "visual_convex_trajectory_retiming_used",
                    "visual_convex_trajectory_peak_speed_mps",
                    "visual_convex_trajectory_peak_acceleration_mps2",
                    "visual_convex_trajectory_velocity_control_bound_mps",
                    "visual_convex_trajectory_acceleration_control_bound_mps2",
                    "visual_convex_trajectory_validation_speed_limit_mps",
                    "visual_convex_trajectory_validation_acceleration_limit_mps2",
                    "visual_convex_trajectory_corridor_violation_m",
                    "visual_convex_trajectory_boundary_position_error_m",
                    "visual_convex_trajectory_boundary_velocity_error_mps",
                    "visual_convex_trajectory_boundary_acceleration_error_mps2",
                    "visual_convex_trajectory_join_position_error_m",
                    "visual_convex_trajectory_join_velocity_error_mps",
                    "visual_convex_trajectory_join_acceleration_error_mps2",
                    "visual_convex_trajectory_join_jerk_error_mps3",

                    "visual_astar_completed_count",
                    "visual_astar_discarded_count",
                    "visual_trajectory_enabled",
                    "visual_trajectory_clearance_margin_m",
                    "safety_action",
                    "safety_intervention_active",
                    "safety_nominal_safe",
                    "safety_critical_component",
                    "safety_minimum_clearance_m",
                    "safety_quad_minimum_clearance_m",
                    "safety_magnet_minimum_clearance_m",
                    "safety_cable_minimum_clearance_m",
                    "safety_payload_check_active",
                    "safety_payload_geometry_available",
                    "safety_payload_pose_age_s",
                    "safety_payload_minimum_clearance_m",
                    "safety_payload_closest_obstacle_id",
                    "safety_payload_closest_sample_index",
                    "safety_payload_closest_time_s",
                    "safety_payload_closest_point_x",
                    "safety_payload_closest_point_y",
                    "safety_payload_closest_point_z",
                    "safety_payload_offset_from_magnet_x",
                    "safety_payload_offset_from_magnet_y",
                    "safety_payload_offset_from_magnet_z",
                    "safety_predicted_payload_stage0_x",
                    "safety_predicted_payload_stage0_y",
                    "safety_predicted_payload_stage0_z",
                    "safety_predicted_payload_terminal_x",
                    "safety_predicted_payload_terminal_y",
                    "safety_predicted_payload_terminal_z",
                    "safety_closest_obstacle_id",
                    "safety_closest_segment_index",
                    "safety_closest_time_s",
                    "safety_closest_point_x",
                    "safety_closest_point_y",
                    "safety_closest_point_z",
                    "safety_closest_obstacle_centre_x",
                    "safety_closest_obstacle_centre_y",
                    "safety_closest_obstacle_centre_z",
                    "safety_magnet_offset_x",
                    "safety_magnet_offset_y",
                    "safety_magnet_offset_z",
                    "safety_predicted_magnet_stage0_x",
                    "safety_predicted_magnet_stage0_y",
                    "safety_predicted_magnet_stage0_z",
                    "safety_predicted_magnet_terminal_x",
                    "safety_predicted_magnet_terminal_y",
                    "safety_predicted_magnet_terminal_z",
                    "safety_cable_closest_obstacle_id",
                    "safety_cable_closest_sample_index",
                    "safety_cable_closest_time_s",
                    "safety_cable_closest_point_x",
                    "safety_cable_closest_point_y",
                    "safety_cable_closest_point_z",
                    "safety_cable_closest_quad_x",
                    "safety_cable_closest_quad_y",
                    "safety_cable_closest_quad_z",
                    "safety_cable_closest_magnet_x",
                    "safety_cable_closest_magnet_y",
                    "safety_cable_closest_magnet_z",
                    "safety_planning_time_ms",
                    "planner_callback_time_ms",
                    "planner_callback_period_ms",
                    "planner_callback_budget_ms",
                    "planner_deadline_miss_count",
                    "quad_x",
                    "quad_y",
                    "quad_z",
                    "target_x",
                    "target_y",
                    "target_z",
                    "tip_target_x",
                    "tip_target_y",
                    "tip_target_z",
                    "tip_actual_x",
                    "tip_actual_y",
                    "tip_actual_z",
                    "tip_error_xy",
                    "tip_error_z",
                    "relative_speed_xy",
                    "ref_stage0_x",
                    "ref_stage0_y",
                    "ref_stage0_z",
                    "ref_terminal_x",
                    "ref_terminal_y",
                    "ref_terminal_z",
                    "phase_elapsed_s",
                    "rate_profile_duration_s",
                    "rate_profile_complete",
                    "transfer_duration_s",
                    "transfer_progress_scale",
                    "pickup_compensation_x",
                    "pickup_compensation_y",
                    "last_transition_reason",
                ],
            )
            self.csv_writer.writeheader()
            self.csv_file.flush()
        except OSError as exc:
            self.csv_file = None
            self.csv_writer = None
            self.csv_path = ""
            self.get_logger().error(f"Could not open planner CSV log: {exc}")

    def write_csv_log(self, reference: TrajectoryReference) -> None:
        if self.csv_writer is None or self.csv_file is None:
            return
        self.update_counter += 1
        if (self.update_counter - 1) % self.log_every_n_updates != 0:
            return
        if self.drone_position is None:
            return

        spec = self.current_phase_spec()
        target = np.full(3, np.nan)
        tip_target = np.full(3, np.nan)
        tip_actual = np.full(3, np.nan)
        xy_error = float("nan")
        z_error = float("nan")
        relative_speed = float("nan")

        pickup_fixed_target = np.full(3, np.nan)
        pickup_point_world = np.full(3, np.nan)
        pickup_face_target = np.full(3, np.nan)
        pickup_marker_target = np.full(3, np.nan)
        pickup_target_delta = np.full(3, np.nan)
        pickup_target_delta_norm = float("nan")
        pickup_overtravel_used = float("nan")
        pickup_support_surface_clamped = ""
        geometry_pickup_active_target = np.full(3, np.nan)
        if self.last_active_geometry_pickup_target is not None:
            geometry_pickup_active_target = (
                self.last_active_geometry_pickup_target.copy()
            )
        if (
            self.pickup_geometry_diagnostics_available
            and self.last_pickup_target_geometry is not None
            and self.last_fixed_clearance_pickup_target is not None
            and self.last_geometry_pickup_target_delta is not None
        ):
            pickup_geometry = self.last_pickup_target_geometry
            pickup_fixed_target = self.last_fixed_clearance_pickup_target.copy()
            pickup_point_world = pickup_geometry.pickup_point_world.copy()
            pickup_face_target = pickup_geometry.contact_face_target_world.copy()
            pickup_marker_target = pickup_geometry.magnet_marker_target_world.copy()
            pickup_target_delta = self.last_geometry_pickup_target_delta.copy()
            pickup_target_delta_norm = float(np.linalg.norm(pickup_target_delta))
            pickup_overtravel_used = pickup_geometry.used_overtravel
            pickup_support_surface_clamped = str(
                bool(pickup_geometry.support_surface_clamped)
            ).lower()

        safety_result = self.last_safety_result
        safety_action = ""
        safety_intervention_active = ""
        safety_nominal_safe = ""
        safety_critical_component = ""
        safety_minimum_clearance = ""
        safety_quad_minimum_clearance = ""
        safety_magnet_minimum_clearance = ""
        safety_cable_minimum_clearance = ""
        safety_payload_check_active = ""
        safety_payload_geometry_available = ""
        safety_payload_pose_age_s = ""
        safety_payload_minimum_clearance = ""
        safety_payload_closest_obstacle_id = ""
        safety_payload_closest_sample_index = ""
        safety_payload_closest_time_s = ""
        safety_payload_closest_point = np.full(3, np.nan)
        safety_payload_offset_from_magnet = np.full(3, np.nan)
        safety_predicted_payload_stage0 = np.full(3, np.nan)
        safety_predicted_payload_terminal = np.full(3, np.nan)
        safety_closest_obstacle_id = ""
        safety_closest_segment_index = ""
        safety_closest_time_s = ""
        safety_closest_point = np.full(3, np.nan)
        safety_closest_obstacle_centre = np.full(3, np.nan)
        safety_magnet_offset = np.full(3, np.nan)
        safety_predicted_magnet_stage0 = np.full(3, np.nan)
        safety_predicted_magnet_terminal = np.full(3, np.nan)
        safety_cable_closest_obstacle_id = ""
        safety_cable_closest_sample_index = ""
        safety_cable_closest_time_s = ""
        safety_cable_closest_point = np.full(3, np.nan)
        safety_cable_closest_quad = np.full(3, np.nan)
        safety_cable_closest_magnet = np.full(3, np.nan)
        safety_planning_time_ms = ""
        if safety_result is not None:
            safety_action = safety_result.action.value
            safety_intervention_active = str(
                bool(safety_result.intervention_active)
            ).lower()
            if safety_result.nominal_safe is not None:
                safety_nominal_safe = str(bool(safety_result.nominal_safe)).lower()
            if safety_result.critical_component is not None:
                safety_critical_component = safety_result.critical_component
            if safety_result.minimum_clearance is not None:
                safety_minimum_clearance = safety_result.minimum_clearance
            if safety_result.quad_minimum_clearance is not None:
                safety_quad_minimum_clearance = (
                    safety_result.quad_minimum_clearance
                )
            if safety_result.magnet_minimum_clearance is not None:
                safety_magnet_minimum_clearance = (
                    safety_result.magnet_minimum_clearance
                )
            if safety_result.cable_minimum_clearance is not None:
                safety_cable_minimum_clearance = (
                    safety_result.cable_minimum_clearance
                )
            safety_payload_check_active = str(
                bool(safety_result.payload_check_active)
            ).lower()
            safety_payload_geometry_available = str(
                bool(safety_result.payload_geometry_available)
            ).lower()
            if safety_result.payload_pose_age_s is not None:
                safety_payload_pose_age_s = safety_result.payload_pose_age_s
            if safety_result.payload_minimum_clearance is not None:
                safety_payload_minimum_clearance = (
                    safety_result.payload_minimum_clearance
                )
            if safety_result.payload_closest_obstacle_id is not None:
                safety_payload_closest_obstacle_id = (
                    safety_result.payload_closest_obstacle_id
                )
            if safety_result.payload_closest_sample_index is not None:
                safety_payload_closest_sample_index = (
                    safety_result.payload_closest_sample_index
                )
            if safety_result.payload_closest_time_s is not None:
                safety_payload_closest_time_s = (
                    safety_result.payload_closest_time_s
                )
            if safety_result.payload_closest_point is not None:
                safety_payload_closest_point = safety_result.payload_closest_point
            if safety_result.payload_offset_from_magnet is not None:
                safety_payload_offset_from_magnet = (
                    safety_result.payload_offset_from_magnet
                )
            if safety_result.predicted_payload_positions is not None:
                safety_predicted_payload_stage0 = (
                    safety_result.predicted_payload_positions[0]
                )
                safety_predicted_payload_terminal = (
                    safety_result.predicted_payload_positions[-1]
                )
            if safety_result.closest_obstacle_id is not None:
                safety_closest_obstacle_id = safety_result.closest_obstacle_id
            if safety_result.closest_segment_index is not None:
                safety_closest_segment_index = safety_result.closest_segment_index
            if safety_result.closest_time_s is not None:
                safety_closest_time_s = safety_result.closest_time_s
            if safety_result.closest_point is not None:
                safety_closest_point = safety_result.closest_point
            if safety_result.closest_obstacle_centre is not None:
                safety_closest_obstacle_centre = (
                    safety_result.closest_obstacle_centre
                )
            if safety_result.predicted_magnet_positions is not None:
                safety_predicted_magnet_stage0 = (
                    safety_result.predicted_magnet_positions[0]
                )
                safety_predicted_magnet_terminal = (
                    safety_result.predicted_magnet_positions[-1]
                )
                safety_magnet_offset = (
                    safety_result.predicted_magnet_positions[0]
                    - reference.positions[0]
                )
            if safety_result.cable_closest_obstacle_id is not None:
                safety_cable_closest_obstacle_id = (
                    safety_result.cable_closest_obstacle_id
                )
            if safety_result.cable_closest_sample_index is not None:
                safety_cable_closest_sample_index = (
                    safety_result.cable_closest_sample_index
                )
            if safety_result.cable_closest_time_s is not None:
                safety_cable_closest_time_s = safety_result.cable_closest_time_s
            if safety_result.cable_closest_point is not None:
                safety_cable_closest_point = safety_result.cable_closest_point
            if safety_result.cable_closest_quad_position is not None:
                safety_cable_closest_quad = (
                    safety_result.cable_closest_quad_position
                )
            if safety_result.cable_closest_magnet_position is not None:
                safety_cable_closest_magnet = (
                    safety_result.cable_closest_magnet_position
                )
            safety_planning_time_ms = safety_result.planning_time_ms

        astar = self.last_visual_astar_result
        trajectory_log = self.build_visual_trajectory_log()

        if self.inputs_ready_for_status():
            try:
                target_state = self.target_state(spec.target_source)
                target = target_state.position
                tip_target = target + self.desired_tip_offset()
                tip_actual = self.current_magnet_tip_position()
                xy_error, relative_speed, z_error = self.tracking_errors()
            except RuntimeError:
                pass

        pickup_object_log = np.full(3, np.nan)
        if self.pickup_object_position is not None:
            pickup_object_log = self.pickup_object_position.copy()
        pickup_object_pose_age_s = float("nan")
        if self.last_pickup_object_pose_time is not None:
            pickup_object_pose_age_s = max(
                0.0,
                self._physical_now_s() - self.last_pickup_object_pose_time,
            )

        lift_object_clearance_m = float("nan")
        loaded_lift_quad_z_error_m = float("nan")
        loaded_lift_profile_complete = False
        loaded_lift_object_airborne = False
        loaded_lift_ready = False
        if self.phase == MissionPhase.LIFT_OBJECT:
            loaded_lift_profile_complete = self.rate_profile().complete(
                self.phase_elapsed()
            )
            if (
                self.latched_pickup_position is not None
                and self.pickup_object_position is not None
            ):
                lift_object_clearance_m = float(
                    self.pickup_object_position[2]
                    - self.latched_pickup_position[2]
                )
            loaded_lift_object_airborne = bool(
                self.object_attached
                and np.isfinite(lift_object_clearance_m)
                and lift_object_clearance_m
                >= self.lift_complete_object_clearance
            )
            loaded_lift_quad_z_error_m = self.loaded_lift_quad_z_error_m()
            loaded_lift_ready = loaded_lift_ready_for_cpp(
                object_attached=self.object_attached,
                object_airborne=loaded_lift_object_airborne,
                profile_complete=loaded_lift_profile_complete,
                abs_quad_z_error_m=abs(loaded_lift_quad_z_error_m),
                z_tolerance_m=self.loaded_lift_z_tolerance,
            )

        self.csv_writer.writerow(
            {
                "time_sec": time.time(),
                "vehicle_id": self.vehicle_id,
                "assigned_attachment_id": self.assigned_attachment_id,
                "mission": self.mission.name,
                "phase": self.phase.value,
                "reference_type": spec.reference_type.value,
                "target_source": spec.target_source.value,
                "controller_authority": spec.controller_authority.value,
                "handoff_ready": str(bool(spec.handoff_ready)).lower(),
                "object_attached": str(bool(self.object_attached)).lower(),
                "collision_diagnostics_enabled": str(
                    bool(self.collision_diagnostics_enabled)
                ).lower(),
                "static_obstacle_count": len(self.static_obstacles),
                "drone_collision_radius_m": self.drone_collision_radius,
                "collision_safety_margin_m": self.collision_safety_margin,
                "magnet_collision_radius_m": self.magnet_collision_radius,
                "magnet_collision_margin_m": self.magnet_collision_margin,
                "cable_collision_radius_m": self.cable_collision_radius,
                "cable_collision_margin_m": self.cable_collision_margin,
                "payload_profile_name": self.payload_profile.name,
                "payload_collision_enabled": str(
                    bool(self.payload_profile.collision_enabled)
                ).lower(),
                "payload_collision_size_x": self.payload_profile.dimensions[0],
                "payload_collision_size_y": self.payload_profile.dimensions[1],
                "payload_collision_size_z": self.payload_profile.dimensions[2],
                "payload_collision_margin_m": self.payload_profile.collision_margin,
                "payload_pose_reference": self.payload_profile.pose_reference,
                "payload_yaw_rad": self.pickup_object_yaw,
                "payload_geometry_source": self.attached_payload_geometry_source,
                "magnet_drop_below_quad_m": self.magnet_drop_below_quad,
                "pickup_object_x": pickup_object_log[0],
                "pickup_object_y": pickup_object_log[1],
                "pickup_object_z": pickup_object_log[2],
                "pickup_object_pose_age_s": pickup_object_pose_age_s,
                "lift_object_clearance_m": lift_object_clearance_m,
                "loaded_lift_z_tolerance_m": self.loaded_lift_z_tolerance,
                "loaded_lift_quad_z_error_m": loaded_lift_quad_z_error_m,
                "loaded_lift_profile_complete": str(
                    bool(loaded_lift_profile_complete)
                ).lower(),
                "loaded_lift_object_airborne": str(
                    bool(loaded_lift_object_airborne)
                ).lower(),
                "loaded_lift_ready_for_cpp": str(
                    bool(loaded_lift_ready)
                ).lower(),
                "operator_override_used": str(
                    bool(self.operator_override_used)
                ).lower(),
                "operator_override_request_count": self.operator_override_request_count,
                "operator_override_accept_count": self.operator_override_accept_count,
                "operator_override_last_from_phase": self.operator_override_last_from_phase,
                "operator_override_last_action": self.operator_override_last_action,
                "operator_override_last_result": self.operator_override_last_result,
                "pickup_geometry_diagnostics_available": str(
                    bool(self.pickup_geometry_diagnostics_available)
                ).lower(),
                "pickup_geometry_diagnostics_source": (
                    self.pickup_geometry_diagnostics_source
                ),
                "pickup_geometry_pose_age_s": (
                    ""
                    if self.pickup_geometry_pose_age_s is None
                    else self.pickup_geometry_pose_age_s
                ),
                "geometry_aware_pickup_enabled": str(
                    bool(self.use_geometry_aware_pickup)
                ).lower(),
                "geometry_aware_pickup_active": str(
                    bool(self.geometry_aware_pickup_active)
                ).lower(),
                "geometry_aware_pickup_mode": self.geometry_aware_pickup_mode,
                "geometry_aware_pickup_fallback_reason": (
                    self.geometry_aware_pickup_fallback_reason
                ),
                "geometry_pickup_latched": str(
                    bool(
                        self.latched_geometry_pickup_contact_target is not None
                        and self.latched_geometry_pickup_approach_target is not None
                    )
                ).lower(),
                "geometry_pickup_latch_source": self.geometry_pickup_latch_source,
                "geometry_pickup_approach_height_m": (
                    self.geometry_pickup_approach_height()
                ),
                "geometry_pickup_active_target_x": geometry_pickup_active_target[0],
                "geometry_pickup_active_target_y": geometry_pickup_active_target[1],
                "geometry_pickup_active_target_z": geometry_pickup_active_target[2],
                "pickup_contact_gap_m": self.pickup_contact_gap,
                "pickup_overtravel_requested_m": self.pickup_overtravel,
                "pickup_overtravel_used_m": pickup_overtravel_used,
                "pickup_max_overtravel_m": self.pickup_max_overtravel,
                "pickup_support_surface_z_m": self.support_surface_z,
                "pickup_support_surface_clamped": pickup_support_surface_clamped,
                "pickup_fixed_target_x": pickup_fixed_target[0],
                "pickup_fixed_target_y": pickup_fixed_target[1],
                "pickup_fixed_target_z": pickup_fixed_target[2],
                "pickup_point_world_x": pickup_point_world[0],
                "pickup_point_world_y": pickup_point_world[1],
                "pickup_point_world_z": pickup_point_world[2],
                "pickup_face_target_x": pickup_face_target[0],
                "pickup_face_target_y": pickup_face_target[1],
                "pickup_face_target_z": pickup_face_target[2],
                "pickup_marker_target_x": pickup_marker_target[0],
                "pickup_marker_target_y": pickup_marker_target[1],
                "pickup_marker_target_z": pickup_marker_target[2],
                "pickup_target_delta_x": pickup_target_delta[0],
                "pickup_target_delta_y": pickup_target_delta[1],
                "pickup_target_delta_z": pickup_target_delta[2],
                "pickup_target_delta_norm_m": pickup_target_delta_norm,
                "pickup_geometry_diagnostics_error": (
                    self.pickup_geometry_diagnostics_error
                ),
                "c1f1_shadow_enabled_config": str(
                    bool(self.c1f1_shadow_transfer_enabled)
                ).lower(),
                "c1f1_shadow_phase_active": str(
                    bool(self.c1f1_shadow_phase_active())
                ).lower(),
                "c1f1_shadow_commanded_enable": str(
                    bool(self.c1f1_shadow_commanded_enable)
                ).lower(),
                "c1f1_shadow_latched_target_x": (
                    np.nan
                    if self.c1f1_shadow_latched_body_target is None
                    else self.c1f1_shadow_latched_body_target[0]
                ),
                "c1f1_shadow_latched_target_y": (
                    np.nan
                    if self.c1f1_shadow_latched_body_target is None
                    else self.c1f1_shadow_latched_body_target[1]
                ),
                "c1f1_shadow_latched_target_z": (
                    np.nan
                    if self.c1f1_shadow_latched_body_target is None
                    else self.c1f1_shadow_latched_body_target[2]
                ),
                "c1f1_shadow_live_target_x": (
                    np.nan
                    if self.c1f1_shadow_live_body_target is None
                    else self.c1f1_shadow_live_body_target[0]
                ),
                "c1f1_shadow_live_target_y": (
                    np.nan
                    if self.c1f1_shadow_live_body_target is None
                    else self.c1f1_shadow_live_body_target[1]
                ),
                "c1f1_shadow_live_target_z": (
                    np.nan
                    if self.c1f1_shadow_live_body_target is None
                    else self.c1f1_shadow_live_body_target[2]
                ),
                "c1f1_shadow_target_drift_m": self.c1f1_shadow_target_drift_m,
                "c1f1_shadow_reference_age_ms": self.c1f1_shadow_reference_age_ms(),
                "c1f1_shadow_reference_fresh": str(
                    bool(self.c1f1_shadow_reference_fresh())
                ).lower(),
                "c1f1_shadow_reference_count": self.c1f1_shadow_reference_count,
                "c1f1_shadow_reference_sample_count": (
                    self.c1f1_shadow_reference_sample_count
                ),
                "c1f1_shadow_reference_period_ms": (
                    self.c1f1_shadow_reference_last_period_ms
                ),
                "c1f1_shadow_reference_terminal_x": (
                    self.c1f1_shadow_reference_terminal[0]
                ),
                "c1f1_shadow_reference_terminal_y": (
                    self.c1f1_shadow_reference_terminal[1]
                ),
                "c1f1_shadow_reference_terminal_z": (
                    self.c1f1_shadow_reference_terminal[2]
                ),
                "c1f1_shadow_status_age_ms": self.c1f1_shadow_status_age_ms(),
                "c1f1_shadow_backend_status": self.c1f1_shadow_backend_status,
                "c1f2_cpp_authority_enabled_config": str(
                    bool(self.c1f2_cpp_authority_enabled)
                ).lower(),
                "c1f2_cpp_phase_active": str(
                    bool(self.c1f2_cpp_phase_active())
                ).lower(),
                "c1f2_prepare_active": str(bool(self.c1f2_prepare_active)).lower(),
                "c1f2_prepare_settled": str(
                    bool(self.c1f2_prepare_settled)
                ).lower(),
                "c1f2_prepare_drift_m": self.c1f2_prepare_drift_m,
                "c1f2_prepare_speed_mps": self.c1f2_prepare_speed_mps,
                "c1f2_handoff_ready": str(bool(self.c1f2_handoff_ready)).lower(),
                "c1f2_prepare_count": self.c1f2_prepare_count,
                "c1f2_prepare_wait_s": self.c1f2_prepare_wait_s,
                "c1f2_cpp_authority_active": str(
                    bool(self.c1f2_cpp_authority_active)
                ).lower(),
                "c1f2_commanded_authority": str(
                    bool(self.c1f2_commanded_authority)
                ).lower(),
                "c1f2_authority_grant_pending": str(
                    bool(self.c1f2_authority_grant_pending)
                ).lower(),
                "c1f2_authority_acknowledged": str(
                    bool(self.c1f2_authority_acknowledged)
                ).lower(),
                "c1f2_authority_grant_count": self.c1f2_authority_grant_count,
                "c1f2_authority_switch_count": self.c1f2_authority_switch_count,
                "c1f2_stale_fallback_count": self.c1f2_stale_fallback_count,
                "c1f2_cached_continuation_active": str(
                    bool(self.c1f2_cached_continuation_active)
                ).lower(),
                "c1f2_cached_continuation_count": self.c1f2_cached_continuation_count,
                "c1f2_recovery_reject_count": self.c1f2_recovery_reject_count,
                "c1f2_exit_switch_count": self.c1f2_exit_switch_count,
                "c1f2_exit_bridge_active": str(
                    bool(self.c1f2_exit_bridge_active)
                ).lower(),
                "c1f2_exit_bridge_complete": str(
                    bool(self.c1f2_exit_bridge_complete)
                ).lower(),
                "c1f2_exit_bridge_elapsed_s": (
                    float("nan")
                    if self.c1f2_exit_bridge_start_time is None
                    else max(0.0, self._physical_now_s() - self.c1f2_exit_bridge_start_time)
                ),
                "c1f2_exit_bridge_duration_s": self.c1f2_exit_bridge_duration_s,
                "c1f2_exit_bridge_feasible": str(
                    bool(self.c1f2_exit_bridge_feasible)
                ).lower(),
                "c1f2_exit_bridge_feasibility_reason": (
                    self.c1f2_exit_bridge_feasibility_reason
                ),
                "c1f2_exit_bridge_peak_velocity_mps": (
                    self.c1f2_exit_bridge_peak_velocity_mps
                ),
                "c1f2_exit_bridge_peak_acceleration_mps2": (
                    self.c1f2_exit_bridge_peak_acceleration_mps2
                ),
                "c1f2_exit_bridge_peak_jerk_mps3": (
                    self.c1f2_exit_bridge_peak_jerk_mps3
                ),
                "c1f2_exit_bridge_initial_position_error_m": (
                    self.c1f2_exit_bridge_initial_position_error_m
                ),
                "c1f2_exit_bridge_initial_velocity_error_mps": (
                    self.c1f2_exit_bridge_initial_velocity_error_mps
                ),
                "c1f2_exit_bridge_initial_acceleration_error_mps2": (
                    self.c1f2_exit_bridge_initial_acceleration_error_mps2
                ),
                "c1f2_cached_reference_age_ms": self.c1f2_cached_reference_age_ms,
                "c1f2_handoff_position_error_m": self.c1f2_handoff_position_error_m,
                "c1f2_handoff_velocity_error_mps": self.c1f2_handoff_velocity_error_mps,
                "c1f2_handoff_acceleration_error_mps2": (
                    self.c1f2_handoff_acceleration_error_mps2
                ),
                "m2b_capture_settle_position_error_m": (
                    self.m2b_capture_settle_position_error_m
                ),
                "m2b_capture_settle_speed_mps": self.m2b_capture_settle_speed_mps,
                "m2b_capture_settle_swing_deg": self.m2b_capture_settle_swing_deg,
                "m2b_capture_settle_dwell_s": self.m2b_capture_settle_dwell_s,
                "m2b_capture_settle_reason": self.m2b_capture_settle_reason,
                "m2b_capture_settle_settled": str(
                    bool(self.m2b_capture_settle_settled)
                ).lower(),
                "visual_astar_enabled": str(
                    bool(self.visual_astar_enabled)
                ).lower(),
                "visual_astar_phase_allowed": str(
                    bool(self.visual_astar_phase_allowed())
                ).lower(),
                "visual_astar_job_pending": str(
                    bool(self.visual_astar_future is not None)
                ).lower(),
                "visual_astar_request_id": astar.request_id,
                "visual_astar_status": astar.status,
                "visual_astar_message": astar.message,
                "visual_astar_success": str(bool(astar.success)).lower(),
                "visual_astar_nominal_blocked": str(
                    bool(astar.nominal_blocked)
                ).lower(),
                "visual_astar_first_blocked_segment_index": (
                    ""
                    if astar.first_blocked_segment_index is None
                    else astar.first_blocked_segment_index
                ),
                "visual_astar_rejoin_index": (
                    "" if astar.rejoin_index is None else astar.rejoin_index
                ),
                "visual_astar_rejoin_time_s": (
                    "" if astar.rejoin_time_s is None else astar.rejoin_time_s
                ),
                "visual_astar_rejoin_candidate_count": len(
                    astar.rejoin_candidate_indices
                ),
                "visual_astar_resolution_m": self.visual_astar_resolution,
                "visual_astar_replan_rate_hz": self.visual_astar_replan_rate_hz,
                "visual_astar_planning_time_ms": astar.planning_time_ms,
                "visual_astar_worker_turnaround_ms": (
                    ""
                    if (
                        astar.request_id < 0
                        or not np.isfinite(
                            self.last_visual_astar_worker_turnaround_ms
                        )
                    )
                    else self.last_visual_astar_worker_turnaround_ms
                ),
                "visual_astar_expanded_nodes": astar.expanded_nodes,
                "visual_astar_generated_nodes": astar.generated_nodes,
                "visual_astar_raw_waypoint_count": astar.raw_path.shape[0],
                "visual_astar_simplified_waypoint_count": (
                    astar.simplified_path.shape[0]
                ),
                "visual_astar_candidate_sample_count": (
                    astar.candidate_positions.shape[0]
                ),
                "visual_astar_raw_path_length_m": astar.raw_path_length_m,
                "visual_astar_simplified_path_length_m": (
                    astar.simplified_path_length_m
                ),
                "visual_astar_start_adjustment_m": astar.start_adjustment_m,
                "visual_astar_goal_adjustment_m": astar.goal_adjustment_m,
                "visual_astar_start_requested_x": (
                    float("nan") if astar.start_requested is None else astar.start_requested[0]
                ),
                "visual_astar_start_requested_y": (
                    float("nan") if astar.start_requested is None else astar.start_requested[1]
                ),
                "visual_astar_start_requested_z": (
                    float("nan") if astar.start_requested is None else astar.start_requested[2]
                ),
                "visual_astar_start_used_x": (
                    float("nan") if astar.start_used is None else astar.start_used[0]
                ),
                "visual_astar_start_used_y": (
                    float("nan") if astar.start_used is None else astar.start_used[1]
                ),
                "visual_astar_start_used_z": (
                    float("nan") if astar.start_used is None else astar.start_used[2]
                ),
                "visual_astar_goal_requested_x": (
                    float("nan") if astar.goal_requested is None else astar.goal_requested[0]
                ),
                "visual_astar_goal_requested_y": (
                    float("nan") if astar.goal_requested is None else astar.goal_requested[1]
                ),
                "visual_astar_goal_requested_z": (
                    float("nan") if astar.goal_requested is None else astar.goal_requested[2]
                ),
                "visual_astar_goal_used_x": (
                    float("nan") if astar.goal_used is None else astar.goal_used[0]
                ),
                "visual_astar_goal_used_y": (
                    float("nan") if astar.goal_used is None else astar.goal_used[1]
                ),
                "visual_astar_goal_used_z": (
                    float("nan") if astar.goal_used is None else astar.goal_used[2]
                ),
                "visual_astar_bounds_min_x": (
                    float("nan") if astar.bounds_min is None else astar.bounds_min[0]
                ),
                "visual_astar_bounds_min_y": (
                    float("nan") if astar.bounds_min is None else astar.bounds_min[1]
                ),
                "visual_astar_bounds_min_z": (
                    float("nan") if astar.bounds_min is None else astar.bounds_min[2]
                ),
                "visual_astar_bounds_max_x": (
                    float("nan") if astar.bounds_max is None else astar.bounds_max[0]
                ),
                "visual_astar_bounds_max_y": (
                    float("nan") if astar.bounds_max is None else astar.bounds_max[1]
                ),
                "visual_astar_bounds_max_z": (
                    float("nan") if astar.bounds_max is None else astar.bounds_max[2]
                ),
                "visual_astar_grid_x": astar.grid_shape[0],
                "visual_astar_grid_y": astar.grid_shape[1],
                "visual_astar_grid_z": astar.grid_shape[2],
                "visual_astar_grid_node_count": astar.grid_node_count,
                "visual_astar_candidate_whole_body_safe": (
                    ""
                    if astar.candidate_whole_body_safe is None
                    else str(bool(astar.candidate_whole_body_safe)).lower()
                ),
                "visual_astar_candidate_critical_component": (
                    astar.candidate_critical_component or ""
                ),
                "visual_astar_candidate_minimum_clearance_m": (
                    ""
                    if astar.candidate_minimum_clearance_m is None
                    else astar.candidate_minimum_clearance_m
                ),
                "visual_astar_candidate_quad_clearance_m": (
                    ""
                    if astar.candidate_quad_clearance_m is None
                    else astar.candidate_quad_clearance_m
                ),
                "visual_astar_candidate_magnet_clearance_m": (
                    ""
                    if astar.candidate_magnet_clearance_m is None
                    else astar.candidate_magnet_clearance_m
                ),
                "visual_astar_candidate_cable_clearance_m": (
                    ""
                    if astar.candidate_cable_clearance_m is None
                    else astar.candidate_cable_clearance_m
                ),
                "visual_astar_candidate_payload_clearance_m": (
                    ""
                    if astar.candidate_payload_clearance_m is None
                    else astar.candidate_payload_clearance_m
                ),
                "visual_astar_candidate_payload_geometry_available": str(
                    bool(astar.candidate_payload_geometry_available)
                ).lower(),

                "visual_astar_corridor_valid": (
                    ""
                    if astar.corridor_valid is None
                    else str(bool(astar.corridor_valid)).lower()
                ),

                "visual_astar_corridor_segment_min_margin_m": (
                    ""
                    if astar.corridor_segment_containment_margins_m.size == 0
                    else float(
                        np.min(
                            astar.corridor_segment_containment_margins_m
                        )
                    )
                ),

                "visual_astar_corridor_overlap_min_margin_m": (
                    ""
                    if astar.corridor_overlap_margins_m.size == 0
                    else float(
                        np.min(
                            astar.corridor_overlap_margins_m
                        )
                    )
                ),

                "visual_astar_corridor_voxel_exclusion_min_margin_m": (
                    ""
                    if astar.corridor_voxel_exclusion_margins_m.size == 0
                    else float(
                        np.min(
                            astar.corridor_voxel_exclusion_margins_m
                        )
                    )
                ),

                "visual_astar_corridor_error": (
                    ""
                    if astar.corridor_error is None
                    else astar.corridor_error
                ),

                "visual_astar_completed_count": self.visual_astar_completed_count,
                "visual_astar_discarded_count": self.visual_astar_discarded_count,
                **trajectory_log,
                "safety_action": safety_action,
                "safety_intervention_active": safety_intervention_active,
                "safety_nominal_safe": safety_nominal_safe,
                "safety_critical_component": safety_critical_component,
                "safety_minimum_clearance_m": safety_minimum_clearance,
                "safety_quad_minimum_clearance_m": safety_quad_minimum_clearance,
                "safety_magnet_minimum_clearance_m": (
                    safety_magnet_minimum_clearance
                ),
                "safety_cable_minimum_clearance_m": (
                    safety_cable_minimum_clearance
                ),
                "safety_payload_check_active": safety_payload_check_active,
                "safety_payload_geometry_available": (
                    safety_payload_geometry_available
                ),
                "safety_payload_pose_age_s": safety_payload_pose_age_s,
                "safety_payload_minimum_clearance_m": (
                    safety_payload_minimum_clearance
                ),
                "safety_payload_closest_obstacle_id": (
                    safety_payload_closest_obstacle_id
                ),
                "safety_payload_closest_sample_index": (
                    safety_payload_closest_sample_index
                ),
                "safety_payload_closest_time_s": (
                    safety_payload_closest_time_s
                ),
                "safety_payload_closest_point_x": safety_payload_closest_point[0],
                "safety_payload_closest_point_y": safety_payload_closest_point[1],
                "safety_payload_closest_point_z": safety_payload_closest_point[2],
                "safety_payload_offset_from_magnet_x": (
                    safety_payload_offset_from_magnet[0]
                ),
                "safety_payload_offset_from_magnet_y": (
                    safety_payload_offset_from_magnet[1]
                ),
                "safety_payload_offset_from_magnet_z": (
                    safety_payload_offset_from_magnet[2]
                ),
                "safety_predicted_payload_stage0_x": (
                    safety_predicted_payload_stage0[0]
                ),
                "safety_predicted_payload_stage0_y": (
                    safety_predicted_payload_stage0[1]
                ),
                "safety_predicted_payload_stage0_z": (
                    safety_predicted_payload_stage0[2]
                ),
                "safety_predicted_payload_terminal_x": (
                    safety_predicted_payload_terminal[0]
                ),
                "safety_predicted_payload_terminal_y": (
                    safety_predicted_payload_terminal[1]
                ),
                "safety_predicted_payload_terminal_z": (
                    safety_predicted_payload_terminal[2]
                ),
                "safety_closest_obstacle_id": safety_closest_obstacle_id,
                "safety_closest_segment_index": safety_closest_segment_index,
                "safety_closest_time_s": safety_closest_time_s,
                "safety_closest_point_x": safety_closest_point[0],
                "safety_closest_point_y": safety_closest_point[1],
                "safety_closest_point_z": safety_closest_point[2],
                "safety_closest_obstacle_centre_x": (
                    safety_closest_obstacle_centre[0]
                ),
                "safety_closest_obstacle_centre_y": (
                    safety_closest_obstacle_centre[1]
                ),
                "safety_closest_obstacle_centre_z": (
                    safety_closest_obstacle_centre[2]
                ),
                "safety_magnet_offset_x": safety_magnet_offset[0],
                "safety_magnet_offset_y": safety_magnet_offset[1],
                "safety_magnet_offset_z": safety_magnet_offset[2],
                "safety_predicted_magnet_stage0_x": (
                    safety_predicted_magnet_stage0[0]
                ),
                "safety_predicted_magnet_stage0_y": (
                    safety_predicted_magnet_stage0[1]
                ),
                "safety_predicted_magnet_stage0_z": (
                    safety_predicted_magnet_stage0[2]
                ),
                "safety_predicted_magnet_terminal_x": (
                    safety_predicted_magnet_terminal[0]
                ),
                "safety_predicted_magnet_terminal_y": (
                    safety_predicted_magnet_terminal[1]
                ),
                "safety_predicted_magnet_terminal_z": (
                    safety_predicted_magnet_terminal[2]
                ),
                "safety_cable_closest_obstacle_id": (
                    safety_cable_closest_obstacle_id
                ),
                "safety_cable_closest_sample_index": (
                    safety_cable_closest_sample_index
                ),
                "safety_cable_closest_time_s": safety_cable_closest_time_s,
                "safety_cable_closest_point_x": safety_cable_closest_point[0],
                "safety_cable_closest_point_y": safety_cable_closest_point[1],
                "safety_cable_closest_point_z": safety_cable_closest_point[2],
                "safety_cable_closest_quad_x": safety_cable_closest_quad[0],
                "safety_cable_closest_quad_y": safety_cable_closest_quad[1],
                "safety_cable_closest_quad_z": safety_cable_closest_quad[2],
                "safety_cable_closest_magnet_x": safety_cable_closest_magnet[0],
                "safety_cable_closest_magnet_y": safety_cable_closest_magnet[1],
                "safety_cable_closest_magnet_z": safety_cable_closest_magnet[2],
                "safety_planning_time_ms": safety_planning_time_ms,
                "planner_callback_time_ms": self.last_planner_callback_time_ms,
                "planner_callback_period_ms": self.last_planner_callback_period_ms,
                "planner_callback_budget_ms": self.planner_callback_budget_ms,
                "planner_deadline_miss_count": self.planner_deadline_miss_count,
                "quad_x": self.drone_position[0],
                "quad_y": self.drone_position[1],
                "quad_z": self.drone_position[2],
                "target_x": target[0],
                "target_y": target[1],
                "target_z": target[2],
                "tip_target_x": tip_target[0],
                "tip_target_y": tip_target[1],
                "tip_target_z": tip_target[2],
                "tip_actual_x": tip_actual[0],
                "tip_actual_y": tip_actual[1],
                "tip_actual_z": tip_actual[2],
                "tip_error_xy": xy_error,
                "tip_error_z": z_error,
                "relative_speed_xy": relative_speed,
                "ref_stage0_x": reference.positions[0, 0],
                "ref_stage0_y": reference.positions[0, 1],
                "ref_stage0_z": reference.positions[0, 2],
                "ref_terminal_x": reference.positions[-1, 0],
                "ref_terminal_y": reference.positions[-1, 1],
                "ref_terminal_z": reference.positions[-1, 2],
                "phase_elapsed_s": self.phase_elapsed(),
                "rate_profile_duration_s": self.last_rate_profile_duration,
                "rate_profile_complete": str(bool(self.last_rate_profile_complete)).lower(),
                "transfer_duration_s": self.transfer_generator.last_duration,
                "transfer_progress_scale": self.transfer_generator.last_progress_scale,
                "pickup_compensation_x": self.pickup_tip_xy_compensation_filtered[0],
                "pickup_compensation_y": self.pickup_tip_xy_compensation_filtered[1],
                "last_transition_reason": self.last_transition_reason,
            }
        )
        self.csv_file.flush()

    def close_csv_logger(self) -> None:
        if self.csv_file is not None:
            try:
                self.csv_file.flush()
                self.csv_file.close()
            finally:
                self.csv_file = None
                self.csv_writer = None

    # ----------------------------------------------------------------------
    # Main planner tick
    # ----------------------------------------------------------------------
    def timer_callback(self) -> None:
        callback_start = time.perf_counter()
        if self._last_planner_callback_start is not None:
            self.last_planner_callback_period_ms = (
                callback_start - self._last_planner_callback_start
            ) * 1000.0
        self._last_planner_callback_start = callback_start

        try:
            if self.is_m2b:
                self.update_m2b_bootstrap()
                self.publish_m2b_control_contracts()

            if self.publish_committed_trajectory and not self.m2d_enabled:
                # Legacy M2C is ground-only and owns a permanent measured hold.
                # M2D publishes exactly one phase-aware commitment from the
                # actual reference path below, avoiding alternating stationary
                # and moving declarations on the same transient-local topic.
                self.publish_stationary_committed_trajectory()
            if self.m2c_ground_reference_enabled:
                # M2C is deliberately ground-only.  Its current-pose reference is
                # the sole reference authority in this mode; do not fall through
                # into the normal join mission and accidentally publish a second
                # moving reference in the same callback.
                self.publish_m2c_ground_reference()
                return

            if not self.inputs_ready():
                return

            self.update_pickup_tip_xy_compensation()
            self.update_state_machine()
            if self.is_m2b:
                # A phase transition can change arm permission or MPC mode.
                self.publish_m2b_control_contracts()
            self.update_pickup_geometry_diagnostics()

            # A transition can change which target source is required.
            if not self.inputs_ready():
                return

            spec = self.current_phase_spec()
            self.publish_magnet_command(
                self.m2b_magnet_command() if self.is_m2b else spec.magnet_command
            )

            # C1F.8: once C++ owns TRANSIT_TO_DROP_POINT, the independently
            # advancing Python transfer is discarded by the authority selector.
            # Avoid constructing it at 30 Hz until the C++ -> Python bridge begins.
            python_nominal_reference: Optional[TrajectoryReference]
            if self.c1f2_cpp_transit_authority_active():
                python_nominal_reference = None
            else:
                python_nominal_reference = self.build_reference()
            self.update_c1f1_shadow_transfer()
            nominal_reference = self.select_c1f2_authority_reference(
                python_nominal_reference
            )
            assert self.drone_position is not None
            assert self.drone_velocity is not None
            magnet_offset_from_quad = (
                self.current_magnet_tip_position() - self.drone_position
            )
            (
                payload_geometry_available,
                payload_offset_from_magnet,
                payload_yaw,
                payload_pose_age_s,
            ) = self.attached_payload_geometry_for_safety()
            safety_request = SafetyPlannerRequest(
                nominal_reference=nominal_reference,
                measured_position=self.drone_position,
                measured_velocity=self.drone_velocity,
                magnet_offset_from_quad=magnet_offset_from_quad,
                phase_name=self.phase.value,
                dt=self.dt,
                object_attached=self.object_attached,
                payload_geometry_available=payload_geometry_available,
                payload_offset_from_magnet=payload_offset_from_magnet,
                payload_yaw=payload_yaw,
                payload_pose_age_s=payload_pose_age_s,
            )
            safety_result = self.safety_planner.plan(safety_request)
            self.update_visual_astar(nominal_reference, safety_request)
            reference = safety_result.reference

            self.last_nominal_reference = nominal_reference
            self.last_safety_result = safety_result
            self.last_reference = reference

            self.write_csv_log(reference)
            self.publish_reference(reference)
            self.maybe_publish_landing_disarm()
            self.publish_markers(reference)
            self.publish_state()
        finally:
            callback_time_ms = (time.perf_counter() - callback_start) * 1000.0
            self.last_planner_callback_time_ms = callback_time_ms
            if callback_time_ms > self.planner_callback_budget_ms:
                self.planner_deadline_miss_count += 1


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OnlineJoinPlanner()
    try:
        rclpy.spin(node)
    finally:
        node.close_visual_astar()
        node.close_csv_logger()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

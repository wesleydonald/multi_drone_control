"""Pure runtime helpers shared by M2A ROS logging and offline replay.

No ROS or Gazebo imports live here.  The aim is to keep the exact detector input
construction and recorded-data replay deterministic and testable offline.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import math
from typing import Mapping

import numpy as np

from .attachment_detection import (
    AttachmentDetectorConfig,
    AttachmentObservation,
    MagnetContactCalibration,
)


def _vec3(value, *, name: str) -> np.ndarray:
    array = np.asarray(value, dtype=float).reshape(3)
    if not np.all(np.isfinite(array)):
        raise ValueError(f"{name} must be finite")
    return array.copy()


def normalize_quaternion_xyzw(value) -> np.ndarray:
    q = np.asarray(value, dtype=float).reshape(4)
    if not np.all(np.isfinite(q)):
        raise ValueError("quaternion must be finite")
    norm = float(np.linalg.norm(q))
    if norm <= 1e-12:
        raise ValueError("quaternion must have non-zero norm")
    return q / norm


def quaternion_xyzw_to_rotation(value) -> np.ndarray:
    x, y, z, w = normalize_quaternion_xyzw(value)
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=float,
    )


def rotation_to_quaternion_xyzw(rotation) -> np.ndarray:
    R = np.asarray(rotation, dtype=float).reshape(3, 3)
    if not np.all(np.isfinite(R)):
        raise ValueError("rotation must be finite")
    trace = float(np.trace(R))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return normalize_quaternion_xyzw([x, y, z, w])


def reconstruct_relative_pose_world(
    *,
    parent_position_world,
    parent_quaternion_xyzw,
    child_position_parent,
    child_quaternion_xyzw,
):
    p_wp = _vec3(parent_position_world, name="parent_position_world")
    R_wp = quaternion_xyzw_to_rotation(parent_quaternion_xyzw)
    p_pc = _vec3(child_position_parent, name="child_position_parent")
    R_pc = quaternion_xyzw_to_rotation(child_quaternion_xyzw)
    return p_wp + R_wp @ p_pc, R_wp @ R_pc




@dataclass(frozen=True)
class NamedTransform:
    """One parent->child rigid transform from a named pose/TF stream."""

    parent_frame: str
    child_frame: str
    translation: np.ndarray
    quaternion_xyzw: np.ndarray

    def __init__(self, parent_frame, child_frame, translation, quaternion_xyzw):
        object.__setattr__(self, "parent_frame", str(parent_frame).strip())
        object.__setattr__(self, "child_frame", str(child_frame).strip())
        object.__setattr__(self, "translation", _vec3(translation, name="translation"))
        object.__setattr__(
            self,
            "quaternion_xyzw",
            normalize_quaternion_xyzw(quaternion_xyzw),
        )


def _frame_leaf(name: str) -> str:
    text = str(name).strip().strip("/")
    if "::" in text:
        text = text.split("::")[-1]
    if "/" in text:
        text = text.split("/")[-1]
    return text


def resolve_named_transform_world(transforms, target_leaf: str):
    """Resolve a named frame to its root by composing a parent/child TF graph.

    Gazebo PosePublisher may namespace child frames.  Callers therefore request
    a unique leaf such as ``carrier_link`` or ``magnet_tip_link``.  The function
    is ROS-independent so the exact graph logic can be unit-tested offline.
    """

    items = tuple(transforms)
    matches = [item.child_frame for item in items if _frame_leaf(item.child_frame) == target_leaf]
    if not matches:
        raise KeyError(f"no transform for frame leaf {target_leaf!r}")
    if len(matches) != 1:
        raise ValueError(f"ambiguous transform leaf {target_leaf!r}: {matches}")

    by_child = {}
    for item in items:
        if item.child_frame in by_child:
            raise ValueError(f"duplicate transform child frame {item.child_frame!r}")
        by_child[item.child_frame] = item

    frame = matches[0]
    chain = []
    visited = set()
    while frame in by_child:
        if frame in visited:
            raise ValueError(f"transform cycle detected at {frame!r}")
        visited.add(frame)
        item = by_child[frame]
        chain.append(item)
        frame = item.parent_frame
        if not frame:
            break

    root_frame = frame
    position = np.zeros(3, dtype=float)
    rotation = np.eye(3, dtype=float)
    for item in reversed(chain):
        relative_rotation = quaternion_xyzw_to_rotation(item.quaternion_xyzw)
        position = position + rotation @ item.translation
        rotation = rotation @ relative_rotation
    return position, rotation, root_frame


def default_calibration() -> MagnetContactCalibration:
    # Legacy rigid/contact-face calibration used by M2A and as the default IRL
    # detector model. M2B simulation explicitly selects sphere-centre observation
    # instead, because the ball-jointed sphere orientation is not a contact cue.
    return MagnetContactCalibration(
        contact_position_magnet=np.array([0.0, 0.0, -0.025], dtype=float),
        rotation_magnet_from_contact=np.eye(3),
    )


def default_detector_config() -> AttachmentDetectorConfig:
    # Commissioning defaults only.  They remain explicit/logged and are expected
    # to be tuned from recorded evidence rather than treated as final IRL values.
    angle = math.radians(15.0)
    proof_direction = np.array([math.sin(angle), 0.0, math.cos(angle)], dtype=float)
    return AttachmentDetectorConfig(
        candidate_xy_m=0.020,
        candidate_normal_m=0.020,
        candidate_speed_mps=0.12,
        candidate_dwell_s=0.15,
        proof_xy_m=0.025,
        proof_normal_m=0.025,
        proof_speed_mps=0.15,
        proof_dwell_s=0.15,
        proof_min_excitation_m=0.012,
        proof_direction_plate=proof_direction,
        loss_xy_m=0.045,
        loss_normal_m=0.045,
        loss_dwell_s=0.12,
        pose_timeout_s=0.25,
        velocity_filter_tau_s=0.06,
    )


def detector_config_to_dict(config: AttachmentDetectorConfig) -> dict:
    result = asdict(config)
    result["proof_direction_plate"] = np.asarray(
        config.proof_direction_plate, dtype=float
    ).tolist()
    return result


def detector_config_from_dict(payload: Mapping[str, object]) -> AttachmentDetectorConfig:
    values = dict(payload)
    values["proof_direction_plate"] = np.asarray(values["proof_direction_plate"], dtype=float)
    return AttachmentDetectorConfig(**values)


def calibration_to_dict(calibration: MagnetContactCalibration) -> dict:
    return {
        "contact_position_magnet": calibration.contact_position_magnet.tolist(),
        "rotation_magnet_from_contact": calibration.rotation_magnet_from_contact.tolist(),
    }


def calibration_from_dict(payload: Mapping[str, object]) -> MagnetContactCalibration:
    return MagnetContactCalibration(
        contact_position_magnet=np.asarray(payload["contact_position_magnet"], dtype=float),
        rotation_magnet_from_contact=np.asarray(payload["rotation_magnet_from_contact"], dtype=float),
    )


def parse_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    text = str(value).strip().lower()
    if text in {"1", "true", "yes", "on"}:
        return True
    if text in {"0", "false", "no", "off", ""}:
        return False
    raise ValueError(f"cannot parse boolean value {value!r}")


def _f(row: Mapping[str, object], name: str) -> float:
    return float(row[name])


def observation_from_csv_row(row: Mapping[str, object]) -> AttachmentObservation:
    ring_q = np.array([_f(row, f"ring_q{a}") for a in "xyzw"], dtype=float)
    magnet_q = np.array([_f(row, f"magnet_q{a}") for a in "xyzw"], dtype=float)
    return AttachmentObservation(
        time_s=_f(row, "ros_time_s"),
        ring_position_world=np.array([_f(row, f"ring_{a}") for a in "xyz"], dtype=float),
        rotation_world_from_ring=quaternion_xyzw_to_rotation(ring_q),
        ring_time_s=_f(row, "ring_time_s"),
        magnet_position_world=np.array([_f(row, f"magnet_{a}") for a in "xyz"], dtype=float),
        rotation_world_from_magnet=quaternion_xyzw_to_rotation(magnet_q),
        magnet_time_s=_f(row, "magnet_time_s"),
        drone_position_world=np.array([_f(row, f"drone_{a}") for a in "xyz"], dtype=float),
        drone_time_s=_f(row, "drone_time_s"),
        magnet_on=parse_bool(row.get("magnet_on", False)),
        proof_requested=parse_bool(row.get("proof_requested", False)),
    )


def parse_gz_detachable_joint_state_line(line: str):
    """Return raw detached-state bool from one Gazebo StringMsg echo line.

    Gazebo Sim 7's DetachableJoint publishes ``msgs::StringMsg`` with data
    equal to ``attached`` or ``detached``.  This helper preserves the raw
    detached-state semantics used by M2A ground truth: True means detached.
    Non-state lines return None.
    """
    text = str(line).strip().lower()
    if "detached" in text:
        return True
    if "attached" in text:
        return False
    return None

#!/usr/bin/env python3
"""Build and verify the articulated M2B B0 ground-start initial condition.

Gazebo's set_pose service cannot reliably teleport links that are already joined
into an articulated model.  B0 therefore creates a run-local copy of the exact
current X3 SDF before Gazebo starts and changes only the tether_rod initial pose.
The repository X3 remains untouched.  A run-local world then points at that copy
and places the model at the computed body pose.

The same module also checks measured B0 telemetry before physics is unpaused or
arming is allowed.  Planned geometry is never accepted as runtime evidence.
"""

from __future__ import annotations

import argparse
import copy
import csv
from dataclasses import asdict, dataclass
import hashlib
import json
import math
from pathlib import Path
import re
import sys
import time
from typing import Any
import xml.etree.ElementTree as ET

import numpy as np

from tejen_mission.m2b_ground_start import GroundStartConfig, compute_ground_start_geometry


_X3_NAME = "m2b_generated_x3.sdf"
_RING_NAME = "m2b_generated_ring.sdf"
_WORLD_NAME = "m2b_generated_world.sdf"
_TETHER_PATTERN = re.compile(
    r'(<link\s+name="tether_rod"\s*>\s*)'
    r'(<pose\s+relative_to="X3/base_link"\s*>[^<]+</pose>)',
    flags=re.MULTILINE,
)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _rotation_from_quaternion_xyzw(q: np.ndarray) -> np.ndarray:
    x, y, z, w = np.asarray(q, dtype=float).reshape(4)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 0.0:
        raise ValueError("Quaternion must be non-zero")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
    ], dtype=float)


def _yaw_rotation(yaw: float) -> np.ndarray:
    c = math.cos(float(yaw))
    s = math.sin(float(yaw))
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=float)


def _rpy_from_rotation(R: np.ndarray) -> tuple[float, float, float]:
    """Return SDF roll-pitch-yaw for a proper rotation matrix."""
    R = np.asarray(R, dtype=float).reshape(3, 3)
    # ZYX convention used by SDF Pose3d: R = Rz(yaw) Ry(pitch) Rx(roll).
    sy = -float(R[2, 0])
    sy = max(-1.0, min(1.0, sy))
    pitch = math.asin(sy)
    cp = math.cos(pitch)
    if abs(cp) > 1e-9:
        roll = math.atan2(float(R[2, 1]), float(R[2, 2]))
        yaw = math.atan2(float(R[1, 0]), float(R[0, 0]))
    else:
        roll = 0.0
        yaw = math.atan2(-float(R[0, 1]), float(R[1, 1]))
    return roll, pitch, yaw


def _format_pose(values: list[float]) -> str:
    return " ".join(f"{float(v):.12g}" for v in values)


@dataclass(frozen=True)
class GroundStartAssets:
    x3_path: Path
    world_path: Path
    manifest_path: Path
    geometry: Any
    source_tether_pose_element: str
    generated_tether_pose_element: str
    ring_path: Path | None = None


@dataclass(frozen=True)
class GroundStartVerification:
    ok: bool
    body_position_error_m: float
    magnet_position_error_m: float
    magnet_plate_xy_error_m: float
    magnet_bottom_z_m: float
    cable_angle_from_horizontal_deg: float
    cable_length_error_m: float
    reasons: tuple[str, ...]


@dataclass(frozen=True)
class PostDetachGroundVerification:
    ok: bool
    drone_speed_mps: float
    magnet_speed_mps: float
    magnet_bottom_z_m: float
    magnet_floor_error_m: float
    body_height_error_m: float
    reasons: tuple[str, ...]


def _set_collide_bitmask(collision: ET.Element, mask: int) -> None:
    value = int(mask)
    if value < 0 or value > 0xFFFF:
        raise ValueError("collision mask must fit in 16 bits")
    surface = collision.find("surface")
    if surface is None:
        surface = ET.SubElement(collision, "surface")
    contact = surface.find("contact")
    if contact is None:
        contact = ET.SubElement(surface, "contact")
    node = contact.find("collide_bitmask")
    if node is None:
        node = ET.SubElement(contact, "collide_bitmask")
    node.text = f"0x{value:04x}"


def _apply_simulation_attachment_stabilization(path: Path, damping: float) -> None:
    damping = float(damping)
    if not math.isfinite(damping) or damping < 0.0:
        raise ValueError("attachment_joint_damping must be finite and non-negative")
    tree = ET.parse(path)
    model = tree.getroot().find("model")
    if model is None:
        raise RuntimeError("Generated X3 SDF has no model")

    magnet = model.find("./link[@name='magnet_tip_link']")
    if magnet is None:
        raise RuntimeError("Generated X3 SDF is missing magnet_tip_link")
    magnet_collision = magnet.find("./collision[@name='magnet_tip_collision']")
    if magnet_collision is None:
        raise RuntimeError("Generated X3 SDF is missing magnet_tip_collision")
    # M2B simulation category 0x0001: the black magnet may contact the floor,
    # but not the generated ring / steel plates (category 0x0002).
    _set_collide_bitmask(magnet_collision, 0x0001)

    for joint_name in ("base_to_tether", "tether_to_magnet_tip"):
        joint = model.find(f"./joint[@name='{joint_name}']")
        if joint is None or joint.attrib.get("type") != "ball":
            raise RuntimeError(f"Expected ball joint {joint_name!r} in generated X3")
        for axis_name, xyz in (("axis", "1 0 0"), ("axis2", "0 1 0")):
            axis = joint.find(axis_name)
            if axis is None:
                axis = ET.SubElement(joint, axis_name)
            xyz_node = axis.find("xyz")
            if xyz_node is None:
                xyz_node = ET.SubElement(axis, "xyz")
            xyz_node.text = xyz
            dynamics = axis.find("dynamics")
            if dynamics is None:
                dynamics = ET.SubElement(axis, "dynamics")
            damping_node = dynamics.find("damping")
            if damping_node is None:
                damping_node = ET.SubElement(dynamics, "damping")
            damping_node.text = f"{damping:.12g}"

    tree.write(path, encoding="utf-8", xml_declaration=True)


def _generate_selective_collision_ring(source_ring: Path, output_path: Path) -> None:
    source_ring = Path(source_ring).resolve()
    source_before = source_ring.read_bytes()
    tree = ET.parse(source_ring)
    model = tree.getroot().find("model")
    if model is None:
        raise RuntimeError("Ring fixture SDF has no model")
    collisions = model.findall(".//collision")
    if not collisions:
        raise RuntimeError("Ring fixture SDF has no collisions")
    for collision in collisions:
        _set_collide_bitmask(collision, 0x0002)
    tree.write(output_path, encoding="utf-8", xml_declaration=True)
    if source_ring.read_bytes() != source_before:
        raise RuntimeError("Source ring SDF changed during selective-collision generation")


def _move_detachable_joint_to_static_ring_parent(x3_path: Path, ring_path: Path) -> None:
    """Reverse only the B1 run-local DetachableJoint parent/child topology.

    Gazebo defines the model containing the DetachableJoint system as the parent
    model.  The repository X3 therefore cannot express a true ring-parent test by
    swapping link names in place: the system plugin itself must move to the
    generated ring model.  Topics and fixed-joint semantics are preserved.
    """
    x3_tree = ET.parse(x3_path)
    x3_model = x3_tree.getroot().find("model")
    if x3_model is None:
        raise RuntimeError("Generated X3 SDF has no model")
    x3_name = (x3_model.attrib.get("name") or "").strip()
    if not x3_name:
        raise RuntimeError("Generated X3 SDF model has no name")

    plugin = x3_model.find("./plugin[@name='gz::sim::systems::DetachableJoint']")
    if plugin is None:
        raise RuntimeError("Generated X3 SDF is missing DetachableJoint plugin")
    if plugin.findtext("parent_link") != "magnet_tip_link":
        raise RuntimeError("Unexpected source DetachableJoint parent_link")
    if plugin.findtext("child_model") != "payload_model":
        raise RuntimeError("Unexpected source DetachableJoint child_model")
    if plugin.findtext("child_link") != "payload_link":
        raise RuntimeError("Unexpected source DetachableJoint child_link")

    ring_tree = ET.parse(ring_path)
    ring_model = ring_tree.getroot().find("model")
    if ring_model is None:
        raise RuntimeError("Generated ring SDF has no model")
    if (ring_model.findtext("static") or "").strip().lower() != "true":
        raise RuntimeError("Topology discriminator requires the generated ring to remain static")
    if ring_model.find("./link[@name='payload_link']") is None:
        raise RuntimeError("Generated ring SDF is missing payload_link")
    if ring_model.find("./plugin[@name='gz::sim::systems::DetachableJoint']") is not None:
        raise RuntimeError("Generated ring SDF already contains a DetachableJoint plugin")

    moved = copy.deepcopy(plugin)
    moved.find("parent_link").text = "payload_link"
    moved.find("child_model").text = x3_name
    moved.find("child_link").text = "magnet_tip_link"

    x3_model.remove(plugin)
    ring_model.append(moved)
    x3_tree.write(x3_path, encoding="utf-8", xml_declaration=True)
    ring_tree.write(ring_path, encoding="utf-8", xml_declaration=True)


def generate_ground_start_assets(
    *,
    source_x3: Path,
    world_template: Path,
    output_dir: Path,
    assigned_plate_id: int,
    config: GroundStartConfig | None = None,
    manifest_path: Path | None = None,
    simulation_attachment_stabilization: bool = False,
    attachment_joint_damping: float = 0.1,
) -> GroundStartAssets:
    """Generate a run-local X3/world pair.

    B0's default remains the historical one-pose edit. B1 may opt into
    simulation-only attachment stabilization: partner-matched 0.1 damping plus
    selective static collision masks. The magnet may contact the floor but not
    the ring / steel plates. Repository source SDFs are never modified.
    """
    source_x3 = Path(source_x3).resolve()
    world_template = Path(world_template).resolve()
    output_dir = Path(output_dir).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    cfg = GroundStartConfig() if config is None else config
    geometry = compute_ground_start_geometry(cfg, assigned_plate_id=int(assigned_plate_id))

    source_before = source_x3.read_text(encoding="utf-8")
    match = _TETHER_PATTERN.search(source_before)
    if match is None or len(_TETHER_PATTERN.findall(source_before)) != 1:
        raise RuntimeError("Expected exactly one real-X3 tether_rod pose relative to X3/base_link")
    source_pose_element = match.group(2)

    R_wb = _yaw_rotation(geometry.body_yaw_rad)
    R_wt = _rotation_from_quaternion_xyzw(geometry.tether_quaternion_xyzw)
    p_bt = R_wb.T @ (geometry.tether_origin_world - geometry.body_position_world)
    R_bt = R_wb.T @ R_wt
    roll, pitch, yaw = _rpy_from_rotation(R_bt)
    generated_pose_element = (
        '<pose relative_to="X3/base_link">'
        + _format_pose([p_bt[0], p_bt[1], p_bt[2], roll, pitch, yaw])
        + "</pose>"
    )
    generated_x3_text = source_before[:match.start(2)] + generated_pose_element + source_before[match.end(2):]

    x3_path = output_dir / _X3_NAME
    x3_path.write_text(generated_x3_text, encoding="utf-8")
    if simulation_attachment_stabilization:
        _apply_simulation_attachment_stabilization(x3_path, attachment_joint_damping)
    # Generation must never modify the repository source as a side effect.
    if source_x3.read_text(encoding="utf-8") != source_before:
        raise RuntimeError("Source X3 SDF changed during ground-start generation")

    # The run-local world preserves ordering/plugins. B0 changes only the X3
    # include/pose. B1 additionally points the ring include at a run-local copy
    # whose collisions use category 0x0002 and gives the floor mask 0x0003.
    tree = ET.parse(world_template)
    root = tree.getroot()
    x3_include = None
    ring_include = None
    for include in root.findall(".//include"):
        uri = (include.findtext("uri") or "").strip()
        if uri == "modelLargeM2BallMagnet.sdf":
            if x3_include is not None:
                raise RuntimeError("World contains more than one real-X3 include")
            x3_include = include
        elif uri == "m2a_ring_fixture.sdf":
            if ring_include is not None:
                raise RuntimeError("World contains more than one ring fixture include")
            ring_include = include
    if x3_include is None:
        raise RuntimeError("World template does not include modelLargeM2BallMagnet.sdf")

    ring_path: Path | None = None
    if simulation_attachment_stabilization:
        if ring_include is None:
            raise RuntimeError("B1 selective collision requires m2a_ring_fixture.sdf include")
        source_ring = world_template.parent / "m2a_ring_fixture.sdf"
        if not source_ring.is_file():
            raise RuntimeError(f"Ring fixture not found next to world template: {source_ring}")
        ring_path = output_dir / _RING_NAME
        _generate_selective_collision_ring(source_ring, ring_path)
        _move_detachable_joint_to_static_ring_parent(x3_path, ring_path)
        ring_include.find("uri").text = ring_path.name
        floor_collision = root.find(".//model[@name='floor']/link/collision[@name='col']")
        if floor_collision is None:
            raise RuntimeError("B1 selective collision requires world floor::link::col")
        _set_collide_bitmask(floor_collision, 0x0003)

    x3_include.find("uri").text = x3_path.name
    pose = x3_include.find("pose")
    if pose is None:
        pose = ET.SubElement(x3_include, "pose")
    pose.text = _format_pose([
        geometry.body_position_world[0], geometry.body_position_world[1], geometry.body_position_world[2],
        0.0, 0.0, geometry.body_yaw_rad,
    ])
    world_path = output_dir / _WORLD_NAME
    tree.write(world_path, encoding="utf-8", xml_declaration=True)

    manifest_path = Path(manifest_path).resolve() if manifest_path is not None else output_dir.parent / "ground_start.json"
    manifest_path.parent.mkdir(parents=True, exist_ok=True)
    cable = geometry.magnet_center_world - geometry.joint_anchor_world
    horizontal = float(np.linalg.norm(cable[:2]))
    angle_deg = float(np.degrees(np.arctan2(abs(float(cable[2])), horizontal)))
    manifest = {
        "schema_version": 3,
        "method": "run_local_real_x3_sdf_initial_pose",
        "source_x3": str(source_x3),
        "source_x3_sha256": _sha256(source_x3),
        "generated_x3": str(x3_path),
        "generated_x3_sha256": _sha256(x3_path),
        "generated_world": str(world_path),
        "generated_ring": None if ring_path is None else str(ring_path),
        "assigned_plate_id": int(assigned_plate_id),
        "config": asdict(cfg),
        "body_position_world_m": geometry.body_position_world.tolist(),
        "body_yaw_rad": float(geometry.body_yaw_rad),
        "tether_origin_world_m": geometry.tether_origin_world.tolist(),
        "joint_anchor_world_m": geometry.joint_anchor_world.tolist(),
        "magnet_center_world_m": geometry.magnet_center_world.tolist(),
        "plate_position_world_m": geometry.plate_position_world.tolist(),
        "cable_angle_from_horizontal_deg": angle_deg,
        "source_tether_pose_element": source_pose_element,
        "generated_tether_pose_element": generated_pose_element,
        "source_x3_modified": False,
        "simulation_attachment_stabilization": bool(simulation_attachment_stabilization),
        "attachment_joint_damping": float(attachment_joint_damping),
        "magnet_tip_collision_removed": False,
        "magnet_collision_bitmask": "0x0001" if simulation_attachment_stabilization else None,
        "ring_collision_bitmask": "0x0002" if simulation_attachment_stabilization else None,
        "floor_collision_bitmask": "0x0003" if simulation_attachment_stabilization else None,
        "selective_collision_policy": (
            "magnet_floor_yes_magnet_ring_no" if simulation_attachment_stabilization else None
        ),
        "detachable_joint_topology": (
            "static_ring_parent_dynamic_x3_child" if simulation_attachment_stabilization
            else "source_x3_parent_payload_child"
        ),
    }
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    return GroundStartAssets(
        x3_path=x3_path,
        world_path=world_path,
        manifest_path=manifest_path,
        geometry=geometry,
        source_tether_pose_element=source_pose_element,
        generated_tether_pose_element=generated_pose_element,
        ring_path=ring_path,
    )


def verify_measured_ground_start(
    *,
    assigned_plate_id: int,
    drone_position_world: np.ndarray,
    drone_yaw_rad: float,
    magnet_position_world: np.ndarray,
    config: GroundStartConfig | None = None,
    body_tolerance_m: float = 0.030,
    magnet_tolerance_m: float = 0.035,
    plate_xy_tolerance_m: float = 0.035,
    floor_tolerance_m: float = 0.005,
    cable_angle_max_deg: float = 10.0,
    cable_length_tolerance_m: float = 0.030,
) -> GroundStartVerification:
    """Check measured Gazebo state against the B0 ground-start contract."""
    cfg = GroundStartConfig() if config is None else config
    expected = compute_ground_start_geometry(cfg, assigned_plate_id=int(assigned_plate_id))
    drone = np.asarray(drone_position_world, dtype=float).reshape(3)
    magnet = np.asarray(magnet_position_world, dtype=float).reshape(3)
    if not np.all(np.isfinite(drone)) or not np.all(np.isfinite(magnet)) or not math.isfinite(float(drone_yaw_rad)):
        return GroundStartVerification(False, math.inf, math.inf, math.inf, -math.inf, math.inf, math.inf, ("non-finite measured pose",))

    body_error = float(np.linalg.norm(drone - expected.body_position_world))
    magnet_error = float(np.linalg.norm(magnet - expected.magnet_center_world))
    plate_xy_error = float(np.linalg.norm(magnet[:2] - expected.plate_position_world[:2]))
    magnet_bottom = float(magnet[2] - cfg.magnet_radius_m)

    # In the current real X3 SDF, base_to_tether joint is -0.05 m in a tether
    # frame whose zero pose is +0.01 m above body origin.  The resulting ball
    # joint anchor is therefore body z - 0.04 m. The offset is vertical and yaw
    # invariant at the grounded upright initial condition.
    joint_offset_body = np.array([
        0.0,
        0.0,
        cfg.tether_origin_initial_body_z_m + cfg.base_joint_offset_tether_z_m,
    ], dtype=float)
    joint_anchor = drone + _yaw_rotation(float(drone_yaw_rad)) @ joint_offset_body
    cable = magnet - joint_anchor
    cable_length = float(np.linalg.norm(cable))
    horizontal = float(np.linalg.norm(cable[:2]))
    angle_deg = math.degrees(math.atan2(abs(float(cable[2])), horizontal))
    expected_joint_length = cfg.tether_length_m + cfg.base_joint_offset_tether_z_m
    length_error = abs(cable_length - expected_joint_length)

    reasons: list[str] = []
    if body_error > body_tolerance_m:
        reasons.append(f"body position error {body_error:.3f} m > {body_tolerance_m:.3f} m")
    if magnet_error > magnet_tolerance_m:
        reasons.append(f"magnet position error {magnet_error:.3f} m > {magnet_tolerance_m:.3f} m")
    if plate_xy_error > plate_xy_tolerance_m:
        reasons.append(f"magnet/plate XY error {plate_xy_error:.3f} m > {plate_xy_tolerance_m:.3f} m")
    if magnet_bottom < -floor_tolerance_m:
        reasons.append(f"magnet bottom z {magnet_bottom:.3f} m is below floor tolerance")
    if angle_deg > cable_angle_max_deg:
        reasons.append(f"cable angle from horizontal {angle_deg:.2f} deg > {cable_angle_max_deg:.2f} deg")
    if length_error > cable_length_tolerance_m:
        reasons.append(f"joint-to-magnet length error {length_error:.3f} m > {cable_length_tolerance_m:.3f} m")

    return GroundStartVerification(
        ok=not reasons,
        body_position_error_m=body_error,
        magnet_position_error_m=magnet_error,
        magnet_plate_xy_error_m=plate_xy_error,
        magnet_bottom_z_m=magnet_bottom,
        cable_angle_from_horizontal_deg=float(angle_deg),
        cable_length_error_m=float(length_error),
        reasons=tuple(reasons),
    )


def verify_post_detach_floor_settle(
    *,
    drone_position_world: np.ndarray,
    drone_velocity_world: np.ndarray,
    magnet_position_world: np.ndarray,
    magnet_speed_mps: float,
    joint_detached: bool,
    magnet_radius_m: float = 0.025,
    expected_body_z_m: float = 0.105,
    floor_z_m: float = 0.0,
    floor_tolerance_m: float = 0.010,
    body_height_tolerance_m: float = 0.030,
    drone_speed_limit_mps: float = 0.030,
    magnet_speed_limit_mps: float = 0.050,
) -> PostDetachGroundVerification:
    """Verify the real post-detach ground state before B1 is allowed to arm.

    This is intentionally a measured physical gate. With the B1 selective
    collision policy, the released black magnet falls a few centimetres from the
    startup plate to the real floor and must settle there while the X3 remains
    grounded and disarmed.
    """
    drone = np.asarray(drone_position_world, dtype=float).reshape(3)
    drone_velocity = np.asarray(drone_velocity_world, dtype=float).reshape(3)
    magnet = np.asarray(magnet_position_world, dtype=float).reshape(3)
    magnet_speed = abs(float(magnet_speed_mps))
    values = np.concatenate((drone, drone_velocity, magnet, np.array([magnet_speed], dtype=float)))
    if not np.all(np.isfinite(values)):
        return PostDetachGroundVerification(
            False, math.inf, math.inf, -math.inf, math.inf, math.inf, ("non-finite post-detach ground evidence",)
        )

    drone_speed = float(np.linalg.norm(drone_velocity))
    magnet_bottom = float(magnet[2] - float(magnet_radius_m))
    floor_error = abs(magnet_bottom - float(floor_z_m))
    body_height_error = abs(float(drone[2]) - float(expected_body_z_m))
    reasons: list[str] = []
    if not bool(joint_detached):
        reasons.append("raw joint truth is not detached")
    if floor_error > float(floor_tolerance_m):
        reasons.append(
            f"magnet is not settled on floor: bottom-z error {floor_error:.3f} m > {floor_tolerance_m:.3f} m"
        )
    if body_height_error > float(body_height_tolerance_m):
        reasons.append(
            f"grounded body height error {body_height_error:.3f} m > {body_height_tolerance_m:.3f} m"
        )
    if drone_speed > float(drone_speed_limit_mps):
        reasons.append(f"grounded drone speed {drone_speed:.3f} m/s > {drone_speed_limit_mps:.3f} m/s")
    if magnet_speed > float(magnet_speed_limit_mps):
        reasons.append(f"magnet speed {magnet_speed:.3f} m/s > {magnet_speed_limit_mps:.3f} m/s")

    return PostDetachGroundVerification(
        ok=not reasons,
        drone_speed_mps=drone_speed,
        magnet_speed_mps=magnet_speed,
        magnet_bottom_z_m=magnet_bottom,
        magnet_floor_error_m=floor_error,
        body_height_error_m=body_height_error,
        reasons=tuple(reasons),
    )


def _config_from_manifest(payload: dict[str, Any]) -> GroundStartConfig:
    config_payload = payload.get("config", {})
    allowed = GroundStartConfig.__dataclass_fields__.keys()
    kwargs = {name: config_payload[name] for name in allowed if name in config_payload}
    return GroundStartConfig(**kwargs)


def _latest_measured_row(csv_path: Path) -> dict[str, str]:
    latest: dict[str, str] | None = None
    with Path(csv_path).open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            required = ("drone_x", "drone_y", "drone_z", "drone_yaw_rad", "magnet_x", "magnet_y", "magnet_z")
            if all(str(row.get(key, "")).strip() for key in required):
                latest = row
    if latest is None:
        raise RuntimeError("No complete measured drone+magnet row exists in B0 telemetry")
    return latest


def _bool_text(value: str) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def _check_post_detach_from_files(
    status_csv: Path,
    manifest_path: Path,
    output_json: Path,
    max_age_s: float,
    magnet_radius_m: float,
) -> PostDetachGroundVerification:
    manifest = json.loads(Path(manifest_path).read_text(encoding="utf-8"))
    row = _latest_measured_row(status_csv)
    wall_time = float(row["wall_time_s"])
    age = time.time() - wall_time
    if not math.isfinite(age) or age > max_age_s:
        result = PostDetachGroundVerification(
            False, math.inf, math.inf, -math.inf, math.inf, math.inf,
            (f"latest measured telemetry is stale ({age:.2f} s)",),
        )
    else:
        expected_body_z = float(manifest["body_position_world_m"][2])
        result = verify_post_detach_floor_settle(
            drone_position_world=np.array([float(row["drone_x"]), float(row["drone_y"]), float(row["drone_z"])]),
            drone_velocity_world=np.array([float(row["drone_vx"]), float(row["drone_vy"]), float(row["drone_vz"])]),
            magnet_position_world=np.array([float(row["magnet_x"]), float(row["magnet_y"]), float(row["magnet_z"])]),
            magnet_speed_mps=float(row["detector_relative_speed_mps"]),
            joint_detached=_bool_text(row["joint_detached"]),
            magnet_radius_m=float(magnet_radius_m),
            expected_body_z_m=expected_body_z,
        )
    payload = {
        "schema_version": 1,
        "ok": result.ok,
        "measured_wall_time_s": wall_time,
        "check_wall_time_s": time.time(),
        "metrics": {
            "drone_speed_mps": result.drone_speed_mps,
            "magnet_speed_mps": result.magnet_speed_mps,
            "magnet_bottom_z_m": result.magnet_bottom_z_m,
            "magnet_floor_error_m": result.magnet_floor_error_m,
            "body_height_error_m": result.body_height_error_m,
        },
        "reasons": list(result.reasons),
        "measured": {key: row.get(key, "") for key in (
            "joint_detached", "drone_x", "drone_y", "drone_z",
            "drone_vx", "drone_vy", "drone_vz",
            "magnet_x", "magnet_y", "magnet_z", "detector_relative_speed_mps",
        )},
    }
    Path(output_json).write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return result


def _check_from_files(status_csv: Path, manifest_path: Path, output_json: Path, max_age_s: float) -> GroundStartVerification:
    manifest = json.loads(Path(manifest_path).read_text(encoding="utf-8"))
    row = _latest_measured_row(status_csv)
    wall_time = float(row["wall_time_s"])
    age = time.time() - wall_time
    if not math.isfinite(age) or age > max_age_s:
        result = GroundStartVerification(False, math.inf, math.inf, math.inf, -math.inf, math.inf, math.inf, (f"latest measured telemetry is stale ({age:.2f} s)",))
    else:
        result = verify_measured_ground_start(
            assigned_plate_id=int(manifest["assigned_plate_id"]),
            drone_position_world=np.array([float(row["drone_x"]), float(row["drone_y"]), float(row["drone_z"])]),
            drone_yaw_rad=float(row["drone_yaw_rad"]),
            magnet_position_world=np.array([float(row["magnet_x"]), float(row["magnet_y"]), float(row["magnet_z"])]),
            config=_config_from_manifest(manifest),
        )
    payload = {
        "schema_version": 1,
        "ok": result.ok,
        "measured_wall_time_s": wall_time,
        "check_wall_time_s": time.time(),
        "metrics": {
            "body_position_error_m": result.body_position_error_m,
            "magnet_position_error_m": result.magnet_position_error_m,
            "magnet_plate_xy_error_m": result.magnet_plate_xy_error_m,
            "magnet_bottom_z_m": result.magnet_bottom_z_m,
            "cable_angle_from_horizontal_deg": result.cable_angle_from_horizontal_deg,
            "cable_length_error_m": result.cable_length_error_m,
        },
        "reasons": list(result.reasons),
        "measured": {key: row.get(key, "") for key in (
            "drone_x", "drone_y", "drone_z", "drone_yaw_rad", "magnet_x", "magnet_y", "magnet_z"
        )},
    }
    Path(output_json).write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return result


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    generate = sub.add_parser("generate", help="Create run-local articulated X3 and world")
    generate.add_argument("--source-x3", type=Path, required=True)
    generate.add_argument("--world-template", type=Path, required=True)
    generate.add_argument("--output-dir", type=Path, required=True)
    generate.add_argument("--manifest", type=Path, required=True)
    generate.add_argument("--assigned-plate-id", type=int, default=0)
    generate.add_argument("--body-yaw", type=float, default=0.0)
    generate.add_argument(
        "--simulation-attachment-stabilization",
        action="store_true",
        help="B1 only: apply selective magnet/ring/floor collision masks and damp both tether ball joints",
    )
    generate.add_argument("--attachment-joint-damping", type=float, default=0.1)

    check = sub.add_parser("check", help="Verify measured CSV ground geometry")
    check.add_argument("--status-csv", type=Path, required=True)
    check.add_argument("--manifest", type=Path, required=True)
    check.add_argument("--output-json", type=Path, required=True)
    check.add_argument("--max-age-s", type=float, default=2.0)

    post = sub.add_parser("check-post-detach-floor", help="Verify detached magnet is measured-settled on the real floor")
    post.add_argument("--status-csv", type=Path, required=True)
    post.add_argument("--manifest", type=Path, required=True)
    post.add_argument("--output-json", type=Path, required=True)
    post.add_argument("--max-age-s", type=float, default=1.0)
    post.add_argument("--magnet-radius-m", type=float, default=0.025)
    return parser


def main(argv=None) -> int:
    args = _build_parser().parse_args(argv)
    if args.command == "generate":
        cfg = GroundStartConfig(initial_body_yaw_rad=float(args.body_yaw))
        result = generate_ground_start_assets(
            source_x3=args.source_x3,
            world_template=args.world_template,
            output_dir=args.output_dir,
            assigned_plate_id=args.assigned_plate_id,
            config=cfg,
            manifest_path=args.manifest,
            simulation_attachment_stabilization=bool(args.simulation_attachment_stabilization),
            attachment_joint_damping=float(args.attachment_joint_damping),
        )
        print(f"Generated run-local real-X3 ground start: {result.x3_path}")
        print(f"Generated run-local world: {result.world_path}")
        print(f"Ground-start manifest: {result.manifest_path}")
        return 0

    if args.command == "check-post-detach-floor":
        try:
            result = _check_post_detach_from_files(
                args.status_csv, args.manifest, args.output_json, args.max_age_s, args.magnet_radius_m
            )
        except (OSError, ValueError, KeyError, RuntimeError, json.JSONDecodeError) as exc:
            print(f"ERROR: unable to verify post-detach M2B floor settle: {exc}")
            return 1
        print(
            "Measured M2B post-detach floor settle: "
            f"ok={result.ok}, magnet_bottom_z={result.magnet_bottom_z_m:.3f} m, "
            f"magnet_speed={result.magnet_speed_mps:.3f} m/s, drone_speed={result.drone_speed_mps:.3f} m/s"
        )
        if not result.ok:
            for reason in result.reasons:
                print(f"  FAIL: {reason}")
            return 1
        return 0

    try:
        result = _check_from_files(args.status_csv, args.manifest, args.output_json, args.max_age_s)
    except (OSError, ValueError, KeyError, RuntimeError, json.JSONDecodeError) as exc:
        print(f"ERROR: unable to verify measured M2B ground geometry: {exc}")
        return 1
    print(
        "Measured M2B ground geometry: "
        f"ok={result.ok}, cable_angle={result.cable_angle_from_horizontal_deg:.2f} deg, "
        f"magnet_plate_xy={result.magnet_plate_xy_error_m:.3f} m, "
        f"magnet_bottom_z={result.magnet_bottom_z_m:.3f} m"
    )
    if not result.ok:
        for reason in result.reasons:
            print(f"  FAIL: {reason}")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())

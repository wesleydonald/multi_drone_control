#!/usr/bin/env python3
"""Generate the run-local four-X3 grounded commissioning assets for M2C.

M2C deliberately does not fly.  Four exact copies of the real X3 model are
staged around the grounded ring with their tether/magnet assemblies extending
radially *outward*.  The magnet rests on the floor, so the rigid simulation
rod is naturally near-horizontal rather than being artificially constrained to
zero inclination.

Gazebo DetachableJoint starts attached.  As in the commissioned M2B B1 setup,
the system plugins therefore live on the generated static ring and the
supervised runner commands/validates all four releases while physics is paused.
Repository source SDFs are never edited in place.
"""

from __future__ import annotations

import argparse
import copy
from dataclasses import asdict, dataclass
import hashlib
import json
import math
import os
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from .m2b_ground_start import quaternion_xyzw_from_two_vectors
from .m2b_ground_spawn import (
    _apply_simulation_attachment_stabilization,
    _format_pose,
    _generate_selective_collision_ring,
    _rotation_from_quaternion_xyzw,
    _rpy_from_rotation,
    _set_collide_bitmask,
    _yaw_rotation,
)
from .m2_fleet_topics import canonical_vehicle_topics


DRONE_IDS = (0, 1, 2, 3)
M2C_RING_YAW_DEG = 20.0
BODY_Z_M = 0.105
MAGNET_RADIUS_M = 0.025
TETHER_LENGTH_M = 0.50
BASE_JOINT_OFFSET_TETHER_Z_M = -0.05
# multi_drone_control: 0.05 for the body-centre pivot test copy (modelLargeM2BallMagnet_ctr.sdf)
TETHER_ORIGIN_INITIAL_BODY_Z_M = float(os.environ.get("M2C_TETHER_ORIGIN_BODY_Z_M", "0.01"))
ATTACHMENT_JOINT_DAMPING = 0.1
M2C_POSE_UPDATE_HZ = 120.0

# Deliberately non-symmetric staging.  This makes the 72-case assignment a real
# optimization rather than a four-way geometrical tie while keeping every cable
# outside the basket footprint.  Yaw remains a vehicle property and is not
# changed to face the ring.
DEFAULT_STAGE_RADIUS_M = (0.92, 1.00, 0.96, 1.04)
DEFAULT_STAGE_ANGLE_DEG = (101.0, 7.0, 193.0, 286.0)
DEFAULT_BODY_YAW_DEG = (12.0, -18.0, 33.0, -41.0)

# Optional deterministic M2D stress layout. Drones 0 -> 1 -> 2 start on one
# radial spoke, ordered outer -> middle -> inner so the first active vehicle must
# traverse the most cluttered initial scene. Drone 3 starts off-axis. Angles are
# resolved relative to the current ring yaw so the three-drone line remains tied
# to ring geometry rather than to an arbitrary world axis. The 0.65 m body
# spacing leaves >0.11 m clearance between each outward floor-supported magnet
# sphere and the next X3 body collision envelope in the generated source model.
M2D_HARD_LINE_STAGE_RADIUS_M = (2.25, 1.60, 0.95, 1.20)
M2D_HARD_LINE_OFF_AXIS_DEG = -95.0
STAGE_LAYOUT_NOMINAL = "nominal"
STAGE_LAYOUT_HARD_LINE = "hard_line"
STAGE_LAYOUT_CHOICES = (STAGE_LAYOUT_NOMINAL, STAGE_LAYOUT_HARD_LINE)


@dataclass(frozen=True)
class DetachableJointChannel:
    vehicle_id: str
    parent_model: str
    parent_link: str
    child_model: str
    child_link: str
    attach_topic: str
    detach_topic: str
    state_topic: str


@dataclass(frozen=True)
class GroundVehicleGeometry:
    drone_id: int
    vehicle_id: str
    body_position_world: tuple[float, float, float]
    body_yaw_rad: float
    joint_anchor_world: tuple[float, float, float]
    magnet_center_world: tuple[float, float, float]
    tether_origin_world: tuple[float, float, float]
    tether_quaternion_xyzw: tuple[float, float, float, float]
    cable_angle_from_horizontal_deg: float


@dataclass(frozen=True)
class GroundStageLayout:
    name: str
    stage_radius_m: tuple[float, float, float, float]
    stage_angle_deg: tuple[float, float, float, float]
    body_yaw_deg: tuple[float, float, float, float]


def resolve_stage_layout(name: str, *, ring_yaw_deg: float) -> GroundStageLayout:
    """Resolve one deterministic four-X3 ground staging layout.

    ``nominal`` preserves the commissioned M2C/M2D geometry byte-for-byte at
    the parameter level. ``hard_line`` is an M2D stress scene only: drones
    0/1/2 share one ring-radial azimuth and drone 3 is deliberately off-axis.
    """

    normalized = str(name).strip().lower()
    if normalized == STAGE_LAYOUT_NOMINAL:
        return GroundStageLayout(
            name=STAGE_LAYOUT_NOMINAL,
            stage_radius_m=tuple(float(v) for v in DEFAULT_STAGE_RADIUS_M),
            stage_angle_deg=tuple(float(v) for v in DEFAULT_STAGE_ANGLE_DEG),
            body_yaw_deg=tuple(float(v) for v in DEFAULT_BODY_YAW_DEG),
        )
    if normalized == STAGE_LAYOUT_HARD_LINE:
        line_angle_deg = float(ring_yaw_deg)
        return GroundStageLayout(
            name=STAGE_LAYOUT_HARD_LINE,
            stage_radius_m=tuple(float(v) for v in M2D_HARD_LINE_STAGE_RADIUS_M),
            stage_angle_deg=(
                line_angle_deg,
                line_angle_deg,
                line_angle_deg,
                line_angle_deg + M2D_HARD_LINE_OFF_AXIS_DEG,
            ),
            # Vehicle yaw is not repointed toward the ring. Preserve the same
            # identity-bound yaw values used by the commissioned nominal scene.
            body_yaw_deg=tuple(float(v) for v in DEFAULT_BODY_YAW_DEG),
        )
    raise ValueError(
        f"unknown stage layout {name!r}; expected one of {STAGE_LAYOUT_CHOICES}"
    )


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def detachable_joint_channels(drone_ids=DRONE_IDS) -> tuple[DetachableJointChannel, ...]:
    """Return the four identity-bound ring->magnet Gazebo joint channels."""

    channels: list[DetachableJointChannel] = []
    seen: set[int] = set()
    for raw in drone_ids:
        drone_id = int(raw)
        if drone_id < 0 or drone_id in seen:
            raise ValueError("drone IDs must be unique non-negative integers")
        seen.add(drone_id)
        topics = canonical_vehicle_topics(drone_id)
        channels.append(
            DetachableJointChannel(
                vehicle_id=f"drone_{drone_id}",
                parent_model="payload_model",
                parent_link="payload_link",
                child_model=f"x3_{drone_id}",
                child_link="magnet_tip_link",
                attach_topic=topics["joint_attach"],
                detach_topic=topics["joint_detach"],
                state_topic=topics["joint_state"],
            )
        )
    return tuple(channels)


def grounded_vehicle_geometry(
    drone_id: int,
    *,
    ring_position_world=(0.0, 0.0, 0.035),
    stage_radius_m: float,
    stage_angle_rad: float,
    body_yaw_rad: float,
) -> GroundVehicleGeometry:
    """Construct a floor-supported outward-tether start for one X3.

    The actual source model's body-to-ball-joint vertical offset is -0.040 m and
    its joint-to-magnet reach is 0.450 m.  The magnet sphere centre is set at its
    physical floor-supported height.  The remaining horizontal reach follows
    from those dimensions, so the resulting cable angle is not hand tuned.
    """

    drone_id = int(drone_id)
    ring = np.asarray(ring_position_world, dtype=float).reshape(3)
    radius = float(stage_radius_m)
    angle = float(stage_angle_rad)
    yaw = float(body_yaw_rad)
    if drone_id < 0 or not np.all(np.isfinite(ring)):
        raise ValueError("invalid drone/ring identity or geometry")
    if not all(math.isfinite(v) for v in (radius, angle, yaw)) or radius <= 0.0:
        raise ValueError("invalid staging geometry")

    outward = np.array([math.cos(angle), math.sin(angle), 0.0], dtype=float)
    body = ring.copy()
    body[:2] += radius * outward[:2]
    body[2] = BODY_Z_M

    joint_anchor_body_z = TETHER_ORIGIN_INITIAL_BODY_Z_M + BASE_JOINT_OFFSET_TETHER_Z_M
    joint_anchor = body.copy()
    joint_anchor[2] += joint_anchor_body_z

    joint_to_magnet = TETHER_LENGTH_M - abs(BASE_JOINT_OFFSET_TETHER_Z_M)
    magnet_z = MAGNET_RADIUS_M
    vertical_delta = magnet_z - joint_anchor[2]
    if abs(vertical_delta) >= joint_to_magnet:
        raise RuntimeError("magnet floor height exceeds available tether reach")
    horizontal_reach = math.sqrt(joint_to_magnet**2 - vertical_delta**2)
    magnet = joint_anchor.copy()
    magnet[:2] += horizontal_reach * outward[:2]
    magnet[2] = magnet_z

    cable = magnet - joint_anchor
    cable_direction = cable / np.linalg.norm(cable)
    tether_origin = joint_anchor - abs(BASE_JOINT_OFFSET_TETHER_Z_M) * cable_direction
    tether_quaternion = quaternion_xyzw_from_two_vectors(
        np.array([0.0, 0.0, -1.0], dtype=float), cable_direction
    )
    angle_horizontal = math.degrees(
        math.atan2(abs(float(cable[2])), float(np.linalg.norm(cable[:2])))
    )
    return GroundVehicleGeometry(
        drone_id=drone_id,
        vehicle_id=f"drone_{drone_id}",
        body_position_world=tuple(float(v) for v in body),
        body_yaw_rad=yaw,
        joint_anchor_world=tuple(float(v) for v in joint_anchor),
        magnet_center_world=tuple(float(v) for v in magnet),
        tether_origin_world=tuple(float(v) for v in tether_origin),
        tether_quaternion_xyzw=tuple(float(v) for v in tether_quaternion),
        cable_angle_from_horizontal_deg=float(angle_horizontal),
    )


def _write_vehicle_copy(
    *, source_x3: Path, output_path: Path, geometry: GroundVehicleGeometry
) -> ET.Element:
    source_before = source_x3.read_bytes()
    tree = ET.parse(source_x3)
    model = tree.getroot().find("model")
    if model is None:
        raise RuntimeError("source X3 SDF has no model")
    model.attrib["name"] = f"x3_{geometry.drone_id}"

    # M2C commissions identity/plumbing on the ground and does not consume the
    # X3 camera or IMU streams.  Camera entities can be removed safely, but the
    # legacy ROS simulation stack still selects X3 poses by numeric PoseArray
    # index.  Gazebo PosePublisher stores publishable entities in an
    # unordered_map keyed by entity id, so deleting the IMU entity can perturb
    # the PoseArray ordering even though sensor poses are not themselves
    # published.  Preserve the inert IMU descriptor to keep entity topology /
    # legacy pose indices stable; the generated M2C world removes the generic
    # Sensors and Imu systems below, so no IMU simulation work is performed.
    # The authoritative source X3 remains untouched for other simulations / IRL.
    for link in model.findall(".//link"):
        for sensor in list(link.findall("sensor")):
            sensor_type = (sensor.attrib.get("type") or "").strip().lower()
            if sensor_type == "camera":
                link.remove(sensor)

    tether_pose = model.find("./link[@name='tether_rod']/pose")
    if tether_pose is None or tether_pose.attrib.get("relative_to") != "X3/base_link":
        raise RuntimeError("source X3 does not expose the expected tether_rod pose")

    body = np.asarray(geometry.body_position_world, dtype=float)
    tether_origin = np.asarray(geometry.tether_origin_world, dtype=float)
    R_wb = _yaw_rotation(geometry.body_yaw_rad)
    R_wt = _rotation_from_quaternion_xyzw(
        np.asarray(geometry.tether_quaternion_xyzw, dtype=float)
    )
    p_bt = R_wb.T @ (tether_origin - body)
    R_bt = R_wb.T @ R_wt
    roll, pitch, yaw = _rpy_from_rotation(R_bt)
    tether_pose.text = _format_pose([p_bt[0], p_bt[1], p_bt[2], roll, pitch, yaw])

    source_plugin = model.find("./plugin[@name='gz::sim::systems::DetachableJoint']")
    if source_plugin is None:
        raise RuntimeError("source X3 is missing the DetachableJoint system")
    plugin_copy = copy.deepcopy(source_plugin)
    model.remove(source_plugin)

    tree.write(output_path, encoding="utf-8", xml_declaration=True)
    _apply_simulation_attachment_stabilization(output_path, ATTACHMENT_JOINT_DAMPING)
    if source_x3.read_bytes() != source_before:
        raise RuntimeError("source X3 changed during M2C generation")
    return plugin_copy


def _append_joint_plugin(
    ring_model: ET.Element,
    source_plugin: ET.Element,
    channel: DetachableJointChannel,
) -> None:
    plugin = copy.deepcopy(source_plugin)
    fields = {
        "parent_link": channel.parent_link,
        "child_model": channel.child_model,
        "child_link": channel.child_link,
        "detach_topic": channel.detach_topic,
        "attach_topic": channel.attach_topic,
        "output_topic": channel.state_topic,
    }
    for key, value in fields.items():
        node = plugin.find(key)
        if node is None:
            node = ET.SubElement(plugin, key)
        node.text = value
    ring_model.append(plugin)


def generate_m2c_assets(
    *,
    source_x3: Path,
    source_ring: Path,
    world_template: Path,
    output_dir: Path,
    ring_yaw_deg: float = M2C_RING_YAW_DEG,
    manifest_path: Path | None = None,
    stage_layout: str = STAGE_LAYOUT_NOMINAL,
) -> dict:
    """Generate four X3 copies, a four-joint ring, and an M2C world."""

    source_x3 = Path(source_x3).resolve()
    source_ring = Path(source_ring).resolve()
    world_template = Path(world_template).resolve()
    output_dir = Path(output_dir).resolve()
    for path in (source_x3, source_ring, world_template):
        if not path.is_file():
            raise FileNotFoundError(path)
    output_dir.mkdir(parents=True, exist_ok=True)
    ring_yaw = math.radians(float(ring_yaw_deg))
    layout = resolve_stage_layout(stage_layout, ring_yaw_deg=float(ring_yaw_deg))

    geometries = tuple(
        grounded_vehicle_geometry(
            drone_id,
            stage_radius_m=layout.stage_radius_m[drone_id],
            stage_angle_rad=math.radians(layout.stage_angle_deg[drone_id]),
            body_yaw_rad=math.radians(layout.body_yaw_deg[drone_id]),
        )
        for drone_id in DRONE_IDS
    )
    channels = detachable_joint_channels(DRONE_IDS)

    vehicle_paths: list[Path] = []
    plugins: list[ET.Element] = []
    for geometry in geometries:
        path = output_dir / f"m2c_x3_{geometry.drone_id}.sdf"
        plugins.append(
            _write_vehicle_copy(source_x3=source_x3, output_path=path, geometry=geometry)
        )
        vehicle_paths.append(path)

    ring_path = output_dir / "m2c_ring_fixture.sdf"
    _generate_selective_collision_ring(source_ring, ring_path)
    ring_tree = ET.parse(ring_path)
    ring_model = ring_tree.getroot().find("model")
    if ring_model is None or ring_model.find("./link[@name='payload_link']") is None:
        raise RuntimeError("generated ring is missing payload_link")
    if (ring_model.findtext("static") or "").strip().lower() != "true":
        raise RuntimeError("M2C ring must remain static")
    for plugin, channel in zip(plugins, channels):
        _append_joint_plugin(ring_model, plugin, channel)
    ring_tree.write(ring_path, encoding="utf-8", xml_declaration=True)

    world_tree = ET.parse(world_template)
    root = world_tree.getroot()
    world = root.find("world")
    if world is None:
        raise RuntimeError("world template has no <world>")
    ring_include = None
    x3_include = None
    for include in list(world.findall("include")):
        uri = (include.findtext("uri") or "").strip()
        if uri == "m2a_ring_fixture.sdf":
            ring_include = include
        elif uri == "modelLargeM2BallMagnet.sdf":
            x3_include = include
    if ring_include is None or x3_include is None:
        raise RuntimeError("world template must contain one ring and one real X3 include")

    ring_include.find("uri").text = ring_path.name
    ring_pose = ring_include.find("pose")
    if ring_pose is None:
        ring_pose = ET.SubElement(ring_include, "pose")
    ring_pose.text = _format_pose([0.0, 0.0, 0.035, 0.0, 0.0, ring_yaw])

    # The template historically carries generic Gazebo IMU and rendering-sensor
    # systems inside the single-X3 <include>.  M2C removes camera entities and
    # preserves only an inert IMU descriptor for PoseArray topology stability.
    # Remove these server-side systems so that descriptor performs no sensor
    # simulation work and is not cloned four times.
    def _generic_sensor_system(plugin: ET.Element) -> bool:
        identity = (
            (plugin.attrib.get("name") or "")
            + " "
            + (plugin.attrib.get("filename") or "")
        ).lower()
        return (
            "systems::sensors" in identity
            or "sensors-system" in identity
            or "systems::imu" in identity
            or "imu-system" in identity
        )

    generic_sensor_plugins = [
        plugin for plugin in x3_include.findall("plugin") if _generic_sensor_system(plugin)
    ]
    if len(generic_sensor_plugins) != 2:
        raise RuntimeError(
            "world template must expose exactly one generic Sensors and one Imu system "
            "on the single-X3 include"
        )
    for plugin in list(generic_sensor_plugins):
        x3_include.remove(plugin)

    def _set_pose_update_rate(include: ET.Element, update_hz: float) -> None:
        matched = 0
        for plugin in include.findall("plugin"):
            identity = (
                (plugin.attrib.get("name") or "")
                + " "
                + (plugin.attrib.get("filename") or "")
            ).lower()
            if "posepublisher" not in identity and "pose-publisher" not in identity:
                continue
            node = plugin.find("update_frequency")
            if node is None:
                node = ET.SubElement(plugin, "update_frequency")
            node.text = f"{float(update_hz):.12g}"
            matched += 1
        if matched != 1:
            raise RuntimeError("each M2C ring/X3 include must expose exactly one PosePublisher")

    _set_pose_update_rate(ring_include, M2C_POSE_UPDATE_HZ)
    _set_pose_update_rate(x3_include, M2C_POSE_UPDATE_HZ)

    insertion_index = list(world).index(x3_include)
    world.remove(x3_include)
    for offset, (path, geometry) in enumerate(zip(vehicle_paths, geometries)):
        include = copy.deepcopy(x3_include)
        include.find("uri").text = path.name
        name_node = include.find("name")
        if name_node is None:
            name_node = ET.Element("name")
            include.insert(1, name_node)
        name_node.text = f"x3_{geometry.drone_id}"
        pose_node = include.find("pose")
        if pose_node is None:
            pose_node = ET.SubElement(include, "pose")
        body = geometry.body_position_world
        pose_node.text = _format_pose([body[0], body[1], body[2], 0.0, 0.0, geometry.body_yaw_rad])
        for plugin in include.findall("plugin"):
            robot_ns = plugin.find("robotNamespace")
            if robot_ns is not None:
                robot_ns.text = f"drone_{geometry.drone_id}"
        world.insert(insertion_index + offset, include)

    floor_collision = root.find(".//model[@name='floor']/link/collision[@name='col']")
    if floor_collision is None:
        raise RuntimeError("world template is missing floor::link::col")
    _set_collide_bitmask(floor_collision, 0x0003)

    world_path = output_dir / "m2c_four_x3_ground.sdf"
    world_tree.write(world_path, encoding="utf-8", xml_declaration=True)

    manifest = {
        "schema_version": 1,
        "mission_stage": "M2C",
        "ground_only": True,
        "run_local_camera_sensors_enabled": False,
        "run_local_imu_sensors_enabled": False,
        "run_local_pose_update_hz": float(M2C_POSE_UPDATE_HZ),
        "ring_yaw_deg": float(ring_yaw_deg),
        "ring_position_world_m": [0.0, 0.0, 0.035],
        "stage_layout": layout.name,
        "stage_radius_m": list(layout.stage_radius_m),
        "stage_angle_deg": list(layout.stage_angle_deg),
        "body_yaw_deg": list(layout.body_yaw_deg),
        "source_x3": str(source_x3),
        "source_x3_sha256": _sha256(source_x3),
        "source_ring": str(source_ring),
        "source_ring_sha256": _sha256(source_ring),
        "world_template": str(world_template),
        "world_template_sha256": _sha256(world_template),
        "generated_world": str(world_path),
        "generated_ring": str(ring_path),
        "generated_x3": [str(path) for path in vehicle_paths],
        "vehicles": [asdict(value) for value in geometries],
        "detachable_joint_channels": [asdict(value) for value in channels],
        "selective_collision_policy": "magnet_floor_yes_magnet_ring_no",
        "magnet_collision_bitmask": "0x0001",
        "ring_collision_bitmask": "0x0002",
        "floor_collision_bitmask": "0x0003",
        "attachment_joint_damping": ATTACHMENT_JOINT_DAMPING,
        "source_assets_modified": False,
    }
    target = (
        Path(manifest_path).resolve()
        if manifest_path is not None
        else output_dir.parent / "m2c_ground_manifest.json"
    )
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return manifest


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    generate = sub.add_parser("generate")
    generate.add_argument("--source-x3", required=True, type=Path)
    generate.add_argument("--source-ring", required=True, type=Path)
    generate.add_argument("--world-template", required=True, type=Path)
    generate.add_argument("--output-dir", required=True, type=Path)
    generate.add_argument("--manifest", type=Path, default=None)
    generate.add_argument("--ring-yaw-deg", type=float, default=M2C_RING_YAW_DEG)
    generate.add_argument(
        "--stage-layout",
        choices=STAGE_LAYOUT_CHOICES,
        default=STAGE_LAYOUT_NOMINAL,
        help="four-X3 ground staging geometry; nominal preserves commissioned M2C/M2D",
    )
    return parser


def main(argv=None) -> int:
    args = _parser().parse_args(argv)
    if args.command == "generate":
        manifest = generate_m2c_assets(
            source_x3=args.source_x3,
            source_ring=args.source_ring,
            world_template=args.world_template,
            output_dir=args.output_dir,
            manifest_path=args.manifest,
            ring_yaw_deg=args.ring_yaw_deg,
            stage_layout=args.stage_layout,
        )
        print(
            json.dumps(
                {
                    "generated_world": manifest["generated_world"],
                    "ring_yaw_deg": manifest["ring_yaw_deg"],
                    "stage_layout": manifest["stage_layout"],
                }
            )
        )
        return 0
    raise RuntimeError(args.command)


if __name__ == "__main__":
    raise SystemExit(main())

"""M2B B1 static-ring-parent DetachableJoint discriminator contracts.

The Sep-10 16:31 selective-collision latch run passed bootstrap and approach but
still produced an immediate post-latch rigid-joint transient.  This increment
changes only the run-local B1 DetachableJoint parent/child topology so the
static ring is the parent and the dynamic X3 magnet link is the child.  Mass,
inertia, collision filtering, damping, mission, controller, and detector
contracts remain unchanged.
"""

from pathlib import Path
import xml.etree.ElementTree as ET

from tejen_mission.m2b_ground_spawn import generate_ground_start_assets


ROOT = Path(__file__).resolve().parents[3]
X3 = ROOT / "simulation_assets" / "tejen" / "modelLargeM2BallMagnet.sdf"
RING = ROOT / "simulation_assets" / "tejen" / "m2a_ring_fixture.sdf"
WORLD = ROOT / "simulation_assets" / "tejen" / "world_m2b_single_attachment.sdf"
DETACHABLE_NAME = "gz::sim::systems::DetachableJoint"


def _model(path: Path) -> ET.Element:
    model = ET.parse(path).getroot().find("model")
    assert model is not None
    return model


def _inertial_signature(model: ET.Element) -> dict[str, tuple[str | None, tuple[str | None, ...]]]:
    result = {}
    for link in model.findall("link"):
        inertial = link.find("inertial")
        if inertial is None:
            result[link.attrib["name"]] = (None, ())
            continue
        mass = inertial.findtext("mass")
        inertia = inertial.find("inertia")
        values = () if inertia is None else tuple(
            inertia.findtext(tag) for tag in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")
        )
        result[link.attrib["name"]] = (mass, values)
    return result


def test_b1_generated_assets_move_exactly_one_detachable_joint_to_static_ring_parent(tmp_path):
    before = {path: path.read_bytes() for path in (X3, RING, WORLD)}
    result = generate_ground_start_assets(
        source_x3=X3,
        world_template=WORLD,
        output_dir=tmp_path,
        assigned_plate_id=0,
        simulation_attachment_stabilization=True,
        attachment_joint_damping=0.1,
    )
    assert {path: path.read_bytes() for path in (X3, RING, WORLD)} == before

    generated_x3 = _model(result.x3_path)
    assert generated_x3.find(f"./plugin[@name='{DETACHABLE_NAME}']") is None

    assert result.ring_path is not None
    generated_ring = _model(result.ring_path)
    assert generated_ring.findtext("static") == "true"
    plugins = generated_ring.findall(f"./plugin[@name='{DETACHABLE_NAME}']")
    assert len(plugins) == 1
    plugin = plugins[0]
    assert plugin.findtext("parent_link") == "payload_link"
    assert plugin.findtext("child_model") == "x3"
    assert plugin.findtext("child_link") == "magnet_tip_link"


def test_topology_discriminator_preserves_detachable_topics_and_fixed_joint_plugin_type(tmp_path):
    source_plugin = _model(X3).find(f"./plugin[@name='{DETACHABLE_NAME}']")
    assert source_plugin is not None
    result = generate_ground_start_assets(
        source_x3=X3,
        world_template=WORLD,
        output_dir=tmp_path,
        assigned_plate_id=0,
        simulation_attachment_stabilization=True,
    )
    assert result.ring_path is not None
    moved = _model(result.ring_path).find(f"./plugin[@name='{DETACHABLE_NAME}']")
    assert moved is not None
    assert moved.attrib.get("filename") == source_plugin.attrib.get("filename")
    for tag in ("detach_topic", "attach_topic", "output_topic"):
        assert moved.findtext(tag) == source_plugin.findtext(tag)


def test_topology_discriminator_does_not_change_x3_or_ring_inertial_properties(tmp_path):
    source_x3_inertia = _inertial_signature(_model(X3))
    source_ring_inertia = _inertial_signature(_model(RING))
    result = generate_ground_start_assets(
        source_x3=X3,
        world_template=WORLD,
        output_dir=tmp_path,
        assigned_plate_id=0,
        simulation_attachment_stabilization=True,
    )
    assert _inertial_signature(_model(result.x3_path)) == source_x3_inertia
    assert result.ring_path is not None
    assert _inertial_signature(_model(result.ring_path)) == source_ring_inertia
    assert _model(result.ring_path).findtext("static") == "true"


def test_b0_default_retains_source_x3_parent_topology(tmp_path):
    result = generate_ground_start_assets(
        source_x3=X3,
        world_template=WORLD,
        output_dir=tmp_path,
        assigned_plate_id=0,
        simulation_attachment_stabilization=False,
    )
    assert result.ring_path is None
    plugin = _model(result.x3_path).find(f"./plugin[@name='{DETACHABLE_NAME}']")
    assert plugin is not None
    assert plugin.findtext("parent_link") == "magnet_tip_link"
    assert plugin.findtext("child_model") == "payload_model"
    assert plugin.findtext("child_link") == "payload_link"

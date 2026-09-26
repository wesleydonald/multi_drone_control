"""Source-level simulation contracts for M2A before the SDFs are implemented."""

from __future__ import annotations

from pathlib import Path
import xml.etree.ElementTree as ET

import pytest


REPO_ROOT = Path(__file__).resolve().parents[3]
SIM = REPO_ROOT / "simulation_assets" / "tejen"
M1_MODEL = SIM / "modelLargeWithDetachablePayload.sdf"
M2_MODEL = SIM / "modelLargeM2BallMagnet.sdf"
M2_FIXTURE = SIM / "m2a_ring_fixture.sdf"
M2_WORLD = SIM / "world_m2a_attachment_foundations.sdf"


def _root(path: Path) -> ET.Element:
    assert path.is_file(), f"M2A test-first RED: expected file does not exist yet: {path}"
    return ET.parse(path).getroot()


def _model(root: ET.Element) -> ET.Element:
    model = root.find("model")
    assert model is not None
    return model


def _joint(model: ET.Element, name: str) -> ET.Element:
    joint = model.find(f"joint[@name='{name}']")
    assert joint is not None, f"missing joint {name!r}"
    return joint


def _plugin(model: ET.Element, *, name_contains: str) -> ET.Element:
    for plugin in model.findall("plugin"):
        if name_contains in plugin.attrib.get("name", ""):
            return plugin
    pytest.fail(f"missing plugin containing {name_contains!r}")


def test_existing_m1_model_remains_universal_plus_fixed() -> None:
    model = _model(_root(M1_MODEL))
    base_joint = _joint(model, "base_to_tether")
    tip_joint = _joint(model, "tether_to_magnet_tip")

    assert base_joint.attrib.get("type") == "universal"
    assert tip_joint.attrib.get("type") == "fixed"
    assert base_joint.findtext("parent") == "X3/base_link"
    assert base_joint.findtext("child") == "tether_rod"
    assert tip_joint.findtext("parent") == "tether_rod"
    assert tip_joint.findtext("child") == "magnet_tip_link"


def test_new_m2_model_uses_ball_joints_at_both_tether_ends() -> None:
    model = _model(_root(M2_MODEL))
    base_joint = _joint(model, "base_to_tether")
    tip_joint = _joint(model, "tether_to_magnet_tip")

    assert base_joint.attrib.get("type") == "ball"
    assert tip_joint.attrib.get("type") == "ball"
    assert base_joint.findtext("parent") == "X3/base_link"
    assert base_joint.findtext("child") == "tether_rod"
    assert tip_joint.findtext("parent") == "tether_rod"
    assert tip_joint.findtext("child") == "magnet_tip_link"


def test_new_m2_model_keeps_nominal_0p5m_rigid_tether() -> None:
    model = _model(_root(M2_MODEL))
    tether = model.find("link[@name='tether_rod']")
    assert tether is not None

    lengths = [
        float(element.text)
        for element in tether.findall(".//cylinder/length")
        if element.text is not None
    ]
    assert lengths, "tether_rod must contain a cylinder length"
    assert all(length == pytest.approx(0.50) for length in lengths)


def test_new_m2_detachable_joint_welds_magnet_tip_not_tether_body() -> None:
    model = _model(_root(M2_MODEL))
    plugin = _plugin(model, name_contains="DetachableJoint")

    assert plugin.findtext("parent_link") == "magnet_tip_link"
    assert plugin.findtext("attach_topic") == "/payload/attach"
    assert plugin.findtext("detach_topic") == "/payload/detach"
    assert plugin.findtext("output_topic") == "/payload/detachable_joint_state"


def test_m2a_ring_fixture_is_static_and_exposes_twelve_named_plate_elements() -> None:
    model = _model(_root(M2_FIXTURE))
    assert (model.findtext("static") or "").strip().lower() == "true"

    named_elements = []
    for element in model.iter():
        name = element.attrib.get("name", "")
        if name.startswith("plate_"):
            named_elements.append(name)

    plate_ids = set()
    for name in named_elements:
        tokens = name.split("_")
        if len(tokens) >= 2 and tokens[1].isdigit():
            plate_ids.add(int(tokens[1]))

    assert plate_ids == set(range(12)), (
        "M2A fixture must expose explicitly identifiable plate_0 ... plate_11 geometry"
    )


def test_m2a_world_is_parseable_and_references_m2_vehicle_and_ring_fixture() -> None:
    root = _root(M2_WORLD)
    world = root.find("world")
    assert world is not None

    text = M2_WORLD.read_text(encoding="utf-8")
    assert "modelLargeM2BallMagnet.sdf" in text
    assert "m2a_x3_support.sdf" in text
    assert "m2a_kinematic_carrier" not in text
    assert "m2a_ring_fixture" in text

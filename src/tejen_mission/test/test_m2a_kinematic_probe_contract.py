from pathlib import Path
import re
import xml.etree.ElementTree as ET


REPO = Path(__file__).resolve().parents[3]
M2_MODEL = REPO / "simulation_assets" / "tejen" / "modelLargeM2BallMagnet.sdf"
M1_MODEL = REPO / "simulation_assets" / "tejen" / "modelLargeWithDetachablePayload.sdf"
PROBE = REPO / "tools" / "sim_test" / "m2a_attachment_probe.py"
REVERT = REPO / "tools" / "sim_test" / "apply_m2a_paused_bootstrap_revert.py"


def _link(root, name: str):
    for link in root.findall(".//link"):
        if link.attrib.get("name") == name:
            return link
    raise AssertionError(f"link {name!r} not found")


def test_m2a_real_x3_base_is_not_kinematic_after_revert():
    root = ET.parse(M2_MODEL).getroot()
    base = _link(root, "X3/base_link")
    tether = _link(root, "tether_rod")
    magnet = _link(root, "magnet_tip_link")

    assert base.findtext("kinematic", default="false").strip().lower() != "true"
    assert tether.findtext("kinematic", default="false").strip().lower() != "true"
    assert magnet.findtext("kinematic", default="false").strip().lower() != "true"


def test_m2a_ball_joints_are_preserved():
    root = ET.parse(M2_MODEL).getroot()
    joints = {j.attrib.get("name"): j.attrib.get("type") for j in root.findall(".//joint")}
    assert joints["base_to_tether"] == "ball"
    assert joints["tether_to_magnet_tip"] == "ball"


def test_m1_model_remains_nonkinematic():
    root = ET.parse(M1_MODEL).getroot()
    base = _link(root, "X3/base_link")
    assert base.findtext("kinematic", default="false").strip().lower() != "true"


def test_probe_contact_height_matches_real_m2_geometry():
    text = PROBE.read_text(encoding="utf-8")
    match = re.search(
        r'add_argument\("--contact-z",\s*type=float,\s*default=([0-9.]+)',
        text,
    )
    assert match is not None
    assert abs(float(match.group(1)) - 0.675) < 1e-12


def test_revert_helper_is_surgical_and_does_not_replace_x3_model():
    text = REVERT.read_text(encoding="utf-8")
    assert "MARKER_BLOCK" in text
    assert "refusing to modify an unknown model edit" in text
    assert "m2a_kinematic_carrier.sdf" in text
    assert "modelLargeM2BallMagnet.sdf" in text
    assert "write_bytes" not in text


def test_probe_still_has_no_flight_reference_authority():
    text = PROBE.read_text(encoding="utf-8")
    assert 'ros_string("/join_planner/reference"' not in text
    assert 'ros_bool("/join_planner/reference"' not in text

"""Source-level RED tests for the M1 ring integration contract.

These intentionally avoid importing the ROS node so they run in plain pytest. They
encode ownership/separation rules that Step 3 must satisfy in the active M1 source.
"""

from pathlib import Path
import re

import yaml


ROOT = Path(__file__).resolve().parents[3]
FAKE_WORLD = ROOT / "src/tejen_mission/tejen_mission/fake_cooperative_transport_world.py"
SYSTEM_LAUNCH = ROOT / "src/tejen_mission/launch/c1f6_system_test.launch.py"
BACKEND_CONFIG = ROOT / "src/tejen_dynamic_planner/config/c1f6_moving_rendezvous.yaml"
BACKEND_SOURCE = ROOT / "src/tejen_dynamic_planner/src/transfer_backend_node.cpp"


def _text(path: Path) -> str:
    return path.read_text()


def _backend_params() -> dict:
    data = yaml.safe_load(BACKEND_CONFIG.read_text())
    # ROS parameter files are one node name -> ros__parameters.
    assert isinstance(data, dict) and len(data) == 1
    node = next(iter(data.values()))
    return node["ros__parameters"]


def test_active_fake_world_declares_ring_parameters_not_legacy_attachment_offsets() -> None:
    source = _text(FAKE_WORLD)
    required = (
        "ring_plate_count",
        "ring_plate_pitch_diameter_m",
        "ring_plate_diameter_m",
        "ring_attachment_plate_index",
        "ring_collision_outer_diameter_m",
        "ring_collision_top_offset_m",
        "ring_collision_bottom_offset_m",
    )
    for name in required:
        assert f"declare_parameter('{name}'" in source or f'declare_parameter("{name}"' in source

    # The M1 attachment target must stop using the old arbitrary body-frame tuple.
    assert "self.declare_parameter('attachment_offset_x', 0.30)" not in source
    assert "self.declare_parameter('attachment_offset_y', 0.00)" not in source
    assert "self.declare_parameter('attachment_offset_z', -0.10)" not in source


def test_active_fake_world_uses_shared_ring_geometry_for_plate_zero_attachment() -> None:
    source = _text(FAKE_WORLD)
    assert "RingNetGeometry" in source
    assert "ring_attachment_plate_index" in source
    assert re.search(r"plate_(body_offset|position_world)\(", source), (
        "fake world should derive /fake_attachment_point from RingNetGeometry rather than "
        "reimplementing the plate transform"
    )


def test_m1_delivery_target_remains_payload_ring_centre() -> None:
    launch = _text(SYSTEM_LAUNCH)
    assert '"drop_point_source": "payload"' in launch
    assert '"payload_pose_topic": "/fake_payload/pose"' in launch
    assert '"payload_twist_topic": "/fake_payload/twist"' in launch

    # Plate 0 is for the later reattachment target, not the C++ transport destination.
    assert '"drop_point_source": "attachment"' not in launch

def test_later_reattachment_still_consumes_fake_attachment_point_topics() -> None:
    planner = _text(ROOT / "src/tejen_mission/tejen_mission/online_join_planner.py")
    assert '"attachment_pose_topic", "/fake_attachment_point/pose"' in planner
    assert '"attachment_twist_topic", "/fake_attachment_point/twist"' in planner


def test_ring_parameters_are_explicit_in_m1_launch_for_reproducibility() -> None:
    launch = _text(SYSTEM_LAUNCH)
    expected_pairs = (
        ('"ring_plate_count": 12',),
        ('"ring_plate_pitch_diameter_m": 0.50', '"ring_plate_pitch_diameter_m": 0.5'),
        ('"ring_plate_diameter_m": 0.06',),
        ('"ring_attachment_plate_index": 0',),
        ('"ring_collision_outer_diameter_m": 0.56',),
        ('"ring_collision_top_offset_m": 0.0',),
        ('"ring_collision_bottom_offset_m": -0.27',),
    )
    for alternatives in expected_pairs:
        assert any(item in launch for item in alternatives), alternatives


def test_backend_config_uses_ring_envelope_parameters_instead_of_old_cuboid_half_extents() -> None:
    params = _backend_params()
    assert params["moving_basket_collision_outer_diameter_m"] == 0.56
    assert params["moving_basket_collision_top_offset_m"] == 0.0
    assert params["moving_basket_collision_bottom_offset_m"] == -0.27

    assert "moving_basket_half_x_m" not in params
    assert "moving_basket_half_y_m" not in params
    assert "moving_basket_half_z_m" not in params


def test_ring_commitment_topic_stays_the_single_ring_centre_future() -> None:
    params = _backend_params()
    assert params["moving_basket_committed_trajectory_topic"] == "/fake_payload/committed_trajectory"

    fake_source = _text(FAKE_WORLD)
    assert "payload_committed_trajectory_topic" in fake_source
    assert "/fake_payload/committed_trajectory" in fake_source

    backend_source = _text(BACKEND_SOURCE)
    # Post-M2 refactor: keep the advertised commitment exactly at the true ring
    # centre for target prediction, and translate only the collision-world copy.
    assert re.search(r"CommittedTrajectory trajectory = trajectoryFromMessage\(\s*\*msg\s*\)", backend_source)
    assert "moving_basket_trajectory_sample_.trajectory = std::move(trajectory)" in backend_source
    assert "translatedCollisionTrajectory(\n                sample.trajectory, moving_basket_geometry_.centerOffset())" in backend_source
    assert "moving_basket_trajectory_sample_.trajectory.evaluate(reference_time_s)" in backend_source


def test_visual_payload_defaults_are_ring_named_not_rectangular_payload_dimensions() -> None:
    source = _text(FAKE_WORLD)
    assert "payload_size_x" not in source
    assert "payload_size_y" not in source
    assert "payload_size_z" not in source
    assert "ring_plate_pitch_diameter_m" in source
    assert "ring_plate_diameter_m" in source

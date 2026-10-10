"""Focused physical-ring calibration and RViz contracts for two-drone IRL."""

from copy import deepcopy
from pathlib import Path

import pytest
import yaml

from tejen_mission.cooperative_trajectory import RingNetGeometry
from tejen_mission.m2_irl_launch_config import parse_irl_ring_geometry


PACKAGE = Path(__file__).resolve().parents[1]
CONFIG = PACKAGE / "config" / "m2_irl_two_drone.yaml"
LAUNCH = PACKAGE / "launch" / "m2_irl_two_drone.launch.py"
MANAGER = PACKAGE / "tejen_mission" / "m2_irl_fleet_manager.py"
OBSERVER = PACKAGE / "tejen_mission" / "m2_attachment_observer.py"
PLANNER = PACKAGE / "tejen_mission" / "online_join_planner.py"


def _payload():
    return yaml.safe_load(CONFIG.read_text(encoding="utf-8"))


def test_measured_annulus_derives_irl_plate_pitch_without_changing_simulation_default():
    ring = parse_irl_ring_geometry(_payload())
    assert ring.inner_diameter_m == pytest.approx(0.385)
    assert ring.outer_diameter_m == pytest.approx(0.510)
    assert ring.plate_diameter_m == pytest.approx(0.060)
    assert ring.plate_pitch_diameter_m == pytest.approx(0.4475)
    assert ring.plate_pitch_radius_m == pytest.approx(0.22375)
    assert ring.radial_width_m == pytest.approx(0.0625)
    assert RingNetGeometry().plate_pitch_diameter_m == pytest.approx(0.50)


@pytest.mark.parametrize("key,value", [
    ("ring_inner_diameter_m", None),
    ("ring_inner_diameter_m", 0.520),
    ("ring_outer_diameter_m", float("nan")),
    ("ring_plate_diameter_m", 0.070),
])
def test_bad_irl_ring_dimensions_fail_before_launch(key, value):
    payload = deepcopy(_payload())
    if value is None:
        del payload["m2_irl"]["geometry"][key]
    else:
        payload["m2_irl"]["geometry"][key] = value
    with pytest.raises(ValueError, match="ring"):
        parse_irl_ring_geometry(payload)


def test_irl_launch_wires_one_ring_pitch_to_assignment_observer_planners_and_cpp():
    launch = LAUNCH.read_text(encoding="utf-8")
    assert "ring_geometry = parse_irl_ring_geometry(payload)" in launch
    assert launch.count('"ring_plate_pitch_diameter_m": ring_geometry.plate_pitch_diameter_m') == 3
    assert '"cooperative_ring_plate_pitch_diameter_m": ring_geometry.plate_pitch_diameter_m' in launch
    assert '"moving_ring_segment_center_radius_m": ring_geometry.plate_pitch_radius_m' in launch
    assert '"moving_ring_segment_radial_width_m": ring_geometry.radial_width_m' in launch
    for source in (MANAGER, OBSERVER, PLANNER):
        assert 'declare_parameter("ring_plate_pitch_diameter_m", 0.50)' in source.read_text(encoding="utf-8")


def test_existing_rviz_assignment_topic_shows_numbered_plates_and_frozen_pair():
    manager = MANAGER.read_text(encoding="utf-8")
    assert '"/m2c/assignment_markers"' in manager
    assert "for plate_index in range(self.ring_geometry.plate_count)" in manager
    assert "plate_position_world(" in manager
    assert "assignment.vehicle_to_plate" in manager
    rviz = (PACKAGE.parent / "drone_visualisation" / "rviz" / "m2d_four_drone.rviz").read_text(encoding="utf-8")
    assert "/m2c/assignment_markers" in rviz



def test_constructed_mission_passes_the_same_measured_ring_to_both_drone_stacks(tmp_path):
    import importlib.util
    from launch import LaunchContext
    from launch.utilities import perform_substitutions

    payload = _payload()
    config = tmp_path / "m2_irl_ring.yaml"
    config.write_text(yaml.safe_dump(payload), encoding="utf-8")
    spec = importlib.util.spec_from_file_location("m2_irl_ring_launch", LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    context.launch_configurations.update({
        "mode": "mission", "config_file": str(config),
        "log_dir": str(tmp_path / "logs"), "rviz": "false",
    })
    actions = module._launch_setup(context)
    names = {
        "ring_plate_pitch_diameter_m", "ring_plate_diameter_m",
        "cooperative_ring_plate_pitch_diameter_m",
        "moving_ring_segment_center_radius_m", "moving_ring_segment_radial_width_m",
        "candidate_normal_m", "proof_normal_m",
        "m2b_capture_hold_normal_m",
    }
    parameters = []
    for action in actions:
        for node in (action, *getattr(action, "actions", [])):
            if not hasattr(node, "_Node__parameters"):
                continue
            values = {}
            for key, value in node._Node__parameters[0].items():
                name = perform_substitutions(context, key)
                if name in names:
                    values[name] = float(perform_substitutions(context, value) if isinstance(value, (list, tuple)) else value)
            parameters.append(values)
    ring_consumers = [values for values in parameters if "ring_plate_pitch_diameter_m" in values]
    assert len(ring_consumers) == 5  # manager + two observers + two planners
    assert all(values["ring_plate_pitch_diameter_m"] == pytest.approx(0.4475) for values in ring_consumers)
    assert all(values["ring_plate_diameter_m"] == pytest.approx(0.060) for values in ring_consumers)
    backends = [values for values in parameters if "cooperative_ring_plate_pitch_diameter_m" in values]
    assert len(backends) == 2
    for values in backends:
        assert values["cooperative_ring_plate_pitch_diameter_m"] == pytest.approx(0.4475)
        assert values["moving_ring_segment_center_radius_m"] == pytest.approx(0.22375)
        assert values["moving_ring_segment_radial_width_m"] == pytest.approx(0.0625)
    observers = [values for values in parameters if "candidate_normal_m" in values]
    assert len(observers) == 2
    assert all(values["candidate_normal_m"] == pytest.approx(0.015) for values in observers)
    assert all(values["proof_normal_m"] == pytest.approx(0.015) for values in observers)
    planners = [values for values in parameters if "m2b_capture_hold_normal_m" in values]
    assert len(planners) == 2
    assert all(values["m2b_capture_hold_normal_m"] == pytest.approx(0.015) for values in planners)



def test_rviz_markers_render_all_plate_centres_and_the_frozen_opposite_pair():
    from types import SimpleNamespace

    import numpy as np
    from builtin_interfaces.msg import Time

    from tejen_mission.m2_irl_fleet_manager import M2IRLFleetManager

    published = []
    manager = SimpleNamespace(
        ring_position=np.zeros(3),
        ring_rotation=np.eye(3),
        ring_stamp_s=10.0,
        ring_timeout_s=0.5,
        ring_frame_id="map",
        ring_geometry=RingNetGeometry(plate_pitch_diameter_m=0.4475),
        assignment_latch=SimpleNamespace(assignment=SimpleNamespace(
            vehicle_to_plate={"drone_0": 0, "drone_1": 6}
        )),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: Time(sec=10))),
        markers_pub=SimpleNamespace(publish=published.append),
    )
    M2IRLFleetManager._publish_markers(manager, 10.1)
    assert len(published) == 1
    markers = published[0].markers
    assert len(markers) == 24
    assert markers[0].pose.position.x == pytest.approx(0.22375)
    assert markers[12].pose.position.x == pytest.approx(-0.22375)
    assert markers[1].text == "0 drone_0"
    assert markers[13].text == "6 drone_1"
    M2IRLFleetManager._publish_markers(manager, 11.0)
    assert len(published) == 1  # stale ring cannot refresh visible targets

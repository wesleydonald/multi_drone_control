"""Focused contracts for the YAML-owned M2 IRL gate profiles."""

from __future__ import annotations

from copy import deepcopy
from pathlib import Path

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[3]
CONFIG = ROOT / "src" / "tejen_mission" / "config" / "m2_irl_two_drone.yaml"


def test_yaml_gate_profiles_are_complete_and_have_commissioned_values():
    from tejen_mission.m2_irl_launch_config import parse_gate_profiles

    payload = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
    profiles = parse_gate_profiles(payload)
    assert profiles.mocap_timeout_s == 0.50
    assert profiles.telemetry_timeout_s == 0.50
    assert profiles.ring_timeout_s == 0.50
    assert profiles.status_timeout_s == 1.25
    assert profiles.link_statistics_timeout_s == 0.75
    assert profiles.ground_speed_mps == 0.05
    assert profiles.ground_dwell_s == 1.0
    assert profiles.airborne_position_tolerance_m == 0.05
    assert profiles.airborne_speed_mps == 0.08
    assert profiles.airborne_swing_deg == 5.0
    assert profiles.airborne_dwell_s == 0.75
    assert not hasattr(profiles, "takeoff_hover_timeout_s")
    assert profiles.c1f2_max_lift_drift_m == 0.15
    assert profiles.c1f2_speed_mps == 0.08
    assert profiles.c1f2_acceleration_mps2 == 0.50
    assert profiles.c1f2_dwell_s == 0.30


@pytest.mark.parametrize(
    "mutation",
    [
        lambda p: p["m2_irl"].pop("gate_profiles"),
        lambda p: p["m2_irl"]["gate_profiles"]["freshness"].pop("link_statistics_timeout_s"),
        lambda p: p["m2_irl"]["gate_profiles"]["airborne"].update(speed_mps=-0.01),
        lambda p: p["m2_irl"]["gate_profiles"]["airborne"].pop("takeoff_swing_deg"),
        lambda p: p["m2_irl"]["gate_profiles"]["c1f2_physical_recheck"].update(dwell_s=float("nan")),
    ],
)
def test_gate_profile_parser_fails_closed_for_missing_or_invalid_values(mutation):
    from tejen_mission.m2_irl_launch_config import parse_gate_profiles

    payload = deepcopy(yaml.safe_load(CONFIG.read_text(encoding="utf-8")))
    mutation(payload)
    with pytest.raises(ValueError):
        parse_gate_profiles(payload)


def test_takeoff_swing_profile_is_separate_from_capture_gate():
    from dataclasses import replace
    import math

    from tejen_mission.m2b_attachment_mission import M2BConfig, M2BSettleGate
    from tejen_mission.m2_irl_launch_config import parse_gate_profiles

    profiles = parse_gate_profiles(yaml.safe_load(CONFIG.read_text(encoding="utf-8")))
    assert profiles.takeoff_swing_deg == 5.0
    assert profiles.airborne_swing_deg == 5.0
    capture_config = M2BConfig(settle_speed_mps=profiles.airborne_speed_mps,
                               settle_swing_deg=profiles.airborne_swing_deg)
    takeoff_config = replace(capture_config, settle_swing_deg=profiles.takeoff_swing_deg)
    sample = dict(inputs_fresh=True, position_error_m=0.03, speed_mps=0.04,
                  swing_magnitude_rad=math.radians(4.5))
    assert M2BSettleGate(takeoff_config).step(now_s=0.0, **sample).reason == "measured settle dwell accumulating"
    assert M2BSettleGate(capture_config).step(now_s=0.0, **sample).reason == "measured settle dwell accumulating"
    assert M2BConfig().settle_swing_deg == 3.0  # unchanged simulation default


def test_quiet_miss_profile_is_validated_and_enabled_only_by_irl_launch():
    from tejen_mission.m2_irl_launch_config import parse_gate_profiles, progressive_ros_parameters
    payload = yaml.safe_load(CONFIG.read_text())
    cfg = parse_gate_profiles(payload).quiet_miss
    assert (cfg.wait_s, cfg.improvement_m, cfg.relative_speed_mps) == (10., .005, .03)
    for field, value in (("wait_s", 0), ("improvement_m", float("nan")), ("relative_speed_mps", -1)):
        bad = deepcopy(payload)
        bad["m2_irl"]["gate_profiles"]["quiet_miss"][field] = value
        with pytest.raises(ValueError, match="quiet_miss"):
            parse_gate_profiles(bad)
    bad = deepcopy(payload)
    del bad["m2_irl"]["gate_profiles"]["quiet_miss"]
    with pytest.raises(ValueError):
        parse_gate_profiles(bad)
    shared = progressive_ros_parameters(payload)["planner"]
    assert not shared.get("m2b_irl_quiet_miss_enabled", False)
    launch = (CONFIG.parent.parent / "launch/m2_irl_two_drone.launch.py").read_text()
    for literal in ('"m2b_irl_quiet_miss_enabled": True',
                    '"m2b_irl_quiet_miss_wait_s": profiles.quiet_miss.wait_s',
                    '"m2b_irl_quiet_miss_improvement_m": profiles.quiet_miss.improvement_m',
                    '"m2b_irl_quiet_miss_relative_speed_mps": profiles.quiet_miss.relative_speed_mps'):
        assert literal in launch


def test_irl_tip_alignment_profile_is_validated_and_owned_by_yaml():
    from tejen_mission.m2_irl_launch_config import parse_gate_profiles

    payload = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
    profile = parse_gate_profiles(payload).tip_alignment
    assert (profile.gain_per_s, profile.filter_tau_s, profile.max_rate_mps) == (0.8, 0.5, 0.025)
    assert (profile.max_offset_m, profile.min_normal_m, profile.pre_low_normal_m) == (0.08, 0.08, 0.10)
    assert (profile.capture_xy_m, profile.capture_dwell_s) == (0.025, 0.5)
    assert (profile.low_xy_m, profile.low_dwell_s, profile.timeout_s) == (0.025, 0.3, 10.0)
    assert profile.max_relative_speed_mps == 0.08
    for mutation in (
        lambda p: p["m2_irl"]["gate_profiles"].pop("tip_alignment"),
        lambda p: p["m2_irl"]["gate_profiles"]["tip_alignment"].update(max_offset_m=-1),
        lambda p: p["m2_irl"]["gate_profiles"]["tip_alignment"].update(min_normal_m=0.11),
        lambda p: p["m2_irl"]["gate_profiles"]["tip_alignment"].pop("max_relative_speed_mps"),
    ):
        bad = deepcopy(payload)
        mutation(bad)
        with pytest.raises(ValueError):
            parse_gate_profiles(bad)

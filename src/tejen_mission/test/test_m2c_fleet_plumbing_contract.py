"""Tests-first integration contract for M2C identity-safe four-drone plumbing.

The post-M2B repository is intentionally single-ego.  These tests should be RED
until M2C adapts the partner-proven `/drone_<id>/...` platform routing while keeping
current thesis planner/MPC/CommittedTrajectory authorities.
"""

from __future__ import annotations

import importlib
import re
from pathlib import Path

import pytest


REPO = Path(__file__).resolve().parents[3]
SRC = REPO / "src"
CALLBACK_MANAGER = SRC / "tejen_utility_objects" / "tejen_utility_objects" / "callback_manager.py"
ONLINE_JOIN = SRC / "tejen_mission" / "tejen_mission" / "online_join_planner.py"
BACKEND = SRC / "tejen_dynamic_planner" / "src" / "transfer_backend_node.cpp"
DRONE_LAUNCH_DIR = SRC / "tejen_mission" / "launch"
SIM_TEST_DIR = REPO / "tools" / "sim_test"


def _topics_module():
    try:
        return importlib.import_module("tejen_mission.m2_fleet_topics")
    except ModuleNotFoundError:
        pytest.fail("M2C RED contract: tejen_mission.m2_fleet_topics does not exist yet")


def _topic_map(module, drone_id: int) -> dict[str, str]:
    fn = getattr(module, "canonical_vehicle_topics", None)
    assert callable(fn), "M2C RED contract: canonical_vehicle_topics() is missing"
    result = fn(int(drone_id))
    if isinstance(result, dict):
        mapping = result
    elif hasattr(result, "__dict__"):
        mapping = vars(result)
    else:
        try:
            mapping = dict(result)
        except Exception as exc:  # pragma: no cover - diagnostic path
            raise AssertionError("canonical_vehicle_topics() must return a mapping-like value") from exc
    return {str(k): str(v) for k, v in mapping.items()}


def test_m2c_partner_derived_platform_topics_are_canonical_per_drone():
    module = _topics_module()
    topics = _topic_map(module, 2)
    expected = {
        "namespace": "/drone_2",
        "motion_capture_state": "/drone_2/motion_capture_state",
        "telemetry": "/drone_2/telemetry",
        "elrs_command": "/drone_2/ELRSCommand",
        "command": "/drone_2/command",
        "arming_service": "/drone_2/arming_service",
        "arming_state_feedback": "/drone_2/arming_state_feedback",
    }
    for key, value in expected.items():
        assert topics.get(key) == value

    # Thesis-owned per-drone channels may choose their exact suffix, but they
    # must still be explicitly identity-bound under the same vehicle namespace.
    for key in ("reference", "pendulum_swing_state", "magnet_command", "committed_trajectory"):
        assert key in topics
        assert topics[key].startswith("/drone_2/")

    maps = [_topic_map(module, i) for i in range(4)]
    common_keys = set.intersection(*(set(mapping) for mapping in maps))
    assert "namespace" in common_keys
    for key in sorted(common_keys):
        values = [mapping[key] for mapping in maps]
        assert len(set(values)) == 4, f"M2C topic alias on {key}: {values}"


def test_m2c_callback_manager_no_longer_binds_control_io_to_absolute_single_drone_topics():
    text = CALLBACK_MANAGER.read_text(encoding="utf-8")
    # Relative names or an injected route map are both acceptable.  What is not
    # acceptable in M2C is a hard-wired absolute singleton that ignores the
    # node's `/drone_i` namespace.
    forbidden_create_calls = (
        r"create_publisher\([^\n]+['\"]\/ELRSCommand['\"]",
        r"create_subscription\([^\n]+['\"]\/motion_capture_state['\"]",
        r"create_subscription\([^\n]+['\"]\/telemetry['\"]",
        r"create_subscription\([^\n]+['\"]\/pendulum_swing_state['\"]",
    )
    for pattern in forbidden_create_calls:
        assert re.search(pattern, text) is None, (
            "M2C RED contract: CallbackManager still owns an absolute single-drone topic"
        )


def test_m2c_ego_planner_publishes_identity_bound_committed_future_for_peers():
    text = ONLINE_JOIN.read_text(encoding="utf-8")
    assert "CommittedTrajectory" in text, (
        "M2C RED contract: online_join_planner does not yet publish the ego committed future"
    )
    assert "committed_trajectory" in text.lower()
    assert "vehicle_id" in text
    assert "sequence" in text
    assert "terminal_hold" in text


def test_m2c_cpp_backend_validates_committed_trajectory_vehicle_id_not_only_topic_index():
    text = BACKEND.read_text(encoding="utf-8")
    assert '"cooperative_vehicle_ids"' in text, (
        "M2C RED contract: backend lacks an explicit expected vehicle ID per cooperative topic"
    )
    # Keep this deliberately semantic rather than requiring one exact C++ line.
    callback_start = text.index("void cooperativeTrajectoryCallback")
    callback_end = text.index("void movingBasketTrajectoryCallback", callback_start)
    callback = text[callback_start:callback_end]
    assert "msg->vehicle_id" in callback
    assert "cooperative_vehicle_ids_" in callback
    assert any(token in callback for token in ("!=", "compare", "Rejected"))


def test_m2c_has_a_ground_only_four_x3_commissioning_launch_and_supervised_runner():
    launches = sorted(DRONE_LAUNCH_DIR.glob("*m2c*.launch.py"))
    runners = sorted(SIM_TEST_DIR.glob("run_m2c*.sh"))
    assert launches, "M2C RED contract: no M2C four-drone launch exists yet"
    assert runners, "M2C RED contract: no supervised M2C runner exists yet"

    launch_text = "\n".join(path.read_text(encoding="utf-8") for path in launches)
    runner_text = "\n".join(path.read_text(encoding="utf-8") for path in runners)
    assert "20.0" in launch_text or "0.349" in launch_text, (
        "M2C commissioning must exercise a nonzero ~20 degree ring yaw"
    )
    for i in range(4):
        assert f"drone_{i}" in launch_text or f"x3_{i}" in launch_text
    assert "TAKEOFF" not in runner_text.upper(), (
        "M2C commissioning is ground-only; takeoff belongs to M2D"
    )


def test_m2c_four_x3_bootstrap_exposes_four_independent_detachable_joint_channels():
    try:
        module = importlib.import_module("tejen_mission.m2c_ground_spawn")
    except ModuleNotFoundError:
        pytest.fail("M2C RED contract: tejen_mission.m2c_ground_spawn does not exist yet")

    channel_fn = getattr(module, "detachable_joint_channels", None)
    assert callable(channel_fn), (
        "M2C RED contract: m2c_ground_spawn.detachable_joint_channels() is missing"
    )
    channels = tuple(channel_fn((0, 1, 2, 3)))
    assert len(channels) == 4

    attach_topics = []
    detach_topics = []
    state_topics = []
    child_models = []
    for channel in channels:
        get = channel.get if isinstance(channel, dict) else lambda key: getattr(channel, key)
        assert get("parent_model") == "payload_model"
        assert get("parent_link") == "payload_link"
        child_models.append(str(get("child_model")))
        assert get("child_link") == "magnet_tip_link"
        attach_topics.append(str(get("attach_topic")))
        detach_topics.append(str(get("detach_topic")))
        state_topics.append(str(get("state_topic")))

    assert set(child_models) == {"x3_0", "x3_1", "x3_2", "x3_3"}
    assert len(set(attach_topics)) == len(set(detach_topics)) == len(set(state_topics)) == 4
    assert all(topic.startswith("/drone_") for topic in attach_topics + detach_topics + state_topics)

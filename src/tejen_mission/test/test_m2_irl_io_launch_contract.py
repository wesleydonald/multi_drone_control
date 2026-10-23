"""Focused ELRS/config/launch/manual-authority contract for M2 IRL."""

from __future__ import annotations

import importlib
import json
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import yaml


ROOT = Path(__file__).resolve().parents[3]
PKG = ROOT / "src" / "tejen_mission"


def test_elrs_explicit_port_magnet_slot_and_persistent_string_override():
    module = importlib.import_module("tejen_mission.elrs_interface_irl")
    source = (PKG / "tejen_mission" / "elrs_interface_irl.py").read_text(
        encoding="utf-8"
    )
    assert "declare_parameter('serial_port', '')" in source
    assert "if self.serial_port:" in source
    assert "serial.tools.list_ports.comports()" in source
    assert "magnet_string_command_topic" in source
    connect_source = source[source.index("def connect_serial"):source.index("def _packet_index_for_elrs_channel")]
    assert connect_source.index("if self.serial_port:") < connect_source.index(
        "serial.tools.list_ports.comports()"
    )


    node = object.__new__(module.ELRSInterface)
    node.idle = 993
    node.range = 820
    node.packet = np.full(16, node.idle, dtype=np.uint16)
    node.packet[4] = 0
    node.magnet_channels = [6]
    node.magnet_override_values = {6: None}
    node.get_logger = lambda: SimpleNamespace(info=lambda *a, **k: None, warn=lambda *a, **k: None)
    assert node._packet_index_for_elrs_channel(6) == 7
    node.magnet_string_commands_callback(SimpleNamespace(data="ON"))
    assert int(node.packet[7]) == 1813
    node.packet = np.full(16, node.idle, dtype=np.uint16)
    node.packet[4] = 0
    node.apply_magnet_override_to_packet()
    assert int(node.packet[7]) == 1813
    assert int(node.packet[4]) == 0


def test_checked_in_config_is_fail_closed_until_both_props_off_checks():
    config_path = PKG / "config" / "m2_irl_two_drone.yaml"
    payload = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    vehicles = payload["m2_irl"]["vehicles"]
    assert vehicles["drone_0"]["serial_port"] == "/dev/QUAD2"
    assert vehicles["drone_1"]["serial_port"] == "/dev/QUAD4"
    assert vehicles["drone_0"]["magnet_channel"] == 6
    assert vehicles["drone_1"]["magnet_channel"] == 6
    assert isinstance(vehicles["drone_0"]["props_off_verified"], bool)
    assert isinstance(vehicles["drone_1"]["props_off_verified"], bool)

    validation = importlib.import_module("tejen_mission.m2_irl_launch_config")
    unverified = deepcopy(payload)
    for vehicle in unverified["m2_irl"]["vehicles"].values():
        vehicle["props_off_verified"] = False
    with pytest.raises(ValueError, match="props-off"):
        validation.validate_mission_hardware_config(unverified)
    for vehicle in payload["m2_irl"]["vehicles"].values():
        vehicle["props_off_verified"] = True
    validated = validation.validate_mission_hardware_config(payload)
    assert validated == {"drone_0": ("/dev/QUAD2", 6), "drone_1": ("/dev/QUAD4", 6)}



def test_irl_geometry_matches_measured_magnet_center_and_contact_face():
    payload = yaml.safe_load((PKG / "config" / "m2_irl_two_drone.yaml").read_text())
    geometry = payload["m2_irl"]["geometry"]
    detector = payload["m2_irl"]["attachment_detector"]
    anchor_z = geometry["tether_anchor_body"][2]
    face_z = detector["magnet_contact_translation_m"][2]
    length = geometry["tether_length_m"]
    assert length == pytest.approx(0.5575)  # anchor to contact face
    assert face_z == pytest.approx(-0.0225)  # marker centre to face
    assert length - anchor_z + face_z == pytest.approx(0.575)  # body to marker centre
    launch = (PKG / "launch" / "m2_irl_two_drone.launch.py").read_text()
    assert '"magnet_drop_below_quad": body_to_magnet_center_m' in launch
    assert '"m2b_anchor_to_contact_length_m": float(geometry["tether_length_m"])' in launch
    assert '"contact_offset_z": float(contact["magnet_contact_translation_m"][2])' in launch

def test_link_statistics_event_requires_crc_length_and_positive_uplink_quality():
    module = importlib.import_module("tejen_mission.elrs_interface_irl")
    frame = bytearray([0xC8, 12, 0x14, 206, 205, 70, 4, 0, 2, 4, 201, 50, 3])
    frame.append(module.crc8_data(frame[2:]))
    assert module.valid_link_statistics_frame(bytes(frame))
    radio_addressed = bytearray(frame)
    radio_addressed[0] = 0xEA
    assert module.valid_link_statistics_frame(bytes(radio_addressed))
    bad_crc = bytearray(frame)
    bad_crc[-1] ^= 1
    assert not module.valid_link_statistics_frame(bytes(bad_crc))
    assert not module.valid_link_statistics_frame(bytes(frame[:-2]))
    zero_quality = bytearray(frame)
    zero_quality[5] = 0
    zero_quality[-1] = module.crc8_data(zero_quality[2:-1])
    assert not module.valid_link_statistics_frame(bytes(zero_quality))


def test_elrs_publishes_link_event_only_from_accepted_statistics_frame():
    module = importlib.import_module("tejen_mission.elrs_interface_irl")
    events = []
    node = object.__new__(module.ELRSInterface)
    node.link_statistics_publisher = SimpleNamespace(publish=events.append)
    node.rssi = 0
    frame = bytearray([0xC8, 12, 0x14, 206, 205, 70, 4, 0, 2, 4, 201, 50, 3])
    frame.append(module.crc8_data(frame[2:]))
    node.handleCrsfPacket(module.PacketsTypes.LINK_STATISTICS, bytes(frame))
    assert len(events) == 1
    zero_quality = bytearray(frame)
    zero_quality[5] = 0
    zero_quality[-1] = module.crc8_data(zero_quality[2:-1])
    node.handleCrsfPacket(module.PacketsTypes.LINK_STATISTICS, bytes(zero_quality))
    assert len(events) == 1


def test_identity_config_is_shared_and_rejects_missing_duplicate_or_mismatched_alias():
    validation = importlib.import_module("tejen_mission.m2_irl_launch_config")
    payload = yaml.safe_load((PKG / "config" / "m2_irl_two_drone.yaml").read_text())
    identity = validation.parse_two_drone_identity(payload)
    assert identity.ring_id == 8
    assert identity.vehicles["drone_0"].physical_quad == 2
    assert identity.vehicles["drone_1"].body_id == 14
    params = validation.identity_ros_parameters(identity)
    encoded = json.loads(params["hardware_identity_json"])["m2_irl"]
    assert encoded["vehicles"]["drone_0"]["body_id"] == 12
    assert encoded["vehicles"]["drone_1"]["ftdi_serial"] == "FTF09R63"
    restored = validation.parse_identity_json(params["hardware_identity_json"])
    assert dict(restored.vehicles) == dict(identity.vehicles)
    for mutation in (
        lambda p: p["m2_irl"]["vehicles"]["drone_0"].pop("body_id"),
        lambda p: p["m2_irl"]["vehicles"]["drone_1"].update(body_id=12),
        lambda p: p["m2_irl"]["vehicles"]["drone_1"].update(ftdi_serial="FTF0AROT"),
        lambda p: p["m2_irl"]["vehicles"]["drone_0"].update(ftdi_serial=None),
        lambda p: p["m2_irl"]["vehicles"]["drone_1"].update(magnet_id=22),
        lambda p: p["m2_irl"]["vehicles"]["drone_1"].update(physical_quad=2),
        lambda p: p["m2_irl"]["vehicles"]["drone_0"].update(serial_port="/dev/QUAD4"),
    ):
        bad = deepcopy(payload)
        mutation(bad)
        with pytest.raises(ValueError):
            validation.parse_two_drone_identity(bad)


def test_launch_io_constructs_and_mission_refuses_unverified_aux(tmp_path):
    import importlib.util
    from launch import LaunchContext

    launch_path = PKG / "launch" / "m2_irl_two_drone.launch.py"
    spec = importlib.util.spec_from_file_location("m2_irl_launch_contract", launch_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    config = str(PKG / "config" / "m2_irl_two_drone.yaml")

    context = LaunchContext()
    context.launch_configurations.update(
        {"mode": "io", "config_file": config, "log_dir": "/tmp/m2_irl_test", "rviz": "false"}
    )
    assert len(module._launch_setup(context)) == 4

    unverified = yaml.safe_load(Path(config).read_text(encoding="utf-8"))
    for vehicle in unverified["m2_irl"]["vehicles"].values():
        vehicle["props_off_verified"] = False
    unverified_path = tmp_path / "unverified_m2_irl.yaml"
    unverified_path.write_text(yaml.safe_dump(unverified), encoding="utf-8")
    context.launch_configurations["mode"] = "mission"
    context.launch_configurations["config_file"] = str(unverified_path)
    with pytest.raises(ValueError, match="props-off"):
        module._launch_setup(context)


def test_launch_router_and_manager_receive_identical_yaml_identity(tmp_path):
    import importlib.util
    from launch import LaunchContext
    from launch.utilities import perform_substitutions

    payload = yaml.safe_load((PKG / "config" / "m2_irl_two_drone.yaml").read_text())
    payload["m2_irl"]["vehicles"]["drone_0"]["body_id"] = 16
    for vehicle in payload["m2_irl"]["vehicles"].values():
        vehicle["props_off_verified"] = True
    config = tmp_path / "m2_identity_test.yaml"
    config.write_text(yaml.safe_dump(payload), encoding="utf-8")
    launch_path = PKG / "launch" / "m2_irl_two_drone.launch.py"
    spec = importlib.util.spec_from_file_location("m2_irl_identity_launch", launch_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    context.launch_configurations.update({
        "mode": "mission", "config_file": str(config),
        "log_dir": str(tmp_path / "logs"), "rviz": "false",
    })
    actions = module._launch_setup(context)

    def raw_parameter(node, name):
        for key, value in node._Node__parameters[0].items():
            if perform_substitutions(context, key) == name:
                return perform_substitutions(context, value)
        raise AssertionError(f"missing launch parameter {name}")

    router_identity = raw_parameter(actions[1], "hardware_identity_json")
    manager_identity = raw_parameter(actions[4], "hardware_identity_json")
    assert router_identity == manager_identity
    assert json.loads(yaml.safe_load(router_identity))["m2_irl"]["vehicles"]["drone_0"]["body_id"] == 16


def _mission_backend_parameters(tmp_path, vehicle_count=2):
    """Inspect both constructed backend nodes, including their YAML defaults."""
    import importlib.util
    from launch import LaunchContext
    from launch.utilities import perform_substitutions

    name = "m2_irl_two_drone.yaml" if vehicle_count == 2 else "m2_irl_four_drone.yaml"
    payload = yaml.safe_load((PKG / "config" / name).read_text())
    for vehicle in payload["m2_irl"]["vehicles"].values():
        vehicle["props_off_verified"] = True
    config = tmp_path / "m2_timing_test.yaml"
    config.write_text(yaml.safe_dump(payload), encoding="utf-8")
    launch_path = PKG / "launch" / "m2_irl_two_drone.launch.py"
    spec = importlib.util.spec_from_file_location("m2_irl_timing_launch", launch_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    context.launch_configurations.update({
        "mode": "mission", "config_file": str(config),
        "log_dir": str(tmp_path / "logs"), "rviz": "false",
        "expected_vehicle_count": str(vehicle_count),
    })
    actions = module._launch_setup(context)
    backends = {}
    for action in actions:
        for node in getattr(action, "actions", []):
            if not hasattr(node, "_Node__parameters"):
                continue
            def resolved(value):
                if isinstance(value, (list, tuple)):
                    return perform_substitutions(context, value)
                return str(value)

            params = {
                resolved(key): str(yaml.safe_load(resolved(value)))
                for key, value in node._Node__parameters[0].items()
                if resolved(key) in {
                    "cooperative_prediction_mode",
                    "vehicle_id",
                    "seed_stationary_on_activation",
                    "fixed_splice_lookahead_s",
                    "shadow_reference_topic",
                    "cooperative_shared_start_skew_s",
                    "cooperative_shared_start_skew_max_speed_mps",
                    "execution_tracking_safety_enabled",
                    "cooperative_tracking_safety_enabled",
                    "ego_tracking_half_x_m",
                    "ego_tracking_half_y_m",
                    "ego_tracking_half_z_m",
                }
            }
            if params.get("cooperative_prediction_mode") == "shared_trajectory":
                backends[params["vehicle_id"]] = params
    assert set(backends) == {f"drone_{i}" for i in range(vehicle_count)}
    return backends


@pytest.mark.parametrize("vehicle_count", [2, 4])
def test_irl_tracking_brakes_disabled_without_removing_collision_inflation(tmp_path, vehicle_count):
    for backend in _mission_backend_parameters(tmp_path, vehicle_count).values():
        assert backend["execution_tracking_safety_enabled"].lower() == "false"
        assert backend["cooperative_tracking_safety_enabled"].lower() == "false"
        assert tuple(float(backend[f"ego_tracking_half_{a}_m"]) for a in "xyz") == (0.05, 0.05, 0.05)
    m1 = yaml.safe_load((ROOT / "src/tejen_dynamic_planner/config/c1f_irl_m1_virtual_ring.yaml").read_text())
    m1_params = m1["dynamic_planner_transfer_backend"]["ros__parameters"]
    assert m1_params["cooperative_tracking_safety_enabled"] is False
    assert tuple(m1_params[f"cooperative_tracking_half_{a}_m"] for a in "xyz") == (0.05, 0.05, 0.05)
    source = (ROOT / "src/tejen_dynamic_planner/src/transfer_backend_node.cpp").read_text()
    assert '"cooperative_tracking_safety_enabled", true' in source
    sim = yaml.safe_load((ROOT / "src/tejen_dynamic_planner/config/c1f6_moving_rendezvous.yaml").read_text())
    assert sim["dynamic_planner_transfer_backend"]["ros__parameters"]["execution_tracking_safety_enabled"] is True


def test_irl_backends_seed_passive_pre_authority_future_splice(tmp_path):
    for vehicle_id, backend in _mission_backend_parameters(tmp_path).items():
        assert backend["seed_stationary_on_activation"].lower() == "true", vehicle_id
        assert float(backend["fixed_splice_lookahead_s"]) == pytest.approx(0.10)
        assert backend["shadow_reference_topic"] != f"/{vehicle_id}/join_planner/reference"


def test_rolling_commitment_recheck_has_future_coverage_or_rejects_late(tmp_path):
    """A commitment rolled forward during a short solve must still cover splice."""
    core = (ROOT / "src" / "tejen_dynamic_planner" / "standalone" / "src"
            / "receding_horizon_planner.cpp").read_text(encoding="utf-8")
    assert "if (had_incumbent && !strictlyBeforeSplice(result.candidate_finish_time_s, splice_time))" in core
    request_s = 100.0
    latest_obstacle_start_s = request_s + 0.025
    ordinary_finish_s = request_s + 0.030
    late_finish_s = request_s + 0.110
    for vehicle_id, backend in _mission_backend_parameters(tmp_path).items():
        has_seed = backend["seed_stationary_on_activation"].lower() == "true"
        splice_s = request_s + (
            float(backend["fixed_splice_lookahead_s"]) if has_seed else 0.0
        )
        assert latest_obstacle_start_s <= splice_s, vehicle_id
        assert ordinary_finish_s < splice_s, vehicle_id
        assert late_finish_s >= splice_s, vehicle_id


def test_launch_runner_and_udev_contract_preserve_manual_authority():
    launch_text = (PKG / "launch/m2_irl_two_drone.launch.py").read_text()
    assert '"external_reference_validate_timing": True' in launch_text
    launch = (PKG / "launch" / "m2_irl_two_drone.launch.py").read_text(encoding="utf-8")
    runner = (ROOT / "tools" / "irl_test" / "run_m2_irl_two_drone.sh").read_text(encoding="utf-8")
    rules = (ROOT / "tools" / "irl_test" / "99-elrs-m2.rules").read_text(encoding="utf-8")
    ignore = (ROOT / ".gitignore").read_text(encoding="utf-8")
    assert '"io", "mission"' in launch
    observer = (PKG / "tejen_mission" / "m2_attachment_observer.py").read_text(
        encoding="utf-8"
    )
    assert "contact_rotation_qx" in observer
    assert '"contact_offset_x": float(contact["magnet_contact_translation_m"][0])' in launch
    assert '"contact_rotation_qw": float(contact["magnet_contact_quaternion_xyzw"][3])' in launch

    assert "m2_irl_mocap_router" in launch
    router = (PKG / "tejen_mission" / "m2_irl_mocap_router.py").read_text(
        encoding="utf-8"
    )
    assert "ring_marker_translation_x" in router
    assert '"ring_marker_translation_x": float(geometry["ring_marker_translation_m"][0])' in launch

    assert "elrs_interface_irl" in launch
    assert '"xy_integral_limit": 0.45' in launch
    assert "TimerAction" in launch
    assert "props_off_verified" in launch
    assert "ros2 topic pub" not in runner
    assert "ros2 service call" not in runner
    assert "m2d_four_drone.rviz" in launch
    assert 'rviz="${3:-false}"' in runner
    assert 'rviz:="${rviz}"' in runner
    assert '"marker_topic": f"{ns}/join_planner/markers"' in launch
    assert '"marker_topic": f"{ns}/dynamic_planner/markers"' in launch
    assert '"min_reference_z": 0.05' in launch
    assert '"takeoff_height": 0.65' in launch
    assert '"m2b_takeoff_require_magnet_clearance": True' in launch
    assert '"m2b_takeoff_magnet_clearance_m": 0.03' in launch
    assert '"m2b_takeoff_hover_timeout_s":' not in launch
    assert '**progressive["planner"]' in launch
    assert '"m2b_freeze_attached_hold_reference": True' in launch
    assert '"m2b_takeoff_settle_swing_deg": profiles.takeoff_swing_deg' in launch
    assert '"m2b_link_statistics_timeout_s": profiles.link_statistics_timeout_s' in launch
    assert '"c1f2_prepare_settle_acceleration_mps2": profiles.c1f2_acceleration_mps2' in launch
    assert '"c1f2_reference_recovery_enabled": True' in launch
    assert '"reference_recovery_enabled": True' in launch
    for name in ('c1f2_recovery_continuation_topic', 'recovery_continuation_topic'):
        assert f'"{name}": f"{{ns}}/dynamic_planner/transfer_reference_continuation"' in launch
    for suffix in ("reference_recovery_request", "reference_recovery_status"):
        assert launch.count(f'f"{{ns}}/dynamic_planner/{suffix}"') == 2
    assert "source /opt/ros/humble/setup.bash" in runner
    assert "set -euo pipefail" in runner
    assert 'ATTRS{serial}=="FTF0AROT"' in rules and 'SYMLINK+="QUAD2"' in rules
    assert 'ATTRS{serial}=="FTF09R63"' in rules and 'SYMLINK+="QUAD4"' in rules
    assert 'ATTRS{serial}=="FTF0AR84"' in rules and 'SYMLINK+="QUAD1"' in rules
    assert 'ATTRS{serial}=="FTF09W4A"' in rules and 'SYMLINK+="QUAD3"' in rules
    assert "/multi_drone_control-week1-ready-for-testing.zip" in ignore



def test_runner_sources_ros_and_workspace_before_enabling_nounset():
    runner = (ROOT / "tools" / "irl_test" / "run_m2_irl_two_drone.sh").read_text(
        encoding="utf-8"
    )
    ros_setup = runner.index("source /opt/ros/humble/setup.bash")
    workspace_setup = runner.index('source "${repo_root}/install/setup.bash"')
    nounset = runner.index("set -euo pipefail")
    assert ros_setup < workspace_setup < nounset

def test_launch_wires_configurable_tether_anchor_and_detector_off_policy():
    launch = (PKG / "launch" / "m2_irl_two_drone.launch.py").read_text(encoding="utf-8")
    router = (PKG / "tejen_mission" / "m2_irl_mocap_router.py").read_text(
        encoding="utf-8"
    )
    assert "tether_anchor_x" in router
    assert "self.tether_anchor_body" in router
    assert '"tether_anchor_x": float(geometry["tether_anchor_body"][0])' in launch
    assert '"magnet_off_counts_as_loss": bool(contact["magnet_off_counts_as_loss"])' in launch


def test_mission_launch_wires_each_planner_to_its_namespaced_permission_and_mpc_topics():
    """Fleet readiness must see the two planner heartbeats, never global defaults."""
    launch = (PKG / "launch" / "m2_irl_two_drone.launch.py").read_text(encoding="utf-8")
    assert '"m2b_arm_permission_topic": f"{ns}/join_planner/arm_permission"' in launch
    assert '"m2b_mpc_mode_topic": f"{ns}/join_planner/mpc_mode"' in launch
    assert '"payload_mpc_mode_topic": f"{ns}/join_planner/mpc_mode"' in launch


def test_launch_uses_configured_irl_magnet_off_loss_policy():
    launch = (PKG / "launch" / "m2_irl_two_drone.launch.py").read_text(encoding="utf-8")
    assert '"magnet_off_counts_as_loss": bool(contact["magnet_off_counts_as_loss"])' in launch

def test_irl_observer_uses_actual_router_tf_frame_leaves():
    from tejen_mission.m2a_attachment_runtime import NamedTransform, resolve_named_transform_world

    launch = (PKG / "launch" / "m2_irl_two_drone.launch.py").read_text(encoding="utf-8")
    assert '"carrier_frame_leaf": "base_link"' in launch
    assert '"magnet_frame_leaf": "magnet_tip_link"' in launch
    for vehicle_id in ("drone_0", "drone_1"):
        transforms = [
            NamedTransform("map", f"{vehicle_id}/base_link", [0, 0, 1], [0, 0, 0, 1]),
            NamedTransform("map", f"{vehicle_id}/magnet_tip_link", [0, 0, 0.4], [0, 0, 0, 1]),
        ]
        body, _, _ = resolve_named_transform_world(transforms, "base_link")
        magnet, _, _ = resolve_named_transform_world(transforms, "magnet_tip_link")
        assert np.allclose(body, [0, 0, 1])
        assert np.allclose(magnet, [0, 0, 0.4])


def test_irl_backend_allows_only_bounded_stationary_peer_start_overlap(tmp_path):
    from tejen_mission.m2_irl_launch_config import parse_gate_profiles

    payload = yaml.safe_load((PKG / "config" / "m2_irl_two_drone.yaml").read_text(encoding="utf-8"))
    profile = parse_gate_profiles(payload)
    assert profile.cooperative_shared_start_skew_s == 0.05
    launch = (PKG / "launch" / "m2_irl_two_drone.launch.py").read_text(encoding="utf-8")
    assert '"cooperative_shared_start_skew_s": profiles.cooperative_shared_start_skew_s' in launch
    assert '"cooperative_shared_start_skew_max_speed_mps": profiles.ground_speed_mps' in launch
    for vehicle_id, configured in _mission_backend_parameters(tmp_path).items():
        assert float(configured["cooperative_shared_start_skew_s"]) == pytest.approx(0.05), vehicle_id
        assert float(configured["cooperative_shared_start_skew_max_speed_mps"]) == pytest.approx(
            profile.ground_speed_mps), vehicle_id

    backend = (ROOT / "src" / "tejen_dynamic_planner" / "src" / "transfer_backend_node.cpp").read_text(encoding="utf-8")
    assert '"cooperative_shared_start_skew_s", 0.0' in backend
    assert 'start_skew_s > cooperative_shared_start_skew_s_' in backend
    assert 'state.velocity.norm() > cooperative_shared_start_skew_max_speed_mps_' in backend
    assert 'advertised.velocity.norm() > cooperative_shared_start_skew_max_speed_mps_' in backend
    assert 'trajectory.evaluate(validation_time_s)' in backend


@pytest.mark.parametrize('id_format', [lambda i: i, str, lambda i: f'ID: {i}'])
def test_router_cached_magnet_keeps_its_packet_time_when_body_updates(id_format):
    from tejen_mission.m2_irl_mocap_router import M2IRLMocapRouter
    from rclpy.time import Time

    node = object.__new__(M2IRLMocapRouter)
    now = [10.]
    packets = [22, 12]
    def receive(_size):
        if not packets:
            raise BlockingIOError
        body_id = packets.pop(0)
        now[0] = 10. if body_id == 22 else 11.
        return str(body_id).encode() + b"|pose", None
    def body(i):
        return SimpleNamespace(id=id_format(i), timestamp=now[0], position=[0., 0., .6],
            rotation=[1., 0., 0., 0.], velocity=[0., 0., 0.], angular_velocity=[0., 0., 0.])
    node.sock = SimpleNamespace(recvfrom=receive)
    node.parsers = {i: SimpleNamespace(parse=lambda _p, i=i: body(i)) for i in (12,22)}
    node.throttle = SimpleNamespace(accept=lambda *a: True)
    node.latest = {}; node.last_rx_monotonic = {}; node.sample_stamps = {}
    node.ring_id = 8; node.hardware = {"drone_0": None}
    node.body_ids = {"drone_0": 12}; node.magnet_ids = {"drone_0": 22}
    published = []
    node.tf_pubs = {"drone_0": SimpleNamespace(publish=published.append)}
    node.motion_pubs = node.magnet_pubs = {"drone_0": SimpleNamespace(publish=lambda m: None)}
    node._publish_pendulum = lambda v: None
    node.get_clock = lambda: SimpleNamespace(now=lambda: Time(seconds=now[0]))
    node._poll_socket()
    assert published[-1].transforms[0].header.stamp.sec == 11
    assert published[-1].transforms[1].header.stamp.sec == 10


@pytest.mark.parametrize("policy", ["legacy", "measured_contact_15deg"])
@pytest.mark.parametrize("frame_prefix", ["drone_0/", "x3::"])
def test_observer_respects_individual_source_stamps_only_for_measured_policy(policy, frame_prefix):
    from tejen_mission.m2_attachment_observer import M2AttachmentObserver
    from geometry_msgs.msg import TransformStamped, PoseArray, Pose
    from tf2_msgs.msg import TFMessage

    node = object.__new__(M2AttachmentObserver)
    node.attachment_policy = policy
    node._now_s = lambda: 10.
    node.latest_drone_world = node.latest_magnet_world = node.latest_ring_world = None
    node.carrier_frame_leaf = "base_link"; node.magnet_frame_leaf = "magnet_tip_link"
    transforms = []
    for leaf, stamp in (("base_link", 10), ("magnet_tip_link", 8)):
        tf = TransformStamped(); tf.header.frame_id = "map"; tf.header.stamp.sec = stamp
        tf.child_frame_id = frame_prefix + leaf; tf.transform.rotation.w = 1.
        transforms.append(tf)
    node._carrier_tf_callback(TFMessage(transforms=transforms))
    assert node.latest_magnet_world[2] == (8. if policy != "legacy" else 10.)
    transforms[1].transform.translation.x = 1.  # same source sample, changed serialization
    node._carrier_tf_callback(TFMessage(transforms=transforms))
    assert node.latest_magnet_world[0][0] == (0. if policy != "legacy" else 1.)
    poses = PoseArray(); poses.header.stamp.sec = 9
    pose = Pose(); pose.orientation.w = 1.; poses.poses = [pose, pose]
    node.drone_index = 0; node.magnet_index = 1; node.ring_pose_index = 0
    node.x3_link_poses_are_relative = False
    node.latest_drone_world = node.latest_magnet_world = None
    node._x3_pose_callback(poses); node._ring_pose_callback(poses)
    assert node.latest_magnet_world[2] == (9. if policy != "legacy" else 10.)
    assert node.latest_ring_world[2] == (9. if policy != "legacy" else 10.)
    if policy != "legacy":
        for stamp in (8, 9, 11):  # out-of-order, duplicate, future
            poses.header.stamp.sec = stamp; poses.poses[0].position.x = 2.
            node._x3_pose_callback(poses); node._ring_pose_callback(poses)
        assert node.latest_drone_world[2] == node.latest_ring_world[2] == 9.
        assert node.latest_drone_world[0][0] == node.latest_ring_world[0][0] == 0.

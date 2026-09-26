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


def test_launch_runner_and_udev_contract_are_manual_and_two_adapter_only():
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
    assert '"m2b_takeoff_hover_timeout_s": 15.0' in launch
    assert "source /opt/ros/humble/setup.bash" in runner
    assert "set -euo pipefail" in runner
    assert 'ATTRS{serial}=="FTF0AROT"' in rules and 'SYMLINK+="QUAD2"' in rules
    assert 'ATTRS{serial}=="FTF09R63"' in rules and 'SYMLINK+="QUAD4"' in rules
    assert "QUAD1" not in rules and "QUAD3" not in rules
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

"""M2_RATE_SOURCE=imu opt-in on the full M2 path (multi_drone_control, 2026-09-27).

The generator's --imu-system must add exactly one world-level Imu system and nothing else
(the X3 PoseArray index 7 depends on entity order); his launch must default to the pose
rate loop with no IMU bridges.
"""

from __future__ import annotations

import copy
import importlib
import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest


REPO = Path(__file__).resolve().parents[3]
SOURCE_X3 = REPO / "simulation_assets/tejen/modelLargeM2BallMagnet.sdf"
SOURCE_RING = REPO / "simulation_assets/tejen/m2a_ring_fixture.sdf"
WORLD_TEMPLATE = REPO / "simulation_assets/tejen/world_m2b_single_attachment.sdf"
LAUNCH = REPO / "src/tejen_mission/launch/m2d_four_drone_sequential.launch.py"
RUNNER = REPO / "tools/sim_test/run_m2d_sequential_attachment.sh"


def _generate(out: Path, **kwargs) -> dict:
    spawn = importlib.import_module("tejen_mission.m2c_ground_spawn")
    return spawn.generate_m2c_assets(
        source_x3=SOURCE_X3, source_ring=SOURCE_RING, world_template=WORLD_TEMPLATE,
        output_dir=out, ring_yaw_deg=20.0, manifest_path=out / "manifest.json", **kwargs)


def _is_imu(el) -> bool:
    return el.tag == "plugin" and "imu-system" in el.attrib.get("filename", "")


def test_imu_system_adds_one_world_plugin_and_changes_nothing_else(tmp_path):
    base = _generate(tmp_path / "pose")
    imu = _generate(tmp_path / "imu", imu_system=True)
    assert base["run_local_imu_sensors_enabled"] is False
    assert imu["run_local_imu_sensors_enabled"] is True

    for a, b in zip(base["generated_x3"] + [base["generated_ring"]],
                    imu["generated_x3"] + [imu["generated_ring"]]):
        assert Path(a).read_bytes() == Path(b).read_bytes()

    w_base = ET.parse(base["generated_world"]).getroot().find("world")
    w_imu = ET.parse(imu["generated_world"]).getroot().find("world")
    assert w_imu.attrib["name"] == "quadcopter"   # the world in the launch's gz IMU topic
    assert not any(_is_imu(el) for el in w_base.iter())
    assert [el for el in w_imu if _is_imu(el)] and sum(_is_imu(el) for el in w_imu.iter()) == 1
    stripped = copy.deepcopy(w_imu)
    for el in [el for el in stripped if _is_imu(el)]:
        stripped.remove(el)
    # same elements in the same order: the entity topology behind pose_index 7 is unchanged
    assert [ET.tostring(el).strip() for el in stripped] == [ET.tostring(el).strip() for el in w_base]


def test_runner_passes_imu_system_only_when_opted_in():
    runner = RUNNER.read_text()
    assert 'RATE_SOURCE="${M2_RATE_SOURCE:-pose}"' in runner
    assert "IMU_SYSTEM_ARGS=(--imu-system)" in runner
    assert '"${IMU_SYSTEM_ARGS[@]}"' in runner


def _launch_nodes(monkeypatch, rate_source):
    pytest.importorskip("launch_ros")
    try:
        from ament_index_python.packages import get_package_share_directory
        get_package_share_directory("tejen_dynamic_planner")
    except Exception:
        pytest.skip("tejen_dynamic_planner not installed (source install/setup.bash)")
    if rate_source is None:
        monkeypatch.delenv("M2_RATE_SOURCE", raising=False)
    else:
        monkeypatch.setenv("M2_RATE_SOURCE", rate_source)
    spec = importlib.util.spec_from_file_location("m2d_launch_under_test", LAUNCH)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    from launch.actions import TimerAction
    from launch_ros.actions import Node
    ld = mod.generate_launch_description()
    nodes = []
    for entity in ld.entities:
        if isinstance(entity, TimerAction):
            nodes += [a for a in entity.actions if isinstance(a, Node)]
    return ld, nodes


def _text(value):
    # launch_ros normalizes parameter names/strings into tuples of TextSubstitution
    if isinstance(value, tuple) and all(hasattr(v, "text") for v in value):
        return "".join(v.text for v in value)
    return value


def _bridge_params(nodes):
    import yaml   # string values are stored as yaml documents ("imu\n...\n")

    def val(v):
        v = _text(v)
        return yaml.safe_load(v) if isinstance(v, str) else v
    return [{_text(k): val(v) for k, v in n._Node__parameters[0].items()}
            for n in nodes if n.node_executable == "tejen_betaflight_communication"]


def test_launch_default_is_pose_without_imu_bridges(monkeypatch):
    ld, nodes = _launch_nodes(monkeypatch, None)
    params = _bridge_params(nodes)
    assert len(params) == 4 and all(p["rate_source"] == "pose" for p in params)
    assert not [n for n in nodes if "imu_bridge" in (n._Node__node_name or "")]
    from launch.actions import OpaqueFunction
    assert not [e for e in ld.entities if isinstance(e, OpaqueFunction)]


def test_launch_imu_bridges_each_gyro_to_its_bridge(monkeypatch):
    ld, nodes = _launch_nodes(monkeypatch, "imu")
    params = _bridge_params(nodes)
    assert [p["rate_source"] for p in params] == ["imu"] * 4
    assert [p["imu_topic"] for p in params] == [f"/drone_{i}/imu" for i in range(4)]
    bridges = sorted((n for n in nodes if "m2d_imu_bridge_" in (n._Node__node_name or "")),
                     key=lambda n: n._Node__node_name)
    assert len(bridges) == 4
    for i, n in enumerate(bridges):
        gz = f"/world/quadcopter/model/x3_{i}/link/X3/base_link/sensor/imu_sensor/imu"
        assert n._Node__arguments == [gz + "@sensor_msgs/msg/Imu[gz.msgs.IMU"]
        assert [(_text(a), _text(b)) for a, b in n._Node__remappings] == [(gz, f"/drone_{i}/imu")]


def test_launch_rejects_unknown_rate_source(monkeypatch):
    with pytest.raises(RuntimeError, match="M2_RATE_SOURCE"):
        _launch_nodes(monkeypatch, "gyro")


def test_launch_imu_refuses_a_world_without_the_imu_system(monkeypatch, tmp_path):
    ld, _ = _launch_nodes(monkeypatch, "imu")
    from launch import LaunchContext
    from launch.actions import OpaqueFunction
    check = [e for e in ld.entities if isinstance(e, OpaqueFunction)]
    assert len(check) == 1
    pose_world = _generate(tmp_path / "pose")["generated_world"]
    imu_world = _generate(tmp_path / "imu", imu_system=True)["generated_world"]
    context = LaunchContext()
    context.launch_configurations["world_path"] = imu_world
    check[0].execute(context)
    context.launch_configurations["world_path"] = pose_world
    with pytest.raises(RuntimeError, match="no Imu system"):
        check[0].execute(context)

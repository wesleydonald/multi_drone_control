"""P1 regression contracts for shared payload-MPC ACADOS artifact reuse."""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path


PKG = Path(__file__).resolve().parents[1]
SRC = PKG / "tejen_mpc"
CACHE_HELPER = SRC / "acados_cache.py"
ACADOS = SRC / "acados.py"
MAIN = SRC / "main.py"
REPO = Path(__file__).resolve().parents[3]
M2C_LAUNCH = REPO / "src/tejen_mission/launch/m2c_vehicle_controller.launch.py"
M2C_RUNNER = REPO / "tools/sim_test/run_m2c_ground.sh"


def _cache_module():
    assert CACHE_HELPER.exists(), "P1 requires a pure/testable ACADOS cache helper"
    spec = importlib.util.spec_from_file_location("payload_mpc_acados_cache", CACHE_HELPER)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_cache_fingerprint_tracks_solver_sources_and_runtime_cost_configuration(tmp_path):
    cache = _cache_module()
    acados_src = tmp_path / "acados.py"
    dynamics_src = tmp_path / "dynamics.py"
    modes_src = tmp_path / "mpc_modes.py"
    acados_src.write_text("A\n", encoding="utf-8")
    dynamics_src.write_text("B\n", encoding="utf-8")
    modes_src.write_text("C\n", encoding="utf-8")
    sources = (acados_src, dynamics_src, modes_src)

    first = cache.build_payload_mpc_cache_identity(
        source_paths=sources,
        xy_integral_weight=15.0,
        xy_integral_terminal_weight=30.0,
    )
    same = cache.build_payload_mpc_cache_identity(
        source_paths=sources,
        xy_integral_weight=15.0,
        xy_integral_terminal_weight=30.0,
    )
    changed_weight = cache.build_payload_mpc_cache_identity(
        source_paths=sources,
        xy_integral_weight=16.0,
        xy_integral_terminal_weight=30.0,
    )
    dynamics_src.write_text("B changed\n", encoding="utf-8")
    changed_source = cache.build_payload_mpc_cache_identity(
        source_paths=sources,
        xy_integral_weight=15.0,
        xy_integral_terminal_weight=30.0,
    )

    assert first.fingerprint == same.fingerprint
    assert first.fingerprint != changed_weight.fingerprint
    assert first.fingerprint != changed_source.fingerprint


def test_cache_is_reusable_only_with_matching_manifest_and_both_solver_artifacts(tmp_path):
    cache = _cache_module()
    source = tmp_path / "source.py"
    source.write_text("source\n", encoding="utf-8")
    identity = cache.build_payload_mpc_cache_identity(
        source_paths=(source,),
        xy_integral_weight=15.0,
        xy_integral_terminal_weight=30.0,
    )
    layout = cache.payload_mpc_cache_layout(tmp_path / "cache", identity.fingerprint)

    assert not cache.payload_mpc_cache_is_reusable(layout, identity)
    layout.root.mkdir(parents=True)
    layout.code_export_dir.mkdir(parents=True)
    cache.write_payload_mpc_cache_manifest(layout, identity)
    assert not cache.payload_mpc_cache_is_reusable(layout, identity)

    layout.ocp_json.write_text("{}", encoding="utf-8")
    layout.sim_json.write_text("{}", encoding="utf-8")
    layout.ocp_shared_library.write_bytes(b"ocp")
    assert not cache.payload_mpc_cache_is_reusable(layout, identity)

    layout.sim_shared_library.write_bytes(b"sim")
    assert cache.payload_mpc_cache_is_reusable(layout, identity)

    manifest = json.loads(layout.manifest.read_text(encoding="utf-8"))
    manifest["fingerprint"] = "wrong"
    layout.manifest.write_text(json.dumps(manifest), encoding="utf-8")
    assert not cache.payload_mpc_cache_is_reusable(layout, identity)


def test_acados_builder_uses_one_locked_fingerprinted_cache_and_explicit_reuse_flags():
    source = ACADOS.read_text(encoding="utf-8")
    assert "build_payload_mpc_cache_identity" in source
    assert "locked_payload_mpc_cache" in source
    assert "payload_mpc_cache_is_reusable" in source
    assert "PAYLOAD_MPC_REBUILD" in source
    assert "generate=need_build" in source
    assert "build=need_build" in source
    assert "ocp.code_export_directory" in source
    assert "sim.code_export_directory" in source
    assert "json_file=str(layout.ocp_json.name)" in source
    assert "json_file=str(layout.sim_json.name)" in source
    assert "os.chdir(layout.root)" in source
    assert "os.chdir(previous_cwd)" in source


def test_controller_cache_is_opt_in_outside_m2c_and_explicitly_passed_to_builder():
    source = MAIN.read_text(encoding="utf-8")
    assert "declare_parameter('acados_cache_dir', '')" in source
    assert "acados_cache_dir=" in source
    assert "self.acados_cache_dir" in source


def test_m2c_all_controllers_keep_separate_workdirs_but_share_one_persistent_solver_cache():
    launch = M2C_LAUNCH.read_text(encoding="utf-8")
    runner = M2C_RUNNER.read_text(encoding="utf-8")

    # Preserve per-process log/CWD isolation from the commissioned M2C lifecycle.
    assert 'DeclareLaunchArgument("controller_work_dir")' in launch
    assert "cwd=str(work_dir)" in launch
    assert 'controller_work_dir:="$CONTROLLER_WORK_ROOT/drone_${i}"' in runner

    # Share only generated ACADOS artifacts across the four independent processes.
    assert '"acados_cache_dir"' in launch
    assert 'DeclareLaunchArgument(' in launch
    assert '"acados_cache_dir": str(cache_dir)' in launch
    assert 'ACADOS_CACHE_ROOT="$REPO/build/acados_cache/payload_mpc"' in runner
    assert 'acados_cache_dir:="$ACADOS_CACHE_ROOT"' in runner

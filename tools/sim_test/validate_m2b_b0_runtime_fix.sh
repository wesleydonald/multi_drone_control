#!/usr/bin/env bash
# Pure/static hard gate for the M2B B0 Sep-10 runtime-fix increment.
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$SCRIPT_DIR/../.." && pwd)"
cd "$REPO"

if ! python3 -c 'import pytest' >/dev/null 2>&1; then
  echo "ERROR: Python pytest module is not installed."
  echo "Install it with: sudo apt install python3-pytest"
  exit 2
fi

export PYTHONPATH="$REPO/src/tejen_mission:$REPO/src/tejen_mpc:$REPO/src/tejen_utility_objects${PYTHONPATH:+:$PYTHONPATH}"

echo "[1/4] M2A + M2B geometry / mission / simulation contracts"
python3 -m pytest -q \
  src/tejen_mission/test/test_m2a_attachment_detection.py \
  src/tejen_mission/test/test_m2a_attachment_geometry_se3.py \
  src/tejen_mission/test/test_m2a_bench_fixture_contract.py \
  src/tejen_mission/test/test_m2a_kinematic_probe_contract.py \
  src/tejen_mission/test/test_m2a_named_transform_runtime.py \
  src/tejen_mission/test/test_m2a_runtime_contract.py \
  src/tejen_mission/test/test_m2a_runtime_support.py \
  src/tejen_mission/test/test_m2a_sim_contract.py \
  src/tejen_mission/test/test_payload_geometry.py \
  src/tejen_mission/test/test_ring_net_geometry_contract.py \
  src/tejen_mission/test/test_m2b_b0_sim_contract.py \
  src/tejen_mission/test/test_m2b_mission_contract.py \
  src/tejen_mission/test/test_m2b_b0_runtime_fix_contract.py \
  src/tejen_mission/test/test_m2b_b0_ground_spawn_contract.py \
  src/tejen_mpc/test/test_m2b_mpc_modes.py \
  src/tejen_mpc/test/test_m2b_arm_interlock_contract.py

echo "[2/4] Existing controller regressions + M2B controller contracts"
python3 -m pytest -q \
  src/tejen_mpc/test/test_c1d_reference_fault_contract.py \
  src/tejen_mpc/test/test_c1e_logging_contract.py \
  src/tejen_mpc/test/test_callback_manager_safety_contract.py \
  src/tejen_mpc/test/test_pendulum_freshness_contract.py \
  src/tejen_mpc/test/test_m2b_mpc_modes.py \
  src/tejen_mpc/test/test_m2b_arm_interlock_contract.py

echo "[3/4] Python / shell syntax"
python3 -m py_compile \
  src/tejen_mission/tejen_mission/m2a_physical_capture_manager.py \
  src/tejen_mission/tejen_mission/m2b_b0_telemetry.py \
  src/tejen_mission/tejen_mission/m2b_ground_initializer.py \
  src/tejen_mission/tejen_mission/m2b_ground_start.py \
  src/tejen_mission/tejen_mission/m2b_ground_spawn.py \
  src/tejen_mission/launch/m2b_b0_single_attachment.launch.py \
  src/tejen_mission/setup.py \
  src/tejen_mission/test/test_m2b_b0_runtime_fix_contract.py \
  src/tejen_mission/test/test_m2b_b0_ground_spawn_contract.py
bash -n tools/sim_test/run_m2b_b0.sh
bash -n tools/sim_test/cleanup_m2b_b0_stale.sh
bash -n tools/sim_test/validate_m2b_b0_runtime_fix.sh

echo "[4/4] B0 world XML"
python3 - <<'PY'
import xml.etree.ElementTree as ET
ET.parse('simulation_assets/tejen/world_m2b_single_attachment.sdf')
print('SDF XML parse: PASS')
PY

echo
echo "PASS: M2B B0 runtime-fix pure/static gate complete."
echo "Expected regression counts above: 329 passed, then 24 passed."

#!/usr/bin/env bash
# M2A bench/runtime hard gate. Deliberately uses set -e only.
set -e

REPO="${REPO:-$(cd "$(dirname "$0")/../.." && pwd)}"
cd "$REPO"
export PYTHONPATH="$REPO/src/tejen_mission${PYTHONPATH:+:$PYTHONPATH}"

echo "=== M2A BENCH / DETECTOR FOCUSED TESTS ==="
python3 -m pytest -q \
  src/tejen_mission/test/test_ring_net_geometry_contract.py \
  src/tejen_mission/test/test_m2a_attachment_detection.py \
  src/tejen_mission/test/test_m2a_sim_contract.py \
  src/tejen_mission/test/test_m2a_runtime_support.py \
  src/tejen_mission/test/test_m2a_runtime_contract.py \
  src/tejen_mission/test/test_m2a_kinematic_probe_contract.py \
  src/tejen_mission/test/test_m2a_bench_fixture_contract.py \
  src/tejen_mission/test/test_m2a_named_transform_runtime.py \
  analysis/m2_attachment/test/test_analyse_attachment.py

echo "=== PYTHON / SHELL / XML SANITY ==="
python3 -m py_compile \
  src/tejen_mission/tejen_mission/attachment_detection.py \
  src/tejen_mission/tejen_mission/m2a_attachment_runtime.py \
  src/tejen_mission/tejen_mission/m2a_attachment_replay.py \
  src/tejen_mission/tejen_mission/m2_attachment_observer.py \
  src/tejen_mission/tejen_mission/m2a_physical_capture_manager.py \
  src/tejen_mission/tejen_mission/m2a_gazebo_joint_truth_bridge.py \
  tools/sim_test/m2a_attachment_probe.py \
  tools/sim_test/apply_m2a_paused_bootstrap_revert.py \
  analysis/m2_attachment/analyse_attachment.py

bash -n tools/sim_test/validate_m2a_runtime_increment.sh tools/sim_test/run_m2a_attachment_foundations.sh

python3 - <<'PY'
import xml.etree.ElementTree as ET
for path in [
    "simulation_assets/tejen/modelLargeM2BallMagnet.sdf",
    "simulation_assets/tejen/m2a_x3_support.sdf",
    "simulation_assets/tejen/m2a_ring_fixture.sdf",
    "simulation_assets/tejen/world_m2a_attachment_foundations.sdf",
]:
    ET.parse(path)
    print("XML OK:", path)
PY

echo "=== M1 / EXISTING RUNNER REGRESSION GATE ==="
python3 -m pytest -q \
  src/tejen_mission/test/test_dynamic_world_model.py \
  src/tejen_mission/test/test_m1_ring_integration_contract.py \
  tools/sim_test/test \
  analysis/sim_test/test

echo "=== ROS 2 HUMBLE BUILD / PACKAGE TEST ==="
source /opt/ros/humble/setup.bash
rm -rf build/tejen_mission install/tejen_mission
colcon build \
  --packages-select tejen_mission \
  --symlink-install \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    -DCMAKE_PREFIX_PATH="$HOME/.local;/opt/ros/humble"
source install/setup.bash
colcon test --packages-select tejen_mission --event-handlers console_direct+
colcon test-result --verbose

echo
echo "PASS: M2A bench fixture/runtime validation is green."
echo "Next: run SANITY mode only and inspect the scene before any scripted motion."

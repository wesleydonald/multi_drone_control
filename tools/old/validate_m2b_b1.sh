#!/usr/bin/env bash
# Deterministic pure/static validation gate for M2B B1 local attachment.
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$SCRIPT_DIR/../.." && pwd)"
cd "$REPO"

export PYTHONPATH="$REPO/src/tejen_mission:$REPO/src/tejen_mpc:$REPO/src/tejen_utility_objects${PYTHONPATH:+:$PYTHONPATH}"

echo "[1/5] M2A + M2B geometry / mission / simulation contracts"
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
  src/tejen_mission/test/test_m2b_b0_ground_spawn_contract.py \
  src/tejen_mission/test/test_m2b_b0_runtime_fix_contract.py \
  src/tejen_mission/test/test_m2b_b0_sim_contract.py \
  src/tejen_mission/test/test_m2b_b1_policy_contract.py \
  src/tejen_mission/test/test_m2b_b1_proof_contract.py \
  src/tejen_mission/test/test_m2b_b1_runtime_contract.py \
  src/tejen_mission/test/test_m2b_b1_phase_framework_contract.py \
  src/tejen_mission/test/test_m2b_b1_feedback_supervisor_contract.py \
  src/tejen_mission/test/test_m2b_b1_latch_stability_contract.py \
  src/tejen_mission/test/test_m2b_b1_selective_collision_contract.py \
  src/tejen_mission/test/test_m2b_b1_topology_discriminator_contract.py \
  src/tejen_mission/test/test_m2b_b1_manual_operator_authority_contract.py \
  src/tejen_mission/test/test_m2b_b2_cpp_transit_contract.py \
  src/tejen_mission/test/test_m2b_sphere_contact_model.py \
  src/tejen_mission/test/test_m2b_mission_contract.py

echo "[2/5] Relevant controller + M2B controller regressions"
python3 -m pytest -q \
  src/tejen_mpc/test/test_c1d_reference_fault_contract.py \
  src/tejen_mpc/test/test_pendulum_freshness_contract.py \
  src/tejen_mpc/test/test_callback_manager_safety_contract.py \
  src/tejen_mpc/test/test_c1e_logging_contract.py \
  src/tejen_mpc/test/test_m2b_mpc_modes.py \
  src/tejen_mpc/test/test_m2b_arm_interlock_contract.py \
  src/tejen_mpc/test/test_thrust_ratio_feedback_policy.py

echo "[3/5] Python syntax"
python3 -m py_compile \
  src/tejen_mission/tejen_mission/attachment_detection.py \
  src/tejen_mission/tejen_mission/m2_attachment_observer.py \
  src/tejen_mission/tejen_mission/m2a_attachment_replay.py \
  src/tejen_mission/tejen_mission/m2a_attachment_runtime.py \
  src/tejen_mission/tejen_mission/m2a_physical_capture_manager.py \
  src/tejen_mission/tejen_mission/m2b_attachment_mission.py \
  src/tejen_mission/tejen_mission/m2b_b1_telemetry.py \
  src/tejen_mission/tejen_mission/m2b_ground_spawn.py \
  src/tejen_mission/tejen_mission/online_join_planner.py \
  src/tejen_mission/launch/m2b_b1_single_attachment.launch.py \
  src/tejen_mpc/tejen_mpc/main.py \
  src/tejen_mission/setup.py

echo "[4/5] Shell syntax"
bash -n tools/sim_test/run_m2b_b1.sh
bash -n tools/sim_test/run_m2b_b2.sh
bash -n tools/sim_test/cleanup_m2b_stale.sh

echo "[5/5] World / ROS-Gazebo bridge static contracts"
python3 - <<'PY'
from pathlib import Path
import xml.etree.ElementTree as ET

ET.parse("simulation_assets/tejen/world_m2b_single_attachment.sdf")
bridge = Path("src/tejen_mission/config/m2b_gz_joint_bridge.yaml").read_text(encoding="utf-8")
assert bridge.count("direction: ROS_TO_GZ") == 2
assert bridge.count('ros_type_name: "std_msgs/msg/Empty"') == 2
assert bridge.count('gz_type_name: "gz.msgs.Empty"') == 2
assert '/m2b/gz/attach' in bridge and '/payload/attach' in bridge
assert '/m2b/gz/detach' in bridge and '/payload/detach' in bridge
print("world XML + bridge config: PASS")
PY

echo "M2B B1/B2 PURE/STATIC VALIDATION PASS"

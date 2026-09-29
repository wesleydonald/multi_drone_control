#!/usr/bin/env bash
# M1 IRL operator runner: real tracked pickup object + moving virtual ring.
# Manual ARM/TAKEOFF/DISARM authority remains with the operator.

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
GUI="${1:-true}"
MODE="${2:-run}"
CONFIG="$ROOT/src/tejen_mission/config/irl_commissioning.yaml"

if [[ "$GUI" != "true" && "$GUI" != "false" ]]; then
  echo "ERROR: first argument must be true or false (RViz GUI)."
  echo "Usage: bash tools/sim_test/run_m1_irl_virtual_ring.sh [true|false] [run|check]"
  exit 2
fi
if [[ "$MODE" != "run" && "$MODE" != "check" ]]; then
  echo "ERROR: second argument must be run or check."
  exit 2
fi

cd "$ROOT" || exit 2

if [[ ! -f "$CONFIG" ]]; then
  echo "ERROR: missing $CONFIG"
  exit 2
fi

if [[ -f /opt/ros/humble/setup.bash ]]; then
  source /opt/ros/humble/setup.bash
else
  echo "ERROR: /opt/ros/humble/setup.bash not found."
  exit 2
fi

if [[ -f "$ROOT/install/setup.bash" ]]; then
  source "$ROOT/install/setup.bash"
else
  echo "ERROR: install/setup.bash not found. Build the workspace first."
  exit 2
fi

echo "============================================================================="
echo "M1 IRL - REAL PICKUP + MOVING VIRTUAL RING"
echo "Repository: $ROOT"
echo "Config:     $CONFIG"
echo "RViz:       $GUI"
echo "Authority:  MANUAL ARM / TAKEOFF / DISARM"
echo "OptiTrack:  ID6 pickup | ID7 drone | ID8 magnet | ID9 ring reserved"
echo "============================================================================="

python3 - "$CONFIG" <<'PY'
import math
import sys
from pathlib import Path

import yaml

path = Path(sys.argv[1])
data = yaml.safe_load(path.read_text())
errors = []


def params(node):
    try:
        return data[node]["ros__parameters"]
    except Exception:
        errors.append(f"missing {node}.ros__parameters")
        return {}

operator = params("m1_irl_operator")
mocap = params("motion_capture_publisher_irl")
magnet = params("magnet_attachment_manager_irl")
planner = params("online_join_planner")
fake = params("fake_cooperative_transport_world")
controller = params("tejen_mpc")
backend = params("dynamic_planner_transfer_backend")

if operator.get("lab_config_ready") is not True:
    errors.append("m1_irl_operator.lab_config_ready must be true after lab geometry/target review")

expected_ids = {
    "pickup_rigid_body_id": "6",
    "quad_rigid_body_id": "7",
    "magnet_rigid_body_id": "8",
    "ring_rigid_body_id": "9",
}
for key, expected in expected_ids.items():
    if str(mocap.get(key)) != expected:
        errors.append(f"{key} must be {expected}, got {mocap.get(key)!r}")

if mocap.get("pickup_object_pose_topic") != "/irl/pickup_object/pose":
    errors.append("pickup object topic must be /irl/pickup_object/pose")
if planner.get("use_static_pickup_object") is not False:
    errors.append("online_join_planner.use_static_pickup_object must be false")
if planner.get("pickup_object_pose_topic") != mocap.get("pickup_object_pose_topic"):
    errors.append("planner and mocap pickup-object topics disagree")
if magnet.get("pickup_object_pose_topic") != mocap.get("pickup_object_pose_topic"):
    errors.append("magnet manager and mocap pickup-object topics disagree")
if int(planner.get("pickup_object_index", -999)) != 0 or int(magnet.get("pickup_object_index", -999)) != 0:
    errors.append("tracked pickup PoseArray index must be 0")

for axis in "xyz":
    a = float(magnet.get(f"pickup_point_offset_{axis}", math.nan))
    b = float(planner.get(f"payload_pickup_point_{axis}", math.nan))
    if not (math.isfinite(a) and math.isfinite(b) and abs(a - b) <= 1e-9):
        errors.append(f"pickup point {axis} mismatch between magnet manager and planner")
    a = float(magnet.get(f"magnet_marker_to_contact_face_{axis}", math.nan))
    b = float(planner.get(f"magnet_marker_to_contact_face_{axis}", math.nan))
    if not (math.isfinite(a) and math.isfinite(b) and abs(a - b) <= 1e-9):
        errors.append(f"magnet contact offset {axis} mismatch between magnet manager and planner")

for axis in "xyz":
    size = float(planner.get(f"payload_collision_size_{axis}", 0.0))
    half = float(backend.get(f"payload_half_{axis}_m", 0.0))
    if not math.isfinite(size) or size <= 0.0:
        errors.append(f"payload_collision_size_{axis} must be measured and > 0")
    if not math.isfinite(half) or half <= 0.0:
        errors.append(f"payload_half_{axis}_m must be > 0")
    if math.isfinite(size) and math.isfinite(half) and abs(half - 0.5 * size) > 1e-9:
        errors.append(f"payload_half_{axis}_m must equal 0.5 * payload_collision_size_{axis}")

for axis in "xyz":
    value = float(backend.get(f"payload_center_from_magnet_{axis}_m", math.nan))
    if not math.isfinite(value):
        errors.append(f"payload_center_from_magnet_{axis}_m must be finite")

if planner.get("drop_point_source") != "payload":
    errors.append("M1 IRL drop_point_source must be payload (virtual moving ring)")
if planner.get("payload_pose_topic") != "/fake_payload/pose" or planner.get("payload_twist_topic") != "/fake_payload/twist":
    errors.append("M1 IRL must use /fake_payload pose/twist for the virtual moving ring")
if planner.get("commissioning_hold_after_lift") is not False:
    errors.append("commissioning_hold_after_lift must be false for the full M1 run")
if planner.get("commissioning_hold_before_drop") is not False:
    errors.append("commissioning_hold_before_drop must be false so the 60 g object is actually released")

if fake.get("enable_fake_obstacles") is not False or int(fake.get("fake_drone_count", -1)) != 0:
    errors.append("fake companion drones must be disabled for tomorrow's M1 IRL")
if fake.get("payload_committed_trajectory_topic") != "/fake_payload/committed_trajectory":
    errors.append("virtual ring commitment topic mismatch")
for key in ("center_x", "center_y", "center_z", "radius", "omega"):
    value = float(fake.get(key, math.nan))
    if not math.isfinite(value):
        errors.append(f"virtual ring {key} must be finite")
if float(fake.get("radius", 0.0)) < 0.0:
    errors.append("virtual ring radius must be non-negative")

for key in ("use_external_reference", "enable_thrust_ratio_ukf"):
    if controller.get(key) is not True:
        errors.append(f"tejen_mpc.{key} must be true")

xy_bias_mode = str(controller.get("xy_bias_mode", "")).strip().lower()
if not xy_bias_mode:
    xy_bias_mode = (
        "legacy_integral"
        if controller.get("enable_xy_integral_action") is True
        else "none"
    )
if xy_bias_mode not in {"legacy_integral", "lateral_disturbance"}:
    errors.append(
        "tejen_mpc.xy_bias_mode must be legacy_integral or "
        "lateral_disturbance for a full M1 flight; shadow/none are test-only"
    )
if (
    xy_bias_mode == "legacy_integral"
    and controller.get("enable_xy_integral_action") is not True
):
    errors.append(
        "legacy_integral mode requires enable_xy_integral_action=true"
    )

if errors:
    print("\nIRL CONFIG GATE: FAIL")
    for error in errors:
        print(f"  - {error}")
    print("\nDo not run the full M1 flight. Edit config/irl_commissioning.yaml and rerun the check.")
    raise SystemExit(3)

print("\nIRL CONFIG GATE: PASS")
print("  rigid bodies: pickup=6, quad=7, magnet=8, ring=9 reserved")
print("  real pickup tracking: enabled")
print("  moving virtual ring: enabled; fake companions disabled")
print("  physical drop: enabled (commissioning_hold_before_drop=false)")
print(f"  XY bias rejection: {xy_bias_mode}; thrust-ratio UKF: enabled")
PY
status=$?
if [[ $status -ne 0 ]]; then
  exit $status
fi

if [[ "$MODE" == "check" ]]; then
  echo "CHECK ONLY: configuration passed; no ROS flight stack was started."
  exit 0
fi

echo
echo "Before ARM, verify the props-off gates in commandsForM1IRL_VirtualRing_20260915_v1.txt."
echo "Known emergency disarm:"
echo "  ros2 service call /drone_arming_service interfaces/srv/SetArming \"{arm: false}\""
echo
echo "Starting full stack. This does NOT arm or request takeoff automatically."

exec ros2 launch tejen_mission m1_irl_virtual_ring.launch.py gui:="$GUI"

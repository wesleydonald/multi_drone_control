#!/usr/bin/env bash
# Supervised M2B B0 ground-start commissioning run.
# Real X3 only. The world remains paused until the existing tether/magnet links
# have been placed in the validated near-horizontal ground-start configuration.
set -e

# Structured B0 telemetry is written by m2b_b0_telemetry into b0_status.csv.
# It records the established contract topics /m2a/sim/joint_detached_truth,
# /join_planner/arm_permission, and /join_planner/mpc_mode without spawning
# separate ros2 topic echo processes.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$SCRIPT_DIR/../.." && pwd)"
STAMP="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="$REPO/logs/m2_attachment/m2b_b0_${STAMP}"
MODE="${1:-sanity}"
GUI="${2:-true}"
ASSIGNED_PLATE_ID="${M2B_ASSIGNED_PLATE_ID:-0}"
ACTIVE_PGID_FILE="$REPO/logs/m2_attachment/.m2b_b0_active_pgid"
LOCK_FILE="$REPO/logs/m2_attachment/.m2b_b0.lock"
STATUS_CSV="$LOG_DIR/b0_status.csv"
mkdir -p "$LOG_DIR" "$REPO/logs/m2_attachment"

if [[ "$MODE" != "sanity" && "$MODE" != "success" ]]; then
  echo "ERROR: first argument must be 'sanity' or 'success'."
  echo "Usage: bash tools/sim_test/run_m2b_b0.sh sanity true"
  echo "   or: bash tools/sim_test/run_m2b_b0.sh success true"
  exit 2
fi

cd "$REPO"
source /opt/ros/humble/setup.bash
source install/setup.bash
export GZ_SIM_RESOURCE_PATH="$REPO/simulation_assets${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"

WORLD_TEMPLATE="$REPO/simulation_assets/tejen/world_m2b_single_attachment.sdf"
GENERATED_DIR="$LOG_DIR/generated"
GENERATED_WORLD="$GENERATED_DIR/m2b_generated_world.sdf"
GROUND_MANIFEST="$LOG_DIR/ground_start.json"
GROUND_MEASURED="$LOG_DIR/ground_start_measured.json"
if [[ ! -f "$WORLD_TEMPLATE" ]]; then
  echo "ERROR: missing $WORLD_TEMPLATE"
  exit 1
fi
if ! grep -q 'modelLargeM2BallMagnet.sdf' "$WORLD_TEMPLATE"; then
  echo "ERROR: B0 world does not reference the real M2 X3 model."
  exit 1
fi
if grep -qi 'm2a_x3_support\|kinematic' "$WORLD_TEMPLATE"; then
  echo "ERROR: abandoned M2A support fixture detected in B0 world."
  exit 1
fi

# Prevent two B0 runners from starting concurrently.
exec 9>"$LOCK_FILE"
if ! flock -n 9; then
  echo "ERROR: another M2B B0 runner currently owns $LOCK_FILE"
  exit 1
fi

assert_clean_gazebo() {
  local stale
  stale="$(pgrep -af 'gz sim' 2>/dev/null || true)"
  if [[ -n "$stale" ]]; then
    echo "ERROR: an existing Gazebo Sim process is already alive."
    printf '%s\n' "$stale"
    echo "Run: bash tools/sim_test/cleanup_m2b_b0_stale.sh"
    echo "Then rerun this B0 command."
    return 1
  fi
}

assert_clean_ros_graph() {
  local nodes
  nodes="$(ros2 node list 2>/dev/null || true)"
  local stale=""
  for node in \
    /online_join_planner \
    /tejen_mpc \
    /m2b_gazebo_joint_truth_bridge \
    /m2b_physical_capture_manager \
    /m2b_b0_telemetry \
    /motion_capture_emulator \
    /betaflight_communication \
    /pendulum_state_publisher \
    /pose_bridge \
    /control_bridge \
    /camera_bridge \
    /camera_info_bridge \
    /pose_bridgey2 \
    /pose_bridgey3 \
    /payload_and_land_bridge \
    /rviz2 \
    /static_tf_mocap_drone \
    /static_tf_orbslam_drone \
    /mocap_drone/robot_state_publisher_mocap_drone \
    /orbslam_drone/robot_state_publisher_orbslam_drone; do
    if printf '%s\n' "$nodes" | grep -qx "$node"; then
      stale="$stale $node"
    fi
  done
  if [[ -n "$stale" ]]; then
    echo "ERROR: stale simulation ROS nodes are already alive:$stale"
    echo "Run: bash tools/sim_test/cleanup_m2b_b0_stale.sh"
    echo "Then rerun this B0 command."
    return 1
  fi
}

if [[ -f "$ACTIVE_PGID_FILE" ]]; then
  OLD_PGID="$(cat "$ACTIVE_PGID_FILE" 2>/dev/null || true)"
  if [[ "$OLD_PGID" =~ ^[0-9]+$ ]] && kill -0 -- -"$OLD_PGID" 2>/dev/null; then
    echo "ERROR: a previous M2B launch process group is still alive (PGID $OLD_PGID)."
    echo "Run: bash tools/sim_test/cleanup_m2b_b0_stale.sh"
    exit 1
  fi
  rm -f "$ACTIVE_PGID_FILE"
fi
assert_clean_gazebo
assert_clean_ros_graph

LAUNCH_PID=""
LAUNCH_PGID=""
cleanup_launch_group() {
  set +e
  if [[ -n "$LAUNCH_PGID" ]] && kill -0 -- -"$LAUNCH_PGID" 2>/dev/null; then
    kill -INT -- -"$LAUNCH_PGID" 2>/dev/null || true
    for _ in $(seq 1 70); do
      kill -0 -- -"$LAUNCH_PGID" 2>/dev/null || break
      sleep 0.10
    done
  fi
  if [[ -n "$LAUNCH_PGID" ]] && kill -0 -- -"$LAUNCH_PGID" 2>/dev/null; then
    kill -TERM -- -"$LAUNCH_PGID" 2>/dev/null || true
    for _ in $(seq 1 30); do
      kill -0 -- -"$LAUNCH_PGID" 2>/dev/null || break
      sleep 0.10
    done
  fi
  if [[ -n "$LAUNCH_PGID" ]] && kill -0 -- -"$LAUNCH_PGID" 2>/dev/null; then
    kill -KILL -- -"$LAUNCH_PGID" 2>/dev/null || true
  fi
  if [[ -n "$LAUNCH_PID" ]]; then
    wait "$LAUNCH_PID" 2>/dev/null || true
  fi
  rm -f "$ACTIVE_PGID_FILE"
}
interrupt_cleanup() {
  trap - EXIT INT TERM
  cleanup_launch_group
  exit 130
}
trap cleanup_launch_group EXIT
trap interrupt_cleanup INT TERM

world_step_once() {
  gz service -s /world/quadcopter/control \
    --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean \
    --timeout 1000 --req 'multi_step: 1' >>"$LOG_DIR/runtime.log" 2>&1
}

wait_for_service() {
  local service="$1"
  local timeout_s="$2"
  local started="$(date +%s)"
  while true; do
    if ros2 service list 2>/dev/null | grep -qx "$service"; then
      return 0
    fi
    if (( $(date +%s) - started >= timeout_s )); then
      echo "ERROR: timeout waiting for service $service"
      return 1
    fi
    sleep 0.25
  done
}

wait_for_node() {
  local node="$1"
  local timeout_s="$2"
  local started="$(date +%s)"
  while true; do
    if ros2 node list 2>/dev/null | grep -qx "$node"; then
      return 0
    fi
    if (( $(date +%s) - started >= timeout_s )); then
      echo "ERROR: timeout waiting for node $node"
      return 1
    fi
    sleep 0.20
  done
}

csv_latest_value_is() {
  local column="$1"
  local expected="$2"
  python3 - "$STATUS_CSV" "$column" "$expected" <<'PY'
import csv
import sys
from pathlib import Path
path = Path(sys.argv[1])
column = sys.argv[2]
expected = sys.argv[3].strip().lower()
if not path.is_file():
    raise SystemExit(1)
latest = ""
try:
    with path.open(newline='') as handle:
        for row in csv.DictReader(handle):
            value = str(row.get(column, '')).strip()
            if value:
                latest = value.lower()
except (OSError, csv.Error):
    raise SystemExit(1)
raise SystemExit(0 if latest == expected else 1)
PY
}

wait_for_csv_value() {
  local column="$1"
  local expected="$2"
  local timeout_s="$3"
  local started="$(date +%s)"
  while true; do
    if csv_latest_value_is "$column" "$expected"; then
      return 0
    fi
    if (( $(date +%s) - started >= timeout_s )); then
      echo "ERROR: timeout waiting for $column=$expected in $STATUS_CSV"
      return 1
    fi
    sleep 0.10
  done
}

echo "M2B B0 - bootstrap detach + real-X3 takeoff"
echo "Repository: $REPO"
echo "Evidence:   $LOG_DIR"
echo "Mode:       $MODE"
echo "Gazebo GUI: $GUI"
echo "Plate:      $ASSIGNED_PLATE_ID"
echo "Telemetry:  $STATUS_CSV"
echo "World is generated from the exact real X3 SDF before Gazebo starts."

# Build the articulated initial condition BEFORE Gazebo creates the ball joints.
# The repository X3 is untouched; the generated copy is retained with the run.
mkdir -p "$GENERATED_DIR"
python3 -m tejen_mission.m2b_ground_spawn generate \
  --source-x3 "$REPO/simulation_assets/tejen/modelLargeM2BallMagnet.sdf" \
  --world-template "$WORLD_TEMPLATE" \
  --output-dir "$GENERATED_DIR" \
  --manifest "$GROUND_MANIFEST" \
  --assigned-plate-id "$ASSIGNED_PLATE_ID" \
  >"$LOG_DIR/runtime.log" 2>&1

export GZ_SIM_RESOURCE_PATH="$GENERATED_DIR:$REPO/simulation_assets${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"

# New session means the launch tree has one owned process group. This fixes the
# previous runner's child-process leak on Ctrl-C or gate failure.
setsid ros2 launch tejen_mission m2b_b0_single_attachment.launch.py \
  gui:="$GUI" \
  assigned_plate_id:="$ASSIGNED_PLATE_ID" \
  m2b_log_dir:="$LOG_DIR" \
  world_path:="$GENERATED_WORLD" \
  >>"$LOG_DIR/runtime.log" 2>&1 &
LAUNCH_PID=$!
LAUNCH_PGID=$LAUNCH_PID
echo "$LAUNCH_PGID" >"$ACTIVE_PGID_FILE"

# Wait for the Gazebo world and B0 evidence plumbing without advancing physics.
wait_for_node /m2b_b0_telemetry 12
wait_for_node /m2b_gazebo_joint_truth_bridge 12

# The articulated pose already exists in SDF before Gazebo creates the joints.
# Advance a small number of PAUSED steps so pose publishers / mocap can report
# actual Gazebo state, then hard-gate on that measured state.
for _ in $(seq 1 20); do
  world_step_once
  sleep 0.03
done

if ! python3 -m tejen_mission.m2b_ground_spawn check \
    --status-csv "$STATUS_CSV" \
    --manifest "$GROUND_MANIFEST" \
    --output-json "$GROUND_MEASURED" \
    --max-age-s 2.0 \
    >>"$LOG_DIR/runtime.log" 2>&1; then
  echo "ERROR: measured Gazebo ground geometry failed the B0 hard gate."
  echo "Inspect $GROUND_MEASURED and $STATUS_CSV"
  exit 1
fi
echo "Measured ground geometry: VERIFIED"

# The M2B planner owns the raw-detach request. Advance paused physics until the
# independent raw truth bridge reports DETACHED into the structured CSV.
DETACHED=false
for _ in $(seq 1 80); do
  world_step_once || true
  if csv_latest_value_is joint_detached true; then
    DETACHED=true
    break
  fi
  sleep 0.05
done
if [[ "$DETACHED" != true ]]; then
  echo "ERROR: M2B bootstrap did not obtain raw Gazebo DETACHED truth."
  echo "Inspect $LOG_DIR/runtime.log and $STATUS_CSV"
  exit 1
fi
echo "Bootstrap raw joint truth: DETACHED"

# Only after release do we run free physics and the configured ground-settle dwell.
gz service -s /world/quadcopter/control \
  --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean \
  --timeout 3000 --req 'pause: false' >>"$LOG_DIR/runtime.log" 2>&1

wait_for_csv_value arm_permission true 10
echo "M2B arm permission: GRANTED"

# SANITY is deliberately a pre-arm gate. Leave the disarmed system running so
# the user can inspect the grounded real X3, near-horizontal tether and plate
# contact before any takeoff is attempted.
if [[ "$MODE" == "sanity" ]]; then
  cat >"$LOG_DIR/b0_result.txt" <<EOF
SANITY_PASS
stage=M2B_B0_GROUND_SANITY
assigned_plate_id=$ASSIGNED_PLATE_ID
joint_detach_verified=true
arm_permission_verified=true
arm_command_sent=false
structured_telemetry=b0_status.csv
ground_start_geometry=ground_start.json
ground_geometry_verified=true
ground_start_measured=ground_start_measured.json
EOF
  echo
  echo "SANITY_PASS: startup detach and ground-settle gates are green."
  echo "DO NOT arm in this run. Visually check the grounded geometry now:"
  echo "  - real X3 resting normally above the floor"
  echo "  - tether approximately horizontal toward the assigned plate"
  echo "  - magnet at the plate, not below the floor"
  echo "  - no violent drift, jump, or constraint explosion"
  echo "Evidence directory: $LOG_DIR"
  echo "Press Ctrl-C here when inspection is complete."
  while true; do sleep 1; done
fi

wait_for_service /drone_arming_service 25
ARM_OUTPUT="$(ros2 service call /drone_arming_service interfaces/srv/SetArming '{arm: true}' 2>&1)"
printf '%s\n' "$ARM_OUTPUT" >>"$LOG_DIR/runtime.log"
if ! printf '%s\n' "$ARM_OUTPUT" | grep -Eqi 'success[=: ]+(true|True)'; then
  echo "ERROR: controller did not accept ARM."
  printf '%s\n' "$ARM_OUTPUT"
  exit 1
fi
wait_for_csv_value armed true 6
echo "Controller arming feedback: ARMED"

ros2 topic pub --once /drone_command std_msgs/msg/String '{data: TAKEOFF}' \
  >>"$LOG_DIR/runtime.log" 2>&1

wait_for_csv_value mission_phase M2_TAKEOFF_HOVER 40
wait_for_csv_value effective_mpc_mode FREE_SWING 5
echo "B0 hover phase reached. Holding for 5 s to expose immediate faults..."
sleep 5

# Fault gating is structured. Never grep human-readable runtime notes for a
# token such as FAULT_LATCHED_HOLD: the controller documentation itself contains
# that phrase even during healthy operation.
wait_for_csv_value external_reference_fault_latched false 5
wait_for_csv_value pendulum_fault_latched false 5
if ! csv_latest_value_is mission_phase M2_TAKEOFF_HOVER \
    || ! csv_latest_value_is external_reference_fault_latched false \
    || ! csv_latest_value_is pendulum_fault_latched false; then
  echo "ERROR: B0 left hover or a controller fault became latched."
  echo "Inspect $STATUS_CSV and $LOG_DIR/runtime.log"
  exit 1
fi

cat >"$LOG_DIR/b0_result.txt" <<EOF
PASS
stage=M2B_B0
assigned_plate_id=$ASSIGNED_PLATE_ID
joint_detach_verified=true
arm_permission_verified=true
armed_feedback_verified=true
hover_phase_verified=true
effective_hover_mpc_mode=FREE_SWING
hover_observation_s=5
structured_telemetry=b0_status.csv
ground_geometry_verified=true
ground_start_measured=ground_start_measured.json
EOF

echo
echo "PASS: M2B B0 automated gates reached with no immediate critical fault signature."
echo "Please visually verify the real X3 is stably hovering and the tether behaviour is sane."
echo "Evidence directory: $LOG_DIR"
echo "Structured telemetry: $STATUS_CSV"
echo "Gazebo remains open for inspection. Press Ctrl-C here when finished."
while true; do sleep 1; done

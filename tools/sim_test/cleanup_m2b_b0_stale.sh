#!/usr/bin/env bash
# Explicit cleanup for stale processes from interrupted pre-fix M2B B0 runs.
# Only use this when no other thesis simulation is intentionally running.
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$SCRIPT_DIR/../.." && pwd)"
ACTIVE_PGID_FILE="$REPO/logs/m2_attachment/.m2b_b0_active_pgid"

# Be self-contained like the supervised runner. The ROS daemon cleanup below must
# use the same Humble environment as the thesis workspace.
source /opt/ros/humble/setup.bash
if [[ -f "$REPO/install/setup.bash" ]]; then
  source "$REPO/install/setup.bash"
fi

if [[ -f "$ACTIVE_PGID_FILE" ]]; then
  PGID="$(cat "$ACTIVE_PGID_FILE" 2>/dev/null || true)"
  if [[ "$PGID" =~ ^[0-9]+$ ]] && kill -0 -- -"$PGID" 2>/dev/null; then
    echo "Stopping recorded M2B process group $PGID"
    kill -INT -- -"$PGID" 2>/dev/null || true
    sleep 2
    kill -TERM -- -"$PGID" 2>/dev/null || true
    sleep 1
    kill -KILL -- -"$PGID" 2>/dev/null || true
  fi
  rm -f "$ACTIVE_PGID_FILE"
fi

# Legacy B0 runner did not own a process group, so remove only the known
# simulation processes that can contaminate this single-drone commissioning run.
patterns=(
  "gz sim"
  "m2b_b0_single_attachment.launch.py"
  "$REPO/install/tejen_mission/lib/tejen_mission/online_join_planner"
  "$REPO/install/tejen_mpc/lib/tejen_mpc/main"
  "$REPO/install/tejen_mission/lib/tejen_mission/m2a_gazebo_joint_truth_bridge"
  "$REPO/install/tejen_mission/lib/tejen_mission/m2a_physical_capture_manager"
  "$REPO/install/tejen_mission/lib/tejen_mission/m2b_b0_telemetry"
  "$REPO/install/tejen_mission/lib/tejen_mission/m2b_ground_initializer"
  "$REPO/install/tejen_mission/lib/tejen_mission/pendulum_state_publisher"
  "$REPO/install/simulation_communication/lib/simulation_communication/motion_capture_emulator"
  "$REPO/install/simulation_communication/lib/simulation_communication/betaflight_communication"
  "ros_gz_bridge.*parameter_bridge"
  "rviz2"
  "robot_state_publisher"
  "static_transform_publisher"
)

for sig in INT TERM KILL; do
  for pattern in "${patterns[@]}"; do
    pkill -"$sig" -u "$(id -u)" -f "$pattern" 2>/dev/null || true
  done
  [[ "$sig" == "INT" ]] && sleep 2 || true
  [[ "$sig" == "TERM" ]] && sleep 1 || true
done

ros2 daemon stop >/dev/null 2>&1 || true
sleep 0.5
ros2 daemon start >/dev/null 2>&1 || true

echo "M2B stale-process cleanup complete."

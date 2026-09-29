#!/usr/bin/env bash
# Cleanup for interrupted M2B/M2C/M2D commissioning runs.
# Use only when no other thesis ROS/Gazebo session is intentionally running.
# M2D follows Wesley's proven clean-slate lesson: stale participants and Fast-DDS
# shared-memory state can make graph discovery lie or cross-talk into the next run.
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$SCRIPT_DIR/../.." && pwd)"
source /opt/ros/humble/setup.bash
if [[ -f "$REPO/install/setup.bash" ]]; then source "$REPO/install/setup.bash"; fi

for ACTIVE_PGID_FILE in \
  "$REPO/logs/m2_attachment/.m2b_b0_active_pgid" \
  "$REPO/logs/m2_attachment/.m2b_b1_active_pgid" \
  "$REPO/logs/m2_attachment/.m2c_active_pgid" \
  "$REPO/logs/m2_attachment/.m2d_active_pgid"; do
  if [[ -f "$ACTIVE_PGID_FILE" ]]; then
    while IFS= read -r PGID; do
      if [[ "$PGID" =~ ^[0-9]+$ ]] && kill -0 -- -"$PGID" 2>/dev/null; then
        echo "Stopping recorded M2B/M2C process group $PGID"
        kill -INT -- -"$PGID" 2>/dev/null || true
        sleep 2
        kill -TERM -- -"$PGID" 2>/dev/null || true
        sleep 1
        kill -KILL -- -"$PGID" 2>/dev/null || true
      fi
    done < "$ACTIVE_PGID_FILE"
    rm -f "$ACTIVE_PGID_FILE"
  fi
done

patterns=(
  "gz sim"
  "m2b_b0_single_attachment.launch.py"
  "m2b_b1_single_attachment.launch.py"
  "m2c_four_drone_ground.launch.py"
  "m2c_vehicle_controller.launch.py"
  "m2d_four_drone_sequential.launch.py"
  "$REPO/install/tejen_mission/lib/tejen_mission/online_join_planner"
  "$REPO/install/tejen_mission/lib/tejen_mission/m2c_fleet_manager"
  "$REPO/install/tejen_mission/lib/tejen_mission/m2d_fleet_supervisor"
  "$REPO/install/tejen_mission/lib/tejen_mission/m2d_ring_commitment"
  "$REPO/tools/sim_test/m2d_commissioning_observer.py"
  "$REPO/install/tejen_dynamic_planner/lib/tejen_dynamic_planner/dynamic_planner_transfer_backend"
  "$REPO/install/tejen_mission/lib/tejen_mission/fake_cooperative_transport_world"
  "$REPO/install/tejen_mpc/lib/tejen_mpc/main"
  "$REPO/install/tejen_mission/lib/tejen_mission/m2a_gazebo_joint_truth_bridge"
  "$REPO/install/tejen_mission/lib/tejen_mission/m2a_physical_capture_manager"
  "$REPO/install/tejen_mission/lib/tejen_mission/m2_attachment_observer"
  "$REPO/install/tejen_mission/lib/tejen_mission/m2b_b0_telemetry"
  "$REPO/install/tejen_mission/lib/tejen_mission/m2b_b1_telemetry"
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
# Stop the discovery daemon before clearing Fast-DDS SHM. Restarting the daemon
# first would immediately create fresh fastrtps_* segments and make the cleanup
# impossible to verify.
ros2 daemon stop >/dev/null 2>&1 || true
sleep 0.5

printf 'Clearing stale Fast-DDS shared-memory segments...\n'
DDS_UID="$(id -u)"
for DDS_PATH in /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*; do
  [[ -e "$DDS_PATH" ]] || continue
  [[ "$(stat -c %u "$DDS_PATH" 2>/dev/null || echo -1)" == "$DDS_UID" ]] || continue
  rm -f -- "$DDS_PATH" 2>/dev/null || true
done
DDS_LEFT=0
for DDS_PATH in /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*; do
  [[ -e "$DDS_PATH" ]] || continue
  [[ "$(stat -c %u "$DDS_PATH" 2>/dev/null || echo -1)" == "$DDS_UID" ]] || continue
  DDS_LEFT=$((DDS_LEFT + 1))
done
if (( DDS_LEFT > 0 )); then
  echo "ERROR: $DDS_LEFT user-owned Fast-DDS shared-memory segment(s) survived cleanup." >&2
  exit 1
fi

ros2 daemon start >/dev/null 2>&1 || true
echo "M2B/M2C/M2D stale-process + Fast-DDS cleanup complete."

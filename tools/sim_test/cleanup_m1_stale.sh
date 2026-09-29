#!/usr/bin/env bash
# Cleanup for interrupted M1 / C1F.6 commissioning runs.
# Safe default: refuse to interfere with a live supervised M1 run.
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$SCRIPT_DIR/../.." && pwd)"
ACTIVE_CAPTURE="$REPO/logs/c1f_experiments/.active_run.json"

cd "$REPO"

# ROS Humble setup scripts may reference variables before defining them.
# Source ROS/workspace setup with nounset disabled, then restore strict mode.
set +u
source /opt/ros/humble/setup.bash
if [[ -f "$REPO/install/setup.bash" ]]; then
  source "$REPO/install/setup.bash"
fi
set -u

# A real supervised run owns one of these top-level processes. Never kill it
# automatically: an active marker plus a live owner is not stale state.
LIVE_PRIMARY="$({
  ps -eo pid=,args= | awk -v self="$$" '
    $1 != self &&
    ($0 ~ /tools\/sim_test\/run_sim_test.py run/ ||
     $0 ~ /c1f6_system_test\.launch\.py/) { print }
  '
} || true)"

if [[ -n "$LIVE_PRIMARY" ]]; then
  echo "ERROR: a supervised M1/C1F.6 run still appears to be alive." >&2
  echo "$LIVE_PRIMARY" >&2
  echo "Refusing automatic cleanup. Stop that run normally first." >&2
  exit 2
fi

if [[ -f "$ACTIVE_CAPTURE" ]]; then
  echo "Finalizing abandoned M1 experiment capture..."
  if ! python3 "$REPO/tools/c1f_experiment/experiment_capture.py" stop; then
    STAMP="$(date +%Y%m%d_%H%M%S)"
    BACKUP="$REPO/logs/c1f_experiments/.active_run.stale_${STAMP}.json"
    echo "WARNING: stale capture could not be finalized cleanly." >&2
    echo "Preserving its marker as: $BACKUP" >&2
    mv "$ACTIVE_CAPTURE" "$BACKUP"
  fi
fi

# These are children that can survive an interrupted/failed C1F.6 launch after
# the top-level runner is gone. Patterns are kept M1-specific where practical.
patterns=(
  "world_drone_env_detach.sdf"
  "$REPO/install/drone_visualisation/share/drone_visualisation/rviz/default.rviz"
  "$REPO/install/tejen_mission/lib/tejen_mission/online_join_planner"
  "$REPO/install/tejen_mission/lib/tejen_mission/simulation_xy_disturbance_gate"
  "$REPO/install/tejen_mission/lib/tejen_mission/simulation_test_supervisor"
  "$REPO/install/tejen_mission/lib/tejen_mission/magnet_attachment_manager"
  "$REPO/install/tejen_mission/lib/tejen_mission/pendulum_state_publisher"
  "$REPO/install/tejen_mission/lib/tejen_mission/fake_cooperative_transport_world"
  "$REPO/install/tejen_dynamic_planner/lib/tejen_dynamic_planner/dynamic_planner_transfer_backend"
  "$REPO/install/tejen_mpc/lib/tejen_mpc/main"
  "$REPO/install/simulation_communication/lib/simulation_communication/motion_capture_emulator"
  "$REPO/install/simulation_communication/lib/simulation_communication/betaflight_communication"
  "$REPO/install/simulation_communication/lib/simulation_communication/pendulum_state_listener"
)

found_stale=0
for pattern in "${patterns[@]}"; do
  if pgrep -u "$(id -u)" -f "$pattern" >/dev/null 2>&1; then
    found_stale=1
    break
  fi
done

if (( found_stale )); then
  echo "Clearing stale M1/C1F.6 child processes..."
  for sig in INT TERM KILL; do
    for pattern in "${patterns[@]}"; do
      pkill -"$sig" -u "$(id -u)" -f "$pattern" 2>/dev/null || true
    done
    [[ "$sig" == "INT" ]] && sleep 2 || true
    [[ "$sig" == "TERM" ]] && sleep 1 || true
  done
fi

# Refresh discovery after an interrupted graph. This does not alter packages,
# configuration, credentials or system services.
ros2 daemon stop >/dev/null 2>&1 || true
sleep 0.3
ros2 daemon start >/dev/null 2>&1 || true

if [[ -f "$ACTIVE_CAPTURE" ]]; then
  echo "ERROR: active capture marker survived cleanup: $ACTIVE_CAPTURE" >&2
  exit 1
fi

echo "M1/C1F.6 stale-session cleanup complete."

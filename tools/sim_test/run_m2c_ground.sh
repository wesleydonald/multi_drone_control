#!/usr/bin/env bash
# Supervised M2C ground-only four-X3 communication/assignment commissioning.
# Flight sequencing deliberately belongs to M2D. This script automates the same
# stage commands that can still be run manually in separate terminals.
set -eo pipefail

REPO="${M2C_REPO:-$HOME/Documents/Thesis git/drone_cage_control}"
GUI="${1:-true}"
M2C_KEEP_OPEN="${M2C_KEEP_OPEN:-$GUI}"
M2C_PERF_DWELL_S="${M2C_PERF_DWELL_S:-4}"
# multi_drone_control: his bridges' rate loop on the X3 gyro (default since 2026-09-28) or
# differenced poses (M2_RATE_SOURCE=pose keeps the world without the Imu system)
export M2_RATE_SOURCE="${M2_RATE_SOURCE:-imu}"
case "$M2_RATE_SOURCE" in
  imu) IMU_SYSTEM_ARG="--imu-system" ;;
  pose) IMU_SYSTEM_ARG="--no-imu-system" ;;
  *) echo "ERROR: M2_RATE_SOURCE must be pose or imu"; exit 2 ;;
esac
cd "$REPO"
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "Clearing stale thesis simulation / ROS processes before M2C..."
bash "$REPO/tools/sim_test/cleanup_m2b_stale.sh"

STAMP="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="$REPO/logs/m2_attachment/m2c_ground_${STAMP}"
GENERATED_DIR="$LOG_DIR/generated"
CONTROLLER_WORK_ROOT="$LOG_DIR/controller_work"
ACADOS_CACHE_ROOT="$REPO/build/acados_cache/payload_mpc"
MANIFEST="$LOG_DIR/m2c_ground_manifest.json"
WORLD="$GENERATED_DIR/m2c_four_x3_ground.sdf"
RUNTIME="$LOG_DIR/runtime.log"
RESULT="$LOG_DIR/m2c_result.txt"
PERF_DIR="$LOG_DIR/performance"
PERF_STAGE_LOG="$PERF_DIR/stages.tsv"
PERF_RTF_LOG="$PERF_DIR/gazebo_world_stats.csv"
PERF_SUMMARY="$PERF_DIR/summary.txt"
PERF_HOST_DIR="$PERF_DIR/host_snapshots"
ACTIVE_PGID_FILE="$REPO/logs/m2_attachment/.m2c_active_pgid"
mkdir -p "$GENERATED_DIR" "$CONTROLLER_WORK_ROOT" "$ACADOS_CACHE_ROOT" "$PERF_HOST_DIR"
printf 'wall_epoch_s\twall_iso\tstage\tdetail\n' > "$PERF_STAGE_LOG"
for i in 0 1 2 3; do
  mkdir -p "$CONTROLLER_WORK_ROOT/drone_${i}"
done

MANAGED_PGIDS=()
LAST_MANAGED_PGID=""
BASE_LAUNCH_PGID=""

record_pgids() {
  if ((${#MANAGED_PGIDS[@]})); then
    printf '%s\n' "${MANAGED_PGIDS[@]}" > "$ACTIVE_PGID_FILE"
  else
    rm -f "$ACTIVE_PGID_FILE"
  fi
}

start_managed() {
  setsid "$@" >>"$RUNTIME" 2>&1 &
  LAST_MANAGED_PGID=$!
  MANAGED_PGIDS+=("$LAST_MANAGED_PGID")
  record_pgids
}

mark_stage() {
  local stage="$1" detail="${2:-}" epoch iso
  epoch="$(date +%s.%N)"
  iso="$(date --iso-8601=ns)"
  detail="${detail//$'\t'/ }"
  detail="${detail//$'\n'/ }"
  printf '%s\t%s\t%s\t%s\n' "$epoch" "$iso" "$stage" "$detail" >> "$PERF_STAGE_LOG"
  printf 'M2C_PERF_STAGE wall_epoch_s=%s stage=%s detail=%s\n' "$epoch" "$stage" "$detail" >> "$RUNTIME"
}

capture_host_snapshot() {
  local label="$1" path="$PERF_HOST_DIR/${label}.txt"
  {
    printf 'wall_iso=%s\n' "$(date --iso-8601=ns)"
    printf 'wall_epoch_s=%s\n' "$(date +%s.%N)"
    printf 'loadavg='
    cat /proc/loadavg 2>/dev/null || true
    printf '\n--- memory ---\n'
    free -h 2>/dev/null || true
    printf '\n--- process snapshot ---\n'
    ps -eo pid,ppid,pgid,pcpu,pmem,etime,comm,args --sort=-pcpu 2>/dev/null || true
  } > "$path" 2>&1 || true
}

write_performance_summary() {
  python3 "$REPO/tools/sim_test/summarize_m2c_performance.py" \
    --stages "$PERF_STAGE_LOG" --rtf "$PERF_RTF_LOG" --output "$PERF_SUMMARY" \
    >> "$RUNTIME" 2>&1 || true
}

performance_window() {
  local label="$1"
  mark_stage "rtf_window_${label}_start" "dwell_s=$M2C_PERF_DWELL_S"
  capture_host_snapshot "${label}_start"
  if [[ "$M2C_PERF_DWELL_S" != "0" && "$M2C_PERF_DWELL_S" != "0.0" ]]; then
    sleep "$M2C_PERF_DWELL_S"
  fi
  capture_host_snapshot "${label}_end"
  mark_stage "rtf_window_${label}_end" "dwell_s=$M2C_PERF_DWELL_S"
}

stop_group() {
  local pgid="$1"
  [[ "$pgid" =~ ^[0-9]+$ ]] || return 0
  kill -0 -- -"$pgid" 2>/dev/null || return 0
  kill -INT -- -"$pgid" 2>/dev/null || true
  for _ in $(seq 1 60); do kill -0 -- -"$pgid" 2>/dev/null || break; sleep 0.10; done
  kill -0 -- -"$pgid" 2>/dev/null || return 0
  kill -TERM -- -"$pgid" 2>/dev/null || true
  sleep 1
  kill -0 -- -"$pgid" 2>/dev/null || return 0
  kill -KILL -- -"$pgid" 2>/dev/null || true
}

cleanup() {
  trap - EXIT INT TERM
  local idx
  for ((idx=${#MANAGED_PGIDS[@]}-1; idx>=0; idx--)); do
    stop_group "${MANAGED_PGIDS[$idx]}"
  done
  for pgid in "${MANAGED_PGIDS[@]}"; do
    wait "$pgid" 2>/dev/null || true
  done
  rm -f "$ACTIVE_PGID_FILE"
}
trap cleanup EXIT INT TERM

snapshot_failure_state() {
  # Preserve current fleet-manager truth before cleanup tears the ROS graph down.
  # These snapshots are diagnostic only and never affect commissioning authority.
  timeout 2 ros2 topic echo --full-length --once /m2c/status std_msgs/msg/String \
    > "$LOG_DIR/m2c_status_failure.txt" 2>&1 || true
  timeout 2 ros2 topic echo --qos-durability transient_local --once \
    /m2c/assignment std_msgs/msg/String \
    > "$LOG_DIR/m2c_assignment_failure.txt" 2>&1 || true
}

fail() {
  local reason="$1"
  mark_stage "fail" "$reason"
  capture_host_snapshot "failure"
  write_performance_summary
  snapshot_failure_state
  printf 'FAIL\nstage=M2C\nreason=%s\nlog_dir=%s\n' "$reason" "$LOG_DIR" > "$RESULT"
  echo "ERROR: $reason"
  exit 1
}

wait_for_node() {
  local node="$1" timeout_s="$2" started="$(date +%s)" nodes=""
  while true; do
    nodes="$(ros2 node list 2>/dev/null || true)"
    grep -Fqx "$node" <<<"$nodes" && return 0
    (( $(date +%s) - started < timeout_s )) || return 1
    sleep 0.25
  done
}

wait_for_service() {
  local service="$1" timeout_s="$2" started="$(date +%s)" services=""
  while true; do
    services="$(ros2 service list 2>/dev/null || true)"
    grep -Fqx "$service" <<<"$services" && return 0
    (( $(date +%s) - started < timeout_s )) || return 1
    sleep 0.25
  done
}

wait_for_subscriber() {
  local topic="$1" timeout_s="$2" started="$(date +%s)" info=""
  while true; do
    info="$(ros2 topic info "$topic" 2>/dev/null || true)"
    if grep -Eq 'Subscription count:[[:space:]]+[1-9][0-9]*' <<<"$info"; then
      return 0
    fi
    if (( $(date +%s) - started >= timeout_s )); then
      echo "ERROR: timeout waiting for ROS subscription on $topic"
      printf '%s\n' "$info"
      return 1
    fi
    sleep 0.25
  done
}

# Buffer one complete ROS message before inspecting it. Do not stream a live
# `ros2 topic echo` process directly into `grep -q` under pipefail: grep may find
# an early match and close the pipe while ros2 still has YAML to print, turning a
# successful observation into a non-zero pipeline status.
topic_once_output() {
  local timeout_s="$1"
  shift
  timeout "$timeout_s" ros2 topic echo --once "$@" 2>/dev/null
}

# /m2c/status is a JSON string longer than Humble ros2cli's default 128-character
# string rendering limit. Always request the complete string before inspecting
# state fields such as disarmed_controller_count. Keep this separate from the
# generic topic helper so large trajectory messages are not expanded needlessly.
m2c_status_once_output() {
  local timeout_s="$1"
  timeout "$timeout_s" ros2 topic echo --full-length --once \
    /m2c/status std_msgs/msg/String 2>/dev/null
}

# Wait for the simulation clock using wall-time supervision. This must succeed
# before Gazebo is intentionally paused, because sim-time ROS timers stop while
# /clock is frozen.
wait_for_sim_clock() {
  local timeout_s="$1" started="$(date +%s)" output=""
  while true; do
    if output="$(topic_once_output 2 /clock rosgraph_msgs/msg/Clock)"; then
      [[ "$output" == *"clock:"* ]] && return 0
    fi
    (( $(date +%s) - started < timeout_s )) || return 1
    sleep 0.10
  done
}

# After the raw release loop has already observed ROS detached=true, confirm that
# the long-lived fleet manager received the same truth. This avoids a redundant
# second volatile ros2cli subscriber whose discovery timing can false-fail even
# though the release and persistent consumer both succeeded.
wait_for_joint_detached_count() {
  local expected="$1" timeout_s="$2" started="$(date +%s)" status=""
  while true; do
    if status="$(m2c_status_once_output 2)"; then
      [[ "$status" == *'"joint_detached_count": '"${expected}"* ]] && return 0
    fi
    (( $(date +%s) - started < timeout_s )) || return 1
    sleep 0.10
  done
}

# Gazebo Transport reports generic CLI echo subscribers as
# `google.protobuf.Message` on Harmonic, while typed subscribers may appear as
# `gz.msgs.*`. Discovery only needs to prove that a real transport endpoint is
# present under the Subscribers section; the advertised subscriber type is not an
# authority signal and must not be over-constrained.
gz_topic_subscriber_line() {
  local info="$1"
  awk '
    /^Subscribers[[:space:]]*\[/ { in_subscribers=1; next }
    in_subscribers && /^[[:space:]]+[^[:space:],]+:\/\/[^,]+,[[:space:]]*[^[:space:]]+/ {
      sub(/^[[:space:]]+/, "")
      print
      exit
    }
  ' <<<"$info"
}

# The raw DetachableJoint output is a transition stream, not a latched current
# state. Before release, prove the gz transport echo process inside our truth bridge
# has completed discovery so the one successful DETACHED transition cannot be lost.
wait_for_gz_subscriber() {
  local topic="$1" timeout_s="$2" started="$(date +%s)" info="" subscriber_line=""
  while true; do
    info="$(gz topic -i -t "$topic" 2>/dev/null || true)"
    subscriber_line="$(gz_topic_subscriber_line "$info" || true)"
    if [[ -n "$subscriber_line" ]]; then
      echo "GZ_SUBSCRIBER_DISCOVERED topic=$topic ${subscriber_line#${subscriber_line%%[![:space:]]*}}" >>"$RUNTIME"
      return 0
    fi
    if (( $(date +%s) - started >= timeout_s )); then
      echo "ERROR: timeout waiting for Gazebo subscriber on $topic"
      printf '%s\n' "$info"
      return 1
    fi
    sleep 0.25
  done
}

world_step_once() {
  gz service -s /world/quadcopter/control \
    --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean \
    --timeout 1000 --req 'multi_step: 1' >>"$RUNTIME" 2>&1
}

release_joint_until_detached() {
  local drone_id="$1" timeout_s="$2" started="$(date +%s)"
  local detach_topic="/drone_${drone_id}/magnet/detach" output=""
  while (( $(date +%s) - started < timeout_s )); do
    if output="$(topic_once_output 2 "/drone_${drone_id}/magnet/joint_detached_truth" std_msgs/msg/Bool)"; then
      [[ "$output" == *"data: true"* ]] && return 0
    fi
    # DetachableJoint ignores detach while its initial joint does not yet exist.
    # Repeating the request while single-stepping is therefore safe and removes
    # dependence on observing the earlier one-shot ATTACHED transition.
    timeout 3 ros2 topic pub --once "$detach_topic" std_msgs/msg/Empty '{}' >>"$RUNTIME" 2>&1 || true
    world_step_once || return 1
    sleep 0.10
  done
  return 1
}

wait_for_controller_ready() {
  local drone_id="$1" timeout_s="$2" started="$(date +%s)"
  local topic="/drone_${drone_id}/tejen_mpc/c1d_status" output=""
  while true; do
    if output="$(topic_once_output 3 "$topic" std_msgs/msg/String)"; then
      [[ "$output" == *"R6.3C.1d MPC INTEGRATION"* ]] && return 0
    fi
    (( $(date +%s) - started < timeout_s )) || return 1
    sleep 0.20
  done
}

wait_for_telemetry() {
  local drone_id="$1" timeout_s="$2" started="$(date +%s)"
  local topic="/drone_${drone_id}/telemetry" output=""
  while true; do
    if output="$(topic_once_output 2 "$topic" interfaces/msg/Telemetry)"; then
      [[ "$output" == *"battery_voltage:"* ]] && return 0
    fi
    (( $(date +%s) - started < timeout_s )) || return 1
    sleep 0.20
  done
}

wait_for_reference() {
  local drone_id="$1" timeout_s="$2" started="$(date +%s)"
  local topic="/drone_${drone_id}/join_planner/reference" output=""
  while true; do
    if output="$(topic_once_output 2 "$topic" trajectory_msgs/msg/MultiDOFJointTrajectory)"; then
      [[ "$output" == *"drone_${drone_id}"* ]] && return 0
    fi
    (( $(date +%s) - started < timeout_s )) || return 1
    sleep 0.20
  done
}

wait_for_assignment() {
  local timeout_s="$1" started="$(date +%s)" status=""
  while true; do
    # /m2c/status is continuously published current state. Do not make
    # commissioning depend on consuming the one-time assignment publication.
    if status="$(m2c_status_once_output 3)"; then
      if [[ "$status" == *'"assignment_frozen": true'* &&
            "$status" == *'"candidate_count": 72'* ]]; then
        return 0
      fi
    fi
    if (( $(date +%s) - started >= timeout_s )); then
      echo "ERROR: timeout waiting for fleet manager to report frozen 72-case assignment"
      [[ -n "$status" ]] && printf 'Last /m2c/status:\n%s\n' "$status"
      return 1
    fi
    sleep 0.20
  done
}

wait_for_disarmed_count() {
  local expected="$1" timeout_s="$2" started="$(date +%s)" status=""
  while true; do
    if status="$(m2c_status_once_output 3)"; then
      [[ "$status" == *"\"disarmed_controller_count\": ${expected}"* ]] && return 0
    fi
    (( $(date +%s) - started < timeout_s )) || return 1
    sleep 0.20
  done
}

wait_ready() {
  local timeout_s="$1" started="$(date +%s)" output=""
  while true; do
    if output="$(topic_once_output 2 /m2c/ready std_msgs/msg/Bool)"; then
      [[ "$output" == *"data: true"* ]] && return 0
    fi
    (( $(date +%s) - started < timeout_s )) || return 1
    sleep 0.20
  done
}

echo "M2C four-X3 ground commissioning"
echo "Repository: $REPO"
echo "Evidence:   $LOG_DIR"
echo "GUI:        $GUI"
echo "ACADOS cache: $ACADOS_CACHE_ROOT"
echo "Perf dwell: ${M2C_PERF_DWELL_S}s per stable RTF window (diagnostic only)"
mark_stage "runner_ready" "gui=$GUI perf_dwell_s=$M2C_PERF_DWELL_S"
capture_host_snapshot "runner_ready"
if [[ "$GUI" == "true" ]]; then
  echo "RViz:       deferred until all four controllers pass the coexistence gate"
fi

python3 -m tejen_mission.m2c_ground_spawn generate \
  --source-x3 "$REPO/simulation_assets/tejen/modelLargeM2BallMagnet.sdf" \
  --source-ring "$REPO/simulation_assets/tejen/m2a_ring_fixture.sdf" \
  --world-template "$REPO/simulation_assets/tejen/world_m2b_single_attachment.sdf" \
  --output-dir "$GENERATED_DIR" --manifest "$MANIFEST" --ring-yaw-deg 20.0 "$IMU_SYSTEM_ARG" \
  >"$RUNTIME" 2>&1
mark_stage "assets_generated" "world=$WORLD"

export GZ_SIM_RESOURCE_PATH="$GENERATED_DIR:$REPO/simulation_assets${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"

# Base launch only: Gazebo, bridges, four platform stacks, four truth bridges,
# four stationary planners and the fleet manager. Controllers remain manual-sized
# stages and are started below only after the ground bootstrap is complete.
start_managed ros2 launch tejen_mission m2c_four_drone_ground.launch.py \
  gui:="$GUI" world_path:="$WORLD"
BASE_LAUNCH_PGID="$LAST_MANAGED_PGID"
mark_stage "base_launch_started" "pgid=$BASE_LAUNCH_PGID"

wait_for_node /m2c_joint_command_bridge 15 || fail "joint_command_bridge_missing"
wait_for_node /m2c_fleet_manager 20 || fail "fleet_manager_missing"
for i in 0 1 2 3; do
  wait_for_node "/drone_${i}/motion_capture_emulator" 20 || fail "drone_${i}_mocap_node_missing"
  wait_for_node "/drone_${i}/online_join_planner" 20 || fail "drone_${i}_mission_node_missing"
  wait_for_node "/drone_${i}/m2c_joint_truth_bridge" 20 || fail "drone_${i}_joint_truth_node_missing"
  wait_for_subscriber "/drone_${i}/magnet/detach" 12 || fail "drone_${i}_detach_bridge_missing"
done
wait_for_node /m2c_clock_bridge 15 || fail "clock_bridge_missing"
wait_for_sim_clock 15 || fail "simulation_clock_missing"
echo "ROS clock:    SIMULATION (/clock ready)"
mark_stage "sim_clock_ready"
mark_stage "base_plumbing_ready"
capture_host_snapshot "base_plumbing_ready"

# Keep the world paused while the artificial startup fixtures are released.
gz service -s /world/quadcopter/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean \
  --timeout 3000 --req 'pause: true' >>"$RUNTIME" 2>&1 || fail "cannot_pause_world"
echo "Gazebo world: PAUSED for supervised startup release"
mark_stage "world_paused"

# Positive discovery gate for the raw one-shot DETACHED event. The truth bridge's
# internal `gz topic -e` subscriber must exist before any release can succeed.
for i in 0 1 2 3; do
  wait_for_gz_subscriber "/drone_${i}/magnet/joint_detached_truth" 15 \
    || fail "drone_${i}_raw_truth_listener_not_discovered"
  echo "Raw joint-truth listener drone_${i}: DISCOVERED"
done
mark_stage "raw_truth_listeners_ready"

# Release one physical identity at a time. A request sent before DetachableJoint
# has created its initial joint is simply retried while the paused world advances.
for i in 0 1 2 3; do
  release_joint_until_detached "$i" 20 || fail "drone_${i}_raw_detached_truth_missing"
  mark_stage "startup_release_drone_${i}_verified"
done
echo "Four startup releases: SENT + VERIFIED DETACHED"
mark_stage "startup_releases_complete"

gz service -s /world/quadcopter/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean \
  --timeout 3000 --req 'pause: false' >>"$RUNTIME" 2>&1 || fail "cannot_resume_world"
echo "Gazebo world: RUNNING after four verified releases"
mark_stage "world_running"
# Fleet-manager readiness/freshness uses simulation time in P3, so its timer is
# intentionally frozen during the paused bootstrap. Confirm the persistent 4/4
# detached authority immediately after /clock resumes.
wait_for_joint_detached_count 4 10 || fail "fleet_detached_truth_missing_after_unpause"
echo "Fleet detached truth: 4/4 confirmed after unpause"
mark_stage "fleet_detached_truth_ready"
# Gazebo Sim 8 publishes native WorldStatistics at 5 Hz. Capture that native
# stream directly for RTF measurement; it remains independent of the ROS /clock
# bridge used for P3 physical-time semantics. The observer is cleaned up normally.
start_managed python3 "$REPO/tools/sim_test/capture_gz_world_stats.py" \
  --topic /world/quadcopter/stats --output "$PERF_RTF_LOG"
mark_stage "rtf_capture_started" "file=$PERF_RTF_LOG"

# The fleet manager requires one continuous second of the existing grounded,
# low-speed and magnet-floor conditions after all four fresh DETACHED truths.
# Only then may the immutable 72-case assignment freeze.
wait_for_assignment 20 || fail "fleet_assignment_never_froze_after_ground_settle"
echo "Fleet ground settle + frozen assignment: READY"
mark_stage "assignment_ready"
performance_window "base_no_controllers"

# Heavy ACADOS constructors are genuinely serialized by completion. Each command
# below is also valid as an independent manual terminal launch.
for i in 0 1 2 3; do
  echo "Starting controller drone_${i}..."
  mark_stage "controller_${i}_start"
  capture_host_snapshot "controller_${i}_start"
  start_managed ros2 launch tejen_mission m2c_vehicle_controller.launch.py \
    drone_id:="$i" controller_work_dir:="$CONTROLLER_WORK_ROOT/drone_${i}" \
    acados_cache_dir:="$ACADOS_CACHE_ROOT"
  wait_for_node "/drone_${i}/tejen_mpc" 30 || fail "drone_${i}_controller_node_missing"
  wait_for_service "/drone_${i}/arming_service" 30 || fail "drone_${i}_controller_service_missing"
  wait_for_node "/drone_${i}/betaflight_communication" 10 || fail "drone_${i}_betaflight_node_missing"
  wait_for_subscriber "/drone_${i}/join_planner/reference" 10 || fail "drone_${i}_reference_not_wired_to_controller"
  wait_for_subscriber "/drone_${i}/ELRSCommand" 10 || fail "drone_${i}_command_not_wired_to_betaflight"
  wait_for_controller_ready "$i" 120 || fail "drone_${i}_controller_never_completed_acados_startup"
  wait_for_disarmed_count "$((i + 1))" 10 || fail "drone_${i}_initial_disarmed_feedback_missing"
  wait_for_reference "$i" 15 || fail "drone_${i}_ground_reference_not_observed"
  wait_for_telemetry "$i" 10 || fail "drone_${i}_telemetry_not_observed"
  mark_stage "controller_${i}_ready"
  capture_host_snapshot "controller_${i}_ready"
  performance_window "after_controller_${i}"
done

echo "Four controller/platform routes: READY"
mark_stage "four_controller_routes_ready"

# Final four-controller coexistence gate. Earlier controllers must still be live
# after the later ACADOS constructors finish; cached fleet state alone is not enough.
for i in 0 1 2 3; do
  wait_for_node "/drone_${i}/tejen_mpc" 5 || fail "drone_${i}_controller_lost_before_pass"
  wait_for_service "/drone_${i}/arming_service" 5 || fail "drone_${i}_arming_service_lost_before_pass"
  wait_for_subscriber "/drone_${i}/join_planner/reference" 5 || fail "drone_${i}_reference_route_lost_before_pass"
  wait_for_subscriber "/drone_${i}/ELRSCommand" 5 || fail "drone_${i}_command_route_lost_before_pass"
  wait_for_controller_ready "$i" 5 || fail "drone_${i}_heartbeat_lost_before_pass"
done
wait_for_disarmed_count 4 5 || fail "fleet_not_disarmed_before_pass"
mark_stage "controller_coexistence_ready"
capture_host_snapshot "controller_coexistence_ready"

if [[ "$GUI" == "true" ]]; then
  mark_stage "rviz_start"
  start_managed ros2 launch drone_visualisation view_frame.launch.py use_sim_time:=true
  wait_for_node /rviz2 15 || fail "rviz_never_started"
  mark_stage "rviz_ready"
  performance_window "after_rviz"
fi

wait_ready 15 || fail "fleet_never_reached_ground_ready"
mark_stage "fleet_ready"
timeout 3 ros2 topic echo --full-length --once /m2c/status std_msgs/msg/String > "$LOG_DIR/m2c_status.txt" 2>&1 || true
timeout 3 ros2 topic echo --qos-durability transient_local --once /m2c/assignment std_msgs/msg/String > "$LOG_DIR/m2c_assignment.txt" 2>&1 || true
ros2 node list > "$LOG_DIR/nodes.txt" 2>&1 || true
ros2 topic list > "$LOG_DIR/topics.txt" 2>&1 || true

cat > "$RESULT" <<EOF
PASS
stage=M2C
ground_only=true
ring_yaw_deg=20.0
candidate_assignments=72
vehicle_count=4
raw_detached_count=4
ground_settle_dwell_s=1.0
disarmed_controller_count=4
controller_startup=sequential_health_gated
p3_sim_time=true
ros_clock=simulation
p3_clock_policy=all_sim_ros
performance_instrumentation=true
performance_dwell_s=$M2C_PERF_DWELL_S
performance_stage_log=$PERF_STAGE_LOG
performance_rtf_log=$PERF_RTF_LOG
performance_summary=$PERF_SUMMARY
log_dir=$LOG_DIR
EOF
mark_stage "pass"
capture_host_snapshot "pass"
write_performance_summary

echo "M2C PASS: four real X3 stacks, settled frozen assignment, stationary commitments, disarmed controllers, and final coexistence are coherent."
echo "Result: $RESULT"
if [[ "$M2C_KEEP_OPEN" == "true" ]]; then
  echo "M2C commissioning is complete. Gazebo/RViz will remain open for inspection; press Ctrl-C here when finished."
  while [[ -n "$BASE_LAUNCH_PGID" ]] && kill -0 -- -"$BASE_LAUNCH_PGID" 2>/dev/null; do
    sleep 1
  done
fi

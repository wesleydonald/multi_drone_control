#!/usr/bin/env bash
# Supervised M2B B1/B2 single-drone attachment commissioning run.
# B1 modes: capture, latch, success. B2 wrapper uses success after C++ transit.
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$SCRIPT_DIR/../.." && pwd)"
STAMP="$(date +%Y%m%d_%H%M%S)"
COMMISSIONING_STAGE="${M2B_COMMISSIONING_STAGE:-b1}"
B2_CPP_TRANSIT_ENABLED="${M2B_B2_CPP_TRANSIT_ENABLED:-false}"
B2_STATIC_OBSTACLE_ENABLED="${M2B_B2_STATIC_OBSTACLE_ENABLED:-false}"
if [[ "$COMMISSIONING_STAGE" == "b2" ]]; then
  LOG_DIR="$REPO/logs/m2_attachment/m2b_b2_${STAMP}"
else
  LOG_DIR="$REPO/logs/m2_attachment/m2b_b1_${STAMP}"
fi
MODE="${1:-capture}"
GUI="${2:-true}"
ASSIGNED_PLATE_ID="${M2B_ASSIGNED_PLATE_ID:-0}"
CONTACT_OBSERVATION_MODEL="${M2B_CONTACT_OBSERVATION_MODEL:-sphere_center}"
MAGNET_SPHERE_RADIUS_M="${M2B_MAGNET_SPHERE_RADIUS_M:-0.025}"
MAGNET_CAPTURE_GAP_M="${M2B_MAGNET_CAPTURE_GAP_M:-0.010}"
ACTIVE_PGID_FILE="$REPO/logs/m2_attachment/.m2b_b1_active_pgid"
LOCK_FILE="$REPO/logs/m2_attachment/.m2b.lock"
STATUS_CSV="$LOG_DIR/b1_status.csv"
HOLD_SECONDS=5
LATCH_STABILITY_OBSERVE_SECONDS=3
mkdir -p "$LOG_DIR" "$REPO/logs/m2_attachment"

if [[ "$COMMISSIONING_STAGE" != "b1" && "$COMMISSIONING_STAGE" != "b2" ]]; then
  echo "ERROR: M2B_COMMISSIONING_STAGE must be b1 or b2."
  exit 2
fi
if [[ "$COMMISSIONING_STAGE" == "b2" && "$B2_CPP_TRANSIT_ENABLED" != "true" ]]; then
  echo "ERROR: B2 requires M2B_B2_CPP_TRANSIT_ENABLED=true."
  exit 2
fi
if [[ "$MODE" != "capture" && "$MODE" != "latch" && "$MODE" != "success" ]]; then
  echo "ERROR: first argument must be one of capture|latch|success."
  echo "Usage: bash tools/sim_test/run_m2b_b1.sh capture true"
  echo "   or: bash tools/sim_test/run_m2b_b1.sh latch true"
  echo "   or: bash tools/sim_test/run_m2b_b1.sh success true"
  exit 2
fi
ATTACHMENT_ENABLED=true
PROOF_ENABLED=true
MAX_ATTEMPTS=3
[[ "$MODE" == "capture" ]] && ATTACHMENT_ENABLED=false
[[ "$MODE" != "success" ]] && PROOF_ENABLED=false
[[ "$MODE" == "latch" ]] && MAX_ATTEMPTS=1

cd "$REPO"
source /opt/ros/humble/setup.bash
source install/setup.bash
export GZ_SIM_RESOURCE_PATH="$REPO/simulation_assets${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"

WORLD_TEMPLATE="$REPO/simulation_assets/tejen/world_m2b_single_attachment.sdf"
GENERATED_DIR="$LOG_DIR/generated"
GENERATED_WORLD="$GENERATED_DIR/m2b_generated_world.sdf"
GROUND_MANIFEST="$LOG_DIR/ground_start.json"
GROUND_MEASURED="$LOG_DIR/ground_start_measured.json"
POST_DETACH_GROUND="$LOG_DIR/post_detach_ground_settle.json"

if [[ ! -f "$WORLD_TEMPLATE" ]] || ! grep -q 'modelLargeM2BallMagnet.sdf' "$WORLD_TEMPLATE"; then
  echo "ERROR: missing/invalid M2B real-X3 world template."
  exit 1
fi
if grep -qi 'm2a_x3_support\|kinematic' "$WORLD_TEMPLATE"; then
  echo "ERROR: abandoned M2A support fixture detected in M2B world."
  exit 1
fi

exec 9>"$LOCK_FILE"
if ! flock -n 9; then
  echo "ERROR: another M2B runner currently owns $LOCK_FILE"
  exit 1
fi

assert_clean_gazebo() {
  local stale="$(pgrep -af 'gz sim' 2>/dev/null || true)"
  if [[ -n "$stale" ]]; then
    echo "ERROR: an existing Gazebo Sim process is already alive."
    printf '%s\n' "$stale"
    echo "If it is stale, run: bash tools/sim_test/cleanup_m2b_stale.sh"
    return 1
  fi
}
assert_clean_ros_graph() {
  local nodes="$(ros2 node list 2>/dev/null || true)"
  local stale=""
  for node in \
    /online_join_planner /tejen_mpc /dynamic_planner_transfer_backend \
    /m2b_b2_stationary_ring_commitment /m2b_gazebo_joint_truth_bridge \
    /m2b_physical_capture_manager /m2b_attachment_observer /m2b_b1_telemetry \
    /m2b_joint_command_bridge /motion_capture_emulator /betaflight_communication \
    /pendulum_state_publisher /rviz2; do
    if printf '%s\n' "$nodes" | grep -qx "$node"; then stale="$stale $node"; fi
  done
  if [[ -n "$stale" ]]; then
    echo "ERROR: stale simulation ROS nodes are already alive:$stale"
    echo "If they are stale, run: bash tools/sim_test/cleanup_m2b_stale.sh"
    return 1
  fi
}
if [[ -f "$ACTIVE_PGID_FILE" ]]; then
  OLD_PGID="$(cat "$ACTIVE_PGID_FILE" 2>/dev/null || true)"
  if [[ "$OLD_PGID" =~ ^[0-9]+$ ]] && kill -0 -- -"$OLD_PGID" 2>/dev/null; then
    echo "ERROR: previous M2B B1 process group still alive (PGID $OLD_PGID)."
    echo "Run: bash tools/sim_test/cleanup_m2b_stale.sh"
    exit 1
  fi
  rm -f "$ACTIVE_PGID_FILE"
fi
assert_clean_gazebo
assert_clean_ros_graph

LAUNCH_PID=""; LAUNCH_PGID=""
cleanup_launch_group() {
  set +e
  if [[ -n "$LAUNCH_PGID" ]] && kill -0 -- -"$LAUNCH_PGID" 2>/dev/null; then
    kill -INT -- -"$LAUNCH_PGID" 2>/dev/null || true
    for _ in $(seq 1 70); do kill -0 -- -"$LAUNCH_PGID" 2>/dev/null || break; sleep 0.10; done
  fi
  if [[ -n "$LAUNCH_PGID" ]] && kill -0 -- -"$LAUNCH_PGID" 2>/dev/null; then
    kill -TERM -- -"$LAUNCH_PGID" 2>/dev/null || true
    for _ in $(seq 1 30); do kill -0 -- -"$LAUNCH_PGID" 2>/dev/null || break; sleep 0.10; done
  fi
  if [[ -n "$LAUNCH_PGID" ]] && kill -0 -- -"$LAUNCH_PGID" 2>/dev/null; then kill -KILL -- -"$LAUNCH_PGID" 2>/dev/null || true; fi
  [[ -n "$LAUNCH_PID" ]] && wait "$LAUNCH_PID" 2>/dev/null || true
  rm -f "$ACTIVE_PGID_FILE"
}
interrupt_cleanup() { trap - EXIT INT TERM; cleanup_launch_group; exit 130; }
trap cleanup_launch_group EXIT
trap interrupt_cleanup INT TERM

world_step_once() {
  gz service -s /world/quadcopter/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean \
    --timeout 1000 --req 'multi_step: 1' >>"$LOG_DIR/runtime.log" 2>&1
}
wait_for_service() {
  local service="$1" timeout_s="$2" started="$(date +%s)"
  while true; do
    ros2 service list 2>/dev/null | grep -qx "$service" && return 0
    if (( $(date +%s) - started >= timeout_s )); then echo "ERROR: timeout waiting for service $service"; return 1; fi
    sleep 0.25
  done
}
wait_for_node() {
  local node="$1" timeout_s="$2" started="$(date +%s)"
  while true; do
    ros2 node list 2>/dev/null | grep -qx "$node" && return 0
    if (( $(date +%s) - started >= timeout_s )); then echo "ERROR: timeout waiting for node $node"; return 1; fi
    sleep 0.20
  done
}
wait_for_topic_subscriber() {
  local topic="$1" timeout_s="$2" started="$(date +%s)" info=""
  while true; do
    info="$(ros2 topic info "$topic" 2>/dev/null || true)"
    if printf '%s\n' "$info" | grep -Eq 'Subscription count:[[:space:]]+[1-9][0-9]*'; then return 0; fi
    if (( $(date +%s) - started >= timeout_s )); then
      echo "ERROR: timeout waiting for ROS subscriber on $topic"
      printf '%s\n' "$info"
      return 1
    fi
    sleep 0.20
  done
}
csv_latest_value_is() {
  local column="$1" expected="$2"
  python3 - "$STATUS_CSV" "$column" "$expected" <<'PY'
import csv, sys
from pathlib import Path
path=Path(sys.argv[1]); column=sys.argv[2]; expected=sys.argv[3].strip().lower(); latest=""
if not path.is_file(): raise SystemExit(1)
try:
    with path.open(newline='') as f:
        for row in csv.DictReader(f):
            value=str(row.get(column,'')).strip()
            if value: latest=value.lower()
except (OSError,csv.Error): raise SystemExit(1)
raise SystemExit(0 if latest==expected else 1)
PY
}
wait_for_csv_value() {
  local column="$1" expected="$2" timeout_s="$3" started="$(date +%s)"
  while true; do
    csv_latest_value_is "$column" "$expected" && return 0
    if (( $(date +%s) - started >= timeout_s )); then
      echo "ERROR: timeout waiting for $column=$expected in $STATUS_CSV"; return 1
    fi
    sleep 0.10
  done
}

record_runtime_failure() {
  local reason="$1"
  cat >"$LOG_DIR/b1_result.txt" <<EOF
FAIL
stage=M2B_${COMMISSIONING_STAGE^^}
assigned_plate_id=$ASSIGNED_PLATE_ID
reason=$reason
structured_telemetry=b1_status.csv
attachment_detector_log=attachment.csv
EOF
}

runtime_health_check() {
  if [[ -n "$LAUNCH_PID" ]] && ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
    echo "ERROR: M2B launch process exited unexpectedly during flight."
    record_runtime_failure "launch_process_exited"
    return 1
  fi
  if [[ "$COMMISSIONING_STAGE" == "b2" ]] && ! ros2 node list 2>/dev/null | grep -qx /dynamic_planner_transfer_backend; then
    echo "ERROR: B2 dynamic planner backend disappeared during commissioning."
    record_runtime_failure "dynamic_planner_backend_missing"
    return 1
  fi
  if csv_latest_value_is external_reference_fault_latched true; then
    echo "ERROR: controller latched an external-reference fault. Stopping B1 immediately."
    record_runtime_failure "external_reference_fault_latched"
    return 1
  fi
  if csv_latest_value_is pendulum_fault_latched true; then
    echo "ERROR: controller latched a pendulum-state fault. Stopping B1 immediately."
    record_runtime_failure "pendulum_fault_latched"
    return 1
  fi
  if csv_latest_value_is mission_phase M2_FAULT; then
    echo "ERROR: mission entered M2_FAULT. Stopping B1 immediately."
    record_runtime_failure "mission_entered_M2_FAULT"
    return 1
  fi
  if [[ "$MODE" == "success" ]] && {
    csv_latest_value_is mission_phase M2_LANDING_STAGE \
      || csv_latest_value_is mission_phase LANDING \
      || csv_latest_value_is mission_phase LANDED_DISARMED;
  }; then
    echo "ERROR: B1 attachment commissioning exhausted/rejected the attempt and entered the landing path."
    record_runtime_failure "attachment_failed_entered_landing_path"
    return 1
  fi
  if [[ "$MODE" == "latch" ]] && {
    csv_latest_value_is mission_phase M2_LANDING_STAGE \
      || csv_latest_value_is mission_phase LANDING \
      || csv_latest_value_is mission_phase LANDED_DISARMED;
  }; then
    echo "ERROR: B1 latch commissioning rejected the attempt and entered the landing path."
    record_runtime_failure "latch_failed_entered_landing_path"
    return 1
  fi
  return 0
}

wait_for_csv_value_guarded() {
  local column="$1" expected="$2" timeout_s="$3" started="$(date +%s)"
  while true; do
    csv_latest_value_is "$column" "$expected" && return 0
    runtime_health_check || return 1
    if (( $(date +%s) - started >= timeout_s )); then
      echo "ERROR: timeout waiting for $column=$expected in $STATUS_CSV"
      record_runtime_failure "timeout_${column}_${expected}"
      return 1
    fi
    sleep 0.10
  done
}

# Operator-owned gates intentionally have no wall-clock timeout. The supervised
# runner continues health monitoring while the user inspects the vehicle and
# explicitly commands ARM / TAKEOFF from RViz.
wait_for_csv_value_guarded_operator() {
  local column="$1" expected="$2"
  while true; do
    csv_latest_value_is "$column" "$expected" && return 0
    runtime_health_check || return 1
    sleep 0.10
  done
}
check_capture_settled() {
  python3 - "$STATUS_CSV" "$ASSIGNED_PLATE_ID" <<'PY'
import csv, math, sys
from pathlib import Path
p=Path(sys.argv[1]); plate=int(sys.argv[2]); latest=None
with p.open(newline='') as f:
    for row in csv.DictReader(f): latest=row
if latest is None: raise SystemExit(1)
angle=2*math.pi*plate/12.0
target=(0.25*math.cos(angle),0.25*math.sin(angle),0.035+0.30+0.475+0.040)
pos=tuple(float(latest[f'drone_{a}']) for a in 'xyz')
vel=tuple(float(latest[f'drone_v{a}']) for a in 'xyz')
phi=float(latest['pendulum_phi']); theta=float(latest['pendulum_theta'])
err=math.sqrt(sum((a-b)**2 for a,b in zip(pos,target)))
speed=math.sqrt(sum(v*v for v in vel)); swing=math.degrees(math.hypot(phi,theta))
print(f"capture measured: position_error={err:.4f} m speed={speed:.4f} m/s swing={swing:.2f} deg")
raise SystemExit(0 if err<=0.05 and speed<=0.03 and swing<=3.0 else 1)
PY
}

echo "M2B $COMMISSIONING_STAGE - single-drone attachment commissioning"
echo "Repository: $REPO"
echo "Evidence:   $LOG_DIR"
echo "Mode:       $MODE"
echo "Stage:      $COMMISSIONING_STAGE"
if [[ "$COMMISSIONING_STAGE" == "b2" ]]; then
  echo "B2 C++:     enabled=$B2_CPP_TRANSIT_ENABLED static_obstacle=$B2_STATIC_OBSTACLE_ENABLED"
  echo "B2 logs:    $LOG_DIR/backend.csv and $LOG_DIR/replans.csv"
fi
echo "Plate:      $ASSIGNED_PLATE_ID"
echo "Contact:    $CONTACT_OBSERVATION_MODEL radius=$MAGNET_SPHERE_RADIUS_M m capture_gap=$MAGNET_CAPTURE_GAP_M m"
echo "Proof:      enabled=$PROOF_ENABLED (latch mode verifies weld stability before proof)"
echo "Telemetry:  $STATUS_CSV"

mkdir -p "$GENERATED_DIR"
python3 -m tejen_mission.m2b_ground_spawn generate \
  --source-x3 "$REPO/simulation_assets/tejen/modelLargeM2BallMagnet.sdf" \
  --world-template "$WORLD_TEMPLATE" --output-dir "$GENERATED_DIR" --manifest "$GROUND_MANIFEST" \
  --assigned-plate-id "$ASSIGNED_PLATE_ID" \
  --simulation-attachment-stabilization --attachment-joint-damping 0.1 \
  >"$LOG_DIR/runtime.log" 2>&1
export GZ_SIM_RESOURCE_PATH="$GENERATED_DIR:$REPO/simulation_assets${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"

setsid ros2 launch tejen_mission m2b_b1_single_attachment.launch.py \
  gui:="$GUI" assigned_plate_id:="$ASSIGNED_PLATE_ID" m2b_log_dir:="$LOG_DIR" \
  commissioning_stage:="$COMMISSIONING_STAGE" \
  b2_cpp_transit_enabled:="$B2_CPP_TRANSIT_ENABLED" \
  b2_static_obstacle_enabled:="$B2_STATIC_OBSTACLE_ENABLED" \
  world_path:="$GENERATED_WORLD" attachment_enabled:="$ATTACHMENT_ENABLED" proof_enabled:="$PROOF_ENABLED" max_attempts:="$MAX_ATTEMPTS" \
  contact_observation_model:="$CONTACT_OBSERVATION_MODEL" \
  magnet_sphere_radius_m:="$MAGNET_SPHERE_RADIUS_M" magnet_capture_gap_m:="$MAGNET_CAPTURE_GAP_M" \
  >>"$LOG_DIR/runtime.log" 2>&1 &
LAUNCH_PID=$!; LAUNCH_PGID=$LAUNCH_PID; echo "$LAUNCH_PGID" >"$ACTIVE_PGID_FILE"

wait_for_node /m2b_b1_telemetry 12
wait_for_node /m2b_attachment_observer 12
wait_for_node /m2b_joint_command_bridge 12
wait_for_node /m2b_gazebo_joint_truth_bridge 12
wait_for_topic_subscriber /m2b/gz/attach 8
wait_for_topic_subscriber /m2b/gz/detach 8
if [[ "$COMMISSIONING_STAGE" == "b2" ]]; then
  wait_for_node /dynamic_planner_transfer_backend 15
  wait_for_node /m2b_b2_stationary_ring_commitment 15
fi

# B1 intentionally keeps the startup DetachableJoint closed until this measured
# geometry check has passed. This preserves the commissioned near-horizontal
# ground pose even though hard magnet/ring collision is filtered in the run-local
# B1 assets. No raw detach request is permitted before the supervised release.
for _ in $(seq 1 20); do world_step_once; sleep 0.03; done
python3 -m tejen_mission.m2b_ground_spawn check \
  --status-csv "$STATUS_CSV" --manifest "$GROUND_MANIFEST" --output-json "$GROUND_MEASURED" --max-age-s 2.0 \
  >>"$LOG_DIR/runtime.log" 2>&1 || { echo "ERROR: measured ground geometry gate failed."; exit 1; }
echo "Measured ground geometry: VERIFIED"

wait_for_topic_subscriber /m2b/bootstrap/release 8
ros2 topic pub --once /m2b/bootstrap/release std_msgs/msg/Bool '{data: true}' \
  >>"$LOG_DIR/runtime.log" 2>&1
echo "Supervised bootstrap release: SENT"

DETACHED=false
for _ in $(seq 1 100); do
  world_step_once || true
  if csv_latest_value_is joint_detached true; then DETACHED=true; break; fi
  sleep 0.05
done
[[ "$DETACHED" == true ]] || { echo "ERROR: bootstrap detach not verified."; exit 1; }
echo "Bootstrap raw joint truth: DETACHED"

# Let the disarmed X3 remain on the ground while the released magnet falls the
# small startup gap onto the real floor. Selective masks allow magnet-floor
# contact while suppressing hard magnet/ring contact. Do not arm until measured
# telemetry proves both bodies are quiet and the magnet is floor-supported.
gz service -s /world/quadcopter/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean \
  --timeout 3000 --req 'pause: false' >>"$LOG_DIR/runtime.log" 2>&1
POST_DETACH_SETTLED=false
POST_DETACH_GOOD_COUNT=0
POST_DETACH_REQUIRED_GOOD=5
for _ in $(seq 1 80); do
  if python3 -m tejen_mission.m2b_ground_spawn check-post-detach-floor \
      --status-csv "$STATUS_CSV" --manifest "$GROUND_MANIFEST" \
      --output-json "$POST_DETACH_GROUND" --max-age-s 1.0 \
      --magnet-radius-m "$MAGNET_SPHERE_RADIUS_M" \
      >>"$LOG_DIR/runtime.log" 2>&1; then
    POST_DETACH_GOOD_COUNT=$((POST_DETACH_GOOD_COUNT + 1))
    if (( POST_DETACH_GOOD_COUNT >= POST_DETACH_REQUIRED_GOOD )); then
      POST_DETACH_SETTLED=true
      break
    fi
  else
    POST_DETACH_GOOD_COUNT=0
  fi
  sleep 0.10
done
[[ "$POST_DETACH_SETTLED" == true ]] || {
  echo "ERROR: detached magnet did not become measured-settled on the floor before arm."
  record_runtime_failure "post_detach_floor_settle_failed"
  exit 1
}
echo "Post-detach floor settle: VERIFIED"

wait_for_csv_value arm_permission true 10
wait_for_service /drone_arming_service 25

echo "OPERATOR GATE: click ARM in RViz when ready. The B1 runner will not arm the vehicle automatically."
wait_for_csv_value_guarded_operator armed true
echo "Operator ARM observed."

echo "OPERATOR GATE: click TAKEOFF in RViz when ready. The B1 runner will not publish TAKEOFF automatically."
wait_for_csv_value_guarded_operator mission_phase M2_VERTICAL_TAKEOFF
if [[ "$COMMISSIONING_STAGE" == "b2" ]]; then
  echo "Operator TAKEOFF observed. Waiting for stationary handoff -> C++ capture transit..."
  wait_for_csv_value_guarded mission_phase M2_CPP_TRANSIT_CAPTURE 60
  echo "B2 C++ authority active. Waiting for checked C++ -> Python bridge into the B1 capture phase..."
  wait_for_csv_value_guarded mission_phase M2_SETTLE_CAPTURE 60
else
  echo "Operator TAKEOFF observed. Waiting for measured post-takeoff settling and local capture transfer..."
  wait_for_csv_value_guarded mission_phase M2_SETTLE_CAPTURE 45
fi

if [[ "$MODE" == "capture" ]]; then
  echo "Capture-only mode: attachment descent is disabled. Waiting for the measured capture hover to settle..."
  PASS=false
  for _ in $(seq 1 200); do
    runtime_health_check || exit 1
    if check_capture_settled >>"$LOG_DIR/runtime.log" 2>&1; then PASS=true; break; fi
    sleep 0.10
  done
  [[ "$PASS" == true ]] || { echo "ERROR: capture hover did not meet the measured settle gate."; exit 1; }
  check_capture_settled
  csv_latest_value_is magnet_command off || { echo "ERROR: magnet must remain OFF in capture-only mode."; exit 1; }
  cat >"$LOG_DIR/b1_result.txt" <<EOF
CAPTURE_PASS
stage=M2B_B1_CAPTURE_SANITY
assigned_plate_id=$ASSIGNED_PLATE_ID
attachment_enabled=false
structured_telemetry=b1_status.csv
attachment_detector_log=attachment.csv
EOF
  echo "CAPTURE_PASS: capture hover is measured-settled; magnet remained OFF; no attachment attempted."
  echo "Inspect the geometry, then Ctrl-C. Run success mode only if this looks sane."
  echo "Evidence directory: $LOG_DIR"
  while true; do sleep 1; done
fi

if [[ "$MODE" == "latch" ]]; then
  echo "Latch-only mode: descending and physically attaching, but proof motion is disabled."
  wait_for_csv_value_guarded joint_detached false 90
  wait_for_csv_value_guarded latch_stable true 5
  csv_latest_value_is mission_phase M2_ATTACH_CAPTURE_WAIT || {
    echo "ERROR: latch-only mode left M2_ATTACH_CAPTURE_WAIT unexpectedly."; exit 1;
  }
  csv_latest_value_is proof_requested false || {
    echo "ERROR: proof must remain disabled during latch-only commissioning."; exit 1;
  }
  csv_latest_value_is detector_confirmed false || {
    echo "ERROR: detector must not confirm without proof excitation."; exit 1;
  }
  echo "Latch measured-stable. Observing frozen weld for ${LATCH_STABILITY_OBSERVE_SECONDS} s before PASS..."
  latch_observation_started_ns="$(date +%s%N)"
  latch_observation_duration_ns=$((LATCH_STABILITY_OBSERVE_SECONDS * 1000000000))
  while (( $(date +%s%N) - latch_observation_started_ns < latch_observation_duration_ns )); do
    runtime_health_check || exit 1
    csv_latest_value_is joint_detached false || {
      echo "ERROR: physical joint separated during latch-only stability observation."
      record_runtime_failure "latch_separated_during_stability_observation"
      exit 1
    }
    csv_latest_value_is latch_stable true || {
      echo "ERROR: measured latch stability was lost during latch-only observation."
      record_runtime_failure "latch_stability_lost_during_observation"
      exit 1
    }
    csv_latest_value_is proof_requested false || {
      echo "ERROR: proof was requested during latch-only commissioning."
      record_runtime_failure "proof_requested_in_latch_only_mode"
      exit 1
    }
    csv_latest_value_is detector_confirmed false || {
      echo "ERROR: detector confirmed without proof during latch-only commissioning."
      record_runtime_failure "detector_confirmed_without_proof"
      exit 1
    }
    sleep 0.10
  done
  cat >"$LOG_DIR/b1_result.txt" <<EOF
LATCH_PASS
stage=M2B_B1_LATCH_SANITY
assigned_plate_id=$ASSIGNED_PLATE_ID
attachment_enabled=true
proof_enabled=false
physical_joint_latched=true
post_latch_measured_stability=true
software_detector_confirmed=false
structured_telemetry=b1_status.csv
attachment_detector_log=attachment.csv
EOF
  echo "LATCH_PASS: physical weld is latched and measured-stable; proof remained disabled."
  echo "Inspect the attached geometry, then Ctrl-C. Run success mode only if this stays sane."
  echo "Evidence directory: $LOG_DIR"
  while true; do sleep 1; done
fi

echo "Full M2B $COMMISSIONING_STAGE enabled. M2_ATTACH_PROOF will run only after the post-latch stability gate; waiting for software-confirmed attached hold..."
wait_for_csv_value_guarded mission_phase M2_ATTACHED_HOLD 90
wait_for_csv_value_guarded detector_confirmed true 5
wait_for_csv_value_guarded joint_detached false 5
wait_for_csv_value_guarded effective_mpc_mode ATTACHED_HOLD 5
echo "Attached hold reached. Observing for ${HOLD_SECONDS} s..."
hold_observation_started_ns="$(date +%s%N)"
hold_observation_duration_ns=$((HOLD_SECONDS * 1000000000))
while (( $(date +%s%N) - hold_observation_started_ns < hold_observation_duration_ns )); do
  runtime_health_check || exit 1
  sleep 0.10
done

if ! csv_latest_value_is mission_phase M2_ATTACHED_HOLD \
  || ! csv_latest_value_is detector_confirmed true \
  || ! csv_latest_value_is joint_detached false \
  || ! csv_latest_value_is external_reference_fault_latched false \
  || ! csv_latest_value_is pendulum_fault_latched false; then
  echo "ERROR: B1 attached-hold acceptance gate failed."
  echo "Inspect $STATUS_CSV, attachment.csv and runtime.log"
  exit 1
fi

cat >"$LOG_DIR/b1_result.txt" <<EOF
PASS
stage=M2B_${COMMISSIONING_STAGE^^}
assigned_plate_id=$ASSIGNED_PLATE_ID
attachment_enabled=true
software_detector_confirmed=true
physical_joint_latched=true
attached_hold_observation_s=$HOLD_SECONDS
structured_telemetry=b1_status.csv
attachment_detector_log=attachment.csv
EOF

echo "PASS: M2B $COMMISSIONING_STAGE reached software-confirmed attached hold for ${HOLD_SECONDS} s."
echo "Evidence directory: $LOG_DIR"
echo "Gazebo remains open for inspection. Press Ctrl-C here when finished."
while true; do sleep 1; done

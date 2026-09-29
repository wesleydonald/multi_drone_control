#!/usr/bin/env bash
# Supervised M2D sequential four-drone attachment commissioning.
# One implementation supports three validation goals:
#   M2D_GOAL_ATTACHMENTS=0  grounded four-stack integration only
#   M2D_GOAL_ATTACHMENTS=1  drone_0 attachment in the full four-stack topology
#   M2D_GOAL_ATTACHMENTS=2  two-drone mission target, useful for IRL-like commissioning
#   M2D_GOAL_ATTACHMENTS=4  full manual 0 -> 1 -> 2 -> 3 sequence
# Set M2D_OPERATOR_HOLD=true to remain in monitored success hold and land the
# proven-attached fleet only after an explicit RViz/operator LAND request.
# ARM / TAKEOFF remains explicit operator authority through RViz or the printed CLI fallback.
set -eo pipefail

REPO="${M2D_REPO:-$HOME/Documents/Thesis git/drone_cage_control}"
GUI="${1:-true}"
GOAL_ATTACHMENTS="${M2D_GOAL_ATTACHMENTS:-4}"
case "$GOAL_ATTACHMENTS" in
  0|1|2|4) ;;
  *) echo "ERROR: M2D_GOAL_ATTACHMENTS must be 0, 1, 2, or 4"; exit 2 ;;
esac
OPERATOR_HOLD="${M2D_OPERATOR_HOLD:-false}"
SIMULTANEOUS="${M2D_SIMULTANEOUS:-false}"   # multi_drone_control: all four at once (testing)
# multi_drone_control handover: dynamic 0.86 kg ring, his MPC ELRS via our per-drone
# muxes (ELRSCommand_tejen), and the success hold kept so our stack can take over
HANDOVER="${M2D_HANDOVER:-false}"
ELRS_OUTPUT="ELRSCommand"
if [[ "$HANDOVER" == "true" ]]; then
  OPERATOR_HOLD=true
  ELRS_OUTPUT="ELRSCommand_tejen"
fi
case "$OPERATOR_HOLD" in
  true|false) ;;
  *) echo "ERROR: M2D_OPERATOR_HOLD must be true or false"; exit 2 ;;
esac
# GOAL=1 remains the historical partial-validation run: its fleet supervisor must
# still advance to ACTIVE_1 after drone_0 attaches. A real mission target of two
# attachments is therefore represented explicitly by GOAL=2.
if [[ "$GOAL_ATTACHMENTS" == "0" || "$GOAL_ATTACHMENTS" == "1" ]]; then
  TARGET_ATTACHMENTS=4
else
  TARGET_ATTACHMENTS="$GOAL_ATTACHMENTS"
fi
SPAWN_SCENARIO="${M2D_SPAWN_SCENARIO:-nominal}"
case "$SPAWN_SCENARIO" in
  nominal|hard_line) ;;
  *) echo "ERROR: M2D_SPAWN_SCENARIO must be nominal or hard_line"; exit 2 ;;
esac
PREAUTH_OCTOPUS_MAX_RUNTIME_S="${M2D_PREAUTH_OCTOPUS_MAX_RUNTIME_S:-0.10}"
if [[ ! "$PREAUTH_OCTOPUS_MAX_RUNTIME_S" =~ ^([0-9]+([.][0-9]*)?|[.][0-9]+)$ ]]; then
  echo "ERROR: M2D_PREAUTH_OCTOPUS_MAX_RUNTIME_S must be a positive number"
  exit 2
fi
if ! awk -v value="$PREAUTH_OCTOPUS_MAX_RUNTIME_S" 'BEGIN { exit !(value > 0.0) }'; then
  echo "ERROR: M2D_PREAUTH_OCTOPUS_MAX_RUNTIME_S must be > 0"
  exit 2
fi
ACTIVE_OCTOPUS_MAX_RUNTIME_S="0.10"
# multi_drone_control: his bridges' rate loop on the X3 gyro (default since 2026-09-28) or
# differenced poses; imu adds the Imu system to the generated world and the launch bridges
# the gyros, pose keeps the old world (M2_RATE_SOURCE=pose)
RATE_SOURCE="${M2_RATE_SOURCE:-imu}"
case "$RATE_SOURCE" in
  pose|imu) ;;
  *) echo "ERROR: M2_RATE_SOURCE must be pose or imu"; exit 2 ;;
esac
export M2_RATE_SOURCE="$RATE_SOURCE"
IMU_SYSTEM_ARGS=(--no-imu-system)
if [[ "$RATE_SOURCE" == "imu" ]]; then
  IMU_SYSTEM_ARGS=(--imu-system)
fi

cd "$REPO"
if [[ -f "$HOME/ros2_humble/install/setup.bash" ]]; then source "$HOME/ros2_humble/install/setup.bash"; else source /opt/ros/humble/setup.bash; fi
source install/setup.bash

printf 'Clearing stale thesis simulation / ROS processes before M2D...\n'
bash "$REPO/tools/sim_test/cleanup_m2b_stale.sh"

STAMP="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="$REPO/logs/m2_attachment/m2d_sequential_${STAMP}"
GENERATED_DIR="$LOG_DIR/generated"
CONTROLLER_WORK_ROOT="$LOG_DIR/controller_work"
ACADOS_CACHE_ROOT="$REPO/build/acados_cache/payload_mpc"
MANIFEST="$LOG_DIR/m2d_manifest.json"
WORLD="$GENERATED_DIR/m2c_four_x3_ground.sdf"
RUNTIME="$LOG_DIR/runtime.log"
RESULT="$LOG_DIR/m2d_result.txt"
OBSERVER_JSON="$LOG_DIR/commissioning_status.json"
OBSERVER_ENV="$LOG_DIR/commissioning_status.env"
ACTIVE_PGID_FILE="$REPO/logs/m2_attachment/.m2d_active_pgid"
mkdir -p "$GENERATED_DIR" "$CONTROLLER_WORK_ROOT" "$ACADOS_CACHE_ROOT"
for i in 0 1 2 3; do
  mkdir -p "$CONTROLLER_WORK_ROOT/drone_${i}"
done

MANAGED_PGIDS=()
LAST_MANAGED_PGID=""
BASE_LAUNCH_PGID=""
OBSERVER_PGID=""

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
  local idx pgid
  for ((idx=${#MANAGED_PGIDS[@]}-1; idx>=0; idx--)); do
    stop_group "${MANAGED_PGIDS[$idx]}"
  done
  for pgid in "${MANAGED_PGIDS[@]}"; do
    wait "$pgid" 2>/dev/null || true
  done
  rm -f "$ACTIVE_PGID_FILE"
}
trap cleanup EXIT INT TERM

snapshot_state() {
  [[ -f "$OBSERVER_JSON" ]] && cp "$OBSERVER_JSON" "$LOG_DIR/commissioning_status_at_result.json" || true
  [[ -f "$OBSERVER_ENV" ]] && cp "$OBSERVER_ENV" "$LOG_DIR/commissioning_status_at_result.env" || true
}

load_observer_snapshot() {
  [[ -s "$OBSERVER_ENV" ]] || return 1
  # Generated by m2d_commissioning_observer.py from fixed scalar keys.
  # shellcheck disable=SC1090
  source "$OBSERVER_ENV"
  [[ "${observer_ready:-false}" == "true" ]] || return 1
  [[ "${observer_wall_unix_s:-}" =~ ^[0-9]+$ ]] || return 1
  local now
  now="$(date +%s)"
  (( now - observer_wall_unix_s <= 3 )) || return 1
}

wait_for_observer_ready() {
  local timeout_s="$1" started="$(date +%s)"
  while true; do
    load_observer_snapshot && return 0
    # grace: right after `setsid ... &` the new process group may not exist yet (a race that
    # failed two starts on 2026-09-26, T0011/T0013 first attempts)
    if (( $(date +%s) - started >= 5 )); then
      [[ -z "$OBSERVER_PGID" ]] || kill -0 -- -"$OBSERVER_PGID" 2>/dev/null || return 2
    fi
    (( $(date +%s) - started < timeout_s )) || return 1
    sleep 0.20
  done
}

wait_for_bool_key() {
  local key="$1" wanted="$2" timeout_s="$3" started="$(date +%s)" value=""
  while true; do
    if load_observer_snapshot; then
      value="${!key:-}"
      [[ "$value" == "$wanted" ]] && return 0
    elif [[ -n "$OBSERVER_PGID" ]] && ! kill -0 -- -"$OBSERVER_PGID" 2>/dev/null; then
      return 2
    fi
    (( $(date +%s) - started < timeout_s )) || return 1
    sleep 0.20
  done
}

wait_for_int_eq() {
  local key="$1" wanted="$2" timeout_s="$3" started="$(date +%s)" value=""
  while true; do
    if load_observer_snapshot; then
      value="${!key:-}"
      [[ "$value" =~ ^-?[0-9]+$ ]] && (( value == wanted )) && return 0
    elif [[ -n "$OBSERVER_PGID" ]] && ! kill -0 -- -"$OBSERVER_PGID" 2>/dev/null; then
      return 2
    fi
    (( $(date +%s) - started < timeout_s )) || return 1
    sleep 0.20
  done
}

wait_for_int_gt_value() {
  local key="$1" baseline="$2" timeout_s="$3" started="$(date +%s)" value=""
  [[ "$baseline" =~ ^[0-9]+$ ]] || return 1
  while true; do
    if load_observer_snapshot; then
      value="${!key:-}"
      [[ "$value" =~ ^[0-9]+$ ]] && (( value > baseline )) && return 0
    elif [[ -n "$OBSERVER_PGID" ]] && ! kill -0 -- -"$OBSERVER_PGID" 2>/dev/null; then
      return 2
    fi
    (( $(date +%s) - started < timeout_s )) || return 1
    sleep 0.20
  done
}

print_commissioning_summary() {
  if ! load_observer_snapshot; then
    echo "observer: STALE_OR_UNAVAILABLE"
    return 0
  fi
  echo "observer: READY age_wall_s=$(( $(date +%s) - observer_wall_unix_s ))"
  echo "clock_seen=${clock_seen:-false} m2c_state=${m2c_state:-unknown} m2c_ready=${m2c_ready:-false} m2c_ready_latched=${m2c_ready_latched:-false} m2d_state=${m2d_state:-unknown} abort=${m2d_abort_reason:-none}"
  echo "m2c: detached=${m2c_joint_detached_count:-?} assignment=${m2c_assignment_frozen:-?} candidates=${m2c_candidate_count:-?} disarmed=${m2c_disarmed_controller_count:-?}"
  local i key joint phase backend_status armed takeoff
  for i in 0 1 2 3; do
    key="drone_${i}_joint_detached"; joint="${!key:-false}"
    key="drone_${i}_phase"; phase="${!key:-unknown}"
    key="drone_${i}_backend_status_seen"; backend_status="${!key:-false}"
    key="drone_${i}_armed"; armed="${!key:-false}"
    key="drone_${i}_takeoff"; takeoff="${!key:-false}"
    echo "drone_${i}: detached=$joint backend_status=$backend_status phase=$phase armed=$armed takeoff=$takeoff"
  done
}

fail() {
  local reason="$1"
  snapshot_state
  {
    echo "FAIL"
    echo "stage=M2D"
    echo "reason=$reason"
    echo "goal_attachments=$GOAL_ATTACHMENTS"
    echo "spawn_scenario=$SPAWN_SCENARIO"
    echo "octopus_pre_authority_max_runtime_s=$PREAUTH_OCTOPUS_MAX_RUNTIME_S"
    echo "octopus_active_max_runtime_s=$ACTIVE_OCTOPUS_MAX_RUNTIME_S"
    echo "manual_arm_takeoff=true"
    echo "target_attachments=$TARGET_ATTACHMENTS"
    echo "operator_hold_after_goal=$OPERATOR_HOLD"
    echo "m2d_sim_time=true"
    echo "ros_clock=simulation"
    echo "m2d_clock_policy=all_sim_ros"
    echo "log_dir=$LOG_DIR"
    echo "commissioning_summary_begin"
    print_commissioning_summary
    echo "commissioning_summary_end"
  } > "$RESULT"
  echo "ERROR: $reason"
  print_commissioning_summary
  echo "Evidence: $LOG_DIR"
  exit 1
}

world_step_once() {
  gz service -s /world/quadcopter/control \
    --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean \
    --timeout 1000 --req 'multi_step: 1' >>"$RUNTIME" 2>&1
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
# Do NOT require an initial ROS `false` sample: Gazebo may emit no state message
# until the first actual attach/detach transition.
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
      echo "ERROR: timeout waiting for Gazebo subscriber on $topic" >>"$RUNTIME"
      printf '%s\n' "$info" >>"$RUNTIME"
      return 1
    fi
    sleep 0.25
  done
}

release_joint_until_detached() {
  local drone_id="$1" timeout_s="$2" started="$(date +%s)"
  local key="drone_${drone_id}_joint_detached"
  local detach_topic="/drone_${drone_id}/magnet/detach" step_rc=0
  while true; do
    # Consume the latest acknowledgement before enforcing the deadline. A detach
    # request issued just before timeout may be processed by the final world step;
    # the old while-condition could then reject a transition already observed true.
    if load_observer_snapshot && [[ "${!key:-false}" == "true" ]]; then
      return 0
    fi
    if [[ -n "$OBSERVER_PGID" ]] && ! kill -0 -- -"$OBSERVER_PGID" 2>/dev/null; then
      return 2
    fi
    if (( $(date +%s) - started >= timeout_s )); then
      return 1
    fi

    timeout 5 ros2 topic pub --once "$detach_topic" std_msgs/msg/Empty '{}' >>"$RUNTIME" 2>&1 || true
    step_rc=0
    world_step_once || step_rc=$?
    sleep 0.15

    # Prefer observed detached truth over a transport/service return code if Gazebo
    # processed the command before that return code was reported.
    if load_observer_snapshot && [[ "${!key:-false}" == "true" ]]; then
      return 0
    fi
    (( step_rc == 0 )) || return 3
  done
}

wait_for_m2d_state() {
  local wanted="$1" timeout_s="$2" started="$(date +%s)"
  while true; do
    if load_observer_snapshot; then
      [[ "${m2d_state:-}" == "M2D_ABORT" ]] && return 2
      [[ "${m2d_state:-}" == "$wanted" ]] && return 0
    elif [[ -n "$OBSERVER_PGID" ]] && ! kill -0 -- -"$OBSERVER_PGID" 2>/dev/null; then
      return 3
    fi
    (( $(date +%s) - started < timeout_s )) || return 1
    sleep 0.20
  done
}

release_bootstrap_until_armable() {
  local drone_id="$1" timeout_s="$2" started="$(date +%s)"
  local phase_key="drone_${drone_id}_phase"
  local release_topic="/drone_${drone_id}/m2b/bootstrap/release" phase=""
  while true; do
    # Check acknowledgement before timeout so a release delivered on the final
    # allowed attempt cannot be rejected before its phase update is consumed.
    if load_observer_snapshot; then
      phase="${!phase_key:-}"
      [[ "$phase" == "M2_WAIT_FOR_ARM" ]] && return 0
      [[ "$phase" == "M2_FAULT" ]] && return 2
    elif [[ -n "$OBSERVER_PGID" ]] && ! kill -0 -- -"$OBSERVER_PGID" 2>/dev/null; then
      return 3
    fi

    (( $(date +%s) - started < timeout_s )) || return 1

    timeout 5 ros2 topic pub --once "$release_topic" \
      std_msgs/msg/Bool '{data: true}' >>"$RUNTIME" 2>&1 || true
    sleep 0.35
  done
}

wait_for_armed_feedback() {
  local drone_id="$1" timeout_s="$2"
  wait_for_bool_key "drone_${drone_id}_armed" true "$timeout_s"
}

planner_phase_is_after_takeoff() {
  local phase="$1"
  case "$phase" in
    *M2_VERTICAL_TAKEOFF*|*M2_TAKEOFF_HOVER*|*M2_CPP_TRANSIT_CAPTURE*|*M2_SETTLE_CAPTURE*|*M2_ATTACH_APPROACH_HIGH*|*M2_ATTACH_APPROACH_LOW*|*M2_ATTACH_CAPTURE_WAIT*|*M2_ATTACH_PROOF*|*M2_ATTACHED_HOLD*|*M2_DETACHED_RETREAT*|*M2_LANDING_STAGE*)
      return 0
      ;;
  esac
  return 1
}

wait_for_takeoff_acknowledgement() {
  local drone_id="$1" timeout_s="$2" started="$(date +%s)"
  local takeoff_key="drone_${drone_id}_takeoff" phase_key="drone_${drone_id}_phase"
  local controller_takeoff_seen=false planner_takeoff_seen=false phase=""

  while true; do
    if load_observer_snapshot; then
      [[ "${!takeoff_key:-false}" == "true" ]] && controller_takeoff_seen=true
      phase="${!phase_key:-}"
      [[ "$phase" == "M2_FAULT" ]] && return 2
      planner_phase_is_after_takeoff "$phase" && planner_takeoff_seen=true
    elif [[ -n "$OBSERVER_PGID" ]] && ! kill -0 -- -"$OBSERVER_PGID" 2>/dev/null; then
      return 3
    fi

    if [[ "$controller_takeoff_seen" == "true" && "$planner_takeoff_seen" == "true" ]]; then
      return 0
    fi

    (( $(date +%s) - started < timeout_s )) || {
      echo "drone_${drone_id} TAKEOFF acknowledgement timeout: controller_takeoff=${controller_takeoff_seen} planner_advanced=${planner_takeoff_seen} phase=${phase:-unknown}" >&2
      return 1
    }
    sleep 0.20
  done
}

print_arm_command() {
  local drone_id="$1"
  cat <<EOT

======================================================================
M2D OPERATOR ACTION - drone_${drone_id} is the active vehicle
In RViz, press ARM ACTIVE when ready. CLI fallback:

ros2 topic pub --once -w 2 /drone_${drone_id}/command std_msgs/msg/String '{data: ARM}'

Terminal 1 will wait for periodic controller armed-state feedback before it
permits TAKEOFF. Do NOT arm another drone.
======================================================================
EOT
}

print_takeoff_command() {
  local drone_id="$1"
  cat <<EOT

======================================================================
M2D ARM CONFIRMED - drone_${drone_id}
In RViz, press TAKEOFF ACTIVE when ready. CLI fallback:

ros2 topic pub --once -w 2 /drone_${drone_id}/command std_msgs/msg/String '{data: TAKEOFF}'

Do NOT command another drone until this runner announces the next active vehicle.
======================================================================
EOT
}

manual_activate_vehicle() {
  local drone_id="$1" rc=0
  print_arm_command "$drone_id"
  if wait_for_armed_feedback "$drone_id" 600; then
    echo "drone_${drone_id} armed-state feedback: CONFIRMED TRUE"
  else
    rc=$?
    [[ "$rc" -eq 2 ]] && fail "commissioning_observer_died_during_drone_${drone_id}_arm"
    fail "drone_${drone_id}_manual_arm_feedback_timeout"
  fi
  print_takeoff_command "$drone_id"
  if wait_for_takeoff_acknowledgement "$drone_id" 600; then
    echo "drone_${drone_id} TAKEOFF acknowledgement: CONTROLLER TRUE + PLANNER PHASE ADVANCED"
  else
    rc=$?
    [[ "$rc" -eq 2 ]] && fail "drone_${drone_id}_planner_fault_after_takeoff"
    [[ "$rc" -eq 3 ]] && fail "commissioning_observer_died_during_drone_${drone_id}_takeoff"
    fail "drone_${drone_id}_manual_takeoff_ack_timeout"
  fi
}

write_pass_result() {
  local achieved="$1"
  snapshot_state
  cat > "$RESULT" <<EOR
PASS
stage=M2D
goal_attachments=$GOAL_ATTACHMENTS
achieved_attachments=$achieved
spawn_scenario=$SPAWN_SCENARIO
octopus_pre_authority_max_runtime_s=$PREAUTH_OCTOPUS_MAX_RUNTIME_S
octopus_active_max_runtime_s=$ACTIVE_OCTOPUS_MAX_RUNTIME_S
vehicle_count=4
sequential_order=0,1,2,3
manual_arm_takeoff=true
target_attachments=$TARGET_ATTACHMENTS
operator_hold_after_goal=$OPERATOR_HOLD
frozen_assignment=true
live_ring_pose=true
ring_motion_model=measured_stationary_over_horizon
attached_tether_collision=endpoint_segment_inflated_convex_prism
m2d_sim_time=true
ros_clock=simulation
m2d_clock_policy=all_sim_ros
log_dir=$LOG_DIR
EOR
  echo "M2D validation PASS: achieved_attachments=$achieved goal_attachments=$GOAL_ATTACHMENTS"
  echo "Result: $RESULT"
}

echo "M2D sequential four-drone commissioning"
echo "Repository: $REPO"
echo "Evidence:   $LOG_DIR"
echo "GUI:        $GUI"
echo "Goal:       $GOAL_ATTACHMENTS attachment(s)"
echo "Target:     $TARGET_ATTACHMENTS attachment(s)"
echo "Op hold:    $OPERATOR_HOLD"
echo "Scenario:   $SPAWN_SCENARIO"
echo "Octopus:    pre-authority=${PREAUTH_OCTOPUS_MAX_RUNTIME_S}s active=${ACTIVE_OCTOPUS_MAX_RUNTIME_S}s"
echo "Rate src:   $RATE_SOURCE"
echo "ARM/TAKEOFF: MANUAL operator authority"

python3 -m tejen_mission.m2c_ground_spawn generate \
  --source-x3 "${M2D_SOURCE_X3:-$REPO/simulation_assets/tejen/modelLargeM2BallMagnet.sdf}" \
  --source-ring "$REPO/simulation_assets/tejen/m2a_ring_fixture.sdf" \
  --world-template "$REPO/simulation_assets/tejen/world_m2b_single_attachment.sdf" \
  --output-dir "$GENERATED_DIR" --manifest "$MANIFEST" --ring-yaw-deg 20.0 \
  --stage-layout "$SPAWN_SCENARIO" "${IMU_SYSTEM_ARGS[@]}" \
  >"$RUNTIME" 2>&1 || fail "asset_generation_failed"

if [[ "$HANDOVER" == "true" ]]; then
  # the generated fixture is static; our controller must lift it: 0.86 kg (the real M2A
  # ring) with the inertia of our sim ring (multi_drone_control payload model)
  python3 - "$GENERATED_DIR/m2c_ring_fixture.sdf" <<'PYEOF' >>"$RUNTIME" 2>&1 || fail "ring_dynamic_patch_failed"
import sys, re
p = sys.argv[1]
s = open(p).read()
s, n = re.subn(r'<static>\s*true\s*</static>', '<static>false</static>', s, count=1)
assert n == 1, 'ring <static> not found'
inertial = ('<inertial><mass>0.86</mass><inertia><ixx>2.733e-02</ixx><ixy>0</ixy><ixz>0</ixz>'
            '<iyy>2.733e-02</iyy><iyz>0</iyz><izz>5.452e-02</izz></inertia></inertial>')
s, n = re.subn(r'(<link name="payload_link">)', r'\1' + inertial, s, count=1)
assert n == 1, 'ring payload_link not found'
open(p, 'w').write(s)
print('handover: ring made dynamic (0.86 kg)')
PYEOF
fi

export GZ_SIM_RESOURCE_PATH="$GENERATED_DIR:$REPO/simulation_assets:$REPO/simulation_assets/tejen${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"

# Start one persistent ROS observer before the large graph. All commissioning gates
# below read its local atomic status files rather than repeatedly creating short-lived
# ros2 CLI discovery/subscription nodes.
start_managed python3 "$REPO/tools/sim_test/m2d_commissioning_observer.py" \
  --status-file "$OBSERVER_JSON" --env-file "$OBSERVER_ENV" --period-s 0.50
OBSERVER_PGID="$LAST_MANAGED_PGID"
if wait_for_observer_ready 60; then   # 15 s timed out on a loaded laptop (multi_drone_control 2026-09-26)
  echo "M2D commissioning observer: READY (persistent functional-state subscriptions; graph discovery is non-authoritative)"
else
  fail "commissioning_observer_failed_to_start"
fi

start_managed ros2 launch tejen_mission m2d_four_drone_sequential.launch.py \
  gui:="$GUI" world_path:="$WORLD" m2d_log_dir:="$LOG_DIR" \
  octopus_pre_authority_max_runtime_s:="$PREAUTH_OCTOPUS_MAX_RUNTIME_S" \
  target_attachment_count:="$TARGET_ATTACHMENTS" \
  operator_hold_after_goal:="$OPERATOR_HOLD" simultaneous:="$SIMULTANEOUS"
BASE_LAUNCH_PGID="$LAST_MANAGED_PGID"

# Do not gate on ROS graph discovery. The following stages prove the useful
# end-to-end contracts directly: /clock delivery, raw joint truth, backend runtime
# output, planner phase, controller post-init DISARMED feedback, then M2C/M2D state.
wait_for_bool_key clock_seen true 60 || fail "simulation_clock_missing"
echo "ROS simulation clock: OBSERVED while world is paused"

# Preserve the proven M2C startup-release sequence. The DetachableJoint state topic
# is transition-based, so first prove that each truth bridge's underlying Gazebo
# Transport listener has completed discovery. Requiring an initial ROS `false`
# sample here is invalid because Gazebo may publish nothing until the first state
# transition.
gz service -s /world/quadcopter/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean \
  --timeout 3000 --req 'pause: true' >>"$RUNTIME" 2>&1 || fail "cannot_pause_world"
echo "Gazebo world: PAUSED for supervised startup release"
for i in 0 1 2 3; do
  wait_for_gz_subscriber "/drone_${i}/magnet/joint_detached_truth" 15 \
    || fail "drone_${i}_raw_truth_listener_not_discovered"
  echo "Raw joint-truth listener drone_${i}: DISCOVERED"
done
for i in 0 1 2 3; do
  if release_joint_until_detached "$i" 30; then
    echo "Startup detach drone_${i}: VERIFIED DETACHED"
  else
    rc=$?
    [[ "$rc" -eq 2 ]] && fail "commissioning_observer_died_during_drone_${i}_detach"
    [[ "$rc" -eq 3 ]] && fail "drone_${i}_world_step_failed_during_detach"
    fail "drone_${i}_detach_confirmation_timeout"
  fi
done
echo "Four startup releases: SENT + VERIFIED DETACHED"

load_observer_snapshot || fail "commissioning_observer_stale_before_unpause"
CLOCK_BEFORE_UNPAUSE="${clock_sim_ns:-}"
[[ "$CLOCK_BEFORE_UNPAUSE" =~ ^[0-9]+$ ]] || fail "simulation_clock_value_missing_before_unpause"

gz service -s /world/quadcopter/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean \
  --timeout 3000 --req 'pause: false' >>"$RUNTIME" 2>&1 || fail "cannot_resume_world"
wait_for_int_gt_value clock_sim_ns "$CLOCK_BEFORE_UNPAUSE" 60 || fail "simulation_clock_not_advancing_after_unpause"
echo "ROS simulation time: ADVANCING after unpause"
wait_for_int_eq m2c_joint_detached_count 4 60 || fail "fleet_detached_truth_missing_after_unpause"
echo "Fleet detached truth: 4/4 confirmed after unpause"

# Backend diagnostics are driven by simulation-time timers. They cannot be required
# while Gazebo is paused, so prove runtime activity only after simulation time advances.
for i in 0 1 2 3; do
  wait_for_bool_key "drone_${i}_backend_status_seen" true 60 || fail "drone_${i}_backend_status_not_observed_after_unpause"
done
echo "Four C++ backend runtime-status streams: OBSERVED after unpause"

for i in 0 1 2 3; do
  if release_bootstrap_until_armable "$i" 60; then
    echo "M2B bootstrap drone_${i}: ACKNOWLEDGED + ARMABLE"
  else
    rc=$?
    [[ "$rc" -eq 2 ]] && fail "drone_${i}_bootstrap_fault"
    [[ "$rc" -eq 3 ]] && fail "commissioning_observer_died_during_drone_${i}_bootstrap"
    fail "drone_${i}_bootstrap_never_reached_wait_for_arm"
  fi
done
echo "Four M2B bootstrap releases: ACKNOWLEDGED + ARMABLE"

# Retain the proven serial ACADOS startup. Each vehicle must be fully ready before
# the next constructor starts; this intentionally optimizes determinism, not launch speed.
for i in 0 1 2 3; do
  echo "Starting controller drone_${i}..."
  start_managed ros2 launch tejen_mission m2c_vehicle_controller.launch.py \
    drone_id:="$i" controller_work_dir:="$CONTROLLER_WORK_ROOT/drone_${i}" \
    acados_cache_dir:="$ACADOS_CACHE_ROOT" require_external_arm_permission:=true \
    arming_state_feedback_period_s:=0.5 elrs_output_topic:="$ELRS_OUTPUT"
  # main.py deliberately publishes its initial DISARMED state only after ACADOS
  # construction is genuinely complete. M2C consumes that feedback, so this is
  # the owner-level controller-ready handshake; service/node/diagnostic discovery
  # would only duplicate it and can produce false negatives.
  wait_for_int_eq m2c_disarmed_controller_count "$((i + 1))" 180 \
    || fail "drone_${i}_post_init_disarmed_feedback_missing"
  echo "Controller drone_${i}: POST-INIT DISARMED feedback confirmed by M2C"
done

# ACTIVE_0 is the strongest grounded commissioning acknowledgement: the M2D
# supervisor can only grant it after receiving the frozen assignment and a true
# M2C-ready edge. Do not duplicate that authority with a second observer gate.
if wait_for_m2d_state ACTIVE_0 120; then
  :
else
  rc=$?
  [[ "$rc" -eq 2 ]] && fail "m2d_aborted_before_first_permission"
  fail "m2d_never_granted_drone_0_permission"
fi
echo "M2D grounded integration: READY, frozen assignment + four controllers + ACTIVE_0 permission."

# Visualization is deliberately non-authoritative. Launch it only after the flight
# stack has passed commissioning so RViz discovery/rendering cannot steal startup
# resources or invalidate an otherwise healthy mission.
if [[ "$GUI" == "true" ]]; then
  RVIZ_CONFIG="$REPO/install/drone_visualisation/share/drone_visualisation/rviz/m2d_four_drone.rviz"
  if [[ ! -f "$RVIZ_CONFIG" ]]; then
    echo "WARNING: M2D RViz config missing; continuing without visualization."
  else
    start_managed ros2 run rviz2 rviz2 -d "$RVIZ_CONFIG" --ros-args -p use_sim_time:=true
    echo "RViz: M2D four-drone planner/collision witness view LAUNCHED (non-authoritative)"
  fi
fi

if [[ "$GOAL_ATTACHMENTS" == "0" ]]; then
  write_pass_result 0
  exit 0
fi

if [[ "$SIMULTANEOUS" == "true" ]]; then
  # all four hold permission at once (testing): arm and launch each in quick
  # succession without waiting for the previous attachment (multi_drone_control)
  for i in $(seq 0 $((GOAL_ATTACHMENTS - 1))); do
    manual_activate_vehicle "$i"
  done
  if [[ "$OPERATOR_HOLD" == "true" ]]; then
    if wait_for_m2d_state M2D_SUCCESS_HOLD 3600; then   # wall clock: the GUI runs at ~0.1 RTF
      :
    else
      rc=$?
      [[ "$rc" -eq 2 ]] && fail "m2d_aborted_before_success_hold"
      fail "timeout_waiting_for_m2d_success_hold"
    fi
    echo "M2D SUCCESS HOLD: $GOAL_ATTACHMENTS SIMULTANEOUS software-proven attachment(s) remain actively monitored."
    # the handover driver takes the fleet from here and stops this runner
    while true; do sleep 5; done
  fi
  if wait_for_m2d_state M2D_PASS 900; then
    :
  else
    rc=$?
    [[ "$rc" -eq 2 ]] && fail "m2d_aborted_during_simultaneous_attachment"
    fail "timeout_waiting_for_m2d_pass"
  fi
  write_pass_result "$GOAL_ATTACHMENTS"
  echo "M2D PASS: $GOAL_ATTACHMENTS SIMULTANEOUS software-proven attachment(s) completed."
  exit 0
fi

manual_activate_vehicle 0
if [[ "$GOAL_ATTACHMENTS" == "1" ]]; then
  if wait_for_m2d_state ACTIVE_1 600; then
    :
  else
    rc=$?
    [[ "$rc" -eq 2 ]] && fail "m2d_aborted_during_drone_0_attachment"
    fail "drone_0_attachment_goal_timeout"
  fi
  echo "drone_0 attachment is software-proven; supervisor advanced to ACTIVE_1."
  write_pass_result 1
  exit 0
fi

# Multi-attachment mission. ACTIVE_0 was already announced. Advance only after
# software proof; the operator remains responsible for each ARM/TAKEOFF action.
for next_index in $(seq 1 $((GOAL_ATTACHMENTS - 1))); do
  if wait_for_m2d_state "ACTIVE_${next_index}" 600; then
    :
  else
    rc=$?
    [[ "$rc" -eq 2 ]] && fail "m2d_aborted_before_drone_${next_index}"
    fail "timeout_waiting_for_drone_${next_index}_permission"
  fi
  echo "drone_$((next_index - 1)) attachment: SOFTWARE-PROVEN and stable."
  manual_activate_vehicle "$next_index"
done

if [[ "$OPERATOR_HOLD" == "true" ]]; then
  if wait_for_m2d_state M2D_SUCCESS_HOLD 600; then
    :
  else
    rc=$?
    [[ "$rc" -eq 2 ]] && fail "m2d_aborted_before_success_hold"
    fail "timeout_waiting_for_m2d_success_hold"
  fi
  echo "M2D SUCCESS HOLD: $GOAL_ATTACHMENTS software-proven attachment(s) remain actively monitored."
  echo "Use the M2 Fleet Panel LAND FLEET button when ready. Landing will run in reverse attachment order."
  if wait_for_m2d_state M2D_LANDED 1800; then
    :
  else
    rc=$?
    [[ "$rc" -eq 2 ]] && fail "m2d_aborted_during_operator_landing"
    fail "timeout_waiting_for_operator_landing"
  fi
  write_pass_result "$GOAL_ATTACHMENTS"
  echo "M2D operator-hold PASS: success hold, release-before-land, reverse sequential landing, and disarm completed."
else
  if wait_for_m2d_state M2D_PASS 600; then
    :
  else
    rc=$?
    [[ "$rc" -eq 2 ]] && fail "m2d_aborted_during_final_attachment"
    fail "timeout_waiting_for_m2d_pass"
  fi
  write_pass_result "$GOAL_ATTACHMENTS"
  echo "M2D PASS: $GOAL_ATTACHMENTS sequential software-proven attachment(s) completed."
fi

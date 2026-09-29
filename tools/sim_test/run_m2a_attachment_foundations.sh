#!/usr/bin/env bash
# Supervised M2A bench test using the unchanged real X3 plus an invisible support.
# No online_join_planner, C++ planner, MPC, Betaflight or flight reference authority.
set -e

REPO="${HOME}/Documents/Thesis git/drone_cage_control"
SCENARIO="${1:-sanity}"
STAMP="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="$REPO/logs/m2_attachment/m2a_${SCENARIO}_${STAMP}"
mkdir -p "$LOG_DIR"

cd "$REPO"
source /opt/ros/humble/setup.bash
source install/setup.bash
export PYTHONPATH="$REPO/src/tejen_mission${PYTHONPATH:+:$PYTHONPATH}"
export GZ_SIM_RESOURCE_PATH="$REPO/simulation_assets${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"

if [[ ! -f "$REPO/simulation_assets/tejen/modelLargeM2BallMagnet.sdf" ]]; then
  echo "ERROR: expected existing simulation_assets/tejen/modelLargeM2BallMagnet.sdf from the M2A checkpoint."
  echo "This runner intentionally does not replace or regenerate the X3 model."
  exit 1
fi

case "$SCENARIO" in
  sanity|success|miss|magnet_off|forced_separation) ;;
  *) echo "Usage: $0 [sanity|success|miss|magnet_off|forced_separation]"; exit 2 ;;
esac

PIDS=()
cleanup() {
  set +e
  for pid in "${PIDS[@]}"; do kill "$pid" 2>/dev/null || true; done
  wait 2>/dev/null || true
}
interrupt_cleanup() {
  trap - EXIT INT TERM
  cleanup
  exit 130
}
trap cleanup EXIT
trap interrupt_cleanup INT TERM

start_bg() {
  "$@" >>"$LOG_DIR/runtime.log" 2>&1 &
  PIDS+=("$!")
}

wait_for_interrupt() {
  while true; do sleep 1; done
}

world_step_once() {
  gz service -s /world/quadcopter/control \
    --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean \
    --timeout 3000 --req 'multi_step: 1' >>"$LOG_DIR/runtime.log" 2>&1
}

wait_for_gz_state() {
  local file="$1"
  local state="$2"
  local tries=0
  while [[ $tries -lt 50 ]]; do
    if grep -q "data: \"${state}\"" "$file" 2>/dev/null; then
      return 0
    fi
    sleep 0.1
    tries=$((tries + 1))
  done
  echo "ERROR: timed out waiting for Gazebo joint state '${state}' in ${file}"
  return 1
}

echo "M2A bench scenario: $SCENARIO"
echo "Log dir: $LOG_DIR"
echo "Flight stack: OFF (no planner / MPC / Betaflight)"
echo "Drone model: existing modelLargeM2BallMagnet.sdf, unchanged"

# IMPORTANT: no -r. Gazebo starts paused. We clear the DetachableJoint's
# mandatory initial attachment before allowing free-running physics.
start_bg gz sim "$REPO/simulation_assets/tejen/world_m2a_attachment_foundations.sdf"
sleep 3

# Capture raw bootstrap transitions before the first simulation step.
stdbuf -oL -eL gz topic -e -t /payload/detachable_joint_state \
  >"$LOG_DIR/payload_joint_bootstrap.log" 2>&1 &
PIDS+=("$!")
stdbuf -oL -eL gz topic -e -t /m2a/support/state \
  >"$LOG_DIR/support_joint_bootstrap.log" 2>&1 &
PIDS+=("$!")

# Named TF bridge for the unchanged real X3.
start_bg ros2 run ros_gz_bridge parameter_bridge \
  '/model/x3/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'
start_bg python3 "$REPO/src/tejen_mission/tejen_mission/m2a_gazebo_joint_truth_bridge.py"
start_bg python3 "$REPO/src/tejen_mission/tejen_mission/m2a_physical_capture_manager.py" --ros-args \
  -p pose_source:=tf_named \
  -p carrier_tf_topic:=/model/x3/pose \
  -p magnet_frame_leaf:=magnet_tip_link \
  -p assigned_plate_id:=0 \
  -p fixed_ring_z:=0.10 \
  -p force_initial_detach:=false
start_bg python3 "$REPO/src/tejen_mission/tejen_mission/m2_attachment_observer.py" --ros-args \
  -p pose_source:=tf_named \
  -p carrier_tf_topic:=/model/x3/pose \
  -p carrier_frame_leaf:=base_link \
  -p magnet_frame_leaf:=magnet_tip_link \
  -p assigned_plate_id:=0 \
  -p fixed_ring_z:=0.10 \
  -p log_dir:="$LOG_DIR"
sleep 2

# STEP 1 while paused: allow both default DetachableJoints to initialize.
# Support->X3 is desired and remains attached. Magnet->ring is bootstrap-only.
world_step_once
wait_for_gz_state "$LOG_DIR/support_joint_bootstrap.log" "attached"
wait_for_gz_state "$LOG_DIR/payload_joint_bootstrap.log" "attached"
echo "Bootstrap: external X3 support attached; default magnet/ring joint observed attached."

# Request magnet/ring detach only after the plugin reports attached. Its API
# ignores detach requests while isAttached=false, so ordering matters.
gz topic -t /payload/detach -m gz.msgs.Empty -p '' >>"$LOG_DIR/runtime.log" 2>&1
world_step_once
wait_for_gz_state "$LOG_DIR/payload_joint_bootstrap.log" "detached"
echo "Bootstrap: magnet/ring joint detached while world still paused."

ros2 topic pub --once /m2a/attachment/proof_requested std_msgs/msg/Bool '{data: false}' >/dev/null
ros2 topic pub --once /magnet/command std_msgs/msg/String '{data: OFF}' >/dev/null
ros2 topic pub --once /m2a/probe/phase std_msgs/msg/String '{data: SANITY}' >/dev/null

# Only now allow physics to run.
gz service -s /world/quadcopter/control \
  --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean \
  --timeout 3000 --req 'pause: false' >>"$LOG_DIR/runtime.log" 2>&1
sleep 2

# Hard preflight: named X3 pose exists and truth is periodically available.
timeout 5 ros2 topic echo /model/x3/pose --once >>"$LOG_DIR/runtime.log" 2>&1
timeout 5 ros2 topic echo /m2a/sim/joint_detached_truth --once >>"$LOG_DIR/runtime.log" 2>&1

if [[ "$SCENARIO" == "sanity" ]]; then
  echo
  echo "SANITY MODE ACTIVE"
  echo "Expected scene:"
  echo "  - unchanged real X3 at x~0.55, held by an invisible external support"
  echo "  - static circular ring at origin, plate 0 highlighted on +x"
  echo "  - 0.5 m tether + magnet hanging freely, clear of the ring"
  echo "  - magnet OFF; magnet/ring detachable joint DETACHED"
  echo "  - no scripted approach motion"
  echo "Gazebo remains open until Ctrl-C."
  echo "Evidence directory: $LOG_DIR"
  wait_for_interrupt
fi

set +e
python3 "$REPO/tools/sim_test/m2a_attachment_probe.py" "$SCENARIO" \
  2>&1 | tee "$LOG_DIR/probe.log"
PROBE_RC=${PIPESTATUS[0]}
set -e
if [[ $PROBE_RC -ne 0 ]]; then
  echo "ERROR: M2A probe failed with code $PROBE_RC"
  exit "$PROBE_RC"
fi

echo "Probe complete. Holding Gazebo for 10 s for visual inspection..."
sleep 10

cleanup
trap - EXIT INT TERM

CSV="$LOG_DIR/attachment.csv"
if [[ ! -s "$CSV" ]]; then
  echo "ERROR: observer did not produce $CSV"
  exit 1
fi

python3 -m tejen_mission.m2a_attachment_replay "$CSV" \
  --metadata "$LOG_DIR/metadata.json" \
  --output "$LOG_DIR/attachment_replay.csv" | tee "$LOG_DIR/replay_summary.txt"
python3 "$REPO/analysis/m2_attachment/analyse_attachment.py" "$CSV" \
  --output-dir "$LOG_DIR/analysis" | tee "$LOG_DIR/analysis_summary.txt"

echo
echo "PASS: supervised M2A paused-bootstrap run completed."
echo "Evidence: $LOG_DIR"

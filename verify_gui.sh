#!/usr/bin/env bash
# Watch the cable-load controller in the Gazebo GUI.
#   bash verify_gui.sh                      # geometric (closed-loop), slow circle
#   bash verify_gui.sh geometric hover      # geometric, hold pose
#   bash verify_gui.sh kinematic circle     # open-loop A/B (load drifts ~1 m off)
# Ctrl+C to stop; it cleans up the stack + DDS shm on exit.
set -m
cd /home/wesley/multi_drone_control
source install/setup.bash                       # LAST, so this repo's pkgs win
export GZ_SIM_RESOURCE_PATH=$PWD/simulation_assets:$GZ_SIM_RESOURCE_PATH

MODE=${1:-geometric}
TRAJ=${2:-circle}
WORLD=simulation_assets/three_soft_paper.sdf
LOG=/tmp/verify_launch.log

PATS='gz sim|ros_gz_bridge/parameter_bridge|simulation_communication/lib|controller_quad_load/lib|controller_load_mpc/lib|payload_mocap|payload_betaflight'
cleanup() {
  echo "[verify] cleaning up..."
  for _ in 1 2 3; do p=$(pgrep -f "$PATS"); [ -z "$p" ] && break; for x in $p; do kill -9 "$x" 2>/dev/null; done; sleep 1; done
  pkill -9 -f 'ros2 launch controller_quad_load' 2>/dev/null
  rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
}
cleanup                       # clean slate (stale nodes/shm fake instability)
trap cleanup EXIT

echo "[verify] launching Gazebo GUI ($WORLD)..."
gz sim -r "$WORLD" -v1 &      # GUI (no -s); -r = start running
sleep 6

echo "[verify] launching stack: planner_mode=$MODE load_traj=$TRAJ"
ros2 launch controller_quad_load mpc_three_soft_quad_load_launch.py \
  planner_mode:=$MODE ff_gate_mode:=airborne start_taut:=true cable_len:=1.0 \
  ki_load_xy:=0.3 load_traj:=$TRAJ traj_speed:=0.1 traj_radius:=0.5 \
  target_z:=0.6 lift_ramp_vel:=0.05 >"$LOG" 2>&1 &

echo "[verify] waiting for planner..."
for i in $(seq 1 45); do grep -q 'ready, waiting for mocap' "$LOG" 2>/dev/null && break; sleep 1; done
sleep 5
echo "[verify] ARM"
for i in $(seq 1 15); do ros2 topic pub --once /fleet/command std_msgs/msg/String "{data: ARM}" >/dev/null 2>&1; grep -q "ARM sequence complete" "$LOG" 2>/dev/null && break; sleep 2; done
sleep 3
echo "[verify] TAKEOFF"
for i in $(seq 1 15); do ros2 topic pub --once /fleet/command std_msgs/msg/String "{data: TAKEOFF}" >/dev/null 2>&1; grep -q "received: 'TAKEOFF'" "$LOG" 2>/dev/null && break; sleep 2; done

cat <<'EOM'

[verify] Flying. In the Gazebo GUI watch the payload (the box under the 3 drones):
   geometric : lifts, then holds directly over the origin / traces a tight circle
   kinematic : the same motion but the load sits ~1 m off to one side (no feedback)
Live lateral error is printed here each second:  ros2 topic echo /payload/state
Or tail the planner:  grep 'planner geometric' /tmp/verify_launch.log | tail
Ctrl+C to stop.
EOM
# stream the planner's live error line (geometric prints e_xy=..)
tail -n0 -f "$LOG" | grep --line-buffered -E 'planner (geometric|kinematic)'

#!/usr/bin/env bash
# run_exit.sh <flight> <tag>: the C++ exit (exit_handoff:=moving, the trial line's resolved parameters) over a 13 Nov
# recording. ring_bridge and his real backend binary on the bag's ring, carriers and ring trajectory state; the mission's
# exit code in driver_exit.py; a virtual QUAD4. Nothing flies.
#   ERR=1|0      the flight's recorded tracking error added to the reference (1, default) or perfect tracking (0)
#   GOAL=mat|own the start hover over the mat (0, +1.55) (default) or the flight's own TAKEOFF xy
#   START=shift|drop  shift (default): the replay's CLEAR_DROP_ZONE entry RUN+1 s before the recorded LAND (so the
#                ring keeps flying its path for the whole exit; never later than the recorded drop); drop: at the
#                recorded CLEAR_DROP_ZONE entry (the fleet lands in the recording 9.8-13.1 s later)
#   LOAD=n       n python busy loops during the run (the 13 Nov stand-in for the rig stack)
#   RUN=s        replay length after the start (default 32)
# (no set -u: the ROS setup scripts read unset variables)
F=$1; TAG=${2:-}
D=$(cd "$(dirname "$0")" && pwd)
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-79} ROS_LOCALHOST_ONLY=1
source ~/ros2_humble/install/setup.bash >/dev/null 2>&1
source /home/wesley/multi_drone_control/install/setup.bash >/dev/null 2>&1
# MDC_PY_SNAPSHOT: a frozen copy of the mission package imported ahead of src (set by disable_attempts.sh)
[ -n "${MDC_PY_SNAPSHOT:-}" ] && export PYTHONPATH=$MDC_PY_SNAPSHOT:$PYTHONPATH
ERR=${ERR:-1}; GOAL=${GOAL:-mat}; START=${START:-shift}; LOAD=${LOAD:-0}; RUN=${RUN:-32}; WARM=${WARM:-5}
O=${OUTROOT:-$D/runs}/$F$TAG; rm -rf "$O"; mkdir -p "$O"
read -r TSTART OFF GX GY CUT < <(python3 - "$D/flights/$F.json" "$START" "$RUN" "$WARM" "$GOAL" <<'EOF'
import json, sys
j = json.load(open(sys.argv[1])); start, run, warm, goal = sys.argv[2], float(sys.argv[3]), float(sys.argv[4]), sys.argv[5]
t = j['t_clear'] if start == 'drop' else min(j['t_clear'], j['t_land'] - run - 1.0)
gx, gy = (0.0, 1.55) if goal == 'mat' else j['start_xy']
print(f"{t:.6f} {t - warm - j['bag_start']:.3f} {gx:.4f} {gy:.4f} {j['land_after_clear_s'] - 0.3:.3f}")
EOF
)
{ echo "flight $F tag $TAG ERR=$ERR GOAL=$GOAL ($GX, $GY) START=$START LOAD=$LOAD RUN=$RUN t_start $TSTART bag offset $OFF err_cut $CUT"
  stat -c "%y %n" /home/wesley/multi_drone_control/build/tejen_dynamic_planner/dynamic_planner_transfer_backend
  stat -c "%y %n" /home/wesley/multi_drone_control/src/tejen_mission/tejen_mission/online_join_planner.py \
    /home/wesley/multi_drone_control/src/tejen_mission/tejen_mission/c1f2_handoff.py
  (cd /home/wesley/multi_drone_control && git rev-parse HEAD && git diff --stat src/tejen_dynamic_planner src/tejen_mission | tail -1 \
    && git diff src/tejen_dynamic_planner src/tejen_mission | sha1sum)
  M=${MDC_PY_SNAPSHOT:-/home/wesley/multi_drone_control/src/tejen_mission}
  sha1sum "$M"/tejen_mission/online_join_planner.py "$M"/tejen_mission/c1f2_handoff.py
  echo "PARAMDIR ${PARAMDIR:-default} RIG_TRANSFER ${RIG_TRANSFER:-0}"
  date; } > "$O/run.txt"
PD=${PARAMDIR:-$D}   # PARAMDIR: the resolved parameter set (backend.yaml, ring_bridge.yaml, planner_params.json)
sed "s#BACKEND_CSV#$O/backend.csv#; s#REPLAN_CSV#$O/replans.csv#" "$PD/backend.yaml" > "$O/backend.yaml"
cp "$PD/ring_bridge.yaml" "$O/ring_bridge.yaml"; cp "$PD/planner_params.json" "$O/planner_params.json"
export PLANNER_PARAMS=$PD/planner_params.json
PIDS=""
for i in $(seq 1 "$LOAD"); do python3 -c "while True: pass" & PIDS="$PIDS $!"; done
/home/wesley/multi_drone_control/install/tejen_mission/lib/tejen_mission/ring_bridge --ros-args --params-file "$O/ring_bridge.yaml" > "$O/ring_bridge.log" 2>&1 &
P1=$!
/home/wesley/multi_drone_control/install/tejen_dynamic_planner/lib/tejen_dynamic_planner/dynamic_planner_transfer_backend --ros-args --params-file "$O/backend.yaml" > "$O/backend.log" 2>&1 &
P2=$!
EARG=""; [ "$ERR" = 1 ] && EARG="-p err_csv:=$D/flights/${F}_err.csv -p err_cut_s:=$CUT"
python3 "$D/driver_exit.py" --ros-args -p use_sim_time:=true -p t_start:=$TSTART -p goal_x:=$GX -p goal_y:=$GY \
  -p run_s:=$RUN -p out:="$O" $EARG > "$O/driver.log" 2>&1 &
P3=$!
trap 'kill $P1 $P2 $P3 $P4 $PIDS 2>/dev/null; [ -n "${P5:-}" ] && kill -INT $P5 2>/dev/null; sleep 2; kill -9 $P1 $P2 $P3 $P4 $PIDS 2>/dev/null; wait 2>/dev/null' EXIT
if [ "${RECORD:-0}" = 1 ]; then   # RECORD=1: ring_bridge's outputs and the ring input, receive-timed (system clock)
  ros2 bag record -o "$O/feeds" /fake_payload/pose_array /fake_payload/committed_trajectory /payload/motion_capture_state \
    /fake_obstacles/drone_0/state /fake_obstacles/drone_1/state /fake_obstacles/drone_2/state \
    /fake_obstacles/drone_0/committed_trajectory /fake_obstacles/drone_1/committed_trajectory \
    /fake_obstacles/drone_2/committed_trajectory /clock > "$O/record.log" 2>&1 &
  P5=$!
fi
sleep 4
timeout $(python3 -c "print(int($RUN + $WARM + 25))") ros2 bag play /home/wesley/multi_drone_control/results/rig/2026-11-13/$F \
  --topics /payload/motion_capture_state /payload/trajectory_state /drone_0/motion_capture_state /drone_1/motion_capture_state /drone_2/motion_capture_state \
  --clock 200 --start-offset "$OFF" --disable-keyboard-controls > "$O/play.log" 2>&1 &
P4=$!
wait $P3
sleep 1
echo "done $F$TAG"

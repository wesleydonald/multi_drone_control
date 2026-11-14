#!/usr/bin/env bash
# run_flight.sh <flight> <bag offset s> <x0> <y0> <z0> <run_s> [tag]: open-loop replay of his backend on a 12 Nov recording
F=$1; OFF=$2; X0=$3; Y0=$4; Z0=$5; RUN=$6; TAG=${7:-}
D=$(cd "$(dirname "$0")" && pwd)
export ROS_DOMAIN_ID=77 ROS_LOCALHOST_ONLY=1
source ~/ros2_humble/install/setup.bash >/dev/null 2>&1
source /home/wesley/multi_drone_control/install/setup.bash >/dev/null 2>&1
O=$D/runs/$F$TAG; mkdir -p $O
stat -c "%y %n" /home/wesley/multi_drone_control/build/tejen_dynamic_planner/dynamic_planner_transfer_backend > $O/binary.txt; date >> $O/binary.txt
sed "s#BACKEND_CSV#$O/backend.csv#; s#REPLAN_CSV#$O/replans.csv#" $D/backend${BK:-}.yaml > $O/backend.yaml
cp $D/ring_bridge.yaml $O/ring_bridge.yaml
[ -n "${RELAY:-}" ] && echo '    ring_pose_array_topic: /rb/pose_array' >> $O/ring_bridge.yaml
/home/wesley/multi_drone_control/install/tejen_mission/lib/tejen_mission/ring_bridge --ros-args --params-file $O/ring_bridge.yaml > $O/ring_bridge.log 2>&1 &
P1=$!
/home/wesley/multi_drone_control/install/tejen_dynamic_planner/lib/tejen_dynamic_planner/dynamic_planner_transfer_backend --ros-args --params-file $O/backend.yaml > $O/backend.log 2>&1 &
P2=$!
python3 $D/driver.py --ros-args -p use_sim_time:=true -p x0:=$X0 -p y0:=$Y0 -p z0:=$Z0 -p run_s:=$RUN -p log:=$O/driver.csv -p latch:=${LATCH:-false} -p relay_hz:=${RELAY:-0.0} > $O/driver.log 2>&1 &
P3=$!
trap 'kill $P1 $P2 $P3 $P4 2>/dev/null; sleep 1; pkill -f "$O/(ring_bridge|backend).yaml" 2>/dev/null; pkill -f "rigcheck/full/runs/$F$TAG/" 2>/dev/null; wait 2>/dev/null' EXIT
sleep 4
timeout $(python3 -c "print(int(${RUN}/${RATE:-1.0}) + 20)") ros2 bag play /home/wesley/multi_drone_control/results/rig/2026-11-12/$F \
  --topics /payload/motion_capture_state /payload/trajectory_state /drone_0/motion_capture_state /drone_1/motion_capture_state /drone_2/motion_capture_state \
  --clock 200 -r ${RATE:-1.0} --start-offset $OFF --disable-keyboard-controls > $O/play.log 2>&1 &
P4=$!
wait $P3
sleep 1
echo "done $F$TAG"

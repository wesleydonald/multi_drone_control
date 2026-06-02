#!/bin/bash
# === CONFIG ===
WORKSPACE=~/multi_drone_control
SIM_ASSETS=$WORKSPACE/simulation_assets
ROS_SETUP="/opt/ros/humble/setup.bash"
PID_FILE=/tmp/multi_drone_pids.txt

# Kill any existing session before starting
if [ -f "$PID_FILE" ]; then
    echo "Existing session found — stopping it first..."
    bash "$(dirname "$0")/stop_multi.sh"
    sleep 2
fi

# Clear PID file
> $PID_FILE

# Helper: launch a gnome-terminal, save its PID
launch_term() {
    local TITLE="$1"
    local CMD="$2"
    gnome-terminal --title="$TITLE" -- bash -ic "$CMD" &
    echo $! >> $PID_FILE
}

# === Gazebo ===
launch_term "Gazebo Sim" "
cd $SIM_ASSETS || { echo 'SIM_ASSETS NOT FOUND'; exec bash; }
source $ROS_SETUP
source /usr/share/gz/setup.bash 2>/dev/null || true
source $WORKSPACE/install/setup.bash
echo 'Starting Gazebo...'
gz sim world_multi.sdf -v 4 -r
exec bash
"
sleep 3

# === ROS Launch ===
launch_term "ROS Launch" "
source $ROS_SETUP
source $WORKSPACE/install/setup.bash
ros2 launch simulation_communication betaflight_simulation_launch.py num_drones:=4
exec bash
"
sleep 4

# === Pre-compile acados (drone 0 only, serialised with lock in controller) ===
echo "Pre-compiling acados solver (drone 0)..."
launch_term "Acados Compile" "
source $ROS_SETUP
source $WORKSPACE/install/setup.bash
echo 'Compiling acados — wait for Controller ready message...'
ros2 run controller_mpc_multi controller --ros-args -p drone_id:=0 -r __node:=controller_0 &
CTRL_PID=\$!
until [ -f $WORKSPACE/c_generated_code/libacados_ocp_solver_quad_dynamics.so ]; do
    sleep 0.5
done
sleep 3
kill \$CTRL_PID 2>/dev/null
echo 'Acados compile done.'
sleep 2
"
sleep 20

# === Controllers ===
for i in 0 1 2 3; do
    launch_term "Controller $i" "
source $ROS_SETUP
source $WORKSPACE/install/setup.bash
ros2 run controller_mpc_multi controller --ros-args -p drone_id:=$i -r __node:=controller_$i
exec bash
"
    sleep 1
done
sleep 3

# === Fleet Manager ===
launch_term "Fleet Manager" "
source $ROS_SETUP
source $WORKSPACE/install/setup.bash
ros2 run controller_mpc_multi main --ros-args -r __node:=central_controller
exec bash
"
sleep 2

# === Fleet Commands terminal ===
launch_term "Fleet Commands" "
source $ROS_SETUP
source $WORKSPACE/install/setup.bash

history -s 'ros2 topic pub --once /fleet/command std_msgs/msg/String \"{data: ARM}\"'
history -s 'ros2 topic pub --once /fleet/command std_msgs/msg/String \"{data: TAKEOFF}\"'
history -s 'ros2 topic pub --once /fleet/command std_msgs/msg/String \"{data: DISARM}\"'
history -s 'ros2 topic pub --once /fleet/command std_msgs/msg/String \"{data: ESTOP}\"'

echo ''
echo '========================================='
echo '  Fleet Command Terminal'
echo '========================================='
echo '  UP ARROW cycles through:'
echo '    ARM  ->  TAKEOFF  ->  DISARM  ->  ESTOP'
echo '========================================='
echo ''
exec bash
"

echo "All terminals launched. PIDs saved to $PID_FILE"
echo "Run ./stop_multi.sh to close everything."
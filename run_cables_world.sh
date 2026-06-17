#!/bin/bash
# run_cables_world.sh — 4-drone rigid-cable payload stack
#
# STEP 1  verify motors (run in Gazebo terminal after world loads):
#   gz topic -t /x3_drone0/gazebo/command/motor_speed --msgtype gz.msgs.Actuators -p 'velocity:[700, 700, 700, 700]'
#   gz topic -t /x3_drone1/gazebo/command/motor_speed --msgtype gz.msgs.Actuators -p 'velocity:[700, 700, 700, 700]'
#   gz topic -t /x3_drone2/gazebo/command/motor_speed --msgtype gz.msgs.Actuators -p 'velocity:[700, 700, 700, 700]'
#   gz topic -t /x3_drone3/gazebo/command/motor_speed --msgtype gz.msgs.Actuators -p 'velocity:[700, 700, 700, 700]'
#
# STEP 2  verify ROS bridge (any terminal after ROS Launch starts):
#   ros2 topic echo /drone_0/motion_capture_state --once
#   ros2 topic echo /payload/motion_capture_state --once
#
# STEP 3  run the controller (Fleet Commands terminal, UP arrow to cycle):
#   ARM -> TAKEOFF -> DISARM

WORKSPACE=~/multi_drone_control
SIM_ASSETS=$WORKSPACE/simulation_assets
ROS_SETUP="/opt/ros/humble/setup.bash"
PID_FILE=/tmp/cables_world_pids.txt

if [ -f "$PID_FILE" ]; then
    echo "Existing session found — stopping it first..."
    bash "$(dirname "$0")/stop_cables_world.sh"
    sleep 2
fi
> $PID_FILE

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
gz sim world_multi_cables_2.sdf -v 4 -r
exec bash
"
sleep 5

# === ROS Launch (bridges + all controller nodes) ===
launch_term "ROS Launch" "
source $ROS_SETUP
source $WORKSPACE/install/setup.bash
ros2 launch controller_cable_payload rigid_cables_launch.py
exec bash
"
sleep 4

# === Fleet Commands ===
launch_term "Fleet Commands" "
source $ROS_SETUP
source $WORKSPACE/install/setup.bash

history -s 'ros2 topic pub --once /fleet/command std_msgs/msg/String \"{data: ARM}\"'
history -s 'ros2 topic pub --once /fleet/command std_msgs/msg/String \"{data: TAKEOFF}\"'
history -s 'ros2 topic pub --once /fleet/command std_msgs/msg/String \"{data: DISARM}\"'
history -s 'ros2 topic pub --once /fleet/command std_msgs/msg/String \"{data: ESTOP}\"'

echo 'Cable-Payload Fleet Commands — UP arrow to cycle: ARM -> TAKEOFF -> DISARM'
exec bash
"

echo "All terminals launched. PIDs saved to $PID_FILE"
echo "Run ./stop_cables_world.sh to close everything."

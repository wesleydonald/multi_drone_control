#!/bin/bash
# Launches the cable-payload hover stack.
# Usage: ./run_payload_hover.sh
#
# After all terminals open:
#   1. In "Fleet Commands" terminal, press UP to cycle:
#        ARM → TAKEOFF → DISARM
#
# Motor topic note:
#   The motor command topic for drones nested inside lift_system is expected
#   to be  /lift_system/x3_drone{i}/gazebo/command/motor_speed
#   Verify with:  gz topic --list | grep motor_speed
#   If different, edit payload_hover_launch.py motor_bridge argument.

WORKSPACE=~/multi_drone_control
SIM_ASSETS=$WORKSPACE/simulation_assets
ROS_SETUP="/opt/ros/humble/setup.bash"
PID_FILE=/tmp/payload_hover_pids.txt

if [ -f "$PID_FILE" ]; then
    echo "Existing session found — stopping it first..."
    bash "$(dirname "$0")/stop_payload_hover.sh"
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
echo 'Starting Gazebo...'
gz sim world_quad_payload.sdf -v 4 -r
exec bash
"
sleep 4

# === ROS Launch (bridges + all nodes) ===
launch_term "ROS Launch" "
source $ROS_SETUP
source $WORKSPACE/install/setup.bash
ros2 launch controller_cable_payload payload_hover_launch.py
exec bash
"
sleep 3

# === Fleet Commands ===
launch_term "Fleet Commands" "
source $ROS_SETUP
source $WORKSPACE/install/setup.bash

history -s 'ros2 topic pub --once /fleet/command std_msgs/msg/String \"{data: ARM}\"'
history -s 'ros2 topic pub --once /fleet/command std_msgs/msg/String \"{data: TAKEOFF}\"'
history -s 'ros2 topic pub --once /fleet/command std_msgs/msg/String \"{data: DISARM}\"'
history -s 'ros2 topic pub --once /fleet/command std_msgs/msg/String \"{data: ESTOP}\"'

echo ''
echo '========================================='
echo '  Cable-Payload Fleet Command Terminal'
echo '========================================='
echo '  ARM → TAKEOFF → DISARM / ESTOP'
echo '========================================='
exec bash
"

echo "All terminals launched.  PIDs saved to $PID_FILE"
echo "Run ./stop_payload_hover.sh to close everything."

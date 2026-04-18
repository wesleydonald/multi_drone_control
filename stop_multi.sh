#!/bin/bash
# Kills all processes started by run_multi.sh

echo "Stopping all drone processes..."

# Kill ROS nodes by name
pkill -f "controller_mpc_multi" 2>/dev/null
pkill -f "betaflight_simulation_launch" 2>/dev/null
pkill -f "gz sim" 2>/dev/null
pkill -f "gzserver" 2>/dev/null
pkill -f "gzclient" 2>/dev/null

# Kill any lingering ros2 processes related to this project
pkill -f "ros2 run controller_mpc_multi" 2>/dev/null
pkill -f "ros2 launch simulation_communication" 2>/dev/null

# Close gnome-terminal windows by title
for TITLE in "Gazebo Sim" "ROS Launch" "Controller 0" "Controller 1" "Controller 2" "Controller 3" "Fleet Manager" "Fleet Commands" "Acados Compile"
do
    WID=$(wmctrl -l 2>/dev/null | grep "$TITLE" | awk '{print $1}')
    if [ -n "$WID" ]; then
        wmctrl -ic "$WID" 2>/dev/null
    fi
done

echo "Done."
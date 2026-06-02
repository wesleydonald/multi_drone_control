#!/bin/bash
echo "Stopping payload hover processes..."
pkill -f "controller_cable_payload" 2>/dev/null
pkill -f "payload_hover_launch" 2>/dev/null
pkill -f "gz sim" 2>/dev/null
pkill -f "gzserver" 2>/dev/null
pkill -f "gzclient" 2>/dev/null
pkill -f "ros2 launch controller_cable_payload" 2>/dev/null
for TITLE in "Gazebo Sim" "ROS Launch" "Fleet Commands"; do
    WID=$(wmctrl -l 2>/dev/null | grep "$TITLE" | awk '{print $1}')
    [ -n "$WID" ] && wmctrl -ic "$WID" 2>/dev/null
done
echo "Done."

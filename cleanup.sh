#!/bin/bash
pkill -9 -f "python3.*controller_cable_payload"
pkill -9 -f "parameter_bridge"
pkill -9 -f "ros2"
killall -9 gzserver gzclient 2>/dev/null || true
echo "All ROS/Gazebo processes killed."

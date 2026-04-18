#!/bin/bash

# === CONFIG ===
WORKSPACE=~/thesis/src/multi_drone_control
SIM_ASSETS=$WORKSPACE/simulation_assets
ROS_SETUP="/opt/ros/humble/setup.bash"

# === Gazebo ===
gnome-terminal --title="Gazebo Sim" -- bash -ic "
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
gnome-terminal --title="ROS Launch" -- bash -ic "
source $ROS_SETUP
source $WORKSPACE/install/setup.bash

ros2 launch simulation_communication betaflight_simulation_launch.py num_drones:=4

exec bash
"

sleep 2

# === Controllers ===
for i in 0 1 2 3
do
gnome-terminal --title="Controller $i" -- bash -ic "
source $ROS_SETUP
source $WORKSPACE/install/setup.bash

ros2 run controller_pid main --ros-args -p drone_id:=$i

exec bash
"
done
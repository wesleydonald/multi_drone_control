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

# === Pre-compile acados (drone 0 only, others reuse the .so) ===
# We compile once synchronously to avoid race conditions when all 4 controllers
# start simultaneously and try to write to c_generated_code/ at the same time.
echo "Pre-compiling acados solver (drone 0)..."
gnome-terminal --title="Acados Compile" -- bash -ic "
source $ROS_SETUP
source $WORKSPACE/install/setup.bash
echo 'Compiling acados — wait for Controller ready message, then this terminal will close.'
ros2 run controller_mpc_multi controller --ros-args -p drone_id:=0 &
CTRL_PID=\$!
# Wait until the .so file exists and controller is ready
until [ -f $WORKSPACE/c_generated_code/libacados_ocp_solver_quad_dynamics.so ]; do
    sleep 0.5
done
sleep 3  # give it a moment to fully initialise
kill \$CTRL_PID 2>/dev/null
echo 'Acados compile done. Closing...'
sleep 1
exec bash
"
sleep 20  # wait for compilation — adjust if your machine is faster/slower

# === Controllers ===
for i in 0 1 2 3
do
gnome-terminal --title="Controller $i" -- bash -ic "
source $ROS_SETUP
source $WORKSPACE/install/setup.bash
ros2 run controller_mpc_multi controller --ros-args -p drone_id:=$i
exec bash
"
sleep 1  # slight stagger so they don't all hit acados init simultaneously
done
sleep 3

# === Fleet Manager (CentralController) ===
gnome-terminal --title="Fleet Manager" -- bash -ic "
source $ROS_SETUP
source $WORKSPACE/install/setup.bash
ros2 run controller_mpc_multi main
exec bash
"
sleep 2

# === Command Terminal ===
# A clean terminal pre-loaded with the ARM/TAKEOFF/DISARM commands as history
gnome-terminal --title="Fleet Commands" -- bash -ic "
source $ROS_SETUP
source $WORKSPACE/install/setup.bash

# Pre-load commands into bash history for quick access
history -s 'ros2 topic pub --once /fleet/command std_msgs/msg/String \"{data: ARM}\"'
history -s 'ros2 topic pub --once /fleet/command std_msgs/msg/String \"{data: TAKEOFF}\"'
history -s 'ros2 topic pub --once /fleet/command std_msgs/msg/String \"{data: DISARM}\"'
history -s 'ros2 topic pub --once /fleet/command std_msgs/msg/String \"{data: ESTOP}\"'

echo ''
echo '========================================='
echo '  Fleet Command Terminal'
echo '========================================='
echo '  Use UP ARROW to cycle through commands:'
echo '    1. ARM'
echo '    2. TAKEOFF'
echo '    3. DISARM'
echo '    4. ESTOP'
echo ''
echo '  Or type manually:'
echo '  ros2 topic pub --once /fleet/command std_msgs/msg/String \"{data: ARM}\"'
echo '  ros2 topic pub --once /fleet/command std_msgs/msg/String \"{data: TAKEOFF}\"'
echo '  ros2 topic pub --once /fleet/command std_msgs/msg/String \"{data: DISARM}\"'
echo '========================================='
echo ''
exec bash
"

echo "All terminals launched."
echo "Sequence: wait for all 4 controllers to show 'Controller ready', then use the Fleet Commands terminal."
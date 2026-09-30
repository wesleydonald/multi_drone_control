#!/usr/bin/env bash
# Same overlay order as the laptop's `mdc` alias: ROS, then the workspace if built.
source /opt/ros/humble/setup.bash
[ -f /home/wesley/multi_drone_control/install/setup.bash ] && source /home/wesley/multi_drone_control/install/setup.bash
exec "$@"

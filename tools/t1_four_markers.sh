#!/usr/bin/env bash
# T1 for four quads with every magnet marker (bodies 11-14, payload 8, markers 21-24), on the lab-private ROS settings.
# Extra launch arguments pass through, e.g. tools/t1_four_markers.sh detach:=true
source ~/ros2_humble/install/setup.bash
cd ~/multi_drone_control && source install/setup.bash
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=23 MDC_LEAN_RCLPY=1 PYTHONPATH=$HOME/multi_drone_control/tools/lean_rclpy:$PYTHONPATH
exec ros2 launch bringup real_io_launch.py num_drones:=4 mocap_drone_body_ids:=11,12,13,14 mocap_payload_body_id:=8 \
  magnet_initial:=ON \
  mocap_extra_bodies:=21:/drone_0/magnet_tip_state,22:/drone_1/magnet_tip_state,23:/drone_2/magnet_tip_state,24:/drone_3/magnet_tip_state "$@"

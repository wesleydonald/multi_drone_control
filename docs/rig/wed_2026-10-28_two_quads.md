# Lab Wed 28 Oct: two quads (QUAD2, QUAD3), 0.55 m cables

QUAD2 on plate 11, QUAD3 on plate 5 (the 22 Oct pair layout, clear of 0/3/6/9). Fly down the list; LAND ~15-20 s
after the top of the lift, or after the path. Fresh packs for each pair of flights.

```bash
# every terminal
alias w5='source ~/ros2_humble/install/setup.bash && cd ~/multi_drone_control && source install/setup.bash && export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=23 MDC_LEAN_RCLPY=1 MDC_PLANNER_CPUS=0-7 PYTHONPATH=$HOME/multi_drone_control/tools/lean_rclpy:$PYTHONPATH'
w5; C=$HOME/multi_drone_control/configs/rig
P="tools/rig_flight.sh real_control_launch.py mode:=dissipative num_drones:=2 attach_azimuths_deg:=330,150 target_z:=1.0 lift_ramp_vel:=0.15 attach_z:=0.0175 payload_rest_z:=0.0275 cable_len:=0.55 throttle_max:=0.9"
# T1 (slope filter, Wesley 28 Oct after hE1/hS1: keep it for every flight)
MDC_MOCAP_VEL_FILTER=slope MDC_MOCAP_VEL_DRONES=0,1 ros2 launch bringup real_io_launch.py num_drones:=2 drone0_serial:=/dev/QUAD2 drone1_serial:=/dev/QUAD3 mocap_drone_body_ids:=12,13 magnet_initial:=ON mocap_extra_bodies:=22:/drone_0/magnet_tip_state,23:/drone_1/magnet_tip_state
```
`cable_len:=0.55` must be typed: the default is now 1.0.

## 1. Hover (done: hE1 old filter, hS1 slope)
Slope won clearly (swing 1.06 against 2.69 deg rms, decaying against growing). One more slope hover if you want a
repeat: `$P load_traj:=hover params_file:=$C/two_quad_nohold.yaml hS2`.

## 2. Figure-8 x 2 (0.75 m, 0.2 m/s, 5 s hold)
```bash
$P load_traj:=fig_8 traj_speed:=0.2 params_file:=$C/two_quad_hold5.yaml f2a      # then f2b
```
With the slope filter. Tests: the path error and the pair's tilt about its line (22 Oct, size 1.0: 25.2 mm, tilt peak 14.3 deg).

## 3. Spinny line x 2 (0.75 m out, 0.2 m/s, one full turn)
```bash
$P load_traj:=line_spin traj_speed:=0.2 params_file:=$C/two_quad_hold5.yaml ls2a  # then ls2b
```
Tests: never flown by a pair in the real world. Expect the ring's heading to trail the turn by ~0.5 s.

## Stop rules
- **See-saw** (one end of the ring up, the other down) over 15 deg peak-to-peak, or growing: LAND.
- **Automatic drop:** the ring drops by itself at 55 deg tilt, and the fleet lands if a quad passes ±1.75 m.
- SPACE disarms at once.

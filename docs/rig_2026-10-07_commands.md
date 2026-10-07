# Rig round 2: measured cable pull vs feedforward

**Since 7 Oct afternoon the measured pull at 60 deg is the default everywhere** (common.yaml;
decisions.md 7 Oct): plain commands fly it, detach included. Round 2 result: circle and
figure-8 path error a third, lag half, bobbing a tenth of feedforward (card 2026-10-07_cable_mocap).
Feedforward for comparisons: `params_file:=$PWD/configs/rig/feedforward_45.yaml`.
Figure-8 since 7 Oct evening: `traj_speed` is the constant path speed (was the peak of one slow
sweep); fly `load_traj:=fig_8 traj_radius:=1.0 traj_speed:=0.125` (lap 53 s). Always type
`traj_speed`: the rig default is 0.4.

## Setup

```bash
cd ~/multi_drone_control; mdc
colcon build --symlink-install --packages-select bringup mpc_planner tracker fleet_manager dissipative_planner simulation_communication && source install/setup.bash
python3 tools/prebuild_planner.py 2 3 4 --load-mass 0.86 --drone-mass 0.55 --cable-len 0.55
D=results/rig/$(date +%F); mkdir -p $D; git rev-parse HEAD > $D/code.txt; git status --short >> $D/code.txt
rio num_drones:=4 magnet_channel:=6 magnet_initial:=ON   # T1, all session
```
- Even ring, plates 0/3/6/9 (counter-clockwise from +x); packs >= 23.6 V.

## Each run

```bash
tools/rig_flight.sh <name> real_control_launch.py <args>     # T2
fleet arm; fleet takeoff                                       # hovers: fleet land after 25 s at 1.0 m
```
Preflight on the first run: `python3 tools/preflight.py --drones 4 --real --planner mpc_planner --max-ground-z 0.5 --battery-min 23.6`

- `BASE` = `mode:=mpc num_drones:=4 target_z:=1.0 lift_ramp_vel:=0.15`
- `HOLD` = `params_file:=$PWD/configs/rig/traj_hold.yaml` (5 s hover hold before a path)

T2 prints `measured cable ... planned ... cap X%` once a second per drone; expect the
measured about 1 m/s^2 more inward than planned, cap 0 %.

## Runs as planned before the switch (feedforward, then measured; `MEAS` was the measured pull)

| # | names | args | compares |
|---|---|---|---|
| 1-2 | `r201`, `r202` | `BASE z_ki:=0.0`, `+ MEAS` | cable angle (58-67 deg) and ring height without the integral |
| 3-4 | `r203`, `r204` | `BASE`, `+ MEAS` | bobbing; force the integral learns (round 1: 0.9 N) |
| 5-6 | `r205`, `r206` | `BASE load_traj:=orbit traj_radius:=0.5 traj_speed:=0.125 HOLD`, then `HOLD_MEAS` | lag (0.45 s), wobble (5 cm); LAND 30 s after it moves; 1.7 m clear on +y |
| 7-8 | `r207`, `r208` | `BASE load_traj:=fig_8 traj_radius:=1.0 traj_speed:=0.25 HOLD`, then `HOLD_MEAS` | lag (0.65 s); lands itself; 2.2 m clear along x |
| 9-12 | | repeats of 1-2 and 5-6 | thesis pairs |
| 13-14 | | 100 g in the ring, `BASE z_ki:=0.0`, `+ MEAS` (optional) | does the measured pull see the weight |

# Rig commands, Thu 1 Oct 2026 (90 min, branch real-world-testing)

**Three drones tonight** (the QUAD3 airframe is out). The radios and mocap bodies are renumbered: drone 0 = QUAD1 / body 11,
drone 1 = QUAD2 / body 12, drone 2 = QUAD4 / body 14. Even ring on plates 0 / 4 / 8 (0, 120, 240 deg). No 3-drone rig
twin has been flown, so H1 is the first look at it.

Card A (`docs/experiments/2026-10-01_tethered_model.md`) holds the sim numbers, the rationale and the bars in
full. Every launch default below is already the rig value (geometry, affine thrust, throttle_max 0.8, pivot
-0.04, z_ki 0.4), so the lines only carry what differs. No SIL or Gazebo on this laptop while the rig is up.

| min | step |
|---|---|
| 0-15 | setup, ring +x on plate 0, magnets 0/4/8, packs, dry launch (props off) |
| 15-32 | D2 tethered holds at 0.3 and 0.7, z_ki 0 |
| 32-47 | H1 hover 0.5, z_ki 0 |
| 47-62 | H2 hover 0.5, net on |
| 62-85 | C1 circle (only if H1 and H2 pass; first to drop) |
| 85-90 | copy logs |

## 0. Setup

```bash
cd ~/multi_drone_control; mdc   # mdc = source both setup files
colcon build --symlink-install --packages-select controller_load_mpc controller_quad_load simulation_communication && source install/setup.bash
python3 tools/prebuild_planner.py --load-mass 0.86 --drone-mass 0.55 3   # n=3 solver
D=results/rig/2026-10-01; mkdir -p $D; git rev-parse HEAD > $D/code.txt; git status --short >> $D/code.txt
ls -l /dev/QUAD*                                   # QUAD1, QUAD2, QUAD4 = drones 0, 1, 2
for i in 0 1 2; do ros2 topic echo --once /drone_$i/telemetry | grep battery_voltage; done   # all >= 23.6, none 0.00
```
- Motive: ring body 8 with +x on plate 0, origin at the ring centre; ring placed with plate 0 toward world +x.

## 1. Terminals

```bash
# T1, all session (every terminal starts with: cd ~/multi_drone_control; mdc)
ros2 launch controller_quad_load real_io_launch.py num_drones:=3 drone0_serial:=/dev/QUAD1 drone1_serial:=/dev/QUAD2 drone2_serial:=/dev/QUAD4 mocap_drone_body_ids:=11,12,14 magnet_channel:=6 magnet_initial:=ON
# T5, all session
python3 tools/fleet_monitor.py --drones 3
```

Per flight, set `F` in T2, T3 and T4 to the same name:
```bash
# T2
cd ~/multi_drone_control; mdc; tools/clean_slate.sh --rig; F=h1; D=results/rig/2026-10-01; export MDC_RUN_DIR=$PWD/$D/${F}_logs
ros2 launch controller_quad_load <LAUNCH LINE BELOW> 2>&1 | tee $D/$F.log
# T3, after "Controller ready" x3
cd ~/multi_drone_control; mdc; F=h1; python3 tools/preflight.py --drones 3 --real --planner load_planner --max-ground-z 0.5 --battery-min 23.6
tail -F results/rig/2026-10-01/$F.log | grep --line-buffered -E "geometry:|datum|slot|SLOT OFFSET|elevation|measure_rod_len|pretension|solve status|near its bound|NOT lifting|timed out|FLEET|disarm|ESTOP|arc-creep|load down"
# T4
cd ~/multi_drone_control; mdc; F=h1; ros2 bag record -o results/rig/2026-10-01/$F /drone_{0..2}/motion_capture_state /payload/motion_capture_state /drone_{0..2}/ELRSCommand /drone_{0..2}/telemetry /fleet/command /fleet/abort /fleet/status /fleet/landed /drone_{0..2}/magnet /rosout
```
Dry launch once (H1 line, props off): "Controller ready" x3, nothing compiles, preflight GO, then Ctrl-C.

## 2. Flights (launch line goes after `ros2 launch controller_quad_load`)

| F | setup | launch line |
|---|---|---|
| `d2_z030` | | `real_control_launch.py num_drones:=3 z_ki:=0.0 target_z:=0.3` |
| `d2_z070` | | `real_control_launch.py num_drones:=3 z_ki:=0.0 target_z:=0.7` |
| `h1` | | `real_control_launch.py num_drones:=3 z_ki:=0.0 target_z:=0.5` |
| `h2` | | `real_control_launch.py num_drones:=3 target_z:=0.5` |
| `c1` | >= 1.7 m clear on +y, 1.2 m elsewhere | `real_control_launch.py num_drones:=3 target_z:=0.5 load_traj:=orbit traj_radius:=0.5 traj_speed:=0.125` |

- D2: hold 15 s steady.
- H1, H2: hold 25 s.
- C1: LAND about 30 s after the ring starts moving (one lap).

Send me the flight name after each one; I read the log before the next lift.

## 3. Log lines (T3 filter), in order

| line | expect | otherwise |
|---|---|---|
| `geometry:` | n=3 cable_len=0.550 attach_radius=0.225 load_mass=0.860 pivot_offset=[0.0, 0.0, -0.04] | Ctrl-C |
| `load yaw datum latched` | within +-45 deg | re-place the ring |
| `slot azimuth error` | all within +-10 (plates 0/4/8) | any `SLOT OFFSET` line: do not ARM |
| `arc-creep` | three drones rise together | one far behind: LAND |
| `elevation XX deg reached` | 50-60 (record it) | `NOT lifting` / timeout: LAND |
| `measure_rod_len` | each rod 0.53-0.58 | `keeping typed cable_len`: LAND |
| `pretension` | ring floats up level; record the breakaway % | tilt > 5 deg: LAND |
| `solve status` | none | the 3rd one: LAND |
| `near its bound` (H2, C1) | none | finish the hold, LAND, no C1 |

## 4. Stop rules

- **LAND** on:
  - a magnet release;
  - ring tilt above 5 deg in the pretension, or above 12 deg in flight (T5);
  - a visible ring run-up;
  - 3 solve failures;
  - a creep timeout.
- **DISARM** (panel, or SPACE in RViz) if:
  - a drone tips or climbs away;
  - anyone enters the cage;
  - during LAND, after "load down, rods slack", a drone leans or slides outward on its rod.

  From the CLI: `ros2 topic pub -r 10 -t 10 /fleet/command std_msgs/msg/String "{data: ESTOP}"`
- One release: no further lift until the log is read. Two releases: done for tonight.
- The same bar failing twice: stop and report.
- Handover above 60 deg on two flights: no C1.
- After a stop: Ctrl-C T2 and T4, then `tools/clean_slate.sh --rig`. If T1 misbehaves, Ctrl-C everything, run `tools/clean_slate.sh`, and relaunch T1.
- After a drop: props off, check the pivots and magnets.

# Rig 8 Oct: constant-speed figure-8, unannounced detach

Measured cable tension throughout. Background: `docs/experiments/2026-10-07_unannounced_detach.md`.
Overnight changes and what is live: `docs/overnight_2026-10-08_progress.md`.

## Conventions (new since 7 Oct)

Plates count CLOCKWISE 0-11 from the ring's +x, like a clock. RViz labels drones 1-4.

| drone (airframe, RViz, T2) | topic index | detach runs: plate | azimuth (deg, anticlockwise from +x) |
|---|---|---|---|
| 1 (QUAD1) | `/drone_0` | 3 | 270 |
| 2 | `/drone_1` | 11 | 30 |
| 3 | `/drone_2` | 7 (r305: 6) | 150 (r305: 180) |
| 4 (leaves) | `/drone_3` | 9 | 90 |

The old 1/3/5/9 is now 11/9/7/3; the old 1/3/6/9 is now 11/9/6/3; the even ring 0/3/6/9 keeps the same plates.

## Setup

```bash
cd ~/multi_drone_control; mdc
colcon build --symlink-install --packages-select bringup mpc_planner tracker fleet_manager dissipative_planner simulation_communication drone_communication utility_objects && source install/setup.bash
python3 tools/prebuild_planner.py 2 3 4 --load-mass 0.86 --drone-mass 0.55 --cable-len 0.55
rio num_drones:=4 magnet_channel:=6 magnet_initial:=ON          # T1, all session
```
- Packs at 23.6 V or more. Mats under every drone.
- Each run: launch in T2 (the run name goes LAST; a repeat of a name gets `_2`, `_3`), preflight in
  T3, then `fleet arm`, wait for `ARM sequence complete: all 4 armed, ready for TAKEOFF.`, then
  `fleet takeoff`.
- Preflight: carry runs `--planner mpc_planner`; detach runs `--planner dissipative_planner --detach`:
  ```bash
  python3 tools/preflight.py --drones 4 --real --planner dissipative_planner --detach --max-ground-z 0.5 --battery-min 23.6
  ```
- **ARM FAILED: drone N not armed:** TAKEOFF is refused and the fleet disarms, but T2 stays up.
  Replug drone N's battery, wait for its telemetry, then `fleet arm` again. No relaunch needed.
- QUAD2 `pending`: power-cycle it.

## Runs

**r300: props-off arm check** (new arm gate, first thing).
- Props off. Launch r301's line with the name `r300`.
- `fleet arm`: expect `ARM sequence complete`, with all four motors idling.
- `fleet disarm`, then `fleet arm` again without relaunching; expect the same line.
- `fleet disarm`, Ctrl-C.

**r301: figure-8 at 0.2 m/s.**
- Even ring, plates 0/3/6/9; keep 2.2 m clear along x.
- Lap 35 s. It lands itself after the lap; then `fleet land` to disarm.
```bash
tools/rig_flight.sh real_control_launch.py mode:=mpc num_drones:=4 target_z:=1.0 lift_ramp_vel:=0.15 load_traj:=fig_8 traj_radius:=1.0 traj_speed:=0.2 params_file:=$HOME/multi_drone_control/configs/rig/traj_hold.yaml r301
```

**r302 (OPTIONAL, your call): r301 again with `ref_time_shift`.** This is the lag fix's first rung,
an existing tracker switch.
- Each tracker flies the plan advanced by the time since it arrived, instead of holding node 0 until
  the next plan.
- Twin, figure-8 0.2 m/s, week4 code (lab PC):

  | | D | lag on one clock | tilt max |
  |---|---|---|---|
  | base (R1127, R1154) | 0.094-0.096 s | 0.104-0.106 s | 1.5-1.7 deg |
  | ref_time_shift (R1176, R1177) | 0.043-0.044 s | -0.005 to 0.001 s | 1.1 deg |

  Both landed.
- On the rig the effect is unknown: the twin's latency-to-lag relation does not fit both 7 Oct rig
  runs. r302 against r301 measures it.
- The twin tilt numbers are over the path; over the whole flight, lift included, there is no
  difference.
- Fly it straight after r301 on the same pack, so the two compare.
- Check in T3 before ARM: `ros2 param get /tracker_0 ref_time_shift` must print True.
```bash
tools/rig_flight.sh real_control_launch.py mode:=mpc num_drones:=4 target_z:=1.0 lift_ramp_vel:=0.15 load_traj:=fig_8 traj_radius:=1.0 traj_speed:=0.2 params_file:=$HOME/multi_drone_control/configs/rig/traj_hold_rts.yaml r302
```

**r303, r304: unannounced detach on 11/9/7/3, twice.**
- Plates as in the table. Before takeoff, T2's `slot azimuth error` line must show `d4 ... (plate 9)`.
- Keep 1.3 m clear on +y: drone 4 steps 0.5 m out, and on LAND it moves to 1.2 m before descending.
```bash
tools/rig_flight.sh real_control_launch.py mode:=dissipative num_drones:=4 target_z:=1.0 lift_ramp_vel:=0.15 attach_azimuths_deg:=30,90,150,270 params_file:=$HOME/multi_drone_control/configs/rig/detach_unannounced.yaml r303
```
- Hover steady for 10 s, then switch drone 4's magnet off without telling the planner:
  `ros2 topic pub -t 3 /drone_3/magnet std_msgs/msg/String "{data: 'OFF'}"`
- Expect in T2 within ~0.5 s: `UNANNOUNCED DETACH: drone 4 (/drone_3)`. Drone 4 then steps 0.5 m out.
- `fleet land` ~8 s later.
- No `UNANNOUNCED` line within 2 s: send `ros2 topic pub -t 3 /fleet/detach std_msgs/msg/Int32 "{data: 3}"`,
  then land.

**r305: unannounced detach on 11/9/6/3** (only if r303 and r304 went well).
- Drone 3 on plate 6 instead of 7; `attach_azimuths_deg:=30,90,180,270`; otherwise as r303.
- Twin peak 13.9-19.2 deg; r0013 capsized on this layout.

**r306: unannounced detach on 11/9/7/3 during a 0.125 m/s circle.** The twin passed twice overnight:
detection 0.21-0.27 s, tilt peak 8.7-9.3 deg, freed drone down 1.45 m from the circle centre.
- Place the ring 0.5 m toward -y of the cage centre: the circle (r 0.5) is then centred in the cage.
- 5 s hover hold, then the circle. Release as in r303, after 10 s of steady circling.
- Drone 4 steps 0.5 m out from the ring, then out to 1.5 m from the circle centre, then lands while
  the others fly on. Keep a 1.6 m radius around the cage centre clear of people.
```bash
tools/rig_flight.sh real_control_launch.py mode:=dissipative num_drones:=4 target_z:=1.0 lift_ramp_vel:=0.15 attach_azimuths_deg:=30,90,150,270 load_traj:=orbit traj_radius:=0.5 traj_speed:=0.125 params_file:=$HOME/multi_drone_control/configs/rig/detach_unannounced_orbit.yaml r306
```

## Stop rules

Restored overnight (they were removed from the 7 Oct version).
- Ring tilt above 15 deg in any detach: no more detaches today.
- A magnet slipping (a drone off its plate without a detach): land, stop.
- A second drone lost, or the ring tilting past 30 deg: disarm (today's code still disarms the
  fleet; drop-and-land is not live yet).
- Packs below 23.6 V.

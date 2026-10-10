# Rig Wed 14 Oct (DRAFT, 9 Oct: every line is checked at Tuesday's desk rehearsal against fake_mocap)

Plates clockwise 0-11 from +x. M1 layout: drone 1 (QUAD1, `/drone_0`) plate 3, drone 2 plate 11, drone 3 plate 7;
QUAD4 (`/drone_3`, body 14, tip body 24) leaves from / welds onto plate 9 (ring +y). Ring body 8.

## Setup

```bash
cd ~/multi_drone_control; mdc
colcon build --symlink-install --packages-select bringup mpc_planner tracker fleet_manager dissipative_planner drone_magnet simulation_communication drone_communication utility_objects && source install/setup.bash
python3 tools/prebuild_planner.py 2 3 4 --load-mass 0.86 --drone-mass 0.55 --cable-len 0.55
C=$HOME/multi_drone_control/configs/rig
R="tools/rig_flight.sh real_control_launch.py num_drones:=4 target_z:=1.0 lift_ramp_vel:=0.15"
D="mode:=dissipative attach_azimuths_deg:=30,90,150,270"
O="load_traj:=orbit traj_radius:=0.5 traj_speed:=0.125"
A="tools/rig_flight.sh real_control_launch.py mode:=attach target_z:=1.0 lift_ramp_vel:=0.15 enable_approach_mpc:=false mocap_magnet_tip_body_id:=24 params_file:=$C/attach_m1.yaml"
```

Carry/detach runs (W): T1 `rio num_drones:=4 magnet_channel:=6 magnet_initial:=ON` all the time they fly.
Attach runs (R8): stop T1 first (mode:=attach starts its own radios and mocap; rig_flight refuses otherwise).
Each run: T2 line -> T3 preflight -> `fleet arm` -> `ARM sequence complete` -> `fleet takeoff`.
```bash
python3 tools/preflight.py --drones 4 --real --planner dissipative_planner --detach --max-ground-z 0.5 --battery-min 23.6   # W runs
python3 tools/preflight.py --drones 3 --attach --real --planner dissipative_planner --max-ground-z 0.5 --battery-min 23.6  # R8 runs
```

## 09:00, props off, packs charging

1. Motive: ring frame (origin, +x plate 0, +y plate 9), bodies 11-14, 8, 24, 6 streaming.
2. Mocap survey: carry QUAD4 by hand along the approach (climb, transit, descent over plate 9): no pose gap over
   0.1 s on 14, 24 or 8.
3. **Tip calibration** (card 2026-10-09_approach_tip_feedback; Wesley 9 Oct): tether hanging, magnet face held
   ON plate 9, 5 s still:
   ```bash
   python3 tools/tip_calibration.py --seconds 5 --standoff 0.0
   ```
   Then the pull-in gap: lift the face slowly until the magnet lets go of a small steel test piece (or the ring
   follows): gap g. Run again with `--standoff <g - 0.015, not below 0>` and write the printed `attach_tip_dz` and
   `weld_point_z` into `configs/rig/attach_m1.yaml` (Wesley signs the two numbers). The standoff must be at
   least 1 cm: the tip loop overshoots up to ~9 mm below its setpoint in the twin (reviewer 9 Oct), and its contact
   freeze is not a protection to rely on (it latched with no contact in 2 of 6 twin runs). The weld radius for R8b comes
   from the same reading (default 0.025).
4. **Tip record:** the same command with QUAD4 still in the air by hand for 10 s, then again with body 24
   covered for 3 s: read the repeats / smallest step / longest gap lines (the loop calls a tip bit-identical for
   0.2 s frozen; a coarse forwarder would show repeats here).
5. Kill path (RViz DISARM, T3 `fleet disarm`, Ctrl-C), radio-loss failsafe, QUAD1/QUAD4 Betaflight status.
6. r400: props off, the W4 line named `r400`: `fleet arm` -> ARM complete -> `fleet disarm` -> `fleet arm` again
   -> `fleet disarm`, Ctrl-C.

## Flights, in order

```bash
$R $D params_file:=$C/drop_and_land_gi.yaml w4                          # hover detach, unannounced, freed drone lands + disarmed alone
$R $D $O params_file:=$C/detach_unannounced_orbit_drop_gi.yaml w5       # circle detach, same
$A weld_radius:=0 r8a0                                                  # carry on 3, QUAD4 armed idle on the floor
$A weld_radius:=0 r8a                                                   # dry approach (no weld), LAND during the hold
$A weld_radius:=0 r8a                                                   # dry approach, OFF abort during the hold
$A weld_radius:=<from 09:00, default 0.025> r8b                         # weld, x2-3
```

- **w4 / w5:** release as on 8 Oct (`ros2 topic pub -t 3 /drone_3/magnet std_msgs/msg/String "{data: 'OFF'}"`
  after 10 s steady). Expect UNANNOUNCED DETACH, drone 4 out 0.5 m, down by itself, then in T2 within ~1 s of
  `[Drone 4] down at its rest off the ring`: `Drone 4 is down at its rest off the ring: disarmed alone.`, and its
  elrs_interface / fc_arm_state reading disarmed while the others fly on (card 2026-10-09_departed_disarm_alone,
  rig bar). If it stays above its rest (rod on the floor), it is held there and the fleet disarm takes it.
- **r8a0:** ring 0.5 m toward -y of the cage centre; QUAD4 on its mat 1.2 m out on +y; 30 s hover, `fleet land`.
  QUAD4 stays down and armed idle (never flew).
- **r8a (dry):** after 10 s steady: `ros2 topic pub -t 3 /magnet/command std_msgs/msg/String "{data: 'ON'}"`.
  QUAD4 climbs, transits, descends over plate 9 and holds; its magnet stays latched OFF (weld 0). The tip loop is
  on in attach_m1.yaml for r8a too: a no-weld preview of R8b's loop (Wesley's decision is "on from the first R8b";
  the preview was said in chat 9 Oct; for r8a with the loop off add `approach_tip_feedback:=` via a params file
  copy). T2 shows `approach: drone 3 tip hold (tip +0.0xx m from its setpoint, + above)`; tip.csv in the run's
  logs shows where it parks.
  After 15 s of hold: first r8a `fleet land` (QUAD4 steps 1.2 m out at its height and lands by itself, disarmed
  alone; the carriers land the ring); second r8a `ros2 topic pub -t 3 /magnet/command std_msgs/msg/String "{data: 'OFF'}"`
  (the abort: QUAD4 out and down, the carriers fly on), then `fleet land`.
- **Decision point (after r8a):** the parked tip height from tip.csv against the 09:00 setpoint; the weld radius.
- **r8b:** ON as above; expect the weld (`ATTACH drone 3 (OCP resize)`), the reconfiguration hold, four carry;
  15 s later `fleet land`.

## Stop rules

- 8 Oct rules for W4/W5 (a drop, ring tilt over 30 deg, a second loss: no more detaches today).
- R8a: QUAD4's magnet latch ON; its rod near a carrier; ring over 15 deg; QUAD4 over 25 deg -> `fleet land`.
- R8b: caught but no weld within 1 s -> `/magnet/command OFF`; a weld with nothing caught -> `/fleet/detach 3`;
  ring over 15 deg -> `fleet land`. A QUAD4 fault over the plate is accepted (Wesley 9 Oct; twin ring ~57 deg).
- Any hand on a drone or any DISARM in attach: relaunch T2. A pose gap over 0.1 s on 14/24/8 with QUAD4 overhead
  ends M1 for the day. Packs below 23.6 V.

## To check at the desk rehearsal (Tuesday)

The tree: `git log -1` on week4 (b4e11f1 or later; rig-prep = week4 since 10 Oct, check runs R1454-R1460).
The newcomer defaults on: `ros2 param get /dissipative_planner` newcomer_lifecycle / newcomer_fault_land / departed_ground_idle -> True
(on by default since 9 Oct; attach_m1.yaml also sets them).
The attach preflight line above (expects the newcomer as drone 3); QUAD4's serial and magnet channel in
rig_io; `ros2 param get /dissipative_planner approach_tip_feedback` -> True and `attach_tip_dz` as written;
`/magnet_attachment_manager object_z_offset` = weld_point_z; require_fc_armed True; the departed / down_alone /
on_load topics in the bag; DISARM from RViz and T3 in the same DDS domain.

# Rig Wed 14 Oct (DRAFT; every line is checked at Tuesday's desk rehearsal against fake_mocap)

10 Oct (Wesley): on the rig Tejen's controller does every QUAD4 attach, down to contact (the magnet only catches
on the plate); our motion-capture check alone confirms it, then we take QUAD4 over and fold it in. Our own approach
rungs R8a/R8b leave this sheet. Wednesday: W4, W5, R8a0, and our attach check tested props off at 09:00.

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
3. **Contact height** (for our attach check): tether hanging, magnet face held ON plate 9 (caught), 5 s still:
   ```bash
   python3 tools/tip_calibration.py --seconds 5 --standoff 0.0
   ```
   Write the printed `weld_point_z` (the tip marker's height with the face on the plate) into
   `configs/rig/attach_m1.yaml` (Wesley signs it). `attach_tip_dz` stays 0: our approach is not flown on the rig.
4. **Tip record:** the same command with QUAD4 still in the air by hand for 10 s, then again with body 24
   covered for 3 s: read the repeats / smallest step / longest gap lines (the loop calls a tip bit-identical for
   0.2 s frozen; a coarse forwarder would show repeats here). Note the `body 14 to tip 24` line from the in-air
   record (tether hanging): the taut length, the reference of the combined M1 lift proof.
5. Kill path (RViz DISARM, T3 `fleet disarm`, Ctrl-C), radio-loss failsafe, QUAD1/QUAD4 Betaflight status.
6. r400: props off, the W4 line named `r400`: `fleet arm` -> ARM complete -> `fleet disarm` -> `fleet arm` again
   -> `fleet disarm`, Ctrl-C.
7. **Attach check, props off** (T1 stopped; ring on the floor; QUAD4 disarmed, props off, in hand with its pack in so
   the magnet switches): `$A weld_radius:=0.01 chk`, then
   `ros2 topic pub -t 3 /magnet/command std_msgs/msg/String "{data: 'ON'}"`.
   (a) Hold the magnet face 2 cm above plate 9 for 5 s: no `ATTACH:` line from magnet_attachment_manager.
   (b) Put the face on the plate (it catches) and hold still 2 s: `ATTACH: d=0.0xx m, rel_speed=...` and
   `/magnet/object_attached` true. Note d: a catch off the plate's centre reads larger, and d sets the radius
   for the combined runs. No ATTACH within 2 s (a catch more than 1 cm off the calibration point): read d from
   `ros2 topic echo --once /magnet/attachment_state` (`distance=`), and repeat with the catch moved around the
   plate to get the spread. (c) Slide the ring 5 cm with QUAD4 on it: no change. Then OFF, Ctrl-C, relaunch.
   T2 shows `attach ignored - not flying yet` every 2 s while attached: expected (the planner is on the ground).
   Motion-capture noise is not a risk for the 0.05 m/s gate: still bodies read 0.01-0.02 m/s at most (8 Oct, r305).

## Flights, in order

```bash
$R $D params_file:=$C/drop_and_land_gi.yaml w4                          # hover detach, unannounced, freed drone lands + disarmed alone
$R $D $O params_file:=$C/detach_unannounced_orbit_drop_gi.yaml w5       # circle detach, same
$A weld_radius:=0 r8a0                                                  # carry on 3, QUAD4 armed idle on the floor
```

- **w4 / w5:** release as on 8 Oct (`ros2 topic pub -t 3 /drone_3/magnet std_msgs/msg/String "{data: 'OFF'}"`
  after 10 s steady). Expect UNANNOUNCED DETACH, drone 4 out 0.5 m, down by itself, then in T2 within ~1 s of
  `[Drone 4] down at its rest off the ring`: `Drone 4 is down at its rest off the ring: disarmed alone.`, and its
  elrs_interface / fc_arm_state reading disarmed while the others fly on (card 2026-10-09_departed_disarm_alone,
  rig bar). If it stays above its rest (rod on the floor), it is held there and the fleet disarm takes it.
- **r8a0:** ring 0.5 m toward -y of the cage centre; QUAD4 on its mat 1.2 m out on +y; 30 s hover, `fleet land`.
  QUAD4 stays down and armed idle (never flew).

## Stop rules

- 8 Oct rules for W4/W5 (a drop, ring tilt over 30 deg, a second loss: no more detaches today).
- R8a0: QUAD4 leaves the floor; ring over 15 deg -> `fleet land`.
- Any hand on a drone or any DISARM in attach: relaunch T2. A pose gap over 0.1 s on 14/24/8 with QUAD4 overhead
  ends M1 for the day. Packs below 23.6 V.

## To check at the desk rehearsal (Tuesday)

The tree: `git log -1` on week4 (b4e11f1 or later; rig-prep = week4 since 10 Oct, check runs R1454-R1460).
The newcomer defaults on: `ros2 param get /dissipative_planner` newcomer_lifecycle / newcomer_fault_land / departed_ground_idle -> True
(on by default since 9 Oct; attach_m1.yaml also sets them).
The attach preflight line above (expects the newcomer as drone 3); QUAD4's serial and magnet channel in
rig_io; require_fc_armed True; the departed / down_alone / on_load topics in the bag; DISARM from RViz and T3
in the same DDS domain.

The 09:00 attach check against fake_mocap (dry-run 10 Oct on rig-prep: all three pass, ATTACH 0.17 s after the
tip reaches the plate). T1 fakes the ring and QUAD4 hanging over plate 9; T2 is the chk line with `real_io:=false`:
```bash
F="python3 tools/fake_mocap.py --rig --num-drones 3 --attach --tip --azimuths-deg 270,30,150 --drone-z 0.08 --tip-plate 9"
$F --tip-dz 0.02                    # T1, (a): tip 2 cm above the weld point (weld_point_z 0 until 09:00)
ros2 launch bringup real_control_launch.py mode:=attach target_z:=1.0 lift_ramp_vel:=0.15 enable_approach_mpc:=false \
  mocap_magnet_tip_body_id:=24 params_file:=$C/attach_m1.yaml weld_radius:=0.01 real_io:=false        # T2
ros2 param get /magnet_attachment_manager attach_radius      # 0.01; object_z_offset = weld_point_z; object_y_offset 0.225
ros2 topic pub -t 3 /magnet/command std_msgs/msg/String "{data: 'ON'}"     # then 5 s: no ATTACH
# Ctrl-C T1, then $F --tip-dz 0.0 (b): ATTACH d=0.000 and /magnet/object_attached true
```

## PROPOSED 11 Oct (for Wesley's review; not on the sheet until he says)

Drafted 11 Oct 17:16. One row per item, in his run order: where it goes, what to do, what to record.
Sources:
- thesis plan D5, D6, P5;
- rig-port critic F8 (the QUAD4 record);
- critic round 4 F3, F8;
- tilt card rev 4 (a)-(f);
- the thesis health check;
- docs/freeze_2026-10-14.md (Q15).

Part B rows run on the pinned m1-rig 0210a27 install, in a fresh terminal, after the W flights.

| # | where | do | record |
|---|---|---|---|
| 1 | 09:00 item 1 | Body-rate fix live (96acbcf: in week4 and m1-rig, never flown): rock QUAD1 by hand, `ros2 topic echo --once /drone_0/motion_capture_state` | angular_velocity finer than 0.1 rad/s steps: yes/no |
| 2 | 09:00 item 2 | In the survey, remember: any of our flying drones' poses older than 0.25 s disarms the whole fleet (rig pose_timeout_s) | longest gap per body (s) |
| 3 | 09:00 item 2 | Wand walk: QUAD4 (14, 24) on a pole at ~1.9 m (the take-back rise height), along the plate-9 corridor out to 1.2 m from the ring centre, then round 1.5 m from the circle centre (0.5 m +y of the ring's start) on the +y half | longest gap on 14 and 24 (s; bar 0.1) |
| 4 | 09:00 item 2 | Tip at plate 9: carriers on their plates, tip (24) held on plate 9 and moved within 10 cm of it (our contact check reads the tip) | longest gap on 24 (s) |
| 5 | 09:00 item 3 | weld_point_z three times, the magnet re-seated each time | 3 values (m) |
| 6 | 09:00 item 4 | QUAD4 hanging still with a ~2 N pull on the caught magnet; then the face flat on the floor | `body 14 to tip 24` = taut length (m); tip z with the face on the floor = marker to face (m) |
| 7 | 09:00 item 4 | d_ok noise: face caught on plate 9, QUAD4 rocked 0-20 deg by hand | spread of the tip's drift from its contact point (mm) |
| 8 | 09:00 item 7(b) | At least 10 hand catches: centre, +-2 cm radial and tangential, tilted +-15 deg, tether slack; `/magnet/attachment_state` after each | d and dz per catch; radius = max dxy + 5 mm, band = max dz + 3 mm |
| 9 | 09:00, after item 7 | Magnet hold: a carrier's magnet ON (pack in), face caught on a plate, ring held down; pull through the rod with a spring scale at 0 / 30 / 45 / 60 deg from the plate's axis, 3 pulls each | N at release, per pull |
| 10 | 09:00 | A carrier's prop tip, measured from the body centre (the twin's overflight uses 0.2 m) | m |
| 11 | 09:00 | QUAD4's mat centre and the puck (body 6) in Motive, ring at its start spot | xy (m); distance of each from the circle centre (swept edge 1.2 m, 1.4 m with QUAD4's rotor; twin: mat 0.90, puck 1.49) |
| 12 | Flights, every flight | Phone on a tripod, the whole cage in frame, from before ARM to after the disarm | file name per run name |
| 13 | Flights, every flight | Packs and time (flights so far ~1 min; the full M1 needs ~2.5 min) | each pack's V at ARM and at LANDED; TAKEOFF to LANDED (s) |
| 14 | w4, w5 | Release at the twin's timing (w4: 6-7 s after the ring reaches 1.0 m; w5: ~10 s after the circle starts), hold 15-20 s on three drones, then `fleet land` (freeze Q15, D5) | release and LAND times (s) |
| 15 | Part B | Hand ramps of the face onto plate 9 at ~3, 5 and 8 cm/s, 3 each; contact_still is on by default (type nothing) | c0 against the rest point per ramp (mm); pass within 1 mm |
| 16 | Part B | Tilt (a): bag 14 and 24 with QUAD4 on a pole at 2.3-2.9 m, over and beside the ring, puck on; replay later with `tools/tilt_replay.py` (m1-comove) | bag name |
| 17 | Part B | Tilt (b): tumble QUAD4 by hand (disarmed) through 90 deg, fast | any attitude step over 3000 deg/s: yes/no; max rate |
| 18 | Part B, his OK (his stream) | Tilt (c): arm props-off through the mux, then the fleet DISARM | abort, elrs disarmed, FC '*' telemetry: times (s) |
| 19 | Part B | Tilt (d): power off QUAD4's RX (or pull its pack) while armed | fc_arm_state still 'armed' after 10 s: yes/no; how long |
| 20 | Part B | Tilt (e): coil ON with the puck, then DISARM | the coil holds the puck: yes/no |
| 21 | Part B | Tilt (f): QUAD4 held at 0 / 30 / 60 / 90 deg. week4 and m1-rig have no `fc_attitude` topic: read the `Attitude:` lines elrs_interface prints in T1 | line rate (Hz); FC minus mocap per angle (deg) |
| 22 | Stop rules: every partner:=true run until 28-29 Oct | After LANDED, if drone 4's mux still reads partner (T2: 'taken back, but our tracker is not flying yet'), clear the cage and press DISARM before anyone walks in (it latches every mux; QUAD4 falls) | pressed: yes/no; QUAD4's height (m) |
| 23 | Any combined circle day (not Wed 14) | His TAKEOFF comes before our orbit starts (a rig traj_hold_s long enough). Twin R1625: with his takeoff after the orbit start, the ring passed 0.12 m above QUAD4's body, 0.23 m from it in xy | his TAKEOFF and our orbit start (s) |

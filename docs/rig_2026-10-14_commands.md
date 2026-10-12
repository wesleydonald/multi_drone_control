# Rig Wed 14 Oct: try everything (tree week5)

**Aim:** fly every item once today, simplest first, so anything that breaks gets fixed for Thursday 15:00-18:00
(retries) or 21-22 Oct. A failed item stops only the items that depend on it.

**Layout.** Plates clockwise 0-11 from +x. Ring body 8.
- Carry flights (R1-R3): even ring.
- Detach and combined flights:
  - QUAD1 (`/drone_0`) plate 3, QUAD2 plate 11, QUAD3 plate 7;
  - QUAD4 (`/drone_3`, body 14, tip body 24) on plate 9 (ring +y);
  - the puck is body 6.

**Order:** P1-P12 props off (09:00, packs charging) -> R1-R5 -> P13 load test -> P14 R9 prelude -> R6-R9 with Tejen.

**Packs:** ___ charged per drone (count them before 09:00). The day is about 19 min of carrier flight and 15 min for QUAD4.
- Charge between the morning block and R1, and during P13.
- Swap all four packs before R6 and again before R9, whatever they read.
- If there are fewer than three sets, drop R2/R3 first (Thursday can fly them), not R4/R5.

## Setup (every terminal, RViz too: `w5`, never `mdc`)

`mdc` sources the main tree, which is not what flies today. The combined launch refuses to start without the two
exports, so they are in `w5` all day.

```bash
alias w5='source ~/ros2_humble/install/setup.bash && cd ~/mdc_wt/week5 && source install/setup.bash && export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=23'
w5
fleet() { python3 ~/mdc_wt/week5/tools/fleet_reach.py --send "$1"; }   # refuses unless the fleet manager answers
git log --oneline -1                     # 46ac389 or later on week5
colcon build --symlink-install --packages-select bringup mpc_planner tracker fleet_manager dissipative_planner drone_magnet simulation_communication drone_communication utility_objects && source install/setup.bash
python3 tools/prebuild_planner.py 2 3 4 --load-mass 0.86 --drone-mass 0.55 --cable-len 0.55
C=$HOME/mdc_wt/week5/configs/rig
W=0.031; L=0.58; R=0.012      # PLACEHOLDERS: type P3's W, P4's L, P7's R as you get them
R="tools/rig_flight.sh real_control_launch.py num_drones:=4 target_z:=1.0 lift_ramp_vel:=0.15"
D="mode:=dissipative attach_azimuths_deg:=30,90,150,270"
O="load_traj:=orbit traj_radius:=0.5 traj_speed:=0.125"
A="tools/rig_flight.sh real_control_launch.py mode:=attach target_z:=1.0 lift_ramp_vel:=0.15 enable_approach_mpc:=false mocap_magnet_tip_body_id:=24 params_file:=$C/attach_m1.yaml"
```

- **P1-P12 and R1-R5:** T1 stays up all the time. It streams the tip (24) and the puck (6) too: without them P3, P4
  and P10 have nothing to read.
  ```bash
  rio num_drones:=4 magnet_channel:=6 magnet_initial:=ON mocap_magnet_tip_body_id:=24 mocap_extra_bodies:=6:/pickup/motion_capture_state
  ```
- **R6-R9:** stop T1 first: `mode:=attach` starts its own radios and mocap.
- **Each flight:** T2 line -> T3 preflight -> `fleet arm` -> `ARM sequence complete` -> `fleet takeoff`.

```bash
python3 tools/preflight.py --drones 4 --real --planner mpc_planner --max-ground-z 0.5 --battery-min 23.6                  # R1-R3
python3 tools/preflight.py --drones 4 --real --planner dissipative_planner --detach --max-ground-z 0.5 --battery-min 23.6  # R4-R5
python3 tools/preflight.py --drones 3 --attach --real --planner dissipative_planner --max-ground-z 0.5 --battery-min 23.6 # R6-R9
```

**Every flight:**
- phone on a tripod, the whole cage in frame, from before ARM to after the disarm;
- record pack V at ARM and at LANDED, and TAKEOFF to LANDED (s).

## Props off (09:00)

- **P1 Motive.** Ring frame: origin, +x plate 0, +y plate 9. Bodies 11-14, 8, 24 and 6 streaming.
  - Body-rate check: rock QUAD1 by hand, then `ros2 topic echo --once /drone_0/motion_capture_state`.
  - Record whether the angular velocity is finer than 0.1 rad/s steps.
- **P1b Still bag.** Ring on the floor, carriers on their plates, QUAD4 on its mat, puck in place, nobody moving: the
  pose noise of every body (the thesis's noise sentence and the proof's noise band).
  `timeout -s INT 60 ros2 bag record -o results/rig/2026-10-14/p1b /payload/motion_capture_state /drone_0/motion_capture_state /drone_1/motion_capture_state /drone_2/motion_capture_state /drone_3/motion_capture_state /magnet_tip_pose /pickup/motion_capture_state`
- **P2 Mocap survey.** Record the longest gap on 14, 24 and 8 (bar 0.1 s; a gap over 0.25 s on a flying drone
  disarms the fleet).
  - Carry QUAD4 by hand along the climb, the transit and the descent over plate 9.
  - QUAD4 on a pole at ~1.9 m: along the plate-9 corridor to 1.2 m out, then round the +y half at 1.5 m.
  - Tip 24 on plate 9, then within 10 cm of it, with the carriers on their plates.
- **P3 Contact height.** Tether hanging, magnet face caught on plate 9, 5 s still. Do it 3 times, re-seating the
  magnet each time. **W** = the mean of the printed `weld_point_z`.
  ```bash
  python3 tools/tip_calibration.py --seconds 5 --standoff 0.0
  ```
- **P4 Tip record.** The same command as P3:
  - QUAD4 still in the air by hand for 10 s, then with body 24 covered for 3 s. Read the repeats and gaps lines.
  - **L** = `body 14 to tip 24`, QUAD4 hanging still with a ~2 N pull on the caught magnet (the taut length).
  - Marker to face: the face flat on the floor; record the tip's z.
  - d_ok noise: face caught, rock QUAD4 0-20 deg by hand; record the tip's drift spread (mm).
  - Slack: record the tip's z hanging, then under the ~2 N pull. The difference is the tether's slack.
- **P5 Kill path.** RViz DISARM, T3 `fleet disarm`, Ctrl-C, the radio-loss failsafe, and the Betaflight status of QUAD1
  and QUAD4.
- **P6 Arm check.** Using R4's line named `p6`: `fleet arm` -> complete -> `fleet disarm`, twice, then Ctrl-C. If the
  second ARM is refused, relaunch the line and count that as the second cycle.
- **P7 Attach check and hand catches** (after P3: it needs W). T1 stopped, ring on the floor, QUAD4 disarmed with its
  pack in. Put W into `configs/rig/attach_m1.yaml` too (yours to sign), so a line that forgets it fails loudly.
  ```bash
  $A weld_radius:=0.01 weld_point_z:=$W p7
  ros2 topic pub -t 3 /magnet/command std_msgs/msg/String "{data: 'ON'}"
  ```
  - (a) Face 2 cm above plate 9 for 5 s: no `ATTACH:` line.
  - (b) Face on the plate, still for 2 s: `ATTACH: d=...` and `/magnet/object_attached` true.
  - (c) Slide the ring 5 cm with QUAD4 on it: no change.
  - Then at least 10 catches: centre, ±2 cm radial, ±2 cm tangential, tilted ±15 deg. Record d and dz per catch from
    `/magnet/attachment_state`.
  - **R** = max dxy + 5 mm (the contact radius).
  - Extra: 3 catches with the ring tilted ~10 deg (does the radius depend on the plate's tilt?).
- **P8 Magnet hold.** A carrier's magnet ON, face caught on a plate, ring held down. Spring scale through the rod at 0,
  30, 45, 60 and 90 deg (pure shear) from the plate's axis, 3 pulls each. Record N at release. Then the peel: with the
  tip caught, tilt the ring by hand until the magnet lets go, and note the angle. These numbers go into the thesis.
- **P9 Measures.** A carrier's prop tip radius from its body centre. QUAD4's mat centre and the puck (body 6) xy, with
  the ring at its start spot.
- **P10 Puck in the net.** Ring on a stand at ~1 m, puck in the net. `ros2 topic hz /pickup/motion_capture_state` for
  10 s: does it stream, gaps under 0.1 s?
  - Yes: R6-R9 keep the default.
  - No: R6-R9 add `drop_puck_wait_s:=0` inside partner_args, and you check the drop by eye.
- **P11 Hand ramps.** Face onto plate 9 at ~3, 5 and 8 cm/s, 3 times each. Record c0 against the rest point; pass is
  within 1 mm.
- **P12 Tilt checks** (QUAD4 disarmed):
  - (a) bag 14 and 24 with QUAD4 on a pole at 2.3-2.9 m, over and beside the ring, puck on:
    `timeout -s INT 120 ros2 bag record -o results/rig/2026-10-14/p12a /drone_3/motion_capture_state /magnet_tip_pose /payload/motion_capture_state /pickup/motion_capture_state`;
  - (b) tumble it through 90 deg by hand, fast, with the same record named p12b; note any step over 3000 deg/s and
    the max rate;
  - (c) only with Tejen's OK: arm props-off through the mux, then the fleet DISARM; record the times of the abort,
    the elrs disarm and the FC '*' telemetry;
  - (d) power off QUAD4's RX while armed; record how long fc_arm_state still reads armed;
  - (e) coil ON with the puck, then DISARM; does the coil hold the puck?
  - (f) hold QUAD4 at 0, 30, 60 and 90 deg; record the `Attitude:` line rate and FC minus mocap per angle.

## Flights

| # | what | line (T2) | expect, record |
|---|---|---|---|
| R1 | 3-min ring hover, carry default | `$R mode:=mpc r1` | `fleet land` 3 min after the ring reaches 1.0 m. Pack V every minute. Says whether R9 (~3 min) fits a pack. |
| R2 | circle, measured pull, 60 deg plan (r206e60 repeat) | `$R mode:=mpc $O params_file:=$C/traj_hold.yaml r2` | Two laps: `fleet land` ~50 s after T2's `hover hold done` (the falsifier needs a lap-to-lap sd). 1.7 m clear on +y. |
| R3 | circle, **planned** pull, 60 deg plan | `$R mode:=mpc $O params_file:=$C/feedforward_60_hold.yaml r3` | As R2. One change from R2: the pull source. |
| R4 | hover detach, unannounced; QUAD4 lands and disarms alone (was W4) | `$R $D params_file:=$C/drop_and_land_gi.yaml r4` | Release 6-7 s after the ring stops rising at 1.0 m (RViz or by eye), never under 5 s: the metric needs 5 s of still reference before it. Hold 15-20 s on three, then `fleet land`. |
| R5 | circle detach, same (was W5) | `$R $D $O params_file:=$C/detach_unannounced_orbit_drop_gi.yaml r5` | Release ~10 s after T2's `hover hold done: orbit starts`. Hold 15-20 s on three, then `fleet land`. |

- **R3's falsifier (frozen):** if its path error is within max(1 cm, the lap-to-lap sd) of 2.91 cm, the gain came from
  the rod angle, not the measured pull.
- **R4/R5 release:**
  ```bash
  ros2 topic pub -t 3 /drone_3/magnet std_msgs/msg/String "{data: 'OFF'}"
  ```
  - Expect `UNANNOUNCED DETACH`, QUAD4 out 0.5 m and down by itself.
  - Then in T2, within ~1 s: `Drone 4 is down at its rest off the ring: disarmed alone.`
  - Record the release and LAND times.
- **R4/R5 bands** (frozen): from the week4 twin runs; week5's same-tree checks fly tonight. Read every rig value
  against the rig's no-event floor too (4.8 deg in hover, 6.3 deg on a path; 12 Oct offline analyses).

### P13 Load test (props off): the go for R6-R9

Procedure: `~/mdc_wt/week5/docs/rig_2026-10-14_load_test.md`, run from week5. The full graph runs for 10 min, with no
CLI calls inside the window. `tools/load_check.py` must print `F1 load bars: PASS`. After the window:
`ros2 topic list | grep /tejen`, and note his MPC's state topics for the record list.

### P14 R9 prelude (props off, ~5 min)

R9 has two links never run on the rig. Rehearse the second here:
1. Launch R9's line with props off: QUAD4 caught on plate 9, the ring on the floor.
2. His ARM, then lift the ring and QUAD4 by hand to ~1 m and hold 60 s.
3. Pass: T2 shows no disarm from his MPC, and `/drone_3/mux_state` stays `ours`.

If it fails, R9 waits for Thursday.

### R6-R9 with Tejen (afternoon)

**Go only if:**
- W, L and R are in hand, and L is within 1 cm of his 0.575 (else type the measured L everywhere);
- P13 passed;
- P10 has decided `drop_puck_wait_s`;
- Tejen is here and agrees.

**Before each:** nothing else running on the laptop. Wait 2 min after the T2 launch before his ARM. Ask Tejen to
keep his own logs for every flight.

**Order:** his ARM (QUAD4 idle on its mat) -> `fleet arm` -> `fleet takeoff` -> ring at 1.0 m (T2: `our planner flies
the ring`) -> his TAKEOFF. **End:** `fleet disarm`, then his DISARM.

```bash
P=$(python3 -c "print(round($L-0.003,3))"); Q=$(python3 -c "print(round($L-0.04,3))")
M="partner:=true partner_contact:=true partner_nodes:=true weld_radius:=$R proof_taut_len:=$P weld_point_z:=$W attach_cable_len:=$Q"
$A $M partner_args:="magnet_drop_below_quad:=$L mpc_thrust_ratio:=22 contact_dz:=0.05" r6      # R6
$A $M partner_args:="magnet_drop_below_quad:=$L mpc_thrust_ratio:=22" r7                       # R7, then r8
```

| # | what | expect |
|---|---|---|
| R6 | his whole mission, no-catch: he stops 5 cm above plate 9 (was R11a) | Pickup, drop into the net, he holds over plate 9 with no contact, and gives up 30 s later. Our take-back: magnet OFF, the release guard, up, out to 1.2 m, down, QUAD4 disarmed alone. Then `fleet land`. The bag gives his hold height over plate 9 and the take-back timeline. |
| R7 | the contact attach (was R11b); only after a clean R6 | Contact, a small lift proves the catch, `ATTACH`, the mux switches QUAD4 to us and folds it in. Hold 20 s on four, then `fleet land`. Record contact-to-ATTACH time, ring tilt by eye, and QUAD4 at the landing. |
| R8 | R7 again | As R7. |
| R9 | the full M1 on hover: QUAD4 starts attached, we detach it, he picks up the puck, drops it in the net and reattaches | **Last, gated: R7 and R8 clean, P14 passed, and R1 shows a pack lasts ~3 min.** First time on the rig; flies clean on hover in the twin. Camera on the pickup site and the ring together (the thesis's F1 insets). |

**R9:** `$A $M partner_attached:=true partner_args:="magnet_drop_below_quad:=$L mpc_thrust_ratio:=22" r9`
- QUAD4 starts on plate 9 with its magnet caught; the puck on the floor at its spot.
- **Order:**
  1. His ARM, on the floor. His kT then learns once he flies; it doesn't learn while we fly QUAD4.
  2. `fleet arm` -> `fleet takeoff` -> the ring at 1.0 m, 10 s steady.
  3. `ros2 topic pub -t 3 /fleet/detach std_msgs/msg/Int32 "{data: 3}"`. QUAD4's magnet goes OFF and it steps out.
  4. T2 shows the release to his stream. **Then his TAKEOFF**: the mux passes his stream once it is armed above idle.
  5. His pickup, drop and attach as in R7. Hold 20 s on four, then `fleet land`.
- **Stop:** if no release line within ~10 s of the detach, or his TAKEOFF doesn't take QUAD4 within 5 s, use `fleet land`.
  The LAND takes QUAD4 back.

## Stop rules

- **Detach (R4, R5):** a drop, ring tilt over 30 deg, or a second loss means no more detaches today.
- **Stopping his attach:**
  ```bash
  ros2 topic pub --once /join_planner/give_up std_msgs/msg/Bool "{data: true}"
  ```
  The ring stays at hover. LAND only as an emergency.
- **No more combined flights today** after a load or authority falsifier (rig card §5.1, §5.7), a RELEASE GUARD failure
  with plate 9 rising, or any fleet abort.
- **After LANDED in R6-R9:** if drone 4's mux still reads partner (T2: 'taken back, but our tracker is not flying yet'),
  clear the cage and press DISARM. Record whether it was pressed, and QUAD4's height.
- A pose gap over 0.1 s on 14, 24 or 8 with QUAD4 overhead ends the combined flights for the day.
- Packs below 23.6 V. Any hand on a drone, or any DISARM in attach mode: relaunch T2.

## If a flight fails

Note it, skip what depends on it (R7 needs R6, R8 needs R7, R9 needs R7 and R8), and carry on down the list.
Thursday 15:00-18:00 re-flies what failed, after a fix.

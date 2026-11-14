# Lab Fri 30 Oct: the first attach, the drop-off, the baselines, then 0/3/6

Written Thu 29 Oct evening and overnight (plan docs/plan_rig_fix_2026-10-30.md; Wesley ~23:55: measurements first, sq6b
first, the drop-off, 0/3/6 once the rest works, B2 skipped). Lines marked [desk ok] came up at the desk against fake
mocap with no radios and their parameters read back (section "Desk check"). The 1.15 kg ring, 1 m cables, the 5.3 Hz
notch (rig default: type nothing). Fresh packs every flight; swap at ~22 V. A line that fails moves on: never the same
failure twice.

## Today's order
| # | Magnets on | Line | What | Why now |
|---|---|---|---|---|
| A | (bench) | section A | every measurement and check, props off | the lines below type them |
| 1 | 3/11/7 + QUAD4 | sq6b | Tejen's attach, 10 cm restart with a 1.0 s debounce; our proof fixed for body 24 (G), K 0.035, his give-up 120 s | the first attach; you chose it first |
| 2 | 3/11/7 + QUAD4 | sq6 | the same with 8 cm | only if sq6b's descents go wrong |
| 3 | 3/11/7 + QUAD4 | k1_5 | the pickup and the drop into the net | the drop-off |
| 4 | 3/11/7 + QUAD4 | k1_6 | k1_5 with the wider capture | if k1_5 hovers 15 s over the object |
| 5 | 3/11/7 + QUAD4 | k1_7 | QUAD4 starts with the object on its magnet, then the drop | if the pickup itself fails |
| 6 | 3/11/7 | h3t, ls3t, f3t | baselines on three quads | results |
| - | re-hang | | QUAD1-4 on 0/3/6/9 | |
| 7 | 0/3/6/9 | h4n, ls4, f4n | baselines on four quads (the notch's first 4-quad flight) | results |
| 8 | 0/3/6 + QUAD4 | sq036 | Tejen's attach on 0/3/6, cap 0.95, short | "try 036 again" |
| 9 | 0/3/6 + QUAD4 | sq036m | sq036 on the 0.86 kg ring (section 3) | if sq036 runs out of thrust |

```bash
# every terminal, first
alias w5='source ~/ros2_humble/install/setup.bash && cd ~/multi_drone_control && source install/setup.bash && export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=23 MDC_LEAN_RCLPY=1 MDC_PLANNER_CPUS=0-7 PYTHONPATH=$HOME/multi_drone_control/tools/lean_rclpy:$PYTHONPATH'
```

## A. First, before any flight (props off, ~20 min)
QUAD1-4 on plates 0/3/6/9 (drone 0-3, bodies 11-14, markers 21-24); the object is Motive body 6.
1. **The object's thickness** with a ruler: `PT`. Measured Fri 08:05: **0.010 m** (the twin's plate is 25 mm, built from 21 Oct's estimate).
2. **QUAD4's magnet markers (body 24):** add the markers, redefine rigid body 24 with them (keep ID 24), and put its
   pivot where the old marker was: on the magnet's axis, **22.5 mm above the face** (Tejen's irl_commissioning.yaml
   `magnet_marker_to_contact_face_z: -0.0225`; corrected Fri 08:10, the sheet said "the face centre"). Check: with the
   face flat on the ground, body 24 reads z +0.0225 (±2 mm). Then hold QUAD4 ~1 m over
   plate 9 and move it slowly ±10 cm: body 24 must never drop out in Motive. (29 Oct: it froze ~0.2 s at a time in
   every attach flight, 13-18 % of samples; 3 of sq5_2's 6 descent restarts came on the jump after a freeze.)
3. **The object's contact height** (after step 2; Tejen's pickup aims at it):
   ```bash
   ros2 launch tejen_mission m1_irl_pickup.launch.py measure:=true quad_body_id:=14 magnet_body_id:=24   # T1: his mocap only, no radio
   python3 tools/pickup_geometry.py          # T2, QUAD4's magnet face resting flat on the object's centre, magnet ON
   ```
   Its `contact_height` is `PC` (21 Oct with QUAD1's marker: 0.018). Ctrl-C T1. Without it the combined launch flies
   Tejen's commissioning puck (86 x 86 x 22 mm, 12 mm over its marker, ~1.7 cm high for the real object).
4. **The ring geometry (QUAD4's plate 9):**
   ```bash
   tools/t1_four_markers.sh detach:=true      # T1
   python3 tools/payload_geometry.py           # T2: ring rest z ~0.013, markers ~0.20 m on plates 0, 3, 6, 9
   ```
   drone_3's line gives `X`, `Y`, `W` (29 Oct: -0.005, +0.197, +0.028). Then hold QUAD4 ~1.1 m above its caught
   magnet, cable taut, and run it again: drone_3's "body to marker" is `L` (29 Oct 1.041).
5. Each MAGNET row in RViz: OFF then ON.
5b. **The catch spread on plate 9 (sets K):** with T1 as in step 4, catch QUAD4's magnet by hand at ~10 places round the
   new slot's edges and corners, and run `python3 tools/payload_geometry.py` at each: K = the furthest drone_3 x,y from
   the centred catch, plus 5 mm. Our contact check fires only inside K: today's K 0.024 is the OLD 60 mm plates' (14 Oct),
   while on 29 Oct the descending magnet sat a median 4.2 cm off our weld point (docs/analysis/attach_gates_2026-10-30.md,
   gate 2). Without this step type K=0.035 (it closes the 24-35 mm dead zone and stays inside his 45 mm loss rule).
6. **Placement:** QUAD4's mat on plate 9's line ~1.4 m out; the object in plate 9's gap ~1.5-1.8 m out, within ~20 deg
   of plate 9's line (29 Oct's k1_4: 1.8 m out, 12 deg off), so his straight transits pass between the carriers.
7. **Type the measured values** (every line below uses them):
   ```bash
   w5; C=$HOME/multi_drone_control/configs/rig
   W=0.028; L=1.041; K=0.035; X=-0.005; Y=0.197; PC=0.018; PT=0.010       # PT measured Fri; the rest 29 Oct / 21 Oct (K: 5b): replace
   P=$(python3 -c "print(round($L-0.003,3))"); Q=$(python3 -c "print(round($L-0.04,3))")
   ```

```bash
# the attach and pickup strings (after A7, in every terminal that launches)
A="tools/rig_flight.sh real_control_launch.py mode:=attach target_z:=1.0 lift_ramp_vel:=0.15 throttle_max:=0.95 enable_approach_mpc:=false mocap_magnet_tip_body_id:=24 params_file:=$C/attach_m1_1m.yaml"
M="partner:=true partner_contact:=true partner_nodes:=true weld_radius:=$K proof_taut_len:=$P weld_point_z:=$W attach_cable_len:=$Q attach_x_offset:=$X attach_y_offset:=$Y"
H="mpc_thrust_ratio:=22 trim_speed_gate:=0.20 carry_clearance:=0.25 plate:=9 carrier_azimuths_deg:=[90.0,330.0,210.0] commit_backdate:=0.04 tracking_check:=off"
V="attach_cpp_transit:=false attach_transit_clearance:=0.30 minimum_search_z_m:=2.20 approach_offset_z:=0.30 attach_twist:=zero match_xy:=0.08 match_dwell:=0.5 climb_speed:=0.3"
K1="cpp_authority:=false end_after_drop:=true climb_speed:=0.3 pickup_xy:=0.06 pickup_speed:=0.15 pickup_contact_height:=$PC pickup_object_thickness:=$PT"
K2="cpp_authority:=false end_after_drop:=true climb_speed:=0.3 pickup_xy:=0.08 pickup_speed:=0.20 pickup_contact_height:=$PC pickup_object_thickness:=$PT"
# our co-motion proof on a real catch (attach_gates gate 1: as configured it proved 0 of 108 start times on body 24
# caught on plate 9 in f4; with these 107 of 108): repeated (frozen) tip poses dropped, a 0.5 s speed window, 8 mm drift
G="tip_skip_repeats:=true comotion_d_ok:=0.008 comotion_speed_window_s:=0.5"
```
Every attach and pickup launch prints `QUAD4 planned body heights (m): ...` in T2 and refuses anything over 2.6 m.

## 1. The triangle 3/11/7 with QUAD4 (Tejen's attach and the drop-off)
Carriers QUAD1-3 on plates 3/11/7, QUAD4 on its mat. Stop T1 first: attach mode starts its own.
```bash
$A attach_azimuths_deg:=90,330,210 $M $G partner_args:="$H $V mission_mode:=floor_join descent_debounce:=1.0 descent_abort_xy:=0.10 contact_give_up_s:=120" sq6b   # [desk ok]
$A attach_azimuths_deg:=90,330,210 $M partner_args:="$H $V mission_mode:=floor_join descent_debounce:=1.0 descent_abort_xy:=0.10" sq6b0   # without G, K as typed, his 45 s give-up [desk ok]
$A attach_azimuths_deg:=90,330,210 $M $G partner_args:="$H $V mission_mode:=floor_join descent_debounce:=1.0 contact_give_up_s:=120" sq6   # 8 cm, 1.0 s [desk ok as sq6b]
$A attach_azimuths_deg:=90,330,210 $M partner_args:="$H $K1 mission_mode:=pickup_delivery" k1_5   # [desk ok]
$A attach_azimuths_deg:=90,330,210 $M partner_args:="$H $K2 mission_mode:=pickup_delivery" k1_6   # [desk ok]
K3="cpp_authority:=false end_after_drop:=true climb_speed:=0.3 pickup_contact_height:=$PC pickup_object_thickness:=$PT"
$A attach_azimuths_deg:=90,330,210 attach_magnet_initial:=ON $M partner_args:="$H $K3 mission_mode:=delivery_rejoin" k1_7   # object on the magnet at start [desk ok]
python3 tools/mocap_freeze_scan.py results/rig/2026-10-30/sq6b     # after each flight: did body 24 freeze?
```
- **Why the direct transfer, not his C++ planner:** from outside plate 9's gap his C++ planner, on the low route,
  takes authority and then never moves on: his own collision check fails on one carrier's tether model (twin R2903
  attach, R2904 pickup, 140 s each) although the real clearance stayed 0.70-0.75 m. So QUAD4's mat and the object stay
  in plate 9's gap (A6).
- **sq6b:** ARM, TAKEOFF; ring steady 10 s; his TAKEOFF. Expect his climb (~8 s, body ~2.37 m), the transit (~7 s),
  MATCH (~2 s), the descent (~3.9 s at 0.08 m/s from 0.30 m over the plate), then his bottom hold and our contact
  check. A swing past 10 cm shorter than 1.0 s no longer sends it back up (replay of sq5_2: 85 % of full descents get
  down; as flown 0 %). GIVE UP if it restarts 4 times; LAND after ~2.5 min of attempts.
- **k1_5:** the object in plate 9's gap. His descent onto the object, the lift, a level transit with the object's
  bottom 0.30 m over the ring's plates (body 2.42 m planned), a short descent into the net, the drop, a 3 cm climb
  (body 2.36 m planned; the twin overshot ~0.19 m on losing the object, so 2.56 m allowed), then he holds until LAND.
  The capture: replayed on k1_4's 26 s hover, 6 cm / 0.15 m/s closes 19 times, the first 0.56 s in (as flown: never);
  k1_6's 8 cm / 0.20 m/s 27 times. The ring takes the object's weight at the drop: watch the triangle's tilt.
- **k1_7 (QUAD4 starts with the object on its magnet):** his `delivery_rejoin`: he takes off already holding it, the
  low loaded transit, the drop into the net, then he holds until LAND. Lay QUAD4's cable out along plate 9's line with
  the object on the magnet at its end. `attach_magnet_initial:=ON` keeps QUAD4's coil ON from boot (else it is OFF for
  the ~5 s before his nodes start, and the object falls); our gate then forwards his ON until our span. Before his
  TAKEOFF, T2 must show his manager's `ASSUMED ATTACHED`; if it keeps printing `delivery_rejoin: a TAKEOFF now would
  be refused: the object does not read attached`, the object is not on the magnet (or body 6 / 24 is not tracked).

Then the 3-quad baselines (QUAD4's magnet off the ring). Start T1 for three quads:
```bash
ros2 launch bringup real_io_launch.py num_drones:=3 mocap_drone_body_ids:=11,12,13 mocap_payload_body_id:=8 magnet_initial:=ON mocap_extra_bodies:=21:/drone_0/magnet_tip_state,22:/drone_1/magnet_tip_state,23:/drone_2/magnet_tip_state   # T1
R3="tools/rig_flight.sh real_control_launch.py mode:=dissipative num_drones:=3 attach_azimuths_deg:=90,330,210 target_z:=1.0 lift_ramp_vel:=0.15 throttle_max:=0.95"
$R3 load_traj:=hover h3t                                                       # [desk ok]
$R3 load_traj:=hover params_file:=$C/hover_wpos2.yaml h3tw                     # OPTIONAL, your word: the planner's position weight x2 [desk ok]
$R3 load_traj:=line_spin traj_speed:=0.2 params_file:=$C/traj_hold.yaml ls3t  # [desk ok]
$R3 load_traj:=fig_8 traj_speed:=0.2 params_file:=$C/traj_hold.yaml f3t      # [desk ok]
```
- `throttle_max:=0.95` is needed: the rig default 0.8 is under the triangle's 0.83.
- **h3tw (optional, a planner gain: your word):** the ring's slow drift (1.4-3.8 cm at 0.07-0.23 Hz, which moves plate
  9 under QUAD4) is the whole formation moving with the planner's references, the trackers holding them to <1 cm
  (docs/analysis/ring_drift_2026-10-30.md; checked on sq2_5: references 2.0 cm, ring 2.4 cm, correlation 0.95-0.97).
  The planner pushes back weakly. h3tw doubles its ring-position weight. Fly h3t, then h3tw, 30 s hover each. Bar,
  frozen now: h3tw's drift below 0.3 Hz over its last 15 s at most 0.7 x h3t's, with no drop and no higher tilt or
  throttle p95. Risk: a harder lean; sq2_4's 1.1 Hz sideways swing. Not used on the attach lines unless it passes.

## 2. The square 0/3/6/9 (re-hang: QUAD1-4 on plates 0/3/6/9)
```bash
tools/t1_four_markers.sh detach:=true      # T1
R4="tools/rig_flight.sh real_control_launch.py mode:=dissipative num_drones:=4 attach_azimuths_deg:=0,90,180,270 target_z:=1.0 lift_ramp_vel:=0.15"
$R4 load_traj:=hover h4n                                                      # T2; LAND ~20 s after the top [desk ok]
$R4 load_traj:=line_spin traj_speed:=0.2 params_file:=$C/traj_hold.yaml ls4   # lands by itself [desk ok]
$R4 load_traj:=fig_8 traj_speed:=0.2 params_file:=$C/traj_hold.yaml f4n       # lands by itself [desk ok]
```
- **h4n first:** the notch's first 4-quad flight (it flew only on the triangle, sq2_5). Expect ~0.63 throttle. If plate
  0 lets go again (QUAD1's weak magnet): stop the square.

**sq036, Tejen's attach on 0/3/6** (QUAD1-3 on plates 0/3/6, QUAD4 on its mat on plate 9's line). Stop T1 first.
```bash
H036="mpc_thrust_ratio:=22 trim_speed_gate:=0.20 carry_clearance:=0.25 plate:=9 carrier_azimuths_deg:=[0.0,90.0,180.0] commit_backdate:=0.04 tracking_check:=off"
$A attach_azimuths_deg:=0,90,180 $M $G partner_args:="$H036 $V mission_mode:=floor_join descent_debounce:=1.0 descent_abort_xy:=0.10 contact_give_up_s:=120" sq036   # [desk ok without $G / give-up: 00:46]
```
- **Thrust:** the T puts nearly all the ring on plates 0 and 6. Predicted 0.86 throttle on fresh packs, rising ~0.066
  per minute, so ~60-80 s before the 0.95 cap (sq1 on 29 Oct: 0.85-0.9, the ring stuck at 0.90 m, dropped at 33.6 deg).
  **Fresh packs on QUAD1 and QUAD3.** Start his TAKEOFF as soon as the ring is at the top (no 10 s wait).
- **Stop:** the ring sinks over 5 cm or tilts over 10 deg: GIVE UP, then LAND. Then sq036m.
- **Twin (R2901):** welded 46.2 s, the loaded pair at 0.80 throttle (max 0.81), but the T leaned up to 8.2 deg toward the
  empty plate 9 before QUAD4 joined: close to the 10 deg stop. On the 0.86 kg ring (R2902): 0.72, 3.9 deg, welded.

## 3. sq036m: the same on the 0.86 kg M2A ring
1. Swap the ring: the M2A ring on the floor in the same place, its Motive asset as body 8 (its 22 Oct pivot), the new
   ring's asset off. QUAD1-3's magnets on its plates 0/3/6, QUAD4 on its mat.
2. Redo A4 with it (its plates are at r 0.225): new `X`, `Y`, `W`, `L`.
3. Fly with the old ring's solvers (no folder swap) and values (configs/rig/attach_m1_1m_m2a.yaml: attach_m1_1m's
   switches plus ring_m2a.yaml's mass, inertia and plate geometry; the inertia is not a launch argument):
```bash
AM="tools/rig_flight.sh real_control_launch.py mode:=attach target_z:=1.0 lift_ramp_vel:=0.15 throttle_max:=0.95 enable_approach_mpc:=false mocap_magnet_tip_body_id:=24 params_file:=$C/attach_m1_1m_m2a.yaml"
MDC_RIG_ACADOS_ROOT=$HOME/mdc_rig_generated_m2a_0p86 $AM attach_azimuths_deg:=0,90,180 $M $G partner_args:="$H036 $V geometry:=rig_m2a mission_mode:=floor_join descent_debounce:=1.0 descent_abort_xy:=0.10 contact_give_up_s:=120" sq036m   # [desk ok without $G / give-up: 00:50, load_mass 0.86]
```
- Predicted 0.76 on fresh packs, ~0.86 after 90 s.

## Why 29 Oct's three failures cannot happen the same way
| 29 Oct | Cause | Now | Checked |
|---|---|---|---|
| QUAD4 flew too high (sq2_5's climb 3.1 m; the drop-off preview 3.3 m) | the high transit over the carriers | the attach types the 0.30 m approach (V); the drop-off flies the low route; **the rig launch refuses any planned QUAD4 body height over 2.6 m** (the after-drop leg counted with the unload overshoot) and prints every leg in T2 | desk: sq6/sq6b 2.37 m, k1_5/k1_6 2.42 m; twin R2896-R2899 |
| QUAD4 never set off for plate 9 (sq3: his C++ hand-over bounds; sq4_5: under my search floor) | his C++ transit's hand-over and floor | his Python transfer (`attach_cpp_transit:=false`): no hand-over, no floor; the launch refuses a typed floor within 0.10 m of the climb end | sq5_2 on the rig: climb 7.8 s, transit 6.8 s, MATCH 1.7 s; twin sq6/sq6b welded on the first descent |
| QUAD4 hovered over the object and never went down (k1_4) | his capture's speed term (0.07 m/s) against the 1 m cable's swing | capture 6 cm / 0.15 m/s (k1_5), 8 cm / 0.20 m/s (k1_6); his contact point measured on the real object (A3) | replay of k1_4: closes 0.56 s in (19 closures in 26 s; k1_6 27) |

Not promised: that the magnet catches. The descents still face the swing (the debounce) and the frozen marker (A2).

## Stop rules
- Ring tilt over 15 deg that doesn't settle, rocking that grows, or a sink of more than ~10 cm: LAND.
- Automatic: the ring drops at 30 deg, and the fleet lands if a quad passes 1.75 m.
- SPACE disarms at once.

## Desk check (no rig)
**Final sweep, Fri 30 Oct 02:14-02:31, the final tree: all 15 lines** (sq6b, sq6b0, sq6, k1_5, k1_6, k1_7, sq036,
sq036m, h3t, h3tw, ls3t, f3t, h4n, ls4, f4n) came up exactly as written with A7's values, reached ready, no refusal,
crash or traceback; QUAD4's planned heights printed for every attach and pickup line (2.20-2.42 m, after a drop 2.56 m
with the overshoot). Script: scratchpad desk/sweep.sh. The per-line read-backs below are from the earlier checks.

Each line was brought up exactly as written plus `real_io:=false` (no radios), against `tools/fake_mocap.py` with the
line's geometry, on a private ROS domain; its parameters were read back, then Ctrl-C. No refusal, crash or traceback
before the Ctrl-C (sq6's "a TAKEOFF now would be refused: the attachment point has no fresh pose" is the startup line
sq2_5 also printed).

| Line | Read back |
|---|---|
| h4n, ls4, f4n (18:52) | 4 drones, load_mass 1.15, cable 1.0, notch 5.3 Hz on every tracker (low-pass 0), throttle_max 0.8, limit 1.75 m; hover / line_spin / fig_8 at 0.2 with the 5 s hold |
| h3t, ls3t, f3t (18:55) | 3 drones on 90,330,210 (read back 270,30,150: the counter-clockwise inside), throttle_max 0.95, the same paths |
| sq6, sq6b (18:56, 19:12, 23:20) | his planner: floor_join, the Python transfer, approach 0.30, climb 0.3 m/s, MATCH 0.08 m / 0.15 m/s / 0.5 s, restart 0.25 m/s and the debounce (sq6b 1.0 s, 0.10 m); search floor 2.20, transit 2.37 m |
| k1_5, k1_6 (23:20-23:45) | pickup_delivery, capture 0.06 / 0.15 (k1_6 0.08 / 0.20), end_after_drop, his C++ authority off; his contact point 4.5 mm under the object's marker (PC 0.018) on a 0.10 x 0.10 x 0.025 m object in his planner and his magnet manager; lift and over the drop 2.42 m |
| sq6b with G (00:57) | our manager: tip_skip_repeats true, comotion d_ok 0.008, speed window 0.5 s, contact xy 0.035 (K), proof_rules comotion; his give-up typed; QUAD4 2.20 / 2.37 m (the knobs needed adding to the launch's allowed arguments: the first try was refused, as on the rig it would have been) |
| sq036, sq036m (00:46, 00:50) | carriers 0,90,180; sq036m: load_mass 0.860 from attach_m1_1m_m2a.yaml, the 0.86 kg solvers (typing load_ixx/izz is refused: they are not launch arguments) |
| k1_7 (01:00) | delivery_rejoin, end_after_drop, his C++ authority off, lift 2.42 m, after the drop 2.56 m with the overshoot; with the object at the magnet his manager reads ASSUMED ATTACHED and the TAKEOFF refusal stops |

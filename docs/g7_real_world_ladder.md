# G7: the ladder to real-world M1 and M2 (planning pass, 2026-09-28)

This is a read-only planning pass: no code, launch, world or rig was touched. It uses these sources:
- the real launches, drone_communication, drone_magnet, the fleet manager, the planner and the dissipative node;
- tests.txt, CURRENT_STATE.md, decisions.md, real_attach_gap.md and the plan file sparkling-sprouting-possum.md;
- every `results/logs/controller_quad_load/*/params.json` with `use_sim_time: false` (the real flights), and results/registry.csv;
- Tejen's `~/tejen/drone_cage_control` (branch SwungPayload2026_Collaborative) and our fork `src/tejen_mission`.

`file:line` references are to this repo unless they are prefixed with `tejen:`. "UNVERIFIED" marks a statement no file or log confirms.

---

## 0. Where the rig actually is (evidence)

| What | Status on the rig | Evidence |
|---|---|---|
| Free hover without a load (real_hover, kT 24) | PASSED 2026-09-16. First one drone, then drones 0/1/2 together (15:09): peak z 0.66/0.66/0.60 against a 0.8 reference, clean LAND | tracker logs `planner_drone*_20260916_1342..1509`. Hover sat 10-18 cm below the reference at throttle 0.435-0.457, which is kT ≈ 21.5-22.6 against the typed 24 (my inference; not settled) |
| Ring lifted by the tethered fleet | NEVER. In every real log the ring z stays 0.050-0.053 m | all `use_sim_time:false` logs from 09-16 and 09-23. The 09-23 13:13 non-lift is written up in CURRENT_STATE.md:571-585 |
| Creep floor start on the 0.86 kg ring | never flown. The 09-16 16:36/16:38 creep runs used load_mass 1.0 and did not lift; the 16:40/16:42 runs had yaw spins | CURRENT_STATE.md:550-555 (open problem 8, unexplained) |
| kt_trim, z_ki, four drones, orbit, detach, attach, velocity-mode gains | never flown | CURRENT_STATE.md:366; tests.txt:68-69; real_attach_gap.md:26; docs/design/velocity_loop.md:213 |
| Preflight | NO-GO on 09-16 16:27/16:28 and 09-23 13:06/13:45: payload body origin 4.3-4.6 cm off the ring centre; drones resting 15.9-18.3 deg in mocap (bar 15); rods fitted at 0.465-0.472 m against the typed 0.50 | results/preflight/2026092313*.md, real_attach_gap.md:43 |
| Desk bring-up of real_attach_launch against fake_mocap | done 2026-09-10 | real_attach_gap.md:8-15 |
| 4 -> 3 detach desk rehearsal (plan :113-121) | NOT done | plan :307, :313 |
| Registry | has no rig rows. All 18 `W-` rows are Wesley's interactive Gazebo runs | registry.csv; docs/experiments/2026-09-16_rig_h0_h1.md:6-7 |
| Tejen, M1 IRL (his single drone, real pickup, virtual moving ring) | real pickups on 09-17 and 09-18 with object clearance 0.17-0.34 m. His loaded kT adapted to 18-20 with his UKF. The drop and the C1F hand-off were never reached (ring-future expiry, then the lift gate) | tejen:docs/thesis/experiment-history.md:686-768 |
| Tejen, M2 IRL (two drones, his stack only) | one run on 09-25, stalled at M2_TAKEOFF_HOVER with ~1 Hz vertical oscillation. The gate-profile fix d9c69415 is NOT in our fork (fork base a7524bae, src/tejen_mission/PROVENANCE.txt) | tejen:experiment-history.md:809-826 |

**Consequence for the draft ladder.** R1 (free hover) has already passed once, and R3 (carry hover) is the real critical path: nobody has ever lifted the ring. The draft's R3 is therefore split in two, a HOLD flight first and then the lift. R1 shrinks to a kT measurement per airframe, including the 4th airframe, which has never flown on our stack.

---

## 1. The rewritten ladder at a glance

| Rung | What it proves | New code needed first | Earliest |
|---|---|---|---|
| R0a | desk at home: real launches up against fake mocap, parameters read back, resize lines | P1-P4 (small) | before Wed |
| R0b | desk at the lab, props off: real mocap and radios, Motive bodies fixed, preflight GO, abort path, ESTOP, magnet channel per drone, mocap jitter recorded | none | **Wed 30 Sep** |
| R1 | each of the 4 airframes free-hovers; kT measured per airframe; no yaw spin | none | **Wed** |
| R2 | each tether magnet holds its share of the ring pull and releases on command; the capture distance is measured (for `weld_radius`) | none | **Wed** |
| R3a | 3 drones on plates 1/5/9: creep sweep to 45 deg, then HOLD taut with no lift (`lift_ramp_vel:=0`) | none | **Wed** |
| R3b | the same 3 lift the ring to 0.60 and hover, then LAND. First ever lift | none | **Wed** |
| R4 | 4 drones on plates 1/3/5/9 hover (the M1 carrier layout) | layout word (Q1) | **Wed if R3b passes early**, else next visit |
| ~~R4v~~ | DROPPED (loop check 2026-09-28): M1 never used the velocity loop. The trackers switch only on `/fleet/control_phase=network`, which OCP-resize mode never publishes; R0696's launch log has zero 'control phase -> VELOCITY LOOP' lines. M1 flew the MPC tracker throughout. | — | — |
| R5 | 4-drone slow orbit, 0.125 m/s, r 0.5 | cage-footprint check | next visit |
| R6 | detach 4 -> 3 in hover (OCP resize, magnet OFF); the freed drone holds; LAND with all disarmed | P6 yaw-datum fix, or place the ring at yaw 0 | next visit |
| R7 | detach during the orbit | as R6 | +1 visit |
| R8a | newcomer approaches the plate on OUR tracker with the magnet OFF and no weld: tip tracking, mocap tip body | P7-P11 | +1-2 visits |
| R8b | rejoin in hover: magnet weld detected, mux switch, OCP fold-in to n=4 | P7-P11 | +1-2 visits |
| R9 | rejoin during the orbit | as R8b | +2 visits |
| R10a | Tejen alone: his M1 IRL (pickup, drop into a virtual moving ring) | his (fork behind; ID map) | his schedule |
| R10b | his approach and hand-off (`/join_planner/handoff_ready`) onto our R8b weld, ring carried by 3 | P12-P14 | after R8b and R10a |
| R11 | full M1 on the rig from a FLOOR start with drone 3 as the 4th carrier | P15 (a sim floor-start M1 exists nowhere yet) | after R10b |
| M2a | Tejen alone: his two-drone M2 IRL join to the success hold | his (d9c6941 gate fix) | his schedule |
| M2b | our real M2 graph (mux, `/ours` namespace, unified mocap router) flies R4 with the drones placed attached by hand; his MPCs off | P12-P14, P16 | after R4 |
| M2c | his two drones join (M2a), then `/fleet/handover` to our trackers: sweep and HOLD (no lift: a 2-point hang capsizes), LAND | P12-P14, P16 | after M2a and M2b |
| M2d | full M2: his four-drone join, hand-over, lift, hover, LAND | his four-drone IRL stack does not exist (his IRL launch is two-drone only) | after M2c |

The draft's "small box" in R1 is dropped. `free_hover` only climbs, holds and lands (src/controller_quad_load/controller_quad_load/free_hover.py:1-16), so a box would be new code for no ladder value.

---

## 2. Cross-cutting facts every rung depends on

**ID map and radios (one table for the session).** Four sources say four different things:
- ours: `RIGID_BODY_TO_DRONE = {11:0, 12:1, 13:2, 14:3}`, payload body 8, `MAGNET_TIP_RIGID_BODY_ID = None` (src/drone_communication/drone_communication/motion_capture_publisher_node.py:25,30,33);
- Tejen M2 IRL: quad2 = body 12 = `/dev/QUAD2` is his drone_0, quad4 = body 14 = `/dev/QUAD4` is his drone_1, magnets 22/24, ring 8 (tejen:config/m2_irl_two_drone.yaml:1-32);
- Tejen M1 IRL: quad 12, magnet 22, ring **9** "reserved", pickup 6 (src/tejen_mission/config/irl_commissioning.yaml:28-38);
- CURRENT_STATE.md:682 still says 10/20/30 (stale).

Our convention is drone i = quad(i+1) = body 11+i = `/dev/QUAD(i+1)`. The udev file maps all four unique FTDI serials (/etc/udev/rules.d/99-elrs-quad.rules). Tejen's install command (`sudo install ... tools/irl_test/99-elrs-m2.rules /etc/udev/rules.d/99-elrs-quad.rules`) **overwrites the same filename with only QUAD2/QUAD4**. Never run it on this laptop.

**The shell helpers inject `num_drones:=3` BEFORE your args** (`~/.bashrc:162-164`). The sheets below spell out the full `ros2 launch` so there is no doubt. After the launch, `ros2 param get /central_controller num_drones` confirms the count.

**Radio node facts** (src/drone_communication/drone_communication/elrs_interface.py):
- Magnet: `magnet_channel` default 6 (= Betaflight AUX4, `tools/aux_sweep.py` 2026-09-16), `magnet_initial` ''|ON, and `/drone_<i>/magnet` String latch (:148-157). The latch overrides whatever the command carries on that channel (:311-313).
- Watchdog: 0.5 s without a command disarms. The log text says 0.1 s (:298-305).
- The node writes a frame only on ticks where no telemetry bytes are waiting (:308-314). Frames can be skipped (flown like this; noted, not a blocker).

**Mocap node facts** (motion_capture_publisher_node.py):
- Velocity is differenced over wall-clock packet ARRIVAL time (:239-252), low-passed with alpha 0.2 (:255-258).
- Angular velocity is rounded to 0.1 rad/s (:283-285). Stamps are the receive time (:355).
- **Bug: `return None` inside `run()` (:419-426) ENDS the receive loop on the first packet without a '|'.** Every pose then goes stale, and the 0.25 s pose watchdog disarms the whole fleet mid-flight. Fix P1 before any flight.

**Abort path:**
- The fleet manager's ESTOP publishes `/fleet/abort` and a direct disarm on each `/drone_i/ELRSCommand` (src/controller_quad_load/controller_quad_load/main.py:285-308).
- Tejen's MPC does not subscribe `/fleet/abort` (grep: only controller_mpc.py and main.py do).
- Behind a mux, his forwarded stream re-arms the radio on its next message, because `elrs_interface` takes `armed` from the latest message (elrs_interface.py:206).
- In the M2 graph, our manager's ELRS topics are remapped to `_diss` (dissipative_launch.py:176-181), so before `/fleet/handover` our ESTOP reaches no radio at all.

  So every rung with a mux (R8+, M2b+) needs P7.
  P7 DONE 2026-09-29 (0aef5c7): mux abort latch; DISARM/ESTOP ground a drone on his stream in Gazebo (R0751), M1 unchanged (R0752); the M2 arm B passed on T0034 once his join stall was fixed (37b3b5d).

**Magnet ownership for drone 3** (the one magnet does the ring weld, the pickup, the drop and the rejoin):
- The manager publishes the value on channels 5-10 of `/magnet/ELRSCommand` (magnet_attachment_manager.py:374), and the mux merges channel `magnet_channel` (default **10**, elrs_mux.py:71; real_attach_launch.py:141). Channel 10 lands on AUX8, not AUX4.
- If `magnet_initial:=ON` is given to real_io, the drone-3 radio latch overrides every mux-merged value.
- Tejen's hardware magnet is the String `/drone_i/magnet/command` into his own `elrs_interface_irl` (tejen:launch/m2_irl_two_drone.launch.py:87-91).
- While armed, his MPC sends channels 5-10 = 1.0 (tejen:controller_mpc_payload/main.py:2414,2417), i.e. magnet ON unless a latch is set.

  One path per drone is required: P10.

**Yaw datum:**
- The payload yaw is latched once (planner_node.py:622-630).
- `resize_fleet` builds a fresh `ReferenceBuilder` (planner_node.py:944-945) whose `psi0` is 0.0 (reference_builder.py:77). Nothing calls `set_yaw_datum` again.
- Callers: detach (dissipative_node.py:759) and attach (:1048). The attach slew is expressed with the node's latched `psi0` (:1080) while the new builder uses 0, so the two frames disagree.

  After any resize, a ring placed at yaw ψ is pulled toward yaw 0. It is invisible in sim because every world places the ring at yaw 0 (the M2 bench's 20 deg ring never resizes). Before R6: fix P6, or place the ring with plate 0 on Motive +x within 2 deg and check that the planner's `load yaw datum latched at` line reads within ±2 deg.

**Solver cache:** the acados signature covers load mass and drone mass (tools/prebuild_planner.py:17-20). The rig's weighed `drone_mass` is not the sim's 0.64, so the first rig launch compiles for 9-40 s per fleet size. An ARM while a tracker still compiles fails and disarms (main.py:236-248). Run `python3 tools/prebuild_planner.py --load-mass <m> --drone-mass <m> 3 4` at the lab after weighing, before the first launch. Switching back to sim masses rebuilds again (UNVERIFIED whether the cache keeps both).

---

## 3. Rungs in detail

Common to every flight rung:
- **Terminals:** 1 = `real_io_launch.py`, 2 = control launch, 3 = preflight and commands.
- **Setup:** `source ~/ros2_humble/install/setup.bash && source ~/multi_drone_control/install/setup.bash` (the `mdc` helper).
- **Logs to read after each flight:**
  - `results/logs/controller_quad_load/planner_drone<i>_<time>/log.csv`: trackers; diag columns for thr, tilt, body rates, qref, solver status.
  - `.../load_planner_<time>/` or `.../dissipative_controller_<time>/log.csv`: planner tick log with payload height and tilt at 10 Hz (since 2026-09-25).
  - `results/preflight/<ts>.md`.
  - Terminal-2 stdout (the `[kT dN]` lines are printed, not logged): run terminal 2 as `ros2 launch ... 2>&1 | tee results/rig/<date>/<flight>.log`.
- **Bag every flight** (cheap, replayable): `ros2 bag record -o results/rig/<date>/<flight> /drone_{0..3}/motion_capture_state /payload/motion_capture_state /drone_{0..3}/ELRSCommand /drone_{0..3}/telemetry /fleet/command /fleet/abort /fleet/status /fleet/landed`.
- **Registry:** one row per flight, id prefix `H` (hardware; my pick, conventional with R/T/S/W), exploratory unless flown under a card.
- **After a spin:** `tools/yaw_excursion.py` on the tracker log.
- **Global abort (every flight rung):** ESTOP (`ros2 topic pub --once /fleet/command std_msgs/msg/String "{data: ESTOP}"` or the panel) on any of:
  - a drone past 35 deg tilt;
  - yaw rotating more than 45 deg;
  - a drone not leaving the floor within 5 s of the others;
  - ring tilt past 15 deg;
  - anyone inside the cage.

  The envelope faults by itself at 60 deg tilt or 3 m/s (controller_mpc.py:593-597); the operator acts earlier.

### R0a Desk at home (laptop only, props irrelevant, nothing armed)
- **Proves:** the real launch graphs start, every parameter reaches its node, the resize and magnet lines appear, and the preflight logic runs.
- **Prerequisites:** P1 (mocap fix), P3 (fake_mocap `--azimuths-deg`/`--yaw-deg`), P4 (preflight read-back).
- **Rig-twin sim rehearsal:** not possible for real_io (serial, UDP). The control launches can be rehearsed in Gazebo only after P5.
- **Desk check:**
  ```bash
  python3 tools/fake_mocap.py --num-drones 3 --cable-len 0.47 --elev-deg 45 --load-z 0.05     # + --azimuths-deg 30,150,270 after P3
  ros2 launch controller_quad_load real_io_launch.py num_drones:=3 rviz:=false   # elrs_interface loops on "No suitable serial port" without adapters: expected
  ros2 launch controller_quad_load real_control_launch.py num_drones:=3 load_mass:=0.86 cable_len:=0.47 attach_azimuths_deg:=30,150,270 kt_trim:=false lift_ramp_vel:=0.0
  python3 tools/preflight.py --drones 3 --planner load_planner --max-ground-z 0.5
  ```
  Then the 4-drone detach graph (plan :113-121):
  ```bash
  python3 tools/fake_mocap.py --num-drones 4
  ros2 launch controller_quad_load real_io_launch.py num_drones:=4 detach:=true magnet_initial:=ON rviz:=false
  ros2 launch controller_quad_load real_dissipative_launch.py num_drones:=4 attach_azimuths_deg:=30,90,150,270 reconfig_mode:=ocp detach_magnet:=true
  ros2 param get /dissipative_controller reconfig_mode          # ocp
  ros2 topic pub --once /fleet/detach std_msgs/msg/Int32 "{data: 1}"
  ```
  Expect "OCP ready for n=3", "FLEET RESIZED to n=3", "DETACH drone 1 (OCP resize)", "magnet OFF -> /drone_1/magnet". Then run `tools/clean_slate.sh`.
- **Pass:** preflight shows every read-back line with the typed values. The only FAILs allowed are telemetry/battery (no `--real`) and "fleet not yet flying" if the fake z is above the bar. The four log lines appear.
- **Time:** 1 h.

### R0b Desk at the lab (props OFF, batteries in, real mocap and radios)
- **Proves:** Motive bodies and the radio path are right before any prop goes on; the abort path works; the magnet channel is right on every drone; the mocap timing is known.
- **Prerequisites:**
  - Motive: the payload body origin at the ring centre on the plate-top plane, +x toward plate 0 (Tejen's convention, tejen:CODEX_HANDOFF §8 lines 196-207).
  - Motive: each drone body redefined level (preflight bar 15 deg). IDs per §2 (Q3).
  - Fresh 6S packs of at least 24 V (preflight bar 22.8).
  - Weigh each airframe with its pack, and the ring.
  - Run `tools/prebuild_planner.py` with the weighed masses.
  - P1 in place.
- **Checks, in order:**
  1. `ros2 topic hz -w 500 /drone_0/motion_capture_state` for each body: ~120 Hz, and record the std dev of dt (input to P9). Bag 60 s of all bodies at rest: velocity noise at rest is the number that decides P9.
  2. `ros2 topic echo /drone_<i>/telemetry --once` for each drone: battery and RSSI (link up).
  3. Magnets: panel MAGNET toggles or `ros2 topic pub --once /drone_<i>/magnet std_msgs/msg/String "{data: ON}"`, then OFF. A steel plate sticks, then drops. If a drone does not respond, run `tools/aux_sweep.py --drone <i>` (terminal 2 down, no `magnet_initial`). This also closes Tejen's open items 1-2 (AUX ch6 on quad2 and quad4, tejen:docs/thesis/m2-irl-two-drone-commissioning-20260924.md:108-115).
  4. Preflight GO: `python3 tools/preflight.py --drones 3 --real --planner load_planner --max-ground-z 0.5`. It must show the payload origin within 3 cm of the drones' circle centre, every drone resting within 15 deg, and the fitted rod equal to the typed `cable_len` ±4 cm.
  5. Abort path (card 2026-09-16_rig_h0_h1.md T0.3): ARM, TAKEOFF with props off, then cover one drone's markers for 2 s. All drones disarm within ~1 s, and the log shows the pose timeout and `/fleet/abort`. Then `tools/clean_slate.sh`, relaunch, ARM and ESTOP: all disarm at once.
- **Pass:** all five. **Abort:** any drone stays armed in step 5, which means no flight until the log is read.
- **Time:** 60-90 min including Motive work.

### R1 Free hover per airframe; kT
- **Proves:** each airframe, including the 4th (never flown on our stack), flies our tracker at the typed kT. It measures kT per airframe on this pack. It checks the 09-16 spin does not recur.
- **Prerequisites:** R0b.
- **Sim rehearsal:** none needed (flown on the rig 09-16).
- **Rig commands (one drone at a time, then all four spaced ≥ 1 m):**
  ```bash
  ros2 launch controller_quad_load real_io_launch.py num_drones:=1 drone0_serial:=/dev/QUAD<n>        # mocap map: that drone's body must be drone 0 -> see P13, or fly all four with num_drones:=4
  ros2 launch controller_quad_load real_hover_launch.py num_drones:=1 hover_z:=0.8 drone_mass:=<unused> 2>&1 | tee ...
  ```
  - Not UNVERIFIED but a trap: with the fixed body map (:25) a 1-drone launch can only fly body 11. For quad2-4 either launch `num_drones:=4` and put all four on the floor (the free hover latches each xy from its first pose; free_hover.py:12-13), or wait for P13.
  - Recommended: `num_drones:=4 drone0_serial:=/dev/QUAD1 ... drone3_serial:=/dev/QUAD4`, all four spaced ≥ 1 m, ARM, TAKEOFF, 30 s hover, LAND.
- **Pass:**
  - every drone settles within ±0.05 m of 0.8 after the trim converges;
  - `[kT dN] measured in hover: X` printed at LAND for every drone;
  - yaw within ±15 deg of the latched heading;
  - clean LAND with all disarmed.

  Record X per airframe and pack voltage before and after.
- **Abort:** global. If a trim estimate rails at ±25 % (24 × 0.75 = 18 or × 1.25 = 30), LAND and type the printed value.
- **Time:** 30-45 min.

### R2 Tether magnets and capture range (props off)
- **Proves:** the magnet holds its rod share and releases on command. Gives the capture distance for `weld_radius` (the manager must not declare a weld the magnet cannot make: real_attach_gap.md:23).
- **Prerequisites:** R0b step 3.
- **Desk check (bench):** on each drone, magnet ON onto a ring plate, then pull along the rod with a spring scale. The load per rod is about m_L·g / (n·sin 45°) = 0.86·9.81/(4·0.707) ≈ 3.0 N static for n = 4 and 4.0 N for n = 3. It must hold at least 2× that (6-8 N). Then OFF: it releases with no residual stick. Record the gap at which ON still captures the plate (feeler or ruler).
- **Pass:** hold ≥ 2× the share on all four; release under 0.2 s; capture gap recorded. **Abort:** a magnet that holds less than 1.5× the share does not fly as a carrier.
- **Time:** 20 min. Can run in parallel with R0b.

### R3a Three-drone creep and HOLD (no lift)
- **Proves:** the creep floor start sweeps each rod from its resting pose to 45 deg and the fleet holds the taut configuration on the rig. This isolates thrust and geometry from the lift (the August order: hold first, CURRENT_STATE.md:709-712).
- **Prerequisites:** R0b, R1 (typed kT per airframe? the tracker takes ONE `thrust_ratio` for all drones: real_control_launch.py:108; use the median measured X), R2. Plates 1/5/9 (azimuths 30/150/270, an even 3-ring rotated 30 deg, which is exactly the M1 carrier set after drone 3 leaves).
- **Sim rehearsal:**
  - existing `configs/experiments/ocp_hover_ground_creep.yaml` (n3, creep, even ring, rotation-equivalent);
  - to create: `ocp_hover_ground_creep_hold.yaml` (copy, `lift_ramp_vel: 0.0`, `kt_trim: false`);
  - to create: `ocp_hover_ground_creep_kt110_kt.yaml` (copy, `thrust_ratio: 91.41` = +10 % typed-high like the rig's 24 vs ~22, `kt_trim: true`).

  All three are headless `tools/run_experiment.py`, ~4 min each. They also serve loop step 8 (re-fly the creep baseline on the gyro default).
- **Desk check:** R0a with the same command line.
- **Rig commands:**
  ```bash
  ros2 launch controller_quad_load real_io_launch.py num_drones:=3 drone0_serial:=/dev/QUAD1 drone1_serial:=/dev/QUAD2 drone2_serial:=/dev/QUAD3 magnet_initial:=ON
  ros2 launch controller_quad_load real_control_launch.py num_drones:=3 load_mass:=<weighed> drone_mass:=<weighed mean> cable_len:=0.47 attach_azimuths_deg:=30,150,270 attach_z:=<measured, 0.0 if the Motive origin is on the plate tops> thrust_ratio:=<R1 median> kt_trim:=false lift_ramp_vel:=0.0 load_traj:=hover 2>&1 | tee ...
  python3 tools/preflight.py --drones 3 --real --planner load_planner --max-ground-z 0.5
  # panel: ARM, TAKEOFF; hold 15 s after the handover line; LAND
  ```
- **Pass:**
  - "handover" in the planner log within 15 s of TAKEOFF;
  - every rod within ±5 deg of 45 deg at the hold;
  - ring floats just off the floor, payload z < 0.15 (the sim twin R0726 floats it 2 cm after a 5.5 cm overshoot: the
    planner takes the full weight at the hand-over; aligned with the sheet 2026-09-29);
  - drone tilt < 25 deg in `[diag dN]` (sim drones lean 18.6 deg against 45-deg rods);
  - coupled: HOLD throttle clearly above the drone's R1 free hover, drone within rod + 3 cm of its plate;
  - no drone at the 0.6 throttle cap for 2 s (LAND if so);
  - no abort; LAND with all disarmed.

  Record the settled throttle per drone.
- **Abort:** global. Also a drone drifting radially with a fixed reference (the tracker, not the planner: h0_h1 T1 falsifier).
- **Logs:** planner tick log (phase, rod elevations), tracker diag, `[kT dN]` lines.
- **Time:** 30 min (2 flights).

### R3b Three-drone lift and hover at 0.60
- **Proves:** the first ever ring lift on the rig; the height loop (z_ki) and the trim (flight 2) on hardware.
- **Prerequisites:** R3a passed.
- **Sim rehearsal:** as R3a (the lifting configs).
- **Rig commands:** the R3a line with `lift_ramp_vel:=0.15`. Flight 1 `kt_trim:=false` (tests.txt:52 omits it, so as written flight 1 would fly the trim ON: correct the sheet). Flight 2 `thrust_ratio:=<printed measured> kt_trim:=true`. ARM, TAKEOFF, 20 s hover, LAND.
- **Pass (thresholds from h0_h1 T2 and G2):**
  - payload z settled 0.55-0.66 (target 0.60, then ±0.03 once z_ki settles);
  - ring tilt peak < 15 deg, settled mean < 5 deg;
  - drone tilt < 25 deg;
  - no envelope fault; LAND with all disarmed.
- **Abort:** global. Pack under 21.6 V. Ring tilt above 15 deg for more than 1 s.
- **Logs:** planner tick log (payload z, tilt), `[kT dN] measured in hover`.
- **Time:** 45-60 min (2-4 flights). **Card required** (rig go/no-go): update docs/experiments/2026-09-16_rig_h0_h1.md or write `2026-09-30_rig_carry.md` via `/experiment` before Wed.

### R4 Four drones on plates 1/3/5/9, hover
- **Proves:** the four-carrier M1 layout lifts level.
- **Prerequisites:**
  - Wesley's word on 1/3/5/9 (decisions.md:25, :29 OPEN; Q1);
  - the 4th airframe passed R1 and R2;
  - P4 (the disc fit handles only n = 3 today, tools/preflight.py:158).
- **Sim rehearsal:** existing `configs/experiments/ocp_hover_ground_creep_n4_3915.yaml` (same plates, creep, `four_rigid_ground_3915.sdf`); re-fly on the gyro default (loop step 8).
- **Rig commands:**
  ```bash
  ros2 launch controller_quad_load real_io_launch.py num_drones:=4 drone0_serial:=/dev/QUAD1 drone1_serial:=/dev/QUAD2 drone2_serial:=/dev/QUAD3 drone3_serial:=/dev/QUAD4 magnet_initial:=ON detach:=true
  ros2 launch controller_quad_load real_dissipative_launch.py num_drones:=4 load_mass:=<w> drone_mass:=<w> cable_len:=0.47 attach_azimuths_deg:=30,90,150,270 thrust_ratio:=<R3b> reconfig_mode:=ocp detach_magnet:=true lift_ramp_vel:=0.15 load_traj:=hover 2>&1 | tee ...
  python3 tools/preflight.py --drones 4 --real --planner dissipative_controller --max-ground-z 0.5
  ```
  `real_dissipative_launch` flies the OCP until a detach (real_dissipative_launch.py:16-20), so R4 already runs the R6 graph.
- **Pass:** as R3b; settled ring tilt ≤ 5 deg (sim R0510 hovers this layout level).
- **Time:** 45 min.

### R4v Velocity-mode tracker (DROPPED: M1 flew the MPC tracker, see the table)
- **Proves:** `velocity_after_handover` (vel_kp_pos 2, vel_kv 4, vel_k_att 8, vel_ki 0: the mode of every M1/attach sim result, real_attach_launch.py:92) is stable on the airframe.
- **Rig command:** the R4 line plus `control_mode:=velocity_after_handover vel_ki:=0.0` (plus `diss_ki_load:=1.0` only once the network is in play; not in R4v).
- **Pass:** as R4 with no oscillation growth. **Abort:** oscillation, which means halving `vel_kp_pos` and `vel_kv` (tests.txt:68-69), one change per flight.
- **Alternative (decision for Wesley):** fly M1 on the MPC tracker in sim instead and skip R4v.
- **Time:** 30 min.

### R5 Slow orbit with four
- **Proves:** the M1 trajectory: 0.125 m/s, r 0.5, `load_traj:=orbit` (smootherstep spin-up, load_trajectory.py:20).
- **Prerequisites:** R4. The cage fits a footprint radius of about 0.5 + 0.25 + 0.47·cos 45° ≈ 1.1 m around the orbit centre (P16 check).
- **Sim rehearsal:** to create `orbit_ground_creep_n4_3915.yaml` (R4 config with `load_traj: orbit`, `traj_speed: 0.125`, `traj_radius: 0.5`).
- **Rig command:** R4 line plus `load_traj:=orbit traj_speed:=0.125 traj_radius:=0.5`. LAND after one lap. `hover`/`orbit` do not self-complete.
- **Pass:** ring tilt mean ≤ 3 deg (G1 orbit bar), radius error recorded, no fault.
- **Time:** 30 min.

### R6 Detach 4 -> 3 in hover
- **Proves:** OCP-resize detach with a real magnet release; the freed drone holds; the LAND gate waits for it.
- **Prerequisites:** R4. P6, or the yaw-0 placement workaround (§2).
- **Sim rehearsal:**
  - existing `detach_hover_n4_3915.yaml`, `detach_ocp_n4_3915.yaml`. LAND v4 is clean 2/2, R0571/R0572: peak ~3 deg, settled ~0.1 deg (tests.txt:188).
  - SIL: `configs/sil/detach_hover_3915_ocp.yaml`, plus a yawed-ring arm after P6.
- **Desk check:** R0a's detach block.
- **Rig commands:** R4 graph with `load_traj:=hover`. About 10 s after the ring reaches 0.60:
  ```bash
  ros2 topic pub --once /fleet/detach std_msgs/msg/Int32 "{data: 1}"      # drone 1 = plate 3 (90 deg), the M1 leaver
  ```
  Hover 20 s on three, then LAND.
- **Pass:**
  - peak ring tilt ≤ 8 deg (G1 detach bar);
  - settled < 2 deg within 5 s;
  - survivors' dip ≤ 0.05 m;
  - drone 1 holds within 0.15 m of its detach point;
  - `/fleet/landed` only after drone 1 is below 0.15 m; all disarmed.
- **Abort:** ring tilt > 15 deg. A magnet that does not release: the drone is then still tethered to a resized OCP, so LAND at once.
- **Time:** 45 min.

### R7 Detach during the orbit
Same graph as R6 with `load_traj:=orbit traj_speed:=0.125 traj_radius:=0.5`. `/fleet/detach 1` mid-lap.
- **Sim:** `detach_ocp_n4_3915.yaml` at 0.4 m/s (R0556: ~7 deg peak).
- **To create:** a 0.125 m/s orbit variant.
- **Pass:** peak ≤ 8 deg, orbit resumes within 5 s.
- **Time:** 30 min.

### R8a Newcomer approach, no weld
- **Proves:**
  - the rig can fly a free 4th drone on our tracker to the plate while three carry the ring;
  - `/magnet_tip_pose` from the tip body agrees with `/attach_target/pose` (real_attach_gap.md step 3);
  - the fleet manager arms and disarms the newcomer.
- **Prerequisites:** P7, P8, P10, P11, P13; R2 capture gap.
- **Sim rehearsal:**
  - existing `m1_rejoin_hover_u_t01_bb.yaml` (partner mode);
  - for our-tracker approach: `configs/sil/attach_hover_ocp.yaml` and `attach_hover_n3_ocp_near.yaml` (Gazebo, `enable_approach_mpc:=false`, as tests.txt:201);
  - the rig-twin run needs P5 plus a `real:=true` attach launch (P8).
- **Rig command:** depends on P8 (UNVERIFIED until built).
  - Shape: the real attach launch with `num_drones:=3 reserved_attach:=1 attach_azimuths_deg:=30,150,270 attach_x_offset:=0.0 attach_y_offset:=0.25 reconfig_mode:=ocp enable_approach_mpc:=false weld_radius:=0.0 ...`, which blocks the weld (no `/magnet/object_attached`);
  - press ATTACH; the newcomer holds above the plate; LAND.
- **Pass:** tip-to-target error < 0.03 m while hovering above the plate for 5 s; newcomer disarmed at LAND.
- **Time:** 45 min.

### R8b Rejoin in hover (weld)
- **Proves:** magnet weld on hardware, weld detection by mocap, mux switch, OCP fold-in 3 -> 4 at a real ring.
- **Pass:** G1 rejoin bars:
  - peak ring tilt ≤ 5 deg;
  - carriers' dip ≤ 0.03 m;
  - under 2 deg within 3 s;
  - the rig's first flight may accept ≤ 10 deg peak (Wesley's call).
- **Abort:** a weld declared without a physical catch (tip > capture gap). Ring tilt > 15 deg.
- **Time:** 45-60 min.

### R9 Rejoin during the orbit
Same with `load_traj:=orbit` (sim: `m1_rejoin_orbit_moving.yaml`, the `attach_moving` default). Pass: G1 bars. Time: 45 min.

### R10a Tejen alone: M1 IRL
His runner, `bash tools/sim_test/run_m1_irl_virtual_ring.sh true run` (config `src/tejen_mission/config/irl_commissioning.yaml`: quad 12, magnet 22, pickup 6, UDP host 192.168.0.87:1511, 0.060 kg object, magnet channels [5..10] "not positively identified", :81-83).
- It shares nothing with our stack. It must not run while our real_io is up: same UDP port 1511 and same `/dev/QUAD2`.
- **Pass (his):** pickup, lift, drop into the virtual ring, clean land.
- **Our input:** the ID map (Q3) and the magnet channel from R0b.

### R10b His approach and hand-off onto our weld
- **Proves:** `/join_planner/handoff_ready` -> mux -> our tracker on hardware, with his MPC stream on `ELRSCommand_tejen`.
- **Prerequisites:** R8b, R10a, P12 (unified router), P14 (his IRL launch remaps his MPC's `ELRSCommand` to `_tejen`; tejen:launch/m2_irl_two_drone.launch.py:302-306 has no such remap today), P7.
- **Sim:** existing `m1_rejoin_hover_u_t01_bb.yaml` (partner-attached start, detach, release, handoff, weld).

### R11 Full M1
- **Prerequisites:** R9, R10b, P15.
- **P15 matters because every sim M1 config is an AIR start** (`partner_attached_orbit*.yaml`: `start_taut: true` on stands, `three_attach_partner_attached*.sdf`), and three_attach_launch.py:75 defaults `start_taut:=true`. The rig must start from the floor with drone 3 welded as the 4th carrier and the creep path. This start has never been simulated.
- **Lab volume:** his transit needs z ≥ 1.8 m (`minimum_search_z_m` 1.8, pickup lift 1.40; CURRENT_STATE.md:598-600). Cage height UNVERIFIED (Q5).

### M2a Tejen alone: two-drone M2 IRL
- **Commands** (tejen:commandsForM2IRL_TwoDroneRViz_20260925_v1.txt:18-107; our copy `tools/irl_test/run_m2_irl_two_drone.sh`):
  ```bash
  ./tools/irl_test/run_m2_irl_two_drone.sh io
  ./tools/irl_test/run_m2_irl_two_drone.sh mission <yaml> true
  ```
- **Blockers on his side:**
  - our fork lacks d9c69415 (the settle gate 0.03 -> 0.08 m/s that his 09-25 stall needs);
  - `props_off_verified: true` was set with no recorded proof (a7524bae);
  - calibration offsets are still identity (tejen:m2_irl_two_drone.yaml:24-32);
  - `rviz:=true` needs `m2d_four_drone.rviz` and the M2FleetPanel, which our fork lacks;
  - his MPC `cable_length` 0.531 vs YAML tether 0.475 is unreconciled.

### M2b Our real M2 graph on a hand-placed attached fleet
- **Proves:** the real M2 launch (P14) works end to end:
  - one mocap owner (P12);
  - one radio node per drone;
  - the `/ours` namespace;
  - muxes forwarding ours (`approach_stream: false`, i.e. no hand-over yet);
  - our ESTOP through the muxes (P7).

  The flight itself is R4 (four drones placed on plates 1/3/5/9, creep, lift, hover, LAND).
- **Sim:** `configs/experiments/m2_bench.yaml` / `m2_bench_imu.yaml` (air start on hangers, `airborne_start: true`). A floor-start M2-graph config is to create (dissipative_launch `partner_m2:=true` on `four_rigid_ground_3915.sdf`; UNVERIFIED that his X3 models are needed).
- **Pass:** as R4.

### M2c Two-drone hand-over, hold only
- **Proves:** `/fleet/handover` -> mux on hardware with his MPC flying before and ours after, on his real join state.
- **Flight:** M2a to the success hold, then:
  ```bash
  ros2 topic pub --once /fleet/handover std_msgs/msg/Bool "{data: true}"
  ```
  Our two trackers take over with `airborne_start:=true lift_ramp_vel:=0.0` (sweep to 45 deg, hold 10 s), then LAND.
- **No lift:** two rim points hang the ring as a pendulum about their chord (tests.txt:112-113).
- **Pass:** mux logs "-> DISSIPATIVE tracker" per drone within 1 s; no drone dips more than 0.05 m at the switch; all disarm at LAND.

### M2d Full M2
- **Needs:** his four-drone IRL stack (his IRL launch is two-drone; tejen:launch/m2_irl_two_drone.launch.py), M2c, and R4.
- **Pass:** G2 bars:
  - ring 0.60 ± 0.05;
  - ring tilt ≤ 5 deg;
  - drone tilt ≤ 25 deg;
  - `/fleet/landed` with all disarmed.

---

## 4. Prerequisite work list for the loop (ordered)

| # | Work | Where | Effort | Card / critic (working rules) | Needed by |
|---|---|---|---|---|---|
| P1 | `return None` -> `continue` in the receive loop (a malformed packet must drop one sample, not end the node); pytest on malformed and good packets | motion_capture_publisher_node.py:419-426 | 30 min | bug fix: no card, no critic | R0b (**Wed**) |
| P2 | Wed command sheet in tests.txt (R0b-R4 lines of §3). Flight 1 `kt_trim:=false`; plates via `attach_azimuths_deg`; `drone_mass`; full `ros2 launch` lines (the helpers inject `num_drones:=3`) | tests.txt:41-62, :103-123 | 30 min | none | **Wed** |
| P3 | fake_mocap: `--azimuths-deg`, `--yaw-deg`, `--drone-z` (drones resting on the floor for the creep start) | tools/fake_mocap.py:23-37 | 45 min | none | R0a |
| P4 | preflight: n-drone least-squares disc fit (today n == 3 only, :158); read back `start_taut`, `handover_elev_deg`, `creep_vel`, `z_ki`, `reconfig_mode`, `kt_trim`, `control_mode`, each `elrs_interface` `magnet_channel`/`magnet_initial`; FAIL when the typed azimuths and the measured bearings differ by more than 10 deg | tools/preflight.py | 1.5 h | none | R0a/R4 |
| P5 | Sim rehearsals: create `ocp_hover_ground_creep_hold.yaml`, `ocp_hover_ground_creep_kt110_kt.yaml`, `orbit_ground_creep_n4_3915.yaml`; fly them plus the n3/n4 creep baselines once each on the gyro default (loop step 8 covers the baselines) | configs/experiments | 20 min + ~5 headless runs | exploratory rows | R3/R4 rehearsal (**Wed**) |
| P5b | Optional (Wesley's word): real_control/real_dissipative take `use_sim_time`, `pose_timeout_s`, `safety_ref_timeout_s` args with today's defaults (false / node 0.25 / node 1.0), so the exact rig launch can fly against Gazebo with rviz_quad_load_launch as a rig-twin rehearsal (with `thrust_ratio:=83.1` or `91.41`; the real launches take no `auto`) | real_control_launch.py:162, real_dissipative_launch.py:197 | 1 h | launch file: ask first (hook) | R6+ rehearsals |
| P6 | Yaw-datum fix: after `self.refs = ReferenceBuilder(...)` in `resize_fleet`, re-apply the latched datum (`if self._yaw_datum_latched: self.refs.set_yaw_datum(self.psi0)`). Pytest (resize keeps psi0). SIL option `initial.load_yaw_deg` (tools/sil/scenario.py:187-208 + plant). One SIL run `detach_hover_3915_ocp` at yaw 30 before and after | planner_node.py:944-945 | 2-3 h | bug fix: no critic; registry rows | R6 (or yaw-0 placement) |
| P7 | Abort through every mux: `elrs_mux` subscribes `/fleet/abort` and from then on forwards only `armed=False` (latched until relaunch). Optionally the same latch in `elrs_interface` (the last hop, and the only one that also covers Tejen's stream). Pytest on the policy | elrs_mux.py:38-141; elrs_interface.py:204-228 | 2 h | a gate that holds the fleet down: **critic** | R8a, M2b |
| P8 | Real rejoin launch, recommended as `real:=true` in three_attach_launch.py (drops every gz node, `use_sim_time` false, fixed `thrust_ratio`, keeps the fleet manager at `n + reserved_attach` (:383-387), the partner args and the attach flags), instead of re-syncing real_attach_launch.py. That file lacks 28 arguments (tools/param_diff.py: `reconfig_mode` falls back to network, no `partner*`, no `pose_timeout_s`/`safety_ref_timeout_s`, no 0.05 m/s weld-speed gate) and its fleet manager manages only n (real_attach_launch.py:187-189) | launch | 0.5-1 day | launch file: Wesley's word (Q6); reviewer on the final rig table | R8a |
| P9 | Mocap velocity: decide from the R0b bag (dt jitter, rest noise). If needed, difference over a fixed window with a least-squares slope on monotonic arrival times, or have the Motive forwarder send the frame timestamp (UNVERIFIED which program forwards) | motion_capture_publisher_node.py:239-258 | 2-4 h | estimator in the flight loop: **card + critic** | before R8 (not a blocker for R1-R4: 09-16 free hover flew on it) |
| P10 | One magnet path per drone on the rig: the manager and the dissipative node publish the String `/drone_<d>/magnet` (the radio latch), the mux merge is off (`magnet_command_topic ''`), `magnet_channel` 6 everywhere; `magnet_initial` per drone (drone 3 OFF in R8, ON in M1's welded start) | magnet_attachment_manager.py:351-386, elrs_mux.py:71-78, real_io_launch.py:87-88 | 2-3 h | none (plumbing); pytest | R8a |
| P11 | Weld detection on the rig: the manager on the tip body (`/magnet_tip_pose`); `weld_radius` ≤ R2 capture gap; `attach_speed_threshold` 0.05; dwell 0.15 s (:108) | real launch (P8) | in P8 | none | R8b |
| P12 | One mocap owner for both stacks (only one socket can bind 1511; his router has no SO_REUSEADDR). Extend his `m2_irl_mocap_router.py` in our fork to also publish `/payload/motion_capture_state` (ring body) and `/magnet_tip_pose` for the drone-3 magnet body, with the ID map from one YAML; our real launches then drop `motion_capture_publisher_node` | src/tejen_mission/tejen_mission/m2_irl_mocap_router.py | 0.5 day + pytest on recorded packets | none; tell Tejen | R10b, M2b |
| P13 | Until P12: our mocap node's body map as launch params (`rigid_body_ids`, `payload_body_id`, `magnet_tip_body_id`; defaults = today's constants) | motion_capture_publisher_node.py:25-33 | 1 h | none | R1 convenience, R8a |
| P14 | Real M2 graph: `real:=true` for dissipative_launch `partner_m2` (no payload emulator, `use_sim_time` false, fixed kT); his IRL launch remaps his MPC's `ELRSCommand` -> `ELRSCommand_tejen`; mux `magnet_channel` 6; one radio node per drone (his `elrs_interface_irl` or ours, not both); a single drone-id map (his drone_0 = quad2, ours drone_1 = quad2) | dissipative_launch.py:164-275; tejen launch :298-330 | 1 day, with Tejen | launch: Wesley's word; reviewer at M2 | M2b |
| P15 | Floor-start M1 in sim: a world with the partner drone welded on plate 3 among three carriers on the floor, `start_taut: false`, the creep; one Gazebo run before any matrix (the working rules: floor/takeoff) | simulation_assets (test copy), configs | 0.5 day | world: Wesley's word; card for the claim | R11 |
| P16 | Cage footprint and height versus the orbit (≈1.1 m radius) and his 1.8 m transit | hardware fact (Q5) | — | — | R5, R11 |
| P17 | Sync our fork to d9c69415 (his gate profiles, C++ authority on in IRL) | src/tejen_mission | Tejen merge | — | M2a |

Not needed for any rung (checked): the 6S constants (applied: real_control_launch.py:127-128, preflight --battery-min 22.8 :87).

---

## 5. Wednesday 30 Sep (R0-R4 target): what is ready and what is not

**Can be ready** (code only P1-P5, all small; the loop can do them Mon-Tue, sim runs included):
- R0a at home (needs P1, P3, P4).
- R0b, R1, R2, R3a, R3b at the lab. No new control code. They need P1, the P2 sheet, the Motive fixes, fresh packs, weighed masses, and the prebuild at the lab.
- R4: command-ready, but it needs Wesley's word on 1/3/5/9 and a 4th airframe through R1/R2 first. Realistic only if R3b passes by mid-session.

  Order of the day (≈5 h in the lab):

  | Step | Time |
  |---|---|
  | Motive and weighing | 1 h |
  | R0b | 1 h |
  | R1 | 45 min |
  | R2 (parallel) | — |
  | R3a | 30 min |
  | R3b | 1 h |
  | R4 | 45 min |
- The card for R3b/R4 (`/experiment`, falsifier stated before flying) must exist before Wed.

**Cannot be ready for Wed:**
- R5 (the cage footprint is unknown, and it is gated on R4).
- R6/R7: the graph is ready (real_dissipative `reconfig_mode:=ocp detach_magnet:=true`), but it is gated on R4, and needs P6 or the yaw-0 placement.
- R8+ (P7-P13), R10b/R11 (P12-P15), M2b-M2d (P12-P14, Tejen's four-drone IRL).
- M2a is Tejen's and needs d9c69415 in whichever tree he flies from. He flies from his own repo, so that is his call.

**Never on Wed, with the rig stack up:** SIL or Gazebo on this laptop (working rules).

---

## 6. Open questions for Wesley (hardware facts and decisions only)

1. **Layout:** plates 1/3/5/9 (azimuths 30/90/150/270, drone at plate 3 leaves) for R4 onward, and 1/5/9 for the three-drone R3? This closes decisions.md:25 and :29.
2. **The 4th airframe and Tejen's quads:** are our drones 0-3 the same four airframes as his quad1-4 (bodies 11-14, magnets 21-24)? Which quad is drone 3 / his M1 drone (his M1 IRL flies quad 12 = our drone 1)? Is the 4th airframe flight-ready on Wed?
3. **Motive asset list:** is the ring body 8 (ours, his M2) or 9 (his M1 IRL)? Do the magnet-tip bodies 21-24 and pickup 6 exist in the current Motive project? Which program forwards Motive to UDP 1511, is it Z-up, and can it send the frame timestamp (for P9)?
4. **The physical kill:** besides ESTOP on the laptop and the 0.5 s radio watchdog, is there a hand-held kill (TX switch or power)? Who holds it?
5. **Cage size:** usable radius and height (orbit ≈1.1 m radius, his transit ≥ 1.8 m)?
6. **Real launches for M1/M2:** add `real:=true` to three_attach_launch.py and dissipative_launch.py (one graph for sim and rig, recommended), or keep and re-sync separate real_* files?
7. **Rejoin bar on the rig's first weld:** keep G1's ≤ 5 deg peak, or accept ≤ 10 deg for the first hardware weld?
8. **M1 on the MPC tracker instead of velocity mode** (skips R4v), or fly R4v?

## Answers (Wesley 2026-09-28)
- Q1 layout: plates 1/3/5/9 (four), 1/5/9 (three).
- Q6 launches: `real:=true` mode on the sim launches (three_attach_launch / dissipative_launch), not separate files.
- Q7 first real weld: <= 10 deg go/no-go at R8b; claim runs keep <= 5 deg.
- Q8 tracker mode: moot. M1 flew the MPC tracker throughout (R4v dropped).
- Still open (hardware facts for the lab): Q2 airframe mapping / drone 3 / 4th airframe, Q3 Motive project ids and the UDP forwarder, Q4 kill switch, Q5 cage size.

## Loop progress (2026-09-28)
- R0a DONE at home 2026-09-29 (D0003, tools/sim_test/desk_r0a.py): the Wednesday R3a lines start and read back as typed, the
  pre-TAKEOFF gate refuses after a 0.26 s pose drop, clean_slate --rig keeps T1 (radios, mocap, fleet_viz). Preflight fixed: all-None
  magnet_channel read-back now FAILs. SIL R0a' (R0758/R0759): at the rig thrust gain a 0.64 kg airframe pins the 0.6 cap (ring sags
  to 0.45 m), 1.0 kg holds with 0.005 margin.
- P9 reader READY (2026-09-29): `tools/r0b_mocap_report.py <bag>` prints per body dt jitter, rest noise of v and w, glitches per 100 s
  (|w| > 0.5 rad/s: the rig twin of the sim kicks) and a 5-sample LSQ-slope candidate; synthetic-bag test; in the R0b sheet (6958199).
- P (mocap receive loop): FIXED ed7084d, offline test (3 bad packets then 2 good -> 2 published).
- P (yaw datum on resize): FIXED, `resize_fleet` carries psi0 into the rebuilt builder; unit test
  test_resize_yaw_datum.py (51/51 planner tests). Live proof still owed: one Gazebo/SIL detach with the ring placed
  at yaw ~30 deg before R6.

- Sim twins flown on the gyro default (2026-09-28): R0726 (R3a hold), R0727 (R3b, kT +10 % + kt_trim), R0728 (R5 orbit
  n4 on 30/90/150/270). R0726 finding: with lift_ramp_vel 0 the ring does NOT stay down: at the creep hand-over the
  OCP takes the full weight (throttle 0.181) and floats the ring 2.2 cm up after a 5.5 cm overshoot. R3a on the rig:
  expect the ring 2-6 cm off the floor at full load; R3a bar ring z <= 0.10 m (config updated).
- P6 yaw datum: PROVEN LIVE (A/B on the rotated-rig test world detach_hover_n4_3915_yaw30): fix on R0730 max yaw
  deviation 0.09 deg after the DETACH resize; fix off R0731 the ring yawed 30 -> 0 deg within ~3 s. Rig R6 no longer
  needs a yaw-0 placement.
- P8 real:=true mode on three_attach_launch / dissipative_launch: built (18 offline tests), under review.
- P7 abort latch: critic NOT READY, parked on Wesley's rulings (GOALS question 6).
- P8 real:=true: COMMITTED a0bc23c after review (sim graphs identical over 145 cases; 21 offline tests). Reviewer fixes:
  approach MPC refused on the rig, num_drones guard for the drone-3 chain, no DETACH row on the M2 graph.
  Open before R8a/R11/M2c (reviewer): (a) real-mode defaults for the creep floor start (start_taut false, handover_elev 45,
  settle 2.0, creep 0.2) and reconfig_mode ocp - a launch default, GOALS question 7; (b) P10 single magnet path (the
  newcomer's magnet is undefined until its first forwarded command; attach_magnet_initial:=ON would override the manager's
  OFF; the mux merge overrides Tejen's channel 6); (c) plumb detach_magnet:=true in real mode (a carrier DETACH releases no
  magnet today); (d) thrust_ratio plausibility range (threshold, Wesley); (e) M2 abort gap = P7.
- P13 mocap body map: DONE (commit above): node params drone_body_ids / payload_body_id / magnet_tip_body_id (defaults
  unchanged) and rig launch args mocap_drone_body_ids:=11,12,13,14 mocap_payload_body_id:=8 mocap_magnet_tip_body_id:=<id>
  (reach the node through the M1/M2 real modes' real_io include). Offline test: typed ids route, defaults unchanged.
- P15 floor-start M1: the M1 world already starts on the floor (air start = start_taut true). Config
  configs/experiments/partner_attached_orbit_floor.yaml (canonical + the creep args). R0732 confirms the gap: drone 3 is
  not creeped and folds in low (ring tilt 8-13 deg, detach 9.68, rejoin 5.23). Fix = GOALS Q9 (attach geometry).

# Rig commands, Wed 30 Sep 2026 (branch real-world-testing)

Copy-paste order for the day. The full sheet (reasons, watch lines, bars) is the WED section of `tests.txt`.
Every terminal first: `mdc` (sources ROS and this workspace). Laptop on a table outside the cage; nothing else runs on it.

## 0. Setup (lab, before anything is powered)

```bash
mkdir -p results/rig/2026-09-30 && cd ~/multi_drone_control
# weigh: each airframe with pack, rod and magnet (~1.2 kg expected), the ring; tape the rods
cat > results/rig/2026-09-30/wed.env <<'ENV'
RING=0.86          # weighed ring (kg)
DM3=1.2            # mean of drones 0-2 (kg)
DM4=1.2            # mean of drones 0-3 (kg)
ROD=0.47           # measured rod (m)
KT=24.0            # replaced by the R1 median later
ENV
source results/rig/2026-09-30/wed.env
python3 tools/prebuild_planner.py --load-mass $RING --drone-mass $DM3 3
python3 tools/prebuild_planner.py --load-mass $RING --drone-mass $DM4 4
ls -l /dev/QUAD*                      # all four present (QUAD1..4 = drones 0..3)
```
Motive: ring body 8 (origin at the ring centre on the plate plane, +x toward plate 0), quads 11-14, pickup 6.
Optional T5, a status window (manager decision, phase, per-drone armed / mocap age / throttle / battery, and every
warning with what it means): `python3 tools/fleet_monitor.py --drones 4` (read-only; close it any time).

## 1. R0b desk (props OFF, packs in)

```bash
# T1 (radios + mocap + RViz), stays up all day
ros2 launch controller_quad_load real_io_launch.py num_drones:=4 drone0_serial:=/dev/QUAD1 drone1_serial:=/dev/QUAD2 drone2_serial:=/dev/QUAD3 drone3_serial:=/dev/QUAD4 magnet_channel:=6 magnet_initial:=ON
# T3
ros2 topic hz -w 500 /drone_0/motion_capture_state            # ~120 Hz; repeat for 1-3 and /payload/...
timeout 60 ros2 bag record -o results/rig/2026-09-30/r0b_rest /drone_{0..3}/motion_capture_state /payload/motion_capture_state
python3 tools/r0b_mocap_report.py results/rig/2026-09-30/r0b_rest        # P9 numbers
ros2 topic echo /drone_0/telemetry --once                     # each drone: >= 24.0 V, RSSI present
ros2 topic pub --once /drone_0/magnet std_msgs/msg/String "{data: OFF}"   # then ON; each drone clicks
```
Then the abort checks, props OFF, R3a T2 up (section 3 line), preflight GO:
1. ARM, wait for every "Armed - waiting for TAKEOFF", cover drone 1's markers for 1 s: expect
   `Drone 1 disarmed before TAKEOFF ... TAKEOFF refused` in T2 (the pre-TAKEOFF gate). Ctrl-C T2, `tools/clean_slate.sh --rig`.
2. Relaunch T2, ARM, press SPACE (with focus on the RViz 3D view, then again with focus in the DETACH box): all disarm.
3. Relaunch T2, ARM, CLI kill: `ros2 topic pub -r 10 -t 10 /fleet/command std_msgs/msg/String "{data: ESTOP}"`: all disarm.
4. Cover the RING's markers for 2 s with the fleet armed: note what T2 prints (the ring pose has no watchdog).

## 2. R2 magnets (props OFF, T2 down)

Each drone on its plate: magnet ON, pull along the rod with the spring scale: holds >= 8 N (three) / 6 N (four);
lift the drone by hand: the ring rim rises (coupled); OFF releases < 0.2 s. Note the capture gap per magnet.

## 3. THRUST CHECK, first thing with props on (replaces R1; about 40 min)

Question it answers: how much thrust the drones really have per unit of throttle, and so what throttle carrying the
ring needs against the tracker's 0.6 cap. Two free hovers of all four drones, rods ON: plain, then with a known mass
taped to each rod tip. Steady hover throttle = weight / full thrust, so the rise in throttle for a known added mass
gives each drone's thrust and effective mass even without trusting the scale.

Before props (5 min):
- weigh each drone with pack, rod and magnet; weigh the hung mass (about 150 g each, e.g. a small bottle of water;
  the same for all four) and write both down;
- Betaflight CLI per quad: `diff all`, save as results/rig/2026-09-30/betaflight_quad<N>.txt (we need throttle_limit,
  motor_output_limit, thrust_linear, tpa: if Betaflight itself caps throttle, raising our cap would do nothing).

T1, plain hover (all four on the floor >= 1 m apart, rods on and hanging, magnets as they are):
```bash
# T2
export MDC_RUN_DIR=$PWD/results/rig/2026-09-30/t1_free_logs
ros2 launch controller_quad_load real_hover_launch.py num_drones:=4 hover_z:=0.8 thrust_ratio:=24.0 2>&1 | tee results/rig/2026-09-30/t1_free.log
# T3, ~10 s after "Controller ready" x4
python3 tools/preflight.py --drones 4 --real --planner free_hover --max-ground-z 0.5
```
ARM, TAKEOFF, hold 20 s at 0.8 m, LAND. Pass: tilt < 3 deg, xy < 0.12 m, no yaw turn > 15 deg (the new yaw hold keeps
yaw 0 until each drone is airborne; `yaw_hold` column). Note each drone's throttle on the panel ("thr").

T2, the same with the mass taped to each rod tip (the mass rests on the floor at takeoff and lifts as the rod goes taut):
```bash
# T2 (Ctrl-C the previous T2 first, then tools/clean_slate.sh --rig)
export MDC_RUN_DIR=$PWD/results/rig/2026-09-30/t2_hung_logs
ros2 launch controller_quad_load real_hover_launch.py num_drones:=4 hover_z:=0.8 thrust_ratio:=24.0 2>&1 | tee results/rig/2026-09-30/t2_hung.log
```
ARM, TAKEOFF, hold 20 s, LAND. Expected throttle: +10 to +25 % over T1. LAND at once if a drone shows "AT CAP" on the
panel (throttle stuck at 0.6: that drone cannot lift the mass under the cap, which is itself the answer).

Analysis (I can run it from these files; send me the four paths and the numbers you wrote down):
```bash
python3 tools/thrust_check.py --free results/rig/2026-09-30/t1_free_logs --loaded results/rig/2026-09-30/t2_hung_logs \
    --hung-kg <mass kg> --masses <d0,d1,d2,d3 kg>
```
It prints per drone: hover throttle plain and loaded, kT, the mass implied by the throttle rise (vs the scale), full
thrust, whether the thrust map is linear, and the throttle each layout needs to carry the ring (three on 1/5/9, even
four, four on 1/3/5/9) against the 0.6 cap. Set KT in wed.env to the median kT it prints.

## 4. Headroom go/no-go (before any tethered flight)

```bash
python3 tools/headroom.py --masses <d0,d1,d2> --az 30,150,270 --ring $RING --u-free <u0,u1,u2>          # three on 1/5/9
python3 tools/headroom.py --masses <d0,d1,d2,d3> --az 0,90,180,270 --ring $RING --u-free <u0,u1,u2,u3>    # even four (R3e)
```
At 1.2 kg and u_free 0.45 the prediction is 0.568 (three), 0.537 (even four), 0.589 (1/3/5/9 plate 9); the cap is 0.60.
OVER CAP: no tethered flight on that layout. Above your stop value: R3a hold as a measurement only.

## 5. R3a creep and HOLD, three on plates 1/5/9 (or R3e: four on 0/3/6/9)

```bash
# T4 (bag), per flight
ros2 bag record -o results/rig/2026-09-30/r3a_f1 /drone_{0..3}/motion_capture_state /payload/motion_capture_state /drone_{0..3}/ELRSCommand /drone_{0..3}/telemetry /fleet/command /fleet/abort /fleet/status /fleet/landed /drone_{0..3}/magnet /rosout
# T1 relaunched for three drones
ros2 launch controller_quad_load real_io_launch.py num_drones:=3 drone0_serial:=/dev/QUAD1 drone1_serial:=/dev/QUAD2 drone2_serial:=/dev/QUAD3 magnet_channel:=6 magnet_initial:=ON
# T2
export MDC_RUN_DIR=$PWD/results/rig/2026-09-30/r3a_f1_logs
git rev-parse HEAD > $MDC_RUN_DIR.code.txt; git status --short >> $MDC_RUN_DIR.code.txt
ros2 launch controller_quad_load real_control_launch.py num_drones:=3 load_mass:=$RING drone_mass:=$DM3 cable_len:=$ROD attach_radius:=0.25 attach_z:=0.0 attach_azimuths_deg:=30,150,270 thrust_ratio:=$KT kt_trim:=false z_ki:=0.4 start_taut:=false handover_elev_deg:=45.0 handover_settle_s:=1.0 creep_vel:=0.2 target_z:=0.6 lift_ramp_vel:=0.0 load_traj:=hover 2>&1 | tee results/rig/2026-09-30/r3a_f1.log
# T3, ~10 s after the trackers print "Controller ready"
python3 tools/preflight.py --drones 3 --real --planner load_planner --max-ground-z 0.5
```
ARM, TAKEOFF, hold 15 s after "handover settle complete", LAND.
Pass: hand-over within 15 s; rods 45 +-5 deg; ring z < 0.15; drone tilt < 25 deg; coupled (hold throttle above R1's,
drone within rod + 3 cm of its plate); nobody at throttle 0.6 for 2 s. Record the settled throttle per drone.
R3e: T1 with four drones, T2 `num_drones:=4 drone_mass:=$DM4 attach_azimuths_deg:=0,90,180,270`, preflight `--drones 4`.
Yaw check (the 09-16 spins were on this creep start): every drone should log `yaw_hold` 1 from TAKEOFF until it leaves the
floor and hold heading within 15 deg; if u3 swings to +-1 within 0.3 s of the release, note it (the remaining mechanism).

## 6. R3b0 first lift (only if R3a coupled, no spin, hold throttle under your stop value)

Same T2 line with `lift_ramp_vel:=0.15` (flight name r3b0_f1). ARM, TAKEOFF, 20 s at 0.60, LAND.
At LAND each tracker prints `[kT dN] measured in hover: X`: that is the tethered kT for the claim flights.

## After every flight

```bash
python3 tools/rig_to_run.py --out results/rig/2026-09-30/<flight>_run --trackers "$MDC_RUN_DIR/logs/controller_quad_load/planner_drone*" --bag results/rig/2026-09-30/<flight> --t2-log results/rig/2026-09-30/<flight>.log
python3 tools/plot_run.py results/rig/2026-09-30/<flight>_run
python3 tools/kick_events.py $MDC_RUN_DIR
```
Between flights: Ctrl-C T2 (and T4), `tools/clean_slate.sh --rig`, relaunch. After a drop or crash: see SAFETY AND RECOVERY in tests.txt.

## 7. Afternoon 30 Sep: lift with pretension (after the 12:49 failure)

`wed.env`: `KT=21.7` for packs at about 23 V (a 25.0 typed on those packs stalls the creep: hold f1). Use `KT=25.0` only on fresh packs at 24 V or more. `ROD=0.47` (0.50 and 0.53 stalled the creep). `kt_trim` stays off: in f3/f4 it never changed the gain; the wobble was the gain mismatch.

The planner now pretensions: every rod's pull ramps 0 -> share together over `pretension_s` (3 s), drones held, full for 0.5 s, then the height ramp. If the creep times out short of 37 deg it refuses to pull or lift ("creep timed out: LAND" in RViz): press LAND. The panel's AT CAP now uses each drone's own cap (0.8 here).

```bash
cd ~/multi_drone_control
source ~/ros2_humble/install/setup.bash && source install/setup.bash
source results/rig/2026-09-30/wed.env
tools/clean_slate.sh --rig
export MDC_RUN_DIR=$PWD/results/rig/2026-09-30/lift_f6_logs
echo "KT=$KT ROD=$ROD"
ros2 launch controller_quad_load real_control_launch.py num_drones:=4 load_mass:=$RING drone_mass:=$DM4 cable_len:=$ROD attach_radius:=0.25 attach_z:=0.0 attach_azimuths_deg:=0,90,180,270 thrust_ratio:=$KT kt_trim:=false z_ki:=0.4 start_taut:=false handover_elev_deg:=45.0 handover_settle_s:=1.0 creep_vel:=0.2 target_z:=0.35 lift_ramp_vel:=0.1 pretension_s:=3.0 load_traj:=hover throttle_max:=0.8 2>&1 | tee results/rig/2026-09-30/lift_f6.log
```

Log order to expect, in Terminal 2:
1. `elevation ... reached`
2. `starting pretension`
3. `pretension done (ring z ..., lifted ...)`
4. `starting lift ramp`

During the 3 s pretension, all four "thr" rise together and the ring floats up level.

- **LAND if:** one side lifts first, a magnet peels, or the creep times out.
- **DISARM** if a drone tips.

**If it holds level at 0.35 m for 15 s:** repeat once, then the same line with `target_z:=0.6`.

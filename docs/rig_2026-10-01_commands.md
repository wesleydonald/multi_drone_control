# Rig session card and commands, Thu 1 Oct 2026 evening (90 min, branch real-world-testing)

Card: `docs/experiments/2026-10-01_tethered_model.md` (card A: rig arms, stop rules, critic). Plan:
`docs/plan_2026-10_tethered_model.md` (Schedule). Every lift has the W1 logging on: planner solve status,
node-0 tension, rod length and elevation, ring quaternion and omega, reference age; tracker thr_out, thr_off.
Freeze is on (it is the only code path; no launch arg). Code under test: W8 (solve-failure handling), W9 (pivot
4 cm below the drone centre), W12 (tracker throttle bookkeeping), A4 (ring quaternion continuity, slot-offset
warning), W13 (safety-net bound 0.15, gate 0.25).

**What the sim does and does not show.** Every sim number below is from the rig TWIN (fitted thrust law, so the
carried fraction is ~1.0 by construction): it does not predict the rig's force shortfall. The exact rig code
(fba2d7a) was flown on the laptop twin (R0810-R0816, card A "Twin on the flown code"): hold, hand-over and orbit
match the lab twins R0783-R0789 (hold z within 0.1 mm, tilt within 0.01 deg, orbit xy max +6 %), 0 failures.
R0816 (z_ki 0.4) disarmed the fleet on LAND after the ring was down (drone 0 tipped, R0793 mechanism; 1 of 11
nominal twin landings). The H2/H3/C1 twins stay single runs per machine (R0788/R0816, R0786/R0813, R0789/R0815).

## What the sim says to expect (lab runs and laptop SIL, 30 Sep night; card A "## Results", Groups 3-6)

| item | sim (rig twin: rig world, rig thrust plant; SIL where stated) | what it means for tonight |
|---|---|---|
| solve failures, ref age | 0 failures in every twin run; ref age 0.0 in R0782-R0789, R0791 and R0810-R0816 (R0793 0.0028 during the LAND abort); SIL at ring yaw 180, A4 on / off: 0 / 0 (R0798 / R0800) | neither sim has the replay trigger (ring yaw near 180 deg plus a 30 deg slot offset; the benches cannot spawn the offset); the rig check that matters is the slot-offset line below |
| rig thrust plant vs tracker | free drone 0 (0.55 kg with rod): model throttle 0.2743 against 0.2747 expected (R0792) | the tracker's offset bookkeeping (W12) closes the loop on the affine plant in sim; D1 is its first rig test |
| rods at hand-over | 0.550 with the pivot on (R0783, R0784, R0786, R0788, R0789; fba2d7a R0810-R0816), 0.583 with it off | on the rig expect 0.53-0.58 per drone (model-f1 read 0.587-0.613 without the pivot) |
| hand-over elevation | 54.7-55.6 deg (R0782-R0789; fba2d7a 55.4-55.6, R0810-R0816; a trial creep fix, R0801-R0809, did not move it and was removed). The legacy floor worlds hand over at 45.7-45.8 (R0773, R0778) | not a measurement error: the drones build an 11-15 deg lead in the first second after lift-off (they leave the 0.10 m vertical lead at 0.5-0.6 m/s), and the latch fires before it decays. The lead comes from the lift-off pop, not from where the sweep starts. Bar 50-60 on the pivot reading (model-f1 53.6 on the centre reading is ~51 on the pivot); recorded, not a stop rule (section 6) |
| hold z, z_ki 0 | 0.510 / 0.504 at target 0.5 (R0783, R0786; fba2d7a R0810/R0811 0.510, R0813 0.504) | the twin carries the ring by construction (fitted law, carried 0.996). The rig force deficit (~0.8 of the ring below 0.45 m) is NOT in the sim: expect H1 to sit low, as model-f1 did (0.31-0.35) |
| ring mass off by -0.16 / +0.16 kg (z_ki 0) | hold +16.4 / -14.5 cm; tilt 1.2-1.3 deg, heave 1.2 cm, 0 failures (R0793 / R0791) | a mass error shows as height only, which is what z_ki closes (H2). The freeze caught the light ring at 97 % of the pull and did nothing on the heavy one |
| standing tilt | 1.2-1.4 deg mean | from the per-drone offsets 0.181-0.187 against one tracker offset 0.185; expect about this much on the rig from that alone |
| H2 (net on) | hold 0.500, heave 1.5 cm, tilt 1.22 deg, \|z_bias\| max 0.013, 0 failures (R0788; fba2d7a R0816 and R0814 the same, R0816 then disarmed on LAND) | the net works with the pivot model in the rig twin; on the rig z_bias will be larger by the force deficit (bar 0.10) |
| C1 (circle r 0.5 at 0.125 m/s) | lap tilt mean 1.22 deg (max 1.54), \|z err\| max 1.3 cm, xy err 3.1 cm mean (4.8 max), 0 failures (R0789); fba2d7a R0815 same window: 1.22 deg (max 1.69), 1.3 cm, 2.95 cm (5.08 max, +6 %) | inside every C1 bar with margin in sim |
| 1/3/5/9 hover (z_ki 0) | R0786: 0 failures, rods 0.550, hand-over 55.4, hold 0.504, heave 0.5 cm, hold tilt mean 1.41 deg (peak 1.77), carried 0.996; fba2d7a R0813: 0 / 0.550 / 55.5 / 0.504 / 0.45 cm / 1.42 (1.79) / 0.996 | the uneven ring holds like the even one in sim; with z_ki 0.4 (H2 line) the hold should close to 0.500 as in R0788 |
| LAND after the ring is down | R0793 (light ring; all four drones hovered wide, radii 0.595-0.604 vs R0783 0.572-0.580): drone 0 tipped 4 -> 22 -> 68 deg at 49.25-49.75 s, envelope fault, fleet disarm with the ring on the floor. Nominal twin, fba2d7a R0816 (z_ki 0.4): ring down t 50.45, drone 0 pushed out by its rod (radius 0.58 -> 0.80 m), rates saturate, tilt 20 -> 29 -> 74, fault 77.9 deg, fleet disarm t 51.86; other three <= 3.6 deg. 1 of 11 nominal twin landings; the other 10 (R0783-R0792, R0810-R0815) landed, every drone <= 21 deg | a rigid rod reaches the ring before the drone reaches the floor; a drone pushed wide of the ring can tip on the way down, now seen on the nominal twin too (rate unknown from one event). LAND is unchanged: watch the drop after "load down, rods slack", hand on the throttle cut (section 6) |

## Open decisions (default applies without an answer)

1. **Ring placement.** Default: plate 0 pointing roughly to world +x (ring yaw within +-45 deg of 0), so the
   ring yaw never crosses +-180 deg tonight. A4 (quaternion continuity) is then not exercised on the rig; it
   passed the replay (the 155ee56 planner replays to 14 failures, 9 + 5; 0 with the sign kept continuous, arm g;
   the shipped A4 replays to 0; the ring quaternion was not logged on the rig, so the trigger is inferred); SIL cannot show it (R0798 / R0800 both 0 failures).
2. **Freeze-off arg (W11).** Default: none tonight; the freeze stays on (the only code path). Adding a launch
   arg to switch it off touches a gate that can hold the fleet down, so it goes through the critic after the
   visit.

## Schedule (90 min)

| min | step | flights |
|---|---|---|
| 0-17 | setup, ring orientation, magnet check, packs, dry launch (props off), preflight | - |
| 17-25 | D1 free hover, drone 0, rod and magnet removed | 1 |
| 25-37 | D2 tethered holds, target 0.3 then 0.7, freeze on, z_ki 0 | 2 |
| 37-47 | H1 tethered hover 0.5, z_ki 0 | 1 |
| 47-57 | H2 as H1 with the safety net | 1 |
| 57-70 | H3 hover on plates 1/3/5/9 (move drones 0 and 2 one plate, magnet check, fly the H2 line) | 1 |
| 70-85 | C1 slow circle r 0.5 at 0.125 m/s, even ring (magnets back on 0/3/6/9), only if H1 and H2 pass | 1 |
| 85-90 | copy logs, land packs, note times | - |

If time is short, C1 is the one to drop. D2 comes before H1 (critic must-fix 2).

## 0. Setup (before props)

Every terminal: `mdc` (or the two `source` lines below). Laptop outside the cage; no SIL or Gazebo on it while
the rig stack is up (same topics).

```bash
cd ~/multi_drone_control
source ~/ros2_humble/install/setup.bash && source install/setup.bash
colcon build --symlink-install --packages-select controller_load_mpc controller_quad_load simulation_communication
source install/setup.bash
python3 -m pytest -q src/controller_quad_load/test/test_thrust_offset.py src/controller_quad_load/test/test_yaw_hold.py \
  src/controller_load_mpc/test/test_pivot_offset.py src/controller_load_mpc/test/test_planner_log.py \
  src/controller_load_mpc/test/test_solve_fallback.py src/controller_load_mpc/test/test_quat_continuity.py   # 62 passed
python3 tools/prebuild_planner.py --load-mass 0.86 --drone-mass 0.55 4
python3 -c "from controller_quad_load import controller_mpc as c; c._solver_is_fresh() or c.generate_ocp_controller()"   # tracker OCP; rebuilds only if acados.py/dynamics.py changed
D=results/rig/2026-10-01; mkdir -p $D
git rev-parse HEAD > $D/code.txt; git status --short >> $D/code.txt; git diff > $D/code.diff
tar czf $D/code_tree.tgz --exclude=__pycache__ src tools configs
ls -l /dev/QUAD*                      # QUAD1..4 = drones 0..3
```

Motive: ring body 8 (origin at the ring centre on the plate plane), quads 11-14.

**Ring rigid-body orientation (model-f1 sat one plate, 30 deg, off).**
1. In Motive, select the ring body: its local +x axis must point at plate 0 (the plate the drone-0 rod goes on).
   If not, re-make or rotate the body pivot so +x is on plate 0, with the body level.
2. Place the ring with plate 0 toward world +x (decision 1). Magnets on plates 0 / 3 / 6 / 9 (even ring:
   azimuths 0 / 90 / 180 / 270, `attach_azimuths_deg` empty).
3. After T2 is up (section 1), the planner prints, before ARM:
   `[planner] slot azimuth error (deg): d0 +x.x (plate 0), ...` with every value within +-10, and no
   `[planner] SLOT OFFSET: drone d sits +30 deg from plate p: check the ring rigid body ...`.
   Read it in the T2 terminal, not on the panel: the manager's next status overwrites the panel line.
   Any SLOT OFFSET line: do not ARM. Fix the rigid body or the plates, relaunch T2.

**Magnet check (props off, T2 down).** Each drone on its plate, magnet ON: pull along the rod with the spring
scale, holds >= 6 N; lift the drone by hand, the ring rim rises with it; magnet OFF releases in < 0.2 s. Note
which plate each magnet sits on (0/3/6/9; 1/3/5/9 for H3) and the rod pivot 4 cm under each drone centre, unchanged.

**Packs.** >= 23.6 V at rest on all four, and telemetry non-zero on all four (drone 1 read 0.00 V on 30 Sep;
without a voltage the tracker drops the voltage term, about 0.02 throttle per volt):
```bash
for i in 0 1 2 3; do ros2 topic echo --once /drone_$i/telemetry | grep battery_voltage; done
```
Swap a pack before any lift where it reads < 23.6 V at rest. Write the voltage per flight in the notes.

## 1. Terminals

**T1 (radios, mocap, RViz), up all session:**
```bash
ros2 launch controller_quad_load real_io_launch.py num_drones:=4 drone0_serial:=/dev/QUAD1 drone1_serial:=/dev/QUAD2 drone2_serial:=/dev/QUAD3 drone3_serial:=/dev/QUAD4 magnet_channel:=6 magnet_initial:=ON
```
**T5 (required for every tethered flight):** `python3 tools/fleet_monitor.py --drones 4` (ring z and tilt, per-drone armed, throttle, battery).

**T4 (bag), per flight** (`F` = flight name):
```bash
cd ~/multi_drone_control; F=h1; ros2 bag record -o results/rig/2026-10-01/$F /drone_{0..3}/motion_capture_state /payload/motion_capture_state /drone_{0..3}/ELRSCommand /drone_{0..3}/telemetry /fleet/command /fleet/abort /fleet/status /fleet/landed /drone_{0..3}/magnet /rosout
```

**T2 (control), per flight:** one block per test below. Every block starts with
```bash
cd ~/multi_drone_control && tools/clean_slate.sh --rig
F=<name>; D=results/rig/2026-10-01; export MDC_RUN_DIR=$PWD/$D/${F}_logs
```
**T3 (checks), per flight, about 10 s after the trackers print "Controller ready":**
```bash
cd ~/multi_drone_control; F=d1_free   # example (D1): must match T2's F for this flight
python3 tools/preflight.py --drones 4 --real --planner load_planner --max-ground-z 0.5 --battery-min 23.6
ros2 param get /load_planner pivot_offset; ros2 param get /controller_0 thrust_offset; ros2 param get /controller_0 throttle_max
# live filter of the planner lines that matter (leave running through the flight)
tail -F results/rig/2026-10-01/$F.log | grep --line-buffered -E "geometry:|datum|slot|SLOT OFFSET|elevation|measure_rod_len|pretension|solve status|near its bound|NOT lifting|timed out|FLEET|disarm|ESTOP|arc-creep|load down"
```
Expect: preflight GO; `pivot_offset` [0.0, 0.0, -0.04]; `thrust_offset` 0.185; `throttle_max` 0.8.

**Dry launch first (props off, not armed).** Launch the H1 block once, wait for "Controller ready" x4 and the
planner's `ready, waiting for mocap`, check nothing compiles (an acados build during a run starved the lab sim
into a pose timeout twice), run T3, then Ctrl-C and `tools/clean_slate.sh --rig`.

## 2. D1: free hover, drone 0, rod and magnet removed (0.475 kg), 20 s

Question: did the rod hanging in the downwash bias the thrust fit? If the rod's drag was in the fit, drone 0
without it hovers below the law. The whole 30 Sep deficit would be about 0.022 throttle below it.
Also the first closed-loop check of W12 (A3).

Take the rod and magnet off drone 0 only. Drones 1-3 stay on the floor, disarmed. Drone 0 >= 1 m from anything.

```bash
# T2
cd ~/multi_drone_control && tools/clean_slate.sh --rig
F=d1_free; D=results/rig/2026-10-01; export MDC_RUN_DIR=$PWD/$D/${F}_logs
ros2 launch controller_quad_load real_hover_launch.py num_drones:=1 hover_z:=0.8 thrust_ratio:=40.7 thrust_offset:=0.185 thrust_offset_v_slope:=0.022 kt_trim:=false 2>&1 | tee $D/$F.log
# T3
python3 tools/preflight.py --drones 1 --real --planner free_hover --max-ground-z 0.5 --battery-min 23.6
```
`thrust_ratio:=40.7` = 9.81 / (0.507 x 0.475) is how the 0.475 kg enters (real_hover_launch has no
`drone_mass` arg). Offset 0.185 is the rig default, as in RIG-0930-model-hover (drone 0 sat +3.7 cm there on
its 0.181 offset). ARM, TAKEOFF (panel; a one-shot topic pub missed free_hover on 30 Sep), hold 20 s, LAND.
Throttle cap 0.6 in real_hover_launch (no throttle_max arg; hover ~0.43; the panel turns red at 0.6).

Check after landing:
```bash
python3 - <<'E'
import glob, pandas as pd
f = sorted(glob.glob('results/rig/2026-10-01/d1_free_logs/logs/controller_quad_load/planner_drone0_*/log.csv'))[-1]
d = pd.read_csv(f); h = d[d.ref_z > d.ref_z.max() - 0.01]; h = h.iloc[len(h) // 3:]
V = h.battery_v.median(); u = h.thr_out.median()
law = 0.181 + 0.507 * 0.475 - 0.0219 * (V - 23.5)
print(f'rows {len(h)}  V {V:.2f}  thr_out {u:.4f}  law {law:.4f}  diff {u - law:+.4f}  z err {100 * (h.pose_z - h.ref_z).median():+.1f} cm')
E
```
(RIG-0930-model-hover predates thr_out; its registry row gives model throttle 0.274-0.284 against 0.279 predicted;
sim check R0792: -0.0004.)

| result | reading |
|---|---|
| \|diff\| <= 0.01 | the rod in the downwash did not bias the fit; the deficit is elsewhere (D2) |
| diff <= -0.015 | rod drag was in the fit; diff / -0.022 is roughly the share of the deficit it explains |
| takeoff (ref rise to within 5 cm of 0.8 m) > 5.5 s (model-hover drone 0: 4.5 s), throttle overshoot > 0.05 over hover | A3 falsified: stop, read the log |

Refit the rod and magnet on drone 0 (pivot 4 cm under the centre), redo its magnet check, back on plate 0.

## 3. D2: tethered holds, target 0.3 then 0.7, freeze on, z_ki 0 (two flights)

Question: does the ~0.8 carried fraction depend on height? The ring will sit below target (z_ki 0, and the
freeze holds the pull at breakaway); the ring's measured height is what counts. Steady means \|vz\| < 0.03 m/s
for >= 5 s; hold 15 s steady, then LAND.

```bash
# T2, D2a (then the same with F=d2_z070 and target_z:=0.7)
cd ~/multi_drone_control && tools/clean_slate.sh --rig
F=d2_z030; D=results/rig/2026-10-01; export MDC_RUN_DIR=$PWD/$D/${F}_logs
ros2 launch controller_quad_load real_control_launch.py num_drones:=4 load_mass:=0.86 drone_mass:=0.55 cable_len:=0.55 attach_radius:=0.225 attach_z:=0.0 pivot_offset_z:=-0.04 thrust_ratio:=35.2 thrust_offset:=0.185 thrust_offset_v_slope:=0.022 kt_trim:=false throttle_max:=0.8 handover_elev_deg:=45.0 pretension_s:=3.0 lift_ramp_vel:=0.1 load_traj:=hover z_ki:=0.0 target_z:=0.3 2>&1 | tee $D/$F.log
```
The geometry, thrust and gate args repeat the launch defaults on purpose, so the log shows them. Watch the
log lines in section 5. Record ring z and throttle per drone on the panel during the steady part.

After both:
```bash
python3 tools/thrust_fit.py --tethered results/rig/2026-10-01/d2_z030_logs results/rig/2026-10-01/d2_z070_logs --out results/rig/2026-10-01/d2_force.md
```
Reported, no bar: carried fraction per height band against 30 Sep (0.78-0.87 below 0.45 m, 0.99 at 0.52 m).
Both near 1.0: the deficit was lift-off dynamics or pre-pivot geometry. Low only at 0.3: height effect. Low at
both: a fixed law error when tethered.

## 4. H1, H2, H3 (1/3/5/9), C1

**H1: tethered hover 0.5 m, z_ki 0, 25 s hold.** Same T2 line as D2 with `F=h1`, `target_z:=0.5`, `z_ki:=0.0`.

**H2: as H1 with the safety net.** `F=h2`, `target_z:=0.5`, `z_ki:=0.4` (bound 0.15 and gate 0.25 are the
launch defaults; `z_taut_gate` stays 0.9, the pivot model is in). If `height integral ... is near its bound`
prints: finish the hold, LAND, no C1.

**H3: hover on plates 1/3/5/9, after H2.** Move the magnets to plates 1, 3, 5 and 9, counted from plate 0
(the +x plate) the same way as the even ring (plate 3 is drone 1's plate in both): drone 0 goes from plate 0
to plate 1, drone 2 from plate 6 to plate 5, drones 1 and 3 stay on 3 and 9. Ring placement unchanged (plate 0
toward world +x). Magnet check on drones 0 and 2. Then the H2 T2 line with `F=h3_1359` and
`attach_azimuths_deg:=30,90,150,270` added. Before ARM the slot line must read ~0 deg for all four:
`slot azimuth error (deg): d0 +0.x (plate 1), d1 ... (plate 3), d2 ... (plate 5), d3 ... (plate 9)`, each
within +-10 and no `SLOT OFFSET` line (a +-30 reading means a drone or magnet is one plate off: do not ARM).
Same pass bars and stop rules as H2. Afterwards move drones 0 and 2 back to plates 0 and 6 for C1 and redo
their magnet check.

**C1: slow circle, even ring, only after H1 and H2 each pass once.** C1 needs H1 and H2 to pass; H3 failing a
bar does not block C1 unless it is the same bar H2 failed. `F=c1`, `target_z:=0.5`, `z_ki:=0.4`,
`load_traj:=orbit traj_radius:=0.5 traj_speed:=0.125`. The orbit starts when the lift tops out, spins up over
4 s and runs until LAND; one lap is 25 s, so LAND about 30 s after it starts moving. The ring's circle starts at
its lift point, heads world +x, and is centred 0.5 m toward world +y: place the ring so there is >= 1.7 m to
the net on +y and >= 1.2 m on +x, -x and -y (ring path plus the drones 0.6 m further out).

Pass bars (card A):

| test | pass | sim twin / 30 Sep baseline |
|---|---|---|
| H1 | 0-1 solve failures; ref age > 0.3 s under 5 % of airborne time; heave p-p <= 6 cm; tilt mean <= 3, max <= 8 deg; hand-over 50-60 deg (pivot reading, recorded; > 60: section 6); rods at hand-over 0.53-0.58 each | twin R0783: 0 / 0.0 / 1.2 cm / 1.22 deg / 55 deg / 0.550 (fba2d7a R0810/R0811 the same). Rig model-f1: 25 failures, 14 cm, 6.5 / 16 deg, 53.6 deg, rods 0.587-0.613 |
| H2 | H1 bars, \|mean z - 0.5\| <= 2 cm over the last 20 s, \|z_bias\| <= 0.10 throughout | twin R0788: hold 0.500, \|z_bias\| max 0.013, 0 failures (fba2d7a R0816: hold 0.500, 0 failures; fleet disarm on LAND) |
| H3 (1/3/5/9) | H2 bars; slot line ~0 deg (within +-10) for all four before ARM | twin R0786 (z_ki 0): 0 / 0.0 / 0.5 cm / 1.41 deg / 55.4 deg / 0.550, hold 0.504 (fba2d7a R0813 the same, tilt 1.42) |
| C1 | ring tilt mean <= 3 deg over the lap, \|z err\| <= 5 cm, xy err <= 10 cm, 0 solve failures, no release | twin R0789: lap tilt 1.22 deg, \|z err\| 1.3 cm, xy 3.1 cm (max 4.8), 0 failures (fba2d7a R0815: 1.22 deg, 1.3 cm, xy max 5.1) |

After each tethered flight (T3):
```bash
cd ~/multi_drone_control; F=h1   # must match T2's F for this flight
python3 tools/metrics.py tethered results/rig/2026-10-01/${F}_logs --log results/rig/2026-10-01/$F.log
grep -c "solve status" results/rig/2026-10-01/$F.log
```
Gives airborne time, hold z, tilt max, max vz, solve failures and carried fraction (the log is read before the
next lift in any case).

## 5. What to check in T2 / the T3 filter, in order (every tethered flight)

| line | expect | act on |
|---|---|---|
| `[planner] geometry: n=4 cable_len=0.550 attach_radius=0.225 attach_z=0.000 load_mass=0.860 pivot_offset=[0.0, 0.0, -0.04] ...` | exactly this | any other value: Ctrl-C, fix the line |
| `[planner] load yaw datum latched at +x.x deg` | within +-45 of 0 (decision 1) | near +-180: re-place the ring |
| `[planner] slot azimuth error (deg): d0 ... (plate 0), ...` | every value within +-10 (H3: plates 1/3/5/9) | `SLOT OFFSET:` line: do not ARM (section 0) |
| `[planner arc-creep] ref=..deg target=45deg ... measured: d0:.. d1:.. d2:.. d3:..` | the four rise together, 10-16 deg ahead of ref (known creep lead) | one drone far behind: LAND |
| `[planner] elevation XX.Xdeg reached (target 45, tol 8) — coupled planner active` | record XX; twin 55.4-55.6 (R0783-R0789, R0810-R0816), bar 50-60 (pivot reading) | `elevation timeout at` below 30: `NOT lifting`, press LAND |
| `[planner] measure_rod_len: rods [...] m from mocap at handover (typed 0.550)` | each 0.53-0.58 (pivot in) | `keeping typed cable_len ...`: rods not trusted; LAND, read the log |
| `[planner] pretension: ramping every rod to its share over 3.0 s ... (length gates [...])` | the four "thr" rise together, ring floats up level | one side lifts first, tilt > 5 deg: LAND |
| `[planner] pretension: ring broke free at NN% of the planned pull ...; pull held there` | record NN (log-only now; 30 Sep 60-80 %) | - |
| `[planner] pretension done (ring z ..., lifted ...) — starting lift ramp` | then the height ramp | - |
| `[planner] solve status N (M ms) — publishing the last horizon shifted (... s)` | none (H1 allows 1) | the 3rd one: LAND |
| `[planner] height integral +0.xxx m is near its bound` (H2, C1) | none | finish the hold, LAND, no circle |

## 6. Stop rules (card A) and session rules

LAND now on: a magnet release; ring tilt > 5 deg in the pretension or > 12 deg in flight (T5 ring tilt);
a visible run-up of the ring (as in f6); vz is checked in the read-back; 3 or more `solve status` lines; a creep timeout or refusal.
- After one release: no further lift until the log has been read. After two releases: no more lifts tonight.
- The same bar failing twice: stop and report.
- Hand-over elevation alone does not stop a flight. Above 60 deg: finish the flight; before the next lift
  read the `arc-creep` lines (which drone leads, by how much) and `measure_rod_len` (a rod read long lifts
  the elevation). Above 60 on two flights is the same bar failing twice: no C1, stop and report.
- LAND with rigid rods (R0793, and R0816 on the nominal twin) is unchanged: after "load down, rods slack", watch each drone's descent. A
  fleet disarm after the ring is down is recorded (not a stop rule for later flights), then props off and
  the section 7 drop check on every drone.
- No soft-hold command exists (card A wants "ramp the pull down and hold, then LAND"): the stop action is LAND
  (the planner lowers the ring at 0.2 m/s and blends the pull out).

## 7. If things go wrong

- **A drone tips, climbs away after a release, or anyone is in the cage: DISARM** (panel DISARM, or SPACE with
  focus in the RViz 3D view). CLI kill from any terminal:
  `ros2 topic pub -r 10 -t 10 /fleet/command std_msgs/msg/String "{data: ESTOP}"`.
- **Anything else from the stop rules: LAND** (panel). The planner lowers the ring at 0.2 m/s, then the drones.
- **Then:** Ctrl-C T2 and T4, `tools/clean_slate.sh --rig` (spares T1, the bag and their shared memory). If
  T1 misbehaves too: Ctrl-C everything, `tools/clean_slate.sh`, relaunch T1 from section 1.
- **Read-back before the next lift:** `python3 tools/metrics.py tethered ...` and
  `grep -E "solve status|SLOT OFFSET|elevation|measure_rod_len|pretension|disarm|ESTOP|timeout" $D/$F.log`;
  after a relaunch, T3's preflight and the three `ros2 param get` lines again. Send the flight name and time;
  the log is read before anything else is flown.
- **After a drop:** props off, check each rod pivot and magnet, redo the magnet check on that drone.

## After the session

Registry rows (one per flight): RIG-1001-d1, RIG-1001-d2-z030, RIG-1001-d2-z070, RIG-1001-h1, RIG-1001-h2,
RIG-1001-h3-1359, RIG-1001-c1, each against its baseline above. Copy `results/rig/2026-10-01/` whole; nothing runs in sim until the
rig stack is down.

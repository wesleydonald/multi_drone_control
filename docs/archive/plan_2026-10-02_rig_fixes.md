# Plan: fixes after the 1 Oct rig day (draft for Wesley, 1 Oct evening)

Inputs:
- Wesley's five items, with his answers;
- the code survey (arming, renaming, tilt, figure-8, bags);
- Sun et al. 2025 (TU Delft);
- a root-cause study of the 1 Oct logs (registry RIG-1001-*).

Nothing has been changed in the code yet.

## Part A: small fixes (no flight-loop change; pytest + one desk test)

| # | change | where | check |
|---|---|---|---|
| A1 | Drones shown as 1-4 (labels only; topics stay /drone_0..). One helper `label(i)` = `i+1` | arm_panel.cpp:290,466 (`D%1`, i+1); main.py `_announce` and ARM/abort lines; fleet_monitor; preflight lines; planner slot/creep/rod log lines; metrics/plot labels; rig sheet. Log CSV columns stay d0_.. (old runs keep loading) | pytest; read the panel and log lines on a desk launch |
| A2 | Arming that confirms with the flight controller | elrs_interface.py: (1) read then ALWAYS write the frame each tick (today an incoming telemetry byte skips the write, :320-327); (2) arm channel = idle +- range (172/1811), not 0/2000, which is outside CRSF; (3) on an ARM edge hold the switch low 0.3 s, then high; (4) armed = the FLIGHT_MODE string (Telemetry.mode) without the `*` suffix, seen within 1.5 s; else low 0.5 s and retry, up to 3 times; (5) publish `/drone_i/fc_armed` + reason (`!ERR*` = Betaflight arming disabled, `no telemetry`, `still *`); (6) drop the per-message prints (:221,:227). main.py: wait up to 5 s for every `fc_armed` (real mode only; sim has no FC telemetry), report per drone with its radio ("drone 2 (QUAD2): FC says !ERR* after 3 tries"), TAKEOFF needs `fc_armed` | unit tests for the mode parser and the arm state machine (fake serial); desk test on the rig, props off: ARM x10 per drone, count first-press arms |
| A3 | Preflight resting tilt 15 -> 20 deg (Wesley's word, 1 Oct) | tools/preflight.py:225 | pytest |
| A4 | Figure-8 a = 1.0 m at 0.25 m/s (67 s sweep, ring path 2 x 1 m) | command line on the rig sheet (`traj_radius:=1.0 traj_speed:=0.25`); the launch default 0.5 is shared with the orbit, so it stays | twin run on the lab PC |
| A5 | One script per flight so nothing is lost to a cut-off paste | `tools/rig_flight.sh F <launch args>`: sets MDC_RUN_DIR, tees the log, and starts the bag with every topic (the 1 Oct bags hold only mocap: the pasted record line was cut, bash history) | dry run on the desk |
| A6 | Serial defaults agree with the labels | real_io_launch.py:158 default QUAD{i+1} like real_mode.py:141 | desk launch |

## Part B: the height and xy error

### What the logs say (root-cause study)

**Main cause (verified in code; the size is a correlation over 6 flights).**
- At breakaway the planner freezes the cable feedforward at the pull fraction it had reached: `_ff_cap`, planner_node.py:929.
- It then caps every later publish with that value for the rest of the flight (:1420). The cap is only reset when the planner phase starts again.
- So the trackers are told the rods pull 7-27 % less than planned. The ring sinks until the planner's slow position action makes up the difference.

| flight | breakaway (= cap) | ring short of target, no integral |
|---|---|---|
| h5 | 0.93 | smallest (6-15 cm) |
| h1 | 0.80 | 5.5 cm (at 0.5 m, still sagging) |
| c1 | 0.83 | 18 cm |
| h6 | 0.77 | 29 cm |
| h2 / f8 | 0.73 | 29-32 cm |

- In the twin the ring breaks free at about 97 %, so the cap is about 1 and the twin never showed the error.
- The 30 Sep "carried 0.8" force deficit was flown with the same freeze.
- The thrust map is not the main term: total thrust from the map is 0.95-1.19 of the weight over the hold.

**Second cause.** The OCP wants the rods at 45 deg (w_s 25, nominal tension 3.98 N); they sit at 57-65 deg.
- At 60 deg the nominal tension lifts more than the ring weighs: 10.3 N against 8.44 N.
- So the plan expects a climb and a widening that never happen.

**Third cause.**
- Every drone tracks its reference within 1 cm by construction: OCP node 0 is the measured state.
- The load position is held only by what OCP nodes 1-20 plan: a P-only loop of about 4-6 N/m.
- So any steady force becomes an offset. The xy drift is such a force: it points to world -x in all six flights, also with the system rotated 124 deg, and drops from 8 to 3 cm at x +1.1 m.
- Likely air recirculation or a local mocap level error (inferred).

**How TU Delft avoid this.**
- Their planner has no integrator. Each drone's onboard INDI adds f_ext = m*a_imu - thrust (rotor speed, 300 Hz). That removes cable-force and mass errors and wind: wind moved the load 0.048 -> 0.055 m.
- We have no rotor speed and no onboard code. The equivalent we can build is on the ground (B4).
- Note: a load-level force estimator (OCP p_dist) was deleted on 2026-09-23 after R0347-R0366. It failed under a WRONG kT; the affine map has fixed that since.

### Steps (each one variable; registry row per run; lab PC for Gazebo)

| step | what | runs | falsifier | needs |
|---|---|---|---|---|
| B1 | **Prove the cap in sim.** A test-only planner param `ff_cap_force` (default off) that sets the cap to 0.75 at breakaway. Twin hover at 1.0, z_ki 0: cap 1.0 vs 0.75 | 1 SIL pair + 1 Gazebo floor-start pair | the 0.75 twin sits less than 10 cm below the 1.0 twin: the cap is not the cause, go to B3 | - |
| B2 | **Fix the cap.** After the pretension the cap ramps back to 1 over 2 s once the lift ramp starts. The freeze still stops the run-up at breakaway | Gazebo floor start x2 (cap 0.75 forced, fix on), then SIL regression on the twin set | ring more than 5 cm short with z_ki 0 at 1.0 m, or a pull run-up (ring vz > 0.3 m/s) after breakaway | **your word**: the freeze is your locked rig default (decisions.md:75) |
| B3 | **Nominal rod angle = the hand-over angle** (about 55 deg, measured at the hand-over) instead of 45 deg, for s_ref and the nominal tension | SIL + Gazebo floor start x2 | hold elevation still off by more than 5 deg, or a worse hand-over | **your word** (changes the reference geometry); critic not needed (it is a reference, not a law) |
| B4 | **Disturbance rejection (TU Delft-like), offset-free planner.** Estimate the load's external wrench (force first; torque later) from mocap: w = m_L (a_L - g) - sum(t_i s_i), low-passed about 1 Hz, held while rods are slack or during the hand-over. Feed it to the OCP as a constant parameter over the horizon. The planner then plans tensions that cancel it: wind, a mass dropped into the ring, the -x push. Bounded (e.g. +-3 N per axis) with a WARN | card + **critic** (new estimator in the flight loop); SIL matrix: step mass +0.2 kg mid-hover, constant 1 N lateral push, orbit with push, then Gazebo floor start x2 per case (Tejen's xy-disturbance tool for the push) | after a +0.2 kg step the ring is more than 3 cm off in z after 5 s, or more than 5 cm off in xy with a 1 N push; any oscillation the estimate drives (tilt > 5 deg) | your word on the card |
| B5 | Only if B4 leaves per-drone error: a ground-side per-drone force observer (mocap acceleration vs the affine map), horizontal only, in the tracker | later | - | critic; close to the locked kT adaptation, so your call |
| B6 | z_ki: keep as the safety net until B2/B4 are proven; then decide whether to drop it | - | - | your word |

After B4, ZBias, the xy drift and the "force deficit" should all be explained by one estimator, or B4 has failed.

## Part C: next rig visit (draft)

| flight | purpose | bar |
|---|---|---|
| desk | A2 arming: ARM x10 per drone, props off | first-press arm 10/10, every failure named with its FC reason |
| H1 | hover 1.0, the B2 fix, z_ki 0 | within 5 cm of 1.0, tilt < 3 deg |
| H2 | same, B4 on | within 3 cm z, 5 cm xy |
| D1 | B4 on, drop a known mass (100-200 g) into the ring mid-hover | back within 3 cm in 5 s |
| D2 | B4 on, a fan on the ring from one side (if one is available) | xy within 5 cm |
| C1 | orbit r 0.5 at 1.0 m, B4 on | xy err < 10 cm |
| F1 | figure-8 a = 1.0 at 0.25 m/s | xy err < 10 cm, visible |
| M1 | free single drone at x 0 and x +1.1, hold 30 s each (props of the others off) | looks for a world -x push without the ring |

Plus the mocap level check before flying: a marker on a string about 1 m above a floor marker, at the centre and at +1.1 m.

## Order and effort

1. Part A (one session, no runs).
2. B1 (decides B2/B3).
3. B2.
4. B4 card + critic, then the SIL matrix and Gazebo.
5. Rig sheet for Part C.

Gazebo runs on the lab PC once Tailscale SSH is re-approved.

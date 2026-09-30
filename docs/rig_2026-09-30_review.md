# Rig day review, Wed 30 Sep 2026

The first rig lifts of the 0.86 kg ring with four drones. Every number here is from the flight logs; the run ids refer to `results/registry.csv` (RIG-0930-*, R0765-R0771). The code is all uncommitted, on `real-world-testing`.

## Summary

- **The ring was lifted and carried** for the first time: f2, f6, f7, f10, f11 and model-f1. f7 was the first sustained, level carry: 12.6 s at 0.28 m.
- **The goal was not reached:** a steady hover at the target height with no oscillation.
- **Seven bugs were found and fixed** on the day (table 2).
- **The drone thrust model was measured and replaced by an affine law.** Free hover now holds within 1 cm with no integral and no trim.
- **What remains is the tethered model and the planner.**
  - The planned rod pull is about 23 % too high at breakaway.
  - The planner failed to solve 25 times in the last flight.
  - The ring heaves at 0.17 Hz with 14 cm peak-to-peak.

## 1. Flights

| id | what | result |
|---|---|---|
| T1a / T1b | free hover, typed kT 24, 4 drones | throttle 0.44-0.46; drones 14-17 cm low (kT really ~21.7); **the manager crashed at DISARM** (bug 1); drone 1's radio link was down |
| R3e f1 | creep and hold, even ring | **fleet flew into the middle**: the creep anchored each drone on the opposite plate (bug 2) |
| R3e f2 | same, fixed | clean: 45 deg in 3.7 s, 10 s hold |
| R3b0 f1 | first lift, cap 0.6 | ring never left the floor: the cable feedforward was zeroed while the ring rested, a deadlock (bug 3); throttle only 0.50 |
| R3b0 f2 | lift, cap 0.8 | **lifted**, but jumped 0.17 -> 0.73 m in 1 s and held 0.91 m (target 0.6); 12.9 deg tilt. Only drone 0 received the pull flag (bug 4) |
| R3b0 f3 | kT 25 typed, rod 0.53 | only drone 0 pulled (bug 4); ring stayed down |
| R3b0 f4 | subscription fixed, kT 25 on 23.3 V packs | creep stalled at 25.6 deg and the lift started anyway; drones pulled one after another; **3 magnets released** |
| hold f1 | kT 25, rod 0.47, trim off | creep stalled at 18 deg (the gain was too high for the packs); fleet slid 0.6 m |
| hold f2 | kT 21.7 | clean hold: 42 deg hand-over, z sd 0.8-1.2 cm |
| lift f5 | all trackers gated, from a clean 45 deg | the **pull stepped on in one tick** (0.44 -> 0.67 throttle): ring launched at 0.29 m/s, 21 deg tilt, **2 magnets released** (bug 5) |
| R0766-R0771 | Gazebo, after the pretension fix and the critic fixes | even ring and 1/3/5/9 PASS (tilt 0.37 / 0.92 deg); the refusal hold PASS (drift <= 1 mm) |
| lift f6 | pretension 3 s, fresh packs | **level breakaway** (tilt <= 4 deg), but at 67 % of the planned pull; the ramp to 100 % ran the ring up at 0.38 m/s; the hard stop released **drone 1's magnet** |
| lift f7 | freeze the pull at breakaway (77 %) | **first sustained carry**: 12.6 s, ring 0.278 m (target 0.35), bouncing sd 4.5 cm, tilt 4.9 deg mean; the integral hit its 0.15 bound |
| lift f8 | target 0.6 | ring stayed at 0.24-0.30; drones sat 10 cm ABOVE their refs; measured rods rejected (bug 6); integral gated off (miss > 0.25) |
| lift f9 | rod trust band widened | rejected again (spread exactly 5.0 cm) |
| lift f10 | spread 0.08 | rods accepted; ring 0.19 m; integral still off |
| lift f11 | new rig defaults, integral free (0.4 bound, 0.6 gate) | ring mean 0.648 but **swinging 0.29-0.86 m**; integral at its bound. Supervisor: the integral is a backdoor, fix the model |
| ladder1 / ladder4 | thrust identification, hung masses 0/98/196 g, 22.7-24.6 V | **throttle = a_i + 0.507 M - 0.022 (V - 23.5)**, a_i = 0.181/0.185/0.187/0.187, rms 0.003 over 13 points |
| model-hover | free hover with the affine map | **within 1 cm** (drone 0 +3.7), throttle as predicted |
| model-f1 | tethered, affine map, no integral, no trim | breakaway 77 %, ring 0.31-0.35 (16 cm low), **0.17 Hz heave, 14 cm p-p, tilt 6.5/16 deg, 25 planner solve failures** (from 0.5 s after breakaway, every ~0.4 s) |

## 2. Bugs found and fixed on the day

| # | bug | fix | proof |
|---|---|---|---|
| 1 | Fleet manager died at the first DISARM after another log level (rclpy fixes the severity per call line) | `_announce` gets a separate line per level | real-logger unit test |
| 2 | Creep fed drones in physical order against slot-ordered plates: every drone anchored on the opposite plate | creep in slot order, published per `slot2drone` | test built from the rig geometry |
| 3 | Tracker zeroed the cable feedforward on a resting ring: lift deadlock | superseded by 5 | f2 lifted |
| 4 | Only drone 0 subscribed to `/payload/desired_position` | superseded by 5 | f3/f4 logs |
| 5 | The pull arrived as a step when the gate opened late | pretension: every rod eased 0 -> share together over 3 s, planner-owned flag `/fleet/cable_ff_active`, equal pull (gate 1) after the hand-over, freeze at breakaway, lift refused below 30 deg, LAND and network clear the flag (critic, 3 must-fix) | R0766-R0771 |
| 6 | Measured rods rejected by a +-15 % / 3 cm guard keyed to the typed length | `rod_tol_frac`, `rod_spread_m` launch args (rig 0.25 / 0.08) | f10 accepted |
| 7 | Hard-coded 0.25 m integral gate and 0.15 bound: the integral never acted on 0.3-0.4 m misses | `z_i_gate` launch arg | f11 (but see section 4) |

Also:
- Throttle cap is now a launch arg (`throttle_max`), and the panel and monitor read each drone's cap.
- Sim RViz is identical to the rig's (MAGNET toggles, sim_magnet).
- The weld-variant generator is aligned with R0552.
- A figure skill was added and the R0209 thesis figure rebuilt.

## 3. What was measured

| quantity | value | used for |
|---|---|---|
| drone (pack) / as flown (+ rod + magnet) | 0.475 / **0.550 kg** | `drone_mass` 0.55 |
| ring | 0.86 kg, diameter 0.51 m, 40 mm thick | |
| magnet radius | **0.225 m** (0.255 - 0.030) | `attach_radius` 0.225 |
| rod | **0.55 m**; pivot **4 cm below the drone centre** (not modelled) | `cable_len` 0.55; mocap reads 0.51-0.61 centre-to-plate, varying with angle |
| thrust | **affine + voltage** (section 1) | `thrust_offset` 0.185, 0.022/V, gain 35.2 above it |
| magnets | 10-12 N rated (Wesley) | released under jerks, never under steady load |
| breakaway fraction of the planned pull | 67 / 77 / 70 / 80 / 70 / 60 / 77 % | the tethered model plans too much pull |
| carry throttle | 0.55-0.60 (real) | matches the affine law (0.569-0.575 predicted) |

## 4. Open problems, most important first

1. **Planner solve failures (25 in model-f1).**
   - They started 0.5 s after breakaway and recurred every ~0.4 s until LAND; each one holds the last reference.
   - Only model-f1 had them, the first flight on the new geometry defaults (magnet radius 0.225, rod 0.55, mass 0.55) and with measured rods 0.59-0.61 accepted. f9 had 1, the others 0.
   - The model the planner is given is inconsistent with the state it measures, most likely the unmodelled 4 cm pivot. Strongest suspect for the heave.
2. **Tethered model error: breakaway at 60-80 % of the planned pull.**
   - The thrust map is now right (free hover within 1 cm; the drones track their refs to 2 cm tethered), so the remainder is on the planner side.
   - Candidates:
     - the pivot offset (rod tension acting 4 cm below the centre, and a rod geometry the OCP does not have);
     - the rod and magnet weight split between drone and ring once welded (drone_mass 0.55 includes a magnet the ring carries);
     - the hand-over at 52-54 deg instead of 45;
     - floor contact at breakaway.
3. **The breakaway freeze leaves the ring low in flight.** The pull stays at 60-80 % for the whole flight: model-f1 sat 16 cm low with no integral.
4. **Slow heave (0.17 Hz, 14 cm) and tilt spikes.**
   - Drones track their refs, so this lives in the planner references: frozen refs after solve failures, the open loop in load height, and the integral windup in f11.
   - Sim cannot show it yet: its plant is proportional thrust, pivot at the centre, rod 0.5, magnets at 0.25, drone 0.64 kg.
5. **The creep over-shoots the hand-over** (52-56 deg for a 45 deg target) with 0.53/0.55 typed rods; it stalled when the gain was too high for the pack. It is sensitive to the typed rod and to kT.
6. **Magnets release under jerks.** Any breakaway or stop above about 0.2 m/s has cost a magnet.
7. **kt_trim oscillation** (Wesley): it did not change the gain in f3/f4. Now off on the rig, and not needed with the affine map.

## 5. Sim vs rig differences to remove (Wesley: sim drones should behave like the real ones)

| | sim now | rig |
|---|---|---|
| thrust law | proportional (a = 88.6 u, 'auto' kT 83.1) | affine 0.185 offset, 19.4 N/unit above it, -0.022/V |
| battery | none | 22.6-24.6 V over a flight |
| drone mass | 0.64 | 0.55 as flown |
| rod | 0.5 m, pivot at the body centre | 0.55 m, pivot 4 cm below the centre |
| magnet radius | 0.25 | 0.225 |
| magnets | rigid welds | release under dynamic load (10-12 N) |
| rate loop | Betaflight 100/100 model | same profile; step response not yet measured |
| mocap | ideal 120 Hz | real latency and noise, not measured |

## 6. Rig launch defaults changed today (Wesley's word)

`real_control_launch.py` (the sim launches are unchanged):
- **Fleet:** `num_drones` 4.
- **Masses and geometry:** `drone_mass` 0.55, `cable_len` 0.55, `attach_radius` 0.225, `attach_z` 0.0.
- **Thrust map:** `thrust_ratio` 35.2, `thrust_offset` 0.185, `thrust_offset_v_slope` 0.022, `kt_trim` false.
- **Height integral:** `z_i_max` 0.4, `z_i_gate` 0.6.
- **Lift:** `lift_ramp_vel` 0.1, `throttle_max` 0.8, `pretension_s` 3.0.
- **Rod trust band:** `rod_tol_frac` 0.25, `rod_spread_m` 0.08.

`real_hover_launch.py` gained the thrust-map args (default 0 = the old map).

## 7. Questions for the plan

In section 8 of the chat, and summarised here:
1. Sim plant changes: all of section 5 at once, re-baselining every sim run?
2. After breakaway: keep the freeze, ramp back to 100 %, or remove the freeze once the model is right?
3. Height integral on the rig: off, or kept small as a safety net?
4. Pivot joint type (ball, hinge, fixed?) and the magnet end (rigid on the plate, or pivoting?).
5. Mechanical options for the magnets and the pivot.
6. Next lab visit: date and duration; hover only, or also three drones / circle.
7. Commit today's work on `real-world-testing`?

## 8. Decisions (Wesley, 30 Sep evening)

1. **Sim plant:** apply all the rig differences: affine thrust with voltage sag, drone 0.55 kg, rod 0.55 m with the pivot 4 cm below the centre, magnets at r 0.225. Then re-fly the key baselines once.
2. **Pull after breakaway:** fix the tethered model so breakaway is near 100 %, and drop the freeze.
3. **Height integral on the rig:** a small safety net only: z_ki 0.4 with the original 0.15 m bound and 0.25 m gate. The model is judged with it off.
4. **Rod joints:** both ends are free to an extent: the pivot 4 cm under the drone and the magnet end on the plate. Range of motion to be confirmed.
5. **Rod joints:** both ends are close to free ball joints (the pivot 4 cm under the drone, and the magnet on the plate).
6. **Next visit:** a steady four-drone tethered hover first, then a slow circle if the hover works early.
7. **Hardware:** unchanged (same magnets, rods and pivot).
8. **Commit:** today's work to be committed on `real-world-testing`.

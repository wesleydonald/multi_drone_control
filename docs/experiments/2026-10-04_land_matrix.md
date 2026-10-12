# land_matrix (4 Oct 2026): LAND with land_unwind on and the tracker box at zero, before the 7 Oct rig visit

## 1. Question

Wesley, 4 Oct: "turn land_unwind on and run the LAND matrix". With both of today's defaults (tracker
node-0 box at zero, `land_unwind` true), does every arm land without a tip?

## 2. Hypothesis

`land_unwind` moves each drone's reference out along its rod circle after load-down. This removes the
rod shove behind the twin's LAND tips:
- R0793 and R0816, 2 of 32 landings with the old box;
- R0981, 1 of 21 with the box at zero.

It did so in 3 of 3 runs on 3 Oct (R0848/R0849/R0851, old box): off-reference 0.5-0.7 cm against 15.8-20.0,
tilt after the blend 2.9-3.1 deg against 4.5-5.6. Expected: the same with the box at zero, and no abort.

## 3. The one variable

`land_unwind` true, now the launch default (common.yaml). Each arm is its existing config with
`land_unwind: true` added and nothing else changed (`*_lu.yaml`). The box is at zero by default.

Arms:
- three drones: hold (`b2_cap075_fix_n3_x0b0`, z_ki 0); the abort arm (`_pushz_x0b0_zki`, z_ki 0.4);
  `_mapneg_x0b0_zki`; `_map_zki`; `_orbit_x0b0`;
- four drones: even hold (`b2_cap075_fix`), plates 1/3/5/9 hold (`w4_3915_hover`), detach 4 to 3
  (`w4_3915_detach`), orbit (`w4_even_orbit`).

## 4. Baseline

The same configs without `land_unwind`, box at zero, landing 4 Oct (R0960-R0986): 1 abort in 21 (R0981);
drone off its reference after load-down 14-20 cm (land_unwind card).

## 5. Pass / fail numbers (load down to disarm, `land_baseline.py`)

| metric | confirms (every run) | falsifies (any run) |
|---|---|---|
| LAND | /fleet/landed, no abort | any abort |
| worst drone off its reference | <= 5 cm | > 8 cm |
| worst drone tilt after the 1.2 s blend | <= 6 deg | > 10 deg |
| ring xy shift | <= 1 cm | > 2 cm |

18 runs cannot prove a rate of a few per cent; the claim is the mechanism (no shove) on every arm.

## 6. Repeats

2 per arm (Gazebo, floor start).

## 7. Cost

18 Gazebo runs, about 75 min on the laptop.

## 8. New code?

No. A default change, on Wesley's word.

## Results (4 Oct; `tools/land_metrics.py`)

18 landings (R0987-R1008; voids R0989, R0993, R0996, R1001 re-flown):

| arm | runs | outside ref (cm) | inside lag (cm) | tilt after blend | outcome |
|---|---|---|---|---|---|
| 3 drones: hold, pushz, orbit | R0987/R0999, R0988/R1000, R0992/R1004 | 0.5-0.6 | 11.6-13.0 | 2.7-3.3 deg | 6/6 landed |
| 3 drones: map- | R0990/R1002 | 0.6 | 23.0-23.4 | 4.9-5.0 deg | 2/2 landed |
| 3 drones: map+ | R0991 / R1003 | 0.5 / 1.5 | 12.7 / 10.1 | 2.9 / 70.2 deg | 1 landed, **1 ABORT** (survivor) |
| 4 drones: even, 1/3/5/9, orbit | R0994/R1005, R0995/R1006, R0998/R1008 | 0.5-0.6 | 11.6-14.2 | 2.4-3.5 deg | 6/6 landed |
| 4 drones: detach 4 to 3 | R0997 / R1007 | 11.3 / 4.1 | 11.5 / 11.7 | 77.3 / 3.2 deg | **1 ABORT** (departed drone), 1 landed |

Baseline without the unwind, box at zero: outside 17.5-19.8 cm, tilt about 5 deg, 1 abort in 21 (R0981).

**Verdict.** The mechanism is confirmed: no shove in 16 of 16 clean landings. The falsifier fired on 2
runs, so the claim "every arm lands" is FALSIFIED.

The two aborts have different causes:
- **R1003, survivor.** At load-down the cable feedforward steps off: planned rod accel (-4.0, -5.5) to
  0, throttle 0.455 to 0.251. The rod still acts, and the tracker's pitch stick oscillates and grows to
  full deflection. R0981, without the unwind, shows the same. Not addressed by the unwind.
- **R0997, departed drone.** It lands first and keeps tracking a reference 10 cm below the floor at
  throttle 0.45 (grounded=1). Its roll oscillation grows, and it flips. The unwind lengthens the
  survivors' landing, which exposes it; R1007 landed clean.

Proposed (Wesley to decide):
- ramp the cable feedforward off over about 1 s at load-down instead of the step;
- idle a drone (zero rates, idle throttle, stay armed) once it is on the floor;

then re-fly this matrix.

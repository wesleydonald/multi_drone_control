# Why the sideways integral is needed: planner, tracker and plant audit (31 Oct 2026)

Wesley, 31 Oct: "why is the integral even needed at all? I feel like there could be something else that is wrong that
causes those offsets, either the planner is wrong or the trackers."

Read-only audit of the code, plus offline work on flown simulation logs. No run was flown and no Gazebo or ROS launch
was started. This file is the only repo change. The real flights are another agent's job. Tree: week8 at 9f20f8b.

Signs in every table: roll + means plate 3 up (plate 3 carries the T's middle carrier). y < 0 means the ring is off
its point towards plate 3. Settled means the last 5 s before LAND. Forces in N, tilt in deg, offsets in mm.

## Bugs and bookkeeping errors, first

Files are in `src/mpc_planner/mpc_planner/`, `src/tracker/tracker/` and
`src/dissipative_planner/dissipative_planner/`. "Effect" is the settled roll of the T, from the offline solver of
section 4.

| # | Where | Effect on the T (deg) | Real flights? |
|---|---|---|---|
| 1 | `load_cable_dynamics.py:172-175`, `reference_builder.py:244-254` | +1.3 with floor, all of +1.2 without | Yes |
| 2 | `geometry.py:239` | R3000 tilts the other way, -5.19 | No, `rod_mass` 0 |
| 3 | `tracker_node.py:1688` | R3206: -5.34 without it, -3.75 with it | Yes |
| 4 | `planner_ocp.py:63,167-170`, `geometry.py:240` | +5.5 to +5.9 | Yes |
| 5 | `planner_solver.py:434,448`, `geometry.py:94` | Lowers the roll by 0.1 to 0.4, not a cause | Yes |
| 6 | `dissipative_node.py:3316` | Latent | No, `rod_mass` 0 |
| 7 | `reference_builder.py:254` | None on the roll | Yes |

1. **The integral's force has the wrong point of action.** In the model it acts at the ring's CoG, as an acceleration
   of the whole system. The real force it stands for acts at the plate, in the attach plane. The pair's lean that
   balances it sits in the attach plane too, above the CoG, so the model has a roll moment the plant does not have.
2. **The "level" share is not level when the rod is in the plan.** `balanced_tensions` does least squares over force
   rows (N) and moment rows (N m) in one vector. On a T with `rod_mass` on, the force rows win (table in section 5).
   Its docstring promises a level load. With `rod_mass` 0 an exact solution exists, so this stays hidden.
3. **The pull clamp's premise is false.** `ac[2] = min(ac[2], 0.0)`, "a rod only pulls down on its drone". The
   measured term is `g - kT u`, and kT is set for drone plus rod. It goes positive with the rod still pulling down,
   whenever the pull is under the rod's own weight. The clamp then removes the tracker's offset-free action in height.
   There the roll becomes sensitive to the thrust map: a 0.002 throttle offset moves it 1.2 deg.
4. **The tension floor.** Not a coding error, a modelling choice. A slot whose level share is zero is planned at 0.1
   N, and no such pull exists in the plant.
5. **The tracker's "steady error" is a bookkeeping term.** The planned drone centre at node 0 uses the planned thrust
   attitude, not the measured one. That is the whole of the logged 1-4 mm.
6. **A join recomputes the shares without `rod_mass` and `rod_lam`** (found by the attach analysis).
7. **The height part of the integral has almost no authority.** It scales the tension reference, so it moves the ring
   only 0.6 mm per N on the T. `int_fz` is mostly the release dip, integrated and kept. It is not a learned weight.

Checked and found right: the planned drone is the rod pivot, the rod length, the plate radius and height, the ring
CoG, the slot mapping, every sign, the feedforward cap, the cable cap, the throttle cap and the tautness gate (section
2).

## Verdict

**The planner's model is where the disagreement sits, not the trackers.** A hinged rod with mass pulls its plate
outward by w cot e on top of what the massless model says. It cancels on even formations and not on a T, and that is
the force the integral learns. Two things turn it into tilt: the integral acts at the CoG and not at the plate, and
the floor adds a pull the plant lacks. With the integral off the ring gives way, and a second error cancels the floor,
which is why R3202 looks level. Offline, with rods of 1 g or with the rod correctly in the plan, the integral learns
nothing and the ring is level on its point. Sure of the statics and the numbers, less sure of the stability of the
roots. Nothing was flown.

## What is shown and what is inferred

**Shown by the code and logs:**
- The plant's statics close from the tracker logs alone, and give the formula of section 1 (section 3).
- Replaying the real planner at a flown state gives the logged tensions and `plan5` (section 4).
- The real tracker's first control is about zero at flown states (section 4).
- The xy integral is about zero on even formations and about one rod's force on uneven ones (section 3).

**Inferred, by offline solves on the planner's own OCP and the tracker's own OCP:**
- The prediction tables and every "switched off" number (sections 4 and 5).
- The stable and unstable reading of the T's roots. The solver finds the roots, and the flown drift gives the
  stability.
- That a rod-free plant or a correct rod model removes the need for the integral. No such run has flown (section 7).

## 1. The disagreement in one formula

A hinged rod with mass cannot hang at an angle without pulling sideways at its lower end. It has to hold up its own
lower share, w, and it can only do that along itself. For each plate, with elevation e:

| | Up at the plate | Outward at the plate |
|---|---|---|
| Planner (`rod_mass` 0) | t sin e | (up) cot e |
| Plant | F_z | (F_z + w) cot e |

w = (1 - lambda) m_r g = 0.5395 N, from the SDF: rod plus magnet 0.075 kg, CoM 0.267 m from the ring end
(`simulation_assets/four_rigid_ground_rig_L100.sdf:223-224`). The vertical parts agree: the planner has the rod's
whole weight on the drone (`drone_mass` 0.55), and the drone does end up carrying it. **The horizontal parts differ by
w cot e on every plate, outward:**

| Elevation (deg) | 50 | 57 | 60 | 62 | 63 | 70 |
|---|---|---|---|---|---|---|
| w cot e (N) | 0.453 | 0.350 | 0.311 | 0.287 | 0.275 | 0.196 |

These cancel on an even formation and not on an uneven one. The T keeps one of them, towards plate 3. That is the
sideways force. The integral is not chasing a fault in a tracker. It has found a real force in the simulation that the
planner's model does not contain.

Two more closed forms, both checked against the logs. Roll about the pair's line, and what the integral learns:

    tan(roll) = r F_z3 / (h m g + r (F_z3 + w) cot e3)
    d_y = -[ (F_z3 + w) cot e3 * t_p / (T_R,p + w sin e_p) - t3 cos e3 ]      (T, massless plan)

| Check | Runs | Miss |
|---|---|---|
| The roll form, F_z3 and e3 from the log | 10 T and no-resize runs | 0.15-0.76 deg |
| The d_y form | 7 T runs with the integral on | 0.001-0.018 N |

So the plant's roll stiffness about the pair's line is m g h plus r (F_z3 + w) cot e3, about 40 % more than the m g h
that earlier analyses used.

## 2. The audit, stage by stage

| Item | Planner or tracker | Plant | Agree? |
|---|---|---|---|
| Planned drone | rod pivot | joint 4 cm below `base_link` | yes |
| Rod length (m) | 1.0000, measured | 1.0000 | yes |
| Plate radius, height over the CoG (m) | 0.225, 0.02 | 0.225, 0.02 | yes |
| Ring CoG | link origin | link origin | yes |
| Ring Ixx, Izz | 0.02733, 0.05452 | 0.02227, 0.04431 | **no**, up to 1 deg |
| Rod weight | all on the drone | shared, section 1 | **no**, the root |
| Integral's force acts at | the CoG | the plate | **no** |
| Pull on a zero-share slot (N) | at least 0.1 | set by the rod | **no** |
| Middle carrier's pull on the T (N) | t 0.13, never compared | T_D 0.93 | **no** |
| Measured z pull | clamped to at most 0 | can be positive | **no**, near zero pull |
| Cable cap (m/s^2) | 14.5 | largest 9.7 | not active |
| Throttle cap | 0.8 | largest 0.735 | not active |
| `ff_cap_force` | 0.75, released | `ff` logged 1.000 | not active |
| Tautness gate | | 1.000 | not active |
| Thrust map | kT = K / 0.55 | airframe 0.475 kg | loop gain only |
| Joint damping | | 0.1 | no steady effect |

Where each one is:
- Planned drone: `load_cable_dynamics.py:231-232`, `planner_node.py:1243-1255`, world
  `four_rigid_ground_rig_L100.sdf:251`.
- Rod length: `planner_node.py:1261-1265, 1948-1965`. Logged `len` is 1.0000 on every real rod (the phantom reads
  1.0086).
- Plate and CoG: `geometry.py:160-172`, world `.sdf:66, 69-70, 206`. The inertia: `params.py:31-32`, world `.sdf:70`.
- Rod weight: `load_cable_dynamics.py:172-175, 266`, world `.sdf:223-224`,
  `simulation_assets/generate_rigid_world.py:65-95`.
- Integral: `lumped_integral.py:47-49`, `planner_node.py:822-831`, `load_cable_dynamics.py:172-175`.
- Floor: `planner_ocp.py:63, 167-170`, `geometry.py:240`.
- Node 0: `planner_ocp.py:148-153`, `planner_solver.py:210-252`. Ring pose and rod directions pinned, tension free.
- Reference to the tracker: `geometry.py:88-94`, `planner_solver.py:434, 448`.
- Tracker model: `tracker_node.py:1646-1651, 1874-1878`, `dynamics.py:118`. Clamp: `tracker_node.py:1688`.
- Caps: `tracker_node.py:1704-1712`, `planner_node.py:1586-1600`. Thrust:
  `simulation_communication/rig_thrust.py:30-32`.

**Is the planned drone position right against the rod geometry?** Yes. Node 0's pivot is the measured pivot by
construction, and the rod closes at 1.0000 m.

**Is the tracker's steady position error zero, and why not?** At node 0 the reference differs from the drone only by
item 5. The tracker is not lagging. The measured pull makes it offset-free: a force error turns into a throttle or
attitude change, not a position error. The exception is the clamp (item 3). With the true thrust as its feedforward,
the roll moves 0.01 deg (section 5). **The trackers are not the cause.**

## 3. Numbers from flown runs

Ring level, settled. `plan5 y` is where the plan puts the ring 0.5 s ahead, relative to where it is.

| Run | Setup | Tilt | y | z | int_fy | int_fz | plan5 y |
|---|---|---|---|---|---|---|---|
| R2349 | Square, b4 hover | 0.06 | +0.1 | -1.0 | +0.001 | -0.076 | 0.0 |
| R2965 before the release | Square | 0.05 | +0.1 | -0.9 | 0.000 | -0.021 | 0.0 |
| R2918 after the release | Even triangle | 0.09 | +0.3 | -0.8 | +0.005 | -0.108 | -0.1 |
| R2934 after the release | Even triangle | 0.10 | +0.2 | -0.6 | +0.006 | -0.123 | -0.1 |
| R2918 before the release | M1 four, uneven | 1.63 | -1.3 | -0.8 | +0.269 | -0.040 | +0.6 |
| R2934 before the release | M1 four, uneven | 1.55 | -2.0 | -1.1 | +0.262 | -0.050 | +0.5 |
| R2969 | T, 5.26 | 6.77 | +0.4 | +0.7 | -0.258 | -0.265 | +1.4 |
| R2965 | T, 5.26 | 8.30 | +1.3 | +1.3 | -0.281 | -0.256 | +0.4 |
| R2972 | T, 5.26 | 10.52 | -1.0 | +0.3 | -0.307 | -0.271 | +0.6 |
| R3202 | T, 0 | 0.61 | -63.9 | -1.1 | 0 | -0.351 | +22.4 |
| R3206 | T, 0, middle floor 0 | 3.33 | -73.6 | +1.8 | 0 | -0.297 | +23.4 |
| R3201 | T, 5.26, middle floor 0 | 1.06 | +0.4 | 0.0 | -0.265 | -0.310 | +0.5 |
| R3000 | T, 5.26, rod in the plan | 5.19 | -1.5 | -13.3 | -0.122 | -0.891 | +1.5 |
| R2966 | No resize, 5.26 | 2.19 | +0.1 | -3.6 | -0.299 | -0.566 | -0.5 |
| R3203 | No resize, 0 | 2.17 | -70.1 | -2.7 | 0 | -0.568 | +25.8 |

**On even formations the xy integral learns about zero in the simulation.** On the uneven M1 four it learns about one
rod's force, and on the T the same. The height part only drifts slowly with a few mm of error (item 7).

Per carrier, settled. T_D is the rod's axial pull at the drone. F_z and F_out are the plant's force on the plate, up
and outward. All are inferred from the tracker log: logged throttle and attitude, the thrust law, the rods' statics.
`ref-c` is the radial part of the tracker's reference minus its position, in mm.

| Run | Drone (plate) | Plan t | T_D | F_z | t sin e | F_out | (F_z+w) cot e | t cos e | Elev | ref-c |
|---|---|---|---|---|---|---|---|---|---|---|
| R2349 square | 0 (0) | 2.457 | 3.236 | 2.118 | 2.130 | 1.529 | 1.529 | 1.226 | 60.08 | -1.76 |
| | 1 (9) | 2.457 | 3.206 | 2.092 | 2.130 | 1.514 | 1.514 | 1.225 | 60.08 | -1.56 |
| | 2 (6) | 2.457 | 3.235 | 2.117 | 2.130 | 1.528 | 1.528 | 1.225 | 60.09 | -1.59 |
| | 3 (3) | 2.457 | 3.205 | 2.091 | 2.130 | 1.514 | 1.514 | 1.226 | 60.08 | -1.69 |
| R2918 triangle | 0 (11) | 3.289 | 4.037 | 2.811 | 2.850 | 1.930 | 1.930 | 1.642 | 60.06 | -1.51 |
| | 2 (7) | 3.289 | 4.036 | 2.811 | 2.850 | 1.930 | 1.930 | 1.642 | 60.06 | -1.48 |
| | 3 (3) | 3.289 | 4.035 | 2.808 | 2.849 | 1.932 | 1.932 | 1.644 | 60.00 | -1.43 |
| R2918 M1 four | 0 (11) | 2.175 | 3.112 | 2.030 | 1.900 | 1.428 | 1.430 | 1.057 | 60.91 | -1.80 |
| | 1 (9) | 1.629 | 2.189 | 1.237 | 1.436 | 0.953 | 0.953 | 0.770 | 61.80 | -1.40 |
| | 2 (7) | 2.175 | 3.109 | 2.028 | 1.900 | 1.426 | 1.428 | 1.057 | 60.91 | -1.85 |
| | 3 (3) | 3.812 | 4.478 | 3.132 | 3.246 | 2.259 | 2.259 | 1.998 | 58.39 | -1.15 |
| R2969 T 5.26 | pair (0, 6) | 5.005 | 5.623 | 4.151 | 4.304 | 2.778 | 2.783 | 2.554 | 59.32 | -1.17 |
| | 3 (3) | 0.130 | 0.925 | 0.125 | 0.115 | 0.351 | 0.351 | 0.061 | 62.18 | -2.23 |
| R2965 T 5.26 | pair (0, 6) | 4.997 | 5.607 | 4.135 | 4.296 | 2.772 | 2.778 | 2.553 | 59.28 | -1.19 |
| | 3 (3) | 0.134 | 0.963 | 0.159 | 0.119 | 0.370 | 0.370 | 0.063 | 62.10 | -2.19 |
| R2972 T 5.26 | pair (0, 6) | 5.008 | 5.580 | 4.110 | 4.303 | 2.762 | 2.769 | 2.562 | 59.23 | -1.03 |
| | 3 (3) | 0.116 | 1.022 | 0.209 | 0.103 | 0.399 | 0.399 | 0.055 | 61.92 | -2.35 |
| R3202 T 0 | pair (0, 6) | 5.059 | 5.691 | 4.213 | 4.355 | 2.807 | 2.811 | 2.575 | 59.40 | -1.08 |
| | 3 (3) | 0.099 | 0.784 | 0.004 | 0.088 | 0.276 | 0.276 | 0.045 | 63.05 | -3.25 |
| R3206 T 0 floor 0 | pair (0, 6) | 5.042 | 5.698 | 4.247 | 4.365 | 2.763 | 2.766 | 2.522 | 59.98 | -0.95 |
| | 3 (3) | -0.002 | 0.703 | -0.068 | -0.001 | 0.236 | 0.236 | -0.001 | 63.38 | -3.48 |
| R3201 T 5.26 floor 0 | pair (0, 6) | 5.051 | 5.655 | 4.209 | 4.373 | 2.742 | 2.746 | 2.529 | 59.96 | -1.07 |
| | 3 (3) | 0.005 | 0.796 | 0.011 | 0.004 | 0.292 | 0.292 | 0.002 | 62.08 | -2.31 |
| R3000 T rod in plan | pair (0, 6) | 6.537 | 5.880 | 4.263 | 5.494 | 3.095 | 3.098 | 3.543 | 57.18 | +0.57 |
| | 3 (3) | 0.434 | 0.672 | -0.104 | 0.376 | 0.251 | 0.251 | 0.217 | 59.99 | -0.85 |
| R2966 no resize | pair (0, 6) | 2.604 | 5.635 | 4.200 | 2.259 | 2.717 | 2.721 | 1.297 | 60.14 | -4.30 |
| | 3 (3) | 2.604 | 0.824 | 0.034 | 2.292 | 0.309 | 0.309 | 1.237 | 61.64 | +4.22 |

Reading it:
- **F_out = (F_z + w) cot e holds on every carrier, in every formation, within 0.007 N.** The planner's t cos e misses
  it.
- **The planned t is not the rod's pull.** It is a model variable that nothing compares with the plant.
- **The tracker's `ref-c` is the attitude term.** It flips sign on the no-resize middle carrier, whose planned thrust
  leans further than the real one, because it plans 2.6 N against a real 0.8 N.

The plant's balance from these numbers, using the ring's own mass and the rods' statics:

| Run | Net force on the ring, y / z (N) | Net moment, x (N m) |
|---|---|---|
| R2349 | 0.000 / -0.019 | +0.0001 |
| R2918 M1 four | -0.001 / -0.009 | +0.0008 |
| R2969 | +0.002 / -0.009 | +0.0012 |
| R3202 | +0.002 / -0.007 | +0.0015 |
| R3206 | +0.004 / -0.011 | +0.0023 |
| R3000 | +0.008 / -0.015 | +0.0029 |

## 4. The steady balance: one calculation for all the T results

**Method.** The closed loop at rest is a fixed point, and every part of it can be computed offline:
1. The drones' pivots are the unknowns.
2. The rods' statics give the ring's pose and every rod force from the pivots. From logged pivots this returns the
   logged ring to 0.06 deg and 0.2 mm in all eight runs tried.
3. The real `PlannerSolver` and `ReferenceBuilder` plan from that state.
4. The real tracker OCP, with the measured pull and the clamp, gives each drone's first control. At rest it must be
   zero.
5. With the integral on, the ring's y must also be zero, and d_y is one more unknown.

Nothing is fitted. Inputs: the SDF's rod, the typed thrust offsets, the planner's typed inertia. `int_fz` is taken
from the run's window, and the result hardly depends on it (section 5).

**Replay check, flown state into the real solvers:**

| Quantity | Replay against the log |
|---|---|
| Planned tensions at node 0 | within 0.001 N |
| `plan5` | within 0.1 mm |
| `ref - pose` | within about 0.1 mm |
| Tracker's first control | under 0.016 /s, against a limit of 1 |

**The five results of the brief, and neighbours.** Solved against flown:

| Run | Setup | Roll flown | Solved | y flown | Solved | d_y flown | Solved | t3 flown | Solved |
|---|---|---|---|---|---|---|---|---|---|
| R2969, R2976 | T, 5.26, floor 0.1 | +6.77, +6.78 | **+6.74** | +0.4 | 0.0 | -0.258 | -0.258 | 0.130 | 0.129 |
| R2965, R2972 | T, 5.26, floor 0.1 | +8.30, +10.52 | see roots | +1.3, -1.0 | 0.0 | -0.281, -0.307 | | 0.134, 0.116 | |
| R3202 | T, 0, floor 0.1 | +0.60 | **+0.57** | -63.9 | **-64.1** | 0 | 0 | 0.099 | 0.099 |
| R3206 | T, 0, middle floor 0 | -3.33 | **-3.75** | -73.6 | **-72.7** | 0 | 0 | -0.002 | -0.002 |
| R3000 | T, 5.26, rod in the plan | -5.19 | **-5.95** | -1.5 | 0.0 | -0.122 | **-0.119** | 0.434 | 0.434 |
| R2966 | No resize, 5.26 | +2.19 | **+2.30** | +0.1 | 0.0 | -0.299 | **-0.302** | 2.604 | 2.605 |
| R3201 | T, 5.26, middle floor 0 | +1.06 | +1.20 | +0.4 | 0.0 | -0.265 | -0.268 | 0.005 | 0.005 |
| R3203 | No resize, 0 | -2.17 | -2.56 | -70.1 | -70.1 | 0 | 0 | 2.547 | 2.543 |

**Out of sample.** Flown on other trees and days, and not used to build anything. The solver takes each flight's typed
values.

| Run | Setup | Roll flown | Solved | y flown | Solved | d_y flown | Solved | t3 flown | Solved |
|---|---|---|---|---|---|---|---|---|---|
| R2249 | T, 0.55 m rods, 5.26 | +6.61 | +6.60 | +0.5 | 0.0 | -0.239 | -0.243 | 0.128 | 0.128 |
| R2840 | T, 1.15 kg, 0 | +0.68 | +0.69 | -64.6 | -65.1 | 0 | 0 | 0.097 | 0.097 |
| R2843 | T, 1.15 kg, 5.26 | +7.81 | +7.93 | -0.2 | 0.0 | -0.352 | -0.350 | 0.091 | 0.091 |
| R2862 | T, 1.15 kg, 5.26 | +8.27 | +8.05 | -0.1 | 0.0 | -0.355 | -0.349 | 0.092 | 0.092 |
| R2918 after the release | Even triangle, 5.26 | 0.09 | 0.00 | +0.3 | 0.0 | +0.005 | 0.000 | 3.289 | 3.290 |

The worst misses are the three plate-3-low cases (R3206, R3000, R3203), by 0.4-0.8 deg. In those the clamp of item 3
binds, and the solved roll moves 1.2 deg for a 0.002 throttle change, so that is where the calculation is least well
set. **No result of the five breaks the explanation.** The plate-3-low ones are right in sign and within 0.8 deg, and
the offsets within 1 mm.

**The T at 5.26 has several roots, which is why its repeats scatter.** Exact fixed points of the solver:

| Root | Roll | d_y | t3 | Reached from |
|---|---|---|---|---|
| Lower | +6.74 | -0.258 | 0.129 | starts at -5, 0, 6, 9.5 and 14 deg |
| Middle | +9.37 | -0.294 | 0.126 | a start at 19 deg, and R2965's and R2972's states |
| Upper | +18.76 | -0.427 | 0.100 | a start at 25 deg, and R2975's state |

Between the lower and middle roots the tracker's first control never exceeds about 0.012 /s, so the roll is close to
neutral there. The flown runs drift just as two stable roots with an unstable one between them would (first and last 1
s of the settled window):

| Run | Roll, window start | Window end | Drift | d_y |
|---|---|---|---|---|
| R2969 | 6.84 | 6.79 | flat, on the lower root | -0.258 |
| R2976 | 6.83 | 6.80 | flat | -0.258 |
| R2997 | 7.32 | 6.77 | down, to the lower root | -0.260 |
| R2967 | 8.54 | 7.27 | down | -0.269 |
| R2974 | 8.63 | 7.22 | down | -0.270 |
| R2965 | 8.71 | 7.89 | down | -0.281 |
| R2968 | 9.79 | 9.90 | up, above the middle root | -0.295 |
| R2972 | 10.27 | 10.97 | up | -0.307 |
| R2975 | 12.46 | 13.80 | up | -0.340 |

So 8.30 and 10.52 are not two samples of one number. They are two runs on opposite sides of the middle root, still
moving at LAND, and the release transient decides the side. This also explains the "r = 0.997" of the forensics: tilt
and learned force move together because they are two coordinates of one near-neutral line, not because one causes the
other.

### The explanation, one row per result

The roll about the pair's line has little stiffness, and the drones are position servos on rigid rods, so the plan's
opinion of the roll is forced on the ring. Three errors set that opinion, and the rod weight is behind two of them.

| Run | The rod's sideways force is handled by | That part | Floor part | Solved | Flown |
|---|---|---|---|---|---|
| R2969 | The integral, at the CoG | +1.2 | +5.5 | +6.74 | +6.77 |
| R3202 | Nothing, the ring gives way 64 mm | -5.3 | +5.9 | +0.57 | +0.60 |
| R3206 | Nothing, and the clamp limits the roll | -3.75 | none | -3.75 | -3.33 |
| R3000 | The model, with a wrong tension reference | -5.95 | not active | -5.95 | -5.19 |
| R2966 | The integral, in a model with a phantom cable | +2.30 | not active | +2.30 | +2.19 |

- **R3202 is level by cancellation.** The ring gives way, rod 3 then sits at 63 deg against a 60 deg reference, and
  the plan holds drone 3 low and out. In the replay it drops drone 3 by 6-9 mm and moves it out 16 mm within 0.3 s,
  then brings it back as its ring rolls up to a roll that never happens. The tracker's first control balances on that.
  Offline, taking the weight off the middle cable's direction (x 0.01) moves the roll from -2.56 to -0.63 at floor 0,
  and from +0.59 to +3.71 with the floor.
- **Why modelling the rod did not fix R3000.** The reference asks plate 3 for 0.25 N where a level ring needs 0.83 N
  (item 2), so the plan pulls plate 3 down.
- **What the integral learns in R3000.** The plan has 0.434 N at plate 3 and the plant carries 0.672 N. The integral
  learns that gap's sideways part, (0.672 - 0.434) x cos 60 = 0.119 N. Flown 0.122 N, solved 0.119 N. R3000 does not
  show the rod weight is unimportant. It shows the rod path of `balanced_tensions` is wrong.

## 5. One disagreement at a time, offline

All on the 0.86 kg T, with offsets 0.185 for all drones so the T stays symmetric, from a level start. With the
integral on these are the lower root. "Planner rod" is `rod_mass` 0.075, `rod_com` 0.27, and the plant at 0.27 too.

| Integral | Change | Roll | y | d_y | t3 planned | T_D plant |
|---|---|---|---|---|---|---|
| 5.26 | None, as flown | +6.72 | 0.0 | -0.259 | 0.129 | 0.93 |
| 5.26 | Middle floor 0 | +1.19 | 0.0 | -0.268 | 0.005 | 0.81 |
| 5.26 | Force in the attach plane | +5.43 | 0.0 | -0.250 | 0.122 | 0.90 |
| 5.26 | Middle floor 0, force in the attach plane | **-0.20** | 0.0 | -0.262 | 0.000 | 0.78 |
| 5.26 | No reference rotation (the OCP keeps d_L) | +5.99 | 0.0 | -0.282 | 0.138 | 0.93 |
| 5.26 | Middle slot not rotated | +6.50 | 0.0 | -0.278 | 0.128 | 0.94 |
| 5.26 | Middle floor 0, no reference rotation | +0.86 | 0.0 | -0.297 | 0.022 | 0.81 |
| 5.26 | Node 0 centre from the true attitude | +7.15 | 0.0 | -0.266 | 0.131 | 0.94 |
| 5.26 | Tracker feedforward from the true thrust | +6.71 | 0.0 | -0.259 | 0.129 | 0.93 |
| 5.26 | No pull clamp | +6.72 | 0.0 | -0.259 | 0.129 | 0.93 |
| 5.26 | **Plant rods 1 g** | +5.24 | 0.0 | **+0.017** | 0.122 | 0.10 |
| 5.26 | **Plant rods 1 g, middle floor 0** | **-0.01** | 0.0 | **-0.004** | 0.000 | 0.01 |
| 5.26 | Planner rod, code's references (R3000) | -4.11 | 0.0 | -0.112 | 0.437 | 0.71 |
| 5.26 | Planner rod, level tension reference | +0.76 | 0.0 | -0.033 | 0.800 | 0.81 |
| 5.26 | **Planner rod, level tensions, pair's reference leaning** | **+0.34** | 0.0 | **-0.008** | 0.805 | 0.80 |
| 0 | None (R3202) | +0.59 | -63.8 | 0 | 0.099 | 0.79 |
| 0 | Middle floor 0 (R3206) | -2.56 | -76.7 | 0 | -0.002 | 0.73 |
| 0 | Middle floor 0, no pull clamp | -5.34 | -67.1 | 0 | -0.001 | 0.68 |
| 0 | Node 0 centre from the true attitude | +0.70 | -63.9 | 0 | 0.099 | 0.80 |
| 0 | Tracker feedforward from the true thrust | +0.59 | -63.8 | 0 | 0.099 | 0.79 |
| 0 | **Plant rods 1 g** | +6.05 | **+2.7** | 0 | 0.126 | 0.11 |
| 0 | **Plant rods 1 g, middle floor 0** | **-0.13** | **-1.0** | 0 | 0.000 | 0.01 |
| 0 | Planner rod, code's references | -5.73 | -27.6 | 0 | 0.429 | 0.68 |
| 0 | **Planner rod, level tensions, pair's reference leaning** | **+0.01** | **-1.3** | 0 | 0.804 | 0.79 |

Reading it:
- **With no rod weight, or with the rod weight correctly in the plan, the integral has nothing to do.** The ring sits
  on its point within 3 mm with it off, and it learns under 0.02 N with it on. This is the direct answer to the
  question.
- **With the rod correctly in the plan, the planned t equals the plant's pull.** Nowhere else in this file do they
  agree.
- **The floor is a separate error.** It stays with rods of 1 g.
- **The tracker terms move the roll by at most 0.4 deg.** The reference rotation is not the lever either. The point of
  action of the force is.

`balanced_tensions` with the rod in the plan, item 2:

| Share on the T, rod 0.075 kg, CoM 0.27 m | Pair | Middle | Left over |
|---|---|---|---|
| Code, `geometry.py:239` | 5.934 | 0.251 | Roll moment -0.106 N m, y force 0.039 N |
| Level, vertical and moment rows only | 5.646 | 0.826 | y force 0.327 N, for the pair's lean to take |
| Level and balanced, the pair leaning 1.6 deg | 5.665 | 0.792 | None |

Rod mass and elevation, both following w cot e. First table with the middle floor at 0, typed offsets:

| Rod mass (kg) | d_y, 5.26 | Roll, 5.26 | y, int off | Roll, int off |
|---|---|---|---|---|
| 0.020 | -0.082 | +0.31 | -21.1 | -1.84 |
| 0.040 | -0.155 | +0.62 | -40.6 | -3.03 |
| 0.075 | -0.268 | +1.19 | -72.7 | -3.75 |

| Elevation (deg) | d_y, 5.26 | Roll, 5.26 | y, int off | Roll, int off |
|---|---|---|---|---|
| 50 | -0.362 | +5.16 | -71.1 | **-3.16** |
| 60 | -0.258 | +6.74 | -64.1 | **+0.57** |
| 70 | -0.166 | +8.25 (not fully converged) | -49.1 | **+4.64** |

The last column is why R3202's level ring should not be trusted: it holds only near 60 deg. Height integral and
inertia:

| Check | Value A | Value B |
|---|---|---|
| T, floor 0, 5.26: `int_fz` | -0.1 N: z -0.5, roll +1.20 | -0.6 N: z -0.2, roll +1.19 |
| T, 0, floor 0.1: planner inertia | Typed 0.02733: +0.57, y -64.1 | World 0.02227: +1.59, y -70.9 |
| T, 0, floor 0: planner inertia | Typed: -3.75, y -72.7 | World: -3.38, y -74.9 |
| No resize, 5.26 (as flown +2.30) | Rotation off: +1.64 | Force in the attach plane: +1.95 |

The first row is 0.6 mm per N, a loop time constant of about 580 s at `z_ki` 0.4 and `int_k_z` 7.13. The second and
third say the inertia mismatch is not the cause.

## 6. The earlier analyses, checked

- **"The T's tilt is the planner's own optimum, 7.35 and 9.38 with the force" (`t_offset_model`).** Right for the plan
  alone, and 9.38 is the middle root. But the plant's stable root is 6.74, and with the integral off the same plan
  flies at 0.6.
- **"The floor alone does not tilt the T" (`t_offset_forensics`, `t_offset_attach`).** Half right. The floor gives
  +5.5 to +5.9 in both arms. With the integral off it is hidden by an equal and opposite offset effect. R3206 shows
  the other half.
- **"The sideways force comes from the rod's weight" (`t_offset_forensics`).** Right, and now exact: w cot e.
- **"The plate-3 rod carries 0.83 N axial" (`t_offset_forensics`).** I read 0.96 N at the drone end for R2965, with
  net up at the plate about the same. It does not change that reading.
- **"Tilt follows the force, r = 0.997" (`t_offset_forensics`).** True, and misleading (section 4). The fixed part of
  the force is w cot e. The line through the roots adds the rest.
- **"R3000 falsifies the rod weight" (card line 118, critic).** No. R3000 flew a wrong reference. With a right one the
  same model gives +0.34 and -0.008 N offline. So I would not drop `rod_mass`, as the critic's section D says.
- **"The roll is held only by the 2 cm pendulum" (`t_offset_model`).** True of the model. The plant has the extra
  term.
- **"The residual 1 deg with the zero floor is the rod pull through the lean" (card line 117).** Confirmed: -0.20 with
  the force in the attach plane.
- **"`int_k_xy` 0 is the no-code fix" (critic D, and the preferred automatic rule in `SESSION_STATE`).** This is the
  one place I push back. It gives a level ring by cancellation, 64 mm off, and only near 60 deg. It removes the
  integral's symptom and keeps the model error. If it is chosen, it should be chosen knowing that.

Trap found on the way: the node logs (`logs/*/log.csv`) run on the world clock and `events.csv` on the runner's. The
offset is 3-21 s and changes per run. A window taken from `events.csv` on the node logs reads the wrong seconds. Align
on the planner's first LAND tick or on the lift. `tools/x1_metrics.py` is safe, as it reads `run.csv`.

## 7. Proposed runs (one run each, exploratory, nothing flown)

Each changes one thing against the named baseline. Expected values are the solver's, except run 5. All on the 0.86 kg
`x1_sqT` cell unless stated.

| # | Change | Baseline | Expected | Falsifier | Needs |
|---|---|---|---|---|---|
| 1 | `int_k_xy` 0, `cable_elev_deg` 70 | R3202 | +4.6, y -49 | Roll under +2.5 | Config |
| 2 | `int_k_xy` 0, `cable_elev_deg` 50 | R3202 | -3.2, y -71 | Roll over -1 | Config |
| 3 | `cable_elev_deg` 70 | R2969 | d_y -0.17, roll about +8 | d_y outside -0.14..-0.20 | Config |
| 4 | LAND 60 s after the release, not 20 | R2965 | +6.7, or towards +18.8 | Flat for 20 s, 8-15 deg | Config |
| 5 | `x1_h4` M1 four, `int_k_xy` 0 | R2918 | y +60 to +70 towards plate 9 | Under 30 mm off | Config |
| 6 | Rods 1 g, `int_k_xy` 0, zero floor | R3206 | -0.1, y -1 | Over 1 deg or 10 mm | **World SDF** |
| 7 | Rods 1 g, 5.26, floor as flown | R2969 | d_y +0.02, roll +5.2 | \|d_y\| over 0.05 | **World SDF** |
| 8 | Rod in the plan, level references, `int_k_xy` 0 | R3000 | 0.0, y -1 | Over 1 deg or 10 mm | Code, critic |
| 9 | The same at 5.26 | R3000 | d_y -0.01, roll +0.3 | \|d_y\| over 0.05 | Code, critic |
| 10 | Force in the attach plane, zero floor, 5.26 | R3201 | -0.2, d_y -0.26, on point | Roll over 0.7 | Code, critic |

Notes:
- **Runs 1-4 confirm the explanation with no code and no world change. They should go first.** Runs 1 and 2 test the
  cancellation directly.
- **Runs 6 and 8 are the check of whether the integral is still needed once the disagreement is gone.** The expected
  answer is no.
- **Runs 6 and 7 are a world SDF, so they need Wesley's word.** Rods of 1 g with no magnet, and the airframe up by
  0.074 kg so each drone's total stays 0.55 kg. The zero floor is `plan_zero_share_floor`, still unmerged in the
  worktree.
- **Run 5 has no solved number.** The solver does not converge on four real rods. The number is from the no-resize
  analogue.
- **Runs 8-10 are new code in the flight loop, behind a switch, default off, with a critic.** Run 8 needs the level
  tension reference and the pair's leaning direction reference. Run 10 changes the OCP source, so it needs a prebuild.
- Run 10 is the only one of these that could also carry over to real flights unchanged.
- In runs 1-3 the hand-over elevation stays at 60 deg, and the plan moves the rods to the typed elevation after it.

Decisions for Wesley:
- Queue runs 1-5 (config only, 5 runs on the lab PC)? Recommended: they confirm or break this explanation cheaply.
- A rods-of-1 g world for runs 6-7? It is the cleanest test of "is the integral needed", and it is a world SDF.
- Which way to go after that: put the rod correctly in the plan (run 8, simulation only), move the force's point of
  action and zero the floor (run 10, also real flights), or keep the automatic `int_k_xy` 0 rule knowing it levels by
  cancellation.

## 8. What carries over to the real world

The real logs were not read here. The planner and trackers are the same code, so the bookkeeping acts there too.

| Quantity | Simulation | Real, 33 g per cable, split unknown |
|---|---|---|
| w (N) | 0.54 | 0.16 to 0.32 |
| w cot 60 deg (N), per taut cable | 0.31 | 0.09 to 0.19 |
| m g h (N m/rad) | 0.169 | 0.316 at 1.15 kg, typed attach_z 0.028 |
| Floor's static roll (deg) | 6.2 | 3.1 |
| Roll from that sideways force at the CoG (deg) | 1.9 | 0.5 to 1.0 |

Inferred from the code and the statics, not from real logs:
- **The sideways force is real but smaller, and only while the cable is taut.** A cable or a magnet has the same
  statics as the rod, with w = (m_magnet + m_cable / 2) g. A slack cable has no sideways force at all.
- **A slack middle cable leaves the magnet's weight on plate 3.** That is a roll moment with no term in the model.
- **A cable cannot push.** So the plate-low half of the simulation's cancellation is capped. The cancellation that
  levels the simulated T at `int_k_xy` 0 should not be expected on real cables. The rig's chain files fly `int_k_xy` 0
  (`configs/rig/chain_pair*.yaml`).
- **The floor acts.** A cable can pull a plate up, so that sign survives.
- **The integral's point of action acts** whenever `int_k_xy` is not 0, whatever the source of d_y.
- **The clamp (item 3) acts** on a near-slack carrier, and there it depends on the thrust map's error
  (`docs/analysis/thrust_margin_2026-10-31.md`).
- **Item 5 acts:** 1-4 mm, largest where the planned pull is furthest from the real one.
- **The planned t is never compared with the real pull.** The measured pull reaches the tracker and not the planner.
- **`balanced_tensions` (item 2) does not act** while `rod_mass` is 0. Fix it first if a light-cable model is tried.
- **`attach_z` is a real-only worry.** The stiffness m g h scales with the true height of the attach plane above the
  true CoG. real.yaml types 0.028, with plate faces noted near the pivot. A wrong h changes every roll number on a T.

## 9. Caveats and reproduction

- The solver is quasi-static. It finds rest states, not how a flight reaches one. It has no swing, noise or timing.
  The stability of the roots rests on the flown drift of nine runs, not on a computed eigenvalue.
- It does not converge on four real rods (square, M1 four): the split of tension between opposite pairs is too
  sensitive. Those formations rest on the logs only. The even triangle and every T do converge.
- It uses a mean state. A mean of a swinging run (R3206: std 0.4 deg) is not a rest state, which may be part of the
  0.4-0.8 deg.
- "Force in the attach plane", "level references" and the `no_rot` switches exist only in scratch code. They are
  offline numbers until they fly.
- Section 5 uses offsets 0.185 for all drones, section 4 the typed ones. They differ only where the clamp binds.

Scripts, in `results/analysis/why_integral_2026-10-31/`:
- `plan.py` and `tracker_off.py` build the real OCPs in `work/`. `state.py` reads a run and closes the plant statics.
- `fp.py` is the fixed point. `replay.py` is the replay.
- `t8b.py`, `t10.py`, `t19.py`, `t25.py` and `t26.py` make the tables.

## Addendum: flown after this was written (31 Oct evening)

| Proposed run | Run | Expected | Flown | Reading |
|---|---|---|---|---|
| 1 | R3235 | +4.6, y -49 | +4.58, y -49 | Held |
| 2 | R3236 | -3.2, y -71 | -2.86, y -73 | Held |
| 3 | R3237 | d_y -0.17 | d_y -0.181, roll +10.40 | Force held, roll about 2 deg over |
| 4 | R3238 | +6.7, or towards +18.8 | +16.90, d_y -0.428 | The high root |
| 5 | R3239 | y +60 to +70 | y +46, 2.12 deg | Under the analogue |
| 6 | R3400 | -0.1, y -1 | -0.19, 0.5 mm off | Held: no rod weight, no floor, no integral needed |
| 7 | R3401 | +5.2, d_y +0.02 | +19.00, d_y -0.13 | **Falsified** |

Run 7's falsifier tripped. The rest states of section 4 and section 5 do not say which one a flight lands on, and with
the floor binding a flight can creep where the solver has no rest state at all. So read the lower root's numbers with
the integral on, in section 4 and section 5, as rest states and not as predictions. With the integral off every
prediction held. The fix chosen is carded in `docs/experiments/2026-10-31_rod_in_plan.md`.

## Addendum: flown after this was written (31 Oct evening)

| Proposed run | Run | Expected | Flown | Reading |
|---|---|---|---|---|
| 1 | R3235 | +4.6, y -49 | +4.58, y -49 | Held |
| 2 | R3236 | -3.2, y -71 | -2.86, y -73 | Held |
| 3 | R3237 | d_y -0.17 | d_y -0.181, roll +10.40 | Force held, roll about 2 deg over |
| 4 | R3238 | +6.7, or towards +18.8 | +16.90, d_y -0.428 | The high root |
| 5 | R3239 | y +60 to +70 | y +46, 2.12 deg | Under the analogue |
| 6 | R3400 | -0.1, y -1 | -0.19, 0.5 mm off | Held: no rod weight, no floor, no integral needed |
| 7 | R3401 | +5.2, d_y +0.02 | +19.00, d_y -0.13 | **Falsified** |

Run 7's falsifier tripped. The rest states of section 4 and section 5 do not say which one a flight lands on, and with the
floor binding a flight can creep where the solver has no rest state at all. So read the lower root's numbers with the integral
on, in section 4 and section 5, as rest states and not as predictions. With the integral off every prediction held. The fix
chosen is carded in `docs/experiments/2026-10-31_rod_in_plan.md`.

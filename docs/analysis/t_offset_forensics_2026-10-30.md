# Why the T hangs tilted after the unannounced square-to-T release: a read of the flown data (30 Oct)

Read-only forensics of X1's sqT cell (R2965-R2978, 30 Oct, lab PC) and every earlier T flight in the twin and on the
rig. No code, config or run was changed. Plots: `results/analysis/t_offset_2026-10-30/` (scripts in `scripts/`).
Angles: "tilt" is the ring normal's lean; + = the plate-9 (released) side low, the plate-3 side high, i.e. a roll about
the 0-6 axis. Windows: settled = t_e+12..18 s for X1, t_e+15..20 s for older runs.

## Short answer

1. **The offset starts at the resize.** Every late arm follows the no-resize trajectory within 1.9 deg until its own
   resize (the no-resize repeats differ by up to 2.5 deg during the swing), then leaves it by more than 2 deg 0.4-1.0 s
   after the resize. The full offset (+5 to +8 deg on top of no resize) builds over the next ~4-5 s.
2. **The 0.1 N tension floor is on every T, tilted or level.** After the resize, our planner plans the middle carrier
   (plate 3) at 0.10-0.13 N, the 0.1 N floor or just above it. The same planned 0.097-0.099 N T **hung level (0.6-0.7 deg)
   in 7 earlier twin flights**: same world, ring and 1 m rods, measured pull. One of them, R2823, is the same unannounced
   plate-9 release. So the floor alone does not tilt the twin ring.
3. **What separates the tilted T's from the level ones is the planner's sideways (xy) model integral.** All 9 T's with
   `int_k_xy` 5.26 and the measured pull tilted 6.6-12.2 deg: X1's 8 resized runs, and R2249 on 21 Oct (no fast refs,
   no SQP). All 7 with `int_k_xy` 0 and throttle headroom hung at 0.6-0.7 deg. The one integral-off T that tilted
   (R2834, 8.3 deg) had its throttle at the 0.8 cap on every tick. Across the 8 X1 resized runs, tilt and the integral's
   force track each other with r = 0.997.
4. **Mechanism, as far as the logs show it.** The twin's plate-3 rod weighs 0.075 kg. To hold its own weight at the
   ring end it must carry ~0.65 N along its length (the planner models a massless cable, `rod_mass` 0). That pull has
   ~0.29 N sideways towards plate 3; the plan accounts for ~0.05 N. With the integral off, the T stays level but sits
   **64 mm off its point** towards plate 3. With it on, the planner brings the ring back to its point (int_fy -0.26 to
   -0.33 N): drones 0/2 lean ~4 cm towards +y, and the plate-3 carrier ends 2.2 cm higher relative to the ring. Through
   the rigid rod that lifts plate 3, a net +0.16 to +0.27 N against +0.035 N when level, and the ring rolls 7-12 deg.
   The square plan (no resize) builds the same integral (-0.30 N) but tilts only 2.2 deg. Its model still holds a 2.6 N
   phantom cable at plate 9 opposite plate 3.
5. **The planned pull also hung level** (R2977, 0.5 deg, integral on but at half the size, -0.145 N; only 1 run, the
   other repeat dropped). So the tilt needs all three: the resized T plan (plate 3 at its floor), the xy integral and
   the measured pull.
6. **No bug found** in the slot-to-plate mapping, the signs or stale values. One misleading log line (see Q4).

Strongest single piece of evidence: R2823 vs R2965. Same world, layout, ring, rods and unannounced plate-9 release,
plate 3 planned at 0.099 vs 0.131 N. R2823 hung at 0.60 deg, 64 mm off its point; R2965 at 8.7 deg, on its point. Their
planner parameters differ only in `int_k_xy` (0 vs 5.26), `plan_fast_refs` and `plan_sqp_iters`. R2249, with the
integral on and the other two off, tilted 6.6 deg. A one-run check: x1_sqT_ours with `int_k_xy` 0 on today's tree
(`configs/twin/x1_sqT_noxy.yaml` already exists, not flown as of 16:47). It should hang below 1 deg and ~6 cm off its
point if this read is right.

## Q1. When does the offset appear?

Plots: `q1_tilt_aligned.png` (aligned on t_e, then on each run's resize), `q1_tilt_zoom.png` (first 6 s).

| Arm | Run | Resize after t_e (s) | Max \|tilt - no-resize mean\| before the resize (deg) | Departure > 2 deg (held 1 s) after t_e / after the resize (s) | Settled tilt (deg) | Settled minus no resize (deg) |
|---|---|---|---|---|---|---|
| ours | R2965 | 0.31 | 1.32 | 0.94 / +0.63 | 8.69 | +6.5 |
| ours | R2972 | 0.31 | 0.59 | 0.96 / +0.65 | 10.10 | +7.9 |
| late 0.5 s | R2967 | 0.81 | 0.50 | 1.17 / +0.36 | 8.57 | +6.4 |
| late 0.5 s | R2974 | 0.81 | 1.56 | 1.19 / +0.38 | 8.56 | +6.4 |
| late 1 s | R2968 | 1.31 | 1.51 | 2.27 / +0.96 | 9.64 | +7.4 |
| late 1 s | R2975 | 1.32 | 1.88 | 2.23 / +0.91 | 12.22 | +10.0 |
| late 2 s | R2969 | 2.31 | 1.54 | 2.84 / +0.53 | 6.91 | +4.7 |
| late 2 s | R2976 | 2.30 | 1.03 | 2.83 / +0.53 | 6.91 | +4.7 |
| no resize | R2966 / R2973 | none | (their own spread: 2.51 max, 0.42 after t_e+3 s) | - | 2.21 / 2.20 | 0 |

- At the resize tick the planned plate-3 pull drops from ~2.4 N to 0.04-0.36 N, and the 0/6 pulls jump to ~5.3 N.
  The planner logs `surviving ring spans a 180 deg gap (CoG ON/OUTSIDE the hull ...)`.
- R2969 (late 2 s) against the no-resize mean at the same time: the plate-3 carrier's reference rises faster from the
  resize on. Its height is 1.9312 vs 1.9151 m at t_e+3.0 s and 1.9472 vs 1.9241 m at t_e+4.0 s. The no-resize ring swings
  through level to -3.9 deg at t_e+3.5 s; the late-2 s ring stops at +0.3 deg, then climbs to 7-8 deg.
- The planner's lateral integral starts at the same time in every arm, resized or not: about -0.08 N from ~1 s
  (or -0.115 N from ~2.4 s), then up to -0.26..-0.35 N by t_e+5-7 s (`q2_tilt_vs_lateral_integral.png`, middle). The
  resized rings climb to their final tilt as the integral grows. In the phase plot (right panel) every resized run ends in
  one cluster (integral 0.26-0.33 N, tilt 7-12 deg); the no-resize runs end at the same integral but 2.2 deg.
- So the resize is the trigger, timed to within 0.4-1.0 s, and the size of the offset arrives with the integral, not
  with the resize.

## Q2. What differs after the resize?

Settled window t_e+12..18 s. Forces in N. The plate-3 rod's axial tension and its net pull on the plate are inferred
from the tracker log (throttle, attitude, thrust map; matches the logged measured pull to 0.001 m/s^2 in the
measured-pull arms) and the twin's rod: 0.075 kg, centre of mass 0.267 m from the ring end, so 73% of its weight bears
on the plate. Check: the three rods' net vertical pull on the ring sums to 8.37-8.39 N against the ring's weight of
8.44 N in every run.

| Arm (runs) | Tilt about 0-6 / about the other axis (deg) | Plate-3 planned pull, total / vertical (share of the weight) | Plate-3 rod axial (inferred) | Net upward pull on plate 3 | Drone 3 (plate 3) position, out / height (m) | Drones 0/2 lean, y (m) | Planner integral int_f x, y, z (N) | Ring position vs reference |
|---|---|---|---|---|---|---|---|---|
| ours (R2965 / R2972) | 8.69 / 10.10 ; 0.01 | 0.131 / 0.116 (1.3%) ; 0.118 / 0.104 (1.2%) | 0.83 / 0.87 | +0.19 / +0.22 | 0.690 / 1.979 ; 0.692 / 1.983 | +0.039 / +0.040 | 0, -0.29, -0.27 ; 0, -0.30, -0.28 | within 1 mm (x, y), +1.5 / +0.7 mm (z) |
| late 0.5 s (R2967 / R2974) | 8.57 / 8.56 ; 0.02 | 0.134 / 0.118 (1.4%) | 0.83 / 0.83 | +0.19 / +0.19 | 0.689 / 1.978 | +0.040 | 0, -0.28, -0.12 | y +2 mm, z +1 mm |
| late 1 s (R2968 / R2975) | 9.64 / 12.22 ; 0.01 | 0.124 / 0.110 (1.3%) ; 0.106 / 0.093 (1.1%) | 0.86 / 0.93 | +0.22 / +0.27 | 0.691 / 1.981 ; 0.694 / 1.987 | +0.040 / +0.042 | 0, -0.29, -0.22 ; 0, -0.33, -0.26 | z +0.9 / -1.9 mm |
| late 2 s (R2969 / R2976) | 6.91 / 6.91 ; 0.02 | 0.130 / 0.115 (1.3%) | 0.79 / 0.79 | +0.16 / +0.16 | 0.690 / 1.972 | +0.036 | 0, -0.26, -0.27 | within 1 mm |
| no resize (R2966 / R2973) | 2.21 / 2.20 ; 0.02 | 2.596 / 2.285 (25.4%; plate 9's phantom is 2.6 N) | 0.69 | +0.06 | 0.702 / 1.945 | +0.029 | 0, -0.30, -0.54 | z -3.7 mm |
| planned pull (R2977) | 0.54 ; 1.29 (present before the release too) | 0.099 / 0.092 (1.1%) | 0.62 | +0.035 | 0.603 / 1.990 | +0.021 | +0.03, -0.145, -0.125 | within 1 mm |
| T with the xy integral off (R2823, 27 Oct) | 0.60 | 0.099 / 0.088 (1.0%) | 0.65 | +0.035 | 0.744 / 1.953 | -0.038 | 0, 0, -0.32 | **y -64 mm** (towards plate 3) |

- **Axis.** The roll is purely about the 0-6 line (the other axis stays under 0.02 deg in every measured-pull run).
  Plate 3's side rises, plate 9's side drops.
- **The ring's roll is pinned by drone 3's position.** The rods are rigid, so the tilt that closes the plate-3 rod
  (1.000 m from drone 3's pivot) matches the measured tilt within 0.3-0.4 deg in the settled windows and ~1 deg in the
  hover before the release (`scripts/ts.py`; the pivot and rod-length estimate is approximate). Drone 3 sits 2-3 mm from
  its reference. So the tilt is wherever our planner puts the plate-3 carrier relative to the ring. Relative to the ring
  centre, the carrier sits 2.2 cm higher and 1.0 cm further out in R2965 than in R2823.
- **The twin's statics.** Tilt rises monotonically with the net upward pull on plate 3 (`q2_plate3_lift_and_drone3.png`):
  0.035 N -> 0.5 deg (R2977, R2823), 0.06 -> 2.2 (no resize), 0.16 -> 6.9, 0.19 -> 8.6, 0.22 -> 9.6-10.1, 0.27 -> 12.2.
  When level, the plate-3 rod carries only its own weight share (0.62-0.65 N along it). Its sideways part (~0.29 N
  towards plate 3) is the force the integral ends up cancelling.
- **The plan is balanced against itself at the tilt, but only in the tilted runs.** Put the planned pulls (node-0
  tension, logged direction) on the measured ring. The roll moment about the 0-6 axis is then -0.009 to +0.012 N m in
  every resized measured-pull run, i.e. the plan sits at its own massless-model equilibrium. That equilibrium, from the
  logged pulls, is 8.6-9.8 deg, 6.2-7.8 deg without the 0/6 lean, and 5.9 deg at 0.1 N with no lean. In R2977 and R2823 the
  same moment is -0.022 N m (the plan "wants" more tilt, plate 3 pinned at the floor), yet both rings hang level. So the
  floor makes our planner's model unable to plan a level T, but by itself it does not bring the twin ring to that model
  equilibrium. The ring gets there only with the xy integral and the measured pull.
- **Trackers.** Drone tracking errors are 1.1-2.5 mm (resized) and 4.3-4.4 mm (no resize). Each measured-pull tracker
  applies exactly its measured pull (applied = raw). The ring reference is (0, 0, 1.0) throughout in every arm.

## Q3. Is our planner's T always tilted?

No. Plot: `q3_T_history.png`.

| Flight | How the T was reached | Ring, cables | xy integral | Pull | Plate-3 (middle) planned (N) | Settled tilt (deg) | Lateral integral (N) |
|---|---|---|---|---|---|---|---|
| R2823 (27 Oct) | unannounced release of plate 9 | 0.86 kg, 1 m | off | measured | 0.099 | 0.60 (ring 64 mm off its point) | 0 |
| R2312 (22 Oct) | soft detach of plate 9 | 0.86 kg, 1 m | off | measured | 0.099 | 0.69 | 0 |
| R2821 / R2822 (27 Oct) | soft detach (rider chain) | 0.86 kg, 1 m | off | measured | 0.099 | 0.69 / 0.69 | 0 |
| R2457 (23 Oct) | soft detach (rejoin chain) | 0.86 kg, 1 m | off | measured | 0.099 | 0.62 | 0 |
| R2840 / R2842 (28 Oct) | soft detach (rider chain) | 1.15 kg, 1 m | off | measured | 0.097 | 0.68 / 0.67 | 0 |
| R2834 (28 Oct) | soft detach | 1.15 kg, 1 m, throttle cap 0.8 | off | measured | 0.124 | 8.25 (throttle at its cap every tick, int_fz -2.1 N; another cause) | 0 |
| R2249 (21 Oct) | unannounced release | 0.86 kg, 0.55 m | **on** | measured | 0.128 | **6.61** | 0.24 |
| X1 sqT resized, 8 runs (30 Oct) | unannounced release | 0.86 kg, 1 m | **on** | measured | 0.106-0.134 | **6.9-12.2** | 0.26-0.33 |
| R2977 (30 Oct) | unannounced release | 0.86 kg, 1 m | on | planned | 0.099 | 0.54 | 0.15 |
| Rig RIG-1022-t3p_2 (22 Oct) | 3 quads flown as a T (plates 11/2/5, rider on 2) | 0.86 kg, 0.55 m cables | off | measured (mocap) | 0.116 | 2.56 mean magnitude (mostly swing); the steady part about the pair axis is 0.9, rider side **low** | 0 |

Uneven but not a T (twin, xy integral on, no planned pull at the floor):

| Flight | Formation | Lowest planned pull (N) | Settled tilt (deg) | Lateral integral (N) |
|---|---|---|---|---|
| R2766 8 quads, after release 1 / 3 | n=7 / n=5, CoG inside the hull | 1.03 / 1.50 | 1.06 / 1.21 | 0.22 / 0.24 |
| R2766 after release 2 / 4 | n=6 / n=4, even | 1.74 / 2.65 | 0.00 / 0.03 | 0.00 |
| R2708 (3/9/1/5 -> even triangle 11/7/3) | even triangle | 3.30 | 0.03 | 0.005 |
| X1 h4 cell (4 -> even triangle) | even triangle | - | 0.05 | - |

- The tilt scales with how uneven the formation is, but only through the lateral integral. Even formations need no
  sideways force and hang level (0.00-0.08 deg). Uneven formations with the CoG inside the hull carry ~0.22 N of it
  and tilt ~1.1 deg, about 5 deg per N. The T carries 0.24-0.33 N and tilts 6.6-12.2 deg, about 25-37 deg per N,
  because its middle carrier, at the floor, gives no roll stiffness about the 0-6 axis.
- R2249's registry note (21 Oct) put its 6.7 deg down to "the 0.1 N floor against the plate-3 tether's weight". R2823
  and the six soft-detach T's have the same floor and hang level, so the floor is not sufficient.
- The rig T had no xy integral and shows no plate-3-high offset (0.9 deg the other way). That fits the twin's
  integral-off T's. Its cables can also go slack, so the rider cannot push or lift the plate the way a rigid rod does.

## Q4. Anything that looks like a bug

- **Slot-to-plate mapping after the resize: correct.** The log reads `slot->drone [0, 2, 3]; attach angles (cw)
  [0.0, 180.0, 90.0]` = plates 0/6/3. The planned and measured cable azimuths per slot agree within 0.001 rad
  (az1_* vs az0_*).
- **Signs: consistent.** The tilt, the plate-3 rod's sideways pull (-y on the ring) and int_fy (-y) agree in every run.
- **Stale values: none found.** The ring integral holds still for ~1-2.5 s after the release and then resumes. It
  carries its square-time value into the T: -0.08 N at the 2 s arm's resize, ~0 at detection. hover_x/y stay 0. The
  planned plate-3 tension at node 0 dipped once to -0.031 N (R2965); node 0's tension is not bounded
  (`src/mpc_planner/mpc_planner/planner_ocp.py:151`, only observed states pinned), by design.
- **Misleading log line, not a control bug.** `src/dissipative_planner/dissipative_planner/dissipative_node.py:1763`
  says a 180 deg gap means "the load cannot hang level". The twin's T hung within 0.7 deg in 8 flights, and the rig T
  did too.
- **Confound for R2977, not a bug.** Every planned-pull run (R2970, R2971, R2977, R2978) already hovered 1.7-1.8 deg
  tilted about the other axis before the release, with drones 0/2 at 1.972 / 1.955 m. That fits the per-drone
  `sim_thrust_offset` (0.181-0.187 vs the typed 0.185), which the planned pull does not absorb and the measured pull does.
- **Where the integral enters the plan** (for whoever checks the path): `src/mpc_planner/mpc_planner/planner_node.py:828-831`
  sets one force, `d_L`, in two places. It goes into the plan's ring dynamics (`planner_solver.py:256`) and into the
  cable-direction references, which tilt with it (`reference_builder.py:244`, g_eff = g - a + d_L/m; at -0.28 N that is
  1.9 deg about the 0-6 axis). The tension floor is `planner_ocp.py:63` (default 0.1 N) and `:167-170` on every node but
  0. The T's nominal tensions are floored at the same 0.1 N in `geometry.py:240` (`balanced_tensions`).

## Caveats

- The between-flight comparison is not one variable. R2823 and the soft-detach T's also had `plan_fast_refs` off and
  `plan_sqp_iters` 0, and older trees. R2249 shows the tilt with those two off, but there is no T with the integral off
  and fast refs/SQP on. The one-run check above settles it.
- Only one planned-pull T survived (R2977), and it was pre-tilted about the other axis.
- The plate-3 rod forces are inferred: the thrust map and the rod's mass split. The vertical balance closes within
  0.07 N, but the split along/across the rod rests on the rod model.
- The twin's links are rigid rods, so in these runs the plate-3 carrier's position sets the ring's roll directly. On the
  rig a cable can go slack, which bounds this towards "plate 3 high" only.

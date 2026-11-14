# Card: the rod in the plan (31 Oct 2026)

Owner Wesley (31 Oct: chose this fix for the T). Code in the main checkout's `src/`, uncommitted, behind
`plan_rod_level`, default off, simulation only: the real-world launch refuses it. The runs below are **PROPOSED and
exploratory**, one each, none flown, no bars frozen. New code in the flight loop: the critic is due before any claim
run (`## Critic` is empty).

## 1. Question

With the rod's weight in the planner's model and references that balance it, does the T hang level on its point with
no sideways integral? And can `int_k_xy` then be 0 everywhere in simulation?

## 2. Cause

Full audit: `docs/analysis/why_integral_code_2026-10-31.md`. In short, a hinged rod with mass pulls its plate outward
by w cot e beyond what the massless plan says, with w = (1 - lambda) m_r g = 0.54 N. That is 0.31 N at 60 deg. It
cancels on even formations and not on a T, and it is the force the integral learns. Flown since that audit:

| Run | Setup | Audit expected | Flown |
|---|---|---|---|
| R3235 | T, `int_k_xy` 0, elevation 70 | +4.6, y -49 | +4.58, y -49 |
| R3236 | T, `int_k_xy` 0, elevation 50 | -3.2, y -71 | -2.86, y -73 |
| R3237 | T, 5.26, elevation 70, late resize | d_y -0.17 | d_y -0.181, roll +10.40 |
| R3238 | T, 5.26, LAND at +60 s | +6.7, or towards +18.8 | +16.90, d_y -0.428 |
| R3239 | M1 four, `int_k_xy` 0, before the release | y +60 to +70 | y +46, 2.12 deg |
| R3400 | Rods 1 g, `int_k_xy` 0, floor 0 | -0.1, y -1 | -0.19, 0.5 mm off |
| R3401 | Rods 1 g, 5.26, floor 0.1 | +5.2, d_y +0.02 | **+19.00, d_y -0.13: missed** |

R3400 is the direct evidence: with no rod weight and no floor, no integral is needed. With the integral off every
prediction held. With it on the forces held and the roll missed twice on which root the flight lands on (section 5).

R3000 flew the rod in the plan and tilted the other way (-5.19). That was not the model. It was the reference: the
code asked plate 3 for 0.25 N where a level ring needs 0.83 N.

## 3. The change

**What the plan knew already.** `rod_mass` and `rod_com` put the rod in the OCP's model (option F, 3 Oct). That code
is right and is not touched. What was wrong is what the plan was asked to fly: references that are not a rest state of
that model.

**The tension reference, in plain words.** The old code solved all six balance rows at once, forces in N and moments
in N m, in one least squares. On a T they cannot all be met, so the force rows won and a roll moment was left on the
ring. The new code solves only lift and the two level moments, from the equal split. Tensions cannot make the sideways
force anyway: the cables' lines all meet at one apex, so any tension that changes it also adds a moment. On the T the
middle carrier now gets what holds its own rod up, and its plate feels no vertical force.

**The cable-direction reference, in plain words.** That leaves the rod's sideways pull, which only a lean can take.
Each cable leans towards the opposite side in proportion to its tension, the least total change of direction. On the T
the pair leans 1.6 deg and the middle 0.2 deg. These references are an exact rest state of the model: ring level, on
its point, nothing to learn. On an even formation nothing is left over, so the directions and the equal split stay as
they are.

| On the T, 0.86 kg, rod 0.075 kg, CoM 0.27 m | Pair (N) | Middle (N) | Pair lean (deg) | Left on the ring |
|---|---|---|---|---|
| Code path, R3000 | 5.934 | 0.251 | 0 | Roll moment -0.106 N m, y force 0.039 N |
| Level tensions only | 5.646 | 0.826 | 0 | y force 0.327 N |
| **`plan_rod_level`** | **5.665** | **0.791** | **1.6** | **None** |

**Where** (planner files in `src/mpc_planner/mpc_planner/`, the detach node in
`src/dissipative_planner/dissipative_planner/`, launch files in `src/bringup/`):

| File:line | What |
|---|---|
| `geometry.py:211` | `_static_balance`: the same A and b, moved out of `balanced_tensions` |
| `geometry.py:231, 250-254` | `balanced_tensions(level=False)`: lift and level moments only when on |
| `geometry.py:258` | `level_rod_refs`: tensions and leaning directions |
| `geometry.py:276` | `rod_neutral_tension`: the pull at which a rod carries itself |
| `reference_builder.py:34, 57-58` | `unload_tensions(level=False)`: lift and level moments only when on |
| `reference_builder.py:69, 84` | `rebalance_incumbents` counts the rods when given them |
| `reference_builder.py:101-111, 174-175` | The builder's `rod_level`: the references, and the join's blend |
| `planner_node.py:435-438, 487, 1869` | The parameter, refused without `rod_mass`, at start and at a resize |
| `dissipative_node.py:1912-1919, 1923, 1945` | A soft detach: the unload target and the gate read the rod's own pull |
| `dissipative_node.py:3312-3313, 3324` | A join: the level references, with the rods |
| `dissipative_node.py:3369-3373` | A join's blend: from the leaning directions flown, the newcomer's rod counted |
| `bringup/real_mode.py:139, 185-194` | Launch: refused on real flights, and with no rod to read |
| `bringup/profiles.py:106` | `rod_model`: the one resolver, for the launch, `--check` and the prebuild |
| `config/common.yaml:55-57`, `config/sim.yaml:35-41, 68` | The switch and the twin's rod, off for the network |
| `tools/check_geometry.py:367-400` | `rod_mismatches`: `--check` refuses a world the rod does not fit |
| `tools/prebuild_planner.py:74`, `tools/remote/lab_batch.py:230, 401` | The solver built is the rod flown |
| `src/dissipative_planner/test/test_plan_rod_level.py` | 46 tests, the real `resize_fleet` and `_attach_ocp` |
| `tools/test/test_rod_model_check.py` | 16 tests: the resolver, the worlds, `--check`, the prebuild |
| `src/bringup/test/test_real_mode_launches.py` | The launch test, last in the file |

Off, every path is the old one. Tests hold `balanced_tensions` and `unload_tensions` against copies of the old code,
bit for bit.

**Prebuild.** `plan_rod_level` itself no: it changes references only, and the OCP source is untouched. `rod_mass` yes:
the rod is compiled into the model, and the cache signature holds its mass and CoM. `rod_com` 0.267 here against
R3000's 0.27 means R3000's solvers are not reused. `tools/remote/lab_batch.py` does it per slot in its own scratch
(`--from-config`), so it touches no running batch. On the laptop: `python3 tools/prebuild_planner.py --from-config
configs/experiments/<cfg>.yaml`. The 8-quad chain builds n = 2 to 8, about 12 min cold.

### The soft detach

Unload the leaver to 0.5 N and its rod hangs on the plate, because the rod alone pulls 0.79 N. With the switch on, the
leaver's target is its rod's own pull (`rod_neutral_tension`), the survivors' target solves lift and the level moments
only, and the release gate is that pull + 0.3 N. `detach_unload_t` is not read. The opposite carrier, the T's rider
to be, comes out at the same pull.

| Square, plate 9 leaves, rod in the model | Pair (N) | Leaver, opposite (N) | Plate: down (N) | Plate: sideways (N) |
|---|---|---|---|---|
| 0.86 kg, `detach_unload_t` 0.5 N | 5.957 | 0.500 | 0.254 | 0.165 |
| 0.86 kg, `plan_rod_level` | 5.664 | 0.793 | 0 | 0.311 |
| 1.15 kg, `plan_rod_level` | 7.306 | 0.793 | 0 | 0.311 |

The second row is an exact rest state of the model. So is the same target on the 8-quad ring and on the M1 four, where
the old target left 0.039 N and 0.107 N m. After the release the T's references are 5.666, 5.666, 0.792 N at 0.86 kg,
and 7.308, 7.308, 0.792 N at 1.15 kg: the tensions step 0.002 N.

**The lean steps at the release: no change.** The plate's 0.31 N sideways goes at the release itself, so a lean that
arrives at once matches the plant, and a blended one would stay wrong for the blend's length. The planner's real OCP
(3 quads, rod in the model, ring at rest on the square's directions) plans nearly the same first second either way:

| Planned after the release | 0.1 s | 0.25 s | 0.5 s | 1 s |
|---|---|---|---|---|
| Pair drones' sideways move (mm), lean stepped in | 13.9 | 21.4 | 24.1 | 19.5 |
| Pair drones' sideways move (mm), no lean | 13.6 | 20.8 | 22.6 | 18.1 |
| Ring y (mm), lean stepped in | -1.4 | -4.5 | -12.3 | -7.9 |
| Ring y (mm), no lean | -1.4 | -4.5 | -12.8 | -10.2 |

The first move comes from the force that is gone, not from the reference. This is one solve, not a closed loop, and no
soft release has flown with the switch on (run 13).

**The sideways force in a join's blend: no change.** A blend balances lift and the level moments and leaves the
sideways row out, as it does with the switch off. Its peak stays the size that has flown:

| Join, peak of the reference's net sideways force (N) | On | Off |
|---|---|---|
| T to square | 0.456 | 0.399 |
| Pair to T | 0.008 | 0.05 |
| Triangle to the M1 four | 0.28 | 0.27 |

### The default in simulation

The line to flip: `plan_rod_level` under `rod_in_plan` in `src/bringup/config/sim.yaml:39`. The rod is one typed pair
next to it (`rod_level_mass` 0.075, `rod_level_com_frac` 0.2667, times `cable_len`). A typed `rod_mass` or `rod_com`
wins. `bringup.profiles.rod_model` resolves it for the launch, for `--check` and for the prebuild, so those three cannot
disagree. `--check` then reads every rod in the world, the newcomer's arm too, against that one rod (0.002 kg, 0.005 m):
another mass, another CoM, rods that differ or a segmented arm fail `--check`, unless named under `geometry_exempt` or
the config says `mis_seed`.

| `configs/experiments/` at the flip, 813 files | Files |
|---|---|
| Change: detach 517, attach 208, carry 2 | 727 |
| Already fly it (typed) | 25 |
| Legacy: not touched, and a typed switch is refused (the legacy profile has no rod to read) | 11 |
| Network mode: not touched (`modes.network`, the port's arms keep their creep and lift) | 50 |
| Would fail `--check` at the flip | 0 |

Also off: SIL (its plant's rods have no mass) and real flights (the launch refuses the switch). The worlds checked: 1 m
and 0.55 m rods, `_p115`, the 8 and 9 quad worlds, the attach and pickup worlds. The one world that fails is
`four_rigid_ground_rig_L100_rod1g.sdf` (rods 1 g), so a config on it must type its rod.

The references are read only by `planner_node.py` and `dissipative_node.py`. The network's law does not read them.
Tejen's packages (`tejen_mission`, `tejen_mpc`) read neither them nor our reference topics, so in an attach flight only
our own side changes: the carriers' references, and the fold-in once we take the newcomer over.

### The solvers at the flip

| Where | What changes |
|---|---|
| Lab, per slot | `lab_batch.py` warms through `--from-config` for any config with a rod, and rebuilds per mass or rod |
| Laptop, `generated/` | One solver per size. `prebuild_planner.py --from-config <cfg>` first, or it compiles at start |
| Plain `prebuild_planner.py` | Rod 0, with or without the flip. It no longer prepares a simulated flight |
| Real flights | `~/mdc_rig_generated` and the 0.86 kg copy are their own folders, rod 0, from `prebuild_rig.sh` |

The OCP sources are untouched and the cache signature at rod 0 is unchanged, so the real flights' solvers stay fresh.
A cache at another rod is not read silently either: the signature differs and it rebuilds.

## 4. What it does not do

- It does not touch the real flights. Real cables are not hinged rods, and a slack cable has no sideways force at all.
- It does not remove the tension floor. On a T the middle reference is now 0.79 N, far above it, so the floor stops
  binding.
- It does not move the integral's force from the CoG to the plate. With the integral on it learns about nothing here,
  so that error has nothing to act on. With no `plan_rod_level` both remain as they were.
- It does not fix the pull clamp (`tracker_node.py:1688`). At the level T the measured z pull stays negative, so it
  does not bind in the solved cases: the roll is the same with it off, and with thrust offset 0.185 or 0.187.
- With the switch off it does not change a soft detach's unload target (`unload_tensions`, `reference_builder.py:34`):
  still the old row mix and `detach_unload_t` 0.5 N. With it on, see "The soft detach".
- It does not model the rod's inertia, only its weight. On a path the rod's own acceleration stays unmodelled.
- It is only as right as the typed rod. Offline, a 10 % wrong mass moves the roll by 1 deg, and y by 3-7 mm.

## 5. Offline check (no Gazebo)

The fixed-point solver of the audit: the real planner OCP with this repo's new references, the real tracker OCP, the
rods' statics. Typed offsets, `int_fz` -0.30 N. These are rest states, not flights.

| Case | Integral | Roll | y | d_y | t3 planned | T_D plant | Roots found |
|---|---|---|---|---|---|---|---|
| T, 0.86 kg | 0 | +0.01 to +0.05 | -1.5 | 0 | 0.804 | 0.79 | one, starts -10 to +20 deg |
| T, 0.86 kg | 5.26 | +0.35 | 0.0 | -0.008 | 0.805 | 0.80 | one, starts -8 to +19 deg |
| T, 1.15 kg | 0 | +0.10 | -0.9 | 0 | 0.804 | 0.80 | one |
| T, 1.15 kg | 5.26 | +0.27 | 0.0 | -0.006 | 0.805 | 0.80 | one, starts -8 to +14 deg |
| T, elevation 70 | 0 | +0.07 | -1.4 | 0 | 0.765 | 0.76 | not searched |
| T, elevation 50 | 0 | +0.02 | -1.1 | 0 | 0.870 | 0.85 | not searched |
| T, elevation 70 | 5.26 | +0.26 | 0.0 | -0.006 | 0.766 | 0.76 | not searched |
| Plant rod 10 % over the plan | 0 | -0.97 | -6.7 | 0 | 0.799 | 0.85 | not searched |
| Plant rod 10 % under the plan | 0 | +1.05 | +3.3 | 0 | 0.808 | 0.74 | not searched |
| Plant CoM 0.30, plan 0.267 | 0 | +0.55 | +0.9 | 0 | 0.806 | 0.80 | not searched |

- **With a massive rod the planned tension now equals the rod's pull.** No flown case before did.
- **The level holds at 50, 60 and 70 deg.** R3235 and R3236 show the massless plan does not.
- **The solver has no result for four real rods** (square, M1 four, the chain): it does not converge. Those expected
  values below are reasoning, not solved, and the table says so.

**Root selection with the integral on is unreliable, and what that means for run 2.** R3401 is the warning. From
starts at 0 and +10 deg the solver found +5.15 and +6.32. The flight swung through 10-19 deg in the first 4 s with the
middle carrier on the floor, then crept up to 19.37 deg by LAND. Starts of +19 and +25 did not converge in 30
iterations, so the solver has no rest state to offer there at all. R3238 is the same story at 16.9 deg. So a list of
roots says what rest states exist, not where a flight lands, and it cannot rule out a slow creep where no root exists.
For run 2 this means:
- +0.35 deg and -0.008 N is the only rest state found, from starts up to +19 deg. A start at +25 had not finished in
  30 iterations, at +1.41 deg and on its way down. **It is not a prediction that the flight gets there.**
- In the runaways the plan was on the 0.1 N floor, with no way to ask plate 3 down. Here it has about 0.7 N of room.
  That reasoning is inference, not shown.
- So run 2 flies a 60 s hold, as R3238 did, and its falsifier includes still moving. Read run 1 first: integral-off
  predictions have all held.

## 6. Proposed runs (one each, exploratory)

Each changes one thing against the named baseline: `plan_rod_level` with `rod_mass` 0.075, `rod_com` 0.267. All
configs pass `python3 tools/run_experiment.py <cfg> --check`.

| # | Config (`configs/experiments/`) | Baseline | Expected | Falsifier |
|---|---|---|---|---|
| 1 | `x1_sqTrodl_noxy` (audit run 8) | R3202, R3000 | 0.0 deg, 1.4 mm off, t3 0.80 | Over 1 deg or 10 mm |
| 2 | `x1_sqTrodl_ours` (audit run 9), 60 s | R3238, R3000 | +0.35, d_y -0.008 | Over 1 deg, \|d_y\| over 0.05, moving |
| 3 | `x1_sqTrodl_noxy_e70` | R3235 | +0.07, 1.4 mm off | Over 1 deg or 10 mm |
| 4 | `b4rodl_hover_L100_slope` | R2349, R2350 | No change, plan t 3.23 | Over 0.2 deg, 2 mm or 0.01 N |
| 5 | `x1_h4rodl_ours`, the even cell | R2918, R2934 | No change after the release | See below |
| 6 | `x1_h4rodl_noxy`, M1 four | R3239 | Under 10 mm, 0.5 deg (not solved) | Over 10 mm or 1 deg |
| 7 | `x1_fTrodl_noxy`, figure-eight T | R3211, R3208 | Under 1 deg, path under 89 mm (not solved) | Either over |
| 8 | `rj3_pairrelease_rejoin_L100_tip_rodl` | R3003 | 0.0 deg, 1.4 mm off | Over 1 deg or 10 mm |
| 9 | `rj3_pairrelease_rejoin_L100_tip_rodl_kxy` | R3002 | +0.35, d_y -0.008 | Over 1 deg, \|d_y\| over 0.05 |
| 10 | `x1_sqTp115t95_rodl_noxy` | R3234 | +0.10, 0.9 mm off | Over 1 deg, 10 mm, or on the cap |
| 11 | `x1_sqTp115t95_rodl_ours` | R3232, R3233 | +0.27, d_y -0.006 | Over 1 deg, \|d_y\| over 0.05, or on the cap |
| 12 | `s8_chain4back_R50_L100_it5_rodl_noxy` | R2766, R2767 | Under 10 mm, 0.5 deg (not solved) | Over 20 mm, 1 deg |

Twin parameter files: `configs/twin/x1_sqT_rodl.yaml`, `x1_sqT_rodl_noxy.yaml`, `s_chain_pair_rodl.yaml`,
`s_chain_pair_rodl_noxy.yaml`, `chain_pair_tip_rodl.yaml`, `chain_pair_tip_rodl_kxy.yaml`, `drop_and_land_rodl.yaml`.

Notes on the runs:
- **Run 1 first, then 3 and 4.** Run 1 answers the question and run 3 tests the cancellation directly. If run 1 fails,
  stop.
- **Run 4 is the strict "nothing may change".** Run 5 is X1's even cell: its M1 four before the release is uneven, and
  that part should change (expected tilt down from 1.6 deg). After the release it must not: settled under 0.1 deg,
  integral under 0.01 N, and peak and dip within 1 deg and 5 mm of R2918 and R2934.
- **Run 12 answers "can `int_k_xy` be 0 everywhere in simulation"**: it flies even and uneven formations, releases and
  joins, in one flight. Runs 6, 7 and 8 are the same question on single cells. The answer is yes only if all of them
  hold.
- **Runs 8 and 9 are the first flights of the join path** with the switch on (blend from the lean, the newcomer's rod
  counted).
- **Watch in every T run, no bar yet:** the dip and the peak. R3000's dip was 115 mm against ours' 100 mm. The first
  seconds after a release were not solved offline.
- **Run time:** the X1 cells about 9 min wall each on the lab PC, run 2 longer, run 12 about 25 min. Score with
  `tools/x1_metrics.py`. Run 6 needs the window before the release, as R3239 was read.

### Default-ready runs (31 Oct evening, PROPOSED)

Each is its flown base plus `plan_rod_level: true` in the launch arguments, nothing else, with the rod read from
`sim.yaml`. All pass `--check`. Expected numbers only where solved. In runs 13 and 18 they are the T before the next
event.

| # | Config (`configs/experiments/`) | Baseline | Expected | Falsifier |
|---|---|---|---|---|
| 13 | `c4_softdetach9_rider_L100_tip_p115_t090_rodl` | R2840, R2842 | T +0.10, 0.9 mm off | See below |
| 14 | `x2_full_fig8_soft_rodl` | R3293, R3296 | Not solved | See below |
| 15 | `x4_relay_short_rodl` | R3244 | Not solved | A swap not done, an event over 5 deg, a RELAY HOLD |
| 16 | `x8_rim_ours_rodl` | `x8_rim_ours`, not flown | +0.35, d_y -0.008 | Over 1 deg, \|d_y\| over 0.05, last 5 s |
| 17 | `rodl_u_meas_d00` | R2459, R2629, R2657 | Not solved | See below |
| 18 | `rj4_softdetach9_rejoin_L100_rodl` | R2578, R2579, R2592, R2596 | T 0.0, 1.4 mm off | See below |

Notes on these runs:
- **Run 13 first: the unload is the new code.** It falsifies on an unload cancelled, the T over 1 deg
  or 10 mm, the peak after DETACH over 26.6 deg, the rider's release not detected within 2 s, or the pair stage on
  `throttle_max`. The peak was not solved. The rider is now at its rod's own pull, not ~0.1 N, and the tip check has
  not seen that.
- **Run 18 next, then 17 and 16.** Run 18 falsifies on REJOIN_ABORT, no weld within 30 s, a fold-in over 5 deg (E1's
  bar, baselines 1.27-1.55) or the T over 1 deg or 10 mm. Run 17 is the first attach with the switch on: it falsifies
  on no weld, a fold-in over 5 deg (baselines 1.76-2.21) or a steady tilt not under 1.55 deg (baselines 1.55-2.29). The
  M1 four flew 0.04 and 0.05 deg before a release (R3422, R3421), on a hover, not this path.
- **Run 16 needs its own baseline:** `x8_rim_ours` has not flown, so fly it next to this one. Its LAND is 30 s after the
  release, where R3420's number is from 60 s.
- **Run 14 falsifies against its baselines, not its card's frozen bars**, which stay as they are: an event tilt over
  28.69 deg, a fold-in over 5 deg (baselines 4.50, 5.19), a lap 6 path over 10.02 mm, or a drop. Runs 14 and 15 are the
  longest, 700 s and 450 s of mission. Run 15 builds n = 2 to 8.

## 7. What Wesley decides

1. Whether to fly runs 1-5 first, as ordered above, then the rest. Recommended.
2. If runs 1, 6, 7, 8 and 12 hold: `int_k_xy` 0 with the rod in the plan for the simulation's claim cells, or keep
   5.26 (run 2). That is a default change, so it needs his word and the reviewer.
3. The fallback stays as in `docs/SESSION_STATE.md` if this is not clean by Wed 4 Nov: `int_k_xy` 0 typed in the T
   cells, with the cancellation stated.

## 8. Results (31 Oct, exploratory, one run each; no bars were frozen)

Laptop, one at a time. Roll in deg, + plate 3 up. Off is the ring's horizontal distance from its point over the last 5 s before LAND, in mm. Expected is section 6's or the critic's, written before the run.

| Run | Case | Roll | Off | Expected | Without the switch |
|---|---|---|---|---|---|
| R3412 | T, 60 deg, `int_k_xy` 0 | +0.11 | 1.2 | 0.0, 1.4 | +0.60, 64 (R3202) |
| R3413 | T, 70 deg, `int_k_xy` 0 | +0.10 | 1.1 | +0.07, 1.4 | +4.58, 49 (R3235) |
| R3415 | T, 50 deg, `int_k_xy` 0 | +0.01 | 0.3 | level | -2.86, 73 (R3236) |
| R3416 / R3417 | plan's rod 10 % heavy / light | +0.97 / -1.14 | 4.4 / 6.1 | +1.05 / -0.97 | |
| R3420 | T, 60 deg, 5.26, 60 s | +0.28 | 0.4 | +0.35 | +16.9, 1 (R3238) |
| R3426 / R3427 | 1.15 kg, 0, 5.26 | -0.11 / +0.06 | 0.1 / 0.1 | +0.10 / +0.27 | +0.58, 65 / +7.37, 0 (R3234, R3232) |
| R3424 / R3425 | Join into a T, 0, 5.26 | +0.05 / +0.21 | 0.1 / 0.1 | 0.0 / +0.35 | +0.82, 64 / +6.91, 0 (R3003, R3002) |
| R3422 | M1 four before the release, 0 | 0.04 | 0.2 | under 0.5, 10 | 2.12, 46 (R3239) |
| R3418 | T, no resize, 0 | -2.08 | 60 | | |
| **R3419** | **T with drone 0 as the rider, 0** | **-3.37** | **9.5** | **under 1, 10: FALSIFIED** | not flown |

With the rod in the plan the resize beats no resize on all three: 0.11 against 2.08 deg, 1.2 against 60 mm, dip 104.7 against 111.5 mm. Even layouts: R3414 as R2349 (planned pull 3.23 N against 2.46, it now carries the rods), R3421 after the release as R2918, R2934. Planned rider pull 0.80 N, against the plant's flown 0.78-0.80 N (R3201, R3202).

**The miss.** The T holds for rider drone 3 and not drone 0. The plant's offsets are 0.181, 0.185, 0.187, 0.187 against the typed 0.185, so drone 0 thrusts more than its map thinks. The critic predicted the clamp at `tracker_node.py:1688` binds there; the raw pull reads over zero on 73 % of ticks (49 % for R3412). That reading is not tested. Riders 1 and 2 are not flown.

**Flown after that** (registry R3428-R3443, one run each, exploratory):

| Run | Case | Result | Baseline |
|---|---|---|---|
| R3428 | 8 -> 4 -> 8, hover, `int_k_xy` 0 | back at 8: 0.4 mm, 0.07 deg; settled 0.04-0.05 deg; fold-in peaks 0.15-0.53 deg | R2766 6.36 mm, 0.99 deg; 1.21; 1.27-1.42 |
| R3433 | Soft detach, rider released, 1.15 kg | detach peak 23.83 deg, rider 2.88 deg; found by the marker | R2840, R2842 24.9-29 deg |
| R3434 | Soft detach, rejoin | detach peak 13.35 deg, fold-in 2.24 deg, 0.05 deg, 1.1 mm after | |
| R3440 | X8 rim, 5.26 | +0.58 deg, 0.3 mm; dip 107.4 mm | R3439 +16.56 deg; dip 92.1 mm |
| R3435 | 3-to-4 attach, 5.26 | **2.62 deg, missed** | R2459, R2629, R2657: 1.45-2.11 deg |
| R3443 | 3-to-4 attach, `int_k_xy` 0 | 0.02-0.25 deg after the weld settles | as above |
| R3438 | X2-full announced, `int_k_xy` 0 | detach peak 19.58 deg, release 4.73 deg, fold-ins 2.01, 1.95 deg; no drop | R3293, R3296: 28.4-28.7, 5.2-5.4, 2.2-5.2 deg |
| R3437 | The short relay, `int_k_xy` 0 | event peak 1.50 deg, path 13.0 to 22.1 mm | R3244: 2.28 deg, 16.6 to 43.8 mm |
| R3441, R3442 | The T's rider, offset swapped | +0.04 deg at 0.187, +3.37 deg at 0.181: R3419's miss follows the offset, the clamp | |

Not read: per-carrier planned pull against the plant's, the critic's ask. Dip: larger than today's planner in the cells read (R3440 107 against 92 mm).

## 9. Reviewer (1 Nov ~03:35): FAIL for a default; the level T holds

The reviewer recomputed section 8 from `run.csv` and the node logs. **The flip does not happen on this evidence.** Section 8's numbers stay as written; these correct them.

| Item | Section 8 | Recomputed | Run |
|---|---|---|---|
| Detach baseline | 24.9-29 | 24.90-26.55 | R2840, R2842 |
| Roll, off | +0.05, 0.1 (8 s) | +0.11, 0.2 | R3424 |
| Off | 0.1; 0.3; 0.3 | 0.39; 0.37; 0.44 | R3426; R3415; R3440 |
| Steady tilt, the falsifier's metric | 2.62; base 1.45-2.11 | 2.00; base 1.55-2.29 | R3435 |
| Tilt after the weld | 0.02-0.25 | 0.02-0.60 | R3443 |
| Fold-ins, the baseline's metric | 2.01, 1.95 | 1.72, 2.90 | R3438 |
| Settled | 0.04-0.05; 1.21 | 0.03-0.06; 1.27 | R3428; R2766 |

- **Second variables, so these comparisons do not isolate the rod:** R3428 and R3437 also change `int_k_xy`. R3443 also changes the filter (ema to slope), and R3435 against R3443 also differ in catch (2.0 against 0.1 cm) and weld time (55.7 against 63.4 s), so "the setting, not the rod" is unsupported. R3434's baselines flew ema. R3442 also changed drone 0's offset. The clamp never flew off.
- **R3418 had a baseline:** R3203 (-2.15 deg, 70 mm, dip 108.0).
- **"The resize beats no resize" holds at n = 1, 0.86 kg, rider 3 only.** The point is because no resize lost its integral: at 5.26 with the switch off it holds 0.1-0.2 mm (R2966, R2973). Dip margin 6.8 mm (8.9 with the switch off, R3202/R3203). At 1.15 kg there is no arm: R3426 133.7 against switch-off R3233 133.5. R3419's 3.37 deg exceeds 2.08 (a different leaver).
- **Unreported trip:** R3421's rise 7.34 deg against R2918's 5.73 passes the 1 deg bar's limit. **R3401:** omits `resize_delay_s` 2.0; the force was wrong too (+0.02 predicted, -0.13 N).
- **Dip rises in all seven hard T pairs, +5.6 to +15.3 mm** (R3415, R3440 worst). It falls on soft detach (R3434 44.5 against 58.1-61.0).
- **Other transients:** T peak no harm. T6 worse (R3427 10.3 against 8.2, R3433 7.4 against 5.8-6.4). Tilt after 6 s better on hard releases, worse on soft (R3433 3.16 against 1.02-1.14, R3434 3.27 against 2.28-3.07, R3438 4.09 against 3.35-3.38). R3434's fold-in 2.24 against 1.27-1.55.
- **One tree:** `dissipative_node.py` and `reference_builder.py` were edited at 20:11, after R3412-R3428.
- **What is fair now:** simulated hinged rods, hover, riders 3 and 1: a level T at 50-70 deg (0.01-0.11 against 0.60-4.58 deg, 0.4-1.2 against 49-73 mm off), and no runaway at 5.26 (+0.28 against +16.9), one run each. Not shown: real cables, rider 0, disturbances.
- **To fly before a default:** repeats on one tree; one-variable arms for R3428, R3437, R3443; no resize at 1.15 kg with the switch.

REVIEWER: fail: the level T holds, but n = 1, second variables, the unreported R3421 trip, an understated dip.

## 10. Reviewer's second look (2 Nov ~23:36, on R3458-R3473): pass with corrections

All 1 Nov numbers were judged after the run (Wesley 2 Nov: no bars frozen for simulation). All 16 runs share one tree, identical to `src/` now; the rows exist; read-backs as typed; no abort, drop, failed solve or compile; late ticks under 0.5 %. The 31 Oct 20:11 edits touch soft detach and launch only, so R3412-R3428 (no DETACH) stand.

| As reported | Recomputed | Runs |
|---|---|---|
| "8 -> 4 -> 8 as R2766", "integral off alone 40 mm back at 8", "it is the rod" | **Wrong: both no-rod arms are invalid.** `/drone_5`'s release was never detected (rise 2.5-2.6 cm under the stretch check's 3 cm rule): n stops at 5, 7 on the ring and 8 planned back, and two of the fold-in peaks (0.16, 0.14) never happened. Checked by hand: their n goes 7, 6, 5 where R3471's goes 7, 6, 5, 4 | R3469, R3470 |
| Attach after the weld "one variable" | Not isolated: the newcomer flies `ema` on R3458-R3460 and `slope` on R3461, R3462 (checked in the configs), and the catch varies (0.9, 1.3, 0.4, 0.5 cm). Same switches, R3435: 2.36 deg | R3459-R3462 |
| `um_base` did not weld | Void: tip 10.3 mm off at 12 mm/s for 38 s, as the arms that welded. A noisy speed gate is inferred, not logged. 2 of 74 runs time out here | R3458 |
| Resize dip gain 6.8 | 1.5-6.1 | R3463-R3466 |
| 1.15 kg gain 18.5 | 10.9 with R3426 | R3468 |
| Trip 7.34 against 5.73 deg | R2934 7.16; absolute peak 7.39 against 4.29, 5.62 | R3421 |
| Relay swaps 25 of 26 | 26: the event file lost a line (checked by hand) | R3456 |
| Same-day baselines | 30-31 Oct, other trees | |

**What holds.**
- **The T, simulated hinged rods, rider 3, hover:** 0.01-0.07 deg and 1.0 mm off, against 8.3-10.5 deg, on repeats.
- **The chain with the rod:** back at 8, 0.37 mm at `int_k_xy` 5.26 and 0.37, 0.50 mm at 0, all 0.07 deg, against 6.36 mm and 0.95-0.99 deg on R2766, R2767 (`ema`, old tree). **There is no valid no-rod chain on this tree, so the rod's share of that is not isolated.**
- **What the resize is worth with the rod:** 0.86 kg 2.0 deg and 59 mm, T6 2.1-2.8 s; 1.15 kg (one run) 1.2 deg and 57 mm, T6 2.8 s; on the peak, nothing. Holding the point is the integral's job: R2966, R2973 hold 0.2 mm.

**The dip's price, today's planner to rod with `int_k_xy` 0 (dip change, mm; T6, s):**
| Today to rod | Dip change | T6 |
|---|---|---|
| R2965, R2972 to R3463, R3465 | +5.5 to +9.0 | 5.4, 13.4 to 5.5, 6.2 |
| R3232 to R3467, R3426 | +1.9, +9.5 | 8.2 to 6.0, 7.2 |
| R2966, R2973 to R3464, R3466 | +0.1 to +3.1 | 4.4-4.5 to 8.3 |
| R3233 to R3468 | +11.1 | 9.7 to 8.8 |

The chain shows no dip price. R3421 was not re-flown: unanswered.

**What a flip would break, beyond `sim.yaml`:** the baselines type no switch; `src/bringup/config/sim.yaml:68` guards only the rod, so the network and SIL modes would lose the integral with `int_k_xy` 0; the solve at n = 8 rises from 243-248 to 338-392 ms p95, and X3b's bench has none of this.

**The thesis may say, once flipped:** simulated hinged rods, rider 3, hover: the T's numbers above with the dip's price. It may not say anything of real cables, of rider 0, or of the rod alone on the attach.

**Still to fly before a flip, queued 2 Nov 23:39 (`~/mdc_overnight/lab_1102b.sh`, `lab_1102c.sh`):** a valid `um_base`; the three chain arms with one detector, the marker check (`s8c_base_tip`, `s8c_noxy_tip`, `s8c_rodl_tip`); `x1_h4rodl_ours` again; soft detach again (R3433, R3434, R3438's configs); R3437's two one-variable arms (`x4_relay_short_v3_norod`, `x4_relay_short_v3_xy`).

**A finding beyond the rod:** on today's tree, without the rod, the stretch check lost a release in 2 of 2 runs of the 8-quad chain. On real flights the marker check comes first (`tip_detect_m` 0.04). The simulated chains without the marker check are at risk until the flip or the marker runs.

REVIEWER: pass with corrections: T, chain hold on repeats; no-rod chains, `um_base` void; attach not isolated; h4, soft detach unrepeated.

## 11. The reviewer's list, flown (2-3 Nov, R3475-R3488; exploratory, judged after the run; not yet reviewed)
One tree (3 Nov's two batches differ only in configs). One run each unless shown.

**The chain, 8 -> 4 -> 8, all four arms on one detector (marker check) except the last row:**
| Arm | Back at 8, mm, deg | With the resize, mm, deg | Settled at 5, 6, 7, 8 (deg) | Fold-in peaks (deg) | Solve p95 (ms) |
|---|---|---|---|---|---|
| Today, R3476 | 6.4, 0.99 | 19.0, 1.63 | 1.27, 0.10, 1.17, 0.03 | 1.25-1.44 | 233 |
| Integral off, R3477 | 1.1, 0.07 | 6.1, 0.57 | 0.94, 0.04, 0.97, 0.02 | 1.03-1.05 | 250 |
| Rod, R3478 | 0.5, 0.07 | 0.9, 0.15 | 0.04, 0.05, 0.03, 0.03 | 0.15-0.56 | 349 |
| Both, R3472 / R3473 (stretch check, no release lost) | 0.37, 0.07 / 0.50, 0.07 | - | 0.06, 0.04, 0.03, 0.03 / 0.06, 0.05, 0.03, 0.02 | 0.18-0.56 / 0.20-0.54 | 329 / 375 |

R3476 reproduces R2766, R2767 (6.36 mm, 0.95-0.99 deg). **Back at 8, either change is enough: the integral off alone gives 1.1 mm, not the 40 mm I reported (R3470, a lost release). What only the rod does is level the odd stages, 0.03-0.04 deg against about 1 deg.**

**The attach, four arms on one newcomer filter (`ema`) and `weld_vel_filter_s` 0.1:**
| Arm | Tilt after the weld, mean, p95 (deg) | Fold-in peak (deg) | Catch offset (cm) | Same arm, raw gate |
|---|---|---|---|---|
| Today, R3485 | 1.26, 2.21 | 1.68 | 2.4 | no weld, R3458, R3475 |
| Integral off, R3486 | 2.20, 2.84 | 2.77 | 1.1 | 2.21, R3459 |
| Rod, R3487 | 1.70, 2.58 | 1.25 | 1.6 | 0.95, R3460 |
| Both, R3488 | 0.76, 1.34 | 1.48 | 1.7 | 0.39, 0.32, R3461, R3462 (newcomer on `slope`) |

The two together are lowest and the integral off alone worst. The rod alone is not clearly better than today's: its two runs span 0.95-1.70 deg. **The weld gate:** `um_base` timed out in 2 of 2 on the raw speed (0.05 m/s, 0.15 s, no filter), and all four arms welded with the 0.1 s filter, earlier in three of them. The simulation's gate only; the real attach is confirmed by the co-motion check. The filter's default (0, `common.yaml`) is Wesley's word.

**The repeats.**
| Case | Run | Against | Reading |
|---|---|---|---|
| Even release, the rod at 5.26 | R3479: rise 6.44, abs 6.49, dip 30.7, T6 5.5, after 6 s 0.24 | R3421: 7.34, 7.39, 32.3, 3.0, 0.23. Today R2934: 7.16, 5.62, 33.7, 3.9, 0.45 | Rise is within scatter, the abs peak stays higher |
| Soft detach, then rider | R3480: 22.47, from +6 s peak 3.34, mean 0.37 | R3433: 23.83, 3.16, 0.37. R2840, R2842 (older tree): 24.9, 26.6, 1.14, 1.02, 0.69, 0.68 | Repeats. Late peak 2 deg higher, mean lower |
| Soft detach, rejoin | R3481: 16.76, 3.73, 0.36, fold-in 2.20 | R3434: 13.35, 3.27, 0.33, 2.30. R2578, R2579 (older tree): 3.07, 2.29, 0.85, 0.80 | Repeats, detach peak scatters 3 deg |
| Full chain, soft | R3482: 18.41, 3.82, 0.99 | R3438: 19.58, 4.09, 0.91. R3293, R3296: 28.42-28.69, 3.35-3.38, 1.26-1.28 | Repeats, peak about 10 deg lower |

**The relay's arms, one variable from the adopted setting, 3 swaps each:**
| Arm | Event tilt max (deg) | Tilt RMS, p95 (deg) | Path first, last (mm) | Solve p95 (ms) |
|---|---|---|---|---|
| Adopted, rod, integral off, R3454 | 1.45 | 0.67, 1.06 | 12.9, 12.0 | - |
| Rod out, R3483 | 2.21 | 1.03, 1.61 | 16.3, 15.8 | 248 |
| Integral on, R3484 | 1.64 | 0.81, 1.31 | 18.1, 19.9 | 394 |

**The rod's cost in solve time:** n = 8 p95 233-250 to 329-394 ms on the lab PC. X3b's idle bench (R8a) has no rod and `tools/solve_scaling.py` cannot fly one; a real flight has none either, so R8a still describes the real planner, and the simulated runs are not real-time anyway.

**Before a flip (Wesley's word):** `sim.yaml:39` on; `int_k_xy` 0 scoped so the network and SIL modes keep theirs (`sim.yaml:68` guards only the rod); the baselines pick the switches up from `sim.yaml`; CLAUDE.md's prebuild line; the reviewer on the final table.

## Critic

Challenge of 31 Oct ~18:05 (the critic, read-only; nothing run, sums by hand). Names resolve under the main checkout.

**1. Tried?** Yes: R2258 (CoM 0.147: 4.4 deg, release undetected), R3000 (5.19 deg). The same lean was carded and Wesley dropped it (`docs/experiments/2026-10-22_chain_step2a_vector_split.md`, decisions.md:363); neither card nor audit cites it.

**2. Simplest explanation.** The defect is `int_k_xy` 5.26 on a T: off, 0.60 deg (R3202). Floor and integral's point of action stay on real flights (audit 1, 4).

**3. Falsifier.** Plan t differing from plant T_D. No bar reads it; run 1 fails only on 1 deg or 10 mm.

**4. If it works.** Simulated "ours" stops being the real flights' planner. U1 passed on dip alone (108.7-109.9 against 100.2 + 5); R3000's 115.3 would invert it, with no rod-level no-resize arm.

**5. Asked?** Yes (decisions.md:751), but not "`int_k_xy` 0 everywhere" (runs 6, 7, 12).

**A. Code.** Other readers are safe: `TIP_RIDER_N` 1.0 (`dissipative_node.py:2188`), carrier map, geometric gates, measured pull (`tracker_node.py:1878`), network. References right at all four steps (`planner_node.py:1867-1869`, `dissipative_node.py:3319-3368`). Untested: the lean's unblended 28 mm step at release; mid-blend sideways force. Newcomer's start is right: otherwise the model hangs the rod on the plate (-0.64 N). Soft detach is unguarded and targets the wrong pull: the row mix is harmless from the square (exact), but leaver and rider at 0.5 N sit under neutral 0.79, which pulls plates down 0.25 N and clamps the measured pull. X2-full (`x2_full_fig8_soft.yaml:58`) and X4 fly it; `real_mode.py:185-189` does not refuse. Off: same operations as 5a7ccd0 (worktree `geometry.py:211-240`).

**B. Physics.** `rod_on_load` reduces to F_out = (F_z + w) cot e; SDF CoM 0.2667 m. 0.8 N is real: flown 0.784, 0.796 at F_z near 0 (R3202, R3201). That puts measured z pull on the clamp's kink (`tracker_node.py:1688`), margin only typed offset error: drone 3 (0.187) 0.07 N, runs 8-9's drone 1 (0.185) none, drone 0 (0.181) binds, unsolved: fly it once (`RELEASE 2`). Inertia is small (about 0.05 N), so R3000's dip and -13.3 mm height stay unexplained; no z column.

**C. Bars.** Roll near 0 is self-consistency: references built from the solver's own statics. Add per-carrier |t - T_D|, height (`off_point_mm` is horizontal), dip, peak. Only elevation 70 discriminates; add 50 (R3236 -2.86) and typed rod +10%, -10% (predicted +1.05, -0.97). Runs 9, 11 lack a still-moving bar; 11 lands at +20 s, R3238 climbed 45 s. R3239 (46 against 60-70) contradicts "every prediction held".

**D. Thesis.** Fair claim: in simulation the integral learned a hinged rod's weight (R3235, R3236, R3400); modelling it removes that if run 1 holds. Nothing about real cables, and the switch moves away from them: our one T agreement (2.6 against 2.3, decisions.md:363) was at `int_k_xy` 0. It would fly the fallback in claim cells; runs 1, 3 and elevation 50 as one table. Strongest against: it is a flown cancellation (+4.58, -2.86), 64 mm off, typed after seeing results.

CRITIC: rewrite the card, because its bars test a level ring, not a plan agreeing with the plant.

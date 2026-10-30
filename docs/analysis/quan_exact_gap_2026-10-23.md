# Quan et al. exactly: gap analysis and build plan (23 Oct 2026)

Wesley (decisions 2026-10-23, "Quan et al. exactly", ~21:10): the comparison arm should be the method as published, not
our port; the world's geometry may change to match their model. This note lists every difference between the paper and
what the paper-form arm flies (`configs/twin/c_net_paperfinal.yaml`), proposes the world, orders the build, and reads why
R2744-R2749 lost the ring. Nothing was built or flown for it. Paper: `docs/papers/quan2025_self_organizing_aerial_swarm_2509.03563.pdf`
(page numbers are the PDF's printed pages).

## 0. What "as published" can mean (needs Wesley's word)
| Source | What it is | What our port follows |
|---|---|---|
| The paper's text, Eq. 3 and 7-10 (p.24-25) | A per-robot force law. The virtual node is an algebraic point (Eq. 3); springs and friction act on the robot; Eq. 10 gives an acceleration command; Fig. 2 (p.9) sends a velocity command `v_d` to the autopilot | Not this |
| The authors' code, github.com/RflyBUAA/Cooperative-Transportation (`code/Normal/init.m`, `caldata.m`, `decentralized.slx`; read 23 Oct, the Simulink model could not be read as text) | A node network: integrated nodes, all-pairs adjacency W, springs K with rest L0 (2.02 m between robot nodes), damping c 10, stiffness k 20, 10 robots; robot reference projected from the node (`caldata.m`) | This (dissipative_network.py:1-49) |
| Supplementary Materials S1-S8 (modelling S2, proofs S4/S7, benchmarks S5/S6/S8) | Cited by the paper; not in the repo and not in the arXiv PDF (33 pages) | - |

Recommendation: "as published" = the text's Eq. 3 and 7-10, with the code used only to settle what the text leaves open.
The text leaves open (each one is a choice we must state in the thesis, row numbers from §1):
`a_expected` (row 9), what `x_0 = q_0` is (row 7), how the acceleration becomes the velocity command (row 12), `h_c` and
every gain (rows 5, 17), how `q̇_i` is obtained (row 3), whether `k_i` is scaled by `k_d,i` (row 4), and what the autopilot
does with gravity and the cable pull (row 11).

## 1. Every difference, paper vs the paper-form arm (`c_net_tune_paperfinal_*_L100`, params `c_net_paperfinal.yaml`)
Closing it: **C** config only, **S** an existing switch, **W** world change, **N** new code in the flight loop (critic),
**T** new tooling (not flight loop). Match: **exact**, **approx** (the closest our twin can do), **no** (cannot).

### 1a. The method
| # | Item | Paper (where) | We fly (file:line) | Close | Match |
|---|---|---|---|---|---|
| 1 | What the method outputs | A force `u_i` on each robot (Eq. 7, p.25) turned into an acceleration command (Eq. 10) and sent as a velocity command `v_d` (Fig. 2, p.9). No position reference | A position, velocity and thrust reference per drone, from a node network, flown by a position-servo velocity loop: `p_ref` = node projected on the cable sphere, `v_ref` = node velocity, `a_ff` = g + modelled cable (dissipative_network.py:709-731) | N: a new law (§4 step 3) | exact (law), approx (interface, row 12) |
| 2 | Virtual node | Algebraic: `q_i = x_i + (h_c-h_i)/(h_i-h_{n+1}) (x_i - x_{n+1})`, the point on the load-robot line at altitude `h_c` (Eq. 3, p.24) | Integrated point mass (`diss_node_mass` 0.5, 10 substeps at 10 Hz), seeded at the pivots at the hand-over (dissipative_network.py:490-559; dissipative_node.py:3875-3880) | N | exact |
| 3 | Node velocity `q̇_i` | Used in Eq. 7-9; not defined. Differentiating Eq. 3 with the measured `ẋ_i`, `ẋ_{n+1}` is the only reading that uses the paper's own inputs (Fig. 2: `p_j, v_j, p_L, v_L, p_self, v_self`) | The node state's own velocity `qd` | N | exact once chosen (ambiguity) |
| 4 | Node-robot spring and damper `k_i`, `c_i` | `k_i (1 - l_i(0)/l_i)(q_i - x_i)` and `c_i/l_i² (q_i-x_i)^T(q̇_i-ẋ_i)(q_i-x_i)`, 3-D, along the cable line; printed outside the `k_d,i` factor (Eq. 7) | None. Nearest: the node-payload spring `k_pay` (rest cable_len) and the anchor spring `k_anchor` that pins the node on a cone (dissipative_network.py:499, 516-529) | N | exact once the `k_d,i` bracket is read |
| 5 | Node-node springs | `-k_d,i k_ij Σ_{j=0..n} w_ij ((1 - l_ij(0)/l_ij)(q_i - q_j))_hor`: horizontal only, all perceived robots plus the formation centre `j=0`, rest = the value at the start (p.25) | 3-D springs to the two ring neighbours only, rest = the slot chord (dissipative_network.py:544-552); no spring to a formation centre | N | exact |
| 6 | Node-node damping | `-k_d,i c_ij Σ (w_ij/l_ij² (q_i-q_j)^T(q̇_i-q̇_j)(q_i-q_j))_hor`: along each edge only, horizontal | `-c (qd_i - qd_j)`, 3-D, all directions, ring neighbours only (dissipative_network.py:552) | N | exact |
| 7 | Friction to the formation centre (Eq. 9) | `-k_d,i f_c (q̇_i - ẋ_0)_hor`, `x_0 = q_0` "the formation center" (p.25): either the robots' centroid or the commanded path point (not said) | `net_damp_relative`: `-2c (qd_i - v_d)`, 3-D, no `k_d,i`, `v_d` the target's commanded velocity open loop (dissipative_network.py:537; dissipative_node.py:4096-4097). `diss_c` 3 is shared with row 6 | S (as is) / N (exact form) | approx now; exact with N |
| 8 | Azimuth pin and anchor | None. Rotation of the swarm is stopped by row 7's friction (p.25: "to prevent the rotation motion") | `k_slot` 18 springs every node to an even slot phased to the ring's measured attitude; anchor node above the target pins the cone (dissipative_network.py:441-488, 423) | N (they go with the new law) | exact |
| 9 | `a_expected,i` (Eq. 10) | In Eq. 10, not defined. Reading: the path's acceleration | `net_traj_lean`: the path's acceleration in `a_ff` **and** the whole cone tilted onto g - a, which is not in the paper (dissipative_network.py:347-378, 421-423, 681-683, 730; dissipative_node.py:4058-4064). A measured negative on the old plant (reference.md §6) | N (split the feedforward from the lean) or inherent in the new law | exact once read |
| 10 | Use of the load's attitude | None: the load is a point (Eq. 1 carries its position only; p.23 "connection points ... at the center of mass"); the fisheye gives position (p.22) | The formation is built in the ring's full measured rotation (`_frame_rot` returns `quat_to_rot_np` unless `diss_balanced_tensions`, which is false in C: run params): attach points, slots and anchor turn and tilt with the ring (dissipative_network.py:317-344, 425-427). The card's "yaw-only" holds only with balanced tensions | N (inherent in the new law; or a port switch, §4 S3) | exact |
| 11 | Cable tension `f_i` and gravity | Eq. 7 subtracts `f_i` and `m_i g`; Eq. 10 adds `f_i/m_i` and `g` back, so as printed they cancel: `a_cmd = a_expected + (h_1 + h_2)/m_i`. The autopilot is left to hold the robot's weight and the pull. `f_i` is plotted from flights (Fig. 3C), its source not stated | A modelled pull `m_L g/(n sin φ)` along the node's cable direction in `a_ff`, gated by rod tautness (dissipative_network.py:722-730); the tracker's measured pull (`cable_source mocap`) is zeroed in the velocity path (tracker_node.py:1389) | C (literal: none) or N (measured `f_i` into the loop's feedforward, §4 S4) | exact (literal reading); approx otherwise |
| 12 | The autopilot interface | Velocity command `v_d` to an autopilot with its own IMU loop (Fig. 2); its type, gains and integrators not given. How `a_cmd` becomes `v_d` not given | Our velocity loop stands in: `v_sp = v_ref + kp_pos (p_ref - p)`, `a_sp = a_ff + kv e_v`, 30 deg tilt cap, `vel_ki` 0 (velocity_loop.py:173, 226, 244; tracker_node.py:1359-1391) | C: `vel_kp_pos 0` makes it a pure velocity-command loop (`v_sp = v_ref`); the planner then publishes `v_ref = v_d` (`v_d` integrated from `a_cmd`) and `a_ff = g + a_cmd` | no (their autopilot is hardware); approx |
| 13 | Integrators | None in the law. A commercial velocity autopilot normally has an I-term | `diss_ki_load` 0.5 (a common-mode load-height trim, not in the paper; dissipative_network.py:409-417, 730), `vel_ki` 0 | C: `diss_ki_load 0`; `vel_ki` is the stand-in autopilot's choice (state it) | exact (law); approx (autopilot) |
| 14 | Rates and timing | Not given | Network 10 Hz in the central planner (planner_node.py:64), velocity loop 50 Hz (tracker_node.py:87), `ema` velocity filter | C/N: the new law is cheap and could run at the tracker rate; state it | approx |
| 15 | Decentralisation and sensing | Each robot computes its own law from its own sensors: GNSS/DGPS self, LiDAR neighbours (2 m filter, EKF), fisheye load position (KF) (p.21-22) | One planner computes every drone from mocap | N (the new law uses only robot i's local inputs, so the central computation is equivalent); sensing noise not modelled | approx (better sensing than theirs) |
| 16 | Perception range `r`, `w_ij` (Eq. 5) | `w_ij = 1` within range `r` | Fixed ring-neighbour graph | C in the new law: `r` 2 m (their LiDAR filter, p.22) gives the complete graph at 1 m | exact |
| 17 | Gains and design values | `k_ij, c_ij, k_i, c_i, f_c, h_c, l_ij(0), l_i(0)` not published in the text; the code's `init.m` uses k 20, c 10, rest 2.02 m for its own (different) model | The port's search-best set (diss_c 3, k_pay = k_anchor 80, k_ring 40, k_slot 18, ki_load 0.5; loop kp 1.5, kv 3, k_att 16) | T + C: rest lengths from the hand-over; the rest searched offline then confirmed in the twin (the card's §3.3 procedure) | no (unpublished); tuned |
| 18 | Hand-over and take-off | Robots take off and lift; how is not described (p.8) | Our OCP creeps and lifts; the network takes over 1.5 s after the top, nodes seeded at the pivots, datum shifted onto the measured ring (dissipative_node.py:1114-1119, 1478-1498, 3862-3907) | none: keep our lift as the common prefix; `l_ij(0)`, `l_i(0)` taken at the hand-over | approx (unspecified) |
| 19 | Pre-path hold | Not described | `net_hold_carry` holds 5 s so both arms start the path together (dissipative_node.py:4160-4168) | keep (scoring plumbing) | n/a |
| 20 | Reference at the drone centre | Robot connection at its centre of mass (p.23) | `net_twin_geometry` moves each reference from the 4 cm pivot to the drone centre (dissipative_node.py:4170-4176) | W (pivot at the CoM, §3) makes it a no-op | exact with W |
| 21 | Horizon preview | n/a | off in the network mode (sim.yaml:63) | none | n/a |

### 1b. The plant and the setup
| # | Item | Paper (where) | Our twin | Close | Match |
|---|---|---|---|---|---|
| 22 | Load model | A point (Eq. 1: `x_{n+1}`, position only) | The rig ring: 0.86 kg, Izz 0.0443, rods on four rim plates at r 0.225 | W: the hub (`--hub-r 0.001`, built) | approx (1 mm, not 0: code needs an azimuth, card) |
| 23 | Where the cables meet the load | Model: the CoM (p.23). Hardware: S-hook carabiners (p.21), so a real payload hangs below the junction | Rim, 2 cm above the CoG (`attach_z` 0.02); hub world: 1 mm hub on that plane | W: keep `attach_z` 0.02 (their hook) or 0 (their model) | exact either way; Wesley's pick |
| 24 | Where the cables meet the robot | The CoM (p.23) | Ball 4 cm below the drone centre (`pivot_offset_z` -0.04), the lever that turns any mismatch into torque (T0010) | W + C: `--pivot-dz 0`, `pivot_offset_z 0` | exact |
| 25 | Cable | Nylon braided cord, little stretch (p.21), slack possible (Table 1); modelled taut (p.23) | Rigid rods, ball at both ends: can push | W + T: a segmented-chain rod (generator code) | approx; not first |
| 26 | Cable length | Hardware 3-4.5 m (p.11), simulation benchmark mean 2.02 m (Fig. 5 caption, p.14) | 1.0 m | W (`--cable-len 2.0`) | exact possible; keep 1 m for the claim (§3) |
| 27 | Number of robots | 4, 5 and 6 on hardware (p.8); 10 in the code | 4 | none | exact (their 4-robot flights) |
| 28 | Spacing | Not given; LiDAR neighbours within 2 m (p.22) | Quads 0.50 m from the axis on the hub (0.71 m apart), 0.73 m on the rim (1.03 m) | none | within their range |
| 29 | Load per robot | 3-5 kg on 4-5 robots, each "2 kg stable payload capacity" (p.21): 0.75-1 kg each, 38-50 % of capacity; robot mass not given | 0.215 kg per 0.55 kg quad; hover throttle 0.59 of 0.8 | none | no (their robot mass unknown) |
| 30 | Environment and scale | Outdoors, wind to Beaufort 4, 1-1.6 m/s, 34 m paths; tracking errors of 1-2 m median (Fig. 5) | Indoor twin, no wind, 0.9 m line at 0.2/0.4 m/s, mm-cm errors | none | no; state the scale difference |

## 2. Can it be exact? (summary)
- **Exact:** the law itself (rows 2-10, 16), the load as a point at the centre (22-24 within 1 mm), the robot
  connection at its centre, 4 robots.
- **Exact after a stated reading:** rows 3, 4, 7, 9, 11. These are gaps in the text; each reading goes in the card.
- **Approx:** the autopilot (12, 13): theirs is hardware, ours a velocity loop set to take a velocity command; rates (14);
  sensing (15, ours better); cable compliance (25).
- **No:** the gains (17, unpublished, so tuned by the card's procedure), their mass ratio (29), outdoor scale (30), and
  their take-off (18, not described).

## 3. Proposed world for the exact arm
| Option | Load end | Robot end | Cable | What it matches |
|---|---|---|---|---|
| A (built, flown R2725-R2763) | 1 mm hub, 2 cm above the CoG | 4 cm below the drone | 1 m rigid | the load as a point; not the robot end |
| **B (proposed)** | 1 mm hub, 2 cm above the CoG (their S-hook) | **at the drone centre** | 1 m rigid | the model's robot connection and the hook's stable hang |
| C | 1 mm hub at the CoG (`--attach-z 0`) | at the drone centre | 1 m rigid | the model to the letter; the ring's tilt is then free (no restoring moment), so the 30 deg tilt loss rule means nothing and must be dropped for both arms |
| D (neighbour) | as B | as B | 2 m | their simulation's cable length (needs a 2 m planner cache) |

Keep 4 quads and 1 m: both are the thesis task and the planner arm's; their own 4-robot flights exist.
Option B, generator (do not run yet; the world and attach geometry are Wesley's word):
1. `tools/make_carry_worlds.py` `WORLDS` (line ~107): add
   `'four_rigid_ground_rig_hubc_L100.sdf': ['--n', '4', '--hub-r', '0.001']`, with
   `WORLD_OVERRIDES[...] += ['--pivot-dz', '0.0']` (it comes after `RIG_ARGS`' 0.04) and the `--tether-pose-hz 60` list;
   `world_expect()` must take `pivot_dz` from `--pivot-dz` like `attach_z` (now fixed by `EXPECT`), and
   `tools/test/test_rig_worlds.py` gets the world. For option C also `--attach-z 0.0`.
2. `python3 tools/make_carry_worlds.py`, then `python3 tools/make_carry_worlds.py --check` (it rewrites every world;
   the others must come out byte-identical).
3. Configs: as `c_net_hub_*_L100` with `world: four_rigid_ground_rig_hubc_L100.sdf`, `attach_radius 0.001`,
   `pivot_offset_z 0.0` (C: `attach_z 0.0`, `payload_rest_z` checked), in both arms; `prebuild_planner.py --from-config`
   for the planner arm (the pivot is in the OCP's kinematics).

## 4. Build plan, in order
**Step 0, config only (fly first, exploratory, one run each, laptop or lab):**

| Run | Config change from | One variable | Falsifier (one line) |
|---|---|---|---|
| E1 | `c_net_tune_paperfinal_hover_L100` | world A + `attach_radius 0.001` | Lost on the hub: the rim's yaw mode is not what kills the paper-form gains (§5 refuted) |
| E2 | same, rim | `diss_c` 3 -> 12 | Still lost: the node damping is not the knob |
| E3 | same, rim | `vel_k_att` 16 -> 8 | Still lost: the attitude gain is not the knob |
| E4 | `c_net_tune_paperfinal_linex_v400_L100` | world A | Lost on the hub: Eq. 9/10 terms or the gains fail even without the yaw mode |
| E5 | none (re-score) | R2685/R2686 window 60 -> 64 s | (no run) R2685's ring passed 30 deg at hand-over + 63.9 s, inside its flight, 3.9 s after the scored window: say so beside C's port rows |

**Step 1, config only (no code):** `diss_ki_load 0` in a paper params file (row 13); `vel_kp_pos 0` reserved for step 3.

**Step 2, switches in the port (default off; each a flight-loop change: critic, one card):**
| Switch | Does | Touches |
|---|---|---|
| S1 `net_accel_ff` | Eq. 10's `a_expected` in `a_ff` without leaning the cone (splits `net_traj_lean`) | dissipative_node.py:4058-4064; dissipative_network.py:421-423, 681-683, 730 |
| S2 `net_damp_paper` | row 7 as printed: horizontal, scaled by `k_d,i`, against `ẋ_0`; row 6 along the edge | dissipative_network.py:537, 552 |
| S3 `net_frame_fixed` | the formation frame frozen at the hand-over (no load attitude, row 10) | dissipative_network.py:338-344, 394, 425-427 |
| S4 `vel_cable_ff` | the measured rod pull (`cable_source mocap`) into the velocity loop's feedforward (row 11, if the measured-`f_i` reading is chosen) | tracker_node.py:1359-1391 (1389) |

These make the port closer; they do not make it the paper's law. Skip step 2 if Wesley wants only the exact arm.

**Step 3, the exact law (new control law in the flight loop: critic before any run):**
- New pure-numpy module `src/dissipative_planner/dissipative_planner/quan_law.py`: Eq. 3 nodes and their analytic
  derivative, Eq. 7-9 with `w_ij` from range `r`, Eq. 10 `a_cmd`, and `v_d` integrated from `a_cmd` (seeded at the
  measured velocity at the hand-over). Unit tests: Eq. 3 geometry, the equilibrium is force-free at the hand-over
  rest lengths, Eq. 10's cancellation.
- Switch `net_law: port|quan` (default `port`) in `dissipative_node.py`: `_enter_network_phase` (3862-3907) records
  `l_ij(0)`, `l_i(0)`, `h_c`; `_network_plan` (4039-4157) calls the law per drone and publishes `p_ref` = measured
  position (logging), `v_ref = v_d`, `a_ff = g + a_cmd` (+ S4 if chosen). New params `quan_k_ij, quan_c_ij, quan_k_i,
  quan_c_i, quan_fc, quan_hc_m, quan_range_m` (common.yaml `network:` block, 117-138).
- Params file `configs/twin/c_net_quan.yaml`: `net_law quan`, `vel_kp_pos 0`, `diss_ki_load 0`, the readings of rows
  3/4/7/9/11/12 stated in its header.
- Tooling (not flight loop): `tools/network_tune.py` and `verify_dissipative.py` call the new law, so the gains can be
  searched offline as the card's §3.3 does; the harness lacks the pivot lever, which world B removes.

**Step 4, exploratory runs on the new law (no card; one registry row each with its falsifier):**
| Run | World | Case | Falsifier |
|---|---|---|---|
| Q1 | B | hover | No hand-over, or ring tilt > 30 deg / a quad > 60 deg before LAND: the law as read does not hold the load in the twin |
| Q2 | B | line 0.2 | Lost, or the ring ends > 0.2 m from the path's ends: the law cannot carry along a path at this gain set |
| Q3 | B | line 0.4 | As Q2 |
| Q4 | rim (C's world) | hover | Holds on the rim with a growing ring yaw: then the point-load assumption, not our port, is what the rim breaks |
| Q5 | B, 3 quads | hover | Lost with 3: the law's own neighbour (fewer robots) fails |

Then a card with frozen bars for the claim (the hub cells' G/P/A pattern), the reviewer on the table.

## 5. Why the paper's form loses the ring (HYPOTHESIS)
**Reading:** the ring's torsional swing on its four rim rods (about 1 Hz) is pumped by a formation that turns with the
ring's measured attitude. The paper-form gain set (node damping `diss_c` 3, `vel_kp_pos` 1.5, `vel_k_att` 16) makes the
quads follow the ring's twist more and damps it less, so the swing grows about 2-4x as fast as at the port's gains, and
the ring capsizes 20-31 s after the hand-over. Eq. 9 and Eq. 10 are not what kills it: in hover they are inactive.

Evidence (computed from each run's `logs/run.csv`, ring yaw from the quaternion, envelope fit from hand-over + 4 s until
the yaw first passes 20 deg):

| Runs | Arm, world | Swing period (s) | Yaw growth (%/s) | Ring tilt > 30 deg (s after hand-over) |
|---|---|---|---|---|
| R2744, R2745 | paper form, rim, hover | 1.02 | 21.5, 36.3 | 31.3, 27.2 |
| R2746, R2747 | paper form, rim, line 0.2 | 1.02-1.03 | 43.7, 32.6 | 20.9, 27.3 |
| R2748, R2749 | paper form, rim, line 0.4 | 1.03-1.05 | 33.0, 20.2 | 23.4, 30.9 |
| R2446, R2447 | port as first ported (c 6, kp 2, k_att 8), rim | 1.02 | 44.4, 44.6 | 25.7, 24.0 |
| R2685, R2686 | port best (c 12, kp 1, k_att 8), rim, hover | 1.07-1.08 | 12.1, 11.0 | 63.9 (after the 60 s window), never |
| R2687-R2690 | port best, rim, lines | 1.07 | 9.6-11.1 | never |
| R2726, R2762 | port best / as first ported, hub, hover | - | ring yaw stays at 0.1 deg | never |

For:
- In hover, `net_damp_relative` and `net_traj_lean` contribute nothing (the target's velocity and acceleration are
  zero: dissipative_node.py:4058-4064, 4096-4097), so R2744/R2745 fly the port's law at the paper-form gains, and they
  lose the ring like the lines do. The loss follows the gains, not the form.
- The same ~1 Hz swing appears in every rim network run; its growth rate orders with the gains (as first ported and
  paper form 20-45 %/s, port best 10-12 %/s). Ring Izz 0.0443 kg m² at 1.0 Hz is a yaw stiffness of ~1.75 N m/rad;
  four 2.4 N rods at r 0.225 to quads ~0.69 m out give ~1.5 N m/rad: the ring's own torsional pendulum.
- The quads follow the swing harder at the paper-form gains: at a ring-yaw envelope of ~50-57 deg, quad 0's azimuth
  about the ring swings 19-22 deg (R2744, R2745) against 5 deg at the port's gains (R2685).
- On the hub (no yaw stiffness, nothing to follow) the swing is gone at both gain sets (R2726, R2762: 0.1 deg).
- Our planner, which holds the formation's yaw datum, shows no swing on the rim (R2733: 0.0 deg).

Against / open:
- Offline (card §3.4) a formation held at the hand-over's yaw still grew the swing (x1.1-1.3 a second): following the
  ring is not the whole drive. The delay chain (10 Hz network, `ema` filter, the 50 Hz loop) may be.
- The port's best gains also grow it (R2685 passed 30 deg at hand-over + 63.9 s, 3.9 s after its scored window, before
  LAND): the rim mode is unstable for every gain set flown, only slower.
- Not separated: which of `diss_c` 3, `vel_k_att` 16 and `vel_kp_pos` 1.5 matters (E2, E3 test it), and whether the
  lines' Eq. 9/10 terms speed it up (the line losses come at 20.9-30.9 s against 27.2-31.3 s in hover: no clear sign).
- Every network run sits its quads 5-6 cm off their references in hover (`track_err` 0.05-0.06 m, rods at ~63 deg
  against the 60 deg cone; our planner 2 mm): no per-quad integrator and a modelled pull. Not shown to drive the swing.

What would refute it: E1 lost (the swing is not the cause), or E2/E3 both still lost (neither knob governs it).

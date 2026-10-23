# Card: Quan et al. exactly, the paper's per-robot law against our planner on world C, in the twin (23 Oct 2026)

Owner Wesley. Drafted Fri 23 Oct ~22:00 in a worktree off week7 63b504f; **rewritten 23:30 after its critic** (below,
"## Critic", and "## Response") and Wesley's decision on it (decisions 2026-10-23 "Quan exact law after its critic",
~23:20). Nothing flown. **Status: DRAFT; every bar PROPOSED; new control law in the flight loop: the critic has seen
it once; Wesley freezes the bars before the first run.** `net_law` defaults to `port`: every existing run flies as
before.

## 1. Question
Flown as its text prints it (Eq. 3, 5, 7-10, with the supplementary's S1-S4 where the main text leaves a point open; not
our port of the authors' node-network code), does Quan et al.'s per-robot law carry the 1 m ring on the model it was
derived for (every cable meeting at the load's centre of mass and at each robot's centre), and how closely against our
planner on the same world, in hover and on the 0.9 m line at 0.2 and 0.4 m/s? Wesley: "we want to be able to get their
results".

## 2. Hypothesis
- **H1:** with the readings of §3.1 and the gains of §3.3, the law carries the ring within world C's band (§5) in all
  three cells.
- **H2:** our planner follows the path at least as closely as the law in every cell (C's claim rule).
- **Expected, offline (§3.4):** the hover held ~124 mm off its target (a standing offset: R7b has no integral action);
  the lines with the ring 44 mm off the line sideways but running 66-145 mm past the ends, 0.3-0.4 m from the path at
  worst at 0.4 m/s, so the 0.4 m/s cell is at the band's edge. The quads calm (tilt peak 9-15 deg, throttle 0.6).
- **Tried before:** the first build (R7, `v_d` accumulating `a_cmd`) held nothing offline; the paper's damping form of
  our port lost the ring on today's top-face hub (R2780 hover rocked: tilt peak 24.5 deg, yaw 26.5 deg, quad 37 deg;
  R2781 line 0.4 lost at 70.4 s, though with ~0 lag: -0.007 s, headline 62.7 mm against the port's 102 mm).

## 3. The one variable per cell: the method after the hand-over
Everything before the hand-over is our planner in every arm (creep, our OCP's lift, the 5 s hold carried across it,
`net_hold_carry`). The arms differ in the whole stack after it (C's pattern: method against method).

| Arm | Mode | After the lift | Params file | Configs (configs/experiments/) |
|---|---|---|---|---|
| Our planner | `dissipative` | our OCP and the MPC trackers | `configs/twin/c_compare_hubcom.yaml` | `c_pl_hubcom_{hover,linex_v200,linex_v400}_L100` |
| Quan et al. as printed | `network`, `net_law quan` | the law (quan_law.py) per quad on the trackers' velocity loop as a velocity-and-acceleration autopilot (`vel_kp_pos 0`, `vel_ki 0`) | `configs/twin/c_net_quan.yaml` | `c_net_quan_hubcom_{hover,linex_v200,linex_v400}_L100` |
| Neighbour: our port on world C | `network`, `net_law port` | the port at C's search-best gains | `configs/twin/c_net_portfinal_hubcom.yaml` | `c_net_port_hubcom_hover_L100` (once) |

- A world-C config differs from its hub config (`c_*_hub_*_L100`) in `world`, `attach_z` 0.0, `pivot_offset_z` 0.0
  and its params file. The rest is C's: azimuths 0/90/180/270, 1 m rods, the 0.86 kg ring, the twin plant (rig map,
  offsets 0.181/0.185/0.187/0.187, 23.8 V), the `ema` filter on every carrier, target 1.0 m, windows and events.
- **World C's tilt rules are off in every arm** (Wesley 23 Oct): the trackers' ring-tilt drop (`drop_tilt_deg` 180,
  never trips; `drop_on_loss` stays on for the loss detector) and their 60 deg ring envelope (`payload_tilt_envelope`
  false: a new twin-only switch in sim.yaml, default true, which sends `max_payload_tilt_deg` 180 to the trackers;
  the rig's profile has no such knob and refuses it).
- `run_experiment.py --check`: ok for all seven (C's params.py inertia warning), with the params paths pointed at the
  worktree's files; the configs name the main tree's `configs/twin/` files, which exist once merged.
- Our planner's arm: the pivot is in the OCP's kinematics, so before the first batch on a host
  `python3 tools/prebuild_planner.py --from-config configs/experiments/c_pl_hubcom_hover_L100.yaml` (not run here).
- **C's port runs stand as flown** (decisions 2026-10-23 ~22:00): every C network run (R2446-R2763, the hub cells
  included) flew `diss_balanced_tensions` false, so its formation frame followed the ring's full measured attitude
  (`_frame_rot`). They are the code-based reference (our port of the authors' node network), never "the paper's law".

### 3.1 The readings (the one place they are set: quan_law.py's docstring; the switches in common.yaml `network:`)
| # | Open point | Reading flown | Source | Changed after the critic |
|---|---|---|---|---|
| R1 | the node and `q̇_i` | `quan_nodes algebraic`: Eq. 3, `q̇` its derivative from the measured `ẋ_i`, `ẋ_L`. Kept as a switch: `dynamic` = S2/S4's nodes as point masses (init.m's 1 kg) on the plane `h_c`, moved by the forces of S4's Lyapunov function (springs and dampers among the nodes and to the centre, the friction, the robot-node spring and damper's reaction), the robots flying Eq. 7 against them | main text Eq. 3 (algebraic); S2 l.368 and S4's proof (dynamic); the authors' code integrates its nodes | new switch; algebraic flies because it alone held offline (§3.4) |
| R2 | the `k_d,i` bracket | `k_d,i` scales the node-node sums and the friction only; `k_i`, `c_i` outside it, + sign | Eq. 7 as typeset; S2 Eq. control1 (term for term) | no (critic: settled) |
| R3 | `x_0 = q_0` | the commanded path point on the node plane, a rigid leader, `ẋ_0` the path's velocity, `w_i0 = 1` | S2: "the center ... can be a specific robot ... leader-follower"; init.m's desired centre `p00` on the node plane | no (critic: defensible; the code's centre is a 2 kg dynamic node) |
| R4 | `h_c` | the node plane **3.41 m** (3.41 cable lengths) above the load's commanded height, moving with it | init.m l.12-24: each robot placed at `mass + r(i)/rr*r0` with `r = rr = 2.02`, i.e. **1.0 m** from the payload; node plane `h0` -12 against the payload's -8.5858: 3.41; 2.02 is the node-node rest length `L0` (l.90). Conflict: Fig. 5's caption and S6 call 2.02 m the mean cable length; under that reading the plane is 1.69 cable lengths up (the first build's value) | yes: 1.69 -> 3.41 |
| R5 | `a_expected,i` | the path's commanded acceleration (zero in hover and holds) | Eq. 10 | no |
| R6 | `f_i`, `g` in Eq. 10 | they cancel (`a_cmd = a_expected + (h1+h2)/m_i`); the stand-in autopilot's thrust feed-forward carries the weight and a modelled pull `(m_L/n_i + m_rod) g / sin φ_i` along the measured load-to-quad line, the rods' 0.075 kg each included (`quan_rod_mass`; the offline harness's rods are massless, so 0 there) | S2 l.396 (`m ẍ = m g + f_i + u_i`); the model is the stand-in's, not the paper's | yes: the rod mass added |
| R7 | acceleration to velocity command | **R7b** (`quan_vcmd measured`): `v_d = ẋ_i + a_cmd dt`, no accumulated state, with `a_cmd` also the acceleration feed-forward; R7 (`integrated`, `v_d` accumulating `a_cmd`) kept for the comparison | S1 l.113: PX4 "executes velocity and acceleration commands"; Fig. 2 feeds `v_self` to the controller | yes: R7 -> R7b (Wesley ~23:20) |
| R8 | gains, range | `r` 2 m; `k_ij` 0.5, `c_ij` 0.25, `f_c` 0.5, `k_i` 1, `c_i` 1, `vel_kv` 3, `vel_k_att` 16 (§3.3's best) | their LiDAR filter (p.22); init.m's coefficients (k 10 per edge, c 10, 1 kg nodes, the Simulink friction gain `mu` 5) start the first searches | yes: re-searched |
| R9 | rate | the planner's 10 Hz | their simulation runs at 100 Hz (init.m `step` 0.01) | no |
| R10 | the load's attitude | never read: no `_frame_rot`, `_yaw_rot` or load quaternion on the law's path; the rod's tautness from the two positions | Eq. 1; S2's `p_{n+2}` (a point) | no |

**The authors' Simulink model** (`code/Normal/decentralized.slx`, its XML read, nothing run): three blocks, "Node
Dynamics", "Dissipative Controller", "Command Generator". Its MATLAB functions build the node distances and rates (the
nodes as `[p_x, p_y]` pairs plus the shared heights `z_u`, `z_l`: S2's generalised coordinates), a payload spring that
cannot push ("吊挂弹簧-负形变量限制模块": a suspension spring zeroed when shorter than its rest or with the load above
the tray) and a perception mask (`distance` 8). Gains in the controller subsystems: 7, `mu` 5 (horizontal), 20 and 4
(vertical); their wiring was not traced. It settles R1 for their simulation (dynamic nodes, the robot commanded from its
node: `caldata.m`'s projection, which is our port), not the main text's per-robot law.

### 3.2 World C (Wesley's word for this world, 23 Oct ~21:45)
`four_rigid_ground_rig_hubcom_L100.sdf` (`tools/make_carry_worlds.py`: `--hub-r 0.001 --attach-z 0.0`, then
`--pivot-dz 0.0` after the rig's 4 cm): every rod's payload end 1 mm from the ring's axis at its centre of mass, its
other end at the drone's centre. Ring, rods (0.075 kg), quads, masses, pose publishers and the drawn plates as the hub
world (`tools/test/test_rig_worlds.py`); `make_carry_worlds.py --check` shows every other world byte-identical. Nothing
holds the ring's tilt or yaw: both are reported, never tested.

### 3.3 The procedure (offline; named before its results were used)
The harness: `tools/network_tune.py --structure quan|quan_dyn|quan_r7|quan_dyn_r7` (the readings R1 x R7), the SIL
plant at world C's geometry (rods from the drone centres to a 1 mm hub at the ring's CoM; 3000 N/m push-pull rods,
massless), the law at 10 Hz from each quad's view with the `ema` velocities, our velocity loop at 50 Hz with
`vel_kp_pos` 0 and no I-term, the start settled on the 60 deg cone.
1. **The critic's discriminators** at the first build's start point (`k_ij = c_ij = f_c` 0.25, `k_i` 2, `c_i` 1,
   `h_c` 1.69, 20 s hovers): (a) R7, 4 quads, the rod tensions and the velocity commands on the self-stress pattern
   (+, -, +, -) against how far the quads moved; (b) R7 with 3 quads; (c) R7b with 4.
2. **Searches** (each: a 3 x 3 grid over `k_ij` x `c_ij`, then one descent pass over the five coefficients at x0.5,
   x1, x2 and `vel_kv` 3-5, `vel_k_att` 8-16; cases hover 60 s, `line_x` 0.4 (8 cycles) and the margin flight at
   `vel_kv` x1.25; `h_c` 3.41):
   - from init.m's coefficients scaled by our quad's 0.55 kg against their 1 kg node (`k_ij = c_ij = k_i = c_i` 5.5,
     `f_c` 2.75), R7b x {algebraic, dynamic};
   - from the low point that held calm in (c), R7b algebraic (`--quan-start low`).
3. **Rank** (fixed after the critic, F2): fewer cases lost first, then fewer guards broken, then how far the ring got
   (the worst case's band), then J. A point that diverges no longer outranks one that holds.
4. **Guards (world C):** no divergence; quad tilt at most 35 deg; throttle at most 0.78; the ring within 300 mm of the
   path for the whole case (stricter than §5's 1 s rule).
5. The best point flies all four cases once (hover, 0.2, 0.4, 0.6) and the margin flight.

Budget: at most 30 min of searching on at most 4 laptop workers, nice 10, no Gazebo live (used: 22:51-23:13, 2+2
workers).

### 3.4 Offline results (exploratory; `results/analysis/network_tune_quan_*_2026-10-23.json`)
**The discriminators** (20 s hovers; tension and velocity sums with the self-stress signs, N and m/s):

| Probe | Ring off the target: RMS / farthest | Quad tilt peak, throttle peak | Self-stress tension (+-+-) | Self-stress `v_d` along the rods | Common-mode `v_d` along the rods | Quads moved |
|---|---|---|---|---|---|---|
| (a) R7, 4 quads | 220 / 549 mm | 34 deg, 0.80 | 0.01-0.21 N (mean rod 2.0-3.4 N) | -0.03 to +0.03 | to -0.40 m/s | 0.50 m |
| (b) R7, 3 quads | 87 / 249 mm | 33 deg, 0.80 | (no such mode) | - | to -0.19 m/s | 0.38 m |
| (c) R7b, 4 quads | 112 / 136 mm | 9 deg, 0.60 | -0.02 N | -0.008 | within 0.006 | 0.14 m, then still |

- **The self-stress wind-up is not what fails** (a): the (+, -, +, -) tension pattern stays within 0.2 N and the
  velocity commands on it within 0.03 m/s while the quads wander 0.5 m. The wind-up is common-mode: R7's `v_d` runs to
  0.4 m/s along the rods in step. Three quads (b), which have no self-stress mode, still wander and reach the tilt cap,
  so (b) does not hold and the 3-quad twin neighbour is not proposed.
- **R7 is the failing reading:** the same point under R7b (c) is calm (tilt 9 deg, throttle 0.6), with a standing
  offset of 0.14 m that stops growing.

**The searches:**

| Reading, start | Points | Held every case | Best point (`k_ij, c_ij, f_c, k_i, c_i`; `vel_kv`, `vel_k_att`) | Hover: RMS / farthest | Line 0.4: headline / farthest | Margin: farthest |
|---|---|---|---|---|---|---|
| R7b, algebraic, init.m scaled | 19 | 0 (every point diverged in every case, 3.4-27 s) | - | - | - | - |
| R7b, dynamic, init.m scaled | 19 | 3 (none within the band) | 5.5, 11, 1.375, 5.5, 2.75; 4, 16 | 1039 / 2572 mm | 1720 / 2544 mm | 2412 mm |
| R7b, algebraic, low start | 19 | 18 | **0.5, 0.25, 0.5, 1, 1; 3, 16** | **124 / 148 mm** | **222 / 319 mm** | 331 mm |

The best point on every case (`results/analysis/network_tune_quan_r7b_alg_best_2026-10-23.json`):

| Case | Hover 3-D RMS (first 5 s) | Clock-free RMS | Reach RMS (mean: - past the ends) | Headline | Lag | Ring farthest from the path | Quad tilt peak | Throttle peak |
|---|---|---|---|---|---|---|---|---|
| Hover | 124.1 (40.2) mm | | | | | 148 mm | 9.3 deg | 0.60 |
| Line 0.2 | | 43.6 mm | 143.1 (-66.1) mm | 186.7 mm | -0.21 s | under 300 mm | 9.9 deg | 0.60 |
| Line 0.4 | | 43.6 mm | 178.1 (-122.7) mm | 221.6 mm | +0.13 s | 319 mm | 11.9 deg | 0.61 |
| Line 0.6 | | 43.9 mm | 200.3 (-145.3) mm | 244.3 mm | +0.19 s | 367 mm | 15.0 deg | 0.62 |
| Margin (0.4, `vel_kv` x1.25) | | 40.7 mm | 186.4 (-140.6) mm | 227.1 mm | +0.11 s | 331 mm | 12.0 deg | 0.61 |

- **What holds offline: R7b with algebraic nodes, at low coefficients** (the springs at or below 1 N/m). No point
  passes every guard: the hover is inside the band, the lines run past their ends and touch 0.32-0.37 m at 0.4-0.6 m/s.
  This is the point `c_net_quan.yaml` flies.
- **Dynamic nodes (S2/S4) at init.m's coefficients do not hold** (1-2.6 m off, quads at the tilt cap); a low start for
  them was not searched (budget).
- The standing hover offset (0.12 m) is the absence of integral action under R7b with springs this soft; the earlier
  "1.95x lever" mechanism (first draft) was mis-stated (critic F1: `k_d` is exactly the inverse lever) and is withdrawn.

## 4. Baseline
- Our planner on world C (`c_pl_hubcom_*`), same batch, same tree. **No expectation is quoted:** R2725 (1.4 mm) flew
  the hub 2 cm above the CoM, where the ring has a restoring moment; on world C the OCP's ring attitude acts through
  1 mm levers (critic F3).
- **Their accuracy, stated apart (not a bar):** Fig. 5A (simulation, 10 robots, cables 2.02 m mean, zero uncertainty):
  the load's tracking error median ~1.6 m, ~0.8 cable lengths, against the clock; hardware: no tracking error given,
  speeds ~1 m/s, tensions balanced (Fig. 3D, 4D). World C's band (0.3 m, path distance) is ~2.7 times stricter than
  their own median, so "within our band" is a stronger result than "as accurate as theirs".

## 5. Scoring (`tools/c_metrics.py`, as C's §5 except the loss rule)
- **Windows, target, headline:** C's: hover 60 s from the hand-over (first 5 s apart), lines over whole cycles from
  `PATH_START`, both arms against the target from before the hand-over.
- **World C's LOSS** (any run on `four_rigid_ground_rig_hubcom_L100.sdf`, every arm, in place of the 30 deg tilt rule):
  before the window ends, the ring more than **300 mm** from the path (the hover's target; the line, its ends
  included) for **1 s** without a break after the hand-over; or a fleet abort, a quad over 60 deg, or a DROP AND LAND /
  ENVELOPE FAULT / UNANNOUNCED DETACH / FLEET ABORT line before LAND. The ring envelope is off on world C, so a ring's
  tilt cannot cause an ENVELOPE FAULT there.
- **VOID:** as C (under 95 % of the window's rows with no loss, or no `NET_HANDOVER` in a network arm); re-flown once.
- **Reported on world C (scored for every run there):** the ring off the quads' centroid (horizontal RMS); the largest
  std/mean of a quad-quad distance; each rod's tension estimated from the logged throttle through the twin's affine map
  (each quad's own offset, the 23.8 V start), its tilt and its vertical acceleration (`t_i u_z,i = T_i cos tilt_i -
  m (a_z,i + g)`), and their spread ((max - min)/mean, averaged). On R2725 (our planner, the hub) it reads 2.94 N per
  rod and a 0.5 % spread: the level carries a bias (the rod's own weight at the quad end, the pack's sag), the spread
  does not.

### Bars (PROPOSED; freeze with Wesley's word before the first run; n = 2 per config)
| Bar | Per cell | Passes when |
|---|---|---|
| **Q1, carries the ring within our band** | hover, line 0.2, line 0.4 | both law runs valid and not lost (§5) |
| **Q3, the formation keeps its shape (their Theorem 1)** | each | both law runs: every quad-quad distance's std at most 2 % of its mean in hover, 5 % on the lines |
| **Q4, balanced load sharing (their Fig. 3D)** | each | both law runs: the estimated tensions' spread at most 10 % in hover, 20 % on the lines (the paper publishes no number for its balance; these are ours) |
| **A, the arms on world C** | each | C's claim rule: "our planner at least as close" when both our runs beat both law runs on the headline by the minimum effect (the larger of 1 mm and 10 % of the better mean); "the law closer" the other way; else "no difference shown" |
| **O, our planner flies world C** | each | both our runs valid and not lost |

- **Q2 (the ring under the quads' centroid) is reported, not claimed:** with equal rigid rods to one point the ring is
  under the centroid whenever the elevations are equal, whatever the controller (critic F3); Theorem 2's content is
  the unequal-capacity weights, which world C does not test.
- **Q5 dropped:** their accuracy (§4) is stated beside the table, not tested.
- No bar says "reproduces their result": Q1, Q3 and Q4 together are "the law carries the ring within our band, keeps its
  formation and shares the load", at our scale, with §3.1's readings.

## 6. Runs (twin; after the frozen bars)
1. **Smoke, exploratory, x1 each:** `c_pl_hubcom_hover_L100`, `c_net_quan_hubcom_hover_L100`, and the neighbour
   `c_net_port_hubcom_hover_L100` (it separates "world C is not flyable by a network on our velocity loop" from "the
   law fails"). Falsified (the world or the arm does not fly as built) by: no lift; no hand-over in a network arm; a
   LOSS before LAND; a VOID. A loss in the law's arm is worded "the law as read **with algebraic nodes, R7b and §3.1's
   readings** does not hold the ring in the twin", never "the law fails". Checked as well: the hand-over line
   "net_law quan, algebraic nodes, v_d measured" with the node plane and `l_i(0)`, the trackers on the velocity loop,
   `drop_tilt_deg` 180 and `max_payload_tilt_deg` 180 in every arm's tracker parameters.
   **Per-run falsifiers (critic pass 2, 23 Oct ~23:25; one registry row each):**
   - `c_pl_hubcom_hover_L100`: our planner does not carry the ring on world C: no lift, a VOID, or a LOSS before LAND
     (band, a quad over 60 deg, a fleet abort, a fault line). If it fires, the world-C baseline O is unusable and the cells wait.
   - `c_net_port_hubcom_hover_L100`: a network on our velocity loop does not hold world C: no NET_HANDOVER, a VOID, or a
     LOSS before LAND. If it fires, a quan LOSS cannot be put on the law.
   - `c_net_quan_hubcom_hover_L100`: the law as read with algebraic nodes, R7b and §3.1's readings does not hold the ring
     in the twin: no NET_HANDOVER, a VOID, a quad over 60 deg, a fleet abort or fault line, or a band LOSS that is still
     growing or comes with the quads at the guards (tilt over 35 deg, throttle over 0.78). A band LOSS with calm quads and
     an error that has stopped growing, explained by `datum_shift_mm` plus the standing offset (R7b has no integral; the
     network shifts its datum onto the ring at the hand-over, and c_metrics scores against the target from before it), is
     recorded as 'held, X mm off the target (datum shift Y)', not 'does not hold'. Each row reads `datum_shift_mm`,
     `z_err_mean_mm` and `err_own_rms_mm` first. A known twin nuisance, noted not fixed: with no vertical integral an
     upward standing offset over 0.10 m can keep `load down` from firing at LAND (dissipative_node.py:4097).
   **The smoke does not set the bars.** It flies (23-24 Oct overnight, exploratory) before Wesley freezes §5's bars;
   the bars may not be moved on its evidence except by Wesley, who sees it in the morning report.
2. **Cells:** the six configs x2 (12 runs), arms interleaved, one tree.
3. `python3 tools/c_metrics.py --table <runs>` on world C's runs alone, then `/report`, the reviewer.

The 3-quad neighbour is not proposed (discriminator (b) did not hold). Cost: 3 + 12 = 15 twin runs.

## 7. New code
- `src/dissipative_planner/dissipative_planner/quan_law.py` (pure numpy): Eq. 3 and its rate, Eq. 5, 7-10; the node
  reading (`algebraic|dynamic`), the velocity command (`measured|integrated`), the rod mass in the pull model.
- `dissipative_node.py`: `net_law port|quan` (default port). With quan, `_enter_network_phase` builds the law on the
  measured state; `_network_plan` hands every tethered quad to `_quan_plan` (the port's network is not stepped; dynamic
  nodes advance once a tick). Port: unchanged (a test compares the published references bit for bit).
- `bringup`: `net_law`, `quan_nodes`, `quan_vcmd` and the `quan_*` coefficients in common.yaml `network:` (network graph
  only); `payload_tilt_envelope` in sim.yaml (twin only; false sends `max_payload_tilt_deg` 180 to the trackers).
- `tools/c_metrics.py`: the arm `quan`; world C's band rule in place of the tilt rule; the world-C metrics with the
  estimated rod tensions.
- `tools/network_tune.py`: the quan structures, world C's geometry and guards, the fixed rank, `--quan-start`.
- World C and its test; the configs; unit tests (Eq. 3 geometry and rate, the hand-over's force-free rest, the signs,
  `k_d`'s scope, Eq. 10 as read, R7 against R7b, dynamic nodes at rest and pulled, the rods in the pull, the range; the
  switch's default path; both node readings through the node; the law never reading the ring's attitude; the envelope
  switch on world C's params and absent elsewhere).

## Critic
(23 Oct, before any run; the critic's text, pasted by the integrator; sources: main text p.24-26, the uncommented
supplementary docs/papers/quan2025_supplementary_S1-S9_from_arxiv_source.tex, init.m, quan_law.py, dissipative_node.py
3918-3925 / 4204-4227, tools/network_tune.py, reference.md §6 and §10; the Simulink XML not decoded gain by gain.)

**1. Already tried?** The law is new; its neighbours lost the ring (the paper's form on the rim R2744-R2749 and on today's
hub R2780 rocked, R2781 line lost). The suspected mechanism is a measured negative: R7's `v_d = ∫a_cmd`, flown with
`a_ff ⊃ a_cmd`, gives `kv(v_d - v) = kv∫(a_cmd - a)`, a per-drone integrator of each quad's own acceleration error;
reference.md §6: "per-drone integrators (`vel_ki 1`) tilt the load to 60°" (R0077 series); root cause
docs/archive/thesis_plan_v2_2026-09.md:824-835 (a differential integral bias is a differential cable tension).

**2. Simplest unaddressed explanation.** Four rigid rods meeting at one point are statically indeterminate by one: the
tension pattern (+s, -s, +s, -s) on quads 0/1/2/3 is self-equilibrated and moves no position. The law reads only
positions, so it has no feedback on that pattern, and R7 integrates whatever part of `a_cmd` falls in it, unbounded,
until the throttle (0.8) and tilt (35 deg) caps. It fits §3.4: every held case hit the tilt cap and the throttle
ceiling even at k_ij 0.25 (spring loop ~1.3 rad/s: a 0.2-0.3 s lag costs < 20 deg of phase, so lag cannot explain it);
the 243-point grid with heavily damped points held 10; the card itself says "v_d integrates that component without
moving it". That is a property of R7 with rigid rods and 4 quads on one point, not of the law.

**3. Does the falsifier test it?** No. The smoke run's LOSS tests the law, R7 and the rigid self-stress mode together,
and would be reported as "the law as read does not hold the ring". Every "what does not change it" probe in §3.4 (50 Hz,
no filter, k_att 24, no integrator, h_c 0.5/3.0, the true f_i) ran at k_ij 5 alone: no damping term at all (c_ij = c_i =
f_c = 0), an undamped spring network that should diverge under any delay. Those probes say nothing about the law.

**4. What it breaks if it works.** Nothing by default (`net_law` stays `port`). The locked yaw-only rule: recorded
exception (decisions 2026-10-23 ~22:00). World C: the trackers' 60 deg ring envelope acts on a ring whose tilt carries
no information (Wesley's call; F3). Twin only.

**5. Asked for?** Yes (Wesley 2026-10-23 ~21:10 and ~21:45: "we want to be able to get their results").

### F1. The readings against the text, S1-S9 and init.m
| # | Verdict | Evidence |
|---|---|---|
| R1 (q̇ from differentiating Eq. 3) | consistent with the main text, CONTRADICTED by S4's proof | S2 (l.368) and Theorem 1's proof (l.588) give each node its own kinetic energy ½m_i‖q̇_i‖² and dynamics, as the authors' code does (integrated nodes, masses m = [2, 1..1, 5] in init.m). No published stability argument covers algebraic nodes. A conflict for Wesley; not the first change. |
| "1.95x lever" (§2, §3.4) | MIS-STATED | k_d,i = (h_L - h_i)/(h_L - h_c) = 1/λ exactly, λ = 1 + s the node's lever; S2 (l.348) says k_d maps the node's input onto the robot. After k_d a spring acts on the robots' spacing with coefficient k_ij (first order, equal heights). "15 N/m from k_ij 5" is the number of springs (3 neighbours + centre), not the lever. Remove that mechanism from §2. |
| R2 (k_i, c_i outside k_d, + sign) | SETTLED, correct | S2 Eq. control1 (l.351-352) matches Eq. 7 term for term; the dissipation function (l.374) puts c_i on l̇_i in 3-D; the signs dissipate for a force on the robot. |
| R3 (x_0 = the path point) | defensible, not settled | S2 (l.287): "the center of the robot swarm can be a specific robot ... leader-follower"; the code's centre is a dynamic 2 kg node (Eq. 16, l.282) the nodes pull on; R3 makes it a rigid exogenous leader. After k_d the path error enters the j=0 spring at ~0.5 k_ij with half projection: at the start point the formation's translation mode has ~20 s period and ~6 s time constant, which alone explains 0.2-0.8 m of wander. |
| R4 (h_c = 1.69 cable lengths) | derivation looks WRONG | init.m l.18 places each robot at `mass + r(i)/rr*r0` with r = rr = 2.02: the robot is 1.0 m from the payload and 2.02 is the node-node rest length L0 (l.90). Node plane -12, payload -8.5858: 3.41 cable lengths, not 1.69. Fig. 5's "2.02 m mean cable" (S6 table) conflicts; check before quoting either. |
| R5 (a_expected) | unsettled | reasonable |
| R6 (f_i, g cancel; modelled pull) | algebra SETTLED; the model is an addition | S2 l.396: m ẍ = m g + f_i + u_i, f_i the tension: the cancellation is right, and Theorem 1's proof needs the robot to cancel the TRUE tension (f_i never enters dV/dt); the load is absent from the Lyapunov function, so Theorem 1 says nothing about the load. On rigid rods cancelling the true rod force is ill-posed (the constraint reacts to thrust). `m_L g/(n sinφ)` is nowhere in the paper: call it the stand-in autopilot's feed-forward. Twin gap: the rods' 4 x 0.075 kg are not in `load_mass` (dissipative_node.py:3919), ~0.74 N per quad, hidden by R7 today. |
| R7 (v_d = ∫a_cmd) | NOT settled; the reading to change FIRST | S1 (l.113): PX4 "executes velocity and acceleration commands": `a_ff = a_cmd` is supported; nothing says v_d accumulates a_cmd. Fig. 2 feeds v_self into the controller; the code integrates the node and projects a robot position reference (a position loop on the measured state). |
| R8 (gains) | partly settled for their simulation | init.m: k = 20, K = 10 per edge, c = 10, node mass 1 kg: damping ratio ~1.6 per edge (heavily overdamped). QUAN_SPACE caps c_ij at 0.5 with k_ij up to 1: never reaches it. |
| R9 (rate) | settled for their simulation | init.m `step = 0.01`: 100 Hz. The planner's 10 Hz is ours; 50 Hz in the tracker would be closer. |
| R10 (point load) | SETTLED | S2: the load is p_{n+2}, z_l, a point. |

**Change R7 first:** R7b, `v_d = v_self + a_cmd·dt` (Fig. 2's own input, no accumulated state), with S1's acceleration
command kept as `a_ff`: each quad realises a_cmd without integrating against the rods. Needs Wesley's word and a new
§3.1 row. Offline discriminators at the START POINT (not k_ij 5 alone), minutes each: (a) log the four quads' thrust on
the (u0, -u1, u2, -u3) self-stress vector under R7: it ramps while positions barely move if R7 is the cause; (b) 3 quads
at R7 (three rods to a point have no self-stress mode: they hold if R7 is the cause); (c) R7b with 4 quads. If (a) shows
no ramp and neither (b) nor (c) holds, R7 is cleared. If R7b is adopted, add the rod mass to the pull model (a
parameter), or the twin sags ~0.74 N / k_i (~0.37 m at k_i 2) along the rods.

### F2. The offline harness
Mostly trustworthy for this failure: its known errors are ring rotation on rim mounts (the 1.5 Hz SIL rock, decisions
2026-09-25; reference.md §6/§10), irrelevant on world C. The suspected mode is internal force; the harness's rods are
3000 N/m push-pull springs (tools/sil/plant.py RodParams) and Gazebo's are rigid joints: both have the self-stress mode.
The twin adds the unmodelled rod mass and the OCP hand-over transient: expect it no better. Not trustworthy on its own
diagnosis (§3.4's robustness list is from an undamped point). Fix the search's rank rule first (3 divergences rank above
8 guard misses).

### F3. The loss rule and Q1-Q5
- The band is stricter than their result: Fig. 5A's median load error ~1.6 m on 2.02 m cables (~0.8 L, a clock error);
  world C's loss is 300 mm path distance (~0.3 L). A law at their accuracy would score LOST, so Q1-Q4 as "their result"
  ask 2.7x their own accuracy. Q5 is empty (any non-lost run passes). Rename Q1 "carries the ring within our band" and
  drop "reproduces", or state their accuracy separately.
- Q4 cannot measure tension balance on world C: equal elevations do not mean equal tensions (the self-stress pattern
  changes no elevation; exactly what R7 winds up). Estimate each rod's force from logged thrust (throttle through the
  rig map, attitude) minus m(a - g); or a force-torque sensor on the rod joint (a world SDF change: Wesley's word).
- Q2 is geometry, not Theorem 2: equal rigid rods to one point put the load under the quads' centroid whenever the
  elevations are equal (any controller passes). Theorem 2's content is the α weights (unequal capacities/lengths).
  Report Q2; do not call it a reproduction.
- The 60 deg ring envelope on a free-tilt ring can fire with no carrying failure: Wesley turns it off on world C, or an
  ENVELOPE FAULT caused only by ring tilt is VOID, not LOSS, on world C (frozen before the runs).
- Baseline O is unknown: R2725 flew the hub 2 cm above the CoM (restoring moment); on world C the OCP's ring attitude is
  nearly uncontrollable through 1 mm levers. Do not quote 1.4 mm as an expectation.

### F4. One variable, baseline, falsifiers, repeats
- The arms differ in the whole post-hand-over stack (method against method, C's pattern). The missing neighbour: the
  PORT on world C, hover, once (same velocity loop, `net_law port`), beside the smoke pair: it separates "world C is not
  flyable by a network on our velocity loop" from "the law fails".
- Add the offline discriminators (a)-(c) to §3.3 before any twin run; reword the smoke result: "the law as read WITH R7
  does not hold the ring".
- n = 2 is fine for within-run bars; before the 12 cell runs fly the 3-quad neighbour once in the twin if (b) holds
  offline.

**Verdict: change readings first.** Run (a)-(c) offline and take Wesley's word on R7 (R7b) and R4 (re-derive from
init.m) before the smoke hover; rewrite Q4 on tension and reword Q1/Q5.

CRITIC: rewrite the card. The smoke run cannot tell the law apart from R7's per-quad integrator acting on four rigid
rods at one point, a mechanism already measured as a negative (reference.md §6, vel_ki 1).

## Response (23 Oct ~23:30, to the critic above, after Wesley's decision ~23:20)
| Critic point | Done | Where |
|---|---|---|
| Change R7 first (R7b) | R7b is the flown reading (`quan_vcmd measured`); R7 kept as a switch. Offline it is the difference between wandering at the tilt cap and a calm hover (§3.4 (a)-(c)) | §3.1 R7, §3.4 |
| The self-stress wind-up (point 2) | tested: not the mechanism. The (+, -, +, -) tensions and commands stay small while R7 winds up in common mode; 3 quads (no such mode) fail too | §3.4 (a), (b) |
| "What does not change it" ran at an undamped point (point 3) | withdrawn; the discriminators ran at the start point with all five terms | §3.3, §3.4 |
| R1 contradicted by S4 | a switch: `algebraic` (Eq. 3) and `dynamic` (S2/S4 with init.m's node mass), both searched; algebraic flies because it alone held | §3.1 R1, §3.4 |
| "1.95x lever" mis-stated | removed | §2, §3.4 |
| R4 from init.m | 3.41 cable lengths, the derivation stated, the Fig. 5 / S6 conflict noted | §3.1 R4 |
| R6: the rods' mass | `quan_rod_mass` 0.075 in the twin's params (0 in the harness, whose rods are massless) | §3.1 R6 |
| R8: init.m's gains, damping ratio | the first searches start at init.m's coefficients (scaled by mass); they diverge (algebraic) or wander metres (dynamic); the flown point is from the low start | §3.3, §3.4 |
| R9: 100 Hz in their simulation | stated; the law stays at 10 Hz (not tested faster in this round) | §3.1 R9 |
| F2: the rank rule | fixed for the quan searches (cases lost first) | §3.3 |
| F3: the band is stricter than their result; Q1 wording; Q5 empty | Q1 renamed "carries the ring within our band"; their accuracy stated apart; Q5 dropped | §4, §5 |
| F3: Q4 on tension, not elevations | Q4 on rod tensions estimated from thrust, tilt and acceleration | §5 |
| F3: Q2 is geometry | reported, not claimed | §5 |
| F3: the 60 deg ring envelope | off on world C in every arm (Wesley ~23:20), a twin-only switch | §3, §5 |
| F3: baseline O unknown | no expectation quoted | §4 |
| F4: the port on world C | `c_net_port_hubcom_hover_L100`, once, beside the smoke pair | §3, §6 |
| F4: the smoke's wording | "the law as read with algebraic nodes, R7b and §3.1's readings" | §6 |
| F4: the 3-quad neighbour if (b) holds | (b) did not hold: not proposed | §3.4, §6 |

Open for Wesley: the R4 conflict (init.m's 1.0 m robot distance against Fig. 5's "2.02 m mean cable"); whether a
low-start search for dynamic nodes is worth its budget; the Q3/Q4 numbers.

## Smoke R2793: the climb (24 Oct ~01:15, diagnosis; offline only, nothing flown)
**Verdict: the climb is the pull model counting each rod twice.** `c_net_quan.yaml` adds `quan_rod_mass` 0.075 to a
`drone_mass` 0.55 that already holds the rod (0.475 kg airframe + 0.075 kg rod and magnet: `make_carry_worlds.py`
l.8-15, `sim.yaml` l.5). At the hand-over each quad's thrust feed-forward stepped up by 0.075 g = 0.74 N (2.9 N on the
3.06 kg fleet), and the only thing in the law that holds a height is the robot-node spring `k_i` = 1 N/m: the fleet
rose until that spring pushed back. The critic's F1 ("the rods are not in `load_mass`") was right about `load_mass` and
missed `drone_mass`; our OCP's own model (0.55 + m_L/4, massless rods) held the ring at 1.000 m with its integral at 0
before the hand-over, so its masses were right.

**The run** (run.csv; planner log; the law's terms re-computed from the logged states, `results/analysis/quan_climb_R2793_2026-10-24/terms.py`, output `terms_R2793.txt`):

| | Ring z (m) | Quad z (m) | Rod elevation | Quad tilt mean / std | Throttle |
|---|---|---|---|---|---|
| Before the hand-over (25.5-27.2 s) | 1.000 | 1.866 | 60.0 deg | 11.5 / 0.0 deg | 0.586-0.593 |
| Top of the climb (30.2 s) | 1.568 | 2.431 | 61 deg | 11 / 1 deg | 0.58 |
| Held (45-90 s) | 1.458 | 2.33 | 60.5 deg | 15.0 / 6.6-7.0 deg | 0.61 |
| After LAND (100-160 s) | 0.52 | 1.39 | 60 deg | 15 / 7 deg | 0.62-0.66 |

| Vertical term per quad (N, + up) | Hand-over | +0.5 s | +1.5 s | +3.0 s (top) | Held 45-90 s (mean) | After LAND (130 s) |
|---|---|---|---|---|---|---|
| Modelled pull (R6), (0.215 + 0.075) g | 2.845 | 2.845 | 2.845 | 2.845 | 2.845 | 2.845 |
| What the drone needs above 0.55 g (our OCP: 0.215 g) | 2.11 | 2.11 | 2.11 | 2.11 | 2.11 | 2.11 |
| Robot-node spring `k_i` (along the rod, 3-D) | 0.00 | -0.10 | -0.45 | -0.56 | **-0.47** | -0.46 |
| Robot-node damper `c_i` | 0.00 | -0.33 | -0.30 | +0.15 | 0.00 | +0.22 |
| `k_ij`, `c_ij`, friction (horizontal only: Eq. 7's "hor") | 0 | 0 | 0 | 0 | 0 | 0 |
| a_cmd,z (m/s^2, mean of four) | 0.00 | -0.78 | -1.36 | -0.75 | -0.85 | -0.44 |

- The excess 0.736 N per quad is held by the spring's -0.47 N plus the R7b velocity term (`kv (v_d - v)` = `kv a_cmd dt`
  = 0.3 a_cmd: ~0.14 N) and the lift the wobble costs (below). Gravity and f_i cancel as read (R6); a_expected is 0.
- **Node plane and l_i(0):** 4.41 m (1.0 + 3.41), l_i(0) 2.94 m, at the hand-over (log line). `h_c` follows the
  commanded height only (1.0 m held, then LAND's ramp to 0.06 m: 3.47 m), never the measured ring, so the offset rides
  down with LAND: the ring stops 0.46 m above its 0.06 m target and `load down` (ring within 0.10 m of it) cannot fire.
- **The cone did not change:** the elevation stays 60-61 deg and each quad 0.49-0.50 m from the ring's axis. As the fleet
  rose under a fixed node plane the nodes moved in; `k_ij` pushes each quad out 0.25 N and `k_i`'s horizontal part
  pulls it in 0.27 N: balanced.
- **The 27-28 deg "lean" is a wobble, not an outward pull:** a 2.6 Hz attitude oscillation (std 6.6-7.0 deg about a
  15 deg mean; 11.5 deg before) that starts ~2.3 s after the hand-over, grows and then stays the same size. Peaks 27-28.
  The same wobble is in the paper's damping form, also at `vel_k_att` 16 (R2780: 2.9 Hz, std 6.3 deg; R2744: 3.1 Hz,
  std 5.5 deg), and not in the port on world C at `vel_k_att` 8 (R2792: std 0.0). The harness cannot show it (its rate
  model is the tracker's own). Likely `vel_k_att` 16 in the twin's attitude loop; not tested. **A caution for
  R2744-R2749 and R2780-R2781:** the paper's form flew this gain too.

**The paper on the vertical** (supplementary, docs/papers/quan2025_supplementary_S1-S9_from_arxiv_source.tex):
- l.339, 372: the per-robot law holds "virtual nodes at a fixed altitude" `h`. Its Lagrangian has `sum m_i g h` and
  the robot-node spring `k_i (||q_i - x_i|| - l_i(0))^2` in 3-D. `z_u` and `z_l` (l.149, the nodes' common height and the
  load's) are generalised coordinates of the node-network model only (the authors' code: vertical gains 20 and 4).
- l.340-354 (Eq. control1 = Eq. 7): `k_ij`, `c_ij` and the friction are horizontal ("hor"); `k_i` and `c_i` are 3-D;
  `u_i` contains `-f_i - m_i g` with `f_i` **the cable's true tension** (l.396, `m_i x''_i = m_i g + f_i + u_i`).
- l.113 and Eq. 10: the autopilot (PX4) gets velocity and acceleration commands without `f_i` or `g`. Whatever carries
  the weight and the tension is the autopilot's.
- So in the paper a robot's height is set by the node plane through `k_i`, at exactly `l_i(0)` below its node, because
  the true tension is cancelled. **Where our build departs:** (a) R6 cancels a model of `f_i`, not the true one (known,
  stated); (b) the model counted the rod twice (the cause); (c) at `k_i` 1 N/m (§3.3's search, picked for horizontal
  calm) every 0.1 N of model error is ~0.1 m of height. Their simulation's vertical gains are 20x stiffer.

**Offline:** the harness's 124 mm standing hover offset is **horizontal** (height error -1 mm): its rods are massless and
its law had `rod_mass` 0, so it could not see this. New harness option `fly(twin_rods=True)`: the plant's quads 0.495 kg
and its ring 1.08 kg (the twin's rod and magnet split between the ends), the controllers unchanged. It reproduces R2793
from the settled cone: climb 0.60 m, held +0.57 m, LAND stuck at 0.64 m. The twin handed over from our OCP's settled
state and the harness from the law's own: both climb, so the start is not the mechanism; the force step is.

### The reading (proposed for §3.1; switch `quan_fz`, default `model` = as flown)
| # | Open point | Reading proposed | Source |
|---|---|---|---|
| R6b | what the stand-in's model misses of `f_i`, vertically | `quan_fz autopilot`: the autopilot's integral on its vertical velocity error, `b_i += fz_ki (v_d,z(last tick) - x'_i,z) dt` (`fz_ki` 2 /s^2, within 3 m/s^2), added to the thrust feed-forward. It starts on the thrust flown before the hand-over (`b_i(0)` = the law's pull minus our OCP's cable term, per quad), as an autopilot keeps its hover thrust across a mode change. In a still hover it settles where a_cmd,z = 0: the paper's equilibrium with the true `f_i` cancelled. With it, `quan_rod_mass` is the rod's share carried at the load end, outside `load_mass` (world C: the 0.035 kg magnet + half the 0.040 kg rod = **0.055**); it sets the pull's horizontal part, and R6b absorbs its vertical over-count | S2 l.354, 396 (`u_i` cancels the true `f_i`); S1 l.113 and Eq. 10 (the autopilot carries what the command leaves out). The integral and its bumpless start are the stand-in's, as R6's model is |

**Before / after, offline** (`results/analysis/quan_climb_R2793_2026-10-24/`; c_net_quan.yaml's point; hover 40 s then
LAND; "held" = 20-40 s; line 0.4 m/s, 8 cycles):

| Plant | Pull model / R6b | Climb peak | Height held | Horizontal RMS (hover) | Ring after LAND (target 0.06) | Line 0.4: headline / farthest | Line: height |
|---|---|---|---|---|---|---|---|
| twin rods | rod 0.075, model (**R2793 as flown**) | +598 mm | +571 mm | 84 mm | 0.64 m (stuck) | 733 / 634 mm | +572 mm |
| twin rods | rod 0.075 + R6b | +100 mm | +93 mm | 53 mm | 0.16 m (at the edge) | 307 / 362 mm | +94 mm |
| twin rods | **rod 0.055 + R6b (proposed)** | **+7 mm** | **+1 mm** | **49 mm** | **0.05 m** | **206 / 319 mm** | **+1 mm** |
| twin rods | rod 0, model | 0 | -161 mm | 78 mm | 0.03 m | 318 / 308 mm | -160 mm |
| twin rods | rod 0 + R6b | 0 | -159 mm | 32 mm | 0.03 m | 296 / 278 mm | -158 mm |
| massless (§3.3's plant) | rod 0, model (§3.4's point) | +4 mm | -1 mm | 130 mm | 0.06 m | 222 / 319 mm | -1 mm |
| massless | rod 0 + R6b | +6 mm | +1 mm | 54 mm | 0.05 m | 149 / 256 mm | +1 mm |

- Quad tilt and throttle peaks are unchanged by R6b in every row that holds (11.6-12.8 deg hover, 0.60-0.62).
- `fz_ki` 5 instead of 2 changes nothing material (rod 0.055: +7 mm, held +1 mm; line 192 / 256 mm).
- **What R6b does not fix: the cone angle.** With R6b each quad sits at exactly `l_i(0)` below its node, so the ring's
  height follows the rods' elevation (h_L = h_c - (1 + l_i(0)) sin phi: ~34 mm per degree), and the elevation is set by
  the modelled pull's horizontal part against the soft node springs. A rod share that is too large flattens the cone and
  lifts the ring (rod 0.075 on the twin: +93 mm; on the massless plant rod 0.075 + R6b climbs 1.37 m); too small narrows
  it and drops the ring (rod 0: -160 mm). So the rod share must be right, R6b or not.
- Not shown offline: the 2.6 Hz wobble (the harness has no such mode), and the rig's slack tethers.

**Code** (all default off; `net_law port` untouched): `quan_law.py` R6b (`QuanParams.fz|fz_ki|fz_max`, `QuanLaw.b_z`,
`start(a_cable0=)`); `dissipative_node.py` passes the OCP's last cable term per drone (`_ho_ac`) and logs `b(0)` on the
hand-over line; `common.yaml` `quan_fz 'model'`, `quan_fz_ki 2.0`, `quan_fz_max 3.0`; `graph_detach.py` passes them;
`tools/network_tune.py`: `quan_rod_mass`, `quan_fz_ki` knobs (0 = R6 alone), `fly(land_at=, twin_rods=)`. Tests: three
in `test_quan_law.py` (off by default; the bumpless start; the integral and its bound), one in `test_quan_switch.py` (the
OCP's thrust carried into the first command), the launch test's key list. `tools/gate.sh --quick` PASSED (1953).

**For the next twin run** (exploratory, one row): `c_net_quan_hubcom_hover_L100` with a params file that is
`c_net_quan.yaml` plus `quan_fz: 'autopilot'`, `quan_fz_ki: 2.0`, `quan_rod_mass: 0.055`. R6b is an estimator in the
flight loop: a critic pass (or Wesley's waiver) first. Falsifier: the ring more than 0.10 m above or below 1.0 m in the
5 s after NET_HANDOVER or on average over the hold, or no `load down` after LAND. Expected: the hand-over line reads
`f_i,z autopilot ki 2 from b` about -0.85 to -1.1 m/s^2 per quad. A no-new-code control, if wanted first: `quan_rod_mass`
0.0 alone (expected: no climb, the ring ~0.16 m low, LAND completes).

## Critic (R6b, 24 Oct ~01:30, before any R6b run; pasted by the integrator)
- **Already tried.** Under R7b, `b_z = fz_ki*dt * (∫a_cmd,z dt - (v_z - v_z0))`: R6b is R7 restricted to z, fed to thrust, at
  an effective 0.2 /s (R7's kv 3), bounded; §3.4 (a) rejected R7 for its common-mode wind-up along the rods (mostly
  vertical at 60 deg); reference.md §6's per-drone integrator negative is its parent. The docstring's unit for fz_ki is
  1/s, and its gain depends on the planner tick.
- **Simplest unaddressed explanation: the error is a known constant.** The pull model's vertical part is
  (m_L/n + rod_mass) g at any cone angle; with rod 0.055 it over-counts exactly 0.055 g per quad, and the bumpless start
  already seeds b(0) = -0.98 m/s^2: the whole correction. The offline table cannot separate the integral from that
  constant. The untested control: `quan_fz autopilot, quan_fz_ki 0.0, quan_rod_mass 0.055` (the harness cannot run ki 0:
  network_tune.py l.139-140, a one-line fix). The 2.6 Hz wobble is untouched by R6b (the katt8 neighbour probes it).
- **Falsifier insufficient.** ±0.10 m and load down test the rod mass and the seeded constant, would pass at ki 0, and
  change two variables vs R2793. b_z is logged only on the hand-over line. Add, frozen before the run: (a) b_z(0) per quad
  in [-1.2, -0.75]; (b) no b_z at ±fz_max, LAND included; (c) per quad, b_z over the hold within ±0.3 of b(0), no monotonic
  trend; (d) the differential (b0 - b1 + b2 - b3)/4 under 0.2 and not ramping; (e) rod elevation 60 ± 2 deg; (f) every quad
  landed and disarmed, no fault line.
- **Scope.** Point-load worlds only: the negative's harm is a differential tension acting as a moment through a lever;
  world C's 1 mm hub has none; on the rim, the hub or the rig R6b IS the vel_ki negative. Refuse `quan_fz autopilot`
  unless attach_z / pivot_offset_z are 0 and the hub radius ~0; off world C it must be common-mode and needs a new
  critic. Q4 (tension balance) becomes partly our integrator's property: its row must name R6b.
- **Not asked for**: an overnight addition, a new reading: Wesley's word; if the ki-0 constant matches it, record it
  and delete the code (the working rules).
- **The four questions.** (1) It is the negative's mechanism; on world C the harm path is absent; the four b_z cannot
  diverge freely (each equilibrium needs l_i = l_i(0)), but the restoring loop runs through soft springs (k_d k_ij ~0.13
  N/m per edge): slow and weakly damped, so (c)/(d) must be measured. A shared b_z is safer with a lever but cannot
  correct the per-quad offsets, needs shared state (less faithful: each of their robots runs its own PX4). (2) A
  stand-in addition, not the paper's: the paper puts the true f_i inside u_i (S2 l.351-352, 396) and does not say how
  it is cancelled; Theorem 1 needs it exact at every instant. Wording: "...vertically only, integrates the residual
  velocity error as a PX4 velocity integral would, bounded and started on the thrust our planner flew before the
  hand-over. This integral is ours, not the paper's." (3) rod 0.055 is exact HORIZONTALLY (moments about the quad's
  ball: 0.27 g cot φ = (0.215 + 0.055) g cot φ); vertically the right value is 0 (drone_mass holds the rod). A one-line
  split of R6's pull, vertical (m_L/n) g and horizontal (m_L/n + r) g cot φ, gives both without an estimator (exact for
  the twin's SDF split, a guess physically). (4) Not enough (above); a line 0.4 with R6b once before the cells.
- **Before it flies:** log b_z per quad per tick; add (a)-(f); fly the ki-0 constant beside it (config-only); restrict
  R6b to point-load geometry. Optional: an offline fz_ki margin sweep (10, 20).

CRITIC: rewrite the card. FIX FIRST: log b_z per quad, falsifiers (a)-(f), the ki-0 constant beside it, R6b restricted
to point-load geometry.

## Results so far (EXPLORATORY, overnight 23-24 Oct; bars PROPOSED and not set by these runs; rows R2791-R2812)
World C (cables on a 1 mm hub at the ring's CoM, the joint at the drone centre; 4 quads, 1 m), the twin, once each unless
noted. The law = Eq. 3, 7-10 as read (algebraic nodes, R7b), c_net_quan.yaml's offline low point, then the changes below.

| Run | Arm / reading | Case | Error (mm) | Height held | Quad tilt | Outcome |
|---|---|---|---|---|---|---|
| R2791 | our planner | hover | 1.0 RMS | on target | <= 11.8 | valid |
| R2806, R2807 | our planner | line 0.2 / 0.4 | 24.3 / 65.4 headline | | | valid |
| R2792 | the port (code-based network) | hover | 12.3 RMS | on target | | valid, yaw 0.1 deg |
| R2810, R2811 | the port | line 0.2 / 0.4 | 40.8 / 100.0 headline (lag 1.24 / 1.13 s) | | | valid |
| R2793 | the law, rod 0.075, k_att 16 | hover | 466 RMS | +0.45 m (climb) | 27-28 peaks (2.6 Hz wobble) | held, off; LAND stalled at 0.52 m |
| R2794 | rod 0 (the double count removed) | hover | 316 | -0.27 m | wobble | band LOSS (drift), landed |
| R2795 | rod 0, k_att 8 | hover | 181 | -0.16 m | 9.4 mean, no wobble | **valid**, landed |
| R2803, R2804 | rod 0, k_att 8 | line 0.2 / 0.4 | 282 / 299 (overshoot 80 / 113) | | <= 12 | valid |
| R2802 | rod 0.055 + the seeded constant, k_att 16 | hover | 194 | -0.11 m | wobble | band LOSS (drift), landed |
| R2805, R2812 | rod 0.055 + the seeded constant, k_att 8 | hover | 110 / 118 | -3.6 / -3.7 mm | <= 11.7 | **valid x2**, landed; a settled 110-127 mm sideways offset |
| R2808, R2809 | rod 0.055 + constant, k_att 8 | line 0.2 / 0.4 | 202 / 253 (overshoot 102 / 175; clock-free 42 / 39) | | <= 14.7 | valid / band LOSS at the ends (calm quads) |

**What was found:**
- The R2793 climb was the rod's mass counted twice (the twin's drone_mass 0.55 already holds rod + magnet); the 2.6 Hz
  quad wobble was vel_k_att 16 (gone at 8). Both confounded the paper's damping-form losses on the rim and the top-face hub
  (R2744-R2749, R2780-R2781, all flown at vel_k_att 16).
- At rod 0.055 (exact horizontally) with the vertical over-count removed by a constant seeded at the hand-over (the R6b
  code path at fz_ki 0: no estimator), the law holds the ring in hover at the right height, reproducibly, with a settled
  ~0.11 m sideways offset (R3: the path point as a rigid leader through soft springs, as the critic predicted).
- On the lines it tracks with little lag but OVERSHOOTS each end (amplitude 1.29-1.42); at 0.4 m/s past the 300 mm band.
- Against the paper's own accuracy (Fig. 5A: ~0.8 cable lengths median, a clock error), 0.1-0.25 m on 1 m cables is
  well inside it; against our planner (1-65 mm) and the port (12-100 mm) it is the least accurate arm on world C.

**Open (Wesley):** R6b's integral is not flown (its critic: fix first, and not asked for); the constant's reading (or the
critic's split of R6's pull into vertical (m_L/n) g and horizontal (m_L/n + r) g cot phi, which gives the same without the
OCP seed) needs his word as a §3.1 row; vel_k_att 8 for the law's arm (a gain of the stand-in autopilot, not the law);
the bars; the line overshoot (R5's a_expected, R3's leader, or the gains: not separated).

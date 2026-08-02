# Cable-Suspended Load Control — Progress & Direction

**Status as of this document:** takeoff and the planner→tracker handoff are working in
stages; the open question is whether the coupled planner can *hold* a taut hover (see
§8 "Current status"). This file documents what was built, every bug overcome, the design
choices, similarity to the reference paper, and next steps.

Reference paper: **Sun et al. 2025, "Agile and Cooperative Aerial Manipulation of a
Cable-Suspended Load" (arXiv 2501.18802)**. Local PDF in the repo root.

---

## 1. Goal

Replicate the paper's **high-level controller**: a centralized kinodynamic motion planner
that solves the full coupled load+cable+drone OCP and emits per-drone reference
trajectories, tracked by an onboard controller. We **keep MPC at the lower level** instead
of the paper's INDI (unless a limitation appears that only INDI overcomes). Simulation
(Gazebo Harmonic) mocap ground truth currently stands in for the paper's EKF.

3 quadrotors (Betaflight, 0.6 kg each) carry a 0.4 kg payload via soft segmented cables in
`simulation_assets/three_soft.sdf`.

---

## 2. System architecture (as built)

Two ROS 2 packages plus the existing sim bridge:

| Package | Role | Rate |
|---|---|---|
| `controller_load_mpc` (`planner` node) | Centralized coupled OCP → per-drone references | 10 Hz |
| `controller_quad_load` (`controller` node) | Per-drone **cable-aware** MPC tracker (NEW) | **50 Hz** |
| `controller_mpc_multi` | Original **cable-blind** tracker, kept for the no-cable case | 30 Hz |

- **Planner** (`controller_load_mpc`): acados OCP over the TU Delft model
  (`load_cable_dynamics.py`, `LoadCableDynamics`). State `x = [p,v,q,ω] + per-drone
  [s_i, r_i, ṙ_i, r̈_i, t_i, ṫ_i]` (nx = 13+14n); control `u = per-drone [γ_i, λ_i]`
  (nu = 4n). `s_i` points **drone→load**. SQP-RTI, `PARTIAL_CONDENSING_HPIPM`, N=20, tf=2.0.
- **Tracker** (`controller_quad_load`): per-drone quad MPC (`QuadLoadDynamics`, nx=17,
  state `[p, q, v, ω, u_state]`, `u_state=[roll_rate,pitch_rate,throttle,yaw_rate]`).
  Identical to `controller_mpc_multi` **plus a cable-acceleration term** in `v_dynamics`.
  Distinct solver identity so it never clobbers the cable-blind one: model
  `quad_load_dynamics`, export dir `c_generated_code_quad_load/`, json
  `quad_load_dynamics_ocp.json`.

### The cable-aware model term (the core addition)
The quad model works in **acceleration units** (no explicit mass; thrust = `thrust_ratio·throttle`
along body-z). The cable force on a drone is `t_i·s_i` (toward the load), so we add the
**cable acceleration** as a stage parameter:

```
v̇ = R·[0,0,kT·throttle] − g + drag + a_cable ,   a_cable = t_i·s_i / m_i  (world frame)
```

This makes the MPC's *predictions* match reality so its feedback stops fighting the cable
(previously it only had the cable in the cost reference, not the model).

---

## 3. Reference handoff format

Planner publishes `/drone_{i}/reference_trajectory` (`Float64MultiArray`), **12 fields/node**:

```
[n_nodes, dt,  px,py,pz, vx,vy,vz, ax,ay,az, cx,cy,cz, ...]
```
- `a = a_thrust = thrust_vec/m_i` — required **specific thrust acceleration**. Magnitude →
  tracker throttle (`throttle = ‖a‖/thrust_ratio`), direction → tracker's yaw-free tilt
  attitude reference. (Paper "piece 1": full quad-state feedforward.)
- `c = a_cable = t_i·s_i/m_i` — cable acceleration fed into the tracker **model**.

The cable-blind `controller_mpc_multi` parses the first 9 fields and ignores `c` (a 1-line
defensive change is the only edit to that package).

---

## 4. Two-phase takeoff (current scheme)

The coupled OCP assumes **taut** cables, but the Gazebo cables spawn **slack** (drone must
climb ~0.4 m to take up slack). Running the lift planner through that snap is unstable, so:

- **Phase 1 — creep:** each drone rises straight up, **xy anchored to its spawn position**,
  z ramped by an accumulator, level attitude, **no cable force**. Independent per drone →
  cables go taut slowly. The climb accumulator is **gated on real liftoff** so it can't run
  away while the drones sit disarmed pre-TAKEOFF.
- **Handover** when every cable reaches `gate ≥ 0.95` (≈ taut): hard-reconverge the OCP from
  the current state, latch the lift-ramp start height, then run the coupled planner.

---

## 5. Bugs & limitations overcome (chronological)

| # | Symptom | Root cause | Fix |
|---|---|---|---|
| 1 | Drones tilt away & fall once cables taut (cable-blind) | Handoff dropped attitude+thrust feedforward (attitude=level, thrust=0) | Publish `a_thrust`; tracker sets throttle + yaw-free tilt attitude (paper piece 1) |
| 2 | "Flew up too fast & crashed" | Planner commanded a big step from the low payload spawn | Gentle/self-paced takeoff (later replaced, see #7) |
| 3 | acados recompiled for every drone | mtime cache checked wrong (outer) path | mtime freshness check on the **nested** `c_generated_code/...so`; fcntl compile lock |
| 4 | New tracker would clobber the old solver | Shared model name / export dir | Separate `quad_load_dynamics` model, `c_generated_code_quad_load/`, own json |
| 5 | Cable force in cost only, not model → feedback fights cable | Tracker model had no cable | Add `a_cable` term to `v_dynamics` as a stage parameter |
| 6 | Over-tension at slack→taut snap; drones overpowered, crash | Rigid-taut feedforward fed while cable physically slack | **Tautness gate**: scale `a_cable` by measured drone→attach distance (0 slack → 1 taut) |
| 7 | Planner diverges online: `\|aT\|` swings 27↔3, then `status 4`/NaN | 1 RTI iteration can't track the stiff transition; bad solve poisons warm start | `STEADY_ITERS=5` SQP iters/cycle; on failure hold last ref + reseed from `x_init` |
| 8 | Monotonic thrust collapse after handover → sinks into ground | **Positive feedback**: `z_target = load_z + LIFT_STEP` chased the load down | Time-based lift ramp (`LIFT_RAMP_VEL`) **decoupled** from measured `load_z` |
| 9 | Tilt winds up to ±0.97 even during gentle creep | Creep xy reference chased the *live* position → no horizontal hold; inward cable pull accumulated | Anchor creep xy to **spawn** position (real position-hold); raise rate 30→**50 Hz** for bandwidth |
| 10 | Creep reference at 1.14 m on takeoff → rocket up & overshoot | Climb accumulator ran from planner launch, not takeoff (depended on pre-takeoff wait) | Gate the climb ramp on **real liftoff**; hold a small fixed lead while grounded |

---

## 6. Key design choices

- **Keep MPC at the lower level** (explicit goal), with the planner's thrust vector carried
  as attitude+throttle feedforward — the model-side equivalent of the paper's piece 1.
- **Sim mocap = ground truth** instead of an EKF (the original EKF step was deferred; we have
  the load pose directly in sim). An EKF is a later real-world step.
- **Tension source = the planner's OCP solution** (`t_i`, `s_i` over the horizon), not an IMU
  estimate — readily available, no new sensing. IMU-measured tension is a deferred refinement.
- **New parallel package** `controller_quad_load` rather than modifying `controller_mpc_multi`
  (kept cable-free for the no-cable case).
- **Two-phase takeoff** chosen over more cost-weight tuning once tuning was exhausted, to
  avoid exciting the soft-cable spring at the slack→taut snap.
- **Tautness gate** so feedforward/model engage only as the cable physically tautens.

---

## 7. Diagnostic learnings (carry these into the next chat)

- **The tracker tracks well** (`ez` stayed small throughout). Every failure was in the
  *reference* (planner) or the takeoff transient — not the tracker's tracking.
- **Thrust authority:** drone max thrust accel ≈ `0.6·38 = 22.8 m/s²`; minus gravity leaves
  **~13 m/s²** to fight the cable. When `|aC|` (cable accel on a drone) exceeds ~13, the drone
  is **physically overpowered** → it falls regardless of throttle. Keep planned tension within
  this.
- **Consistency:** payload mass 0.4 = planner `LOAD_MASS`; drone 0.6 = `DRONE_MASS`;
  `thrust_ratio=38` is the same calibration the single-drone MPC already flies with.
- **Soft-cable snap** (segmented, low joint damping ~0.01) vs the planner's **ideal rigid
  taut** model is the central model mismatch and the source of most takeoff trouble.
- **Monotonic** thrust collapse = a feedback dynamic (positive feedback); **swinging** `|aT|`
  = an unconverged/diverging solve. Useful fingerprints when reading the diag logs.
- Diagnostic logging is in place: tracker `[diag dN] z zref ez thr roll pitch |aT| |aC|`,
  planner `[planner cable]` / `[planner creep]` lines.

---

## 8. Current status & the pending decisive test

Last fixes in place but **not yet verified in sim**: creep-runaway gate (#10) + time-based
lift ramp (#8). The next run's decisive signal is the **post-handover `|aT|` column**:

- **Holds ≈12** and load climbs toward 0.6 → the planner-hold instability was the
  positive-feedback target; the system hovers. **Proceed to trajectory tracking.**
- **Still decays to ~3** → instability is *inside* the coupled 10 Hz planner loop. Conclusion:
  the planner cannot be the stabilizing loop. Move to the architectural change in §9.1.

---

## 8b. Outward-drift diagnosis & feedforward-correction hypothesis (2026-07-01)

**Observation (invariant across every test):** during the lift the drones tilt/fall *away*
from the payload. Log archaeology shows the radial distance `r` grows (0.5 → ~0.7) while each
drone-to-attach distance pins at the cable length (0.6) — i.e. the drones slide **down and
outward along the taut-cable sphere**. This is *identical* in logs taken before any
cable-force-model change, so the failure is **independent of the cable-force estimator** — it
lives in code that was never the suspect (the thrust feedforward / acados cost), which
overturns §8's framing that "the tracker tracks well, the problem is the planner."

**Computed mechanism (hypothesis, not yet verified in sim):** the thrust feedforward commands
`a_thrust = [0,0,g]` → an **unloaded, level** hover (`_tilt_quat_from_accel([0,0,g], 38)`
gives throttle ≈ 0.258, identity/level attitude). But the loaded equilibrium at r=0.5 (≈44°
cables) needs throttle ≈ 0.32 **and ≈10° outward tilt** to support the load's share and pull
in. The acados cost (throttle weight 0.3, attitude weight 0.5) then holds the drone near the
*wrong* feedforward → chronic under-throttle + under-tilt → it loses altitude and slides
outward. Invariant to cable-model changes because none of them corrected the thrust FF.

**Potential solution (INDI-style thrust FF correction):** feed the *loaded* equilibrium into
the feedforward using the measured cable acceleration (§9.3's IMU estimate):

```
a_ff = a_planned − a_cable_measured + a_cable_planned      # during creep: [0,0,g] − a_cable_meas
```

This raises the commanded throttle and adds the outward tilt the equilibrium requires, instead
of pinning the tracker to an unloaded hover. Prototyped in this session but **reverted** (code
returned to last commit) pending a clean implementation.

**Decisive confirming experiment (do this to split the hypothesis before trusting the fix):**
regenerate the world with **near-vertical cables** (horizontal tension ≈ 0):
`python3 simulation_assets/generate_soft_world.py --n 3 --r-drone 0.15 --out simulation_assets/three_soft.sdf`
(planner reads spawn positions from mocap, so no code change needed; revert with `--r-drone 0.5`).
- Drones **hold and lift** → horizontal cable tension / loaded-hover FF was the cause → the
  correction above is the right track.
- Drones **still drift out** with ≈no horizontal force → it is **not** the cable tension.
  Next suspect is the inner loop: the MPC→Betaflight rate command path, the throttle slew-rate
  limit (`throttle_dot ∈ [−1,1]`), or a frame error in the rate mapping — none yet audited.

---

## 8c. Geometry mismatch found + §8b refuted (via `three_soft_paper.sdf`)

Switched to `simulation_assets/three_soft_paper.sdf` — drones spawn **elevated & taut**
(1.0 m cables at ~45° from vertical), which removes the slack→taut snap and isolates the
steady taut-hover problem. **Blocking bug found:** the planner geometry was hard-coded for
`three_soft.sdf` (`CABLE_LEN=0.6`) while the paper world has **1.0 m** cables. With the
wrong length the kinematic constraint `p_i = p + R·ρ_i − l_i·s_i` places every drone
reference ~0.4 m *inside* the physical cable sphere, and the tautness gate saturates — so
every reference in that world was invalid, upstream of any feedforward question.

**Fix (done):** planner geometry is now ROS params — `cable_len`, `attach_radius`,
`attach_z`, and `start_taut` (skip creep, enter the coupled planner on cycle 0 for taut
worlds). The quad-load launch declares `cable_len:=1.0 start_taut:=true` by default (paper
world); pass `cable_len:=0.6 start_taut:=false` for `three_soft.sdf`.

**Offline equilibrium check** (`controller_load_mpc/diag_equilibrium.py`, no Gazebo) with
the corrected geometry: OCP **status 0**, `t_i=1.85 N` (= analytic), drone refs consistent
(residual 0), and feedforward **loaded** — `throttle 0.321, tilt 10.3° outward` vs the
unloaded `0.258, 0°`. **This refutes §8b for the planner phase:** the unloaded `[0,0,g]`
FF was a *creep-phase* artifact; the coupled planner already commands the loaded tilt +
throttle. `|aC|=3.1 m/s²` per drone, well under the ~13 authority ceiling. The live test
below is now the pending signal.

## 9. Next steps (toward replicating the paper)

### 9.1 If the planner-hold is still unstable — make the planner open-loop, tracker stabilizes
The paper's planner is a **trajectory generator**; the high-rate tracker does the
stabilizing. Mirror that: have the planner emit a **smooth, slowly-updated** reference
(don't chase noisy mocap every 100 ms; resample its own previous solution as the paper does)
and let the **50 Hz (→100 Hz) tracker** hold the system. This decouples the nested loops that
are currently fighting.

### 9.2 Push tracker rate toward the paper's regime
Paper tracker ≥100 Hz (INDI inner loop 300 Hz). We are at 50 Hz. Watch the acados "Solve
time" log; if it fits, push to 80–100 Hz for more disturbance-rejection bandwidth against the
cable.

### 9.3 IMU-measured tension (paper's `f_ext`) — the principled snap fix
Close the loop on the **actual** cable force each drone feels:
`f_ext = a_meas − T_modeled` from the bridged IMU (the Gazebo drones already have IMUs to
bridge). Robust to the rigid-vs-soft model mismatch because it reacts to the real force at
the moment of the snap. This is the one place INDI's mechanism may beat open-loop MPC
feedforward; can be folded into the MPC as a measured external-force term rather than a full
INDI controller.

### 9.4 EKF load-cable state estimator (deferred step 1)
For real-world transfer / removing mocap dependence: paper's EKF estimates load pose+twist +
cable directions from quadrotor states + accelerometers (cable direction from
`s̃ = (a_m − T̄ − D̄)/‖·‖`); cable rates/tensions come from **resampling the previous OCP
solution**, not the EKF. (Notes already gathered in earlier discussion.)

### 9.5 Once hover holds — the actual demo
Trajectory tracking (figure-eight, the paper's benchmark) via the load pose reference. Then
tune for agility (velocity/accel/jerk) per the paper's Table 1.

### 9.6 Secondary tuning levers
- Planner `thrust_max` is 15 N but the real per-drone useful max is ~13.7 N — align them so
  the planner only plans feasible configs.
- Increase Gazebo cable joint damping (currently ~0.01) to reduce soft-cable ringing.
- `CABLE_TAUT_LO_FRAC` / `TAUT_SWITCH_GATE` to tune when feedforward/handover engage.

---

## 10. Key files & constants

**`controller_load_mpc/controller_load_mpc/`**
- `load_cable_dynamics.py` — `LoadCableDynamics`; `thrust_vec(i)`, `cable_accel(i)`.
- `planner_ocp.py` — `generate_load_ocp`. Weights: `w_pose=[60,60,80, 18,18,22, 30,30,30, 1,1,1]`
  (velocity raised to damp climb), `w_t=0.05`, `w_r=3.0`, `w_u=[1e-3,1e-3,1e-3,5e-3]`.
  Bounds: thrust [0.5,15], tension [0.1,25], `γ`≤50, `λ`≤80. `levenberg_marquardt=1e-2`.
- `planner_node.py` — constants: `N_DRONES=3`, `LOAD_MASS=0.4`, `CABLE_LEN=0.6`,
  `ATTACH_RADIUS=0.08`, `DRONE_MASS=0.6`, `PLANNER_HZ=10`, `STEADY_ITERS=5`, `CREEP_VEL=0.10`,
  `CREEP_LEAD=0.10`, `LIFTOFF_MARGIN=0.05`, `TAUT_SWITCH_GATE=0.95`, `TARGET_Z=0.6`,
  `LIFT_RAMP_VEL=0.05`, `CABLE_TAUT_LO_FRAC=0.85`. (`LIFT_STEP` now vestigial.)

**`controller_quad_load/controller_quad_load/`**
- `dynamics.py` — `QuadLoadDynamics`: `p_param` length 9 (6 dyn + 3 `a_cable`); cable term in
  `v_dynamics`.
- `acados.py` — model `quad_load_dynamics`; `model.p = [dyn(6), a_cable(3), q_ref(4)]` (13);
  `set_planner_reference(..., ref_cable=...)`. Throttle cost weight 0.3; throttle bounds
  [0.05, 0.6].
- `controller_mpc.py` — `FREQUENCY_HZ=50.0`; `est_params=[24.0, 0.0, 0.12, 70.0, 670.0, 0.5]`;
  parses 12 fields/node → `planner_ref_cable`; `ACADOS_DIR=c_generated_code_quad_load`.
- `launch/mpc_three_soft_quad_load_launch.py` — planner + 3 cable-aware trackers + fleet mgr.

---

## 11. How to run

```bash
colcon build --packages-select controller_quad_load controller_load_mpc \
  controller_mpc_multi interfaces utility_objects --symlink-install
source install/setup.bash

gz sim simulation_assets/three_soft.sdf -v 4 -r
ros2 launch controller_quad_load mpc_three_soft_quad_load_launch.py
ros2 topic pub --once /fleet/command std_msgs/msg/String "{data: ARM}"
ros2 topic pub --once /fleet/command std_msgs/msg/String "{data: TAKEOFF}"
```

Read the diag stream: tracker `[diag dN] ... |aT| |aC|` and planner
`[planner creep]` / `[planner cable]`. Success = `ez` small during creep, clean handover,
`|aT|≈12` held after handover, load to z≈0.6.
```
```

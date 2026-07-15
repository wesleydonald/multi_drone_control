# Cable-Suspended Load Control — Plan & Handoff

**Purpose.** This is the authoritative handoff/plan doc for the payload-control work. A
fresh chat should read this file first and can then resume with full context. It captures
the resolved goal, the corrected architecture decision, the current code state, a map of
every important file (with line numbers), the staged plan, and the environment gotchas
needed to actually run and test in this repo.

Last updated by the planning session on 2026-07-14.

---

## 1. The goal (resolved with the user)

Long-term vision: two drones fly a cable-suspended payload on a circular trajectory; a
**third drone (flown by someone else's controller) attaches to the payload mid-motion**;
this controller then takes over and **reconfigures from a 2-drone to a 3-drone carry**.

Three decisions were made that pin the near-term scope:

| Decision | Answer | Consequence |
|---|---|---|
| **Thesis contribution** | The **2→3 mid-motion reconfiguration** is the novelty (NOT reproducing the paper's agility) | Only need a *reliable slow* baseline. **Do not fight the coupled OCP / agility.** |
| **Environment** | **Sim only** for now (ground-truth mocap) | Skip EKF / model-ID / delay handling. |
| **DOF scope** | **Position first (3-DOF)**, orientation passive | No direction-allocation QP or SO(3) error yet. |

**Immediate target:** a reliable, closed-loop, **N-reconfigurable 3-DOF payload position
controller**, built as a new layer that feeds the existing cable-aware tracker
(`controller_quad_load`), so the 2→3 handoff has a stable base.

---

## 2. Architecture — the key correction

The paper in the repo root — *"Agile and Cooperative Aerial Manipulation of a
Cable-Suspended Load"*, Sun et al., arXiv 2501.18802v2 (`./Agile and Cooperative Aerial
Manipulation of a Cable-Suspended Load - 2501.18802v2.pdf`) — is frequently mis-summarized.
The correct reading:

- **The paper's method = option A: a coupled online kinodynamic planner (whole-body OCP,
  receding horizon) + a per-drone tracker that compensates cable tension.** The paper
  *explicitly rejects* the cascaded/hierarchical force-based structure — it names that
  cascade as **Challenge #1**, the reason existing methods can't be agile ("outer-loop
  commands can easily exceed the bandwidth of the inner loops, leading to instability").
- **Option B = hierarchical: load-PD → desired wrench → per-cable tension/direction
  allocation → per-drone tracker.** This is the *pre-paper baseline* (the works the paper
  beats, refs 12/16/24), capped at low speed (**<1.5 m/s, <0.5 m/s²**). It is **NOT** "the
  paper's structure" — a previous planning chat claimed this; it is wrong.

**Chosen route: B (hierarchical), used deliberately as a slow, debuggable foundation for
the reconfiguration work.** Because reconfiguration (not agility) is the contribution, B is
the right call and the coupled OCP's divergence is a distraction to be avoided, not fixed.

This repo **already implements the paper's architecture (A):** a centralized planner
(`controller_load_mpc`) + a cable-aware tracker (`controller_quad_load`). The coupled OCP
mode exists but **"overshot & diverged"** (documented in code) — hence a `kinematic`
open-loop fallback exists. Our new work adds a **third planner mode**: a closed-loop
hierarchical *position* controller. We do **not** modify the coupled OCP.

---

## 3. Current system state

- **Planner** (`controller_load_mpc`) has two modes today (param `planner_mode`, default
  `'coupled'`):
  - `coupled` — solves the full load-cable OCP online each cycle. **Unstable (diverges).**
  - `kinematic` — OPEN-LOOP feedforward: rigidly translates a latched taut config and
    streams an analytic loaded-hover feedforward. Stable but **no feedback on load pose**,
    so any load error (sag, tilt, lag, missing centripetal FF) is never corrected.
  - We will add a `geometric` (closed-loop position) mode — see the plan.
- **Two trackers exist:**
  - `controller_mpc_multi` — **cable-BLIND**. Does not model the payload load → with a real
    suspended mass it **under-commands thrust and the drones sag / won't lift**. (Verified:
    on `mpc_three_soft_planner_launch.py` the ARM/TAKEOFF commands work end-to-end but the
    drones don't lift because this tracker is cable-blind.)
  - `controller_quad_load` — **cable-AWARE**. Adds the cable acceleration `t_i·s_i/m_i` to
    its MPC prediction model. **Use this for load-control work.** Launch:
    `mpc_three_soft_quad_load_launch.py`.
- **Load feedback:** none end-to-end yet. Load pose is available from mocap
  (`/payload/motion_capture_state`) but the planner does not close a loop on it.
- The tracker-side half of paper-fidelity (**IMU-measured cable tension / f_ext, no load
  sensors**) is partly built already ("Part 2" work in `controller_quad_load`). Reuse it;
  don't re-derive an EKF from scratch.

---

## 4. Important files (with line anchors)

### Planner — `src/controller_load_mpc/controller_load_mpc/`
- **`planner_node.py`** (664 lines) — the centralized planner ROS node. Key anchors:
  - `43  N_DRONES = 3` — fleet size baked in at module scope.
  - `48  ATTACH_Z = 0.025`, `~46 ATTACH_RADIUS` — payload attach geometry (load frame).
  - `50  PLANNER_HZ = 10.0` — plan/publish rate (`236` creates the timer).
  - `56  TAUT_SWITCH_GATE = 0.95` — creep→coupled handover threshold (`319` uses it).
  - `102–105` — `attach_radius` / `attach_z` ROS params (override per world).
  - `116–124  planner_mode` param (default `'coupled'`; also `'kinematic'`).
  - `133  ff_gate_mode` (default `'airborne'`), `145 load_traj` (default `'hover'`).
  - `166  self.n = N_DRONES`, `171  self.dyn = LoadCableDynamics(self.n, …)` — **N is fixed
    at construction; this is what must become runtime-reconfigurable.**
  - `282  t_nom = m·g / (n·sin(45°))` — coupled-path nominal tension (equal split).
  - `458  _enter_planner_phase(reason)` — **latches the config once** at takeoff:
    `468–480` builds `kin_drone0`, `kin_acable`, `kin_athrust`.
  - `477  t_i = m·g / (n·sin_elev)` — **kinematic equal-split tension. Replace with a
    per-cable QP for N∈{2,3}.**
  - `485  _publish_kinematic_refs(lift_vel)` — streams the per-drone references.
  - `511  a_i = (1-gate)*a_unloaded + gate*athrust` — **taut-gate blend; reuse this exact
    mechanism for the smooth 2→3 tension hand-in.**
  - `586  _cable_taut_gate(i)` — per-cable slack→taut gate (drone→attach distance vs cable
    length). Reuse for the hand-in ramp.
  - Kinematic constraint (docstring `14`): `p_i = p + R(q)·rho_i − l_i·s_i` — how a load
    pose maps to each drone's desired position. Reuse in the new mode's per-drone step.
- **`planner_ocp.py`** — the coupled load-cable OCP (the diverging mode). **Leave alone.**
- **`load_cable_dynamics.py`** — `LoadCableDynamics(n, …)`: load mass `m`, gravity `g`,
  cable helpers. Source of `self.dyn.m`, `self.dyn.g`.
- **`diag_equilibrium.py`** — equilibrium diagnostics helper (useful for the 2-drone
  asymmetric static check in Stage 4).

### Cable-aware tracker — `src/controller_quad_load/controller_quad_load/`
- **`controller_mpc.py`** — per-drone cable-aware MPC. Key anchors:
  - `207` subscribes `/drone_{id}/reference_trajectory`.
  - `299–312` parses the wire format (see §6). N=20 horizon, DT ~ per node.
- `acados.py`, `dynamics.py` — the OCP/model (adds `t_i·s_i/m_i` cable accel to `v̇`).
- `main.py` — self-contained fleet manager variant; `trajectories.py`, `simulator.py`.
- Launch: **`src/controller_quad_load/launch/mpc_three_soft_quad_load_launch.py`**.

### Cable-blind tracker + fleet manager — `src/controller_mpc_multi/`
- `controller_mpc_multi/controller_mpc.py` — cable-blind tracker (per-drone MPC).
- `controller_mpc_multi/main.py` — **central fleet manager**: owns `/fleet/command`
  (ARM | TAKEOFF | DISARM | ESTOP) and broadcasts `/fleet/step` at 30 Hz.
- Launches: `mpc_three_soft_planner_launch.py` (planner + **cable-blind** tracker —
  under-thrusts a load), `mpc_three_soft_launch.py`, `two_sim_io_launch.py` /
  `two_sim_control_launch.py` (the split sim I/O + control + RViz stack).

### Sim I/O — `src/simulation_communication/simulation_communication/`
- `payload_mocap_emulator.py` — Gazebo pose → `/drone_{id}/motion_capture_state` and
  `/payload/motion_capture_state`. Has an optional `broadcast_tf` param (default off) that
  publishes `map → drone_{id}/base_link` TF for RViz.
- `payload_betaflight_comm.py` — `/drone_{id}/ELRSCommand` → Gazebo motor_speed.

### Worlds — `simulation_assets/`
- Nested-model (parent `lift_system`, drones `x3_drone0/1/2`): `two_rigid.sdf`,
  `three_rigid.sdf`, `three_rigid_short.sdf`, `three_soft.sdf`, `three_soft_paper.sdf`,
  `two_soft.sdf`, `four_*`, `two_no_cables.sdf`, etc.
- **Rigid** cables = the stable config (use for early stages). **Soft** = multi-segment.
- ⚠️ **Worlds must match the planner geometry.** `three_soft_paper.sdf` spawns drones at
  radius ≈0.787 m / z≈0.757 m with the payload on the ground, but the planner assumes a
  ~0.4 m formation and `cable_len=0.6` — they don't line up, so its references are wrong
  there. `mpc_three_soft_planner_launch.py` is written for `three_soft.sdf`.

### Paper
- `./Agile and Cooperative Aerial Manipulation of a Cable-Suspended Load - 2501.18802v2.pdf`

---

## 5. The staged plan (each stage independently testable)

**Route B, position-first, sim-only. Build on `controller_quad_load` (cable-aware).**

- **Stage 1 — Load-state pipeline (no-regret, do first). ✅ DONE (2026-07-14).** Publishes
  `/payload/state`: position + linear velocity + the N cable directions `s_i`, from the mocap
  already bridged (`payload_mocap_emulator` / `/payload/motion_capture_state`).
  - New node `controller_load_mpc/load_state_node.py` (entry point `load_state`), added to
    `mpc_three_soft_quad_load_launch.py`. Params: `n_drones`, `attach_radius`, `attach_z`
    (kept in sync with the planner geometry).
  - `s_i = (attach_i − p_i)/‖·‖`, `attach_i = p + R(q)·rho_i` — **same convention as the
    planner's `_build_x_init` (planner_node.py:270)**, so downstream closed-loop code reuses
    it without re-deriving geometry.
  - **Wire format — `Float64MultiArray` on `/payload/state`:**
    `[n_drones, px,py,pz, vx,vy,vz, s0x,s0y,s0z, s1x,s1y,s1z, …]` (3 per drone;
    len = 1 + 3 + 3 + 3·n).
  - Verified end-to-end on `three_soft.sdf`: publishes ~290 Hz (≫ tracker rate), `n=3`, load
    pos/vel correct, all `‖s_i‖≈1` at 120° spacing. Deliverable met.
- **Stage 2 — Baseline the open-loop error. ✅ DONE (2026-07-15).** Ran 3-drone hover + slow
  circle in `planner_mode:=kinematic` on **`three_soft_paper.sdf`** (elevated taut world) and
  logged load position vs `/payload/desired_position`.
  - **Known-good launch config** (the committed HEAD launch defaults, NOT the current
    uncommitted coupled/circle edits): `planner_mode:=kinematic ff_gate_mode:=airborne
    start_taut:=true cable_len:=1.0 target_z:=0.6 lift_ramp_vel:=0.05` on `three_soft_paper.sdf`.
    Geometry lines up with planner defaults (attach_radius=0.08, attach_z=0.025 →
    drone_radius=0.08+1·cos45=0.787, drone_z=0.05+1·sin45=0.757).
  - **Result: open-loop kinematic is STABLE/bounded** (matches the doc's prediction). Steady
    **lateral (xy) load-position error ≈ 1.05–1.10 m** (hover: x≈+0.98, y≈−0.51; circle: x≈+0.92,
    y≈+0.51), constant, no divergence. **This is the number Stage 3 must beat.**
  - **z-error (~1.08 m high) is NOT a control error** — it's the sim thrust-ratio being slightly
    low (more accurate in real experiment, per user). Exclude z from the metric; track **xy only**.
  - Artifacts in `baseline_stage2/`: `stage2_openloop_baseline.png`, `hover_openloop.csv`,
    `circle_openloop.csv`, `baseline_plot.py`.
  - ⚠️ **Env gotcha (cost me several runs):** back-to-back launches leak ~15 nodes each and
    exhaust Fast-DDS `/dev/shm` (`RTPS_TRANSPORT_SHM ... open_and_lock_file failed`), which
    starves the pose pipeline → `Pose timeout — disarming` → ESTOP, AND cross-talks duplicate
    mocap publishers → false "divergence". **Always fully kill the prior stack + `rm -f
    /dev/shm/fastrtps_*` between runs** (see `scratchpad/kill_stack.sh`). A contaminated env
    makes a stable controller look unstable.
- **Stage 3 — Closed-loop 3-DOF position controller (fixed N=3). ✅ DONE (2026-07-15).**
  New `planner_mode='geometric'` in `planner_node.py`.
  - **What it does:** reuses the open-loop kinematic taut-config feedforward, but layers a
    PD(+I) correction on the MEASURED load position (`_publish_geometric_refs`). The
    correction is a world-frame SHIFT of the whole formation — `corr = Kp·e + Kd·ė + Ki·∫e`,
    `e = p_set − p_meas` — passed into `_publish_kinematic_refs(lift_vel, corr=…)`. Shifting
    all drones together drags the suspended load to its setpoint while keeping cables ~taut
    (cached FF stays valid). Implemented as a minimal add-on; the coupled OCP is untouched.
  - **Scope note:** the per-cable **tension QP** the plan mentions here is deferred to Stage 5
    (as the plan itself assigns it); Stage 3 keeps the equal-split FF and does the closed-loop
    work through the position PD. This was the right call — it beat the baseline decisively
    without added complexity.
  - **xy closed, z open by default** (`kp_load_z=0`): the sim's residual load-height offset is
    a thrust-ratio artifact (accurate on hw), not a control error — don't fight it.
  - **Gains** (ROS params + launch args): `kp_load_xy=1.5, kd_load_xy=0.6, ki_load_xy=0.3`,
    `i_clamp=4.0, corr_clamp=1.8`. The **integral is the key to nulling the offset**: it must
    have enough authority (i_clamp·ki ≈ 1.2 m) to supply the full formation shift that cancels
    the ~1 m open-loop model-mismatch bias as e→0. (First try with i_clamp=0.6 only got to
    0.31 m; raising it drove e→0.02 m.)
  - **Results (three_soft_paper.sdf, settled lateral xy error):**
    - Hover:  open-loop **1.100 m → closed-loop 0.023 m**  (**48.6×** better)
    - Circle (slow, 0.1 m/s): open-loop ~1.05 m → closed-loop **0.024 m** (~44×), tracks the
      moving setpoint tightly.
  - **Disturbance rejection:** the persistent model mismatch (thrust ratio, cable sag, tracker
    bias) IS a sustained disturbance force on the load — open-loop kinematic drifts to 1.1 m
    and stays (no feedback); closed-loop geometric rejects it to 0.02 m. That is "holds pose +
    rejects a disturbance kinematic can't." A *transient external shove* was NOT achievable in
    this Gazebo build: **no wrench API** (`/world/*/wrench` topic/service absent — every
    ApplyLinkWrench attempt silently no-ops), and **`set_pose` teleport of the payload is
    rejected within one physics step** by the cable constraints (load never moved). If a
    literal shove is needed later, inject it by teleporting/offsetting a DRONE (drags the load
    via its cable) or add a wind/force plugin to the world.
  - Artifacts in `stage3_geometric/`: `stage3_kinematic_vs_geometric.png`, `geometric_hover.csv`,
    `geometric_circle.csv`, `stage3_plot.py`.
  - ⚠️ Relaxed the tracker pose-timeout watchdog `POSE_TIMEOUT_THRESHOLD` 0.25→0.75 s
    (`controller_quad_load/controller_mpc.py`): on a loaded headless box, mocap delivery
    transiently stalls >0.25 s at takeoff and spuriously trips the disarm→ESTOP. Purely a
    test-robustness change; does not affect control.
- **Stage 4 — Prove a stable 2-drone carry (RISK GATE). ✅ DONE (2026-07-15) — BOUNDED, with a
  key caveat.**
  - **Enablers built:** `N_DRONES` is now a runtime param (`n_drones`, `planner_node.py:185`);
    everything derives from `self.n`. Generated an elevated 2-drone world matching the planner
    (`simulation_assets/two_rigid_paper.sdf`, via `generate_rigid_world.py --n 2`): drones at
    (±0.787, 0, 0.757), load centred, 45° cables, attach points at 0/180° (opposite — the
    balanced phase-1 geometry). New launch `mpc_two_quad_load_launch.py`. Pre-built the n=2
    acados OCP (`load_cable_2`).
  - **Result (geometric, default gains 1.5/0.6/0.3, rigid world):** the carry stays **bounded**
    (no divergence) — risk gate passes the literal deliverable. BUT the two DOFs behave very
    differently:
    - **In-plane (x, the drone-drone line): well controlled** — load x std **0.10 m**.
    - **Out-of-plane (y, perpendicular): under-actuated** — load y swings **±0.6–0.9 m**
      (std 0.45, max 0.87). Two collinear cables have near-zero out-of-plane force authority:
      the controller saturates its y-correction at the 1.8 m clamp but the load barely responds.
  - **Fragility:** softer gains (kp1.0/kd1.0/ki0.05) **diverged** (load shot to z=10 m, crashed
    gz) — rigid cables are numerically touchy; the default gains sit in the stable island.
  - **Implication for the thesis:** a centred 2-drone carry can *hold bounded* but cannot
    precisely position/track the load out-of-plane (so a 2-drone *circle* with y-motion is not
    cleanly trackable). This is fine for the 2→3 scenario **if** the 2-drone phase is treated as
    "bounded holding until drone 3 attaches" (a 3rd non-collinear drone makes the load fully
    actuated horizontally), and the fly-in tolerates the ±0.6 m out-of-plane swing.
  - **Principled next step (feeds Stage 5):** project the load-position correction onto the
    *controllable* subspace (the plane spanned by the cable directions) instead of fighting the
    unactuated DOF — the controller currently wastes authority saturating in y.
  - Artifacts: `stage4_two_drone/stage4_two_drone_carry.png`, `two_drone_hover.csv`.
- **Stage 5 — Runtime-reconfigurable allocation + N (where the contribution lives).**
  Factor the FF/tension derivation out of `_enter_planner_phase` into a
  `_relatch_config(active_set)` you can call again. Replace the equal-split (`line 477`)
  with a small **per-cycle tension QP** (tensions ≥ `t_min`, thrust limits) valid for
  `|active_set|` ∈ {2,3}. Drive with a manual `/fleet/reconfigure` topic, drone 3
  pre-spawned + taut. Deliverable: manual 2→3 switch, load-z stays flat.
- **Stage 6 — Smooth tension hand-in.** Reuse `_cable_taut_gate(i)` to ramp the incoming
  drone's tension 0→⅓ while ramping the two existing drones →⅓, holding **total vertical
  tension = m·g** throughout. Deliverable: load-z flat through the reallocation.
- **Stage 7 — Mid-circle + real fly-in handshake.** Trigger mid-orbit (carry `traj_t`,
  don't reset). Then the interface with the other team: explicit topic handshake
  (`/drone_2/attach_request` → your planner verifies tautness → `/drone_2/attached`); the
  other controller stops commanding drone 3 on that signal. **Hard safety rule: never two
  controllers on one drone's ELRS.** Deliverable: real fly-in → your stack takes over.

**Immediate next step:** Stages 1–4 done. Stage 4 risk gate **passed (bounded) but exposed the
2-drone out-of-plane under-actuation** (±0.6 m swing, uncontrollable with 2 collinear cables).
**Next is Stage 5** — runtime reconfigurable allocation + N (the contribution): factor the
FF/tension derivation out of `_enter_planner_phase` into a re-callable `_relatch_config(active_set)`,
replace the equal-split with a per-cycle tension QP valid for |active_set|∈{2,3}, and drive a
manual `/fleet/reconfigure`. Consider the subspace-projection fix from Stage 4 as part of the
allocation (don't command force in the uncontrollable direction).

---

## 6. Facts, wire formats, and gotchas needed to run/test

### Reference-trajectory wire format (planner → tracker)
`Float64MultiArray` on `/drone_{id}/reference_trajectory`:
`[n_nodes, dt, px,py,pz, vx,vy,vz, ax,ay,az, cx,cy,cz, …]` — **12 fields/node**
(position, velocity, specific-thrust accel `a`, cable accel `c`). Legacy = 9 fields.
Parsed in `controller_quad_load/controller_mpc.py:299–312`.

### Fleet control
- `ros2 topic pub --once /fleet/command std_msgs/msg/String "{data: ARM}"` then `TAKEOFF`
  (also `DISARM`, `ESTOP`). Owner: `controller_mpc_multi/main.py`. `/fleet/step` @ 30 Hz.
- ARM/TAKEOFF are verified to reach the controllers; if drones "don't take off" it's the
  tracker/geometry, not the command path.

### Environment (critical — silent failures otherwise)
- The shell `.bashrc` overlays a **separate** workspace `/home/wesley/thesis`. **Always
  `source /home/wesley/multi_drone_control/install/setup.bash` LAST** so this repo's
  packages win (verify: `python3 -c "import utility_objects,os;print(utility_objects.__file__)"`
  should point under `multi_drone_control`, not `thesis`).
- Gazebo needs `export GZ_SIM_RESOURCE_PATH=/home/wesley/multi_drone_control/simulation_assets:$GZ_SIM_RESOURCE_PATH`.

### acados solver caching (avoids the "controller died on first launch" trap)
- Each tracker compiles its acados solver once, serialized by a file lock
  (`c_generated_code/.compile.lock`); freshness = solver `.so` mtime vs `acados.py`/`dynamics.py`.
- After rebuilding a controller package, the solver goes "stale" and the next launch
  recompiles — the one fragile moment. If a controller dies on first launch after a
  rebuild, force one clean build first:
  `rm -rf c_generated_code/c_generated_code c_generated_code/quad_dynamics_ocp.json`
  (analogous dir for `controller_quad_load`), then relaunch.

### Running the stacks (headless-server examples)
```bash
cd /home/wesley/multi_drone_control
source install/setup.bash                       # LAST, after any thesis overlay
export GZ_SIM_RESOURCE_PATH=$PWD/simulation_assets:$GZ_SIM_RESOURCE_PATH

# Gazebo (server only; use `gz sim <world>.sdf -v4 -r` for GUI)
gz sim -s -r simulation_assets/three_soft.sdf &

# Cable-AWARE tracker + planner (use this for load work).
ros2 launch controller_quad_load mpc_three_soft_quad_load_launch.py &
#   For a stable baseline add:  planner_mode:=kinematic   (coupled diverges)

# Arm + take off (or use the RViz ArmPanel)
ros2 topic pub --once /fleet/command std_msgs/msg/String "{data: ARM}"
ros2 topic pub --once /fleet/command std_msgs/msg/String "{data: TAKEOFF}"
```
Build after edits:
`colcon build --symlink-install --packages-select <pkg> && source install/setup.bash`.

---

## 7. Open design decision (needed before Stage 4–5)

**Payload attach geometry — RESOLVED 2026-07-15 (user): option (b), "2-then-3 / re-centering".**
The load has **only 2 attach points in phase 1** and physically **gains a 3rd** when drone 3
joins, so it **re-centers** on attachment. Consequences:
- Stage 4's 2-drone carry is **symmetric/balanced** (cables ~180° apart, load centered) — the
  easier, cleaner risk-gate. Equal-split tension is valid in the 2-drone phase.
- Stage 5's tension allocation transitions from a symmetric 2-cable split to a symmetric
  3-cable split as the new attach point engages (Stage 6 ramps it in via `_cable_taut_gate`).
- The rejected alternative (a) fixed 3-point/offset carry is NOT the scenario — don't build the
  asymmetric static solve.

---

## 8. Questions still worth confirming with the user

- Attach geometry (§7): fixed 3-point vs 2-then-3.
- Attach detection for the handoff: explicit handshake topic (recommended) vs your planner
  auto-detecting from tension/geometry.
- Orientation: when (if ever) 6-DOF is needed — deferred for now (position-first chosen).

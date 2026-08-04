# CLAUDE.md — multi_drone_control (Wesley's thesis)

Read this before touching anything. It is the orientation document; the deep
write-ups it points at are the source of truth for their own threads.

---

## 1. What this project is

**Thesis goal:** a controller that lets quadcopters **attach to and detach from a
cooperatively-carried cable-suspended payload mid-flight**, so a fleet can
reconfigure without putting the load down.

Two published methods are the foundation, and the thesis contribution is the
third thing:

| Piece | Source | Role here |
|---|---|---|
| **OCP / centralized MPC** | Sun et al. 2025, *"Agile and Cooperative Aerial Manipulation of a Cable-Suspended Load"* (arXiv 2501.18802, PDF in repo root) | Trajectory following, including agile trajectories. Also owns takeoff (ground break-off). |
| **Dissipative network** | Quan et al., *"Self-Organizing Aerial Swarm Robotics: A Table-Mechanics-Inspired Approach"* (arXiv 2509.03563, PDF in repo root) | Decentralized virtual-node spring-damper network. A member **leaving** the fleet (e.g. a failure mid-mission) is its native operating mode — no solver switch, no hand-designed redistribution. |
| **ADDING a drone back in** | **none — this is the novelty** | A free drone joins an already-flying fleet, the formation reconfigures, and the trajectory continues. To our knowledge unresearched. |

**The collaboration:** another student (Tejen) built the drone with a **swung
electromagnet** on a tether and the approach controller that flies it to a target
and welds on. This project's controller takes over **at the weld**: it must
absorb the newcomer, reconfigure the fleet, and carry on with the trajectory.

**Real-world:** there is a working hardware rig (UNSW motion capture + Betaflight
quads + ExpressLRS radio). Results for the thesis publication are to be collected
and collated on it over roughly the next 3–4 months (from 2026-08).

---

## 2. Honest status (2026-08-04, branch `attach-week9`)

Do not overstate any of this in reports, plots or commit messages.

| Capability | Status |
|---|---|
| Centralized OCP: takeoff, hover, line, circle, fig-8 | **Works in sim**, 2–4 drones. The baseline/reference controller. |
| Detach (4→3→2) mid-flight | **Works and is verified in sim** with both OCP and dissipative. Even redistribution, azimuth pin, LAND lands attached + detached drones, trajectory continues through the event. This is the demo video. |
| Dissipative network: hold + reconfigure | **Works** (that is what it is for). |
| Dissipative network: **trajectory tracking** | **Weak — open problem.** Circle r=0.5 at 0.6 m/s: mean payload error 0.202 m, flown radius 0.374 vs 0.500, phase lag 26° (0.72 s). Root-caused to the *tracker* stage, not the network references. Full write-up + the diagnostic to re-run: **`DISSIPATIVE_TRACKING_ISSUE.md`** (repo root). |
| Attach: approach → weld → join | **Close.** The newcomer flies in, welds, and joins the network. As a **central lifter** (`attach_central:=true`) it is stable and tilt-free. |
| Attach: **restabilize / reconfigure as a ring member** | **Not there yet.** The out-and-down transit from a high central weld to a rim slot is where it destabilises. The big runaway was solved (magnet drone-side joint universal → **ball**); what remains is a stability↔level tradeoff on settle elevation (65° = stable but load parks ~25° tilted; 45° = briefly level then diverges). See `drone-attach-integration` memory. |
| Real-world | I/O layer + real launches exist and takeoff bugs were fixed on the rig (see `learning.txt`, session 2026-08-03). Results collection is upcoming. |

**When in doubt, the honest framing is:** detach is the working stepping stone;
attach is the contribution and is partially working; dissipative trajectory
tracking is a known weakness with the OCP available as the tracking controller.

---

## 3. Where the real documentation lives

Read these before re-deriving anything. They contain evidence, rejected
approaches and measurements, and are far more detailed than this file.

| File | Contents |
|---|---|
| `DISSIPATIVE_TRACKING_ISSUE.md` (root) | The open tracking problem. Includes the stage-split diagnostic, what was tried and **rejected with evidence** (do not retry those), and priority-ordered next steps. |
| `general/CABLE_LOAD_CONTROLLER_PROGRESS.md` | The centralized planner + cable-aware tracker: architecture, every bug overcome, design choices, constants. |
| `general/DIAGNOSE_CABLE_LOAD.md` | Diagnostic procedures for the cable-load stack. |
| `learning.txt` (root) | Chronological session log. Newest at the top. The 2026-08-03 block is the real-world takeoff bug fixes (yaw-aware slot assignment, latched payload yaw, yaw-free tilt quaternion). |
| `AA_Learnings.txt` (root) | Scratch command reference (gz topics, arm/takeoff without RViz, wifi recovery). |
| `progress.txt` (root) | Supervisor-facing status summary; good template for updates. |
| `src/controller_load_mpc/DESIGN.md` | Planner design notes. |
| `~/.claude/.../memory/` | Long-form project memories: `drone-attach-integration`, `dissipative-detach-controller`, `dissipative-only-flight`, `adaptive-thrust-ratio`, `cable-load-controller`, `lift-system-anchor-pose-fix`. |

---

## 4. Package map (`src/`)

| Package | What it is |
|---|---|
| `controller_load_mpc` | **Centralized planner** (`planner` node, 10 Hz). acados OCP over the coupled load+cable+drone model (`load_cable_dynamics.py`), emits per-drone reference trajectories. Also holds shared stateless helpers: `geometry.py`, `load_trajectory.py`, `creep_controller.py`, `reference_builder.py`, `params.py`. |
| `controller_quad_load` | **Per-drone cable-aware MPC tracker** (`controller`, 50 Hz) + fleet manager (`main`) + kT estimator (`kt_estimator`). Owns all the **launch files**. |
| `controller_dissipative` | **The dissipative network.** `dissipative_node.py` subclasses `LoadPlanner` so takeoff stays on the proven OCP creep→lift; `dissipative_network.py` is the spring-damper network; `verify_dissipative.py` + `mini_plant.py` are the offline gate. |
| `controller_mpc_payload` | Tejen's **approach MPC** for the magnet drone (+ his thrust-ratio UKF). |
| `drone_magnet` | The attach chain: `online_join_planner`, `attach_target_publisher`, `magnet_attachment_manager`, `magnet_tip_publisher`, `elrs_mux`. |
| `simulation_communication` | Gazebo bridges + mocap/betaflight emulators + `fleet_viz`. |
| `drone_communication` | **Real hardware**: mocap publisher, ELRS interface. |
| `drone_visualisation` | RViz + the ArmPanel (ARM/DISARM/TAKEOFF/LAND/**DETACH**/**ATTACH** buttons). |
| `interfaces`, `utility_objects` | Msgs/srvs; `CallbackManager`, `DataLogger`, visualisation helpers. |
| `controller_ukf` | Single-drone UKF controller inherited from the upstream platform — the source of the ported thrust-ratio UKF. |
| `tejen/` | Tejen's original stack, `COLCON_IGNORE`d. Reference only. |

This repo is a **fork, isolated to this thesis** — nobody else works in it, so
nothing here is off-limits. `controller_ukf` and `controller_mpc_payload` were
inherited from the upstream platform; they still matter (the thrust-ratio UKF was
ported from one, and the approach MPC is the attach path), so change them
deliberately rather than casually, but there is no permission to ask for.

---

## 5. Architecture and data flow

```
                      /payload/motion_capture_state
                      /drone_i/motion_capture_state
                                  |
        +-------------------------+--------------------------+
        |                                                    |
   OCP PLANNER (10 Hz)                        DISSIPATIVE NETWORK (10 Hz)
   controller_load_mpc/planner                controller_dissipative/dissipative
   coupled acados OCP                         virtual-node spring-damper
        |                                                    |
        +----------------> /drone_i/reference_trajectory <----+
                            (Float64MultiArray, 12 fields/node)
                            [n_nodes, dt, p(3), v(3), a_thrust(3), a_cable(3), ...]
                                  |
                   PER-DRONE TRACKER (50 Hz, acados MPC)
                   controller_quad_load/controller
                   quad model + a_cable term in v_dynamics
                                  |
                          /drone_i/ELRSCommand
                                  |
                  sim: payload_betaflight_comm -> gz motor_speed
                 real: elrs_interface -> ExpressLRS TX -> Betaflight
```

Both reference generators publish the **identical wire format**, which is why the
trackers and fleet manager are untouched by the dissipative/attach work. Keep it
that way.

**Phase dispatch:** `DissipativeController` overrides `_plan` — in the `network`
phase it runs the network, otherwise it calls `super()._plan()` (the OCP). It
enters the network phase on `/fleet/detach`, on an attach weld, or (only in
`dissipative_only_launch.py`) via `auto_network_handover` once the OCP lift tops
out and settles.

**Fleet control topics**

```bash
ros2 topic pub -t 3 /fleet/command std_msgs/msg/String "{data: ARM}"      # ARM|TAKEOFF|LAND|DISARM|ESTOP
ros2 topic pub -t 3 /fleet/detach  std_msgs/msg/Int32  "{data: 1}"        # drone index to release
ros2 topic pub -t 3 /magnet/command std_msgs/msg/String "{data: ON}"      # = the RViz ATTACH button
```

---

## 6. How to run

### Build

```bash
colcon build --symlink-install && source install/setup.bash
# or: --packages-select controller_quad_load controller_load_mpc controller_dissipative \
#     controller_mpc_payload drone_magnet interfaces utility_objects simulation_communication
```

### Offline gate — run this BEFORE any Gazebo test, and before every commit

```bash
./tools/gate.sh              # all 5 stages, ~2 min
./tools/gate.sh --quick      # skips stage 5 (the slow one)
```

Ordered cheapest-first so it fails fast. Every stage exists because something got
through without it:

| | stage | catches |
|---|---|---|
| 1 | workspace | a stale `~/thesis` shadowing this repo's `interfaces`/`utility_objects` |
| 2 | unit tests | envelope checker, RViz config structure, planner reference, config tools |
| 3 | **import check** (`tools/import_check.py`) | a missing import that kills a node at launch. `ast.parse` and `colcon build` **both pass** on that — only an actual import catches it |
| 4 | world geometry (`tools/check_geometry.py`) | world SDF vs launch/`params.py` disagreement on `cable_len`/`attach_radius`/`attach_z`/`load_mass` — presents as "the controller can't fly", not as a config bug |
| 5 | dissipative A–K | detach/attach correctness. Saves an annotated PNG to `dissipative_verify/`. **Know its limits** (§8) |

Stage 4 only covers the three world/launch pairs listed in `gate.sh`. **Add new worlds
there** or the check silently covers less than it appears to.

Other config tools, not in the gate:

```bash
tools/param_diff.py --sim-vs-real     # what changes between sim and the rig, incl.
                                      # args declared on ONE side only (silent node-default
                                      # fallback — this is how the real dissipative launch
                                      # ended up with no kT block at all)
tools/check_geometry.py simulation_assets/<world>.sdf --launch <launch.py>
```

### Simulation

RViz launch must come up **first**, then Gazebo/controllers.

```bash
# Centralized OCP baseline
cd simulation_assets && gz sim three_rigid_ground.sdf -v4 -r
ros2 launch controller_quad_load rviz_quad_load_launch.py num_drones:=3
ros2 launch controller_quad_load mpc_quad_load_launch.py num_drones:=3 load_traj:=circle

# Dissipative network for the WHOLE flight (OCP takeoff, auto handover)
ros2 launch controller_quad_load dissipative_only_launch.py num_drones:=3 \
     load_traj:=circle traj_speed:=0.6 traj_radius:=0.5

# Detach demo (network engages only on the detach event)
cd simulation_assets && gz sim four_rigid_ground.sdf -v4 -r
ros2 launch controller_quad_load rviz_quad_load_launch.py num_drones:=4 detach:=true
ros2 launch controller_quad_load dissipative_launch.py num_drones:=4
#   ARM -> TAKEOFF -> DETACH button (or /fleet/detach)

# Attach (the novelty). Central lifter = the stable, working config:
cd simulation_assets && gz sim three_attach.sdf -v4 -r
ros2 launch controller_quad_load rviz_quad_load_launch.py num_drones:=3 attach:=true
ros2 launch controller_quad_load three_attach_launch.py
#   ARM -> TAKEOFF -> ATTACH button

# Attach as a RING member (reconfiguration, still unstable — the open work):
ros2 launch controller_quad_load three_attach_launch.py \
     attach_central:=false diss_balanced_tensions:=true attach_handout:=true \
     attach_x_offset:=-0.08 attach_elev_deg:=65 enable_obstacle_avoidance:=false
```

`load_traj` ∈ `hover | line_x | circle | fig_8 | spin`. `hover`/`line_x` never
self-complete — end those runs with LAND.

### Real world (two terminals)

```bash
ros2 launch controller_quad_load real_io_launch.py num_drones:=2        # terminal 1: mocap + ELRS + RViz
ros2 launch controller_quad_load real_control_launch.py num_drones:=2   # terminal 2: trackers + planner
# dissipative variant: real_dissipative_launch.py
```

The mocap rigid-body → drone routing table is at the top of
`drone_communication/motion_capture_publisher_node.py` (`RIGID_BODY_TO_DRONE`,
`PAYLOAD_RIGID_BODY_ID`). Real launches keep `thrust_ratio:=24.0` and
`thrust_quad_c:=0.0` — the sim values (30 / 88.6) are **wrong on hardware**.

### Logs and plots

```
c_generated_code_quad_load/logs/controller_quad_load/<traj>_drone<N>_<ts>/log.csv + params.json
python3 plot_run.py --logdir c_generated_code_quad_load/logs/controller_quad_load
```

Drone 0's `log.csv` carries `payload_*` and `payload_ref_*`; every drone carries
`pose_*` and `ref_*`. Each run writes `params.json` — **always check it** to
confirm the parameter you think you changed actually plumbed through.

---

## 7. Comparing two runs — the rules

1. Check `params.json` on both. Launch args have silently failed to plumb through
   before.
2. Check the sweep durations match within ~10%, or the comparison is meaningless.
3. Always re-run the **stage-split diagnostic** in `DISSIPATIVE_TRACKING_ISSUE.md §2`
   after a change to the tracking path — desired → reference centroid → actual
   centroid → payload, with radius ratio and phase lag per stage. Which stage
   moved is the whole answer.

---

## 8. Traps and hard-won lessons

**Verification honesty**

- The offline harness (`mini_plant`) uses a **PD tracker proxy**, so it cannot
  reproduce the Gazebo divergences: the real MPC has **no integrator** and caps
  the cable feedforward, and it under-thrusts where a PD proxy does not. Several
  fixes shipped on `mini_plant` evidence had to be walked back. Treat its results
  as necessary, not sufficient.
- `verify_dissipative` holds a fixed `p_des` and **never calls `_network_plan`**,
  so it does not cover trajectory following at all. Smoke test, not a guard.
- The payload in `mini_plant` is now a rigid body (tilt reproducible), but it
  still settles cases that diverge in Gazebo.

**Diagnosis method that works**

- Watch `|aCm|` (measured cable accel) vs `|aC|` (modelled): bounded = healthy;
  exploding beyond ~g = the drone is being physically dragged, not mis-commanded.
- Watch `ERR` in the `[diag dN]` line: near zero while things go wrong = the
  reference is fine and the plant is the problem.
- Monotonic thrust collapse = a positive-feedback loop. Swinging `|aT|` = an
  unconverged solve.

**Physics constraints that no amount of tuning removes**

- A drone has ~13 m/s² of thrust acceleration left after gravity. `|aC| > 13` =
  physically overpowered, it falls.
- With three tethers fixed at 120°, a **fourth drone cannot form an even 4-gon
  under equal force sharing** — confirmed geometrically impossible. Level +
  reconfigured needs **unequal (moment-balanced) tensions**
  (`diss_balanced_tensions:=true`), and even-4-gon *spreading* only exists in the
  **non-balanced** branch. The two goals are in different code paths; you cannot
  have both from one flag combination.
- A **centre weld** cannot be a load-bearing ring member — a centre-welded cable
  pulling up-and-out just shoves the payload. Ring reconfiguration requires an
  **off-centre** weld, ideally at a **gap centre** (az 180 → `attach_x_offset:=-0.08`)
  so drones do not bunch.

**Environment**

- Back-to-back launches leak nodes and exhaust Fast-DDS shared memory, which
  fakes "divergence". Kill the stack fully and `rm /dev/shm/fastrtps_*` between
  runs.
- Nested `lift_system` worlds need a world-fixed `anchor` canonical link or every
  nested pose is published relative to drone 0 and the MPC flies blind.
- Gazebo `DetachableJoint` **spawns attached** and makes a **fixed weld** — all
  compliance must come from real joints. Weld/detach commands must go through the
  ROS→gz **bridge**, not a `gz topic` subprocess.
- The magnet drone-side joint must be a **ball** joint. As a universal joint it
  binds when the welded drone swings around the load and transmits force instead
  of pivoting — that was the attach runaway.

**Stale code to not be fooled by**

- `controller_load_mpc/dissipative_node.py` and `dissipative_network.py` are
  **superseded** (last real change 2026-07-22). The live implementation is
  `controller_dissipative/`. The old `dissipative` entry point still exists in
  `controller_load_mpc/setup.py`.
- `dissipative_launch.py` alone **never engages the network** unless a detach or
  attach event fires. For a whole-flight network run use
  `dissipative_only_launch.py`.
- Two different kT (thrust ratio) estimators exist — Tejen's UKF in
  `controller_mpc_payload` and the ported UKF in `controller_quad_load`. Don't
  conflate them. `kt_seed` is deliberately **separate** from `thrust_ratio`
  (`thrust_ratio` is the takeoff constant, kept low; `kt_seed` is the estimator's
  band centre and wants the true hover kT). See the `adaptive-thrust-ratio`
  memory before touching either.

---

## 9. Open problems, in priority order

1. **Dissipative trajectory tracking** — close the 0.589 s tracker-stage lag.
   Candidates, in order: reference staleness (10 Hz planner vs 50 Hz tracker,
   interpolate the horizon by message age), velocity cost weight (2.0 against a
   position weight of 80), `skip_steps=3`. If none close it, the honest options
   are documented in `DISSIPATIVE_TRACKING_ISSUE.md §7`: use the OCP for tracking
   and the network for resilience, or restructure the network to command velocity
   as the paper does.
2. **Attach restabilization** — find the settle elevation (probe 55°, then 60°)
   where an off-centre newcomer is stable. A stable angle with a modest residual
   tilt is a real result; perfectly level *and* stable may be unreachable with a
   rigid weld, in which case the central lifter is the robust fallback.
3. **Real-world results collection** — the thesis deliverable. Validate OCP and
   dissipative across the trajectory set on hardware, then detach, then attach.

Known and explicitly **out of scope** unless asked: the 5–7 cm steady payload sag
below target height; the two latent bugs in `controller_mpc.py:906`
(multiplicative node-0 box) and `acados.py:203` (`set_initial_guess` never sets
`x`) — both confirmed to be real but not the cause of any current symptom.

---

## 10. Working conventions

- **Run the offline gate before proposing a Gazebo run**, and say plainly when
  the gate cannot cover what is being changed.
- **Do not change the tracker's MPC weights casually.** They are proven by the
  three working tethers; the user's steer is to fix the *geometry* of the
  attached configuration instead.
- **Keep the reference wire format stable** (12 fields/node) so both reference
  generators stay interchangeable.
- **Preserve verified paths.** New behaviour goes behind a parameter that
  defaults to the old behaviour (`reserved_attach=0`, `auto_network_handover=False`,
  `attach_handout`, `k_slot=0`, balanced-mode-only branches). This is how the
  verified detach config has survived the attach work.
- **Report results honestly**, including "this settles offline but has not been
  seen in Gazebo". The reason this project has moved is that failed sim runs were
  written down with their mechanism, not just their outcome.
- Append significant findings to `learning.txt` (newest block at the top) and
  update the relevant `general/*.md` write-up.

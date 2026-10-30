# The T's settled tilt from our planner's own model (30 Oct 2026, offline, no Gazebo)

Question (X1 cell sqT, R2965 ours / R2966 no resize): after the square 0/9/6/3 loses plate 9 and our planner resizes
to the T 0/6/3, the twin's ring settles 7-13 deg off level. Without the resize it settles at 2.2 deg. Is that tilt
the planner's own optimum, or is it from the closed loop?

**Answer: the planner's own optimum.** Closed on its own model, the real OCP settles the T at **7.35 deg** with no
integral force. With the ring force the integral had learned in R2965, it settles at **9.38 deg**. The twin's hover
window in R2965 reads 8.7 deg (7.8-9.4). The tilt is a roll about the pair's line (plates 0 and 6), with the middle
carrier's side (plate 3) up. What causes it is the **0.1 N tension floor** on the middle carrier. It is set twice:
as a hard bound in the OCP, and as the floor in the tension reference. The square and the even triangle settle
exactly level. The T with the middle carrier's floor removed also settles exactly level.

## 1. The model, the cost and the constraints (code as flown: R2965 = 97da92d + diff, no planner file in the diff; HEAD 5a7ccd0 changes only the relay code)

| What | Where | Value for sqT |
|---|---|---|
| Ring dynamics: force -sum t_i s_i + d_L, moment sum t_i (R^T s_i x rho_i), massless cables | `src/mpc_planner/mpc_planner/load_cable_dynamics.py:171-181` | m 0.86 kg, J = `params.load_inertia(0.86)` |
| Drone = attach point - l s_i | `load_cable_dynamics.py:231-232` | l = measured rod length (~1.0 m) |
| Attach points rho_i: ring radius, height over the CoG, azimuths | `geometry.py:160-172` | r 0.225 m, h = attach_z 0.02 m (the twin's stubs are also 0.02 m over the body: `four_rigid_ground_rig_L100.sdf:66,206`) |
| After the release, each survivor keeps its own attach point (slot order = survivors) | `src/dissipative_planner/dissipative_planner/dissipative_node.py:1664-1668`, `planner_node.py:1808-1876` (`resize_fleet`), `planner_solver.py:260-275` (rho and l as runtime parameters) | slots [drone 0, 2, 3] at 0, 180, 270 deg ccw: the pair on the x axis, the middle carrier (drone 3, plate 3) at -y |
| Cable-direction reference | `planner_node.py:1848` -> `geometry.py:175-186` | radial, elevation `cable_elev_deg` 60 |
| Tension reference: static balance, least squares from the equal split, **floored at t_min 0.1** | `reference_builder.py:97` -> `geometry.py:211-242` (floor at :242) | T: 4.871 / 4.871 / 0 -> **0.1 N** |
| Effect of the integral force on the references: g_eff = g - d_L/m tilts every s_ref and scales every t_ref | `reference_builder.py:244-254`. The integral is a force with no moment: `lumped_integral.py:47-49`, `planner_node.py:822-831` | R2965 hover: d_L = (0, -0.28, -0.27) N |
| Attitude reference: yaw only (level) | `reference_builder.py:256-261` | - |
| Cost: p 60/60/80, v 18/18/22, **attitude 5**, w 1, **s 25**, **t 5**, r 3, u 1e-3 / 5e-3 | `planner_ocp.py:89-99, 113, 121, 129-133` | - |
| **Tension bound tension_min 0.1 N** (hard, stages 1..N-1; none at node 0 or N) | `planner_ocp.py:63, 166-170` | binds on the middle carrier |
| Thrust 0.5-15 N (soft) and input boxes | `planner_ocp.py:172-189` | inactive (pair 9.9 N, middle 5.5 N) |
| Node 0 pins the measured ring pose and twist and the cable directions. Tensions stay free | `planner_ocp.py:148-153` | - |
| Horizon N 20, tf 2 s, 5 warm SQP iterations | `planner_solver.py:33-36`, launch `plan_sqp_iters 5` | - |

No cone or elevation limit on the cables, and no pretension term, appears anywhere in the OCP.

## 2. Why the T's roll about the pair's line is weak in the model (and in the twin)

The T's gap is exactly 180 deg, so the CoG lies on the line through the pair's attach points. The pair's pulls act
on that line and have no lever about it. Two things set the roll about it:
- the 2 cm pendulum. The attach points sit h = 0.02 m above the CoG, so the restoring stiffness is m g h =
  **0.169 N m/rad (2.9 mN m per deg)**. This holds in the model (`rho_z` = attach_z) and in the twin (stubs at +0.02 m).
- the middle carrier's pull. Its lever is r sin(60) - h cos(60) = **0.185 m**.

Each newton of middle tension therefore holds about **63 deg** of roll, so 0.016 N is enough for 1 deg. A level T
needs the middle carrier's vertical pull to be zero. The planner's authority over this roll is also one-sided:
- it can raise the middle side by pulling harder
- it cannot lower that side below the tilt that 0.1 N already holds.

The square and the even triangle have full levers about every horizontal axis. Their floors never bind (2.44 N and
3.25 N per cable).

## 3. The planned steady state, offline

Two independent checks, both on the planner's own model. Neither uses Gazebo.
- **Static**: the stage cost that can be non-zero at rest (attitude, s, t) is minimised over the equilibria of the
  model: force and moment balance, |s_i| = 1, t >= bound. The references come from the repo's `ReferenceBuilder`.
- **OCP**: the real `PlannerSolver` and `ReferenceBuilder` are closed on their own model. The next state is the plan's
  node 1 (the kinematic plant of `tools/solve_scaling.py`). It runs at 10 Hz with 5 SQP iterations, hovering at 1.0 m.
  It starts from the square's level hover minus the released cable and runs for 400 ticks; the tilt is constant to 1e-7 deg.
  The solvers were built in a scratch `MDC_ACADOS_ROOT` through `tools/latency_bench.build_solver`.

The common settings are m 0.86, r 0.225, h 0.02, l 1.0 and 60 deg. Tilt is the ring's roll about the x axis (the
pair's line). A minus sign means the -y side is up, which is the middle carrier's side. The pull shares are the
vertical components of the cable pulls.

| Case | OCP tilt (deg) | Static tilt (deg) | Pulls node 1 (N) | Middle share | Elevations (deg) | Middle-drone height over the pair (cm) |
|---|---|---|---|---|---|---|
| Square 0/9/6/3 | 0.00 | 0.00 | 2.435 x4 | 25 % | 60 x4 | 0 |
| Even triangle 3/11/7 | 0.00 | 0.00 | 3.247 x3 | 33 % | 60 x3 | 0 |
| **T 0/6/3, as built** | **-7.35** | -6.17 | 4.847 / 4.847 / **0.121** | 1.2 % | 59.3 / 59.3 / 60.0 | +3.7 |
| **T, with R2965's integral force (0, -0.29, -0.29) N** | **-9.38** | -7.88 | 5.017 / 5.017 / **0.128** | 1.3 % | 59.2 / 59.2 / 61.9 | **+6.2** |
| T, integral force y only (0, -0.29, 0) | -9.73 | - | 4.848 / 4.848 / 0.129 | - | 59.1 / 59.1 / 62.0 | - |
| T, integral force z only (0, 0, -0.29) | -7.10 | - | 5.015 / 5.015 / 0.121 | - | 59.3 / 59.3 / 60.0 | - |
| **T, middle free (bound 0 and reference floor 0)** | **0.00** | 0.00 | 4.871 / 4.871 / 0.000 | 0 % | 60 x3 | +0.1 |
| T, bound 0, reference floor 0.1 kept | -3.14 | -2.63 | 4.861 / 4.861 / 0.047 | - | - | - |
| T, bound 0.1 kept, reference floor 0 | -6.19 | -6.17 | 4.852 / 4.852 / 0.101 | - | - | - |
| T, bound and floor 0.01 N | -0.80 | -0.66 | 4.868 / 4.868 / 0.012 | 0.1 % | - | - |
| T, attitude weight 30 / 500 (static only) | - | -6.14 / -5.43 | - | - | 500: the middle carrier at 50.6 | - |

R2965's hover window (54.5-65.9 s, n = 3) for comparison:

| Quantity | Logged | Offline (with R2965's integral force) |
|---|---|---|
| Tilt (deg) | 8.7 (7.8-9.4), roll -8.7 | 9.38, roll -9.38 |
| Planned pulls (N) | 5.01 / 5.01 / 0.131 | 5.02 / 5.02 / 0.128 |
| Elevations (deg) | 59.3 / 59.3 / 62.1 | 59.2 / 59.2 / 61.9 |
| Middle-drone height over the pair (cm) | 6.0 | 6.2 |

At the fixed point, every plan intends to level the ring but never gets there. In the converged horizon of the T,
the middle tension holds 0.12 N for nodes 0-2 and then falls to the 0.1 N bound. It stays **on the bound for nodes
7-19**. Along the horizon the roll eases from 7.35 to 5.0 deg, then rises again towards node 20, which has no bound.
Node 1 always equals node 0, so the ring never moves. The reference cannot be reached for the T: no equilibrium has
the ring level, the middle carrier at its 0.1 N floor and the cables at 60 deg. The receding-horizon fixed point
therefore sits beyond the static optimum (7.35 against 6.17 deg).

## 4. Reading

1. **The model's optimum for the T is tilted. The closed loop does not cause the offset.** The twin reproduces the
   plan almost exactly: the tilt, the planned pulls, the middle cable's elevation and how far the middle drone is
   placed above the pair (6.0 against 6.2 cm). The drones are position-controlled on the twin's rigid rods. When the
   planner places the middle drone 6 cm high, the twin's ring takes the planned tilt. The no-resize arm still plans
   four level cables. It therefore places the middle drone for a level ring, and the ring flies at about 2 deg.
2. **The term responsible is the 0.1 N tension floor on the middle carrier**, set in two places:
   - the hard bound `tension_min` (`planner_ocp.py:63`, applied `:166-170`). Alone it gives 6.2 deg.
   - the reference floor `t_min` in `balanced_tensions` (`geometry.py:242`, via `reference_builder.py:97`). Alone it
     gives 3.1 deg.

   Together they give 7.35 deg.

   The planner's own integral adds about 2 deg. Its learned ring force (-0.28 N in y, -0.27 N in z) tilts g_eff in
   the references (`reference_builder.py:244-254`). This steepens the middle carrier's reference from 60 to 62 deg,
   so the same floor pull holds more moment. The same force was learned in the no-resize arm (int_fy -0.30 N), so it
   comes from the plant, not from the resize.
3. Neither the attitude weight nor the cable cost causes it. Raising the attitude weight from 5 to 30 or 500 barely
   moves the tilt (6.1 / 5.4 deg static), because the floor and the statics set it. The thrust limits and the input
   boxes are inactive.
4. In the model, the T's roll about the pair's line is held only by the 2 cm pendulum: 0.169 N m/rad, about 63 deg
   per newton of middle tension. So a level T must give its middle carrier zero vertical pull. This is a property of
   the physics as well as of the model. Our planner cannot have a level ring and a taut middle cable at the same time.

## 5. The smallest model-side change that removes it

Give a slot whose static share is zero a floor of zero, in both places:
- `balanced_tensions(..., t_min=0)` for the reference
- `lbx` 0 for that slot at stages 1..N-1. acados can change this at run time with `constraints_set(k, 'lbx', ...)`,
  so no rebuild is needed.

Offline the T then settles at **0.00 deg** with the middle pull at 0.000 N. A 0.01 N floor everywhere gives 0.8 deg.
Removing only one of the two floors leaves 3.1 or 6.2 deg.

Caveat (real world first): in a level T the middle carrier carries nothing.
- In the twin, a rigid rod at zero pull still pins its plate, because the drone is position-controlled.
- On the rig, a cable at zero pull goes slack and stops holding the plate. The roll is then left to the 2 cm
  pendulum, and a middle pull error of 0.016 N is already 1 deg.

The change therefore removes the tilt the planner asks for. It does not make the T's roll stiff, which no tension
plan can do while the CoG sits on the pair's line. It is a change to the OCP's constraints, so it is critic
territory and needs Wesley's word before it is built. Nothing was changed in the repo.

## Reproduction (scratch, not in the repo)

The scripts are in `/tmp/claude-1000/-home-wesley-multi-drone-control/d021810e-9d5d-427c-be70-40b127760327/scratchpad/t_model/`:
- `static_opt.py`: the static optimum, all cases in one run, about 2 s
- `ocp_loop.py <case>`: the real OCP closed on its own model, one case per process. The cases are square, tri, T,
  T_dL, T_dLy, T_dLz, T_free, T_lb001, T_lb0_ref01 and T_lb01_ref0. Each takes about 13 s once the n = 3 and n = 4
  solvers are built in its `work/`.

The results are in `static_opt.json` and `ocp_<case>.json` next to the scripts.

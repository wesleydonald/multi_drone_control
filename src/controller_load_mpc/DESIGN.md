# controller_load_mpc — design notes

Implements the centralized cable-suspended load planner from:
**Sun et al., "Agile and Cooperative Aerial Manipulation of a Cable-Suspended
Load", arXiv 2501.18802v2** (root of repo).

Kept as a **separate package** so the working `controller_mpc_multi`
(per-drone free-quad MPC, flies the two-drone worlds) is untouched.

## Paper framework (3 parts)
1. **Centralized kinodynamic planner** — one OCP over the whole-body load-cable
   dynamics, ~10 Hz, 2 s horizon. Outputs the *full reference trajectory of each
   quadrotor* (position + derivatives). This is the part we build here.
2. **Per-drone onboard tracker** — INDI, ≥100 Hz, follows the reference and
   compensates the measured cable force via IMU. (Not the planner.)
3. **Load EKF** — estimates load pose/cable directions from quad states + IMU,
   no sensors on the load.

> The planner does NOT emit motor commands — it emits reference trajectories.
> Something must still track them (see Decision 2 below).

## Load-cable model (paper Eqs 1–3)
State (n = number of drones):
```
x = [ p, v, q, ω,                              # load: pos, vel, attitude(quat), ang vel
      s_i, r_i, ṙ_i, r̈_i, t_i, ṫ_i  for i=1..n ]  # per cable
```
- `p,v ∈ R³` load position/velocity
- `q ∈ S³` load attitude, `ω ∈ R³` load angular velocity (load frame)
- `s_i ∈ S²` cable direction (quad→load), `r_i ∈ R³` cable angular velocity
- `t_i ≥ 0` cable tension (TAUT model: tension ≥ 0 enforced as path constraint)

Dynamics:
```
ṗ = v
v̇ = −Σ t_i s_i / m + g
q̇ = ½ Λ(q) [0; ω]
J ω̇ = −ω × Jω + Σ t_i ( R(q)ᵀ s_i × ρ_i )       # ρ_i = attachment point in load frame
ṡ_i = r_i × s_i
r⃛_i = γ_i        # control: snap of cable direction
ẗ_i  = λ_i        # control: 2nd deriv of tension
```
Inputs: `γ_i ∈ R³`, `λ_i ∈ R` per cable (high-order → smooth refs up to jerk).
Quad position derived: `quad_i = p + R(q) ρ_i − l_i s_i` (l_i = cable length).

Path constraints in the OCP: thrust limits, cable tautness (t_i ≥ 0),
inter-quad collision avoidance, obstacle avoidance.

## Adaptation to OUR setup (2 drones, two_soft.sdf)
Paper uses **n ≥ 3** quads for full 6-DOF rigid-load pose control. We have
**2 drones** → see Decision 1.
- Sim params (two_soft.sdf): load 0.4 kg box (0.2×0.2×0.05), 2 cables ~0.5 m,
  drones 0.6 kg. Cables are soft (spring segments) in sim — intentional
  model–plant mismatch vs the taut OCP model.

## Resolved decisions
1. **Load model:** full **rigid-body** load (q, ω, J, n attachment points ρ_i)
   for full 6-DOF control. Target n=3 (then 4) on three_soft.sdf.
2. **Tracker:** reuse the existing `controller_mpc_multi` per-drone MPC (NOT
   INDI) — feed it the planner's reference trajectory instead of the hardcoded
   circle.
3. **No EKF (sim):** the paper needs an EKF because it has no load sensors. We
   have `/payload/motion_capture_state` (ground truth) + drone mocap, so build
   x_init directly: load pose/twist from payload mocap; cable directions
   s_i = (p + R(q)ρ_i − p_i)/‖·‖ from geometry; cable rates / tensions /
   higher derivatives by resampling the previous OCP solution.

## OCP spec (paper Eqs 6–12)
- min  Σ_k ‖x_k − x_k,ref‖²_Q + ‖u_k − u_k,ref‖²_R + ‖x_N − x_N,ref‖²_P
- s.t. x_0 = x_init;  x_{k+1} = f(x_k,u_k) [load-cable dyn, Eqs 2–3];  h(·) ≤ 0
- input u = [γ_1, λ_1, …, γ_n, λ_n];  N = 20 (non-equidistant), SQP-RTI, acados
- path constraints h ≤ 0 (slack-softened):
  - thrust:    T_i,min ≤ ‖m_i(v̇_i−g) − t_i s_i‖ ≤ T_i,max     (Eqs 8–9)
  - tautness:  0 < t_min ≤ t_i ≤ t_max                          (Eq 10)
  - collision: d_min ≤ ‖p_i − p_j‖   ∀ i<j                      (Eq 11)
  - no-fly:    d²_o,min ≤ (p_c−p_o)ᵀ C (p_c−p_o)                (Eq 12)
  - input bounds u_min ≤ u ≤ u_max
- reference x_k,ref: polynomial LOAD pose ref → remaining states via flatness.
- quad ref out: p_i = p + R(q)ρ_i − l_i s_i (Eq 5) + derivatives; zero yaw.

## Build plan (incremental, each verifiable)
1. [DONE] `load_cable_dynamics.py` — CasADi model: state (Eq 1), f_expl
   (Eqs 2–3), kinematics p_i/v_i/v̇_i (Eq 5 + derivs), thrust T_i (Eq 9).
   Verified: nx=55, nu=12 for n=3; f_expl + thrust evaluate correctly.
2. [DONE] `planner_ocp.py` — acados OCP (NONLINEAR_LS load-pose cost; soft
   thrust + tautness + input-box constraints), SQP-RTI, PARTIAL_CONDENSING_HPIPM.
   Verified: compiles; symmetric 3-drone hover converges (status 0) to the exact
   equilibrium — tensions 1.85 N, thrusts 7.31 N. Collision/no-fly still TODO.
3. [DONE] `planner_node.py` — LoadPlanner node: x_init from mocap (load pose/
   twist from /payload/..., s_i from drone positions, rates/tensions resampled
   from previous solution), solve at 10 Hz, publish per-drone
   /drone_{i}/reference_trajectory = [n_nodes, dt, px,py,pz,vx,vy,vz,...].
   Codegen isolated in c_generated_code_load_planner/.
4. [DONE] Tracker hookup — controller_mpc.py gained a `reference_source` param
   ('internal' = own circle, default/unchanged; 'planner' = track
   /drone_{id}/reference_trajectory via set_planner_reference()). Combined launch
   controller_mpc_multi/launch/mpc_three_soft_planner_launch.py runs bridges +
   3 planner-tracking controllers + fleet manager + the load planner.
   Existing worlds unaffected (default stays 'internal').

Note: with three_soft's short 0.42 m cables the hover is near-horizontal; the
OCP still solves but a longer cable will give healthier thrust/tension margins.

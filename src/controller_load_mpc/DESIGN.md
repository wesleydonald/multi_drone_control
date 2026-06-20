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

## Open decisions (pending user)
1. **Payload model:** point-mass (position control, drop q/ω/J/ρ — natural for 2
   drones which can't fully steer load orientation) vs full rigid-body.
2. **Tracker:** reuse existing `controller_mpc_multi` per-drone MPC as the
   tracker (feed it the planned reference instead of the hardcoded circle) vs
   implement the paper's INDI tracker.

## Plan (once decisions made)
1. `load_cable_dynamics.py` — CasADi model (state/dynamics above, n=2).
2. `planner_ocp.py` — acados OCP (cost tracks load trajectory; tautness/thrust
   constraints), warm-start, ~10 Hz.
3. `planner_node.py` — ROS node: subscribe `/payload/...` + `/drone_N/...`,
   solve OCP, publish each drone's reference trajectory.
4. Wire references into the chosen tracker; launch on two_soft.sdf.

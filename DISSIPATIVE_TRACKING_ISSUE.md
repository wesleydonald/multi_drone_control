# Dissipative controller: load trajectory tracking

**Status:** open. Takeoff, hover, detach and attach all work. The open problem is
that the **dissipative network phase tracks a moving load trajectory noticeably
worse than the centralized OCP phase does.**

Branch `attach-week9`, last measured 2026-08-03.

---

## 1. The symptom, quantified

Circle, radius 0.5 m, `traj_speed:=0.6`, 3 drones, `dissipative_only_launch.py`.
Numbers are from the payload's logged desired vs actual position.

| quantity | value |
|---|---|
| mean payload position error | **0.202 m** |
| flown orbit radius | 0.374 m vs 0.500 desired (**0.747**) |
| payload phase lag | **+26.2°** = 0.719 s |

The payload traces a *clean* circle (scatter ~0.04 m) that is too small and too
late. It is not noisy or unstable — it is geometrically contracted and delayed.

## 2. Where the error lives — the key diagnostic

Split the chain into stages and fit a circle to each. **This is the measurement to
re-run after any change**; the script is in §6.

```
                            radius   ratio    phase lag
 payload DESIRED             0.500   1.000
 drone REFERENCE centroid    0.476   0.953    +10.1°  (0.277 s)   <- network
 drone ACTUAL centroid       0.402   0.805    +21.4°  (0.589 s)   <- tracker
 payload ACTUAL              0.374   0.747     -5.4°  (-0.148 s)  <- plant
```

Read this as:

* The **network's references are nearly right** — only 4.7% small, 0.28 s late.
* The **tracker is where the error is**: the drones fly an orbit 15.5% smaller
  than the references they were handed, and **82% of the total phase lag (0.589 s
  of 0.719 s) is between a drone's reference and the drone**.
* The **payload follows the drones faithfully** (it actually leads them slightly).
  The rods are rigid, so payload ≈ formation. Payload dynamics are *not* the
  problem, and neither is cable swing.

A 0.589 s lag from an MPC with position weight 80 is the anomaly to explain.

## 3. What has been tried

### Worked (keep)

**Horizon rollout** — `DissipativeNetwork.horizon_references()`.
The network used to publish node 0 repeated across all 21 horizon nodes, so the
tracker's 2 s lookahead was told "be at this point and stay". Now a *copy* of the
network is rolled forward along the load target's future and every node carries
its own `p, v, a_ff, a_cable`.

Why the repeat was wrong: the formation does **not** purely translate on a curve.
The virtual nodes trail the moving cone and that trailing direction rotates with
the velocity, so over one 2 s horizon at speed 0.6 the drone's offset from the
load drifts 0.35 m and `a_cable` swings by 68% of its own magnitude.

Measured effect: tracker stage 0.669 → **0.845**, payload orbit 0.647 → **0.747**.
With a constant target every node comes out identical (verified to 1.9e-30), so
hover and LAND are bit-identical to before.

*Caveat:* total payload error did **not** improve (0.201 → 0.202 m). Radius
shortfall fell 0.176 → 0.126 m but the phase error term `2R·sin(φ/2)` grew
0.136 → 0.170 m because the orbit is now bigger at the same phase lag. The fix is
real; it moved error from radius to phase. **Phase is now the dominant term.**

### Rejected, with evidence — do not retry

| attempt | result |
|---|---|
| **Tangent extrapolation** of the horizon along current velocity | Worse. A tangent leaves a circular path by 0.56 m over 2 s. Superseded by evaluating on the trajectory. |
| **`net_traj_lean`** — tilt the cone onto `g_eff = g − a_traj` | **No radius benefit** in a clean A/B (−30.3% → −30.9%), and it makes the reference contraction *worse* (0.923 → 0.899). Arithmetic says why: it can supply 0.037 m of correction against a 0.155 m deficit. Flag exists, defaults off, **recommend deleting**. |
| **Damping the nodes against the moving cone** instead of the world (`dissipative_network.py:415` damps against absolute velocity) | Never validated — the emulation used to test it double-integrated the node positions. Still an untested *idea*, not a rejected one. |
| Multiplicative node-0 box (`controller_mpc.py:906`) and `set_initial_guess` never seeding `x` | Both are **real latent bugs** (see §5) but neither causes the tracking error or the solver failures. Controlled tests: 0% failure attributable. |
| Mocap quaternion sign (`q` vs `−q`) causing infeasible solves | **Disproven.** An early "24.5% of configs fail" result was acados warm-start contamination between offline trials; 0% under a controlled test. |

## 4. Next steps, in priority order

All target the same number: the **0.589 s tracker-stage lag**.

1. **Terminal velocity reference is never set.** `acados.py:320` builds `yref_N`
   as `np.zeros(16)` and fills only position (`0:3`) and throttle (`11`) —
   **`yref_N[3:6]` stays zero**. With `W_e` velocity weight 2.0, the terminal cost
   therefore commands the drone to **stop** at the end of every 2 s horizon while
   it is flying a circle at 0.6 m/s. The stage costs do set `yref[3:6] = ref_vel[j]`;
   only the terminal was missed. The planner already publishes N+1 nodes, so the
   fix is `yref_N[3:6] = ref_vel[N_horizon]`. **Leading suspect** — found 2026-08-04,
   not yet measured.
2. **Reference staleness.** The planner publishes at 10 Hz (sim); the tracker runs
   at 50 Hz and holds the last message between updates — up to 0.1 s of pure age,
   about a sixth of the lag. The horizon carries `dt` per node, so the tracker
   could interpolate against message age instead of using node 0 verbatim.
3. **Cost weights.** `acados.py:86` — position 80/80/40, **velocity 2.0/2.0/2.0**.
   Velocity weight is 2.5% of position weight, and tracking a moving reference is
   exactly the case where it matters. One-line A/B.
4. **`CABLE_ACCEL_CAP = 6.0`** (`controller_mpc.py:78`) — measured cable accel
   reaches 15–20 during transients, so the model is wrong exactly when it matters.
   Expected to matter more for attach than for steady tracking, but test it here.

~~**`skip_steps = 3`**~~ — **STRUCK 2026-08-04.** It was assigned at
`controller_mpc.py:433` and **never read**; this package maps planner node j
directly to MPC stage j. Not a candidate, and the line has been deleted so nobody
investigates it again.

## 5. Known latent bugs, unrelated to this issue

* `controller_mpc.py:906` pins node 0 with a **multiplicative** box:
  `lbx = x*(1-0.025)`, `ubx = x*(1+0.025)`. For any negative state component this
  inverts (`lbx > ubx`, an empty box), and the pin width scales with distance from
  the mocap origin (±3 mm at 0.1 m, ±10 cm at 4 m). Should be additive.
* `acados.py:203 set_initial_guess` sets only `u`, never `x`, so the first solve
  and every post-failure recovery runs on acados' zero state guess — including a
  zero, non-unit quaternion.
* Payload sits **5–7 cm below** its target height in every run. Separate
  steady-state sag, probably thrust/tension feedforward scale. Explicitly out of
  scope per the user.

## 6. How to reproduce and measure

```bash
gz sim simulation_assets/three_rigid_ground.sdf -v 4 -r
ros2 launch controller_quad_load rviz_quad_load_launch.py num_drones:=3   # FIRST
ros2 launch controller_quad_load dissipative_only_launch.py num_drones:=3 \
    load_traj:=circle traj_speed:=0.6 traj_radius:=0.5
```

Logs (each run now self-identifying via `params.json`):

```
c_generated_code_quad_load/logs/controller_quad_load/planner_drone<N>_<ts>/   log.csv + params.json
logs/controller_quad_load/dissipative_controller_<ts>/                        params.json
```

Drone 0's `log.csv` carries `payload_x/y/z` and `payload_ref_x/y/z`; every drone
carries `pose_*` and `ref_*`. That is everything the stage split in §2 needs.

**Two rules before comparing any two runs:**

1. Check `params.json` — confirm the parameter you think you changed actually
   changed. Launch args have silently failed to plumb through before.
2. Check the sweep durations match within ~10%. Before the `use_sim_time` fix
   these varied with Gazebo's real-time factor and made comparisons meaningless.

## 7. Architectural note worth weighing

The paper being replicated (*Self-Organizing Aerial Swarm Robotics: A
Table-Mechanics-Inspired Approach*, in the repo root) shows in its **Fig. 2**
block diagram (not Fig. 3 — corrected 2026-08-04) a dissipative controller whose
output is **`vd`, a velocity command straight to the autopilot**:

```
pj, vj / pL, vL / pself, vself  ->  Dissipative Controller  ->  vd  ->  Auto Pilot
```

This repo instead turns the virtual node into a **position setpoint for a 2 s
horizon MPC**, which then re-plans against it. That extra stiff position loop is
not in the paper, and the tracker stage is exactly where the remaining error sits.

Two honest options if §4 doesn't close the gap:

* **Accept it.** Use the OCP phase for trajectory following and the dissipative
  network for what it is actually for — resilience through detach/attach, where it
  is verified and works well.
* **Restructure** the network path to command velocity, closer to the paper. This
  is a real piece of work, not a tuning change.

**CONFIRMED 2026-08-04.** The earlier caveat here (that this was read from a
poorly-extracted figure and needed checking) has been discharged: the paper's
Fig. 2 was read directly and the chain is unambiguous —

```
LIDAR  -> Relative Localization -> p_j, v_j  --+
Camera -> Load Localization     -> p_L, v_L  --+-> Dissipative Controller -> v_d -> Auto Pilot -> ESC -> Motors
DGPS   -> Pos                   -> p_self    --+                                       ^
                                        v_self ---------------------------------------+
```

There is no MPC position loop between the controller and the autopilot. One
adaptation is forced on us and should be documented as such rather than treated as
a deviation: **their autopilot accepts velocity natively; Betaflight accepts body
rates + throttle**, so the velocity→attitude→rate layer their autopilot provides
internally is ours to supply. See `general/THESIS_PLAN.md` §12.1 stage V.

## 8. Offline harnesses

`python3 -m controller_dissipative.verify_dissipative` — tests A–K (hold, detach,
attach, hand-out). All pass. **It holds a fixed `p_des` and never calls
`_network_plan`**, so it does not cover trajectory following at all and will not
catch a regression in any of this. It is a smoke test, not a guard.

Be sceptical of `mini_plant`-based results for *this* issue: its PD tracker proxy
consumes only node 0 (so it cannot see horizon effects at all) and it shows a 1.6%
radius deficit where Gazebo shows 18.5%. Two fixes were shipped on its evidence
this session and both had to be walked back.

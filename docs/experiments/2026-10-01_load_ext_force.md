# 2026-10-01 measured external force on the carried system, fed to the planner (plan B4)

Plan: `docs/plan_2026-10-02_rig_fixes.md` Part B, step B4.

History, read before this card: `2026-09-23_effective_load_force.md`, deleted 09-23.
- That estimator was a Kalman filter driven by the OCP's own planned node-0 acceleration.
- It converged to `f = -m * a_model`, the force that reconciles the plan with a load standing still, not to the real disturbance.
- So it recovered 0-2 cm of a 13-17 cm sag (wrong kT) and added a 5-6 cm oscillation.
- Its critic and reviewer: the information about a missing force is in what the drones actually deliver, not in the plan.

This card measures the force from the physics instead of from the plan.

## 1. Question
Wesley, 1 Oct: "We need to fix this height and xy error properly. TU Delft's paper and videos show that their system can respond to wind and an object getting thrown into the basket."

Can the planner hold the ring on its reference under a steady external force by planning with a measured estimate of that force? The forces in question:
- a mass added to the ring;
- a sideways push, as from wind or the room's -x drift.

## 2. Hypothesis
**Estimate.** Lumped external force on the whole carried system (drones + rods + ring), world frame, low-passed:

`w = sum_i m_i a_i + m_L a_L - M g - sum_i T_i b3_i`

- a_i, a_L: accelerations from mocap.
- b3_i: each drone's thrust axis from its mocap attitude.
- T_i: from the throttle each tracker actually sent (thr_out) and its pack voltage, through the identified affine map (rig `thrust_offset` 0.185, slope 0.022/V; sim: the plant's map).
- No term comes from the plan, so the 09-23 fixed point cannot form.

**Use.** w enters the OCP load dynamics as a constant runtime parameter:

`v_dot_L = -sum(t_i s_i)/m_L + g + w/m_L`

The plan then carries the extra tension or tilt that cancels the force. The trackers get it through the existing cable feedforward. Nothing changes in the trackers.

**Prediction:**

| case | estimator off | estimator on |
|---|---|---|
| +0.2 kg on the ring (1.96 N down) | sinks about 15 cm (from the B1 scale: 26 cm for a 25 % feedforward shortfall) | back within 3 cm of target |
| 1 N sideways on the ring | about 10 cm offset (the planner's P-only stiffness, about 4-6 N/m on the rig, inferred) | within 3 cm |
| no disturbance | - | hold moves less than 1 cm; \|w\| below 0.5 N |

## 3. The one variable
Planner parameter `ext_force_est`, default **false**; launch arg on the OCP launches (sim and real). Everything else is pinned to the B2 baseline config `b2_nocap_fix.yaml`:
- rig twin, four drones, target 1.0;
- z_ki 0, so the height integral cannot mask the result;
- cap release 2 s.

On, the estimator:
- **Where it runs:** `ext_force_estimator.py`, pure and unit-tested.
- **Acceleration:** a second-order kinematic Kalman filter per body on the mocap position (100 Hz), then a first-order low-pass on w with tau 1.0 s (about 0.16 Hz).
- **Gating:** w only updates while airborne:
  - lift started;
  - every rod taut (z_taut_gate);
  - not in LAND or descent;
  - mocap fresher than 0.1 s.

  Otherwise it is frozen. It resets to 0 at the planner phase start.
- **Bound:** each axis is limited to 0.3 m_L g (2.5 N), with a `railed` flag and a WARN.
- **Planner inputs:** the planner subscribes to `/drone_i/ELRSCommand` (throttle) and `/drone_i/telemetry` (voltage). Thrust map parameters are the trackers' own values, passed by the launch.
- **OCP:** parameter vector p grows by 3 (`p_ext`). This needs one solver rebuild per fleet size; the cache signature is tagged, with canonical zeros in `parameter_values`.
- **Tension regulariser:** the target `t_nom` follows `(m_L g - w_z)` (09-23 critic 4: otherwise the regulariser fights the extra tension).
- **Logs:** a 1 Hz line `[planner xf] w = (x, y, z) N`, and CSV columns `wx, wy, wz, w_rail`.

Disturbance tools (test-only, no world-default change):
- Gazebo:
  - test copies `four_rigid_ground_rig_wrench.sdf` and `three_rigid_ground_rig_wrench.sdf`, each with the ApplyLinkWrench system;
  - a runner event `WRENCH <link> fx fy fz` using persistent wrenches, as in `tools/sim_test/stage1_torque_test.py`.
- SIL: a plant hook for a constant force on the load (`load_force_n` event) for the cheap first pass.

Configs, written after this card:
- `configs/experiments/b4_{nodist,mass,push,orbitpush,mapbias}_{off,on}.yaml`;
- SIL `configs/sil/b4_*.yaml`.

## 4. Baseline
| arm | baseline |
|---|---|
| no disturbance | B2 runs `b2_nocap_fix` and `b2_cap075_fix` (R0819-R0822; numbers filled in when they land) |
| disturbances | the same configs with the estimator off, flown here, once each |
| rig | RIG-1001-h5 (1.01 m with the integral at its bound; xy -7.6 cm) and RIG-1001-h6 (-2.8 cm) |

## 5. Pass / fail numbers
All arms are Gazebo floor starts, rig twin, four drones, target 1.0, z_ki 0. Windows are the last 15 s of each disturbance phase.

| arm | disturbance | the effect must exist (off) | supports (on) | falsifies (on, either repeat) |
|---|---|---|---|---|
| nodist | none | - | hold within 1 cm of off; \|w\| < 0.5 N; tilt mean within 0.3 deg of off | \|w\| > 1 N (invents a force), or hold moves more than 2 cm |
| mass | -1.96 N z on the ring at hold + 10 s (= +0.2 kg) | sinks at least 8 cm | within 3 cm of 1.0 within 8 s of the step; w_z within 20 % of -1.96 N | more than 5 cm low after 8 s, or z p-p over 3 cm (oscillation), or railed |
| push | +1 N x on the ring at hold + 10 s | offset at least 5 cm | xy within 3 cm; w_x within 20 % of 1 N | more than 5 cm, xy p-p over 3 cm, or tilt mean up more than 1 deg |
| orbitpush | the orbit r 0.5 at 0.125 m/s, 1 N x throughout | lap xy err mean at least 5 cm above no-push | lap xy err mean within 1.5 cm of no-push | worse than off |
| mapbias | the plant's thrust offsets all +0.01 (the trackers keep 0.185: a 1 Oct-size map error) | hold at least 3 cm off | within 2 cm | oscillation over 3 cm p-p, or worse than off |

The mapbias arm is the case that killed the 09-23 estimator.

## 6. Repeats
- Estimator on: 2 per arm.
- Estimator off: 1 per arm. These are the effect-exists checks, not the claim.
- SIL: 1 per arm, first, as a gate. If SIL falsifies, no Gazebo runs.

## 7. Cost
- SIL: 10 runs (free).
- Gazebo: 5 arms x (2 on + 1 off) = 15 runs. That is beyond the laptop's 10 per session, so it goes on the lab PC once Tailscale SSH is re-approved, or over two laptop sessions.
- Before any of these: one Gazebo floor start with the estimator on and no disturbance (CLAUDE.md: anything touching takeoff or the floor).

## 8. New code?
Yes:
- `controller_load_mpc/ext_force_estimator.py` (new);
- OCP parameter `p_ext` (planner_ocp.py, planner_solver.py, a rebuild);
- planner_node wiring, subscriptions and the regulariser target;
- launch args;
- the Gazebo wrench event in tools/run_experiment.py;
- the two world test copies;
- the SIL plant force hook.

**Removal if it fails:** registry rows for every run, this card kept as history, and the estimator, `p_ext`, launch args and world copies deleted the same session (CLAUDE.md default). The SIL and runner disturbance hooks stay, because they are useful for any later method.

**Interactions to watch:**
- ZBias (z_ki) stays off in every arm. Whether to keep it with B4 on is a later decision for Wesley (plan B6).
- kT adaptation is locked out. This estimator does not touch kT or the trackers. It lumps any thrust-map error into w, and the planner then plans more or less pull. The mapbias arm tests exactly that. Wesley should see this before it goes near the rig.
- The rig's -x push acts partly on the drones, not only on the ring. w lumps it onto the ring, which moves the formation as a whole. That is the intended correction, but the per-drone part stays with the trackers.

## Critic
**Critic, 2026-10-01: NOT READY.** Summary of the challenge.

**Blind to internal forces.** The lumped w reads zero for any internal force error, by Newton. That covers the frozen cap, the 45 deg vs 57-65 deg rod angle, and force the trackers supply through their position error. So B4 cannot fix the 1 Oct height error (B1: R0818 sits 26 cm low at cap 0.75). Plan line "ZBias, the xy drift and the force deficit explained by one estimator" is wrong by construction.

**Static bias.** Static bias would dominate w on the rig and in the twin:

| source | size |
|---|---|
| rods missing from M (twin rods 0.075 kg each, about 2.94 N) | rails the 2.5 N bound in the nodist arm |
| 1-2 % map gain error | 0.3-0.6 N |
| 0.01 throttle offset | 0.19 N per drone |
| 1 V voltage error | 0.43 N per drone |
| +-20 g mass error | +-0.2 N per drone |
| 1 deg marker vs thrust axis | about 0.13 N per drone, horizontal |

The rig's measured map ratio of 0.95-1.19 alone is a w_z of about -4.7 to +1.2 N.

**Timing.** The throttle-derived thrust must be time-aligned with the mocap acceleration filter; that alignment is what fixed kt_trim v3. Timing is otherwise not destabilising (loop gain about 0.25 at 0.5 Hz).

**Wrong place for drone-borne forces.** The trackers already reject a force that acts on a drone. B4 then also plans -F on the ring, which settles the ring about F/k_P the wrong way: 17-25 cm for 1 N. Per-drone map scatter cannot be fixed as one ring force.

**Arms and bands.**
- mapbias (uniform offset) is the favourable case.
- Section 2 contradicts section 3 on which map is used.
- The predictions are off by the card's own scales (about 24 cm and 17-25 cm).
- Bands should be on dw (after the step minus before), not absolute w.
- LAND at 48 s is too early for the windows.
- Missing arms: a drone-borne push, per-drone map mismatch, and a gain error sized to the rig.
- The SIL plant may lack rod mass.
- B2 must be accepted first; order B3 before B4.

**Also missing.**
- Lateral rejection is resisted by w_t and w_s: build t_nom and s_ref from the balanced solution for (m_L g - w).
- Ramp w to 0 on LAND, descent and ring at rest (R0816 lean).
- State the behaviour at the rail.
- Confirm ELRSCommand is post-mux and the throttle is not clipped.
- The affine map was identified only in steady hover.

**Must-fix:**
1. Replay w offline on R0817/R0818 and RIG-1001-h5/h6/c1 first. Falsifier: rig hold |w| bias > 0.5 N on any axis, or w_x about 0 while the ring sits 8-15 cm toward -x.
2. Rod and magnet masses in M.
3. Restate the question: external forces on the ring only; B2 first.
4. A drone-borne push arm; if it confirms -F/k_P, z only.
5. The trackers' map in every arm, per-drone mismatch and slope arms, time alignment.
6. dw bands, corrected predictions, LAND time.
7. Balanced regulariser references; B3 before B4.
8. w to 0 on LAND and at rest; rail behaviour; post-mux topic.
9. Wesley's explicit word on feeding a fleet thrust residual to the planner (decisions.md:15-16, :73; 09-23 section 11.1).

## Must-fix 1: offline replay (2026-10-01 night, no runs)
Static part of the lumped force over steady holds: w = M g - sum_i T_i b3_i, with
- T_i from each tracker's thr_out - thr_off through its own map (/0.507 * g);
- b3 from mocap;
- M = n * 0.55 + 0.86.

Script: scratchpad `w_replay.py`, to move to `tools/` if B4 goes ahead.

| hold | window | w (x, y, z) N | ring xy offset (cm) | ring z |
|---|---|---|---|---|
| twin R0817 (cap off) | 15 s | +0.03, 0.00, +0.05 | +0.6, 0.0 | 1.033 |
| twin R0818 (cap 0.75) | 15 s | +0.03, 0.00, +0.03 | +0.7, 0.0 | 0.768 |
| rig h2 (0.8, yaw 0) | 9.5 s | **-0.52**, -0.11, +0.36 | -11.3, -4.2 | 0.664 |
| rig h5 (1.0, rotated 124 deg) | 15 s | **-0.43**, +0.05, -0.45 | -6.8, +2.8 | 1.023 |
| rig h6 (1.0, at x +1.1) | 1.8 s (short) | **-0.41**, -0.05, +0.44 | -2.6, +0.5 | 0.921 |

Reading:
- **The twin is blind to the cap,** as the critic predicted: w is about 0 at both cap values while the ring sits 26 cm apart. So B4 cannot replace B2.
- **The rig shows a steady -0.4 to -0.5 N force along world x** in every hold, whatever the system's orientation or position. That is about 1.0 deg of the fleet's thrust (24.6 N) tilted toward +x.
- **The size fits the drift.** At the planner's inferred stiffness of 4-6 N/m it gives 7-11 cm, which matches the measured -x offsets. So the drift is a real world-fixed force in the mocap frame, not something internal.
- **Two possible causes:**
  - a room airflow;
  - the mocap ground plane tilted by about 1 deg. Gravity then has a horizontal component in the frame the planner works in, and the plumb-line check would show 1.7 cm per metre.

  Both act on every body, drones included. That is the critic's "drone-borne" case: B4 lumped onto the ring is the wrong place for it.
- **w_z = +-0.45 N** is the fleet's thrust-map bias (about 2 %), which flips sign between holds. That bias is the noise floor any z disturbance must clear.
- **Critic's falsifier:** not met. |w| is 0.41-0.52 N, at the 0.5 N line, and w_x is clearly non-zero.

Next before any B4 code:
1. The mocap level check at the rig (a plumb line, or the wand on the floor at several points).
2. If the frame is tilted, re-level the ground plane in Motive. This is a calibration fix, not a controller change.
3. If it is level, the force is air. Then decide between a horizontal force at the formation level (all bodies) and the ring-level B4.

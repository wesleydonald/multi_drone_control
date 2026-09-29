# 2026-09-23 effective load force as an augmented state (supervisor option 3)

## 1. Question
Supervisor, via Wesley: "Effective payload force is adaptively tuned: add it to the state
vector and take the error between where we want to be and where we are as an input, which
gives how much extra mass is pulling us down. Ideally we do not adaptively tune the thrust
ratio as we go." Can the planner estimate the load force it is not modelling and plan
with it, so the rig does not need a hand-tuned kT per flight?

## 2. Hypothesis
With a disturbance force `f_d` estimated by an augmented-state Kalman filter on the load
and applied in the OCP load dynamics, a +20 % unmodelled load mass hovers within 2 cm of
the correct-mass hover (today's SIL: 9.4 cm low, R0324/R0325 0.540 vs 0.634), and a kT
10 % below the derived value hovers within 3 cm of the nominal Gazebo hover instead of
sagging. The correct-mass case is unchanged within 1 cm (no-op).

## 3. The one variable
Planner parameter `load_force_est` (launch arg on the OCP launches, sim and real),
default **false**. On:

* **Model.** The load translational dynamics become
  `v_dot = -sum(t_i s_i)/m + g + f_d/m`, with `f_d` (world frame, N) appended to the OCP
  runtime parameter vector `p = [q_ref(4), geometry(4n), f_d(3)]`. Constant over the
  horizon (`f_d_dot = 0`), so it is a parameter in the OCP, not a decision state: a state
  with zero dynamics is the same information and costs 3 QP columns per node plus a
  pinned node-0 value. One rebuild per fleet size (the parameter vector grew); the cache
  signature covers it.
* **Estimator, the augmented state.** Linear Kalman filter at the planner rate on
  `x = [p_L, v_L, f_d]`, `p_dot = v`, `v_dot = T_cmd/m + g + f_d/m`, `f_d_dot = 0`
  (random walk, process noise `q_f`); measurement `p_L` from mocap; input `T_cmd` = the
  sum of node-0 tensions the planner sent to the trackers last cycle, i.e. what the
  fleet was asked to pull. The innovation is exactly "where we want to be minus where we
  are". Bound `|f_d| <= 0.4 m g`. Vertical only by default (x,y process noise 0); a
  parameter enables lateral. Runs only while the fleet is airborne (after the lift starts)
  and resets on LAND.
* **Where it acts.** Larger `f_d` raises the OCP's planned tensions, so every tracker's
  cable feedforward rises through the existing reference path. No tracker change, no per-
  drone integrator, kT untouched.

What it is not, against §6 negatives: not a per-drone integrator (`vel_ki`, tilted the
load, fleet-level here), not an IMU residual through kT (`diss_k_adm`, biased; this uses
load kinematics from mocap and the commanded tension), not in the network (`net_pull_max`).

Configs: `configs/sil/carry_hover_n3.yaml` (baseline, now 0.86 kg), new
`configs/sil/carry_hover_n3_mass120.yaml` (plant 1.032 kg, told 0.86, `mis_seed: true`),
both with `load_force_est` off / on; new `configs/experiments/ocp_hover_ground_kt09.yaml`
(three_rigid_ground.sdf, `thrust_ratio` = 0.9 x the derived value for 0.86 kg / n=3,
`mis_seed: true`), off / on.

## 4. Baseline
Re-flown today at the new defaults (the mass and inertia defaults moved to 0.86 kg /
ring this afternoon, so R0308/R0309 at 0.6 kg are not the baseline any more):
`carry_hover_n3` off, 2 repeats, hover-only window; and the same for the +20 % plant
with the estimator off. Gazebo: `ocp_hover_ground_ring086` (pending, 2 repeats) is the
nominal-kT reference for the kT arm.

## 5. Pass / fail numbers
| arm | supports | falsifies |
|---|---|---|
| SIL, +20 % plant, est on | hover within 2 cm of the off/correct-mass hover; `f_d` settles within 20 % of 0.2 m g | still > 5 cm low, or `f_d` oscillates > 3 cm p-p in load z, or rails at the bound |
| SIL, correct mass, est on | within 1 cm of est off; `|f_d|` < 10 % of m g | > 1 cm shift or `f_d` > 10 % of m g (the estimator invents a force) |
| SIL, load never lifts (rods detached, `mis_seed`) | `f_d` rails at the bound, no abort, drones hover <= 0.1 m above their references | abort, or drones climb > 0.2 m above reference |
| Gazebo, kT 0.9x, est off | sags >= 4 cm below the nominal hover, 2 of 2 (the effect exists) | < 2 cm sag (Gazebo does not show the kT sensitivity; then the Gazebo arm is uninformative and the claim is SIL-only) |
| Gazebo, kT 0.9x, est on | within 3 cm of the nominal hover, 2 of 2, no abort | > 5 cm low or abort in either |

## 6. Repeats
2 per arm.

## 7. Cost
4 Gazebo runs (kT off/on x 2) + the 2 pending ring-world smoke runs = 6 of 6. SIL first;
if SIL falsifies, no Gazebo runs.

## 8. New code?
Yes, ~120 lines: `load_cable_dynamics.py` (+3 parameters, `f_d` in `_load_accel`),
`planner_ocp.py` (+3 zeros in `parameter_values`), `planner_solver.py`
(`set_disturbance`), new `load_force_estimator.py` (the KF, ~60 lines) with a unit test
(constant-force step converges to the true value within 3 s; bound holds; no-op stays
at 0), `planner_node.py` wiring, a log column `f_d_z`, the launch arg on the OCP launches.
Removal plan: delete the estimator file and the arg, drop the 3 parameter slots, prebuild.

Alternative the supervisor may prefer: literal state augmentation inside the OCP
(`f_d` as 3 states with zero dynamics, pinned at node 0 to the estimate). Same numbers,
more QP; offered if the parameter form is judged not formal enough.

## Critic (2026-09-23)

**1. Already built, and the card doesn't cite it.** The supervisor's input, "the error between where we want to be and where we are", is the z-only common-mode load trim `diss_ki_load` (2026-09-09, `test_load_trim.py`, `docs/design/velocity_loop.md` §11.5). Gazebo: trim off (R0079, R0081, R0155) 0.498 m, trim on (R0159–R0161) 0.600 m, radius ratio / lag / tilt unchanged. SIL R0149–R0154: 3/3 capsizes to 3/3 clean. It absorbs the −25 % mass belief in E10 (§4.7). §11.1 rejects measured-vs-commanded thrust trim as "kT adaptation by another name". This card is the same integral action rebuilt as a Kalman filter inside the OCP model.
**2. f_d is not identifiable as mass.** One measurement (load z), one scalar at steady state: mass error, kT secant error (kT derived from the *told* mass), the tautness gate and FF soft-start under-delivery, battery sag and the planner's own offset (open problem 9) are all the same additive vertical force. "f_d within 20 % of 0.2 m g" cannot be met cleanly.
**3. Commanded tension is the wrong input.** Node-0 tension is a free OCP variable; the published FF is scaled by the tautness gate × FF soft-start (`planner_node.py:784-793`), zeroed in the tracker while `payload_resting`, slew-limited (`CABLE_SLEW` 25) and scaled by `cable_ff_scale`. All of that is attributed to f_d, worst during the lift and with a short rod (gate 47 %, R0310). A scalar tension sum overstates the vertical force by 1/sin 45°.
**4. It is an integrator through the OCP, and the bands are anchored to the wrong number.** f_d drives the hover toward the 0.60 target, not the 0.634 baseline: the correct-mass arm would shift 3.4 cm and trip the card's own no-op falsifier while working. Bands must be set against 0.60. Also: the filter's assumed gain 1/m from commanded tension to load acceleration is far above the real one (integrator bandwidth q_f/R unstated); a safe gain cannot settle in the 8 s SIL hover window; `reference_builder.py` regularises tension toward `t_nom = m g/(n sin 45°)` with the baked m (w_t 5), so the OCP resists the tension f_d asks for and f_d over-winds.
**5. "Drones on reference" is circular.** The logged node-0 drone reference is `attach − l·(attach − drone)/|attach − drone|`, which equals the drone's own position whenever the rod is at length l. R0284/R0324 "on reference to 0.1 mm" is true by construction; it cannot tell planner-not-planning from trackers-not-delivering. Read the existing zN/d0N diagnostic (`planner_node.py:818-827`) from R0324 and R0284 first.
**6. Floor, detached rod, LAND.** The tracker zeroes cable FF while the payload rests, so f_d only winds up and releases as a step at breakaway (open problem 4): no arm covers it. The bench has no rods-detached-from-start option. Gate on load z above rest + 5 cm and freeze rather than reset; resetting on LAND misses the auto-descent; nothing says what f_d does on attach/detach. Drop the lateral option (trim design §11.2 rejected XY integration).
**7. The Gazebo kT arm has the sign wrong.** A kT below the truth over-throttles: 5 % lower gave +15 cm (R0230 vs R0228). At 0.9× "est off" sits high, not ≥ 4 cm low, and may abort on the descent. The arm is also §11.1(e) territory: Wesley should rule on kT compensation explicitly.
**8. It does not address today's rig failure.** Both 13:13 flights stayed on the floor: sagging 6S pack against 4S thresholds, start_taut references 18–22 cm inward while grounded, typed 0.50 against ~0.47 rods. f_d cannot lift a load on the floor.
**9. Cost and state.** Every fleet-size solver rebuilds (rig laptop too); `parameter_values` needs canonical zeros (R0297–R0301 trap); "default off" needs a no-op check against a pre-change solver; 6 of 6 Gazebo runs with no margin.
**10. Cheaper, in order.** (a) read zN/d0N from R0324 and R0284, no runs; (b) free SIL, no code: the +20 % plant with `thrust_ratio` pinned at `secant_kt(1.032, 3)`, separating kT error from OCP mass belief; (c) port the tested z-only common-mode trim to `_publish_refs` (~20 lines, no rebuild), written up as the disturbance state `f_d = m·a_I` observed by a pure integrator.
Verdict: not ready. Rewrite as integral action on load height, reuse or port `diss_ki_load`, run the free kT-pinned SIL split first.

Response (2026-09-23), before Wesley rules: accepted 3 (input), 4 (bands against 0.60,
regulariser, gain), 5 (read zN/d0N first), 6 (gating), 7 (kT sign) and 9 (rebuild, budget;
the mass defaults moved the same afternoon, so decisions.md line 13 is closed). On 1 and 10c:
the trim exists on the dissipative path only; the OCP path Wesley flies on the rig has no
load-height feedback at all, and that is the gap. Point 2 stands: the estimate is a net
unmodelled vertical force, and the card must say so instead of "mass". Not yet done: the
zero-code checks 10a/10b, and the rewrite. Wesley chose the formal (option 3) form; the
rewrite keeps the observer but takes the measured load acceleration as its innovation,
enters the OCP through `f_d` AND the tension regulariser target, gates on airborne, sets
every band against 0.60, and fixes the kT arm to 1.1x kT (fleet sags).

Critic 10a done (no runs): R0284's planner diagnostic in the hover reads `load_z=0.51
z_tgt=0.60 zN=0.60 d0N=0.97 t=4.82`, i.e. the OCP plans the load AT the target by the
end of every horizon while the measured load sits 9 cm low, with planned tension
3 x 4.82 x sin 41 deg = 9.5 N against 9.81 N of weight. The OCP is planning the move; the
trackers are not delivering the force. The force channel (cable feedforward) is therefore
the right place to act, and it is known to move the hover (gate 47 % gave -22 cm, R0310).
R0324 is a SIL run and has no planner console lines (SIL logging gap, noted).

## v2 (2026-09-23, after the critic; Wesley chose the formal form)

**Restated question.** Estimate the NET unmodelled vertical force on the load (mass
error, thrust-model error, sag: not separable, and not claimed to be) as an augmented
state, and plan with it, so a typed kT that is 10-15 % off no longer sinks the hover.

**Mechanism, as built.**
* `load_force_estimator.py`: KF on `[z, v_z, f]`, `f_dot = 0` with process noise set by
  `load_force_tau_s` (3 s), measurements z and v_z from mocap, input = the OCP's own
  planned vertical load acceleration at node 0 (gravity in, disturbance out). Bound
  `|f| <= load_force_max_frac * m g` (0.4); `railed` flag.
* OCP: `p_dist` (3, world N) appended to the runtime parameters, `v_dot += p_dist/m`.
  Parameter vector grew, so one rebuild per fleet size (done, `_ocp_signature` tagged).
  Canonical zeros in `parameter_values` (the R0297 trap).
* Tension regulariser target scales by `(m g - f)/(m g)` (critic 4).
* Gating (critic 3, 6): updates only when the load is 5 cm above its lift start, the
  FF soft-start and every tautness gate read 1, and no descent/LAND is active; frozen
  otherwise (value held, kinematics re-synced on resume). No lateral estimate.
* Log: `[planner lfe] f_d=... N (...% of m g)` at 1 Hz and the read-back parameter
  `load_force_est_value`.
* Launch arg `load_force_est` (default false) and `load_force_tau_s` on the seven OCP
  launches.

**Bands, against the 0.60 target (critic 4).** Baseline: `carry_hover_n3_long` (LAND at
30 s so a 3 s estimator has ~20 s of hover), 0.86 kg, derived kT 36.75.

| arm (SIL, free) | est off expected | est on supports | est on falsifies |
|---|---|---|---|
| correct mass and kT | ~0.63 (3 cm high, open problem 9) | within 2 cm of 0.60, `|f|` < 15 % m g | > 2 cm from 0.60 both repeats AND `|f|` > 15 % m g (invents a force) |
| kT +10 % told | sags (the effect must exist: >= 3 cm below est-off baseline) | within 2 cm of 0.60 | > 4 cm low, oscillation > 3 cm p-p, or railed |
| kT +15 % told | sags more | within 3 cm of 0.60 | > 5 cm low, oscillation, or railed |
| mass +20 % plant | ~9 cm low (R0324 analogue) | within 2 cm of 0.60 | > 4 cm low, oscillation, or railed |

Gazebo (kT +10 %, off vs on, 2 repeats each = 4 runs): only if SIL supports; 3 runs
remain today, so tomorrow.

**Repeats** 2 per arm. **Cost** SIL 16 runs (free), Gazebo 4 (deferred).

## Outcome, first SIL matrix (R0331–R0346, LAND at 30 s)

| arm | est off | est on | f_d at LAND | verdict |
|---|---|---|---|---|
| correct mass, kT | 0.629 / 0.629 | 0.579 / 0.579 | +0.32 N, still falling | partial: crossed 0.60 and ends 2.1 cm low, not settled at 20 s of hover |
| kT +10 % | 0.498 / 0.373 | 0.510 / 0.510 | −1.0 … +0.3 N | VOID (gate bug), see below |
| kT +15 % | 0.462 / 0.462 | 0.462 / 0.462 | never engaged | VOID (gate bug) |
| mass +20 % | 0.511 / 0.511 | 0.593 / 0.593 | −0.55 N | partial: 8 of 9 cm recovered; the true unmodelled force is −1.69 N |

Gate bug: "airborne" was `z > lift_z0 + 0.05`, and in the SIL air-start `lift_z0` is
the 0.45 m spawn height, so a sagging hover at 0.46–0.51 m sat on the threshold: kT +15 %
never engaged, kT +10 % toggled and oscillated (5 cm p-p). Fixed: the gate is now the
trackers' own rule, `z > payload_rest_z + 0.05` (same launch value), and the tautness
condition is `>= 0.99` (rigid rods read 0.9999). Second matrix with LAND at 46 s below.

Finding, independent of the gate bug (mass arm, valid): the load-level observer
converges to `f_d = −m · a_model`, the force that reconciles the OCP's planned node-0
acceleration with the load standing still. That is NOT the true unmodelled force,
because the trackers, holding their position references on rigid rods, already supply
most of the missing 1.69 N through their steady tracking error (the sag). The load-level
residual only sees what the trackers do not absorb, so the correction is partial and its
equilibrium depends on the trackers' stiffness. The critic's point 5 in the mechanism:
the information about the missing force is in the DRONES' position error, not in the
load's acceleration. A complete formal version therefore has to model the tracker as a
stiffness (force delivered = feedforward + K·error) or take the drone-level error as the
supervisor's "where we want to be minus where we are". Decision for Wesley after the
second matrix.

## Outcome, second SIL matrix (R0347–R0362, gate fixed, LAND at 46 s, one tree 3b33588d+17)

| arm | est off | est on | f_d at LAND | band (vs 0.60) | verdict |
|---|---|---|---|---|---|
| correct mass, kT | 0.628 / 0.629 | 0.580 / 0.579 | +0.32 N | within 2 cm: NO (2.0–2.1 cm low) | partial |
| kT +10 % | 0.498 / (0.373*) | 0.501 / 0.500 | −1.2 … +0.8 N after engaging, never settles | > 4 cm low; p-p 5–6 cm ON vs 1.8 cm OFF (the estimator adds oscillation) | not supported |
| kT +15 % | 0.462 / 0.461 | 0.483 / 0.483 | −0.21 N (2 % of m g) | > 5 cm low | not supported |
| mass +20 % | 0.511 / (0.373*) | 0.592 / 0.592 | −0.53 N (true −1.69 N) | within 2 cm: yes (0.8 cm) | partial |

\* R0358 and R0362 (and R0336 in the first matrix) never took off: the bench sends
TAKEOFF 2 s after ARM and arming finished 1.1–5.0 s after ARM in those runs, so the
fleet manager rejected TAKEOFF ("fleet is not armed"), throttle stayed at idle and the
load dangled at 0.30–0.45 m. Reviewer's finding; the harness still marked them
completed. Fixed: TAKEOFF at 5 s in the long configs and the bench now fails a run whose
console shows a rejected TAKEOFF. Until re-flown, the kT +10 % off and mass +20 % off
arms have ONE repeat each (R0357, R0361); their values agree with the first matrix
(R0335 0.498, R0343 0.511) to 1 mm on a different tree.

Falsified for the case that matters (a wrong kT, the rig's failure mode): the estimator
recovers 0–2 cm of a 13–17 cm sag and adds a 5–6 cm oscillation at +10 %. Mechanism, as
far as the traces show it: f_d reaches the filter's own fixed point `f_d = −m·a_model`
(75–87 % of it at LAND, still drifting in the correct-mass runs), i.e. the force that
reconciles the OCP's planned node-0 acceleration with a load that is not moving. That
fixed point is not z = 0.60, so the estimator is not integral action on height and
settles at 2–6 % of m g while the true deficit is 15–20 %. HYPOTHESIS, not tested by
these runs (reviewer): the missing force is being supplied by the trackers through their
tracking error over the horizon, which no logged quantity shows because the node-0 drone
reference is pinned to the drone's own position (critic 5). Whatever the channel, a
load-level observer does not see the deficit; the drone level does.

Kept: the code (default off) as the formal attempt, for the thesis discussion; v3
candidates are drone-level (bounded vertical integrator, or force feedback from mocap
acceleration), each its own card.

Reviewer (second matrix): weak as first worded. Corrections applied above: the three
unflown runs are an ARM/TAKEOFF race in the bench, not an artefact; two off arms have one
repeat until re-flown; the kT +10 % f_d range and the added oscillation; the tracker-
stiffness mechanism is a hypothesis. "The kT arm as falsified stands on the numbers
either way."

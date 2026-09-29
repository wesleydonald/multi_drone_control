# Control methods that could make the carry, attach and detach more robust (2026-09-10)

Scope: what the literature offers against the failure modes THIS stack has actually shown
(CURRENT_STATE.md, results R01xx–R02xx), ranked by evidence and by what can be proven on the
existing ladder (offline harness → SIL → Gazebo → rig) before 27 Nov. Not a general review;
the lit review already exists.

## 1. What breaks today, and why (the targets)

| # | Observed failure | Stage | Root cause in the current architecture |
|---|---|---|---|
| F1 | Load height wrong by +18 cm at 0.6 kg until kT was re-derived; mass **over**-estimate of 25 % is fatal within 10 s of the weld, under-estimate is benign (R0215–R0222) | carry, attach | The tracker flies an **open-loop modelled cable force** (planner t·s/m) and an integrator-free position loop: every model error (kT, mass, geometry) lands in position, and the only correction is the z-only common-mode trim in the network |
| F2 | Weld transient 24° mean / 47° peak tilt over the reconfiguration; 45° settle collapses the newcomer every time | attach | The newcomer's actual cable force is whatever its position error makes it; the soft hand-out ramps a *commanded* share, nothing measures the *delivered* share |
| F3 | Three-drone 3/12/9 hover hangs at 34° and is only level once the 4th is on; the true-attitude wrench self-amplifies tilt | carry | Tension solve is a moment balance in a yaw-only frame; there is no feedback on load attitude itself |
| F4 | Network detach 3→2 capsizes in SIL; OCP-resize detach is ~1° but needs a re-solve | detach | Survivors' thrust follows the commanded reallocation, not the load they suddenly carry |
| F5 | Reference-stale disarms, weld mis-captures, hold timers | all | Event handling is ad hoc: hold-until-weld, blend windows; no dwell-time or invariance argument |

Everything below is chosen because it attacks one of these with a mechanism, not a gain.

## 2. Candidates, ranked

### R1. Measured-force feedback in the velocity loop (INDI-lite / tension-to-thrust feed-forward) — RECOMMENDED

**Idea.** Replace the open-loop cable acceleration with the *measured* residual: from the
IMU specific force (already subscribed, `_imu_callback`; `cable_source=measured` exists in
the MPC tracker) or, equivalently, from a cable-tension estimate. In the velocity loop the
thrust command becomes incremental, u = u_prev + (a_cmd − a_meas)/kT, so a wrong kT,
mass, or cable geometry produces a transient, not a steady offset. This is exactly what the
two reference stacks do: Sun et al. track through INDI with IMU-measured external force
([Science Robotics 2025](https://www.science.org/doi/10.1126/scirobotics.adu8015)), and the
May-2026 "passive fault tolerance" paper shows that feeding **measured tension straight
into thrust** lets a decentralised fleet survive an *unannounced* cable loss within one
pendulum period, with a hybrid input-to-state-stability certificate
([arXiv 2605.05339](https://arxiv.org/pdf/2605.05339): disabling it raised cruise error
34–39 % and peak sag 3.6–4×, five UAVs, 10 kg load).

**Why it fits all three stages.** F1: kT and mass errors stop being position errors. F2:
the newcomer's hand-out becomes a ramp on a *measured* share, so its cable force tracks
the feed-forward the incumbents assume. F4: after a detach the survivors' measured tension
jumps and their thrust follows automatically, before any reallocation arrives — the
paper's whole point.

**What went wrong before, and why this is different.** The August attempt fed IMU f_ext
into the *MPC* held constant over the horizon and the short-horizon RTI could not hold
the mismatch (CURRENT_STATE.md §10). The velocity loop has no horizon; INDI is designed
for exactly this one-step incremental form. The known INDI risks are IMU noise and the
actuator-lag synchronisation of a_meas with u_prev; the learned-INDI paper
([arXiv 2503.09441](https://arxiv.org/pdf/2503.09441)) exists because rotor-RPM sensing is
usually unavailable, so the filter design is the work.

**Proof ladder.** SIL bench publishes IMU already. (1) `mini_plant`/SIL hover with kT set
25 % wrong: height error must fall from +18 cm to a transient. (2) Re-run the mis-seed
sweep (`attach_circle_n3_m075`): mass +25 % must go from fatal to benign. (3) SIL
`carry_trim_n3` detach 3→2 and Gazebo round trip R0231: post-detach tilt. (4) Falsifier:
if the loop rings at the IMU filter bandwidth, the actuator-sync is wrong, not the idea.

**Cost.** ~2 days: a filtered a_meas path in `velocity_loop.step`, one parameter
(`vel_indi_gain` 0→1), SIL configs. Thesis value: a mechanism-level robustness result
with a literature theorem behind it.

### R2. Tension allocation as a bounded QP, with the hand-out as a constraint ramp

**Idea.** The current balanced solve is an unconstrained least-squares moment balance
with a blend for the newcomer. Wahba & Hönig
([RA-L 2024, arXiv 2304.02359](https://arxiv.org/pdf/2304.02359)) and the Feb-2026 SQP
allocation ([arXiv 2602.04801](https://arxiv.org/pdf/2602.04801), milliseconds per solve)
minimise Σtᵢ² subject to tᵢ ≥ t_min (no slack), tᵢ ≤ t_max, and a cable-separation
penalty. The newcomer's upper bound ramps 0 → t_max over T_handout, which *is* the soft
hand-out but with slack and saturation guarded by construction.

**Targets.** F2 (no incumbent goes slack while the newcomer takes share), F4 (a detach is
a constraint removal; the QP redistributes with bounds), and it is what
`_solve_tensions` already almost is. F3 is not addressed: the QP still needs a level
frame to balance in.

**Proof ladder.** Offline harness G/K (tension share), SIL attach; compare weld-transient
tilt and minimum incumbent tension against R0234. Cost ~1–2 days (scipy or acados QP;
21 horizon nodes × 33 ms tick is the budget).

### R3. Load-level extended-state observer (common-mode disturbance estimate in xy and attitude)

**Idea.** The z-only trim (§11 of velocity_loop.md) is a one-axis integral ESO. The
decentralised geometric controller of Jul-2026
([arXiv 2607.00024](https://arxiv.org/pdf/2607.00024)) pairs concurrent-learning **mass
estimation** with an ESO for wind, without knowing the number of agents; the prescribed-time
UIO / finite-time ESO papers for four-quadrotor transport do the same centrally
([ScienceDirect 2025](https://www.sciencedirect.com/science/article/abs/pii/S0921889025002817)).
Extending the trim to the load's xy and roll/pitch as a *common-mode* estimate keeps the
property that made z-only safe (no differential bias between drones) while removing the
34° hang's steady moment error.

**Targets.** F3 directly, F1 partially. Risk: the true-attitude wrench experiment
(R0212/R0213) shows attitude feedback can self-amplify; an ESO with a slow time constant
and the yaw-only frame kept for the formation is the safe version. Cost ~2 days; the §11
falsification ladder applies unchanged.

### R4. Attach/detach as a hybrid system: reset map + dwell time, with an ISS bound

**Idea.** Formalise what the stack already does. The weld is a jump: the bumpless xy datum
shift and the tension blend are the *reset map*; the trajectory hold is a *dwell time*.
The severance paper's certificate ([arXiv 2605.05339](https://arxiv.org/pdf/2605.05339)) is
built from exactly these pieces (post-jump Lyapunov increase, inter-event decay,
per-cycle contraction ρ<1), and the single-quadrotor lift literature models the
slack→taut transition the same way
([hybrid lift, IEEE 2015](https://ieeexplore.ieee.org/document/7171008/)). Using the
measured post-weld tilt decay from R0208/R0209/R0234 gives the decay rate; the hold time
becomes a computed minimum dwell instead of 10 s by trial.

**Targets.** F5, and it turns two tuned numbers into a result with a bound. Cost ~1 day of
analysis plus a figure; no new control code. Best paired with R1 (the certificate assumes
tension feed-forward).

### R5. Safety filter (control barrier function) on tilt and tension instead of a disarm

**Idea.** The envelope currently disarms the fleet at 60° tilt. A CBF layer on the load
tilt and on minimum cable tension, with a disturbance estimator setting the margin
([Yang & Xie 2025](https://www.sciencedirect.com/science/article/abs/pii/S0967066125003260)),
would modulate references before the limit instead. Robust in the literal sense, but it
is a third loop around a stack that already has three, and the sim runs that hit 60° were
already unrecoverable. Not recommended before 27 Nov; note it as future work.

### R6. Newcomer approach as an impedance / contact-transition problem

**Idea.** The perching literature treats the free-flight → contact switch with an
impedance law and a contact-force observer so the vehicle neither bounces nor drags
([Scientific Reports 2026](https://www.nature.com/articles/s41598-026-36857-9),
[impedance switching](https://www.researchgate.net/publication/282582335_Unified_Switching_between_Active_Flying_and_Perching_of_a_Bioinspired_Robot_Using_Impedance_Control)).
Our weld already goes through a compliant rod and a tip-velocity gate; the collaborator's
approach MPC is not ours to rewrite. Low priority; R1 covers the post-weld half of this.

## 3. Not recommended (and why)

- **Tube / robust MPC** on the tracker ([tube MPC, tiltrotor cargo](https://link.springer.com/article/10.1007/s40313-024-01129-2)): the uncertainty set would have to include kT, mass and the cable model; explicit tube synthesis for a 17-state nonlinear tracker is a project of its own, and R1 removes the dominant uncertainty at far lower cost.
- **Learning-based control** (learned INDI, contraction-based neural control): no sim-to-real budget before the rig campaign.
- **Full geometric SE(3) rigid-body payload control** (Lee 2014 and descendants): a different architecture, not an augmentation; the OCP + network stack is the thesis.

## 4. Recommended order

1. **R1** measured-force velocity loop — the one change that touches all three stages and
   converts the mis-seed asymmetry (the strongest negative result on the branch) into a
   positive one. Gate it on the mis-seed sweep.
2. **R4** hybrid framing — free analysis on data already in `results_archive`, gives the
   attach chapter a bound.
3. **R2** bounded tension QP — if the weld transient is still >20° after R1.
4. **R3** load-level ESO — only if the three-drone hang matters for the demo (with the
   4th drone the load is level anyway).

Sources: [arXiv 2605.05339](https://arxiv.org/pdf/2605.05339) · [arXiv 2607.00024](https://arxiv.org/pdf/2607.00024) · [arXiv 2602.04801](https://arxiv.org/pdf/2602.04801) · [arXiv 2304.02359](https://arxiv.org/pdf/2304.02359) · [arXiv 2503.09441](https://arxiv.org/pdf/2503.09441) · [Science Robotics 2025](https://www.science.org/doi/10.1126/scirobotics.adu8015) · [arXiv 2410.23929](https://arxiv.org/pdf/2410.23929) · [ScienceDirect 2025 CBF+DE](https://www.sciencedirect.com/science/article/abs/pii/S0967066125003260) · [ScienceDirect 2025 PPC](https://www.sciencedirect.com/science/article/abs/pii/S0921889025002817) · [IEEE 2015 hybrid lift](https://ieeexplore.ieee.org/document/7171008/) · [Springer 2024 tube MPC](https://link.springer.com/article/10.1007/s40313-024-01129-2) · [Sci. Rep. 2026 perching](https://www.nature.com/articles/s41598-026-36857-9)

## 5. Status (branch `measured-force-velocity-loop`, 2026-09-10)

R1 implemented (`vel_indi_gain`, design note `docs/design/measured_force_loop.md`) and R4
measured (`tools/hybrid_dwell.py`, `attach_traj_hold_mode: settle`). SIL: R1 removes the
tracker's model-error offset exactly (wrong kT: drone z error 0.21 m → 0.00), and shows
that the remaining capsizes in the wrong-model scenarios are network-level (allocation,
attitude feedback), which is where R2/R3 pick up. Gazebo mis-seed and settle-dwell runs:
see CURRENT_STATE.md for the run ids.

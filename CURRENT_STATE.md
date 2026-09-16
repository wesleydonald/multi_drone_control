# CURRENT STATE — multi_drone_control

Updated 2026-09-16. This is the one document to read first. It condenses the chronological
logs that used to live in `learning.txt`, `update.txt`, `progress.txt` and `controllers.txt`
(removed 2026-09-16; still in git history). Every number cites a run id; runs live in
`results/<date>/R####_*/`, the index is `results/registry.csv`.

Companions: `general/THESIS_PLAN.md` (the plan, Aug 2026, partly superseded),
`general/TERM3_STRATEGY.md` (schedule re-baseline, 9 Sep), `docs/design/*.md` (design
notes with derivations), `docs/experimentation/*.md` (findings), `docs/decisions.md`
(dated decisions, OPEN lines wait on Wesley), `README.md` (install, build, launch).

---

## 1. What this is

A fleet of 2–4 Betaflight quadrotors carries a cable-suspended payload cooperatively.
Drones can **detach** from the payload mid-flight and a free drone can **attach** to it
mid-flight, so the fleet reconfigures without putting the load down. Mid-flight physical
attach is the thesis contribution. Submission Thu 27 Nov 2026.

| Piece | Source | Role |
|---|---|---|
| Centralized OCP planner | Sun et al. 2025, arXiv 2501.18802 (`docs/papers/`) | trajectory following, takeoff (ground break-off), detach via fleet resize |
| Dissipative virtual-node network | Quan et al. 2025, arXiv 2509.03563 (`docs/papers/`) | hold and reconfigure; a member leaving or joining is its native mode |
| Physical attach: rendezvous, weld, authority handover, reconfiguration | this thesis | a free drone with a swung electromagnet welds onto the carried payload and joins |

The approach drone, its magnet and its approach controller are a collaborator's (Tejen).
This project's controllers take over at the weld.

Payload: a 500 mm disc, 0.6 kg in sim, rim magnets placeable anywhere. Rig mass,
thickness and rim azimuths are still unmeasured.

---

## 2. Architecture

Two reference generators feed one unchanged per-drone tracker over one wire format.

```
mocap (/drone_i, /payload motion_capture_state)
        |                                   |
  OCP PLANNER (10 Hz)              DISSIPATIVE NETWORK (10 Hz)
  controller_load_mpc              controller_dissipative (subclasses the planner)
        +----> /drone_i/reference_trajectory <----+
               [n_nodes, dt, p(3), v(3), a_ff(3), a_cable(3)] x (N+1)
                          |
             PER-DRONE TRACKER (50 Hz) controller_quad_load
             control_mode: mpc | velocity | velocity_after_handover
                          |
                   /drone_i/ELRSCommand  ->  sim betaflight emulator | real ELRS
```

- **Phase dispatch.** The dissipative node runs the OCP for creep, lift and carry, and
  switches to the network on a detach, a weld, or `auto_network_handover`. Takeoff is
  always the OCP; the network cannot break the load off the ground.
- **Tracker modes.** `mpc` is the original cable-aware acados tracker (no integrator,
  cable feedforward capped at 6 m/s²). `velocity` is the paper's architecture: a
  velocity loop with attitude and rate layers (`velocity_loop.py`), which cannot fly the
  lift. `velocity_after_handover` flies MPC through the lift and the velocity loop after
  the network engages. This is the default in `three_attach_launch.py`.
- **Common-mode load trim** (`diss_ki_load`): one z-only integrator on measured load
  height, distributed identically (or by tension share with
  `diss_trim_share_weighted`) to every attached node. Per-drone integrators tilt the load.
- **Measured-force throttle** (`vel_indi_gain`, opt-in): incremental throttle on the IMU
  residual. Exact at the drone, destabilises a moving load at gain 1 (§4.4).
- **kT is fixed.** Sim launches derive it from the operating point
  (`thrust_ratio:=auto`, `thrust_model.py`); hardware keeps the measured 24. A wrong kT
  lands in payload height, not throttle.
- **Detach** has two modes: `reconfig_mode:=network` (network redistributes, then hands
  back to a pre-built OCP for the new n) and `reconfig_mode:=ocp` (the OCP resizes
  directly; attach geometry is a runtime parameter so the survivors keep their true
  azimuths).
- **Attach chain** (`drone_magnet`): join planner, target publisher, magnet manager,
  `elrs_mux` (forwards the approach controller's commands until the weld, then ours;
  never hands authority to a stream that is not commanding thrust).

---

## 3. Status by capability

| Capability | State | Evidence |
|---|---|---|
| OCP carry: takeoff, hover, line, circle, fig-8, n=2–4 | works in sim; the baseline | gate SIL smoke R0232 settles 0.598 m on a 0.60 target |
| Detach, OCP fleet resize (`reconfig_mode:=ocp`) | works; the stronger mode | R0111–R0114 |
| Detach, network then OCP hand-back | works; larger transient | R0108, R0109 |
| Dissipative trajectory tracking | resolved by `velocity_after_handover` + common-mode trim | §4.4 table 1 |
| Attach during a circle, 3/12/9 layout (the demo) | works in sim, repeatable | R0196, R0197, R0234, R0243 |
| Attach round trip (attach, resume, detach the newcomer) | clean | R0231 |
| Attach on a fig-8 | clean | R0226 |
| Attach on the 4/12/8 layout | level hover, rolled circle; layout OPEN | R0266–R0272 |
| Attach during a circle, measured-force throttle | opt-in; best weld transient, aborts on the moving load at gain 1 | R0244, R0247, R0257 |
| Robustness to parameter mis-seed | asymmetric envelope, table in `docs/experimentation/robustness_table.md` | R0215–R0222 |
| Hardware carry, detach, attach | **not started**; launches and rig checklist exist; takeoff bugs fixed on the rig 2026-08-03 | — |

---

## 4. Results

### 4.1 Detach

One-flag A/B, n=4 → 3 during a circle, `detach_ocp_n4` vs `detach_network_n4`:

| metric | OCP resize | network hand-off |
|---|---|---|
| settled load tilt | ~1° | 26–48° |
| survivor tracking error | ~1 mm | 60–910 mm |
| runs | R0111–R0114 | R0111–R0114 |

Network-then-hand-back sequence (hold trajectory, redistribute, prime the new solver, hand
back): definitive runs R0108/R0109 held mid-sweep, resized to n=3 on azimuths
[90, 180, 0], resumed. Minimum payload height after the detach 0.46 m (was 0.07 m before
the two fixes: do not re-zero the feedforward soft-start on an airborne hand-back, and
prime the new solver twice before handing over). Known: LAND with a parked detached drone
aborts after the trajectory completes. A 3 → 2 detach in the SIL bench capsizes because
two support points leave the CoG outside the support line; Gazebo 3 → 2 survives because
survivors re-space first.

### 4.2 Attach during a circle (the demo)

Sequence: three drones fly the circle at 0.2 m/s, ATTACH freezes the load target, the
approach welds the stationary payload at 6 o'clock, bumpless handover to the network, the
load levels, the trajectory resumes with four drones, LAND.

| run | config | pre-weld hover | weld transient | four-drone circle | notes |
|---|---|---|---|---|---|
| R0196, R0197 | first two clean runs, 3/12/9, hold 15 s | 27° tilt (geometry) | 39° → 4° in 11 s | 4–15° tilt, z 0.60 | LAND 78 s |
| R0203, R0204 | fast: hand-out 8 s, hold 10 s | — | level in ~8 s | completed | ~50 s takeoff to finish |
| R0234 | classic loop baseline, kT auto | 0.569 m | 47° peak, 24° mean | 3.7° mean, z 0.601 | the registry baseline |
| R0243 | `attach_traj_hold_mode: settle` | — | hold ended itself at 13.6 s, tilt 1.0° | 3.6° | resume window 1.2° mean vs 2.5° timed |

Fixes that got it there (each from a run): newcomer tracker in velocity mode; weld point
captured from the magnet tip, rotated by payload attitude; ATTACH implies ARM in the
approach node; hold arms once per approach and lasts until the weld; bumpless xy
handover (datum shifts to the measured load at network entry); settle elevation 65° (45°
collapses at hand-out end); the network ring built from the physical tethers plus a
gap-centre placeholder (it was a 0/90/180/270 ring against 0/120/240 tethers); symmetric
hand-out (incumbents' tension solution blends over the same ramp as the newcomer);
`handover_blend_s` for the incumbents' cable feedforward.

Three drones at 3/12/9 hang ~27–34° tilted by geometry (CoG on the 3–9 chord). The fourth
drone is what makes level flight possible. n=2+1 rim attach is not flown: a disc on two rim
cables is a pendulum about their chord.

### 4.3 Layout: 3/12/9 vs 4/12/8 (OPEN)

| layout | three-drone hover | weld | four-drone circle | run |
|---|---|---|---|---|
| 3/12/9, newcomer 6 | 33° tilt | 47° peak, 12 s to settle | 3.7° mean, level | R0234 |
| 4/12/8, newcomer 6, classic loop | 2.0° | 10° mean first 4 s | rolls 20–37°, low side at 12 o'clock | R0266 |
| 4/12/8, INDI 0.5 + swing | level | 7° first 4 s | 16 → 22° mean, abort +30 s | R0268 |
| 4/12/8, INDI 0.5 + swing + share trim | level | 4.9° at +10–16 s | 21–23° | R0272, first completed run on this layout |

Mechanism: on the 12/4/6/8 rim the 12 o'clock drone carries ~40 % of the load and the
other three 20 % each. A no-integrator tracker sags in proportion to force error, so the
common-mode trim cannot level it. Neither layout gives a level hover and a level circle today.

### 4.4 Tracker variants on the attach demo

Table 1, Part A architecture study on `diss_circle_n3` (n=3 drones, circle r=0.5 at 0.6 m/s,
medians over 5 and 3 runs):

| metric | MPC tracker | velocity loop, `vel_ki 0` | + common-mode trim |
|---|---|---|---|
| payload radius ratio | 0.816 | 1.001 | unchanged |
| payload phase lag | 0.676 s | 0.514 s | unchanged |
| payload RMSE (sweep) | 0.272 m | 0.215 m | 0.215 m |
| payload height | 0.498 m | 0.498 m | 0.600 m |
| post-sweep tilt | 1.4° | 5.4° | unchanged |

Table 2, measured-force throttle (INDI) on the attach demo, 3/12/9:

| config | weld +0–4 s mean/max | weld +4–10 s mean | four-drone circle mean/max | outcome | run |
|---|---|---|---|---|---|
| classic loop | 28.8 / 47 | 28.8 | 3.7 / 9.4 | completes | R0234 |
| INDI gain 1 | — | 7.1 | — | ~1 Hz tilt growth, abort +18 s | R0244 |
| INDI 0.5 | 32.8 / 38 | 17.4 | 8.5 / 16 | completes | R0247 |
| INDI 0.5 + anti-swing 0.3 | — | 18.2 | 5.7 / 11.1 | completes; best compromise | R0257 |
| INDI 1 + anti-swing, mass +25 % mis-seed | — | — | 21–22 mean, 50 max | first full run of this mis-seed | R0255 |

Mechanism at gain 1: INDI makes each drone a stiff position servo, the load hangs from four
fixed points on rods whose lengths the network only approximates, rods go slack, a load
on intermittently slack rods is a pendulum. The compliant MPC tracker kept rods taut by
sagging into the load. The newcomer's reference leads the physical load by 0.4 m after the
resume in every run (network pins its payload node to the desired position); measured
rod length (0.490 m) and a horizontal leash both failed to fix it (§6).

### 4.5 Thrust ratio and hover height

The OCP hover parked the load 12–18 cm high at 0.6 kg because the fixed sim kT (32.9) was
the secant at the 0.4 kg operating point. SIL A/B on `carry_hover_n3`, identical throttle
0.391:

| kT | settled load z (target 0.60) | run |
|---|---|---|
| 32.9 | 0.782 | R0230 |
| 33.0 | 0.773 | R0229 |
| 34.6 | 0.632 | R0228 |

`thrust_model.py` derives kT per launch from load mass, fleet size and cable elevation and
reproduces both measured hover points to 1e-3. Gate smoke now settles at 0.598 (R0232).

### 4.6 Attach faults and the weld-mechanics ladder (Aug 2026)

Three stacked faults, each masking the next, all fixed: the mux handed authority to an
armed-idle stream so the newcomer's motors cut at every weld; the approach drone spawned on
a corridor through drone 0; the pose watchdog ran on the wall clock in a 0.3× sim.

Weld ladder, welded runs, payload tilt after the weld:

| variant | survival s | tilt +2 s | tilt +8 s | runs |
|---|---|---|---|---|
| ball baseline | 10.1–10.2 | 46–49 | 31–34 | R0126, R0128 |
| rod (proper arm inertia) | 10.4 | 47 | 36 | R0129 |
| seg2 (extra mid-span joint) | 10.0–10.3 | 40–44 | 26–35 | R0132, R0133 |
| rigid (moment-transmitting) | 1.7–1.9 (5/5) | 83–106 | — | R0135, R0138–R0141 |

Compliance does not help; a genuinely rigid weld capsizes 5/5. Decision (9 Sep): the
attachment is modelled as a ball joint after the weld, the `rod` world is canonical, and
the rigid-weld control problem is out of scope.

### 4.7 Robustness to a wrong controller belief (E10)

Attach-during-circle demo, world true, controller mis-seeded, two runs each:

| belief | result | runs |
|---|---|---|
| mass −25 % | clean 2/2 | R0215–R0222 |
| mass +25 % | abort 2/2, load over-lifted to 0.96 m | |
| cable −10 % | clean 2/2 | |
| cable +10 % | abort 2/2, load dropped to 0.17 m | |

Under-estimates are absorbed by the common-mode trim; over-estimates are fatal within ~10 s
of the weld. Rig rule: weigh the payload and measure the rods, err low.

---

## 5. Locked decisions (do not "fix")

- No geofence: the pilot at the cage owns out-of-bounds. Kept: reference staleness, tilt
  and speed envelopes, fleet-wide disarm on any envelope fault.
- kT is fixed. Sim derives the secant at launch; hardware uses 24. The adaptive UKF and the
  airborne kT schedule were deleted.
- The network is yaw-only and open-loop in load pose except the bounded common-mode trim.
  Building geometry on the measured load attitude or position closes a loop that
  self-amplifies tilt (§6).
- Attach at 0.2 m/s. 0.4 m/s flips the load at the weld (R0267).
- The attachment behaves as a ball joint; N3 is stated around what the rig does.
- The reference wire format is frozen at 12 fields per node.
- New behaviour goes behind a parameter defaulting to the old behaviour.
- `results/` is single-copy; anything behind a figure is promoted to `results_archive/`
  the same day.
- 2026-09-09: payload is the 500 mm disc, 0.6 kg in sim, rim magnets.
- 2026-09-10: tethers at 4/12/8 in `three_attach.sdf` and the launch defaults; RViz role
  colours and error lines removed, banner kept.

---

## 6. Negatives — measured, do not retry

| idea | result | evidence |
|---|---|---|
| Terminal velocity reference (T1) | slightly worse on all four tracking metrics | R0054–R0059 vs R0061–R0065 |
| `net_traj_lean` (tilt the cone onto g_eff) | no radius benefit, contraction worse | Aug 2026 A/B |
| Per-drone integrators (`vel_ki 1`) | tilt the load to 60°; `vel_ki 0` keeps the tracking win | R0077 series |
| Tilt-aware wrench (`diss_wrench_true_attitude`) | weld transient 68°, abort 2/2 | R0212, R0213 |
| Extra weld compliance (seg2) | no change in outcome | R0132, R0133 |
| IMU-residual tension admittance (`diss_k_adm`) | worse in Gazebo (biased estimate through a secant kT) | R0250, R0251 |
| Horizontal leash (`net_pull_max`) | closes the position loop; classic loop aborts, newcomer never welds | R0258, R0259 |
| Measured rod length as the network length | no benefit; the 0.53 m reading was drone-to-rim with the tip stand-off | R0261–R0263 |
| Classic loop + anti-swing | no benefit; the compliant tracker already has the damping | R0256 |
| Full-gain INDI + anti-swing | capsizes after the weld on 4/12/8 | R0269 |
| OCP hand-back with an even n=3 ring after 4 → 3 | 62° tilt; survivors keep their true azimuths instead | R0093 |
| SIL bench as an attach discriminator | 45° and 65° both diverge on the current plant; use Gazebo | R0034–R0043 |
| Offline harness (`verify_dissipative`, PD proxies) for tilt questions | holds level where Gazebo tilts 34°; smoke test only | — |

---

## 7. Open problems, priority order

1. **Layout default** for the demo: 4/12/8 (level hover, rolled circle) or 3/12/9 (tilted
   hover, level circle). OPEN in `docs/decisions.md`.
2. **Moving-load stability with a stiff tracker.** Slack rods at INDI gain 1; the
   newcomer's 0.4 m reference lead after the resume. Candidates: a slack-aware tension
   solve with a t_min bound, or softer position gains with exact thrust.
3. **Make `attach_traj_hold_mode: settle` the default?** OPEN.
4. **Lift transient variance.** The n=4 circle lift is bimodal (R0096 4° vs R0098 51°);
   `diss_circle_n3` lost 4 of 10 runs in the OCP lift. Undiagnosed.
5. **LAND with a parked detached drone aborts** after the trajectory completes.
6. **A drone that disarms before takeoff does not trigger the fleet abort**, producing junk
   that looks like flight data.
7. **~18° residual tilt after 4 → 3** on the uneven surviving ring. Possibly geometry
   (`metrics.cog_margin`), not a bug.
8. **Hardware: zero rig runs.** This is the critical path.

---

## 8. Plan to submission

Results freeze Fri 7 Nov 2026. Submit Thu 27 Nov 2026.

| window | milestone |
|---|---|
| Sep 15–26 | attach hand-out and layout decision; hardware H0 bring-up and H1 two-drone carry |
| Sep 29–Oct 17 | hardware carry and detach campaign; attach matrix in sim |
| Oct 20–Nov 7 | hardware attach window, gap-filling, demo capture; freeze |
| Nov 10–21 | full draft, supervisor review, revisions |
| Nov 24–27 | final revisions, submit |

Rig access about 2 days a week. The lit review is drafted in LaTeX outside the repo.
Chapters are written the week their method freezes.

Descope ladder, in order, if the clock wins: drop the failure-redistribution study; drop
scalability beyond n=2/4 spot checks; fig-8 only where already flown; one perturbation per
axis in robustness; hardware attach steps down moving weld → hover weld → approach without
weld → sim-only attach. The floor: hardware carry and detach, the sim attach boundary,
the architecture study, honest sim-to-real accounting.

---

## 9. Hardware

Launches: `real_io_launch.py` (mocap + ELRS + RViz, terminal 1, must be up first),
`real_control_launch.py` (OCP), `real_dissipative_launch.py`, `real_attach_launch.py`
(twin of the sim attach launch, same 69 arguments; `tools/param_diff.py --sim-vs-real`
shows only the intended deltas). Desk test without the rig: `tools/fake_mocap.py`.
Pre-arm checklist: `tools/preflight.py --real`. Rig checklist for attach:
`docs/experimentation/real_attach_gap.md`.

Rig facts: mocap rigid bodies 10/20/30 → drones 0/1/2, payload body 8
(`RIGID_BODY_TO_DRONE` at the top of `motion_capture_publisher_node.py`); the magnet tip
needs its own rigid body; `thrust_ratio:=24`; `start_taut:=true` on real launches (no
creep); `load_mass:=<weighed> cable_len:=<measured>` every session.

Test order, earned in August: hold (`lift_ramp_vel:=0.0`) to isolate thrust, then hover
at `lift_ramp_vel:=0.15`, then shuffled and yawed starting placements (the load yaw datum
must print in the planner log), then line, circle, fig-8, spin at 0.2 m/s, then detach,
then attach last.

Real-world bugs already fixed (2026-08-03): slot assignment in the load frame (yawed
payload made the QP infeasible), latched payload yaw datum, latched per-drone heading (a
drone did a 360° on takeoff), trackers honour `use_sim_time`.

Raw commands without RViz:

```bash
ros2 topic pub -t 3 /fleet/command std_msgs/msg/String "{data: ARM}"      # ARM|TAKEOFF|LAND|DISARM|ESTOP
ros2 topic pub -t 3 /fleet/detach  std_msgs/msg/Int32  "{data: 1}"
ros2 topic pub -t 3 /magnet/command std_msgs/msg/String "{data: ON}"      # = the RViz ATTACH button
```

Shell helpers (in `~/.bashrc`): `mdc` sources both setups; `cb [pkg]` builds; `rio`,
`rctl`, `rdiss` are the real launches; `sgz <world>`, `sviz`, `smpc`, `sdo`, `sdet`,
`satt` are the sim ones; `simcheck <world> <launch>` runs the geometry check.

---

## 10. Traps and hard-won lessons

- **Read the parameters back.** Launch args have silently failed to plumb through three
  times (net_traj_lean, control_mode on the newcomer, a cut command line). Every run
  writes `params/` by read-back; check it before believing a comparison.
- **Same wall-clock duration or no comparison.** Trackers once ran on wall time while the
  planner ran on sim time; RTF then set the control rate.
- **`tools/prebuild_planner.py` after touching the planner model, masses or inertia.** The
  SIL bench starts its clock before the solver builds; an interrupted build deletes the
  old `.so` and never writes the signature, which presents as "the planner publishes no
  references".
- **Start Gazebo from inside `simulation_assets/`** (relative model paths) and **RViz
  launch first** (it owns the pose bridges and mocap emulators).
- **`tools/clean_slate.sh` between runs.** Leaked nodes and Fast-DDS shared memory fake
  divergence.
- **Wall-clock watchdogs in a 0.3× sim** (pose timeout, reference staleness) fire on
  healthy fleets under CPU load. Sim launches carry longer budgets and headless Gazebo
  runs niced. Do not run analysis while a batch flies.
- **`dissipative_launch.py` never engages the network** unless a detach or attach fires;
  `dissipative_only_launch.py` does.
- **The offline harness and the SIL bench are necessary, not sufficient.** PD proxies
  settle cases Gazebo capsizes; the bench has no rotor vibration and an exact plant.
- **The tracker has no integrator and trusts the cable feedforward.** Any transient where
  the real cable force exceeds the modelled one under-thrusts the drone. Fixes are
  reference-side (keep actual ≈ modelled) or the common-mode trim.
- **Physics that tuning cannot remove:** ~13 m/s² of thrust authority after gravity; three
  fixed tethers cannot form an even 4-gon under equal shares; a centre weld cannot be a
  ring member; a disc on two rim cables is a pendulum.
- **Circle is one eased sweep, not laps.** Trajectory metrics use `metrics.sweep_window`;
  a whole-run radius ratio is meaningless.
- **A pre-takeoff disarm does not abort the fleet** (open problem 6).

---

## 11. How to run and verify

```bash
source ~/ros2_humble/install/setup.bash && source install/setup.bash   # or: mdc
colcon build --symlink-install                                          # or: cb

./tools/gate.sh --quick        # pytest, imports, geometry (seconds)
./tools/gate.sh                # + dissipative A–K + SIL smoke (~2.5 min)
python3 tools/prebuild_planner.py
python3 tools/sil_bench.py configs/sil/carry_hover_n3.yaml
python3 tools/run_experiment.py configs/experiments/attach_circle_n3.yaml --repeats 2
tools/plot_run.py R0234 ; tools/compare_runs.py R0234 R0243 --label timed settle
```

Verification ladder, cheapest first: pytest → gate quick → SIL bench → headless Gazebo
(~4 min a run) → rig. A claim is never promoted above the rung that produced it. Every
experiment starts from a card (`docs/experiments/<date>_<name>.md`), one variable, a
named baseline, a falsifier, two repeats, one registry row per run.

The demo by hand:

```bash
cd simulation_assets && gz sim three_attach_rod.sdf -v4 -r
ros2 launch controller_quad_load rviz_quad_load_launch.py num_drones:=3 attach:=true
ros2 launch controller_quad_load three_attach_launch.py load_traj:=circle traj_speed:=0.2
#   ARM -> TAKEOFF -> ATTACH
```

---

## 12. Where things live

| path | what |
|---|---|
| `src/controller_load_mpc` | OCP planner, geometry, trajectories, `params.py` |
| `src/controller_quad_load` | tracker (`controller_mpc.py`, `velocity_loop.py`, `thrust_model.py`), fleet manager, **all launch files** |
| `src/controller_dissipative` | network, node, offline harness |
| `src/drone_magnet`, `src/controller_mpc_payload` | attach chain and the approach MPC |
| `src/simulation_communication`, `src/drone_communication` | sim bridges; real mocap and ELRS |
| `simulation_assets/generate_rigid_world.py` | source of truth for every world; `tools/make_weld_variants.py` for the rod/seg2/rigid worlds |
| `configs/experiments/*.yaml`, `configs/sil/*.yaml` | every flown configuration |
| `tools/` | gate, runner, bench, metrics, plots, preflight, geometry and parameter checks |
| `results/registry.csv`, `docs/decisions.md` | what has been tried; what was decided |
| `docs/design/` | velocity loop, measured-force loop, SIL bench, fleet safety |
| `docs/experimentation/` | control-methods survey, hybrid dwell, robustness table, rig gap, agenda |
| `results_archive/` | tracked data behind thesis figures; `configs/thesis_figures.yaml` + `tools/thesis_figures.py` regenerate them |

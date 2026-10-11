# Reference: multi_drone_control

The parts of the old `CURRENT_STATE.md` that live nowhere else (moved 11 Oct 2026; Wesley). They were last reviewed on
7 Oct, so check anything here against the code before you rely on it. The section numbers are the old ones, so
existing pointers ("§9", "§10") still work.

Where the rest went:
- **What is running and what's next:** `docs/SESSION_STATE.md`.
- **Decisions:** `docs/decisions.md`. The locked ones are in `CLAUDE.md`.
- **Run results:** `results/registry.csv`.
- **Plans:** `docs/plan_m1_remaining_2026-10-11.md` and `docs/thesis_plan_2026-10-11.md`.
- **The old sections 3 (status), 4 (results to 7 Oct), 5 (locked decisions), 7 (open problems) and 8 (plan):** in git,
  `git show 0e91d86:CURRENT_STATE.md`.

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

Since 2026-09-25 his code is forked into this tree (base `a7524bae` of his `drone_cage_control`,
branch `SwungPayload2026_Collaborative`; `authority.md` §1): `src/tejen_mission` (mission planner
`online_join_planner`, magnet manager, supervisors, launches), `src/tejen_mpc` (his payload-pendulum
MPC), `src/tejen_dynamic_planner` (C++ transfer planner, needs GLPK), `src/tejen_utility_objects`,
his worlds under `simulation_assets/tejen/`. Two integrated demos, both claims SUPPORTED in sim
(§4.11): **M1** (four carriers start attached, orbit, drone 3 detaches, flies his pickup/drop
mission, is handed back and rejoins by weld + OCP resize, LAND) and **M2** (his four-drone ground
join, hand-over to our OCP, lift, hover, LAND). Git: branch `integration` (from
`week1-ready-for-testing` 35c2363, first commit d8aea49) holds `src/` and `simulation_assets/` only;
`tools/`, `docs/`, `configs/` stay untracked (backup `results/backups/pre_loop_2026-09-26.tgz`).

Payload: Te-Jen's M2A ring (`simulation_assets/m2a_ring_fixture.sdf`): a 500 mm ring of
24 segments carrying 12 magnet plates every 30 deg on its top face (plate 0 on +x), so
attach azimuths are clock positions on the plates. Official mass 0.86 kg (2026-09-23).
Every world carries this model since 2026-09-23 (`generate_rigid_world.py`, ring inertia
Ixx 0.0273 / Izz 0.0545 at 0.86 kg); the fixture's bench legs are left out because the
flying payload rests on its ring (mocap origin 5 cm off the floor). RViz draws the same
ring and plates (`fleet_viz`). `params.py`, every launch default, the Gazebo and SIL
configs and the SIL plant moved to 0.86 kg the same afternoon (Wesley's word); the
inertia now follows `load_mass` through `params.load_inertia` and
`tools/sil/plant.PayloadParams`, so a `load_mass` arg carries its ring inertia. Robustness
arms that deliberately mis-tell the mass carry `mis_seed: true`;
`tools/run_experiment.py` refuses any other config whose `load_mass` differs from its
world.

---

## 2. Architecture

Two reference generators feed one unchanged per-drone tracker over one wire format.

```
mocap (/drone_i, /payload motion_capture_state)
        |                                   |
  OCP PLANNER (10 Hz)              DISSIPATIVE NETWORK (10 Hz)
  mpc_planner                      dissipative_planner (subclasses the planner)
        +----> /drone_i/reference_trajectory <----+
               [n_nodes, dt, p(3), v(3), a_ff(3), a_cable(3)] x (N+1)
                          |
             PER-DRONE TRACKER (50 Hz) tracker (node tracker_<i>)
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
  the network engages. This is the default in the attach mode (`sim_control_launch.py mode:=attach`).
- **Common-mode load trim** (`diss_ki_load`): one z-only integrator on measured load
  height, distributed identically (or by tension share with
  `diss_trim_share_weighted`) to every attached node. Per-drone integrators tilt the load.
- **Measured-force throttle (INDI-lite) was removed on 2026-09-23** (Wesley): exact at the
  drone, never beat the classic loop on the moving load (§4.4 table 2, §6). History in
  `docs/design/measured_force_loop.md`; the runs stay in the registry.
- **Thrust map.** The rig and the rig-twin sim fly the identified affine map (30 Sep:
  `thrust_offset` 0.185, `thrust_ratio` 35.2 above it, `thrust_offset_v_slope` 0.022,
  `kt_trim` off). A wrong map lands in payload height, not throttle.
- **The rig twin is the simulation (5 Oct 2026).** `bringup/config/sim.yaml` holds the twin's
  values (the rig's geometry, masses and thrust map; `sim_thrust_map rig`, pack 23.8 V,
  `ff_cap_force 0.75`), so a twin config sets only its world, fleet and events. The legacy
  worlds (0.5 m rods, centre pivot, 0.64 kg drones, linear plant kT 83.1, `thrust_ratio:=auto`,
  `kt_trim` on, §4.10) are in `simulation_assets/old_worlds/` and fly on `sim_legacy.yaml`
  with `legacy:=true`, which `tools/run_experiment.py` infers from an `old_worlds/` or
  `tejen/` world path. The SIL bench and `mode:=attach` / `mode:=network` exist only on the
  legacy profile until ported; what has to be flown again on the twin is `docs/redo_on_twin.md`.
- **Detach** has two modes: `reconfig_mode:=network` (network redistributes, then hands
  back to a pre-built OCP for the new n) and `reconfig_mode:=ocp` (the OCP resizes
  directly; attach geometry is a runtime parameter so the survivors keep their true
  azimuths).
- **Attach chain** (`drone_magnet`): join planner, target publisher, magnet manager,
  `elrs_mux` (forwards the approach controller's commands until the weld, then ours;
  never hands authority to a stream that is not commanding thrust).
- **Partner paths (2026-09-26/27, §4.11).** M1: `sim_control_launch.py mode:=attach partner:=true
  partner_attached:=true`; drone 3 starts welded on plate 3, the detach is an OCP resize,
  our tracker steps it out, `/partner/release` starts his mission, `/join_planner/handoff_ready`
  hands it back at ATTACH_READY, our approach, weld, OCP resize n=3→4. M2: his join, then
  `sim_control_launch.py mode:=dissipative partner_m2:=true sim_interface:=false`; per-drone ELRS muxes switch each
  drone to ours on the first flying command after `/fleet/handover`; `airborne_start` creep holds
  live until TAKEOFF, then sweeps each rod from its measured elevation to 45° (T0008).

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
| Measured-force (INDI-lite) throttle, any gain | exact at the drone (kT told 19 % low: 0.21 m -> 4 mm) but a stiff position servo on rods of approximate length lets them go slack; code removed 2026-09-23 | R0238/R0239, R0244, R0247, R0257 |
| OCP hand-back with an even n=3 ring after 4 → 3 | 62° tilt; survivors keep their true azimuths instead | R0093 |
| SIL bench as an attach discriminator | 45° and 65° both diverge on the current plant; use Gazebo | R0034–R0043 |
| Offline harness (`verify_dissipative`, PD proxies) for tilt questions | holds level where Gazebo tilts 34°; smoke test only | — |
| Attitude-reference hemisphere alignment (rig yaw spins) | mechanism exercised in sim, no spin; the squared attitude cost is sign-invariant, so it cannot matter; code removed | R0276–R0279 |
| Sim rate-loop I-term ki 10 | limit cycle at 3.1 Hz on our X3, M2 ring rock 4.74°; ki 2 and 5 clean | R0647–R0650, T0016/T0017 |
| Steeper nominal rods for the M2 takeover (`cable_elev_deg` 70) | same lever flip (75.2°): a rigid rod turns any position mismatch into torque at the 0.04 m lever | T0010 |
| His `endsInStoppedHold` tolerance 1e-9 → 1e-6 (join stall) | same stall; reverted (cause was the simultaneous transits) | T0013 |
| Fixed or revolute tip joint on the partner rod | rod becomes a cantilever from the ring: 6.5° hang before the detach, drone 3 faults | R0682, R0683 |
| Stopping his launch at the handoff (M1 yank) | yank persists (15.4°): the cause was the state his mission leaves, not his nodes | R0679 |
| His thrust-ratio UKF for the 0.1 kg object | not flown: critic, recovers ~20–30 s after the drop, too slow for the rejoin; scheduled kT instead | card 2026-09-27_g1b_mass |

---

## 9. Hardware

Launches (package `bringup`): `real_io_launch.py` (mocap + ELRS + RViz, terminal 1, must be
up first), `real_control_launch.py` in a mode: `mpc` (carry, default), `dissipative` (detach),
`free_hover`; M1/M2 on the rig are `mode:=attach` and `mode:=m2`, one launch each that starts
the rig I/O itself. Defaults: `src/bringup/config/real.yaml` (§12); every rig mode flies the
rig carry's thrust map and geometry since 2026-10-03 (M1/M2 and free_hover unflown on them).
`tools/param_diff.py --sim-vs-real` shows the deltas. Desk test without the rig: `tools/fake_mocap.py`.
Pre-arm checklist: `tools/preflight.py --real`. Rig checklist for attach:
`docs/experimentation/real_attach_gap.md`.

Rig facts (updated 2026-09-29): our drones 0-3 = Tejen's quad1-4 = `/dev/QUAD1..4` (udev: four unique serials) =
Motive bodies 11-14 (`mocap_drone_body_ids`), ring body 8, Tejen's pickup object 6 (Wesley 2026-09-29; the code defaults
match); airframes 0.55 kg (`drone_mass` default, 30 Sep); the magnet tip
needs its own rigid body; the affine thrust map (`real_control_launch.py` defaults); floor start on the CREEP path,
`start_taut:=false handover_elev_deg:=45 handover_settle_s:=2.0` (the launch default
`start_taut:=true` runs the OCP from tick 1, whose references sit 18–22 cm inward of a
resting drone with flat rods: the 2026-09-23 13:13 non-lift; the arc creep anchors each
drone where it rests and sweeps the rod to 45° first; plan 2026-09-24);
`load_mass:=<weighed> cable_len:=<measured>` every session (the real launches default load_mass to 0.86, the ring).
ELRS adapters: `real_io_launch.py` defaults drone i to `/dev/QUAD<i+1>`.
`tools/preflight.py --real --planner mpc_planner` for the OCP launch (the default
planner name is the dissipative node's, `dissipative_planner`); `--max-ground-z 0.5` when the drones sit on taut
rods before takeoff. Since 2026-09-16 the rig detach launch takes `control_mode`,
`vel_*`, `auto_network_handover` and `diss_ki_load` like the sim launch (defaults unchanged:
MPC tracker, network only on a detach), so the Part A configuration can be flown on the rig. Tether magnets: each drone's
`elrs_interface` latches a magnet value into aux channel `magnet_channel` (6 = AUX4 on every launch; `/drone_<i>/magnet` String
ON|OFF; `rio magnet_initial:=ON` holds them from boot), and the RViz panel shows one
MAGNET toggle per drone on both launches (sim, since 2026-09-29: `sim_magnet` releases the tether's
DetachableJoint on OFF; ON cannot re-weld, restart Gazebo; the sim RViz config is byte-identical
to the rig's, pinned by `test_sim_rviz_config_equals_the_rig`). The tethered airframes' magnet mode is on `channel_6` (Betaflight AUX4), found with
`tools/aux_sweep.py` on 2026-09-16 (confirmed on drone 0 only: check the others at R0b), and is the launch default since
2026-09-16; one magnet path per drone through the radio latch since 6b8e01c.

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

Shell helpers (`shortcuts.md` at the repo root, gitignored, sourced from `~/.bashrc`): `mdc`
sources both setups; `cb [pkg]` builds; `rio`, `rctl`, `rdiss` are the real launches;
`sgz <world>`, `sio`, `smpc`, `sdo`, `sdet`, `satt` are the sim ones; `simcheck <world> <launch>`
runs the geometry check; `fleet arm|takeoff|land|disarm|estop`.

---

## 10. Traps and hard-won lessons

- **A LAND reference must keep descending.** Touchdown is detected as the drones no longer
  following a descending reference (`TouchdownDetector`). A LAND reference that pauses reads
  as touchdown: /fleet/landed disarmed the fleet 0.58 m up (R0852, an unwind held through the
  feedforward blend; reverted). The runner's criteria do not check LAND; read the drone heights
  at LANDED.
- **Read the parameters back.** Launch args have silently failed to plumb through three
  times (net_traj_lean, control_mode on the newcomer, a cut command line). Every run
  writes `params/` by read-back; check it before believing a comparison.
- **Same wall-clock duration or no comparison.** Trackers once ran on wall time while the
  planner ran on sim time; RTF then set the control rate.
- **`tools/prebuild_planner.py` after touching the planner model, masses or inertia.** The
  SIL bench starts its clock before the solver builds; an interrupted build deletes the
  old `.so` and never writes the signature, which presents as "the planner publishes no
  references".
- **The sim's CPU goes to the ROS side, not to physics.** Gazebo alone runs the ring
  world at RTF 1.0; with the controller stack up it sat at 0.37 idle / 0.29 in flight
  (12-thread laptop, 2026-09-23). Causes: Gazebo's /clock at 700–1000 Hz handled in a
  PYTHON callback by every node, the emulators republishing every Gazebo pose
  (300–500 Hz) into every tracker, and fleet_viz redrawing per message.
  `sim_io_launch.py` (then rviz_quad_load_launch.py) throttled the clock to 100 Hz from 2026-09-23 (clock via
  `clock_throttle`, publisher RELIABLE or the runner sees no sim time): 0.60 idle /
  0.45 in flight with the same hover to 1 mm (R0329 vs R0330). The default is clock_hz 500
  since 2026-09-25 (a free drone's rate estimate needs it, §4.11), mocap_hz 120, viz_hz 30. Still unexplained: each
  tracker burns ~0.9 core while DISARMED. Simplifying drone, rod or ring models does
  nothing (measured).
- **Payload contact geometry is one cylinder.** The ring's 24 box collisions, each
  touching the floor, took Gazebo's RTF from ~30 % to 7 % (2026-09-23); the ring and
  plates are visuals only.
- **acados refuses a cached solver whose `parameter_values` differ** ("OCP formulation
  has changed"), even when the signature matches; a run that hits this compiles mid-run
  and is void (R0297–R0301). `planner_ocp.py` now seeds canonical parameter values so the
  cache is geometry-independent.
- **`metrics.steady_window` ends at LAND.** Before 2026-09-23 it ran to the end of the
  record and averaged the descent into every "settled" height (0.598 reported for a
  0.634 hover). Recompute with `tools/plot_run.py --metrics` before comparing to old rows.
- **Start Gazebo from inside `simulation_assets/`** (relative model paths) and **RViz
  launch first** (it owns the pose bridges and mocap emulators).
- **`tools/clean_slate.sh` between runs.** Leaked nodes and Fast-DDS shared memory fake
  divergence.
- **Wall-clock watchdogs in a 0.3× sim** (pose timeout, reference staleness) fire on
  healthy fleets under CPU load. Sim launches carry longer budgets and headless Gazebo
  runs niced. Do not run analysis while a batch flies.
- **`mode:=dissipative` never engages the network** unless a detach or attach fires;
  `mode:=network` does.
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
- **A pre-takeoff disarm grounds the fleet and refuses TAKEOFF** since 189b6c6 (was open problem 6; R0749c, D0001, D0003).
- **The sim rate loop is PI by default since 2026-09-26** (`rate_ki` 10, clamp 200,
  integrates only armed and u > 0.09; all four sim bridges via `rate_pid.RatePid`).
  `SIM_RATE_KI=0` gives the old P-only loop (the m2d launch honours it; `M2_RATE_KI`
  overrides it there). Gazebo runs before this date were P-only: compare only against
  baselines flown with the same `rate_ki`. The SIL plant (`tools/sil/plant.py`) has no rate PID, so SIL and Gazebo now
  differ in the inner loop. [Superseded 2026-09-27: default `rate_ki` 5 on every bridge, Tejen's included, env
  fallback '5.0' (a4c748e); 10 limit-cycles (R0647, T0017). Runs R0647–R0650 and T0016/T0017 are the ki sweep.]
- **Never throttle /clock in Tejen's world** (breaks his velocity estimates, W-20:56); never pause the world
  mid-flight. His per-drone nodes dominate M2 CPU (planners 163 %, MPCs 128 %).
- **Difference poses over sim stamps, not the wall or node clock.** The magnet manager's weld speed gate read
  4× low at RTF 0.25 and welded mid-descent (R0668).
- **Never run M1 with /clock unthrottled** (clock_hz 0): ~180 carrier rate kicks and speed faults (R0702/R0703).
- **gz CLI one-shots drop** (the ball stayed welded after DROP, R0678): use ros_gz bridges, or send 3× (the
  runner's GZ_PUB does).
- **DART ignores `<axis><damping>` on ball joints** (S0005–S0008): every ball-joint damping in our models is a
  no-op; a universal joint takes it (S0009/S0010 settle a swing in 0.6 s).
- **Partner runs are void without the start weld**: check 'drone 3 welded at start: plate at 90'
  (`m1.start_weld_ok`); R0676 lost a start-up race and first read as a falsification.
- **The partner model's pickup DetachableJoint** holds the tip to the ball until his mission sends
  `/pickup/detach`; a bench without his mission uses a `_nopick` copy or GZ_PUB it (R0654–R0659 void).
- **Runner events are time-sorted**: a WAIT placed before the event it waits for blocks forever (R0671).

---

- **Lab: switching back to an older tree breaks the build** (11 Oct).
  - The lab builds with `--symlink-install` into one `~/mdc`. Files that exist only in the newer tree (for example
    tejen_mission's m1 launch and config files) leave dangling links in `build/` and `install/`, and colcon then
    fails with "can't copy ... doesn't exist".
  - Fix inside the container, where the links resolve:
    `tools/remote/lab_run.sh --tag clean -- bash -c 'find build install -xtype l -delete'`. Then rerun the batch.
  - From the host every link looks broken, so don't run the find there.

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

M1, M2 and the weld benches (§4.11):

```bash
python3 tools/run_experiment.py configs/experiments/partner_attached_orbit.yaml   # M1, 11-13 min wall
tools/clean_slate.sh; python3 tools/sim_test/drive_m2_handover.py results/<date>/T00NN false 20   # M2, ~21 min
python3 tools/run_experiment.py configs/experiments/m1_rejoin_orbit_fix_pick.yaml # rejoin bench, ~4-5 min
python3 tools/metrics.py run|m2 <run_dir>   # re-score: writes <run_dir>/metrics.json (M1 stages / summarise_m2)
```

Tools added 2026-09-26/27:
- `tools/metrics.py`: `event_window_metrics` (per event: peak ring and drone tilt, carriers' dip, time under 2°,
  ring z error; window to the next disturbing event or +15 s), `m1_stage_metrics` (every M1 stage, ball in the
  net in the ring frame, reattach retries, `start_weld_ok`), `summarise_m2` (join, fallbacks, faults, ring z and
  tilt per segment, drone tilt, landed; CLI writes `metrics.json`), `partner_clearance`; unit tests on R0646 and
  T0015 fixtures.
- Runner (`tools/experiment/`): logs PHASE/HANDOFF/PICKUP/DROP/FOLD_IN/PARTNER_* at sim time, the ball pose, a
  `wall` column, `wall_s` and `mean_rtf`; events GZ_PUB (gz topic, sent 3×), HANDOFF, WAIT_PARTNER_RELEASE,
  HANGER_RELEASE; `timing.stop_launch_on_handoff` (SIGINT his launch at the handoff).
- Weld benches (M1 without his mission): `m1_rejoin_hover*` (hover), `m1_rejoin_orbit_fix*` (orbit; `_pick` the
  full model with GZ_PUB `/pickup/detach`, `_late` detach 70 s later), `m1_rejoin_orbit_held` / `_moving`,
  `m1_zki_late_off` / `_on`.
- `tools/sim_test/stage1_torque_test.py` (one X3: body torque and tether swing, S-rows).

The demo by hand:

```bash
cd simulation_assets && gz sim three_attach_rod.sdf -v4 -r
ros2 launch bringup sim_io_launch.py num_drones:=3 attach:=true
ros2 launch bringup sim_control_launch.py mode:=attach load_traj:=circle traj_speed:=0.2
#   ARM -> TAKEOFF -> ATTACH
```

---

## 12. Where things live

| path | what |
|---|---|
| `src/mpc_planner` | the load planner: OCP, solver, cable dynamics, creep, geometry, trajectories, `params.py` |
| `src/tracker` | per-drone tracker (`controller_mpc.py`, `velocity_loop.py`, `thrust_model.py`, `kt_trim.py`), `free_hover` |
| `src/fleet_manager` | the fleet manager (`main.py`) |
| `src/bringup` | the five launch files, `config/` (the launch profiles), `profiles.py`, the mode graphs (`graph_carry.py`, `graph_detach.py`, `graph_attach.py`), `rviz_config.py`, `real_mode.py` |
| `src/dissipative_planner` | detach/attach planner (OCP resize, network), approach, offline harness |
| `src/drone_magnet`, `src/controller_mpc_payload` | attach chain and the approach MPC |
| `src/simulation_communication`, `src/drone_communication` | sim bridges; real mocap and ELRS |
| `simulation_assets/generate_rigid_world.py` | source of truth for every world; `tools/make_weld_variants.py` for the rod/seg2/rigid worlds |
| `configs/experiments/*.yaml`, `configs/sil/*.yaml` | every flown configuration |
| `tools/` | gate, runner, bench, metrics, plots, preflight, geometry and parameter checks |
| `results/registry.csv`, `docs/decisions.md` | what has been tried; what was decided |
| `docs/design/` | velocity loop, measured-force loop, SIL bench, fleet safety |
| `docs/experimentation/` | control-methods survey, hybrid dwell, robustness table, rig gap, agenda |
| `results_archive/` | tracked data behind thesis figures; `configs/thesis_figures.yaml` + `tools/thesis_figures.py` regenerate them |
| `src/tejen_mission`, `src/tejen_mpc`, `src/tejen_dynamic_planner`, `src/tejen_utility_objects` | Tejen's forked code (base a7524bae); our changes listed in `authority.md` §6 |
| `simulation_assets/tejen/`, `simulation_assets/three_attach_partner*.sdf`, `simulation_assets/models/x3_drone3_magnet_rod_partner*.sdf` | his worlds; M1 worlds (incl. `_m100` 0.1 kg object, `_nopick` bench copies); the partner rod models |
| `src/simulation_communication/.../rate_pid.py` | the sim rate PI shared by every sim bridge |
| `tools/sim_test/` | M2 driver `drive_m2_handover.py`, stage-1/2 single-drone benches, his runner scripts |
| `docs/GOALS.md`, `authority.md` | the integration loop's plan and status; Tejen integration notes |

### Rename map (3 Oct 2026)

History (registry rows, cards, plans, rig sheets) keeps the old names. Topics, parameters
and message types did not change. Old runs still read: every log tool goes through
`tools/run_logs.py`, which knows both folder layouts.

| before | after |
|---|---|
| package `controller_load_mpc`, executable `planner`, node `load_planner` | `mpc_planner` (package, executable, node) |
| package `controller_quad_load`: tracker modules, executable `controller`, node `controller_<i>` | package `tracker`, executable `tracker`, node `tracker_<i>` (`free_hover` unchanged) |
| package `controller_quad_load`: `main.py`, executable `main`, node `central_controller` | `fleet_manager` (package, executable, node) |
| package `controller_quad_load`: launch files, `rviz_config.py`, `real_mode.py` | package `bringup` |
| package `controller_dissipative`, executable `dissipative`, node `dissipative_controller` | `dissipative_planner` (package, executable, node) |
| `logs/controller_quad_load/planner_drone<i>_<stamp>` (tracker) | `logs/tracker/drone<i>_<stamp>` |
| `logs/controller_quad_load/load_planner_<stamp>` | `logs/mpc_planner/<stamp>` |
| `logs/controller_quad_load/dissipative_controller_<stamp>` | `logs/dissipative_planner/<stamp>` |
| `params/controller_<i>.yaml`, `central_controller.yaml`, `load_planner.yaml`, `dissipative_controller.yaml` (runner read-back) | `params/tracker_<i>.yaml`, `fleet_manager.yaml`, `mpc_planner.yaml`, `dissipative_planner.yaml` |
| `controller_ukf`, `real_attach_launch.py`, unused sim bridges and launches | deleted |
| shell helpers in `~/.bashrc` | `shortcuts.md` (repo root, gitignored), sourced from `~/.bashrc` |
| `tracker/controller_mpc.py`, `fleet_manager/main.py` | `tracker/tracker_node.py`, `fleet_manager/fleet_manager_node.py` |

### Launch files (consolidated 3 Oct 2026, plan §3b)

Five launch files in `bringup/launch`; the controllers take `mode:=`. Defaults live in
`src/bringup/config/common.yaml`, `sim.yaml`, `real.yaml` (each with a `modes:` entry per
mode); a knob that is not a launch argument goes in `params_file:=<yaml>` (the harnesses
write it into the run dir). `tools/param_diff.py sim/<mode> real/<mode>` shows what differs.
`sim.yaml` is the rig twin; `legacy:=true` reads `sim_legacy.yaml` (the `old_worlds/` plant;
`tools/param_diff.py sim_legacy/<mode> ...`).
`tools/launch_gate.py` replays the 360 frozen commands of the old launches.

| before | after |
|---|---|
| `rviz_quad_load_launch.py` | `sim_io_launch.py` |
| `real_io_launch.py` | `real_io_launch.py` |
| `mpc_quad_load_launch.py` (`reference:=free_hover`) | `sim_control_launch.py mode:=mpc` (`mode:=free_hover`) |
| `dissipative_launch.py` | `sim_control_launch.py mode:=dissipative` |
| `dissipative_only_launch.py` | `sim_control_launch.py mode:=network` |
| `three_attach_launch.py` (`sil:=true` for the bench) | `sim_control_launch.py mode:=attach` |
| `real_control_launch.py` | `real_control_launch.py` (`mode:=mpc`, default) |
| `real_dissipative_launch.py` | `real_control_launch.py mode:=dissipative` |
| `real_hover_launch.py` | `real_control_launch.py mode:=free_hover` |
| `three_attach_launch.py real:=true` | `real_control_launch.py mode:=attach` |
| `dissipative_launch.py real:=true` | `real_control_launch.py mode:=m2` |
| `m2_bench_io_launch.py` | `sim_m2_bench_launch.py` |

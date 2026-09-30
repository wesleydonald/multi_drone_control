# Plan: make the sim match the rig, then fix the tethered model

**Status:** approved (30 Sep 2026, evening; his answers are the 30 Sep lines in `docs/decisions.md`). Phase 1 (W1, W2, W3, W8, W9, W12, W13) and phase 2 (A4, W0, W4, W5, W6, W7 reproduction) are done, nothing committed. Late 30 Sep (R0787-R0800): SIL deterministic again (R0796 = R0797 bit for bit); H2 and C1 twins flown and pass (R0788, R0789); rig-plant free-drone check passes (R0792, 0.0004 off); ring-mass neighbours move height only (R0791, R0793), R0793 fleet-disarmed on LAND (rigid rod tips a wide drone). Open: creep hand-over 55° (fix proposed, not built), freeze-off arg (W11), LAND with rigid rods, sil_smoke re-baseline. Next: the rig visit, `docs/rig_2026-10-01_commands.md`.

**Phase 2 done (30 Sep, night; results in card A "## Results", registry R0772-R0786):**
- **W0 lab parity.** Gazebo holds (R0773 against R0767: dz 0.0 cm, tilt 0.12°). SIL is no longer deterministic on one machine (R0775 against R0777: tilt 1.47° apart), which breaks the "one SIL run per arm" rule until it is traced.
- **W7 reproduction: not reproduced.** R0781 (pre-fix planner and tracker) and R0782 (current) are the same: 0 failures, rods 0.583, carried 0.996. The twin has neither cause: the failure trigger needs the ring near yaw 180° with the 30° slot offset, and the fitted-law plant carries the full ring by construction. Line stopped after two runs.
- **Fixed twin passes.** Pivot on (R0783, R0784 identical; R0786 on 3915): rods 0.550, hold within 1 cm of target on z_ki 0, 0 failures, reference age 0.
- **Not closed in sim:** hand-over at about 55° in every twin run (outside the rig card's 45 ± 5); standing tilt 1.2-1.4° from the per-drone offsets against one tracker offset; H2 twin (`rig_twin_hover_fixed_zki`, R0785 void: solver compiled in the run) and C1 twin (`rig_twin_orbit_slow`, session budget) not flown; the critic's free-drone rig-plant check not run.
- **Needs Wesley's word:** the `four_rigid_ground_rig_m070.sdf` world for the mass-mismatch neighbour, model_f1 `attach_z`, the ring inertia in `params.py`, and the sil_smoke re-baseline (peak_cable_accel 11.3 against 8.0).

## Schedule

### Before tomorrow's rig visit

This work does not depend on the full sim twin being finished.

1. **W1 logging.**
   - **Planner columns:** solve status and time, node-0 tension per rod, t·s_z, ff, gate, rod length and elevation, ring quaternion and ω, reference age.
   - **Failure dumps:** a dump on every solve failure.
   - **Tracker columns:** thr_out, thr_off, a_cable, ff_active, grounded.
   - Without these, tomorrow's data cannot answer the open questions.
2. **W2 replay.** `tools/planner_replay.py` on model-f1 (the status-4 hunt, toggles a-f), a static-hold pytest at 53.6 and 65 deg, and an f7 heave replay.
3. **W3.** `tools/thrust_fit.py`: reproduces ladder4 exactly, and has a `--tethered` rod-force-over-weight mode. Recompute f6, f7 and model-f1 under it offline.
4. **W8 solve-failure fix.**
   - A full solver reset on reseed and a cold-iteration time budget.
   - Shifted-horizon publishing capped at N and keeping the original stamp, so the watchdog still works.
   - Critic (card A), replay, SIL ×1.
5. **W9 planner pivot model** (rod starts 4 cm below the drone centre).
   - Covers the measured rod lengths, the taut gate, x_init and the creep.
   - Critic (card A), replay toggle (c), SIL ×1, Gazebo floor start ×1.
6. **W12 tracker consistency.** `_applied_u` and the cap in model units after the offset, under the card A critic. W13 safety-net defaults: z_ki 0.4, bound 0.15, gate 0.25. `z_taut_gate` 0.6 is the rig fallback if W9 is not in.
7. **W0 lab PC**, in parallel.
   - A Docker parity image, the parity run against R0767, and the parallel runner (at most 4 slots).
   - It then carries the Gazebo work: the W4/W5 sim twin (affine plant with pack sag, rig geometry on the carry worlds with their own drone models, legacy worlds pinned), the W7 reproduction of model-f1, and the W11 freeze-removal arms with the mass-mismatch neighbour.
   - The sim twin is started tonight and continues after the visit if it is not finished. The visit does not wait for it.
8. **Freeze.** It stays the rig default (Wesley). Freeze removal exists only behind a launch arg until the deficit is explained.

### Tomorrow evening: 90 minutes on the rig, all for data plus the fixes that are ready

The card is written with /experiment before the visit. Every lift runs with the W1 logging on.

| # | test | why | pass / what we learn |
|---|---|---|---|
| D1 | One drone, free hover, rod and magnet removed (0.475 kg), 20 s; `drone_mass:=0.475 thrust_ratio:=40.7` | Did the rod hanging vertically in the downwash bias the thrust fit? | Model throttle against the law at 0.475 kg; tells us whether tethered thrust differs |
| D2 | Tethered hold at 0.3 m, then 0.7 m, freeze on, `z_ki:=0.0`, 15 s steady each | Does the ~80 % airborne force deficit depend on height (downwash) or not? | Rod-force sum over weight at each height (thrust_fit --tethered) |
| H1 | Tethered hover at 0.5 m with W8 + W9 + W12, freeze on, `z_ki:=0.0`, 25 s | Are the solve failures and the heave fixed? | 0-1 solve failures; reference age > 0.3 s under 5 %; heave p-p <= 6 cm; tilt mean <= 3, max <= 8; hand-over 45 +- 5 deg |
| H2 | As H1 with the safety net (z_ki 0.4, bound 0.15, gate 0.25) | Final hover quality | Mean z within 2 cm of target, z_bias within +-0.10 |
| C1 | Slow circle, r 0.5 m at 0.125 m/s, net on; only if H1 and H2 pass | Next goal | Ring tilt mean <= 3 over the lap, z error <= 5 cm |

**Stop rules:**
- **LAND** on a magnet release, tilt > 5 deg in the pretension or > 12 deg in flight, ring vz > 0.25 m/s, 3 or more solve failures, or a creep timeout.
- **Sequence:** after one release, read the log before the next lift. After two releases, stop lifting for the session.

The draft (section A) was written from six investigations of the code and the rig logs, then red-teamed (section B). **The critic's must-fixes override the draft wherever they conflict.** They are summarised here:

1. **Rate I-term.** Keep the sim rate-loop I-term at ki 5 (DECIDED 2026-09-27). The draft's C4 and Q14 are withdrawn.
2. **Freeze.** The rig keeps the breakaway freeze as its default until the force deficit (H3) is closed by weighing and by P2 (tethered holds at 0.3 m and 0.7 m). P2 is flown with the freeze on, before H1. The stop rules become soft (ramp the pull down, hold) rather than an abrupt LAND.
3. **Mismatch neighbour.** The sim claim for freeze removal adds a deliberate mismatch neighbour: world ring 0.70, planner 0.86, plus the other sign.
4. **Pass metric.** The breakaway % is replaced by the measured rod-force sum over the ring's weight (thrust_fit --tethered). The baselines f6, f7 and model-f1 are recomputed offline under the same metric.
5. **Solve-failure fallback.** W8's shifted-horizon fallback is capped at N consecutive publishes and keeps the original stamp, so the reference-age watchdog still trips. It is covered explicitly by the critic on card A.
6. **Rig twin models.** The rig twin gets its own drone model files. The legacy attach, M1, M2 and weld worlds, and their configs, are pinned to the legacy geometry explicitly.
7. **Rod check.** The rig pre-flight rod check uses per-drone bands of about 0.53-0.58, not 0.54-0.57.
8. **kT shadow.** It is not independent evidence of the deficit (it is fed the planner's static share).

**Should-fixes adopted:**
- Rank H2's trigger by lift-off dynamics, stale pose or ω noise ahead of the 45 deg reference.
- W8 is not gated on the W7 reproduction (SIL ×1 is enough; no floor run).
- Add an f7 replay item for the heave, which also happens with zero solve failures.
- W1-W3 (logging, replay, thrust fit) come ahead of W0 (lab Docker).
- Re-baseline `gate_thresholds.yaml` sil_smoke right after W4, before any Gazebo run.
- Budget arithmetic: W11 is 8 runs; B0 merges with the W11 even-ring claim.
- The twin gets `z_taut_gate` 0.9 and `attach_z`, and the reference age is measured in sim time.
- The rig safety net (H2) needs W9 (pivot), or a lowered `z_taut_gate` with Wesley's word.
- A hand-over elevation bar of 45 ± 5 deg on the rig card.
- W12 (the tracker's pinned throttle) goes under card A's critic.
- Before the circle: the planner `thrust_max` is about 11.9 N at the 0.8 cap.
- Candidate for the deficit: downwash drag on the rod when it hangs vertically (the ladder) against running out at 55 deg (tethered). Rig check: free hover with the rod removed.

---

## A. Draft plan

# Plan: make the sim match the rig, then fix the tethered model (30 Sep 2026 → next rig visit)

Branch `real-world-testing`, base commit 155ee56. I read the inputs below myself and quote numbers only from them: the review, the pretension card, registry rows 578-605, `tools/remote/README_lab_access.md`, `planner_node.py`, `planner_solver.py`, `real_control_launch.py`, `payload_betaflight_comm.py`, `run_context.py`, `run_dir.py`, `decisions.md` and the model-f1 planner CSV. Figures marked (sub) come from the six investigation reports and I did not re-derive them.

## 0. Corrections to the review, checked against the code and logs

| # | Review says | What the code and logs say |
|---|---|---|
| C1 | Breakaway at 77 % means the planned pull is 23 % too high | 77 % is a clock reading: `_ff_cap = _ff_t / pretension_s` (`planner_node.py:771`). It fires on vz > 0.03 or a 1 cm rise (`:97-99, 767-768`), which can be one edge lifting (sub: tilt 2.2-3.2° at the trigger). It is not a measured force. |
| C2 | model-f1 heaved at 0.17 Hz with 14 cm p-p | In the planner CSV, z_tgt ≥ 0.45 lasted only 3.8 s before LAND, and z_tgt ≥ 0.49 only 1.1 s. Ring z over that window was 0.300-0.398, mean 0.339 (22 rows). That is too short to measure a heave. The spectrum is broadband 0.1-0.4 Hz (sub). |
| C3 | The planner over-plans | Using the affine law with tilt and voltage, the drones delivered about 6.8-6.9 N of vertical pull while the ring was airborne. That is 80-82 % of 0.86 kg·g (sub, tracker and rig-data agree). The kT shadow estimate reads +5.8 % (`model_f1.log:106-107`, sub). The deficit shrinks with ring height, and f11 at 0.63 m reached 104 %. So part or all of the "over-plan" is on the plant side: either the ring is lighter than 0.86, or something holds it up near the floor. The planner side (tension reference at 45° against rods at 54-66°) is suspected but not logged. |
| C4 | Sim rate I-term default | `decisions.md:41` says rate_ki 10. The code says `SIM_RATE_KI` default 5.0 (`payload_betaflight_comm.py:96`). This has to be reconciled before any baseline is re-flown. |
| C5 | Four-drone rig layout | `decisions.md:54` decided plates 1/3/5/9 for four drones. The rig flew the even ring, and this plan assumes the even ring for the next visit (Q2). |
| C6 | Planner logs cannot replay the failures | Confirmed. The planner CSV header has no status, tension, ff, ring quaternion/ω or x_init. Only an approximate replay is possible from today's data. |
| C7 | Sim kT "88.6" | The current sim value is 83.1 (0.64 kg X3). 88.6 is the old 0.6 kg number. |

## 1. Goal and pass criteria

**Goal.** A 4-drone tethered hover on the rig that holds its height on the model alone, then a slow circle. First, a sim that reproduces model-f1 on the pre-fix code.

**Sim claim** (Gazebo ×2, rig-twin world and plant, fix in, no freeze, z_ki 0 unless noted):

| metric | bar | model-f1 / old sim |
|---|---|---|
| breakaway (ff at lift-off, now a log-only metric) | ≥ 90 % | 77 % |
| planner status ≠ 0, hand-over → LAND | 0 | 25 |
| airborne time with tracker reference age > 0.3 s | 0 % | 75 % (sub) |
| ring tilt, pretension and breakaway | ≤ 5° | ≤ 4.2 (f6) |
| ring tilt, hold | mean ≤ 2°, max ≤ 5° | 6.5 / 16 |
| ring vz, pretension → end of hold | ≤ 0.15 m/s | f6 0.38 |
| hold, last 20 s, \|mean z − target\| | ≤ 5 cm on the model alone (Q6); ≤ 2 cm with z_ki 0.4, \|z_bias\| ≤ 0.10 | 16 cm |
| heave over ≥ 20 s of hold | p-p ≤ 3 cm | — |
| drones | median reference error ≤ 3 cm, throttle < 0.75 (cap 0.8) | 0.5-2.5 cm |
| run | no abort, LANDED | |
| orbit neighbour | ring tilt mean ≤ 3° over the lap, \|z err\| ≤ 5 cm | R0728 tilt peak 1.89 |

**Rig pass bars** (next visit): see section 6.

## 2. Root-cause hypotheses, ranked

| rank | hypothesis | evidence | cheapest test that confirms or kills it |
|---|---|---|---|
| H1 | **Heave and tilt spikes come from stale, stepped references.** A failed or cold solve blocks the single-threaded planner, and the last references are held. | Ring, drones and references move as one body (xcorr 0.91-1.00). References were > 0.3 s old for 8.8 of 11.7 s airborne. Reference steps after a gap: median 1.6 cm, max 10.8 cm in z. f7/f10/f11 had 0 failures and still bounced (sd 4.5 cm in f7) (all sub). | Offline replay: count ticks over 100 ms. Then the sim twin (W7): reproduced if status ≠ 0 appears and reference age > 0.3 s. Killed if the twin has 0 failures and still heaves ≥ 5 cm p-p; the cause is then the open loop in load height or the rod model. |
| H2 | **Solve failures: the reseed is incomplete.** `reseed` sets only x (`planner_solver.py:203-205`). u, slacks and multipliers from a NaN or MINSTEP iterate survive. COLD_ITERS 15 (`:33`) blocks the loop. A contributing trigger is the s reference at 45° (`planner_node.py:301`) pulling against pinned rods at 54-74°, and rods measured 3-4 cm long. | 60 of 68 QP errors at QP iteration 1. Failures about every 0.41 s. Only the first flight on the new .so and geometry (rods 0.587-0.613) (sub). | `tools/planner_replay.py` 150.0-166.4 s. Reproduced if status ≠ 0 appears at 154.6-155.6 s. Then one toggle per run: (a) `solver.reset()` and u = 0, (b) references at the hand-over elevation, (c) pivot-corrected s and l, (d) 0.5 s stale pose, (e) ω noise, (f) old geometry. Killed if (a) alone does not remove it; try (b) and (c) next. |
| H3 | **Plant-side force deficit.** The ring is effectively lighter (a) or held up near the floor (b), or the thrust law under-reads when tethered (c). | 80-82 % of weight carried while airborne. Height-dependent (f11 fit R² 0.84, 1.03-1.20 at 0.6-0.9 m). kT shadow +5.8 % (sub). | **Weigh the ring (free).** 0.70-0.75 kg → (a): set `load_mass` and H3 is closed. 0.86 kg → (b)/(c): rig checks at the next visit (tethered holds at 0.3 m and 0.7 m, free hover at 1.3 m). Gazebo has no aerodynamics, so (b) cannot be reproduced in sim (see W7 arm A2). |
| H4 | **Planner over-plans.** t_nom is built for 45°. Node-0 t is free (w_t 5), so the published t·s/m can sit at the reference instead of at statics. At 53.6-62° that is 1.14-1.25× the ring's weight vertically (sub). | Code path `planner_node.py:301` → reference builder. The tension is not logged. The tracker equilibrium argument says only about 3 % high (sub), so this contradicts H3's size. | Replay: read node-0 Σt·s_z against 8.44 N at the logged geometry, plus a static-hold pytest at 53.6° and 65°. **Discriminator in sim:** the twin carries the true mass, so H4 predicts sim breakaway 80-88 %, while H3 alone predicts ≥ 90 % (R0766 old sim floated at the end of the pretension). |
| H5 | **The pivot is missing from the planner geometry.** Rods read 3-4 cm long. The node-0 reference sits 1.7-2.6 cm beyond the drone along the rod, so feedback adds pull. The length gates read 0.75-0.82, which keeps the safety net shut. | Pivot fit 0.036 ± 0.006 m. Pivot-corrected rods 0.537-0.573 (sub). | Replay toggle (c). The twin reproduces the long rods. A/B in Gazebo: pivot on the planner vs off. |
| H6 | **FF inconsistency.** `acc` carries the full pull while `cable` is scaled by gate·ff (`planner_node.py:1245` onwards, sub `:1270-1273`). The attitude reference is tilted for 100 %. | Code. Matters only while ff < 1 (pretension, and today's freeze). | pytest on `_publish_refs`. It disappears once ff reaches 1, so it ranks low. |
| H7 | **Rod or pendulum mode.** | Killed for z: ring and drones move in phase. A 1.15 Hz tilt line in f10/f11 suggests rocking (sub). | Report only. The new 75 g, undamped rods in Gazebo can bring in a new mode (risk R4). |

## 3. Work items, in dependency order

Headless Gazebo budget: at most 10 per session on the laptop (the lab budget is Q1). SIL is one run per arm.

**W0: Lab compute.** This is the first item. No repo behaviour changes.
- Image `mdc-humble:db1c3ca`:
  - Ubuntu 22.04 base (`osrf/ros:humble-desktop`), Gazebo Harmonic pinned to 8.10, ros_gz for Harmonic at the laptop's commit (record it), acados db1c3ca built into `/opt/acados`, Python 3.10.
  - User `wesley` with uid 1000, HOME `/home/wesley`.
- Lab checkout at `~/mdc` on drones, kept up to date by rsync from the laptop working tree, excluding `results/`, `build/`, `install/` and `c_generated_code*`. It is bind-mounted at `/home/wesley/multi_drone_control`. colcon builds inside the container into a lab-only `install/`.
- The lab PC today has 24 cores, 57 GB available, 314 GB free and no Docker images.
- New `tools/remote/lab_batch.py`, running on the laptop:
  - Allocates the run ids on the laptop before dispatch. `next_run_id` scans the directory tree (`tools/run_dir.py:65-80`), so parallel allocation would race. `run_experiment.py` gets a `--run-dir` option.
  - Starts one container per slot, `docker run --rm --name wesley_slot<n> --cpus 3`, on its own bridge network, with:
    - `ROS_DOMAIN_ID` 40+n and `GZ_PARTITION wesley_<n>`;
    - `MDC_ACADOS_ROOT=/scratch/slot<n>` (`run_context.py:114`) and cwd `/scratch/slot<n>`, because the planner `CODE_DIR` is relative (`planner_solver.py:38`);
    - `MDC_RESULTS_ROOT` set to a slot staging directory.
  - After the run, rsyncs `R####_*` back to the laptop's `results/<date>/`. Registry rows are written on the laptop only.
  - Total load at most 16 of 24 cores (README). The runner already kills only its own process groups.
- **Parity:**
  - SIL: `carry_hover_n3` on both machines. Payload-z trace max \|Δ\| ≤ 1 mm (lockstep).
  - Gazebo: `rig_lift_n4_even_floor` (R0767: tilt peak 0.37, hold 0.600) once on each machine at 155ee56. Same verdict, hold \|Δz\| ≤ 1 cm, tilt peak \|Δ\| ≤ 0.5°.
- **Parallel pilot:** 4 identical copies, then 6. Each must pass the parity bars and log no reference-watchdog trips.
- Budget: 1 laptop run and 5 then 6 lab runs. They are infrastructure runs, not claims.

**W1: Logging** (cheap; needed for every replay and for the next rig visit)
- `planner_node._log_tick` (`:457`) gets new columns:
  - solve status, solve time, QP iterations;
  - per rod: node-0 t_i, t_i·s_z, ff, gate, measured length and elevation;
  - ring quaternion and ω, drone xyz;
  - reference age.
- `planner_solver.solve_horizon`: on status ≠ 0, write an `np.savez` of x_init, last_X, yref/q_ref, `_geom` and poses, plus `solver.dump_last_qp_to_json()`.
- `controller_mpc` log (sub `:539-568`): `thr_out`, `thr_off`, `acable_xyz`, `ff_active`, `grounded`.
- Test: header pytest. No runs.

**W2: Offline replay and a static-hold pytest** (answers H2, H4, H5)
- `tools/planner_replay.py`:
  - builds in a scratch code directory;
  - n 4, load 0.86, drone 0.55, rho 0.225, attach_z 0, l = [0.613, 0.605, 0.587, 0.597], yaw datum 158.9°, slot map [2, 0, 3, 1];
  - runs the toggles a-f listed under H2.
- `test_planner_static_hold.py`: at 53.6° and 65°, 50 ticks with status 0. Print node-0 Σt·s_z against 8.44 N.
- Output: a table per toggle giving the status count, the maximum solve time and Σt·s_z/weight. No runs.

**W3: `tools/thrust_fit.py` and `tools/test/test_rig_replay.py`** (fixtures in `tools/test/fixtures/rig_0930/`)
- The fit reproduces ladder4 exactly: a_i 0.181 / 0.185 / 0.187 / 0.187 ± 0.002, b 0.507 ± 0.005, c −0.0219 ± 0.002, rms 0.0032, n 13.
- A `--tethered` mode gives the rod-force sum against weight.
- Tests:
  - model-hover throttle within 0.01;
  - model-f1 hold Σ within ±10 % of 0.86 g, marked xfail until H3 is resolved;
  - one law in three places, equal to 1e-6.
- No runs.

**W4: Sim thrust plant (affine thrust + pack sag)**
- New module `simulation_communication/rig_thrust.py`:
  - K = 9.81/0.507, A_I, slope 0.0219, V_REF 23.5;
  - `PackModel` with V0 24.4, R 1.0, τ 5 s, drain 22 mV per throttle-second, telemetry quantised to 0.1 V (Q4).
- Files:
  - `payload_betaflight_comm._cmd_cb` (`:239-243`): `thrust_map` rig/linear parameter, per-drone a_i, `rate_i_min_u` 0.35 under the rig map;
  - `sim_telemetry.py` owns the pack: `/drone_i/sim/pack_v` plus `/drone_i/telemetry` at 10 Hz;
  - `tools/sil/plant.py`, `bench_node.py` (publishes telemetry), `scenario.py`, `standin.py`;
  - `thrust_model.py` gains a rig branch: 'auto' = K/m;
  - four sim launches: `sim_thrust_map`, the offset, slope and v_ref, `throttle_max` 0.8. The carry throttle 0.588-0.608 is over the 0.6 cap otherwise.
  - `tools/metrics.py` `THROTTLE_SAT` taken per run;
  - `tools/headroom.py`.
- Out of scope: Tejen's bridges and `m2_bench*` (linear map, Q5).
- Tests: new `test_rig_thrust.py`; rewrite `test_thrust_model.py`, `test_sil_plant.py`, `test_real_mode_launches.py` pins.
- Verification: pytest, then `gate.sh --quick`.

**W5: Sim geometry twin** (lands together with W4; `test_config_tools.py:26-43` forces that)
- `generate_rigid_world.py`:
  - `PAYLOAD_RADIUS` 0.225 (ring outer 0.255), ring thickness to 40 mm, `ATTACH_PLANE_Z` (Q3);
  - `--pivot-dz 0.04` with the drone-end ball posed in base_link;
  - rod + magnet 0.075 kg on the rod, magnet at the payload end;
  - `--cable-len` defaults to 0.55;
  - ground start subtracts `pivot_dz`.
- Drone models: x3 base_link mass 0.435 (0.475 bare), inertia scaled.
- `params.py`: CABLE_LEN 0.55, ATTACH_RADIUS 0.225, DRONE_MASS 0.55, LOAD_IXX/IZZ 2.227e-2 / 4.431e-2. Then run `prebuild_planner.py`.
- Sim launch defaults; `check_geometry.py` gets pivot_dz, rod mass and drone mass extractors; new `tools/make_carry_worlds.py --check`; `run_experiment.py:688-697` also checks cable_len, attach_radius and drone_mass.
- Regenerate `four_rigid_ground{,_3915,_3969}`, `three_rigid_ground`, `two_rigid_ground`, `three_rigid_ground_m10`, `four_rigid`.
- Attach, M1 and M2 worlds: see Q7.
- Also:
  - reconcile rate_ki 5 vs 10 (C4) before any run;
  - the SIL gets the pivot as an end-point offset only. Full rigid-body torque in SIL is Q8. The pivot torque is judged in Gazebo.

**W6: Sim launch parity with the rig launch, and the harness**
- Sim launches gain `attach_radius`, `z_i_gate`, `rod_tol_frac`, `rod_spread_m` (sub F1: none exist today).
- `runner_node.py:362` gets a `WAIT_LIFT` timeout.
- `metrics.py`:
  - `heave_metrics` (p-p, sd, period; flags windows under 3 periods);
  - `breakaway_frac`;
  - `planner_solve_failures`;
  - `ref_age_frac`.
- `config.py` Criteria: `max_planner_solve_fail`, `max_hold_z_err_m`, `hold_window_s`, `max_heave_pp_m`, `max_payload_vz_mps`, `min_breakaway_frac`.
- Tests: metric unit tests on the model-f1 fixtures.

**W7: Sim reproduction of model-f1** (pre-fix code with the freeze; card; see section 4)
- Budget: SIL ×1, Gazebo 2-3 floor starts.
- **This gates W8-W11.**

**W8: Solve-failure fix**
- `planner_solver.solve_horizon`: on reseed, `solver.reset()`, x on every node, u = 0.
- Cold-iteration time budget of about 60 ms.
- On failure, `_plan` publishes the previous horizon shifted by one node instead of holding the old one.
- Order: replay toggle (a) passes → SIL ×1 on the twin → Gazebo floor-start ×1. Two-run time box.

**W9: Planner pivot model**
- `pivot_offset` in params.py (rig [0, 0, −0.04]); `_drone_cb` stores orientation; `_pivot_at`.
- Route these call sites through it: `build_x_init` callers (`:886, :912` sub), `_apply_measured_rod_lengths`, `_cable_taut_gate`, the diagnostic elevation, the creep inputs.
- `drone_kinematics` output shifted by −R·b. The same shift goes in `creep_controller.py`.
- No OCP rebuild.
- Test: `test_pivot_offset.py` (measured rod = L, p_ref = centre).
- Critic (card A). Order: replay (c) → SIL ×1 → Gazebo floor ×1.

**W10: Pull model**
- s and t references from the hand-over elevation.
- Pretension cable FF = ff × statics at the measured pivot directions.
- `acc + (1 − gate·ff)·cable`, so the planner and the tracker's resting gate agree (T6).
- Only if the W2 replay shows Σt·s_z/weight ≥ 1.05 at the logged geometry. Otherwise W10 shrinks to the acc consistency fix.
- Critic (card A, a separate arm). SIL ×1, Gazebo floor ×1.

**W11: Remove the breakaway freeze**
- Delete `_ff_cap` (`planner_node.py:388, 767-775, 1105`, the min at `:1245`).
- Keep a log-only breakaway line with a stricter trigger: all plates +5 mm, or ring +1 cm with tilt < 3°.
- Keep the `lift_z0` re-latch.
- Rig fallback: Q9.
- Precondition: the W7/W8-W10 twin reaches breakaway ≥ 90 %. Otherwise the f6 run-up comes back.
- Card A claim: Gazebo ×2 plus neighbours ×1 each: n3 floor, 1/3/5/9, pack starting at 22.8 V and at 24.6 V, target 0.6, slow orbit on the even ring. That is 7 runs.

**W12: Tracker consistency**
- T1: `_applied_u` and `last_cmd_throttle` = max(0, out − off(V)) after the cap (sub, `controller_mpc.py:813-855`).
- T3: model cap = `throttle_max` − off(V).
- T5: guards on `kt_batt_sag_frac` against `v_slope`; the kt_trim gate uses `_model_thr_max`; `velocity_loop` cap.
- Not in this plan without Wesley: T2 (spool only the part above the offset) and T4 (per-drone offsets as launch defaults) (Q10).
- Touches takeoff: pytest, then one Gazebo floor start, which can share W8's run if flown as its own variable (it cannot). So one extra run.

**W13: Safety-net defaults**
- `real_control_launch.py:96-97`: z_i_max 0.15, z_i_gate 0.25; z_ki stays 0.4 (Wesley's word given).
- The net opens only after W9, because the length gates read 0.75-0.82, below z_taut_gate 0.9.
- One SIL run with z_ki 0.4 on the twin.

**W14: Re-baseline.** Section 4. Also `gate_thresholds.yaml` sil_smoke (Q11).

**W15: Rig card.** Section 6; written with `/experiment`.

**Gazebo totals:** W0 1 laptop + 11 lab; W7 3; W8-W10 3-6; W11 9; W12 1; W14 6-7. About 34 runs, or about 4 laptop sessions at 10 per session. At the proposed lab budget (Q1) the lab takes it in about 2 sessions.

## 4. Re-baselines and the sim reproduction of the rig

**Reproduction, card R** (after W4-W6, pre-fix planner, freeze on, z_ki 0):
- Config `rig_twin_model_f1`: n 4, even ring, rig world, affine plant with sag from 23.8 → 22.9 V.
- Launch args: `cable_len 0.55`, `attach_radius 0.225`, `drone_mass 0.55`, `thrust_ratio 35.2`, offset 0.185, slope 0.022, `kt_trim false`, target 0.5, `lift_ramp_vel 0.1`, `pretension_s 3`, `throttle_max 0.8`, `measure_rod_len true`, `rod_tol_frac 0.25`, `rod_spread_m 0.08`, hold ≥ 20 s.

| arm | variable | runs |
|---|---|---|
| R0 | SIL air-start twin: solve status and ref age, no floor | SIL 1 |
| R1 | Gazebo floor start, hand-over 45° | 1 |
| R2 | as R1, hand-over elevation 54° (the rig creep overshot to 53.6°) | 1 |
| A2 (optional, after the ring is weighed) | world ring mass 0.70, planner 0.86: plant deficit injected | 1 |

**Verdict bars:**

| metric | REPRODUCED | NOT REPRODUCED |
|---|---|---|
| planner status ≠ 0 | ≥ 5 in the 10 s after breakaway, first within 2 s | 0 |
| reference age > 0.3 s | ≥ 25 % of airborne time | < 5 % |
| measured rods at hand-over | 0.57-0.62 (pivot +3-4 cm) | 0.55 ± 0.01 (pivot not in the world) → VOID, fix W5 |
| breakaway | 55-85 % means H4 is in the planner | ≥ 90 % means the deficit is plant-side (H3); expected if the ring is light |
| ring z in the hold | mean 0.28-0.42 | within 5 cm of 0.5 |
| drone reference error | median ≤ 3 cm | > 5 cm → VOID (plant port wrong) |
| heave | reported only (C2) | — |

**Rules:**
- If solve failures and long rods are not reproduced, stop and report. Do not tune.
- If breakaway is not reproduced but solve failures are, that is a valid split: H3 is outside the sim and the rig checks carry it.

**Re-baseline once, after the fix is final** (CLAUDE.md "re-fly a matrix once"):

| # | config | baseline |
|---|---|---|
| B0 | `rig_lift_n4_even_floor` (first; floor rule) | R0767 |
| B1 | `ocp_hover_ground_creep_defaults` | R0721, R0587 |
| B2 | `ocp_hover_ground_creep_n4_3915` | R0729 |
| B3 | `orbit_ground_creep_n4_3915` | R0728 |
| B4 | `detach_hover_n4_3915` | R0722 |
| B5 | `attach_circle_n3_ocp` (only if Q7 = regenerate) | R0723 |
| B6 | `partner_attached_orbit` | R0724 |
| SIL | `carry_hover_n3`, `attach_*`, `carry_trim_n3*`, `demo_traj_*` | registry |

- Retire, or re-express around the offset, the `kt110/115/125` and `*_rigkt_m064/m100` configs. Mark them superseded in the registry.
- M2 bench: unchanged (Q5).

## 5. Experiment cards and critic passes

| experiment | card | critic | reviewer |
|---|---|---|---|
| W0 parity / parallel pilot | no; registry rows marked "infrastructure" | no | no |
| W4/W5 plant change | no; registry rows, BUILD row | no | no |
| R: reproduction | **yes**: it is the go/no-go for the sim as a rig proxy | no | no |
| A: W8 solve-failure handling (shifted horizon on failure), W9 pivot, W10 pull model, W11 freeze removal | **yes**, one card with separate arms, one variable per arm | **yes, one pass**, under "## Critic" in `docs/experiments/<date>_tethered_model.md`. W9/W10 change the flight-loop model; W8 changes what the loop publishes on a failure. | **yes**, on the final section-1 table before the rig |
| W12 tracker consistency | no; exploratory row with a falsifier | no (estimator bookkeeping, no new law) | covered by A |
| W13 safety-net defaults | no (threshold, Wesley's word) | no | no |
| Rig visit | **yes** (rig go/no-go) | covered by A | covered by A |

## 6. Rig card for the next visit (even ring unless Q2 says otherwise; target 0.5)

**Pre-flight:**
- The ring weighed with its plates, and one rod plus magnet weighed (before the visit, Q3).
- Packs ≥ 23.6 V; telemetry non-zero on all four (T1b: drone 1 read 0.00 V); rods measured and accepted.
- The planner prints "pivot on" and the rod lengths read 0.54-0.57.

| step | what | pass | stop and LAND on |
|---|---|---|---|
| P1 (optional, Q12) | one drone, free hover at 1.3 m, 20 s | model throttle within 0.01 of the law | — |
| H1 | tethered hover, z_ki 0, hold 25 s | breakaway ≥ 85 %; \|mean z − 0.5\| ≤ 5 cm over the last 20 s; heave p-p ≤ 6 cm; ring z sd ≤ 2 cm; tilt mean ≤ 3°, max ≤ 8°; solve failures ≤ 1; reference age > 0.3 s ≤ 5 %; ring vz ≤ 0.15 m/s; no release | any of the stop rules below |
| P2 (optional, Q12) | tethered holds at ring 0.3 m and 0.7 m, z_ki 0 | reported: rod-force sum against weight (H3b) | same |
| H2 | as H1 with z_ki 0.4, bound 0.15, gate 0.25 | the H1 bars, \|mean z − 0.5\| ≤ 2 cm, \|z_bias\| ≤ 0.10 throughout | z_bias at its bound: finish the hold, LAND, no circle |
| C1 | slow circle, only after H1 and H2 each pass once; r 0.5 at 0.125 m/s (Q13), net on | ring tilt mean ≤ 3° over the lap; \|z err\| ≤ 5 cm; xy err ≤ 10 cm; 0 solve failures; no release | same |

**Stop rules** (LAND now on any of):
- a magnet release;
- ring tilt > 5° in the pretension or > 12° in flight;
- ring vz > 0.25 m/s;
- planner status ≠ 0 three times or more;
- a creep timeout or refusal.

**Session rules:**
- After the first release, no further lift until the log has been read. After two releases, no more lifts that session.
- If the same bar fails twice, stop and report (time box).

## 7. Risks and what could make this plan wrong

1. **H3b is aerodynamic.** If the ring weighs 0.86 kg, the deficit is aerodynamic or a tethered thrust error that Gazebo cannot show. The sim will then pass at breakaway ~100 % while the rig still lifts at about 80 %. Without the freeze the rig then repeats f6: run-up at 0.38 m/s and a magnet release. Mitigations: Q9 fallback, the vz stop rule, and P2 to measure the effect.
2. **The pivot brings back the flip.** Putting the 4 cm pivot back in Gazebo can reopen the post-weld flip (`decisions.md:35`) and flip drones under a weak rate loop (T0008/T0010: 83.9° and 75.2° on P-only). This depends on C4 (ki 5 or 10). T0015 flew 4 cm pivots cleanly with the I-term, but not the attach transient.
3. **Undamped rod modes.** 75 g rods on two free balls, with DART ignoring the damping, add modes that the 1 g rods did not have. The heave bar could fail in sim for a sim-only reason.
4. **The approximate replay is not faithful.** The ring quaternion and ω are not logged (C6). The approximate replay may not reproduce status 4, so the W1 logging is needed before the exact cause is known. The W7 twin is the fallback.
5. **The magnet end is not a ball joint while the ring rests** (sub e4: distance grows 0.51 → 0.59 m with elevation). A sim ball joint will get the creep and hand-over wrong.
6. **Lab parity drift.** The laptop runs source-built Humble and the container runs apt Humble. If parity fails, every lab run is VOID until it passes. Starved wall-clock watchdogs under 6 parallel runs are the other known way to fail.
7. **Mixed plants.** Tejen's drones stay on the linear map (Q5), so M1 and M2 results mix plants.
8. **Pack-sag model.** It is ill-conditioned (τ 1-20 s all fit within 0.01 V rms). Voltage-sensitive conclusions should use the 22.8 V and 24.6 V neighbours, not the model.
9. **Model-form error.** The plant copies the tracker's affine form, so the sim cannot show an error from choosing affine over the multiplicative form (ladder1 fits both).

## 8. Questions for Wesley (the answer changes the plan; the default is what happens without an answer)

1. **Lab headless budget.** Default: 24 per session (at most 4 in parallel, ≤ 16 cores), with `/report` after each matrix. The laptop stays at 10.
2. **Ring layout for the next visit.** The rig flew the even ring, and `decisions.md:54` says 1/3/5/9. Default: even ring (the twin of model-f1), with 3915 as the sim neighbour.
3. **Weigh the ring and one rod plus magnet before any planner work**, and say where the ring's mocap origin is (top face or centre, for `attach_z`). Default: planner model work (W9/W10) waits for the weights. W0-W8 go ahead.
4. **Pack-sag model.** Default: the fitted dynamic sag (V0 24.4, τ 5 s, R 1.0, 22 mV per throttle-second), refit by `tools/fit_pack_sag.py`, rather than a linear ramp.
5. **Tejen's X3 and the M2 bench stay on the linear sim map.** Default: yes, and M2 is not re-flown.
6. **Model-only sim hold bar.** Default ≤ 5 cm, not 3 cm (the old sim sat +6 cm with z_ki 0, R0755).
7. **Attach, M1 and M2 worlds.** Default: keep them on the legacy geometry (0.5 m rod, r 0.25, centre pivot) until rig attach work starts. Regenerating now reopens the closed post-weld-flip line (`decisions.md:35`). This departs from "all differences", so it needs your yes.
8. **SIL pivot.** Default: end-point offset only, with the pivot torque judged in Gazebo. The alternative is rigid-body attitude plus a PI rate loop in SIL (extra work; inertia and authority not measured).
9. **Freeze removal on the rig.** Default: delete it from the flight logic, but keep `breakaway_freeze:=true` as a launch arg, default off, as a fallback if H1 lifts under 85 %.
10. **Takeoff spool (T2), per-drone offsets as launch defaults (T4), and a sim takeoff 'auto' = kT for floor starts.** Default: all three no for now. Per-drone a_i go in the plant only, so the sim sees the rig's offset mismatch.
11. **Re-baseline `gate_thresholds.yaml` sil_smoke to the new plant.** Default: yes, set from the first SIL run on the rig plant.
12. **Add P1 and P2 before the circle at the next visit** (about 5 min). Default: yes if the ring weighs 0.86, skip if it weighs ≈ 0.72.
13. **Circle in the lab space.** Default: r 0.5 m at 0.125 m/s.
14. **Sim rate_ki (C4).** Default: 10, as decided at `decisions.md:41`, with the code default fixed to match. Check the baselines' logged value first.

**Files** (all absolute):
- Inputs:
  - /home/wesley/multi_drone_control/docs/rig_2026-09-30_review.md
  - /home/wesley/multi_drone_control/docs/experiments/2026-09-30_pretension.md
  - /home/wesley/multi_drone_control/results/registry.csv (rows 578-605)
  - /home/wesley/multi_drone_control/results/rig/2026-09-30/model_f1_logs/logs/controller_quad_load/load_planner_20260930_172526/log.csv
  - /home/wesley/multi_drone_control/tools/remote/README_lab_access.md
- Code checked:
  - /home/wesley/multi_drone_control/src/controller_load_mpc/controller_load_mpc/planner_node.py
  - /home/wesley/multi_drone_control/src/controller_load_mpc/controller_load_mpc/planner_solver.py
  - /home/wesley/multi_drone_control/src/utility_objects/utility_objects/run_context.py
  - /home/wesley/multi_drone_control/tools/run_dir.py
  - /home/wesley/multi_drone_control/tools/run_experiment.py
- Scratch files from the investigations: local scratch (not kept)
---

## B. Critic

**Verdict: needs changes.** The plan's diagnosis is mostly sound, but four things stop it being approved as written. It would reverse a decided rate-loop setting (C4/Q14). Its default for dropping the pull freeze on the rig sets up a repeat of f6. Its sim claim cannot tell whether that change is safe on the rig. And one rig pre-flight check fails on drone 0's own rod.

## Must-fix

1. **C4 and Q14 are wrong and would revert a decision.**
   - `decisions.md:41` (09-26, rate_ki 10) was replaced by `decisions.md:47` (DECIDED 2026-09-27): "Sim rate-loop I-term ki 5 on every bridge (10 sat on the 3.2 Hz PI zero: R0647 limit cycle…)".
   - The code default of 5.0 (`payload_betaflight_comm.py:96`) already matches that decision.
   - Q14's default ("10, with the code default fixed to match") would bring back the R0647 limit cycle and invalidate the T0018/T0019 claim.
   - Fix: delete C4 and Q14, and keep ki 5.
   - Risk 2 also needs a correction. The pivot evidence it relies on (T0015) was flown before the rate loop moved to the simulated gyro by default (`decisions.md`, DECIDED 2026-09-28). The 4 cm pivot has never been flown on the gyro rate loop with ki 5.

2. **The rig should not drop the freeze by default while the force deficit (H3) is open.**
   - Q9's default takes the freeze out of the flight logic, with `breakaway_freeze` off.
   - The sim twin carries the true ring mass, so it cannot show the rig's 80 % airborne force balance. The plan admits this in risk 1.
   - The rig card still flies H1 without the freeze before P2, which is the step that measures the deficit.
   - The stop rule on vz > 0.25 is a LAND. In f6 the hard stop itself released drone 1's magnet (review §1; registry row 595).
   - Fix:
     - Make the freeze the rig default until the ring is weighed and either H3a closes or P2 shows about 100 %.
     - Fly P2 with the freeze on, before H1.
     - Say what the vz and tilt stop rules actually do: a soft hold or ramping the pull down, not an abrupt LAND.

3. **The sim claim cannot tell whether freeze removal is safe on the rig.**
   - The old sim already floated at the end of the pretension (R0766: +1.6 cm, pretension card). So "breakaway ≥ 90 %" passes on the pre-fix code and does not show the fix works.
   - Fix: add a neighbour with a deliberate mass mismatch to the W11 claim: world ring 0.70, planner 0.86 (the rig's measured condition), and optionally the other sign.
   - The plan's own A2 arm is optional and sits only in the reproduction. That is not enough. CLAUDE.md asks for "the other sign of the error" as a neighbour.

4. **Once the freeze goes, the breakaway metric changes meaning.**
   - W11 changes the trigger (all plates +5 mm, or ring +1 cm with tilt < 3°). W10 redefines 100 % as statics at the measured directions.
   - Rig H1's "≥ 85 %" and the sim's "≥ 90 %" are then not comparable with the 60-80 % baseline, which used vz > 0.03 (`planner_node.py:97-99, 767-771`).
   - Fix: take the measured rod-force sum over the ring weight (the W3 `--tethered` mode) as the pass metric. Recompute model-f1 and f6/f7 under the new trigger offline to get the baseline.

5. **W8's shifted horizon defeats the reference-staleness safety gate.**
   - If a failed solve republishes the previous horizon shifted by one node, the trackers' reference-age watchdog sees a fresh message. The watchdog is 1.0 s on the rig (`decisions.md`, 09-26 line), and a trip is a fleet-wide disarm (locked decision).
   - A planner stuck in failures would then never trip it.
   - Fix:
     - Cap the consecutive shifted publishes (at most N nodes), then let the reference go stale.
     - Keep the original stamp, or log a separate age.
     - This is a change to a gate that can ground the fleet, so the critic must cover it explicitly on card A.

6. **W5 changes the drone model shared by the legacy worlds.**
   - `x3_drone{0..3}.sdf` is included by `three_attach*.sdf`, `three_attach_partner*`, the weld variants and `three_random_seed*` (grep of `simulation_assets/`).
   - Setting base_link to 0.435 moves every attach, M1 and weld world that Q7 says stays legacy.
   - The same problem applies to the params.py and launch-default changes: `dissipative_launch` and `three_attach_launch` configs flying 0.5 m worlds would silently take 0.55 / 0.225 / 0.55.
   - Fix: either new model files for the rig twin, or pin cable_len, attach_radius and drone_mass in every legacy config. List which one.

7. **The rig pre-flight rod check fails drone 0's own rod.**
   - "Rod lengths read 0.54-0.57" rejects drone 0, whose pivot-corrected length is 0.537-0.545 (rig-data e2).
   - Fix: use per-drone expected values, or a band of about 0.53-0.58.

8. **The kT shadow is not independent evidence (C3, H3, rig-data a5).**
   - The estimator is fed `self._cable_static_z` (`controller_mpc.py:1071-1072`). That value is the planner's static share, computed with load_mass 0.86 (`planner_node._publish_static_cable`).
   - So the +5.8 % (`model_f1.log`: d1 37.23, d2 37.25) is the same throttle-below-law observation as a3, not a second measurement.
   - Fix: remove "independently agrees" and treat the deficit as one measurement.

## Should-fix

1. **Evidence for H3b (aerodynamic support near the floor) is weak.**
   - The height fit (a4) comes from oscillating flights where force leads height by 0.3-0.6 s (rig-data c2). That correlation appears in any force-driven heave.
   - f11 had the integral pinned at its 0.4 bound, and the ring swung with sd 0.187 m.
   - P2 needs a steadiness bar (for example |vz| < 0.03 m/s for at least 5 s per level), or it repeats the confound.

2. **A candidate for the deficit is missing.**
   - The ladder was flown with the rod, magnet and hung mass hanging vertically in the downwash (registry rows 600-601). When tethered, the rod runs out at about 55°.
   - Downwash drag moving out of the wash would read as extra thrust, and weighing the ring would not detect it.
   - Cheap rig check: free hover with the rod removed (0.475 kg) against the law.
   - Also weigh one drone as flown and the ladder's hung masses. The slope 0.507 depends on them.

3. **H2 ranking.**
   - The reference at 45° was held against rods pinned at 53.6° from hand-over (151.25) to breakaway (154.59) with no failures.
   - The first failure, at 155.04 (`model_f1.log`), follows lift-off.
   - So (b) alone is not the trigger. Rank (a), (d) and (e) (lift-off dynamics, stale pose, ω noise) above it.

4. **Don't make W7 gate W8.**
   - The W8 reseed fix is justified by the code (`planner_solver.py:201-203` sets only x, then COLD_ITERS 15 at `:33`) and by the W2 replay.
   - A Gazebo run with ideal mocap may never reproduce status 4. As written, a failed reproduction would stall W8-W11.
   - W8 does not touch the floor, so its Gazebo floor run is not required. One SIL run is enough.

5. **The heave has no work item.**
   - f7, f10 and f11 heaved with 0 failures (f7 sd 4.5 cm), and the old sim held level (R0767).
   - The rig bars "heave p-p ≤ 6 cm" and "ring z sd ≤ 2 cm" may fail even with W8 working, and the sim may never show it.
   - Add an f7 replay item, or state that the bar is untested in sim.

6. **Ordering.**
   - W0 (Docker, parity, a batch tool; 12 runs) is the first item, but it is not in Wesley's decisions.
   - W1-W3 need no runs and are what the next rig visit most depends on (exact replay).
   - Take W0 off the critical path.
   - The full `gate.sh` runs sil_smoke (`tools/gate.sh:119-150`). After W4 it fails until `gate_thresholds.yaml` is re-baselined, which needs Wesley's word. That has to happen before the first Gazebo run, not in W14.

7. **Budget arithmetic.**
   - W11: 2 + 6 neighbours = 8, not the 7 stated in W11 or the 9 in the totals.
   - B0 is the same config as the W11 even-ring claim; merge them.
   - The W9 and W10 floor runs are needed (creep and pretension touch the floor); the W8 one is not.

8. **Twin configuration gaps.**
   - Rig `z_taut_gate` is 0.9 (`real_control_launch.py:98`) against 0.99 in sim (`planner_node.py:1162`). The twin launch args omit it.
   - `attach_z` is also not stated in the twin args (rig 0.0 against the 0.020 plane in W5).
   - Reference age must be measured in sim time. Parallel lab runs at low real-time factor produce stale references on their own, which would confuse the "≥ 25 %" reproduction bar.

9. **The safety net (H2) depends on the pivot fix (W9) on the rig.**
   - In flight, drone 0's taut gate is about (0.580 − 0.85·0.613)/(0.15·0.613) ≈ 0.64, below 0.9 (`planner_node.py:1180`).
   - If W9 is not ready, H2 cannot run. The card needs a fallback, for example z_taut_gate lowered, with Wesley's word.

10. **Floor-phase rod measurement.**
   - The rod length is measured at hand-over, with the ring on the floor, where the magnet end is not a ball joint (rig-data e4: 0.51 → 0.59 m with elevation).
   - Consider tape-measured, typed rods per drone.
   - Add a hand-over elevation bar (45 ± 5°) to the rig card. Model-f1's creep read 51-58° against a 40-45° reference (`model_f1.log`, arc-creep lines).

11. **W12 T1/T3 change `_applied_u`, the tracker MPC's pinned throttle state.** That is estimator bookkeeping in the flight loop. Put it under card A's critic, or say why not.

12. **Planner thrust bound.** Include `thrust_max` of about 11.9 N at the 0.8 cap (against 15 N at `planner_ocp.py:58`) before the circle.

13. **Unverifiable inputs and small inconsistencies.**
   - The scratch files the plan cites are absent in this environment, so every (sub) figure is unverified.
   - Section 1 still quotes "16 cm" as the model-f1 hold baseline, but C2 shows it is ramp data.
   - My check: z_tgt ≥ 0.45 for 3.78 s, ≥ 0.49 for 1.13 s, ring z 0.300-0.398, mean 0.339. That confirms C2.

## Missing questions for Wesley

1. When is the next rig visit, and how long is it? No date was given, and the answer decides what gets cut from roughly 34-38 Gazebo runs.
2. Should the freeze stay the rig default until the ring is weighed and H3 is closed? (This replaces Q9.)
3. Can the ring, one drone as flown, the rod plus magnet, and the ladder's hung masses be weighed now, before W7? And can one free hover with the rod removed be added to the next visit?
4. CLAUDE.md's locked decisions say "sim: 83.1 on the linear plant" and "kt_trim default on". Should they be updated for the rig plant? Should the sim keep kt_trim on while the rig runs it off?
5. Who watches the stop rules (ring vz, tilt, solve count) live, and on what screen? Does the monitor show them, or should the planner soft-hold by itself?
6. Is lab Docker compute (W0) wanted now, and under what run budget? CLAUDE.md sets 10 headless runs per session. `decisions.md` has "no cap" only for the GOALS loop.
7. Should the 09-25 "pivot at body centre in sim" lines be superseded for the carry worlds only? They need a new DECIDED line either way.
8. Is lowering z_taut_gate on the rig acceptable if the pivot fix (W9) is not ready?

Files checked:
- /home/wesley/multi_drone_control/docs/decisions.md
- /home/wesley/multi_drone_control/src/controller_load_mpc/controller_load_mpc/planner_node.py
- /home/wesley/multi_drone_control/src/controller_load_mpc/controller_load_mpc/planner_solver.py
- /home/wesley/multi_drone_control/src/controller_quad_load/controller_quad_load/controller_mpc.py
- /home/wesley/multi_drone_control/src/simulation_communication/simulation_communication/payload_betaflight_comm.py
- /home/wesley/multi_drone_control/tools/gate.sh
- /home/wesley/multi_drone_control/tools/run_experiment.py
- /home/wesley/multi_drone_control/results/rig/2026-09-30/model_f1.log
- /home/wesley/multi_drone_control/results/rig/2026-09-30/model_f1_logs/logs/controller_quad_load/load_planner_20260930_172526/log.csv
- /home/wesley/multi_drone_control/simulation_assets/ (the worlds that include `models/x3_drone*.sdf`)
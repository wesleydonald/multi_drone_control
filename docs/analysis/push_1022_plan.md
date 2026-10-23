# Ultracode push 22 Oct: designs, order of work and decisions

Six read-only designers and a critic (workflow wf_5bad4277-125, 22 Oct 13:25-14:55). Full designs:
`results/analysis/push_1022_designs.json` (every change with file and line, tests, configs, risks). Scope and code
authority: decisions.md, DECIDED 2026-10-22 ultracode push.

## The packages

| Package | What it delivers | Work | Twin runs | Card / critic |
|---|---|---|---|---|
| 1m-defaults | Make 1 m cables the project default and give the figure-8 its own size knob, without changing what any existing config or run means. | About 1.5 days: | 5 | - / - |
| pair-swing | I measured the rig's loop delays against the twin from today's bags and tracker logs, using read-only scripts in /tmp/claude-1000/-home-wesley-multi-drone-control/f2a54328-3214-4ac3-989e-f5a0c5208168/scratchpad/design/pair-swing/. Then I used those numbers to  | About 3.5 days of work, in order: | 38 | card / critic |
| sim-U | How far off its slot the magnet can catch before the task fails, measured in the twin. The 3->4 attach uses the M1 layout at 1 m: carriers on plates 3/11/7, and a QUAD4-like newcomer with a 1.0 m rod that flies our own approach to plate 9. A new twin-only knob | About 2 days of work, plus compute: | 120 | card / - |
| sim-S | S can reach a thesis-worthy result with the flight machinery that exists today plus two small speed fixes that do not change any output. | About 3-4 agent-days plus 2 lab evenings, in order: | 47 | card / - |
| sim-C | Thesis result C: the paper's method against our planner, in the twin only. The paper's method here means our port of Quan et al.: the spring-damper network (src/dissipative_planner/dissipative_planner/dissipative_network.py) feeding the paper's velocity loop ( | About 4-5 working days in order of work: | 45 | card / critic |
| re-fly-R | R re-flies at 1 m every thesis table and figure built only from 0.55 m twin runs, using the same arms, repeats and frozen bars as the original. Twin results tied to a 0.55 m rig flight, or to the retired attach sequence, are kept as labelled history. That is 7 | Desk work, about 1.5 days: | 150 | card / - |

## Critique (verbatim from the critic agent, sections 1-5)

(Two of its questions are left out of the decision list: committing the tree, which waits for Wesley, and merging
tejen-sync2, a git action listed here as a dependency for QUAD4's 1 m tether geometry. Worktrees therefore start
from be34f95 with the main tree's uncommitted diff applied.)

# Critique of the six package designs

I checked the designs against day-1021 at be34f95 plus today's 19 uncommitted files, and read the live state at 14:56–15:15 on 22 Oct. Nothing in the repo was edited.

**Live state:**
- A lab batch is running: the c3 delay calibration, `lab_batch.py` PID 125537, 4 slots, started about 14:54.
- No laptop run is live.
- Laptop disk: 15 GB free (92% full).

---

## 1. Conflicts and how to resolve them

**C1. `four_rigid_ground_rig_3915_L100.sdf` is defined twice.**
- 1m-defaults adds it to WORLDS with `--tether-pose-hz 60`. re-fly-R creates it through `R1M_SIBLINGS` without the publishers. WORLDS is a dict, so whichever runs last wins without a word.
- **Fix:** one entry, owned by 1m-defaults, with the publishers. They do nothing unless io `tether_tips` is on (`/home/wesley/multi_drone_control/src/bringup/launch/sim_io_launch.py:116-129` only bridges them then).
- R drops 3915 from its sibling list. Its "only the cable differs" test must allow the publishers. The 0.55 m push world already differs from its base the same way (checked with diff).

**C2. Both 1m-defaults and sim-U rewrite `/home/wesley/multi_drone_control/tools/make_attach_worlds_L100.py`, with the same output names.**
- **Fix:** keep the 1m-defaults version (the whole family: 8 worlds and 3 models). U uses its `three_rigid_ground_rig_attach_L100.sdf` and `x3_rig_drone3_magnet_L100.sdf` and drops its own generator edit.

**C3. `DEPARTED_CLEAR_R`: the three designs want different things.**
- 1m-defaults keeps it at 1.2 m (its Q2).
- R lists "1m-defaults scales `DEPARTED_CLEAR_R`" as a prerequisite for R-D.
- S wants it to follow the ring: on the R 0.5 ring the carriers sit at r 1.0.
- **Fix:** `clear_r = 1.2 + max(0, reach − 0.725)`. That gives 1.2 m for the rig ring at both cable lengths and 1.475 m for S's ring.
- R's real worry is a freed drone landing inside the orbit's swept disc. The path-clear scaling already covers that (1.225 m at 1 m, so 1.725 m from the orbit centre). R-D therefore needs 1m-defaults' path-clear change, not a larger `DEPARTED_CLEAR_R`.

**C4. The new figure-8 rule would refuse configs the other packages write.**
- 1m-defaults' `validate()` refuses any `fig_8` config that does not type `fig8_size`. Today:
  - U's template types `traj_radius 1.0` only ("regardless");
  - R's transform types `traj_radius` ("or the key");
  - S's s4/s8 figure-8 configs are copies of `b4_fig8big`, which types `traj_radius`;
  - pair-swing's round-D cells are figure-8s too.
- All of these would be refused, and `test_the_shipped_experiment_configs_are_valid` would fail in the gate.
- **Fix:** once 1m-defaults lands, every new figure-8 config types `fig8_size`:
  - U's template: `fig8_size 1.0`;
  - R's transform: the migrated sources' 0.5 becomes 1.0;
  - S and pair-swing: copy from the migrated files.

**C5. `latency_bench.py`: both packages edit the same functions.** 1m-defaults adds `--rod`, `--fig8-size` and the RingPlant rod CoM; S adds `build_solver(inertia=)`, `--n` and `--rod`. **Fix:** 1m-defaults first, then S rebases. Keep one `--rod` flag.

**C6. `planner_replay.py`: both packages edit it.**
- 1m-defaults adds `path_radius` with a `--src` fallback, and sets arm 'c' rods to the flight's `cable_len`.
- S adds the `i<k>` arms, `--quality`, `--q-source auto` and `--flown-pivot`.
- S's check that "the base arm gives identical node-0 refs" can only pass if its fidelity changes are opt-in flags whose defaults keep today's behaviour.
- **Fix:** 1m-defaults first; S makes the fidelity changes opt-in.

**C7. `planner_node.py`: three packages edit it.**
- 1m-defaults: the resolver at :360/:382 and the CABLE LENGTH warning.
- S: `plan_res_every`, `_publish_refs`, and `finish_tick()` in `_plan`'s finally block.
- C: appended log columns, `_tick_t0` in `_plan_timer`, blanked OCP columns, and `_log_tick` called from `_network_plan`.
- **Fix:** integrate 1m-defaults, then C, then S. S's `finish_tick` must do nothing on a tick without an OCP solve (C's network ticks).

**C8. The logged `solve_ms` changes meaning under S.**
- `/home/wesley/multi_drone_control/src/mpc_planner/mpc_planner/planner_solver.py:324` times from t0 to after `_residuals()`. S moves the residual out, so `solve_ms` drops by about 4 ms at n=4 and about 43 ms at n=9.
- Every earlier run logged it with the residual included. That includes the thesis's rig solve-time numbers, C's compute column and S's 22 Oct JSON.
- **Fix:** add an appended `res_ms` column, and have every table say which compute number it uses.

**C9. `dissipative_node.py` is edited by four packages, in different functions.**
- 1m-defaults: the reach helpers and the `tip_valid` call.
- U: `attach_point_source`, `_attach_ocp`, the ATTACH line, and `rho` in RESIZE_INPUTS.
- S: the prebuild sizes at :704-709.
- C: the network parameters, `_enter_network_phase` and `_network_plan`.
- **Fix:** integrate 1m-defaults, U, C, S in that order. After each, run the whole `src/dissipative_planner/test` suite. The stub tests need `getattr` fallbacks for every new attribute.

**C10. Many config, launch and test files are shared, so one person integrates them, in package order.**
- `sim.yaml`: 1m-defaults, U (`modes.attach`), C (`modes.network`, header).
- `common.yaml`: 1m-defaults, S, C.
- `real_mode.py`: 1m-defaults, S.
- `profiles.py`: 1m-defaults (KEPT_COMMON, the `apply` check), C (`LEGACY_ONLY`).
- `graph_attach.py`: 1m-defaults (refusals), U (aim).
- `test_real_mode_launches.py`: 1m-defaults, U, C.
- `test_config_tools.py`: 1m-defaults, C.
- `test_rig_worlds.py`: 1m-defaults, R, S, pair-swing.
- `gate.sh`: 1m-defaults and C (step 4), S (step 2).
- `tools/experiment/config.py`: 1m-defaults, pair-swing.
- `run_experiment.py`: 1m-defaults, S.
- `make_carry_worlds.py`: 1m-defaults, R, S, pair-swing.
- `tracker_node.py`: pair-swing, C.
- `payload_mocap_emulator.py`: 1m-defaults, pair-swing.
- `thesis_sim_tables.py`: 1m-defaults, C, R.

**C11. `table_params` makes R's "0.55 m outputs stay byte-identical" check fail.** `table_params` in `/home/wesley/multi_drone_control/tools/thesis_sim_tables.py:294-310` reads `real.yaml`. Once 1m-defaults changes `real.yaml`, the '055' set prints 1.0 m. **Fix:** take the parameters per run set, not from `real.yaml`, or leave that table out of R's diff.

**C12. Pair-swing could move the twin's timing default under R, U, S and C.**
- Pair-swing may propose a "rig-timing twin" default after round B. The claim tables and the frozen b22 bands assume today's twin: ema velocity filter, no command delay, no body-rate EMA.
- **Proposed policy:**
  - no command delay as a default in this push: it faults landings and creeps (R2366, R2379);
  - the body-rate EMA becomes a default only if round B plus a 3/4-quad no-harm check pass before the claim freeze;
  - otherwise the claims fly today's twin and the thesis states the difference.

**C13. The rig's `attach_z`: 1m-defaults and pair-swing point different ways.**
- 1m-defaults Q1 makes 0.0175 (Wesley's 22 Oct value) the `real.yaml` default. Pair-swing puts the effective hinge at 43-54 mm and proposes typing about 0.05, with cable 0.53.
- Wed 28 is not affected: every sheet line types 0.0175.
- **Fix:** 0.0175 as the default now. The hinge value becomes a later rig A/B, after round C and a 1 m marker measurement; the 0.55 m numbers don't carry over to the 1 m cables.

**C14. C's reach term "for B's bands" would change bars already frozen.** The b22 bands are frozen in the Wed sheet, so the reach term can only be a reported column on Wed 28, never a bar.

**C15. C's new runner events land in everyone's runs.** C's `_ROSOUT` regexes (`PATH_START` matches the parent planner's "hover hold done" line) add rows to every path run's `events.csv`, including R, U and S. **Fix:** land them before the claim freeze, and check any scorer that reads events by position.

---

## 2. Gaps the implementation will hit

### Integration mechanics

**G1. With no commits there is no base for worktrees.** A worktree from be34f95 lacks today's 19 modified files: LAB_ENV, the twin's ema default, `SIM_CMD_DELAY_FROM_S`, the clock-free `n2_bands`, the constant-rate `line_spin`, and the carrier map. U, R, C and pair-swing all build on these. Fallback without a commit: apply `git diff HEAD` plus the untracked files to each worktree, and integrate by patch.

**G2. A worktree's tests and tools run the main tree's code.**
- No `conftest` exists, and the installed packages are symlinks into the main `src` (`build/mpc_planner/mpc_planner` points at `/home/wesley/multi_drone_control/src/mpc_planner/mpc_planner`). So tests in a worktree import the main tree's packages.
- `repo_root()` resolves through the installed `utility_objects`, so `results/` and `generated/` land in the main tree.
- Gate step 1 reports WRONG WORKSPACE.
- **Fix:** each worktree gets its own `colcon build --symlink-install` (as `/home/wesley/mdc_wt/tejen_sync2` has), or PYTHONPATH plus `MDC_REPO_ROOT`, `MDC_ACADOS_ROOT` and `MDC_RESULTS_ROOT` pointed at scratch. No Gazebo runs and no registry rows from worktrees.

**G3. The laptop is both the development machine and a twin host.** pytest and colcon in a worktree starve a live laptop run (the 9 Oct voids). Develop while only the lab runs, run laptop batches overnight, and never do heavy work in a run's first 3 minutes.

**G4. SESSION_STATE says "one general-purpose helper agent at a time".** Parallel implementers need Wesley's word (Q1).

**G5. Several files need a single writer:** `registry.csv`, `SESSION_STATE`, `decisions.md`, the plan's status table (1m-defaults and R both own row 3), `CLAUDE.md`, and the Wed sheet.

### Run ids and provenance

**G6. Two runs can get the same R id without any error.** `/home/wesley/multi_drone_control/tools/run_dir.py:82-95` scans for the next id, then calls `makedirs(exist_ok=True)` with a different slug. A laptop run and `lab_batch.reserve` (`/home/wesley/multi_drone_control/tools/remote/lab_batch.py:224`) allocating within the same seconds both get the id. This matters with the laptop and lab running in parallel all week. **Fix:** an flock around the scan and mkdir in both places.

**G7. "One code tree per claim table" cannot be checked from the manifests.** `diff_sha256` hashes `git diff HEAD` over every tracked file (`run_dir.py:48`), so each registry or docs edit changes it. **Fix:** freeze the tree with a commit from Wesley, or record a code-only hash (`src/`, `tools/`, `configs/`, `simulation_assets/`).

### Tests and gate

**G8. `gate --quick` skips most of the suites these packages touch.**
- Only 4 of the 31 tests in `src/mpc_planner/test` are listed (`/home/wesley/multi_drone_control/tools/gate.sh:46-79`). Outside the gate: `test_planner_log`, `test_solve_fallback`, `test_pivot_offset`, `test_line_spin_trajectory` and the new `test_fig8_size`.
- Only `test_fleet_viz.py` runs from `src/simulation_communication/test`, so pair-swing's two new tests would never run.
- `verify_dissipative`, which backs C's "tests A-K unchanged", runs only in the full gate.
- **Fix:** list the new tests in step 2. At each integration step also run `pytest src/mpc_planner/test src/simulation_communication/test` and `python3 -m dissipative_planner.verify_dissipative`.

**G9. 1m-defaults breaks existing combined-rig launch tests.** `test_real_mode_launches.py` uses `QUAD4_RECORD` with `attach_cable_len` 0.54 (:850, and the explicit cases at :944) against the profile's `cable_len`. With `real.yaml` at 1.0, the new refusal for a gap over 0.15 m fires in all of them. **Fix:** type `cable_len 0.55` there, or use a 1 m record.

**G10. The migration lists are already stale.**
- There are now 44 `fig_8` configs: 15 type `traj_radius` and 29 rely on the default. The design's scan found 36; `b3`/`b4 *_cd060` and the `c3_*` calibration configs were added since.
- **Fix:** generate the migration from a live scan at implementation time, and do the same for the `attach_cable_len` list.

### Solver caches

**G11. S's builds would evict everyone else's on the laptop.** There is one `.so` and one signature per n per code directory. S's 0.9 kg / inertia builds of n=2..4 would replace the 0.86 kg ones. S must use `MDC_ACADOS_ROOT=generated/s_ring` for its runs, bench and replays.

**G12. S must not touch two solver source files.** S's `kinematics_funcs` must stay in `planner_solver.py` and `geometry.py`. `SRC_FILES` (`planner_solver.py:55`) is `planner_ocp.py` plus `load_cable_dynamics.py`; any edit there invalidates every cache on the laptop and in each lab slot, and runs then compile mid-flight and are void.

**G13. The lab warm-up only prebuilds the fleet's own size.**
- `warm_script` (`lab_batch.py:196-210`) builds n with the masses. Resize sizes and inertia overrides compile inside the run.
- S's `--from-config` fix changes every lab batch. Land it with a `--dry-run` plus one real batch before the claim batches, not between them.

### Short-cable constants and tools still at 0.55 m (not in 1m-defaults' list)

**G14. The approach tip loop's lean limit is still sized for 0.55 m.** `approach.py:42` `TIP_XY_MAX` is 0.15 m. At 1 m that is 8.6° of rod lean instead of 15.8°, so the tip loop will reject samples as 'lean' in the hold. Scale it with body-to-tip, as the band is.

**G15. The tip-detect constants were set on 0.55 m bags.**
- `dissipative_node.py:81-88`: `TIP_ARM_MAX_M` 0.06, `TIP_ARM_SLACK_M` 0.15, `TIP_BAND_M` 0.03, `TIP_RADIAL_M` 0.015.
- Wed's Part C runs `chain_pair_tip.yaml` at 1 m. After the `tip_replay` fix, gate 3 must replay the 1 m A/B bags before Part C. Any change to these constants is a card amendment.

**G16. `rod_spread_m` 0.08 is an absolute length.** It is in `real.yaml:14` and `sim.yaml:15`. If the measured 1 m cables differ by more than 8 cm, the planner silently falls back to the typed length (`planner_node.py:281`). Compute the spread from Wed's bench hanging lengths.

**G17. The model integral's gains were identified at 0.55 m.** `int_k_xy` 5.26 and `int_k_z` 7.13 (`common.yaml:52-53`) came from 0.55 m rods at 45°, so the integral's force per metre is off at 1 m. R flags it; 1m-defaults does not.

**G18. `tools/attach_metrics.py:46` has `BODY_TIP 0.59`.** At 1 m it is 1.04; read it from `params.json`.

**G19. The rig's environment guard misses some switches.**
- The unset list in `tools/rig_flight.sh:74-78` lacks `TRACKER_DELAY_COMP_S`, which `tracker_node.py:498` already reads (an existing gap). It would also lack pair-swing's new `TRACKER_W_VEL_SCALE` and `TRACKER_DELAY_COMP_THR_S`/`_ATT_S`.
- The mocap switches (`MDC_MOCAP_RATE_FILTER`, `MDC_MOCAP_VEL_RING`) are read by the T1 mocap node, which `rig_flight.sh` does not launch. Pair-swing's F3 says that list protects them; it cannot.

**G20. The twin's carrier list for the velocity filter defaults to three drones.** `payload_mocap_emulator.py:83` defaults `MDC_MOCAP_VEL_DRONES` to '0,1,2'. Any slope config with 4 or 8 carriers must list all of them; R does, S and C must.

**G21. 1m-defaults' CABLE LENGTH pre-flight warning will fire wrongly on the rig.** Slack 1 m cables make a short rim distance normal there, so warn only on over-length. The warning also overwrites the latched `/fleet/manager_status` line that SLOT OFFSET uses.

**G22. The twin ring is still symmetric.** The worlds keep 0.02/0.02, and decisions.md says "until they are regenerated". Regenerating moves every table, so it needs a decision (Q4).

### Design-specific

**G23. Pair-swing's tip world as written comes out at 0.55 m.**
- `make_carry_worlds.py:130` passes the WORLDS arguments before RIG_ARGS, and RIG_ARGS contains `--cable-len 0.55`, so a `--cable-len 0.53` in WORLDS loses.
- **Fix:** put it in WORLD_OVERRIDES, as the `_L100` worlds and S's R50 worlds do. `attach_z 0.05` also needs S's per-world expectations in `test_rig_worlds`.

**G24. Pair-swing's round B and round A need adjusting.**
- Round B's "attitude +0" leaves the twin's attitude path at 49-57 ms against the rig's 17-24 ms (pair-swing's own numbers). Add a neighbour arm with faster emulated rate PID gains, or say in the card that the timing is only half matched.
  - *22 Oct 17:11, the Betaflight readout (`docs/rig/betaflight_2026-10-22.md`):* the rig's FC runs a setpoint-derivative feedforward (F 120-125) on P45/I80/D40 at 8 kHz, and no loop on collective thrust. The faithful neighbour is a feedforward on the rate setpoint's derivative in the emulator's rate loop (`rate_kff`, twin-only, default 0), set so `loop_timing.py att` reads the rig's 17-24 ms; the throttle delay stays the other half.
- Round A's baselines (R2365, R2381-R2388) are on an older tree. Re-fly one cd060 all-channel arm on the new tree.

**G25. C needs three fixes before its claim runs.**
- Our planner has never flown `line_x` at 0.4 m/s at 1 m (R1937-R2079 reach 0.3). Fly `c_pl_linex_v400` once as a neighbour.
- The network's LAND disarms the quads on the cone about 0.9 m up, which trips `forbid_abort`. The card must define where a run ends.
- The network hand-over steps the datum in `payload_ref`, so any `sweep_window` or clock-free window needs an explicit `t_from`. U's `n2_bands` change serves both U and C.

**G26. U's E1 and 1m-defaults' V4 fly the same thing.** Both fly our approach on the new free 1 m world. Fly U's E1 with `approach_tip_log` on (logging only, no behaviour change) and it answers V4 as well.

**G27. R's 1 m ledger band (R-L) needs rig data and a tool change by Wed afternoon.** It needs at least 8 s of steady hover before LAND in Wed's first 1 m rig hover (the `sigma_rig` window, `n2_bands.py:90-97`), so that hover goes on the sheet. `release_bands --factors` and `--sigma-runs` must exist by Wed afternoon.

**G28. Disk space.** With 15 GB free, R's state logs (about 2.5 GB), U's roughly 60 logs and S's 8-quad logs need the delete list or a render-and-delete step after every batch.

**G29. S's flight-path change on the rig would add a second variable.** References would go out 12-16 ms sooner, which mixes into Wed 28's comparison with the 22 Oct twin bands and into the pair work. **Fix:** put it behind a planner switch, default off. S's configs turn it on, and the rig stays off until the reviewer has seen it.

---

## 3. Order of work and run schedule

### Integration order

All integration happens in the main tree, by one integrator, never while a laptop run is live. After each step: the affected suites, `gate --quick`, then prebuild.

0. Wesley commits today's tree (Q0) and answers the blocking questions.
1. **1m-defaults A, needed for Wednesday:**
   - `tip_replay` (the `params.json` lookup and an exit code);
   - `fig8_size` end to end, with the live-scan migration and the `validate` rule;
   - `cable_len` 1.0, the gate step-4 pairs, and the test fixes including G9;
   - the 1 m attach family and `3915_L100` (C1, C2), plus `make_attach_worlds_L100 --check` in the gate;
   - the `tether_len` default and its check;
   - the attach-arm check and its exemptions;
   - the CoM defaults in `real.yaml`.
   Then fly V1 and V2.
2. **1m-defaults B:** the approach band and G14; the reach-scaled path and zone constants with C3's clear-radius form; the `graph_attach` refusal; the warning (G21). Then fly V3 and V4 (V4 merged with U's E1).
3. **pair-swing:** the per-channel delay, the emulator rate filter, the LAB_ENV names, its tests added to the gate, `loop_timing.py` and `pair_mode.py`. Its card bars for rounds A-C are frozen before its runs.
4. **U:** the aim knob, `attach_point_source` with a test that the default path is unchanged, `n2_bands t_from`, `catch_offset.py`, and the config generator (with `fig8_size`).
5. **R:** `make_refly_configs` (after the migration), the siblings and push world, `thesis_runsets`, `refly_score`, and the `release_bands` factors.
6. **C:** developed in a worktree. The network switches stay off by default.
7. **S:** developed in a worktree, behind a switch that defaults off.

**Code freeze:** once steps 1-7 are in, Wesley commits. R, U, C and S claims then all fly on that one tree. The velocity filter is the only open config value, set after Wed's A/B.

**Parallel worktrees:** C, S, the tools-only parts of R, and pair-swing's analysis tools, with code and offline tests only. Everything in C10 and every world regeneration stays sequential.

Implemented one at a time this is about 8 agent-days; with 3 worktrees it is about 3. The parallel version is what makes a code freeze before Wed 28 possible.

### Run schedule

These are the earliest windows. Rig days are fixed; anything not through its gates waits.

| Window | Laptop (one run at a time) | Lab PC (at most 4 slots, when free) |
|---|---|---|
| Thu 22 eve/night | V1, V2 after step 1; V3, V4/U-E1 after step 2 (about 45 min) | c3 calibration (running now); nothing new until step 3 is in |
| Fri 23 | Overnight: R-smoke (10 runs, about 1.7 h) and U E2-E5 plus neighbours (7 runs, about 1.5 h) | pair-swing rounds A and B (18 runs, about 1 h) |
| Sat 24 | C smoke (2 runs; `net_twin_geometry` on only after the critic); S smoke (1); S bench re-time in an idle window with no Gazebo | C offline search in a container with no Gazebo, or S 8-quad exploratory (4 runs, 2 × 12-cpu slots) |
| Sun 25 – Mon 26 | S replays (27, offline, laptop idle); renders | C candidates (10) and neighbours plus `c_pl_linex_v400` (4); pair-swing C (4, if the world is approved) and D (16) |
| Tue 27 | R-L (3) on the frozen code if ema stays (re-fly Wed otherwise); V5 | voids, spare |
| Wed 28 08:00-12:00 | RIG | — |
| Wed 28 eve/night (laptop stops before the rig stack is up) | R-A appendix block (54 runs, about 3.5 h, one host) | R-L and R-D (36 runs, about 1.2 h); U grid (52 runs, about 2.3 h) if the evening allows |
| Thu 29 08:00-12:00 | RIG | — |
| Thu 29 eve | renders, scoring | R-C (30) and the C claim (18, or 27 with both forms), about 1.5-2 h |
| Fri 30 – Sun 1 | voids, S lab-copy bench | U grid if not done Wed; S base and it1 arms (8); S 8-quad claims (4 runs, 2 big slots); U extension (8) |
| Mon 2 – Tue 3 | reviewers; tables `--set 1m` | voids; C2 (12, optional); pair-swing D 3/4-quad cells |
| Wed 4 / Thu 5 | RIG (K, re-flies) | — |
| Sat 7 | Results freeze | |

Totals: about 12-13 h of lab time over about 7 evenings, and about 13-15 h of laptop time. If the lab is lost, R-D, R-C and C move to laptop overnights on non-rig nights. The S 8-quad runs give way first.

---

## 4. Questions for Wesley

These are merged and deduplicated across all six designs. Each has a recommended answer.

### Before implementation starts

- **Q0.** Commit today's day-1021 tree now (by explicit path)?
  *Recommended: yes.* It is the base for the worktrees and for the named claim tree.
- **Q1.** May up to 3 agents implement in parallel worktrees (code and offline tests only, one integrator, no runs or registry rows from worktrees), despite "one helper at a time"?
  *Recommended: yes for this push.*
- **Q2.** What may reach the rig through Thu 29?
  *Recommended:* only 1m-defaults, `tip_replay` and pair-swing's read-only tools. S's planner speedups stay off on the rig until the reviewer (from 4 Nov). C's log columns ride along, since they only add log columns. U's and pair-swing's code is twin-only or off.
- **Q3.** `attach_cable_len` means QUAD4's rod from its pivot. Refuse a config whose value disagrees with the world's arm; old 0.59 and 1.04 configs get a named exemption and stay runnable as flown; 1 m twin configs leave it untyped. OK?
  *Recommended: yes.*
- **Q4.** Make the ring's centre of mass the rig default (`real.yaml attach_z` 0.0175, `payload_rest_z` 0.0275, `RIG_DEFAULTS` 0.05 → 0.0275 for Thu 29's `mode:=attach`)? And keep the twin's symmetric ring (0.02/0.02) through this push?
  *Recommended: yes to both.*
- **Q5.** Departed clearance: keep `DEPARTED_CLEAR_R` at 1.2 m for the rig ring and scale it only above the 1 m rig ring's reach (S's ring: 1.475 m)? Path-clear and take-back zone c scale with the formation's reach. `ZONE_RISE_FAR_M` going from 0.9 to 1.125 changes Thu 29's take-back, so fly one 1 m twin take-back first.
  *Recommended: yes.*
- **Q6.** Twin timing: no command delay as a twin default in this push; the body-rate EMA only if round B and a no-harm check pass before the freeze; pair-swing's rounds run on the per-channel twin. OK?
  *Recommended: yes.*
- **Q7.** One approval for all the generated worlds and models: the 1 m attach family and QUAD4 models, `3915_L100`, R's 1 m 3969/gap135/165/180/push worlds, and S's R50 worlds plus drone models 4-7. The tip-hinge world is separate (Q14).
  *Recommended: yes.*
- **Q8.** `params.py` additions, with node defaults unchanged: `fig8_size` (0 means use `traj_radius`), `load_ixx`/`load_izz` (0 means from the mass), and `prebuild_sizes`.
  *Recommended: yes.*

### Before the claim runs

- **Q9.** One frozen tree for the R, U, C and S claims (S behind its switch). The carriers' filter is whatever the rig picks after Wed's A/B, typed in every claim config with all carriers listed. OK?
  *Recommended: yes.*
- **Q10.** Every 1 m figure-8 table at size 1.0, with the size-0.5 cells kept as history? T7t at 1 m on the 22 Oct clock-free, path-only ruler, keeping the carry card's values (25 mm, 6 deg, 0.95 ratio), with the 16 Oct metric shown beside it?
  *Recommended: yes.*
- **Q11.** A 1 m W4 ledger band for Thu 29's Nr hover releases: 3 twin runs, f = 1 and 2σ_rig from Wed's first 1 m hover (at least 8 s steady before LAND), frozen before Thu 08:00. This changes the band rule.
  *Recommended: yes.*
- **Q12.** R's scope: the 0.55 m items paired with rig flights, and the earlier attach-sequence figures, stay as labelled history. The 1 m tables replace the 0.55 m twin tables, with one appendix table comparing 0.55 m and 1 m. The b22 bands replace tab:bands1m, and F12b drops the 0.55 m rig points.
  *Recommended: yes.*
- **Q13.** The integral's gains (`int_k_xy`/`int_k_z`) were identified at 0.55 m and 45°. Re-identify them at 1 m?
  *Recommended: not in this push.* Report the change in meaning, and revisit if Wed's 1 m holds oscillate.
- **Q14.** Approve the tip-hinge twin world for pair-swing's round C (0.55 m)? Rig typing of `attach_z` about 0.05 and `cable_len` about 0.53 would come only after round C and a 1 m marker measurement, as its own rig A/B.
  *Recommended: approve the world; defer the rig typing.*
- **Q15.** A read-only Betaflight `diff all` on QUAD2/3/4, plus the blackbox status, on Wed's bench (no changes)?
  *Recommended: yes.*
- **Q16.** Wed 28 flies the pair A/B as carded, with pair-swing's timing and mode readouts added (no extra flights). C's reach term is only a reported column beside the frozen b22 bands.
  *Recommended: yes.*
- **Q17.** On Wed's bench, weigh one 1 m cable with its magnet and record the hanging lengths (for `rod_spread_m`). Change `drone_mass` only if the cable is more than 10 g heavier.
  *Recommended: yes.*
- **Q18.** Merge tejen-sync2 before Thu 29, and take his `magnet_drop_below_quad` from our typed QUAD4 record so there is one source? This is attach geometry, so it is your call.
  *Recommended: yes.*
- **Q19 (U).** U as designed:
  - the M1 layout;
  - the aim knob moves the twin's weld point with the aim, keeping radius 0.025 (a deviation from the plan's "weld_radius ≥ d");
  - an inward radial sweep plus one outward neighbour at 8 cm;
  - a newcomer–carrier distance under 0.30 m counts as failure;
  - a 20 cm extension, pre-registered;
  - the critic waived for the twin-only 'slot' arm, noted in the rows.
  *Recommended: yes.*
- **Q20 (S).** S's scope:
  - N_max is 8, with 9 only as an extra hover;
  - the claim is 8→4 with unannounced releases, plus one exploratory soft →2;
  - no return to 8 in this push (it needs `rejoin_departed`, which is chain work, out of scope like step 2b), so the thesis says "N→4 (→2)";
  - the ring is R 0.5 m (outer 0.53 m), 0.9 kg;
  - the proposed "not heavily worse" bars.
  *Recommended: yes.*
- **Q21.** Two 12-cpu lab slots for 1-2 evenings (`lab_batch --cpus` up to 12), yielding to anyone else?
  *Recommended: yes.*
- **Q22.** S's speedups as the default: in the twin after its smoke run and tests; on the rig after the reviewer, not before 4 Nov. `plan_res_every` stays 1.
  *Recommended: yes.*
- **Q23 (C).** Fly both network forms in table C: our port as it stands and the paper's form (+9 runs)? Fly the paper's form only if the lab is short.
  *Recommended: both.*
- **Q24.** C's headline metric is the clock-free path error plus a reach term at the line's ends, with lag and time-indexed RMSE beside it. The hover is scored from the hand-over, with the first 5 s reported separately. The arm is called "our port of Quan et al." and its deviations are listed.
  *Recommended: yes.*
- **Q25.** The twin network mode's values in `sim.yaml`: automatic hand-over, the velocity loop with `vel_ki` 0, `diss_ki_load` 1.0, `diss_elev_deg` 60, preview off, `net_twin_geometry` on. The tuned values are frozen with your word before the freeze; if they come later, they go in a params file so the tree doesn't change.
  *Recommended: yes.*
- **Q26.** C2 (an announced 4→3 detach for both arms, 12 runs) only after R, if the lab has room?
  *Recommended: yes, lowest priority.*
- **Q27.** Approve the delete list for laptop disk space (plan item 6) before the R and U batches?
  *Recommended: yes.*

---

## 5. Items needing the project's critic, or a card with frozen bars

### Critic

These are new control laws or estimators in a flight loop.
- **pair-swing F3:** a switch on the rig mocap node's body-rate filter, which is an estimator path on the rig. The critic looks at it before any run that could make it a rig candidate.
- **pair-swing F5:** a per-channel Smith predictor using the planned cable force. Critic before it flies; if it does not beat its baseline, record it and delete the code.
- **sim-C:** `damp_relative` (a change to the network's control law), `net_twin_geometry` (reference mapping and seeding at the measured pivots), and the hold carry-over. All need the critic before they fly in the twin; the as-ported smoke run may go first with them off. The retune protocol (search space, objective J, guards, time box) is pre-registered in the card.

### No critic, but gated

- **S's speedups:**
  - bitwise-identity tests, including pivot −0.04 and the |a| ≈ 0 fallback;
  - state-machine tests of the residual gate;
  - an offline replay of one rig flight giving identical node-0 references;
  - a twin smoke run;
  - the reviewer before it becomes the rig default.
- **1m-defaults' zone and path constants:** they change the rig's take-back and the departed drone's geometry. Fly one 1 m twin take-back run, with a falsifier, before Thu 29.
- **U's 'slot' arm:** the critic can be waived (twin-only, refused on the rig). The default 'measured' path is the rig's fold-in and must stay byte-identical, with a test that proves it.
- **pair-swing D3 (`w_vel_scale`):** it is a constant, so twin pass plus card bars.

### Cards with frozen bars before their runs

- **pair-swing:** `2026-10-23_pair_mode_timing`. Falsifiers for rounds A-C before Friday's lab batch; round-D bars before round-D runs.
- **U:** `catch_offset`, frozen after the E-runs and before the grid. The exploratory rows may only set the hold length and weld settings.
- **S:** `scalability`. Bench definition, the replay flight list chosen before looking, and the quality and closed-loop bars, all before the replays and twin arms.
- **C:** `paper_method_compare`, before any C twin run, including the network LAND and run-end rule.
- **R:** `refly_1m`. The S/A/C bars, the inventory and the must-not-say list before the claim runs. The 1 m W4 band is frozen before Thu 08:00.
- **Any twin-wide timing default** (from pair-swing) needs its own card and reviewer before the claim tree.
- **1m-defaults:** exploratory rows with falsifiers are enough; no card.

### Reviewers

One per table: U, S, C, and R (5 tables). Pair-swing's fix also needs one before it becomes a default.

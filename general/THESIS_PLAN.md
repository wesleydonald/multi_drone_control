# THESIS IMPLEMENTATION PLAN — v2

**Wesley Donald — cooperative attach/detach control for cable-suspended payload transport**
v2 written 2026-08-04 after a full codebase audit. Branch `attach-week9`.
Assumed submission **Fri 27 Nov 2026** (17 weeks).

Companion docs: `general/CLAUDE.md` (orientation) · `DISSIPATIVE_TRACKING_ISSUE.md` (open
tracking problem) · `general/CABLE_LOAD_CONTROLLER_PROGRESS.md` (planner history).

---

## 0. Decisions locked (2026-08-04)

| # | Decision | Choice |
|---|---|---|
| D1 | Novelty positioning | **Physical mid-flight attachment**, explicitly distinguished from Quan et al.'s controller-side "entry" (§1) |
| D2 | Tracking architecture | **Restructure the network path to command velocity** — now *confirmed* correct against the paper (§1.2) |
| D3 | Attach bar | **Ring reconfiguration required**; central lifter is the fallback, not the target |
| D4 | Hardware scope | **Full — attach flies on the rig** |
| D5 | Tejen's approach stack | **Keep it** as the integration path, **plus** a simple deterministic weld harness so reconfiguration can be tested independently while he is still developing (§7.2) |
| D6 | Failure policy | **Fleet-wide coordinated abort.** Failure→redistribution remains a *sim-only experiment*, never the safety policy (§5.3) |
| D7 | Log storage | `results/` at repo root, absolute path, **automatic external backup** (§4) |
| D8 | Visualisation | **One shared config module, identical in sim and real** (§6) |
| D9 | Sim telemetry | **Constant placeholder values** — enough to populate the panel (§6.3) |
| D10 | Baselines | **No third controller.** OCP vs dissipative + the architecture study; cite their Table 2 for the wider landscape (§11) |
| D11 | Working method | **Design note → your approval → implementation with readable tests** (§8) |

Resources: full-time (~35–40 h/wk), mocap **1–2 days/week**, 4 tethered quads + payload +
Tejen's magnet drone. **~25 lab days exist before the freeze — that is the binding
constraint, not developer time.**

---

## 1. Novelty positioning — read this first

### 1.1 What the Quan et al. paper actually claims

I read the paper (arXiv 2509.03563, 3 Sep 2025, Quan Quan et al., Beihang). Relevant text,
page 8:

> "…remains robust to both robot failure and dynamic reconfiguration, such as **the entry or
> exit** of individual flying robots during transport. … **When a new robot joins during
> transport**, the newly involved robot is connected to the payload and its neighbors,
> enabling the system to stabilize into a new equilibrium state."

Fig. 2 also contains an **"Unplug-and-Play"** outdoor experiment panel (four numbered
stages). So the claim "adding drones back has not been done" **cannot be made as it
stands**, and would be a serious exposure at examination.

What limits their claim, and where your contribution survives:

- Joining is **not** one of the three illustrated application cases (Fig. 1C: halfway
  failure, different payload capacity, different cable length).
- The abstract's experimental claims are **single-robot failure, disconnection events, 25%
  payload variation, 40% cable length uncertainty** — joining is absent.
- Their joining is a **controller-side event**: a robot *already connected* to the payload
  has its virtual node activated, and the network settles to a new equilibrium. There is no
  rendezvous with a moving payload, no physical connection event, no control-authority
  handover, and no moment-transmitting joint.

### 1.2 The reframed contribution (D1)

> **Physical mid-flight attachment**: a free-flying drone rendezvouses with a payload that is
> already being cooperatively carried along a trajectory, **physically attaches** to it via an
> electromagnet, control authority is **handed over** from an approach controller to the fleet
> controller at the weld, and the fleet then **reconfigures its load distribution** — under a
> rigid joint that transmits moments, not a tension-only cable — while the trajectory
> continues.

Sub-claims that are individually defensible and individually testable:

- **N1** — Rendezvous and physical attachment to a *moving, cooperatively-carried* payload.
- **N2** — Control-authority handover at the weld without loss of stability (the mux, the
  feedforward gate ramp, the reference discontinuity).
- **N3** — Reconfiguration under a **rigid, moment-transmitting** attachment, which the
  tension-only cable formulation of both reference papers does not cover. This is why the
  fixed-tripod geometry problem and the balanced-tension solve exist at all — they are
  *your* additions, forced by the physical joint.
- **N4** — A characterised **stability boundary** for where a newcomer can settle (the
  elevation curve), i.e. *when* reconfiguration is possible, not just that it sometimes is.

**Actions (W1):**
- [ ] Search for supplementary video/material for 2509.03563 and any follow-up publication by the group; check whether "Unplug-and-Play" shows a *physical* connection event or a controller-side one. Record the finding in the thesis background chapter either way.
- [ ] Write the positioning paragraph now, while it is fresh, into the background chapter draft. Do not leave it to W15.
- [ ] Frame E6 (attach experiments) around N1–N4 so each sub-claim has its own evidence.

### 1.3 Correction: the velocity-command reading is CONFIRMED (D2)

`DISSIPATIVE_TRACKING_ISSUE.md §7` flagged its own architecture reading as uncertain
("read from the extracted Fig. 3 block diagram; the equations did not extract cleanly").
I verified it directly. It is **Fig. 2**, not Fig. 3, and the pipeline is unambiguous:

```
LIDAR  -> Relative Localization -> p_j, v_j  ---+
Camera -> Load Localization     -> p_L, v_L  ---+--> Dissipative Controller --> v_d --> Auto Pilot --> ESC -> Motors
DGPS   -> Pos                   -> p_self    ---+                                          |
                                        v_self <----------------------------------------- +
```

**The dissipative controller's output is a velocity command straight to the autopilot.**
There is no MPC position loop between them. Your restructure (D2) is aimed at the right
target, and this is now evidence rather than inference. Fix the figure number in
`DISSIPATIVE_TRACKING_ISSUE.md §7`.

One consequence worth designing around: their autopilot *natively accepts velocity*
(PX4/ArduPilot-style). **Betaflight does not** — it accepts body rates + throttle. So
faithfully reproducing their architecture requires you to supply the velocity→attitude→rate
layer their autopilot provides internally. That layer is yours to design and is where the
sim-to-real differences will live (§5.2).

---

## 2. Codebase audit — findings and remediation

Twelve findings from the 2026-08-04 audit. Severity: **S1** = will lose data or break a
flight, **S2** = will cost days, **S3** = quality/risk.

| # | Sev | Finding | Evidence | Fix | When |
|---|---|---|---|---|---|
| F1 | **S1** | **237 MB of flight logs live inside `c_generated_code_quad_load/`** — a gitignored acados *build* directory deleted whenever you force a solver rebuild. No backup exists. | `.gitignore`; `du -sh` = 237 MB | §4 data architecture | **W1 day 1** |
| F2 | **S1** | **`cleanup.sh` is syntactically broken** — `bash -n` fails at line 32; a duplicated block was pasted into line 27. The script that exists to stop two planners fighting **cannot run at all**. It also never clears `/dev/shm/fastrtps_*`, which the project's own notes call essential. | `bash -n cleanup.sh` | Rewrite as `tools/clean_slate.sh`, called automatically by the experiment runner | **W1 day 1** |
| F3 | **S1** | **No geofence, no tilt abort, no velocity abort, no battery failsafe.** Only three safeguards exist: pose timeout, solver-failure disarm, manual ESTOP. | grep across all controllers | §5 failsafe architecture | **W1–W3, before any multi-drone flight** |
| F4 | **S1** | **A pose timeout disarms ONE drone mid-flight** on a mechanically coupled fleet, and nothing tells the others. | `controller_mpc.py:830` | Coordinated abort (D6, §5.3) | **W1–W3** |
| F5 | S2 | **Log location depends on each node's working directory** (`os.getcwd()`), and two nodes `os.chdir()` into the acados dir at import. Hence logs scattered across three trees. | `data_logger.py:12,56`; `controller_mpc.py:9`; `thrust_ratio_node.py:34` | Absolute `results/` root (§4) | W1 |
| F6 | S2 | **`ACADOS_DIR` is a hardcoded absolute path** (`/home/wesley/multi_drone_control/...`). Breaks portability and any second machine. | `controller_mpc.py:7` | Resolve from an env var with a package-relative default | W1 |
| F7 | S2 | **Sim publishes no `Telemetry`** — the RViz battery rows only ever populate on hardware, so the panel is untested until you are at the rig. | Only publisher is `elrs_interface.py:249` | §6.3 sim telemetry (D9) | W2 |
| F8 | S2 | **Two divergent RViz config builders.** The real one lacks the DETACH/ATTACH panel and the MPC-plan display. | `rviz_quad_load_launch.py` vs `real_io_launch.py` | §6 shared module (D8) | W2 |
| F9 | S2 | **Sim/real parameter divergence is implicit.** `real_dissipative_launch.py` omits the entire kT parameter block, so node defaults silently apply. `params.py` defaults (`CABLE_LEN=0.6`, `N_DRONES=3`) are stale versus the worlds actually flown (0.5). Geometry mismatch has already cost this project a week. | `params.py`; launch diff | §7 parameter profiles + startup validation | W1–W2 |
| F10 | S2 | **`yref_N[3:6]` is never set** — the terminal cost commands a **stop** at the end of every 2 s horizon while flying at 0.6 m/s. Not previously identified. | `acados.py:320-322` | Experiment T1 (§9.1) | W3 |
| F11 | S3 | **Redundancy**: 3 lineage copies of `acados.py`/`dynamics.py`; **2 divergent copies of `thrust_ratio_ukf.py`** (different md5); 2 copies of `dissipative_network.py`/`dissipative_node.py` (one stale, *still exposed as an entry point*); 3 ad-hoc plot scripts; `tejen/` duplicate tree. | md5sum; `controller_load_mpc/setup.py:25` | §10 cleanup | W2 |
| F12 | S3 | **`skip_steps = 3` is dead code** in `controller_quad_load` — assigned, never read. `DISSIPATIVE_TRACKING_ISSUE.md §4.3` lists it as a tracking-lag candidate; that entry is void. | `controller_mpc.py:433` | Delete; strike §4.3 | W1 |
| **F13** | **S1** | **`~/.bashrc:120` sources a DIFFERENT ROS workspace** — `~/thesis/install/setup.bash` (project `drone_cage_control`) — which installs packages overlapping this repo: `interfaces`, `utility_objects`, `drone_communication`, `drone_visualisation`, `controller_ukf`. **Every new terminal gets those first** unless this workspace is sourced afterwards. The dangerous one is `interfaces`: a different `ELRSCommand` / `MotionCaptureState` definition produces silent mismatches, not errors. The second is `utility_objects`, where edits appear to have no effect at all. Found 2026-08-04 when pytest imported the wrong module. | `python3 -c "import utility_objects"` in a fresh shell resolves to `~/thesis/...` | Workspace guard in `tools/clean_slate.sh` (done); **Wesley to decide on the `.bashrc` line** | W1 |

**F1–F4 are the week-one block.** Everything else can queue behind them.

### Progress (updated 2026-08-04)

| Finding | Status | Evidence |
|---|---|---|
| F1 logs in build dir | **DONE** | 268 MB / 682 runs moved to `results/legacy/`; counts, sizes and checksums verified identical; `tools/backup_results.sh` written, run (709 files), and its restore drill passes byte-identical |
| F2 broken cleanup.sh | **DONE** | `tools/clean_slate.sh`: passes `bash -n`, clears `/dev/shm/fastrtps_*`, exits 1 when not clean. Tested against a decoy stale node — detected (exit 1) and killed. `cleanup.sh` is now a forwarding shim |
| F5 CWD-relative logs | **DONE** | `utility_objects/run_context.py`; `base_dir=` added to `DataLogger`/`run_log_dir` as an **additive** change — legacy callers verified byte-identical behaviour |
| F6 hardcoded ACADOS_DIR | **DONE** | Resolved via `acados_dir('quad_load')` from the workspace root / `MDC_ACADOS_ROOT`; verified from the installed package |
| F12 dead `skip_steps` | **DONE** | Deleted; `DISSIPATIVE_TRACKING_ISSUE.md §4.3` struck, T1 added as candidate 1, §7 corrected to Fig. 2 and its caveat discharged |
| F11 redundancy | **DONE** | Deleted 718 lines of superseded dissipative controller from `controller_load_mpc` + its entry point (a clean rebuild was needed — the stale install kept exposing `dissipative` and a `load_state` whose source no longer exists). Root scripts tidied into `tools/`. Offline gate A–K still ALL PASS. **The `thrust_ratio_ukf` "duplication" was a wrong call and was withdrawn — see §10.** |
| F13 wrong workspace | **DONE** | `~/.bashrc:120` removed (backup `~/.bashrc.bak-20260804`); verified in a clean login shell that nothing resolves to `~/thesis` and that sourcing this workspace resolves correctly. `tools/clean_slate.sh` also guards it, so a wrong overlay is caught before every launch |
| F3, F4 safety | **DONE** | `utility_objects/safety.py` (pure, ROS-free) + **20 unit tests**, wired into the tracker; emergency disarm now broadcasts `/fleet/abort` and direct-publishes `ELRSCommand` **synchronously, ~1 ms**, before any service call; pre-arm interlock via an optional `safety_preflight_block()` hook. Live-verified on the running manager. Offline gate A–K still ALL PASS |
| F7, F8 RViz + telemetry | **DONE** | `controller_quad_load/rviz_config.py` is now the single builder; ~300 lines of duplicated config removed from the two launches. Verified the only remaining sim-vs-real difference is the DETACH/ATTACH panel flags. **Regression, found by Wesley and fixed same day:** the refactor dropped the top-level `Window Geometry:` key, so its children fell inside `Visualization Manager:` whose `Displays:` key they then duplicated — YAML keeps the last, so the entire display list was replaced by `{collapsed: false}` and RViz opened empty while every process logged healthy. Now byte-identical to pre-refactor, with 12 structural tests (`test_rviz_config.py`) including a duplicate-key-rejecting YAML loader that reproduces the failure. `sim_telemetry` publishes placeholder Telemetry so the battery rows populate in sim; live-tested that `ros2 param set voltage_drone_1 13.2` drives one drone low without touching the others, so the warning path is checkable off-hardware. Hardware launch deliberately does not start it (real telemetry comes from ELRS) |
| F9 parameters | **DONE** | `tools/check_geometry.py` (world SDF vs controller config) and `tools/param_diff.py` (sim vs real). **Design changed from the plan:** no YAML profiles — the launch defaults and the world SDF are already the two authorities, and a third file describing them would be one more thing to drift, which is the very failure F9 exists to close. Found and fixed a stale `CABLE_LEN = 0.6` in `params.py` (every world is 0.5; it was `three_soft.sdf`'s value and that world no longer exists), so any node run outside a launch had a 20% cable-length error. Also renamed `three_attach_launch.py`'s `attach_radius` → `weld_radius`: it is the magnet's weld-proximity threshold (0.15), not the payload attach ring (0.08), and the name collision had already caused confusion. |
| F10 terminal velocity ref | **DONE** | `terminal_vel_ref` parameter + launch arg on all six tracker launches, **defaulting to the historical behaviour** so nothing changes until measured — T1 is now a one-flag experiment. 7 tests pin both modes, prove nothing but `yref_N[3:6]` differs, and prove hover is bit-identical either way. Verified at runtime that the flag reaches all three trackers (`T1 ACTIVE` in the log). |

**All 13 findings closed.** Gate is now 5 steps and green: workspace · 43 unit tests ·
25 module imports · world-geometry agreement · dissipative A–K.

**Geofence removed at Wesley's direction (2026-08-04)** after being implemented: he is
at the cage with a kill switch, so out-of-bounds is a human's job. Kept are the checks a
human cannot cover — reference staleness, tilt, speed. Recorded in the code and in
`docs/design/fleet_safety.md` as a decision, not an oversight.

Housekeeping done alongside: `log/` 642 MB → 12 MB (291 stale colcon build-log dirs);
93 of this project's own runs moved out of `logs/` into `results/legacy/` (including 44
`join_planner` attach-development runs); the remaining 87 MB of inherited tracked data
in `logs/` removed at Wesley's request; shared-repo governance language stripped from
`README.md` and `CLAUDE.md` now that the repo is an isolated fork; README rewritten to
describe this project.

`plot_run.py` was updated to search the new location, the migrated legacy copy, and
the old build path in that order, so analysis works across the move.

Backup policy settled — see §4.3. `results_archive/` + GitHub is the off-machine
copy; raw `results/` is single-copy by decision, with a local same-disk snapshot at
`~/thesis_backups/multi_drone_control` against accidental deletion.

---

## 3. Architecture principles for everything new

These exist so the code stays scalable and quick to work in, and so you can defend it.

1. **One source of truth per concept.** Geometry lives in `geometry.py`, parameters in a
   profile, metrics in `tools/metrics.py`. If a number appears in two files, one of them is
   wrong — this is exactly how `thrust_quad_c` went stale at 203 and how `CABLE_LEN`
   diverged.
2. **Pure core, thin ROS shell.** Every new control component is a plain Python class with
   no ROS imports, wrapped by a thin node. This is what makes the SIL bench, the unit tests
   and the offline harness possible at all. `PlannerConfig` in `params.py` is the pattern
   already in the repo — extend it, don't invent a second style.
3. **New behaviour behind a parameter that defaults to the old behaviour.** This convention
   is why the verified detach configuration survived the entire attach effort. Keep it.
4. **The reference wire format is frozen** (12 fields/node). Both reference generators stay
   interchangeable — every head-to-head experiment depends on it.
5. **Fail loudly at startup, never mid-flight.** Geometry and parameter mismatches must
   raise at launch, not manifest as a crash at 1.5 m.
6. **Every artefact self-describes.** A result directory that cannot tell you what produced
   it is not evidence.

---

## 4. Data architecture — logs, runs, backups (D7)

**Problem being solved:** 397 MB of flight data in three CWD-dependent directories, 237 MB
of it inside a build directory that gets deleted, with no backup.

### 4.1 Layout

```
results/                                   # absolute path, resolved from repo root
  2026-08-11/
    R0031_sim_dissipative_circle_n3/
      manifest.json          # git SHA, dirty diff hash, world hash, run_id, sim|real,
                             #   operator, start/end (wall + sim), exit reason
      params/                # ros2 param dump PER NODE, read back after startup
        controller_0.yaml  planner.yaml  dissipative.yaml ...
      logs/
        drone_0.csv  drone_1.csv  payload.csv  events.csv
      plots/                 # auto-generated on run completion (§9.4)
      metrics.json           # computed by tools/metrics.py, the only source of numbers
      console.log
  2026-08-11/R0032_real_.../
results_archive/                            # tracked; frozen datasets behind thesis figures
```

- **`run_id`** is a monotonic counter (`R0031`) plus a descriptive slug. It appears in the
  directory name, in `manifest.json`, in every plot title, and in the thesis figure caption.
  You can always get from a figure back to the raw data.
- **`sim` vs `real`** is in the run_id and the manifest — never inferable only from context.
- **`events.csv`** logs every fleet command, detach, weld, abort and mode change with both
  clocks. Event-relative metrics depend on this.

### 4.2 Implementation

- `utility_objects/run_context.py`: resolves the results root **absolutely** (env
  `MDC_RESULTS_ROOT`, default `<repo>/results`), allocates the `run_id`, writes the manifest,
  and hands out the directory. `DataLogger` takes a directory instead of computing one from
  `os.getcwd()` — fixes F5 without touching any node's logging calls.
- Parameters are captured by **reading back `ros2 param dump` after startup**, not by
  recording launch arguments. Launch args have silently failed to plumb through before;
  reading back is the only version that cannot lie.
- Migrate the existing 397 MB into `results/legacy/` once, preserving timestamps.

### 4.3 Backup — the policy Wesley chose (2026-08-04)

**Decision: `results_archive/` is pushed to GitHub; raw `results/` is single-copy and
not backed up off-machine.** This was chosen knowingly after the alternatives
(external drive, rclone to university cloud, network share) were laid out — none of
which existed on the machine at the time.

| Tree | Tracked | Off-machine | Protection |
|---|---|---|---|
| `results/` | no | **no** | none beyond the local snapshot below |
| `~/thesis_backups/multi_drone_control` | n/a | no (same disk) | accidental deletion / `git clean -xdf` only |
| `results_archive/` | **yes** | **yes (GitHub)** | full |

**What this makes important:** promotion is the moment data becomes safe. Anything
behind a figure, a table, a threshold or a claim gets archived and pushed the same
day — and **hardware sessions get archived in full**, because a lab day cannot be
regenerated from a laptop.

- `tools/archive_run.sh <run_dir> "<reason>"` — copies into `results_archive/`, records
  provenance in `ARCHIVED.json` (source path, git SHA, dirty-tree count, file count,
  SHA-256 of contents), and **refuses to overwrite** an existing archive so figures stay
  traceable to exact bytes. `--list` shows what is archived and why.
- `tools/backup_results.sh [dest...]` — rsync with `--checksum`; refuses to exit 0 with no
  destination; **warns when the destination is on the same filesystem** so a same-disk copy
  is never mistaken for a backup; `--verify` runs a restore drill.
- A run at ~8 KB–0.5 MB means plain git handles the archive comfortably; git-LFS is not
  needed at this scale.
- **Revisit if an external drive appears.** The one-line change is
  `export MDC_BACKUP_DEST=...` — the mechanism is already built and tested.

---

## 5. Failsafe architecture (D6)

**Policy: fleet-wide coordinated abort.** Any qualifying fault triggers a synchronised
response for the whole fleet, because the drones are mechanically coupled — one drone
disarming alone drops its share of the load onto the others without warning (F4).

### 5.1 Layers, outermost first

| Layer | Trigger | Response | Where |
|---|---|---|---|
| L0 **Pilot** | Human judgement | Physical TX kill / ESTOP button | RViz panel + hardware TX |
| L1 **Fleet abort** | Any drone: pose timeout, sustained solver failure, geofence breach, tilt/velocity limit, low battery | **Coordinated abort**: fleet descends together at `abort_vel`, then disarms on ground contact. Broadcast on `/fleet/abort` with a reason code | New `fleet_safety` node |
| L2 **Per-drone limits** | Throttle saturation, attitude error, reference staleness | Warn + log; escalates to L1 if sustained | `controller_mpc` |
| L3 **Solver** | acados failure | Hold last good command; escalate at `MAX_CONSEC_SOLVE_FAILS` | exists (`controller_mpc.py:968`) |

**Design note required before implementation** (D11): the abort descent is itself a
controlled manoeuvre — a fleet that aborts badly is worse than one that does not abort.
Reuse the existing LAND path (already handles attached *and* detached drones, and the
network-phase height ramp) rather than inventing a new descent.

### 5.2 New checks to add

- **Geofence**: an axis-aligned box plus a ceiling, in mocap coordinates, per venue. Set it
  from the actual flyable volume. This directly addresses the documented "shoots straight up,
  out of the map" failure — in the lab that is a wall strike.
- **Tilt limit**: abort above a configured payload or drone tilt (the attach runaway reached
  90°+ in sim; on hardware that is a crash).
- **Velocity limit**: abort above a speed that has no business occurring in a 5 m volume.
- **Battery**: on hardware, abort on low voltage. With placeholder sim telemetry (D9) the
  *path* is exercised in sim even though the values are constant — set the placeholder below
  the threshold in a dedicated test to prove the abort fires.
- **Reference staleness**: if the planner stops publishing, hold then abort — currently a
  tracker will happily fly a stale reference.

### 5.3 Failure response as a sim-only experiment

Automatic failure→dissipative redistribution is **not** the safety policy (D6). But it is
squarely on the thesis's topic — Quan et al.'s entire motivation is resilience to mid-flight
failure, and your system currently only ever redistributes on an *operator-commanded*
detach. So it is planned as **experiment E10-F, sim only**: inject a drone failure, let the
network redistribute, and measure. Clearly separated in the thesis as a simulation study,
with the hardware policy stated as coordinated abort and the reason given.

### 5.4 Pre-flight verification (every session, logged)

A `tools/preflight.py` checklist that must pass before arming, with results appended to the
session's manifest:

1. Every drone's mocap rigid body present, fresh, and mapped to the right ID.
2. ELRS link up per drone; **disarm verified per drone** on the ground.
3. Geofence loaded and matching the venue profile.
4. Parameter profile validated against physical truth (§7.2).
5. Battery voltages above threshold; kT seed per airframe recorded.
6. Abort path tested once, on the ground, with props off.

---

## 6. Visualisation and operator interface (D8, D9)

**One shared config module, used identically by sim and real** — `drone_visualisation/rviz_config.py`,
consumed by `rviz_quad_load_launch.py` and `real_io_launch.py`. What you rehearse in sim is
what you get at the rig, and the real path cannot silently rot (F8).

### 6.1 What is displayed, by case

| Element | Standard carry | Removal (detach) | Adding (attach) |
|---|---|---|---|
| Per-drone airframe mesh, fixed colour per ID | ✓ | ✓ | ✓ |
| **Role colouring** | all "attached" | departing drone switches to "detached", survivors stay | newcomer "approaching" → "welded" → "attached" |
| **Cables rendered** (drone→attach point) | ✓ | released cable follows the departing drone | newcomer's link appears at weld |
| Payload box + **desired vs actual path** (two colours) | ✓ | ✓ | ✓ |
| Per-drone reference marker + error line to actual | ✓ | ✓ | ✓ — this is how you see the newcomer being dragged |
| MPC plan / network horizon | ✓ | ✓ | ✓ |
| **Event banner** ("DETACH d1 @ t=62.3") | — | ✓ | ✓ |
| **Load tilt readout** (deg, colour-coded) | ✓ | ✓ | ✓ — the attach failure signature |
| Panel: ARM/TAKEOFF/LAND/DETACH/ATTACH + per-drone armed & battery | ✓ | ✓ | ✓ |
| **Geofence box** | ✓ | ✓ | ✓ |

Two additions carry most of the diagnostic value and neither exists today: **role colouring**
(you can see at a glance who is carrying) and the **reference→actual error line** (you can
see a drone being physically dragged off its reference — the exact attach signature that
took weeks to identify from logs).

### 6.2 Scaling

Config is generated for exactly `num_drones` (the existing generators already do this —
keep it, unify it). Colours from one shared palette keyed by drone ID, used by RViz paths,
the URDF mesh, **and the plotting pipeline**, so drone 2 is the same colour in RViz and in
every thesis figure.

### 6.3 Sim telemetry (D9)

New `simulation_communication/sim_telemetry.py`: publishes `/drone_i/telemetry` with
**constant placeholder values** (nominal voltage, RSSI, mode) for each drone, so the panel
populates identically in sim and real (F7). Parameterised so a test can drive one drone's
voltage below threshold to prove the battery abort path fires (§5.2). ~40 lines.

---

## 7. Parameter management (F9)

**Problem:** you cannot currently tell what differs between sim and real without reading two
launch files side by side, and the node defaults are stale relative to every world actually
flown.

### 7.1 Profiles

```
config/
  common.yaml          # physics and geometry shared by every configuration
  sim.yaml             # thrust_ratio 31, takeoff_thrust_ratio 30, load_mass 0.4 ...
  real.yaml            # thrust_ratio 24, takeoff_thrust_ratio 0,  load_mass 0.1 ...
  worlds/three_rigid_ground.yaml   # cable_len 0.5, attach_radius 0.08, n 3, payload_rest_z ...
  rig/unsw_mocap.yaml              # geofence, mocap IDs, ELRS devices, per-airframe kT
```

Launches load `common + {sim|real} + world/rig`, in that order. **The stale module-level
defaults in `params.py` are then deleted** — a missing parameter should fail loudly, not
silently supply `CABLE_LEN=0.6` for a world with 0.5 m cables.

### 7.2 Validation at startup, and against physical truth

A `config_validator` that runs before arming and **refuses to launch** on:

- Any parameter absent from the profile that has no defensible default.
- Geometry inconsistent with the world SDF, in sim: parse the SDF and assert `cable_len`,
  `attach_radius`, `load_mass`, drone count and spawn ring actually match. This single check
  would have prevented the documented week lost to `CABLE_LEN` 0.6 vs 1.0.
- On hardware: geometry inconsistent with the **measured rig values** recorded in
  `config/rig/*.yaml`, which you physically measure and update at the start of each campaign.

Plus `tools/param_diff.py sim real` — prints exactly which parameters differ between two
profiles, and which exist in one but not the other. That is the direct answer to "how do I
know what to change between sim and the real world", and it would have surfaced the missing
kT block in `real_dissipative_launch.py` immediately.

---

## 8. How you stay the author (D11)

For every new module, in order:

1. **Design note** (`docs/design/<module>.md`, 1–2 pages): the problem, the equations, why
   this structure, what the failure modes are, and what would falsify it. **You approve it
   before any code is written.**
2. **Implementation** with unit tests written to be *read* — each test names the property it
   protects in plain language.
3. **Walkthrough** appended to a one-page-per-package doc: what each file does, in your
   words, with the two or three subtleties that matter.

Plus: for anything that will be defended in an exam — the velocity loop, the tension solve,
the hand-out, the reconfiguration logic — the design note includes a **derivation you can
reproduce on a whiteboard**. If a derivation can't be written that way, the design is too
complicated and should be simplified before it is built.

**Inherited code (D5).** Tejen's stack stays as the integration path. To make it
understandable without rewriting it:
- Write a walkthrough covering **only the join path you actually exercise** — the state
  machine states, the reference builders, and the parameters the launch sets. The pickup/drop,
  obstacle-avoidance and blocked-fallback subsystems are documented as "present, disabled,
  not exercised" with the parameter that disables them.
- Add a small **integration contract test**: given a payload pose stream, the planner must
  produce a reference that converges to the weld point within a bound. That way, if his code
  changes underneath you, you find out from a test rather than from a crash at the rig.

---

## 9. Test infrastructure (W1–W2)

The project's dominant failure mode is iterating blind in Gazebo. Two weeks here pays for
itself many times over and is what makes the ablation experiments affordable at all.

### 9.1 Batch experiment runner — `tools/run_experiment.py`

```bash
python3 tools/run_experiment.py configs/E1_ocp_circle_n3.yaml --repeats 5
```

Config: world, launch, args, scripted **sim-time** events, duration, success criteria.
Requirements, each earned by a past failure:

- **Headless** (`gz sim -s -r`) so batches run overnight.
- **Sim-time gated**, never wall-clock — unequal sweep durations have invalidated comparisons.
- **Clean slate every run**: the rewritten `tools/clean_slate.sh` (F2), including
  `rm -f /dev/shm/fastrtps_*`, then verify no `ros2 node list` residue.
- **Manifest + read-back parameter dump** (§4.2).
- **Auto-plot and auto-metrics on completion** (§9.4), so a finished run is already readable.
- **Fails loudly** on pose timeout, solver disarm, criteria violation, or a duration
  mismatch >10% across repeats.

### 9.2 Metrics library — `tools/metrics.py`

Every number in the thesis comes from here and nowhere else. One function per metric, each
unit-tested against a synthetic signal with a known answer.

`payload_rmse` · `stage_split` (desired → reference centroid → actual centroid → payload,
radius ratio and phase lag per stage) · `radius_ratio` · `phase_lag_s` · `load_tilt_deg`
(peak + settled) · `tension_share` (spread and per-drone fraction) · `event_settling_time` ·
`event_peak_error` · `control_effort` (mean throttle, % saturated) · `solver_health` ·
`cable_accel_gap` (‖measured − modelled‖ — the attach runaway signature).

Conventions fixed now: **steady window** = event + 5 s → end of sweep; **event time** from
`events.csv`; **repeats** 5 sim / ≥3 real; report **median and full range**, never mean alone.

### 9.3 Software-in-the-loop bench — `tools/sil_bench.py` ← highest-leverage item

Closes the loop around the **real controller nodes** against a fast numerical plant, over
real ROS topics, with no Gazebo.

```
mini_plant (extended) --/drone_i/motion_capture_state--> real tracker nodes
                      --/payload/motion_capture_state--> real dissipative node
        ^                                                        |
        +---------------- /drone_i/ELRSCommand <-----------------+
```

Why not the existing harness: `verify_dissipative` drives a **PD proxy**, and it is
documented repeatedly that this cannot reproduce the Gazebo failures — because the real
tracker (a) has **no integrator**, (b) **caps cable feedforward** at `CABLE_ACCEL_CAP = 6.0`
while measurements reach 15–20 during the attach transit, and (c) has solver dynamics. All
three are the mechanisms that produce the runaway.

**Acceptance test for the bench itself: it must reproduce the documented attach runaway**
(2026-07-28 gap-centre config, `attach_elev_deg:=45`) — drone 3's error growing, `|aCm|`
climbing past 15 m/s². If it settles cleanly like the PD harness, the bench is not good
enough yet. **Do not skip this test** — it is the entire justification for the investment.

> **BLOCKED ON A STALE REFERENCE (2026-08-05).** The 2026-07-28 records were flown with
> `motorConstant 1.42e-06` → plant gain `c = 4·mc·maxRotVel²/mass = 203 m/s²`, hover
> throttle 0.220. Commit `f0f7b9f` (2026-08-03, "Changed the motorconstant to match real
> world") set `0.62e-06` → `c = 88.6`, hover 0.333: **2.29× less thrust authority**. The
> bench models today's 88.6, so this criterion currently asks it to reproduce an aircraft
> the repo no longer contains, and neither agreement nor disagreement would mean anything.
> Git clears the earlier ball-joint suspicion: `three_attach.sdf` and
> `x3_drone3_magnet.sdf` were created together in `42efaaf` with `arm_to_magnet_tip`
> already a ball joint, and the magnet model changed exactly once afterwards — `f0f7b9f`,
> motorConstant only.
>
> **Resolution: re-fly 45° and 65° in Gazebo against the current SDF and replace the
> recorded numbers.** The criterion itself stands; only its reference data is stale. Do
> not weaken the bars to make the pair pass.
>
> **DONE 2026-08-05 — and the criterion does not survive it.** Both configs re-flown
> headless, 5 repeats each (`tools/run_experiment.py`, R0034–R0043). The 45° half holds:
> welds 5/5, diverges 5/5, drone-3 error median **1.11 m** vs the recorded 1.06. The 65°
> half does **not**: it diverges too (aborts 4/5, drone-3 error median **1.17 m** vs the
> recorded 0.01–0.06 m), so the two elevations are now indistinguishable and the
> DISCRIMINATION claim is simply not true of the current simulator. No bench faithful to
> Gazebo can reproduce a difference that Gazebo no longer shows.
>
> The SIL bench says the same thing under the same controller config — both elevations
> capsize with a fixed kT — so bench and Gazebo now AGREE, which is evidence *for* the
> bench, not against it. Prime suspect is the fixed thrust ratio against Gazebo's
> quadratic motor model (§7.1, `controller_mpc._effective_kT`): restoring the quadratic
> inversion makes the bench discriminate again. **Resolve the kT/plant mismatch, then
> re-fly and re-derive this criterion.** Part 3's gate should be read against that, not
> against the 2026-07-28 numbers.

### 9.4 Analysis and plotting pipeline — `tools/plot_run.py`, `tools/compare_runs.py`

Direct answer to "how do I efficiently see how close the drones were to the correct path".
Today: one ad-hoc 3D plotter whose docstring promises more than the code delivers, and no
comparison capability.

**Auto-generated on every run** (no command to remember, written into `plots/`):
1. **XY path**: desired vs actual payload, plus per-drone reference vs actual, event markers.
2. **Error vs time**: ‖e_p‖ per drone and payload, with event lines and the steady window shaded.
3. **Per-axis x/y/z** desired vs actual, payload and drones.
4. **Health strip**: throttle (with saturation band), tilt, `|aC|` vs `|aCm|`, solver status.
5. **Formation view**: per-drone tension share and azimuth over time — how reconfiguration
   actually looks.
6. **Stage split** (moving trajectories): the four-stage radius/phase table as a figure.

**`tools/compare_runs.py R0031 R0032 ...`** overlays N runs on the same axes with a metric
table — the ablation and head-to-head workhorse. **`tools/thesis_figures.py`** regenerates
every thesis figure from `results_archive/` in one command, so a late data fix propagates
everywhere instead of leaving a stale PNG in the document.

Shared style module: one colour per drone ID (matching RViz, §6.2), consistent axes and
fonts, vector output. Set this up once in W2 and every figure for the next four months is
publication-ready by default.

> **BUILT 2026-08-05.** `tools/plot_style.py`, `tools/plot_run.py`,
> `tools/compare_runs.py`, `tools/thesis_figures.py` (+ `configs/thesis_figures.yaml`),
> tested in `tools/test/test_plot_pipeline.py` against a synthetic circle with a known
> radius deficit and lag. Auto-plotting is wired into both harnesses on completion, and
> `run_experiment.py`'s hand-rolled metrics were replaced by `metrics.summarise_run` so
> §9.2's "one source of numbers" is now literally true.
>
> Three things the figures do NOT show, because nothing logs them — stated here rather
> than quietly dropped from the list above:
> - **`|aC|` (modelled cable accel)** — only `|aCm|` is logged, so figure 4 plots the
>   measured trace alone and the `cable_accel_gap` metric has no input from a run
>   directory. Reinstating it needs a column in both loggers.
> - **acados solver status** — not logged by either harness; figure 4's fourth panel
>   shows armed state instead and says so on the axes.
> - **cable tension / elevation / `|aCm|` in Gazebo** — internal to the tracker, NaN in
>   `run.csv`. Those panels are bench-only and annotate themselves.
>
> The palette is IMPORTED from `rviz_config.py`, not copied, and a test locks the two
> together — that is what makes the §6.2 promise hold over four months.

### 9.5 Unit tests and the gate

`pytest` for the ROS-free core: `geometry.attach_points` and `azimuth_slot_assignment`
(**regression-lock the 2026-08-03 load-frame yaw fix across payload yaw 0–360°**);
`_solve_tensions` (wrench residual ≈ 0, tensions ≥ 0, equal-share reduction, newcomer share
> 10%); `DissipativeNetwork.step` invariants (detach→attach returns to equilibrium,
`horizon_references` side-effect free); `load_trajectory` continuity; wire-format round-trip;
`_tilt_quat_from_accel`; `betaflight_rates` inverse round-trip.

`tools/gate.sh` — pre-push: `pytest` → `verify_dissipative` (A–K) → SIL smoke suite →
2 headless Gazebo runs. Thresholds in `configs/gate_thresholds.yaml`, set from the W2
baseline, not guessed.

> **BUILT 2026-08-05, minus the Gazebo stage.** Stage 6 is the SIL smoke suite
> (`carry_hover_n3` through the real launch, then `tools/check_thresholds.py --profile
> sil_smoke`), and `tools/hooks/pre-push` + `tools/install_hooks.sh` wire the full gate
> to `git push`. Whole gate ~2.5 min. Thresholds come from four measured repeats
> (R0047–R0050), recorded per check in the YAML so a later tightening can see what it
> is tightening against.
>
> **The 2 headless Gazebo runs are NOT in the gate.** At ~235 s each they would put the
> gate at ~10 minutes, and a gate that slow stops being run — which costs more than it
> catches. The mechanism is ready (`check_thresholds.py` reads a Gazebo run's
> metrics.json exactly the same way); what is missing is a Gazebo scenario whose bars
> are meaningful, and on the current plant the attach scenarios all diverge. Revisit
> once the kT/plant question is settled and there is a Gazebo run that passing means
> something.

### Part 0 exit criteria (end W2)

- [ ] One command runs a headless experiment and writes a self-describing result directory
- [ ] Metrics reproduce the four `DISSIPATIVE_TRACKING_ISSUE.md §2` stage numbers to within 2%
- [ ] **SIL bench reproduces the attach runaway**
- [x] Plots auto-generate; `compare_runs` works across two archived runs — *works
      across a bench run and a Gazebo run too (R0020 vs R0034); the runs it has been
      exercised on are in `results/`, not yet promoted to `results_archive/`*
- [x] Gate green and wired to a pre-push hook — *6 stages, ~2.5 min, green as of
      2026-08-05; the 2 headless Gazebo runs are deliberately left out (see §9.5)*
- [ ] F1–F6, F12 closed; backup restore drill passed once

---

## 10. Redundancy cleanup (F11)

Do this in W2, once, so nothing later is built on a duplicate.

| Item | Action |
|---|---|
| `controller_load_mpc/dissipative_node.py` + `dissipative_network.py` (stale since 2026-07-22, superseded by `controller_dissipative/`) | **Delete**, and remove the `dissipative` entry point at `controller_load_mpc/setup.py:25`. It is currently possible to launch the wrong, dead controller. |
| Two `thrust_ratio_ukf.py` (two packages) | **RESOLVED (2026-08-05): the `controller_quad_load` copy is DELETED**, together with `thrust_ratio_node.py` and the `kt_estimator` entry point — kT is now a fixed launch parameter (supervisor-approved). Only `controller_mpc_payload`'s three-backend original remains, and it now defaults OFF (`approach_kt_ukf:=false`). The earlier note stands as history: these were never duplicates (417 vs 277 lines, 669 diff lines) and unifying them would have broken one; the hazard was the shared filename + class name `ThrustRatioUKF` and the deadlock if both were imported into one process. Deletion removed the hazard rather than documenting it. |
| 3 lineage copies of `acados.py` / `dynamics.py` (`controller_ukf` → `controller_mpc_payload` → `controller_quad_load`) | **Leave `controller_ukf` alone** (Mitchell's). Document the lineage and the intentional differences in the package walkthrough. Do not unify — divergence here is deliberate and unification would risk the working trackers. |
| `plot_run.py`, `plot_xy.py`, `plot_takeoff.py` at repo root | Superseded by §9.4. Move to `tools/legacy/` rather than deleting, in case a figure depends on one. |
| `tejen/` duplicate tree (COLCON_IGNORE'd) | Keep for reference until integration completes (D5); note in `CLAUDE.md` that `src/` is authoritative. |
| `skip_steps = 3` (F12) | Delete the line; strike `DISSIPATIVE_TRACKING_ISSUE.md §4.3`. |
| `solver_probe.py`, `ref_dump.py`, `rate_probe.py` | Keep — genuine diagnostics. Move to `tools/`. |

---

## 11. Alternative methods considered (and why not)

You asked what else could work. Quan et al.'s own Table 2 benchmarks four families; here is
each against **your** problem, which is specifically *reconfiguration under a rigid weld*.

| Method | What it would buy | Why not (for this thesis) |
|---|---|---|
| **Admittance / impedance control** | Responds directly to measured external force — which is exactly what a weld imposes. The closest competitor in their Table 2, and arguably the most natural fit for attach. | Needs a leader, and the paper notes it fails with slack/relaxed cables. Weeks of work that come out of the attach effort. **The good idea inside it is portable without adopting it**: your velocity loop's integral term plus a measured-cable-force feedforward *is* an admittance-like response, and it is already planned (§12.2, A2.1/A2.2). |
| **Distributed MPC (ADMM/consensus)** | Keeps optimality and constraints while decentralising; would handle the fixed-tripod geometry natively. | Solve times and consensus iterations at 50 Hz over 4 agents are a research project by themselves. Not in 17 weeks. |
| **Geometric control on SE(3)** (Lee et al. lineage) | Rigorous stability proofs for a payload with cables; well established. | Assumes tension-only cables and known geometry — it does not model a moment-transmitting weld, which is precisely your novelty (N3). Would need extension, i.e. the same work you are already doing. |
| **Null-space / behavioural formation control** | Cheap, scales, natural priority between "hold the load" and "space out". | No load-sharing guarantee; you would be re-deriving tension distribution anyway. |
| **Control barrier functions for the approach** | Formal safety guarantees during rendezvous — genuinely attractive for the attach phase. | Tejen's planner already has obstacle avoidance; adding CBFs duplicates it. **Worth one paragraph in future work**, especially for a moving-payload rendezvous. |
| **Learning-based residual / RL** | Could absorb the modelled-vs-actual cable mismatch that causes the runaway. | Sim-to-real transfer, data collection and defensibility all fight a 17-week schedule. |

**Recommendation (D10): implement none of them.** Your comparison set — centralized OCP vs
decentralized dissipative, and MPC-tracked vs velocity-commanded — is already two genuine
head-to-heads. Discuss the table above in the background chapter to show the landscape was
considered; that is what an examiner wants to see. Revisit admittance control only if the
velocity loop fails at the W6 gate.

---

## 12. The work

### 12.1 Part A — tracking restructure (W3–W6)

**Goal:** kill the 0.589 s tracker-stage lag, and produce a measured head-to-head between
"network → position setpoint → MPC" and "network → velocity command → attitude/rate loop"
(the paper's architecture, now confirmed in §1.3).

**Framing that de-risks it:** the restructure is an *experiment*, not a bet. Either result is
a contribution — the paper asserts its architecture but never compares it against an
MPC-tracked alternative.

**Stage T — cheap candidates first (W3, ≤4 days, timeboxed).** Each measured separately
against the baseline; never bundled.

| ID | Change | File | Prediction |
|---|---|---|---|
| **T1** | Set terminal velocity reference: `yref_N[3:6] = ref_vel[N_horizon]` | `acados.py:320` | **Leading suspect.** Removes a commanded stop at the end of every horizon; phase lag falls, radius ratio rises |
| **T2** | Interpolate reference by **message age** rather than using node 0 verbatim | `controller_mpc.py` | Removes up to 0.1 s of pure staleness (~1/6 of the lag) |
| **T3** | Velocity cost weight 2.0 → {8, 20, 40} | `acados.py:88` | Radius ratio rises; watch noise sensitivity |
| **T4** | `CABLE_ACCEL_CAP` 6.0 → {10, 15} | `controller_mpc.py:78` | Little effect on steady tracking; large effect on attach transients (also Part B) |

**Stage V — the velocity architecture (W4–W5).**

```
reference(i) -> (p_ref, v_ref, a_ff, a_cable)          [wire format UNCHANGED]
a_sp   = a_ff + Kv*(v_ref - v) + Ki*∫(p_ref - p)dt
thr,q  = _tilt_quat_from_accel(a_sp, kT_eff, heading)  [reuse acados.py:251]
w_cmd  = K_att * quat_error(q, q_meas)
stick  = betaflight_rates_inv(w_cmd)                   [NEW: invert dynamics.py:123]
```

- `velocity_loop.py` is a **pure class** (no ROS) → unit testable and usable directly in the
  SIL bench. Design note first (D11), including the derivation.
- Behind `control_mode: 'mpc' | 'velocity'`, default **`mpc`**, so every verified config is
  byte-unchanged.
- Anti-windup: integral clamped to a configured accel authority (start ±2 m/s²), frozen while
  disarmed/grounded/landing, reset on re-arm.
- **The integrator is not incidental** — the steady payload sag *and* the attach ring runaway
  were both root-caused to the tracker having no integral action. Part A's deliverable is
  Part B's leading fix.
- Note for the thesis: Betaflight is a *rate* autopilot, so you supply the layer Quan et al.'s
  autopilot provides internally. Document that as an adaptation, not a deviation.

**Validation ladder:** V-a unit tests → V-b SIL free drone (lag < 0.1 s) → V-c SIL 3-drone
carry → V-d Gazebo hover+LAND → V-e Gazebo full trajectory set n=2,3,4 (**the decision
number**) → V-f detach under velocity mode → V-g hardware n=2.

**Gate — Fri 12 Sep.** Velocity wins ⇒ it becomes the network-phase default. Velocity loses
or ties ⇒ keep MPC+T-fixes, and **report the comparison as E3** — a measured answer to
`DISSIPATIVE_TRACKING_ISSUE.md §7`. Either way, **one architecture goes forward**.

### 12.2 Part B — attach reconfiguration (W4–W10)

**Committed goal (D3):** a welded newcomer settles as a genuine off-centre ring member, the
load stays near level, the fleet's load share visibly redistributes, and the trajectory
continues.

**A1 — reproduce and characterise (W4–W5).** Reproduce both endpoints in the SIL bench (45°
divergence, 65° tilted-but-stable). Automated elevation sweep {40…75}, 5 repeats, bench then
Gazebo → **the stability boundary curve, a thesis figure regardless of outcome (N4)**.
Attribute the mechanism with data: modelled vs measured cable accel, thrust deficit integral,
payload angular rate, tension share vs the wrench solve's assumption. Working hypothesis to
confirm or kill: newcomer transits out → real cable force exceeds the capped feedforward →
no integrator ⇒ under-thrust → sinks → payload moment grows → tilt → geometry rotates →
larger mismatch.

**A2 — fixes in dependency order (W6–W8).** Each gets an ablation row in E7.

| ID | Fix | Targets |
|---|---|---|
| A2.1 | **Velocity loop with integral action** on the newcomer (from Part A) | The under-thrust link. If Part A chose MPC, add a bounded integral trim instead — do **not** leave the tracker integrator-free |
| A2.2 | Raise/adapt `CABLE_ACCEL_CAP` for the welded newcomer | Modelled/measured mismatch (6 vs 15–20) |
| A2.3 | Slow and shape the hand-out (`T_handout` 12 → {20, 30}); ease elevation **last** | Transit rate |
| A2.4 | Tension solve uses **instantaneous** hand-out elevation, not design elevation | Geometry/wrench mismatch during transit; suspected cause of the 65° residual tilt |
| A2.5 | Slow payload-attitude outer loop biasing the cone target to null measured tilt | Residual tilt, only if A2.1–A2.4 leave it |
| A2.6 | Load-height integrator on the reference apex | The steady droop (worsens with fewer drones); also improves detach numbers |

**A3 — validation (W8–W10).** Gazebo matrix: weld azimuth {180° gap-centre, 60°, 300°} ×
elevation {45, 55, 65} × hand-out {on, off} × 5 repeats. **Success (fixed now so it cannot
drift): settled load tilt ≤ 10°, payload height within 0.05 m of target, newcomer carrying
≥ 15% of total tension, no drone exceeding 0.2 m tracking error, sustained ≥ 60 s, trajectory
resumed.** Then the same **during a circle** — the real demonstration of the contribution,
not something to leave to the last week.

**Fallback ladder (W10 gate only):** (1) modest-tilt ring member, 10–20°, reported with the
boundary curve; (2) **2 tethers + 1 newcomer** — an even 3-gon *is* reachable where a 4-gon on
a fixed 120° tripod is not; **probe this in the SIL bench in W5, not at the gate**; (3) central
lifter as the headline, ring as a characterised negative result.

### 12.3 Part C — hardware campaign (W3–W13)

~25 lab days. Standing rules: **no first-time-anywhere code at the rig** (the exact config must
have passed in Gazebo within 3 days, archived); **a written flight card per session** prepared
the day before; **data analysed the same day** (`tools/metrics.py` runs before you leave, and
`tools/backup_results.sh` is the last line of every card); **every hardware config has a
matching sim config ID** so E9 comes free.

| Stage | Content | Prereq | Days |
|---|---|---|---|
| H0 | Rig bring-up: mocap routing, ELRS per drone, **per-airframe kT bench measurement**, solo hovers, abort-path test | §5 failsafes | 1 |
| H1 | 2-drone carry: hover → line_x → circle. Establishes the sim-to-real gap **early (W4)** | Baseline sim | 2 |
| H2 | 3- and 4-drone carry, full trajectory set | H1 | 3 |
| H3 | **Detach** 4→3, 3→2, hover then circle | H2 + sim matrix | 3 |
| H4 | Dissipative controller (frozen architecture) across the trajectory set, n=3,4 | Part A gate | 3 |
| H5 | Magnet drone with Tejen: mocap bodies, ELRS mux, approach to a **static** payload, then a **hovering** payload — no weld | Part B A2 | 3 |
| H6 | **Attach** mid-flight: weld in hover, then during a circle | H5 + A3 | 4 |
| H7 | Contingency / repeats / gap-filling | — | 4 |
| H8 | Demo capture for thesis and presentation | H3, H6 | 2 |

**H5 is the hidden schedule risk** — it depends on another person's hardware and time.
**Book those sessions with Tejen in W1** for W9–W11.

**kT on hardware:** real launches keep `thrust_ratio:=24.0` — a FIXED gain; the adaptive
estimator and the `thrust_quad_c` schedule were deleted on 2026-08-05 (supervisor-approved).
Bench-measure each airframe's hover kT in H0 and record it per airframe in the flight card and
the rig profile. If the packs sag enough to matter, calibrate `kt_batt_sag_frac` (hover the same
load on a full and a nearly-flat pack, compare hover throttle) — it is built but ships at 0.0.

---

## 13. Experiment matrix

| ID | Experiment | Where | Scale | Supports |
|---|---|---|---|---|
| E1 | Baseline: {OCP, dissipative} × {hover, line_x, circle, fig_8} × n∈{2,3,4} | Sim | 120 | thresholds, C3 |
| E2 | Tracking fixes T1–T4, individually + combined | Sim | 6×5 | Part A |
| E3 | **Architecture head-to-head**: MPC-tracked vs velocity-commanded | Sim + HW | 24 + 4 | **C3** |
| E4 | **Controller head-to-head**: centralized OCP vs dissipative | Sim + HW | 24 + 6 | **C3** |
| E5 | **Detach**: 4→3, 3→2, hover and mid-trajectory | Sim + HW | 20 + 6 | **C1** |
| E6 | **Attach**: central vs ring, hover and mid-trajectory, per sub-claim N1–N4 | Sim + HW | 30 + 6 | **C2 / novelty** |
| E7 | Network ablations: slot spring, hand-out, balanced tensions, horizon rollout, integral trim | Sim | 10×5 | design justification |
| E8 | Scalability: n=2,3,4 carry; n±1 at each size | Sim (HW subset) | 18 + 4 | generality |
| E9 | **Sim-to-real gap**: every HW config paired with an identical sim config | Paired | derived | **C4** |
| E10 | Robustness: mocap dropout, kT mis-seed ±15%, payload mass ±25%, delayed reference | Sim (+1 HW) | 12×5 | practical validity |
| E10-F | **Failure→redistribution study** (sim only; hardware policy is coordinated abort, §5.3) | Sim | 8×5 | resilience claim |
| E11 | Attach elevation **stability boundary** | SIL + Sim | 8×5 | **N4**, mechanism figure |

~310 sim runs ≈ 12 h headless — two overnight batches. Affordable only because of §9.

---

## 14. Schedule

Week 1 begins Mon 4 Aug 2026. ▣ = lab day.

| Wk | Dates | Sim / dev | HW | Milestone |
|---|---|---|---|---|
| 1 | Aug 4–8 | **F1–F6, F12**: results/ + backup, clean_slate, ACADOS_DIR, geofence design note. **Book Tejen W9–W11.** Novelty search (§1.2) | — | Data safe |
| 2 | Aug 11–15 | SIL bench + runaway reproduction; metrics; plotting; redundancy cleanup; E1 baseline | — | **M1: infrastructure green, baseline measured** |
| 3 | Aug 18–22 | Stage T (T1–T4), E2; failsafe implementation | ▣ H0 | Safety verified |
| 4 | Aug 25–29 | Velocity loop build; A1 reproduce | ▣▣ H1 | **M2: sim-to-real gap known early** |
| 5 | Sep 1–5 | V-a…V-d; A1 elevation sweep; **2+1 geometry probe** | ▣ H1 | Boundary curve v1 |
| 6 | Sep 8–12 | V-e, V-f, E3; **write Methods ch.** | ▣▣ H2 | **M3: tracking architecture FROZEN** |
| 7 | Sep 15–19 | A2.1–A2.3 in SIL | ▣▣ H2/H3 | |
| 8 | Sep 22–26 | A2.4–A2.6; E7 | ▣▣ H3 | **M4: detach validated on hardware** |
| 9 | Sep 29–Oct 3 | A3 Gazebo matrix (hover attach) | ▣▣ H4 | |
| 10 | Oct 6–10 | A3 moving-trajectory attach; E11 | ▣▣ H5 | **M5: reconfiguration decision** |
| 11 | Oct 13–17 | E4, E5, E6 campaigns (overnight) | ▣▣ H5/H6 | |
| 12 | Oct 20–24 | E8, E10, E10-F; Results scaffolding | ▣▣ H6 | **M6: attach flown on hardware** |
| 13 | Oct 27–31 | Gap-filling; all analysis; `thesis_figures.py` | ▣▣ H7 | **M7: RESULTS FREEZE** |
| 14 | Nov 3–7 | Contingency only; figures | ▣ H7/H8 | Demo video |
| 15 | Nov 10–14 | Writing: Results + Discussion | — | Full draft |
| 16 | Nov 17–21 | Writing: Intro, Conclusion, revisions | — | Supervisor review |
| 17 | Nov 24–28 | Final revisions | — | **SUBMIT Thu 27 Nov** |

**Write continuously.** Method chapters are written the week a method freezes (W6, W10), not
in W15. The background chapter's novelty positioning (§1.2) is written in **W1**.

**Weekly cadence:** Mon plan + queue the sim batch + prepare flight cards · Tue/Thu lab ·
Wed/Fri development against the SIL bench, analyse lab data same-day · Fri run the gate,
append to `learning.txt`, update this plan, commit, **verify the backup ran**.

---

## 15. Risk register

| # | Risk | Lik. | Impact | Trigger | Response |
|---|---|---|---|---|---|
| R0 | **Novelty challenged on the Quan et al. joining claim** | Medium | **High** | Examination | §1.2 reframe, written in W1; N1–N4 each separately evidenced; state their claim explicitly rather than hoping it is missed |
| R1 | SIL bench can't reproduce the runaway | Medium | **High** | Fails acceptance end W2 | +3 days of fidelity (solver timing, IMU noise, rod compliance); else fall back to headless batch Gazebo as the discriminator |
| R2 | Velocity architecture loses | Medium | Low | W6 gate | Report as E3; keep MPC+T-fixes |
| R3 | Ring reconfiguration doesn't stabilise | **Med-high** | **High** | W10 gate | §12.2 ladder; 2+1 geometry probed in W5, not at the gate |
| R4 | Magnet drone unavailable | Medium | **High** | Missed H5 booking | Book W1. Fallback: fixed downward magnet on one of your own airframes — far simpler than a swung one and sufficient to demonstrate N1–N4 |
| R5 | Hardware damage during attach | Medium | Medium | — | Spares; H6 only after H5 approach-without-weld; hover weld before trajectory weld |
| R6 | Lab access drops below 1 day/wk | Low | High | 2 weeks missed | H1/H3/H6 are load-bearing; H2/H4/H8 compress |
| R7 | Sim-to-real gap invalidates sim results | Medium | Medium | H1 (W4) | Found in W4 by design, not W12; if large, characterise it as a result and shift claim weight to hardware |
| R8 | Environment friction (DDS leaks, acados rebuilds, geometry drift) | **High** | Medium | Ongoing | §9.1 clean slate; §7.2 startup validation; manifests catch parameter drift |
| R9 | Writing compressed into the last fortnight | Medium | **High** | No draft chapter by end W6 | "Method frozen ⇒ chapter written that week" |
| R10 | Tejen's stack changes underneath you | Medium | Medium | Any H5/H6 session | Integration contract test (§8); pin his commit for each experiment batch and record it in the manifest |

---

## 16. Thesis structure

| Ch | Content | Evidence |
|---|---|---|
| 1 Introduction | Problem, motivation, contribution N1–N4 | — |
| 2 Background | Sun et al.; Quan et al. **including their entry/exit claim and exactly how yours differs (§1)**; the alternatives table (§11) | — |
| 3 System & methodology | Rig, sim, ROS 2 architecture, wire format, **verification methodology (offline gate → SIL bench → headless Gazebo → hardware)** — a methodological contribution in its own right | §9 |
| 4 Centralized OCP controller | Coupled planner + cable-aware tracker | E1, E4 |
| 5 Dissipative network | Port, adaptations, and the **architecture study** | E1, E2, **E3**, E7 |
| 6 Detach | Method, redistribution, landing; failure-response study | **E5**, E10-F, E8, E9 |
| 7 **Attach — the contribution** | Rendezvous, weld, handover, reconfiguration, stability boundary | **E6**, E11, E7, E9 |
| 8 Results & discussion | Head-to-heads, scalability, robustness, sim-to-real | E4, E8, E9, E10 |
| 9 Conclusion & future work | Honest limits; CBF rendezvous; admittance; distributed MPC | — |

---

## 17. Immediate actions (this week)

Ordered by lead time and irreversibility.

- [ ] **Book Tejen's H5 sessions for W9–W11** — longest lead time, do it first
- [ ] **Move logs out of `c_generated_code_quad_load/` and run the first backup** (F1) — you are one `rm -rf` from losing 237 MB of flight data
- [ ] Rewrite `cleanup.sh` → `tools/clean_slate.sh`, including `/dev/shm` (F2)
- [ ] Search for Quan et al. supplementary material; **draft the novelty positioning paragraph** (§1.2)
- [ ] `run_context.py` + absolute `results/` + manifest; `ACADOS_DIR` from env (F5, F6)
- [ ] Delete `skip_steps` and strike `DISSIPATIVE_TRACKING_ISSUE.md §4.3`; fix "Fig. 3" → "Fig. 2" in §7 (F12, §1.3)
- [ ] Design note for the **fleet abort** and the **geofence** (D11) → your approval → implement (F3, F4)
- [ ] Implement and measure **T1** (terminal velocity reference) — smallest change, largest suspected effect
- [ ] Merge `attach-week9` → `main` (4 commits, fast-forward) so the baseline sits on a stable branch

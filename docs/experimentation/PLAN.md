# claude-experimentation — agenda (branched from term3 @ 3aed55b, 2026-09-10)

Wesley's brief: a sandbox for new methodology and new deliverables — experiments from the
literature, how results are displayed in the thesis, setup improvements, real-world
testing ease, the RViz setup. Everything here is evidence-first: a change ships with the
run IDs that justify it, and the gate stays green.

## Queue (in leverage order)

| # | Item | Why | How verified |
|---|---|---|---|
| 1 | **Mass-scaled OCP hover offset** (+3.5 cm @0.4 kg, +18 cm @0.6 kg, SIL; +12 cm Gazebo) — RESOLVED: sim kT operating point | Every flight started high; feeds the weld transient | SIL A/B on kT (R0228–R0230), gate smoke R0232 |
| 2 | **Tilt-aware network wrench** — attach points at the *true* load attitude for the tension solve, yaw-only kept for the formation build | The remaining weld transient (+8°) is the network modelling a 34°-tilted three-drone hover as level | Harness H/J (rigid-body tilt), then the circle-attach A/B |
| 3 | **Thesis figure set for the attach demo** — storyboard (3D path + events), tilt/height vs time with weld/hold/resume bands, tension share, newcomer transit | Chapter 7's evidence, regenerable from the registry | `tools/thesis_figures.py` entries pointing at R0208/R0209 |
| 4 | **Robustness sweep the paper reports** (Quan et al.: payload ±25 %, cable length ±40 %) on the attach demo | E10 for chapter 8, cheap headless | 5 configs × 2 repeats, one table |
| 5 | **Pre-flight tool** (`tools/preflight.py`, plan §5.4) + flight card + `param_diff` for the new attach params on the real launch | Hardware campaign starts in ~2 weeks; nothing exists for the rig attach | Runs against the real launch files; dry run at the rig |
| 6 | **RViz**: phase/hold/tilt banner, newcomer role colour, reference→actual error lines (plan §6) | The attach signature was found from logs, not RViz | Structural tests in `test_rviz_config.py` |
| 7 | fig-8 attach; n=2+1 attach; detach-then-attach round trip | Generality claims for chapter 8 | Headless matrix |

Findings land in `learning.txt` as usual; this file tracks status.

## Status
- 2026-09-10: branch created; items 1–3 started.
- Item 1: the OCP is NOT the culprit — a standalone `PlannerSolver` test at 0.6 kg holds
  the target from the target and plans a clean descent from a high start (0.78 → 0.605
  over the horizon). The planner's own diag also shows `z_tgt 0.60` against `load_z 0.78`.
  So the closed loop is not executing the horizon; the planner diag now logs the planned
  end-of-horizon load height (`zN`) and drone-0 height (`d0N`) to split tracker vs planner.
- Item 2: the offline harness cannot reproduce the tilted 3/12/9 hover (its PD proxies hold
  it level: 1.6°), so the tilt-aware wrench flag (`diss_wrench_true_attitude`) is decided in
  Gazebo (`attach_circle_n3_fast_wta`).
- Item 3: `tools/attach_storyboard.py` renders the demo as one figure (plan view with
  ATTACH/WELD/RESUME, tilt+height with hold bands, the newcomer's transit) — `docs/figures/`.
- Item 4: four mis-seed configs (mass ±25 %, cable ±10 % as controller beliefs) queued.
- Item 6: `/fleet/status` banner (phase / hold / drones on load / tilt) above the payload in
  RViz, colour-coded by tilt; structural test added.
- Item 2 DECIDED (negative): the tilt-aware wrench drove the weld transient to 68° and an
  abort in 2/2 runs (R0212/R0213) vs +8° yaw-only. Flag stays off; recorded in learning.txt.
- Item 5: `tools/preflight.py` verified live against a running stack: mocap rates, rod
  length from mocap at each rim point (0.500 ±0.001 m), per-node parameter read-back
  (planner geometry, every tracker's kT and control_mode), banner, ground check; writes a
  flight-card section under results/preflight/.
- Item 3: `F_attach_demo_storyboard` is a registry figure (`kind: storyboard`), regenerated
  by `tools/thesis_figures.py`.
- Item 7: fig-8 attach clean (R0226); round trip attach → resume → detach the newcomer
  clean (R0231) after fixing the detach-path crash on a reserved drone (R0223/R0224).
- Item 1 RESOLVED: the offset is the fixed sim thrust ratio at the wrong operating
  point (SIL A/B R0228–R0230). Sim launches now derive kT from load_mass and fleet
  size (`thrust_model.py`, `thrust_ratio:=auto`); gate smoke settles on target (R0232).
- Item 5 DONE: `real_attach_launch.py` (hardware twin of the sim attach launch, brought up
  node-for-node without hardware), magnet-tip rigid body in the mocap publisher, mocap-state
  payload input for the magnet manager, magnet aux channel merged in `elrs_mux`, `attach:=`
  on `real_io_launch.py`; `docs/experimentation/real_attach_gap.md` is now the rig checklist.
- Item 6 DONE: drone id labels coloured by role (T/N/W/D from `/fleet/status`) and a
  reference→actual error line per drone (`/fleet/error_markers`), structural tests added.
- Item 7, n=2+1: NOT RUN, by geometry. A disc hung from two rim points is a pendulum about
  their chord: with the tethers at 9 and 3 o'clock the chord passes through the centre and
  the attitude is indifferent (any tilt is an equilibrium); any other pair leaves the
  centre of mass off the chord and the disc swings edge-on. The third cable is what gives
  the 3/12/9 hover its (34°) equilibrium at all. Two-drone rim carry needs a payload whose
  attach points straddle the centre of mass in both axes, not this ring.
- Stale-reference aborts (R0169, R0225) traced to CPU contention in Gazebo runs (tick
  watchdog: 86 slow ticks in R0231, none in SIL): headless Gazebo now runs niced, and
  the attach launch carries a 2 s sim reference budget.
- Gate: stage 6 now prebuilds the planner solvers before the SIL smoke — the cache goes
  stale on any planner-source or mass/inertia edit and the bench starts its clock before
  the build finishes; this cost three separate false "no lift" failures today.

## Branch `measured-force-velocity-loop` (2026-09-10)
Survey `control_methods_survey.md` → R1 (measured-force throttle, `docs/design/measured_force_loop.md`)
and R4 (hybrid dwell, `hybrid_dwell.md`, `attach_traj_hold_mode: settle`) implemented; results in
learning.txt under the branch heading.

# claude-experimentation — agenda (branched from term3 @ 3aed55b, 2026-09-10)

Wesley's brief: a sandbox for new methodology and new deliverables — experiments from the
literature, how results are displayed in the thesis, setup improvements, real-world
testing ease, the RViz setup. Everything here is evidence-first: a change ships with the
run IDs that justify it, and the gate stays green.

## Queue (in leverage order)

| # | Item | Why | How verified |
|---|---|---|---|
| 1 | **Mass-scaled OCP hover offset** (+3.5 cm @0.4 kg, +18 cm @0.6 kg, SIL; +12 cm Gazebo) | Every flight starts high; feeds the weld transient; a planner correctness question | Standalone `PlannerSolver` hover test at 0.4/0.6 kg (no ROS), then SIL smoke height |
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

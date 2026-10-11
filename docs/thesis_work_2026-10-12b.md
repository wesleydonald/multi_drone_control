# Thesis lane, overnight 11-12 October 2026: Chapter 2 and Chapter 3 drafts (hand-back)

Branch `thesis-work3` (worktree `/home/wesley/mdc_wt/thesis-work3`, from `thesis-rig` 5dc4eb1). Nothing merged, nothing
pushed; the main tree, the rig branches, the registry, the lab PC and the rig stack untouched; no `desk-ext3` was
needed (no scripts). Commits are authored by Wesley, no trailers. Times are from `date`.

## What was drafted

Chapter 2 (literature) and Chapter 3 (method) prose, in the structure of `docs/thesis_plan_2026-10-11.md` section 4,
from the literature already placed and the signed metrics. No result number that the Friday 16 claim or Wednesday's
rig flights will change is written; those places are bracketed `[...]` in the text. The thesis builds with no
undefined references or citations (108 pages before Wesley's cuts; Chapter 2 is pages 5-14, Chapter 3 pages 15-37
of the PDF).

### Chapter 2, Literature Review (`Source/Literature_review.tex` and new `Source/src/lit_*.tex`)

| section | file | content |
|---|---|---|
| 2.1 Cooperative transport and link models | `lit_transport.tex` | link models (rod, chain, the taut assumption, the rod as the rig's own link); centralised control with Sun 2025 as the benchmark (solve time grows with the team; the INDI contribution not separated); distributed and decentralised in one paragraph; why the network is not a research question; what the thesis takes |
| 2.2 Changes of team membership | `lit_membership.tex` | reconfiguration defined as membership change; the leave on hardware (Quan, Zeng, Bacheti); the link that fails in simulation (Liang, Saiella, Hajieghrary, Farras); the join (no work; Quan's prose; Tasooji; Shankar's hand-over); plug-and-play MPC; what statics decides; **T3 `tab:membership`**, the numeric table with detection kept apart from recovery |
| 2.3 Capture proof, docking and magnetic grasping | `lit_capture.tex` | proof from a sensor against proof from pose; docking in flight (ModQuad, TRADY, EPM, Miyazaki, LEGION, Antal, Xu); magnetic grasping and the angle of pull |
| 2.4 Switching, bumpless transfer and runtime assurance | `lit_switching.tex` | bumpless transfer and TRADY's blend against the warm start, ramp and seed; real-time iteration and why a switch breaks the warm start; Simplex-style runtime assurance and open multi-agent systems |
| 2.5 Execution layer and force estimation | `lit_execution.tex` | INDI onboard against a ground tracker on unmodified Betaflight; estimating the link force; offset-free MPC and the model integral |
| 2.6 Simulation to reality | `lit_sim.tex` | SIL fidelity, the twin's stated omissions, the twin scored as a predictor |
| 2.7 The gap | wrapper | three sentences: the join, the release transient on hardware, the scored simulator |

The old `modelling.tex` and `control.tex` are no longer input (their text is condensed into the files above); the
distributed and decentralised detail with the architecture figure is now the first part of Appendix F
(`app_network_lit.tex`). The old comparison table `tab:control_comparison` is gone; `tab:lit_compare` in the
discussion is untouched (the plan folds it into T3 at the results rewrite).

### Chapter 3, Methodology (`Source/Methodology.tex`)

| section | file | content |
|---|---|---|
| 3.1 Rig, twin and roles | `system_overview.tex` (rewritten) | the rig (cage, mocap, Betaflight, ELRS, the three nodes); UAVs, rods and ring with the magnet's catch-on-contact and hold; **T4 `tab:plates`** plate numbering by angle; the twin (moved here from `simulation.tex`, label `sec:simulation` kept) with the six things it does not model and the body-rate rounding fix; roles (ours against the collaborator's, with the credit wording bracketed) |
| 3.2 Model and planner | `model.tex`, `controller.tex` (headings re-levelled) | the model as before; the planner with the runtime geometry, one solver per team size primed twice, the 60 ms budget, and the rig's "dissipative mode" relabelled (the OCP inside the reconfiguration node) |
| 3.3 Tracker, thrust map, measured pull and model integral | `controller.tex`, `takeoff_landing.tex` | the tracker and the model integral as before; takeoff and landing condensed to half a page, the detail moved to Appendix C "Floor start and landing in detail" (`sec:app_floor`) |
| 3.4 Reconfiguration | `reconfiguration.tex` (rewritten) | notice / re-plan / redistribute / fall back; **F6 `fig:reconfig_states`** (the leave and join lanes and the exits); the three-equation statics `eq:statics`; the support margin `eq:margin` with the qualifier and the 112/86/58/29/0 mm values; the leaver rule and what control still sets (kept from 11 Oct); the ramp and the seed; the freed UAV and the take-back; the admission contract (`sec:admission`); **T5b `tab:constants`**, every constant with a one-line reason |
| 3.5 Authority and safety | `authority.tex` (new; label `sec:safety` moved here) | **F5 `fig:owners`**, the cascade and its owners (replaces the old nodes and cascade figures, whose sources are kept in comments); the envelope and the fleet disarm argued; the six fall-backs as runtime assurance; what is tested where (rig switch states bracketed per plan D6); the fault matrix pointed at Appendix D |
| 3.6 Method and metrics | `simulation.tex` (rewritten as the protocol), `trajectories.tex`, `metrics.tex` (extended) | cards and frozen bars; one tree per claim table and rig rows carrying their own tree; the critic and reviewer described as automated checks, with the AI-disclosure placeholder; replication against new prediction; the runner and the ladder; the metrics now also list the join metrics, the magnet margin and the switch validation, and the pre-event state reported with every arm |

The old attach sequence (the twin's proximity weld) is Appendix D `sec:app_proximity_weld`, so nothing was deleted.

## Placeholders left in the text (grep for `[` in the chapter files)

- Citations the plan lists for Chapter 2 that are not in the bibliography and wait for the approval batch (plan
  section 7, D14): Six 2018; Fink 2011 and Tognon 2018; Farina 2020; Loianno 2018; Gawel 2017; Graebe and Ahlen 1996;
  Peng 1996; Sha 2001; Gros 2020; Hendrickx and Martin 2017; Sekercioglu 2026; Tagliabue 2019; de Marina and Smeur
  2019; Bulka 2022; RotorTM; RotorS; Cruz and Fierro 2017. Each is named in brackets where it would be cited.
- T3 cells to fill from the full texts (plan C2-A): Bacheti's agent count; Liang's, Saiella's and Farras's team sizes
  and numbers; Hajieghrary's attitude number (the 0.32 s is marked as a recovery time).
- Numbers that wait for flights: the magnet hold against the angle of pull (14 Oct spring scale); the rig half of T3's
  "this work" row (the 14 Oct flights extend the 8 Oct range); the join claim (16 Oct); the rig proof lift cap and
  tether slack; the planner's measured solve times; the counterfactuals of the seed and of the take-back; the tilt
  cut's rig value; the run and test counts at the freeze.
- The AI-disclosure statement (plan D13); the roles and originality wording (plan D11).

## For Wesley: cuts and checks

- Length: Chapter 2 is about 10 pages against the plan's 8.5, Chapter 3 about 23 against 12, before any cut. The
  method keeps the full planner, tracker and integral derivations from the 4 Oct draft; the plan's cuts (the
  integral's alternatives to Appendix D, the derivation to two equations) are not made here because they are
  deletions of text you wrote. Cutting is yours.
- `fig:tension` (the two link models) is no longer in the main text; the file `modelling.tex` still holds it if you
  want it back in 2.1.
- The old figures `fig:nodes` and `fig:cascade` are replaced by `fig:owners`; their sources are kept as comments in
  `system_overview.tex` (gone with the rewrite; recover from git 5dc4eb1) and `controller.tex`.
- Two overfull boxes remain (Table 2's work column, 2 pt; the state-machine figure, 5 pt); harmless.

## Decisions for the record

None taken. Two questions, with a recommendation:
1. The literature approval batch (plan D14): the bracketed citations above are the ones the drafted text actually
   needs; the rest of plan section 7 can wait. Recommended: approve these seventeen, in one batch, before Friday.
2. The magnet hold: 3.1 says "about 9-10 N straight off the face" (your note of 7 Oct); the 30 Sep review said
   10-12 N rated. Recommended: keep 9-10 N as measured and let the 14 Oct spring scale replace both.

## Script for Wesley (deletions are refused here)

```bash
# the two review files no longer input (their text lives in lit_*.tex and app_network_lit.tex)
cd /home/wesley/mdc_wt/thesis-work3/docs/thesis_drafts/thesis_v2/Source/src
git rm modelling.tex control.tex
# when the branch is merged or declined
git -C /home/wesley/multi_drone_control worktree remove /home/wesley/mdc_wt/thesis-work3
git -C /home/wesley/multi_drone_control branch -D thesis-work3
```

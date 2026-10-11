# Thesis and desk lane, 11 Oct 2026: hand-back

Done in two worktrees, nothing merged, nothing pushed, nothing in the main tree, the registry, the sheets, the
cards or the lab PC touched. Commits are on `thesis-work` (from thesis-rig 1659d7d) and `desk-ext` (from
rig-prep 6129ea3). The other session copies the "decisions for the record" below into docs/decisions.md.

## 1. What was done and where

### thesis-work (`/home/wesley/mdc_wt/thesis-work`, docs/thesis_drafts/thesis_v2)

| commit | what |
|---|---|
| 231677a | Literature. The leave claim corrected against Quan et al.'s full 33-page text and Zeng et al. 2025; seven entries added and placed; builds clean. |
| 1772b42 | Structure prep for the adopted spine: RQ1-RQ3, C1-C3, M1-M2; Appendix F for the network; Q1/Q2 folded; the bar rationale; the ledger outline. Builds clean, 96 pp. |

**Literature facts now in the draft, verified from the PDFs:**
- Quan et al. 2025 (unplug-and-play, PDF 15-17, Fig. 6E-H): four robots, 4 kg load, each with a cable release;
  Robot 2 releases on command at 40 s; the other three reorganise to a near-equilateral layout and the tensions
  are balanced after 13 s; one robot's velocity plotted. No load attitude or swing number anywhere in the paper.
  Joining is prose only ("connected to the payload and its neighbors", PDF 7-8 and 19); varying team sizes are
  referred to the supplementary S5.
- Zeng et al. 2025 (CoRL, PMLR 305): three MAVs, 1.4 kg basket on 1 m cables; one MAV is switched off in flight
  and hangs under the load; the other two keep five degrees of freedom of the load under a learned decentralised
  policy; load position and attitude are plotted through the loss (Figs. 8-9), no transient number; no join.
- So the claim is now, everywhere it appears (intro, literature gap paragraph and table, discussion "Against the
  literature" and the Quan row of tab:lit_compare, conclusion): **no work reports a number for the payload's
  attitude through a departure, or flies a join.** The detach is positioned as the centralised resize with
  load-pose control and a measured attitude transient, never as the first mid-flight detach.
- Added to references_extra.bib (metadata from Crossref or the arXiv API) and placed: Liang 2021 (AST), Saiella
  2021 (ICUAS), Hajieghrary and Schmitt 2026 (arXiv 2605.05339): cable-loss fault tolerance, simulation only, a
  new paragraph in the team-composition section and three rows in tab:control_comparison; Jackson 2020 (RA-L):
  "reconfiguration" defined in the intro as a change of team membership against a fixed team changing shape;
  LEGION (arXiv 2605.19431) and Miyazaki 2018 (IROS) in the docking paragraph; Zeng 2025 as above. Zeng was not
  on the brief's list: it was added because the corrected claim is false without it.

**Undermind citation sweep** (workspace "Wesley's workspace"; citations of Quan 2025 and of Sun 2025):
- Nothing cites either paper with a join onto a carried load. Worth a look for the approval batch: Lv et al.
  2026, "Multirotor UAVs Transporting Cable-Suspended Loads: A Literature Review", IEEE T-ITS 33:7308-7331
  (the most recent survey; abstract lists no team-change topic); Liu et al. 2026, IEEE TNSE 13:5905-5923 (the
  virtual-passivity version of the network, field tests with six UAVs and 6 kg; the plan already names it as
  the peer-reviewed replacement where it overlaps the preprint); Xu, Bhattacharya and Saldaña 2026 (ACC,
  midair control of cable hitches, simulation); Antal et al. 2026 (RA-L, hook pick-and-place between moving
  platforms, hardware); Fu et al. 2026 (EITCE, payload-level reward for RL transport). None changes a claim.
- The agent's own caveat stands: web and Undermind only, no IEEE Xplore full-text search; one more sweep in
  early November.

**Structure prep (outlines and non-results text only, per the plan: prose from Fri 16):**
- contributions.tex: RQ1 carry, RQ2 leave, RQ3 join in the plan's words; the ledger named as Section 4.4; the
  scope paragraph for the planner-against-network question; tab:rq_map; C1-C3 plus M1-M2 as one paragraph
  each. C3's numbers are a bracketed placeholder until the Fri 16 claim and the 21-22 Oct rig attach. C2 uses
  only fixed rig facts (five of six releases carried on, 4.6-17.8 deg, 0.33-0.42 s) and the admissibility result.
- Results introduction: the question order and a "Scope" paragraph pointing the network question to Appendix F.
- Appendix F "The dissipative network on the earlier plant": dissipative.tex and sim_architecture.tex moved out of
  the method and results chapters with their earlier-plant text restored from the comments; the network blocks
  of the detach and attach sections (tab:detach_ab, fig:attach, tab:attach_layout, the SIL A/B boxes) collected
  into app_network_results.tex; the RQ2 redo boxes in the conclusion, discussion and abstract removed. The
  appendix opens with the earlier-plant label. The timeline is Appendix E, the network F, as the plan names it.
- Q1/Q2 fold: res_steady.tex is a subsubsection of the carry section with its parts as paragraphs; "Answer to
  Q1/Q2" renamed; RQ1a/RQ1b renamed RQ2/RQ3 in the detach, attach, discussion and conclusion sections. The
  estimator comparisons still sit in the main text: moving them to Appendix D is part of the results rewrite.
- metrics.tex: "Why these bars" (the magnet's hold under shear and the 7 Oct releases at 45 deg; the 30 deg drop
  rule as where the controller gives up; the planner's near-level model), with the 14 Oct spring-scale number
  marked as missing. The T6 definitions (tilt-vector rise over a 5 s pre-window, the post-event band) are not
  written: they are frozen Mon 12 (plan P1/D4).
- ledger.tex: the outline of Section 4.4, no numbers.
- A `sec:safety` label on the safety paragraph of system_overview.tex (M1 and the bar paragraph cite it).
- Not done, on purpose: the abstract (plan section 8, after the join numbers), T1 roles, T3/T8/T11 tables, the
  storyboard F1, the vocabulary pass (QUAD4 -> newcomer, flight labels), the 50-page cut. The redo boxes that
  remain are join and leave results awaiting the claim (sim_detach.tex 3, sim_attach.tex 4, discussion.tex 3,
  Conclusion.tex 2) and the method boxes in reconfiguration.tex (4) and Appendices.tex.

### desk-ext (`/home/wesley/mdc_wt/desk-ext`, commit 5889052; outputs under results/analysis/)

**(1) tools/admissibility_map.py** (metrics.cog_margin and the planner's own balanced_tensions; 5 tests in
tools/test/test_admissibility_map.py). Findings:
- **No four-plate layout on the ring can lose an arbitrary carrier and stay level.** The worst single removal
  merges two adjacent gaps, and the four gaps sum to 360 deg, so some merged pair is at least 180 deg
  (0 of 495 layouts; 255 can lose none). A layout admits at most two leavers. This sharpens the N-1 rule: an
  N-1 layout is designed for a known leaver, and a team that must tolerate any single loss needs five carriers.
- Layouts of record, leaver at 90 deg: 1/3/5/9 margin 112.5 mm (gap 120), 1/3/6/9 58.2 mm (gap 150); the even
  ring 0 mm after any loss. Losing the 270 deg carrier of 1/3/5/9 puts the centre 112.5 mm outside (gap 240).
- Where a newcomer should attach: on an even triple every free plate works for the newcomer's own later
  departure (back to the triple, 112.5 mm); a plate 30 deg from a carrier also makes that carrier an admissible
  leaver, a gap centre makes only the newcomer admissible. The level-hover share spread is 0.207 beside a
  carrier against 0.222 at a gap centre. On 1/5/9 the M1 plate 3 (90 deg) is the only choice that gives one
  admissible leaver (itself) with spread 0.222; plates 0, 2, 4, 6, 8, 10 give two leavers and spread 0.207.
- The static shares also predict each survivor's step at the resize (see (2)).

**(2) tools/release_transient.py** (twin pairs R1140/R1145, R1206/R1211, R1207/R1212, R1208/R1213; rig r0012,
r0013, r302_3, r303_4, r304_3, r305; the rig layouts typed from the registry rows; clocks aligned on lift-off).
Table: results/analysis/release_transient/table.md. Findings:
- **The resize is the static redistribution.** The planner's largest tension step after the resize against the
  static prediction from the layout: twin 1.08-1.13 N against 1.07 (gap 120) and 1.63-1.65 against 1.60
  (gap 150); rig 1.14-1.21 against 1.07 and 1.74 against 1.60. The one outlier is r0013: 2.75 N against 1.61
  (70 % over; the leaver carried 1.97 N under the 45 deg plan and reference integral), and a survivor's magnet
  let go 0.155 s later.
- Tilt rise within 5 s (scalar tilt above the 3 s pre-event mean): gap 120 twin 4.1-7.1 deg (hover 5.9-7.1,
  circle 4.1-5.4), rig 4.0 (unannounced hover), 5.4 (circle), 2.4 (r0012, planned pull), 10.7 (r302_3,
  announced); gap 150 twin 14.7 / 17.4, rig 16.5; r0013 87.7. Dip: gap 120 twin 2.1-3.0 cm, rig 3.4-3.6 (7.0 on
  the announced r302_3); gap 150 twin 4.5-4.8, rig 5.7. The 150 deg gap has 1.5 times the step and 2.5 times
  the tilt rise of the 120 deg gap, twin and rig alike; the rig's unannounced releases sit within about 2 deg
  of the twin, the announced one 4-5 deg above it.
- Detection on the rig from the bag's DETECTED event: 0.42 / 0.33 / 0.34 s (as the draft says); the twin's,
  from the planner's first smaller problem at 10 Hz: 0.27-0.42 s.
- Caveat: the rise is the scalar tilt, not the T6 tilt-vector form the plan freezes Mon 12; the script takes
  one line to switch once T6 is signed.

**(3) tools/mission_value.py** (rig segment medians from r0012, r206e60, r302_3, r303_4, r304_3, r305; the twin
full M1 R1573 and R1579 for the errand). Findings:
- Rig medians: creep 5.9 s, lift to 1.0 m 10.5 s, TAKEOFF to height 16.4 s, LAND to LANDED 9.3 s. A land-and-
  swap pauses the mission 30.7 s plus the ground swap (60.7 / 90.7 / 150.7 s for a 30 / 60 / 120 s swap).
- The in-flight errand (release to the proven attach) is 89.9 s in the hover M1 and 115.7 s on the circle, with
  no pause, at the cost of 270-347 carrier hover-seconds against 92 for the landing cycle: 178-255 extra
  drone-seconds of hover. Survivors' mean throttle 0.42 before the release, 0.45 with three on the ring (0.40
  with four). Energy in joules needs a measured hover current; the proxy is drone-seconds.
- The honest statement for the discussion: in-flight reconfiguration trades carrier energy for mission time and
  continuity; it pays when the mission cannot pause (a moving payload, a crossing) or when the ground swap is
  slow.

## 2. What needs Wesley
1. Approve Zeng 2025 in the bibliography (added beyond the brief because the corrected claim needs it), and the
   five sweep candidates above for the approval batch (Lv 2026 review first).
2. The T6 metric definitions and the W4/W5 bands (plan P1/D4, Mon 12): the transient table and the bar
   paragraph switch to them once signed.
3. The 14 Oct spring-scale magnet hold at 0/30/45/60 deg (plan D6): it is the one number the bar paragraph
   is written around.
4. Whether the admissibility result goes into Section 3.4 as stated (a four-carrier layout admits at most two
   leavers; any-single-loss tolerance needs five) and whether the "newcomer beside a carrier" rule is wanted in
   the join chapter.
5. The r0013 step (2.75 N against 1.61 static) as evidence in F11 "magnet anatomy": it says the planned step,
   not only the geometry, was large before the magnet let go.

## 3. Decisions for the record (none new; the brief's stand)
- 11 Oct: the planner-against-network question is a scope paragraph plus Appendix F (earlier plant), done in
  the draft; no port, no re-fly.
- 11 Oct: Q1 and Q2 folded into RQ1 in the draft's headings; the estimator comparisons move to Appendix D with
  the results rewrite.
- 11 Oct: the leave claim narrowed to "no number for the payload's attitude through a departure, or a join",
  after Quan et al.'s full text (13 s rebalance, no attitude) and Zeng et al. (plot, no number).

## 4. Twin runs wanted (asked, not run)
- None are needed for the three analyses. One would help the ledger: the twin pair at r0012/r0013's settings
  (planned pull, 45 deg plan, reference integral) on the rig candidate tree, so the 7 Oct rig rows get a twin
  cell in the transient table; the plan's F11 note quotes 5.5 deg for the r0013 settings, so if that run
  exists its id is enough.

## 5. Scripts for Wesley
Nothing to delete. Both worktrees stay until merged or declined: `git -C /home/wesley/multi_drone_control
worktree remove /home/wesley/mdc_wt/thesis-work` and `... /home/wesley/mdc_wt/desk-ext`, then `git branch -D`.

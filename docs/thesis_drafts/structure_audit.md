# Thesis structure audit (9 Oct 2026, on the draft copy thesis_v2 = Overleaf 4732d01)

Read against the marking rubric (literature 10 %, execution 50 %, conclusions 20 %, presentation 20 %).

## Headline

- 80 pp; main text pp 1-59 against the template's **50-page limit** (title_page.tex:49-50), before the 38
  `\redo` boxes are refilled (their commented originals hold 8 tables and 1 figure). 16 figures, 20 tables.
- **All six data figures are twin; none from the rig.** No photograph of the rig.
- Pages: literature 15, methodology 22, results 18 (more method than results).

## Problems, ranked

1. **Evidence for half the questions is not on the page:** 38 redo boxes (attach 8, architecture 6, detach 4,
   reconfiguration 4, discussion 4, contributions 3, ...). RQ2 (4.4) and the attach half of RQ1 (4.3) are empty; the
   abstract, contributions 2/4/5 and conclusion bullets 1/3 point into them.
2. **Results follow development history, not the questions:** no section names its question, baseline or bar;
   4.1 opens with a bug fix (tracker IC) before the fixed-team baseline; robustness after the RQ2 comparison; the
   hardware last, as a diary, with no figure.
3. **Over the page limit** (59 vs 50, plus 5-7 to refill).
4. **Material in the wrong chapter:** the thesis's own model sits in the literature review (modelling.tex:150-224)
   and is derived again in controller.tex:43-81 with different notation; the methodology holds results
   (controller.tex:168-172, 203-206, 226; trajectories.tex:41-66; takeoff_landing.tex outcomes; hardware.tex:38-40;
   simulation.tex:23-33); the results hold methods (sim_results.tex:9-19, 195-219; rig_results.tex:121-124); the
   review holds a stale design (control.tex:192-259, fig:control_arch).
5. **Repeats:** thrust identification (hardware / rig_results / controller), the tracker IC box (six places), "an
   integral corrects forces not moments" (five), the breakaway cap (five), stiffness 5.3/7.1 N/m (four), layout and
   support polygon (five), floor-start summary (three), simulator artefacts, sim vs rig, the three key papers
   (existing.tex vs control.tex), the research gap (three times), two parameter tables.
6. **Stale content:** proposal-era wording (Literature_review.tex:15 "centralised dissipative architecture";
   control.tex:192, 297; modelling.tex:3, 230 SITL vs the emulator; intro.tex:77; contributions.tex:26); old-code
   tables without a redo box (tab:methods, tab:sim_vs_rig twin column, tab:x0 circle row, tab:detach baseline row,
   fig:traj dashed curve, tab:rig_oct1); numbers implying the old 0.15 m integral bound (sim_results.tex:78-79);
   tab:traj flown with the integral off; markers left (Provisional, DRAFT, STILL TO DO, STATUS, TODO).
7. **Forward references:** fig:geometry (3.7) defines the symbols the model uses in 3.2; the thrust map in 3.2 before
   its identification (3.7); "rig twin" used before 3.8; the SIL bench never defined; sec:rq never referenced.
8. **Terminology and notation:** N (team size and horizon), q (cable direction, payload and UAV quaternions), u_T,
   a_i, c, m, T, d overloaded; UAV/drone/quadrotor/quadcopter/agent; cable/rod/tether; detach/leave/removal/release;
   attach/join/rejoin/docking/weld; five names for the integral; circle vs orbit; figure-eight spellings and
   periods; "hand-over" for five things; layouts named three ways; centralised/centralized spelling;
   contradictions (twin parameters vs Appendix; the detach metric; "every case flown twice" vs single runs;
   "twelve numbers per node"; contributions.tex:32 cites the wrong section; sim_robustness.tex:24 "2 mm" vs 8 mm).
9. **What the rubric rewards and the draft misses:** no numeric comparison with TU Delft / Quan / Bacheti; the review
   is a catalogue; missing topics (offset-free MPC, plug-and-play / variable-dimension MPC, static equilibrium of
   suspended loads, aerial docking); the good story (geometry decides; four kinds of steady-error source; resize not
   hand-off) is told only in the discussion; the aims and experiments E1-E4 live only in the excluded Project_plan;
   a provisional bulleted conclusion with no value statement; placeholder nomenclature and acknowledgments;
   undefined acronyms; an invented curve (fig:cost_scaling).

## Proposed structure (old -> new), target ~50 pp

1 Introduction (3.5 pp): motivation; mission profile and scope (+ the collaborator split); **aims and questions**
  as one table question -> experiment -> results section (Q1 carry, Q2 steady accuracy, RQ1a leave, RQ1b join, RQ2
  comparison: **changing the research questions is Wesley's call**); contributions (6, each naming its evidence
  section); outline.

2 Literature (10.5 pp): cable-suspended transport and model assumptions; centralised; distributed; **changes in team
  composition** (new focal section: Quan, Bacheti, Tasooji, Wang 2024, Wahba + plug-and-play MPC, static equilibrium,
  aerial docking); execution layer and steady-error rejection (INDI, offset-free MPC); simulation fidelity (emulated
  flight controller); comparison and gaps (+ a "this work" row).

3 Methodology (15 pp): system and platform (+ fig:geometry moved here, rig photo); **system model once** (modelling
  + controller merged, planner notation); planner; tracking MPC and thrust map (+ identification procedure);
  steady-error rejection (+ the alternatives, two lines each); floor start, lift, landing (outcomes moved out);
  reconfiguration by OCP resize (+ attach sequence diagram); the dissipative alternative; experimental method
  (twin, runner, verification ladder, test references, metrics; parameters to Appendix B).

4 Results and Discussion (19.5 pp), each section opening with its question, baseline and bar: 4.1 carrying with a
  fixed team (rig and twin side by side; fig:flight; tab:traj on the default controller; rig flights + a rig figure;
  thrust identification once + a fit figure; landing; computation; twin validity; literature context); 4.2 steady
  accuracy (IC box, breakaway cap rig and twin, external forces, model errors, estimators summary, tilt vs height,
  a source -> signature -> remedy table); 4.3 detach (layout + support polygon, OCP-resize transient, vs network
  hand-off, rig detach); 4.4 attach and the rejoin (OCP resize vs network attach, fold-in and approach, the rejoin
  mission storyboard, errors during an attach); 4.5 trajectory-based vs dissipative (RQ2); 4.6 discussion (answers
  in question order against the literature; geometry decides; the steady-error taxonomy; twin and rig; limitations).

5 Conclusion (1.5 pp): one paragraph per question, a value statement, research-level future work.

Appendices: A software; B parameters and weights (one table, twin and rig columns); C rig operations and log (tab:rig_days,
  arming, preflight); D negative results (+ tab:methods detail, the weld ladder, ground join); E timeline.

## Cuts (no claim lost), ~11-13 pp

existing.tex (merge into control.tex), modelling.tex:5-67 (single quadrotor), the spring-damper equation, forced
`\newpage`s, fig:cost_scaling, the RL and hybrid paragraphs to one sentence each, control.tex:192-259 (stale
architecture), intro.tex:15 repeat, contributions.tex:60-64, system_overview.tex:52-60 duplicates, the method's
result numbers (to chapter 4), tab:weights and tab:params (Appendix B), arming/preflight (Appendix C),
simulation.tex:23-33 (to the discussion), metrics.tex:30-32 (unused metric), sim_results.tex:9-19, the tab:methods
detail (Appendix D), sim_attach.tex ground join (Appendix D), rig diary (Appendix C), discussion/conclusion repeats,
Appendices.tex:1-165 (dead comments), unused files and images.

## Claims with no figure or table behind them (figure candidates)

Solve time vs N (measured, N = 2-6); support-polygon plan views per layout with the tilt; attach sequence diagram and
rejoin storyboard; rig photographs; thrust map fit (throttle vs mass, voltage colour, residuals); floor-start sketch
and lambda vs elevation; breakaway cap (1 Oct ring height vs time, twin overlay); tilt vs height; -x drift plan view;
rig orbit and figure-eight; force-step response with the first-order fit; IC box error vs distance; an artefact ->
symptom -> fix table; the source -> signature -> remedy table; replay variants table; network settling trace;
**a numeric "this work vs TU Delft / Quan / Bacheti" table**.

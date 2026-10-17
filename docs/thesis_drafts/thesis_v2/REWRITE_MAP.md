# Rewrite map: thesis_v2 to thesis_v3_full and thesis_v3_50 (17 Oct 2026)

Brief: `~/multi_drone_control/docs/thesis_rewrite_brief_2026-10-17.md`. This map is the plan for both versions: one
row per section of thesis_v2, with what it claims now, whether that claim is current, and the action.

Sources of "current" (all in `~/multi_drone_control`, read-only):
- the master map `docs/analysis/sim_results_inventory_2026-10-16.md` (labels CURRENT / DRIFT / STALE-CFG / STALE-MECH /
  LEGACY; ranks 1-12; Wesley's 12 decisions of 16 Oct);
- `docs/analysis/appendix_refly_2026-10-17.md` (54/54 S pass; C-D2, C-D3, C-D5 falsified; C-L1 not supported; the
  reviewer's must-not-say list overrides the sections above it);
- the cards: carry `2026-10-16_carry_speed_envelope` (39/40, spin 0.3 out); corner cutting `2026-10-16_corner_cutting`
  (closed 17 Oct: defaults stay, lag reported); Tier A `2026-10-12_tier_a_crossover` rev 3 (T3, T8, T9 no sign, K1
  unresolved, T15 narrow); Tier B `2026-10-16_tier_b_support_margin` (B1-B3, B5 confirmed; B4 falsified as written);
  W4/W5 freeze `docs/rig/freeze_2026-10-14.md` with `2026-10-09_departed_disarm_alone` (F1, P1-P4); drop and land
  `2026-10-08_drop_and_land` v3 (D1, D4 supported); LAND `2026-10-17_touchdown_height_gate` (A: twin default) and
  `2026-10-17_land_ref_below_rest` (rev 3 flying tonight, not thesis evidence); rig attach
  `2026-10-15_attach_comotion_rig`; N2 bands `2026-10-16_bands_1m` (frozen 17 Oct ~19:54);
- `docs/analysis/thesis_tables_T10_T13_2026-10-16.md` and the generated `docs/figures/tables/T10_switch.tex`,
  `T13_parity.tex`; figures `docs/figures/F10b_crossover`, `F12_levers`;
- the clock-free carry rescore (`tools/corner_metrics.py R1847-R1991`, 16 Oct; the 0.3 m/s base rows are in the
  corner-cutting card, the rest only in that session's scratchpad `rescore_1847_1991.md`);
- the registry for the rig rows of 14-15 Oct (RIG-1014-t5_2, RIG-1015-a1, a2_2, a3_2) and the exploratory
  `cable_source model` pair R2084-R2087;
- `docs/decisions.md` (the latest DECIDED lines win).

## Page counts of thesis_v2 (built 12 Oct, 122 pp)

Main matter (Introduction to the end of the Conclusion): pp 1-75 in the arabic numbering. References pp 76-86,
appendices pp 87 onwards. The template (title_page.tex, abstract comment) limits the report to 50 pages "excluding
appendices, bibliography, cover sheet, abstract, acknowledgements, nomenclature, contents and lists of figures and
tables", so the 50 counts the main matter.

| Chapter (v2 pages) | v2 pp | Plan of 11 Oct (pp) |
|---|---|---|
| 1 Introduction, RQs, contributions, outline | 1-5 (4.5) | 3.0 |
| 2 Literature | 5-14 (9.5) | 8.5 |
| 3 Methodology | 15-37 (23) | 12.0 |
| 4.1 Carry incl. steady error and the carry skeleton | 38-50 (12.5) | 3.5 |
| 4.2 Leave incl. skeleton | 50-59 (9) | 8.0 |
| 4.3 Join incl. skeleton | 59-67 (8) | 6.5 |
| 4.4 Ledger | 67-68 (1.5) | 2.5 |
| 4.5 Discussion | 68-73 (5.5) | 3.5 |
| 5 Conclusion | 74-75 (2) | 1.5 |

## Section by section

Words: uncommented words in the file (`sed 's/^\s*%.*//' | wc -w`). Actions: keep / rewrite / condense / to appendix
(App) / cut. "Full" is thesis_v3_full, "50" is thesis_v3_50.

### Front matter
| File | Words | Claims now | Current? | Full | 50 |
|---|---|---|---|---|---|
| title_page.tex (abstract, ack., nomenclature) | 346 | abstract: 4 UAVs within 3 cm, 4 detaches, "in simulation the ring settled within a second of a detach" | the last clause STALE (old detach 0.9-1.0 s; freeze A1 back in band 3.1-3.2 s) | rewrite the abstract (<=100 words) on current results | same |
| nomenclature.tex | 9 | placeholder entries | placeholder | leave (Wesley's) and list in CHANGES | same |

### 1 Introduction (intro.tex 864, contributions.tex 1253)
| Part | Claims now | Current? | Full | 50 |
|---|---|---|---|---|
| Motivation, mission profile figure | context; Quan/Zeng/Bacheti facts | current (11 Oct full-text reads) | keep; cut the repeat of the membership facts (they are in 2.2) | condense to ~1.5 pp |
| RQs, tab:rq_map | RQ1-RQ3; RQ1 lists "pushes, wrong model values and the breakaway cap in the twin" | spine current; RQ1 text must say the measured pull now absorbs vertical errors | keep; update RQ1 experiments row and RQ2/RQ3 rows | keep |
| Contributions C1-C3, M1-M2 | C1: rig 3 cm; C2: "six commanded releases ... five carried, 4.6-17.8 deg"; C3: placeholders | C1 add the twin envelope (39/40); C2 add Tier A/B; C3 still placeholder; "dissipative" wording in the RQ paragraph | rewrite each to one testable sentence with its evidence section | same, shorter |
| Outline | | | rewrite to the new structure | same |

### 2 Literature (Literature_review.tex 330, lit_* 4.2k)
| File | Words | Claims | Current? | Full | 50 |
|---|---|---|---|---|---|
| Literature_review.tex | 330 | roadmap; the gap | current | keep; gap paragraph says "admission contract" and "ledger" | keep |
| lit_transport.tex | 921 | link models, centralised, distributed (Quan "dissipative control") | "dissipative" wording; "Why the network is not an RQ" repeats App F and 4 intro scope | replace "dissipative" with "passivity-based network"; merge the network paragraph into two sentences | condense to ~1 pp |
| lit_membership.tex + tab:membership | 1559 | the core comparison; "This work" row has 8 Oct numbers and [14 Oct] placeholders | literature current; our row stale | keep; our row: rig 4-to-3 releases from T8, twin levers, join "[join tree]"; bracketed cells for unread papers stay and are listed in CHANGES | keep the table; cut the plug-and-play and statics prose by half |
| lit_capture.tex | 590 | proof of a catch, docking, magnets | current; "contact and the lift" wording | keep; proof from co-motion | condense to ~0.5 pp |
| lit_switching.tex | 518 | bumpless transfer, RTI, Simplex | current | keep | condense |
| lit_execution.tex | 362 | INDI, measured link force, offset-free MPC | current | keep | keep |
| lit_sim.tex | 254 | SIL practice, RotorTM per-axis, the ledger | current | keep | condense |

### 3 Methodology (23 pp in v2)
| File | Words | Claims | Current? | Full | 50 |
|---|---|---|---|---|---|
| system_overview.tex | 1198 | rig, twin, roles; plates counted ANTICLOCKWISE; tab:plates; twin cannot fly with latency | plate numbering WRONG (locked 14 Oct: clockwise, angle = 30 x plate, carriers 3/11/7, QUAD4 9); latency: 15-30 ms extra fine, 45 ms tips at takeoff (exploratory R1794-R1799) | rewrite numbering (drop tab:plates, one sentence on the collaborator's CCW count); twin paragraph: latency line updated as exploratory | condense ~1.5 pp |
| fig_geometry.tex | 347 | top view numbered CCW, carry layout 1/3/5/9, side view at 45 deg | numbering and the 45 deg rod both stale (60 deg since 7 Oct) | redraw: plates numbered clockwise (carrying 3, 7, 9, 11; leaver 9), side view at 60 deg | same |
| model.tex | 631 | model, 13+14N states, fig:payload_system | current | keep | keep; fig:payload_system to App (fig:geometry carries the symbols) |
| controller.tex: planner | ~900 | OCP, refs, outputs; "[the measured solve times]" | current | keep; fill the bracket from fig:solve_time | keep, condense |
| controller.tex: tracker, measured pull | ~500 | 17-state MPC, affine map, measured pull | current | keep | keep |
| controller.tex: model integral | ~800 | d as a lumped force; convergence derivation; "learns the applied force"; "removes a force in all three axes"; bound and limits | derivation current as a method; the claims built on it are FALSIFIED vertically on the freeze stack (C-D2, C-D3, C-D5) | keep the method; rewrite "Properties"/"Bound and limits" so they do not claim vertical learning; state that the measured pull already takes up vertical forces at each UAV | 0.5 pp in main; derivation and reference-shift form to App |
| takeoff_landing.tex | 460 | floor start, lift, LAND, touchdown "for 1 s after 5 cm" | current except LAND: the touchdown re-arm at ring touchdown (land_rearm_touchdown) is the twin default since 17 Oct; rig after the critic | keep; one paragraph on the false touchdown and the re-arm (limitation in 4.5) | keep 0.5 pp |
| reconfiguration.tex | 3011 | notice/re-plan/redistribute/fall back; statics; support margin; leaver rule; contact-and-lift proof; tab:constants; fig:reconfig_states | statics and margin CURRENT (now backed by Tier B); the join notice describes the OLD contact-and-lift proof (co-motion is the default since 14 Oct, eacab58); angles CCW | rewrite the join notice and the admission contract to the co-motion proof; constants table to the co-motion values; angles to plates | 3 pp: keep statics, margin, leaver rule, contract; constants table and the ramp/seed detail to App |
| authority.tex | 1190 | owners figure, envelope, fall-backs, what is tested where (rig switch states "[...]") | rig states stale: drop_on_loss on in every mode since 14 Oct; drop_takes_back on at the rig via params file; the drop rule FIRED on the rig on 15 Oct | rewrite the "tested where" paragraph to the 17 Oct state | condense to 1.25 pp |
| simulation.tex (protocol) | 675 | cards, frozen bars, one tree, critic/reviewer as LLM agents, the AI disclosure placeholder | current | keep (AI disclosure kept: decision for Wesley) | 0.5 pp; detail to App |
| trajectories.tex | 326 | the six references; "results use orbit 0.125 and fig-8 a = 1.0 at 0.25" | the carry claim flies five shapes at 0.125/0.2/0.3 with radius 0.5 and line 1.0 | rewrite the sizes; "peak path speed" wording (carry reviewer) | condense |
| metrics.tex | 1306 | height/horizontal error, disturbance, release metrics (signed freeze), join metrics, magnet margin, switch validation, resolution rule, why these bars | current; add the clock-free carry ruler (cut, across-path, lag; Wesley 16 Oct) and say the frozen RMSE scores against the 10 Hz held reference | keep + add | 1 pp in main; the full release-metric text to App |

### 4 Results and discussion
| File | Words | Claims | Current? | Full | 50 |
|---|---|---|---|---|---|
| Results_and_Discussion.tex | 173 | roadmap; the network scope paragraph | "dissipative" wording | rewrite (scope in one sentence, App F) | same |
| res_carry.tex: takeoff, fig:flight | ~200 | rods hold 52 deg vs planned 45 | STALE (4 Oct, 45 deg) | regenerated figure on the freeze stack (R2034, 60 deg) | keep |
| res_carry.tex: trajectories (tab:traj ref) | ~120 | "about a centimetre, under 3 cm" with the integral off, 3-4 UAVs | STALE-CFG (C1) | replace by the carry claim T7t (twin, 5 shapes x 3 speeds + 1 m) with the frozen bars and the clock-free columns; fig:traj regenerated (orbit and figure-8 at 0.125) | T7t condensed (0.3 m/s detail and 1 m arm to App) |
| res_carry.tex: rig flights, tab:rig_oct7 | ~250 | 7 Oct circle and fig-8 before/after: 8.3 -> 2.9 cm, 8.2 -> 3.0 cm | current (rig, two changes at once) | merge into T7 (skel_carry) with K1/K2/K5/K6 on the reviewer's window; K3/K4/K7 placeholders (21-22 Oct) | keep T7 + fig:F8 |
| res_carry.tex: thrust identification | ~200 | the affine map, offsets 0.181-0.187, free hover within 3.7 cm | current | keep | keep 0.5 pp |
| res_carry.tex: landing, fig:land_phases | ~350 | "none of 18 tipped"; LAND every phase 33 of 34 | land table re-flown: 82 landings, but 4 false touchdowns (C-L1 not supported); land_phases DRIFT/STALE-MECH (9 Oct) | rewrite: the freeze-tree landing table with the false touchdown counted; one paragraph on the touchdown re-arm; fig:land_phases to App (labelled 9 Oct tree) | one paragraph + limitation |
| res_carry.tex: computation, fig:solve_time | ~150 | rig 7-8 Oct solve times | current; "dissipative mode" in the caption | keep; caption "detach mode" | keep |
| res_carry.tex: twin vs rig, tab:sim_vs_rig, fig:twin_rig_parity | ~400 | hover twin vs rig (before the tracker fix) | tab:sim_vs_rig STALE (F3): cut; parity figure CURRENT as agreement | cut the table; parity figure moves to 4.4 | same |
| res_steady.tex: tracker IC, fig:tracker_x0, tab:x0 | ~450 | box raises the hover 4.8 cm; pinning cuts the circle error | C-X1 confirmed LARGER (+13.1 cm), C-X2 confirmed; the figure's hold panel clips the new trace | rewrite with the freeze numbers; table in the text, figure cut until rebuilt; must not call +13 cm a tracking error | 3 sentences + App table |
| res_steady.tex: breakaway cap | ~250 | 1 Oct rig held low; cap release since 7 Oct | current (rig history) | keep, shorter | 4 sentences |
| res_steady.tex: external forces, fig:disturbance, tab:disturbance, tab:gain | ~600 | vertical push leaves 8.4 cm without integral, integral learns 0.58 N, K from stiffness, gain trade-off, thrust-map moment | FALSIFIED vertically: the measured pull absorbs 0.6 N in ~1 s (0.0 cm steady, peak 1.4 cm); with the planned pull it leaves -7.95 cm (exploratory pair); horizontal 4.8 cm -> 0.0 with the model integral (0.32 of 0.40 N); thrust-map error 0.0 cm / 0.1 deg both forms; gain/gate unresolved | rewrite around the measured pull; regenerated tables; fig:disturbance regenerated (horizontal push) | rewrite, 0.75 pp; tables to App |
| res_steady.tex: rig -x pull | ~200 | 0.41-0.52 N toward -x on 1 Oct | current (rig finding) | keep | 3 sentences |
| res_steady.tex: rig integral + measured pull, fig:rig_integral | ~250 | 7 Oct: integral learned -0.9 N on the planned pull, -0.1 N on the measured | current (rig); consistent with the twin finding | keep; it now leads the steady-error story | keep |
| res_steady.tex: wrong model values (tab:robust in App) | ~150 | 1.25 cm per % of mass; +20 % over the gate 26 cm; rods 8 cm | FALSIFIED: mass +-20 % leaves 0.0-0.1 cm with or without the integral (measured pull); rods not measured +17.1 cm; radius +2 cm +0.9 cm with the integral winding to 0.76 N (not learned) | rewrite; relabel the learned-force column | 3 sentences |
| res_steady.tex: estimators, tab:methods | ~200 | earlier-stack comparison (box 0.025) | STALE-CFG; history | to App D as dated history; one sentence in main | same |
| res_steady.tex: tilt and height | ~100 | rig 1 Oct tilt falls with height | current (rig) | keep | one sentence |
| res_steady.tex: tab:error_sources, answer | ~300 | four kinds of source; integral removes forces in all three axes | the "all three axes" and mass rows FALSIFIED as written | rewrite the table rows and the answer | keep the table |
| skel_carry.tex (T7, fig:F8, falsifier) | 664 | K1/K2/K5/K6 filled; K3/K4/K7 pending; the 60 deg falsifier | current; K4 never flew | fold into 4.1 | same |
| sim_detach.tex: layout redo, the transient, fig:detach | ~500 | old resize 2.8-4.0 deg, back in band 0.9-1.0 s | STALE-CFG (D6); layout study LEGACY (D10, superseded by Tier B) | replace by the freeze-tree A1 release (fig:detach regenerated, back in band 3.1-3.2 s); layout redo cut (Tier B answers it) | fig:detach to App, T3 carries it |
| sim_detach.tex: rig detach, tab:rig_oct8, fig:rig_detach | ~350 | 8 Oct four flights | current; tab:rig_oct8 is superseded by T8 (skel_leave) | merge into T8a/T8b; keep fig:rig_detach | keep T8b + fig:F10 |
| sim_detach.tex: circle detach, fig:rig_circle_detach | ~150 | | current; figure prints CCW angles | keep; caption maps the printed angles to plates; figure to regenerate | keep |
| sim_detach.tex: constant offset + redo | ~150 | 5-6.5 cm offset on the reference integral | current (rig); redo box stale (14 Oct did not fly W4/W5) | keep; redo -> placeholder for the next rig day | condense |
| sim_detach.tex: freed UAV, fig:departed_landing, fig:rig_departed (pending), fig:disarm_lanes | ~350 | disarm alone | DRIFT; W4/W5 freeze P1-P4 confirmed (0.27-0.43 s); disarm_lanes prints "rig, 14 October: pending" | keep; numbers from the freeze; pending rig figure sized for 21-22 Oct | departed_landing kept, disarm_lanes to App |
| sim_detach.tex: fewer than three + redo | ~80 | | redo stale | keep the rule, cut the redo | one sentence |
| skel_leave.tex: T8a/T8b, no-event floor, F11, F10, F9 (pending), F12 (pending), T10 (empty) | 1874 | the rig release record L1-L7; L8/L9 pending | L1-L7 current (P6 reviewer); add the 14-15 Oct uncommanded releases (t5_2, a2_2) as rig findings; F12 and T10 now exist; F9 never built | fold in; F10b (Tier A T3), F12 (Tier B and the pairs), T10 (generated), T9 lever table to App | keep T8b, F10b, F12, T10 |
| sim_attach.tex | 815 | redo boxes (legacy attach, rejoin, errors during attach); the tip loop (fig:tip_feedback) | redo content LEGACY (A8); tip loop DRIFT, twin-only (A6) | cut the redo boxes (one line of dated history in App); tip loop to App D with the twin's own approach | same |
| skel_join.tex | 805 | T11 (contact-and-lift, 12 Oct), F14 (contact-and-lift), F15/T12 pending | T11 and F14 STALE-MECH (co-motion proof since 14 Oct; decision: re-fly on the join tree after 21-22 Oct) | T11 and F14 become placeholders sized for the co-motion re-fly; rig attach record so far (8 flights, one contact, no fold-in) as exploratory rig findings; fault coverage from current cards | same, shorter |
| ledger.tex | 588 | band rule, 14 Oct bands | current; the 14 Oct flights flew no W4/W5 | keep; T13 generated; N2 1 m bands (frozen 17 Oct) summarised, full table to App | keep 2 pp |
| skel_ledger.tex | 255 | T13 empty, F16 pending | T13 now generated (rig cells "not yet flown") | fold in | same |
| discussion.tex | 1872 | RQ1-RQ3 discussion, geometry, sim and rig, literature table, limitations | RQ1 steady-error paragraph FALSIFIED in part (integral in all axes; 7.1 N/m stiffness); geometry redo now answered by Tier B; literature table this-work row stale; limitations stale | rewrite in question order; literature table updated; limitations rewritten | 3.5 pp |
| skel_discussion.tex | 196 | feedforward vs feedback, magnet margin, design rules (placeholders) | the 1-DOF signs are now scored (T3, T8 confirmed; T9 no sign; T2/T4 not flown) | fold in | same |
| Conclusion.tex | 837 | answers, value added, future work | detach in sim "back within its hover band"; the attach claims | rewrite | 1.5 pp |

### Appendices (Appendices.tex 5885 + app_network_* 2.4k)
| Section | Current? | Full | 50 |
|---|---|---|---|
| A Software | package names; "dissipative" | keep; the planner's detach mode named without the package | same |
| B Parameters and weights | 10 Oct values; the network stiffness paragraph | update (rods 60 deg, land_rearm_touchdown twin only, the co-motion constants); cut the network paragraph | same |
| C Rig operations and log | rig days to 1 Oct | add 7, 8, 14, 15 Oct rows; keep the 1 Oct table | same |
| D Negative results, further tests | tab:methods (earlier stack), tab:traj, tab:x0, tab:gain, tab:detach, fig:traj, fig:landing, tab:land, tab:solve_times, tab:robust, weld ladder (redo), proximity weld, ground join (redo) | tab:traj and tab:detach replaced (T7t, T3); tab:solve_times (twin, load-bound) cut; fig:landing cut (overlapping labels in the re-render; the table carries the claim); tab:x0, tab:gain, tab:disturbance, tab:robust, tab:land regenerated on the freeze tree with the reviewer's corrections; tab:methods kept as dated history; redo boxes become one-line history | same |
| E Timeline | to 4 Oct | extend to 17 Oct | same |
| F The network on the earlier plant (4.4k words) | LEGACY (D9) | one page of dated history, no "dissipative" wording | half a page |
| New: lever table T9, fault coverage, the 1 m bands, LAND in every phase, the tip loop | | add | same |

## Repetition found (and where it goes)

- The membership facts (Quan, Zeng, Bacheti) appear in intro.tex, lit_membership.tex, discussion.tex and
  tab:lit_compare: keep them in 2.2 and the discussion's table; the introduction keeps one sentence.
- The network question: contributions.tex, lit_transport.tex ("Why the network ..."), Results_and_Discussion.tex
  ("Scope"), App F intro, Conclusion comments: one sentence in 1.1 and App F.
- The controller described twice: the planner in model.tex/controller.tex and again in App A/B; the takeoff in
  takeoff_landing.tex and App C.2: the main text keeps the summary, the appendix the detail (as now), no third copy.
- The magnet-loss sentence ("every ring lost on the rig began with a magnet letting go") appears in lit_capture,
  metrics, contributions, sim_detach, discussion: say it once in 4.2 with the evidence; elsewhere point to it.
- The 30 deg drop rule is defined in authority.tex, reconfiguration.tex (tab:constants) and metrics.tex: define once
  in 3.5.
- The release metrics: metrics.tex and the T8b/T13 captions repeat the definition: captions point to 3.6.
- Project_plan.tex (2.1k words) and Project_preparations.tex (520): NOT input by main.tex in v2 (out of the build
  already). Their aims and experiments E1-E4 are superseded by the RQ spine. Not copied into v3 (decision for Wesley).
- Unused files in v2: tables/four.tex, tables/params.tex (not input); Images not referenced.

## Rules applied in both versions

- No run ids, flight names, SHAs or branch names in the text (LaTeX comments keep provenance).
- No "dissipative" wording: "our planner", "detach mode", "the network controller (earlier plant)".
- Plates clockwise from the ring's +x, angle = 30 x plate: carriers 3/11/7, QUAD4 9. Old CCW plate p is CW (12 - p) mod
  12: 1/3/5/9 -> 11/9/7/3; CCW azimuths 30/90/150/180/270 deg are plates 11/9/7/6/3. Figures that print CCW angles
  get a caption mapping and go on the regenerate list.
- Numbers in tables; a claim stated once with its reviewer caveats; the cards' must-not-say lists honoured.
- Exploratory results only as exploratory.

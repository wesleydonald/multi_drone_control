# Thesis audit, Fri 30 Oct 2026

Read-only audit of the thesis for the 20 Nov hand-in (results freeze Sat 7 Nov; lab days left Wed 4, Thu 5, Fri 6 Nov if
added). No thesis file was edited anywhere and nothing was committed. Three sections were drafted as new files (section 8).
Effort: **S** up to 2 h of writing, **M** half a day to a day, **L** more than a day. Flying time is not counted.

**Verdict.** The front half is solid: a one-page introduction, a 15-page literature review with figures and a gap table
(revised after the supervisor's first read), and a 15-page methodology. The back half is thin: the results chapter is
five figures with no text, the conclusion is blank, the abstract dates from 4 Oct, and the nomenclature is the template's.
Most result figures and every real-world release are on the 0.55 m cables and the 0.86 kg ring, while the methodology's
photos and every flight since 28 Oct are on 1 m cables and the 1.15 kg ring. The real-world join, the thesis's headline
gap, has not flown yet.

## 0. Which copy is newest

| Copy | State | Use it for |
|---|---|---|
| Overleaf, `origin/main` 9eea11f (fetched 30 Oct 12:01) | **Newest master.** Last commit 28 Oct 20:37 (platform photos); Wesley's own Overleaf edits of 27-28 Oct are in it (intro, lit review wording). Builds clean: 53 pp., 0 undefined references | intro, lit review, methodology, the results skeleton |
| `~/thesis` (local clone) | Stale: HEAD 52f3866 of 6 Aug, **29 commits behind** origin (not 18), with local edits to 7 files (`Project_plan`, `modelling`, `control`, `intro`, `main.tex` ...). Not touched | nothing |
| `~/mdc_wt/thesis/.../thesis_v3_full` | The 17 Oct full rewrite (117 pp.) plus the six unreviewed 23 Oct drafts (`drafts_1023/`). Its lit review and methodology are older than the master's; its results, discussion, conclusion and appendices are the newest text of those parts | the source for writing the results, discussion and conclusion |
| `~/thesis_snapshots/overleaf_rev6_2026-10-28.pdf` | 53 pp., 28 Oct 09:43, before the photos | reading only |

## 1. Status by chapter and section (master, page numbers as built)

| Section | Pages | Status | Waits on | Effort | Notes |
|---|---|---|---|---|---|
| Title, originality | i | final | - | - | |
| Abstract | ii | **stale** (4 Oct) | the results freeze | S | 0.86 kg ring, "outperforming a dissipative alternative" (the comparison is not claim-ready, and the word is retired), "three UAVs carried the ring to 1.0 m". Two hard-wrapped lines |
| Acknowledgements | iii | **stub** ("Mitch, Te-Jen, Mohammad.") | Wesley | S | |
| Nomenclature | front | **stub** (template placeholders A, C_p, D, alpha, TLA; `\nompreamble` still the template's sentence) | - | done as a draft | `drafts_1030/nomenclature.tex`; needs makeindex on Overleaf |
| 1 Introduction | 1 | drafted, near final (28 Oct) | - | S | One question, no contributions list. 2.7 calls it "the research questions of the introduction" (plural) |
| 2.1 Transport and link models | 2-4 | final, pending the supervisor's second read | - | - | |
| 2.2 Changes of team membership | 4-9 | final, pending | the results (its table's "this thesis" row: "0.86 kg ring") | S | |
| 2.3 Capture proof and docking | 9-11 | final, pending | - | - | |
| 2.4 Switching and runtime assurance | 11-12 | final, pending | - | - | |
| 2.5 Execution layer | 12-13 | final, pending | - | - | |
| 2.6 Simulation to reality | 13-14 | final, pending | - | - | |
| 2.7 The research gaps | 14-16 | drafted | the results: G1 (join in simulation only), G5 (overclaims, section 5) | S | |
| 3.1.1 The experimental platform | 17-19 | drafted, **numbers stale** (section 2) | the magnet's hold against the pull angle (a bracketed placeholder in the text) | S | the photos (28 Oct) show the 1.15 kg ring on 1 m cables in the square; the text says 0.55 m, 0.86 kg, carriers 3/7/11 |
| 3.1.2 Simulation | 19-20 | drafted | - | S | add the slope velocity filter and the 30 Oct finding (the motion-capture path ~95 ms late with 10 bodies streamed) to the left-out effects |
| 3.2 Model and planner | 20-22 | final candidate | - | - | notation collisions (section 8) |
| 3.3 Tracker, thrust map, pull, integral | 22-24 | drafted, two facts stale | - | S | cap 12 -> 14.5 m/s^2 (21 Oct); the rig's 5.3 Hz band-stop on the measured pull (29 Oct) and the velocity estimate (slope, 28 Oct) are not described. Two hard-wrapped paragraphs |
| 3.4 Reconfiguration | 24-28 | drafted, **stale** | the attach-gate decisions (OPEN 30 Oct: G, K, give-up 120 s), the chain's scope | M | margin numbers and the 1.6 N seed share are for r 0.225 / 0.86 kg; the drop rule omits `drop_gap_deg` (a T or opposite pair flies on); the magnet-marker release check is not described |
| 3.5.1 Protocol | 28-29 | final candidate | - | - | |
| 3.5.2 Test references | 29 | **stale** | the rescope's case list | done as a draft | `drafts_1030/test_cases.tex` (replacement) |
| 3.5.3 Metrics | 29-31 | final candidate | - | - | |
| 3.6 Studies in simulation only | - | **missing** (the methods behind three results figures) | - | done as a draft | `drafts_1030/sim_studies.tex` |
| Method of the comparison (Quan et al.) | - | **missing** | the scope decision and C's bars | M | only if C stays |
| 4.1 Carrying with a fixed team | 32 | **stub** (one figure) | the 1 m / 1.15 kg baselines (Wed 4 to Fri 6 Nov; blocked by the 30 Oct heave until the reduced motion-capture stream is flown); the pair's ring (OPEN 29 Oct) | L | sources: v3_full `res_carry`, `res_steady` (0.55 m), `drafts_1023/res_refly_1m` |
| 4.2 Reconfiguration | 32-33 | **stub** (two figures) | a real-world join (none yet); the chain's scope; U's reviewer and final-tree re-fly | L | sources: v3_full `res_leave` (8 Oct releases), `res_join`, `drafts_1023/res_rejoin`, `res_catch_offset` |
| 4.3 Scalability | 33-34 | **stub** (tick figure, four snapshots) | the N4 window decision (open in `res_scale`) | M | source: `drafts_1023/res_scale` |
| Comparison with Quan et al. | - | **missing** (draft superseded) | Wesley: which arm, bars (proposed, not frozen) | L | section 3 |
| Simulation against the real world (G5) | - | **missing** | bands for any 1 m / 1.15 kg cell | M | source: v3_full `ledger.tex` (17 Oct, says "none scored": now 11 are) |
| Discussion | - | **missing** in the master | all results | M-L | source: v3_full `discussion.tex` (4.2k words, 17 Oct, 0.55 m) |
| 5 Conclusion | 35 | **blank** | all results | M | source: v3_full `Conclusion.tex` (17 Oct) |
| References | 36- | final | - | - | 0 undefined; several near-duplicate keys (`...2024`/`...2024a` for Wang, Wahba, Gimenez, Tasooji x3, Barakou, Nathan): check none is cited twice |
| Appendices | - | **not compiled** | Wesley: keep or drop | M | Methodology says the parameters are "listed in an appendix", which does not print. v3_full's appendix (17 Oct, 8.2k words) is the newer source |

**Pages.** Main text 35 pp. (introduction 1, literature 15, methodology 15, results 3, conclusion 0). With the three
drafts the methodology is 18 pp., above the school's 10-15. A results chapter of 15-20 pp. and a discussion and conclusion
of 4-6 pp. would put the main text at about 55-60 pp. against the 50-page limit (reported only: not enforced yet).

**Style lint of the compiled chapters:** no "rig"/"twin" in text, no semicolons in prose, every float `[H]`, no bold
leads. Hard wraps: two paragraphs in `controller.tex` and the abstract. The 23 Oct drafts break every one of these rules
(rig/twin throughout, `[htbp]`, long captions, wrapped lines, semicolons), so each needs a style pass on the way in.

## 2. Facts in the compiled text that are now wrong

| Where | The text says | Now | Source |
|---|---|---|---|
| 3.1.1, `fig:geometry` | cable 0.55 m, UAV 0.55 kg, cable and magnet 0.075 kg | cables 1.0 m (22 Oct); UAV 0.583 kg with pack, cable and magnet; re-weigh the 1 m cable and magnet | `real.yaml` |
| 3.1.1, `fig:geometry` | ring 0.86 kg, 0.51 m, plates at r 0.225, magnets hold 9-10 N | ring 1.15 kg, 510 mm, magnets at r 0.204, slots ~100 x 85 mm, plate faces ~25 mm over the pivot (estimate); it bends at 5.3 Hz; a net in its centre (photo) | decisions 28 Oct; RIG-1028-payload |
| 3.1.1 | carriers on plates 3, 7, 11 and the fourth on 9 | still the attach layout; the baselines fly 0/6, a triangle (3/7/11 now, 0/4/8 on 0.55 m) and 0/3/6/9, as the photo shows | decisions 21 Oct; Fri 30 sheet |
| 3.3.1 | measured pull "capped at 12 m/s^2" | 14.5 m/s^2 (8 N) since 21 Oct | decisions 21 Oct (3) |
| 3.3.1 | no filter on the measured pull | the rig passes it through a 5.3 Hz band-stop (Q 1); the simulation does not | decisions 29 Oct |
| 3.3.1 | velocity estimate not described | the carriers' velocity filter is `slope` in both since 28 Oct (it halved the pair's swing) | decisions 28 Oct |
| 3.4.3 | margins 112, 86, 58, 29, 0 mm at r 0.225; the even triangle 112 mm | at r 0.204: 102, 78, 53, 27, 0 mm; the triangle 102 mm | eq. (margin) |
| 3.4.2 | fewer than three survivors, or tilt over 30 deg, drops the ring | true by default; with `drop_gap_deg` 200 (the chain configs) a T or an opposite pair flies on, `min_survivors` 2, the tilt drop at 45 deg | decisions 21 Oct (4) |
| 3.4.1 | a leave is noticed by the apparent cable stretch | also by the magnet-marker check (a switch, rev 3), the only cue for a zero-share carrier's release | decisions 21 Oct; plan WP4 |
| Table `tab:constants` | seed 0.1 N "against its balanced share (1.6 N on plate 9)"; contact radius 24 mm | recompute the share at 1.15 kg; contact radius, co-motion constants and the give-up are OPEN for the rig (G, K 0.035, 120 s) | decisions OPEN 30 Oct |
| 3.5.2 | figure-eight swept by the eased angle; real-world figure-eight a = 1.0 m at 0.25 m/s; no spinny line, no hold | constant path speed with 4 s ramps (since 7 Oct); a = 0.75 m and the spinny line 0.75 m at 0.2 m/s; a 5 s hold before every path | `load_trajectory.py`; `real.yaml`; decisions 22 Oct |
| 3.5 intro | parameters "listed in an appendix" | the appendices do not compile | `main.tex` |

## 3. The 23 Oct drafts (`drafts_1023/`, index `~/mdc_wt/thesis/DRAFTS_2026-10-23.md`)

All six are unreviewed as thesis text, written before the 27-28 Oct style rules, and fly the 0.86 kg ring in simulation
with the `ema` velocity filter on the 23 Oct tree (be34f95 + patch, or 63b504f for S). Since then the defaults changed
(slope filter 28 Oct, the 1.15 kg payload, the Quan code behind a switch, Tejen's ports, the attach gates), so any claim
table that stays must be re-flown on the final tree before its reviewer (the working rule), as R and U already planned.

| Draft | What it covers | Numbers current? | To move it into the thesis | Effort |
|---|---|---|---|---|
| `res_refly_1m` (R) | every 0.55 m simulation claim table re-flown on 1 m rods: 27 claims, 19 hold, 5 do not, 2 unresolved, 1 not confirmed; release margin, pushes, gain | current for its tree (R2483-R2628, two reviewer passes, weak); stale against the final tree | decide if R stays (it re-validates old claims, not a new result: an appendix table may do); re-fly on the final tree; style pass; `tab:faults` row (G165 vs G150p) | L with the re-fly, S as an appendix table |
| `res_scale` (S) | the bench and the real-time limit (3 UAVs at 5 iterations, 4 at 2 or 1, a projection), how few iterations suffice, the 8-UAV chain 8 -> 4 -> 8 at 5 iterations, the 2-iteration check | **current** (no S runs after R2801; the rig keeps 5 iterations, decided 24 Oct, consistent) | cut 6.3k words to ~1.5k; the N4 window (missed as written, or a new window for later runs: Wesley); figures already in the master | M |
| `res_rejoin` | a departed carrier flies back to its plate and is folded in, simulation only (E0-E5; E5 toward the plate 1 of 2 clean) | current (the code used by S since) | style pass, shorten; `fig_rejoin` (R2594) from the review folder; a reviewer | S-M |
| `res_catch_offset` (U) | the magnet caught off its slot, measured catch point against the slot, along the rim and radially in | the six owed runs **flew** (R2727-R2732, 23 Oct 17:45): verdict still "between"; measured arm 8 cm along the rim, 20 cm radially in; slot arm 4 / 6 cm. The draft's brackets are unfilled | fill the brackets; reviewer; re-fly whole on the final tree (decided 23 Oct); its figure is in the master | M |
| `res_compare` (C) | our planner against our port of Quan et al.'s network, hover and lines at 0.2 / 0.4 m/s | **partly superseded**: our planner's pending cells flew (R2733-R2737); the comparison moved to the law as published (card `2026-10-23_quan_exact`, world C, R2744-R2818, R6c split, R7c) with exploratory results only and bars **not frozen** (Wesley: after videos and accuracy figures) | Wesley picks the arm(s) and freezes bars; claim runs (3 per cell per the 21 Oct spec) on the final tree; rewrite | L |
| `res_pair_swing` | the pair's 1.4 Hz swing on the 22 Oct rig, the timing table, simulation rounds A-D | stale: the 28 Oct pair flights decided the slope filter (hE1 against hS1); of the timing conflict it flags, the body-rate numbers are settled in the master (25-35 against 48-62 ms) but the simulation's throttle-to-lift still differs (15-25 ms in the master, 17-29 in the draft's table); since then the 5.3 Hz bending (29 Oct) and the 30 Oct heave (motion-capture delay) | reframe as "delay in the measured-pull loop" with the 28-30 Oct findings, or drop to an appendix; style pass | M |

Beside them, the 17 Oct v3_full results (`res_carry`, `res_steady`, `res_leave`, `res_join`, `ledger`, `discussion`,
`Conclusion`, `Appendices`) hold the 0.55 m real-world results (7-8 Oct carry and releases, 14-15 Oct attach attempts,
the thrust map, landing, computation) and are the main source for 4.1 and 4.2. `ledger.tex` says no row is scored: 11 are
now (section 5).

## 4. Figures

### 4a. In the master now

| Label | Where | Data | Status |
|---|---|---|---|
| `fig:mission_profile` | intro, TikZ | - | final |
| `fig:lit_configs`, `fig:lit_join`, `fig:lit_simplex`, `fig:lit_map` | lit review, TikZ | - | final |
| `fig:geometry` | 3.1.1, TikZ | - | **redraw** at 1 m and r 0.204 (or both payloads) |
| `fig:rig_photo` | 3.1.1, photos | 28 Oct, the 1.15 kg ring on 1 m cables | final; the text beside it describes the old setup |
| `fig:owners`, `fig:reconfig_states`, `fig:protocol` | 3.1.1, 3.4, 3.5.1, TikZ | - | final |
| `fig:res_traj` | 4.1, `Images/results/fig_traj.pdf` | real world, figure-eight: 2 carriers RIG-1022-f2slope, 3 RIG-1022-f3h (0.55 m, 0.86 kg, 0.2 m/s), 4 RIG-1007-r2008e60 (7 Oct tree, half the speed: not one variable) | **re-render** on the 1 m / 1.15 kg flights (RIG-1029-f4 flew the 2 Hz low-pass, not the notch; f3t / f4n / ls3t / ls4 not flown yet). Script `docs/figures/review_2026-10-23_best/fig_traj.py` reads `best_data.RIG_BEST` |
| `fig:res_release` | 4.2, `fig_release_rig.pdf` | RIG-1008-r303_4 (unannounced, 4 -> 3, 0.55 m) | keep: the only clean real-world release |
| `fig:res_catch` | 4.2, `fig_catch_offset_explained.pdf` | R2629-R2684, R2727-R2732 (0.86 kg, ema) | re-render after U's final-tree re-fly |
| `fig:res_tick` | 4.3, `fig_s_tick.pdf` | bench B, `results/analysis/s_bench_B_2026-10-24.json` | final |
| `fig:res_s8` | 4.3, four PNG snapshots | R2779 (exploratory, draft bars not frozen) | keep; caption must say exploratory or swap to a claim run (R2766/R2767 hover) |

### 4b. Needed, by results section (for the figure agent)

| Section | Figure (one message) | Status | Data |
|---|---|---|---|
| 4.1 | ring path on the figure-eight, 2 / 3 / 4 carriers, real world | exists on old data | re-render: Wed 4 to Fri 6 Nov flights |
| 4.1 | path error and tilt, hover / figure-eight / spinny line x 2 / 3 / 4, real world against simulation | candidate `review_2026-10-23_best/fig_summary*` | new flights + p115 simulation R2833-R2838 (exploratory) |
| 4.1 | the spinny line's yaw against its reference | **missing** | RIG-1022-ls3c (0.55 m), RIG-1028-ls2c (pair), 1 m flights to come |
| 4.1 | the ring's 5.3 Hz bending and the band-stop | **missing** | RIG-1029-h4 (none), -h4f (low-pass), -sq2_5 (notch); tracker `acm_z` |
| 4.1 | the affine thrust map | exists `docs/figures/F_thrust_map` (30 Sep) | still valid (locked map) |
| 4.1 | the model integral on the hardware | exists `F_rig_integral` (7 Oct, 0.55 m) | keep as is or drop |
| 4.1 | the planner's solve time on the hardware | exists `F_solve_time` (7-8 Oct) | add 29 Oct's 4-UAV ticks if it stays |
| 4.2 | every real-world release: tilt rise, detection, back in band | exists `F_rig_detach`, `F_rig_circle_detach`, v3_full `releases.pdf` (8 Oct) | as is |
| 4.2 | a release in simulation at 1 m | candidate `review_2026-10-23/fig_detach_1m` | R2486 |
| 4.2 | the join in simulation: contact, lift, confirmation, switch, fold-in (F14) | **missing** (`Images/attach_storyboard.pdf` is R0209, the 0.6 kg ring of Sep) | p115 floor joins R2843/R2844, R2906 (Tejen's controller, our confirmation) |
| 4.2 | the join in the real world (F15) | **missing**, no attach has completed | Wed 4 to Fri 6 Nov, if one does |
| 4.2 | a departed carrier rejoins | candidate `review_2026-10-23/fig_rejoin` | R2594 |
| 4.2 | catch offset | exists | as 4a |
| 4.2 | the release chain 4 -> 3 -> 2 -> 3 -> 4 | **missing**, no clean run (the square-to-T release drops the ring in simulation) | depends on the rescope |
| 4.2 | the pickup and the drop into the net | **missing** | simulation R2900, R2905; real world to come |
| 4.3 | tick against team size | exists | as 4a |
| 4.3 | the 8-UAV chain: team size and tilt against time | candidate `review_2026-10-23/fig_s_chain` (R2597, 1 iteration) | re-render on the claim runs R2766/R2767 (5 iterations) |
| 4.3 | plan quality against iterations at 8 UAVs | candidate `fig_s_quality_n8` | Q6 JSON |
| comparison | line tracking and yaw, our planner against Quan et al. | candidates `fig_c_line`, `fig_c_yaw`, `review_2026-10-24/fig_quan_lines` (exploratory) | waits on the arm and the bars |
| G5 | the ledger: band against real-world value per flight (F16) | **missing** | 11 scored rows RIG-1021-*, RIG-1022-f3 against `docs/experiments/2026-10-16_bands_1m.md` |
| G5 | simulation against real world, one release | exists `F_twin_rig_parity` (10 Oct, 0.55 m) | as is |

## 5. What the thesis claims, and whether the results support it

The compiled master states its claims in the introduction (one question) and in 2.7 (G1-G5 and the "this thesis" row of
`tab:lit_gap`). The 4 Oct `contributions.tex` and the 17 Oct RQ1-3 / C1-C5 are not compiled.

| Claim (where) | Support now | Gap | What closes it |
|---|---|---|---|
| G1: a vehicle joins a payload a team is already carrying, as an admission contract with another controller, **in simulation** (2.7, lit map, intro) | simulation joins exist (v3_full `res_join`, the p115 floor joins, U, the rejoin); no join text or figure in the master's results | the real-world join has never completed (29 Oct: the magnet's swing on 1 m beat the collaborator's windows; 30 Oct: no ring flight held) | write 4.2's join from the simulation; the real-world cell is a bonus if Wed 4 to Fri 6 Nov deliver |
| G2: the leave flown on the hardware, commanded and unannounced, with tilt rise, detection time and recovery as numbers (2.7, `tab:membership`) | yes, 8 Oct (r302_3 announced, r303_4 and r305 unannounced, r304_3 a 150 deg gap), 0.55 m / 0.86 kg | none on 1 m / 1.15 kg (29 Oct chain: plate 0 let go under load, an unplanned release) | state the payload; the 29 Oct releases can be reported as magnets that let go |
| G3: the planner switched to a pre-built, primed solver at an unscheduled event, judged by its first solves (2.7) | yes in v3_full `res_leave` ("The switch itself") and S | not in the master | move it into 4.2 |
| G4: a ground tracking MPC per vehicle for closed flight controllers, the cable's pull planned or measured (2.7, intro) | yes, 7-22 Oct carry on 0.55 m, the 1 m pair and four-UAV flights of 28-29 Oct | the measured pull needed a 5.3 Hz band-stop on the 1.15 kg ring, and the 30 Oct heave runs through the measured-pull loop (h3tp) | report both as findings of the measured pull on a flexible ring |
| G5: the simulation's prediction frozen **for each real-world test condition** before it flies (2.7) | 11 carry flights of 21-22 Oct scored against bands frozen 18 Oct: RMSE inside 2 (f3 missed by 0.7 mm), tilt inside 2. The release bands W4/W5 (11 Oct) never flew | "each" overclaims: every flight since 22 Oct is exploratory, with no band for the 1.15 kg ring | freeze bands from the p115 simulation runs before Wed 4, or narrow G5 to what was scored |
| Abstract: "outperforming a dissipative alternative" | no claim table (C's bars not frozen) | | rewrite after the freeze |
| Scalability (results 4.3) | S is done in simulation (a projected real-time limit, 8 UAVs in simulation) | motivated by no gap or question in the intro or 2.7 | one sentence in the intro or 2.7 |

## 6. Top gaps, in priority order

1. **The results chapter has no text, and its real-world figures are on the old setup.** Decide which payload each
   claim reports; write 4.1-4.3 now from the parts that will not change (8 Oct releases, S, U, the rejoin, the 0.55 m
   carry) and add the 1 m flights after 7 Nov.
2. **The real-world join has not flown.** Write G1 so it stands on the simulation join; the 4 -> 3 -> 4 of the
   supervisor's spec is at risk.
3. **The methodology no longer matches the hardware** (section 2) and lacks the methods behind three result figures
   (drafted) and behind the comparison and the chain (not drafted, waiting on scope).
4. **G5 overclaims.** Freeze bands for the Wed 4 to Fri 6 Nov cells from the 1.15 kg simulation runs, or reword.
5. **Discussion, conclusion and abstract** are missing or stale, and the comparison with Quan et al. that the supervisor
   asked for is not claim-ready (no frozen bars).

## 7. Questions for Wesley

1. Which real-world flights do the carry and leave results report: the 0.55 m / 0.86 kg flights (7-22 Oct, the only
   clean releases), the 1 m / 1.15 kg flights from 28 Oct, or both side by side? The methodology follows the answer.
2. G5: freeze bands from the p115 simulation runs for next week's cells, or narrow the claim to the 21-22 Oct ledger?
3. Do the comparison (C) and the release chain (N) stay? If C stays: our port, the law as published, or both, and when
   are its bars frozen?
4. Add a short contributions list to the introduction (2.7 already speaks of "research questions")?
5. Compile the appendices (the methodology points to one), or drop the pointer?
6. Rename the symbols with two meanings (f_i, r, theta, m)?

## 8. Drafted today (no new results needed)

In `~/mdc_wt/thesis/docs/thesis_drafts/thesis_v3_full/Source/src/drafts_1030/` (index `index.md`), written against the
Overleaf master's labels and style; a scratch build of the master with all three in place gives 59 pp., 0 undefined
references, no overfull box.

| File | What | Replaces |
|---|---|---|
| `nomenclature.tex` | every symbol and acronym of the compiled chapters, with units, grouped Latin / Greek / acronyms | the template placeholders |
| `test_cases.tex` | 3.5.2 "Test references and cases": the figure-eight as flown since 7 Oct, the spinny line, the hold, the real-world sizes, the two payloads, a test-case table | `trajectories.tex` |
| `sim_studies.tex` | new 3.6 "Studies in simulation only": the catch offset, the departed carrier's rejoin, the larger team and the bench | nothing |

# Review B: is the thesis heading the right way, and does it claim only what the evidence supports?

Read-only review, Thu 5 Nov 2026, 22:26-23:10 AEDT. Thesis: `~/thesis_full_draft` working tree (0439f7a + local edits to
`results_discussion.tex` and `conclusion_draft.tex`, the F051/F052 changes). Evidence: `docs/results_feed.md` F030-F052,
`docs/decisions.md` (28 Oct onward, especially lines 758, 801, 809, 827), `results/registry.csv`, the cards X1, X4 and X8,
`docs/claim_batch_2026-11-07.md`, `docs/sim_runs_todo.md`, the Wed 11 and Thu 12 sheets, and
`docs/figures/review_2026-10-23_best/index.md`.

**Labels.** CONFIRMED means I checked it against the registry, the feed, a card or a config. OPINION means my judgement.
Line numbers refer to the working-tree files. RD means `Source/src/drafts_1030/results_discussion.tex`, CO means
`conclusion_draft.tex`, CI means `contributions_intro.tex`, and TP means `Source/title_page.tex`.

---

## 0. Summary of the top findings

1. **X1, the planned headline, falsified its own claim, and the one arm that gave it a positive result was removed.**
   CONFIRMED. On the X1 card, B1 ("the resize keeps the ring up") and B2 are FALSIFIED as frozen
   (`docs/experiments/2026-10-30_x1_release_ablation.md:215-216`). F035 on the current tree shows the same: with and
   without the resize, the tilt rise and the dip are equal. The only confirmed results were B3 (a planned pull
   instead of the measured pull makes the release peak 2.4-4.4x worse) and the 2x2 neighbour (with neither mechanism the
   ring hangs at 13-19 deg). Both were removed on 2 Nov (decisions.md:758: "no planned pull anywhere"), and the weekend
   batch cannot score them (claim_batch:24). As written, the X1 table (RD:104-122) shows a 0.06 against 0.21 deg final
   tilt, which the text itself calls "about 1 mm". In the T layout the resize makes the result worse (RD:132). OPINION:
   as it stands, X1 is a null result for C1 and is not evidence of added value. It is also the weakest point an examiner
   would find.
2. **C4's measured pull now has no supporting evidence in the draft.** CONFIRMED. Decision 758 left C4 resting on "the
   fixed-team tracking and the check against the ring's weight". That check (`tab:res_pull`) is commented out
   (RD:515-540), and every planned-pull comparison is gone (X1 B3; the gap table's G150p row,
   `docs/analysis/refly_1m_table_2026-10-23.md:34,106`). C4 therefore rests only on Setup 1's fixed-team flights.
3. **X4 is stale in the table, the text, the abstract and the conclusion, and its headline ratio no longer means what
   it says.** CONFIRMED. On the frozen tree, R3772 and R3773 give a 2118 / 2109 s mission (3.53 / 3.51 B), an event tilt
   of 2.52 / 2.61 deg and a longest pack of 707 / 703 s, against a B of 600 s, so R7 is missed (decisions.md:827). The
   draft still says 1670 s, 2.8 B, under 1.8 deg and "no quadrotor flew longer than its budget" (RD:274, RD:287-291,
   TP:50, CO:11). OPINION: "x B against 0.95 for a fixed team" is set by the script. The fixed team is programmed to land
   at B, the relay to fly 26 leaves, and B itself was raised from 5 to 10 min so the rota could fit inside it (X4
   card:84-90). The longer mission now comes mostly from slower rejoins while the path is paused.
4. **The real-world join results are the novel result, but the abstract overstates them and the draft leaves out the
   conditions they were flown under.** CONFIRMED. "Quadrotors detached from and re-attached to the carried ring" (TP:50)
   rests on one re-attachment, sq8_2, out of 43 re-attach descents: 1 of 2 on the square and 0 of 41 on the triangle
   (RIG-1105-joins, RD:501-504). That flight ran with the ring-tilt drop limit raised to 55 deg (registry RIG-1105-sq8_2,
   decisions.md:799). The ring reached 30.7 deg at the fold-in (Wed sheet:100). The default 30 deg limit would have
   dropped it (decisions.md:801), and the flight ended with the ring dropped by a false loss detection. RD:482 and
   RD:412 give 21-28 deg and never mention the raised limit.
5. **The simulation-to-real comparison is not like for like, and its real cells are best-of-N picks described as single
   flights.** CONFIRMED. RD:371 says "most cells are a single flight". In fact the 3- and 4-quadrotor cells are the
   lowest of 2, 4 and 6 flights (`docs/figures/review_2026-10-23_best/index.md:41-45, 77-83`). They also flew 0.55 m
   cables, against 1 m in simulation, and the 4-quadrotor hover was a 5.3 s window on the 7 Oct tree. The simulation's
   own configuration (0.86 kg ring on 1 m cables) is Setup 3, which has no fixed-team flight yet.
6. **Table and text inconsistencies** (section 4): the X8 rim-loss "tilt peak" for the proposed planner is the late
   tilt, not the peak (56.4-56.9 deg in the registry). The 4.3 join re-fly flies the 1.15 kg ring, but the text says
   0.86 kg. `tab:setups` has no Setup 3. The simulated T's "8-12 deg" comes from exploratory runs that do not reproduce,
   and after the re-fly the claimed T cells fly a different controller setting from the real T.
7. **The schedule risk is in the real-world sessions, not in compute.** CONFIRMED. The like-for-like Setup 3 baselines
   and the 1 m unannounced release were planned for S1 and again for S2, and neither was flown (handin_plan:120-124).
   They are now at the end of Wednesday's 3-hour order (Wed sheet:40-45).

---

## 1. The story: what the thesis claims, and whether the evidence carries it

| # | Claim (where) | Evidence | Verdict |
|---|---|---|---|
| A1 | A planner resizes its OCP at each team change (TP:50; CI:14 C1: "a team can lose or gain agents without putting the payload down") | Switches: the first solve never failed in 20 simulated resizes (F012). On the real system it stayed within budget, but on the two Setup 2 switches the residual was up to 10.5x and the step took 157-161 ms (F013, RD:472) | **Mechanism supported. "Enables losing agents" is not supported.** X1 shows the ring survives a loss without the resize (B1 falsified). For gains the resize is necessary. OPINION: state C1's value as joins, freeing the leaver and the final level, not survival |
| A2 | Unannounced detection within 0.42 s in simulation and the real world (TP:50, CO:13) | Simulation 0.25-0.38 s (tab:res_gap, tab:res_s8). Real 0.33-0.42 s on Setup 1 only: 0.55 m cables, 8 Oct, an earlier velocity estimate (RIG-1008-r303_4 0.42, -r304_3 0.33, -r305 0.34) | **Supported** for Setup 1. Wednesday's `rel` line detects with the magnet-marker check (`tip_detect_m` 0.04, `configs/rig/m1_release_tip_m2a_det15.yaml:3`), while the simulation and Setup 1 use the 6 cm cable-stretch check, so a new real detection time is a different detector (section 4, item 12) |
| A3 | The largest gap sets the tilt, not detection or the resize (CO:13, RD:79) | tab:res_gap matches the refly table (R2486-R2614). Real 5.3 against 16.9 deg at 120 and 150 deg gaps | **Supported** |
| A4 | The resize brings the final tilt to about a third (CO:13, RD:99) | 0.06 against 0.21 deg in hover (F035). Opposite in the T: 10 against 2.2 deg (RD:132) | **True but trivial**, as RD:99 says itself (~1 mm). Not a headline. The T result changes after the re-fly with int_k_xy 0 |
| A5 | 8 to 4 to 8 and 9 to 5 to 9 at one iteration per control step (TP:50, CO:11) | tab:res_s8 (R2764-R2767, ema, older tree), tab:res_x3b (R3568-R3585) | **Supported in simulation.** Re-flown in chunks 2-3. Simulation only, real-time factor 0.22. Real time is a projection: the 3.5 flight factor gives 178 ms or more at 8 (RD:245) |
| A6 | The relay kept the ring flying for 2.8 battery budgets, against 0.95 (TP:50, CO:11, RD:274) | Superseded by R3772/R3773: 3.5 B with packs over B (R7 missed) | **STALE, and the metric needs reframing** (finding 3) |
| A7 | Closer than Quan et al. in every fixed-team case, and less ring motion through a loss (CO:11, RD:310-318) | X8 reviewer: supported with corrections (F045). Port and gains are this work's (RD:308) | **Supported against the port.** The rim-loss table cell is wrong (section 4, item 1). OPINION: an examiner will ask whether the port is fair (the paper's damping form tipped over 6/6 times) |
| A8 | A new agent attached to the carried ring; detachment and re-attachment were demonstrated (TP:50, CO:15, RD:478-486) | sq6m: 1 of 3 attaches, on the second contact. sq8_2: 1 of 43 re-attaches. Both with loosened gates (decisions.md:767: contact xy 0.06 m on 27.5 mm-radius plates) and the drop limit at 55 deg (sq8_2) | **Demonstrated once each. Reliability is not shown.** The abstract's plural overstates it. F048 itself says "demonstrated once each ... not claims" |
| A9 | Simulation: every attachment was confirmed on the first attempt (CO:15, RD:150) | 6 exploratory runs, 1.15 kg (R2843/R2844, R2862, R2906-R2908, R2865). In the simulation the magnet welds wherever the tip is (rescope §2, C3) | **Supported as stated.** It says little about a real catch |
| A10 | The tracking MPC with the measured pull kept the ring within about 2 cm of most paths (CO:17, RD:399, C4) | Setup 1 best-of-N flights (index.md:41-50), 0.55 m cables | **Stronger than the evidence:** best-of-N, the old setup, and no ablation of the measured pull in the thesis (finding 2) |
| A11 | C2: "can handle an unexpected loss and still complete the mission" (CI:15) | Holds for gaps up to 150 deg. Setup 2's three losses dropped the ring (RD:469). X2's unannounced square-to-T needed the drop limit raised to 80 deg (peaks 58-59 deg, F042) | **Stronger than the evidence.** CO:11 has the right hedge ("whenever the remaining quadrotors could support it"). CI:15 does not |
| A12 | Limitations (CO:22-24) | Honest on delays, magnet weight, the T, 8-9 quadrotors | Good. Missing: the raised drop limit on sq8_2, the loosened confirmation, best-of-N real cells, and the per-cell integral setting in simulation |

---

## 2. The headline: are X1 and X4 still right, and is the real world framed right?

**X1 (OPINION, on CONFIRMED data).** No, not as "what the resize is worth". Its frozen falsifier tripped on both fleets
(card:215). The thesis now reports a resize effect of about 1 mm and a second 4 deg swing at +2 s in hover (RD:99), and a
worse T (RD:132). The X1 data hold a strong result, but it is a different one: **two redundant mechanisms carry a
release.** The measured pull holds the transient (planned pull 16.7 against 5.7-7.2 deg; dip 53 against 29-34 mm,
R2923/R2939 against R2918/R2934). The resize is the backup that levels the ring when the pull is only planned (1.9
against 19.0 deg settled, R2923 against R2924). That is evidence for C4 and gives C1 a defined role. It needs the
planned-pull arms, which decision 758 removed. The configs exist (`x1_{h4,f4,h8}_{plan,nplan}.yaml`), so they can fly on
the frozen tree.

**X4 (OPINION).** Keep it as the demonstration that the events compose at scale: 26 of 26 swaps, about 35 min, event
tilt at most 2.6 deg, and no growth in path error over 28 laps. Stop selling it as an endurance multiple:
- The fixed team's 0.95 B is scripted (`traj_laps` 24), and the relay's length is set by its 26-leave rota.
- B was raised from 5 to 10 min because no rota fit 5 min (X4 card:58, 84-90).
- On the frozen tree B is violated (695-707 s against 600 s).
- The path pauses for each rejoin, so 28 laps took about 2110 s, against 24 laps in 570 s.

The honest numbers are mission / longest pack (about 3.0), the laps flown, and the share of time paused.
Decisions.md:827 already chose the wording ("a 35 min mission with 26 swaps on ~12 min packs"). The text and abstract
have not caught up.

**Real world (OPINION).** These results are the thesis's real novelty: the first real join (sq6m), the first real detach
and re-attach (sq8_2), and the pickup and drop into the flying ring (k1d). RD:478-486 and tab:res_descents already give
denominators and the miss mechanism, which is good. The framing still needs:
- the abstract's plural changed to "once";
- the drop limit at 55 deg and the 30.7 deg tilt at the fold-in stated;
- the 4 Nov loosening of the confirmation stated;
- the triangle's 0 of 41 stated as the partner controller's failure mode (RD:486 does this well);
- a figure. The most novel result has none: handin_plan:174 "The first real join (if it happens)" is still unticked
  although it happened.

**What an examiner attacks first (OPINION, ranked):**
1. "Your ablation shows your key mechanism, the resize, does not keep the payload up. What does C1 add?" (X1 B1 and B2
   falsified.)
2. "The real ring tilts 21-28 deg on a T, against 8-12 in simulation. Real errors are 1.4-23x simulation. Your headline
   scale results (X4, 8-9 quadrotors) exist only in a simulation that is not deterministic (the T reads 8.5, 11.2-12.0 and
   2.1 deg on the same line, R3708/R3712-R3715/R3771). How much transfers?"
3. "One real re-attachment in 43 descents, with the drop limit raised and the confirmation loosened. What is the success
   rate?"
4. "The real fixed-team rows are best-of-N."
5. "The settings change per scenario." In simulation the T cells fly int_k_xy 0 while the real T flies it on, and the
   drop limit is 30, 55 or 80 deg per line.
6. "Is your port of Quan et al. a fair baseline?"

---

## 3. Gaps against the hand-in (20 Nov)

**In progress (CONFIRMED, SESSION_STATE, claim_batch):** the 255-run claim batch, started Thu 18:25 on the frozen tree
(bf52a8a), about 22 h, so it ends around Fri evening. Every simulation table in RD changes after it: X1, X2, X3, X3b, X4,
X8, the 4.1 fixed team, the release in both, the gap table, catch offset U, rejoin and the 4.3 joins. Rejoins will be
slower everywhere (decisions.md:827). That moves tab:res_rejoin (15.6-23.9 s), tab:res_s8 ("time from start to resize"),
X2's T6 and X3's N3. The X4 fixed-team re-fly (R3774-R3776) is pending. handin_plan §5 asks for a reviewer per claim
table: about 10 tables in about 3 days, with the scoring and the feed entries on top.

**Figures still owed (CONFIRMED, handin_plan:159-176, RD):**
- fig_x1_resize and fig_x4_relay must be re-run on the re-fly (scripts exist).
- `\pendingfig{fig:res_join_sim}` (RD:170).
- fig_rejoin cannot be redrawn from its old run (its Gazebo state log was deleted). Redraw it from the chunk 8c re-fly.
- The real figure-eight at 1 m (needs S3).
- The real release at 1 m (needs S3 `rel`).
- The first real join (data exists now).

**Real world (CONFIRMED, sheets):** Wed 11 has nrs x3 (the re-attach claim), the 12 baseline flights and rel x2, about
3 h in a 3 h window. The baselines and rel come last. On 4-5 Nov the same lines were planned and not flown
(handin_plan:120-124). Thu 12 has the pickup k1z x2, the funnel try, Nr on the orbit and Wednesday's leftovers, with the
freeze at 11:00.

**Writing:**
- `tab:setups` needs Setup 3.
- The methods must state the per-cell settings: int_k_xy 0 for T cells (F050 asks for this, and only an orange note at
  RD:24 has it), and the drop limits.
- The abstract and conclusion need rewriting on the final numbers.
- Supervisor round 2 is suggested for Mon 16 (handin_plan:74). Round 1 is unticked on the board; I could not verify
  whether the 4 Nov supervisor meeting covered it.

**At risk by 20 Nov (OPINION):**
1. nrs x3 not catching would leave the real re-attachment at n = 1.
2. The Setup 3 baselines and rel slipping a third time would leave the simulation-to-real comparison not like for like.
3. The reviewer pass on every re-flown table.
4. A rewrite of every number, abstract and conclusion included, in the week before the Mon 16 draft.

**What to cut if time runs short (OPINION, in order):**
1. Catch offset U: skip the reviewer, or reduce it to a sentence. A real magnet only catches within 2.5 cm, where both
   arms succeed (RD:201), so its message does not reach the real system. Its 60 runs (chunk 8a) take the most scoring
   and review time.
2. X3's "further cases" paragraph (RD:270): scorer only, no reviewer.
3. The simulated 4.3 joins table (exploratory; keep one run for the figure).
4. Real-world Nr on the orbit and the funnel try.
5. X2's unannounced arm: keep the soft arm as the claim and give the 80 deg arm one sentence.

Do not cut nrs, rel, or one flight of each Setup 3 baseline.

---

## 4. Consistency: the draft against the registry and the feed

| # | Draft (file:line) | Says | Source | CONFIRMED discrepancy |
|---|---|---|---|---|
| 1 | RD:338 (tab:res_compare) | Proposed, loss with cables on the rim: "Tilt peak 7.0-16.1" | Registry R3642/R3643: ring tilt rise 56.85 / 56.39, abs peak 56.87 / 56.40, settled 15.8 / 6.8 deg | The cell reports the late tilt for the proposed planner but the peak ("Over 80") for Quan's method. Its real peak is 56-57 deg. The re-fly flies `x8_rim_ours_noxy`, so the number changes anyway |
| 2 | RD:274, RD:287-291, TP:50, CO:11 | 1670 s, 2.8 B; event tilt under 1.8 deg; "no quadrotor flew longer than its budget" | R3772/R3773: 2118 / 2109 s, 3.53 / 3.51 B, 2.52 / 2.61 deg, longest pack 707 / 703 s (> 600); decisions.md:827 | Stale on the frozen tree. The "budget" sentence is now false |
| 3 | RD:371 | "Most cells are a single flight, and the two-quadrotor cells are the best of several flights" | index.md:41-45: h3 lowest of 2, r20004 lowest of 4 (a 5.3 s window, 7 Oct tree, "to be replaced by Wed 28"), f3h lowest of 6, r2008e60 lowest of 2 (half speed) | Best-of-N in 4 of the 5 non-pair cells |
| 4 | RD:150 (orange), RD:24 | 4.3 joins "re-flown on the 0.86 kg ring"; "only Tables res_x1 and res_x4 keep a 1.15 kg ring" | `configs/batches/claim1107_8b_x11_joins.txt`: all 7 configs have `load_mass: 1.15` and `_p115` worlds (the other chunks are 0.86 or 0.9) | The re-fly is on 1.15 kg, contradicting the text and F050 / decisions.md:809(3) |
| 5 | RD:482, RD:412, tab:res_both RD:428 | Real T 21-28 deg; no mention of the drop limit | Registry RIG-1105-sq8_2 "drop 55 deg"; decisions.md:799, 801 ("the default 30 deg drop would have fired"); Wed sheet:100 (30.7 deg at the fold-in) | The safety setting and the peak are missing |
| 6 | RD:412, RD:428 | Simulated T "about 8 to 12 deg" | R3708 8.5 and R3712/R3714/R3715 11.2-12.0 (exploratory, the difference unexplained, F049). R3771 (frozen tree, the same square with the rig's arguments, on the paused orbit) T median 2.1 deg | The range is not reproducible. After the re-fly, every claimed T cell flies int_k_xy 0 (about 1.8 deg, R3701), the opposite of the real T's setting |
| 7 | system_overview.tex:93-115 (`tab:setups`, "The two hardware setups") | Setup 1 and Setup 2 only | RD:24, 466, 478, 482, 496 use Setup 3; it is defined only in Appendices.tex:225 | Setup 3 is missing from the methods |
| 8 | RD:515-540 (commented) and decisions.md:758 | C4 rests on tracking plus "the check against the ring's weight" | The check is commented out | C4 lost its second leg |
| 9 | Methods (no match for "horizontal part" or "sideways" outside controller.tex:29) | | F050: "the method describes these settings" | The int_k_xy 0 setting for T cells is stated only in an orange note (RD:24) |
| 10 | RD:408 | The real release "used ... the earlier exponential-average velocity estimate" | F032: write "an earlier velocity estimate" (ema not established). claim_batch:44: R3507/R3508 themselves flew ema although their registry notes say slope | Overstated on the real side. The simulated side also flew ema. Fixed by the re-fly (`x11_r1m_ta_a1_r1/_r2`, slope) |
| 11 | TP:50 | "quadrotors detached from and re-attached to" | RIG-1105-joins: 1 caught out of 43 re-attach descents | Plural and no denominator |
| 12 | (future) RD:408, tab:res_both | Detection 0.42 (real) against 0.34-0.35 s (simulation), cable-stretch check 0.06 | Wed `rel` types `tip_detect_m 0.04` (magnet-marker) and `detach_detect_m 0.15` (`configs/rig/m1_release_tip_m2a_det15.yaml:1-10`). The simulated X1 and 4.2 cells use the 0.06 cable check (`configs/twin/s_chain_pair.yaml:7`, `configs/rig/drop_and_land.yaml:4`) | A 1 m real release flown Wednesday would use a different detector from its simulated partner. Same manoeuvre, different detection |
| 13 | RD:466 | "The tilt rise at the release was 2.9 to 5.2 deg" | F040: "ring tilt 2.9-5.2 deg at the release" | Possibly a tilt, not a rise (not re-scored; check) |

**Spot checks that MATCH (CONFIRMED):**
- tab:res_sim_fixed against F031 (every cell).
- tab:res_x1 against F035.
- tab:res_gap against refly_1m_table (rows 103-107).
- tab:res_release against RIG-1008-r302_3/r303_4/r304_3/r305 (detection 0.42 / 0.33 / 0.34; peaks 15.4 / 6.5 / 17.8 / 8.1) and F031/F033.
- tab:res_x3b against R3568-R3573 and F037.
- tab:res_tick against S-BENCHB-1024 / -1024c (83.1 ms at 8 and 1 iteration).
- tab:res_descents against RIG-1105-joins.
- X2 and X8 text against F042/F045.
- res_scale_8 numbers against their sources (comments, 30 Oct re-read).

---

## 5. Recommendations (ranked by value per cost)

1. **Re-centre the headline on what is new and true** (OPINION). Proposed order:
   - (a) the first real attachment and the first real detach and re-attachment to a carried payload (G1), as
     demonstrations with denominators;
   - (b) composition at scale in simulation (8 to 4 to 8, 9 to 5 to 9, the 26-swap relay);
   - (c) X1 recast as a mechanism result: the measured pull carries the transient, and the resize levels the ring and
     enables joins.
   Rewrite C1's "enables" and C2's "complete the mission" with the hedges the data support.
   *Why:* the current headline X1 is falsified as worded, and examiners read the abstract first. *Cost:* about half a
   day of writing (abstract, CI, CO, RD:99-132).
2. **Put the planned-pull arms back in X1 on the frozen tree** (needs Wesley to revisit decision 758). Fly
   `x1_h4_plan`, `_nplan`, `x1_f4_plan`, `_nplan`, `x1_h8_plan`, `_nplan` x2 (12 runs, about 1 h on 2 slots after
   chunk 9; scorer `tools/x1_metrics.py --bars` already scores B3).
   *Why:* it is the only causal evidence for C4 and turns X1 from a null into a 2x2 result. Without it C4 rests on
   best-of-N Setup 1 flights. *Cost:* about 1 h of lab PC, about 1 h of scoring, and one table column.
3. **Fix X4 now to the frozen tree's numbers and a meaningful metric.** Report:
   - 26 of 26 swaps, about 2110 s, event tilt at most 2.6 deg, no error growth;
   - packs of 695-707 s against the 600 s budget, with its cause (slower rejoin on the hardware-matched links);
   - mission / longest pack about 3.0 against about 1 for a fixed team;
   - laps flown and the share of time paused.

   Drop "2.8 battery budgets" from the abstract and the conclusion.
   *Why:* the current numbers are stale and the ratio is scripted. *Cost:* about 2 h, plus re-running fig_x4_relay
   after R3774-R3776.
4. **State the real join's conditions and denominators everywhere it is cited:**
   - 1 of 3 attaches;
   - 1 of 43 re-attaches (1 of 2 on the square, 0 of 41 on the triangle);
   - pickups 6 of 15;
   - the drop limit raised to 55 deg on sq8_2, with a 30.7 deg tilt at the fold-in;
   - the confirmation loosened on 4 Nov;
   - the flight ended with a false-loss drop.

   Fix the abstract's plural.
   *Why:* this is the first thing an examiner checks on a "first" claim. *Cost:* about 1 h.
5. **Order Wednesday so that every cell gets one flight before any repeat.** Fly nrs x3 (stopping early if two catch),
   then h4 and f4 x1, re-hang, rel x1, h3 and f3 x1, and only then the repeats and the spinny lines. Fly rel with the
   same detector as its simulated partner, or say it is a different detector (section 4, item 12).
   *Why:* the like-for-like Setup 3 table and the 1 m release have slipped twice. One flight per cell secures the
   table, and repeats are a bonus. *Cost:* none (reordering only).
6. **Ask Wesley about nrs0 (int_k_xy 0 on the real square, one flight; script exists:
   `results/rig/2026-11-11/scripts/rig_nrs0.sh`).** *Why:* it is the only real test of the thesis's own explanation of
   the 21-28 deg T lean. It is the setting every claimed simulated T now flies. If the real T levels, the re-attachment
   may no longer need the 55 deg limit. *Cost:* about 11 min of the session. It has never flown on the real system and
   needs Wesley's word.
7. **Make the simulation-to-real comparison like for like.** After Wednesday, compare Setup 3 (0.86 kg on 1 m, the
   simulation's own configuration) with the simulation, and keep Setup 1 as a second setup. Report the median, or all
   flights per cell. Where a best flight is shown, say "best of N" with N.
   *Why:* removes findings 5 and A10. *Cost:* about 2-3 h of scoring after Wednesday.
8. **Correct the five quick consistency items:**
   - RD:338: the proposed planner's rim-loss tilt peak is about 56.4-56.9 deg; label the late tilt separately.
   - RD:150 and RD:24: the 4.3 joins are on 1.15 kg.
   - `tab:setups`: add the Setup 3 column.
   - The methods: one paragraph on the per-cell settings (int_k_xy 0 in T cells, the drop limits of 30, 45, 55 and 80
     deg and where each applies, the loss check at 6 and 15 cm).
   - RD:408: the velocity-estimate wording from F032.

   *Cost:* about 2 h in total.
9. **Make the real detach and re-attach figure now** from the sq8_2 bag: the team size, the ring tilt, QUAD4's path and
   the events, in the style of the existing fig_rejoin. *Why:* the most novel result has no figure, and the data will
   not change. *Cost:* about half a day with the thesis-figure skill.
10. **Set the review order for the re-fly.** Review X1, X4, X8 and the release flown in both first, then X2 and X3b.
    U, X3's extra cases and the 4.3 simulated joins get the scorer only, unless time allows.
    *Why:* reviewer time, not lab-PC time, is the constraint before Mon 16. *Cost:* none.

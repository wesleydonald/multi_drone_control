# Everything before hand-in (started 30 Oct 2026; hand-in Fri 20 Nov)

One list of all that is left: the writing (Wesley's steps), the supervisor's feedback, the real-world sessions, the
simulation runs, the figures, the reviews and the admin. Tick items as they close. The order of work is what matters;
the dates below are only the fixed ones (lab sessions, the freezes Wesley set, hand-in).

## Direction from Fri 30 Oct evening (the order of work; tick below as items close)

**Where it stands, Thu 5 Nov night (the lab chat; the 30 Oct direction below is history where it says so):**
- The T fix: settled by the 31 Oct fallback (the T cells fly the sideways integral off); the rod-in-the-plan fix was
  reversed 4 Nov. X2-full and X8 no longer wait on it.
- QUAD4's fold-in cap: decided 5 Nov ('level' kept off, 'weld' deleted).
- The code freeze is committed (bf52a8a, Wesley 5 Nov); the weekend claim batch runs every claim on it from 5 Nov 18:25
  (docs/claim_batch_2026-11-07.md, ~22 h). X4 on that tree: 26/26 swaps but each pack ~12 min against the 10 min budget
  (the rejoin is slower on the hardware-matched twin); Wesley: report as flown.
- The laptop's X3b timing runs cannot fly 8-9 quads with Gazebo on it (R3451 31 Oct, R3777/R3778 5 Nov): the ground
  computer keeps R3451 and the idle bench (re-run 6 Nov 00:30).
- S1/S2 flew the combined attach work instead of the baselines (see S1 and S2 below); the baselines and the release move to
  S3/S4. The real-world attach table (every descent of 4-5 Nov): feed F052.

**First, before the code freeze (Wesley 30 Oct): fix the square-to-T settle.** X1's first pass: after an unannounced
release from the square, our planner settles the T 7-13 deg off level where no resize settles at 2.2 deg. Diagnose
from the logs, fix behind a switch, critic, twin (X1's sqT cell), then the default. X2-full and X8 wait for it.

**Lab PC (lab chat), each batch as the last ends, from the 30 Oct commit on week8:**
1. X3b, 18 runs (~2.1 h): 8 and 9 carriers at one planner iteration per tick, with the five-iteration comparators.
   Bars frozen (lab runs compute the residual every tick).
2. X4, the short exploratory run, then the claims (~5 h on 2 slots). Bars frozen, B = 10 min. Merged 30 Oct.
3. X2-full, 6 runs (~1 h), after the T fix: the 4 -> 3 -> 2 -> 3 -> 4 chain on the figure-eight. Bars frozen.
4. X8, ours against the port (15 runs, ~45 min on 4 slots; bars frozen 30 Oct), after the T fix; the exact law's loss row: P3.
5. Then the code freeze (Fri 6 Nov): X11 (86 cited claims x2) and the claims of X1-X4 and X8 on that one tree
   (Sat 7 - Sun 8), simulation freeze Mon 9.

**This laptop (the ground computer), on a quiet day with the rig stack down:** X3b's two timing runs (`*_rt0`) and the
solver bench, then the solve-time figure with the one-iteration tick for 2-9 carriers.

**After the T fix, before the freeze:** QUAD4's fold-in gets the rejoin's weld-height cap (card, critic; Wesley 30 Oct).

**Real world:** S1 Wed 4 Nov, the sheet `docs/rig/archive/wed_2026-11-04.md` is desk-checked (h3t, h4m1, sq6b x3-4, release
x2; W and L measured on the day). S2 Thu 5 Nov: the sheet `docs/rig/archive/thu_2026-11-05.md` is desk-checked 2 Nov (attach first if S1 went to the heave, then
the release x2 if missing, then the 1 m baselines); its calls are typed Wed afternoon from Wed's logs.

**Thesis chat (starts once the draft is on Overleaf):** supervisor round 1 (this weekend: Wesley picks and rewords the
sections), the arena fix (6 x 6 x 5 m), cite the supervisors and reword G4, F1's caption ("halved", not "removed"),
then each planned box as the lab chat ticks its result here.

**Wesley's open calls:** the T fix: he asked first why the sideways integral is needed at all (31 Oct, two read-only
investigations running); the automatic rule is the preferred fix if nothing is found. Push week8 once the T fix is
merged (Wesley).

**Fixed dates**
- Lab sessions 8-11 am: 4/5 Nov, 11/12 Nov, 13 Nov; next week (Wesley 14 Nov) Mon 16, Thu 19, Fri 20 (Wesley), Tue 17 and
  Wed 18 (Te-Jen alone). Sheets: docs/rig/mon_2026-11-16.md, docs/rig/tue_2026-11-17.md.
- Code tree freeze Fri 6 Nov; simulation freeze Mon 9 Nov.
- Real-world freeze (Wesley 14 Nov): the thesis takes flights up to Thu 19 Nov; Friday's are for the combined paper.
  Hand-in Fri 20 Nov 17:00.
- Hand-in Fri 20 Nov.

**Who:** sections 1 and 4 (writing, figures) are the thesis chat's; sections 2, 3 and 5 the lab chat's (it ticks a result
here with its run ids when final, the thesis chat's signal to write it up); section 6 both, as marked.

**Where things are**
- Full draft: `~/mdc_wt/thesis/docs/thesis_drafts/full_draft_2026-10-30/thesis_full_draft_2026-10-30.pdf`.
- Simulation runs: `docs/sim_runs_todo.md`.
- Plan: `docs/plan_rescope_2026-10-30.md`.
- Decisions: `docs/decisions.md`.
- Today's state: `docs/SESSION_STATE.md`.

## 1. Writing (Wesley)

Everything shown to the supervisor is read and reworded by Wesley first.

- [ ] **Read the full draft** (79 pp, 57 main text) and pick the sections the supervisor should give feedback on.
      Orange = still open, boxes = planned results.
- [ ] **Supervisor feedback, round 1** (this weekend, 31 Oct / 1 Nov): the chosen sections, reworded, with short questions.
      Candidates: the contributions against the gaps (2.7-2.8), the methodology's depth (21 pp against the guide's 10-15),
      the results-chapter format (one section as the template), which planned real-world results matter most.
      A ready 30-page pack exists: `~/mdc_wt/thesis/docs/thesis_drafts/supervisor_pack_2026-10-31/`.
- [ ] **Fold in round 1** and settle the methodology's cuts (to the appendix or out).
- [ ] **Move the drafts into Overleaf** section by section as each is reworded (Wesley pushes; nothing is pushed for him).
- [ ] **Results and discussion** (pp. 39-56): fill each planned box as its result freezes; reword each section once its
      numbers are final; the discussion last.
- [ ] **4.1 around the three main trajectories** (hover, figure-eight, spinning line), each with its metrics table for
      2/3/4 carriers x both real-world setups x simulation (being restructured 30 Oct); other paths to the appendix.
- [ ] **No separate Discussion section**: each result's discussion straight after it (Wesley 30 Oct; being done).
- [ ] **Conclusion** (~1 page) after the results freeze.
- [ ] **Abstract** (still the 4 Oct one) last.
- [ ] **Supervisor feedback, round 2:** the full draft (suggested Mon 16 Nov).
- [ ] **Fold in round 2.** Main text to 50 pp: detail to the appendix (appendix candidates are marked in comments).
- [ ] **Final checks:** references, figure and table numbering, nomenclature, page count, the school's format; PDF Thu 19 Nov.
- [ ] **Hand in** Fri 20 Nov.

- [ ] **From the supervisors' papers** (review `docs/analysis/supervisor_papers_review_2026-10-30.md`, "Borrow now"; thesis
      chat): cite their papers and reword G4 (WIND, CEP, PEND already fly off-board MPC to stock Betaflight over ELRS in
      this lab: the thesis extends that to a cable's pull); the thrust map in their units (19.4 N per unit throttle, the
      voltage term) beside theirs; setup tables in their format (the UAV, how each parameter was found, loop timing,
      cameras, arena, ground computer); tilt RMS and p95 beside the peak; X1 as their ablation table (arms as columns,
      mean +- SD, % against no resize); contour / lag error for our path error / lag; `fig:owners` with rates; the
      conclusion with its own limitations and future work. Needs Wesley's five facts (the review's last section).

Open items in the text (orange), each with its reason, are listed in the session notes of 30 Oct. After 30 Oct's
answers, the ones left:
- the confirmation tolerance (settled after the first real join);
- the test-case table's planned join rows;
- the abstract and the conclusion.

## 2. Real-world sessions (order of the rescope, section 4)

**Before every session:**
- [ ] `python3 tools/mocap_stream_check.py --expect <bodies>` says OK: Motive streams only the bodies flown, at most 8.
- [ ] Fresh packs.
- [ ] The arming fix holds: QUAD1-4 recalibrated, `small_angle` 90; console `--small-angle 90`.
- [ ] Desk-checked sheet.

**S1, Wed 4 Nov:**
- [ ] Before the first attach: re-measure QUAD4's W (`payload_geometry.py`, magnet caught on plate 9) and L (cable taut)
      with body 24's new point (22.6 mm above the face; today's sheet typed the old point's 0.028 / 1.041; expect
      ~0.0165 / ~1.0525), then P = L - 0.003, Q = L - 0.04, and Tejen's `magnet_drop_below_quad` ~1.050 for the 1 m geometry
      (attach geometry: Wesley's word). His MPC cable length stays 1.0.
- [ ] Bench latency with the flight's own body set streaming (`tools/bench_latency.py`, props off, 40 s): confirm ~7 ms with
      7-8 bodies, not just one (Tejen's review 30 Oct).
- [ ] h3t triangle hover with the reduced body set: is the heave gone? If not, stop ring flights and blackbox a carrier.
- [ ] The M1 four-quad hover.
- [ ] Tejen's attach sq6b x3-4: the first real join. Catch radius 45 mm; `attach_z` 0.028.
- [ ] The 1 m release x2 (QUAD4's magnet off, unannounced).
- Flown instead (registry RIG-1104-*, feed F039-F041): the first real join from the mat (sq6m, caught and proven on the
  0.86 kg ring), the release then re-attach (nr9, nr9m x5, nr1_2: no catch; nr1_2's plate-1 contacts were dead centre but
  not proven), and the pickup (k1m x4). W and L were re-measured (decisions 4 Nov). h3t, h4m1, the bench latency and the
  release x2 were not flown: they move to S3/S4.

**S2, Thu 5 Nov:**
- [ ] The cable-length re-read (Wesley 31 Oct; card `docs/experiments/2026-10-31_rod_len_reread.md`): if Wed's logs hold its reading, the first h3 flies the
      switch; if the height and tilt bars hold, all the baselines fly it, otherwise all off. `throttle_max:=0.95` on h3, f3, ls3; packs at 24.0 V or more at rest.
- [ ] Attach x3 (first, if S1 went to the heave).
- [ ] 1 m baselines: h3, f3, ls3, h4, ls4, 2 flights per cell (Wesley 30 Oct; the third in S4 for the cells the thesis
      leans on). f4 added last on the sheet (2 Nov): RIG-1029-f4 flew the 2 Hz low-pass that has since come off.
- [ ] Release x2 if missing.
- Flown instead (Wesley's order of 4 Nov evening; registry RIG-1105-*, feed F048): the pickup and drop into the flying ring
  (k1d, worked), the first real detach and re-attach (square, sq8_2, worked), the orbit with the pause (re-attach not
  caught), the drop into the moving ring (k1ob_2, in the net, then an automatic drop), the triangle re-attach (not caught).
  The re-read, the 1 m baselines and release x2 were not flown: they move to S3/S4.

**S3-S6, Wed 11 / Thu 12 / Wed 18 / Thu 19 Nov: ONE list** (Wesley 10 Nov after the supervisor's "results are king":
a full run once + 3-5 detach and re-attach; decisions DECIDED 2026-11-10 23:10 / 23:27 / 23:42). Sheet
`docs/rig/wed_2026-11-11.md`, flown as far as each session gets; the real-world freeze moved to Thu 19 Nov. Funnel v4 on
plate 9, then the supervisor's (`BAND=cone`). No frozen bars.
- [ ] (1) Detach and re-attach: nrtf x5 (hover, plate 9), nr1f x3 (re-attach on plate 1), nro x3 (circle, his TAKEOFF
      stops it for the re-attach).
- [ ] (2) The drop into the circling ring: k1zo x3 (small object).
- [ ] (3) Only if pickups keep failing: fm9hc (the held object into the circling ring, then the re-attach).
- [ ] THE FULL MISSION on the circle: fm9c until one completes (the ring stops by itself for the re-attach); fallback fm9
      in hover.
- [ ] Videos and the supervisor's demo (18/19 Nov).
- Dropped (Wesley 10 Nov): rel, relo (the unannounced releases), fm9o, k1z as a line (backup), the 1 m baselines.

**After every flight:** a registry row; read the log before answering.

## 3. Simulation (lab PC)

All runs are in `docs/sim_runs_todo.md`. In order:
- [x] X1 release ablation: FINAL on the earlier masses R4150-R4193 (feed F087, reviewer corrections F088: the release always caught;
      the resize leaves the ring more level in hover only, adds a second swing, never settles sooner; square-to-T 0.6 vs 2.2 deg only
      on the earlier masses with the integral off (1.74 vs 1.80 on the final masses): reported on both models side by side (Wesley 7 Nov, F089);
      34 solver switches, none failed). The final-mass
      R3821-R3906 (F059 / F060) stay recorded.
- [x] Merge X4, then the short relay check, then the 30-min relays. (Final tree: R3772-R3776, feed F054; R7 missed, reported as flown.)
- [x] X2 the chain: FINAL on the earlier masses R3986-R3991 (feed F069, reviewer corrections F070: A falsified in one repeat, its rise 30.45 deg vs 30 and last fold-in 5.36 vs 5; B confirmed, under 3 deg from 6 s on).
- [x] X3 the 8-quad set: FINAL on the earlier masses R3922-R3927, R3954-R3979 (feed F065 + its reviewer's corrections F066: every return welded; N4 missed by 0.7-1.6 mm in five cases (the window opens with the ring 28-31 mm off after the last fold-in), held in chain2back; the claim chain falsified by N4 as written).
- [ ] X3b eight and nine carriers at one iteration per tick (real-time) + the ground computer's tick (before X4's long runs).
      Chains FINAL on the earlier masses R3980, R3981, R3984, R3985 (feed F067, reviewer corrections F068); C8 and N9 on one model with R4194-R4199 (F090, reviewer corrections F093: C8
      CONFIRMED; N9's flight items hold, N9 not confirmed); R9a done 8 Nov (F095: 50.4 ms at 8, 63.1 at 9, holds); R9b not flyable here (void twice).
- [x] X8 the paper's method through a loss: FINAL on the final tree R4000-R4013 (feed F073: L1 world C and L2 rim CONFIRMED, L3 holds; reviewer corrections F074; the hub's integral-off arm R4200 / R4201 (F091): same verdicts, the centre ends 33-34 mm off, the free ring past 80 deg to LAND).
- [x] 4.1 the fixed team and 4.2 the 4 -> 3 release: FINAL on the final tree and masses R4014-R4033 (feed F076: numbers
      only for 4.1, no card bars; the release detected 0.36-0.37 s, the freed drone disarmed alone (it tipped over in R4033); reviewer corrections F077: yaw 3.7-4.0 deg RMS on the exact clock).
- [x] 4.3 U, the magnet caught off its slot: FINAL on the final tree and masses R4034-R4093 (feed F078: the measured
      arm's fold-in within 2.8 deg to 16 cm along the rim / 20 cm in; 10 fleet aborts in the landing, the newcomer tipping
      over with the ring on the floor; falsified as the card is written by those aborts; reviewer corrections F079: aborts on a touchdown spike, rim ordering reversed since 23 Oct).
- [x] 4.3 the newcomer's join: FINAL on the final tree and masses R4094-R4107 (feed F080, reviewer corrections F081: joins proven, 20-34 s
      on the triangle, 32 s on the square, 97-99 s with a pickup, one cell 3 contacts; the square floor-join landings abort after LAND: TB14, the known twin effect). Re-flown on the 0.86 kg ring (Q011, Wesley 9 Nov;
      R4214-R4227, F097 + F098): every join on the first contact, 20-21 s triangle, 32 s square, 97 s with the pickup; Figure 15 from R4227.
- [x] 4.5 comparison with Quan et al.: FINAL on the final tree and masses R4108-R4139 (feed F082: our planner closer than the port
      in every rim and hub cell; the paper's form lost all 6 runs, as on 23 Oct; the law as printed on world C ~0.14 m off, landed; reviewer corrections F083: rim values accepted by Wesley 23 Oct, like-for-like hub reading "the port", thesis draft line 325 stale).
- [x] 4.2 the gap table (tab:res_gap): FINAL on the final tree and masses R4140-R4149 + R4033 / R4032 (feed F084: tilt rise
      6.3 / 10.3 / 16.0 deg at 120 / 135 / 150, one 165 deg run of two dropped, 180 tips without the drop rule; B1, B2, B4 confirmed; reviewer corrections F085: R4146 valid, Tier B bars never approved by Wesley;
      the draft's table, results_discussion.tex lines 79-96, cites 23 Oct).
- [x] Code freeze: the tree as of Thu 5 Nov 18:25 (Wesley: "Start running"; 9f20f8b + uncommitted, recorded in every
      manifest; gate 2200 passed).
- [ ] The final-tree re-fly of every claim, X1-X4, X8, 4.1/4.2 and X11 (`docs/claim_batch_2026-11-07.md`, 255 runs,
      started 5 Nov 18:25 on the lab PC, ~22 h; scorers `tools/*_metrics.py`). Each chunk is posted in the feed when scored.
- [ ] Simulation freeze Mon 9 Nov.
- [ ] P2 items if time.

## 4. Figures (review folder first; Wesley signs each off)

Style: full box, legends below and framed, tables with full borders (`tools/plot_style.thesis()`, `legend_below()`).

- [x] Solve time per team size as flown (in progress). (done 30 Oct, docs/figures/review_2026-10-30/)
- [x] Figure 17 (the rejoin) redesigned so its message is obvious: a storyboard of the trip + phase-banded time plots. (done 30 Oct, docs/figures/review_2026-10-30/)
- [x] Simulation screenshots (Figure 22 and any others) replaced by renders from the logged poses (tools/snapshot_render.py). (done 30 Oct, docs/figures/review_2026-10-30/)
- [x] Figure 12 reframed: hover / figure-eight / spinning line, grouped bars 2/3/4 carriers, real (red) vs simulation (blue). (done 30 Oct, docs/figures/review_2026-10-30/)
- [ ] F1 the ring's bending and the band-stop (data exists).
- [ ] F2 the simulated join storyboard (after the re-fly).
- [ ] F5 X1: what the resize is worth.
- [ ] F6 X4: the ring outlives every pack.
- [ ] X2 the chain on three paths.
- [ ] The real-world figure-eight with 2/3/4 carriers at 1 m (after S2).
- [ ] The real-world release at 1 m (after S1/S2).
- [ ] The first real join (if it happens).
- [ ] Optional: F4 the gap. (F3 bands dropped 2 Nov, Wesley: no frozen predictions.)
- [ ] Photos and the mission storyboard: Wesley.

## 5. Reviews and checks

- [ ] A reviewer per claim table, on the final tree, before it goes in the thesis.
- [ ] A critic for anything new in the flight loop (X8 if it touches the network's law).
- [ ] A fact-check pass over each chapter before the supervisor sees it (done 30 Oct for the pack's sections).
- [ ] The re-fly's tables replace the older numbers; no table mixes code trees.

## 6. Admin and housekeeping

- [x] **Commit** the work of 28-30 Oct (committed on week8 by 9f20f8b, 30 Oct 17:31; later work on Wesley's word).
- [x] Merge X4's worktree (30 Oct 16:12, into week8; 37 tests pass).
- [x] Tejen: our reply sent; his review back 30 Oct 14:53 (`~/Downloads/RIG_LATENCY_AND_GEOMETRY_REPLY_20261030.md`).
- [ ] Tejen follow-ups (lab chat): export one rejected low-route candidate per witness type with its full geometry (R2903);
      the real plate's overrides in our launch (collision 0.100/0.100/0.025, his pickup point -0.0045, backend half
      0.050/0.050/0.0125, object centre vs magnet, object_mass_kg stays 0) before any pickup flight; document the frozen-
      marker rule's limit in CHANGES_FROM_TEJEN.md; partial_step re-test in the twin only after the delay paths are separated.
- [x] Wed 4 Nov sheet: `docs/rig/archive/wed_2026-11-04.md`, desk-checked again 2 Nov on the final tree: lines h3t, h3t180, h4m1, sq6b, rel1/2; detection and body
      set settled, his MPC cable typed 1.0 (Wesley), throttle_max 0.95 for the three left after a release; every line
      desk-ok on fake mocap 30 Oct (the release's marker check moved to configs/rig/m1_release_tip.yaml after a refusal).
- [x] Thrust margin (lab chat, exploratory, from logs; done 31 Oct, `docs/analysis/thrust_margin_2026-10-31.md`, feed
      F015): the +9 to +27 % read is refuted as a map error. For S2: type `throttle_max:=0.95` on h3, f3 and ls3 (no
      margin at the default 0.8), packs at 24.0 V or more at rest. Weigh one airframe and the ring on 4 Nov (sheet P5).
- [ ] Backups of the thesis sources (Overleaf is the master).
- [ ] RA-L drafts: parked until after 20 Nov.

## 7. If time runs short, cut in this order

1. P2 simulation (X5-X7).
2. The real-world K and Nr on paths.
3. The 2-quad real-world rows (keep the 28 Oct 0.55 m pair, labelled).
4. X8 to a short note.
5. Real-world line_spin with 4.

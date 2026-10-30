# Everything before hand-in (started 30 Oct 2026; hand-in Fri 20 Nov)

One list of all that is left: the writing (Wesley's steps), the supervisor's feedback, the real-world sessions, the
simulation runs, the figures, the reviews and the admin. Tick items as they close. The order of work is what matters;
the dates below are only the fixed ones (lab sessions, the freezes Wesley set, hand-in).

**Fixed dates**
- Lab sessions, Wed and Thu 8-11 am: 4/5 Nov, 11/12 Nov, 18/19 Nov.
- Code tree freeze Fri 6 Nov; simulation freeze Mon 9 Nov.
- Real-world freeze: aim Thu 12 Nov 11:00, hard limit Thu 19 Nov. Anything flown 18-19 Nov goes in only if clean.
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

**S2, Thu 5 Nov:**
- [ ] Attach x3 (first, if S1 went to the heave).
- [ ] 1 m baselines: h3, f3, ls3, h4, ls4.
- [ ] Release x2 if missing.

**S3, Wed 11 Nov:**
- [ ] Nr in hover x3: release, then Tejen's re-attach.
- [ ] If the attach is unreliable: attach attempts only.
- **Cut line:** nothing below it is counted on for the thesis.

**S4, Thu 12 Nov:**
- [ ] K in hover x2 (k1_7 as fallback).
- [ ] Nr on the figure-eight x1.
- [ ] Missing re-flies.
- Real-world freeze 11:00 (aim).

**S5/S6, 18/19 Nov:** videos and a demo for the supervisor. Results only if clean.

**After every flight:** a registry row; read the log before answering.

## 3. Simulation (lab PC)

All runs are in `docs/sim_runs_todo.md`. In order:
- [ ] X1 release ablation (running).
- [ ] Merge X4, then the short relay check, then the 30-min relays.
- [ ] X2 the chain.
- [ ] X3 the 8-quad set.
- [ ] X3b eight and nine carriers at one iteration per tick (real-time) + the ground computer's tick (before X4's long runs).
- [ ] X8 the paper's method through a loss.
- [ ] Code freeze Fri 6 Nov.
- [ ] X11 final-tree re-fly of every cited table (slope estimate).
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
- [ ] Optional: F3 bands, F4 the gap.
- [ ] Photos and the mission storyboard: Wesley.

## 5. Reviews and checks

- [ ] A reviewer per claim table, on the final tree, before it goes in the thesis.
- [ ] A critic for anything new in the flight loop (X8 if it touches the network's law).
- [ ] A fact-check pass over each chapter before the supervisor sees it (done 30 Oct for the pack's sections).
- [ ] The re-fly's tables replace the older numbers; no table mixes code trees.

## 6. Admin and housekeeping

- [ ] **Commit** the work since 284ffb4 (Wesley's word; nothing is committed): a lot of code, configs and docs from
      28-30 Oct. Recommended before the code freeze at the latest.
- [ ] Merge X4's worktree.
- [x] Tejen: our reply sent; his review back 30 Oct 14:53 (`~/Downloads/RIG_LATENCY_AND_GEOMETRY_REPLY_20261030.md`).
- [ ] Tejen follow-ups (lab chat): export one rejected low-route candidate per witness type with its full geometry (R2903);
      the real plate's overrides in our launch (collision 0.100/0.100/0.025, his pickup point -0.0045, backend half
      0.050/0.050/0.0125, object centre vs magnet, object_mass_kg stays 0) before any pickup flight; document the frozen-
      marker rule's limit in CHANGES_FROM_TEJEN.md; partial_step re-test in the twin only after the delay paths are separated.
- [ ] Wed 4 Nov sheet: S1's order, the stream check, `attach_z` 0.028, catch radius 45 mm, the arming check.
- [ ] Backups of the thesis sources (Overleaf is the master).
- [ ] RA-L drafts: parked until after 20 Nov.

## 7. If time runs short, cut in this order

1. P2 simulation (X5-X7).
2. The real-world K and Nr on paths.
3. The 2-quad real-world rows (keep the 28 Oct 0.55 m pair, labelled).
4. X8 to a short note.
5. Real-world line_spin with 4.

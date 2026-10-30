# Thesis state (the THESIS chat reads this first, via `/thesis`; keep it short and current)

Updated: Fri 30 Oct 2026, 17:16 AEDT. Hand-in Fri 20 Nov. The lab chat keeps `docs/SESSION_STATE.md`; this file is the
thesis chat's own. The shared board is `docs/handin_plan.md`: tick thesis items there.

## Results feed
Last processed: none yet. Start with F001 in `docs/results_feed.md` (the lab chat's feed; read every entry after the
one named here, act, then update this line to the newest entry processed).

## Where the thesis is
- **Overleaf is the master.** Wesley pushes; this chat never pushes. The full draft (1f755ef, 7420c37, c943094) IS on
  Overleaf (checked 30 Oct 17:14): pull Overleaf first and edit there. `~/thesis_full_draft` has one untracked file,
  `Images/results/fig_res_bend.pdf` (F1), to add through Overleaf when F003 is applied.
- **For Wesley's review:** `docs/thesis_review/REVIEW_GUIDE.md` (section by section: pages, status, what to look at;
  F001-F009 mapped to file:line; the three things to review first) and the PDF built 30 Oct beside it. 83 pp, main text
  59 (limit 50), 0 undefined refs, 0 overfull boxes; 11 planned boxes and 4 pending figures in chapter 4.
- **`~/thesis_full_draft`** is a git worktree of `~/thesis`, branch `full-draft-1030`, three commits on Overleaf 9eea11f:
  - 1f755ef: the full draft;
  - 7420c37: the nomenclature as TODO;
  - the four figures of 30 Oct with their text.

- **`~/thesis`:** the old clone (two August local edits in Project_plan / Project_preparations; leave them).
- **Build check, as Overleaf does it:** `latexmk -pdf -outdir=<scratch> main.tex`, with the latexmkrc's nomencl rule.
  Expect 0 undefined references and no overfull boxes.
- **Draft sources and history:** `~/mdc_wt/thesis/docs/thesis_drafts/`:
  - `full_draft_2026-10-30/`: the build folder the commits came from;
  - `thesis_v3_full/Source/src/drafts_1023/` and `drafts_1030/`, each with an index.md;
  - `supervisor_pack_2026-10-31/`: a 30-page pack, optional;
  - `figures_needed.md`.
- **Review figures:** `docs/figures/review_2026-10-30/` (index.md has the captions, data and run ids).

## How it is written (Wesley's rules; details in memory)
- Never "rig" / "twin": write "real world" / "simulation".
- One source line per paragraph; no semicolons in prose.
- Short captions; floats `[H]`; no bold leads.
- Headings in title case (`tools/thesis_titlecase.py`).
- Tables: full borders. Figures: full box, legends below and framed (`tools/plot_style.thesis()`, `legend_below()`).
- Structure:
  - contributions at the end of the literature review (2.8, after the gaps);
  - Results and discussion has no separate Discussion: each result is discussed straight after it;
  - 4.1 is built around hover, figure-eight and spinny line.
- Orange (`\unv{}`) = open item; boxed `planned` = a result still to come; run ids only in comments.
- Everything shown to the supervisor is read and reworded by Wesley first.
- Main text <= 50 pp at hand-in (85 now with the appendix; appendix candidates marked in comments).

## Open in the text
- The arena is 6 x 6 x 5 m (Wesley 30 Oct): `system_overview.tex` line 12 says 6 x 6 x 4 m. Fix it in Overleaf after the push.
- Setup table facts (Wesley 30 Oct): QUAD1-2 CineLog35 V2, QUAD3-4 CineLog35 V3; 1350 mAh packs; masses as measured (0.55 kg,
  0.583 kg with the 1 m cable and magnet); ten OptiTrack PrimeX 13 at 240 Hz; Motive version from the supervisors' paper or
  omitted; every real flight from this laptop. Do not source the rate model's tau 0.12 s / 100 deg/s.
- The supervisors' papers (`docs/papers/supervisors/`, review `docs/analysis/supervisor_papers_review_2026-10-30.md`): cite
  them, reword G4, and the review's "Borrow now" items (listed in `handin_plan.md` section 1).
- The confirmation tolerance (6 mm), settled after the first real join.
- The test-case table's planned join cells.
- The nomenclature (TODO), the abstract, the conclusion.

## Figures
F1 (the ring's bending, `fig_res_bend`, 30 Oct) is in the review folder and both Images/results, not yet in any
.tex: word it as "two low-pass stages halved the bending in the lift", not "removed", and do not claim the band-stop
removes it (it flew only with three carriers, where the bending never showed; four carriers first fly it Wed 4 Nov).
Done 30 Oct and committed: the solve time as flown (replaces the tick figure), Figure 17 (the rejoin storyboard),
Figure 12 (grouped bars), Figure 22 (rendered laps, `tools/snapshot_render.py`). Still to make (figures_needed.md):
F2 the simulated join storyboard (after the re-fly); X1, X4, X2 when their runs
are final; the real-world 1 m figures after S1/S2. Photos asked of Wesley: the ring in flight, a carrier close-up,
QUAD4 with its tether, the earlier setup, the cage.

## Next for this chat
1. Catch up on the feed (F001-F009 at first): apply each to the Overleaf sources, F003/F004 (the bending paragraph,
   now wrong) and F009 (G4, cite the supervisors) first; Appendix B's title still says "Rig".
2. Wesley reviews with REVIEW_GUIDE.md and picks the sections for supervisor feedback round 1 (this weekend).
3. Fold in the feedback; fill each planned box as the lab chat posts a RESULT in the feed.

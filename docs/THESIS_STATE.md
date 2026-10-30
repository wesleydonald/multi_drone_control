# Thesis state (the THESIS chat reads this first; keep it short and current)

Updated: Fri 30 Oct 2026, 14:50 AEDT. Hand-in Fri 20 Nov. The lab chat keeps `docs/SESSION_STATE.md`; this file is the
thesis chat's own. The shared board is `docs/handin_plan.md`: tick thesis items there.

## Where the thesis is
- **Overleaf is the master.** Wesley pushes; this chat never pushes.
- **`~/thesis_full_draft`** is a git worktree of `~/thesis`, branch `full-draft-1030`, three commits on Overleaf 9eea11f:
  - 1f755ef: the full draft;
  - 7420c37: the nomenclature as TODO;
  - the four figures of 30 Oct with their text.

  To put it on Overleaf, Wesley runs `git -C ~/thesis_full_draft push origin full-draft-1030:main`. Once pushed, edit
  in Overleaf (or pull it) and treat Overleaf as the source again.
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
- The confirmation tolerance (6 mm), settled after the first real join.
- The test-case table's planned join cells.
- The nomenclature (TODO), the abstract, the conclusion.

## Figures
Done 30 Oct and committed: the solve time as flown (replaces the tick figure), Figure 17 (the rejoin storyboard),
Figure 12 (grouped bars), Figure 22 (rendered laps, `tools/snapshot_render.py`). Still to make (figures_needed.md):
F1 the ring's bending and band-stop; F2 the simulated join storyboard (after the re-fly); X1, X4, X2 when their runs
are final; the real-world 1 m figures after S1/S2. Photos asked of Wesley: the ring in flight, a carrier close-up,
QUAD4 with its tether, the earlier setup, the cage.

## Next for this chat
1. Wesley reads the draft and picks the sections for supervisor feedback round 1.
2. Fold in the feedback.
3. Fill each planned box as the lab chat ticks its result in `handin_plan.md`.

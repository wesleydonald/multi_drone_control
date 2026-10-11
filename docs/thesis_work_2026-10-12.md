# Thesis and desk lane, 11 October 2026 (evening): hand-back

Three tasks from the brief, in order, each stopped and shown. Branches: `thesis-work2` (from `thesis-rig` 67cb4df;
thesis text, the deck's notes) and `desk-ext2` (from `rig-prep` 6209f7f; analysis and figure scripts). Nothing merged,
nothing pushed; the main working tree, the rig branches, the registry, the lab PC and the rig stack untouched.

## 1. Tuesday's supervisor pack (done, 11 Oct 20:40; orbit video added 21:40)

Slides artifact: https://claude.ai/artifact/TwjqA7ebTDrkgDK6nwutiR (nine slides, about ten minutes: cover, where the
project stands, the rig detaches, the full mission in the twin, the two videos, what flies next, the spine, the three
desk results, the asks). Every number on a slide comes from the registry or the 11 Oct documents; no run ids on the
slides; the hover video (`results/videos/full_M1_twin_R1530_edit.mp4`) and the circle video
(`results/videos/full_M1_twin_R1633_orbit_edit.mp4`, which landed at 20:35) sit side by side on slide 5, click to play.
Speaker notes under each slide. The deck is private until Wesley shares it.

## 2. The signed T6 metrics in the thesis (done, commit 3be85fa on thesis-work2; 11d32ef on desk-ext2)

- `metrics.tex`: the "Detach" item replaced by "Release metrics" (tilt vector, event time, the 5 s pre-window and its
  validity flag, the tilt rise to LAND with pairs cut at the shorter hold, absolute peak and scalar rise beside, time
  back in band) and a paragraph "Resolving a lever pair" (n >= 2, sign, non-overlapping ranges, the max(1 deg, 2 x
  larger range) gap; replication against new prediction).
- `ledger.tex`: "The band" (the band rule as an equation), "Scoring" (hit, width, log ratio; the two naive
  predictors) and "The 14 October bands" with the W4 and W5 table of freeze section 2.4.
- `tools/release_transient.py` (desk-ext2) rebuilt on `tools/release_bands.py`: the twin rows move under 0.05 deg,
  the rig rows rise 0.4-2.6 deg (most on the circle, 5.4 -> 8.0, where the ring's own rocking no longer subtracts
  from the rise), the rig/twin ratios reproduce the freeze's class factors (0.85, 1.05, 1.70, 1.71). Table, plot and
  a one-page summary of the changes in `results/analysis/release_transient/`.

## 3. Leave-chapter figures on the 8 Oct rig data (done; desk-ext2 and thesis-work2 commits of 11 Oct, 21:45)

Rather than new figures, the three figures the thesis plan lists for the 8 Oct data (section 5: "Figs 18-20 updated
on delta theta", "Fig 13 -> F16, placeholder point removed") were rebuilt on the signed metric through the
thesis-figure loop (render, audit, look, one blind critic each, accepted fixes, re-render, audit PASS):

| figure | thesis label | what changed |
|---|---|---|
| F_rig_detach | `fig:rig_detach` (sim_detach.tex) | tilt panels are the tilt departure from the 5 s pre-release mean; the marker and label give the tilt rise (rig 11, 5, 17 deg; twin 6-7, 6, 15-17); release, detection and LAND lines; the dips keep their 3 s window |
| F_rig_circle_detach | `fig:rig_circle_detach` (sim_detach.tex) | panel (c) on the tilt departure: rise 8.0 deg (twin 4.0-5.4, cut at the rig's hold), the first spike 5.7, the band of the rig's own rocking (2 sigma, 4.7 deg) and the time back in band (3.9 s) |
| F_twin_rig_parity | `fig:twin_rig_parity` (res_carry.tex) | panel (a) is the tilt rise, so its ratios are the ledger's class factors (0.85, 1.05, 1.70, 1.71); the attach placeholder pentagon is gone; ratio labels beside every marker; the key moved to the empty corner of (b) |

The two captions and the three citing passages were rewritten to the new numbers (the rig's unannounced hover
detach rose 5 deg, not 4; the circle rise 8.0 against the twin's 4.0-5.4; the parity sentence: the twin predicts
the unannounced tilt rise and the detection to within a quarter, the announced and circle detaches rose 1.7 times
the twin). The PDFs are in `docs/thesis_drafts/thesis_v2/Images/`; captions, section and citing sentence in
`docs/figures/<ID>/caption.md` on desk-ext2; `configs/thesis_figures.yaml` and `docs/figures/PROVENANCE.md` updated.
The thesis builds with no undefined references.

Critic suggestions rejected, with the reason: house-style panel labels (the skill's `panel_label`), a drawn key
instead of the text-only key, recolouring the twin and carrier greys (the RViz palette is kept), the x-label wording,
plain "150 deg" in a text label (siunitx is used), and the "(deg)" axis convention.

Not done, on purpose:
- F10 (the headline), F11 (magnet anatomy) and F12 (levers and margin) need W4/W5, Tier A or the 7 Oct magnet
  data, which the brief excludes tonight.
- `tab:rig_oct8` (the 8 Oct table) and the leave prose in `discussion.tex`, `contributions.tex` and the end of
  `sim_detach.tex` still quote the absolute peak tilt (6.5-17.8 deg, 15.4, 8.1). That is the T8 rewrite of the plan
  ("T8 on delta theta") and belongs to the prose phase from Friday 16; the figure captions say which measure they
  show, so the chapter is consistent as it stands. A tilt-rise column (11, 5, 17, 8 deg) is a five-minute change
  when the table is rewritten.
- The departed-landing figure (`fig:departed_landing`) has no ring-tilt panel, so nothing moved there; the plan folds
  it into F10a later.

## Decisions for the record

None taken by me. Two for Wesley, with a recommendation:

1. The circle detach figure stays a plan view in panel (a) (recommended) rather than 3-D snapshot frames from the
   twin: the plan view carries the rig data; 3-D frames would be twin renders of a rig flight.
2. The 8 Oct table gets its tilt-rise column at the T8 rewrite (recommended), not now, so the chapter's numbers
   change once, with the prose.

## Commits (all authored by Wesley, no trailers, nothing pushed)

- desk-ext2: 11d32ef (release transient on the signed metrics), then the figure commit of 21:45 (scripts, outputs,
  captions, registry, provenance).
- thesis-work2: 3be85fa (metrics and ledger text), then the figure commit of 21:45 (Images, sim_detach.tex,
  res_carry.tex, this note).

Merging: thesis-work2 onto thesis-rig and desk-ext2 onto rig-prep are both fast-forwards from the branches they were
cut from, unless the other session has moved them since; `git merge --no-ff` either way.

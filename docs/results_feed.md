# Results feed: from the lab chat to the thesis chat (started 30 Oct 2026)

The lab and results chat (`/lab`) appends an entry here the moment something the thesis depends on changes. The thesis
chat (`/thesis`) reads every entry after the last one it processed (recorded in `docs/THESIS_STATE.md`), acts on it,
and records the newest one. Append only: a later entry corrects an earlier one, nothing is rewritten.

**Entry format**
```
### F<nnn> <date> <time> <KIND>: <one-line title>
- Section: <the thesis section(s) it touches>
- What changed: <plain words, the numbers>
- Text should now say: <the claim, as the writer should put it; or "keep the planned box">
- Evidence: <card / analysis / figure path; run ids for source comments only>
```
KIND is one of: RESULT (final, ticked in `handin_plan.md`), INTERIM (a first pass: do not write it up yet), CORRECTION,
STALE (a cited result is now in doubt), FIGURE (ready in a review folder), DECISION (Wesley's, changes the text), FACT
(about the setup).

---

### F001 2026-10-30 16:52 FACT: the flight arena is 6 x 6 x 5 m
- Section: System overview (`system_overview.tex` line 12 says 6 x 6 x 4 m).
- What changed: Wesley confirmed the supervisors' wind-mapping paper's figure, 6 x 6 x 5 m.
- Text should now say: 6 x 6 x 5 m.
- Evidence: `docs/decisions.md` 30 Oct.

### F002 2026-10-30 16:52 FACT: the hardware for the setup table
- Section: the experimental platform (the UAV and motion-capture tables).
- What changed: QUAD1-2 are CineLog35 V2, QUAD3-4 CineLog35 V3; 1350 mAh packs; masses as measured (0.55 kg, 0.583 kg
  with the 1 m cable and magnet); ten OptiTrack PrimeX 13 cameras at 240 Hz; every real flight ran from the same laptop
  (the ground computer). The Motive version: from the supervisors' paper, or omitted. The rate model's constants (tau
  0.12 s, 100 deg/s) are not sourced in the text.
- Text should now say: a UAV table and a "how each value was found" table in the supervisors' format.
- Evidence: `docs/decisions.md` 30 Oct; `docs/analysis/supervisor_papers_review_2026-10-30.md` sections 1 and 3.

### F003 2026-10-30 16:15 FIGURE: F1, the ring's bending and the band-stop
- Section: Results, the later setup (fig:res_bend).
- What changed: `fig_res_bend` is in `docs/figures/review_2026-10-30/` and both Images/results folders, not in any .tex.
- Text should now say: with four carriers the ring bends at about 5.3 Hz; two 2 Hz low-pass stages HALVED it in the lift
  (not removed it); their lag let the triangle slide sideways at 1.1 Hz; the band-stop has flown only with three
  carriers, where the bending never showed, so the text must not claim the band-stop removes it (its first four-carrier
  flight is 4 Nov).
- Evidence: `docs/figures/review_2026-10-30/index.md` (fig_res_bend); real-world flights of 29 Oct.

### F004 2026-10-30 16:15 CORRECTION: the low-pass bending numbers
- Section: wherever the draft quotes the two-stage low-pass bending amplitude.
- What changed: the registry's "two stages 0.07-0.09 m/s^2" was measured at the top of the flight, against the others
  in the lift. Over the same lift window two stages give 0.25-0.33 m/s^2 (no filter 0.57-0.62, one stage 0.40-0.47).
- Text should now say: the like-for-like numbers above.
- Evidence: `docs/figures/review_2026-10-30/index.md`.

### F005 2026-10-30 16:20 INTERIM: X1, what the resize is worth (first pass, not for write-up)
- Section: Results, the release ablation (X1's planned box).
- What changed: 63 runs. In releases that leave an even formation (four quads hover and figure-eight, eight quads, the
  1.15 kg ring) a late resize or no resize flies the same as ours: the measured cable pull holds the ring through the
  release. The planned pull is 2.4-4.4 times worse. In the square-to-T release our planner settled the T 7-13 deg off
  level where no resize settled at 2.2 deg: a defect, being fixed before the code freeze (F006).
- Text should now say: keep the planned box. Wesley's framing: X1 is about the resize; measured against planned pull is
  not a thesis comparison (the measured pull is the method).
- Evidence: `docs/experiments/2026-10-30_x1_release_ablation.md`, `results/analysis/x1_2026-10-30.json`.

### F006 2026-10-30 17:00 STALE: any T formed by an unannounced release, in simulation
- Section: X1's square-to-T cell, the chain (X2), the comparison with Quan et al. through a loss (X8), and any draft text
  on our planner's T after a release.
- What changed: the cause of the T's tilt is found (the planner's 0.1 N minimum pull on the T's middle carrier, the
  sideways model integral, and the simulation's rigid rods with mass); a fix is being tested tonight. Real-world T's fly
  with the sideways integral off and are not affected.
- Text should now say: nothing new about the simulated T until the fix's result is posted here.
- Evidence: `docs/analysis/t_offset_forensics_2026-10-30.md`, `t_offset_model_2026-10-30.md`,
  `t_offset_attach_2026-10-30.md`.

### F007 2026-10-30 16:45 DECISION: what the final re-fly keeps
- Section: The Switch of Solvers; 4.1's later setup; the delay lesson.
- What changed: the 16 Oct 0.55 m runs with a 1 m version (ta_*, tb_*) are dropped in favour of their 1 m re-fly;
  dl_d1 and w4_gi are cited as the earlier setup, as flown; the 30 Oct delay runs, the 1.15 kg triangle joins and two
  network smokes stay cited as exploratory; the 1.15 kg fixed-team cells and the simulated joins are re-flown.
- Text should now say: cite the 1 m rows in The Switch of Solvers; label the exploratory rows as such.
- Evidence: `docs/analysis/x11_refly_manifest.md`, `docs/decisions.md` 30 Oct.

### F008 2026-10-30 16:26 DECISION: X8, the paper's method through a loss
- Section: the comparison with Quan et al.
- What changed: X8 compares our planner with our port of their network, told of the loss 0.35 s after it (the authors'
  own failure model); the exact published law's loss row is left out (it cannot meet a loss as built). It flies after
  the T fix. The exact law's hover row is re-flown on the kept force reading (R6c), which gave the same result.
- Text should now say: keep the planned box for X8.
- Evidence: `docs/experiments/2026-10-30_x8_network_loss.md`.

### F009 2026-10-30 15:37 DECISION: cite the supervisors and reword G4
- Section: the literature review (the gaps, 2.7-2.8) and the experimental setup.
- What changed: the supervisors' five papers (`docs/papers/supervisors/`) already fly off-board MPC to stock Betaflight
  over ExpressLRS in this lab; the thesis cites none of them.
- Text should now say: G4 as "extends off-board MPC on stock Betaflight to a cable's pull"; cite them where the setup,
  the thrust model and the MPC are described.
- Evidence: `docs/analysis/supervisor_papers_review_2026-10-30.md` ("Borrow now").

### F010 2026-10-30 17:17 INTERIM: the simulated T's tilt, cause found, fix being chosen (not for write-up yet)
- Section: X1's square-to-T cell, the chain (X2), X8's rim cell; any text on the simulated T after a release or a join.
- What changed: the cause is the planner's 0.1 N minimum pull on the T's middle carrier together with the sideways
  model integral (on by default in simulation), which holds the ring on its point against the simulated rods' weight.
  With the integral off the resize beats flying on without it (settled 0.58 deg against 2.19); with a zero minimum pull
  for a carrier whose level share is zero the T settles at 1.06 deg but slowly. A join into a T tilts the same way as a
  release into one (6.91 deg with the integral on, 0.83 off): it is the formation, not the direction. Real-world T's
  fly with the integral off and are not affected.
- Text should now say: nothing yet; F006 stands until the fix's default is chosen and its claim runs are posted here.
- Evidence: registry rows R2997-R3003 (exploratory), the three analysis files of F006, the card to come
  (`docs/experiments/2026-10-30_t_settle_fix.md`).

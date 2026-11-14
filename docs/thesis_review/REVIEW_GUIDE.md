# Review guide: full draft of 30 Oct 2026

PDF: `docs/thesis_review/thesis_full_draft_2026-10-30.pdf` (83 PDF pages: 10 front, main text pp 1-59, references
60-69, appendices 70-73). Built from `~/thesis_full_draft` at c943094. 0 undefined references or citations. 0 overfull
boxes. That commit is already on Overleaf (origin/main = c943094), so Overleaf is the master again.

| Part | Pages | Status | What to look at |
|---|---|---|---|
| Front matter | i-x | Partly written | Abstract is the 4 Oct text (orange). Nomenclature is TODO. |
| 1 Introduction | 1-2 | Written | Does the question still match the rescope (X1 and X4 headline)? |
| 2.1-2.6 Literature | 3-14 | Written | Skim. None of the supervisors' five papers is cited (F009). |
| 2.7 Research Gaps | 15-16 | Written | G4 claims a gap the supervisors' lab work partly fills (F009). |
| 2.8 Contributions | 17-18 | Written | Will the results back each of the four? |
| 3.1 Platform, Simulation | 19-22 | Written, facts stale | Arena height (F001). The hardware table (F002). |
| 3.2 Model and Planner | 23-25 | Written | Equations and notation. |
| 3.3 Tracker | 26-27 | Written | The low-pass sentence says "removed" (F003). |
| 3.4 Reconfiguration | 28-32 | Written, 1 orange | The 6 mm confirmation tolerance is open. |
| 3.5 Method and Metrics | 33-36 | Written, 1 orange | The test-case table's planned join cells. |
| 3.6 Simulation-only studies | 37-39 | Written | Catch off the plate, rejoin, eight UAVs. |
| 4.1 Fixed Team | 40-45 | Partly written | 1 m rows are planned boxes (4-12 Nov). 4.1.4 bending text (F003, F004). |
| 4.2 Losing Carriers | 46-48 | Partly written | 4.2.3 cites the 0.55 m runs (F007). 4.2.4 X1 is a planned box. |
| 4.3 Gaining Carriers | 49-51 | Partly written | Simulated join is exploratory with a pending figure. No real join yet. |
| 4.4 Composition, Scale | 52-57 | Partly written | 4.4.3 X4 endurance is a planned box. |
| 4.5 Quan et al. | 57-58 | Written, exploratory | No X8 loss row yet (F008). |
| 5 Conclusion | 59 | Planned box | Blank until the results freeze. |
| Appendices A-E | 70-73 | Written | Appendix B's title says "Rig". |

## Feed entries F001-F009 (all not yet applied)
Paths are under `~/thesis_full_draft/Source/`. `rd` is `src/drafts_1030/results_discussion.tex`.
- **F001** `src/system_overview.tex:12`: "6 x 6 x 4 m" becomes 6 x 6 x 5 m.
- **F002** `src/system_overview.tex:12-14` and the setup table `:48-70` (mass row `:57`): add a UAV table (QUAD1-2
  CineLog35 V2, QUAD3-4 V3, 1350 mAh, 0.55 and 0.583 kg, ten PrimeX 13 at 240 Hz, one laptop) and a "how each value
  was found" table. The rate model at `src/controller.tex:43-46` stays unsourced.
- **F003** `rd:197` and `src/controller.tex:65`: two 2 Hz stages halved the bending in the lift, not removed it. Their
  lag let the triangle slide at 1.1 Hz. The band-stop flew only with three carriers, so no claim that it removes the
  bending. `rd:206` is the pending box for `fig_res_bend` (the PDF is in Images/results, untracked).
- **F004** `rd:197` (and the comment `src/controller.tex:69`): replace "0.43-0.60 to 0.07-0.09 m/s^2" with the lift
  window: none 0.57-0.62, one stage 0.40-0.47, two stages 0.25-0.33.
- **F005** `rd:342-349` (4.2.4, box `:348`): interim. Keep the planned box. No write-up yet.
- **F006** Hold any new text on the simulated T after a release. Check `rd:304-330` (4.2.2 gap table, a 180 deg gap
  leaves a T), `rd:348` (X1), `src/drafts_1030/test_cases.tex:41` (the chain) and 4.5 (`rd:515-545`).
- **F007** `rd:331-341` (4.2.3): cite the 1 m re-fly rows, not the 0.55 m ones. Label the 30 Oct delay runs (`rd:210`)
  and the 1.15 kg triangle joins (4.3.1, `rd:356-381`) as exploratory.
- **F008** `rd:515-545` (4.5, table `tab:res_compare`): add a planned box for X8 (ours against the port told 0.35 s
  late). No loss row for the exact law. The exact law's hover row (`rd:544`) stands.
- **F009** `src/lit_gap.tex:50` (G4): "extends off-board MPC on stock Betaflight to a cable's pull". Also `rd:193`. Cite
  the supervisors at `src/lit_execution.tex:10`, `src/system_overview.tex:12-14` and `src/controller.tex:36-46`.

## Review these three first
1. **The bending paragraph** (p 44, `rd:197`, and p 26). It says the low-pass removed the bending and quotes numbers
   from the wrong window. It is a real-world claim that is now wrong (F003, F004).
2. **G4 and the supervisors' papers** (p 15-16). The gap is phrased so that their own lab work seems absent. Your
   supervisors will read this first (F009).
3. **Length.** The main text is 59 pages against the 50-page limit, with 15 planned boxes or pending figures still to
   fill. Choose what moves to the appendix (candidates are marked in source comments).

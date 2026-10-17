# Thesis v3: what changed, and what is yours to decide (17 Oct 2026)

Two new versions sit beside `thesis_v2`, which is kept unchanged apart from a new `REWRITE_MAP.md`:
- `thesis_v3_full/`: the best version with no page limit. Every item of the brief is fixed, the Chapter 4 skeletons
  are merged into real sections, and repetition is cut.
- `thesis_v3_50/`: the same content condensed toward 50 pages of main matter. Detail moves to the appendices; no
  claim is dropped.

The plan for both, one row per section, is `thesis_v2/REWRITE_MAP.md`. Both versions build clean: no errors, no
undefined references or citations, no float too large. The only overfull boxes are 1.3 and 1.7 pt, in two tables.

## Page counts

| Version | Main matter (Introduction to Conclusion) | Front matter | References | Appendices | Total PDF |
|---|---|---|---|---|---|
| v2 (12 Oct) | 75 pp | 12 | 11 | 24 | 122 |
| v3_full | 76 pp | 11 | 10 | 20 | 117 |
| v3_50 | 50 pp (page 50 holds the last 10 lines, so about 49.2) | 11 | 8 | 34 | 103 |

v3_full is no shorter than v2 (main-matter text about 35.6k words against 33.9k; v3_50 about 23.4k). The legacy material went out (the network comparison, the 4-5 Oct tables, the redo
boxes). The new evidence came in: the carry envelope, the levers of a release, the switch numerics, the ledger table,
the 14-15 Oct rig losses, the join placeholders and the landing finding.

The template's comment in `title_page.tex` sets what counts. It limits the report to 50 pages "excluding appendices,
bibliography, cover sheet, abstract, acknowledgements, nomenclature, contents and lists of figures and tables". So the
count that matters is the main matter (decision 2).

## Your decisions (recommended option first)

1. **The AI disclosure (this conflicts with the brief; I kept the text).** The brief says never to mention AI. But
   v2's protocol paragraph, and your thesis plan of 11 Oct (section 3.6), describe the critic and the reviewer as
   language-model agents and keep a slot for the disclosure UNSW requires. Removing that would hide how the claims
   were checked.
   - *Recommended:* keep it as it is, in both versions (`Source/src/simulation.tex`), and fill the disclosure in the
     form the University asks for.
   - *Alternative:* call them "automated checks" in the text, and make the disclosure only in the front matter.
2. **What the 50 pages count.**
   - *Recommended:* the main matter only, as the template says; v3_50 meets it with about one page to spare once the
     join results are in.
   - *Alternative:* check the course guide. If the whole PDF counts, a different plan is needed: about 50 pages would
     have to go.
3. **The project-plan and preparations chapters.** v2 already left them out of the build: `main.tex` never input
   them.
   - *Recommended:* leave them out. Their aims and experiments E1-E4 are replaced by the three questions, and the
     Gantt is history. They are not copied into v3.
   - *Alternative:* a one-page appendix with the original aims and the timeline.
4. **How much literature in v3_50.** The literature runs 10 pp in v3_full (pp 5-14) and 6 pp in v3_50 (pp 4-9). Your plan said 8.5;
   the report guide's indicative length is about 15.
   - *Recommended:* the v3_50 cut, with the membership table kept whole, and a sentence to the supervisor that the
     split is deliberate.
   - *Alternative:* restore sections 2.3-2.4 from v3_full (+1.5 pp).
5. **Page breaks between chapters (v3_50).** v3_50 drops the `\newpage` before Chapters 3, 4 and 5, which saves
   about 1.5 pages.
   - *Recommended:* keep them dropped.
   - *Alternative:* restore them if the template wants each chapter on a new page; v3_50 would then be about 51.5
     pages.
6. **The horizontal integral's force.** The reviewer's must-not-say list says "that the integral learns the push, or a
   share of it". The appendix re-fly's own table says to keep "learns the applied force" for the horizontal push only
   (0.32 of 0.40 N). I read the list as vertical only. The text avoids the word "learns" and gives the horizontal
   force as a measured value (`res_steady.tex`, External forces).
   - *Recommended:* confirm that reading.
   - *Alternative:* drop the 0.32 N from the prose and keep it only in the table.
7. **Which figures stay in v3_50's main text.** Moved to the appendix in v3_50: the takeoff flight, the plan view of
   the twin's paths, the thrust map, the rig solve times, the horizontal push, the 8 Oct release comparison, the
   one-release-in-the-twin figure, the freed UAV's landing, the twin-rig parity figure, the releases strip, the owners
   diagram, and T8a.
   - *Recommended:* as is, one figure per claim in the main text.
   - *Alternative:* name the ones you want back. Each costs 0.4-0.6 pp.
8. **The 14-15 Oct uncommanded releases (L10, L11).** They are in the release record but not scored on the frozen
   metric.
   - *Recommended:* score them with `tools/release_bands.py` from their bags (desk work, after tonight's batch), then
     fill the bracketed cells.
   - *Alternative:* leave them as text only.
9. **The reported carry columns** of the twin carry table (path-only RMSE, path-window tilt, cut, across-path rms,
   lag). They come from the corner-cutting rescore of 16 Oct, a committed tool. Today's reviewer checked every row
   against that file, but no card's reviewer has passed the 0.125 and 0.2 m/s rows.
   - *Recommended:* one reviewer pass on those columns before the thesis freeze.

## What was fixed (the brief's list)

1. **Vertical claims (C-D2, C-D3, C-D5):** the disturbance and robustness text is rewritten around the measured pull.
   - A vertical push or a mass error leaves no offset on the measured pull. With the planned pull (the exploratory
     pair) the push leaves 8.0 cm and +20 % mass leaves 22.2 cm, and both are labelled exploratory.
   - The 8.4 cm is gone.
   - The radius row's 0.76 N is described as wind-up, not learned: the column is relabelled "integral force at LAND"
     with a footnote.
   - "Every landing does not tip" (C-L1) is gone: the land table now counts the 4 of 82 false touchdowns.
   - The integral's vertical channel is called a liability in the discussion.
   - Every item of the re-fly reviewer's must-not-say list is honoured.
2. **Carry:**
      - The three-carrier claim is the new table T7t: 39/40 runs pass; spin at 0.3 m/s is outside the envelope; the 1 m
     cables carry as well in the five cells flown (up to 0.2 m/s), with the window and LAND caveat.
   - The scored column is now called "ring RMSE" (from takeoff + 5 s to LAND), as the carry reviewer asked, and the
     path-only RMSE sits beside it.
   - The clock-free cut, the across-path rms and the lag in seconds are reported beside the frozen columns.
   - Corner cutting is one appendix paragraph: the defaults stay, the timing fix failed on the spin.
   - The carry reviewer's must-not-say list is honoured: "peak path speed"; no "path-tracking RMSE"; no trend read
     from peak tilt.
3. **Detach:**
   - T3/T8/T9/K1/T15 (Tier A), Tier B, the hover/circle release cells and drop and land D1/D4 are the Chapter 4 tables
     and figures.
   - F10b and F12 come from `docs/figures`. T10 and T13 are the generated tables, with shorter captions.
   - The full lever table T9 is in the appendix.
   - The reviewers' corrections and must-not-say lists are honoured: no "W4 n = 3", no K1 credit, no "a third", B4
     falsified as written.
4. **LAND:** one paragraph and a limitation.
   - The false touchdown (2 of 4 on the rig's LAND path), the re-arm (0 of 5, twin default, the rig after review) and
     the 16 cm residual with "a fix under test".
   - The landing table keeps its freeze tree.
5. **Join:** T11, F14, F15 and T12 are placeholders sized for the join code tree.
   - The old contact-and-lift T11 numbers and F14 figure are out (one line of history in the appendix).
   - The method section now describes the co-motion confirmation.
   - The 14-15 Oct rig attach record is in as exploratory rig findings.
   - Tonight's join re-fly is not used.
6. **Stale or legacy rows:**
   - C1, D6, C4, A8, C5, C9, D10, F3, the weld ladder, the ground join and the old join claim are one line each in
     "Dated history".
   - The network (D9) is a one-page appendix with one comparison table, and contains no "dissipative" wording.

Rules applied throughout:
- no run ids, flight names, SHAs or branch names in visible text (checked by a lint script);
- no "dissipative" (now "network controller", "passivity-based", "detach mode", "carry mode");
- plates clockwise (carriers 3/11/7 and the leaver on 9; old counter-clockwise p is (12 - p) mod 12). The geometry
  figure is redrawn clockwise with the rods at 60 deg, and every caption maps the counter-clockwise angles that older
  figures print;
- blank-line paragraph breaks kept.

Also fixed:
- the `ulem` package underlined every `\emph` in v2 (now `[normalem]`);
- the geometry figure's rods were drawn at 45 deg;
- the abstract's "settled within a second" was stale.

## What moved, what was cut (one line each)

v3_full against v2:
- Cut: the redo boxes and their commented legacy text; the Chapter 4 skeleton boxes; tab:sim_vs_rig (stale, F3);
  tab:solve_times (twin solve times are load, not the solver); fig:tracker_x0 and fig:landing (their freeze-tree
  re-renders clip or overlap; the tables carry the claims); tab:detach (replaced by T3/F10b); tab:plates (one rule
  now); fig:attach storyboard (legacy).
- Rewritten: the abstract, contributions, the model-integral text, takeoff/landing (the re-arm), reconfiguration
  (co-motion), authority (switch states of 17 Oct), metrics (the clock-free ruler), 4.1 (envelope, rig, steady
  errors), 4.2 (rig record and the 14-15 Oct losses, levers, switch, freed UAV, drop), 4.3 (placeholders and the rig
  record), 4.4 (T13, the 1 m bands), 4.5 (new paragraphs on the 1-DOF signs and the magnet margin), the Conclusion.
- Moved to appendices: fig:land_phases and fig:disarm_lanes (fault coverage), the tip loop (the twin's own approach),
  the estimator comparison (earlier stack).
- Appendix F (the network, 4.4k words) is condensed to one page.
- New: the lever table (T9), the 1 m carry bands, corner cutting, the fault-coverage table, dated history, and
  timeline rows to 17 Oct.

v3_50 against v3_full:
- Introduction condensed (the motivation in two paragraphs; each contribution in a few lines).
- Literature condensed to 6 pp: the link models and the centralised and distributed work in one subsection; the
  failure, joining, plug-and-play and statics paragraphs halved; capture, switching, execution and sim-to-real each one
  paragraph.
- Methodology to about 14 pp. To a new appendix "Method details": the tracker's OCP, the integral's properties and
  reference-shift form, the ramp, seed, fall-backs and rejoin mission, the constants table, the metrics in full,
  "why these bars", and the owners diagram. Takeoff and landing become one paragraph; the payload free-body figure is
  cut (the geometry figure carries the symbols).
- Results: the secondary figures go to a new appendix "Secondary figures and results" (list in decision 7), with the
  1 m rows of T7t, T8a and the no-event floor paragraph. Steady errors: five paragraphs become one ("Other sources"),
  and the sources table is shorter. The captions of the main figures are shortened.
- The discussion is condensed; the literature table loses Lee and Gabellieri (no position number) and the solve-time
  column.

## Still placeholders

Join, after the 21-22 Oct rig day and the join code tree:
- T11 (twin claim, 8 lines), F14 (confirmation timeline), counterfactuals, false accepts/rejects over the rig bags;
- F15 (rig attach), T12 (full mission: 2 hover runs under a card, circle exploratory);
- the T10 join row, the abstract's join sentence, and the RQ3 answers in 4.3 and the Conclusion;
- the membership table's join cell, and the literature table's "added" cell.

Rig:
- T7 K3, K4, K7 (the measured-pull repeat, the 60 deg planned-pull falsifier, the 3-min hover);
- T8 L8, L9 (hover and circle releases against their bands) and the L10, L11 frozen-metric cells;
- F16 (the ledger) and fig:rig_departed;
- the rig photo; the magnets' hold against the angle of pull.

Other:
- the four-UAV solve median against the 60 ms budget;
- the stretch-threshold false alarms over the rig recordings;
- the catch radius in the twin (parameters table);
- the bracketed literature cells (Bacheti, Liang, Saiella, Farras, Hajieghrary: full texts needed);
- the roles wording with Te-Jen and the supervisor;
- the nomenclature (template placeholder);
- the acknowledgements.

## Figures that need the thesis-figure pass

Re-rendered on the freeze tree on 17 Oct by `tools/thesis_sim_figures.py` (house style, but no look-revise rounds,
audit or critic): flight, trajectories, detach, disturbance.

From `docs/figures` (16 Oct; the loop's audit and critic are not filled in): F10b crossover, and F12 levers, whose
panel (b) label "drop rule (absolute tilt)" collides with "planner's pull, dropped".

To regenerate (cannot be done tonight, the laptop is busy):
- rig_circle_detach and magnet_anatomy print counter-clockwise angles (each caption maps them to plates);
- disarm_lanes prints "rig, 14 October: pending" (its caption now says not yet flown);
- land_phases shows the 9 Oct attach sequence ("weld"), labelled in its caption;
- tracker_x0: the hold panel clips the +13 cm trace;
- landing: overlapping labels in (a).

## Notes

- **The overnight batch will touch three passages.** The no-floor LAND rev 3 is the "fix under test" in the landing
  paragraph and the limitations. The radius +2 cm wind-up runs bear on the wind-up sentence. The join re-fly is not
  thesis evidence.
- **Review before the commit (read-only, against the cards, the registry and the analyses).**
  - First pass on v3_full: *weak*, 36 findings. Every number in every table matched its source, and the plate
    numbering was right. The prose overclaimed. The main ones:
    - the corner cut blamed on the plan's age: a lead-equals-delay test had failed, so its source is open;
    - "the pull source sets the transient" from one twin cell: now the sign only, in two cells;
    - the rig's 3 cm credited to the measured pull alone: six settings changed together;
    - the ledger described as if rig rows were scored: none is yet;
    - "3 % stronger": now a throttle offset 0.03 below the map, about 1 m/s^2;
    - the counter-swing count: 9 of 9 survivors of three releases;
    - the L2 step: up to 2.75 N within a second, +1.8 N at the first solve.
  - All 36 were fixed in both versions.
  - Re-check: 30 fixed, 6 partly, plus 8 slips introduced by the condensation, one of them wrong (v3_50 said the
    breakaway cap was released at the lift on 1 October). All of these were fixed too; there was no third pass.
- **Renamed:** Section 4.4 is now "The prediction ledger" (it was "What the twin predicted").

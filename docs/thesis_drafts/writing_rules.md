# Rules for writing this thesis

What the marker rewards (rubric, Final Thesis Report): literature in context 10 %; **execution, analysis
and discussion 50 %** ("a coherent engineering story with excellent outcomes"; top band: "something
unexpected, thoughtful and original"); **conclusions and value added 20 %** (publishable; limitations;
new avenues); **presentation 20 %** (clear structure, easy to read, consistent and *economical* figures,
tables and references). Every rule below serves one of those four.

## 1. The story

1. **One question, stated early, answered at the end.** The introduction states the research question
   and two to four sub-questions. Each results section opens by naming the sub-question it answers and
   closes with the answer in one sentence. The conclusion answers them in the same order.
2. **Question -> method -> evidence -> answer, at every level.** A chapter, a section and a paragraph
   each follow it. If a section has no question, it is background and belongs earlier or in an appendix.
3. **Each chapter has one job.** Literature: what is known and what is missing. Methodology: what was
   built and why each design choice was made. Results: what happened, against what baseline. Discussion:
   what it means, what it does not show, how it compares with the literature. Conclusion: the answers
   and the value added. Do not mix them (no results in the method, no new methods in the results).
4. **Lead with the claim.** The first sentence of a section or paragraph says what it shows; the rest
   supports it. A reader who reads only first sentences should get the thesis.
5. **Signpost, do not narrate.** One sentence at the start of a chapter on what comes and why in this
   order; no "in this section we will", no chronology of how the work was done ("first we tried...")
   unless the failure itself is the finding.
6. **Hardware is the evidence, simulation is the instrument.** The rig results are the strongest
   claims; the twin explains and de-risks them. Present the simulation as a tool with a stated
   fidelity (what it reproduces of the rig, what it does not), not as the end result.

## 2. Claims and evidence

7. **Every claim has a figure, a table or a citation behind it.** A sentence with a number points at
   the table or figure it comes from. A claim with no evidence is cut or marked as a hypothesis.
8. **Every number has a baseline.** "Tilt 8 deg" means nothing; "8 deg against 15 deg with the old
   rule" does. Name the baseline in the caption or the table header.
9. **Repeats and spread.** State how many runs each number rests on; give the range or both values
   for two repeats; never a single run presented as a result without saying so.
10. **Falsifiers and failures are results.** A result that failed its bar, a twin artefact, a void run
    and why: report them. A thesis that shows only successes reads as cherry-picked (rubric: "convince
    of its correctness").
11. **Separate the measured from the inferred.** "The ring tilted 57 deg" (measured) vs "because the
    newcomer fell onto the rim" (inferred, say from what).
12. **No run ids, file names or code paths in the text.** They live in the LaTeX comments, the
    figure provenance file and the registry.
13. **Precision follows the data.** Two significant figures unless the measurement supports more;
    units on everything (siunitx); degrees with the degree sign.

## 3. Figures and tables

14. **A figure proves one sentence.** Write that sentence first (it usually becomes the first line of
    the caption). Everything in the figure serves it; everything else goes.
15. **The caption makes the figure readable alone:** conditions (mass, layout, controller), what the
    lines, fills and markers mean, the key numbers. No run ids.
16. **Draw every number in the caption** (a reference line for a bar, a bracket for a duration, an
    annotation for a peak).
17. **Colour only what the claim is about.** Drones keep their RViz colours everywhere; the payload is
    always the payload colour; context in grey. One palette for the whole thesis.
18. **Comparisons side by side on shared axes**, baseline in grey, method in colour; repeats overlaid
    thin, mean thick; the twin and the rig on the same axes where they are compared.
19. **Tables for exact numbers, figures for shapes and trends.** Never both for the same numbers.
20. **House style for every figure:** built at print size (full 6.27 in, half 3.03 in), 9 pt text,
    vector PDF with embedded fonts, panels labelled (a), (b), no titles inside the figure, no twin y
    axes; run the thesis-figure checklist and audit before a figure goes in.
21. **Every figure and table is cited in the text before it appears** and discussed: what to look at,
    what it shows.
22. **Pending data gets a placeholder, never a gap.** Use `\pendingfig{label}{caption}{run that fills
    it}`: the caption and label are final, the box says which run fills it, so the text around it can be
    written now and cross-references work.

## 4. Language

23. **Plain, specific, active.** "The planner resizes the OCP" not "resizing of the OCP is performed".
    Short sentences; one idea each. Prefer the concrete noun to the abstract one.
24. **One term per thing, everywhere.** Keep a glossary of the project's names (newcomer, carrier,
    rig twin, plate, weld, detach, rejoin, ...) and use exactly those. Define each at first use.
25. **Consistent notation.** One symbol per quantity, defined in the nomenclature; vectors bold;
    frames named the same in text and figures.
26. **No hedging filler** ("it can be seen that", "it is worth noting", "very", "quite"); no
    overclaiming ("proves", "robust", "optimal") without the evidence that earns the word.
27. **Tense:** present for what the thesis shows and for the method; past for what was flown on a date.
28. **Cut until it hurts.** If a paragraph can lose a sentence without losing a claim, it loses it.
    Economy is marked (presentation, top band).

## 5. Literature in context

29. **Synthesise, do not summarise.** Group by idea (models, centralised vs distributed control,
    reconfiguration), compare the works with each other, and end each group with what it leaves open.
30. **The gap is specific and the thesis fills exactly it.** The last paragraph of the review states the
    gap in the words the introduction's question uses.
31. **Return to the literature in the discussion:** compare the numbers (tracking error normalised by
    path size, tilt, transient after a dropout) with the closest works, like for like.
32. **Recent and peer-reviewed first**; the last five years for the core; preprints marked as such.

## 6. Discussion, limitations, conclusion

33. **Discuss meaning, not results again.** What the result implies, why it came out this way, where
    it would fail, how it compares with the literature.
34. **Limitations are specific and owned:** what was not flown (and why), where the twin is known to
    differ from the rig (rigid links vs slack tethers, ...), single runs, assumptions. Each with its
    consequence for the claims.
35. **Future work follows from the limitations and opens new avenues**, not a to-do list.
36. **The conclusion adds no new results.** It answers the questions, states the contributions (what
    is new, who could use it) and ends on the value added.

## 7. Process

37. **Write the figures and the captions first, then the text around them.**
38. **One change of structure at a time; build after each.** The document must compile clean
    (no undefined references, no overfull boxes over 1 pt in the body).
39. **Every result section goes past a reviewer** (someone who checks each number against the data)
    before it is final.
40. **Read it as the marker:** for each rubric criterion, point at the page that earns the top band.

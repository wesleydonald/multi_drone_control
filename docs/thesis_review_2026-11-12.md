# Thesis review, 12 Nov 2026

**Reviewed:** Overleaf version a0b8ec5 (77 pages, 27 figures).

**Method:** seven read-only agents worked in parallel:
- W1: abstract and Chapters 1 and 2;
- W2: Chapter 3;
- W3: Section 4.1;
- W4: Section 4.2, Chapter 5 and the appendices;
- F1 and F2: figures 1–14 and 15–27;
- E1: an examiner with no project context.

The raw findings, about 230 items, are in `docs/thesis_review_2026-11-12_raw/`.

**Checks done here:** every "must" below that states a fact was checked against the source, code, configs, run parameters or analysis files. Each one is marked **[checked]**, or **[agent]** where the agent showed its own evidence (file:line) and I did not repeat the check.

**Not repeated:** wording findings are kept as the agents wrote them and need no data check.

---

## 1. The examiner's view (read this first)

**Verdict:** the system work is strong and the reporting is unusually honest. The framing claims more than the data
show. The examiner would pass the thesis on the integration work but asks for the claims to be matched to the results.

The concerns that need **your decision**, not a wording fix:

| # | Concern | What the data say | Options |
|---|---|---|---|
| D1 | **Resizing is the headline (C1), but Table 13 shows it does little.** The first swing and height loss are the same as without resizing. It adds a larger second swing and does not settle sooner. The only gain is the settled tilt in hover (0.03 vs 0.22°, about 1 mm). The baseline is also never defined: what does it do with the detached quadrotor's planned tension? | [checked] Table 13 as written. The baseline's behaviour is not stated in 4.1.2. | (a) State what the baseline does and which loop moves the load (the trackers' measured tension), then reframe C1 around what resizing enables: adding a quadrotor at all, limits that stay consistent, and the level settled ring. (b) Keep it as is. |
| D2 | **Real-world attachment evidence is thin.** There are two confirmed catches, no real-world attachment figure, and the confirmation rule's accuracy (false accepts and rejects) is never measured. | 12 Nov adds the full mission and 3 of 3 drops with re-attachment (F103, F104). | Write in 12 Nov (planned). Add a real-attachment time series (the tip in the plate frame, the lift, the confirmation, the radio switch, the tensions and tilt). Add a confusion table of contacts against confirmations from the logs. Word C3 as the confirmation and hand-over of an attachment flown by the collaborator's controller. |
| D3 | **The simulation is optimistic, and the real-vs-sim comparison is confounded.** The real flights used 0.55 m cables against 1 m, larger paths, half speed for the four-quadrotor figure-eight, an earlier software version, and the best of several flights. The 8-quadrotor and relay results use an idealised joint attachment and do not run in real time. | [checked] as stated in 4.2.1, 4.1.3 and 4.1.4. | Say these limits in the abstract and conclusion. Add a small table of what differs next to Fig 26. |
| D4 | **The degree on the title page reads "Bachelor of Engineering in Mechatronics Engineering".** | [checked] title_page.tex:20 | Confirm or correct it. |
| D5 | **"Averaged over two runs" with no spread,** while differences of 0.3° or 1 mm are interpreted. | This conflicts with your 9 Nov choice of averages. | (a) Keep the averages and stop interpreting differences that fall inside the run-to-run spread. (b) Add the spread as min–max in brackets. |
| D6 | **The simulated studies with returns flew the earlier mass split** (relay, sequences, the returning quadrotor), while the other studies flew the measured split. The text never says so. | [checked] the source comments and the 6 Nov decision. | One sentence before Table 15, or leave it unstated. |
| D7 | **The "measured cable tension" is really a model residual** (it absorbs any unmodelled force), and C4 has no ablation. | [agent] | Rename it "estimated cable force", or add one sentence saying what it contains. A cheap check from the logs: the hover estimate against the static tension (share × 1.15) per quadrotor. |
| D8 | **The introduction is one page.** It has no explicit research question, objectives, scope (indoors, motion capture) or outline, and the contributions are not mapped to the gaps. | [checked] | Add a research question, the scope, and the gap each contribution answers. You removed roadmap sentences before, so an outline is your call. |

---

## 2. Must fix: facts and contradictions (checked)

1. **The drop rule in 3.4.2 is wrong** [checked: code and flown parameters].
   - **What 3.4.2 says:** "fewer than three quadrotors, or ... largest gap is 200° or more".
   - **What the code does:** the default is fewer than three quadrotors or a gap of **180°** or more, with a 30° tilt drop.
   - **Where 200° comes from:** it is the *relaxed* rule (two quadrotors allowed, gap up to 200°), used by the sequences and the layout (e) runs. "The relaxed drop rule of Section 3.4.2" (4.1.3, 4.1.4) points to a rule that 3.4.2 never defines.
   - **Fix:** state the default and the relaxed rule in 3.4.2, and name in each study which rule it used.
2. **The layout (e) settings are not stated** [checked: R3893 parameters]. The X1 harder detachment flew:
   - the relaxed drop rule;
   - a tilt drop of **80°**;
   - the trackers' **60° tilt disarm switched off**;
   - the horizontal disturbance estimate off.

   This is why Table 12 (180° tipped or dropped) and Table 13 (layout (e) settles at 1.75°) look contradictory. The 60° disarm is never described in Chapter 3. That disarm is what "the quadrotors disarmed in the air" refers to.
3. **"Maximum tilt" in Tables 12 and 13 is the tilt rise,** the change from the mean before the event, not the peak tilt [checked: x11_gap and x1 jsons].
   - Fig 14(a) plots the absolute tilt, which peaks at about 6°, while the table gives 6.7–7.0°.
   - The text's "peaked at 29.8°" (absolute) sits beside "Over 30" (rise).
   - The same quantity is 6.3° in Table 12 and 6.7° in Table 13 for layout (d) in hover. These are different runs.
   - **Fix:** go back to "Tilt rise" and define it, and keep "peaked at" for the absolute tilt.
4. **Tables 17 and 18 disagree for the same runs** (R3972/R3973) [checked: tab_averages.py].
   - Table 17 gives the worst of four, averaged over the runs (2.38°). Table 18 gives the mean over all eight events (1.93°), under the same label "Tilt rise at a detachment".
   - The reattachment is "Tilt rise 1.43" in Table 17 against "Tilt peak 1.50" in Table 18.
   - **Fix:** use one measure and one label (my error from the averaging pass).
5. **The 8-quadrotor timing numbers are mixed** [agent: registry].
   - 50 ms (bench, residual every 10th iteration), 66 ms (Table 18, solve p95, lab computer, simulation running), 83.1 ms (Table 16, older bench setting) and 261 ms (control step, ground computer, simulation running).
   - These are different computers and different measures (control step or solve).
   - **Fix:** one timing paragraph that defines each number. Label Table 18's row "Solve time, lab computer".
6. **The conclusion is out of date or over-claims** [checked against Ch 4]:
   - "21 to 28°" should be 15.6° on average (Table 22).
   - "more level ... by about 0.1 to 0.2°" holds only in hover with four quadrotors (0.03 vs 0.22). It does not hold in layout (e) or on the figure-eight.
   - "the ring kept flying whenever the remaining quadrotors could support it" is contradicted by the 165° drop, the false-detection drop during the landing and the drop from an object's impact.
   - "the re-attachment to three quadrotors never caught" is wrong: layout (e) caught, and the funnel flights caught.
   - "0.25 to 0.44 s in simulation" should be about 0.27 to 0.43 s across Ch 4.
   - "the first solve after an unannounced detachment never failed" and "all but one on the first contact" are not in Ch 4 as stated.
7. **The layout (e) lean in 4.2.1** [checked]: 15.6° (real) against 11.2° (simulation). Those simulated runs (R3708, R3712, R3714, R3715) flew with the horizontal estimate **on**, unlike Table 13's layout (e) at 1.75°. Say so.
8. **The thrust map, 3.3.2** [checked: arithmetic]: "RMS error of 0.004, which is about 0.15 N". 0.004 × 19.4 N ≈ 0.08 N. The 0.15 N belongs to the 0.008 maximum.
9. **Speeds and sizes, 3.5.1 against 3.6.3** [checked: the configs].
   - "0.2 m/s was used for all trajectories" contradicts the Quan comparison at 0.4 m/s.
   - That comparison flies a 0.9 m line, while Table 8 gives d = 0.75 m (the real-world size).
   - The simulation's path sizes are never given (common.yaml: a 0.9 m figure-eight and a 1.0 m line). The circle in the results is r = 0.3 m, against 0.5 m in Table 8 [agent].
10. **The motion-capture recording, 3.1.2** [checked: text; decisions 5 Nov]. "Recording a video on the computer ... So the lab computer did not run anything" is wrong: it was a recording in the motion-capture software on the motion-capture computer. The same fact is repeated in 4.2.1.
11. **Table 2 (literature, p. 6) is stale** [checked]. Its "This work" attachment row says "simulation, 1.15 kg, tilt at most 2.5°". It is now Setup 3's ring at up to 1.9°, and the real-world attachments are missing.
12. **Appendix B is stale** [agent; checked against decisions]:
    - the detection margin says 0.06 only, but later Setup 3 flights used 0.15;
    - the position limit is now 1.8 m (12 Nov);
    - Setup 1's tension cap was 12, not 14.5, until 21 Oct;
    - the larger 0.9 kg ring of the 8-quadrotor studies is missing;
    - there is no row for the drop rule's tilt.
13. **k_T = 35.2 m/s²** was identified at 0.55 kg but flown on the 0.583 kg quadrotors of Setups 2 and 3, and this is never explained [agent: real.yaml].
14. **Figure 26** [agent]: the simulation bars read 8.7 and 3.8 against Table 11's 8.6 and 3.7, and panel (c)'s title is clipped ("contour erro"). Figure 25 is never cited.
15. **Figure 18** [checked by eye]: the titles of (a) and (b) are clipped at the top, and those of (c) and (d) run into the row above. The red-to-blue scale also clashes with Figure 17 on the same page, where red means the plate's centre. Your colour choice: keep it, or change one.
16. **Figure 13(b)** [agent]: the figure-eight is drawn about 4.6:1 instead of the 2:1 that Table 8 defines.
17. **Figure 14/Table 13 measure mismatch:** see item 3.
18. **Typos that change meaning:**
    - "The peaked at about 58°" (4.1.3);
    - a sentence fragment at the start of the returning-quadrotor method (4.1.3);
    - "The harder detachment (layout (e)) in hover, added a larger second swing" (the subject is wrong: resizing added it);
    - "a collaborators controller" (C3);
    - "over a ExpressLRS";
    - "modified to handed tethered quads" (acknowledgements);
    - straight quotes around "reconfiguration" (2.2), which print as two closing quotes.

## 3. Must fix: wording you have ruled out (from the scan and the agents)

- **"tick":** 17 times in Chapter 3, plus 2.4 and C1. Use "control step". Table 10 says "Late ticks" where the results say "Late control steps".
- **"fold" / "folded":** Introduction, C3, and Table 3.
- **"the catch", "the switch", "attach" as a noun:** 3.4, 3.4.4, 4.2.3 ("The radio switch", "At the switch"), and Appendix A ("Detect the magnet's catch").
- **"miss":** 4.2.3 and Table 24. Use "horizontal error".
- **"triangle" and "leaves" (meaning detaches):** 3.1.1. Also "take-off" (3.1.1, Fig 23) and "loss" (3.4, 4.1.4, 2.2).
- **Figure labels:**
  - Fig 14: "Time after detach";
  - Fig 16: "released", "staged", "folded in", "Carriers" and "t after the release", plus a **"5° limit" line in a result figure**;
  - Fig 22: "Departed";
  - Fig 23: "take-off".
- **"re-attachment" against "reattachment":** both are used (4.1 uses "reattachment"; 4.2, Ch 5 and the abstract use "re-attached"). Pick one.
- **"A closed flight controller such as Betaflight"** (2.5, G4, Fig 5): Betaflight is open-source. Write "an unmodified flight controller running Betaflight".
- **Acknowledgements still read as notes:** "Idea for funnel", "potentially reviewing some of the writing", "Thesis group mainly thomas, ryan, sina".
- **Nomenclature:** still the template's placeholders (your 30 Oct decision). It is needed before hand-in.

## 4. Should fix, by chapter (selected; the full lists are in the raw files)

**Chapters 1–2 (W1):**
- The motivation, gaps and contributions chain holds.
- Quan and Bacheti are each explained two or three times (2.1.1, 2.1.2, 2.2).
- G3 contradicts 2.4: Saccani et al. need no request.
- G2 can be broken by Zeng et al. (rigid-body attitude through a failure, but the agent stays attached). Narrow it.
- "Stationary structure" is wrong for ModQuad, Shankar and LEGION.
- Liu [14] is distributed, not decentralised.
- Acronyms are never expanded: MPC (in the abstract), NMPC, QP, SQP, ESC and ExpressLRS.
- Contributions C2 and C3 are vague ("can handle", "will autonomously adapt").
- The abstract's "a relay flew the ring three times longer than one battery lasts" needs fixing: it is a 600 s budget, not a battery model.
- Literature claims checked against the stored papers hold: Sun et al.'s numbers, and Quan et al.'s DGPS, 4 kg and 13 s.

**Chapter 3 (W2):**
- Equations and constants checked against the code all match:
  - the sign of the integral;
  - eq. (6) and eq. (13);
  - the RTI and its 60 ms budget;
  - the horizons;
  - the 1.6 N and 2.2 N shares;
  - the support margins.
- 3.1.2 uses the thrust map, the measured tension, the disturbance estimate and the velocity estimate before they are explained. That is forward in content, not in references.
- 4.1.2's two studies (the gap sizes, and the baseline with delayed resizing) have no method in Ch 3, and Table 9 omits them.
- Table 10 does not match the metrics the results use: it lacks maximum tilt or tilt rise, second swing, settled tilt, height loss and recovery time, and time in band is never used.
- Undefined before use:
  - LAND (only in Appendix A);
  - the take-back;
  - the position check;
  - "one-step" detachment;
  - the blend time for a new agent.
- Attachment is spread across 3.4.1, 3.4.3, 3.4.4 and Table 7, and the 0.1 N start appears three times.
- The bold **q** is used for three things: payload attitude, cable direction and quadrotor attitude.
- "Three nodes" in 3.1 against eight in Appendix A, and the fleet manager's role differs between them.
- The thrust ladder: only one quadrotor flew 196 g; the other three flew 0 and 98 g.

**Section 4.1 (W3):**
- 4.1.1 "Otherwise the number of quadrotors made little difference" is wrong: the spinning line's error falls from 6.6 to 3.7 mm.
- "Baseline controller" in 4.1.1 means the proposed planner, while 4.1.2's baseline is something else.
- The 10 of 60 landing disarms (every quadrotor at 0.5–0.7 m) have their own safety meaning. The pointer "as in the attachment runs above" leads nowhere.
- "Attached within 0.04 s of reaching it" against the 0.3 s rule. [To check: does "reaching" mean after the hold?]
- 4.1.5:
  - the lag is put on the method, though the port leaves out the paper's damping terms;
  - "hub" against "centre" is never defined once;
  - one paragraph does six jobs;
  - "tilted slightly more" understates 1.5° against 0.73°.
- Relay:
  - 3.1 times longer but only 17 % more laps; the pauses are unexplained, and so is what ended the relay;
  - the text leads with "last lap no worse than the first", while most laps (flown with seven quadrotors) were twice as bad.
- 4.1.4 order: the timing is split around the 8-quadrotor results, real-world solve times sit in the simulation section, and Fig 19(b) is never described.
- Figure 16 sits after the sequence paragraphs, its "earlier run" is unexplained, and its numbers differ from Table 15.

**Section 4.2, Chapter 5 and the appendices (W4):**
- Statements valid only for 4 and 5 Nov will be contradicted by 11 and 12 Nov unless dated:
  - "in six flights";
  - "6 of 15 pickups";
  - "the smaller object's impact left every magnet in place";
  - "never caught".
- "No reviewed work has shown either in the real world" has an unclear antecedent, and detachment on hardware exists in Quan, Zeng and Bacheti.
- 17 mm (4.2.3) against 1.6 cm (Table 24) for the same catch.
- "The simulation results are therefore optimistic" is contradicted for the detachment flown in both (the real ring had a smaller rise and recovered faster).
- The 60° explanation in 4.2.1 explains no difference unless it is tied to Setup 1's 0.55 m cables (0.28 m out, 0.48 m up).
- Appendix C:
  - "let the simulated links go slack" contradicts Table 6 (rigid links);
  - the compliance row is unclear;
  - three times the weight gives no reason for not keeping it.
- Appendix D: the four-quadrotor spin carried an extra 0.1 kg.

## 5. Figures (F1, F2)

**Redraw (data):**
- Fig 18: clipping and the colour conflict with Fig 17.
- Fig 26: numbers and the clipped title.
- Fig 13(b): the figure-eight's aspect.
- Fig 15: the default tab10 palette. It is the only one in the thesis, and in deuteranopia the red new agent looks olive.

**Fixable without data:**
- Fig 6: the dashed region borders run through "Throttle and body rates, 100 Hz over ExpressLRS". The figure shows five purple blocks against "three nodes" in the text, and motion capture feeds the planner only. The caption has "collaborators" without an apostrophe and calls the boxes "purple" when they look blue.
- Fig 9: in greyscale the red quadrotor looks like the others. Add a second cue.
- Fig 12: the red chord disappears in greyscale.
- Fig 11:
  - take-back and LAND appear before they are defined;
  - "from motion capture" is wrong for an announced detachment.
- Fig 19: "32 % of solves over 60 ms" crosses the right spine.
- Fig 23: (b)'s y label is aligned with (a)'s categories, and "1764 s" against 1763 in the table.

**Fig 16 cannot be redrawn** (its recording is gone). A vector edit could:
- remove the "5° limit" line;
- rename the event labels.

Otherwise, drop it.

**Consistency across figures:**
- Red means too many things: the detaching or attaching quadrotor, the baseline, the real world, a large offset, the heading, feedback and the safety controller.
- Panel-label and title styles differ.
- The time-axis wording differs ("Time after …", "t after …", "take-off").
- The names differ: Carriers, Carrying, Q1–Q8 and Quadrotor 1.
- Grey bands mean three different things.
- Fig 22 is 5.9 in tall, against the 4 in guide.

## 6. Dropped or doubtful findings

- W3-5 said "Table 17 has no group rows". It has bold group rows, so this is dropped. Its point about duplicate labels within groups still stands as a "could".
- E1 asked to "re-fly Setup 1 cases on 0.55 m cables": that is new work, so it is left to your decision (D3).
- E1 asked for a confirmation-rule confusion matrix from the logs. It needs lab analysis (a lab request), so it is left to your decision (D2).
- The figure critics asked for longer captions. You chose very short captions, so these are not listed.
- Several agents suggested numbers in prose belong in tables. Your style keeps key numbers in prose, so only conflicts are listed.

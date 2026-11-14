# E1 examiner (12 Nov) - key points as received (full text in the session)
Overall: system work strong, honest reporting; framing over-claims vs data. Bachelor of Engineering on the title page (check!).
1 Resizing value not shown; baseline undefined + misattributed ("baseline use of Sun et al." is our planner without resizing); say what the baseline does with the detached quad's planned tension/ref and which loop moves the load; reframe C1 (adding agents, consistent limits, uneven-layout lean); tone down 0.44 s.
2 Real-world C3 evidence thin (2 catches; 0/11 contacts layout d); no real attachment figure; confirmation rule accuracy (confusion matrix from logs); word C3 as confirmation + hand-over of an attachment flown by the collaborator's controller; add a time series of the real attachment.
3 Sim optimistic + real-vs-sim comparison confounded (0.55 vs 1 m cables, bigger paths, half speed, earlier software, best-of); 8-quad/relay use idealised attachment, not real time: say so in abstract/conclusions.
4 Safety rules inconsistent: drop rule 200 deg (p24) vs 180 deg drops (Table 12, p50); tilt limits 30/45/55/80; "relaxed drop rule of 3.4.2" not defined there; detection margin 0.06 (Table 27) vs 0.15; horizontal disturbance estimate off for 180 deg cases not in method -> one table per experiment of drop rule / tilt limit / margin / estimate setting.
5 Stats: averages of 2 runs without spread while interpreting 0.3 deg / 1 mm differences (CONFLICTS with Wesley's 9 Nov choice of averages: flag only).
6 Quan comparison fairness: port deviations, two variants fail, detachment test in 180 deg layout, "informed 0.35 s" vs lit "not told" (p6), line error metric differs from contour error, heading says Dissipative vs text passivity-based.
7 "Measured tension" is a model residual: name it estimated cable force; no ablation; cap/clip caused sinks; "shorter lag cut the swing by half" has no table; cheap check: estimate vs static tension (share x 1.15) per quad in hover.
8 Uneven-formation lean: 15.6 deg (p47/51) vs "21 to 28 deg" (p54) - reconcile; discuss as design flaw.
9 Scalability timing numbers conflict: 8-quad one-iteration tick 50 ms (p40) vs 66 (Table 18) vs 83.1 (Table 16) vs 261 with sim; 3.5x factor from 3 quads extrapolated; abstract lacks "not real time".
10 Relay: charging wait + stopping criterion not stated; relay ~30 mm contour most laps vs 15-16 fixed, text leads with "no worse last lap than first".
11 Attachment-point study: draw the hardware-relevant conclusion (<=2.5 cm both work); 10 of 60 landing disarms (17%) not in the tables.
12 Reproducibility: missing lambda min/max, f limits, gamma/eta max, uT max, u bar, tau_omega, c_z, rate curve, integrator for F, equal-shares split for N>=4, new-agent blend duration, sim magnet catch + collaborator controller, "warm-starts twice", tau in damping formula, how K measured; Nomenclature "[TODO]".
13 Metrics after the fact: recovery time invented in results; Fig 27(b) pre-event tilt ~5 deg vs 5.3 rise (signal to noise).
14 Numbers disagree: detection 0.44 / 0.25-0.44 (conclusion) vs 0.27-0.43 in results; Table 2 (p6) attachment "1.15 kg, tilt at most 2.5" vs now 0.86 kg, 1.9; Fig 26 vs Table 11 (8.7 vs 8.6, 3.8 vs 3.7); limitation "re-attachment to three never caught" vs layout (e) caught; kT 35.2 at 0.55 kg vs 0.583 kg quads Setups 2/3 (Table 27).
15 Intro one page, no research question/objectives/scope/outline; contributions at end of lit review; C2/C3 informal ("can handle", "will autonomously adapt"); G2 partly self-created.
16 RTA/bumpless framing not backed by data; drop = indoor-only exit.
17 Everything depends on mocap; map each mocap signal to an onboard alternative.
18 Fig 16 from earlier run, "5 deg limit" line; idealised joint attachment.
19 "This work" rows in lit review tables (Tables 2-3) before the methods; point-mass sag vs tilt not comparable.
20 Polish: Acknowledgements notes ("Way better design for funnel", "thomas, ryan, sina"); typos: "modified to handed tethered quads" (p.iii), "a collaborators controller" (p12), "over a ExpressLRS" (p13), "The peaked at about 58" (p34), "then attached onto the ring" (p32), "centralised control is that they" (p29); refs [7]=[22] duplicate, [24][25] no venue/year, [11] spacing; jargon glossary (notice, fold in, take-back, errand, give up, carriers, drop rule; control step/tick/solve); Fig 6 three nodes vs Appendix A eight.
Oral Qs: 8 (baseline, residual tension, confirmation false pos/neg, drop rule exact, sim-real gap dominant factor, port faithfulness, 3.5x at 8 quads, magnet-at-cable-end).
Strengths: honesty, support-margin analysis (best insight), real integration, thrust map quantified, Appendix C, lit review tables, Figs 14/17/20/23.

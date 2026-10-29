# Tejen's follow-up, 29 Oct ~13:10 (pasted in the session by Tejen; after rig-evidence-20261029.zip)

Thanks—I've inspected the metadata, logs and relevant bag windows.
The timing discrepancy is confirmed. All three runs used oct10, with command_execution: first_stage_endpoint, at approximately 30 Hz. My first recommendation is to port the elapsed-interval command correction without changing the oct10 weights or bounds. Simply selecting oct14 changes several things simultaneously. This is a confirmed implementation discrepancy, not proof that it caused all the swing.
The completion-collection fix is already present. Correction to my earlier advice: your supplied patch contains it, and sq2_5 installed all 27 accepted plans. Median completion-to-collection delay was 0.20 ms. Don't port that fix again or remove the version check based on the old starvation report.
The descent restart is genuinely single-tick and position-triggered. The bag reconstruction supports repeated excursions beyond 8 cm. However, its speed condition uses body velocity, not magnet-to-plate velocity. Low body speed therefore doesn't establish a quiet magnet. A short sustained-loss dwell remains worth evaluating, but these interrupted descents cannot prove that a 0.3 s debounce would have completed attachment.
Pickup oscillation is real—not simply a 4 cm constant offset. I reproduced the k1_4 window:
Median magnet–object XY error: 4.07 cm.
Mean offset: approximately (−1.39, +0.31) cm.
Oscillation about that mean: 5.09 cm RMS.
Inside 2.5 cm: 16.8% of samples.
A stiff cable bend is still an unresolved confound. Small mean magnet-to-target error does not rule out a bent-cable bias: body positioning or trim could compensate for it. We need to distinguish target error from the anchor-to-magnet geometry used to calculate swing. Don't simply subtract a mean from collision geometry—the displaced magnet still occupies that position.
Also, the MPC CSVs already contain phi, theta and their rates, although the separate raw swing topic was unavailable. Those channels can help inspect what the controller actually believed.
Suggested order: isolate the timing correction first; then reassess swing and the narrow descent/capture changes. No wholesale controller merge, broad threshold relaxation or 1° collision-swing assumption.

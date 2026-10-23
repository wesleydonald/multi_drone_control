# F10b_crossover (F10 panel b: where the survivors learn of the change, unannounced half)

File: `crossover.pdf` (full width, 6.27 x 2.45 in; `\includegraphics[width=\textwidth]{...}`).
Script: `python3 tools/thesis_lever_figures.py crossover` (prints every number below per run).
Data: Tier A T3 on the detach freeze tree (card `docs/experiments/2026-10-12_tier_a_crossover.md`, rev 3 result and
reviewer). The announced ramped pair (T2) has not flown, so this is the unannounced half of the crossover only; no rig
flight of this cell exists yet (W4 has not flown on the rig; add it to `RIG_F10B` when it does).

## Caption

\caption[Where the survivors learn of an unannounced release]{An unannounced release in the twin, one cell: four UAVs
hover the 0.86\,kg ring at 1.0\,m from plates 3, 7, 9 and 11, and the magnet on plate 9, the smallest share, opens
without telling the planner, leaving a 120$^\circ$ gap; the model integral; every link rigid;
three runs per pull source, the release instant moved by 0.5\,s between runs. (a) The ring's tilt departure (its tilt
vector's distance from its mean over the 5\,s before the release) against time, black with the survivors flying the
measured rod pull, red with them flying the planner's pull; marker: each run's tilt rise $\Delta\theta$, the largest
departure up to 15\,s after the release; grey: the planner finding the loss and resizing to three UAVs, 0.28--0.37\,s
after the release over the six runs. (b) $\Delta\theta$ per run (filled: measured pull; hollow: the planner's pull)
and the arm means: 5.5--7.7$^\circ$ against 16.6--17.1$^\circ$. The means differ by 10.4$^\circ$ against the
4.4$^\circ$ the resolution rule frozen before the runs requires, so the pair resolves with the predicted sign. The
two arms do not start from the same state: on the planner's pull the ring hung 2.0$^\circ$ tilted before the release
and again after it, on the measured pull 0.4--0.8$^\circ$ before and level after.}
\label{fig:F10b}

## Where it goes

Section 4.2 (RQ2), "Where the survivors learn of the change" (`skel_leave.tex`, the paragraph after fig:F10), as F10's
panel (b) or a figure of its own.

## Sentence that cites it

"In the twin cell the frozen prediction names (Figure~\ref{fig:F10b}), the measured rod pull kept the ring's tilt rise
to 5.5--7.7$^\circ$ where the planner's pull let it reach 16.6--17.1$^\circ$, a difference the frozen rule resolves;
the announced half of the crossover has not been flown."

## Must not say (the cards' lists, 12 and 16 Oct)

A general "the measured pull cuts the tilt to a third" (one twin cell); any rig statement or rig-sized number; the
late detection as the cause (the formations differ: the planned-pull ring hovers 2$^\circ$ tilted at a steeper rod
elevation); "even with a correct model the planned pull loses"; "the twin confirms the crossover" (T2 and T4 not
flown); "the measured pull detects the loss sooner" (it is the other way round: 0.28--0.37 s measured, 0.30--0.32 s
planned).

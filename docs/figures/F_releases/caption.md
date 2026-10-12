# F_releases (F10, panel (a): the hardware record)

File: `releases.pdf` (full width, 6.27 x 3.3 in). Script: `python3 tools/thesis_leave_figures.py releases` (prints
each row's event, window and tilt rise). Rows and their twin pairs in `configs/thesis_figures.yaml` (`F_releases`).
The 14 Oct releases are added as rows there; the twin crossover panel (b) comes with Tier A.

## Caption

\caption[Every release on the rig]{Every release on the rig, one row per event, grouped by the day and the rod
plan: the ring's tilt rise $\Delta\theta$ (the departure of its tilt vector from its mean over the 5\,s before the
event, over the event to LAND or to the fleet's latch) as a marker, filled for the measured rod pull and hollow for
the planner's pull, shaped by how the release came (announced, unannounced, or a magnet letting go with no command);
the grey bar with end ticks is the range of the twin pair flown on the same configuration, cut at the rig flight's
hold, and rows without one say so. The two capsizes and the one recovery sit on the broken axis against the
30$^\circ$ drop rule, which these flights predate, and the trackers' 60$^\circ$ latch. The 7 October circle flew the
even ring of four, whose three survivors leave a 180$^\circ$ gap; it lost two magnets 2.8\,s apart, recovered from the
first, and capsized on the second during the landing, so that row's pre-window holds the first recovery.}
\label{fig:F10}

## Where it goes

Section 4.2 (RQ2, the leave; `sim_detach.tex`), "The full hardware record", beside table T8.

## Sentence that cites it

"Of the nine release events on the rig (Figure~\ref{fig:F10}), the three remaining UAVs carried the ring on through
every commanded release on the measured pull, with tilt rises of 5--17$^\circ$ that the twin pairs bracket to within
a factor of 1.7; the ring was lost three times, each time after a magnet let go with no command, and every one of
those came with the rods planned at 45$^\circ$."

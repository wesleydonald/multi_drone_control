# F_rig_circle_detach

File: `rig_circle_detach.pdf` (full width, 6.27 x 3.75 in; `\includegraphics[width=\textwidth]{...}`).
Script: `python3 tools/thesis_rig_twin_figures.py circle` (prints every number below).

11 Oct: panel (c) is on the signed T6 release metric (docs/rig/freeze_2026-10-14.md 1.1): the tilt vector's departure from
its mean over the 5 s before the release; the labelled rise is the frozen tilt rise (the twin cut at the rig's hold).

## Caption

\caption[Unannounced detach during the circle on the rig]{An unannounced detach during the circle on the rig: four
UAVs carry the 0.86\,kg ring at 1.0\,m round a circle of radius 0.5\,m at 0.125\,m/s, on the plates at 30, 90, 150
and 270$^\circ$, and the magnet of the UAV at 90$^\circ$ is switched off without telling the planner. (a) Plan view:
the circle (dashed; +, its centre; arrow, the direction of travel), the ring's centre from 2.5\,s before the release
(open circle) to LAND (gold), and the fleet at three instants, each numbered at the ring's centre and at the freed
UAV: the ring (thin gold circle), the carriers with their rods to their plates (grey, darker for later instants) and
the freed UAV (red, with its path to LAND). 1: the release; 2: the freed UAV starts its own landing, 6.1\,s later;
3: the fleet's LAND, 14.0\,s after the release, with the freed UAV 1.5\,m from the circle's centre (dotted arc).
(b)--(d) The 2.5\,s before the release to 8\,s after it. (b) The planner's tension per UAV, labelled by plate: the
freed UAV's (dashed) stays in the plan until the planner finds the detach from its stretched rod 0.34\,s after the
release (dotted line) and drops it at its next logged tick (cross); the three survivors then share the ring at
3.2--3.3\,N each, from 2.1--3.9\,N before. Grey band: the path held for 1.5\,s from the detection. (c) The ring's
tilt departure, the length of its tilt vector's difference from that vector's mean over the 5\,s before the
release, and (d) ring height error (measured minus planned): the rig (gold, one run) against the twin flying the
same configuration (grey band and line: two runs, nearly identical). The twin's departure peaks once, at
4.0--5.4$^\circ$ 0.4\,s after the release (cut at the rig's hold); the rig's spikes to 5.7$^\circ$ at the same time
and peaks at 8.0$^\circ$, its tilt rise $\Delta\theta$, 1.1\,s after it, against up to 5.9$^\circ$ away from the
release (dashed). The ring dips 3.9\,cm against the twin's 2.1--2.5\,cm.}
\label{fig:rig_circle_detach}

## Where it goes

Section `sec:res_detach_rig` (`thesis_v2/Source/src/sim_detach.tex`, paragraph "The circle detach."), next to
Figure `fig:rig_detach` (the hover detaches). The freed UAV's own landing is `fig:departed_landing`.

## Sentence that cites it

"During the circle (Figure~\ref{fig:rig_circle_detach}) the planner found the unannounced detach 0.34\,s after the
magnet was switched off, held the path for 1.5\,s while the three survivors took up the freed UAV's share, and then
carried the ring on round the circle; the ring's tilt rose 8.0$^\circ$ and it dipped 3.9\,cm, against
4.0--5.4$^\circ$ and 2.1--2.5\,cm in the twin, while the freed UAV cleared the circle and began its own landing
6.1\,s after the release."

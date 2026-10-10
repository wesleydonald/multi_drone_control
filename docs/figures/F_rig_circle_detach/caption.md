# F_rig_circle_detach

File: `rig_circle_detach.pdf` (full width, 6.27 x 3.75 in; `\includegraphics[width=\textwidth]{...}`).
Script: `python3 tools/thesis_rig_twin_figures.py circle` (prints every number below).

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
3.2--3.3\,N each, from 2.1--3.9\,N before. Grey band: the path held for 1.5\,s from the detection. (c) Ring tilt and
(d) ring height error (measured minus planned): the rig (gold, one run) against the twin flying the same
configuration (grey band and line: two runs, nearly identical). The twin's tilt peaks once, at 6.0--7.3$^\circ$
0.4\,s after the release; the rig's spikes to 6.4$^\circ$ at the same time and peaks at 8.1$^\circ$ 1.0\,s after
it, against up to 6.0$^\circ$ away from the release (dashed). The ring dips 3.9\,cm against the twin's
2.1--2.5\,cm.}
\label{fig:rig_circle_detach}

## Where it goes

Section `sec:res_detach_rig` (`thesis_v2/Source/src/sim_detach.tex`, paragraph "On the rig."), after
Table `tab:rig_oct8` and next to Figure `fig:rig_detach` (the hover detaches). The freed UAV's own landing is
`fig:departed_landing` (it stalled above its rest on 8 October; not drawn here).

## Sentence that cites it

"During the circle (Figure~\ref{fig:rig_circle_detach}) the planner found the unannounced detach 0.34\,s after the
magnet was switched off, held the path for 1.5\,s while the three survivors took up the freed UAV's share, and then
carried the ring on round the circle; the ring tilted 8.1$^\circ$ and dipped 3.9\,cm, against 6.0--7.3$^\circ$ and
2.1--2.5\,cm in the twin, while the freed UAV cleared the circle and began its own landing 6.1\,s after the
release."

Note for the text: the current paragraph says the survivors "carried on along the path without a pause"; the rig
reference (and the planner's log, "trajectory held 1.5 s") shows a 1.5 s hold from the detection. The sentence
above replaces that claim.

## Evidence map (caption clause -> mark)

- 0.86 kg, 1.0 m, 0.5 m circle at 0.125 m/s, UAV at 90 deg -> planner params.json (printed by the script);
  (a) the dashed circle and the four plates at instant 1.
- found 0.34 s after the release -> (b) dotted line labelled "found +0.34 s"; the freed UAV's dashed plan ends at
  the cross one planner tick later (on the rig the logged tension row follows the detection log line by 0.14 s).
- plates 30/90/150/270 -> (b) labels "270", "30, 150", "90, freed"; (a) the rods at instant 1.
- instants 2 (+6.1 s) and 3 (+14.0 s), 1.5 m -> (a) badges on the ring and the freed UAV; (b) badges 1 and 2 over
  the strips; the dotted arc "1.5 m from the centre" (at instant 2 it is 1.4 m out, printed by the script).
- survivors 3.2-3.3 N from 2.1-3.9 N -> (b) grey steps and the "three survivors" label.
- path held 1.5 s -> grey band from the detection to the reference's first move (b-d), labelled in (d).
- peak 8.1 deg vs 6.0-7.3, the twin at 0.4 s, the rig's 6.4 deg spike at 0.37 s and peak at 1.0 s -> (c) gold dot
  and label; the twin's single grey peak; the gold trace.
- dip 3.9 cm vs 2.1-2.5 -> (d) gold dot and label; twin band.
- up to 6.0 deg away from the release -> (c) dashed gold line, labelled.

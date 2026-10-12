# F_magnet_anatomy (F11)

File: `magnet_anatomy.pdf` (full width, 6.27 x 3.9 in). Script: `python3 tools/thesis_leave_figures.py magnet`
(prints the release instants, the latch and the planned tension step per rod). Runs in `configs/thesis_figures.yaml`
(`F_magnet_anatomy`): the two 7 Oct flights, converted with `tools/rig_to_run.py`; the planned tensions from each
flight's planner log; the release instants from the rods' measured length over their own (the replay's rule:
3\,mm over the baseline, sustained).

## Caption

\caption[How the ring was lost]{How the ring was lost on the rig, twice, both with the rods planned at 45$^\circ$.
Left, an announced release from the plate at 90$^\circ$ of a hover on the plates at 30, 90, 180 and 270$^\circ$ (a
150$^\circ$ gap left), on the planner's pull: (a) the planned tension per rod, the leaver's dashed, with the
survivors' step at the resize (2.8 to 4.6\,N and 2.3 to 3.8\,N on the two rods beside the gap); (b) the ring's tilt;
(c) each rod's measured length over its own, so that a rod leaving its plate shows as a rise: 0.17\,s after the
release the 30$^\circ$ rod let go with no command, at 2.6$^\circ$ of tilt, and the 180$^\circ$ rod followed at
0.39\,s; the trackers latched the fleet at 60$^\circ$ of tilt 0.47\,s after the release. Right, the circle at
0.125\,m/s on the measured pull with four rods on the even ring: (d) the planned tensions, 3.0\,N each; (e) the tilt;
(f) the 270$^\circ$ rod let go with no command at 0.9$^\circ$ of tilt and the ring recovered from 31$^\circ$; the
operator's LAND came 0.6\,s after it, and during the descent the 180$^\circ$ rod let go, at 7.7$^\circ$, and the ring
capsized. Dotted: the 30$^\circ$ drop rule and the trackers' 60$^\circ$ latch; the freed rods leave the panel.}
\label{fig:F11}

## Where it goes

Section 4.2 (RQ2), "How the ring was lost", the magnet paragraph (`sim_detach.tex`).

## Sentence that cites it

"Every ring lost on the rig began with a magnet letting go (Figure~\ref{fig:F11}): once 0.17\,s after a commanded
release whose planned step raised a survivor's share by two thirds, and twice with no command on the circle, all with
the rods planned at 45$^\circ$, where each pull meets its plate 15$^\circ$ further from the axis than at 60$^\circ$."

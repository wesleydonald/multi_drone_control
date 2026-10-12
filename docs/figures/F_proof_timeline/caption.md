# F_proof_timeline (F14)

File: `proof_timeline.pdf` (full width, 6.27 x 4.0 in). Script: `python3 tools/thesis_join_figures.py proof` (prints
the contact-to-proof time, the lift, the plate rise, the drift at the proof, the throttle step and the ring tilt per
run, and the thresholds the manager logged). Runs in `configs/thesis_figures.yaml` (`F_proof_timeline`): the two
hover attaches of the claim batch on Wednesday's install. Pending: the rig's plate-noise band from the proof replay.

## Caption

\caption[The contact-and-lift proof]{The contact-and-lift proof in the twin: a newcomer flown by the collaborator's
controller comes down onto the plate at 90$^\circ$ of a hovering ring and the catch is proven from motion capture
alone. Two runs of the same arm, aligned on the contact: the darker run first, the lighter its repeat, which arrived
more slowly. (a) Heights above their value at the contact: the newcomer's body, its magnet tip (from the tip's own
marker, unfiltered) and the plate point (from the ring's pose). The collaborator's lift aims 10\,mm above the contact;
the catch is proven once the plate has risen 2.5\,mm with the tip and held it for 0.3\,s, which came 1.08 and
0.72\,s after the contact. (b) The tip's drift from the tip-to-plate vector it touched with, in the ring frame: a
proof needs it within 3\,mm and a contact is lost at 8\,mm. (c) The newcomer's throttle as a percentage of its hover
throttle: its radio switches to this work's tracker on the proof tick; the 0.3\,s mean after the switch differs from
the 0.1\,s before it by $-0.2$ and $-0.5$\,\%, and the rise that follows is the fold-in taking up its share of the
ring. (d) The ring's tilt departure from its mean over the 5\,s before the contact, against the 5$^\circ$ bar.}
\label{fig:F14}

## Where it goes

Section 4.3 (RQ3, the join; `sim_attach.tex`), "The admission contract and the proof timeline".

## Sentence that cites it

"The proof needs no sensor in the magnet: the plate rises with the tip while the tip keeps the vector it touched with
(Figure~\ref{fig:F14}), and the radio switches on the same tick with the throttle continuous to within a percent."

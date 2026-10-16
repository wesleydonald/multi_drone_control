# F12_levers (the other levers: one-variable pairs, and the support margin)

File: `levers.pdf` (full width, 6.27 x 3.0 in; `\includegraphics[width=\textwidth]{...}`).
Script: `python3 tools/thesis_lever_figures.py levers` (prints every value, every cut instant and every pair's
resolution). Data: the detach freeze batch (Tier A rev 3 and Tier B cards, both reviewed 16 Oct), plus the rig points
the Tier B card names. Not drawn: the ramp, integral-mode and model-error pairs (not flown on this tree), the 60 deg
circle pair T15 (a path error, not Delta-theta), the landing lean (K1 not credited).

## Caption

\caption[The other levers]{The other levers in the twin: unannounced hover releases, the model integral, every link
rigid; one marker per run; arrow or triangle: a lower bound, the run cut where the ring was dropped or the fleet
latched. (a) One-variable pairs, each headed by its variable: the pull source with a 120$^\circ$ and a 150$^\circ$ gap
left, and detecting the release on and off (off: the planner never resizes) on each pull source; filled: the measured
rod pull; hollow: the planner's pull; right: the pair under the resolution rule frozen before the runs. With detection
off on the planner's pull the largest departure came 9.5--10.7\,s after the release, the tilt creeping to within
1.5$^\circ$ of the drop rule. (b)
$\Delta\theta$ against the support margin $m = r\cos(g/2)$, $r = 0.225$\,m, that the largest gap $g$ left between the
three remaining UAVs gives (top axis), on the measured pull, the line through the means: 5.5--7.7$^\circ$ at 112\,mm,
7.8--9.1$^\circ$ at 86\,mm, 16.7--17.0$^\circ$ at 58\,mm and 27.9--28.1$^\circ$ at 29\,mm. At zero margin, with the
drop rule off, the trackers' 60$^\circ$ latch disarmed the fleet in the air 0.6\,s after the release (arrows); on the
default rules the planner's gap rule dropped the ring when it found the loss (filled triangle). Hollow triangles: the
planner's pull at 58\,mm, cut by the trackers' 30$^\circ$ drop rule in both runs. Diamonds: the rig's two unannounced
hover releases of 8 October (the reference integral, the freed UAV landing with the fleet, $\Delta\theta$ up to their
LAND 7.6--7.7\,s after the release); red arrow: the rig's announced release on the planner's pull with the rods
planned at 45$^\circ$, in which two magnets let go (Figure~\ref{fig:F11}). The twin has no magnet holding limit: the
curve speaks for the planner and the statics, not for a margin of safety on the rig.}
\label{fig:F12}

## Where it goes

Section 4.2 (RQ2), "The other levers" (`skel_leave.tex`, replacing `\pendingfig{fig:F12}`).

## Sentence that cites it

"With the measured pull the tilt rise grows as the support margin shrinks, from 5.5--7.7$^\circ$ at 112\,mm to
27.9--28.1$^\circ$ at 29\,mm, and at zero margin three UAVs cannot hold the ring (Figure~\ref{fig:F12}b); of the
one-variable levers, only detection on the measured pull made no difference the rule could resolve at three runs
(Figure~\ref{fig:F12}a)."

## Must not say (Tier A rev 3 and Tier B reviewers, 16 Oct)

"No run at gaps up to 150 deg reached the drop line" without "with the measured pull" (the planner's pull at 150 deg
dropped the ring in 2 of 2); G180's ~61 deg or G150p's ~29.5 deg as the ring's tilt (lower bounds at the latch and the
drop); "the gap rule drops the ring before it passes 30 deg"; B4 as confirmed (falsified as written); "detection lowers
the survivors' landing lean" or "K1 confirmed"; "the planned pull holds the ring after an undetected loss"; T9 as
"detection makes no difference on the measured pull" (no resolvable difference at n = 3, A1's range 2.2 deg); any rig
reading of A6's landing tip-over and fleet abort.

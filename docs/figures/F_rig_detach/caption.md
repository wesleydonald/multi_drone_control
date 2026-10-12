# F_rig_detach

File: `rig_detach.pdf` (full width, 6.27 x 3.0 in; `\includegraphics[width=\textwidth]{...}`).
Script: `python3 -c "import tools.thesis_rig_figures as f; f.fig_rig_detach()"` with `MDC_RESULTS_ROOT` set to the
results root (prints every number below in its stats).

11 Oct: the tilt panels are on the signed T6 release metric (docs/freeze_2026-10-14.md 1.1): the tilt vector's departure
from its mean over the 5 s before the release, and the labelled rise is the frozen tilt rise. The dips keep the 3 s window.

## Caption

\caption[Detach on the rig against the twin]{Detaching one UAV of four on the rig (gold, one run each) against
the rig twin (grey band and line: two runs, nearly identical), aligned on the release (dotted). Announced: the
planner releases the magnet and drops the UAV from its optimisation at once. Unannounced: the magnet is switched
off without telling the planner, which finds the detach from the stretched rod (second dotted line, 0.33--0.42\,s
after the release on the rig). (a--c) The ring's tilt departure: the length of its tilt vector's difference from
that vector's mean over the 5\,s before the release, so that the rig ring's own rocking in hover (1--1.4$^\circ$
about its mean) is measured about the mean rather than added to the rise; the marker and label give the tilt rise
$\Delta\theta$, the departure's peak from the release to LAND, with the twin cut at the rig flight's hold.
(d--f) Ring height error, measured minus planned (negative: below). Announced: the rig rose 11$^\circ$ against the
twin's 6--7$^\circ$ and dipped 6.9\,cm against 2.6--3.0\,cm. Unannounced: the rig rose 5$^\circ$ against 6$^\circ$
and 17$^\circ$ against 15--17$^\circ$, and dipped about 1\,cm more than the twin (3.7 against 2.5--2.7\,cm; 5.8
against 4.5--4.8\,cm). Rig traces end at the fleet's LAND.}
\label{fig:rig_detach}

## Where it goes

Section `sec:res_detach_rig` (`thesis_v2/Source/src/sim_detach.tex`, paragraph "On the rig."), after Table `tab:rig_oct8`.

## Sentence that cites it

"Against the twin (Figure~\ref{fig:rig_detach}) the unannounced detaches rose 0.85 and 1.05 times the twin's tilt
rise and dipped about 1\,cm more; the single announced detach rose 1.7 times the twin and dipped 2.5 times as far, so
whether the rig's announced release is systematically harsher is open."

# F_twin_rig_parity

File: `twin_rig_parity.pdf` (full width, 6.27 x 1.95 in; `\includegraphics[width=\textwidth]{...}`).
Script: `python3 tools/thesis_rig_twin_figures.py parity` (prints every number below, with the ratios).

11 Oct: panel (a) is the signed T6 tilt rise (docs/rig/freeze_2026-10-14.md 1.1; the twin cut at the rig flight's hold,
Q1), so its ratios are the ledger's class factors (0.85, 1.05, 1.70, 1.71). The attach placeholder is gone (thesis
plan: Fig 13 -> F16, placeholder point removed); the rig attach goes into the ledger figure when it flies.

## Caption

\caption[The twin against the rig]{The rig against its twin for the four detach flights of 8 October (four UAVs,
0.86\,kg ring at 1.0\,m; one marker per flight; 150$^\circ$ gap: the three left on the ring leave a 150$^\circ$ gap
between two of them), each metric computed the same way from both, on logarithmic axes: $x$, the twin flying the same
configuration (mean of two runs; bar: their range, hidden where it is shorter than the marker); $y$, the rig (one run).
Dashed: $y = x$; grey band: within 25\,\% of the twin; dotted: the rig twice the twin. (a) Tilt rise $\Delta\theta$:
the peak of the ring's tilt-vector departure from its mean over the 5\,s before the release, from the release to the
rig flight's LAND, the twin cut at the same hold. (b) Ring dip: its lowest height error in the 3\,s after the release.
(c) Detection delay: from the release to the planner finding an unannounced detach (an announced detach needs none).
(d) Horizontal error: the ring's mean distance from its reference over the 10\,s before the release. Within 25\,\%
of the twin: the tilt rise of the unannounced hover and 150$^\circ$-gap detaches (0.85 and 1.05 times the twin), the
dip of the 150$^\circ$-gap detach, and two of the three detection delays (the third is 1.26 times the twin; all three
within 0.1\,s). The announced hover detach and the circle detach rose 1.7 times the twin; the other three dips were
1.4--2.5 times the twin's, and the horizontal error was 1.9--3.1 times (5.3--6.8\,cm against 1.7--3.3\,cm).}
\label{fig:twin_rig_parity}

## Where it goes

Section `sec:sim_to_real` (`thesis_v2/Source/src/res_carry.tex`, "The twin against the rig"), after Table
`tab:sim_vs_rig`.

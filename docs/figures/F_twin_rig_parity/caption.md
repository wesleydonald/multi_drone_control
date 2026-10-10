# F_twin_rig_parity

File: `twin_rig_parity.pdf` (full width, 6.27 x 2.1 in; `\includegraphics[width=\textwidth]{...}`).
Script: `python3 tools/thesis_rig_twin_figures.py parity` (prints every number below, with the ratios).

PENDING: the attach (R8b weld). Add the Wednesday 14 Oct r8b run directories to `PARITY_ATTACH['rig']` at the top of
the script (e.g. `'results/rig/2026-10-14/r8b'`), run `python3 tools/rig_summary.py <path>` once per run if
`<path>_run` does not exist, and re-run the script: the hollow pentagons (now on y = x at the twin's value) become filled
points at (twin, rig mean; bar: rig range), the legend entry becomes "attach" and the "rig pending" label goes. Then
replace the last sentence of the caption with the attach numbers the script prints.

## Caption

\caption[The twin against the rig]{The rig against its twin for the four detach flights of 8 October (four UAVs,
0.86\,kg ring at 1.0\,m; one marker per flight; 150$^\circ$ gap: the three left on the ring leave a 150$^\circ$ gap
between two of them), each metric computed the same way from both, on logarithmic axes: $x$, the twin flying the same
configuration (mean of two runs; bar: their range, hidden where it is shorter than the marker); $y$, the rig (one run).
Dashed: $y = x$; grey band: within 25\,\% of the twin; dotted: the rig twice the twin. (a) Tilt rise: the ring's peak
tilt in the 3\,s after the release above its mean over the 3\,s before. (b) Ring dip: its lowest height error in the
3\,s after the release. (c) Detection delay: from the release to the planner finding an unannounced detach (an
announced detach needs none). (d) Horizontal error: the ring's mean distance from its reference over the 10\,s before
the release. Within 25\,\% of the twin: the tilt rise of the 150$^\circ$-gap and circle detaches, the dip of the
150$^\circ$-gap detach, and two of the three detection delays (the third is 1.26 times the twin; all three within
0.1\,s). The unannounced hover detach rose 0.65 times the twin and the announced one 1.6 times; the other three dips
were 1.4--2.5 times the twin's, and the horizontal error was 1.9--3.1 times (5.3--6.8\,cm against 1.7--3.3\,cm).
Hollow pentagon: the attach (the newcomer's weld), drawn on $y = x$ at the value of the two twin runs (a 1.5$^\circ$
rise, a 0.2\,cm dip) until the rig flies it.}
\label{fig:twin_rig_parity}

## Where it goes

Section `sec:sim_to_real` ("The twin against the rig", `thesis_v2/Source/src/res_carry.tex`), after
Table `tab:sim_vs_rig`. It can also be cited from `sec:res_detach_rig`.

## Sentence that cites it

"Figure~\ref{fig:twin_rig_parity} sets each detach flight of 8 October against the twin on the same configuration:
the twin predicts how far the ring tilts after an unannounced detach and how soon the planner finds it to within
about a third, but on the rig the ring dips up to 2.5 times as far and sits 2--3 times as far from its reference before the
release."

## Evidence map (caption clause -> mark)

- one run vs mean of two with range -> filled markers (white edge so overlaps stay visible), horizontal bars where
  the two twin runs differ by more than the marker.
- y = x, 25 % band, 2x line -> dashed, grey band, dotted; named once in (c): "1x", "2x", "+-25 %".
- 150-gap and circle rise within 25 %, hover unannounced 0.65, announced 1.6 -> (a) triangle and diamond in the band,
  square below it, circle between the band and the 2x line.
- gap dip within 25 %, the others 1.4-2.5x -> (b) triangle at the band's upper edge, square/diamond above it, circle
  on the 2x line.
- detection: two within 25 %, the square at 1.26x, all within 0.1 s -> (c); "announced: none".
- horizontal error 1.9-3.1x -> (d) all four on or above the dotted 2x line.
- attach pending -> hollow pentagons on y = x at the twin's values with their range bars, "rig pending" in (a),
  legend "attach, twin only (rig pending)".

Metric definitions are in the script's `transient()` and `detach_metrics()`; the detection time is the planner's
"UNANNOUNCED DETACH" log line (rig: wall clock; twin: its launch.log stamps put on the run clock through the
runner's fleet commands), the same source as the rig's DETECTED event in Table `tab:rig_oct8`.

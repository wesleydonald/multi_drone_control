# F_disarm_lanes

File: `disarm_lanes.pdf` (full width, 6.27 x 4.0 in; `\includegraphics[width=\textwidth]{...}`).
Script: `python3 tools/thesis_rig_twin_figures.py lanes` (prints every time below, per run, with the armed-flag
cross-check).

PENDING: the rig rows of (b). Put the Wednesday 14 Oct run directories into `LANES_FREED` at the top of the script,
`('hover, rig', ['results/rig/2026-10-14/w4'])` and `('circle, rig', ['results/rig/2026-10-14/w5'])` (a second run of either adds
the bracketed repeat times), run `python3 tools/rig_summary.py <path>` once per run if `<path>_run` does not
exist, and re-run the script; the hatched blocks become lanes. Then replace the last sentence of the caption with the
rig's numbers (the script prints them per run). See the note on rig_to_run below before trusting a rig run.csv for these flights.

## Caption

\caption[Who is disarmed when]{Which UAV is disarmed when, one lane per UAV: dark, carrying the ring; mid (or the
UAV's own colour), flying off it; light and thin, landing; cross, disarmed in the air; bar, disarmed at rest. The
UAV off the ring is drawn in its own colour once it is off, the carriers in grey; times in brackets are the repeat
run's. (a) The newcomer loses its motion capture 2\,s into its transit to the ring, in the twin. Old rule (one run):
its pose timeout disarms it and the fleet-wide stop disarms all four in the air, 0.25\,s after the loss. New rule
(a fault on the newcomer lands the fleet; two runs): it alone is disarmed, 0.24\,s after the loss and 1.8\,m above
its rest, and the carriers land the ring at once and are disarmed at rest 9.7\,s after the loss. (b) A carrier
released without warning from the 90$^\circ$ plate, in hover and in the circle (twin, two runs each): it steps out,
lands by itself and is disarmed alone at rest 7.9\,s (hover) and 8.5\,s (circle) after the release, while the three
others carry the ring on until the LAND 20\,s after the release (dashed) and are disarmed together 29.5--29.6\,s
after the release. The repeats' disarms are within 0.2\,s. Hatched: the same two flights on the rig, to be flown on
14 October.}
\label{fig:disarm_lanes}

## Where it goes

Section `sec:res_detach_rig` (`sim_detach.tex`), after the paragraph "The freed UAV's own landing" and
`fig:departed_landing`; panel (b)'s rig rows are the "disarmed alone" half of the pending `fig:rig_departed`. Panel
(a) is also the evidence for the newcomer exception to the fleet-wide disarm, so the Safety paragraph of
`sec:overview` (`system_overview.tex`) should cite it.

## Sentence that cites it

"Only a UAV off the ring is ever disarmed alone (Figure~\ref{fig:disarm_lanes}): a newcomer that fails in its transit
is disarmed at once while the carriers land the ring, where the old rule disarmed all four in the air, and a freed
carrier is disarmed once it is down at its rest, about 8\,s after the release, while the others fly on until LAND."

Note for the text: the Safety paragraph of `system_overview.tex` still says "Any violation disarms the whole fleet,
not the one UAV"; since 9 Oct (`newcomer_fault_land` on by default) a fault on an unwelded newcomer lands the fleet
instead (the locked decision in the working rules). The sentence there needs the exception.

## Evidence map (caption clause -> mark)

- states and markers -> legend row (state by shade and bar height, so it holds in greyscale); times from the console
  log lines (the trackers' "Disarm triggered" / "Disarmed via service", the fleet manager's LAND, the planner's
  "departed drone ... lands now"), cross-checked against each UAV's armed flag in run.csv (within 0.06 s; printed by
  the script). In the air = more than 0.10 m above its height before take-off at the disarm.
- old rule, all four in the air at +0.25 s -> (a) four crosses, "all four disarmed in the air, +0.25 s".
- new rule, newcomer alone at +0.24 s, 1.8 m up -> (a) cross, "+0.24 s, alone, 1.8 m up (+0.26 s)".
- carriers land at once, disarmed at rest at +9.7 s -> (a) thin light lanes from +0.24 s, bars, "+9.7 s (+9.6 s)".
- freed carrier off the ring, landing, alone at rest at +7.9 / +8.5 s -> (b) orange lanes from the release, thin
  light orange landing, bars, "+7.9 s, alone (+8.1 s)", "+8.5 s, alone (+8.7 s)".
- carriers on until LAND at +20 s, disarmed at +29.5-29.6 s -> (b) dark lanes to the dashed LAND line, bars,
  "+29.6 s (+29.5 s)", "+29.5 s (+29.5 s)".
- repeats within 0.2 s -> the bracketed times.
- rig pending -> four outlined, lightly hatched lanes per block, "rig, 14 October: pending".

## Note: rig_to_run on the 14 Oct w4/w5 flights

`tools/rig_to_run.py` builds the rig run.csv on the time span every tracker log covers
(`t1 = min(last tracker sample)`), and it keeps only events up to `t1 + 5 s`. With the alone-disarm, the freed UAV's
tracker shuts down at its disarm ("Disarmed via service -- initiating shutdown"), so w4/w5's run.csv and events.csv
will end about 8 s after the release, before LAND. This script does not depend on it (it reads the T2 log and the
trackers' own logs past the end of run.csv), but `rig_summary.py` and anything else reading those run.csv files will
show a flight that stops at the freed UAV's disarm.

## Note: the rig carriers on 7-8 October were not at rest when the fleet disarmed

Run through this script's test of the rig path (`lanes('results/rig/2026-10-08/r305', 'freed')`), every 7-8 Oct rig
run that reached LANDED ends with the carriers 0.08-0.17 m above their heights before take-off and not moving
(vz about 0): r0012, r206e60, r302_3, r303_4, r304_3, r305 (run.csv, last 0.1 s, which ends at the trackers' disarm;
the freed UAV of the detach flights 0.28-0.38 m, the known stall). Not checked further: whether they were resting on
their rods or hovering. With the 0.10 m threshold the rig rows will mark such carriers "disarmed in the air". If the rig
carriers on 14 Oct are again 0.1 m up at the fleet disarm, the figure will show it as it is; the threshold is
`IN_AIR_M` at the top of the lanes section.

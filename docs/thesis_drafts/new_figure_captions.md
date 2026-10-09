# Captions for the new figures (9 Oct 2026), numbers checked against the data

## fig:rig_detach (Images/rig_detach.pdf, full width)
Detaching one UAV of four on the rig (gold, one run each) against the rig twin (grey band and line: two runs,
nearly identical), aligned on the release (dotted). Announced: the planner releases the magnet and drops the UAV
from its optimisation at once. Unannounced: the magnet is switched off without telling the planner, which finds
the detach from the stretched rod (second dotted line, 0.33--0.42\,s after the release on the rig). (a--c) Ring
tilt, the angle of the ring's normal from vertical; the labels give the rise of the peak above each trace's own
mean over the 3\,s before the release, because the rig ring rocks 2--5$^\circ$ in hover (ring swing and
motion-capture noise the twin does not model) where the twin sits at 1.5$^\circ$. (d--f) Ring height error,
measured minus planned (negative: below). Announced: the rig rose 11$^\circ$ against the twin's 6--7$^\circ$ and
dipped 6.9\,cm against 2.6--3.0\,cm. Unannounced: the rig rose 4$^\circ$ against 6$^\circ$ and 17$^\circ$ against
15--17$^\circ$, and dipped about 1\,cm more than the twin (3.7 against 2.5--2.7\,cm; 5.8 against 4.5--4.8\,cm).
Rig traces end at the fleet's LAND.

## fig:solve_time (Images/solve_time.pdf, half width)
The planner's optimisation time in flight on the rig, 7--8 October: three UAVs (after a detach, 15\,883 ticks),
four UAVs (dissipative mode, 4\,762 ticks; mpc mode, 22\,385 ticks). Box: quartiles and median (gold); whiskers:
1st and 99th percentile; bar: the slowest tick. Medians 31, 66 and 51\,ms; 99th percentiles 44, 88 and 78\,ms;
no tick reached the 100\,ms budget (dashed), but four UAVs leave about 12\,ms of headroom at the 99th percentile.

## fig:departed_landing and fig:tip_feedback
As written into the draft (see the restructure), numbers unchanged.

# Te-Jen's application demonstrations in simulation (card, 11 Nov 2026; Wesley's request for Te-Jen's individual paper)

**What and why.** Te-Jen asked (via Wesley, 11 Nov) for three simulation demonstrations of the whole task for his own
paper, as fallback evidence if the real-world flights do not complete: pickup, delivery into the basket (the ring's
net), then attachment, with the basket hovering, circling and flying a figure-eight. Reported as SIMULATION results, not
real-world validation. Not the combined paper's experiments.

**Setup (the rig twin, tree bf52a8a + the uncommitted work of 11 Nov; configs `configs/experiments/tejen_app_*_twin.yaml`).**
- QUAD1-3 carry the 0.86 kg ring on 1 m cables, plates 3/11/7, at 1.2 m. QUAD4 starts on the floor (Wesley: floor start).
- QUAD4 (Te-Jen's planner and controller): takes off after the lift, picks up the object (his r 0.08 ball at
  (1.6, 2.0) m), carries it to the moving drop point, settle drop with the release 0.03 m over it, then his contact
  attach on plate 9 (bands 0.05 / 0.05, contact_dz -0.03); our co-motion check confirms the attach and folds QUAD4 in.
  These are the 12 Nov rig defaults. LAND 10 s after the attach is confirmed.
- The ring's commanded motion never pauses (no `attach_pause`; Wesley 11 Nov).
- Case 1 hover. Case 2 circle: radius 0.3 m at 0.2 m/s (orbit, continuous). Case 3 figure-eight: 1.0 m (x +-1.0,
  y +-0.5 m) at 0.2 m/s (Wesley 11 Nov: 1 m, 0.2 m/s for all), repeated laps; each lap starts and ends at rest at the
  crossing point (4 s ramps), the only figure-eight the planner has.
- 5 trials per case, 15 runs, lab PC. Videos (fixed wide camera and a top view) of trial 1 of each case.

**Read per run.** Every phase in order (his phase log); pickup descents; the ball at rest in the net (distance from the
ring centre); the attach: descents, contact, confirmed, fold-in time; ring tilt peak; the ring's path error (desired
against actual); the ring's speed at the drop and at the attach. One figure per run: desired and actual ring path,
ring height, tilt, QUAD4's path, phase marks.

**No bars** (CLAUDE.md, Wesley 2 / 7 Nov): stated as measured, success counted per case (of 5).

**Expected risk.** His attach onto a MOVING ring without a pause has not passed in this twin before: every circle
attach flown so far paused the ring (R4228, R4231, R4247), and a paused attempt failed (R4234). The hover case is
tonight's R4245 from a floor start. A failure on the moving cases is reported as such.

**Falsifier for the setup itself** (the configs, not the method): a run where QUAD4 starts welded, the ring pauses,
the trajectory is not the case's, or LAND does not follow the weld by ~10 s.

## What was flown (11-12 Nov; every run in the registry; per-trial table results/analysis/tejen_app_2026-11-12/table.md)
Judged after the runs, no bars. "Attached" = our co-motion check confirmed the weld (the runner's PASS/FAIL is not the
measure: the 0.55 m configs do not require a weld, and a landing abort after the attach fails the runner).

| Case | Cables | Settings | Attached | Runs |
|---|---|---|---|---|
| hover | 1 m | the 12 Nov rig defaults (bands 0.05) | 5 of 5 | R4254, R4257, R4260, R4275, R4276 |
| circle 0.3 m, 0.2 m/s | 1 m | bands 0.05, then 0.08/0.10 + give-up 120 s, then + lead 0.4 s | 0 of 6 | R4255, R4258, R4269, R4271 (and R4270, R4272 figure-eight) |
| circle 0.3 m, 0.125 m/s | 1 m | 0.08/0.10, give-up 120, lead 0.4 / 0 / 1.0 s, his defaults | 0 of 4 | R4273, R4285, R4287, R4289 |
| figure-eight 1.0 m, 0.125 m/s | 1 m | as the circle | 0 of 2 | R4274, R4286 |
| hover | 0.55 m | October's m1c set (his defaults, lead 0) | 5 of 5 | R4291, R4294, R4297, R4300, R4303 |
| circle 0.5 m, 0.125 m/s | 0.55 m | October's m1c set | 5 of 5 | R4288, R4292, R4295, R4298, R4301 |
| figure-eight 1.0 m, 0.125 m/s | 0.55 m | his defaults; looser limits + 120 s | 0 of 4 | R4290, R4293, R4296, R4299; R4305 |
| figure-eight 1.0 m, 0.08 m/s | 0.55 m | looser limits (match 0.08 / 0.5 s / 0.20, abort 0.10 m / 0.35 m/s held 1 s, hold) + 120 s | 5 of 5 | R4306, R4307, R4308, R4309, R4310 |

Why the moving attach fails on 1 m: at the bottom hold his magnet trails plate 9 by a steady 5-6 cm at 0.125 m/s (about
10 cm at 0.2 m/s), the offset turning with the travel direction: QUAD4 and its 1 m tether lag the moving plate by about
0.9 s, and the plates are 5.5 cm across. His lead time (0.4, 1.0 s) did not remove it; neither did his own defaults.
On 0.55 m it attaches on the circle (as in October: R1508, R1700). On the figure-eight's turns the tip's speed relative
to the plate aborts his descents at 0.125 m/s; at 0.08 m/s it attaches.

Also seen: R4272 (1 m figure-eight, 0.2 m/s) QUAD4's tether struck the formation at the figure-eight's crossing after
an aborted descent, the ring dropped (decisions FINDING 23:05); the planner holds the ring's path 1.5 s at the fold-in
after the attach (a 15 cm path-error blip on the circle); the twin's landing abort (TB14) after the task in some runs.

**Result for Te-Jen's paper (overnight choice, Wesley to confirm):** on 0.55 m cables the whole task (pickup, the drop
into the basket, the attach) completed in every trial: hover 5 of 5, circle (0.125 m/s) 5 of 5, figure-eight (0.08 m/s)
5 of 5. On 1 m cables hover 5 of 5, the moving cases 0 of 12. Reported as simulation.

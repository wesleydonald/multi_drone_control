# QUAD4's bounce on the ring (11 Nov 2026, exploratory analysis of the rig flights of 4, 5 and 11 Nov)

Wesley, 11 Nov: "quad4 bounces when attached during the takeoff and hover ... on a couple of runs it was bouncing a lot
more than the rest". Read from the recorded flights only (tracker logs at 50 Hz, the planner log, the bags); nothing
changed. Scripts: the lab chat's scratchpad `bounce/survey2.py`, `bounce/slack.py` (same reading as below).

> The "Answer" below is the first reading (11:41). The second reading at the end (evening) keeps the slack cable and
> drops the claim that the share alone explains which runs bounce.

## Answer
QUAD4 bounces more when its cable runs close to slack, and it runs close to slack because the plan gives it the
smallest share of the ring's weight. On the triangle with QUAD4 on plate 9 (plates 3/7/9/11) the level-ring balance
puts plate 3, alone on its side, at the most and plate 9, opposite it, at the least: the planned split at the start weld
is 3.79 / 2.17 / 2.16 / 1.62 N (plate 3 / 7 / 11 / 9), so QUAD4 carries ~17 % where an even split is 25 %, and the plan
dips it to 0.53-0.59 N at times (the OCP's floor is tension_min 0.1 N). On the rig it pulls even less: 0.3-1.1 N
(measured cable pull) against the carriers' 1.3-2.6 N. A cable that light goes slack whenever the ring rocks, and a
quad on a cable that keeps going slack and taut bobs.

## Evidence
Welded flights (pretension done -> DETACH), QUAD4 against the carriers' mean, 20 flights with the tracker's cable column:

| flight | QUAD4 bounce / carriers | QUAD4 cable near slack (acm_z > -0.8 m/s^2) | carriers near slack |
|---|---|---|---|
| 11 Nov fm9c_3 | x1.4 (7.3 mm rms, 2.8 Hz) | 41 % | 0 % |
| 11 Nov fm9c_4 | x1.2 | 11 % | 0 % |
| 5 Nov fm9_4 | x1.2 | 4 % | 7 % |
| the other 17 | x0.8-1.1 | 0-3 % | 0-10 % |

Correlation over the 20 flights: bounce ratio against QUAD4's slack share r = +0.77.

After a re-attach the same thing, longer: the fold-in starts QUAD4's share at zero and ramps it over ~10 s, so its cable
is near slack for seconds. nrtfc_2 x1.6 (9.9 mm rms, 0.6 Hz, the cable going from -0.07 to -4.4 m/s^2 over 10 s while
its tracker's thrust-gain estimate drifted 35.2 -> 30.8), sq8_2 (5 Nov) x1.6, nrtfc_3 x2.4 (includes the magnet letting
go, the loose lead). nrtfc_5's re-attach x0.9.

## What it is not
- Not the command path: QUAD4's commands pass the mux in 0.8 ms (p95 4.3 ms), at the carriers' 50 Hz, only our stream
  (nrtfc_2, 428 of 428 matched).
- Not the tracker settings: QUAD1's and QUAD4's tracker parameters are identical but for the id and the mux topic.
- Not QUAD4's thrust map: its thrust commands against its cable load fall on the carriers' line (7 flights).
- Not the object's mass: the planner adds it to QUAD4 only at the pickup (`his ball on`).
- Not the circle: QUAD4 was welded only in the hover hold (traj_t 0).

## Options (Wesley's call; none flown)
1. Raise the planner's minimum cable tension (planner_ocp.py `tension_min`, 0.1 N now) to ~1.0 N, so no cable is
   planned near slack, welded or in the fold-in. A rebuild of the planner solvers, then the twin, then the rig.
2. Start the fold-in with QUAD4 at that floor instead of zero, and hold its thrust-gain estimate during the fold-in.
3. The layout itself: any QUAD4 plate in a gap of the triangle gets the same ~17 % (the gaps are equivalent); only an
   even layout (the square 0/3/6/9) splits 25 % each, and the triangle was chosen over the square (6 Nov).

## Second reading (11 Nov evening): bouncy runs against calm runs (Wesley: "it was mostly in the last few runs")
Scripts: the lab chat's scratchpad `bounce/` (stages.py, tipnoise.py, timing.py, slackcycle.py, refgeom.py).

The bounce is QUAD4's cable going slack and snapping taut. Body-to-magnet-marker distance, pretension to DETACH:

| flight | ring height | funnel on 9 | stack | cable slack more than 4 mm | deepest | main frequency |
|---|---|---|---|---|---|---|
| nrtfc_2 | 1.0 m | v4 | re-attach | 14 % | 6 mm | - |
| nrtfc_3 | 1.0 m | v4 | re-attach | 9 % | 5 mm | - |
| nrtfc_5 | 1.0 m | supervisor's | re-attach | 21 % | 7 mm | - |
| fm9c | 1.2 m | supervisor's | pickup | 36 % | 22 mm | 4.0 Hz |
| fm9c_3 | 1.2 m | supervisor's | pickup | 69 % | 36 mm | 2.9 Hz |
| fm9c_4 | 1.2 m | supervisor's | pickup | 51 % | 33 mm | 3.4 Hz |
| fm9c_6 | 1.2 m | supervisor's | pickup | 36 % | 6 mm | - |

What is the same in both groups: the planner's tick (104-113 ms), the trackers' loop (dt p95 32-38 ms), the mocap rate
(89-93 Hz in the bag, longest gap 30 ms), the mux (0.8 ms), the pack voltages, QUAD4's tracker parameters. In the twin
(R4233 at 1.0 m, R4243 and R4239 at 1.2 m, R4228 on the circle) QUAD4's bounce and tilt equal the carriers' at every
height; its rigid links cannot go slack, so it only rules out the plan itself oscillating.

What holds across the 11 flights checked since 4 Nov: QUAD4's position reference sits 20-45 mm closer to its magnet
(seated on plate 9, as measured) than its taut cable length in all 11; the ring hangs with plate 9 low by 11-19 mm
(plate 3 high) in 10 of them (5 Nov nr9o_2: level within 2 mm); QUAD4's measured pull is 0.5-1.5 N against 1.6 N
planned. So its cable is only just taut; in the three bouncy flights it tipped into a slack-taut cycle. Second by second,
slack and the ring's level go together (r +0.24 to +0.65; in fm9c_3 plate 9 was 5 mm low while slack, 15 mm while taut).

Not the cause on its own: the supervisor's funnel (nrtfc_5 calm), 1.2 m with the pickup stack (fm9c_6 calm), the rod
length the plan measured at handover (QUAD4's reads 7-14 mm under its in-flight median in both groups; the 95th
percentile is 19-22 mm over in the bouncy flights, but the snap itself stretches that reading).

Open: why the three tipped over. Levers: (a) built, default off, not flown: `partner_rod_add_m` (script switch
`QROD=0.010`): metres added to the start-welded QUAD4's hand-over cable reading, so its reference sits further out
along its cable. A planned minimum tension was NOT built: the trackers apply the measured cable pull (`cable_source:
mocap`), so a planned tension floor would not change QUAD4's flight; (b) `rod_len_reread` (exists since 31 Oct,
twin-tested, never flown: the rods read again under full pull before the lift; a start weld cancels it); (c) a longer
typed cable for QUAD4. One line per flight on the rig: `tools/rig_cable_check.py <flight>` (the table above, plus the
ring's level and QUAD4's longest frozen pose).

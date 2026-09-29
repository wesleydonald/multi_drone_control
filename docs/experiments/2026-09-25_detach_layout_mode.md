# 2026-09-25 Detach: which plate layout and which reconfiguration mode keep the ring level

## 1. Question
The end goal (Wesley, 2026-09-24): one drone leaves the ring, picks up an object, places it
in the carried basket and rejoins, while the fleet flies a circle. Priority: stability and
payload orientation. On the even 4-ring the 4 → 3 detach hangs the 0.86 kg ring at 31–41°
(R0493/R0494), because any three of four 90°-spaced plates leave a 180° gap: the centre of
mass sits on the edge of the survivors' hull. Which plate layout and which handling of the
transient (OCP resize in place, network then OCP hand-back, network only) keep the ring
level through a detach in hover and during the circle?

## 2. Geometry (free, computed 2026-09-24)
Hull margin = distance from the ring centre to the nearest edge of the survivors' triangle
(R = 0.25 m; ≤ 0 means the load cannot hang level):
| 4-ring (deg, plates every 30°) | leaving drone | survivors | margin after | margin before |
|---|---|---|---|---|
| 0 / 90 / 180 / 270 (even) | 90 | 0 / 180 / 270 | 0.000 | 0.177 |
| **30 / 90 / 150 / 270** | 90 | **30 / 150 / 270 (even triple)** | **0.125** | 0.125 |
| 0 / 90 / 120 / 240 | 90 | 0 / 120 / 240 (even triple) | 0.125 | 0.125 |
| 0 / 90 / 150 / 270 | 90 | 0 / 150 / 270 | 0.065 | 0.125 |
The leaving drone sits between two survivors 60° either side, the fourth opposite: the
survivors are a perfect 120° triple, the 4-ring before the detach is uneven (gaps
60/60/120/120), which the OCP handles (attach geometry is a runtime parameter). On the
real ring these are plates 1, 3, 5 and 9. The rejoin puts the drone back at 90°.

## 3. Variables (one per arm, SIL free, then Gazebo)
A. Layout: even 0/90/180/270 vs 30/90/150/270, detach of the 90° drone in hover, OCP mode.
B. Mode at the 30/90/150/270 layout: `reconfig_mode` ocp (resize in place) vs network
   (network carries the transient, hands back to the n−1 OCP once still) — both exist.
C. Timing: detach in hover vs during the 0.4 m/s circle, best layout and mode from A/B.
Falsifiers: payload tilt > 25° settled after the detach; a survivor tilt fault; load
height error > 5 cm settled; the circle not resumed within 5 s (C); an abort.

## 4. Baseline
R0494 (even ring, OCP mode, hover detach then circle at 0.6 m/s: settled tilt 31°, max 57°).

## 5. Pass / fail
| arm | supports | falsifies |
|---|---|---|
| A even vs 30/90/150/270, OCP, hover | 30/90/150/270 settles below 5° tilt within 5 s; even ≥ 30° | 30/90/150/270 ≥ 15° |
| B ocp vs network at 30/90/150/270 | the mode with the smaller transient peak tilt and the faster settle, both under 25° peak | both peak > 25° |
| C hover vs circle at the best of A/B | circle detach peak tilt within 10° of the hover detach; circle resumes ≤ 5 s | peak > 25° or abort |
| Gazebo confirmation, best config, ×2 | same bands | anything else |

## 6. Repeats
SIL one per arm. Gazebo 2 for the chosen configuration.

## 7. Cost
SIL: 6 runs, free (the bench releases the rod in its plant on the DETACH event and
publishes /fleet/detach, `bench_node._fire`). Gazebo: 2.

## 8. New code
None. Configs only: `configs/sil/detach_*.yaml`.

## 9. Beyond this card (the end goal)
- The freed drone's mission (pick-up, basket) is another student's controller: the hold
  point after the OCP-mode detach is its start state (Wesley's word 2026-09-24).
- Rejoin during the circle = the attach path: the mux hand-over zero-throttle gap and the
  one-sided hand-out are open (CURRENT_STATE §4.6); attach at 0.2 m/s is locked.
- Basket on the ring: the payload mass and inertia change when the object is placed;
  `load_mass` is a launch value, the height integral and the trim absorb the change
  bounded (0.15 m / 25 %); to be checked in SIL with a mass step mid-flight.

## 10. Outcome, first SIL matrix (2026-09-24 late, trim on, z_ki 0.4, R0496–R0501)
| arm | layout | mode | when | 4-ring before | detach peak | settled | run |
|---|---|---|---|---|---|---|---|
| A | even | ocp | hover | level, 0.600 | 57° | 41° | R0496 |
| A | 30/90/150/270 | ocp | hover | never lifted (stands, tensions flipping between vertices) | 3.4° | 1.0° | R0497 |
| B | 30/90/150/270 | network | hover | 0.61 m at 12–27° tilt | 53° | abort | R0498 |
| C | 30/90/150/270 | ocp | circle | 17.6° lean | 18.7° | 2.0° in 5.6 s, circle continues | R0499 |
| C | 30/90/150/270 | network | circle | 17.4° | 61° | abort | R0500 |
| C | even | ocp | circle | 0.37 m | 30° | abort | R0501 |
Mode: the network cannot carry this load's transient (53–61° in both arms); the OCP
resize in place is the only mode to fly. "Dissipative for the transient then OCP" is the
network mode, and it is the one that aborts.
Layout: the survivors 30/150/270 hang level after the detach (1–2°), exactly as the
geometry says, and the circle continues. But the 4-ring BEFORE the detach is wrong on the
uneven layout: the reference builder feeds every cable the same nominal tension, which
on 30/90/150/270 carries a net moment, so the OCP hovers tilted (R0498) or flips between
vertex tension solutions on the stands and never lifts (R0497).
Fix (built the same night): `geometry.balanced_tensions` — the least-squares static
balance (zero net force and moment) closest to the equal split, per layout; the even
ring is unchanged, the survivors' triple is even. Used for `t_ref` in `yref_at` and
`hold_yref`. Unit-tested. Re-fly of A/C on 30/90/150/270 and the even-ring regression
running (R0502–R0505).

### Re-fly with balanced tensions (R0502–R0505): inconclusive, the 4-drone SIL air start is the problem
R0505 (n3, balanced = equal split) is unchanged at 0.600. R0502/R0504 (30/90/150/270)
never left the stands: 4 drones at 0.83 m for the whole run, load hanging at 0.31; the
detach on that hanging load was level (peak 1.3–1.9°, as the geometry says). R0503 (even
ring) is a startup hang of the dissipative node (no planner ticks). Tonight the 4-drone
SIL air start lifted in 2 of 5 runs (R0496 even/ocp, R0498 3915/network) and not in 3
(R0497, R0502, R0503, all ocp mode). The tick-1 rod measurement was refused in the 3915
runs because one drone's pose had not arrived (a 2.5 m "rod"), which is harmless, and
the trackers' spool is the next thing to read. Decision: stop chasing the SIL stand
start for four drones; the test that matters is the Gazebo floor start (creep) with four
drones on 30/90/150/270 and the balanced tensions, then the detach in hover and in the
circle: configs `ocp_hover_ground_creep_n4_3915.yaml` and `detach_ocp_n4_3915.yaml`.
Answered tonight regardless: mode = OCP resize (network aborts at 53–61°); layout =
30/90/150/270 gives a level triple after the detach (R0497/R0499/R0502/R0504: 0.5–2°).

## 11. Gazebo, 2026-09-25 (headless, linear plant, trim on, z_ki 0.4, balanced tensions)
| arm | config | result | run |
|---|---|---|---|
| 4-ring hover on 30/90/150/270 | ocp_hover_ground_creep_n4_3915 | level: tilt 0.1–0.9° (peak 2.7°), load 0.600 from 26 s to LAND, no abort; throttle 0.158/0.148/0.158/0.191 (drone 3 at 270° carries the most) | R0510 |
Wesley's interactive runs the same morning (W-07:21 hover, W-07:25 circle detach of drone 1):
lifted, held, detached and continued the circle; payload tilt not on disk for those two
(the planner now writes `log.csv` at 10 Hz beside its params.json, R0508/R0509).
The balanced-tension fix is what made the uneven 4-ring hover: SIL R0498 with equal
tensions sat at 12–27°, R0497/R0502/R0504 never lifted.

## 12. Gazebo claim, circle detach on 30/90/150/270 (fixed sim inner loop, clock 500)
| run | detach | peak (6 s) | < 5° after | settled | LAND | verdict |
|---|---|---|---|---|---|---|
| R0554 | 22 s fixed, load up since 14 s | 7.7° | 1.1 s | 0.3° (max 0.9) | clean, freed drone down first | supported |
| R0556 | lift + 8 s (WAIT_LIFT), load moving 0.43 m/s | 6.9° | 1.0 s | 0.3° (max 0.9) | clean | supported |
Void: R0553 (fixed-time detach hit the ring at liftoff: the creep lift varies by ~9 s,
hence the runner's new WAIT_LIFT event), R0555 (wall-clock pose watchdog before lift).
Against SIL R0499 (18.7° peak, 2.0° settled) Gazebo is gentler. The layout answer holds:
plates 1/3/5/9, leaving drone at plate 3, OCP resize, in hover and mid-circle.

## 13. The claim, re-flown after the reviewer (2026-09-25 evening, one tree)
Protocol: creep start, WAIT_LIFT 0.5 m, DETACH of drone 1 (90°) 12 s after the lift in HOVER,
32 s airborne on the triple, LAND. Fixed sim inner loop, clock 500, pose_timeout 1.0, trim on,
z_ki 0.4. "Settled" = last 20 s before LAND, load airborne.
| layout | runs | peak | < 5° after | settled tilt | height |
|---|---|---|---|---|---|
| 30/90/150/270 | R0559, R0567, R0569, R0570, R0571, R0572 | 3.0–3.3° | never above 5° | 0.1–0.2° (max 0.37) | 0.600 |
| 0/90/180/270 (even) | R0561, R0562 | 25.6–28.5° | 1.4 s, then back above 5° until ~32 s | 6.7–6.8° | 0.600 |
Mid-circle (0.4 m/s) on 30/90/150/270: R0554 7.7°, R0556 6.9° peak, < 5° in ~1 s; the circle is
held 1.5 s at the resize and the lap then ends, so no settled figure in the circle.
Void: R0553 (fixed-time detach at liftoff), R0555, R0560 (a tracker died before ARM: fleet
manager fixed, see below), R0563.
LAND, two fixes found on the way (card 2026-09-24_detach_land_fix): the freed drone now steps
out to 1.2 m from the ring centre before descending (R0567: straight down it landed 0.63 m out,
its trailing cable under the ring, pinned and tipped, abort) and its reference freezes at its
stall (R0570: a reference sinking into the floor tipped it 0.2 s before the fleet disarm).
R0571/R0572 land clean: freed drone 1.20 m out, ≤ 4° while armed, survivors ≤ 19°.
Fleet manager (main.py): an ARM with any drone failed now aborts and refuses TAKEOFF (R0560),
critic "ready with changes" applied, 4 unit tests.

Reviewer, second pass (2026-09-25): SUPPORTED with corrections, applied above. Parameter dumps identical except the layout; four diff SHAs across the six 30/90/150/270 runs (LAND work between them), detach numbers within 0.37° across them; R0562 detached 17.4 s after the lift (others 12.0 s). LAND v4 = R0571/R0572 only (R0569 had the step-out without the stall freeze): 2 of 2 clean.

## 14. More room for the leaving/joining drone (Wesley 2026-09-25)
With plates every 30°, three choices exist for the leaving drone at 90° (rig rods 0.47 m, drones
0.58 m from the ring centre):
| layout (plates) | room either side | survivor margin | hover detach peak | settled | busiest survivor throttle | runs |
|---|---|---|---|---|---|---|
| 1/3/5/9 (30/90/150/270) | 0.58 / 0.58 m | 0.125 m | 3.0–3.2° | 0.11° | 0.181 | R0571, R0572 |
| **1/3/6/9 (30/90/180/270)** | 0.58 / **0.82 m** | 0.065 m | 5.7° | 0.54–0.57° | 0.199 | R0582, R0584 |
| even 0/3/6/9 (0/90/180/270) | 0.82 / 0.82 m | 0 | 25.6–28.5° | 6.7–6.8° | — | R0561, R0562 |
Two shifts is ruled out by geometry (three survivors with a 180° gap hang the ring). One shift buys
42 % more room on one side for +2.5° peak and +0.45° settled; the busiest survivor works ~10 % harder
(on the rig, at kT 24, about 0.63 → 0.69 throttle). The same geometry applies to the rejoin: the
newcomer comes back into a 150° gap (60° + 90°) instead of 120°. R0583 void (the new ARM gate aborted
a failed ARM before takeoff, as designed).
Rejoin with one shift (tethers 330/90/180, newcomer at 270; world three_attach_rod_3969.sdf): R0586
weld peak 11.2°, 10–20 s mean 4.2° with four on the circle (current layout R0580/R0581: 2.6 / 0.9°);
the newcomer carries ~2.3 N. One run.

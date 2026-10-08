# [SHELVED, not built] A drone landing by itself lays its rod down (no stilt)

Shelved 9 Oct 00:40 after the critic's REWRITE: the twin "stilt" was the stall freeze pulling a falling rod back up, and the rig's failure (4/4 landings of the freed QUAD4 on 8 Oct) is a float, not a stilt; the arc fixes neither. The critic's simpler alternative (no freeze and no 'down' above rest + margin, the reference keeps descending) is part of 2026-10-09_departed_ground_idle rev 2, whose B1 neighbour (twin_gi_b1) tests whether the rod falls by itself. The arc is built only if it still stands there.

1. **Question.** A drone that leaves the ring and lands by itself (an aborted newcomer; a freed carrier with
   `departed_land`) carries a 0.55 m rod hanging below it. Does it reach the floor, or stand on its rod?
2. **Hypothesis.** Its reference descends straight down at 0.4 m/s, so the magnet tip meets the floor with the rod
   vertical under the body (body at its body-to-tip height, 0.60 m in the twin). Whether the rod tips over is a coin
   flip: in A1 (R1250) it fell within 1 s and the drone landed; in B1 (R1253) it balanced (8-19 deg wobble), the
   touchdown detector read the 1.0 s stall as touchdown at 0.586 m, froze the reference there, the landed gate
   counted it down and the fleet disarm dropped QUAD4 from 0.6 m. With `departed_lay_rod` on: once the reference
   is at the body-to-tip height above the drone's resting height (+0.03 m: the tip just above the floor), it
   follows `land_arc_ref` (the carriers' LAND unwind) about the tip's ground point, outward along the drone's own
   radial from the ring centre: the rod pivot swings from vertical to level at land_vel, so the tip stays put and
   the rod lies down, then the drone descends onto its skids and the touchdown detector (armed only after the arc)
   finds the stall at the floor. The landed gate counts a departed drone as down only if it stalled within 0.08 m
   of its resting height (or is below departed_down_z, as today).
3. **The one variable.** `departed_lay_rod` (bringup common.yaml, default false; planner param):
   dissipative_node.py `_publish_departed_refs` (the arc after the trigger height; the detector reset after it) and
   `_departed_down` (the height condition). Resting height: an unwelded newcomer's z at ATTACH (`_approach_z0`), a
   carrier's z at TAKEOFF (recorded); body-to-tip: the newcomer's `_attach_body_to_tip`, a carrier's rod + pivot drop.
4. **Baseline.** R1250 (A1: rod fell, landed, disarm 0.144 m), R1253 (B1: stilt at 0.60 m, disarmed at 0.602 m),
   R1226/R1227 (freed carrier in the r305 config: landed at 0.100 m).
5. **Pass / fail.**
   | Run | Config | Supports if | Falsified if |
   |---|---|---|---|
   | L1 | twin_nc_b1_off_hold + switch on | QUAD4's tip touches with the rod tilting outward and the body follows the arc (rod angle from vertical rises monotonically to > 60 deg); body z <= 0.13 m within 6 s of the trigger; 'is down' below 0.13 m; disarm z <= 0.13 m | a stall above 0.2 m, or disarm above 0.15 m |
   | L2 | twin_nc_a1_land_hold + switch on (in LAND) | as L1 | as L1 |
   | L3 | w4_3915_release_orbit_drop (r305 config) + switch on | the freed carrier lands as L1; survivors as R1226/R1227 (tilt peak +/- 2 deg) | as L1, or survivors change |
   Neighbour: A3's config (lands before the ring) with the switch on.
   Unit tests: the arc starts at the trigger height, keeps the tip point fixed, goes outward from the ring, ends
   level; the detector does not run during the arc; a high stall is not "down".
6. **Repeats.** L1 x2 (the coin flip needs two), L2 x1, L3 x1, A3 neighbour x1.
7. **Cost.** 5 Gazebo runs.
8. **New code?** Yes, a landing law and a gate. Behind `departed_lay_rod`, default off. It matters on the rig for
   R8a (LAND with QUAD4 in the hold) and for the freed drone in W4/W5 (departed_land); default only with these
   rows, the reviewer and Wesley's word. Removal: drop the commit.

## Critic

Verdict REWRITE (9 Oct 00:35); shelved instead. Main points:
- The stall freeze (dissipative_node.py, departed 'is down') caused the twin stilt: in R1253 the tip rolled from 1.3
  to 18.7 cm off the body's vertical between 62.52 and 63.51 s, the 1.0 s stall timer froze the reference at
  0.586 m, throttle rose 0.354 -> 0.471 and the rod swung back vertical. In A1 (R1250) it fell past the point of no
  return before the timer: a race with the timer, not a balance.
- Rig, same airframe (QUAD4 freed in the detach runs, 8 Oct; rest 0.061 m): no stilt in 4/4; each stalled at about
  0.17-0.21 m, the freeze stepped its reference up to that height and it rose:
  | run | stall z (m) | reference after "is down" (m) | z at disarm (m) |
  |---|---|---|---|
  | RIG r302_3 | 0.18-0.20 | 0.206 | 0.367 |
  | RIG r303_4 | 0.198 | 0.199 | 0.340 |
  | RIG r304_3 | 0.17-0.19 | 0.189 | 0.352 |
  | RIG r305 | 0.19-0.21 | 0.216 | 0.377 |
  Its released tether resting on the floor makes it lighter (r305 logs a measured +1.1 to +1.7 m/s^2 upward 'cable').
- The arc would also move every landing spot 0.55 m further out (decision 'Newcomer abort path', the 1.6 m clear
  radius on the 8 Oct sheet), its centre would be in the air (the trigger mixed the body's rest with the tip's floor
  height), and two runs cannot beat a coin flip. Not built.

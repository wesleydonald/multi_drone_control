# A departed drone landing by itself reaches its resting height and idles there while the ring flies

Revision 3 (critic REWRITE on rev 1 (wrong diagnosis), then GO on rev 2 with N1-N3 to fix first: done).

1. **Question.** A drone that leaves the ring and lands by itself (an aborted newcomer; a freed carrier with
   `departed_land`, Wednesday's W4/W5) should sit on the floor at armed idle until the fleet lands. Does it?
2. **Hypothesis.** No. A departed drone cannot reach a floor reference: its tracker has no vertical integrator and,
   with its rod (or released cable) and magnet resting on the floor, it needs less thrust than its model, so it
   hovers above any reference; the planner's touchdown detector reads "the descent stopped" as touchdown, freezes
   the reference there and the drone rises further; the tracker's ground_idle needs a resting ring and its own
   height within 3 cm of rest, so it never latches; the fleet disarm at LANDED drops it.
   - Twin R1252 (A3, tracker_3 log rows 255-380): sinking to 0.118 m against a reference at 0 (rest 0.100), "is
     down" at 0.118 m, reference frozen at 0.1185, then steady at 0.239 m (throttle 0.44); disarmed at 0.237 m.
   - Twin R1254 (B2): the same float to 0.236 m. R1226 (freed carrier, r305 config): needed exactly its model
     thrust and sat at 0.100-0.102 m, at throttle 0.48 on the floor (no idle either).
   - **Rig r305 (8 Oct)**: rest 0.061 m; reference clamped at 0 with the drone at 0.193 m, stall, reference frozen
     at 0.216 m, the drone hovered at 0.37-0.38 m (throttle 0.45) for ~10 s with the ring resting, then the fleet
     disarm dropped it ~0.32 m. Wednesday's W4/W5 freed drone will do the same with today's code.
   With `departed_ground_idle` on: the planner records each drone's resting height at TAKEOFF; a departed drone's
   descending reference goes down to DEPARTED_PRESS_M (0.35 m, sized from the rig's 0.19-0.22 m floor offset) below
   its rest and never further than that below the drone itself (a gentle press, also on a drone standing on its
   rod); 0.5 s at its rest (within 0.03 m) makes it down and ends the press (also a drone that never left the
   floor, which the stall test cannot see); a stall more than 0.03 m above its rest is not "down" and gets at most
   2 s more press, then it is held at its own height (warned, not down); the landed gate counts it down only within
   0.03 m of its rest; resting heights are logged at TAKEOFF and cleared at ARM; refused on an air start; the planner
   latches `/drone_d/departed` True when its own descent starts (False at ARM and TAKEOFF); the tracker's existing
   ground_idle gate also latches for a departed drone (no resting ring needed, no airborne latch: the flag implies
   TAKEOFF) when |z - its resting z| <= 0.03 m with its reference at the floor for 0.2 s, and only on a departed
   flag received after this ARM.
3. **The one variable.** `departed_ground_idle` (bringup common.yaml default false, PLANNER_OPTIONAL; planner):
   dissipative_node.py `_departed_flight_start`, the departed descent and stall in `_publish_departed_refs`,
   `_departed_down`; tracker_node.py `_departed_cb` and the departed branch of `_ground_idle_update` (acts only with
   the tracker's ground_idle on, the default since 5 Oct; inert without the planner's flag). Off in partner runs.
4. **Baseline.** R1252, R1254 (float), R1253 (stood on its rod), R1226/R1227 (freed carrier at throttle 0.48 on the
   floor), rig r305 (hover at 0.37 m, dropped ~0.32 m).
5. **Pass / fail.** Per departed drone (QUAD4 in the twin_gi rows, the freed carrier in the w4_gi rows; carriers
   in LAND are reported, not barred), `tools/departed_metrics.py` on run.csv (rest = its first 2 s): disarm_dz
   <= 0.03 m; press_max_s <= 1 s (reference >= 0.1 m below it within 0.3 m of rest, not idle); tilt_idle <= 10 deg;
   stand_s < 2 s (still between rest + 0.3 and + 0.8 m on its way down); idle_t before LANDED. The planner's 'is down
   (at its rest)' / 'held there' lines and the tracker's 'on the floor after the flight' line are in launch.log; any
   'held there' line = not supported for that row.
   | Run | Config | Supports if | Falsified if |
   |---|---|---|---|
   | G1 | twin_gi_g1_a3 (A3 + switch: lands before the ring rests) | the tracker's 'on the floor after the flight' line before LANDED; |z - rest| <= 0.03 m at that latch; throttle 0 from the latch to the disarm; tilt <= 10 deg from the latch to the disarm; disarm z - rest <= 0.03 m | an "above its rest" stall that never comes down, throttle > 0.05 after the latch, disarm z - rest > 0.03 m |
   | B1n | twin_gi_b1 (B1 + switch: aborted while the carriers hover) | as G1, while the ring hovers | as G1, or stand_s >= 2 s (then the shelved lay-rod card returns) |
   | G2 | w4_gi_g2_release_orbit (r305 config + switch) | the freed carrier as G1 while the fleet circles; survivors as R1226/R1227 (tilt peak +/- 2 deg) | as G1, or the survivors change |
   | G3 | twin_gi_g3_a1 (A1 + switch, in LAND) | QUAD4 as G1 (R1250's 'is down' came after the fleet's LANDED: here it must be down first) | as G1 |
   | N1 | w4_gi_release_drop (hover release) | as G2 | as G2 |
   | N2 | w4_gi_drop_forced (every drone departed, the ring resting) | every drone as G1, the drop as R1205/R1210 | as G1 |
   G2 in the twin cannot stand in for the rig's floor offset (the twin carrier had none, R1226); the rig check is
   W4/W5 on Wednesday, only if Wesley makes it live. A newcomer taken out before it is 5 cm up never stalls: the
   gate counts it at its rest (unit test).
6. **Repeats.** G1 x2; B1n, G2, G3, N1, N2 x1 (neighbours before the G1 repeat).
7. **Cost.** 7 Gazebo runs.
8. **New code?** Yes, a change to a gate (the tracker's idle latch) and to the departed landing. Default off; to
   the rig (W4/W5, R8a) only with these rows, the reviewer and Wesley's word. Rig question for Wesley (critic S3):
   a drone armed at throttle 0 for 10-30 s, possibly tilted on its rod: Betaflight airmode and I-term behaviour.
   Removal: drop the commits.

## Critic

Revision 1 (9 Oct 00:15): REWRITE. M1 the mechanism (light on its rod, no integrator, the stall read as touchdown)
and the rig evidence (r305): restated above. M2 floor relative to rest and deep enough, no freeze/'down' above rest:
done. M3 the gate with 'payload_resting or departed', not a one-shot: done. M4 False at ARM/TAKEOFF, after-ARM only,
airborne first: done. M5 two-sided margin for the departed path: done. M6 bars: done. M7 neighbours: B1n, N1, N2;
the 5 cm case as a unit test. M8 registry rows for R1250-R1254 and the r305 'down' correction: at the end of batch 5.
S1 carrier pose staleness: open (the tracker's own two-sided check is the guard). S2 the stilt: the lay-rod card
(B1n shows whether the gentle press is enough). S3 Betaflight at throttle 0: question for Wesley. S4 partner off:
done. S5 named `/drone_d/departed`. S6 G2 note: done. S7 the W4/W5 rig behaviour: in the morning report.

Second pass (9 Oct 00:50, on rev 2): GO after N1-N3, done. N1 a drone that never left the floor was pressed for
ever: 0.5 s at its rest ends the press, and the tracker's departed branch no longer needs the airborne latch (both
tested). N2 the press after a high stall had no limit: 2 s, then held at its own height and warned (tested). N3/N4
bars with numbers and a tool (tools/departed_metrics.py). S1 press 0.35 m. S2 rest heights cleared at ARM, logged
at TAKEOFF, a missing pose warned. S3 refused on an air start. S4 warnings reset. M8 registry rows: with batch 5's.

## Result (9 Oct night, twin, rig-prep 38fa1f3 / 7da010a)

Per departed drone (tools/departed_metrics.py; bars: disarm z - rest <= 0.03 m, press <= 1 s, tilt after idle
<= 10 deg, stand < 2 s, the tracker's idle before LANDED, no 'held there' line).
| Row | Run | Who | Back at rest -> idle (s) | Disarm z - rest (m) | Press (s) | Stand (s) | Verdict |
|---|---|---|---|---|---|---|---|
| G1 | R1267 | QUAD4 (lands before the ring) | 0.22 | 0.000 | 1.02 (final descent; 0.19 on the floor) | 0.35 | grey (press by 0.02 s) |
| G1 | R1279 | QUAD4 | 0.17 | 0.000 | 0.96 | 0.41 | supports |
| B1n | R1268 (before 6c1bc5b) | QUAD4 (aborted, carriers hover) | 0.20 | 0.000 | 1.03 | 0.84 | not supported: 'held there' at +0.00 m (timer wrinkle, fixed) |
| B1n | R1276 | QUAD4 | 0.21 | 0.000 | 0.99 | 0.41 | supports |
| B1n | R1277 | QUAD4 | 0.20 | 0.000 | 1.00 | 0.95 | supports |
| G2 | R1269 | freed carrier (fleet circles) | 0.20 | 0.000 | 0.81 | 0.0 | supports; survivors as R1226/R1227 |
| G3 | R1275 | QUAD4 (in LAND) | reached rest at LANDED | 0.003 | 0.85 | 0.52 | grey (the gate waited; the fleet disarm came before the idle line) |
| N1 | R1271 | freed carrier (hover release) | 0.19 | 0.000 | 0.82 | 0.0 | supports; survivors as R1221/R1225 |
| N2 | R1272 | all four (forced drop) | at rest before LANDED | 0.000-0.008 | <= 0.85 | 0.0 | supports |
VOID: R1270, R1273 (the planner lagged at startup and had no newcomer pose at TAKEOFF: the switch fell back to the old
landing as designed; rest heights now come from the last pose, 02dba91), R1278 (a carrier flipped in the creep, cause
open, nothing of this switch acts before ATTACH).

Baselines without the switch: QUAD4 floated 0.24 m (R1252, R1254, R1255, R1257) or stood on its rod (R1253) and was
dropped at the fleet disarm; the freed carrier sat at throttle 0.48 on the floor (R1226). Verdict: SUPPORTS in the
twin: every drone that lands by itself now reaches its own rest and idles there (disarm at rest in every scored row),
with the ring still in the air; the two greys are bar-letter edges, not failures of the behaviour. The rig's offset
(0.19-0.22 m near the floor, r305) is larger than the twin's (0.12 m): the 0.35 m press is sized for it but untested.

Reviewer (9 Oct 03:15): WEAK. Taken:
- Two trees: R1267, G2, N1, N2 flew 38fa1f3; R1279, B1n x2, G3 flew 7da010a (after 6c1bc5b and 02dba91, which change
  this path). On the rig tree: G1 x1, B1n x2, G3 (grey).
- The press bar cannot separate rows: all five QUAD4 rows read 0.96-1.03 s, the designed final descent (0.16-0.19 s
  of it on the floor); grey vs supports turns on 40 ms.
- N2 is grey, not supports: drones 0-2 never idled (throttle 0.41-0.43 at the disarm, z 0.101-0.109 m); the drop
  path announced LANDED (43.72 s) before their 'is down (at its rest)' (44.01-44.11 s). Its baselines also differ in
  int_mode.
- N1 does not test the switch: the freed carrier came down in the fleet LAND, as the baselines did (idle at the same
  point); survivor tilt peak 5.91 deg vs R1221 3.59 (+2.32, outside the +/-2 bar; the registry misquoted R1221 as
  4.49). Not supported.
- "Idles at its rest in every scored row" is false (G3, N2); "disarmed at its rest" holds in every scored row
  (<= 0.009 m above rest).
- R1278 is not a plain void: an unexplained carrier flip on the rig tree; the switch does act at ARM/TAKEOFF
  (departed False, rest heights), though nothing in the log points to it.
So the twin claim stands on G1 (R1279), B1n x2 and G2; for W4/W5 the freed-carrier evidence is G2 x1 on 38fa1f3 in a
twin with no floor offset.

## On the rig candidate (52ede66 / c0604f7, after the reviewer)

| Row | Run | Result |
|---|---|---|
| combined A1 | R1280, R1281 | QUAD4 down at its rest at the fleet disarm (0.000 m), the gate waited (no idle before LANDED) |
| combined B1 | R1282 | **rod stand**: landed on its rod tip at +0.50 m, the 2 s press tipped it only to +0.25 m, then the press limit held it there; stood 35 s, the gate fell back after 15 s, dropped 0.37 m |
| combined B1 | R1283 | rod fell in 0.72 s; idle 0.19 s after rest; disarm at rest: supports |
| G2 | R1288 | the freed carrier idles 0.18 s after rest, disarm at rest; survivors 9.2 deg at the release transient (outside +/-2, before the switch acts): grey |
| N2 | R1289 | all four disarmed at their rest; three reached rest at LANDED (no idle): grey |

B1-type with the switch: 3 of 4 land and idle (R1276, R1277, R1283), 1 of 4 loses to the rod stand (R1282). The
press limit (critic N2: no endless press) is what held R1282 on its rod: the open design question is a longer press
or a sideways nudge to tip the rod.

## The rod tip (9 Oct 04:45-05:50, rig candidate)

QUAD4 landing by itself (combined B1) comes down on its rod tip at +0.47-0.50 m in almost every run; what decides
the landing is whether the press tips the rod over before the press limit holds it there.
| Press limit | Runs | Landed and idled | Held propped (disarm z - rest) |
|---|---|---|---|
| 2 s (critic N2) | R1282, R1283, R1295-R1298 | 2 (R1283, R1295) | 4 (R1282 0.37, R1296 0.23, R1297 0.21, R1298 0.27 m) |
| 6 s (exploratory, 6e0c825) | R1299-R1302 | 4 (stand 0.8-1.8 s, idle 0.18-0.20 s after rest, disarm at rest) | 0 |
Kept on rig-prep: the press is still time-bounded (the critic's N2), only longer.

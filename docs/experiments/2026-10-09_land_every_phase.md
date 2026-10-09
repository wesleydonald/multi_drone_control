# land_every_phase (9 Oct 2026): LAND lands every drone, whatever it is doing

## 1. Question

Wesley, 9 Oct: "The land bug needs to be fixed, LAND should land all drones safely, regardless of what they
are doing." The fix becomes the default after a LAND-in-every-phase twin matrix and the reviewer.

## 2. Hypothesis

With the three newcomer switches on (`newcomer_lifecycle`, `newcomer_fault_land`, `departed_ground_idle` =
push to rest + disarm alone) and the two LAND fixes (0400066: LAND before TAKEOFF disarms the armed fleet;
LAND in the floor creep ends the creep with no lift and descends), a LAND in any phase below brings every
drone down and disarmed at its rest, with no abort and no drone tipping.

## 3. The one variable

The switches on (they are the candidate defaults); everything else as the twin defaults. One code tree for
the whole table: rig-prep 7d63170 (the tip feedback in it is off by default). Configs (all existing):

| phase | config |
|---|---|
| e0 armed on the floor, before TAKEOFF | twin_land_e0_prearm |
| e1 floor creep, 1.5 s after TAKEOFF | twin_land_e1_creep |
| e2 mid-lift (ring 0.3 m) | twin_land_e2_lift |
| e3 QUAD4 taken out while still parked (ATTACH, LAND 0.3 s later) | twin_land_e3_parked |
| e4 QUAD4 climb | twin_land_e4_climb |
| e5 QUAD4 transit | twin_land_e5_transit |
| e6 QUAD4 descent (tip near the ring) | twin_land_e6_descend |
| e7 QUAD4 hold over the plate | twin_all_a1 |
| e9 0.5 s after the weld (reconfiguration hold) | twin_land_e9_welded |
| e9b 2 s after the weld | twin_land_e9b_welded2 |
| e9c 4 s after the weld | twin_land_e9c_welded4 |
| e11 freed carrier stepping out (1.5 s after a release) | w4_land_e11_stepout |
| e12 freed carrier landing by itself | w4_land_e12_landing |
| e13 1 s after a forced drop | w4_land_e13_drop |
| e14 LAND twice | twin_land_e14_twice |
| e15 circle, QUAD4 in its hold | twin_land_e15_orbit_hold |
| e16 carry with QUAD4 armed on the floor (R8a0) | twin_land_e16_r8a0 |

Not covered here: Tejen's phase (combined stack not built; LAND relay to his land_now); "caught but not
welded" (in the twin the catch and the weld are one event); a welded start.

## 4. Baseline

The exploratory matrix (mixed trees): e0 and e1 failed before the fixes (R1311: fleet left armed; R1303: LAND
ignored, ring lifted to 1.0 m) and pass on them (R1323, R1322); e9 tipped QUAD4 once (R1315, 80 deg, fleet
abort) and landed once (R1327); every other phase landed once (R1305, R1308, R1312-R1314, R1316-R1321,
R1324, R1328, R1329).

## 5. Pass / fail numbers

Per run, `tools/land_metrics.py` (from the first LAND to the end of run.csv) and the launch logs:

| metric | supports | falsifies |
|---|---|---|
| fleet abort / emergency stop | none | any |
| every drone disarmed at the end of the run | yes (e0: within 1 s of LAND) | any drone still armed |
| each drone's z at its disarm edge minus its rest | <= 0.06 m | > 0.10 m (dropped) |
| each drone's peak tilt from LAND to its disarm | <= 30 deg | > 45 deg |
| ring peak tilt after LAND | <= 15 deg | > 30 deg |
| LAND -> LANDED | <= 20 s | never (except e0, which has no LANDED) |

The 0.06 m disarm bar: drone 1 (/drone_0) disarms 4.4-5.1 cm above its start in every twin run since at least
R1288, before any of this code (pre-existing; reported, not scored as this card's effect).

Pre-declared reading for e9-e9c: a QUAD4 tip-over within seconds of a weld is the twin's rigid rod; on the
rig QUAD4 hangs on a tether that goes slack and the rig lands straight down (Wesley 9 Oct, real.yaml
land_unwind false). A failure there is reported as a twin-model limitation, with its rate, and does not block
the rig default, but it is not hidden: the table shows it.

## 6. Repeats

2 per phase (17 configs, 34 runs).

## 7. Cost

34 Gazebo runs on the lab PC (6 slots, ~1 h).

## 8. New code?

No new code for this card: the fixes are committed with unit tests (0400066; the switches cb09b39 and earlier).
Reviewer on the final table before the defaults change (Wesley's word on the defaults).

## Results (9 Oct, tree 7d63170; rows R1342-R1358, R1374-R1392)

| phase | runs | LAND -> LANDED (s) | worst drone disarm above rest (m) | worst drone tilt (deg) | ring tilt (deg) | verdict |
|---|---|---|---|---|---|---|
| e0 armed on the floor | R1342, R1374 | (no LANDED: fleet disarmed at LAND) | 0.000 | 0 | 0 | supports |
| e1 floor creep | R1343, R1375 | 3.45, 3.41 | 0.048 | 4.9 | 0 | supports |
| e2 mid-lift | R1344, R1376 | 6.55, 6.52 | 0.051 | 14.2 | 3.9 | supports |
| e3 QUAD4 parked, taken out | R1345, R1377 | 9.37, 9.37 | 0.048 | 14.2 | 6.3 | supports |
| e4 QUAD4 climb | R1346, R1391 (R1378 void) | 9.39, 9.50 | 0.046 | 13.8 | 2.8 | supports |
| e5 QUAD4 transit | R1379, R1392 (R1347 void) | 9.49, 9.40 | 0.047 | 14.0 | 2.9 | supports |
| e6 QUAD4 descent | R1348, R1380 | 12.40, 13.80 | 0.046 | 14.6 | 4.2 | supports |
| e7 QUAD4 hold over the plate | R1349, R1381 | 12.59, 12.30 | 0.047 | 14.3 | 3.8 | supports |
| e9 0.5 s after the weld | R1350, R1382 | 10.42, 10.44 | 0.043 | 14.8 | 4.4 | supports |
| e9b 2 s after the weld | R1351, R1383 | -, 9.56 | -, 0.071 | **85.6**, 38.2 | 2.7, 7.7 | **falsifies (R1351: QUAD4 tipped, abort); R1383 grey** |
| e9c 4 s after the weld | R1352, R1384 | 9.85, 9.55 | 0.051 | 23.3 | 4.9 | supports |
| e11 freed carrier stepping out | R1353, R1385 | 9.41, 9.46 | 0.048 | 14.3 | 6.5 | supports |
| e12 freed carrier landing by itself | R1354, R1386 | 9.30, 9.60 | 0.048 | 14.3 | 3.9 | supports |
| e13 after a forced drop | R1355, R1387 | 7.69, 7.80 | 0.006 | 8.6 | 0.7 | supports |
| e14 LAND twice | R1356, R1388 | 12.31, 12.70 | 0.047 | 13.9 | 3.9 | supports |
| e15 circle, QUAD4 hold | R1357, R1389 | 13.61, 14.30 | 0.051 | 14.0 | 3.9 | supports |
| e16 carry, QUAD4 armed on the floor | R1358, R1390 | 9.47, 9.45 | 0.051 | 13.8 | 3.7 | supports |

Voids, with evidence: R1378 (a pose timeout on drone 1 3.6 s after LAND: the runner's own run.csv has the same
0.54 s sim gap, 2.3 s wall, at that moment: every node stalled together); R1347 (the flight landed and disarmed,
then the runner hung 2 h without writing run.csv; lab_batch now caps a run at 30 min wall, 097c524).

The post-weld family over all runs today: LAND 0.5 s after the weld tipped QUAD4 1 of 4 (R1315; R1327, R1350,
R1382 landed), 2 s after 1 of 4 (R1351; R1328 landed, R1383 grey at 38 deg), 4 s after 0 of 3 (R1329, R1352,
R1384; worst 23 deg). Pre-declared reading: the twin's rigid link (Wesley 9 Oct: the twin keeps rigid links); on
the rig QUAD4's tether goes slack and the rig lands straight down.

**Verdict:** LAND lands and disarms every drone in every phase the twin can fly (16 of 17 phases 2 of 2), except
a LAND within ~2 s of a weld, where the twin's rigid link tips QUAD4 about 1 time in 4 (twin artefact by the
pre-declared reading). The drone 1 (/drone_0) disarm ~5 cm above its start is the pre-existing fleet-disarm drop.

## Reviewer (9 Oct): NOT SUPPORTED (corrections taken; the table is re-flown on the fix)

1. **R1378 was not a host stall.** The runner's gap (0.54 s sim in 2.30 s wall) is the normal sim speed; tracker_0
   and tracker_1 logged inside it; the parallel slots R1379/R1380 had no gap. It starts 0.2 s after QUAD4 was
   disarmed alone at its rest: its tracker shut down and its process died, then drones 1 and 3 timed out on pose
   (0.54 / 0.53 s) and the fleet got an EMERGENCY STOP at 0.83-0.93 m. The same gap appears only where tracker
   processes exit (at the fleet disarm in R1318, R1348, R1352, R1376, R1390; R1347's runner hang starts there too).
   Mid-flight disarm-alones: 1 abort in 9 in the claim runs, 1 in ~20 over the day (gap after each disarm-alone:
   0.02-0.05 s sim in 20 runs, 0.54 s in R1378). **Reclassified: R1378 FAILS (e4 1 of 3 on this tree).**
   Fix f330bd2: a tracker disarmed alone keeps its process up (one-shot); the fleet disarm after LANDED closes it.
2. e12 never had a freed drone mid-landing at LAND: /drone_1 was already down and disarmed alone (R1354 38.08 vs
   LAND 38.57; R1386 38.01 vs 38.48; R1320). Retimed: LAND 1.5 s after "lands now".
3. e11-e13 are detach configs: no newcomer, so newcomer_lifecycle / newcomer_fault_land read back false there
   (departed_ground_idle on); "the switches on" holds for 28 of 34 runs. No row injects a fault on an unwelded
   QUAD4: newcomer_fault_land's active path is shown by card 2026-10-09_newcomer_fault_scope, not here.
   R1391/R1392 manifests say e816ab2 (flight code identical; same git_diff.patch md5).
4. Every cell reproduces; the post-weld text was miscounted: **2 s after the weld: 3 runs, not 4** (R1328 clean
   6.5 deg, R1351 tipped 85.6 deg, R1383 grey 38.2 deg / 0.071 m): 1 of 3 tipped, 1 of 3 inside the bars; on the
   claim tree 0 of 2 inside the bars. 0.5 s: 1 of 4 tipped (R1315, on cb09b39). 4 s: 0 of 3 tipped (R1384 23.3 deg,
   0.051 m). Drone 1's 4-5 cm disarm height is pre-existing (R1290, switches off: 0.049); /drone_1 also disarms
   3.0-3.3 cm up in 13 of 33 scored runs.
5. Not covered: Tejen's phase, caught-not-welded, a welded start (newcomer_fault_land is forced off there), a fault
   on an unwelded QUAD4; **the rig's LAND path** (every twin run has land_unwind true, real.yaml false; R1351's tip
   began after "load down - survivors unwind"; a straight-down LAND with a welded QUAD4 is not flown in any row);
   the rig's slack tether vs the twin's rigid rod; whether a tracker exit mid-flight also stalls the rig's graph
   (the fix removes the exit there too).

Re-fly: the whole table on f330bd2 (one tree), e12 retimed; the disarm-alone card's rows with it (their
SUPPORTED verdict predates the R1378 finding: P2 "no abort caused by the expected disarm" was met in its own 10
runs, but R1378 is exactly that failure).


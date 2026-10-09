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

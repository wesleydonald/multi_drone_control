# Pickup seat: what separates a held lift from a failed one (every rig pickup on record, 14 Oct - 13 Nov)

Exploratory analysis, 13 Nov 2026. No thresholds were set before looking. Scripts and data are in this folder.

## Question
fm9c_2 (13 Nov) suggested that a magnet seated within ~1 mm of the calibrated contact height holds and one 2+ mm above
does not. Does this hold for every real pickup, and does anything else separate held from failed lifts?

## Data
27 rig flights whose log has `DESCEND_TO_PICKUP` (14 Oct t5_4/t5_5/t6, 29 Oct k1_8, 4 Nov k1m*, 5 Nov k1d/fm9y_2/k1o_2/
k1ob_2, 6 Nov q4p, 11 Nov fm9c/fm9c_3/fm9c_4/fm9c_6/k1zo/k1zo_3, 12 Nov fm9_2/fm9_5/fm9c_3/fm9c_5/q4d_3/q4d_4/q4d_7,
13 Nov fm9c_2). Every bag is present and has the tip (body 24), the object (body 6), QUAD4 and his reference. The
one-quad bags (q4p, q4d_*) have no `/drone_3/magnet_coil`: his `/magnet/command` stands in for the coil. Bags record
the mocap at ~90 Hz (combined launch) or ~240 Hz (one-quad launch).

That gives 53 descents: **27 lifts** (17 held, one of them lost 7.5 s into the lift; 10 not held) and 26 descents that
never lifted (his gates stopped them: tip off target, no progress, attach latch, take-back). Only the lifts test the
grip.

Objects and contact heights (tip marker over object marker with the magnet seated flat, props off):

| object | flights | contact height | source |
|---|---|---|---|
| puck (his commissioning puck, 86 mm) | 14 Oct | 37.9 mm | RIG-1014-puck (the planner used 34.5) |
| object of 29 Oct (not recorded) | k1_8 | 34.5 mm (his default, not measured) | no lifts, no grip test |
| big object, 100 x 100 x 10 mm, 0.15 kg | 4, 5, 11 Nov | 18.0 mm | typed, measured 21 Oct with QUAD1's marker 21 |
| small object, bare (54 mm steel disc in a 70 mm holder, 60 g) | 5, 6, 11 Nov | 38.0 mm | typed (re-checked 37.6 mm on 11 Nov) |
| small object with the supervisor's funnel | 12, 13 Nov | 11.0 mm | typed (11.4 / 11.0 mm on 11 Nov, 10.7 mm on 12 Nov) |

## Measures (per descent; `analyze.py`)
- **seat gap**: tip z - object z - contact height, median over the 0.5 s before `LIFT_OBJECT` (for a descent without a
  lift: the 0.5 s around its lowest point). 0 = seated as calibrated.
- **tilt**: angle between the magnet face normal and the object's up axis in the same window. The face normal in body
  24's frame is calibrated from the samples where the magnet carries the object (it then hangs flat on the steel): body
  24 of 4 Nov morning (1385 samples, spread 1.2 deg) and body 24 as redefined on 4 Nov ~17:25 (15498 samples, spread
  1.9 deg); both normals lie within 1.5 deg of body 24's z axis. The 14 Oct (and 29 Oct) body 24 has no carried sample:
  its raw z axis is used (marked *). **free-hang tilt**: the same angle in the 2 s before the descent.
- **xy**: tip to the typed contact point (object marker + typed `pickup_point_x/_y` in the object's axes).
- **tip speed**: horizontal tip speed in the seat window. **slack**: free-hang body-to-tip height - body-to-tip
  height at the seat (> 0: the cable slackened). **body over ref**: QUAD4 body z - his reference z at the seat.
  **ref depth min**: lowest reference tip (reference z - free-hang body-to-tip height) relative to the seated height.
- **dwell**: unbroken time within 5 mm of the seat before the lift. **coil ON**: time from the last coil ON to the lift.
- **held**: the object rose >= 3 cm with the tip in the lift (it rose 25-141 cm in every held lift and at most 0.3 cm
  in every failed one, so the 3 cm line makes no difference).

## Every lift (27)
| flight | try | object | seat gap mm | tilt deg | free-hang tilt deg | xy mm | tip speed mm/s | slack mm | body over ref mm | ref depth min mm | dwell <5 mm s | coil ON s | object rise cm | outcome |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 10-14/t5_4 | 1 | puck | 6.51 | 7.1* | 6.2 | 19.8 | 105.8 | 1.3 | 31 | -31 | 0.00 | 6.1 | -0.0 | not held |
| 10-14/t5_5 | 2 | puck | 21.41 | 9.0* | 8.0 | 25.9 | 70.8 | -0.2 | 45 | -24 | 0.00 | 5.0 | 0.1 | not held |
| 10-14/t5_5 | 3 | puck | -1.19 | 8.4* | 6.8 | 45.5 | 15.7 | 0.4 | 10 | -12 | 0.46 | 3.7 | 0.3 | not held |
| 11-04/k1m | 1 | big | -0.46 | 1.2 | 15.3 | 25.1 | 1.2 | 0.3 | 9 | -10 | 1.55 | 4.1 | 129.3 | held |
| 11-04/k1m_2 | 2 | big | -0.35 | 1.2 | 11.7 | 29.2 | 0.7 | 0.8 | 16 | -18 | 1.55 | 4.3 | 28.4 | held |
| 11-04/k1m_3 | 2 | big | 0.22 | 1.0 | 10.6 | 34.1 | 1.0 | 2.1 | 17 | -20 | 1.56 | 5.0 | 25.0 | held |
| 11-04/k1m_4 | 1 | big | 0.22 | 1.0 | 14.4 | 33.2 | 0.4 | 0.1 | 5 | -6 | 1.54 | 4.5 | 25.3 | held |
| 11-05/k1d | 1 | bare small | -1.98 | 5.6 | 9.7 | 17.1 | 2.6 | 1.6 | 38 | -45 | 1.54 | 4.6 | 137.4 | held |
| 11-05/fm9y_2 | 3 | bare small | 0.38 | 2.4 | 16.6 | 15.9 | 2.0 | 0.9 | 17 | -31 | 1.56 | 5.8 | 139.5 | held |
| 11-05/k1o_2 | 2 | bare small | 1.56 | 8.3 | 13.4 | 21.0 | 2.0 | 1.0 | 29 | -30 | 1.58 | 4.2 | 0.2 | not held |
| 11-05/k1ob_2 | 1 | big | 0.21 | 1.5 | 13.3 | 36.0 | 1.1 | 7.7 | 32 | -44 | 1.54 | 8.9 | 137.7 | held |
| 11-06/q4p | 1 | bare small | 0.39 | 2.5 | 7.4 | 8.2 | 3.1 | 2.8 | 26 | -29 | 1.61 | 4.1 | 31.0 | held |
| 11-11/fm9c_4 | 3 | big | -0.23 | 0.5 | 7.9 | 28.3 | 0.9 | 1.3 | 12 | -14 | 1.58 | 3.7 | 137.6 | held |
| 11-11/fm9c_6 | 1 | big | -0.29 | 0.9 | 5.8 | 23.9 | 0.6 | 0.9 | 32 | -33 | 1.53 | 4.2 | 48.4 | held, fell later at +7.5 s |
| 11-11/k1zo | 1 | bare small | -0.70 | 1.6 | 4.9 | 14.9 | 0.9 | -0.1 | 16 | -18 | 1.56 | 3.8 | 141.3 | held |
| 11-11/k1zo_3 | 1 | bare small | -0.20 | 0.9 | 3.7 | 11.8 | 7.9 | 1.9 | 23 | -26 | 1.57 | 4.1 | 138.9 | held |
| 11-12/fm9_2 | 1 | funnel | 0.04 | 6.8 | 8.2 | 17.5 | 6.0 | 1.3 | 37 | -38 | 1.58 | 7.1 | 0.0 | not held |
| 11-12/fm9_5 | 3 | funnel | -0.25 | 1.2 | 8.9 | 12.0 | 0.4 | 1.9 | 40 | -42 | 1.51 | 8.6 | 140.1 | held |
| 11-12/fm9c_5 | 1 | funnel | 0.14 | 5.3 | 7.5 | 14.4 | 0.8 | 0.3 | 41 | -41 | 1.91 | 7.7 | 0.0 | not held |
| 11-12/fm9c_5 | 2 | funnel | 1.47 | 8.6 | 8.1 | 15.5 | 12.1 | 1.8 | 36 | -38 | 2.22 | 7.3 | 0.1 | not held |
| 11-12/q4d_3 | 1 | funnel | -1.16 | 1.0 | 16.5 | 7.3 | 1.5 | 0.4 | 40 | -41 | 2.29 | 4.4 | 31.2 | held |
| 11-12/q4d_4 | 1 | funnel | -0.81 | 1.9 | 17.0 | 12.5 | 2.7 | -0.0 | 39 | -40 | 2.05 | 4.4 | 30.7 | held |
| 11-12/q4d_7 | 1 | funnel | -0.57 | 10.6 | 15.3 | 25.2 | 17.0 | 0.3 | 41 | -42 | 2.21 | 4.5 | 0.0 | not held |
| 11-12/q4d_7 | 2 | funnel | -0.86 | 2.3 | 16.6 | 10.7 | 4.0 | 2.3 | 37 | -41 | 2.05 | 4.4 | 30.2 | held |
| 11-13/fm9c_2 | 1 | funnel | 1.86 | 10.5 | 15.3 | 13.4 | 8.9 | -0.3 | 24 | -22 | 1.93 | 7.7 | 0.0 | not held |
| 11-13/fm9c_2 | 2 | funnel | 2.28 | 10.8 | 14.7 | 12.8 | 15.1 | 0.4 | 37 | -36 | 1.52 | 11.7 | 0.0 | not held |
| 11-13/fm9c_2 | 3 | funnel | 0.33 | 2.0 | 14.5 | 6.1 | 1.5 | 0.5 | 36 | -36 | 1.79 | 9.4 | 65.4 | held |

\* tilt not calibrated (14 Oct body 24: raw z axis). fm9c_6's lift lost the object 7.5 s in, after hanging 7.8 deg askew
(every other carried object hung within 0.8-3.2 deg).

## The 26 descents without a lift (no grip test)

| flight | try | object | lowest gap mm | tilt there deg | xy there mm | his messages |
|---|---|---|---|---|---|---|
| 10-14/t5_4 | 2 | puck | 56.8 | 6.4* | 70 |  |
| 10-14/t5_5 | 1 | puck | 71.5 | 7.5* | 27 | HIGH no measured clearance/descent progress for 10s |
| 10-14/t6 | 1 | puck | -21.3 | 9.9* | 74 | object attached but the tip 0.064 m off target |
| 10-29/k1_8 | 1 | 29 Oct obj | -22.1 | 13.4* | 106 | ASSUMED ATTACHMENT LOST, then contact wait made no progress for 10s |
| 10-29/k1_8 | 2 | 29 Oct obj | -23.9 | 20.0* | 137 | ASSUMED ATTACHMENT LOST |
| 11-04/k1m_2 | 1 | big | -10.2 | 17.8 | 181 | ASSUMED ATTACHMENT LOST, then contact wait made no progress for 10s |
| 11-04/k1m_3 | 1 | big | 0.3 | 0.3 | 44 | object attached but the tip 0.043 m off target |
| 11-05/fm9y_2 | 1 | bare small | -8.7 | 18.2 | 78 | ASSUMED ATTACHMENT LOST, then contact wait made no progress for 10s |
| 11-05/fm9y_2 | 2 | bare small | -16.5 | 11.3 | 65 | object attached but the tip 0.051 m off target |
| 11-05/k1o_2 | 1 | bare small | -21.6 | 8.2 | 72 | object attached but the tip 0.060 m off target |
| 11-05/k1o_2 | 3 | bare small | -18.9 | 11.3 | 74 | object attached but the tip 0.075 m off target |
| 11-05/k1o_2 | 4 | bare small | -14.6 | 15.6 | 103 |  |
| 11-11/fm9c | 1 | bare small | -20.0 | 9.1 | 75 | ASSUMED ATTACHMENT LOST, then contact wait made no progress for 10s |
| 11-11/fm9c | 2 | bare small | -19.4 | 8.3 | 76 |  |
| 11-11/fm9c_3 | 1 | bare small | -17.4 | 12.6 | 74 | ASSUMED ATTACHMENT LOST, then contact wait made no progress for 10s |
| 11-11/fm9c_3 | 2 | bare small | -17.7 | 9.5 | 76 | object attached but the tip 0.085 m off target |
| 11-11/fm9c_3 | 3 | bare small | -19.7 | 14.1 | 145 | ASSUMED ATTACHMENT LOST |
| 11-11/fm9c_4 | 1 | big | -6.2 | 6.8 | 92 | ASSUMED ATTACHMENT LOST, then contact wait made no progress for 10s |
| 11-11/fm9c_4 | 2 | big | -11.3 | 1.5 | 87 | object attached but the tip 0.084 m off target |
| 11-12/fm9_2 | 2 | funnel | 3.7 | 7.7 | 74 | contact wait made no progress for 10s |
| 11-12/fm9_2 | 3 | funnel | 221.3 | 8.3 | 18 |  |
| 11-12/fm9_5 | 1 | funnel | 24.5 | 0.9 | 10 | contact wait made no progress for 10s |
| 11-12/fm9_5 | 2 | funnel | 4.4 | 6.9 | 71 | contact wait made no progress for 10s |
| 11-12/fm9c_3 | 1 | funnel | -0.4 | 1.0 | 8 | ASSUMED ATTACHMENT LOST, then contact wait made no progress for 10s |
| 11-12/fm9c_3 | 2 | funnel | -0.6 | 0.5 | 13 | ASSUMED ATTACHMENT LOST, then contact wait made no progress for 10s |
| 11-12/fm9c_5 | 3 | funnel | -19.3 | 9.8 | 63 |  |

"Lowest gap" there is the 0.5 s around the descent's lowest point. A gap near -20 mm with the tip 6-15 cm off is the
magnet lying on the floor beside the object. fm9_5 try 1 reads +24.5 mm because body 6 was redefined 25.0 mm lower
during it (its rest height 34.1 mm in tries 1-2, 59.1 mm in try 3): corrected, the magnet sat flat at about -0.5 mm
(tilt 0.9 deg) and was never lifted. Flat seats that were never lifted: fm9_5 try 1, 12 Nov fm9c_3 tries 1-2 (his attach
latch), k1m_3 try 1 (refused 4.3 cm off target).

## Verdict
**Seat depth alone does not separate held from failed lifts. The magnet's tilt on the object does: 26 of 27.**

| predictor (27 lifts) | held (17) | not held (10) | AUC (failure higher) |
|---|---|---|---|
| seat gap (mm) | -1.98 .. +0.39, median -0.25 | -1.19 .. +21.4, median +1.52 | 0.76 |
| tilt on the object (deg) | 0.5 .. 2.5, and k1d 5.6 | 5.3 .. 10.8 | 0.99 |
| tip speed in the seat (mm/s) | 0.4 .. 7.9, median 1.3 | 0.8 .. 106, median 13.6 | 0.89 |
| xy to the contact point (mm) | 6.1 .. 36.0 | 12.8 .. 45.5 | 0.59 |
| dwell within 5 mm (s) | 1.51 .. 2.29 | 1.52 .. 2.22 (0 where it never got within 5 mm) | 0.49 |
| coil ON before the lift (s) | 3.7 .. 9.4 | 3.7 .. 11.7 | 0.67 |
| reference depth (mm) | -45 .. -6 | -42 .. -12 | 0.46 |
| slack (mm) | -0.1 .. 7.7 | -0.4 .. 1.8 | 0.33 |
| free-hang tilt (deg) | 3.7 .. 17.0 | 6.2 .. 15.4 | 0.43 |

1. **The gap hypothesis holds only at its top end.** No lift held with the seat more than +0.4 mm above the calibrated
   height, and all six lifts at +1.4 mm or more failed (fm9c_2 1 and 2, fm9c_5 2, k1o_2 2, t5_4 1, t5_5 2). But four
   failures sat at or below the calibrated height (fm9c_5 1 +0.14, fm9_2 1 +0.04, q4d_7 1 -0.57, t5_5 3 -1.19 mm),
   inside the held range. On the funnel object alone: held -1.16 .. +0.33 mm, not held -0.57 .. +2.28 mm.
2. **Tilt separates.** Held magnets lay flat on the steel (0.5-2.5 deg, 16 of 17). Every failed magnet was tilted
   5.3-10.8 deg: it kept 60-120 % of its free-hang tilt (the magnet hangs 4-17 deg off its cable's line), so it touched
   with an edge and never turned flat. Within one flight the contrast is stark: fm9c_2 tries 1-2 at 10.5 / 10.8 deg,
   try 3 at 2.0 deg (it went from 15.8 to 1.9 deg in 0.5 s as it seated); q4d_7 try 1 at 10.6 deg, try 2 at 2.3 deg.
   The one overlap: k1d held at 5.6 deg (the bare object's steel disc can move in its holder: its carried seat read
   3.2 mm higher than its resting seat, so the holder's markers may not show the disc's tilt) against fm9c_5 try 1
   failed at 5.3 deg. Any line between 2.5 and 5.3 deg classifies 26 of 27.
3. **The seat gap is a symptom of the tilt, and a noisy one.** A 20 mm face resting on its edge at 10.5 deg lifts its
   centre by 10 x sin(10.5 deg) = 1.8 mm: fm9c_2's failures sat 1.5-2.0 mm above its held seat. But where the edge
   lands changes that: q4d_7 try 1 was tilted 10.6 deg only 0.3 mm above q4d_7 try 2's flat seat.
4. **On the funnel object, flat seats came only near the seat point.** All 8 flat funnel seats (5 held, 3 never lifted)
   had the tip 6.1-12.7 mm from the typed contact point, the 6 tilted seats of the failed lifts 12.8-25.2 mm. That suggests the funnel holds
   the magnet tilted when it lands more than ~1.3 cm off. It is a lead, not a threshold: 14 seats, a 0.1 mm margin,
   and the typed point itself moved 4.6 mm between 11 and 12 Nov (two props-off seats 8 mm apart on 11 Nov). On the flat
   big object every lift held, 24-36 mm off.
5. **Not predictive:** dwell at the bottom (1.5-2.3 s in every lift that reached the seat), coil ON time (ON before touchdown in every lift that
   reached the object), reference depth, free-hang tilt, descent time. The faster tip in failed seats (median 13.6 vs
   1.3 mm/s) is a consequence: a magnet balanced on an edge rocks with the swing, a flat one is held still.
6. **The overtravel does not reach the magnet.** At the seat QUAD4's body was 5-45 mm above his reference in every
   lift, held or not (0.2-5 cm above it already in mid-descent). The cable stayed within -0.4 .. +2.8 mm of its
   free-hang length in 26 of 27 lifts (k1ob_2: 7.7 mm). A deeper reference (2.3-3.6 cm in fm9c_2) did not
   slacken the cable. The one descent with real slack, fm9_5 try 1 (31 mm, while the planner believed the object 25 mm
   lower), lay flat at 0.9 deg. A guess, not tested: with the magnet's weight off the cable QUAD4 is lighter than its
   model and floats.

## Mocap resolution and calibration
- Noise while seated flat: the gap varies 0.03-0.34 mm (sd, median 0.10 mm) over the 0.5 s window, the object at rest
  0.07 mm (median sd), the tip's tilt 0.03-0.6 deg. The per-try medians resolve 0.1 mm and 0.5 deg. The edge-seated
  failures rock: tilt sd 1.2-1.9 deg in fm9c_2 1-2 and q4d_7 1.
- Calibration: with the object hanging from the magnet (the tip in the object's frame, turned back to its resting
  attitude), the in-flight seat agrees with the typed contact height within -0.49 .. +0.45 mm on the big object,
  -0.44 .. +0.42 mm on the funnel object, -0.44 .. +1.21 mm on the bare small object. The props-off seats of the funnel
  object span 10.7-11.4 mm. A flat seat resting on the floor reads up to ~1 mm lower than the hanging one (q4d_3 -1.16
  vs -0.12, k1d -1.98 vs +1.21). So any absolute gap carries about +-1 mm, the size of the effect it is meant to show.
- Body 6 moved between and within flights: its resting height on 12 Nov was 33-60 mm (floor, mat, redefinitions), and
  fm9_5's tries 1-2 ran on a pivot 25.0 mm low. Gaps are only comparable within a flight unless a carried sample checks
  the pivot (it did in every held flight).

## Caveats
- 27 lifts, 10 failures. Three failures are 14 Oct (puck, 0.55 m cable, old body 24 with an uncalibrated tilt: there
  the magnet never reached the puck in two, t5_4 1 at +6.5 mm and t5_5 2 at +21.4 mm, and sat at its edge, 45 mm off an
  86 mm puck, in the third). k1o_2 flew a body 6 redefined off-centre (its xy is not usable, its tilt is). That leaves
  6 failures on calibrated geometry, all on 12-13 Nov with the funnel object.
- fm9_2 (12 Nov) flew before the loose magnet crimp was found (09:24). Its failure could be electrical. It was tilted
  6.8 deg anyway.
- Correlation only: no flight controlled the tilt.
- Registry RIG-1111-fm9c_3 says its three descents reached the object and "the object did not rise with the magnet".
  The bag shows no lift in any of them: the magnet lay on the floor 7-15 cm beside the object (gap about -18 mm, his
  "tip 0.085 m off target" and "attachment lost"). A correction row is the lab chat's call. Not edited here.

## What it suggests (not built)
A pre-lift check on the measured tilt (magnet face within ~3 deg of the object's up axis, the mocap resolves it to
~0.5 deg) would have refused all 10 failed lifts and passed 16 of 17 held ones, in place of a height test that cannot
separate them. It is a gate threshold in his planner, so Wesley's word and Te-Jen's first. Making the magnet land flat
(real slack on the cable, a seat that does not hold it on the funnel wall) is the fix the data points at.

## Files
- `extract.py`: pulls the pickup window of each bag to `extract/<day>_<flight>/*.csv.gz` (tip, object, QUAD4, his
  reference, coil, attach flags).
- `analyze.py`: one row per descent -> `tries.csv`, the face-normal calibration -> `face_normal.json`.
- `plot.py`: `seat_gap_vs_outcome.png` ((a) seat gap by object, held or not; (b) seat gap against tilt).
- Rerun: `source ~/ros2_humble/install/setup.bash && source install/setup.bash`, then `python3 extract.py`,
  `python3 analyze.py`, `python3 plot.py` in this folder.

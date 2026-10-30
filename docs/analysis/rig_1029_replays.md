# Rig 29 Oct replays: formation thrust, Tejen's descent and pickup gates, the ring's motion

Offline, from the logs and bags of 29 Oct (`results/rig/2026-10-29`, registry rows RIG-1029-*), for the Fri 30 Oct sheet.
Written Thu 29 Oct 19:10. Scripts and JSON: `results/analysis/rig_1029_replays/` (`w1_formation_thrust.py`,
`w2a_descent_abort.py`, `w2b_pickup_capture.py`, `w2c_ring_motion.py`, `freeze_scan.py`). Exploratory analysis, no claim.

## Rig finding first: the magnet-tip marker freezes

The magnet marker (body 24, `/magnet_tip_pose`) repeats one value for about 0.21 s at a time while its stamps keep
advancing, then jumps. The ring and QUAD4 bodies do not. Three of the six sq5_2 aborts fired on the first new sample
after such a hold.

| Flight | Tip samples repeated | Holds | Median hold | Longest hold | Jump after a hold (median, max) | Ring / QUAD4 body repeated |
|---|---|---|---|---|---|---|
| sq5_2 | 13.9 % | 141 | 0.21 s | 0.34 s | 1.4 cm, 15.9 cm | 0.4 % / 0 % |
| sq2_5 | 17.5 % | 203 | 0.21 s | 2.27 s | 2.0 cm, 158 cm | 0 % / 0 % |
| k1_4 | 2.8 % | 27 | 0.22 s | 0.23 s | 2.2 cm, 6.0 cm | 0.3 % / 0 % |

**Verdict:** a debounce timed on tip stamps (his new `contact_position_lost`) counts a frozen marker as fresh. Check
Motive's marker 24 (occlusion under QUAD4, a one-marker body) before tuning his gates on it.

## W1. Formation thrust

Predicted hover throttle per carrier, calibrated (statics at the planner's 60 deg, the affine map, the per-quad and
gain correction below). 22.0 V is where the loaded pair of the T sat from the first second on fresh packs (sq1).

| Layout | Tension per plate, 1.15 kg (N) | 1.15 kg at 23.5 V | 1.15 kg at 22.0 V | 0.86 kg at 23.5 V | 0.86 kg at 22.0 V |
|---|---|---|---|---|---|
| square 0/3/6/9 | 3.26 each | 0.62 | 0.65 | 0.57 | 0.60 |
| four 3/7/9/11 (after the attach) | p3 5.07, p7/p11 2.89, p9 2.17 | p3 0.73, p7/p11 0.60, p9 0.56 | p3 0.76 | p3 0.65 | p3 0.68 |
| triangle 3/11/7 | 4.34 each | 0.69 | 0.72 | 0.62 | 0.65 |
| T 0/3/6 | p0/p6 6.51, p3 0.1 | p0/p6 0.82, p3 0.49 (model) | p0/p6 0.85 | p0/p6 0.72 | p0/p6 0.75 |
| pair 0/6 | 6.51 each | 0.82 | 0.85 | 0.72 | 0.75 |

Check against what flew: median `thr_out` in the first 4 s of each steady hover, range over the carriers.

| Flight | Layout, ring | Pack V | Measured | Model | Calibrated | Later in the same hover |
|---|---|---|---|---|---|---|
| h4f_3 | square, 1.15 | 23.1-23.5 | 0.61-0.64 | 0.64 | 0.61-0.64 | 5 s window only |
| f4 | square, 1.15 | 22.6-23.0 | 0.61-0.67 | 0.65 | 0.63-0.65 | 5 s window only |
| ch1b | square, 1.15 | 23.5-23.8 | 0.59-0.63 | 0.63 | 0.60-0.63 | 0.59-0.65 at 8 s |
| ch1b_2 | square, 1.15 | 22.9-23.4 | 0.61-0.66 | 0.64-0.65 | 0.62-0.64 | 0.61-0.68 at 12 s |
| sq2_5 | triangle, 1.15 | 23.2-23.7 | 0.69-0.70 | 0.68-0.69 | 0.68-0.70 | 0.73-0.76 at 40 s (22.3-22.5 V) |
| sq5_2 | triangle, 1.15 | 23.4-24.0 | 0.67-0.69 | 0.68-0.69 | 0.68-0.70 | 0.67-0.69 at 12 s |
| sq3 | triangle, 1.15 | 23.7-24.0 | 0.65-0.69 | 0.68 | 0.67-0.69 | 0.68-0.71 at 32 s |
| sq4_5 | triangle, 1.15 | 22.5-22.8 | 0.71-0.73 | 0.70-0.71 | 0.70-0.72 | 0.73-0.75 at 28 s |
| k1_4 | triangle, 1.15 | 22.2-22.4 | 0.69-0.73 | 0.71-0.72 | 0.70-0.72 | 0.71-0.76 at 20 s |
| sq1 | T, 1.15 (out of sample) | p0 22.0, p6 22.2 | p0 0.868, p6 0.829, p3 0.506 | 0.829, 0.824, 0.500 | 0.859, 0.835 | dropped at 2.7 s |
| ch1b_2 | T after the release, sinking, tilt 24 deg | 22.7-22.9 | p0 0.781, p6 0.779 | 0.809, 0.813 | not steady | dropped |
| hS1 (28 Oct) | pair 11/5, 0.86 on 0.55 m (out of sample) | 23.0-23.1 | 0.727-0.730 | 0.706-0.709 | 0.698-0.714 | |
| hS2 (28 Oct) | pair 11/5, 0.86 on 0.55 m | 22.3-22.7 | 0.745-0.782 | 0.715-0.724 | 0.704-0.731 | |

| Calibration (first bins of 4 square and 5 triangle hovers) | Value |
|---|---|
| bias QUAD1 / QUAD2 / QUAD3 / QUAD4 | -0.005 / -0.006 / -0.024 / -0.022 |
| gain correction per kg of thrust above the square's 0.886 kg | +0.11 |
| rms on the calibration set, model then calibrated | 0.019, then 0.012 |
| hover fade, triangle hovers of 20-40 s (median, range) | +0.066 per min (-0.035 to +0.088) |
| the map's voltage term over the same hovers | +0.013 to +0.040 per min |
| control effort, p95 minus median in a bin (square, triangle, T) | 0.010, 0.025, 0.014 |

| Margin under throttle_max 0.95, worst carrier (start at 23.5 V, the T and the pair at 22.0 V) | 1.15 kg, start | 1.15 kg, after 90 s | 0.86 kg, start | 0.86 kg, after 90 s |
|---|---|---|---|---|
| 4: square | 0.32 | 0.22 | 0.37 | 0.27 |
| 4: after the attach (plate 3) | 0.21 | 0.11 | 0.29 | 0.19 |
| 3: triangle | 0.24 | 0.14 | 0.31 | 0.21 |
| 3: T (chain stage 1) | 0.09 | none (-0.01) | 0.19 | 0.09 |
| 2: pair (chain stage 2) | 0.09 | none (-0.01) | 0.19 | 0.09 |

**Verdict:** with the 1.15 kg ring the square, the triangle and the four after the attach fit with at least 0.11 left
after 90 s, while the T and the pair start at 0.83-0.87 and reach the cap in about 90 s of hover (sq1's QUAD1 already
had a p95 of 0.89). With the 0.86 kg ring every baseline and chain stage fits at the start (0.19 or more), but the T and
the pair keep only 0.09 after 90 s, and less if the 28 Oct under-prediction of 0.02-0.05 holds.

The prediction misses by more than 0.03 only late in long hovers (the pack fades about twice as fast as the map's
voltage term), on QUAD3 and QUAD4 (they fly 0.02 below the model throughout), and on the pairs of 28 Oct (measured 0.02-0.05
above the calibrated value). The cable angle is
not a cause: measured 59-62 deg on every carrier. The ring rides 1-4 cm low and the planner's height integral grows to
-1 to -2 N in the long hovers while the throttle climbs.

## W2a. sq5_2's six descents

His abort (`tracking_errors()`, contact mode): xy of the measured tip minus the ring bridge's plate point (ring pose
through its full attitude, sampled at 50 Hz), or QUAD4's own xy speed (the twist is zero on the rig), each 30 Hz tick.
His trim moves the reference, not this target. Rebuilt from the bag, it matches his 5 Hz CSV to 0.07 cm median (0.55 cm
p95) and his six abort ticks.

| Descent | Start after his TAKEOFF (s) | Flown (s) | Abort tick (s) | Error at abort (cm) | Speed at abort (m/s) | Peak before (cm) | After a marker hold | Excursion past 8 cm in the record (s) | 0.30 s debounce fires at (s) |
|---|---|---|---|---|---|---|---|---|---|
| 1 | 16.6 | 0.60 | 0.61 | 9.4 | 0.10 | 6.3 | 0.22 s hold, 4.9 cm jump | 1.13 | 0.94 |
| 2 | 18.8 | 1.77 | 1.72 | 8.0 | 0.15 | 7.5 | no | 1.62 | 2.03 |
| 3 | 23.7 | 1.93 | 1.96 | 8.3 | 0.09 | 7.8 | no | 1.68 | 2.27 |
| 4 | 33.5 | 0.34 | 0.35 | 8.9 | 0.02 | 4.1 | 0.21 s hold, 5.0 cm jump | 0.43 | 0.65 |
| 5 | 34.9 | 2.74 | 2.72 | 9.1 | 0.04 | 7.2 | 0.22 s hold, 2.7 cm jump | 0.86 | 3.04 |
| 6 | 39.0 | 0.04 | 0.02 | 8.4 | 0.03 | none (first tick) | no | 0.94 | 0.32 |

| His descent on the rig | Value |
|---|---|
| start, tip above the plate point (`approach_offset_z`, typed) | 0.30 m |
| end (`contact_dz`, rig) | -0.01 m |
| speed (`descend_rate`, his default) | 0.08 m/s |
| full descent (his CSV's `rate_profile_duration_s`) | 3.875 s (about 3.75 s to the plate) |

| A full 3.875 s descent survives, from any tick between his first MATCH and TAKEN_BACK | debounce 0 | 0.3 s | 0.5 s | 1.0 s |
|---|---|---|---|---|
| abort at 8 cm | 0 % | 2 % | 12 % | 49 % |
| abort at 10 cm | 1 % | 28 % | 32 % | 85 % |
| abort at 12 cm | 24 % | 30 % | 46 % | 85 % |

**Verdict:** every abort was the 8 cm position term on its first tick, and a 0.30 s debounce would only have delayed each
one by 0.3-0.35 s: every excursion lasted 0.43-1.68 s. The descent needs a wider threshold with a longer debounce (10 cm
and 1 s survive 85 % of starts), a calmer ring, or both. MATCH at 8 cm equals the abort at 8 cm, so descent 6 began at
8.4 cm and left on its first tick. The plate itself moved 5.1 cm rms over these 25 s, as much as the error.

The record after each real abort is his re-approach (QUAD4 climbing 0.3 m to the same xy), not a continued descent. The
data cannot show what QUAD4 would have done had it kept descending, nor whether a contact a few cm off would have caught.

## W2b. k1_4's pickup hover (SETTLE_ABOVE_PICKUP, 26.3 s)

His predicate (`update_pickup_tier_alignment` into `TipAlignment.step`, phase capture), all at once on each planner
tick, unbroken for the dwell, any failing tick resets it: xy of the magnet face (tip marker, 2.2 cm down) to the object
point within `capture_xy`, the 3-D speed of face minus object within the speed bound (his 0.35/0.65 filter, updated only
when the tip stamp advances), and the face at least 8 cm above the object (0.25-0.33 m here, always true). Rebuilt per
tick, it matches his CSV's xy to 0.12 cm median and his logged waiting reason on 93 % of rows.

| Setting | First closes (s into SETTLE) | Ticks with all true | xy true | speed true | Unbroken runs of 0.5 s or more | Longest run (s) |
|---|---|---|---|---|---|---|
| as flown: 0.025 m, 0.07 m/s, 0.5 s | never | 2 % | 17 % | 29 % | 0 | 0.23 |
| typed next: 0.06 m, 0.15 m/s, 0.5 s | 0.56 | 57 % | 74 % | 74 % | 9 | 3.53 |
| 0.06 m only (0.07 m/s) | 12.4 | 21 % | 74 % | 29 % | 1 | 1.20 |
| 0.15 m/s only (0.025 m) | 7.2 | 13 % | 17 % | 74 % | 2 | 0.74 |

| Hover statistics | Value |
|---|---|
| xy, median / p90 | 4.0 cm / 8.2 cm |
| speed, median / p90 (its xy part, its z part) | 0.11 / 0.21 m/s (0.106, 0.016 median) |
| his logged reason (132 rows) | speed 91, xy 38, dwell 2, stale 1 |
| correlation of xy and speed | 0.03 |

**Verdict:** the speed, not the xy, held the pickup: the typed 0.06 m and 0.15 m/s closes the capture at 0.56 s and
holds it for runs up to 3.5 s, and either change alone closes it only after 7-12 s.

The replay stops at the capture: his descent to the object (low tier 0.10 m, high 0.12 m, 1 s dwell) is not replayed.

## W2c. The ring's own motion

`/payload/motion_capture_state`. Tilt sd is the rocking of the ring's normal (roll and pitch about their window mean).
Drift is the xy rms below 0.3 Hz. No window with the notch has a spectral peak between 0.3 and 2 Hz: the spectrum falls
from the lowest frequency.

| Flight, window | Filter | QUAD4 from the ring centre | Length (s) | xy rms (cm) | z sd (cm) | Tilt sd (deg) | Dominant |
|---|---|---|---|---|---|---|---|
| h4f_3, square at the top | 2 Hz low-pass x1 | carrier | 5.0 | 0.7 | 0.8 | 1.2 | drift (0.28 Hz) |
| ch1b_2, square hover | 2 Hz low-pass x2 | carrier | 15.0 | 1.9 | 1.4 | 1.6 | drift |
| sq2_4, triangle lift and top | 2 Hz low-pass x2 | on its mat | 6.5 | 2.8 | 11.2 (lift) | 6.0 | 1.14 Hz |
| sq5_2, his climb | notch | 1.93 m | 8.1 | 1.9 | 2.0 | 1.7 | drift |
| sq3, QUAD4 held at its climb end | notch | 1.34 m | 38.0 | 2.2 | 1.2 | 1.9 | drift |
| sq2_5, his climb | notch | 1.79 m | 23.5 | 3.1 | 1.9 | 3.2 | drift |
| k1_4, QUAD4 over the pickup | notch | 1.80 m | 26.2 | 3.3 | 1.8 | 2.0 | drift |
| sq4_5, QUAD4 at the search floor | notch | 1.77 m | 33.4 | 3.4 | 3.7 | 2.9 | drift |
| sq5_2, his transfer to plate 9 | notch | 1.14 m | 6.8 | 2.4 | 2.1 | 2.0 | drift |
| sq2_5, his C++ transit | notch | 0.43 m | 19.8 | 3.1 | 2.2 | 2.5 | drift |
| sq2_5, his MATCH and descent | notch | 0.21 m | 23.7 | 3.3 | 3.4 | 2.7 | drift |
| sq5_2, his MATCH and 6 descents | notch | 0.21 m | 25.0 | 5.0 | 3.6 | 3.7 | drift |

**Verdict:** the motion is there before QUAD4 comes near: our triangle wanders 1.9-3.4 cm rms with QUAD4 1.3-1.9 m away,
a slow drift and not a swing. QUAD4 over plate 9 left it unchanged in sq2_5 (3.3 cm) and raised it to 5.0 cm in sq5_2
(no contact was made). The square is calmer (0.7-1.9 cm). sq2_4's 1.14 Hz swing is the low-pass, with QUAD4 on its mat.

## Method

- W1: statics with the planner's own `balanced_tensions` (level ring, straight cables at 60 deg, attach radius and height
  from real.yaml and ring_m2a.yaml, carriers 0.583 kg), then throttle = 0.185 + 0.022 (23.5 - V) + 0.507 |F|/g. Measured:
  tracker `thr_out` and `battery_v` medians in 4 s bins of hand-picked steady windows (per-second timelines). Margin
  = 0.95 - (calibrated start + the bin's p95 effort) and, after 90 s, plus the median fade.
- W2a, W2b: his predicates re-run on his 30 Hz ticks (`/join_planner/reference` stamps) with sample-and-hold inputs from
  the bag, each checked against his own 5 Hz CSV (`logs/join_planner/`, files 10-28-45 and 10-39-12).
- W2c: ring pose resampled at 100 Hz, Welch with 10 s segments.

## Limits

- The ring-motion script named in the request was not in the scratchpad, so W2c uses its own definitions. Its sq5_2
  attach window (5.0 cm, tilt sd 3.7 deg) is not the quoted 4.6 cm and 2.2 deg.
- No `launch_resolved_params` files exist for these runs. The descent values come from his CSV, our launch and his code.
- No clean triangle hover before his TAKEOFF exists: on every attach flight his TAKEOFF went within 2 s of the top. W2c
  uses QUAD4 far from the ring instead.
- The fade is measured on triangle hovers of 20-40 s only. The T and the pair draw more current and probably fade faster.
- The calibration's gain correction is linear in thrust. It mispredicts a near-free carrier (the T's plate 3), so that
  cell is the model's.
- The 28 Oct pairs flew 0.55 m cables (0.55 kg quads), not 1 m.
- Past each real abort and past the capture, the record is what QUAD4 actually flew, not what it would have done.

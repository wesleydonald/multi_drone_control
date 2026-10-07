# drop_trigger_replay (8 Oct 2026): when would a tracker-side ring-tilt drop trigger have fired on the rig?

Exploratory, offline (no card, no flight). Input to `2026-10-08_drop_and_land.md` (critic must-fix 1-3).

- **What:** candidate tracker-side rules for "ring tilt over 30 deg: release the magnets and land", replayed against the trackers' 60 deg / 3-tick latch. A mocap glitch must not fire them.
- **Data:** every rosbag2 bag of 30 Sep (none recorded), 1 Oct (`h1` has no ring pose, `h5_z100_rot120`) and 7 Oct (32 bags): 14 flights in all with the ring airborne, ring pose at 91 Hz; tracker ticks from the trackers' `log.csv`. Flights with no ring pose in a bag come from the planner log (10 Hz).
- **Method:** tilt = acos(1 - 2(qx^2 + qy^2)). Airborne = ring z > first-second median + 0.05 m, armed, before the first disarm. Releases = magnet OFF commands plus any drone whose pivot-to-plate distance leaves its rod length (the rod is rigid). Rules run on every ring sample ("mocap") and on the four trackers' logged 50 Hz ticks ("50 Hz"); a stale repeat is not a new sample.
- **Reproduce:** `python3 tools/drop_trigger_replay.py` (read-only, rosbag2_py; about 6 min from the bags, or `--cache DIR` to keep the extracted arrays). It prints every table below.

## Tables

<!-- tools/drop_trigger_replay.py: nominal mocap frame 10.99 ms, S 15 deg/frame, R 100 deg/s -->

### Per-event timeline

Bag clock (recorder receive time; rosout stamps are the same host clock), shown as epoch seconds mod 1000. Offsets from the event's first release (the magnet OFF, or the onset of an uncommanded release).

| event | what (source) | time | offset (s) | ring z (m) | tilt (deg) |
|---|---|---|---|---|---|
| r0012 recovered | magnet OFF -> /drone_3/magnet (bag) | 34.749 | +0.000 | 1.01 | 1.0 |
| r0012 recovered | radio write (drone_3.elrs_interface, rosout stamp) | 34.750 | +0.001 |  |  |
| r0012 recovered | /drone_3 (az +89, plate 9) off its plate (commanded): separation > 3 mm | 34.950 | +0.201 | 1.00 | 2.5 |
| r0012 recovered | /drone_3 (az +89, plate 9): rod error > 3 cm (rod 0.554 m) | 36.042 | +1.293 |  |  |
| r0012 recovered | operator LAND (/fleet/command, bag) | 42.749 | +8.000 |  |  |
| r0012 recovered | peak ring tilt (then recovered) | 48.803 | +14.054 | 0.17 | 7.4 |
| r0013 capsize | magnet OFF -> /drone_3/magnet (bag) | 934.636 | +0.000 | 1.00 | 2.5 |
| r0013 capsize | radio write (drone_3.elrs_interface, rosout stamp) | 934.637 | +0.001 |  |  |
| r0013 capsize | /drone_1 (az +34, plate 11) off its plate (no command): separation > 3 mm | 934.790 | +0.155 | 1.00 | 2.6 |
| r0013 capsize | /drone_3 (az +89, plate 9) off its plate (commanded): separation > 3 mm | 934.798 | +0.162 | 0.99 | 2.8 |
| r0013 capsize | /drone_1 (az +34, plate 11): rod error > 3 cm (rod 0.552 m) | 934.832 | +0.196 |  |  |
| r0013 capsize | /drone_3 (az +89, plate 9): rod error > 3 cm (rod 0.550 m) | 934.861 | +0.225 |  |  |
| r0013 capsize | ring tilt > 30 | 934.956 | +0.321 | 0.92 | 30.7 |
| r0013 capsize | first 40 deg warning (tracker_3, rosout stamp) | 934.995 | +0.359 |  |  |
| r0013 capsize | ring tilt > 45 | 935.010 | +0.375 | 0.88 | 46.0 |
| r0013 capsize | /drone_2 (az -178, plate 6) off its plate (no command): separation > 3 mm | 935.021 | +0.386 | 0.88 | 46.0 |
| r0013 capsize | /drone_2 (az -178, plate 6): rod error > 3 cm (rod 0.560 m) | 935.056 | +0.420 |  |  |
| r0013 capsize | ring tilt > 60 | 935.065 | +0.430 | 0.82 | 60.1 |
| r0013 capsize | **LATCH** tracker_1 "payload tilt 68.6 deg > 60.0 -" (rosout stamp) | 935.107 | +0.472 | 0.77 | 68.6 |
| r0013 capsize | /fleet/abort (bag) | 935.111 | +0.475 |  |  |
| r200006 recovered | /drone_1 (az -87, plate 3) off its plate (no command): separation > 3 mm | 855.543 | +0.000 | 1.00 | 0.9 |
| r200006 recovered | /drone_1 (az -87, plate 3): rod error > 3 cm (rod 0.555 m) | 855.600 | +0.057 |  |  |
| r200006 recovered | ring tilt > 30 | 856.053 | +0.510 | 0.88 | 30.1 |
| r200006 recovered | peak ring tilt (then recovered) | 856.108 | +0.564 | 0.88 | 30.6 |
| r200006 recovered | operator LAND (/fleet/command, bag) | 856.131 | +0.588 |  |  |
| r200006 capsize | /drone_2 (az -177, plate 6) off its plate (no command): separation > 3 mm | 858.330 | +0.000 | 0.61 | 7.7 |
| r200006 capsize | /drone_2 (az -177, plate 6): rod error > 3 cm (rod 0.549 m) | 858.367 | +0.037 |  |  |
| r200006 capsize | ring tilt > 30 | 858.532 | +0.202 | 0.44 | 30.1 |
| r200006 capsize | first 40 deg warning (tracker_0, rosout stamp) | 858.588 | +0.258 |  |  |
| r200006 capsize | ring tilt > 45 | 858.606 | +0.276 | 0.35 | 45.2 |
| r200006 capsize | ring tilt > 60 | 858.702 | +0.372 | 0.24 | 61.9 |
| r200006 capsize | **LATCH** tracker_3 "payload tilt 61.1 deg > 60.0 -" (rosout stamp) | 858.755 | +0.426 | 0.22 | 61.1 |
| r200006 capsize | /fleet/abort (bag) | 858.761 | +0.431 |  |  |

### Trigger rules (S = 15 deg per 11 ms frame = 1365 deg/s; R = 100 deg/s)

Capsize columns: fire time minus the logged latch, ms (negative = before the latch; after it = FAIL). "mocap" = checked on every ring sample (bag time); "50 Hz" = checked at the trackers' logged ticks, fleet = first tracker; "min" = the smallest lead of any tracker over its own replayed 60 deg latch.

| rate | rule | r0013 fire - latch (ms) | r200006 fire - latch (ms) | fires after a release the ring recovered from | false fires (no release; 14 flights) |
|---|---|---|---|---|---|
| mocap | a1: tilt > 30, 1 sample | -151 | -224 | r200006 +0.51 s, peak 30.6 | 0 |
| mocap | a2: tilt > 30, 2 samples | -140 | -215 | r200006 +0.52 s, peak 30.6 | 0 |
| mocap | a3: tilt > 30, 3 samples | -130 | -203 | r200006 +0.53 s, peak 30.6 | 0 |
| mocap | b1: a1 + glitch reset | -151 | -224 | r200006 +0.51 s, peak 30.6 | 0 |
| mocap | b2: a2 + glitch reset | -140 | -215 | r200006 +0.52 s, peak 30.6 | 0 |
| mocap | b3: a3 + glitch reset | -130 | -203 | r200006 +0.53 s, peak 30.6 | 0 |
| mocap | c1: tilt > 25 and rate > 100 deg/s, 1 sample, glitch reset | -171 | -247 | none | 0 |
| mocap | c2: tilt > 25 and rate > 100 deg/s, 2 samples, glitch reset | -161 | -236 | none | 0 |
| mocap | d: c2, or tilt > 45 on 2 samples; glitch reset | -161 | -236 | none | 0 |
| 50 Hz | a1: tilt > 30, 1 sample | -150 (min +135) | -213 (min +192) | r200006 +0.51 s, peak 30.6 | 0 |
| 50 Hz | a2: tilt > 30, 2 samples | -131 (min +117) | -199 (min +171) | r200006 +0.53 s, peak 30.6 | 0 |
| 50 Hz | a3: tilt > 30, 3 samples | -112 (min +100) | -179 (min +152) | r200006 +0.55 s, peak 30.6 | 0 |
| 50 Hz | b1: a1 + glitch reset | -150 (min +135) | -213 (min +192) | r200006 +0.51 s, peak 30.6 | 0 |
| 50 Hz | b2: a2 + glitch reset | -131 (min +117) | -199 (min +171) | r200006 +0.53 s, peak 30.6 | 0 |
| 50 Hz | b3: a3 + glitch reset | -112 (min +100) | -179 (min +152) | r200006 +0.55 s, peak 30.6 | 0 |
| 50 Hz | c1: tilt > 25 and rate > 100 deg/s, 1 sample, glitch reset | -170 (min +160) | -242 (min +220) | none | 0 |
| 50 Hz | c2: tilt > 25 and rate > 100 deg/s, 2 samples, glitch reset | -150 (min +135) | -226 (min +199) | none | 0 |
| 50 Hz | d: c2, or tilt > 45 on 2 samples; glitch reset | -150 (min +135) | -226 (min +199) | none | 0 |

### Replay check: today's 60 deg / 3-tick latch on each tracker's logged ticks

| run | tracker | replayed latch | tilt seen (deg) | logged latch | logged tilt | replay - logged (ms) |
|---|---|---|---|---|---|---|
| r0013 | tracker_0 | 935.109 | 68.6 | fleet abort came first | - | - |
| r0013 | tracker_1 | 935.117 | 71.3 | 935.107 | 68.6 | +10 |
| r0013 | tracker_2 | 935.129 | 72.2 | 935.113 | 65.9 | +16 |
| r0013 | tracker_3 | 935.107 | 68.6 | fleet abort came first | - | - |
| r200006 | tracker_0 | 858.744 | 63.9 | fleet abort came first | - | - |
| r200006 | tracker_1 | 858.751 | 61.1 | fleet abort came first | - | - |
| r200006 | tracker_2 | 858.746 | 61.1 | fleet abort came first | - | - |
| r200006 | tracker_3 | 858.746 | 61.1 | 858.755 | 61.1 | -10 |

### Largest airborne ring tilt per flight (bags)

Airborne = ring z > its first-second median + 0.05 m, while a tracker log runs (armed), before the first disarm.

| date | run | airborne (s) | max tilt, no release (deg) | margin to 30 | max tilt after a recovered release | max tilt on the floor after TAKEOFF | releases |
|---|---|---|---|---|---|---|---|
| 2026-10-01 | h5_z100_rot120 | 41 | 11.9 | +18.1 | - | 5.3 | - |
| 2026-10-07 | r000014 | 65 | 11.5 | +18.5 | - | 11.0 | - |
| 2026-10-07 | r0001 | 18 | 7.3 | +22.7 | - | 8.1 | - |
| 2026-10-07 | r00013 | 0 | - | - | - | - | ring never airborne |
| 2026-10-07 | r00014 | 0 | - | - | - | - | ring never airborne |
| 2026-10-07 | r0005 | 46 | 9.6 | +20.4 | - | 9.5 | - |
| 2026-10-07 | r0007 | 80 | 11.3 | +18.7 | - | 10.7 | - |
| 2026-10-07 | r001 | 0 | - | - | - | - | ring never airborne |
| 2026-10-07 | r0012 | 31 | 14.9 | +15.1 | 7.4 | 7.6 | recovered: /drone_3 (OFF) |
| 2026-10-07 | r0013 | 14 | 10.8 | +19.2 | - | 9.8 | capsize: /drone_3 (OFF), /drone_1 (no command), /drone_2 (no command) |
| 2026-10-07 | r0014 | 0 | - | - | - | - | ring never airborne |
| 2026-10-07 | r002 | 32 | 10.5 | +19.5 | - | 8.9 | - |
| 2026-10-07 | r005 | 0 | - | - | - | - | ring never airborne |
| 2026-10-07 | r006 | 52 | 6.6 | +23.4 | - | 4.4 | - |
| 2026-10-07 | r007 | 0 | - | - | - | - | ring never airborne |
| 2026-10-07 | r00_dry | 0 | - | - | - | - | ring never airborne |
| 2026-10-07 | r01 | 0 | - | - | - | - | ring never airborne |
| 2026-10-07 | r011 | 19 | 17.1 | +12.9 | - | 11.0 | - |
| 2026-10-07 | r012 | 0 | - | - | - | - | ring never airborne |
| 2026-10-07 | r014 | 0 | - | - | - | - | ring never airborne |
| 2026-10-07 | r02 | 0 | - | - | - | - | ring never airborne |
| 2026-10-07 | r06 | 0 | - | - | - | - | ring never airborne |
| 2026-10-07 | r200006 | 20 | 6.6 | +23.4 | 30.6 | 6.7 | recovered: /drone_1 (no command); capsize: /drone_2 (no command) |
| 2026-10-07 | r20004 | 21 | 7.1 | +22.9 | - | 4.9 | - |
| 2026-10-07 | r20006 | 0 | - | - | - | - | ring never airborne |
| 2026-10-07 | r2004 | 0 | - | - | - | - | ring never airborne |
| 2026-10-07 | r2006 | 0 | - | - | - | - | ring never airborne |
| 2026-10-07 | r2008e60 | 80 | 7.5 | +22.5 | - | 8.8 | - |
| 2026-10-07 | r204 | 0 | - | - | - | - | ring never airborne |
| 2026-10-07 | r206 | 0 | - | - | - | - | ring never airborne |
| 2026-10-07 | r206e60 | 48 | 7.4 | +22.6 | - | 9.0 | - |
| 2026-10-07 | r208e60 | 0 | - | - | - | - | ring never airborne |

### Flights with no ring pose in a bag: planner log (10 Hz)

Armed = a tracker log is running; airborne = planner load z > its first-second median + 0.05 m. 30 Sep ran the earlier controller (controller_quad_load).

| date | run | planner session | airborne (s) | max tilt (deg) | margin to 30 | time > 30 (s) | ring z at max | in LAND | max 10 Hz rate above 25 (deg/s) | a1-b3 fire (> 30 for >= 0.1 s) | c1 at 10 Hz (R 100) | c2 at 10 Hz |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 2026-09-30 | lift_f10 | 154156 | 20 | 10.3 | +19.7 | 0.0 | 0.27 | no | - | no | no | no |
| 2026-09-30 | lift_f11 | 155815 | 17 | 8.3 | +21.6 | 0.0 | 0.36 | yes | - | no | no | no |
| 2026-09-30 | lift_f11 | 160236 | 31 | 12.8 | +17.2 | 0.0 | 0.31 | no | - | no | no | no |
| 2026-09-30 | lift_f5 | 124851 | 1 | 23.0 | +7.0 | 0.0 | 0.14 | no | - | no | no | no |
| 2026-09-30 | lift_f6 | 151732 | 4 | 45.6 | -15.6 | 1.1 | 0.27 | yes | 142 | yes | yes | no |
| 2026-09-30 | lift_f7 | 152654 | 19 | 12.2 | +17.8 | 0.0 | 0.33 | no | - | no | no | no |
| 2026-09-30 | lift_f8 | 153155 | 27 | 13.7 | +16.3 | 0.0 | 0.37 | no | - | no | no | no |
| 2026-09-30 | lift_f9 | 153713 | 26 | 11.3 | +18.7 | 0.0 | 0.20 | no | - | no | no | no |
| 2026-09-30 | model_f1 | 172526 | 8 | 16.0 | +14.0 | 0.0 | 0.39 | no | - | no | no | no |
| 2026-09-30 | r3b0_f2 | 120212 | 13 | 12.9 | +17.1 | 0.0 | 0.12 | no | - | no | no | no |
| 2026-09-30 | r3b0_f4 | 122337 | 1 | 32.3 | -2.3 | 0.3 | 0.18 | no | 66 | yes | no | no |
| 2026-10-01 | c1_z100 / h2_z080 | 162039 | 24 | 11.5 | +18.5 | 0.0 | 0.21 | no | - | no | no | no |
| 2026-10-01 | c1_z100 | 162812 | 46 | 11.3 | +18.7 | 0.0 | 0.40 | yes | - | no | no | no |
| 2026-10-01 | f8_z100 | 170032 | 62 | 9.0 | +21.0 | 0.0 | 0.62 | no | - | no | no | no |
| 2026-10-01 | h1 | 161415 | 13 | 13.8 | +16.2 | 0.0 | 0.43 | yes | - | no | no | no |
| 2026-10-01 | h6_z100_xp60 | 173400 | 32 | 10.5 | +19.5 | 0.0 | 0.39 | no | - | no | no | no |

### Magnet command to release (announced detaches)

The commanded drone, and any other drone with a rod error over 3 cm within 1 s of the OFF and before the fleet disarm. Baseline = median excess over the 1 s before the OFF; separation = 6 samples in a row > baseline + 3 mm; climb = peak drone vz in the 0.3 s after the separation. The ring-attitude column is confounded: the planner resizes the OCP at the same instant.

| run | drone | commanded | rod before OFF (m) | OFF (bag) | radio write (ms) | separation > 3 mm (ms) | rod error > 3 cm, either sign (ms) | climb (m/s) | ring attitude 1 deg off its OFF pose (ms) |
|---|---|---|---|---|---|---|---|---|---|
| r0012 | /drone_3 (az +89, plate 9) | yes | 0.551 (1.5 mm sd) | 34.749 | +1 | +201 | +1293 | +0.10 | +78 |
| r0013 | /drone_3 (az +89, plate 9) | yes | 0.551 (1.1 mm sd) | 934.636 | +1 | +162 | +225 | +0.03 | +103 |
| r0013 | /drone_1 (az +34, plate 11) | NO | 0.552 (0.7 mm sd) |  |  | +155 | +196 | +1.65 |  |
| r0013 | /drone_2 (az -178, plate 6) | NO | 0.560 (1.0 mm sd) |  |  | +386 | +420 | +1.06 |  |

### Ring pose data quality (bags)

| date | run | ring samples | rate (Hz) | gaps > 0.1 s | NaN | quaternion sign flips | largest step in flight, outside a capsize (deg) | steps > S (15 deg) |
|---|---|---|---|---|---|---|---|---|
| 2026-10-01 | h1 | no ring pose in the bag |  |  |  |  |  |  |
| 2026-10-01 | h5_z100_rot120 | 61289 | 91.3 | 56.76 s at 626.260 (not in flight), 945.11 s at 859.574 (not in flight), 0.35 s at 44.907 (not in flight) | 0 | 0 | 0.69 | 0 (0 in flight) |
| 2026-10-07 | r000014 | 12377 | 91.1 | none | 0 | 1 | 0.99 | 0 (0 in flight) |
| 2026-10-07 | r0001 | 5425 | 90.7 | none | 0 | 0 | 0.43 | 0 (0 in flight) |
| 2026-10-07 | r00013 | 1206 | 92.0 | none | 0 | 0 | - | 0 (0 in flight) |
| 2026-10-07 | r00014 | 3144 | 91.8 | none | 0 | 0 | - | 0 (0 in flight) |
| 2026-10-07 | r0005 | 7593 | 90.5 | none | 0 | 0 | 0.60 | 0 (0 in flight) |
| 2026-10-07 | r0007 | 12794 | 90.7 | none | 0 | 0 | 0.94 | 0 (0 in flight) |
| 2026-10-07 | r001 | 5115 | 91.9 | 0.34 s at 738.571 (not in flight) | 0 | 0 | - | 0 (0 in flight) |
| 2026-10-07 | r0012 | 68083 | 91.5 | 2.46 s at 445.859 (not in flight) | 0 | 0 | 0.81 | 0 (0 in flight) |
| 2026-10-07 | r0013 | 15863 | 91.2 | none | 0 | 0 | 0.84 | 1 (0 in flight) |
| 2026-10-07 | r0014 | 4109 | 91.8 | 0.15 s at 458.689 (not in flight) | 0 | 0 | - | 0 (0 in flight) |
| 2026-10-07 | r002 | 46262 | 91.6 | none | 0 | 0 | 0.64 | 0 (0 in flight) |
| 2026-10-07 | r005 | 3313 | 90.7 | none | 0 | 0 | - | 0 (0 in flight) |
| 2026-10-07 | r006 | 7541 | 90.3 | none | 0 | 0 | 0.50 | 0 (0 in flight) |
| 2026-10-07 | r007 | 53 | 88.7 | none | 0 | 0 | - | 0 (0 in flight) |
| 2026-10-07 | r00_dry | 7511 | 90.4 | none | 0 | 0 | - | 0 (0 in flight) |
| 2026-10-07 | r01 | 5556 | 91.0 | none | 0 | 0 | - | 0 (0 in flight) |
| 2026-10-07 | r011 | 10116 | 90.7 | 0.34 s at 595.258 (not in flight) | 0 | 0 | 0.69 | 0 (0 in flight) |
| 2026-10-07 | r012 | 6845 | 91.1 | none | 0 | 0 | - | 0 (0 in flight) |
| 2026-10-07 | r013 | no ring pose in the bag |  |  |  |  |  |  |
| 2026-10-07 | r014 | 14557 | 91.8 | 0.34 s at 257.360 (not in flight) | 0 | 0 | - | 0 (0 in flight) |
| 2026-10-07 | r02 | 1517 | 91.3 | 0.21 s at 115.419 (not in flight) | 0 | 0 | - | 0 (0 in flight) |
| 2026-10-07 | r06 | 3827 | 91.5 | none | 0 | 0 | - | 0 (0 in flight) |
| 2026-10-07 | r200006 | 41981 | 92.0 | none | 0 | 0 | 1.18 | 3 (0 in flight) |
| 2026-10-07 | r20004 | 4856 | 90.8 | none | 0 | 0 | 0.72 | 0 (0 in flight) |
| 2026-10-07 | r20006 | 1480 | 90.2 | none | 0 | 0 | - | 0 (0 in flight) |
| 2026-10-07 | r2004 | 1696 | 90.7 | none | 0 | 0 | - | 0 (0 in flight) |
| 2026-10-07 | r2006 | 2034 | 91.3 | none | 0 | 0 | - | 0 (0 in flight) |
| 2026-10-07 | r2008e60 | 26427 | 91.8 | none | 0 | 0 | 0.65 | 0 (0 in flight) |
| 2026-10-07 | r204 | 5905 | 91.1 | none | 0 | 0 | - | 0 (0 in flight) |
| 2026-10-07 | r206 | 3076 | 91.7 | none | 0 | 0 | - | 0 (0 in flight) |
| 2026-10-07 | r206e60 | 44121 | 91.6 | none | 0 | 0 | 0.56 | 0 (0 in flight) |
| 2026-10-07 | r208e60 | 2461 | 90.5 | none | 0 | 0 | - | 0 (0 in flight) |

### How S and R were chosen

| quantity | flight | value |
|---|---|---|
| largest attitude step in a capsize (release to latch), per nominal frame (raw) | r0013 | 4.13 (4.47) deg |
| largest attitude step in a capsize (release to latch), per nominal frame (raw) | r200006 | 3.00 (3.00) deg |
| S = 3 x that, rounded up to 5 deg |  | 15 deg per frame (1365 deg/s) |
| slowest of the first three capsize samples above 25 deg | r0013 | 229 deg/s |
| slowest of the first three capsize samples above 25 deg | r200006 | 170 deg/s |
| fastest tilt rate above 25 deg after a release the ring recovered from | r200006 | 67 deg/s |
| R = geometric mean of the two, rounded down to 10 |  | 100 deg/s |

### Synthetic ring-pose mis-fits injected into r0013 hover at 929.636 (mocap rate)

| injected | tilt (deg) | a1 | a2 | a3 | b1 | b2 | b3 | c1 | c2 | d |
|---|---|---|---|---|---|---|---|---|---|---|
| mis-fit 90 deg, 1 frame | 88 | FIRES | no | no | no | no | no | no | no | no |
| mis-fit 90 deg, 2 frames | 88 | FIRES | FIRES | no | FIRES | no | no | no | no | no |
| mis-fit 90 deg, 3 frames | 88 | FIRES | FIRES | FIRES | FIRES | FIRES | no | no | no | FIRES |
| mis-fit 90 deg, 5 frames | 88 | FIRES | FIRES | FIRES | FIRES | FIRES | FIRES | no | no | FIRES |
| mis-fit 90 deg, held, 50 frames | 88 | FIRES | FIRES | FIRES | FIRES | FIRES | FIRES | no | no | FIRES |
| upside down, 1 frame | 177 | FIRES | no | no | no | no | no | no | no | no |
| mis-fit 35 deg, 1 frame | 33 | FIRES | no | no | no | no | no | no | no | no |
| mis-fit 35 deg, held, 50 frames | 33 | FIRES | FIRES | FIRES | FIRES | FIRES | FIRES | no | no | no |

## Recommendation

- **Rule d, checked on every ring sample in the payload callback while the ring is airborne:** drop when tilt > 25 deg and rising > 100 deg/s on 2 consecutive samples, or tilt > 45 deg on 2 consecutive samples.
- **Glitch reset:** a sample more than 15 deg (attitude, per 11 ms frame) from the previous one resets the counts and is skipped. The same callback stands down the 60 deg payload fault, because the magnets open 0.16-0.20 s after the command.
- **Lead over the latch:** 161 ms on r0013 and 236 ms on r200006 (50 Hz ticks: 150 / 226; slowest tracker 135 / 199 ahead of its own latch). It made 0 false fires in 14 flights and did not fire on the recovered r200006 first slip (30.6 deg). A plain 30 deg rule (b2) leads by 140 / 215 ms, but it fires on that slip and on 30 Sep lift_f6 and r3b0_f4.

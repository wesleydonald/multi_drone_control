## Flights

| date | run | ring flew | airborne s | fleet stop (disarm, abort, fault) | detach_detect_m flown | largest rod excess while on (cm) | |log len - bag| median / p95 (cm) |
|---|---|---|---|---|---|---|---|
| 2026-10-07 | r000014 | yes | 65.0 | none | 0.0 | /drone_0:0.9 /drone_1:1.0 /drone_2:1.3 /drone_3:1.8 | 0.00 / 0.25 |
| 2026-10-07 | r0001 | yes | 18.0 | none | 0.0 | /drone_0:0.5 /drone_1:0.7 /drone_2:0.8 /drone_3:0.6 | 0.01 / 0.52 |
| 2026-10-07 | r00013 | no (ring never airborne) |  |  |  |  |  |
| 2026-10-07 | r00014 | no (ring never airborne) |  |  |  |  |  |
| 2026-10-07 | r0005 | yes | 46.4 | none | 0.0 | /drone_0:0.7 /drone_1:0.9 /drone_2:1.3 /drone_3:2.3 | 0.01 / 0.34 |
| 2026-10-07 | r0007 | yes | 80.2 | none | 0.0 | /drone_0:0.7 /drone_1:0.9 /drone_2:0.9 /drone_3:1.5 | 0.00 / 0.20 |
| 2026-10-07 | r001 | no (ring never airborne) |  |  |  |  |  |
| 2026-10-07 | r0012 | yes | 31.1 | none | 0.0 | /drone_0:0.4 /drone_1:1.1 /drone_2:1.6 /drone_3:0.5 | 0.00 / 0.02 |
| 2026-10-07 | r0013 | yes | 14.4 | tracker_1 ENVELOPE FAULT | 0.0 | /drone_0:3.2 /drone_1:0.4 /drone_2:1.2 /drone_3:0.5 | 0.00 / 0.14 |
| 2026-10-07 | r0014 | no (ring never airborne) |  |  |  |  |  |
| 2026-10-07 | r002 | yes | 32.1 | none | 0.0 | /drone_0:1.0 /drone_1:0.9 /drone_2:1.5 /drone_3:1.8 | 0.00 / 0.05 |
| 2026-10-07 | r005 | no (ring never airborne) |  |  |  |  |  |
| 2026-10-07 | r006 | yes | 51.8 | none | 0.0 | /drone_0:0.6 /drone_1:0.7 /drone_2:1.0 /drone_3:1.4 | 0.01 / 0.27 |
| 2026-10-07 | r007 | no (ring never airborne) |  |  |  |  |  |
| 2026-10-07 | r00_dry | no (ring never airborne) |  |  |  |  |  |
| 2026-10-07 | r01 | no (ring never airborne) |  |  |  |  |  |
| 2026-10-07 | r011 | yes | 19.5 | operator DISARM | 0.0 | /drone_0:0.4 /drone_1:1.4 /drone_2:2.2 /drone_3:2.0 | 0.01 / 0.47 |
| 2026-10-07 | r012 | no (ring never airborne) |  |  |  |  |  |
| 2026-10-07 | r013 | no (ring never airborne) |  |  |  |  |  |
| 2026-10-07 | r014 | no (ring never airborne) |  |  |  |  |  |
| 2026-10-07 | r02 | no (ring never airborne) |  |  |  |  |  |
| 2026-10-07 | r06 | no (ring never airborne) |  |  |  |  |  |
| 2026-10-07 | r200006 | yes | 19.9 | tracker_3 ENVELOPE FAULT | 0.0 | /drone_0:2.8 /drone_1:0.5 /drone_2:0.8 /drone_3:2.2 | 0.00 / 0.07 |
| 2026-10-07 | r20004 | yes | 20.5 | none | 0.0 | /drone_0:1.2 /drone_1:0.7 /drone_2:1.4 /drone_3:0.6 | 0.01 / 0.70 |
| 2026-10-07 | r20006 | no (ring never airborne) |  |  |  |  |  |
| 2026-10-07 | r2004 | no (ring never airborne) |  |  |  |  |  |
| 2026-10-07 | r2006 | no (ring never airborne) |  |  |  |  |  |
| 2026-10-07 | r2008e60 | yes | 80.5 | none | 0.0 | /drone_0:1.0 /drone_1:0.6 /drone_2:1.0 /drone_3:0.6 | 0.00 / 0.14 |
| 2026-10-07 | r204 | no (ring never airborne) |  |  |  |  |  |
| 2026-10-07 | r206 | no (ring never airborne) |  |  |  |  |  |
| 2026-10-07 | r206e60 | yes | 47.5 | none | 0.0 | /drone_0:1.3 /drone_1:0.7 /drone_2:1.1 /drone_3:0.5 | 0.00 / 0.02 |
| 2026-10-07 | r208e60 | no (ring never airborne) |  |  |  |  |  |


## 1. Slip detector (bag, full rate); times on the run clock of tools/rig_to_run.py

| run | drone | what | OFF (s) | onset (s) | |rod error| > 3 cm (s) | onset - OFF (ms) | 3 cm - OFF (ms) | peak |rod error| (cm) | verdict |
|---|---|---|---|---|---|---|---|---|---|
| r0012 | /drone_3 | magnet OFF (planner) | 32.893 | 33.094 | 34.186 | 201 | 1293 | - | detected |
| r0013 | /drone_1 | UNCOMMANDED | - | 31.863 | 31.904 | - | - | 257 | no OFF explains it |
| r0013 | /drone_3 | magnet OFF (planner) | 31.708 | 31.870 | 31.933 | 162 | 225 | - | detected |
| r0013 | /drone_2 | UNCOMMANDED | - | 32.105 | 32.128 | - | - | 60 | no OFF explains it |
| r200006 | /drone_1 | UNCOMMANDED | - | 36.250 | 36.307 | - | - | 113 | no OFF explains it |
| r200006 | /drone_2 | UNCOMMANDED | - | 39.037 | 39.074 | - | - | 297 | no OFF explains it |


## 2. Stretch detector (detach_detect_m 0.06), replayed at the planner ticks

| run | replayed on | drone | release | release (s) | first tick over (s) | fire (s) | fire - release (ms) | over at fire (cm) | flight logged (s) | logged - release (ms) | verdict |
|---|---|---|---|---|---|---|---|---|---|---|---|
| r000014 | log | - | - | - | - | - | - | - | - | - | no fire, no release to detect |
| r000014 | bag | - | - | - | - | - | - | - | - | - | no fire, no release to detect |
| r0001 | log | - | - | - | - | - | - | - | - | - | no fire, no release to detect |
| r0001 | bag | - | - | - | - | - | - | - | - | - | no fire, no release to detect |
| r0005 | log | - | - | - | - | - | - | - | - | - | no fire, no release to detect |
| r0005 | bag | - | - | - | - | - | - | - | - | - | no fire, no release to detect |
| r0007 | log | - | - | - | - | - | - | - | - | - | no fire, no release to detect |
| r0007 | bag | - | - | - | - | - | - | - | - | - | no fire, no release to detect |
| r0012 | log | - | - | - | - | - | - | - | - | - | no fire (announced release: the planner already knew) |
| r0012 | bag | - | - | - | - | - | - | - | - | - | no fire (announced release: the planner already knew) |
| r0013 | log | /drone_1 | uncommanded | 31.863 | 31.927 | 32.018 | 155 | 30.6 | none logged | - | detection |
| r0013 | log | /drone_2 | uncommanded | 32.105 | - | - | - | - | - | - | no fire possible: the fleet stopped 75 ms after it |
| r0013 | bag | /drone_1 | uncommanded | 31.863 | 32.018 | 32.138 | 275 | 72.6 | none logged | - | detection |
| r0013 | bag | /drone_2 | uncommanded | 32.105 | - | - | - | - | - | - | no fire possible: the fleet stopped 75 ms after it |
| r002 | log | - | - | - | - | - | - | - | - | - | no fire, no release to detect |
| r002 | bag | - | - | - | - | - | - | - | - | - | no fire, no release to detect |
| r006 | log | - | - | - | - | - | - | - | - | - | no fire, no release to detect |
| r006 | bag | - | - | - | - | - | - | - | - | - | no fire, no release to detect |
| r011 | log | - | - | - | - | - | - | - | - | - | no fire, no release to detect |
| r011 | bag | - | - | - | - | - | - | - | - | - | no fire, no release to detect |
| r200006 | log | /drone_1 | uncommanded | 36.250 | 36.419 | 36.528 | 278 | 17.5 | none logged | - | detection |
| r200006 | log | /drone_2 | uncommanded | 39.037 | - | - | - | - | - | - | detector not active (after LAND) |
| r200006 | bag | /drone_1 | uncommanded | 36.250 | 36.419 | 36.528 | 278 | 39.7 | none logged | - | detection |
| r200006 | bag | /drone_2 | uncommanded | 39.037 | - | - | - | - | - | - | detector not active (after LAND) |
| r20004 | log | - | - | - | - | - | - | - | - | - | no fire, no release to detect |
| r20004 | bag | - | - | - | - | - | - | - | - | - | no fire, no release to detect |
| r2008e60 | log | - | - | - | - | - | - | - | - | - | no fire, no release to detect |
| r2008e60 | bag | - | - | - | - | - | - | - | - | - | no fire, no release to detect |
| r206e60 | log | - | - | - | - | - | - | - | - | - | no fire, no release to detect |
| r206e60 | bag | - | - | - | - | - | - | - | - | - | no fire, no release to detect |


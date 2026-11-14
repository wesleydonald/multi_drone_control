# Re-attach contact rule: every real contact replayed at comotion_normal_max 0.04 / 0.02 / 0.01 (13 Nov 2026)

## Verdict
The data support `comotion_normal_max` **0.01**:
- Across all 25 rig contacts it blocks or delays all 7 high contacts (+8.5 mm or more), and every one of those failed.
- It blocks none of the 9 proven catches. Two are delayed, by 13 ms and 94 ms, inside QUAD4's ~0.1 s response delay.

0.02 is not enough. It still fires on:
- fm9c_2 (13 Nov), 14 ms later, at +15.6 mm;
- fm9_5 try 1 (12 Nov), 35 ms later, at +11.2 mm.

## Method
- **Bags:** 179 rig bags with /magnet/attach_proof. 27 flights had the magnet ON over a descent, and 15 of them gave 25 contacts (4, 5, 11, 12 and 13 Nov).
- **Replay:** the flight code's own ComotionProof, stepped on /magnet_tip_pose and /payload/motion_capture_state. Each flight uses its own typed gates, plate point and magnet ON/OFF.
- **Check:** the replay reproduces all 25 logged contacts at the typed gate (median 2 ms, max 13 ms).
- **Counterfactual:** a fresh WATCH starts from each try's start, with only normal_max changed. Each replay is valid only up to the real contact plus about 0.1 s, because his LIFT_PROOF followed every contact by 6-38 ms.

## Every contact event (normal in mm at the logged contact)
See events.csv and tables.md.
- **Proven catches:** -2.7 to +1.2 mm (at 0.01: -3.1 to +1.2).
- **Highest real catch:** nr1_2 on 4 Nov, plate 1, at +5.3 to +6.1 mm. It still passes, with about 4 mm of margin.
- **fm9c_2 (13 Nov):**
  - The logged contact normal is +13.6 mm. The mean of its last 0.1 s is +6.6.
  - The tip dipped to +1.8 mm at -0.11 s while the ring bobbed by about 1 cm, then rebounded as QUAD4's body rose 2 cm.
  - The 0.30 s speed fit passed (0.050 m/s against the 0.06 limit) on a path that went down and back up.
  - His LIFT_PROOF started 27 ms after the contact.
- **Not fixed by any normal gate:** nro (moving ring), sq6m try 1 (plate edge), and the off-plate contacts under 4 Nov's 60 mm xy gate.

## QUAD4's tip against his reference
- **His lift is immediate:** his planner has no bottom hold. 15 of the 25 contacts, including 6 of the 9 proven, came during the descent.
- **11-13 Nov, during the descent:** the body lagged his reference by 0.06-0.45 s, sitting 2-25 mm above it.
- **At the bottom (ATTACH_READY):**
  - The body sat **33-47 mm above his reference** and did not close the gap.
  - His reference would put the tip 34-40 mm below the plate.
  - So the intended ~3.5 cm press ends with the tip at about plate level, and the ring's ±1 cm bob decides whether it touches.
  - The same offset shows at the pickup (results/analysis/pickup_seat_2026-11-13/: body 5-45 mm above the reference, cable taut).

## Caveats
- The 10 mm margin depends on weld_point_z being right. A funnel, magnet or plate change needs it re-measured.
- With 0.01, a QUAD4 sitting more than about 1 cm high at the bottom gets no contact. It waits there until an alignment abort or the 120 s give-up.
- A deeper contact_dz is tuning on his side.
- The sample is small: 9 proven catches, one ring (M2A), funnels only since 11 Nov.

Files: events.csv, replay.csv, verify.csv, flights.csv, scan_all_bags.csv, contact_events.png, tables.md, windows/, scripts
(scan.py, extract.py, analyze.py, plot_events.py, tables.py). data/ (74 MB) can be rebuilt with extract.py.

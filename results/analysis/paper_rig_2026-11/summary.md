# The combined paper's real-world tables: what the flights give so far (6 Nov 2026, 15:24)

These come from `paper_rig.py` and `paper_tables.py`. All flights so far are exploratory: no bars were set before them, so
any judgement is made after the flight. The paper's own flights start on Wed 11 Nov: `rel`, `relo`, `nrtf` and `nro`
(scripts written), and `fm9`, `fm9o`, `fm9h` (scripts for 18 Nov).

## Re-run after each session (11, 12, 18 and 19 Nov)
```bash
source ~/ros2_humble/install/setup.bash && source ~/multi_drone_control/install/setup.bash
cd ~/multi_drone_control
# first: the day's registry rows (a VOID verdict there leaves a flight out)
# then: the funnel lines in results/analysis/paper_rig_2026-11/conditions.csv (e.g. "2026-11-11/nrtf*,yes,,v3 on plate 9");
#   a flight from 11 Nov on without a line shows as "funnel ?"
nice -n 10 python3 results/analysis/paper_rig_2026-11/paper_rig.py
python3 results/analysis/paper_rig_2026-11/paper_tables.py
```
- With no argument the script reads the 8 Oct context flights, every attach-mode flight of 4-5 Nov, and every flown flight
  from 11 Nov on, found through `launches.csv`.
- The first read of a bag takes about 1 min per flight; after that the bags are cached in `~/mdc_overnight/joins_cache` and
  `~/mdc_overnight/paper_rig_cache`, and a re-run takes seconds.
- Run it with the rig stack down, and not in the first 3 min of a laptop Gazebo run.
- `paper_rig.py 2026-11-11` reads one day only; the CSVs then hold only that day.

Flights are sorted by their launch line, not their name:
- Dissipative mode with an operator OFF on `/drone_N/magnet` is an unannounced release (`rel`, `relo`); on an orbit it
  counts as moving.
- In attach mode, an OFF on `/drone_3/magnet_operator` while QUAD4 is still a carrier also counts as an unannounced release.
- `floor_join` with QUAD4 welded is a re-attach.
- `pickup_*` with QUAD4 welded is the application; `pickup_*` from the mat is pickup and drop only.
- `delivery_rejoin` (fm9h) is timed as lift, transit, drop, re-attach and land.

## What each cell has now (1 m cables, ring config 0.86 kg)

| Cell | Plan | Now | Result |
|---|---|---|---|
| Detach unannounced, static | 5 | 0 | none yet |
| Detach unannounced, moving | 3 | 0 | none yet |
| Detach planned, static, triangle | – | 8 | 8/8 held; dip 22 mm (13-50, n=7); swing 6.0 deg (3.9-11.6); settled at the new tilt in 5.0 s (1.3-6.6), 7 of 7 |
| Detach planned, moving | – | 1 (nr9o_2) | held; swing 6.2 deg; dip 40 mm; path deviation 5.2 cm |
| Detach planned, square | – | 1 (sq8_2) | held, but the T leaned +12.4 deg (peak 24.5) |
| Re-attach on 9, static, no funnel | – | 5 | 0/5 (0/4 with a descent); 23 descents, 20 at plate height, 6 contacts; miss 6.4 cm median (3.1-9.7) |
| Re-attach static, funnel | 5 | 0 | none yet |
| Re-attach moving | 3 | 1 (no funnel) | 0/1; misses 8.3 and 8.9 cm |
| Re-attach on plate 1 | – | 2 | 0/2 |
| Re-attach, square | – | 2 | 1/2: proven 14.3 s after release, folded in 2.5 s after contact; tilt 26.7 -> 4.2 deg, settled in 6.7 s |
| Attach from the mat | – | 1 (sq6m) | 1/1, 24.8 s from his TAKEOFF |
| Application, static | 2-3 | 4 | 0/4 completed (below) |
| Application, moving | 1-2 | 0 | none yet |
| Pickup and drop only | – | 5 static, 2 moving | 1 completed (k1d, 81 s) |

Application details: fm9_2, fm9_4 and fm9s stopped in the lift (the Motive recording was on); fm9y_2 got through the
detach (12.4 s), the pickup (99.3 s, 3 attempts) and the transit (6.8 s), and the object landed in the net; it then missed
the re-attach (3 descents, no contact).

Left out of the main tables: nr9 and nr1_2 (they flew the 1.15 kg config on the 0.86 kg ring), sq6b (the 1.15 kg ring),
nr9m_3, fm9y and k1o (VOID in the registry).

No 1 m unannounced release exists yet. The 1 m `UNANNOUNCED DETACH` lines are accidental releases (h4, ch1, ch1b on 29 Oct,
each a drop by rule), a misread (ch1b_2: QUAD4's released magnet read as drone 1's, the markers swapped), during LAND (h3t,
sq8_2's false loss) and a void start (nr9m_3). The only operator releases are the 8 Oct flights on 0.55 m links (r303_4,
r304_3, r305), kept in a separate context table as a check of the method (r303_4: detected 0.42 s after the OFF, tilt peak
6.5 deg, dip 31 mm, settled in 6.0 s).

## Definitions (as coded; Wesley decides)
The metrics are the twin's X1 / T6 ones (`tools/x1_metrics.py`, `tools/release_bands.py`), so the rig and the twin compare
directly.
- Event time: the unannounced detach at the operator's OFF in the bag; the planned detach at our OFF after the 3 s unload;
  the attach at the fold-in (FLEET RESIZED back to 4 after `ATTACH: proven`).
- Window: ends at the first of the operator's LAND, a loss, the next resize, the event + 15 s, or (for a detach) QUAD4's
  next descent.
- Before: the 5 s before the event, cut at the set-point's last move; under 1 s left leaves the "before" metrics empty (only
  nr9m_4, which detached 0.9 s after the lift).
- Swing: the tilt vector's largest change from its mean before (tilt peak and tilt rise also reported).
- Time to settle: X1's t_level, the first time after the peak from which the tilt stays within max(2 sigma_pre, 1 deg) of its
  mean before for 1 s; a second version measures the same band around the mean of the window's last 5 s, searched from the
  event. The attach table uses the second (a fold-in changes the ring's level on purpose); planned detaches also settle at a
  new level (2 of 7 came back to the old one). A "1 deg, held 2 s" band never holds on the rig (sigma_pre 0.4-2.4 deg); a 2 s
  hold column is kept in `events.csv`.
- Dip: the mean height before minus the lowest height in the next 10 s. Horizontal excursion: hovers only.
- Set-point deviation: 3-D RMS against `/payload/desired_position`, over the 10 s after and over the time before. Path
  deviation: timing-free on a path (the planner holds the trajectory 1.5 s at each resize).
- Detection delay: the planner's `UNANNOUNCED DETACH` line minus the OFF. Separation: QUAD4's magnet marker 2 cm from where
  it sat on the plate.
- Success: detach: acted on, then no drop, fault or position limit, and the ring above 0.3 m until LAND or the event + 15 s;
  attach: proven, folded in, then the same 15 s.
- Descents and misses: F052's definitions (the unchanged re-run of `joins.py`).
- Mission phases, each from the end of the previous: lift (TAKEOFF to the ring within 10 cm of the top), detach (UNLOAD to
  PARTNER RELEASE), pickup (to "object lifted"), transit (to DROP_OBJECT), drop (to CLEAR_DROP_ZONE), re-attach (to the
  fold-in), land (LAND to "Landed - disarming"). A failed mission is "stopped in" the phase running at the first loss or the
  operator's LAND.

## Checks
- Recomputed by hand from the raw files for r303_4 and sq8_2 (`check_by_hand.py` shares no code with `paper_rig.py`: it reads
  the bag's sqlite itself, the log lines, the tracker CSV and the raw ~90 Hz mocap): every metric agrees, the largest
  difference 0.09 mm of dip (`check_by_hand.txt`).
- Against `tools/release_bands.py`: r303_4 swing 5.28 vs 5.29, peak 6.51 vs 6.50, rise 3.87 vs 3.87; r305 swing 8.01 vs 8.02,
  settled at the new level 3.93 vs 3.93 s; r304_3 swing 16.88 vs 16.88.
- Against the registry: detection 0.418, 0.332, 0.345 s against 0.42, 0.33, 0.34; tilt peaks 6.5, 17.8, 8.1 deg match;
  r304_3's 5.3 cm drop and 16.1 cm swing match.
- Against F052: the `joins.py` re-run matches all 64 descents cell for cell.

## Reading notes
- In a hover the ring sits 3-6 cm off its set-point all the time: compare "after" with "before".
- sq8_2's attach counts as a success, but the ring was dropped during LAND 13.7 s after the fold-in on a false loss
  (recorded in `flights.csv` under losses).

(Written by an agent 6 Nov ~15:24; saved by the lab chat from its report.)

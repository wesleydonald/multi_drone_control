# Plan after the 15 Oct rig day (written 15 Oct night; supersedes the 14 Oct plan's row 9)

Deadline 20 Nov. Branch `overnight-1015` (from week5 48ccad6; today's rig work committed first, 1057c7e).

## Where we are

- **Rig access.** Four quads together reach 98 dB(A), so the lab stopped them. The supervisor (Mitch) is getting
  approval for four-drone flights, aiming for Wed 21 Oct, probably early-morning slots; no flying Fri 16 Oct.
  He says it is time to start wrapping up: once reattaching is reliable, the rest is data collection.
- **Hardware he is making for next week:** bigger plates (discs), longer cables (1 m, as TU Delft), maybe a smaller
  basket; later perhaps 5" drones or bi-blade props (quieter). All of it touches the attach geometry and the twin.
- **Today on the rig** (rows RIG-1015-*):

| Flight | What happened | What it tells us |
|---|---|---|
| q4solo, q4circle_2 | QUAD4 alone, FC accelerometer vs mocap | FC accel = mocap to ~0.1 m/s^2 in the world frame after its z offset (+2.5 %); live accel not needed |
| c1_3 | props-off console check | PASS after replugging QUAD1/QUAD2 (`!ERR*`, FC tilt wrong) |
| a1 (aim B) | 3 descents, 2 bottom holds, no contact | at plate height the tip slid 6-8 cm toward the ring centre (coil ON); QUAD4's tether passed 0.10 m from QUAD2 on the way in |
| a2_2 | QUAD2 let go of plate 7 in MATCH | ring tilt 37 deg, drop rule worked; LAND left QUAD4 on his mission for 14 s |
| a3 | flown with the carriers two plates off | pretension stretched QUAD2's rod: drop on the floor |
| a3_2 (filter on) | 42 s in MATCH, never descended | his match gate needs 1.0 s unbroken; held 1.03 s at best |
| fh3_2 | three carriers alone | clean hover and landing |

## What blocks a reliable reattach (ranked from the data)

1. **The catch is small for how much the tip moves.** Over 8 rig flights (14-15 Oct, 42 s at the bottom) the tip
   was inside the 24 mm catch 22 % of the time (14 % inside and slow). The plate's own motion (0.8 cm bounce at
   1.8 Hz) and QUAD4's wander (2-4 cm) are each about the catch size.
2. **The ring moves the target.** His reference copies the plate's bounce and QUAD4 follows it 0.25 s late (tip vs
   plate 1.6 cm, twice the plate's own); the plate's sideways speed (0.09 m/s rms) keeps his match gate shut.
3. **At plate height the tip slides off the inner edge** (a1, twice, coil ON): cause not known.
4. **Carrier release** (a2_2, QUAD2 on plate 7, packs at 21.9 V): not explained; the magnets' hold was never measured.
5. **Operations:** layout errors get flown (a3); QUAD4's straight path can cross a carrier (a1); a drop during his
   flight does not take QUAD4 back on LAND (a2_2); QUAD1/QUAD2 FC `!ERR*` (fc_request, supervisor's OK pending).

## The supervisor's hardware, quantified (answers for Mitch)

**Disc size** (`tools/attach_window.py --catch K`, the same 8 flights, tip vs plate point):

| Catch radius K | Disc diameter (about) | Bottom: inside | Bottom: inside and slow | MATCH: inside |
|---|---|---|---|---|
| 24 mm | today | 22 % | 14 % | 13 % |
| 35 mm | 70 mm | 35 % | 21 % | 25 % |
| 45 mm | 90 mm | 49 % | 28 % | 38 % |
| 55 mm | 110 mm | 58 % | 34 % | 52 % |

A 90 mm disc doubles the time the magnet can catch. It also forgives the inward slide of item 3 by 2 cm.

**Cable length and basket size** (geometry, 60 deg carrier cables, QUAD4's tip on plate 9, carriers on 7 and 11):

| Carrier cable | Ring radius | QUAD4 tether | QUAD4 body to nearest carrier | QUAD4 tether to nearest rod (at the ring) | QUAD4 above the ring | Swing |
|---|---|---|---|---|---|---|
| 0.55 (today) | 0.225 | 0.575 | 0.45 m | 0.23 m | 0.64 m | 0.66 Hz |
| 0.55 | 0.225 | 1.0 | 0.70 m | 0.23 m | 1.06 m | 0.50 Hz |
| 1.0 | 0.225 | 0.575 | 0.70 m | 0.23 m | 0.64 m | 0.66 Hz |
| 1.0 | 0.225 | 1.0 | 0.66 m | 0.23 m | 1.06 m | 0.50 Hz |
| 1.0 | 0.15 | 1.0 | 0.61 m | 0.15 m | 1.06 m | 0.50 Hz |

- Either 1 m cable gives QUAD4 ~0.25 m more room from the carriers' bodies (0.45 to ~0.7 m).
- A 1 m **QUAD4 tether** also lifts QUAD4 from 0.64 to 1.06 m above the ring: less downwash on the ring (on 14 Oct the
  ring tilted 5-8 deg with QUAD4 overhead, 1-3 deg without), and a slower swing (0.50 Hz).
- **A smaller basket** costs little body clearance (0.04 m) but brings the neighbouring rods closer to QUAD4's tether
  at the ring (0.23 to 0.15 m), and twelve 90 mm discs need a radius of at least ~0.18 m (30 deg pitch). Fewer
  plates (only 3, 7, 9, 11 are used) would allow a smaller ring.

## Work packages

| # | Package | Why | Status | Evidence |
|---|---|---|---|---|
| A1 | Next week's hardware in the twin: 1 m cables (carriers, QUAD4), bigger catch | the next rig day flies them; the stack's geometry, his parameters and the gates must be ready | to do | |
| A2 | A drop during his flight takes QUAD4 back (behind a switch, critic, twin) | a2_2: QUAD4 flew on toward the dropped ring | to do | |
| A3 | The console refuses ARM on a SLOT OFFSET over 30 deg | a3 flew two plates off | to do | |
| A4 | His match gate: typed-only dwell (default his 1.0 s) | a3_2: 42 s without a descent; replay: 0.5 s passes at 1.1 s | to do (Wesley skipped it on the day; build, ask) | |
| A5 | The plate-height filter (attach_z_lowpass) through the twin | flown on the rig with critic and twin waived | to do | |
| A6 | Rig sheet for the next lab day (four quads, early slot, new hardware) | | after A1-A5 | |
| B1 | Carry on every trajectory x 3 speeds, three carriers (twin) | Wesley's rig plan for 15 Oct, thesis carry table | running (R1723-R1752, lab PC) | |
| B2 | Inventory of thesis-ready sim results, stale vs current code, figure plan | thesis | to do | |
| B3 | Carry with 1 m cables vs 0.55 (twin) | design result for the thesis and Mitch | after A1's worlds | |
| C1 | Disc size and cable/basket numbers for Mitch | his questions | done (tables above) | |

## Overnight queue (15-16 Oct; the loop works down it)

1. B1 on the lab PC (running), score it: tracking error, ring tilt, aborts per kind and speed.
2. A1: rig-twin worlds with 1 m carrier cables (and a 1 m QUAD4 tether for the attach world), new files only;
   carry configs; then B3 (hover + circle, 0.55 vs 1.0) on the lab PC.
3. A3 and A4 (small, behind switches), unit tests, gate.
4. A2: card, critic, code behind a switch, twin runs (drop during his MATCH).
5. A5: the filter in the twin attach (m1c_floorjoin_hover with and without).
6. B2 inventory.
7. Morning report `docs/reports/morning_report_2026-10-16.md`.

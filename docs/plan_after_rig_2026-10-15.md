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
3. **At plate height the tip falls off the plate's edge.** Every bottom hold on 14-15 Oct (his ATTACH_READY, ring frame,
   plates 60 mm): the slides (a1 x2, t6h5_3) came with the tip at plate height 3.3-4.3 cm along the ring (+x)
   from the plate point, past the plate's 30 mm edge; it dropped 4-5 cm and swung toward the ring's hollow centre.
   The one hold that went on to the proof (t6h_4) arrived 2.2 cm off. A 90 mm disc (45 mm radius) would have had all
   three slides on the plate. (scratchpad slide_scan.py; the a1 registry row's "cause not known" is superseded.)
4. **Carrier release** (a2_2, QUAD2 on plate 7, packs at 21.9 V): not explained; the magnets' hold was never measured.
   The carrier coils are ON from each launch onward (rig_magnet_latches) and nothing records their current or
   temperature (readiness review 29 Sep): a low pack and a warm coil both weaken an electromagnet. A spring-scale pull
   test per carrier, cold and after ~20 min ON, at a full and a low pack, separates the two (props off).
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
| A1 | Next week's hardware in the twin: 1 m cables (carriers, QUAD4), bigger catch | the next rig day flies them; the stack's geometry, his parameters and the gates must be ready | done: 1 m carry = 0.55 m (R1769-R1778), 1 m attach 2/2 (R1760, R1761) | cca8803, 6856cf8 |
| A2 | A drop during his flight takes QUAD4 back (behind a switch, critic, twin) | a2_2: QUAD4 flew on toward the dropped ring | done (twin): 8 on runs pass, off = a2_2's hazard; reviewer SUPPORTED WITH CORRECTIONS; rig use is Wesley's decision (morning report 2) | bad76e3, 42d81fe, 0a3ac30, 8694f95 |
| A3 | The console refuses ARM on a SLOT OFFSET over 30 deg | a3 flew two plates off | done (`--slot-gate`, off by default) | 35d2b86 |
| A4 | His match gate: typed-only dwell (default his 1.0 s) | a3_2: 42 s without a descent; replay: 0.5 s passes at 1.1 s | done (`match_dwell`, `match_speed` typed only; replay tool) | ab6fd18 |
| A5 | The plate-height filter (attach_z_lowpass) through the twin | flown on the rig with critic and twin waived | twin done: -35 % tip bounce at 1.8 Hz, worse at 0.8 Hz (reviewer: weak there); needs his bottom hold on the raw plate first: not for the next lab day. Zero plate velocity (`attach_twist:=zero`): -27 % at 1.8 Hz, opens a3_2's gate on the bag, critic fixes done; the guard bug fixed (9c04ee3, it also fired in a hover); a low-passed twist (critic F2) tried and removed (R1835-R1842) | 8d2036c, e9ff80a, efb9b5d, 9c04ee3, b0398f6 |
| A6 | Rig sheet for the next lab day (four quads, early slot, new hardware) | | draft: `docs/rig/next_lab_day_draft.md` (date, hardware and Wesley's decisions to fill in) | d34c421, 9ca9b28, 44cf72c |
| B1 | Carry on every trajectory x 3 speeds, three carriers (twin) | Wesley's rig plan for 15 Oct, thesis carry table | done: 30/30 clean (docs/analysis/traj3_2026-10-15.md); draft claim card | d4f8ce8, e96ae68, 7af0260 |
| B2 | Inventory of thesis-ready sim results, stale vs current code, figure plan | thesis | done: docs/analysis/sim_results_inventory_2026-10-16.md | 26ac24f |
| B3 | Carry with 1 m cables vs 0.55 (twin) | design result for the thesis and Mitch | done: same within noise | b786e8f |
| C1 | Disc size and cable/basket numbers for Mitch | his questions | done (tables above) | |
| D1 | his LAND console button (Wesley 16 Oct) | a drop during his flight; drop_takes_back on at the rig as well | built + twin: after a drop it lands QUAD4 on the fallen ring (R1844-R1846); use only clear of the ring | 5375333, e836cf3 |
| D2 | Carry claim on the freeze tree (card bars frozen 16 Oct) | thesis T7t/F8t | flown: 39/40 pass; spin at 0.3 m/s fails bar 3 once (R1875, 6.31 deg): outside the envelope; 1 m arm passes (R1847-R1886); reviewer running | 5375333, e836cf3 |
| D3 | Detach freeze: Tier A rev 3 (T3, T8, T9, K1, T15), Tier B, W4/W5, D1/D4 | thesis T3/T9/T13, F12 | flown and scored (R1887-R1919): every bar confirmed except K1 planned (unresolved); B4 holds on the measured pull (G150p reached 30.9 deg); A6 (planned, detection off) holds the ring at 28 deg and aborts at landing 3/3; reviewer next | 83a7088, 8c468bd |
| D4 | N2: twin bands on the 1 m worlds | the next rig day's ledger rows | flown: every cell clean after the laptop re-fly of 10 lab voids and 7 starved lab passes (R2006-R2015, R2077-R2083; corrections R1921c-R1936c); the band rule needs Wesley's word (`docs/experiments/2026-10-16_bands_1m.md`) | 83a7088 |
| D5 | Appendix re-fly on the freeze tree and the tables regenerated | thesis appendix | flown and scored (R2023-R2076, laptop); reviewer weak: C-D2/C-D3/C-D5 falsified (the measured pull absorbs a vertical push), C-L1 not supported (false touchdown: 3-4 of the 4 thrust-map landings disarmed 0.4 m up, R2032c-R2045c); generators edited, tables regenerated to the scratchpad only; `docs/analysis/appendix_refly_2026-10-17.md` | 48d93d7, 8c468bd |
| D6 | Corner cutting at speed (Wesley 16 Oct ~10:00; fix plan approved ~17:20, `~/.claude/plans/kind-snacking-pinwheel.md`) | carry tracking at 0.3 m/s | ruler fixed: the frozen RMSE charged an on-time ring 17 mm; clock-free, the timing combination removes the cut (figure-8 bends +20..+31 -> +3..+7 mm) and turns the lag into a 0.04-0.06 s lead on the circle/orbit (correction rows R1986c-R1991c). G2 (R1992-R2005): pure delay on the orbit/circle (centred at plan_ref_lag_s 0.075-0.078 = 0.1 - D), the figure-8 needs ~0.03 s less; offline bench failed G1 (code paths only). Figure-8 repeats (R2016-R2022): the extra 0.03 s is not supported; one lead fits all three shapes. Next: Wesley's call on the build (measured lead recommended) | 63e4b52, baf50e1, 98a2b8e, 7281a3c, 4d88533, 6970467, 391bd9a |

## Overnight queue (15-16 Oct; the loop works down it)

1. B1 on the lab PC (running), score it: tracking error, ring tilt, aborts per kind and speed.
2. A1: rig-twin worlds with 1 m carrier cables (and a 1 m QUAD4 tether for the attach world), new files only;
   carry configs; then B3 (hover + circle, 0.55 vs 1.0) on the lab PC.
3. A3 and A4 (small, behind switches), unit tests, gate.
4. A2: card, critic, code behind a switch, twin runs (drop during his MATCH).
5. A5: the filter in the twin attach (m1c_floorjoin_hover with and without).
6. B2 inventory.
7. Morning report `docs/reports/morning_report_2026-10-16.md`.

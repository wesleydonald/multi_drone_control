# Thesis R: every 0.55 m twin table re-flown at 1 m (lab batches 7 and 16, 23 Oct)

Card `docs/experiments/2026-10-23_refly_1m.md`, bars frozen 02:15 (each original card's own claim and thresholds).
Scored with `tools/thesis_runsets.py` and `tools/refly_score.py`. 1 m runs: R2483-R2577 (part 1 R2483-R2537, part 2
R2538-R2577), lab PC, 4 slots. A (agreement with 0.55 m) is for information only. The tree changed since 0.55 m
(card section 3), so a change cannot be put down to the cable without an attribution re-fly.
The five 1 m worlds' tables (Tier B, the push rows, gain) flew in lab batch 16: R2604-R2628, scored 23 Oct 11:28 with
the original cards' own tools (section "Batch 16" below).

| table | claim | 0.55 m | 1 m | verdict |
|---|---|---|---|---|
| ledger_w4 (T13 W4) | the freed drone is disarmed alone (P1-P4) | R1887 (n = 1): P1 0.28 s, dtheta 6.05 deg | R2483-R2485: P1 0.35-0.36 s, P3 <= 0.003 m, P4 0, dtheta 6.95-7.15 deg | HOLDS (3 of 3) |
| ledger_w5 (T13 W5) | the freed drone is disarmed alone, circle (P1-P4) | R1913/R1914: P1 0.28/0.27 s, dtheta 6.70/7.11 deg | R2492/R2501: P1 0.34/0.32 s, dtheta 7.85/8.02 deg | HOLDS (2 of 2) |
| tier_a T3 | measured < planned pull (A1 vs A2) | 6.40 vs 16.80 deg, gap +10.40 (need 4.44) | 7.28 vs 17.09 deg, gap +9.82 (need 1.20) | HOLDS |
| tier_a T8 | detection on < off, planned pull (A2 vs A6) | 16.80 vs 28.18 deg, gap +11.39 (need 1.00) | 17.09 vs 21.66 deg, gap +4.56 (need 1.00) | HOLDS (A6 smaller; no LAND abort, 0.55 m had 3 of 3) |
| tier_a T9 | no sign, measured pull (A1 vs A5) | gap -0.81 (need 4.44): unresolved | gap +0.34 (need 1.38): unresolved | HOLDS |
| tier_a K1 | landing lean, reported, not credited | reported | not scored (release_transient.py) | reported only |
| t15 (T9 row T15) | planned < measured on the 60 deg circle (A8 vs A7) | 0.58/0.49 vs 0.87/0.98 cm, gap +0.39 (need 0.30) | 0.61/0.61 vs 0.91/1.00 cm, gap +0.35 (need 0.30) | HOLDS |
| drop (tab:faults) | a second loss (D1) or a 240 deg gap (D4) drops the ring and lands everyone (v2 bars) | 4 of 4: drop 0.22-0.23 s, tilt <= 13.6 deg, all down 8.5-8.9 s | 4 of 4: drop 0.20-0.22 s, tilt <= 13.7 deg, all down 8.1-8.5 s | HOLDS |
| T7t (main text) | each cell inside at 0.55 m (14 of 15) stays inside (clock-free <= 25 mm, tilt <= 6 deg, ratio >= 0.95) | 14 of 15 inside; spin_v300 out (R1875 tilt 6.31 deg) | 14 of 15 inside; orbit_v300 out (clock-free 25.7/25.9 mm, ratio 0.9499/0.9495); spin_v300 now in | DOES NOT HOLD (orbit_v300 leaves the envelope) |
| traj (tab:traj) | none (reported table: hover row + five shapes at 0.125 m/s) | hover 0.04/0.04 cm mean, 0.06 deg | R2544/R2551: hover 0.03/0.04 cm mean, 0.06 deg; A agrees 18 of 18 | reported (no bar) |
| x0 C-X1 | the relaxed box raises the hover (>= +2 cm, resolved) | +13.13 cm | +18.73 cm | HOLDS |
| x0 C-X2 | pinning cuts the orbit error (X0-O0 <= 0.5 x X0-O1, resolved) | 0.74 vs 3.87 cm, ratio 0.19 | 0.63 vs 3.98 cm, ratio 0.16 | HOLDS |
| robust C-R1 | inside the gate the integral removes the error (every run within 1.0 cm) | worst 0.87 cm | worst 0.41 cm | HOLDS |
| robust C-R2 | footnotes a and b (m+20, rod+5nomeas: integral within 1.0 cm of none) | 0.01 / 0.00 cm (rod+5nomeas +17.09 both) | 0.01 / 0.00 cm (rod+5nomeas +17.75 both; did not shrink) | HOLDS |
| robust C-R3 | measuring the rods absorbs a 5 cm error (within 1.0 cm of X0-H0) | 0.00 cm | 0.00 cm | HOLDS |
| robust C-R4 | every flight landed, no tip past 45 deg | 28 of 28 | 28 of 28 | HOLDS |
| land C-L1 | the default landing does not tip (0 tips, 0 false touchdowns, outward <= 6.0 cm, tilt <= 8.0 deg) | 0 tips of 82; 4 false touchdowns (mapneg, 0.26-0.53 m up); NOT supported | 0 tips of 82 (68 + batch 16's 14 push-arm landings R2615-R2628, all clean); 4 false touchdowns (mapneg R2542/R2549/R2543/R2550, drone 1 0.17 m up); outward <= 0.8 cm, tilt <= 5.3 deg | DOES NOT HOLD (as at 0.55 m) |
| land C-L2 | a straight descent pushes the UAVs outward (L-S above X0-H0) | 23.5/23.6 vs 0.5 cm | 46.7/46.8 vs 0.3 cm | HOLDS on its bar; both L-S runs miss S (false touchdown 16.7-16.8 cm, new at 1 m, rigid-rod) |
| disturbance C-D5 | the model integral removes a thrust-map error's horizontal error (D-MM < D-MR, resolved) | 0.01 vs 0.01 cm: not resolved (falsified) | 0.00 vs 0.01 cm (need 1.00): not resolved | DOES NOT HOLD (as at 0.55 m) |
| flight (fig:flight) | none (one flight from the floor) | R2034 | R2544 (S PASS) | reported (no bar) |
| tier_b B1 (F12b, T9) | curve on rig plates: G120 -> G150 -> G180, smaller margin, larger dtheta (both pairs resolved) | 6.40 -> 16.82 -> >= 61.22 deg: +10.42 (need 4.44), +44.40 (need 2.02) | G120 (A1 R2486/R2495/R2504) 7.28 -> G150 (R2605/R2611) 17.88 -> G180 (R2608/R2614) >= 62.19: +10.59 (need 1.20), +44.31 (need 1.18) | HOLDS |
| tier_b B2 (F12b) | shape: no neighbour pair resolved inverted | G120 -> G135 +2.04 (need 4.44), unresolved; the rest resolved | G135 (R2604/R2610) 10.72, G165 (R2607/R2613) >= 30.16: every neighbour pair resolved with the sign (G120 -> G135 +3.44, need 1.20) | HOLDS |
| tier_b B3 (T9) | the planned pull above the measured at 150 deg (G150 vs G150p) | 16.82 vs >= 29.54: +12.71 (need 4.30) | 17.88 vs G150p (R2606/R2612) 28.36: +10.48 (need 5.04) | HOLDS |
| tier_b B4 (tab:faults) | every G120-G150 absolute peak under the 30 deg drop line | falsified as written: G150p dropped 2 of 2 (abs 28.77/30.90 at the drop); measured-pull arms <= 16.86 | every abs peak < 30, G150p included: max 29.81 (R2606, 0.19 under; R2612 27.31); no drop | HOLDS by the letter, MARGINAL (reviewer 2: 0.19 deg of margin, n = 2 with repeats 2.50 deg apart, under the R card's 0.5 deg tilt floor; the tier_b must-not-say still applies) (0.55 m: falsified as written) |
| tier_b B5 | safety G120-G165 (G150p in): no abort, every drone lands, the freed drone disarmed alone (P1) | 11 of 11: P1 0.19-0.43 s | 9 of 11 (P1 0.33-0.37 s); G165 R2607/R2613 drop the ring at the trackers' 30 deg line (+0.50 s) and the freed drone is disarmed with the fleet, 0.04 s after the drop's LANDED; no abort or fault latch | NOT CONFIRMED (reviewer 2: the bar's falsifier, an abort or fault latch, did not fire; the freed drone disarmed with the fleet, on the ground) |
| tier_b reported | G165, G180, G180d (no bar) | G165 landed, abs 28.10/27.84; G180 >= 60.71/61.72 (60 deg fault, fleet disarmed at 1.45-1.62 m); G180d >= 24.08 (gap rule, +0.32 s) | G165 >= 30.49/29.84 (dropped, all down in 8.2 s); G180 >= 62.46/61.91 (60 deg fault at +0.64 s, fleet disarmed at 1.84-2.00 m); G180d R2609 >= 23.15 (gap rule, +0.30 s) | reported |
| gain C-G1 (tab:gain) | gain 0.8 lowers the peak (G-K8 below D-ZM, resolved) | 1.26/1.44 vs 1.25/1.41 cm: +0.02, ranges overlap: unresolved | G-K8 R2625/R2627 1.42/1.44 vs D-ZM R2617/R2622 1.47/1.42 cm: -0.02, ranges overlap (need 1.00) | UNRESOLVED (as at 0.55 m) |
| gain C-G2 (tab:gain) | the 5 s gate lowers overshoot (G-S5 below D-ZM, resolved) | 1.33/1.31 vs 1.39/1.39 cm: -0.07, under the 1.0 cm floor: unresolved | G-S5 R2626/R2628 1.39/1.40 vs 1.41/1.44 cm: -0.04, under the floor | UNRESOLVED (as at 0.55 m) |
| disturbance C-D1 | the model integral removes a constant push (D-ZM, D-XM steady <= 1.0 cm, every run) | 0.01/0.02, -0.01/0.00 cm | D-ZM R2617/R2622 0.06/0.03, D-XM R2619/R2624 0.05/0.05 cm | HOLDS in x; vertically EMPTY (reviewer 2: D-Z0 without the integral already reads 0.06/0.06, so z shows nothing; the appendix card's C-D2 text) |
| disturbance C-D2 | the push is visible without it (D-Z0, D-X0 steady >= 2 cm) | z 0.03/0.01 (falsified vertically); x 4.82/4.81 cm | D-Z0 R2615/R2620 0.06/0.06; D-X0 R2618/R2623 6.56/6.58 cm | DOES NOT HOLD (vertically, as at 0.55 m; x holds) |
| disturbance C-D3 | it learns the applied force (z 0.45-0.75 N, x 0.30-0.50 N, every run) | z 0.099/0.102 (falsified vertically); x 0.325/0.325 N | z R2617/R2622 0.120/0.112; x R2619/R2624 0.325/0.327 N | DOES NOT HOLD (vertically, as at 0.55 m; x holds) |
| disturbance C-D4 | the target shift removes the vertical push (D-ZR steady <= 1.0 cm) | 0.00/0.02 cm | D-ZR R2616/R2621 -0.06/-0.07 cm | HOLDS, but EMPTY (reviewer 2: the vertical push is invisible without it too, D-Z0 0.06/0.06) |

**Tally (scored claims, 23 Oct 11:28, after batch 16; corrected after reviewer 2, 23 Oct 11:47, verdict [weak]):** 19
hold (W4, W5, T3, T8, T9, T15, drop, C-X1, C-X2, C-R1, C-R2, C-R3, C-R4, B1, B2, B3, B4, C-D1, C-D4), of which B4 is
marginal (0.19 deg, n = 2) and C-D1 (vertically) and C-D4 are empty holds. 5 do not hold: T7t is new at 1 m; C-L1, C-D2,
C-D3 and C-D5 failed the same way at 0.55 m. 1 not confirmed: B5 (its falsifier did not fire). 2 unresolved, as at 0.55 m (C-G1, C-G2: "no
difference resolved"). C-L2 is out of the table (reviewer 05:15; Wesley 23 Oct (9): L-S at 1 m not reported). 0 void,
0 skipped. Reported without a bar: traj, flight, K1, the W4/W5 0.55 m bands (F1), and Tier B's G165/G180/G180d.
B4's verdict flips: falsified as written at 0.55 m, it holds at 1 m. (Before batch 16, as first scored: 14 hold
with C-L2, 3 do not, 3 skipped; after the reviewer: 13, 3, 3.)

**S misses (6 runs, all the frozen false-touchdown bar: a drone more than 10 cm above its rest at LANDED):**
- mapneg R2542/R2549/R2543/R2550. The stall exit fires with drone 1 (its thrust map is stronger than the model) 0.17 m
  up. The fleet disarms and drone 1 drops upright. This is the same S miss as at 0.55 m (R2032c-R2045c, 0.26-0.53 m,
  3 of 4 on their side). It is what fails C-L1. C-D5's hold window ends before it.
- L-S R2564/R2577. New at 1 m (0.55 m: 2.8/4.2 cm). The drones sit 46.7 cm outside their references on the rigid 1 m
  rods, so they cannot descend without moving outward. The descent stalls 0.14-0.17 m up and the stall exit fires;
  they drop and end tilted up to 15 deg, with no tip. This is a rigid-rod twin artefact (the rig's tethers go slack).
  The card's rule says to re-fly a twin artefact once and record it, and the two repeats already agree to 0.1 cm.
  Wesley's call: keep L-S in tab:land at 1 m as a labelled rigid-rod result, or keep the cell out.

**Runner FAILs (13). None is a real 1 m result; each is the same harness check that failed at 0.55 m:**

| runs | check | 1 m | 0.55 m (same check) | cause |
|---|---|---|---|---|
| R2539/R2546 (X0-H1), R2541/R2548 (X0-O1) | hold mean z - target <= 0.05 m | +0.186/+0.187 m | R2024/R2036 +0.131, R2026/R2038 +0.130/+0.129 | the relaxed box's designed hover rise (what C-X1 measures) |
| R2556/R2569 (rod+5nomeas z0), R2562/R2575 (rod+5nomeas int) | hold mean z - target <= 0.05 m | +0.177/+0.178 m | R2054/R2069/R2060/R2075 +0.171 | the unmeasured +5 cm rod (footnote b, what C-R2 scores) |
| R2542/R2549 (D-MR), R2543/R2550 (D-MM) | payload tilt bounded < 5 deg | 8.6-8.7 deg | R2032/R2044/R2033/R2045 9.2-9.4 deg | the peak is at ring lift-off (t 17.9-18.0 s, ring 0.07 m up) |
| R2511 (T7t orbit_v125, part 1) | mocap stale 2.02 s after LANDED | runner stalled 2.4 s wall at the disarm | none | harness; the flight is complete and scored |

## Reviewer corrections and attribution (23 Oct 05:15)
Reviewer [weak]: C-L2 is out of the table (both L-S runs miss S): **13 hold, 3 do not, 3 skipped**; C-D5's mapneg cells
"out (S miss) and unresolved"; R2485's read-back lacks /tracker_2 (rows R2485c, R2564-R2577c, R2542-R2550c, R2521-R2536c).
Attribution re-flies on this tree at 0.55 m (rows R2580-R2583):
| Cell | 0.55 m, old tree | 0.55 m, this tree | 1 m, this tree | Reading |
|---|---|---|---|---|
| T7t orbit_v300 (path mm / radius ratio) | 15.24 / 0.975 (R1871) | 23.72, 22.70 / 0.954, 0.956 | 25.71, 25.91 / 0.950, 0.950 | mostly the tree (the twin's ema filter among others), the cable adds ~2.6 mm |
| L-S landing (S) | pass (R2049/R2064) | pass, pass | FAIL, FAIL | the 1 m cable (rigid rods), not land_rearm_touchdown |

## Batch 16 (23 Oct, scored 11:28): the five 1 m worlds' tables, R2604-R2628
Runs: Tier B R2604-R2614 (11), the push rows R2615-R2624 (10), gain R2625-R2628 (4); lab PC batch 16, every runner PASS.
Tree be34f95 + patch cf1ab9ad. The only `src/` change since R2483-R2577 (patch 6d980933) is the rejoin fixes in
`dissipative_node.py`, all inside the rejoin path (`rejoin_departed` is off in every R config). Checked in each run's
logs: the layout (`attach_azimuths_deg`), `drop_on_loss` false in G180's planner parameters, `cable_source model` on
G150p's four trackers, `int_mode` / `z_ki` / `int_settle_s` per push arm, the push world with its 15 s WRENCH, the ema
velocity filter on every carrier, and the parameter read-back (`missing_required []`).

Scorers: each original card's own tools. `tools/refly_score.py` has no Tier B or push scorer, so a scratchpad wrapper
called them. Tier B: `release_bands.py` Delta-theta from the event to +15 s. A run that drops or faults is cut at the
planner's DROP AND LAND line or the first ENVELOPE FAULT line, mapped onto the ARM-stamp clock as `drop_metrics.py` maps
it. Detection and the planned steps come from `release_transient.py`'s resize, the freed drone from
`departed_metrics.py`. The wrapper reproduces the card's 0.55 m values: R1899 16.99, R1900 >= 28.46, R1901 >= 60.71,
R1903 28.12, R1905 >= 30.61, R1906 >= 61.72, R1919 >= 24.08. The push and gain rows use `push_metrics.push`,
`thesis_sim_tables.onset_settle` / `.learned` and `land_touchdown.land`; these reproduce R2027-R2031, R2047 and R2048.

**Tier B per run** (dtheta in deg; ">=" cut at the drop or fault; P1 = the freed drone's disarm minus its first moment
at rest):

| arm | run | dtheta | abs peak | back in band (s) | detection (s) | leaver's pull before (N) | survivors' largest step (N) | end | freed drone |
|---|---|---|---|---|---|---|---|---|---|
| G120 (A1) | R2486 / R2495 / R2504 | 6.90 / 7.45 / 7.50 | 5.92 / 6.24 / 6.10 | 3.53 / 3.55 / 3.68 | 0.34 / 0.25 / 0.26 | 1.64 / 1.63 / 1.63 | 1.13 / 1.15 / 1.14 | landed | alone, P1 0.37 / 0.37 / 0.36 s |
| G135 | R2604 / R2610 | 10.42 / 11.02 | 9.82 / 10.21 | 3.74 / 3.84 | 0.26 / 0.25 | 1.74 / 1.74 | 1.39 / 1.39 | landed | alone, 0.34 / 0.36 s |
| G150 | R2605 / R2611 | 18.17 / 17.58 | 17.80 / 17.03 | 4.29 / 5.03 | 0.27 / 0.26 | 1.90 / 1.91 | 1.70 / 1.70 | landed | alone, 0.33 / 0.34 s |
| G150p | R2606 / R2612 | 29.62 / 27.10 | 29.81 / 27.31 | 10.11 / 8.12 | 0.23 / 0.22 | 1.90 / 1.89 | 1.72 / 1.71 | landed, no drop (0.55 m: dropped 2 of 2) | alone, 0.34 / 0.36 s |
| G165 | R2607 / R2613 | >= 30.49 / >= 29.84 | 30.30 / 29.57 | - | 0.26 / 0.29 | 2.13 / 2.14 | 2.25 / 2.31 | the trackers' 30 deg drop request at +0.51 / +0.50 s, DROP AND LAND; all down in 8.2 s, LANDED, no fault (0.55 m: landed, peaks 28.10 / 27.84) | with the fleet, 0.04 s after LANDED (0.13 / 0.11 s after rest) |
| G180 | R2608 / R2614 | >= 62.46 / >= 61.91 | 62.42 / 61.92 | - | 0.28 / 0.29 | 2.44 / 2.46 | 3.72 / 3.89 | 60 deg payload fault at +0.64 / +0.65 s, fleet disarmed in the air at 1.84-2.00 m (0.55 m 1.45-1.62) | no (fleet disarm in the air) |
| G180d | R2609 | >= 23.15 | 23.16 | - | none (dropped at detection) | - | - | the planner's gap rule at +0.30 s; all down in 8.2 s, LANDED | alone, 0.22 s |

A agrees on every arm: G135 10.72 against 8.44, G150 17.88 against 16.82, G150p 28.36 against >= 29.54, G165 >= 30.16
against 28.02, G180 >= 62.19 against >= 61.22, G180d >= 23.15 against >= 24.08.

What is new at 1 m:
- **G165 crosses the 30 deg line in 2 of 2.** At 0.55 m it peaked 28.10 / 27.84 deg and landed. G150p no longer
  reaches the line: 29.81 deg in R2606, 0.19 under it, and 27.31 in R2612, with no drop. At 0.55 m G150p dropped 2 of 2.
  So the "tilt over 30 deg" row of tab:faults, which is G150p at 0.55 m, has no G150p drop at 1 m; G165 drops instead.
  G150p's hold is marginal (reviewer 2): one repeat 0.19 deg under the line, the other 2.50 deg under, n = 2, under the
  R card's 0.5 deg tilt floor. So the tier_b card's must-not-say still applies at 1 m: nothing here says a gap up to
  150 deg stays under the line with the planned pull.
- **B5 misses on the drop's landing order.** After a drop the planner announces LANDED once every drone is down, and
  the fleet disarms. In G165 that came 0.07-0.09 s after the freed drone first came to rest, before the freed drone's own
  disarm, which comes 0.33-0.37 s after rest in the other runs. Each drone was disarmed at rest, and there was no abort or
  fault latch, which is B5's falsifier. At 0.55 m G150p's drops left the freed drone disarmed alone, 0.19 s after rest.
- G120 -> G135 is now resolved (+3.44, need 1.20); at 0.55 m it was unresolved (+2.04, need 4.44).
- S: refly_score.py's S check predates Tier B. It reads FAIL on G165 (twice, "drop not expected"), G180 (twice,
  "in-flight fault, no LANDED") and G180d ("drop not expected"). G180's fault and G180d's drop are those arms' designed
  ends, as at 0.55 m. G165's drop was not designed, and the frozen S exempts only D1/D4/A2/A6: reading the tier_b card's
  ">=" rule to exempt it came after the run (reviewer 2). So **G165 is an S miss in both runs** (R2607, R2613: a drop
  not expected); G180 and G180d are not.

**The push and gain arms per run** (cm unless marked):

| arm | runs | steady | peak | overshoot | settling (s) | onset to 1 cm (s) | learned (N) | 0.55 m steady / peak (learned) |
|---|---|---|---|---|---|---|---|---|
| D-Z0 | R2615 / R2620 | 0.06 / 0.06 | 1.47 / 1.45 | 1.44 / 1.43 | 1.19 / 1.15 | 1.31 / 1.28 | - | 0.03 / 0.01, 1.43 / 1.34 |
| D-ZR | R2616 / R2621 | -0.06 / -0.07 | 1.35 / 1.34 | 1.50 / 1.52 | 1.18 / 1.21 | 1.15 / 1.13 | - | 0.00 / 0.02, 1.31 / 1.34 |
| D-ZM | R2617 / R2622 | 0.06 / 0.03 | 1.47 / 1.42 | 1.41 / 1.44 | 1.20 / 1.22 | 1.32 / 1.28 | 0.120 / 0.112 | 0.01 / 0.02, 1.25 / 1.41 (0.099 / 0.102) |
| D-X0 | R2618 / R2623 | 6.56 / 6.58 | 7.22 / 7.16 | 0.55 / 0.50 | 0.82 / 0.85 | not back within 1 cm | - | 4.82 / 4.81, 5.44 / 5.35 |
| D-XM | R2619 / R2624 | 0.05 / 0.05 | 7.09 / 6.99 | 7.03 / 7.09 | 4.47 / 4.46 | 4.45 / 4.47 | 0.325 / 0.327 | -0.01 / 0.00, 5.34 / 5.63 (0.325 / 0.325) |
| G-K8 | R2625 / R2627 | 0.03 / 0.05 | 1.42 / 1.44 | 1.44 / 1.41 | 1.21 / 1.20 | 1.28 / 1.32 | 0.232 / 0.235 | -, 1.26 / 1.44 (0.210 / 0.224) |
| G-S5 | R2626 / R2628 | 0.06 / 0.07 | 1.47 / 1.44 | 1.39 / 1.40 | 1.17 / 1.18 | 1.33 / 1.29 | 0.121 / 0.122 | -, 1.32 / 1.31 (0.105 / 0.106) |

S passes on all 14 runs, and A agrees on all 29 cells (the appendix card's floors; D-X0's steady is 1.36x its 0.55 m
value). The 14 landings are clean: pushed outward 0.27-0.32 cm, tilt 1.2 s after touchdown 2.72-2.92 deg, highest drone
4.5-4.6 cm above rest at LANDED (stall exit). They take C-L1's pool to 82 with the same 4 false touchdowns (mapneg).

Not yet: the reviewer on Tier B, gain and the push rows. The card's reviewer list covers "x0 + robust + disturbance"
and "land"; Tier B and gain need adding.

For Wesley: tab:faults' "tilt over 30 deg" row at 1 m. Recommended: cite G165 (measured pull, 165 deg; drops 2 of 2,
labelled) and keep G150p as B3's arm. The alternative is to keep that row as 0.55 m history. B5's text is reworded by
the card's rule ("9 of 11 freed drones disarmed alone; in G165's two drops, with the fleet at LANDED"), with no re-fly.

# Best-flight review figures, 23 Oct 2026 (not in the thesis)

Built Fri 23 Oct, 15:41-16:14 AEDT; revised 16:15-16:30 on Wesley's answers (colour, ls3c, r302_3, the spin column,
Wed 28 replacements). From recorded run data, read-only (7 Oct rig flights that had no run directory were converted
into the session scratchpad, not into `results/`). Nothing committed, nothing in the thesis. Each figure is a PDF at
print size (thesis text width 6.27 in, or 3.03 in for a half-width figure) and a 300 dpi PNG, drawn by the script of the
same name. `contact_sheet.pdf` has every figure on its own page with its one-line message. Every PDF passes the skill's
audit (size, embedded Computer Modern, no text under 7 pt, no collisions).

**Colour (Wesley 23 Oct: less black, colour the data).** One colour per fleet size in every figure: 2, 3 and 4
carriers are orange `#f79143`, rose `#cc4778` and purple `#6a00a8` (matplotlib's plasma at 0.74, 0.50, 0.20). They are
ordered in lightness (L* 71, 51, 30), so they stay apart in greyscale print and in the deuteranopia and protanopia
previews, and none of them is the comparison red or blue. Red `#b2182b` and blue `#2166ac` (the first set's pair) are
kept for two-way comparisons: planned (blue) against measured (red) tension, and the survivors (blue) against the
released carrier (red). Grey is only for references and context: the reference path, bands, thresholds, caps,
events. Black appears once, for the ring's tilt in `fig_release`, where it sits beside the drones' red and blue.
Rig against twin is filled against open. Fleet-size labels are printed in their colour, so every figure carries its own
key. The rest is the first set's look, reused, not copied: `best_style.py` imports `../review_2026-10-23/review_style.py`.

**How the flights were scored** (`select_flights.py`, `best_data.py`; all numbers in `candidates.json`). Wesley's 22 Oct
rule, as `tools/n2_bands.py` implements it: on a path, the ring's sideways distance to the path its reference traces
plus its height error, RMS from the path's first move to the planner's own descent, clock-free; the lag (s) is
reported apart (`metrics.contour_lag`, per leg on a line). In a hover, the 3-D distance to the hold point from 2 s after
the reference tops out to LAND; a hover scored over less than 4 s does not count. My clock-free numbers reproduce the
registry's for the 22 Oct twin rows (R2405-R2434) to 0.1 mm.

**Re-rendering on new flights (Wed 28).** Every figure reads its flights from the tables at the top of `best_data.py`
(`RIG_BEST`, `TWIN_FILL`, `TWIN_GRID`, `RELEASE`, `DETACH`, `RELEASES`); axis limits and panel names follow the data. To
swap in Wed 28's 1 m flights, change the keys there (`'2026-10-28/<name>'` for a rig flight, `'R####'` for a twin run),
then run each `fig_*.py` and rebuild the contact sheet (`pdflatex contact_sheet.tex`). A rig flight without a
`<name>_run` directory is converted on first use into the scratchpad cache (needs `~/ros2_humble` sourced to read its
bag). The cells marked **Wed 28** below are listed in `best_data.WED28`.

## The flights (step 1)

Rig first; the twin (1 m cables) only where the rig has no flight. Rig flights so far fly 0.55 m rods.

| Cell | Flight | Rig / twin | Error (mm) | Parts | Why this one |
|---|---|---|---|---|---|
| hover, 2 | R2330 `b2_hover_L100_slope` | **twin** | 1.0 | xy 0.5, z 0.9 | **To be replaced by Wed 28.** The rig never held a pair hover: h2slope was landed 2 s after the top of its lift (0 s scored); h2, h2t9p, h2m and h2rts were landed or dropped before the top. |
| hover, 3 | RIG-1022-h3 | rig | 21.8 | xy 19.0, z 10.6 (6.6 s) | Lowest of h3 and lift3_3 (42.1); h3n was landed 1.9 s after the top. |
| hover, 4 | RIG-1007-r20004 | rig | 21.5 | xy 21.3, z 3.1 (5.3 s) | **To be replaced by Wed 28.** Lowest of four (r002 62.1 over 15 s; r011, r0001 under 4 s). 7 Oct tree, carry planner, rods at 44-48 deg. |
| figure-8, 2 | RIG-1022-f2slope | rig | 25.2 | side 21.6, z 13.0; lag 0.05 s | The only pair path flight on the rig (slope filter, cap 0.9). |
| figure-8, 3 | RIG-1022-f3h | rig | 19.0 | side 14.4, z 12.4; lag 0.17 s | Lowest of six (f125_2 27.1, f200 30.3, f3 30.6, f3big 36.8, f125 45.2). |
| figure-8, 4 | RIG-1007-r2008e60 | rig | 17.4 | side 14.7, z 9.3; lag 0.21 s | **To be replaced by Wed 28.** Lowest of two (r0007 83.0). Flown at 0.10 m/s mean (typed 0.25), half the 22 Oct eights' 0.18. |
| circle, 3 | RIG-1021-c200_4 | rig | 17.8 | side 16.9, z 5.9; lag 0.17 s | Lowest of three (c200_2 41.7, c125_2 43.2). |
| circle, 4 | RIG-1007-r206e60 | rig | 14.3 | side 12.9, z 6.3; lag 0.23 s | **To be replaced by Wed 28.** Lowest of three (r006 61.7, r0005 70.9). |
| line, 3 | RIG-1021-l200 | rig | 17.7 | side 14.3, z 10.4; lag 0.10 s | Lowest of two (l125 20.9). |
| spinny line, 2 | R2363 `b2_linespinc_h5_v200_L100_slope` | **twin** | 5.5 | side 4.0, z 3.8; lag 0.04 s | Not flown on the rig; best of six (1 m line, the rig's length). |
| spinny line, 3 | RIG-1022-ls3c | rig | 36.4 | side 25.1, z 26.4; lag 0.11 s | Wesley 23 Oct: today's law (one full turn at a constant rate). ls3, the older law (229 deg), scored 22.6. |
| spinny line, 4 | R2357 `b4_linespinc_h5_v200_L100_slope` | **twin** | 4.0 | side 1.9, z 3.5; lag 0.08 s | **To be replaced by Wed 28** if it flies a four-carrier spinny line. Best of six. |
| spin, 3 | RIG-1021-s125 | rig | 44.5 | side 40.5, z 18.5; lag 0.27 s | In the summary as its own column (Wesley 23 Oct). Lower of two (s200_3 46.4). |
| spin, 4 | RIG-1007-r000014 | rig | 85.7 | side 70.3, z 49.0 | **To be replaced by Wed 28.** The only one (7 Oct, with 100 g added to the ring). |
| release (unannounced) | RIG-1008-r303_4 | rig | | detected 0.42 s, resized 0.51 s, back in band 1.8 s, tilt 6.5 deg | Lowest tilt and fastest settle of the three rig releases (r304_3 17.8 deg / 3.7 s, a 150 deg gap; r305 8.1 deg / 3.8 s on the circle). |
| detach (announced) | RIG-1008-r302_3 | rig | | resized at the command, back in band 1.95 s, tilt 15.4 deg | Wesley 23 Oct. (RIG-1007-r0012 tilted 4.7 deg but the ring rose 15 cm.) |
| release, twin | R2486 `r1m_ta_a1_r1` | twin | | resized 0.37 s, back in band 3.4 s, tilt 5.9 deg | The 1 m version of the rig release (first set's fig_detach_1m); in no figure here. |

The **spin** is the circle (radius 0.5 m, 0.125 m/s) flown with one full yaw turn of the ring (`load_trajectory`
'spin'), not a turn in place.

The **twin grid** in `fig_summary` (b) uses the best repeat of each cell at the rig's path sizes: the 22 Oct baselines
for hover (R2330, R2336, R2349), figure-8 size 1.0 (R2334, R2341, R2353) and spinny line 1 m (R2363, R2345, R2357), and
for three carriers the 23 Oct 1 m re-fly's cells flown as the rig flew them: circle r 0.5 at 0.2 m/s R2528 (7.9 mm),
line 1 m at 0.2 m/s R2530 (1.5), spin at 0.125 m/s R2512 (4.5). The slope filter won 8 of the 9 baseline cells (the
hovers tie within 0.1 mm).

**Not one variable across fleet size on the rig.** 2 and 3 carriers flew the 22 Oct tree in detach mode (the pair with
the slope filter and a 0.9 throttle cap, plates 11/5; three on plates 0/4/8, ema); 4 carriers flew the 7 Oct tree with
the carry planner (cable_source mocap, 60 deg cables), even ring, and its figure-8 at half the speed. Every rig cell is
one flight. Read the rig numbers as "best so far", not as a trend with fleet size; the twin panel is the like-for-like.

<details><summary>Every candidate, clock-free error (mm)</summary>

| Motion | Carriers | Rig candidates | Twin candidates (filter, path) |
|---|---|---|---|
| hover | 2 | h2slope (LAND 2 s after the top: 0 s scored); h2, h2t9p, h2m, h2rts landed or dropped before the top | R2330 1.0 (slope), R2329 1.0 (ema), R2331 1.0, R2332 1.1 |
| hover | 3 | h3 21.8, lift3_3 42.1; h3n (LAND 1.9 s after the top) | R2336 0.9 (ema), R2337 1.0, R2339 1.0, R2338 1.0 |
| hover | 4 | r20004 21.5, r002 62.1, r011 44.5 (2.9 s), r0001 102.1 (2.0 s) | R2349 1.3 (slope), R2348 1.3, R2350 1.4, R2351 1.4 |
| circle | 3 | c200_4 17.8, c200_2 41.7, c125_2 43.2 | 1 m re-fly at 0.2 m/s: R2528 7.9, R2513 8.1 |
| circle | 4 | r206e60 14.3, r006 61.7, r0005 70.9 | -- |
| figure-8 | 2 | f2slope 25.2 | size 1.0: R2334 9.9 (slope), R2335 10.2, R2333 12.0, R2360 12.3; size 0.9: R2406 8.3, R2407 8.5, R2405 10.2, R2408 10.3; size 0.75: R2424 8.3, R2423 10.8 |
| figure-8 | 3 | f3h 19.0, f125_2 27.1, f200 30.3, f3 30.6, f3big 36.8, f125 45.2 | size 1.0: R2341 9.0 (slope), R2342 9.2, R2343 11.5, R2340 11.5; size 0.9: R2410 8.9, R2411 9.0, R2409 11.9, R2412 12.1; size 0.75: R2428 10.6, R2427 13.4 |
| figure-8 | 4 | r2008e60 17.4, r0007 83.0 | size 1.0: R2353 10.0 (slope), R2354 10.1, R2355 11.9, R2352 12.4; size 0.9: R2415 10.1, R2414 10.6, R2416 12.9, R2413 13.5; size 0.75: R2432 11.6, R2431 14.6 |
| line | 3 | l200 17.7, l125 20.9 | 1 m re-fly at 0.2 m/s: R2530 1.5, R2515 1.5 |
| spinny line | 2 | -- | 1 m: R2363 5.5 (slope), R2361 7.4, R2364 7.5, R2362 8.4; 0.75 m: R2426 8.3, R2425 12.8 |
| spinny line | 3 | ls3 22.6 (older law), ls3c 36.4 (today's law: chosen) | 1 m: R2345 4.0 (slope), R2346 4.5, R2344 6.0, R2347 6.1; 0.75 m: R2430 6.1, R2429 10.4 |
| spinny line | 4 | -- | 1 m: R2357 4.0 (slope), R2358 4.1, R2356 5.3, R2359 5.8; 0.75 m: R2434 5.3, R2433 8.3 |
| spin | 3 | s125 44.5, s200_3 46.4 | 1 m re-fly at 0.125 m/s: R2512 4.5, R2527 4.5 |
| spin | 4 | r000014 85.7 | -- |

Rig flights not scored as candidates: VOIDs, flights that never lifted, the 21 Oct pairs (all ended early), the M1
attach flights, t3p_2/t3p_3 (the T-to-pair release that released the wrong quad), the 1 Oct three-quad flights (the z
integral at its bound; by the registry the paths 15-29 cm off, the hovers 3-8 cm off and up to 11 cm low).
</details>

## The figures (step 2)

Where two designs compete, both are here; **bold** is the one I would use.

| # | Figure | One-line message | Flights |
|---|---|---|---|
| 1 | **`fig_traj`** | With two, three or four carriers the ring follows the 2 m by 1 m figure-8 within about 2 cm (25, 19 and 17 mm RMS), so at print scale its path lies on the reference. | f2slope, f3h, r2008e60 (rig). Drawn about the eight's crossing. |
| 2 | **`fig_summary`** | On the rig the ring keeps within 14-36 mm of its path on every motion flown with 2, 3 or 4 carriers, and within 22 mm of its hover point; the spin is the hardest (45 and 86 mm). The twin keeps within 1-10 mm in every cell, with no trend across 2-4 carriers. | (a) the rig cells above, (b) the twin grid. Each fleet size's digit in its colour, on a log axis (the cells span 1-86 mm). |
| 2b | `fig_summary_grid` | The same numbers as a grid of cells, darker = larger on a log scale, one scale for rig and twin. | as 2. It is really a table: the thesis can carry it as one instead. |
| 3 | **`fig_components`** | On the rig the sideways part of the error is the larger on every path but the spinny line (13-25 mm against 6-26 mm of height), and the ring runs 0.05-0.23 s behind its schedule (reported apart). | Every chosen path flight but the spin (its parts are off the scale: in the table above); the twin's two spinny lines open. |
| 3b | `fig_components_eight` | Where the error builds along one lap: three carriers on the figure-8, sideways (+ inside), height and lag against the ring's position, the four tight bends shaded. | f3h. Busy; a structure story more than a result. |
| 4 | **`fig_tilt_box`** | With two carriers the ring tilts 7 deg on average (to 14 deg) about the pair's line, a rotation no cable resists; with three or four it stays under 6 deg (median about 2 deg). | f2slope, f3h, r2008e60. Boxes: quartiles, whiskers 5-95 %, dot = peak. |
| 4b | `fig_tilt` | The same along the eight: the pair rocks (0.7 Hz) between 1 and 14 deg. | as 4. |
| 5 | **`fig_throttle_box`** | No carrier reached its cap: four carriers flew at about 0.62 and three at 0.66 (cap 0.8); the pair flew at 0.72-0.75 with its cap raised to 0.9, where the default 0.8 would have left 0.01-0.05. | f2slope, f3h, r2008e60; the tracker's `thr_out` (the command sent, offset and battery term in) against `throttle_max`. |
| 5b | `fig_throttle` | The same in time, one panel per fleet size: flat lines, so it says less. | as 5. |
| 6 | **`fig_tension`** (half width) | The planner's tensions (blue) sit on the static share of the ring's weight (m g / n sin 60 deg: 4.9, 3.2 and 2.4 N) within 3 %; the pull the rig measures (red) reads 13-23 % above it. | f2slope, f3h, r2008e60. Measured pull = drone mass x the tracker's mocap cable acceleration (acceleration less the modelled thrust): it carries any thrust-map error. Rods held 58-63 deg against the planned 60. |
| 6b | `fig_tension_eight` | Three carriers in time: the plan holds 3.3 N while the measured pull wanders 3.4-4.8 N. | f3h. |
| 7 | **`fig_release`** | When one of four carriers lets go unannounced, the planner sees the cable stretch past its 6 cm threshold 0.42 s later, resizes to three 0.09 s after that (each survivor now planned at about 3.2-3.3 N), and the ring's tilt, peaking at 6.5 deg, is back in its band 1.8 s after the release. | RIG-1008-r303_4. Released carrier red, survivors blue, the ring black. Stretch computed at 50 Hz as the planner computes it (2-4 mm from its 10 Hz log). |
| 7b | `fig_reaction` | On every rig release the planner detected the loose cable in 0.33-0.42 s and resized 0.09-0.11 s later; the ring was back in band 1.8-3.8 s after the release (tilt 6.5-17.8 deg). The announced detach (r302_3) resized at once but tilted the ring to 15.4 deg. | r303_4, r304_3, r305, r302_3, in the four-carrier colour. A companion to 7 more than a rival: use both if there is room. |
| 8 | **`fig_solve`** | On the rig laptop every solve finished inside the planner's 100 ms period (median 16, 40 and 54 ms); 16 % and 29 % of the ticks passed the 60 ms budget with 3 and 4 carriers. | f2slope, f3h, r2008e60. Not one variable (4 carriers is the 7 Oct carry planner); the controlled scaling is the S bench. The ticks themselves came p95 123-152 ms apart (max 144-178). |
| 9 | **`fig_yaw`** | Through the full 360 deg turn the ring's heading runs about 0.5 s behind its reference: the error holds near -12 deg while the rate is constant (RMS 10.5 deg, at most 16 deg), about 2 deg RMS once that delay is taken out. | RIG-1022-ls3c (`tools/yaw_track.py`). ls3 (older law, 229 deg): RMS 6.6 deg, delay 0.44 s. |

**Why these components.** A results section on cooperative transport usually shows the path (1), the error and its
parts (2, 3), the payload's attitude (4), the actuators' margin (5), the internal forces against a model (6), the
reaction to a fault (7), real-time cost (8) and, where the task asks for it, the heading (9). Not drawn, worth a line
or a table: the rods' elevation against the planned 60 deg (58-63 deg on every figure-8: the cable model holds);
repeatability (every rig cell is one flight; the twin's two repeats of a cell differ by 0-2.9 mm, most under 0.5 mm);
the lift-off.

## For Wesley

1. **Designs.** My picks are in bold above: digits for the summary, the dot table for the components, boxes for tilt
   and throttle, the one-panel tension, the release storyboard (with the reaction rows as a companion). Your call on
   each pair.
2. **Wed 28.** The pair hover and every four-carrier cell are marked above. The four-carrier spinny line is twin-only:
   if Wed 28 flies one, it replaces R2357 too. The four-carrier release figures (7, 7b) stay on 8 Oct's flights unless
   a 1 m release flies.

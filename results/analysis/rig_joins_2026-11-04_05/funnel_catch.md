# Funnel catch count: 4-5 Nov plate descents (geometry only, exploratory)

Made by `funnel_catch.py` from `joins.csv`, `flights.csv`, the bags and `cad/funnel/funnel.py` (P, VARIANTS, mouth(); read, not run). Per-descent numbers are in `funnel_catch.csv`.

| variant | mouth a x b (half-axes, mm) | mouth plane above the ring top H (mm) | fully-inside ellipse (mm) | tip height above the weld point at the mouth plane (mm) |
|---|---|---|---|---|
| v1 | 107 x 82 | 85.6 | 75 x 50 | 68.6 |
| v2 | 87 x 72 | 75.0 | 55 x 40 | 58.0 |
| v3 | 104 x 78 | 85.0 | 72 x 46 | 68.0 |

The disc's bottom sits 17 mm above the ring top when seated, so it reaches the mouth plane when the tip is H - 17 mm above its welded position. The height is measured along the ring's normal. At the first sample at or below that height the tip's offset from the weld point is scored. "Into the mouth" means the disc centre is inside the a x b ellipse. "Fully inside" means it is inside the ellipse shrunk by the disc's 32 mm radius. **Ring frame** (the main reading) measures the offsets in the ring's own plane, which the funnel turns with. **Horizontal** is the joins table's reading. The two differ only where the ring leaned: sq8_2 at 29 deg, the rest at 0-8 deg.

## Counts (scored descents: those that came down through the mouth plane)

| descents | v1 into mouth | v1 fully inside | v1 (horizontal) mouth / inside | v2 into mouth | v2 fully inside | v2 (horizontal) mouth / inside | v3 into mouth | v3 fully inside | v3 (horizontal) mouth / inside |
|---|---|---|---|---|---|---|---|---|---|
| all | 30/32 | 22/32 | 31/32 / 22/32 | 27/32 | 16/32 | 27/32 / 15/32 | 29/32 | 19/32 | 30/32 / 18/32 |
| 4 Nov | 11/11 | 9/11 | 11/11 / 9/11 | 11/11 | 6/11 | 11/11 / 6/11 | 11/11 | 8/11 | 11/11 / 8/11 |
| 5 Nov | 19/21 | 13/21 | 20/21 / 13/21 | 16/21 | 10/21 | 16/21 / 9/21 | 18/21 | 11/21 | 19/21 / 10/21 |
| triangle | 29/31 | 21/31 | 30/31 / 21/31 | 26/31 | 15/31 | 26/31 / 14/31 | 28/31 | 18/31 | 29/31 / 17/31 |
| square (plate 8) | 1/1 | 1/1 | 1/1 / 1/1 | 1/1 | 1/1 | 1/1 / 1/1 | 1/1 | 1/1 | 1/1 / 1/1 |
| triangle, plate 9 | 22/24 | 16/24 | 23/24 / 16/24 | 20/24 | 11/24 | 20/24 / 10/24 | 21/24 | 14/24 | 22/24 / 14/24 |
| triangle, plate 9, 4 Nov | 10/10 | 9/10 | 10/10 / 9/10 | 10/10 | 6/10 | 10/10 / 6/10 | 10/10 | 8/10 | 10/10 / 8/10 |
| triangle, plate 9, 5 Nov | 12/14 | 7/14 | 13/14 / 7/14 | 10/14 | 5/14 | 10/14 / 4/14 | 11/14 | 6/14 | 12/14 / 6/14 |
| triangle, plate 1 | 7/7 | 5/7 | 7/7 / 5/7 | 6/7 | 4/7 | 6/7 / 4/7 | 7/7 | 4/7 | 7/7 / 3/7 |

## Against the earlier count (a lost script, 17 descents)

The earlier set is taken to be the 16 triangle descents of 5 Nov that reached plate height (the notes' 16) plus sq8_2's catch: 17 descents here.

| reading | v1 into mouth | v1 fully inside | v2 into mouth | v2 fully inside |
|---|---|---|---|---|
| earlier count | 10/17 | 5/17 | 8/17 | 0/17 |
| this script, ring frame | 14/15 | 9/15 | 12/15 | 8/15 | 13/15 | 8/15 |
| this script, horizontal | 14/15 | 9/15 | 12/15 | 8/15 | 14/15 | 7/15 |
| ring frame, re-descents read at their start | 15/17 | 9/17 | 12/17 | 8/17 | 14/17 | 8/17 |
| joins table: offsets at plate height | 15/17 | 8/17 | 11/17 | 5/17 | 14/17 | 7/17 |
| joins table: offsets at the lowest point | 13/17 | 5/17 | 10/17 | 1/17 | 13/17 | 3/17 |

**It does not agree.** At the mouth plane this script finds more catches than the earlier count, for both variants and both scores. Reading the ring frame or the horizontal makes no difference here. No reading tried reproduces 10/5/8/0. The nearest is the lowest point of each descent (where a magnet that slid off the plate ended up), so the lost script probably scored that instant rather than the mouth plane. The question for the funnel is whether the disc came down through the mouth, and that is the mouth-plane reading. Two descents in this set (nr1r_3 #2 and #4) are re-descents after a failed proof that start below v1's plane. They are scored at their first sample only in the last ring-frame row.

## Not scored

| day | flight | plate | # | from | outcome | tip height at the descent's start (mm) | lowest (mm) | v1 | v2 | v3 |
|---|---|---|---|---|---|---|---|---|---|---|
| 04 | sq6b | 9 | 1 | MATCH_VELOCITY | aborted | 283 | 261 | never came down to the mouth | never came down to the mouth | never came down to the mouth |
| 04 | sq6m | 9 | 2 | LIFT_PROOF | caught+proven | 1 | 1 | started below the mouth | started below the mouth | started below the mouth |
| 04 | nr1_2 | 1 | 2 | LIFT_PROOF | contact not proven | 6 | 5 | started below the mouth | started below the mouth | started below the mouth |
| 04 | nr1_2 | 1 | 3 | LIFT_PROOF | contact not proven | 6 | 5 | started below the mouth | started below the mouth | started below the mouth |
| 04 | nr9m | 9 | 2 | MATCH_VELOCITY | miss | -25 | -92 | started below the mouth | started below the mouth | started below the mouth |
| 04 | nr9m_4 | 9 | 2 | LIFT_PROOF | miss | 40 | -9 | started below the mouth | started below the mouth | started below the mouth |
| 04 | nr9m_4 | 9 | 4 | LIFT_PROOF | miss | 45 | -24 | started below the mouth | started below the mouth | started below the mouth |
| 04 | nr9m_5 | 9 | 2 | LIFT_PROOF | miss | 21 | -32 | started below the mouth | started below the mouth | started below the mouth |
| 04 | nr9m_5 | 9 | 4 | LIFT_PROOF | miss | 20 | -24 | started below the mouth | started below the mouth | started below the mouth |
| 05 | nr1r_3 | 1 | 2 | LIFT_PROOF | miss | 53 | -33 | started below the mouth | started below the mouth | started below the mouth |
| 05 | nr1r_3 | 1 | 4 | LIFT_PROOF | miss | -10 | -26 | started below the mouth | started below the mouth | started below the mouth |
| 05 | sq8_2 | 8 | 1 | MATCH_VELOCITY | aborted | 284 | 131 | never came down to the mouth | never came down to the mouth | never came down to the mouth |
| 05 | nr9o_2 | 9 | 2 | MATCH_VELOCITY | aborted | 163 | 133 | never came down to the mouth | never came down to the mouth | never came down to the mouth |
| 05 | nr9o_2 | 9 | 3 | MATCH_VELOCITY | aborted | 311 | 140 | never came down to the mouth | never came down to the mouth | never came down to the mouth |

"Started below the mouth": most are re-descents after a failed proof (from LIFT_PROOF), where the magnet had lifted only centimetres. With a funnel the disc would still be in the cone, so they are not a new entry. Some of these began below the plate because the magnet slid off during the failed proof (5 Nov nr1r_3 #4 at -10 mm). One is not a re-descent: 4 Nov nr9m #2 (from MATCH) began at -25 mm, with the magnet still below the plate beside it after the previous miss; that retry never lifted it clear. "Never came down to the mouth" rows were aborted higher up.

## Every scored descent (ring frame, mm; Y = yes)

| day | flight | plate | # | outcome | miss at plate height (cm, joins) | v1 dr | v1 dt | v1 mouth | v1 inside | v2 dr | v2 dt | v2 mouth | v2 inside | v3 dr | v3 dt | v3 mouth | v3 inside | ring tilt at the mouth plane (deg) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 04 | sq6m | 9 * | 1 | contact not proven | 2.8 | +20 | -15 | Y | Y | +28 | -8 | Y | Y | +21 | -14 | Y | Y | 3.5 |
| 04 | nr1_2 | 1 | 1 | contact not proven | 0.4 | -51 | +46 | Y | - | -46 | +45 | Y | - | -51 | +46 | Y | - | 2.3 |
| 04 | nr9m | 9 * | 1 | miss | 6.2 | -42 | +35 | Y | Y | -48 | +48 | Y | - | -42 | +35 | Y | Y | 8.7 |
| 04 | nr9m | 9 * | 3 | miss | 6.6 | +22 | -13 | Y | Y | +35 | -9 | Y | Y | +23 | -13 | Y | Y | 6.8 |
| 04 | nr9m | 9 * | 4 | aborted | 1.8 | -4 | +8 | Y | Y | -7 | +0 | Y | Y | -4 | +8 | Y | Y | 5.1 |
| 04 | nr9m_4 | 9 * | 1 | contact not proven | 3.0 | -54 | +11 | Y | Y | -46 | +17 | Y | Y | -54 | +11 | Y | Y | 0.9 |
| 04 | nr9m_4 | 9 * | 3 | contact not proven | 2.4 | -20 | -12 | Y | Y | -18 | -8 | Y | Y | -20 | -12 | Y | Y | 0.6 |
| 04 | nr9m_4 | 9 * | 5 | contact not proven | 5.9 | +41 | +0 | Y | Y | +41 | +1 | Y | Y | +41 | +0 | Y | Y | 1.7 |
| 04 | nr9m_5 | 9 * | 1 | contact not proven | 6.2 | -74 | -10 | Y | Y | -63 | -16 | Y | - | -73 | -10 | Y | - | 3.9 |
| 04 | nr9m_5 | 9 * | 3 | contact not proven | 5.4 | +36 | -31 | Y | Y | +41 | -31 | Y | - | +37 | -31 | Y | Y | 3.5 |
| 04 | nr9m_5 | 9 * | 5 | contact not proven | 7.6 | +23 | -59 | Y | - | +20 | -64 | Y | - | +23 | -59 | Y | - | 0.3 |
| 05 | fm9y_2 | 9 * | 1 | miss | 5.7 | -109 | -6 | - | - | -84 | -26 | - | - | -109 | -6 | - | - | 2.9 |
| 05 | fm9y_2 | 9 * | 2 | miss | 10.7 | -10 | -38 | Y | Y | -4 | -38 | Y | Y | -10 | -38 | Y | Y | 2.1 |
| 05 | fm9y_2 | 9 * | 3 | miss | 5.1 | +61 | -22 | Y | Y | +63 | -0 | Y | - | +61 | -19 | Y | Y | 1.7 |
| 05 | nr1r_3 | 1 | 1 | contact not proven | 2.3 | +5 | +8 | Y | Y | +5 | +11 | Y | Y | +5 | +9 | Y | Y | 2.0 |
| 05 | nr1r_3 | 1 | 3 | contact not proven | 0.9 | -34 | -51 | Y | - | -28 | -43 | Y | - | -34 | -51 | Y | - | 2.1 |
| 05 | nr1r_3 | 1 | 5 | miss | 9.1 | +40 | -31 | Y | Y | +39 | -17 | Y | Y | +40 | -29 | Y | Y | 2.2 |
| 05 | nr1r_3 | 1 | 6 | miss | 3.4 | -17 | +35 | Y | Y | -14 | +33 | Y | Y | -17 | +35 | Y | Y | 5.6 |
| 05 | nr1r_3 | 1 | 7 | miss | 3.4 | -71 | +7 | Y | Y | -32 | +27 | Y | Y | -70 | +9 | Y | Y | 2.6 |
| 05 | nr1r_3 | 1 | 8 | aborted | 15.7 | -57 | +29 | Y | Y | -84 | +36 | - | - | -57 | +29 | Y | - | 3.6 |
| 05 | sq8_2 | 8 | 2 | caught+proven | 1.6 | -40 | -28 | Y | Y | -43 | -9 | Y | Y | -41 | -27 | Y | Y | 28.6 |
| 05 | nr9o_2 | 9 * | 1 | miss | 8.3 | -26 | +47 | Y | Y | -23 | -8 | Y | Y | -27 | +46 | Y | - | 2.4 |
| 05 | nr9o_2 | 9 * | 4 | miss | 8.9 | -26 | +14 | Y | Y | -19 | +13 | Y | Y | -25 | +14 | Y | Y | 3.6 |
| 05 | nr9m | 9 * | 1 | miss | 6.2 | -39 | -16 | Y | Y | -27 | -6 | Y | Y | -39 | -16 | Y | Y | 4.9 |
| 05 | nr9m | 9 * | 2 | miss | 3.9 | +17 | +41 | Y | Y | +7 | +43 | Y | - | +16 | +41 | Y | Y | 2.3 |
| 05 | nr9m | 9 * | 3 | miss | 5.5 | +53 | -67 | Y | - | +71 | -64 | - | - | +57 | -66 | - | - | 1.3 |
| 05 | nr9m | 9 * | 4 | miss | 8.0 | +35 | +71 | Y | - | -20 | +16 | Y | Y | +35 | +71 | Y | - | 2.4 |
| 05 | nr9m | 9 * | 5 | miss | 7.5 | -27 | +10 | Y | Y | -62 | +26 | Y | - | -28 | +11 | Y | Y | 0.9 |
| 05 | nr9m | 9 * | 6 | miss | 3.1 | -23 | -55 | Y | - | -19 | -73 | - | - | -23 | -56 | Y | - | 5.0 |
| 05 | nr9m | 9 * | 7 | miss | 7.1 | +30 | -57 | Y | - | +35 | -43 | Y | - | +30 | -57 | Y | - | 1.0 |
| 05 | nr9m | 9 * | 8 | miss | 9.0 | -83 | -10 | Y | - | -77 | -17 | Y | - | -82 | -11 | Y | - | 2.9 |
| 05 | nr9m | 9 * | 9 | miss | 6.7 | -112 | +46 | - | - | -112 | +41 | - | - | -112 | +45 | - | - | 1.3 |

`*` = plate 9 on the triangle, the funnel's intended place.

## Caveats

1. **Geometry only.** A real cone changes the descent. The disc slides on the wall toward the bore, and the swinging magnet can hit the rim and bounce. QUAD4's controller reacts to all of this. A descent scored "into the mouth" is one whose disc would meet the cone, not a predicted catch. One scored outside would land on the rim or beside the funnel, not on the plate.
2. **The mouth plane is read with the ring's own tilt.** The height and (main reading) the offsets are along the ring's normal and in its plane, as the funnel is bolted to it. sq8_2 leaned 29 deg (the T), so its horizontal and ring-frame offsets differ by about H x sin(29 deg). The rest leaned 0-8 deg.
3. **The disc is the tip marker.** Body 24's point is 22.6 mm above the magnet face, about the disc's height. Its welded position is taken as the disc seated (17 mm above the ring top), and its xy as the disc centre. A tilted holder (the magnet swinging on its 1 m cable) puts the real disc edge a few mm off that.
4. **The weld point** is the one each flight was scored against in joins.csv (5 Nov: 0.0, 0.2293, 0.0491; 4 Nov: the typed 0.0039, 0.2271, 0.0365, uncertain by about 1 cm in height, see summary.md). Plates 1 and 8 use plate 9's point turned. A 1 cm height error moves the mouth-plane instant by about 0.1-0.2 s.
5. Exploratory: the descents are the ones flown without a funnel, and no bars apply.


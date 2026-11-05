# Why the integral is needed, from the real flights (31 Oct 2026)

Offline and exploratory, one pass, no card, not reviewed, nothing flown or run in Gazebo. It answers Wesley: "why is
the integral even needed at all? ... either the planner is wrong or the trackers." Scripts:
`results/analysis/why_integral_2026-10-31/`
All numbers come from flights before Motive's recalibration of 30 Oct (09:40). No steady hover has flown since.

## Verdict

**Partly the planner, not the trackers.** On today's ring the sideways force stands in for two things, and neither is
a force the planner should need an integral for. The height integral stands in for one of them and, on the measured
pull, does nothing.

| # | The force stands in for | Moves with | New ring, N | Old ring, N | How sure |
|---|---|---|---|---|---|
| 1 | Motive's Z about 1 deg off true vertical | the room | -0.17, +0.10 | -0.13, +0.07 | section 3a |
| 2 | Cable lengths the planner latches too short | each hand-over | y -0.31 to -0.14 | up to 0.15 | high |
| 3 | The ring's Motive pivot off its centre | the ring | fixed | -0.32, +0.45 to 21 Oct | high, fixed |
| 4 | Not explained, flight to flight | | sd 0.12 | sd 0.2 | |

Part 1, how sure: high that a room-fixed force exists, medium that it is the frame and not air.

| Height | Value | Section |
|---|---|---|
| Ring height error against the stale length | 4.4 x, R2 0.85, 50 windows (1 m: 4.15 x, R2 0.93) | 5 |
| Height force needed to move the ring | none seen: 8 flights moved it 0.5-1.8 N, ring -4.4 to +5.9 mm per N | 5 |
| The height force reaches its bound | sq3 at 21.7 s, -2.14 N, ring still 37 mm low | 2 |

| Size against a yardstick | N |
|---|---|
| The integral's bound, sideways | 1.58 |
| 1 deg lean on 10 N of thrust | 0.17 |
| 1.15 kg ring's weight x sin(1 deg), the frame part, 1.8 % of the ring's weight | 0.20 |
| Learned force, new ring, windows of 8 s or more, rms | 0.30 |
| Left after parts 1 and 2 are taken off, rms (67 % of F squared removed) | 0.17 |

**What the logs show:** every number in the tables, except those marked "loop". **What is inferred:** that the
room-fixed force is the frame and not air (section 3a), and the new ring's split between parts 1 and 2, which uses the
offline loop of section 5. The new ring has never flown at another yaw, so on it the room and the ring cannot be told
apart (section 8).

## 1. What the force is, and the steady rule

- The reference is the ring's own position when the planner first sees it (`planner_node.py`, `hover_xy`). A Motive
  origin error cannot make an offset.
- `F = -k b`, where `b` integrates `ref - measured` at 0.40 /s per axis, bound 0.30 m. Sideways it runs only while the
  ring is under 0.05 m/s and 0.25 m from its point, the reference at rest, every cable taut. Its gate was open 28-100
  %.
- The force enters the model and the cable-direction reference (`reference_builder.py:244`). It leans the formation.
  With no real force to meet, the ring tilts instead (section 5).
- K is the ring's sideways stiffness to a force, from the offline loop (section 5). tau = K / (k x 0.40) is the
  integral's time constant. The card's 5.26 N/m came from the twin at 0.55 m and 45 deg.

| Setup | K, N/m | tau, s | A 5 s hold is |
|---|---|---|---|
| 1 m, 1.15 kg, 3 quads | 7.3 | 3.5 | 1.4 tau |
| 1 m, 1.15 kg, 4 quads | 9.6 | 4.5 | 1.1 tau |
| 0.55 m, 0.86 kg, 3 quads | 12.9 | 6.1 | 0.8 tau |
| 0.55 m, 0.86 kg, 4 quads | 18.2 | 8.6 | 0.6 tau |
| 0.55 m, 0.86 kg, 2 quads | 12.4 | 5.9 | 0.8 tau |

| Rule | Value |
|---|---|
| Window starts | 2 s after the height reference tops out, later if the ring is out of band |
| Window ends | 0.5 s before LAND, DROP, DETACH, DETECTED, UNLOAD, magnet OFF, abort, disarm, descent or path start |
| Ring band | within 0.10 m of the top reference |
| QUAD4 | an attach flight's window ends 0.5 s before QUAD4 comes within 1.0 m of the ring sideways |
| Length | 3 s or more is tabled, 1-3 s is short and not a measurement |
| F, offsets | means over the window's last half |
| Settled, integral on | gate open at least 2 tau since the top, F moving under 0.03 N/s, mean offset under 15 mm |
| Settled, integral off | 3 s or more |
| F-inf | F + K x offset, what the integral would reach, used only to compare unsettled windows |

| 217 flight folders | Count |
|---|---|
| Ring never lifted, void or not flown | 110 |
| Never within the band at the top (30 Sep's lifts, 1 Oct's sag, h3tp) | 7 |
| Under 1 s from settle to first event | 36 |
| Short, 1-3 s | 27 |
| 3 s or more, tabled | 37 |

Short, left out: ch1b, f2a, f2b, f2c, f2slope, f3h, f4, h1, h4f_3, h6, hS1, ls2a, ls2b_4, ls2b_5, ls2c, ls3, ls3c,
r0001, r0007, r006, r011, r200006, r2008e60, r206e60, r304_3, r305, t6h5_3. Every path flight with a 5 s hold is among
them.

## 2. Every usable steady hover

Fx, Fy, Fz: the integral's force in N, world frame. Ex, Ey, Ez: ring minus its reference in mm. Tilt: the ring's body
z leaning toward +x, +y, deg. `off`: the sideways integral was off (`int_k_xy` 0 or reference mode), so Ex, Ey is the
raw offset. `ref`: height integral on the target. `*`: trackers flew the planned pull. Set: yes, or why not: g gate
open too little, F still moving, E offset over 15 mm. Yaw: the ring's. V: pack under load. Plates in today's clockwise
numbering.

**1 m cables, 1.15 kg ring** (29 Oct is the only day with a steady hover on it)

| Date | Flight | Quads on plates | s | Yaw | V | Fx | Fy | Fz | Ex | Ey | Ez | Tilt x | Tilt y | Set |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 10-29 | k1_4 | Q1:3 Q2:11 Q3:7 | 25.8 | -2 | 22.1 | -0.20 | -0.44 | -1.24 | -5 | +2 | -26 | +0.8 | +4.9 | yes |
| 10-29 | k1_8 | Q1:3 Q2:11 Q3:7 | 35.0 | -2 | 22.9 | -0.25 | -0.29 | -1.33 | -8 | +5 | -22 | +0.2 | +4.0 | yes |
| 10-29 | sq2_5 | Q1:3 Q2:11 Q3:7 | 26.6 | +3 | 22.9 | -0.29 | +0.01 | -0.59 | -6 | +2 | -16 | +1.0 | +2.1 | yes |
| 10-29 | sq3 | Q1:3 Q2:11 Q3:7 | 36.3 | +3 | 23.3 | -0.05 | -0.17 | -2.13 | -3 | +1 | -37 | -0.2 | +3.8 | yes |
| 10-29 | sq4_5 | Q1:3 Q2:11 Q3:7 | 12.0 | -3 | 22.4 | -0.20 | +0.14 | -0.73 | -2 | -1 | -21 | +0.7 | +0.9 | yes |
| 10-29 | sq5_2 | Q1:3 Q2:11 Q3:7 | 8.2 | -6 | 23.4 | -0.09 | -0.07 | -0.45 | +0 | +7 | -21 | +1.5 | +2.8 | g |
| 10-29 | ch1b_2 | Q1:0 Q2:3 Q3:6 Q4:9 | 12.1 | +1 | 23.0 | off | off | -0.83 | -19 | +2 | -23 | -1.5 | +0.7 | yes |

**0.55 m cables, 0.86 kg ring, after the pivot fix** (22 and 28 Oct)

| Date | Flight | Quads on plates | s | Yaw | V | Fx | Fy | Fz | Ex | Ey | Ez | Tilt x | Tilt y | Set |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 10-28 | hS2 | Q2:11 Q3:5 | 6.2 | -31 | 22.4 | -0.28 | +0.17 | -0.27 | -18 | -4 | -12 | +0.6 | -1.6 | gFE |
| 10-22 | h3 | Q2:0 Q4:8 Q3:4 | 6.3 | +1 | 23.3 | -0.29 | -0.09 | -0.21 | -13 | -8 | -10 | +0.7 | +0.4 | gE |
| 10-22 | t3p_2 | Q3:2 Q2:11 Q4:5 | 44.2 | -26 | 23.5 | off | off | -0.85 | -39 | -78 | -6 | -0.9 | +0.8 | yes |

**0.55 m cables, 0.86 kg ring, before the pivot fix** (1-21 Oct)

| Date | Flight | Quads on plates | s | Yaw | V | Fx | Fy | Fz | Ex | Ey | Ez | Tilt x | Tilt y | Set |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 10-21 | c2q | Q2:0 Q3:6 | 7.6 | +180 | 22.0 | +0.63 | -0.13 | -0.26 | +26 | +4 | -16 | -2.0 | +0.4 | gFE |
| 10-01 | h5* | Q2:0 Q4:8 Q1:4 | 3.1 | +123 | 22.7 | off | off | ref | -112 | +3 | -75 | +1.5 | -4.3 | yes |
| 10-14 | lift3_3 | Q3:3 Q2:11 Q1:7 | 7.3 | +2 | 22.4 | -0.23 | +0.16 | -0.02 | -32 | +13 | -1 | -0.3 | +0.2 | gFE |
| 10-14 | t5_4 | Q3:3 Q2:11 Q1:7 | 55.6 | +6 | 22.7 | -0.18 | +0.00 | -0.71 | +3 | -3 | -5 | +1.4 | -0.8 | yes |
| 10-14 | t5_5 | Q3:3 Q2:11 Q1:7 | 86.7 | -13 | 23.2 | -0.64 | +0.36 | -1.51 | -1 | +0 | -8 | +3.0 | -2.5 | yes |
| 10-14 | t6 | Q3:3 Q2:11 Q1:7 | 48.9 | -7 | 23.4 | -0.49 | +0.41 | -1.03 | -1 | +6 | -8 | +2.2 | -1.8 | yes |
| 10-14 | t6f | Q3:3 Q2:11 Q1:7 | 7.5 | -0 | 22.7 | -0.41 | +0.22 | -0.09 | -10 | +6 | -0 | +1.5 | -1.1 | g |
| 10-14 | t6f2_4 | Q3:3 Q2:11 Q1:7 | 9.1 | +3 | 23.9 | -0.68 | +0.26 | -0.45 | -21 | +2 | -13 | +3.3 | +0.2 | gFE |
| 10-14 | t6g_2 | Q3:3 Q2:11 Q1:7 | 6.0 | +4 | 22.6 | -0.36 | +0.24 | -0.18 | -17 | +9 | -6 | +1.6 | -0.6 | gFE |
| 10-14 | t6h_4 | Q3:3 Q2:11 Q1:7 | 19.2 | +2 | 23.7 | -0.80 | +0.34 | -0.94 | -15 | +12 | -16 | +4.1 | -0.2 | E |
| 10-15 | a1 | Q1:3 Q3:11 Q2:7 | 10.4 | -19 | 23.5 | -0.25 | +0.46 | -0.16 | -3 | +3 | -1 | +0.0 | -1.7 | g |
| 10-15 | a2_2 | Q1:3 Q3:11 Q2:7 | 12.0 | +3 | 22.1 | -0.32 | +0.30 | -0.14 | -8 | +7 | -3 | +1.6 | -0.4 | g |
| 10-15 | a3_2 | Q3:3 Q2:11 Q1:7 | 9.6 | +3 | 23.6 | -0.38 | +0.19 | -0.19 | -5 | +0 | -5 | +1.5 | -1.0 | g |
| 10-21 | c200_4 | Q2:0 Q4:8 Q3:4 | 7.7 | +178 | 23.2 | +0.33 | -0.42 | -0.09 | +9 | -12 | -2 | -1.1 | +2.1 | g |
| 10-21 | f125_2 | Q2:0 Q4:8 Q3:4 | 7.6 | +3 | 22.2 | -0.71 | +0.39 | -0.51 | -22 | +17 | -19 | +3.3 | -0.9 | gFE |
| 10-21 | f200 | Q2:0 Q4:8 Q3:4 | 7.7 | +1 | 22.8 | -0.56 | +0.42 | -0.40 | -22 | +22 | -15 | +2.6 | -1.0 | gFE |
| 10-21 | l125 | Q2:0 Q4:8 Q3:4 | 7.7 | +3 | 23.6 | -0.65 | +0.36 | -0.46 | -14 | +14 | -17 | +3.5 | -1.1 | gFE |
| 10-21 | l200 | Q2:0 Q4:8 Q3:4 | 7.6 | +4 | 22.7 | -0.55 | +0.42 | -0.23 | -14 | +14 | -9 | +1.7 | -2.2 | gFE |
| 10-21 | s125 | Q2:0 Q4:8 Q3:4 | 7.6 | -8 | 22.5 | -0.25 | +0.45 | -0.33 | -8 | +12 | -13 | -0.3 | -2.5 | g |
| 10-21 | s200_3 | Q2:0 Q4:8 Q3:4 | 7.6 | -2 | 23.4 | -0.26 | +0.19 | -0.20 | -17 | +9 | -10 | -0.7 | -0.4 | gFE |
| 10-07 | r000014* | Q2:0 Q4:9 Q3:6 Q1:3 | 7.7 | -2 | 22.4 | -0.29 | +0.34 | -1.02 | +4 | -1 | -8 | +1.5 | +2.3 | g |
| 10-07 | r0012* | Q2:11 Q4:9 Q3:7 Q1:3 | 6.1 | -4 | 23.5 | off | off | ref | -61 | +20 | -27 | +0.2 | +1.9 | yes |
| 10-07 | r0013* | Q2:11 Q4:9 Q3:6 Q1:3 | 3.6 | -3 | 22.8 | off | off | ref | -65 | +81 | -15 | -1.1 | +2.4 | yes |
| 10-07 | r002* | Q1:0 Q4:9 Q3:6 Q2:3 | 14.5 | -5 | 23.0 | -0.28 | -0.15 | -0.75 | -2 | -28 | +17 | +3.8 | +1.8 | gE |
| 10-07 | r20004 | Q4:0 Q1:9 Q3:6 Q2:3 | 5.0 | -2 | 23.9 | -0.23 | +0.18 | -0.09 | -13 | +12 | -2 | -0.2 | -0.8 | gE |
| 10-08 | r302_3 | Q2:11 Q4:9 Q3:7 Q1:3 | 4.6 | -0 | 23.9 | off | off | ref | -47 | +42 | +1 | -4.2 | +1.0 | yes |
| 10-08 | r303_4 | Q2:11 Q4:9 Q3:7 Q1:3 | 3.1 | +0 | 23.0 | off | off | ref | -47 | +29 | -1 | -2.2 | +0.6 | yes |

Settled with the integral on, 8 in all: t5_4, t5_5, t6, k1_4, k1_8, sq2_5, sq3, sq4_5. No flight after the pivot fix
on the old ring settles. Those windows measure only the start of the force.

## 3. What the force moves with

### 3a. Fixed to the room

| Reading of true up, in Motive's frame | x, y, deg | Size at direction | Flights |
|---|---|---|---|
| Ring at rest, its body offset fitted out | +0.80, -0.46 (+-0.03) | 0.93 at -30 | 118 rests, yaw -176 to +179 |
| Bench, ring turned in tape marks | | 0.9 at -20 | RIG-1021-ringbench |
| Free-flying quads, each quad's offset fitted out | +0.94, -0.57 (+-0.27) | 1.10 at -31 | 16 windows, Q1-Q4 |
| QUAD4 alone, no ring, yaw 0 to 85 | +1.09, -0.63 (+-0.31) | 1.26 at -30 | 12 windows, 14-29 Oct |

The +- are fit errors, and rests of one placement are not independent. The floor and the hovering quads agree in size
and direction, and a hovering quad knows nothing about the floor. If true up leans 1 deg toward (+x, -y) in Motive's
frame, gravity pushes everything toward 150 deg world. The ring feels `M g sin(1 deg)`: 0.20 N at 1.15 kg, 0.15 N at
0.86 kg. That is the direction of the old "-x push".

The deciding flights are the one-turn spins. The ring turns under a held force, so a room-fixed force shows in the
mean offset over the turn and a ring-fixed one turns with the ring. World F = held F + K x mean offset. Turning F = K
x the offset's amplitude at the ring's yaw. A first try read the constant part of the planner's pull sum, which only
echoed the held force, so it was dropped before these numbers were seen.

| Flight | Turn, deg | Held F, N | Mean offset, mm | World F, N | As a tilt | Turning F, N |
|---|---|---|---|---|---|---|
| s125, before the fix | 344 | -0.29, +0.52 | +12, -33 | -0.13, +0.10 | 1.11 deg | -0.33, +0.47 |
| s200_3, before the fix | 348 | -0.33, +0.23 | +17, -8 | -0.11, +0.12 | 1.11 deg | -0.41, +0.52 |
| ls3c, after the fix | 316 | -0.07, -0.15 | -2, +12 | -0.09, 0.00 | 0.61 deg | -0.05, -0.03 |
| ls3, after the fix | 192 | -0.23, -0.09 | +13, +11 | -0.06, +0.05 | 0.53 deg | partial turn |
| Predicted: 1 deg at 150 deg | | | | -0.13, +0.07 | 1.00 deg | |
| Predicted: the bench's pivot x the loop's 0.32 N/cm | | | | | | -0.32, +0.45 |

Both predictions come from readings taken on the floor, on the bench or in free flight, and were written down before
this table was computed. The four pair spins of 28 Oct scatter too much to use (world -0.32, +0.10 to +0.10, -0.24):
pairs rock.

| Same ring and day, turned 180 deg (21 Oct, before the fix) | Fx, Fy after the stale part, N |
|---|---|
| Six triangles at yaw -8 to +4 | -0.63, +0.50 |
| c200_4 at yaw +178 | +0.41, -0.49 |
| Half the sum: fixed to the room | -0.11, 0.00 |
| Half the difference: fixed to the ring (bench pivot predicts -0.32, +0.45) | -0.52, +0.49 |

| Other check | Reading | Worth |
|---|---|---|
| 1 Oct, whole system turned 124 deg (h5) | offset -104, +10 mm, still toward -x | weak: planned pull, ring low |
| 1 Oct, moved to x +1.1 m (h6) | offset -23, +4 mm, 1.5 s | short |
| New ring at y -0.6 m (k1_4, k1_8), others near y 0 | y left -0.05 against +0.19 N | 5 flights, QUAD4 moved too |
| Motive origin | the reference is the ring's own start | cannot matter, from the code |

Room air is also fixed to the room, and the logs cannot split it from a tilted frame. For the frame: the floor reads
the same tilt as the quads. A ground plane set on that floor should read it as zero, and why it does not is not known.
Against: nobody has put a spirit level on that floor. It was on the 22 Oct sheet (`docs/rig/thu_2026-10-22.md`), no
result recorded.

### 3b. Fixed to the ring or the layout

| Candidate | Numbers | Verdict |
|---|---|---|
| Ring's pivot, before 21 Oct | spin -0.33, +0.47 and -0.41, +0.52 N, bench -0.32, +0.45 N | yes, fixed 21-22 Oct |
| Same, after | ls3c -0.05, -0.03 N | gone on the old ring |
| Same, new ring | bench 5.1 mm (RIG-1028-payload) would be -0.07, -0.11 N | small, not separated |
| Attach points, against tip markers | tips' centre within 3.5 mm, table below | no |
| Plate heights | tip heights 25.5-32.1 mm, plate 6 highest | inside the lengths, small |
| Ring's real centre of mass | loop: 0.35 N per cm on the new ring | not decided, section 8 |
| T, 0.55 m (t3p_2, integral off) | raw -39, -78 mm. Rider pulls 0.46 N, plan 0.12 N | layout-fixed, a real pull |
| 1/3/5/9, 0.55 m (8 Oct, raw) | -47, +29 to +42 mm. Pivot + room over K: -30, +36 mm | no new cause needed |

| Flight | Tips' centre, ring frame, mm | Model's pull sum, N | Pull sum along the tips, N |
|---|---|---|---|
| h3 | +0.7, +1.1 | -0.294, -0.056 | -0.270, -0.040 |
| h4f_3 | -0.6, -3.5 | -0.142, +0.107 | -0.139, +0.068 |
| f4 | +0.8, -3.5 | -0.174, +0.148 | -0.160, +0.105 |

### 3c. Fixed to a quad

| Quad | Thrust axis against body z, 28-30 Oct, deg | Same, 7-15 Oct, deg | Cable top, from length against yaw, mm |
|---|---|---|---|
| Q1 | +0.38, -0.40 | +0.49, -0.44 | +1.5, -2.6 (+-2.6) |
| Q2 | +0.82, -0.09 | +0.58, -0.18 | +4.7, +1.0 (+-1.9) |
| Q3 | -0.20, +2.77 | +0.97, +1.91 | +10.0, -0.6 (+-3.3) |
| Q4 | -0.76, +0.43 | -1.04, -0.03 | +6.5, +10.8 (+-3.1) |

- Q3's axis is 2.1-2.8 deg off its Motive body, about 0.49 N at 10 N of thrust. On 29 Oct the same triangle shows a
  team thrust tilt from 0.09 to 2.29 deg as Q3's heading goes from +57 to -119 deg, more than 1 N in the sum. The
  ring's force does not swing with it (left over within +-0.2 N). So a quad's axis error does not reach the ring
  [inferred: the measured pull is computed with the same wrong axis and absorbs it].
- So the old thrust-sum replay (-0.41 to -0.52 N, 1 Oct, B4 card) is not a clean reading of the room force.
- Quads swapped on plates, same day, F-inf: a2_2 (Q1:3 Q3:11 Q2:7) -0.42, +0.40 N. a3_2 (Q3:3 Q2:11 Q1:7) -0.45, +0.19
  N.
- A cable top 1 cm off is worth 0.04-0.06 N (T/L x 0.9). No quad's offset is much over 1 cm.

**Verdict: no.** A quad's error is real and large in its own lean. It stays with that quad.

### 3d. Fixed to the team's own flow

| Test | Numbers | Reading |
|---|---|---|
| Even layouts | 2, 3 and 4 quads learn the same direction | no net sideways wash visible |
| QUAD4 far, then over the ring, 1 m | sq2_5 -0.24, +0.07 then -0.30, +0.10 N | up to 0.1 N |
| Same | sq5_2 -0.08, +0.02 then -0.18, +0.08 N | |
| Same, 0.55 m | a3_2 -0.35, +0.18 then -0.72, +0.63 N | 0.4-0.6 N, close, tether near |
| Same | a1 -0.24, +0.43 then +0.10, +0.60 N | |
| Lean excess by team size | the same at 2, 3 and 4 quads, section 4 | not mutual wash |

Windows stop before QUAD4 comes near. **Verdict: not in an even hover.** It matters when QUAD4 is over the ring.

### 3e. The planner's or tracker's steady maths: the stale length

`_apply_measured_rod_lengths` (`planner_node.py:1948`) sets each cable's length once, at hand-over, with the ring
still on the floor, before the lift loads the cables. In flight they read longer: mean +6 mm at 1 m (-6 to +16), +3 mm
at 0.55 m (-5 to +13). The plan places each drone at `ring + attach - l s`, so with `l` short by `d` the plan's node 0
sits `d` from the real drone, along the cable, toward the ring.

| Tracker's steady error (ref - pose) on d x cable direction, 185 carrier windows | Slope | r |
|---|---|---|
| Height | 1.00 | 0.98 |
| x | 1.69 | 0.81 |
| y | 1.95 | 0.82 |

So the trackers' steady error is the plan's own offset. The trackers hold what they are given.

The real planner solver on one logged state (sq3, 15 s mean, 170-185 s from the log's start):

| Quantity | Logged | Replay, as flown | Replay, true lengths |
|---|---|---|---|
| Plan: ring rises in 0.5 s, mm | +12.7 | +12.5 | +12.5 |
| Plan's node 0 against the drone, height, Q1 Q2 Q3, mm | +2.0, -11.3, -12.9 | +1.9, -11.8, -13.6 | 0, 0, 0 |
| Planned tensions at node 0, N | 5.28, 5.17, 5.16 | 5.283, 5.169, 5.161 | same |
| Same state with the height force off: ring rises in 0.5 s, mm | | +12.2 | |

## 4. Per carrier

Full table: appendix. By setup, windows of 3 s or more:

| Setup | Windows | rms error xy, z, mm | Stale d (range), mm | Elev | Lean | Out, side | Plan, N | Pull, N |
|---|---|---|---|---|---|---|---|---|
| 1 m, 3 quads | 18 | 6.6, 7.6 | +5.9 (-5.7, +15.6) | 60.3 | 16.8 | +4.1, -0.1 | 4.68 | 4.71 |
| 1 m, 4 quads | 4 | 4.8, 5.9 | +6.1 (+2.5, +9.3) | 60.3 | 13.3 | +3.3, +0.2 | 3.46 | 3.25 |
| 0.55 m, 2 quads | 4 | 4.8, 2.0 | +2.6 (0.0, +4.8) | 60.3 | 18.5 | +3.5, +1.9 | 5.01 | 5.80 |
| 0.55 m, 3 quads | 63 | 5.1, 3.8 | +2.7 (-5.0, +12.9) | 60.3 | 15.3 | +4.0, +0.8 | 3.40 | 3.68 |
| 0.55 m, 4 quads | 28 | 7.1, 4.0 | +3.4 (-4.4, +9.2) | 58.7 | 14.9 | +5.2, +0.3 | 2.71 | 2.95 |

Elev: deg, pivot to the model's attach point. Lean: deg. Out, side: lean beyond what its cable and thrust imply, deg.
Plan: the planner's node-0 tension, inflated by the height force. Pull: thrust map minus the airframe's weight.

| Tips against the model, 1 m (h4f_3, f4) | Value |
|---|---|
| Elevation, model and along the tip, per cable | within 0.6 deg |
| Tip off the model's attach point, per cable | 1-10 mm (plate 9 -10 mm along y, the slot is wide) |

**A finding, not part of the ring's balance:** every carrier leans about 4 deg further out than its cable and thrust
imply, in every flight, whatever its heading, about 0.7 N. It does not follow the quad, the plate or the team size.

| Lean excess, fitted with each quad's axis and the room taken out | Deg |
|---|---|
| 60 deg cables, 0.55 m, 141 windows | +4.17 +- 0.08 |
| 60 deg cables, 1 m, 30 windows | +3.90 +- 0.14 |
| 45 deg cables, 0.55 m, 8 windows (7 Oct) | +9.03 +- 0.27 |

It sums to zero in an even layout and it acts on the quad, so it does not move the ring. Thrust itself is not the
cause: in the instant after a release the real thrust matches the map at the throttle just before (ch1b_2 13.4 N
against 13.4, t3p_3 10.2 against 10.4). Sideways acceleration at those instants is 0.81 of what the body lean predicts
(median of 9 releases), and 0.9 on QUAD4's free circle, so part of the excess is the quad itself. The rest is not
explained.

## 5. The offline loop: how much of each cause

The real planner solver and the real integral, closed around a ring in static balance on taut cables, with trackers
modelled as following the plan 0.46 s ahead (the one free number, read from sq3's plan rise above). No swing, no
Gazebo, no wash. Each flight's in-flight and latched lengths go in, and nothing else: no force, no tilt. All "loop"
numbers are this.

| Flight | Stale d, mm | Height flown, loop, mm | Tilt y flown, loop | F-inf, N | Stale part, N | Left, N |
|---|---|---|---|---|---|---|
| sq3 | Q1 -2, Q2 +14, Q3 +16 | -36, -37 | +3.8, +4.5 | -0.07, -0.16 | +0.04, -0.29 | -0.11, +0.13 |
| k1_4 | Q1 -6, Q2 +14, Q3 +12 | -26, -27 | +4.9, +5.1 | -0.23, -0.42 | -0.04, -0.31 | -0.19, -0.11 |
| k1_8 | Q1 -6, Q2 +9, Q3 +13 | -20, -21 | +4.0, +4.6 | -0.31, -0.25 | +0.04, -0.27 | -0.35, +0.02 |
| sq2_5 | Q1 -4, Q2 +6, Q3 +5 | -13, -10 | +2.1, +2.7 | -0.33, +0.02 | 0.00, -0.15 | -0.33, +0.18 |
| sq4_5 | Q1 0, Q2 +10, Q3 +8 | -27, -25 | +0.9, +2.4 | -0.22, +0.13 | -0.03, -0.14 | -0.20, +0.27 |
| sq5_2, not settled | Q1 -4, Q2 +14, Q3 +6 | -22, -22 | +2.8, +3.7 | -0.08, -0.02 | -0.14, -0.22 | +0.05, +0.21 |
| ch1b_2, integral off | Q1 +6 Q2 +3 Q3 +9 Q4 +7 | -25, -18 | +0.7, +0.3 | -0.18, +0.02 | 0.00, -0.09 | -0.18, +0.11 |
| Mean of the seven | | | | -0.20, -0.10 | -0.02, -0.21 | -0.19, +0.12 |
| Predicted left: 1 deg at 150 deg, not fitted | | | | | | -0.17, +0.10 |

| New ring, F-inf, the seven windows | rms, N |
|---|---|
| As flown | 0.30 |
| Minus the stale part | 0.28 |
| Minus the 1 deg frame | 0.28 |
| Minus both (67 % of F squared removed) | 0.17 |
| Left over, mean | -0.02, +0.02 |
| Left over, flight to flight, sd | 0.13, 0.12 |

F-inf y against the stale part's y: r 0.93 on the six triangles. The new ring's six x values do not follow it (r
-0.39).

| Old ring, triangles | F-inf rms, N | After stale + frame + pivot | Removed | Left, mean | Left, sd |
|---|---|---|---|---|---|
| Before the fix, 18 windows, 3 settled | 0.79 | 0.31 | 85 % | -0.09, -0.15 | 0.20, 0.16 |
| After the fix, 4 windows, none settled | 0.43 | 0.38 | 21 % | -0.22, -0.28 | 0.11, 0.06 |

The last row is the weak spot. Those windows run 2-5 s after the top while the offset is still falling, so F-inf reads
high. The one clean reading after the fix is ls3c's spin (world -0.09, 0.00 N). Do not use that row as a measurement.

| Loop, one cause at a time, 1 m, 3 quads | Raw offset, mm | Integral force, N | Ring tilt, deg |
|---|---|---|---|
| Nothing wrong | 0 | 0 | 0 |
| Frame tilt 1 deg | 27 | 0.20 | 0.95 |
| Real force 0.2 N | 27 | 0.20 | 0.97 |
| All cables 1 cm short | 0 sideways, 41 low | 0 | 0 |
| One cable 1 cm short | 20 sideways, 14 low | 0.16 | 2.7 |
| Ring's point 1 cm off its centre | 34 | 0.25 | 2.8 |
| Real centre of mass 1 cm off | 47 | 0.35 | 1.7 |
| sq3's lengths, flown as latched | 34 sideways, 38 low | 0.33 | 4.8 |
| sq3's lengths, set right | 0 | 0 | 0 |

In a square, one stale cable just goes slack in the loop, with no offset. That fits ch1b_2's small stale part.

**The height.** With the sideways force at zero the ring still reads low by 4.4 x the mean vertical part of `d`:

| Fit, height error on the stale term | Slope | R2 | Windows |
|---|---|---|---|
| Measured pull, all | 4.42 | 0.85 | 50 |
| 1 m | 4.15 | 0.93 | 9 |
| 0.55 m | 3.91 | 0.68 | 41 |
| Loop against flown, 1 m triangles | 0.85 | 0.98 | 6 |

The ring rides low until its error makes the plan rise fast enough to cancel the node-0 offset. This is the "constant
32 mm low" of the figure-eight (thrust margin, section 8). The height force cannot help: with the trackers on the
measured pull it changes the planned tension, not the plan's motion (replay above, 12.5 against 12.2 mm).

| Flight, 25 s or more, measured pull | Fz start, end, N | Ring z start, end, m | Height per N, mm |
|---|---|---|---|
| sq3 (reaches -2.14 N at 21.7 s) | -0.48, -2.14 | 0.960, 0.966 | -1.5 |
| k1_8 | -0.24, -1.60 | 0.973, 0.978 | +0.2 |
| k1_4 | -0.30, -1.47 | 0.980, 0.972 | +4.1 |
| sq2_5 | -0.20, -0.73 | 0.977, 0.981 | +5.9 |
| t5_5 | -0.14, -1.90 | 0.989, 0.988 | -0.6 |
| t6 | -0.16, -1.29 | 0.989, 0.994 | -1.3 |
| t5_4 | -0.09, -0.87 | 0.990, 0.997 | -2.7 |
| t3p_2 | -0.13, -1.02 | 0.988, 0.994 | -4.4 |
| The card's 7.13 N/m would be | | | -140 |

Its cost: the planned tension at the bound is 5.16 N against 4.33 N (replay), and it fires the "near its bound"
warning. On the planned pull the force does reach the trackers. The ring's tilt of 2-5 deg on the triangle is the
sideways integral acting on the stale part: flown against loop r 0.92 in y, rms flown 2.9 deg, rms difference 1.2 deg.

## 6. Earlier explanations, checked

| Claim | Holds? | Numbers |
|---|---|---|
| RIG-1021-offsetc: the pivot, 5.5-6.6 cm, 1.7 cm after | yes, as one part | (pivot + 1 deg) / K: 5.3 cm, then 1.1 cm |
| RIG-1021-offsetc: "QUAD2 link short" | yes in the logs | Q2 reads 0.53-0.54 m, the loop takes it in |
| "-x push" of 0.41-0.52 N, 1 Oct, B4 replay | direction yes, size no | 0.13-0.20 N. The replay summed axis errors |
| z_int_model card: k = K = 5.26 N/m | not on these setups | K 7.3-18.2 N/m by setup |
| RIG-1022-t3p_2: "T hung 2.6 deg" | steady part 1.2 deg, the rest swing | tilt -0.94, +0.76 deg |
| Thrust margin: ring 3 cm low on f4 | now explained | mean d 6.8 mm, height -30 mm |
| Ring drift: the integral is not the wander | still holds | a slow force, not the 0.2 Hz mode |

## 7. The test (propose only)

**One hover, one extra pack: the same triangle turned 180 deg in the room.** Wed 4 or Thu 5 Nov, 8-11 am, right after
`h3t`, same calibration, same line. Ring at yaw about 180, QUAD1-3 on plates 3/11/7 as usual, so they stand on the
other side of the room. 21 Oct flew this on the old ring (c200_4). Hold 25 s at the top, about 3 tau with the gate's
duty. LAND by hand.

Read over the last 10 s of both flights: `int_f` + 7.3 x offset, minus the stale part from the planner's own lengths.

| If the force is | h3t, yaw 0 | Turned 180 deg |
|---|---|---|
| Fixed to the room (frame or air) | -0.2, +0.1 N | -0.2, +0.1 N |
| Fixed to the ring (centre of mass, pivot) | -0.2, +0.1 N | +0.2, -0.1 N |
| Half and half | -0.2, +0.1 N | 0, 0 N |

Flight-to-flight scatter is 0.13 N, so a shift under about 0.2 N says nothing. The same two flights also test section
5 with nothing changed: height and ring tilt should follow the lengths in their logs.

Two additions, no flying, 1 minute each:
- A spirit level where the ring rests. Level, while Motive reads that floor about 0.9 deg off (the ring at rest at two
  yaws), means the frame and not air.
- Motive was recalibrated on 30 Oct. The first steady hover and QUAD4's lean in `sq6b` give the new tilt.

Nothing here edits a sheet, a default or the code. Typing equal lengths will not test the stale part: the loop gives
sq3 +3 mm height with `measure_rod_len:=false cable_len:=0.994`, but the tilt rises from 4.7 to 6.6 deg, because Q1
reads 2.2 cm shorter. A direct test needs each length read again at the top of the lift, a planner change and your
word. Loop's expectation on sq3's lengths with the frame tilt added: height error 0, sideways force 0.20 N, ring tilt
about 1 deg.

## 8. What could not be decided

| Open | Why |
|---|---|
| New ring: room or ring | every hover at yaw -6 to +3. A centre of mass 5.7 mm off toward 150 deg equals 1 deg (loop) |
| Frame or room air | both fixed to the room. No spirit level or plumb reading exists |
| QUAD4's tether as a plumb line | tried: scatter 1-2 deg and a body offset of its own, not usable |
| Flight-to-flight scatter | 0.13 N on the new ring. On 14 Oct, same setup, F -0.18 (t5_4) against -0.64 (t5_5) |
| Slow creep in F | t5_5 walks from -0.25 to -0.65 N over 80 s at a steady 4 mm mean offset, not explained |
| The T and uneven layouts | few windows, integral off, the stiffness of a T not measured |
| The lean excess, the rest of it | about 0.7 N on each quad, section 4 |
| After 30 Oct's recalibration | no steady hover yet. The tilt may have changed |
| Why Q1 reads 2.2 cm short at 1 m | cable or body origin, no bench reading separates them |

Limits of the loop: static ring, one-number tracker, no air. It reproduces height and tilt, which the frame tilt does
not touch, so the stale part is well tied down. The frame part is checked by its own three readings, not by the loop.

## Appendix: per carrier, windows of 3 s or more

Flown: cable length in the window, pivot to the model's attach point, m. Latched: the planner's value. Stale: flown
minus latched, mm. ex, ey, ez: tracker ref minus pose, mm. Elev: cable elevation, deg. Az: cable azimuth against the
plan's radial direction, deg. Lean: the quad's total, deg. Out, Side: lean beyond what cable and thrust imply, along
and across the outward direction, deg. Plan: the planner's tension, N. Pull: map thrust minus weight, N. Pl: plate.
Q1-Q4: QUAD1-4.

**1 m, 1.15 kg**

| Flight | Quad | Pl | Yaw | ex | ey | ez | Flown | Latched | Stale | Elev | Az | Lean | Out | Side | Plan | Pull |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| ch1b_2 | Q1 | 0 | +98 | -4 | -2 | -5 | 0.991 | 0.985 | +6 | 59.8 | -1.3 | 13.9 | +4.5 | +0.5 | 3.51 | 3.02 |
| ch1b_2 | Q2 | 3 | +9 | -1 | +4 | -2 | 1.002 | 0.999 | +3 | 60.3 | -0.5 | 15.1 | +3.7 | +1.2 | 3.48 | 3.88 |
| ch1b_2 | Q3 | 6 | -90 | +5 | +0 | -9 | 1.002 | 0.993 | +9 | 60.7 | -1.3 | 9.8 | +0.7 | -0.3 | 3.42 | 2.67 |
| ch1b_2 | Q4 | 9 | -85 | +1 | -6 | -6 | 1.002 | 0.995 | +7 | 60.5 | -2.3 | 14.5 | +4.3 | -0.6 | 3.45 | 3.44 |
| k1_4 | Q1 | 3 | -17 | -1 | -1 | +5 | 0.978 | 0.984 | -6 | 62.2 | +0.3 | 16.8 | +4.0 | +0.9 | 4.78 | 5.26 |
| k1_4 | Q2 | 11 | +47 | -8 | -6 | -11 | 0.996 | 0.982 | +14 | 58.8 | +0.9 | 17.4 | +4.7 | -0.1 | 4.69 | 4.43 |
| k1_4 | Q3 | 7 | -119 | +5 | -4 | -10 | 1.002 | 0.990 | +12 | 59.8 | -4.4 | 13.2 | +0.5 | -0.8 | 4.69 | 4.17 |
| k1_8 | Q1 | 3 | -2 | -1 | -1 | +5 | 0.978 | 0.984 | -6 | 61.7 | +1.6 | 17.1 | +4.4 | +0.8 | 4.78 | 5.08 |
| k1_8 | Q2 | 11 | +131 | -5 | -3 | -7 | 0.998 | 0.989 | +9 | 58.9 | +0.6 | 16.5 | +3.6 | +0.0 | 4.72 | 4.39 |
| k1_8 | Q3 | 7 | -111 | +5 | -4 | -10 | 1.003 | 0.990 | +13 | 60.3 | -2.7 | 13.2 | +0.7 | -1.1 | 4.69 | 4.19 |
| sq2_5 | Q1 | 3 | -64 | -1 | -1 | +4 | 0.977 | 0.981 | -4 | 60.1 | +1.0 | 17.2 | +4.2 | +0.7 | 4.57 | 4.84 |
| sq2_5 | Q2 | 11 | +129 | -5 | -5 | -4 | 0.998 | 0.992 | +6 | 59.5 | -2.6 | 18.7 | +4.8 | +0.6 | 4.50 | 5.26 |
| sq2_5 | Q3 | 7 | -77 | +3 | -3 | -5 | 1.001 | 0.996 | +5 | 61.2 | -2.1 | 14.3 | +1.4 | -2.1 | 4.50 | 4.69 |
| sq3 | Q1 | 3 | -96 | +0 | -0 | +2 | 0.978 | 0.980 | -2 | 61.1 | +1.7 | 16.7 | +3.9 | +0.6 | 5.08 | 4.91 |
| sq3 | Q2 | 11 | +77 | -8 | -5 | -11 | 1.000 | 0.986 | +14 | 59.7 | +2.0 | 17.4 | +5.1 | -0.1 | 4.99 | 4.41 |
| sq3 | Q3 | 7 | +53 | +9 | -6 | -13 | 1.002 | 0.986 | +16 | 60.4 | +0.2 | 18.1 | +6.4 | -0.6 | 4.97 | 4.33 |
| sq4_5 | Q1 | 3 | -2 | +0 | +3 | +1 | 0.977 | 0.977 | +0 | 59.9 | +3.4 | 18.7 | +5.0 | +0.7 | 4.64 | 5.39 |
| sq4_5 | Q2 | 11 | +123 | -6 | -2 | -8 | 1.001 | 0.991 | +10 | 59.8 | -0.5 | 16.2 | +3.1 | +0.5 | 4.60 | 4.65 |
| sq4_5 | Q3 | 7 | +35 | +5 | -5 | -7 | 1.002 | 0.994 | +8 | 61.5 | +1.9 | 17.5 | +5.1 | -1.2 | 4.58 | 4.85 |
| sq5_2 | Q1 | 3 | -8 | -1 | -0 | +5 | 0.979 | 0.983 | -4 | 60.4 | +0.7 | 17.9 | +4.9 | +0.3 | 4.55 | 5.01 |
| sq5_2 | Q2 | 11 | +13 | -10 | -4 | -11 | 0.995 | 0.981 | +14 | 59.9 | -0.0 | 17.4 | +4.8 | -0.4 | 4.47 | 4.57 |
| sq5_2 | Q3 | 7 | +57 | +5 | -4 | -4 | 1.000 | 0.994 | +6 | 60.7 | -0.7 | 18.1 | +6.4 | +0.5 | 4.49 | 4.42 |

**0.55 m, 0.86 kg, after the pivot fix**

| Flight | Quad | Pl | Yaw | ex | ey | ez | Flown | Latched | Stale | Elev | Az | Lean | Out | Side | Plan | Pull |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| h3 | Q2 | 0 | +1 | -5 | -1 | -2 | 0.536 | 0.532 | +4 | 58.5 | -2.0 | 16.9 | +5.3 | -0.0 | 3.34 | 3.66 |
| h3 | Q4 | 8 | +126 | +2 | -3 | -2 | 0.566 | 0.564 | +2 | 60.8 | -5.7 | 14.8 | +3.5 | -0.7 | 3.31 | 3.74 |
| h3 | Q3 | 4 | -124 | -2 | +4 | +0 | 0.559 | 0.559 | +0 | 61.5 | -0.0 | 15.0 | +3.7 | +3.3 | 3.30 | 3.69 |
| t3p_2 | Q3 | 2 | +2 | +1 | +5 | -2 | 0.552 | 0.549 | +3 | 66.9 | +6.4 | 2.4 | +0.7 | -0.6 | 0.12 | 0.46 |
| t3p_2 | Q2 | 11 | -9 | -4 | +0 | -3 | 0.538 | 0.534 | +4 | 58.7 | +3.3 | 19.4 | +4.2 | -0.5 | 5.26 | 5.51 |
| t3p_2 | Q4 | 5 | +149 | +3 | +1 | +0 | 0.566 | 0.565 | +1 | 60.2 | -5.2 | 17.9 | +4.1 | +0.5 | 5.16 | 5.08 |
| hS2 | Q2 | 11 | +105 | -5 | -1 | -2 | 0.538 | 0.534 | +4 | 58.7 | -3.3 | 19.7 | +3.5 | +1.1 | 5.04 | 6.10 |
| hS2 | Q3 | 5 | +176 | +5 | +3 | -1 | 0.561 | 0.559 | +2 | 62.0 | -0.2 | 18.3 | +3.9 | +3.2 | 4.98 | 5.87 |

**0.55 m, 0.86 kg, before the pivot fix**

| Flight | Quad | Pl | Yaw | ex | ey | ez | Flown | Latched | Stale | Elev | Az | Lean | Out | Side | Plan | Pull |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| h5 | Q2 | 0 | +125 | +6 | -5 | -7 | 0.556 | 0.547 | +9 | 58.8 | -2.4 | 16.4 | +4.1 | -0.0 | 3.81 | 3.92 |
| h5 | Q4 | 8 | +92 | +6 | +13 | -9 | 0.552 | 0.541 | +11 | 65.7 | +3.3 | 15.1 | +5.7 | +0.2 | 3.66 | 3.88 |
| h5 | Q1 | 4 | +95 | -6 | +2 | -3 | 0.546 | 0.542 | +4 | 59.4 | -4.1 | 17.6 | +5.2 | +1.0 | 3.82 | 4.17 |
| r000014 | Q2 | 0 | -81 | -7 | +1 | -3 | 0.556 | 0.553 | +3 | 55.8 | +3.4 | 15.4 | +4.1 | -0.5 | 2.96 | 3.07 |
| r000014 | Q4 | 9 | +12 | +0 | -8 | -6 | 0.549 | 0.543 | +6 | 63.3 | -3.8 | 13.4 | +4.6 | +1.0 | 2.84 | 3.02 |
| r000014 | Q3 | 6 | -172 | +8 | +2 | -5 | 0.559 | 0.555 | +4 | 64.5 | +0.3 | 13.7 | +5.4 | +1.0 | 2.88 | 3.01 |
| r000014 | Q1 | 3 | +4 | -1 | +8 | -3 | 0.546 | 0.543 | +3 | 55.8 | -1.0 | 15.6 | +4.4 | +1.2 | 3.01 | 3.06 |
| r0012 | Q2 | 11 | -45 | -6 | -6 | -5 | 0.550 | 0.544 | +6 | 59.1 | +8.1 | 13.2 | +4.0 | -0.8 | 2.43 | 2.65 |
| r0012 | Q4 | 9 | +0 | +0 | -10 | -5 | 0.552 | 0.546 | +6 | 65.2 | -4.9 | 10.0 | +4.4 | +1.1 | 1.71 | 1.87 |
| r0012 | Q3 | 7 | +148 | +8 | -7 | -6 | 0.560 | 0.553 | +7 | 67.2 | -2.4 | 11.5 | +5.2 | +0.9 | 2.31 | 2.43 |
| r0012 | Q1 | 3 | +8 | +1 | +3 | -2 | 0.547 | 0.544 | +3 | 58.5 | +0.3 | 17.7 | +4.1 | +1.1 | 4.30 | 4.53 |
| r0013 | Q2 | 11 | -48 | -6 | -5 | -6 | 0.552 | 0.544 | +8 | 58.2 | +3.8 | 14.3 | +4.1 | -0.9 | 2.79 | 2.92 |
| r0013 | Q4 | 9 | +84 | +1 | -8 | -4 | 0.551 | 0.546 | +5 | 61.2 | -1.7 | 10.7 | +3.5 | +0.1 | 1.98 | 2.07 |
| r0013 | Q3 | 6 | -178 | +10 | -2 | -4 | 0.561 | 0.555 | +6 | 65.7 | +0.4 | 11.6 | +4.9 | +1.1 | 2.29 | 2.37 |
| r0013 | Q1 | 3 | -0 | +3 | +6 | -2 | 0.546 | 0.542 | +4 | 56.0 | -0.8 | 17.2 | +4.3 | +1.1 | 3.59 | 3.76 |
| r002 | Q1 | 0 | +92 | -7 | -2 | -3 | 0.541 | 0.538 | +3 | 59.3 | -1.3 | 14.6 | +4.8 | +1.1 | 2.82 | 2.96 |
| r002 | Q4 | 9 | +3 | +1 | -10 | -4 | 0.549 | 0.544 | +5 | 61.4 | -3.1 | 14.3 | +5.3 | +0.8 | 2.81 | 2.94 |
| r002 | Q3 | 6 | +178 | +11 | -2 | -8 | 0.560 | 0.551 | +9 | 64.9 | -1.7 | 13.4 | +5.6 | +0.5 | 2.83 | 2.86 |
| r002 | Q2 | 3 | -179 | +1 | +3 | -1 | 0.559 | 0.558 | +1 | 56.6 | +6.7 | 14.5 | +3.8 | -0.5 | 2.85 | 2.93 |
| r20004 | Q4 | 0 | -4 | -5 | +0 | +1 | 0.552 | 0.551 | +1 | 43.7 | -0.8 | 23.5 | +7.9 | +0.6 | 3.03 | 3.75 |
| r20004 | Q1 | 9 | -177 | -0 | -1 | +3 | 0.525 | 0.528 | -3 | 46.3 | -1.0 | 20.4 | +8.4 | +0.2 | 2.99 | 3.01 |
| r20004 | Q3 | 6 | -176 | +3 | -0 | +5 | 0.548 | 0.552 | -4 | 48.0 | +3.2 | 23.6 | +10.5 | +0.5 | 2.97 | 3.70 |
| r20004 | Q2 | 3 | -87 | +1 | +8 | -2 | 0.553 | 0.548 | +5 | 44.2 | +2.0 | 22.0 | +9.7 | -0.4 | 3.01 | 3.13 |
| r302_3 | Q2 | 11 | -62 | -2 | -1 | +1 | 0.553 | 0.555 | -2 | 59.9 | -1.8 | 12.8 | +3.7 | -0.7 | 2.19 | 2.65 |
| r302_3 | Q4 | 9 | +91 | -1 | -4 | -0 | 0.550 | 0.550 | +0 | 61.9 | -1.3 | 9.1 | +3.0 | -0.4 | 1.61 | 1.75 |
| r302_3 | Q3 | 7 | +146 | +6 | -5 | -2 | 0.558 | 0.555 | +3 | 62.9 | +4.8 | 13.5 | +6.4 | -0.1 | 2.13 | 2.44 |
| r302_3 | Q1 | 3 | +3 | +1 | +5 | -5 | 0.547 | 0.542 | +5 | 59.9 | +3.7 | 17.1 | +4.6 | +1.0 | 3.82 | 4.30 |
| r303_4 | Q2 | 11 | -34 | -3 | -2 | -2 | 0.552 | 0.548 | +4 | 60.0 | -2.5 | 12.8 | +4.4 | -0.8 | 2.18 | 2.47 |
| r303_4 | Q4 | 9 | +86 | -1 | -3 | +0 | 0.549 | 0.548 | +1 | 61.8 | -1.8 | 10.1 | +3.3 | -1.4 | 1.62 | 1.94 |
| r303_4 | Q3 | 7 | +149 | +4 | -2 | +2 | 0.557 | 0.558 | -1 | 62.6 | +1.8 | 13.6 | +6.3 | -0.1 | 2.14 | 2.48 |
| r303_4 | Q1 | 3 | +3 | +1 | +6 | -4 | 0.547 | 0.542 | +5 | 60.1 | +3.6 | 17.4 | +4.5 | +0.4 | 3.80 | 4.58 |
| lift3_3 | Q3 | 3 | -6 | +0 | +3 | -2 | 0.560 | 0.557 | +3 | 58.7 | +2.8 | 13.5 | +0.9 | -0.7 | 3.28 | 3.73 |
| lift3_3 | Q2 | 11 | +113 | -3 | -3 | -0 | 0.550 | 0.549 | +1 | 59.1 | -4.1 | 16.5 | +4.3 | +1.7 | 3.27 | 3.87 |
| lift3_3 | Q1 | 7 | -109 | +1 | +0 | +3 | 0.546 | 0.549 | -3 | 62.3 | -0.1 | 14.1 | +4.6 | -0.0 | 3.22 | 3.19 |
| t5_4 | Q3 | 3 | -86 | -3 | +4 | -2 | 0.558 | 0.554 | +4 | 59.8 | +1.5 | 16.1 | +4.2 | +3.0 | 3.44 | 3.81 |
| t5_4 | Q2 | 11 | +40 | -3 | -2 | -1 | 0.533 | 0.531 | +2 | 59.5 | -1.8 | 16.1 | +4.3 | +0.1 | 3.45 | 3.80 |
| t5_4 | Q1 | 7 | -110 | +2 | -1 | +1 | 0.546 | 0.547 | -1 | 61.4 | -0.3 | 14.6 | +5.0 | -0.2 | 3.44 | 3.15 |
| t5_5 | Q3 | 3 | -113 | -1 | +5 | -4 | 0.558 | 0.552 | +6 | 59.0 | +6.5 | 16.1 | +4.4 | +2.7 | 3.66 | 3.61 |
| t5_5 | Q2 | 11 | +49 | -4 | -1 | -3 | 0.533 | 0.529 | +4 | 57.8 | -4.0 | 16.0 | +4.6 | +0.4 | 3.68 | 3.42 |
| t5_5 | Q1 | 7 | -124 | +0 | -2 | +3 | 0.545 | 0.547 | -2 | 63.8 | -0.1 | 15.1 | +5.2 | -0.9 | 3.66 | 3.71 |
| t6 | Q3 | 3 | -94 | -2 | +5 | -2 | 0.555 | 0.552 | +3 | 58.2 | +4.4 | 16.6 | +4.5 | +3.0 | 3.55 | 3.67 |
| t6 | Q2 | 11 | +33 | -4 | -2 | -2 | 0.533 | 0.530 | +3 | 58.6 | -5.8 | 15.8 | +4.3 | +0.8 | 3.55 | 3.51 |
| t6 | Q1 | 7 | -34 | +2 | -2 | +0 | 0.545 | 0.545 | +0 | 63.8 | -0.4 | 14.7 | +4.6 | -0.7 | 3.53 | 3.74 |
| t6f | Q3 | 3 | -95 | -4 | +5 | -1 | 0.556 | 0.553 | +3 | 58.8 | +2.6 | 16.5 | +4.4 | +3.5 | 3.29 | 3.74 |
| t6f | Q2 | 11 | +25 | -3 | -2 | +0 | 0.534 | 0.534 | +0 | 58.6 | -5.3 | 16.3 | +4.5 | +0.6 | 3.30 | 3.66 |
| t6f | Q1 | 7 | -118 | +1 | -0 | +3 | 0.545 | 0.548 | -3 | 62.9 | -1.3 | 14.8 | +4.7 | -0.7 | 3.26 | 3.57 |
| t6f2_4 | Q3 | 3 | -87 | -5 | +4 | -2 | 0.558 | 0.555 | +3 | 58.6 | +3.6 | 16.3 | +4.5 | +3.2 | 3.42 | 3.56 |
| t6f2_4 | Q2 | 11 | +41 | -8 | -5 | -9 | 0.533 | 0.522 | +11 | 57.7 | -8.5 | 16.2 | +4.5 | +0.8 | 3.42 | 3.50 |
| t6f2_4 | Q1 | 7 | -101 | +2 | +0 | +1 | 0.545 | 0.547 | -2 | 64.3 | -4.3 | 14.6 | +4.4 | +0.2 | 3.37 | 3.89 |
| t6g_2 | Q3 | 3 | -75 | -3 | +4 | -2 | 0.558 | 0.555 | +3 | 58.8 | +1.9 | 15.7 | +3.7 | +2.7 | 3.33 | 3.66 |
| t6g_2 | Q2 | 11 | +43 | -4 | -3 | -2 | 0.535 | 0.532 | +3 | 59.2 | -5.9 | 15.8 | +3.9 | +0.5 | 3.32 | 3.74 |
| t6g_2 | Q1 | 7 | -104 | +1 | -0 | +3 | 0.548 | 0.551 | -3 | 62.7 | -0.7 | 14.5 | +4.0 | -0.4 | 3.29 | 3.69 |
| t6h_4 | Q3 | 3 | -83 | -3 | +4 | -4 | 0.559 | 0.554 | +5 | 58.3 | +6.3 | 15.8 | +3.9 | +2.9 | 3.56 | 3.56 |
| t6h_4 | Q2 | 11 | +39 | -8 | -4 | -9 | 0.533 | 0.521 | +12 | 57.5 | -7.1 | 16.4 | +4.9 | +0.6 | 3.55 | 3.43 |
| t6h_4 | Q1 | 7 | -118 | +2 | -1 | +0 | 0.546 | 0.546 | +0 | 65.0 | -1.5 | 14.3 | +4.6 | -0.2 | 3.51 | 3.82 |
| a1 | Q1 | 3 | +1 | +0 | +5 | -2 | 0.558 | 0.555 | +3 | 58.3 | +2.3 | 15.9 | +4.4 | +0.4 | 3.32 | 3.52 |
| a1 | Q3 | 11 | +32 | -1 | -2 | +1 | 0.551 | 0.551 | -0 | 58.9 | -6.9 | 14.4 | +2.4 | +2.7 | 3.32 | 3.52 |
| a1 | Q2 | 7 | +158 | +2 | -2 | +1 | 0.531 | 0.532 | -1 | 63.1 | -0.2 | 15.1 | +5.0 | -0.2 | 3.28 | 3.68 |
| a2_2 | Q1 | 3 | -3 | -2 | +4 | -0 | 0.557 | 0.556 | +1 | 58.6 | +1.0 | 16.2 | +4.1 | +0.6 | 3.31 | 3.78 |
| a2_2 | Q3 | 11 | +23 | -3 | -5 | -3 | 0.551 | 0.547 | +4 | 59.2 | -7.1 | 15.7 | +3.4 | +3.7 | 3.30 | 3.72 |
| a2_2 | Q2 | 7 | +156 | +2 | -1 | +3 | 0.529 | 0.532 | -3 | 62.5 | -0.8 | 15.1 | +4.4 | -0.5 | 3.28 | 3.80 |
| a3_2 | Q3 | 3 | -84 | -3 | +5 | -2 | 0.556 | 0.553 | +3 | 59.1 | +2.9 | 16.3 | +4.3 | +3.0 | 3.32 | 3.72 |
| a3_2 | Q2 | 11 | +43 | -4 | -2 | -2 | 0.532 | 0.530 | +2 | 58.7 | -5.2 | 16.5 | +5.0 | +0.9 | 3.33 | 3.58 |
| a3_2 | Q1 | 7 | -120 | +2 | -0 | +2 | 0.545 | 0.547 | -2 | 62.2 | -0.7 | 14.5 | +4.5 | -0.4 | 3.30 | 3.42 |
| c200_4 | Q2 | 0 | +163 | +3 | +0 | -2 | 0.535 | 0.533 | +2 | 58.1 | -5.3 | 14.7 | +2.9 | +1.1 | 3.30 | 3.44 |
| c200_4 | Q4 | 8 | -76 | -1 | +1 | +5 | 0.545 | 0.550 | -5 | 63.2 | -2.5 | 16.1 | +5.4 | +0.1 | 3.26 | 4.03 |
| c200_4 | Q3 | 4 | +73 | -0 | -4 | -2 | 0.557 | 0.554 | +3 | 59.0 | +4.7 | 14.5 | +2.7 | +1.4 | 3.29 | 3.54 |
| c2q | Q2 | 0 | +166 | +5 | +1 | -3 | 0.539 | 0.534 | +5 | 56.7 | -2.4 | 18.8 | +2.9 | +1.1 | 5.06 | 5.27 |
| c2q | Q3 | 6 | +6 | -3 | -2 | +1 | 0.552 | 0.552 | +0 | 63.9 | +0.1 | 17.0 | +3.6 | +2.2 | 4.96 | 5.96 |
| f125_2 | Q2 | 0 | -2 | -10 | -1 | -9 | 0.535 | 0.523 | +12 | 57.1 | -5.7 | 16.3 | +4.0 | +0.9 | 3.45 | 3.60 |
| f125_2 | Q4 | 8 | +33 | +1 | -3 | +0 | 0.545 | 0.544 | +1 | 64.4 | -6.7 | 14.9 | +4.2 | +0.1 | 3.39 | 4.18 |
| f125_2 | Q3 | 4 | -105 | -2 | +5 | -3 | 0.556 | 0.552 | +4 | 59.9 | +7.0 | 14.7 | +2.8 | +2.4 | 3.43 | 3.70 |
| f200 | Q2 | 0 | -6 | -10 | -0 | -9 | 0.536 | 0.524 | +12 | 57.3 | -5.8 | 16.5 | +4.2 | +0.9 | 3.41 | 3.68 |
| f200 | Q4 | 8 | +43 | +1 | -2 | +2 | 0.544 | 0.546 | -2 | 63.8 | -4.5 | 14.6 | +4.3 | +0.1 | 3.36 | 3.84 |
| f200 | Q3 | 4 | -47 | +0 | +4 | -5 | 0.556 | 0.551 | +5 | 59.4 | +6.5 | 12.8 | +1.1 | -0.1 | 3.40 | 3.45 |
| l125 | Q2 | 0 | -0 | -10 | -1 | -10 | 0.535 | 0.522 | +13 | 57.0 | -5.4 | 16.5 | +4.4 | +0.9 | 3.43 | 3.56 |
| l125 | Q4 | 8 | +51 | +1 | -2 | +1 | 0.543 | 0.545 | -2 | 64.0 | -7.0 | 14.6 | +4.5 | +0.4 | 3.38 | 3.78 |
| l125 | Q3 | 4 | -117 | -2 | +5 | -3 | 0.556 | 0.552 | +4 | 59.8 | +5.9 | 15.5 | +3.9 | +2.5 | 3.42 | 3.65 |
| l200 | Q2 | 0 | +6 | -6 | -1 | -3 | 0.535 | 0.529 | +6 | 57.4 | -6.1 | 16.6 | +4.3 | +0.7 | 3.36 | 3.71 |
| l200 | Q4 | 8 | +26 | +1 | -1 | +4 | 0.544 | 0.548 | -4 | 64.3 | -5.5 | 15.0 | +4.4 | +0.3 | 3.30 | 4.13 |
| l200 | Q3 | 4 | -118 | -1 | +6 | -4 | 0.556 | 0.551 | +5 | 59.2 | +5.2 | 15.8 | +3.9 | +2.5 | 3.34 | 3.69 |
| s125 | Q2 | 0 | +14 | -4 | +2 | -1 | 0.534 | 0.532 | +2 | 58.6 | -1.7 | 16.5 | +4.5 | +0.6 | 3.39 | 3.79 |
| s125 | Q4 | 8 | +102 | -1 | -2 | +1 | 0.545 | 0.546 | -1 | 63.1 | +3.0 | 14.4 | +4.0 | -1.1 | 3.34 | 3.72 |
| s125 | Q3 | 4 | -102 | +2 | +6 | -8 | 0.557 | 0.548 | +9 | 58.8 | +7.9 | 14.1 | +2.4 | +2.1 | 3.37 | 3.41 |
| s200_3 | Q2 | 0 | +8 | -4 | +0 | -1 | 0.532 | 0.531 | +1 | 58.7 | -1.2 | 16.6 | +4.5 | +0.9 | 3.35 | 3.84 |
| s200_3 | Q4 | 8 | +109 | +0 | -4 | -1 | 0.545 | 0.544 | +1 | 61.9 | -0.1 | 15.0 | +4.7 | -1.3 | 3.30 | 3.47 |
| s200_3 | Q3 | 4 | -51 | +2 | +3 | -4 | 0.557 | 0.552 | +5 | 59.9 | +5.1 | 13.2 | +1.2 | -0.2 | 3.31 | 3.68 |


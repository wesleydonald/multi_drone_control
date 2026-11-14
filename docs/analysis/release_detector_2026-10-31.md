# The release detector at 8 and 9 carriers (31 Oct 2026)

Read-only look for the lab chat, from the section "The missed releases" of
`docs/experiments/2026-10-30_x3b_realtime_8_9.md`. Nothing was flown and no code, config or result was changed.
`[log]` marks what the logs show, `[inf]` what follows from them. Thresholds are Wesley's word, so section 4 only
proposes. How it was computed is in section 5, every release is in the appendix.

## Verdict

| Question | Answer | Basis |
|---|---|---|
| Cause of the 3 misses | A low share, then the tick phase. No stale pose or RTF is needed | `[log]` t1, t6, t8 |
| Would fresh poses have fired? | No, in all 3. The first look holds, and is now tested | `[log]` t7 |
| Is it chance? | Yes. The same free curve fires at other tick phases | `[log]` t6 |
| Why only since 31 Oct? | `slope` lowered the curve. `ema`, same pull, lower RTF: no miss | `[inf]` t5 |
| A re-fly on a lighter slot? | Still a draw: about 2 in 3 valid at 8, 1 in 3 at 9 | `[inf]` t6 |
| After a miss | Nothing catches it. The flight lands, one rejoin never happens | `[log]` t10 |
| Real flights | Not exposed, the pulls are far larger. The T's rider needs the marker | `[log]` t11 |
| Recommendation | Hold the 3 re-flies. Marker check on for the 8, 9 chains, one test run | section 4 |

The scalability card's section 9 (`docs/experiments/2026-10-23_scalability.md`, "S section 9" below) names the
wall-clock gate and poses at 60 Hz as the cause. That is not what these runs show: the 60 Hz stream is the rod's end,
and ring and drone poses read back at twice that (table 8).

## 1. What the detector tests

**The stretch check** (`_detect_tick`, `dissipative_node.py:1954`). Once per planner tick, before the solve, for each
carrier in the model:

1. The reading: pivot to plate point (`_rim_dist`) minus the longer of the typed and measured rod. Call it "over".
2. Over at or under `detach_detect_m` (0.06 m): the count goes to 0 and the reading is remembered.
3. Over above it: the count goes up by 1, and the jump is this reading minus the remembered one.
4. It fires when the count is 2 or more and the jump is at least 3 cm. The 3 cm is a literal in the code, not a
   parameter. The remembered reading has no age limit, so "jump" means "above the last reading inside", not "fast".
5. All carriers over at once reads as a jump of the ring's pose: the counts reset, nothing fires.
6. It takes one loss per tick. Firing resets every count, then acts: the drop rule, or the resize.

| Gate | Effect | Clock |
|---|---|---|
| Ring pose older than `DETECT_STALE_S` 0.1 s | The tick is skipped, every count resets, no log line | wall |
| A drone's pose older than 0.1 s | That drone's count resets, no log line | wall |
| Off: phase not `planner`, not `ocp`, before TAKEOFF, soft-detach unload running | Tick skipped | - |
| Off in LAND once the ring is within 5 cm of its lift height (with `drop_on_loss`) | Tick skipped | - |

A fire in LAND with the ring still up means DROP AND LAND (`_drop_due`).

**The magnet-marker check** (`_tip_check`). Off unless `tip_detect_m` is above 0, and `s_chain_pair.yaml` leaves it
off, so it was off in all 91 releases below. On each marker sample it takes the rod end's offset from its plate point
in the ring's frame, against the offset learnt after the lift. It fires when the offset is past 4 cm and the distance
from the ring's centre has moved 1.5 cm, for 0.1 s on stamps and 6 samples. Samples off the learnt body-to-marker
band, repeated bit for bit, or within 0.2 s of a ring-pose glitch are no evidence. If no other armed marker is under
0.1 s old (wall) it logs WOULD-FIRE and tries again on the next sample. It does not read how much the carrier pulled.

## 2. The physics

**The share.** After one loss from an even ring the planner's level pull is, for carrier i at angle a_i from the gap,
T_i = W / (n - 1 - k) x (1 + k cos a_i), with k = 2 / (n - 2) and W = m g / sin(elevation). That is
`balanced_tensions` in closed form, and it reproduces the logged values. The carrier opposite the gap carries least.
On 8 carriers that is `/drone_5` once `/drone_1` has gone, on 9 it is `/drone_5` and `/drone_6`.

Table 1. Pull at each step of the chain (computed, 60 deg rods), against the plan logged before the release.

| Ring | Release | From | Computed N | Logged N | Note |
|---|---|---|---|---|---|
| 8, 0.9 kg | /drone_1 | 8 | 1.274 | 1.28 | even ring |
| 8, 0.9 kg | /drone_5 | 7 | 1.019 | 1.03 | opposite the gap, the lowest |
| 8, 0.9 kg | /drone_3 | 6 | 1.699 | 1.74 | |
| 8, 0.9 kg | /drone_7 | 5 | 1.456 | 1.50 | |
| 9, 0.9 kg | /drone_1 | 9 | 1.133 | 1.14 | even ring |
| 9, 0.9 kg | /drone_5 | 8 | 0.967 | 0.98 | 160 deg from the gap, the lowest |
| 9, 0.9 kg | /drone_3 | 7 | 1.576 | 1.43 | logged from 8, the ghost in the model |
| 9, 0.9 kg | /drone_7 | 6 | 1.177 | 0.95 | logged from 7, the ghost in the model |
| 8, 1.15 kg | /drone_5 | 7 | 1.303 | - | not flown |
| 9, 1.15 kg | /drone_5 | 8 | 1.235 | - | not flown |

So the answer to "why `/drone_5`" is the share, and the order only because the first release makes the gap. It has one
earlier release, not two. Another carrier opposite a gap behaves the same (R2776 `/drone_7`, from 7, plan 1.06 N, rise
4.5 cm under `ema`). `/drone_5` released with `/drone_1`, at the even share, rises as the others do (R2773).

**The rise.** Released, the quad accelerates along the rod at about T / m, and the plate moves the other way. The
tracker then reads the pull falling and drops its thrust, so the rise stops by itself. The first moments fit a t^2 / 2
(table 3) `[log]`. Most of a is the quad, the rest the plate `[inf]`.

Table 2. Free curves: no resize in the window, so the true peak shows. Measured-pull arms.

| Layout | Rod | Filter | Runs | Plan N | Pull N | Peak cm | At s | Quad up cm | Plate down cm |
|---|---|---|---|---|---|---|---|---|---|
| 9, /drone_5 | 1 m | slope | 2 | 0.98 | 1.14-1.15 | 7.6-7.8 | 0.83-0.84 | 3.6 | 1.5-1.6 |
| 8, /drone_5 | 1 m | slope | 1 | 1.03 | 1.19 | 8.3 | 0.82 | 3.5 | 2.1 |
| 8, /drone_1 | 1 m | slope | 6 | 1.28 | 1.34-1.58 | 9.7-11.5 | 0.56-0.81 | 4.3-5.2 | 2.2-2.9 |
| 4 (R 0.225), /drone_1 | 1 m | ema | 3 | 1.63 | 1.75-1.80 | 16.6-17.2 | 0.47-0.49 | 6.9-7.3 | 6.0-6.2 |
| 4 (R 0.225), /drone_1 | 1 m | slope | 15 | 1.63 | 1.68-1.81 | 14.4-15.5 | 0.52-0.69 | 5.4-6.1 | 5.0-5.4 |
| 4 (R 0.204, 1.15 kg) | 1 m | slope | 2 | 2.18 | 2.23-2.36 | 19.5-21.5 | 0.60-0.70 | 7.9-9.0 | 6.6-7.0 |
| Square to T | 1 m | slope | 13 | 2.44 | 1.80-2.63 | 45.8-55.4 | 0.65-0.76 | 8.4-15.9 | 29.5-31.9 |
| T's rider | 0.55 m | - | 2 | 0.10 | 0.29 | 5.6 | 1.01-1.04 | 0.4-0.6 | 2.3-2.4 |
| T's rider, hover | 1 m | - | 1 | 0.10 | 0.29 | 6.4 | 1.22 | 0.6 | 1.4 |

- **Pull.** It sets the peak (table 3). The measured pull runs over the plan by a steady amount, not separated here.
- **Carrier count.** It enters through the share and through the plate: a small fleet lets the plate fall further.
- **Ring size.** Seen only through the plate's fall (table 2). Not varied alone.
- **Cable length.** Little in the early rise (table 3). No free pair at the same pull exists to compare peaks.

Table 3. The rise in numbers. The fits are the free peak against the measured pull at release, 8 and 9 carriers,
`slope`, 9 runs.

| Item | Value |
|---|---|
| First 0.15 s, 87 curves at 8 and 9: a / T | 2.49 per kg, sd 0.20: an effective 0.40 kg, airframe 0.475 kg |
| Measured pull minus plan pull, 87 releases | 0.20 N, sd 0.07 |
| Measured pull under half its value after the release (R3221, R3218, R3227) | 0.10 s |
| Reading at 0.2 s, plan 2.44 N, cable 0.55 m and 1 m | 13.1, 14.3 cm (a / T 2.82, 3.09) |
| Fit | peak (cm) = 8.35 x pull (N) - 1.72, r 0.993 |
| The same fit on the 7 runs at 8 carriers | peak = 7.68 x pull - 0.72, largest residual 0.26 cm |
| Its prediction at 9 carriers against the flown peak | 8.01, 8.12 against 7.63, 7.79 cm |
| Spread of pull for one plan pull, run to run | plan 1.03 N: 1.10-1.26 N, plan 1.28 N: 1.34-1.58 N |

The 9-carrier runs sit just under the 8-carrier line. Their plate falls less (table 2), which accounts for it, so no
RTF term is needed to explain them `[inf]`.

Table 4. What the rule does with a peak. It follows from the rule: the last reading inside is at most 6 cm, so a 9 cm
peak always clears the jump, and a peak under 6 cm can never count. The pulls come from table 3's fit, the lowest
under its data `[inf]`.

| Free peak | Measured pull | Plan pull | The rule |
|---|---|---|---|
| Under 6 cm | under 0.92 N | under 0.72 N | can never fire |
| 6 to 9 cm | 0.92-1.28 N | 0.72-1.08 N | a draw: it depends on where the ticks fall |
| Over 9 cm | over 1.28 N | over 1.08 N | always fires |

Only two steps in table 1 sit in the draw: `/drone_5` at 8 and at 9. With the 1.15 kg ring neither does.

**The filter.** Same plan pull, same RTF band, two filters (tables 5, 5b). The `ema` runs (trees of 22-28 Oct) crossed
6 cm sooner and kept a margin on every release. The `slope` runs cross later and peak lower. At 4 carriers, where both
have free curves, the peak is about 12 % lower under `slope` (table 2). So the 28 Oct change of filter moved
`/drone_5` into the draw `[inf]`: no controlled pair exists, the trees differ too.

Table 5. Release summary by filter, measured-pull arms, plan pull up to 1.3 N. Lengths in cm.

| Carriers | Plan N | Filter | Rel. | Missed | Rise low, median | Over at 0.3 s, mean (range) | To 6 cm (s) | RTF |
|---|---|---|---|---|---|---|---|---|
| 8 | 1.0 | ema | 12 | 0 | 4.4, 5.3 | 7.60 (7.15-8.03) | 0.254 | 0.216-0.261 |
| 8 | 1.0 | slope | 4 | 1 | 3.0, 3.1 | 6.60 (5.93-7.11) | 0.291 | 0.228-0.255 |
| 8 | 1.1 | ema | 1 | 0 | 4.5, 4.5 | 7.59 (7.59-7.59) | 0.259 | 0.218-0.218 |
| 8 | 1.3 | ema | 16 | 0 | 5.4, 6.6 | 9.64 (8.05-14.94) | 0.213 | 0.216-0.261 |
| 8 | 1.3 | slope | 15 | 0 | 4.9, 5.8 | 8.40 (7.52-10.35) | 0.228 | 0.218-0.257 |
| 9 | 0.9 | slope | 1 | 0 | 3.0, 3.0 | 6.97 (6.97-6.97) | 0.270 | 0.190-0.190 |
| 9 | 1.0 | slope | 3 | 2 | 3.1, 3.1 | 6.34 (5.95-7.04) | 0.295 | 0.190-0.191 |
| 9 | 1.1 | slope | 2 | 0 | 3.2, 4.4 | 7.29 (7.06-7.52) | 0.252 | 0.190-0.191 |

Table 5b. `/drone_5` as the second release, one row per run, plan 0.98-1.03 N. Lengths in cm.

| Run | Carriers | Filter | RTF | Phase ms | Pull N | Over at 0.3 s | Inside | Highest or fired | Rise | Found, s |
|---|---|---|---|---|---|---|---|---|---|---|
| R3230 | 8 | slope | 0.255 | 47 | 1.26 | 7.11 | 5.8 | 8.9 | 3.1 | 0.45 |
| R2981 | 8 | slope | 0.241 | 40 | 1.22 | 6.87 | 5.5 | 8.6 | 3.1 | 0.44 |
| R3225 | 8 | slope | 0.234 | 33 | 1.10 | 5.93 | 4.7 | 7.7 | 3.0 | 0.63 |
| R3221 | 8 | slope | 0.228 | 62 | 1.19 | 6.50 | 5.8 | 8.3 | 2.6 | no |
| R3218 | 9 | slope | 0.191 | 45 | 1.14 | 5.95 | 5.0 | 7.6 | 2.7 | no |
| R3227 | 9 | slope | 0.190 | 45 | 1.15 | 6.05 | 5.0 | 7.8 | 2.8 | no |
| R2597 | 8 | ema | 0.261 | 48 | 1.22 | 7.57 | 2.8 | 8.6 | 5.8 | 0.35 |
| R2587 | 8 | ema | 0.257 | 44 | 1.24 | 7.98 | 2.9 | 8.9 | 6.0 | 0.34 |
| R2588 | 8 | ema | 0.254 | 53 | 1.23 | 7.69 | 3.0 | 8.8 | 5.8 | 0.35 |
| R2779 | 8 | ema | 0.250 | 0 | 1.14 | 7.15 | 4.3 | 8.7 | 4.4 | 0.40 |
| R2768 | 8 | ema | 0.224 | 74 | 1.18 | 7.41 | 3.7 | 8.7 | 5.0 | 0.37 |
| R2777 | 8 | ema | 0.221 | 67 | 1.20 | 7.50 | 3.6 | 8.6 | 5.0 | 0.37 |
| R2770 | 8 | ema | 0.221 | 78 | 1.16 | 7.23 | 3.7 | 8.4 | 4.7 | 0.38 |
| R2775 | 8 | ema | 0.221 | 72 | 1.21 | 7.53 | 3.6 | 8.8 | 5.2 | 0.37 |
| R2767 | 8 | ema | 0.218 | 70 | 1.22 | 7.71 | 3.8 | 9.2 | 5.4 | 0.37 |
| R2776 | 8 | ema | 0.218 | 79 | 1.23 | 8.03 | 4.2 | 9.5 | 5.3 | 0.38 |
| R2766 | 8 | ema | 0.217 | 80 | 1.22 | 7.88 | 4.0 | 9.4 | 5.4 | 0.38 |
| R2772 | 8 | ema | 0.216 | 70 | 1.16 | 7.50 | 3.6 | 8.8 | 5.2 | 0.37 |

**The tick phase.** Reading the free curves on a 10 Hz grid at every offset gives how often each would fire.

Table 6. Free curves against the 100 ms tick phase (phase = release to the next tick).

| Curve | Peak cm | Fires | Misses at phases (ms) | Own phase (ms) | Crosses 6 cm (s) | Speed there (m/s) |
|---|---|---|---|---|---|---|
| R3221 (8) | 8.31 | 66 % | 46-78 | 62, missed | 0.298 | 0.179 |
| R3218 (9) | 7.63 | 30 % | 0-2, 34-98 | 45, missed | 0.315 | 0.154 |
| R3227 (9) | 7.79 | 38 % | 36-96 | 45, missed | 0.304 | 0.157 |
| R2951, R2958 (8, first) | 10.05, 10.10 | 100 % | none | fired | 0.24 | - |

The phase holds for a whole run, since the releases are whole seconds apart, and differs between runs (section 5). As
flown under `slope`, 3 of 4 found at 8 and none of 2 at 9, in line with table 6 `[log]`. Six more releases fired just
over the bar (R2981, R3225, R3230 `/drone_5`, R3218 `/drone_1`, `/drone_7`, R3227 `/drone_7`, rises in the appendix),
up to 3 ticks later than the others.

**Fresh poses.** A tick reads the newest pose, at most one pose interval old: at the 120 Hz cap, under the 18 ms of
table 7.

Table 7. The missed runs' own ticks moved later along the same curve, as if each pose had been that much fresher.

| Run | Logged ticks after the release: time s, over cm | No fire when moved by | First fires at |
|---|---|---|---|
| R3221 | 0.26 5.75, 0.36 7.39, 0.46 8.06, 0.56 8.19, 0.86 8.31 | 0-16 ms | 18 ms |
| R3218 | 0.25 4.97, 0.35 6.63, 0.44 7.35, 0.55 7.50, 0.85 7.63 | 0-40 ms | never |
| R3227 | 0.24 4.97, 0.34 6.67, 0.45 7.42, 0.55 7.60, 0.84 7.79 | 0-40 ms | never |

**RTF.** Table 8 is why the wall-clock gate is not the cause. It could only make a miss less likely here: a dropped
tick leaves an older, lower remembered reading, and so a larger jump.

Table 8. The stretch check's gate.

| Evidence | Value | Basis |
|---|---|---|
| Pose rate, ring and drones | 500 Hz in the world, `max_publish_hz` 120 (read back, R3218) | `[log]` |
| Rod end's rate | 60 Hz in the world: S section 9's number | `[log]` |
| Wall time for 10 ms of sim, R3218: p50, p99, max | 53, 65, 70 ms, against the 100 ms gate | `[log]` |
| The same, R3227 | 49, 66, 79 ms | `[log]` |
| The same, R3221 | 42, 61, 68 ms | `[log]` |
| Detections acted on at the first eligible tick of the logged readings | 84 of 88 | `[log]` |
| The other 4 | 3 two-at-once (one loss per tick), 1 reading of exactly 6.00 cm | `[log]` |
| The line's "up" against the rise from the logged reading inside | within 1 cm in 87 of 88 | `[log]` |
| `ema` at RTF 0.216-0.218, `/drone_5` | rises 5.2-5.4 cm | `[log]` |
| Ticks dropped by the gate | not logged, no effect visible in 88 detections | not computed |

The three missed runs are the three lowest RTF of the six, but RTF is not the variable: at 9 carriers it comes with
the lower share, R3225 at a higher RTF than R3221 flew a lower curve, and `ema` at lower RTF never came close. No
9-carrier chain has flown at another RTF, so that last step is `[inf]`.

**Every release, by carrier count, order and drone.** Top = the highest over seen, up to the resize (a lower bound for
the free peak unless no resize came). Rise = the rule's jump at firing. Fired = release to the resize or the line.
Found = releases with a line.

Table 9. The 91 releases at 8 and 9 carriers. 88 found, 3 missed, all `/drone_5` at plan 0.98-1.03 N under `slope`.

| Start | No. | Drone | n | Filter | Arm | Rel. | Plan N | Top cm | Top s | Rise cm | Fired s | Found |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 8 | 1 | /drone_1 | 8 | ema | meas | 14 | 1.27-1.29 | 10.0-13.4 | 0.27-0.40 | 5.6-12.3 | 0.28-0.40 | 14/14 |
| 8 | 1 | /drone_1 | 8 | slope | meas | 14 | 1.28 | 8.9-11.5 | 0.32-0.81 | 4.9-6.4 | 0.34-0.37 | 14/14 |
| 8 | 1 | /drone_1 | 8 | slope | plan | 4 | 1.34 | 16.5-64.8 | 0.39-1.85 | 9.3-15.8 | 0.30-0.41 | 4/4 |
| 8 | 2 | /drone_2 | 8 | ema | meas | 1 | 1.28 | 20.2 | 0.48 | 18.4 | 0.50 | 1/1 |
| 8 | 2 | /drone_5 | 7 | ema | meas | 12 | 1.01-1.03 | 8.3-9.4 | 0.34-0.40 | 4.4-6.0 | 0.34-0.40 | 12/12 |
| 8 | 2 | /drone_5 | 7 | slope | meas | 4 | 1.03 | 7.7-8.9 | 0.43-0.82 | 3.0-3.1 | 0.44-0.63 | 3/4 |
| 8 | 2 | /drone_5 | 8 | ema | meas | 1 | 1.29 | 10.0 | 0.47 | 5.4 | 0.59 | 1/1 |
| 8 | 3 | /drone_3 | 6 | ema | meas | 11 | 1.69-1.74 | 10.3-14.9 | 0.26-0.36 | 8.9-10.8 | 0.27-0.37 | 11/11 |
| 8 | 3 | /drone_3 | 6 | slope | meas | 3 | 1.74 | 11.2-12.4 | 0.32-0.34 | 7.9-8.7 | 0.33-0.34 | 3/3 |
| 8 | 3 | /drone_3 | 7 | ema | meas | 1 | 1.58 | 11.0 | 0.29 | 9.8 | 0.30 | 1/1 |
| 8 | 3 | /drone_3 | 7 | slope | meas | 1 | 1.57 | 12.7 | 0.35 | 7.6 | 0.36 | 1/1 |
| 8 | 4 | /drone_7 | 5 | ema | meas | 10 | 1.43-1.50 | 10.6-12.9 | 0.29-0.37 | 7.0-9.0 | 0.30-0.38 | 10/10 |
| 8 | 4 | /drone_7 | 5 | slope | meas | 3 | 1.50 | 9.7-10.3 | 0.31-0.33 | 6.5-7.2 | 0.33-0.34 | 3/3 |
| 8 | 4 | /drone_7 | 6 | ema | meas | 1 | 1.74 | 14.7 | 0.47 | 13.2 | 0.48 | 1/1 |
| 8 | 4 | /drone_7 | 6 | slope | meas | 1 | 1.32 | 11.4 | 0.35 | 6.8 | 0.36 | 1/1 |
| 8 | 4 | /drone_7 | 7 | ema | meas | 1 | 1.06 | 8.9 | 0.39 | 4.5 | 0.39 | 1/1 |
| 8 | 5 | /drone_2 | 8 | ema | meas | 1 | 1.81 | 11.5 | 0.28 | 10.2 | 0.30 | 1/1 |
| 9 | 1 | /drone_1 | 9 | slope | meas | 2 | 1.14 | 8.2-9.0 | 0.33-0.54 | 3.2-5.5 | 0.34-0.54 | 2/2 |
| 9 | 2 | /drone_5 | 8 | slope | meas | 2 | 0.98 | 7.6-7.8 | 0.83-0.84 | - | - | 0/2 |
| 9 | 3 | /drone_3 | 8 | slope | meas | 2 | 1.43 | 11.0 | 0.33 | 7.3-7.7 | 0.34 | 2/2 |
| 9 | 4 | /drone_7 | 7 | slope | meas | 2 | 0.95 | 8.7-8.9 | 0.53-0.54 | 3.0-3.1 | 0.54-0.55 | 2/2 |

## 3. After a miss

Table 10. The flight after the miss, against the detected repeats of the same cell.

| Item | R3221 (8) | R3218 (9) | R3227 (9) | R2981, R3225, R3230 |
|---|---|---|---|---|
| Release to LAND (s), the ghost in the model | 243.3 | 243.2 | 244.4 | out in 0.44-0.63 |
| Over 6 cm, from and until (s) | 0.30, 1.33 | 0.32, 1.30 | 0.30, 1.31 | - |
| Over 3 cm, from and until (s) | 0.17, 1.56 | 0.19, 1.55 | 0.18, 1.56 | - |
| Next hump (cm, at s) | 3.93, 3.26 | 3.94, 3.26 | 3.86, 3.27 | - |
| Over, +1.5 s to LAND, p50 / p99 (cm) | 0.45 / 1.74 | 0.43 / 1.81 | 0.44 / 1.83 | - |
| Plan pull on the ghost, before, +10 s (N) | 1.03, 1.05 | 0.98, 0.99 | 0.98, 0.99 | - |
| Measured pull, before, after (N) | 1.19, 0.016 | 1.14, 0.016 | 1.15, 0.017 | - |
| Ghost's throttle, before, after | 0.528, 0.509 | 0.525, 0.509 | 0.526, 0.509 | - |
| Ghost's reference error, p50 / p99 (cm) | 0.46 / 1.44 | 0.44 / 1.56 | 0.45 / 1.55 | - |
| This release's peak tilt, settled (deg) | 1.67, 0.08 | 1.45, 0.40 | 1.50, 0.40 | 1.62-1.71, 0.04-0.05 |
| N4 RMSE (mm), tilt (deg) | 6.52, 1.32 | 6.11, 1.20 | 6.11, 1.23 | 6.16-6.48, 0.93-0.96 |
| Integral at LAND, x, y, z (N) | 0.17, 0.17, -0.96 | 0.22, 0.08, -0.96 | 0.22, 0.08, -0.96 | 0, 0, -0.55 to -0.58 |
| Ring height at LAND (m) | 0.9984 | 0.9984 | 0.9984 | 0.9989-0.9991 |
| Over in LAND while the check is on (cm) | 1.34 | 1.17 | 0.96 | - |
| Over once the ring is down, check off (cm) | 18.64 | 17.45 | 15.85 | - |
| Next release, `/drone_3`: plan, pull (N) | 1.57, 1.93 | 1.43, 1.81 | 1.43, 1.80 | 1.74, 1.82-2.01 |
| The ghost's ATTACH | 50 s timeout | 50 s timeout | 50 s timeout | rejoins |
| The ghost's height at the end (m) | 0.100 | 0.100 | 0.100 | - |

- **The slot** `[log]`. The planner keeps planning for it, with its full pull. The ghost follows the reference, level,
  with its rod hanging. The others make up the missing force through the measured pull and the model integral (table
  10). Tilt stays inside its bars, and N4's position misses as it does in the detected runs.
- **The next releases** `[log]`. Their plan is computed with the ghost in, so plan and real pull part (table 10). All
  were still found. The drop rule also counts the ghost as a survivor (`_drop_due`), which matters only near its
  limits, not at these sizes `[inf]`.
- **The ATTACH** `[log]`. `_do_attach` goes to `_attach_ocp`, which returns False with no line because the drone is
  still in `slot2drone`. The runner waits out its 50 s, then the next ATTACH goes normally. The chain ends with 7 of 8
  or 8 of 9 attached.
- **Does anything catch it?** No, in all 3 `[log]`. After the first hump the reading goes back inside 6 cm, so a
  second crossing would fire, but it never crosses. In LAND the reading stays small until the ring is down and the
  check is off.
- **Could a later check?** The marker check, if on: it reads the rod end, not the pull. The twin's joint state already
  says "tether joint ... detached" within 0-34 ms of the release, and the planner logs it and acts only for a rejoin:
  twin only, useful as an audit line, not as a detector. The measured pull reads the loss too (option D).

**Real flights.** Table 11 has every line on the rig. Real pulls are far above the draw, so every clean release jumped
well past the bar `[log]`. The one exposure is the T's rider: the stretch check often misses it in the twin (table
11b). No rider release has flown on the rig (t3p_3 released the wrong quad), and the rig's chain files carry the
marker check. Wed 4 Nov's `rel1`, `rel2` fly `m1_release_tip.yaml`, nearest twin cell in table 2 (the 1.15 kg row):
not exposed `[inf]`.

Table 11. Every UNANNOUNCED DETACH line on the rig.

| Flight | n | Rod m | Ring kg | Plan N | Check | Over cm | Up cm | What it was |
|---|---|---|---|---|---|---|---|---|
| 1008 r303_4 | 4 | 0.55 | 0.86 | 1.62 | stretch | 14 | 14 | planned release |
| 1008 r304_3 | 4 | 0.55 | 0.86 | 1.89 | stretch | 20 | 19 | planned release |
| 1008 r305 | 4 | 0.55 | 0.86 | 1.58 | stretch | 15 | 15 | planned release, circle |
| 1014 t5_2 | 3 | 0.55 | 0.86 | 3.37 | stretch | 74 | 74 | magnet let go |
| 1014 t5_2 | 3 | 0.55 | 0.86 | 3.42 | stretch | 9 | 7 | a second carrier, ring at 81 deg |
| 1015 a3 | 3 | 0.55 | 0.86 | 2.85 | stretch | 9 | 3 | FALSE: wrong layout, pretension, ring on floor |
| 1021 c200_4 | 3 | 0.55 | 0.86 | 3.18 | stretch | 286 | 287 | FALSE: 183 s after the DISARM |
| 1021 c2q_noxy_3 | 2 | 0.55 | 0.86 | 5.05 | stretch | 39 | 41 | magnet let go |
| 1021 c2q_p11p5_2 | 2 | 0.55 | 0.86 | 5.06 | stretch | 59 | 58 | magnet let go |
| 1021 s200 | 3 | 0.55 | 0.86 | 3.32 | stretch | 32 | 32 | magnet let go |
| 1021 s200_2 | 3 | 0.55 | 0.86 | 3.16 | stretch | 27 | 27 | magnet let go |
| 1022 h2 | 2 | 0.55 | 0.86 | 4.98 | stretch | 48 | 48 | magnet let go |
| 1022 t3p_3 | 3 | 0.55 | 0.86 | 4.92 | stretch | 39 | 40 | planned release, not the rider |
| 1029 h4 | 4 | 1.0 | 1.15 | 3.38 | stretch | 26 | 23 | magnet let go |
| 1029 ch1 | 4 | 1.0 | 1.15 | 3.46 | stretch | 24 | 23 | magnet let go |
| 1029 ch1b | 4 | 1.0 | 1.15 | 5.39 | stretch | 41 | 41 | magnet let go |
| 1029 ch1b_2 | 4 | 1.0 | 1.15 | 7.47 | marker | 31 off | - | magnet let go, 105 ms |
| 1030 h3t | 3 | 1.0 | 1.15 | 4.24 | stretch | 51 | 48 | a carrier lost in LAND |

Table 11b. Low-pull releases in the twin (plan under 0.6 N, mostly the T's rider).

| Check on | Releases | Found | Not found |
|---|---|---|---|
| Stretch only | 9 | 4 | 5 (R2258, R2296, R2297, R2315, R2317) |
| Stretch and marker | 24 | 24, all by the marker | 0 |

Table 12. Which experiments in `docs/sim_runs_todo.md` are exposed.

| Experiment | Release | Lowest plan N | Exposed | Basis |
|---|---|---|---|---|
| X3b C8 chain, X3 `s8_chain4back` | `/drone_5` from 7 | 1.03 | once a run | `[log]` 1 miss of 4, `slope` |
| X3b N9 chain | `/drone_5` from 8 | 0.98 | once a run | `[log]` 2 misses of 2 |
| X3 chain4b2b (+ orbit), fig8laps, relblend | `/drone_5` from 7 | 1.01-1.03 | once a run | `[inf]` only at `ema` |
| X3 chain4task, fig8rel2 | `/drone_5` from 7 | 1.03 | once a run | `[inf]` only at `ema` |
| X3 `s8_chain2back` | `/drone_5` from 7 | 1.02 computed | once a run | `[inf]` not flown |
| X3 `s8_relay` | `/drone_5`, `/drone_7`, each from 7 | 1.03, 1.06 | twice a run | `[inf]` only at `ema` |
| X3 `chain4back_pairs`, `adjpair` | together, from the even ring | 1.28 | no | `[inf]` |
| X1 at 8 quads | first, from 8 | 1.28 | no | `[log]` 14 of 14, every phase |
| X6 at 8, 6, 4 quads | one loss on the figure-8 | 1.17 at 8 | no, a second loss yes | `[log]` plan, `[inf]` rest |
| X4 relay | announced DETACH | - | no | `[log]` not the detector |
| X2 full chain | marker on | - | no | `[log]` |
| X11 | re-flies the X3 set | as X3 | as X3 | `[inf]` |

If the X3 set flies as it stands on the final tree, about a third of the runs with one such release void by the card's
rule `[inf]`.

## 4. Options

Table 13. How close moments that are not a release come. The highest reading held for two ticks in each log, in
flight, from TAKEOFF to DISARM, the real-release windows taken out. The twin's rods are rigid, the rig's are cables.

| Flights | Logs | Ticks | p50 cm | p99 cm | Max cm | The max |
|---|---|---|---|---|---|---|
| Twin, 8 carriers, 1 m | 50 | 81,439 | 0.11 | 0.82 | 0.92 | R3407, the relay |
| Twin, 9 carriers, 1 m | 4 | 6,937 | 0.28 | 0.53 | 0.53 | |
| Twin, 4 carriers, 1 m | 283 | 153,723 | 0.09 | 0.45 | 1.16 | R3410, told a rod 1 cm short |
| Twin, 3 carriers, 1 m | 370 | 326,355 | 0.11 | 1.39 | 2.14 | R2731, a catch off the plate |
| Twin, 3 carriers, 0.55 m | 585 | 451,159 | 0.09 | 0.24 | 4.24 | R2293, an injected ring-pose glitch |
| Rig, 1 m | 24 | 6,595 | 0.36 | 1.57 | 1.57 | sq1, sq6b_7 |
| Rig, 0.55 m | 90 | 31,454 | 0.69 | 2.10 | 3.86 | c2q_noxy_4: SLOT OFFSET 29 deg logged, steady |

Table 14. Each rule replayed on all of those logs (twin 1,523, rig 115), with its reach on the three missed curves:
the share of the tick phase that fires, then the time to fire from the release (lowest median to worst, s). r200006 on
the rig, a real release before the rule existed, hits them all and is not counted. The rig's 2 false fires of today's
rule (table 11) sit in the windows taken out.

| Rule | R3221 | R3218 | R3227 | Fires by (s) | False, twin | False, rig |
|---|---|---|---|---|---|---|
| Today: over 6 cm, jump 3 cm | 66 % | 30 % | 38 % | 0.41-0.85 | 0 | 0 |
| A. Jump 2.5 cm | 91 % | 54 % | 61 % | 0.42-0.86 | 0 | 0 |
| A. Jump 2 cm | 100 % | 78 % | 89 % | 0.43-0.89 | 0 | 0 |
| A. Jump 1.5 cm | 100 % | 100 % | 100 % | 0.43-0.60 | 0 | 0 |
| A. No jump, two ticks over 6 cm | 100 % | 100 % | 100 % | 0.43-0.50 | 0 | 0 |
| B. 3 cm rise inside 0.3 or 0.5 s, over 6 cm | 100 % | 100 % | 100 % | 0.43-0.50 | 0 | 0 |
| C. Over 5 cm, jump 3 cm | 100 % | 82 % | 91 % | 0.38-0.83 | 0 | 0 |
| C. Over 4 cm, jump 3 cm | 100 % | 100 % | 100 % | 0.35-0.40 | 1 (R2293) | 0 (3.86 cm seen) |
| E. Over 3 cm for 1 s | 100 % | 100 % | 100 % | 1.17-1.33 | 1 (R2293) | 1 (c2q_noxy_4) |
| E. Over 2 cm for 2 s | 0 % | 0 % | 0 % | - | 1 (R1767) | 1 (c2q_noxy_4) |

Table 15. Option D, the measured pull: held under 0.3 N for 0.3 s with the plan at 0.5 N or more, outside the real
releases.

| Flights | Logs | With such a stretch |
|---|---|---|
| Twin, 8 and 9 carriers | 50 | 0 |
| Twin, 3 and 4 carriers, from 23 Oct, 1 m | 463 | 48 |
| Rig, with a measured pull logged | 39 | 0 |

Table 16. Option F, the marker check, as flown.

| Item | Value |
|---|---|
| Twin runs with it on | 43, at 2 to 4 carriers, RTF 0.228-0.607 |
| Releases found by the marker | 30 of 30, plan pull 0.005-4.9 N |
| Release to resize | 29 runs 0.26-0.38 s, median 0.30, 1 dropped the ring (R2320) |
| The same, plan under 0.6 N | 24: 0.26-0.38 s |
| The same, plan over 2 N | 5: 0.29-0.36 s |
| Fires in the 15 twin runs with no release | 0 |
| WOULD-FIRE | 1 run (R2320) |
| NOT armed lines | 2 runs (R2287, R2288), both void runs of 22 Oct (harness faults) |
| Rig flights with it on | 5: 1 true marker fire, 3 where the stretch fired first, 0 false, 0 would-fire |

Table 17. The numbers behind the options.

| Option | Item | Value |
|---|---|---|
| A, jump 1.5 cm | Margin on the lowest free peak (R3218) | 0.13 cm |
| B | Lowest plan pull it reaches, margin on R3218 | 0.72 N, 1.63 cm |
| B | a3 at its firing tick: rise in the last 0.5 s, against the 3 cm | 2.6 cm |
| C, 4 cm | Free peak that always fires, plan pull, margin on R3218 | 7 cm, 0.84 N, 0.63 cm |
| C | Readings outside a release: twin 8-9, rig 1 m, rig 0.55 m, injected glitch | 0.92, 1.57, 3.86, 4.24 cm |
| D | Measured pull under 0.3 N after the release, 34 at 8-9, then the 0.3 s hold | 0.25-0.29 s, 0.55-0.59 s |
| D | Attached, plan about 2 N, pull near zero: longest (R2680) | 65 s |
| E | The ghost over 3 cm, first hump | 1.4 s |
| F | Rod-end sample spacing at 9 carriers, wall, p50 and p99 | 82-88, 109-111 ms |
| G | Flights for 2 valid 9-carrier chains, at ~25 min each `[inf]` | 5 to 7 |

**A. A lower jump** (the literal in the code). It moves the draw's top edge, with a thin margin (table 17). The false
count is 0 because no moment outside a release reaches the 6 cm bar, so the logs neither support nor oppose it: the
jump is what stops a steady mocap bias firing, and none reached the bar. It needs a code change and reaches the rig.
Risk: unknown, not low.

**B. A longer window** (the jump measured from the lowest reading in the last 0.3-0.5 s, behind a switch). It removes
the tick phase at the root: the reference sits near the rest reading, not near the bar, so a free peak over the bar
fires at every phase (tables 4, 17). It keeps the bar and the two ticks, and it is stricter on slow ramps than today's
unlimited memory: a3's false fire would not have fired at its tick. Cost: new code in a gate that can drop the ring,
so a critic, and it must make the final tree.

**C. A lower `detach_detect_m`, typed for the 8 and 9 chain cells only** (a copy of `configs/twin/s_chain_pair.yaml`).
No code, no RTF dependence, the fastest of all. At 8 and 9 carriers no reading outside a release comes near it. Not
for the rig: real cables and a ring-pose glitch come too close (tables 13, 17). The thesis must then say that 8 and 9
carriers use a bar the real flights do not.

**D. Measured pull (or thrust, which it is computed from).** Clean in the twin at 8 and 9 (table 10), and clean on the
rig so far (table 15). But a carrier can stay attached and carry nothing while the plan asks for a full share: long
stretches of that in the T and catch-off-the-plate runs (R2680, `u_slot_*`, `x1_sqT_*`, table 17). It needs the
trackers' pull sent to the planner, a critic, and it cannot see the rider. Not recommended.

**E. A time limit on an unexplained long rod.** The ghost reads long only briefly (table 10), then the tracker brings
it back, so the limit must be short, and then a steady bias on a real flight passes it (c2q_noxy_4, through its lift).
Slowest of all, and the only false fire on a real flight in table 14. Not recommended.

**F. The marker check on for the 8 and 9 chains**, typed (`tip_detect_m` 0.04 in that copy of the file, `tether_tips`
is already on in these cells). No code. The same two checks the rig's release flights fly, so no bar differs from a
real flight. It reads the rod end, which falls whatever the pull `[inf]`, and the time to resize shows no trend with
pull (table 16) `[log]`. It also covers the rider. It has passed a critic (`2026-10-21_tip_detect`). Unknown: it has
never armed above 4 carriers. At 9 carriers' RTF the rod-end samples come close to the 0.1 s gate for "another marker
is fresh" (table 17), and that is where S section 9's number applies. A late one costs only the next sample, logged
WOULD-FIRE `[inf]`, and the stretch check stays on behind it, so it cannot find less than today.

**G. Leave it and report the limit.** No change and no risk of a new false fire. The chains stay a draw (table 6):
N9's 2 valid runs take several flights (table 17), and X3's exposed cells void at about a third `[inf]`. The thesis
says the stretch check alone has this reach (table 4), and that it is simulation at 8 and 9 carriers only. S section
9's cause should be corrected either way.

### Recommendation

**Hold the re-flies of R3218, R3221, R3227, and propose F for Wesley's word**: `tip_detect_m` 0.04 typed for the 8 and
9 carrier chain cells only, with the stretch check, `detach_detect_m` and the rig all unchanged. Of the options that
end the draw it is the only one that needs no code and leaves no bar different from a real flight, it does not depend
on the share, and table 16 has every release found with no false fire. Said plainly, the claim then flies two checks
where the misses flew one, so the chain arms of C8 and N9 re-fly with it (one tree per claim table, and they re-fly on
the final tree anyway).

First one exploratory run, no repeat: `x3b_s9_chain4back_rt` with the marker on, at the lab's RTF. **Falsified by:** a
marker not arming, a release not found by the marker, a fire with no release, or a WOULD-FIRE that holds past the next
sample. If falsified, the fallback is C at 4 cm, then B. Cost: a file copy and one ~25 min slot. The slots are on X4
until tonight (`docs/SESSION_STATE.md`).

Whatever is picked, two small lines would save a void next time, code and so also Wesley's word: a WARN when an ATTACH
names a carrier still in the slots, and a twin-only WARN when the joint reports detached and no resize follows within
2 s.

## 5. Method, and what could not be computed

The cable reading is the planner log's `len<slot>` at each tick, and `logs/run.csv` (about 21 ms sim) plus the tracker
log's attitude between ticks. The two agree (median 0.48 mm RMS on 250 checked curves). Firing readings come from the
launch log's line, since the row after firing has lost the slot. The phase sweep reads the high-rate curve on a 100 ms
grid at 1 ms steps. False-fire replays run row by row with today's reset rules, the measured rods, and the
real-release windows (RELEASE, DETACH, the joint's state, rig log lines) taken out. Measured pull is 0.55 kg x |acm|
from the tracker log.

| Could not compute | Why |
|---|---|
| A free peak for `/drone_3`, `/drone_7`, or `/drone_5` at `ema` | Always resized first |
| The filter's effect alone | No pair on one tree, only the two trees' runs |
| Whether the gate dropped a tick | It logs nothing. 88 detections show no effect |
| A 9-carrier chain at another RTF, or with no ghost | Never flown: both flew the ghost from the second release on |
| The rod end's path at 8 and 9 carriers | Not in the run logs, Gazebo's state log not parsed. F has no data there |
| How even the tick phase really is | 34 first releases span 0-88 ms, median 64. The percentages assume even |
| Free curves on the rig | Every real release was acted on |
| The marker's false-fire rate on the rig | 5 flights only. Card 2026-10-21 has the bag replays |

## Appendix: every release at 8 and 9 carriers

Start = carriers at the start, No. = its order in the run, n = planned fleet before it, Filt = e `ema` or s `slope`,
Pull = measured before the release, Inside = last reading inside 6 cm, Top = highest before the resize, Free y = no
resize cut the curve. Length in cm, force in N, time in s.

| Run | Cell | Start | No. | Drone | n | Filt | RTF | Plan N | Pull N | Inside | Top | Rise | Top s | Fired s | Free |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| R2587 | s8_chain4 | 8 | 1 | 1 | 8 | e | 0.26 | 1.28 | 1.49 | 3.6 | 10.4 | 7.3 | 0.33 | 0.35 | n |
| R2587 | s8_chain4 | 8 | 2 | 5 | 7 | e | 0.26 | 1.03 | 1.24 | 2.9 | 8.9 | 6.0 | 0.34 | 0.34 | n |
| R2587 | s8_chain4 | 8 | 3 | 3 | 6 | e | 0.26 | 1.74 | 1.97 | 5.0 | 14.8 | 9.8 | 0.34 | 0.34 | n |
| R2587 | s8_chain4 | 8 | 4 | 7 | 5 | e | 0.26 | 1.50 | 1.71 | 4.2 | 12.9 | 8.7 | 0.34 | 0.34 | n |
| R2588 | s8_chain4back | 8 | 1 | 1 | 8 | e | 0.25 | 1.28 | 1.47 | 3.5 | 10.2 | 6.8 | 0.34 | 0.35 | n |
| R2588 | s8_chain4back | 8 | 2 | 5 | 7 | e | 0.25 | 1.03 | 1.23 | 3.0 | 8.7 | 5.8 | 0.35 | 0.35 | n |
| R2588 | s8_chain4back | 8 | 3 | 3 | 6 | e | 0.25 | 1.74 | 1.80 | 4.5 | 13.4 | 8.9 | 0.35 | 0.35 | n |
| R2588 | s8_chain4back | 8 | 4 | 7 | 5 | e | 0.25 | 1.50 | 1.59 | 3.8 | 11.6 | 7.8 | 0.35 | 0.35 | n |
| R2597 | s8_chain4back | 8 | 1 | 1 | 8 | e | 0.26 | 1.28 | 1.55 | 3.7 | 10.8 | 7.5 | 0.33 | 0.35 | n |
| R2597 | s8_chain4back | 8 | 2 | 5 | 7 | e | 0.26 | 1.03 | 1.22 | 2.8 | 8.5 | 5.8 | 0.34 | 0.35 | n |
| R2597 | s8_chain4back | 8 | 3 | 3 | 6 | e | 0.26 | 1.74 | 1.84 | 4.5 | 13.3 | 9.1 | 0.34 | 0.35 | n |
| R2597 | s8_chain4back | 8 | 4 | 7 | 5 | e | 0.26 | 1.50 | 1.65 | 4.0 | 11.8 | 8.1 | 0.34 | 0.35 | n |
| R2766 | s8_chain4back | 8 | 1 | 1 | 8 | e | 0.22 | 1.28 | 1.49 | 4.9 | 11.3 | 6.4 | 0.38 | 0.38 | n |
| R2766 | s8_chain4back | 8 | 2 | 5 | 7 | e | 0.22 | 1.03 | 1.22 | 4.0 | 9.4 | 5.4 | 0.38 | 0.38 | n |
| R2766 | s8_chain4back | 8 | 3 | 3 | 6 | e | 0.22 | 1.74 | 1.81 | 1.3 | 10.3 | 9.8 | 0.26 | 0.28 | n |
| R2766 | s8_chain4back | 8 | 4 | 7 | 5 | e | 0.22 | 1.50 | 1.62 | 5.5 | 12.2 | 7.0 | 0.36 | 0.38 | n |
| R2767 | s8_chain4back | 8 | 1 | 1 | 8 | e | 0.22 | 1.28 | 1.47 | 4.3 | 10.6 | 6.6 | 0.36 | 0.37 | n |
| R2767 | s8_chain4back | 8 | 2 | 5 | 7 | e | 0.22 | 1.03 | 1.22 | 3.8 | 9.1 | 5.4 | 0.37 | 0.37 | n |
| R2767 | s8_chain4back | 8 | 3 | 3 | 6 | e | 0.22 | 1.73 | 1.87 | 1.1 | 10.7 | 10.0 | 0.26 | 0.27 | n |
| R2767 | s8_chain4back | 8 | 4 | 7 | 5 | e | 0.22 | 1.50 | 1.71 | 5.4 | 12.9 | 7.7 | 0.36 | 0.37 | n |
| R2768 | s8_chain4b2b | 8 | 1 | 1 | 8 | e | 0.22 | 1.28 | 1.52 | 4.7 | 11.4 | 6.8 | 0.37 | 0.38 | n |
| R2768 | s8_chain4b2b | 8 | 2 | 5 | 7 | e | 0.22 | 1.03 | 1.18 | 3.7 | 8.5 | 5.0 | 0.37 | 0.37 | n |
| R2768 | s8_chain4b2b | 8 | 3 | 3 | 6 | e | 0.22 | 1.74 | 1.98 | 1.3 | 11.3 | 10.8 | 0.26 | 0.27 | n |
| R2768 | s8_chain4b2b | 8 | 4 | 7 | 5 | e | 0.22 | 1.50 | 1.59 | 4.8 | 11.9 | 7.1 | 0.37 | 0.37 | n |
| R2770 | s8_chain4b2b_orbit | 8 | 1 | 1 | 8 | e | 0.22 | 1.28 | 1.49 | 4.8 | 11.1 | 6.6 | 0.36 | 0.38 | n |
| R2770 | s8_chain4b2b_orbit | 8 | 2 | 5 | 7 | e | 0.22 | 1.01 | 1.16 | 3.7 | 8.3 | 4.7 | 0.37 | 0.38 | n |
| R2770 | s8_chain4b2b_orbit | 8 | 3 | 3 | 6 | e | 0.22 | 1.69 | 1.92 | 1.3 | 11.3 | 10.0 | 0.28 | 0.28 | n |
| R2770 | s8_chain4b2b_orbit | 8 | 4 | 7 | 5 | e | 0.22 | 1.43 | 1.63 | 5.4 | 12.2 | 7.1 | 0.36 | 0.38 | n |
| R2772 | s8_fig8rel2 | 8 | 1 | 1 | 8 | e | 0.22 | 1.27 | 1.52 | 5.4 | 11.1 | 6.0 | 0.37 | 0.39 | n |
| R2772 | s8_fig8rel2 | 8 | 2 | 5 | 7 | e | 0.22 | 1.03 | 1.16 | 3.6 | 8.6 | 5.2 | 0.35 | 0.37 | n |
| R2773 | s8_chain4back_pairs | 8 | 1 | 1 | 8 | e | 0.22 | 1.29 | 1.53 | 4.4 | 10.0 | 5.6 | 0.39 | 0.39 | n |
| R2773 | s8_chain4back_pairs | 8 | 2 | 5 | 8 | e | 0.22 | 1.29 | 1.49 | 4.2 | 10.0 | 5.4 | 0.47 | 0.59 | n |
| R2773 | s8_chain4back_pairs | 8 | 3 | 3 | 6 | e | 0.22 | 1.74 | 1.99 | 1.4 | 11.0 | 9.6 | 0.29 | 0.29 | n |
| R2773 | s8_chain4back_pairs | 8 | 4 | 7 | 6 | e | 0.22 | 1.74 | 2.00 | 1.5 | 14.7 | 13.2 | 0.47 | 0.48 | n |
| R2774 | s8_adjpair | 8 | 1 | 1 | 8 | e | 0.22 | 1.28 | 1.50 | 1.6 | 13.4 | 12.3 | 0.27 | 0.28 | n |
| R2774 | s8_adjpair | 8 | 2 | 2 | 8 | e | 0.22 | 1.28 | 1.47 | 1.7 | 20.2 | 18.4 | 0.48 | 0.50 | n |
| R2775 | s8_chain4back_relblend | 8 | 1 | 1 | 8 | e | 0.22 | 1.28 | 1.41 | 4.2 | 10.4 | 6.4 | 0.36 | 0.37 | n |
| R2775 | s8_chain4back_relblend | 8 | 2 | 5 | 7 | e | 0.22 | 1.03 | 1.21 | 3.6 | 8.7 | 5.2 | 0.36 | 0.37 | n |
| R2775 | s8_chain4back_relblend | 8 | 3 | 3 | 6 | e | 0.22 | 1.73 | 1.98 | 5.9 | 14.9 | 9.4 | 0.36 | 0.37 | n |
| R2775 | s8_chain4back_relblend | 8 | 4 | 7 | 5 | e | 0.22 | 1.50 | 1.65 | 5.0 | 12.4 | 7.7 | 0.36 | 0.38 | n |
| R2775 | s8_chain4back_relblend | 8 | 5 | 2 | 8 | e | 0.22 | 1.81 | 2.03 | 1.9 | 11.5 | 10.2 | 0.28 | 0.30 | n |
| R2776 | s8_relay | 8 | 1 | 1 | 8 | e | 0.22 | 1.28 | 1.49 | 4.8 | 10.9 | 6.4 | 0.36 | 0.38 | n |
| R2776 | s8_relay | 8 | 2 | 5 | 7 | e | 0.22 | 1.03 | 1.23 | 4.2 | 9.2 | 5.3 | 0.36 | 0.38 | n |
| R2776 | s8_relay | 8 | 3 | 3 | 7 | e | 0.22 | 1.58 | 1.83 | 1.6 | 11.0 | 9.8 | 0.29 | 0.30 | n |
| R2776 | s8_relay | 8 | 4 | 7 | 7 | e | 0.22 | 1.06 | 1.21 | 4.5 | 8.9 | 4.5 | 0.39 | 0.39 | n |
| R2777 | s8_chain4task | 8 | 1 | 1 | 8 | e | 0.22 | 1.28 | 1.46 | 4.3 | 10.7 | 6.6 | 0.35 | 0.37 | n |
| R2777 | s8_chain4task | 8 | 2 | 5 | 7 | e | 0.22 | 1.03 | 1.20 | 3.6 | 8.4 | 5.0 | 0.35 | 0.37 | n |
| R2777 | s8_chain4task | 8 | 3 | 3 | 6 | e | 0.22 | 1.74 | 1.97 | 5.6 | 14.7 | 9.6 | 0.35 | 0.37 | n |
| R2777 | s8_chain4task | 8 | 4 | 7 | 5 | e | 0.22 | 1.50 | 1.61 | 4.9 | 12.0 | 7.3 | 0.36 | 0.37 | n |
| R2779 | s8_chain4back_fig8laps | 8 | 1 | 1 | 8 | e | 0.25 | 1.27 | 1.48 | 5.8 | 11.4 | 5.6 | 0.40 | 0.40 | n |
| R2779 | s8_chain4back_fig8laps | 8 | 2 | 5 | 7 | e | 0.25 | 1.01 | 1.14 | 4.3 | 8.7 | 4.4 | 0.40 | 0.40 | n |
| R2779 | s8_chain4back_fig8laps | 8 | 3 | 3 | 6 | e | 0.25 | 1.72 | 1.96 | 2.4 | 12.7 | 10.3 | 0.30 | 0.30 | n |
| R2779 | s8_chain4back_fig8laps | 8 | 4 | 7 | 5 | e | 0.25 | 1.43 | 1.64 | 1.7 | 10.6 | 9.0 | 0.29 | 0.30 | n |
| R2950 | x1_h8_ours | 8 | 1 | 1 | 8 | s | 0.22 | 1.28 | 1.47 | 4.2 | 9.4 | 5.4 | 0.36 | 0.37 | n |
| R2951 | x1_h8_nors | 8 | 1 | 1 | 8 | s | 0.22 | 1.28 | 1.44 | 4.0 | 10.1 | 5.0 | 0.69 | 0.37 | y |
| R2953 | x1_h8_d10 | 8 | 1 | 1 | 8 | s | 0.22 | 1.28 | 1.34 | 3.8 | 9.7 | 4.9 | 0.81 | 0.37 | y |
| R2954 | x1_h8_d20 | 8 | 1 | 1 | 8 | s | 0.25 | 1.28 | 1.58 | 4.4 | 11.5 | 5.9 | 0.56 | 0.36 | y |
| R2955 | x1_h8_plan | 8 | 1 | 1 | 8 | s | 0.24 | 1.34 | nan | 6.0 | 16.5 | 11.1 | 0.39 | 0.40 | n |
| R2956 | x1_h8_nplan | 8 | 1 | 1 | 8 | s | 0.23 | 1.34 | nan | 1.7 | 64.8 | 9.8 | 1.85 | 0.30 | y |
| R2957 | x1_h8_ours | 8 | 1 | 1 | 8 | s | 0.23 | 1.28 | 1.43 | 3.9 | 9.0 | 5.3 | 0.35 | 0.37 | n |
| R2958 | x1_h8_nors | 8 | 1 | 1 | 8 | s | 0.23 | 1.28 | 1.41 | 3.9 | 10.1 | 5.1 | 0.76 | 0.37 | y |
| R2959 | x1_h8_d05 | 8 | 1 | 1 | 8 | s | 0.23 | 1.28 | 1.48 | 3.4 | 10.5 | 5.8 | 0.60 | 0.35 | y |
| R2960 | x1_h8_d10 | 8 | 1 | 1 | 8 | s | 0.22 | 1.28 | 1.45 | 4.0 | 10.5 | 5.3 | 0.73 | 0.37 | y |
| R2961 | x1_h8_d20 | 8 | 1 | 1 | 8 | s | 0.23 | 1.28 | 1.56 | 3.7 | 11.1 | 6.0 | 0.67 | 0.35 | y |
| R2962 | x1_h8_plan | 8 | 1 | 1 | 8 | s | 0.23 | 1.34 | nan | 1.8 | 16.6 | 15.8 | 0.39 | 0.41 | n |
| R2963 | x1_h8_nplan | 8 | 1 | 1 | 8 | s | 0.25 | 1.34 | nan | 1.8 | 64.7 | 9.3 | 1.85 | 0.30 | y |
| R2964 | x1_h8_d05 | 8 | 1 | 1 | 8 | s | 0.26 | 1.28 | 1.51 | 3.6 | 10.7 | 5.8 | 0.59 | 0.35 | y |
| R2981 | s8_chain4back | 8 | 1 | 1 | 8 | s | 0.24 | 1.28 | 1.51 | 3.1 | 9.1 | 6.4 | 0.32 | 0.34 | n |
| R2981 | s8_chain4back | 8 | 2 | 5 | 7 | s | 0.24 | 1.03 | 1.22 | 5.5 | 8.5 | 3.1 | 0.43 | 0.44 | n |
| R2981 | s8_chain4back | 8 | 3 | 3 | 6 | s | 0.24 | 1.74 | 1.82 | 3.7 | 11.2 | 7.9 | 0.33 | 0.34 | n |
| R2981 | s8_chain4back | 8 | 4 | 7 | 5 | s | 0.24 | 1.50 | 1.60 | 3.4 | 9.7 | 6.5 | 0.33 | 0.34 | n |
| R3221 | s8_chain4back | 8 | 1 | 1 | 8 | s | 0.23 | 1.28 | 1.50 | 3.9 | 9.7 | 5.8 | 0.37 | 0.37 | n |
| R3221 | s8_chain4back | 8 | 2 | 5 | 7 | s | 0.23 | 1.03 | 1.19 | 5.8 | 8.3 | miss | 0.82 | - | y |
| R3221 | s8_chain4back | 8 | 3 | 3 | 7 | s | 0.23 | 1.57 | 1.93 | 5.3 | 12.7 | 7.6 | 0.35 | 0.36 | n |
| R3221 | s8_chain4back | 8 | 4 | 7 | 6 | s | 0.23 | 1.32 | 1.74 | 5.0 | 11.4 | 6.8 | 0.35 | 0.36 | n |
| R3225 | s8_chain4back | 8 | 1 | 1 | 8 | s | 0.23 | 1.28 | 1.49 | 3.0 | 8.9 | 6.1 | 0.33 | 0.34 | n |
| R3225 | s8_chain4back | 8 | 2 | 5 | 7 | s | 0.23 | 1.03 | 1.10 | 4.7 | 7.7 | 3.0 | 0.62 | 0.63 | n |
| R3225 | s8_chain4back | 8 | 3 | 3 | 6 | s | 0.23 | 1.74 | 2.01 | 3.9 | 12.4 | 8.7 | 0.32 | 0.33 | n |
| R3225 | s8_chain4back | 8 | 4 | 7 | 5 | s | 0.23 | 1.50 | 1.71 | 3.4 | 10.2 | 7.2 | 0.31 | 0.33 | n |
| R3230 | s8_chain4back | 8 | 1 | 1 | 8 | s | 0.26 | 1.28 | 1.55 | 3.5 | 9.4 | 6.2 | 0.33 | 0.35 | n |
| R3230 | s8_chain4back | 8 | 2 | 5 | 7 | s | 0.26 | 1.03 | 1.26 | 5.8 | 8.9 | 3.1 | 0.45 | 0.45 | n |
| R3230 | s8_chain4back | 8 | 3 | 3 | 6 | s | 0.26 | 1.74 | 1.84 | 4.2 | 11.9 | 7.9 | 0.34 | 0.34 | n |
| R3230 | s8_chain4back | 8 | 4 | 7 | 5 | s | 0.26 | 1.50 | 1.65 | 3.7 | 10.3 | 7.0 | 0.33 | 0.34 | n |
| R3218 | s9_chain4back | 9 | 1 | 1 | 9 | s | 0.19 | 1.14 | 1.33 | 5.8 | 9.0 | 3.2 | 0.54 | 0.54 | n |
| R3218 | s9_chain4back | 9 | 2 | 5 | 8 | s | 0.19 | 0.98 | 1.14 | 5.0 | 7.6 | miss | 0.83 | - | y |
| R3218 | s9_chain4back | 9 | 3 | 3 | 8 | s | 0.19 | 1.43 | 1.81 | 3.7 | 11.0 | 7.7 | 0.33 | 0.34 | n |
| R3218 | s9_chain4back | 9 | 4 | 7 | 7 | s | 0.19 | 0.95 | 1.34 | 5.8 | 8.9 | 3.1 | 0.54 | 0.55 | n |
| R3227 | s9_chain4back | 9 | 1 | 1 | 9 | s | 0.19 | 1.14 | 1.43 | 2.8 | 8.2 | 5.5 | 0.33 | 0.34 | n |
| R3227 | s9_chain4back | 9 | 2 | 5 | 8 | s | 0.19 | 0.98 | 1.15 | 5.0 | 7.8 | miss | 0.84 | - | y |
| R3227 | s9_chain4back | 9 | 3 | 3 | 8 | s | 0.19 | 1.43 | 1.80 | 3.9 | 11.0 | 7.3 | 0.33 | 0.34 | n |
| R3227 | s9_chain4back | 9 | 4 | 7 | 7 | s | 0.19 | 0.95 | 1.33 | 5.7 | 8.7 | 3.0 | 0.53 | 0.54 | n |

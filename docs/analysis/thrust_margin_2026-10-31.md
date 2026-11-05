# Thrust margin and the measured pull, from the real-flight logs (31 Oct 2026)

Offline and exploratory: one pass, not reviewed, no card, nothing flown or simulated. It answers `docs/handin_plan.md`
section 6, request Q006 (sections 1-7) and request Q003 (section 8). Nothing else was edited.

## Verdict

| # | Finding | Size |
|---|---|---|
| 1 | Real-world finding: QUAD4 over the ring, no contact, loads it (likely its wash) | +2.6 N, +0.04 throttle |
| 2 | The +9 to +27 % read reproduces but is no map error: a 0.55 kg acceleration x 0.583 kg, plus QUAD4 | 1.8-2.0 N |
| 3 | Without those the force balance closes (QUAD4 away, median per 60 deg group) | -3.1 to +3.9 % of thrust |
| 4 | The map over-predicts at high throttle, far less than read: right at 0.55, rising (0.55 m) | +0.34 N at 0.75 |
| 5 | Largest open number: the airframe's mass. 1 m flights throttle as if the 33 g were not there | 0.97-1.29 N |
| 6 | Three carriers on the 1.15 kg ring have no margin at the default cap 0.8, and enough at 0.95 | section 6.4 |

**For the lab chat: propose, not edited.**
- **Wed 4 Nov (`docs/rig/wed_2026-11-04.md`):** no line changes. `throttle_max:=0.95` on `$R3` and `$R4` stays. Replace
  the note in the sheet's section 4 ("the thrust map predicts 0.71, so it is optimistic up there") with: the triangle
  flew 0.69-0.74 team mean with QUAD4 away and 0.75-0.79 with QUAD4 over the ring, peaks above the default 0.8, see this
  file. Add one minute to the pre-flight: weigh one airframe as flown (pack, 1 m cable, magnet) and the ring. That
  settles finding 5.
- **Thu 5 Nov (sheet not written yet):** type `throttle_max:=0.95` on h3, f3 and ls3 (`$R3` as Wednesday, typed on the
  line, no default changed). At the default 0.8 those cells fail. h4 and ls4 fly at either cap, and `$R4` as Wednesday
  avoids the thin ls4 cells at 22.0 V and below. Packs at 24.0 V or more at rest keep every baseline in the `ok` cells
  at 0.95. No baseline is flagged at 0.95 above 21.5 V.
- **The attach is the tightest flight, not the baselines:** QUAD4 over the ring peaked at 0.925 against the cap 0.95
  (sq5_2). The sheet's fresh packs matter most there.

## 1. What the measured pull is, from the code

`src/tracker/tracker/tracker_node.py`, `measured_cable_accel` and `_mocap_accel`, on every 50 Hz tick:

```
a_c = a_mocap - kT u R e3 + g e3          (drag term 0 on the rig, world frame, m/s^2)
u   = thr_out - offset(V)                 offset(V) = 0.185 + 0.022 (23.5 - V), kT = 35.2
```

- `a_mocap` is the least-squares slope of the mocap velocity over 80 ms with a 50 ms low-pass, the thrust term through
  the same filter. The raw `a_c` is logged as `acm_x/y/z` from 22 Oct on. `acx/y/z` come after the clamp, filter, cap
  and slew. For earlier flights it was rebuilt offline from the logged throttle, the mocap quaternion and `x0_vz`.
- The tracker holds no mass: the pull is an acceleration. A pull in newtons needs a mass chosen by the reader.
- In a steady hover `a_mocap` averages zero (table below), so the pull's hover mean is `kT u R33 - g` and nothing else:
  the thrust map, the pack voltage reading and the quad's lean. **The map's error lands in it one for one.**
- `kT` 35.2 is 19.36 N per unit throttle over 0.55 kg (`src/tracker/test/test_thrust_offset.py`). It was not rescaled
  when `drone_mass` became 0.583 kg for the 1 m cable (`real.yaml`, and `thrust_ratio` 35.2 in every flight's
  `params.json`).

Two readings in newtons, per carrier, both compared to the ring's weight `M g`:

| Name | Formula | Note |
|---|---|---|
| Logged x m | `m (-acm_z)` | the earlier review's reading |
| Map balance | `19.36 u R33 - m g - m a_z` | map's force, real mass: the sum is thrust minus system weight |

They agree at m = 0.55 kg. At m = 0.583 kg Logged x m is larger by `1.16 u R33` N per carrier, a pure bookkeeping term.

| Book value | Value | Source |
|---|---|---|
| Ring | 0.86 kg M2A, 1.15 kg flown from 29 Oct | Wesley 23 Sep, 28 Oct, not re-weighed here |
| Airframe with pack, cable, magnet | 0.55 kg on 0.55 m, 0.583 kg on 1 m | decisions 30 Oct, not re-weighed here |
| Cable length from mocap, all windows | 0.548-0.566 m, 0.996-1.003 m | first 1 m flight 29 Oct, 22 Oct flew 0.55 m |
| Mean vertical acceleration in a window | -0.015 to +0.016 m/s^2 | carriers' mocap, all windows of 3 s or more |
| Rebuilt against logged pull, summed | under 0.04 m/s^2, ~0.02 N | 23 windows that have both |

## 2. The steady window

Set in the script before the first bias was computed. Two rules changed after a first look, and this is why it is not a
claim: the start now moves up until the ring is inside its band (four 1 m flights missed it by a few cm in their first
seconds), and the QUAD4 rule was added after sq2_5's time series showed the pull following QUAD4.

| Rule | Value |
|---|---|
| Starts | 2 s after the reference reaches its top (`n2_bands`' hover rule), later if the ring is out of its band |
| Ends | 0.5 s before the first LAND, DROP, DETACH, DETECTED, RELEASE, abort, disarm or reference move |
| Ring | within 0.10 m of the top reference, vertical drift under 0.02 m/s over the window |
| QUAD4 | an attach flight's window ends 0.5 s before QUAD4 comes within 1.0 m of the ring's centre, sideways |
| Length | 3 s or more counts, 1-3 s is listed as short and left out of the statistics |
| Sum | the planner's `num_drones` are the carriers, QUAD4 in an attach flight is not one |

Of 217 flight folders, 54 windows count and 7 are short. Not usable:

| Why | Flights |
|---|---|
| Reference stayed on the floor: a lift that never ran, void, refused | 58 |
| No tracker rows: not flown | 35 |
| Under 1 s from settle to first event: LAND at the top of the lift (h4, h3n, h2, ch1, sq1, 30 Oct's heave) | 35 |
| Proportional map, before the affine map (30 Sep) | 15 |
| Free hover, no ring (section 5.3 uses those with a plateau) | 10 |
| Ring drifting, log ends early | 3 |

## 3. The table (Q006)

Bias, SD and RMSE of the summed vertical pull against the ring's weight, map balance. `% ring` is of the ring's weight,
`% thrust` of the whole system's weight. Median over the hovers. There is no 2-carrier flight on the 1.15 kg ring and no
M2A flight on 1 m.

| Setup | n | Hovers | Thr | V flown | Bias, N | % ring | Range, % | SD, N | RMSE, N | % thrust | Logged x m, % |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 1 m, 1.15 kg | 3 | 6 | 0.71 | 22.1-23.4 | -0.04 | -0.4 | -6.8 to +5.0 | 0.62 | 0.86 | -0.2 | +14.7 |
| 1 m, 1.15 kg | 4 | 3 | 0.64 | 22.7-23.6 | -1.04 | -9.3 | -12.6 to -8.6 | 0.32 | 1.12 | -3.1 | +8.4 |
| 0.55 m, 0.86 kg | 2 | 10 | 0.75 | 21.8-23.5 | +0.75 | +8.8 | +2.7 to +15.1 | 0.20 | 0.77 | +3.9 | +8.8 |
| 0.55 m, 0.86 kg | 3 | 22 | 0.64 | 22.1-23.9 | +0.22 | +2.6 | -2.1 to +9.8 | 0.30 | 0.45 | +0.9 | +2.6 |
| 0.55 m, 0.86 kg, T | 3 | 1 | 0.70 | 23.5-23.5 | +0.38 | +4.5 | +4.5 to +4.5 | 0.46 | 0.59 | +1.5 | +4.5 |
| 0.55 m, 0.86 kg, 44 deg | 4 | 2 | 0.57 | 23.5-23.9 | -1.58 | -18.7 | -18.8 to -18.6 | 0.25 | 1.60 | -5.2 | -18.7 |
| 0.55 m, 0.86 kg | 4 | 10 | 0.60 | 22.3-24.2 | +0.36 | +4.3 | -2.5 to +8.8 | 0.36 | 0.55 | +1.2 | +4.3 |

Sensitivity to the airframe's mass, the 1 m rows only:

| Carriers | m 0.583 kg, bias N | % ring | m 0.55 kg, bias N | % ring | Range, % ring at 0.55 kg |
|---|---|---|---|---|---|
| 3 | -0.04 | -0.4 | +0.93 | +8.2 | +1.8 to +13.6 |
| 4 | -1.04 | -9.3 | +0.25 | +2.2 | -1.1 to +2.9 |

A percent of the ring's weight looks large because the pull is a small difference of large numbers: with four carriers,
one percent of thrust is three percent of the ring.

**1 m, 1.15 kg, every hover** (29 Oct, the only day with a steady window on this setup):

| Flight | n | s | Throttle | V | Logged x m bias, N | % | RMSE, N | Map bias, N | % | SD, N | RMSE, N |
|---|---|---|---|---|---|---|---|---|---|---|---|
| ch1b_2 | 4 | 15.4 | 0.636 | 23.0 | +1.02 | +9.0 | 1.08 | -0.97 | -8.6 | 0.32 | 1.02 |
| ch1b | 4 | 5.5 | 0.621 | 23.6 | +0.55 | +4.9 | 0.64 | -1.42 | -12.6 | 0.31 | 1.45 |
| f4 | 4 | 3.7 | 0.642 | 22.7 | +0.94 | +8.4 | 1.04 | -1.04 | -9.3 | 0.40 | 1.12 |
| h4f_3 (short) | 4 | 1.9 | 0.630 | 23.2 | +0.66 | +5.9 | 0.70 | -1.31 | -11.6 | 0.22 | 1.33 |
| sq3 | 3 | 36.5 | 0.686 | 23.3 | +0.89 | +7.9 | 1.01 | -0.77 | -6.8 | 0.44 | 0.89 |
| k1_8 | 3 | 27.1 | 0.714 | 22.4 | +1.66 | +14.7 | 1.76 | -0.04 | -0.4 | 0.55 | 0.55 |
| sq2_5 | 3 | 26.8 | 0.713 | 22.9 | +2.31 | +20.5 | 2.51 | +0.57 | +5.0 | 0.94 | 1.10 |
| k1_4 | 3 | 26.0 | 0.717 | 22.1 | +1.66 | +14.7 | 1.78 | -0.04 | -0.4 | 0.61 | 0.61 |
| sq4_5 | 3 | 17.7 | 0.737 | 22.1 | +2.29 | +20.3 | 2.49 | +0.55 | +4.9 | 0.90 | 1.06 |
| sq5_2 | 3 | 8.4 | 0.687 | 23.4 | +1.14 | +10.1 | 1.32 | -0.54 | -4.8 | 0.63 | 0.83 |

**0.55 m, 0.86 kg, every hover.** The two readings are the same at this mass. `rebuilt` means no `acm_z` in that log. 28
Oct's planner had `drone_mass` 0.583 through real.yaml, while these airframes were 0.55 kg. The balance uses 0.55 kg.

| Day | Flight | n | s | Throttle | V | Bias, N | % of ring | SD, N | RMSE, N | % of thrust | Source |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 10-21 | c2q | 2 | 10.1 | 0.760 | 22.1 | +0.77 | +9.1 | 0.24 | 0.81 | +4.0 | rebuilt |
| 10-22 | f2slope | 2 | 3.8 | 0.721 | 23.3 | +0.32 | +3.7 | 0.09 | 0.33 | +1.6 | log |
| 10-28 | f2a | 2 | 3.8 | 0.755 | 22.4 | +0.76 | +9.0 | 0.44 | 0.88 | +4.0 | log |
| 10-28 | f2b | 2 | 3.8 | 0.767 | 21.8 | +0.73 | +8.6 | 0.14 | 0.74 | +3.8 | log |
| 10-28 | f2c | 2 | 3.8 | 0.736 | 23.5 | +0.89 | +10.5 | 0.13 | 0.90 | +4.6 | log |
| 10-28 | hS1 (short) | 2 | 1.4 | 0.732 | 23.0 | +0.41 | +4.9 | 0.19 | 0.45 | +2.1 | log |
| 10-28 | hS2 | 2 | 6.3 | 0.767 | 22.4 | +1.28 | +15.1 | 0.13 | 1.28 | +6.6 | log |
| 10-28 | ls2a | 2 | 3.5 | 0.723 | 23.4 | +0.38 | +4.5 | 0.24 | 0.45 | +2.0 | log |
| 10-28 | ls2b_4 | 2 | 3.9 | 0.760 | 22.5 | +1.09 | +12.9 | 0.20 | 1.11 | +5.7 | log |
| 10-28 | ls2b_5 | 2 | 3.5 | 0.712 | 23.5 | +0.23 | +2.7 | 0.21 | 0.31 | +1.2 | log |
| 10-28 | ls2c | 2 | 3.9 | 0.741 | 22.6 | +0.64 | +7.6 | 0.24 | 0.69 | +3.3 | log |
| 10-01 | h1 (short) | 3 | 2.4 | 0.591 | 23.9 | -1.39 | -16.5 | 0.37 | 1.44 | -5.6 | rebuilt |
| 10-14 | lift3_3 | 3 | 7.4 | 0.649 | 22.4 | +0.21 | +2.5 | 0.55 | 0.59 | +0.8 | rebuilt |
| 10-14 | t5_4 | 3 | 55.7 | 0.646 | 22.6 | -0.03 | -0.4 | 0.44 | 0.44 | -0.1 | rebuilt |
| 10-14 | t5_5 | 3 | 86.9 | 0.626 | 23.1 | -0.16 | -1.8 | 0.54 | 0.56 | -0.6 | rebuilt |
| 10-14 | t6 | 3 | 49.1 | 0.626 | 23.4 | +0.04 | +0.4 | 0.43 | 0.43 | +0.1 | rebuilt |
| 10-14 | t6f | 3 | 7.7 | 0.643 | 22.7 | +0.03 | +0.3 | 0.33 | 0.33 | +0.1 | rebuilt |
| 10-14 | t6f2_4 | 3 | 9.3 | 0.614 | 23.9 | +0.05 | +0.5 | 0.37 | 0.37 | +0.2 | rebuilt |
| 10-14 | t6g_2 | 3 | 6.2 | 0.649 | 22.6 | +0.31 | +3.7 | 0.25 | 0.40 | +1.3 | rebuilt |
| 10-14 | t6h5_3 (short) | 3 | 1.7 | 0.646 | 22.6 | +0.16 | +1.9 | 0.56 | 0.58 | +0.6 | rebuilt |
| 10-14 | t6h_4 | 3 | 19.5 | 0.618 | 23.7 | -0.01 | -0.1 | 0.54 | 0.54 | -0.0 | rebuilt |
| 10-15 | a1 | 3 | 10.5 | 0.626 | 23.4 | -0.03 | -0.4 | 0.46 | 0.46 | -0.1 | rebuilt |
| 10-15 | a2_2 | 3 | 12.1 | 0.663 | 22.1 | +0.45 | +5.3 | 0.42 | 0.61 | +1.8 | rebuilt |
| 10-15 | a3_2 | 3 | 9.7 | 0.619 | 23.6 | -0.18 | -2.1 | 0.27 | 0.32 | -0.7 | rebuilt |
| 10-21 | c200_4 | 3 | 9.5 | 0.635 | 23.3 | +0.27 | +3.2 | 0.20 | 0.34 | +1.1 | rebuilt |
| 10-21 | f125_2 | 3 | 8.9 | 0.661 | 22.2 | +0.76 | +9.0 | 0.23 | 0.80 | +3.1 | rebuilt |
| 10-21 | f200 | 3 | 8.8 | 0.643 | 22.8 | +0.38 | +4.5 | 0.33 | 0.51 | +1.6 | rebuilt |
| 10-21 | l125 | 3 | 8.4 | 0.629 | 23.7 | +0.17 | +2.1 | 0.25 | 0.30 | +0.7 | rebuilt |
| 10-21 | l200 | 3 | 8.2 | 0.652 | 22.7 | +0.64 | +7.6 | 0.22 | 0.67 | +2.6 | rebuilt |
| 10-21 | s125 | 3 | 10.4 | 0.651 | 22.5 | +0.30 | +3.5 | 0.20 | 0.35 | +1.2 | rebuilt |
| 10-21 | s200_3 | 3 | 9.5 | 0.631 | 23.4 | +0.31 | +3.7 | 0.35 | 0.47 | +1.3 | rebuilt |
| 10-22 | f3h | 3 | 3.8 | 0.651 | 22.7 | +0.56 | +6.6 | 0.18 | 0.59 | +2.3 | log |
| 10-22 | h3 | 3 | 6.3 | 0.634 | 23.3 | +0.23 | +2.7 | 0.13 | 0.26 | +0.9 | log |
| 10-22 | ls3 | 3 | 3.6 | 0.620 | 23.7 | +0.16 | +1.9 | 0.10 | 0.19 | +0.7 | log |
| 10-22 | ls3c | 3 | 3.5 | 0.655 | 22.9 | +0.83 | +9.8 | 0.19 | 0.85 | +3.4 | log |
| 10-22 | t3p_2 (T) | 3 | 44.5 | 0.704 | 23.5 | +0.38 | +4.5 | 0.46 | 0.59 | +1.5 | log |
| 10-07 | r000014 | 4 | 10.5 | 0.619 | 22.4 | +0.68 | +8.0 | 0.22 | 0.71 | +2.3 | rebuilt |
| 10-07 | r0001 (short) | 4 | 1.8 | 0.587 | 23.6 | +0.29 | +3.5 | 0.09 | 0.31 | +1.0 | rebuilt |
| 10-07 | r0007 | 4 | 5.5 | 0.610 | 22.6 | +0.35 | +4.2 | 0.28 | 0.45 | +1.2 | rebuilt |
| 10-07 | r0012 | 4 | 6.3 | 0.572 | 23.6 | +0.43 | +5.1 | 0.38 | 0.58 | +1.4 | rebuilt |
| 10-07 | r0013 | 4 | 3.6 | 0.594 | 22.8 | -0.06 | -0.7 | 0.11 | 0.12 | -0.2 | rebuilt |
| 10-07 | r002 | 4 | 14.7 | 0.599 | 23.0 | +0.37 | +4.3 | 0.50 | 0.62 | +1.2 | rebuilt |
| 10-07 | r006 | 4 | 3.9 | 0.567 | 24.2 | -0.21 | -2.5 | 0.20 | 0.29 | -0.7 | rebuilt |
| 10-07 | r200006 (44 deg) | 4 | 3.9 | 0.578 | 23.5 | -1.57 | -18.6 | 0.23 | 1.58 | -5.2 | rebuilt |
| 10-07 | r20004 (44 deg) | 4 | 5.0 | 0.572 | 23.9 | -1.58 | -18.8 | 0.26 | 1.61 | -5.3 | rebuilt |
| 10-07 | r2008e60 | 4 | 5.4 | 0.610 | 22.4 | +0.74 | +8.8 | 0.33 | 0.81 | +2.5 | rebuilt |
| 10-07 | r206e60 | 4 | 3.9 | 0.604 | 23.0 | +0.25 | +2.9 | 0.42 | 0.49 | +0.8 | rebuilt |
| 10-08 | r302_3 | 4 | 4.9 | 0.561 | 23.9 | +0.07 | +0.8 | 0.53 | 0.53 | +0.2 | rebuilt |
| 10-08 | r303_4 (short) | 4 | 2.8 | 0.578 | 23.1 | +0.27 | +3.2 | 0.56 | 0.62 | +0.9 | rebuilt |
| 10-08 | r304_3 (short) | 4 | 2.8 | 0.603 | 22.6 | +1.19 | +14.1 | 0.62 | 1.34 | +4.0 | rebuilt |
| 10-08 | r305 | 4 | 3.8 | 0.598 | 22.3 | +0.61 | +7.3 | 0.41 | 0.74 | +2.0 | rebuilt |

## 4. What this validates and what it cannot

| Validates | Cannot validate |
|---|---|
| The sum's mean closes against a known weight within the mass uncertainty, throttle 0.56-0.77 | The carriers' split |
| The map, the voltage term, the lean and the masses together, at each team's one point | The horizontal pull |
| | Lag or dynamic response: no reference signal, and the pull does not follow the ring's motion |
| | Which of map, mass, wash or a lean offset causes a residual: all land in one number |

| Check | Value |
|---|---|
| Correlation, summed pull against the ring's vertical acceleration, median (range), 54 windows | -0.07 (-0.40, +0.25) |
| Ring mass x its vertical acceleration SD against the pull's SD, median | 0.10 N against 0.31 N |
| A 2 deg mocap frame error at 15 deg lean (q4solo: FC within 2 deg of the body) | 0.9 % of thrust, ~0.3 N at four |

For the thesis: the wind-mapping paper validates its observer against an anemometer, a separate instrument. The ring's
weight gives a check of the mean only. Say "the summed vertical pull closes against the ring's weight" and name the
residual's size, do not say the pull is validated as a dynamic measurement.

## 5. The earlier read: reproduced, then refuted as a map error

### 5.1 Where each number came from

| Flight | Read, N | Reproduced, N | Excess | Mix: logged x m - map, N | Map balance bias, N | QUAD4 near, away, N |
|---|---|---|---|---|---|---|
| sq2_5 | 14.3 | 14.32 | +27 % | 1.80 | +1.58 | +2.30, +0.57 |
| f4 | 12.3 | 12.35 | +9 % | 1.98 | -0.96 | no QUAD4 |
| hS2 | 9.8 | 9.70 | +16 % | 0 | +1.27 | no QUAD4 |

Reproduced as the review did it: sum of per-carrier medians from the top to LAND (m 0.583, 0.583, 0.55 kg, and the
review's hS2 window was 3 s). "Near" is QUAD4 within 1.0 m of the ring's centre (38.5 s of that flight), "away" the
other 27.3 s.

- f4's +9 % is the mix alone. Without it the sign flips.
- sq2_5's +27 % is the mix, then QUAD4 over the ring. With QUAD4 away about 5 % of the ring's weight is left.
- hS2 stands, with no mix and no QUAD4. It is the top of the pairs on 0.55 m (+2.7 to +15.1 % of the ring, section 3) at
  the highest throttle in the logs. That is +6.6 % of thrust, about 0.6 N per carrier.

### 5.2 The map against a fit to the same hovers

Thrust needed per carrier, `(M/n + m) g / R33`, against throttle and pack voltage over 75 points: 52 ring hovers at 60
deg, 9 free hovers, the 13 points the map was fitted to, and solo1. Exploratory, chosen after seeing the data.

| Term in `T = K (u - a - c (V - 23.5))` | Map, 30 Sep | Fit | Bootstrap 95 % |
|---|---|---|---|
| K, N per unit throttle | 19.35 | 17.71 | 16.84 to 18.37 |
| a, offset at 23.5 V | 0.185 | 0.152 | 0.132 to 0.165 |
| c, throttle per volt | -0.0219 | -0.0227 | -0.0277 to -0.0167 |
| 1 m setup, N per carrier at book mass | none | +0.38 | +0.27 to +0.50 |
| RMS residual, N per carrier | 0.27 | 0.20 | |

A form with thrust growing as a power of voltage gave the same residual (0.20 N, exponent 1.18), so the map's additive
voltage term is fine across packs.

| Throttle at 23.5 V | Map, N | Fit 0.55 m, N | Map minus fit | Fit 1 m, N | Map minus fit |
|---|---|---|---|---|---|
| 0.46 | 5.32 | 5.46 | -0.14 N, -2.5 % | 5.84 | -0.52 N, -8.8 % |
| 0.55 | 7.06 | 7.05 | +0.01 N, +0.2 % | 7.43 | -0.37 N, -5.0 % |
| 0.65 | 9.00 | 8.82 | +0.17 N, +2.0 % | 9.20 | -0.20 N, -2.2 % |
| 0.75 | 10.93 | 10.59 | +0.34 N, +3.2 % | 10.97 | -0.04 N, -0.4 % |
| 0.85, past any steady data | 12.87 | 12.36 | +0.50 N, +4.1 % | 12.74 | +0.12 N, +1.0 % |

So the map over-predicts above about 0.55 throttle on the 0.55 m setup, by about 0.35 N per carrier at the highest
throttle flown, not the review's 0.3-1.0 N. On 1 m at book mass it does not over-predict up to 0.75. Fitted only between
0.46 and 0.56, its slope runs 9 % above this fit's. CEP's full-throttle test agrees in sign, but no log here reaches
full throttle.

### 5.3 Does the residual depend on throttle, voltage, carrier or setup?

Map thrust minus system weight, % of thrust: positive means the map gives more than the hover needs. Regression over
the 51 hovers at 60 deg in even or uneven layouts, excluding the T and the 44 deg flights, residual SD 1.1 %. Throttle
and voltage correlate -0.48.

| Factor | Effect | Evidence |
|---|---|---|
| Throttle | +2.1 % per +0.10 throttle, SE 0.32 | yes, the regression |
| Pack voltage, fixed throttle | -0.82 % per volt, SE 0.31 | yes, weak |
| Voltage inside a flight | -0.9 to -7.0 % per volt, median -2.3 on 0.55 m | 7 flights, includes the throttle rise |
| Setup, 1 m against 0.55 m | -3.6 %, SE 0.41 | yes, and 33 g per carrier is -3.4 to -3.8 % of thrust |
| Carrier | -3.5 to -1.3 % in one free hover (below) | small, ~0.1 N. Not separable in ring flights |
| Cable elevation | 44 deg -5.2 %, 60 deg the same day -0.7 to +2.5 % | 2 flights, lean 22 against 14 deg, unexplained |

The 1 m offset is the size of the extra 33 g. Either the airframes on 29-30 Oct weigh about 0.55 kg, or they make that
much more thrust per throttle than in mid-October. A free hover says the same: QUAD1 on its 1 m cable flew the throttle
the map gives 0.539 kg (solo1, 2.5 s only). A scale settles it. No case here for changing `thrust_ratio`: that free
hover reads 35.9 against the typed 35.2.

Free hovers, map thrust against the airframe's weight, plateau rule of `tools/thrust_fit.py` (solo1: only 2.5 s at the
top):

| Day | Flight | Quad | Mass, kg | s | V | Throttle | Map predicts | Map thrust, N | Weight, N | Map over weight |
|---|---|---|---|---|---|---|---|---|---|---|
| 10-14 | fh3_2 | QUAD1 | 0.55 | 3.1 | 23.3 | 0.457 | 0.468 | 5.21 | 5.40 | -3.4 % |
| 10-14 | fh3_2 | QUAD2 | 0.55 | 3.2 | 23.2 | 0.460 | 0.470 | 5.21 | 5.40 | -3.5 % |
| 10-14 | fh3_2 | QUAD3 | 0.55 | 3.1 | 23.2 | 0.466 | 0.470 | 5.33 | 5.40 | -1.3 % |
| 10-14 | fh3_2 | QUAD4 | 0.55 | 3.2 | 23.9 | 0.448 | 0.455 | 5.24 | 5.40 | -2.9 % |
| 10-15 | fh3_2 | drone_0 | 0.55 | 8.8 | 24.8 | 0.439 | 0.435 | 5.48 | 5.40 | +1.5 % |
| 10-15 | fh3_2 | drone_1 | 0.55 | 8.6 | 24.0 | 0.461 | 0.453 | 5.52 | 5.40 | +2.3 % |
| 10-15 | fh3_2 | drone_2 | 0.55 | 8.6 | 24.5 | 0.447 | 0.442 | 5.51 | 5.40 | +2.1 % |
| 10-15 | q4circle_2 | QUAD4 | 0.55 | 35.0 | 23.1 | 0.467 | 0.473 | 5.29 | 5.40 | -2.0 % |
| 10-15 | q4solo | QUAD4 | 0.55 | 11.8 | 23.7 | 0.453 | 0.459 | 5.26 | 5.40 | -2.5 % |
| 10-30 | solo1 | QUAD1 | 0.58 | 2.5 | 23.7 | 0.453 | 0.475 | 5.29 | 5.72 | -7.5 % |

In ring flights the highest-throttle carrier changes from flight to flight (section 6.3). QUAD3 made the whole-flight
peak in sq2_5, sq4_5 and sq5_2 (0.894-0.925). One line to watch, not a finding.

## 6. Throttle and margin on the 1.15 kg ring on 1 m cables

### 6.1 The flights

`thr_out` over every carrier, cap as typed. "V at rest" is idle before TAKEOFF. Pack voltage falls 0.7-0.9 V from rest
to lift-off, then ~0.02 V/s with three or four carriers.

| Flight | n | Form | Cap | V at rest | V at lift-off | V at end | s up | Lift p99 | All p99 | Peak |
|---|---|---|---|---|---|---|---|---|---|---|
| h4 | 4 | even | 0.80 | 24.8 | 24.0 | 23.7 | 9 | 0.680 | 0.691 | 0.746 |
| h4f_3 | 4 | even | 0.80 | 24.4 | 23.6 | 23.2 | 12 | 0.662 | 0.659 | 0.673 |
| f4 | 4 | even | 0.80 | 23.8 | 23.0 | 22.3 | 40 | 0.677 | 0.733 | 0.762 |
| ch1 | 4 | even | 0.90 | 25.0 | 24.3 | 24.1 | 6 | 0.646 | 0.646 | 0.679 |
| ch1b | 4 | even | 0.90 | 24.7 | 23.9 | 23.6 | 16 | 0.632 | 0.678 | 0.681 |
| ch1b_2 | 4 | even | 0.90 | 24.2 | 23.5 | 22.9 | 26 | 0.655 | 0.685 | 0.695 |
| sq1 | 3 | T | 0.90 | 23.5 | 22.9 | 22.2 | 11 | 0.868 | 0.887 | 0.896 |
| sq2_2 | 3 | even | 0.95 | 23.2 | 22.3 | 21.9 | 14 | 0.754 | 0.842 | 0.851 |
| sq2_4 | 3 | even | 0.95 | 24.9 | 24.1 | 23.8 | 8 | 0.713 | 0.724 | 0.728 |
| sq2_5 | 3 | even | 0.95 | 24.4 | 23.6 | 21.9 | 75 | 0.711 | 0.846 | 0.894 |
| sq3 | 3 | even | 0.95 | 24.9 | 24.0 | 23.0 | 46 | 0.689 | 0.734 | 0.758 |
| sq4_5 | 3 | even | 0.95 | 23.6 | 22.8 | 22.0 | 40 | 0.743 | 0.848 | 0.903 |
| sq5_2 | 3 | even | 0.95 | 24.7 | 23.8 | 22.4 | 46 | 0.699 | 0.867 | 0.925 |
| k1_4 | 3 | even | 0.95 | 23.4 | 22.6 | 21.9 | 36 | 0.743 | 0.802 | 0.821 |
| k1_8 | 3 | even | 0.95 | 24.6 | 23.8 | 22.2 | 73 | 0.710 | 0.778 | 0.809 |

### 6.2 What each team needs per pack voltage

Team mean throttle in a steady hover, voltage under load. `*` is outside the voltage flown. The map rows use the plan's
lean (11, 13, 16 deg for 4, 3, 2 carriers). The quads lean 15 and 17 deg with 4 and 3.

| Fit | At 23 V | Per volt | Residual SD | Bins |
|---|---|---|---|---|
| Three carriers, even triangle | 0.703 | -0.0302 | 0.009 | 27 |
| Four carriers (h4f_3's short window agrees: 0.630 at 23.25 V) | 0.6367 | -0.0263 | 0.002 | 5 |

| Team | 24.0 V | 23.5 V | 23.0 V | 22.5 V | 22.0 V | 21.5 V | Source | V flown |
|---|---|---|---|---|---|---|---|---|
| 4, square | 0.610* | 0.624 | 0.637 | 0.650* | 0.663* | 0.676* | fit, ch1b, ch1b_2, f4 | 22.7-23.6 |
| 3, even triangle | 0.673* | 0.688 | 0.703 | 0.718 | 0.733 | 0.748* | fit, 6 flights, QUAD4 away | 22.0-23.7 |
| 2, opposite | 0.804* | 0.815* | 0.826* | 0.838* | 0.849* | 0.861* | the fitted law only | never flown |
| Map, 4 | 0.623 | 0.634 | 0.645 | 0.656 | 0.667 | 0.678 | the map, plan's 60 deg | |
| Map, 3 | 0.677 | 0.688 | 0.699 | 0.710 | 0.721 | 0.732 | the map, plan's 60 deg | |
| Map, 2 | 0.785 | 0.796 | 0.807 | 0.818 | 0.829 | 0.840 | the map, plan's 60 deg | |

The map's 0.71 at 22.5 V for three carriers (the Wed sheet's note) is within 0.01 of the 0.718 needed with QUAD4 away.
The 0.73-0.76 the note quotes is sq2_5's whole window, QUAD4 within 1 m of the ring's centre for over half of it.

Two carriers are out at any pack: the law predicts 0.83 at 23 V and the pairs on 0.55 m ran +0.05 to +0.15 above their
hover (table below), up to and past the cap 0.9. The rescope already cut this cell. The T is out too: sq1's lift ran p99
0.868 at cap 0.9, and in ch1b_2 after the release its two loaded carriers ran 0.81 and 0.88 while the ring sank 0.32 m
in 4 s. Neither is on a sheet: Wednesday's release leaves the even triangle.

### 6.3 What sits on top of the team mean

Hover, QUAD4 away:

| Flight | n | V | Team mean | Highest carrier | p95 | p99 | Peak | High over mean | p99 over mean | Peak over mean |
|---|---|---|---|---|---|---|---|---|---|---|
| k1_4 | 3 | 22.1 | 0.724 | 0.750 | 0.781 | 0.801 | 0.821 | +0.026 | +0.077 | +0.097 |
| sq4_5 | 3 | 22.1 | 0.739 | 0.746 | 0.781 | 0.796 | 0.804 | +0.008 | +0.057 | +0.066 |
| k1_8 | 3 | 22.4 | 0.716 | 0.734 | 0.760 | 0.775 | 0.797 | +0.018 | +0.059 | +0.081 |
| sq2_5 | 3 | 22.9 | 0.720 | 0.727 | 0.780 | 0.823 | 0.842 | +0.007 | +0.103 | +0.122 |
| sq3 | 3 | 23.3 | 0.687 | 0.706 | 0.723 | 0.735 | 0.758 | +0.018 | +0.048 | +0.071 |
| sq5_2 | 3 | 23.4 | 0.690 | 0.704 | 0.727 | 0.739 | 0.749 | +0.014 | +0.049 | +0.059 |
| f4 | 4 | 22.7 | 0.645 | 0.677 | 0.691 | 0.702 | 0.710 | +0.033 | +0.057 | +0.065 |
| ch1b_2 | 4 | 23.0 | 0.638 | 0.670 | 0.679 | 0.689 | 0.695 | +0.032 | +0.051 | +0.057 |
| ch1b | 4 | 23.6 | 0.621 | 0.649 | 0.673 | 0.679 | 0.681 | +0.028 | +0.058 | +0.060 |

On a path, against the same flight's hold:

| Flight | n | Ring, kg | Lap s | V hold | Team hold | Path median | p99 | Peak | p99 over hold | Peak over hold |
|---|---|---|---|---|---|---|---|---|---|---|
| f4 | 4 | 1.15 | 26 | 22.7 | 0.644 | 0.647 | 0.736 | 0.762 | +0.091 | +0.118 |
| f3h | 3 | 0.86 | 34 | 22.7 | 0.651 | 0.662 | 0.710 | 0.732 | +0.059 | +0.080 |
| ls3 | 3 | 0.86 | 18 | 23.7 | 0.620 | 0.626 | 0.663 | 0.674 | +0.043 | +0.054 |
| ls3c | 3 | 0.86 | 18 | 22.9 | 0.652 | 0.653 | 0.695 | 0.714 | +0.043 | +0.061 |
| f200 | 3 | 0.86 | 18 | 22.8 | 0.646 | 0.652 | 0.701 | 0.708 | +0.055 | +0.062 |
| l200 | 3 | 0.86 | 15 | 22.7 | 0.656 | 0.656 | 0.703 | 0.709 | +0.047 | +0.053 |
| s200_3 | 3 | 0.86 | 26 | 23.4 | 0.632 | 0.641 | 0.692 | 0.711 | +0.060 | +0.080 |
| 8 pairs | 2 | 0.86 | 14-34 | 21.8-23.5 | 0.71-0.77 | 0.73-0.78 | 0.77-0.90 | 0.77-0.90 | +0.05..+0.15 | +0.05..+0.15 |

No figure-eight or line on three carriers and no line on four has flown at 1 m. The margin table borrows:

| Line | p99 over the team mean | Peak over the team mean | From |
|---|---|---|---|
| h3 | +0.058 | +0.122 | 3 hover |
| f3, ls3 | +0.092 | +0.174 | 3 hover + f4's extra |
| h4 | +0.057 | +0.065 | 4 hover |
| ls4 | +0.091 | +0.118 | f4 itself |
| 3 left, rel | +0.108 | +0.123 | r303_4, r304_3, r305 at 0.55 m |

"3 hover" is the median p99 and the worst peak of the six triangle hovers. "f4's extra" is f4's path over f4's own hold
(+0.034 p99, +0.052 peak). "3 left" is the 4 to 3 releases at 0.55 m, the first 3 s after the release against the new
steady mean, the worst of three, because no even-triangle release has flown at 1 m.

### 6.4 Margin to the cap, per baseline and voltage

Each cell: margin at p99 / margin at the peak, then the flag. `NO`: p99 within 0.03 of the cap or over it. `thin`: the
peak within 0.03 of the cap or over it. Voltage is under load: a pack 24.0 V at rest is ~23.2 V at lift-off and ~22.4 V
at the end of a 40 s flight (f4's length). Estimates from the fits and offsets above, not flights.

| Line | Cap | 23.5 V | 23.0 V | 22.5 V | 22.0 V | 21.5 V |
|---|---|---|---|---|---|---|
| h3 | 0.95 | +0.20 / +0.14 ok | +0.19 / +0.12 ok | +0.17 / +0.11 ok | +0.16 / +0.09 ok | +0.14 / +0.08 ok |
| h3 | 0.80 | +0.05 / -0.01 thin | +0.04 / -0.03 thin | +0.02 / -0.04 NO | +0.01 / -0.06 NO | -0.01 / -0.07 NO |
| f3, ls3 | 0.95 | +0.17 / +0.09 ok | +0.15 / +0.07 ok | +0.14 / +0.06 ok | +0.12 / +0.04 ok | +0.11 / +0.03 thin |
| f3, ls3 | 0.80 | +0.02 / -0.06 NO | +0.00 / -0.08 NO | -0.01 / -0.09 NO | -0.03 / -0.11 NO | -0.04 / -0.12 NO |
| h4 | 0.95 | +0.27 / +0.26 ok | +0.26 / +0.25 ok | +0.24 / +0.23 ok | +0.23 / +0.22 ok | +0.22 / +0.21 ok |
| h4 | 0.80 | +0.12 / +0.11 ok | +0.11 / +0.10 ok | +0.09 / +0.08 ok | +0.08 / +0.07 ok | +0.07 / +0.06 ok |
| ls4 | 0.95 | +0.24 / +0.21 ok | +0.22 / +0.20 ok | +0.21 / +0.18 ok | +0.20 / +0.17 ok | +0.18 / +0.16 ok |
| ls4 | 0.80 | +0.09 / +0.06 ok | +0.07 / +0.05 ok | +0.06 / +0.03 ok | +0.05 / +0.02 thin | +0.03 / +0.01 thin |
| 3 left, rel | 0.95 | +0.15 / +0.14 ok | +0.14 / +0.12 ok | +0.12 / +0.11 ok | +0.11 / +0.09 ok | +0.09 / +0.08 ok |
| 3 left, rel | 0.80 | +0.00 / -0.01 NO | -0.01 / -0.03 NO | -0.03 / -0.04 NO | -0.04 / -0.06 NO | -0.06 / -0.07 NO |

**Flags for Thu 5 Nov:** h3 at cap 0.8 (thin, then NO from 22.5 V), f3 and ls3 at cap 0.8 (NO everywhere), ls4 at cap
0.8 (thin at 22.0 V and below). At cap 0.95 the only flag is f3 and ls3 at 21.5 V, thin. h4 has margin at either cap.
The three left after a release fail at 0.8 at every voltage, which is why the Wed sheet types 0.95.

### 6.5 QUAD4 over the ring

Same flights, 5 s bins. "Over the triangle's fit" is team mean minus the fit in 6.2 at that bin's voltage. No contact in
these windows (`RIG-1029-sq2_5`, `-sq5_2`). The ring stayed at its height, so the measured pull did its job.

| Flight | QUAD4 | 5 s bins | V | Team mean | Over the triangle's fit | Balance, N | p99 | Highest carrier |
|---|---|---|---|---|---|---|---|---|
| sq2_5 | away, 1 m or more | 5 | 22.6-23.2 | 0.718 | +0.013 | +0.53 | 0.838 | 0.764 |
| sq2_5 | over the ring, 2.2 m up | 3 | 22.4-22.5 | 0.748 | +0.027 | +1.49 | 0.861 | 0.758 |
| sq2_5 | over the ring, 1.3 m up | 4 | 22.0-22.2 | 0.787 | +0.056 | +3.11 | 0.879 | 0.812 |
| sq5_2 | away, 1 m or more | 2 | 23.4-23.5 | 0.690 | +0.000 | -0.58 | 0.742 | 0.705 |
| sq5_2 | over the ring, 1.3 m up | 5 | 22.6-23.2 | 0.753 | +0.046 | +2.08 | 0.916 | 0.793 |

Within a flight, from away to 1.3 m above the ring: +2.57 and +2.65 N, +0.042 and +0.046 throttle. Two flights. The
attach descends from this height, so expect about this load at the catch (no catch has flown on this ring). The peak
0.925 against 0.95 is the smallest margin on the even triangle in the logs, on a pack at 22.9 V.

### 6.6 If the heave comes back

30 Oct's flights heaved (95 ms mocap delay, `RIG-1030-bench_mocap`), and none has a steady window. None of the margins
above covers them: their throttle swing is far larger, and h4n reached its cap of 0.80. The sheet already stops ring
flights on a heave.

| Flight | n | Cap | V at lift-off | Median in the lift | p99 | Peak | Ring top, m |
|---|---|---|---|---|---|---|---|
| h3t | 3 | 0.95 | 23.2 | 0.675 | 0.822 | 0.827 | 0.56 |
| h4n | 4 | 0.80 | 23.1 | 0.599 | 0.800 | 0.800 | 0.29 |
| sq6b_2 | 3 | 0.95 | 24.1 | 0.609 | 0.814 | 0.825 | 0.42 |
| sq6b_5 | 3 | 0.95 | 23.7 | 0.645 | 0.860 | 0.888 | 0.43 |
| sq6b_7 | 3 | 0.95 | 23.2 | 0.662 | 0.818 | 0.837 | 0.39 |

## 7. What could not be computed

| Item | Why |
|---|---|
| Bias for 2 carriers on 1.15 kg, or for the M2A ring on 1 m | never flown |
| Bias for the T on 1 m | no steady window (sq1 landed, ch1b_2 sank) |
| Any steady hover on 30 Oct | the heave, LAND before the top |
| Bias per carrier | only the sum has a truth, and shares differ |
| Horizontal pull, lag, dynamic accuracy | no reference signal |
| Map error apart from mass error, wash and lean | they share one residual. Needs a scale and a load cell |
| Steady thrust above 0.77 throttle, thrust at full throttle | not in the logs (highest instant 0.925) |
| f3, ls3, ls4 and the even-triangle release at 1 m | not flown. Section 6.3 borrows, and says from what |
| Which quad is which in most logs | the T2 log names them only in attach flights. Other rows use only the sum |

## 8. One metric for the figure-eight (Q003)

Both through `tools/n2_bands.py`'s `path_metrics_clock_free`, the combined number being what it returns. Its two parts
come from a line-for-line copy of its body. The window is that function's: the path's first move to the planner's
descent or LAND.

| | RIG-1029-f4 (real) | R2835 (simulation) |
|---|---|---|
| Setup | 4 carriers, 1 m, 1.15 kg, figure-eight 0.75 m at 0.2 m/s, 5 s hold | the same, world `_p115` |
| Tree | 413ec0e + working copy | 413ec0e + working copy |
| Measured pull filter | `cable_lp_hz` 2.0, one stage | none |
| Window, s | 28.86 to 55.48 (26.6) | 31.70 to 58.34 (26.6) |
| Sideways RMS, mm | 21.7 | 12.8 |
| Height RMS, mm | 33.8 | 2.0 |
| Height mean, SD, mm | -32.1, 10.7 | -0.8, 1.8 |
| **Combined, as returned, mm** | **40.2** | **12.9** |
| Tilt peak, as returned, deg | 4.73 | 2.04 |
| Combined with the height's mean removed, mm (not what the function returns) | 24.2 | 12.9 |

- This is one flight against one run, and the flight flew a low-pass the simulation did not (the rig default on 29 Oct).
  Today's real-world default is the 5.3 Hz band-stop, with no 1 m figure-eight on it yet.
- The real flight's height part is nearly all a constant offset: the ring rides ~3 cm low. Sideways the ratio is 1.7.
- Why the older numbers differ: the rig summary's "off the path" (24.4 mm on f4) measures square to the path at the
  reference point of that instant, over its own window (23.1-55.8 s, the hold included), height 33.5 mm. The same
  summary on R2835 runs to LAND at 103 s, 45 s after the lap (8.0 mm sideways, 9.4 mm height), so it is not the same
  kind of window and is not used here.

For the paragraph: in the one metric the real figure-eight gives 21.7 mm sideways and 33.8 mm in height, 40.2 mm
combined, against 12.8, 2.0 and 12.9 mm in simulation. The real ring's height error is a constant 32 mm offset.

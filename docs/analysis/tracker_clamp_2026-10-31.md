# The tracker's pull clamp (31 Oct 2026)

Wesley, 31 Oct: `ac[2] = min(ac[2], 0.0)` in `_condition_measured_cable`. How often does it bind, on real flights and
in simulation, does it matter, and should it become a switch?

Offline and exploratory, one pass, no card, not reviewed. Nothing was flown or launched and no flight code changed.
Tree: week8 at 9f20f8b with the uncommitted rod work, as it stood at 20:49. `src/tracker` has no uncommitted change.
Scripts and rows: `results/analysis/tracker_clamp_2026-10-31/`.

Signs and units: the raw pull is `acm_z` in m/s^2, below zero means pulling down. Roll + means the rider's plate up. E
and F_z are in N. "Over 0" means raw above zero. "1 s" means the mean of the last 1 s is above zero, so noise does not
count.

## Verdict

| # | Finding | Number | Source |
|---|---|---|---|
| 1 | A carrier under load does not touch it | Real 0.11 %, simulation 0.12 % of ticks | Logs |
| 2 | It binds when the plate's upward force F_z is under E, the map's and the mass's error | Section 1.2 | Logs, form |
| 3 | R3419's tilt is the clamp | Flown +3.37, solved +2.69, no clamp +0.05 | Solved, not flown |
| 4 | R3290's tilt is the clamp too | Flown +26.44, solved +26.10, no clamp -0.08 | Solved, not flown |
| 5 | Sensitive on one side only | 0.65 deg per 0.001 throttle of rider offset | Solved |
| 6 | The one long real T's rider was not on it | t3p_2: raw -0.72, 0.1 % over 0, 45 s | Logs |
| 7 | Real free quads sit on it | 64-83 % over 0, 19-36 mm high at rest | Logs |
| 8 | Added for the floor, gated now. Spikes are what it still stops | Up to 12.8 m/s^2 | History, logs |
| 9 | No claim cell's T has a positive-E rider | Riders are drone 3 or drone 1 | Configs |

**Propose only, for Wesley:** make the clamp a bound with a parameter, default 0 as today, typed +0.5 m/s^2 in
simulation if the runs of section 4 pass. No real-flight change. On real flights a bound would change only free quads,
and no real free quad has ever flown the positive side of this loop.

## What the logs show and what is inferred

**Shown by the logs:**
- Every percentage, mean and SD in section 2.
- The rule's checks: free drones read E / 0.55 per plant offset, riders' rolls follow F_z from the raw pull, and F_z
  from the rods' statics agrees with F_z from the raw pull.
- R3419's rider 100 % over 0 in the settled window, R3412's 0 %.
- The free drones' height error, simulated and real.

**Inferred, by offline solves on the planner's and tracker's own OCPs (the audit's `fp.py`, rest states, not
flights):**
- That taking the clamp off levels R3419 and R3290. No run has flown without the clamp.
- Every "no clamp" and "bound" number, the C of section 1.3, and what happens to the no-resize arm.
- That a real rider on the clamp would lift its plate. No real rider has been there in a steady hold.

## 1. What the clamp does

### 1.1 The path of the measured pull, one 50 Hz tick

| Step | `tracker_node.py` | What |
|---|---|---|
| Raw, logged `acm_*` | 1629-1651 | a = a_mocap - kT u R e3 + g e3. u above the offset, kT 35.2 |
| Gates | 1846-1859 | Not used: ring at rest and no planner pull, drone under rest + 4 cm, ring under rest + 5 cm |
| **Clamp** | **1688** | **z only: min(z, 0). x and y pass** |
| Filters | 1689-1700 | Low-pass and band-stop, real flights only, after the clamp |
| Cap, slew | 1703-1712 | 14.5 m/s^2 and 25 m/s^3, on the vector |
| Conditioned, logged `ac*` | 1895 | What the model gets at node 0 |
| Horizon | 1874-1878 | Planned pull at node k + (conditioned - planned pull at node 0) |
| Model | `dynamics.py:118` | v_dot = R (0, 0, kT u) - g e3 + a_c, a parameter of the tracker's OCP |

The conditioned pull reaches thrust only through that model. The throttle and attitude references come from the plan's
`ref_acc` (`acados.py:286`). The planner never sees the pull. While another stream flies the drone the raw is not
read.

**Without the clamp the tracker is offset-free in height.** At node 0 the model's acceleration is a_mocap, whatever
the thrust map or the mass says. On the real tracker OCP, one drone at rest with no height error, the first throttle
rate is zero to five decimals for any a from -1 to +1.5 m/s^2 (`stiff.py`).

**With the clamp binding the model has a sink that does not exist.** It sees kT u R33 - g = -raw, so it believes the
drone is falling at the raw value, and asks for more throttle.

### 1.2 When it binds

At rest, T R33 = m_true g + F_z. F_z is the net upward force on the plate, m_true the airframe plus the rod the drone
holds. The map gives kT u = (K / m_told) (T / K + d), with d the plant's offset minus the typed one. So:

    raw = (E - F_z) / m_told          E = (m_told - m_true) g - K d R33          K = 19.35 N per unit throttle

**It binds when F_z < E.** A rod can hold its own weight and pull nothing on the plate, F_z = 0, while still pulling
down on the drone. So the code's comment is wrong, as the audit said, and zero is the kink only when E is zero.

| Plant | m_true, kg | Offset | E, N | E / 0.55, m/s^2 |
|---|---|---|---|---|
| Drone 0 (and 4 in the 8-quad world) | 0.55 | 0.181 | +0.077 | +0.141 |
| Drone 1 (and 5) | 0.55 | 0.185 | 0.000 | 0.000 |
| Drones 2, 3 (and 6, 7) | 0.55 | 0.187 | -0.039 | -0.070 |
| R3290 and R3291's rider, told 0.55 | 0.477 | 0.187 | +0.678 | +1.23 |

Check 1, free drones in simulation, F_z = 0, windows of 20 s or more, medians. The height error is 0.130 m (section
1.3) times the mean the clamp takes off, in all three rows:

| Offset | Runs | Carrier-s | Raw | E / 0.55 | Over 0, % | Mean clamped off | 0.130 x it, mm | Flown, mm high |
|---|---|---|---|---|---|---|---|---|
| 0.181 | 2 | 110 | +0.142 | +0.141 | 97 | 0.150 | 19.5 | 20.4 |
| 0.185 | 50 | 7220 | +0.009 | 0.000 | 62 | 0.021 | 2.8 | 3.3 |
| 0.187 | 23 | 2827 | -0.061 | -0.070 | 7 | 0.006 | 0.8 | 1.3 |

Check 2, F_z two ways on the settled T's. "Statics" uses only the logged drone pivots and the rods' equations, no
throttle.

| Run | Rider | Ring tilt, statics / log | F_z, statics | F_z = E - 0.55 raw |
|---|---|---|---|---|
| R3419 | d0 | 3.34 / 3.37 | +0.064 | +0.050 |
| R3412 | d3 | 0.07 / 0.12 | +0.001 | -0.008 |
| R3206 | d3 | 3.31 / 3.33 | -0.057 | -0.069 |
| R3000 | d3 | 5.17 / 5.19 | -0.091 | -0.104 |
| R2969 | d3 | 6.75 / 6.77 | +0.131 | +0.125 |
| R3236 | d3 | 2.93 / 2.86 | -0.056 | -0.062 |
| R3290 | d3 | 26.43 / 26.44 | +0.525 | +0.514 |

### 1.3 The steady state when it binds

The drone is at rest only if something balances the model's sink.

**A free drone: the height error does.** The OCP's first throttle rate is zero at e = 0.130 m per m/s^2 of raw, for
every raw from 0.05 to 1.5 (`stiff.py`). The throttle is the plant's own hold throttle. Only the height is wrong:
0.236 m per N of E, drone above its reference.

**A near-slack carrier on a rod: the plate does.** Node 0 of the plan is pinned to the measured pose, so the tracker's
node-0 error stays near zero (0.09 mm in R3419). The balance moves into the horizon. The plan wants the plate back at
level, the reference falls away over the horizon, and the tracker takes that for the sink it believes in. The plate
rests above level, and the throttle above the hold lifts it. Three relations close it, for small angles:

| Relation | Form | On the 0.86 kg T |
|---|---|---|
| Tracker and plan | r sin(roll) = C raw | C = 0.25 m per m/s^2 (solved, 0.248-0.257 at five offsets) |
| Plant | F_z = k tan(roll), k = (h M g + r w cot e) / r | k = 1.06 N (the audit's statics) |
| Raw | raw = (E - F_z) / 0.55 | Above |

So tan(roll) = E / (k + 0.55 r / C) = E / 1.56, and **F_z is about two thirds of E** (0.65-0.70 solved, 0.76 at
R3290's large angle). With E under zero and the plan's F_z at zero, nothing happens. **So the steady error is a tilt,
with an offset beside it, and it shows in no tracker error.**

### 1.4 Against the flights

| Quantity | R3412 flown, rider 0.187 | R3419 flown, last 5 s | Over 16.7 s | Solved as flown | Solved, no clamp |
|---|---|---|---|---|---|
| Roll, deg | +0.10 | +3.37 | +2.46 to +3.38 | +2.69 | +0.05 |
| Off its point, mm | 1.2 | 9.5 | | 8.2 | 1.5 |
| Raw, m/s^2 | -0.057 | +0.049 | +0.032 to +0.063 | +0.042 | +0.133 |
| Over 0, % | 0 | 100 | 100 | | |
| Model throttle u | 0.2808 | 0.2778 | | 0.2780 | 0.2754 |
| Planned rider pull, N | 0.80 | 0.81 | | 0.81 | 0.80 |

Rider's plant offset, one number changed, typed 0.185, the rest of the T at 0.185. Solved:

| Offset | E, N | Roll as flown | Off, mm | Raw | F_z, N | Roll, no clamp | Raw, no clamp |
|---|---|---|---|---|---|---|---|
| 0.170 | +0.290 | +9.79 | 29.5 | +0.154 | +0.202 | +0.05 | +0.520 |
| 0.175 | +0.194 | +6.67 | 19.4 | +0.105 | +0.132 | +0.05 | +0.344 |
| 0.178 | +0.135 | +4.71 | 13.7 | +0.074 | +0.091 | +0.05 | +0.239 |
| 0.181 | +0.077 | +2.69 | 8.2 | +0.042 | +0.051 | +0.05 | +0.133 |
| 0.183 | +0.039 | +1.31 | 4.6 | +0.020 | +0.025 | +0.05 | +0.063 |
| 0.185 | 0 | +0.05 | 1.5 | -0.007 | +0.001 | +0.05 | -0.007 |
| 0.187 | -0.039 | +0.01 | 1.4 | -0.076 | 0.000 | +0.05 | -0.078 |
| 0.190 | -0.097 | +0.05 | 1.5 | -0.183 | +0.001 | +0.05 | -0.183 |
| 0.195 | -0.194 | +0.05 | 1.5 | -0.359 | +0.001 | +0.05 | -0.359 |

The rest states of the other riders, solved: drone 1, +0.05, drone 2, +0.01. The drone 0 root is the same from starts
at 0 and +6 deg. Riders 1 and 2 have not flown as a release. **So the T is level for three riders of four, and the
fourth is off by 0.004 throttle of map error.** The slope is 0.65 deg per 0.001 throttle, against 0.71 from the
small-angle form above.

A bound instead of zero, R3419, solved:

| Bound, m/s^2 | Roll | Off, mm | Raw |
|---|---|---|---|
| 0 (as flown) | +2.69 | 8.2 | +0.042 |
| +0.05 | +1.71 | 5.6 | +0.077 |
| +0.1 | +0.72 | 3.1 | +0.111 |
| +0.2, +0.5, +1.5 | +0.05 | 1.5 | +0.133 |

The other cells, solved:

| Cell | Flown | Solved as flown | Solved, no clamp |
|---|---|---|---|
| R3202, T, floor 0.1, rider 0.187 | +0.60, 64 mm | +0.58, 64 mm | +0.58, 64 mm |
| R3206, T, floor 0, rider 0.187 | -3.33, 74 mm | -3.75, 73 mm | -5.33, 67 mm |
| The same at rider 0.181, floor 0.1 | Not flown | +2.51, 71 mm | +0.56, 64 mm |
| The same at rider 0.181, floor 0 | Not flown | -0.21, 84 mm | -5.35, 67 mm |
| R3290, told 0.55 on 0.477, floor 0 | +26.44, 95 mm | +26.10, 93 mm | -0.08, 0.8 mm |
| R3291, the same, 5.26, floor 0.1 | +31.43, 0 mm | +29.95, 0 mm | No rest state found, start +25 |
| R3426, 1.15 kg, rod in plan, rider 0.187 | -0.11, 0.4 mm | +0.10, 0.9 mm | Not run |
| The same at rider 0.181 | Not flown | +1.82, 7.3 mm | +0.10, 0.9 mm |
| R3203, no resize, massless | -2.17, 70 mm | -2.56, 70 mm | -2.78, 69 mm |
| R3418, no resize, rod in plan | -2.09, 60 mm | -2.39, 59 mm | -2.45, 59 mm |

**R3290's whole 26 deg is the clamp, by this solve, and it is the second independent reproduction after R3419.**
Taking the clamp off moves one flown kind of cell the wrong way: R3206, by 1.6 deg, because the clamp holds that plate
up. That is the zero floor with a massless plan, which was not chosen. The no-resize arm sits on the kink with raw
near zero, so it barely moves: 0.06 and 0.22 deg, solved.

## 2. How often it binds

### 2.1 The rule for a tick and a phase

A tick counts only when the raw pull is on the clamp's path: `grounded` 0, the drone 4 cm over its rest, the ring 5 cm
over its rest, the value live. Phases come from the planner's own log, on the tracker's clock (`events.csv` runs on
another clock in simulation).

| Phase | Rule |
|---|---|
| Climb | Ring in the air, height target rising, plus 2 s |
| Hold, path | Reference at rest or moving, plan pull 1 N or more |
| Rider climb, hold, path, descent | The same with a plan pull under 1 N: a T's middle carrier |
| Unloaded | Plan pull 1 N or more, last 1 s of raw over -1 m/s^2: a no-resize arm's phantom or rider |
| Event | First 3 s after the plan's team changes |
| Unload | The drone leaves the plan within 6 s and its plan pull is under 1.5 N: a soft detach |
| Free | The drone has left the plan, ring in the air, no LAND |
| Before the fold-in | Not yet in the plan |
| After a DROP | The planner's log has ended, so the ring is not carried. Real flights mostly |

### 2.2 Real flights, 22-30 Oct

44 flights have ticks on the path, of 66 log folders. % of ticks over 0 / % over 1 s:

| Setup | Climb | Hold | Path | Descent | Rider climb | Rider hold | Rider descent |
|---|---|---|---|---|---|---|---|
| 0.55 m, 0.86 kg | 0.0 / 0.0 | 0.1 / 0.0 | 0.0 / 0.0 | 0.1 / 0.0 | 5.4 / 5.5 | 0.1 / 0.0 | 34.4 / 24.8 |
| 1 m, 1.15 kg | 0.3 / 0.0 | 0.0 / 0.0 | 0.0 / 0.0 | 0.7 / 0.0 | 8.8 / 13.4 | 0.0 / 0.0 | |

| Setup | Event | Unload | Free | Free, LAND | After a DROP |
|---|---|---|---|---|---|
| 0.55 m, 0.86 kg | | | | | 63.7 / 60.8 |
| 1 m, 1.15 kg | 3.6 / 0.0 | 0.0 / 0.0 | 68.1 / 52.2 | 63.3 / 61.5 | 82.5 / 81.3 |

Behind the cells. "Off" is the mean the clamp takes off, in m/s^2:

| Setup | Phase | Flights | Carrier-s | Raw mean | SD | Off | Largest raw |
|---|---|---|---|---|---|---|---|
| 0.55 m | Hold | 20 | 259 | -6.89 | 0.39 | 0.00 | 2.29 |
| 0.55 m | Path | 13 | 750 | -7.03 | 0.53 | 0.00 | -4.04 |
| 0.55 m | Rider hold | 2 | 45 | -0.73 | 0.29 | 0.00 | 0.05 |
| 0.55 m | Rider climb | 2 | 20 | -0.65 | 0.33 | 0.01 | 0.44 |
| 0.55 m | After a DROP | 2 | 47 | +0.50 | 0.89 | 0.60 | 4.07 |
| 1 m | Hold | 12 | 909 | -7.50 | 0.76 | 0.00 | 3.36 |
| 1 m | Path | 1 | 109 | -5.29 | 0.70 | 0.00 | -2.13 |
| 1 m | Rider climb | 1 | 10 | -0.71 | 0.47 | 0.04 | 1.21 |
| 1 m | Event | 2 | 11 | -6.90 | 1.33 | 0.06 | 2.77 |
| 1 m | Descent | 13 | 173 | -5.67 | 1.44 | 0.03 | 12.79 |
| 1 m | Free | 2 | 5 | -0.01 | 0.76 | 0.22 | 1.70 |
| 1 m | Free, LAND (QUAD4 taken back) | 6 | 31 | +0.43 | 0.48 | 0.55 | 2.47 |
| 1 m | After a DROP | 10 | 291 | +0.84 | 1.02 | 0.95 | 10.13 |

**The quiet real T's, rider only:**

| Flight | Setup | Phase | s | Plan, N | Raw | SD | Over 0, % | 1 s, % | Tilt |
|---|---|---|---|---|---|---|---|---|---|
| t3p_2 | 0.55 m | Climb | 9.9 | 0.11 | -0.79 | 0.36 | 4.4 | 5.6 | 3.6 |
| | | **Hold** | **44.9** | **0.12** | **-0.72** | **0.28** | **0.1** | **0.0** | **2.6** |
| | | Descent | 5.0 | 0.10 | -0.15 | 0.30 | 34.4 | 24.8 | 2.2 |
| t3p_3 | 0.55 m | Climb | 9.7 | 0.11 | -0.50 | 0.30 | 6.4 | 5.3 | 4.9 |
| | | Hold, the wrong quad let go | 0.4 | 0.09 | -2.72 | 1.79 | 0.0 | 0.0 | 14.8 |
| sq1 | 1 m | Climb | 10.0 | 0.13 | -0.71 | 0.47 | 8.8 | 13.4 | 9.2 |
| | | Hold, then DROP | 1.0 | 0.15 | -1.41 | 1.66 | 0.0 | 0.0 | 10.9 |
| ch1b | 1 m | Four to T, then DROP | 0.5 | 0.22 | -3.36 | 1.91 | 8.0 | 0.0 | 14.1 |
| ch1b_2 | 1 m | Four to T, first 3 s | 3.0 | 0.16 | -4.22 | 0.90 | 0.0 | 0.0 | 24.2 |
| | | Hold, then DROP | 1.6 | 0.36 | -4.57 | 1.15 | 0.0 | 0.0 | 14.4 |

The real rider pulls more than its plan on all of these, and it has never sat near raw zero in a hold. There is no
steady real T on 1 m (`thrust_margin_2026-10-31.md`, section 7). What the raw pull cannot say is why: it gives E -
F_z, not F_z.

**Soft detach.** The leaver in ch1b and ch1b_2, plan under 1.5 N: 1.1-1.2 s, raw -3.3 and -4.7, 0 % over 0. Its pull
had not come down with the plan when the magnet opened, so on these two the unload did not reach the clamp.

**QUAD4 before the weld.** Not on the path. While Tejen's stream flies it, our tracker does not read the pull (`acm_z`
is NaN in the logs, for example sq1 at 22-26 s). **No real fold-in has flown since 22 Oct:** on every logged flight
QUAD4 was never in our plan after being out of it. So Wed's sq6b is the first real look at its raw pull after the
take-over.

**Free real quads, no ring on them.** After a DROP, QUAD4 taken back, a freed quad:

| Vertical speed, m/s | 1 m: s | Raw | SD | Over 0, % | mm high | 0.55 m: s | Raw | Over 0, % | mm high |
|---|---|---|---|---|---|---|---|---|---|
| Under -0.3 | 150 | +0.61 | 1.01 | 82 | 319 | 18 | +0.33 | 69 | 248 |
| -0.3 to -0.1 | 95 | +1.15 | 0.80 | 89 | 331 | 14 | +0.95 | 76 | 326 |
| **-0.1 to +0.1** | **59** | **+0.40** | **0.96** | **61** | **36** | **13** | **+0.15** | **40** | **19** |
| Over +0.1 | 20 | +1.46 | 2.16 | 86 | 161 | 3 | +0.86 | 79 | 121 |

At rest that is 0.09-0.13 m per m/s^2, against the OCP's 0.130. `solo1` (free hover, planned pull, the same zero term)
agrees: raw rebuilt +0.16 to +0.36, 21-36 mm high. Moving, the reference runs ahead and speed and pack mix in, so I do
not read those rows as the clamp alone.

**This is the number behind "throttles as if the 33 g were not there".** A perfect map with m_told 0.55 and m_true
0.583 would give E = -0.32 N, raw -0.59. A free 1 m quad at rest reads +0.40, so E = +0.22 N. On 1 m the kink is above
zero.

**Is a real flight exposed?**

| Where | Exposed? | Evidence |
|---|---|---|
| A loaded carrier: lift, hold, path, descent | No | 0.11 % of ticks, 0.00 % over 1 s, 3408 carrier-s |
| The T's rider in a hold | Not so far | t3p_2. On 1 m no window long enough |
| The T's rider in the climb | Slightly | 4-9 % of ticks, mean -0.5 to -0.8 |
| The T's rider in the descent | Partly | 34 % on t3p_2 |
| QUAD4 before the weld | No | Not on the path |
| QUAD4 after the take-over | Unknown | Not flown. Inferred: near-free, so raw near +0.4 and on the clamp |
| A soft detach's leaver | Not on two flights | ch1b, ch1b_2 |
| A freed quad, QUAD4 taken back, after a DROP | Yes, as always flown | Height error, no ring on them |

Wed's h3t, h4m1 and the release fly even or loaded layouts. Only sq6b's take-over and the freed quad reach the clamp.

### 2.3 Simulation

740 runs with the raw pull logged, 226,052 carrier-s. In every cell the climb, hold and path are at 0.3 % or less over
0, and 0.0 % over 1 s. The loaded descent runs higher, 2.0 % overall, 6.0 % on the T's release. I did not look into
why. The rest, % over 0 / % over 1 s:

| Cell | Rider climb | Rider hold | Rider path | Rider descent | Event |
|---|---|---|---|---|---|
| Even, hover and path | | | | 0.0 / 0.0 | 0.0 / 0.0 |
| M1 four and its release | | | | | 0.0 / 0.0 |
| T, release | | 12.5 / 12.1 | 30.3 / 28.6 | 22.9 / 13.4 | 1.3 / 1.1 |
| T, join | 11.8 / 7.0 | 20.8 / 20.3 | | 37.3 / 29.7 | 13.6 / 12.5 |
| Square to T to square | | 0.2 / 0.0 | 0.0 / 0.0 | 6.7 / 0.0 | 1.4 / 2.7 |
| T to pair | 15.2 / 0.8 | 10.5 / 10.4 | 26.5 / 22.7 | 40.1 / 24.2 | 0.0 / 0.0 |
| Full M1-M3 | | 2.1 / 0.2 | 1.0 / 0.5 | | 1.4 / 1.9 |
| Chain, 8-9 quads | | 8.2 / 9.0 | 0.0 / 0.0 | 0.0 / 0.0 | 2.3 / 2.3 |
| Attach, QUAD4 | 12.3 / 1.7 | 1.6 / 0.8 | 0.0 / 0.0 | 4.1 / 0.2 | 0.3 / 0.1 |

| Cell | Unload | Unloaded (hold) | Free | Before the fold-in |
|---|---|---|---|---|
| Even, hover and path | | | 57.4 / 42.3 | |
| M1 four and its release | | 59.7 / 59.9 | 55.0 / 57.9 | |
| T, release | | 43.8 / 44.6 | 57.4 / 61.7 | |
| T, join | | | 57.1 / 60.9 | |
| Square to T to square | 0.0 / 0.0 | 0.0 / 0.0 | 68.9 / 63.6 | |
| T to pair | 10.9 / 6.7 | | 49.8 / 43.3 | |
| Full M1-M3 | | 0.0 / 0.0 | 47.6 / 48.0 | |
| Chain, 8-9 quads | 5.6 / 1.9 | 50.2 / 52.4 | 43.8 / 42.9 | |
| Attach, QUAD4 | 1.5 / 0.0 | 53.8 / 52.7 | 44.3 / 21.2 | 20.5 / 16.4 |

The network's trackers never reach it while carrying: 0.0 / 0.0, 77 runs. Across all cells:

| Phase | Carrier-s | Raw mean | SD | Off, m/s^2 | Largest raw |
|---|---|---|---|---|---|
| Climb | 24,332 | -4.49 | 0.06 | 0.000 | -1.42 |
| Hold | 108,889 | -4.65 | 0.54 | 0.000 | 7.87 |
| Path | 46,981 | -3.90 | 0.38 | 0.000 | 1.38 |
| Rider hold | 6,821 | -0.51 | 0.27 | 0.003 | 0.84 |
| Unload | 208 | -1.66 | 0.39 | 0.013 | 1.14 |
| Free | 12,983 | -0.10 | 0.16 | 0.028 | 2.25 |
| Before the fold-in | 2,658 | +0.06 | 0.28 | 0.121 | 3.34 |

Noise in simulation is small, SD 0.01-0.03 m/s^2 on a settled rider, so "over 0" there means the mean sits there.

**The settled T's, one row per run.** Last 5 s before the planner's LAND tick, rider only. F_z = E - 0.55 raw. The
roll form is the audit's, tan(roll) = r F_z / (h M g + r (F_z + w) cot e), fed only from the raw pull and the logged
elevation.

| Run | Setup | Rider, offset | Plan, N | Raw | Over 0, % | E, N | F_z, N | Form | Flown | Off, mm |
|---|---|---|---|---|---|---|---|---|---|---|
| R2969 | T, 5.26, floor 0.1 | d3, 0.187 | 0.13 | -0.297 | 0 | -0.039 | +0.125 | +6.47 | +6.77 | 0.4 |
| R2965 | same | d3, 0.187 | 0.13 | -0.360 | 0 | -0.039 | +0.159 | +8.09 | +8.30 | 1.3 |
| R3238 | same, +60 s | d3, 0.187 | 0.10 | -0.745 | 0 | -0.039 | +0.371 | +16.58 | +16.90 | 1.3 |
| R3202 | T, 0, floor 0.1 | d3, 0.187 | 0.10 | -0.075 | 0 | -0.039 | +0.003 | +0.15 | +0.60 | 63.9 |
| R3235 | same, elevation 70 | d3, 0.187 | 0.10 | -0.195 | 0 | -0.039 | +0.069 | +4.16 | +4.58 | 49.1 |
| R3236 | same, elevation 50 | d3, 0.187 | 0.10 | +0.043 | 100 | -0.039 | -0.062 | -3.26 | -2.86 | 73.1 |
| R3201 | T, 5.26, floor 0 | d3, 0.187 | 0.00 | -0.090 | 0 | -0.039 | +0.011 | +0.60 | +1.06 | 0.4 |
| R3206 | T, 0, floor 0 | d3, 0.187 | -0.00 | +0.055 | 100 | -0.039 | -0.069 | -3.99 | -3.33 | 73.6 |
| R3000 | T, 5.26, rod, old references | d3, 0.187 | 0.43 | +0.120 | 100 | -0.039 | -0.104 | -5.95 | -5.19 | 1.5 |
| R3412 | T, 0, rod in plan | d3, 0.187 | 0.80 | -0.057 | 0 | -0.039 | -0.008 | -0.41 | +0.10 | 1.2 |
| R3415 | same, elevation 50 | d3, 0.187 | 0.87 | -0.059 | 0 | -0.039 | -0.006 | -0.29 | +0.01 | 0.3 |
| R3416 | same, plan rod +10 % | d3, 0.187 | 0.89 | -0.087 | 0 | -0.039 | +0.009 | +0.48 | +0.97 | 4.4 |
| R3417 | same, plan rod -10 % | d3, 0.187 | 0.72 | -0.016 | 18 | -0.039 | -0.030 | -1.64 | -1.14 | 6.1 |
| R3419 | **same, rider drone 0** | d0, 0.181 | 0.81 | +0.049 | 100 | +0.077 | +0.050 | +2.65 | +3.37 | 9.5 |
| R3420 | T, 5.26, rod in plan, 60 s | d3, 0.187 | 0.80 | -0.088 | 0 | -0.039 | +0.010 | +0.52 | +0.28 | 0.4 |
| R3426 | T, 0, rod in plan, 1.15 kg | d3, 0.187 | 0.80 | -0.066 | 0 | -0.039 | -0.003 | -0.09 | -0.11 | 0.4 |
| R3424 | Join into T, 0, rod in plan | d1, 0.185 | 0.79 | -0.009 | 36 | +0.000 | +0.005 | +0.28 | +0.12 | 0.2 |
| R3003 | Join into T, 0 | d1, 0.185 | 0.10 | -0.034 | 0 | +0.000 | +0.018 | +1.02 | +0.82 | 63.8 |
| R3002 | Join into T, 5.26 | d1, 0.185 | 0.13 | -0.238 | 0 | +0.000 | +0.131 | +6.77 | +6.91 | 0.0 |
| R2966 | No resize, 5.26 | d3, 0.187 | 2.60 | -0.129 | 0 | -0.039 | +0.032 | +1.76 | +2.19 | 0.2 |
| R3203 | No resize, 0 | d3, 0.187 | 2.55 | +0.018 | 100 | -0.039 | -0.049 | -2.73 | -2.17 | 70.1 |
| R3418 | No resize, 0, rod in plan | d3, 0.187 | 3.32 | +0.015 | 98 | -0.039 | -0.047 | -2.63 | -2.09 | 60.0 |
| R3290 | Void: told 0.55, floor 0 | d3, 0.187 | -0.02 | +0.296 | 100 | +0.677 | +0.515 | +26.10 | +26.44 | 94.6 |
| R3291 | Void: same, 5.26, floor 0.1 | d3, 0.187 | 0.14 | -0.075 | 0 | +0.678 | +0.719 | +31.03 | +31.43 | 0.4 |
| R3400 | Rods 1 g, 0, floor 0 | d3, 0.187 | 0.00 | -0.052 | 0 | -0.039 | -0.010 | -0.79 | -0.18 | 0.5 |
| R3401 | Rods 1 g, 5.26, floor 0.1 | d3, 0.187 | 0.10 | -0.669 | 0 | -0.039 | +0.329 | +19.03 | +19.00 | 2.4 |

- **The form gives the flown roll from the raw pull within 0.0-0.8 deg on all 26.** So the raw pull really is E - F_z.
- **The clamp binds on 7 of them, on two sides.** R3419 and R3290 have E above the plan's F_z, plate up. R3236, R3206,
  R3000, R3203 and R3418 have the plan's F_z under E, plate down, and there the clamp holds the plate up.
- **R3291 is not on the clamp in its settled window**, raw -0.075, 0 %. It is just past the kink, with F_z near E, and
  its tilt also needs the plan's floor and the integral. So of the two void runs, R3290 is the direct evidence.
- **A rider with E = 0 sits on the kink and nothing happens:** R3424, 36 % over 0, +0.12 deg.

**The readings in the registry row.** The 73 % for R3419 reproduces as `events.csv`'s RELEASE to the log's end on the
node clock. That window holds the descent and the touchdown, when raw over 0 is not on the path. The same window gives
17.8 % for R3412, not 49 %, and I could not reproduce the 49 %. **In the settled window it is 100 % against 0 %**, a
cleaner split.

**Do the claim cells reach it?** Configs read, not flown. X1's T releases drone 1 or detaches plate 9, so the rider is
drone 3 (E -0.039). So do X8's rim, X2-full and `rj4_softdetach9`. `rj3` has rider drone 1 (E 0). `rj4_softdetach0`
frees drone 0, which stands 20 mm high while free. So no claim cell's T has a positive-E rider. Drone 0 as a rider was
the critic's neighbour run.

## 3. Why it is there, and what breaks without it

### 3.1 The history

| Date | What happened | Where |
|---|---|---|
| 24 Sep | `kt_trim` held the fleet down on the floor for 14 s | `2026-10-04_indi_fext.md`, critic |
| 4 Oct | Critic on the IMU path's card, must-fix 3: planned until airborne, "z component <= 0" | Same, lines 94, 101 |
| 4 Oct | Built in one fix: the airborne and grounded gates, and "z clamped <= 0" | Same, lines 111-112 |
| 5 Oct | Committed with the comment "a rod only pulls down on its drone" | `bbce103`, `git log -S` |
| 7 Oct | The mocap path: thrust through the same filter, the offset form, the touchdown blend | `0e91d86`, card |
| 21 Oct | `acm_*` logged | `75b1ec2` |
| 31 Oct | The audit calls the premise false, the critic predicts R3419 | Audit item 3, rod card critic B |

The critic's reason on 4 Oct was the floor: "Cap 12 lets the floor's ~9.8 m/s^2 through". **So it was a second belt
for the floor**, written for an IMU path with an unsynchronised thrust term and a pull held flat over the horizon.
None of that flies today. The 7 Oct card does not mention it. No card or decision ever measured it, and no test feeds
a positive z.

### 3.2 What each protection is worth today

| What it may guard against | Size | Other guard today | Left to the clamp |
|---|---|---|---|
| The floor under a drone | Raw +9.8 to +15 | `_kt_airborne`, 4 cm | Nothing seen |
| The floor under the ring | Raw mean +0.6 (R3412) | `payload_resting` swaps in the plan | Nothing seen |
| A spike after a magnet lets go | 15 at 5.1-12.8, 0.02-0.26 s, all 30 Oct | Slew, cap | **Yes, on a free quad** |
| The loop, positive side | Thrust falls, raw rises in the delay | Loaded carriers fly it | **Free quads, unflown** |
| A falling drone | Reads near zero: a fall is not a pull | Not needed | Nothing |
| Noise on a slack rod | Real SD 0.3-1.0 | See below | It makes this worse |

**The largest real spike** (RIG-1030-h3t, drone 1, 43.8-44.0 s). QUAD2 lost the ring in the heave, during LAND. The
throttle fell from 0.81 to 0.25 in 0.3 s, the thrust followed late, and raw read +12.7 while the drone flew up at 2.6
m/s. The clamp held the model's term under zero. A loaded carrier needs no clamp for this: the slew starts from far
below zero.

**Noise.** A positive sample does mean less thrust asked for. But the same noise reaches the model on every loaded
carrier already, SD 0.39-0.76, and those fly. What the clamp adds is a bias, because it cuts the top half only: with a
mean near zero, 0.4 SD of extra thrust asked. On the freed quads of ch1b and ch1b_2, SD 0.76, mean -0.01, it took 0.22
m/s^2 off, which is 0.12 N of lift made from noise. In simulation that is the 2.8 mm of check 1.

### 3.3 What breaks if it is removed

Replay of the z chain on every logged real raw pull: the bound, then the slew. No loop is closed, the thrust is as
flown.

| Carrier | Carrier-s | Raw mean | Bound | Applied mean | Largest applied | Largest cut, throttle | Over +0.25, % |
|---|---|---|---|---|---|---|---|
| Loaded | 3408 | -6.744 | as flown, 0 | -6.751 | +0.00 | 0.000 | 0.00 |
| | | | +0.5 | -6.751 | +0.50 | 0.014 | 0.03 |
| | | | +1.5 | -6.751 | +1.50 | 0.043 | 0.03 |
| | | | none | -6.751 | +2.16 | 0.061 | 0.03 |
| T rider | 83 | -0.778 | as flown, 0 | -0.786 | +0.00 | 0.000 | 0.00 |
| | | | +0.5 | -0.777 | +0.50 | 0.014 | 1.66 |
| | | | +1.5 | -0.776 | +1.21 | 0.034 | 1.66 |
| | | | none | -0.776 | +1.21 | 0.034 | 1.66 |
| Free, no ring | 373 | +0.755 | as flown, 0 | -0.157 | +0.00 | 0.000 | 0.00 |
| | | | +0.5 | +0.163 | +0.50 | 0.014 | 62.98 |
| | | | +1.5 | +0.556 | +1.50 | 0.043 | 63.23 |
| | | | none | +0.643 | +4.17 | 0.119 | 63.29 |

| If removed | Evidence | Reading |
|---|---|---|
| Loaded carriers | The replay: the same mean under all four | Nothing changes |
| A spike on a free quad | Applied up to +4.17 m/s^2, 0.12 throttle, about 0.2 s | **A bound stops this** |
| A real free quad's LAND | Raw +0.6 to +1.2. It would follow closer, 30 cm lower. Ground effect too | **Unflown** |
| A real slack cable | Clamped: a small pull. Unclamped: holds position, the cable stays slack | Inference |
| Plate-down T's in simulation | The clamp holds those plates up. Off, R3206 tilts 1.6 deg more | Solved, 1.4 |
| The OCP | Throttle floor 0.05, no constraint on a_c | Code read, cannot fail a solve |
| `kt_trim` | Reads `abs(_applied_cable_z0)`, legacy only | Code read |

## 4. Options, and the test

Nothing below is built. Real-flight rows are the replay's and reasoning, not flights.

| Option | What | R3419, solved | Real flights |
|---|---|---|---|
| **A. Leave it** | No change | +2.69, 8.2 mm | No change |
| **B. A bound, +0.5 m/s^2** | `min(z, b)`, default 0 | +0.05, 1.5 mm | Free quads: at most 0.014 throttle, 0.28 N |
| **B2. A bound, +1.5** | The same | +0.05, 1.5 mm | Free quads: at most 0.043 throttle, 0.82 N |
| **C. Only when known near-slack** | Off when plan pull is under ~1 N | +0.05, 1.5 mm | As removing it, spikes too |
| **D. An offset correction** | Simulation: type `thrust_offset` 0.181 | +0.05, 1.5 mm | Not a constant, section 2.2 |
| **E. A low-passed test** | Clamp the slow part only | About +2.69 | Takes out the noise bias, 0.12 N |

Risks:
- **A.** T's with a positive-E rider stay off, 0.65 deg per 0.001 throttle. It stays reported as the limit (decision
  31 Oct, 20:04). Free drones with E above zero stand high. No claim cell is hit.
- **B.** Small. The cells on the clamp today move, solved: R3206's kind by 1.6 deg, the no-resize arm by 0.06-0.22
  deg, so X1's table holds in sign. All of X1 re-flies on the final tree anyway. A free drone 0 flies 20 mm lower.
- **B2.** As B, and it opens the positive side on real free quads almost fully: LAND and ground effect, unflown.
- **C.** Loaded carriers hardly bind, so the condition buys nothing. It leaves free quads unbounded, where the spikes
  are, and it reads the plan, which is wrong on the no-resize arm (plan 2.6-3.3 N, real near-slack).
- **D.** No code. But the typed value stops matching real flights, the kink only moves, and the plate-down cells move
  as in B. On real flights E is not a constant: free raw runs -0.33 to +1.46 by flight, speed and pack. It would need
  a scale and a load cell.
- **E.** Does not fix what was asked: R3419's raw is steady, SD 0.009. It adds a corner and lag to a loop that swung
  at 1.1 Hz on two low-pass stages (RIG-1029-sq2_2).

**B is my pick, for simulation, and only if the runs below pass.** B over D because it removes the cause and leaves
the typed map as flown. B over removal because the slew alone lets +4 m/s^2 through on a free quad. The offsets need
+0.14, so +0.2 is already enough solved, and I have no number that separates +0.2 from +0.5. A mass error of 0.073 kg
(R3290) needs +1.23.

**Real flights: leave it for Wed 4 and Thu 5 Nov.** Nothing on those sheets needs it. Read the logs after sq6b.

**Should it become a switch?** Yes, as a typed bound, default 0, because the tree has no way to fly the test today.
Whether it needs the critic is Wesley's call: a constant, but it opens the positive side of a loop in the flight code.

### The runs: one each, exploratory, simulation only

To `x1_sqTrodl_noxy_rel2` (R3419) unless named. Expected numbers are solved. Run 5's is the no-clamp solve: its raw,
+0.004, never reaches the bound.

| # | Change | Needs | Expected | Falsified if |
|---|---|---|---|---|
| **1** | **`sim_thrust_offset: 0.187,0.185,0.187,0.187`** | Config | +0.01, 1.4 mm, raw -0.076 | Over 1 deg, 10 mm |
| 2 | `x1_sqTrodl_noxy` with `0.187,0.185,0.187,0.181` | Config | +2.69, 8.2 mm, raw +0.042 | Under 1.5 deg |
| **3** | **Bound +0.5** | **2 lines + parameter** | +0.05, 1.5 mm, raw +0.133 | Over 1 deg or 10 mm |
| 4 | Bound +0.5 on `x1_sqTrodl_noxy`, baseline R3412 | The same | No change, +0.05 | Over 0.3 deg or 2 mm of change |
| 5 | Bound +0.5 on `x1_sqTrodl_noxy_nors`, baseline R3418 | The same | -2.45, 59 mm | Over 0.5 deg or 5 mm of change |
| 6 | Bound +0.5 on `rj4_softdetach0_fig8stage_L100`, R2591 | Same | Free: 20.4 to under 3 mm | Over 3 mm, an abort |

**Run 1 first.** It is the cheapest and tests the cause with no code, about 9 min on the lab PC. If it is level the
offset was the cause and the clamp the mechanism, by the solve. Run 2 is its other sign. Runs 1-2 cannot say the bound
works, runs 3-6 do.

**Bars for keeping B, frozen here before any run:**
- Run 3: settled roll under 0.5 deg, off under 3 mm, raw +0.10 to +0.16, T6 within 1 s of R3412's, dip within 5 mm of
  R3412's.
- Run 4: settled roll and off within 0.3 deg and 2 mm of R3412.
- Run 6: as its row, and the touchdown lean within 1 deg of the baseline.
- Any failure: no bound. Record it and delete the code, and R3419 stays the reported limit.

**Decisions for Wesley:**
1. Fly runs 1 and 2, config only? Recommended.
2. If they hold, build the bound, default 0, and fly runs 3-6? It touches the tracker: critic or waiver.
3. Real flights: no change proposed. On Wed, read QUAD4's `acm_z` for 10 s after sq6b's take-over, and plate 9's
   height.

## 5. What could not be computed

| Item | Why |
|---|---|
| A real rider's F_z and its E apart | The raw pull gives E - F_z only. Needs a load cell or a scale |
| E per real quad | It moves with flight, speed and pack. No free quad at rest for long |
| A real rider's steady state on 1 m | No steady T on 1 m. sq1 dropped, ch1b_2 dropped |
| QUAD4's raw after a real fold-in | None flown since `acm_*` was logged |
| A real flight with a bound | None. The replay holds the thrust as flown, so it is not a closed loop |
| Whether the loop stays stable on a free quad with the real delays | Not modelled. The solver is quasi-static |
| M1 four and the 8-quad chain, solved | The solver does not converge on four real rods. Those rows are logs only |
| The transient: a release's peak and dip with a bound | The solver has rest states only. Run 3 reads them |
| The registry's 49 % | Not reproduced in any window I tried |
| Binding before 21 Oct | No raw pull logged. Rebuilding it was not done here |

## Reproduction

Scripts and rows in `results/analysis/tracker_clamp_2026-10-31/`:
- `scan.py` makes `all_rig.csv` and `all_sim.csv` (one row per flight, drone and phase), `tables.py` and `matrix.py`
  the tables.
- `tcells.py` the settled T's, `fz_check.py` F_z from the statics, `freetab.py` and `freeaero.py` the free drones.
- `spikes.py` the real ticks, `replay_opts.py` the replay.
- `stiff.py` the height stiffness, `jobs.py` the rest states, with `fp.py`, `plan.py` and `tracker_off.py` from the
  audit. `tracker_off.py` gained one thing, a number for `clamp_z`. `j_*.out` are the solver's lines.
- `rigts.py` and `look*.py` read single flights.

The solves ran on a copy of `src/` taken at 20:49 and not kept. The copies here read the live `src/`, so rebuilt
results may differ once the rod work moves.

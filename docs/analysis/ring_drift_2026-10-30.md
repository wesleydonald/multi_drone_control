# The ring's slow drift in the triangle: what drives it (rig logs, 29 and 22 Oct)

Written Fri 30 Oct 01:14 for the attach on plate 9 (queue item N6). Exploratory analysis on the rig logs and bags, no
claim. Scripts: `results/analysis/ring_drift_1030/` (`d1.py` the main table from our planner's and trackers' CSVs,
`d2.py` phase and restoring lean, `d3.py` autocorrelation, `d4.py` body 8 and the notch, `d5.py` equal windows, `d6.py`
QUAD4 over the ring). `d4.py` and `d6.py` read the bag caches that `results/analysis/attach_gates_1030/extract*.py`
build (their `G` / path line points at that folder). Windows as in docs/analysis/rig_1029_replays.md W2c (seconds from the planner log's start).
"Below 0.3 Hz" is a 2nd-order zero-phase low-pass of the xy about the window mean.

**Verdict:** the ring does not wander under still carriers. The whole formation wanders together: ring, carriers and
the carriers' references, coherent at 0.97-1.00 below 0.3 Hz. The trackers hold their references to 0.1-0.3 cm. The
planner moves the formation with the ring and pushes back weakly. The carriers lean only 12-24 % of the ring's offset
toward the hover point, about 1.6-3.1 N/m on the 1.15 kg ring. That sets a weakly restored 0.19-0.26 Hz mode, which
any small disturbance excites. The filters, the integral and the mocap are not the cause.

## Tables

| Window | s | QUAD4 | Ring xy below 0.3 Hz (cm) | Carriers' centroid (cm) | Ring minus centroid (cm) | Carriers' references' centroid (cm) | Carriers' tracking error, centroid (cm) |
|---|---|---|---|---|---|---|---|
| h4f_3 square, top | 5.0 | carrier | 0.61 | 0.59 | 0.26 | 0.56 | 0.12 |
| ch1b_2 square, hover | 15.0 | carrier | 1.66 | 1.34 | 0.43 | 1.36 | 0.16 |
| sq2_5 triangle, his climb | 23.5 | 1.8 m away | 2.51 | 2.24 | 0.87 | 2.26 | 0.21 |
| sq2_5, his transit | 19.8 | 0.4 m away, high | 2.32 | 1.79 | 0.65 | 1.84 | 0.26 |
| sq2_5, MATCH | 23.7 | over plate 9 | 2.33 | 2.07 | 0.55 | 2.10 | 0.21 |
| sq5_2, his climb | 8.1 | 1.9 m away | 1.39 | 1.16 | 0.44 | 1.17 | 0.11 |
| sq5_2, MATCH and 6 descents | 25.0 | over plate 9 | 3.80 | 3.00 | 1.19 | 3.02 | 0.33 |
| sq3, QUAD4 held | 38.0 | 1.3 m away | 1.89 | 1.73 | 0.52 | 1.74 | 0.17 |
| sq4_5, QUAD4 held | 33.4 | 1.8 m away | 2.60 | 2.16 | 0.68 | 2.18 | 0.17 |
| k1_4, at the pickup | 26.2 | 1.8 m away | 2.70 | 2.45 | 0.78 | 2.47 | 0.17 |
| h3 (22 Oct), 0.55 m triangle, 0.86 kg ring | 8.8 | carrier | 0.68 | 1.13 | 0.54 | 1.12 | 0.03 |

| Window | Ring vs carriers, phase (+ ring lags), coherence | References vs ring, phase (- refs lead), coherence | Carriers' lean per ring offset (m/m), corr. | Implied stiffness (N/m) and mode (Hz) | Autocorrelation trough (s, depth) |
|---|---|---|---|---|---|
| ch1b_2 square | +6.2 deg, 1.00 | -13.8 deg, 1.00 | -0.205, -0.83 | 2.67, 0.24 | 3.9, -0.46 |
| sq2_5 climb | -4.1 deg, 0.99 | -2.0 deg, 0.99 | -0.162, -0.55 | 2.11, 0.22 | 4.0, -0.32 |
| sq2_5 transit | -2.4 deg, 0.97 | -3.2 deg, 0.97 | -0.237, -0.84 | 3.09, 0.26 | 6.9, -0.39 |
| sq2_5 MATCH | -3.6 deg, 1.00 | -1.1 deg, 0.99 | -0.132, -0.59 | 1.72, 0.19 | 4.3, -0.42 |
| sq5_2 climb | +6.8 deg, 1.00 | -16.6 deg, 1.00 | -0.193, -0.46 | 2.51, 0.24 | 1.5, -0.36 |
| sq5_2 MATCH | +8.4 deg, 1.00 | -14.6 deg, 1.00 | -0.226, -0.82 | 2.94, 0.25 | 2.3, -0.42 |
| sq3 | -1.8 deg, 0.98 | -4.2 deg, 0.98 | -0.121, -0.49 | 1.58, 0.19 | 3.9, -0.34 |
| sq4_5 | -1.1 deg, 0.99 | -3.2 deg, 0.99 | -0.189, -0.76 | 2.46, 0.23 | 2.2, -0.42 |
| k1_4 | -0.2 deg, 0.99 | -4.2 deg, 0.99 | -0.127, -0.42 | 1.65, 0.19 | 2.3, -0.52 |
| h3 (0.55 m) | -6.2 deg, 1.00 | +1.0 deg, 1.00 | +0.57, +0.25 (8.8 s, settling) | - | 1.2, -0.42 |

Lean = the carriers' centroid minus the ring, regressed on the ring minus the hover point. Negative means the carriers
lean back toward the hover point. Stiffness = m g |lean| / h with m 1.15 kg and h = 1 m x sin 60 deg = 0.87 m (the
cables measured 59-62 deg, W1). Mode = sqrt(k/m) / 2 pi. A trough at T s is a half period (2T), damped where shallow.

| Window | Integral xy force, mean / variation below 0.3 Hz (N) | Integral xy gate on (1 Hz log lines) | corr(integral force, offset) | Ring tilt below 0.3 Hz (deg), corr with ring minus centroid | Carriers' throttle |
|---|---|---|---|---|---|
| ch1b_2 square | 0.00 / 0.00 | 79 % | - | 0.76, -0.44 | 0.64 |
| sq2_5 climb | 0.19 / 0.10 | 41 % | -0.11 | 1.23, -0.56 | 0.71 |
| sq2_5 MATCH | 0.30 / 0.05 | 23 % | -0.31 | 0.78, -0.58 | 0.78 |
| sq5_2 MATCH | 0.22 / 0.05 | 21 % | -0.07 | 1.66, -0.79 | 0.76 |
| sq3 | 0.17 / 0.07 | 81 % | -0.08 | 1.05, -0.57 | 0.69 |
| sq4_5 | 0.24 / 0.05 | 35 % | -0.26 | 0.95, -0.59 | 0.74 |
| k1_4 | 0.43 / 0.09 | 48 % | -0.26 | 1.11, -0.78 | 0.72 |

The integral's bound is 1.58 N in xy (ki 0.40/s, 0.30 m, int_k_xy 5.4 N/m, sq5_2.log). Its mean holds the ring
0.2-1.6 cm from the hover point in every window.

| Same 15 s windows (sliding by 5 s) | Ring xy below 0.3 Hz, median (range) |
|---|---|
| ch1b_2 square (1 window) | 1.66 cm |
| sq3, QUAD4 held 1.3 m away (5) | 1.80 (1.51-2.24) cm |
| sq4_5, QUAD4 held 1.8 m away (4) | 2.03 (1.61-2.58) cm |
| k1_4, QUAD4 at the pickup (3) | 2.85 (1.96-2.89) cm |
| sq2_5 climb / transit / MATCH (2 / 1 / 2) | 2.48 / 2.49 / 2.45 cm |
| sq5_2 MATCH and 6 descents (3) | 4.50 (4.49-4.53) cm |

| Body 8 as a sensor | Value |
|---|---|
| on the floor before lift-off, below 0.3 Hz (sq5_2, sq2_5, k1_4: 40-101 s) | 0.55-1.03 mm rms |
| in the air: ring minus the mean of the 4 plate magnet markers, below 0.3 Hz (h4f_3, f4, ch1b) | 0.06, 0.10, 0.32 cm rms |

| The 5.3 Hz notch (Q 1) on the measured pull | 0.05 Hz | 0.1 Hz | 0.2 Hz | 0.3 Hz |
|---|---|---|---|---|
| gain | 1.000 | 1.000 | 0.999 | 0.998 |
| phase | -0.5 deg | -1.1 deg | -2.2 deg | -3.3 deg |

| QUAD4 over plate 9 (MATCH to TAKEN_BACK, bags) | sq5_2 | sq2_5 |
|---|---|---|
| QUAD4 body above the ring, median (range) | 1.30 (1.12-1.41) m | 1.34 (1.27-2.07) m |
| ring toward plate 9 below 0.3 Hz, sd | 2.2 cm | 1.1 cm |
| ring tilted plate-9-down, mean | +4.4 deg | +2.7 deg |
| corr(QUAD4 height, ring toward plate 9) | +0.27 | -0.38 |

## One line per candidate

| Candidate | Verdict | Numbers |
|---|---|---|
| Our planner's slow integral (int_mode model) | **ruled out** as the driver | xy force varies 0.03-0.10 N (bound 1.58), opposes the offset (corr -0.07 to -0.31), gate on 21-81 %. It holds a static 0.17-0.43 N, which keeps the mean on the hover point |
| The trackers' references moving (planner replans) | **supported**, as the path the drift takes | the references' centroid moves 1.2-3.0 cm below 0.3 Hz, within 10-25 % of the ring, coherence 0.97-1.00, leading the ring by 1-17 deg. The planner pushes back at 12-24 % lean per offset: 1.6-3.1 N/m, a 0.19-0.26 Hz mode. The autocorrelation troughs (2.2-6.9 s, depth -0.32 to -0.52) put the wander at ~0.07-0.23 Hz, the same order |
| The carriers' own position error (slope filter) | **ruled out** | tracking error below 0.3 Hz 0.11-0.33 cm (centroid), 7-12x less than the ring's drift. The square flew slope too |
| The notch's phase at 0.1-0.3 Hz | **ruled out** | gain 0.998-1.000, phase -1.1 to -3.3 deg (30 ms). The square, flown on low-pass filters with no notch, drifts too (1.66 cm in 15 s) |
| The triangle's geometry | **unknown, minor** | triangle 1.80-2.85 cm against the square's 1.66 cm on equal 15 s windows. The lean per offset is the same in both (0.12-0.24 vs 0.205). Only one square window is 15 s long |
| Cable length (1 m vs 0.55 m) | **unknown, suggestive** | 22 Oct h3 (0.55 m, other ring, 8.8 s, settling): 0.68 cm against sq5_2's 1.39 cm over 8.1 s. Faster trough 1.2 s against 1.5-6.9 s. h is 0.48 against 0.87 m, so the same lean gives ~1.8x the restoring force |
| The ring's tilt coupling into xy | **supported, minor** | the ring's offset from the carriers' centroid is 0.26-1.19 cm (a third or less of the drift) and follows the tilt (corr -0.33 to -0.79). The bulk moves with the carriers |
| QUAD4's downwash over the ring | **unknown**: adds in one flight of two | sq5_2, descending over plate 9: 4.50 cm on 15 s windows. sq2_5, holding 0.3 m over it: 2.45 cm. QUAD4 far: 1.80-2.85 cm. Height vs displacement corr +0.27 / -0.38. The ring sits 2.7-4.4 deg plate-9-down while QUAD4 is over it |
| Mocap noise of body 8 | **ruled out** | 0.55-1.03 mm on the floor. In the air body 8 agrees with the four plate markers within 0.06-0.32 cm |

## The narrowest change for tomorrow

**On sq6b: none.** It is the attach's line, and its gates already changed (one variable per flight). The drift moves
plate 9 at the ring's own pace (~0.2 Hz). His descent and our gates judge the magnet relative to the moving plate
point, so they follow it. What the drift adds is his tracking lag on a moving target.

**If Wesley wants to test it first, his word (a planner gain, not a default):**
- A typed value already in the code: `plan_w_pos_scale` multiplies the planner's ring-position weights at run time.
  real_mode passes it only when typed, and the solver is not rebuilt.
- Fly it as a pair on the triangle hover of section 2: `$R3 load_traj:=hover h3t` as written, then the same line with
  `plan_w_pos_scale:=2` (h3t_w2), both QUAD4 on its mat.
- Bar, frozen before the flights: the ring xy below 0.3 Hz over the last 15 s, h3t_w2 at most 0.7x h3t. Falsifier:
  h3t_w2's drift not lower, or its tilt or throttle p95 higher than h3t's.
- Only the twin has flown it: x3 on the figure-8 (R1957, R1981), a small gain in path error. The twin cannot show a
  drift benefit, because its ring wanders 0.1-1.1 cm.
- Risk: a stiffer ring loop leans the carriers harder (tilt), and with the filter lag it could excite the 1.1 Hz
  sideways swing of sq2_4.
- `plan_w_s_scale` (the cable-direction weight, which resists the lean) is the other typed lever. It is untested on the
  rig and in this context.

## Limits

- The windows are not matched: QUAD4's position, the packs and the filters (square: low-pass, triangle: notch) all
  vary between them. There is no 4-quad hover with the notch yet (h4n flies first today).
- Only one square window lasts 15 s (ch1b_2). h4f_3's 5 s is too short to measure below 0.3 Hz.
- The stiffness assumes the restoring force comes from the cables' lean alone (h = 0.87 m, a level ring). The OCP's
  own velocity cost (the damping) is not identified.
- The disturbance that excites the mode is not identified: air in the cage, the ring's tilt, thrust asymmetry and QUAD4
  are all candidates. The data shows only that the loop accepts it.
- The 0.55 m comparison is one 8.8 s hover of the 0.86 kg ring on other plates, while the integral was still pulling
  in.
- The references' lead over the ring (1-17 deg) is within the planner's look-ahead. So it does not show whether the
  plan starts the motion or follows it.

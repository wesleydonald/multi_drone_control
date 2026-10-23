# Rig day review, Thu 22 Oct 2026 (0.55 m cables, QUAD2/3/4)

Tree `day-1021` be34f95 for every flight (from ls3c on: `load_trajectory.py` uncommitted, the constant-rate spin; diff in
`results/rig/2026-10-22/ls3c_code.patch`). Registry rows RIG-1022-*. Figures in `results/rig/2026-10-22/figures/`.

## Verdict
The pair problem has a strong lead: with a ~29 ms faster velocity estimate on the quads the climb swing halved and
decayed, and the pair flew a full figure-8 (one flight each; the reviewer rates it weak until an interleaved repeat).
Three quads fly well with the hold and the size-1.0 figure-8. The T hangs at ~2.5 deg as the twin said. Part C (the
release into a pair) did not happen: the release never reached the radio once, then went to the wrong quad.

## Rig findings (in order of importance)

| # | Finding | Evidence | Why it matters |
|---|---|---|---|
| 1 | **Two quads swing at 1.3-1.6 Hz in the climb; with the quads' `slope` velocity filter (lag 45-47 -> 16-17 ms) the swing halved and decayed.** Reviewer **[weak]**: one flight per config, h2slope on drained packs, the slope flights last (not interleaved), c2q (ema, 21 Oct) as calm, mechanism unproven (the throttle's lag behind vertical speed fell 41-72 -> 22-24 ms; the commanded-position trail, 101-115 ms, did not change). | Ema climbs: 2.60-4.22 deg rms at 1.4 Hz in the first 6 s; h2 lost a magnet at the top; h2t9p 12.1 and h2rts 16.7 deg p-p at LAND. Slope climbs: 1.35-1.46 deg rms, decaying; f2slope flew the whole figure-8 (3-D RMSE 3.7 cm). Ruled out as the cause: throttle headroom (h2t9p never above 0.77), the plates, the planned pull (h2m, whose ring stalled 0.37 m short), the plan's age (h2rts). | Every 2-quad thesis cell and the chain's pair stage depend on it. Needs the interleaved rig A/B and the twin with the rig's filter. |
| 2 | **The rig T hangs ~2.5 deg** (int_k_xy 0), as the twin predicted (2.3). | t3p_2: 49 s at the top, mean 2.6, median 2.2 deg; rider planned 0.12 N. | Chain step 2a (the vector split to level the T, drafted for 4.4-6.8 deg) may not be needed. |
| 3 | **The −x push**: every flight lifts 1-5 cm toward −x of its rest spot; the 5 s hold lets the integral remove it. | h3n −2.1, h3 −2.5 -> −0.8 (learned fx −0.34 N), f3 −2.3, f3big −5.1 (lap centre −4.7 cm, no hold), f3h with the hold: lap centre −0.8 cm. | Path flights need the hold (decided 09:20). Source unknown: QUAD2's cable is the short one (0.531-0.537 m) on plate 0 (+x). |
| 4 | **Corner cutting is the figure-8's size plus the rig lag.** Size 0.5 has 10.4 cm corners (0.38 m/s^2 at 0.2 m/s); size 1.0 (7 Oct's) cuts as little as 7 Oct did. | f3 corners +44 mm (twin +18); f3h +13 mm (7 Oct +9). Rig lag 0.17-0.21 s on 3 quads, twin 0.05-0.06. f2slope (slope filter, 2 quads) lag 0.10 s. | Decided: size 1.0 default (not built yet). The slope filter may also halve the lag: untested on 3 quads. |
| 5 | **The spinny line's yaw runs ~0.45 s behind** the reference; the constant-rate spin turns it into a steady −12 deg; a 1.5 Hz yaw wobble grew with it (sd 0.74 -> 1.11 deg). | ls3 (old law): max 11.5 deg, delay 0.43 s; ls3c (constant, full turn 359/360): mean −9.4, max 16.2, delay 0.49 s. | The 1.5 Hz wobble is the pair's frequency: the slope filter may calm it. The delay needs a yaw lead (card + critic) if it stays. |
| 6 | **Part C's release failed twice for operational reasons.** t3p_2: T4's OFF never reached the rig's ROS graph (no OFF in the bag). t3p_3: the OFF went to /drone_0, but the planner had put /drone_1 on plate 2 (T2 marked it "rider"); a pair quad left, the 270 deg gap dropped the ring as designed. | Bags; t3p_3 OFF at 449.35, detection 0.84 s later (the first `ros2 topic pub` messages may be lost, so the magnet's own release time is unknown). | The must-drop case worked on the rig. The release needs a tool that targets the rider and logs its time. |
| 7 | **The magnet-marker check** stayed silent on every A/B flight (gate 3: WOULD-FIRES 0), armed on the T (QUAD3 marked rider), and in t3p_3's replay would have fired 36 ms before the stretch check. In h2 (swinging) it never armed (no 1 s of stillness), so the stretch caught that release. | tip_replay on h3, f3h, ls3c, h2slope, f2slope, t3p_3, h2. | Ready for the zero-share release; a swinging ring leaves it unarmed (falls back to the stretch check). |
| 8 | **QUAD2's flight controller refused ARM 3 times** (`!ERR*`), as on 21 Oct; each cleared by a re-ARM. Two other refusals: ARM pressed before a tracker was up. | f3big (2), f3h (1); ls3c, t3p relaunch. | A Betaflight arming flag; any change needs the supervisor. |
| 9 | **Magnet margin**: QUAD3's magnet let go at 6.4 N peak pull (h2), as QUAD2's at 6.5 N (21 Oct). A pair or the T's opposite pair carries ~5 N per magnet steady. | Tracker measured pull. | ~1.5 N of margin for swings; the calm pair (slope) stays under it. |

## The flights

| Flight | What | Result |
|---|---|---|
| bench 0 | Motive ring pivot to the centre of mass | rests 2.76 cm (target 2.75), 0.7 mm off centre; attach_z 0.0175 typed |
| bench 5 | magnet markers | routed right, 91 Hz; a covered marker repeats its last pose (6 s for a ~2 s cover) |
| A1 h3n | 3q hover, no xy integral | offset 1.7 cm (21 Oct 5.5-6.6): the pivot explanation holds |
| A2 h3 | 3q hover, integral | lean 0.9-1.0 deg (21 Oct 4-6) |
| A3 f3 | figure-8 size 0.5, no hold | band: RMSE 38.3 mm MISS by 0.7, tilt 3.75 HIT; corners +44 mm |
| f3big | size 1.0, no hold | corners as 7 Oct at the same bend demand; lap 4.7 cm toward −x |
| A3' f3h | size 1.0, 5 s hold | corners +13 mm, lap centre (−0.8, −1.1) cm, 3-D RMSE 4.0 cm |
| A4 ls3 | spin, old law | yaw max 11.5 deg, 0.43 s behind; 3-D RMSE 4.6 cm |
| A4' ls3c | spin, constant rate | full turn; steady −12 deg; wobble up; 3-D RMSE 4.0 cm |
| B1 h2 | 2q hover, 0/6 | see-saw 6.6 -> 14.8 deg p-p; QUAD3 magnet off; drop and land |
| B1' h2t9p | 11/5, cap 0.9 | 12.1 deg p-p at LAND, growing (25 after LAND) |
| B1'' h2m | planned pull | slid 12-14 cm along the line at 1.3 Hz; the ring stalled 0.37 m short; LAND |
| P1 h2rts | ref_time_shift | 16.7 deg p-p at LAND, growing |
| P2 h2slope | slope filter | half the climb swing, decaying (8 -> 2 deg p-p; drained packs) |
| f2slope | 2q figure-8 size 1.0, slope | full lap; 3-D RMSE 3.7 cm; tilt peak 14.3, mean 7.3 deg |
| t3p_2 | T 11/2/5, R3on, slope | T hung 2.6 deg for 49 s; the OFF never reached; LAND |
| t3p_3 | as t3p_2 | OFF to the wrong quad (a pair quad): drop and land, as designed |

## Plan (Wesley's answers ~11:50-12:05 in `docs/decisions.md`), in order of work to Wed 28

| # | Work | What it settles | Status |
|---|---|---|---|
| 1 | **Twin, stage 1:** the 0.55 m pair hover with no filter / the rig's old filter / slope, 2 each (`b2_hover_vf_*`) | Whether the twin swings like the rig once it has the rig's filter, i.e. whether the twin can test the fix at all | done 12:26 (R2323-R2328): it does not (ema 0.03-0.05 deg against the rig's 2.6-3.7); the rig decides |
| 2 | **Card** `2026-10-22_velocity_filter.md`, bars frozen before any claim run: the twin at 1 m (pair, T release, 3 and 4 quads, old vs slope), the ring-on-slope arm (critic: a new estimator path), the rig A/B on Wed | The claim and its falsifier | written (bars frozen 12:15-12:20) |
| 3 | **Code**, between batches only (symlink install), then gate and prebuild: (a) the twin's filter = the rig's, set from the run config on the laptop and the lab PC; (b) the figure-8 size 1.0 default, figure-8 only (circles stay 0.5), and the tools' call sites; (c) the 1 m defaults (cable 1.0, the L100 worlds, the short-cable constants; given with the plan's approval); (d) a ring-on-slope option (mocap node and twin, off); (e) the RViz "release rider" button (asks the planner which drone is the rider, two clicks, repeats OFF until the detach is seen, logs the time); (f) today's analyses as tools (pair swing and band, velocity lag, rig yaw plot, cut per bend demand) | Wed's tree | (a) (d) (e) (f: pair_swing) done 12:35, gate passed; (b) (c) not needed for Wed (typed on the lines) |
| 4 | **Twin, 1 m** (lab PC evenings, laptop): the nine baselines (size-1.0 figure-8, 5 s hold, constant spin) x {old filter, slope} x 2 = 36, then the bands for both (the N2 rule); the pair and the T release with each filter; the ring-on-slope arm; the spin's yaw lag with slope | Wed's bands; the twin side of the A/B | the 36 baseline runs started 12:37 on the laptop |
| 5 | **Wed 28 sheet** (1 m), in order: props-off bench (the new cables' hanging lengths, the markers, the RViz release props-off); the first 1 m lift low (3 quads); the pair A/B; Part C; the 3-quad cells once per filter; the 2-quad cells; the 4-quad cells | The rig half | before Wed |
| 6 | **Chain step 2b** (the reaction to a loaded release): card, critic, code behind a switch, twin | Step N's transient | after 1-5 |
| 7 | After Wed: the report, a reviewer per table, the thesis section on the filter if confirmed | | |

Not doing (your calls): the -x push's cause (the hold handles it), QUAD2's FC flag, the magnet margin, chain step 2a.

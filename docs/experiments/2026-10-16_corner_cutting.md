# Corner cutting at speed: where it comes from and what changes it (Wesley 16 Oct, exploratory)

Wesley (16 Oct ~10:00, after the carry figures): "the faster speed cuts corners a bit; could the planner's or
trackers' MPC weights change so they do not sacrifice tracking at higher speeds? In-depth analysis." This revisits the
8 Oct decision "lag: report it, do not fix it" (decisions.md) for the cross-track part at speed; the decision is
Wesley's. Twin only; exploratory until an arm wins, then a claim card (and the critic for any planner change).

## 1. Where along the chain the cut appears (existing runs, no new flights)

Signed cross-track error against the exact path (`LoadTrajectory`), + = inside the curve, over the
highest-curvature quarter of the path (`scratchpad/corner_chain.py`):

| run | cell | planner's planned ring, 0.5 s ahead | ring | drones' references (centroid) | drones (centroid) |
|---|---|---|---|---|---|
| R1849 | fig-8 0.125 m/s | +9.4 mm | +9.9 | +14.8 | +14.8 |
| R1861 | fig-8 0.2 | +16.6 | +15.2 | +25.9 | +26.0 |
| R1863 | fig-8 0.3 | +32.7 | +29.9 | +49.6 | +49.5 |
| R1859 | circle 0.3 | +9.4 | +9.6 | +13.2 | +13.0 |
| R1871 | orbit 0.3 | +10.0 | +13.0 | +21.5 | +21.7 |

- **The trackers are not the cause:** the drones fly their references to within 0.5 mm across the path.
- The drones fly inside the ring by design: the rods lean into the turn (about 20 mm at the figure-8's tight bends
  at 0.3 m/s, the formation tilt a/g times the rods' drop).
- **The planner plans the cut:** its own planned ring 0.5 s ahead is as far inside as the ring ends up.

## 2. Two mechanisms

**a. Circle and orbit (steady curvature): the plan carries the error and does not correct it.** Per planner tick,
fitting plan(node 5) = a x ring(now) + b across the path: a = 0.93-0.99, b ~ 0 (R1859, R1871). The plan adds no cut
but removes almost none over 0.5 s; whatever the fleet falls short by (a steady centripetal shortfall the model does
not have) accumulates to ~1 cm at 0.3 m/s. Levers: a stronger position correction (position weight up, or the
cable-direction pinning to the flatness reference down), and less delay.

**b. Figure-8 lobes (tight, fast-changing curvature): the plan cuts the corner by choice.** At the lobes' tightest
bends (near y = +-0.25 m, curvature above 4 /m; the lobe tips at x = +-0.5 are gentler, 2 /m) the plan goes from +10-14 mm (ring now) to +27-33 mm (node 5): it adds the cut. The cost pulls every rod's angular rate
toward zero (`w_r` 3, reference 0, "damp cable swing"), but the flatness tilt the path demands turns at |jerk| / g:

| cell | peak accel | peak jerk | rod rate needed | cut the w_r term alone predicts (order of magnitude: sqrt(3 r^2 / 60), one stage, statics) | planned cut at the ends |
|---|---|---|---|---|---|
| fig-8 0.125 | 0.15 m/s^2 | 0.19 m/s^3 | 0.019 rad/s | 4 mm | 9.4 mm |
| fig-8 0.2 | 0.38 | 0.76 | 0.078 | 17 mm | 16.6 mm |
| fig-8 0.3 | 0.86 | 2.65 | 0.27 | 60 mm (peak) | 32.7 mm (mean), 51.5 (max) |
| circle 0.3 | 0.18 | 0.12 | 0.012 | 3 mm | 9.4 mm (mechanism a) |

So on the figure-8 it is cheaper for the OCP to cut the corner than to swing the rods as fast as the path needs.
Levers: a consistent rate reference (the rate the flatness directions turn at, instead of zero), or a lower w_r.

**Delay (both):** today the trackers fly a plan 0.075 s old (`tools/lag_metrics.py` D, R1849-R1871; 8 Oct: 0.096)
and the plan recovers 20-35 % of the along-path gap by node 5 (c). `ref_time_shift` (an existing tracker switch,
parked 8 Oct) halved D on 8 Oct and cut the corner cutting there by about two thirds:

| runs (8 Oct, week4, four carriers) | cell | ring cut, top-curvature quarter | plan, node 5 |
|---|---|---|---|
| R1127, R1154 (base) | fig-8 0.2 m/s | +12.4, +14.9 mm | +12.9, +15.3 |
| R1176, R1177 (ref_time_shift) | fig-8 0.2 | +4.7, +5.5 | +5.7, +5.6 |
| R1152, R1153 (base) | orbit 0.125 | +4.9, +4.4 | +1.8, +5.0 |
| R1165, R1166 (ref_time_shift) | orbit 0.125 | +2.1, +2.0 | +0.6, +0.4 |

## 3. Arms (exploratory, one variable each against today's 0.3 m/s cells; lab batch 17)

| arm | switch (all off by default) | mechanism it targets | configs |
|---|---|---|---|
| base | none | | `traj3_{fig8,circle,orbit}_v300` (+ R1859/R1860, R1863/R1864, R1871/R1872) |
| rts | `ref_time_shift: true` (tracker, exists) | delay | `cc_rts_*` |
| wp3 | `plan_w_pos_scale: 3` (new, 63e4b52) | a: stronger correction | `cc_wp3_*` |
| ws03 | `plan_w_s_scale: 0.3` (new) | a: looser cable pinning | `cc_ws03_*` |
| wr01 | `plan_w_r_scale: 0.1` (new, baf50e1) | b: cheaper rod swing | `cc_wr01_*` |
| rff | `plan_rate_ff: true` (new, 98a2b8e) | b: consistent rate reference | `cc_rff_*` |

The new switches set acados weights at runtime (the compiled solver is unchanged) or the rate reference; 1.0 / off
reproduces today's planner exactly (unit tests `test_scale_cost.py`, `test_rate_ff.py`; gate --quick passes).

Measured per run: the cross-track table above (ring and plan), path-only RMSE (`payload_rmse_sweep_m`), the frozen
carry metrics, peak ring tilt on the path, drone tilt and thrust peaks (does a winner buy tracking with harder
flying?), solve time and failures, aborts. Falsifier per arm: no lower ring cut at the top-curvature quarter than the
base runs on the shape it targets, or any abort, drop, fault or solver failure streak.

## 4. Before anything becomes a default

The winner(s) go through the neighbours (0.125 and 0.2 m/s, hover, the lift and landing, a detach and the attach),
a claim card with frozen bars, the critic (a planner reference or cost change is in the flight loop), the reviewer,
and Wesley's word (a launch default). On the rig, `ref_time_shift` was planned as a figure-8 A/B on 8 Oct (r302) and
never flown.

## First results (lab batch 17, stopped at ~11:20 for another user; figure-8 at 0.3 m/s only, one run per arm)

| arm | run | ring inside at the tight bends, mean / max (mm) | plan node 5 there | path-only RMSE (mm) | D / A0 (s) | note |
|---|---|---|---|---|---|---|
| base | R1955 (+ R1863, R1864) | 20.7 / 34.8 (29.9, 26.9) | 22.3 | 16.9 (20.9, 20.2) | 0.075 / +0.013 | base spread is wide |
| ref_time_shift | R1956 | **2.2** / 20.0 | 6.9 | 31.5 | 0.023 / **-0.069** | the cut is gone, but the ring runs 0.07 s AHEAD: along-path rms 31.9 against 12.5 mm |
| position weight x3 | R1957 | 16.4 / 26.9 | 19.3 | 16.4 | 0.078 / +0.003 | a little better on both; throttle peak 0.671 against 0.648 |
| cable-direction weight x0.3 | R1958 | 37.1 / 64.6 | 37.9 | 26.4 | | worse: falsified |

**Why ref_time_shift overshoots:** the planner's clock steps one tick (0.1 s) before each solve, a lead that offset the
trackers' 0.075 s plan age. With the age removed (D 0.023 s) the lead is no longer cancelled and the ring runs ahead.
The 8 Oct card named removing the lead as the next variable. Switch built: `plan_ref_lag_s` (482a710; 0.1 cancels the
lead; unit-tested). Next, on the laptop (one at a time): ref_time_shift + plan_ref_lag_s 0.1, the rate reference,
the rate weight x0.1 (figure-8 0.3 m/s), then the circle and orbit for the arms that help.

## Laptop runs (16 Oct 11:48-12:00, figure-8 at 0.3 m/s, one each; R1973-R1975 VOID: launched without the acados path)

| arm | run | ring inside at the tight bends, mean / max (mm) | path-only RMSE (mm) | ring tilt on the path (deg) | D / A0 (s) | solve p95 (ms) | verdict |
|---|---|---|---|---|---|---|---|
| ref_time_shift + plan_ref_lag_s 0.1 | R1976 | 14.9 / 44.2 | 18.3 | 2.7 | 0.040 / +0.074 | 84 | the lead over-cancelled: the ring 0.07 s behind |
| rod-rate reference (plan_rate_ff) | R1977 | 29.7 / 39.1 | 19.4 | 3.6 | 0.088 / +0.045 | 95 | **falsified**: no lower cut, more tilt, solve near the 100 ms budget |
| rod-rate weight x0.1 | R1978 | 22.6 / 37.6 | 19.4 | 3.4 | 0.088 / +0.023 | 84 | **falsified**: no lower cut, more tilt |

**Mechanism b (the rod-rate cost) is refuted:** freeing the rod rate, or giving it the flatness rate, does not shrink
the figure-8's cut. What moves it is timing: ref_time_shift removes the cut (R1956: 2.2 mm) but leaves the clock's
one-tick lead uncancelled (ring 0.07 s ahead); cancelling the whole lead (R1976) leaves it 0.07 s behind. Next:
plan_ref_lag_s 0.05 (x2), and repeats of position x3 and the base (laptop queue, 12:0x).

## Repeats (laptop, 12:00-12:16; R1979-R1982) and where this stands

| arm | runs (host) | ring cut at the tight bends (mm) | path-only RMSE (mm) | A0 (s) |
|---|---|---|---|---|
| base | R1863, R1864, R1955 (lab); R1982 (laptop) | 29.9, 26.9, 20.7; 20.1 | 20.9, 20.2, 16.9; 25.1 | +0.024, -, +0.013; -0.030 |
| ref_time_shift | R1956 (lab) | 2.2 | 31.5 | -0.069 |
| + plan_ref_lag_s 0.05 | R1979, R1980 (laptop) | 17.3, 4.5 | 21.1, 23.8 | -0.008, -0.036 |
| + plan_ref_lag_s 0.1 | R1976 (laptop) | 14.9 | 18.3 | +0.074 |
| position x3 | R1957 (lab); R1981 (laptop) | 16.4; 14.9 | 16.4; 21.8 | +0.003; -0.013 |
| cable direction x0.3 | R1958 (lab) | 37.1 | 26.4 | |
| rod-rate weight x0.1 | R1978 (laptop) | 22.6 | 19.4 | +0.023 |
| rod-rate reference | R1977 (laptop) | 29.7 | 19.4 | +0.045 |

The laptop's planner solves slower (base p95 99.7 ms against 75-78 on the lab), so compare within a host.

**Reading (exploratory, n = 1-2 per arm):**
- Weights are not the fix. The trackers already fly their plan; the planner weights tried either do nothing
  resolvable (position x3: a little lower on both hosts, inside the base spread) or make it worse (cable direction
  x0.3, the two rod-rate arms).
- The cut is a timing effect of the chain: the trackers fly a plan ~0.08 s old. ref_time_shift removes the cut, but
  then the planner clock's one-tick lead has to be re-tuned (0 s: 0.07 s ahead; 0.1 s: 0.07 s behind; 0.05 s: centred
  on the means, but with a wide spread). The total path error does not yet drop below the base's spread.
- The base itself scatters 17-25 mm (path) and 20-30 mm (cut) at 0.3 m/s; resolving a 3-5 mm gain needs n >= 3 per arm
  per shape, on one host (the lab, when it is free).

**Candidates for the next step (Wesley's call):**
1. Repeats on the lab: base, ref_time_shift + lag 0.05, position x3; figure-8, circle, orbit at 0.3; n = 3 (27 runs).
2. The principled timing fix (8 Oct "rung c"): predict the planner's start state forward by the measured delay instead
   of shifting the trackers' reference; new code in the flight loop: card and critic first.
3. A faster planner solve (75-100 ms p95 at 10 Hz is most of the delay): fewer SQP iterations or a shorter horizon.
4. Or report it (the 8 Oct decision): 1.5-3 cm inside at the figure-8's tight bends at 0.3 m/s, ~1 cm at 0.125.

## Faster planner solve (Wesley ~12:20: "faster planner solve")

**What the solve costs.** The planner log's `solve_ms` holds its last value after the path ends (the figure-8 and the
circle end before LAND), so whole-run percentiles read ~75 ms (lab) / ~100 ms (laptop) for those shapes. On the path
alone (unique ticks):

| host | runs | solve p50 / p95 / max (ms) | share over 50 ms |
|---|---|---|---|
| lab twin (4 slots) | R1955, R1859, R1871, R1863 | 21-26 / 78-83 / 86-88 | 0.39-0.45 |
| laptop twin | R1982, R1979 | 50-51 / 96-97 / 120-128 | 0.49-0.54 |
| rig laptop (14-15 Oct, planner phase) | a1, a2_2, a3_2, t5_*, t6* | 31-46 / 46-65 (the rig's 0.06 s budget) / 67-116 | 0.01-0.25 |
| offline, the solver alone (laptop) | `scratchpad/solve_bench.py`, figure-8 0.3 | 20.6 / 21.0 / 21.7 | 0 |

So in the stack the solve is 2-4 times slower than alone, and bimodal: CPU contention with Gazebo and the other nodes
(acados runs outside the GIL; the node's Python work and the other processes compete for the cores).

**Iterations (offline, the planner flying the figure-8 on its own model, no delay):**

| SQP iterations per tick | solve p50 (ms) | acados alone (ms) | plan error on the model, p50 / max (mm) |
|---|---|---|---|
| 1 | 7.4 | 3.2 | 5.7 / 12.4 |
| 2 | 10.8 | 6.5 | 5.2 / 12.4 |
| 3 | 13.8 | 9.5 | 5.0 / 12.0 |
| 5 (today) | 20.6 | 16.1 | 4.7 / 11.3 |
| 8 | 30.4 | 25.7 | 4.4 / 10.9 |

Two iterations halve the solve for 0.5 mm of plan quality on the model (the orbit: 0.9 against 0.8 mm). Note also: with
no delay and an exact model the planner's own figure-8 error is ~5 mm; the 20-30 mm in the twin is the loop (delay and
model mismatch), not the OCP's intent. The 5 iterations were set for the slack-to-taut lift ("one RTI step cannot
track the transition"): the twin runs include the floor start and the lift. Switch `plan_sqp_iters` (e843065; a cold
reseed keeps its 15 iterations; unit-tested). Running on the laptop: `cc_it2_fig8_v300` x2 and a base.

## Iterations and the timing combination (laptop, 12:24-13:00; R1983-R1991)

**2 SQP iterations (R1983, R1985 against the laptop bases R1982, R1984):** the solve on the path halves (p50 26-28 against
49-51 ms, p95 57-63 against 93-96), no solve failure, the lift unchanged (23 ms). But the plan's age as flown drops only
0.083-0.088 -> 0.073-0.077 s (the 10 Hz hold dominates), and the cut stays inside the base spread (8.8, 24.8 against
18.4, 20.1 mm).

**The combination: ref_time_shift + plan_ref_lag_s 0.05 + 2 iterations (D 0.022-0.025 s against 0.085-0.088):**

| shape (0.3 m/s) | run | across-path mean (+ inside) / rms (mm) | along-path rms (mm) | path RMSE (mm) | ring tilt on the path (deg) |
|---|---|---|---|---|---|
| figure-8 | base R1984 | +8.2 / 14.9 | 12.7 | 19.1 | 2.8 |
| figure-8 | combination R1986, R1987 | -3.5, -4.6 / 15.4, 13.7 | 20.7, 20.7 | 24.7, 23.9 | 4.6, 4.3 |
| circle | base R1989 | +7.9 / 12.2 | 12.5 | 16.7 | 3.2 |
| circle | combination R1988 | -3.1 / 8.3 | 25.0 | 24.6 | 3.3 |
| orbit | base R1991 | +13.7 / 19.1 | 15.8 | 26.3 | 5.1 |
| orbit | combination R1990 | -5.0 / 13.0 | 31.8 | 34.5 | 4.1 |

**Reading:** the corner cutting is a latency effect and the timing fix removes it on every shape (inside 8-14 mm ->
slightly outside 3-5 mm; the across-path rms falls). But the error moves along the path (rms roughly doubles), so the
total path error does not improve, and gets worse on the circle and orbit. The planner's weights were tuned with the
latency in the loop; taking it out changes the along-path dynamics, which would need its own re-tune (the clock lead
per shape, the velocity weight) with n >= 3 per arm on the lab, or the state-prediction fix (rung c).

**Recommendation:** keep every switch off (today's flight unchanged); report the corner cutting with its mechanism (the
8 Oct decision stands): ~1 cm at 0.125 m/s, 1.5-3 cm at the figure-8's tight bends at 0.3 m/s, from the ~0.08 s plan
age in the loop, not from the trackers or a cost term. Separately, 2 SQP iterations halve the planner's solve at no
cost to the plan: worth a claim card as a CPU measure for the rig (its solve reaches the 60 ms budget), with the lift,
the attach and a detach as neighbours.

## Correction (16 Oct, ~17:00): the along-path reading above was the ruler, not the ring

The "path RMSE" and "along-path rms" above score the ring against `run.csv payload_ref`. That is the desired the
planner publishes once per 10 Hz tick (`planner_node.py`, before `traj_t` steps) and the runner holds between ticks: a
staircase 0.04-0.06 s behind the schedule. Two consequences:
- **An on-time ring is charged v x 0.1/sqrt(3).** That is 17 mm at 0.3 m/s; unit test in
  `tools/test/test_metrics.py`.
- **A ring ~0.05 s late scores best.** So the timing fix, which brings the ring onto the schedule, read as "the error
  moved along the path".

Wesley chose the ruler for this work (decisions.md, 16 Oct ~17:20):
- the ring's signed distance to the path it was sent, which needs no clock;
- the lag in seconds on the planner's own schedule, separately;
- `metrics.contour_lag` and `tools/corner_metrics.py` (commit 6970467).

The frozen metrics and every frozen carry bar stay as they are. Re-scored:

| shape (0.3 m/s) | arm | run | cut at the tight bends (mm) | across mean / rms (mm) | lag (s, + late) | plan node 5 cut (mm) | schedule RMSE (mm) | frozen RMSE (mm) |
|---|---|---|---|---|---|---|---|---|
| figure-8 | lab base | R1863 | +31.1 | +15.7 / 20.6 | +0.025 | +34.6 | 22.4 | 20.9 |
| figure-8 | lab base | R1864 | +28.5 | +12.7 / 17.9 | +0.015 | +27.6 | 19.1 | 20.2 |
| figure-8 | lab base | R1955 | +22.1 | +8.4 / 13.7 | +0.016 | +24.8 | 15.3 | 16.9 |
| figure-8 | laptop base | R1982 | +21.9 | +9.4 / 15.2 | +0.010 | +26.7 | 24.2 | 25.1 |
| figure-8 | laptop base | R1984 | +19.6 | +9.8 / 16.2 | +0.034 | +23.5 | 20.8 | 19.1 |
| figure-8 | ref_time_shift (lab) | R1956 | +3.1 | -4.1 / 9.6 | -0.057 | +9.0 | 22.3 | 31.5 |
| figure-8 | rts + lag 0.1 (laptop) | R1976 | +15.2 | +3.8 / 15.1 | +0.077 | +17.8 | 27.4 | 18.3 |
| figure-8 | rts + lag 0.05 (laptop) | R1979 | +18.8 | +3.0 / 14.7 | +0.007 | +14.3 | 18.0 | 21.1 |
| figure-8 | rts + lag 0.05 (laptop) | R1980 | +5.6 | -1.7 / 11.2 | -0.001 | +10.9 | 20.7 | 23.8 |
| figure-8 | combination | R1986 | +2.7 | -4.2 / 16.5 | +0.006 | +7.9 | 22.6 | 24.7 |
| figure-8 | combination | R1987 | +7.3 | -4.7 / 14.3 | -0.011 | +6.0 | 18.0 | 23.9 |
| circle | lab base | R1859 | +13.0 | +13.2 / 14.6 | +0.056 | +10.4 | 21.1 | 13.4 |
| circle | lab base | R1860 | +14.3 | +14.2 / 15.8 | +0.054 | +14.9 | 21.8 | 14.3 |
| circle | laptop base | R1989 | +11.6 | +9.5 / 14.5 | +0.055 | +12.0 | 23.2 | 16.7 |
| circle | combination | R1988 | -5.6 | -4.9 / 10.1 | -0.064 | -7.2 | 22.3 | 24.6 |
| orbit | lab base | R1871 | +12.7 | +13.2 / 15.2 | +0.034 | +14.6 | 19.6 | 19.3 |
| orbit | lab base | R1872 | +11.6 | +11.7 / 13.2 | +0.027 | +11.0 | 17.0 | 18.3 |
| orbit | laptop base | R1991 | +14.0 | +14.0 / 19.3 | +0.047 | +13.0 | 27.4 | 26.3 |
| orbit | combination | R1990 | -4.5 | -4.9 / 13.1 | -0.042 | -5.8 | 22.6 | 34.5 |

Over the fast part of the path (schedule speed above half its peak). "Cut" is the mean signed distance inside the
curve over the path's tightest quarter. "Lag" is the time since the schedule passed the ring's point.

**Corrected reading:**
- **The combination removes the cut on every shape** (tight bends +20..+31 -> +3..+7 mm on the figure-8, -5 mm on the
  circle and orbit) and lowers the across rms on the circle and orbit.
- **It turns the lag into a lead.** The figure-8 is centred (+0.006 / -0.011 s); the circle and orbit run 0.04-0.06 s
  ahead.
- **The schedule RMSE (both errors in one number) is neutral, not worse.**
- **The plan's node 5 carries the cut in every arm.**

What is left is the fixed 0.1 s clock lead, which matches neither the twin's delay nor the rig's. The superseded
sentences above ("the error moves along the path", "gets worse on the circle and orbit") are kept as they were scored.
Correction rows R1986c-R1991c are in the registry. Next (plan of 16 Oct): the lead sweep on the orbit (G2: is the
leftover lead pure delay?), the offline delayed-loop bench, then the one-clock predictor or the direction-at-the-ring
reference.

## G2: is the leftover lead pure delay? (laptop, 17:38-18:19; R1992-R1999)

Orbit at 0.3 m/s with ref_time_shift and 2 SQP iterations (the timing combination), sweeping the lead it removes
(`plan_ref_lag_s`). Bases in the same session, scored with `tools/corner_metrics.py --chain --D`. Pure delay predicts
a lag of 0 at 0.1 - D = 0.075 s (D 0.025 s measured with ref_time_shift).

| plan_ref_lag_s | runs | cut at the tight bends (mm) | across rms (mm) | lag (s, + late) | schedule RMSE (mm) | frozen RMSE (mm) |
|---|---|---|---|---|---|---|
| base (no shift) | R1991, R1992, R1996 | +14.0, +11.1, +15.0 | 19.3, 17.4, 18.2 | +0.047, +0.044, +0.054 | 27.4, 26.0, 26.3 | 26.3, 25.4, 22.7 |
| 0.05 | R1990 | -4.5 | 13.1 | -0.042 | 22.6 | 34.5 |
| 0.065 | R1994, R1998 | -0.4, -0.2 | 11.2, 11.8 | -0.013, -0.007 | 17.3, 16.7 | 26.6, 25.8 |
| 0.075 | R1993, R1997 | -0.7, -2.7 | 13.2, 11.2 | -0.003, -0.003 | 19.0, 15.9 | 26.9, 24.4 |
| 0.085 | R1995, R1999 | -2.3, -2.9 | 11.5, 12.4 | +0.012, +0.003 | 18.2, 16.8 | 23.4, 24.3 |

**Reading:**
- The lag crosses zero at about 0.078 s (n = 2 per point), within 0.01 s of the pure-delay prediction: G2 passes on
  the orbit.
- The cut is gone at every lead (-0.2..-2.9 mm) and the across rms falls from 17-19 to 11-13 mm.
- With the lag centred, the one-clock total falls from 26-27 to 16-19 mm. The frozen RMSE does not move (23-27 mm
  everywhere), as its staircase predicts.
- Caveat: the figure-8 was already centred at 0.05 (R1986/R1987, R1979/R1980), so the lead may depend on the shape.
  The figure-8 and circle at 0.075 are flying (laptop, from 18:34).

R1999's runner FAIL is the simulator stalling 0.7 s after LANDED; its path data are complete.

The offline bench (`tools/latency_bench.py`) failed its validation gate G1: its catch-up is about six times weaker
than the twin's (see the tool's docstring). It is kept for code paths and solve time only.

## G2 follow-up: the figure-8 and the circle at 0.075 (laptop, 18:34-18:56; R2000-R2005)

| shape (0.3 m/s) | arm | runs | cut at the tight bends (mm) | across rms (mm) | lag (s, + late) | schedule RMSE (mm) |
|---|---|---|---|---|---|---|
| circle | base | R2002 (R1989) | +10.0 (+11.6) | 10.2 (14.5) | +0.049 (+0.055) | 17.8 (23.2) |
| circle | plan_ref_lag_s 0.075 | R2003, R2005 | +0.5, -2.1 | 13.4, 5.2 | +0.008, -0.003 | 19.6, 13.8 |
| figure-8 | base | R2000 (R1982, R1984) | +32.7 (+21.9, +19.6) | 21.5 (15.2, 16.2) | +0.064 (+0.010, +0.034) | 29.5 (24.2, 20.8) |
| figure-8 | plan_ref_lag_s 0.075 | R2001, R2004 | +15.5, +9.6 | 12.7, 11.3 | +0.038, +0.033 | 19.1, 17.6 |
| figure-8 | plan_ref_lag_s 0.05 | R1986, R1987 | +2.7, +7.3 | 16.5, 14.3 | +0.006, -0.011 | 22.6, 18.0 |

**Reading:**
- **Circle and orbit (constant curvature) centre at the same lead, 0.075-0.078.** That is the pure-delay value
  0.1 - D; the cut is gone and the lag is ~0.
- **The figure-8 centres at ~0.05.** At 0.075 it runs 0.033-0.038 s late and the cut is partly back (+10..+16 against
  +20..+33).
- So the leftover lead is pure delay on steady curves, plus a further ~0.03 s on the figure-8. The figure-8's base lag
  also scatters most (0.010-0.064 s). That part is not delay; its source is open (the figure-8's changing curvature
  and speed profile are candidates).
- A lead chosen per shape stays a band-aid, never a default.

## Figure-8 repeats in one session (laptop, 16 Oct 21:03-21:37; R2016-R2022, ABCABCA)

Flown to settle the reviewer's "weak" on the figure-8's extra ~0.03 s (afternoon claim C3: the same-session base was
n = 1 and the figure-8 base lag drifts between sessions). Tree c3bf628, RTF 0.44 throughout.

| figure-8 at 0.3 m/s | runs | cut at the tight bends (mm) | across rms (mm) | lag (s, + late) | lag sd (s) | schedule RMSE (mm) |
|---|---|---|---|---|---|---|
| base | R2016, R2019, R2022 | +26.2, +25.1, +20.5 | 18.0, 20.3, 17.2 | +0.031, +0.054, +0.040 | 0.032, 0.043, 0.022 | 21.6, 28.0, 21.3 |
| plan_ref_lag_s 0.05 | R2017, R2020 | +5.2, +5.2 | 10.4, 20.1 | -0.015, -0.004 | 0.059, 0.037 | 20.0, 21.7 |
| plan_ref_lag_s 0.075 | R2018, R2021 | -8.8, -2.4 | 15.5, 12.8 | +0.032, +0.006 | 0.056, 0.077 | 24.4, 26.2 |

**Reading:**
- **At 0.05 the figure-8 repeats:** cut +2.7, +7.3 (other session), +5.2, +5.2 (this one). It is centred both times.
- **At 0.075 it does not repeat across sessions:**
  - the cut was +15.5 / +9.6 at 18:34 and -8.8 / -2.4 tonight;
  - R2018 reads the ring outside the bends (-8.8 mm) while its lag reads +0.032 s late.
- **The figure-8 lag is not a usable discriminator at this n:**
  - its within-run sd (0.04-0.08 s) is larger than the differences between arms;
  - its sign disagrees with the cut in R2018.
- **So the "figure-8 needs ~0.03 s less" point is not supported.** On the cut, both leads remove most of the bend cut
  against the same-session base (+20.5..+26.2). 0.075 scatters around zero, 0.05 sits ~5 mm inside.
- **Taken with the orbit and circle:** one lead (0.075, = 0.1 - D) does not leave a resolved figure-8 residual.

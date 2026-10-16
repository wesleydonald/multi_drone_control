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

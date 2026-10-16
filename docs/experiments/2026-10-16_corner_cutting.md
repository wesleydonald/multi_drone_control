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

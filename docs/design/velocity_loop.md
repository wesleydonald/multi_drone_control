# Velocity loop — design note (THESIS_PLAN §12.1 Stage V, D11)

**Status: built, wired behind `control_mode`, V-a and V-b PASS, V-c fails through the
lift (§8).** Design written before implementation, per D11.

---

## 1. Why

The dissipative network's references are nearly right and the tracker is where the error
is. `DISSIPATIVE_TRACKING_ISSUE.md §2`, re-measured on R0054 (2026-08-05):

| stage | radius ratio | phase lag |
|---|---|---|
| desired → reference centroid | 0.939 | +0.302 s |
| **reference → actual centroid (tracker)** | **0.869** | **+0.538 s** |
| actual centroid → payload | 0.996 | −0.117 s |

A 0.54 s lag out of an MPC with position weight 80 is the anomaly. Quan et al.'s Fig. 2
has no such stage: their dissipative controller emits **`v_d` straight to the autopilot**
(confirmed, THESIS_PLAN §1.3). This repo inserted a stiff 2 s-horizon position MPC that the
paper does not have, and that inserted stage is exactly where the error sits.

So the restructure is not a tuning change and not a guess — it removes a stage the reference
architecture never had.

**It is an experiment, not a bet.** Either outcome is a contribution: the paper asserts its
architecture and never compares it against an MPC-tracked alternative. If velocity loses, the
answer is "MPC + T-fixes, and here is the measured comparison" (E3).

## 2. The adaptation we are forced into

Their autopilot accepts velocity natively (PX4/ArduPilot-style). **Betaflight does not** — it
accepts body rates + throttle on four channels. The velocity→attitude→rate layer their
autopilot provides internally is ours to supply. Document as an adaptation, not a deviation.

```
                    (theirs, internal to the autopilot)
network ──► v_d ──► [ velocity → attitude → rate ] ──► motors
                    (ours, velocity_loop.py)
```

## 3. The loop

Per drone, at the tracker's 50 Hz, from the reference node the network already publishes
(`p_ref, v_ref, a_ff, a_cable` — **wire format unchanged**, principle 4):

```
v_sp  = v_ref + kp_pos * (p_ref - p)                     clamped to |v_sp| <= v_max
e_v   = v_sp - v
I    += e_v * dt                                         clamped to |ki * I| <= a_i_max
a_sp  = a_ff + kv * e_v + ki * I
thr, q_sp = tilt_quat_from_accel(a_sp, kT_eff, heading)  [reuse acados.py:251]
w_cmd = k_att * quat_error_vec(q_sp, q_meas)             clamped to w_max
stick = betaflight_rates_inv(w_cmd)                      [NEW: inverts dynamics.py:123]
```

Design points that are not arbitrary:

- **`v_sp` is a position-servo'd velocity**, not `v_ref` alone. The network's node position is
  the primary reference; a pure velocity feedforward integrates its own error and drifts off
  the formation. This is what an autopilot's velocity-setpoint layer does anyway.
- **`a_ff` already contains gravity and cable tension.** It is the specific thrust acceleration
  `f_i/m_i` the planner solved for, the same quantity the MPC gets as `ref_acc`, so the
  feedback terms are corrections *on top of it* and hover needs no separate gravity term.
- **The integrator is the point, not a detail.** The tracker has no integral action, and both
  the steady payload sag *and* the attach ring runaway were root-caused to that. Part A's
  deliverable is Part B's leading fix (A2.1).
- **Anti-windup**: `I` is clamped so `ki*I` cannot exceed `a_i_max` (start ±2 m/s²), frozen
  while disarmed / on the ground / landing, and reset on re-arm. An integrator that winds up
  on the stands is a lurch at takeoff.
- **Attitude error is the reduced (yaw-free) tilt error**, so a yaw disagreement never steals
  thrust-axis authority. Yaw is commanded separately toward the held heading.

## 4. Why `betaflight_rates_inv` is a bisection

The forward curve (`dynamics.py:123`) is

```
j(x) = sgn(x) * [ d·|x| + (f−d)·( g·|x|⁶ + (1−g)·|x|² ) ]      d=70, f=670, g=0.5 deg/s
```

It is strictly increasing on [0, 1] for `f ≥ d ≥ 0`, `g ∈ [0, 1]`, so the inverse exists and is
unique. There is no closed form (a sextic), and Newton needs a guard near `x=0` where the
derivative approaches `d`. Bisection on a monotone function is ~40 flops for 1e-9 and cannot
diverge — at 50 Hz that is free, and this runs in the flight path.

**The inverse must be exact against the forward curve, not approximately right.** A rate-loop
gain error is indistinguishable in flight from a bad attitude gain, and we would tune the wrong
thing. A round-trip test pins it to 1e-9 across the full stick range.

## 5. How it is guarded

- `velocity_loop.py` is a **pure class**: no ROS, no acados. Unit-testable, and usable directly
  in the SIL bench.
- Behind `control_mode: 'mpc' | 'velocity'`, **default `mpc`** (principle 3). Every verified
  configuration stays byte-unchanged until the flag is thrown.
- The MPC path is not modified. The two are alternative producers of the same four channels.

## 6. Validation ladder (§12.1)

| step | what | status |
|---|---|---|
| V-a | unit tests: rates round-trip, hover fixed point, anti-windup, integral freeze, tilt limit | **done** (21 tests) |
| V-b | SIL, free drone, lag < 0.1 s | **PASS** — ratio 1.008, lag −0.014 s, RMSE 0.029 m |
| V-c | SIL, 3-drone carry | **FAILS THROUGH THE LIFT** — see §8 |
| V-d | Gazebo hover + LAND | not run |
| V-e | Gazebo full trajectory set n=2,3,4 — **the decision number** | not run |
| V-f | detach under velocity mode | not run |
| V-g | hardware n=2 | not run |

**Nothing goes to the rig on bench evidence alone** — the bench has no contact physics, no
aerodynamics and no mocap noise (`docs/design/sil_bench.md` §7).

## 7. What would make this the wrong answer

Stated in advance so the comparison stays honest:

- If **T1 alone** closes most of the tracker-stage lag, the MPC path is fine and the terminal
  cost was simply mis-specified — take the one-line fix and keep the verified architecture.
- If the velocity loop tracks better but is noisier at the rig, the MPC's constraint handling
  is worth its lag, and that is a finding too.
- The velocity loop has **no constraint handling at all** — no thrust envelope beyond a clamp,
  no obstacle avoidance. Where the MPC's constraints matter (the approach path), it stays.

## 8. V-c result: it flies the carry, it cannot fly the lift (2026-08-06)

`tools/velocity_probe.py --tethered` (same plant as the bench, control at the real
50 Hz) tracks a 3-drone tethered carry cleanly: **ratio 1.022, lag −0.011 s, RMSE
0.036 m, peak drone tilt 33°**, starting from an airborne taut cone.

The SIL bench, which starts on the stands and runs ARM → TAKEOFF → lift, does not:

| run | gains | peak drone tilt | peak load tilt | throttle range | outcome |
|---|---|---|---|---|---|
| R0067 | kp 2, kv 4, ki 1, no tilt limit | 69° | — | — | envelope fault, drone tilt |
| R0068 | + 30° tilt limit | 39.8° | 80.6° | 0.000–0.600 | envelope fault, **load** tilt |
| R0069 | kp 1, kv 1.5, ki 0.5 | 13.4° | 74.5° | 0.000–0.436 | envelope fault, **load** tilt |

Read that carefully. The tilt limit did what it was added to do, and softening the gains
fixed the drone-side oscillation almost entirely — 39.8° → 13.4° of tilt, throttle peak
0.60 → 0.44. **The payload still capsizes.** So this is not gain tuning, and the
remaining failure is not the drone loop ringing.

The mechanism it points at: the velocity loop has **no cable term anywhere in its
feedback path**. Each drone independently servos its own position, and nothing
coordinates the three cable tensions. The MPC path feeds `a_cable` into its model for
exactly this reason. Through the lift — where tension is large, changing, and unevenly
shared as the cone opens — three independent position servos tilt the load over.

**This is the same boundary the dissipative network already has**, and it is documented
as deliberate in `dissipative_only_launch.py`: *"the pure decentralized network cannot
break the load off the ground from the shallow creep handover … OCP does creep + lift,
the network does everything after."* The velocity loop inherits that limit rather than
discovering a new one, which is consistent with the architecture being replicated —
Quan et al.'s controller is a transport controller, not a takeoff controller.

**So V-c is not "the velocity loop fails".** It is "the velocity loop cannot own the
lift", and the fair test is the one the network already gets: engage it *after* the
handover and measure the trajectory phase. That needs a mode that follows the network
handover rather than being set for the whole flight, and it is the next piece of work.
Until it exists, no Gazebo comparison against the MPC baseline is meaningful, so none
has been run.

## 9. V-d/V-e first result: tracking transformed, load attitude worse (2026-08-06)

`control_mode: velocity_after_handover` — OCP owns creep and lift, the velocity loop
takes over at the same instant the dissipative network takes the references over
(`/fleet/control_phase`, published by `dissipative_node._enter_network_phase`).

Gazebo, `diss_circle_n3` vs `diss_circle_n3_V`, one launch arg apart. Median [min–max]:

| metric | MPC (n=5) | velocity (n=3) |
|---|---|---|
| **tracker-stage radius ratio** | 0.878 [0.865–0.883] | **1.042** [1.037–1.042] |
| **tracker-stage phase lag** | 0.538 s [0.537–0.550] | **0.305 s** [0.301–0.311] |
| payload radius ratio | 0.816 [0.803–0.821] | **1.001** [0.994–1.003] |
| payload phase lag | 0.676 s [0.664–0.690] | **0.514 s** [0.494–0.514] |
| payload RMSE (sweep) | 0.272 m [0.270–0.275] | **0.215 m** [0.214–0.222] |
| per-drone settled error | 0.121 m | **0.045 m** |
| payload z settled | 0.477 m | **0.521 m** (target 0.60) |
| **payload tilt, peak** | **16.0°** [15.7–18.4] | 60.6° [23.9–69.6] |
| **payload tilt, settled** | **1.4°** | 5.7° [4.0–7.4] |
| runs ending in an abort | 1 of 6 | 2 of 3 |

**The tracking claim is confirmed, and it is not marginal.** The radius contraction the
whole issue was named for is *gone* — the tracker stage goes from losing 12% of the
commanded radius to losing none. Tracker-stage lag falls 44%, total payload lag 24%,
per-drone settled error 63%. No range overlaps anywhere. That is the architecture doing
what §12.1 predicted: the inserted position-MPC stage was the error, and removing it
removes the error. The settled height also improves (0.477 → 0.521 m against a 0.60 m
target), which is the integrator doing its job on the documented steady sag.

**And it costs load attitude.** Peak payload tilt goes from 16° to 61°, settled from
1.4° to 5.7°, and two of three runs tripped the 60° envelope and aborted (at t=59.5 s
and t=52.5 s — both *after* the sweep, so the tracking numbers above are on complete
sweep data, but the fleet did not survive the run).

That cost is the same root cause as §8, showing up in a milder form: the loop has no
cable term in its feedback path. Each drone now tracks its own reference beautifully
(0.045 m settled) while nothing coordinates the load's attitude, so the load hangs
increasingly askew and eventually trips the envelope.

**Honest reading.** This is a genuine head-to-head with each architecture winning a
different axis, which is exactly the E3 outcome §12.1 asked for — not a reason to
switch the default. Neither arm is shippable as it stands: the MPC tracks a 20% small
circle, and the velocity loop tracks the circle and tilts the load. The obvious next
question is whether a load-attitude term in the velocity loop (or a tension-sharing
term across the fleet) keeps the tracking win without the tilt, which is A2.1/A2.2
territory and would also serve Part B.

**Not yet done:** n=3 against §9.2's 5, no n=2/n=4, no fig-8 or line_x, no detach under
velocity mode (V-f), nothing on hardware.

## 10. The integrator was the instability, and removing it keeps the win (2026-08-06)

The slow post-sweep load-tilt divergence (§9) is caused by the **per-drone integrators**,
confirmed by a one-variable A/B (`vel_ki: 1.0 -> 0.0`, `diss_circle_n3_V_noI.yaml`).

| | radius ratio | payload lag | RMSE (sweep) | tracker stage | post-sweep tilt |
|---|---|---|---|---|---|
| MPC (n=5) | 0.816 | +0.676 s | 0.272 m | 0.878 / +0.538 s | 2.7° [2.1–3.6] |
| velocity, ki=1 (n=3) | **1.001** | **+0.514 s** | **0.215 m** | 1.042 / +0.305 s | **60.6°** [12.6–69.6] |
| velocity, ki=0 (n=2) | 0.947 | +0.521 s | 0.227 m | **0.997 / +0.322 s** | 5.4° [5.2–5.5] |

Two of three `ki=1` runs diverged and tripped the 60° envelope; **two of two valid
`ki=0` runs flew the full 75 s** with post-sweep tilt back down to 5.4°.

**Why the integrator does this, and why hover did not show it.** Each drone integrates
its OWN position error with nothing coupling the three. On a shared rigid load a
*differential* integral bias is a differential cable tension, i.e. a moment on the
payload, and nothing forces the three integrators to unwind symmetrically. The sweep is
what charges them: it is the only phase with a sustained tracking error. Hover for 75 s
(R0077) never diverged — peak airborne tilt 6.7° — because the error there is ~0 and
there is nothing to charge on. That is why the failure appears ~30 s AFTER the sweep,
in a phase that looks quiescent.

**The integrator was buying very little.** Dropping it costs 0.054 of radius ratio
(1.001 → 0.947) and nothing measurable in lag (+0.514 → +0.521 s). Against the MPC it
still removes essentially the whole tracker-stage contraction (0.878 → 0.997) and 40%
of the tracker-stage lag (0.538 → 0.322 s).

**So the honest conclusion for Part A is now:** the paper's architecture does what §12.1
predicted — the inserted position-MPC stage was the error — and the integral action that
Part A wanted for the sag must be **fleet-symmetric, not per-drone**, or omitted. A
common-mode-only integrator (share one vertical trim across the fleet, or subtract the
fleet mean from each drone's) is the obvious next design and is exactly the A2.1 term
Part B needs for the attach ring, where the same differential-tension mechanism is
suspected.

**Caveats.** ki=0 is n=2, against §9.2's five. No n=2/n=4, no fig-8, no detach under
velocity mode, nothing on hardware. `ki=0` also gives up the settled-height improvement
the integrator was added for, so it is a diagnosis, not the final design.

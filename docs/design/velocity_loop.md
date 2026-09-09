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

## 11. PROPOSED: the common-mode integrator lives in the NETWORK, not the trackers (2026-09-09)

**Awaiting Wesley's approval before implementation (D11).** Revised same day after
going back to both reference papers (Wesley's ask).

### 11.0 What the reference stacks actually do about steady thrust error

Both papers this project stands on carry exactly one adaptive/integral layer *below*
their swarm logic, and ours is the stack that removed it:

- **Sun et al.** track through an **INDI low-level controller**, described in their own
  words as "a sensor-based adaptive controller robust to model mismatch … hover thrust
  bias, etc." (Supplementary, INDI section). Their agile-tracking numbers lean on that
  layer absorbing exactly the thrust-model error our fixed open-loop kT does not.
- **Quan et al.**'s dissipative law itself has **no integral action** (Eq. 7–10 of the
  arXiv HTML: nonlinear rest-length springs `(1 − l(0)/l)` plus two damping terms;
  the command of Eq. 10 is assembled from expected accel + dissipative force + gravity).
  The trim lives in the **autopilot underneath** — a cascaded position/velocity loop
  of the PX4/ArduPilot family, which carries per-axis integral action per drone.
  Notably their system does not capsize with per-drone integrators; plausible reasons
  (long 3–4.5 m cables = low angular stiffness per unit tension differential, 5 kg
  load, and load-position feedback re-referencing an over-pulling drone) are not
  separable from here — but the measured fact in OUR system (§10) is that per-drone
  integral action on formation-rigid references diverges, so we do not copy theirs.
- One more check while in the paper: Quan et al.'s real-world experiments demonstrate
  **removal only** — the "joins during transport" sentence has no physical mid-flight
  attachment experiment behind it in the main text. That strengthens the thesis
  positioning (plan §1.2), and it also means the paper offers no guidance on
  post-attach trim — this design is on its own there, correctly labelled as ours.
- Flag for the methods chapter: the HTML's Eq. 10 is an **acceleration-level**
  command, while Fig. 2 draws `v_d` into the autopilot. §1.3's "velocity command
  CONFIRMED" reading should be re-verified against Supplementary S2–S4 before the
  chapter asserts it; the practical architecture here is unaffected either way.

**Consequence:** the fixed-kT decision (locked, supervisor-approved) plus a
no-integrator MPC means the droop/under-thrust family is structural, and the only open
question is *where* the single integral term goes. That is what §11.1 chooses.

### 11.1 The design choice

Three ways to make integral action fleet-symmetric were considered:

| Option | Mechanism | Rejected because |
|---|---|---|
| (a) Trackers subtract the fleet-mean integral | Each tracker needs every other tracker's integral state | New cross-drone wiring at 50 Hz; three copies of state that must agree; violates "tracker stays dumb" |
| (b) Each tracker integrates the *measured load* error | Same input ⇒ approximately equal integrals | Only approximately: nodes tick asynchronously, so residual differential bias survives — the exact mechanism §10 convicts, merely attenuated |
| (d) Per-drone **leaky** integrator (forgetting time-constant bounds the differential bias) | What a practically-tuned autopilot PID amounts to; closest to what Quan et al.'s stack implicitly runs | Bounds the divergence instead of removing it; the residual differential bias is still a standing moment, and the leak trades away exactly the steady trim we want. Kept as a fallback if (c) underperforms |
| (e) **INDI-lite / accel-feedback thrust trim** — Sun et al.'s actual answer | Measured accel vs commanded closes the thrust-model error per drone, no windup on position error | It is kT adaptation by another name, and adaptive kT is **deleted and locked out** (2026-08-05, supervisor-approved, twice). Also needs a clean accel signal in the command path we don't have. Recorded honestly: this is what both reference stacks do, and it is the first thing to revisit if the lock is ever lifted |
| (f) Raise the virtual-spring stiffness (Quan et al.'s stated knob for load attitude) | Stiffer springs shrink the droop proportionally | P-gain answer to a bias: steady droop = thrust deficit / effective stiffness, so it attenuates rather than removes, and buys oscillation risk. Their knob regulates *distribution*, not net vertical deficit |
| (g) Static apex height offset (plan A2.6 as a calibration) | Add a fixed trim to the reference apex | Open-loop: wrong the moment mass, battery, or n changes. (c) is precisely its adaptive version |
| **(c) One integrator in the reference generator** | The dissipative node already holds `p_des_load` and the measured load pose at 10 Hz; integrate the load error ONCE, add the correction identically to every node's `a_ff` | — |

**(c) is proposed.** Identical-by-construction beats approximately-equal. It is
reference-side only (the user's standing steer for Part B), the wire format is
untouched (the correction rides inside the existing `a_ff` field), and one pure-class
integrator is unit-testable in `DissipativeNetwork` without ROS.

### 11.2 The law

In the network step (10 Hz), while airborne in the network phase:

    e_L  = p_des_load.z - p_load_measured.z      # Z-ONLY by default (see below)
    I_L += e_L * dt                              # frozen when not integrating (below)
    a_I  = clip(ki_load * I_L, a_I_max) * ẑ      # default a_I_max = 2 m/s^2
    a_ff_i += a_I        for EVERY node i        # the only change to the output

**Z-only by default — a revision from the first draft, forced by scrutiny.** The
first draft integrated the full 3-vector. But an XY common-mode integral charges on
the sweep's tracking lag exactly the way §10's per-drone integrals charged — common-
mode, so it cannot tilt the load, but it would distort phase during the sweep and
overshoot after it, re-importing a milder version of the "quiescent phase goes wrong
~30 s later" signature as a tracking artefact. The measured symptom is the vertical
sag (5–7 cm steady; worse with fewer drones); z-only targets it and *cannot* couple to
the horizontal sweep. A `diss_i_load_xyz` flag enables the 3-axis version for a later
one-variable A/B if the XY error turns out to matter; it is not the default.

Whiteboard derivation of why this cannot tilt the load: each drone's commanded
specific-thrust vector shifts by the same world-frame `a_I`, so the extra force each
cable transmits to the load is (to first order in the geometry) the same vector `Δf`.
The net moment about the load centre is `Σ r_i × Δf = (Σ r_i) × Δf`, and `Σ r_i` is the
attach-ring centroid offset from the CoM — zero for the symmetric ring, small and
bounded for an asymmetric post-attach set. Contrast the per-drone case: independent
`Δf_i` make `Σ r_i × Δf_i` unbounded in the differential components, which is §10's
divergence. (For a markedly asymmetric fleet a tension-share weighting could zero the
residual moment exactly; not proposed now — measure the simple version first.)

Anti-windup and gating, mirroring the per-drone loop's rules:
- integrate only when the fleet is airborne in the network phase and not landing;
  freeze (do not zero) otherwise;
- reset on network-phase ENTRY (handover), not on attach/detach events — the droop the
  integrator holds does not vanish at a weld, and dumping it there is a step;
- clamp the *contribution* to `a_I_max` so the authority bound is gain-independent
  (same rationale as `velocity_loop.py`).

### 11.3 Parameters

`diss_ki_load` (default **0.0 = off**, so every verified config is byte-unchanged) and
`diss_a_i_max` (default 2.0). Wired through `dissipative_only_launch.py` and
`three_attach_launch.py`. Per-drone `vel_ki` stays available but the E3 candidate
config becomes `vel_ki: 0.0` + `diss_ki_load: 0.5`.

### 11.4 What would falsify it / validation ladder

- **Unit**: symmetric ring ⇒ adding the integrator changes no *relative* reference
  quantity (all `a_ff` shifted by one shared vector); clamp respected; freeze/reset
  semantics; detach mid-integration does not step any reference.
- **SIL** (`carry_hover_n3_vel`): settled load height error → ~0 without new drone
  oscillation.
- **Gazebo, the decisive test** (5 repeats, one launch arg apart, §9.2 rules):
  `diss_circle_n3_V_noI` vs the same + `diss_ki_load`. The z-only design makes a
  sharper prediction than the first draft: radius ratio, phase lag and post-sweep tilt
  should be **statistically unchanged** from ki=0 (the term cannot couple to the
  horizontal sweep or produce a moment), while settled payload height error should
  drop toward zero. Any radius-ratio or tilt shift outside the ki=0 ranges falsifies
  the decoupling claim, not just the tuning. Failure mode if tilt DOES grow: the
  residual centroid moment matters after all → the tension-share weighting above is
  the next (and last) step before declaring ki=0 the final answer.
- Then the frozen-architecture decision (Part A gate) is taken on the full table.

### 11.5 RESULT: the z-only common-mode trim does exactly what §11.2 predicted (2026-09-09)

**Gazebo, one launch arg apart (`diss_ki_load` 0 → 1.0), velocity_after_handover,
vel_ki=0, circle r=0.5 v=0.6.** Valid = full 75 s flight. Values are per-run
(n=3 each, ranges shown), sweep-window metrics:

| metric | trim OFF (R0079, R0081, R0155) | trim ON (R0159–R0161) | MPC (plan §12.1) |
|---|---|---|---|
| payload radius ratio | 0.939–0.947 | 0.935–0.949 | 0.816 |
| payload phase lag | +0.502–0.526 s | +0.505–0.516 s | +0.676 s |
| payload RMSE (sweep) | 0.224–0.230 m | 0.212–0.216 m | 0.272 m |
| payload tilt peak / settled | 19–27° / 2.8–3.3° | 18–19° / 2.5–3.2° | 16° / 2.7° |
| **payload height, t=55–65 s** | **0.498 m (10 cm sag, flat)** | **0.600 m (on target)** | ~5–7 cm sag |

The decoupling claim held: radius ratio, phase lag and tilt are inside the OFF arm's
ranges, and the ONLY thing that moved is the vertical — the load climbs from 0.498 to
0.600 over ~30 s at `ki_load=1.0` (0.580 at t=25, 0.595 at t=35, 0.600 from t=55) and
holds there. Not saturated (cap 2 m/s²); a higher gain would just get there sooner.

**SIL rung (R0149–R0154, central-lifter weld, n=3 per arm):** stronger than predicted —
without the trim the post-weld velocity-mode hover under-thrusts, sags, and capsizes
at weld+8–11 s (3/3, envelope trip at payload tilt >60°); with the trim it flies clean
3/3 (tilt peak 5–8°, settled ~1°). In that scenario the trim is *stabilising*, not just
height-correcting — the sag degrades the cone geometry until the load tips. A SIL-only
observation until seen in Gazebo with a weld.

**Yield caveat, not a trim caveat:** 4 of 10 Gazebo runs on this config died at t≈13 s
in the OCP *lift* (payload tilt >60° before the network phase starts — R0080, R0156,
R0157, R0158), so both arms are n=3 valid rather than §9.2's five. This is the
"1-in-5 lift failure" the plan already flagged, now measured at 4-in-10 and worth its
own investigation before any more batches on this config.

**Part A gate recommendation (plan §12.1, "one architecture goes forward"):**
`control_mode: velocity_after_handover` + `vel_ki: 0` + `diss_ki_load: 1.0` beats the
MPC tracker on radius ratio, lag, RMSE and height while matching its load attitude.
Not yet flipped as a launch default — that is Wesley's call; E3 gaps still open:
n=2/n=4, fig-8, detach under velocity mode (V-f), hardware.

# 2026-09-24 OCP hover height offset (open problem 9): a bounded load-height integral on the planner target

## 1. Question
With the tracker exact (linear plant, trim converged to 0.4 % of the plant) and every
drone sitting on its node-0 reference to 1 mm, the load hovers 5–6 cm HIGH in SIL
(R0470 0.654, R0472 0.663; target 0.60) and sat 5.5 cm LOW in Gazebo on the quadratic
plant (R0466 0.545). The planner's own log says `zN=0.60 d0N=0.98` while the drone is at
1.018: it asks for a 4 cm descent every cycle and the fleet never moves. On the rig the
same mechanism will turn any small static error (feedforward, rod length, mass, gain)
into a fixed height miss. Does a bounded, hover-gated integral on the planner's height
target close it, in SIL and Gazebo, without wind-up during the lift or LAND?

## 2. Mechanism (v2, rewritten against the diagnostics)
- Node 0 of the OCP is pinned to the MEASURED state, including the measured cable
  directions s_i, so every drone reference at node 0 is the drone's own position and its
  tracking error is zero by construction. The load height is therefore held only by the
  tracker's very weak position stiffness against the node-0 force budget (R0475: −10 %
  feedforward = −10 cm, about 6 m/s² per metre).
- The reference builder asks for cable directions at 45° and the nominal 45° tension at
  every node (yref_at), while the flown elevation is 42–43°. Re-solved from a node 0
  pinned at 42.5° every 100 ms, the plan's first 100 ms is "steepen the rods", an
  upward-inward acceleration the trackers execute over and over (critic 2.2). This is a
  HYPOTHESIS (no diagnostic has changed the nominal elevation yet); it fits: the offset is insensitive to
  mass and gain (R0469/R0470/R0472/R0473 all 0.654–0.663), the same size in Gazebo on the
  linear plant with the trim on (W-21:52 0.658), and ADDITIVE with DC gain 1 on the target
  (R0477: target 0.54 → 0.598).
- Closed suspects: terminal velocity reference (R0474, 0 mm), the inverted node-0 box in
  the tracker (R0476, −4 mm), planner/plant attach geometry (dist 0.50 on every rod,
  attach_z 0.025 both sides).
- On the rig the same loop turns any static error (feedforward, rods, mass, mocap origin)
  into a height miss of the same kind, in either direction; the additive, unit-gain
  behaviour is what makes a bounded target correction the right lever until node-0
  pinning itself is redesigned (its own card, not five days before the rig).

## 3. The one variable (v2 after the critic)
Planner parameters `z_ki` (1/s, default **0** = off) and `z_i_max` (m, default 0.15,
above the 13 cm a 10 % gain error costs):

    gated tick:  I += z_ki * (target_z - load_z_meas) / PLANNER_HZ,  clipped to ±z_i_max
    reference:   z_base = min(target_z, lift_z0 + lift_progress) + I     (both signs)

Gate, all true: lift complete (`lift_progress` at its cap); not `descending`, no LAND;
lateral trajectory inactive (`traj_t == 0`: frozen through any circle/line, by design);
|load vz| < 0.05 m/s; every cable gate ≥ 0.99 (rigid rods read 0.9999); |error| < 0.25 m;
payload pose fresher than 0.2 s. Outside the gate I is FROZEN, never zeroed, except: zero
at ARM. Frozen through LAND (no step on the first descent tick; `_load_down` uses the measured
z, unaffected). I lives in the planner and is passed to the reference builder every tick,
so it survives a detach resize and resumes once the gate reopens. At network entry the
network keeps the unshifted `target_z` in `p_des`; the auto-handover fires 1.5 s after the
lift cap when I is ~1 cm, so the network still sees today's ~5 cm offset (not a
regression). `_publish_load_desired`, the diag `z_tgt`, `_current_load_des` and
`_auto_handover_due` all read the unshifted target, on purpose. Stopgap, not a fix: the
force balance behind the offset (R0475) stays; the node-0 pinning redesign is its own card. Logged in the 1 Hz
`[planner cable]` line as `zI=`; WARN once when |I| > 0.7 z_i_max. Plainly: with the
nominal I at −0.06 that WARN fires 4.5 cm further for a high fault and 16.5 cm for a low
one, so it does NOT detect a 10 % gain fault (I ends near +0.07); the detectors are `zI=`
in the log (in every registry row) and kt_hat. `z_taut_gate` (0.99 sim, 0.9 on the real
launches: a typed rod length 0.7 mm short would otherwise keep the gate shut silently). τ = 1/z_ki: z_ki 0.2 → 5 s; the long hover gives ~30 s of gated time (6 τ).
Not per-drone, not a gain adaptation (kt_trim untouched, τ 1.5 s vs 5 s), node-0 pinning
untouched.

## 4. Baseline
SIL: R0472 (0.663, short window) and a fresh `carry_hover_n3_long` on the 0.64 / 83.1
plant (this card, off arm). Gazebo linear plant: the creep n3 re-fly (tomorrow) is the
off arm.

## 5. Pass / fail
| arm | supports | falsifies |
|---|---|---|
| SIL long hover, `z_ki` 0 vs 0.2 | on: load within 1.5 cm of 0.60 in the last 8 s; I settled (< 2 mm change over the last 5 s), |I| ≈ 0.06; tilt not worse than off by 1°; no oscillation > 1 cm p-p; lift time and touchdown time unchanged | > 2 cm from 0.60, I railed, p-p > 2 cm, or the lift/LAND changed |
| SIL kT +10 % typed (91.41), trim OFF, `z_ki` 0.2 | within 2 cm of 0.60 with I inside the bound (the integral alone carries a 10 % gain error) | > 3 cm, or railed |
| SIL fault-masking: kT +25 % typed, trim OFF, `z_ki` 0.2 | I rails at +0.15, the WARN is printed, and the load rises by ~+0.15 m against its own off run (it stays low: the fault is not hidden) | no WARN, or the load reads on target with I at the rail, or the load does not rise |
| SIL circle 0.4 m/s, `z_ki` 0.2 | traj_t advances on the tick the lift completes, so the gate is open for at most one tick: I stays 0 and the circle height equals the off arm (the loop does nothing for trajectories; a gated pre-trajectory hold would be new code) | I moves during the circle |
| SIL n=2 hover, trim OFF, `z_ki` 0.2 | within 2 cm of 0.60 | > 3 cm |
| Gazebo linear plant, creep n3, trim on, `z_ki` 0 vs 0.2 ×2, hover ≥ 30 s (≥ 5 τ) | 0.60 ± 2 cm on (off: 0.658, W-21:52) | anything else |
| Gazebo two drones, `z_ki` 0.2 (Wesley, interactive) | 0.60 ± 2 cm | |

## 6. Repeats
SIL one per arm (deterministic). Gazebo 2 per arm.

## 7. Cost
SIL: 5 runs, free. Gazebo: 4 (two of tomorrow's after the creep and detach runs, the
rest the day after).

## 8. New code
planner_node.py: `ZBias` (pure class: gate inputs → update, freeze, bound, warn flag,
~40 lines) + wiring (~20); reference_builder `update()` takes `z_bias`; dissipative node
freezes at network entry / resize. Launch args `z_ki`, `z_i_max` on the seven OCP launches
(default 0 / 0.15). Unit tests on the gate, bound, freeze and the warning. Removal plan:
delete the class, the wiring and the args.

## 9. Diagnostics (SIL, 0.64 / 83.1 linear plant, long hover, last 8 s before LAND)
| arm | load | run |
|---|---|---|
| baseline (FF 1.0, terminal_vel_ref false) | 0.663 | R0473 |
| terminal_vel_ref true | 0.664 | R0474 |
| cable_ff_scale 0.9 | 0.561 | R0475 |
| x0_relax_symmetric true (critic 2.3, inverted node-0 box) | 0.659 | R0476 |
| target_z 0.54 (critic 3.6, DC gain of the target lever) | 0.598 | R0477 |
The terminal velocity reference is not it. A 10 % feedforward change moves the hover by
10 cm: the height is a force-balance equilibrium set by the feedforward (a scale of
~0.94 would sit at 0.60 on this plant), and the re-pinned references carry no load-height
error to correct it. Same mechanism on the rig for any feedforward, rod, mass or gain
error, in either direction. The integral in §3 is the generic closer; the alternative is
to make the feedforward exact, which no model will be on the rig.

## Critic (2026-09-24, on the card above, before any code)
Verdict: **not ready**. (1) The precedent is a different mechanism: `diss_ki_load` adds a
force to the thrust feedforward, bounded in acceleration; a target offset through the
OCP is a new loop that has to beat the deleted load-level KF and the rejected per-drone
integrator. (2) §2's "ramp lag" is not shown and the feedforward story has the wrong
sign: the planner feeds 3.95 N against 4.16 N real at 42.5°, an under-count that should
sag, yet SIL sits high; R0475 shows height tracks the node-0 force budget steeply (about
6 m/s² per metre of stiffness). Better candidates: the 45° s_ref / t_nom references
replayed every cycle from a node 0 re-pinned at 42.5° (an upward-inward opening
transient the tracker executes over and over, which also fits the flatter rods and the
insensitivity to mass and gain), and the tracker's ±2.5 % multiplicative node-0 bound
relaxation (`controller_mpc.py:1160-1164`, ±2.5 cm of free height, inverted bounds for
negative state components). Zero-code check: log node-0 ref_acc and a_cable next to the
static share. (3) Fatal as written: `z_base = min(target_z + I, lift_z0 + lift_progress)`
clips every positive I, so the loop can only correct a HIGH hover; the Gazebo (−5.5 cm)
and kT +10 %/trim-off arms fail by construction, and 0.10 m is below the 13 cm a 10 %
gain error costs. The gate does not freeze during the circle (vz is horizontal); the
tautness gate must read ≥ 0.99 (rigid rods read 0.9999, the bug that voided the first
estimator matrix); τ 10 s against ~31 s of gated hover is borderline. Missing arms:
fault masking (a slow gain drop must rail I with a warning), n=2, detach resize (frozen
I wrong for the new fleet), network handover (two integrals in series, or a −I step in
`p_des`), a zero-code `target_z 0.54` step to measure the lever's DC gain. (4) SIL is not
representative: SIL high, Gazebo low; the only offset shown with the trim ON is in SIL,
and W-20:18 (two drones, linear, trim on) reached 0.60–0.62 with no planner term.
Freeze through LAND and at network entry, zero at DISARM. Rig: a stale payload pose
integrates a stale error; the 4.3 cm mocap origin would be "corrected" to. (5) Not asked
for; low priority five days before the rig, where n=2 has no arm.
Required: (a) apply I both ways (on z_base or the lift cap); (b) freeze during trajectory
motion, taut ≥ 0.99; (c) freeze through LAND and at network entry, network gets the
unshifted target; (d) target-step and fault-masking arms, n=2 arm; (e) rewrite §2 against
R0475; (f) **gate the build on the Gazebo linear-plant creep n3 re-fly with the trim ON
showing a residual above 2 cm.** If it holds within 2 cm, this card fixes a SIL artefact.

## Decision after the critic
Gate (f) MET the same evening: Wesley's interactive Gazebo creep n3 on the linear plant
with the trim on settled at 0.658 for 13 s (W-21:52; p-p 2 mm, drones on their references
to 1 mm, estimates frozen 83.15–83.18 vs plant 83.1). Same sign and size as SIL (R0473
0.663). The offset is real, not a SIL artefact. Next, before any rewrite of §3: the two
zero-code / one-flag diagnostics the critic named, in SIL (free): `target_z 0.54` (DC gain
of the target lever) and `x0_relax_symmetric` (the inverted node-0 box). Then the rewrite
along (a)–(e) or, if the box is the cause, a bound fix instead of a loop.

## Critic, second pass (2026-09-24, on v2 before the SIL arms)
Verdict: build it with four changes. (1) Positive I was still clipped one line further
down (`z_k = min(z_k, target_z)` in yref_at) → the cap now uses `target_z + z_bias`.
(2) The circle arm tested nothing (traj_t advances on the lift-complete tick, the gate is
open for one tick) → arm rewritten as "I stays 0, circle height equals off". (3) The
fault-masking WARN is asymmetric and does not detect a 10 % gain fault → said plainly;
detectors are zI and kt_hat; the fault arm now requires the load to rise +0.15 against its
off run. (4) n=2 arm: trim OFF. Also: pose freshness stamped in `_payload_cb`; reset once
per flight in `_enter_planner_phase`; `z_taut_gate` parameter (0.9 on the real launches);
Gazebo hover ≥ 30 s; network-entry wording corrected; z_ki 0.2 judged safe (crossover
0.16 rad/s, 50–65° phase margin). Built accordingly; ready without another pass.

## Outcome, SIL arms (2026-09-24 late evening, linear plant 0.64 / 83.1, z_ki 0.2, z_i_max 0.15)
| arm | off | on | zI | verdict |
|---|---|---|---|---|
| n3 long hover, correct gain | 0.663 (R0473) | **0.600**, p-p 2 mm, tilt 1.24 vs 1.2 (R0478) | −0.059 settled | supported |
| n3 kT typed +10 %, trim off | 0.501 (R0483) | 0.505 (R0479) | +0.150 railed, WARN | FALSIFIED: +0.15 on the reference moved the load +4 mm. Against a thrust deficit the re-pinned horizon cannot lift the fleet: the target lever has DC gain 1 downward (R0477) but ~0 upward under a typed-high gain. The integral does NOT carry a gain error; kt_trim does |
| n3 kT typed +25 %, trim off (fault masking) | — | 0.455 = never lifts off the 0.45 hang (R0480) | +0.150 railed, WARN | as designed: the fault is not hidden |
| n3 circle 0.4 m/s | 0.508 (R0484) | 0.507 (R0481) | 0.000 | as predicted: inert in the circle |
| n2 long hover, trim off | 0.643 (R0482) | **0.600**, p-p 1 mm (R0485) | −0.040 settled | supported |
| n3 feedforward 0.9 (load LOW with the correct gain), z_ki | 0.561 (R0475) | **0.600**, p-p 1 mm (R0486) | +0.037 settled | supported: upward works with an exact gain |

**SIL verdict (one run per arm, deterministic):** the integral closes a static planner
offset in both directions (−6.3 cm → 0.600, −4.3 cm n2 → 0.600, +3.9 cm low → 0.600),
is inert in the circle, and rails with a WARN under a gain fault without hiding it. It
does NOT substitute for the trim: against a typed-high gain the re-pinned horizon has no
upward authority (R0479 vs R0483: +4 mm for +0.15). Rig recipe: kt_trim ON and z_ki 0.2
together (the trim fixes the gain in ~5 s, the integral then closes what is left in
another ~10 s). Gazebo pair (creep n3, trim on, z_ki 0 vs 0.2, 30 s hover) tomorrow;
Wesley's interactive runs tonight are free.

## Gazebo, first run (Wesley, interactive, 2026-09-24 22:16, linear plant, creep n3, trim on, z_ki 0.2)
| | z_ki 0 (W-21:52) | z_ki 0.2 (W-22:16) |
|---|---|---|
| load after the lift | 0.658 settled (13 s flat) | 0.644 peak at +16 s, then 0.628, 0.615, 0.608, 0.604, 0.601, 0.600 at +18…+28 s |
| at LAND | 0.658 | 0.600 |
| drones vs reference | 1 mm | 1 mm |
| estimates | 83.15–83.18 frozen | 83.20 frozen |
| pitch-rate rms | 0.01 | 0.01–0.04 |
Converges with the designed τ ≈ 5 s from the lift overshoot; no oscillation, no WARN. One
run; the headless pair (z_ki 0 vs 0.2 ×2, 30 s hover) is tomorrow's claim.

## Gain sweep (SIL, air start, R0478/R0487–R0489)
| z_ki | τ | settles into ±1 cm after TAKEOFF | max after the lift | p-p |
|---|---|---|---|---|
| 0.2 | 5.0 s | 10.2 s | 0.619 | 2 mm |
| 0.3 | 3.3 s | 8.0 s | 0.610 | 2 mm |
| 0.4 | 2.5 s | 6.2 s | 0.601 | 1 mm |
| 0.6 | 1.7 s | 5.7 s | 0.602 | 2 mm |
No ringing at any gain. 0.4 is the setting to fly (0.6 buys 0.5 s). Creep speed-up:
`creep_vel` parameter (default 0.10, the old constant) on the seven launches; 0.2 halves
the sweep; `handover_settle_s` 1.0 instead of 2.0. Gazebo check by Wesley.

## Gazebo hover pair (2026-09-25, fixed sim inner loop, clock 500)
| run | settings | lift | last 15 s z | tilt | zI |
|---|---|---|---|---|---|
| R0557 | trim off, z_ki 0 | 17 s | 0.661 (0.661–0.662) | 0.0° | — |
| R0558 | defaults: trim on, z_ki 0.4 | 15 s | 0.600 (0.599–0.600) | 0.1° | −0.053 |
One run per arm today (with Wesley's interactive W-21:52 / W-22:26 on the same pair of
settings the evening before). The integral closes the linear plant's +5–6 cm offset in
Gazebo exactly as in SIL.

## Reviewer on the pair above (2026-09-25): not supported as a claim
Numbers confirmed (R0557 0.661, R0558 0.600; R0558 peak 0.638 at 20.2 s). But the arms
differ in four settings (kt_trim, z_ki, creep_vel, handover_settle_s), one run each, ~24 s
of gated hover against the card's ≥ 30 s. zI −0.053 covers 5.3 of the 6.1 cm; the trim moved
+0.1 % (≈ 2 mm). Corrections: the SIL "0.600 in every static-offset arm" holds for z_ki 0.2
with the trim OFF (R0488 alone at 0.4) and excludes the gain-fault arm R0479 (0.505, I at
the limit). Remedy flying now: defaults vs defaults + z_ki 0, ×2 each, LAND at 52 s.

## One-variable pair (reviewer remedy), LAND at 52 s, last 20 s before LAND
| arm | settings | runs | height | tilt |
|---|---|---|---|---|
| A | defaults, z_ki 0 | R0564, R0568 | 0.658 (0.658–0.659) | 0.04–0.05° |
| B | defaults, z_ki 0.4 | R0558, R0565, R0566 | 0.600 (0.597–0.602) | 0.03–0.05° |
Only z_ki differs. Void: R0563 (sim pose watchdog).

Reviewer, second pass (2026-09-25): SUPPORTED. Only z_ki differs; arm B without R0558 (earlier LAND) spans 0.599–0.601.

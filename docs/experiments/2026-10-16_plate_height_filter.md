# Card: low-pass the plate height his reference follows (`attach_z_lowpass`)

Written 16 Oct ~01:20 (overnight loop, plan 2026-10-15 A5/N6). Bars frozen now, before batch 4's N6 runs report
(R1790-R1793 launched at 01:16, results unseen). The switch flew once on the rig with the critic and the twin waived
(RIG-1015-a3, a3_2; Wesley's call on the day). Typed only; default raw.

## Why

RIG-1015-a1: the ring's plate 9 bounced 0.8 cm sd at 1.8 Hz. His reference copied it (corr 0.9-0.99) and QUAD4's
height followed with gain 0.81 and a 0.25 s delay (antiphase at 1.8 Hz), so the tip's bounce against the plate was
1.6 cm, twice the plate's own (`docs/plan_after_rig_2026-10-15.md`).

## Change

`ring_bridge.py` `_lowpass_z`: the present, measured plate height (only, not the predicted path, not x/y) through two
first-order lags at `attach_z_lowpass_hz` (typed `attach_z_lowpass:=0.8` in his launch, rig and twin). Replayed on
a1: the 1.2-2.5 Hz bounce of the target 0.82 -> 0.13 cm; slow tracking lag 0.5 cm rms (max 1.8 cm).

## Evidence so far (exploratory, tree 66a701c / 2ec85cb)

| Condition | Baseline | Filter |
|---|---|---|
| No shake (m1c_floorjoin_hover) | 4/4 attach (R1762, R1764, R1787, R1789) | 4/4 attach (R1763, R1765, R1786, R1788), same timing |
| Ring shaken to 0.85 cm (rig 0.81) at 1.8 Hz (n6_arm_*) | 0/2 (R1782, R1784: contacts lost in the proof, his give-up) | 2/2 (R1783, R1785) |

The twin's QUAD4 landing tip-over after the fold-in (R1502 type) hit both arms (baseline 2/4, filter 3/4 unshaken).

## Bars (frozen 01:20; scored on the n6_arm runs of batch 4 together with R1782-R1785)

1. Shaken (0.85 cm): the filter attaches (weld + fold-in) in at least 3 of 4 runs; the baseline in at most 1 of 4.
2. Unshaken: the filter's MATCH -> weld time within 0.5 s of the baseline's in every run; no abort before the fold-in.
3. No false proof (an `ATTACH: proven` without a weld in the twin's sim catch) in any run.
4. The landing tip-over is scored separately (known twin artefact), not against the filter.

Falsifier: bar 1 or 2 failing.

## Limits

The twin's tether is a rigid rod (the rig's goes slack and its magnet can rest on the plate); the shake is a square-wave
torque, not downwash; QUAD4 in the twin does not wander like the rig's. The claim is: under a rig-sized plate bounce,
holding QUAD4's height still beats following the bounce late, in the twin.

## Critic (16 Oct ~01:50): rewrite the card. N6's 2/2 vs 0/2 is not evidence.

1. **N6's outcome is an artefact, not a better catch.** The twin welds at the instant CONTACT is declared (sim_catch);
   the contact gate is a 0.3 s least-squares slope (about half a 1.8 Hz period), so it fires near bounce extremes
   with tip and plate moving fast relative to each other (R1782 ~0.24 m/s, R1785 ~0.1 m/s); the proof's "drift" is
   then the welded offset minus c0 (the dwell's last 0.1 s mean), against d_ok 6 mm: R1785 passed by 0.6 mm (5.4),
   R1783's first try failed at 5.5, R1782 inconclusive at 9.7. The arm difference is the bounce phase at the gate.
   The registry rows' "contacts lost in the proof" (R1782, R1784) is wrong: the logs say PROOF FAILED
   (inconclusive) with the sim joint attached.
2. Bars 2 and 3 cannot fail (the filter is a no-op on a still plate; the twin welds at every contact); bar 1 counted
   R1782-R1785, already seen when it was frozen, and 2 v 2 more passes it with p ~ 0.56 under no effect.
3. **Rig regression:** his bottom hold's plate-frame tip offset and its "tip sunk below the plate" exit (-0.02 m for
   0.12 s), and his proof lift's base, read the same attachment_position: the filter's lag (0.5 cm rms, 1.8 max) plus
   the bounce can make false exits and phase-dependent proof lifts. Rig-only; no twin run exercises it. Fix: filter
   only the reference (or leave the position raw).
4. **Untested alternative: the twist.** The bridge publishes a raw 50 Hz finite difference of the plate point as the
   attachment twist; his reference extrapolates it over a 3 s horizon and his match gate uses it (1.8 Hz, 1.1 cm ->
   +-0.13 m/s, ~6 cm of reference 0.5 s ahead). His own measured-ring mode publishes a zero twist. One-variable arm:
   raw position, zero twist.
5. Below ~1 Hz the filter makes the tip move more against the plate (QUAD4 gain 0.81, 0.25 s delay: 1.31 vs 1.29 at
   1 Hz, 1.39 vs 0.91 at 0.66 Hz, 1.31 vs 0.71 at 0.5 Hz); N6 shakes only at 1.8 Hz; 1 m cables lower the modes.
6. Spurious contacts on the rig: a still tip over a bouncing plate is where the slope gate fires at extremes.
7. The shake is an external square wave; the twin's QUAD4 follows ~1x, not the rig's 2x; the twin differs from the
   rig in every setting the filter touches (proof_rise_max, contact_dz, bottom hold, catch, require_motion, tether).
8. Minor: the unit test uses 0.6 Hz (0.8 flies); no ramp-lag test; vz becomes a late derivative.
9. Process: trees mixed; bars frozen after half the runs; the next-day draft stacks six changes.

**Response (loop, 01:55):** the N6 result is withdrawn as evidence (registry rows R1782-R1785 keep their numbers; this
card records the correction). Not a default; if it flies again it flies alone as an A/B, and only after finding 3.
Next: a zero-twist switch (finding 4) in the bridge, typed only, and a rev 2 test that scores the mechanism (the
tip's motion against the plate before the first contact, inside-and-slow time) on unseen runs, with a slow-shake
neighbour (finding 5).

## Rev 2 (16 Oct ~02:05; frozen before any of its runs start)

**Question.** Under a rig-sized plate bounce, does QUAD4's tip move less against the plate before the first contact
with (a) the plate-height filter or (b) a zero attachment twist (`attach_twist:=zero`, critic F4: raw position, no
extrapolated bounce), than with the raw bridge? The sim weld is not the outcome (critic F1).

**Arms** (twin, his floor join, the rig's match gate typed, 0.17 N m square-wave torque about world x, 40 s from his
APPROACH): `n6_arm_base`, `n6_arm_zlp08`, `n6_arm_twist0` at 1.8 Hz, 4 runs each, all new (R1782-R1785 and batch 4's
N6 runs are excluded); neighbour at 0.8 Hz (critic F5): `n6s08_base`, `n6s08_zlp08`, `n6s08_twist0`, 1 run each.

**Metric** (from run.csv, the window from his first DESCEND to the first CONTACT, or 10 s if none): the tip's motion
against plate 9's point along the ring normal, `rel = tip_z - plate_z`, band-passed 0.3-3 Hz, its sd (the mechanism);
reported with it: QUAD4's height sd in 1.2-2.5 Hz, the time inside the catch (xy <= 24 mm) and slow (|d rel/dt| <=
0.03 m/s), CONTACTs per bottom second, welds and proof outcomes.

**Bars.**
1. At 1.8 Hz an arm helps if its median rel sd is at least 25 % below the baseline's median and all four of its runs
   are below the baseline's median.
2. At 0.8 Hz an arm's rel sd is no more than 20 % above the baseline's (single runs: reported as a check, not a claim).
3. No arm aborts before the fold-in or a take-back (the landing tip-over is scored apart).

Falsifier for each arm: bar 1 failing. Only an arm passing bars 1-3 goes on to a rig A/B (alone, alternating with
raw), and the filter only after critic finding 3 (the hold and the proof base on the raw plate).

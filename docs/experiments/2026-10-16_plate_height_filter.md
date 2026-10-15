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

## Critic

(below)

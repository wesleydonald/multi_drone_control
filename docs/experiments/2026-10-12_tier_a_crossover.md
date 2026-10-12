# Tier A, the crossover: where the survivors learn of the change (thesis plan P11, freeze 1.5 T2/T3/T4)

Card written 12 Oct 2026 (13:40 from `date`), before any config and before any run. Twin only (the lab PC, when free;
not the laptop before Wednesday's rig day). Predictions and bars are the ones signed on 11 Oct in
`docs/freeze_2026-10-14.md` (1.2 the resolution rule, 1.5 the 1-DOF signs); nothing here re-opens them. Labels per
freeze 1.3: T2, T3 and T4 are **new predictions** (no arm of either pair had flown at the signature).

## 1. Question

Does the pull source the survivors fly on decide how much the ring tilts when one UAV leaves, in opposite directions
for an announced ramped release and for an unannounced one: the planned pull better when the planner knows first, the
measured pull better when it does not?

## 2. Hypothesis

The 1-DOF model (freeze 1.5): a survivor's feedforward pull is F_plan + k (F_meas - F_plan), k = 0 on the planned pull
(`cable_source model`), k = 1 on the measured pull (`cable_source mocap`). The ring tilts by an amount that grows with
the size and the duration of the mismatch between the pull the survivors apply and the pull the ring needs.

- **Announced, ramped (soft v2):** the plan steps to the new shares as the leaver unloads over 3 s; the model is right
  (the twin's plant is the planner's). k = 0 applies the right pull at once; k = 1 applies it late by the measurement lag
  and reports the ring's own rocking back into the thrust (`docs/counter_swing_2026-10-12.md`). **T2: planned <
  measured.**
- **Unannounced:** the planner learns at detection, about 0.3 s after the magnet opens (0.27-0.42 s, 8 Oct twin and
  rig); k = 0 flies the old shares for that long and then steps; k = 1 follows the true pull from the first 80 ms. **T3:
  measured < planned.**
- **T4, the crossover:** (measured - planned) > 0 on the announced ramped pair and < 0 on the unannounced pair.

How much, from what exists (not on the frozen metric and not on this tree, so evidence, not baseline): announced plain,
planned 3.3 deg against measured 8.4 deg (R1128, R1118, 7 Oct tree, peak tilt); the ramp took the planned plain 3.3 to
2.5-2.6 (R1134, R1137). Unannounced measured on the W4 base: Delta-theta 5.74-6.67 (R1459, R1460, R1626). Unannounced
planned has never flown; the model says it sits above the measured arm by the load change held wrong for ~0.3 s plus
the step at the resize, so more than the 1 deg floor of the rule and likely more than twice the arm's range. The ramp's
own effect (about 0.8 deg) is below what the rule resolves; it is not what this card tests.

## 3. The one variable

Two pairs, each one variable, on one base config and one tree. Base: `configs/experiments/w4_da_release_hover.yaml`
(the W4 line: 120 deg layout 30/90/150/270, leaver at 90, hover at 1.0 m, lift 0.15, `drop_and_land.yaml`, detection
0.06 m, departed_land, departed_ground_idle, model integral, the rig thrust map). Four arms, all from that base:

| arm | pull source (tracker `cable_source`) | event | config | status |
|---|---|---|---|---|
| A1 | mocap (measured) | RELEASE (runner magnet OFF, detection on) | `w4_da_release_hover.yaml` as is | the W4 set exists on a25c40d; re-flown here for one tree |
| A2 | model (planned) | RELEASE, detection on | new: A1 + `cable_source: model` | never flown |
| A3 | mocap (measured) | DETACH with `detach_unload_s: 3.0` (soft v2) | new: A1 with the DETACH event + `detach_unload_s: 3.0` | never flown at 120 deg (D4 flew it at 150 deg, 7 Oct tree) |
| A4 | model (planned) | DETACH, soft v2 | new: A3 + `cable_source: model` | R1134/R1137 flew its 7 Oct form (`w4_3915_softdetach_model`) |

- Pair T2 = A4 against A3 (announced soft v2; the one variable is the pull source).
- Pair T3 = A2 against A1 (unannounced; the one variable is the pull source).
- T4 reads the two pairs' signs together. Nothing else moves: `detach_unload_t 0.5`, `detach_unload_wait_s 2.0`,
  `detach_detect_m 0.06`, `cable_elev_deg 60`, the step-out 0.5 m, LAND 20 s after the event as in the base.
- The configs are written after this card is accepted; each one's `description` names this card. The release instant
  shifts between repeats (events at 20.0, 20.5 and 21.0 s after the lift's WAIT_LIFT) so the three runs of an arm meet
  the ring at three phases of its settling, as the plan asks.
- Tree: the week5 candidate at the time of the batch; its SHA goes at the top of the result table and every run's
  registry row. All twelve runs on that SHA (one code tree per claim table).

## 4. Baseline

Each pair is its own comparison; there is no outside baseline on the frozen metric and this tree. What is known:

| arm | nearest existing runs | metric there | value |
|---|---|---|---|
| A1 | R1459, R1460, R1626 (W4 set, a25c40d / c2279cf) | Delta-theta cut at 15 s (freeze) | 6.12, 5.74, 6.67 deg |
| A2 | none | | |
| A3 | none at 120 deg (R1133? D4 at 150 deg, 7 Oct tree: not comparable) | | |
| A4 | R1134, R1137 (7 Oct tree, reference integral, 1/3/5/9) | peak tilt (pre-freeze) | 2.5, 2.6 deg |

The 8 Oct rig pair r302_3 (announced plain, measured) against r0012 (announced plain, planned at 45 deg) differs in
five settings and is not a baseline for T2 (plan 4.2 B).

## 5. Pass / fail numbers (frozen; freeze 1.1 and 1.2)

**Metric:** Delta-theta per run (the tilt vector's largest departure from its 5 s pre-window mean), window from the
event to event + 15 s (Q1, the W4/W5 horizon; every arm holds 20 s). Event times: A1, A2 the runner's RELEASE; A3, A4
the **ramp start**, the first planner tick after DETACH at which the leaver's planned tension falls (freeze 1.1, "soft").
Beside it, reported not scored: the absolute peak, the dip, time back in band (the hold is 20 s, so it is scorable),
sigma_pre and the Q12 flag, the landing lean, the survivors' largest planned step, and the measured-pull swing
amplitude after the arrival (`tools/counter_swing.py`).

**Resolution (freeze 1.2), per pair, arms of n = 3:** resolved only if (1) sign(mean B - mean A) is the frozen sign,
(2) the ranges do not overlap, (3) |mean B - mean A| >= max(1 deg, 2 x the larger range). Otherwise unresolved.

| | confirmed | falsified | unresolved |
|---|---|---|---|
| T2 (A4 vs A3) | resolved with planned < measured | resolved with measured < planned | neither |
| T3 (A2 vs A1) | resolved with measured < planned | resolved with planned < measured | neither |
| T4 (crossover) | T2 and T3 both confirmed | T2 or T3 falsified | any other outcome |

An unresolved pair is reported as unresolved with its six values; it does not become a claim and is not re-flown to
make it one (the plan's "levers stay twin-unresolved" is the stated risk). A run with a magnet letting go, a drop, a
fleet abort or a solver fault is reported and not scored; the arm is re-flown once for it.

**What falsifies the mechanism, not only the sign:** if A2 (planned, unannounced) is **not** above A1 by the rule, the
"planner learns 0.3 s late" mismatch does not drive the tilt in the twin, and the plan's C2-B sentence ("the measured
pull wins when the change is unannounced") loses its twin support. If A3 is **not** above A4, the measurement lag is not
what costs the measured pull on an announced change.

## 6. Repeats

3 per arm (the plan P11's x3; Wesley's plan), 12 runs. Order: A1, A2, A3, A4, then the second and third of each in the
same order, so a tree or lab fault part-way leaves one complete 2x2 at n = 1 or n = 2.

## 7. Cost

12 Gazebo runs at about 4 min each: about 50 min on one slot; on the lab PC with up to 6 slots about 10 min of wall
time, only when nobody else is on it (`who`, load, containers first). Not on the laptop before Wednesday. Plus
`tools/prebuild_planner.py` once on the tree.

## 8. New code?

**Flight code: no.** Every arm is a config of parameters that exist (`cable_source`, `detach_unload_s`, the runner's
RELEASE/DETACH events).

**Analysis code: yes, small, two items; the critic's challenge is below.**
1. The soft-detach event time. `tools/release_bands.py` finds RELEASE and DETACH only; the freeze defines the soft
   event as the ramp start. The planner prints `[dissipative] UNLOAD <drone> over 3.0 s before release` at that tick;
   the runner (and `tools/rig_to_run.py`) would log it as an `UNLOAD` event and `release_bands.Run` would take `UNLOAD`
   over `DETACH` when both exist. About 15 lines, no flight behaviour. If it fails to land in time, A3 and A4 are
   scored from DETACH (the command, which precedes the ramp start by one tick) and the card says so.
2. Nothing else: the metrics, the bands and the swing numbers come from `release_bands.py`, `release_transient.py` and
   `counter_swing.py` as they are.
Removal plan: the UNLOAD event is a log line and a lookup; if the card is dropped, the two edits are reverted in one
commit.

**What the result cannot say.** The twin has no magnet hold, no slack tether, no mocap noise and none of the rig's
1.4-1.7 Hz rocking (`docs/counter_swing_2026-10-12.md`); the measured pull's costs on the rig (the swing fed forward)
are not in these runs. A twin crossover is a statement about the mechanism in the planner's own plant; the rig rows of
the plan's P14 (28-29 Oct) decide whether it carries.

## Critic

(The critic agent's challenge is pasted here before Wesley sees the card.)

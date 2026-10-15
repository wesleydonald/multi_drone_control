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

## Rev 1 result (batch 4, R1790-R1793; superseded, recorded)

Rev 1 bar 1 (welds): the filter welded in 2 of 4 (R1783, R1785 yes; R1791, R1793 no), the baseline in 0 of 4: FAIL
as frozen (needs 3 of 4), the outcome the critic predicted for a weld count. The mechanism metric favoured the filter
in all four pairs (tip vs plate 0.88-0.97 vs 1.37-1.41 cm; QUAD4 1.8 Hz height 0.12 vs 0.61 cm; inside and slow 20-22
vs 9 %), but these runs were seen before rev 2 and are excluded from it.

## Rev 2 result (batch 5, R1800-R1814; scored as frozen)

| Bar | Baseline | Filter (0.8 Hz low-pass) | Zero twist |
|---|---|---|---|
| 1: tip vs plate sd at 1.8 Hz, before the first contact (4 runs) | 1.37, 1.40, 1.39, 1.43 cm (median 1.40) | 0.90, 0.88, 0.95, 0.92 (median 0.91, -35 %): PASS | 0.93, 1.01, 1.05, 1.03 (median 1.02, -27 %): PASS |
| QUAD4 height 1.2-2.5 Hz (reported) | 0.59-0.63 cm | 0.12-0.14 | 0.08-0.10 |
| Inside the catch and slow (reported) | 9-10 % | 17-20 % | 15-18 % |
| 2: 0.8 Hz neighbour (1 run each) | 0.48 cm | 0.39: PASS | 3.94: FAIL |
| 3: no abort before the fold-in / take-back | none | none: PASS | none: PASS |

(R1800 hit the lab's 30 min wall cap after its LAND; its pre-contact window is complete. Welds, not a bar: baseline
0/4, filter 3/4, zero twist 3/4.)

**Bar 2 is confounded.** The 0.8 Hz shake at the same torque moved plate 9 by 3.0 cm sd (0.5-1.2 Hz; ring tilt up to
13 deg), four times the rig's bounce. The baseline and the filter stayed in MATCH for the whole 40 s shake (the plate's
sideways speed 0.094 m/s kept his match gate shut, the a3_2 symptom) and descended only after it, so their windows saw
a still ring; zero twist descended 1 s after MATCH and sat 26 s at the bottom of a ring still moving 3 cm, without a
contact. As frozen: the filter passes bars 1-3; zero twist fails bar 2.

## Rev 3 (16 Oct ~03:30, frozen before its runs): the low-frequency check at the rig's amplitude

0.8 Hz square-wave torque scaled to ~0.8 cm of plate motion (0.045 N m; the 0.17 N m run gave 3.0 cm), 40 s from his
APPROACH, rig match gate typed: `n6s08b_base`, `n6s08b_zlp08`, `n6s08b_twist0`, 2 runs each. Reported per run: plate z
sd 0.5-1.2 Hz over the shake; time from MATCH to the first descent; QUAD4 height against the plate in his MATCH (rel sd
0.5-1.2 Hz: how well QUAD4 follows a slow bounce while it waits); the weld.
Bar: an arm's MATCH-phase rel sd at 0.8 Hz is no more than 20 % above the baseline's (the mean of its two runs);
the arm that passes rev 2 bar 1 and this bar is the one to fly as a rig A/B.

## Rev 3 result (batch 6, R1818-R1823; 04:00)

As frozen: **inconclusive.** At the rig's amplitude (plate 0.77 cm sd at 0.8 Hz) every arm left MATCH 1.0 s after
entering it (the slow plate no longer blocks his match gate), so the frozen MATCH-phase metric had ~1 s of data and
is not computable. Reported instead (not a frozen bar), the rev 2 metric (tip vs plate, his first descent to the first
contact or 10 s):

| Arm | 0.8 Hz runs | tip vs plate (sd) | vs baseline | contact / weld |
|---|---|---|---|---|
| Baseline | R1818, R1821 | 0.96, 0.92 cm | - | none (his give-up, both) |
| Filter | R1819, R1822 | 1.16, 1.17 cm | +24 % | none (his give-up, both) |
| Zero twist | R1820, R1823 | 0.88, 0.88 cm | -6 % | both |

Together with rev 2 (1.8 Hz): the filter -35 % at 1.8 Hz but worse at 0.8 Hz (consistent with critic finding 5, on an
after-the-fact metric); zero twist -27 % at 1.8 Hz and no different from raw at 0.8 Hz; it removed the match-gate
hold-up in rev 2's 3 cm neighbour (one run). See the reviewer below for what may be claimed.

## Reviewer (16 Oct ~04:15): SUPPORTED WITH CORRECTIONS (weak at 0.8 Hz)

- Every number in the rev 2 and rev 3 tables reproduces; the bars were frozen before their runs (rev 2 e9ff80a
  01:31 < R1800 01:48; rev 3 83a2c75 03:21:31 < R1818 03:22:00) on one code tree, and applied as frozen. Exact
  reductions at 1.8 Hz: -34.8 % (filter), -26.9 % (zero twist).
- **1.8 Hz holds on a common window** (the first 3.6 s after DESCEND, before any contact in all 12 runs): baseline median
  1.43 cm, filter 0.88 (-39 %), zero twist 1.01 (-29 %), all below the baseline median; in 1.2-2.5 Hz only, -40 % and
  -36 %. Window length and detrending do not explain it.
- **0.8 Hz (rev 3) moves with the window:** filter +24 % (card), +32 % (common 3.6 s), +17 % (matched 4.8 s): worse in
  every window; zero twist -6 %, 0 %, -7 %: no different from raw. Each pair of twin runs samples one phase (R1820 and
  R1823 contact at the same instant): effectively one run per arm.
- Corrections: "inside the catch" adds nothing (xy <= 24 mm in every sample: the row is the slow fraction during the
  descent); R1812/R1813 entered MATCH 6.8/6.5 s into the shake and stayed 36.4/36.7 s, to 3.2 s after it ended; bar 3
  "none" is true as worded, but FLEET ABORT "drone 4 disarmed unexpectedly" fired after LAND in 9 of 10 welded runs.
- Each comparison changes exactly the intended variable (configs and parameter read-backs).
- **What may be said:** in the twin, under the rig's 1.8 Hz plate bounce, the filter and zero twist each cut the tip's
  motion against the plate before the first contact by about a third (4 runs per arm, all below the baseline, bars
  frozen before the runs); as frozen, the filter passed rev 2 and zero twist failed its confounded bar 2. Labelled
  after the fact and from two near-identical runs per arm: at 0.8 Hz the filter makes it worse, zero twist no
  different. **What may not be said:** that zero twist improves the 0.8 Hz case, that finding 5 was confirmed by a
  frozen test, that either switch improves attaching (the weld counts are the twin's catch artefact), "no aborts",
  that the twin's repeats measure spread, or anything about the rig without the slack-tether and downwash caveats.
  Whether to A/B zero twist on the rig is Wesley's decision.

## Critic on zero twist as a rig A/B (16 Oct ~04:20): fix first. Response (04:35)

1. *The free rig falsifier was never run* -> run (`tools/match_gate_replay.py --zero-twist`, his exact gate on the
   rig bags): zero target velocity opens 3 of the 4 long-blocked rig MATCH windows: a3_2 (42 s, raw: at best one
   marginal pass) first pass at 4.9 s, 8 passes; t6f2_4's third (16.7 s, raw: never) at 1.0 s; t6g_2's second (27.8 s,
   raw: never) at 20.7 s; not t6h5_3's second (16.2 s: the xy half fails, both true 11 -> 26 %). The twin never
   reproduced the block at the rig's amplitude (no mocap jitter there).
2. With a zero twist the relative-velocity guards (match gate, descent abort, the hold's gross exit) become QUAD4-speed
   guards; ring motion is then caught only by the xy limits (R1814 sat 26 s over a ring moving 3 cm). Open; the
   better variable may be a low-passed twist (keeps slow ring motion). Built 04:40, typed only (`attach_twist:=lowpass`,
   `attach_twist_hz`, 0.5 Hz, two first-order lags, hover only): see the end of the card.
3. Moving ring: fixed (efb9b5d): never zero while the ring's trajectory runs; unit test. **Bug (found 05:10 from
   R1832):** the guard read the planner's `running`, which is also true in a lifted hover, so from efb9b5d on
   `attach_twist` did nothing in the twin or on the rig. Fixed: moving = running, a kind other than hover and a speed
   above zero; unit test for the lifted hover. R1824-R1826 flew before the guard (valid); R1828, R1830 and R1831-R1834
   flew raw.
4. The slow end (ring drift ~0.25 Hz, the 1 m hardware's lower modes) is untested: open (twin follow-ups).
5. The A/B had no frozen bars: draft below, for Wesley to freeze before the lab day.
6. Mislabelling: fixed (efb9b5d): `choices` on the argument, the bridge's start-up line names both switches, the rig
   bags record /fake_attachment_point/twist.
7. It changes what his gates measure: logged in CHANGES_FROM_TEJEN.md; Tejen should agree before it flies.

### Draft rig A/B (not frozen; Wesley freezes before the lab day)

- Go/no-go, on the new hardware's first two attach flights with the raw bridge: fly the A/B only if (a) plate 9 still
  bounces >= 0.5 cm sd in 1.2-2.5 Hz with QUAD4's height following it (gain >= 0.5), or (b) a MATCH window over 10 s
  where the speed half of his gate fails more often than the xy half (replay).
- Arms alternate raw / zero twist, 3 flights each, the same packs order.
- Primary (resolvable in 3 v 3): QUAD4's height response to the plate in 1.2-2.5 Hz per DESCEND / bottom window
  (gain), and MATCH -> first descent time; secondary: tip vs plate sd before the first contact, contacts, proofs.
- Stop rules as the attach card.

## Zero twist under a slow sideways drift (critic F4; 04:40, exploratory, single runs)

| Drift (0.25 Hz square wave along x) | Raw | Zero twist |
|---|---|---|
| 0.4 N (plate range 15 cm) R1827/R1828 | MATCH 35 s; tip vs plate xy median 6.5, p90 9.8 cm | MATCH 35 s; 6.1 / 10.0 cm |
| 0.05 N (3 cm, rig-like) R1829/R1830 | MATCH 1.0 s; 0.9 / 1.6 cm | MATCH 1.0 s; 1.1 / 2.2 cm |

~~Falsifier (1.5x the raw error) not met; at the rig-like drift zero twist trails it by 1.2x (median) / 1.4x (p90).~~
**Void (05:10):** R1828 and R1830 flew raw (the guard bug in item 3), so both columns are raw runs and the 1.2x / 1.4x
is run-to-run spread (a third raw run, R1831: 1.0 / 1.6 cm). Re-flown on the fixed guard: below.

## Re-fly on the fixed guard, and a low-passed twist (critic F2) (05:10-05:50, exploratory)

A low-passed twist (`attach_twist:=lowpass`, two first-order lags at 0.5 Hz, hover only) was built at 04:40 to keep a
slow drift that zero twist drops. Code at e85bb17 + 9c04ee3; **removed** after these runs (it did not beat zero twist).

| Arm | 1.8 Hz bounce: tip vs plate z sd / QUAD4 z (1.2-2.5 Hz) | Rig-like drift (0.05 N, 0.25 Hz, ~3 cm): tip vs plate xy median / p90 | Still ring |
|---|---|---|---|
| Raw | 1.37-1.43 / 0.59-0.63 cm (R1800, R1803, R1806, R1809, R1832) | 0.9 / 1.6, 1.1 / 2.2, 1.0 / 1.6, 1.0 / 1.8 cm (R1829, R1830, R1831, R1841) | baseline |
| Zero twist | 0.93-1.05 / 0.08-0.10 cm (R1802, R1805, R1808, R1811) | 1.3 / 2.1, 1.2 / 1.7 cm (R1837, R1840) | no regression (R1824-R1826, R1838) |
| Low-pass 0.5 Hz | 1.04 / 0.12 cm (R1835; R1842 below) | 1.6 / 2.4, 1.5 / 2.3 cm (R1836, R1839) | (R1834 flew raw) |

Every drift run descended after 1.0-1.1 s of MATCH and welded. Reading: at 1.8 Hz the low-pass acts as zero twist; under
a slow drift its 0.5 Hz lag costs more than no feed-forward at all (median +0.5 cm on raw, zero twist +0.2 cm). Zero
twist stays the candidate for the rig A/B; its small drift cost (median 1.2-1.3 vs 0.9-1.1 cm, two runs) is well
inside a 45 mm catch.

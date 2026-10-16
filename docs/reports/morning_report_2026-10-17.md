# Morning report, Sat 17 Oct 2026

Evening queue on the laptop, 16 Oct 20:25 to 17 Oct 02:33: 78 runs, R2006-R2083, tree c3bf628. The lab PC was busy with
another user's 12-sim batch (checked 20:22), so it was not used.

## Rig finding first: a landing can disarm the fleet 0.4 m up

| arm (twin) | runs | drones' height at disarm (m, rest 0.10) | end state |
|---|---|---|---|
| thrust map off on drone 1 (offset 0.155 against its map's 0.185), target-shift integral | R2032 / R2044 | 0.47-0.53 / 0.46-0.53 | all three on their side |
| the same, model integral | R2033 / R2045 | 0.10-0.26 / 0.48-0.53 | drone 1 at 25 deg / all three on their side |
| every other landing tonight and in the carry claim | 80 | at most 0.15 | upright |

**What happens:**
- After the ring is down, the planner decides "on the floor" when the highest drone stops descending (the stall test).
- A drone that is stronger than its thrust map sinks too slowly at hover thrust, and that reads as a stall.
- LAND then disarms everyone in the air.

**Why it matters on the rig:** a pack or prop change that moves one quad's thrust map could do the same, with a 0.4 m
fall. A fix changes a gate that lands the fleet, so it needs a card and the critic. Nothing is built yet.

## Appendix re-fly (54 runs, bars frozen 16 Oct 09:05), reviewer: weak

| claim | result | verdict |
|---|---|---|
| relaxed tracker box raises the hover (C-X1) | +13.1 cm (was +4.8) | confirmed, larger |
| pinning cuts the orbit error (C-X2) | 0.74 against 3.87 cm (ratio 0.19) | confirmed |
| the integral removes a constant push (C-D1) | steady at most 1 cm | confirmed |
| the vertical push shows without the integral (C-D2) | 0.0 cm (was 8.4); horizontal 4.8 cm | **falsified (vertical)** |
| the integral learns the push (C-D3) | 0.10 N of 0.6 N vertical; 0.33 of 0.4 N horizontal | **falsified (vertical)** |
| model beats target shift on the thrust-map error (C-D5) | both ~0 cm | **falsified** |
| gain 0.8 / 5 s gate lower peak and overshoot (C-G1, C-G2) | ranges overlap | unresolved |
| robustness rows (C-R1..C-R4) | integral rows within 1 cm; all landed | confirmed |
| the default landing never tips (C-L1) | 3 or 4 of 82 tipped (the rig finding above) | **not supported** |

**Why the vertical claims fail** (reviewer: supported): the trackers measure the cable pull and absorb a vertical push
or a mass error within ~1 s, with no integral. The vertical model integral then has nothing left to do: on a 2 cm
radius error it winds up to 0.76 N with no effect on the height. Detail and the must-not-say list are in
`docs/analysis/appendix_refly_2026-10-17.md`.

## Other results

| item | runs | result |
|---|---|---|
| N2 bands on the 1 m cables: re-fly of the 10 lab voids and of 7 starved lab "passes" (correction rows R1921c-R1936c) | R2006-R2015, R2077-R2083 | every cell now clean: RMSE 11-24 mm, tilt 1.7-5.3 deg. Spin at 0.3 m/s 6.16 deg, over the 6 deg bar, as at 0.55 m (R1875) |
| corner cutting: figure-8 repeats in one session | base R2016 / R2019 / R2022; lead 0.05 R2017 / R2020; lead 0.075 R2018 / R2021 | cut at the bends +26 / +25 / +21 mm; +5 / +5; -9 / -2. The "figure-8 needs 0.03 s less lead" point is not supported: one lead (0.075) does for all three shapes |

## Decisions for Wesley

1. **The false touchdown (rig).** (Recommended) Card and critic for a stricter touchdown test: stall *and* each
   drone near its own rest height, or its thrust well under hover. Behind a switch (off by default); the mapneg arm
   and a floor-start LAND are the twin test. Alternative: leave it and watch thrust maps on the rig.
2. **The thesis's vertical claims.** (Recommended) First fly one pair with the modelled pull (`cable_source model`)
   on the push and on the +20 % mass arm: 4 runs, ~20 min. Then rewrite the text around the measured pull. Alternative:
   rewrite now, with the cause marked as likely.
3. **Corner cutting, still open from yesterday.** (Recommended) A measured lead (the clock lead from the measured plan
   age) with `ref_time_shift`, behind a switch, card and critic first.
4. **The N2 band rule**, now that every cell is clean.
5. **Merge `day-1016`.**

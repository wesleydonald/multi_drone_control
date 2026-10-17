# Card: the LAND touchdown fired 0.5 m up (re-arm at ring touchdown, plus Wesley's height gate)

Written Sat 17 Oct 2026 (Wesley's decision ~10:30-10:40, docs/decisions.md 17 Oct (1)). **Rev 2 after the critic
(below).** Bars frozen before any run.

## Why

In the appendix re-fly's thrust-map arm (drone 1 flies a sim thrust offset of 0.155 against its map's 0.185: stronger
than the map), LAND disarmed the fleet with the drones in the air:

| run | drones' height at LANDED (m; rest 0.10) | end |
|---|---|---|
| R2032 | 0.47-0.53 | all three on their side |
| R2044 | 0.46-0.53 | all three on their side |
| R2045 | 0.48-0.53 | all three on their side |
| R2033 | 0.10-0.26 | drone 1 at 25 deg |
| every other landing (80: app_ and the carry claim) | at most 0.15 | upright |

**Mechanism** (R2032 trace; the critic's reading):
- `TouchdownDetector` is armed once at LAND and never re-armed. Its grace and minimum-descent checks are used up
  during the loaded descent.
- When the ring touches down (63.8 s), the drones switch to `_land_hold_refs`: the ring's weight comes off and the
  feedforward blends to hover over `LAND_BLEND_S` (1.0 s).
- In that second the strong drone nearly held its height (0.549 -> 0.542 m). That read as a stall, `_touched_down`
  latched at ~0.5 m, the reference froze, and LANDED went out at 64.7 s, 0.9 s after touchdown.
- The throttles then were 0.46-0.47: free hover, not pressing on anything.

**Rig relevance:** a pack or prop change that moves one quad's thrust map could do the same, ending in a ~0.4 m
fall.

## The change: two arms, each behind its own switch (default off: today's behaviour bit for bit)

**A. `land_rearm_touchdown`:** `_touchdown.reset()` when `_land_hold_refs` first anchors (ring touchdown). The grace
(1.5 s) and the minimum descent (0.05 m) then apply again from there, so the hover-blend second cannot read as a
stall. One line, and it may fix R2032 by itself (critic should-fix 1).

**B. `land_height_gate` (Wesley's rule, on top of A):** the survivors' stall counts as touchdown only if every
survivor is at most `land_gate_dz` (0.30 m) above its rest height. The test is one-sided: z - rest <= 0.30.
- Rest height is the TAKEOFF record our planner already keeps for departed drones (`_rest_z`, cleared at ARM,
  recorded at TAKEOFF, keyed by physical drone id).
- A drone with no record (air start, partner runs, a newcomer first seen in the air) is not gated.
- **Fallback time:** the existing exit `lift_progress <= -LAND_MAX_DROP` (planner_node, the LAND branch). It fires
  once the reference has descended 1.5 m below the handover height: about 7.5 s after ring touchdown from a 1 m
  hover. It stays ungated, so the fleet never hangs armed. No new timer, and no throttle test.
- **Logged:** which exit fired (gated stall / max drop / departed wait), with every drone's height and z - rest at
  that moment.

**Also in A and B:** the stall test reads only poses fresher than 0.2 s, and a stale pose resets the stall count. A
frozen pose reads as a stall today (critic should-fix 2).

**Code:**
- `TouchdownDetector.update(zs, rest=None)`: the default keeps every existing call, the departed drones' detectors
  included, unchanged;
- `planner_node` LAND branch: reset, gate, exit log;
- the parameters go through `bringup` (`real_mode.PLANNER_OPTIONAL`, `common.yaml`);
- no new subscriptions.

## Tests (unit, before any twin run)

- off: every existing test_touchdown case unchanged. A replay of R2032's heights fires at the recorded tick, and a
  replay of a departed-drone case is unchanged;
- A, R2032's heights: no fire during the blend second;
- A, R2023's heights: fires within 0.5 s of today's tick;
- B, a stall at 0.5 m (rest 0.10): no fire, then the max-drop exit fires;
- B, a drone with no rest record: not gated;
- B, a drone resting above its rest (raised mat) and stalled: one-sided gate, so it is gated only if more than 0.30
  above. Covered by the max-drop exit;
- a stale pose: stall count reset.

## Twin runs (laptop, tree = the commit with the switches)

| arm | config | n | switch |
|---|---|---|---|
| T1 thrust map strong, target shift (the fault) | `app_mapneg_ref` | 2 + 2 | A; A+B |
| T2 thrust map strong, model integral | `app_mapneg_model` | 2 + 2 | A; A+B |
| T3 normal hover landing | `app_x0_hold_eps0` | 2 | A+B |
| T4 floor-start lift and LAND on a path | `traj3_orbit_v125` | 2 | A+B |
| T5 the rig's LAND path (`land_unwind false`, real.yaml since 7 Oct) | `app_x0_hold_eps0` + land_unwind false | 2 | A+B |
| neighbour: thrust map weak (sim offset 0.215 on drone 1) | new `app_mappos_ref` | 2 | A+B |
| neighbour: two survivors after a detach | a Tier A detach config | 2 | A+B |
| neighbour: LAND in the creep, and LAND during the lift | the existing land_in_creep configs | 1 each | A+B |

Base for T1-T5: their runs on c3bf628 (R2032/R2044, R2033/R2045, R2023/R2035, the carry claim's orbit v0.125, and a
new pair with land_unwind false and the switches off).

**Scoring:** `tools/land_touchdown.py` must read past LANDED until the drones stop (it stops at LANDED today, and
missed R2032). That change is made and tested before scoring.

## Bars (frozen now, rev 2)

| # | bar | pass | fail |
|---|---|---|---|
| H1 | the fault is gone | T1, T2 with A+B: every drone at most 0.30 m above its rest at LANDED, every run; post-LANDED drone tilt at most the worst of the 80 normal landings + 5 deg; LANDED from the gated stall | any drone higher, any larger tilt, or LANDED from the max-drop exit |
| H1a | A alone (reported, decides whether B is needed) | the same on T1, T2 with A only | (reported, not a fail of the card) |
| H2 | normal landings unchanged | T3, T4, T5: LANDED within 1.0 s of the base's time after ring touchdown; drone heights at LANDED within 0.05 m of the base | outside either |
| H3 | the fleet never hangs armed | every arm: LANDED reached and every drone disarmed within 10 s of ring touchdown | not reached, or later |
| H4 | no new faults | no FLEET ABORT, ENVELOPE FAULT, DROP AND LAND or UNANNOUNCED DETACH the base did not have | any new one |

**Falsifier:** H1 fails with A+B (the strong drone does not come down before the max-drop exit), or H2/H3 fail.

## Rig

Only after the twin, a second look by the critic at the code, and Wesley's word. On the rig it goes in through a
`configs/rig/` params file. QUAD4's rest comes from the TAKEOFF record: `attach_pos` when it is reserved, otherwise
its pose.

## Critic

Critic, 17 Oct ~10:50. Verdict: **rewrite the card.** Rev 2 above takes in every must-fix.
- must-fix 1-3: the throttle fallback and the 15 s backstop are dropped. The existing max-drop exit is the fallback
  time.
- must-fix 4: T5 replaced.
- must-fix 5-6: TAKEOFF record, one-sided gate.
- must-fix 7: H1 tilt bar, exit check, scoring past LANDED.
- must-fix 8: the timers are now one.
- should-fix 1: arm A. Should-fix 2: stale poses. Should-fix 3: default arg. Should-fix 4: physical id.
  Should-fix 5: T5. Should-fix 6: the neighbours above.
- **Not taken:** the combined-M1 LAND with QUAD4 folded in. It waits for the join tree after the rig day.

The critic's text, as written:

1. **Tried before?**
   - Same symptom, different cause: R0852 disarmed the fleet 0.58 m up because the LAND reference paused
     (reference.md §10). That was fixed by keeping the descent going.
   - The mechanism already exists twice:
     - `dissipative_node` departed drones: `_rest_z` (TAKEOFF record) plus "stalled and <= rest + 0.10" and a 6 s
       press limit;
     - the tracker's `ground_idle` (`_kt_spawn_z`).
   - The card would add a third rest height with different rules.
2. **Simplest explanation:** `TouchdownDetector` is never re-armed at ring touchdown. The hover-blend second reads as
   a stall, and LANDED came 0.9-1.3 s after touchdown. The gate hides this instead of fixing it, and does not fix
   R2033 (drone 1 disarmed 0.26 m up, inside rest + 0.30, ended at 25 deg).
3. **Falsifier right; H1 too weak:**
   - the 45 deg limit passes R2033;
   - `land_touchdown.py` stops at LANDED;
   - it does not record which exit fired.
4. **What it breaks:** it is a gate that can hold the fleet armed. Changing `update`'s signature changes the
   departed detectors, and the throttle fallback needs plumbing that does not exist.

Must-fix:
1. The planner reads ELRSCommand and the thrust map only when `dist_est` is on, which it is not by default.
2. Drop the throttle fallback and the backstop. `lift_progress <= -LAND_MAX_DROP` already fires ~7.5 s after
   touchdown. Keep it ungated and log the exit.
3. The throttle test fails both ways:
   - on a raised surface the reference stops at z = 0, so the press is small (R1252, r305: hover 0.12-0.22 m above);
   - on the floor `ground_idle` already idles.
4. T5 (R1844-R1846) never runs this code: `_drop_tick` lands it, at 30 s.
5. Rest = the TAKEOFF record, not "lowest seen while disarmed" (glitches, swaps, unknown disarm state).
6. One-sided gate. Two-sided hangs on an air start, a raised mat, or a newcomer.
7. H1: tilt bar from the normal landings, an exit check, and score past LANDED.
8. H3's 10 s against a 15 s backstop, plus the 15 s departed wait (up to 30 s). Name each timer's clock.

Should-fix:
1. Re-arm the detector at touchdown; fly it alone.
2. Fresh poses only; a stale pose resets the stall.
3. Bit-identical off: default argument, no new subscriptions.
4. Physical ids.
5. A land_unwind false arm.
6. Neighbours: the weak map, two survivors n = 2, LAND in creep and in lift, combined M1.
7. Use the filtered velocity if a dz/dt test survives.

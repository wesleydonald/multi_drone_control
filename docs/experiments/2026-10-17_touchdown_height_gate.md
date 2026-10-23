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

## Results (17 Oct, laptop, tree b7ab074; R2088-R2108; scored with tools/land_touchdown.py past LANDED)

Bars as frozen (rev 2). H1 tilt bar: the worst post-LANDED drone tilt over the 80 normal landings is 7.2 deg (R2049),
so the bar is 12.2 deg.

| arm | runs | LANDED - ring down (s) | highest drone above rest at LANDED (cm) | drone tilt after LANDED, max / end (deg) | exit |
|---|---|---|---|---|---|
| T1 base (off) | R2032 / R2044 | 1.03 / 1.09 | 43.8 / 43.0 | 127 / 123, end 90 / 90 | (stall, not logged) |
| T1 A | R2088 / R2099 | 4.38 / 4.44 | 15.8 / 15.8 | 25.6 / 26.3 (end the same) | stall |
| T1 A+B | R2089 / R2100 | 4.39 / 4.37 | 15.8 / 15.8 | 29.9 / 27.7 | stall |
| T2 base (off) | R2033 / R2045 | 4.46 / 1.02 | 15.7 / 43.5 | 25.4 / 125 | (stall) |
| T2 A | R2090 / R2101 | 4.27 / 4.41 | 15.8 / 15.8 | 25.2 / 27.8 | stall |
| T2 A+B | R2091 / R2102 | 4.45 / 4.40 | 15.7 / 15.8 | 23.6 / 24.5 | stall |
| T3 base (off) | R2023 / R2035 | 4.40 / 4.44 | 4.7 / 4.6 | 1.9 / 1.3 | |
| T3 A+B | R2092 / R2103 | 4.36 / 4.35 | 4.6 / 4.7 | 2.7 / 2.3 | stall |
| T4 base (off, same session) | R2109 / R2113 | 4.29 / 4.35 | 4.8 / 4.7 | 2.5 / 2.3 | |
| T4 A+B | R2093 / R2104 | 4.28 / 4.31 | 4.9 / 4.8 | 1.9 / 2.5 | stall |
| T5 base (off, land_unwind false) | R2094 / R2105 | **1.08** / 5.33 | **42.0** / 0.1 | **142** / 1.5 | |
| T5 A+B | R2095 / R2106 | 4.89 / 4.54 | 0.9 / 2.9 | 10.0 / 11.2 (end 10.0 / 10.6) | stall |
| weak map (A+B) | R2096 / R2107 | 4.45 / 4.33 | 4.7 / 4.7 | 0.9 / 1.9 | stall |
| two survivors after a detach: base / A+B | R1706, R1887 / R2097, R2108 | 4.35, 4.49 / 4.49, 4.36 | 4.8, 4.3 / 4.3, 4.8 | 1.8, 1.9 / 1.7, 2.3 | stall |
| LAND in the creep: base / A+B | R1414, R1431 / R2098 | 3.41, 3.47 / 3.45 | 4.7, 4.7 / 4.7 | 2.6, 1.4 / 0.8 | stall |

R2098: the sim clock froze at 11.5 s, after LANDED (10.95 s); the landing is scored. No arm has a fault its base did
not have (the detach arm's UNANNOUNCED DETACH is the scenario's, in R1706/R1887 too).

**Verdicts against the frozen bars:**
- **H1 FAILS** (A+B on T1, T2): height 15.7-15.8 cm, within the 30 cm gate; exit by the gated stall; but drone tilt
  after LANDED 23.6-29.9 deg against the 12.2 bar.
- **H1a (A alone):** the same as A+B to the millimetre. The gate never acted (15.8 < 30), so B adds nothing on these
  arms.
- **H2 passes** on T3 and T4. On T5, within 1.0 s and 5 cm of R2105, but the end tilt is 10-11 deg against 1.5.
- **H3 passes**: LANDED 4.3-4.9 s after ring touchdown, everywhere.
- **H4 passes.**

**Why the strong drone still stops 16 cm up** (R2088, drone 1's tracker log):
- After ring touchdown the tracker is `grounded` and its cable feedforward is off (`ff_active` 0). The measured pull,
  which had absorbed the thrust-map error in the carry, is gone.
- The reference goes to z = 0, but drone 1 holds at 0.258 m with throttle 0.427 (its true hover) and u2 0.22, well
  above the OCP's 0.05 floor. This reads as a steady-state error: a 0.26 m error balancing the drone's ~1 m/s^2
  thrust surplus (offset 0.155 against 0.185, gain 35.2). Whether any integral is live while grounded is not yet
  checked in the tracker code.
- The stall the detector sees is real, so re-arming it cannot help; Wesley's 30 cm gate lets the disarm through.
- The normal drone 0 does the same on a small scale: 4.7 cm above rest at LANDED in every normal landing.
- The 25 deg end tilt is the drone lying on its rigid rod after the 16 cm drop. The rig's tethers go slack, so the
  rig would see the drop but maybe not the tilt.

**New, rig-relevant: R2094.** With the switches off, normal thrust maps and the rig's LAND path (`land_unwind`
false), LAND disarmed the fleet 0.42 m up, 1.08 s after ring touchdown, and the drones ended at 142 deg. This is the
blend-second stall with no thrust error at all. R2105 (the same config) landed normally, so it happens 1 in 2 here. A
fixes it in both runs (R2095, R2106). Open: why T5 with A+B ends one drone at 10-11 deg (R2105, off: 1.5).

## Reviewer (17 Oct ~14:40): weak

- 1 [weak]: the strong-map part holds (3 of 4 bases fired at 43-44 cm, 0 of 8 with A). The rig-path part rests on
  one event: with the switches off the 0.42 m fire is 1 of 2 (R2094 / R2105, same params), so two clean A runs
  happen 25 % of the time by chance. Its support is the mechanism: the +1.08 s fire falls inside the 1.5 s grace A
  restarts. A alone was never flown on that path.
- 2 [supported]: 15.7-15.8 cm and 23.6-29.9 deg in all 8; B blocked nothing.
- 3 [weak]: H3/H4 hold in all 21. The detach and creep bases are older trees; the creep has n = 1; "LAND during the
  lift" was not flown; T5 has one normal base.
- 4: the 10-11 deg on the rig path is most likely A: LANDED comes 0.4-0.8 s earlier and disarms drone 0 2.9-3.9 cm
  up while it is still rocking. It is above the worst normal landing (7.2 deg), 2 of 2: it matters for the default.
- Tree: the runs are on b7ab074; A is unchanged at 3edeb04, but the rule asks for a re-fly on the final tree.

Re-fly queued (17 Oct afternoon batch): the rig path off / A ABAB plus one more A (2 + 3), A alone on hover and the
creep, LAND in the lift with its base. A stays off by default until the reviewer supports it.

## Re-fly (17 Oct 14:45-17:37, laptop, final tree 0a77fa1; R2133-R2137, R2164-R2167)

Scored with `tools/land_touchdown.py` and the per-drone floor scorer of the land card (registry rows hold each value).

| arm | runs | LANDED - ring down (s) | highest drone at LANDED (cm) | tilt at the end (deg) | false touchdown |
|---|---|---|---|---|---|
| rig LAND path, switches off | R2094, R2105, R2133, R2135 | 1.08 / 5.33 / 5.05 / 1.08 | 42.0 / 0.1 / 0.1 / 42.1 | 142 / 1.5 / 2.3 / 90 | 2 of 4 |
| rig LAND path, A (A+B, B never acted) | R2095, R2106 | 4.89 / 4.54 | 0.9 / 2.9 | 10.0 / 10.6 | 0 of 2 |
| rig LAND path, A alone | R2134, R2136, R2137 | 5.14 / 4.63 / 4.56 | 1.0 / 4.6 / 4.6 | 11.0 / 1.5 / 1.9 | 0 of 3 |
| hover, off / A | R2023, R2035 / R2164 | 4.40, 4.44 / 4.42 | 4.7, 4.6 / 4.7 | 0.4, 0.3 / 0.2 | 0 / 0 |
| LAND in the creep, A+B / A | R2098 / R2165 | 3.45 / 3.38 | 4.7 / 4.5 | 0.7 / 4.0 | 0 / 0 |
| LAND in the lift, off / A | R2166 / R2167 | 4.50 / 4.40 | 4.1 / 5.1 | 0.4 / 1.1 | 0 / 0 |

- Off, the rig LAND path fires the false touchdown in 2 of 4 runs (0.42 m up, the fleet falls). With A: 0 of 5.
- The 10-11 deg end tilt on that path: 1 of 3 A-alone runs on 0a77fa1 (R2134), 2 of 2 A+B on b7ab074; 0 of 2 normal off landings.
- With A + `land_descend_to_stall` on the same path (R2144, R2163; land card) the end tilt is 0.4 / 1.0.

## Reviewer on the re-fly (17 Oct ~17:43): supported (twin default)

1. Counts and trees reproduced; A-alone runs on 0a77fa1, the configs differ by the one key; no fault in the 9 re-fly runs.
2. Off 2/4 against A 0/5: one-sided Fisher p 0.17 (final tree alone 1/2 against 0/3, p 0.40). The claim stands on the
   mechanism: the fire comes 1.08 s after ring touchdown, and the reset makes a fire before 1.5 s impossible by construction.
3. End tilt on the final tree is 1 of 3 (fixed above); 11 deg is under the 12.2 bar and far from the 90-144 deg falls.
   With A, drone 0 is 4.6 cm up at LANDED (R2136/R2137): H2's 5 cm bar passes by 0.4 cm.
4. "Lands as its base" is same-tree only for the lift pair (R2166/R2167); the hover base is on c3bf628, the creep base is
   A+B on b7ab074 and R2165 ends at 4.0 deg against 0.7.
5. A's fresh-pose test uses wall time (0.2 s); the twin ran at RTF 0.23-0.41, so that branch has not run at RTF 1.
Must not say: "ready for the rig". The rig needs the critic's look at the code path and Wesley's word.

## Critic (rig path, 17 Oct evening, ~19:50): go with must-fixes

1. Tried before? The re-arm is new; the combined-M1 LAND with QUAD4 folded in was left out of the card.
2. Stale poses: a stale pose only zeroes `_stall_ct` (planner_node.py:1036); it cannot hang the fleet, since the max-drop
   exit counts planner ticks (planner_node.py:1400, 1415), ~7.5 s after the ring's rest height. Freshness is arrival
   time (planner_node.py:969): if Motive repeats a lost body's last pose, the frozen pose passes (= A off). The hold
   path has no solve, so ticks are cheap after ring touchdown.
3. A ring that never reads down (tilted, on a mat, z above lift_z0 + 0.05, planner_node.py:1131): no reset, A does
   nothing; z noise latches one harmless reset. The max-drop exit is a real backstop.
4. Unwelded QUAD4, departed drones, `_drop_tick`, his LAND: untouched; the 15 s departed wait starts up to ~1.5 s later.
5. A cannot cause a disarm in the air, and does not prevent R2088's real 16 cm stall.

Must-fix:
1. **A welded QUAD4 is never stamped:** `_attach_drone_cb` (dissipative_node.py:727-740) writes `drone_pos[d]` but not
   `_drone_t[d]`, so with A on every tick reads stale: no stall touchdown, LAND only by max drop (~7.5 s pressing z 0;
   with `land_descend_to_stall` the reference sinks 1.5 m below the anchors). Fly one combined-M1 LAND with QUAD4 folded in.
2. The stale-pose unit test the card promised (none existed; test_touchdown_gate.py only replays R2032).

Should-fix: the count of stale resets in the touchdown log line (planner_node.py:1422); before the rig, read one rig
log through a dropout (does Motive skip a lost body or repeat it?).

**Taken (17 Oct ~19:55):** must-fix 1: `_attach_drone_cb` stamps `_drone_t[d]` for an OCP-attached newcomer
(dissipative_node.py). It was live in the twin default (sim.yaml) from 7468bb7 until this fix; no run flew in between.
Must-fix 2: `src/mpc_planner/test/test_touchdown_stale.py` (fresh poses reach the stall; one stale pose holds the count;
an unstamped slot reads stale; a welded QUAD4 is stamped). 387 planner tests and gate --quick pass. Combined-M1 LAND
with QUAD4 folded in: `partner_attached_orbit` x2 with A on (below). Should-fixes: open, for before the rig.

## Critic must-fix 1 re-fly: LAND with QUAD4 folded in (18 Oct, flight code 9d30665, scored ~06:30)

**What flew with A and what did not.** The QUAD4 configs on an old-worlds SDF (`partner_attached_orbit`,
`q4_welded_land`, their `td_nf_` twins) make the runner type `legacy:=true`, and `sim_legacy.yaml` does not set A: the
planner read back `land_rearm_touchdown: false` in R2172, R2173, R2192, R2193, R2200 and R2237-R2240. They do not
test A (correction rows R2172c, R2173c, R2192c2, R2193c, R2200c2). The combined-M1 join configs (`m1c_*`, the rig-twin
pickup world) fly the twin profile, A on (read back true in every one).

| run | scenario | QUAD4 at LAND | exit | heights at the stall (cm above rest) |
|---|---|---|---|---|
| R2220 | m1c_hover | welded, folded in | stall | 4, 0, 0, QUAD4 0 |
| R2221 | m1c_high | welded, folded in | stall | 5, 0, 0, QUAD4 1 |
| R2222 | m1c_low | welded, folded in | stall | 5, 3, 0, QUAD4 0 |
| R2223 | m1c_offset | welded, folded in | stall | 4, 0, 0, QUAD4 3 |
| R2228 | m1c_nocatch_hover_2 (a false proof, below) | folded in, not caught | stall | 4, 0, 0, QUAD4 0 |
| R2233 | m1c_abort | welded, folded in | stall | 4, 0, 0, QUAD4 6 |
| R2234 | m1c_abort | welded, folded in | stall | 5, 0, 0, QUAD4 3 |
| R2224-R2227, R2232 | QUAD4 taken back, not folded in | - | stall | carriers 0-5 |
| R2235, R2236 | m1c_abort_negx | welded, folded in | TB14 tip-over 2 s after ring touchdown (abort before the 1.5 s grace + 1.0 s stall can act) | - |

**Must-fix 1: met.** In all 7 landings with QUAD4 folded in under A, the stall fired with QUAD4 in the list, never the
max drop: the stamp fix works on the twin's combined-M1 path. The TB14 tip-overs (R2235/R2236) are the known rigid-rod
landing artefact (combined-M1 card 6.4, 12 of 48 in October; the rig's tether goes slack); they start 2 s after ring
touchdown, before A can act. The join runs are exploratory for the mission itself (not thesis evidence).

**Open:** the legacy profile has no A. Whether `sim_legacy.yaml` should get it is a launch default (Wesley). The
should-fixes (the stale-reset count in the log line; a rig log through a Motive dropout) are still open before the rig.

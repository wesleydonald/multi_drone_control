# Plan: rescoping the results around what the approach adds (30 Oct 2026)

**APPROVED 30 Oct** (docs/decisions.md): all of it except X9 (dropped); freezes by Thu 19 Nov at the latest (aim
Thu 12 Nov); both real-world setups reported; the 2-quad real-world rows are the 28 Oct pair flights on the M2A ring.

Draft: nothing approved, flown or changed in src/ or configs/. On approval it amends
`docs/plan_thesis_results_2026-10-21.md` (spec items stay, the freeze date moves). Brief: "how do I prove my approach has
added value and what can it do", in simulation (no cage, no packs), with six real-world sessions left.

**Recommendation.** Prove added value with ablations of the same event (with and without the resize, the measured pull,
the blend) and with the paper's method put through a loss, not with more fixed-team tracking. Show what the approach can
do with missions only it makes possible: a ring that outlives every pack, carriers leaving on errands and coming back,
losses at random times. In the real world aim first for the result nobody has, a join to a carried payload. Freeze
simulation Mon 9 Nov and the real world Thu 12 Nov. Use 18-19 Nov for videos only.

## 1. Contributions

| # | New | Enables |
|---|---|---|
| C1 | A load-cable MPC whose attachment geometry is a runtime parameter, with a pre-built problem per team size warm-started from the measured state: a change of team costs a planner tick, not a rebuild | losing or gaining carriers without putting the payload down |
| C2 | An unannounced loss detected from motion capture, a drop rule from the survivors' largest gap, the measured cable pull fed forward | flying on after a loss when the geometry allows, landing safely when not |
| C3 | A departed carrier rejoins its plate. A newcomer flown by another team's controller is admitted only on a co-motion proof, then folded in at its measured catch point | gaining carriers in flight, including ones our planner does not fly |
| C4 | The events compose while the ring flies (chains, relays, errands, two losses at once, a loss during a fold-in) | carriers leaving to recharge or work and coming back: a mission not bounded by one team or one pack |

## 2. Evidence today and the gap

| # | Evidence (run ids) | Status | Gap |
|---|---|---|---|
| C1 | Releases resized within 0.4 s at 4 and 8 quads (R2708/R2709, R2766/R2767). 4-quad claim cells hold at 1, 2, 5 iterations (R2700-R2711, R2796-R2801). Fixed team against the paper's method: ours closer on every cell, rim and hub (line 0.4: 46 vs 99-105 mm, R2685-R2690, R2735-R2737; hub R2750-R2763). The paper's damping form lost the ring 6/6 (R2744-R2749). The exact law hovers ~110 mm off (R2805, R2812) | fixed-team comparison confirmed (2 runs a cell) | **No arm without the resize**: nothing shows the resize is what helps (the RA-L review's point 8). The paper's method never met a loss. Real time to ~3-4 quads only; 8 is twin only |
| C2 | Real world, 0.55 m: 3 unannounced releases, detected in 0.33-0.42 s, tilt 6.5-17.8 deg (RIG-1008-r303_4, -r304_3, -r305). **Real world, 1 m, 1.15 kg: 4 unplanned plate-0 losses on 29 Oct, each detected and handled by the drop rule, every drone landed** (RIG-1029-h4, -ch1, -ch1b, -ch1b_2). Twin: the tilt rise grows with the largest gap, 165 deg crosses 30 deg (R2604-R2628); the measured pull cuts the rise to ~2/3 | rig release shown (n = 3, 0.55 m); gap curve confirmed | No planned 1 m release in the real world. Twin square-to-T unannounced: peaks 57-61 deg (R2249, R2823); with the rules on, the ring is dropped (R2254). No "unaware" baseline |
| C3 | Twin rejoin: 10 of 11 fold-ins under 5 deg, welds within 1.5 cm (R2578-R2596, R2602/R2603, R2712/R2713). Tejen's newcomer welds in the twin (R2843/R2844, R2862, R2864/R2865, R2896-R2908). Catch offset U: measured arm 8 cm rim / 20 cm radial vs slot 4 / 6 (R2629-R2732) | twin shown; U **between** | **No real-world weld yet** (RIG-1029-sq5_2, -k1_4); one real catch refused by the proof (RIG-1014-t6h_4). The twin welds wherever the tip is |
| C4 | 8 -> 4 -> 8 claims: no drop, release peaks <= 2.9 deg, back at 8 missed its position bar 6.4 vs 5.7 mm (R2766/R2767). Nine harder cases once each, no falsifier tripped (R2768-R2779). 4 -> 3 -> 2 -> 3 with Tejen's attach (R2829); soft 4 -> 3 -> 2 (R2840/R2842) | **exploratory** (1 run each) | Not claims. The supervisor's N table and N -> 2 -> N not flown. Rejoins only with the path held. Nothing long |

The tables span several code trees and mostly flew the `ema` filter (`slope` since 28 Oct): each cited table re-flies on
the final tree (X11).

## 3. New simulation experiments

Twin, 1 m. Exploratory first pass now; claims (2 repeats, bars frozen in a card) after the tree freeze. Cost: 4-quad run
~4 min on 4 slots (~60/h); 8-quad ~20 min on 2 slots (~6/h). Lab-PC time (~27 h over 10 days) is not the limit;
scoring, review and Wesley's reading are.

| Id | Claim (set-up) | Metric | Baseline | Falsifier | Runs; h | Pri | New code |
|---|---|---|---|---|---|---|---|
| X1 | **Release ablation**: the resize at detection, with the measured pull, keeps the ring up. One unannounced release, hover and mid-figure-8, 4 quads (M1 layout, plate 9; cell R2708/R2709) and 8. Arms: ours, no resize, resize 0.5/1/2 s late, planned pull, the network's hand-over (4 only) | tilt rise, height dip, path error 10 s after, drop, time to band | no-resize | no-resize within 1 deg and 5 mm of ours on both fleets | 22 + 44; 5 | **P1** | `resize_delay_s` (twin switch, ~20 lines + test); smoke the no-resize arm; offline: time to build an n-1 solver |
| X2 | **Supervisor's chain N**: 4 -> 3 -> 2 -> 3 -> 4, hover / figure-8 / line_spin, a lap per stage | per-event tilt, per-lap path error | same paths, 4 quads, no event | a drop, or a release peak > 30 deg | 11; 1 | **P1** spec | none (laps built 23 Oct); see D3 |
| X3 | **8-quad set as claims**: R2768-R2779's nine cases + the owed 8 -> 2 -> 8 (`s8_chain2back`, never flown) | S card bars L, J, N4, P | s8 hover / figure-8 / orbit | a drop, or L or J missed in both repeats | 20; 4 | **P1** | it5 config copy; scorer merge for same-time releases |
| X4 | **Endurance relay**: 8 quads, lapped figure-8, 30 min. Each lap end one carrier leaves (announced soft detach) for a charging pad (waypoint + dwell), a charged one rejoins in the hold; >= 7 on the ring; none airborne past a pack budget B; one leave per cycle is an errand (pickup, then delivery point) | mission time / B, swaps done/tried, per-event tilt, per-lap error trend, height drift | fixed team (lands at B); laps without events (R2778) | a drop, a failed rejoin, or last-lap error > 1.25 x the first | 3; 5 | **P1** headline | tools: rota generator, per-lap scorer. B is a budget, not a battery model. X4b (P3): a 200 m line, pads along it |
| X8 | **The paper's method through a loss** (its own resilience claim): 4 quads hover, one unannounced loss, world C (their centre hook) and the rim; ours, the port, the exact law | load excursion, settle time, quad tilt, loss | the network | network excursion <= ours on world C | 15; 1 | **P1**, 1 day box | check how the network drops a node; flight-loop code -> critic, X8 to P3 |
| X5 | **Physical pickup and delivery**: QUAD4 leaves, picks up the plate, drops it in the ring's net, rejoins | m/n done, tilt per phase, throttle before/after | Nr without object | < 4 of 5 complete | 5; 0.5 | P2 | none (R2898-R2900, R2905) |
| X6 | **Random failures**: survival is set by the largest gap, so more carriers buy tolerance. Seeded carrier and time on the figure-8, 4/6/8 quads x10, + 5 no-resize each | drop rate; tilt vs gap and path acceleration | no-resize; Tier B | a loss leaving <= 135 deg drops, or tilt not ordered by gap | 45; 4 | P2 | seeded event generator; 6-quad configs (world exists) |
| X7 | **Join ablations**: blend vs step (`rejoin_blend_s` ~0); measured vs slot point (U, done); attach paused vs moving (`attach_moving`) | fold-in tilt, path error after | ours (R2578-R2596) | the step arm also stays under 5 deg | 10; 0.7 | P2 | none if the runner passes `rejoin_blend_s` |
| X9 | **Delay tolerance**: 0/45/90 ms mocap delay on X1's release (the rig had 95 ms on 30 Oct) | tilt rise, rocking | 0 ms | none (characterisation) | 12; 1.5 | P2 | none (R2909-R2914 configs) |
| X0 | **A join buys margin**: per-carrier throttle per stage from flown runs (R2708, R2766, R2840, R2901; rig RIG-1029-sq1) | throttle p95 | none | none | 0; 2 h analysis | P2 | script |
| X11 | **Final-tree re-fly** of every cited table (R as cited, U, S, C, rejoin, B twin) | as each card | as flown | as each card | ~260; 6 | **P1** | none |

Not proposed: beyond 8 quads (cut first if raised); rejoin while the path runs (flight-loop code; the lap-end hold is
the stated limit); a quad moving between two teams; a battery model. Beside every claim: rigid hinged rods, welds at
any tip position, no collisions, 8 quads below real time.

## 4. Real-world sessions (Wed/Thu 8-11, ~10-15 flights each)

Each opens with the mocap stream check (<= 8 bodies) and the arming check.

| Session | In order | Stop rule |
|---|---|---|
| S1 Wed 4 Nov | h3t triangle hover with the reduced body set (heave gone?), the M1 four-quad hover, Tejen's attach sq6b x3-4 (**first real join**), the 1 m release x2 (QUAD4's magnet off, unannounced) | heave back: stop ring flights, blackbox one carrier |
| S2 Thu 5 Nov | attach x3 (first, if S1 went to the heave); baselines at 1 m: h3, f3, ls3, h4, ls4 (f4 exists); release x2 if missing | as S1 |
| S3 Wed 11 Nov | Nr in hover x3 (release, Tejen's re-attach); if the attach is unreliable, attach attempts only | |
| **Cut line** | below it nothing is counted on for the thesis | |
| S4 Thu 12 Nov | K in hover x2 (k1_7 fallback), Nr on the figure-8 x1, missing re-flies. **Real-world freeze 11:00** | |
| S5/S6 18-19 Nov | videos, a demo for the supervisor | |

Cut first: the 2-quad rig baselines (the pair cannot carry 1.15 kg on 1 m cables: keep the 0.55 m rows, labelled), the
rig 4 -> 3 -> 2 chain, line_spin with 4, Nr on paths, K.

## 5. Calendar

| Dates | Simulation (lab PC) | Real world | Thesis (agents draft in parallel) |
|---|---|---|---|
| Sat 31 Oct - Tue 3 Nov | cards X1-X4, X8 (bars frozen); small code, gate; first passes | S1/S2 sheet | audit, drafts |
| Wed 4 - Thu 5 Nov | first passes X5-X7, X9 | S1, S2 | |
| **Fri 6 Nov** | **tree freeze** (better-becomes-default calls first) | | |
| Sat 7 - Sun 8 Nov | claims X1-X4, X8 + X11, one tree | | |
| **Mon 9 Nov** | **simulation freeze**; a reviewer per table to Tue | S3/S4 sheet | sim figures; writing weeks start |
| Wed 11 - Thu 12 Nov | | S3, S4; **freeze Thu 12 11:00** | |
| Fri 13 - Tue 17 Nov | | real-world tables, figures | chapter 4-5, abstract; full draft to the supervisor Mon 16 (suggested) |
| Wed 18 - Thu 19 Nov | | videos only | final edits, PDF Thu 19 |
| **Fri 20 Nov** | | | **hand-in** |

Figures (the figure agent with the thesis-figure skill; Wesley signs each off): by **Tue 10**: what the resize is worth
(X1), the ring outlives every pack (X4), survival against the largest gap by fleet size (X6 + Tier B), the chain N on
three paths (X2), our planner against the paper's method fixed and through a loss (C + X8), and the re-rendered catch
offset, 8 carriers lap by lap and planner tick against N (F13-F15). By **Fri 13**: the real-world figure-8 with 2/3/4
carriers (F11, + S2), the real-world release (F12, + a 1 m panel), the first real join if it happens (analysis agent,
then figure agent). Photos and the mission storyboard: Wesley.

## 6. Risks and cut order

- **The heave is not the mocap delay** (S1's h3t heaves with <= 8 bodies): ring flights stop; the real-world chapter
  stands on C2's rows, B and the 30 Oct root cause.
- **Tejen's attach never welds** (S1-S3): the real-world join stays open, said plainly; the twin carries C3.
- **The no-resize arm is ill-defined** (X1 smoke: the leaver misbehaves): the harness steps it out, planner unaware.
- **The network cannot take a loss as built** (X8 check): X8 to P3, C stays fixed-team.
- **Lab PC busy**: laptop on non-lab days, one run at a time; 8-quad arms go first.
- **Code after Fri 6** means a new tree and a re-fly: only void-making bugs are fixed after it.
- **Experiments eat writing time**: hard stop on new runs Mon 9.

Cut order: X4b, X9, X7, X6's 8-quad arm, X5, X8 (if it needs code), S5/S6, K, Nr on paths.

## 7. Decisions for Wesley (recommendation first)

1. **Contributions** C1-C4 as in §1. Recommend: approve (they match both RA-L drafts).
2. **Headline added-value evidence**: X1 (ablation) and X4 (endurance), with X2 and X3 as claims and X8 boxed to a
   day. Recommend: yes.
3. **Chain N's square-to-T step**: (a) unannounced, as decided 21 Oct, peaks 57-61 deg and the rules drop the ring;
   (b) soft detach for that step only (25-29 deg, R2840/R2842), unannounced for T to pair; (c) start from the M1
   layout. Recommend (b), reporting (a) beside it as the gap rule's boundary. Push-back: unannounced-only cannot pass
   this layout at 1 m, and the supervisor asked for as little tilt as possible.
4. **The twin's ring for claims**: 0.86 kg (every table so far) with one 1.15 kg cell per table, or all at 1.15 kg.
   Recommend: 0.86 kg plus one 1.15 kg cell.
5. **Re-fly scope**: every R table, or only those the thesis cites. Recommend: cited only.
6. **Freezes**: tree Fri 6 Nov, simulation Mon 9 Nov, real world Thu 12 Nov 11:00 (replacing Sat 7 Nov); 18-19 Nov
   videos only. Recommend: yes. Push-back: anything flown 18-19 Nov lands after the text closes.
7. **X4**: planned swaps announced (a swap is planned), B = the flight time you measure with the ring (placeholder
   5 min). Recommend: yes, and give B.
8. **New code**, twin only, switches default off: `resize_delay_s`, the rota and failure generators, the scorer merge.
   Baseline arms, not control laws. Recommend: approve, critic waived for `resize_delay_s`; X8's network loss handling
   gets the critic if it touches the law.
9. **Real-world order and cut line** as §4 (the first real join right after the heave check). Recommend: yes.
10. **RA-L drafts** (`docs/paper/ral_2026-10-23*`): park until after 20 Nov. Recommend: park.

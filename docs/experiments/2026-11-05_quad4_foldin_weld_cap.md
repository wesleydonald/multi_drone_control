# Card: QUAD4's fold-in under the rejoin's weld-height cap (foldin_cap, twin only; 5 Nov 2026, rev 2)

Owner Wesley. Origin: decisions.md DECIDED 2026-10-30 (Wesley 16:45, X11): "QUAD4's fold-in weld-height cap (R2829
peaked 9.9 deg; the rejoin has the cap): a card and the critic after the T fix, before the code freeze";
handin_plan.md "After the T fix, before the freeze". **Status (5 Nov ~16:00): CLOSED by Wesley's decision (end of the
card): `foldin_cap level` kept, off by default; `weld` deleted; no default change. Sections 1-9 are rev 2 as written
before the runs (14:30); the results and the decision are at the end. Simulation bars were written before the runs and
were judged after them (Wesley 2 Nov).**

## 1. Question
Does putting QUAD4's fold-in blend under the rejoin's cap (`_rejoin_cap`) stop the fold-in from tilting the ring
where QUAD4's height is the ring's pitch (R2829, onto the pair), without slowing the fold-in or changing it in the
joins the thesis claims (the 4.3 joins on the 1.15 kg ring)?

## 2. What the logs show (read 5 Nov from run.csv; no new runs)
Tilt is the ring normal's angle from vertical; "plate 9" is plate 9's height off the level ring (+ above); the window
is the weld to LAND (+10 s in every run here).

| Run | Join | Tilt at the weld | Plate 9 then | Peak tilt (when) | Peak minus weld | QUAD4's rise (when) | Under 3 deg for good from |
|---|---|---|---|---|---|---|---|
| R2829 (0.86 kg, int_k_xy 0) | pair 0/6 -> T (QUAD4 the rider; the pair alone ~30 s) | 0.1 deg | 0.0 cm | 9.9 deg (+1.41 s) | +9.8 deg | +3.9 cm (+1.39 s), plate 9 +3.9 cm | +7.5 s |
| R3708 (0.86 kg, integral on) | T -> square | 6.8 | -2.7 | 6.8 (+0.01 s) | +0.0 | +1.2 cm (+1.61 s) | +2.1 s |
| R3712 | T -> square | 15.0 | -5.8 | 15.1 (+0.18 s) | +0.1 | +4.7 cm (+2.17 s) | +3.3 s |
| R3714 | T -> square | 14.7 | -5.7 | 14.8 (+0.26 s) | +0.1 | +4.4 cm (+2.25 s) | +3.6 s |
| R3715 | T -> square | 15.7 | -6.1 | 15.8 (+0.22 s) | +0.1 | +4.7 cm (+2.27 s) | +3.8 s |
| R2843 / R2844 (1.15 kg, integral on, 28 Oct) | T -> square, from the floor | 7.3 / 7.9 | -2.9 / -3.1 | 7.4 / 7.9 (+0.27 s) | +0.1 / +0.0 | +1.0 / +1.3 cm (+2.1 s) | +5.0 / +4.7 s |
| R3511 (0.86 kg) | triangle 3/11/7 + QUAD4 | 0.1 | 0.0 | 1.8 (+1.86 s) | +1.7 | none (it descends) | +0.0 s |
| R3674 (1.15 kg, fri30_sq6b, 4 Nov) | triangle, from the floor | 0.1 | 0.0 | 1.8 (+2.73 s) | +1.7 | none | +0.0 s |

- **R2829 is the rejoin's E2 (R2584) again.** Folded onto the pair, QUAD4 is the T's rider: the pair on one diameter
  holds no pitch about its axis, so on the twin's rigid rod QUAD4's height is the ring's pitch. Its plan starts from
  its measured body (node 0 pinned), the fold-in's thrust step lifted the body (it led its reference by ~6 mm at
  +1.1 s) and the plan followed it 3.9 cm up; plate 9 went 3.9 cm above level and the ring 9.9 deg. R2829 flew the
  27 Oct tree and links; whether it still happens on the links of 4 Nov is S1's first question.
- **The T -> square peaks of 5 Nov (14.8-15.8 deg) are not the fold-in's.** They are the T's own lean at the weld
  with the sideways integral on (plate 9 5.7-6.1 cm low); the fold-in raises QUAD4 4.4-4.7 cm and levels the ring in
  3.3-3.8 s, 0.1 deg over the weld instant.
- **The triangle's fold-in never lifts QUAD4** (it descends as its rod swings out).

**The rejoin's cap has two parts**: (a) no node above where a *level* ring puts the drone on its rod, and (b) a ratchet
that never lets the reference rise above the weld height. (a) alone stops R2829 (the ring was level at the weld). (b)
would hold QUAD4 at its weld height where the ring leans down on plate 9 at the weld (R3712-R3715), so the lean would
stay until the cap lets go at the blend's end (+10 s) against 3.3-3.8 s today. So the switch has two modes: `weld`
((a) + (b), the rejoin's cap as decided) and `level` ((a) alone).

## 3. Hypothesis (predictions, twin, hover, 1 m)
- S1 off on today's tree: R2829's problem is still there (excess >= 5 deg). If it is not, the cap has nothing to fix.
- `weld` and `level`: S1 peak at most 5 deg (the rejoin's E2 went 10.6 -> 1.45-3.0 deg), the cap binding on nodes 0-1;
  S2 and S3 (the 1.15 kg claim joins, int_k_xy 0 where a T forms) as off.
- `weld` only, the stress case (the T leaning 11-15 deg at the weld with the integral on): the lean held until ~+10 s.
- **Not claimed:** anything for the rig (refused there). The twin's rods are rigid and push; the rig's cables only pull.

## 4. The one variable
`foldin_cap` (planner; '' / `off` default, `weld`, `level`; twin only). Everything else as each scenario's config. Off
is today's tree bit for bit (section 8).

| Scenario | Off (baseline) | weld | level | Why |
|---|---|---|---|---|
| S1: R2829's chain, square -> T -> pair -> T (QUAD4 the T's rider), 0.86 kg | `sq_chain_soft_rejoin_hover_L100_rigargs` | `..._foldcap_weld` | `..._foldcap_level` | the case the cap is for |
| S2: the claim's T -> square join from the floor, 1.15 kg, int_k_xy 0 (claim batch chunk 8b) | `sq_floorjoin_hover_L100_p115_noxy` | `..._foldcap_weld` | `..._foldcap_level` | the join the thesis cites, where a T forms |
| S3: the claim's triangle join from the floor, 1.15 kg (chunk 8b) | `fri30_sq6b` | `..._foldcap_weld` | `..._foldcap_level` | the join the thesis cites, nothing wrong today |
| S2x (stress, `weld` only): the T -> square with the integral on, 0.86 kg (R3712/R3714) | `sq_nr_hover_L100_1104_rigargs` | `..._foldcap_weld` | | where the ratchet should hurt |
| S4 (optional): the orbit re-attach with attach_pause (R3696) | `nr_orbit_L100_p9_pause_rigargs` | | `..._foldcap_level` | a path: the cap must let go once the circle moves |

S1's off config is new: R2829's events and params file (`configs/rig/chain_pair_attach_1m.yaml`: int_k_xy 0, drop_gap
200 with 2 survivors) on today's tree, with today's velocity filters (slope on the carriers; R2829 flew ema on all four),
his partner args as the rig line's $V $T3 (the Python approach, low route; R3706 stalled 300 s on his C++ route), the
rider's DETACH 7 s after QUAD4's (R2829: 15 s; his Python route reaches contact ~10.7 s after the partner release, so
the pair now hangs ~10 s alone after its resize; R2829's T was under 1.5 deg from +4 s after QUAD4's release, so the
rider's unload passes its 10 deg gate), and LAND 20 s after the re-weld (R2829: 10) so the cap's letting go at +10 s
is seen in hover. The S2/S3 arms are copies of the claim cells with `foldin_cap` typed and nothing else changed. All
configs in `configs/experiments/`; all pass `tools/remote/lab_batch.py --dry-run` (5 Nov ~14:25).

## 5. Bars (simulation; written 5 Nov before the runs, may be judged after them, Wesley 2 Nov)
Scored from events.csv (REWELD or WELD, DETACH_RESIZE, FOLDIN_CAP_ON, FOLDIN_CAP_OVER, LAND), run.csv and the
FOLD-IN CAP OVER line. "Excess" = the largest tilt from the weld to min(weld + 15 s, LAND) minus the tilt at the weld.
"Settle" = the time after the weld from which the tilt stays under 3 deg until LAND. **"Binds"** = the line's "nodes
0-1 at most X mm above it" with X >= 2 mm (FOLDIN_BIND_M): nodes 0-1 are what QUAD4's tracker flies now; far nodes bind
~6 mm whenever the measured ring centre sits under the plan's (R3712: 0.994 against 1.000 m), and that alone is not a
bind. Every arm against its scenario's off arm on the same tree.

| Scenario | An arm passes when | It fails when |
|---|---|---|
| All | the weld; every quad lands; no drop or abort; FOLDIN_CAP_ON at the weld and FOLDIN_CAP_OVER by LAND (neither with off) | any of these missing |
| S1 void rule | a run whose REWELD comes before, or within 5 s after, the rider's DETACH_RESIZE is void, not scored (the pair not settled, or the cap ends at once on the second resize) | |
| S1 gate (off) | excess at least 5 deg: the capped arms fly | under 5 deg: R2829's problem is gone on today's tree; no arm flies, the card closes with that result and the code is deleted by default |
| S1 | peak at most 5 deg (the weld to +15 s), excess at most half the off arm's, and the cap binds | peak over 5 deg, or no bind |
| S2 | settle no later than off's + 1.0 s, excess no more than off's + 1.0 deg | either worse |
| S3 | excess within 0.5 deg of off's, and no bind | over, or a bind |
| S2x (`weld`) | reported, no bar; settle later than +8 s means `weld` is not proposed as the default | |
| S4 | FOLDIN_CAP_OVER says "the path moves" within 5 s of the weld; peak within 1.0 deg of off's | either |

A mode becomes the proposed default only if it passes S1 x2, S2 and S3; Wesley's word (a planner default), after the
reviewer.

## 6. Order (the run list)
1. **S1 off**, once (with **S2 off** beside it: the claim cell, it says whether that join has an excess of its own).
2. Only if S1 off passes its gate: S1 `weld`, S1 `level`; S2 `weld`, S2 `level`; S3 off, `weld`, `level`; S2x off and
   `weld`.
3. S1's second repeat of each mode still passing, with S1 off again; S4 (off and `level`) if a mode is proposed.

## 7. Cost
2 runs, then 9, then 3-5, ~5 min each on the lab PC (2-4 slots).

## 8. New code (uncommitted on week8)
`src/dissipative_planner/dissipative_planner/dissipative_node.py`, all behind `foldin_cap`:
- `foldin_cap` (node parameter, 'off' default; anything but off/weld/level refuses to start). `FOLDIN_BIND_M` (2 mm).
- `_attach_ocp`: for a reserved newcomer (QUAD4: `j` not None), not a rejoin, not a start weld before takeoff (the
  floor start's creep weld returns earlier; the air and taut start welds are left alone, their rods laid out like a
  carrier's), after the blend starts: `_foldin_cap_start(d, rho_new, lens[-1], measured)`. The OCP's inputs (rods,
  plate points, blend, hold) are untouched.
- `_foldin_cap_start`: QUAD4's cap state: its plate point (the OCP's rho); its rod = |pivot - plate point| clipped to
  REJOIN_WELD_M (1.5 cm) of the OCP's rod. The capture puts the plate point body_to_tip under the body, so that arm is
  a constant of the arm, not a measurement (the rejoin's is one); the line says so ("pivot to the captured plate
  point"). The weld height for `weld` (infinity for `level`), the ratchet for `weld` only. Line `FOLD-IN CAP ON` with
  the rod, the weld height and plate 9's height off level.
- `_rejoin_cap(i, nodes, b=None)`: reused, not copied. `b` None is the rejoiner's state as before (its arithmetic and
  its state dict unchanged); a state with `rho` is QUAD4's; `ratchet` False skips the weld-height ratchet; for QUAD4's
  state it keeps the largest node-0/1 margin, the last one, and the ticks from 2 mm.
- `_publish_ref`: QUAD4's nodes go through the cap while its state exists (after the rejoin's, before the feed-forward
  gate, as the rejoin's).
- `_foldin_cap_tick` (each planner tick, before the drop branch): the cap lets go for good when QUAD4 leaves the ring,
  the fleet resizes again, the ring drops, at LAND or the descent, once the path clock runs with no hold (the rejoin
  card's critic M2; QUAD4's fold-in holds the path only 1.5 s with attach_moving), or when the blend ends. Line
  `FOLD-IN CAP OVER` with why, the ticks any node bound, the last far margin, the largest node-0/1 margin and its ticks
  from 2 mm, and the last node-0/1 margin (the step the reference takes as the cap lets go).
- Launch: `foldin_cap` in `src/bringup/config/common.yaml` ('' = off) and `PLANNER_OPTIONAL` (`real_mode.py`, passed
  only when typed); `planner_options` refuses it with `real:=true` (as plan_rod_level and traj_laps).
- Runner: `FOLDIN_CAP_ON` / `FOLDIN_CAP_OVER` events (`tools/experiment/runner_node.py`).
- Tests: `src/dissipative_planner/test/test_foldin_cap.py` (18): off unchanged; a start weld not capped; R2829's
  model with the pivot at 0 and -0.04 m; an arm off the OCP's rod clipped and logged; `weld` equal to the rejoin's cap
  node for node; the leaning T (level lifts, weld holds); a ring sagging 6 mm under the plan (far nodes bound 6 mm,
  nodes 0-1 0.3 mm: no bind); every let-go and its line; the publish wrapper; the step at the let-go equal to the last
  node-0/1 margin. `test_real_mode_launches.py`: typed only, refused on the rig. `tools/test/test_rejoin_runner.py`.

## 9. Risks
- **The cap clamps outside the OCP** (the rejoin critic's M1): the position is lowered and the upward velocity zeroed,
  but QUAD4's `a` and cable feed-forward pass through; the OCP plans the other carriers as if QUAD4 went where it
  planned. In S1 its share ramps up over the blend while its height is held.
- **`level` has no ratchet**, so it follows the measured ring's centre: a ring rising as plate 9 lifts raises the cap
  with it (a small leak the ratchet closes). S1 says whether it matters.
- **The let-go at the blend's end** (+10 s): the reference steps up by the last node-0/1 margin (logged); R2594/R2595
  peaked 1.45 / 2.25 deg ~1.3 s after the rejoin's cap let go. S1 LANDs at +20 s to see it; S2 and S3 LAND at +10 s,
  where the cap ends with LAND.
- **S1's timing**: the rider leaves 7 s after QUAD4. Its unload needs the ring under 10 deg within 2 s after its 3 s
  blend (R2829's T was under 1.5 deg by then); a cancelled unload leaves the T, and the run is void by the rule.
- **A path**: on an orbit without a hold the cap ends at the first tick the clock runs (S4); its benefit is then only
  the 1.5 s hold. Hover is where it acts.
- **Our own approach** (R8a/R8b, twin) folds in through the same `_attach_ocp` and gets the cap when it is typed.

## Critic
(project critic, 5 Nov ~13:45, via the main chat; its points as relayed, verbatim in substance)

1. (high) S1 welds onto a pair ~4 s old, not R2829's settled one (~30 s alone, level 0.1 deg): in
   `sq_chain_soft_rejoin_hover_L100_rigargs.yaml` the rider's DETACH is ~47.3 s, its resize ~3 s later, his contact
   ~10.7 s after PARTNER_RELEASE. Move the rider's DETACH to t ~27 so the pair hangs >= 10 s alone; void a run whose
   REWELD is within 5 s of DETACH_RESIZE (the cap ends at once when the fleet resizes again); fly S1 OFF first and the
   capped arms only if its excess is >= 5 deg (the falsifier: R2829's problem may not exist on the 4 Nov links).
2. (high if a mode becomes default) S2/S3 fly the 0.86 kg ring with the integral on; the claim batch's 4.3 joins fly
   `sq_floorjoin_*_p115_noxy` and `fri30_sq6b*` (1.15 kg, int_k_xy 0; docs/claim_batch_2026-11-07.md chunk 8b). Fly
   S2/S3 on those cells; keep the integral-on S2 only as the stress case for 'weld'.
3. (medium) "the cap bound" is not a test: `_rejoin_cap` counts a tick when any horizon node is over by any amount
   and bind_m keeps the last margin. Log the largest node-0/1 margin and put the bar at 2 mm ('level' binds ~6 mm on
   far nodes in R3712 because the measured ring centre sat 0.994 vs 1.000).
4. (low) the "measured" rod is an identity (rho_new = body - body_to_tip): fix the log wording.
5. (low) a non-creep start weld is capped too (the air/taut start-weld path reaches the blend): exclude
   `_start_attached and not takeoff_seen`.
6. (low) not refused with real:=true: refuse it on the rig as rejoin_departed is.
7. (medium) tests: pivot_offset is 0 everywhere; add tests at -0.04 m, a sagging ring centre, the step when the cap
   lets go, a start weld (point 5) and the rig refusal (point 6).

Verdict as relayed: rewrite the card, with small code changes.

## Response
(5 Nov ~14:30; nothing flown before or since)

1. Done. S1's rider DETACH at t 27 (7 s after QUAD4's), WAIT_REWELD at t 28, LAND 20 s after the re-weld, in the off
   and both capped configs. The void rule and the off-first gate (excess >= 5 deg) are in section 5, the order in
   section 6.
2. Done. S2 = `sq_floorjoin_hover_L100_p115_noxy`, S3 = `fri30_sq6b` (the chunk 8b base cells; their variants are not
   flown here), each with `_foldcap_weld` / `_foldcap_level` copies that type `foldin_cap` and nothing else. The
   integral-on T -> square stays as S2x, `weld` only, reported with no bar. Removed (never flown):
   `sq_nr_hover_L100_1104_rigargs_foldcap_level`, `nr_hover_L100_p9_rigargs` and its two arms.
3. Done. QUAD4's state keeps the largest node-0/1 margin, the ticks it was at least FOLDIN_BIND_M (2 mm) and the last
   one; the OVER line prints them beside the old count. "Binds" in section 5 is the node-0/1 margin >= 2 mm. The
   rejoiner's state and line are unchanged.
4. Done: "rod 1.000 m (pivot to the captured plate point 1.000, the OCP's 1.000)"; the docstring says it is a constant
   of the arm, not a measurement.
5. Done: no cap for `_start_attached and not takeoff_seen` (test).
6. Done: `planner_options` refuses `foldin_cap` other than off with `real:=true` (test: weld and level refused, off
   accepted).
7. Done: tests at pivot -0.04 m (R2829's model, and an arm off the OCP's rod clipped to 1.5 cm), a ring centre 6 mm
   under the plan (no bind by the 2 mm bar, far nodes 6 mm), the step at the let-go (equal to the last node-0/1 margin,
   through the publish wrapper), the start weld, the rig refusal.

Tests: `test_foldin_cap.py` 18 pass; the rejoin, partner-contact and lap-hold suites unchanged; launch tests pass.

## Results (5 Nov ~15:50)
Exploratory, one run per arm, scored after the runs by section 5 (registry rows R3743, R3747-R3754). "Excess" is the
largest tilt from the weld to +15 s (or LAND) minus the tilt at the weld; "bind" is nodes 0-1 at least 2 mm over the
cap (the FOLD-IN CAP OVER line).

| Scenario | Run | Arm | Tilt at the weld | Peak | Excess | Bind (nodes 0-1) | Verdict |
|---|---|---|---|---|---|---|---|
| S1 (R2829's chain, 0.86 kg) | R3743 | off | 0.1 deg | 5.6 deg | 5.5 deg (R2829: 9.8) | | gate met (>= 5): the arms fly; the problem is smaller on the 4 Nov links |
| S1 | R3747 | weld | 0.7 | 3.8 | 3.1 | up to 6.3 mm, 20 ticks | FAIL, narrowly (excess over half of off's, 2.75) |
| S1 | R3748 | level | 0.1 | 2.8 | 2.6 | up to 6.5 mm, 21 ticks | PASS (peak <= 5, excess <= 2.75, binds); under 2 deg from +10.7 s |
| S2 (the 1.15 kg T -> square floor join) | R3738 / R3749 / R3750 | off / weld / level | | | | | VOID: no weld in any arm; his C++ transit stalls in this cell on today's tree (a separate issue, under investigation elsewhere) |
| S3 (the 1.15 kg triangle floor join) | R3751 | off | 0.0 | 1.9 | 1.8 | | base |
| S3 | R3752 | weld | | | 1.8 | up to 4.0 mm, 6 ticks | FAIL the no-bind bar (excess holds) |
| S3 | R3753 | level | | | 2.1 (+0.3) | up to 2.2 mm, 1 tick | marginal: excess holds, a 1-tick bind just over 2 mm |
| S2x (stress: the T leaning at the weld, integral on, 0.86 kg) | R3754 | weld | 5.7 (R3714: 14.7) | 5.9 | | | reported, no bar: under 2 deg only at +13.8 s against +5.1 s in R3714; weld holds plate 9 low |

- **S1, the case the cap was built for:** on today's links R2829's fold-in excess is 5.5 deg, not 9.8. `level` halves
  it (2.6); `weld` nearly does (3.1).
- **S3, the triangle join the thesis cites:** neither mode helps (excess 1.8 off, 1.8 weld, 2.1 level). Both bind a
  little, so neither is free there.
- **S2x:** as section 2 predicted, the weld-height ratchet holds the lean through the blend (settle 5 -> 14 s).
- S2 says nothing about the cap: no arm welded.
- Not flown: S1's second repeat and S4 (stopped by the decision below).

## Decision (Wesley 5 Nov ~15:57)
- **`level` is kept, off by default.** It is cited only as an exploratory finding: it halves R2829's fold-in tilt on
  the pair -> T join (R3748 against R3743). No claim, no default change; it flies only where it is typed, in the twin.
- **`weld` is deleted** (its configs too; the run directories keep their copies and the registry rows stay).
  `foldin_cap` takes '' / `off` or `level`. The node and the launch refuse `weld` with a clear message, and the launch
  refuses any mode on the rig. The rejoin's own `_rejoin_cap` is back to its committed text (9f20f8b). QUAD4's cap is
  its own method, `_foldin_cap_nodes`: the level-ring half without the ratchet, plus the node-0/1 margin for its line.
- **The S2 cell's stall** (his C++ transit in `sq_floorjoin_hover_L100_p115_noxy` on today's tree) is a separate issue,
  not this card's.
- Checks after the change (5 Nov 16:05): `test_foldin_cap.py` 16 pass (level only), the launch tests (typed only;
  `weld` refused; refused on the rig) pass, the rejoin and partner-contact suites unchanged; `./tools/gate.sh --quick`
  passed (2199 passed, 6 skipped).

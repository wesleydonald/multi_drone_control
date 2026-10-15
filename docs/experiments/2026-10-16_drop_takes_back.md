# Card: a ring drop during his flight takes QUAD4 back (`drop_takes_back`), rev 2

Rev 1 written 15 Oct night (overnight loop, plan 2026-10-15 A2); rev 2 after the critic (below), before any run.
Exploratory safety change behind a switch, default off; Wesley decides the default and any rig use (no rig day
before 21 Oct). It touches the locked rules' "Open" line ("a drop while Tejen flies QUAD4 does not take it back"):
closing that line is Wesley's.

## Why

RIG-1015-a2_2: QUAD2 let go of plate 7 during his MATCH, the ring tilted 37 deg and the drop rule released every
magnet and landed the carriers. QUAD4 was outside it:
- the drop tick returns before the line that keeps QUAD4's warm reference (`dissipative_node.py` `_plan_inner`),
  so our idle tracker for QUAD4 faulted on a stale reference 1.04 s after the drop;
- with that tracker faulted, a GIVE UP cannot hand QUAD4 over (the mux needs our tracker flying,
  `drone_magnet/handover_policy.py`), so **today's operator rule "GIVE UP after a drop" does not work** either;
- LAND after a drop is swallowed ("LAND during a drop: every drone is already landing"), so his planner flew QUAD4 on
  toward the ring on the floor for 14 s;
- `/fleet/landed` waited the full 30 s timeout (QUAD4's drop hold is None).

## Change (all behind `drop_takes_back`, '' = off, `bringup/config/common.yaml`; planner option)

1. `_drop_and_land` runs the existing take-back (`_take_back(j, 'DROP')`) for every newcomer still pending, after the
   drop has set up the departed drones and released every coil: our tracker takes QUAD4 (mux ours), it is latched at
   its stopping point, rises over the carriers, goes radially out to 1.2 m from the ring's xy at the drop, lands and
   is disarmed alone.
2. The drop tick keeps publishing QUAD4's warm reference until a take-back owns it (critic F1; the hold check is
   `is not None`, since the drop inserts a None hold), so a take-back that returned early, or a later GIVE UP, can
   still hand it over.
3. `/fleet/landed`'s timeout after a drop is 30 s + 30 s while a take-back flies QUAD4 (critic F2: at 30 s the fleet
   disarm would now reach QUAD4, possibly in the air).

Not switched: the rig's release guard stands down after any drop (critic F3: every coil is OFF, the plate falls; its
timeout path would land QUAD4 0.6 m from the centre over the fallen ring). Only reachable after a drop.

Out of scope (critic F10): a non-partner newcomer (our own approach, twin/thesis) caught by a drop also loses its
reference; `_take_back` does nothing there.

## Arms (rig twin, combined M1 floor join = `m1c_floorjoin_hover` + an event; MATCH lasts 1.3-1.9 s in R1720-R1722)

| Config | Switch | Event | Runs |
|---|---|---|---|
| `w4_drop_his_match_off` | off | DROP 1.0 s into his MATCH_VELOCITY, LAND 0.5 s later | 2 |
| `w4_drop_his_match_tb` | on | same | 2 |
| `w4_release_his_match_off` | off | RELEASE drone 2 (plate 7) 1.0 s into MATCH (detection + drop rule, as a2_2), LAND 1.0 s later | 1 |
| `w4_release_his_match_tb` | on | same | 1 |
| `w4_drop_his_bottom_tb` | on | DROP 0.5 s into his ATTACH_READY (tip at the plate), LAND 0.5 s later | 1 |

## Bars (rev 2, frozen before the runs; scored from run.csv, events.csv and launch.log)

For every switch-on run, from the drop to QUAD4's disarm:
1. QUAD4's mux goes to ours (`Drone 4 mux: ours` / a MUX event) after the drop.
2. Until QUAD4 is at least 1.1 m horizontally from the ring's xy at the drop: QUAD4's body stays at least 0.5 m from
   every carrier's body, and its tip at least 0.15 m from the ring (centre within 0.26 m of the tip in xy and the
   tip within 0.05 m of the ring's height counts as on it) and from every carrier's rod (carrier body to its plate).
3. QUAD4 ends within 0.05 m of its rest height, disarmed alone or at LANDED, never airborne (> 0.15 m above its rest)
   when it is disarmed.
4. No fault on QUAD4's tracker (ENVELOPE FAULT, FLEET ABORT) in that window; later events (the twin's known post-landing
   tip-over, R1502) are scored separately.

Switch-off runs record what today's code does: QUAD4's tracker fault time, whether his planner keeps flying QUAD4
after the drop, the drop -> LANDED time. If an off run does not show his planner flying on, the twin does not
reproduce a2_2: the card is then inconclusive, not passed.

Falsifier: any bar failing in any switch-on run.

## Critic (15 Oct night, before any run): "rewrite the card", addressed in rev 2

1. The stale-reference fault also breaks GIVE UP after a drop: fixed under the switch (change 2); the card now says
   so for the switch off.
2. QUAD4's None hold makes `/fleet/landed` wait 30 s, and with the switch on that timeout could disarm QUAD4 in the
   air: change 3.
3. The rig-only release guard is reachable from a drop and lands QUAD4 beside plate 9 over the fallen ring: the guard
   stands down after a drop.
4. Rev 1's arms did not reproduce a2_2 (DROP at +2 s fired in DESCEND; a forced drop skips the capsize): DROP at
   +1.0 s, plus the RELEASE neighbour and an ATTACH_READY neighbour.
5. Bar 2 could not fail for the right reason: replaced by clearance bars (carriers, rods, ring, rest).
6. Bar 3's window was open: frozen from the drop to QUAD4's disarm.
7. The unit test mocked `_take_back`: added tests for the warm reference, the longer wait and the guard; the real
   take-back path is exercised by the twin runs (not a unit test yet).
8. Bar 1's timing was automatic: only the mux part is kept.
9. The out-step's tip passes ~0.18 m under the carriers' rotor plane: bar 2 measures it.
10. Non-partner newcomers out of scope (above).

## Result (rev 2 runs, 16 Oct ~00:05; tree 66a701c, lab PC)

| Run | Arm | QUAD4 after the drop | Drop -> LANDED | Bars |
|---|---|---|---|---|
| R1753, R1755 | off, drop in MATCH | tracker fault 1.06 s after; his planner flew on (DESCEND, ATTACH_READY); 0.53 m up at the end | 30 s (timeout) | (baseline) |
| R1757 | off, carrier release in MATCH | fault 1.05 s after; he flew on; 0.50 m up at the end | 30 s | (baseline) |
| R1754, R1756 | on, drop in MATCH | taken back on the drop's tick, out in 6.7-6.8 s; carrier >= 0.56 m, rods 0.44 m, ring >= 0.29 m; down at its rest | 14.1-14.3 s | 1-4 PASS |
| R1758 | on, carrier release in MATCH | as above (carrier 0.53 m, ring 0.30 m) | 14.1 s | 1-4 PASS |
| R1759 | on, drop in ATTACH_READY | taken back; down at its rest, no fault | 17.0 s | 1, 3, 4 PASS; **2 FAIL** |

The switch-off arm reproduces RIG-1015-a2_2 (fault, his planner flying on, the 30 s timeout), so the twin tests the
right thing. **As frozen the card is falsified by R1759's bar 2**: at the drop instant QUAD4 sat 0.45 m from a
carrier with its tip on the plate, which is the bottom hold's own geometry (0.45 m is the body clearance at the plate
with 0.55 m rods). Within 0.5 s the ring had fallen 1 m and every distance only grew (carrier 0.45 -> 1.0 m in 4 s);
the take-back made nothing closer. The bar was wrong for a drop that starts at the plate; per the working rules
R1759 stays scored against it, and rev 3's bar applies to later runs only.

## Rev 3 (16 Oct ~00:10, before the runs it scores)

Bar 2 for a drop that starts with QUAD4 at the ring (ATTACH_READY, LIFT_PROOF): from the drop until QUAD4 is 1.1 m
out, its body never comes closer to any carrier, and its tip never closer to any rod, than at the drop minus 0.02 m;
once the tip is more than 0.15 m from the ring it stays so. Bars 1, 3, 4 unchanged. Runs: `w4_drop_his_bottom_tb` x2,
`w4_drop_his_bottom_off` x1 (record).

## Result (rev 3 runs, 16 Oct ~01:05)

| Run | Arm | At the drop | Until 1.1 m out | End | Bars |
|---|---|---|---|---|---|
| R1779, R1780 | on, drop in ATTACH_READY | carrier 0.45 m, tip-rod 0.44 m | never closer; tip never back near the ring | down at its rest, landed 16.9-17.1 s after, no fault | 1, 2 (rev 3), 3, 4 PASS |
| R1781 | off, same | same | his planner flew the tip back toward the fallen ring | fault, 30 s timeout, QUAD4 0.52 m up at the end | (baseline) |

**Verdict.** Switch on: 7 runs, every bar passes except R1759's bar 2 as frozen in rev 2 (a bar that did not fit a drop
starting at the plate; scored as a fail, explained above). Switch off (4 runs): the a2_2 hazard every time (tracker
fault ~1.05 s after the drop, his planner flying QUAD4 on, toward the fallen ring in the bottom case, QUAD4 still
airborne at the 30 s timeout). Next: the reviewer; then Wesley decides the default, the rig use and the locked
rules' "Open" line.

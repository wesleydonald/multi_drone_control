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
| R1754, R1756 | on, drop in MATCH | taken back on the drop's tick, out in 6.7-6.8 s; carrier >= 0.56 m, rods 0.31 m [reviewer; 0.44 was the scorer's vertical rods], ring >= 0.29 m; down at its rest | 14.1-14.3 s | 1-4 PASS |
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
| R1779, R1780 | on, drop in LIFT_PROOF (0.25 s after CONTACT) | carrier 0.45 m, tip-rod 0.22 m [reviewer] | never closer; tip never back near the ring | down at its rest, landed 16.9-17.1 s after, no fault | 1, 2 (rev 3), 3, 4 PASS |
| R1781 | off, same | same | his planner flew the tip back toward the fallen ring | fault, 30 s timeout, QUAD4 0.52 m up at the end | (baseline) |

**Verdict.** Switch on: 6 runs [reviewer; 7 was a miscount], every bar passes except R1759's bar 2 as frozen in rev 2 (a bar that did not fit a drop
starting at the plate; scored as a fail, explained above). Switch off (4 runs): the a2_2 hazard every time (tracker
fault 0.97-1.03 s after the drop [reviewer; 1.05-1.06 s is the reference's age at the fault], his planner flying QUAD4 on, toward the fallen ring in the bottom case, QUAD4 still
airborne at the 30 s timeout). Next: the reviewer; then Wesley decides the default, the rig use and the locked
rules' "Open" line.

## Reviewer (16 Oct ~01:35): SUPPORTED WITH CORRECTIONS

Its independent re-score matches the card's mux, out times, carrier and ring clearances, rest and disarm heights,
faults and drop -> LANDED times. Corrections (applied above where marked):
1. Tip to the nearest rod: 0.31 m in the MATCH arms (R1754 0.315, R1756 0.313, R1758 0.315) and 0.22 m in the bottom
   arm (plates 7 and 11 are 0.225 m from plate 9), not 0.44 m: the scorer modelled the rods as hanging straight down.
   Re-modelled as free pendulums driven by the logged carrier motion, the closest approach is still at the release.
2. Six switch-on runs, not seven.
3. The switch-off fault came 0.97-1.03 s after the drop (R1753, R1755, R1781; R1757 1.05 s after the detected drop).
4. The nearest carrier dipped after the bottom drops (0.45 -> 0.71 -> 0.63 -> 0.99 m) but never below its drop value.
5. The "ATTACH_READY" drops fired 0.25 s into LIFT_PROOF, after CONTACT (R1759, R1779, R1780).
6. On this tree MATCH lasted 1.03 s, so "1.0 s into MATCH" fell 21-35 ms before his DESCEND; a drop partway through a
   long MATCH (a2_2: 1.32 s into a 5.3 s MATCH) was not flown. In R1758 the take-back came after DESCEND.
7. R1779 and R1780 also fail rev 2's bar 2: the bottom arm's pass rests on rev 3.
8. In the switch-off MATCH arms too (R1753, R1755) his planner set the tip down on the fallen ring from about +4 s.
9. On the rig (a2_2) the second LAND came 14.85 s and the DISARM 16.97 s after the drop.
10. **Twin evidence for critic F1:** in R1781 (switch off) his own GIVE_UP fired 4.4 s before LANDED; a TAKE_BACK was
    logged but no MUX:ours, and QUAD4 stayed 0.63 m up with its tip on the ring. With the switch off, a GIVE UP after
    a drop cannot hand QUAD4 over: the operator rule on the sheets does not work.
11. After the bars' window: QUAD4's rigid twin tether touches the floor at body z 0.60 m and is levered ~0.45 m along it
    ("still pressing down", R1758); tilt <= 3 deg, no tip-over (a twin landing artefact of the rigid tether).
12. Not exercised: slack tethers, undamped rod swings, a drop in DESCEND or early in a long MATCH, the rig's release
    guard (rig only, stands down after any drop), the 60 s /fleet/landed path. The release arm and the bottom
    switch-off arm are single runs.

What the claim may say (twin only): with drop_takes_back on, in three drop cases (end of MATCH x2, a carrier release
caught by the tilt rule x1, LIFT_PROOF with the tip on the plate x2 under rev 3, plus R1759 under rev 2), the take-back
takes QUAD4 on the drop tick, keeps it >= 0.53 m from every carrier and its tip >= 0.31 m (MATCH) / 0.22 m (bottom)
from the rods until 1.1 m out, and lands it at its rest 14-17 s after the drop with no fault; with the switch off the
twin reproduces a2_2. It may not say the card passed as frozen (R1759 falsifies rev 2), nor anything about the rig.

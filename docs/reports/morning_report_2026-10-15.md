# Morning report, Thu 15 Oct (lab 15:00-18:00)

**Verdict:** Thursday's attach flights are ready to fly on the new confirmation (twin passes, both critic rounds and
Tejen's five points fixed). Two things need you before 15:00: QUAD4's aim (A or B) and sending the FC request.

## Rig findings first (the 14 Oct bags, analysed overnight)

| Finding | Number | Why it matters |
|---|---|---|
| The new attach check would have accepted 14 Oct's one real catch (t6h_4) | proven 2.8 s after contact; the tip stayed within 2 mm of its contact point with the tether taut 100 % | that catch would have folded QUAD4 in; no false proof on the other flights |
| How far off the magnet is while holding over plate 9 | median 5 cm (3.8-5.7 by flight); inside the 60 mm disc's catch area 16 % of the time, inside and slow 8 % | the tip lands on the plate by chance |
| 90 mm discs on the same 14 Oct tracks | inside 35 %, inside and slow 15 % | about twice the chance per second: the most direct hardware fix |
| Tejen's tip trim at his 0.08 m/s learning gate | learns at most 1 cm in 7 s (the tip moves 0.16-0.20 m/s); at 0.20 m/s it learns the 4-5 cm offset in 2-4 s | his trim as configured would not have centred the magnet; Tejen agrees to try 0.20 above contact |
| A ring drop could not release QUAD4's coil after a confirmed attach | the drop's OFF was refused by our own latch | broke the locked drop rule; fixed |
| After a hold exit QUAD4 climbed with its coil ON | an edge catch nobody confirmed would have lifted plate 9 | fixed: the coil goes OFF on the climb back |
| QUAD1/QUAD2's FCs think they are tilted 60-84 deg at about 12 deg | 11 of 46 launches refused to arm on 14 Oct | the FC request (small_angle, accelerometer calibration) fixes it; an FC accelerometer log would show which |

## What was done overnight

| Commit | What |
|---|---|
| bfbd7a9, 305e50c | Plates stay clockwise; every typed and shown angle is clockwise too (carriers typed 90,330,210; QUAD4 at 270). Same argument names, meaning flipped, as you chose: an old command typed the old way puts the carriers on the mirror-image plates, and T2 then shows a SLOT OFFSET error naming it |
| eacab58, f035625, (rev 4) | The attach check (contact, his lift, the tip staying with the plate on a taut tether), the bottom hold, the coil OFF on the climb back, the drop releasing QUAD4, a dry-hold switch (coil held OFF for a whole flight), Tejen's five points |
| a73d9a4, f9d0b72, 33ceec4 | The console (READY instead of the 2-min wait; buttons in flight order; SPACE = DISARM; who is in the air; warns if his planner misses his TAKEOFF), T2 events only, the RViz panel rules (compile-checked only: check it in test 1) |
| 3297746, dade70d | Arming and landing fixes (his ARM first; refused drones named; QUAD4 disarms alone after a take-back) |
| 657fee1, f035625 | Scoring tools: attach_window (how well the magnet sits over the plate), proof_replay --catch (the card's bars between flights) |
| Twin | R1718 PASS, R1719 FAIL (found the lost tether length), R1720 PASS (the clockwise layout end to end), R1721 PASS (the final tree; the bottom hold is rig-only, not exercised in the twin) |

## Decisions for you

1. **QUAD4's aim for the attach flights.** A: t6h_4's target with his trim off (`contact_dx 0.031 / contact_dy 0.010`),
   the one setting that has caught. B: his trim learning up to 0.20 m/s (Tejen OK'd it as an experiment). The sheet
   flies A; B is one variable on the sheet.
2. **Send `docs/rig/fc_request.md`** to whoever configures QUAD1-3 (QUAD4 with Tejen): it fixes the arming refusals.
3. **Send Tejen the updated package** (`~/tejen_review_package_2026-10-15.zip`): his five points fixed, the wiring and
   the replay checks he asked for. Read its README first.
4. **Thursday's order** (the sheet, `docs/rig/thu_2026-10-15.md`): props-off console and RViz check; the confirmation
   bench by hand (the negatives with the coil held OFF); one dry-hold flight; then up to four attach flights, each
   scored with `proof_replay --catch` before the next.
5. **For next week (your supervisor's list):** 90 mm discs first, then weight on the magnet, then the 1 m tether; 45 deg
   carriers only with stronger carrier magnets (about 1.7x the sideways pull: the detach Tejen saw).

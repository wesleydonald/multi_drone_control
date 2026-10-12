# Morning report, Mon 12 Oct (overnight 11-12 Oct)

**Verdict:** the overnight claim batch flew all 41 runs on Wednesday's exact install (rig-wed ba40bd0) with no
infrastructure re-fly. Every hover line that R11a and R11b depend on is supported. The combined claim as a whole is
NOT SUPPORTED: the circle take-back and the push abort fail their frozen bars. The tilt cut works in every injected
case, but its table is 1 of 2 on T1 and F1. One real problem was found: a take-back race after a cut, which must be
fixed before the cut is ever switched on, on the rig or in the twin.

## 1. Rig findings first

| # | Finding | Why it matters on the rig | Status |
|---|---|---|---|
| 1 | **Wednesday's hover path holds on the exact install.** Hover attach x5 (R1644, R1645, R1650-R1652): one contact each, proof 0.72-1.08 s, throttle step at most 0.6 %, ring at most 2.4 deg, every clearance at least 0.21 m. Hover take-back, the no-catch hover (3 contacts rejected each time), LAND in transit and LAND during the proof all pass. The R11a rehearsal R1658: no contact, his give-up at 135.8 s, our take-back clean (rod 0.22 m, ring 1.7 deg), landed. | R11a, then R11b, fly this path. | Supported (reviewer). |
| 2 | **His descent can take up to 20 s to make contact.** From the start of his descent to first contact: 4.7-20.3 s in the 9 catch runs; only 2 of 9 within 10 s (R1650, target 1 cm high: 20.3 s). His give-up is 30 s after the first descent. | On Wednesday a slow descent is normal, not a fault. Don't call a stop before his 30 s. | Report only (TB15, no bar). |
| 3 | **The push abort came 6 mm too close.** R1654: during the push, his rod passed 0.094 m from a carrier, against the 0.10 m bar. The abort itself and the re-attach worked. | Only matters if the push abort is flown on the rig. It is not on Wednesday's sheet. | NOT SUPPORTED as frozen. Decision 1. |
| 4 | **A cut QUAD4 can still be "taken back".** In 3 of 9 cut runs (R1662, R1673, R1678), the fleet's LAND reached our planner a millisecond before the tilt fault did. So it published a take-back, and his planner ended in TAKEN_BACK. Nothing flew: the mux stayed cut and sends DISARM only. This comes from the new take-back code on this tree, not from any switch. | The mux's cut would be the only thing stopping a take-back of a tumbled QUAD4. The cut is off on Wednesday and 21-22 Oct, so neither is affected. | Fix needed before the cut goes on (28-29 Oct). Decision 3. |
| 5 | **A real tumble under his controller in a normal hover (twin).** R1673, F1: about 5 s after his takeoff QUAD4 climbed to 1.82 m (1.06-1.33 m in the other runs). Its tilt grew in a slow oscillation (19, 23, 33, 40 deg) while it drifted 0.4 m, until the cut disarmed it at 65 deg about 0.4 m up. It is 1 of his 42 takeoffs tonight: in the other 41, QUAD4's tilt peaked at 6.1 deg or less in the 10 s after takeoff. His loop and solve times match the healthy runs. R1681 re-flew it alone: no repeat (takeoff peak 6.0 deg, 1.32 m; clean hover attach and landing). | His real takeoffs have been fine many times, so this may be twin-only. But if it ever happens on the rig, it happens with the cut off. | Cause not established. Decision 5: ask Tejen. |
| 6 | **His attach on a moving plate is marginal (twin).** In the first circle wave his match-velocity gate stalled in 3 of 4 runs. In R1656 (full M1 on the circle) his ready-to-attach lost alignment twice, then he gave up. The circle attaches that did happen are fine (R1646, R1647: proof 0.64/0.93 s, ring 2.5 deg). | The rig refuses an orbit with his attach anyway (hover only). It matters for any later circle run with him. | For the thesis. |
| 7 | **The landing tip-over** (rigid twin rod, scored on its own) came in 4 of 16 landings with QUAD4 attached (R1644, R1665, R1670, R1671). Both C2 runs ended in a FLEET_ABORT through it. | Twin artefact; the rig tether goes slack. | Report only. |

## 2. Overnight results (one row each in results/registry.csv; corrections as `-corr` rows)

| Table | Runs | Result |
|---|---|---|
| Combined claim (card rev 10 §6.3) | R1638-R1654 | **Reviewer: NOT SUPPORTED overall.** Supported: hover attach, hover take-back, no-catch hover, LAND during the proof. With corrections: circle attach (n=2), circle no-catch (no attach only), LAND in transit (step under 2 deg in 1 of 2). Not supported: circle take-back (R1639 fails three bars on its LAND path; only R1640 reached a give-up) and push abort (R1654). Verdict in card §6.3a. |
| Beside the table | R1655-R1659 | Full M1 hover R1655: end to end. Full M1 circle R1656: no reattach (see 1.6). Full M1 circle without a catch R1657: 3 catches rejected, give-up, clean take-back. R11a hover R1658 and circle R1659: clean take-backs. |
| Tilt cut (card rev 4) | R1660-R1678 | **Reviewer: not supported as a 2-of-2 table.** Supported: T0 (the baseline), F6c, C3. Supported with corrections: F6b, C1, C2 (up to the tip-over), N1, N2. Not supported: T1 (R1662, the take-back race of 1.4), F1 (R1673, the tumble of 1.5), N3 (between the bars, the same race). Every timing bar in every cut run passes: fault 0.23-0.25 s after the injection, cut and disarm within 0.022 s, fleet LAND within 0.006 s. |
| Diagnostic (exploratory) | R1679-R1681 | The first circle wave (R1638-R1641) re-flown on 2 slots: his gate passed in 1.6-1.9 s, 3 catches rejected and a clean take-back in both. Carrier reference age p99 0.100-0.108 s, against 0.120-0.127 s in wave 1. That is consistent with load in the batch's first wave, but later 4-slot waves were fine, so the cause is not established. R1681 (F1 alone): clean, no repeat of R1673's tumble. |

Corrections I made from the reviewers: I had written that his MPC logs "armed/takeoff: False" after a cut. It does
not: it logs the disarm and shuts down. Five tilt rows now carry the right wording. Also, the tilt card's §9.1 names
m1-tiltcut4 as the tree, but these runs flew on rig-wed ba40bd0.

## 3. Committed overnight (rig-prep; nothing pushed, no code changed)
Registry rows R1638-R1681, the corr rows, card §6.3a, SESSION_STATE. The lab scripts are in the job folder, not the
repo.

## 4. Decisions for you
1. **The combined claim.** Recommended: claim the hover lines now (they are what Wednesday flies). Re-fly the circle
   take-back, circle no-catch and the push abort on week5 after decisions 2 and 3, as a new table with the same bars.
   The alternative is to re-fly the whole table now on rig-wed, which spends a lab night on lines the rig doesn't fly
   this month.
2. **The first-wave confound.** Recommended: treat wave 1 as suspect and, for any re-fly, start the batch with hover
   arms so the circle arms don't share the first wave. Its rows stand as the record.
3. **The take-back race after a cut** (1.4). Recommended: on week5, our planner handles a tilt fault before a LAND
   take-back, and the LAND take-back skips a drone whose mux reads cut. This touches a fault path, so it gets a critic.
   Until then the cut stays off.
4. **The tilt cut's defaults.** C1 (tilt cut + not-live + FC disarm LAND) passes 2 of 2 and C3 2 of 2. Make nothing
   default before decision 3 and Tejen's OK.
5. **R1673's tumble.** Ask Tejen whether QUAD2/QUAD4 has ever oscillated after his takeoff. Ask for a log of one of his
   takeoffs if he has one.
6. **Still open from before:** send the Tejen note (docs/tejen_note_2026-10.md; add item 5's question). Delete
   week1-ready-for-testing (`git branch -D`, yours).

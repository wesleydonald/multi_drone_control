# Morning report, Tue 13 Oct (overnight 12-13 Oct; supervisor meeting today)

**Verdict:** week5 flew 32 runs on the lab last night and is ready for Wednesday. Every hover line Wednesday depends on
passed its rehearsal, both claim lines that failed on the old tree pass on week5, the take-back race fix is clean in all
four of its arms, and the first Tier A pair came out decisively for the measured pull. Two things need your word before
Wednesday: the order of the day, and Tejen's controller tumbling QUAD4 in the twin for the second night running.

## 1. Rig findings first

| # | Finding | Why it matters on the rig | Status |
|---|---|---|---|
| 1 | **Tejen's controller tumbled QUAD4 again in the twin** (R1683): 1.6 s from level to 70 deg at the start of his lift after the pickup, then a crash beside the mat. The other three of his missions last night were clean, as were 41 of 42 the night before (that one tumbled 5 s after takeoff). His loop timing and kT estimate are normal in both. | His rig pickups have worked many times, so this may be twin-only; but on Wednesday the tilt cut is off, so nothing would catch it there. | Cause not established. The question is in the Tejen note (item 6) with a request for his logs. **Send the note today.** |
| 2 | **Wednesday's order.** You asked to start with the combined flights and skip the QUAD4 checks, the pack test and the releases. Three props-off items can't be skipped: the contact height W (the launch refuses without it), the catch radius R from about 10 hand catches (our check confirms his attach against it), and the 10-min load test (the radio disarms QUAD4 after 0.5 s of silence; the laptop failed it under load). About 25 min, no Tejen needed. The releases are the twin's first scored predictions, not a check that release works. | Order of the day. | **Your word:** props-off (W, R, load test) -> R6 -> R7, R8 -> R9 -> the two releases if time -> Thursday: the circles, the spring scale, retries. I rewrite the sheet on your yes. |
| 3 | **The push-abort clearance bar measures the push, not the abort.** Five runs, both push directions: his abort and re-attach worked every time; his rod came 8.0-9.5 cm from the carrier on the pushed side every time (bar 0.10 m), because a 3 N, 1 s push moves the ring and its carriers while QUAD4 holds its place over the plate. | Not on Wednesday's sheet. For the week5 claim table. | **Decision:** re-define the clearance for the push test, or push with less than 3 N. |
| 4 | **The circle reattach in the twin is marginal, not broken:** 1 of 4 last night and 0 of 1 the night before on the Wednesday tree, against 6 of 6 earlier; no flight code differs. Every miss is his controller inside its 2.5 cm gate only 66-82 % of the time on the moving plate (93-98 % in the successes). The mat-start circle attach on the same tree works. | The rig refuses a circle with his attach, so Wednesday is unaffected. | **Decision:** spend lab time on it (his lead or gate) or report it as marginal (6 of 10). |
| 5 | **Wednesday's rehearsals on week5 all pass:** R11a (R1682), LAND in his transit with the race fix's 0.2 s take-back (R1684), the full M1 on hover end to end (R1685), the W4 and W5 same-tree checks inside their frozen bands (R1686 4.6 deg, R1687 7.4 deg). The hover attach itself is 0 of 1 because of finding 1. | Wednesday's lines. | Done. |

## 2. Overnight results (week5, lab PC, 4 slots; one registry row each, R1682-R1713)

| Group | Runs | Result |
|---|---|---|
| Wednesday rehearsals | R1682-R1687 | 5 of 6 support (above); R1683 the tumble. |
| Push abort, both directions | R1688, R1689, R1692, R1693 | abort and re-attach 4 of 4; rod clearance 8.0-9.5 cm in all four (finding 3); two ended in the twin's landing tip-over. |
| Full M1 on the circle | R1690, R1691, R1700, R1701 | 1 of 4 reattached (R1700: contact on the first descent, proof 0.74 s); the three misses are his alignment on the moving plate (finding 4); our circle take-back met every bar in all three. |
| The claim's failed circle lines | R1694, R1695 (no-catch), R1698, R1699 (LAND take-back) | **4 of 4 pass** on week5: three catches rejected each time; the LAND take-back 0.13-0.17 s after LAND, rod 0.25 m, ring 3.0-3.2 deg, down in 17 s (R1639 on the old tree read ring 8.6 deg, rod 0.07 m). Exploratory; the week5 claim table re-flies them. |
| The 0.3 s lead | R1696, R1697 | attach on the 2nd and 3rd try, proofs 2.75 and 1.44 s: slower than lead 0 on the same tree. Lead 0 stays. |
| Tilt cut with the race fix | R1702, R1703 (T1), R1704 (N3), R1705 (C1) | **4 of 4 clean, no take-back published** (the old tree published one in 3 of 9 cut runs). Fault caught in 0.23-0.24 s, disarm within 15 ms. |
| Tier A pair T3 (unannounced hover release) | R1706-R1711 | **measured pull 5.4 / 6.2 / 6.1 deg against planned pull 17.1 / 17.2 / 17.0 deg.** Resolved with measured < planned under the frozen rule (sign, no overlap, 11.2 deg against a 1.6 deg bar). The twin's planned-pull unannounced arm had never flown before. Reviewer: ran overnight (its verdict is appended below when in). |
| The L5 twin pair re-flown on one tree | R1712, R1713 | 6.3 / 6.0 deg; replaces R1140/R1145 in T8b. |

Landing tip-overs (the rigid twin rod): R1688, R1692, R1697, as before; scored separately.

## 3. Also done since yesterday's report
- **Repo tidy (your request):** one tree on week5 (code, docs, registry, rules); docs sorted into rig/, tejen/, reports/, analysis/ and archive/; 303 old configs and 19 one-off tools into old/ folders; `general/`, `results_archive/`, `dissipative_verify/`, two local notes and the supplementary bags retired or archived; every acados solver under `generated/`. Full gate passes. Every branch and tag backed up on GitHub under `backup-2026-10-12/`.
- **The other chat (lanes 5-6):** the Wednesday red-team (its fixes are on the sheet), the thesis assembled on thesis-rig (122 pp, Ch2-Ch4 with four figures and the T7/T8 tables after a reviewer), the Wednesday-night pipeline (`tools/rig_night.py`), the counter-swing finding, the Tier A card.
- **Offline analyses (P6):** every rig release on the frozen metric; the rig's no-event tilt floor is 5-6 deg; the slip detector has no false alarms over the 8 Oct bags; the double-count hypothesis is not in the logs.
- **Thesis lint:** no run ids, flight names or SHAs in prose.
- **Supervisor brief:** `docs/reports/supervisor_brief_2026-10-13.md`.

## 4. Decisions for you
1. **Wednesday's order** (finding 2): yes to the reorder, or the sheet as it stands.
2. **The push-abort clearance bar** (finding 3).
3. **The circle reattach** (finding 4): lab time or "marginal".
4. **Tier A's announced half** (A3/A4): the measured soft release does not unload the leaver, so the critic expects that pair unresolved as written. Fly it as written and label it, or make the leaver fly the planned unloading term first (flight code, critic).
5. **Send the Tejen note** (`docs/tejen/tejen_note_2026-10.md`), with item 6.
6. **At the meeting:** the five items in the brief (due date and deliverables, the three RQs, the AI policy, Tejen's credit, the page budget).

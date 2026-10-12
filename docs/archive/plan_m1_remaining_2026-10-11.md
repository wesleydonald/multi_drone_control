# Plan for the remaining combined-M1 work (11 Oct; workflow wf_e76df7b1-616: three lenses, judged, stress-tested)

# Adversarial review of the plan for the remaining combined-M1 work

## Verdict

The backbone holds: all code lands before one joint claim batch, critic round 2 and a smoke test run before that batch, the Thu 15 gate stands, and the rig installs are pinned. The plan was written before wave A ended, and it has three structural faults.

1. **R1582 is scored with a rule the tilt card never froze.** The plan scores LAND separately on the tilt card, but rev 3 has no such line. Read as written, F6b stands at 1 of 2.
2. **The hardest merge sits on a rig-day morning.** It is the co-move rewrite of the take-back against the tilt hook, and step 23 does it on Thu 15 morning.
3. **The rig switch decision lumps three different switches together** (D6).

There are also some decisions scheduled without Wesley's word, bars written after the arms have flown, and laptop-load breaches on rig days. The corrected plan keeps the shape and fixes those.

Checked read-only at 12:35:
- Wave A is complete: R1586 F6b exit 0 PASS in 24.0 min; 8 runs in 35.5 min. The lab is idle.
- The rig-prep tree differs from week4 only in docs.
- week4 is an ancestor of m1-rig (30 commits) and of ffa41ef.
- m1-tiltcut3 is ead6afe plus cherry-picks (20626d3, ffa41ef). It does not descend from m1-rig.

## Findings (most severe first)

| # | Severity | Finding (evidence) | Fix |
|---|---|---|---|
| 1 | High | **The state has moved.** R1586 passed and the lab is fully idle. Step 2's "top up beside wave A, `--no-sync --no-build`, wait on the PID" is moot. With `--no-sync`, lab_batch records the laptop SHA without re-syncing the lab tree. | Step 2 becomes an ordinary batch on ffa41ef with sync and build, after checking `who`, load and containers. |
| 2 | High | **R1582 is re-scored with an unfrozen rule.** The tilt card rev 3 F6b row confirms on "no drone-3 disarm" and falsifies on "a drone-3 disarm". R1582: weld 114.18 s, LAND 122.18 s, `/fleet/abort` at 129.03 s ("drone 4 disarmed unexpectedly", 87.8 deg). That is the TB14 landing class. "LAND scored separately" is a ruling on the combined card (10 Oct); rev 3 has no such line. | Score R1582 by the text as written: falsifier met, by a landing tip-over, trace attached. Give the artefact reading beside it for the reviewer. Put the landing rule into tilt rev 4, for later runs only. By rev 3 §6.3, F6b is 1 of 2, so H1, F6a and F6a2 fly. |
| 3 | High | **D7 contradicts step 9.** D7 keeps partner_stuck_guard and taken_back_self_land, but step 9 flies H1/F6a/F6a2 only if F6b failed. That leaves untested code in the rig tree, against "keeping code that did not clearly beat the baseline: default delete". | Fly H1, F6a and F6a2 in wave B whatever F6b scores (3 runs, about 0.25 h of lab). Decide D7 on their results. |
| 4 | High | **The hardest merge runs on Thu 15 morning.** m1-comove rewrites `_take_back` (m1-rig L1641-1709) where the tilt hunk (~1646, "overrides an earlier take-back") sits. The cherry-pick lineage also replays the known test_partner_contact.py conflict. The H3 race tests would be written by the merger, 3.5 h before the laptop goes to the rig. | Make m1-tiltcut4 a **merge** of m1-rig 0210a27 into ffa41ef (resolve the test conflict once, today). Branch **m1-comove from m1-tiltcut4** (switches off gives the 0210a27 launch graph), so its author writes the H3 race tests with the code. Merge wave B fixes into it before TB6. Thursday then only fast-forwards m1-rig and adds the scorers (tools), the loader and the band split. week4 → m1-rig → candidate stays a straight line. |
| 5 | High (rig) | **D6 lumps three switches of different kinds.** warm_fault_not_live adds no disarm authority; it removes the stuck take-back latch (T0 R1580/R1584; F6b R1586 runner PASS). With it off on 21-22 Oct, that stuck state ends only with DISARM: QUAD4 falls from 2.3-2.9 m and the ring drops. tilt_cut and fc_disarm_land are the new authority over Tejen's drone. "Typed if W1/W2 pass that morning" also makes the first real-radio test and the first flight the same day. | **Pushback (once):** split D6. (a) warm_fault_not_live becomes a 21-22 Oct candidate after its reviewer. Add a one-variable arm C3 (nominal m1c_hover with it alone): x1 in wave B, x2 in the claim. (b) tilt_cut and fc_disarm_land stay off on 21-22 Oct; W1/W2 are evidence only; they go on from 28-29 Oct if W1/W2 pass and Tejen agrees. |
| 6 | Medium (rig) | **The D9 fallback leaves R1565's world-fixed circle hold in the rig tree.** Only the card's order keeps an orbit off the launch line. | Extend TB16: with `real:=true`, partner_contact refuses any `load_traj` other than hover until the co-move is in the reviewed tree. It does not depend on the co-move, so build it either way. |
| 7 | Medium (rig) | **The co-move would first run on the rig by accident**, inside R11c after a failed catch. Separately, rig card §7 (R11b x2, R11c x2) falls short of "Repeats: 2 in Gazebo, **more on the rig**". | Add **R11a-circle** (`contact_dz +0.03` on the circle, a planned no-catch) before R11c, rehearsed in TB6 and in the claim batch. Either fly R11b at least 3 times, or call 21-22 Oct a first look whose claim completes on 28-29 Oct (D21). |
| 8 | Medium (real life) | **The twin has no rotor wash.** B1's escape passes over the carriers with the tether tip 0.15-0.3 m above their rotor plane and the body about 0.8 m above. The downwash on the carriers and the tether cannot be seen in Gazebo. | TB5 also reports overflight time over the carrier discs and the minimum vertical gap. Say so in D1 and list it as a rig finding. B2 stays the rig alternative if the replay shows more than a brief overflight. |
| 9 | Medium (Wesley's word) | **TB13 and TB16 are built without his word.** TB13 changes the hover arm m1c_land_transit (`traj: hover`), so "hover keeps today's path" is not true. TB13 was only proposed (morning report §5 item 5); his five answers on 11 Oct do not cover it. TB16 changes what the launch accepts. | Add both to today's questions (D19, D20). Add m1c_land_transit_2 to the claim. |
| 10 | Medium (bars) | **Bars are written after the arms are seen.** Step 27 writes the rev 10 claim section (lead030 scoring, the _2 arms) and the rev 4 neighbour bars after TB6, wave B and the smoke runs. Friday morning is also overfull: critic response, fixes, rebuild, smoke, scoring and bars, all before a 13:00 batch. | Write all of rev 10 in step 6, and the rev 4 neighbour and C3 bars with their configs (step 4). Step 27 only names the SHA. |
| 11 | Medium | **Only nocatch_orbit gets a second repeat.** lead030 (circle take-back) and land_transit (TB13) also change path. | Add m1c_lead030_2 and m1c_land_transit_2 to the claim (+2 runs). |
| 12 | Medium (laptop) | **Step 8 runs Gazebo on the laptop while steps 3, 4 and 7 build, test and prebuild there.** Its gate waits for the builds only. It also duplicates the R11a rehearsal in steps 17 and 28. | Drop step 8. Fly the R11a hover rehearsal on the lab in wave B (m1-tiltcut4, switches off). No laptop Gazebo this week. |
| 13 | Medium (rig days) | **Lab batches and fixes on rig days.** lab_batch runs on the laptop (config check, reservations, rsync back). Step 20's fix loop spans Tue evening to Wed evening, and step 21 allows "lab batches remotely". Tuesday's load test needs nothing else on the laptop. | Nothing on the laptop while the rig stack is up: no lab_batch process, build, test or scoring (Tue load test, Wed 14, Thu 15:00-18:00, Tue 20, 21-22 Oct). TB6 must end before the load test. The fix loop pauses until the stack is down. |
| 14 | Medium | **Step 33 assumes week4 stays an ancestor of the final SHA.** A Wed/Thu rig fix committed on week4 would turn the fast-forward into a merge that changes the claim tree. | No code commits on week4 until step 33. A rig fix lands on the candidate before the Thu gate, or waits until after 22 Oct. |
| 15 | Low-med | **Step 3 commits before D5 is answered.** Wesley is at the machine (SESSION_STATE 12:40). | Put D5 in the first AskUserQuestion batch. Until he answers, leave work uncommitted (lab manifests carry the diff; the runs count as exploratory only). |
| 16 | Low | **Missing items:** (a) the 11 Oct defaults (contact_still on, the circle take-back moves with the ring) are not in the CLAUDE.md locked list; (b) saving the plan to docs/ and presenting it (SESSION_STATE to-do); (c) the daily morning report; (d) the loader has no worktree (m1-score is tools-only); (e) nothing waits for Tejen's OK; (f) no rule for 21-22 Oct if F5 is not met. | Steps 0, 14, 12, 19 and 35 below. |
| 17 | Low | **lead030 in TB9a flies the failing circle take-back and almost never makes contact** (R1525, R1543, R1566), so it says nothing about contact_still. | Drop it from wave B; it flies in TB6. |
| 18 | Low | **74 m1-rig configs hard-code** `params_file /home/wesley/multi_drone_control/configs/rig/...`. On the lab this resolves inside the synced worktree (CTR_REPO); on the laptop it resolves to rig-prep. | The `traj_hold_s` 11 and 17.5 arms get their own params files in m1-comove and fly on the lab only. |
| 19 | Low | **"One arm with the band split on"** is no evidence for a change in contact acceptance, and it splits the twin table (sphere) from the rig (band). | If D17 is on, put it in the rig-twin claim profile so every contact arm flies it. Otherwise off everywhere for 21-22 Oct. |
| 20 | Low | **The W-line checks belong to the week4 merge record**, with their 10 Oct bars (R1454-R1460, reviewer SUPPORTED), not to rev 10. | Score them against those bars and record them on the decisions.md merge line. |
| 21 | Low | **D3 is a routine call.** SESSION_STATE already says "card + critic". | Run the critic; Wesley may waive it. Do not spend a question on it. |
| 22 | Low | **The clock-time Date column reads as a timetable** (CLAUDE.md: "No day deadlines"). | Show the earliest start and the gates. Only lab days are fixed. A slip moves the flight; it never shortens a review. |

## The corrected plan

**Verdict.** Use plan 2's evidence-first order with plan 1's pinned installs and plan 3's one-worktree-per-change rule, with these corrections:
- One straight lineage: week4 → m1-rig → m1-tiltcut4 → m1-comove → candidate.
- Every bar frozen before its arms fly.
- R1582 scored as written, so H1, F6a and F6a2 fly.
- The rig switches split: warm_fault_not_live is a candidate; the cut stays off on 21-22 Oct.
- An orbit refusal on the rig until the co-move is reviewed, and R11a-circle before R11c.
- No laptop load while the rig stack is up.

| # | Step | Who | Machine | Gate before | Done when | Est. | Earliest |
|---|---|---|---|---|---|---|---|
| 0 | Save this plan to docs/ (rig-prep). Refresh SESSION_STATE: `Updated:` line, wave A done, lab idle. Add the 11 Oct defaults to CLAUDE.md Locked decisions. | main | laptop (docs) | none | Committed with his OK | 0.3 h | Sun, now |
| 1 | First AskUserQuestion batch: D1, D2, D4, D5, D6, D7, D9, D19, D20. | main | none | none | Answers in decisions.md the same session | 0.2 h | Sun, now |
| 2 | Score R1582 and R1584-R1587 by rev 3 as written. T0: its five items. F6b: R1582's falsifier met by a landing tip-over, both readings in the row. Count tip-overs for TB14. Rows say "first look on ffa41ef; the claim moves to the converged tree". Run /report. | main | laptop (offline) | none | 5 rows appended with LF | 1 h | Sun |
| 3 | Batch B0 on ffa41ef with sync and build: C1, T6, T7, T7b, T8 (their bars do not depend on the attach path). Read T6 from the tip (z + 0.59) until tip_metrics is fixed. | main | lab, 4 slots | `who`, load, containers clear; fresh `env -i` shell | 5 runs back | ~0.5 h lab | Sun |
| 4 | m1-tiltcut4 = merge of m1-rig 0210a27 into ffa41ef; fix the stale texts. Tilt rev 4 draft: the landing rule (later runs only), the TC-08 neighbours and C3 with their bars. Build (nice, 2 workers), pytest, `gate --quick`, launch gate (switches off identical to 0210a27), tc configs against the m1c bases, prebuild. | helper T | laptop (no Gazebo) | D5 before the commit | Green; launch graph identical | 2 h | Sun |
| 5 | Rebuild and pin the m1-rig install at 0210a27 for the Tuesday rehearsal and Wednesday Part B. Nothing else is built in that worktree until Wed evening. | main | laptop | Step 4's build done | SHA in SESSION_STATE | 0.3 h | Sun |
| 6 | Card rev 10, complete except the SHA (note A). | helper drafts, main edits | laptop (docs) | Step 1 (recommended options if no answer) | Committed before any TB6 run | 3 h | Sun |
| 7 | Wave B on m1-tiltcut4 (note B). | main | lab, 4 slots | Step 3 ended; step 4 built; rev 4 bars written | ~23 runs back | ~2.3 h wall | Sun pm |
| 8 | Scorers into tools/ in worktree m1-score (note C). | helper S | laptop (niced) | D5 | Tests pass; logged numbers reproduced | 4.5 h | Sun evening |
| 9 | Critic on the rev 10 take-back (with TB13 and TB16), under ## Critic, then the response. | critic, then main | laptop | Step 6 committed | Challenge and response in the card | 2 h | Sun evening |
| 10 | Score wave B. Tail: T6b only if T6 never passes 60 deg; one re-fly per fixed arm; each void re-flown once. Tilt fixes go on m1-tiltcut4. Add `fake_mocap --tilt`. Collect the D7 evidence. | main, helper T | laptop; lab for the tail | Step 7 back | Rows logged; fixes committed | 4 h | Mon |
| 11 | m1-comove from m1-tiltcut4: TB3 (per D1), TB13 (D19), TB16 including the `real:=true` orbit refusal (D20), H3 race tests, contract doc. Merge step 10's fixes into it before TB6. Gate, planner tests, prebuild. | helper C; main reviews | laptop (niced) | Step 9's response in the design | Green; committed | 8.5 h + 1.5 h | Mon (overnight OK) |
| 12 | m1-loader (from m1-tiltcut4): the OCT-3 loader and its refusals; band split pre-built behind a default-off switch. | helper S | laptop | D15 | With the argument empty, 0 launch cases change | 5.5 h | Mon to Tue am |
| 13 | Wednesday paperwork: WED-1 Part A/B, WED-2 form, TC-15 (a)-(e) ((f) from Betaflight or deferred), WED-6, WED-7, Thursday lines. One Tejen note (TEJ-1 + TC-14) with explicit asks for his OK. | helper W; main checks every flag on both trees | laptop (docs) | none | Committed; Wesley notified | 4.5 h | Mon |
| 14 | Morning report every day: rig findings first, table, verdict, decisions. | main | none | none | Given | 0.5 h/day | Daily |
| 15 | TB5 desk replay of R1565, R1554, R1566, R1549, R1543, plus R1565 phase-shifted. Report body and rod-segment clearance, overflight time and the minimum vertical gap. | helper C | laptop | Steps 8 and 11 | ≥ 0.15 m everywhere, or a fix before any lab time | 2 h | Tue am |
| 16 | Wesley reviews the Wednesday block, form and Tejen note; adds the alias exports; answers D10-D16. | Wesley | none | Step 13 | Approved | 1 h | Mon eve / Tue am |
| 17 | TB6 on m1-comove (note D). | main | lab, 4 slots | Step 15 clean; step 10 tail ended; rev 10 frozen | ~9 runs back, finished before step 18 | ~1.5 h wall | Tue am |
| 18 | Desk rehearsal (WED-3) on the two pinned installs, then the F1 load test part 1: at least 10 min with nothing else on the laptop (no lab_batch, no helper builds). Gap script committed to tools/. | main + Wesley | laptop, rig stack | Steps 16 and 17 ended | Every line read back; gaps against the 5.1 bars | 4 h | Tue pm |
| 19 | Send the Tejen note. Record his answers on our fork, the coil rule, switches over his drone, and R10a logging. | Wesley | none | Step 16 | Sent; answers are a gate for step 35 | none | Tue |
| 20 | Score TB6, then the fix loop (one run on the failed arm per tweak), paused while the rig stack is up. If B1 fails, put fallback (a) to Wesley. | main, helper C | laptop with stack down; lab | Step 17 | Verdict on the co-move take-back | 3 h | Tue eve, then Wed eve |
| 21 | Wed 14 rig day. Part A on week4. Part B on the pinned 0210a27 install in a fresh terminal. No laptop load while the stack is up. | Wesley flies; main reads each log | rig | Steps 18-19 | Every line has a value or a reason; bags saved | 8 h | Wed 14 |
| 22 | Post-rig analysis (as planned), rig findings first; D17 values proposed. | main, helper T | laptop (stack down) | Step 21 | Report given | 2.5 h | Wed eve / Thu am |
| 23 | **Thu 15 gate.** m1-rig fast-forwards to m1-comove if TB6 passed; otherwise to m1-tiltcut4 plus the orbit refusal. Merge m1-score and m1-loader. Band split per D17 (if on, in the claim profile). F1-new only per D16. Any rig fix from Wednesday lands here. Run the full gate. | helper T merges; main reviews; Wesley OKs | laptop (free to 15:00) | Steps 10, 20, 22; D5, D16, D17 | Candidate SHA; all green | 2.5 h | Thu 15 am |
| 24 | R10a, and the 3-min ring hover (OCT-12) if D13 is yes. | Wesley + Tejen | rig | Step 19 | His values logged | 3 h | Thu 15:00-18:00 |
| 25 | Rig-port critic round 2, part A (code) on the candidate; findings applied. | critic, then main | laptop | Step 23 | GO, or GO WITH CHANGES applied | 3 h | Thu eve |
| 26 | Smoke on the candidate: m1c_nocatch_orbit, tc C2, m1c_hover, m1_full_orbit. | main | lab | Step 25 applied | Clean; final SHA named | 0.5 h wall | Fri am |
| 27 | Name the SHA in rev 10 and tilt rev 4 (bars already frozen). | main | docs | Step 26 | Committed before run 1 | 0.5 h | Fri |
| 28 | **Joint claim batch** on the final SHA (note E). | main, detached | lab, 4 slots | Step 27; lab clear; fresh `env -i` shell | All runs back | ~5 h wall | Fri pm |
| 29 | Score both tables with the committed scorers. Add the TB15 proof-time distribution; score LAND separately; score the W lines against their 10 Oct bars into the merge record. Run /report. | main + helpers | laptop | Step 28 | Both tables in the cards with run ids | 3.5 h | Sat 17 |
| 30 | Two reviewers in parallel (combined M1 and tilt); responses; re-flies on the same tree only. | reviewers, main | laptop; lab | Step 29 | Verdicts in the cards | 5 h | Sat-Sun |
| 31 | In parallel with 30: QUAD4 record; rig card update (note F); combined-day sheet (OCT-9); critic round 2 part B (documents). | main, helper W, critic | laptop (docs) | Steps 22 and 24 | Drafts ready for signing | 5 h | Sat-Sun |
| 32 | Wesley signs the record, the rig card and D6; Tejen's answers checked; OPEN lines closed with run ids. | Wesley | none | Steps 19, 30, 31 | Recorded | 1 h | Mon 19 |
| 33 | Fast-forward week4 to the final SHA (no week4 code commits since today), merge rig-prep docs, gate, rebuild, read back the SHA. Never push. | main (his OK) | laptop | Step 32 | week4 = the claim tree | 1 h | Mon 19 |
| 34 | Desk rehearsal on the final install: `fake_mocap caught|uncaught`; F1 load test re-run; W1 `--tilt` check and `takeoff_requested` reaching drone 3's tracker (as evidence); band-split read-back if it is on. | main + Wesley | laptop, rig stack | Step 33 | Every line read back; load bars met | 4 h | Tue 20 |
| 35 | Wed 21 / Thu 22: W1/W2 props-off (evidence); R11a hover; R11b (per D21); R11a-circle, then R11c only with the co-move in the reviewed tree and R11b passed. If F5 is not met: carry follow-ups and leftover Part B items; the combined flight moves to 28-29 Oct. | Wesley + Tejen; main reads every log | rig | Rig card F1-F9 closed; step 34; Tejen's OK | Scored against the frozen bars | 2 days | Wed 21 |

Notes:
- **A. Rev 10 (step 6).**
  - Take-back: B1 per D1, the TB13 ramp, TB16 including the orbit refusal, and the bounded decouple. Fallbacks B2 and (a).
  - TB4 bars:
    - Clearance: ≥ 0.15 m confirms, < 0.10 m falsifies.
    - Attitude step: ≤ 2 deg confirms, > 5 deg falsifies.
    - Ring tilt ≤ 5 deg.
  - TB5 acceptance, including the overflight report. The TB6 list with falsifiers.
  - Claim section: lead030 scoring; the _2 arms for nocatch_orbit, lead030 and land_transit; full M1 beside the table; the TB14 note and count; m1x_still_* retired; the §6 sentence deleted.
- **B. Wave B (step 7).**
  - Order: C2, C3, F1, T2, T3, T9, H1, F6a, F6a2, the 3 neighbours, TB9a ×5 (high, low, offset, abort, land_proof), the R11a hover rehearsal.
  - Last, and can be cut: T4, T5a-c, F2.
- **C. Scorers (step 8).** claim_row, contact_score and claim_checks keyed on REWELD; tilt_row with per-arm readouts; rod-segment, attitude-step, ring-tilt and overflight metrics; the tip_metrics torque fix; the §4 pointer. Reproduce R1573, R1579 and R1581, and read the rod metric on R1565.
- **D. TB6 (step 17).**
  - nocatch_orbit; lead030.
  - nocatch_orbit at `traj_hold_s` 11 and 17.5, each with its own params file.
  - land_rejoin_orbit; m1_full_nocatch_orbit; land_transit; nocatch_hover.
  - R11a-circle.
- **E. Joint batch (step 28), about 50 runs.**
  - Changed arms first: nocatch_orbit, lead030 and land_transit, each with its _2.
  - Then C2, C3, T2, T3, T9.
  - Then the other 10 m1c arms; full M1 ×3 (hover, orbit, nocatch_orbit) beside the table; the R11a hover and R11a-circle rehearsals.
  - Tilt claim: T0, T1, F6b, F6c, C1, C2, F1 ×2, plus C3 ×2 if D6(a) is yes.
  - The 3 neighbours; H1, F6a and F6a2 if D7 keeps them.
  - The 4 W-line checks.
- **F. Rig card (step 31).** F5 SHA; F7 on; F6 per D6; §5.6 take-back bars; R11a-circle before R11c; repeats per D21; the §7 stop-and-report sentence deleted.

## Critical path

1. **Claim chain:**
   - rev 10 (Sun) → critic → m1-comove on m1-tiltcut4 (Mon) → replay (Tue am) → TB6 (Tue am) → fix loop (Tue eve, Wed eve)
   - → Thu gate (fast-forward plus small merges) → critic round 2 part A (Thu eve) → smoke (Fri am) → name the SHA → joint batch (Fri pm)
   - → scoring (Sat) → reviewers (Sat-Sun) → signatures and week4 fast-forward (Mon 19) → rehearsal (Tue 20) → R11a.
2. **Measurement chain (fixed dates):**
   - Wednesday block reviewed (Mon) → rehearsal and load test (Tue) → measurements (Wed 14) → R10a (Thu 15) → record → loader values → rig card.
   - Tejen's answers (from Tue) gate step 35.

Slack is about 1.5 days, because Friday's bars moved earlier and the Thursday merge is smaller. Four things eat it:
- F1-new being built: about +1 day.
- Fallback (a) being chosen: about +0.5 day.
- A critic round 2 finding that needs a second smoke.
- A reviewer asking for more than same-tree re-flies. Then the combined flight moves to 28-29 Oct; reviews are never compressed to keep it.

## Decisions for Wesley (recommended option first)

**Today**
- **D1:** B1, the take-back moving with the ring.
  - Note: the twin has no rotor wash, so B1's overflight of the carriers is unmeasured; B2 stays the rig alternative.
  - Do you authorise fallback (a) if B1 fails TB6? It costs about +0.5 day.
- **D2:** Yes, hover keeps the c6f2032 rise.
- **D3:** Dropped as a question. The critic runs; you can waive it.
- **D4:** Yes. Freeze all of rev 10, the take-back bars and the claim section, before any run (not on Friday).
- **D5:** Yes, one tree along the lineage week4 → m1-rig → m1-tiltcut4 → m1-comove → candidate.
  - Commits as you, with no attribution, on m1-tiltcut4, m1-comove, m1-score, m1-loader and the rig-prep docs.
  - No code commits on week4 until Mon 19.
- **D6 (pushback):** Split the switches.
  - (a) warm_fault_not_live: a candidate for the 21-22 Oct launch line after its reviewer, with arm C3.
  - (b) tilt_cut and fc_disarm_land: off on 21-22 Oct; W1/W2 that morning are evidence only; on from 28-29 Oct if W1/W2 pass and Tejen agrees. The DISARM sheet line is accepted in writing until then.
  - Alternative (as drafted): all three typed as in C2 if W1/W2 pass that morning.
- **D7:** By the frozen text F6b is 1 of 2 (R1582's landing tip-over).
  - Recommended: fly H1, F6a and F6a2 now, and decide keep or delete after them and after D6(a).
  - Also: add the landing-artefact rule to tilt rev 4, for later runs only.
- **D8:** Full M1 stays exploratory beside the table; it becomes its own claim after 22 Oct.
- **D9:** Yes, converge without the co-move if TB6 has not passed by the Thu gate. Add the `real:=true` orbit refusal until the co-move is reviewed.
- **D19 (new):** Yes, TB13's ramped hold goes in with the co-move, with no switch. It changes the hover m1c_land_transit, so land_transit_2 joins the claim.
- **D20 (new):** Yes, TB16: partner_contact refuses trajectories other than hover and orbit; on the rig it refuses orbit until the co-move is reviewed.

**Monday and Tuesday**
- **D10 to D16:** as drafted (Wednesday split and whether Tejen attends, alias exports, the Tejen note and R10a laptop and port, 3-min hover, the sheet lines, loader plus pre-built band split, F1-new deferred).
- **D12 extended:** the note asks for Tejen's written OK on our fork, the coil rule, any switch over his drone, and the R10a logging. Step 35 is gated on his answer.
- Is the lab PC free on Fri 16 afternoon and over the weekend?

**Thursday morning**
- **D17:** The band split, only if the catch spread needs a radius above about 1.5 cm. If on, it goes in the claim profile for every contact arm, not on one side arm. Also proof_rise_max (0.02 or 0.03) and proof_taut_len = L - 0.003.

**Mon 19**
- **D18:** Sign the record and the rig card; OK the week4 fast-forward; pushing stays deferred.
- **D21 (new):** Fly R11b at least 3 times ("more on the rig"), or call 21-22 Oct a first look. Fly R11a-circle before R11c.

## Risks and mitigations

| Risk | Mitigation |
|---|---|
| The co-move take-back fails a fourth time. | Phase-shifted desk replay; phase arms in TB6; R11a-circle; fallback (a); the Thu gate keeps R11a/R11b alive. |
| The rig flies the failing circle take-back after a D9 fallback. | The `real:=true` orbit refusal (D20). |
| The twin cannot see rotor wash during B1's overflight. | Overflight time reported in TB5; listed as a rig finding; B2 as the rig alternative. |
| A seen run is re-scored with a new bar (R1582). | Score by the frozen text; new rules only in rev 4, for later runs. |
| Merge conflicts in the take-back and tilt code on a rig morning. | m1-tiltcut4 as a true merge now; m1-comove built on it; H3 race tests written by its author. |
| Kept switch code with no runs (H1/F6a). | H1, F6a and F6a2 fly in wave B; D7 decided on the evidence. |
| A stuck take-back on the rig drops QUAD4 from 2.3-2.9 m. | D6(a), warm_fault_not_live, as a candidate after its reviewer. |
| The cut's first real-radio use and first flight on the same day. | D6(b): evidence only on 21 Oct. |
| Laptop CPU starvation (9 Oct disarms; R1270 void). | No laptop Gazebo this week. Nothing on the laptop while the rig stack is up, lab_batch included. Helper builds niced to 2 workers. |
| The week4 fast-forward breaks. | No week4 code commits until step 33. |
| Bars tuned to runs already seen. | All bars written before TB6 and wave B; step 27 only names the SHA. |
| Tejen's dependencies: R10a logs, his OK, Wednesday presence. | Explicit asks in Tuesday's note; his answers gate step 35. |
| The reviewer grades single-run changed arms WEAK. | _2 repeats for lead030, land_transit and nocatch_orbit. |
| Lab collisions. | One tree per batch, synced; a fresh `env -i` shell; whole process groups killed; the main session is the only writer of the registry, SESSION_STATE, decisions and cards. |
| Scorers lost from the ephemeral job directory. | Committed in tools/ on Sunday (step 8). |

Files:
- /home/wesley/multi_drone_control/results/2026-10-11/R1582_sim_gz_tc_f6b_not_live/lab_console.log
- /home/wesley/.claude/jobs/50d19a1f/tmp/lab_tc_a2.txt
- /home/wesley/mdc_wt/m1-tiltcut3/docs/experiments/2026-10-11_quad4_tilt_cut.md
- /home/wesley/mdc_wt/m1-rig/docs/experiments/2026-10-21_combined_m1_rig.md
- /home/wesley/mdc_wt/m1-rig/configs/experiments/m1c_land_transit.yaml
- /home/wesley/mdc_wt/m1-rig/configs/experiments/m1c_nocatch_orbit.yaml
- /home/wesley/mdc_wt/m1-tiltcut3/tools/remote/lab_batch.py
- /home/wesley/multi_drone_control/docs/SESSION_STATE.md
- /home/wesley/multi_drone_control/docs/decisions.md
- /home/wesley/multi_drone_control/docs/morning_report_2026-10-11.md

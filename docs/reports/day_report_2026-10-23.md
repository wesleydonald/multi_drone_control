# Day report, Fri 23 Oct 2026

For Wesley, Mitchell Torok and Mohammad Deghat. No rig flights today: twin (Gazebo) runs, analysis, figures and drafts.

**Verdict:** both lab days next week are prepared; Thu 29 has one blocker, and the fix is Tejen's. Most of the twin
results now have numbers at 1 m. Nothing is committed yet.

## Rig (next week)
| Item | Result | Why it matters |
|---|---|---|
| Thu 29 rehearsal: Tejen's install gate | In five twin rehearsals his merged planner threw away 3,533 of 3,541 finished transit plans. It discards any plan when its world model changed after the final check, and our carrier (50 Hz) and ring (20 Hz) poses change that model all the time. QUAD4 waited 8 s to over 3 min for a plan; before the merge it waited under 0.6 s. The clearance was not the cause (18 failed checks in 3,689) | The rig runs the same code, so Thu 29 would stall the same way. The fix belongs in his code: re-check the finished plan against the current world instead of discarding it. Note for Tejen: `docs/rig/note_tejen_install_gate_2026-10-23.md` |
| Thu 29: 60 s GIVE UP | New stop rule on the sheet: GIVE UP if QUAD4 has not started its transit 60 s after his planner reports "settled" | Stops a stalled attempt from using up lab time if his fix has not arrived |
| Thu 29: carry_clearance 0.25 | QUAD4 carries its tether tip 0.25 m above the carriers. Its body flies at about 3.25 m, 0.75 m below the ceiling (the default gave ~3.5 m). It is in the sheet and the twin configs | Keeps QUAD4 well below the ceiling. In his model the magnet passes 5-10 cm from a carrier's body (30-35 cm at 0.5). A twin A/B of 0.25 against 0.5 is flying now |
| Wed 28 readiness | All 15 sheet lines resolve on the merged tree (1 m cables, 0.75 m paths, 1.75 m position limit). The planner is pinned to the rig laptop's fast cores (`MDC_PLANNER_CPUS=0-7`), and every new feature is off on the rig. The day opens with the pair A/B of the velocity filters (ema against slope) | In the twin, the slope filter removes the throttle-delay swing (below). Wed's rig pair is the real test of the 1.4 Hz swing |

## Twin and thesis results
| Result | Numbers | Verdict |
|---|---|---|
| R: every old table re-flown at 1 m | 19 hold, 5 do not, 2 unresolved, 1 not confirmed. At 1 m the 165 deg gap release now tips the ring past the 30 deg drop line (2 of 2 runs; 28 deg at 0.55 m) | Done. Reviewer: every number reproduces |
| U: magnet caught off its slot | Along the rim, aiming at the measured catch point works up to about 8 cm of offset (bar 12 cm). Aiming at the slot works only to 2-4 cm, but folds the newcomer in more gently: tilt 3.5-4.7 deg against 5.1-8.9 deg at 11.6 cm or more. Radially inward, both aims work to 16 cm | Neither confirmed nor falsified. The mechanism is shown radially but not along the rim. 6 owed runs are flying |
| S: iterations against fleet size | With 4 quads, 1 solver iteration per tick held every bar in both repeats of every cell. With 8 quads, the offline check fails 1 and 2 iterations and passes 5. Replayed on 9 earlier flights, 1 iteration misses on every rig flight; 2 iterations pass all 9 | The 8-quad claims fly at 5 iterations, labelled "twin only, not real time" (the rig laptop runs about 4 quads in real time) |
| S: the 8-quad chain | 8 -> 4 -> 8 with four rejoins flies (exploratory): tilt at most 3.5 deg; after the last rejoin 6.3 mm / 0.89 deg | Claim runs queued (batch 18) |
| C: the network's yaw problem comes from the rim attachment | The paper's method models the load as a point with every cable at its centre. Our ring hangs from four rim plates. With every cable hooked at the centre (the hub smoke pair): network 11.8 mm, yaw 0.1 deg (22-51 deg on the rim); ours 1.4 mm, yaw 0.0. On the rim at its best gains, 0.4 m/s line: network 99-105 mm against our 46 mm | Explains the comparison. The full hub table flies in batch 18h |
| Pair swing, rounds C and D | C: a hinge at the tether tip leaves the twin calm (0.08-0.09 deg against the 1.0 deg bar), so it does not explain the swing. D: both config-only fixes remove the twin's swing: slope filter 0.07/0.06 deg, the planner's modelled cable pull 0.01/0.01, against 7.39/7.36 with no fix | Confirmed in the twin. Wed 28's rig pair decides |
| Rejoin E5 (a pair leaning 3 deg) | 3 of 4 runs within the 5 deg bar. One miss: 5.28 deg for 0.19 s while folding in toward the weld plate | Reported as is; no change |

## Built today
- **S cases 6-7:** a freed quad does a task before it rejoins, and the chain changes lap by lap. Both are behind switches, default off (critic waived by Wesley); all tests pass (1,927).
- **Hub world:** every cable hooked at the ring's centre, the geometry of the paper's model. Six configs, bars frozen.
- **Runner fix:** the runner no longer flags stale motion-capture data after the fleet has landed (this race voided one U run).
- **s_metrics fix:** the S scorer now aligns its clocks and stops at the end of the run; it no longer reports false VOIDs.
- **Figures:** two review sets, for Wesley's review before any goes into the thesis. `docs/figures/review_2026-10-23/` (13 figures) and `.../review_2026-10-23_best/` (9 figures: the best 2/3/4-quad flights, rig first). Colour scheme: one colour per fleet size (2 orange, 3 rose, 4 purple), red/blue for planned against measured, grey for references, black only for the ring.
- **Journal drafts (IEEE RA-L):** `docs/paper/ral_2026-10-23/` (7 pp, with the comparison to the paper's method) and `docs/paper/ral_2026-10-23_reconfig/` (8 pp, reconfiguration only, with the release-transient and catch-offset subsections). Placeholders in red: Thu 29 attach, real-time bench, 8-quad claims, the hub results.
- **Thesis drafts:** five results sections updated plus a new U section (unreviewed, thesis worktree).
- **Videos:** the 7 overnight highlights and a reel; 12 afternoon runs and a 24 min reel (`results/videos/reel_2026-10-23_*.mp4`). Both emailed.

## Queued on the lab PC
- **18u** (25 runs, running since 16:37; 13 passed by 17:39): U's 6 owed runs; our planner's 5 C cells; the paper's own damping form x6; the Thu 29 re-flies and the clearance A/B.
- **18h** (14): the hub table, both methods x2, plus the network as first ported, on the hub.
- **18** (16): the 8-quad claims at 5 iterations x2, the 9 harder exploratory cases, the 3 case 6-7 configs.

**Pending:** scoring and reviewers for 18u/18h/18, then U, S and C; the timed solve bench (needs the laptop idle,
tonight); Tejen's fix and its critic; U and R re-flown whole on the final tree at the results freeze (Sat 7 Nov).

## Decisions for you
1. **Send Tejen the install-gate note.** Without his fix Thu 29 will probably stall.
2. **Commit today's tree** (the Tejen merge, the fixes, the worlds, the configs): nothing is committed.
3. **Rig solver at 2 iterations?** Recommended for fleets of up to 4 quads: it passed every quality bar on all 9 flights at about half the solve time (the rig planner runs close to its tick at 5). Needs a twin check and a reviewer first; it changes a launch default.
4. **Faults table:** cite the 1 m result (the 165 deg gap drop) as I have, or keep the 0.55 m history.
5. **Which journal draft to take forward:** with the comparison or reconfiguration only; also the affiliations.
6. **Restart the backup refresh** (`~/mdc_backup/2026-10-22/refresh.sh`); it stopped at 11:00.

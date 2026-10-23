# Morning report, Fri 23 Oct 2026 (the overnight loop, 22 Oct 23:40 to 23 Oct 07:09)

**Verdict:** the merged tree (1 m defaults, the position limit, rejoin, S, C, U, R, pair-swing) is green on
`gate --quick`, ready for Wed 28 at 0.75 m paths, and about 180 twin runs are scored. Nothing is committed.

## Rig (Wed 28 / Thu 29)
| What | Result | Why it matters |
|---|---|---|
| Position limit | On for every rig flight: `pos_limit_m` 1.75 in real.yaml (your word). QUAD4 is exempt while Tejen flies it (his pickup may sit outside). Twin runs at 0.75 m never come near it: reach <= 1.469 m (R2435-R2437) | The LAND line is 27 cm past the outboard quad |
| Rig paths 0.75 m | Wed sheet updated; its band table is filled for 2/3/4 quads, figure-8 and spinny line, both filters (R2423-R2434) | The spinny line now turns at ~32 deg/s; expect ~16 deg of yaw lag |
| Rig solvers | The rig has its own folder (`~/mdc_rig_generated`, 0.583 kg), refreshed this morning; `rig_flight.sh` and `rctl`/`rdiss` use it | Twin runs can no longer leave the rig a planner at the wrong mass |
| Planner timing on the rig | From the 22 Oct logs: at 3 quads the planner runs near its 100 ms tick (p95 116 ms, 10-12 % of ticks over 110 ms), 2-3.4x slower in flight than on the bench; pinned to an E-core it is 2.6x slower than on a P-core | A switch `MDC_PLANNER_CPUS=0-7` (default off) pins it to the fast cores |
| Betaflight (QUAD3, read-only) | Radio + FC add ~10-15 ms; rate mode, linear 100 deg/s, feedforward on, ELRS 333 Hz | No FC change needed; `docs/rig/betaflight_2026-10-22.md` |
| Pair swing (twin) | Round A: the swing rides on the throttle path (R2450/51 7.4 deg vs R2452/53 0.6-0.8). Round B: a twin with the rig's measured timing stays calm, so it does not reproduce the rig pair. R2438: with 60 ms delay and the old filter, a pair on the 0.75 m figure-8 swung until a quad disarmed | Supports slope for the pair A/B on Wed; the twin's 1 m pair prediction is calm with either filter |
| Spin yaw lag | Planner-side: in the twin's 0.75 m spinny line (R2593) the plan itself trails the desired yaw (0.14 s at node 0, still 0.09 s at its 0.5 s node; ring 0.14 s, 3.7 deg). New log columns plus `tools/yaw_track.py <run> --plan` show the same split on the rig | Run it after Wed's spinny lines; the rig's 0.49 s vs the twin's 0.14 s tells where the extra lag enters |

## Thesis sims
| Result | Runs | Verdict |
|---|---|---|
| R, every 0.55 m table re-flown at 1 m | R2483-R2577, attribution R2580-R2583 | **13 hold, 3 do not, 3 skipped** (no 1 m world: tier_b, gain, push rows). Reviewer [weak], corrections applied. T7t orbit_v300 is just outside at 1 m, mostly from the newer tree (the ema filter), not the cable. The 2-quad landing stalls at 1 m on the twin's rigid rods. Table: `docs/analysis/refly_1m_table_2026-10-23.md` |
| Rejoin of a departed quad (twin only) | R2445, R2457-R2596 | **E1 x3, E2, E3 and E4 all PASS** after two fixes: the card's 10 s blend now reaches the planner (it ran 1.5 s), and three bugs were fixed (the rider's height rose, an out-step stopped short, the staging ramp overshot). Fold-in tilt 1.3-2.3 deg on the final tree (R2594-R2596, the path held for the 10 s blend after the critic). E0 (a floor re-weld) fails its tilt bar on an off-plate weld |
| S, scalability | R2464, R2587, R2588, R2597, bench | **8 quads hover, chain 8 -> 4, and the full 8 -> 4 -> 8 with four rejoins all fly in the twin** (tilt <= 3.5 deg; R2597 re-flown on the final rejoin tree). The rig-calibrated bench gives N_max ~4 in real time, so 8 quads would be "twin only, not real time" |
| C, the paper's method vs ours | R2446-R2448, R2468-R2469, offline search | As ported, the network capsizes in yaw in the twin too. **Corrected 23 Oct 09:55:** the twin's "retuned" runs (R2468/69, yaw to +-60 deg; 156 mm vs our planner's 46 mm on the 0.4 m/s line) flew the search's stage-1 point (diss_c 12, vel_kp_pos 1, the rest at defaults), which fails two offline guards. The search's best (also k_pay 80, k_ring 40, ki_load 0.5, kv 3; J 115, all guards pass) has not flown in the twin. The offline harness lacks the drones' 4 cm pivot lever, the likely reason it misses the twin |
| U, the magnet caught off its slot | R2459-R2463 (E-runs) | Built, its critic answered, plumbing clean. At 16 cm along the rim the slot arm beat the measured arm; the measured arm crowded a carrier (0.16 m). The grid (56 runs) waits for your four freeze questions |
| Position-limit runs | R2435-R2444 | On at 0.75 m: never trips. A quad freed at the lobe tip lands inside 1.65 m |

## Decisions for you
1. **Pin the rig planner to the fast cores?** `MDC_PLANNER_CPUS=0-7` in front of the rig lines. Recommended: yes for Wed after a 2-minute desk check (the planner runs near its tick at 3 quads already).
2. **S's 8 quads:** label them "twin only, not real time" (recommended) or keep S to about 4 quads.
3. **C:** add the 4 cm pivot lever to the offline harness and retune (~half a day, time-boxed), or report "the network as ported cannot hold the ring's yaw" as the result. Recommended: the harness fix, once.
4. **U's grid:** four questions at its freeze (in its card's Response): may separation stay out of the arm comparison; 2 repeats for the 11.6 cm wrong-plate cell; the model-error wording past 2.5 cm; does confirming need the mechanism bar.
5. **Pair-swing round C:** approve the tip-hinge twin world (5.75 cm), or the rig-timing twin switches are deleted (the card's rule after round B).
6. **R's skipped tables:** four 1 m gap worlds (tier_b) and a 1 m push world, or leave those tables at 0.55 m.
7. **Rejoin fixes:** the new reference cap changes a carrier's reference in the twin's flight loop. Its critic (this morning,
   in the rejoin card) says run it: nothing blocks the hover tests, the switch-off and QUAD4 paths are untouched; before S
   or the chain cite it, hold the path for the full 10 s blend (done), re-fly S's 8 -> 4 -> 8 and E2 x2 on that tree (done:
   R2594-R2597 pass), and add one leaning-pair neighbour (not flown: it needs a new config; your call).

Details: `docs/SESSION_STATE.md` (overnight block), the cards' Results sections, `results/registry.csv` R2417-R2592.

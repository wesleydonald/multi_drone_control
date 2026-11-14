# Te-Jen's C++ planner against the 12 Nov 2026 real flights (13 Nov 00:10-03:00, an agent for the lab chat)

Exploratory; one replay per row unless noted. Files: probe2/3.cpp, extract*.py, prep.py, summ.py, maps.py, out/, maps/
(the quick check); full/ (the open-loop replay: driver.py, run_flight.sh, chain_r.sh, load_run.sh, backend*.yaml,
ring_bridge.yaml, nodes_fm9h8.json = the rig desk check's dumped parameters; runs/<run>/ with driver.csv, replans.csv,
backend.csv, logs and binary.txt), table.py, loadsum.py, dist.py, precap.py, capring.py, swing_q4d.py.

## Bottom line (revised 13 Nov 03:43 after the splice-0.20 moving replays; Te-Jen's Codex entry 16)
- Moving ring, real time, splice 0.10 (the frozen baseline): no capture of a ring circling at 0.10-0.15 m/s (nro 90 s:
  parked beside the ring's path, tethers within 0.19 m of its hanging line every lap; 0.05 m once in fm9hc_4 with refresh
  0.2). This is a result for those exact settings, not a general impossibility.
- Moving ring, real time, splice 0.20 + refresh 0.2 (last section): several captures while the ring moved (nro 3 of 3 at
  spline 4, 26-41 s; spline 3/2 faster), BUT unsafe stopped-hold outcomes remain: in nro_mv_z29s20sp3_r2 and
  nro_mv_z29s20sp3_load8 QUAD4 stood still 10-11 s and a carrier tether crossed its hanging line (0.00 m, centre lines).
  These two are FAILED SAFETY RESULTS, whatever happened after (both later "captured"). fm9hc_4_mv_z29s20sp3 parked 21.5 s
  at 0.12 m. The moving-ring operation is not supported.
- Still ring (hover, or the circle after it stopped): captured every time once his world refresh is coalesced.
- Line A (outer_octagon, swing 5, spline 4, carrier tracking 0.06, 30 rendezvous-time steps,
  cooperative_world_refresh_period_s 0.2, maximum_search_z_m 2.9, fixed_splice_lookahead_s 0.15): 8 of 8 with 8 of the
  laptop's 12 CPUs busy, QUAD4 at 2.81-2.88 m, no stranding. 0.20 s splice: 2 of 2, 0 late (evidence_compact.md).
- Without the splice change, load breaks it (cap 2.9: 1 of 1 failed at 8 busy cores; cap 3.8: 1 of 3).
- His default legacy_square never works: the drop point is inside his obstacle set at every sample, still at 3.2 m.
- All of it is open-loop replay (see Caveats and the Codex limits below); the splice and coalescing are changed timing
  policies, not approved as contract-preserving.

## Why (real time)
Every carrier state / plate / ring pose message changed his collision world, ~214 versions/s (plate_from_body follows
the measured samples). With outer_octagon his final recheck near the tethers takes 60-90 ms on this laptop (legacy
0.5 ms), so almost every candidate ended WORLD_CHANGED_DURING_FINAL_RECHECK: 0 of 4 captured. The twin ran at a
real-time factor of 0.13-0.16 and hid this; the same replay slowed to 0.15/0.3/0.5x captured each time.

## The real QUAD4 cable swing on 12 Nov (body 14 to tip 24, from vertical; static offset <= ~1 deg)
| flight (who planned) | median | p95 | max | time over 3 deg |
|---|---|---|---|---|
| fm9_5 (Python, straight after the lift) | 4.9 | 8.8 | 10.3 | 85 % |
| fm9hc_2/_4/_5 (Python) | 1.7-2.9 | 3.5-5.5 | 4.0-5.9 | 13-47 % |
| q4d_3/_4/_7 (his C++) | 1.5-3.1 | 2.3-4.2 | 2.4-4.5 | 0-60 % |

## Caveats
Open loop: the virtual QUAD4 tracks his reference perfectly and does not swing; the legacy Bool authority grant, not
the recovery handshake; the target synthesised from the mission's formula; nro's ring at 1.0 m; the fm9hc moving windows
only ~20-25 s; "busy cores" (python spin loops) only a stand-in for the rig stack (measure it with pidstat in a flight);
centre-line distances to the measured carriers; replan runtimes rose over the session (p95 161-208 ms in the last runs).
Binaries: v0 before 01:52:46, v1 the refresh build 01:52:46, v2 the lab chat's changes after Te-Jen's Codex review (02:03:52; Codex reviewed diffs only, it did not build or run this fork) (all still-ring, load
and nro results are v2).

## Te-Jen's Codex review of this summary (entry 14, 13 Nov 02:57), recorded as limits
- 8/8 are captures in an open-loop replay (perfect virtual tracking, no swing, a synthesised target, the legacy Bool
  grant), not integrated missions: they do not validate the prepare/grant/post-grant contract, the drop/clearance, the
  return or an abort.
- The real swing on 12 Nov exceeded 5 deg on Python transits after a lift (fm9_5 p95 8.8, max 10.3; some fm9hc maxima
  above 5): the 5 deg envelope is not established for every case; 3 deg is not valid.
- fixed_splice_lookahead_s 0.15/0.20 is a changed timing policy (a longer immutable prefix, later reaction), separate from
  the 0.10 baseline and from the refresh coalescing; neither is approved as contract-preserving.
- CPU spin loops are a stress stand-in, not the rig stack's scheduling, DDS and publication behaviour.

## Compact timing evidence (13 Nov 03:00): evidence_compact.md (full/evidence.py)
Per selected run: parameters, binary, authority and capture, replans before and after authority, accepted vs INSTALLED,
late, the resolved splice, replan runtime, installed splice margin. Splice 0.15 (A): installed margins down to 0-5 ms
and 2-22 late candidates per run under load; splice 0.20 (B): margins >= 70 ms, 0 late (2 runs). The rig scripts type
0.20. Splice 0.10 with cap 2.9 (z29_load8): 199 late of 240, never captured.

## The moving ring at real time with splice 0.20 (13 Nov 03:00-03:25, the lab chat; chain_mv*.sh, full/precap2.py, full/dist.py)
All: outer_octagon, swing 5, tracking 0.06, 30 steps, refresh 0.2, fixed_splice_lookahead_s 0.20, binary v2; one replay per
row. nro = the continuous 90 s circle; fm9hc_* = the circle's moving windows (~20-25 s, then the ring stopped). "Parked" =
QUAD4 still (< 0.05 m/s) with a carrier tether or body within 0.5 m of its hanging line, before capture.

| replay | spline / cap | captured after authority (ring speed then) | QUAD4 z max | closest cable-tether before capture | parked before capture |
|---|---|---|---|---|---|
| nro_mv_z38s20 | 4 / 3.8 | 39.5 s (0.124 m/s, moving) | 3.13 | 0.39 | 0.3 s, 0.40 m |
| nro_mv_z38s20_r2 | 4 / 3.8 | 30.2 s (0.133, moving) | 2.77 | 0.35 | 0.4 s, 0.43 m |
| nro_mv_z29s20 | 4 / 2.9 | 40.8 s (0.132, moving) | 2.62 | 0.35 | 0.4 s, 0.38 m |
| fm9hc_4/_2/_5_mv_z38s20 | 4 / 3.8 | 32.3 / 25.8 / 28.2 s (0.035-0.043: after the ring slowed) | 3.01 / 3.12 / 3.52 | 0.37-0.47 | none |
| nro_mv_z38s20sp3 | 3 / 3.8 | 11.2 s (moving) | 2.66 | 0.42 | none |
| nro_mv_z38s20sp2 | 2 / 3.8 | 9.2 s (moving) | 2.58 | 0.38 | 0.1 s, 0.36 m |
| nro_mv_z29s20sp3 | 3 / 2.9 | 25.8 s (moving) | 2.59 | 0.39 | none |
| nro_mv_z29s20sp3_r2 | 3 / 2.9 | 38.9 s (moving) | 2.60 | **0.00** | **10.5 s, 0.00 m** |
| nro_mv_z29s20sp3_load8 | 3 / 2.9, 8 busy loops | 40.6 s (moving) | 2.58 | **0.00** | **10.9 s, 0.00 m** |
| fm9hc_4_mv_z29s20sp3 | 3 / 2.9 | 47.1 s (after the ring slowed) | 2.84 | 0.12 | **21.5 s, 0.12 m** |
| fm9hc_2 / _5_mv_z29s20sp3 | 3 / 2.9 | 22.9 / 31.5 s (after the ring slowed) | 2.77 / 2.79 | 0.37 / 0.39 | none / 0.7 s, 0.39 m |
| fm9_5_z29s20sp3_load8 (still) | 3 / 2.9, 8 busy loops | 13.5 s | 2.81 | 0.36 | none |

Reading: the splice (0.10 -> 0.20) is what let his planner reach a moving ring at real time at all (with 0.10: never in
90 s, LATE ~40 %; with 0.20: LATE 0-2 %). But it still has no escape: when no plan is found it stops, and a stopped QUAD4
beside a moving ring is swept by the carriers' tethers (0.00 m, centre lines, in 2 of 3 spline-3 nro replays; the
virtual QUAD4 stood still 10-11 s first). Spline 4 parked under 0.5 s in 6 of 6, but took 25-41 s and in the circle's
20-25 s moving windows never caught it while it moved. Not proposed for the rig: one park is a cable-on-cable contact.

## The two zero-clearance windows (zero_windows.md, full/zero_window.py; 13 Nov 03:44)
Both show the same pattern in 1 s bins: near the ring his candidates fail (LOCAL_PLAN_FAILED:PADDED_CLOSEST_PARTIAL...,
late candidates rejected after the recheck), commit end - now falls below zero, the incumbent revalidates unsafe, and his
backend is in UNCERTIFIED_HARD_HOVER (execution_fallback_certified 0, reason INCUMBENT_REVALIDATION_UNSAFE) for several
seconds while QUAD4 holds still and the moving ring's tether closes from 0.38 m to 0.00 m. Failed safety results either
way. Not established (Te-Jen's Codex entry 18): that the plan's expiry is the root cause (a stopped tail is allowed), or
that no geometry/admission discrepancy contributed (the bins mix the last driver/backend rows with the bin's minimum
distance; the cooperative receipt ages shown are not the represented geometry's age at the crossing).

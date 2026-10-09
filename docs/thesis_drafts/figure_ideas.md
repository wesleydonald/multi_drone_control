# Figure ideas for the thesis (from the reference papers, 9 Oct 2026)

Ranked; each: the claim, the layout, the paper figure it borrows from, and the data (exists / which run fills it).
Built ones are marked [built]; placeholders in the draft are marked [pending].

1. **Rig detach three ways vs the twin** -- the OCP-resize detach holds the ring on hardware (announced,
   unannounced, 150 deg gap) and the twin predicts the transient. Columns per condition; rows: planned tension
   per UAV (leaver dashed, its colour), ring tilt (rig solid gold, twin repeats thin grey, detection tick), ring
   height error; x from -3 to +10 s after release. (Sun 2025 Fig 2A, S5; F_detach.) Data: RIG-1008-r302_3,
   r303_4, r304_3; twin R1207/R1212, R1140/R1145, R1206/R1211.
2. **Rig circle detach storyboard** -- plan view with the drones at three instants and the freed UAV's exit, strips
   of tilt, error and survivors' tensions. (Wang 2024 Fig 9, Sun Fig 3A, Quan Fig 2.) Data: RIG-1008-r305;
   twin R1208/R1213; Wednesday w5 adds the disarm alone.
3. **Rig rejoin (R8b) storyboard** [pending] -- stages climb, transit, hold, weld; tilt with the weld bracket; tip
   height error from tip.csv with the weld radius. (Quan Fig 2, Wang Fig 9.) Data: Wednesday 14 Oct R8b x2-3
   and r8a; twin R1266, R1404/R1410.
4. **Twin-vs-rig parity** -- the twin predicts rig metrics within a band: x twin (mean of 2, range), y rig, y = x
   with a band, per metric (tracking error, detach tilt peak, dip, detection latency). (Sun Table 1, Gabellieri
   Fig 2.) Data: RIG-1007-r006/R1092, r0007/R1087, r000014/R1069-R1071, r0012/R1093; RIG-1008 detaches vs their
   twins; attach after Wednesday.
5. **Model integral on the rig** -- off: the ring sinks; on: it holds and the learned force settles; 100 g added:
   learned F_z = the weight. (Lee 2018 Fig 2, Klausen 2020 Fig 6/7.) Data: RIG-1007-r0001, r002, r000014.
6. **Anatomy of the rig capsize** (r0013 vs r0012) -- tensions, tilt clipped with "89 deg at +0.61 s", heights;
   twin R1125 grey. (Sun Fig 2A clipped baselines, Gabellieri Fig 3.)
7. **Measured-tip loop as offset-free control** [built: F_tip_feedback] -- parked tip error vs injected rod error
   (off slope ~1, on flat) and e_z / c from hold entry. (Pannocchia 2007 Fig 3/4, Gabellieri Fig 2/5.) Data:
   R1393-R1410 (rev 5); Wednesday r8a adds a rig point.
8. **LAND in every flight phase** [pending re-fly] -- phases in flight order; outcome per repeat; peak tilt and time
   to disarm. (Sun Fig 6A.) Data: the re-fly on f330bd2 (running 9 Oct evening); before/after R1303/R1311 vs
   R1322/R1323.
9. **Planner solve time vs fleet size** -- solve time through r305 with the 4 -> 3 resize; boxes per N and planner,
   rig vs twin, 100 ms budget dashed. (Sun Fig 7A/B.) Data: every planner log.csv (rig 4 UAVs: p50 66, p99 88,
   max 92 ms in flight; 3 UAVs p50 31 ms).
10. **Thrust headroom through a detach** -- throttle per survivor around the release, twin band, rig line, cap
    dashed. (Andrien 2024 Fig 7.) Data: rig tracker logs r302_3-r305, r0012/r0013; twin R1206/R1207/R1211/R1212.
11. **Clearance during the approach and the freed UAV's exit** -- minimum distance to each carrier vs time with
    d_min. (Sun Fig 3A(iv), 7C(iv).) Twin data exists; rig approach [pending] Wednesday r8a/r8b.
12. **Who disarms when** -- swimlanes per drone (carrying / free / landing / disarmed) for the fault and freed-drone
    scenarios. (Wang Fig 9 bands, Quan Fig 2.) Twin: R1258-R1265, R1284-R1287, R1309/R1326, R1310/R1325; rig
    [pending] Wednesday w4/w5.
13. **Robustness to a wrong belief** -- mass, rod, radius errors with the integral off/on, rig 100 g starred. (Sun
    Fig 4A/B.) Data: R1038-R1067, RIG-1007-r000014.
14. **Reconfiguration triptych + annotated rig photo** [photo pending] -- 4 -> 3 -> 4 top views with tension arrows to
    scale; photo with masses, rod, pivot, magnets, mocap bodies. (Quan Fig 1C, Sun Fig S3.) Photo: Wednesday 09:00.
15. **Rig vs twin trajectories** -- plan view reference / twin / rig, error vs time, RMS per axis. (Sun Fig 2A,
    Andrien Fig 9, Klausen Fig 10.) Data: RIG-1007-r006, r0005, r206e60, r0007, r2008e60; twin F_traj runs.

Also built: **F_departed_landing** (rig 8 Oct freed UAV hovered and was dropped vs twin departed landing).

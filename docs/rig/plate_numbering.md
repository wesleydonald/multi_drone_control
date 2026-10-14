# Plate numbers and angles

**Plates** count clockwise from the ring's +x (ring rigid body 8) seen from above, like a clock: plate 0 on +x,
plate 3 on -y, plate 6 on -x, plate 9 on +y (Wesley 7 Oct, f3b3072). The plates carry no physical labels; RViz draws
the numbers. QUAD4's plate (+y) is plate 9, which Tejen calls plate 3 (he counts counter-clockwise).

**Angles** (Wesley 14 Oct): every angle that is typed (launch arguments, configs) or shown (log lines, the console,
RViz, the sheets) is in clockwise degrees from +x, so the angle is 30 x the plate number: carriers on plates 3 / 11 / 7
at 90 / 330 / 210 deg, QUAD4 on plate 9 at 270 deg. The maths inside (ROS frames, mocap, the optimiser, Tejen's code)
keeps counter-clockwise azimuths, converted at one boundary: `bringup.profiles.apply` for `attach_azimuths_deg`
(once per launch), `partner_mission.launch.py` `_his_ring` for `plate` and `carrier_azimuths_deg` (our clockwise
plate number becomes Tejen's index). Node parameters and per-run parameter dumps stay counter-clockwise, as before;
run manifests from 15 Oct carry `angles: cw` (`tools/run_dir.manifest_azimuths_ccw` reads either). The arguments are
the same names with the meaning flipped (Wesley's choice): an old line typed the old way puts the carriers on the
mirror-image plates, and the planner's start-up check then logs an error naming mirrored angles.

| Plate | Clockwise angle (typed, shown) | Counter-clockwise azimuth (inside; typed before 15 Oct) | On it (M1 rig) | Tejen's plate |
|---|---|---|---|---|
| 0 | 0 | 0 | +x (the orange plate in RViz) | 0 |
| 1 | 30 | 330 | | 11 |
| 2 | 60 | 300 | | 10 |
| 3 | 90 | 270 | carrier drone 1 (`/drone_0`) | 9 |
| 4 | 120 | 240 | | 8 |
| 5 | 150 | 210 | | 7 |
| 6 | 180 | 180 | | 6 |
| 7 | 210 | 150 | carrier drone 3 (`/drone_2`) | 5 |
| 8 | 240 | 120 | | 4 |
| 9 | 270 | 90 | QUAD4 (+y, `attach_y_offset` 0.225) | 3 |
| 10 | 300 | 60 | | 2 |
| 11 | 330 | 30 | carrier drone 2 (`/drone_1`) | 1 |

## History

- Before 8 Oct: plates counted counter-clockwise (the 30 Sep, 1 Oct and 7 Oct rig days).
- From 8 Oct: plates clockwise (f3b3072); angles stayed counter-clockwise azimuths. The 8-14 Oct sheets and the
  RIG-1008-* and RIG-1014-* rows use this.
- 14 Oct evening: the plates were renumbered counter-clockwise by mistake (4a60ca3, a73d9a4) and put back the same
  evening; the angles change to clockwise instead.

# Plate numbering

Plate k sits at +30k deg, counter-clockwise from the ring's +x (ring rigid body 8), seen from above: plate 0 on +x,
plate 3 on +y, plate 6 on -x, plate 9 on -y. This matches the azimuths in every config (`attach_azimuths_deg`,
`attach_x_offset` / `attach_y_offset`), Tejen's plate index, the thesis and the twin worlds' `plate_k_visual`.
The code helper is `mpc_planner.geometry.plate_of_azimuth`, and RViz draws the numbers. The plates carry no
physical labels.

Today's layout: carriers at 270/30/150 deg are plates 9/1/5 (drones 1/2/3 = `/drone_0..2`). QUAD4 (`/drone_3`)
attaches on plate 3 (+y, `attach_y_offset` 0.225), which is Tejen's plate 3 too. The four-drone detach layout
30/90/150/270 deg reads 1/3/5/9.

## History

- Before 8 Oct 2026: counter-clockwise, as now (the 30 Sep, 1 Oct and 7 Oct rig days, RIG-0930/1001/1007).
- 8-14 Oct: clockwise, plate k at -30k deg (commit f3b3072). The sheets, cards and logs of those days use it,
  including the 14 Oct sheet (`wed_2026-10-14.md`) and the registry rows RIG-1008-* and RIG-1014-*. Their
  plate 9 is today's plate 3, and their carriers on 3/11/7 are today's 9/1/5.
- From 15 Oct: counter-clockwise again. Only the numbers changed; every azimuth stayed the same.

Old to new: new = (12 - old) mod 12.

| 8-14 Oct (clockwise) | from 15 Oct | azimuth (deg) | on it |
|---|---|---|---|
| 0 | 0 | 0 | +x (the orange plate in RViz) |
| 1 | 11 | 330 | |
| 2 | 10 | 300 | |
| 3 | 9 | 270 | carrier drone 1 (`/drone_0`) |
| 4 | 8 | 240 | |
| 5 | 7 | 210 | |
| 6 | 6 | 180 | |
| 7 | 5 | 150 | carrier drone 3 (`/drone_2`) |
| 8 | 4 | 120 | |
| 9 | 3 | 90 | QUAD4's attach plate (+y) |
| 10 | 2 | 60 | |
| 11 | 1 | 30 | carrier drone 2 (`/drone_1`) |

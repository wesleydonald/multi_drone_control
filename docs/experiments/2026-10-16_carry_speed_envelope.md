# Card: the three-carrier carry's speed envelope in the rig twin

Drafted 16 Oct night from the exploratory read (R1723-R1752, R1769-R1778; `docs/analysis/traj3_2026-10-15.md`).
**Bars frozen 16 Oct ~08:05 (Wesley: as drafted), before any claim run.** Freeze tree: week5 4b3560e's flight code,
flown from branch day-1016 (its first commit adds only a console button, a harness event and docs). Thesis place:
the new twin carry table and figure for Section 4.1 (inventory T7t/F8t), replacing the 4-5 Oct appendix trajectory
table flown on the old stack.

## Claim

Our planner (mode detach, int_mode auto, rig thrust map) carries the 0.86 kg ring with three carriers on 0.55 m rods
along all five load trajectories (circle, figure-8, line, orbit, spin; radius 0.5 m, line 1.0 m) at 0.125, 0.2 and
0.3 m/s in the rig twin, within the bars below, and does the same with 1 m cables (one arm, 0.2 m/s on four shapes).

## Arms and runs

- 15 cells (5 shapes x 3 speeds), configs `traj3_<shape>_v<speed>`; 2 runs each on the freeze tree (30 runs).
- Cable-length arm: `traj3_L100_*` (circle 0.125, circle/fig-8/line/orbit 0.2), 2 runs each (10 runs).
- Baseline cell: circle at 0.125 m/s (the rig's only flown speed, 14 Oct).
- Floor start, ring to 1.0 m, LAND after the path; drop_on_loss on; lab PC.

## Bars (proposed; every run of every cell)

1. No abort, drop or unannounced detach; every drone lands and disarms.
2. Path RMSE (metrics.json `payload_rmse_m`) <= 25 mm (exploratory max 19 mm).
3. Peak ring tilt (`payload_tilt_peak_deg`) <= 6 deg (exploratory max 5.1, the spin at 0.3 m/s).
4. Radius ratio (`payload_radius_ratio`) >= 0.95 (exploratory min 0.964).
5. The 1 m arm: bars 1-4, and RMSE within 1.5x its 0.55 m cell.

Falsifier: any run failing a bar; the cell is then reported as outside the envelope, not re-flown until it passes.

## Limits to state with the table

The twin's links are rigid (the rig's cables go slack), it has no downwash and its latency is the bridge's model
only (`traj3d_*`, 15-45 ms extra, is the latency check); the rig carried the ring at 0.125 m/s only (14 Oct).

## Reviewer

On the final table, before the thesis.

## Result (16 Oct 08:22-09:27; lab PC; registry R1847-R1886)

One flight-code tree (week5 4b3560e; parameter read-back identical in all 40 runs), flown from 5375333 and e836cf3,
which differ only in docs, unused configs and worlds, the console, and a runner event these runs do not use.

**Verdict: 39 of 40 runs pass every bar. The spin at 0.3 m/s fails bar 3 in one run (R1875: peak tilt 6.31 deg,
mid-spin; its repeat R1876 3.76): as frozen, that cell is outside the envelope, and it is not re-flown.** Every other
cell is inside at 0.125, 0.2 and 0.3 m/s. The 1 m arm passes bars 1-5: RMSE at most 1.27x its 0.55 m cell's mean on the frozen window (1.35x on the path
alone); the 1 m configs also LAND 5 s later, so the comparison is not one variable.
No abort, drop or fault in any run. R1880's harness stopped on a stalled sim clock 1.8 s after LANDED (teardown):
the flight is complete and scored (after the fleet disarmed).

| cell | runs | ring position RMSE, takeoff + 5 s to LAND (mm) | peak ring tilt over the whole flight, lift-off and landing included (deg) | radius ratio | bars |
|---|---|---|---|---|---|
| circle_v125 | R1847 R1848 | 8.8 / 10.6 | 1.84 / 1.86 | 0.997 / 0.997 | pass |
| circle_v200 | R1857 R1858 | 9.2 / 12.4 | 1.73 / 1.73 | 0.995 / 0.991 | pass |
| circle_v300 | R1859 R1860 | 14.0 / 14.7 | 2.04 / 2.81 | 0.982 / 0.982 | pass |
| fig8_v125 | R1849 R1850 | 12.7 / 10.3 | 2.26 / 2.30 | 0.987 / 0.996 | pass |
| fig8_v200 | R1861 R1862 | 13.6 / 14.5 | 2.01 / 1.88 | 0.984 / 0.979 | pass |
| fig8_v300 | R1863 R1864 | 17.9 / 17.8 | 2.40 / 2.32 | 0.971 / 0.977 | pass |
| linex_v125 | R1851 R1852 | 6.5 / 8.3 | 1.91 / 1.96 | 0.996 / 0.992 | pass |
| linex_v200 | R1865 R1866 | 9.1 / 9.8 | 2.81 / 2.51 | 0.986 / 0.984 | pass |
| linex_v300 | R1867 R1868 | 12.5 / 16.8 | 3.04 / 3.66 | 0.978 / 0.966 | pass |
| orbit_v125 | R1853 R1854 | 10.1 / 8.1 | 1.88 / 1.83 | 0.990 / 0.993 | pass |
| orbit_v200 | R1869 R1870 | 12.0 / 13.7 | 2.32 / 2.04 | 0.984 / 0.982 | pass |
| orbit_v300 | R1871 R1872 | 17.8 / 17.1 | 2.85 / 2.75 | 0.975 / 0.977 | pass |
| spin_v125 | R1855 R1856 | 10.4 / 10.5 | 1.88 / 1.88 | 0.995 / 0.998 | pass |
| spin_v200 | R1873 R1874 | 11.5 / 11.3 | 1.95 / 2.20 | 0.992 / 0.999 | pass |
| spin_v300 | R1875 R1876 | 15.9 / 14.2 | 6.31 / 3.76 | 1.004 / 0.997 | **3 FAIL (R1875)** |
| L100_circle_v125 | R1877 R1878 | 10.3 / 11.1 | 1.84 / 1.80 | 0.997 / 0.996 | pass |
| L100_circle_v200 | R1879 R1880 | 12.1 / 12.4 | 1.97 / 1.77 | 0.989 / 0.988 | pass |
| L100_fig8_v200 | R1881 R1882 | 14.6 / 16.0 | 2.46 / 1.66 | 0.979 / 0.979 | pass |
| L100_linex_v200 | R1883 R1884 | 12.0 / 11.0 | 3.01 / 1.87 | 0.984 / 0.983 | pass |
| L100_orbit_v200 | R1885 R1886 | 11.9 / 13.2 | 2.84 / 2.18 | 0.985 / 0.982 | pass |

Against the exploratory read (R1723-R1752): the same picture; its spin at 0.3 m/s peaked at 5.1 deg, so that cell
sat closest to the bar then too.

## Reviewer (16 Oct ~09:36): SUPPORTED WITH CORRECTIONS (applied above)

- Every value recomputed from run.csv/events.csv matches; flight code, worlds, configs (sha256) and the parameter
  read-back are one tree; bars unchanged from the draft to the freeze; no abort/drop/fault line in any log; R1875 is a
  real fail (above 6 deg for about 0.3 s, t 32.1 s, inside the path); R1880 is a complete flight.
- **The metrics' windows:** `payload_rmse_m` runs from TAKEOFF + 5 s to LAND (floor, lift, hovers and, for circle,
  fig-8 and spin, the planner's own descent included); the path-only `payload_rmse_sweep_m` reads 4.3 mm (circle
  0.125) to 20.9 mm (fig-8 0.3). `payload_tilt_peak_deg` is over the whole run: 20 of 40 peaks fall in the lift, 8 on
  the path, 12 in the descent or LAND; at 0.125 m/s every peak is the lift-off (path-only 0.86-1.11 deg). Bars 2 and 3
  give the same verdicts on path-only windows.
- **The thesis must not say:** "path-tracking RMSE" or "tilt during the carry" for these columns; any speed or shape
  trend read from the peak-tilt column; that the twin carries at 0.3 m/s on every shape; the 1 m arm "within 1.27x"
  without the window and LAND caveat; a plain "m/s" (write "peak path speed": circle and spin are one eased lap whose
  mean speed is speed/1.875, line_x's speed is a sinusoid's peak); for line_x, the "radius ratio" is an amplitude
  ratio about the line's centre.
- The thesis table should show the path-only columns (`payload_rmse_sweep_m` and a path-window tilt) beside the
  frozen ones, labelled as reported, not scored.

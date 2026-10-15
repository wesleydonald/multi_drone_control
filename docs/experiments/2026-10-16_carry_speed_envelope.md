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

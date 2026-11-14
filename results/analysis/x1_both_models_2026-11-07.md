# X1, the release ablation, on both quadrotor models (7 Nov 2026)

Wesley 7 Nov (decisions.md, feed F089): X1 is reported on both models side by side. This table is built from `results/analysis/x1_2026-11-06.json` (the **final**, hardware-matched masses: cable 0.002 kg + magnet 0.038 kg on a 0.510 kg airframe; R3821-R3906, feed F059 / F060) and `results/analysis/x1_oldlinks_2026-11-07.json` (the **earlier** split of before 4 Nov: 0.040 kg along the link + 0.035 kg magnet on 0.475 kg; R4150-R4193, feed F087 with F088's corrections). Each drone is 0.55 kg in both. Same code in src/ for both; only the worlds, models and tools differ. Results as is (F086): no bars. It replaces the draft's X1 table and paragraphs, results_discussion.tex lines 101-138 (they cite R3520-R3563 of 3-4 Nov).

## The measures (all on the ring's tilt, from run.csv, the release at t = 0)

- **Release rise**: the largest distance of the tilt vector from its mean over the 5 s before the release, from the release to LAND (deg; `tools/release_bands.py` dtheta). Where its peak is not the first swing, its time is given: at 8 quadrotors on the earlier masses the resize's peak is its second swing (+1.2 s) and the no-resize peak is a slow drift to a new level (+6.2 s, F088).
- **Second swing**: the largest absolute tilt from +1 to +4 s (deg). Not given for the square-to-T, whose first swing is still ending then (35-44 deg at +1.0 s).
- **Settled tilt**: the tilt of the mean tilt vector over the last 5 s before LAND (deg).
- **Last band exit**: the last time the tilt is outside its band around that settled tilt (max(2 sigma before, 0.5 deg), release_bands' BAND_MIN_DEG; corrected 7 Nov, the order holds at 1 deg too), in s after the release. ~17 s on the figure-8 means it never stays in the band before LAND (the path keeps moving it). F088: this, not the scorer's T6 (searched from the peak), is the settling reading.
- **Dip**: the ring's mean height over the 5 s before the release minus its lowest in the 10 s after (mm).

Each cell gives the two repeats as "a / b" (the final masses' 1.15 kg resize arm has one run: R3903 was void).

## Hover, 8 quadrotors (8 -> 7)

| Arm | Release rise, final | Release rise, earlier | Second swing, final | Second swing, earlier | Settled, final | Settled, earlier | Last band exit (s), final | Last band exit (s), earlier | Dip (mm), final | Dip (mm), earlier |
|---|---|---|---|---|---|---|---|---|---|---|
| resize at detection | 1.59 / 1.76 | 1.61 (+1.2 s) / 1.61 (+1.2 s) | 1.28 / 1.32 | 1.62 / 1.63 | 0.81 / 0.80 | 1.06 / 1.06 | 4.4 / 4.4 | 4.5 / 4.5 | 14.6 / 15.4 | 14.8 / 15.2 |
| resize 0.5 s after detection | 1.72 / 1.66 | 1.65 / 1.58 | 1.18 / 1.07 | 1.46 / 1.41 | 0.82 / 0.82 | 1.05 / 1.05 | 4.8 / 4.9 | 5.3 / 5.2 | 15.6 / 15.0 | 15.8 / 15.2 |
| resize 1 s after | 1.65 / 1.56 | 1.60 / 1.58 | 1.06 / 1.14 | 1.46 / 1.46 | 0.81 / 0.81 | 1.06 / 1.06 | 5.2 / 5.3 | 5.5 / 5.5 | 14.9 / 14.4 | 15.0 / 14.9 |
| resize 2 s after | 1.69 / 1.65 | 1.50 / 1.55 | 1.15 / 1.05 | 1.39 / 1.39 | 0.81 / 0.82 | 1.06 / 1.06 | 3.0 / 3.1 | 5.4 / 5.4 | 15.1 / 14.8 | 14.3 / 15.2 |
| no resize | 1.59 / 1.72 | 1.66 (+6.2 s) / 1.58 (+6.2 s) | 1.10 / 1.03 | 1.34 / 1.39 | 0.93 / 0.91 | 1.18 / 1.18 | 3.1 / 3.1 | 3.5 / 3.4 | 14.7 / 15.4 | 15.5 / 15.2 |

## Hover, 4 quadrotors (4 -> 3)

| Arm | Release rise, final | Release rise, earlier | Second swing, final | Second swing, earlier | Settled, final | Settled, earlier | Last band exit (s), final | Last band exit (s), earlier | Dip (mm), final | Dip (mm), earlier |
|---|---|---|---|---|---|---|---|---|---|---|
| resize at detection | 6.43 / 6.95 | 7.06 / 7.04 | 3.49 / 3.47 | 4.19 / 4.29 | 0.03 / 0.03 | 0.03 / 0.06 | 6.8 / 6.8 | 6.6 / 6.8 | 28.9 / 31.2 | 33.8 / 32.7 |
| resize 0.5 s after detection | 7.59 / 6.62 | 6.23 / 6.83 | 3.67 / 3.52 | 4.42 / 4.39 | 0.01 / 0.04 | 0.06 / 0.07 | 6.8 / 4.9 | 6.8 / 6.8 | 31.3 / 29.6 | 30.0 / 33.4 |
| resize 1 s after | 6.92 / 7.66 | 6.71 / 6.77 | 2.76 / 2.94 | 3.63 / 3.67 | 0.03 / 0.03 | 0.01 / 0.08 | 5.1 / 5.1 | 5.2 / 5.1 | 31.7 / 34.6 | 32.1 / 32.9 |
| resize 2 s after | 6.83 / 7.61 | 7.44 / 6.68 | 2.49 / 2.32 | 3.18 / 3.28 | 0.03 / 0.02 | 0.05 / 0.03 | 5.6 / 5.7 | 3.6 / 3.6 | 31.0 / 34.1 | 34.9 / 32.6 |
| no resize | 6.97 / 6.95 | 7.10 / 6.77 | 2.45 / 2.30 | 3.27 / 3.20 | 0.20 / 0.23 | 0.21 / 0.21 | 5.8 / 5.8 | 5.5 / 5.5 | 31.0 / 32.0 | 33.5 / 32.0 |

## Hover, 4 quadrotors, the 1.15 kg ring (4 -> 3)

| Arm | Release rise, final | Release rise, earlier | Second swing, final | Second swing, earlier | Settled, final | Settled, earlier | Last band exit (s), final | Last band exit (s), earlier | Dip (mm), final | Dip (mm), earlier |
|---|---|---|---|---|---|---|---|---|---|---|
| resize at detection | 10.55 | 9.66 / 9.76 | 3.25 | 3.70 / 3.74 | 0.05 | 0.06 / 0.06 | 4.6 | 5.4 / 5.5 | 46.8 | 44.7 / 45.8 |
| no resize | 10.13 / 10.45 | 9.74 / 9.46 | 2.73 / 2.75 | 3.02 / 3.06 | 0.22 / 0.23 | 0.22 / 0.20 | 3.4 / 4.4 | 4.4 / 4.2 | 47.9 / 48.9 | 48.3 / 44.9 |

## Figure-8, 4 quadrotors (4 -> 3)

| Arm | Release rise, final | Release rise, earlier | Second swing, final | Second swing, earlier | Settled, final | Settled, earlier | Last band exit (s), final | Last band exit (s), earlier | Dip (mm), final | Dip (mm), earlier |
|---|---|---|---|---|---|---|---|---|---|---|
| resize at detection | 7.15 / 7.05 | 6.95 / 7.32 | 3.37 / 3.49 | 4.33 / 4.49 | 2.50 / 2.55 | 3.64 / 3.66 | 17.0 / 17.0 | 17.0 / 17.0 | 29.9 / 28.4 | 30.8 / 31.3 |
| resize 0.5 s after detection | 7.51 / 7.05 | 6.85 / 7.21 | 3.66 / 3.55 | 4.56 / 4.58 | 2.55 / 2.55 | 3.59 / 3.62 | 17.0 / 17.0 | 17.0 / 17.0 | 31.5 / 30.6 | 31.9 / 32.0 |
| resize 1 s after | 7.08 / 7.53 | 7.12 / 6.72 | 3.42 / 3.51 | 4.34 / 4.26 | 2.48 / 2.55 | 3.59 / 3.70 | 17.0 / 17.0 | 17.0 / 17.0 | 29.5 / 31.9 | 33.2 / 31.2 |
| resize 2 s after | 6.74 / 7.07 | 6.75 / 7.00 | 3.29 / 3.33 | 4.29 / 4.30 | 2.58 / 2.62 | 3.63 / 3.64 | 17.0 / 17.0 | 17.0 / 17.0 | 27.3 / 29.0 | 30.1 / 31.0 |
| no resize | 7.12 / 7.54 | 6.83 / 7.01 | 3.38 / 3.49 | 4.26 / 4.21 | 2.60 / 2.63 | 3.65 / 3.62 | 17.0 / 17.0 | 17.0 / 17.0 | 29.4 / 31.5 | 31.1 / 30.4 |

## Square to T, 4 quadrotors, sideways integral off (int_k_xy 0)

| Arm | Release rise, final | Release rise, earlier | Second swing, final | Second swing, earlier | Settled, final | Settled, earlier | Last band exit (s), final | Last band exit (s), earlier | Dip (mm), final | Dip (mm), earlier |
|---|---|---|---|---|---|---|---|---|---|---|
| resize at detection | 61.61 (+0.7 s) / 59.18 (+0.7 s) | 58.12 (+0.7 s) / 56.92 (+0.7 s) | n/a | n/a | 1.74 / 1.76 | 0.63 / 0.59 | 13.8 / 14.3 | 14.1 / 14.3 | 101.9 / 96.6 | 99.3 / 97.5 |
| resize 0.5 s after detection | 57.64 (+0.7 s) / 57.93 (+0.7 s) | 55.28 (+0.7 s) / 56.30 (+0.7 s) | n/a | n/a | 1.80 / 1.77 | 0.63 / 0.64 | 14.3 / 14.3 | 14.5 / 7.2 | 106.6 / 103.6 | 104.1 / 108.2 |
| resize 1 s after | 61.44 (+0.7 s) / 62.02 (+0.7 s) | 56.63 (+0.7 s) / 57.41 (+0.7 s) | n/a | n/a | 1.77 / 1.69 | 0.65 / 0.60 | 6.8 / 14.3 | 14.4 / 14.2 | 108.6 / 108.3 | 107.8 / 107.7 |
| resize 2 s after | 57.81 (+0.7 s) / 62.70 (+0.7 s) | 57.31 (+0.7 s) / 55.96 (+0.7 s) | n/a | n/a | 1.64 / 1.67 | 0.56 / 0.59 | 14.4 / 14.2 | 14.1 / 14.4 | 103.9 / 111.8 | 106.0 / 107.7 |
| no resize | 58.16 (+0.7 s) / 62.27 (+0.7 s) | 57.74 (+0.7 s) / 57.19 (+0.7 s) | n/a | n/a | 1.78 / 1.80 | 2.20 / 2.21 | 14.6 / 14.6 | 15.2 / 15.2 | 106.0 / 109.3 | 107.1 / 108.0 |

## The solver switches (resize arms only)

| Cell | Final masses: switches, failed solves (whole run) | Earlier masses: switches | solves in the 1 s after, failed | first solve less converged than the last before | largest first-solve residual |
|---|---|---|---|---|---|
| h8 | 8, 0 | 8 | 80, 0 | 7 of 8 | 9.5e-05 |
| h4 | 8, 0 | 8 | 80, 0 | 6 of 8 | 2.3e-04 |
| h4p115 | 1, 0 | 2 | 20, 0 | 0 of 2 | 3.1e-04 |
| f4 | 8, 0 | 8 | 82, 0 | 8 of 8 | 5.9e-04 |
| sqTnoxy | 8, 0 | 8 | 80, 0 | 3 of 8 | 1.5e-03 |

On the earlier masses the residuals were read per switch (docs/analysis/switch_first_solve_1m_2026-10-31.md's method, results/analysis/x1_oldlinks_switches_2026-11-07.json; F087 / F088): no solve failed (0 of 342), but the first solve after a switch is less converged than the last before it in 24 of 34, and in the square-to-T resize arm the residual reached 1.6e-2 to 2.2e-2 at +0.1 s (status 0). On the final masses only the whole run's failed solves were counted (0 in every run); the per-switch residuals were not read.

## What differs between the two models (plain words)

- **Square to T** (the sideways integral off in both): on the earlier masses the resize arms leave the ring settled at 0.56-0.65 deg against 2.20-2.21 without a resize, and the resize at detection dips 97-99 mm against 107-108 (means -9.2 mm); on the final masses the resize arms settle at 1.64-1.80 against 1.78-1.80, and the resize at detection dips 96.6-101.9 mm against 106.0-109.3 (means -8.4 mm). The settled-tilt benefit is the earlier model's only; the smaller dip at detection shows on both (with the integral on, 30 Oct, on an earlier tree, ours settled at 8.3 / 10.5 deg against 2.2).
- **Hover**: on both models the resize leaves the ring more level (4 quadrotors: about 0.03-0.06 deg against 0.2; 8: about 0.8 against 0.9 on the final masses, 1.06 against 1.18 on the earlier), and on both it adds a larger second swing (about 2 s after the release with 4 quadrotors, +2.05-2.19 s; about 1.2-1.3 s with 8); it does not settle sooner: the last band exit comes later with the resize at detection at 4 and at 8 quadrotors on both models (the 2 s-late arm exits sooner than no resize at h4 on the earlier masses, 3.6 vs 5.5 s, and at h8 on the final, 3.0-3.1 vs 3.1).
- **Figure-8**: the arms settle the same on either model (2.5-2.6 deg final, 3.6-3.7 earlier); the resize's second swing is about 0.1-0.3 deg larger on the earlier masses and the same on the final ones.
- The release rise itself (the first swing) does not depend on the resize on either model, except that at h8 on the earlier masses the rise column holds the second swing (+1.2 s) and a drift (+6.2 s), not the first swing. The dip does not depend on it in hover; in the square-to-T the resize at detection dips 8-9 mm less on both models.
- (Reviewed 7 Nov, feed F094: these four statements corrected; all 348 cells match the jsons.)

Evidence: registry R3821-R3906, R4150-R4193, R4150c; feed F059, F060, F087, F088, F089.

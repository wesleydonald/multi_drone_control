### F1xx 2026-11-13 17:05 ANSWER Q009: X1's second swing in layout (e), read from the end of the first swing
- Section: 4.1.2 Losing Quadrotors in Simulation (tab:res_x1), the harder detachment of layout (e) (sqTnoxy,
  R3893-R3902, final masses).
- What was checked: the reading was recomputed independently from run.csv, with the tilt taken from the ring's
  quaternion.
  - t = 0 is the RELEASE event.
  - The first swing ends at its first minimum after the peak in [0, 1.5] s (the lowest sample within +-0.1 s).
  - The second swing is the largest tilt from there to +4.0 s.
  - The run folders are on the lab PC (drones:~/mdc_archive/results/2026-11-06/). The run.csv checksums match.
- **The thesis numbers stand.** All 10 runs and all 5 means reproduce (largest difference 0.005 deg):

  | Arm | Runs | First swing ends (s) | Tilt there (deg) | Second swing (deg) | at (s) | Mean (deg) |
  |---|---|---|---|---|---|---|
  | Baseline | R3895 / R3896 | 1.39 / 1.40 | 0.19 / 0.14 | 9.45 / 10.34 | 2.63 / 2.56 | 9.9 |
  | Proposed | R3893 / R3894 | 1.50 / 1.55 | 12.94 / 12.47 | 23.44 / 21.27 | 2.16 / 2.19 | 22.4 |
  | 0.5 s late | R3897 / R3898 | 1.46 / 1.58 | 0.13 / 0.08 | 14.60 / 14.11 | 2.70 / 2.56 | 14.4 |
  | 1 s late | R3899 / R3900 | 1.40 / 1.40 | 0.02 / 0.43 | 15.68 / 15.13 | 2.77 / 2.74 | 15.4 |
  | 2 s late | R3901 / R3902 | 1.41 / 1.42 | 0.30 / 0.34 | 10.31 / 10.50 | 2.73 / 2.70 | 10.4 |

  - The first swing ends at +1.39-1.58 s, so "+1.4-1.6 s" holds.
  - The second swing peaks at +2.16-2.19 s with the resize at detection and +2.56-2.63 s without. The comment's
    +2.2 / +2.6 s holds.
  - Ending the window 4 s after detection (+4.30-4.32 s) changes no value.
- The h4 and f4 claim holds: the reading equals F091's +1 to +4 s window in 20 of 20 runs.
  - Their first swing ends at +0.74-0.81 s.
  - The table's h4 and f4 second-swing column reproduces: 2.4/3.5/3.6/2.9/2.4 and 3.4/3.4/3.6/3.5/3.3.
- Two notes for the text:
  1. The "(35-44 deg at +1.0 s)" in the comment at results_discussion.tex line 138 covers both models. On these runs the
     tilt at +1.0 s is 41.3-44.1 deg. 35-42 deg is the earlier masses (R4180-R4189). Write 41-44 deg.
  2. With the resize at detection the ring does not return to level after the first swing. It comes back only to
     12.5-12.9 deg, on the same side, then tips further to 21-23 deg.
     - In the other four arms it passes through level, leans up to 8 deg the other way, and swings back.
     - Measured from the end of the first swing, the second swing rises 8.8-10.5 deg with the resize and 9.3-10.2 deg
       for the baseline.
     - So 22.4 against 9.9 is how far the ring tips again, not a larger oscillation. The text "added a larger second
       swing" is right as the largest tilt. It should not say the swing doubles.
- Text could say: in layout (e), with resizing at detection the ring did not return to level after the first swing
  (12.5-12.9 deg at its lowest). It tipped again to 22.4 deg on average, against 9.9 deg with the baseline.
- Side notes, not Q009:
  - h4's "2 s late no larger than the baseline" is 2.40 against 2.38 deg on the means, so "about the same" is exact.
  - On the figure-eight, three arms' column values fall at +3.4-3.6 s, not at the +2.2 s swing, and the baseline's tilt
    is still rising at +4 s. The arms differ by at most 0.3 deg.
- Evidence: results/analysis/q009_x1_second_swing/ (second_swing.py, per_run.csv: 30 runs with the end of the first
  swing, the signed tilt, the F091 window, the tilt at +1.0 s and each run.csv's sha256). An analysis check, no new
  runs and no registry rows. Not reviewed. Closes Q009.

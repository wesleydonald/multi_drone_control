# What changed when the transient table moved to the signed T6 metrics (11 Oct, docs/freeze_2026-10-14.md 1.1)

Old reading: the scalar tilt's peak over the 5 s after the event above its mean over the 3 s before. New: the
tilt-vector rise dtheta (max |theta(t) - mean_pre| over event..LAND, 5 s pre-window), computed by
tools/release_bands.py's Run class; twin rows also cut at the paired rig flight's hold (Q1).

| run | old scalar rise (deg) | T6 dtheta (deg) | change |
|---|---|---|---|
| twin, 8 Oct pairs (8 runs) | 4.05-17.42 | 4.00-17.39 | within 0.05 deg: the twin ring is level before the event (sigma_pre 0.08-0.26 deg), so vector and scalar readings coincide |
| r302_3 announced hover | 10.68 | 11.21 | +0.5 |
| r303_4 unannounced hover | 4.01 | 5.29 | +1.3 |
| r304_3 unannounced, 150 deg gap | 16.52 | 16.88 | +0.4 |
| r305 unannounced circle | 5.39 | 8.02 | +2.6: the rig ring rocks 2.3 deg (sigma_pre) around a near-level mean vector, so the excursion from the mean vector is nearly the absolute peak (8.09), where the scalar rise subtracted the rocking's mean |
| r0012 (7 Oct, not tabulated) | 2.41 | 6.07 | +3.7: the window runs to LAND (8 s) and the ring crossed its tilted pre-event mean |
| r0013 (7 Oct, not tabulated) | 87.68 | 91.27 | the capsize, planner log at 10 Hz |

- The rig/twin ratios on the new metric reproduce the freeze's factors exactly: 0.85 (r303_4), 1.05 (r304_3),
  1.70 (r305), 1.71 (r302_3).
- The gap scaling holds on T6: 120 deg gap 4.0-7.1 deg twin, 5.3-8.0 rig unannounced (11.2 announced);
  150 deg gap 14.6-17.4 twin, 16.9 rig.
- Cutting the twin rows at the rig hold (6.6-14.0 s) changes nothing: every twin peak comes within 1-2 s.
- No row is flagged: every reference was still for 5.3-17.5 s before the event (as the freeze's table).
- New columns: sigma_pre (twin 0.08-0.26, rig 1.0-2.3 deg); time back in band (twin hover 2.5-3.0 s; twin circle
  14-15 s, because the band is 0.5 deg about the final mean and the circle's tilt drifts with orbit phase; rig circle
  3.9 s; the rig hover flights land under 10 s after the event and are not scored).
- The dip now runs event..LAND against the 5 s pre-window mean (8 Oct rig 3.1-6.9 cm; twin 2.1-4.7 cm).
- The planned-step and static-step columns are unchanged.

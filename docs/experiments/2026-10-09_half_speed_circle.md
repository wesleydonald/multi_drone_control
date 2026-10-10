# Half-speed circle (the supervisor's test)

1. **Question** (supervisor, 8 Oct): if you fly the circle at half speed, does the error halve?
2. **Hypothesis.** The ring's lag behind the circle is a timing effect (the planner-to-tracker delay amplified
   by the OCP: twin R1165/R1166, R1172/R1173), so the along-the-path error scales with speed while the lag in
   seconds stays; the off-the-path and height errors are offsets and do not scale.
3. **The one variable.** traj_speed 0.125 -> 0.0625 m/s; everything else equal: the M1 layout 30/90/150/270,
   mode dissipative on the new defaults, configs/rig/traj_hold.yaml (5 s hold), r 0.5, ring start 0.5 m toward
   -y of the cage centre, the same day and pack set. Twin: w4_3915_orbit_0125 vs w4_3915_orbit_00625.
4. **Baseline.** The 0.125 run of the same session (rig W1; twin w4_3915_orbit_0125 x2 tonight).
5. **Pass / fail** (metrics from tools/rig_summary.py: along-the-path, off-the-path and height RMSE over the
   scored part; lag = lag_metrics A0). Supports if along-path(0.0625) / along-path(0.125) = 0.5 +/- 0.15 and the
   lag changes by less than 0.05 s. Falsified if the ratio is > 0.75 (the error does not scale with speed) or the
   lag changes by more than 0.1 s. Off-path and height are reported, with no bar.
6. **Repeats.** Twin 2 per speed. Rig 1 per speed (W1, W2).
7. **Cost.** Gazebo 4 runs (~50 min laptop). Rig 2 flights.
8. **New code?** No.

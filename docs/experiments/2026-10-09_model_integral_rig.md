# Model integral on the rig detach runs (Wednesday W4, W5)

1. **Question** (Wesley, 8 Oct): with the model integral on, does the 5-6 cm sideways offset of the rig detach
   runs close?
2. **Hypothesis.** Every dissipative detach run on 8 Oct flew the reference integral (the model integral was
   refused for a fleet that can detach) and sat 5.2-6.5 cm off its hover point; the model-integral carry r206e60
   sat 1.9 cm off. With the model integral (896aff2: kept through the resize, frozen in the reconfiguration
   hold) the pre-release hover offset drops to about 2 cm and r305's off-path error falls with it.
3. **The one variable.** int_mode reference -> model, on the 8 Oct rig configs unchanged:
   configs/rig/drop_and_land.yaml (W4, r303_4's line) and configs/rig/detach_unannounced_orbit_drop.yaml
   (W5, r305's line), layout 30/90/150/270, QUAD4 leaving from plate 9. The creep fix (6bfb881) changes only the
   takeoff path before the hover.
4. **Baseline.** RIG-1008-r303_4: hover offset 5.8 cm, tilt peak 6.5 deg, detected 0.42 s. RIG-1008-r305:
   hover offset 6.5 cm, off-path RMSE 5.4 cm (constant ~6 cm shift), 3-D 6.8 cm, tilt 8.1 deg. Twin: R1221 1.9 cm
   (model) and R1207/R1212 3.2 cm (reference) on the 1/3/5/9 hover.
5. **Pass / fail.** Supports if the hover offset before the release is <= 2.5 cm in both W4 and W5 and the tilt
   peak is no worse than the baseline + 5 deg. Falsified if the offset is >= 4.5 cm in both. W5: the xy
   integral learns only in the hover (planner_node gxy), so the integral value at the circle start is logged;
   if it is still moving more than 10 % per second at the circle start, W5's off-path result is inconclusive,
   not falsified. In between: grey, fly the repeat Thursday.
6. **Repeats.** Twin 2 per arm (R1221 + tonight's repeat; R1222 + tonight's repeat). Rig 1 each on Wednesday,
   the second on Thursday if grey.
7. **Cost.** Gazebo: 2 more runs tonight (laptop). Rig: 2 flights.
8. **New code?** No (896aff2, twin-checked R1217-R1222).

# Rig 30 Sep: rod-force balance in the tethered flights

Carried fraction = the sum over the four drones of the vertical force each puts into its rod, over the ring's weight
(0.86 kg * g). Per drone, from the RIG-0930-ladder4 law u = a_i + 0.507 M - 0.0219 (V - 23.5):
supported mass M = (u_real - a_i + 0.0219 (V - 23.5)) / 0.507, vertical rod force = M g R33 - 0.55 g
(R33 from the mocap quaternion; u_real = u2 on the old map, u2 + 0.185 + 0.022 (23.5 - V) on the affine map).
"Airborne" is ring z above its creep rest + 0.05 m. The accel-corrected column subtracts each drone's m a_z and
divides by 0.86 (g + a_ring), so the heave does not bias it.

Made with `python3 tools/thrust_fit.py --tethered results/rig/2026-09-30/{lift_f6,lift_f7,lift_f10,lift_f11,model_f1}_logs`.
Refitting the law from the 13 fixture points (`--spec tools/test/fixtures/rig_0930/ladder_points.yaml`) moves the
medians by under 0.005.

The sum does not depend on how the rod and magnet weight is split between drone and ring: in steady flight the
drones' total thrust must carry 4 * 0.55 + 0.86 kg. A median below 1.0 while airborne therefore means the ladder law
under-reads the thrust in tethered flight (or a mass is wrong), not that the ring is under-carried.

The reading is not constant: the flights held below 0.45 m read 0.78-0.87, f11 (median ring z 0.52) reads 0.99, and
inside every flight the fraction rises with ring height (by-height table). The D2 holds at 0.3 and 0.7 m separate
a height effect from a fixed law error.

Launch 155815 in `lift_f11_logs` is an earlier lift in the same folder that has no registry row of its own;
RIG-0930-lift-f11 is launch 160236.

## Per flight (ring airborne)

| flight | launch | airborne s | ring z med (m) | carried med | IQR | accel-corrected med | rod pull per drone med (N) |
|---|---|---|---|---|---|---|---|
| lift_f6 | 151732 | 4.1 | 0.354 | 0.816 | 0.691-0.916 | 0.850 | 2.02 / 1.25 / 1.74 / 1.80 |
| lift_f7 | 152654 | 18.5 | 0.271 | 0.812 | 0.745-0.885 | 0.825 | 1.72 / 1.88 / 1.57 / 1.69 |
| lift_f10 | 154156 | 19.5 | 0.185 | 0.843 | 0.755-0.907 | 0.836 | 1.87 / 2.00 / 1.66 / 1.54 |
| lift_f11 | 155815 | 17.1 | 0.415 | 0.867 | 0.754-0.907 | 0.870 | 2.00 / 1.95 / 1.71 / 1.54 |
| lift_f11 | 160236 | 31.1 | 0.523 | 0.989 | 0.842-1.155 | 0.982 | 2.00 / 2.05 / 2.21 / 2.20 |
| model_f1 | 172526 | 12.3 | 0.292 | 0.780 | 0.690-0.856 | 0.759 | 1.68 / 1.62 / 1.62 / 1.58 |

## By ring height

| flight | launch | ring z band (m) | samples | carried med | accel-corrected med |
|---|---|---|---|---|---|
| lift_f6 | 151732 | 0.10-0.20 | 36 | 0.845 | 0.786 |
| lift_f6 | 151732 | 0.20-0.30 | 40 | 0.606 | 0.666 |
| lift_f6 | 151732 | 0.30-0.40 | 42 | 1.085 | 1.061 |
| lift_f6 | 151732 | 0.40-0.50 | 35 | 0.813 | 0.971 |
| lift_f6 | 151732 | 0.50-0.60 | 51 | 0.819 | 0.972 |
| lift_f7 | 152654 | 0.10-0.20 | 235 | 0.723 | 0.729 |
| lift_f7 | 152654 | 0.20-0.30 | 441 | 0.879 | 0.861 |
| lift_f7 | 152654 | 0.30-0.40 | 252 | 0.807 | 0.846 |
| lift_f10 | 154156 | 0.10-0.20 | 608 | 0.842 | 0.815 |
| lift_f10 | 154156 | 0.20-0.30 | 233 | 0.813 | 0.853 |
| lift_f10 | 154156 | 0.30-0.40 | 137 | 0.910 | 0.924 |
| lift_f11 | 155815 | 0.10-0.20 | 190 | 0.696 | 0.696 |
| lift_f11 | 155815 | 0.20-0.30 | 40 | 0.728 | 0.796 |
| lift_f11 | 155815 | 0.30-0.40 | 163 | 0.875 | 0.870 |
| lift_f11 | 155815 | 0.40-0.50 | 263 | 0.883 | 0.891 |
| lift_f11 | 155815 | 0.50-0.60 | 203 | 0.897 | 0.962 |
| lift_f11 | 160236 | 0.10-0.20 | 154 | 0.684 | 0.707 |
| lift_f11 | 160236 | 0.20-0.30 | 100 | 0.778 | 0.807 |
| lift_f11 | 160236 | 0.30-0.40 | 264 | 0.841 | 0.836 |
| lift_f11 | 160236 | 0.40-0.50 | 197 | 0.955 | 0.933 |
| lift_f11 | 160236 | 0.50-0.60 | 146 | 0.957 | 0.938 |
| lift_f11 | 160236 | 0.60-0.80 | 300 | 1.109 | 1.109 |
| lift_f11 | 160236 | 0.80-up | 399 | 1.203 | 1.195 |
| model_f1 | 172526 | 0.10-0.20 | 123 | 0.666 | 0.609 |
| model_f1 | 172526 | 0.20-0.30 | 212 | 0.782 | 0.754 |
| model_f1 | 172526 | 0.30-0.40 | 280 | 0.814 | 0.848 |
